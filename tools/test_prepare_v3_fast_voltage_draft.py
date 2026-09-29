"""File-only fast-voltage review draft; no serial, I2C, model or motor access."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prepare_v3_fast_voltage_draft as draft_tool

live = draft_tool.live


def save(path, value):
    path.write_text(json.dumps(value, sort_keys=True)+'\n')
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


class FastVoltageDraftTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.base=live.template(schema=live.SCHEMA_V3)
        self.base['duration_s']=3.
        self.base['startup_duration_s']=.5
        self.base['policy_ramp_s']=.5
        self.base['stop_duration_s']=.5
        self.base['boot_id']='00000000-0000-0000-0000-000000000001'
        self.base['motor_power_epoch']='synthetic-epoch'
        self.base['blockers'].append('missing_measurements:full_pipeline_diagnostic_not_eligible')
        for name in live.ARTIFACTS:
            value={'schema':live.REVIEW_SCHEMA,'review':None} if name=='hardware_review' else {'source':name}
            self.base['artifacts'][name]=save(self.root/(name+'.json'),value)
        self.base_path=self.root/'base-profile.json'
        save(self.base_path,self.base)
        self.report_path=self.root/'new-report.json'
        save(self.report_path,{'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                               'boot_id':self.base['boot_id'],
                               'v3_voltage_fast_pipeline':{'records_sha256':'a'*64}})
        baseline=self.base['artifacts']['model_manifest']['sha256']
        self.scalar_path=self.root/'scalar.json'
        save(self.scalar_path,{'schema':'native-step-scalar-file-only-v1',
            'status':'PASS_FILE_ONLY_COMPARE','baseline_manifest_sha256':baseline,
            'hardware_opened':False,'output_allowed':False,
            'approved_for_runtime':False,'live_50hz_verified':False})
        self.output=self.root/'new-private-draft'

    def prepare(self):
        return draft_tool.prepare(base_profile=self.base_path,
            pipeline_diagnostic=self.report_path,
            scalar_step_manifest=self.scalar_path,output=self.output)

    def test_new_draft_binds_exact_inputs_and_leaves_acceptance_pending(self):
        source_bytes={path:path.read_bytes() for path in self.root.iterdir() if path.is_file()}
        with patch.object(live,'_timing',return_value={'kind':'stop_proxy_diagnostic_only',
                                                     'cycles':501}) as timing:
            result=self.prepare()
        self.assertEqual(result['status'],'UNAPPROVED_REVIEW_DRAFT')
        self.assertFalse(result['output_allowed']);self.assertFalse(result['hardware_opened'])
        timing.assert_called_once()
        profile=json.loads((self.output/'profile.json').read_text())
        review=json.loads((self.output/'hardware-review.json').read_text())
        audit=json.loads((self.output/'draft-audit.json').read_text())
        self.assertEqual(profile['schema'],live.SCHEMA_V3)
        self.assertEqual((profile['model_backend'],profile['voltage_overlap'],
                          profile['voltage_pipeline'],profile['diagnostic_timing_acceptance']),
                         (live.SCALAR_BACKEND,True,True,live.MEASURED_R17_STARTUP_TIMING))
        self.assertIsNone(profile['review']);self.assertFalse(profile['approved_for_supported_policy_output'])
        self.assertIsNone(review['review'])
        self.assertIsNone(review['voltage_pipeline_acceptance']['review'])
        self.assertIsNone(review['voltage_pipeline_acceptance']['hard_output_and_freshness_limits_unchanged'])
        self.assertEqual(review['voltage_pipeline_acceptance']['diagnostic_sha256'],
                         profile['artifacts']['pipeline_diagnostic']['sha256'])
        self.assertEqual(review['reviewed_settings_sha256'],live.reviewed_settings_sha256(profile))
        self.assertEqual(review['artifact_sha256'],{name:profile['artifacts'][name]['sha256']
            for name in live.artifact_names(profile) if name!='hardware_review'})
        self.assertNotIn('missing_measurements:full_pipeline_diagnostic_not_eligible',profile['blockers'])
        self.assertIn('named_review:v3_fast_voltage_pipeline_acceptance_pending',profile['blockers'])
        self.assertFalse(audit['actual_policy_output_20ms_verified'])
        self.assertEqual(audit['fast_records_sha256'],'a'*64)
        self.assertFalse(live.load_profile(self.output/'profile.json',require_approved=False)['output_allowed'])
        for path,raw in source_bytes.items():self.assertEqual(path.read_bytes(),raw)
        self.assertEqual((self.output).stat().st_mode & 0o777,0o700)
        for path in self.output.iterdir():self.assertEqual(path.stat().st_mode & 0o777,0o600)

    def test_ineligible_or_incomplete_diagnostic_does_not_publish(self):
        with self.assertRaisesRegex(ValueError,'Full real-input/inference/STOP diagnostic required'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_missing_unapproved_review_file_gets_blank_skeleton_and_rebased_sources(self):
        for name,reference in self.base['artifacts'].items():
            reference['path']=Path(reference['path']).name
        self.base['artifacts']['hardware_review']={'path':None,'sha256':None}
        save(self.base_path,self.base)
        with patch.object(live,'_timing',return_value={'kind':'stop_proxy_diagnostic_only',
                                                     'cycles':501}):
            self.prepare()
        profile=json.loads((self.output/'profile.json').read_text())
        review=json.loads((self.output/'hardware-review.json').read_text())
        audit=json.loads((self.output/'draft-audit.json').read_text())
        self.assertTrue(audit['generated_hardware_review_skeleton'])
        self.assertIsNone(review['angles']['1']['zero_and_sign_physically_verified'])
        self.assertIsNone(review['device_watchdog']['1']['actual_command_loss_test_passed'])
        self.assertIsNone(review['imu']['gravity_direction_verified'])
        for name,reference in profile['artifacts'].items():
            self.assertTrue(Path(reference['path']).is_absolute(),name)
        self.assertTrue(Path(review['source_captures'][0]['path']).is_absolute())

    def test_invalid_scalar_or_reviewed_source_cannot_be_rebound(self):
        scalar=json.loads(self.scalar_path.read_text())
        scalar['output_allowed']=True;save(self.scalar_path,scalar)
        with self.assertRaisesRegex(ValueError,'unapproved scalar-step'):
            self.prepare()
        self.assertFalse(self.output.exists())
        scalar['output_allowed']=False;save(self.scalar_path,scalar)
        hardware=json.loads(Path(self.base['artifacts']['hardware_review']['path']).read_text())
        hardware['review']={'decision':'APPROVED_SUPPORTED_CHARACTERIZATION'}
        save(Path(self.base['artifacts']['hardware_review']['path']),hardware)
        with self.assertRaisesRegex(ValueError,'Artifact SHA256 mismatch'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_mismatched_boot_fails_and_stale_unapproved_pins_require_new_review(self):
        report=json.loads(self.report_path.read_text())
        report['boot_id']='00000000-0000-0000-0000-000000000002'
        save(self.report_path,report)
        with self.assertRaisesRegex(ValueError,'same current boot'):
            self.prepare()
        self.assertFalse(self.output.exists())
        report['boot_id']=self.base['boot_id'];save(self.report_path,report)
        first=live.CADENCE_SOURCE_PATHS[0]
        self.base['cadence_source_sha256'][first]='0'*64
        omitted=live.CADENCE_SOURCE_PATHS[-1]
        self.base['cadence_source_sha256'].pop(omitted)
        save(self.base_path,self.base)
        with patch.object(live,'_timing',return_value={'kind':'stop_proxy_diagnostic_only','cycles':501}):
            self.prepare()
        profile=json.loads((self.output/'profile.json').read_text())
        audit=json.loads((self.output/'draft-audit.json').read_text())
        self.assertEqual(profile['cadence_source_sha256'],live.cadence_source_hashes())
        self.assertIn(first,audit['cadence_sources_requiring_review'])
        self.assertIn(omitted,audit['cadence_sources_requiring_review'])
        self.assertIn('named_review:v3_fast_voltage_updated_source_pins_pending',profile['blockers'])
        self.assertFalse(profile['approved_for_supported_policy_output'])

    def test_never_overwrites_an_existing_draft(self):
        self.output.mkdir()
        marker=self.output/'keep';marker.write_text('original')
        with self.assertRaises(FileExistsError):self.prepare()
        self.assertEqual(marker.read_text(),'original')


if __name__=='__main__':unittest.main()
