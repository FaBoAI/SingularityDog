"""Explicit FK STOP-only diagnostics; synthetic file-only fixtures, no devices."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import policy_live_profile as profiles
from singularitydog_hw import policy_active_fk as active
import test_active_fk_profile as profile_fixture


class ActiveFKDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.fixture=profile_fixture.ActiveFKProfileTests();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.data,self.docs,self.base=self.fixture.data,self.fixture.docs,self.fixture.base
        self.data.update(approved_for_supported_policy_output=False,blockers=['SYNTHETIC TEST NOT REVIEWED'],review=None)
        capture=self.docs['local_reference_capture']
        capture.update(approved_for_runtime=False,angle_wrap_applied=False,
            stop_state='UNVERIFIED_BY_READ_ONLY_PROTOCOL',motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT')
        for mid,row in capture['telemetry']['rows'].items():
            begin=int(mid)*1_000
            capture['identities'][mid].update(request_monotonic_ns=begin,reply_monotonic_ns=begin+1)
            row.update(current=0.,voltage=39.,position_span_deg=0.)
            row['position_samples']=[{**sample,'request_monotonic_ns':begin+2+i*2,
                'reply_monotonic_ns':begin+3+i*2} for i,sample in enumerate(row['position_samples'])]
        self.path=self.fixture.fixture.seal()
        self.digest=hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.uid=self.base/'uids.json'
        self.uid.write_text(json.dumps({mid:axis['uid'] for mid,axis in self.data['axes'].items()}))
        self.argv=['--active-fk-profile',str(self.path),'--active-fk-profile-sha256',self.digest,
            '--supported-disabled','--mode','stop-proxy','--v3-voltage-proxy','--v3-voltage-overlap',
            '--v3-voltage-validation-overlap','--v3-voltage-fast-pipeline',
            '--prepare-voltage-before-feedback-publication',
            '--provenance-mode',self.data['diagnostic_timing_acceptance'],'--power-epoch',self.data['motor_power_epoch'],
            '--request-gap-us',str(self.data['request_gap_us']),'--request-window',str(self.data['request_window']),
            '--h-hypothesis',str(int(self.data['h_hypothesis'])),'--cycles','5','--startup-cycle-allowance','1',
            '--record-storage','trace',
            '--calibration',str(self.base/'calibration.json'),'--mount',str(self.base/'mount.json'),
            '--gyro-bias',str(self.base/'bias.json'),'--bundle',str(self.base/self.data['bundle_path']),
            '--native-policy-manifest',str(self.base/'model_manifest.json'),
            '--native-policy-manifest-sha256',self.data['artifacts']['model_manifest']['sha256'],
            '--scalar-step-manifest',str(self.base/'scalar_step_manifest.json'),
            '--scalar-step-manifest-sha256',self.data['artifacts']['scalar_step_manifest']['sha256']]

    def main(self,argv):
        out=io.StringIO()
        with contextlib.redirect_stdout(out):value=bench.main(argv)
        return value,json.loads(out.getvalue())

    def test_plan_binds_full_profile_and_model_refs_without_torch_or_native_load(self):
        imported=set(sys.modules)
        with patch.object(bench.native,'load_library') as native,patch.object(active,'diagnostic_load') as model:
            result,plan=self.main(self.argv)
        self.assertEqual(result,0);native.assert_not_called();model.assert_not_called()
        self.assertFalse(any(name.startswith('torch') for name in set(sys.modules)-imported))
        self.assertTrue(plan['native_target_fk_cache'])
        self.assertEqual(plan['target_fk_manifest'],self.fixture.ref)
        self.assertEqual(plan['active_fk_profile']['sha256'],self.digest)
        self.assertFalse(plan['active_controller_qualification']);self.assertFalse(plan['enable_available'])
        self.assertFalse(plan['learned_targets_sent']);self.assertEqual(plan['type1_requests_per_cycle'],0)
        self.assertEqual(plan['requests_per_cycle'],26)
        self.assertEqual(plan['source_provenance']['cadence_source_sha256'],self.data['cadence_source_sha256'])

    def test_absent_flag_keeps_exact_unselected_plan_and_backend(self):
        with patch.object(active,'plan') as planner,patch.object(active,'diagnostic_load') as load:
            result,plan=self.main([])
        self.assertEqual(result,0);planner.assert_not_called();load.assert_not_called()
        self.assertNotIn('native_target_fk_cache',plan);self.assertNotIn('target_fk_manifest',plan)
        self.assertEqual(plan['policy_backend_requested'],'reference_bundle')

    def test_incomplete_pair_or_bad_profile_pin_rejected_before_any_load(self):
        for argv in (['--active-fk-profile',str(self.path)],['--active-fk-profile-sha256',self.digest],
            self.replace('--active-fk-profile-sha256','f'*64),self.replace('--active-fk-profile-sha256','bad')):
            with self.subTest(argv=argv),contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(bench.native,'load_library') as native,patch.object(active,'diagnostic_load') as model,\
                 self.assertRaises(SystemExit) as error:
                bench.main(argv)
            self.assertEqual(error.exception.code,2);native.assert_not_called();model.assert_not_called()

    def replace(self,key,value):
        argv=self.argv[:];argv[argv.index(key)+1]=value;return argv

    def test_args_cannot_change_profile_inputs_epoch_pacing_or_selected_pipeline(self):
        for key,value in (('--power-epoch','other'),('--provenance-mode',profiles.SUPPORTED_POLICY_PROBE),
                          ('--request-gap-us','1000'),('--request-window','1'),('--h-hypothesis','1'),
                          ('--calibration',str(self.path)),('--scalar-step-manifest-sha256','f'*64),
                          ('--native-policy-manifest-sha256','f'*64),('--bundle',str(self.base/'other'))):
            with self.subTest(key=key),contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(bench.native,'load_library') as native,self.assertRaises(SystemExit):
                bench.main(self.replace(key,value))
            native.assert_not_called()
        for flag in ('--supported-disabled','--v3-voltage-overlap','--v3-voltage-fast-pipeline',
                     '--v3-voltage-validation-overlap','--prepare-voltage-before-feedback-publication'):
            argv=self.argv[:];argv.remove(flag)
            with self.subTest(flag=flag),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):bench.main(argv)

    def test_no_other_experimental_or_acquisition_backend_can_select_fk(self):
        for extra in (['--acquisition-only'],['--compare-feedback'],['--retain-gil-trace-copy'],
                      ['--view-cache-manifest','other'],['--native-boot-guard-artifact','other']):
            with self.subTest(extra=extra),contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(bench.native,'load_library') as native,self.assertRaises(SystemExit):bench.main(self.argv+extra)
            native.assert_not_called()

    def execute_args(self,label):
        return self.argv+['--execute','--expected-uids',str(self.uid),'--library','SYNTHETIC NO LOAD',
                         '--front-port','SYNTHETIC FRONT','--rear-port','SYNTHETIC REAR',
                         '--output',str(self.base/label)]

    def test_failed_execute_preserves_selection_input_pin_and_no_model_or_devices(self):
        with patch.object(bench.native,'load_library',side_effect=ValueError('SYNTHETIC pre-device failure')),\
             patch.object(active,'diagnostic_load') as model:
            result,_=self.main(self.execute_args('early-failure'))
        model.assert_not_called();self.assertEqual(result,2)
        report=json.loads((self.base/'early-failure'/'report.json').read_text())
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['native_target_fk_cache'])
        self.assertEqual(report['input_sha256']['target_fk_manifest'],self.fixture.ref['sha256'])
        self.assertTrue(report['active_fk_profile_files_unchanged'])
        self.assertFalse(report['active_controller_qualification'])

    def test_selected_model_uses_raw_fk_proof_and_explicit_scalar_dependency(self):
        policy=object();raw=profile_fixture.provenance(profiles.load_profile(self.path,require_approved=False))
        torch=SimpleNamespace(set_num_threads=lambda x:None,set_num_interop_threads=lambda x:None)
        with patch.dict(sys.modules,{'torch':torch}),patch.object(bench.native,'load_library',return_value=object()),\
             patch.object(active,'diagnostic_load',return_value=(policy,raw)) as load,\
             patch.object(bench.observer,'StatefulPolicyObserver') as observer,\
             patch.object(bench.dual,'validate_ports',side_effect=ValueError('SYNTHETIC stop before ports')):
            result,_=self.main(self.execute_args('model-selected'))
        self.assertEqual(result,2);self.assertEqual(load.call_count,1)
        self.assertEqual(load.call_args.args[0]['bundle_path'],str((self.base/self.data['bundle_path']).absolute()))
        self.assertIs(observer.call_args.args[0],policy)
        report=json.loads((self.base/'model-selected'/'report.json').read_text())
        self.assertEqual(report['model_source'],raw)
        self.assertEqual(report['scalar_step_model_source'],raw['original_scalar_dependency'])
        self.assertEqual(report['model_source']['manifest_sha256'],self.fixture.ref['sha256'])
        self.assertFalse(report['model_source']['approved_for_runtime'])

    def test_current_boot_mismatch_aborts_before_serial_or_imu(self):
        policy=object();raw=profile_fixture.provenance(profiles.load_profile(self.path,require_approved=False))
        torch=SimpleNamespace(set_num_threads=lambda x:None,set_num_interop_threads=lambda x:None)
        guard=SimpleNamespace(boot_id='other-boot',close=lambda:None)
        with patch.dict(sys.modules,{'torch':torch}),patch.object(bench.native,'load_library',return_value=object()),\
             patch.object(active,'diagnostic_load',return_value=(policy,raw)),\
             patch.object(bench.observer,'StatefulPolicyObserver'),patch.object(bench.dual,'validate_ports',return_value={}),\
             patch.object(bench.dual.pipeline,'ownership_locks',return_value=contextlib.nullcontext()),\
             patch.object(bench.live,'imu_ownership_lock',return_value=contextlib.nullcontext()),\
             patch.object(bench.dual,'BootIdentityGuard',return_value=guard),\
             patch.object(bench,'collect') as collect:
            result,_=self.main(self.execute_args('wrong-boot'))
        collect.assert_not_called();self.assertEqual(result,2)
        report=json.loads((self.base/'wrong-boot'/'report.json').read_text())
        self.assertIn('current boot',report['errors'][0]);self.assertEqual(report['imu_restore_status'],'not_started')

    def test_cleanup_pin_mutation_marks_abort_and_preserves_primary_failure(self):
        data=profiles.load_profile(self.path,require_approved=False)
        context=dict(profile=data,reference=dict(path=str(self.path),sha256=self.digest),proof=data['_native_target_fk_cache_provenance'])
        report=dict(status='ABORTED',errors=['primary fake failure'])
        (self.base/'calibration.json').write_text('{}')
        bench._finish_active_fk_diagnostic(report,context)
        self.assertEqual(report['status'],'ABORTED');self.assertFalse(report['active_fk_profile_files_unchanged'])
        self.assertEqual(report['errors'][0],'primary fake failure');self.assertEqual(len(report['errors']),2)


if __name__=='__main__':unittest.main()
