"""Synthetic file-only active FK admission; never robot/approval evidence."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_active_fk as active
from singularitydog_hw import policy_live_profile as live
from test_policy_live_profile import _write
import test_prepared_voltage_publication_profile as prepared
import test_post_reply_input_age_extensions as extension


KEY = 'native_target_fk_cache'
FALSE_FLAGS = ('output_allowed', 'approved_for_runtime', 'active_controller_qualification',
               'timing_admission_eligible', 'live_50hz_verified')


def proof(data):
    refs = {key: copy.deepcopy(data['artifacts'][artifact]) for key, artifact in
            (('target_fk_manifest', 'target_fk_manifest'), ('scalar_manifest', 'scalar_step_manifest'),
             ('baseline_manifest', 'model_manifest'))}
    return dict(schema='singularitydog.active-fk-file-plan.v1', manifest_sha256=refs['target_fk_manifest']['sha256'],
        model_sha256='b'*64, library_sha256='c'*64, torch_or_native_loaded=False,
        baseline_provenance=dict(manifest_sha256=refs['baseline_manifest']['sha256']),
        active_binding=dict(schema='singularitydog.active-fk-binding.v1', native_target_fk_cache=True,
            model_backend=live.SCALAR_BACKEND, **refs,
            adapter_source_sha256=data['cadence_source_sha256']['singularitydog_hw/policy_active_fk.py'],
            model_artifact_grants_output=False), diagnostic_plan={'SYNTHETIC_NOT_ROBOT_EVIDENCE': True},
        **dict.fromkeys(FALSE_FLAGS, False))


def provenance(data):
    expected = proof(data)
    return dict(schema='singularitydog.fk-cache-stop-diagnostic-loader.v1',
        manifest_sha256=expected['manifest_sha256'], model_sha256=expected['model_sha256'],
        library_sha256=expected['library_sha256'], baseline_provenance=expected['baseline_provenance'],
        active_binding=expected['active_binding'],
        original_scalar_dependency=dict(manifest_sha256=data['artifacts']['scalar_step_manifest']['sha256']),
        diagnostic_only=True, hardware_opened=False, **dict.fromkeys(FALSE_FLAGS, False))


def select(data, diagnostic, fk_ref):
    data[KEY] = True
    data['artifacts']['target_fk_manifest'] = copy.deepcopy(fk_ref)
    for name in active.source_paths():
        data['cadence_source_sha256'][name] = hashlib.sha256(
            (Path(active.__file__).resolve().parents[1]/name).read_bytes()).hexdigest()
    diagnostic[KEY] = True
    diagnostic['plan'].update(native_target_fk_cache=True, target_fk_manifest=copy.deepcopy(fk_ref))
    diagnostic['input_sha256']['target_fk_manifest'] = fk_ref['sha256']
    diagnostic['model_source'] = provenance(data)
    diagnostic['cadence_source_sha256'] = copy.deepcopy(data['cadence_source_sha256'])
    diagnostic.setdefault('source_provenance', {}).update(source_files_unchanged=True,
        cadence_source_sha256=copy.deepcopy(data['cadence_source_sha256']),
        motor_power_epoch=data['motor_power_epoch'])


class ActiveFKProfileTests(unittest.TestCase):
    def setUp(self):
        self.fixture = prepared.PreparedVoltageProfileTests()
        self.fixture.setUp(); self.addCleanup(self.fixture.doCleanups)
        self.data, self.docs, self.base = self.fixture.data, self.fixture.docs, self.fixture.base
        self.fk = dict(SYNTHETIC_NOT_ROBOT_EVIDENCE=True, **dict.fromkeys(FALSE_FLAGS, False))
        self.docs['target_fk_manifest'] = self.fk
        self.ref = _write(self.base/'target_fk_manifest.json', self.fk)
        self.ref['path'] = str(self.base/'target_fk_manifest.json')
        for name in ('model_manifest', 'scalar_step_manifest'):
            self.data['artifacts'][name]['path'] = str(self.base/self.data['artifacts'][name]['path'])
        select(self.data, self.docs['pipeline_diagnostic'], self.ref)
        self.plan_mock = self.enterContext(patch.object(active, 'plan', side_effect=lambda data, docs=None: proof(data)))

    def load(self):
        return live.load_profile(self.fixture.seal())

    def test_complete_loader_binds_distinct_fk_scalar_baseline_and_keeps_unknowns(self):
        imported = set(sys.modules)
        parsed = self.load()
        self.assertEqual(live.native_target_fk_cache_settings(parsed), self.ref)
        self.assertEqual(parsed['_native_target_fk_cache_provenance'], proof(parsed))
        self.assertEqual(live.execution_settings(parsed)[KEY], True)
        self.assertIn('target_fk_manifest', live.artifact_names(parsed))
        self.assertIn('singularitydog_hw/policy_active_fk.py', live.cadence_source_paths(parsed))
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertTrue(parsed['support_must_remain'])
        self.assertIsNone(parsed['axes']['9']['uncertainty_rad'])
        self.assertFalse(any(name.startswith('torch') for name in set(sys.modules)-imported))
        self.assertEqual((parsed['period_ms'], parsed['hard_cycle_ms'], parsed['max_sample_age_ms']), (20.,20.,20.))

    def test_absent_false_have_exact_legacy_execution_and_no_extra_artifact_or_source(self):
        for choice in ('absent', False):
            data = live.template(schema=live.SCHEMA_V3)
            if choice is False: data[KEY] = False
            self.assertEqual(live.execution_settings(data), dict(model_backend='native_baseline',
                voltage_overlap=False, voltage_pipeline=False, diagnostic_timing_acceptance=None))
            self.assertIsNone(live.native_target_fk_cache_settings(data))
            self.assertNotIn('target_fk_manifest', live.artifact_names(data))
            self.assertNotIn('singularitydog_hw/policy_active_fk.py', live.cadence_source_paths(data))
        self.plan_mock.assert_not_called()

    def test_nonbool_and_selected_legacy_schema_rejected(self):
        for value in (None, 0, 1, '', {}, []):
            data = copy.deepcopy(self.data); data[KEY] = value
            with self.subTest(value=value), self.assertRaises(live.ProfileError): live.execution_settings(data)
        for schema in (live.SCHEMA_V1, live.SCHEMA_V2):
            data = copy.deepcopy(self.data); data['schema'] = schema
            with self.subTest(schema=schema), self.assertRaises(live.ProfileError): live.execution_settings(data)

    def test_diagnostic_source_selection_adds_only_current_adapter_to_legacy_set(self):
        selected=live.cadence_source_hashes({KEY:True})
        baseline=live.cadence_source_hashes()
        self.assertEqual({key:value for key,value in selected.items() if key in baseline},baseline)
        self.assertEqual(set(selected)-set(baseline),{'singularitydog_hw/policy_active_fk.py',
                                                    'singularitydog_hw/policy_output_model.py'})
        self.assertEqual(selected['singularitydog_hw/policy_active_fk.py'],hashlib.sha256(Path(active.__file__).read_bytes()).hexdigest())

    def test_boxed_scope_duration_selection_and_hard_limits_cannot_expand(self):
        changes = [('voltage_pipeline',False), ('voltage_overlap',False), ('model_backend','native_baseline'),
            ('scope','ground'), ('local_characterization',None), ('watchdog_review_policy',None),
            ('duration_s',2.001), ('policy_weight',0), ('policy_weight',.00501),
            ('period_ms',21), ('hard_cycle_ms',21), ('max_sample_age_ms',20.001),
            ('max_sample_gap_ms',21.001), ('max_consecutive_20ms_misses',1), ('command',[.01,0.,0.])]
        for key,value in changes:
            data=copy.deepcopy(self.data);data[key]=value
            with self.subTest(key=key), self.assertRaises(live.ProfileError):live.execution_settings(data)
        for mode in (live.CURRENT_HOLD_PROBE,live.CURRENT_HOLD_AFTER_SUPPORTED_10S,
                     live.FIXED_CATCH_CURRENT_HOLD_30S,live.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
                     live.SUPPORTED_PRELOAD_5S,live.SUPPORTED_POLICY_GAIN_STEP_3S,
                     live.SUPPORTED_POLICY_MIX_STEP_10PCT):
            data=copy.deepcopy(self.data);data['diagnostic_timing_acceptance']=mode
            with self.subTest(mode=mode),self.assertRaises(live.ProfileError):live.execution_settings(data)
        for key,value in (('kp',3.001),('kd',.15001),('max_displacement_from_start_rad',math.radians(1.001)),
                          ('max_estimated_pd_torque_nm',.10001),('max_measured_torque_nm',1.001),
                          ('max_command_velocity_rad_s',math.radians(1.001)),('max_temperature_c',45.01)):
            data=copy.deepcopy(self.data);data['axes']['9'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.execution_settings(data)

    def test_long_duration_scopes_keep_reviewed_v1_or_v2_and_exact_named_modes(self):
        data=copy.deepcopy(self.data)
        for duration,mode in ((10.,live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S),
                              (20.,live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S),
                              (60.,live.SUPPORTED_POLICY_PROBE_60S_AFTER_20S)):
            data.update(duration_s=duration,diagnostic_timing_acceptance=mode)
            self.assertIs(live.execution_settings(data)[KEY],True)
            reviewed_v1=copy.deepcopy(data['post_reply_deadline_policy'])
            data['post_reply_deadline_policy']=extension.settings()
            self.assertIs(live.execution_settings(data)[KEY],True)
            data['post_reply_deadline_policy']=reviewed_v1
            data['duration_s']=duration-.001
            with self.assertRaises(live.ProfileError):live.execution_settings(data)

    def test_unapproved_plan_verifies_file_bytes_but_never_mints_output_token(self):
        self.data.update(approved_for_supported_policy_output=False,blockers=['SYNTHETIC NOT REVIEWED'],review=None)
        path=self.base/'unapproved.json';path.write_text(json.dumps(self.data))
        parsed=live.load_profile(path,require_approved=False)
        self.assertFalse(parsed['output_allowed']);self.assertEqual(self.plan_mock.call_count,1)
        self.assertEqual(parsed['_native_target_fk_cache_provenance'],proof(parsed))
        with self.assertRaisesRegex(live.ProfileError,'complete loader proof'):live.native_target_fk_cache_settings(parsed)
        Path(self.ref['path']).write_text('{}')
        with self.assertRaisesRegex(live.ProfileError,'SHA256'):live.load_profile(path,require_approved=False)

    def test_missing_artifact_extra_artifact_and_source_pin_are_rejected(self):
        original=copy.deepcopy(self.data)
        for change in (lambda p:p['artifacts'].pop('target_fk_manifest'),
                       lambda p:p['cadence_source_sha256'].pop('singularitydog_hw/policy_active_fk.py'),
                       lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_active_fk.py':'f'*64})):
            self.data.clear();self.data.update(copy.deepcopy(original));change(self.data)
            path=self.base/'invalid.json';path.write_text(json.dumps(self.data))
            with self.assertRaises(live.ProfileError):live.load_profile(path)
        self.data.clear();self.data.update(copy.deepcopy(original));self.data[KEY]=False
        with self.assertRaises(live.ProfileError):live.load_profile(self.fixture.seal())

    def test_plan_cannot_promote_artifact_or_use_changed_adapter(self):
        for key,value in [('output_allowed',True),('torch_or_native_loaded',True),('model_sha256',None),
                          ('active_controller_qualification',True)]:
            bad=proof(self.data);bad[key]=value;self.plan_mock.side_effect=None;self.plan_mock.return_value=bad
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.load()
        bad=proof(self.data);bad['active_binding']['adapter_source_sha256']='f'*64
        self.plan_mock.return_value=bad
        with self.assertRaisesRegex(live.ProfileError,'adapter source'):self.load()

    def test_timing_both_selections_and_exact_fk_input_required(self):
        saved=copy.deepcopy(self.docs['pipeline_diagnostic'])
        for place in ('report','plan'):
            for value in ('absent',False,None,1):
                report=copy.deepcopy(saved);target=report if place=='report' else report['plan']
                if value=='absent':target.pop(KEY)
                else:target[KEY]=value
                self.docs['pipeline_diagnostic']=report
                with self.subTest(place=place,value=value),self.assertRaisesRegex(live.ProfileError,'FK selection'):self.load()
        self.docs['pipeline_diagnostic']=copy.deepcopy(saved)
        self.docs['pipeline_diagnostic']['input_sha256']['target_fk_manifest']='f'*64
        with self.assertRaisesRegex(live.ProfileError,'FK input'):self.load()
        self.docs['pipeline_diagnostic']=copy.deepcopy(saved)
        self.docs['pipeline_diagnostic']['plan']['target_fk_manifest']['path']+='.other'
        with self.assertRaisesRegex(live.ProfileError,'FK input'):self.load()

    def test_fk_scalar_baseline_library_model_source_and_flags_are_distinct(self):
        saved=copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes=(lambda p:p.update(manifest_sha256=self.data['artifacts']['scalar_step_manifest']['sha256']),
            lambda p:p.update(model_sha256='f'*64),lambda p:p.update(library_sha256='f'*64),
            lambda p:p['original_scalar_dependency'].update(manifest_sha256='f'*64),
            lambda p:p['baseline_provenance'].update(manifest_sha256='f'*64),
            lambda p:p['active_binding']['target_fk_manifest'].update(sha256='f'*64),
            lambda p:p['active_binding'].update(adapter_source_sha256='f'*64),
            lambda p:p.update(approved_for_runtime=True))
        for change in changes:
            self.docs['pipeline_diagnostic']=copy.deepcopy(saved);change(self.docs['pipeline_diagnostic']['model_source'])
            with self.subTest(change=change),self.assertRaisesRegex(live.ProfileError,'FK model'):self.load()

    def test_loaded_selection_refs_proof_source_and_axes_are_immutable(self):
        for change in (lambda p:p['artifacts']['target_fk_manifest'].update(sha256='f'*64),
            lambda p:p['artifacts']['scalar_step_manifest'].update(sha256='f'*64),
            lambda p:p['artifacts']['model_manifest'].update(sha256='f'*64),
            lambda p:p['_native_target_fk_cache_provenance'].update(model_sha256='f'*64),
            lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_active_fk.py':'f'*64}),
            lambda p:p['axes']['9'].update(offset_rad=.123),lambda p:p.update(motor_power_epoch='other')):
            parsed=self.load();change(parsed)
            with self.subTest(change=change),self.assertRaisesRegex(live.ProfileError,'complete loader proof'):
                live.native_target_fk_cache_settings(parsed)
        for remove in (False,True):
            parsed=self.load()
            if remove:parsed.pop(KEY)
            else:parsed[KEY]=False
            with self.assertRaisesRegex(live.ProfileError,'changed after loading'):live.native_target_fk_cache_settings(parsed)
        parsed=self.load();returned=live.native_target_fk_cache_settings(parsed);returned['path']='other'
        self.assertEqual(live.native_target_fk_cache_settings(parsed),self.ref)


class ActiveFKDurationChainTests(unittest.TestCase):
    """Use the existing canonical toy wire journals, not summary-only approvals."""
    def setUp(self):
        self.fixture=extension.V2TwentySecondTests()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.base=self.fixture.base
        self.fk=dict(SYNTHETIC_NOT_ROBOT_EVIDENCE=True,**dict.fromkeys(FALSE_FLAGS,False))
        ref=_write(self.base/'target_fk_manifest.json',self.fk)
        ref['path']=str(self.base/'target_fk_manifest.json')
        self.shared={'target_fk_manifest':ref}
        for name in ('model_manifest','scalar_step_manifest'):
            self.shared[name]=copy.deepcopy(self.fixture.data['artifacts'][name])
            self.shared[name]['path']=str(self.base/Path(self.shared[name]['path']).name)
        self.enterContext(patch.object(active,'plan',side_effect=lambda data,docs=None:proof(data)))
        self.data,self.docs=self.convert(copy.deepcopy(self.fixture.data),copy.deepcopy(self.fixture.docs),'twenty')

    def convert(self,data,docs,label):
        directory=self.base/('fk-'+label);directory.mkdir()
        if 'prior_supported_profile' in docs:
            prior=docs['prior_supported_profile']
            prior_base=Path(data['artifacts']['prior_supported_profile']['path'])
            prior_base=(self.base/prior_base).parent if not prior_base.is_absolute() else prior_base.parent
            prior_docs={}
            for key,ref in prior['artifacts'].items():
                path=Path(ref['path']);path=path if path.is_absolute() else prior_base/path
                prior_docs[key]=json.loads(path.read_text())
            prior,prior_docs=self.convert(copy.deepcopy(prior),prior_docs,'ten' if label=='twenty' else 'two')
            docs['prior_supported_profile']=prior
            actual=docs['prior_supported_report']
            actual.update(model_provenance=provenance(prior),execution_settings=live.execution_settings(prior),
                          cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']))
            if 'prepared_voltage_publication' in actual:
                actual['prepared_voltage_publication']['cadence_source_sha256']=copy.deepcopy(prior['cadence_source_sha256'])
        data['artifacts'].update(copy.deepcopy(self.shared))
        if not Path(data['bundle_path']).is_absolute():data['bundle_path']=str(self.base/data['bundle_path'])
        docs['target_fk_manifest']=copy.deepcopy(self.fk)
        select(data,docs['pipeline_diagnostic'],self.shared['target_fk_manifest'])
        self.seal_graph(data,docs,directory)
        return data,docs

    def seal_graph(self,data,docs,directory):
        # Immutable input/model paths stay shared through every stage. Only the
        # synthetic review/evidence graph is rewritten, in dependency order.
        names=['pipeline_diagnostic','prior_supported_profile','prior_supported_report','prior_supported_observation']
        for name in names:
            if name not in docs:continue
            if name=='prior_supported_report':docs[name]['profile_sha256']=data['artifacts']['prior_supported_profile']['sha256']
            if name=='prior_supported_observation':docs[name]['report_sha256']=data['artifacts']['prior_supported_report']['sha256']
            ref=_write(directory/(name+'.json'),docs[name]);ref['path']=str(directory/(name+'.json'))
            data['artifacts'][name]=ref
        # Resolve retained old fixture refs to their original existing bytes.
        for key,ref in data['artifacts'].items():
            if not Path(ref['path']).is_absolute():ref['path']=str(self.base/ref['path'])
        for name in ('operator_acceptance','hardware_review'):
            doc=docs[name]
            def absolute_refs(value):
                if type(value) is dict:
                    if set(value)=={'path','sha256'} and not Path(value['path']).is_absolute():
                        value['path']=str(self.base/value['path'])
                    else:
                        for item in value.values():absolute_refs(item)
                elif type(value) is list:
                    for item in value:absolute_refs(item)
            absolute_refs(doc)
            doc['reviewed_settings_sha256']=live.reviewed_settings_sha256(data)
            excluded={'operator_acceptance','hardware_review'} if name=='operator_acceptance' else {'hardware_review'}
            doc['artifact_sha256']={key:ref['sha256'] for key,ref in data['artifacts'].items() if key not in excluded}
            if name=='hardware_review':
                for row in doc.values():
                    if type(row) is dict and 'diagnostic_sha256' in row:
                        row['diagnostic_sha256']=data['artifacts']['pipeline_diagnostic']['sha256']
                acceptance=doc.get('supported_extension_acceptance')
                if acceptance and 'prior_supported_profile' in data['artifacts']:
                    for field,key in (('prior_profile_sha256','prior_supported_profile'),
                                      ('prior_report_sha256','prior_supported_report'),
                                      ('prior_observation_sha256','prior_supported_observation')):
                        acceptance[field]=data['artifacts'][key]['sha256']
            ref=_write(directory/(name+'.json'),doc);ref['path']=str(directory/(name+'.json'))
            data['artifacts'][name]=ref
        _write(directory/'profile.json',data)

    def load(self):
        self.seal_graph(self.data,self.docs,self.base/'fk-twenty')
        return live.load_profile(self.base/'fk-twenty'/'profile.json')

    def test_full_nested_v2_fk_twenty_second_chain_preserves_all_live_limits(self):
        parsed=self.load()
        self.assertEqual(live.native_target_fk_cache_settings(parsed),self.shared['target_fk_manifest'])
        self.assertEqual(parsed['duration_s'],20.)
        self.assertEqual(parsed['policy_weight'],.005)
        self.assertEqual((parsed['hard_cycle_ms'],parsed['max_sample_age_ms']),(20.,20.))
        self.assertTrue(parsed['support_must_remain']);self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertFalse(parsed['_accel_input_hypothesis_provenance']['formal_calibration_approved'])

    def test_ten_second_stage_itself_qualifies_only_after_selected_fk_two_seconds(self):
        ten=self.docs['prior_supported_profile']
        parsed=live.load_profile(Path(ten['artifacts']['hardware_review']['path']).parent/'profile.json')
        self.assertEqual(parsed['duration_s'],10.)
        self.assertEqual(live.native_target_fk_cache_settings(parsed),self.shared['target_fk_manifest'])
        self.assertTrue(live.execution_settings(parsed)[KEY])

    def test_twenty_refuses_ordinary_scalar_or_changed_fk_actual_predecessor(self):
        original=copy.deepcopy(self.docs['prior_supported_report'])
        for change in (lambda r:r['execution_settings'].pop(KEY),
            lambda r:r['model_provenance'].update(manifest_sha256=self.shared['scalar_step_manifest']['sha256']),
            lambda r:r['model_provenance']['active_binding']['scalar_manifest'].update(sha256='f'*64),
            lambda r:r['model_provenance'].update(library_sha256='f'*64)):
            self.docs['prior_supported_report']=copy.deepcopy(original);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):self.load()

    def test_nested_two_second_fk_proof_and_selection_cannot_be_replaced(self):
        ten=self.docs['prior_supported_profile']
        report_path=Path(ten['artifacts']['prior_supported_report']['path'])
        report=json.loads(report_path.read_text());report['model_provenance']['original_scalar_dependency']['manifest_sha256']='f'*64
        # Reseal outer hashes as a synthetic named review would: the actual
        # provenance mismatch must still fail, rather than just a stale hash.
        ref=_write(report_path,report);ref['path']=str(report_path);ten['artifacts']['prior_supported_report']=ref
        ten_docs={key:json.loads(Path(value['path']).read_text()) for key,value in ten['artifacts'].items()}
        self.seal_graph(ten,ten_docs,report_path.parent)
        with self.assertRaisesRegex(live.ProfileError,'same actual FK predecessor'):self.load()


if __name__=='__main__':unittest.main()
