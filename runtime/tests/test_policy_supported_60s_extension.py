"""Synthetic file-only60s admission: full original20/10/2 evidence, no robot."""
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_active_fk as fk
import test_policy_supported_20s_extension as twenty_fixture
import test_policy_supported_extension_profile as extension_fixture
import test_active_fk_profile as fk_fixture
from test_post_reply_input_age_extensions import wire_cycles


class SupportedSixtySecondExtensionTests(unittest.TestCase):
    def setUp(self):
        twenty_fixture.SupportedTwentySecondExtensionTests.setUp(self)
        twenty_path = twenty_fixture.SupportedTwentySecondExtensionTests.seal(self)
        live.load_profile(twenty_path)
        history = self.base/'history-20s'
        history.mkdir()
        for path in self.base.iterdir():
            if path.is_file():
                shutil.copyfile(path, history/path.name)
        self.twenty = copy.deepcopy(self.data)
        for ref in self.twenty['artifacts'].values():
            name = Path(ref['path'])
            if not name.is_absolute():
                ref['path'] = str(history/name.name)
        self.data.update(duration_s=60., assembly_id='SYNTHETIC sixty-second extension',
            diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_60S_AFTER_20S)
        self.docs['hardware_review']['assembly_id'] = self.data['assembly_id']
        report = copy.deepcopy(self.docs['prior_supported_report'])
        rows = []
        for index in range(993):
            row = copy.deepcopy(report['cycles'][1])
            begin = 30_000_000_000+index*20_000_000
            phase = 'starting' if index == 0 else 'stopped' if index == 992 else 'active'
            row.update(index=index, begin_ns=begin, end_ns=begin+19_000_000,
                phase=phase, effective_policy_weight=.005 if phase == 'active' else 0.)
            row['command'].update(phase=phase,
                kp=[self.twenty['axes'][mid]['kp'] if phase == 'active' else 0.
                    for mid in live.IDS],
                kd=[self.twenty['axes'][mid]['kd'] if phase == 'active' else 0.
                    for mid in live.IDS])
            rows.append(row)
        report.update(cycles=rows, actual_model_calls=968,
            execution_settings=live.execution_settings(self.twenty),
            cadence_source_sha256=copy.deepcopy(self.twenty['cadence_source_sha256']))
        self.docs.update(prior_supported_profile=self.twenty, prior_supported_report=report,
            prior_supported_observation=dict(user_statement='SYNTHETIC operator observed supported20s normally finish',
                observed_by='operator', audio_heard=True, abnormal_noise_vibration_slip_sinking_contact=False,
                box_support_maintained=True, autonomous_standing_or_walking_observed=False))
        self.extension = dict(mode=live.SUPPORTED_POLICY_PROBE_60S_AFTER_20S, scope=self.data['scope'],
            only_duration_extended=True, live_limits_unchanged=True, support_must_remain=True,
            load_bearing_not_established=True, walking_allowed=False,
            review={**self.data['review'], 'decision':'ACCEPT_60S_SUPPORTED_AFTER_20S'})
        self.docs['hardware_review']['supported_extension_acceptance'] = self.extension
        self.replay_wire()
        diagnostic = self.docs['pipeline_diagnostic']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'):
                    row[key] += 60_000_000_000
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += 60_000_000_000
        diagnostic.update(motor_power_epoch=self.data['motor_power_epoch'],
            cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']))

    def seal(self):
        return extension_fixture.SupportedExtensionProfileTests.seal(self)

    def load(self):
        return live.load_profile(self.seal())

    def replay_wire(self):
        # The reusable canonical wire generator normally builds V2; keep its
        # original physical records and replay them under this unchanged V1.
        with patch('test_post_reply_input_age_extensions.settings',
                   return_value=copy.deepcopy(self.twenty['post_reply_deadline_policy'])):
            wire_cycles(self.docs['prior_supported_report'], self.twenty, {})

    def sized_report(self, size, *, hardware_capture=False):
        """Preserve a real synthetic journal/chain and pad only JSON evidence."""
        self.seal()
        report = self.docs['prior_supported_report']
        report['synthetic_file_only_padding'] = ''
        encoded = json.dumps(report, sort_keys=True, allow_nan=False).encode()
        self.assertLessEqual(len(encoded), size)
        report['synthetic_file_only_padding'] = ' '*(size-len(encoded))
        self.seal()
        ref = copy.deepcopy(self.data['artifacts']['prior_supported_report'])
        self.assertEqual((self.base/ref['path']).stat().st_size, size)
        if hardware_capture:
            self.docs['hardware_review']['source_captures'].append(ref)
        return ref

    def test_full_sixty_second_loader_keeps_small_boxed_scope_and_real_wire_chain(self):
        loaded = self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertEqual(loaded['duration_s'], 60.)
        self.assertEqual(loaded['timing_review']['kind'], 'supported_policy_60s_after_20s_admission_only')
        self.assertEqual((loaded['policy_weight'], loaded['h_hypothesis'], loaded['hard_cycle_ms']),(.005,0.,20.))
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])
        self.assertEqual(live.post_reply_deadline_settings(loaded), self.twenty['post_reply_deadline_policy'])

    def test_large_original_twenty_report_and_same_hardware_capture_keep_full_chain(self):
        for size in (16*1024*1024+1, 32*1024*1024):
            with self.subTest(size=size):
                self.sized_report(size, hardware_capture=True)
                loaded = self.load()
                self.assertTrue(loaded['output_allowed'])
                self.assertEqual((loaded['duration_s'], loaded['hard_cycle_ms']), (60., 20.))
                self.assertEqual(loaded['artifacts']['prior_supported_report']['sha256'],
                                 self.data['artifacts']['prior_supported_report']['sha256'])
                self.docs['hardware_review']['source_captures'].pop()

    def test_large_twenty_report_still_rejects_over32mib_wrong_sha_and_wrong_cycle_shape(self):
        self.sized_report(32*1024*1024+1)
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            self.load()
        self.sized_report(16*1024*1024+1)
        path = self.seal()
        reference = self.data['artifacts']['prior_supported_report']
        (self.base/reference['path']).write_bytes((self.base/reference['path']).read_bytes()+b' ')
        with self.assertRaisesRegex(live.ProfileError, 'SHA256 mismatch'):
            live.load_profile(path)
        self.docs['prior_supported_report']['cycles'].extend(
            copy.deepcopy(self.docs['prior_supported_report']['cycles'][-1]) for _ in range(10))
        with self.assertRaisesRegex(live.ProfileError, 'completed ten-second learned cycles'):
            self.load()

    def test_large_hardware_capture_needs_same_report_path_and_sha(self):
        ref = self.sized_report(16*1024*1024+1)
        foreign = self.base/'same-bytes-but-unpinned-report.json'
        shutil.copyfile(self.base/ref['path'], foreign)
        captures = self.docs['hardware_review']['source_captures']
        captures.append(dict(path=foreign.name, sha256=ref['sha256']))
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            self.load()
        captures[-1] = dict(path=ref['path'], sha256='0'*64)
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            self.load()

    def test_sixty_mode_does_not_expand_other_artifacts_or_profile_itself(self):
        self.docs['prior_supported_observation']['synthetic_file_only_padding'] = ' '*(16*1024*1024)
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            self.load()
        self.docs['prior_supported_observation'].pop('synthetic_file_only_padding')
        path = self.seal()
        path.write_bytes(path.read_bytes()+b' '*(16*1024*1024))
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            live.load_profile(path)

    def test_large_outer_report_does_not_expand_nested_ten_second_report(self):
        self.sized_report(16*1024*1024+1)
        reference = self.twenty['artifacts']['prior_supported_report']
        path = Path(reference['path'])
        self.assertTrue(path.is_absolute())
        raw = path.read_bytes()
        raw += b' '*(16*1024*1024+1-len(raw))
        path.write_bytes(raw)
        reference['sha256'] = hashlib.sha256(raw).hexdigest()
        with self.assertRaisesRegex(live.ProfileError, 'JSON is too large'):
            self.load()

    def test_legacy_duration_modes_are_not_a_generic_sixty_second_unlock(self):
        original = copy.deepcopy(self.data)
        for duration in (59.999,60.001,120.,True):
            self.data=copy.deepcopy(original);self.data['duration_s']=duration
            with self.subTest(duration=duration),self.assertRaises(live.ProfileError):self.load()
        for mode in (live.SUPPORTED_POLICY_PROBE, live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                     live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S, live.CURRENT_HOLD_PROBE):
            self.data=copy.deepcopy(original);self.data['diagnostic_timing_acceptance']=mode
            with self.subTest(mode=mode),self.assertRaises(live.ProfileError):live._settings(self.data)

    def test_different_source_session_input_pose_or_motion_contract_fails_after_reseal(self):
        original=copy.deepcopy(self.data)
        changes=(lambda d:d.update(boot_id='00000000-0000-0000-0000-000000000001'),
            lambda d:d.update(motor_power_epoch='different'),lambda d:d.update(policy_weight=.004),
            lambda d:d.update(h_hypothesis=.1),lambda d:d.update(request_gap_us=880),
            lambda d:d['axes']['1'].update(kp=3.001),lambda d:d['axes']['1'].update(max_measured_torque_nm=1.001),
            lambda d:d['axes']['1'].update(physical_lower_rad=d['axes']['1']['physical_lower_rad']+.001),
            lambda d:d['cadence_source_sha256'].update({'singularitydog_hw/policy_active_fk.py':'f'*64}),
            lambda d:d['native_batch_encoder'].update(sha256='f'*64),
            lambda d:d['post_reply_deadline_policy'].update(max_misses_per_window=2))
        for change in changes:
            self.data=copy.deepcopy(original);change(self.data)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):self.load()

    def test_twenty_second_predecessor_must_be_complete_long_enough_learned_and_stopped(self):
        original=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r.update(status='ABORTED'),lambda r:r.update(errors=['fault']),
            lambda r:r.update(actual_model_calls=899),lambda r:r.update(cycles=r['cycles'][:974]),
            lambda r:r.update(normal_ramp_completed=False),lambda r:r.update(learned_targets_sent=False),
            lambda r:r.update(stop_confirmed=False),lambda r:r['stop_reports']['rear'].update(confirmed_ids=list(range(7,12))),
            lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1]),
            lambda r:r['cycles'][-1].update(phase='active'))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(original);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):self.load()

    def test_raw_wire_failure_cannot_be_hidden_by_success_summary(self):
        original=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r['journal'][0].update(error='cancelled'),lambda r:r['journal'].pop(),
            lambda r:r['journal'][1]['records'][0].update(received=0),
            lambda r:r['journal'][1]['records'][0].update(finish_ns=r['cycles'][0]['begin_ns']+20_000_001),
            lambda r:r['journal'][1]['records'][0].update(received_ns=r['cycles'][0]['begin_ns']+20_000_001),
            lambda r:r['cycles'][0]['post_reply_deadline'].update(rolling_misses=1),
            lambda r:r['cycles'][0].update(oldest_input_to_final_host_write_ms=0.))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(original);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):self.load()

    def test_single_existing_rare_miss_is_replayed_but_cluster_or_overbudget_fails(self):
        report=self.docs['prior_supported_report'];row=report['cycles'][161]
        row['end_ns']=row['begin_ns']+20_042_529
        for later in report['cycles'][162:]:
            later['begin_ns']+=50_000;later['end_ns']+=50_000
        self.replay_wire()
        self.assertEqual(report['post_reply_deadline_allowance_uses'],1)
        self.assertTrue(self.load()['output_allowed'])
        saved=copy.deepcopy(report)
        for index,late in ((162,20_001_000),(200,20_001_000),(400,21_000_001)):
            self.docs['prior_supported_report']=copy.deepcopy(saved)
            row=self.docs['prior_supported_report']['cycles'][index];row['end_ns']=row['begin_ns']+late
            with self.subTest(index=index),self.assertRaises((RuntimeError,live.ProfileError)):
                self.replay_wire();self.load()

    def test_vectors_limits_and_current_fresh_diagnostic_are_checked(self):
        original=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r['cycles'][60]['command'].update(kp=[3.]*11),
            lambda r:r['cycles'][60]['command']['q_model_rad'].__setitem__(0, 100.),
            lambda r:r['cycles'][60]['command']['estimated_pd_torque_nm'].__setitem__(0,.101),
            lambda r:r['cycles'][60]['feedback']['velocity_rad_s'].__setitem__(0,.351),
            lambda r:r['cycles'][60]['feedback']['torque_nm'].__setitem__(0,1.001))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(original);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):self.load()
        self.docs['prior_supported_report']=original
        self.docs['pipeline_diagnostic']['measurements'][0]['release_ns']=1
        with self.assertRaises(live.ProfileError):self.load()

    def test_actual_vectors_use_id_order_and_swapped_model_tensor_order_is_rejected(self):
        report=self.docs['prior_supported_report']
        actual=report['cycles'][60]['command']['q_model_rad']
        expected=[report['trial_origin_model_rad_by_id'][mid] for mid in live.IDS]
        self.assertEqual(actual,expected)
        swapped=[report['trial_origin_model_rad_by_id'][str(mid)] for mid in live.shadow.CAN_ORDER]
        self.assertNotEqual(actual,swapped)
        self.assertTrue(self.load()['output_allowed'])
        # Identical values merely reordered must not be tested against another
        # joint's physical interval or displacement origin.
        report['cycles'][60]['command']['q_model_rad']=swapped
        with self.assertRaisesRegex(live.ProfileError,'predecessor position ID|displacement ID'):
            self.load()

    def test_direct_supported_observation_and_explicit_named_sixty_review_required(self):
        original=copy.deepcopy(self.docs['prior_supported_observation'])
        for key,value in (('audio_heard',False),('observed_by','inferred'),('box_support_maintained',False),
                          ('autonomous_standing_or_walking_observed',True),('user_statement','')):
            self.docs['prior_supported_observation']=copy.deepcopy(original)
            self.docs['prior_supported_observation'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.load()
        self.docs['prior_supported_observation']=original
        self.extension['review']['decision']='ACCEPT_20S_SUPPORTED_AFTER_10S'
        with self.assertRaises(live.ProfileError):self.load()

    def test_entire_nested_twenty_ten_two_hash_graph_is_required(self):
        self.seal()
        old = Path(self.twenty['artifacts']['prior_supported_report']['path'])
        old.write_text(old.read_text()+' ')
        with self.assertRaisesRegex(live.ProfileError,'SHA256'):self.load()

    def test_admission_loader_only_source_change_remains_the_only_contract_exception(self):
        self.twenty['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py']='a'*64
        self.docs['prior_supported_report']['cadence_source_sha256']=copy.deepcopy(self.twenty['cadence_source_sha256'])
        # Retain an internally consistent historical review of the older
        # loader bytes; a stale settings hash is not a source-delta test.
        for name in ('operator_acceptance','hardware_review'):
            path=Path(self.twenty['artifacts'][name]['path'])
            document=json.loads(path.read_text())
            document['reviewed_settings_sha256']=live.reviewed_settings_sha256(self.twenty)
            excluded={'operator_acceptance','hardware_review'} if name=='operator_acceptance' else {'hardware_review'}
            document['artifact_sha256']={key:ref['sha256'] for key,ref in self.twenty['artifacts'].items()
                                        if key not in excluded}
            ref=fk_fixture._write(path,document);ref['path']=str(path)
            self.twenty['artifacts'][name]=ref
        self.assertTrue(self.load()['output_allowed'])


class PreparedV1TwentyAndFKSixtyScopeTests(unittest.TestCase):
    def setUp(self):
        fixture=twenty_fixture.SupportedTwentySecondExtensionTests()
        fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.data=copy.deepcopy(fixture.data)
        self.data.update(voltage_pipeline=True,prepare_voltage_before_feedback_publication=True)

    def test_v1_prepared_twenty_scope_is_admissible_but_does_not_create_loader_proof(self):
        live.execution_settings(self.data)
        with self.assertRaises(live.ProfileError):live.prepared_voltage_publication_settings(self.data)
        for key,value in (('duration_s',20.001),('hard_cycle_ms',20.001),('max_sample_age_ms',20.001),
                          ('policy_weight',.005001),('max_consecutive_20ms_misses',1)):
            candidate=copy.deepcopy(self.data);candidate[key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.execution_settings(candidate)

    def test_fk_sixty_selector_only_matches_exact_named_bounded_mode(self):
        self.data.update(native_target_fk_cache=True,duration_s=60.,
            diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_60S_AFTER_20S)
        self.assertTrue(fk.selected(self.data))
        live.execution_settings(self.data)
        for key,value in (('duration_s',60.001),('diagnostic_timing_acceptance',live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S),
                          ('policy_weight',.01),('scope','walk')):
            candidate=copy.deepcopy(self.data);candidate[key]=value
            with self.subTest(key=key),self.assertRaises((ValueError,live.ProfileError)):fk.selected(candidate)


class FKSixtySecondNestedChainTests(unittest.TestCase):
    """Authenticate a whole FK-selected graph with the ordinary V1 budget."""
    seal_graph=fk_fixture.ActiveFKDurationChainTests.seal_graph

    def setUp(self):
        fixture=SupportedSixtySecondExtensionTests()
        fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.base=fixture.base
        # Seal the old scalar graph before converting all historical stages.
        fixture.seal()
        self.fk=dict(SYNTHETIC_NOT_ROBOT_EVIDENCE=True,**dict.fromkeys(fk_fixture.FALSE_FLAGS,False))
        ref=fk_fixture._write(self.base/'target_fk_manifest.json',self.fk)
        ref['path']=str(self.base/'target_fk_manifest.json')
        self.shared={'target_fk_manifest':ref}
        for name in ('model_manifest','scalar_step_manifest'):
            self.shared[name]=copy.deepcopy(fixture.data['artifacts'][name])
            self.shared[name]['path']=str(self.base/Path(self.shared[name]['path']).name)
        self.enterContext(patch.object(fk,'plan',side_effect=lambda data,docs=None:fk_fixture.proof(data)))
        # Only the pre-existing disabled diagnostic wire converter is outside
        # this synthetic graph. Actual20s native Type1/IMU records are real toy
        # bytes and are independently replayed by the new admission.
        self.enterContext(patch.object(live,'_voltage_fast_pipeline_trace'))
        self.data,self.docs=self.convert(copy.deepcopy(fixture.data),copy.deepcopy(fixture.docs),'sixty')

    def convert(self,data,docs,label):
        directory=self.base/('fk-'+label);directory.mkdir()
        if 'prior_supported_profile' in docs:
            prior=docs['prior_supported_profile']
            ref=Path(data['artifacts']['prior_supported_profile']['path'])
            prior_base=(self.base/ref).parent if not ref.is_absolute() else ref.parent
            prior_docs={}
            for key,reference in prior['artifacts'].items():
                path=Path(reference['path']);path=path if path.is_absolute() else prior_base/path
                prior_docs[key]=json.loads(path.read_text())
            earlier={'sixty':'twenty','twenty':'ten','ten':'two'}[label]
            prior,prior_docs=self.convert(copy.deepcopy(prior),prior_docs,earlier)
            docs['prior_supported_profile']=prior
            actual=docs['prior_supported_report']
            actual.update(model_provenance=fk_fixture.provenance(prior),
                execution_settings=live.execution_settings(prior),
                cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']))
            for row in actual['cycles']:
                if 'command' in row:
                    for name in ('kp','kd'):
                        row['command'][name]=[prior['axes'][mid][name] if row['phase']=='active' else 0.
                                               for mid in live.IDS]
        for mid in live.IDS:
            # Legal but unequal limits expose accidental model-tensor order.
            data['axes'][mid].update(kp=1.+.1*int(mid),kd=.05+.005*int(mid),
                max_estimated_pd_torque_nm=.025+.002*int(mid))
        data['artifacts'].update(copy.deepcopy(self.shared))
        data['voltage_pipeline']=True
        data['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py']=live.cadence_source_hashes(data)[
            'singularitydog_hw/policy_live_profile.py']
        if not Path(data['bundle_path']).is_absolute():data['bundle_path']=str(self.base/data['bundle_path'])
        docs['target_fk_manifest']=copy.deepcopy(self.fk)
        diagnostic=docs['pipeline_diagnostic']
        diagnostic['plan'].update(v3_voltage_fast_pipeline=True,v3_voltage_pipeline=False)
        docs['hardware_review']['voltage_pipeline_acceptance']=dict(
            pipeline='feedback_then_voltage.fast_v1',scope=data['scope'],
            diagnostic_sha256='0'*64,
            hard_output_and_freshness_limits_unchanged=True,
            review={**data['review'],'decision':'ACCEPT_FEEDBACK_THEN_VOLTAGE'})
        fk_fixture.select(data,diagnostic,self.shared['target_fk_manifest'])
        self.seal_graph(data,docs,directory)
        return data,docs

    def load(self):
        self.seal_graph(self.data,self.docs,self.base/'fk-sixty')
        return live.load_profile(self.base/'fk-sixty'/'profile.json')

    def test_complete_v1_fk_sixty_graph_keeps_pins_box_and_timing_limits(self):
        loaded=self.load()
        self.assertEqual(live.native_target_fk_cache_settings(loaded),self.shared['target_fk_manifest'])
        self.assertEqual((loaded['duration_s'],loaded['policy_weight'],loaded['hard_cycle_ms']), (60.,.005,20.))
        self.assertEqual(live.post_reply_deadline_settings(loaded)['mode'],'bounded_post_reply_v1')
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        report=self.docs['prior_supported_report']
        report['model_provenance']['active_binding']['adapter_source_sha256']='f'*64
        with self.assertRaisesRegex(live.ProfileError,'encoder/model/backend/pacing differs'):self.load()

    def test_heterogeneous_gain_caps_follow_actual_id_order(self):
        loaded=self.load()
        report=self.docs['prior_supported_report']
        expected=[loaded['axes'][mid]['kp'] for mid in live.IDS]
        self.assertEqual(report['cycles'][60]['command']['kp'],expected)
        report['cycles'][60]['command']['kp']=[loaded['axes'][str(mid)]['kp'] for mid in live.shadow.CAN_ORDER]
        with self.assertRaisesRegex(live.ProfileError,'predecessor kp'):self.load()


if __name__=='__main__':unittest.main()
