"""Synthetic file-only 2s -> 10s -> 20s contracts; no physical trial evidence."""
import copy
import json
import math
from pathlib import Path
import shutil
import unittest

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
from test_policy_local_profile import seal_local
import test_policy_supported_extension_profile as extension_fixture


class SupportedTwentySecondExtensionTests(unittest.TestCase):
    def setUp(self):
        extension_fixture.SupportedExtensionProfileTests.setUp(self)
        ten_path = extension_fixture.SupportedExtensionProfileTests.seal(self)
        profile.load_profile(ten_path)
        # Freeze the *whole* prior ten-second graph under a separate directory.
        # The new outer prior_* files must never overwrite its two-second files.
        self.history = self.base/'history-10s'
        self.history.mkdir()
        for path in self.base.iterdir():
            if path.is_file():
                shutil.copyfile(path, self.history/path.name)
        ten = copy.deepcopy(self.data)
        for reference in ten['artifacts'].values():
            reference['path'] = str(self.history/Path(reference['path']).name)
        self.ten = ten
        self.data.update(duration_s=20., assembly_id='SYNTHETIC twenty-second extension',
            diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)
        self.docs['hardware_review']['assembly_id'] = self.data['assembly_id']
        self.rows = []
        origins = {}
        for mid in profile.IDS:
            raw = self.docs['local_reference_capture']['telemetry']['rows'][mid]['median_position_rad']
            axis = ten['axes'][mid]
            origins[mid] = axis['sign']*raw+axis['offset_rad']
        positions = [origins[str(mid)] for mid in profile.shadow.CAN_ORDER]
        for index in range(493):
            begin = 2_000_000_000+index*20_000_000
            end = begin+19_000_000
            phase = 'starting' if index == 0 else 'stopped' if index == 492 else 'active'
            gain = 0. if phase in ('starting', 'stopped') else 1.
            self.rows.append(dict(index=index, begin_ns=begin, output_reply_end_ns=begin+18_000_000,
                end_ns=end, phase=phase, effective_policy_weight=.005 if phase == 'active' else 0.,
                oldest_input_to_final_host_write_ms=18., deadline20ms_missed=False,
                steady_deadline20ms_missed=False, startup_20ms_allowance_used=False,
                post_reply_deadline=dict(accepted=True, checked_ns=end, allowance_used=False,
                    startup_allowance_used=False, rolling_misses=0, consecutive_misses=0),
                command=dict(phase=phase, q_model_rad=positions[:],
                    kp=[ten['axes'][str(mid)]['kp']*gain for mid in profile.shadow.CAN_ORDER],
                    kd=[ten['axes'][str(mid)]['kd']*gain for mid in profile.shadow.CAN_ORDER],
                    velocity_reference_rad_s=[0.]*12, feedforward_torque_nm=[0.]*12,
                    command_velocity_rad_s=[0.]*12, tracking_error_rad=[0.]*12,
                    estimated_pd_torque_nm=[0.]*12),
                feedback=dict(q_model_rad=positions[:], velocity_rad_s=[0.]*12,
                    torque_nm=[0.]*12, temperature_c=[30.]*12)))
        ten_report = dict(status='COMPLETE_SUPPORTED_OUTPUT', errors=[], scope=ten['scope'],
            boot_id=ten['boot_id'], motor_power_epoch=ten['motor_power_epoch'],
            cadence_source_sha256=copy.deepcopy(ten['cadence_source_sha256']),
            normal_ramp_completed=True, motor_enable_sent=True, motion_gain_sent=True,
            command_output_sent=True, learned_targets_sent=True, stop_confirmed=True,
            current_position_hold_only=False, cyclic_inference_skipped=False,
            post_reply_deadline_policy=copy.deepcopy(ten['post_reply_deadline_policy']),
            post_reply_deadline_rejections=[], deadline20ms_misses=0,
            steady_deadline20ms_misses=0, post_reply_deadline_allowance_uses=0,
            startup_20ms_allowance_enabled=False, startup_20ms_allowance_uses=0,
            trial_displacement_origin='final_pre_enable_feedback', trial_origin_model_rad_by_id=origins,
            cycles=self.rows, actual_model_calls=468, execution_settings=profile.execution_settings(ten),
            native_batch_encoder=dict(enabled=True,binary_sha256=ten['native_batch_encoder']['sha256']),
            model_provenance=dict(manifest_sha256=ten['artifacts']['scalar_step_manifest']['sha256'],
                baseline_provenance=dict(manifest_sha256=ten['artifacts']['model_manifest']['sha256'])),
            transport_settings=dict(request_gap_us=ten['request_gap_us'],request_window=ten['request_window']),
            stop_reports={bus:dict(complete=True,confirmed_ids=ids,unconfirmed_ids=[],ambiguous_ids=[],
                                   fault_by_id={str(mid):0 for mid in ids})
                for bus,ids in (('front',list(range(1,7))),('rear',list(range(7,13))))})
        self.docs.update(prior_supported_profile=ten, prior_supported_report=ten_report,
            prior_supported_observation=dict(user_statement='SYNTHETIC ONLY operator report, never robot evidence',
                observed_by='operator', audio_heard=True, abnormal_noise_vibration_slip_sinking_contact=False,
                box_support_maintained=True, autonomous_standing_or_walking_observed=False))
        self.extension = dict(mode=profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S,scope=self.data['scope'],
            only_duration_extended=True,live_limits_unchanged=True,support_must_remain=True,
            load_bearing_not_established=True,walking_allowed=False,
            review={**self.data['review'],'decision':'ACCEPT_20S_SUPPORTED_AFTER_10S'})
        self.docs['hardware_review']['supported_extension_acceptance'] = self.extension

    def seal(self):
        return extension_fixture.SupportedExtensionProfileTests.seal(self)

    def load(self):
        return profile.load_profile(self.seal())

    def rewrite_history(self, key, document):
        path=Path(self.ten['artifacts'][key]['path'])
        reference=_write(path,document)
        self.ten['artifacts'][key]={'path':str(path),'sha256':reference['sha256']}

    def reseal_history_reviews(self):
        operator=json.loads(Path(self.ten['artifacts']['operator_acceptance']['path']).read_text())
        operator['reviewed_settings_sha256']=profile.reviewed_settings_sha256(self.ten)
        operator['artifact_sha256']={k:self.ten['artifacts'][k]['sha256']
            for k in profile.artifact_names(self.ten) if k not in ('operator_acceptance','hardware_review')}
        self.rewrite_history('operator_acceptance',operator)
        hardware=json.loads(Path(self.ten['artifacts']['hardware_review']['path']).read_text())
        hardware['reviewed_settings_sha256']=profile.reviewed_settings_sha256(self.ten)
        hardware['artifact_sha256']={k:self.ten['artifacts'][k]['sha256']
            for k in profile.artifact_names(self.ten) if k != 'hardware_review'}
        acceptance=hardware['supported_extension_acceptance']
        for field,name in (('prior_profile_sha256','prior_supported_profile'),
                           ('prior_report_sha256','prior_supported_report'),
                           ('prior_observation_sha256','prior_supported_observation')):
            acceptance[field]=self.ten['artifacts'][name]['sha256']
        self.rewrite_history('hardware_review',hardware)

    def test_exact_20_seconds_preserves_learned_limits_unknowns_and_box_scope(self):
        loaded = self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertEqual(loaded['timing_review']['kind'], 'supported_policy_20s_after_10s_admission_only')
        self.assertEqual((loaded['duration_s'],loaded['policy_weight'],loaded['hard_cycle_ms']), (20.,.005,20.))
        self.assertFalse(profile.current_position_hold_only(loaded))
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertEqual(profile.post_reply_deadline_settings(loaded),self.ten['post_reply_deadline_policy'])
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])
        self.assertFalse(self.docs['hardware_review']['type2_dynamic']['1']['velocity_scale_and_sign_verified'])
        self.assertFalse(loaded['watchdog_by_id']['1']['usb_disconnect_test_passed'])

    def test_20_exact_only_and_legacy_duration_caps_unchanged(self):
        for invalid in (19.999,20.001,30.,True):
            self.data['duration_s']=invalid
            with self.subTest(duration=invalid),self.assertRaises(profile.ProfileError): self.load()
        for mode,cap in ((profile.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,10.),
                         (profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,2.),
                         (profile.SUPPORTED_POLICY_PROBE_5S,5.)):
            self.data.update(diagnostic_timing_acceptance=mode,duration_s=cap+.001)
            with self.subTest(mode=mode),self.assertRaises(profile.ProfileError):profile._settings(self.data)

    def test_no_contract_change_even_if_new_review_is_resealed(self):
        baseline=copy.deepcopy(self.data)
        changes=(lambda p:p.update(boot_id='00000000-0000-0000-0000-000000000001'),
            lambda p:p.update(motor_power_epoch='different-power'),lambda p:p.update(policy_weight=.004),
            lambda p:p.update(startup_duration_s=.5),lambda p:p.update(request_gap_us=1000),
            lambda p:p['axes']['1'].update(kp=2.),lambda p:p['axes']['1'].update(kd=.1),
            lambda p:p['axes']['1'].update(max_displacement_from_start_rad=math.radians(.9)),
            lambda p:p['axes']['1'].update(max_measured_velocity_rad_s=.3),
            lambda p:p['axes']['1'].update(physical_lower_rad=p['axes']['1']['physical_lower_rad']+.001),
            lambda p:p['post_reply_deadline_policy'].update(max_lateness_ms=.5),
            lambda p:p.update(start_pose_bounds={'not_the_prior_pose': True}))
        for change in changes:
            self.data=copy.deepcopy(baseline);change(self.data)
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_predecessor_must_be_approved_10s_learned_not_hold_or_human(self):
        baseline=copy.deepcopy(self.ten)
        for key,value in (('duration_s',3.),('policy_weight',0.),('approved_for_supported_policy_output',False),
                          ('blockers',['pending']),('diagnostic_timing_acceptance',profile.CURRENT_HOLD_AFTER_SUPPORTED_10S),
                          ('scope',profile.HUMAN_SUPPORTED_PARTIAL_SCOPE)):
            self.docs['prior_supported_profile']=copy.deepcopy(baseline)
            self.docs['prior_supported_profile'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()

    def test_static_model_and_source_pins_cannot_change(self):
        baseline=copy.deepcopy(self.ten)
        for change in (lambda p:p['artifacts']['calibration'].update(sha256='f'*64),
                       lambda p:p['artifacts']['model_manifest'].update(sha256='f'*64),
                       lambda p:p['artifacts']['scalar_step_manifest'].update(sha256='f'*64),
                       lambda p:p['artifacts']['local_reference_capture'].update(sha256='f'*64),
                       lambda p:p['native_batch_encoder'].update(sha256='f'*64),
                       lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64})):
            self.docs['prior_supported_profile']=copy.deepcopy(baseline);change(self.docs['prior_supported_profile'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_incomplete_model_output_stop_or_wrong_report_binding_rejected(self):
        baseline=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r.update(status='ABORTED'),lambda r:r.update(errors=['failure']),
            lambda r:r.update(motor_enable_sent=False),lambda r:r.update(motion_gain_sent=False),
            lambda r:r.update(command_output_sent=False),lambda r:r.update(learned_targets_sent=False),
            lambda r:r.update(current_position_hold_only=True),lambda r:r.update(cyclic_inference_skipped=True),
            lambda r:r.update(actual_model_calls=0),lambda r:r.update(cycles=r['cycles'][:474]),
            lambda r:r.update(motor_power_epoch='stale'),lambda r:r.update(stop_confirmed=False),
            lambda r:r['stop_reports']['rear'].update(confirmed_ids=list(range(7,12))),
            lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1]),
            lambda r:r['stop_reports']['front']['fault_by_id'].update({'1':1}),
            lambda r:r['model_provenance'].update(manifest_sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(baseline);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_causal_cycle_and_original_deadlines_are_checked_not_summary_labels(self):
        baseline=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r['cycles'][4].update(index=6),
            lambda r:r['cycles'][4].update(output_reply_end_ns=r['cycles'][4]['begin_ns']-1),
            lambda r:r['cycles'][4].update(end_ns=r['cycles'][4]['begin_ns']+21_000_001),
            lambda r:r['cycles'][4].update(oldest_input_to_final_host_write_ms=20.001),
            lambda r:r['cycles'][4]['post_reply_deadline'].update(checked_ns=1),
            lambda r:r['cycles'][4]['post_reply_deadline'].update(accepted=False),
            lambda r:r.update(deadline20ms_misses=1),
            lambda r:r['cycles'][-1].update(phase='active'))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(baseline);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_twelve_axis_record_and_gain_torque_displacement_caps_verified(self):
        baseline=copy.deepcopy(self.docs['prior_supported_report'])
        index=profile.shadow.CAN_ORDER.index(1)
        changes=(lambda r:r['cycles'][60]['command'].update(kp=[3.]*11),
            lambda r:r['cycles'][60]['command']['kp'].__setitem__(index,3.001),
            lambda r:r['cycles'][60]['command']['velocity_reference_rad_s'].__setitem__(index,.001),
            lambda r:r['cycles'][60]['command']['feedforward_torque_nm'].__setitem__(index,.001),
            lambda r:r['cycles'][60]['command']['q_model_rad'].__setitem__(index,-1.+math.radians(1.01)),
            lambda r:r['cycles'][60]['command']['estimated_pd_torque_nm'].__setitem__(index,.101),
            lambda r:r['cycles'][60]['feedback']['velocity_rad_s'].__setitem__(index,.351),
            lambda r:r['cycles'][60]['feedback']['torque_nm'].__setitem__(index,1.001),
            lambda r:r['cycles'][60]['feedback']['temperature_c'].__setitem__(index,45.001),
            lambda r:r['cycles'][60]['feedback']['q_model_rad'].__setitem__(index,'NaN'),
            lambda r:[c['command'].update(kp=[0.]*12,kd=[0.]*12) for c in r['cycles']],
            lambda r:r['cycles'][-1]['command'].update(kp=[.1]*12))
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(baseline);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_actual_operator_observation_and_supported_only_named_review_required(self):
        baseline=copy.deepcopy(self.docs['prior_supported_observation'])
        for key,value in (('audio_heard',False),('abnormal_noise_vibration_slip_sinking_contact',True),
            ('box_support_maintained',False),('observed_by','inferred'),('user_statement',''),
            ('autonomous_standing_or_walking_observed',True)):
            self.docs['prior_supported_observation']=copy.deepcopy(baseline)
            self.docs['prior_supported_observation'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
        self.docs['prior_supported_observation']=baseline
        for key,value in (('only_duration_extended',False),('live_limits_unchanged',False),
                          ('support_must_remain',False),('load_bearing_not_established',False),('walking_allowed',True)):
            original=self.extension[key];self.extension[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.extension[key]=original
        self.extension['review']['decision']='ACCEPT_10S_SUPPORTED_AFTER_2S'
        with self.assertRaises(profile.ProfileError):self.load()

    def test_nested_two_second_source_graph_is_verified(self):
        self.seal()
        earlier=Path(self.ten['artifacts']['prior_supported_report']['path'])
        earlier.write_text(earlier.read_text()+' ')
        with self.assertRaisesRegex(profile.ProfileError,'SHA256'):self.load()

    def test_chain_that_changes_physical_pose_is_rejected_even_under_old_10s_gate(self):
        earlier=json.loads(Path(self.ten['artifacts']['prior_supported_profile']['path']).read_text())
        earlier['axes']['1']['physical_lower_rad']-=.001
        self.rewrite_history('prior_supported_profile',earlier)
        report=json.loads(Path(self.ten['artifacts']['prior_supported_report']['path']).read_text())
        report['profile_sha256']=self.ten['artifacts']['prior_supported_profile']['sha256']
        self.rewrite_history('prior_supported_report',report)
        observed=json.loads(Path(self.ten['artifacts']['prior_supported_observation']['path']).read_text())
        observed['report_sha256']=self.ten['artifacts']['prior_supported_report']['sha256']
        self.rewrite_history('prior_supported_observation',observed)
        self.reseal_history_reviews()
        # Old ten-second admission deliberately allowed new local envelopes;
        # the new twenty-second gate explicitly does not reuse that exception.
        nested={k:profile._artifact(ref,self.history)[0] for k,ref in self.ten['artifacts'].items()}
        profile._supported_extension_evidence(nested,self.ten)
        with self.assertRaisesRegex(profile.ProfileError,'same physical pose'):self.load()

    def test_only_historical_admission_loader_hash_may_differ(self):
        self.ten['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py']='a'*64
        self.docs['prior_supported_report']['cadence_source_sha256']=copy.deepcopy(self.ten['cadence_source_sha256'])
        self.reseal_history_reviews()
        self.assertTrue(self.load()['output_allowed'])

    def test_existing_rare_deadline_policy_is_inherited_without_expansion(self):
        report=self.docs['prior_supported_report']
        rows=report['cycles'];index=25
        rows[index]['end_ns']=rows[index]['begin_ns']+20_100_000
        rows[index]['post_reply_deadline'].update(checked_ns=rows[index]['end_ns'],allowance_used=True,
            rolling_misses=1,consecutive_misses=1)
        rows[index].update(deadline20ms_missed=True,steady_deadline20ms_missed=True)
        for offset,row in enumerate(rows[index+1:],index+1):
            for name in ('begin_ns','output_reply_end_ns','end_ns'):row[name]+=101_000
            row['post_reply_deadline']['checked_ns']=row['end_ns']
            row['post_reply_deadline']['rolling_misses']=int(offset-index<100)
        report.update(deadline20ms_misses=1,steady_deadline20ms_misses=1,post_reply_deadline_allowance_uses=1)
        loaded=self.load()
        self.assertEqual(profile.post_reply_deadline_settings(loaded),self.ten['post_reply_deadline_policy'])
        self.assertEqual((loaded['hard_cycle_ms'],loaded['max_sample_age_ms']),(20.,20.))
        self.data['post_reply_deadline_policy']['max_lateness_ms']=1.001
        with self.assertRaises(profile.ProfileError):self.load()

    def test_first_cycle_policy_requires_identical_chain_and_named_review(self):
        self.data['startup_cycle_allowance']=profile.FIRST_CYCLE_POST_REPLY
        self.ten['startup_cycle_allowance']=profile.FIRST_CYCLE_POST_REPLY
        earlier=json.loads(Path(self.ten['artifacts']['prior_supported_profile']['path']).read_text())
        earlier['startup_cycle_allowance']=profile.FIRST_CYCLE_POST_REPLY
        self.rewrite_history('prior_supported_profile',earlier)
        report=json.loads(Path(self.ten['artifacts']['prior_supported_report']['path']).read_text())
        report.update(profile_sha256=self.ten['artifacts']['prior_supported_profile']['sha256'],
            startup_20ms_allowance_enabled=True,startup_20ms_allowance_uses=0,steady_deadline20ms_misses=0)
        self.rewrite_history('prior_supported_report',report)
        observed=json.loads(Path(self.ten['artifacts']['prior_supported_observation']['path']).read_text())
        observed['report_sha256']=self.ten['artifacts']['prior_supported_report']['sha256']
        self.rewrite_history('prior_supported_observation',observed)
        acceptance=dict(mode=profile.FIRST_CYCLE_POST_REPLY,scope=self.data['scope'],first_cycle_only=True,
            hard_output_and_freshness_limits_unchanged=True,steady_miss_budget_unchanged=True,
            review={**self.data['review'],'decision':'ACCEPT_FIRST_CYCLE_POST_REPLY'})
        self.docs['hardware_review']['startup_cycle_acceptance']=acceptance
        hardware=json.loads(Path(self.ten['artifacts']['hardware_review']['path']).read_text())
        hardware['startup_cycle_acceptance']=copy.deepcopy(acceptance)
        self.rewrite_history('hardware_review',hardware)
        self.reseal_history_reviews()
        self.docs['prior_supported_report']['startup_20ms_allowance_enabled']=True
        self.assertTrue(profile.reviewed_startup_cycle_allowance(self.load()))
        del self.ten['startup_cycle_allowance']
        with self.assertRaises(profile.ProfileError):self.load()

    def test_malformed_proof_mappings_fail_closed_before_output(self):
        baseline=copy.deepcopy(self.docs['prior_supported_report'])
        for field in ('native_batch_encoder','model_provenance','transport_settings','stop_reports'):
            self.docs['prior_supported_report']=copy.deepcopy(baseline)
            self.docs['prior_supported_report'][field]=[]
            with self.subTest(field=field),self.assertRaises(profile.ProfileError):self.load()
        self.docs['prior_supported_report']=copy.deepcopy(baseline)
        self.docs['prior_supported_report']['cycles'][7]['post_reply_deadline']=[]
        with self.assertRaises(profile.ProfileError):self.load()

    def test_twenty_second_artifact_tampering_rejected(self):
        path=self.seal()
        report_path=self.base/'prior_supported_report.json'
        report_path.write_text(report_path.read_text()+' ')
        with self.assertRaisesRegex(profile.ProfileError,'SHA256'):profile.load_profile(path)


if __name__ == '__main__':unittest.main()
