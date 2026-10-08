"""Synthetic native890 saved-file admission; never robot/user evidence or I/O."""
import copy
import hashlib
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_active_fk as fk
import test_policy_preauthorized_boxed_sequence as legacy
from test_post_reply_input_age_extensions import select_v2, wire_cycles
from test_post_reply_input_age_v2 import settings
from test_native_phase_pair_20s_extension import _native_report


class NativeBoxedPreauthorizationTests(unittest.TestCase):
    POLICY_V1 = False
    write = legacy.PreauthorizedBoxedSequenceTests.write
    capture = legacy.PreauthorizedBoxedSequenceTests.capture
    seal_graph = legacy.PreauthorizedBoxedSequenceTests.seal_graph

    def setUp(self):
        old = legacy.PreauthorizedBoxedSequenceTests()
        old.setUp(); self.addCleanup(old.doCleanups)
        self.base, self.shared, self.nodes = old.base, old.shared, old.nodes
        self.data, self.docs = old.data, old.docs
        self.original = copy.deepcopy(old.original)
        self.original.update(user_statement='SYNTHETIC authorize boxed native890 2,10,20; questions waived',
                             authorized_durations_s=[2,10,20])
        conditions = json.loads(Path(old.auth['current_conditions_source']['path']).read_text())
        conditions.update(user_statement='SYNTHETIC current native boxed conditions')
        original_ref = self.write(self.base/'native-original.json', self.original)
        conditions_ref = self.write(self.base/'native-conditions.json', conditions)
        for data, docs, directory in self.nodes:
            data.pop('preauthorized_boxed_sequence')
            data.update(preauthorized_native_boxed_sequence=True, native_phase_pair=True,
                        request_gap_us=890, request_window=3,
                        post_reply_deadline_policy=(copy.deepcopy(data['post_reply_deadline_policy'])
                                                    if self.POLICY_V1 else settings()))
            if data['duration_s'] == 30.:
                data.update(duration_s=20.,diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)
                docs['hardware_review']['supported_extension_acceptance'].update(
                    mode=live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)
                docs['hardware_review']['supported_extension_acceptance']['review']['decision']='ACCEPT_20S_SUPPORTED_AFTER_10S'
            data['cadence_source_sha256']=live.cadence_source_hashes(data)
            diagnostic=docs['pipeline_diagnostic']
            diagnostic['native_phase_pair']=True
            diagnostic['plan'].update(native_phase_pair=True,request_gap_us=890,request_window=3)
            diagnostic['transport_settings'].update(request_gap_us=890,request_window=3)
            diagnostic['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
            diagnostic['source_provenance']['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
            diagnostic['native_phase_pair_proof']=dict(mode='persistent_dual_owner.v1',
                request_count_per_cycle=26,all_phases_joined=True,owner_placement_verified=True,
                owner_settings_restored=True,coordinator_placement_verified=True,
                coordinator_settings_restored=True,active_deadlines_unchanged=True)
            hardware=docs['hardware_review']
            if not self.POLICY_V1:select_v2(data,hardware)
            hardware['native_phase_pair_acceptance']=dict(
                schema='singularitydog.native-phase-pair-review.v1',mode='persistent_dual_owner.v1',
                scope=data['scope'],request_count_per_cycle=26,active_deadlines_unchanged=True,
                stop_proxy_does_not_certify_active_api_latency=True,
                cadence_source_sha256=copy.deepcopy(data['cadence_source_sha256']),
                pre_send_input_and_native_output_limits_unchanged=True,
                output_feedback_sample_age_limit_unchanged=True,post_reply_input_age_budget_ms=1.,
                review={**data['review'],'decision':'ACCEPT_NATIVE_PHASE_PAIR'})
            if self.POLICY_V1:
                acceptance=hardware['native_phase_pair_acceptance']
                for key in ('pre_send_input_and_native_output_limits_unchanged',
                            'output_feedback_sample_age_limit_unchanged','post_reply_input_age_budget_ms'):
                    acceptance.pop(key)
                acceptance['hard_output_and_freshness_limits_unchanged']=True
        manifest={'files':{'runtime/'+name:{'sha256':sha} for name,sha in self.data['cadence_source_sha256'].items()}}
        source=self.write(self.base/'native-source-manifest.json',manifest)
        self.auth=dict(schema=live.NATIVE_BOXED_SEQUENCE_AUTHORIZATION_SCHEMA,
            scope='native_boxed_small_sequence_only',authorized_durations_s=[2,10,20],
            boot_id=self.data['boot_id'],motor_power_epoch=self.data['motor_power_epoch'],
            assembly_id=self.data['assembly_id'],native_phase_pair=True,request_gap_us=890,request_window=3,
            reference_capture_sha256=self.data['artifacts']['local_reference_capture']['sha256'],
            sequence_contract_sha256=live.preauthorized_boxed_sequence_contract_sha256(self.data),
            source_manifest=source,source_manifest_sha256=source['sha256'],
            human_authorization_source=original_ref,current_conditions_source=conditions_ref,
            automatic_continuation_explicitly_authorized=True,post_trial_confirmation_questions_waived=True,
            future_physical_observations_must_remain_unknown=True,box_removal_allowed=False,
            load_transfer_allowed=False,standing_allowed=False,walking_allowed=False,
            review={**self.data['review'],'decision':'AUTHORIZE_NATIVE_BOXED_2_10_20_WITH_UNKNOWN_POST_OBSERVATIONS'})
        self.auth_ref=self.write(self.base/'native-authorization.json',self.auth)
        self.reseal()

    def actual(self,report,prior,docs):
        after=legacy.PreauthorizedBoxedSequenceTests.actual(self,report,prior,docs)
        with patch('test_post_reply_input_age_extensions.settings',return_value=prior['post_reply_deadline_policy']):
            wire_cycles(report,prior,docs['pipeline_diagnostic']['observer']['accel_input_hypothesis'])
        _native_report(report,prior)
        change=getattr(self,'_report_change',None)
        if change is not None and report is change[0]:change[1](report)
        return after

    def reseal(self):
        for data,docs,directory in self.nodes:
            data['artifacts']['boxed_sequence_authorization']=copy.deepcopy(self.auth_ref)
            docs['boxed_sequence_authorization']=copy.deepcopy(self.auth)
            after=500_000_000
            if 'prior_supported_profile' in docs:
                prior=docs['prior_supported_profile'];report=docs['prior_supported_report']
                prior_docs=next(d for p,d,_ in self.nodes if p is prior)
                after=self.actual(report,prior,prior_docs)
                prior_ref=self.write(directory/'prior_supported_profile.json',prior)
                report['profile_sha256']=prior_ref['sha256']
                report_ref=self.write(directory/'prior_supported_report.json',report)
                scope=dict(kind='latency_power_scope_exit',status='RESTORED',restored=True,
                    cpu_performance_restored=True,restore_errors=[],caught_signal=None,
                    child_exit_code=0,restored_ns=after)
                console=json.dumps(scope)+'\n'+json.dumps(dict(kind='python_switch_interval_restore',restored=True))+'\n'
                receipt=dict(report_sha256=report_ref['sha256'],profile_sha256=prior_ref['sha256'],
                    source_manifest_sha256=self.auth['source_manifest_sha256'],boot_before=data['boot_id'],
                    boot_after=data['boot_id'],operator_power_epoch=data['motor_power_epoch'],exit_code=0,
                    source_files_unchanged=True,box_removal_allowed=False,
                    cpu_scope_restoration_console=console,files={'console.stdout':hashlib.sha256(console.encode()).hexdigest()})
                self.receipt=receipt
                receipt_ref=self.write(directory/'execution-receipt.json',receipt)
                docs['prior_supported_observation']=dict(schema=live.NATIVE_BOXED_SEQUENCE_NUMERIC_RESULT_SCHEMA,
                    source='authenticated_original_supported_report',synthetic_interaction=False,
                    profile_sha256=prior_ref['sha256'],report_sha256=report_ref['sha256'],
                    preauthorization_sha256=self.auth_ref['sha256'],boot_id=data['boot_id'],
                    motor_power_epoch=data['motor_power_epoch'],numeric_result_only=True,
                    physical_result_inferred=False,post_trial_audio_heard=None,post_trial_anomalies=None,
                    post_trial_support_maintained=None,execution_receipt=receipt_ref)
            capture=self.capture(data,docs,after)
            docs['boxed_sequence_current_capture']=capture
            data['artifacts']['boxed_sequence_current_capture']=self.write(directory/'current-capture.json',capture)
            self.seal_graph(data,docs,directory)
            hardware=docs['hardware_review']
            hardware['native_phase_pair_acceptance']['diagnostic_sha256']=data['artifacts']['pipeline_diagnostic']['sha256']
            self.seal_graph(data,docs,directory)

    def load(self):
        return live.load_profile(self.nodes[-1][2]/'profile.json')

    def test_full_nested_native890_2_10_20_loader_keeps_physical_observations_unknown(self):
        for data,docs,directory in self.nodes:
            with self.subTest(seconds=data['duration_s']):
                value=live.load_profile(directory/'profile.json')
                self.assertTrue(live.native_phase_pair_settings(value))
                self.assertTrue(fk.selected(value))
                self.assertTrue(value['support_must_remain'])
                self.assertFalse(value['actual_policy_output_20ms_verified'])
                self.assertEqual(value['_post_trial_physical_observation'],
                    dict(audio_heard=None,anomalies=None,support_maintained=None,observed=False))
                self.assertEqual((value['request_gap_us'],value['request_window'],value['policy_weight']), (890,3,.005))

    def test_flags_do_not_expand_legacy_or_native_scope_or_future_results(self):
        for key,bad in (('preauthorized_native_boxed_sequence',1),('preauthorized_boxed_sequence',True),
                        ('native_phase_pair',False),('request_gap_us',880),('request_gap_us',900),
                        ('request_gap_us',891),('request_window',2),('duration_s',20.001),('duration_s',30.),
                        ('duration_s',60.),('policy_weight',.0051),('voltage_max_v',43.),
                        ('hard_cycle_ms',21.),('max_sample_age_ms',21.),('max_sample_gap_ms',21.001),
                        ('max_consecutive_20ms_misses',1)):
            value=copy.deepcopy(self.data);value[key]=bad
            with self.subTest(key=key,bad=bad),self.assertRaises(live.ProfileError):live.execution_settings(value)
        for key,bad in (('kp',3.001),('kd',.151),('max_displacement_from_start_rad',math.radians(1.001)),
                        ('max_estimated_pd_torque_nm',.101),('max_measured_velocity_rad_s',.351)):
            value=copy.deepcopy(self.data);value['axes']['1'][key]=bad
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.execution_settings(value)
        value=copy.deepcopy(self.data);value['post_reply_deadline_policy']['post_reply_input_age_budget_ms']=1.001
        with self.assertRaises(live.ProfileError):live.execution_settings(value)

    def test_native_authorization_cannot_copy_legacy_or_claim_future_physical_facts(self):
        for change in (lambda d:d['boxed_sequence_authorization'].update(schema=live.BOXED_SEQUENCE_AUTHORIZATION_SCHEMA),
                       lambda d:d['boxed_sequence_authorization'].update(authorized_durations_s=[2,10,30]),
                       lambda d:d['boxed_sequence_authorization'].update(request_gap_us=900),
                       lambda d:d['boxed_sequence_authorization'].update(walking_allowed=True),
                       lambda d:d['boxed_sequence_authorization'].update(motor_power_epoch='old'),
                       lambda d:d['boxed_sequence_authorization'].update(sequence_contract_sha256='f'*64),
                       lambda d:d['prior_supported_observation'].update(post_trial_audio_heard=True),
                       lambda d:d['prior_supported_observation'].update(post_trial_anomalies=False),
                       lambda d:d['prior_supported_observation'].update(physical_result_inferred=True)):
            docs=copy.deepcopy(self.docs);change(docs)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_native_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_completed_numeric_label_cannot_replace_raw_stop_restore_or_capture(self):
        for change in (lambda d:d['prior_supported_report'].update(status='ABORTED'),
                       lambda d:d['prior_supported_report'].update(normal_ramp_completed=False),
                       lambda d:d['prior_supported_report']['journal'][0]['records'].clear(),
                       lambda d:d['prior_supported_report']['cycles'][1]['imu'].update(read_started_monotonic_ns=1),
                       lambda d:d['prior_supported_report']['stop_reports']['rear']['evidence']['records'].pop(),
                       lambda d:d['prior_supported_report']['stop_reports']['front']['evidence']['records'][0].update(rx_hex='00'),
                       lambda d:d['boxed_sequence_current_capture']['identities']['1'].update(mcu_uid_hex='foreign'),
                       lambda d:d['boxed_sequence_current_capture'].update(boot_id='old'),
                       lambda d:d['boxed_sequence_current_capture']['telemetry']['rows']['1']['position_samples'][0].update(request_monotonic_ns=1)):
            docs=copy.deepcopy(self.docs);change(docs)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_native_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        docs=copy.deepcopy(self.docs);receipt=copy.deepcopy(self.receipt);receipt['exit_code']=2
        docs['prior_supported_observation']['execution_receipt']=self.write(self.base/'bad-restore.json',receipt)
        with self.assertRaises(live.ProfileError):
            live._preauthorized_native_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_twenty_still_requires_actual_native_two_and_ten_and_fresh_post_ten_diagnostic(self):
        for report in (self.nodes[1][1]['prior_supported_report'],self.docs['prior_supported_report']):
            self._report_change=(report,lambda r:r['native_phase_pair'].update(enabled=False))
            self.reseal()
            with self.assertRaisesRegex(live.ProfileError,'selected actual native predecessor'):self.load()
            self._report_change=None;self.reseal()
        diagnostic=self.docs['pipeline_diagnostic'];saved=copy.deepcopy(diagnostic)
        from test_native_phase_pair_20s_extension import _diagnostic_at
        _diagnostic_at(diagnostic,1_000_000)
        self.reseal()
        with self.assertRaisesRegex(live.ProfileError,'follow the completed predecessor|follow the completed ten-second|after the completed ten-second'):self.load()
        diagnostic.clear();diagnostic.update(saved);self.reseal()

    def test_numeric_capture_or_continuation_permission_cannot_replace_current_physical_confirmation(self):
        ref=self.auth['current_conditions_source']
        original=json.loads(Path(ref['path']).read_text())
        # The valid, unchanged numeric capture proves no physical clearance.
        # The same automatic permission still cannot supply a missing current
        # confirmation after adjustment of a joint or of the supporting box.
        for key in ('all12_local_plus_minus3deg_clear',
                    'power_pose_unchanged_since_direct_confirmation', 'box_will_remain'):
            for bad in (None,False):
                conditions=copy.deepcopy(original);conditions['current_conditions'][key]=bad
                docs=copy.deepcopy(self.docs)
                docs['boxed_sequence_authorization']['current_conditions_source']=self.write(
                    self.base/'unconfirmed-current-conditions.json',conditions)
                with self.subTest(key=key,bad=bad),self.assertRaisesRegex(
                        live.ProfileError,'Direct current boxed conditions incomplete'):
                    live._preauthorized_native_boxed_sequence_authorization(docs,self.data,self.base)
        for key,bad,reason in (('direct_human',False,'original direct human'),
                               ('source','inferred_from_numeric_capture','original direct human'),
                               ('boot_id','old','different boot or power epoch'),
                               ('motor_power_epoch','old','different boot or power epoch')):
            conditions=copy.deepcopy(original);conditions[key]=bad
            docs=copy.deepcopy(self.docs)
            docs['boxed_sequence_authorization']['current_conditions_source']=self.write(
                self.base/'unconfirmed-current-conditions.json',conditions)
            with self.subTest(key=key),self.assertRaisesRegex(live.ProfileError,reason):
                live._preauthorized_native_boxed_sequence_authorization(docs,self.data,self.base)

    def test_raw_or_mutated_loaded_dict_cannot_mint_native_runtime_proof(self):
        with self.assertRaises(live.ProfileError):live.native_phase_pair_settings(self.data)
        value=self.load();value['request_gap_us']=900
        with self.assertRaises(live.ProfileError):live.native_phase_pair_settings(value)

    def test_enabling_or_disabling_new_flag_after_loading_breaks_immutable_native_binding(self):
        value=self.load();value['preauthorized_native_boxed_sequence']=False
        with self.assertRaisesRegex(live.ProfileError,'immutable complete loader proof'):
            live.native_phase_pair_settings(value)
        # A normally admitted two-second native/FK profile also cannot be
        # promoted by toggling this option after loading. No runtime executes.
        raw,docs,_=self.nodes[0]
        raw,docs=copy.deepcopy(raw),copy.deepcopy(docs)
        raw['preauthorized_native_boxed_sequence']=False
        for name in live._BOXED_SEQUENCE_ARTIFACTS:
            raw['artifacts'].pop(name);docs.pop(name)
        directory=self.base/'non-preauthorized-native-two';directory.mkdir()
        self.seal_graph(raw,docs,directory)
        value=live.load_profile(directory/'profile.json')
        value['preauthorized_native_boxed_sequence']=True
        live._preauthorized_native_boxed_sequence_scope(value)  # Scope alone is insufficient.
        with self.assertRaisesRegex(live.ProfileError,'immutable complete loader proof'):
            live.native_phase_pair_settings(value)

    def test_complete_sequence_cannot_change_uid_source_encoder_power_or_model(self):
        for change in (lambda d:d.update(motor_power_epoch='different-power'),
                       lambda d:d.update(boot_id='different-boot'),
                       lambda d:d['axes']['1'].update(uid='different-uid'),
                       lambda d:d['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64}),
                       lambda d:d['native_batch_encoder'].update(sha256='f'*64),
                       lambda d:d['artifacts']['target_fk_manifest'].update(sha256='f'*64),
                       lambda d:d['artifacts']['local_reference_capture'].update(sha256='f'*64)):
            value=copy.deepcopy(self.data);change(value)
            with self.subTest(change=change),self.assertRaisesRegex(live.ProfileError,'exact current sequence contract'):
                live._preauthorized_native_boxed_sequence_result(
                    self.docs['prior_supported_observation'],self.docs,value,self.base)

    def test_mutable_future_facts_or_foreign_current_capture_cannot_be_admitted(self):
        for key in ('post_trial_audio_heard','post_trial_anomalies','post_trial_support_maintained'):
            for bad in (True,False):
                docs=copy.deepcopy(self.docs);docs['prior_supported_observation'][key]=bad
                with self.subTest(key=key,bad=bad),self.assertRaises(live.ProfileError):
                    live._preauthorized_native_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        for change in (lambda c:c['telemetry']['rows']['1'].update(voltage=34.9),
                       lambda c:c['telemetry']['rows']['1'].update(run_mode=2),
                       lambda c:c['telemetry']['rows']['1'].update(current=.1),
                       lambda c:c['telemetry']['rows']['1'].update(median_position_rad=100.),
                       lambda c:c.update(angle_wrap_applied=True)):
            docs=copy.deepcopy(self.docs);change(docs['boxed_sequence_current_capture'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_native_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)


class NativeBoxedV1PreauthorizationTests(NativeBoxedPreauthorizationTests):
    POLICY_V1 = True

    def test_original_v1_is_kept_without_inventing_v2_input_age_budget(self):
        value=self.load()
        self.assertEqual(value['post_reply_deadline_policy']['mode'],'bounded_post_reply_v1')
        self.assertNotIn('post_reply_input_age_budget_ms',value['post_reply_deadline_policy'])
        for data,docs,_ in self.nodes:
            self.assertEqual(data['post_reply_deadline_policy'],value['post_reply_deadline_policy'])

    def test_switching_original_policy_cannot_extend_the_authenticated_sequence(self):
        value=copy.deepcopy(self.data);value['post_reply_deadline_policy']=settings()
        with self.assertRaisesRegex(live.ProfileError,'exact current sequence contract'):
            live._preauthorized_native_boxed_sequence_result(
                self.docs['prior_supported_observation'],self.docs,value,self.base)


if __name__=='__main__':unittest.main()
