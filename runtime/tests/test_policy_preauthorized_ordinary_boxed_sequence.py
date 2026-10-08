"""Synthetic ordinary900 full saved-file 2/10 admission; never robot evidence."""
import copy
import json
import math
from pathlib import Path
import unittest
from singularitydog_hw import policy_live_profile as live
import test_policy_preauthorized_boxed_sequence as legacy


class OrdinaryBoxedPreauthorizationTests(legacy.PreauthorizedBoxedSequenceTests):
    # Reuse exact original full raw-wire/STOP/restoration fixture machinery,
    # without inheriting legacy30 test scope into the new two/ten class.
    def setUp(self):
        self.ordinary_ready=False
        super().setUp()
        self.nodes=self.nodes[:2]
        self.data,self.docs,_=self.nodes[-1]
        conditions=json.loads(Path(self.auth['current_conditions_source']['path']).read_text())
        self.original.update(user_statement='SYNTHETIC persistent automatic boxed2/10 permission; questions waived',
                             authorized_durations_s=[2,10])
        conditions_ref=self.write(self.base/'ordinary-conditions.json',conditions)
        self.context=dict(schema=live.ORDINARY_BOXED_SEQUENCE_CONTEXT_SCHEMA,
            sequence_authorization_source=dict(kind='latest_direct_user_message_in_codex',exact_text=self.original['user_statement']),
            authorized_durations_s=[2,10],native_phase_pair=False,request_gap_us=900,request_window=3,
            automatic_continuation_explicitly_authorized=True,post_trial_confirmation_questions_waived=True,
            observation_reference=conditions_ref,post_trial_physical_observation_inferred=False,
            physical_anomaly_after_new_trials=None,physical_audio_heard_after_new_trials=None,
            box_removal_allowed=False,load_transfer_allowed=False,standing_allowed=False,walking_allowed=False)
        self.context_ref=self.write(self.base/'ordinary-context.json',self.context)
        self.original['source_context']=self.context_ref
        original_ref=self.write(self.base/'ordinary-original.json',self.original)
        for data,docs,_ in self.nodes:
            data.pop('preauthorized_boxed_sequence')
            data.update(preauthorized_ordinary_boxed_sequence=True,native_phase_pair=False)
            data['cadence_source_sha256']=live.cadence_source_hashes(data)
            docs['pipeline_diagnostic']['native_phase_pair']=False
            docs['pipeline_diagnostic']['plan']['native_phase_pair']=False
            docs['pipeline_diagnostic']['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
            docs['pipeline_diagnostic']['source_provenance']['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
        manifest={'files':{'runtime/'+name:{'sha256':sha} for name,sha in self.data['cadence_source_sha256'].items()}}
        source=self.write(self.base/'ordinary-source-manifest.json',manifest)
        self.auth.update(schema=live.ORDINARY_BOXED_SEQUENCE_AUTHORIZATION_SCHEMA,scope='ordinary_boxed_two_ten_only',
            authorized_durations_s=[2,10],native_phase_pair=False,request_gap_us=900,request_window=3,
            sequence_contract_sha256=live.preauthorized_boxed_sequence_contract_sha256(self.data),
            source_manifest=source,source_manifest_sha256=source['sha256'],human_authorization_source=original_ref,
            current_conditions_source=conditions_ref)
        self.auth['review']['decision']='AUTHORIZE_ORDINARY_BOXED_2_10_WITH_UNKNOWN_POST_OBSERVATIONS'
        self.auth_ref=self.write(self.base/'ordinary-authorization.json',self.auth)
        self.ordinary_ready=True
        self.reseal()

    def reseal(self):
        legacy.PreauthorizedBoxedSequenceTests.reseal(self)
        if self.ordinary_ready:
            for data,docs,directory in self.nodes:
                if 'prior_supported_observation' in docs:
                    docs['prior_supported_observation']['schema']=live.ORDINARY_BOXED_SEQUENCE_NUMERIC_RESULT_SCHEMA
                    self.seal_graph(data,docs,directory)

    def load(self):return live.load_profile(self.nodes[-1][2]/'profile.json')

    def test_ordinary_full_two_ten_graph_only_keeps_future_physical_unknown(self):
        for raw,docs,directory in self.nodes:
            value=live.load_profile(directory/'profile.json')
            self.assertTrue(value['output_allowed'])
            self.assertFalse(live.native_phase_pair_settings(value))
            self.assertEqual(value['duration_s'],raw['duration_s'])
            self.assertEqual((value['request_gap_us'],value['request_window'],value['policy_weight']),(900,3,.005))
            self.assertEqual(value['_post_trial_physical_observation'],dict(audio_heard=None,anomalies=None,support_maintained=None,observed=False))
            self.assertFalse(value['actual_policy_output_20ms_verified'])
            self.assertEqual(live.prepared_voltage_publication_settings(value),True)

    def test_ordinary_flag_scope_is_mutually_exclusive_exact_two_ten(self):
        for key,bad in (('preauthorized_ordinary_boxed_sequence',1),('preauthorized_boxed_sequence',True),
                        ('preauthorized_native_boxed_sequence',True),('native_phase_pair',True),
                        ('request_gap_us',890),('request_gap_us',900.),('request_window',2),
                        ('duration_s',20),('duration_s',30),('duration_s',60),('policy_weight',.0051),
                        ('voltage_max_v',43),('hard_cycle_ms',21),('max_sample_age_ms',21),('max_sample_gap_ms',21.001)):
            value=copy.deepcopy(self.data);value[key]=bad
            with self.subTest(key=key,bad=bad),self.assertRaises(live.ProfileError):live.execution_settings(value)
        for key,bad in (('kp',3.001),('kd',.151),('max_displacement_from_start_rad',math.radians(1.001)),
                        ('max_estimated_pd_torque_nm',.101),('max_measured_velocity_rad_s',.351)):
            value=copy.deepcopy(self.data);value['axes']['1'][key]=bad
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.execution_settings(value)

    def test_direct_selectors_reject_multiple_scopes_without_recursive_calls(self):
        selectors=(live.preauthorized_boxed_sequence_selected,
                   live.preauthorized_native_boxed_sequence_selected,
                   live.preauthorized_ordinary_boxed_sequence_selected)
        flags=('preauthorized_boxed_sequence','preauthorized_native_boxed_sequence','preauthorized_ordinary_boxed_sequence')
        for left in range(3):
            for right in range(left+1,3):
                value=dict(schema=live.SCHEMA_V3,**{flags[left]:True,flags[right]:True})
                for selector in selectors:
                    with self.subTest(left=left,right=right,selector=selector.__name__),self.assertRaises(live.ProfileError):selector(value)

    def test_new_selector_changes_break_immutable_prepared_runtime_binding(self):
        loaded=self.load();loaded['preauthorized_ordinary_boxed_sequence']=False
        with self.assertRaises(live.ProfileError):live.prepared_voltage_publication_settings(loaded)
        raw,docs,_=self.nodes[0];raw,docs=copy.deepcopy(raw),copy.deepcopy(docs)
        raw['preauthorized_ordinary_boxed_sequence']=False
        for name in live._BOXED_SEQUENCE_ARTIFACTS:raw['artifacts'].pop(name);docs.pop(name)
        directory=self.base/'ordinary-no-preauth';directory.mkdir();self.seal_graph(raw,docs,directory)
        loaded=live.load_profile(directory/'profile.json');loaded['preauthorized_ordinary_boxed_sequence']=True
        live._preauthorized_ordinary_boxed_sequence_scope(loaded)
        with self.assertRaises(live.ProfileError):live.prepared_voltage_publication_settings(loaded)

    def test_distinct_auth_schema_source_context_and_unknown_results_required(self):
        for change in (lambda d:d['boxed_sequence_authorization'].update(schema=live.NATIVE_BOXED_SEQUENCE_AUTHORIZATION_SCHEMA),
                       lambda d:d['boxed_sequence_authorization'].update(schema=live.BOXED_SEQUENCE_AUTHORIZATION_SCHEMA),
                       lambda d:d['boxed_sequence_authorization'].update(authorized_durations_s=[2,10,20]),
                       lambda d:d['boxed_sequence_authorization'].update(authorized_durations_s=[2,10,30]),
                       lambda d:d['boxed_sequence_authorization'].update(native_phase_pair=True),
                       lambda d:d['boxed_sequence_authorization'].update(request_gap_us=890),
                       lambda d:d['boxed_sequence_authorization'].update(source_manifest_sha256='f'*64),
                       lambda d:d['prior_supported_observation'].update(schema=live.BOXED_SEQUENCE_NUMERIC_RESULT_SCHEMA)):
            docs=copy.deepcopy(self.docs);change(docs)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_ordinary_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        for key in ('post_trial_audio_heard','post_trial_anomalies','post_trial_support_maintained'):
            for bad in (True,False):
                docs=copy.deepcopy(self.docs);docs['prior_supported_observation'][key]=bad
                with self.subTest(key=key,bad=bad),self.assertRaises(live.ProfileError):
                    live._preauthorized_ordinary_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_actual_original_stop_restore_source_and_current_capture_still_required(self):
        for change in (lambda d:d['prior_supported_report'].update(status='ABORTED'),
                       lambda d:d['prior_supported_report'].update(motor_enable_sent=False),
                       lambda d:d['prior_supported_report'].update(normal_ramp_completed=False),
                       lambda d:d['prior_supported_report']['journal'][0]['records'].clear(),
                       lambda d:d['prior_supported_report']['stop_reports']['rear']['evidence']['records'].pop(),
                       lambda d:d['prior_supported_report']['stop_reports']['front']['evidence']['records'][0].update(rx_hex='00'),
                       lambda d:d['boxed_sequence_current_capture'].update(boot_id='old'),
                       lambda d:d['boxed_sequence_current_capture']['identities']['1'].update(mcu_uid_hex='foreign'),
                       lambda d:d['boxed_sequence_current_capture']['telemetry']['rows']['1'].update(run_mode=2),
                       lambda d:d['boxed_sequence_current_capture']['telemetry']['rows']['1'].update(voltage=34.9),
                       lambda d:d['boxed_sequence_current_capture']['telemetry']['rows']['1']['position_samples'][0].update(request_monotonic_ns=1)):
            docs=copy.deepcopy(self.docs);change(docs)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_ordinary_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        docs=copy.deepcopy(self.docs);receipt=copy.deepcopy(self.receipt);receipt['exit_code']=2
        docs['prior_supported_observation']['execution_receipt']=self.write(self.base/'bad-ordinary-restore.json',receipt)
        with self.assertRaises(live.ProfileError):live._preauthorized_ordinary_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_changed_current_physical_conditions_cannot_be_filled_from_numeric_data(self):
        ref=self.auth['current_conditions_source'];original=json.loads(Path(ref['path']).read_text())
        for key,bad in (('all12_local_plus_minus3deg_clear',False),('power_pose_unchanged_since_direct_confirmation',False),
                        ('box_will_remain',False)):
            value=copy.deepcopy(original);value['current_conditions'][key]=bad
            docs=copy.deepcopy(self.docs);docs['boxed_sequence_authorization']['current_conditions_source']=self.write(self.base/'bad-current-ordinary.json',value)
            # Keep matching declared context so the current physical gate itself is tested.
            context=copy.deepcopy(self.context);context['observation_reference']=docs['boxed_sequence_authorization']['current_conditions_source']
            original_human=copy.deepcopy(self.original);original_human['source_context']=self.write(self.base/'bad-context-ordinary.json',context)
            docs['boxed_sequence_authorization']['human_authorization_source']=self.write(self.base/'bad-human-ordinary.json',original_human)
            with self.subTest(key=key),self.assertRaisesRegex(live.ProfileError,'Direct current boxed conditions incomplete'):
                live._preauthorized_ordinary_boxed_sequence_authorization(docs,self.data,self.base)

# Do not execute inherited tests about legacy30 semantics using ordinary2/10 data.
for name in list(vars(legacy.PreauthorizedBoxedSequenceTests)):
    if name.startswith('test_') and name not in vars(OrdinaryBoxedPreauthorizationTests):
        setattr(OrdinaryBoxedPreauthorizationTests,name,None)

if __name__=='__main__':unittest.main()
