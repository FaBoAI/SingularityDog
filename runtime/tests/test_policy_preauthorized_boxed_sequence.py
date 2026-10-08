"""Synthetic saved-file authorization; never robot observations or outputs."""
import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_active_fk as fk
from singularitydog_hw import rs05_trial_protocol as protocol
import test_active_fk_profile as fixture
from test_post_reply_input_age_extensions import wire_cycles


class PreauthorizedBoxedSequenceTests(unittest.TestCase):
    seal_graph = fixture.ActiveFKDurationChainTests.seal_graph

    def setUp(self):
        original = fixture.ActiveFKDurationChainTests()
        original.setUp(); self.addCleanup(original.doCleanups)
        self.base, self.shared = original.base, original.shared
        self.enterContext(patch.object(fk, 'plan', side_effect=lambda data, docs=None: fixture.proof(data)))
        # Existing disabled wire conversion has its own suite. Actual predecessor
        # journals, timing admissions, STOPs, source and restoration are not mocked.
        self.enterContext(patch.object(live, '_voltage_fast_pipeline_trace'))
        self.nodes = []
        self.data, self.docs = self.convert(copy.deepcopy(original.data), copy.deepcopy(original.docs), 30)
        self.original = dict(SYNTHETIC_NOT_ROBOT_EVIDENCE=True,
            source='direct_current_user_reply_in_codex', direct_human=True, synthetic_interaction=False,
            user_statement='SYNTHETIC automatic boxed 2,10,30 permission; no post confirmation questions',
            user_reply_id='synthetic-original-message', boot_id=self.data['boot_id'],
            motor_power_epoch=self.data['motor_power_epoch'], authorized_durations_s=[2,10,30],
            automatic_continuation_explicitly_authorized=True, post_trial_confirmation_questions_waived=True)
        conditions = copy.deepcopy(self.original)
        conditions['current_conditions'] = dict.fromkeys(('motor_40v_on', 'box_supports_body',
            'four_feet_touch_floor', 'all12_local_plus_minus3deg_clear', 'hands_off', 'immediate_40v_cutoff',
            'other_drive_tools_stopped', 'box_will_remain', 'power_pose_unchanged_since_direct_confirmation'), True)
        conditions['current_conditions'].update(load_transfer_allowed=False, standing_allowed=False, walking_allowed=False)
        original_ref = self.write(self.base/'original-authorization.json', self.original)
        conditions_ref = self.write(self.base/'original-conditions.json', conditions)
        manifest = {'files': {'runtime/'+name: {'sha256': sha}
                            for name,sha in self.data['cadence_source_sha256'].items()}}
        source = self.write(self.base/'source-manifest.json', manifest)
        self.auth = dict(schema=live.BOXED_SEQUENCE_AUTHORIZATION_SCHEMA, scope='boxed_small_sequence_only',
            authorized_durations_s=[2,10,30], boot_id=self.data['boot_id'],
            motor_power_epoch=self.data['motor_power_epoch'], assembly_id=self.data['assembly_id'],
            reference_capture_sha256=self.data['artifacts']['local_reference_capture']['sha256'],
            sequence_contract_sha256=live.preauthorized_boxed_sequence_contract_sha256(self.data),
            source_manifest=source, source_manifest_sha256=source['sha256'],
            human_authorization_source=original_ref, current_conditions_source=conditions_ref,
            automatic_continuation_explicitly_authorized=True, post_trial_confirmation_questions_waived=True,
            future_physical_observations_must_remain_unknown=True, box_removal_allowed=False,
            load_transfer_allowed=False, standing_allowed=False, walking_allowed=False,
            review={**self.data['review'], 'decision':'AUTHORIZE_BOXED_2_10_30_WITH_UNKNOWN_POST_OBSERVATIONS'})
        self.auth_ref = self.write(self.base/'authorization.json', self.auth)
        self.reseal()

    def write(self, path, data):
        ref = fixture._write(path, data); ref['path'] = str(path); return ref

    def convert(self, data, docs, seconds):
        directory=self.base/('preauthorized-'+str(seconds)); directory.mkdir()
        if 'prior_supported_profile' in docs:
            prior=docs['prior_supported_profile']
            prior_base=Path(data['artifacts']['prior_supported_profile']['path']).parent
            prior_docs={key:json.loads((Path(ref['path']) if Path(ref['path']).is_absolute()
                                       else prior_base/ref['path']).read_text())
                        for key,ref in prior['artifacts'].items()}
            prior, prior_docs=self.convert(copy.deepcopy(prior), prior_docs, 10 if seconds==30 else 2)
            docs['prior_supported_profile']=prior
        data.update(preauthorized_boxed_sequence=True, request_gap_us=900, assembly_id='SYNTHETIC same boxed assembly', duration_s=float(seconds),
            diagnostic_timing_acceptance={2:live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                10:live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,30:live.SUPPORTED_POLICY_PROBE_30S_PREAUTHORIZED}[seconds])
        data['cadence_source_sha256']=live.cadence_source_hashes(data)
        # This fixture begins with V2 histories; the new path must also retain
        # the original V1 1-per-100, <=1ms nonconsecutive policy unchanged.
        data['post_reply_deadline_policy'] = dict(mode='bounded_post_reply_v1', max_lateness_ms=1.,
            rolling_window_cycles=100, max_misses_per_window=1, max_consecutive_misses=1)
        data['start_pose_bounds']={}
        for mid in live.IDS:
            row=docs['local_reference_capture']['telemetry']['rows'][mid]
            q=data['axes'][mid]['sign']*row['median_position_rad']+data['axes'][mid]['offset_rad']
            data['start_pose_bounds'][mid]=[q-math.radians(.5),q+math.radians(.5)]
        hardware=docs['hardware_review']; hardware['assembly_id']=data['assembly_id']
        for key in ('post_reply_deadline_acceptance','rare_jitter_diagnostic_acceptance',
                    'voltage_pipeline_acceptance','native_batch_encoder_acceptance',
                    'startup_cycle_acceptance','prepared_voltage_publication_acceptance'):
            if key not in hardware: continue
            acceptance=hardware[key]
            for name in ('pre_send_input_and_native_output_limits_unchanged',
                         'output_feedback_sample_age_limit_unchanged','post_reply_input_age_budget_ms'):
                acceptance.pop(name,None)
            acceptance['hard_output_and_freshness_limits_unchanged']=True
            if key=='rare_jitter_diagnostic_acceptance': acceptance['live_deadline_policy_unchanged']=True
            if key=='post_reply_deadline_acceptance':
                acceptance.update(schema='singularitydog.post-reply-deadline-review.v1',
                    settings=copy.deepcopy(data['post_reply_deadline_policy']))
                acceptance['review']['decision']='ACCEPT_BOUNDED_POST_REPLY_DEADLINE'
        diagnostic=docs['pipeline_diagnostic']
        diagnostic['plan']['request_gap_us']=900
        diagnostic.setdefault('transport_settings',{})['request_gap_us']=900
        diagnostic['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
        diagnostic['source_provenance']['cadence_source_sha256']=copy.deepcopy(data['cadence_source_sha256'])
        diagnostic['model_source']=fixture.provenance(data)
        if seconds>2:
            offset=20_000_000_000 if seconds==10 else 40_000_000_000
            for row in diagnostic['measurements']:
                for key in row:
                    if key.endswith('_ns'): row[key]+=offset
            diagnostic['absolute_epoch_schedule']['epoch_ns']+=offset
            hardware['supported_extension_acceptance']['mode']=data['diagnostic_timing_acceptance']
            hardware['supported_extension_acceptance']['review']['decision']=(
                'ACCEPT_10S_SUPPORTED_AFTER_2S' if seconds==10 else 'ACCEPT_30S_PREAUTHORIZED_SUPPORTED_AFTER_10S')
        self.nodes.append((data,docs,directory))
        return data,docs

    def capture(self, data, docs, after):
        capture=copy.deepcopy(docs['local_reference_capture'])
        capture.update(motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT', approved_for_runtime=False,
                       angle_wrap_applied=False)
        for mid,row in capture['telemetry']['rows'].items():
            row.update(current=0.,voltage=40.)
            for index,sample in enumerate(row['position_samples']):
                # Deepcopy duplicate dictionaries in the old toy fixture.
                row['position_samples'][index]=dict(rad=sample['rad'],request_monotonic_ns=after+1+index*2_000_000,
                                                    reply_monotonic_ns=after+1_000_000+index*2_000_000)
        return capture

    def actual(self, report, prior, docs):
        origins={mid: prior['axes'][mid]['sign']*docs['local_reference_capture']['telemetry']['rows'][mid]['median_position_rad']
                     +prior['axes'][mid]['offset_rad'] for mid in live.IDS}
        for row in report['cycles']:
            phase=row['phase']; gain=phase=='active'
            row['command']=dict(phase=phase,q_model_rad=list(origins.values()),
                kp=[prior['axes'][mid]['kp'] if gain else 0. for mid in live.IDS],
                kd=[prior['axes'][mid]['kd'] if gain else 0. for mid in live.IDS],
                velocity_reference_rad_s=[0.]*12,feedforward_torque_nm=[0.]*12,
                command_velocity_rad_s=[0.]*12,tracking_error_rad=[0.]*12,estimated_pd_torque_nm=[0.]*12)
            row['feedback']=dict(q_model_rad=list(origins.values()),velocity_rad_s=[0.]*12,torque_nm=[0.]*12,temperature_c=[30.]*12)
        report.update(trial_origin_model_rad_by_id=origins, motor_enable_sent=True, imu_restore_status='restored',
            actual_model_calls=len(report['cycles'])-2,current_position_hold_only=False,cyclic_inference_skipped=False,
            model_provenance=fixture.provenance(prior), execution_settings=live.execution_settings(prior),
            cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']))
        report['transport_settings'].update(request_gap_us=900,request_window=3)
        with patch('test_post_reply_input_age_extensions.settings',return_value=prior['post_reply_deadline_policy']):
            wire_cycles(report,prior,docs['pipeline_diagnostic']['observer']['accel_input_hypothesis'])
        last=report['cycles'][-1]['end_ns']; last_reply=last
        for bus,ids in (('front',range(1,7)),('rear',range(7,13))):
            raw=[]
            for ordinal,mid in enumerate(ids):
                start=last+1_000_000+ordinal*2_000_000; received=start+1_000_000
                tx=protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=mid)
                rx=protocol._wire((2<<24)|(mid<<8)|0xFD,struct.pack('>4H',32767,32767,32767,300))
                raw.append(dict(tx_hex=tx.hex(),rx_hex=rx.hex(),start_ns=start,finish_ns=start+10_000,
                    received_ns=received,written=17,received=17))
                last_reply=max(last_reply,received)
            report['stop_reports'][bus]['evidence']={'records':raw}
        return last_reply+1_000_000

    def reseal(self):
        for data,docs,directory in self.nodes:
            data['artifacts']['boxed_sequence_authorization']=copy.deepcopy(self.auth_ref)
            docs['boxed_sequence_authorization']=copy.deepcopy(self.auth)
            after=500_000_000
            if 'prior_supported_profile' in docs:
                prior=docs['prior_supported_profile']; report=docs['prior_supported_report']
                prior_docs=next(d for p,d,_ in self.nodes if p is prior)
                after=self.actual(report,prior,prior_docs)
                prior_ref=self.write(directory/'prior_supported_profile.json',prior)
                report['profile_sha256']=prior_ref['sha256']
                report_ref=self.write(directory/'prior_supported_report.json',report)
                scope=dict(kind='latency_power_scope_exit',status='RESTORED',restored=True,
                    cpu_performance_restored=True,restore_errors=[],caught_signal=None,child_exit_code=0,restored_ns=after)
                console=json.dumps(scope)+'\n'+json.dumps(dict(kind='python_switch_interval_restore',restored=True))+'\n'
                receipt=dict(report_sha256=report_ref['sha256'],profile_sha256=prior_ref['sha256'],
                    source_manifest_sha256=self.auth['source_manifest_sha256'], boot_before=data['boot_id'],boot_after=data['boot_id'],
                    operator_power_epoch=data['motor_power_epoch'],exit_code=0,source_files_unchanged=True,box_removal_allowed=False,
                    cpu_scope_restoration_console=console,files={'console.stdout':hashlib.sha256(console.encode()).hexdigest()})
                self.receipt=receipt
                receipt_ref=self.write(directory/'execution-receipt.json',receipt)
                docs['prior_supported_observation']=dict(schema=live.BOXED_SEQUENCE_NUMERIC_RESULT_SCHEMA,
                    source='authenticated_original_supported_report',synthetic_interaction=False,
                    profile_sha256=prior_ref['sha256'],report_sha256=report_ref['sha256'],
                    preauthorization_sha256=self.auth_ref['sha256'],boot_id=data['boot_id'],motor_power_epoch=data['motor_power_epoch'],
                    numeric_result_only=True,physical_result_inferred=False,post_trial_audio_heard=None,
                    post_trial_anomalies=None,post_trial_support_maintained=None,execution_receipt=receipt_ref)
            capture=self.capture(data,docs,after)
            docs['boxed_sequence_current_capture']=capture
            data['artifacts']['boxed_sequence_current_capture']=self.write(directory/'current-capture.json',capture)
            self.seal_graph(data,docs,directory)

    def load(self):
        return live.load_profile(self.base/'preauthorized-30'/'profile.json')

    def test_full_original_v1_fk_chain_admits_2_10_30_and_keeps_future_physical_unknown(self):
        for duration in (2,10,30):
            with self.subTest(duration=duration):
                loaded=live.load_profile(self.base/('preauthorized-'+str(duration))/'profile.json')
                self.assertTrue(loaded['output_allowed']);self.assertEqual(loaded['duration_s'],duration)
                self.assertEqual((loaded['policy_weight'],loaded['hard_cycle_ms'],loaded['request_gap_us'],loaded['request_window']),(.005,20,900,3))
                self.assertEqual(loaded['_post_trial_physical_observation'],
                    dict(audio_heard=None,anomalies=None,support_maintained=None,observed=False))
                self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertTrue(fk.selected(self.data))
        self.assertEqual(live.post_reply_deadline_settings(self.load())['mode'],'bounded_post_reply_v1')

    def test_selected_flag_and_duration_do_not_unlock_legacy_or_longer_modes(self):
        for field,value in (('preauthorized_boxed_sequence',False),('duration_s',29.),('duration_s',60.),
                            ('policy_weight',.006),('scope','walking')):
            candidate=copy.deepcopy(self.data);candidate[field]=value
            with self.subTest(field=field,value=value),self.assertRaises(live.ProfileError):live.execution_settings(candidate)

    def test_authorization_future_observation_and_source_contract_cannot_be_forged(self):
        for mutate in (lambda d:d['boxed_sequence_authorization'].update(motor_power_epoch='old'),
                       lambda d:d['boxed_sequence_authorization'].update(sequence_contract_sha256='f'*64),
                       lambda d:d['prior_supported_observation'].update(post_trial_audio_heard=True),
                       lambda d:d['prior_supported_observation'].update(post_trial_anomalies=False),
                       lambda d:d['prior_supported_observation'].update(physical_result_inferred=True)):
            documents=copy.deepcopy(self.docs);mutate(documents)
            with self.subTest(mutate=mutate),self.assertRaises(live.ProfileError):
                live._preauthorized_boxed_sequence_result(documents['prior_supported_observation'],documents,self.data,self.base)

    def test_completed_summary_cannot_hide_missing_faulted_stop_or_failed_restore(self):
        documents=copy.deepcopy(self.docs)
        for change in (lambda r:r.update(status='ABORTED'),lambda r:r.update(motor_enable_sent=False),
                       lambda r:r.update(normal_ramp_completed=False),lambda r:r['stop_reports']['rear']['evidence']['records'].pop(),
                       lambda r:r['cycles'][60]['feedback']['velocity_rad_s'].__setitem__(0,.351)):
            docs=copy.deepcopy(documents);change(docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        receipt=copy.deepcopy(self.receipt);receipt['exit_code']=2
        docs=copy.deepcopy(documents);docs['prior_supported_observation']['execution_receipt']=self.write(self.base/'failed-receipt.json',receipt)
        with self.assertRaisesRegex(live.ProfileError,'execution receipt'):
            live._preauthorized_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_current_read_after_restore_is_required_without_relabeling_power(self):
        docs=copy.deepcopy(self.docs);capture=docs['boxed_sequence_current_capture']
        self.assertEqual(capture['motor_power_epoch'],'NOT_INFERRED_FROM_JETSON_BOOT')
        capture['telemetry']['rows']['1']['position_samples'][0]['request_monotonic_ns']=1
        capture['telemetry']['rows']['1']['position_samples'][0]['reply_monotonic_ns']=2
        with self.assertRaisesRegex(live.ProfileError,'follow the completed predecessor'):
            live._preauthorized_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)

    def test_loaded_derived_fields_do_not_change_common_contract(self):
        loaded=self.load()
        self.assertEqual(live.preauthorized_boxed_sequence_contract_sha256(loaded),
                         live.preauthorized_boxed_sequence_contract_sha256(self.data))

    def test_legacy_ten_second_route_still_requires_a_real_post_trial_observation(self):
        data,documents,_=self.nodes[1]
        data,documents=copy.deepcopy(data),copy.deepcopy(documents)
        data['preauthorized_boxed_sequence']=False
        documents['prior_supported_profile']['preauthorized_boxed_sequence']=False
        for profile in (data,documents['prior_supported_profile']):
            for name in live._BOXED_SEQUENCE_ARTIFACTS:profile['artifacts'].pop(name)
        with self.assertRaisesRegex(live.ProfileError,'operator observation'):
            live._supported_extension_evidence(documents,data,self.base)

    def test_original_journal_fault_deadline_and_changed_joint_caps_are_rejected(self):
        for change in (lambda d:d['prior_supported_report']['journal'][0].update(error='cancelled'),
                       lambda d:d['prior_supported_report']['journal'][1]['records'][0].update(
                           received_ns=d['prior_supported_report']['cycles'][0]['begin_ns']+20_000_001),
                       lambda d:d['prior_supported_report']['stop_reports']['front']['evidence']['records'][0].update(rx_hex='00')):
            docs=copy.deepcopy(self.docs);change(docs)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_boxed_sequence_result(docs['prior_supported_observation'],docs,self.data,self.base)
        for key,value in (('kp',3.001),('max_measured_velocity_rad_s',.351),('max_measured_torque_nm',1.001)):
            profile=copy.deepcopy(self.data);profile['axes']['1'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.execution_settings(profile)

    def test_authentication_and_current_capture_do_not_accept_missing_or_foreign_evidence(self):
        for change in (lambda a:a.update(source_manifest_sha256='f'*64),
                       lambda a:a.update(source_manifest=dict(path='missing',sha256='f'*64)),
                       lambda a:a.update(automatic_continuation_explicitly_authorized=False)):
            docs=copy.deepcopy(self.docs);change(docs['boxed_sequence_authorization'])
            with self.subTest(change=change),self.assertRaises((live.ProfileError,OSError)):
                live._preauthorized_boxed_sequence_authorization(docs,copy.deepcopy(self.data),self.base)
        for change in (lambda c:c.update(boot_id='old'),
                       lambda c:c['identities']['1'].update(mcu_uid_hex='foreign'),
                       lambda c:c['telemetry']['rows']['1'].update(voltage=34.99),
                       lambda c:c['telemetry']['rows']['1'].update(current=True),
                       lambda c:c['telemetry']['rows']['1']['position_samples'][1].update(request_monotonic_ns=1)):
            capture=copy.deepcopy(self.docs['boxed_sequence_current_capture']);change(capture)
            with self.subTest(change=change),self.assertRaises(live.ProfileError):
                live._preauthorized_boxed_sequence_capture(capture,self.data,self.docs['hardware_review'])


if __name__=='__main__':unittest.main()
