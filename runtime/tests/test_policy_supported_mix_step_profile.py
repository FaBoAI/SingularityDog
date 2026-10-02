"""Synthetic 10-percent boxed admission only; no robot/model/device operation."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
from test_policy_local_profile import seal_local
import shutil
import test_policy_supported_extension_profile as extension_fixture

TOOLS = Path(__file__).resolve().parents[2]/'tools'
sys.path.insert(0, str(TOOLS))
from analyze_policy_target_mixture import analyze


class SupportedMixStepTests(unittest.TestCase):
    def setUp(self):
        self._prepare_ten_second_history()
        self.data.update(diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_MIX_STEP_10PCT,
                         duration_s=5., startup_duration_s=1., policy_ramp_s=2., stop_duration_s=.4,
                         policy_weight=.1)
        self.data.pop('startup_damping_duration_s',None)
        self.docs['hardware_review']['local_characterization']['local_clearance_rad']=math.radians(7)
        origins=self.docs['prior_supported_report']['trial_origin_model_rad_by_id']
        ids=[origins[mid] for mid in profile.IDS]
        self.origins=origins
        # Active-output vectors use ID order. Model CAN_ORDER is a separate ABI.
        for index,row in enumerate(self.rows):
            row['phase']='starting' if index==0 else 'active' if index<468 else 'stopped' if index==492 else 'stopping'
            row['effective_policy_weight']=.005 if row['phase']=='active' else 0.
            row['command']['phase']=row['phase']
            row['command']['q_model_rad']=ids[:]
            row['feedback']['q_model_rad']=ids[:]
            row['startup_20ms_allowance_used']=False
            if index==492:
                row['command']['kp']=[0.]*12;row['command']['kd']=[0.]*12
        prior_report=self.docs['prior_supported_report']
        prior_report['cycles']=self.rows
        for mid,axis in self.data['axes'].items():
            q=origins[mid];model_index=profile.shadow.CAN_ORDER.index(int(mid))
            axis.update(physical_lower_rad=max(profile.shadow.LOWER[model_index],q-math.radians(7)),
                        physical_upper_rad=min(profile.shadow.UPPER[model_index],q+math.radians(7)),
                        max_command_velocity_rad_s=.12,max_command_acceleration_rad_s2=.5,
                        max_tracking_error_rad=math.radians(3),max_estimated_pd_torque_nm=.25,
                        max_displacement_from_start_rad=math.radians(6))
            self.docs['hardware_review']['angles'][mid].update(
                physical_lower_rad=axis['physical_lower_rad'],physical_upper_rad=axis['physical_upper_rad'])
        diag=self.docs['pipeline_diagnostic']
        # One initial whole-cycle exception remains admission-only, age/reply20ms.
        diag['motor_power_epoch']=self.data['motor_power_epoch']
        diag['cadence_source_sha256']=copy.deepcopy(self.data['cadence_source_sha256'])
        self.source=self.base/'replay_source.py';self.source.write_text('# SYNTHETIC no model/device replay\n')
        self.kit=_write(self.base/'replay_kit.json',dict(files={
            'runtime/'+k:v for k,v in self.ten['cadence_source_sha256'].items()}))
        targets=dict(schema='singularitydog.saved-policy-target-sequence.v1',id_order=list(range(1,13)),
            rows=[dict(cycle_index=i,raw_target_model_rad=[q+math.radians(5) for q in ids]) for i in range(468)],
            profile_sha256='0'*64,report_sha256='0'*64,historical_boot_id=self.data['boot_id'],
            historical_motor_power_epoch=self.data['motor_power_epoch'],model_backend=profile.SCALAR_BACKEND,
            command=self.data['command'],h_hypothesis=self.data['h_hypothesis'],
            model_provenance=copy.deepcopy(prior_report['model_provenance']),kit_manifest_sha256=self.kit['sha256'],
            output_allowed=False,hardware_accessed=False,closed_loop_prediction=False,
            load_bearing_verified=False,standing_verified=False,box_removal_allowed=False)
        self.original_statement=dict(schema='singularitydog.supported-mix-step-operator-receipt.v1',
            boot_id=self.data['boot_id'],motor_power_epoch=self.data['motor_power_epoch'],
            source_id='SYNTHETIC original current physical question',question='SYNTHETIC seven-degree boxed readiness?',
            answer='SYNTHETIC ONLY all twelve 7-degree paths, box, paws, hands-off, immediate cutoff unchanged',
            clearance_deg=7,all_twelve_current_clearance_confirmed=True,box_supports_body=True,
            four_paws_floor=True,hands_clear=True,cutoff_ready=True,power_and_pose_unchanged_since_prior_10s=True,
            box_removal_authorized=False,load_bearing_verified=False,standing_verified=False)
        self.statement=_write(self.base/'real_statement.json',self.original_statement)
        source=dict(schema='singularitydog.supported-mix-step-source-review.v1',
            prior_source_sha256=copy.deepcopy(self.ten['cadence_source_sha256']),
            new_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
            changed_sources={k:dict(before=self.ten['cadence_source_sha256'][k],after=v) for k,v in self.data['cadence_source_sha256'].items() if self.ten['cadence_source_sha256'][k]!=v},replay_source=dict(path=str(self.source),sha256=hashlib.sha256(self.source.read_bytes()).hexdigest()),
            replay_kit_manifest=self.kit,target_sequence_sha256='0'*64,replay_input_report_sha256='0'*64,
            model_values_unchanged=True,replay_uses_original_inputs=True,recorded_feedback_not_new_mix_feedback=True,
            closed_loop_prediction=False,standing_prediction=False,output_allowed=False,
            review={**self.data['review'],'decision':'ACCEPT_SUPPORTED_MIX_STEP_SOURCE_DELTA'})
        clearance=dict(schema='singularitydog.supported-mix-step-clearance.v1',
            mode=profile.SUPPORTED_POLICY_MIX_STEP_10PCT,scope=self.data['scope'],
            boot_id=self.data['boot_id'],motor_power_epoch=self.data['motor_power_epoch'],
            capture_sha256=self.data['artifacts']['local_reference_capture']['sha256'],
            uids_by_id={mid:a['uid'] for mid,a in self.data['axes'].items()},
            reference_turns_by_id=copy.deepcopy(self.docs['hardware_review']['local_characterization']['reference_turns_by_id']),
            local_clearance_rad=math.radians(7),support_must_remain=True,four_paws_floor=True,hands_off=True,
            cutoff_ready=True,current_pose_unchanged=True,current_power_unchanged=True,box_removal_allowed=False,
            standing_allowed=False,walking_allowed=False,load_bearing_verified=False,
            user_statement=json.loads((self.base/self.statement['path']).read_text())['answer'],source_receipt=self.statement,
            review={**self.data['review'],'decision':'ACCEPT_CURRENT_7DEG_SUPPORTED_MIX_STEP_CLEARANCE'})
        self.docs.update(saved_policy_target_sequence=targets,policy_mixture_analysis={},
                         mix_step_source_review=source,mix_step_clearance=clearance)
        for name in profile._MIX_STEP_ARTIFACTS:self.data['artifacts'][name]={'path':name+'.json','sha256':'0'*64}
        self.acceptance=dict(mode=profile.SUPPORTED_POLICY_MIX_STEP_10PCT,scope=self.data['scope'],
            support_must_remain=True,load_bearing_not_established=True,box_removal_allowed=False,
            standing_allowed=False,walking_allowed=False,
            review={**self.data['review'],'decision':'ACCEPT_5S_SUPPORTED_LEARNED_MIX_STEP_10PCT'})
        self.docs['hardware_review']['supported_mix_step_acceptance']=self.acceptance

    def _prepare_ten_second_history(self):
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
        self.data.update(assembly_id='SYNTHETIC ten-percent mixture')
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

    def seal(self, *, refresh_analysis=True):
        for name in ('prior_supported_profile','prior_supported_report','prior_supported_observation'):
            if name=='prior_supported_report':self.docs[name]['profile_sha256']=self.data['artifacts']['prior_supported_profile']['sha256']
            if name=='prior_supported_observation':self.docs[name]['report_sha256']=self.data['artifacts']['prior_supported_report']['sha256']
            self.data['artifacts'][name]=_write(self.base/(name+'.json'),self.docs[name])
        targets=self.docs['saved_policy_target_sequence']
        targets.update(profile_sha256=self.data['artifacts']['prior_supported_profile']['sha256'],
                       report_sha256=self.data['artifacts']['prior_supported_report']['sha256'])
        self.data['artifacts']['saved_policy_target_sequence']=_write(self.base/'saved_policy_target_sequence.json',targets)
        if refresh_analysis:
            self.docs['policy_mixture_analysis']=analyze(self.docs['prior_supported_report'],targets,
                report_sha256=targets['report_sha256'],targets_sha256=self.data['artifacts']['saved_policy_target_sequence']['sha256'],
                physical_clearance_deg=7,max_displacement_deg=6)
        source=self.docs['mix_step_source_review']
        source.update(target_sequence_sha256=self.data['artifacts']['saved_policy_target_sequence']['sha256'],
                      replay_input_report_sha256=self.data['artifacts']['prior_supported_report']['sha256'])
        seal_local(self.base,self.data,self.docs)
        for field,name in (('prior_profile_sha256','prior_supported_profile'),('prior_report_sha256','prior_supported_report'),
            ('prior_observation_sha256','prior_supported_observation'),('target_sequence_sha256','saved_policy_target_sequence'),
            ('mixture_analysis_sha256','policy_mixture_analysis'),('clearance_sha256','mix_step_clearance'),
            ('source_review_sha256','mix_step_source_review'),('capture_sha256','local_reference_capture'),
            ('diagnostic_sha256','pipeline_diagnostic')):self.acceptance[field]=self.data['artifacts'][name]['sha256']
        seal_local(self.base,self.data,self.docs)
        return self.base/'profile.json'

    def load(self):return profile.load_profile(self.seal())

    def test_exact_mode_all_evidence_and_unknowns_preserved(self):
        loaded=self.load()
        self.assertTrue(loaded['output_allowed']);self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(profile.current_position_hold_only(loaded));self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertEqual((loaded['duration_s'],loaded['policy_weight']), (5.,.1))
        self.assertAlmostEqual(profile.local_characterization_settings(loaded)['max_displacement_rad'],math.radians(6))
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])
        self.assertFalse(self.docs['hardware_review']['type2_dynamic']['1']['torque_interpretation_verified'])

    def test_newmode_rejects_other_percent_duration_schedule_and_scope(self):
        for key,value in (('policy_weight',.01),('policy_weight',.3),('policy_weight',1.),('duration_s',10.),
            ('startup_duration_s',.4),('policy_ramp_s',1.),('stop_duration_s',.5),('hard_cycle_ms',21.),
            ('max_sample_age_ms',21.),('max_sample_gap_ms',22.),('scope',profile.HUMAN_SUPPORTED_PARTIAL_SCOPE),
            ('model_backend','native_baseline')):
            old=self.data[key];self.data[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(profile.ProfileError):self.load()
            self.data[key]=old

    def test_oldmode_still_cannot_accept_ten_percent_or_six_degrees(self):
        self.data['diagnostic_timing_acceptance']=profile.SUPPORTED_POLICY_PROBE_5S
        with self.assertRaises(profile.ProfileError):profile._settings(self.data)
        self.data['policy_weight']=.005
        with self.assertRaises(profile.ProfileError):profile._axes(self.data,self.docs['calibration'])

    def test_all_new_axis_limits_have_exact_ceiling_and_gain_no_increase(self):
        for key,value in (('kp',3.01),('kd',.151),('max_command_velocity_rad_s',.120001),
            ('max_command_acceleration_rad_s2',.500001),('max_tracking_error_rad',math.radians(3)+1e-6),
            ('max_measured_velocity_rad_s',.350001),('max_measured_torque_nm',1.01),
            ('max_estimated_pd_torque_nm',.250001),('max_temperature_c',45.01),
            ('max_displacement_from_start_rad',math.radians(6)+1e-6)):
            old=self.data['axes']['1'][key];self.data['axes']['1'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.data['axes']['1'][key]=old

    def test_unapproved_template_returns_nooutput_and_cannot_mint_token(self):
        self.data.update(approved_for_supported_policy_output=False,review=None,blockers=['SYNTHETIC pending review'])
        path=self.seal();loaded=profile.load_profile(path,require_approved=False)
        self.assertFalse(loaded['output_allowed'])
        with self.assertRaises(profile.ProfileError):profile.load_profile(path)
        with self.assertRaises(profile.ProfileError):profile.local_characterization_settings(loaded)

    def test_token_is_bound_to_settings_axes_and_proof(self):
        loaded=self.load()
        for change in (lambda d:d['axes']['1'].update(max_displacement_from_start_rad=math.radians(10)),
                       lambda d:d['artifacts']['mix_step_clearance'].update(sha256='f'*64)):
            changed=copy.copy(loaded);changed['axes']=copy.deepcopy(loaded['axes']);changed['artifacts']=copy.deepcopy(loaded['artifacts'])
            change(changed)
            with self.assertRaises(profile.ProfileError):profile.local_characterization_settings(changed)

    def test_prior_failure_incomplete_stop_or_wrong_pose_cannot_admit(self):
        original=copy.deepcopy(self.docs['prior_supported_report'])
        for change in (lambda r:r.update(status='ABORTED'),lambda r:r.update(errors=['failure']),
            lambda r:r.update(learned_targets_sent=False),lambda r:r.update(actual_model_calls=469),
            lambda r:r['stop_reports']['rear'].update(ambiguous_ids=[12]),
            lambda r:r['cycles'][20].update(end_ns=r['cycles'][20]['begin_ns']+20_000_001),
            lambda r:r['cycles'][25]['feedback']['q_model_rad'].__setitem__(0,0.)):
            self.docs['prior_supported_report']=copy.deepcopy(original);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()

    def test_source_upgrade_only_named_loader_and_benchmark(self):
        self.docs['mix_step_source_review']['changed_sources']={'singularitydog_hw/policy_output_runtime.py':{'before':'a'*64,'after':'b'*64}}
        with self.assertRaisesRegex(profile.ProfileError,'source delta'):self.load()
        self.docs['mix_step_source_review']['changed_sources']={k:dict(before=self.ten['cadence_source_sha256'][k],after=v) for k,v in self.data['cadence_source_sha256'].items() if self.ten['cadence_source_sha256'][k]!=v}
        self.docs['mix_step_source_review']['review']=None
        with self.assertRaisesRegex(profile.ProfileError,'named review'):self.load()

    def test_targets_cannot_omit_calls_hide_overrange_or_claim_prediction(self):
        original=copy.deepcopy(self.docs['saved_policy_target_sequence'])
        changes=(lambda d:d['id_order'].__setitem__(0,True),lambda d:d['rows'][0].update(cycle_index=False),lambda d:d['rows'].pop(),lambda d:d['rows'][2].update(cycle_index=5),
                 lambda d:d.update(closed_loop_prediction=True),lambda d:d['rows'][3]['raw_target_model_rad'].__setitem__(0,True),
                 lambda d:d['rows'][3]['raw_target_model_rad'].__setitem__(0,-2.2))
        for change in changes:
            self.docs['saved_policy_target_sequence']=copy.deepcopy(original);change(self.docs['saved_policy_target_sequence'])
            with self.subTest(change=change),self.assertRaises((profile.ProfileError,ValueError)):self.load()

    def test_operator_receipt_old_clearance_unsupported_or_unreviewed_rejected(self):
        original=copy.deepcopy(self.docs['mix_step_clearance'])
        for key,value in (('review',None),('local_clearance_rad',math.radians(3)),('motor_power_epoch','old'),
            ('support_must_remain',False),('current_pose_unchanged',False),('box_removal_allowed',True),
            ('user_statement','invented alternate statement')):
            self.docs['mix_step_clearance']=copy.deepcopy(original);self.docs['mix_step_clearance'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()

    def test_analysis_must_equal_all_targets_and_remain_unapproved(self):
        self.seal();original=copy.deepcopy(self.docs['policy_mixture_analysis'])
        for change in (lambda d:d.update(output_allowed=True),lambda d:d.update(target_rows=467),
            lambda d:d['mixtures'][0]['per_axis'][0].update(peak_absolute_delta_deg=.1),
            lambda d:d['cap_deg'].update(maximum_displacement=[7.]*12)):
            self.docs['policy_mixture_analysis']=copy.deepcopy(original);change(self.docs['policy_mixture_analysis'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):profile.load_profile(self.seal(refresh_analysis=False))

    def test_derived_clearance_cannot_override_stale_or_denied_original_receipt(self):
        changes=(('schema','answer-only'),('boot_id','old-boot'),('motor_power_epoch','old-power'),
            ('source_id',''),('question',''),('answer','invented answer'),('clearance_deg',3),
            ('clearance_deg',True),('all_twelve_current_clearance_confirmed',False),
            ('box_supports_body',False),('four_paws_floor',False),('hands_clear',False),
            ('cutoff_ready',False),('power_and_pose_unchanged_since_prior_10s',False),
            ('box_removal_authorized',True),('load_bearing_verified',True),('standing_verified',True))
        for key,value in changes:
            statement=copy.deepcopy(self.original_statement);statement[key]=value
            reference=_write(self.base/'real_statement.json',statement)
            self.docs['mix_step_clearance']['source_receipt']=reference
            with self.subTest(key=key,value=value),self.assertRaisesRegex(profile.ProfileError,'source receipt|original pinned'):
                self.load()

    def test_missing_original_receipt_fields_and_alternate_statement_cannot_admit(self):
        for key in self.original_statement:
            statement=copy.deepcopy(self.original_statement);statement.pop(key)
            self.docs['mix_step_clearance']['source_receipt']=_write(self.base/'real_statement.json',statement)
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
        statement=copy.deepcopy(self.original_statement)
        statement['answer']='different original answer'
        statement['user_statement']=self.original_statement['answer']
        self.docs['mix_step_clearance']['source_receipt']=_write(self.base/'real_statement.json',statement)
        with self.assertRaisesRegex(profile.ProfileError,'original pinned'):self.load()

    def test_diagnostic_keeps_initial_admission_but_current_boot_power_sources(self):
        self.assertEqual(self.load()['timing_review']['kind'],'supported_policy_mix_step_10pct_5s_admission_only')
        original=copy.deepcopy(self.docs['pipeline_diagnostic'])
        for change in (lambda d:d.update(motor_power_epoch='old'),lambda d:d.update(cadence_source_sha256={}),
            lambda d:d['measurements'][10].update(skipped_slots_before=1),
            lambda d:d['measurements'][10].update(last_proxy_reply_ns=d['measurements'][10]['oldest_input_start_ns']+20_000_001)):
            self.docs['pipeline_diagnostic']=copy.deepcopy(original);change(self.docs['pipeline_diagnostic'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):self.load()


if __name__=='__main__':unittest.main()
