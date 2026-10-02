"""Synthetic preparation evidence only; no individual robot data or approvals."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from tools import prepare_supported_policy_mix_step as tool


class FakeLive:
    SUPPORTED_POLICY_PROBE_10S_AFTER_2S = 'supported-policy-probe-10s-after-2s-v1'
    LOCAL_NUMERICAL_MARGIN_RAD = .001
    shadow = SimpleNamespace(CAN_ORDER=list(range(1,13)),LOWER=[-.3]*12,UPPER=[.3]*12)
    @staticmethod
    def reviewed_settings_sha256(p):
        excluded = ('artifacts','review','blockers','approved_for_supported_policy_output','assembly_id')
        return hashlib.sha256(json.dumps({k:v for k,v in p.items() if k not in excluded},sort_keys=True).encode()).hexdigest()
    @staticmethod
    def _structure(p):
        if not all(key in p['artifacts'] for key in ('mix_step_clearance','mix_step_source_review','saved_policy_target_sequence','policy_mixture_analysis')):
            raise ValueError('Missing new artifact')
    @staticmethod
    def _settings(p):
        if p['diagnostic_timing_acceptance'] != tool.MODE: raise ValueError('Wrong mode')


class MixBuilderTests(unittest.TestCase):
    def setUp(self):
        ref = lambda c: dict(path='/synthetic/'+c,sha256=c*64)
        self.refs = {name:ref(chr(97+i)) for i,name in enumerate(tool.REF_NAMES)}
        self.refs.update(operator_receipt=ref('h'),replay_source=ref('i'),replay_kit_manifest=ref('j'))
        axis = dict(uid='synthetic',sign=1,offset_rad=0.,uncertainty_rad=None,
            kp=3,kd=.15,max_command_velocity_rad_s=math.radians(1),max_command_acceleration_rad_s2=math.radians(5),
            max_tracking_error_rad=math.radians(2),max_displacement_from_start_rad=math.radians(1),
            max_estimated_pd_torque_nm=.1,max_measured_velocity_rad_s=.35,max_measured_torque_nm=1.,
            max_temperature_c=45.,physical_lower_rad=-math.radians(3),physical_upper_rad=math.radians(3))
        self.sources = {'singularitydog_hw/policy_live_profile.py':'b'*64,'singularitydog_hw/policy_output_runtime.py':'c'*64}
        self.prior = dict(approved_for_supported_policy_output=True,blockers=[],review={'decision':'old'},
            diagnostic_timing_acceptance=FakeLive.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,scope=tool.SCOPE,duration_s=10.,
            policy_weight=.005,boot_id='synthetic-boot',motor_power_epoch='operator-source-current-on',model_backend='scalar_step_cpp',
            cadence_source_sha256={**self.sources,'singularitydog_hw/policy_live_profile.py':'a'*64},
            axes={mid:copy.deepcopy(axis) for mid in tool.IDS},start_pose_bounds={mid:[-.01,.01] for mid in tool.IDS},
            artifacts=dict(local_reference_capture=ref('l'),scalar_step_manifest=ref('s')),
            startup_duration_s=.4,policy_ramp_s=.4,stop_duration_s=.4,
            startup_cycle_allowance='old',startup_damping_duration_s=.08,post_reply_deadline_policy={'old':True},
            period_ms=20,hard_cycle_ms=20,max_sample_age_ms=20,max_sample_gap_ms=21,max_consecutive_20ms_misses=0)
        report = dict(profile_sha256=self.refs['prior_supported_profile']['sha256'],boot_id=self.prior['boot_id'],
            motor_power_epoch=self.prior['motor_power_epoch'],cadence_source_sha256=self.prior['cadence_source_sha256'],
            status='COMPLETE_SUPPORTED_OUTPUT',errors=[],normal_ramp_completed=True,learned_targets_sent=True,
            stop_confirmed=True,deadline20ms_misses=0,post_reply_deadline_allowance_uses=0,actual_model_calls=2)
        observed = dict(report_sha256=self.refs['prior_supported_report']['sha256'],observed_by='operator',audio_heard=True,
            abnormal_noise_vibration_slip_sinking_contact=False,box_support_maintained=True,
            autonomous_standing_or_walking_observed=False,user_statement='original observation',
            source_question_item_id='original question',statement_source='human answer')
        op = dict(schema='singularitydog.supported-mix-step-operator-receipt.v1',boot_id=self.prior['boot_id'],
            motor_power_epoch=self.prior['motor_power_epoch'],source_id='new physical question',question='original question',
            answer='original current seven-degree clearance answer',clearance_deg=7,all_twelve_current_clearance_confirmed=True,
            box_supports_body=True,four_paws_floor=True,hands_clear=True,cutoff_ready=True,
            power_and_pose_unchanged_since_prior_10s=True,box_removal_authorized=False,load_bearing_verified=False,standing_verified=False)
        targets = dict(profile_sha256=self.refs['prior_supported_profile']['sha256'],report_sha256=self.refs['prior_supported_report']['sha256'],
            kit_manifest_sha256=self.refs['replay_kit_manifest']['sha256'],model_backend='scalar_step_cpp',
            input_sequence='saved_non_stopping_feedback_and_uncorrected_sensor_IMU',feedback_not_generated_by_new_mixture=True,
            output_allowed=False,hardware_accessed=False,closed_loop_prediction=False,model_provenance=dict(manifest_sha256='s'*64))
        analysis = dict(sequence_extent='logged_model_call_count_matches',target_rows=2,
            mixtures=[dict(weight=.1,physical_clearance_exceeded_ids=[],maximum_displacement_exceeded_ids=[])])
        h = dict(review={'decision':'old'},reviewed_settings_sha256=FakeLive.reviewed_settings_sha256(self.prior),
            assembly_id='old',local_characterization=dict(reference_turns_by_id={mid:0 for mid in tool.IDS},local_clearance_rad=math.radians(3),
                absolute_zero_uncertainty_rad=None,absolute_calibration_not_certified=True,full_dynamic_feedback_not_certified=True),
            angles={mid:copy.deepcopy(axis) for mid in tool.IDS},type2_dynamic={mid:dict(velocity_scale_and_sign_verified=False,
                torque_interpretation_verified=False) for mid in tool.IDS},voltage_pipeline_acceptance=dict(review={'decision':'old'}),
            native_batch_encoder_acceptance=dict(review={'decision':'old'}),rare_jitter_diagnostic_acceptance=dict(review={'decision':'old'}))
        capture = dict(boot_id=self.prior['boot_id'],motor_output_allowed=False,
            identities={mid:dict(mcu_uid_hex='synthetic') for mid in tool.IDS},
            telemetry=dict(rows={mid:dict(median_position_rad=0.) for mid in tool.IDS}))
        self.docs = dict(prior_supported_profile=self.prior,prior_supported_report=report,prior_supported_observation=observed,
            operator_receipt=op,saved_policy_target_sequence=targets,policy_mixture_analysis=analysis,recomputed_analysis=copy.deepcopy(analysis),
            hardware_review=h,operator_acceptance=dict(review={'decision':'old'},reviewed_settings_sha256=h['reviewed_settings_sha256']),
            local_reference_capture=capture)

    def build(self,**kwargs):
        return tool.build_documents(FakeLive,self.docs,self.refs,self.sources,assembly_id='fresh-synthetic',**kwargs)

    def test_default_draft_same_calibration_boundaries_start_pose_and_all_reviews_null(self):
        original = copy.deepcopy(self.docs)
        p,h,o,c,s,_ = self.build()
        self.assertFalse(p['approved_for_supported_policy_output']); self.assertTrue(p['blockers'])
        self.assertEqual(p['policy_weight'],.1);self.assertEqual(p['duration_s'],5.)
        self.assertEqual(p['start_pose_bounds'],self.prior['start_pose_bounds'])
        self.assertFalse(set(tool.OMIT).intersection(p))
        self.assertEqual(p['startup_cycle_allowance'],self.prior['startup_cycle_allowance'])
        self.assertEqual(p['post_reply_deadline_policy'],self.prior['post_reply_deadline_policy'])
        for mid in tool.IDS:
            self.assertEqual(p['axes'][mid]['offset_rad'],0.)
            self.assertIsNone(p['axes'][mid]['uncertainty_rad'])
            self.assertEqual(p['axes'][mid]['max_estimated_pd_torque_nm'],.25)
            self.assertEqual(p['axes'][mid]['physical_lower_rad'],-math.radians(7))
        def reviews(value):
            if isinstance(value,dict):
                for key,item in value.items():
                    if key=='review':self.assertIsNone(item)
                    else:reviews(item)
            elif isinstance(value,list):
                for item in value:reviews(item)
        for doc in (p,h,o,c,s):reviews(doc)
        self.assertFalse(c['load_bearing_verified']);self.assertFalse(s['closed_loop_prediction'])
        self.assertEqual(self.docs,original)

    def test_only_explicit_named_finalization_has_reviews_and_no_physical_changes(self):
        draft = self.build()
        final = self.build(finalize=True,reviewer='synthetic reviewer',reviewed_at='2026-01-01T00:00:00+00:00',rationale='bounded file review')
        self.assertTrue(final[0]['approved_for_supported_policy_output'])
        self.assertEqual(final[3]['user_statement'],draft[3]['user_statement'])
        self.assertEqual(final[3]['review']['decision'],'ACCEPT_CURRENT_7DEG_SUPPORTED_MIX_STEP_CLEARANCE')
        self.assertFalse(final[3]['standing_allowed'])
        for args in (dict(reviewer='silent'),dict(finalize=True),dict(finalize=True,reviewer='x',reviewed_at='2026-01-01',rationale='x')):
            with self.subTest(args=args),self.assertRaises(ValueError):self.build(**args)

    def test_stale_boot_power_or_no_actual_clearance_cannot_create_draft(self):
        for key,value in (('boot_id','new'),('motor_power_epoch','new'),('hands_clear',False),
                          ('clearance_deg',True),('power_and_pose_unchanged_since_prior_10s',False),('answer','')):
            original=copy.deepcopy(self.docs['operator_receipt']);self.docs['operator_receipt'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.build()
            self.docs['operator_receipt']=original

    def test_guard_motion_or_source_changes_rejected(self):
        self.sources['singularitydog_hw/policy_output_runtime.py']='d'*64
        with self.assertRaises(ValueError):self.build()
        self.sources['singularitydog_hw/policy_output_runtime.py']='c'*64
        self.prior['axes']['1']['kp']=4
        with self.assertRaises(ValueError):self.build()

    def test_replay_mismatch_numerical_excess_or_partial_sequence_rejected(self):
        for kind in ('report','manifest','recomputed','excess','partial','model'):
            original=copy.deepcopy(self.docs)
            if kind=='report':self.docs['saved_policy_target_sequence']['report_sha256']='x'*64
            elif kind=='manifest':self.docs['saved_policy_target_sequence']['kit_manifest_sha256']='x'*64
            elif kind=='recomputed':self.docs['recomputed_analysis']['target_rows']=1
            elif kind=='excess':
                for name in ('policy_mixture_analysis','recomputed_analysis'):self.docs[name]['mixtures'][0]['maximum_displacement_exceeded_ids']=[1]
            elif kind=='partial':
                for name in ('policy_mixture_analysis','recomputed_analysis'):self.docs[name]['sequence_extent']='single_target_snapshot'
            else:self.docs['saved_policy_target_sequence']['model_provenance']['manifest_sha256']='x'*64
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.build()
            self.docs=original;self.prior=self.docs['prior_supported_profile']

    def test_wrong_observation_failed_trial_or_verified_dynamic_claim_rejected(self):
        for kind in ('noise','report','timing','dynamic'):
            original=copy.deepcopy(self.docs)
            if kind=='noise':self.docs['prior_supported_observation']['abnormal_noise_vibration_slip_sinking_contact']=True
            elif kind=='report':self.docs['prior_supported_report']['status']='ABORTED'
            elif kind=='timing':self.docs['prior_supported_report']['deadline20ms_misses']=1
            else:self.docs['hardware_review']['type2_dynamic']['1']['velocity_scale_and_sign_verified']=True
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.build()
            self.docs=original;self.prior=self.docs['prior_supported_profile']

    def test_reference_integral_turn_preserved_and_model_bounds_intersection(self):
        # The embedded offset already selects a branch: do not apply another360deg.
        self.prior['axes']['1']['offset_rad']=-2*math.pi
        self.docs['hardware_review']['reviewed_settings_sha256']=FakeLive.reviewed_settings_sha256(self.prior)
        self.docs['operator_acceptance']['reviewed_settings_sha256']=FakeLive.reviewed_settings_sha256(self.prior)
        self.docs['local_reference_capture']['telemetry']['rows']['1']['median_position_rad']=2*math.pi+.25
        p,h,_,clearance,_,_=self.build()
        self.assertEqual(p['axes']['1']['offset_rad'],-2*math.pi)
        self.assertEqual(clearance['reference_turns_by_id']['1'],0)
        self.assertEqual(p['axes']['1']['physical_upper_rad'],.3)
        self.assertAlmostEqual(p['axes']['1']['physical_lower_rad'],.25-math.radians(7))

    def test_hash_read_recheck_duplicate_json_symlink_and_exclusive_output(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory).resolve()/'data.json';p.write_text('{"v":1}')
            evidence=tool.Evidence();doc,ref=evidence.read(p)
            self.assertEqual(doc,{'v':1})
            with self.assertRaises(ValueError):evidence.read(p,'a'*64)
            link=p.parent/'link.json';link.symlink_to(p)
            with self.assertRaises(ValueError):tool.Evidence().read(link)
            p.write_text('{"v":2}')
            with self.assertRaises(ValueError):evidence.verify()
            p.write_text('{"v":1,"v":2}')
            with self.assertRaises(ValueError):tool.Evidence().read(p)
            target=p.parent/'new.json';tool.write(target,{'draft':True})
            with self.assertRaises(FileExistsError):tool.write(target,{'draft':False})


if __name__=='__main__':unittest.main()
