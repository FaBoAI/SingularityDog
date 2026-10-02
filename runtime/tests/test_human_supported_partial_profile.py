"""Synthetic file-only human-catch contracts; never real approval or robot evidence."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
import wave
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw.human_supported_hold import SETTINGS
from test_policy_local_profile import local_fixture, seal_local
from test_policy_live_profile import _write


def review(data, decision):
    return {**copy.deepcopy(data['review']), 'decision': decision,
            'rationale': 'SYNTHETIC TEST ONLY: no physical robot, no output authorization.'}


def human_fixture(base):
    data, docs, pins = local_fixture(base)
    data.update(scope=live.HUMAN_SUPPORTED_PARTIAL_SCOPE,
        diagnostic_timing_acceptance=live.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
        human_supported_hold=copy.deepcopy(SETTINGS), model_backend=live.SCALAR_BACKEND,
        voltage_overlap=True, duration_s=8., startup_duration_s=1.,
        stop_duration_s=.4, policy_ramp_s=.2, policy_weight=0.)
    for axis in data['axes'].values(): axis.update(kp=6., max_estimated_pd_torque_nm=.2)
    old = copy.deepcopy(data)
    for key in ('human_supported_hold',): old.pop(key)
    old.update(scope='supported_characterization_only',
        diagnostic_timing_acceptance=live.CURRENT_HOLD_AFTER_SUPPORTED_10S,
        duration_s=3., motor_power_epoch='SYNTHETIC-prior-box-power',
        cadence_source_sha256=live.cadence_source_hashes())
    old['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'b'*64
    old['review'] = review(data, 'APPROVED_SUPPORTED_CHARACTERIZATION')
    data['cadence_source_sha256'] = live.cadence_source_hashes(data)
    data['review'] = review(data, 'APPROVED_HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD')
    capture = docs['local_reference_capture']
    capture.update(approved_for_runtime=False, motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT',
                   stop_state='UNVERIFIED_BY_READ_ONLY_PROTOCOL')
    for mid in live.IDS:
        clock = 2_000_000_000+int(mid)*1_000_000
        capture['identities'][mid].update(request_monotonic_ns=clock, reply_monotonic_ns=clock+1000)
        row = capture['telemetry']['rows'][mid]
        row.update(current=0., voltage=40.)
        q = row['median_position_rad']
        row['position_samples'] = [dict(rad=q, request_monotonic_ns=clock+2000+i*2000,
            reply_monotonic_ns=clock+3000+i*2000) for i in range(3)]
        watchdog = docs['command_loss_report']['axes'][mid]
        watchdog['version']['request_start_ns'] = 3_000_000_000+int(mid)*1000
        watchdog['stop_probe']['received_ns'] = 4_000_000_000+int(mid)*1000
    scalar = dict(schema='native-step-scalar-file-only-v1', status='PASS_FILE_ONLY_COMPARE',
        baseline_manifest_sha256=data['artifacts']['model_manifest']['sha256'],
        hardware_opened=False, output_allowed=False, approved_for_runtime=False, live_50hz_verified=False)
    docs['scalar_step_manifest'] = scalar
    data['artifacts']['scalar_step_manifest'] = _write(base/'scalar_step_manifest.json', scalar)
    old['artifacts']['scalar_step_manifest'] = copy.deepcopy(data['artifacts']['scalar_step_manifest'])
    old['artifacts'] = {name:copy.deepcopy(old['artifacts'].get(name,
        dict(path='SYNTHETIC-historical-'+name+'.json',sha256='0'*64)))
        for name in live.artifact_names(old)}
    diag = docs['pipeline_diagnostic']
    diag.update(boot_id=data['boot_id'], motor_power_epoch=data['motor_power_epoch'],
        cadence_source_sha256=copy.deepcopy(data['cadence_source_sha256']), approved_for_runtime=False,
        imu_restore_status='restored', cycles_completed=501, cycles_requested=501,
        model_source=dict(manifest_sha256=data['artifacts']['scalar_step_manifest']['sha256'],
            baseline_provenance={'manifest_sha256':data['artifacts']['model_manifest']['sha256']}))
    diag['plan'].update(startup_cycle_allowance=1, steady_cycles_requested=500,
        absolute_epoch_cadence=True, v3_voltage_overlap=True, v3_voltage_validation_overlap=True)
    diag['observer'].update(ticks_completed=501, ticks_requested=501)
    seed = diag['measurements'][0]
    relative = {k:v-seed['release_ns'] for k,v in seed.items() if k.endswith('_ns')}
    epoch = 5_000_000_000
    diag['measurements'] = []
    for index in range(501):
        begin = epoch+index*20_000_000
        row = {**seed, **{k:begin+v for k,v in relative.items()},
            'timing_phase':'steady' if index else 'startup', 'cadence_slot':index,
            'scheduled_release_ns':begin, 'skipped_slots_before':0}
        diag['measurements'].append(row)
    diag['absolute_epoch_schedule'] = dict(enabled=True, epoch_ns=epoch, period_ns=20_000_000)
    prior_report = dict(profile_sha256=None, boot_id=old['boot_id'], motor_power_epoch=old['motor_power_epoch'],
        cadence_source_sha256=copy.deepcopy(old['cadence_source_sha256']),
        status='COMPLETE_SUPPORTED_OUTPUT', errors=[], normal_ramp_completed=True, stop_confirmed=True,
        current_position_hold_only=True, cyclic_inference_skipped=True, actual_model_calls=0,
        learned_targets_sent=False, deadline20ms_misses=0, startup_20ms_misses=0,
        steady_deadline20ms_misses=0, stop_reports={}, cycles=[])
    for index in range(138):
        begin = 1_000_000_000+index*20_000_000
        prior_report['cycles'].append(dict(index=index, begin_ns=begin,
            output_reply_end_ns=begin+18_000_000, end_ns=begin+19_000_000,
            deadline20ms_missed=False,
            phase='starting' if not index else 'stopped' if index==137 else 'active'))
    for bus, ids in (('front',list(range(1,7))), ('rear',list(range(7,13)))):
        prior_report['stop_reports'][bus] = dict(complete=True, confirmed_ids=ids,
            unconfirmed_ids=[], ambiguous_ids=[], fault_by_id={str(mid):0 for mid in ids})
    observed = dict(report_sha256=None, observed_by='operator', audio_heard=True,
        abnormal_noise_vibration_slip_sinking_contact=False, box_support_maintained=True,
        autonomous_standing_or_walking_observed=False)
    source = dict(schema='singularitydog.human-supported-source-review.v1', scope=data['scope'],
        prior_profile_sha256=None, changes=[], new_source=dict(path=live._HUMAN_SUPPORTED_NEW_SOURCE,
            sha256=data['cadence_source_sha256'][live._HUMAN_SUPPORTED_NEW_SOURCE]),
        review=review(data,'ACCEPT_HUMAN_SUPPORTED_PARTIAL_SOURCE_DELTA'))
    receipts = {}
    for kind in ('power','pose','rehearsal','video','clearance','physical_observation'):
        receipts[kind] = dict(schema='singularitydog.human-supported-operator-receipt.v1',
            kind=kind, observed_by='operator', source_message_id='SYNTHETIC-'+kind,
            user_statement='SYNTHETIC ONLY, no physical observation.',
            boot_id=data['boot_id'], motor_power_epoch=data['motor_power_epoch'],
            review=review(data,'ACCEPT_HUMAN_SUPPORTED_OPERATOR_RECEIPT'))
    receipts['power'].update(power_epoch_origin='operator_statement', off_on_confirmed=True,
        motor_power_on=True, no_power_operation_since_capture=True)
    receipts['pose'].update(capture_sha256=None, pose_kind='human_full_support', box_removed=True,
        operator_count=2, full_body_weight_supported=True, all_four_paws_on_floor=True,
        all_axes_simultaneously_stationary=True, continuous_body_catch=True, hands_remain_on_body=True,
        legs_and_wiring_contact_free=True)
    receipts['rehearsal'].update(operator_count=2, motor_power_off=True,
        body_full_support_continuous=True, box_removed_and_restored=True, cutoff_role_maintained=True,
        abnormal_noise_vibration_slip_sinking_contact=False)
    receipts['video'].update(side_view_recording_ready=True, camera_fixed=True,
        body_four_paws_and_supporting_hands_visible=True)
    receipts['clearance'].update(capture_sha256=None, selected_ids=list(range(1,13)),
        local_clearance_rad=math.radians(3), legs_and_wiring_contact_free=True,
        pose_maintained_since_capture=True, immediate_40v_cutoff_ready=True)
    receipts['physical_observation'].update(report_sha256=None, audio_heard=True,
        abnormal_noise_vibration_slip_sinking_contact=False, human_full_support_maintained=True,
        pose_maintained_since_capture=True)
    prep = dict(schema='singularitydog.human-supported-preparation.v1', scope=data['scope'],
        settings=copy.deepcopy(data['human_supported_hold']), boot_id=data['boot_id'],
        motor_power_epoch=data['motor_power_epoch'], uids_by_id={mid:a['uid'] for mid,a in data['axes'].items()},
        local_reference_capture_sha256=None, command_loss_report_sha256=None, pipeline_diagnostic_sha256=None,
        reviewed_settings_sha256=None, fixed_catch_authorized=False, ground_progression_allowed=False,
        absolute_calibration_certified=False, dynamic_feedback_certified=False, source_receipts={},
        review=review(data,'ACCEPT_HUMAN_SUPPORTED_PARTIAL_PREPARATION'))
    audio = dict(schema='singularitydog.human-supported-audio-manifest.v1', scope=data['scope'],
        acceptance=data['diagnostic_timing_acceptance'], prepare_ease_is_not_go=True, go_is_short_tone=True,
        resupport_starts_with_urgent_tone=True, operator_must_resupport_before_voice_finishes=True,
        audio_process_completion_is_not_proof_of_audibility=True,
        physical_ease_duration_not_verified_by_audio=True, clips={},
        review=review(data,'ACCEPT_HUMAN_SUPPORTED_SPOKEN_CUES'))
    for stage in ('brief','prepare_ease','go','resupport','abort'):
        wav = base/(stage+'.wav'); duration = .1 if stage=='go' else .2
        with wave.open(str(wav),'wb') as writer:
            writer.setnchannels(2); writer.setsampwidth(2); writer.setframerate(48000)
            writer.writeframes(b'\0'*int(duration*48000)*4)
        audio['clips'][stage] = dict(path=wav.name, sha256=hashlib.sha256(wav.read_bytes()).hexdigest(),
            duration_s=duration, transcript='SYNTHETIC ONLY: '+stage)
    docs.update(prior_current_hold_profile=old, prior_current_hold_report=prior_report,
        prior_current_hold_observation=observed, human_supported_source_review=source,
        human_supported_preparation=prep, human_supported_audio_manifest=audio)
    data['artifacts'] = {name:copy.deepcopy(data['artifacts'].get(name,dict(path=name+'.json',sha256='0'*64)))
                        for name in live.artifact_names(data)}
    hardware = docs['hardware_review']
    hardware.update(scope=data['scope'], review=copy.deepcopy(data['review']))
    for mid in live.IDS:
        hardware['angles'][mid]['uncertainty_rad'] = None
    hardware['human_supported_partial_acceptance'] = dict(mode=data['diagnostic_timing_acceptance'],
        scope=data['scope'], settings=copy.deepcopy(data['human_supported_hold']),
        strict_current_hold_deadline=True, load_bearing_not_yet_observed=True, fixed_catch_authorized=False,
        ground_progression_allowed=False, artifact_sha256={},
        review=review(data,'ACCEPT_HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD'))
    acceptance = docs['operator_acceptance']
    acceptance.pop('box_must_remain')
    acceptance.update(schema='singularitydog.human-supported-hold-operator-acceptance.v1', scope=data['scope'],
        review=review(data,'ACCEPT_COMMAND_LOSS_ONLY_HUMAN_SUPPORTED_PARTIAL_TRIAL'),
        human_supported_hold_settings=copy.deepcopy(data['human_supported_hold']),
        continuous_body_catch_required=True, hands_remain_on_body_required=True,
        two_operators_required=True, fixed_catch_authorized=False)
    return data, docs, pins, receipts


def seal_human(base, data, docs, receipts):
    for name in ('calibration','mount','bias','model_manifest','scalar_step_manifest',
                 'pipeline_diagnostic','local_reference_capture','command_loss_report',
                 'human_supported_audio_manifest','prior_current_hold_profile'):
        data['artifacts'][name] = _write(base/(name+'.json'),docs[name])
    docs['prior_current_hold_report']['profile_sha256'] = data['artifacts']['prior_current_hold_profile']['sha256']
    data['artifacts']['prior_current_hold_report'] = _write(base/'prior_current_hold_report.json',docs['prior_current_hold_report'])
    docs['prior_current_hold_observation']['report_sha256'] = data['artifacts']['prior_current_hold_report']['sha256']
    data['artifacts']['prior_current_hold_observation'] = _write(base/'prior_current_hold_observation.json',docs['prior_current_hold_observation'])
    source = docs['human_supported_source_review']; old = docs['prior_current_hold_profile']['cadence_source_sha256']
    source.update(prior_profile_sha256=data['artifacts']['prior_current_hold_profile']['sha256'],
        changes=[dict(path=name,before_sha256=old[name],after_sha256=data['cadence_source_sha256'][name])
                 for name in sorted(old) if old[name]!=data['cadence_source_sha256'][name]])
    data['artifacts']['human_supported_source_review'] = _write(base/'human_supported_source_review.json',source)
    for name in ('pose','clearance'): receipts[name]['capture_sha256'] = data['artifacts']['local_reference_capture']['sha256']
    receipts['physical_observation']['report_sha256'] = data['artifacts']['command_loss_report']['sha256']
    prep = docs['human_supported_preparation']
    prep.update(local_reference_capture_sha256=data['artifacts']['local_reference_capture']['sha256'],
        command_loss_report_sha256=data['artifacts']['command_loss_report']['sha256'],
        pipeline_diagnostic_sha256=data['artifacts']['pipeline_diagnostic']['sha256'],
        reviewed_settings_sha256=live.reviewed_settings_sha256(data),
        source_receipts={name:_write(base/(name+'-receipt.json'),receipt) for name,receipt in receipts.items()})
    data['artifacts']['human_supported_preparation'] = _write(base/'human_supported_preparation.json',prep)
    docs['hardware_review']['human_supported_partial_acceptance']['artifact_sha256'] = {
        name:data['artifacts'][name]['sha256'] for name in (*live._HUMAN_SUPPORTED_ARTIFACTS,
            'local_reference_capture','command_loss_report','pipeline_diagnostic')}
    return seal_local(base,data,docs)


class HumanSupportedPartialProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name)
        self.data,self.docs,pins,self.receipts=human_fixture(self.base)
        self.enterContext(patch.object(live.shadow,'SOURCE_HASHES',pins))

    def load(self):
        seal_human(self.base,self.data,self.docs,self.receipts)
        return live.load_profile(self.base/'profile.json')

    def test_complete_separate_human_proof_is_current_hold_only(self):
        loaded=self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertTrue(live.current_position_hold_only(loaded))
        self.assertEqual(live.human_supported_partial_current_hold_settings(loaded),SETTINGS)
        self.assertTrue(loaded['human_body_catch_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertIsNone(live.fixed_catch_current_hold_settings(loaded))
        self.assertEqual(loaded['timing_review']['cycles'],501)
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])
        self.assertFalse(loaded['watchdog_by_id']['1']['usb_disconnect_test_passed'])

    def test_raw_or_mutated_loaded_profile_cannot_mint_proof(self):
        for method in (live.current_position_hold_only,live.human_supported_partial_current_hold_settings,
                       live.human_supported_audio_settings):
            with self.subTest(method=method),self.assertRaises(live.ProfileError):method(self.data)
        loaded=self.load()
        for mutate in (lambda p:p['axes']['1'].update(kp=6.001),
            lambda p:p['axes']['1'].update(physical_lower_rad=-2.),
            lambda p:p.update(motor_power_epoch='changed'),
            lambda p:p['_human_supported_audio']['clips']['go'].update(path='/tmp/forged.wav'),
            lambda p:p['artifacts']['command_loss_report'].update(sha256='f'*64)):
            altered=copy.deepcopy(loaded)
            # Opaque tokens retain identity across a caller's shallow copy.
            altered['_human_supported_token']=loaded['_human_supported_token']
            altered['_current_hold_token']=loaded['_current_hold_token']
            mutate(altered)
            with self.assertRaises(live.ProfileError):live.human_supported_partial_current_hold_settings(altered)

    def test_duration_gain_mix_allowance_and_fixed_receiver_cannot_expand(self):
        changes=(('duration_s',8.001),('duration_s',3.),('startup_duration_s',.999),
            ('policy_weight',.000001),('hard_cycle_ms',21.),('max_sample_age_ms',20.001),
            ('max_consecutive_20ms_misses',1),('scope','supported_characterization_only'),
            ('startup_cycle_allowance',live.FIRST_CYCLE_POST_REPLY),('startup_damping_duration_s',.08),
            ('post_reply_deadline_policy',{}),('fixed_catch',{'catch_gap_mm':40}))
        baseline=copy.deepcopy(self.data)
        for key,value in changes:
            self.data=copy.deepcopy(baseline);self.data[key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.load()
        self.data=baseline
        for key,value in (('kp',6.001),('kd',.151),('max_measured_velocity_rad_s',.251),
            ('max_estimated_pd_torque_nm',.201),('max_displacement_from_start_rad',math.radians(1.01))):
            saved=self.data['axes']['1'][key];self.data['axes']['1'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.load()
            self.data['axes']['1'][key]=saved

    def test_historical_source_sets_unchanged_new_supervisor_required_only_here(self):
        self.assertEqual(live.cadence_source_paths(),live.CADENCE_SOURCE_PATHS)
        for schema in (live.SCHEMA_V1,live.SCHEMA_V2,live.SCHEMA_V3):
            self.assertEqual(live.cadence_source_paths({'schema':schema}),live.CADENCE_SOURCE_PATHS)
        self.assertEqual(set(live.cadence_source_paths(self.data)),
                         {*live.CADENCE_SOURCE_PATHS,live._HUMAN_SUPPORTED_NEW_SOURCE})
        self.data['cadence_source_sha256'].pop(live._HUMAN_SUPPORTED_NEW_SOURCE)
        with self.assertRaises(live.ProfileError):self.load()

    def test_fresh_power_pose_full_catch_and_operator_roles_required(self):
        cases=(('power','power_epoch_origin','jetson_boot'),('power','off_on_confirmed',False),
            ('power','motor_power_epoch','old'),('pose','pose_kind','box_support'),
            ('pose','operator_count',1),('pose','hands_remain_on_body',False),
            ('pose','all_axes_simultaneously_stationary',False),('rehearsal','motor_power_off',False),
            ('rehearsal','cutoff_role_maintained',False),('video','camera_fixed',False),
            ('clearance','selected_ids',list(range(1,12))),('clearance','pose_maintained_since_capture',False),
            ('physical_observation','human_full_support_maintained',False),
            ('physical_observation','abnormal_noise_vibration_slip_sinking_contact',True))
        baseline=copy.deepcopy(self.receipts)
        for kind,key,value in cases:
            self.receipts=copy.deepcopy(baseline);self.receipts[kind][key]=value
            with self.subTest(kind=kind,key=key),self.assertRaises(live.ProfileError):self.load()

    def test_raw_capture_reply_order_and_watchdog_diagnostic_freshness_required(self):
        cases=(lambda d:d['local_reference_capture']['identities']['1'].update(reply_monotonic_ns=1),
            lambda d:d['local_reference_capture']['telemetry']['rows']['1']['position_samples'][1].update(request_monotonic_ns=1),
            lambda d:d['local_reference_capture']['telemetry']['rows']['1'].update(current=.01),
            lambda d:d['local_reference_capture'].update(stop_state='VERIFIED'),
            lambda d:d['command_loss_report']['axes']['1']['version'].update(request_start_ns=1),
            lambda d:d['pipeline_diagnostic'].update(motor_power_epoch='old'))
        baseline=copy.deepcopy(self.docs)
        for mutate in cases:
            self.docs=copy.deepcopy(baseline);mutate(self.docs)
            with self.assertRaises(live.ProfileError):self.load()

    def test_no_startup_or_steady_20ms_exception_and_no_hidden_age(self):
        report=self.docs['pipeline_diagnostic'];baseline=copy.deepcopy(report['measurements'])
        for index in (0,500):
            report['measurements']=copy.deepcopy(baseline)
            row=report['measurements'][index]
            row['cycle_end_ns']=row['release_ns']+20_000_001
            with self.subTest(index=index),self.assertRaisesRegex(live.ProfileError,'strict whole-cycle'):
                live._timing(report,self.data)
        report['measurements']=copy.deepcopy(baseline)
        row=report['measurements'][-1]
        row['oldest_input_start_ns']=row['release_ns']
        row['cycle_end_ns']=row['release_ns']+20_000_001
        with self.assertRaises(live.ProfileError):live._timing(report,self.data)

    def test_unknown_calibration_dynamic_flags_and_fixed_gap_not_promoted(self):
        for key in ('output_shaft_position_verified','velocity_scale_and_sign_verified','torque_interpretation_verified'):
            self.docs['hardware_review']['type2_dynamic']['1'][key]=True
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.load()
            self.docs['hardware_review']['type2_dynamic']['1'][key]=False
        self.docs['human_supported_preparation']['fixed_catch_authorized']=True
        with self.assertRaises(live.ProfileError):self.load()

    def test_prior_box_pose_power_or_partial_stop_cannot_substitute_fresh_human_proof(self):
        prior=self.docs['prior_current_hold_profile']
        prior['motor_power_epoch']=self.data['motor_power_epoch']
        with self.assertRaisesRegex(live.ProfileError,'separate human-pose power epoch'):self.load()
        prior['motor_power_epoch']='SYNTHETIC-prior-box-power'
        self.docs['prior_current_hold_report']['stop_reports']['rear']['confirmed_ids']=list(range(7,12))
        with self.assertRaisesRegex(live.ProfileError,'twelve fault-free STOP'):self.load()

    def test_audio_bytes_durations_semantics_and_reserve_are_pinned(self):
        manifest=self.docs['human_supported_audio_manifest'];go=manifest['clips']['go']
        go['duration_s']=.101
        with self.assertRaisesRegex(live.ProfileError,'duration differs'):self.load()
        go['duration_s']=.1
        manifest['prepare_ease_is_not_go']=False
        with self.assertRaisesRegex(live.ProfileError,'stage meaning'):self.load()
        manifest['prepare_ease_is_not_go']=True
        path=self.base/go['path'];path.write_bytes(path.read_bytes()+b'changed')
        with self.assertRaisesRegex(live.ProfileError,'SHA256 mismatch'):self.load()

    def test_audio_helper_detached_and_raw_profile_no_permission(self):
        loaded=self.load();audio=live.human_supported_audio_settings(loaded)
        self.assertEqual(set(audio['clips']),{'brief','prepare_ease','go','resupport','abort'})
        self.assertTrue(audio['physical_ease_duration_not_verified_by_audio'])
        audio['clips']['go']['path']='changed'
        self.assertNotEqual(live.human_supported_audio_settings(loaded)['clips']['go']['path'],'changed')

    def test_every_physical_source_requires_hash_and_named_review(self):
        for kind in self.receipts:
            decision=self.receipts[kind]['review']['decision']
            self.receipts[kind]['review']['decision']='APPROVED_SUPPORTED_CHARACTERIZATION'
            with self.subTest(kind=kind),self.assertRaises(live.ProfileError):self.load()
            self.receipts[kind]['review']['decision']=decision

    def test_new_source_delta_and_fresh_diagnostic_pins_cannot_be_omitted(self):
        manifest=self.docs['human_supported_source_review']
        manifest['new_source']['sha256']='f'*64
        with self.assertRaisesRegex(live.ProfileError,'executable delta'):self.load()
        manifest['new_source']['sha256']=self.data['cadence_source_sha256'][live._HUMAN_SUPPORTED_NEW_SOURCE]
        self.docs['pipeline_diagnostic']['cadence_source_sha256'].pop(live._HUMAN_SUPPORTED_NEW_SOURCE)
        with self.assertRaisesRegex(live.ProfileError,'exact execution sources'):self.load()

    def test_unrelated_source_change_is_not_admitted_by_named_review(self):
        old=self.docs['prior_current_hold_profile']['cadence_source_sha256']
        old['singularitydog_hw/can_readonly.py']='f'*64
        with self.assertRaisesRegex(live.ProfileError,'Unreviewable unrelated'):self.load()

    def test_audio_real_duration_must_fit_finite_reserve_and_go_cap(self):
        manifest=self.docs['human_supported_audio_manifest']
        for stage,duration in (('prepare_ease',6.),('go',.12002083333333334)):
            baseline=copy.deepcopy(manifest['clips'][stage])
            clip=manifest['clips'][stage];path=self.base/clip['path']
            with wave.open(str(path),'wb') as writer:
                writer.setnchannels(2);writer.setsampwidth(2);writer.setframerate(48000)
                writer.writeframes(b'\0'*round(duration*48000)*4)
            clip.update(duration_s=round(duration*48000)/48000,
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            expected='do not fit' if stage=='prepare_ease' else 'at most120ms'
            with self.subTest(stage=stage),self.assertRaisesRegex(live.ProfileError,expected):self.load()
            manifest['clips'][stage]=baseline
            # Recreate original exact bytes for the next independent mutation.
            with wave.open(str(path),'wb') as writer:
                writer.setnchannels(2);writer.setsampwidth(2);writer.setframerate(48000)
                writer.writeframes(b'\0'*round(baseline['duration_s']*48000)*4)

    def test_empty_or_truncated_audio_is_rejected_even_when_rehashed(self):
        clip=self.docs['human_supported_audio_manifest']['clips']['go'];path=self.base/clip['path']
        valid=path.read_bytes()
        for raw in (valid[:44],valid[:-4]):
            path.write_bytes(raw);clip['sha256']=hashlib.sha256(raw).hexdigest()
            with self.subTest(length=len(raw)),self.assertRaises(live.ProfileError):self.load()
        path.write_bytes(valid)

    def test_unapproved_complete_candidate_has_no_current_hold_or_audio_token(self):
        self.data.update(approved_for_supported_policy_output=False,review=None,
                         blockers=['SYNTHETIC ONLY: physical approvals absent'])
        seal_human(self.base,self.data,self.docs,self.receipts)
        plan=live.load_profile(self.base/'profile.json',require_approved=False)
        self.assertFalse(plan['output_allowed'])
        for method in (live.current_position_hold_only,live.human_supported_partial_current_hold_settings,
                       live.human_supported_audio_settings):
            with self.subTest(method=method),self.assertRaises(live.ProfileError):method(plan)


if __name__=='__main__':unittest.main()
