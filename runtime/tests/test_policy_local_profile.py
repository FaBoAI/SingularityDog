"""Synthetic relative characterization contracts; never physical robot evidence."""
import copy
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import synthetic_fixture, _write


def seal_local(base, data, docs):
    """Bind synthetic artifacts in dependency order without circular hashes."""
    for name, doc in docs.items():
        if name not in ('operator_acceptance', 'hardware_review'):
            data['artifacts'][name] = _write(base/(name+'.json'), doc)
    acceptance = docs['operator_acceptance']
    acceptance['reviewed_settings_sha256'] = profile.reviewed_settings_sha256(data)
    acceptance['artifact_sha256'] = {k: data['artifacts'][k]['sha256']
        for k in profile.artifact_names(data) if k not in ('operator_acceptance', 'hardware_review')}
    data['artifacts']['operator_acceptance'] = _write(base/'operator_acceptance.json', acceptance)
    hardware = docs['hardware_review']
    hardware['reviewed_settings_sha256'] = profile.reviewed_settings_sha256(data)
    hardware['artifact_sha256'] = {k: data['artifacts'][k]['sha256']
        for k in profile.artifact_names(data) if k != 'hardware_review'}
    data['artifacts']['hardware_review'] = _write(base/'hardware_review.json', hardware)
    return _write(base/'profile.json', data)


def local_fixture(base):
    data, docs, pins = synthetic_fixture(base)
    data.update(schema=profile.SCHEMA_V3, telemetry_cadence=profile.CADENCE_PRE_ENABLE,
        cadence_source_sha256=profile.cadence_source_hashes(),
        watchdog_review_policy=profile.COMMAND_LOSS_ONLY_SUPPORTED,
        local_characterization=profile.LOCAL_RELATIVE_SUPPORTED,
        duration_s=2., startup_duration_s=.4, policy_ramp_s=.4, stop_duration_s=.4,
        policy_weight=.005)
    capture = dict(schema='singularitydog.readonly-12-angle-capture.v1', status='RECORDED_REVIEW_REQUIRED',
        errors=[], motor_output_allowed=False, boot_id=data['boot_id'], identities={}, telemetry={'rows':{}})
    command_loss = dict(status='COMPLETE_COMMAND_LOSS_DIAGNOSTIC', errors=[], selected_ids=list(range(1,13)),
        configured_timeout_ms=200, watchdog_ticks=4000, stop_confirmed=True,
        positive_gain_sent=False, learned_targets_sent=False, usb_disconnect_tested=False,
        stop_reports={}, axes={}, boot_id=data['boot_id'], motor_power_epoch=data['motor_power_epoch'])
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        command_loss['stop_reports'][scope] = dict(complete=True, confirmed_ids=ids,
            unconfirmed_ids=[], ambiguous_ids=[], errors=[])
    hardware = docs['hardware_review']
    hardware['local_characterization'] = dict(schema='singularitydog.local-relative-review.v1',
        reference_turns_by_id={i:0 for i in profile.IDS}, operator_confirmed_local_clearance=True,
        local_clearance_rad=math.radians(3), absolute_zero_uncertainty_rad=None,
        absolute_calibration_not_certified=True, full_dynamic_feedback_not_certified=True)
    hardware['imu'].update(gravity_direction_verified=False,
        gravity_direction_compared_to_operator_level=True, absolute_gravity_error_bound_rad=None)
    for mid, a in data['axes'].items():
        q = -1. if int(mid)%3 == 1 else 0.
        a.update(uncertainty_rad=None, physical_lower_rad=q-math.radians(3),
            physical_upper_rad=q+math.radians(3), kp=3., kd=.15,
            max_command_velocity_rad_s=math.radians(1), max_command_acceleration_rad_s2=math.radians(5),
            max_tracking_error_rad=math.radians(2), max_measured_velocity_rad_s=.25,
            max_measured_torque_nm=1., max_estimated_pd_torque_nm=.1,
            max_temperature_c=45., max_displacement_from_start_rad=math.radians(1))
        capture['identities'][mid] = {'mcu_uid_hex': a['uid']}
        capture['telemetry']['rows'][mid] = dict(run_mode=0, position_samples=[{'rad':q}]*3,
            median_position_rad=q)
        hardware['angles'][mid] = {k:a[k] for k in
            ('sign','offset_rad','physical_lower_rad','physical_upper_rad','uncertainty_rad')}
        hardware['angles'][mid].update(zero_reference_recorded=True, sign_evidence_reviewed=True,
            relative_local_clearance_verified=True, power_cycle_branch_method_verified=True)
        hardware['type2_dynamic'][mid].update(output_shaft_position_verified=False,
            velocity_scale_and_sign_verified=False, torque_interpretation_verified=False, limited_trial_reviewed=True)
        hardware['device_watchdog'][mid].update(usb_disconnect_test_passed=False,
            max_observed_disable_ms=225.)
        command_loss['axes'][mid] = dict(uid=a['uid'], command_loss_tested=True,
            disabled_on_command_loss=True, usb_disconnect_tested=False, configured_timeout_ms=200,
            disable_reply_upper_bound_ms=225., version={'version_bytes_hex':'05001300'},
            disable_upper_bound_origin='last_zero_host_write_started_ns',
            stop_probe={'mode_state':0,'fault_bits':0}, watchdog_readback={'value':4000})
    acceptance = dict(schema='singularitydog.supported-trial-operator-acceptance.v1',
        scope=data['scope'], watchdog_review_policy=profile.COMMAND_LOSS_ONLY_SUPPORTED,
        review={**copy.deepcopy(data['review']), 'decision':'ACCEPT_COMMAND_LOSS_ONLY_SUPPORTED_TRIAL'},
        user_statement='SYNTHETIC ONLY: omit USB test for box-supported trial.',
        usb_disconnect_test_waived=True, box_must_remain=True, immediate_40v_cutoff_required=True,
        ground_progression_allowed=False, uids_by_id={mid:a['uid'] for mid,a in data['axes'].items()})
    docs.update(command_loss_report=command_loss, operator_acceptance=acceptance,
                local_reference_capture=capture)
    for name in ('command_loss_report', 'operator_acceptance', 'local_reference_capture'):
        data['artifacts'][name] = {'path':name+'.json','sha256':'0'*64}
    seal_local(base,data,docs)
    return data,docs,pins


class LocalProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name)
        self.data,self.docs,pins=local_fixture(self.base)
        self.enterContext(patch.object(profile.shadow,'SOURCE_HASHES',pins))

    def load(self):
        seal_local(self.base,self.data,self.docs)
        return profile.load_profile(self.base/'profile.json')

    def test_local_trial_preserves_unknown_accuracy_and_unverified_usb(self):
        loaded=self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])
        settings=profile.local_characterization_settings(loaded)
        self.assertEqual(settings['numerical_position_margin_rad'],2*25.14/65535)
        self.assertIsNone(settings['absolute_zero_uncertainty_rad'])
        self.assertFalse(loaded['watchdog_by_id']['1']['usb_disconnect_test_passed'])
        self.assertAlmostEqual(loaded['axes']['1']['lower_rad'],
            self.data['axes']['1']['physical_lower_rad']+settings['numerical_position_margin_rad'])
        self.assertFalse(self.docs['hardware_review']['type2_dynamic']['1']['velocity_scale_and_sign_verified'])

    def test_raw_profile_cannot_forge_local_proof(self):
        with self.assertRaisesRegex(profile.ProfileError,'loader proof'):
            profile.local_characterization_settings({**self.data,'_local_validation_token':True})
        self.data['_local_validation_token']='forged'
        seal_local(self.base,self.data,self.docs)
        with self.assertRaisesRegex(profile.ProfileError,'Unsupported profile fields'):
            profile.load_profile(self.base/'profile.json')

    def test_local_caps_remain_hard_and_absolute_uncertainty_cannot_be_invented(self):
        for key,value in (('kp',3.01),('kd',.151),('max_measured_torque_nm',1.01),
                ('max_tracking_error_rad',math.radians(2.1)),('uncertainty_rad',.001),
                ('max_displacement_from_start_rad',math.radians(1.1))):
            old=self.data['axes']['1'][key]; self.data['axes']['1'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.data['axes']['1'][key]=old
        for key,value in (('duration_s',3.01),('policy_weight',.011),('hard_cycle_ms',21),
                           ('max_consecutive_20ms_misses',1)):
            old=self.data[key];self.data[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.data[key]=old

    def test_missing_explicit_acceptance_or_uid_binding_rejected(self):
        a=self.docs['operator_acceptance']
        for key,value in (('usb_disconnect_test_waived',False),('box_must_remain',False),
                ('immediate_40v_cutoff_required',False),('ground_progression_allowed',True),('user_statement','')):
            old=a[key];a[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            a[key]=old
        a['uids_by_id']['1']='bad'
        with self.assertRaisesRegex(profile.ProfileError,'UID binding'):self.load()

    def test_command_loss_still_requires_all_ids_firmware_disable_and_stop(self):
        rpt=self.docs['command_loss_report']
        for key,value in (('disabled_on_command_loss',False),('disable_reply_upper_bound_ms',250.001),
                ('command_loss_tested',False),('configured_timeout_ms',199)):
            row=rpt['axes']['7'];old=row[key];row[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            row[key]=old
        rpt['axes']['7']['version']['version_bytes_hex']='00000000'
        with self.assertRaisesRegex(profile.ProfileError,'differs from measured'):self.load()
        rpt['axes']['7']['version']['version_bytes_hex']='05001300'
        rpt['stop_reports']['rear']['confirmed_ids'].pop()
        with self.assertRaisesRegex(profile.ProfileError,'STOP evidence'):self.load()

    def test_command_loss_current_epoch_and_conservative_bound_origin_required(self):
        rpt=self.docs['command_loss_report']
        for key in ('boot_id','motor_power_epoch'):
            old=rpt[key];rpt[key]='different'
            with self.subTest(key=key),self.assertRaisesRegex(profile.ProfileError,'current boot'):
                self.load()
            rpt[key]=old
        rpt['axes']['1']['disable_upper_bound_origin']='last_zero_host_write_finished_ns'
        with self.assertRaisesRegex(profile.ProfileError,'before the final zero-command write'):
            self.load()

    def test_local_bounds_require_exact_current_reference_and_clearance(self):
        local=self.docs['hardware_review']['local_characterization']
        local['operator_confirmed_local_clearance']=False
        with self.assertRaisesRegex(profile.ProfileError,'clearance review'):self.load()
        local['operator_confirmed_local_clearance']=True
        self.data['axes']['1']['physical_lower_rad']-=.001
        with self.assertRaisesRegex(profile.ProfileError,'Local bounds'):self.load()
        self.data['axes']['1']['physical_lower_rad']+=.001
        self.docs['local_reference_capture']['boot_id']='different'
        with self.assertRaisesRegex(profile.ProfileError,'Current-boot'):self.load()

    def test_local_scope_cannot_promote_to_full_or_ground_review(self):
        self.data['local_characterization']=None
        with self.assertRaises(profile.ProfileError):self.load()

    def test_waiver_does_not_waive_diagnostic_freshness_or_model_pins(self):
        row=self.docs['pipeline_diagnostic']['measurements'][-1]
        row['cycle_end_ns']=row['release_ns']+20_000_001
        with self.assertRaisesRegex(profile.ProfileError,'cycle/freshness'):self.load()

    def test_usb_waiver_is_not_a_claimed_usb_pass(self):
        self.docs['hardware_review']['device_watchdog']['1']['usb_disconnect_test_passed']=True
        with self.assertRaisesRegex(profile.ProfileError,'watchdog'):self.load()


if __name__=='__main__':unittest.main()
