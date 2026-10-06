"""Synthetic file-only contracts; fixture reviews are never hardware evidence."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw import can_readonly as codec


def _write(path, data):
    raw = json.dumps(data, sort_keys=True, allow_nan=False).encode()
    path.write_bytes(raw)
    return {'path': path.name, 'sha256': hashlib.sha256(raw).hexdigest()}


def synthetic_fixture(base):
    """All fake pins must be patched explicitly; cannot load against production pins."""
    data = profile.template()
    review = {'reviewer': 'SYNTHETIC UNIT TEST ONLY', 'reviewed_at': '2026-09-28T00:00:00+09:00',
              'decision': 'APPROVED_SUPPORTED_CHARACTERIZATION', 'rationale': 'Fixture, no real robot.'}
    data.update(approved_for_supported_policy_output=True, blockers=[], review=review,
                boot_id='12345678-1234-4234-9234-123456789abc', motor_power_epoch='synthetic-epoch',
                assembly_id='synthetic-assembly', bundle_path='bundle')
    bundle = base/'bundle'; bundle.mkdir()
    pins = {}
    for filename in profile.shadow.SOURCE_HASHES:
        raw = ('synthetic '+filename).encode(); (bundle/filename).write_bytes(raw)
        pins[filename] = hashlib.sha256(raw).hexdigest()
    identities = {mid: f'{int(mid):016x}' for mid in profile.IDS}
    defaults = dict(kp=6., kd=.15, max_command_velocity_rad_s=.1,
        max_command_acceleration_rad_s2=.5, max_tracking_error_rad=.1,
        max_measured_velocity_rad_s=.4, max_measured_torque_nm=1.8, max_temperature_c=50.,
        max_estimated_pd_torque_nm=1.2, max_displacement_from_start_rad=math.radians(5))
    for mid in profile.IDS:
        index = profile.shadow.CAN_ORDER.index(int(mid))
        data['axes'][mid] = dict(defaults, uid=identities[mid], sign=1, offset_rad=0., uncertainty_rad=.005,
            physical_lower_rad=profile.shadow.LOWER[index], physical_upper_rad=profile.shadow.UPPER[index])
    calibration = {'status': 'MANUAL_NOMINAL_CANDIDATES_ONLY', 'approved_for_runtime': False,
        'formula': 'q_model = sign * raw + offset; rad; no wrapping',
        'model_can_order_candidate': profile.shadow.CAN_ORDER, 'identities': identities,
        'candidates': [{'motor_id': int(mid), 'sign_candidate': 1, 'offset_candidate_rad': 0.} for mid in profile.IDS]}
    mount = {'schema_version': 1, 'status': 'IMU_MOUNT_CANDIDATE_ONLY', 'input_frame': 'sensor',
        'output_frame': 'body_x_forward_y_left_z_up', 'R_body_from_sensor': [[0,1,0],[1,0,0],[0,0,-1]],
        'raw_driver_axes_verified': False, 'approved_for_runtime': False, 'provenance': {'test': 'synthetic'}}
    bias = {'schema_version': 1, 'kind': 'fixed_mount_baseline', 'status': 'GYRO_BIAS_CANDIDATE',
        'frame': 'sensor', 'axis_order': ['x','y','z'], 'gyro_bias_candidate_eligible': True,
        'operator_confirmed_stationary': True, 'approved_for_runtime': False, 'automatically_applied': False,
        'mount_rotation_applied': False, 'gyro_bias_candidate_rad_s': [0.,0.,0.],
        'captures': {'a': {'gyro_mean_rad_s': [0.,0.,0.]}},
        'provenance': {'a': {'summary_sha256': '1'*64, 'events_sha256': '2'*64},
                       'b': {'summary_sha256': '3'*64, 'events_sha256': '4'*64}}}
    manifest = {'schema': 'native-policy-overnight-v1', 'status': 'VALIDATED_FILE_ONLY',
        'bundle_hashes': pins, 'output_allowed': False, 'approved_for_runtime': False, 'live_50hz_verified': False}
    documents = dict(calibration=calibration, mount=mount, bias=bias, model_manifest=manifest)
    for name, doc in documents.items():
        data['artifacts'][name] = _write(base/(name+'.json'), doc)
    rows = []
    for i in range(20):
        start = 1_000_000_000+i*20_000_000
        names = ('release_ns', 'oldest_input_start_ns', 'input_latest_reply_ns', 'gather_end_ns',
                 'prepare_end_ns', 'infer_end_ns', 'final_host_write_ns', 'last_proxy_reply_ns', 'cycle_end_ns')
        values = (0, 1, 8_000_000, 9_000_000, 10_000_000, 12_000_000, 15_000_000, 16_000_000, 17_000_000)
        rows.append(dict(zip(names, [start+n for n in values]), learned_targets_sent=False,
                         host_write_is_can_wire_completion=False))
    timing = {'status': 'COMPLETE_DIAGNOSTIC', 'mode': 'stop-proxy', 'errors': [],
        'plan': {'request_gap_us': 600, 'window': 3},
        'motor_enable_sent': False, 'learned_targets_sent': False, 'full_controller_50Hz_verified': False,
        'observer': {'status': 'COMPLETE_NO_OUTPUT_DIAGNOSTIC', 'failure': None, 'incomplete': False,
                     'ticks_completed': 20, 'ticks_requested': 20, 'h_hypothesis': 0., 'output_allowed': False},
        'cycles_completed': 20, 'cycles_requested': 20, 'measurements': rows,
        'input_sha256': {'calibration': data['artifacts']['calibration']['sha256'],
                         'mount': data['artifacts']['mount']['sha256'], 'gyro_bias': data['artifacts']['bias']['sha256']},
        'model_source': {'manifest_sha256': data['artifacts']['model_manifest']['sha256']}}
    data['artifacts']['pipeline_diagnostic'] = _write(base/'pipeline_diagnostic.json', timing)
    docsources = [_write(base/'synthetic-capture.json', {'synthetic_fixture': True})]
    hardware = {'schema': profile.REVIEW_SCHEMA, 'scope': data['scope'], 'review': copy.deepcopy(review),
        'assembly_id': data['assembly_id'], 'uids_by_id': identities,
        'reviewed_settings_sha256': profile.reviewed_settings_sha256(data),
        'artifact_sha256': {k: data['artifacts'][k]['sha256'] for k in profile.ARTIFACTS if k != 'hardware_review'},
        'source_captures': docsources, 'angles': {}, 'type2_dynamic': {}, 'device_watchdog': {},
        'imu': dict.fromkeys(('right_handed_mount_physically_verified','nose_up_verified','left_up_verified',
                             'yaw_left_verified','gyro_bias_independent_stationary_validation','gravity_direction_verified'), True),
        'mode0_readback_required_before_enable': True, 'timing_budget_rationale': 'Synthetic budget.'}
    hardware['imu'].update(gravity_direction_max_error_rad=.01, corrected_static_gyro_max_rad_s=.001,
                           raw_gravity_norm_min_m_s2=9.7, raw_gravity_norm_max_m_s2=9.9,
                           norm_deviation_rationale='Synthetic independent references; no fitted accel bias.')
    for mid, row in data['axes'].items():
        hardware['angles'][mid] = {k: row[k] for k in ('sign','offset_rad','physical_lower_rad','physical_upper_rad','uncertainty_rad')}
        hardware['angles'][mid].update(zero_and_sign_physically_verified=True, physical_range_and_clearance_verified=True,
                                       power_cycle_branch_method_verified=True)
        hardware['type2_dynamic'][mid] = dict(output_shaft_position_verified=True, velocity_scale_and_sign_verified=True,
            torque_interpretation_verified=True, position_range_rad=[-12.57,12.57], velocity_range_rad_s=[-50.,50.], torque_range_nm=[-5.5,5.5])
        hardware['device_watchdog'][mid] = dict(motor_model='RS05', actual_command_loss_test_passed=True,
            usb_disconnect_test_passed=True, disabled_after_loss_verified=True, configured_timeout_ms=200.,
            max_observed_disable_ms=180., firmware_version=None,version_bytes_hex='05001300')
    data['artifacts']['hardware_review'] = _write(base/'hardware_review.json', hardware)
    documents.update(pipeline_diagnostic=timing, hardware_review=hardware)
    _write(base/'profile.json', data)
    return data, documents, pins


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.data, self.docs, pins = synthetic_fixture(self.base)
        self.enterContext(patch.object(profile.shadow, 'SOURCE_HASHES', pins))

    def save(self, *, bind_review=False):
        for name, doc in self.docs.items():
            if name != 'hardware_review':
                self.data['artifacts'][name] = _write(self.base/(name+'.json'), doc)
        if bind_review:
            review = self.docs['hardware_review']
            review['reviewed_settings_sha256'] = profile.reviewed_settings_sha256(self.data)
            review['artifact_sha256'] = {k: self.data['artifacts'][k]['sha256'] for k in profile.artifact_names(self.data) if k != 'hardware_review'}
        self.data['artifacts']['hardware_review'] = _write(self.base/'hardware_review.json', self.docs['hardware_review'])
        _write(self.base/'profile.json', self.data)
        return self.base/'profile.json'

    def test_complete_review_resolves_margin_without_claiming_active20ms(self):
        parsed = profile.load_profile(self.base/'profile.json')
        self.assertTrue(parsed['output_allowed'])
        self.assertTrue(parsed['support_must_remain'])
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertAlmostEqual(parsed['axes']['1']['lower_rad'], -2.195)
        self.assertEqual(parsed['watchdog_by_id']['7']['configured_timeout_ms'], 200.)
        self.assertEqual(parsed['watchdog_by_id']['7']['version_bytes_hex'],'05001300')
        self.assertIsNone(parsed['watchdog_by_id']['7']['firmware_version'])
        self.assertNotIn('lower_rad', self.data['axes']['1'])

    def test_experimental_guard_diagnostic_cannot_qualify_default_active_guard(self):
        for location in ('report', 'plan'):
            with self.subTest(location=location):
                target = self.docs['pipeline_diagnostic']
                if location == 'plan': target = target['plan']
                target['sourced_boot_guard'] = {'scope': 'disabled_stop_proxy_diagnostic_only'}
                with self.assertRaisesRegex(profile.ProfileError, 'Experimental boot-guard'):
                    profile.load_profile(self.save(bind_review=True))
                del target['sourced_boot_guard']

    def test_experimental_trace_copy_cannot_qualify_default_active_controller(self):
        for location in ('report', 'plan'):
            with self.subTest(location=location):
                target = self.docs['pipeline_diagnostic']
                if location == 'plan': target = target['plan']
                target['trace_copy_provenance'] = {'scope': 'disabled_stop_proxy_diagnostic_only'}
                with self.assertRaisesRegex(profile.ProfileError, 'Experimental trace-copy'):
                    profile.load_profile(self.save(bind_review=True))
                del target['trace_copy_provenance']

    def test_full_charge_voltage_requires_matching_diagnostic_and_review(self):
        self.select_scalar()
        self.assertEqual(profile.template()['voltage_max_v'], 42.)
        self.data['voltage_max_v'] = 43.
        with self.assertRaisesRegex(profile.ProfileError, 'Diagnostic voltage envelope'):
            profile.load_profile(self.save(bind_review=True))
        self.docs['pipeline_diagnostic']['plan']['voltage_max_v'] = 43.
        with self.assertRaisesRegex(profile.ProfileError, 'Diagnostic voltage range'):
            profile.load_profile(self.save(bind_review=True))
        self.docs['pipeline_diagnostic']['plan']['voltage_range_v'] = [35., 43.]
        self.docs['pipeline_diagnostic']['voltage_range_v'] = [35., 43.]
        self.docs['pipeline_diagnostic']['voltage_max_v'] = 43.
        self.assertTrue(profile.load_profile(self.save(bind_review=True))['output_allowed'])
        self.data['voltage_max_v'] = 43.001
        with self.assertRaisesRegex(profile.ProfileError, 'voltage_max_v'):
            profile.load_profile(self.save(bind_review=True))

    def test_legacy_voltage_ceiling_remains_42(self):
        self.data['voltage_max_v'] = 43.
        with self.assertRaisesRegex(profile.ProfileError, 'voltage_max_v'):
            profile.load_profile(self.save(bind_review=True))

    def select_scalar(self):
        self.data.update(schema=profile.SCHEMA_V3, telemetry_cadence=profile.CADENCE_PRE_ENABLE,
            cadence_source_sha256=profile.cadence_source_hashes(), model_backend=profile.SCALAR_BACKEND,
            voltage_overlap=True)
        scalar={'schema':'native-step-scalar-file-only-v1','status':'PASS_FILE_ONLY_COMPARE',
            'baseline_manifest_sha256':self.data['artifacts']['model_manifest']['sha256'],
            **dict.fromkeys(('hardware_opened','output_allowed','approved_for_runtime','live_50hz_verified'),False)}
        self.docs['scalar_step_manifest']=scalar
        self.data['artifacts']['scalar_step_manifest']=_write(self.base/'scalar_step_manifest.json',scalar)
        self.docs['pipeline_diagnostic']['model_source']={
            'manifest_sha256':self.data['artifacts']['scalar_step_manifest']['sha256'],
            'baseline_provenance':{'manifest_sha256':self.data['artifacts']['model_manifest']['sha256']}}
        self.docs['pipeline_diagnostic']['plan'].update(v3_voltage_overlap=True,
                                                        v3_voltage_validation_overlap=True)

    def test_scalar_and_overlap_are_explicit_review_bound_v3_selections(self):
        legacy=profile.execution_settings(self.data)
        self.assertEqual(legacy,{'model_backend':'native_baseline','voltage_overlap':False,
                                 'voltage_pipeline':False,
                                 'diagnostic_timing_acceptance':None})
        self.select_scalar()
        with self.assertRaisesRegex(profile.ProfileError,'exact gains, limits'):
            profile.load_profile(self.save())
        parsed=profile.load_profile(self.save(bind_review=True))
        self.assertEqual(profile.execution_settings(parsed)['model_backend'],profile.SCALAR_BACKEND)
        self.assertTrue(profile.execution_settings(parsed)['voltage_overlap'])
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertFalse(self.docs['scalar_step_manifest']['approved_for_runtime'])
        old=profile.reviewed_settings_sha256(parsed)
        parsed['voltage_overlap']=False
        self.assertNotEqual(profile.reviewed_settings_sha256(parsed),old)

    def test_scalar_requires_matching_diagnostic_and_baseline_pins(self):
        self.select_scalar()
        self.docs['pipeline_diagnostic']['model_source']['baseline_provenance']['manifest_sha256']='f'*64
        with self.assertRaisesRegex(profile.ProfileError,'Scalar timing baseline'):
            profile.load_profile(self.save(bind_review=True))
        self.docs['pipeline_diagnostic']['model_source']['baseline_provenance']['manifest_sha256']=self.data['artifacts']['model_manifest']['sha256']
        self.docs['pipeline_diagnostic']['model_source']['manifest_sha256']='f'*64
        with self.assertRaisesRegex(profile.ProfileError,'model-manifest mismatch'):
            profile.load_profile(self.save(bind_review=True))

    def test_scalar_artifact_review_and_original_flags_cannot_be_bypassed(self):
        self.select_scalar()
        path=self.save(bind_review=True)
        self.docs['hardware_review']['artifact_sha256'].pop('scalar_step_manifest')
        with self.assertRaisesRegex(profile.ProfileError,'all exact input artifacts'):
            profile.load_profile(self.save())
        self.docs['scalar_step_manifest']['approved_for_runtime']=True
        with self.assertRaisesRegex(profile.ProfileError,'retain file-only provenance'):
            profile.load_profile(self.save(bind_review=True))

    def test_native_batch_encoder_requires_pinned_binary_and_separate_review(self):
        self.select_scalar()
        binary = self.base/'bundle'/'native_batch.so'
        binary.write_bytes(b'synthetic extension bytes; never loaded')
        digest = hashlib.sha256(binary.read_bytes()).hexdigest()
        self.data['native_batch_encoder'] = {'path': binary.name, 'sha256': digest}
        with self.assertRaisesRegex(profile.ProfileError, 'native batch encoder acceptance'):
            profile.load_profile(self.save(bind_review=True))
        self.docs['hardware_review']['native_batch_encoder_acceptance'] = {
            'binary_sha256': digest, 'scope': self.data['scope'],
            'hard_output_and_freshness_limits_unchanged': True,
            'review': {'reviewer': 'synthetic test',
                       'reviewed_at': '2026-09-29T00:00:00+00:00',
                       'decision': 'ACCEPT_NATIVE_BATCH_ENCODER',
                       'rationale': 'Synthetic file-only fixture, never a motor approval.'},
        }
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertEqual(parsed['_native_batch_encoder_path'], str(binary))
        self.assertEqual(parsed['native_batch_encoder']['path'], binary.name)
        binary.write_bytes(b'changed extension bytes')
        with self.assertRaisesRegex(profile.ProfileError, 'Pinned native batch encoder binary'):
            profile.load_profile(self.save(bind_review=True))

    def test_native_batch_encoder_rejects_escape_and_unreviewed_selection(self):
        self.select_scalar()
        binary = self.base/'bundle'/'native_batch.so'
        binary.write_bytes(b'synthetic extension bytes')
        digest = hashlib.sha256(binary.read_bytes()).hexdigest()
        for name in ('../native_batch.so', '/tmp/native_batch.so', 'sub/native_batch.so'):
            self.data['native_batch_encoder'] = {'path': name, 'sha256': digest}
            with self.assertRaisesRegex(profile.ProfileError, 'bundle-relative'):
                profile.load_profile(self.save(bind_review=True))
        self.data['native_batch_encoder'] = {'path': binary.name, 'sha256': digest}
        old = profile.reviewed_settings_sha256(self.data)
        self.data['native_batch_encoder']['sha256'] = '0'*64
        self.assertNotEqual(profile.reviewed_settings_sha256(self.data), old)
        with self.assertRaisesRegex(profile.ProfileError, 'Pinned native batch encoder binary'):
            profile.load_profile(self.save(bind_review=True))

    def test_fast_choices_rejected_for_legacy_unknown_backend_and_nonboolean_overlap(self):
        for change in ({'model_backend':profile.SCALAR_BACKEND},{'voltage_overlap':True},
                       {'voltage_pipeline':True}):
            candidate=profile.template();candidate.update(change)
            _write(self.base/'bad-fast.json',candidate)
            with self.assertRaises(profile.ProfileError):
                profile.load_profile(self.base/'bad-fast.json',require_approved=False)
        for change in ({'model_backend':'unknown'},{'voltage_overlap':1},
                       {'voltage_pipeline':1},
                       {'diagnostic_timing_acceptance':'unlimited'}):
            candidate=profile.template(schema=profile.SCHEMA_V3);candidate.update(change)
            _write(self.base/'bad-fast.json',candidate)
            with self.assertRaises(profile.ProfileError):
                profile.load_profile(self.base/'bad-fast.json',require_approved=False)

    def test_voltage_pipeline_requires_distinct_diagnostic_and_acceptance(self):
        self.select_scalar()
        self.data['voltage_pipeline'] = True
        self.data['duration_s'] = 3.
        self.data['startup_duration_s'] = .5
        self.data['stop_duration_s'] = .5
        self.data['policy_ramp_s'] = .5
        with self.assertRaisesRegex(profile.ProfileError, 'own disabled diagnostic'):
            profile.load_profile(self.save(bind_review=True))
        self.docs['pipeline_diagnostic']['plan']['v3_voltage_pipeline'] = True
        with self.assertRaisesRegex(profile.ProfileError, 'own disabled diagnostic'):
            profile.load_profile(self.save(bind_review=True))
        del self.docs['pipeline_diagnostic']['plan']['v3_voltage_pipeline']
        self.docs['pipeline_diagnostic']['plan']['v3_voltage_fast_pipeline'] = True
        report = self.docs['pipeline_diagnostic']
        report['v3_voltage_overlap'] = {
            'enabled': True, 'validation_overlap_enabled': True,
            'voltage_dispatch_schedule': 'after_each_bus_feedback'}
        records = []
        for index, timing in enumerate(report['measurements']):
            release = timing['release_ns']
            fast = {
                'status': 'VALIDATED_BEFORE_PROXY_STOP', 'output_allowed': False,
                'range_v': [35., 42.], 'feedback_join_ns': timing['gather_end_ns'],
                'feedback_snapshot_validated_ns': release+9_100_000,
                'inference_end_ns': timing['infer_end_ns'],
                'voltage_join_ns': release+11_100_000,
                'post_inference_verified_ns': release+12_100_000,
                'voltage_verified_ns': release+12_200_000,
                'hard_deadline_ns': release+20_000_000,
                'feedback_dispatch_ns_by_bus': {'front': release+100, 'rear': release+100},
                'feedback_reply_end_ns_by_bus': {'front': release+8_000_000,
                                                 'rear': release+8_000_000},
                'feedback_ready_ns_by_bus': {'front': release+8_100_000,
                                              'rear': release+8_100_000},
                'voltage_dispatch_ns_by_bus': {'front': release+8_200_000,
                                                'rear': release+8_200_000},
                'voltage_reply_end_ns_by_bus': {'front': release+11_000_000,
                                                 'rear': release+11_000_000},
                'stop_reply_end_ns_by_bus': {'front': release+16_000_000,
                                              'rear': release+16_000_000},
                'stop_reply_count': 12,
                'stop_reply_verified_ns': release+16_100_000,
            }
            partitions = {}
            for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
                partitions[bus] = {
                    'acquired': {'records': [
                        {'start_ns': release+100,
                         'tx_hex': protocol.stop_request(
                             phase=protocol.TrialPhase.STOP, motor_id=axis).hex()}
                        for axis in ids]},
                    'voltage': {'records': [
                        {'start_ns': release+8_200_000,
                         'tx_hex': codec.read_request(tuple(ids)[index % 6], 'voltage').hex()}]},
                    'output': {'records': [
                        {'start_ns': release+13_000_000,
                         'received_ns': release+16_000_000,
                         'rx_hex': (b'AT'+((((2<<24)|(axis<<8)|0xfd)<<3)|4).to_bytes(4,'big')+
                                    b'\x08'+bytes(8)+b'\r\n').hex(),
                         'tx_hex': protocol.stop_request(
                             phase=protocol.TrialPhase.STOP, motor_id=axis).hex()}
                        for axis in ids]},
                }
            records.append({'cycle': index+1, 'voltage_fast_pipeline': fast,
                            'voltage_overlap': {'status': 'VALIDATED_BEFORE_PROXY_STOP'},
                            **{phase: {bus: part[phase] for bus, part in partitions.items()}
                               for phase in ('acquired', 'voltage', 'output')}})
        record_reference = _write(self.base/'records.json', records)
        report['v3_voltage_fast_pipeline'] = {
            'enabled': True, 'schema': 'immediate-feedback-voltage-proxy-v1',
            'period_ns': 20_000_000, 'voltage_dispatch_schedule': 'after_each_bus_feedback',
            'voltage_may_precede_global_feedback_validation': True,
            'voltage_verified_before_proxy_stop': True, 'diagnostic_only': True,
            'motor_output_allowed': False, 'learned_targets_sent': False,
            'active_feedback_safety_equivalent': False,
            'records_sha256': record_reference['sha256'],
        }
        with self.assertRaisesRegex(profile.ProfileError, 'voltage-pipeline acceptance'):
            profile.load_profile(self.save(bind_review=True))
        acceptance = {
            'pipeline': 'feedback_then_voltage.fast_v1',
            'diagnostic_sha256': self.data['artifacts']['pipeline_diagnostic']['sha256'],
            'scope': self.data['scope'],
            'hard_output_and_freshness_limits_unchanged': True,
            'review': {'reviewer': 'synthetic test', 'reviewed_at': '2026-09-29T00:00:00+00:00',
                       'decision': 'ACCEPT_FEEDBACK_THEN_VOLTAGE',
                       'rationale': 'Synthetic disabled diagnostic and bounded supported scope.'},
        }
        self.docs['hardware_review']['voltage_pipeline_acceptance'] = acceptance
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertTrue(profile.execution_settings(parsed)['voltage_pipeline'])
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        tampered = copy.deepcopy(records)
        tampered[0]['voltage_fast_pipeline']['voltage_dispatch_ns_by_bus']['front'] -= 2_000_000
        _write(self.base/'records.json', tampered)
        with self.assertRaisesRegex(profile.ProfileError, 'Artifact SHA256 mismatch'):
            profile.load_profile(self.save(bind_review=True))
        _write(self.base/'records.json', records)
        tampered = copy.deepcopy(records)
        tampered[0]['output']['front']['records'][0]['rx_hex'] = '00'
        report['v3_voltage_fast_pipeline']['records_sha256'] = _write(
            self.base/'records.json', tampered)['sha256']
        with self.assertRaisesRegex(profile.ProfileError, 'exact feedback, voltage and STOP replies'):
            profile.load_profile(self.save(bind_review=True))
        _write(self.base/'records.json', records)
        report['v3_voltage_fast_pipeline']['records_sha256'] = record_reference['sha256']
        self.docs['hardware_review']['voltage_pipeline_acceptance']['diagnostic_sha256'] = '0'*64
        with self.assertRaisesRegex(profile.ProfileError, 'voltage-pipeline acceptance'):
            profile.load_profile(self.save(bind_review=True))

    def test_voltage_pipeline_trace_cannot_be_replaced_by_plan_marker(self):
        self.select_scalar()
        self.data.update(voltage_pipeline=True, duration_s=3., startup_duration_s=.5,
                         stop_duration_s=.5, policy_ramp_s=.5)
        self.docs['pipeline_diagnostic']['plan']['v3_voltage_fast_pipeline'] = True
        with self.assertRaisesRegex(profile.ProfileError, 'disabled trace provenance'):
            profile.load_profile(self.save(bind_review=True))

    def test_new_timing_acceptance_never_covers_future_or_changed_report(self):
        self.select_scalar()
        self.data['diagnostic_timing_acceptance']=profile.OBSERVED_R17_TIMING
        with self.assertRaisesRegex(profile.ProfileError,'two original R17'):
            profile.load_profile(self.save(bind_review=True))

    def select_measured_r17(self):
        self.select_scalar()
        self.data['diagnostic_timing_acceptance']=profile.MEASURED_R17_STARTUP_TIMING
        report=self.docs['pipeline_diagnostic']
        report.update(boot_id=self.data['boot_id'],approved_for_runtime=False,
                      imu_restore_status='restored',cycles_completed=501,cycles_requested=501)
        report['plan'].update(startup_cycle_allowance=1,steady_cycles_requested=500,
                              absolute_epoch_cadence=True)
        report['observer'].update(ticks_completed=501,ticks_requested=501)
        seed=report['measurements'][0]
        epoch=seed['release_ns']
        rows=[]
        for index in range(501):
            lateness=(193_669 if index else 0)+(7_383 if index>=300 else 0)
            offset=index*20_000_000+lateness
            row={key:value+offset if key.endswith('_ns') else value for key,value in seed.items()}
            release=row['release_ns']
            row.update(timing_phase='startup' if index==0 else 'steady',
                       cadence_slot=index,scheduled_release_ns=epoch+index*20_000_000,
                       skipped_slots_before=0)
            row['last_proxy_reply_ns']=release+(19_318_095 if index==0 else 19_131_272)
            row['cycle_end_ns']=release+(20_140_131 if index==0 else 19_492_561)
            rows.append(row)
        report['measurements']=rows
        report['absolute_epoch_schedule']={'enabled':True,'epoch_ns':epoch,'period_ns':20_000_000,
            'slots_skipped':0,'diagnostic_only':True,'learned_targets_sent':False}

    def test_fresh_r17_measured_startup_accepts_exact_501_cycles(self):
        self.select_measured_r17()
        parsed=profile.load_profile(self.save(bind_review=True))
        timing=parsed['timing_review']
        self.assertEqual(timing['diagnostic_timing_acceptance'],profile.MEASURED_R17_STARTUP_TIMING)
        self.assertEqual(timing['cycles'],501)
        self.assertEqual(timing['startup_whole_iteration_ms'],20.140131)
        self.assertEqual(timing['twenty_ms_misses'],0)
        self.assertEqual(timing['release_intervals_over_21ms'],0)
        self.assertFalse(timing['strict_start_interval_20ms_met'])
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])

    def test_hold_probe_keeps_diagnostic_miss_visible_and_bounds_admission(self):
        self.select_measured_r17()
        self.data['diagnostic_timing_acceptance']=profile.CURRENT_HOLD_PROBE
        self.save(bind_review=True)
        report=self.docs['pipeline_diagnostic']
        def late(index,elapsed_ns):
            row=report['measurements'][index]
            row['cycle_end_ns']=row['release_ns']+elapsed_ns
            if index+1<len(report['measurements']):
                for key in ('release_ns','oldest_input_start_ns','input_latest_reply_ns',
                            'gather_end_ns','prepare_end_ns','infer_end_ns',
                            'final_host_write_ns','last_proxy_reply_ns','cycle_end_ns'):
                    report['measurements'][index+1][key]+=200_000
        late(500,20_100_000)
        timing=profile._timing(report,self.data)
        self.assertEqual(timing['kind'],'current_position_probe_admission_only')
        self.assertEqual(timing['twenty_ms_misses'],1)
        self.assertFalse(timing['actual_policy_output_20ms_verified'])
        late(500,21_000_001)
        with self.assertRaisesRegex(profile.ProfileError,'21ms bound'):
            profile._timing(report,self.data)
        late(500,20_100_000);late(450,20_100_000)
        with self.assertRaisesRegex(profile.ProfileError,'one miss per100'):
            profile._timing(report,self.data)

    def test_policy_probe_records_late_diagnostic_and_rejects_stale_inputs(self):
        self.select_measured_r17()
        self.save(bind_review=True)
        self.data['diagnostic_timing_acceptance']=profile.SUPPORTED_POLICY_PROBE
        report=self.docs['pipeline_diagnostic']
        row=report['measurements'][-1]
        row['cycle_end_ns']=row['release_ns']+20_300_000
        timing=profile._timing(report,self.data)
        self.assertEqual(timing['kind'],'supported_policy_probe_admission_only')
        self.assertEqual(timing['twenty_ms_misses'],1)
        self.assertFalse(timing['actual_policy_output_20ms_verified'])
        # Inference must still consume inputs no older than20ms even at
        # diagnostic admission, before the live hard20ms checks take over.
        row['infer_end_ns']=row['oldest_input_start_ns']+20_000_001
        row['final_host_write_ns']=row['infer_end_ns']+10_000
        row['last_proxy_reply_ns']=row['final_host_write_ns']+10_000
        row['cycle_end_ns']=row['last_proxy_reply_ns']+10_000
        with self.assertRaisesRegex(profile.ProfileError,'freshness budget'):
            profile._timing(report,self.data)

    def test_fresh_r17_requires_bound_schedule_and_steady_deadlines(self):
        self.select_measured_r17()
        good=copy.deepcopy(self.docs['pipeline_diagnostic'])
        def delay_row(report):
            row=report['measurements'][300]
            for key in ('release_ns','oldest_input_start_ns','input_latest_reply_ns',
                        'gather_end_ns','prepare_end_ns','infer_end_ns',
                        'final_host_write_ns','last_proxy_reply_ns','cycle_end_ns'):
                row[key]+=1_100_000
        def late_reply(report):
            row=report['measurements'][2]
            row['last_proxy_reply_ns']=row['oldest_input_start_ns']+20_000_001
            row['cycle_end_ns']=row['last_proxy_reply_ns']+100_000
        cases=(
            ('different boot',lambda r:r.update(boot_id='00000000-0000-4000-8000-000000000000'),
             'bound 1\\+500'),
            ('wrong steady count',lambda r:r['plan'].update(steady_cycles_requested=499),
             'bound 1\\+500'),
            ('second startup',lambda r:r['measurements'][1].update(timing_phase='startup'),
             'startup classification'),
            ('skipped slot',lambda r:r['measurements'][2].update(skipped_slots_before=1),
             'skipped a slot'),
            ('release gap',delay_row,
             'release-gap budget'),
            ('steady work',lambda r:r['measurements'][2].update(cycle_end_ns=
                r['measurements'][2]['release_ns']+20_000_001,whole_iteration_ms=1.,
                iteration_deadline_met=True),
             'scheduled, or STOP-reply deadline'),
            ('steady reply',late_reply,
             'STOP-reply deadline'),
            ('startup beyond allowance',lambda r:r['measurements'][0].update(cycle_end_ns=
                r['measurements'][0]['release_ns']+21_000_001),
             'startup, scheduled'),
            ('startup late reply',lambda r:r['measurements'][0].update(last_proxy_reply_ns=
                r['measurements'][0]['oldest_input_start_ns']+20_000_001),
             'STOP-reply deadline'),
            ('motor enabled',lambda r:r.update(motor_enable_sent=True),
             'Full real-input'),
        )
        for label,mutate,message in cases:
            with self.subTest(label=label):
                self.docs['pipeline_diagnostic']=copy.deepcopy(good)
                mutate(self.docs['pipeline_diagnostic'])
                with self.assertRaisesRegex(profile.ProfileError,message):
                    profile.load_profile(self.save(bind_review=True))

    def test_pinned_observed_timing_keeps_first_cycle_and_strict_cadence_visible(self):
        self.select_scalar()
        self.data['diagnostic_timing_acceptance']=profile.OBSERVED_R17_TIMING
        report=self.docs['pipeline_diagnostic']
        report['plan']['startup_cycle_allowance']=1
        seed=report['measurements'][0]
        rows=[]
        for index in range(501):
            offset=index*20_000_000+(650_000 if index else 0)+(1_566_252 if index>=349 else 0)
            row={key:value+offset if key.endswith('_ns') else value for key,value in seed.items()}
            row['timing_phase']='startup' if index==0 else 'steady'
            if index==0:row['cycle_end_ns']=row['release_ns']+20_604_890
            rows.append(row)
        report.update(measurements=rows,cycles_completed=501,cycles_requested=501)
        report['observer'].update(ticks_completed=501,ticks_requested=501)
        path=self.save(bind_review=True)
        digest=self.data['artifacts']['pipeline_diagnostic']['sha256']
        with patch.object(profile,'OBSERVED_R17_REPORT_SHA256',frozenset((digest,))):
            parsed=profile.load_profile(path)
        timing=parsed['timing_review']
        self.assertEqual(timing['cycles'],501)
        self.assertEqual(timing['twenty_ms_misses'],0)
        self.assertEqual(timing['startup_whole_iteration_ms'],20.60489)
        self.assertFalse(timing['strict_start_interval_20ms_met'])
        self.assertEqual(timing['release_intervals_over_21ms'],1)
        self.assertEqual(parsed['hard_cycle_ms'],20.)
        self.assertEqual(parsed['max_sample_age_ms'],20.)
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        report['measurements'][-1]['cycle_end_ns']=report['measurements'][-1]['release_ns']+20_000_001
        path=self.save(bind_review=True)
        with patch.object(profile,'OBSERVED_R17_REPORT_SHA256',
                          frozenset((self.data['artifacts']['pipeline_diagnostic']['sha256'],))):
            with self.assertRaisesRegex(profile.ProfileError,'cycle/freshness budget'):
                profile.load_profile(path)

    def test_overlap_requires_the_measured_worker_validation_variant(self):
        self.select_scalar()
        self.docs['pipeline_diagnostic']['plan']['v3_voltage_validation_overlap']=False
        with self.assertRaisesRegex(profile.ProfileError,'worker voltage validation'):
            profile.load_profile(self.save(bind_review=True))

    def test_bias_alias_is_supported_but_conflicting_bias_pin_is_rejected(self):
        bindings=self.docs['pipeline_diagnostic']['input_sha256']
        bindings['bias']=bindings.pop('gyro_bias')
        profile.load_profile(self.save(bind_review=True))
        bindings['gyro_bias']='f'*64
        with self.assertRaisesRegex(profile.ProfileError,'Timing input mismatch|Conflicting'):
            profile.load_profile(self.save(bind_review=True))

    def test_template_only_is_not_permission_to_output(self):
        _write(self.base/'template.json', profile.template())
        parsed = profile.load_profile(self.base/'template.json', require_approved=False)
        self.assertFalse(parsed['output_allowed'])
        with self.assertRaisesRegex(profile.ProfileError, 'unapproved'):
            profile.load_profile(self.base/'template.json')

    def test_v1_retains_original_settings_hash_and_implicit_pacing(self):
        self.data['schema'] = profile.SCHEMA_V1
        for key in profile.TRANSPORT_KEYS:
            del self.data[key]
        # Legacy timing artifacts did not record pacing; keep that contract.
        del self.docs['pipeline_diagnostic']['plan']
        legacy_digest = profile.reviewed_settings_sha256(self.data)
        self.assertEqual(legacy_digest, '3b6a74469e75c19f85dcdb275b375d8955ee7e8127e6c35db6072ad363b06ea4')
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertNotIn('request_gap_us', parsed)
        self.assertNotIn('request_window', parsed)
        self.assertEqual(profile.reviewed_settings_sha256(parsed), legacy_digest)
        self.assertEqual(profile.transport_settings(parsed), {
            'request_gap_us': 600, 'request_window': 3,
            'source_profile_schema': profile.SCHEMA_V1, 'emergency_stop_uses_same_gap': True})

    def test_v2_pacing_requires_matching_review_hash_without_claiming_active20ms(self):
        self.data.update(request_gap_us=800, request_window=2)
        self.docs['pipeline_diagnostic']['plan'].update(request_gap_us=800, window=2)
        with self.assertRaisesRegex(profile.ProfileError, 'exact gains, limits'):
            profile.load_profile(self.save())
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertEqual(profile.reviewed_settings_sha256(parsed),
                         self.docs['hardware_review']['reviewed_settings_sha256'])
        self.assertEqual(profile.reviewed_settings_sha256(parsed),
                         profile.reviewed_settings_sha256(self.data))
        self.assertEqual(profile.transport_settings(parsed)['request_gap_us'], 800)
        self.assertEqual(profile.transport_settings(parsed)['request_window'], 2)
        self.assertEqual(parsed['timing_review']['kind'], 'stop_proxy_diagnostic_only')
        self.assertFalse(parsed['timing_review']['actual_policy_output_20ms_verified'])
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertEqual(parsed['hard_cycle_ms'], 20.)
        self.assertEqual(parsed['max_consecutive_20ms_misses'], 0)

    def test_each_v2_pacing_field_changes_reviewed_settings_digest(self):
        original = profile.reviewed_settings_sha256(self.data)
        for key, value in (('request_gap_us', 700), ('request_window', 1)):
            changed = copy.deepcopy(self.data); changed[key] = value
            with self.subTest(key=key):
                self.assertNotEqual(profile.reviewed_settings_sha256(changed), original)

    def test_profile_pacing_schema_types_and_bounds_are_strict_even_in_plan(self):
        for key, values in (('request_gap_us', (True, 600., '600', None, 599, 5001)),
                            ('request_window', (True, 1., '1', None, 0, 4))):
            for value in values:
                bad = profile.template(); bad[key] = value
                with self.subTest(key=key, value=value):
                    _write(self.base/'bad-plan.json', bad)
                    with self.assertRaises(profile.ProfileError):
                        profile.load_profile(self.base/'bad-plan.json', require_approved=False)
            bad = profile.template(); del bad[key]
            with self.subTest(missing=key):
                _write(self.base/'bad-plan.json', bad)
                with self.assertRaisesRegex(profile.ProfileError, 'Unsupported profile fields'):
                    profile.load_profile(self.base/'bad-plan.json', require_approved=False)
        for schema in (profile.SCHEMA_V1, profile.SCHEMA_V2):
            bad = profile.template(schema=schema)
            bad['unknown_transport_field'] = 800
            _write(self.base/'bad-plan.json', bad)
            with self.subTest(unknown_schema=schema), self.assertRaisesRegex(profile.ProfileError, 'Unsupported profile fields'):
                profile.load_profile(self.base/'bad-plan.json', require_approved=False)
        bad = profile.template(schema=profile.SCHEMA_V1); bad['request_gap_us'] = 800
        _write(self.base/'bad-plan.json', bad)
        with self.assertRaisesRegex(profile.ProfileError, 'Unsupported profile fields'):
            profile.load_profile(self.base/'bad-plan.json', require_approved=False)

    def test_v2_pacing_boundaries_remain_unapproved(self):
        for gap, window in ((600, 1), (5000, 3)):
            candidate = profile.template(); candidate.update(request_gap_us=gap, request_window=window)
            _write(self.base/'boundary-plan.json', candidate)
            parsed = profile.load_profile(self.base/'boundary-plan.json', require_approved=False)
            with self.subTest(gap=gap, window=window):
                self.assertFalse(parsed['output_allowed'])
                self.assertIsNone(parsed['review'])
                self.assertEqual(profile.transport_settings(parsed)['request_gap_us'], gap)

    def test_v2_diagnostic_pacing_must_match_reviewed_values_and_types(self):
        for plan in (None, {}, {'request_gap_us': 800, 'window': 3},
                     {'request_gap_us': 600, 'window': 2},
                     {'request_gap_us': 600., 'window': 3},
                     {'request_gap_us': 600, 'window': True}):
            self.docs['pipeline_diagnostic']['plan'] = plan
            with self.subTest(plan=plan), self.assertRaisesRegex(profile.ProfileError, 'Diagnostic pacing'):
                profile.load_profile(self.save(bind_review=True))

    def test_old_semantic_only_and_malformed_raw_firmware_reviews_rejected(self):
        row=self.docs['hardware_review']['device_watchdog']['7']
        row['firmware_version']='0.5.0.13'
        for value in (None,'','0.5.0.13','050013','0500130000','AB001300',12345678,True):
            row['version_bytes_hex']=value
            with self.subTest(value=value),self.assertRaisesRegex(profile.ProfileError,'raw firmware'):
                profile.load_profile(self.save())
        del row['version_bytes_hex']
        with self.assertRaisesRegex(profile.ProfileError,'raw firmware'):
            profile.load_profile(self.save())

    def test_flipping_approval_does_not_promote_template(self):
        candidate = profile.template(); candidate['approved_for_supported_policy_output'] = True
        candidate['blockers'] = []
        _write(self.base/'template.json', candidate)
        with self.assertRaisesRegex(profile.ProfileError, 'Explicit named review'):
            profile.load_profile(self.base/'template.json')

    def test_candidate_file_stays_unapproved_after_review(self):
        profile.load_profile(self.base/'profile.json')
        self.assertFalse(json.loads((self.base/'mount.json').read_text())['approved_for_runtime'])

    def test_duplicate_nonfinite_and_bool_settings_rejected(self):
        original = (self.base/'profile.json').read_text()
        (self.base/'profile.json').write_text(original[:-1]+',"period_ms":20}')
        with self.assertRaisesRegex(profile.ProfileError, 'JSON'):
            profile.load_profile(self.base/'profile.json')
        for key, value in (('policy_weight', float('nan')), ('hard_cycle_ms', True), ('h_hypothesis', True)):
            bad = copy.deepcopy(self.data); bad[key] = value
            (self.base/'profile.json').write_text(json.dumps(bad))
            with self.assertRaises((ValueError, profile.ProfileError)):
                profile.load_profile(self.base/'profile.json')

    def test_model_bundle_and_artifact_tampering_rejected(self):
        (self.base/'bundle/model_149.pt').write_bytes(b'changed')
        with self.assertRaisesRegex(profile.ProfileError, 'bundle member'):
            profile.load_profile(self.base/'profile.json')

    def test_missing_hardware_capture_rejected(self):
        (self.base/'synthetic-capture.json').unlink()
        with self.assertRaisesRegex(profile.ProfileError, 'file required'):
            profile.load_profile(self.base/'profile.json')

    def test_uid_sign_and_offset_must_match_exact_candidate(self):
        for key, value in (('uid', 'f'*16), ('sign', -1), ('offset_rad', .1)):
            with self.subTest(key=key):
                old = self.data['axes']['10'][key]; self.data['axes']['10'][key] = value
                with self.assertRaisesRegex(profile.ProfileError, 'Calibrated'):
                    profile.load_profile(self.save())
                self.data['axes']['10'][key] = old

    def test_limits_cannot_disable_monitor_or_exceed_native_caps(self):
        for key, value in (('kd', 1.01), ('kp', 31), ('max_measured_torque_nm', 100),
                           ('max_temperature_c', 1000), ('max_command_acceleration_rad_s2', 0)):
            with self.subTest(key=key):
                old = self.data['axes']['2'][key]; self.data['axes']['2'][key] = value
                with self.assertRaisesRegex(profile.ProfileError, 'Out-of-scope'):
                    profile.load_profile(self.save())
                self.data['axes']['2'][key] = old

    def test_profile_changes_require_matching_settings_review(self):
        self.data['policy_weight'] = .2
        with self.assertRaisesRegex(profile.ProfileError, 'exact gains, limits'):
            profile.load_profile(self.save())

    def test_boundaries_and_acceleration_stop_reserve(self):
        self.data['axes']['2']['physical_upper_rad'] = 2.
        with self.assertRaisesRegex(profile.ProfileError, 'model range'):
            profile.load_profile(self.save())
        self.data['axes']['2']['physical_upper_rad'] = 1.2
        self.data['axes']['2']['max_command_acceleration_rad_s2'] = .01
        with self.assertRaisesRegex(profile.ProfileError, 'braking'):
            profile.load_profile(self.save())

    def test_watchdog_requires_real_loss_and_usb_disconnection(self):
        for key in ('actual_command_loss_test_passed', 'usb_disconnect_test_passed', 'disabled_after_loss_verified'):
            with self.subTest(key=key):
                self.docs['hardware_review']['device_watchdog']['7'][key] = False
                with self.assertRaisesRegex(profile.ProfileError, 'watchdog'):
                    profile.load_profile(self.save())
                self.docs['hardware_review']['device_watchdog']['7'][key] = True

    def test_static_type2_results_cannot_approve_dynamic_scale(self):
        self.docs['hardware_review']['type2_dynamic']['10']['velocity_scale_and_sign_verified'] = False
        with self.assertRaisesRegex(profile.ProfileError, 'Static-only'):
            profile.load_profile(self.save())

    def test_watchdog_evidence_must_match_native200ms_configuration(self):
        self.docs['hardware_review']['device_watchdog']['7']['configured_timeout_ms'] = 100.
        with self.assertRaisesRegex(profile.ProfileError, 'watchdog timeout'):
            profile.load_profile(self.save())

    def test_imu_missing_yaw_or_gravity_anomaly_is_not_autoaccepted(self):
        self.docs['hardware_review']['imu']['yaw_left_verified'] = False
        with self.assertRaisesRegex(profile.ProfileError, 'IMU direction'):
            profile.load_profile(self.save())
        self.docs['hardware_review']['imu']['yaw_left_verified'] = True
        self.docs['hardware_review']['imu']['raw_gravity_norm_max_m_s2'] = 10.7
        with self.assertRaisesRegex(profile.ProfileError, 'gravity norm outside'):
            profile.load_profile(self.save())

    def test_non_supported_scope_and_nonzero_walk_command_rejected(self):
        self.data['scope'] = 'walking'
        with self.assertRaisesRegex(profile.ProfileError, 'Only reviewed supported or fixed-catch'):
            profile.load_profile(self.save())
        self.data['scope'] = 'supported_characterization_only'; self.data['command'] = [.1,0.,0.]
        with self.assertRaisesRegex(profile.ProfileError, 'zero locomotion'):
            profile.load_profile(self.save())

    def test_diagnostic_needs_real_inference_and_full_12axis_send_scope(self):
        self.docs['pipeline_diagnostic']['observer'] = None
        with self.assertRaisesRegex(profile.ProfileError, 'Full real-input'):
            profile.load_profile(self.save(bind_review=True))

    def test_recomputed_timing_rejects_short_summary_hiding_slow_work(self):
        row = self.docs['pipeline_diagnostic']['measurements'][0]
        row['cycle_end_ns'] = row['release_ns']+25_000_000
        row['whole_iteration_ms'] = 1.
        with self.assertRaisesRegex(profile.ProfileError, 'cycle/freshness'):
            profile.load_profile(self.save(bind_review=True))

    def test_diagnostic_characterization_can_review_bounded20ms_miss(self):
        self.data.update(hard_cycle_ms=40., max_consecutive_20ms_misses=2,
                         max_sample_age_ms=40., max_sample_gap_ms=40.)
        rows = self.docs['pipeline_diagnostic']['measurements']
        rows[0]['cycle_end_ns'] = rows[0]['release_ns']+25_000_000
        for row in rows[1:]:
            for key in list(row):
                if key.endswith('_ns'): row[key] += 5_000_000
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertEqual(parsed['timing_review']['twenty_ms_misses'], 1)
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])

    def test_ordinary_sleep_jitter_not_a_perpetual20ms_work_failure(self):
        for i, row in enumerate(self.docs['pipeline_diagnostic']['measurements']):
            for key in list(row):
                if key.endswith('_ns'): row[key] += i*30_000
        parsed = profile.load_profile(self.save(bind_review=True))
        self.assertEqual(parsed['timing_review']['twenty_ms_misses'], 0)

    def test_default21ms_sample_gap_does_not_relax20ms_age_or_execution(self):
        parsed = profile.load_profile(self.base/'profile.json')
        self.assertEqual(parsed['max_sample_gap_ms'], 21.)
        self.assertEqual(parsed['max_sample_age_ms'], 20.)
        self.assertEqual(parsed['hard_cycle_ms'], 20.)
        for key, value in (('max_sample_gap_ms', 21.001), ('max_sample_age_ms', 20.001)):
            with self.subTest(key=key):
                old = self.data[key]; self.data[key] = value
                with self.assertRaisesRegex(profile.ProfileError, 'Out-of-scope'):
                    profile.load_profile(self.save())
                self.data[key] = old
        row = self.docs['pipeline_diagnostic']['measurements'][0]
        row['cycle_end_ns'] = row['release_ns']+20_010_000
        with self.assertRaisesRegex(profile.ProfileError, 'cycle/freshness'):
            profile.load_profile(self.save(bind_review=True))

    def test_overlapping_cycles_and_noncausal_timestamps_rejected(self):
        self.data.update(hard_cycle_ms=40., max_consecutive_20ms_misses=2)
        rows = self.docs['pipeline_diagnostic']['measurements']
        rows[0]['cycle_end_ns'] = rows[0]['release_ns']+25_000_000
        with self.assertRaisesRegex(profile.ProfileError, 'Overlapping'):
            profile.load_profile(self.save(bind_review=True))

    def test_start_pose_bounds_cannot_exceed_uncertainty_reduced_range(self):
        self.data['start_pose_bounds'] = {mid: [r['physical_lower_rad'], r['physical_upper_rad']]
                                         for mid, r in self.data['axes'].items()}
        with self.assertRaisesRegex(profile.ProfileError, 'Start-pose interval exceeds'):
            profile.load_profile(self.save(bind_review=True))

    def test_mode_readback_is_required_not_just_documented_assumption(self):
        self.docs['hardware_review']['mode0_readback_required_before_enable'] = False
        with self.assertRaisesRegex(profile.ProfileError, 'Mode0'):
            profile.load_profile(self.save())


class AccelerationHypothesisProfileTests(unittest.TestCase):
    """Mocked input audit exercises admission; it is never physical evidence."""
    select_scalar = ProfileTests.select_scalar

    def setUp(self):
        from test_policy_local_profile import local_fixture
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.data, self.docs, pins = local_fixture(self.base)
        self.enterContext(patch.object(profile.shadow, 'SOURCE_HASHES', pins))
        ProfileTests.select_measured_r17(self)
        self.data.update(diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_PROBE,
                         accel_input_hypothesis=True)
        self.data['cadence_source_sha256'] = profile.cadence_source_hashes(self.data)
        self.docs['accel_input_hypothesis'] = {'SYNTHETIC_UNIT_TEST_ONLY': True}
        self.data['artifacts']['accel_input_hypothesis'] = _write(
            self.base/'accel_input_hypothesis.json', self.docs['accel_input_hypothesis'])
        self.reference = {'path': str(self.base/'accel_input_hypothesis.json'),
                          'sha256': self.data['artifacts']['accel_input_hypothesis']['sha256']}
        self.provenance = {
            'kind': 'singularitydog.supported-accel-input-hypothesis.v1',
            'scope': 'boxed_small_mix_only', 'hypothesis_sha256': self.reference['sha256'],
            'candidate_sha256': '3'*64, 'manifest_sha256': '4'*64,
            'raw_norm_bounds_m_s2': [self.data['imu_accel_norm_min_m_s2'],
                                     self.data['imu_accel_norm_max_m_s2']],
            'formal_calibration_approved': False, 'absolute_orientation_error_bound_rad': None,
            'fit_and_independent_captures_reaudited': True, 'grants_motor_output': False}
        class FakeAuditedInput:
            def provenance(inner): return copy.deepcopy(self.provenance)
        self.loader = self.enterContext(patch.object(profile, '_load_accel_input_hypothesis',
                                                      return_value=FakeAuditedInput()))
        report = self.docs['pipeline_diagnostic']
        report.update(motor_power_epoch=self.data['motor_power_epoch'],
                      cadence_source_sha256=self.data['cadence_source_sha256'])
        report['plan']['accel_input_hypothesis'] = copy.deepcopy(self.reference)
        report['input_sha256']['accel_input_hypothesis'] = self.reference['sha256']
        report['observer']['accel_input_hypothesis'] = copy.deepcopy(self.provenance)

    def seal(self):
        from test_policy_local_profile import seal_local
        seal_local(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def load(self):
        return profile.load_profile(self.seal())

    def test_complete_load_binds_input_without_formal_calibration_or_ground_permission(self):
        loaded = self.load()
        self.assertEqual(profile.accel_input_hypothesis_settings(loaded), self.reference)
        self.loader.assert_called_once_with(self.reference, self.docs['mount']['R_body_from_sensor'])
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['formal_calibration_approved'])
        self.assertIsNone(loaded['_accel_input_hypothesis_provenance']['absolute_orientation_error_bound_rad'])
        result = profile.accel_input_hypothesis_settings(loaded)
        result['sha256'] = 'f'*64
        self.assertEqual(profile.accel_input_hypothesis_settings(loaded), self.reference)

    def test_selected_source_graph_pins_input_dependencies_without_changing_default_graph(self):
        original = profile.cadence_source_paths()
        self.assertEqual(original, profile.CADENCE_SOURCE_PATHS)
        for selection in (None, False):
            self.assertEqual(profile.cadence_source_paths({'accel_input_hypothesis': selection}), original)
        selected = profile.cadence_source_paths({'accel_input_hypothesis': True})
        self.assertEqual(len(selected), len(set(selected)))
        self.assertTrue(set(profile._ACCEL_INPUT_HYPOTHESIS_SOURCES) <= set(selected))
        self.assertEqual(profile.cadence_source_hashes({'accel_input_hypothesis': True}),
                         self.data['cadence_source_sha256'])

    def test_raw_unapproved_and_forged_token_never_return_selected_ref(self):
        for token in (None, True, 'forged'):
            data = copy.deepcopy(self.data)
            if token is not None: data['_accel_input_hypothesis_token'] = token
            with self.subTest(token=token), self.assertRaisesRegex(profile.ProfileError, 'loader proof'):
                profile.accel_input_hypothesis_settings(data)
        self.data.update(approved_for_supported_policy_output=False, review=None, blockers=['UNREVIEWED'])
        path = self.seal()
        with self.assertRaisesRegex(profile.ProfileError, 'unapproved'):
            profile.load_profile(path)
        plan = profile.load_profile(path, require_approved=False)
        self.assertFalse(plan['output_allowed'])
        self.assertEqual(self.loader.call_count, 0)
        with self.assertRaisesRegex(profile.ProfileError, 'loader proof'):
            profile.accel_input_hypothesis_settings(plan)

    def test_formal_selection_legacy_schema_and_nonboolean_rejected(self):
        for key, value in (('apply_reviewed_accel_calibration', True),
                           ('schema', profile.SCHEMA_V2), ('accel_input_hypothesis', 1),
                           ('accel_input_hypothesis', None)):
            data = copy.deepcopy(self.data); data[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(profile.ProfileError):
                profile._settings(data)

    def test_selected_scope_cannot_be_hold_ground_preload_gain_or_time_extension(self):
        for mode in (None, profile.CURRENT_HOLD_PROBE, profile.CURRENT_HOLD_AFTER_SUPPORTED_10S,
                     profile.FIXED_CATCH_CURRENT_HOLD_30S, profile.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
                     profile.SUPPORTED_POLICY_PROBE_5S, profile.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                     profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S, profile.SUPPORTED_POLICY_GAIN_STEP_3S,
                     profile.SUPPORTED_PRELOAD_5S, profile.SUPPORTED_POLICY_MIX_STEP_10PCT):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaisesRegex(profile.ProfileError, 'boxed two-second'):
                profile._accel_input_hypothesis_scope(data)
        for key, value in (('scope', 'ground'), ('local_characterization', None),
                           ('duration_s', 2.001), ('policy_weight', 0.), ('policy_weight', .005001),
                           ('fixed_catch', {}), ('human_supported_hold', {})):
            data = copy.deepcopy(self.data); data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): profile._settings(data)

    def test_existing_small_probe_axis_caps_and_deadlines_remain(self):
        limits = (('kp', 3.001), ('kd', .15001), ('max_displacement_from_start_rad', math.radians(1.001)),
                  ('max_estimated_pd_torque_nm', .10001), ('max_measured_torque_nm', 1.001),
                  ('max_tracking_error_rad', math.radians(2.001)), ('max_temperature_c', 45.001),
                  ('max_command_velocity_rad_s', math.radians(1.001)),
                  ('max_command_acceleration_rad_s2', math.radians(5.001)))
        for key, value in limits:
            data = copy.deepcopy(self.data); data['axes']['7'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): profile._settings(data)
        for key, value in (('hard_cycle_ms', 20.001), ('max_sample_age_ms', 20.001),
                           ('max_sample_gap_ms', 21.001), ('max_consecutive_20ms_misses', 1)):
            data = copy.deepcopy(self.data); data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): profile._settings(data)

    def test_raw_diagnostic_wrong_pin_and_missing_observer_provenance_rejected(self):
        baseline = copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes = (
            lambda r: r['plan'].pop('accel_input_hypothesis'),
            lambda r: r['plan'].update(accel_input_hypothesis=None),
            lambda r: r['plan'].update(accel_input_hypothesis={**self.reference, 'sha256': 'f'*64}),
            lambda r: r['input_sha256'].pop('accel_input_hypothesis'),
            lambda r: r['input_sha256'].update(accel_input_hypothesis='f'*64),
            lambda r: r['observer'].pop('accel_input_hypothesis'),
            lambda r: r['observer'].update(accel_input_hypothesis=None),
            lambda r: r['observer']['accel_input_hypothesis'].update(formal_calibration_approved=0),
            lambda r: r['observer']['accel_input_hypothesis'].update(candidate_sha256='f'*64),
            lambda r: r.update(motor_power_epoch='previous-epoch'),
            lambda r: r.update(cadence_source_sha256={}),
        )
        for index, change in enumerate(changes):
            self.docs['pipeline_diagnostic'] = copy.deepcopy(baseline)
            change(self.docs['pipeline_diagnostic'])
            with self.subTest(index=index), self.assertRaisesRegex(profile.ProfileError, 'hypothesis'):
                self.load()

    def test_loader_reaudit_failure_and_artifact_sha_failure_do_not_grant_output(self):
        self.loader.side_effect = ValueError('Synthetic capture audit failure')
        with self.assertRaisesRegex(ValueError, 'capture audit failure'): self.load()
        self.loader.side_effect = None
        path = self.seal()
        (self.base/'accel_input_hypothesis.json').write_bytes(b'changed')
        with self.assertRaisesRegex(profile.ProfileError, 'Artifact SHA256 mismatch'):
            profile.load_profile(path)

    def test_selected_missing_artifact_reference_and_file_rejected(self):
        reference = self.data['artifacts'].pop('accel_input_hypothesis')
        with self.assertRaisesRegex(profile.ProfileError, 'artifact references'):
            profile._structure(self.data)
        self.data['artifacts']['accel_input_hypothesis'] = reference
        path = self.seal()
        (self.base/'accel_input_hypothesis.json').unlink()
        with self.assertRaises(profile.ProfileError): profile.load_profile(path)

    def test_rare_jitter_two_second_mode_keeps_separate_diagnostic_acceptance(self):
        self.data['diagnostic_timing_acceptance'] = profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER
        for axis in self.data['axes'].values(): axis['max_measured_velocity_rad_s'] = .35
        for row in self.docs['pipeline_diagnostic']['measurements']:
            row['oldest_input_start_ns'] = row['release_ns']+2_000_000
        with self.assertRaisesRegex(profile.ProfileError, 'rare-jitter diagnostic acceptance'):
            self.load()
        self.docs['hardware_review']['rare_jitter_diagnostic_acceptance'] = {
            'mode': profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            'scope': self.data['scope'], 'strict_50hz_not_established': True,
            'live_deadline_policy_unchanged': True,
            'diagnostic_sha256': self.data['artifacts']['pipeline_diagnostic']['sha256'],
            'review': {**self.data['review'],
                       'decision': 'ACCEPT_RARE_JITTER_DIAGNOSTIC_FOR_2S_SUPPORTED_PROBE'}}
        loaded = self.load()
        self.assertEqual(profile.accel_input_hypothesis_settings(loaded), self.reference)
        self.assertEqual(loaded['hard_cycle_ms'], 20.)
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])

    def test_raw_norm_bounds_and_unapproved_scope_cannot_be_widened_or_promoted(self):
        for key, value in (('raw_norm_bounds_m_s2', [8.8, 10.2]),
                           ('formal_calibration_approved', True), ('absolute_orientation_error_bound_rad', .01),
                           ('grants_motor_output', True), ('fit_and_independent_captures_reaudited', False),
                           ('scope', 'ground')):
            old = self.provenance[key]; self.provenance[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(profile.ProfileError, 'hypothesis'):
                self.load()
            self.provenance[key] = old

    def test_loaded_selection_refs_settings_and_provenance_cannot_mutate(self):
        loaded = self.load()
        changes = (
            lambda p: p['artifacts']['accel_input_hypothesis'].update(sha256='f'*64),
            lambda p: p.update(policy_weight=.004),
            lambda p: p['axes']['7'].update(kp=2.9),
            lambda p: p['_accel_input_hypothesis_provenance'].update(formal_calibration_approved=True),
            lambda p: p.update(motor_power_epoch='different'),
            lambda p: p.update(accel_input_hypothesis=False),
        )
        for index, change in enumerate(changes):
            # deepcopy would duplicate the private object token; preserve it as
            # an already validated in-memory mapping, then mutate one field.
            altered = copy.deepcopy(loaded)
            altered['_accel_input_hypothesis_token'] = loaded['_accel_input_hypothesis_token']
            change(altered)
            with self.subTest(index=index), self.assertRaises(profile.ProfileError):
                profile.accel_input_hypothesis_settings(altered)

    def test_unselected_absent_or_null_diagnostic_fields_compatible_unexpected_input_rejected(self):
        self.data['accel_input_hypothesis'] = False
        self.data['cadence_source_sha256'] = profile.cadence_source_hashes(self.data)
        self.docs['pipeline_diagnostic']['cadence_source_sha256'] = self.data['cadence_source_sha256']
        self.data['artifacts'].pop('accel_input_hypothesis')
        self.docs.pop('accel_input_hypothesis')
        report = self.docs['pipeline_diagnostic']
        holders = (report['plan'], report['input_sha256'], report['observer'])
        for holder in holders: holder.pop('accel_input_hypothesis')
        loaded = self.load()
        self.assertIsNone(profile.accel_input_hypothesis_settings(loaded))
        for holder in holders: holder['accel_input_hypothesis'] = None
        self.assertIsNone(profile.accel_input_hypothesis_settings(self.load()))
        for holder in holders:
            holder['accel_input_hypothesis'] = self.reference['sha256']
            with self.assertRaisesRegex(profile.ProfileError, 'hypothesis'): self.load()
            holder['accel_input_hypothesis'] = None
        self.assertEqual(self.loader.call_count, 0)


if __name__ == '__main__':
    unittest.main()
