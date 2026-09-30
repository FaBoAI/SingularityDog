"""Geometric preload integration with real coordination and byte-level buses.

These are deterministic software fault tests, not hardware timing, contact,
support-load or physical emergency-stop evidence. Only the profile admission
accessor is substituted; path validation, shaping, parsing and STOP ownership
run through the production implementation.
"""
import copy
import hashlib
import json
import math
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import supported_preload_path as preload
from singularitydog_hw import policy_live_profile as live
from test_policy_output_runtime import (
    FakeIMU, FakeSession, SimulatedClock, encode_motion, measured_startup_profile,
    quantize,
)
from test_supported_preload_path import fixture as mathematical_fixture
import test_supported_preload_profile as profile_fixture


def fixture():
    """Retain real encoder quantization around a modest, model-valid pose."""
    path, _ = mathematical_fixture()
    data = measured_startup_profile()
    data.update(duration_s=5., startup_duration_s=1., policy_ramp_s=.2,
                stop_duration_s=.3, policy_weight=0.,
                diagnostic_timing_acceptance=live.SUPPORTED_PRELOAD_5S)
    data['cadence_source_sha256'] = live.cadence_source_hashes(data)
    initial = path['source_screen']['initial_model_rad_by_id']
    path['source_screen']['initial_raw_rad_by_id'] = initial.copy()
    for row in path['samples']:
        row['q_raw_rad_by_id'] = row['q_model_rad_by_id'].copy()
    for mid, axis in data['axes'].items():
        axis.update(sign=1, offset_rad=0., kp=6., kd=.15,
                    max_estimated_pd_torque_nm=.2,
                    max_displacement_from_start_rad=math.radians(1.),
                    max_command_velocity_rad_s=math.radians(1.),
                    max_command_acceleration_rad_s2=math.radians(5.),
                    max_tracking_error_rad=math.radians(2.),
                    max_measured_velocity_rad_s=.25,
                    max_measured_torque_nm=1.)
    # No hand-built cached path or skipped validator: production admission
    # will inspect the same dictionary again when the coordinator starts.
    preload.validate_path(path, data)
    return data, path


class ValidatingPolicy:
    """Stand-in model that records full validation and forbids inference."""
    def __init__(self, callback=None):
        self.validations = []
        self.callback = callback
        self.inference_calls = 0

    def validate_inputs(self, sample, imu, now):
        self.validations.append((sample, copy.deepcopy(imu), now))
        if not imu.get('valid') or not .9 <= math.dist(imu['projected_gravity'], (0., 0., 0.)) <= 1.1:
            raise ValueError('Synthetic model input gravity validation')
        if self.callback is not None:
            self.callback(self, sample, imu, now)

    def __call__(self, *_):
        self.inference_calls += 1
        raise AssertionError('A geometric preload must never run learned inference')


class SupportedPreloadRuntimeTests(unittest.TestCase):
    def run_case(self, *, clock=None, front=None, rear=None, imu=None,
                 policy=None, profile_data=None, path_data=None,
                 stop_requested=None, sleep=None, **kwargs):
        clock = clock or SimulatedClock()
        data, path = fixture()
        if profile_data is not None:
            data = profile_data
        if path_data is not None:
            path = path_data
        sessions = {'front': front or FakeSession(1, clock=clock),
                    'rear': rear or FakeSession(7, clock=clock)}
        initial = path['source_screen']['initial_raw_rad_by_id']
        for session in sessions.values():
            for mid in session.ids:
                if not getattr(session, 'preserve_initial_positions', False):
                    session.positions[mid] = initial[str(mid)]
        policy = policy or ValidatingPolicy()
        cancelled = threading.Event()
        payload = json.dumps(path, sort_keys=True, allow_nan=False).encode()
        settings = {'path': path, 'path_sha256': hashlib.sha256(payload).hexdigest(),
                    'return_complete_s': preload.RETURN_COMPLETE_S}
        kwargs.setdefault('absolute_epoch_cadence', True)
        with patch.object(runtime, 'supported_preload_settings', return_value=settings):
            report = runtime.run_supported_policy(
                data, sessions, imu or FakeIMU(clock=clock), policy,
                cancel_io=cancelled.set, encode_motion=encode_motion,
                clock=clock, sleep=sleep or clock.sleep,
                stop_requested=stop_requested, **kwargs)
        self.assertTrue(cancelled.is_set())
        self.assertTrue(all(len(session.stop_times) == 1 for session in sessions.values()))
        self.assertFalse(report['learned_targets_attempted'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(policy.inference_calls, 0)
        return report, sessions, policy

    def assert_stopped_fault(self, report, phrase=None):
        self.assertEqual(report['status'], 'ABORTED', report['errors'])
        self.assertTrue(report['stop_confirmed'], report['errors'])
        self.assertFalse(report['normal_ramp_completed'])
        if phrase:
            self.assertIn(phrase.lower(), ' '.join(report['errors']).lower())

    def test_five_second_path_returns_before_normal_gain_down_and_stops_both_buses(self):
        report, sessions, policy = self.run_case()
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertTrue(report['preload_targets_attempted'])
        self.assertTrue(report['preload_targets_sent'])
        self.assertTrue(report['preload_return_commanded'])
        self.assertTrue(report['preload_return_measured'])
        audit = report['preload_wire_reference_audit']
        self.assertEqual(report['preload_wire_reference_audit_origin'], 'reviewed_capture')
        self.assertEqual(audit['sample_count'], 251)
        self.assertTrue(audit['quantized_static_bounds_passed'])
        self.assertGreater(audit['maxima']['quantization_error_rad'], 0.)
        self.assertFalse(audit['actual_runtime_commands_audited'])
        self.assertFalse(audit['measured_physical_motion_verified'])
        self.assertFalse(audit['physical_speed_acceleration_or_torque_cap'])
        self.assertLessEqual(report['preload_return_max_error_rad'],
                             preload.MEASURED_RETURN_TOLERANCE_RAD)
        self.assertTrue(report['normal_ramp_completed'])
        self.assertTrue(report['stop_confirmed'])
        self.assertGreater(len(report['cycles']), 225)
        self.assertGreaterEqual(len(policy.validations), len(report['cycles'])+1)
        self.assertEqual(report['cycles'][-1]['phase'], 'stopped')
        origin = report['cycles'][0]['command']['q_model_rad']
        self.assertGreater(max(abs(c['command']['q_model_rad'][0]-origin[0])
                               for c in report['cycles']), .004)
        for cycle in report['cycles']:
            if cycle['cadence_slot'] >= 200:
                self.assertLessEqual(max(abs(a-b) for a, b in
                    zip(cycle['command']['q_model_rad'], origin)),
                    preload.COMMAND_RETURN_TOLERANCE_RAD)
        for session in sessions.values():
            self.assertFalse(session.enabled)
            self.assertGreater(session.positive_gain_writes, 0)

    def test_real_profile_loader_admission_executes_without_accessor_substitution(self):
        admission = profile_fixture.SupportedPreloadProfileTests(
            methodName='test_supported_finite_path_has_loader_token_and_preserves_candidate')
        admission.setUp()
        self.addCleanup(admission.doCleanups)
        data = admission.load()
        settings = live.supported_preload_settings(data)
        clock = SimulatedClock()
        class ProfileIdentitySession(FakeSession):
            def _exchange(self, wires, timeout_ns, send_only):
                result = super()._exchange(wires, timeout_ns, send_only)
                for record in result[0]:
                    tx = codec.ATParser().feed(bytes(record.tx))[0]
                    if tx.kind == 0:
                        record.rx[7:15] = bytes.fromhex(data['axes'][str(tx.destination)]['uid'])
                return result
        sessions = {'front': ProfileIdentitySession(1, clock=clock),
                    'rear': ProfileIdentitySession(7, clock=clock)}
        initial = settings['path']['source_screen']['initial_raw_rad_by_id']
        for session in sessions.values():
            session.positions = {mid: initial[str(mid)] for mid in session.ids}
        policy = ValidatingPolicy()
        cancelled = threading.Event()
        report = runtime.run_supported_policy(
            data, sessions, FakeIMU(clock=clock), policy,
            cancel_io=cancelled.set, encode_motion=encode_motion,
            clock=clock, sleep=clock.sleep, absolute_epoch_cadence=True)
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertTrue(report['preload_targets_sent'])
        self.assertTrue(report['preload_return_commanded'])
        self.assertTrue(report['preload_return_measured'])
        self.assertTrue(report['normal_ramp_completed'])
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['learned_targets_attempted'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(policy.inference_calls, 0)
        self.assertTrue(cancelled.is_set())
        self.assertTrue(all(len(s.stop_times) == 1 for s in sessions.values()))

    def test_capture_pose_drift_and_changed_raw_turn_fail_before_enable(self):
        for delta in (math.radians(.1), 2*math.pi):
            with self.subTest(delta=delta):
                clock = SimulatedClock()
                front = FakeSession(1, clock=clock)
                front.preserve_initial_positions = True
                _, path = fixture()
                front.positions = {mid: path['source_screen']['initial_raw_rad_by_id'][str(mid)]
                                   for mid in front.ids}
                front.positions[1] += delta
                report, sessions, _ = self.run_case(clock=clock, front=front)
                self.assert_stopped_fault(report, 'capture/origin mismatch')
                self.assertFalse(report['motor_enable_sent'])
                self.assertTrue(all(s.positive_gain_writes == 0 for s in sessions.values()))

    def test_origin_is_checked_again_after_zero_gain_startup(self):
        clock = SimulatedClock()
        class ShiftDuringEnable(FakeSession):
            def _exchange(self, wires, timeout_ns, send_only):
                result = super()._exchange(wires, timeout_ns, send_only)
                for record in result[0]:
                    tx = codec.ATParser().feed(bytes(record.tx))[0]
                    if tx.kind == 3 and tx.destination == 1:
                        self.positions[1] += math.radians(.1)
                    if tx.kind == 1 and tx.destination == 1 and not any(tx.data[4:]):
                        self.positions[1] += math.radians(.1)
                        record.rx[7:9] = quantize(self.positions[1], -12.57, 12.57).to_bytes(2, 'big')
                return result
        report, sessions, _ = self.run_case(clock=clock, front=ShiftDuringEnable(1, clock=clock))
        self.assert_stopped_fault(report, 'capture/origin mismatch')
        self.assertTrue(report['motor_enable_sent'])
        self.assertTrue(all(s.positive_gain_writes == 0 for s in sessions.values()))

    def test_operator_cancellation_stops_immediately_without_a_forced_return(self):
        stop = threading.Event()
        def cancel(policy, *_):
            if len(policy.validations) == 100:
                stop.set()
        report, _, _ = self.run_case(policy=ValidatingPolicy(cancel), stop_requested=stop)
        self.assert_stopped_fault(report, 'cancel')
        self.assertTrue(report['preload_targets_sent'])
        self.assertFalse(report['preload_return_commanded'])
        self.assertFalse(any(row['phase'] == 'graceful_stop' for row in report['journal']))

    def test_usb_write_failure_uses_existing_stop_owners(self):
        clock = SimulatedClock()
        report, _, _ = self.run_case(clock=clock, front=FakeSession(1, clock=clock, fail_motion=True))
        self.assert_stopped_fault(report, 'USB lost')
        self.assertFalse(report['preload_return_commanded'])

    def test_numerical_path_is_revalidated_before_enable(self):
        _, path = fixture()
        path['samples'][100]['q_raw_rad_by_id']['1'] += 2*math.pi
        clock = SimulatedClock()
        front = FakeSession(1, clock=clock)
        rear = FakeSession(7, clock=clock)
        with self.assertRaisesRegex(ValueError, 'encoder branch'):
            self.run_case(clock=clock, front=front, rear=rear, path_data=path)
        self.assertFalse(front.calls)
        self.assertFalse(rear.calls)
        self.assertFalse(front.stop_times)
        self.assertFalse(rear.stop_times)

    def test_measured_torque_fault_retains_existing_hard_limit(self):
        clock = SimulatedClock()
        report, _, _ = self.run_case(clock=clock,
            front=FakeSession(1, clock=clock, high_returned_torque=True))
        self.assert_stopped_fault(report, 'torque')
        self.assertFalse(report['preload_return_commanded'])

    def test_relative_cadence_cannot_execute_a_geometric_path(self):
        clock = SimulatedClock()
        front, rear = FakeSession(1, clock=clock), FakeSession(7, clock=clock)
        with self.assertRaisesRegex(RuntimeError, '[Aa]bsolute'):
            self.run_case(clock=clock, front=front, rear=rear, absolute_epoch_cadence=False)
        self.assertFalse(front.calls)
        self.assertFalse(rear.calls)

    def test_stale_imu_and_invalid_gravity_stop_without_inference(self):
        for kind in ('stale', 'gravity'):
            with self.subTest(kind=kind):
                clock = SimulatedClock()
                base = FakeIMU(clock=clock)
                count = [0]
                def imu():
                    value = base()
                    count[0] += 1
                    if count[0] > 60:
                        if kind == 'stale':
                            value['read_started_monotonic_ns'] -= 100_000_000
                            value['read_finished_monotonic_ns'] -= 100_000_000
                        else:
                            value['projected_gravity'] = [0., 0., -2.]
                    return value
                report, _, policy = self.run_case(clock=clock, imu=imu)
                self.assert_stopped_fault(report)
                self.assertGreater(len(policy.validations), 50)
                self.assertFalse(report['preload_return_commanded'])

    def test_stale_bus_feedback_stops_both_buses(self):
        clock = SimulatedClock()
        class StaleFeedback(FakeSession):
            def _exchange(self, wires, timeout_ns, send_only):
                result = super()._exchange(wires, timeout_ns, send_only)
                if self.positive_gain_writes > 750:
                    for record in result[0]:
                        tx = codec.ATParser().feed(bytes(record.tx))[0]
                        if tx.kind == 1:
                            record.start_ns -= 100_000_000
                return result
        report, _, _ = self.run_case(clock=clock, rear=StaleFeedback(7, clock=clock))
        self.assert_stopped_fault(report)
        self.assertFalse(report['preload_return_commanded'])

    def test_first_and_steady_preload_deadlines_have_no_learned_startup_allowance(self):
        for validation_number in (2, 80):
            with self.subTest(validation_number=validation_number):
                clock = SimulatedClock()
                def delay(policy, *_):
                    if len(policy.validations) == validation_number:
                        clock.advance(21_000_000)
                report, _, _ = self.run_case(clock=clock, policy=ValidatingPolicy(delay))
                self.assert_stopped_fault(report, 'deadline')
                self.assertFalse(report['startup_20ms_allowance_enabled'])
                self.assertFalse(report['preload_return_commanded'])

    def test_skipped_absolute_slot_is_not_replayed_or_compressed(self):
        clock = SimulatedClock()
        sleeps = [0]
        def delayed_sleep(seconds):
            sleeps[0] += 1
            clock.sleep(seconds)
            if sleeps[0] == 90:
                clock.advance(25_000_000)
        report, _, _ = self.run_case(clock=clock, sleep=delayed_sleep)
        self.assert_stopped_fault(report, 'slot')
        self.assertFalse(report['preload_return_commanded'])
        slots = [row['cadence_slot'] for row in report['cycles']]
        self.assertEqual(slots, list(range(len(slots))))

    def test_measured_return_is_required_before_gain_down(self):
        clock = SimulatedClock()
        inject = threading.Event()
        class ReturnError(FakeSession):
            def _exchange(self, wires, timeout_ns, send_only):
                result = super()._exchange(wires, timeout_ns, send_only)
                if inject.is_set():
                    for record in result[0]:
                        tx = codec.ATParser().feed(bytes(record.tx))[0]
                        if tx.kind == 1 and tx.destination == 1:
                            value = self.positions[1]+math.radians(.2)
                            record.rx[7:9] = quantize(value, -12.57, 12.57).to_bytes(2, 'big')
                return result
        def begin_error(policy, *_):
            if len(policy.validations) >= 201:
                inject.set()
        report, _, _ = self.run_case(clock=clock, front=ReturnError(1, clock=clock),
                                     policy=ValidatingPolicy(begin_error))
        self.assert_stopped_fault(report, 'return')
        self.assertTrue(report['preload_return_commanded'])
        self.assertFalse(report['preload_return_measured'])
        self.assertFalse(any(row['phase'] == 'graceful_stop' for row in report['journal']))

    def test_unconfirmed_stop_never_reports_completion_or_safe_shutdown(self):
        clock = SimulatedClock()
        report, _, _ = self.run_case(clock=clock,
                                     rear=FakeSession(7, clock=clock, stop_unconfirmed=True))
        self.assertEqual(report['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertIn('physical power cutoff required', ' '.join(report['errors']))


if __name__ == '__main__':
    unittest.main()
