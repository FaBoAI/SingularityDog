"""Offline per-axis Type2 diagnostic thresholds; no physical torque-cap claim."""
from dataclasses import replace
import math
import unittest

from test_rs05_bounded_pose_trial import PoseTransport
from test_rs05_fr_step4_kp12 import run as step_run
from test_rs05_fr_current_kp12 import run as hold_run
from test_rs05_serial_enable import SequentialEnableTransport


PROFILE = 'fr_step4_kp12_torque1_diagnostic'


class TorqueTransport(SequentialEnableTransport):
    def __init__(self, motor_id=2, torque=0.):
        super().__init__()
        self.torque_motor_id, self.torque = motor_id, torque

    def value(self, motor_id):
        value = super().value(motor_id)
        return replace(value, torque_nm=self.torque) if motor_id == self.torque_motor_id and self.commanded else value


def run(transport, **changes):
    options = dict(gain_profile=PROFILE)
    options.update(changes)
    return step_run(transport, **options)


class FRStep4Torque1Tests(unittest.TestCase):
    def assert_stopped(self, t, result):
        self.assertTrue(result['stop_confirmed'], result['errors'])
        self.assertEqual(t.stop_calls[-1], (1, 2, 3))
        self.assertTrue(all(f.kind == 4 for _, f in t.frames[-3:]))

    def test_id2_inclusive_one_nm_and_other_axes_inclusive_half_nm(self):
        for mid, torque in ((2, .5001), (2, .75), (2, 1.), (2, -1.),
                           (1, .5), (1, -.5), (3, .5), (3, -.5)):
            with self.subTest(mid=mid, torque=torque):
                t = TorqueTransport(mid, torque); result = run(t)
                self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
                self.assertEqual(result['max_abs_torque_feedback_candidate_nm_by_motor'], {1:.5, 2:1., 3:.5})
                self.assertIsNone(result['max_abs_torque_feedback_candidate_nm'])
                self.assertFalse(result['physical_torque_cap_verified'])
                self.assert_stopped(t, result)

    def test_each_axis_aborts_strictly_above_its_limit_in_both_directions(self):
        for mid, limit in ((1, .5), (2, 1.), (3, .5)):
            for sign in (-1, 1):
                with self.subTest(mid=mid, sign=sign):
                    t = TorqueTransport(mid, sign * (limit + .0001)); result = run(t)
                    self.assertEqual(result['status'], 'ABORTED')
                    self.assertTrue(any(f'ID{mid} Type2 torque feedback exceeds{limit:g}Nm' in e for e in result['errors']))
                    self.assert_stopped(t, result)

    def test_nonfinite_and_non_numeric_feedback_fail_closed_for_all_axes(self):
        for mid in (1, 2, 3):
            for torque in (float('nan'), float('inf'), -float('inf'), True, '0.1'):
                with self.subTest(mid=mid, torque=torque):
                    t = TorqueTransport(mid, torque); result = run(t)
                    self.assertEqual(result['status'], 'ABORTED')
                    self.assertTrue(any('torque feedback' in e for e in result['errors']))
                    self.assert_stopped(t, result)

    def test_older_profiles_keep_id2_half_nm_limit(self):
        for profile in ('fr_step4_kp12_diagnostic', 'fr_current_kp12_diagnostic',
                        'fr_hip_kp6_diagnostic', 'kp4_diagnostic'):
            t = TorqueTransport(2, .5001)
            result = hold_run(t, gain_profile=profile)
            self.assertEqual(result['status'], 'ABORTED', profile)
            self.assertTrue(any('ID2 Type2 torque feedback exceeds0.5Nm' in e for e in result['errors']))
            self.assertEqual(result['max_abs_torque_feedback_candidate_nm'], .5)
            self.assert_stopped(t, result)

    def test_wire_trajectory_gains_time_and_watchdog_writes_are_identical(self):
        old, new = SequentialEnableTransport(), SequentialEnableTransport()
        old_result, result = step_run(old), run(new)
        self.assertEqual(old_result['status'], result['status'])
        self.assertEqual([(when, f.wire) for when, f in old.frames], [(when, f.wire) for when, f in new.frames])
        self.assertEqual(result['Kp_by_motor'], {1:3., 2:12., 3:12.})
        metadata_changes = {'gain_profile', 'max_abs_torque_feedback_candidate_nm',
                            'max_abs_torque_feedback_candidate_nm_by_motor'}
        self.assertEqual({k:v for k,v in result['bounded_pose_plan'].items() if k not in metadata_changes},
                         {k:v for k,v in old_result['bounded_pose_plan'].items() if k not in metadata_changes})
        for mid in new.ids:
            times = [when for when, f in new.frames if f.destination == mid and f.kind == 1 and f.data[4:8] != bytes(4)]
            self.assertEqual(len(times), 100)
            self.assertLess(times[-1] - times[0], 5.)
        self.assert_stopped(new, result)

    def test_scope_and_target_limits_reject_before_io(self):
        changes = [{'matched_start_positions':None}, {'profile':'legacy-rms-v1'},
                   {'profile':'position-v2-all'}, {'enable_profile':'legacy-burst'},
                   {'position_response_evidence':None}, {'observation_profile':'rr_settling_1s'}]
        for mid in (1, 2, 3):
            for sign in (-1, 1):
                t = SequentialEnableTransport(); targets = dict(t.centers)
                targets[mid] += sign * math.radians(4.0001)
                changes.append({'absolute_targets':targets})
        for change in changes:
            t = SequentialEnableTransport()
            with self.subTest(change=change), self.assertRaises(ValueError): run(t, **change)
            self.assertEqual((t.calls, t.frames, t.stop_calls), ([], [], []))
        for leg in ('FL', 'RR', 'RL'):
            t = PoseTransport(leg)
            with self.subTest(leg=leg), self.assertRaises(ValueError): run(t)
            self.assertEqual((t.calls, t.frames, t.stop_calls), ([], [], []))

    def test_mode_fault_stale_and_final_arrival_guard_still_abort(self):
        for failure in ('mode0', 'fault', 'stale', 'hold_drift'):
            t = SequentialEnableTransport(failure if failure != 'hold_drift' else None)
            if failure == 'hold_drift': t.failure = failure
            result = run(t)
            self.assertEqual(result['status'], 'ABORTED', failure)
            if failure == 'hold_drift': self.assertTrue(any('1-degree' in e for e in result['errors']))
            self.assert_stopped(t, result)


if __name__ == '__main__': unittest.main()
