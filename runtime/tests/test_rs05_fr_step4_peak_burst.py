"""The brief ID2 peak allowance is bounded by fresh active feedback, not a motor cap."""
from dataclasses import replace
import unittest

from test_rs05_fr_step4_kp12 import run as step_run
from test_rs05_serial_enable import SequentialEnableTransport


PROFILE = 'fr_step4_kp12_peak_burst_diagnostic'


class BurstTransport(SequentialEnableTransport):
    def __init__(self, high_samples=(), high_value=1.2):
        super().__init__()
        self.high_samples = set(high_samples)
        self.high_value = high_value
        self.id2_active_read_count = 0

    def value(self, motor_id):
        value = super().value(motor_id)
        if motor_id == 2 and self.commanded:
            self.id2_active_read_count += 1
            if self.id2_active_read_count in self.high_samples:
                return replace(value, torque_nm=self.high_value)
        return value


def run(transport):
    return step_run(transport, gain_profile=PROFILE)


class FRStep4PeakBurstTests(unittest.TestCase):
    def test_continuous_point_nine_nm_completes_five_seconds(self):
        transport = BurstTransport(range(1, 120), .9)
        result = run(transport)
        self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['id2_above_one_nm_active_fresh_samples'], 0)
        self.assertEqual(result['id2_active_peak_abs_torque_feedback_nm'], .9)
        self.assertTrue(result['stop_confirmed'])

    def test_three_fresh_high_samples_complete_and_stop(self):
        transport = BurstTransport((18, 19, 20))
        result = run(transport)
        self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['id2_above_one_nm_active_fresh_samples'], 3)
        self.assertEqual(result['id2_active_peak_abs_torque_feedback_nm'], 1.2)
        self.assertEqual(result['id2_above_one_nm_max_active_fresh_samples'], 3)
        self.assertEqual(result['max_abs_torque_feedback_candidate_nm_by_motor'], {1:.5,2:1.5,3:.5})
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['physical_torque_cap_verified'])

    def test_fourth_fresh_high_sample_aborts_and_stops(self):
        transport = BurstTransport((18, 19, 20, 21))
        result = run(transport)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['motion_completed'])
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['id2_above_one_nm_active_fresh_samples'], 4)
        self.assertTrue(any('brief peak sample/time budget' in e for e in result['errors']))

    def test_scattered_high_samples_exceed_time_budget(self):
        transport = BurstTransport((18, 30))
        result = run(transport)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['id2_above_one_nm_active_fresh_samples'], 2)
        self.assertTrue(any('brief peak sample/time budget' in e for e in result['errors']))
        self.assertTrue(result['stop_confirmed'])

    def test_above_one_nm_aborts_on_first_sample(self):
        transport = BurstTransport((18,), 1.5001)
        result = run(transport)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('exceeds1.5Nm' in e for e in result['errors']))
        self.assertTrue(result['stop_confirmed'])

    def test_rejects_other_legs_and_all_previous_profiles_are_unchanged(self):
        from test_rs05_bounded_pose_trial import PoseTransport
        for leg in ('FL', 'RR', 'RL'):
            with self.assertRaises(ValueError):
                step_run(PoseTransport(leg), gain_profile=PROFILE)
        old = step_run(BurstTransport((18,)))
        self.assertEqual(old['status'], 'ABORTED')
        self.assertTrue(any('exceeds0.5Nm' in e for e in old['errors']))


if __name__ == '__main__':
    unittest.main()
