"""FR four-degree diagnostic may finish a bounded observation without target success."""
from dataclasses import replace
import math
import unittest

from singularitydog_hw import bounded_pose_plan as pose
from test_rs05_fr_step4_kp12 import run as step_run
from test_rs05_serial_enable import SequentialEnableTransport


PROFILE = 'fr_step4_kp12_peak_burst_diagnostic'
OBSERVATION = 'fr_settling_1s'


class UndertrackingFR(SequentialEnableTransport):
    def __init__(self, error=lambda index: 1.4):
        super().__init__()
        self.error = error

    def value(self, motor_id):
        value = super().value(motor_id)
        hold_index = len(self.motion_batches) - 81
        if motor_id == 2 and hold_index >= 0 and self.commanded:
            value = replace(value, protocol_position_rad=
                self.commanded[motor_id] - math.radians(self.error(hold_index)))
        return value


class FRSettlingObservationTests(unittest.TestCase):
    def run_fr(self, transport, **options):
        return step_run(transport, gain_profile=PROFILE, observation_profile=OBSERVATION,
                        **options)

    def test_undertracking_observation_completes_without_target_success(self):
        transport = UndertrackingFR()
        result = self.run_fr(transport)
        self.assertEqual(result['status'], 'FR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED',
                         result['errors'])
        self.assertTrue(result['settling_observation_completed'])
        self.assertFalse(result['arrival_candidate_met'])
        self.assertFalse(result['hold_candidate_met'])
        self.assertEqual(len(result['settling_observation_samples']), 20)
        self.assertEqual(len(transport.motion_batches), 100)
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(transport.enabled)
        self.assertAlmostEqual(result['settling_observation_evaluation']['motors']['2']
                               ['final_target_error_rad'], -math.radians(1.4), delta=math.radians(.03))
        self.assertEqual(result['settling_observation_evaluation']['arrival_threshold_unchanged_rad'],
                         math.radians(1))
        for flag in pose.FLAGS:
            self.assertFalse(result[flag])

    def test_default_trial_still_aborts_at_one_degree(self):
        transport = UndertrackingFR()
        result = step_run(transport, gain_profile=PROFILE)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['motion_completed'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(any('1-degree' in error for error in result['errors']))

    def test_error_above_two_degrees_or_worsening_aborts_and_stops(self):
        for error in (lambda index: 2.001,
                      lambda index: 1.4 if index < 3 else 1.651):
            with self.subTest(error=error):
                transport = UndertrackingFR(error)
                result = self.run_fr(transport)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertFalse(result['settling_observation_completed'])
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(transport.enabled)
                self.assertLess(transport.clock()-transport.enable_time, 4.3)

    def test_scope_is_exclusive_to_explicit_fr_peak_burst(self):
        transport = UndertrackingFR()
        with self.assertRaises(ValueError):
            step_run(transport, gain_profile='fr_step4_kp12_diagnostic',
                     observation_profile=OBSERVATION)
        self.assertEqual((transport.calls, transport.frames, transport.stop_calls), ([], [], []))
        with self.assertRaises(ValueError):
            pose.evaluate_fr_settling_observation('not an FR plan', [], run_start_ns=0,
                                                  ended_ns=0)


if __name__ == '__main__':
    unittest.main()
