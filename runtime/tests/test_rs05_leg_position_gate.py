"""Offline prospective position-first guards; no serial or actuator access."""
import math
import unittest
from dataclasses import replace
from test_rs05_leg_settled import samples, CENTERS, WindowTransport
from test_rs05_leg_trial import UIDS, DIRECTIONS
from singularitydog_hw.rs05_leg_trial import evaluate_settled_window, run_leg_trial

class PositionGateTests(unittest.TestCase):
    def evaluate(self, rows):
        return evaluate_settled_window(rows, CENTERS, profile="position-v2")

    def test_alternating_velocity_noise_retained_and_warned_only_in_v2(self):
        rows = samples(velocity=lambda t: .06 if round(t*10)%2 else -.06)
        self.assertFalse(evaluate_settled_window(rows, CENTERS)["passed"])
        report = self.evaluate(rows)
        self.assertTrue(report["passed"])
        self.assertTrue(report["warnings"])
        self.assertFalse(report["absolute_rest_proven"])
        self.assertFalse(report["joint_calibration_verified"])
        self.assertAlmostEqual(report["motors"][1]["velocity_RMS_rad_s"], .06)
        self.assertEqual(report["motors"][1]["samples"], rows[1])

    def test_frozen_position_does_not_mask_reported_constant_velocity(self):
        self.assertFalse(self.evaluate(samples(velocity=lambda t: .06))["passed"])

    def test_creep_returning_oscillation_and_late_motion_rejected(self):
        for position in (lambda t: .0006*t,
                         lambda t: .0006*math.sin(math.pi*t),
                         lambda t: max(0, t-1.5)*.0015):
            with self.subTest(position=position):
                self.assertFalse(self.evaluate(samples(position=position))["passed"])

    def test_missing_stale_fault_and_instantaneous_guards_preserved(self):
        for failure in ("missing", "stale", "fault", "instant", "mode"):
            rows = samples()
            if failure == "missing": rows[1].pop(10)
            if failure == "stale": rows[1][10]["checked_monotonic_s"] += .11
            if failure == "fault": rows[1][10]["feedback"]["fault_bits"] = 1
            if failure == "instant": rows[1][10]["feedback"]["velocity_rad_s"] = .51
            if failure == "mode": rows[1][10]["feedback"]["mode_state"] = 2
            with self.subTest(failure=failure):
                self.assertFalse(self.evaluate(rows)["passed"])

    def test_failed_gate_collects_exact_window_and_never_enables_or_retries(self):
        def change(t, found):
            fb, ts = found[2]
            found[2] = (replace(fb, velocity_rad_s=.06), ts)
            return found
        t = WindowTransport(change)
        result = run_leg_trial(t, UIDS, lambda: None, lambda _: None,
            directions=DIRECTIONS, clock=t.clock, wait=t.clock.wait, profile="position-v2")
        self.assertEqual(t.window_count, 21)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(any(f.kind == 3 for _, f in t.frames))
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["joint_calibration_verified"])

    def test_profile_restricted_to_fr(self):
        with self.assertRaises(ValueError):
            evaluate_settled_window({4:[],5:[],6:[]}, {4:0,5:0,6:0}, profile="position-v2")

if __name__ == "__main__":
    unittest.main()
