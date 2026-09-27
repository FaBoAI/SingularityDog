"""Offline-only checks for the finite, shared-Enable front-hip plan."""

import math
import unittest

from singularitydog_hw.continuous_front_hip_plan import (
    ALL_IDS, CYCLE_S, HOLD_TICKS, INITIAL_HOLD_TICKS, RAMP_TICKS,
    build_front_hip_continuous_plan,
)
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request


def fresh_centers():
    return {mid: .2 * mid for mid in ALL_IDS}


class ContinuousFrontHipPlanTests(unittest.TestCase):
    def test_fixed_hold_then_two_segments_keep_twelve_axes_continuous(self):
        starts = fresh_centers()
        plan = build_front_hip_continuous_plan(starts, [5., 10.])
        self.assertEqual(len(plan), 380)
        self.assertEqual(CYCLE_S * len(plan), 19.)
        self.assertEqual(plan.initial_hold_end_tick, 19)
        self.assertEqual(plan.segment_end_ticks, (199, 379))
        self.assertEqual(plan.waypoints_deg, (5., 10.))
        self.assertTrue(all(plan[tick] == starts for tick in range(INITIAL_HOLD_TICKS)))
        self.assertEqual(plan[20], starts)
        self.assertEqual(plan[199], plan[200])
        self.assertEqual(plan[199], plan.waypoint_targets[0])
        self.assertEqual(plan[379], plan.waypoint_targets[1])
        self.assertAlmostEqual(plan[199][3] - starts[3], math.radians(5.), places=10)
        self.assertAlmostEqual(plan[379][3] - starts[3], math.radians(10.), places=10)
        self.assertAlmostEqual(plan[379][6] - starts[6], -math.radians(10.), places=10)
        for tick in range(len(plan)):
            self.assertEqual(set(plan[tick]), set(ALL_IDS))
            self.assertTrue(all(plan[tick][mid] == starts[mid]
                                for mid in ALL_IDS if mid not in (3, 6)))
            # The same initial raw center is used for every wire, including
            # the second segment, so no hidden target rebase occurs.
            for mid in (3, 6):
                motion_request(phase=TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10,
                               center_rad=starts[mid], offset_rad=plan[tick][mid]-starts[mid],
                               motor_id=mid)

    def test_ramps_are_monotone_with_zero_velocity_at_each_boundary(self):
        plan = build_front_hip_continuous_plan(fresh_centers(), [5., 10.])
        for first, last in ((20, 180), (200, 360)):
            self.assertTrue(all(plan[tick][3] <= plan[tick + 1][3]
                                for tick in range(first, last)))
            self.assertTrue(all(plan[tick][6] >= plan[tick + 1][6]
                                for tick in range(first, last)))
        for first, last in ((180, 199), (360, 379)):
            self.assertTrue(all(plan[tick] == plan[first]
                                for tick in range(first, last + 1)))
        self.assertLess(plan[21][3] - plan[20][3], math.radians(.001))
        self.assertLess(plan[201][3] - plan[200][3], math.radians(.001))
        self.assertEqual(RAMP_TICKS + HOLD_TICKS, 180)

    def test_wrong_waypoints_cannot_expand_or_reverse_the_authorized_sweep(self):
        for waypoints in ([], [5.], [10., 5.], [5., 10., 0.], [5., 0.],
                          [5., 11.], [4., 10.], (5., 10.), [True, 10.],
                          [5., float('nan')], ['5', 10.], [5., float('inf')]):
            with self.subTest(waypoints=waypoints), self.assertRaises(ValueError):
                build_front_hip_continuous_plan(fresh_centers(), waypoints)

    def test_missing_unwrapped_or_nonfinite_start_fails_before_any_plan(self):
        for mid, bad in ((3, 12.56), (6, -12.56), (1, float('nan')), (2, True)):
            starts = fresh_centers()
            starts[mid] = bad
            with self.subTest(mid=mid, bad=bad), self.assertRaises(ValueError):
                build_front_hip_continuous_plan(starts, [5., 10.])
        starts = fresh_centers()
        del starts[12]
        with self.assertRaises(ValueError):
            build_front_hip_continuous_plan(starts, [5., 10.])
        starts = fresh_centers()
        starts['12'] = starts.pop(12)
        with self.assertRaises(ValueError):
            build_front_hip_continuous_plan(starts, [5., 10.])


if __name__ == '__main__':
    unittest.main()
