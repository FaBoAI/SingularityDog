"""Offline-only safety and trajectory checks; no motor transport."""
from dataclasses import replace
import math
import unittest

from singularitydog_hw.fullbody_step10_plan import (
    IDS, build_plan, check_feedback, require_current_hold_gate)
from singularitydog_hw.rs05_trial_protocol import Type2Feedback


class FullbodyStep10PlanTests(unittest.TestCase):
    def setUp(self):
        self.starts = {i: 0.1 * i for i in IDS}
        self.directions = {i: 1 if i % 2 else -1 for i in IDS}
        self.plan = build_plan(self.starts, self.directions)

    def feedback(self, tick, now=1.0):
        target = self.plan.samples[tick]['target_rad_by_id']
        return {i: (Type2Feedback(2, 0, 32767, target[i], 0.0, 0.0, 30.0), now - .01)
                for i in IDS}

    def test_common_quintic_timeline_and_bounded_endpoint(self):
        self.assertEqual(len(self.plan.samples), 180)
        for index, expected in ((0, 0.), (160, 8.), (179, 8.95)):
            self.assertAlmostEqual(self.plan.samples[index]['elapsed_s'], expected)
        self.assertEqual(self.plan.samples[0]['fraction'], 0.)
        self.assertEqual(self.plan.samples[160]['fraction'], 1.)
        self.assertEqual(self.plan.samples[-1]['fraction'], 1.)
        for i in IDS:
            self.assertAlmostEqual(self.plan.samples[-1]['target_rad_by_id'][i] - self.starts[i],
                                   self.directions[i] * math.radians(10))
        self.assertTrue(all(self.plan.samples[t]['fraction'] <= self.plan.samples[t + 1]['fraction']
                            for t in range(179)))
        self.assertAlmostEqual(self.plan.samples[1]['fraction'],
                               self.plan.samples[159]['fraction'] * -1 + 1,
                               places=10)

    def test_rejects_implicit_directions_missing_axes_and_protocol_overflow(self):
        with self.assertRaises(ValueError):
            build_plan(self.starts, {i: 1 for i in range(1, 12)})
        with self.assertRaises(ValueError):
            build_plan(self.starts, {**self.directions, 4: 0})
        with self.assertRaises(ValueError):
            build_plan(self.starts, self.directions, amplitude_deg=10.01)
        with self.assertRaises(ValueError):
            build_plan({**self.starts, 1: 12.55}, self.directions)

    def test_all_twelve_feedback_guards_and_no_silent_drop(self):
        tick = 90
        self.assertEqual(len(check_feedback(self.plan, tick, self.feedback(tick), now_s=1.0)), 12)
        base = self.feedback(tick)
        variants = [
            {i: row for i, row in base.items() if i != 4},
            {**base, 4: (base[4][0], .89)},
            {**base, 4: (replace(base[4][0], mode_state=0), .99)},
            {**base, 4: (replace(base[4][0], fault_bits=1), .99)},
            {**base, 4: (replace(base[4][0], torque_nm=.81), .99)},
            {**base, 4: (replace(base[4][0], velocity_rad_s=.61), .99)},
            {**base, 4: (replace(base[4][0], protocol_position_rad=base[4][0].protocol_position_rad+.04), .99)},
        ]
        for values in variants:
            with self.subTest(values=values.get(4)):
                with self.assertRaises((ValueError, RuntimeError)):
                    check_feedback(self.plan, tick, values, now_s=1.0)

    def test_r13_abort_cannot_open_ten_degree_gate(self):
        kwargs = {'hold_boot_id': 'boot', 'current_boot_id': 'boot',
                  'hold_stop_confirmed': True, 'id4_mechanical_check_passed': True}
        with self.assertRaises(RuntimeError):
            require_current_hold_gate(hold_status='ABORTED', **kwargs)
        with self.assertRaises(RuntimeError):
            require_current_hold_gate(hold_status='CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                                      **{**kwargs, 'id4_mechanical_check_passed': False})
        self.assertTrue(require_current_hold_gate(
            hold_status='CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', **kwargs))


if __name__ == '__main__':
    unittest.main()
