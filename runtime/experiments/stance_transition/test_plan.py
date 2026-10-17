"""File-only tests of the stance transition plan; no device or model."""
import math
from pathlib import Path
import unittest

from . import plan as P

LIMITS = {mid: (math.radians(-126.), math.radians(68.)) for mid in range(1, 13)}
BOX_POSE_DEG = {6: 1.7, 5: 36.8, 4: -62.2, 3: .9, 2: 30., 1: -64.6,
                12: 13.7, 11: 36., 10: -50.5, 9: -9.6, 8: 46.5, 7: -60.6}
BOX_POSE = {mid: math.radians(deg) for mid, deg in BOX_POSE_DEG.items()}
TORCH_BUNDLE = Path(__file__).resolve().parents[4]/'RS05_30Hz_Model_20261013'


class PlanTests(unittest.TestCase):
    def test_full_transition_respects_caps_and_reaches_nominal(self):
        result = P.make_plan(BOX_POSE, LIMITS)
        self.assertEqual(result['status'], 'PLAN_ONLY_NO_DEVICE')
        self.assertFalse(result['output_allowed'])
        self.assertLessEqual(result['max_step_per_cycle_deg'], 10./30.+1e-9)
        for mid, rad in zip(P.CAN_ORDER, P.NOMINAL_STANCE_RAD):
            self.assertAlmostEqual(result['goal_by_id_deg'][str(mid)], math.degrees(rad), places=9)
        self.assertGreater(result['expected_body_rise_m'], 0.)
        waypoints = result['waypoints_by_id_rad']
        self.assertEqual(len(waypoints), result['moving_cycles']+result['holding_cycles']+1)
        self.assertEqual(waypoints[0], [BOX_POSE[mid] for mid in range(1, 13)])

    def test_fraction_scales_motion_and_rise(self):
        full = P.make_plan(BOX_POSE, LIMITS)
        tenth = P.make_plan(BOX_POSE, LIMITS, fraction=.1)
        self.assertAlmostEqual(tenth['max_joint_delta_deg'], full['max_joint_delta_deg']*.1, places=9)
        self.assertLess(abs(tenth['expected_body_rise_m']), abs(full['expected_body_rise_m']))
        self.assertLess(tenth['duration_s'], full['duration_s'])

    def test_rejects_bad_inputs_and_limits(self):
        with self.assertRaisesRegex(ValueError, 'twelve'):
            P.make_plan({1: 0.}, LIMITS)
        for fraction in (0., -.1, 1.1):
            with self.assertRaisesRegex(ValueError, 'Fraction'):
                P.make_plan(BOX_POSE, LIMITS, fraction=fraction)
        with self.assertRaisesRegex(ValueError, 'caps'):
            P.make_plan(BOX_POSE, LIMITS, max_velocity=math.radians(45.))
        narrow = dict(LIMITS)
        narrow[12] = (math.radians(-10.), math.radians(28.6))
        with self.assertRaisesRegex(ValueError, 'ID12 goal outside'):
            P.make_plan(BOX_POSE, narrow)

    @unittest.skipUnless((TORCH_BUNDLE/'model/policy_controller74_30hz.pt').exists(), 'private bundle not present')
    def test_nominal_and_height_match_frozen_controller(self):
        import torch
        controller = torch.jit.load(str(TORCH_BUNDLE/'model/policy_controller74_30hz.pt')).controller
        nominal = [float(x) for x in controller.nominal.reshape(12)]
        self.assertEqual(nominal, list(P.NOMINAL_STANCE_RAD))
        q = {mid: value for mid, value in zip(P.CAN_ORDER, nominal)}
        feet = controller.fk(controller.nominal.reshape(1, 12).to(controller.phase))
        origins = controller.origins
        expected = float(-(feet[0, :, 2]-origins[:, 2]).mean())
        self.assertAlmostEqual(P.body_height_m(q), expected, places=12)


if __name__ == '__main__':
    unittest.main()
