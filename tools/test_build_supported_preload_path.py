import copy
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools import build_supported_preload_path as path
from tools.offline_box_rise_candidate import _ik_near, _point


class PreloadPathTests(unittest.TestCase):
    def setUp(self):
        self.initial = {str(i): -.8 if i in (1, 4, 7, 10) else 0.
                        for i in range(1, 13)}
        self.signs = {str(i): -1 if i % 2 else 1 for i in range(1, 13)}
        self.raw = {str(i): 6.1 for i in range(1, 13)}
        self.samples = []
        for tick in range(path.TICKS):
            t = tick*path.PERIOD
            model = {i: q+math.radians(.6)*path.rise_fraction(t)
                     for i, q in self.initial.items()}
            raw = {i: self.raw[i]+self.signs[i]*(q-self.initial[i])
                   for i, q in model.items()}
            self.samples.append(dict(time_s=t, q_model_rad_by_id=model,
                                     rise_fraction=path.rise_fraction(t),
                                     q_raw_rad_by_id=raw))

    def audit(self, rows):
        return path.audit_samples(rows, self.initial, self.signs, self.raw)

    def test_finite_return_preserves_both_encoder_signs(self):
        result = self.audit(self.samples)
        self.assertAlmostEqual(math.degrees(result['displacement_rad']), .6)
        self.assertLess(result['speed_rad_s'], path.MAX_SPEED)
        self.assertLess(result['acceleration_rad_s2'], path.MAX_ACCELERATION)

    def test_branch_shift_and_missing_axis_are_rejected(self):
        rows = copy.deepcopy(self.samples)
        rows[50]['q_raw_rad_by_id']['1'] -= 2*math.pi
        with self.assertRaisesRegex(ValueError, 'encoder branch'):
            self.audit(rows)
        rows = copy.deepcopy(self.samples)
        del rows[50]['q_model_rad_by_id']['12']
        with self.assertRaisesRegex(ValueError, 'twelve'):
            self.audit(rows)

    def test_position_step_is_rejected_even_within_displacement_limit(self):
        rows = copy.deepcopy(self.samples)
        for tick in range(51, 100):
            rows[tick]['q_model_rad_by_id']['1'] += math.radians(.1)
            rows[tick]['q_raw_rad_by_id']['1'] -= math.radians(.1)
        with self.assertRaisesRegex(ValueError, 'speed cap'):
            self.audit(rows)

    def test_partial_path_and_invalid_times_are_rejected(self):
        with self.assertRaises(ValueError):
            self.audit(self.samples[:-1])
        for t in (-.1, 5.1, float('nan')):
            with self.assertRaises(ValueError):
                path.rise_fraction(t)

    def test_micro_target_is_not_lost_to_old_ik_tolerance(self):
        geometry = dict(hip=(0., 0., 0.), offset_y=.04, upper_m=.12, lower_m=.12)
        start = (-.8, .4, 0.)
        point = _point(start, geometry)
        target = (point[0], point[1], point[2]-2e-6)
        self.assertEqual(_ik_near(start, target, geometry), start)
        solved = _ik_near(start, target, geometry, tolerance_m=1e-9)
        self.assertNotEqual(solved, start)
        self.assertLess(math.dist(_point(solved, geometry), target), 1e-9)

    def test_missing_initial_axis_and_invalid_sign_rejected(self):
        self.signs['1'] = 0
        with self.assertRaisesRegex(ValueError, 'calibration'):
            self.audit(self.samples)
        del self.initial['12']
        with self.assertRaisesRegex(ValueError, 'twelve'):
            self.audit(self.samples)

    def test_stationary_pd_checks_interior_not_only_return_endpoint(self):
        axes = {i: dict(kp=6., max_estimated_pd_torque_nm=.2) for i in self.initial}
        self.assertAlmostEqual(path.audit_stationary_pd(self.samples, self.initial, axes),
                               6*math.radians(.6))
        axes['1']['max_estimated_pd_torque_nm'] = .01
        with self.assertRaisesRegex(ValueError, 'stationary PD'):
            path.audit_stationary_pd(self.samples, self.initial, axes)

    def test_nonfinite_time_fraction_and_boolean_sign_rejected(self):
        for field in ('time_s', 'rise_fraction'):
            rows = copy.deepcopy(self.samples)
            rows[100][field] = float('nan')
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'finite'):
                self.audit(rows)
        self.signs['2'] = True
        with self.assertRaisesRegex(ValueError, 'calibration'):
            self.audit(self.samples)

    def test_changed_schedule_or_start_hold_rejected(self):
        rows = copy.deepcopy(self.samples)
        rows[100]['rise_fraction'] = 0.
        with self.assertRaisesRegex(ValueError, 'schedule'):
            self.audit(rows)
        rows = copy.deepcopy(self.samples)
        rows[2]['q_model_rad_by_id']['1'] += .00001
        rows[2]['q_raw_rad_by_id']['1'] -= .00001
        with self.assertRaisesRegex(ValueError, 'captured-pose hold'):
            self.audit(rows)

    def test_pd_audit_rejects_nonfinite_targets_even_for_zero_gain(self):
        axes = {i: dict(kp=0., max_estimated_pd_torque_nm=.2) for i in self.initial}
        rows = copy.deepcopy(self.samples)
        rows[100]['q_model_rad_by_id']['1'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'finite'):
            path.audit_stationary_pd(rows, self.initial, axes)

    def test_file_only_build_keeps_screen_failure_and_provenance(self):
        geometry = {leg: dict(ids=ids, hip=(0., 0., 0.), offset_y=.04,
                             upper_m=.12, lower_m=.12) for leg, ids in path.LEGS.items()}
        base = dict(initial_model_rad_by_id=self.initial,
                    initial_raw_rad_by_id=self.raw, profile_sha256='a'*64,
                    readiness_blockers=['new power epoch needs binding'],
                    screen_failures=['source screen has an unresolved numerical failure'])
        profile = dict(profile_sha256='a'*64, axes={
            i: dict(sign=self.signs[i], kp=6., max_estimated_pd_torque_nm=.2)
            for i in self.initial})
        with tempfile.TemporaryDirectory() as directory:
            inputs = [Path(directory)/name for name in ('profile', 'capture', 'urdf')]
            for source in inputs:
                source.write_bytes(b'fixture')
            with (patch.object(path, 'plan', return_value=base),
                  patch.object(path, 'load_profile', return_value=profile),
                  patch.object(path, 'parse_d17', return_value=geometry)):
                result = path.build(*inputs)
                self.assertEqual(result['status'], 'FILE_ONLY_PATH_SCREEN_FAILED')
                self.assertTrue(result['kinematic_path_checks_passed'])
                self.assertFalse(result['source_screen_passed'])
                for flag in ('motor_output_allowed', 'approved_for_runtime',
                             'learned_model_output', 'box_removal_allowed'):
                    self.assertFalse(result[flag])
                self.assertIn(base['screen_failures'][0], result['blockers'])
                self.assertIn(str(Path(path.__file__).resolve()), result['source_fingerprints'])
                base['screen_failures'] = []
                result = path.build(*inputs)
                self.assertEqual(result['status'], 'FILE_ONLY_PATH_REVIEW_REQUIRED')
                self.assertFalse(result['source_screen_passed'])


if __name__ == '__main__':
    unittest.main()
