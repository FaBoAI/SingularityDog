"""Finite path mathematics only: no runtime arming, bus access or STOP proof."""
import copy
from dataclasses import FrozenInstanceError
import math
import unittest

from singularitydog_hw import supported_preload_path as preload


def fixture():
    initial = {str(mid): -.8 if mid in (1, 4, 7, 10) else 0. for mid in range(1, 13)}
    axes = {mid: dict(sign=-1 if int(mid) % 2 else 1, offset_rad=.1,
                     lower_rad=-1.1, upper_rad=1.1, kp=6.,
                     max_estimated_pd_torque_nm=.2,
                     max_displacement_from_start_rad=math.radians(1.),
                     max_command_velocity_rad_s=math.radians(1.),
                     max_command_acceleration_rad_s2=math.radians(5.))
            for mid in initial}
    raw_start = {mid: axes[mid]['sign']*(q-axes[mid]['offset_rad'])+2*math.pi
                 for mid, q in initial.items()}
    samples = []
    for slot in range(251):
        t = slot*.02
        if t <= 1 or t >= 4:
            value = 0.
        elif t <= 2.25:
            x = (t-1)/1.25
            value = x**3*(10+x*(-15+6*x))
        elif t <= 2.75:
            value = 1.
        else:
            x = (4-t)/1.25
            value = x**3*(10+x*(-15+6*x))
        q = {mid: start+.006*value for mid, start in initial.items()}
        raw = {mid: raw_start[mid]+axes[mid]['sign']*(v-initial[mid]) for mid, v in q.items()}
        samples.append(dict(time_s=t, rise_fraction=value,
                            q_model_rad_by_id=q, q_raw_rad_by_id=raw))
    data = dict(schema='singularitydog.supported-preload-path-file-only.v1',
                duration_s=5., period_s=.02, motor_output_allowed=False,
                approved_for_runtime=False, learned_model_output=False,
                box_removal_allowed=False, source_screen=dict(
                    screen_failures=[], motor_output_allowed=False, rise_mm=.25,
                    initial_model_rad_by_id=initial, initial_raw_rad_by_id=raw_start),
                samples=samples)
    return data, {'axes': axes}


class SupportedPreloadPathTests(unittest.TestCase):
    def setUp(self):
        self.data, self.profile = fixture()

    def path(self):
        return preload.validate_path(self.data, self.profile)

    def test_finite_extension_returns_and_retains_both_raw_signs_and_turn(self):
        path = self.path()
        bound = path.bind(path.initial_model, path.initial_raw)
        targets = [bound.target_for_slot(slot) for slot in range(251)]
        self.assertEqual(targets[0], path.initial_model)
        self.assertEqual(targets[200:], [path.initial_model]*51)
        self.assertTrue(bound.return_complete)
        self.assertTrue(all(abs(q-q0-.006) < 1e-12
                            for q, q0 in zip(targets[120], path.initial_model)))
        self.assertEqual(path.signs, tuple(-1 if mid % 2 else 1 for mid in range(1, 13)))
        with self.assertRaisesRegex(ValueError, 'exhausted'):
            bound.target_for_slot(251)

    def test_all_candidate_permission_flags_must_stay_false(self):
        for key in ('motor_output_allowed', 'approved_for_runtime',
                    'learned_model_output', 'box_removal_allowed'):
            with self.subTest(key=key):
                changed = copy.deepcopy(self.data)
                changed[key] = True
                with self.assertRaisesRegex(ValueError, 'file-only'):
                    preload.validate_path(changed, self.profile)

    def test_partial_nonfinite_or_branch_shifted_path_rejected(self):
        for kind in ('partial', 'nonfinite', 'branch'):
            with self.subTest(kind=kind):
                changed = copy.deepcopy(self.data)
                if kind == 'partial':
                    changed['samples'].pop()
                elif kind == 'nonfinite':
                    changed['samples'][70]['q_model_rad_by_id']['1'] = float('nan')
                else:
                    changed['samples'][70]['q_raw_rad_by_id']['1'] += 2*math.pi
                with self.assertRaises(ValueError):
                    preload.validate_path(changed, self.profile)

    def test_repeated_reversed_or_skipped_slots_are_rejected(self):
        for next_slot in (0, 2, -1):
            with self.subTest(next_slot=next_slot):
                path = self.path()
                bound = path.bind(path.initial_model, path.initial_raw)
                bound.target_for_slot(0)
                with self.assertRaisesRegex(ValueError, 'slot'):
                    bound.target_for_slot(next_slot)

    def test_small_origin_difference_is_explicit_and_larger_drift_rejected(self):
        path = self.path()
        delta = math.radians(.025)
        model = tuple(q+delta for q in path.initial_model)
        raw = tuple(r+s*delta for r, s in zip(path.initial_raw, path.signs))
        bound = path.bind(model, raw)
        self.assertTrue(all(abs(d-delta) < 1e-12 for d in bound.anchor_difference_rad))
        self.assertEqual(bound.target_for_slot(0), model)
        for shift in (math.radians(.1), 2*math.pi):
            with self.subTest(shift=shift):
                changed = (path.initial_raw[0]+shift, *path.initial_raw[1:])
                with self.assertRaisesRegex(ValueError, 'origin mismatch|branch changed'):
                    path.bind(path.initial_model, changed)

    def test_model_raw_disagreement_rejected_even_when_each_difference_is_small(self):
        path = self.path()
        model = (path.initial_model[0]+.0001, *path.initial_model[1:])
        with self.assertRaisesRegex(ValueError, 'raw/model origin disagree'):
            path.bind(model, path.initial_raw)

    def test_every_shifted_target_is_checked_against_reviewed_bounds(self):
        self.profile['axes']['1']['upper_rad'] = -.8+.006+.0001
        path = self.path()
        delta = .0002
        model = (path.initial_model[0]+delta, *path.initial_model[1:])
        raw = (path.initial_raw[0]+path.signs[0]*delta, *path.initial_raw[1:])
        with self.assertRaisesRegex(ValueError, 'fresh-anchored target outside'):
            path.bind(model, raw)

    def test_path_mutation_after_validation_cannot_change_loaded_targets(self):
        path = self.path()
        self.data['samples'][120]['q_model_rad_by_id']['1'] = 0.
        self.profile['axes']['1']['kp'] = 12.
        self.assertAlmostEqual(path.deltas[120][0], .006)
        self.assertEqual(path.kp[0], 6.)

    def test_bound_origin_path_and_cursor_cannot_be_reassigned(self):
        path = self.path()
        bound = path.bind(path.initial_model, path.initial_raw)
        bound.target_for_slot(0)
        for name, value in (('path', None), ('initial_model', (0.,)*12),
                            ('initial_raw', (0.,)*12), ('anchor_difference_rad', (0.,)*12),
                            ('next_slot', 0), ('_next_slot', 0),
                            ('return_complete', True), ('_return_complete', True)):
            with self.subTest(name=name), self.assertRaises(FrozenInstanceError):
                setattr(bound, name, value)
        self.assertEqual(bound.next_slot, 1)
        self.assertFalse(bound.return_complete)
        with self.assertRaisesRegex(ValueError, 'slot'):
            bound.target_for_slot(0)

    def test_profile_structure_and_missing_fields_fail_with_validation_error(self):
        for profile in (None, [], {'axes': {}}, {'axes': {**self.profile['axes'], '1': None}}):
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                preload.validate_path(self.data, profile)
        for field in self.profile['axes']['1']:
            profile = copy.deepcopy(self.profile)
            del profile['axes']['1'][field]
            with self.subTest(missing=field), self.assertRaises(ValueError):
                preload.validate_path(self.data, profile)

    def test_all_profile_numerical_fields_reject_bool_nonfinite_and_wrong_types(self):
        for field in self.profile['axes']['1']:
            for value in (True, None, '1', float('nan'), float('inf'), -float('inf'), 10**1000):
                profile = copy.deepcopy(self.profile)
                profile['axes']['1'][field] = value
                with self.subTest(field=field, value=str(value)[:30]), self.assertRaises(ValueError):
                    preload.validate_path(self.data, profile)

    def test_profile_motion_limits_are_positive_and_within_preload_scope(self):
        for field, maximum in (('max_displacement_from_start_rad', math.radians(1)),
                               ('max_command_velocity_rad_s', math.radians(1)),
                               ('max_command_acceleration_rad_s2', math.radians(5))):
            for value in (0., -1., maximum*1.001):
                profile = copy.deepcopy(self.profile)
                profile['axes']['1'][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    preload.validate_path(self.data, profile)
        self.profile['axes']['1']['upper_rad'] = self.profile['axes']['1']['lower_rad']
        with self.assertRaisesRegex(ValueError, 'model bounds'):
            self.path()

    def test_extreme_finite_calibration_does_not_raise_overflow(self):
        self.data['source_screen']['initial_model_rad_by_id']['1'] = 1e308
        self.profile['axes']['1']['offset_rad'] = -1e308
        with self.assertRaisesRegex(ValueError, 'calibration/path mismatch'):
            self.path()

    def test_bad_timing_and_stationary_pd_limits_fail_closed(self):
        self.data['samples'][70]['time_s'] += .001
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            self.path()
        self.data, self.profile = fixture()
        self.profile['axes']['1']['max_estimated_pd_torque_nm'] = .01
        with self.assertRaisesRegex(ValueError, 'PD estimate'):
            self.path()

    def test_start_return_and_slew_cannot_be_replaced_with_a_step(self):
        self.data['samples'][70]['q_model_rad_by_id']['1'] += .001
        self.data['samples'][70]['q_raw_rad_by_id']['1'] -= .001
        with self.assertRaisesRegex(ValueError, 'speed limit|acceleration limit'):
            self.path()


if __name__ == '__main__':
    unittest.main()
