"""Pure affine/quintic candidate tests; no transport or physical authorization."""
import copy
import json
import math
import unittest

from singularitydog_hw import raw_stance_transition as transition
from singularitydog_hw import stance_transition as model_transition


class RawStanceTransitionTests(unittest.TestCase):
    def setUp(self):
        order = transition.CAN_ORDER
        self.current_model = dict(zip(order, [0., .1, -1.6]*4))
        self.target = dict(zip(order, [-.2, .38, -.77, .2, .38, -.77]*2))
        self.signs = {i: 1 if i % 2 else -1 for i in range(1, 13)}
        self.raw = {i: 4.+i/10 for i in range(1, 13)}
        self.offsets = {i: self.current_model[i]-self.signs[i]*self.raw[i] for i in self.raw}
        self.options = dict(sign_by_id=self.signs, offset_rad_by_id=self.offsets,
                            max_velocity_rad_s=math.radians(10),
                            max_acceleration_rad_s2=math.radians(20))

    def plan(self, **changes):
        return transition.build_plan(self.raw, self.target, **(self.options | changes))

    def test_affine_roundtrip_signs_and_exact_endpoints_without_input_mutation(self):
        before = copy.deepcopy((self.raw, self.target, self.options))
        result = self.plan()
        for i in self.raw:
            self.assertAlmostEqual(result['current_model_rad_by_id'][i], self.current_model[i])
            delta = result['delta_raw_rad_by_id'][i]
            self.assertAlmostEqual(delta, self.signs[i]*(self.target[i]-self.current_model[i]))
        self.assertEqual(result['samples'][0]['q_raw_rad_by_id'], self.raw)
        self.assertEqual(result['samples'][-1]['q_model_rad_by_id'], self.target)
        self.assertEqual(result['samples'][-1]['q_raw_rad_by_id'], result['target_raw_rad_by_id'])
        for sample in result['samples']:
            restored = transition.model_to_raw(sample['q_model_rad_by_id'], sign_by_id=self.signs,
                                                offset_rad_by_id=self.offsets)
            converted = transition.raw_to_model(sample['q_raw_rad_by_id'], sign_by_id=self.signs,
                                                offset_rad_by_id=self.offsets)
            for i in self.raw:
                self.assertAlmostEqual(restored[i], sample['q_raw_rad_by_id'][i], places=13)
                self.assertAlmostEqual(converted[i], sample['q_model_rad_by_id'][i], places=13)
                self.assertAlmostEqual(sample['dq_raw_rad_s_by_id'][i], self.signs[i]*sample['dq_model_rad_s_by_id'][i])
        self.assertEqual((self.raw, self.target, self.options), before)
        json.dumps(result, allow_nan=False)

    def test_one_fixed_duration_with_50ms_sampling_and_continuous_derivative_bounds(self):
        result = self.plan()
        duration = result['duration_s']
        self.assertEqual(result['duration_ns'] % 50_000_000, 0)
        self.assertEqual(result['samples'][-1]['elapsed_ns'], result['duration_ns'])
        self.assertEqual(result['sample_count'], result['duration_ns']//50_000_000+1)
        for frame in ('raw', 'model'):
            deltas = result[f'delta_{frame}_rad_by_id']
            for i, delta in deltas.items():
                peak_v = abs(delta)*1.875/duration
                peak_a = abs(delta)*10*math.sqrt(3)/(3*duration**2)
                self.assertEqual(result['analytic_peaks_by_frame'][frame]['velocity_rad_s'][i], peak_v)
                self.assertAlmostEqual(result['analytic_peaks_by_frame'][frame]['acceleration_rad_s2'][i], peak_a)
                self.assertLessEqual(peak_v, self.options['max_velocity_rad_s'])
                self.assertLessEqual(peak_a, self.options['max_acceleration_rad_s2'])
            for first, second in zip(result['samples'], result['samples'][1:]):
                self.assertEqual(second['elapsed_ns']-first['elapsed_ns'], 50_000_000)
                for i in self.raw:
                    secant = abs(second[f'q_{frame}_rad_by_id'][i]-first[f'q_{frame}_rad_by_id'][i])/.05
                    acceleration = abs(second[f'dq_{frame}_rad_s_by_id'][i]-first[f'dq_{frame}_rad_s_by_id'][i])/.05
                    self.assertLessEqual(secant, self.options['max_velocity_rad_s']+1e-12)
                    self.assertLessEqual(acceleration, self.options['max_acceleration_rad_s2']+1e-12)
            for endpoint in (result['samples'][0], result['samples'][-1]):
                self.assertEqual(set(endpoint[f'dq_{frame}_rad_s_by_id'].values()), {0.})
                self.assertEqual(set(endpoint[f'ddq_{frame}_rad_s2_by_id'].values()), {0.})
        largest = max(abs(d) for d in result['delta_raw_rad_by_id'].values())
        needed = max(1.875*largest/self.options['max_velocity_rad_s'],
                     math.sqrt(10*math.sqrt(3)*largest/(3*self.options['max_acceleration_rad_s2'])))
        self.assertGreaterEqual(duration+1e-14, needed)
        self.assertLess(duration-needed, .05+1e-14)

    def test_outside_start_requires_opt_in_and_monotonic_unsupported_classification(self):
        self.raw[3] = self.signs[3]*(.9-self.offsets[3])
        self.raw[6] = self.signs[6]*(-.9-self.offsets[6])
        with self.assertRaisesRegex(ValueError, 'unsupported recovery'): self.plan()
        result = self.plan(allow_unsupported_recovery=True)
        self.assertEqual(result['status'], 'UNSUPPORTED_RECOVERY_CANDIDATE_ONLY')
        self.assertEqual(result['initial_out_of_model_range_ids'], [3, 6])
        self.assertTrue(result['monotonic_distance_to_model_range_required'])
        self.assertTrue(result['monotonic_distance_to_model_range_satisfied'])
        for i in self.raw:
            distances = [s['distance_to_model_range_rad_by_id'][i] for s in result['samples']]
            self.assertTrue(all(a >= b for a,b in zip(distances, distances[1:])))
            self.assertEqual(distances[-1], 0.)
        current_in_policy_order = [result['current_model_rad_by_id'][i] for i in transition.CAN_ORDER]
        with self.assertRaisesRegex(ValueError, 'outside policy model range'):
            model_transition.build_plan(current_in_policy_order, [self.target[i] for i in transition.CAN_ORDER],
                max_velocity_rad_s=self.options['max_velocity_rad_s'],
                max_acceleration_rad_s2=self.options['max_acceleration_rad_s2'])
        self.assertIs(result['output_allowed'], False)

    def test_no_shortest_angle_wrapping_is_applied(self):
        self.signs = {i:1 for i in self.raw}
        self.offsets = {i:self.current_model[i]-self.raw[i] for i in self.raw}
        self.raw[3], self.offsets[3], self.target[3] = 3.1, 3.1, 0.
        result = self.plan(sign_by_id=self.signs, offset_rad_by_id=self.offsets,
                           allow_unsupported_recovery=True)
        self.assertEqual(result['target_raw_rad_by_id'][3], -3.1)
        self.assertEqual(result['delta_raw_rad_by_id'][3], -6.2)
        self.assertIs(result['angle_wrapping_applied'], False)
        path = [s['q_raw_rad_by_id'][3] for s in result['samples']]
        self.assertTrue(all(a >= b for a,b in zip(path, path[1:])))
        self.assertGreater(result['duration_s'], 60.)

    def test_stationary_path_and_flags_never_authorize_a_runner(self):
        self.target = transition.raw_to_model(self.raw, sign_by_id=self.signs, offset_rad_by_id=self.offsets)
        # Exact affine values make both coordinate deltas zero for this case.
        self.raw = transition.model_to_raw(self.target, sign_by_id=self.signs, offset_rad_by_id=self.offsets)
        result = self.plan()
        self.assertEqual(result['status'], 'OFFLINE_RAW_STANCE_CANDIDATE_ONLY')
        self.assertEqual(result['sample_count'], 2)
        self.assertEqual(result['duration_ns'], 50_000_000)
        self.assertEqual(result['initial_out_of_model_range_ids'], [])
        for key in ('output_allowed', 'live_runner_available', 'policy_handoff_available',
                    'calibration_verified', 'identity_verified', 'physical_limits_verified',
                    'physical_standing_verified', 'collision_clearance_verified', 'automatic_retries',
                    'automatic_segmentation', 'existing_runtime_guards_changed'):
            self.assertIs(result[key], False, key)

    def test_exact_integer_ids_and_calibration_signs_are_required(self):
        for field in ('current', 'target', 'sign_by_id', 'offset_rad_by_id'):
            source = {'current':self.raw, 'target':self.target, **self.options}[field]
            variants = [None, list(source.values()), {k:v for k,v in source.items() if k != 12},
                        source | {13:0.}, {str(k):v for k,v in source.items()},
                        {float(k):v for k,v in source.items()},
                        {True if k == 1 else k:v for k,v in source.items()}]
            for bad in variants:
                args, options = [self.raw, self.target], dict(self.options)
                if field in ('current', 'target'): args[field == 'target'] = bad
                else: options[field] = bad
                with self.subTest(field=field,bad=bad), self.assertRaises(ValueError):
                    transition.build_plan(*args, **options)
        for bad_sign in (0, 2, -2, True, 1., -1.):
            with self.subTest(sign=bad_sign), self.assertRaises(ValueError):
                self.plan(sign_by_id=self.signs | {1:bad_sign})

    def test_invalid_targets_raw_ranges_and_nonfinite_fields_are_rejected(self):
        for i in self.raw:
            low, high = transition.MODEL_RANGES[i]
            for bad in (low-.00001, high+.00001):
                with self.subTest(i=i,target=bad), self.assertRaises(ValueError):
                    transition.build_plan(self.raw, self.target | {i:bad}, **self.options,
                                          allow_unsupported_recovery=True)
        for field in ('current', 'target', 'offset_rad_by_id', 'max_velocity_rad_s', 'max_acceleration_rad_s2'):
            for bad in (float('nan'), float('inf'), -float('inf'), True, '0', None, 10**1000):
                args, options = [self.raw, self.target], dict(self.options)
                if field in ('current', 'target'): args[field == 'target'] = args[field == 'target'] | {1:bad}
                elif field == 'offset_rad_by_id': options[field] = self.offsets | {1:bad}
                else: options[field] = bad
                with self.subTest(field=field,bad=repr(bad)), self.assertRaises(ValueError):
                    transition.build_plan(*args, **options)
        for raw in (-12.570001, 12.570001):
            with self.assertRaisesRegex(ValueError, 'protocol range'):
                transition.build_plan(self.raw | {1:raw}, self.target, **self.options,
                                      allow_unsupported_recovery=True)
        with self.assertRaisesRegex(ValueError, 'protocol range'):
            self.plan(offset_rad_by_id=self.offsets | {1:100.}, allow_unsupported_recovery=True)

    def test_explicit_positive_limits_boolean_opt_in_and_bounded_duration(self):
        for field in ('max_velocity_rad_s', 'max_acceleration_rad_s2'):
            for bad in (0., -1., 5e-324):
                with self.subTest(field=field,bad=bad), self.assertRaises(ValueError): self.plan(**{field:bad})
        for bad in (0, 1, None, 'true'):
            with self.assertRaises(ValueError): self.plan(allow_unsupported_recovery=bad)
        for bad in (True, 1, 2., 10_002, None):
            with self.assertRaises(ValueError): self.plan(max_samples=bad)
        with self.assertRaisesRegex(ValueError, 'sample count'): self.plan(max_samples=2)
        with self.assertRaises(TypeError):
            transition.build_plan(self.raw, self.target, sign_by_id=self.signs, offset_rad_by_id=self.offsets)


if __name__ == '__main__': unittest.main()
