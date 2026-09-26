"""Pure mathematical transition tests; no learned policy, device, or transport."""
import copy
import json
import math
import unittest

from singularitydog_hw import stance_transition as transition


class StanceTransitionTests(unittest.TestCase):
    def setUp(self):
        self.current = [0., .1, -1.6]*4
        self.target = [-.2, .38, -.77, .2, .38, -.77]*2
        # Explicit illustrative mathematical limits, not hardware defaults.
        self.limits = dict(max_velocity_rad_s=math.radians(10),
                           max_acceleration_rad_s2=math.radians(20))

    def plan(self, **kwargs):
        return transition.build_plan(self.current, self.target, **(self.limits | kwargs))

    def test_exact_endpoints_zero_derivatives_and_no_output(self):
        current, target = copy.deepcopy(self.current), copy.deepcopy(self.target)
        result = self.plan()
        self.assertEqual(result['samples'][0]['q_model_rad'], current)
        self.assertEqual(result['samples'][-1]['q_model_rad'], target)
        for endpoint in (result['samples'][0], result['samples'][-1]):
            self.assertEqual(endpoint['dq_model_rad_s'], [0.]*12)
            self.assertEqual(endpoint['ddq_model_rad_s2'], [0.]*12)
        for name in ('output_allowed', 'live_runner_available', 'calibration_verified',
                     'physical_limits_verified', 'physical_standing_verified',
                     'policy_handoff_verified', 'raw_angle_conversion_available',
                     'automatic_segmentation', 'existing_trial_limits_changed'):
            self.assertIs(result[name], False)
        self.assertEqual(self.current, current)
        self.assertEqual(self.target, target)
        json.dumps(result, allow_nan=False)

    def test_analytic_derivative_bounds_and_20ms_sampling(self):
        result = self.plan()
        duration = result['duration_s']
        delta = max(abs(end-start) for start, end in zip(self.current, self.target))
        self.assertAlmostEqual(result['max_analytic_velocity_rad_s'], delta*1.875/duration)
        self.assertAlmostEqual(result['max_analytic_acceleration_rad_s2'],
                               delta*10*math.sqrt(3)/(3*duration*duration))
        self.assertLessEqual(result['max_analytic_velocity_rad_s'], self.limits['max_velocity_rad_s'])
        self.assertLessEqual(result['max_analytic_acceleration_rad_s2'], self.limits['max_acceleration_rad_s2'])
        for before, after in zip(result['samples'], result['samples'][1:]):
            self.assertEqual(after['elapsed_ns']-before['elapsed_ns'], 20_000_000)
            for a,b in zip(before['q_model_rad'], after['q_model_rad']):
                self.assertLessEqual(abs(b-a)/.02, self.limits['max_velocity_rad_s']+1e-12)
            for a,b in zip(before['dq_model_rad_s'], after['dq_model_rad_s']):
                self.assertLessEqual(abs(b-a)/.02, self.limits['max_acceleration_rad_s2']+1e-12)
        self.assertEqual(result['samples'][-1]['elapsed_ns'], result['duration_ns'])

    def test_acceleration_limit_can_determine_duration(self):
        result = self.plan(max_velocity_rad_s=100., max_acceleration_rad_s2=.01)
        delta = max(abs(end-start) for start,end in zip(self.current,self.target))
        required = math.sqrt(10*math.sqrt(3)*delta/(3*.01))
        self.assertGreaterEqual(result['duration_s'], required)
        self.assertLess(result['duration_s']-required, .02)
        self.assertLessEqual(result['max_analytic_acceleration_rad_s2'], .01)

    def test_all_samples_stay_between_endpoints_and_in_model_ranges(self):
        self.current = list(transition.LOWER)
        self.target = list(transition.UPPER)
        result = self.plan()
        for sample in result['samples']:
            for j,q in enumerate(sample['q_model_rad']):
                self.assertGreaterEqual(q, self.current[j])
                self.assertLessEqual(q, self.target[j])
        reversed_result = transition.build_plan(self.target,self.current,**self.limits)
        for sample in reversed_result['samples']:
            for j,q in enumerate(sample['q_model_rad']):
                self.assertGreaterEqual(q, self.current[j])
                self.assertLessEqual(q, self.target[j])

    def test_zero_distance_two_samples_and_zero_derivatives(self):
        self.target = list(self.current)
        result = self.plan()
        self.assertEqual(result['sample_count'], 2)
        self.assertEqual(result['duration_ns'], 20_000_000)
        self.assertEqual(result['max_analytic_velocity_rad_s'], 0.)
        self.assertEqual(result['max_analytic_acceleration_rad_s2'], 0.)

    def test_invalid_vectors_and_ranges_rejected(self):
        for bad in ([], self.current[:-1], self.current+[0.], None, 'angles'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                transition.build_plan(bad,self.target,**self.limits)
        for index,value in ((0,-.50001),(1,1.20001),(2,-.07999)):
            for field in ('current','target'):
                current,target = list(self.current),list(self.target)
                (current if field=='current' else target)[index] = value
                with self.subTest(field=field,index=index), self.assertRaises(ValueError):
                    transition.build_plan(current,target,**self.limits)

    def test_nonfinite_boolean_and_overflow_rejected(self):
        for value in (float('nan'),float('inf'),-float('inf'),True,False,'0',None,10**1000):
            for field in ('current','target','max_velocity_rad_s','max_acceleration_rad_s2'):
                current,target,limits = list(self.current),list(self.target),dict(self.limits)
                if field in ('current','target'):
                    (current if field=='current' else target)[0] = value
                else:
                    limits[field] = value
                with self.subTest(field=field,value=repr(value)), self.assertRaises(ValueError):
                    transition.build_plan(current,target,**limits)

    def test_positive_explicit_limits_required(self):
        for field in self.limits:
            for value in (0.,-1.):
                with self.subTest(field=field,value=value), self.assertRaises(ValueError):
                    self.plan(**{field:value})
        with self.assertRaises(TypeError):
            transition.build_plan(self.current,self.target)

    def test_excessive_samples_and_invalid_sample_budgets_rejected(self):
        for kwargs in ({'max_samples':2}, {'max_velocity_rad_s':1e-300},
                       {'max_acceleration_rad_s2':5e-324}):
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError,'sample count'):
                self.plan(**kwargs)
        for value in (True,1,0,-1,2.0,transition.MAX_SAMPLES+1,None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.plan(max_samples=value)


if __name__ == '__main__':
    unittest.main()
