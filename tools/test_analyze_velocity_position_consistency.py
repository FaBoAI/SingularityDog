"""Generated file fixtures and exact integration checks; no hardware/network."""
from contextlib import redirect_stdout
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import analyze_velocity_position_consistency as c
import analyze_stationary_velocity_probe as a
from test_analyze_stationary_velocity_probe import SavedFixtures, BOOT


def series(times, velocities, positions, slots=None):
    return [{'slot_index': slot, 'time_ns': round(1_000_000_000+t*1e9), 'velocity_rad_s': v,
             'position_proxy_rad': q, 'position_before_rad': q-.001, 'position_after_rad': q+.001,
             'host_position_bracket_ns': 2_000_000, 'host_interpolation_fraction': .5,
             'sensor_sample_time_verified': False, 'velocity_ground_truth': False}
            for slot, t, v, q in zip(slots or range(len(times)), times, velocities, positions)]


class IntegrationTests(unittest.TestCase):
    def test_reported_level_grid_and_exact_repeats_do_not_certify_sensor_resolution(self):
        data=series([0,.02,.04,.06],[.01,.01,.02,.01],[3,3.001,3.002,3.001])
        result=c.analyze_series(data)['signal_structure']
        self.assertEqual(result['reported_velocity_levels']['unique_value_count'],2)
        self.assertAlmostEqual(result['reported_velocity_levels']['adjacent_sorted_unique_spacing']['minimum'],.01)
        self.assertEqual(result['adjacent_identical_reported_velocity_pairs'],1)
        self.assertEqual(result['longest_identical_reported_velocity_run_samples'],2)
        self.assertFalse(result['reported_position_levels']['spacing_is_sensor_resolution_or_accuracy'])
        self.assertFalse(result['physical_cause_identified'] or result['internal_update_period_identified'])

    def test_irregular_host_sine_recovers_candidate_and_predicts_unused_half(self):
        times=[i*.05+.002*math.sin(i) for i in range(201)]
        velocities=[.03+.07*math.sin(2*math.pi*2.5*t+.2) for t in times]
        data=series(times,velocities,[3]*len(times))
        result=c.analyze_series(data)['signal_structure']['host_sinusoid_candidates']
        best=result['top_candidates'][0]
        self.assertLess(abs(best['frequency_hz']-2.5),result['frequency_grid_step_hz'])
        self.assertGreater(best['fraction_centered_sample_variance_explained'],.99)
        self.assertGreater(result['chronological_half_holdout']['held_out_fraction_variance_explained'],.98)
        self.assertFalse(result['chronological_half_holdout']['held_out_values_used_to_fit'])
        self.assertFalse(result['physical_period_identified'] or result['significance_test_performed'])
        self.assertFalse(result['acquisition_bandwidth_verified'] or result['unobserved_alias_excluded'])

    def test_signal_association_preserves_offsets_and_is_not_position_groundtruth(self):
        data=series([i*.02 for i in range(20)],[.01*i for i in range(20)],[3+.001*i for i in range(20)])
        before=c.analyze_series(data)['signal_structure']
        shifted=copy.deepcopy(data)
        for row in shifted:
            for key in ('position_proxy_rad','position_before_rad','position_after_rad'):row[key]+=10
        after=c.analyze_series(shifted)['signal_structure']
        self.assertAlmostEqual(before['velocity_Pearson_r_by_supplied_quantity']['host_position_proxy_rad'],1)
        self.assertAlmostEqual(after['velocity_Pearson_r_by_supplied_quantity']['host_position_proxy_rad'],1)
        self.assertFalse(after['position_derivative_is_ground_truth'])
        self.assertFalse(after['localization_or_timing_error_bound_created'])

    def test_velocity_distribution_at_same_reported_position_keeps_every_value(self):
        data=series([0,.02,.04,.06],[.03,-.04,.05,-.02],[3,3,4,3])
        groups=c.analyze_series(data)['signal_structure']['reported_velocity_by_exact_reported_position_before_level']
        self.assertEqual([r['sample_count'] for r in groups],[3,1])
        self.assertAlmostEqual(groups[0]['reported_velocity_rad_s']['minimum'],-.04)
        self.assertAlmostEqual(groups[0]['reported_velocity_rad_s']['maximum'],.03)
        self.assertAlmostEqual(groups[0]['reported_velocity_rad_s']['mean'],-.01)
        self.assertEqual(sum(r['sample_count'] for r in groups),len(data))

    def test_index_lags_keep_irregular_host_gaps_and_missing_slots_without_resampling(self):
        data=series([0,.02,.05,.09,.1,.16,.19,.25],[1,-1,1,-1,1,-1,1,-1],
                    [3]*8,[0,1,3,4,5,8,9,12])
        before=copy.deepcopy(data);result=c.analyze_series(data)
        structure=result['signal_structure'];lag=structure['chronological_velocity_autocorrelations'][1]
        self.assertEqual(lag['chronological_index_lag'],2);self.assertEqual(lag['pairs'],6)
        self.assertAlmostEqual(lag['velocity_Pearson_r'],1)
        self.assertAlmostEqual(lag['actual_host_elapsed_s']['minimum'],.05)
        self.assertAlmostEqual(lag['actual_host_elapsed_s']['maximum'],.09)
        self.assertEqual(result['intervals_crossing_missing_slots'],3)
        self.assertFalse(structure['host_sinusoid_candidates']['missing_values_interpolated'])
        self.assertEqual(data,before)

    def test_constant_empty_and_short_series_leave_period_and_correlations_unknown(self):
        for data in ([],series([0],[.02],[3]),
                     series([i*.02 for i in range(20)],[.02]*20,[3]*20)):
            result=c.analyze_series(data)['signal_structure']
            self.assertEqual(result['host_sinusoid_candidates']['top_candidates'],[])
            self.assertIsNone(result['host_sinusoid_candidates']['chronological_half_holdout'])
            self.assertIsNone(result['velocity_Pearson_r_by_supplied_quantity']['host_position_proxy_rad'])
            self.assertFalse(result['physical_cause_identified'])

    def test_constant_velocity_exact_nonuniform_intervals_and_offset_cancels(self):
        times = [0., .021, .043, .1, .15]
        data = series(times, [.25]*5, [3.+.25*t for t in times])
        result = c.analyze_series(data)
        self.assertAlmostEqual(result['net_sampled_velocity_integral_rad']['trapezoid'], .0375)
        self.assertAlmostEqual(result['net_host_proxy_position_change_rad'], .0375)
        self.assertLess(result['scale_and_sign_candidates']['declared_rad_s']['interval_residual_rad']['max_abs'], 1e-15)
        shifted = copy.deepcopy(data)
        for row in shifted:
            for key in ('position_proxy_rad', 'position_before_rad', 'position_after_rad'): row[key] += 100.
        after = c.analyze_series(shifted)
        self.assertAlmostEqual(after['net_host_proxy_position_change_rad'], result['net_host_proxy_position_change_rad'])
        self.assertFalse(after['scale_and_sign_candidates']['declared_rad_s']['approved'])
    def test_linear_velocity_trapezoid_exact_and_left_right_sensitivity_is_not_bound(self):
        times = [0., .02, .05, .09]
        result = c.analyze_series(series(times, [2*t for t in times], [t*t for t in times]))
        nets = result['net_sampled_velocity_integral_rad']
        self.assertAlmostEqual(nets['trapezoid'], .09**2)
        self.assertLess(nets['left'], nets['trapezoid']); self.assertGreater(nets['right'], nets['trapezoid'])
        self.assertFalse(result['integration_error_bound_verified'])
    def test_sign_and_reduction_candidates_are_diagnostic_only(self):
        times = [0., .02, .04, .06]
        result = c.analyze_series(series(times, [.775]*4, [-.1*t for t in times]))
        self.assertAlmostEqual(result['interval_scale_fit']['factor'], -1/7.75)
        self.assertFalse(result['interval_scale_fit']['approved'] or result['interval_scale_fit']['applied'])
        for candidate in result['scale_and_sign_candidates'].values():
            self.assertFalse(candidate['approved'] or candidate['applied'])
    def test_alias_can_generate_integral_position_disagreement_without_software_bug(self):
        period, amplitude = .02, .00002
        omega = 2*math.pi/period
        times = [i*period for i in range(30)]
        # Sampling at complete sine periods observes q≈0 and v>0 every time.
        velocities = [amplitude*omega*math.cos(omega*t) for t in times]
        positions = [amplitude*math.sin(omega*t) for t in times]
        result = c.analyze_series(series(times, velocities, positions))
        self.assertGreater(result['net_sampled_velocity_integral_rad']['trapezoid'], .003)
        self.assertLess(abs(result['net_host_proxy_position_change_rad']), 1e-15)
        self.assertFalse(result['velocity_ground_truth'] or result['integration_error_bound_verified'])
    def test_missing_slots_kept_and_no_zero_or_hold_fill(self):
        result = c.analyze_series(series([0., .04, .06], [1., -1., 1.], [0., 0., 0.], [0, 2, 3]))
        self.assertEqual(result['complete_sample_count'], 3)
        self.assertEqual(result['interval_count'], 2)
        self.assertEqual(result['intervals_crossing_missing_slots'], 1)
        self.assertEqual(result['intervals'][0]['missing_requested_slots_between'], 1)
        self.assertEqual(result['net_sampled_velocity_integral_rad']['trapezoid'], 0.)
    def test_empty_and_one_sample_do_not_invent_integral_or_scale(self):
        for data in ([], series([0.], [0.], [3.])):
            result = c.analyze_series(data)
            self.assertEqual(result['interval_count'], 0)
            self.assertIsNone(result['net_sampled_velocity_integral_rad'])
            self.assertIsNone(result['interval_scale_fit']['factor'])
            self.assertIsNone(result['net_host_proxy_position_change_rad'])
    def test_invalid_time_values_schema_and_sensor_approval_rejected(self):
        mutations = [lambda x: x[1].__setitem__('time_ns', x[0]['time_ns']),
                     lambda x: x[1].__setitem__('slot_index', 0),
                     lambda x: x[0].__setitem__('velocity_rad_s', math.nan),
                     lambda x: x[0].__setitem__('position_proxy_rad', True),
                     lambda x: x[0].__setitem__('unknown', False),
                     lambda x: x[0].__setitem__('sensor_sample_time_verified', True)]
        for mutation in mutations:
            data = series([0., .02], [0., 0.], [3., 3.]); mutation(data)
            with self.assertRaises(ValueError): c.analyze_series(data)


class SavedFileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.f = SavedFixtures(self.temp.name); self.f.create(20, 50); self.f.create(200, 5)
        self.path = self.f.directory/'comparison.json'
        self.path.write_text(json.dumps(self.f.compare(), indent=2)+'\n')
    def test_frozen_comparator_replays_all_artifacts_and_bounded_host_proxies(self):
        result = c.analyze(self.path, expected_boot_id=BOOT, motor_id=5)
        self.assertEqual(result['status'], 'HOST_CONSISTENCY_DESCRIBED_REVIEW_REQUIRED')
        fast = result['runs']['20']
        self.assertEqual(fast['requested_slots'], 50); self.assertEqual(fast['complete_triplets'], 48)
        self.assertEqual(fast['unacquired_slots'], [48, 49]); self.assertFalse(fast['data_coverage'])
        for analysis in fast['time_references'].values():
            self.assertEqual(analysis['interval_count'], 47)
            self.assertTrue(all(0 < r['host_interpolation_fraction'] < 1 for r in analysis['series']))
            self.assertFalse(analysis['sensor_sample_time_verified'] or analysis['velocity_ground_truth'])
        self.assertTrue(all(result[k] is False for k in c.FALSE_FLAGS))
    def test_independent_expected_boot_id_and_comparator_source_pin_required(self):
        with self.assertRaises(ValueError): c.analyze(self.path, expected_boot_id='99999999-2222-4333-8444-555555555555', motor_id=5)
        with self.assertRaises(ValueError): c.analyze(self.path, expected_boot_id=BOOT, motor_id=6)
        with patch.object(c, 'COMPARATOR_SHA256', 'a'*64):
            with self.assertRaises(ValueError): c.analyze(self.path, expected_boot_id=BOOT, motor_id=5)
    def test_input_binding_comparison_and_original_tampering_rejected(self):
        value = a.strict_json(self.path.read_bytes()); value['runs']['20']['complete_triplets'] += 1
        self.path.write_text(json.dumps(value))
        with self.assertRaises(ValueError): c.analyze(self.path, expected_boot_id=BOOT, motor_id=5)
        self.path.write_text(json.dumps(self.f.compare()))
        p = self.f.runs[0]['paths']['report']; original = p.read_bytes(); p.write_bytes(original+b' ')
        with self.assertRaises(ValueError): c.analyze(self.path, expected_boot_id=BOOT, motor_id=5)
    def test_incomplete_artifacts_remain_descriptive_and_not_successful(self):
        with tempfile.TemporaryDirectory() as directory:
            f = SavedFixtures(directory); f.create(20, 50, fail=3); f.create(200, 5)
            path = f.directory/'comparison.json'; path.write_text(json.dumps(f.compare()))
            result = c.analyze(path, expected_boot_id=BOOT, motor_id=5)
            self.assertEqual(result['status'], 'INCOMPLETE_CONSISTENCY_RECORDS')
            self.assertEqual(result['runs']['20']['complete_triplets'], 0)
            self.assertEqual(result['runs']['20']['incomplete_triplets'], 1)
            self.assertIsNone(result['runs']['20']['time_references']['raw_frame']['net_sampled_velocity_integral_rad'])
    def test_cli_file_only_no_original_edits_and_failure_flags(self):
        before = {p: p.read_bytes() for p in self.f.directory.iterdir()}
        argv = ['--comparison', str(self.path), '--expected-boot-id', BOOT, '--id', '5']
        with redirect_stdout(io.StringIO()) as out: self.assertEqual(c.main(argv), 0)
        self.assertFalse(json.loads(out.getvalue())['software_bug_confirmed'])
        self.assertEqual(before, {p: p.read_bytes() for p in self.f.directory.iterdir()})
        argv[-1] = '6'
        with redirect_stdout(io.StringIO()) as out: self.assertEqual(c.main(argv), 1)
        self.assertEqual(json.loads(out.getvalue())['status'], 'INVALID_ARTIFACTS')
        self.assertEqual(before, {p: p.read_bytes() for p in self.f.directory.iterdir()})


if __name__ == '__main__':
    unittest.main()
