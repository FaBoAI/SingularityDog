"""Synthetic file-only optical fixtures; no images/hardware/private observations."""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import analyze_optical_joint_motion as optical


def reference(upper):
    return {'upper': upper, 'evidence': 'synthetic bounded fixture, not a physical calibration'}


def vector(angle, length=100, origin=(300, 300), radius=.1):
    x, y = origin
    return [{'xy_px': [x, y], 'max_error_px': None if radius is None else reference(radius)},
            {'xy_px': [x + length * math.cos(angle), y - length * math.sin(angle)],
             'max_error_px': None if radius is None else reference(radius)}]


def fixture(stator=(0, 0, 0), output=(0, .02, .04), times=(1, 1.02, 1.04)):
    v = optical.template(frame_count=len(times))
    v['record_kind'] = 'SYNTHETIC_FIXTURE'
    v['camera'].update(frame_dimensions_px=[1000, 1000], timestamp_clock='synthetic exposure clock',
        timestamp_definition='exposure_midpoint', nominal_frame_interval_s=.02)
    v['frames'] = [dict(index=i, state='CAPTURED', frame_id='synthetic-frame-' + str(i),
        image_sha256=hashlib.sha256(('synthetic-image-' + str(i)).encode()).hexdigest(), timestamp_s=t,
        timestamp_max_error_s=reference(.0001), exposure_s=.001, missing_reason=None,
        stator_points=vector(s), output_points=vector(o))
        for i, (s, o, t) in enumerate(zip(stator, output, times))]
    return v


def physical(v):
    v['geometry_conditions'] = dict.fromkeys(optical.CONDITIONS, True)
    for m in v['markers'].values(): m['rigid_attachment_confirmed'] = True
    v['per_frame_projection_error_bound_rad'] = reference(.002)
    v['image_to_joint_sign'] = {'value': 1, 'evidence': 'synthetic known viewing direction and positive joint axis'}
    return v


class OpticalTests(unittest.TestCase):
    def test_template_is_unmeasured_with_no_approval(self):
        r = optical.analyze(optical.template(joint_id=5, frame_count=4))
        self.assertEqual(r['coverage']['unmeasured_frames'], 4)
        self.assertEqual(r['coverage']['usable_projected_angles'], 0)
        self.assertFalse(r['coverage']['data_coverage_complete'])
        self.assertIsNone(r['absolute_origin_uncertainty_rad'])
        self.assertTrue(all(r[k] is False for k in optical.FLAGS))
        bad = optical.template(); bad['frames'][0]['timestamp_s'] = 1
        with self.assertRaises(ValueError): optical.analyze(bad)

    def test_both_rigid_bodies_move_relative_change_is_difference(self):
        r = optical.analyze(fixture(stator=(.1, .2, .3), output=(.3, .45, .6)))
        self.assertAlmostEqual(r['frames'][0]['projected_relative_angle_rad'], .2)
        self.assertAlmostEqual(r['frames'][2]['projected_relative_angle_rad'], .3)
        self.assertAlmostEqual(r['changes'][0]['projected_principal_displacement_rad'], .05)
        self.assertIsNone(r['changes'][0]['conditional_joint_displacement_circular_interval_rad'])
        self.assertIsNone(r['physical_velocity_rad_per_s'])

    def test_common_camera_rotation_translation_and_scale_cancel(self):
        # A moving camera rotates both observed marker vectors identically.
        a = optical.analyze(fixture(stator=(0, .5, 1), output=(.2, .7, 1.2)))
        self.assertTrue(all(abs(r['projected_relative_angle_rad'] - .2) < 1e-12 for r in a['frames']))
        self.assertTrue(all(abs(r['projected_principal_displacement_rad']) < 1e-12 for r in a['changes']))
        v = fixture(stator=(0, .5, 1), output=(.2, .7, 1.2))
        for i, f in enumerate(v['frames']):
            for key in ('stator_points', 'output_points'):
                for p in f[key]: p['xy_px'] = [20 + .7 * x for x in p['xy_px']]
        b = optical.analyze(v)
        for x, y in zip(a['frames'], b['frames']):
            self.assertAlmostEqual(x['projected_relative_angle_rad'], y['projected_relative_angle_rad'])
        self.assertFalse(b['physical_groundtruth_established'])

    def test_asin_bound_and_tangent_case_not_atan(self):
        pts = vector(0, radius=10)
        p = optical.points(pts, [1000, 1000])
        self.assertAlmostEqual(p['error_bound_rad'], math.asin(.2))
        self.assertGreater(p['error_bound_rad'], math.atan(.2))
        r = optical.analyze(fixture())
        self.assertAlmostEqual(r['frames'][0]['projected_relative_angle_interval_rad']['halfwidth_rad'], 2 * math.asin(.002))
        # Endpoint error disks permit the vector touching their combined disk.
        tangent_angle = math.asin(.2)
        candidate = (100 * math.cos(tangent_angle) ** 2,
                     100 * math.cos(tangent_angle) * math.sin(tangent_angle))
        self.assertAlmostEqual(math.hypot(candidate[0] - 100, candidate[1]), 20)
        self.assertAlmostEqual(math.atan2(candidate[1], candidate[0]), p['error_bound_rad'])

    def test_unknown_or_dominating_pixel_error_is_not_bounded(self):
        for radius in (None, 50, 60):
            v = fixture(); v['frames'][0]['stator_points'] = vector(0, radius=radius)
            r = optical.analyze(v)
            self.assertIsNotNone(r['frames'][0]['projected_relative_angle_rad'])
            self.assertIsNone(r['frames'][0]['projected_relative_angle_interval_rad'])
            self.assertIsNone(r['changes'][0]['conditional_no_extra_turn_projected_rate_interval_rad_per_s'])
            self.assertEqual(r['coverage']['bounded_projected_angles'], 2)

    def test_short_zero_markers_remain_in_denominator(self):
        for length in (0, .5, 1):
            v = fixture(); v['frames'][1]['output_points'] = vector(0, length=length)
            r = optical.analyze(v)
            self.assertIsNone(r['frames'][1]['projected_relative_angle_rad'])
            self.assertEqual(r['coverage']['captured_frames'], 3)
            self.assertEqual(r['coverage']['degenerate_captured_frames'], 1)
            self.assertEqual(r['changes'][0]['unusable_requested_frames_between'], 1)
            self.assertFalse(r['coverage']['data_coverage_complete'])

    def test_occluded_marker_keeps_captured_frame_and_missing_reason(self):
        v = fixture(); v['frames'][1]['output_points'] = None
        v['frames'][1]['missing_reason'] = 'output ordered marker pair occluded'
        r = optical.analyze(v)
        self.assertEqual(r['coverage']['captured_frames'], 3)
        self.assertEqual(r['coverage']['unobserved_marker_frames'], 1)
        self.assertEqual(r['coverage']['degenerate_captured_frames'], 0)
        self.assertEqual(r['coverage']['unusable_captured_frames'], 1)
        self.assertEqual(r['frames'][1]['image_sha256'], v['frames'][1]['image_sha256'])
        self.assertIsNone(r['frames'][1]['projected_relative_angle_rad'])
        self.assertEqual(r['changes'][0]['unusable_requested_frames_between'], 1)
        bad = copy.deepcopy(v); bad['frames'][1]['missing_reason'] = None
        with self.assertRaises(ValueError): optical.analyze(bad)

    def test_pi_wrap_is_circular_and_shortest_delta_not_scalar_span(self):
        v = fixture(output=(math.pi - .001, -math.pi + .001, -math.pi + .003))
        r = optical.analyze(v)
        self.assertEqual(len(r['frames'][0]['projected_relative_angle_interval_rad']['arcs_rad']), 2)
        self.assertAlmostEqual(r['changes'][0]['projected_principal_displacement_rad'], .002)
        self.assertFalse(r['changes'][0]['extra_turns_excluded'])
        self.assertFalse(r['alias_excluded'])
        # A displacement at the sign cut cannot have an unambiguous signed rate.
        v = fixture(output=(0, math.pi, math.pi))
        r = optical.analyze(v)
        self.assertIsNone(r['changes'][0]['conditional_no_extra_turn_projected_rate_interval_rad_per_s'])

    def test_conditional_geometry_needs_every_condition_and_separate_bound(self):
        v = physical(fixture()); r = optical.analyze(v)
        change = r['changes'][0]
        self.assertAlmostEqual(change['conditional_joint_displacement_circular_interval_rad']['halfwidth_rad'],
            change['projected_displacement_circular_interval_rad']['halfwidth_rad'] + .004)
        self.assertIsNone(r['absolute_joint_angle_rad'])
        self.assertTrue(all(r[k] is False for k in optical.FLAGS))
        for key in optical.CONDITIONS:
            bad = copy.deepcopy(v); bad['geometry_conditions'][key] = None
            self.assertIsNone(optical.analyze(bad)['changes'][0]['conditional_joint_displacement_circular_interval_rad'])
        for body in ('stator', 'output'):
            bad = copy.deepcopy(v); bad['markers'][body]['rigid_attachment_confirmed'] = False
            self.assertIsNone(optical.analyze(bad)['changes'][0]['conditional_joint_displacement_circular_interval_rad'])
        bad = copy.deepcopy(v); bad['per_frame_projection_error_bound_rad'] = None
        self.assertIsNone(optical.analyze(bad)['changes'][0]['conditional_joint_displacement_circular_interval_rad'])
        bad = copy.deepcopy(v); bad['image_to_joint_sign'] = None
        self.assertIsNone(optical.analyze(bad)['changes'][0]['conditional_joint_displacement_circular_interval_rad'])
        bad = copy.deepcopy(v); bad['image_to_joint_sign']['value'] = -1
        reversed_change = optical.analyze(bad)['changes'][0]
        self.assertAlmostEqual(reversed_change['conditional_joint_displacement_circular_interval_rad']['center_rad'],
                               -change['projected_principal_displacement_rad'])

    def test_missing_frame_never_filled_and_temporal_coverage_reported(self):
        v = fixture(); f = v['frames'][1]
        for k in f:
            if k != 'index': f[k] = None
        f.update(state='MISSING', missing_reason='camera did not provide requested frame')
        v['highest_motion_frequency_of_interest_hz'] = 20
        r = optical.analyze(v)
        self.assertEqual(r['coverage']['requested_frames'], 3)
        self.assertEqual(r['coverage']['captured_frames'], 2)
        self.assertEqual(r['coverage']['missing_frames'], 1)
        self.assertEqual(r['changes'][0]['unusable_requested_frames_between'], 1)
        self.assertFalse(r['changes'][0]['missing_frames_interpolated'])
        self.assertTrue(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertAlmostEqual(r['timing']['max_captured_frame_gap_s'], .04)
        self.assertTrue(r['timing']['measurement_bandwidth_unknown'])
        self.assertFalse(r['timing']['motion_bandlimited'])
        bad = copy.deepcopy(v); del bad['frames'][1]
        with self.assertRaises(ValueError): optical.analyze(bad)

    def test_unknown_time_bound_and_exposure_stay_unknown(self):
        v = fixture(); v['frames'][0]['timestamp_max_error_s'] = None; v['frames'][1]['exposure_s'] = None
        r = optical.analyze(v)
        self.assertIsNone(r['changes'][0]['elapsed_timestamp_interval_s'])
        self.assertIsNone(r['changes'][0]['conditional_no_extra_turn_projected_rate_interval_rad_per_s'])
        self.assertEqual(r['timing']['timestamp_error_unknown_frames'], 1)
        self.assertEqual(r['timing']['exposure_unknown_frames'], 1)
        self.assertTrue(r['timing']['clock_alignment_unknown'])
        self.assertFalse(r['timing']['sensor_acquisition_time_verified'])
        v = fixture(); v['frames'][0]['timestamp_max_error_s'] = reference(.05)
        r = optical.analyze(v)
        self.assertLess(r['changes'][0]['elapsed_timestamp_interval_s'][0], 0)
        self.assertIsNone(r['changes'][0]['conditional_no_extra_turn_projected_rate_interval_rad_per_s'])

    def test_no_extra_turn_or_alias_can_be_resolved_by_constant_frames(self):
        v = fixture(output=(.2, .2, .2))
        r = optical.analyze(v)
        self.assertTrue(all(c['projected_principal_displacement_rad'] == 0 for c in r['changes']))
        # Both static and one full turn per exposure interval fit these points.
        self.assertFalse(r['alias_excluded']); self.assertFalse(r['physical_stationarity_proven'])
        self.assertIsNone(r['physical_velocity_rad_per_s'])
        # At exactly half the sample rate a sine may vanish at every exposure.
        v = fixture(output=(0, 0, 0), times=(0, 1, 2))
        v['highest_motion_frequency_of_interest_hz'] = .5
        self.assertIsNone(optical.analyze(v)['timing']['spacing_fails_nyquist_for_declared_interest'])

    def test_sampling_conclusion_uses_timestamp_intervals_instead_of_centers(self):
        v = fixture(); v['highest_motion_frequency_of_interest_hz'] = 20
        r = optical.analyze(v)
        self.assertFalse(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        intervals = r['timing']['captured_frame_intervals']
        self.assertEqual([(g['from_index'], g['to_index']) for g in intervals], [(0, 1), (1, 2)])
        self.assertAlmostEqual(intervals[0]['elapsed_timestamp_interval_s'][0], .0198)
        self.assertAlmostEqual(intervals[0]['elapsed_timestamp_interval_s'][1], .0202)
        # Centers satisfy the spacing, but their evidenced uncertainties do not
        # establish which side of the required .025 s gap the acquisition used.
        for row in v['frames']: row['timestamp_max_error_s'] = reference(.003)
        r = optical.analyze(v)
        self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertAlmostEqual(r['timing']['captured_frame_intervals'][0]['elapsed_timestamp_interval_s'][1], .026)
        v['highest_motion_frequency_of_interest_hz'] = 40
        self.assertTrue(optical.analyze(v)['timing']['spacing_fails_nyquist_for_declared_interest'])

    def test_unknown_time_error_cannot_establish_sampling_spacing(self):
        v = fixture(); v['highest_motion_frequency_of_interest_hz'] = 20
        v['frames'][1]['timestamp_max_error_s'] = None
        r = optical.analyze(v)
        self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertTrue(all(g['elapsed_timestamp_interval_s'] is None for g in r['timing']['captured_frame_intervals']))
        self.assertTrue(all(c['conditional_no_extra_turn_projected_rate_interval_rad_per_s'] is None for c in r['changes']))
        self.assertAlmostEqual(r['timing']['max_captured_frame_gap_s'], .02)

    def test_bounded_host_receipt_and_unknown_times_cannot_judge_acquisition_spacing(self):
        for definition in ('host_receipt', 'unknown'):
            for interest in (20, 40):
                with self.subTest(definition=definition, interest=interest):
                    v = fixture(); v['camera']['timestamp_definition'] = definition
                    v['highest_motion_frequency_of_interest_hz'] = interest
                    r = optical.analyze(v)
                    self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
                    self.assertFalse(r['timing']['acquisition_timestamp_definition_supplied'])
                    self.assertEqual(r['timing']['spacing_timestamp_definition'], definition)
                    # Receipt timestamp arithmetic remains available even with
                    # bounded host errors; it is not an acquisition-time bound.
                    self.assertAlmostEqual(r['timing']['max_captured_frame_gap_s'], .02)
                    self.assertAlmostEqual(r['timing']['captured_frame_intervals'][0]['elapsed_timestamp_interval_s'][0], .0198)
                    self.assertAlmostEqual(r['timing']['captured_frame_intervals'][0]['elapsed_timestamp_interval_s'][1], .0202)
                    self.assertTrue(all(r[k] is False for k in optical.FLAGS))

    def test_each_supplied_acquisition_definition_keeps_conditional_spacing_behavior(self):
        for definition in ('exposure_midpoint', 'frame_start', 'frame_end'):
            for interest, fails in ((20, False), (40, True)):
                with self.subTest(definition=definition, interest=interest):
                    v = fixture(); v['camera']['timestamp_definition'] = definition
                    v['highest_motion_frequency_of_interest_hz'] = interest
                    r = optical.analyze(v)
                    self.assertIs(r['timing']['spacing_fails_nyquist_for_declared_interest'], fails)
                    self.assertTrue(r['timing']['acquisition_timestamp_definition_supplied'])
                    self.assertFalse(r['timing']['sensor_acquisition_time_verified'])
                    self.assertTrue(all(r[k] is False for k in optical.FLAGS))

    def test_irregular_host_receipt_gap_remains_descriptor_with_unknown_metrology(self):
        # Synthetic replay of the observed gap shape, not image/exposure evidence.
        v = fixture(times=(1, 1.0325072635, 1.1153127805))
        v['camera'].update(timestamp_clock='synthetic host monotonic receipt',
                           timestamp_definition='host_receipt', nominal_frame_interval_s=1/30)
        v['highest_motion_frequency_of_interest_hz'] = 15
        for row in v['frames']:
            row['exposure_s'] = None
            row['timestamp_max_error_s'] = None
            for pair in ('stator_points', 'output_points'):
                for point in row[pair]:
                    point['max_error_px'] = None
        r = optical.analyze(v)
        self.assertAlmostEqual(r['timing']['max_captured_frame_gap_s'], .082805517)
        self.assertAlmostEqual(r['timing']['nominal_spacing_nyquist_hz_only'], 15)
        self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertEqual(r['timing']['exposure_unknown_frames'], 3)
        self.assertTrue(r['timing']['measurement_bandwidth_unknown'])
        self.assertEqual(r['coverage']['bounded_projected_angles'], 0)
        self.assertTrue(all(g['elapsed_timestamp_interval_s'] is None for g in r['timing']['captured_frame_intervals']))
        self.assertIsNone(r['physical_velocity_rad_per_s'])
        self.assertTrue(all(r[k] is False for k in optical.FLAGS))

    def test_output_preserves_ordered_coordinates_and_bound_evidence_without_certification(self):
        v = fixture()
        v['markers']['stator']['point_ids'] = ['silver-left-candidate', 'silver-right-candidate']
        v['markers']['output']['point_ids'] = ['black-top-candidate', 'black-bottom-candidate']
        v['frames'][0]['stator_points'][0]['max_error_px']['evidence'] = 'synthetic supplied maximum, not certified'
        v['frames'][0]['output_points'][1]['max_error_px'] = None
        r = optical.analyze(v)
        for body in ('stator', 'output'):
            saved = r['frames'][0]['supplied_' + body + '_points']
            self.assertEqual([p['point_id'] for p in saved], v['markers'][body]['point_ids'])
            self.assertEqual([{k: p[k] for k in ('xy_px', 'max_error_px')} for p in saved],
                             v['frames'][0][body + '_points'])
        self.assertFalse(r['marker_identity_verified'])
        self.assertFalse(r['uncertainty_bounds_verified'])
        original_coordinate = r['frames'][0]['supplied_stator_points'][0]['xy_px'][0]
        v['frames'][0]['stator_points'][0]['xy_px'][0] += 10
        v['frames'][0]['stator_points'][0]['max_error_px']['evidence'] = 'changed after analysis'
        v['markers']['stator']['point_ids'][0] = 'changed-id-after-analysis'
        saved = r['frames'][0]['supplied_stator_points'][0]
        self.assertEqual(saved['xy_px'][0], original_coordinate)
        self.assertEqual(saved['max_error_px']['evidence'], 'synthetic supplied maximum, not certified')
        self.assertEqual(saved['point_id'], 'silver-left-candidate')

    def test_point_provenance_survives_occlusion_and_degenerate_geometry(self):
        v = fixture(); v['frames'][1]['output_points'] = None
        v['frames'][1]['missing_reason'] = 'candidate output pair occluded'
        v['frames'][2]['stator_points'] = vector(0, length=0, radius=None)
        r = optical.analyze(v)
        self.assertIsNone(r['frames'][1]['supplied_output_points'])
        self.assertIsNotNone(r['frames'][1]['supplied_stator_points'])
        self.assertEqual(r['frames'][2]['supplied_stator_points'][0]['xy_px'],
                         r['frames'][2]['supplied_stator_points'][1]['xy_px'])
        self.assertIsNone(r['frames'][2]['projected_relative_angle_rad'])
        plan = optical.analyze(optical.template(frame_count=1))
        self.assertIsNone(plan['frames'][0]['supplied_stator_points'])
        self.assertIsNone(plan['frames'][0]['supplied_output_points'])

    def test_one_bounded_failing_gap_remains_evidence_despite_an_unknown_gap(self):
        v = fixture(); v['highest_motion_frequency_of_interest_hz'] = 40
        v['frames'][0]['timestamp_s'] = None
        v['frames'][0]['timestamp_max_error_s'] = None
        r = optical.analyze(v)
        self.assertTrue(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertIsNone(r['timing']['captured_frame_intervals'][0]['elapsed_timestamp_s'])
        self.assertIsNone(r['timing']['captured_frame_intervals'][0]['elapsed_timestamp_interval_s'])
        self.assertFalse(r['alias_excluded'])

    def test_spacing_for_captured_pairs_does_not_restore_missing_endpoint_coverage(self):
        v = fixture(); v['highest_motion_frequency_of_interest_hz'] = 20
        row = v['frames'][0]
        for key in row:
            if key != 'index': row[key] = None
        row.update(state='MISSING', missing_reason='first requested exposure unavailable')
        r = optical.analyze(v)
        self.assertEqual(r['timing']['spacing_scope'], 'ADJACENT_CAPTURED_FRAME_TIMESTAMP_INTERVALS_ONLY')
        self.assertFalse(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertEqual([(g['from_index'], g['to_index']) for g in r['timing']['captured_frame_intervals']], [(1, 2)])
        self.assertFalse(r['coverage']['data_coverage_complete'])
        self.assertFalse(r['physical_stationarity_proven'])

    def test_tiny_interest_is_serializable_and_uncertain_time_order_is_inconclusive(self):
        v = fixture(); v['highest_motion_frequency_of_interest_hz'] = 1e-320
        r = optical.analyze(v)
        self.assertFalse(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        json.dumps(r, allow_nan=False)
        for row in v['frames']: row['timestamp_max_error_s'] = reference(.02)
        r = optical.analyze(v)
        self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        self.assertTrue(all(c['conditional_no_extra_turn_projected_rate_interval_rad_per_s'] is None for c in r['changes']))
        json.dumps(r, allow_nan=False)

    def test_conditional_pixel_bound_contains_perturbed_endpoint_directions(self):
        v = fixture(stator=(.7, .7, .7), output=(-.8, -.8, -.8))
        row = v['frames'][0]
        observed = optical.analyze(v)['frames'][0]['projected_relative_angle_interval_rad']
        for index in range(64):
            angles = []
            for body, phase in (('stator_points', .3), ('output_points', 1.2)):
                p0, p1 = row[body]
                perturb = []
                for endpoint, point in enumerate((p0, p1)):
                    phi = 2 * math.pi * index / 64 + phase + endpoint * 1.7
                    x, y = point['xy_px']; radius = point['max_error_px']['upper']
                    perturb.append((x + radius * math.cos(phi), y + radius * math.sin(phi)))
                (x0, y0), (x1, y1) = perturb
                angles.append(math.atan2(-(y1 - y0), x1 - x0))
            true_relative = optical.wrap(angles[1] - angles[0])
            self.assertLessEqual(abs(optical.wrap(true_relative - observed['center_rad'])), observed['halfwidth_rad'] + 1e-12)

    def test_unknown_frame_time_keeps_angle_with_null_rates(self):
        v = fixture(); v['frames'][1]['timestamp_s'] = None; v['frames'][1]['timestamp_max_error_s'] = None
        v['highest_motion_frequency_of_interest_hz'] = 1
        r = optical.analyze(v)
        self.assertEqual(r['coverage']['usable_projected_angles'], 3)
        self.assertIsNotNone(r['frames'][1]['projected_relative_angle_rad'])
        self.assertEqual(r['timing']['timestamp_unknown_frames'], 1)
        self.assertIsNone(r['timing']['captured_span_s'])
        self.assertIsNone(r['timing']['observed_average_frame_rate_hz'])
        self.assertIsNone(r['timing']['spacing_fails_nyquist_for_declared_interest'])
        for c in r['changes']:
            self.assertIsNone(c['elapsed_timestamp_s'])
            self.assertIsNone(c['conditional_no_extra_turn_projected_rate_rad_per_s'])
            self.assertIsNotNone(c['projected_displacement_circular_interval_rad'])
        bad = copy.deepcopy(v); bad['frames'][1]['timestamp_max_error_s'] = reference(.001)
        with self.assertRaises(ValueError): optical.analyze(bad)

    def test_repeated_image_bytes_are_ambiguity_not_extra_independence_proof(self):
        v = fixture(); v['frames'][1]['image_sha256'] = v['frames'][0]['image_sha256']
        r = optical.analyze(v)
        self.assertEqual(r['coverage']['repeated_image_hash_frames'], 1)
        self.assertFalse(r['image_hashes_verified'])

    def test_strict_schema_types_nan_identity_and_limits(self):
        modifications = [lambda v: v.update(unknown=1),
            lambda v: v.update(joint_id=True), lambda v: v.update(requested_frame_count=5001),
            lambda v: v['geometry_conditions'].update(same_camera_viewpoint=1),
            lambda v: v.update(image_to_joint_sign={'value': True, 'evidence': 'synthetic'}),
            lambda v: v['markers']['output'].update(point_ids=v['markers']['stator']['point_ids']),
            lambda v: v['markers']['output'].update(rigid_body_id='stator'),
            lambda v: v['frames'][0].update(index=True),
            lambda v: v['frames'][1].update(timestamp_s=1),
            lambda v: v['frames'][1].update(frame_id='synthetic-frame-0'),
            lambda v: v['frames'][0].update(image_sha256='not-a-sha'),
            lambda v: v['frames'][0]['stator_points'][0].update(xy_px=[True, 5]),
            lambda v: v['frames'][0]['stator_points'][0].update(xy_px=[float('nan'), 5]),
            lambda v: v['frames'][0]['stator_points'][0].update(xy_px=[float('inf'), 5]),
            lambda v: v['frames'][0]['stator_points'][0].update(xy_px=[1000, 5]),
            lambda v: v['frames'][0]['stator_points'][0].update(max_error_px=reference(0)),
            lambda v: v['frames'][0]['stator_points'][0].update(max_error_px=reference(-1))]
        for change in modifications:
            v = fixture(); change(v)
            with self.subTest(change=change):
                with self.assertRaises(ValueError): optical.analyze(v)
        for raw in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{"x":1e999}'):
            with self.assertRaises(ValueError): optical.strict_json(raw)
        with self.assertRaises(ValueError): optical.analyze(optical.strict_json(json.dumps(fixture()).replace('1.02', '1e999')))

    def test_all_missing_and_single_frame_never_prove_static_or_bandwidth(self):
        v = fixture()
        for f in v['frames']:
            for k in f:
                if k != 'index': f[k] = None
            f.update(state='MISSING', missing_reason='no image received')
        r = optical.analyze(v)
        self.assertEqual(r['changes'], []); self.assertIsNone(r['timing']['captured_span_s'])
        self.assertFalse(r['coverage']['data_coverage_complete']); self.assertFalse(r['physical_stationarity_proven'])
        r = optical.analyze(fixture(stator=(0,), output=(0,), times=(1,)))
        self.assertEqual(r['changes'], []); self.assertIsNone(r['timing']['observed_average_frame_rate_hz'])

    def test_file_only_cli_binding_fresh_no_original_changes_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d).resolve()
            path = folder / 'synthetic.json'; raw = json.dumps(fixture()).encode(); path.write_bytes(raw)
            before = path.stat()
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf): self.assertEqual(optical.main(['--input', str(path)]), 0)
            result = json.loads(buf.getvalue())
            self.assertEqual(result['input_artifact']['sha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(result['input_artifact']['bytes'], len(raw))
            self.assertEqual(path.read_bytes(), raw); self.assertEqual(path.stat().st_mtime_ns, before.st_mtime_ns)
            self.assertEqual(list(folder.iterdir()), [path])
            linked = folder / 'link.json'; linked.symlink_to(path)
            with self.assertRaises(ValueError): optical.read_file(linked)
            directory_link = folder / 'linked-directory'; directory_link.symlink_to(folder, target_is_directory=True)
            with self.assertRaises(ValueError): optical.read_file(directory_link / path.name)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf): self.assertEqual(optical.main(['--template', '--frame-count', '4']), 0)
            self.assertEqual(json.loads(buf.getvalue())['requested_frame_count'], 4)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf): self.assertEqual(optical.main(['--input', str(linked)]), 2)
            self.assertEqual(json.loads(buf.getvalue())['status'], 'INVALID_OPTICAL_ARTIFACT')
            nested = folder / 'nested.json'; nested.write_text('[' * 5000 + '0' + ']' * 5000)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf): self.assertEqual(optical.main(['--input', str(nested)]), 2)
            self.assertEqual(json.loads(buf.getvalue())['status'], 'INVALID_OPTICAL_ARTIFACT')

    def test_input_rejects_explicit_template_options_including_default_values(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d).resolve() / 'joint-7-observations.json'
            v = fixture(); v['joint_id'] = 7
            raw = json.dumps(v).encode(); path.write_bytes(raw)
            for options in (['--joint-id', '5'], ['--frame-count', '3'],
                            ['--joint-id', '7'], ['--frame-count', '4']):
                with self.subTest(options=options):
                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        self.assertEqual(optical.main(['--input', str(path), *options]), 2)
                    self.assertEqual(json.loads(buf.getvalue())['status'], 'INVALID_OPTICAL_ARTIFACT')
            self.assertEqual(path.read_bytes(), raw)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                self.assertEqual(optical.main(['--template']), 0)
            self.assertEqual(json.loads(buf.getvalue())['joint_id'], 5)
            self.assertEqual(json.loads(buf.getvalue())['requested_frame_count'], 3)


if __name__ == '__main__':
    unittest.main()
