"""Synthetic file-only data: no physical reading, hardware or network."""
from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest

import analyze_level_reversal as a


def fixture():
    value = a.template(); value['record_kind'] = 'SYNTHETIC_FIXTURE'; value['instrument_id'] = 'synthetic-level'
    cycle = value['cycles'][0]
    cycle.update(id='synthetic-cycle', datum_id='synthetic-datum', contact_patch_id='synthetic-patch', measuring_face_id='synthetic-bottom')
    cycle['conditions'] = dict.fromkeys(a.CONDITIONS, True)
    for row, reading in zip(cycle['readings'], ([2.8, 3.2], [-1.2, -.8], [2.7, 3.3])):
        row['displacement_interval'] = reading
    return value


class ReversalTests(unittest.TestCase):
    def test_blank_template_never_invents_precision(self):
        result = a.analyze(a.template()); row = result['cycles'][0]
        self.assertIsNone(row['conditional_linear_model_contrasts_units']['zero_indication'])
        self.assertIsNone(row['zero_bias_with_supplied_bounds_rad'])
        self.assertIsNone(row['surface_tilt_with_supplied_bounds_rad'])
        self.assertTrue(row['angular_sensitivity_unknown'] and row['seating_bound_unknown'])
        self.assertTrue(all(result[k] is False for k in a.FLAGS))
        self.assertIsNone(result['absolute_origin_error_rad'])
        self.assertEqual(result['record_kind'], 'PLANNED_UNMEASURED')
        self.assertFalse(result['synthetic_fixture_result'])

    def test_plan_cannot_claim_measured_values_or_operator_conditions(self):
        value = a.template(); value['cycles'][0]['readings'][0]['displacement_interval'] = [0, 1]
        with self.assertRaises(ValueError): a.analyze(value)
        value = a.template(); value['cycles'][0]['conditions']['same_footprint'] = True
        with self.assertRaises(ValueError): a.analyze(value)

    def test_instrument_axis_reversal_sign_and_return_variation(self):
        result = a.analyze(fixture())['cycles'][0]
        contrasts = result['conditional_linear_model_contrasts_units']
        self.assertAlmostEqual(contrasts['zero_indication'][0], .8)
        self.assertAlmostEqual(contrasts['zero_indication'][1], 1.2)
        self.assertAlmostEqual(contrasts['surface_indication'][0], 1.8)
        self.assertAlmostEqual(contrasts['surface_indication'][1], 2.2)
        self.assertAlmostEqual(result['observed_forward_return_difference_units'][0], -.5)
        self.assertAlmostEqual(result['observed_forward_return_difference_units'][1], .5)
        self.assertIsNone(result['zero_bias_with_supplied_seating_bound_units'])

    def test_known_positive_scale_and_seating_bound_are_only_input_conditioned(self):
        value = fixture()
        value['linear_reading_range'] = {'interval': [-5, 5], 'evidence': 'synthetic linearity bound'}
        value['angular_sensitivity_rad_per_unit'] = {'interval': [.001, .0011], 'evidence': 'synthetic sensitivity interval'}
        value['per_reading_seating_bound_units'] = {'upper': .1, 'evidence': 'synthetic per-reading bound'}
        result = a.analyze(value); row = result['cycles'][0]
        self.assertTrue(result['synthetic_fixture_result'])
        self.assertAlmostEqual(row['zero_bias_with_supplied_bounds_rad'][0], .0007)
        self.assertAlmostEqual(row['zero_bias_with_supplied_bounds_rad'][1], .00143)
        self.assertAlmostEqual(row['surface_tilt_with_supplied_bounds_rad'][0], .0017)
        self.assertAlmostEqual(row['surface_tilt_with_supplied_bounds_rad'][1], .00253)
        self.assertIsNone(result['joint_reference_angle_rad'])
        self.assertFalse(result['fit_observations_generated'] or result['calibration_approved'])

    def test_unknown_or_false_conditions_and_range_block_physical_separation(self):
        value = fixture(); value['per_reading_seating_bound_units'] = {'upper': .1, 'evidence': 'synthetic bound'}
        value['linear_reading_range'] = {'interval': [-5, 5], 'evidence': 'synthetic range'}
        for key in a.CONDITIONS:
            for missing in (None, False):
                with self.subTest(key=key, missing=missing):
                    changed = copy.deepcopy(value); changed['cycles'][0]['conditions'][key] = missing
                    self.assertIsNone(a.analyze(changed)['cycles'][0]['zero_bias_with_supplied_seating_bound_units'])
        value['linear_reading_range']['interval'] = [-1, 1]
        self.assertIsNone(a.analyze(value)['cycles'][0]['surface_tilt_with_supplied_seating_bound_units'])

    def test_zero_return_scatter_is_not_zero_uncertainty(self):
        value = fixture()
        for row, n in zip(value['cycles'][0]['readings'], (3, -1, 3)):
            row['displacement_interval'] = [n, n]
        result = a.analyze(value)
        self.assertEqual(result['cycles'][0]['observed_forward_return_difference_units'], [0, 0])
        self.assertIsNone(result['absolute_origin_uncertainty_rad'])
        self.assertFalse(result['uncertainty_inferred_from_repeatability'])

    def test_geometry_uses_horizontal_run_and_keeps_unknown_alignment(self):
        value = fixture(); value['geometry_references'] = [{'id': 'synthetic-slope',
            'rise_mm': {'interval': [9.9, 10.1], 'evidence': 'synthetic measured rise'},
            'horizontal_run_mm': {'interval': [99.9, 100.1], 'evidence': 'synthetic measured horizontal run'},
            'alignment_error_bound_rad': None}]
        result = a.analyze(value); geo = result['geometry_references'][0]
        self.assertLess(geo['angle_from_length_intervals_rad'][0], .1)
        self.assertGreater(geo['angle_from_length_intervals_rad'][1], .1)
        self.assertIsNone(geo['angle_with_supplied_alignment_bound_rad'])
        self.assertIsNone(result['cycles'][0]['surface_tilt_with_supplied_bounds_rad'])
        self.assertIsNone(result['absolute_origin_error_rad'])

    def test_geometry_positive_bound_expands_corners_without_automatic_fit(self):
        value = fixture(); value['geometry_references'] = [{'id': 'negative-slope',
            'rise_mm': {'interval': [-10.1, -9.9], 'evidence': 'synthetic rise'},
            'horizontal_run_mm': {'interval': [99.9, 100.1], 'evidence': 'synthetic run'},
            'alignment_error_bound_rad': {'upper': .01, 'evidence': 'synthetic alignment bound'}}]
        result = a.analyze(value); geo = result['geometry_references'][0]
        self.assertAlmostEqual(geo['angle_with_supplied_alignment_bound_rad'][0], geo['angle_from_length_intervals_rad'][0]-.01)
        self.assertAlmostEqual(geo['angle_with_supplied_alignment_bound_rad'][1], geo['angle_from_length_intervals_rad'][1]+.01)
        self.assertFalse(result['fit_observations_generated'])

    def test_unknown_fields_boolean_numbers_wrong_sign_or_order_are_rejected(self):
        changes = [lambda x: x.__setitem__('approved_for_runtime', True),
            lambda x: x.__setitem__('coordinate_convention', 'camera_left_positive'),
            lambda x: x['cycles'][0]['readings'][1].__setitem__('orientation', 'upside_down'),
            lambda x: x['cycles'][0]['readings'][0].__setitem__('displacement_interval', [False, 2]),
            lambda x: x['cycles'][0]['readings'][0].__setitem__('displacement_interval', [4, 2]),
            lambda x: x['cycles'][0]['conditions'].__setitem__('same_footprint', 1),
            lambda x: x['cycles'].append(copy.deepcopy(x['cycles'][0]))]
        for change in changes:
            with self.subTest(change=change):
                value = fixture(); change(value)
                with self.assertRaises(ValueError): a.analyze(value)

    def test_unknown_reference_cannot_be_zero_unbounded_or_source_free(self):
        changes = [('per_reading_seating_bound_units', {'upper': 0, 'evidence': 'unsupported exact seating'}),
            ('per_reading_seating_bound_units', {'upper': True, 'evidence': 'bad type'}),
            ('angular_sensitivity_rad_per_unit', {'interval': [.001, .001], 'evidence': 'unsupported exact scale'}),
            ('angular_sensitivity_rad_per_unit', {'interval': [-.001, .002], 'evidence': 'sign unknown'}),
            ('linear_reading_range', {'interval': [-1, 1], 'evidence': ''})]
        for key, v in changes:
            with self.subTest(key=key, v=v):
                value = fixture(); value[key] = v
                with self.assertRaises(ValueError): a.analyze(value)

    def test_strict_json_duplicate_nonfinite_and_overflow_refused(self):
        for raw in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{"x":1e400}'):
            with self.assertRaises(ValueError): a.strict_json(raw)

    def test_cli_file_only_binding_and_no_changes(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d).resolve()/'synthetic.json'; path.write_text(json.dumps(fixture()))
            before = path.read_bytes()
            with redirect_stdout(io.StringIO()) as out: self.assertEqual(a.main(['--input', str(path)]), 0)
            result = json.loads(out.getvalue()); self.assertEqual(result['input_binding']['byte_count'], len(before))
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(list(Path(d).resolve().iterdir()), [path])
            self.assertFalse(result['hardware_opened'] or result['approved_for_runtime'])
            link = Path(d).resolve()/'symlink.json'; link.symlink_to(path)
            with redirect_stdout(io.StringIO()) as out: self.assertEqual(a.main(['--input', str(link)]), 1)
            self.assertEqual(json.loads(out.getvalue())['status'], 'INVALID_LEVEL_RECORDS')


if __name__ == '__main__':
    unittest.main()
