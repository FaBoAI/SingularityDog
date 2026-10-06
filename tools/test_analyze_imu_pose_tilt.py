"""Tilted synthetic poses, partition separation and pinned file-only captures."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import analyze_imu_pose_tilt as tool

G = tool.calibration.GRAVITY
BIAS = [.12, -.18, -.61]
SCALE = [1.02, .98, .975]
DIRECTIONS = [[1, .2, .1], [-1, .2, -.1], [.1, 1, .2], [-.15, -1, .3],
              [.1, -.12, 1], [-.2, -.05, -1]]


def means():
    return {label: [b+G*x/math.hypot(*v)/s for b, s, x in zip(BIAS, SCALE, v)]
            for label, v in zip(tool.calibration.FACES, DIRECTIONS)}


def datasets():
    result = {'fit': {}, 'independent': {}}
    for partition, offset in (('fit', 0), ('independent', .002)):
        for j, (label, mean) in enumerate(means().items()):
            result[partition][label] = [{'frame': 'sensor', 'monotonic_ns':
                1_000_000_000+(j+(0 if partition == 'fit' else 6))*5_000_000_000+i*10_000_000,
                'accel_m_s2': [x+offset+(.001 if i % 2 else -.001) for x in mean],
                'gyro_rad_s': [.01, .02, .003]} for i in range(400)]
    return result


class AlgebraTests(unittest.TestCase):
    def test_non_antipodal_tilted_poses_recover_bias_scale_without_aligning_axes(self):
        result = tool.fit_norm_ellipse(means())
        for actual, expected in zip(result['bias_m_s2'], BIAS): self.assertAlmostEqual(actual, expected, places=11)
        for actual, expected in zip(result['scale'], SCALE): self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(result['fit_residual_degrees_of_freedom'], 0)
        self.assertLess(result['maximum_mean_equation_residual'], 1e-13)
        self.assertFalse(result['numerical_condition_is_physical_uncertainty_bound'])

    def test_positive_ellipse_fit_is_ordered_by_labels_not_dictionary_order(self):
        selected = means()
        self.assertEqual(tool.fit_norm_ellipse(selected), tool.fit_norm_ellipse(dict(reversed(list(selected.items())))))

    def test_negative_quadratic_coefficient_is_rejected(self):
        directions = [[.4, math.sqrt(1.16), 0], [.6, 0, math.sqrt(1.36)], [.2, .7, math.sqrt(.55)]]
        values = [v for direction in directions for v in (direction, [-x for x in direction])]
        with self.assertRaisesRegex(ValueError, 'Positive finite ellipse'):
            tool.fit_norm_ellipse({label: [G*x for x in v] for label, v in zip(tool.calibration.FACES, values)})

    def test_rank_deficient_same_direction_faces_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Rank-deficient'):
            tool.fit_norm_ellipse({label: [G, G*.01, G*.02] for label in tool.calibration.FACES})

    def test_column_scaling_does_not_hide_weak_orientation_coverage(self):
        e = 1e-6
        values = [[1, 0, 0], [-1, 0, 0], [1, e, 0], [-1, -e, 0], [1, 0, e], [-1, 0, -e]]
        with self.assertRaisesRegex(ValueError, 'Ill-conditioned'):
            tool.fit_norm_ellipse({label: [G*x for x in v] for label, v in zip(tool.calibration.FACES, values)})

    def test_boolean_nonfinite_oversize_and_incomplete_means_rejected(self):
        for value in (True, math.nan, math.inf, 5*G):
            selected = means(); selected['x+'][0] = value
            with self.subTest(value=value), self.assertRaises(ValueError): tool.fit_norm_ellipse(selected)
        selected = means(); del selected['z-']
        with self.assertRaises(ValueError): tool.fit_norm_ellipse(selected)

    def test_holdout_changes_never_refit_candidate(self):
        selected = datasets(); result = tool.analyze_datasets(**selected)
        changed = copy.deepcopy(selected)
        for records in changed['fit'].values():
            for row in records[300:]: row['accel_m_s2'][0] += .04
        for records in changed['independent'].values():
            for row in records: row['accel_m_s2'][1] += .03
        after = tool.analyze_datasets(**changed)
        self.assertEqual(result['accel_diagnostic_candidate'], after['accel_diagnostic_candidate'])
        self.assertNotEqual(result['evaluation']['independent_captures'], after['evaluation']['independent_captures'])
        self.assertFalse(after['heldout_used_for_fit'] or after['independent_captures_used_for_fit'])

    def test_pair_means_are_descriptors_and_tilt_is_not_replaced_by_zero(self):
        result = tool.analyze_datasets(**datasets())
        self.assertGreater(result['pair_descriptors']['x']['corrected_pair_midpoint_norm_m_s2'], .1)
        self.assertFalse(result['pair_descriptors']['x']['pair_midpoint_is_common_sensor_bias'])
        self.assertGreater(result['evaluation']['fit_training']['x+']['corrected_mean_angle_from_labeled_axis_deg_descriptive'], 10)
        self.assertIsNone(result['physical_pose_uncertainty_rad'])
        for key in ('approved_for_runtime', 'automatically_applied', 'hardware_opened', 'motor_output_allowed', 'profile_changed', 'thresholds_changed'):
            self.assertIs(result[key], False)
        self.assertNotIn('schema_version', result)

    def test_retimed_duplicate_independent_measurement_is_rejected(self):
        selected = datasets()
        for a, b in zip(selected['fit']['x+'], selected['independent']['x+']):
            b['accel_m_s2'] = a['accel_m_s2'][:]
        with self.assertRaisesRegex(ValueError, 'Reused'): tool.analyze_datasets(**selected)

    def test_overlap_and_corrected_input_are_rejected(self):
        selected = datasets()
        for a, b in zip(selected['fit']['x+'], selected['independent']['x+']):
            b['monotonic_ns'] = a['monotonic_ns']
        with self.assertRaisesRegex(ValueError, 'overlapping'): tool.analyze_datasets(**selected)
        selected = datasets(); selected['fit']['x+'][0]['calibration_applied'] = True
        with self.assertRaisesRegex(ValueError, 'explicitly false'): tool.analyze_datasets(**selected)


class PinnedCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(); cls.root = Path(cls.temp.name).resolve()
        fixture_path = Path(__file__).resolve().parents[1]/'runtime/tests/test_imu_fixed_mount_baseline.py'
        spec = importlib.util.spec_from_file_location('_pose_tilt_synthetic_capture_fixtures', fixture_path)
        fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
        cls.manifest = {'schema': tool.INPUT_SCHEMA, 'operator_confirmed_stationary': True, 'fit': {}, 'independent': {}}
        for partition_index, partition in enumerate(('fit', 'independent')):
            for j, (label, mean) in enumerate(means().items()):
                meta, rows = fixture.synthetic_capture(20+j+partition_index*10, 100+20*(j+6*partition_index))
                meta['plan']['face_label'] = label
                scale = meta['configuration']['accel_m_s2_per_lsb']
                for i, row in enumerate(rows):
                    row['raw_accel'] = [round(x/scale)+(1 if i % 2 else -1) for x in mean]
                    row['accel_m_s2'] = [x*scale for x in row['raw_accel']]
                fixture.refresh_summary(meta, rows)
                directory = cls.root/(partition+'-'+label); directory.mkdir()
                (directory/'summary.json').write_text(json.dumps(meta)+'\n')
                (directory/'events.jsonl').write_text('\n'.join(json.dumps(r) for r in [{'kind':'capture_metadata', **meta['plan']}]+rows)+'\n')
                cls.manifest[partition][label] = {'directory': str(directory),
                    'summary_sha256': hashlib.sha256((directory/'summary.json').read_bytes()).hexdigest(),
                    'events_sha256': hashlib.sha256((directory/'events.jsonl').read_bytes()).hexdigest()}
        cls.path = cls.root/'input.json'
        cls.path.write_text(json.dumps(cls.manifest)+'\n')

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def test_full_baseline_raw_audit_pinned_inputs_and_private_no_overwrite_cli(self):
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        output = self.root/'candidate.json'
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(['--input', str(self.path), '--output', str(output)]), 0)
            result = json.loads(output.read_text())
            self.assertTrue(result['capture_audit_verified'])
            self.assertEqual(len(result['input_bindings']), 25)
            self.assertFalse(result['approved_for_runtime'])
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)
            saved = output.read_bytes()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tool.main(['--input', str(self.path), '--output', str(output)])
            self.assertEqual(output.read_bytes(), saved)
            self.assertEqual(before, {p: p.read_bytes() for p in before})
        finally: output.unlink(missing_ok=True)

    def test_stored_hash_mismatch_and_unconfirmed_operator_rejected(self):
        for mutate in (lambda x:x['fit']['x+'].__setitem__('events_sha256','0'*64),
                       lambda x:x.__setitem__('operator_confirmed_stationary',False)):
            data = copy.deepcopy(self.manifest); mutate(data)
            with tempfile.TemporaryDirectory() as name:
                path = Path(name).resolve()/'input.json'; path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): tool.analyze_file(path)

    def test_input_mutation_during_computation_is_rejected_before_publication(self):
        path = Path(self.manifest['fit']['x+']['directory'])/'events.jsonl'; before = path.read_bytes()
        original = tool.analyze_datasets
        def changed(*args):
            result = original(*args); path.write_bytes(before+b'\n'); return result
        try:
            with patch.object(tool, 'analyze_datasets', side_effect=changed), self.assertRaisesRegex(ValueError, 'changed during analysis'):
                tool.analyze_file(self.path)
        finally: path.write_bytes(before)

    def test_symlink_input_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name).resolve()/'input.json'; path.symlink_to(self.path)
            with self.assertRaisesRegex(ValueError, 'symlink'): tool.analyze_file(path)


if __name__ == '__main__': unittest.main()
