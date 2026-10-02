"""Synthetic saved targets only; no private robot observations or model execution."""
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools import analyze_policy_target_mixture as tool


class TargetMixtureTests(unittest.TestCase):
    def setUp(self):
        self.report = dict(trial_origin_model_rad_by_id={str(i): 0.0 for i in range(1, 13)},
            actual_model_calls=3,
            cycles=[dict(index=i, feedback=dict(q_model_rad=[0.0] * 12)) for i in range(3)])
        self.report_sha = 'a' * 64
        self.targets = dict(schema=tool.TARGET_SCHEMA, id_order=list(range(1, 13)),
            report_sha256=self.report_sha, output_allowed=False,
            rows=[dict(cycle_index=i, raw_target_model_rad=[0.0] * 12) for i in range(3)])

    def analyze(self, targets=None, **kwargs):
        return tool.analyze(self.report, self.targets if targets is None else targets,
                            report_sha256=self.report_sha, targets_sha256='b' * 64,
                            physical_clearance_deg=kwargs.get('physical', 3),
                            max_displacement_deg=kwargs.get('displacement', 1))

    def test_full_sequence_first_sign_peak_and_last_endpoint_differ(self):
        # First, worst excursion, and endpoint are deliberately different.
        for row, value in zip(self.targets['rows'], (-2, 12, 4)):
            row['raw_target_model_rad'][0] = math.radians(value)
        result = self.analyze()
        self.assertEqual(result['sequence_extent'], 'logged_model_call_count_matches')
        low = result['mixtures'][0]
        axis = low['per_axis'][0]
        self.assertAlmostEqual(axis['first_delta_deg'], -.2)
        self.assertEqual(axis['first_direction'], 'negative')
        self.assertAlmostEqual(axis['peak_absolute_delta_deg'], 1.2)
        self.assertEqual(axis['peak_cycle_index'], 1)
        self.assertAlmostEqual(axis['final_delta_deg'], .4)
        self.assertAlmostEqual(axis['mixed_target_endpoint_model_rad'], math.radians(.4))
        self.assertEqual(low['physical_clearance_exceeded_ids'], [])
        self.assertEqual(low['maximum_displacement_exceeded_ids'], [1])
        self.assertEqual(low['recommendation'], 'CURRENT_MAXIMUM_DISPLACEMENT_EXCEEDED_NEW_REVIEW_REQUIRED')
        high = result['mixtures'][-1]
        self.assertEqual(high['physical_clearance_exceeded_ids'], [1])
        self.assertFalse(result['closed_loop_prediction'])
        self.assertFalse(result['approvals_created'])
        self.assertFalse(result['output_allowed'])

    def test_origin_not_each_cycle_feedback_and_angles_never_wrap_or_clip(self):
        self.report['trial_origin_model_rad_by_id']['1'] = math.radians(20)
        self.report['cycles'][1]['feedback']['q_model_rad'][0] = math.radians(30)
        for row in self.targets['rows']:
            row['raw_target_model_rad'][0] = math.radians(380)
        result = self.analyze()
        self.assertAlmostEqual(result['mixtures'][0]['per_axis'][0]['first_delta_deg'], 36)
        axis = result['mixtures'][-1]['per_axis'][0]
        self.assertAlmostEqual(axis['peak_absolute_delta_deg'], 360)
        self.assertAlmostEqual(axis['mixed_target_endpoint_model_rad'], math.radians(380))
        self.assertEqual(self.report['cycles'][1]['feedback']['q_model_rad'][0], math.radians(30))

    def test_single_snapshot_fallback_initial_and_inputs_unchanged(self):
        del self.report['trial_origin_model_rad_by_id']
        self.report['cycles'][0]['feedback']['q_model_rad'][2] = .5
        self.targets['rows'] = self.targets['rows'][:1]
        self.targets['rows'][0]['raw_target_model_rad'][2] = .6
        original = copy.deepcopy((self.report, self.targets))
        result = self.analyze()
        self.assertEqual(result['sequence_extent'], 'single_target_snapshot')
        self.assertEqual(result['initial_pose_source'], 'first_saved_cycle_feedback')
        self.assertAlmostEqual(result['mixtures'][0]['per_axis'][2]['mixed_target_endpoint_model_rad'], .51)
        self.assertEqual((self.report, self.targets), original)

    def test_negative_peak_and_supplied_per_axis_caps(self):
        for row, value in zip(self.targets['rows'], (-2, -4, 1)):
            row['raw_target_model_rad'][3] = math.radians(value)
        physical = [3.0] * 12
        physical[3] = .3
        result = self.analyze(physical=physical)
        axis = result['mixtures'][0]['per_axis'][3]
        self.assertAlmostEqual(axis['peak_signed_delta_deg'], -.4)
        self.assertAlmostEqual(axis['maximum_negative_delta_deg'], -.4)
        self.assertAlmostEqual(axis['maximum_positive_delta_deg'], .1)
        self.assertEqual(result['mixtures'][0]['physical_clearance_exceeded_ids'], [4])

    def test_boundaries_and_partial_sequence_are_not_approvals(self):
        self.targets['rows'] = self.targets['rows'][:2]
        self.targets['rows'][0]['raw_target_model_rad'][0] = math.radians(1)
        result = self.analyze(physical=1, displacement=1)
        self.assertEqual(result['sequence_extent'], 'partial_saved_sequence')
        last = result['mixtures'][-1]
        self.assertEqual(last['maximum_displacement_exceeded_ids'], [])
        self.assertEqual(last['physical_clearance_exceeded_ids'], [])
        self.assertEqual(last['recommendation'], 'NUMERIC_CAPS_ONLY_MATCH_NO_EXECUTION_APPROVAL')
        self.assertFalse(last['output_allowed'])

    def test_report_sha_id_order_and_replay_feedback_bindings(self):
        for kind in ('report', 'order', 'bool_order', 'initial', 'feedback', 'approval'):
            with self.subTest(kind=kind):
                targets = copy.deepcopy(self.targets)
                if kind == 'report':
                    targets['report_sha256'] = 'c' * 64
                elif kind == 'order':
                    targets['id_order'].reverse()
                elif kind == 'bool_order':
                    targets['id_order'][0] = True
                elif kind == 'initial':
                    targets['initial_q_model_rad_by_id'] = [1.0] * 12
                elif kind == 'feedback':
                    targets['rows'][0]['feedback_q_model_rad_by_id'] = [1.0] * 12
                else:
                    targets['output_allowed'] = True
                with self.assertRaises(ValueError):
                    self.analyze(targets)

    def test_duplicate_reordered_unknown_and_boolean_cycles_rejected(self):
        for indices in ((0, 0), (1, 0), (0, 100), (False, 1)):
            targets = copy.deepcopy(self.targets)
            targets['rows'] = [dict(cycle_index=i, raw_target_model_rad=[0.0] * 12) for i in indices]
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                self.analyze(targets)
        self.report['cycles'][1]['index'] = 0
        with self.assertRaises(ValueError):
            self.analyze()

    def test_nonfinite_bool_bad_shape_origin_and_caps_rejected(self):
        for value in (float('nan'), float('inf'), True, '1', 10 ** 500):
            targets = copy.deepcopy(self.targets)
            targets['rows'][0]['raw_target_model_rad'][0] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.analyze(targets)
        for cap in (0, -1, float('nan'), True, [1] * 11):
            with self.subTest(cap=cap), self.assertRaises(ValueError):
                self.analyze(physical=cap)
        targets = copy.deepcopy(self.targets)
        targets['rows'][0]['raw_target_model_rad'].pop()
        with self.assertRaises(ValueError):
            self.analyze(targets)
        del self.report['trial_origin_model_rad_by_id']['12']
        with self.assertRaises(ValueError):
            self.analyze()

    def test_nonfinite_arithmetic_result_is_not_clamped(self):
        self.report['trial_origin_model_rad_by_id']['1'] = -1e308
        self.targets['rows'][0]['raw_target_model_rad'][0] = 1e308
        with self.assertRaises(ValueError):
            self.analyze()

    def test_file_sha_symlink_duplicate_and_cli_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            report_path, targets_path = root / 'report.json', root / 'targets.json'
            report_raw = json.dumps(self.report).encode()
            self.targets['report_sha256'] = hashlib.sha256(report_raw).hexdigest()
            report_path.write_bytes(report_raw)
            targets_path.write_text(json.dumps(self.targets))
            before = {p.name: p.read_bytes() for p in root.iterdir()}
            with mock.patch('sys.stdout', new_callable=io.StringIO) as output:
                code = tool.main(['--report', str(report_path), '--targets', str(targets_path),
                                  '--physical-clearance-deg', '3', '--max-displacement-deg', '1'])
            self.assertEqual(code, 0)
            result = json.loads(output.getvalue())
            self.assertEqual(result['input_sha256']['report'], self.targets['report_sha256'])
            self.assertFalse(result['model_executed'])
            self.assertEqual(before, {p.name: p.read_bytes() for p in root.iterdir()})
            with self.assertRaisesRegex(ValueError, 'SHA256 differs'):
                tool.read_json(report_path, '0' * 64)
            linked = root / 'linked.json'
            linked.symlink_to(report_path)
            with self.assertRaises(ValueError):
                tool.read_json(linked)
            linked.unlink()
            targets_path.write_text('{"rows":[],"rows":[]}')
            with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
                tool.read_json(targets_path)


if __name__ == '__main__':
    unittest.main()
