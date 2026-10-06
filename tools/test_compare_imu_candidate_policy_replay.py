"""Synthetic file-only comparison tests; no device or production model loading."""
import ast
import copy
import hashlib
import inspect
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'runtime'/'tests'))
from test_policy_observer import FakeTorch, Policy, bias_candidate, calibration, mount, snapshot
from test_policy_observer_replay import records
import test_policy_shadow as capture_fixtures
import compare_imu_candidate_policy_replay as tool


def candidate(bias=(.2, -.3, -.6), scale=(1.02, .97, .98)):
    return {'bias_m_s2': list(bias), 'scale': list(scale)}


class ComparisonTests(unittest.TestCase):
    def compare(self, data=None, policies=None, **overrides):
        models, rows = policies or [Policy() for _ in range(4)], []
        options = dict(candidate=candidate(), imu_mount_candidate=mount(), max_ticks=3,
            max_age_ns=10_000_000, max_spread_ns=5_000_000, torch_module=FakeTorch,
            warmup_ticks=2, emit=rows.append)
        options.update(overrides)
        result = tool.compare_records(records() if data is None else data,
                                      calibration(), models, **options)
        return result, rows, models

    def test_four_states_original_snapshots_and_order_are_independent(self):
        data = records(); before = copy.deepcopy(data)
        result, rows, models = self.compare(data)
        self.assertEqual(data, before)
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_COMPLETE_UNAPPROVED')
        self.assertEqual([p.resets for p in models], [1]*4)
        self.assertEqual([len(p.calls) for p in models], [5]*4)
        for index, model in enumerate(models):
            self.assertEqual(model.calls[-1][-1], [float(index//2)]*12)
        for flag in tool.FLAGS: self.assertIs(result[flag], False)
        ticks = [r for r in rows if r['kind'] == 'offline_candidate_policy_tick']
        self.assertEqual(len(ticks), 12)
        self.assertEqual(ticks[0]['observation74'][33:45], [0.]*12)
        self.assertEqual(ticks[1]['observation74'][33:45], ticks[0]['actor_residual12'])
        self.assertEqual(ticks[0]['provenance']['snapshot_canonical_json_sha256'],
                         ticks[3]['provenance']['snapshot_canonical_json_sha256'])
        self.assertNotIn('inputs', ticks[3])
        self.assertEqual(ticks[3]['observer_validated_inputs'], ticks[0]['observer_validated_inputs'])
        self.assertNotEqual(ticks[3]['actual_model_inputs']['gravity_body_unit'],
                            ticks[0]['actual_model_inputs']['gravity_body_unit'])

    def test_only_gravity_tensor_object_replaced_and_context_consumed_once(self):
        class SeenPolicy(Policy):
            def __call__(self, *tensors):
                self.seen = tensors
                return super().__call__(*tensors)
        model = SeenPolicy()
        wrapper = tool.GravitySubstitutionPolicy(model, candidate(), mount()['R_body_from_sensor'], FakeTorch)
        tensors = tuple(FakeTorch.tensor([row]) for row in ([0.]*3, [0., 0., -1.], [0.]*3,
                       [0., .4, -.8]*4, [0.]*12, [1.]*12))
        wrapper(*tensors)  # Explicit synthetic warmup.
        self.assertTrue(all(a is b for a, b in zip(model.seen, tensors)))
        wrapper.reset(FakeTorch.tensor([0])); source = snapshot(); wrapper.arm(source)
        with self.assertRaisesRegex(ValueError, 'already armed'): wrapper.arm(source)
        wrapper(*tensors)
        for i in (0, 2, 3, 4, 5): self.assertIs(model.seen[i], tensors[i])
        self.assertIsNot(model.seen[1], tensors[1])
        self.assertEqual(wrapper.actual['substitution']['snapshot_canonical_json_sha256'], tool.digest(source))
        with self.assertRaisesRegex(ValueError, 'Unarmed'): wrapper(*tensors)

    def test_common_scalar_preserves_gravity_and_anisotropic_bias_changes_it(self):
        data = records()
        for row in data:
            if row['kind'] == 'imu': row['accel_m_s2'] = [2., 3., -9.]
        _, rows, _ = self.compare(data, candidate=candidate((0., 0., 0.), (1.03,)*3))
        tick_rows = [r for r in rows if r['kind'] == 'offline_candidate_policy_tick']
        for a, b in zip(tick_rows[0]['actual_model_inputs']['gravity_body_unit'],
                        tick_rows[3]['actual_model_inputs']['gravity_body_unit']):
            self.assertAlmostEqual(a, b, places=14)
        _, changed, _ = self.compare(data)
        correction = next(r for r in changed if r.get('condition') == 'corrected')
        self.assertGreater(correction['substitution']['gravity_angle_difference_deg_descriptive'], .1)

    def test_corrected_float32_input_and_target_difference_are_real_model_effects(self):
        import torch
        class SensitivePolicy(Policy):
            def __call__(self, *tensors):
                super().__call__(*tensors)
                gravity = tensors[1].tolist()[0]
                return torch.tensor([[gravity[0]*.1, .4, -.8]*4], dtype=torch.float32)
        result, rows, _ = self.compare(policies=[SensitivePolicy() for _ in range(4)], torch_module=torch)
        correction = next(r for r in rows if r.get('condition') == 'corrected')
        double = correction['substitution']['gravity_before_float32']
        actual = correction['actual_model_inputs']['gravity_body_unit']
        self.assertEqual(actual, torch.tensor([double], dtype=torch.float32).tolist()[0])
        self.assertNotEqual(actual, double)
        self.assertGreater(result['paired_comparisons'][0]['target_delta_max_absolute_rad_by_axis'][0], .001)

    def test_raw_wrapper_preserves_all_tensors_and_records_float32(self):
        import torch
        class Seen(Policy):
            def __call__(self, *tensors):
                self.seen = tensors
                return super().__call__(*tensors)
        model = Seen(); wrapper = tool.GravitySubstitutionPolicy(model, None, mount()['R_body_from_sensor'], torch)
        wrapper.reset(torch.tensor([0])); wrapper.arm(snapshot())
        inputs = tuple(torch.tensor([row], dtype=torch.float32) for row in
                       ([.123456789]*3, [0., 0., -1.], [0.]*3, [0., .4, -.8]*4, [0.]*12, [0.]*12))
        wrapper(*inputs)
        self.assertTrue(all(a is b for a, b in zip(model.seen, inputs)))
        self.assertIsNone(wrapper.actual['substitution'])
        self.assertEqual(wrapper.actual['actual_model_inputs']['gyro_body_rad_s'], inputs[0].tolist()[0])

    def test_raw_out_of_range_blocks_before_every_real_model_call(self):
        data = records()
        next(r for r in data if r.get('parameter') == 'position')['value'] = 20.
        result, rows, models = self.compare(data)
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_BLOCKED')
        self.assertEqual([len(p.calls) for p in models], [2]*4)
        self.assertTrue(all(r['ticks_completed'] == 0 for r in result['hypotheses']))
        self.assertEqual([r['common_successful_ticks'] for r in result['paired_comparisons']], [0, 0])
        self.assertTrue(all('outside' in r['first_blocked_tick']['reason'] for r in result['hypotheses']))

    def test_double_correction_or_review_extension_rejected(self):
        for flag in tool.observer.RAW_IMU_CORRECTION_FLAGS:
            data = records(); next(r for r in data if r['kind'] == 'imu')[flag] = True
            with self.subTest(flag=flag), self.assertRaises(ValueError): self.compare(data)
        with self.assertRaisesRegex(ValueError, 'Reviewed acceleration'):
            self.compare(gyro_bias_candidate={'accel_calibration_review': None})

    def test_context_cleared_after_observer_or_model_failure(self):
        model = Policy(); wrapper = tool.GravitySubstitutionPolicy(model, candidate(), mount()['R_body_from_sensor'], FakeTorch)
        run = tool.observer.StatefulPolicyObserver(wrapper, calibration(), imu_mount_candidate=mount(),
            h_hypothesis=0, command=[0., 0., 0.], max_ticks=1, max_age_ns=10_000_000,
            max_spread_ns=5_000_000, torch_module=FakeTorch)
        run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot(); source['status'] = 'BLOCKED'
        with self.assertRaises(ValueError): tool.consume_corrected(run, wrapper, source)
        self.assertIsNone(wrapper._pending)
        self.assertEqual(model.calls, [])

    def test_valid_gyro_hypothesis_is_shared_and_not_reestimated(self):
        result, rows, _ = self.compare(gyro_bias_candidate=bias_candidate())
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_COMPLETE_UNAPPROVED')
        ticks = [r for r in rows if r['kind'] == 'offline_candidate_policy_tick']
        for row in ticks:
            for actual, expected in zip(row['actual_model_inputs']['gyro_body_rad_s'], [.2, .1, -.3]):
                self.assertAlmostEqual(actual, expected)
            self.assertFalse(row['provenance']['accel_bias_subtracted'])

    def test_arm_failure_or_wrong_snapshot_receipt_invalidates_and_clears(self):
        from unittest.mock import Mock
        wrapper = tool.GravitySubstitutionPolicy(Policy(), candidate(), mount()['R_body_from_sensor'], FakeTorch)
        wrapper.reset(FakeTorch.tensor([0])); run = Mock()
        source = snapshot(); source['imu']['accel_m_s2'][0] = math.nan
        with self.assertRaises(ValueError): tool.consume_corrected(run, wrapper, source)
        self.assertIsNone(wrapper._pending); run.invalidate.assert_called_once()
        source = snapshot()
        def wrong_receipt(s):
            inputs = tuple(FakeTorch.tensor([row]) for row in
                ([0.]*3, [0., 0., -1.], [0.]*3, [0., .4, -.8]*4, [0.]*12, [0.]*12))
            wrapper(*inputs)
            return {'tick_ns': s['tick_ns'], 'provenance': {'snapshot_canonical_json_sha256': 'a'*64}}
        run = Mock(); run.consume.side_effect = wrong_receipt
        with self.assertRaisesRegex(ValueError, 'snapshot binding'):
            tool.consume_corrected(run, wrapper, source)
        self.assertIsNone(wrapper._pending); run.invalidate.assert_called_once()

    def test_partial_failure_keeps_all_conditions_and_common_scope(self):
        class LaterFailure(Policy):
            def __call__(self, *tensors):
                if len(self.calls) == 3: self.bad = 'target_bounds'
                return super().__call__(*tensors)
        result, rows, _ = self.compare(policies=[Policy(), LaterFailure(), Policy(), Policy()])
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_BLOCKED')
        self.assertEqual([r['ticks_completed'] for r in result['hypotheses']], [3, 1, 3, 3])
        self.assertEqual(result['paired_comparisons'][0]['common_successful_ticks'], 1)
        self.assertEqual(len([r for r in rows if r['kind'] == 'offline_candidate_policy_blocked']), 1)

    def test_nonfinite_wrong_shape_and_exhausted_capture_remain_blocked(self):
        for bad in ('actor_nan', 'observation_batch', 'target_shape', 'target_bounds'):
            result, _, _ = self.compare(policies=[Policy(), Policy(bad=bad), Policy(), Policy()])
            self.assertEqual(result['status'], 'OFFLINE_COMPARISON_BLOCKED')
        result, _, _ = self.compare(max_ticks=6)
        self.assertTrue(all(r['first_blocked_tick']['reason'] == 'capture_exhausted' for r in result['hypotheses']))

    def test_future_or_stale_sources_not_zero_filled(self):
        data = records()
        next(r for r in data if r['kind'] == 'imu')['read_finished_monotonic_ns'] += 500_000
        result, rows, _ = self.compare(data)
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_COMPLETE_UNAPPROVED')
        data = [r for r in records() if r['kind'] != 'imu' or r['monotonic_ns'] < 1_000_000_000]
        result, rows, _ = self.compare(data)
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_BLOCKED')
        self.assertTrue(all(r['ticks_completed'] == 1 for r in result['hypotheses']))

    def test_shared_models_and_nonpositive_or_zero_norm_candidate_rejected(self):
        shared = Policy()
        with self.assertRaisesRegex(ValueError, 'independent'): self.compare(policies=[shared]*4)
        with self.assertRaisesRegex(ValueError, 'positive'): self.compare(candidate=candidate(scale=(1., 0., 1.)))
        result, _, _ = self.compare(candidate=candidate(bias=(0., 0., -10.)))
        self.assertEqual(result['status'], 'OFFLINE_COMPARISON_BLOCKED')
        self.assertEqual(result['paired_comparisons'][0]['common_successful_ticks'], 0)


class FileTests(unittest.TestCase):
    def prepare(self, directory):
        root = Path(directory).resolve(); capture = root/'capture'; capture.mkdir()
        with patch.object(capture_fixtures, 'records', return_value=records()):
            capture_fixtures.capture_fixture(capture)
        cal, imu, norm_input, norm_report = [root/name for name in ('cal.json', 'mount.json', 'norm-input.json', 'norm-report.json')]
        cal.write_text(json.dumps(calibration())); imu.write_text(json.dumps(mount())); norm_input.write_text('{}')
        norm = {'schema': tool.norm_fit.SCHEMA, 'status': 'UNAPPROVED_NORM_ELLIPSE_DIAGNOSTIC',
            'approved_for_runtime': False, 'automatically_applied': False, 'heldout_used_for_fit': False,
            'independent_captures_used_for_fit': False, 'accel_diagnostic_candidate': candidate(),
            'input_bindings': [tool.read_file(norm_input)[1]], 'source_bindings': []}
        norm_report.write_text(json.dumps(norm))
        bundle = root/'bundle'; bundle.mkdir(); hashes = {}
        for name in tool.shadow.SOURCE_HASHES:
            raw = ('synthetic test '+name).encode(); (bundle/name).write_bytes(raw)
            hashes[name] = hashlib.sha256(raw).hexdigest()
        args = ['--capture', str(capture), '--calibration', str(cal), '--bundle', str(bundle),
            '--imu-mount-candidate', str(imu), '--accel-diagnostic-input', str(norm_input),
            '--accel-diagnostic-report', str(norm_report), '--output', str(root/'out'),
            '--max-ticks', '3', '--max-age-ms', '10', '--max-spread-ms', '5', '--warmup-ticks', '2']
        return root, args, norm, hashes

    def invoke(self, args, norm, hashes, loader=None):
        def default_loader(bundle):
            return Policy(), {'bundle': str(bundle), 'sha256': hashes.copy(), 'source': 'synthetic test'}
        with patch.object(tool.norm_fit, 'analyze_file', return_value=copy.deepcopy(norm)), \
             patch.dict(tool.shadow.SOURCE_HASHES, hashes, clear=True), \
             patch.object(tool.shadow, 'load_policy', side_effect=loader or default_loader), \
             patch.dict(sys.modules, {'torch': FakeTorch}), patch('sys.stdout', new=io.StringIO()), \
             patch('sys.stderr', new=io.StringIO()):
            return tool.main(args)

    def test_cli_complete_wire_capture_private_manifest_and_unchanged_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            before = (root/'capture'/'events.jsonl').read_bytes()
            self.assertEqual(self.invoke(args, norm, hashes), 0)
            summary = json.loads((root/'out'/'summary.json').read_text())
            self.assertEqual(summary['status'], 'OFFLINE_COMPARISON_COMPLETE_UNAPPROVED')
            self.assertEqual(len(summary['provenance']['models']), 4)
            self.assertEqual(before, (root/'capture'/'events.jsonl').read_bytes())
            self.assertEqual((root/'out').stat().st_mode & 0o777, 0o700)
            for path in (root/'out').iterdir(): self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertTrue(summary['provenance']['all_input_source_files_unchanged'])
            self.assertFalse(summary['live_50hz_verified'])

    def test_norm_report_changes_and_json_type_changes_rejected_before_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            changed = copy.deepcopy(norm); changed['approved_for_runtime'] = 0
            (root/'norm-report.json').write_text(json.dumps(changed))
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            report = json.loads((root/'out'/'summary.json').read_text())
            self.assertEqual(report['failure_phase'], 'norm_candidate_recomputation')
            self.assertFalse((root/'out'/'events.jsonl').exists())

    def test_norm_report_recomputed_exactly_excluding_creation_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, _, norm, _ = self.prepare(tmp)
            report = copy.deepcopy(norm); report['generated_at_utc'] = 'synthetic metadata'
            (root/'norm-report.json').write_text(json.dumps(report))
            with patch.object(tool.norm_fit, 'analyze_file', return_value=norm) as recompute:
                actual, pins = tool.audit_norm_candidate(root/'norm-input.json', root/'norm-report.json')
            recompute.assert_called_once_with(root/'norm-input.json')
            self.assertEqual(actual, candidate()); self.assertEqual(len(pins), 2)

    def test_input_mutation_during_models_rejected_and_failure_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            def mutate(bundle):
                (root/'cal.json').write_text(json.dumps(calibration())+' ')
                return Policy(), {'sha256': hashes.copy()}
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes, mutate)
            report = json.loads((root/'out'/'summary.json').read_text())
            self.assertEqual(report['failure_phase'], 'model_loading')
            self.assertFalse(report['input_binding_verification_completed'])

    def test_mutation_during_replay_preserves_partial_events_but_no_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            class Mutate(Policy):
                def __call__(self, *tensors):
                    if self.resets: (root/'norm-input.json').write_text('{ }')
                    return super().__call__(*tensors)
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes,
                lambda bundle: (Mutate(), {'sha256': hashes.copy()}))
            report = json.loads((root/'out'/'summary.json').read_text())
            self.assertEqual(report['failure_phase'], 'final_file_binding_verification')
            self.assertTrue(report['partial_events_preserved'])
            self.assertGreater((root/'out'/'events.jsonl').stat().st_size, 0)

    def test_bundle_mismatch_and_incomplete_capture_have_no_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            (root/'bundle'/'model_149.pt').write_bytes(b'tampered')
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            self.assertFalse((root/'out'/'events.jsonl').exists())
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            summary = json.loads((root/'capture'/'summary.json').read_text()); summary['status'] = 'INCOMPLETE'
            (root/'capture'/'summary.json').write_text(json.dumps(summary))
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            self.assertEqual(json.loads((root/'out'/'summary.json').read_text())['failure_phase'], 'capture_wire_validation')

    def test_source_pin_and_regular_file_checks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); original = root/'data'; original.write_text('original')
            pin = tool.read_file(original)[1]; original.write_text('changed')
            with self.assertRaisesRegex(ValueError, 'Pinned file'): tool.verify_bindings([pin])
            link = root/'link'; link.symlink_to(original)
            with self.assertRaisesRegex(ValueError, 'symlink'): tool.read_file(link)
            with self.assertRaises((ValueError, OSError)): tool.read_file(root)
        names = {Path(pin['path']).name for pin in tool.source_bindings()}
        self.assertTrue({'imu_calibration_review.py', 'policy_observer.py', 'event_snapshot.py',
                         'analyze_imu_pose_tilt.py'} <= names)

    def test_output_overwrite_symlink_or_git_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp)
            out = root/'out'; out.mkdir(); (out/'keep').write_text('unchanged')
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            self.assertEqual(list(out.iterdir()), [out/'keep'])
            out.rename(root/'existing'); out.symlink_to(root/'existing', target_is_directory=True)
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            self.assertFalse((root/'existing'/'summary.json').exists())
        with tempfile.TemporaryDirectory() as tmp:
            root, args, norm, hashes = self.prepare(tmp); (root/'.git').mkdir()
            with self.assertRaises(SystemExit): self.invoke(args, norm, hashes)
            self.assertFalse((root/'out').exists())

    def test_new_tool_has_no_transport_or_device_imports(self):
        tree = ast.parse(inspect.getsource(tool))
        names = {alias.name.split('.')[0] for node in ast.walk(tree) if isinstance(node, ast.Import)
                 for alias in node.names}
        self.assertFalse(names & {'serial', 'socket', 'subprocess', 'smbus', 'smbus2', 'gi'})


if __name__ == '__main__':
    unittest.main()
