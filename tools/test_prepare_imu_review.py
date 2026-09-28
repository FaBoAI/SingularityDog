"""Offline IMU review preparation cannot manufacture physical approval."""
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'runtime'), str(ROOT/'runtime/tests')]
import prepare_imu_review as prepare
from singularitydog_hw import imu_commissioning_audit as audit
from test_imu_fixed_mount_baseline import synthetic_capture, refresh_summary


class PrepareImuReviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.rows = {}
        for label, seed, start in (('a', 11, 100), ('b', 22, 140)):
            meta, rows = synthetic_capture(seed, start)
            self.rows[label] = (meta, rows)
            self.write_capture(label)
        self.manifest = {'schema_version': 1,
                         'stationary': {'a': 'a', 'b': 'b', 'operator_confirmed': True},
                         'movements': []}
        self.save_inputs()

    def write_capture(self, label):
        meta, rows = self.rows[label]
        path = self.root/label
        path.mkdir(exist_ok=True)
        (path/'summary.json').write_text(json.dumps(meta))
        (path/'events.jsonl').write_text('\n'.join(json.dumps(v) for v in
            [{'kind': 'capture_metadata', **meta['plan']}, *rows])+'\n')

    def save_inputs(self):
        (self.root/'manifest.json').write_text(json.dumps(self.manifest))
        (self.root/'audit.json').unlink(missing_ok=True)
        result = audit.write_audit(self.root/'manifest.json', self.root/'audit.json')
        (self.root/'mount.json').write_text(json.dumps(result['mount_candidate']))
        (self.root/'bias.json').write_text(json.dumps(result['stationary']))
        return result

    def run_prepare(self, name='review.json'):
        return prepare.prepare(*(self.root/(k+'.json') for k in ('manifest', 'audit', 'mount', 'bias')), self.root/name)

    def mutate_json(self, name, mutation):
        path = self.root/(name+'.json')
        data = json.loads(path.read_text())
        mutation(data)
        path.write_text(json.dumps(data))

    def test_recomputed_metrics_all_sources_pinned_and_no_approval(self):
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        result = self.run_prepare()
        self.assertEqual(result['status'], 'UNREVIEWED')
        for field in ('approved_for_runtime', 'dependency_eligible', 'hardware_opened', 'automatically_applied'):
            self.assertIs(result[field], False)
        for field in prepare.PHYSICAL_KEYS:
            self.assertIsNone(result['imu'][field])
        self.assertIsNone(result['imu']['gravity_direction_max_error_rad'])
        self.assertEqual(result['imu']['norm_deviation_rationale'], '')
        self.assertIsNone(result['external_gravity_reference']['synchronization']['start_monotonic_ns'])
        bias = json.loads((self.root/'bias.json').read_text())['gyro_bias_candidate_rad_s']
        expected = max(math.hypot(*(v-b for v, b in zip(r['gyro_rad_s'], bias))) for r in self.rows['b'][1])
        self.assertAlmostEqual(result['imu']['corrected_static_gyro_max_rad_s'], expected)
        norms = [math.hypot(*r['accel_m_s2']) for _, rows in self.rows.values() for r in rows]
        self.assertEqual(result['imu']['raw_gravity_norm_min_m_s2'], min(norms))
        self.assertEqual(result['imu']['raw_gravity_norm_max_m_s2'], max(norms))
        self.assertGreater(min(norms), 10.5)  # Keep the original +9% anomaly.
        self.assertEqual(len(result['references']), 8)
        self.assertEqual(len(result['source_captures']), 6)
        from singularitydog_hw.policy_live_profile import _artifact
        for ref in result['source_captures']:
            self.assertEqual(ref['sha256'], hashlib.sha256(Path(ref['path']).read_bytes()).hexdigest())
            _artifact(ref, self.root)  # Existing hardware-review JSON contract.
        self.assertEqual((self.root/'review.json').stat().st_mode & 0o777, 0o600)
        self.assertEqual(before, {p: p.read_bytes() for p in before})

    def test_old_audit_missing_only_new_max_field_is_supported(self):
        self.mutate_json('audit', lambda d: d['heldout_body_diagnostic'].pop(prepare.MAX_FIELD))
        result = self.run_prepare()
        self.assertGreater(result['imu']['corrected_static_gyro_max_rad_s'], 0)
        self.assertIsNone(result['imu']['yaw_left_verified'])

    def test_forged_max_scalar_is_rejected(self):
        self.mutate_json('audit', lambda d: d['heldout_body_diagnostic'].__setitem__(prepare.MAX_FIELD, 0.))
        with self.assertRaisesRegex(ValueError, 'Saved audit'):
            self.run_prepare()
        self.assertFalse((self.root/'review.json').exists())

    def test_changed_exact_raw_bytes_rejected_even_same_measurements(self):
        path = self.root/'b'/'events.jsonl'
        path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'Saved audit'):
            self.run_prepare()

    def test_changed_manifest_hash_rejected(self):
        path = self.root/'manifest.json'
        path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'manifest hash'):
            self.run_prepare()

    def test_mount_and_bias_must_equal_saved_evidence(self):
        for name, mutation in (
                ('mount', lambda d: d['R_body_from_sensor'].__setitem__(0, [0., -1., 0.])),
                ('bias', lambda d: d['gyro_bias_candidate_rad_s'].__setitem__(0, 0.))):
            with self.subTest(name=name):
                self.save_inputs()
                self.mutate_json(name, mutation)
                with self.assertRaises(ValueError):
                    self.run_prepare()

    def test_unconfirmed_stationarity_cannot_prepare_bias_review(self):
        self.manifest['stationary']['operator_confirmed'] = False
        self.save_inputs()
        with self.assertRaisesRegex(ValueError, 'eligible'):
            self.run_prepare()

    def test_unknown_physical_directions_remain_unknown_even_all_motion_checks_pass(self):
        # Reuse the audited synthetic guided motion fixture, not mocked PASS scalars.
        from test_imu_commissioning_audit import CommissioningAuditTests
        fixture = CommissioningAuditTests()
        fixture.setUp()
        try:
            fixture.root.joinpath('manifest.json').write_text(json.dumps(fixture.manifest))
            result = audit.write_audit(fixture.root/'manifest.json', fixture.root/'audit.json')
            (fixture.root/'mount.json').write_text(json.dumps(result['mount_candidate']))
            (fixture.root/'bias.json').write_text(json.dumps(result['stationary']))
            review = prepare.prepare(*(fixture.root/(k+'.json') for k in ('manifest','audit','mount','bias')),
                                     fixture.root/'review.json')
            for direction in audit.MOVEMENTS:
                self.assertTrue(review['recomputed_evidence']['checks'][direction+'_axis_sign'])
            self.assertTrue(all(review['imu'][key] is None for key in prepare.PHYSICAL_KEYS))
            self.assertEqual(review['review']['decision'], 'UNREVIEWED')
        finally:
            fixture.doCleanups()

    def test_static_and_yaw_only_pins_evidence_without_promoting_old_directions(self):
        from test_imu_commissioning_audit import CommissioningAuditTests
        fixture = CommissioningAuditTests()
        fixture.setUp()
        try:
            fixture.manifest['movements'] = [item for item in fixture.manifest['movements']
                                             if item['movement'] == 'turn_left']
            fixture.root.joinpath('manifest.json').write_text(json.dumps(fixture.manifest))
            result = audit.write_audit(fixture.root/'manifest.json', fixture.root/'audit.json')
            (fixture.root/'mount.json').write_text(json.dumps(result['mount_candidate']))
            (fixture.root/'bias.json').write_text(json.dumps(result['stationary']))
            review = prepare.prepare(*(fixture.root/(k+'.json') for k in ('manifest','audit','mount','bias')),
                                     fixture.root/'review.json')
            checks = review['recomputed_evidence']['checks']
            self.assertTrue(checks['gyro_bias_candidate'])
            self.assertTrue(checks['turn_left_axis_sign'])
            self.assertFalse(checks['nose_up_axis_sign'])
            self.assertFalse(checks['left_side_up_axis_sign'])
            self.assertEqual(review['status'], 'UNREVIEWED')
            self.assertTrue(all(review['imu'][key] is None for key in prepare.PHYSICAL_KEYS))
            self.assertEqual(len(review['references']), 10)
        finally:
            fixture.doCleanups()

    def test_heldout_spike_not_hidden_by_small_mean_or_clipped_to_review_limit(self):
        meta, rows = self.rows['b']
        rows[500]['raw_gyro'][0] += 400
        rows[500]['gyro_rad_s'][0] = rows[500]['raw_gyro'][0]*meta['configuration']['gyro_rad_s_per_lsb']
        refresh_summary(meta, rows)
        self.write_capture('b')
        result = self.save_inputs()
        self.assertTrue(result['checks']['gyro_bias_candidate'])
        review = self.run_prepare()
        self.assertGreater(review['imu']['corrected_static_gyro_max_rad_s'], .02)
        self.assertFalse(review['recomputed_evidence']['numeric_review_contract']['heldout_gyro_max_at_most_0_02_rad_s'])
        self.assertFalse(review['approved_for_runtime'])

    def test_invalid_json_and_approved_candidate_rejected(self):
        original = (self.root/'bias.json').read_bytes()
        for raw in (b'{"x":NaN}', b'{"x":1,"x":2}'):
            (self.root/'bias.json').write_bytes(raw)
            with self.assertRaises(ValueError):
                self.run_prepare()
        (self.root/'bias.json').write_bytes(original)
        self.mutate_json('bias', lambda d: d.__setitem__('approved_for_runtime', True))
        with self.assertRaises(ValueError):
            self.run_prepare()

    def test_concurrent_source_change_rejected(self):
        original = audit.audit_manifest
        def changed(*args, **kwargs):
            value = original(*args, **kwargs)
            path = self.root/'mount.json'
            path.write_bytes(path.read_bytes()+b'\n')
            return value
        with patch.object(audit, 'audit_manifest', side_effect=changed):
            with self.assertRaisesRegex(ValueError, 'Source changed'):
                self.run_prepare()

    def test_output_never_overwrites_or_uses_git_and_symlink_sources_rejected(self):
        self.run_prepare()
        with self.assertRaises(FileExistsError):
            self.run_prepare()
        (self.root/'repository').mkdir()
        (self.root/'repository'/'.git').mkdir()
        with self.assertRaisesRegex(ValueError, 'outside Git'):
            self.run_prepare('repository/out.json')
        (self.root/'mount-link.json').symlink_to(self.root/'mount.json')
        with self.assertRaisesRegex(ValueError, 'Regular source'):
            prepare.prepare(self.root/'manifest.json', self.root/'audit.json', self.root/'mount-link.json',
                            self.root/'bias.json', self.root/'fresh.json')

    def test_cli_prints_only_unreviewed_output_and_missing_input_no_partial_file(self):
        args = [arg for key in ('manifest', 'audit', 'mount', 'bias', 'output')
                for arg in ('--'+key, str(self.root/('cli.json' if key == 'output' else key+'.json')))]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(prepare.main(args), 0)
        value = json.loads(output.getvalue())
        self.assertEqual(value['status'], 'UNREVIEWED')
        self.assertFalse(value['dependency_eligible'])
        self.assertEqual(value['output_sha256'], hashlib.sha256((self.root/'cli.json').read_bytes()).hexdigest())
        (self.root/'a'/'events.jsonl').unlink()
        with self.assertRaises(ValueError):
            self.run_prepare('missing.json')
        self.assertFalse((self.root/'missing.json').exists())


if __name__ == '__main__':
    unittest.main()
