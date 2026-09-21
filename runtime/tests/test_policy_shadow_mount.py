"""File-only tests for unverified IMU mounting hypotheses; no device access."""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_shadow as shadow
from test_policy_shadow import calibration, records, capture_fixture, FakeTorch, Policy


def candidate(rotation=None):
    return {
        'schema_version': 1, 'status': 'IMU_MOUNT_CANDIDATE_ONLY',
        'input_frame': 'sensor', 'output_frame': 'body_x_forward_y_left_z_up',
        'R_body_from_sensor': rotation if rotation is not None else [[0,1,0],[1,0,0],[0,0,-1]],
        'raw_driver_axes_verified': False, 'approved_for_runtime': False,
        'provenance': {'source': 'synthetic test fixture',
                       'observation': 'Printed X left, component face down; driver mapping unverified'},
    }


class MountTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.mount = self.base / 'mount.json'
        self.write_candidate(candidate())

    def write_candidate(self, value):
        self.mount.write_text(json.dumps(value))

    def samples(self, raw=None):
        return shadow.build_samples(records() if raw is None else raw, calibration(),
                                    imu_mount_candidate=self.mount)

    def test_requested_rotation_applies_to_both_vectors_and_preserves_raw(self):
        raw = records()
        imu = next(e for e in raw if e['kind'] == 'imu')
        imu['accel_m_s2'] = [.05, -.15, -10.73]
        imu['gyro_rad_s'] = [.005, .014, -.006]
        unchanged = copy.deepcopy(raw)
        sample = self.samples(raw)[0]
        norm = math.hypot(*imu['accel_m_s2'])
        self.assertEqual(raw, unchanged)
        self.assertEqual(sample['gyro_rad_s'], [.014, .005, .006])
        self.assertEqual(sample['accel_body_candidate_m_s2'], [-.15, .05, 10.73])
        self.assertEqual(sample['gravity_body_unit'], [.15/norm, -.05/norm, -10.73/norm])
        self.assertEqual(sample['raw_accel_m_s2'], imu['accel_m_s2'])
        self.assertEqual(sample['raw_gyro_rad_s'], imu['gyro_rad_s'])
        self.assertEqual(sample['raw_accel_norm_m_s2'], norm)
        self.assertAlmostEqual(sample['raw_accel_norm_relative_deviation'], norm/9.80665-1.)
        self.assertFalse(sample['projected_gravity_z_positive'])
        self.assertIsNone(sample['identity_hypothesis_projected_gravity_z_positive'])
        for flag in ('gyro_bias_subtracted', 'accel_bias_subtracted', 'accel_scale_corrected',
                     'raw_driver_axes_verified', 'sensor_alignment_verified', 'calibration_verified'):
            self.assertFalse(sample['assumptions'][flag])

    def test_existing_identity_hypothesis_keeps_previous_values(self):
        sample = shadow.build_samples(records(), calibration(), assume_sensor_aligned=True)[0]
        self.assertEqual(sample['gyro_rad_s'], [.01, .02, .03])
        self.assertEqual(sample['gravity_body_unit'], [0., 0., -1.])
        self.assertEqual(sample['imu_mount']['mode'], 'identity_hypothesis')
        self.assertIsNone(sample['imu_mount']['source'])

    def test_proper_non_axis_aligned_rotation_is_accepted_without_snapping(self):
        angle = .37
        c, s = math.cos(angle), math.sin(angle)
        rotation = [[c,-s,0.],[s,c,0.],[0.,0.,1.]]
        self.write_candidate(candidate(rotation))
        sample = self.samples()[0]
        self.assertAlmostEqual(sample['gyro_rad_s'][0], c*.01-s*.02)
        self.assertAlmostEqual(sample['gyro_rad_s'][1], s*.01+c*.02)
        self.assertEqual(sample['imu_mount']['R_body_from_sensor'], rotation)

    def test_reflection_scale_shear_and_nonfinite_rotations_are_rejected(self):
        invalid = [
            [[1,0,0],[0,1,0],[0,0,-1]],  # Preserves norms but is not SO(3).
            [[1,0,0],[0,1,0],[0,0,.99]],
            [[1,.01,0],[0,1,0],[0,0,1]],
            [[1,0,0],[0,1,0],[0,0,math.nan]],
            [[1,0,0],[0,1,0],[0,0,math.inf]],
            [[True,0,0],[0,1,0],[0,0,1]],
            [[10**1000,0,0],[0,1,0],[0,0,1]],
            [[1,0,0],[0,1,0]], [[1,0,0],[0,1,0],[0,0]],
        ]
        for rotation in invalid:
            with self.subTest(rotation=str(rotation)[:100]), self.assertRaises(ValueError):
                shadow.validate_imu_mount_candidate(candidate(rotation))

    def test_candidate_cannot_claim_validation_or_supply_hidden_bias(self):
        changes = [('raw_driver_axes_verified', True), ('approved_for_runtime', True),
                   ('raw_driver_axes_verified', 0), ('schema_version', True),
                   ('input_frame', 'body'), ('output_frame', 'sensor'),
                   ('provenance', {}), ('provenance', {'source': ''}),
                   ('gyro_bias', [0., 0., 0.])]
        for key, value in changes:
            data = candidate()
            data[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                shadow.validate_imu_mount_candidate(data)

    def test_candidate_source_hash_is_over_exact_bytes_and_preserves_provenance(self):
        raw = (json.dumps(candidate(), indent=3) + '\n').encode()
        self.mount.write_bytes(raw)
        result = shadow.load_imu_mount_candidate(self.mount)
        self.assertEqual(result['source'], {'path': str(self.mount.resolve()),
                                          'sha256': hashlib.sha256(raw).hexdigest()})
        self.assertEqual(result['provenance'], candidate()['provenance'])
        self.assertFalse(result['approved_for_runtime'])
        self.assertFalse(result['raw_driver_axes_verified'])
        self.assertFalse(result['sensor_alignment_verified'])

    def test_duplicate_json_fields_and_missing_candidate_file_are_rejected(self):
        self.mount.write_text('{"schema_version":1,"schema_version":1}')
        with self.assertRaises(ValueError):
            self.samples()
        self.mount.unlink()
        with self.assertRaises(FileNotFoundError):
            self.samples()

    def test_python_api_rejects_ambiguous_hypotheses(self):
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            shadow.build_samples(records(), calibration(), assume_sensor_aligned=True,
                                 imu_mount_candidate=self.mount)

    def test_transformed_vectors_reach_policy_unchanged_without_hidden_corrections(self):
        raw = records()
        next(e for e in raw if e['kind'] == 'imu')['accel_m_s2'] = [0., 0., -10.73]
        samples = self.samples(raw)
        policy = Policy()
        shadow.evaluate_samples(samples, policy, FakeTorch)
        for call in policy.calls:
            self.assertEqual(call[0], [.02, .01, -.03])
            self.assertEqual(call[1], [0., 0., -1.])
        self.assertEqual(samples[0]['raw_accel_norm_m_s2'], 10.73)

    def test_cli_options_are_exclusive_before_loading_capture_or_model(self):
        args = ['--bundle', str(self.base), '--capture', str(self.base),
                '--calibration', str(self.base/'cal.json'), '--output', str(self.base/'out'),
                '--assume-sensor-aligned', '--imu-mount-candidate', str(self.mount)]
        with patch.object(shadow, 'load_capture') as capture, patch.object(shadow, 'load_policy') as model:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                shadow.main(args)
            capture.assert_not_called()
            model.assert_not_called()

    def test_cli_receipt_persists_candidate_without_upgrading_approval(self):
        capture = self.base/'capture'
        capture.mkdir()
        capture_fixture(capture)
        cal = self.base/'cal.json'
        cal.write_text(json.dumps(calibration()))
        output = self.base/'out'
        evaluate = shadow.evaluate_samples
        with patch.object(shadow, 'load_policy', return_value=(Policy(), {'test': 'synthetic'})), \
                patch.object(shadow, 'evaluate_samples', side_effect=lambda s,p: evaluate(s,p,FakeTorch)), \
                contextlib.redirect_stdout(io.StringIO()):
            code = shadow.main(['--bundle', str(self.base), '--capture', str(capture),
                                '--calibration', str(cal), '--output', str(output),
                                '--imu-mount-candidate', str(self.mount)])
        self.assertEqual(code, 0)
        report = json.loads((output/'summary.json').read_text())
        self.assertEqual(report['imu_mount']['source']['sha256'], shadow.sha(self.mount))
        self.assertEqual(report['imu_mount']['provenance'], candidate()['provenance'])
        self.assertTrue(report['mount_hypothesis_projects_gravity_upward'])
        self.assertIsNone(report['identity_hypothesis_projects_gravity_upward'])
        for key in ('approved_for_runtime', 'raw_driver_axes_verified', 'sensor_alignment_verified',
                    'hardware_opened', 'motor_output_available', 'calibration_verified'):
            self.assertFalse(report[key])


if __name__ == '__main__':
    unittest.main()
