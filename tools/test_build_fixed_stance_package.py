"""Evidence-bound, disabled-only package tests with no hardware access."""

import json
import math
from pathlib import Path
import tempfile
import unittest

import build_fixed_stance_package as builder
from singularitydog_hw.fixed_stance_package import verify_package_files


BOOT = 'same-jetson-boot'
UIDS = {str(i): f'{i:016x}' for i in range(1, 13)}
START = {str(i): .5 + .1*i for i in range(1, 13)}
TARGET = {str(i): START[str(i)] + math.radians(5.) for i in range(1, 13)}


def put(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + '\n')


def hold():
    workers = {}
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        workers[bus] = {
            'completed': True, 'cycle_count': 100,
            'centers': {str(i): START[str(i)] for i in ids},
            'stop_reports': {str(i): {'confirmed': True} for i in ids},
        }
    return {
        'boot_id': BOOT, 'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
        'errors': [], 'motor_enable_sent': True, 'motion_gain_sent': True,
        'trial_device_closed': True, 'locks_released': True,
        'result': {
            'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
            'stop_confirmed': True, 'errors': [], 'gain_profile': 'id4-id10-kp4',
            'review': {'motor_uids': UIDS}, 'workers': workers,
        },
    }


def capture():
    return {
        'schema': builder.CAPTURE_SCHEMA, 'boot_id': BOOT,
        'motor_uids': UIDS, 'read_only': True, 'motor_enable_sent': False,
        'supported_pose_placed_by_operator': True,
        'simultaneous_physical_stance_verified': True, 'stand_removed': False,
        'pose_class': builder.FLOOR_STANCE_CLASS, 'foot_support_kind': 'floor',
        'foot_support_height_cm': 0, 'physical_pose_review_sha256': 'd'*64,
        'sampling_stability_heuristic_passed': True,
        'output_allowed': False, 'approved_for_runtime': False,
        'operator_note': 'Operator placed and held all four feet in a supported stance.',
        'evidence_reference': 'private/front-and-side-floor-pose-video.mp4',
        'raw_rad_by_id': TARGET,
    }


def route(hold_hash, capture_hash):
    return {
        'schema': builder.ROUTE_SCHEMA, 'boot_id': BOOT,
        'motor_uids': UIDS, 'hold_summary_sha256': hold_hash,
        'stance_capture_sha256': capture_hash,
        'start_raw_rad_by_id': START,
        'fixed_stance_raw_rad_by_id': TARGET,
        'old_d17_target_reused': False,
        'clearance_reviewed_start_envelope_deg': 3.,
        'reviewed_raw_corridor_by_id': {str(i): {
            'min_rad': START[str(i)] - math.radians(4.),
            'max_rad': START[str(i)] + math.radians(12.),
        } for i in range(1, 13)},
        'raw_corridor_physical_source_note': 'Virtual test corridor, no physical claim.',
        'whole_route_physically_reviewed': True,
        'support_and_foot_clearance_verified': True,
        'attended_40v_cutoff_ready': True,
        'waypoints_raw_rad_by_id': [TARGET],
        'segments': [{
            'index': 0, 'from_raw_rad_by_id': START,
            'to_raw_rad_by_id': TARGET,
            'swept_clearance_verified': True,
            'all_joint_limits_verified': True,
            'front_upper_leg_carbon_clamp_clear': True,
            'operator_note': 'Full sweep was checked at this supported pose.',
        }],
    }


class FrozenStancePackageTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.hold = self.root / 'hold.json'
        self.capture = self.root / 'capture.json'
        self.route = self.root / 'route.json'
        self.output = self.root / 'package'
        put(self.hold, hold())
        put(self.capture, capture())
        put(self.route, route(builder.sha(self.hold), builder.sha(self.capture)))

    def build(self):
        return builder.build(self.hold, self.capture, self.route, self.output,
                             expected_boot_id=BOOT)

    def test_copies_and_hashes_actual_files_but_never_authorizes_output(self):
        result = self.build()
        self.assertEqual(result['status'], 'DISABLED_ONLY_FILE_PACKAGE')
        self.assertFalse(result['output_allowed'])
        self.assertFalse(result['live_runner_enabled'])
        review = builder.read_json(self.output / 'review.json')
        self.assertFalse(review['supported_transition_authorized'])
        self.assertFalse(review['joint_limits_all_samples_verified'])
        self.assertFalse(review['physical_route_all_samples_verified'])
        self.assertTrue(review['operator_asserted_joint_limits_on_segments'])
        self.assertTrue(review['operator_asserted_swept_clearance_on_segments'])
        self.assertEqual(review['hold_summary_sha256'], builder.sha(self.hold))
        self.assertEqual(review['stance_capture_sha256'], builder.sha(self.capture))
        self.assertEqual(review['physical_route_review_sha256'], builder.sha(self.route))
        candidate = builder.read_json(self.output / 'candidate.json')
        self.assertFalse(candidate['output_allowed'])
        manifest = builder.read_json(self.output / 'manifest.json')
        self.assertTrue(manifest)
        self.assertTrue(all(builder.sha(self.output / name) == digest
                            for name, digest in manifest.items()))
        for directory in (self.output, self.output / 'evidence', self.output / 'source'):
            self.assertEqual(directory.stat().st_mode & 0o777, 0o700)
        for path in (self.output / 'review.json', self.output / 'candidate.json',
                     self.output / 'manifest.json',
                     *(self.output / 'evidence' / name for name in
                       ('hold-summary.json', 'stance-capture.json', 'physical-route.json'))):
            self.assertEqual(path.stat().st_mode & 0o777, 0o600, str(path))

    def test_git_checkout_symlink_and_existing_output_rejected(self):
        checkout = self.root / 'checkout'
        checkout.mkdir()
        (checkout / '.git').mkdir()
        alias = self.root / 'checkout-alias'
        alias.symlink_to(checkout, target_is_directory=True)
        for output in (checkout / 'package', alias / 'package'):
            with self.subTest(output=output), self.assertRaisesRegex(ValueError, 'outside Git'):
                builder.build(self.hold, self.capture, self.route, output,
                              expected_boot_id=BOOT)
            self.assertFalse((checkout / 'package').exists())
        link = self.root / 'package-link'
        link.symlink_to(self.root / 'absent-package', target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'must not be a symlink'):
            builder.build(self.hold, self.capture, self.route, link,
                          expected_boot_id=BOOT)
        self.build()
        with self.assertRaisesRegex(ValueError, 'fresh package path'):
            self.build()

    def test_prior_boot_or_changed_capture_rejected_without_partial_package(self):
        for mutation in ('boot', 'capture'):
            with self.subTest(mutation=mutation):
                data = builder.read_json(self.route)
                if mutation == 'boot':
                    data['boot_id'] = 'prior-boot'
                else:
                    data['stance_capture_sha256'] = 'f'*64
                put(self.route, data)
                with self.assertRaises(ValueError):
                    self.build()
                self.assertFalse(self.output.exists())
                put(self.route, route(builder.sha(self.hold), builder.sha(self.capture)))

    def test_unreviewed_carbon_route_and_old_d17_rejected(self):
        for key, value in (('front_upper_leg_carbon_clamp_clear', False),
                           ('old_d17_target_reused', True)):
            with self.subTest(key=key):
                data = builder.read_json(self.route)
                if key == 'old_d17_target_reused':
                    data[key] = value
                else:
                    data['segments'][0][key] = value
                put(self.route, data)
                with self.assertRaises(ValueError):
                    self.build()
                self.assertFalse(self.output.exists())
                put(self.route, route(builder.sha(self.hold), builder.sha(self.capture)))

    def test_l_calibration_or_raised_foot_platform_cannot_be_packaged(self):
        for field, value in (('pose_class', 'calibration_L'),
                             ('foot_support_kind', 'platform'),
                             ('foot_support_height_cm', 12),
                             ('sampling_stability_heuristic_passed', False)):
            with self.subTest(field=field):
                data = capture()
                data[field] = value
                put(self.capture, data)
                with self.assertRaisesRegex(ValueError, 'physical pose'):
                    self.build()
                self.assertFalse(self.output.exists())
        put(self.capture, capture())

    def test_runtime_verifies_entire_frozen_package_and_detects_tampering(self):
        self.build()
        review = builder.read_json(self.output / 'review.json')
        verified = verify_package_files(self.output, review)
        self.assertEqual(verified['manifest_sha256'], builder.sha(self.output / 'manifest.json'))
        for changed_path in (self.output / 'evidence' / 'physical-route.json',
                             self.output / 'source' / 'rs05_fixed_stance_trial.py'):
            with self.subTest(path=changed_path.name):
                original = changed_path.read_bytes()
                changed_path.write_bytes(original + b' ')
                with self.assertRaisesRegex(ValueError, 'changed'):
                    verify_package_files(self.output, review)
                changed_path.write_bytes(original)
        bad_review = dict(review, boot_id='different-boot')
        with self.assertRaisesRegex(ValueError, 'review differs'):
            verify_package_files(self.output, bad_review)
        candidate_path = self.output / 'candidate.json'
        original_candidate = candidate_path.read_bytes()
        candidate = builder.read_json(candidate_path)
        candidate['samples'][1]['raw_rad_by_id']['1'] += .001
        put(candidate_path, candidate)
        manifest_path = self.output / 'manifest.json'
        original_manifest = manifest_path.read_bytes()
        manifest = builder.read_json(manifest_path)
        manifest['candidate.json'] = builder.sha(candidate_path)
        put(manifest_path, manifest)
        with self.assertRaisesRegex(ValueError, 'regenerated finite plan'):
            verify_package_files(self.output, review)
        candidate_path.write_bytes(original_candidate)
        manifest_path.write_bytes(original_manifest)
        extra = self.output / 'source' / 'unreviewed_helper.py'
        extra.write_text('pass\n')
        with self.assertRaisesRegex(ValueError, 'missing or extra files'):
            verify_package_files(self.output, review)


if __name__ == '__main__':
    unittest.main()
