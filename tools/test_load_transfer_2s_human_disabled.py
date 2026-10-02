"""Offline-only bundle and log gates for the future human-supported preflight."""

import json
from pathlib import Path
import shutil
import unittest
from unittest import mock

import build_load_transfer_2s_human_disabled as builder
import test_load_transfer_2s_human_supported as fixture_module
from human_supported_capture_guard import sha


PRIVATE = Path('/private/tmp/fabo-stance-20260927-private')
SOURCE = PRIVATE / 'load-transfer-2s-preflight-r8'
PRIOR_LOG = PRIVATE / 'load-transfer-2s-preflight-20260927-r8'
KNOWN_STAND_R2 = PRIVATE / 'fixed-stance-human-supported-20260927-r2'
REMOTE = '/home/jetson/singularitydog-tests/load-transfer-2s-human-disabled-synthetic'


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + '\n')


def require_saved_files(*paths):
    """Historical private captures are optional; present corrupt files still fail."""
    if any(not path.is_file() for path in paths):
        raise unittest.SkipTest('Optional 2026-09-27 private raw capture/package is unavailable')


class HumanDisabledOffline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_saved_files(SOURCE / 'prepared_load_transfer.py', SOURCE / 'manifest.json')
        cls.prior_review = builder._load_frozen_wrapper(
            SOURCE / 'prepared_load_transfer.py').verify_files(
                SOURCE, builder.REVIEWED_PACKAGE_MANIFEST_SHA256)

    def setUp(self):
        with (mock.patch.object(fixture_module, 'BOOT', builder.BOOT),
              mock.patch.object(fixture_module, 'UIDS', self.prior_review['motor_uids'])):
            self.fixture = fixture_module.SyntheticCapture()
            self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def build(self):
        frozen = self.fixture.root / 'frozen'
        result = builder.build(
            SOURCE, self.fixture.summary, self.fixture.events,
            self.fixture.draft, fixture_module.SOURCE,
            self.fixture.physical, frozen, REMOTE)
        return frozen, result

    def test_reviewed_runtime_is_unchanged_and_bundle_is_hash_bound(self):
        frozen, result = self.build()
        self.assertEqual(result['status'], 'HUMAN_DISABLED_PACKAGE_FROZEN_NOT_RUN')
        self.assertFalse(result['motor_output_sent'])
        self.assertEqual(result['manifest_sha256'], sha(frozen / 'manifest.json'))
        for name in builder.SOURCES:
            self.assertEqual(
                sha(frozen / 'singularitydog_hw' / name),
                sha(SOURCE / 'singularitydog_hw' / name))
        self.assertIn('LIVE_OUTPUT_ENABLED = False',
                      (frozen / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text())
        wrapper = builder._load_frozen_wrapper(frozen / 'prepared_load_transfer.py')
        review = wrapper.verify_files(frozen, result['manifest_sha256'])
        self.assertEqual(review['supported_floor_start_raw_rad_by_id'],
                         self.fixture.capture['pose']['raw_rad_by_id'])
        self.assertFalse(review['supported_stance_passed'])
        self.assertFalse(review['load_transfer_hold_authorized'])

    def test_known_center_stand_r2_is_rejected_without_output(self):
        require_saved_files(*(KNOWN_STAND_R2 / name for name in
                              ('summary.json','events.jsonl','capture-draft.json')))
        frozen = self.fixture.root / 'must-not-exist'
        with self.assertRaisesRegex(ValueError, 'Center-stand r2'):
            builder.build(
                SOURCE, KNOWN_STAND_R2 / 'summary.json',
                KNOWN_STAND_R2 / 'events.jsonl',
                KNOWN_STAND_R2 / 'capture-draft.json',
                fixture_module.SOURCE, self.fixture.physical,
                frozen, REMOTE)
        self.assertFalse(frozen.exists())

    def test_missing_stand_absence_report_is_rejected_without_output(self):
        physical = json.loads(self.fixture.physical.read_text())
        physical['support_stand_absent_during_capture'] = False
        write_json(self.fixture.physical, physical)
        with self.assertRaisesRegex(ValueError, 'physical review'):
            self.build()
        self.assertFalse((self.fixture.root / 'frozen').exists())

    def synthetic_pass_log(self):
        # Match a synthetic capture to an already recorded r8 disabled center
        # set, then rebind only its file-only review and wrapper hashes. Nothing
        # here invokes a serial/CAN implementation or creates a real approval.
        require_saved_files(PRIOR_LOG / 'summary.json', PRIOR_LOG / 'events.jsonl')
        old = json.loads((PRIOR_LOG / 'summary.json').read_text())
        for bus in ('front', 'rear'):
            for mid, center in old['result']['workers'][bus]['centers'].items():
                self.fixture.capture['pose']['raw_rad_by_id'][mid] = center
                for sweep in self.fixture.capture['pose']['samples'][mid]:
                    sweep['position']['value'] = center
        with (mock.patch.object(fixture_module, 'BOOT', builder.BOOT),
              mock.patch.object(fixture_module, 'UIDS', self.prior_review['motor_uids'])):
            self.fixture.persist()
        frozen, package = self.build()
        logs = self.fixture.root / 'logs'
        logs.mkdir()
        shutil.copyfile(PRIOR_LOG / 'events.jsonl', logs / 'events.jsonl')
        old['human_supported'] = True
        old['stand_absent'] = True
        old['wrapper_sha256'] = json.loads((frozen / 'manifest.json').read_text())[
            'prepared_load_transfer.py']
        old['events_sha256'] = sha(logs / 'events.jsonl')
        old['result']['review'] = json.loads((frozen / 'preflight-review.json').read_text())
        write_json(logs / 'summary.json', old)
        return frozen, package, logs

    def test_future_log_gate_checks_all_axes_static_window_and_stop(self):
        frozen, package, logs = self.synthetic_pass_log()
        checked = builder.validate_disabled_run(
            frozen, logs / 'summary.json', logs / 'events.jsonl',
            package['manifest_sha256'])
        self.assertEqual(checked['status'], 'HUMAN_DISABLED_PREFLIGHT_VERIFIED')
        self.assertEqual(checked['capture_summary_sha256'], sha(self.fixture.summary))
        old = json.loads((logs / 'summary.json').read_text())
        old['result']['workers']['front']['settled_windows']['FR']['motors']['1'][
            'sample_count'] = 20
        write_json(logs / 'summary.json', old)
        with self.assertRaisesRegex(ValueError, 'static-window'):
            builder.validate_disabled_run(
                frozen, logs / 'summary.json', logs / 'events.jsonl',
                package['manifest_sha256'])
        old['result']['workers']['front']['settled_windows']['FR']['motors']['1'][
            'sample_count'] = 21
        old['result']['workers']['rear']['stop_reports']['12']['confirmed'] = False
        write_json(logs / 'summary.json', old)
        with self.assertRaisesRegex(ValueError, 'STOP gate'):
            builder.validate_disabled_run(
                frozen, logs / 'summary.json', logs / 'events.jsonl',
                package['manifest_sha256'])

    def test_future_log_gate_rejects_enable_and_tampered_bundle(self):
        frozen, package, logs = self.synthetic_pass_log()
        old = json.loads((logs / 'summary.json').read_text())
        old['motor_enable_sent'] = True
        write_json(logs / 'summary.json', old)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            builder.validate_disabled_run(
                frozen, logs / 'summary.json', logs / 'events.jsonl',
                package['manifest_sha256'])
        old['motor_enable_sent'] = False
        write_json(logs / 'summary.json', old)
        (frozen / 'singularitydog_hw/rs05_load_transfer_hold.py').write_text(
            'LIVE_OUTPUT_ENABLED = True\n')
        with self.assertRaisesRegex(RuntimeError, 'pin mismatch'):
            builder.validate_disabled_run(
                frozen, logs / 'summary.json', logs / 'events.jsonl',
                package['manifest_sha256'])


if __name__ == '__main__':
    unittest.main()
