"""Synthetic, file-only gates for the full-human-support package builder."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build_load_transfer_2s_human_supported as builder
from human_supported_capture_guard import (PARAMETERS, PARAMETER_INDEX,
                                           PHYSICAL_FLAGS, sha,
                                           validate_capture)
import human_supported_capture_guard as guard


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'runtime/singularitydog_hw/fixed_stance_readonly_capture.py'
CAN_SOURCE = ROOT / 'runtime/singularitydog_hw/can_readonly.py'
BOOT = 'synthetic-boot-only'
UIDS = {str(i): f'{i:016x}' for i in range(1, 13)}


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + '\n')


def request(mid, parameter):
    kind = 0 if parameter == 'identity' else 17
    can_id = (kind << 27) | 0x7e804 | (mid << 3)
    data = (bytes(8) if parameter == 'identity' else
            PARAMETER_INDEX[parameter].to_bytes(2, 'little') + bytes(6))
    return (b'AT' + can_id.to_bytes(4, 'big') + b'\x08' + data + b'\r\n').hex()


class SyntheticCapture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.summary = self.root / 'summary.json'
        self.events = self.root / 'events.jsonl'
        self.draft = self.root / 'capture-draft.json'
        self.physical = self.root / 'physical-review.json'
        self.capture = self.make_capture()
        self.persist()

    def make_capture(self):
        samples = {}
        raw = {}
        for mid in range(1, 13):
            center = mid / 10.
            raw[str(mid)] = center
            rows = []
            for sweep in range(3):
                row = {}
                for index, key in enumerate(PARAMETERS):
                    value = {'position': center, 'velocity': .01, 'current': 0.,
                             'voltage': 40., 'run_mode': 0.}[key]
                    timestamp = 1_000 + mid * 1_000 + sweep * 100 + index * 10
                    row[key] = {'value': value, 'request_monotonic_ns': timestamp,
                                'reply_monotonic_ns': timestamp + 1}
                rows.append(row)
            samples[str(mid)] = rows
        return {
            'schema': 'singularitydog.fixed-stance-readonly-capture.v1',
            'status': 'RECORDED_REVIEW_REQUIRED', 'boot_id': BOOT, 'errors': [],
            'output_allowed': False, 'approved_for_runtime': False,
            'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
            'operator_enter_monotonic_ns': 100,
            'source_sha256': {'fixed_stance_readonly_capture.py': sha(SOURCE),
                              'can_readonly.py': sha(CAN_SOURCE)},
            'plan': {'allowed_can_types': [0, 17], 'motor_output_available': False,
                     'stop_command_available': False, 'sweeps': 3},
            'identities': {mid: {'mcu_uid_hex': uid} for mid, uid in UIDS.items()},
            'pose': {'raw_rad_by_id': raw, 'samples': samples,
                     'started_monotonic_ns': 200, 'ended_monotonic_ns': 50_000,
                     'sampling_issues': [],
                     'sampling_stability_heuristic_passed': True,
                     'stationarity_verified': False},
        }

    def persist(self):
        write_json(self.summary, self.capture)
        events = []
        for mid in range(1, 13):
            bus = 'front' if mid <= 6 else 'rear'
            for parameter in ('identity', *PARAMETERS):
                times = 1 if parameter == 'identity' else 3
                for _ in range(times):
                    events.append({'kind': 'can_tx', 'bus': bus,
                                   'motor_id': mid, 'parameter': parameter,
                                   'hex': request(mid, parameter)})
                    events.append({'kind': 'can_rx_frame',
                                   'type': 0 if parameter == 'identity' else 17})
        self.events.write_text(''.join(json.dumps(event) + '\n' for event in events))
        draft = {'schema': 'singularitydog.fixed-stance-capture-draft.v1',
                 'boot_id': BOOT, 'motor_uids': UIDS,
                 'raw_rad_by_id': self.capture['pose']['raw_rad_by_id'],
                 'stance_capture_sha256': sha(self.summary),
                 'output_allowed': False, 'approved_for_runtime': False}
        write_json(self.draft, draft)
        physical = {'schema': 'singularitydog.human-supported-physical-review.v1',
                    'boot_id': BOOT, 'capture_summary_sha256': sha(self.summary),
                    'capture_events_sha256': sha(self.events),
                    'capture_draft_sha256': sha(self.draft),
                    'operator_note': 'Synthetic four-paw, two-person support review.'}
        physical.update({flag: True for flag in PHYSICAL_FLAGS})
        write_json(self.physical, physical)

    def check(self):
        return validate_capture(self.summary, self.events, self.draft, SOURCE,
                                self.physical, boot=BOOT, uids=UIDS,
                                can_readonly_sha256=sha(CAN_SOURCE))

    def test_valid_complete_capture(self):
        self.assertEqual(self.check(), self.capture['pose']['raw_rad_by_id'])

    def test_known_stand_supported_capture_cannot_be_relabelled(self):
        with mock.patch.object(guard, 'DISQUALIFIED_STAND_SUPPORTED_CAPTURE_SHA256',
                               {sha(self.summary)}):
            with self.assertRaisesRegex(ValueError, 'Known stand-supported capture'):
                self.check()

    def test_velocity_spike_and_failed_heuristic_cannot_be_waived(self):
        self.capture['pose']['samples']['3'][1]['velocity']['value'] = .1153
        self.capture['pose']['sampling_issues'] = [{'motor_id': 3}]
        self.capture['pose']['sampling_stability_heuristic_passed'] = False
        self.persist()
        with self.assertRaisesRegex(ValueError, 'stability gate failed'):
            self.check()

    def test_lying_heuristic_still_rejects_velocity_spike(self):
        self.capture['pose']['samples']['8'][1]['velocity']['value'] = .1179
        self.persist()
        with self.assertRaisesRegex(ValueError, 'electrical/motion envelope'):
            self.check()

    def test_mismatched_boot_uid_or_physical_report_refused(self):
        self.capture['boot_id'] = 'other-boot'
        self.persist()
        with self.assertRaisesRegex(ValueError, 'same-boot'):
            self.check()
        self.capture['boot_id'] = BOOT
        self.capture['identities']['7']['mcu_uid_hex'] = 'deadbeefdeadbeef'
        self.persist()
        with self.assertRaisesRegex(ValueError, 'identities differ'):
            self.check()
        self.capture['identities']['7']['mcu_uid_hex'] = UIDS['7']
        self.persist()
        physical = json.loads(self.physical.read_text())
        physical['support_stand_absent_during_capture'] = False
        write_json(self.physical, physical)
        with self.assertRaisesRegex(ValueError, 'physical review'):
            self.check()
        physical['support_stand_absent_during_capture'] = True
        physical['no_load_easing_planned'] = False
        write_json(self.physical, physical)
        with self.assertRaisesRegex(ValueError, 'physical review'):
            self.check()

    def test_nonreadonly_can_request_refused(self):
        lines = self.events.read_text().splitlines()
        first = json.loads(lines[0])
        wire = bytearray.fromhex(first['hex'])
        wire[2] = 0x18  # Type3 Enable in place of Type0 identity.
        first['hex'] = wire.hex()
        lines[0] = json.dumps(first)
        self.events.write_text('\n'.join(lines) + '\n')
        physical = json.loads(self.physical.read_text())
        physical['capture_events_sha256'] = sha(self.events)
        write_json(self.physical, physical)
        with self.assertRaisesRegex(ValueError, 'nonread-only CAN request'):
            self.check()

    def test_builder_refuses_unstable_capture_without_creating_package(self):
        self.capture['pose']['sampling_issues'] = [{'motor_id': 3}]
        self.capture['pose']['sampling_stability_heuristic_passed'] = False
        self.persist()
        source = self.root / 'prior'
        source.mkdir()
        (source / 'manifest.json').write_text('{}\n')
        (source / 'prepared_load_transfer.py').write_bytes(builder.WRAPPER.read_bytes())
        source_manifest = {'singularitydog_hw/can_readonly.py': sha(CAN_SOURCE)}
        write_json(source / 'manifest.json', source_manifest)
        out = self.root / 'frozen'
        with (mock.patch.object(builder, 'REVIEWED_SOURCE_MANIFEST_SHA256',
                                sha(source / 'manifest.json')),
              mock.patch.object(builder, 'BOOT', BOOT),
              mock.patch.object(builder, 'verify_prior',
                                return_value={'motor_uids': UIDS})):
            with self.assertRaisesRegex(ValueError, 'stability gate failed'):
                builder.build(source, self.summary, self.events, self.draft,
                              SOURCE, self.physical, source, '0' * 64,
                              self.root / 'disabled-summary.json',
                              self.root / 'disabled-events.jsonl', out,
                              '/home/jetson/singularitydog-tests/load-transfer-2s-human-supported-test')
        self.assertFalse(out.exists())

    def test_builder_freezes_synthetic_private_bundle_with_hashes(self):
        source = self.root / 'prior'
        (source / 'singularitydog_hw').mkdir(parents=True)
        (source / 'evidence').mkdir()
        for name in builder.SOURCES:
            payload = (CAN_SOURCE.read_bytes() if name == 'can_readonly.py'
                       else (b'LIVE_OUTPUT_ENABLED = True\n' if name == 'rs05_load_transfer_hold.py'
                             else ('synthetic reviewed ' + name).encode()))
            (source / 'singularitydog_hw' / name).write_bytes(payload)
        for name in builder.PRIOR_EVIDENCE:
            (source / 'evidence' / name).write_text('synthetic prior evidence\n')
        (source / 'prepared_load_transfer.py').write_bytes(builder.WRAPPER.read_bytes())
        source_manifest = {'singularitydog_hw/' + name:
                           sha(source / 'singularitydog_hw' / name)
                           for name in builder.SOURCES}
        write_json(source / 'manifest.json', source_manifest)
        disabled = self.root / 'disabled'
        (disabled / 'singularitydog_hw').mkdir(parents=True)
        for name in builder.SOURCES:
            payload = (b'LIVE_OUTPUT_ENABLED = False\n'
                       if name == 'rs05_load_transfer_hold.py' else
                       (source / 'singularitydog_hw' / name).read_bytes())
            (disabled / 'singularitydog_hw' / name).write_bytes(payload)
        disabled_manifest = {'singularitydog_hw/' + name:
                             sha(disabled / 'singularitydog_hw' / name)
                             for name in builder.SOURCES}
        write_json(disabled / 'manifest.json', disabled_manifest)
        disabled_summary = self.root / 'disabled-summary.json'
        disabled_events = self.root / 'disabled-events.jsonl'
        write_json(disabled_summary, {'synthetic': True})
        disabled_events.write_text('{"kind":"synthetic"}\n')
        before = {str(path.relative_to(source)): sha(path)
                  for path in source.rglob('*') if path.is_file()}
        stub = ("from pathlib import Path\n"
                "import hashlib, json\n"
                "def verify_files(base, digest):\n"
                "    base = Path(base)\n"
                "    assert hashlib.sha256((base / 'manifest.json').read_bytes()).hexdigest() == digest\n"
                "    manifest = json.loads((base / 'manifest.json').read_text())\n"
                "    assert all(hashlib.sha256((base / name).read_bytes()).hexdigest() == value "
                "for name, value in manifest.items())\n")
        out = self.root / 'frozen'
        with (mock.patch.object(builder, 'REVIEWED_SOURCE_MANIFEST_SHA256',
                                sha(source / 'manifest.json')),
              mock.patch.object(builder, 'BOOT', BOOT),
              mock.patch.object(builder, 'verify_prior',
                                return_value={'motor_uids': UIDS}),
              mock.patch.object(builder, 'validate_disabled_package',
                                return_value=disabled_manifest),
              mock.patch.object(builder, 'render_wrapper', return_value=stub)):
            result = builder.build(
                source, self.summary, self.events, self.draft, SOURCE,
                self.physical, disabled, sha(disabled / 'manifest.json'),
                disabled_summary, disabled_events, out,
                '/home/jetson/singularitydog-tests/load-transfer-2s-human-supported-test')
        self.assertEqual(result['status'], 'HUMAN_SUPPORTED_PACKAGE_FROZEN_NOT_RUN')
        self.assertFalse(result['motor_output_sent'])
        self.assertEqual(result['manifest_sha256'], sha(out / 'manifest.json'))
        self.assertEqual((out / 'singularitydog_hw/rs05_load_transfer_hold.py').read_bytes(),
                         (source / 'singularitydog_hw/rs05_load_transfer_hold.py').read_bytes())
        self.assertEqual(before, {str(path.relative_to(source)): sha(path)
                                  for path in source.rglob('*') if path.is_file()})
        review = json.loads((out / 'review.json').read_text())
        self.assertFalse(review['partial_load_allowed'])
        self.assertTrue(review['no_load_easing_required'])
        self.assertEqual(review['human_capture_summary_sha256'], sha(self.summary))
        self.assertEqual(review['human_disabled_summary_sha256'], sha(disabled_summary))


class WrapperRenderTests(unittest.TestCase):
    def test_render_keeps_original_wire_gate_and_requires_full_support(self):
        original = builder.WRAPPER.read_text()
        rendered = builder.render_wrapper(
            original, '/home/jetson/singularitydog-tests/load-transfer-2s-human-supported-test')
        compile(rendered, '<synthetic package wrapper>', 'exec')
        self.assertIn('class ExactWirePort:', rendered)
        self.assertEqual(original.split('class ExactWirePort:', 1)[1].split('\ndef verify_files(', 1)[0],
                         rendered.split('class ExactWirePort:', 1)[1].split('\ndef verify_files(', 1)[0])
        self.assertIn('fresh raw start is outside supported floor envelope',
                      (ROOT / 'runtime/singularitydog_hw/rs05_load_transfer_hold.py').read_text())
        self.assertIn("args.two_operators_full_support, args.no_load_easing", rendered)
        self.assertIn("review.get('full_support_through_stop_required') is True", rendered)
        self.assertIn('verify_package_capture(base, review, manifest, BOOT)', rendered)
        self.assertIn('verify_disabled_result(base, review, manifest, BOOT)', rendered)
        self.assertNotIn('args.stand_fully_supporting', rendered)
        self.assertNotIn('slight_ease_max_duration_s', rendered)

    def test_renderer_rejects_wrong_source_or_remote_path(self):
        with self.assertRaisesRegex(ValueError, 'exact reviewed source'):
            builder.render_wrapper(builder.WRAPPER.read_text() + '\n',
                                   '/home/jetson/singularitydog-tests/load-transfer-2s-human-supported-test')
        with self.assertRaisesRegex(ValueError, 'Remote package path'):
            builder.render_wrapper(builder.WRAPPER.read_text(), '/tmp/hold')


if __name__ == '__main__':
    unittest.main()
