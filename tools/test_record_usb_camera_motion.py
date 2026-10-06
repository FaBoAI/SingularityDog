"""Meaningful offline capture/failure tests; injected backends never import GI."""
import builtins
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import record_usb_camera_motion as tool

TEST_DEVICE = '/dev/v4l/by-id/usb-SYNTHETIC_CAMERA_TEST-video-index0'


def sample(raw=b'\xff\xd8fixture\xff\xd9\x00', **changes):
    result = {'jpeg': raw, 'actual_caps': 'image/jpeg,width=1920,height=1080,framerate=30/1',
        'media_type': 'image/jpeg', 'width': 1920, 'height': 1080, 'fps_num': 30, 'fps_den': 1,
        'pts_ns': 42, 'dts_ns': None, 'duration_ns': None, 'offset': None, 'offset_end': None,
        'buffer_flags': 0}
    result.update(changes)
    return result


class Backend:
    def __init__(self, binding, samples, *, start_error=None, stop_error=None, late_error=False, on_pull=None):
        self.binding, self.samples = binding, iter(samples)
        self.start_error, self.stop_error, self.late_error = start_error, stop_error, late_error
        self.on_pull = on_pull
        self.opened_device, self.stopped, self.starts, self.pulls = None, False, 0, 0

    def start(self, timeout_ns):
        self.starts += 1
        if self.start_error:
            raise self.start_error
        self.opened_device = self.binding['device_identity']

    def pull(self, timeout_ns):
        assert 0 < timeout_ns <= 2_000_000_000
        self.pulls += 1
        if self.on_pull:
            self.on_pull()
        value = next(self.samples)
        if isinstance(value, BaseException):
            raise value
        return value

    def check(self):
        pass

    def stop(self):
        self.stopped = True
        if self.stop_error:
            raise self.stop_error
        return True

    def diagnostics(self):
        return {'messages': [{'type': 'ERROR', 'message': 'late synthetic bus error', 'debug': 'fixture'}]
                if self.late_error else [], 'error_seen': self.late_error,
                'pipeline_null_confirmed': self.stopped}


class Clock:
    def __init__(self, step=10_000_000):
        self.now, self.step = 1_000_000_000, step

    def __call__(self):
        self.now += self.step
        return self.now


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.binding = {'by_id': TEST_DEVICE, 'resolved': '/dev/video0',
            'link_identity': {'st_dev': 1, 'st_ino': 2},
            'device_identity': {'st_dev': 3, 'st_ino': 4, 'st_rdev': 5}}

    def tearDown(self):
        self.temp.cleanup()

    def capture(self, samples, *, frames=1, seconds=1, max_bytes=1024, reader=None, **kwargs):
        backend = Backend(self.binding, samples, **kwargs)
        output = self.root/'record'
        result, receipt = tool.record(tool.make_plan(TEST_DEVICE, seconds=seconds, frames=frames, max_bytes=max_bytes), output,
            backend_factory=lambda p, b: backend, binding_reader=reader or (lambda d: self.binding),
            clock=Clock(), boot_reader=lambda: 'synthetic-boot', synthetic=True)
        return result, receipt, backend, output

    def test_plan_does_not_import_gi_open_device_or_create_output(self):
        original = builtins.__import__
        def no_gi(name, *a, **kw):
            if name == 'gi' or name.startswith('gi.'):
                self.fail('PLAN imported GI')
            return original(name, *a, **kw)
        with (patch.object(builtins, '__import__', no_gi),
                patch.object(tool, 'GstCamera', side_effect=AssertionError('device')),
                patch.object(tool, 'private_output', side_effect=AssertionError('output')),
                contextlib.redirect_stdout(io.StringIO()) as out):
            self.assertEqual(tool.main(['--output', str(self.root/'unused')]), 0)
        result = json.loads(out.getvalue())
        self.assertEqual(result['status'], 'PLAN_ONLY')
        self.assertFalse(result['camera_opened'])
        self.assertIsNone(result['device'])
        self.assertEqual(list(self.root.iterdir()), [])

    def test_plan_bounds_and_paths_fail_closed(self):
        for arguments in ({'seconds': float('nan')}, {'seconds': float('inf')}, {'seconds': True},
                {'seconds': .09}, {'seconds': 31}, {'frames': 0}, {'frames': 901}, {'frames': True},
                {'max_bytes': 1023}, {'max_bytes': 512*1024*1024+1},
                {'device': '/dev/video0'}, {'device': TEST_DEVICE+'" ! fakesink'}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                tool.make_plan(**arguments)

    def test_synthetic_requires_explicit_backend_and_boolean_label(self):
        for kwargs in ({'synthetic': True}, {'synthetic': 1},
                       {'backend_factory': lambda p, b: None, 'synthetic': False}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                tool.record(tool.make_plan(TEST_DEVICE), self.root/'unused', **kwargs)
        self.assertFalse((self.root/'unused').exists())

    def test_record_requires_explicit_device_without_hardware_or_output(self):
        with patch.object(tool, 'GstCamera', side_effect=AssertionError('device')), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main(['--record-camera', '--output', str(self.root/'unused')]), 1)
        self.assertIn('explicit stable --device', json.loads(out.getvalue())['error'])
        self.assertFalse((self.root/'unused').exists())

    def test_padded_jpeg_original_bytes_and_metadata_preserved(self):
        raw = sample()['jpeg']
        result, receipt, backend, output = self.capture([sample()])
        self.assertEqual(result['status'], 'SYNTHETIC_CAMERA_FIXTURE_REVIEW_REQUIRED')
        self.assertEqual((output/'frame-000000.jpg').read_bytes(), raw)
        row = result['frames'][0]
        self.assertEqual(row['sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(row['jpeg_marker_observation']['bytes_after_last_eoi'], 1)
        self.assertFalse(result['jpeg_decode_verified'])
        self.assertIsNone(row['exposure_s']); self.assertIsNone(row['timestamp_max_error_s'])
        self.assertEqual(row['pts_ns'], 42); self.assertIsNone(row['dts_ns'])
        self.assertFalse(result['camera_opened']); self.assertTrue(result['synthetic_fixture'])
        self.assertTrue(result['pipeline_null_confirmed']); self.assertTrue(backend.stopped)
        self.assertTrue(result['saved_frames_hash_verified'])
        self.assertEqual(json.loads((output/'report.json').read_text()), result)
        self.assertEqual(receipt['report_sha256'], hashlib.sha256((output/'report.json').read_bytes()).hexdigest())
        for path in output.iterdir():
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(output.stat().st_mode & 0o777, 0o700)

    def test_caps_mismatch_and_bad_jpeg_are_retained_as_failures(self):
        for value in (sample(width=640), sample(raw=b'not a JPEG')):
            with self.subTest(value=value):
                result, _, backend, output = self.capture([value])
                self.assertEqual(result['status'], 'INCOMPLETE')
                self.assertEqual((output/'frame-000000.jpg').read_bytes(), value['jpeg'])
                self.assertTrue(backend.stopped); self.assertTrue(result['errors'])
                for p in output.iterdir():
                    p.unlink()
                output.rmdir()

    def test_started_pipeline_failure_has_diagnostics_and_null_cleanup(self):
        result, _, backend, output = self.capture([], start_error=RuntimeError('PLAYING failed: device busy, Gst debug fixture'))
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('device busy', result['errors'][0]); self.assertTrue(backend.stopped)
        self.assertTrue(result['pipeline_null_confirmed']); self.assertEqual(result['frames'], [])
        self.assertTrue((output/'report.json').is_file())

    def test_partial_interrupt_preserves_frame_and_stops(self):
        result, _, backend, output = self.capture([sample(), KeyboardInterrupt()], frames=2)
        self.assertEqual(result['status'], 'INTERRUPTED_PARTIAL')
        self.assertEqual(len(result['frames']), 1); self.assertTrue(backend.stopped)
        self.assertTrue((output/'frame-000000.jpg').is_file())
        self.assertFalse(result['physical_stationarity_proven'])

    def test_timeout_or_eos_never_retries(self):
        for failure in (TimeoutError('fixture timeout'), EOFError('fixture EOS')):
            with self.subTest(failure=failure):
                result, _, backend, output = self.capture([failure, sample()])
                self.assertEqual(result['status'], 'INCOMPLETE')
                self.assertEqual(backend.pulls, 1); self.assertTrue(backend.stopped)
                self.assertFalse(result['automatic_retry'])
                for p in output.iterdir():
                    p.unlink()
                output.rmdir()

    def test_byte_limit_preserves_prior_frame_and_hashes_unretained_sample(self):
        raw = b'\xff\xd8'+b'x'*596+b'\xff\xd9'
        result, _, backend, output = self.capture([sample(raw=raw), sample(raw=raw)], frames=2)
        self.assertEqual(result['termination_reason'], 'BYTE_LIMIT')
        self.assertEqual(result['status'], 'INCOMPLETE'); self.assertEqual(len(result['frames']), 1)
        self.assertEqual(result['jpeg_bytes_saved'], 600)
        self.assertEqual(result['unretained_sample']['sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(len(list(output.glob('*.jpg'))), 1); self.assertTrue(backend.stopped)

    def test_late_gst_error_and_cleanup_failure_block_success(self):
        for kwargs in ({'late_error': True}, {'stop_error': RuntimeError('NULL failed fixture')}):
            with self.subTest(kwargs=kwargs):
                result, _, backend, output = self.capture([sample()], **kwargs)
                self.assertEqual(result['status'], 'INCOMPLETE')
                self.assertTrue(result['errors']); self.assertTrue(backend.stopped)
                for p in output.iterdir():
                    p.unlink()
                output.rmdir()

    def test_changed_device_binding_blocks_capture_before_playing(self):
        calls = []
        def reader(device):
            calls.append(device)
            return self.binding if len(calls) == 1 else dict(self.binding, resolved='/dev/video1')
        result, _, backend, _ = self.capture([sample()], reader=reader)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertEqual(backend.starts, 0); self.assertTrue(backend.stopped)

    def test_output_existing_symlink_git_and_missing_parent_refused(self):
        existing = self.root/'existing'; existing.mkdir(); (existing/'keep').write_text('unchanged')
        link = self.root/'link'; link.symlink_to(existing, target_is_directory=True)
        for path in (existing, link, self.root/'absent'/'output'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                tool.private_output(path)
        (self.root/'.git').mkdir()
        with self.assertRaises(ValueError): tool.private_output(self.root/'git-output')
        self.assertEqual((existing/'keep').read_text(), 'unchanged')

    def test_output_directory_swap_does_not_follow_symlink(self):
        output, renamed, other = self.root/'record', self.root/'renamed', self.root/'other'
        other.mkdir()
        def swap():
            output.rename(renamed); output.symlink_to(other, target_is_directory=True)
        result, _, backend, _ = self.capture([sample()], on_pull=swap)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertFalse(result['named_output_binding_valid'])
        self.assertEqual(list(other.iterdir()), [])
        self.assertTrue((renamed/'report.json').is_file()); self.assertTrue(backend.stopped)

    def test_saved_frame_mutation_is_reported_without_losing_failure_receipt(self):
        backend = Backend(self.binding, [sample()])
        output = self.root/'record'
        def check():
            (output/'frame-000000.jpg').write_bytes(b'changed')
        backend.check = check
        result, _ = tool.record(tool.make_plan(TEST_DEVICE, frames=1), output, backend_factory=lambda p, b: backend,
            binding_reader=lambda d: self.binding, clock=Clock(), boot_reader=lambda: 'fixture', synthetic=True)
        self.assertEqual(result['status'], 'INCOMPLETE'); self.assertFalse(result['saved_frames_hash_verified'])
        self.assertTrue((output/'report.json').is_file()); self.assertTrue(backend.stopped)

    def test_sample_metadata_rejects_boolean_and_sentinel_claims(self):
        for value in (sample(pts_ns=True), sample(offset=2**64-1), sample(width=True), sample(jpeg=b'')):
            with self.subTest(value=value), self.assertRaises(ValueError): tool.validate_sample(value)

    def test_duration_endpoint_timeout_is_not_a_stall_or_retry(self):
        clock = Clock(step=1_000_000)
        backend = Backend(self.binding, [sample(), TimeoutError('natural end')])
        def advance_at_last_pull():
            if backend.pulls == 2:
                clock.now += 200_000_000
        backend.on_pull = advance_at_last_pull
        result, _ = tool.record(tool.make_plan(TEST_DEVICE, seconds=.1, frames=3), self.root/'boundary',
            backend_factory=lambda p, b: backend, binding_reader=lambda d: self.binding,
            clock=clock, boot_reader=lambda: 'fixture', synthetic=True)
        self.assertEqual(result['status'], 'SYNTHETIC_CAMERA_FIXTURE_REVIEW_REQUIRED')
        self.assertEqual(result['termination_reason'], 'DURATION_LIMIT')
        self.assertEqual(backend.pulls, 2); self.assertEqual(len(result['frames']), 1)
        self.assertEqual(result['terminal_pull']['result'], 'NO_SAMPLE_TIMEOUT')
        self.assertTrue(result['pipeline_null_confirmed']); self.assertEqual(result['errors'], [])

    def test_duration_endpoint_does_not_conceal_gst_error(self):
        clock = Clock(step=1_000_000)
        backend = Backend(self.binding, [sample(), TimeoutError('natural end')])
        backend.on_pull = lambda: setattr(clock, 'now', clock.now+200_000_000) if backend.pulls == 2 else None
        def check():
            if backend.pulls >= 2:
                raise RuntimeError('Gst ERROR boundary fixture')
        backend.check = check
        result, _ = tool.record(tool.make_plan(TEST_DEVICE, seconds=.1, frames=3), self.root/'boundary',
            backend_factory=lambda p, b: backend, binding_reader=lambda d: self.binding,
            clock=clock, boot_reader=lambda: 'fixture', synthetic=True)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('Gst ERROR', result['errors'][0]); self.assertTrue(backend.stopped)

    def test_ready_event_requires_first_saved_valid_frame(self):
        backend = Backend(self.binding, [sample()]); events = []
        result, _ = tool.record(tool.make_plan(TEST_DEVICE, frames=1), self.root/'ready',
            backend_factory=lambda p, b: backend, binding_reader=lambda d: self.binding,
            clock=Clock(), boot_reader=lambda: 'fixture', synthetic=True, ready_callback=events.append)
        self.assertEqual(len(events), 1); self.assertEqual(events[0]['event'], 'CAMERA_READY')
        self.assertEqual(events[0]['first_frame']['sha256'], result['frames'][0]['sha256'])
        self.assertEqual(events[0]['capture_backend'], 'SYNTHETIC_TEST_INJECTION')
        self.assertFalse(events[0]['clock_alignment_verified'])

    def test_source_refusal_precedes_output_creation(self):
        with patch.object(tool, 'regular_source', side_effect=ValueError('changed source')), self.assertRaises(ValueError):
            tool.record(tool.make_plan(TEST_DEVICE), self.root/'unused',
                backend_factory=lambda p, b: None, synthetic=True)
        self.assertFalse((self.root/'unused').exists())

    def test_gst_bus_error_keeps_precise_debug_and_cleanup_confirmation(self):
        backend = tool.GstCamera.__new__(tool.GstCamera)
        backend.Gst = SimpleNamespace(MessageType=SimpleNamespace(ERROR=1, WARNING=2, EOS=4),
            State=SimpleNamespace(NULL=0), StateChangeReturn=SimpleNamespace(FAILURE=-1))
        message = SimpleNamespace(type=1, src=SimpleNamespace(get_name=lambda: 'camera'),
                                  parse_error=lambda: ('device busy fixture', 'v4l2 debug fixture'))
        messages = iter([message, None, None])
        backend.bus = SimpleNamespace(pop_filtered=lambda kinds: next(messages))
        backend.messages, backend.messages_truncated, backend.error_seen = [], False, False
        backend.opened_device, backend.null_confirmed = None, False
        backend.pipeline = SimpleNamespace(set_state=lambda state: 1, get_state=lambda wait: (1, 0, 0))
        with self.assertRaisesRegex(RuntimeError, 'v4l2 debug fixture'):
            backend.messages_read()
        self.assertEqual(backend.messages[0]['source'], 'camera')
        self.assertEqual(backend.messages[0]['message'], 'device busy fixture')
        self.assertTrue(backend.stop()); self.assertTrue(backend.diagnostics()['error_seen'])
        self.assertTrue(backend.diagnostics()['pipeline_null_confirmed'])


if __name__ == '__main__':
    unittest.main()
