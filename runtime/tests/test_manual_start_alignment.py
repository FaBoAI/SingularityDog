"""Fake serial and clock only; no device/network access."""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import signal
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import manual_start_alignment as alignment
from singularitydog_hw import can_readonly as ro

EXPECTED = {str(i): format(i, '016x') for i in range(1, 13)}
BOOT = '12345678-1234-1234-1234-123456789abc'


def reference():
    return {'schema': 'singularitydog.private-matched-start-reference.v1',
        'leg': 'RR', 'ids': [7, 8, 9], 'boot_id': BOOT,
        'ports': {'front': '/dev/serial/by-path/front', 'rear': '/dev/serial/by-path/rear'},
        'expected_uids': dict(EXPECTED), 'expected_uids_file_sha256': 'a'*64,
        'positions_rad': {'7': .7, '8': .8, '9': .9},
        'absolute_targets_rad': {'7': .7, '8': .8, '9': .9-math.radians(4)},
        'source_before_positions': {str(i): {'raw_position_rad': i/10.,
            'request_monotonic_ns': 1, 'received_monotonic_ns': 2} for i in (7, 8, 9)},
        'source': {'stage': 'before', 'raw_readonly_pairs_verified': 492,
                   'summary_sha256': 'b'*64, 'events_sha256': 'c'*64, 'wrapper_sha256': 'd'*64},
        'output_allowed': False, 'runtime_approved': False, 'calibration_modified': False,
        'angle_wrapping_applied': False, 'old_r2_AB_equivalent': False}


class Clock:
    def __init__(self): self.ns = 10_000_000_000
    def __call__(self): return self.ns
    def sleep(self, seconds): self.ns += round(seconds*1e9)


class FakeSerial:
    def __init__(self, clock, *, failure=None, moving=False):
        self.clock, self.failure, self.moving = clock, failure, moving
        self.port = None
        self.queue = bytearray()
        self.writes = []
        self.closed = self.opened = False
        self.timeout = self.write_timeout = None
    def open(self):
        self.opened = True
        if self.failure == 'open': raise OSError('partial open')
    def close(self):
        if self.failure == 'close': raise OSError('close failed')
        self.closed = True
    @property
    def in_waiting(self): return len(self.queue)
    def read(self, count):
        self.clock.ns += 1_000_000
        result = bytes(self.queue[:count]); del self.queue[:count]
        return result
    def write(self, wire):
        frame = ro.ATParser().feed(wire)[0]
        assert frame.kind in (0, 17) and frame.destination in (7, 8, 9)
        if frame.kind == 17: assert int.from_bytes(frame.data[:2], 'little') == 0x7019
        self.writes.append(wire)
        if self.failure == 'partial': return 8
        if self.failure == 'timeout': return len(wire)
        mid, kind = frame.destination, frame.kind
        if kind == 0:
            payload = bytes.fromhex('f'*16 if self.failure == 'uid' else EXPECTED[str(mid)])
        else:
            value = mid/10. + (.1*math.sin(len(self.writes)) if self.moving else 0)
            if self.failure == 'nan': value = float('nan')
            payload = bytes.fromhex('19700000') + struct.pack('<f', value)
        dest = 254 if kind == 0 else 253
        if self.failure == 'wrong_source': mid = 12
        if self.failure == 'unsolicited': kind = 2
        cid = (kind << 24) | (mid << 8) | dest
        response = b'AT'+((cid << 3)|4).to_bytes(4, 'big')+b'\x08'+payload+b'\r\n'
        self.queue.extend(response)
        if self.failure == 'duplicate': self.queue.extend(response)
        if self.failure == 'residual': self.queue.extend(b'A')
        return len(wire)


class AlignmentTests(unittest.TestCase):
    def run_observation(self, *, seconds=1, failure=None, moving=False, external=None, sink=None, display=None):
        self.clock = Clock(); self.raw = FakeSerial(self.clock, failure=failure, moving=moving)
        self.events, self.shown, self.report = [], [], {'errors': []}
        return alignment.observe(reference(), EXPECTED, {'rear': {'path': '/fake/rear'}}, seconds,
            sink or self.events.append, external or (lambda: None), self.report,
            factory=lambda: self.raw, clock=self.clock, wait=self.clock.sleep,
            fd_check=lambda *a: None, display=display or self.shown.append)

    def test_one_second_exact_five_new_samples_and_no_auto_action(self):
        self.run_observation()
        self.assertEqual(self.report['samples_completed'], 5)
        self.assertEqual(self.report['tx_count'], 18)
        self.assertTrue(self.raw.closed)
        self.assertEqual([r['sample_index'] for r in self.shown], list(range(5)))
        for row in self.shown:
            for flag in alignment.FALSE_FLAGS: self.assertFalse(row[flag])
            self.assertTrue(row['all_three_within_candidate_tolerance'])
            self.assertGreater(row['acquisition_span_ns'], 0)

    def test_maximum_120_seconds_request_bound_and_movement_allowed(self):
        self.run_observation(seconds=120, moving=True)
        self.assertEqual(self.report['tx_count'], 1803)
        self.assertEqual(self.report['samples_completed'], 600)
        self.assertLess(self.report['acquisition_finished_ns']-self.report['acquisition_started_ns'], 120_000_000_000)
        self.assertTrue(any(not r['all_three_within_candidate_tolerance'] for r in self.shown))
        self.assertTrue(self.raw.closed)

    def test_timeout_partial_uid_duplicate_residual_wrong_frame_all_close_no_retry(self):
        for failure in ('timeout', 'partial', 'uid', 'duplicate', 'residual', 'wrong_source', 'unsolicited'):
            with self.subTest(failure=failure):
                with self.assertRaises((RuntimeError, TimeoutError, OSError)):
                    self.run_observation(failure=failure)
                self.assertEqual(len(self.raw.writes), 1)
                self.assertEqual(self.shown, [])
                self.assertTrue(self.raw.closed)

    def test_nonfinite_position_never_displayed(self):
        with self.assertRaises(RuntimeError): self.run_observation(failure='nan')
        self.assertEqual(len(self.raw.writes), 4)
        self.assertEqual(self.shown, [])
        self.assertTrue(self.raw.closed)

    def test_partial_open_and_close_failure_recorded(self):
        with self.assertRaisesRegex(OSError, 'partial open'): self.run_observation(failure='open')
        self.assertTrue(self.raw.closed)
        self.assertEqual(self.raw.writes, [])
        with self.assertRaisesRegex(OSError, 'close failed'): self.run_observation(failure='close')
        self.assertFalse(self.report['serial_closed'])

    def test_boot_fd_signal_or_logging_error_stops_and_closes(self):
        count = [0]
        def check():
            count[0] += 1
            if count[0] > 10: raise RuntimeError('boot changed')
        with self.assertRaisesRegex(RuntimeError, 'boot changed'): self.run_observation(external=check)
        self.assertTrue(self.raw.closed)
        self.assertLess(len(self.raw.writes), 18)
        def sink(event): raise OSError('log failed')
        with self.assertRaisesRegex(OSError, 'log failed'): self.run_observation(sink=sink)
        self.assertTrue(self.raw.closed)
        self.assertEqual(self.raw.writes, [])

    def test_slow_display_fails_without_catchup_or_previous_sample_reuse(self):
        def slow(row): self.clock.ns += 310_000_000
        with self.assertRaisesRegex(RuntimeError, 'loop late'): self.run_observation(seconds=2, display=slow)
        self.assertEqual(self.report['samples_completed'], 1)
        self.assertEqual(len(self.raw.writes), 6)
        self.assertTrue(self.raw.closed)

    def test_slow_sample_log_does_not_display_stale_positions(self):
        def slow_log(event):
            if event['kind'] == 'manual_alignment_sample': self.clock.ns += 101_000_000
        with self.assertRaisesRegex(RuntimeError, 'stale before display'):
            self.run_observation(sink=slow_log)
        self.assertEqual(self.shown, [])
        self.assertEqual(self.report['samples_completed'], 0)
        self.assertTrue(self.raw.closed)

    def test_byte_boundary_blocks_other_ids_parameters_control_and_repeated_write(self):
        clock = Clock(); raw = FakeSerial(clock)
        proxy = alignment.GuardedSerial(raw, lambda: None, clock, clock()+1_000_000_000, lambda _: None, 18)
        for bad in (ro.read_request(1), ro.read_request(7, 'velocity'), bytes(17)):
            proxy.expected_wire = bad
            with self.assertRaisesRegex(RuntimeError, 'Write blocked'): proxy.write(bad)
        good = ro.read_request(7)
        proxy.expected_wire = good
        self.assertEqual(proxy.write(good), len(good))
        with self.assertRaises(RuntimeError): proxy.write(good)
        self.assertEqual(len(raw.writes), 1)

    def test_remaining_budget_bounds_serial_blocking_timeouts(self):
        clock = Clock(); raw = FakeSerial(clock)
        proxy = alignment.GuardedSerial(raw, lambda: None, clock, clock()+15_000_000, lambda _: None, 18)
        proxy.expected_wire = ro.read_request(7)
        proxy.write(proxy.expected_wire)
        self.assertLessEqual(raw.write_timeout, .015)
        proxy.read(17)
        self.assertLessEqual(raw.timeout, .003)
        clock.ns += 20_000_000
        with self.assertRaisesRegex(RuntimeError, 'deadline'): proxy.read(1)

    def test_slow_write_log_rechecks_budget_before_actual_io(self):
        clock = Clock(); raw = FakeSerial(clock)
        def emit(event): clock.ns += 30_000_000
        proxy = alignment.GuardedSerial(raw, lambda: None, clock, clock()+15_000_000, emit, 18)
        proxy.expected_wire = ro.read_request(7)
        with self.assertRaisesRegex(RuntimeError, 'deadline'): proxy.write(proxy.expected_wire)
        self.assertEqual(raw.writes, [])
        self.assertEqual(proxy.write_count, 0)

    def test_display_raw_no_wrap_no_up_down_and_tolerance_endpoints(self):
        ref = reference()
        readings = {i: {'value': ref['positions_rad'][str(i)]+alignment.TOLERANCE_RAD,
                        'request_monotonic_ns': 1, 'monotonic_ns': 2} for i in alignment.IDS}
        row = alignment.alignment_row(ref, readings, 0, 3)
        self.assertTrue(row['all_three_within_candidate_tolerance'])
        readings[7]['value'] = math.nextafter(readings[7]['value'], math.inf)
        self.assertFalse(alignment.alignment_row(ref, readings, 0, 3)['all_three_within_candidate_tolerance'])
        readings[7]['value'] = ref['positions_rad']['7']+2*math.pi
        row = alignment.alignment_row(ref, readings, 0, 3)
        self.assertAlmostEqual(row['joints']['7']['raw_delta_deg'], 360)
        with contextlib.redirect_stdout(io.StringIO()) as out: alignment.display_row(row)
        self.assertIn('+360.00', out.getvalue())
        self.assertNotIn('上げ', out.getvalue()); self.assertNotIn('下げ', out.getvalue())
        with self.assertRaises(RuntimeError): alignment.alignment_row(ref, readings, 0, alignment.MAX_AGE_NS+2)


class InputAndLifecycleTests(unittest.TestCase):
    def test_reference_schema_uid_boot_flags_and_targets(self):
        self.assertEqual(alignment.validate_reference(reference(), EXPECTED, 'a'*64)['leg'], 'RR')
        for alter in (lambda r: r.update(boot_id='bad'), lambda r: r.update(output_allowed=True),
                      lambda r: r['expected_uids'].update({'9': 'f'*16}),
                      lambda r: r['absolute_targets_rad'].update({'7': .71}),
                      lambda r: r['source'].update(events_sha256='bad')):
            ref = reference(); alter(ref)
            with self.assertRaises((RuntimeError, ValueError)): alignment.validate_reference(ref, EXPECTED, 'a'*64)

    def test_input_hash_pins_and_no_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            uid, ref = Path(tmp)/'uid.json', Path(tmp)/'ref.json'
            uid.write_text(json.dumps(EXPECTED))
            value = reference(); value['expected_uids_file_sha256'] = alignment.sha(uid)
            ref.write_text(json.dumps(value))
            self.assertEqual(alignment.load_inputs(ref, alignment.sha(ref), uid, alignment.sha(uid))[1], EXPECTED)
            with self.assertRaisesRegex(RuntimeError, 'hash mismatch'):
                alignment.load_inputs(ref, 'f'*64, uid, alignment.sha(uid))
            link = Path(tmp)/'link'; link.symlink_to(ref)
            with self.assertRaises(RuntimeError): alignment.load_inputs(link, alignment.sha(ref), uid, alignment.sha(uid))

    def test_plan_without_input_files_or_device_creation_and_explicit_declaration(self):
        args = ['--reference', '/nonexistent/ref', '--reference-sha256', 'x',
                '--expected-uids', '/nonexistent/uid', '--expected-uids-sha256', 'x']
        with patch.object(alignment, 'load_inputs', side_effect=AssertionError('IO')), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(alignment.main(args), 0)
        self.assertEqual(json.loads(out.getvalue())['max_requests'], 1353)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            alignment.main(args+['--execute-readonly'])
        for bad in (0, 121, True, 1.5):
            with self.assertRaises(RuntimeError): alignment.make_plan(bad)

    def test_real_locks_release_on_closed_port_and_retain_on_close_failure(self):
        import fcntl
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            bindings = {b: {'resolved': str(root/b)} for b in ('front', 'rear')}
            report = {'errors': [], 'serial_closed': False}
            try:
                with patch.object(alignment.Path, 'home', return_value=root):
                    with alignment.ownership(bindings, report): pass
                self.assertFalse(report['locks_released'])
                with (root/'.cache/singularitydog/manual-calibration.lock').open('a+') as handle:
                    with self.assertRaises(BlockingIOError): fcntl.flock(handle, fcntl.LOCK_EX|fcntl.LOCK_NB)
            finally:
                while alignment._RETAINED_LOCKS: alignment._RETAINED_LOCKS.pop().close()
            report = {'errors': [], 'serial_closed': True}
            with patch.object(alignment.Path, 'home', return_value=root):
                with alignment.ownership(bindings, report): pass
            self.assertTrue(report['locks_released'])
            for b in bindings.values():
                name = hashlib.sha256(b['resolved'].encode()).hexdigest()[:24]
                Path('/tmp/singularitydog-can-port-'+name+'.lock').unlink()


class MainLifecycleTests(unittest.TestCase):
    def run_main(self, scenario):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            output = root/'capture'
            bindings = {b: {'path': '/fake/'+b, 'resolved': str(root/b), 'st_rdev': n}
                        for n, b in enumerate(('front', 'rear'), 1)}
            args = ['--execute-readonly', '--operator-confirmed-relaxed-supported', '--seconds', '1',
                    '--reference', str(root/'ref'), '--reference-sha256', 'b'*64,
                    '--expected-uids', str(root/'uids'), '--expected-uids-sha256', 'a'*64,
                    '--output', str(output)]
            saved_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
            clock = Clock(); raw = FakeSerial(clock)
            if scenario == 'signal':
                original_write = raw.write
                def write(wire):
                    result = original_write(wire)
                    signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                    return result
                raw.write = write
            if scenario == 'close': raw.failure = 'close'
            if scenario == 'signal_close':
                original_close = raw.close
                def close():
                    original_close()
                    signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                raw.close = close
            original_observe = alignment.observe
            def observe(*a, **k):
                return original_observe(*a, **k, factory=lambda: raw, clock=clock,
                    wait=clock.sleep, fd_check=lambda *_: None, display=lambda _: None)
            original_read = Path.read_text
            def read(path, *a, **k):
                if str(path) == '/proc/sys/kernel/random/boot_id':
                    return 'changed' if scenario == 'boot' else BOOT
                return original_read(path, *a, **k)
            try:
                with patch.object(alignment, 'load_inputs', return_value=(reference(), EXPECTED)), \
                        patch.object(alignment, 'validate_bindings', return_value=bindings), \
                        patch.object(alignment, 'check_binding'), patch.object(alignment, 'observe', side_effect=observe), \
                        patch.object(Path, 'home', return_value=root), patch.object(Path, 'read_text', new=read), \
                        contextlib.redirect_stdout(io.StringIO()):
                    code = alignment.main(args)
                report = json.loads((output/'summary.json').read_text())
                self.assertTrue(report['events_flush_confirmed'])
                self.assertEqual(report['events_sha256'], alignment.sha(output/'events.jsonl'))
                self.assertEqual({s: signal.getsignal(s) for s in saved_handlers}, saved_handlers)
                self.assertFalse(report['output_allowed'] or report['disabled_mode_verified'])
                return code, report, raw
            finally:
                while alignment._RETAINED_LOCKS: alignment._RETAINED_LOCKS.pop().close()
                for b in bindings.values():
                    name = hashlib.sha256(b['resolved'].encode()).hexdigest()[:24]
                    Path('/tmp/singularitydog-can-port-'+name+'.lock').unlink(missing_ok=True)

    def test_complete_main_closes_logs_ports_locks_and_restores_handlers(self):
        code, report, raw = self.run_main('normal')
        self.assertEqual(code, 0)
        self.assertEqual(report['samples_completed'], 5)
        self.assertTrue(raw.closed and report['locks_released'])

    def test_boot_mismatch_stops_before_port_creation(self):
        code, report, raw = self.run_main('boot')
        self.assertEqual(code, 2)
        self.assertFalse(raw.opened)
        self.assertIn('Boot changed', str(report['errors']))

    def test_signal_stops_without_retry_and_saves_incomplete(self):
        code, report, raw = self.run_main('signal')
        self.assertEqual(code, 2)
        self.assertEqual(len(raw.writes), 1)
        self.assertTrue(raw.closed and report['locks_released'])
        self.assertEqual(report['signals'], [signal.SIGTERM])

    def test_close_failure_saves_incomplete_and_retains_ownership(self):
        code, report, raw = self.run_main('close')
        self.assertEqual(code, 2)
        self.assertFalse(report['serial_closed'] or report['locks_released'])
        self.assertIn('locks retained', str(report['errors']))

    def test_signal_during_cleanup_cannot_report_completion(self):
        code, report, raw = self.run_main('signal_close')
        self.assertEqual(code, 2)
        self.assertTrue(raw.closed and report['locks_released'])
        self.assertEqual(report['status'], 'INCOMPLETE')


if __name__ == '__main__': unittest.main()
