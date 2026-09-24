"""Offline pipeline tests with fake serial arrival times and synthetic identities."""
from contextlib import redirect_stdout, redirect_stderr
import io
import json
import math
from pathlib import Path
import signal
import stat
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.can_pipeline_probe import PipelineCAN, main, make_plan

UIDS = {i: f'{i:016x}' for i in range(1, 13)}


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, ns): self.now += ns


def wire(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+bytes([len(data)])+data+b'\r\n'


class Serial:
    def __init__(self, clock, *, failure=None, delay_ns=3_000_000, reordered=False):
        self.clock, self.failure, self.delay_ns, self.reordered = clock, failure, delay_ns, reordered
        self.queue, self.buffer, self.writes, self.write_times = [], bytearray(), [], []
        self.closed = False
        self._timeout, self._write_timeout = .003, .1
        self.read_timeouts, self.write_timeouts = [], []
        self.timeout_settings, self.write_timeout_settings = [], []

    @property
    def timeout(self): return self._timeout

    @timeout.setter
    def timeout(self, value):
        self.timeout_settings.append(value)
        self._timeout = value

    @property
    def write_timeout(self): return self._write_timeout

    @write_timeout.setter
    def write_timeout(self, value):
        self.write_timeout_settings.append(value)
        self._write_timeout = value

    def fill(self):
        while self.queue and self.queue[0][0] <= self.clock():
            _, data = self.queue.pop(0)
            self.buffer.extend(data)

    @property
    def in_waiting(self):
        self.fill()
        return len(self.buffer)

    def close(self): self.closed = True

    def write(self, data):
        frame = ATParser().feed(data)[0]
        self.writes.append(frame)
        self.write_times.append(self.clock())
        self.write_timeouts.append(self.write_timeout)
        index = len(self.writes)
        if self.failure == 'shortwrite' and index == 13: return len(data)-1
        if self.failure == 'writeerror' and index == 13: raise IOError('simulated write error')
        if self.failure == 'timeout' and index == 13: return len(data)
        if self.failure == 'slowwrite' and index == 13:
            self.clock.advance(300_000_000)
            return len(data)
        mid = frame.destination
        delay = self.delay_ns
        if frame.kind == 0:
            uid = 'f'*16 if self.failure == 'uid' and mid == 2 else UIDS[mid]
            answer = wire((mid << 8)|0xFE, bytes.fromhex(uid))
        else:
            parameter = int.from_bytes(frame.data[:2], 'little')
            value = math.nan if self.failure == 'nan' and index == 13 else mid/10.
            status = 1 if self.failure == 'status' and index == 13 else 0
            if self.failure == 'wrongid' and index == 13: mid = 12
            if self.failure == 'wrongparam' and index == 13: parameter = 0x701C
            if self.reordered:
                delay += (3-index % 3)*1_000_000
            answer = wire((17 << 24)|(status << 16)|(mid << 8)|0xFD,
                          struct.pack('<H2xf', parameter, value))
        if index == 13:
            if self.failure == 'duplicate': answer += answer
            if self.failure == 'noise': answer = b'noise'+answer
            if self.failure == 'unsolicited': answer = wire((2 << 24)|(mid << 8)|0xFD, bytes(8))+answer
            if self.failure == 'reserved':
                payload = struct.pack('<HHf', 0x7019, 1, value)
                answer = wire((17 << 24)|(mid << 8)|0xFD, payload)
        if index == 36:
            if self.failure == 'boundary_partial': answer += b'A'
            if self.failure == 'lastduplicate': answer += answer
        if self.failure == 'crosscycle' and index == 37:
            # Old cycle's ID12 velocity is not pending in the next cycle.
            answer = wire((17 << 24)|(12 << 8)|0xFD, struct.pack('<H2xf', 0x701B, 0.))+answer
        self.queue.append((self.clock()+delay, answer))
        self.queue.sort(key=lambda item: item[0])
        return len(data)

    def read(self, n):
        self.read_timeouts.append(self.timeout)
        self.fill()
        if not self.buffer:
            wait = max(0, int(self.timeout*1e9))
            if self.queue:
                wait = min(wait, max(0, self.queue[0][0]-self.clock()))
            self.clock.advance(wait)
            self.fill()
        chunk = bytes(self.buffer[:n])
        del self.buffer[:n]
        return chunk


def fixture(*, failure=None, window=4, gap_ms=0., cycles=2, seconds=10.,
            delay_ns=3_000_000, reordered=False, emit=None, request_order='interleaved'):
    clock = Clock()
    serial = Serial(clock, failure=failure, delay_ns=delay_ns, reordered=reordered)
    events = []
    probe = PipelineCAN(emit or events.append, window=window, gap_ms=gap_ms,
                        cycles=cycles, max_seconds=seconds, serial_port=serial, clock=clock,
                        request_order=request_order)
    return probe, serial, events, clock


class PipelineTests(unittest.TestCase):
    def run_probe(self, **kwargs):
        probe, serial, events, clock = fixture(**kwargs)
        with probe: report = probe.collect(UIDS)
        return probe, serial, events, clock, report

    def test_identity_sequential_then_window_concurrent_exact_count(self):
        probe, serial, events, clock, report = self.run_probe()
        self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
        self.assertTrue(serial.closed)
        self.assertEqual(report['write_attempts'], 60)
        self.assertEqual(report['replies'], 60)
        self.assertEqual(report['max_observed_pending'], 4)
        self.assertTrue(report['identities_verified'])
        self.assertEqual([f.kind for f in serial.writes], [0]*12+[17]*48)
        self.assertEqual(serial.write_times[:12], [1_000_000_000+i*3_000_000 for i in range(12)])
        self.assertEqual(report['statistics_ms']['parameter_wire_RTT']['median'], 3.)
        self.assertEqual(report['statistics_ms']['cycle_duration']['max'], 18.)
        self.assertEqual(report['cycles_completed'], 2)
        self.assertTrue(all(not any(c['residual'].values()) for c in report['cycles']))
        self.assertTrue(all(r['write_started_monotonic_ns'] <= r['write_finished_monotonic_ns'] <= r['received_monotonic_ns'] for r in report['requests']))

    def test_reordered_responses_match_id_and_parameter_not_send_order(self):
        _, _, _, _, report = self.run_probe(window=12, reordered=True)
        self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
        times = [r['received_monotonic_ns'] for r in report['requests'] if r['cycle'] == 1]
        self.assertNotEqual(times, sorted(times))

    def test_by_parameter_order_keeps_identity_phase_and_matches_reordered_replies(self):
        _, serial, _, _, report = self.run_probe(request_order='by-parameter', reordered=True)
        self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
        self.assertEqual(report['plan']['request_order'], 'by-parameter')
        self.assertEqual([f.destination for f in serial.writes[:12]], list(range(1, 13)))
        self.assertEqual([f.kind for f in serial.writes[:12]], [0]*12)
        expected = [(i, index) for index in (0x7019, 0x701B) for i in range(1, 13)]
        actual = [(f.destination, int.from_bytes(f.data[:2], 'little')) for f in serial.writes[12:]]
        self.assertEqual(actual, expected*2)
        self.assertEqual(report['replies'], 60)

    def test_default_request_order_remains_interleaved(self):
        _, serial, _, _, report = self.run_probe(cycles=1)
        self.assertEqual(report['plan']['request_order'], 'interleaved')
        actual = [(f.destination, int.from_bytes(f.data[:2], 'little')) for f in serial.writes[12:]]
        self.assertEqual(actual, [(i, index) for i in range(1, 13) for index in (0x7019, 0x701B)])

    def test_write_return_diagnostics_distinguish_complete_partial_and_exception(self):
        for failure, returned in ((None, 17), ('shortwrite', 16), ('writeerror', None)):
            with self.subTest(failure=failure):
                _, serial, events, _, report = self.run_probe(failure=failure, cycles=1)
                row = report['requests'][12]
                self.assertEqual(row['write_expected_bytes'], 17)
                self.assertEqual(row['write_returned_bytes'], returned)
                self.assertIs(row['write_call_entered'], True)
                logged = [e for e in events if e['kind']=='pipeline_write_timing' and e['sequence']==13][0]
                self.assertEqual(logged['write_returned_bytes'], returned)
                if failure:
                    self.assertEqual(report['status'], 'INCOMPLETE')
                    self.assertEqual(len(serial.writes), 13)
                else:
                    self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
                    self.assertTrue(all(r['write_returned_bytes']==17 for r in report['requests']))
                self.assertTrue(serial.closed)

    def test_rejection_before_physical_write_is_distinct_from_write_exception(self):
        probe, serial, events, _ = fixture()
        with probe, patch.object(probe, 'validate_wire', side_effect=ValueError('reject before I/O')):
            report = probe.collect(UIDS)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertFalse(serial.writes)
        self.assertIs(report['requests'][0]['write_call_entered'], False)
        self.assertIsNone(report['requests'][0]['write_returned_bytes'])

    def test_gap_is_from_last_write_finish_and_does_not_block_receive(self):
        _, serial, events, _, report = self.run_probe(gap_ms=5., window=12, cycles=1)
        self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
        self.assertTrue(all(b-a >= 5_000_000 for a, b in zip(serial.write_times, serial.write_times[1:])))
        self.assertEqual(report['statistics_ms']['parameter_wire_RTT']['max'], 3.)
        self.assertEqual(report['max_observed_pending'], 1)

    def test_window_one_matches_sequential_budget(self):
        _, _, _, _, report = self.run_probe(window=1)
        self.assertEqual(report['statistics_ms']['cycle_duration']['max'], 72.)
        self.assertEqual(report['max_observed_pending'], 1)

    def test_all_failures_keep_partial_results_without_retry(self):
        for failure in ('timeout', 'uid', 'status', 'nan', 'wrongid', 'wrongparam',
                        'duplicate', 'noise', 'unsolicited', 'reserved', 'shortwrite',
                        'slowwrite', 'boundary_partial', 'lastduplicate', 'crosscycle'):
            with self.subTest(failure=failure):
                _, serial, _, _, report = self.run_probe(failure=failure)
                self.assertEqual(report['status'], 'INCOMPLETE')
                self.assertTrue(report['errors'])
                self.assertLess(len(serial.writes), 60)
                sent = [(r['cycle'], r['motor_id'], r['parameter']) for r in report['requests']]
                self.assertEqual(len(sent), len(set(sent)))
                if failure == 'uid':
                    self.assertEqual(len(serial.writes), 2)
                    self.assertFalse(report['identities_verified'])
                else:
                    self.assertEqual(report['cycles'][-1]['status'], 'INCOMPLETE')

    def test_timeout_and_write_time_are_bounded_by_remaining_global_budget(self):
        _, serial, _, _, report = self.run_probe(seconds=1., delay_ns=200_000_000)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertLessEqual(report['write_attempts'], 5)
        self.assertLessEqual(report['elapsed_s'], 1.000001)
        self.assertTrue(all(0 <= x <= .003 for x in serial.read_timeouts))
        self.assertTrue(all(0 < x <= .1 for x in serial.write_timeouts))

    def test_unchanged_timeout_does_not_reconfigure_port(self):
        _, serial, _, _, report = self.run_probe(gap_ms=.5, window=4)
        self.assertEqual(report['status'], 'READONLY_PIPELINE_COMPLETE')
        self.assertEqual(report['plan']['gap_ms'], .5)
        self.assertEqual(report['plan']['window'], 4)
        self.assertEqual(serial.write_timeout_settings, [])
        self.assertEqual(serial.write_timeouts, [.1]*60)
        self.assertLess(len(serial.timeout_settings), len(serial.read_timeouts))
        settings = [.003, *serial.timeout_settings]
        self.assertTrue(all(a != b for a, b in zip(settings, settings[1:])))
        self.assertTrue(all(0 <= x <= .003 for x in serial.read_timeouts))

    def test_external_timeout_change_is_not_hidden_by_a_cache(self):
        probe, serial, _, _ = fixture()
        with probe:
            probe._send((1, 'position'), 1)
            serial.write_timeout = .2
            serial.timeout = .2
            probe._send((2, 'position'), 1)
            probe._receive(.003, UIDS)
        self.assertEqual(serial.write_timeout_settings, [.2, .1])
        self.assertEqual(serial.timeout_settings, [.2, .003])
        self.assertEqual(serial.write_timeouts, [.1, .1])
        self.assertTrue(serial.closed)

    def test_changed_read_timeout_respects_absolute_request_and_global_deadlines(self):
        for limit in ('request', 'global'):
            with self.subTest(limit=limit):
                probe, serial, _, clock = fixture(seconds=1.)
                with probe:
                    if limit == 'global':
                        clock.advance(999_000_000)
                    probe._send((1, 'position'), 1)
                    if limit == 'request':
                        clock.advance(249_000_000)
                    serial.queue.clear()  # Leave the outstanding request unanswered.
                    deadline = probe.pending[(1, 'position')]['deadline_monotonic_ns']
                    with self.assertRaises(TimeoutError):
                        probe._receive(.003, UIDS)
                    self.assertEqual(clock(), deadline)
                self.assertEqual(serial.timeout_settings, [.001])
                self.assertEqual(serial.read_timeouts, [.001])
                self.assertEqual(serial.write_timeouts, [.001 if limit == 'global' else .1])
                self.assertTrue(serial.closed)

    def test_timeout_reconfiguration_errors_abort_and_close_without_retry(self):
        for name, writes in (('write_timeout', 0), ('timeout', 1)):
            with self.subTest(setting=name):
                probe, serial, _, _ = fixture()
                setattr(serial, name, .2)  # Force one necessary reconfiguration.
                original = getattr(Serial, name)
                def fail_setting(port, value):
                    raise OSError('simulated timeout reconfiguration error')
                with patch.object(Serial, name, property(original.fget, fail_setting)):
                    with probe:
                        report = probe.collect(UIDS)
                self.assertEqual(report['status'], 'INCOMPLETE')
                self.assertIn('simulated timeout reconfiguration error', str(report['errors']))
                self.assertEqual(len(serial.writes), writes)
                self.assertTrue(serial.closed)

    def test_allowlist_is_checked_at_physical_write_and_no_repeat_key(self):
        probe, serial, _, _ = fixture()
        with probe:
            for mid, parameter in ((0, 'position'), (True, 'position'), (13, 'position'), (1, 'voltage'), (1, 'enable')):
                with self.assertRaises(ValueError): probe._send((mid, parameter), 1)
            with self.assertRaises(ValueError): probe.serial.write(read_request(1, 'position'))
            probe.current_write = (1, 'position')
            probe.pending[(1, 'position')] = {'deadline_monotonic_ns': probe.deadline_ns}
            for bad in (read_request(1, 'velocity'), wire((3 << 24)|(0xFD << 8)|1, bytes(8))):
                with self.assertRaises(ValueError): probe.serial.write(bad)
            with self.assertRaises(RuntimeError): probe._send((1, 'position'), 1)
        self.assertFalse(serial.writes)

    def test_interruption_and_initial_residual_open_no_command(self):
        probe, serial, _, _ = fixture()
        serial.buffer.extend(b'AT')
        with probe: report = probe.collect(UIDS)
        self.assertFalse(serial.writes)
        self.assertEqual(report['status'], 'INCOMPLETE')
        probe, serial, _, _ = fixture()
        def interrupt(): raise InterruptedError('test interruption')
        with probe:
            probe.check_interrupt = interrupt
            report = probe.collect(UIDS)
        self.assertFalse(serial.writes)
        self.assertTrue(serial.closed)

    def test_slow_logger_budget_is_rechecked_before_write(self):
        probe, serial, events, clock = fixture(seconds=1.)
        def emit(event):
            events.append(event)
            if event['kind'] == 'pipeline_tx_intent': clock.advance(1_000_000_000)
        probe.emit = emit
        with probe: report = probe.collect(UIDS)
        self.assertFalse(serial.writes)
        self.assertEqual(report['status'], 'INCOMPLETE')

    def test_an_old_pending_deadline_limits_later_write_timeout(self):
        probe, serial, events, clock = fixture(window=2, cycles=1)
        with probe:
            probe._send((1, 'position'), 1)
            clock.advance(245_000_000)
            probe._send((2, 'position'), 1)
        self.assertLessEqual(serial.write_timeouts[-1], .005)
        self.assertEqual(len(serial.writes), 2)


class CLITests(unittest.TestCase):
    def args(self, root):
        uid = root/'uids.json'
        uid.write_text(json.dumps(UIDS))
        return ['--expected-uids', str(uid), '--output', str(root/'capture')]

    def test_dry_run_never_opens_port_locks_or_creates_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch('singularitydog_hw.can_pipeline_probe.PipelineCAN') as can, patch('singularitydog_hw.can_pipeline_probe.ownership_locks') as locks, redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main(self.args(root)), 0)
            can.assert_not_called(); locks.assert_not_called()
            self.assertFalse((root/'capture').exists())
            self.assertNotIn(UIDS[1], out.getvalue())

    def test_ranges_nonfinite_and_bool_rejected(self):
        for kwargs in ({'window': 0}, {'window': 13}, {'window': True}, {'gap_ms': math.nan},
                       {'gap_ms': math.inf}, {'gap_ms': -1}, {'gap_ms': 5.1}, {'gap_ms': True},
                       {'cycles': 0}, {'cycles': 21}, {'cycles': True}, {'max_seconds': 0},
                       {'max_seconds': 31}, {'max_seconds': math.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError): make_plan(**kwargs)

    def test_invalid_request_order_and_explicit_dry_plan(self):
        for value in ('velocity-only', '', None, True, []):
            with self.subTest(value=value), self.assertRaises(ValueError):
                make_plan(request_order=value)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch('singularitydog_hw.can_pipeline_probe.PipelineCAN') as can, redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main([*self.args(root), '--request-order', 'by-parameter']), 0)
            can.assert_not_called()
            self.assertEqual(json.loads(out.getvalue())['request_order'], 'by-parameter')
            self.assertFalse((root/'capture').exists())

    def test_git_output_rejected_before_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args = self.args(root); (root/'.git').mkdir()
            with patch('singularitydog_hw.can_pipeline_probe.PipelineCAN') as can, redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main([*args, '--execute-readonly'])
            can.assert_not_called()

    def test_partial_capture_private_saved_port_closed_and_handlers_restored(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args = self.args(root)
            clock = Clock(); serial = Serial(clock, failure='timeout')
            def factory(emit, **kwargs): return PipelineCAN(emit, **kwargs, serial_port=serial, clock=clock)
            previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
            with patch('singularitydog_hw.can_pipeline_probe.PipelineCAN', side_effect=factory), patch('singularitydog_hw.can_timing_probe.Path.home', return_value=root), redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main([*args, '--execute-readonly', '--window', '4']), 1)
            report = json.loads((root/'capture/summary.json').read_text())
            self.assertEqual(report['status'], 'INCOMPLETE')
            self.assertTrue(serial.closed)
            self.assertEqual(stat.S_IMODE((root/'capture').stat().st_mode), 0o700)
            self.assertEqual(stat.S_IMODE((root/'capture/summary.json').stat().st_mode), 0o600)
            self.assertEqual(stat.S_IMODE((root/'capture/events.jsonl').stat().st_mode), 0o600)
            self.assertNotIn(UIDS[1], out.getvalue())
            self.assertNotIn('wire_hex', out.getvalue())
            self.assertEqual(previous, {sig: signal.getsignal(sig) for sig in previous})

    def test_both_signals_abort_pending_run_and_close_port(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=signum), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp); args = self.args(root)
                clock = Clock(); serial = Serial(clock)
                def factory(emit, **kwargs):
                    def interrupting_emit(event):
                        emit(event)
                        if event['kind'] == 'pipeline_write_timing' and event['sequence'] == 13:
                            signal.getsignal(signum)(signum, None)
                    return PipelineCAN(interrupting_emit, **kwargs, serial_port=serial, clock=clock)
                old = signal.getsignal(signum)
                with patch('singularitydog_hw.can_pipeline_probe.PipelineCAN', side_effect=factory), patch('singularitydog_hw.can_timing_probe.Path.home', return_value=root), redirect_stdout(io.StringIO()):
                    self.assertEqual(main([*args, '--execute-readonly', '--window', '4']), 1)
                report = json.loads((root/'capture/summary.json').read_text())
                self.assertTrue(serial.closed)
                self.assertEqual(len(serial.writes), 13)
                self.assertEqual(report['status'], 'INCOMPLETE')
                self.assertIn('InterruptedError', str(report['errors']))
                self.assertEqual(signal.getsignal(signum), old)


if __name__ == '__main__': unittest.main()
