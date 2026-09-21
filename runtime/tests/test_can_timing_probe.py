"""Offline request/reply timing and failure tests with synthetic motor identities."""
from contextlib import redirect_stdout, redirect_stderr
import io
import json
import math
from pathlib import Path
import stat
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.can_timing_probe import TimingCAN, collect, main, make_plan, ownership_locks

UIDS = {i: f'{i:016x}' for i in range(1, 13)}


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, ns): self.now += ns


def wire(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


class Serial:
    def __init__(self, clock, failure=None, delay_ns=5_000_000):
        self.clock, self.failure, self.delay_ns = clock, failure, delay_ns
        self.buffer, self.writes, self.closed = bytearray(), [], False
    @property
    def in_waiting(self): return len(self.buffer)
    def close(self): self.closed = True
    def write(self, data):
        if self.buffer: raise AssertionError('A second request preceded the first response')
        frame = ATParser().feed(data)[0]
        self.writes.append(frame)
        index = len(self.writes)
        mid = frame.destination
        if self.failure == 'timeout' and index == 13: return len(data)
        if self.failure == 'shortwrite' and index == 13: return len(data)-1
        if frame.kind == 0:
            payload = bytes.fromhex('f'*16 if self.failure == 'uid' and mid == 2 else UIDS[mid])
            answer = wire((mid << 8)|0xFE, payload)
        else:
            parameter = int.from_bytes(frame.data[:2], 'little')
            val = math.nan if self.failure == 'nan' and index == 13 else mid/10.
            status = 1 if self.failure == 'status' and index == 13 else 0
            if self.failure == 'wrongid' and index == 13: mid = 2
            answer = wire((17 << 24)|(status << 16)|(mid << 8)|0xFD,
                          struct.pack('<H2xf', parameter, val))
        if index == 13:
            if self.failure == 'duplicate': answer += answer
            if self.failure == 'noise': answer = b'noise'+answer
            if self.failure == 'partial': answer += b'A'
            if self.failure == 'unsolicited': answer = wire((2 << 24)|(mid << 8)|0xFD, bytes(8))+answer
        self.buffer.extend(answer)
        return len(data)
    def read(self, n):
        self.clock.advance(self.delay_ns)
        data = bytes(self.buffer[:n]); del self.buffer[:n]
        return data


def fixture(failure=None, cycles=2, max_requests=None, seconds=30., delay_ns=5_000_000, emit=None):
    clock = Clock(); serial = Serial(clock, failure, delay_ns)
    events = []
    probe = TimingCAN(emit or events.append, max_requests=max_requests or 12+24*cycles,
                      max_seconds=seconds, serial_port=serial, clock=clock)
    return probe, serial, events, clock


class ProbeTests(unittest.TestCase):
    def test_fixed_order_one_outstanding_read_and_exact_timings(self):
        probe, serial, events, _ = fixture()
        with probe:
            report = collect(probe, UIDS, events.append, cycles=2)
        self.assertEqual(report['status'], 'READONLY_TIMING_COMPLETE')
        self.assertEqual(report['write_attempts'], 60)
        self.assertTrue(report['identities_verified'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertTrue(serial.closed)
        self.assertEqual([(f.kind, f.destination) for f in serial.writes[:12]], [(0, i) for i in range(1, 13)])
        for f in serial.writes[12:]:
            self.assertEqual(f.kind, 17)
            self.assertIn(int.from_bytes(f.data[:2], 'little'), (0x7019, 0x701B))
        self.assertEqual(report['statistics_ms']['request_RTT']['median'], 5.)
        self.assertEqual(report['statistics_ms']['wire_RTT']['max'], 5.)
        self.assertEqual(report['statistics_ms']['cycle_duration']['p95'], 120.)
        self.assertEqual(report['statistics_ms']['cycle_oldest_newest_spread']['max'], 115.)
        self.assertEqual(report['residual_final'], {'serial_pending_bytes': 0, 'parser_pending_bytes': 0, 'parser_discarded_bytes': 0})

    def test_timeout_poisons_and_preserves_partial_results_without_retry(self):
        probe, serial, events, _ = fixture('timeout')
        with probe:
            report = collect(probe, UIDS, events.append, cycles=2)
            with self.assertRaises(RuntimeError): probe.query(1, 'position')
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertEqual(len(serial.writes), 13)
        self.assertEqual(len(report['requests']), 13)
        self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(report['statistics_ms']['request_RTT']['count'], 12)

    def test_uid_mismatch_stops_before_parameter_reads(self):
        probe, serial, events, _ = fixture('uid')
        with probe: report = collect(probe, UIDS, events.append, cycles=2)
        self.assertFalse(report['identities_verified'])
        self.assertEqual(len(serial.writes), 2)
        self.assertTrue(all(f.kind == 0 for f in serial.writes))

    def test_invalid_or_unexpected_replies_abort_immediately(self):
        for failure in ('status', 'nan', 'wrongid', 'duplicate', 'noise', 'partial', 'unsolicited', 'shortwrite'):
            with self.subTest(failure=failure):
                probe, serial, events, _ = fixture(failure)
                with probe: report = collect(probe, UIDS, events.append, cycles=2)
                self.assertEqual(report['status'], 'INCOMPLETE')
                self.assertEqual(len(serial.writes), 13)
                self.assertFalse(report['requests'][-1]['ok'])

    def test_request_and_wall_time_budgets_end_without_extra_request(self):
        probe, serial, events, _ = fixture(max_requests=36)
        with probe: report = collect(probe, UIDS, events.append, cycles=2)
        self.assertEqual(len(serial.writes), 36)
        self.assertEqual(report['cycles_completed'], 1)
        self.assertEqual(report['status'], 'INCOMPLETE')
        probe, serial, events, _ = fixture(seconds=1., delay_ns=200_000_000)
        with probe: report = collect(probe, UIDS, events.append, cycles=2)
        self.assertLessEqual(len(serial.writes), 5)
        self.assertEqual(report['status'], 'INCOMPLETE')

    def test_only_exact_read_allowlist_at_query_and_physical_write(self):
        probe, serial, _, _ = fixture()
        with probe:
            for name in ('current', 'voltage', 'can_timeout', 'enable'):
                with self.assertRaises(ValueError): probe.query(1, name)
            for mid in (True, 0, 13):
                with self.assertRaises(ValueError): probe.query(mid, 'position')
            probe.pending = (1, 'position')
            with self.assertRaises(ValueError): probe.serial.write(read_request(1, 'voltage'))
            with self.assertRaises(ValueError): probe.serial.write(wire((3<<24)|(0xFD<<8)|1, bytes(8)))
            probe.pending = None
        self.assertFalse(serial.writes)

    def test_interrupt_and_nonempty_initial_boundary_never_write(self):
        for cause in ('interrupt', 'buffer'):
            probe, serial, events, _ = fixture()
            if cause == 'interrupt': probe.check_interrupt = lambda: (_ for _ in ()).throw(InterruptedError('operator'))
            else: serial.buffer.extend(b'AT')
            with probe: report = collect(probe, UIDS, events.append, cycles=2)
            self.assertEqual(report['status'], 'INCOMPLETE')
            self.assertFalse(serial.writes)

    def test_nested_query_rejected_while_one_is_outstanding(self):
        probe, serial, events, _ = fixture()
        def emit(event):
            events.append(event)
            if event['kind'] == 'can_tx':
                with self.assertRaises(RuntimeError): probe.query(2, 'position')
        probe.output = emit
        with probe: report = collect(probe, UIDS, events.append, cycles=2)
        self.assertEqual(report['status'], 'READONLY_TIMING_COMPLETE')
        self.assertEqual(len(serial.writes), 60)


class CLITests(unittest.TestCase):
    def args(self, root):
        uid = root/'uids.json'; uid.write_text(json.dumps(UIDS))
        return ['--expected-uids', str(uid), '--output', str(root/'capture')]

    def test_dry_run_never_opens_serial_locks_or_creates_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch('singularitydog_hw.can_timing_probe.TimingCAN') as can, patch('singularitydog_hw.can_timing_probe.ownership_locks') as lock, redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main(self.args(root)), 0)
            can.assert_not_called(); lock.assert_not_called()
            self.assertFalse((root/'capture').exists())
            self.assertNotIn(UIDS[1], out.getvalue())

    def test_invalid_plan_and_git_output_before_hardware(self):
        for cycles, seconds in ((0, 30), (101, 30), (True, 30), (1, math.nan), (1, 121)):
            with self.assertRaises(ValueError): make_plan(cycles, seconds)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args = self.args(root); (root/'.git').mkdir()
            with patch('singularitydog_hw.can_timing_probe.TimingCAN') as can, redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main([*args, '--execute-readonly'])
            can.assert_not_called()

    def test_cooperating_manual_calibration_lock_excludes_probe(self):
        with tempfile.TemporaryDirectory() as tmp, patch('singularitydog_hw.can_timing_probe.Path.home', return_value=Path(tmp)):
            with ownership_locks():
                with self.assertRaises(BlockingIOError):
                    with ownership_locks(): pass

    def test_partial_capture_is_private_and_saved_on_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args = self.args(root)
            clock = Clock(); serial = Serial(clock, 'timeout')
            def factory(emit, **kwargs): return TimingCAN(emit, **kwargs, serial_port=serial, clock=clock)
            with patch('singularitydog_hw.can_timing_probe.TimingCAN', side_effect=factory), patch('singularitydog_hw.can_timing_probe.Path.home', return_value=root), redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main([*args, '--execute-readonly', '--cycles', '2']), 1)
            report = json.loads((root/'capture/summary.json').read_text())
            self.assertEqual(report['status'], 'INCOMPLETE')
            self.assertEqual(report['write_attempts'], 13)
            self.assertTrue((root/'capture/events.jsonl').exists())
            self.assertEqual(stat.S_IMODE((root/'capture').stat().st_mode), 0o700)
            self.assertEqual(stat.S_IMODE((root/'capture/summary.json').stat().st_mode), 0o600)
            self.assertNotIn(UIDS[1], out.getvalue())


if __name__ == '__main__': unittest.main()
