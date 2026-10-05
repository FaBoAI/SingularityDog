"""File/fake-CAN tests only. No RobStride ports, network or device deployment."""
from contextlib import contextmanager, redirect_stdout, redirect_stderr
import errno
import hashlib
import io
import json
import math
import os
from pathlib import Path
import stat
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import stationary_velocity_probe as p

UIDS = {i: f'{i:016x}' for i in range(1, 13)}
BOOT = '11111111-2222-4333-8444-555555555555'  # Synthetic; never a live boot identity.


class Clock:
    def __init__(self): self.now, self.waits = 1_000_000_000, []
    def __call__(self): return self.now
    def advance(self, ns): self.now += ns
    def wait(self, seconds):
        assert seconds >= 0, 'Negative wait'
        self.waits.append(seconds); self.advance(round(seconds*1e9))


def wire(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


class Serial:
    def __init__(self, clock, bus, shared, *, fail=None, delay_ns=2_000_000, close_failure=False):
        self.clock, self.bus, self.shared = clock, bus, shared
        self.fail, self.delay_ns, self.close_failure = fail, delay_ns, close_failure
        self.buffer, self.writes, self.closed, self.fd = bytearray(), [], False, None
    @property
    def in_waiting(self): return len(self.buffer)
    def fileno(self):
        if self.fd is None: self.fd = os.open(os.devnull, os.O_RDONLY)
        return self.fd
    def close(self):
        self.shared['order'].append('device-close-'+self.bus)
        if self.close_failure: raise OSError('synthetic device close failure')
        self.closed = True
        if self.fd is not None:
            os.close(self.fd); self.fd = None
    def force_test_cleanup(self):
        self.close_failure = False
        if not self.closed: self.close()
    def write(self, raw):
        assert not self.buffer, 'Second request before first reply'
        parser = p.codec.ATParser(); frames = parser.feed(raw)
        assert len(frames) == 1 and not parser.buffer and not parser.discarded_bytes
        frame = frames[0]; mid = frame.destination
        assert mid in p.IDS[self.bus]
        self.writes.append(frame); self.shared['writes'].append((self.bus, frame))
        self.shared['order'].append('write-'+self.bus)
        if frame.kind == 0:
            uid = 'f'*16 if self.fail == 'uid' and mid == 2 else UIDS[mid]
            answer = wire((mid << 8)|0xFE, bytes.fromhex(uid))
        else:
            assert frame.kind == 17
            self.shared['parameter_count'] += 1
            first = self.shared['parameter_count'] == 1
            if first and self.fail == 'timeout': return len(raw)
            if first and self.fail == 'shortwrite': return len(raw)-1
            index = int.from_bytes(frame.data[:2], 'little')
            value = 0.125 if index == 0x701B else 2.5
            if first and self.fail == 'nan': value = math.nan
            status = 1 if first and self.fail == 'status' else 0
            if first and self.fail == 'wrongid': mid = 2 if mid != 2 else 3
            reserved = 1 if first and self.fail == 'reserved' else 0
            answer = wire((17<<24)|(status<<16)|(mid<<8)|0xFD, struct.pack('<HHf', index, reserved, value))
            if first:
                if self.fail == 'duplicate': answer += answer
                if self.fail == 'noise': answer = b'noise'+answer
                if self.fail == 'partial': answer += b'A'
                if self.fail == 'unsolicited': answer = wire((2<<24)|(mid<<8)|0xFD, bytes(8))+answer
        self.buffer.extend(answer)
        return len(raw)
    def read(self, count):
        self.clock.advance(self.delay_ns)
        data = bytes(self.buffer[:count]); del self.buffer[:count]
        return data


def fixture(mid=6, samples=2, period=200, fail=None, delay_ns=2_000_000, sink=None):
    clock, shared = Clock(), {'writes': [], 'parameter_count': 0, 'order': []}
    plan, events, cans, serials = p.make_plan(mid, samples, period), [], {}, {}
    for bus in p.IDS:
        serials[bus] = Serial(clock, bus, shared, fail=fail, delay_ns=delay_ns)
        cans[bus] = p.ProbeCAN(bus, plan, lambda e, b=bus: (sink or events.append)({'bus': b, **e}),
                               clock=clock, serial_port=serials[bus])
    return plan, clock, shared, cans, serials, events


def run_fixture(parts, check=lambda: None):
    plan, clock, shared, cans, serials, events = parts
    with cans['front'], cans['rear']:
        return p.collect(cans, UIDS, plan, clock=clock, wait=clock.wait, check=check)


class AcquisitionTests(unittest.TestCase):
    def test_exact_identity_barrier_then_single_id_pvp_raw_redecode(self):
        parts = fixture(mid=10); report = run_fixture(parts)
        plan, clock, shared, cans, serials, events = parts
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertTrue(report['all12_identities_verified'])
        self.assertTrue(report['full_requested_slot_coverage'])
        self.assertEqual(report['complete_triplets'], 2)
        self.assertEqual(report['write_attempts'], 18)
        self.assertEqual([(b, f.kind, f.destination) for b, f in shared['writes'][:12]],
                         [('front' if i <= 6 else 'rear', 0, i) for i in range(1, 13)])
        self.assertEqual([(b, f.kind, f.destination, int.from_bytes(f.data[:2], 'little'))
                          for b, f in shared['writes'][12:]], [('rear', 17, 10, n) for n in (0x7019, 0x701B, 0x7019)*2])
        for row in report['samples']:
            for key, param in zip(('position_before', 'velocity', 'position_after'), p.READS):
                p.validate_receipt(row[key], 10, param)
        self.assertEqual(len(events), 5*18)
        self.assertTrue(all(s.closed for s in serials.values()))
        self.assertEqual(report['analysis']['reported_velocity_rad_s']['max_abs'], .125)
        self.assertTrue(all(d['position_change_rad'] == 0 and not d['velocity_ground_truth']
                            for d in report['analysis']['host_bracketing_position_differences']))
        self.assertFalse(report['analysis']['stationary_gate_applied'])
        self.assertTrue(all(report[k] is False for k in p.FLAGS))

    def test_uid_mismatch_has_no_type17_and_zero_velocity_statistics(self):
        parts = fixture(fail='uid'); report = run_fixture(parts)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertFalse(report['all12_identities_verified'])
        self.assertEqual(len(parts[2]['writes']), 2)
        self.assertTrue(all(f.kind == 0 for _, f in parts[2]['writes']))
        self.assertEqual(report['analysis']['status'], 'NO_COMPLETE_SAMPLES')
        self.assertIsNone(report['analysis']['reported_velocity_rad_s'])

    def test_timeout_and_invalid_replies_preserve_partial_trace_without_retry(self):
        for failure in ('timeout', 'shortwrite', 'status', 'nan', 'wrongid', 'reserved', 'duplicate', 'noise', 'partial', 'unsolicited'):
            with self.subTest(failure=failure):
                parts = fixture(fail=failure); report = run_fixture(parts)
                self.assertEqual(report['status'], 'INCOMPLETE')
                self.assertEqual(len(parts[2]['writes']), 13)
                self.assertEqual(len(report['requests']), 13)
                self.assertEqual(report['incomplete_triplets'], 1)
                self.assertEqual(report['analysis']['complete_triplets'], 0)
                self.assertIsNone(report['analysis']['reported_velocity_rad_s'])
                self.assertFalse(report['samples'][0]['complete'])
                self.assertIn('error', report['samples'][0])
                self.assertTrue(parts[5])
                self.assertTrue(all(c.poisoned for c in parts[3].values()))

    def test_twenty_ms_whole_triplet_reserve_drops_before_tx_and_retains_denominator(self):
        parts = fixture(samples=5, period=20); report = run_fixture(parts)
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertEqual(report['requested_slots'], 5)
        self.assertEqual(report['complete_triplets'], 3)
        self.assertEqual(report['unacquired_slots'], [3, 4])
        self.assertEqual(report['dropped_slots'], [{'slot_index': i, 'state': 'DROPPED', 'reason': 'insufficient_segment_budget'} for i in (3, 4)])
        self.assertEqual(len(parts[2]['writes']), 12+3*3)
        self.assertFalse(report['full_requested_slot_coverage'])
        self.assertEqual(report['analysis']['reported_velocity_rad_s']['samples'], 3)
        self.assertEqual(report['analysis']['requested_triplets'], 5)
        self.assertFalse(report['analysis']['full_requested_slot_coverage'])
        self.assertLessEqual(report['segment_end_ns'], report['segment_deadline_ns'])

    def test_zero_triplet_completed_is_not_success_even_with_all_identity_and_valid_trace(self):
        parts = fixture(samples=1, period=20); report = run_fixture(parts)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertEqual(report['complete_triplets'], 0)
        self.assertEqual(report['dropped_slot_count'], 1)
        self.assertEqual(len(parts[2]['writes']), 12)
        self.assertTrue(report['all12_identities_verified'])
        self.assertIsNone(report['analysis']['reported_velocity_rad_s'])

    def test_first_physical_write_rechecks_reserve_after_guard_overhead(self):
        parts = fixture(samples=2); selected = parts[3]['front']; delayed = False
        def check():
            nonlocal delayed
            if selected.pending == (6, 'position') and selected.tx_count == 10 and not delayed:
                parts[1].advance(160_000_000); delayed = True
        selected.check_interrupt = check
        report = run_fixture(parts)
        self.assertTrue(delayed)
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertEqual(report['complete_triplets'], 1)
        self.assertEqual(report['triplet_reserve_abort_before_write_count'], 1)
        self.assertEqual(report['dropped_slots'], [{'slot_index': 1, 'state': 'DROPPED', 'reason': 'insufficient_segment_budget'}])
        self.assertEqual(len(parts[2]['writes']), 15)
        self.assertEqual(report['queries_sent'], 15)
        self.assertEqual(report['request_intents_logged'], 16)
        self.assertFalse(report['full_requested_slot_coverage'])

    def test_saved_request_intent_count_survives_reserve_consumed_inside_sink(self):
        seen, delayed = [], False
        def sink(event):
            nonlocal delayed
            seen.append(event)
            if event['bus'] == 'front' and event['kind'] == 'can_tx' and event['sequence'] == 10:
                parts[1].advance(160_000_000); delayed = True
        parts = fixture(samples=2, sink=sink); report = run_fixture(parts)
        self.assertTrue(delayed)
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertEqual(report['complete_triplets'], 1)
        self.assertEqual(report['triplet_reserve_abort_before_write_count'], 1)
        self.assertEqual(report['queries_sent'], 15)
        self.assertEqual(report['request_intents_logged'], 16)
        self.assertEqual(sum(c.tx_count for c in parts[3].values()), 15)
        self.assertEqual(sum(e['kind'] == 'can_tx' for e in seen), 16)

    def test_slow_triplets_drop_missed_releases_without_catchup(self):
        parts = fixture(samples=6, period=20, delay_ns=9_000_000); report = run_fixture(parts)
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertEqual([r['slot_index'] for r in report['samples']], [0, 1, 2])
        self.assertEqual([r['slot_index'] for r in report['dropped_slots']], [3, 4, 5])
        self.assertEqual(report['dropped_slots'][0]['reason'], 'release_elapsed_before_triplet_start')
        starts = [r['begin_ns'] for r in report['samples']]
        self.assertTrue(all(b-a >= 20_000_000 for a, b in zip(starts, starts[1:])))
        self.assertEqual(report['period_deadline_missed_slots'], [0, 1, 2])

    def test_mid_triplet_deadline_failure_remains_incomplete(self):
        parts = fixture(samples=1, period=200, delay_ns=70_000_000); report = run_fixture(parts)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertEqual(report['complete_triplets'], 0)
        self.assertEqual(report['incomplete_triplets'], 1)
        self.assertFalse(report['samples'][0]['complete'])
        self.assertEqual(report['analysis']['complete_triplets'], 0)
        self.assertGreaterEqual(len(parts[2]['writes']), 14)

    def test_release_crossed_by_guard_never_passes_negative_wait(self):
        parts = fixture(samples=2); clock = parts[1]; advanced = False
        def check():
            nonlocal advanced
            if clock.waits and not advanced:
                clock.advance(200_000_000); advanced = True
        report = run_fixture(parts, check)
        self.assertTrue(advanced)
        self.assertEqual(report['status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
        self.assertTrue(all(v >= 0 for v in clock.waits))

    def test_query_and_physical_boundary_enforce_bus_id_and_types(self):
        parts = fixture(); front = parts[3]['front']
        with front:
            for mid, name in ((7, None), (1, 'position'), (6, 'voltage'), (True, None), (0, None)):
                with self.subTest(mid=mid, name=name), self.assertRaises(ValueError): front.query(mid, name)
            for pending, raw in (((7, 'position'), p.codec.read_request(7, 'position')),
                                 ((1, 'position'), p.codec.read_request(1, 'position')),
                                 ((6, 'position'), wire((3<<24)|(0xFD<<8)|6, bytes(8))),
                                 ((6, 'position'), wire((1<<24)|(0xFD<<8)|6, bytes(8))),
                                 ((6, 'position'), p.codec.read_request(6, 'current'))):
                front.pending = pending
                with self.assertRaises(ValueError): front.serial.write(raw)
            front.pending = None
        self.assertFalse(parts[2]['writes'])

    def test_initial_residual_or_interrupt_has_no_tx(self):
        for kind in ('residual', 'interrupt'):
            parts = fixture()
            if kind == 'residual': parts[4]['front'].buffer.extend(b'AT')
            def check():
                if kind == 'interrupt': raise InterruptedError('operator')
            report = run_fixture(parts, check)
            self.assertEqual(report['status'], 'INCOMPLETE')
            self.assertFalse(parts[2]['writes'])

    def test_strict_raw_receipt_schema_nonfinite_and_changed_bytes_rejected(self):
        original = run_fixture(fixture())
        for mutate in (lambda r: r['samples'][0]['velocity'].update(value=math.nan),
                       lambda r: r['samples'][0]['velocity'].update(value=.126),
                       lambda r: r['samples'][0]['velocity'].update(extra=1),
                       lambda r: r['samples'][0]['velocity'].update(raw_request_hex=p.codec.read_request(6, 'position').hex()),
                       lambda r: r['samples'][0].update(complete=False),
                       lambda r: r['samples'].append(r['samples'][0]),
                       lambda r: r['unacquired_slots'].append(1)):
            value = json.loads(json.dumps(original)); mutate(value)
            with self.assertRaises((ValueError, KeyError)): p.describe_samples(value)


class FileAndPlanTests(unittest.TestCase):
    def test_plan_limits_query_and_independent_trace_budgets(self):
        plan = p.make_plan(6, 500, 20)
        self.assertEqual(plan['segment_duration_bound_s'], 10)
        self.assertEqual(plan['maximum_queries'], 1512)
        self.assertEqual(plan['trace_events_minimum_estimate'], 7560)
        self.assertEqual(plan['trace_events_two_receive_chunks_estimate'], 9072)
        self.assertGreater(plan['trace_event_budget'], 9072)
        self.assertEqual(plan['trace_event_budget'], 16384)
        self.assertEqual(p.make_plan(6, 50, 200)['segment_duration_bound_s'], 10)
        for args in ((0, 50, 200), (True, 50, 200), (1, 501, 20), (1, 51, 200), (1, 0, 200), (1, 1, 100), (1, True, 20)):
            with self.assertRaises(ValueError): p.make_plan(*args)

    def test_default_plan_never_reads_files_checks_boot_locks_paths_or_devices(self):
        with patch.object(p, 'load_uids') as uids, patch.object(p, 'private_path') as paths, \
             patch.object(p, 'ProbeCAN') as can, patch.object(p.timing, 'ownership_locks') as lock, \
             patch.object(p.dual, 'validate_ports') as ports, patch.object(p.dual, 'BootIdentityGuard') as boot, \
             redirect_stdout(io.StringIO()) as out:
            self.assertEqual(p.main(['--id', '10', '--output', '/does/not/exist', '--expected-uids', '/does/not/exist']), 0)
        for item in (uids, paths, can, lock, ports, boot): item.assert_not_called()
        self.assertEqual(json.loads(out.getvalue())['status'], 'PLAN_ONLY')

    def test_fresh_symlink_ancestors_git_and_existing_files_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); original = root/'existing'; original.write_text('keep')
            link = root/'link'; link.symlink_to(root, target_is_directory=True)
            broken = root/'broken'; broken.symlink_to(root/'absent')
            git = root/'repo'; git.mkdir(); (git/'.git').mkdir()
            for dest in (original, broken, link/'new', git/'new', root/'absent'/'new'):
                with self.subTest(dest=str(dest)), self.assertRaises(ValueError): p.PrivateFile(dest)
            self.assertEqual(original.read_text(), 'keep')
            reserved = p.PrivateFile(root/'new')
            self.assertEqual(stat.S_IMODE((root/'new').stat().st_mode), 0o600)
            with self.assertRaises(ValueError): p.PrivateFile(root/'new')
            reserved.close()

    def test_strict_uid_json_rejects_duplicates_nonfinite_and_wrong_identity_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp).resolve()/'uid.json'
            for value in ('{"1":"0000000000000001","1":"0000000000000001"}',
                          '{"x":NaN}', '{"x":1e999}', '{}', json.dumps({**UIDS, 12: UIDS[1]})):
                path.write_text(value)
                with self.assertRaises(ValueError): p.load_uids(path)
            raw = json.dumps(UIDS).encode(); path.write_bytes(raw)
            actual, digest = p.load_uids(path)
            self.assertEqual(actual, UIDS); self.assertEqual(digest, hashlib.sha256(raw).hexdigest())
            path.unlink(); path.symlink_to(Path(tmp).resolve()/'absent')
            with self.assertRaises(ValueError): p.load_uids(path)

    def test_reservation_initialization_failure_closes_created_fd(self):
        with tempfile.TemporaryDirectory() as tmp:
            fd_seen = []
            def fail(fd, mode): fd_seen.append(fd); raise OSError('fchmod')
            with patch.object(p.os, 'fchmod', side_effect=fail), self.assertRaises(OSError): p.PrivateFile(Path(tmp).resolve()/'new')
            with self.assertRaises(OSError) as error: os.fstat(fd_seen[0])
            self.assertEqual(error.exception.errno, errno.EBADF)

    def test_event_and_byte_budget_failures_preserve_valid_prefix(self):
        for cap in ('event', 'byte'):
            with tempfile.TemporaryDirectory() as tmp:
                trace = p.EventTrace(Path(tmp).resolve()/'events.jsonl'); trace.record('front', {'kind': 'one'})
                prefix = trace.path.read_bytes()
                with patch.object(p, 'MAX_EVENTS', 1 if cap == 'event' else 16384), \
                     patch.object(p, 'MAX_TRACE_BYTES', len(prefix) if cap == 'byte' else 4*1024*1024):
                    with self.assertRaises(ValueError): trace.record('rear', {'kind': 'two'})
                    summary = trace.close()
                self.assertFalse(summary['complete']); self.assertEqual(summary['event_count'], 1)
                self.assertEqual(summary['attempted_event_count'], 2)
                self.assertEqual(trace.path.read_bytes(), prefix)
                self.assertEqual(summary['sha256'], hashlib.sha256(prefix).hexdigest())

    def test_partial_raw_write_and_flush_failure_never_claim_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace = p.EventTrace(Path(tmp).resolve()/'partial')
            def fail(raw): trace.stream.write(raw[:4]); raise OSError('partial')
            with patch.object(trace, 'write', side_effect=fail), self.assertRaises(OSError): trace.record('front', {'kind': 'one'})
            summary = trace.close()
            self.assertFalse(summary['complete']); self.assertEqual(summary['byte_count'], 4)
            self.assertEqual(summary['event_count'], 0)
            self.assertEqual(summary['sha256'], hashlib.sha256(trace.path.read_bytes()).hexdigest())
            trace = p.EventTrace(Path(tmp).resolve()/'flush'); trace.record('rear', {'kind': 'one'})
            with patch.object(p.os, 'fsync', side_effect=OSError('fsync')): summary = trace.close()
            self.assertFalse(summary['complete']); self.assertTrue(trace.stream.closed)

    def test_changed_reserved_path_never_passes_finalization(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); artifact = p.PrivateFile(root/'report')
            artifact.report({'status': 'RECORDED_REVIEW_REQUIRED'}); artifact.close()
            artifact.path.rename(root/'old'); artifact.path.write_text('changed')
            with self.assertRaises(ValueError): artifact.finalized_sha256()

    def test_live_known_descriptor_is_unclosed_even_if_serial_attribute_was_cleared(self):
        class ClearedSerial:
            serial = None
            def __exit__(self, *_): pass
        fd = os.open(os.devnull, os.O_RDONLY)
        try:
            closed, errors = p.close_devices([{'can': ClearedSerial(), 'fd': fd, 'bus': 'front'}])
            self.assertFalse(closed); self.assertTrue(errors)
        finally:
            os.close(fd)


class CLITests(unittest.TestCase):
    def setup_cli(self, root, **serial_options):
        clock, shared = Clock(), {'writes': [], 'parameter_count': 0, 'order': []}
        uids = root/'uids.json'; uids.write_text(json.dumps(UIDS))
        args = ['--id', '10', '--samples', '2', '--execute-readonly', '--expected-uids', str(uids),
                '--expected-boot-id', BOOT, '--output', str(root/'report.json'), '--trace-events', str(root/'events.jsonl')]
        serials = {bus: Serial(clock, bus, shared, **serial_options) for bus in p.IDS}
        bindings = {b: {'resolved': 'fake-'+b, 'st_rdev': os.fstat(s.fileno()).st_rdev} for b, s in serials.items()}
        probe_class, collect = p.ProbeCAN, p.collect
        @contextmanager
        def lease(name):
            shared['order'].append('lock-'+name)
            try: yield
            finally: shared['order'].append('unlock-'+name)
        class BootGuard:
            boot_id = BOOT
            def check(self): pass
            def close(self): shared['order'].append('boot-close')
        def factory(bus, plan, sink, **kw):
            return probe_class(bus, plan, sink, serial_port=serials[bus], clock=clock, **kw)
        patches = [patch.object(p.dual, 'validate_ports', return_value=bindings),
                   patch.object(p.dual, 'binding_matches', return_value=True),
                   patch.object(p.dual, 'BootIdentityGuard', BootGuard),
                   patch.object(p.dual, 'port_lock', side_effect=lambda value: lease(value)),
                   patch.object(p.timing, 'ownership_locks', side_effect=lambda: lease('common')),
                   patch.object(p, 'ProbeCAN', side_effect=factory),
                   patch.object(p, 'collect', side_effect=lambda cans, uids, plan, **kw: collect(cans, uids, plan, clock=clock, wait=clock.wait, **kw))]
        return args, patches, shared, serials

    def call_cli(self, args, patches):
        from contextlib import ExitStack
        with ExitStack() as stack, redirect_stdout(io.StringIO()) as out:
            for item in patches: stack.enter_context(item)
            code = p.main(args)
        return code, json.loads(out.getvalue())

    def test_success_receipt_only_after_device_close_and_private_report_finalization(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
            code, receipt = self.call_cli(args, patches)
            self.assertEqual(code, 0); self.assertEqual(receipt['status'], 'SAVED_READONLY_PROBE_REVIEW_REQUIRED')
            raw = (root/'report.json').read_bytes(); report = json.loads(raw)
            self.assertEqual(receipt['report_sha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(report['status'], 'RECORDED_REVIEW_REQUIRED')
            self.assertEqual(report['data_collection_status'], 'COMPLETE_READONLY_PROBE_CAPTURE')
            self.assertFalse(report['report_finalization_claimed'])
            self.assertTrue(report['all_device_contexts_closed'])
            self.assertTrue(report['trace_events']['complete'])
            self.assertEqual(report['trace_events']['sha256'], hashlib.sha256((root/'events.jsonl').read_bytes()).hexdigest())
            self.assertIn('can_readonly.py', report['source_sha256'])
            order = shared['order']; last_close = max(order.index('device-close-'+b) for b in p.IDS)
            self.assertGreater(order.index('unlock-common'), last_close)
            self.assertGreater(order.index('unlock-fake-front'), last_close)
            self.assertGreater(order.index('unlock-fake-rear'), last_close)
            self.assertNotIn(UIDS[1], json.dumps(receipt))

    def test_report_close_or_fsync_failure_cannot_publish_completion_receipt(self):
        for failure in ('close', 'fsync'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
                original_close, original_report = p.PrivateFile.close, p.PrivateFile.report
                def close(file):
                    original_close(file)
                    if file.path.name == 'report.json': raise OSError('summary close')
                def report(file, value):
                    with patch.object(p.os, 'fsync', side_effect=OSError('summary fsync')): original_report(file, value)
                patches.append(patch.object(p.PrivateFile, 'close', close) if failure == 'close' else patch.object(p.PrivateFile, 'report', report))
                code, receipt = self.call_cli(args, patches)
                self.assertEqual(code, 1); self.assertEqual(receipt['status'], 'INCOMPLETE')
                self.assertIsNone(receipt['report_sha256'])
                saved = json.loads((root/'report.json').read_text())
                self.assertEqual(saved['status'], 'RECORDED_REVIEW_REQUIRED')
                self.assertFalse(saved['report_finalization_claimed'])

    def test_trace_finalization_failure_makes_saved_capture_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
            close = p.EventTrace.close
            def fail(trace):
                with patch.object(p.os, 'fsync', side_effect=OSError('raw fsync')): return close(trace)
            patches.append(patch.object(p.EventTrace, 'close', fail))
            code, receipt = self.call_cli(args, patches)
            self.assertEqual(code, 1); self.assertIsNone(receipt['report_sha256'])
            saved = json.loads((root/'report.json').read_text())
            self.assertEqual(saved['status'], 'INCOMPLETE')
            self.assertFalse(saved['trace_events']['complete'])

    def test_overbudget_trace_stops_before_next_pvp_write_and_retains_failed_raw(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
            patches.append(patch.object(p, 'MAX_EVENTS', 70))
            code, receipt = self.call_cli(args, patches)
            self.assertEqual(code, 1); self.assertIsNone(receipt['report_sha256'])
            self.assertEqual(len(shared['writes']), 14)
            saved = json.loads((root/'report.json').read_text())
            self.assertFalse(saved['trace_events']['complete'])
            self.assertEqual(saved['trace_events']['event_count'], 70)
            self.assertEqual(saved['analysis']['complete_triplets'], 0)
            self.assertFalse(saved['full_requested_slot_coverage'])
            self.assertEqual(saved['status'], 'INCOMPLETE')

    def test_second_open_failure_or_opened_descriptor_mismatch_closes_both_before_unlock(self):
        for cause in ('open', 'descriptor'):
            with self.subTest(cause=cause), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
                if cause == 'open':
                    enter = p.timing.TimingCAN.__enter__
                    def fail(can):
                        if can.bus == 'rear': raise OSError('second open')
                        return enter(can)
                    patches.append(patch.object(p.timing.TimingCAN, '__enter__', fail))
                else:
                    bindings = {b: {'resolved': 'fake-'+b, 'st_rdev': os.fstat(s.fileno()).st_rdev+(1 if b == 'rear' else 0)}
                                for b, s in serials.items()}
                    patches.append(patch.object(p.dual, 'validate_ports', return_value=bindings))
                code, receipt = self.call_cli(args, patches)
                self.assertEqual(code, 1); self.assertFalse(shared['writes'])
                self.assertTrue(all(s.closed for s in serials.values()))
                last_close = max(shared['order'].index('device-close-'+b) for b in p.IDS)
                self.assertGreater(shared['order'].index('unlock-common'), last_close)
                saved = json.loads((root/'report.json').read_text())
                self.assertTrue(saved['all_device_contexts_closed'])
                self.assertFalse(saved['full_requested_slot_coverage'] if 'full_requested_slot_coverage' in saved else False)

    def test_boot_mismatch_and_binding_change_prevent_all_writes(self):
        for cause in ('boot', 'binding'):
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
                if cause == 'boot': args[args.index('--expected-boot-id')+1] = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
                else: patches.append(patch.object(p.dual, 'binding_matches', return_value=False))
                code, receipt = self.call_cli(args, patches)
                self.assertEqual(code, 1); self.assertFalse(shared['writes'])
                self.assertFalse(json.loads((root/'report.json').read_text())['hardware_opened'])
                for s in serials.values(): s.force_test_cleanup()

    def test_lock_contention_prevents_device_constructor_and_preserves_error_trace(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root)
            patches.append(patch.object(p.timing, 'ownership_locks', side_effect=BlockingIOError('owned')))
            code, receipt = self.call_cli(args, patches)
            self.assertEqual(code, 1); self.assertFalse(shared['writes'])
            saved = json.loads((root/'report.json').read_text())
            self.assertFalse(saved['hardware_open_attempted']); self.assertTrue(saved['trace_events']['complete'])
            for s in serials.values(): s.force_test_cleanup()

    def test_unknown_device_close_retains_shared_leases_until_process_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); args, patches, shared, serials = self.setup_cli(root, close_failure=True)
            before = len(p._UNCLOSED_DEVICE_LEASES)
            code, receipt = self.call_cli(args, patches)
            self.assertEqual(code, 1); self.assertIsNone(receipt['report_sha256'])
            saved = json.loads((root/'report.json').read_text())
            self.assertFalse(saved['all_device_contexts_closed'])
            self.assertTrue(saved['resource_leases_retained_until_process_exit'])
            self.assertNotIn('unlock-common', shared['order'])
            self.assertNotIn('unlock-fake-front', shared['order'])
            self.assertEqual(len(p._UNCLOSED_DEVICE_LEASES), before+1)
            lease, devices = p._UNCLOSED_DEVICE_LEASES.pop()
            self.assertTrue(all(item['can'].poisoned for item in devices))
            for s in serials.values(): s.force_test_cleanup()
            lease.close()


if __name__ == '__main__': unittest.main()
