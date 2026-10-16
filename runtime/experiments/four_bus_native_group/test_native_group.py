"""Genuine all-four C++ regressions using only anonymous socket peers."""
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.native_diagnostic_transport import Record
from experiments.four_bus_type1 import type1_transport as T
from experiments.four_bus_type1.test_subset_active import reply, type1, unreachable
from . import build

BOOT = '11111111-2222-3333-4444-555555555555'


class Port(C.Structure):
    _fields_ = [('owner_started_ns', C.c_uint64), ('owner_finished_ns', C.c_uint64),
                ('status', C.c_int32), ('count', C.c_uint32), ('records', Record*3),
                ('stats', active.Stats), ('error', C.c_char*256)]


class Result(C.Structure):
    _fields_ = [(name, C.c_uint64) for name in ('ticket', 'submitted_ns', 'validated_ns',
        'released_ns', 'collected_ns', 'cancel_requested_ns')] + [('ports', Port*4)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@unittest.skipUnless(shutil.which('c++'), 'A native compiler is required')
class NativeGroupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = build.build(Path(cls.directory.name)/'build')
        cls.lib = active.load_library(cls.path, expected_sha256=sha(cls.path))
        for name, arguments, result in (
            ('sdg_abi', (), C.c_uint32), ('sdg_result_size', (), C.c_uint32),
            ('sdg_port_size', (), C.c_uint32),
            ('sdg_create', (C.POINTER(C.c_void_p), C.POINTER(C.c_uint32),
                C.POINTER(C.c_uint64), C.c_uint64, C.POINTER(active.PairOwnerSettings),
                C.POINTER(C.c_char), C.c_uint32), C.c_void_p),
            ('sdg_start_hold', (C.c_void_p, C.POINTER(C.c_ubyte), C.POINTER(C.c_ubyte),
                C.c_uint64, C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_initial_hold', (C.c_void_p, C.POINTER(C.c_ubyte), C.c_uint64,
                C.POINTER(Result), C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_wait_hold', (C.c_void_p, C.c_uint64, C.POINTER(Result),
                C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_wait_voltage', (C.c_void_p, C.c_uint64, C.POINTER(Result),
                C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_output', (C.c_void_p, C.c_uint64, C.POINTER(C.c_ubyte), C.c_uint64,
                C.c_uint64, C.c_int, C.POINTER(Result), C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_cancel', (C.c_void_p,), None),
            ('sdg_cancel_snapshot', (C.c_void_p, C.POINTER(Result), C.POINTER(Result),
                C.POINTER(Result), C.POINTER(C.c_uint32), C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sdg_close', (C.c_void_p, C.POINTER(active.PairOwnerSettings),
                C.POINTER(C.c_char), C.c_uint32), C.c_int),
            ('sda_emergency_stop_subset', (C.c_void_p, C.c_uint32, C.c_uint64,
                C.POINTER(Record), C.POINTER(active.Stats), C.POINTER(active.StopResult),
                C.POINTER(C.c_char), C.c_uint32), C.c_int)):
            f = cls.lib[name]; f.argtypes = list(arguments); f.restype = result
            setattr(cls.lib, name, f)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.group = None; self.sessions = []; self.hosts = []; self.peers = []
        self.pipes = []; self.threads = []; self.seen = [[], [], [], []]
        self.errors = []; self.peer_stop = threading.Event()
        self.boot = tempfile.TemporaryFile(); self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.error = C.create_string_buffer(256)
        self.ids = [(7, 8, 9), (10, 11, 12), (4, 5, 6), (1, 2, 3)]
        self.masks = (C.c_uint32*4)(7, 56, 56, 7)

    def tearDown(self):
        if self.group:
            self.close()
        for s in self.sessions:
            s.close()
        self.peer_stop.set()
        for host in self.hosts:
            host.close()
        for peer in self.peers:
            peer.close()
        for thread in self.threads:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive(), 'Synthetic peer remains alive')
        for r, w in self.pipes:
            os.close(r); os.close(w)
        self.boot.close()
        if self.errors:
            raise self.errors[0]

    def configure(self, *, gap=900_000, window=3):
        for ids in self.ids:
            host, peer = socket.socketpair(); host.setblocking(False)
            self.hosts.append(host); self.peers.append(peer)
            r, w = os.pipe(); self.pipes.append((r, w))
            first = 1 if ids[0] < 7 else 7
            lower, upper, kp, kd = {}, {}, {}, {}
            for mid in range(first, first+6):
                if mid in ids:
                    lower[mid], upper[mid], kp[mid], kd[mid] = -.5, .5, 3., .15
                else:
                    (lower[mid], upper[mid]), kp[mid], kd[mid] = unreachable(), 0., 0.
            self.sessions.append(active.ActiveSession(self.lib, host.fileno(), first_id=first,
                cancel_fd=r, boot_fd=self.boot.fileno(), boot_id=BOOT, raw_lower_by_id=lower,
                raw_upper_by_id=upper, kp_max_by_id=kp, kd_max_by_id=kd, gap_ns=gap, window=window))

    def create(self, *, masks=None, handles=None):
        if not self.sessions:
            self.configure()
        raw = (C.c_void_p*4)(*(s._handle for s in self.sessions)) if handles is None else handles
        settings = (active.PairOwnerSettings*4)()
        self.group = self.lib.sdg_create(raw, self.masks if masks is None else masks,
            (C.c_uint64*4)(0, 0, 0, 0), 0, settings, self.error, len(self.error))
        return settings

    def close(self):
        rows = (active.PairOwnerSettings*4)()
        rc = self.lib.sdg_close(self.group, rows, self.error, len(self.error))
        self.group = None
        self.assertEqual(rc, 0, self.error.value)
        self.assertTrue(all(r.status == 0 and r.restored == 1 for r in rows))
        return rows

    def start_peers(self, mutate=None):
        for port, peer in enumerate(self.peers):
            ready = threading.Event()
            def run(index=port, sock=peer, flag=ready):
                parser = ATParser(); flag.set()
                try:
                    while not self.peer_stop.is_set():
                        if not select.select([sock], [], [], .02)[0]:
                            continue
                        raw = sock.recv(4096)
                        if not raw:
                            return
                        for frame in parser.feed(raw):
                            self.seen[index].append(frame.wire)
                            answer = reply(frame.wire) if mutate is None else mutate(index, frame, len(self.seen[index]))
                            if answer:
                                sock.sendall(answer)
                except (OSError, ValueError):
                    pass
                except BaseException as error:
                    self.errors.append(error)
            thread = threading.Thread(target=run, name=f'group-socket-peer-{port}')
            thread.start(); self.threads.append(thread); self.assertTrue(ready.wait(1))

    def wires(self):
        return b''.join(type1(mid, kp=3., kd=.15) for ids in self.ids for mid in ids)

    def voltages(self):
        return b''.join(read_request(ids[0], 'voltage') for ids in self.ids)

    def start(self, *, wires=None, volts=None, deadline=None):
        ticket = C.c_uint64()
        w = self.wires() if wires is None else wires
        v = self.voltages() if volts is None else volts
        rc = self.lib.sdg_start_hold(self.group,
            (C.c_ubyte*len(w)).from_buffer_copy(w), (C.c_ubyte*len(v)).from_buffer_copy(v),
            time.monotonic_ns()+150_000_000 if deadline is None else deadline,
            C.byref(ticket), self.error, len(self.error))
        return rc, ticket.value

    def wait(self, ticket, *, voltage=False):
        result = Result()
        rc = getattr(self.lib, 'sdg_wait_voltage' if voltage else 'sdg_wait_hold')(
            self.group, ticket, C.byref(result), self.error, len(self.error))
        return rc, result

    def output(self, ticket, *, wires=None, validated=1, write_cap=None, reply_cap=None):
        result = Result(); wire = self.wires() if wires is None else wires
        rc = self.lib.sdg_output(self.group, ticket, (C.c_ubyte*len(wire)).from_buffer_copy(wire),
            time.monotonic_ns()+150_000_000 if reply_cap is None else reply_cap,
            time.monotonic_ns()+100_000_000 if write_cap is None else write_cap,
            validated, C.byref(result), self.error, len(self.error))
        return rc, result

    def snapshot(self):
        hold, voltage, output, bits = Result(), Result(), Result(), C.c_uint32()
        rc = self.lib.sdg_cancel_snapshot(self.group, C.byref(hold), C.byref(voltage),
            C.byref(output), C.byref(bits), self.error, len(self.error))
        self.assertEqual(rc, 0, self.error.value)
        return bits.value, hold, voltage, output

    def successful_inputs(self):
        rc, ticket = self.start(); self.assertEqual(rc, 0, self.error.value)
        rc, hold = self.wait(ticket); self.assertEqual(rc, 0, self.error.value)
        rc, voltage = self.wait(ticket, voltage=True); self.assertEqual(rc, 0, self.error.value)
        return ticket, hold, voltage

    def test_abi_same_library_receipt_and_exact_source_seals(self):
        self.assertEqual(self.lib.sdg_abi(), 1)
        self.assertEqual(self.lib.sdg_result_size(), C.sizeof(Result))
        self.assertEqual(self.lib.sdg_port_size(), C.sizeof(Port))
        self.assertIsNone(build.receipt_problem(self.path.parent))
        record = self.path.parent/'build-record.json'
        lib = T.load_library(self.path, expected_sha256=sha(self.path),
            ordinary_source_sha256=sha(self.path.parent/'ordinary_transport.cpp'),
            subset_stop_source_sha256=sha(self.path.parent/'subset_stop.cpp'),
            extension_source_sha256=sha(self.path.parent/'transport.cpp'), build_record_sha256=sha(record))
        self.assertEqual(lib.sda_subset_write_deadline_abi(), 1)
        with tempfile.TemporaryDirectory() as tmp:
            copied = Path(tmp)/'build'; shutil.copytree(self.path.parent, copied)
            (copied/'subset_active.cpp').write_text('// changed\n')
            self.assertIn('included source', build.receipt_problem(copied))

    def test_plan_opens_nothing_builds_nothing(self):
        with patch.object(C, 'CDLL', side_effect=AssertionError('dlopen')), \
             patch.object(subprocess_module(), 'run', side_effect=AssertionError('compiler')):
            self.assertEqual(build.main(['--output', '/unused/plan']), 0)

    def test_four_hold_voltage_output_byte_and_stamp_equivalence(self):
        self.create(); self.assertTrue(self.group, self.error.value); self.start_peers()
        for expected in (1, 2):
            ticket, hold, voltage = self.successful_inputs(); self.assertEqual(ticket, expected)
            rc, output = self.output(ticket); self.assertEqual(rc, 0, self.error.value)
            for port, ids in enumerate(self.ids):
                for phase, count in ((hold, 3), (voltage, 1), (output, 3)):
                    row = phase.ports[port]; self.assertEqual((row.status, row.count), (0, count))
                    self.assertEqual(row.stats.writes, count)
                    self.assertGreaterEqual(row.owner_started_ns, phase.released_ns)
                    self.assertGreaterEqual(phase.collected_ns, row.owner_finished_ns)
                    for r in list(row.records)[:count]:
                        self.assertEqual((r.written, r.received), (17, 17))
                        self.assertEqual(bytes(r.rx), reply(bytes(r.tx)))
                        self.assertLessEqual(r.start_ns, r.finish_ns)
                        self.assertLessEqual(r.finish_ns, r.received_ns)
                        self.assertLess(r.received_ns, r.deadline_ns)
                offset = (expected-1)*7
                self.assertEqual(self.seen[port][offset:offset+7],
                    [type1(mid, kp=3., kd=.15) for mid in ids]+[
                    read_request(ids[0], 'voltage')]+[type1(mid, kp=3., kd=.15) for mid in ids])
        self.close()

    def test_hold_return_preserves_native_voltage_overlap(self):
        self.create(); release_voltage = threading.Event()
        def mutate(_index, frame, _n):
            if frame.kind == 17:
                release_voltage.wait(1)
            return reply(frame.wire)
        self.start_peers(mutate)
        try:
            rc, ticket = self.start(); self.assertEqual(rc, 0)
            rc, hold = self.wait(ticket); self.assertEqual(rc, 0)
            self.assertTrue(all(row.count == 3 for row in hold.ports))
            self.assertEqual(self.output(ticket)[0], -1)
        finally:
            release_voltage.set()
        self.assertEqual(self.wait(ticket, voltage=True)[0], 0)
        self.assertEqual(self.output(ticket)[0], 0)

    def test_bad_fourth_group_is_atomic_no_writes_and_poison_sticky(self):
        self.create(); self.start_peers()
        wires = bytearray(self.wires()); wires[-1] = 0
        self.assertEqual(self.start(wires=bytes(wires)), (-1, 0))
        time.sleep(.01); self.assertEqual(self.seen, [[], [], [], []])
        self.assertEqual(self.start(), (-1, 0)); self.close()
        for s in self.sessions:
            with self.assertRaisesRegex(RuntimeError, 'poisoned'):
                s.exchange([read_request(s.first_id, 'position')], timeout_ns=20_000_000)

    def test_bad_voltage_is_atomic_even_though_hold_triples_valid(self):
        self.create(); self.start_peers()
        v = bytearray(self.voltages()); v[-10] = 0x19
        self.assertEqual(self.start(volts=bytes(v)), (-1, 0))
        time.sleep(.01); self.assertEqual(self.seen, [[], [], [], []])

    def test_duplicate_mask_or_handle_rejected_before_owner_start(self):
        self.configure()
        self.create(masks=(C.c_uint32*4)(7, 7, 56, 7))
        self.assertFalse(self.group); self.assertIn(b'Overlapping', self.error.value)
        handles = (C.c_void_p*4)(*(s._handle for s in self.sessions))
        handles[3] = handles[0]; self.create(handles=handles)
        self.assertFalse(self.group)

    def test_gap_window_drift_rejected(self):
        self.configure(gap=890_000); self.create()
        self.assertFalse(self.group); self.assertIn(b'gap/window', self.error.value)

    def test_foreign_dso_and_unknown_pointer_rejected_before_dereference(self):
        self.configure()
        foreign = active.load_library(self.path, expected_sha256=sha(self.path))
        old = self.sessions[0]
        other = active.ActiveSession(foreign, old.fd, first_id=old.first_id,
            cancel_fd=self.pipes[0][0], boot_fd=self.boot.fileno(), boot_id=BOOT,
            raw_lower_by_id={old.first_id+i: old._limits.lower[i] for i in range(6)},
            raw_upper_by_id={old.first_id+i: old._limits.upper[i] for i in range(6)},
            kp_max_by_id={old.first_id+i: old._limits.kp[i] for i in range(6)},
            kd_max_by_id={old.first_id+i: old._limits.kd[i] for i in range(6)},
            gap_ns=900_000, window=3)
        try:
            handles = (C.c_void_p*4)(*(s._handle for s in self.sessions))
            handles[0] = other._handle; self.create(handles=handles)
            self.assertFalse(self.group); self.assertIn(b'this composite DSO', self.error.value)
            handles[0] = 0xdeadbeef; self.create(handles=handles)
            self.assertFalse(self.group); self.assertIn(b'this composite DSO', self.error.value)
        finally:
            other.close()

    def test_wrong_ticket_unordered_duplicate_waits_and_no_unvalidated_output(self):
        self.create(); self.start_peers()
        rc, ticket = self.start(); self.assertEqual(rc, 0)
        self.assertEqual(self.wait(ticket+1)[0], -1)
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1)
        self.assertEqual(self.wait(ticket)[0], 0)
        self.assertEqual(self.wait(ticket)[0], -1)
        self.assertEqual(self.wait(ticket, voltage=True)[0], 0)
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1)
        self.assertEqual(self.output(ticket, validated=0)[0], -1)
        self.assertEqual(self.output(ticket)[0], 0)
        self.assertEqual(self.output(ticket)[0], -1)

    def test_borrowed_session_ordinary_exchange_refused_then_stop_after_close(self):
        self.create(); self.start_peers()
        s = self.sessions[0]
        with self.assertRaisesRegex(RuntimeError, 'borrowed'):
            s.exchange([read_request(7, 'position')], timeout_ns=20_000_000)
        self.close()
        records = (Record*6)(); stats = active.Stats(); stopped = active.StopResult()
        rc = self.lib.sda_emergency_stop_subset(s._handle, 7, time.monotonic_ns()+200_000_000,
            records, C.byref(stats), C.byref(stopped), self.error, len(self.error))
        self.assertEqual(rc, 0, self.error.value); self.assertEqual(stopped.confirmed_mask, 7)

    def test_terminal_close_raw_native_poison_blocks_reactivation_but_all_stops_valid(self):
        self.create(); self.start_peers(); self.close()
        for s, ids, mask in zip(self.sessions, self.ids, self.masks):
            wire = type1(ids[0], kp=0., kd=0.)
            records = (Record*1)(); stats = active.Stats()
            rc = self.lib.sda_exchange(s._handle, (C.c_ubyte*17).from_buffer_copy(wire), 1, 0,
                time.monotonic_ns()+100_000_000, records, C.byref(stats), self.error, len(self.error))
            self.assertEqual(rc, -1); self.assertIn(b'poisoned', self.error.value)
            self.assertEqual(stats.writes, 0)
            stop_records = (Record*6)(); stop_stats = active.Stats(); stopped = active.StopResult()
            rc = self.lib.sda_emergency_stop_subset(s._handle, mask, time.monotonic_ns()+100_000_000,
                stop_records, C.byref(stop_stats), C.byref(stopped), self.error, len(self.error))
            self.assertEqual(rc, 0, self.error.value)
            self.assertEqual(stopped.confirmed_mask, mask)

    def test_cancel_and_close_during_wait_is_bounded_and_no_output(self):
        self.create(); self.start_peers(lambda *_: None)
        rc, ticket = self.start(); self.assertEqual(rc, 0)
        before = time.monotonic(); self.lib.sdg_cancel(self.group)
        self.assertEqual(self.wait(ticket)[0], -1)
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1)
        self.assertEqual(self.output(ticket)[0], -1)
        self.close(); self.assertLess(time.monotonic()-before, .5)

    def test_boot_change_before_publication_no_writes(self):
        self.create(); self.start_peers()
        self.boot.seek(0); self.boot.write(b'aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee\n'); self.boot.flush()
        self.assertEqual(self.start(), (-1, 0)); self.assertEqual(self.seen, [[], [], [], []])

    def test_fd_nonblocking_drift_before_publication_no_writes(self):
        self.create(); self.start_peers(); self.hosts[3].setblocking(True)
        self.assertEqual(self.start(), (-1, 0)); self.assertEqual(self.seen, [[], [], [], []])
        self.hosts[3].setblocking(False)

    def test_duplicate_reply_fault_and_deadline_prohibit_output(self):
        self.create()
        def duplicate(port, frame, count):
            raw = reply(frame.wire)
            return raw+raw if port == 0 and count == 1 else raw
        self.start_peers(duplicate)
        rc, ticket = self.start(); self.assertEqual(rc, 0)
        self.assertEqual(self.wait(ticket)[0], -1)
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1)
        self.assertEqual(self.output(ticket)[0], -1)

    def test_expired_output_write_cap_no_new_writes(self):
        self.create(); self.start_peers(); ticket, _, _ = self.successful_inputs()
        before = [len(v) for v in self.seen]
        self.assertEqual(self.output(ticket, write_cap=time.monotonic_ns()-1)[0], -1)
        time.sleep(.01); self.assertEqual(before, [len(v) for v in self.seen])

    def test_unreleased_output_after_bootstrap_correlates_ticket_without_fake_raw(self):
        self.create(); self.start_peers(); initial = Result()
        zero = b''.join(type1(mid, kp=0., kd=0.) for ids in self.ids for mid in ids)
        self.assertEqual(self.lib.sdg_initial_hold(self.group,
            (C.c_ubyte*len(zero)).from_buffer_copy(zero), time.monotonic_ns()+100_000_000,
            C.byref(initial), self.error, len(self.error)), 0)
        image = bytes(initial); ticket, _, _ = self.successful_inputs()
        before = [len(v) for v in self.seen]
        rc, failed = self.output(ticket, write_cap=time.monotonic_ns()-1)
        self.assertEqual(rc, -1); self.assertIn(b'Require now<write_deadline', self.error.value)
        self.assertEqual(failed.ticket, ticket); self.assertEqual(ticket, 1)
        self.assertGreater(failed.submitted_ns, 0); self.assertGreaterEqual(failed.collected_ns, failed.submitted_ns)
        self.assertEqual((failed.released_ns, failed.validated_ns), (0, 0))
        self.assertGreater(failed.cancel_requested_ns, 0)
        for row in failed.ports:
            self.assertEqual((row.status, row.count, row.owner_started_ns, row.owner_finished_ns), (-1, 0, 0, 0))
            self.assertEqual(bytes(row.records), bytes(3*C.sizeof(Record)))
            self.assertEqual(bytes(row.stats), bytes(C.sizeof(active.Stats)))
            self.assertIn(b'Require now<write_deadline', bytes(row.error))
        self.assertEqual(before, [len(v) for v in self.seen])
        bits, hold, voltage, output = self.snapshot()
        self.assertEqual(bits, 3); self.assertEqual((hold.ticket, voltage.ticket), (ticket, ticket))
        self.assertEqual(bytes(output), bytes(C.sizeof(Result)))
        self.assertEqual(bytes(initial), image); self.assertEqual(initial.ticket, 0)
        self.close()
        for s, mask in zip(self.sessions, self.masks):
            records = (Record*6)(); stats = active.Stats(); stopped = active.StopResult()
            self.assertEqual(self.lib.sda_emergency_stop_subset(s._handle, mask,
                time.monotonic_ns()+100_000_000, records, C.byref(stats), C.byref(stopped),
                self.error, len(self.error)), 0)
            self.assertEqual(stopped.confirmed_mask, mask)

    def test_initial_zero_hold_fresh_native_sample_and_only_once(self):
        self.create(); self.start_peers()
        zero_wires = b''.join(type1(mid, kp=0., kd=0.) for ids in self.ids for mid in ids)
        result = Result(); before = time.monotonic_ns()
        rc = self.lib.sdg_initial_hold(self.group,
            (C.c_ubyte*len(zero_wires)).from_buffer_copy(zero_wires), before+150_000_000,
            C.byref(result), self.error, len(self.error))
        self.assertEqual(rc, 0, self.error.value); self.assertEqual(result.ticket, 0)
        for row in result.ports:
            self.assertEqual((row.status, row.count), (0, 3))
            self.assertTrue(all(r.start_ns >= before and r.received == 17 for r in row.records))
        self.assertEqual(self.lib.sdg_initial_hold(self.group,
            (C.c_ubyte*len(zero_wires)).from_buffer_copy(zero_wires), time.monotonic_ns()+100_000_000,
            C.byref(result), self.error, len(self.error)), -1)
        ticket, _, _ = self.successful_inputs(); self.assertEqual(ticket, 1)
        self.assertEqual(self.output(ticket)[0], 0)

    def test_initial_hold_rejects_positive_gain_before_any_write(self):
        self.create(); self.start_peers(); result = Result(); wire = self.wires()
        self.assertEqual(self.lib.sdg_initial_hold(self.group,
            (C.c_ubyte*len(wire)).from_buffer_copy(wire), time.monotonic_ns()+100_000_000,
            C.byref(result), self.error, len(self.error)), -1)
        self.assertIn(b'zero only', self.error.value)
        time.sleep(.01); self.assertEqual(self.seen, [[], [], [], []])

    def test_output_write_cap_stays_earlier_than_reply_cap_on_every_write(self):
        self.create(); self.start_peers(); ticket, _, _ = self.successful_inputs()
        write_cap = time.monotonic_ns()+1_000_000
        rc, result = self.output(ticket, write_cap=write_cap,
            reply_cap=write_cap+50_000_000)
        self.assertEqual(rc, -1)
        for row in result.ports:
            for r in row.records:
                if r.written:
                    self.assertLess(r.start_ns, write_cap)
                    # A syscall may finish after its cap and is retained as a
                    # failed late write; it must never authorize another axis.
            written = [r for r in row.records if r.written]
            if any(r.finish_ns >= write_cap for r in written):
                self.assertEqual(written[-1].finish_ns, max(r.finish_ns for r in written))

    def test_external_cancel_before_publication_writes_nothing(self):
        self.create(); self.start_peers(); os.write(self.pipes[2][1], b'x')
        self.assertEqual(self.start(), (-1, 0)); self.assertEqual(self.seen, [[], [], [], []])

    def test_fault_reply_retains_raw_and_prevents_later_output(self):
        self.create()
        self.start_peers(lambda port, frame, count:
            reply(frame.wire, fault=1) if port == 1 and count == 1 else reply(frame.wire))
        rc, ticket = self.start(); self.assertEqual(rc, 0)
        rc, hold = self.wait(ticket); self.assertEqual(rc, -1)
        self.assertTrue(any(row.stats.rejected_total for row in hold.ports))
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1)
        self.assertEqual(self.output(ticket)[0], -1)

    def test_close_without_collecting_inflight_phase_joins_before_return(self):
        self.create(); self.start_peers(lambda *_: None)
        self.assertEqual(self.start()[0], 0)
        before = time.monotonic(); self.close(); self.assertLess(time.monotonic()-before, .5)

    def test_cancel_before_wait_exports_partial_native_raw_and_no_fake_output(self):
        self.create(); written = threading.Event()
        def drop(_port, _frame, _count):
            written.set(); return None
        self.start_peers(drop)
        rc, ticket = self.start(); self.assertEqual(rc, 0); self.assertTrue(written.wait(.2))
        bits, hold, voltage, output = self.snapshot()
        self.assertEqual(bits, 3); self.assertEqual((hold.ticket, voltage.ticket), (ticket, ticket))
        self.assertEqual(bytes(output), bytes(C.sizeof(Result)))
        self.assertGreater(sum(row.stats.writes for row in hold.ports), 0)
        self.assertTrue(any(r.written for row in hold.ports for r in row.records))
        self.assertTrue(all(row.owner_finished_ns for row in hold.ports))
        self.assertTrue(all(row.status == -1 for row in hold.ports))
        self.assertTrue(all(row.status == -1 for row in voltage.ports))
        self.assertEqual(self.output(ticket)[0], -1)

    def test_partial_voltage_snapshot_retains_consumed_hold_and_pending_raw(self):
        self.create(); release = threading.Event(); pending = threading.Event()
        def mutate(port, frame, _count):
            if port == 0 and frame.kind == 17:
                pending.set(); release.wait(.5)
            return reply(frame.wire)
        self.start_peers(mutate)
        try:
            rc, ticket = self.start(); self.assertEqual(rc, 0)
            rc, consumed = self.wait(ticket); self.assertEqual(rc, 0)
            self.assertTrue(pending.wait(.2))
            bits, hold, voltage, output = self.snapshot()
            self.assertEqual(bits, 3); self.assertEqual(bytes(output), bytes(C.sizeof(Result)))
            self.assertEqual(hold.collected_ns, consumed.collected_ns)
            for actual, original in zip(hold.ports, consumed.ports):
                self.assertEqual(bytes(actual), bytes(original))
            row = voltage.ports[0]
            self.assertEqual((row.count, row.stats.writes, row.records[0].written,
                row.records[0].received), (1, 1, 17, 0))
            self.assertEqual(bytes(row.records[0].tx), read_request(7, 'voltage'))
            self.assertEqual(row.status, -1)
        finally:
            release.set()

    def test_snapshot_phase_bits_distinguish_no_job_and_initial_zero_hold(self):
        self.create(); bits, hold, voltage, output = self.snapshot()
        self.assertEqual(bits, 0)
        self.assertEqual(bytes(hold)+bytes(voltage)+bytes(output), bytes(3*C.sizeof(Result)))
        self.close()

    def test_initial_snapshot_has_only_explicit_ticket_zero_phase(self):
        self.create(); self.start_peers(); result = Result()
        wire = b''.join(type1(mid, kp=0., kd=0.) for ids in self.ids for mid in ids)
        self.assertEqual(self.lib.sdg_initial_hold(self.group,
            (C.c_ubyte*len(wire)).from_buffer_copy(wire), time.monotonic_ns()+100_000_000,
            C.byref(result), self.error, len(self.error)), 0)
        bits, hold, voltage, output = self.snapshot()
        self.assertEqual(bits, 8); self.assertEqual(output.ticket, 0)
        self.assertEqual(bytes(hold)+bytes(voltage), bytes(2*C.sizeof(Result)))
        self.assertTrue(all(row.count == 3 and row.status == 0 for row in output.ports))

    def test_snapshot_complete_output_and_repeated_collection_preserves_images(self):
        self.create(); self.start_peers(); ticket, _, _ = self.successful_inputs()
        rc, consumed = self.output(ticket); self.assertEqual(rc, 0)
        bits, hold, voltage, output = self.snapshot(); self.assertEqual(bits, 7)
        self.assertEqual((hold.ticket, voltage.ticket, output.ticket), (ticket, ticket, ticket))
        for actual, original in zip(output.ports, consumed.ports):
            self.assertEqual(bytes(actual), bytes(original))
        later = self.snapshot()
        self.assertEqual(tuple(bytes(r) for r in later[1:]), (bytes(hold), bytes(voltage), bytes(output)))

    def test_validation_deadline_rejection_never_publishes_unreleased_job(self):
        # Isolated CPU-only source fixture forces the final native publication
        # boundary to equal the original deadline. It verifies the otherwise
        # scheduling-dependent branch and is never used as runtime/timing proof.
        result = build.subprocess.run([sys.executable, '-B', '-c',
            'from experiments.four_bus_native_group.test_native_group import '
            'validation_expiry_fixture; validation_expiry_fixture()'],
            capture_output=True, text=True, timeout=45, check=False)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_missing_type2_retains_ambiguity_across_repeated_terminal_stop(self):
        self.create()
        self.start_peers(lambda _p, frame, _n: None if frame.kind == 1 else reply(frame.wire))
        rc, ticket = self.start(deadline=time.monotonic_ns()+20_000_000)
        self.assertEqual(rc, 0); self.assertEqual(self.wait(ticket)[0], -1)
        self.assertEqual(self.wait(ticket, voltage=True)[0], -1); self.close()
        any_ambiguous = False
        for s, mask in zip(self.sessions, self.masks):
            for _attempt in range(2):
                records = (Record*6)(); stats = active.Stats(); stopped = active.StopResult()
                rc = self.lib.sda_emergency_stop_subset(s._handle, mask,
                    time.monotonic_ns()+100_000_000, records, C.byref(stats),
                    C.byref(stopped), self.error, len(self.error))
                if stopped.ambiguous_mask:
                    any_ambiguous = True; self.assertEqual(rc, 1)
                    self.assertNotEqual(stopped.confirmed_mask, mask)
        self.assertTrue(any_ambiguous)


def subprocess_module():
    return build.subprocess


def validation_expiry_fixture():
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp)/'deadline-fixture.cpp'
        marker = 'const uint64_t validated=now(),released=now();'
        original = build.EXTENSION.read_text()
        if original.count(marker) != 3:
            raise ValueError('Explicit publication boundary fixture no longer matches')
        source.write_text(original.replace(marker,
            'const uint64_t validated=deadline,released=deadline;', 1))
        with patch.object(build, 'EXTENSION', source):
            NativeGroupTests.setUpClass()
        case = NativeGroupTests(); case.setUp()
        # Compilation has its own outer allowance; only Session lifecycle is
        # bounded here, so slower target compilers cannot mimic a close hang.
        signal.setitimer(signal.ITIMER_REAL, 5.)
        try:
            case.create(); case.start_peers()
            case.assertEqual(case.start(), (-1, 0))
            case.assertEqual(case.seen, [[], [], [], []])
            case.close()  # Old ordering would deadlock here; child is bounded.
        finally:
            case.tearDown(); NativeGroupTests.tearDownClass()
            signal.setitimer(signal.ITIMER_REAL, 0.)


if __name__ == '__main__':
    unittest.main()
