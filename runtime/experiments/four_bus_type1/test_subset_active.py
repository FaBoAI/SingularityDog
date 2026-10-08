"""Genuine C++ mask-bound Type1 exchange over anonymous sockets; no hardware."""
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import socket
import struct
import tempfile
import threading
import time
import unittest

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.motor_version_probe import version_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from experiments.four_bus_diagnostic import build as stop_build
from experiments.four_bus_diagnostic import transport_adapter
from . import build

BOOT = '11111111-2222-3333-4444-555555555555'
LSB = 25.14/65535.
SUBSET_ARGS = (C.c_void_p, C.c_uint32, C.c_uint64, C.POINTER(Record), C.POINTER(active.Stats),
               C.POINTER(active.StopResult), C.POINTER(C.c_char), C.c_uint32)
VALIDATE_ARGS = (C.c_void_p, C.c_uint32, C.POINTER(C.c_ubyte), C.c_uint32,
                 C.POINTER(C.c_char), C.c_uint32)
EXCHANGE_ARGS = (C.c_void_p, C.c_uint32, C.POINTER(C.c_ubyte), C.c_uint32, C.c_int, C.c_uint64,
                 C.POINTER(Record), C.POINTER(active.Stats), C.POINTER(C.c_char), C.c_uint32)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def wire(kind, mid, data=bytes(8), middle=0xfd):
    return frame((kind << 24)|(middle << 8)|mid, data)


def type1(mid, q=0., kp=1., kd=.1):
    return active.encode_motion(mid, q, kp, kd)


def watchdog(mid):
    return wire(18, mid, bytes((0x28, 0x70, 0, 0, 0xa0, 0x0f, 0, 0)))


def reply(raw, *, type1_mode=2, fault=0):
    value = ATParser().feed(raw)[0]; mid = value.destination
    if value.kind == 0:
        return frame((mid << 8)|0xfe, bytes(range(1, 9)))
    if value.kind == 17:
        return frame((17 << 24)|(mid << 8)|0xfd, value.data[:4]+struct.pack('<f', 40.))
    if value.kind == 4 and value.data[:2] == b'\x00\xc4':
        return frame((2 << 24)|(mid << 8)|0xfd, b'\x00\xc4\x56'+bytes(5))
    mode = {1: type1_mode, 3: 2}.get(value.kind, 0)
    return frame((2 << 24)|(mode << 22)|(fault << 16)|(mid << 8)|0xfd,
                 struct.pack('>4H', 32767, 32767, 32767, 250))


def unreachable(code=32767):
    base = code*25.14/65535.-12.57
    return base+LSB/3, base+2*LSB/3


class SubsetActiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        selected = os.environ.get('FOUR_BUS_TYPE1_TEST_LIBRARY')
        cls.library_path = (Path(selected).resolve() if selected else
                            build.build(Path(cls.directory.name)/'build'))
        problem = build.receipt_problem(cls.library_path.parent)
        if problem: raise ValueError(problem)
        lib = active.load_library(cls.library_path, expected_sha256=sha(cls.library_path))
        for name, arguments, result in (('sda_subset_active_abi', (), C.c_uint32),
                ('sda_emergency_stop_subset_abi', (), C.c_uint32),
                ('sda_subset_validate', VALIDATE_ARGS, C.c_int),
                ('sda_subset_exchange', EXCHANGE_ARGS, C.c_int),
                ('sda_emergency_stop_subset', SUBSET_ARGS, C.c_int)):
            function = lib[name]; function.argtypes = list(arguments); function.restype = result
            setattr(lib, name, function)
        cls.lib = lib

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.host, self.peer = socket.socketpair(); self.host.setblocking(False)
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile(); self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.sessions = []; self.thread = None; self.seen = []; self.peer_error = None

    def tearDown(self):
        for session in self.sessions: session.close()
        self.host.close(); self.peer.close()
        if self.thread: self.thread.join(timeout=1)
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()
        if self.peer_error: raise self.peer_error

    def session(self, first_id=1, mask=0x07):
        for session in self.sessions: session.close()
        self.sessions = []
        lower, upper, kp, kd = {}, {}, {}, {}
        for slot in range(6):
            mid = first_id+slot
            if mask & (1 << slot):
                lower[mid], upper[mid], kp[mid], kd[mid] = -.5, .5, 3., .15
            else:
                (lower[mid], upper[mid]), kp[mid], kd[mid] = unreachable(), 0., 0.
        result = active.ActiveSession(self.lib, self.host.fileno(), first_id=first_id,
            cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
            raw_lower_by_id=lower, raw_upper_by_id=upper, kp_max_by_id=kp, kd_max_by_id=kd,
            gap_ns=900_000, window=3)
        self.sessions.append(result); return result

    def peer_loop(self, count, *, mutate=reply):
        if self.thread:
            self.thread.join(timeout=1)
            self.assertFalse(self.thread.is_alive(), 'Previous synthetic peer remains active')
        ready = threading.Event()
        def run():
            parser = ATParser(); seen = 0; ready.set()
            try:
                while seen < count:
                    if not select.select([self.peer], [], [], 1)[0]: return
                    raw = self.peer.recv(4096)
                    if not raw: return
                    for value in parser.feed(raw):
                        self.seen.append(value); seen += 1
                        response = mutate(value.wire)
                        if response: self.peer.sendall(response)
            except OSError:
                pass
            except BaseException as error:
                self.peer_error = error
        self.thread = threading.Thread(target=run, name='four-type1-synthetic-peer')
        self.thread.start(); self.assertTrue(ready.wait(timeout=1))

    def raw(self, wires):
        joined = b''.join(wires)
        return (C.c_ubyte*max(1, len(joined))).from_buffer_copy(joined or b'\0')

    def exchange(self, session, mask, wires, *, count=None, deadline_ns=None, records=None):
        records = (Record*6)() if records is None else records
        stats, error = active.Stats(), C.create_string_buffer(256)
        deadline = time.monotonic_ns()+50_000_000 if deadline_ns is None else deadline_ns
        status = self.lib.sda_subset_exchange(session._handle, mask, self.raw(wires),
            len(wires) if count is None else count, 0, deadline, records, C.byref(stats), error, len(error))
        return status, records, stats, error.value.decode()

    def validate(self, session, mask, wires):
        error = C.create_string_buffer(256)
        status = self.lib.sda_subset_validate(session._handle, mask, self.raw(wires), len(wires),
                                              error, len(error))
        return status, error.value.decode()

    def stop_subset(self, session, mask, timeout_ns=90_000_000):
        records, stats, result, error = (Record*6)(), active.Stats(), active.StopResult(), C.create_string_buffer(256)
        status = self.lib.sda_emergency_stop_subset(session._handle, mask, time.monotonic_ns()+timeout_ns,
            records, C.byref(stats), C.byref(result), error, len(error))
        return status, records, result

    def assert_poisoned(self, session, mask):
        valid = [type1(session.first_id+slot) for slot in range(6) if mask & (1 << slot)]
        self.assertEqual(self.validate(session, mask, valid), (-1, 'Session poisoned; active retry prohibited'))

    def assert_silent(self):
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def rejected(self, first_id, mask, wires, pattern, *, count=None):
        session = self.session(first_id, mask)
        records = (Record*6)(); records[0].written = 123
        status, records, stats, error = self.exchange(session, mask, wires, count=count, records=records)
        self.assertEqual(status, -1); self.assertRegex(error, pattern)
        self.assertEqual(stats.writes, 0); self.assertEqual(records[0].written, 123)
        self.assertTrue(stats.begin_ns and stats.end_ns >= stats.begin_ns)
        self.assert_silent(); self.assert_poisoned(session, mask)
        fresh = self.session(first_id, mask)
        if count is None:
            self.assertEqual(self.validate(fresh, mask, wires)[0], -1)
            self.assert_poisoned(fresh, mask)
        self.assert_silent()

    def test_abi_and_receipt_scope_cannot_be_swapped_with_stop_only(self):
        self.assertEqual(self.lib.sda_subset_active_abi(), 1)
        self.assertEqual(self.lib.sda_emergency_stop_subset_abi(), 1)
        directory = self.library_path.parent
        record = json.loads((directory/'build-record.json').read_bytes())
        scope = record['four_bus_subset_active']
        self.assertNotIn('four_bus_subset_stop', record)
        self.assertEqual((scope['scope'], scope['allowed_masks'], scope['type1_exchange_abi'],
                          scope['output_allowed'], scope['allowed_kinds']),
                         ('four_bus_subset_active.v1', [7, 56], 1, False, [0, 1, 3, 4, 17, 18]))
        with self.assertRaisesRegex(ValueError, 'pins required'):
            transport_adapter.load_library(self.library_path, expected_sha256=sha(self.library_path),
                ordinary_source_sha256=scope['ordinary_source_sha256'],
                extension_source_sha256=scope['extension_source_sha256'],
                build_record_sha256=sha(directory/'build-record.json'))
        stop_library = stop_build.build(Path(self.directory.name)/f'stop-{time.monotonic_ns()}')
        self.assertRegex(build.receipt_problem(stop_library.parent), 'STOP-only receipt rejected')
        forged = Path(self.directory.name)/f'forged-{time.monotonic_ns()}'
        shutil.copytree(directory, forged)
        changed = dict(record, four_bus_subset_stop={'schema': 'singularitydog.four-bus-subset-build.v1'})
        (forged/'build-record.json').write_text(json.dumps(changed))
        self.assertRegex(build.receipt_problem(forged), 'STOP-only receipt rejected')
        (forged/'build-record.json').write_text(json.dumps(record))
        self.assertIsNone(build.receipt_problem(forged))
        (forged/'subset_stop.cpp').write_bytes((directory/'subset_stop.cpp').read_bytes()+b'\n')
        self.assertRegex(build.receipt_problem(forged), 'included source differs')

    def test_masks_other_than_7_and_56_rejected_without_write(self):
        for mask in (0, 1, 3, 15, 0x39, 63, 0xffffffff):
            with self.subTest(mask=mask):
                session = self.session(1, 0x07)
                records = (Record*6)(); records[0].written = 123
                status, records, stats, error = self.exchange(session, mask, [stop_wire(1)], records=records)
                self.assertEqual((status, stats.writes, records[0].written), (-1, 0, 123))
                self.assertRegex(error, 'mask required'); self.assert_silent()
                self.assert_poisoned(session, 0x07)
                session = self.session(1, 0x07)
                self.assertEqual(self.validate(session, mask, [stop_wire(1)]),
                                 (-1, 'Explicit three-axis half-envelope mask required'))
                self.assert_poisoned(session, 0x07)

    def test_count_bounds(self):
        self.rejected(1, 0x07, [stop_wire(1)], '1..6 wires', count=0)
        self.rejected(1, 0x07, [stop_wire(1), stop_wire(2), stop_wire(3)]*2+[stop_wire(1)], '1..6 wires')

    def test_foreign_id_type1_type3_stop_rejected_with_no_bytes_written(self):
        for first_id, mask, mid in ((1, 0x07, 4), (1, 0x07, 6), (1, 0x38, 1), (7, 0x07, 10),
                                     (7, 0x38, 9), (1, 0x07, 7), (7, 0x38, 1)):
            for value in (type1(mid), wire(3, mid), stop_wire(mid), read_request(mid, 'voltage')):
                with self.subTest(first_id=first_id, mask=mask, mid=mid, kind=value[2] >> 3):
                    self.rejected(first_id, mask, [value], "outside this port's three-axis mask")
        # A foreign ID anywhere in a batch rejects the whole batch.
        self.rejected(1, 0x07, [stop_wire(1), stop_wire(2), stop_wire(4)], 'outside')

    def test_disallowed_kinds_rejected(self):
        for kind in (2, 5, 6, 7, 21, 31):
            with self.subTest(kind=kind):
                self.rejected(1, 0x07, [wire(kind, 2)], 'Disallowed subset active kind')

    def test_type1_batches_must_be_pure_single_or_exact_ascending_triple(self):
        self.rejected(1, 0x07, [type1(1), stop_wire(2)], 'only Type1')
        self.rejected(1, 0x07, [stop_wire(1), type1(2)], 'only Type1')
        self.rejected(1, 0x07, [type1(1), type1(2), read_request(3, 'voltage')], 'only Type1')
        self.rejected(7, 0x38, [type1(10), type1(11)], 'one axis or the exact masked three')
        self.rejected(1, 0x07, [type1(2), type1(1), type1(3)], 'ascending ID order')
        self.rejected(1, 0x07, [type1(1), type1(3), type1(2)], 'ascending ID order')
        self.rejected(7, 0x38, [type1(12), type1(11), type1(10)], 'ascending ID order')
        self.rejected(1, 0x07, [type1(1), type1(1), type1(2)], 'Duplicate')

    def test_type1_caps_and_window_rejected(self):
        for value in (type1(2, kp=3.2), type1(2, kd=.2), type1(2, q=.6), type1(2, q=-.6)):
            with self.subTest(value=value.hex()):
                self.rejected(1, 0x07, [value], 'out-of-bounds')
        self.rejected(1, 0x07, [type1(1), type1(2, kp=3.2), type1(3)], 'out-of-bounds')
        noncanonical = bytearray(type1(2)); noncanonical[9:11] = b'\x00\x00'
        self.rejected(1, 0x07, [bytes(noncanonical)], 'out-of-bounds')

    def test_nonmember_slot_window_is_unreachable_by_any_code(self):
        lo, hi = unreachable()
        self.assertTrue(-12.57 < lo < hi < 12.57)
        decoded = [code*25.14/65535.-12.57 for code in range(65536)]
        self.assertFalse(any(lo <= q <= hi for q in decoded))

    def test_validate_is_static_and_accepts_valid_batches_without_io(self):
        session = self.session(7, 0x38)
        for wires in ([type1(10), type1(11), type1(12)], [type1(11, kp=0., kd=0.)],
                      [stop_wire(10), stop_wire(11), stop_wire(12)], [wire(0, 12)], [wire(3, 10)],
                      [watchdog(11)], [read_request(12, 'voltage')], [version_request(10)],
                      [read_request(mid, 'voltage') for mid in (10, 11, 12)]):
            self.assertEqual(self.validate(session, 0x38, wires), (0, ''))
        self.assert_silent()
        self.assertEqual(self.validate(session, 0x38, [type1(10), type1(11)])[0], -1)
        self.assert_poisoned(session, 0x38)

    def check_triple(self, first_id, mask, ids):
        session = self.session(first_id, mask); self.peer_loop(3)
        status, records, stats, error = self.exchange(session, mask, [type1(mid, q=.1*i) for i, mid in enumerate(ids)])
        self.assertEqual(status, 0, error); self.assertEqual(stats.writes, 3)
        self.assertEqual([value.destination for value in self.seen], list(ids))
        self.assertEqual([value.kind for value in self.seen], [1, 1, 1])
        for index in range(3):
            self.assertEqual((records[index].written, records[index].received), (17, 17))
            self.assertEqual((records[index].rx[2] << 24 | records[index].rx[3] << 16) >> 25 & 3, 2)
        self.assertEqual(records[3].written, 0)
        return session

    def test_valid_triple_accepted_with_mode2_replies_both_masks(self):
        for first_id, mask, ids in ((1, 0x07, (1, 2, 3)), (1, 0x38, (4, 5, 6)),
                                     (7, 0x07, (7, 8, 9)), (7, 0x38, (10, 11, 12))):
            with self.subTest(ids=ids):
                self.seen = []
                session = self.check_triple(first_id, mask, ids)
                self.assertEqual(self.validate(session, mask, [type1(ids[0])]), (0, ''))
                self.thread.join(timeout=1)

    def test_allowed_handshake_kinds_exchange_and_single_type1(self):
        session = self.session(1, 0x38)
        steps = ([wire(0, mid) for mid in (4, 5, 6)], [stop_wire(mid) for mid in (4, 5, 6)],
                 [version_request(5)], [read_request(mid, 'voltage') for mid in (4, 5, 6)],
                 [watchdog(4)], [read_request(4, 'can_timeout')], [wire(3, 6)],
                 [type1(6, q=.2, kp=0., kd=0.)])
        self.peer_loop(sum(len(step) for step in steps))
        for wires in steps:
            status, records, stats, error = self.exchange(session, 0x38, wires)
            self.assertEqual(status, 0, (error, [value.hex() for value in wires]))
            self.assertEqual(stats.writes, len(wires))
        self.assertEqual([value.wire for value in self.seen], [value for step in steps for value in step])

    def test_mode0_reply_to_type1_rejected_and_leaves_ambiguity(self):
        session = self.session(1, 0x07)
        self.peer_loop(3, mutate=lambda raw: reply(raw, type1_mode=0 if ATParser().feed(raw)[0].destination == 3 else 2))
        status, records, stats, error = self.exchange(session, 0x07, [type1(1), type1(2), type1(3)])
        self.assertEqual(status, -1); self.assertRegex(error, 'Unmatched/duplicate/fault/mode')
        self.assertEqual(stats.writes, 3); self.assertEqual(records[2].received, 0)
        self.assert_poisoned(session, 0x07)
        self.thread.join(timeout=1); self.peer_loop(3)
        status, records, result = self.stop_subset(session, 0x07)
        self.assertEqual((status, result.attempted_mask), (1, 7))
        self.assertEqual((result.confirmed_mask, result.ambiguous_mask), (3, 4))

    def test_fault_reply_to_type1_rejected(self):
        session = self.session(7, 0x07)
        self.peer_loop(3, mutate=lambda raw: reply(raw, fault=1))
        status, _, _, error = self.exchange(session, 0x07, [type1(7), type1(8), type1(9)])
        self.assertEqual(status, -1); self.assertRegex(error, 'fault')

    def test_subset_stop_after_pending_type1_reports_it_ambiguous(self):
        session = self.session(7, 0x38)
        self.peer_loop(3, mutate=lambda raw: None if ATParser().feed(raw)[0].destination == 11 else reply(raw))
        status, records, stats, error = self.exchange(session, 0x38, [type1(10), type1(11), type1(12)],
                                                      deadline_ns=time.monotonic_ns()+20_000_000)
        self.assertEqual(status, -1); self.assertRegex(error, 'deadline')
        self.assertEqual((records[1].written, records[1].received), (17, 0))
        self.thread.join(timeout=1); self.peer_loop(3)
        status, records, result = self.stop_subset(session, 0x38)
        self.assertEqual(status, 1)
        self.assertEqual(result.attempted_mask, 0x38)
        self.assertEqual(result.ambiguous_mask, 1 << 4)
        self.assertEqual(result.confirmed_mask, (1 << 3)|(1 << 5))
        self.assertEqual([value.kind for value in self.seen], [1, 1, 1, 4, 4, 4])
        self.assert_poisoned(session, 0x38)

    def test_cancel_is_rechecked_by_ordinary_exchange_without_write(self):
        session = self.session(1, 0x07); os.write(self.cancel_write, b'x')
        status, _, stats, error = self.exchange(session, 0x07, [type1(1), type1(2), type1(3)])
        self.assertEqual(status, -1); self.assertRegex(error, 'Cancelled')
        self.assertEqual(stats.writes, 0); self.assert_silent(); self.assert_poisoned(session, 0x07)

    def test_poisoned_session_refuses_valid_batch_without_write(self):
        session = self.session(1, 0x07)
        self.assertEqual(self.exchange(session, 0x07, [type1(4)])[0], -1)
        status, _, stats, error = self.exchange(session, 0x07, [stop_wire(1), stop_wire(2), stop_wire(3)])
        self.assertEqual((status, stats.writes), (-1, 0)); self.assertRegex(error, 'poisoned')
        self.assert_silent()

    def test_existing_stop_only_exports_still_work(self):
        session = self.session(1, 0x07); self.peer_loop(3)
        records, stats, error = (Record*3)(), active.Stats(), C.create_string_buffer(256)
        raw = self.raw([stop_wire(mid) for mid in (1, 2, 3)])
        status = self.lib.sda_exchange(session._handle, raw, 3, 0, time.monotonic_ns()+50_000_000,
                                       records, C.byref(stats), error, len(error))
        self.assertEqual(status, 0, error.value)
        self.thread.join(timeout=1); self.peer_loop(3)
        status, records, result = self.stop_subset(session, 0x07)
        self.assertEqual((status, result.attempted_mask, result.confirmed_mask, result.ambiguous_mask), (0, 7, 7, 0))
        self.thread.join(timeout=1); self.peer_loop(6)
        result = session.emergency_stop(timeout_ns=250_000_000)
        self.assertTrue(result['complete'], result)
        self.assertEqual(result['confirmed_ids'], [1, 2, 3, 4, 5, 6])
        self.assertEqual(self.lib.sda_feedback_decode_abi(), 1)
        self.assertEqual([value.kind for value in self.seen], [4]*12)


if __name__ == '__main__': unittest.main()
