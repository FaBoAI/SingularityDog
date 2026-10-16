"""Genuine C++ separate-write-cutoff checks over local sockets; no motor I/O."""
import ctypes as C
import json
import math
import os
from pathlib import Path
import select
import shutil
import tempfile
import threading
import time
import unittest

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from experiments.four_bus_diagnostic.transport_adapter import Group
from . import build
from . import type1_transport as T
from . import test_type1_transport as fixtures


class SeparateWriteDeadlineTests(unittest.TestCase):
    # Reuse only fixture methods, not their unrelated tests.
    setUp = fixtures.NativeType1TransportTests.setUp
    tearDown = fixtures.NativeType1TransportTests.tearDown
    transport = fixtures.NativeType1TransportTests.transport
    ready = fixtures.NativeType1TransportTests.ready
    peer_loop = fixtures.NativeType1TransportTests.peer_loop
    deadline = fixtures.NativeType1TransportTests.deadline
    cancel_all = fixtures.NativeType1TransportTests.cancel_all
    cancelled = fixtures.NativeType1TransportTests.cancelled
    assert_silent = fixtures.NativeType1TransportTests.assert_silent
    pins_for = staticmethod(fixtures.NativeType1TransportTests.pins_for)

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.library_path = build.build(Path(cls.directory.name) / "build")
        cls.lib = T.load_library(cls.library_path, **cls.pins_for(cls.library_path))
        # Private timing fixture: the first two commands use the real transport
        # untouched. Delay ONLY the third write boundary until the actual 20 ms
        # cutoff has passed, then let the existing cutoff check reject it. This
        # avoids depending on Python/peer scheduling inside an artificial 1.6 ms
        # window. It changes no production source or selected timing limit.
        directory = Path(cls.directory.name) / "third-write-boundary-fixture"
        directory.mkdir()
        source = build.ORDINARY.read_text()
        marker = "if(sent<count&&sent-done<effective_window&&t>=next) {"
        injected = marker + (
            "\n            if(write_deadline_ns&&sent==2) {"
            "const uint64_t injected_until=write_deadline_ns+1000000ULL;"
            "while(true) {const uint64_t current=now();if(current>=injected_until)break;"
            "const uint64_t remaining=injected_until-current;"
            "timespec injected_wait{time_t(remaining/1000000000ULL),long(remaining%1000000000ULL)};"
            "nanosleep(&injected_wait,nullptr);}}"
        )
        if marker not in source:
            raise AssertionError("Native third-write boundary fixture marker missing")
        ordinary = directory / "third_write_boundary.cpp"
        ordinary.write_text(source.replace(marker, injected, 1))
        cls.cutoff_library_path = build.build(directory / "build", ordinary_source=ordinary)
        cls.cutoff_lib = T.load_library(cls.cutoff_library_path, **cls.pins_for(cls.cutoff_library_path))

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def prepared(self, *, separate=True, lib=None):
        transport = self.ready(self.transport(lib=lib, separate_write_deadline=separate))
        wires = tuple(active.encode_motion(mid, .01, 3., .15) for mid in transport.group.ids)
        return transport, wires

    def direct(self, transport, wires, *, reply_deadline, write_deadline, mask=None, send_only=0):
        raw = (C.c_ubyte * (17 * len(wires))).from_buffer_copy(b"".join(wires))
        records = (Record * len(wires))()
        stats, error = active.Stats(), C.create_string_buffer(256)
        status = transport._library.sda_subset_exchange_write_deadline(
            transport._session._handle, transport.group.mask if mask is None else mask,
            raw, len(wires), send_only, reply_deadline, write_deadline,
            records, C.byref(stats), error, len(error))
        return status, error.value.decode(), records, stats

    def test_new_abi_receipt_and_exact_function_pins(self):
        scope = json.loads((self.library_path.parent / "build-record.json").read_bytes())["four_bus_subset_active"]
        self.assertEqual(scope["write_deadline_abi"], 1)
        self.assertEqual(self.lib.sda_subset_write_deadline_abi(), 1)
        self.assertEqual(self.lib.sda_subset_exchange_write_deadline.argtypes, list(T._WRITE_ARGS[1]))
        original = self.lib.sda_subset_exchange_write_deadline
        try:
            self.lib.sda_subset_exchange_write_deadline = self.lib["sda_subset_exchange_write_deadline"]
            with self.assertRaisesRegex(ValueError, "function binding changed"):
                T.verify_library(self.lib)
        finally:
            self.lib.sda_subset_exchange_write_deadline = original
        self.assertEqual(len(T.verify_library(self.lib).write_deadline), 2)

    def test_20ms_writes_can_receive_22ms_ack_inside_30hz_cycle(self):
        transport, wires = self.prepared()
        start = time.monotonic_ns()
        write_cap, reply_cap = start + 20_000_000, start + 33_333_334
        def response(wire):
            value = fixtures.ATParser().feed(wire)[0]
            if value.kind == 1 and value.destination == transport.group.ids[-1]:
                remaining = start + 22_000_000 - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1e9)
            return fixtures.reply(wire)
        self.peer_loop(6, mutate=response)
        batch = transport.output(wires, deadline_ns=reply_cap, write_deadline_ns=write_cap,
                                 check=lambda: None)
        self.assertEqual([record.written for record in batch.records], [17, 17, 17])
        self.assertEqual([record.received for record in batch.records], [17, 17, 17])
        self.assertTrue(all(record.start_ns <= record.finish_ns < write_cap for record in batch.records))
        self.assertGreater(batch.records[-1].received_ns, write_cap)
        self.assertLess(batch.records[-1].received_ns, reply_cap)
        self.assertTrue(all(record.deadline_ns == reply_cap for record in batch.records))
        proof = transport.write_deadline_journal[-1]
        self.assertEqual(proof[:3], (len(transport.journal) - 1, reply_cap, write_cap))
        self.assertLess(proof[3], write_cap)
        self.assertTrue(transport.stop_repeated()["complete"])
        self.thread.join(timeout=1)
        self.assertTrue(transport.poisoned)  # STOP permanently closes the active scope.

    def test_unsent_third_frame_is_rejected_at_write_cutoff_and_stop_still_runs(self):
        transport, wires = self.prepared(lib=self.cutoff_lib)
        # Hold the old Type1 replies until cleanup starts. An active cutoff
        # must retain their ambiguity even after all three STOP rounds reply
        # mode zero. Success must never depend on the peer winning an ACK race
        # before the cutoff. The private fixture delays the third native write
        # boundary; the real 20 ms input-to-write limit is not enlarged.
        self.assertFalse(self.thread.is_alive(), "Previous peer remains active")
        ready = threading.Event()
        def run_peer():
            parser = fixtures.ATParser()
            pending = []
            stops = 0
            ready.set()
            try:
                while stops < 9:
                    if not select.select([self.peer], [], [], 1)[0]:
                        return
                    raw = self.peer.recv(4096)
                    if not raw:
                        return
                    for value in parser.feed(raw):
                        self.seen.append(value)
                        if value.kind == 1:
                            pending.append(fixtures.reply(value.wire))
                            continue
                        if value.kind == 4:
                            if pending:
                                self.peer.sendall(b"".join(pending))
                                pending.clear()
                            self.peer.sendall(fixtures.reply(value.wire))
                            stops += 1
                        else:
                            raise AssertionError("Unexpected native request in cutoff/STOP fixture")
            except OSError:
                pass
            except BaseException as error:
                self.peer_error = error
        self.thread = threading.Thread(target=run_peer, name="cutoff-with-delayed-type1-peer")
        self.thread.start()
        self.assertTrue(ready.wait(timeout=1))
        start = time.monotonic_ns()
        write_cap, reply_cap = start + 20_000_000, start + 33_333_334
        with self.assertRaisesRegex(active.ExchangeError, "write deadline") as caught:
            transport.output(wires, deadline_ns=reply_cap, write_deadline_ns=write_cap, check=lambda: None)
        error = caught.exception
        self.assertEqual(error.records[0].written, 17)
        self.assertEqual(error.records[2].written, 0)
        self.assertLessEqual(error.stats.writes, 2)
        self.assertTrue(all(record.finish_ns < write_cap for record in error.records if record.written))
        self.assertEqual(transport.journal[-1][0], "output")
        self.assertIs(transport.journal[-1][1][0], error.records)
        self.assertEqual(transport.write_deadline_journal[-1][1:3], (reply_cap, write_cap))
        self.assertTrue(transport.poisoned)
        self.assertTrue(self.cancelled())
        ambiguous = {mid for mid, record in zip(transport.group.ids, error.records)
                     if record.written and not record.received}
        self.assertTrue(ambiguous)
        result = transport.stop_repeated()
        self.assertFalse(result["complete"], result)
        self.assertTrue(result["physical_cutoff_required"])
        self.assertEqual(set(result["ambiguous_ids"]), ambiguous)
        self.assertEqual(set(result["unconfirmed_ids"]), ambiguous)
        self.assertEqual(result["retry_policy"]["rounds_completed"], 3)
        self.assertEqual(len(result["rounds"]), 3)
        for row in result["rounds"]:
            self.assertEqual(set(row["attempted_ids"]), set(transport.group.ids))
            self.assertEqual(set(row["ambiguous_ids"]), ambiguous)
            self.assertEqual(set(row["confirmed_ids"]), set(transport.group.ids) - ambiguous)
        self.assertGreater(result["rounds"][0]["raw"]["rejected_total_bytes"], 0)
        self.thread.join(timeout=1)
        self.assertFalse(self.thread.is_alive(), "Three-round peer did not finish")
        self.assertTrue(all(value.destination in transport.group.ids for value in self.seen))
        self.assertEqual([value.kind for value in self.seen][-9:], [4] * 9)

    def test_bad_native_deadlines_reject_before_any_write(self):
        for case in ("zero", "expired", "after_reply"):
            transport = self.transport(separate_write_deadline=True)
            wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in transport.group.ids)
            now = time.monotonic_ns()
            write_cap = {"zero": 0, "expired": now - 1, "after_reply": now + 50_000_001}[case]
            status, message, records, stats = self.direct(
                transport, wires, reply_deadline=now + 50_000_000, write_deadline=write_cap)
            self.assertEqual(status, -1)
            self.assertIn("Separate deadline", message)
            self.assertEqual(stats.writes, 0)
            self.assertEqual([record.written for record in records], [0, 0, 0])
            self.assert_silent()
            transport.close()

    def test_native_new_entry_requires_exact_type1_triple_and_mask(self):
        cases = (
            (tuple(stop_wire(mid) for mid in (4, 5, 6)), 7, 0),
            ((active.encode_motion(4, 0., 0., 0.),), 56, 0),
            (tuple(active.encode_motion(mid, 0., 0., 0.) for mid in (4, 6, 5)), 56, 0),
            (tuple(active.encode_motion(mid, 0., 0., 0.) for mid in (4, 5, 6)), 7, 0),
            (tuple(active.encode_motion(mid, 0., 0., 0.) for mid in (4, 5, 6)), 56, 1),
        )
        for wires, mask, send_only in cases:
            transport = self.transport(separate_write_deadline=True)
            now = time.monotonic_ns()
            status, _, records, stats = self.direct(transport, wires, mask=mask, send_only=send_only,
                reply_deadline=now + 33_333_334, write_deadline=now + 20_000_000)
            self.assertEqual(status, -1)
            self.assertEqual(stats.writes, 0)
            self.assertTrue(all(record.written == 0 for record in records))
            self.assert_silent()
            transport.close()

    def test_missing_ack_preserves_raw_failure_and_stop_ambiguity(self):
        transport, wires = self.prepared()
        missing = transport.group.ids[-1]
        def response(wire):
            value = fixtures.ATParser().feed(wire)[0]
            return None if value.kind == 1 and value.destination == missing else fixtures.reply(wire)
        self.peer_loop(6, mutate=response)
        now = time.monotonic_ns()
        with self.assertRaisesRegex(active.ExchangeError, "deadline") as caught:
            transport.output(wires, deadline_ns=now + 33_333_334,
                             write_deadline_ns=now + 20_000_000, check=lambda: None)
        self.assertEqual([record.written for record in caught.exception.records], [17] * 3)
        self.assertEqual(caught.exception.records[-1].received, 0)
        result = transport.stop_repeated(rounds=1)
        self.assertFalse(result["complete"])
        self.assertIn(missing, result["ambiguous_ids"])
        self.assertTrue(result["physical_cutoff_required"])
        self.thread.join(timeout=1)

    def test_python_invalid_cutoffs_and_missing_selection_never_write(self):
        for value in (True, False, math.nan, math.inf, "20ms", 0, -1):
            transport, wires = self.prepared()
            with self.assertRaisesRegex(ValueError, "Separate Type1 deadline"):
                transport.output(wires, deadline_ns=self.deadline(33), write_deadline_ns=value,
                                 check=lambda: None)
            self.assert_silent()
            transport.close()
            # Each intentionally aborted Python call broadcasts cancellation;
            # clear that local test pipe before the next isolated handshake.
            fixtures.os.read(self.cancel_read, 4096)
        for selected, cutoff in ((True, None), (False, self.deadline(20))):
            transport, wires = self.prepared(separate=selected)
            with self.assertRaisesRegex(ValueError, "supplied exactly when selected"):
                transport.output(wires, deadline_ns=self.deadline(33), write_deadline_ns=cutoff,
                                 check=lambda: None)
            self.assert_silent()
            transport.close()
            fixtures.os.read(self.cancel_read, 4096)

    def test_unrecorded_capability_cannot_open_a_30hz_session(self):
        directory = Path(self.directory.name) / f"old-receipt-{time.monotonic_ns()}"
        shutil.copytree(self.library_path.parent, directory)
        receipt = directory / "build-record.json"
        record = json.loads(receipt.read_bytes())
        del record["four_bus_subset_active"]["write_deadline_abi"]
        receipt.write_text(json.dumps(record))
        library = directory / build.LIBRARY_NAME
        lib = T.load_library(library, **self.pins_for(library))
        self.assertEqual(T.verify_library(lib).write_deadline, ())
        with self.assertRaisesRegex(ValueError, "receipt and ABI"):
            self.transport(lib=lib, separate_write_deadline=True)
        self.assertEqual(self.transports, [])
        self.assert_silent()
        # The original path remains usable with that same old receipt.
        transport = self.transport(lib=lib)
        self.peer_loop(3)
        self.assertEqual(set(transport.stop(deadline_ns=self.deadline())), set(transport.group.ids))
        self.thread.join(timeout=1)

    def test_selected_capability_cannot_be_changed_after_creation(self):
        transport, wires = self.prepared()
        transport.separate_write_deadline = False
        with self.assertRaisesRegex(ValueError, "binding changed"):
            transport.validate_output(wires)
        self.assert_silent()

    def test_cancel_before_new_exchange_writes_nothing_and_cleanup_stop_is_available(self):
        transport, wires = self.prepared()
        os.write(self.cancel_write, b"x")
        now = time.monotonic_ns()
        with self.assertRaisesRegex(active.ExchangeError, "Cancelled") as caught:
            transport.output(wires, deadline_ns=now + 33_333_334,
                             write_deadline_ns=now + 20_000_000, check=lambda: None)
        self.assertEqual(caught.exception.stats.writes, 0)
        self.assertEqual([record.written for record in caught.exception.records], [0] * 3)
        self.assert_silent()
        self.peer_loop(3)
        self.assertTrue(transport.stop_repeated(rounds=1)["complete"])
        self.thread.join(timeout=1)

    def test_changed_boot_is_still_rejected_before_new_exchange_writes(self):
        transport, wires = self.prepared()
        self.boot.seek(0)
        self.boot.write(b"99999999-2222-3333-4444-555555555555\n")
        self.boot.flush()
        now = time.monotonic_ns()
        with self.assertRaisesRegex(active.ExchangeError, "Boot identity") as caught:
            transport.output(wires, deadline_ns=now + 33_333_334,
                             write_deadline_ns=now + 20_000_000, check=lambda: None)
        self.assertEqual(caught.exception.stats.writes, 0)
        self.assert_silent()

    def test_legacy_output_remains_single_deadline_and_has_no_new_proof(self):
        transport, wires = self.prepared(separate=False)
        self.peer_loop(6)
        batch = transport.output(wires, deadline_ns=self.deadline(20), check=lambda: None)
        self.assertEqual([record.received for record in batch.records], [17] * 3)
        self.assertEqual(transport.write_deadline_journal, [])
        self.assertTrue(transport.stop_repeated(rounds=1)["complete"])
        self.thread.join(timeout=1)

    def test_new_deadline_cannot_be_used_for_other_exchange_labels(self):
        transport = self.transport(separate_write_deadline=True)
        now = time.monotonic_ns()
        with self.assertRaisesRegex(ValueError, "Only the exact output triple"):
            transport._exchange("stop", transport._wires["stop"], now + 33_333_334,
                                write_deadline_ns=now + 20_000_000)
        self.assert_silent()
        self.assertFalse(transport.attempts["type1_sent"])

    def test_bool_receipt_capability_is_not_an_integer_abi(self):
        directory = Path(self.directory.name) / f"bool-receipt-{time.monotonic_ns()}"
        shutil.copytree(self.library_path.parent, directory)
        receipt = directory / "build-record.json"
        record = json.loads(receipt.read_bytes())
        record["four_bus_subset_active"]["write_deadline_abi"] = True
        receipt.write_text(json.dumps(record))
        self.assertIsNotNone(build.receipt_problem(directory))
        with self.assertRaisesRegex(ValueError, "receipt pins differ"):
            T.load_library(directory / build.LIBRARY_NAME,
                           **self.pins_for(directory / build.LIBRARY_NAME))

    def test_post_write_crossing_is_recorded_as_failure_and_poisoned(self):
        # Private source fixture injects scheduler delay only AFTER the actual
        # first write syscall. This exercises the cutoff's after-write boundary,
        # rather than merely letting the next write be rejected in the loop.
        directory = Path(self.directory.name) / f"delayed-write-{time.monotonic_ns()}"
        directory.mkdir()
        source = build.ORDINARY.read_text()
        original = "const ssize_t n=write(s->fd,r.tx,17);r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;"
        injected = ("const ssize_t n=write(s->fd,r.tx,17);"
                    "timespec injected_delay{0,12000000};nanosleep(&injected_delay,nullptr);"
                    "r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;")
        self.assertIn(original, source)
        ordinary = directory / "delayed_ordinary.cpp"
        ordinary.write_text(source.replace(original, injected, 1))
        library = build.build(directory / "build", ordinary_source=ordinary)
        lib = T.load_library(library, **self.pins_for(library))
        transport = self.transport(lib=lib, separate_write_deadline=True)
        wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in transport.group.ids)
        self.peer_loop(1)
        now = time.monotonic_ns()
        status, message, records, stats = self.direct(transport, wires,
            reply_deadline=now + 100_000_000, write_deadline=now + 8_000_000)
        self.assertEqual(status, -1)
        self.assertIn("Late active write exceeded separate write deadline", message)
        self.assertEqual(stats.writes, 1)
        self.assertEqual(records[0].written, 17)
        self.assertLess(records[0].start_ns, now + 8_000_000)
        self.assertGreaterEqual(records[0].finish_ns, now + 8_000_000)
        self.assertEqual([records[i].written for i in (1, 2)], [0, 0])
        self.thread.join(timeout=1)
        # Poison lives in the native session even for this ABI-level call.
        later = time.monotonic_ns()
        status, message, _, stats = self.direct(transport, wires,
            reply_deadline=later + 100_000_000, write_deadline=later + 20_000_000)
        self.assertEqual(status, -1)
        self.assertIn("poisoned", message)
        self.assertEqual(stats.writes, 0)


if __name__ == "__main__":
    unittest.main()
