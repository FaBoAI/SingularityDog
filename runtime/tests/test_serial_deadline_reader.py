"""POSIX PTY and deterministic fault tests for deadline-based receive only."""
import errno
from dataclasses import FrozenInstanceError
import os
import select
import threading
import time
import tty
import unittest
from unittest.mock import patch

from singularitydog_hw.serial_deadline_reader import (DeadlineSerialReader, ReceivedChunk,
                                                     record_failed_read)

MODULE = "singularitydog_hw.serial_deadline_reader"


class SerialShim:
    def __init__(self, fd):
        self.fd = fd
        self.timeout_settings = []

    @property
    def timeout(self):
        return self.timeout_settings[-1] if self.timeout_settings else .003

    @timeout.setter
    def timeout(self, value):
        self.timeout_settings.append(value)

    def fileno(self):
        return self.fd


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now


@unittest.skipUnless(os.name == "posix", "POSIX fd/select implementation")
class DeadlineSerialReaderTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.slave, False)
        self.raw = SerialShim(self.slave)

    def tearDown(self):
        for fd in (self.master, self.slave):
            if fd is not None:
                try: os.close(fd)
                except OSError: pass

    def test_real_pty_partial_chunks_and_configuration_only_once(self):
        reader = DeadlineSerialReader(self.raw)
        os.write(self.master, b"AT")
        start = time.monotonic_ns()
        first, received = reader.read_until(start+100_000_000, start+300_000_000)
        self.assertEqual(first, b"AT")
        self.assertGreaterEqual(received, start)
        os.write(self.master, b"payload\r\n")
        second, _ = reader.read_until(time.monotonic_ns()+100_000_000, start+300_000_000)
        self.assertEqual(second, b"payload\r\n")
        self.assertEqual(self.raw.timeout_settings, [0])
        self.assertFalse(os.get_blocking(self.slave))
        self.assertEqual(reader.stats()["bytes_received"], 11)
        self.assertIsNone(reader.last_failed_read)
        snapshot = reader.stats()
        snapshot["bytes_received"] = -1
        self.assertEqual(reader.stats()["bytes_received"], 11)
        del reader
        self.assertGreaterEqual(os.fstat(self.slave).st_ino, 0)  # caller owns fd

    def test_real_pty_arrival_during_wait(self):
        reader = DeadlineSerialReader(self.raw)
        def deliver():
            time.sleep(.005)
            os.write(self.master, b"late")
        worker = threading.Thread(target=deliver)
        worker.start()
        try:
            start = time.monotonic_ns()
            chunk, stamp = reader.read_until(start+500_000_000, start+1_000_000_000)
        finally:
            worker.join(1)
        self.assertEqual(chunk, b"late")
        self.assertGreater(stamp, start)
        self.assertGreater(reader.stats()["select_wait_ns"], 0)

    def test_soft_expiry_and_zero_wait_ready_drain(self):
        reader = DeadlineSerialReader(self.raw)
        start = time.monotonic_ns()
        self.assertEqual(reader.read_until(start+1_000_000, start+100_000_000)[0], b"")
        os.write(self.master, b"ready")
        self.assertTrue(select.select([self.slave], [], [], .1)[0])
        before = reader.stats()["select_calls"]
        self.assertEqual(reader.read_until(0, start+100_000_000)[0], b"ready")
        self.assertEqual(reader.stats()["select_calls"]-before, 1)
        before = reader.stats()["select_calls"]
        self.assertEqual(reader.read_until(0, start+100_000_000)[0], b"")
        self.assertEqual(reader.stats()["select_calls"]-before, 1)

    def test_hard_expiry_prevents_read_and_late_chunk_is_rejected(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        with patch(MODULE+".select.select") as poll:
            with self.assertRaises(TimeoutError):
                reader.read_until(clock.now+100, clock.now)
            poll.assert_not_called()
        hard = clock.now+100
        def late_read(*_):
            clock.now = hard
            return b"too late"
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
             patch(MODULE+".os.read", side_effect=late_read):
            with self.assertRaises(TimeoutError):
                reader.read_until(hard-1, hard)
        self.assertEqual(reader.stats()["bytes_received"], 8)
        self.assertEqual(reader.last_failed_read, ReceivedChunk(b"too late", hard - 100, hard))

    def test_postread_guard_keeps_exact_exception_bytes_and_pre_guard_timestamp(self):
        for original in (TimeoutError("phase deadline"), RuntimeError("binding changed"),
                         KeyboardInterrupt("cancel")):
            with self.subTest(exception=type(original).__name__):
                clock = Clock()
                reader = DeadlineSerialReader(self.raw, clock=clock)
                start, done, checks = clock.now, False, []
                def guard():
                    checks.append(clock())
                    if done:
                        clock.now += 10
                        raise original
                def read(*_):
                    nonlocal done
                    done = True
                    clock.now += 5
                    return b"AT\x01\x02"
                reader.check = guard
                with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
                     patch(MODULE+".os.read", side_effect=read):
                    with self.assertRaises(type(original)) as caught:
                        reader.read_until(start+50, start+100)
                self.assertIs(caught.exception, original)
                evidence = original.serial_read_evidence
                self.assertEqual(evidence, ReceivedChunk(b"AT\x01\x02", start, start+5))
                self.assertIs(reader.last_failed_read, evidence)
                self.assertEqual(len(checks), 5)  # Existing checks, including post-read.
                self.assertEqual(reader.stats()["read_calls"], 1)
                self.assertEqual(reader.stats()["bytes_received"], 4)
                with self.assertRaises(FrozenInstanceError):
                    evidence.received_ns = clock()

    def test_postread_clock_failure_keeps_bytes_without_inventing_timestamp(self):
        for mode in ("raises", "backwards", "invalid"):
            with self.subTest(mode=mode):
                clock = Clock()
                reader = DeadlineSerialReader(self.raw, clock=clock)
                start = clock.now
                original = RuntimeError("clock unavailable")
                def read(*_):
                    if mode == "raises":
                        reader.clock = lambda: (_ for _ in ()).throw(original)
                    else:
                        clock.now = start - 1 if mode == "backwards" else True
                    return b"partial"
                with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
                     patch(MODULE+".os.read", side_effect=read):
                    with self.assertRaises((RuntimeError, ValueError)) as caught:
                        reader.read_until(start+50, start+100)
                if mode == "raises":
                    self.assertIs(caught.exception, original)
                self.assertEqual(caught.exception.serial_read_evidence,
                                 ReceivedChunk(b"partial", start, None))
                self.assertEqual(reader.stats()["bytes_received"], 7)
                self.assertEqual(reader.stats()["read_wall_ns"], 0)

    def test_pre_read_and_syscall_failures_do_not_attach_stale_bytes(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        prior = ReceivedChunk(b"old", clock(), clock())
        reader.last_failed_read = prior
        first = KeyboardInterrupt("before read")
        reader.check = lambda: (_ for _ in ()).throw(first)
        with patch(MODULE+".os.read") as read:
            with self.assertRaises(KeyboardInterrupt) as caught:
                reader.read_until(clock()+50, clock()+100)
            self.assertIs(caught.exception, first)
            self.assertFalse(hasattr(caught.exception, "serial_read_evidence"))
            read.assert_not_called()
        reader.check = lambda: None
        error = OSError(errno.EIO, "failed read")
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
             patch(MODULE+".os.read", side_effect=error):
            with self.assertRaises(OSError) as caught:
                reader.read_until(clock()+50, clock()+100)
            self.assertIs(caught.exception, error)
            self.assertFalse(hasattr(error, "serial_read_evidence"))
        self.assertIs(reader.last_failed_read, prior)
        self.assertEqual(reader.stats()["bytes_received"], 0)

    def test_empty_eof_has_no_received_byte_evidence(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
             patch(MODULE+".os.read", return_value=b""):
            with self.assertRaises(EOFError) as caught:
                reader.read_until(clock()+50, clock()+100)
        self.assertFalse(hasattr(caught.exception, "serial_read_evidence"))
        self.assertIsNone(reader.last_failed_read)

    def test_reused_guard_exception_cannot_relog_prior_received_chunk(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        original = KeyboardInterrupt("same cancellation object")
        after_read = False
        def guard():
            if after_read:
                raise original
        def read(*_):
            nonlocal after_read
            after_read = True
            return b"AT"
        reader.check = guard
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])), \
             patch(MODULE+".os.read", side_effect=read) as syscall:
            with self.assertRaises(KeyboardInterrupt) as caught:
                reader.read_until(clock()+50, clock()+100)
            prior = caught.exception.serial_read_evidence
            with self.assertRaises(KeyboardInterrupt) as caught_again:
                reader.read_until(clock()+50, clock()+100)
        self.assertIs(caught_again.exception, original)
        self.assertFalse(hasattr(original, "serial_read_evidence"))
        self.assertIs(reader.last_failed_read, prior)
        self.assertEqual(syscall.call_count, 1)
        self.assertEqual(reader.stats()["bytes_received"], 2)

    def test_earliest_deadline_is_absolute_after_guard_and_eintr(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        start = clock.now
        waits = []
        def poll(_r, _w, _x, timeout):
            waits.append(timeout)
            if len(waits) == 1:
                clock.now += 20
                raise InterruptedError(errno.EINTR, "signal")
            clock.now = start+50
            return [], [], []
        with patch(MODULE+".select.select", side_effect=poll):
            self.assertEqual(reader.read_until(start+50, start+100), (b"", start+50))
        self.assertEqual(waits, [50/1e9, 30/1e9])
        self.assertEqual(reader.stats()["select_eintr"], 1)

    def test_guard_work_does_not_extend_wake_and_rx_stamp_precedes_guard(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        start = clock.now
        waits = []
        def check(): clock.now += 1
        reader.check = check
        def poll(_r, _w, _x, timeout):
            waits.append(timeout)
            return [self.slave], [], []
        def read(*_):
            clock.now += 5
            return b"x"
        with patch(MODULE+".select.select", side_effect=poll), patch(MODULE+".os.read", side_effect=read):
            chunk, received = reader.read_until(start+50, start+100)
        self.assertEqual(waits, [48/1e9])
        self.assertEqual(chunk, b"x")
        self.assertEqual(received, start+9)
        self.assertEqual(clock.now, received+1)

    def test_eagain_and_read_eintr_recompute_original_deadline(self):
        for err in (errno.EAGAIN, errno.EINTR):
            with self.subTest(errno=err):
                clock = Clock()
                reader = DeadlineSerialReader(self.raw, clock=clock)
                start = clock.now
                waits, reads = [], []
                def poll(_r, _w, _x, timeout):
                    waits.append(timeout)
                    return [self.slave], [], []
                def read(*_):
                    reads.append(True)
                    if len(reads) == 1:
                        clock.now += 20
                        raise OSError(err, "transient")
                    return b"ok"
                with patch(MODULE+".select.select", side_effect=poll), patch(MODULE+".os.read", side_effect=read):
                    self.assertEqual(reader.read_until(start+50, start+100)[0], b"ok")
                self.assertEqual(waits, [50/1e9, 30/1e9])

    def test_zero_wait_eagain_returns_empty_without_second_poll(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])) as poll, \
             patch(MODULE+".os.read", side_effect=BlockingIOError(errno.EAGAIN, "race")):
            self.assertEqual(reader.read_until(0, clock.now+100)[0], b"")
        self.assertEqual(poll.call_count, 1)

    def test_spurious_ready_loop_is_finitely_bounded(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])) as poll, \
             patch(MODULE+".os.read", side_effect=BlockingIOError(errno.EAGAIN, "race")):
            with self.assertRaisesRegex(RuntimeError, "Too many transient"):
                reader.read_until(clock.now+50, clock.now+100)
        self.assertEqual(poll.call_count, reader.MAX_TRANSIENT_RETRIES)

    def test_pty_peer_close_and_closed_fd_fail(self):
        reader = DeadlineSerialReader(self.raw)
        os.close(self.master)
        self.master = None
        now = time.monotonic_ns()
        with self.assertRaises((EOFError, OSError)):
            reader.read_until(now+100_000_000, now+200_000_000)
        os.close(self.slave)
        self.slave = None
        with self.assertRaises(OSError):
            reader.read_until(now+100_000_000, now+200_000_000)

    def test_explicit_eof_eio_exceptional_and_fd_replacement_fail(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])):
            with patch(MODULE+".os.read", return_value=b""):
                with self.assertRaises(EOFError): reader.read_until(clock.now+50, clock.now+100)
            with patch(MODULE+".os.read", side_effect=OSError(errno.EIO, "disconnected")):
                with self.assertRaises(OSError): reader.read_until(clock.now+50, clock.now+100)
        with patch(MODULE+".select.select", return_value=([], [], [self.slave])):
            with self.assertRaises(OSError): reader.read_until(clock.now+50, clock.now+100)
        self.raw.fd = self.master
        with self.assertRaisesRegex(RuntimeError, "changed or closed"):
            reader.read_until(clock.now+50, clock.now+100)

    def test_nonblocking_required_initially_and_before_read(self):
        os.set_blocking(self.slave, True)
        with self.assertRaisesRegex(RuntimeError, "nonblocking"):
            DeadlineSerialReader(self.raw)
        os.set_blocking(self.slave, False)
        reader = DeadlineSerialReader(self.raw)
        os.set_blocking(self.slave, True)
        now = time.monotonic_ns()
        with self.assertRaisesRegex(RuntimeError, "nonblocking"):
            reader.read_until(now+100_000_000, now+200_000_000)

    def test_invalid_clock_deadlines_and_backward_clock_fail(self):
        for value in (float("nan"), float("inf"), 1.0, True, -1):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    DeadlineSerialReader(self.raw, clock=lambda: value)
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        for value in (float("nan"), 1.0, True, -1):
            with self.assertRaises(ValueError): reader.read_until(value, clock.now+100)
        clock.now -= 1
        with self.assertRaisesRegex(RuntimeError, "backwards"):
            reader.read_until(clock.now+50, clock.now+100)

    def test_hard_deadline_bounds_wait_and_slow_postread_guard(self):
        clock = Clock()
        reader = DeadlineSerialReader(self.raw, clock=clock)
        start = clock.now
        waits = []
        def poll(_r, _w, _x, timeout):
            waits.append(timeout)
            clock.now = start+40
            return [], [], []
        with patch(MODULE+".select.select", side_effect=poll):
            with self.assertRaises(TimeoutError): reader.read_until(start+50, start+40)
        self.assertEqual(waits, [40/1e9])
        read_done = False
        hard = clock.now+50
        def guard():
            if read_done: clock.now = hard
        def read(*_):
            nonlocal read_done
            read_done = True
            return b"x"
        reader.check = guard
        with patch(MODULE+".select.select", return_value=([self.slave], [], [])), patch(MODULE+".os.read", side_effect=read):
            with self.assertRaises(TimeoutError): reader.read_until(hard-1, hard)


class FailedReadLoggingTests(unittest.TestCase):
    def log(self, evidence, *, limit=100, chunks=4, raw_log=None, report=None):
        error = KeyboardInterrupt("cancel after read")
        error.serial_read_evidence = evidence
        raw_log = [] if raw_log is None else raw_log
        report = {} if report is None else report
        try:
            raise error
        except BaseException as caught:
            count = record_failed_read(caught, report=report, raw_log=raw_log, raw_bytes=0,
                                       read_started_ns=1, max_raw_bytes=limit, max_raw_chunks=chunks)
        return count, raw_log, report, error

    def test_failed_chunk_uses_original_wrapper_start_and_is_never_a_reply(self):
        count, log, report, error = self.log(ReceivedChunk(b"AT", 3, 4))
        self.assertEqual((count, log), (2, [(1, 4, b"AT")]))
        self.assertEqual(report["rejected_receive_chunks"], 1)
        self.assertEqual(report["rejected_receive_bytes"], 2)
        self.assertEqual(error.args, ("cancel after read",))

    def test_missing_timestamp_uses_separate_bounded_evidence(self):
        evidence = ReceivedChunk(b"AT", 3, None)
        count, log, report, _ = self.log(evidence)
        self.assertEqual((count, log), (2, []))
        self.assertEqual(report["unclocked_receive_evidence"], [evidence.record()])
        count, log, report, _ = self.log(evidence, chunks=1, report=report)
        self.assertEqual(report["unclocked_receive_evidence"], [evidence.record()])
        self.assertTrue(report["raw_log_overflow"])
        self.assertEqual(report["unlogged_receive_bytes"], 2)

    def test_capacity_failure_is_visible_and_does_not_replace_original_error(self):
        for options in ({"limit": 1}, {"chunks": 0}):
            with self.subTest(options=options):
                count, log, report, error = self.log(ReceivedChunk(b"AT", 3, 4), **options)
                self.assertEqual((count, log), (2, []))
                self.assertTrue(report["raw_log_overflow"])
                self.assertEqual(report["unlogged_receive_bytes"], 2)
                self.assertIsInstance(error, KeyboardInterrupt)

    def test_storage_exception_does_not_hide_received_count_or_original_error(self):
        class BrokenLog(list):
            def append(self, _):
                raise OSError("evidence unavailable")
        count, log, report, error = self.log(ReceivedChunk(b"AT", 3, 4), raw_log=BrokenLog())
        self.assertEqual((count, log), (2, []))
        self.assertTrue(report["receive_evidence_storage_failed"])
        self.assertEqual(report["receive_evidence_storage_error_type"], "OSError")
        self.assertEqual(report["unlogged_receive_bytes"], 2)
        self.assertIsInstance(error, KeyboardInterrupt)


if __name__ == "__main__":
    unittest.main()
