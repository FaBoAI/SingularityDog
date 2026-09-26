"""Receive-only POSIX serial reader with absolute scheduling deadlines.

``read_until(wake_ns, hard_ns)`` returns one chunk and its host read-completion
timestamp. An ordinary wake expiry returns ``b''``; hard expiry always raises.
This is not the motor's sampling timestamp. The caller exclusively owns the
serial object and closes it; this class neither transmits nor closes the fd.
"""
import errno
from dataclasses import dataclass
import os
import select
import time


@dataclass(frozen=True)
class ReceivedChunk:
    """Rejected receive evidence, never a successful read or parsed reply.

    read_started_ns is the clock immediately before os.read, not the caller's
    broader select/read interval. None means no valid completion clock could
    be obtained; callers must keep that record outside timestamped RX streams.
    """

    data: bytes
    read_started_ns: int
    received_ns: int | None

    def record(self):
        return {"read_started_ns": self.read_started_ns,
                "received_ns": self.received_ns, "hex": self.data.hex()}


def record_failed_read(error, *, report, raw_log, raw_bytes, read_started_ns,
                       max_raw_bytes, max_raw_chunks):
    """Retain one failed read in a bounded probe log, without parsing it.

    Call only from the read_until exception handler, then re-raise that error.
    Missing timestamps use a separate diagnostic list. Storage failures and
    exhausted budgets are recorded, and never replace the original exception.
    The byte count includes rejected and unlogged bytes as well as normal RX.
    """
    evidence = getattr(error, "serial_read_evidence", None)
    if not isinstance(evidence, ReceivedChunk):
        return raw_bytes
    raw_bytes += len(evidence.data)
    report["rejected_receive_chunks"] = report.get("rejected_receive_chunks", 0) + 1
    report["rejected_receive_bytes"] = report.get("rejected_receive_bytes", 0) + len(evidence.data)
    other = report.get("unclocked_receive_evidence", [])
    if raw_bytes > max_raw_bytes or len(raw_log) + len(other) >= max_raw_chunks:
        report["raw_log_overflow"] = True
        report["unlogged_receive_bytes"] = report.get("unlogged_receive_bytes", 0) + len(evidence.data)
        return raw_bytes
    try:
        if evidence.received_ns is None:
            report.setdefault("unclocked_receive_evidence", []).append(evidence.record())
        else:
            raw_log.append((read_started_ns, evidence.received_ns, evidence.data))
    except BaseException as storage_error:
        report["receive_evidence_storage_failed"] = True
        report["receive_evidence_storage_error_type"] = type(storage_error).__name__
        report["unlogged_receive_bytes"] = report.get("unlogged_receive_bytes", 0) + len(evidence.data)
    return raw_bytes


class DeadlineSerialReader:
    """Use select + nonblocking read without changing termios per receive.

    ``clock`` returns monotonic integer nanoseconds. ``check`` may raise to
    enforce cancellation/ownership/global guards, but must not read the port.
    Setup sets PySerial's timeout to zero once and requires a nonblocking fd.
    Do not change that fd/configuration or share its reads during this lifetime.
    Counters measure host elapsed time, including syscall scheduling delays.
    A post-read exception keeps its original type/identity and carries immutable
    ``serial_read_evidence``. ``last_failed_read`` also retains the latest such
    chunk (at most4096 bytes); use the exception attribute for per-call logging,
    since cleanup reads can replace the diagnostic slot. Normal reads are not
    duplicated there. This class never turns rejected bytes into successful RX.
    """

    CHUNK_SIZE = 4096
    MAX_TRANSIENT_RETRIES = 32

    def __init__(self, raw_serial, *, clock=time.monotonic_ns, check=lambda: None):
        self.raw_serial, self.clock, self.check = raw_serial, clock, check
        self._last_now = None
        self.last_failed_read = None
        self._stats = {name: 0 for name in (
            "read_until_calls", "select_calls", "select_wait_ns", "read_calls",
            "read_wall_ns", "bytes_received", "select_eintr", "read_eintr",
            "spurious_readiness", "soft_expiries", "hard_expiries", "eof_events")}
        self._now()
        self.check()
        raw_serial.timeout = 0
        self.fd = raw_serial.fileno()
        if type(self.fd) is not int or self.fd < 0:
            raise ValueError("Serial fd must be a nonnegative integer")
        self._check_fd()

    @staticmethod
    def _validate_ns(value, name):
        if type(value) is not int or not 0 <= value < 2**63:
            raise ValueError(f"{name} must be nonnegative integer nanoseconds below 2**63")
        return value

    def _now(self):
        now = self._validate_ns(self.clock(), "clock")
        if self._last_now is not None and now < self._last_now:
            raise RuntimeError("Monotonic clock moved backwards")
        self._last_now = now
        return now

    def _check_fd(self):
        if self.raw_serial.fileno() != self.fd:
            raise RuntimeError("Serial fd changed or closed")
        if os.get_blocking(self.fd):
            raise RuntimeError("Serial fd must already be nonblocking")

    def _guard(self, hard_ns):
        self.check()
        now = self._now()
        if now >= hard_ns:
            self._stats["hard_expiries"] += 1
            raise TimeoutError("Serial hard receive deadline reached")
        return now

    def _empty(self, hard_ns):
        now = self._guard(hard_ns)
        self._stats["soft_expiries"] += 1
        return b"", now

    def stats(self):
        """Return a detached counter snapshot; no logging or file I/O."""
        return dict(self._stats)

    def read_until(self, wait_deadline_ns, hard_deadline_ns):
        """Receive up to 4096 bytes, bounded by absolute wake and hard deadlines.

        If the wake is already due, perform exactly one zero-time readiness
        poll while the hard deadline remains live. EINTR/EAGAIN never extend
        either deadline; a finite transient cap also prevents a stuck clock
        from causing an unbounded retry loop. Received bytes at/after the hard
        deadline are rejected, even if the OS read itself succeeded.
        """
        self._failed_read_for_call = None
        try:
            return self._read_until(wait_deadline_ns, hard_deadline_ns)
        except BaseException as error:
            # Guards may reuse an exception instance during cleanup. Its old
            # chunk must not be mistaken for new bytes on a pre-read failure.
            if (self._failed_read_for_call is None
                    and isinstance(getattr(error, "serial_read_evidence", None), ReceivedChunk)):
                BaseException.__delattr__(error, "serial_read_evidence")
            raise

    def _read_until(self, wait_deadline_ns, hard_deadline_ns):
        wake = self._validate_ns(wait_deadline_ns, "wait_deadline_ns")
        hard = self._validate_ns(hard_deadline_ns, "hard_deadline_ns")
        self._stats["read_until_calls"] += 1
        retries = 0
        polled = False
        while True:
            now = self._guard(hard)
            self._check_fd()
            # fd/ownership checks must not extend the original absolute wait.
            now = self._guard(hard)
            if polled and now >= wake:
                return self._empty(hard)
            polled = True
            self._stats["select_calls"] += 1
            started = self._now()
            if started >= hard:
                self._stats["hard_expiries"] += 1
                raise TimeoutError("Serial hard receive deadline reached")
            timeout = max(0, min(wake, hard) - started) / 1e9
            try:
                ready, _, exceptional = select.select([self.fd], [], [self.fd], timeout)
            except OSError as exc:
                ended = self._now()
                self._stats["select_wait_ns"] += ended - started
                if exc.errno != errno.EINTR:
                    raise
                self._stats["select_eintr"] += 1
                retries += 1
                self._guard(hard)
                if retries >= self.MAX_TRANSIENT_RETRIES:
                    raise RuntimeError("Too many interrupted serial waits") from exc
                continue
            ended = self._now()
            self._stats["select_wait_ns"] += ended - started
            self._guard(hard)
            if exceptional:
                raise OSError(errno.EIO, "Serial fd reported an exceptional condition")
            if not ready:
                if ended < min(wake, hard):
                    # A real select timeout advances to its deadline. Bound
                    # unexpected early empty wakeups as well as EAGAIN races.
                    retries += 1
                    self._stats["spurious_readiness"] += 1
                    if retries >= self.MAX_TRANSIENT_RETRIES:
                        raise RuntimeError("Too many spurious serial wakeups")
                    continue
                return self._empty(hard)
            self._check_fd()
            self._guard(hard)
            self._stats["read_calls"] += 1
            started = self._now()
            try:
                chunk = os.read(self.fd, self.CHUNK_SIZE)
            except OSError as exc:
                received_ns = self._now()
                self._stats["read_wall_ns"] += received_ns - started
                if exc.errno not in (errno.EINTR, errno.EAGAIN, errno.EWOULDBLOCK):
                    raise
                key = "read_eintr" if exc.errno == errno.EINTR else "spurious_readiness"
                self._stats[key] += 1
                retries += 1
                self._guard(hard)
                if retries >= self.MAX_TRANSIENT_RETRIES:
                    raise RuntimeError("Too many transient serial reads") from exc
                continue
            # Capture immediately after os.read, before counters or callbacks.
            received_ns = None
            try:
                received_ns = self._now()
                self._stats["read_wall_ns"] += received_ns - started
                self._stats["bytes_received"] += len(chunk)
                self._guard(hard)
                if not chunk:
                    self._stats["eof_events"] += 1
                    raise EOFError("Serial peer closed or returned EOF")
            except BaseException as error:
                if chunk:
                    if received_ns is None:
                        self._stats["bytes_received"] += len(chunk)
                    evidence = ReceivedChunk(bytes(chunk), started, received_ns)
                    self.last_failed_read = evidence
                    self._failed_read_for_call = evidence
                    # Bypass custom exception __setattr__ without changing the
                    # original cancellation/deadline exception or its traceback.
                    BaseException.__setattr__(error, "serial_read_evidence", evidence)
                raise
            return chunk, received_ns
