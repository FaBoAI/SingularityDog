"""Explicit C++ backend for bounded Type0/17 and STOP-proxy diagnostics.

No native library is loaded on import. This is not the active motor transport.
Callers own nonblocking serial ports, port/common locks, UID verification, and
boot guard lifetimes. Any error poisons a session; no flush/retry is available.
"""
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import threading
import time

from .can_readonly import ATParser, decode_reply, read_request
from .fast_policy_inputs import UNVERIFIED_FLAGS


class Record(C.Structure):
    _fields_ = [(name, C.c_uint64) for name in
                ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns')] + [
        ('tx', C.c_ubyte*17), ('rx', C.c_ubyte*17),
        ('written', C.c_uint32), ('received', C.c_uint32)]


class Stats(C.Structure):
    _fields_ = [(name, C.c_uint64) for name in
                ('begin_ns', 'end_ns', 'waits', 'reads', 'bytes', 'writes')] + [
        ('rejected', C.c_ubyte*4096), ('rejected_size', C.c_uint32)]


def load_library(path):
    path = Path(path).resolve(strict=True)
    record = json.loads((path.parent/'build-record.json').read_text())
    for key, source in (('source_sha256', path.parent/'transport.cpp'), ('binary_sha256', path)):
        if hashlib.sha256(source.read_bytes()).hexdigest() != record[key]:
            raise ValueError('Native source/binary differs from build record')
    lib = C.CDLL(str(path))  # CDLL releases GIL during sd_exchange; PyDLL must not be used.
    lib.sd_abi.restype = C.c_uint32
    if lib.sd_abi() != 1:
        raise ValueError('Native ABI mismatch')
    lib.sd_now_ns.restype = C.c_uint64
    before=time.monotonic_ns(); native_now=lib.sd_now_ns(); after=time.monotonic_ns()
    if not before-1000 <= native_now <= after+1000:
        raise ValueError('Native and Python monotonic clocks differ')
    lib.sd_exchange.argtypes = [C.c_int, C.c_int, C.c_int, C.c_char_p,
        C.POINTER(C.c_ubyte), C.c_uint32, C.c_int, C.c_int, C.c_uint64,
        C.c_uint32, C.c_uint64, C.c_uint64, C.POINTER(Record), C.POINTER(Stats),
        C.POINTER(C.c_char), C.c_uint32]
    lib.sd_exchange.restype = C.c_int
    # Older validated diagnostic libraries have the exchange ABI but not the
    # optional release waiter. Keep the default exchange path loadable.
    waiter = getattr(lib, 'sd_wait_until', None)
    if waiter is not None:
        waiter.argtypes = [C.c_int, C.c_uint64, C.c_uint32,
            C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32]
        waiter.restype = C.c_int
    return lib


class WaitError(RuntimeError):
    """A bounded read-only deadline wait was cancelled or failed."""


def wait_until(library, cancel_fd, deadline_ns, *, spin_us=200):
    """Return actual monotonic wake time, never the planned deadline.

    The native CDLL call releases the GIL. Only a cancellation fd is passed;
    no serial fd or motor command can be issued by this waiter.
    """
    if (type(cancel_fd) is not int or type(deadline_ns) is not int or
            not 0 < deadline_ns < 2**64 or type(spin_us) is not int or
            spin_us not in (200, 500)):
        raise ValueError('Invalid bounded diagnostic wait arguments')
    waiter = getattr(library, 'sd_wait_until', None)
    if waiter is None:
        raise WaitError('Optional native deadline wait is unavailable in this library')
    actual, error = C.c_uint64(), C.create_string_buffer(256)
    status = waiter(cancel_fd, deadline_ns, spin_us, C.byref(actual), error, len(error))
    if status:
        raise WaitError(error.value.decode('utf-8', errors='replace'))
    if actual.value < deadline_ns:
        raise WaitError('Native wait returned a backdated time')
    return actual.value


_WAIT_ARGUMENT_TYPES = (C.c_int, C.c_uint64, C.c_uint32,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)
_EMPTY_WAIT_ERROR = bytes(256)


class _OwnedDiagnosticWaiter:
    """Opt-in scratch buffers for one coordinator; owns no FD or library.

    Construct only after ``load_library`` has verified the diagnostic library.
    The original ``wait_until`` path and all its callers remain unchanged.
    This calls the same GIL-releasing routine with the same absolute deadline,
    cancellation FD and spin option. Buffer reuse is not a timing guarantee.
    """

    def __init__(self, library, cancel_fd, spin_us):
        if (type(cancel_fd) is not int or not 0 <= cancel_fd < 2**31 or
                type(spin_us) is not int or spin_us not in (200, 500)):
            raise ValueError('Invalid owned diagnostic wait arguments')
        waiter = getattr(library, 'sd_wait_until', None)
        if waiter is None:
            raise WaitError('Optional native deadline wait is unavailable in this library')
        self._library, self._waiter = library, waiter
        self._verify_function()
        abi = getattr(library, 'sd_abi', None)
        value = abi() if callable(abi) else None
        if type(value) is not int or value != 1:
            raise ValueError('Native wait ABI mismatch')
        self._cancel_fd, self._spin_us = cancel_fd, spin_us
        self._owner = threading.current_thread()
        self._busy = threading.Lock()
        self._actual, self._error = C.c_uint64(), C.create_string_buffer(256)
        self._actual_ptr = C.byref(self._actual)

    def _verify_function(self):
        waiter = self._waiter
        if (getattr(self._library, 'sd_wait_until', None) is not waiter or
                not isinstance(waiter, C._CFuncPtr) or
                waiter._flags_ != C._FUNCFLAG_CDECL or
                tuple(waiter.argtypes or ()) != _WAIT_ARGUMENT_TYPES or
                waiter.restype is not C.c_int or
                getattr(waiter, 'errcheck', None) is not None):
            raise ValueError('Exact GIL-releasing diagnostic wait ABI required')

    def __call__(self, deadline_ns):
        if threading.current_thread() is not self._owner:
            raise WaitError('Owned diagnostic wait called from another thread')
        if type(deadline_ns) is not int or not 0 < deadline_ns < 2**64:
            raise ValueError('Invalid bounded diagnostic wait arguments')
        if not self._busy.acquire(blocking=False):
            raise WaitError('Reentrant owned diagnostic wait')
        try:
            self._verify_function()
            self._actual.value = 0
            self._error.raw = _EMPTY_WAIT_ERROR
            status = self._waiter(self._cancel_fd, deadline_ns, self._spin_us,
                self._actual_ptr, self._error, 256)
            if status:
                raise WaitError(self._error.value.decode('utf-8', errors='replace') or
                                'Native diagnostic wait failed without an error message')
            if self._error.value:
                raise WaitError('Native wait returned success with an error')
            if self._actual.value < deadline_ns:
                raise WaitError('Native wait returned a backdated time')
            return self._actual.value
        finally:
            # No prior wake/error survives a success, cancellation or exception.
            # Assigning .raw copies in Python; do not add a GIL-releasing memset.
            self._actual.value = 0
            self._error.raw = _EMPTY_WAIT_ERROR
            self._busy.release()


def make_owned_waiter(library, cancel_fd, *, spin_us=200):
    """Return an explicit, non-reentrant callable for one coordinator thread.

    No device/FD is opened, closed or written, and no existing path selects it.
    The caller must retain ownership of its cancellation FD and the verified
    library, and keep its original boot checks, poll cadence and hard deadlines.
    """
    return _OwnedDiagnosticWaiter(library, cancel_fd, spin_us)


def stop_wire(mid):
    # Canonical all-zero STOP (Type4); never enables the actuator.
    wire = bytearray(read_request(mid))
    encoded = (((4 << 24) | (0xfd << 8) | mid) << 3) | 4
    wire[2:6] = encoded.to_bytes(4, 'big')
    return bytes(wire)


class ExchangeError(RuntimeError):
    def __init__(self, message, records, stats):
        super().__init__(message)
        self.records, self.stats = records, stats


class NativeSession:
    """No device open/close or configuration; caller owns the FD for this lifetime."""
    def __init__(self, library, fd, *, first_id, cancel_fd, boot_fd=-1, boot_id=None,
                 stop_proxy=False, gap_ns=600_000, window=3):
        if (type(first_id) is not int or first_id not in (1, 7)
                or type(stop_proxy) is not bool or type(gap_ns) is not int
                or not 600_000 <= gap_ns <= 5_000_000
                or type(window) is not int or not 1 <= window <= 3):
            raise ValueError('Invalid native diagnostic configuration')
        if os.get_blocking(fd):
            raise ValueError('FD must already be nonblocking')
        self.lib, self.fd, self.first_id = library, fd, first_id
        self.cancel_fd, self.boot_fd = cancel_fd, boot_fd
        self.boot_id = boot_id.encode('ascii') if boot_id is not None else None
        self.stop_proxy, self.gap_ns, self.window = stop_proxy, gap_ns, window
        st = os.fstat(fd)
        self.binding = (st.st_dev, st.st_ino, st.st_rdev)
        self.poisoned, self.busy = False, threading.Lock()
        self.last_finish_ns = 0

    def exchange(self, wires, *, timeout_ns=100_000_000, before_native=None):
        """Optionally publish completed prior work after preparing this call.

        The hook runs once, after FD checks and buffer/deadline preparation,
        immediately before entering the GIL-releasing native call. A hook
        failure poisons the session without issuing this batch.
        """
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Concurrent exchange on one native session')
        try:
            if self.poisoned:
                raise RuntimeError('Session poisoned; no retry')
            st = os.fstat(self.fd)
            if self.binding != (st.st_dev, st.st_ino, st.st_rdev) or os.get_blocking(self.fd):
                raise ValueError('Native FD binding/configuration changed')
            wires = tuple(wires)
            if (not 1 <= len(wires) <= 12 or any(type(w) is not bytes or len(w) != 17 for w in wires)
                    or type(timeout_ns) is not int or not 1_000_000 <= timeout_ns <= 250_000_000
                    or (before_native is not None and not callable(before_native))):
                raise ValueError('Invalid native request batch/deadline')
            raw = (C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires))
            records, stats, error = (Record*len(wires))(), Stats(), C.create_string_buffer(256)
            native_call = self.lib.sd_exchange
            arguments = (self.fd, self.cancel_fd, self.boot_fd, self.boot_id,
                raw, len(wires), self.first_id, int(self.stop_proxy), self.gap_ns,
                self.window, time.monotonic_ns()+timeout_ns, self.last_finish_ns,
                records, C.byref(stats), error, 256)
            if before_native is not None:before_native()
            status = native_call(*arguments)
            self.last_finish_ns = max(r.finish_ns for r in records)
            if status:
                raise ExchangeError(error.value.decode(), records, stats)
            return records, stats
        except BaseException:
            self.poisoned = True
            raise
        finally:
            self.busy.release()


def records_as_events(records, *, cycle=1):
    """Post-exchange conversion for legacy validators and evidence, not hot RX."""
    rows = []
    for r in records:
        if r.written != 17 or r.received != 17:
            raise ValueError('Incomplete native record')
        tx, rx = ATParser().feed(bytes(r.tx))[0], ATParser().feed(bytes(r.rx))[0]
        mid = tx.destination
        if tx.kind == 4:
            import struct
            p, v, torque, temp = struct.unpack('>4H', rx.data)
            scale = lambda x, limit: x*(2.*limit)/65535.-limit
            parameter = 'stop_feedback'
            result = {**dict.fromkeys(UNVERIFIED_FLAGS, False), 'motor_id': mid,
                'parameter': parameter, 'ok': True, 'raw_frame': rx.record(),
                'position_velocity_in_one_reply': True, 'mode_state': 0, 'fault_bits': 0,
                'position_u16': p, 'velocity_u16': v, 'torque_u16': torque, 'temperature_u16': temp,
                'position_rad_candidate': scale(p, 12.57), 'velocity_rad_s_candidate': scale(v, 50.),
                'torque_nm_candidate': scale(torque, 5.5), 'temperature_c': temp/10.}
        else:
            parameter = (None if tx.kind == 0 else
                         {b'\x19\x70':'position',b'\x1b\x70':'velocity',
                          b'\x1c\x70':'voltage'}.get(tx.data[:2]))
            if tx.kind == 17 and parameter is None:
                raise ValueError('Unknown diagnostic Type17 parameter')
            result = decode_reply(rx, mid, parameter)
            if not result['ok']:
                raise ValueError('Rejected native reply')
            parameter = parameter or 'identity'
        rows.append({'kind': 'pipeline_reply', 'ok': True, 'motor_id': mid,
            'parameter': parameter, 'cycle': cycle, 'result': result,
            'write_call_entered': True, 'write_expected_bytes': 17, 'write_returned_bytes': r.written,
            'write_started_monotonic_ns': r.start_ns, 'write_finished_monotonic_ns': r.finish_ns,
            'read_started_monotonic_ns': r.read_start_ns, 'received_monotonic_ns': r.received_ns,
            'deadline_monotonic_ns': r.deadline_ns,
            'request_wire_hex': bytes(r.tx).hex(), 'reply_wire_hex': bytes(r.rx).hex()})
    return rows


def exchange_evidence(records, stats):
    return {'records': [{'tx_hex': bytes(r.tx).hex(), 'rx_hex': bytes(r.rx).hex(),
        'start_ns': r.start_ns, 'finish_ns': r.finish_ns, 'read_start_ns': r.read_start_ns,
        'received_ns': r.received_ns, 'deadline_ns': r.deadline_ns,
        'written': r.written, 'received': r.received} for r in records],
        'stats': {name: getattr(stats, name) for name in
                  ('begin_ns', 'end_ns', 'waits', 'reads', 'bytes', 'writes')},
        'rejected_hex': bytes(stats.rejected[:stats.rejected_size]).hex()}
