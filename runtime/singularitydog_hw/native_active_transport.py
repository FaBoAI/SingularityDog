"""Explicit native active RS05 transport; no library/device open on import.

This layer validates wires, not mechanical readiness, calibration, watchdog
operation, or safe gains. The supervisor must establish those before enable.
It owns the serial descriptors, process-level locks, physical cutoff and arming.
"""
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import threading
import time

from .native_diagnostic_transport import Record, exchange_evidence
from .can_readonly import Frame
from .motor_version_probe import VERSION_PAYLOAD, VERSION_PREFIX, decode_version, version_request


class Stats(C.Structure):
    _fields_ = [(name, C.c_uint64) for name in
                ('begin_ns', 'end_ns', 'waits', 'reads', 'bytes', 'writes')] + [
        ('rejected', C.c_ubyte*4096), ('rejected_size', C.c_uint32),
        ('rejected_total', C.c_uint64)]


class Limits(C.Structure):
    _fields_ = [(key, C.c_double*6) for key in ('lower', 'upper', 'kp', 'kd')]


class StopResult(C.Structure):
    _fields_ = [(key, C.c_uint32) for key in
                ('attempted_mask', 'confirmed_mask', 'ambiguous_mask')] + [('fault', C.c_uint32*6)]


def load_library(path):
    path = Path(path).resolve(strict=True)
    record = json.loads((path.parent/'build-record.json').read_text())
    for key, source in (('source_sha256', path.parent/'transport.cpp'), ('binary_sha256', path)):
        if hashlib.sha256(source.read_bytes()).hexdigest() != record[key]:
            raise ValueError('Active native source/binary differs from build record')
    lib = C.CDLL(str(path))
    lib.sda_abi.restype = C.c_uint32
    if lib.sda_abi() != 1 or record.get('abi') != 1:
        raise ValueError('Active native ABI mismatch')
    lib.sda_now_ns.restype = C.c_uint64
    before = time.monotonic_ns(); native_now = lib.sda_now_ns(); after = time.monotonic_ns()
    if not before-1000 <= native_now <= after+1000:
        raise ValueError('Native and Python monotonic clocks differ')
    lib.sda_create.argtypes = [C.c_int, C.c_int, C.c_int, C.c_char_p, C.c_int,
        C.POINTER(Limits), C.c_uint64, C.c_uint32, C.POINTER(C.c_char), C.c_uint32]
    lib.sda_create.restype = C.c_void_p
    lib.sda_destroy.argtypes = [C.c_void_p]; lib.sda_destroy.restype = None
    lib.sda_exchange.argtypes = [C.c_void_p, C.POINTER(C.c_ubyte), C.c_uint32,
        C.c_int, C.c_uint64, C.POINTER(Record), C.POINTER(Stats), C.POINTER(C.c_char), C.c_uint32]
    lib.sda_exchange.restype = C.c_int
    lib.sda_emergency_stop.argtypes = [C.c_void_p, C.c_uint64, C.POINTER(Record),
        C.POINTER(Stats), C.POINTER(StopResult), C.POINTER(C.c_char), C.c_uint32]
    lib.sda_emergency_stop.restype = C.c_int
    waiter = getattr(lib, 'sda_wait_until', None)
    if waiter is not None:
        waiter.argtypes = [C.c_int, C.c_uint64, C.c_uint32,
            C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32]
        waiter.restype = C.c_int
    return lib


class ActiveWaitError(RuntimeError):
    """Bounded release wait was cancelled or failed before a motor cycle."""


def wait_until(library, cancel_fd, deadline_ns, *, spin_us=500):
    """Wait without motor I/O and return the actual monotonic wake timestamp."""
    if (type(cancel_fd) is not int or type(deadline_ns) is not int or
            not 0 < deadline_ns < 2**64 or type(spin_us) is not int or
            spin_us not in (200, 500)):
        raise ValueError('Invalid bounded active release wait arguments')
    waiter = getattr(library, 'sda_wait_until', None)
    if waiter is None:
        raise ActiveWaitError('Optional active release wait is unavailable in this library')
    actual, error = C.c_uint64(), C.create_string_buffer(256)
    status = waiter(cancel_fd, deadline_ns, spin_us, C.byref(actual), error, len(error))
    if status:
        raise ActiveWaitError(error.value.decode('utf-8', errors='replace'))
    if actual.value < deadline_ns:
        raise ActiveWaitError('Active release wait returned a backdated time')
    return actual.value


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite numeric')
    return float(value)


def encode_motion(mid, q, kp, kd):
    """Canonical Type1 with zero FF/vref; no clipping, wrapping, or gain selection.

    Session position limits apply to the decoded quantized target. A target
    exactly on a lower bound can round below it and will be rejected by C++.
    """
    if type(mid) is not int or not 1 <= mid <= 12:
        raise ValueError('Explicit motor ID 1..12 required')
    q, kp, kd = (_finite(x, name) for x, name in ((q, 'q'), (kp, 'kp'), (kd, 'kd')))
    if not -12.57 <= q <= 12.57 or not 0 <= kp <= 36 or not 0 <= kd <= 1:
        raise ValueError('Target/gain outside active software caps')
    payload = struct.pack('>4H', int((q+12.57)*65535./25.14), 32767,
                          int(kp*65535./500.), int(kd*65535./5.))
    can_id = (1 << 24) | (32767 << 8) | mid
    return b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+payload+b'\r\n'


class ExchangeError(RuntimeError):
    def __init__(self, message, records, stats):
        super().__init__(message)
        self.records, self.stats = records, stats


class ActiveSession:
    """One owner, one bus, explicit immutable limits. Caller owns all FDs.

    Call emergency_stop after any fault and save its evidence. It does not
    unpoison/re-arm the session. close releases ownership, never closes the FD.
    No destructor sends motor commands.
    """
    def __init__(self, library, fd, *, first_id, cancel_fd, boot_fd, boot_id,
                 raw_lower_by_id, raw_upper_by_id, kp_max_by_id, kd_max_by_id,
                 gap_ns=600_000, window=3):
        if type(first_id) is not int or first_id not in (1, 7):
            raise ValueError('first_id must be 1 or 7')
        if (type(gap_ns) is not int or not 600_000 <= gap_ns <= 5_000_000
                or type(window) is not int or not 1 <= window <= 3):
            raise ValueError('Invalid active gap/window')
        if type(boot_id) is not str or len(boot_id) != 36:
            raise ValueError('Explicit boot identity required')
        ids = set(range(first_id, first_id+6))
        limits = Limits()
        for name, values in (('lower', raw_lower_by_id), ('upper', raw_upper_by_id),
                             ('kp', kp_max_by_id), ('kd', kd_max_by_id)):
            if not isinstance(values, dict) or set(values) != ids or any(type(k) is not int for k in values):
                raise ValueError('Each limit map must have exactly this bus\'s six integer IDs')
            for index, mid in enumerate(range(first_id, first_id+6)):
                getattr(limits, name)[index] = _finite(values[mid], f'{name}[{mid}]')
        for i in range(6):
            if not (-12.57 <= limits.lower[i] < limits.upper[i] <= 12.57
                    and 0 <= limits.kp[i] <= 36 and 0 <= limits.kd[i] <= 1):
                raise ValueError('Invalid explicit per-axis software limits')
        if any(type(x) is not int for x in (fd, cancel_fd, boot_fd)):
            raise ValueError('Integer caller-owned FDs required')
        self.lib, self.fd, self.first_id = library, fd, first_id
        self.busy, self.poisoned, self._handle = threading.Lock(), False, None
        self._limits = limits
        self.boot_id = boot_id
        error = C.create_string_buffer(256)
        self._handle = library.sda_create(fd, cancel_fd, boot_fd, boot_id.encode('ascii'),
            first_id, C.byref(limits), gap_ns, window, error, len(error))
        if not self._handle:
            raise ValueError(error.value.decode())

    def _call(self, wires, timeout_ns, send_only, deadline_ns=None):
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Concurrent active session use')
        try:
            if not self._handle:
                raise RuntimeError('Active session closed')
            if self.poisoned:
                raise RuntimeError('Session poisoned; active retry prohibited')
            wires = tuple(wires)
            if (not 1 <= len(wires) <= 12 or any(type(w) is not bytes or len(w) != 17 for w in wires)
                    or (deadline_ns is None and
                        (type(timeout_ns) is not int or not 1_000_000 <= timeout_ns <= 250_000_000))):
                raise ValueError('Invalid active request batch/deadline')
            raw = (C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires))
            records, stats, error = (Record*len(wires))(), Stats(), C.create_string_buffer(256)
            now_ns = time.monotonic_ns()
            if deadline_ns is None:
                native_deadline_ns = now_ns + timeout_ns
            else:
                # The active controller's 20 ms deadline is absolute. Never
                # restart its clock after a worker has waited in a queue or
                # spent time preparing the native batch.
                if (type(deadline_ns) is not int or
                        not now_ns < deadline_ns <= now_ns + 250_000_000):
                    raise ValueError('Expired or invalid absolute active deadline')
                native_deadline_ns = deadline_ns
            status = self.lib.sda_exchange(self._handle, raw, len(wires), int(send_only),
                native_deadline_ns, records, C.byref(stats), error, len(error))
            if status:
                raise ExchangeError(error.value.decode(), records, stats)
            # STOP faults are returned, never turned into an apparently healthy reply.
            if any(r.received == 17 and (int.from_bytes(bytes(r.rx)[2:6], 'big') >> 19) & 63 for r in records
                   if (int.from_bytes(bytes(r.tx)[2:6], 'big') >> 27) == 4):
                self.poisoned = True
            return records, stats
        except BaseException:
            self.poisoned = True
            raise
        finally:
            self.busy.release()

    def exchange(self, wires, *, timeout_ns=100_000_000, deadline_ns=None):
        return self._call(wires, timeout_ns, False, deadline_ns)

    def send_only(self, wires, *, timeout_ns=100_000_000):
        """Unsupported: native active commands require acknowledgement.

        Kept as a fail-closed ABI compatibility entry point. Use exchange for
        the exact volatile watchdog write, then independently verify readback.
        """
        return self._call(wires, timeout_ns, True)

    def emergency_stop(self, *, timeout_ns=250_000_000, deadline_ns=None):
        """All six STOP attempts, independent of cancelled/poisoned/boot state.

        Returned confirmation is a matching mode-zero frame, not CAN wire time
        or proof of physical cutoff. Any uncertain prior enable/STOP response
        attribution remains explicitly ambiguous and unconfirmed (including
        pending Type1, which can reply mode zero after a watchdog transition).

        The aggregate 250 ms default reserves about 41.7 ms per axis. Real
        disabled-state replies took 22..26 ms, exceeding the former 25 ms
        first-axis slice. This only budgets STOP acknowledgement collection;
        it does not extend active-cycle or device-watchdog deadlines. No reply
        arriving after its own recorded deadline is promoted to confirmation.
        """
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Cancel active call and join owner before emergency_stop')
        try:
            if not self._handle:
                raise RuntimeError('Active session closed')
            if type(timeout_ns) is not int or not 20_000_000 <= timeout_ns <= 500_000_000:
                raise ValueError('Emergency budget must be 20..500 ms')
            self.poisoned = True
            records, stats, result, error = (Record*6)(), Stats(), StopResult(), C.create_string_buffer(256)
            now_ns = time.monotonic_ns()
            if deadline_ns is not None and (type(deadline_ns) is not int or
                    not 20_000_000 <= deadline_ns-now_ns <= 500_000_000):
                raise ValueError('Absolute emergency deadline must leave 20..500 ms')
            stop_deadline_ns = now_ns+timeout_ns if deadline_ns is None else deadline_ns
            status = self.lib.sda_emergency_stop(self._handle, stop_deadline_ns,
                records, C.byref(stats), C.byref(result), error, len(error))
            selected = lambda mask: [self.first_id+i for i in range(6) if mask & (1 << i)]
            confirmed = selected(result.confirmed_mask)
            evidence = exchange_evidence(records, stats)
            evidence['rejected_total_bytes'] = stats.rejected_total
            evidence['rejected_truncated'] = stats.rejected_total > stats.rejected_size
            return {'complete': status == 0, 'message': error.value.decode(),
                'timeout_ns': timeout_ns, 'deadline_monotonic_ns': stop_deadline_ns,
                'attempted_ids': selected(result.attempted_mask), 'confirmed_ids': confirmed,
                'unconfirmed_ids': [i for i in range(self.first_id, self.first_id+6) if i not in confirmed],
                'ambiguous_ids': selected(result.ambiguous_mask),
                'fault_by_id': {str(self.first_id+i): result.fault[i] for i in range(6) if records[i].received == 17},
                'replies': [decode_record(r) for r in records if r.received == 17],
                'evidence': evidence, 'poisoned': True}
        finally:
            self.busy.release()

    def emergency_stop_repeated(self, *, max_attempts=3, total_timeout_ns=1_000_000_000):
        """Retry STOP only, at most three rounds within one absolute budget.

        A successful first round is not repeated. Failed/ambiguous exchanges
        retain their raw evidence and the native session's sticky ambiguity;
        a later mode-zero frame cannot erase an uncertain Type1/enable/STOP
        transaction. This improves STOP delivery without resuming motion or
        claiming physical cutoff. Every observed fault remains in the result.
        The first round retains its 250 ms cap so first STOP delivery to later
        axes is not delayed. Retries may use 500 ms, within one total deadline.
        """
        if (type(max_attempts) is not int or not 1 <= max_attempts <= 3 or
                type(total_timeout_ns) is not int or
                not 21_000_000 <= total_timeout_ns <= 1_000_000_000):
            raise ValueError('STOP retries require 1..3 rounds and a 21..1000 ms total budget')
        self.poisoned = True
        begin = time.monotonic_ns(); deadline = begin+total_timeout_ns
        ids = set(range(self.first_id, self.first_id+6))
        attempts = []; ambiguous = set(); faults = {}
        result = {'complete': False, 'confirmed_ids': [], 'unconfirmed_ids': sorted(ids)}
        for attempt_index in range(max_attempts):
            now = time.monotonic_ns()
            # Keep a small preparation reserve; the native call still gets an
            # absolute deadline, so a scheduling delay never restarts a budget.
            remaining = deadline-now
            if remaining < 21_000_000:break
            round_limit = 250_000_000 if attempt_index == 0 else 500_000_000
            round_deadline = min(deadline, now+round_limit)
            try:
                row = self.emergency_stop(timeout_ns=min(remaining,round_limit),
                                          deadline_ns=round_deadline)
            except Exception as error:
                row = {'complete': False, 'confirmed_ids': [], 'unconfirmed_ids': sorted(ids),
                       'error': type(error).__name__+': '+str(error)}
            attempts.append(row)
            ambiguous.update(row.get('ambiguous_ids', []))
            for mid, bits in row.get('fault_by_id', {}).items():
                faults[mid] = faults.get(mid, 0) | bits
            confirmed = set(row.get('confirmed_ids', []))-ambiguous
            result = {**row, 'confirmed_ids': sorted(confirmed),
                      'unconfirmed_ids': sorted(ids-confirmed),
                      'ambiguous_ids': sorted(ambiguous), 'fault_by_id': dict(faults)}
            result['complete'] = (row.get('complete') is True and confirmed == ids and
                                  not row.get('unconfirmed_ids') and not ambiguous)
            if result['complete']:break
        end = time.monotonic_ns()
        result.update(attempts=attempts, poisoned=True, retry_policy={
            'stop_only': True, 'max_attempts': max_attempts,
            'attempts_completed': len(attempts), 'total_timeout_ns': total_timeout_ns,
            'first_attempt_cap_ns': 250_000_000, 'retry_attempt_cap_ns': 500_000_000,
            'begin_ns': begin, 'end_ns': end, 'deadline_ns': deadline,
            'budget_exhausted': deadline-end < 21_000_000 and not result['complete'],
            'motion_retry_allowed': False, 'ambiguity_preserved': True})
        return result

    def close(self):
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Cannot close an active native call')
        try:
            if self._handle:
                self.lib.sda_destroy(self._handle)
                self._handle = None
        finally:
            self.busy.release()

    def __del__(self):
        handle = getattr(self, '_handle', None)
        if handle:
            self.lib.sda_destroy(handle)
            self._handle = None


def decode_record(record):
    """Decode one native-validated fixed17B record without a stream parser.

    Native exchange already frames the stream. Check the canonical envelope
    again here, retaining raw bytes, fault fields and version discrimination.
    """
    if record.written != 17 or record.received != 17:
        raise ValueError('Incomplete active record')
    tx, rx = bytes(record.tx), bytes(record.rx)
    for wire in (tx, rx):
        if (len(wire) != 17 or wire[:2] != b'AT' or wire[6] != 8 or
                wire[-2:] != b'\r\n' or wire[5] & 7 != 4):
            raise ValueError('Noncanonical active frame')
    tx_id = int.from_bytes(tx[2:6], 'big') >> 3
    rx_id = int.from_bytes(rx[2:6], 'big') >> 3
    mid, tx_kind, rx_kind = tx_id & 255, (tx_id >> 24) & 31, (rx_id >> 24) & 31
    data = rx[7:15]
    result = {'motor_id': mid, 'request_kind': tx_kind,
              'reply_kind': rx_kind, 'reply_wire_hex': rx.hex(),
              'received_monotonic_ns': record.received_ns}
    if tx_kind == 4 and tx[7:15] == VERSION_PAYLOAD:
        result.update(decode_version(Frame(rx_id, 4, data, rx), mid))
        result['request_started_monotonic_ns'] = record.start_ns
        return result
    if rx_kind == 2:
        if data[:3] == VERSION_PREFIX:
            raise ValueError('Firmware version reply cannot be active telemetry or STOP acknowledgement')
        p, v, torque, temp = struct.unpack('>4H', data)
        result.update(mode_state=(rx_id >> 22) & 3, fault_bits=(rx_id >> 16) & 63,
            position_u16=p, velocity_u16=v, torque_u16=torque, temperature_u16=temp,
            position_rad_candidate=p*25.14/65535.-12.57,
            velocity_rad_s_candidate=v*100./65535.-50., torque_nm_candidate=torque*11./65535.-5.5,
            temperature_c=temp/10.)
    elif rx_kind == 17:
        index = int.from_bytes(data[:2], 'little')
        value = (int.from_bytes(data[4:], 'little') if index == 0x7028 else
                 data[4] if index == 0x7005 else struct.unpack('<f', data[4:])[0])
        result.update(index=index, value=value)
    else:
        result['uid_hex'] = data.hex()
    return result
