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
from .can_readonly import ATParser
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
    return lib


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

    def _call(self, wires, timeout_ns, send_only):
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Concurrent active session use')
        try:
            if not self._handle:
                raise RuntimeError('Active session closed')
            if self.poisoned:
                raise RuntimeError('Session poisoned; active retry prohibited')
            wires = tuple(wires)
            if (not 1 <= len(wires) <= 12 or any(type(w) is not bytes or len(w) != 17 for w in wires)
                    or type(timeout_ns) is not int or not 1_000_000 <= timeout_ns <= 250_000_000):
                raise ValueError('Invalid active request batch/deadline')
            raw = (C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires))
            records, stats, error = (Record*len(wires))(), Stats(), C.create_string_buffer(256)
            status = self.lib.sda_exchange(self._handle, raw, len(wires), int(send_only),
                time.monotonic_ns()+timeout_ns, records, C.byref(stats), error, len(error))
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

    def exchange(self, wires, *, timeout_ns=100_000_000):
        return self._call(wires, timeout_ns, False)

    def send_only(self, wires, *, timeout_ns=100_000_000):
        """Unsupported: native active commands require acknowledgement.

        Kept as a fail-closed ABI compatibility entry point. Use exchange for
        the exact volatile watchdog write, then independently verify readback.
        """
        return self._call(wires, timeout_ns, True)

    def emergency_stop(self, *, timeout_ns=250_000_000):
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
            if type(timeout_ns) is not int or not 20_000_000 <= timeout_ns <= 250_000_000:
                raise ValueError('Emergency budget must be 20..250 ms')
            self.poisoned = True
            records, stats, result, error = (Record*6)(), Stats(), StopResult(), C.create_string_buffer(256)
            stop_deadline_ns = time.monotonic_ns()+timeout_ns
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
    """Pure evidence decoder; no clipping/scaling repair or health approval."""
    if record.written != 17 or record.received != 17:
        raise ValueError('Incomplete active record')
    tx = ATParser().feed(bytes(record.tx))[0]
    rx = ATParser().feed(bytes(record.rx))[0]
    result = {'motor_id': tx.destination, 'request_kind': tx.kind,
              'reply_kind': rx.kind, 'reply_wire_hex': bytes(record.rx).hex(),
              'received_monotonic_ns': record.received_ns}
    if tx.kind==4 and tx.data==VERSION_PAYLOAD:
        result.update(decode_version(rx,tx.destination))
        result['request_started_monotonic_ns']=record.start_ns
        return result
    if rx.kind == 2:
        if rx.data[:3]==VERSION_PREFIX:
            raise ValueError('Firmware version reply cannot be active telemetry or STOP acknowledgement')
        p, v, torque, temp = struct.unpack('>4H', rx.data)
        result.update(mode_state=(rx.can_id >> 22) & 3, fault_bits=(rx.can_id >> 16) & 63,
            position_u16=p, velocity_u16=v, torque_u16=torque, temperature_u16=temp,
            position_rad_candidate=p*25.14/65535.-12.57,
            velocity_rad_s_candidate=v*100./65535.-50., torque_nm_candidate=torque*11./65535.-5.5,
            temperature_c=temp/10.)
    elif rx.kind == 17:
        index = int.from_bytes(rx.data[:2], 'little')
        value = (int.from_bytes(rx.data[4:], 'little') if index == 0x7028 else
                 rx.data[4] if index == 0x7005 else struct.unpack('<f', rx.data[4:])[0])
        result.update(index=index, value=value)
    else:
        result['uid_hex'] = rx.data.hex()
    return result
