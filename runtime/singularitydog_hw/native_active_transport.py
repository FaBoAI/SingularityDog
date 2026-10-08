"""Explicit native active RS05 transport; no library/device open on import.

This layer validates wires, not mechanical readiness, calibration, watchdog
operation, or safe gains. The supervisor must establish those before enable.
It owns the serial descriptors, process-level locks, physical cutoff and arming.
"""
import ctypes as C
from contextlib import contextmanager
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import struct
import stat
import tempfile
import threading
import time
import weakref
from types import MappingProxyType

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


class PairStats(C.Structure):
    _fields_ = [(key, C.c_uint64) for key in
                ('generation', 'submitted_ns', 'validated_ns', 'released_ns')] + [
        ('owner_started_ns', C.c_uint64*2), ('owner_finished_ns', C.c_uint64*2),
        ('cancel_requested_ns', C.c_uint64), ('owner_status', C.c_int32*2)]


class PairOwnerSettings(C.Structure):
    _fields_ = [(key, C.c_uint64) for key in ('native_tid', 'cpu_mask', 'timer_slack_ns',
                'original_cpu_mask', 'original_timer_slack_ns')] + [
        (key, C.c_int32) for key in ('status', 'configured', 'restored')]


class FeedbackDecoded(C.Structure):
    _fields_ = [(key, C.c_uint32) for key in
                ('motor_id', 'mode_state', 'fault_bits', 'position_u16')] + [
        (key, C.c_double) for key in ('protocol_position_rad', 'velocity_rad_s',
                                     'torque_nm', 'temperature_c')] + [
        (key, C.c_uint64) for key in ('request_started_ns', 'received_ns')]


@contextmanager
def _library_binding(path, expected_sha256):
    if expected_sha256 is None:
        yield str(path), None
        return
    if (type(expected_sha256) is not str or len(expected_sha256) != 64 or
            any(c not in '0123456789abcdef' for c in expected_sha256)):
        raise ValueError('Expected active library SHA256 is invalid')
    # A fresh private snapshot both retains the authenticated descriptor bytes
    # and gives dlopen a unique pathname. Bare /proc/self/fd/N can reuse an old
    # dlopen name after N is closed, even if a new file's digest was verified.
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= 64 * 1024 * 1024:
            raise ValueError('Bounded regular active library required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
        fingerprint = lambda row: (row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns)
        digest = hashlib.sha256(raw).hexdigest()
        if len(raw) != before.st_size or fingerprint(before) != fingerprint(after) or digest != expected_sha256:
            raise ValueError('Active library differs from the pinned launcher')
        with tempfile.TemporaryDirectory(prefix='dog-active-dso-') as temporary:
            snapshot = Path(temporary).resolve() / 'verified-transport.so'
            with os.fdopen(os.open(snapshot, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                                    getattr(os, 'O_NOFOLLOW', 0), 0o400), 'wb') as stream:
                stream.write(raw)
            if hashlib.sha256(snapshot.read_bytes()).hexdigest() != digest:
                raise ValueError('Active library snapshot differs from its pin')
            yield str(snapshot), digest
            if hashlib.sha256(snapshot.read_bytes()).hexdigest() != digest:
                raise ValueError('Pinned active library snapshot changed while loading')
        os.lseek(fd, 0, os.SEEK_SET)
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            final_raw = stream.read(after.st_size + 1)
        if fingerprint(os.fstat(fd)) != fingerprint(after) or hashlib.sha256(final_raw).hexdigest() != digest:
            raise ValueError('Pinned active library changed while loading')
    finally:
        os.close(fd)


def load_library(path, *, expected_sha256=None):
    path = Path(path).resolve(strict=True)
    record_raw = (path.parent/'build-record.json').read_bytes()
    record = json.loads(record_raw.decode('utf-8'))
    with _library_binding(path, expected_sha256) as (load_path, binary_digest):
        for key, source in (('source_sha256', path.parent/'transport.cpp'), ('binary_sha256', path)):
            digest = binary_digest if key == 'binary_sha256' and binary_digest is not None else hashlib.sha256(source.read_bytes()).hexdigest()
            if digest != record[key]:
                raise ValueError('Active native source/binary differs from build record')
        lib = C.CDLL(load_path)
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
    pair_abi = getattr(lib, 'sda_pair_abi', None)
    if pair_abi is not None:
        pair_abi.argtypes = []; pair_abi.restype = C.c_uint32
        if pair_abi() != 1:
            raise ValueError('Active native pair ABI mismatch')
        lib.sda_pair_create.argtypes = [C.c_void_p, C.c_void_p, C.POINTER(C.c_char), C.c_uint32]
        lib.sda_pair_create.restype = C.c_void_p
        lib.sda_pair_cancel.argtypes = [C.c_void_p]; lib.sda_pair_cancel.restype = None
        lib.sda_pair_destroy.argtypes = [C.c_void_p]; lib.sda_pair_destroy.restype = None
        lib.sda_pair_exchange.argtypes = [C.c_void_p,
            C.POINTER(C.c_ubyte), C.c_uint32, C.c_uint64,
            C.POINTER(C.c_ubyte), C.c_uint32, C.c_uint64,
            C.POINTER(Record), C.POINTER(Stats), C.POINTER(C.c_char),
            C.POINTER(Record), C.POINTER(Stats), C.POINTER(C.c_char),
            C.c_uint32, C.POINTER(PairStats)]
        lib.sda_pair_exchange.restype = C.c_int
        lib.sda_pair_owner_settings.argtypes = [C.c_void_p, C.c_uint64, C.c_uint64,
            C.c_int, C.POINTER(PairOwnerSettings), C.POINTER(C.c_char), C.c_uint32]
        lib.sda_pair_owner_settings.restype = C.c_int
        notification = tuple(getattr(lib, name, None) for name in _NOTIFICATION_SYMBOLS)
        if any(symbol is not None for symbol in notification):
            if not all(symbol is not None for symbol in notification):
                raise ValueError('Incomplete native pair notification ABI')
            abi, setter, completion_wait = notification
            abi.argtypes = []; abi.restype = C.c_uint32
            if abi() != 1:
                raise ValueError('Native pair notification ABI mismatch')
            setter.argtypes = list(_NOTIFICATION_SET_ARGUMENT_TYPES); setter.restype = C.c_int
            completion_wait.argtypes = list(_NOTIFICATION_WAIT_ARGUMENT_TYPES)
            completion_wait.restype = C.c_int
    feedback_symbols = tuple(getattr(lib, name, None) for name in _FEEDBACK_SYMBOLS)
    if any(symbol is not None for symbol in feedback_symbols):
        if not all(symbol is not None for symbol in feedback_symbols):
            raise ValueError('Incomplete native feedback codec ABI')
        feedback_abi, batch_decode = feedback_symbols
        feedback_abi.argtypes = []; feedback_abi.restype = C.c_uint32
        if feedback_abi() != 1:
            raise ValueError('Native feedback codec ABI mismatch')
        batch_decode.argtypes = list(_FEEDBACK_ARGUMENT_TYPES); batch_decode.restype = C.c_int
    readiness_symbols = tuple(getattr(lib, name, None) for name in _FUTURE_READINESS_SYMBOLS)
    if any(symbol is not None for symbol in readiness_symbols):
        if not all(symbol is not None for symbol in readiness_symbols):
            raise ValueError('Incomplete native Future readiness ABI')
        readiness_abi, readiness_wait = readiness_symbols
        readiness_abi.argtypes = []; readiness_abi.restype = C.c_uint32
        if readiness_abi() != 1:
            raise ValueError('Native Future readiness ABI mismatch')
        readiness_wait.argtypes = list(_FUTURE_READINESS_ARGUMENT_TYPES)
        readiness_wait.restype = C.c_int
    # Only an explicit authenticated snapshot can bind the opt-in pure codec.
    # Its dlopen pathname is temporary, so it cannot identify the original file.
    if expected_sha256 is not None:
        lib._verified_active_source_binding = _VerifiedActiveSourceBinding(
            _ACTIVE_SOURCE_BINDING_TOKEN, weakref.ref(lib), str(path), str(path.parent/'transport.cpp'),
            str(path.parent/'build-record.json'), expected_sha256, record['source_sha256'],
            hashlib.sha256(record_raw).hexdigest(), str(lib._name),
            _active_source_function_bindings(lib))
        # A caller-set attribute is not a load attestation. Register only this
        # exact post-authentication value, without retaining library/function
        # objects through the weak-key registry value.
        _ACTIVE_SOURCE_BINDINGS[lib] = weakref.ref(lib._verified_active_source_binding)
    return lib


_ACTIVE_SOURCE_BINDING_TOKEN = object()
_ACTIVE_SOURCE_BINDINGS = weakref.WeakKeyDictionary()
_ACTIVE_SOURCE_FUNCTIONS = ('sda_abi', 'sda_create', 'sda_exchange',
                            'sda_emergency_stop', 'sda_destroy',
                            'sda_feedback_decode_abi', 'sda_feedback_decode_batch',
                            'sda_wait_until', 'sda_future_readiness_abi',
                            'sda_wait_future_ready', 'sda_now_ns')


def _active_source_function_bindings(library):
    values = []
    for name in _ACTIVE_SOURCE_FUNCTIONS:
        function = getattr(library, name, None)
        values.append((None, None, None, None, None) if function is None else
                      (function, tuple(function.argtypes or ()), function.restype,
                       getattr(function, 'errcheck', None), function._flags_))
    return tuple(values)


@dataclass(frozen=True)
class _VerifiedActiveSourceBinding:
    token: object
    library: object
    path: str
    source_path: str
    build_record_path: str
    binary_sha256: str
    source_sha256: str
    build_record_sha256: str
    loaded_name: str
    functions: tuple


def verified_active_source_binding(library):
    """Setup-only identity of the actual authenticated, ordinary ABI1 library.

    This grants no output or timing eligibility. Existing libraries loaded
    without an expected SHA have no binding and remain valid on the default
    transport path. No source file or descriptor is opened by this getter.
    """
    value = getattr(library, '_verified_active_source_binding', None)
    registered = _ACTIVE_SOURCE_BINDINGS.get(library) if type(library) is C.CDLL else None
    if (type(library) is not C.CDLL or registered is None or registered() is not value or
            type(value) is not _VerifiedActiveSourceBinding or
            value.token is not _ACTIVE_SOURCE_BINDING_TOKEN or value.library() is not library or
            value.loaded_name != str(library._name) or len(value.functions) != len(_ACTIVE_SOURCE_FUNCTIONS) or
            any(getattr(library, name, None) is not original or
                (original is not None and (tuple(original.argtypes or ()) != arguments or
                 original.restype is not result or getattr(original, 'errcheck', None) is not errorcheck or
                 original._flags_ != flags))
                for name, (original, arguments, result, errorcheck, flags) in
                    zip(_ACTIVE_SOURCE_FUNCTIONS, value.functions))):
        raise ValueError('Explicit authenticated original active library binding required')
    return value


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


_WAIT_ARGUMENT_TYPES = (C.c_int, C.c_uint64, C.c_uint32,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)
_EMPTY_WAIT_ERROR = bytes(256)
_NOTIFICATION_SYMBOLS = ('sda_pair_notification_abi', 'sda_pair_set_notification',
                         'sda_pair_wait_completion')
_NOTIFICATION_SET_ARGUMENT_TYPES = (C.c_void_p, C.c_int, C.c_int,
                                  C.POINTER(C.c_char), C.c_uint32)
_NOTIFICATION_WAIT_ARGUMENT_TYPES = (C.c_void_p, C.c_uint64, C.c_uint64,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)
_FEEDBACK_SYMBOLS = ('sda_feedback_decode_abi', 'sda_feedback_decode_batch')
_FEEDBACK_ARGUMENT_TYPES = (C.POINTER(Record), C.c_uint32, C.c_int,
    C.POINTER(FeedbackDecoded), C.POINTER(C.c_char), C.c_uint32)
_FUTURE_READINESS_SYMBOLS = ('sda_future_readiness_abi', 'sda_wait_future_ready')
_FUTURE_READINESS_ARGUMENT_TYPES = (C.c_int, C.c_int, C.c_uint64,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)


class NativeFeedbackBatchDecoder:
    """Optional pure six-feedback codec, with exact legacy fallback semantics.

    No session, FD, transport, clock or position wrapping is accessed. The
    caller must provide its stable owned records after native writers joined.
    It retains original raw records and runs decode_records when None returns.
    Reusable scratch belongs to this instance; contended calls safely fall
    back rather than changing the legacy pipeline's concurrency semantics.
    """
    def __init__(self, library):
        from . import rs05_trial_protocol
        self._library = library
        self._protocol = rs05_trial_protocol
        self._feedback_type = rs05_trial_protocol.Type2Feedback
        self._busy = threading.Lock()
        self._decoded = (FeedbackDecoded * 6)()
        self._error = C.create_string_buffer(256)
        self._abi = self._function = None
        symbols = tuple(getattr(library, name, None) for name in _FEEDBACK_SYMBOLS)
        if not any(symbol is not None for symbol in symbols):
            self.available = False
            return
        if not all(symbol is not None for symbol in symbols):
            raise ValueError('Incomplete native feedback codec ABI')
        self._abi, self._function = symbols
        self._verify()
        if self._abi() != 1: raise ValueError('Native feedback codec ABI mismatch')
        self.available = True

    def _verify(self):
        abi, function = self._abi, self._function
        if (getattr(self._library, _FEEDBACK_SYMBOLS[0], None) is not abi or
                getattr(self._library, _FEEDBACK_SYMBOLS[1], None) is not function or
                not isinstance(abi, C._CFuncPtr) or abi._flags_ != C._FUNCFLAG_CDECL or
                tuple(abi.argtypes or ()) != () or abi.restype is not C.c_uint32 or
                getattr(abi, 'errcheck', None) is not None or
                not isinstance(function, C._CFuncPtr) or function._flags_ != C._FUNCFLAG_CDECL or
                tuple(function.argtypes or ()) != _FEEDBACK_ARGUMENT_TYPES or
                function.restype is not C.c_int or getattr(function, 'errcheck', None) is not None):
            raise ValueError('Exact GIL-releasing native feedback codec ABI required')

    def decode(self, records, first_id):
        if not self.available:
            if any(getattr(self._library, name, None) is not None for name in _FEEDBACK_SYMBOLS):
                raise ValueError('Native feedback codec capability changed')
            return None
        self._verify()
        if (type(first_id) is not int or first_id not in (1, 7) or
                type(records) is not Record * 6 or records._b_base_ is not None or
                records._b_needsfree_ != 1 or self._protocol.POSITION_MIN != -12.57):
            return None
        if not self._busy.acquire(blocking=False): return None
        try:
            self._error.raw = _EMPTY_WAIT_ERROR
            status = self._function(records, 6, first_id, self._decoded, self._error, 256)
            if status not in (-1, 0, 1):
                raise ValueError('Native feedback codec returned an invalid status')
            if status: return None
            if self._error.value:
                raise ValueError('Native feedback codec returned success with an error')
            rows = {}
            for row in self._decoded:
                value = self._feedback_type(row.mode_state, row.fault_bits, row.position_u16,
                    row.protocol_position_rad, row.velocity_rad_s, row.torque_nm, row.temperature_c)
                rows[(row.motor_id, 'feedback')] = (value, row.request_started_ns, row.received_ns)
            return rows
        finally:
            self._error.raw = _EMPTY_WAIT_ERROR
            self._busy.release()


def _verify_notification_function(library, name, arguments):
    function = getattr(library, name, None)
    if (not isinstance(function, C._CFuncPtr) or function._flags_ != C._FUNCFLAG_CDECL or
            tuple(function.argtypes or ()) != arguments or function.restype is not C.c_int or
            getattr(function, 'errcheck', None) is not None):
        raise ValueError('Exact GIL-releasing native pair notification ABI required')
    return function


_ACTIVE_WAITER_CREATIONS = weakref.WeakKeyDictionary()
_ACTIVE_SESSION_CREATIONS = weakref.WeakKeyDictionary()


def verified_owned_waiter_creation(waiter):
    value = _ACTIVE_WAITER_CREATIONS.get(waiter)
    if (type(waiter) is not _OwnedActiveWaiter or value is None or
            value[0]() is not waiter._library or value[1] != waiter._cancel_fd or
            value[2] is None or _readiness_fd_binding(value[1]) != value[2] or
            value[3]() is not waiter._owner):
        raise ValueError('Original owned waiter cancellation creation binding required')
    waiter._verify_function()
    if waiter.future_readiness_available is not True:
        raise ValueError('Original owned waiter readiness capability required')
    return value[1], value[2]


def verified_active_session_creation(session):
    value = _ACTIVE_SESSION_CREATIONS.get(session)
    if (type(session) is not ActiveSession or value is None or
            value[0]() is not session.lib or value[1] != session._handle or
            value[2] != session._cancel_fd or value[3] is None or
            _readiness_fd_binding(value[2]) != value[3]):
        raise ValueError('Original active session cancellation creation binding required')
    return value[2], value[3]


class _OwnedActiveWaiter:
    """One coordinator's scratch buffers; caller owns the verified lib and FD.

    Call the same GIL-releasing active wait without serial or motor I/O. Keep
    the original absolute deadlines, cancellation scope and boot checks.
    Reuse never certifies latency and does not replace the legacy wait API.
    """
    def __init__(self, library, cancel_fd, spin_us):
        if (type(cancel_fd) is not int or not 0 <= cancel_fd < 2**31 or
                type(spin_us) is not int or spin_us not in (200, 500)):
            raise ValueError('Invalid owned active wait arguments')
        waiter = getattr(library, 'sda_wait_until', None)
        if waiter is None:
            raise ActiveWaitError('Optional active release wait is unavailable in this library')
        try: original_cancel_binding = _readiness_fd_binding(cancel_fd)
        except ActiveWaitError: original_cancel_binding = None
        self._library, self._waiter = library, waiter
        self._verify_function()
        abi = getattr(library, 'sda_abi', None)
        value = abi() if callable(abi) else None
        if type(value) is not int or value != 1:
            raise ValueError('Active wait ABI mismatch')
        self._cancel_fd, self._spin_us = cancel_fd, spin_us
        self._owner = threading.current_thread()
        self._busy = threading.Lock()
        self._actual, self._error = C.c_uint64(), C.create_string_buffer(256)
        self._actual_ptr = C.byref(self._actual)
        self._readiness_group = None
        symbols = tuple(getattr(library, name, None) for name in _FUTURE_READINESS_SYMBOLS)
        self._readiness_abi, self._readiness_wait = symbols
        if any(symbol is not None for symbol in symbols):
            if not all(symbol is not None for symbol in symbols):
                raise ValueError('Incomplete native Future readiness ABI')
            self._verify_readiness()
            if self._readiness_abi() != 1:
                raise ValueError('Native Future readiness ABI mismatch')
        _ACTIVE_WAITER_CREATIONS[self] = (weakref.ref(library), cancel_fd,
            original_cancel_binding, weakref.ref(self._owner))

    def _verify_function(self):
        waiter = self._waiter
        if (getattr(self._library, 'sda_wait_until', None) is not waiter or
                not isinstance(waiter, C._CFuncPtr) or
                waiter._flags_ != C._FUNCFLAG_CDECL or
                tuple(waiter.argtypes or ()) != _WAIT_ARGUMENT_TYPES or
                waiter.restype is not C.c_int or
                getattr(waiter, 'errcheck', None) is not None):
            raise ValueError('Exact GIL-releasing active wait ABI required')

    def _verify_readiness(self):
        symbols = tuple(getattr(self._library, name, None) for name in _FUTURE_READINESS_SYMBOLS)
        if any(actual is not expected for actual, expected in
               zip(symbols, (self._readiness_abi, self._readiness_wait))):
            raise ValueError('Native Future readiness capability changed')
        if self._readiness_abi is None:
            return False
        abi, waiter = symbols
        if (not isinstance(abi, C._CFuncPtr) or abi._flags_ != C._FUNCFLAG_CDECL or
                tuple(abi.argtypes or ()) != () or abi.restype is not C.c_uint32 or
                getattr(abi, 'errcheck', None) is not None or
                not isinstance(waiter, C._CFuncPtr) or waiter._flags_ != C._FUNCFLAG_CDECL or
                tuple(waiter.argtypes or ()) != _FUTURE_READINESS_ARGUMENT_TYPES or
                waiter.restype is not C.c_int or getattr(waiter, 'errcheck', None) is not None):
            raise ValueError('Exact GIL-releasing native Future readiness ABI required')
        return True

    @property
    def future_readiness_available(self):
        self._verify_function()
        return self._verify_readiness()

    def readiness_group(self, futures):
        """Register original Futures; None explicitly requests the legacy poll.

        Notifications are hints only. Callers recheck these same Futures and
        their original absolute deadline after each actual native wake.
        """
        if threading.current_thread() is not self._owner:
            raise ActiveWaitError('Owned Future readiness group created from another thread')
        if not self.future_readiness_available:
            return None
        if (type(futures) is not tuple or not 1 <= len(futures) <= 16 or
                any(type(future) is not Future for future in futures) or
                len({id(future) for future in futures}) != len(futures)):
            return None
        if self._readiness_group is not None or self._busy.locked():
            raise ActiveWaitError('Overlapping owned Future readiness group')
        return _FutureReadinessGroup(self, futures)

    def __call__(self, deadline_ns):
        if threading.current_thread() is not self._owner:
            raise ActiveWaitError('Owned active wait called from another thread')
        if type(deadline_ns) is not int or not 0 < deadline_ns < 2**64:
            raise ValueError('Invalid bounded active release wait arguments')
        if not self._busy.acquire(blocking=False):
            raise ActiveWaitError('Reentrant owned active wait')
        try:
            self._verify_function()
            self._actual.value = 0
            self._error.raw = _EMPTY_WAIT_ERROR
            status = self._waiter(self._cancel_fd, deadline_ns, self._spin_us,
                self._actual_ptr, self._error, 256)
            if status:
                raise ActiveWaitError(self._error.value.decode('utf-8', errors='replace') or
                                      'Native active wait failed without an error message')
            if self._error.value:
                raise ActiveWaitError('Active wait returned success with an error')
            if self._actual.value < deadline_ns:
                raise ActiveWaitError('Active release wait returned a backdated time')
            return self._actual.value
        finally:
            # No GIL-releasing memset or stale wake/error between calls.
            self._actual.value = 0
            self._error.raw = _EMPTY_WAIT_ERROR
            self._busy.release()


def _readiness_fd_binding(fd, *, pipe_mode=None, require_readable=True):
    """Retain identity and endpoint flags, never just a pipe's shared inode."""
    try:
        row = os.fstat(fd)
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        if pipe_mode is not None:
            if (not stat.S_ISFIFO(row.st_mode) or not flags & os.O_NONBLOCK or
                    flags & os.O_ACCMODE != pipe_mode):
                raise ActiveWaitError('Future readiness pipe endpoint/mode changed')
        elif require_readable and flags & os.O_ACCMODE == os.O_WRONLY:
            raise ActiveWaitError('Future readiness cancellation descriptor is not readable')
        # Darwin may add its internal no-SIGPIPE flag after the first write.
        # Pin the two contract flags, not unrelated kernel bookkeeping bits.
        return (row.st_dev, row.st_ino, row.st_rdev, stat.S_IFMT(row.st_mode),
                flags & (os.O_ACCMODE | os.O_NONBLOCK))
    except OSError as error:
        raise ActiveWaitError('Future readiness descriptor binding failed') from error


class _FutureReadinessGroup:
    """Owner-bound per-join notification pipe; exact Future callbacks only.

    Weak callbacks cannot retain a closed group when GC is deferred. Every
    callback is bounded to one nonblocking byte and never takes a Future result
    or performs emergency cleanup. The coordinator alone waits/drains/closes.
    """
    def __init__(self, owner, futures):
        self._owner = owner
        self._futures = futures
        self._lock = threading.Lock()
        self._active = False
        self._closed = False
        self._waiting = False
        self._failure = None
        self._fds = None
        self._bindings = None
        self._allocated_bindings = None
        # Exact original construction cancellation identity; a new FD cannot
        # become authoritative by changing the mutable Python waiter fields.
        _, self._cancel_binding = verified_owned_waiter_creation(owner)
        try:
            self._fds = os.pipe()
            self._allocated_bindings = tuple(_readiness_fd_binding(fd, require_readable=False)[:4]
                                              for fd in self._fds)
            for fd in self._fds:
                os.set_blocking(fd, False)
                os.set_inheritable(fd, False)
            self._bindings = tuple(_readiness_fd_binding(fd, pipe_mode=mode)
                for fd, mode in zip(self._fds, (os.O_RDONLY, os.O_WRONLY)))
            self._active = True
            owner._readiness_group = self
            reference = weakref.ref(self)

            def publish_hint(original_future):
                group = reference()
                if group is not None:
                    group._publish_hint(original_future)

            # add_done_callback may synchronously call us for an already-done
            # Future. The complete pipe/identity binding is active beforehand,
            # and registration never holds the callback lock.
            for future in futures:
                future.add_done_callback(publish_hint)
        except BaseException as primary:
            failures = []
            # Earlier registrations may already be publishing on a worker.
            # Rollback uses the same write/close fence as normal teardown.
            with self._lock:
                self._active = False
                self._closed = True
                if owner._readiness_group is self:
                    owner._readiness_group = None
                if self._fds is not None:
                    for index, fd in enumerate(self._fds):
                        try:
                            current = _readiness_fd_binding(fd, require_readable=False)
                            if (self._allocated_bindings is None or
                                    current[:4] != self._allocated_bindings[index] or
                                    (self._bindings is not None and current != self._bindings[index])):
                                raise ActiveWaitError('Future readiness pipe binding changed during setup cleanup')
                            os.close(fd)
                        except BaseException as cleanup:
                            failures.append(cleanup)
                self._fds = None
                self._futures = ()
            for cleanup in failures:
                primary.add_note('Future readiness setup cleanup failed: ' + str(cleanup))
            raise

    def _check_owner(self):
        if threading.current_thread() is not self._owner._owner:
            raise ActiveWaitError('Owned Future readiness group used from another thread')
        if self._closed or self._owner._readiness_group is not self:
            raise ActiveWaitError('Future readiness group is closed or no longer current')

    def _verify_fds(self):
        for fd, mode, expected in zip(self._fds, (os.O_RDONLY, os.O_WRONLY), self._bindings):
            if _readiness_fd_binding(fd, pipe_mode=mode) != expected:
                raise ActiveWaitError('Future readiness pipe binding changed')
        if _readiness_fd_binding(self._owner._cancel_fd) != self._cancel_binding:
            raise ActiveWaitError('Future readiness cancellation descriptor binding changed')

    def _publish_hint(self, original_future):
        with self._lock:
            if (not self._active or self._closed or
                    self._owner._readiness_group is not self or
                    not any(original_future is expected for expected in self._futures)):
                return
            try:
                self._verify_fds()
                if os.write(self._fds[1], b'\x01') != 1:
                    raise ActiveWaitError('Future readiness hint write was incomplete')
            except BaseException as error:
                # Future.set_result invokes callbacks on the publishing worker.
                # Report faults on the coordinator's next wait/close, without
                # raising into that worker or inventing completion readiness.
                if self._failure is None:
                    self._failure = error

    def wait(self, deadline_ns):
        self._check_owner()
        if type(deadline_ns) is not int or not 0 < deadline_ns < 2**64:
            raise ValueError('Invalid bounded Future readiness deadline')
        owner = self._owner
        if not owner._busy.acquire(blocking=False):
            raise ActiveWaitError('Reentrant owned Future readiness wait')
        self._waiting = True
        try:
            owner._verify_function()
            if not owner._verify_readiness():
                raise ActiveWaitError('Native Future readiness capability became unavailable')
            with self._lock:
                self._verify_fds()
                if self._failure is not None:
                    raise ActiveWaitError('Future readiness callback failed') from self._failure
            owner._actual.value = 0
            owner._error.raw = _EMPTY_WAIT_ERROR
            before = time.monotonic_ns()
            status = owner._readiness_wait(owner._cancel_fd, self._fds[0], deadline_ns,
                owner._actual_ptr, owner._error, 256)
            after = time.monotonic_ns()
            if status not in (0, 1):
                raise ActiveWaitError(owner._error.value.decode('utf-8', errors='replace') or
                                      'Native Future readiness wait failed without an error message')
            actual = owner._actual.value
            if (owner._error.value or not before <= actual <= after or
                    (status == 0 and actual >= deadline_ns) or
                    (status == 1 and actual < deadline_ns)):
                raise ActiveWaitError('Invalid native Future readiness wake/status')
            with self._lock:
                self._verify_fds()
                if self._failure is not None:
                    raise ActiveWaitError('Future readiness callback failed') from self._failure
                if status == 0:
                    try:
                        hint = os.read(self._fds[0], len(self._futures))
                    except OSError as error:
                        raise ActiveWaitError('Future readiness hint drain failed') from error
                    if not hint:
                        raise ActiveWaitError('Future readiness pipe reached EOF')
            return {'kind': 'NOTIFIED' if status == 0 else 'DEADLINE', 'actual_ns': actual}
        finally:
            owner._actual.value = 0
            owner._error.raw = _EMPTY_WAIT_ERROR
            self._waiting = False
            owner._busy.release()

    def close(self):
        if threading.current_thread() is not self._owner._owner:
            raise ActiveWaitError('Owned Future readiness group closed from another thread')
        if self._closed:
            return
        if self._waiting:
            raise ActiveWaitError('Future readiness group closed during native wait')
        failures = []
        with self._lock:
            # Fence callbacks before closing either endpoint. A late callback
            # sees inactive and cannot write to a subsequently reused FD.
            self._active = False
            self._closed = True
            if self._owner._readiness_group is self:
                self._owner._readiness_group = None
            if self._failure is not None:
                failures.append(self._failure)
            for fd, mode, expected in zip(self._fds, (os.O_RDONLY, os.O_WRONLY), self._bindings):
                try:
                    if _readiness_fd_binding(fd, pipe_mode=mode) != expected:
                        raise ActiveWaitError('Future readiness pipe binding changed before close')
                    os.close(fd)
                except BaseException as error:
                    # Never close an unrelated descriptor after external reuse.
                    failures.append(error)
            self._fds = None
            self._futures = ()
        if failures:
            raise ActiveWaitError('Future readiness group cleanup/publication failed') from failures[0]

    def __enter__(self):
        self._check_owner()
        return self

    def __exit__(self, kind, error, traceback):
        try:
            self.close()
        except BaseException as cleanup:
            if error is None:
                raise
            error.add_note('Future readiness cleanup failed: ' + str(cleanup))


def make_owned_waiter(library, cancel_fd, *, spin_us=500):
    """Explicit reusable wait; owns no FD and grants no motor output.

    Construct after load_library has pinned the source/binary/clock ABI. The
    coordinator must keep that library and its cancellation FD alive, and
    retain all existing boot, freshness, signal and native deadline guards.
    """
    return _OwnedActiveWaiter(library, cancel_fd, spin_us)


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


def _stop_reply_has_fault(record):
    # The encoded big-endian CAN ID puts type in bits27..31 and STOP reply
    # faults in bits19..24. Inspect owned header bytes without copying frames.
    return (record.tx[2] >> 3 == 4 and record.received == 17 and
            bool(record.rx[2] & 1 or record.rx[3] & 0xf8))


@dataclass(frozen=True)
class PreparedExchangeCapability:
    """Explicit transport contract; a callable attribute alone is insufficient."""
    schema: str = 'singularitydog.active-prepared-exchange.v1'
    deadline_return: str = 'absolute_tighten_only'


PREPARED_EXCHANGE_CAPABILITY = PreparedExchangeCapability()


class ActiveSession:
    """One owner, one bus, explicit immutable limits. Caller owns all FDs.

    Call emergency_stop after any fault and save its evidence. It does not
    unpoison/re-arm the session. close releases ownership, never closes the FD.
    No destructor sends motor commands.
    """
    prepared_exchange_capability = PREPARED_EXCHANGE_CAPABILITY

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
        self._cancel_fd = cancel_fd  # Original native session cancellation setup binding.
        self.busy, self.poisoned, self._handle = threading.Lock(), False, None
        self._phase_pair = None
        self._limits = limits
        self.boot_id = boot_id
        error = C.create_string_buffer(256)
        try: original_cancel_binding = _readiness_fd_binding(cancel_fd)
        except ActiveWaitError: original_cancel_binding = None
        self._handle = library.sda_create(fd, cancel_fd, boot_fd, boot_id.encode('ascii'),
            first_id, C.byref(limits), gap_ns, window, error, len(error))
        if not self._handle:
            raise ValueError(error.value.decode())
        _ACTIVE_SESSION_CREATIONS[self] = (weakref.ref(library), self._handle,
            cancel_fd, original_cancel_binding)

    def _call(self, wires, timeout_ns, send_only, deadline_ns=None, before_native=None):
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Concurrent active session use')
        try:
            if not self._handle:
                raise RuntimeError('Active session closed')
            if self.poisoned:
                raise RuntimeError('Session poisoned; active retry prohibited')
            if before_native is not None and (send_only or not callable(before_native)):
                raise ValueError('Prepared exchange requires a callable acknowledged-exchange hook')
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
            if before_native is None:
                # Preserve the existing default and send-only call path.
                status = self.lib.sda_exchange(self._handle, raw, len(wires), int(send_only),
                    native_deadline_ns, records, C.byref(stats), error, len(error))
            else:
                native_call = self.lib.sda_exchange
                arguments = [self._handle, raw, len(wires), int(send_only),
                    native_deadline_ns, records, C.byref(stats), error, len(error)]
                if time.monotonic_ns() >= native_deadline_ns:
                    raise TimeoutError('Active deadline expired before prepared publication')
                # The owner hook checks cancellation before/after publication.
                # It may narrow the shared input-age deadline, never extend it.
                # C++ retains its independent cancel/boot checks before writing.
                narrowed = before_native()
                if narrowed is not None:
                    if type(narrowed) is not int or not 0 < narrowed <= native_deadline_ns:
                        raise ValueError('Prepared hook must only tighten the absolute deadline')
                    native_deadline_ns = narrowed
                if time.monotonic_ns() >= native_deadline_ns:
                    raise TimeoutError('Active deadline expired after prepared publication')
                arguments[4] = native_deadline_ns
                status = native_call(*arguments)
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

    def exchange(self, wires, *, timeout_ns=100_000_000, deadline_ns=None, before_native=None):
        """Publish only after preparation when explicitly given an owner hook.

        The hook runs once with session/buffer ownership held and may return a
        tighter absolute deadline (or None). Failure poisons without this batch
        being written. The default call path and native limits are unchanged.
        """
        return self._call(wires, timeout_ns, False, deadline_ns, before_native)

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
        if self._phase_pair is not None:
            raise RuntimeError('Close/join the borrowing native pair before session close')
        if not self.busy.acquire(blocking=False):
            raise RuntimeError('Cannot close an active native call')
        try:
            if self._handle:
                self.lib.sda_destroy(self._handle)
                self._handle = None
        finally:
            # Destroyed or uncertain handle lifetime cannot retain setup authority.
            _ACTIVE_SESSION_CREATIONS.pop(self, None)
            self.busy.release()

    def __del__(self):
        handle = getattr(self, '_handle', None)
        if handle:
            self.lib.sda_destroy(handle)
            self._handle = None


class ActivePhasePair:
    """Opt-in persistent C++ serial owners borrowing two existing sessions.

    One coordinator call validates both batches and releases the C++ owners on
    the same generation. Existing individual 17-byte writes, original absolute
    deadlines and sticky ambiguity remain unchanged. The ordinary session API
    still works between paired phases. Cancel and join this pair before emergency
    STOP; close/destroy it before session close. This creates no arming permission.

    Linux placement controls must be explicitly applied/read back by the runner;
    synthetic macOS tests can use the transport but cannot claim those controls.
    """
    capability = 'singularitydog.native-active-phase-pair.v1'

    def __init__(self, front_session, rear_session):
        sessions = (front_session, rear_session)
        if (any(type(s) is not ActiveSession for s in sessions) or
                front_session is rear_session or front_session.first_id != 1 or
                rear_session.first_id != 7 or front_session.lib is not rear_session.lib):
            raise ValueError('Exact independent front/rear active sessions from one library required')
        self.sessions = dict(zip(('front', 'rear'), sessions))
        self.lib = front_session.lib
        if not callable(getattr(self.lib, 'sda_pair_abi', None)) or self.lib.sda_pair_abi() != 1:
            raise ValueError('Optional native active phase pair ABI unavailable')
        self._lock = threading.RLock(); self._busy = threading.Lock()
        self._handle = None; self._closed = False; self._cancelled = False
        self._pending = None; self._generation = 0
        self._idle = threading.Event(); self._idle.set()
        self.last_phase = None; self.owner_settings_history = []
        self.last_completed_bus_results = None
        self._settings_applied = False
        self._settings_ready = True
        self._coordinator_thread = None
        self._coordinator_original = None
        self._coordinator_prctl = None
        self._coordinator_settings = None
        self.coordinator_settings_history = []
        self._coordinator_settings_applied = False
        self._coordinator_configuration_attempted = False
        self._executor_closed = False
        self._lost_submission_entered = None
        self._current_futures = None
        self._current_publication = None
        self._notification_fds = None
        self._notification_fingerprints = None
        self._notification_wait_lock = threading.Lock()
        self._notification_waiter = None
        self._notification_actual = C.c_uint64()
        self._notification_error = C.create_string_buffer(256)
        acquired = []
        try:
            for session in sessions:
                if not session.busy.acquire(blocking=False):
                    raise RuntimeError('Session busy while constructing native phase pair')
                acquired.append(session)
                if not session._handle or session.poisoned or session._phase_pair is not None:
                    raise RuntimeError('Closed/poisoned/borrowed native active session')
            error = C.create_string_buffer(256)
            self._handle = self.lib.sda_pair_create(front_session._handle, rear_session._handle,
                                                   error, len(error))
            if not self._handle:
                raise ValueError(error.value.decode('utf-8', errors='replace'))
            notification = tuple(getattr(self.lib, name, None) for name in _NOTIFICATION_SYMBOLS)
            if any(symbol is not None for symbol in notification):
                if not all(symbol is not None for symbol in notification):
                    raise ValueError('Incomplete native pair notification ABI')
                abi, _, _ = notification
                if (not isinstance(abi, C._CFuncPtr) or abi._flags_ != C._FUNCFLAG_CDECL or
                        tuple(abi.argtypes or ()) != () or abi.restype is not C.c_uint32 or
                        getattr(abi, 'errcheck', None) is not None or abi() != 1):
                    raise ValueError('Native pair notification ABI mismatch')
                setter = _verify_notification_function(self.lib, _NOTIFICATION_SYMBOLS[1],
                                                       _NOTIFICATION_SET_ARGUMENT_TYPES)
                self._notification_waiter = _verify_notification_function(self.lib,
                    _NOTIFICATION_SYMBOLS[2], _NOTIFICATION_WAIT_ARGUMENT_TYPES)
                self._notification_fds = os.pipe()
                for fd in self._notification_fds:
                    os.set_blocking(fd, False); os.set_inheritable(fd, False)
                self._notification_fingerprints = tuple(
                    (os.fstat(fd).st_dev, os.fstat(fd).st_ino, os.fstat(fd).st_rdev)
                    for fd in self._notification_fds)
                status = setter(self._handle, *self._notification_fds, error, len(error))
                if status or error.value:
                    raise ValueError(error.value.decode('utf-8', errors='replace') or
                                     'Native pair notification binding failed')
            for session in sessions: session._phase_pair = self
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='policy-native-pair')
            # Start alongside the native owners, before the caller may pin
            # itself elsewhere. No placement or libc access is selected here.
            self._executor.submit(self._start_coordinator).result()
        except BaseException:
            executor = getattr(self, '_executor', None)
            if executor is not None: executor.shutdown(wait=True, cancel_futures=False)
            if self._handle:
                self.lib.sda_pair_destroy(self._handle); self._handle = None
            self._close_notification_pipe()
            for session in sessions:
                if session._phase_pair is self: session._phase_pair = None
            raise
        finally:
            for session in reversed(acquired): session.busy.release()

    def _start_coordinator(self):
        self._coordinator_thread = threading.current_thread()

    @property
    def completion_notification_available(self):
        """Optional native hint wait; this does not certify Future readiness."""
        return bool(self._notification_waiter is not None and
                    self._notification_fds is not None and not self._closed)

    def owns_futures(self, futures):
        """Only this exact current generation's genuine public Futures match."""
        with self._lock:
            return (type(futures) is dict and set(futures) == {'front', 'rear'} and
                    self._current_futures is not None and all(
                        type(futures[scope]) is Future and
                        futures[scope] is self._current_futures[scope]
                        for scope in ('front', 'rear')))

    def publication_complete(self, futures):
        """Publication fence, separate from native join and Future.done()."""
        with self._lock:
            if not self.owns_futures(futures):
                raise ValueError('Exact current native pair Futures required')
            publication = self._current_publication
            if not publication['event'].is_set(): return False
            if publication['error'] is not None: raise publication['error']
            return True

    def wait_published(self, futures, timeout=0.35):
        """Bounded coordinator publication join after native writers finish.

        Periodic callers pass the remaining original host deadline when both
        genuine Futures are ready. Non-periodic collection keeps the existing
        native join bound. Neither use restarts an owner's serial deadline.
        """
        with self._lock:
            if not self.owns_futures(futures):
                raise ValueError('Exact current native pair Futures required')
            publication = self._current_publication
        if not publication['event'].wait(timeout=timeout):
            raise TimeoutError('Native pair Future publication did not complete within join bound')
        if publication['error'] is not None: raise publication['error']

    def _close_notification_pipe(self):
        if self._notification_fds is not None:
            for fd in self._notification_fds: os.close(fd)
            self._notification_fds = None

    def _signal_published_completion(self):
        # Both public Futures already hold their real generation's results.
        # Keep a second hint: native completion may wake before Python gets
        # the GIL to decode/publish. No callback outlives the coordinator.
        if self._notification_fds is None: return
        for fd, expected in zip(self._notification_fds, self._notification_fingerprints):
            actual = os.fstat(fd)
            if (actual.st_dev, actual.st_ino, actual.st_rdev) != expected:
                raise ActiveWaitError('Native completion pipe binding changed')
        try: os.write(self._notification_fds[1], b'\x01')
        except OSError as error:
            if error.errno not in (errno.EAGAIN, errno.EWOULDBLOCK): raise

    def wait_completion(self, futures, *, tick_ns, deadline_ns):
        """GIL-releasing bounded wait for a hint or tick, never a ready proof.

        Native cancellation and the caller's original hard deadline retain
        priority over a readable hint. The caller must recheck both genuine
        Futures after every return. Old-generation identities are rejected.
        """
        with self._lock:
            if self._closed or not self._handle:
                raise ActiveWaitError('Native phase pair closed')
            if not self.owns_futures(futures):
                raise ValueError('Exact current native pair Futures required')
            if not self.completion_notification_available:
                raise ActiveWaitError('Optional native pair completion notification unavailable')
            if (type(tick_ns) is not int or type(deadline_ns) is not int or
                    not 0 < tick_ns <= deadline_ns < 2**64):
                raise ValueError('Invalid bounded native completion wait arguments')
            waiter = _verify_notification_function(self.lib, _NOTIFICATION_SYMBOLS[2],
                                                   _NOTIFICATION_WAIT_ARGUMENT_TYPES)
            if waiter is not self._notification_waiter:
                raise ValueError('Native pair notification function changed')
            if not self._notification_wait_lock.acquire(blocking=False):
                raise ActiveWaitError('Reentrant native completion wait')
            handle = self._handle
        try:
            before = time.monotonic_ns()
            self._notification_actual.value = 0; self._notification_error.raw = _EMPTY_WAIT_ERROR
            status = waiter(handle, tick_ns, deadline_ns, C.byref(self._notification_actual),
                            self._notification_error, 256)
            if status not in (0, 1):
                raise ActiveWaitError(self._notification_error.value.decode('utf-8', errors='replace') or
                                      'Native completion wait failed without an error message')
            actual = self._notification_actual.value
            if (self._notification_error.value or not before <= actual < deadline_ns or
                    (status == 0 and actual < tick_ns)):
                raise ActiveWaitError('Native completion wait returned invalid timing/error evidence')
            return {'kind': 'NOTIFIED' if status == 1 else 'TICK', 'actual_ns': actual}
        finally:
            self._notification_actual.value = 0; self._notification_error.raw = _EMPTY_WAIT_ERROR
            self._notification_wait_lock.release()

    @property
    def coordinator_settings(self):
        """Latest actual Python coordinator readback, separate from both buses."""
        return None if self._coordinator_settings is None else dict(self._coordinator_settings)

    def _change_coordinator_settings(self, cpu_mask, timer_slack_ns, restore):
        """Run only on the persistent executor, including exact restoration."""
        from . import thread_timer_slack
        original = self._coordinator_original
        row = {'native_tid': threading.get_native_id(), 'cpu_mask': None,
               'timer_slack_ns': None, 'original_cpu_mask': None,
               'original_timer_slack_ns': None, 'status': -1,
               'configured': 0, 'restored': 0, 'applied': False}
        error = None
        try:
            if threading.current_thread() is not self._coordinator_thread:
                raise RuntimeError('Native pair coordinator thread changed')
            thread_timer_slack.require_supported_platform()
            if not hasattr(os, 'sched_getaffinity') or not hasattr(os, 'sched_setaffinity'):
                raise RuntimeError('Native pair coordinator requires Linux thread affinity')
            if self._coordinator_prctl is None:
                self._coordinator_prctl = thread_timer_slack._load_prctl()
            if original is None:
                cpus = os.sched_getaffinity(0)
                slack = self._coordinator_prctl.get()
                if (not cpus or any(type(cpu) is not int or cpu < 0 for cpu in cpus) or
                        type(slack) is not int or slack <= 0):
                    raise RuntimeError('Exact original coordinator affinity/slack required')
                original = self._coordinator_original = (frozenset(cpus), slack)
            row.update(original_cpu_mask=sum(1 << cpu for cpu in original[0]),
                       original_timer_slack_ns=original[1])
            target_cpus = original[0] if restore else {
                cpu for cpu in range(64) if cpu_mask & (1 << cpu)}
            target_slack = original[1] if restore else timer_slack_ns
            # Any partially applied setting requires a same-thread restoration.
            self._coordinator_settings_applied = True
            row['configured'] = 1
            if restore:
                # Try both originals even if one restoration syscall fails.
                failures = []
                try: os.sched_setaffinity(0, target_cpus)
                except BaseException as failure: failures.append(failure)
                try: self._coordinator_prctl.set(target_slack)
                except BaseException as failure: failures.append(failure)
                if failures: raise failures[0]
            else:
                os.sched_setaffinity(0, target_cpus)
                self._coordinator_prctl.set(target_slack)
            row['cpu_mask'] = sum(1 << cpu for cpu in os.sched_getaffinity(0))
            row['timer_slack_ns'] = self._coordinator_prctl.get()
            if (row['cpu_mask'] != sum(1 << cpu for cpu in target_cpus) or
                    row['timer_slack_ns'] != target_slack):
                raise RuntimeError('Native pair coordinator affinity/slack readback mismatch')
            row.update(status=0, restored=int(restore), applied=not restore)
            if restore: self._coordinator_settings_applied = False
        except BaseException as failure:
            error = failure
            # Preserve whatever actual values remain after a partial failure.
            if self._coordinator_prctl is not None:
                try: row['timer_slack_ns'] = self._coordinator_prctl.get()
                except BaseException: pass
            if original is not None:
                row.update(original_cpu_mask=sum(1 << cpu for cpu in original[0]),
                           original_timer_slack_ns=original[1])
                try: row['cpu_mask'] = sum(1 << cpu for cpu in os.sched_getaffinity(0))
                except BaseException: pass
        self._coordinator_settings = dict(row)
        self.coordinator_settings_history.append({'restore': restore,
            'coordinator': dict(row), 'status': row['status'],
            'error': '' if error is None else type(error).__name__ + ': ' + str(error)})
        if error is not None:
            error.native_pair_coordinator_settings = dict(row)
            raise error
        return dict(row)

    def _restore_coordinator(self):
        if not self._coordinator_configuration_attempted:
            return self.coordinator_settings
        if self._executor_closed:
            if self._coordinator_settings_applied:
                raise RuntimeError('Native pair coordinator exited before settings restoration')
            return self.coordinator_settings
        # Inspect on the worker after earlier jobs: a setting Future itself may
        # have been lost after enqueue, before its first syscall became visible.
        def restore_if_applied():
            if self._coordinator_settings_applied:
                return self._change_coordinator_settings(0, 0, True)
            return self.coordinator_settings
        try:
            return self._executor.submit(restore_if_applied).result()
        except BaseException as error:
            self.coordinator_settings_history.append({'restore': True,
                'coordinator': self.coordinator_settings, 'status': -1,
                'error': type(error).__name__ + ': ' + str(error)})
            raise

    def _shutdown_coordinator(self):
        """Queue restoration after all phases, then join even a lost Future."""
        if self._executor_closed: return
        failure = None
        try:
            self._restore_coordinator()
        except BaseException as error:
            failure = error
        finally:
            self._executor.shutdown(wait=True, cancel_futures=False)
            self._executor_closed = True
            # Only a confirmed executor join proves a missing phase Future did
            # not leave a coordinator job that can still borrow the sessions.
            entered = self._lost_submission_entered
            if entered is not None:
                if not entered.is_set(): self._busy.release()
                self._idle.set()
                self._lost_submission_entered = None
        # submit can enqueue restoration and raise before returning its Future.
        # Shutdown has joined it; its actual readback is the restoration proof.
        if failure is not None and self._coordinator_settings_applied:
            failure.native_pair_coordinator_settings = self.coordinator_settings
            raise failure

    def submit(self, wires_by_scope, *, deadline_ns_by_scope=None, deadline_ns=None,
               result_transform=None):
        """Return two genuine Futures; no stale phase can overwrite their arrays.

        All frames are copied before the one coordinator task is submitted.
        A second phase cannot queue while an earlier phase owns either bus.
        Future results are published after both native owners have joined the
        generation; each failed bus retains its original records and stats.
        An optional transformer receives immutable stable raw results once,
        before the original Futures publish. It must never send STOP or wait
        for this coordinator. Raw evidence remains separately retained.
        """
        with self._lock:
            if self._closed or not self._handle or self._cancelled:
                raise RuntimeError('Native phase pair closed or cancelled')
            if not self._settings_ready:
                raise RuntimeError('Native phase pair placement is not configured')
            if not self._busy.acquire(blocking=False):
                raise RuntimeError('Native phase pair already has an in-flight generation')
            submission_attempted = False; entered = threading.Event()
            try:
                if result_transform is not None and not callable(result_transform):
                    raise ValueError('Native pair result transformer must be callable')
                if type(wires_by_scope) is not dict or set(wires_by_scope) != {'front', 'rear'}:
                    raise ValueError('A complete front/rear phase is required')
                if deadline_ns_by_scope is None:
                    deadline_ns_by_scope = {'front': deadline_ns, 'rear': deadline_ns}
                if (type(deadline_ns_by_scope) is not dict or
                        set(deadline_ns_by_scope) != {'front', 'rear'}):
                    raise ValueError('Exact absolute deadlines required for both buses')
                stamp = time.monotonic_ns(); buffers = {}
                for scope in ('front', 'rear'):
                    wires = tuple(wires_by_scope[scope]); deadline = deadline_ns_by_scope[scope]
                    if (not 1 <= len(wires) <= 12 or any(type(w) is not bytes or len(w) != 17 for w in wires)
                            or type(deadline) is not int or not stamp < deadline <= stamp+250_000_000):
                        raise ValueError('Invalid paired active request batch/absolute deadline')
                    buffers[scope] = ((C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires)),
                        (Record*len(wires))(), Stats(), C.create_string_buffer(256), deadline)
                futures = {scope: Future() for scope in ('front', 'rear')}
                # These represent a submitted native phase, not independently
                # cancellable motor jobs. pair.cancel cancels both owners.
                for future in futures.values(): future.set_running_or_notify_cancel()
                self._current_futures = dict(futures)
                publication = {'event': threading.Event(), 'error': None}
                self._current_publication = publication
                self._idle.clear(); submission_attempted = True
                self.last_completed_bus_results = None; self.last_phase = None
                self._pending = self._executor.submit(self._exchange, buffers, futures, entered,
                                                      result_transform, publication)
                return futures
            except BaseException as error:
                if submission_attempted:
                    # submit can enqueue a job and then be interrupted before
                    # returning its Future. Shut down/join the exact executor
                    # rather than assuming an absent Future means no owner ran.
                    self._cancelled = True; self.lib.sda_pair_cancel(self._handle)
                    self._lost_submission_entered = entered
                    try: self._shutdown_coordinator()
                    except BaseException as restoration_error:
                        error.native_pair_coordinator_restoration_error = restoration_error
                    # A Future can be lost after one or more real writes. Keep
                    # the exact completed phase on the exception so the runner
                    # can journal it and derive sent-target flags from raw TX.
                    error.native_pair_bus_results = self.last_completed_bus_results
                    error.native_pair_phase = self.last_phase
                else:
                    self._busy.release()
                raise

    def _exchange(self, buffers, futures, entered, result_transform=None, publication=None):
        entered.set()
        held = []; results = {}; failure = None; phase = PairStats()
        try:
            for session in self.sessions.values():
                if not session.busy.acquire(blocking=False):
                    raise RuntimeError('Existing bus owner raced native paired phase')
                held.append(session)
                if not session._handle or session.poisoned:
                    raise RuntimeError('Closed/poisoned session at native phase publication')
            front, rear = buffers['front'], buffers['rear']
            status = self.lib.sda_pair_exchange(self._handle,
                front[0], len(front[1]), front[4], rear[0], len(rear[1]), rear[4],
                front[1], C.byref(front[2]), front[3], rear[1], C.byref(rear[2]), rear[3],
                256, C.byref(phase))
            self.last_phase = {key: int(getattr(phase, key)) for key in
                ('generation', 'submitted_ns', 'validated_ns', 'released_ns', 'cancel_requested_ns')}
            self.last_phase.update(owner_started_ns=list(phase.owner_started_ns),
                owner_finished_ns=list(phase.owner_finished_ns), owner_status=list(phase.owner_status))
            if phase.generation:
                if phase.generation <= self._generation:
                    raise RuntimeError('Stale native pair generation')
                self._generation = phase.generation
            for index, scope in enumerate(('front', 'rear')):
                raw, records, stats, error, _ = buffers[scope]
                del raw
                message = error.value.decode('utf-8', errors='replace')
                if phase.owner_status[index] or message or (status and not phase.generation):
                    self.sessions[scope].poisoned = True
                    results[scope] = ExchangeError(message or 'Native pair failed before owner result', records, stats)
                else:
                    if any(_stop_reply_has_fault(r) for r in records):
                        self.sessions[scope].poisoned = True
                    results[scope] = (records, stats)
            if status or any(s.poisoned for s in self.sessions.values()): self._cancelled = True
        except BaseException as error:
            failure = error; self._cancelled = True
            self.lib.sda_pair_cancel(self._handle)
            for session in self.sessions.values(): session.poisoned = True
        finally:
            if self.last_phase is None and phase.generation:
                self.last_phase = {key: int(getattr(phase, key)) for key in
                    ('generation', 'submitted_ns', 'validated_ns', 'released_ns', 'cancel_requested_ns')}
                self.last_phase.update(owner_started_ns=list(phase.owner_started_ns),
                    owner_finished_ns=list(phase.owner_finished_ns), owner_status=list(phase.owner_status))
            # An interruption can occur after the C ABI populated output slots
            # but before Python assigned results. The already-owned raw arrays
            # remain evidence; never replace them with a plain exception.
            for scope in futures:
                if scope not in results:
                    results[scope] = ExchangeError(str(failure or 'Native pair lacks bus result'),
                                                  buffers[scope][1], buffers[scope][2])
            completed = MappingProxyType(results)
            self.last_completed_bus_results = completed
            for session in reversed(held): session.busy.release()
            self._idle.set()
        publication_failure = None
        try:
            published = completed
            if result_transform is not None:
                try:
                    transformed = result_transform(completed)
                    if (not isinstance(transformed, (dict, MappingProxyType)) or
                            set(transformed) != {'front', 'rear'}):
                        raise ValueError('Native pair transformer must return exact front/rear results')
                    # A raw bus error may not become a claimed successful bus.
                    published = {scope: completed[scope] if isinstance(completed[scope], BaseException)
                                 else transformed[scope] for scope in futures}
                except BaseException as error:
                    error.native_pair_bus_results = completed
                    error.native_pair_phase = self.last_phase
                    published = {scope: completed[scope] if isinstance(completed[scope], BaseException)
                                 else error for scope in futures}
            # Publish only after native writers are joined, retaining the
            # generation fence until both results and the final hint exist.
            # Future callbacks can raise BaseException after state publication;
            # finish the peer Future and release ownership even in that case.
            for scope, future in futures.items():
                try:
                    row = published[scope]
                    if isinstance(row, BaseException): future.set_exception(row)
                    else: future.set_result(row)
                except BaseException as error:
                    if publication_failure is None: publication_failure = error
            try: self._signal_published_completion()
            except BaseException as error:
                if publication_failure is None: publication_failure = error
        finally:
            if publication_failure is not None:
                publication_failure.native_pair_bus_results = completed
                publication_failure.native_pair_phase = self.last_phase
            if publication is not None: publication['error'] = publication_failure
            self._busy.release()
            if publication is not None: publication['event'].set()
        if publication_failure is not None: raise publication_failure

    def cancel(self):
        with self._lock:
            self._cancelled = True
            if self._handle: self.lib.sda_pair_cancel(self._handle)

    def wait_idle(self, timeout=0.35):
        """Join the one coordinator and its bounded original native phase."""
        if not self._idle.wait(timeout=timeout):
            raise TimeoutError('Native phase pair did not join within its original bounded phase')

    def configure_owners(self, cpu_ids, *, timer_slack_ns=1000, restore=False):
        """Configure both C++ owners and their persistent Python coordinator.

        Return the original front/rear rows. Coordinator apply/restoration
        readbacks are retained separately in coordinator_settings/history.
        All settings run on their owning threads, never on the caller.
        """
        with self._lock:
            if self._closed or not self._handle: raise RuntimeError('Native phase pair closed')
            if (type(restore) is not bool or type(timer_slack_ns) is not int or
                    (not restore and timer_slack_ns != 1000)):
                raise ValueError('Native owner timer slack must be exactly 1000ns')
            cpu_ids = tuple(cpu_ids)
            if not restore and (not cpu_ids or any(type(cpu) is not int or not 0 <= cpu < 64 for cpu in cpu_ids)
                                or len(set(cpu_ids)) != len(cpu_ids)):
                raise ValueError('Explicit unique native owner CPU IDs 0..63 required')
            self.wait_idle()
            self._settings_ready = False
            rows = (PairOwnerSettings*2)(); error = C.create_string_buffer(256)
            mask = sum(1 << cpu for cpu in cpu_ids) if not restore else 0
            status = -1; failure = None
            try:
                status = self.lib.sda_pair_owner_settings(self._handle, mask, timer_slack_ns,
                    int(restore), rows, error, len(error))
            except BaseException as native_error:
                failure = native_error
            evidence = {scope: {key: int(getattr(rows[i], key)) for key, _ in PairOwnerSettings._fields_}
                        for i, scope in enumerate(('front', 'rear'))}
            self.owner_settings_history.append({'restore': restore, 'owners': evidence,
                'status': status, 'error': error.value.decode('utf-8', errors='replace') if failure is None
                else type(failure).__name__ + ': ' + str(failure)})
            self._settings_applied = self._settings_applied or any(row.configured for row in rows)
            if failure is None and status:
                failure = RuntimeError(error.value.decode('utf-8', errors='replace') or
                                       'Native owner settings failed')
            # Unsupported native configuration fails before any Python OS
            # setting access. Restoration still tries an already-changed worker
            # even if one of the native owners cannot restore its own setting.
            if restore or failure is None:
                try:
                    if restore:
                        self._restore_coordinator()
                    else:
                        self._coordinator_configuration_attempted = True
                        try:
                            self._executor.submit(self._change_coordinator_settings,
                                                  mask, timer_slack_ns, False).result()
                        except BaseException as coordinator_error:
                            self.coordinator_settings_history.append({'restore': False,
                                'coordinator': self.coordinator_settings, 'status': -1,
                                'error': type(coordinator_error).__name__ + ': ' + str(coordinator_error)})
                            raise
                except BaseException as coordinator_error:
                    if failure is None: failure = coordinator_error
                    else: failure.native_pair_coordinator_restoration_error = coordinator_error
            if failure is not None:
                self._cancelled = True; self.lib.sda_pair_cancel(self._handle)
                failure.native_pair_coordinator_settings = self.coordinator_settings
                raise failure
            self._settings_ready = not restore
            return evidence

    def restore_owners(self):
        return self.configure_owners((), restore=True)

    def close(self):
        """Cancel/join, restore owner placement, destroy before session close."""
        with self._lock:
            if self._closed: return
            self.cancel()
            restoration_error = None
            if self._lost_submission_entered is not None:
                try: self._shutdown_coordinator()
                except BaseException as error:
                    # A failed restoration after a confirmed join can still
                    # destroy safely. An interrupted join retains the borrow.
                    if not self._executor_closed: raise
                    restoration_error = error
            self.wait_idle()
            if self._settings_applied:
                try: self.restore_owners()
                except BaseException as error: restoration_error = error
            try: self._shutdown_coordinator()
            except BaseException as error:
                if restoration_error is None: restoration_error = error
                else: restoration_error.native_pair_coordinator_restoration_error = error
            if not self._executor_closed:
                # Keep the handle and session ownership for a later bounded
                # cleanup attempt; never publish a closed/FD-release proof.
                raise restoration_error or RuntimeError('Native pair coordinator join unconfirmed')
            # Cancellation woke any GIL-releasing hint waiter. Keep its native
            # handle and both pipe descriptors until that waiter has returned.
            with self._notification_wait_lock:
                self.lib.sda_pair_destroy(self._handle); self._handle = None
                self._close_notification_pipe()
            for session in self.sessions.values(): session._phase_pair = None
            self._closed = True
            if restoration_error is not None: raise restoration_error

    def __enter__(self): return self

    def __exit__(self, *_): self.close()


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
