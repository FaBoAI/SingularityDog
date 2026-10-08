"""Explicit, authenticated, read-only C++ CurrentGuard; no CAN capability."""
import copy
import ctypes as C
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import weakref

from .foreground import CurrentGuard as _OriginalCurrentGuard
from . import topology

BUILD_SCHEMA = 'singularitydog.four-bus-current-guard-build.v1'
_TOKEN = object()
_LIBRARIES = weakref.WeakKeyDictionary()
_GUARDS = weakref.WeakKeyDictionary()


class Device(C.Structure):
    _fields_ = [(k, C.c_uint64) for k in ('dev', 'ino', 'mode', 'rdev')]


class Alias(C.Structure):
    _fields_ = [(k, C.c_uint64) for k in ('dev', 'ino', 'mode')] + [
        (k, C.c_int64) for k in ('size', 'mtime_ns', 'ctime_ns')]


class Parent(C.Structure):
    _fields_ = [('path', C.c_char*1024)] + [(k, C.c_uint64) for k in ('dev', 'ino', 'mode')]


class Port(C.Structure):
    _fields_ = [(k, C.c_char*1024) for k in ('path', 'resolved', 'link')] + [
        ('alias', Alias), ('target', Device)]


class Pins(C.Structure):
    _fields_ = [('abi', C.c_uint32), ('parent_count', C.c_uint32),
        ('boot_fd', C.c_int32), ('boot_size', C.c_uint32), ('boot_identity', Device),
        ('boot_bytes', C.c_ubyte*128), ('parents', Parent*32), ('ports', Port*4)]


class Timing(C.Structure):
    _fields_ = [(k, C.c_uint64) for k in ('started_ns', 'boot_checked_ns',
        'ancestors_checked_ns', 'ports_checked_ns', 'finished_ns')] + [
        ('phase', C.c_uint32), ('index', C.c_uint32), ('system_errno', C.c_int32),
        ('reserved', C.c_uint32)]


_ARGUMENTS = (C.POINTER(Pins), C.POINTER(Timing), C.POINTER(C.c_char), C.c_uint32)


def _digest(raw):
    return hashlib.sha256(raw).hexdigest()


def _need(test, message):
    if not test:
        raise ValueError(message)


def _function(function):
    return (function, tuple(function.argtypes or ()), function.restype,
        getattr(function, 'errcheck', None), function._flags_)


@dataclass(frozen=True)
class _LibraryProof:
    token: object
    library: object
    loaded_name: str
    functions: tuple
    source_sha256: str
    binary_sha256: str
    build_sha256: str


def load_library(path, *, expected_sha256, source_sha256, build_record_sha256):
    """Authenticate own source/build/binary, then load an immutable byte copy.

    The temporary pathname is intentionally removed after dlopen. Actual
    CDLL/function identities and the authenticated bytes remain sealed;
    attributes on an unregistered foreign CDLL cannot manufacture this proof.
    """
    path = Path(path).resolve(strict=True)
    record_path, source_path = path.parent/'build-record.json', path.parent/'native_current_guard.cpp'
    raw, record_raw, source_raw = path.read_bytes(), record_path.read_bytes(), source_path.read_bytes()
    _need(_digest(raw) == expected_sha256 and _digest(record_raw) == build_record_sha256 and
          _digest(source_raw) == source_sha256, 'Current guard supplied source/build/binary SHA differs')
    record = json.loads(record_raw)
    _need(type(record) is dict and record.get('schema') == BUILD_SCHEMA and record.get('abi') == 1 and
          record.get('source_sha256') == source_sha256 and record.get('binary_sha256') == expected_sha256 and
          record.get('source_bytes') == len(source_raw) and record.get('CAN_IO_available') is False and
          record.get('output_allowed') is False and record.get('timing_admission_eligible') is False,
          'Current guard adjacent build/source contract differs')
    with tempfile.TemporaryDirectory() as value:
        snapshot = Path(value).resolve()/'current-guard.so'
        snapshot.write_bytes(raw)
        lib = C.CDLL(str(snapshot))
    abi, check = getattr(lib, 'sdcg_abi', None), getattr(lib, 'sdcg_check', None)
    _need(type(abi) is lib._FuncPtr and type(check) is lib._FuncPtr,
          'Complete genuine native current-guard ABI required')
    abi.argtypes, abi.restype = [], C.c_uint32
    check.argtypes, check.restype = list(_ARGUMENTS), C.c_int
    _need(C.sizeof(Pins) == 46320 and C.sizeof(Timing) == 56 and abi() == 1,
          'Native current-guard ABI/layout differs')
    _need(path.read_bytes() == raw and record_path.read_bytes() == record_raw and
          source_path.read_bytes() == source_raw, 'Current guard load inputs changed')
    proof = _LibraryProof(_TOKEN, weakref.ref(lib), str(lib._name), (_function(abi), _function(check)),
        source_sha256, expected_sha256, build_record_sha256)
    lib._current_guard_proof = proof
    _LIBRARIES[lib] = weakref.ref(proof)
    _verify_library(lib)
    return lib


def _verify_library(lib):
    proof = getattr(lib, '_current_guard_proof', None)
    registered = _LIBRARIES.get(lib) if type(lib) is C.CDLL else None
    _need(registered is not None and registered() is proof and type(proof) is _LibraryProof and
          proof.token is _TOKEN and proof.library() is lib and str(lib._name) == proof.loaded_name,
          'Genuine authenticated current-guard library required')
    for name, sealed in zip(('sdcg_abi', 'sdcg_check'), proof.functions):
        current = getattr(lib, name, None)
        _need(current is sealed[0] and _function(current) == sealed,
              'Current guard native function identity/signature changed')
    return proof


def _text(text):
    _need(type(text) is str and '\x00' not in text, 'Bounded original guard path/link required')
    raw = os.fsencode(text)
    _need(0 < len(raw) < 1024, 'Original guard path/link exceeds bounded native ABI')
    return raw


class NativeCurrentGuard:
    """Same setup/end resolution and same dynamic checks in one native call.

    Const pins are owned for this object's whole lifetime. Each concurrent
    caller has private native result/error buffers. close never closes caller
    descriptors and rejects use until all in-flight native checks have joined.
    """
    def __init__(self, library, bindings, descriptors, boot_fd, boot_id, stop, *, clock=time.monotonic_ns):
        proof = _verify_library(library)
        original = _OriginalCurrentGuard(bindings, descriptors, boot_fd, boot_id, stop, clock=clock)
        _need(type(boot_fd) is int and 0 <= boot_fd <= 2**31-1 and
              0 < len(original.boot_bytes) < 128 and 0 < len(original.ancestors) <= 32,
              'Current guard setup exceeds bounded native ABI')
        pins = Pins(abi=1, parent_count=len(original.ancestors), boot_fd=boot_fd,
                    boot_size=len(original.boot_bytes))
        pins.boot_identity = Device(*original.boot_fd_identity)
        for i, byte in enumerate(original.boot_bytes): pins.boot_bytes[i] = byte
        for i, (path, identity) in enumerate(original.ancestors.items()):
            pins.parents[i] = Parent(_text(path), *identity)
        for i, port in enumerate(topology.PORTS):
            row = original.bindings[port]
            alias, target, link = original.identities[port]
            pins.ports[i] = Port(_text(row['path']), _text(row['resolved']), _text(link),
                                 Alias(*alias), Device(*target))
        self._library, self._original, self._pins = library, original, pins
        self._stop, self._clock = stop, clock
        self._owner = threading.get_ident()
        self._lock, self._active, self._closed = threading.Lock(), 0, False
        self.calls = []
        self.setup = copy.deepcopy(original.setup)
        self.setup.update(native_current_guard_abi=1, source_sha256=proof.source_sha256,
            binary_sha256=proof.binary_sha256, build_record_sha256=proof.build_sha256,
            dynamic_guard_checks_unchanged=True, native_CAN_IO_available=False)
        _GUARDS[self] = (weakref.ref(library), proof, original, pins, bytes(pins), stop, clock,
            copy.deepcopy(original.bindings), copy.deepcopy(original.identities), copy.deepcopy(original.ancestors),
            (original.boot_fd, original.boot_bytes, original.boot_fd_identity))

    def _verify(self):
        seal = _GUARDS.get(self)
        _need(seal is not None and not self._closed and seal[0]() is self._library and
              _verify_library(self._library) is seal[1] and self._original is seal[2] and
              self._pins is seal[3] and bytes(self._pins) == seal[4] and
              self._stop is seal[5] and self._clock is seal[6] and
              self._original.bindings == seal[7] and self._original.identities == seal[8] and
              self._original.ancestors == seal[9] and
              (self._original.boot_fd, self._original.boot_bytes, self._original.boot_fd_identity) == seal[10],
              'Original immutable current-guard ownership/pins changed')

    def __call__(self):
        self._check(check_cancel=True)

    def _check(self, *, check_cancel):
        record = {'started_ns': None, 'thread_native_id': threading.get_native_id(),
            'ok': False, 'cancellation_check_applied': check_cancel, 'native_guard': True}
        entered = False
        primary = None
        try:
            record['started_ns'] = self._clock()
            with self._lock:
                self._verify()
                self._active += 1
                entered = True
            if check_cancel: _need(not self._stop.is_set(), 'Foreground cancelled')
            timing, error = Timing(), C.create_string_buffer(512)
            # Per-call snapshot is private: mutation of the exposed original
            # ctypes pins can never redirect the in-flight native boot read.
            call_pins = Pins.from_buffer_copy(_GUARDS[self][4])
            before = time.monotonic_ns()
            # This is the only C call per hot check. CDLL releases the GIL.
            status = self._library.sdcg_check(C.byref(call_pins), C.byref(timing), error, len(error))
            after = time.monotonic_ns()
            post_cancel, post_cancel_error = False, None
            if check_cancel:
                try:
                    post_cancel = self._stop.is_set()
                    record['cancellation_after_native_checked'] = True
                    record['cancelled_after_native'] = post_cancel
                except BaseException as cancellation_error:
                    post_cancel_error = cancellation_error
            record['native'] = {key: int(getattr(timing, key)) for key, _ in Timing._fields_}
            record['native_call_before_ns'], record['native_call_after_ns'] = before, after
            _need(status in (-1, 0) and before <= timing.started_ns <= timing.finished_ns <= after and
                  timing.reserved == 0, 'Native guard status/actual clock contract differs')
            if status:
                failure = ValueError(error.value.decode('utf-8', errors='replace') or 'Native current guard failed')
                if post_cancel or post_cancel_error is not None:
                    failure.add_note('Cancellation observed/failed after native guard failure')
                raise failure
            _need(not error.value and timing.phase == 4 and timing.system_errno == 0 and
                  timing.started_ns <= timing.boot_checked_ns <= timing.ancestors_checked_ns <=
                  timing.ports_checked_ns <= timing.finished_ns, 'Native guard success phases differ')
            if post_cancel_error is not None: raise post_cancel_error
            if check_cancel: _need(not post_cancel, 'Foreground cancelled')
            with self._lock: self._verify()
            record['boot_checked_ns'] = int(timing.boot_checked_ns)
            record['ancestors_checked_ns'] = int(timing.ancestors_checked_ns)
            record['ports_checked_ns'] = int(timing.ports_checked_ns)
            record['ok'] = True
        except BaseException as error:
            primary = error
            record['error_type'], record['error'] = type(error).__name__, str(error)
            raise
        finally:
            clock_error = None
            try:
                record['finished_ns'] = self._clock()
            except BaseException as failure:
                clock_error = failure
                record.update(finished_ns=None, ok=False, clock_error=str(failure))
                if primary is not None: primary.add_note('Current guard final clock: '+str(failure))
                else: record.update(error_type=type(failure).__name__, error=str(failure))
            finally:
                with self._lock:
                    if entered: self._active -= 1
                    self.calls.append(record)
            if clock_error is not None and primary is None: raise clock_error

    def finish(self):
        _need(threading.get_ident() == self._owner, 'Original setup owner must finish current guard')
        with self._lock:
            self._verify()
            _need(self._active == 0, 'Join all original current-guard checks before finish')
        # Retain the exact reference end/full path-resolution implementation.
        # Its extra cleanup check is outside all successful timed cycles.
        original_end = self._original.finish()
        self._check(check_cancel=False)
        return {'setup': copy.deepcopy(self.setup), 'end_full_path_resolution_verified': True,
            'reference_end_checks': original_end['calls'], 'calls': list(self.calls),
            'timing_scope': 'original hot-check wall including Python seals, cancellation and GIL handoff; native subphases overlap across callers',
            'native_CAN_IO_available': False, 'output_allowed': False}

    def close(self):
        _need(threading.get_ident() == self._owner, 'Original setup owner must close current guard')
        with self._lock:
            if self._closed: return
            self._verify()
            _need(self._active == 0, 'Join original current-guard calls before close')
            self._closed = True
            _GUARDS.pop(self, None)
