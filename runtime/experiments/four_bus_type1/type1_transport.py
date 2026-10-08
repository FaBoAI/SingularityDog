"""Four-bus boxed Type1 transport: one port, one masked three-axis half session.

This module opens no device and grants no output permission. Caller-owned
descriptors, boot/cancel bindings, the admitted profile and the direct-human
condition record remain required. Every exchange goes through the native
mask-bound sda_subset_exchange and an exact per-method Python whitelist; the
ordinary sda_exchange and fixed-six emergency STOP are never called here.
"""
from concurrent.futures import Future
from contextlib import contextmanager
import ctypes as C
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import threading
import time
import weakref

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.motor_version_probe import version_request
from singularitydog_hw.native_diagnostic_transport import Record, exchange_evidence
from singularitydog_hw.policy_output_runtime import decode_records
from experiments.four_bus_diagnostic.transport_adapter import Batch, Group
from . import build

GAP_NS = 900_000
WINDOW = 3
MAX_DEADLINE_NS = 250_000_000
LSB_RAD = 25.14/65535.
KP_CAP = 3.
KD_CAP = .15
MAX_RAW_WINDOW_RAD = 2*math.radians(3)+1e-12
UNREACHABLE_CODE = 32767
PARAMETERS = ('run_mode', 'position', 'velocity', 'voltage', 'can_timeout')
REPLY_MODES = {'stop': (0,), 'watchdog_setup': (0,), 'enable': (0, 2), 'zero_gain': (2,),
               'feedback_hold': (2,), 'output': (2,)}
STOP_SCHEMA = 'singularitydog.four-bus-type1-subset-stop-result.v1'
_SYMBOLS = ('sda_subset_active_abi', 'sda_emergency_stop_subset_abi', 'sda_subset_validate',
            'sda_subset_exchange', 'sda_emergency_stop_subset')
_ARGS = ((), (),
         (C.c_void_p, C.c_uint32, C.POINTER(C.c_ubyte), C.c_uint32, C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.c_uint32, C.POINTER(C.c_ubyte), C.c_uint32, C.c_int, C.c_uint64,
          C.POINTER(Record), C.POINTER(active.Stats), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.c_uint32, C.c_uint64, C.POINTER(Record), C.POINTER(active.Stats),
          C.POINTER(active.StopResult), C.POINTER(C.c_char), C.c_uint32))
_RESULTS = (C.c_uint32, C.c_uint32, C.c_int, C.c_int, C.c_int)
_LIBRARIES = weakref.WeakKeyDictionary()
_TRANSPORTS = weakref.WeakKeyDictionary()
_TOKEN = object()


def need(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def decoded_q(code):
    # Same operation order as native valid_request.
    return code*25.14/65535.-12.57


def unreachable_window(code=UNREACHABLE_CODE):
    """Raw window strictly between two adjacent u16 codes: no Type1 decodes inside."""
    need(type(code) is int and 0 <= code < 65535, 'Explicit interior u16 code required')
    base = decoded_q(code)
    lower, upper = base+LSB_RAD/3, base+2*LSB_RAD/3
    need(decoded_q(code) < lower < upper < decoded_q(code+1), 'Unreachable window is not between codes')
    return lower, upper


def type1_fields(wire):
    """(mid, q code, kp code, kd code) only for a canonical zero-FF/vref Type1."""
    if type(wire) is not bytes or len(wire) != 17:
        return None
    mid = (int.from_bytes(wire[2:6], 'big') >> 3) & 255
    q, _, kp, kd = struct.unpack('>4H', wire[7:15])
    canonical = (b'AT'+((((1 << 24)|(32767 << 8)|mid) << 3)|4).to_bytes(4, 'big')+b'\x08'+
                 struct.pack('>4H', q, 32767, kp, kd)+b'\r\n')
    return (mid, q, kp, kd) if wire == canonical else None


@dataclass(frozen=True)
class _LibrarySeal:
    token: object
    library: object
    active_binding: object
    functions: tuple
    build_record_sha256: str
    pins: tuple


def _function_seal(function):
    return (function, tuple(function.argtypes or ()), function.restype,
            getattr(function, 'errcheck', None), function._flags_)


def load_library(path, *, expected_sha256, ordinary_source_sha256, subset_stop_source_sha256,
                 extension_source_sha256, build_record_sha256):
    """Explicit setup authentication of the subset-active receipt and all three sources."""
    path = Path(path).resolve(strict=True)
    directory = path.parent
    record_path = directory/'build-record.json'
    pins = (ordinary_source_sha256, subset_stop_source_sha256, extension_source_sha256)
    copies = ('ordinary_transport.cpp', 'subset_stop.cpp', 'transport.cpp')

    def check_files():
        need(path.name == build.LIBRARY_NAME, 'Four-bus Type1 library name required')
        need(sha(record_path) == build_record_sha256, 'Four-bus Type1 build receipt differs')
        problem = build.receipt_problem(directory)
        need(problem is None, problem or '')
        scope = json.loads(record_path.read_bytes())['four_bus_subset_active']
        need((scope['ordinary_source_sha256'], scope['subset_stop_source_sha256'],
              scope['extension_source_sha256']) == pins and
             tuple(sha(directory/name) for name in copies) == pins,
             'Explicit ordinary, subset STOP and subset-active source pins required')
    check_files()
    lib = active.load_library(path, expected_sha256=expected_sha256)
    try:
        # Resolve from the authenticated dlopen handle, not cached attributes.
        functions = tuple(lib[name] for name in _SYMBOLS)
    except AttributeError:
        raise ValueError('Explicit subset-active ABI required; ordinary/fixed-six fallback prohibited')
    for name, function, arguments, result in zip(_SYMBOLS, functions, _ARGS, _RESULTS):
        function.argtypes = list(arguments); function.restype = result
        setattr(lib, name, function)
    if lib.sda_subset_active_abi() != 1 or lib.sda_emergency_stop_subset_abi() != 1:
        raise ValueError('Subset-active/subset-STOP ABI mismatch')
    binding = active.verified_active_source_binding(lib)
    check_files()
    if (binding.build_record_sha256 != build_record_sha256 or
            binding.source_sha256 != extension_source_sha256 or
            binding.binary_sha256 != expected_sha256 or binding.path != str(path) or
            binding.source_path != str(directory/'transport.cpp') or
            binding.build_record_path != str(record_path)):
        raise ValueError('Subset-active source/receipt changed across authenticated load')
    seal = _LibrarySeal(_TOKEN, weakref.ref(lib), binding,
                        tuple(_function_seal(function) for function in functions),
                        build_record_sha256, pins)
    lib._four_bus_type1_seal = seal
    _LIBRARIES[lib] = weakref.ref(seal)
    verify_library(lib)
    return lib


def verify_library(lib):
    seal = getattr(lib, '_four_bus_type1_seal', None)
    registered = _LIBRARIES.get(lib) if type(lib) is C.CDLL else None
    if (registered is None or registered() is not seal or type(seal) is not _LibrarySeal or
            seal.token is not _TOKEN or seal.library() is not lib or
            active.verified_active_source_binding(lib) is not seal.active_binding):
        raise ValueError('Genuine authenticated four-bus Type1 library required')
    for name, original in zip(_SYMBOLS, seal.functions):
        current = getattr(lib, name, None)
        if current is not original[0] or _function_seal(current) != original:
            raise ValueError('Four-bus Type1 function binding changed')
    return seal


def session_limits(group, axis_raw_bounds, kp_cap_by_id, kd_cap_by_id):
    """Member slots: explicit raw window and caps. Non-members: kp=kd=0, unreachable q."""
    need(type(group) is Group, 'Immutable group required')
    for name, values in (('axis_raw_bounds', axis_raw_bounds), ('kp_cap_by_id', kp_cap_by_id),
                         ('kd_cap_by_id', kd_cap_by_id)):
        need(type(values) is dict and set(values) == set(group.ids) and
             all(type(mid) is int for mid in values), f'{name} must hold exactly this group\'s three IDs')
    finite = lambda value: (type(value) in (int, float) and math.isfinite(value))
    lower, upper, kp, kd = {}, {}, {}, {}
    for mid in range(group.first_id, group.first_id+6):
        if mid in group.ids:
            bounds = axis_raw_bounds[mid]
            need(type(bounds) is tuple and len(bounds) == 2 and all(finite(x) for x in bounds),
                 f'ID{mid} raw bounds must be a finite (lower, upper) tuple')
            lo, hi = float(bounds[0]), float(bounds[1])
            need(-12.57 <= lo < hi <= 12.57 and hi-lo <= MAX_RAW_WINDOW_RAD,
                 f'ID{mid} raw window must be ordered, in range and at most six degrees wide')
            need(finite(kp_cap_by_id[mid]) and 0 <= kp_cap_by_id[mid] <= KP_CAP,
                 f'ID{mid} kp cap must be 0..{KP_CAP}')
            need(finite(kd_cap_by_id[mid]) and 0 <= kd_cap_by_id[mid] <= KD_CAP,
                 f'ID{mid} kd cap must be 0..{KD_CAP}')
            lower[mid], upper[mid] = lo, hi
            kp[mid], kd[mid] = float(kp_cap_by_id[mid]), float(kd_cap_by_id[mid])
        else:
            (lower[mid], upper[mid]), kp[mid], kd[mid] = unreachable_window(), 0., 0.
    return lower, upper, kp, kd


class Type1Transport:
    """One persistent owner thread per physical port; no FD ownership transfer."""
    def __init__(self, session, group, *, cancel_all, token=None):
        if token is not _TOKEN or type(session) is not active.ActiveSession or type(group) is not Group:
            raise ValueError('Use the four-bus Type1 creation factory with a genuine session')
        if not callable(cancel_all):
            raise ValueError('Explicit shared cancel_all capability required')
        self._session = session
        self.group = group
        self._library = session.lib
        self._seal = verify_library(session.lib)
        self._creation = active.verified_active_session_creation(session)
        self._binding = (session._handle, session.fd, session.first_id, bytes(session._limits))
        limits = session._limits
        self._bounds = {mid: (limits.lower[mid-session.first_id], limits.upper[mid-session.first_id])
                        for mid in group.ids}
        self._caps = {mid: (limits.kp[mid-session.first_id], limits.kd[mid-session.first_id])
                      for mid in group.ids}
        self._cancel_all = cancel_all
        self._owner = None
        self._closed = False
        self._poisoned = False
        self._aborted = None
        self._last_output = None
        self._enabled, self._zero_gain = set(), set()
        self._stop_ambiguous = set()
        self.journal = []
        self._batches = []
        self._batch_index = {}  # identity index of _batches: O(1) genuineness lookup
        self.failures = []
        self.attempts = {'motor_enable_sent': False, 'type1_sent': False, 'positive_gain_sent': False}
        ids = group.ids
        self._wires = {
            'identify': tuple(read_request(mid) for mid in ids),
            'stop': tuple(protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=mid) for mid in ids),
            'version': tuple(version_request(mid) for mid in ids),
            'watchdog_setup': tuple(protocol.watchdog_setup_request(
                phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid) for mid in ids),
            'enable': tuple(protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=mid) for mid in ids),
            'voltage': tuple(read_request(mid, 'voltage') for mid in ids)}
        self._params = {}
        for size in (1, 2):
            for names in itertools.permutations(PARAMETERS, size):
                self._params[tuple(read_request(mid, name) for name in names for mid in ids)] = names
        _TRANSPORTS[self] = (weakref.ref(session), group, self._binding, dict(self._wires),
                             dict(self._params), cancel_all)

    @classmethod
    def create(cls, library, fd, *, group, cancel_fd, boot_fd, boot_id, axis_raw_bounds,
               kp_cap_by_id, kd_cap_by_id, cancel_all):
        verify_library(library)
        if not callable(cancel_all):
            raise ValueError('Explicit shared cancel_all capability required')
        lower, upper, kp, kd = session_limits(group, axis_raw_bounds, kp_cap_by_id, kd_cap_by_id)
        session = active.ActiveSession(library, fd, first_id=group.first_id,
            cancel_fd=cancel_fd, boot_fd=boot_fd, boot_id=boot_id,
            raw_lower_by_id=lower, raw_upper_by_id=upper, kp_max_by_id=kp, kd_max_by_id=kd,
            gap_ns=GAP_NS, window=WINDOW)
        try:
            return cls(session, group, cancel_all=cancel_all, token=_TOKEN)
        except BaseException:
            session.close()
            raise

    @property
    def poisoned(self):
        return self._poisoned or self._session.poisoned

    @property
    def last_batch(self):
        return self._batches[-1] if self._batches else None

    def _verify(self):
        session = self._session
        original = _TRANSPORTS.get(self)
        if (original is None or original[0]() is not session or self.group is not original[1] or
                self._binding != original[2] or self._wires != original[3] or
                self._params != original[4] or self._cancel_all is not original[5] or
                self._closed or not session._handle or session.lib is not self._library or
                verify_library(self._library) is not self._seal or
                active.verified_active_session_creation(session) != self._creation or
                (session._handle, session.fd, session.first_id, bytes(session._limits)) != self._binding or
                session._phase_pair is not None):
            raise ValueError('Original four-bus Type1 owner/session/source binding changed')

    def _owner_check(self):
        current = threading.get_native_id()
        if self._owner is None:
            self._owner = current
        if current != self._owner:
            raise RuntimeError('Original physical port worker must retain ownership')
        self._verify()

    def _abort(self, error):
        if error is self._aborted:
            return  # Nested scopes: one poison/cancel per failure.
        self._aborted = error
        self._poisoned = True
        self._session.poisoned = True
        self.failures.append(type(error).__name__+': '+str(error))
        try:
            self._cancel_all()
        except BaseException as cancel_error:
            self.failures.append('cancel_all failed: '+type(cancel_error).__name__+': '+str(cancel_error))
            error.add_note('Shared cancel_all failed: '+str(cancel_error))

    @contextmanager
    def _abort_on_failure(self):
        try:
            yield
        except BaseException as error:
            self._abort(error)
            raise

    def _type1_problem(self, wire, mid=None, *, zero=False):
        fields = type1_fields(wire)
        if fields is None:
            return 'Noncanonical Type1 frame'
        target, q, kp, kd = fields
        if target not in self.group.ids or (mid is not None and target != mid):
            return 'Type1 destination outside this exact group/slot'
        lo, hi = self._bounds[target]
        kp_cap, kd_cap = self._caps[target]
        if not lo <= decoded_q(q) <= hi:
            return f'ID{target} quantized Type1 target outside its raw window'
        if zero and (kp or kd):
            return f'ID{target} zero-gain Type1 must encode kp=kd=0'
        if kp*500./65535. > kp_cap or kd*5./65535. > kd_cap:
            return f'ID{target} Type1 gain above its cap'
        return None

    def check_output_wires(self, wires):
        """Pure check (no I/O, no owner): exactly three Type1 in ascending group order."""
        wires = tuple(wires)
        need(len(wires) == 3, 'Output requires exactly three Type1 wires')
        for mid, wire in zip(self.group.ids, wires):
            problem = self._type1_problem(wire, mid)
            need(problem is None, problem or '')
        return wires

    def validate_wires(self, label, wires):
        """Exact per-method whitelist. Returns (wires, exact decoded row keys)."""
        wires = tuple(wires)
        need(1 <= len(wires) <= 6 and all(type(w) is bytes and len(w) == 17 for w in wires),
             'Four-bus Type1 batch must hold 1..6 canonical 17-byte wires')
        ids = self.group.ids
        destination = (int.from_bytes(wires[0][2:6], 'big') >> 3) & 255
        feedback = lambda mids: {(mid, 'feedback') for mid in mids}
        if label in ('identify', 'stop', 'version', 'watchdog_setup'):
            need(wires == self._wires[label], f'Only the exact three {label} wires are allowed')
            keys = ({(mid, 'identity') for mid in ids} if label == 'identify' else
                    {(mid, 'version') for mid in ids} if label == 'version' else feedback(ids))
        elif label == 'params':
            names = self._params.get(wires)
            need(names is not None, 'Only one or two allowlisted Type17 reads for all three IDs')
            keys = {(mid, name) for name in names for mid in ids}
        elif label == 'enable':
            need(len(wires) == 1 and wires[0] in self._wires['enable'], 'Only one same-group Type3 enable')
            keys = feedback((destination,))
        elif label == 'zero_gain':
            need(len(wires) == 1, 'Zero-gain handshake is exactly one Type1')
            problem = self._type1_problem(wires[0], zero=True)
            need(problem is None, problem or '')
            keys = feedback((destination,))
        elif label == 'output':
            self.check_output_wires(wires)
            keys = feedback(ids)
        elif label == 'feedback_hold':
            need(self._last_output is not None and wires == self._last_output,
                 'Hold must re-send this port\'s last validated output batch byte for byte')
            keys = feedback(ids)
        elif label == 'voltage':
            need(len(wires) == 1 and wires[0] in self._wires['voltage'], 'Only one same-group Type17 voltage')
            keys = {(destination, 'voltage')}
        else:
            raise ValueError('Unknown four-bus Type1 exchange label')
        return wires, frozenset(keys)

    def _mark(self, wires):
        # Truthful attempt flags: set before the native call, never reset.
        for wire in wires:
            kind = int.from_bytes(wire[2:6], 'big') >> 27
            if kind == 3:
                self.attempts['motor_enable_sent'] = True
            elif kind == 1:
                self.attempts['type1_sent'] = True
                fields = type1_fields(wire)
                if fields is None or fields[2] or fields[3]:
                    self.attempts['positive_gain_sent'] = True

    def _native(self, wires, deadline_ns, before_native):
        session = self._session
        if not session.busy.acquire(blocking=False):
            raise RuntimeError('Concurrent active session use')
        try:
            if not session._handle:
                raise RuntimeError('Active session closed')
            if self.poisoned:
                raise RuntimeError('Session poisoned; active retry prohibited')
            now = time.monotonic_ns()
            if type(deadline_ns) is not int or not now < deadline_ns <= now+MAX_DEADLINE_NS:
                raise ValueError('Expired or invalid absolute active deadline')
            raw = (C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires))
            records, stats, error = (Record*len(wires))(), active.Stats(), C.create_string_buffer(256)
            if before_native is not None:
                if time.monotonic_ns() >= deadline_ns:
                    raise TimeoutError('Active deadline expired before prepared publication')
                narrowed = before_native()
                if narrowed is not None:
                    if type(narrowed) is not int or not 0 < narrowed <= deadline_ns:
                        raise ValueError('Prepared hook must only tighten the absolute deadline')
                    deadline_ns = narrowed
            if time.monotonic_ns() >= deadline_ns:
                raise TimeoutError('Active deadline expired before subset exchange')
            self._mark(wires)
            status = self._library.sda_subset_exchange(session._handle, self.group.mask, raw, len(wires),
                0, deadline_ns, records, C.byref(stats), error, len(error))
            if status:
                raise active.ExchangeError(error.value.decode('utf-8', errors='replace'), records, stats)
            return records, stats
        except BaseException:
            session.poisoned = True
            raise
        finally:
            session.busy.release()

    def _exchange(self, label, wires, deadline_ns, *, before_native=None):
        with self._abort_on_failure():
            return self._checked_exchange(label, wires, deadline_ns, before_native)

    def _checked_exchange(self, label, wires, deadline_ns, before_native):
        self._owner_check()
        wires, keys = self.validate_wires(label, wires)
        try:
            raw = self._native(wires, deadline_ns, before_native)
        except active.ExchangeError as error:
            self.journal.append((label, (error.records, error.stats)))
            raise
        self.journal.append((label, raw))  # Keep raw even if decoding rejects.
        rows = decode_records(raw)
        need(set(rows) == keys, f'{label} replies do not cover the exact requested keys')
        modes = REPLY_MODES.get(label)
        for key, (value, _, _) in rows.items():
            if key[1] == 'feedback':
                need(modes is not None and value.mode_state in modes and value.fault_bits == 0,
                     f'ID{key[0]} {label} requires mode {"/".join(map(str, modes or ()))} and fault zero')
        batch = Batch(self.group, label, raw[0], raw[1], rows, bytes(raw[0]), bytes(raw[1]),
                      time.monotonic_ns())
        self._batches.append(batch)
        self._batch_index[id(batch)] = batch
        return batch

    def verify_batch(self, batch, label):
        if (type(batch) is not Batch or batch.group is not self.group or batch.label != label or
                self._batch_index.get(id(batch)) is not batch):
            raise ValueError('Genuine current owner batch required')
        return batch.verify()

    # Preflight / handshake.
    def identify(self, *, deadline_ns):
        with self._abort_on_failure():
            rows = self._exchange('identify', self._wires['identify'], deadline_ns).rows
            return {mid: bytes.fromhex(rows[mid, 'identity'][0]['mcu_uid_hex']) for mid in self.group.ids}

    def stop(self, *, deadline_ns):
        with self._abort_on_failure():
            rows = self._exchange('stop', self._wires['stop'], deadline_ns).rows
            return {mid: rows[mid, 'feedback'][0] for mid in self.group.ids}

    def version_probe(self, *, deadline_ns):
        with self._abort_on_failure():
            rows = self._exchange('version', self._wires['version'], deadline_ns).rows
            return {mid: bytes.fromhex(rows[mid, 'version'][0]['version_bytes_hex']) for mid in self.group.ids}

    def read_params(self, names, *, deadline_ns):
        with self._abort_on_failure():
            names = tuple(names)
            need(1 <= len(names) <= 2 and len(set(names)) == len(names) and
                 all(name in PARAMETERS for name in names), 'One or two distinct allowlisted parameters')
            wires = tuple(read_request(mid, name) for name in names for mid in self.group.ids)
            rows = self._exchange('params', wires, deadline_ns).rows
            return {(mid, name): rows[mid, name][0]['value'] for name in names for mid in self.group.ids}

    def write_watchdog(self, ticks, *, deadline_ns):
        with self._abort_on_failure():
            need(type(ticks) is int and ticks == protocol.WATCHDOG_TICKS,
                 'Only the volatile 4000-tick (200 ms) watchdog is allowed')
            rows = self._exchange('watchdog_setup', self._wires['watchdog_setup'], deadline_ns).rows
            return {mid: rows[mid, 'feedback'][0] for mid in self.group.ids}

    def enable(self, mid, *, deadline_ns):
        with self._abort_on_failure():
            need(type(mid) is int and mid in self.group.ids, 'Same-group enable ID required')
            wire = protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=mid)
            value = self._exchange('enable', (wire,), deadline_ns).rows[mid, 'feedback'][0]
            self._enabled.add(mid)
            return value

    def zero_gain(self, mid, q_raw, *, deadline_ns):
        with self._abort_on_failure():
            need(type(mid) is int and mid in self._enabled, 'Zero-gain Type1 requires this axis enabled here')
            wire = active.encode_motion(mid, q_raw, 0., 0.)
            value = self._exchange('zero_gain', (wire,), deadline_ns).rows[mid, 'feedback'][0]
            self._zero_gain.add(mid)
            return value

    # Cycle.
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check):
        """Re-send the last validated output; publish the hold Batch only with the owner held.

        The prefix Future is completed from before_native of the voltage
        exchange. The caller's outer Future, not the prefix, joins the owner.
        """
        with self._abort_on_failure():
            try:
                self._owner_check()
                need(type(prefix_future) is Future and not prefix_future.done(),
                     'Genuine current unpublished hold Future required')
                need(type(voltage_id) is int and voltage_id in self.group.ids,
                     'Rotating voltage axis must belong to the physical group')
                check()
                hold = self._exchange('feedback_hold', wires, deadline_ns)
                def publish():
                    check()
                    if prefix_future.done():
                        raise ValueError('Hold Future was externally completed')
                    prefix_future.set_result(hold)
                    check()
                    return deadline_ns
                voltage = self._exchange('voltage', (read_request(voltage_id, 'voltage'),), deadline_ns,
                                         before_native=publish)
                return hold, voltage
            except BaseException as error:
                if type(prefix_future) is Future and not prefix_future.done():
                    prefix_future.set_exception(error)
                raise

    def validate_output(self, wires):
        """Optional owner-side no-I/O native pre-validation of one output batch."""
        with self._abort_on_failure():
            self._owner_check()
            wires, _ = self.validate_wires('output', wires)
            session = self._session
            if not session.busy.acquire(blocking=False):
                raise RuntimeError('Concurrent active session use')
            try:
                if not session._handle or self.poisoned:
                    raise RuntimeError('Session poisoned/closed; active retry prohibited')
                raw = (C.c_ubyte*(17*len(wires))).from_buffer_copy(b''.join(wires))
                error = C.create_string_buffer(256)
                if self._library.sda_subset_validate(session._handle, self.group.mask, raw, len(wires),
                                                     error, len(error)):
                    raise ValueError(error.value.decode('utf-8', errors='replace'))
            finally:
                session.busy.release()
            return wires

    def output(self, wires, *, deadline_ns, check):
        with self._abort_on_failure():
            need(self._zero_gain == set(self.group.ids),
                 'Output requires every member enabled with a confirmed zero-gain handshake')
            check()
            wires = self.check_output_wires(wires)
            batch = self._exchange('output', wires, deadline_ns)
            self._last_output = wires
            return batch

    # Termination.
    def _stop_round(self, deadline_ns):
        session = self._session
        if not session.busy.acquire(blocking=False):
            raise RuntimeError('Join original owner before subset STOP')
        try:
            if not session._handle:
                raise RuntimeError('Active session closed')
            records, stats, result = (Record*6)(), active.Stats(), active.StopResult()
            error = C.create_string_buffer(256)
            status = self._library.sda_emergency_stop_subset(session._handle, self.group.mask,
                deadline_ns, records, C.byref(stats), C.byref(result), error, len(error))
            self.journal.append(('stop_subset', (records, stats)))
            first = session.first_id
            ids = lambda mask: [first+i for i in range(6) if mask & self.group.mask & (1 << i)]
            evidence = exchange_evidence(records, stats)
            evidence.update(original_record_slots=6, selected_ids=list(self.group.ids),
                            rejected_total_bytes=int(stats.rejected_total),
                            rejected_truncated=stats.rejected_total > stats.rejected_size)
            return {'native_status': status, 'message': error.value.decode('utf-8', errors='replace'),
                    'deadline_ns': deadline_ns, 'attempted_ids': ids(result.attempted_mask),
                    'confirmed_ids': ids(result.confirmed_mask), 'ambiguous_ids': ids(result.ambiguous_mask),
                    'fault_by_id': {str(first+i): int(result.fault[i]) for i in range(6)
                                    if self.group.mask & (1 << i) and records[i].received == 17},
                    'raw': evidence}
        finally:
            session.busy.release()

    def stop_repeated(self, *, total_budget_ns=1_000_000_000, rounds=3):
        """Subset STOP only, at most three rounds in one absolute budget.

        Never fixed-six STOP, never motion retry. Ambiguity (native and this
        transport's union) is sticky, so a pending Type1/enable/STOP is never
        promoted to confirmation. Complete needs all three confirmed, no
        ambiguity and zero fault bits; otherwise physical cutoff is required.
        """
        self._owner_check()
        if (type(rounds) is not int or not 1 <= rounds <= 3 or type(total_budget_ns) is not int or
                not 21_000_000 <= total_budget_ns <= 1_000_000_000):
            raise ValueError('Subset STOP retries require 1..3 rounds and a 21..1000 ms total budget')
        self._poisoned = True
        self._session.poisoned = True
        ids = set(self.group.ids)
        begin = time.monotonic_ns(); deadline = begin+total_budget_ns
        attempts, faults, confirmed, complete = [], {}, set(), False
        for index in range(rounds):
            now = time.monotonic_ns()
            if deadline-now < 21_000_000:
                break
            round_deadline = min(deadline, now+(250_000_000 if index == 0 else 500_000_000))
            try:
                row = self._stop_round(round_deadline)
            except Exception as error:
                row = {'native_status': None, 'error': type(error).__name__+': '+str(error),
                       'attempted_ids': [], 'confirmed_ids': [], 'ambiguous_ids': [], 'fault_by_id': {}}
            attempts.append(row)
            self._stop_ambiguous.update(row['ambiguous_ids'])
            for mid, bits in row['fault_by_id'].items():
                faults[mid] = faults.get(mid, 0) | bits
            confirmed = set(row['confirmed_ids'])-self._stop_ambiguous
            settled = confirmed == ids and not self._stop_ambiguous & ids
            complete = settled and row['native_status'] == 0 and not any(faults.values())
            if settled:
                break  # A further STOP cannot clear a reported fault.
        end = time.monotonic_ns()
        ambiguous = sorted(self._stop_ambiguous & ids)
        return {'schema': STOP_SCHEMA, 'port': self.group.port, 'selected_ids': list(self.group.ids),
                'complete': complete, 'confirmed_ids': sorted(confirmed),
                'unconfirmed_ids': sorted(ids-confirmed), 'ambiguous_ids': ambiguous,
                'faults': dict(faults), 'rounds': attempts, 'physical_cutoff_required': not complete,
                'poisoned': True, 'cleanup_only': True, 'active_deadline_extended': False,
                'motion_retry_allowed': False, 'fixed_six_stop_used': False,
                'retry_policy': {'max_rounds': rounds, 'rounds_completed': len(attempts),
                    'total_budget_ns': total_budget_ns, 'first_round_cap_ns': 250_000_000,
                    'retry_round_cap_ns': 500_000_000, 'begin_ns': begin, 'end_ns': end,
                    'deadline_ns': deadline,
                    'budget_exhausted': deadline-end < 21_000_000 and not complete,
                    'ambiguity_preserved': True}}

    def close(self):
        if not self._closed:
            try:
                self._session.close()
            finally:
                _TRANSPORTS.pop(self, None)
                self._closed = True
