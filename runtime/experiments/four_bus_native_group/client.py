"""Explicit, optional Python lease for the four-owner native Type1 group.

Importing this module opens no library, descriptor or thread. The composite
library and four already-preflighted Type1 transports are supplied explicitly.
This client does not grant motion or timing qualification.
"""
import ctypes as C
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
import weakref

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.native_diagnostic_transport import Record
from experiments.four_bus_diagnostic.transport_adapter import PORTS
from experiments.four_bus_type1 import type1_transport as T
from . import build


class GroupPort(C.Structure):
    _fields_ = [('owner_started_ns', C.c_uint64), ('owner_finished_ns', C.c_uint64),
                ('status', C.c_int32), ('count', C.c_uint32), ('records', Record*3),
                ('stats', active.Stats), ('error', C.c_char*256)]


class GroupResult(C.Structure):
    _fields_ = [(key, C.c_uint64) for key in (
        'ticket', 'submitted_ns', 'validated_ns', 'released_ns', 'collected_ns',
        'cancel_requested_ns')] + [('ports', GroupPort*4)]


_SYMBOLS = ('sdg_abi', 'sdg_result_size', 'sdg_port_size', 'sdg_create',
            'sdg_initial_hold', 'sdg_start_hold', 'sdg_wait_hold', 'sdg_wait_voltage', 'sdg_output',
            'sdg_cancel', 'sdg_cancel_snapshot', 'sdg_close')
_ARGS = ((), (), (),
         (C.POINTER(C.c_void_p), C.POINTER(C.c_uint32), C.POINTER(C.c_uint64), C.c_uint64,
          C.POINTER(active.PairOwnerSettings), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.POINTER(C.c_ubyte), C.c_uint64, C.POINTER(GroupResult),
          C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.POINTER(C.c_ubyte), C.POINTER(C.c_ubyte), C.c_uint64,
          C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.c_uint64, C.POINTER(GroupResult), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.c_uint64, C.POINTER(GroupResult), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.c_uint64, C.POINTER(C.c_ubyte), C.c_uint64, C.c_uint64,
          C.c_int, C.POINTER(GroupResult), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p,),
         (C.c_void_p, C.POINTER(GroupResult), C.POINTER(GroupResult), C.POINTER(GroupResult),
          C.POINTER(C.c_uint32), C.POINTER(C.c_char), C.c_uint32),
         (C.c_void_p, C.POINTER(active.PairOwnerSettings), C.POINTER(C.c_char), C.c_uint32))
_RESULTS = (C.c_uint32, C.c_uint32, C.c_uint32, C.c_void_p,
            C.c_int, C.c_int, C.c_int, C.c_int, C.c_int, None, C.c_int, C.c_int)
_TOKEN = object()
_LIBRARIES = weakref.WeakKeyDictionary()
_GROUP_TOKEN = object()
_GROUPS = weakref.WeakKeyDictionary()
_PROVISIONAL_GROUPS = weakref.WeakKeyDictionary()
_LIVE_GROUPS = set()  # A forgotten lease cannot be GC'd while C owns Sessions.
_FAILED_ATTACHES = []  # Retain any C lease that could not be joined during constructor rollback.


@dataclass(frozen=True)
class _GroupSeal:
    token: object
    library: object
    functions: tuple
    pins: tuple
    receipt_sha256: str


@dataclass(frozen=True)
class _GroupCreation:
    token: object
    reference: object
    adapters: tuple
    sessions: tuple
    library: object
    library_seal: object
    binding: tuple
    handle: int
    api_lock: object
    state_lock: object


def _function_seal(function):
    return (function, tuple(function.argtypes or ()), function.restype,
            getattr(function, 'errcheck', None), function._flags_)


def load_group_library(path, *, expected_sha256, ordinary_source_sha256,
                       subset_stop_source_sha256, subset_active_source_sha256,
                       group_source_sha256, build_record_sha256):
    """Authenticate the composite DSO, its four source copies and the C ABI."""
    path = Path(path).resolve(strict=True)
    directory = path.parent
    pins = (ordinary_source_sha256, subset_stop_source_sha256,
            subset_active_source_sha256, group_source_sha256)

    def check_files():
        problem = build.receipt_problem(directory)
        T.need(problem is None, problem or '')
        T.need(T.sha(directory/'build-record.json') == build_record_sha256,
               'Native group receipt changed')
        record = json.loads((directory/'build-record.json').read_bytes())
        scope = record['four_bus_native_group']
        T.need(tuple(scope['included_source_sha256'][key] for key in build.SOURCE_KEYS) == pins,
               'Explicit four native group source pins required')
        T.need(tuple(T.sha(directory/name) for name in build.SOURCE_NAMES) == pins,
               'Native group source copies changed')
        return scope

    scope = check_files()
    lib = T.load_library(path, expected_sha256=expected_sha256,
        ordinary_source_sha256=ordinary_source_sha256,
        subset_stop_source_sha256=subset_stop_source_sha256,
        extension_source_sha256=group_source_sha256,
        build_record_sha256=build_record_sha256)
    try:
        functions = tuple(lib[name] for name in _SYMBOLS)
    except AttributeError as error:
        raise ValueError('Explicit native four-owner group ABI required') from error
    for name, function, arguments, result in zip(_SYMBOLS, functions, _ARGS, _RESULTS):
        function.argtypes = list(arguments)
        function.restype = result
        setattr(lib, name, function)
    if (lib.sdg_abi() != 1 or lib.sdg_result_size() != C.sizeof(GroupResult) or
            lib.sdg_port_size() != C.sizeof(GroupPort)):
        raise ValueError('Native group ABI/layout mismatch')
    queries = scope['queried_abi']
    if (queries['sdg_abi'] != 1 or queries['sdg_result_size'] != C.sizeof(GroupResult) or
            queries['sdg_port_size'] != C.sizeof(GroupPort)):
        raise ValueError('Native group receipt ABI sizes differ')
    check_files()
    seal = _GroupSeal(_TOKEN, weakref.ref(lib), tuple(_function_seal(f) for f in functions),
                      pins, build_record_sha256)
    lib._four_owner_group_seal = seal
    _LIBRARIES[lib] = weakref.ref(seal)
    verify_group_library(lib)
    return lib


def verify_group_library(lib):
    seal = getattr(lib, '_four_owner_group_seal', None)
    registered = _LIBRARIES.get(lib) if type(lib) is C.CDLL else None
    if (registered is None or registered() is not seal or type(seal) is not _GroupSeal or
            seal.token is not _TOKEN or seal.library() is not lib):
        raise ValueError('Genuine authenticated composite group library required')
    T.verify_library(lib)
    for name, original in zip(_SYMBOLS, seal.functions):
        function = getattr(lib, name, None)
        if function is not original[0] or _function_seal(function) != original:
            raise ValueError('Native group function binding changed')
    return seal


def verify_group_publication(lease, adapter, label, records, stats, wires):
    """Consume one genuine just-collected private native result, once only."""
    if type(lease) is not NativeGroup or type(adapter) is not T.Type1Transport:
        raise ValueError('Genuine native group result required')
    if not lease._api_lock.locked() or lease._api_owner != threading.get_ident():
        raise ValueError('Native group publication requires its active operation')
    lease._bound()
    key = id(adapter)
    claim = lease._claims.pop(key, None)
    if (claim is None or claim[0] is not adapter or claim[1] != label or
            claim[2] is not records or claim[3] is not stats or
            claim[4] != tuple(wires) or claim[5] != bytes(records) or
            claim[6] != bytes(stats)):
        raise ValueError('Only one original native group buffer may be published')


def _error(error):
    return error.value.decode('utf-8', errors='replace') or 'Native group operation failed'


def _array(wires):
    return (C.c_ubyte*sum(map(len, wires))).from_buffer_copy(b''.join(wires))


def _raw_port(port, count):
    records = (Record*count)()
    for index in range(count):
        records[index] = port.records[index]
    stats = active.Stats.from_buffer_copy(bytes(port.stats))
    return records, stats


def _settings_rows(rows):
    return [dict((field, int(getattr(row, field))) for field, _ in active.PairOwnerSettings._fields_)
            for row in rows]


class NativeGroup:
    """Borrow four genuine same-DSO Type1 Sessions until close has joined C owners.

    The ordinary Python owner threads remain alive for preflight and STOP.
    Every native phase must be consumed in order under its exact ticket.
    """
    def __init__(self, adapters_by_port, *, cpu_masks=(1, 2, 4, 8), timer_slack_ns=1000,
                 socket_fixture=False):
        if (type(adapters_by_port) is not dict or set(adapters_by_port) != set(PORTS) or
                any(type(adapters_by_port[p]) is not T.Type1Transport for p in PORTS)):
            raise ValueError('Exactly four genuine physical Type1 transports required')
        if type(socket_fixture) is not bool:
            raise ValueError('Explicit socket fixture selection required')
        if socket_fixture:
            if cpu_masks != (0, 0, 0, 0) or timer_slack_ns != 0:
                raise ValueError('Socket-only fixture requires zero placement and slack')
        elif cpu_masks != (1, 2, 4, 8) or timer_slack_ns != 1000:
            raise ValueError('Actual four CPU owners require masks 1,2,4,8 and 1000 ns slack')
        self.adapters = dict(adapters_by_port)
        adapters = tuple(self.adapters[p] for p in PORTS)
        lib = adapters[0]._library
        self._seal = verify_group_library(lib)
        sessions = tuple(adapter._session for adapter in adapters)
        if (len({id(s) for s in sessions}) != 4 or any(a._library is not lib or s.lib is not lib
                for a, s in zip(adapters, sessions)) or
                {mid for a in adapters for mid in a.group.ids} != set(range(1, 13))):
            raise ValueError('Four distinct same-DSO sessions covering 12 IDs required')
        for adapter in adapters:
            adapter._verify()  # Before borrowing: this rejects a previous/altered binding.
            if (adapter.poisoned or adapter._zero_gain != set(adapter.group.ids) or
                    adapter._last_output is None or adapter._last_fields is None or
                    not adapter.separate_write_deadline):
                raise ValueError('Four confirmed zero-gain outputs and write-capable owners required')
        self.lib = lib
        self.sessions = sessions  # Strong ownership of Session, DSO and caller-owned FD bindings.
        self._binding = tuple((s._handle, s.fd, s.first_id, bytes(s._limits),
                               active.verified_active_session_creation(s)) for s in sessions)
        self._api_lock = threading.Lock()
        self._api_owner = None
        self._state_lock = threading.Lock()
        self._handle = None
        self._closing = self._native_closing = self._closed = self._poisoned = False
        self._ticket = None
        self._phase = 'ready'
        self._initial_done = False
        self._claims = {}
        # Only collection/cancellation metadata may change when native work is
        # recovered during close. Keep the original port image for comparison.
        self._journaled = {}
        self.raw_snapshot_conflicts = []
        self.unissued_phase_failures = []
        self._close_report = None
        self.journal = []
        self.attempts = {'type1_sent': False, 'positive_gain_sent': False}
        self.settings_at_create = self.settings_at_close = None
        acquired = []
        _PROVISIONAL_GROUPS[self] = _GROUP_TOKEN
        self._api_lock.acquire()  # close() waits for create/rollback before touching C ownership.
        try:
            for session in sessions:
                if not session.busy.acquire(blocking=False):
                    raise RuntimeError('Native group Session busy before ownership transfer')
                acquired.append(session)
                if not session._handle or session.poisoned or session._phase_pair is not None:
                    raise RuntimeError('Native group Session closed, poisoned or already borrowed')
            # Reserve Python ownership before sdg_create can release the GIL.
            # ActiveSession.close must refuse during native owner creation too.
            for session in sessions:
                session._phase_pair = self
            handles = (C.c_void_p*4)(*(s._handle for s in sessions))
            masks = (C.c_uint32*4)(*(a.group.mask for a in adapters))
            cpus = (C.c_uint64*4)(*cpu_masks)
            settings = (active.PairOwnerSettings*4)()
            error = C.create_string_buffer(256)
            handle = lib.sdg_create(handles, masks, cpus, timer_slack_ns,
                                    settings, error, len(error))
            if not handle:
                raise ValueError(_error(error))
            self._handle = handle
            self.settings_at_create = _settings_rows(settings)
            _GROUPS[self] = _GroupCreation(
                _GROUP_TOKEN, weakref.ref(self), adapters, sessions, lib, self._seal,
                self._binding, handle, self._api_lock, self._state_lock)
            _LIVE_GROUPS.add(self)
        except BaseException as construction_error:
            # Null create guarantees the native owners have joined and released
            # their Session reservations; an unexpected post-create exception
            # must explicitly join before returning control to old STOP code.
            if self._handle is not None:
                self._poisoned = True
                for adapter in self.adapters.values():
                    adapter._poisoned = True
                    adapter._session.poisoned = True
                try:
                    self.lib.sdg_cancel(self._handle)
                    restored = (active.PairOwnerSettings*4)()
                    close_error = C.create_string_buffer(256)
                    status = self.lib.sdg_close(self._handle, restored, close_error, len(close_error))
                except BaseException as close_error:
                    _FAILED_ATTACHES.append(self)
                    raise RuntimeError('Native group constructor rollback could not join; lease retained') from close_error
                try:
                    self.settings_at_close = _settings_rows(restored)
                except BaseException:
                    # The native join already completed; Python formatting must
                    # not leave a borrowed Session stranded after C destruction.
                    self.settings_at_close = None
                self._handle = None
                self._closed = True
                self._close_report = {'closed': True, 'restored': status == 0,
                                      'settings': self.settings_at_close,
                                      'error': None if status == 0 else _error(close_error)}
            else:
                self._closed = True
                self._close_report = {'closed': True, 'restored': False,
                                      'settings': None, 'error': str(construction_error)}
            _GROUPS.pop(self, None)
            _LIVE_GROUPS.discard(self)
            for session in reversed(acquired):
                if session._phase_pair is self:
                    session._phase_pair = None
                session.busy.release()
            raise
        finally:
            _PROVISIONAL_GROUPS.pop(self, None)
            self._api_lock.release()

    @property
    def closed(self):
        return self._closed

    @property
    def poisoned(self):
        return self._poisoned or any(session.poisoned for session in self.sessions)

    @property
    def settings(self):
        return self.settings_at_create

    @property
    def close_report(self):
        return self._close_report

    def _creation_entry(self):
        creation = _GROUPS.get(self)
        if (type(creation) is not _GroupCreation or creation.token is not _GROUP_TOKEN or
                creation.reference() is not self or creation.library is not self.lib or
                creation.library_seal is not self._seal or creation.sessions != self.sessions or
                creation.binding != self._binding or creation.handle != self._handle or
                creation.api_lock is not self._api_lock or
                creation.state_lock is not self._state_lock or
                any(self.adapters.get(port) is not creation.adapters[i]
                    for i, port in enumerate(PORTS))):
            raise ValueError('Original native group creation/binding required')
        return creation

    def _bound(self):
        self._creation_entry()
        verify_group_library(self.lib)
        if self._closed or self._closing or self._handle is None or self.poisoned:
            raise RuntimeError('Native group closed, closing or poisoned')
        for session, saved in zip(self.sessions, self._binding):
            if (session._phase_pair is not self or
                    (session._handle, session.fd, session.first_id, bytes(session._limits),
                     active.verified_active_session_creation(session)) != saved):
                raise RuntimeError('Native group Session binding changed')

    def _abort(self, error):
        self._poisoned = True
        for adapter in self.adapters.values():
            adapter._poisoned = True
            adapter._session.poisoned = True
            adapter.failures.append(type(error).__name__+': '+str(error))
        self.cancel()
        # Shared cancellation is requested even if native reports a partial result.
        try:
            next(iter(self.adapters.values()))._cancel_all()
        except BaseException as cancel_error:
            for adapter in self.adapters.values():
                adapter.failures.append('cancel_all failed: '+str(cancel_error))

    def cancel(self):
        """Asynchronous cancellation; safe while another thread waits in ctypes."""
        with self._state_lock:
            if self._closed:
                return
            if _PROVISIONAL_GROUPS.get(self) is not _GROUP_TOKEN:
                self._creation_entry()
            self._poisoned = True
            for adapter in self.adapters.values():
                adapter._poisoned = True
                adapter._session.poisoned = True
            if self._handle is not None and not self._native_closing:
                self.lib.sdg_cancel(self._handle)

    def _enter(self):
        if not self._api_lock.acquire(blocking=False):
            raise RuntimeError('Concurrent native group operation')
        try:
            self._bound()
            self._api_owner = threading.get_ident()
        except BaseException:
            self._api_lock.release()
            raise

    def _leave(self):
        self._api_owner = None
        self._api_lock.release()

    def _wires(self, wire_map, label):
        if type(wire_map) is not dict or set(wire_map) != set(PORTS):
            raise ValueError('Exact four-port wire map required')
        flattened = []
        normalized = {}
        for port in PORTS:
            adapter = self.adapters[port]
            wires, _, _ = adapter._validate(label, wire_map[port])
            flattened.extend(wires)
            normalized[port] = wires
        return _array(flattened), normalized

    def _evidence(self, result, label, index, raw):
        native = result.ports[index]
        return {'port': PORTS[index], 'label': label, 'ticket': int(result.ticket),
                'status': int(native.status), 'count': int(native.count),
                'raw_count': len(raw[0]),
                'written_count': sum(record.written == 17 for record in raw[0]),
                'received_count': sum(record.received == 17 for record in raw[0]),
                'owner_started_ns': int(native.owner_started_ns),
                'owner_finished_ns': int(native.owner_finished_ns),
                'error': bytes(native.error).split(b'\0', 1)[0].decode('utf-8', errors='replace'),
                'raw': raw}

    @staticmethod
    def _port_fingerprint(native):
        return (int(native.status), int(native.count), int(native.owner_started_ns),
                int(native.owner_finished_ns), bytes(native.records), bytes(native.stats),
                bytes(native.error))

    @staticmethod
    def _fingerprint_record(fingerprint):
        status, count, begin, end, records, stats, error = fingerprint
        return {'status': status, 'count': count, 'owner_started_ns': begin,
                'owner_finished_ns': end, 'records_hex': records.hex(),
                'stats_hex': stats.hex(), 'error_hex': error.hex()}

    def _check_journaled(self, result, label, index):
        key = (int(result.ticket), label, PORTS[index])
        original = self._journaled.get(key)
        if original is None:
            return False
        actual = self._port_fingerprint(result.ports[index])
        if original != actual:
            # Keep both images without replacing or republishing the original
            # ACK. A close-time conflict must still join and permit only STOP.
            self.raw_snapshot_conflicts.append({
                'ticket': key[0], 'label': key[1], 'port': key[2],
                'admission_eligible': False, 'original_journal_retained': True,
                'expected': self._fingerprint_record(original),
                'actual': self._fingerprint_record(actual)})
            raise ValueError('Native group repeated raw image differs: '+str(key))
        return True

    def _retain_partial(self, result, label):
        """Keep actual native partial writes/replies, including rejection stats."""
        for index, port in enumerate(PORTS):
            key = (int(result.ticket), label, port)
            if self._check_journaled(result, label, index):
                continue
            native = result.ports[index]
            requested = max(0, min(3, int(native.count)))
            # count is the issued request's width, not the number of records
            # touched before cancellation. Keep only actual native record slots.
            touched = [record_index+1 for record_index in range(requested)
                       if any(bytes(native.records[record_index]))]
            raw = _raw_port(native, max(touched, default=0))
            evidence = self._evidence(result, label, index, raw)
            if (len(raw[0]) or native.status or native.owner_started_ns or native.owner_finished_ns or
                    evidence['error'] or any(bytes(raw[1]))):
                self.adapters[port].journal.append((label, raw))
                self.journal.append(evidence)
                self._journaled[key] = self._port_fingerprint(native)

    @staticmethod
    def _has_native_exchange_evidence(result):
        # A pre-publication rejection may contain a ticket, attempt clocks,
        # statuses and error strings, but no owner job or wire exchange. Do
        # not confuse those diagnostics with a partial output transaction.
        return any(port.count or port.owner_started_ns or port.owner_finished_ns or
                   any(bytes(port.records)) or any(bytes(port.stats))
                   for port in result.ports)

    def _publish(self, result, label, expected_count, wires_by_port, deadline_ns,
                 write_deadline_ns=None):
        """Validate all C port envelopes before any original Python publication."""
        expected_ticket = 0 if not self._initial_done and label == 'output' and self._ticket is None else self._ticket
        if (result.ticket != expected_ticket or not 0 < result.submitted_ns <=
                result.validated_ns <= result.released_ns <= result.collected_ns or
                result.released_ns >= deadline_ns):
            self._retain_partial(result, label)
            raise ValueError('Native group result ticket/stamps differ from current operation')
        if (any(result.ports[i].status or result.ports[i].count != expected_count
                for i in range(4))):
            self._retain_partial(result, label)
            raise RuntimeError('Native group '+label+' had failed or incomplete port results')
        for index in range(4):
            if self._check_journaled(result, label, index):
                raise ValueError('Native group raw image already journaled before publication')
        batches = {}
        raws = [_raw_port(result.ports[index], expected_count) for index in range(4)]
        for index, port in enumerate(PORTS):
            adapter = self.adapters[port]
            records, stats = raws[index]
            raw = (records, stats)
            self._claims[id(adapter)] = (adapter, label, records, stats,
                                         wires_by_port[port], bytes(records), bytes(stats))
            journal_before = len(adapter.journal)
            try:
                batches[port] = adapter.publish_group_raw(
                    label, records, stats, lease=self, wires=wires_by_port[port],
                    reply_deadline_ns=deadline_ns, write_deadline_ns=write_deadline_ns,
                    dispatched_ns=int(result.released_ns) if write_deadline_ns is not None else None)
            except BaseException:
                # The original publisher may have journaled this port already.
                # Preserve all remaining native raw buffers before fail-closed STOP.
                if len(adapter.journal) == journal_before:
                    adapter.journal.append((label, raw))
                for later in range(index+1, 4):
                    later_port = PORTS[later]
                    self.adapters[later_port].journal.append((label, raws[later]))
                    self.journal.append(self._evidence(result, label, later, raws[later]))
                    self._journaled[(int(result.ticket), label, later_port)] = \
                        self._port_fingerprint(result.ports[later])
                raise
            finally:
                self._claims.pop(id(adapter), None)
                self.journal.append(self._evidence(result, label, index, raw))
                self._journaled[(int(result.ticket), label, port)] = \
                    self._port_fingerprint(result.ports[index])
        return batches

    def initial_hold(self, last_wires_by_port, deadline_ns, check):
        """One fresh zero-gain ACK per axis after native owners are ready."""
        self._enter()
        try:
            if self._phase != 'ready' or self._initial_done or not callable(check):
                raise ValueError('Initial native hold is once before regular tickets')
            check()
            flat, wires = self._wires(last_wires_by_port, 'output')
            for port in PORTS:
                for wire in wires[port]:
                    fields = T.type1_fields(wire)
                    if fields is None or fields[2] or fields[3]:
                        raise ValueError('Initial native group hold requires zero Kp/Kd')
            for port in PORTS:
                self.adapters[port]._mark(wires[port])
            self.attempts['type1_sent'] = True
            result = GroupResult()
            error = C.create_string_buffer(256)
            status = self.lib.sdg_initial_hold(self._handle, flat, deadline_ns,
                                                C.byref(result), error, len(error))
            if status:
                self._retain_partial(result, 'output')
                raise RuntimeError(_error(error))
            batches = self._publish(result, 'output', 3, wires, deadline_ns)
            self._initial_done = True
            return batches
        except BaseException as error:
            self._abort(error)
            raise
        finally:
            self._leave()

    def start_hold(self, last_wires_by_port, voltage_id_by_port, deadline_ns, check):
        self._enter()
        try:
            if self._phase != 'ready' or not callable(check):
                raise ValueError('Current idle native group and explicit guard required')
            if type(voltage_id_by_port) is not dict or set(voltage_id_by_port) != set(PORTS):
                raise ValueError('Exact four rotating voltage IDs required')
            check()
            hold, hold_wires = self._wires(last_wires_by_port, 'feedback_hold')
            voltages = {}
            for port in PORTS:
                mid = voltage_id_by_port[port]
                adapter = self.adapters[port]
                if type(mid) is not int or mid not in adapter.group.ids:
                    raise ValueError('Voltage ID outside its exact group')
                voltages[port] = (adapter._wires['voltage'][adapter.group.ids.index(mid)],)
            voltage, voltage_wires = self._wires(voltages, 'voltage')
            ticket = C.c_uint64()
            error = C.create_string_buffer(256)
            self.attempts['type1_sent'] = True
            for adapter in self.adapters.values():
                adapter.attempts['type1_sent'] = True
            status = self.lib.sdg_start_hold(self._handle, hold, voltage, deadline_ns,
                                             C.byref(ticket), error, len(error))
            if status or not ticket.value:
                raise RuntimeError(_error(error))
            self._ticket = ticket.value
            self._voltage_ids = dict(voltage_id_by_port)
            self._hold_wires, self._voltage_wires = hold_wires, voltage_wires
            self._hold_deadline = deadline_ns
            self._phase = 'hold_started'
            return ticket.value
        except BaseException as error:
            self._abort(error)
            raise
        finally:
            self._leave()

    def _wait(self, ticket, *, label, expected_phase, function, count):
        self._enter()
        try:
            if type(ticket) is not int or ticket != self._ticket or self._phase != expected_phase:
                raise ValueError('Exact current unconsumed ordered native group ticket required')
            result = GroupResult()
            error = C.create_string_buffer(256)
            status = function(self._handle, ticket, C.byref(result), error, len(error))
            if status:
                self._retain_partial(result, label)
                raise RuntimeError(_error(error))
            wires = self._hold_wires if label == 'feedback_hold' else self._voltage_wires
            batches = self._publish(result, label, count, wires, self._hold_deadline)
            self._phase = 'hold_taken' if label == 'feedback_hold' else 'voltage_taken'
            return batches
        except BaseException as error:
            self._abort(error)
            raise
        finally:
            self._leave()

    def wait_hold(self, ticket):
        return self._wait(ticket, label='feedback_hold', expected_phase='hold_started',
                          function=self.lib.sdg_wait_hold, count=3)

    def wait_voltage(self, ticket):
        return self._wait(ticket, label='voltage', expected_phase='hold_taken',
                          function=self.lib.sdg_wait_voltage, count=1)

    def output(self, wires_by_port, reply_deadline_ns, write_deadline_ns, check):
        self._enter()
        try:
            if self._phase != 'voltage_taken' or not callable(check):
                raise ValueError('Both current-ticket input phases and explicit guard required')
            check()
            output, output_wires = self._wires(wires_by_port, 'output')
            for port in PORTS:
                adapter = self.adapters[port]
                _, fields = adapter._checked_output(output_wires[port])
                adapter._mark(output_wires[port], fields)
            self.attempts['type1_sent'] = True
            self.attempts['positive_gain_sent'] |= any(
                adapter.attempts['positive_gain_sent'] for adapter in self.adapters.values())
            result = GroupResult()
            error = C.create_string_buffer(256)
            status = self.lib.sdg_output(self._handle, self._ticket, output,
                reply_deadline_ns, write_deadline_ns, 1, C.byref(result), error, len(error))
            if status:
                native_error = _error(error)
                raw_observed = self._has_native_exchange_evidence(result)
                if not result.released_ns:
                    self.unissued_phase_failures.append({
                        'label': 'output', 'expected_ticket': int(self._ticket),
                        'result_ticket': int(result.ticket),
                        'submitted_ns': int(result.submitted_ns),
                        'validated_ns': int(result.validated_ns),
                        'released_ns': int(result.released_ns),
                        'collected_ns': int(result.collected_ns),
                        'issued': False, 'raw_observed': raw_observed,
                        'native_error': native_error})
                if result.released_ns or raw_observed:
                    # A published job may have partial wire evidence. Preserve
                    # and compare it against any later close-time snapshot.
                    # Unexpected raw data with released_ns==0 is still real
                    # evidence; never silently classify it as an empty phase.
                    try:
                        self._retain_partial(result, 'output')
                    except BaseException as retention_error:
                        raise RuntimeError(native_error+'; partial raw conflict: '+
                                           str(retention_error)) from retention_error
                raise RuntimeError(native_error)
            batches = self._publish(result, 'output', 3, output_wires,
                                    reply_deadline_ns, write_deadline_ns)
            self._phase = 'ready'
            return batches
        except BaseException as error:
            self._abort(error)
            raise
        finally:
            self._leave()

    def close(self):
        """Cancel first, join native owners, then release Python leases for STOP."""
        with self._state_lock:
            if self._closed:
                return self._close_report
            if _PROVISIONAL_GROUPS.get(self) is not _GROUP_TOKEN:
                self._creation_entry()
            self._closing = True
            if self._handle is not None and _GROUPS.get(self) is not None:
                self.lib.sdg_cancel(self._handle)
        self._api_lock.acquire()
        try:
            with self._state_lock:
                if self._closed:
                    return self._close_report
                self._creation_entry()
                handle = self._handle
                self._native_closing = True  # No concurrent sdg_cancel on a soon-to-be-freed handle.
            snapshot = {'phase_bits': 0, 'complete': False, 'error': None}
            try:
                hold, voltage, output = GroupResult(), GroupResult(), GroupResult()
                bits = C.c_uint32()
                snapshot_error = C.create_string_buffer(256)
                snapshot_status = self.lib.sdg_cancel_snapshot(
                    handle, C.byref(hold), C.byref(voltage), C.byref(output),
                    C.byref(bits), snapshot_error, len(snapshot_error))
                snapshot['phase_bits'] = bits.value
                snapshot['complete'] = snapshot_status == 0
                if snapshot_status:
                    snapshot['error'] = _error(snapshot_error)
                else:
                    # Presence bits come from issued native jobs. Zero-filled
                    # absent phases and already-journaled ports never become ACKs.
                    for bit, result, label in ((1, hold, 'feedback_hold'),
                                               (2, voltage, 'voltage'),
                                               (4, output, 'output'),
                                               (8, output, 'output')):
                        if bits.value & bit:
                            self._retain_partial(result, label)
            except BaseException as snapshot_error:
                snapshot['complete'] = False
                snapshot['error'] = type(snapshot_error).__name__+': '+str(snapshot_error)
                self._poisoned = True
            settings = (active.PairOwnerSettings*4)()
            error = C.create_string_buffer(256)
            status = self.lib.sdg_close(handle, settings, error, len(error))
            self._handle = None  # C destroys the group even when restoration reports failure.
            self._closed = True
            # A closed native lease is a terminal handback to bounded STOP,
            # never authority for another active command on old Sessions.
            self._poisoned = True
            for adapter in self.adapters.values():
                adapter._poisoned = True
                adapter._session.poisoned = True
            try:
                self.settings_at_close = _settings_rows(settings)
            except BaseException as formatting_error:
                self.settings_at_close = None
                snapshot['error'] = ((snapshot['error'] or '') +
                    ' close-settings formatting: '+type(formatting_error).__name__)
            _GROUPS.pop(self, None)
            _LIVE_GROUPS.discard(self)
            for session in self.sessions:
                if session._phase_pair is self:
                    session._phase_pair = None
            for session in reversed(self.sessions):
                session.busy.release()
            self._close_report = {'closed': True, 'restored': status == 0,
                                  'settings': self.settings_at_close,
                                  'error': None if status == 0 else _error(error),
                                  'snapshot': snapshot,
                                  'snapshot_complete': snapshot['complete'],
                                  'snapshot_error': snapshot['error'],
                                  'raw_snapshot_conflicts': list(self.raw_snapshot_conflicts),
                                  'unissued_phase_failures': list(self.unissued_phase_failures)}
            return self._close_report
        finally:
            self._api_lock.release()


__all__ = ('GroupPort', 'GroupResult', 'NativeGroup', 'load_group_library',
           'verify_group_library', 'verify_group_publication')
