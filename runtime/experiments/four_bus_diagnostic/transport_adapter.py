"""Four-bus experiment: genuine ordinary sessions, exact-three STOP whitelist.

This module opens no device. Caller-owned descriptors and actual boot/cancel
bindings remain required. The optional recovery symbol never uses fixed-six
emergency_stop and never clears a poisoned session or sticky ambiguity.
"""
import ctypes as C
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading
import time
from types import MappingProxyType
import weakref

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire, exchange_evidence
from singularitydog_hw.policy_output_runtime import decode_records
from singularitydog_hw.rs05_trial_protocol import Type2Feedback

PORTS = ('port0', 'port1', 'port2', 'port3')
IDS = tuple(range(1, 13))
GROUPS = ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12))
GAP_NS = 900_000
WINDOW = 3
PERIOD_NS = 20_000_000
_SUBSET_SYMBOLS = ('sda_emergency_stop_subset_abi', 'sda_emergency_stop_subset')
_SUBSET_ARGS = (C.c_void_p, C.c_uint32, C.c_uint64, C.POINTER(Record),
                C.POINTER(active.Stats), C.POINTER(active.StopResult),
                C.POINTER(C.c_char), C.c_uint32)
_LIBRARIES = weakref.WeakKeyDictionary()
_ADAPTERS = weakref.WeakKeyDictionary()
_TOKEN = object()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class Group:
    port: str
    ids: tuple

    def __post_init__(self):
        if self.port not in PORTS or type(self.ids) is not tuple or self.ids not in GROUPS:
            raise ValueError('Explicit port and one original half-envelope group of three required')

    @property
    def first_id(self):
        return 1 if self.ids[0] <= 6 else 7

    @property
    def mask(self):
        return sum(1 << (mid - self.first_id) for mid in self.ids)


@dataclass(frozen=True)
class _LibrarySeal:
    token: object
    library: object
    active_binding: object
    functions: tuple
    build_sha256: str
    ordinary_source_sha256: str


def _function_seal(function):
    return (function, tuple(function.argtypes or ()), function.restype,
            getattr(function, 'errcheck', None), function._flags_)


def load_library(path, *, expected_sha256, ordinary_source_sha256,
                 extension_source_sha256, build_record_sha256):
    """Explicit setup authentication, including the separately included source."""
    path = Path(path).resolve(strict=True)
    build = path.parent / 'build-record.json'
    if sha(build) != build_record_sha256:
        raise ValueError('Four-bus build receipt differs')
    record = json.loads(build.read_bytes())
    scope = record.get('four_bus_subset_stop', {})
    if (scope.get('schema') != 'singularitydog.four-bus-subset-build.v1' or
            scope.get('abi') != 1 or scope.get('allowed_masks') != [7, 56] or
            scope.get('original_active_abi') != 1 or
            scope.get('ordinary_source_sha256') != ordinary_source_sha256 or
            scope.get('extension_source_sha256') != extension_source_sha256 or
            sha(path.parent / 'ordinary_transport.cpp') != ordinary_source_sha256 or
            sha(path.parent / 'transport.cpp') != extension_source_sha256 or
            len((path.parent / 'ordinary_transport.cpp').read_bytes()) != scope.get('ordinary_source_bytes') or
            len((path.parent / 'transport.cpp').read_bytes()) != scope.get('extension_source_bytes') or
            scope.get('output_allowed') is not False or scope.get('timing_admission_eligible') is not False):
        raise ValueError('Original included source and exact-three extension pins required')
    lib = active.load_library(path, expected_sha256=expected_sha256)
    try:
        # Resolve from the authenticated dlopen handle rather than trusting a
        # caller-replaceable cached CDLL attribute before subset setup.
        abi, function = (lib[name] for name in _SUBSET_SYMBOLS)
    except AttributeError:
        raise ValueError('Explicit subset STOP ABI is required; fixed-six fallback prohibited')
    abi.argtypes = []; abi.restype = C.c_uint32
    function.argtypes = list(_SUBSET_ARGS); function.restype = C.c_int
    lib.sda_emergency_stop_subset_abi = abi
    lib.sda_emergency_stop_subset = function
    if abi() != 1:
        raise ValueError('Subset STOP ABI mismatch')
    binding = active.verified_active_source_binding(lib)
    if (binding.build_record_sha256 != build_record_sha256 or
            binding.source_sha256 != extension_source_sha256 or
            binding.binary_sha256 != expected_sha256 or binding.path != str(path) or
            binding.source_path != str(path.parent/'transport.cpp') or
            binding.build_record_path != str(build) or sha(build) != build_record_sha256 or
            sha(path.parent/'transport.cpp') != extension_source_sha256 or
            sha(path.parent/'ordinary_transport.cpp') != ordinary_source_sha256):
        raise ValueError('Subset source/receipt changed across authenticated load')
    seal = _LibrarySeal(_TOKEN, weakref.ref(lib), binding,
                        (_function_seal(abi), _function_seal(function)),
                        build_record_sha256, ordinary_source_sha256)
    lib._four_bus_subset_seal = seal
    _LIBRARIES[lib] = weakref.ref(seal)
    verify_library(lib)
    return lib


def verify_library(lib):
    seal = getattr(lib, '_four_bus_subset_seal', None)
    registered = _LIBRARIES.get(lib) if type(lib) is C.CDLL else None
    if (registered is None or registered() is not seal or type(seal) is not _LibrarySeal or
            seal.token is not _TOKEN or seal.library() is not lib or
            active.verified_active_source_binding(lib) is not seal.active_binding):
        raise ValueError('Genuine authenticated subset library required')
    for name, original in zip(_SUBSET_SYMBOLS, seal.functions):
        current = getattr(lib, name, None)
        if current is not original[0] or _function_seal(current) != original:
            raise ValueError('Subset function binding changed')
    return seal


@dataclass(frozen=True)
class Batch:
    """Own, truthful original count; sealed bytes do not create readiness."""
    group: Group
    label: str
    records: object
    stats: object
    rows: dict
    record_image: bytes
    stats_image: bytes
    completed_ns: int

    def verify(self):
        if bytes(self.records) != self.record_image or bytes(self.stats) != self.stats_image:
            raise ValueError('Owner raw batch was changed after publication')
        # Deliberately retain the original parser at every safety takeout. There
        # is no pretend six-record pure codec in this three-record experiment.
        if decode_records((self.records, self.stats)) != self.rows:
            raise ValueError('Owner decoded rows changed after publication')
        return self.rows

    def verify_images(self):
        """Raw record/Stats images only; decoded rows are not consulted."""
        if bytes(self.records) != self.record_image or bytes(self.stats) != self.stats_image:
            raise ValueError('Owner raw batch was changed after publication')

    def evidence(self):
        value = exchange_evidence(self.records, self.stats)
        value.update(physical_port=self.group.port, physical_ids=list(self.group.ids),
                     phase=self.label, original_record_count=len(self.records),
                     codec='original_python_parser_exact_three_or_one',
                     rejected_total_bytes=int(self.stats.rejected_total),
                     rejected_truncated=self.stats.rejected_total > self.stats.rejected_size,
                     owner_completed_monotonic_ns=self.completed_ns)
        if self.label == 'acquisition_combined4':
            value.update(codec='original_python_parser_exact_four_mixed_requests',
                         stats_scope='original_four_request_exchange',
                         projected_feedback_record_indices=[0, 1, 2],
                         voltage_record_index=3,
                         all_four_replies_joined_before_publication=True)
        return value


class ThreeAxisTransport:
    """One persistent owner; no actuator FD ownership is transferred."""
    def __init__(self, session, group, *, combined_acquisition=False, decode_once=False, token=None):
        if token is not _TOKEN or type(session) is not active.ActiveSession or type(group) is not Group:
            raise ValueError('Use the exact-three creation factory with genuine original session')
        if type(combined_acquisition) is not bool:
            raise ValueError('Combined acquisition selection must be an exact bool')
        if type(decode_once) is not bool:
            raise ValueError('Decode-once selection must be an exact bool')
        self.combined_acquisition = combined_acquisition
        self.decode_once = decode_once
        self._session = session
        self.group = group
        self._library = session.lib
        self._seal = verify_library(session.lib)
        self._creation = active.verified_active_session_creation(session)
        self._binding = (session._handle, session.fd, session.first_id, bytes(session._limits))
        self._owner = None
        self._closed = False
        self.journal = []
        self._batches = []
        # Identity index of the same retained batches: O(1) genuineness lookup.
        self._batch_index = {}
        self._stop_wires = tuple(stop_wire(mid) for mid in group.ids)
        self._voltage_wires = tuple(read_request(mid, 'voltage') for mid in group.ids)
        _ADAPTERS[self] = (weakref.ref(session), group, self._binding,
                           self._stop_wires, self._voltage_wires, combined_acquisition, decode_once)

    @classmethod
    def create(cls, library, fd, *, group, cancel_fd, boot_fd, boot_id,
               raw_lower_by_id, raw_upper_by_id, combined_acquisition=False, decode_once=False):
        verify_library(library)
        if type(group) is not Group:
            raise ValueError('Immutable group required')
        if type(combined_acquisition) is not bool:
            raise ValueError('Combined acquisition selection must be an exact bool')
        if type(decode_once) is not bool:
            raise ValueError('Decode-once selection must be an exact bool')
        ids = tuple(range(group.first_id, group.first_id + 6))
        session = active.ActiveSession(library, fd, first_id=group.first_id,
            cancel_fd=cancel_fd, boot_fd=boot_fd, boot_id=boot_id,
            raw_lower_by_id=raw_lower_by_id, raw_upper_by_id=raw_upper_by_id,
            kp_max_by_id={mid: 0. for mid in ids}, kd_max_by_id={mid: 0. for mid in ids},
            gap_ns=GAP_NS, window=WINDOW)
        try:
            return cls(session, group, combined_acquisition=combined_acquisition,
                       decode_once=decode_once, token=_TOKEN)
        except BaseException:
            session.close()
            raise

    def _verify(self):
        session = self._session
        original = _ADAPTERS.get(self)
        if (original is None or original[0]() is not session or self.group is not original[1] or
                self._binding != original[2] or self._stop_wires != original[3] or
                self._voltage_wires != original[4] or
                self.combined_acquisition is not original[5] or
                self.decode_once is not original[6] or
                self._closed or not session._handle or session.lib is not self._library or
                verify_library(self._library) is not self._seal or
                active.verified_active_session_creation(session) != self._creation or
                (session._handle, session.fd, session.first_id, bytes(session._limits)) != self._binding or
                session._phase_pair is not None):
            raise ValueError('Original three-axis owner/session/source binding changed')

    def _owner_check(self):
        current = threading.get_native_id()
        if self._owner is None:
            self._owner = current
        if current != self._owner:
            raise RuntimeError('Original physical bus worker must retain ownership')
        self._verify()

    def validate_wires(self, wires):
        wires = tuple(wires)
        # No header-only whitelist: exact flags, payload and destination bytes.
        if not (wires == self._stop_wires or
                len(wires) == 1 and wires[0] in self._voltage_wires or
                getattr(self, 'combined_acquisition', False) is True and
                len(wires) == 4 and wires[:3] == self._stop_wires and
                wires[3] in self._voltage_wires):
            raise ValueError('Only exact three Type4 STOPs or one same-group Type17 voltage allowed')
        return wires

    def _exchange(self, wires, deadline_ns, label, *, before_native=None):
        self._owner_check()
        wires = self.validate_wires(wires)
        try:
            raw = self._session.exchange(wires, deadline_ns=deadline_ns, before_native=before_native)
            self.journal.append((label, raw))  # Keep raw even if decoding rejects.
            rows = decode_records(raw)
            if self.decode_once:
                # Sole reference to a private copy: the published rows are read-only.
                rows = MappingProxyType(dict(rows))
            for mid in self.group.ids:
                feedback = rows.get((mid, 'feedback'))
                if feedback is not None and (feedback[0].mode_state != 0 or feedback[0].fault_bits != 0):
                    raise ValueError('STOP diagnostic requires mode-zero fault-zero feedback')
            batch = Batch(self.group, label, raw[0], raw[1], rows,
                          bytes(raw[0]), bytes(raw[1]), time.monotonic_ns())
            self._batches.append(batch)
            self._batch_index[id(batch)] = (batch, rows)
            return batch
        except BaseException as error:
            self._session.poisoned = True
            if getattr(error, 'records', None) is not None and getattr(error, 'stats', None) is not None:
                self.journal.append((label, (error.records, error.stats)))
            raise

    def acquire(self, voltage_id, feedback_future, *, deadline_ns, check):
        """Publish genuine immutable three feedback rows only with owner held.

        The original outer Future remains incomplete through the voltage call.
        Its result, rather than the prefix hint, joins the descriptor owner.
        """
        if self.combined_acquisition is not False:
            raise ValueError('Selected combined acquisition cannot use the separate acquisition route')
        if voltage_id not in self.group.ids:
            raise ValueError('Rotating voltage axis must belong to the physical group')
        from concurrent.futures import Future
        if type(feedback_future) is not Future or feedback_future.done():
            raise ValueError('Genuine current unpublished feedback Future required')
        try:
            feedback = self._exchange(self._stop_wires, deadline_ns, 'feedback')
            def publish():
                check()
                if feedback_future.done():
                    raise ValueError('Feedback Future was externally completed')
                feedback_future.set_result(feedback)
                check()
                return deadline_ns
            voltage = self._exchange((read_request(voltage_id, 'voltage'),), deadline_ns,
                                     'voltage', before_native=publish)
            return feedback, voltage
        except BaseException as error:
            if not feedback_future.done():
                feedback_future.set_exception(error)
            raise

    def acquire_combined(self, voltage_id, feedback_future, *, deadline_ns, check):
        """Join one genuine four-request exchange before publishing its Batch.

        The three STOP requests remain ordered before the one voltage request.
        Window3 may send voltage when an outstanding STOP reply frees a slot;
        no partial prefix or independently successful three/one Stats is made.
        """
        self._owner_check()
        if self.combined_acquisition is not True or voltage_id not in self.group.ids:
            raise ValueError('Explicit combined acquisition and same-group voltage axis required')
        from concurrent.futures import Future
        if type(feedback_future) is not Future or feedback_future.done():
            raise ValueError('Genuine current unpublished feedback Future required')
        try:
            check()
            batch = self._exchange(self._stop_wires + (read_request(voltage_id, 'voltage'),),
                                   deadline_ns, 'acquisition_combined4')
            self.verify_combined_batch(batch, voltage_id)
            check()
            if feedback_future.done():
                raise ValueError('Feedback Future was externally completed')
            feedback_future.set_result(batch)
            check()
            return batch
        except BaseException as error:
            if not feedback_future.done():
                feedback_future.set_exception(error)
            raise

    def output_stop(self, *, deadline_ns, check):
        check()
        return self._exchange(self._stop_wires, deadline_ns, 'output_stop')

    def read_voltage(self, voltage_id, *, deadline_ns, check):
        check()
        if voltage_id not in self.group.ids:
            raise ValueError('Same-group voltage axis required')
        return self._exchange((read_request(voltage_id, 'voltage'),), deadline_ns, 'setup_voltage')

    def verify_batch(self, batch, label):
        # Same identity membership as scanning the retained _batches list.
        entry = self._batch_index.get(id(batch))
        if (type(batch) is not Batch or batch.group is not self.group or batch.label != label or
                entry is None or entry[0] is not batch):
            raise ValueError('Genuine current owner batch required')
        rows = batch.rows
        # Decode-once reuse: only the exact read-only mapping this owner
        # published, holding frozen Type2 rows; anything else is re-decoded.
        if (self.decode_once and rows is entry[1] and type(rows) is MappingProxyType and all(
                type(value) is tuple and len(value) == 3 and type(value[0]) is Type2Feedback and
                type(value[1]) is int and type(value[2]) is int for value in rows.values())):
            batch.verify_images()
            return rows
        return batch.verify()

    def verify_combined_batch(self, batch, voltage_id):
        rows = self.verify_batch(batch, 'acquisition_combined4')
        if (self.combined_acquisition is not True or voltage_id not in self.group.ids or
                len(batch.records) != 4 or
                tuple(bytes(record.tx) for record in batch.records) !=
                self._stop_wires + (read_request(voltage_id, 'voltage'),) or
                set(rows) != {(mid, 'feedback') for mid in self.group.ids} | {(voltage_id, 'voltage')}):
            raise ValueError('Original combined four-request batch binding required')
        return rows

    def recover_subset(self, *, timeout_ns=250_000_000):
        self._owner_check()
        if type(timeout_ns) is not int or not 20_000_000 <= timeout_ns <= 500_000_000:
            raise ValueError('STOP cleanup budget must be 20..500 ms; active deadline is unchanged')
        session = self._session
        if not session.busy.acquire(blocking=False):
            raise RuntimeError('Join original owner before three-axis cleanup')
        try:
            session.poisoned = True
            records, stats, result = (Record * 6)(), active.Stats(), active.StopResult()
            error = C.create_string_buffer(256)
            deadline = time.monotonic_ns() + timeout_ns
            status = self._library.sda_emergency_stop_subset(session._handle, self.group.mask,
                deadline, records, C.byref(stats), C.byref(result), error, len(error))
            ids = lambda mask: [session.first_id + i for i in range(6) if mask & (1 << i)]
            attempted, confirmed = ids(result.attempted_mask), ids(result.confirmed_mask)
            fault = {str(session.first_id+i): int(result.fault[i]) for i in range(6) if records[i].received == 17}
            evidence = exchange_evidence(records, stats)
            evidence.update(original_record_slots=6, selected_ids=list(self.group.ids),
                            rejected_total_bytes=int(stats.rejected_total),
                            rejected_truncated=stats.rejected_total > stats.rejected_size)
            complete = (status == 0 and attempted == list(self.group.ids) and
                        confirmed == list(self.group.ids) and result.ambiguous_mask == 0 and
                        all(value == 0 for value in fault.values()))
            return {'schema': 'singularitydog.four-bus-subset-stop-result.v1',
                    'complete': complete, 'native_status': status,
                    'attempted_ids': attempted, 'confirmed_ids': confirmed,
                    'ambiguous_ids': ids(result.ambiguous_mask), 'fault_by_id': fault,
                    'message': error.value.decode('utf-8', errors='replace'),
                    'cleanup_only': True, 'active_deadline_extended': False,
                    'physical_cutoff_required': not complete,
                    'raw': evidence, 'raw_pair': (records, stats), 'poisoned': True}
        finally:
            session.busy.release()

    def close(self):
        if not self._closed:
            try:
                self._session.close()
            finally:
                _ADAPTERS.pop(self, None)
                self._closed = True
