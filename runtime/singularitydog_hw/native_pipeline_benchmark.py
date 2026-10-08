"""Three concurrent inputs -> real stateful policy -> twelve STOP proxy writes.

This diagnostic never enables motors or sends learned targets. STOP changes
state: run only on an already disabled, independently supported robot. Type17
mode measures acquisition/inference only and sends no STOP. Model/CAN/IMU setup
and warmup are outside the timed cycle; every measured cycle uses new inputs.
"""
import argparse
from array import array
from collections import namedtuple
import copy
from concurrent.futures import FIRST_COMPLETED, FIRST_EXCEPTION, Future, ThreadPoolExecutor, wait
from contextlib import ExitStack
import ctypes as C
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import struct
import sys
import threading
import time
from types import MappingProxyType

from . import native_diagnostic_transport as native
from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from . import imu
from . import policy_observer as observer
from . import policy_observer_live as live
from . import policy_observer_replay as replay
from . import policy_shadow as shadow
from . import thread_timer_slack
from . import math_thread_startup as math_threads

PERIOD_NS = 20_000_000
ABSOLUTE_MIN_START_SEPARATION_NS = 15_000_000
LIMIT_NS = 100_000_000
WORKER_STARTUP_TIMEOUT_S = .5
_TimingRecord = namedtuple('_TimingRecord','start_ns finish_ns received_ns')
_FeedbackProof = namedtuple('_FeedbackProof','images sample snapshot')
_VoltageProof = namedtuple('_VoltageProof','images values')
_ORDINARY_FEEDBACK_PUBLICATION = 'after_voltage_native_preparation'
_TRACE_SCOPE_INDEX = {scope:index for index,scope in enumerate(dual.SCOPES)}
_READ_WIRES = {(i,p):codec.read_request(i,p) for i in range(1,13)
               for p in ('position','velocity','voltage')}
_STOP_WIRES = {i:native.stop_wire(i) for i in range(1,13)}
# Exact Type2, disabled/fault-free, destination 0xfd, extended-frame flag4,
# DLC8 headers. Matching these bytes is equivalent to decoding flags/can_id.
_STOP_REPLY_HEADERS = {i:b'AT'+((((2<<24)|(i<<8)|0xfd)<<3)|4).to_bytes(4,'big')+b'\x08'
                       for i in range(1,13)}
_STOP_REPLY_HEADER_BY_WIRE = MappingProxyType({wire:_STOP_REPLY_HEADERS[mid]
                                              for mid,wire in _STOP_WIRES.items()})
_DISABLED_PAIR_STOP_BATCHES = MappingProxyType({
    scope:tuple(_STOP_WIRES[mid] for mid in ids) for scope,ids in dual.SCOPES.items()})
# An unjoined native writer must not release borrowed descriptors or locks for
# reuse in this interpreter. A failing executable process retains them to exit.
_UNRELEASED_DISABLED_PAIR_RESOURCES = []
_OUTPUT_DISPATCH_FIELDS = (
    'infer_end_ns', 'main_check_start_ns', 'main_check_end_ns',
    'front_submit_end_ns', 'rear_submit_end_ns',
    'front_worker_enter_ns', 'front_worker_check_end_ns', 'front_native_begin_ns',
    'front_first_write_ns', 'rear_worker_enter_ns', 'rear_worker_check_end_ns',
    'rear_native_begin_ns', 'rear_first_write_ns',
    'main_infer_thread_cpu_ns', 'main_submits_end_thread_cpu_ns')


def _disabled_pair_source_paths():
    """The isolated candidate's sources; no selection or hardware access."""
    root = Path(__file__).resolve().parents[1]
    names = ('singularitydog_hw/native_pipeline_benchmark.py',
             'singularitydog_hw/native_active_transport.py',
             'singularitydog_hw/native_diagnostic_transport.py',
             'singularitydog_hw/can_readonly.py',
             'experiments/native_active_transport/transport.cpp')
    return {name: root / name for name in names}


def _disabled_diagnostic_result(records, stats, *, validate_stop=True):
    """Preserve the diagnostic trace ABI and active rejected-byte count."""
    if type(stats) is native.Stats:
        diagnostic_stats = stats
    else:
        diagnostic_stats = native.Stats.from_buffer_copy(bytes(stats)[:C.sizeof(native.Stats)])
        diagnostic_stats.rejected_total = int(getattr(stats, 'rejected_total', stats.rejected_size))
    if validate_stop:
        for record in records:
            header = _STOP_REPLY_HEADER_BY_WIRE.get(bytes(record.tx))
            if header is not None and bytes(record.rx)[:7] != header:
                raise native.ExchangeError('Disabled STOP reply has fault/mode/source mismatch',
                                           records, diagnostic_stats)
    return records, diagnostic_stats


def _disabled_pair_phase_snapshot(phase):
    """Copy the fixed native phase layout without recursive-copy dispatch.

    Each generation owns new scalar/list metadata. Keep snapshots detached
    from that metadata and from callers; unfamiliar instrumentation falls
    back to the original general copy rather than dropping any field.
    """
    scalars = ('generation', 'submitted_ns', 'validated_ns', 'released_ns',
               'cancel_requested_ns')
    vectors = ('owner_started_ns', 'owner_finished_ns', 'owner_status')
    if (type(phase) is dict and set(phase) == set(scalars + vectors) and
            all(type(phase[key]) is int for key in scalars) and
            all(type(phase[key]) is list and len(phase[key]) == 2 and
                all(type(value) is int for value in phase[key]) for key in vectors)):
        return {**phase, **{key: phase[key][:] for key in vectors}}
    return copy.deepcopy(phase)


class _DisabledOnlyNativeSession:
    """Borrowed-FD adapter exposing only the diagnostic wire allowlist.

    The underlying active parser is needed by the optional pair ABI. This
    adapter offers no enable, motion, watchdog write, retry or emergency API.
    Its per-axis gain ceilings are zero as an additional native check; even
    zero-gain Type1 is rejected by the exact Python wire allowlist.
    """
    def __init__(self, library, fd, *, first_id, cancel_fd, boot_fd, boot_id,
                 request_gap_us=900):
        from . import native_active_transport as active
        if type(first_id) is not int or first_id not in (1, 7):
            raise ValueError('Exact diagnostic bus required')
        if type(request_gap_us) is not int or request_gap_us not in (880, 890, 900):
            raise ValueError('Disabled native pair request gap must be exactly 880, 890 or 900 us')
        ids = tuple(range(first_id, first_id + 6))
        self.__allowed = frozenset(
            [codec.read_request(mid, parameter) for mid in ids
             for parameter in (None, 'position', 'velocity', 'voltage')] +
            [native.stop_wire(mid) for mid in ids])
        self.__session = active.ActiveSession(library, fd, first_id=first_id,
            cancel_fd=cancel_fd, boot_fd=boot_fd, boot_id=boot_id,
            raw_lower_by_id={mid: -12.57 for mid in ids},
            raw_upper_by_id={mid: 12.57 for mid in ids},
            kp_max_by_id={mid: 0. for mid in ids},
            kd_max_by_id={mid: 0. for mid in ids}, gap_ns=request_gap_us*1000, window=3)
        self.__raw = []

    def _borrow_for_pair(self):
        return self.__session

    def exchange(self, wires, *, timeout_ns=100_000_000, before_native=None):
        wires = tuple(wires)
        if (not 1 <= len(wires) <= 12 or
                any(type(wire) is not bytes or wire not in self.__allowed for wire in wires)):
            raise ValueError('Disabled candidate accepts only exact Type0/17 reads and all-zero STOP')
        try:
            result = self.__session.exchange(wires, timeout_ns=timeout_ns,
                                             before_native=before_native)
            self.__raw.append(result)
            return _disabled_diagnostic_result(*result)
        except Exception as error:
            # Preserve the diagnostic collector's existing error/evidence ABI.
            from . import native_active_transport as active
            if isinstance(error, active.ExchangeError):
                self.__raw.append((error.records, error.stats))
                records, stats = _disabled_diagnostic_result(error.records, error.stats, validate_stop=False)
                raise native.ExchangeError(str(error), records, stats) from error
            raise

    def evidence(self):
        return [{**native.exchange_evidence(records, stats),
                 'rejected_total': int(getattr(stats, 'rejected_total', stats.rejected_size))}
                for records, stats in self.__raw]

    def close(self):
        self.__session.close()


class _DisabledNativePairCandidate:
    """Source-pinned STOP-only candidate for the explicit disabled comparison.

    The CLI keeps the existing feedback/voltage owner hook, pins current
    input/model/source provenance, and limits collection to 5/501 cycles.
    Helper/socket success cannot manufacture hardware qualification.
    Callers own and retain all FDs and process/port locks.
    """
    def __init__(self, *, library_path, library_sha256, source_sha256,
                 fd_by_scope, boot_fd_by_scope, cancel_fd, boot_id, motor_power_epoch,
                 request_gap_us=900):
        from . import native_active_transport as active
        if type(request_gap_us) is not int or request_gap_us not in (880, 890, 900):
            raise ValueError('Disabled native pair request gap must be exactly 880, 890 or 900 us')
        paths = _disabled_pair_source_paths()
        if (type(source_sha256) is not dict or set(source_sha256) != set(paths) or
                any(type(value) is not str or len(value) != 64 or
                    any(char not in '0123456789abcdef' for char in value)
                    for value in source_sha256.values())):
            raise ValueError('Exact externally pinned disabled candidate source inventory required')
        if (type(fd_by_scope) is not dict or type(boot_fd_by_scope) is not dict or
                set(fd_by_scope) != {'front', 'rear'} or
                set(boot_fd_by_scope) != {'front', 'rear'} or
                type(motor_power_epoch) is not str or not motor_power_epoch or
                motor_power_epoch.strip() != motor_power_epoch or
                not motor_power_epoch.isprintable() or len(motor_power_epoch) > 256):
            raise ValueError('Explicit independent bus FDs and current power epoch required')
        if (type(library_sha256) is not str or len(library_sha256) != 64 or
                any(char not in '0123456789abcdef' for char in library_sha256)):
            raise ValueError('Externally pinned active library SHA256 required')
        self.__pins = MappingProxyType(dict(source_sha256))
        self.__paths = MappingProxyType(paths)
        self.__boot_id = boot_id
        self.__power_epoch = motor_power_epoch
        self.__library_sha256 = library_sha256
        self.__request_gap_us = request_gap_us
        self.__phase_busy = threading.Lock()
        self.__pair = None
        self.__closed = False
        self.__placement = None
        self.__placement_attempted = False
        self.__restoration = None
        self.__joined = True
        self.__errors = []
        self.__journal = []
        self.__completed_futures = None
        self.verify_sources()
        library = active.load_library(library_path, expected_sha256=library_sha256)
        self.__library = library
        self.__cancel_fd = cancel_fd
        sessions = {}
        try:
            for scope, first in (('front', 1), ('rear', 7)):
                sessions[scope] = _DisabledOnlyNativeSession(library, fd_by_scope[scope],
                    first_id=first, cancel_fd=cancel_fd, boot_fd=boot_fd_by_scope[scope],
                    boot_id=boot_id, request_gap_us=request_gap_us)
            self.__pair = active.ActivePhasePair(
                sessions['front']._borrow_for_pair(), sessions['rear']._borrow_for_pair())
        except BaseException:
            if self.__pair is not None:self.__pair.close()
            for session in sessions.values():
                session.close()
            raise
        self.sessions = MappingProxyType(sessions)

    def future_readiness_waiter(self):
        """Setup-only optional hints from this exact pinned pair library."""
        if self.__closed:
            raise RuntimeError('Native pair candidate is closed')
        self.verify_sources()
        _require_future_notification_library(self.__library)
        from . import native_active_transport as active
        waiter = active.make_owned_waiter(self.__library, self.__cancel_fd, spin_us=500)
        if waiter.future_readiness_available is not True:
            raise ValueError('Explicit native Future notification ABI is unavailable')
        return waiter

    def verify_sources(self):
        for name, path in self.__paths.items():
            if hashlib.sha256(path.read_bytes()).hexdigest() != self.__pins[name]:
                raise ValueError('Disabled candidate source differs from its pin: ' + name)
        return True

    @staticmethod
    def _placement_verified(rows, *, restoring=False):
        if type(rows) is not dict or set(rows) != {'front', 'rear'}:
            return False
        try:
            if (any(type(row) is not dict for row in rows.values()) or
                    rows['front']['native_tid'] == rows['rear']['native_tid']):
                return False
            return all(row['native_tid'] > 0 and row['status'] == 0 and
                (row['restored'] == 1 and row['cpu_mask'] == row['original_cpu_mask'] and
                 row['timer_slack_ns'] == row['original_timer_slack_ns'] if restoring else
                 row['configured'] == 1 and row['cpu_mask'] == 15 and
                 row['timer_slack_ns'] == 1000)
                for row in rows.values())
        except (KeyError, TypeError):
            return False

    def configure_owners(self):
        self.verify_sources()
        if self.__closed or self.__placement_attempted:
            raise RuntimeError('Disabled candidate placement must be selected once before use')
        self.__placement_attempted = True
        rows = self.__pair.configure_owners((0, 1, 2, 3), timer_slack_ns=1000)
        self.__placement = copy.deepcopy(rows)
        if not self._placement_verified(rows):
            raise RuntimeError('Native diagnostic owners did not read back CPUs0..3/slack1000ns')
        if not self._coordinator_verified():
            raise RuntimeError('Native pair coordinator CPU0..3/slack1000ns placement unconfirmed')
        return copy.deepcopy(rows)

    def _coordinator_verified(self, *, restoring=False):
        row = getattr(self.__pair, 'coordinator_settings', None)
        if type(row) is not dict:return False
        try:
            return bool(row['native_tid'] > 0 and row['status'] == 0 and
                (row['restored'] == 1 and row['cpu_mask'] == row['original_cpu_mask'] and
                 row['timer_slack_ns'] == row['original_timer_slack_ns'] if restoring else
                 row['configured'] == 1 and row['cpu_mask'] == 15 and row['timer_slack_ns'] == 1000))
        except (KeyError, TypeError):return False

    def exchange_stop_proxy(self, wires_by_scope, *, deadline_ns):
        if not self.__phase_busy.acquire(blocking=False):
            raise RuntimeError('Concurrent disabled candidate coordinator use')
        try:
            self.__completed_futures = None
            return self._exchange_stop_proxy(wires_by_scope, deadline_ns=deadline_ns)
        finally:self.__phase_busy.release()

    def _exchange_stop_proxy(self, wires_by_scope, *, deadline_ns):
        """Join both owners, preserve every raw slot, never extend the deadline."""
        # Sources are verified during setup and after the finite collection.
        # Do not read files in the measured cycle.
        if (self.__closed or not self._placement_verified(self.__placement) or
                not self._coordinator_verified()):
            raise RuntimeError('Disabled candidate requires verified current native owner placement')
        if type(wires_by_scope) is not dict or set(wires_by_scope) != {'front', 'rear'}:
            raise ValueError('Paired diagnostic output must be exactly twelve all-zero STOP requests')
        batches = {scope: tuple(wires_by_scope[scope]) for scope in dual.SCOPES}
        if any(any(type(wire) is not bytes for wire in batches[scope]) or
               batches[scope] != _DISABLED_PAIR_STOP_BATCHES[scope]
               for scope in dual.SCOPES):
            raise ValueError('Paired diagnostic output must be exactly twelve all-zero STOP requests')
        now = time.monotonic_ns()
        if type(deadline_ns) is not int or not now < deadline_ns <= now + PERIOD_NS:
            raise ValueError('Disabled pair requires the original remaining 20ms absolute deadline')
        from . import native_active_transport as active
        row = {'deadline_ns': deadline_ns, 'raw_by_scope': {}, 'joined': False,
               'phase': None, 'errors': []}
        self.__journal.append(row)
        futures = {}
        converted = {}
        failure = None
        previous = self.__pair.last_completed_bus_results
        self.__joined = False
        def transform(raw_by_scope):
            # Run after both native owners joined, before their original
            # Futures publish. Keep raw arrays/stats as evidence; the public
            # results alone receive the diagnostic trace ABI normalization.
            row['raw_by_scope'].update(raw_by_scope)
            row['phase'] = _disabled_pair_phase_snapshot(self.__pair.last_phase)
            failed = any(isinstance(result, BaseException) for result in raw_by_scope.values())
            return {scope: result if isinstance(result, BaseException) else
                    _disabled_diagnostic_result(*result, validate_stop=not failed)
                    for scope, result in raw_by_scope.items()}
        try:
            futures = self.__pair.submit(batches, deadline_ns=deadline_ns,
                                         result_transform=transform)
            # Each genuine Future now holds the already validated diagnostic
            # result; no second decode/normalization is needed on the main.
            for scope, future in futures.items():
                try:
                    converted[scope] = future.result(timeout=.35)
                except BaseException as error:
                    row['raw_by_scope'].setdefault(scope, error)
                    failure = failure or error
            # Native idle means writers joined, not publication finished.
            # Keep late callback/hint failures visible and prevent re-submit
            # racing publication. The outer original 20ms clock gate still
            # includes this join; .35 remains only the existing cleanup bound.
            self.__pair.wait_published(futures)
        except BaseException as error:
            failure = failure or error
            partial = getattr(error, 'native_pair_bus_results', None)
            if partial is not None:
                row['raw_by_scope'].update(partial)
        finally:
            try:
                if failure is not None:
                    self.__pair.cancel()
                self.__pair.wait_idle()
                row['joined'] = self.__joined = True
                completed = self.__pair.last_completed_bus_results
                if completed is not None and completed is not previous:
                    for scope, raw in completed.items():
                        existing = row['raw_by_scope'].get(scope)
                        if existing is None or (isinstance(existing, BaseException) and
                                not (hasattr(existing, 'records') and hasattr(existing, 'stats'))):
                            row['raw_by_scope'][scope] = raw
                    if row['phase'] is None:
                        row['phase'] = _disabled_pair_phase_snapshot(self.__pair.last_phase)
            except BaseException as error:
                failure = failure or error
            if failure is not None:
                row['errors'].append(type(failure).__name__ + ': ' + str(failure))
                self.__errors.extend(row['errors'])
        if failure is not None:
            if isinstance(failure, active.ExchangeError):
                translated = native.ExchangeError(str(failure), failure.records, failure.stats)
                translated.native_pair_bus_results = dict(row['raw_by_scope'])
                translated.native_pair_phase = copy.deepcopy(row['phase'])
                raise translated from failure
            failure.native_pair_bus_results = dict(row['raw_by_scope'])
            failure.native_pair_phase = copy.deepcopy(row['phase'])
            raise failure
        # Retain the genuine, already joined native publication Futures. The
        # collector can apply its existing readiness/deadline checks without
        # allocating another pair of completed Futures after normalization.
        self.__completed_futures = (row['phase']['generation'], MappingProxyType(dict(futures)))
        return converted

    def last_completed_futures(self):
        if not self.__phase_busy.acquire(blocking=False):
            raise RuntimeError('Native publication unavailable during an active phase')
        try:
            saved = self.__completed_futures
            if (self.__closed or saved is None or not self.__joined or not self.__journal or
                    not self.__journal[-1]['joined'] or self.__journal[-1]['errors'] or
                    saved[0] != self.__journal[-1]['phase']['generation'] or
                    saved[0] != self.__pair.last_phase['generation'] or
                    set(saved[1]) != {'front', 'rear'} or
                    any(type(future) is not Future or not future.done() or future.cancelled()
                        for future in saved[1].values())):
                raise RuntimeError('No validated current joined native publication')
            return MappingProxyType(dict(saved[1]))
        finally:self.__phase_busy.release()

    def last_phase(self):
        return _disabled_pair_phase_snapshot(self.__pair.last_phase)

    def fd_release_safe(self):
        return self.__closed

    def evidence(self):
        if not self.__phase_busy.acquire(blocking=False):
            raise RuntimeError('Evidence unavailable while disabled pair owners are active')
        try:return self._evidence()
        finally:self.__phase_busy.release()

    def _evidence(self):
        """Return raw candidate evidence; it intentionally grants no qualification."""
        journal = []
        for row in self.__journal:
            raw = {}
            for scope, result in row['raw_by_scope'].items():
                if isinstance(result, BaseException):
                    if hasattr(result, 'records') and hasattr(result, 'stats'):
                        records, stats = result.records, result.stats
                    else:
                        raw[scope] = {'error': type(result).__name__ + ': ' + str(result)}
                        continue
                else:
                    records, stats = result
                raw[scope] = native.exchange_evidence(records, stats)
                raw[scope]['rejected_total'] = int(getattr(stats, 'rejected_total', stats.rejected_size))
            journal.append({**row, 'raw_by_scope': raw})
        try:
            source_unchanged = self.verify_sources()
        except (ValueError, OSError):
            source_unchanged = False
        same_restored_owners = bool(self.__placement is not None and
            self.__restoration is not None and all(
                self.__restoration.get(scope, {}).get('native_tid') ==
                self.__placement.get(scope, {}).get('native_tid')
                for scope in ('front', 'rear')))
        return {'schema': 'singularitydog.disabled-native-pair-candidate.v1',
            'status': 'DISABLED_CANDIDATE_EVIDENCE_NOT_ACTIVE_QUALIFICATION',
            'source_sha256': dict(self.__pins), 'library_sha256': self.__library_sha256,
            'request_gap_us': self.__request_gap_us, 'request_window': 3,
            'source_files_unchanged': source_unchanged,
            'boot_id': self.__boot_id, 'motor_power_epoch': self.__power_epoch,
            'owner_settings': copy.deepcopy(self.__placement),
            'owner_restoration': copy.deepcopy(self.__restoration),
            'coordinator': copy.deepcopy(getattr(self.__pair, 'coordinator_settings', None)),
            'coordinator_history': copy.deepcopy(getattr(self.__pair, 'coordinator_settings_history', [])),
            'all_phases_joined': self.__joined and all(row['joined'] for row in self.__journal),
            'owner_placement_verified': self._placement_verified(self.__placement),
            'owner_settings_restored': same_restored_owners and
                self._placement_verified(self.__restoration, restoring=True),
            'coordinator_placement_verified': bool(self.__placement is not None and
                any(row.get('restore') is False and row.get('status') == 0 and
                    type(row.get('coordinator')) is dict and row['coordinator'].get('configured') == 1 and
                    row['coordinator'].get('cpu_mask') == 15 and row['coordinator'].get('timer_slack_ns') == 1000
                    for row in getattr(self.__pair, 'coordinator_settings_history', []))),
            'coordinator_settings_restored': self._coordinator_verified(restoring=True),
            'prepared_owner_raw_exchanges': {scope: session.evidence()
                                            for scope, session in self.sessions.items()},
            'journal': journal, 'errors': list(self.__errors),
            'motor_enable_sent': False, 'learned_targets_sent': False,
            'output_allowed': False, 'approved_for_runtime': False,
            'active_controller_qualification': False}

    def close(self):
        if not self.__phase_busy.acquire(blocking=False):
            raise RuntimeError('Join disabled candidate coordinator before close')
        try:return self._close()
        finally:self.__phase_busy.release()

    def _close(self):
        if self.__closed:
            return
        self.__restoration = None
        try:
            self.__pair.cancel()
            self.__pair.wait_idle()  # Unjoined writers retain sessions/FD ownership.
            self.__joined = True
            # The pair owns one authoritative restoration/destruction path.
            self.__pair.close()
            if self.__placement_attempted:
                history = self.__pair.owner_settings_history
                final = history[-1] if history else {}
                rows = final.get('owners')
                if (final.get('restore') is not True or final.get('status') != 0 or
                        not self._placement_verified(rows, restoring=True)):
                    raise RuntimeError('Native diagnostic owner settings restoration unconfirmed')
                if self.__placement is not None and any(
                        rows[scope]['native_tid'] != self.__placement[scope]['native_tid']
                        for scope in rows):
                    raise RuntimeError('Native diagnostic restoration owner TIDs changed')
                self.__restoration = copy.deepcopy(rows)
                if not self._coordinator_verified(restoring=True):
                    raise RuntimeError('Native pair coordinator settings restoration unconfirmed')
        except BaseException as failure:
            self.__restoration = None
            if not getattr(self.__pair, '_closed', False):
                self.__joined = False
            self.__errors.append(type(failure).__name__ + ': ' + str(failure))
            if getattr(self.__pair, '_closed', False) is True:
                for session in self.sessions.values():session.close()
                self.__closed = True
            raise
        for session in self.sessions.values():
            session.close()
        self.__closed = True
        try:self.verify_sources()
        except BaseException as failure:
            self.__errors.append(type(failure).__name__ + ': ' + str(failure))
            raise


def _native_pair_diagnostic_plan(args, active_fk_context):
    """File-only binding for the exact best20 STOP-only comparison scope."""
    from . import policy_live_profile as profiles
    required = (args.mode == 'stop-proxy', args.supported_disabled,
        active_fk_context is not None, args.cycles in (5, 501), args.startup_cycle_allowance == 1,
        type(args.request_gap_us) is int and args.request_gap_us in (880, 890, 900),
        args.request_window == 3, args.main_thread_cpu == 4,
        args.exclude_policy_cpu_from_workers, args.timer_slack_ns == 1000,
        args.release_spin_us == 500, args.pre_cycle_policy_warmup_calls == 10,
        args.post_pin_policy_prime_calls == 10, args.setup_gc == 'before-warmup',
        args.defer_gc_during_cycles, args.single_thread_math, args.require_pinned_fast_model,
        args.record_storage == 'trace', args.output_dispatch_trace, args.inference_thread_cpu_trace,
        args.absolute_epoch_cadence, args.v3_voltage_proxy, args.v3_voltage_overlap,
        args.v3_voltage_validation_overlap, args.v3_voltage_fast_pipeline,
        args.prepare_voltage_before_feedback_publication,
        type(args.voltage_max_v) is int and args.voltage_max_v in (42, 43),
        not args.v3_voltage_pipeline, not args.acquisition_only, not args.compare_feedback,
        not args.retain_gil_trace_copy, args.native_boot_guard_artifact is None,
        args.native_boot_guard_artifact_sha256 is None,
        bool(args.provenance_mode), bool(args.power_epoch))
    if not all(required):
        raise ValueError('Native pair requires explicit supported-disabled FK STOP-proxy 5/501 and every best20 setting')
    if not args.native_pair_active_library or not args.native_pair_source_inventory:
        raise ValueError('Native pair requires externally pinned active library and source inventory')
    library_sha = profiles._hash(args.native_pair_active_library_sha256, 'native pair active library')
    inventory_sha = profiles._hash(args.native_pair_source_inventory_sha256, 'native pair source inventory')
    inventory_path = Path(args.native_pair_source_inventory).expanduser().absolute()
    inventory, _ = profiles._read_json(inventory_path, digest=inventory_sha)
    if (type(inventory) is not dict or set(inventory) != {'schema', 'source_sha256'} or
            inventory['schema'] != 'singularitydog.disabled-native-pair-sources.v1' or
            type(inventory['source_sha256']) is not dict or
            set(inventory['source_sha256']) != set(_disabled_pair_source_paths())):
        raise ValueError('Exact disabled native pair source inventory schema required')
    for name, path in _disabled_pair_source_paths().items():
        digest = profiles._hash(inventory['source_sha256'][name], 'native pair source ' + name)
        if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Native pair source differs from external inventory: ' + name)
    library_path = Path(args.native_pair_active_library).expanduser().absolute()
    if (not library_path.is_file() or library_path.is_symlink() or
            library_path.stat().st_size > 64 * 1024 * 1024 or
            hashlib.sha256(library_path.read_bytes()).hexdigest() != library_sha):
        raise ValueError('Native pair active library differs from external pin')
    plan = {'mode': 'persistent_dual_owner.v1',
        'request_gap_us': args.request_gap_us, 'request_window': 3,
        'active_library': {'path': str(library_path), 'sha256': library_sha},
        'source_inventory': {'path': str(inventory_path), 'sha256': inventory_sha},
        'source_sha256': dict(inventory['source_sha256']), 'paired_phases': 'output_stop_proxy_only',
        'prepared_feedback_voltage_owners': 'existing_python_owner_hook_on_disabled_allowlist_adapter',
        'scope': 'disabled_stop_proxy_only', 'output_allowed': False, 'approved_for_runtime': False}
    if args.native_pair_prime_before_cycles:
        plan['pre_cycle_stop_prime'] = {
            'scope': 'administrative_stop_only_before_cadence', 'phase_count': 1,
            'request_count': 12, 'deadline_budget_ns': PERIOD_NS,
            'counted_as_measured_cycle': False, 'active_controller_qualification': False}
    return plan


def _start_source_provenance(mode, power_epoch, *, accel_input_hypothesis=False,
                             native_target_fk_cache=False, native_phase_pair=False,
                             native_feedback_batch_decode=False, native_checked_policy_dispatch=False,
                             unpaired_output_future_notifications=False):
    """Optional file provenance; no power detection or output authorization.

    Legacy diagnostic invocations remain unbound. A scoped timing record must
    explicitly name its epoch and pin its own source set before opening any
    devices. The caller's epoch string is an assertion, not a sensor reading.
    """
    if (type(accel_input_hypothesis) is not bool or type(native_target_fk_cache) is not bool or
            type(native_phase_pair) is not bool or type(native_feedback_batch_decode) is not bool or
            type(native_checked_policy_dispatch) is not bool or
            type(unpaired_output_future_notifications) is not bool):
        raise ValueError('Acceleration hypothesis selection must be boolean')
    if mode is None and power_epoch is None and not accel_input_hypothesis and not native_target_fk_cache:
        return None
    from . import policy_live_profile as profiles
    allowed = ((profiles.SUPPORTED_POLICY_PROBE, profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                profiles.SUPPORTED_POLICY_PROBE_10S_AFTER_2S, profiles.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)
               if native_target_fk_cache else
               (profiles.SUPPORTED_POLICY_PROBE, profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER)
               if accel_input_hypothesis else (profiles.SUPPORTED_PRELOAD_5S,
                    profiles.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S))
    if mode not in allowed:
        raise ValueError('Explicit supported --provenance-mode is required with --power-epoch')
    if (type(power_epoch) is not str or not 0 < len(power_epoch) <= 256 or
            power_epoch.strip() != power_epoch or not power_epoch.isprintable()):
        raise ValueError('Scoped source provenance requires an explicit nonempty --power-epoch')
    selection = {'diagnostic_timing_acceptance':mode}
    if native_checked_policy_dispatch:
        selection.update(schema=profiles.SCHEMA_V3,native_checked_policy_dispatch=True)
    if unpaired_output_future_notifications:
        selection.update(schema=profiles.SCHEMA_V3,unpaired_output_future_notifications=True)
    if accel_input_hypothesis:
        selection['accel_input_hypothesis'] = True
    if native_target_fk_cache:
        selection['native_target_fk_cache'] = True
    result = {'schema':'singularitydog.diagnostic-source-provenance.v1',
            'mode':mode,'motor_power_epoch':power_epoch,
            'power_epoch_source':'explicit_operator_argument_not_hardware_detected',
            'cadence_source_sha256':profiles.cadence_source_hashes(
                selection),
            'source_files_unchanged':None,
            'output_allowed':False,'approved_for_runtime':False}
    if native_checked_policy_dispatch:
        result['native_checked_policy_dispatch'] = True
    if unpaired_output_future_notifications:
        result['unpaired_output_future_notifications'] = True
    if accel_input_hypothesis:
        result['accel_input_hypothesis'] = True
    if native_target_fk_cache:
        result['native_target_fk_cache'] = True
    if native_phase_pair:
        result['native_phase_pair'] = True
    if native_feedback_batch_decode:
        result['native_feedback_batch_decode'] = True
        result['cadence_source_sha256'] = profiles.cadence_source_hashes(
            dict(selection, schema=profiles.SCHEMA_V3, native_feedback_batch_decode=True))
    return result


def _finish_source_provenance(report, provenance):
    """Fail the diagnostic if any pinned file changed during its finite run."""
    if provenance is None:
        return
    from . import policy_live_profile as profiles
    try:
        selection = {'diagnostic_timing_acceptance':provenance['mode']}
        if provenance.get('accel_input_hypothesis') is True:
            selection['accel_input_hypothesis'] = True
        if provenance.get('native_target_fk_cache') is True:
            selection['native_target_fk_cache'] = True
        if provenance.get('native_feedback_batch_decode') is True:
            selection['schema'] = profiles.SCHEMA_V3
            selection['native_feedback_batch_decode'] = True
        if provenance.get('native_checked_policy_dispatch') is True:
            selection.update(schema=profiles.SCHEMA_V3,native_checked_policy_dispatch=True)
        if provenance.get('unpaired_output_future_notifications') is True:
            selection.update(schema=profiles.SCHEMA_V3,unpaired_output_future_notifications=True)
        current = profiles.cadence_source_hashes(selection)
        if current != provenance['cadence_source_sha256']:
            raise ValueError('Diagnostic cadence source changed during execution')
        provenance['source_files_unchanged'] = True
    except Exception as error:
        provenance['source_files_unchanged'] = False
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append(type(error).__name__+': '+str(error))


def _active_fk_diagnostic_context(args):
    """Bind an explicit file-only FK profile to the existing STOP-only arguments."""
    from . import policy_live_profile as profiles
    from . import policy_active_fk
    digest=profiles._hash(args.active_fk_profile_sha256,'active FK diagnostic profile')
    path=Path(args.active_fk_profile).expanduser().absolute()
    profiles._read_json(path,digest=digest)
    data=profiles.load_profile(path,require_approved=False)
    if not policy_active_fk.selected(data):
        raise ValueError('Diagnostic profile must explicitly select active FK')
    if (args.mode!='stop-proxy' or not args.supported_disabled or args.acquisition_only or
            args.compare_feedback or args.view_cache_manifest is not None or
            args.view_cache_manifest_sha256 is not None or args.retain_gil_trace_copy or
            args.native_boot_guard_artifact is not None or args.native_boot_guard_artifact_sha256 is not None):
        raise ValueError('Active FK diagnostic requires the unchanged supported STOP-only inference route')
    if profiles.unpaired_output_future_notifications_selected(data) is not getattr(
            args,'unpaired_output_future_notifications',False):
        raise ValueError('Diagnostic output notification selection differs from its explicit profile')
    execution=profiles.execution_settings(data)
    if getattr(args,'native_feedback_batch_decode',False) is not data.get('native_feedback_batch_decode',False):
        raise ValueError('Diagnostic feedback codec selection differs from pinned FK profile')
    if getattr(args,'native_phase_pair',False) is not data.get('native_phase_pair',False):
        raise ValueError('Diagnostic native phase pair selection differs from pinned FK profile')
    if (args.provenance_mode!=execution['diagnostic_timing_acceptance'] or
            args.power_epoch!=data['motor_power_epoch'] or
            args.request_gap_us!=data['request_gap_us'] or args.request_window!=data['request_window'] or
            args.h_hypothesis!=data['h_hypothesis'] or args.voltage_max_v!=data['voltage_max_v'] or
            args.v3_voltage_overlap is not True or args.v3_voltage_validation_overlap is not True or
            args.v3_voltage_fast_pipeline is not True or args.v3_voltage_pipeline is True or
            args.prepare_voltage_before_feedback_publication is not data.get('prepare_voltage_before_feedback_publication',False) or
            args.apply_reviewed_accel_calibration is not profiles.acceleration_calibration_selected(data)):
        raise ValueError('Active FK diagnostic choices differ from the pinned profile')
    for argument,key in (('calibration','calibration'),('mount','mount'),('gyro_bias','bias'),
                         ('native_policy_manifest','model_manifest'),('scalar_step_manifest','scalar_step_manifest')):
        _,reference=profiles._artifact(data['artifacts'][key],path.parent)
        value=getattr(args,argument)
        if not value or str(Path(value).expanduser().absolute())!=reference['path']:
            raise ValueError('Active FK diagnostic input differs: '+argument)
        data['artifacts'][key]=reference
    if (args.native_policy_manifest_sha256!=data['artifacts']['model_manifest']['sha256'] or
            args.scalar_step_manifest_sha256!=data['artifacts']['scalar_step_manifest']['sha256']):
        raise ValueError('Active FK diagnostic scalar/baseline manifest pins differ')
    bundle=Path(data['bundle_path']).expanduser()
    if not bundle.is_absolute():bundle=path.parent/bundle
    if not args.bundle or Path(args.bundle).expanduser().absolute()!=bundle.absolute():
        raise ValueError('Active FK diagnostic bundle differs')
    data['bundle_path']=str(bundle.absolute())
    hypothesis=(data['artifacts']['accel_input_hypothesis'] if profiles.accel_input_hypothesis_selected(data) else None)
    if hypothesis is not None:
        _,hypothesis=profiles._artifact(hypothesis,path.parent)
    actual=(dict(path=str(Path(args.accel_input_hypothesis).expanduser().absolute()),
                 sha256=args.accel_input_hypothesis_sha256) if args.accel_input_hypothesis else None)
    if actual!=hypothesis:
        raise ValueError('Active FK diagnostic acceleration hypothesis differs')
    from . import policy_checked_dispatch as checked
    selected_checked=checked.selected(data)
    actual_checked=(dict(path=str(Path(getattr(args,'checked_model_manifest',None)).expanduser().absolute()),
                        sha256=getattr(args,'checked_model_manifest_sha256',None)) if getattr(args,'checked_model_manifest',None) else None)
    if actual_checked != (data['artifacts']['checked_model_manifest'] if selected_checked else None):
        raise ValueError('Diagnostic checked model path/SHA differs from explicit profile selection')
    if selected_checked and (args.cycles not in (5,501) or args.native_phase_pair):
        raise ValueError('Checked diagnostic is unpaired original normal-observer5/501 only')
    proof=policy_active_fk.plan(data)
    if proof!=data['_native_target_fk_cache_provenance']:
        raise ValueError('Active FK diagnostic file-only proof changed')
    profiles._read_json(path,digest=digest)
    context={'profile':data,'reference':dict(path=str(path),sha256=digest),'proof':proof}
    calibration,_=profiles._artifact(data['artifacts']['calibration'],path.parent)
    context['observer_calibration'],context['local_branch_provenance']=\
        _active_fk_observer_calibration(context,calibration)
    return context


def _active_fk_observer_calibration(context, calibration):
    """Select one fixed local input branch from pinned files, never physical proof.

    The unapproved profile loader deliberately does not perform hardware review.
    Validate its numerical local reference here before forming an observation-only
    calibration copy. This is the active runtime's offset - sign*turns*2*pi
    formula, not a StaticBranchComparison: no Off/On or no-physical-turn statement
    is inferred. Original raw capture values and nominal calibration stay intact.
    """
    from . import policy_live_profile as profiles
    from .angle_calibration_audit import resolve_unique_numeric_branch
    data=context['profile'];base=Path(context['reference']['path']).parent
    if data.get('local_characterization')!=profiles.LOCAL_RELATIVE_SUPPORTED:
        raise ValueError('Active FK diagnostic requires a pinned local reference')
    capture,capture_ref=profiles._artifact(data['artifacts']['local_reference_capture'],base)
    ids={str(i) for i in range(1,13)}
    if (capture.get('schema')!='singularitydog.readonly-12-angle-capture.v1' or
            capture.get('status')!='RECORDED_REVIEW_REQUIRED' or capture.get('errors')!=[] or
            capture.get('approved_for_runtime') is not False or capture.get('angle_wrap_applied') is not False or
            capture.get('stop_state')!='UNVERIFIED_BY_READ_ONLY_PROTOCOL' or
            capture.get('motor_output_allowed') is not False or capture.get('boot_id')!=data['boot_id'] or
            capture.get('motor_power_epoch') not in
                (data['motor_power_epoch'],'NOT_INFERRED_FROM_JETSON_BOOT')):
        raise ValueError('Active FK diagnostic local capture epoch/status differs')
    identities=capture.get('identities',{});telemetry=capture.get('telemetry',{}).get('rows',{})
    rows=shadow.validate_calibration(calibration)
    start_bounds=data.get('start_pose_bounds')
    if (set(identities)!=ids or set(telemetry)!=ids or set(data['axes'])!=ids or
            (start_bounds is not None and (type(start_bounds) is not dict or set(start_bounds)!=ids)) or
            calibration.get('approved_for_runtime') is not False):
        raise ValueError('Active FK diagnostic needs twelve unapproved calibrated identities')
    derived=copy.deepcopy(calibration);derived_rows=shadow.validate_calibration(derived)
    raw_by_id={};q_by_id={};turns={};offsets={};capture_times={}
    for mid in sorted(ids,key=int):
        axis=data['axes'][mid];row=telemetry[mid];candidate=rows[int(mid)]
        if (identities[mid].get('mcu_uid_hex')!=axis['uid'] or calibration['identities'][mid]!=axis['uid'] or
                candidate['sign_candidate']!=axis['sign'] or
                candidate['offset_candidate_rad']!=axis['offset_rad'] or row.get('run_mode')!=0):
            raise ValueError('Active FK diagnostic local UID/sign/offset/mode differs: ID'+mid)
        samples=row.get('position_samples')
        if type(samples) is not list or len(samples)!=3:
            raise ValueError('Active FK diagnostic requires three local positions: ID'+mid)
        values=[profiles._number(s.get('rad'),'local raw ID'+mid,-1000,1000) for s in samples]
        raw=statistics.median(values)
        span=math.degrees(max(values)-min(values))
        if (span>.1 or row.get('median_position_rad')!=raw or
                type(row.get('position_span_deg')) not in (int,float) or
                not math.isfinite(row['position_span_deg']) or abs(row['position_span_deg']-span)>1e-9):
            raise ValueError('Active FK diagnostic local median/span differs: ID'+mid)
        if type(row.get('current')) not in (int,float) or row['current']!=0:
            raise ValueError('Active FK diagnostic local current is not zero: ID'+mid)
        profiles._number(row.get('voltage'),'local voltage ID'+mid,data['voltage_min_v'],data['voltage_max_v'])
        begin=identities[mid].get('request_monotonic_ns');previous=identities[mid].get('reply_monotonic_ns')
        if type(begin) is not int or type(previous) is not int or not 0<begin<=previous:
            raise ValueError('Active FK diagnostic local UID timing differs: ID'+mid)
        for sample in samples:
            begin,end=sample.get('request_monotonic_ns'),sample.get('reply_monotonic_ns')
            if (type(begin) is not int or type(end) is not int or
                    not previous<=begin<=end or end-begin>30_000_000):
                raise ValueError('Active FK diagnostic local position causality differs: ID'+mid)
            previous=end
        capture_times[mid]=previous
        index=shadow.CAN_ORDER.index(int(mid));lo,hi=shadow.LOWER[index],shadow.UPPER[index]
        lower,upper=axis['physical_lower_rad'],axis['physical_upper_rad']
        if not lo<=lower<upper<=hi:
            raise ValueError('Active FK diagnostic local interval exceeds model: ID'+mid)
        branch=resolve_unique_numeric_branch(raw,sign=axis['sign'],offset_rad=axis['offset_rad'],
            lower_rad=lower,upper_rad=upper,uncertainty_rad=profiles.LOCAL_NUMERICAL_MARGIN_RAD)
        turn=branch['turns'];q=axis['sign']*(raw-turn*2*math.pi)+axis['offset_rad']
        expected=(max(lo,q-math.radians(3)),min(hi,q+math.radians(3)))
        if (abs(lower-expected[0])>1e-12 or abs(upper-expected[1])>1e-12 or
                (start_bounds is not None and not start_bounds[mid][0]<=q<=start_bounds[mid][1])):
            raise ValueError('Active FK diagnostic local bounds differ from capture: ID'+mid)
        fixed=axis['offset_rad']-axis['sign']*turn*2*math.pi
        derived_rows[int(mid)]['offset_candidate_rad']=fixed
        raw_by_id[mid]=raw;q_by_id[mid]=q;turns[mid]=turn;offsets[mid]=fixed
    provenance=dict(schema='singularitydog.active-fk-diagnostic-local-branch.v1',
        profile=copy.deepcopy(context['reference']),capture=capture_ref,
        calibration=copy.deepcopy(data['artifacts']['calibration']),boot_id=data['boot_id'],
        motor_power_epoch=data['motor_power_epoch'],
        capture_motor_power_epoch_label=capture.get('motor_power_epoch'),
        uids_by_id=copy.deepcopy(calibration['identities']),raw_rad_by_id=raw_by_id,
        model_rad_by_id=q_by_id,reference_turns_by_id=turns,fixed_offsets_rad_by_id=offsets,
        capture_last_position_reply_ns_by_id=capture_times,
        formula='q_model = sign * raw + (nominal_offset - sign * turns * 2*pi)',
        scope='stop_only_observation_input',raw_angles_modified=False,nominal_calibration_modified=False,
        physical_branch_or_motion_proven=False,absolute_calibration_error_rad=None,
        motor_output_allowed=False,approved_for_runtime=False,active_controller_qualification=False)
    derived['diagnostic_local_reference_branch']=copy.deepcopy(provenance)
    # The nominal file is historical evidence, even when its numerical offsets
    # remain applicable. Bind only this derived observation copy to the fresh
    # validated capture; never relabel or mutate the pinned nominal artifact.
    historical_fields=('source_current_boot_id','source_current_motor_power_epoch_label',
        'source_capture_sha256','source_raw_rad_by_id','model_rad_at_source_capture_by_id',
        'current_operator_power_statement','diagnostic_branch_derivation')
    derived['diagnostic_nominal_source']={
        'calibration':copy.deepcopy(data['artifacts']['calibration']),
        'historical_fields':{key:copy.deepcopy(calibration[key])
            for key in historical_fields if key in calibration},
        'original_artifact_modified':False,'historical_qualification_reused':False,
        'physical_branch_or_motion_proven':False,'approved_for_runtime':False,
        'motor_output_allowed':False}
    for key in ('current_operator_power_statement','diagnostic_branch_derivation'):
        derived.pop(key,None)
    derived.update(source_current_boot_id=capture['boot_id'],
        source_current_motor_power_epoch_label=capture.get('motor_power_epoch'),
        source_capture_sha256=capture_ref['sha256'],source_raw_rad_by_id=copy.deepcopy(raw_by_id),
        model_rad_at_source_capture_by_id=copy.deepcopy(q_by_id),
        approved_for_runtime=False,motor_output_available=False,output_allowed=False)
    return derived,provenance


class _ActiveFKDiagnosticObserver:
    """Keep the selected branch and local bounds fixed; delegate ordinary input checks."""
    def __init__(self, run, context):
        self._run=run
        p=context['profile'];offsets=context['local_branch_provenance']['fixed_offsets_rad_by_id']
        self._capture_times=context['local_branch_provenance']['capture_last_position_reply_ns_by_id']
        self._axes=tuple((int(mid),a['sign'],offsets[mid],a['physical_lower_rad'],
            a['physical_upper_rad'])
            for mid,a in p['axes'].items())

    def __getattr__(self, name):
        return getattr(self._run,name)

    def consume(self, snapshot):
        positions={}
        for row in snapshot.get('motors',[]):
            if row.get('parameter')=='position':
                mid=row.get('motor_id')
                if mid in positions:raise observer.ObserverError('Duplicate local raw position')
                if (type(row.get('request_ns')) is not int or
                        row['request_ns']<self._capture_times.get(str(mid),math.inf)):
                    raise observer.ObserverError('Diagnostic sample predates local capture: ID'+str(mid))
                positions[mid]=row.get('value')
        if set(positions)!={i for i, *_ in self._axes}:
            raise observer.ObserverError('Missing local raw position; no branch reselection')
        for mid,sign,offset,lo,hi in self._axes:
            raw=positions[mid]
            if type(raw) not in (int,float) or not math.isfinite(raw):
                raise observer.ObserverError('Nonfinite local raw position: ID'+str(mid))
            q=sign*raw+offset
            if not lo<=q<=hi:
                raise observer.ObserverError('Outside fixed diagnostic local branch/range: ID'+str(mid))
        # Passive STOP-only observation does not enter the active output start
        # posture gate. A post-trial sample may leave that initial +/-0.5 degree
        # window while remaining inside the unchanged local physical envelope.
        return self._run.consume(snapshot)


def _finish_active_fk_diagnostic(report, context):
    if context is None:
        return
    from . import policy_live_profile as profiles
    from . import policy_active_fk
    try:
        profiles._read_json(context['reference']['path'],digest=context['reference']['sha256'])
        for key in ('calibration','mount','bias',
                    *(('accel_input_hypothesis',) if profiles.accel_input_hypothesis_selected(context['profile']) else ())):
            profiles._artifact(context['profile']['artifacts'][key],Path(context['reference']['path']).parent)
        if 'local_branch_provenance' in context:
            calibration,_=profiles._artifact(context['profile']['artifacts']['calibration'],
                Path(context['reference']['path']).parent)
            derived,branch=_active_fk_observer_calibration(context,calibration)
            if branch!=context['local_branch_provenance'] or derived!=context['observer_calibration']:
                raise ValueError('Active FK diagnostic local branch changed during execution')
        if policy_active_fk.plan(context['profile'])!=context['proof']:
            raise ValueError('Active FK diagnostic artifact/proof changed during execution')
        if context['profile'].get('native_checked_policy_dispatch',False):
            from .policy_checked_dispatch import plan as checked_plan, verify_files
            verify_files(context['profile']['_checked_model_plan'])
            if checked_plan(context['profile']) != context['profile']['_checked_model_plan']:
                raise ValueError('Checked model proof changed during diagnostic')
        report['active_fk_profile_files_unchanged']=True
    except BaseException as error:
        report['active_fk_profile_files_unchanged']=False
        report['status']='ABORTED'
        report.setdefault('errors',[]).append(type(error).__name__+': '+str(error))


def _native_record_frame(wire):
    """Decode one complete fixed-size native record without streaming state.

    A valid Type17 or STOP reply has eight data bytes, so its AT frame must
    occupy all 17 bytes. The native ABI stores exactly one 17-byte wire per
    record; incomplete or extra bytes are never accepted as telemetry.
    """
    if wire[:2]!=b'AT' or wire[6]!=8 or wire[15:]!=b'\r\n':
        raise ValueError('Malformed native record')
    encoded=int.from_bytes(wire[2:6],'big')
    return codec.Frame(encoded>>3,encoded&7,wire[7:15],wire)


class _TraceExchange(C.Structure):
    """Fixed capacity for one native exchange; allocated before measured cycles."""
    _fields_ = [('count',C.c_uint32),('records',native.Record*12),('stats',native.Stats)]


class _TraceRow:
    __slots__ = ('storage','cycle_index','metadata','phase_scopes')

    def __init__(self,storage,cycle_index,metadata,phase_scopes):
        self.storage=storage;self.cycle_index=cycle_index;self.metadata=metadata
        self.phase_scopes=phase_scopes

    def serialize(self):
        return {**self.metadata,
            **{phase:{s:self.storage.evidence(self.cycle_index,phase,s)
                       for s in scopes}
               for phase,scopes in self.phase_scopes.items()}}


_TRACE_COPY_TOKEN = object()


class _RetainedGILTraceCopy:
    """Private bounded copy experiment; same routine, with no GIL handoff.

    No library is loaded: PYFUNCTYPE wraps the already resolved ctypes.memmove
    address. Only the trace's owned Record/Stats buffers reach this callable.
    This is diagnostic storage and cannot qualify the active controller.
    """
    def __init__(self, original, prototype, function, address, token):
        self._original=original;self._prototype=prototype;self._function=function
        self._address=address;self._token=token
        self._source_path=Path(__file__).resolve()
        self._source_sha256=hashlib.sha256(self._source_path.read_bytes()).hexdigest()
        self._calls=self._bytes=0

    def _verify_abi(self):
        if (self._token is not _TRACE_COPY_TOKEN or C.memmove is not self._original or
                getattr(self._original,'_flags_',None)!=1 or
                getattr(self._original,'restype',None) is not C.c_void_p or
                tuple(getattr(self._original,'argtypes',()) or ())!=(C.c_void_p,C.c_void_p,C.c_size_t) or
                type(self._function) is not self._prototype or
                self._function._flags_!=5 or self._function.restype is not C.c_void_p or
                tuple(self._function.argtypes)!=(C.c_void_p,C.c_void_p,C.c_size_t)):
            raise ValueError('Retained-GIL trace-copy ABI/binding changed')

    def provenance(self):
        return {'schema':'singularitydog.retained-gil-trace-copy.v1',
            'scope':'disabled_stop_proxy_diagnostic_only',
            'source_path':str(self._source_path),'source_sha256':self._source_sha256,
            'prototype':'PYFUNCTYPE(c_void_p,c_void_p,c_void_p,c_size_t)',
            'default_function_flags':1,'selected_function_flags':5,
            'default_address':self._address,'selected_address':self._address,
            'same_memmove_routine':True,'retains_gil':True,
            'new_native_library_loaded':False,'guard_check_frequency_changed':False,
            'copy_scope':'owned native Record and Stats buffers only',
            'max_copy_bytes':max(C.sizeof(native.Record)*12,C.sizeof(native.Stats)),
            'completed_copy_calls':self._calls,'completed_copy_bytes':self._bytes,
            'copy_and_checks_inside_whole_iteration':True,
            'all_original_trace_proof_preserved':True,'source_files_unchanged':None,
            'active_controller_qualification':False,'approved_for_runtime':False,
            'output_allowed':False,'timing_gain_proven':False}

    def verify(self):
        self._verify_abi()
        if (type(self._address) is not int or self._address<=0 or
                C.cast(self._original,C.c_void_p).value!=self._address or
                C.cast(self._function,C.c_void_p).value!=self._address or
                hashlib.sha256(self._source_path.read_bytes()).hexdigest()!=self._source_sha256):
            raise ValueError('Retained-GIL trace-copy routine/source changed')
        result=self.provenance();result['source_files_unchanged']=True
        return result

    def __call__(self,destination,source,length):
        self._verify_abi()
        if (type(destination) is not int or destination<=0 or type(source) is not int or source<=0 or
                type(length) is not int or not 0<=length<=max(C.sizeof(native.Record)*12,C.sizeof(native.Stats))):
            raise ValueError('Invalid bounded owned trace-copy arguments')
        returned=self._function(destination,source,length)
        if type(returned) is not int or returned!=destination:
            raise ValueError('Retained-GIL trace-copy returned a different destination')
        self._calls+=1;self._bytes+=length
        return returned


def _retained_gil_trace_copy():
    """Check the exact existing routine/three-argument ABI, without copying."""
    original=C.memmove
    if (getattr(original,'_flags_',None)!=1 or
            getattr(original,'restype',None) is not C.c_void_p or
            tuple(getattr(original,'argtypes',()) or ())!=(C.c_void_p,C.c_void_p,C.c_size_t)):
        raise ValueError('Default trace-copy routine must remain CFUNCTYPE flags1')
    address=C.cast(original,C.c_void_p).value
    if type(address) is not int or address<=0:
        raise ValueError('Default trace-copy address unavailable')
    prototype=C.PYFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_size_t)
    function=prototype(address)
    backend=_RetainedGILTraceCopy(original,prototype,function,address,_TRACE_COPY_TOKEN)
    backend.verify()
    return backend


_OWNED_BUFFER_VIEW = memoryview
_OWNED_BUFFER_COPY_TOKEN = object()


class _OwnedBufferTraceCopy:
    """Explicit diagnostic-only copy of already checked owned ctypes buffers.

    Builtin memoryview byte assignment copies while holding the interpreter
    lock; no C function pointer, library, device or borrowed address is used.
    The trace retains both destination views and their ctypes allocation.
    """
    def __init__(self):
        self._view = _OWNED_BUFFER_VIEW
        self._token = _OWNED_BUFFER_COPY_TOKEN
        self._source_path = Path(__file__).resolve()
        self._source_sha256 = hashlib.sha256(self._source_path.read_bytes()).hexdigest()
        self._calls = self._bytes = 0
        self._verify_binding()

    def _verify_binding(self):
        if (self._token is not _OWNED_BUFFER_COPY_TOKEN or
                self._view is not _OWNED_BUFFER_VIEW or memoryview is not _OWNED_BUFFER_VIEW or
                type(self).copy_exchange is not _OWNED_BUFFER_COPY_FUNCTION or
                getattr(self.copy_exchange, '__func__', None) is not _OWNED_BUFFER_COPY_FUNCTION):
            raise ValueError('Owned-buffer trace-copy binding changed')

    def copy_exchange(self, destinations, records, stats, record_bytes):
        # capture() already validated source Record/Stats types and extents;
        # cached destination views belong to its verified fixed allocation.
        record_view = self._view(records).cast('B')
        stats_view = self._view(stats).cast('B')
        if record_view.nbytes != record_bytes or stats_view.nbytes != C.sizeof(native.Stats):
            raise ValueError('Owned-buffer trace-copy extent changed')
        destinations[0][:record_bytes] = record_view
        self._calls += 1; self._bytes += record_bytes
        destinations[1][:] = stats_view
        self._calls += 1; self._bytes += stats_view.nbytes

    def provenance(self):
        return {'schema': 'singularitydog.owned-buffer-trace-copy.v1',
            'scope': 'disabled_stop_proxy_diagnostic_only',
            'source_path': str(self._source_path), 'source_sha256': self._source_sha256,
            'copy_method': 'builtin_owned_byte_memoryview_assignment',
            'retains_gil': True, 'new_native_library_loaded': False,
            'copy_scope': 'owned native Record and Stats buffers only',
            'completed_copy_calls': self._calls, 'completed_copy_bytes': self._bytes,
            'copy_and_checks_inside_whole_iteration': True,
            'all_original_trace_proof_preserved': True, 'source_files_unchanged': None,
            'active_controller_qualification': False, 'approved_for_runtime': False,
            'output_allowed': False, 'timing_gain_proven': False}

    def verify(self):
        self._verify_binding()
        if hashlib.sha256(self._source_path.read_bytes()).hexdigest() != self._source_sha256:
            raise ValueError('Owned-buffer trace-copy source changed')
        result = self.provenance(); result['source_files_unchanged'] = True
        return result


_OWNED_BUFFER_COPY_FUNCTION = _OwnedBufferTraceCopy.copy_exchange




def distribution(values):
    if not values:return None
    values=sorted(values)
    return {'count':len(values),'median':statistics.median(values),
            'p95':values[math.ceil(.95*len(values))-1],
            'p99':values[math.ceil(.99*len(values))-1],'max':values[-1]}


def _steady_timing_summary(measurements, requested, startup_cycles, complete):
    """An explicit startup allowance never drops evidence or hides later misses."""
    startup=measurements[:startup_cycles]
    steady=measurements[startup_cycles:]
    all_complete=complete and len(measurements)==requested and len(steady)>0
    scheduled=[r for r in steady if 'scheduled_completion_slack_ms' in r]
    pairs=steady[1:]
    iteration_misses=sum(not r['iteration_deadline_met'] for r in steady)
    scheduled_misses=sum(r['scheduled_completion_slack_ms']<0 for r in scheduled)
    strict_misses=sum(r['actual_release_interval_ms']>20. for r in pairs)
    skipped=sum(r.get('skipped_slots_before',0) for r in steady)
    return {'startup_cycle_allowance':startup_cycles,
        'startup_cycles_recorded':len(startup),
        'startup_whole_iteration_ms':[r['whole_iteration_ms'] for r in startup],
        'startup_iteration_deadline_misses':sum(not r['iteration_deadline_met'] for r in startup),
        'startup_to_steady_interval_ms':(steady[0]['actual_release_interval_ms']
                                       if startup_cycles and steady else None),
        'steady_cycles_requested':requested-startup_cycles,
        'steady_cycles_completed':len(steady),
        'steady_whole_iteration_ms':distribution([r['whole_iteration_ms'] for r in steady]),
        'steady_iteration_deadline_misses':iteration_misses,
        'steady_processing_20ms_met':bool(all_complete and iteration_misses==0),
        'steady_scheduled_completion_slack_ms':distribution(
            [r['scheduled_completion_slack_ms'] for r in scheduled]),
        'steady_min_completion_slack_ms':min(
            (r['scheduled_completion_slack_ms'] for r in scheduled),default=None),
        'steady_scheduled_deadline_misses':scheduled_misses if scheduled else None,
        'steady_slots_skipped':skipped,
        'steady_scheduled_deadlines_met':bool(all_complete and len(scheduled)==len(steady)
                                              and scheduled_misses==0 and skipped==0),
        'steady_start_interval_ms':distribution([r['actual_release_interval_ms'] for r in pairs]),
        'steady_start_intervals_over_20ms':strict_misses,
        'strict_steady_start_interval_20ms_met':bool(all_complete and len(pairs)>0
                                                   and strict_misses==0 and skipped==0),
        'all_cycles_retained':bool(all_complete),
        'diagnostic_only':True,'full_controller_50Hz_verified':False}


def _absolute_epoch_slot(epoch_ns, previous_slot, previous_start_ns, now_ns):
    """Choose a fixed 20 ms slot without replaying missed work in a burst.

    A late wake can skip one or more slots. The 15 ms minimum spacing allows
    small scheduler jitter to settle at the next epoch while preventing an
    almost immediate second STOP transaction after a very late first start.
    """
    if now_ns<epoch_ns or (previous_slot is None)!=(previous_start_ns is None):
        raise ValueError('Invalid absolute-epoch schedule state')
    minimum_slot=0 if previous_slot is None else previous_slot+1
    elapsed_slot=(now_ns-epoch_ns)//PERIOD_NS
    if previous_start_ns is None:
        spaced_slot=0
    else:
        earliest=previous_start_ns+ABSOLUTE_MIN_START_SEPARATION_NS
        spaced_slot=max(0,(earliest-epoch_ns+PERIOD_NS-1)//PERIOD_NS)
    slot=max(minimum_slot,elapsed_slot,spaced_slot)
    return slot,epoch_ns+slot*PERIOD_NS


class _RecordTrace:
    """Bounded native buffers, with no JSON or evidence dicts in the timed cycle."""
    def __init__(self,cycles,mode,*,voltage_overlap=False,trace_copy_backend=None):
        if _TraceExchange._fields_!=[('count',C.c_uint32),('records',native.Record*12),('stats',native.Stats)]:
            raise ValueError('Native trace storage ownership/ABI changed')
        if trace_copy_backend is not None:
            if type(trace_copy_backend) not in (_RetainedGILTraceCopy, _OwnedBufferTraceCopy):
                raise ValueError('Validated diagnostic trace-copy backend required')
            trace_copy_backend.verify()
        self.trace_copy_backend=trace_copy_backend
        self.capacity_cycles=cycles
        self.phases=('acquired','voltage','output') if voltage_overlap else (
            ('acquired','output') if mode=='stop-proxy' else ('acquired',))
        self.slots_per_cycle=2*len(self.phases)
        self.slots=(_TraceExchange*(cycles*self.slots_per_cycle))()
        self.allocated_bytes=C.sizeof(self.slots)
        # ctypes gives zeroed virtual storage, but its pages may first fault
        # when capture writes them after a timed reply. Materialize all fixed
        # trace pages now, while setup remains outside the measured schedule.
        C.memset(C.addressof(self.slots),0,self.allocated_bytes)
        self.pretouched_bytes=self.allocated_bytes
        # Keep the allocation and its views alive together. All destination
        # geometry is fixed here; capture still checks the live source ABI and
        # that this allocation has not been replaced, resized or relocated.
        self._slot_owner=self.slots
        self._slot_base=C.addressof(self.slots)
        self._slot_array_type=type(self.slots)
        self._exchange_type=_TraceExchange
        self._record_type=native.Record;self._stats_type=native.Stats
        self._stats_bytes=C.sizeof(native.Stats)
        self._record_array_types=tuple(native.Record*count for count in range(13))
        self._record_copy_bytes=tuple(C.sizeof(kind) for kind in self._record_array_types)
        self._phase_slot_offsets={phase:2*index for index,phase in enumerate(self.phases)}
        self._layout=(cycles,self.phases,self.slots_per_cycle)
        slot_bytes=C.sizeof(_TraceExchange)
        records_offset=_TraceExchange.records.offset
        stats_offset=_TraceExchange.stats.offset
        self._destinations=tuple((self.slots[index],
            self._slot_base+index*slot_bytes+records_offset,
            self._slot_base+index*slot_bytes+stats_offset)
            for index in range(cycles*self.slots_per_cycle))
        self._destination_owner = self._destinations
        self._trace_copy_backend_owner = trace_copy_backend
        self._byte_destinations = None
        if type(trace_copy_backend) is _OwnedBufferTraceCopy:
            self._byte_destinations = tuple(
                (trace_copy_backend._view(slot.records).cast('B'),
                 trace_copy_backend._view(slot.stats).cast('B'))
                for slot, _, _ in self._destinations)
        self._byte_destination_owner = self._byte_destinations

    def _validate_storage(self):
        if (self.slots is not self._slot_owner or type(self.slots) is not self._slot_array_type or
                _TraceExchange is not self._exchange_type or
                native.Record is not self._record_type or native.Stats is not self._stats_type or
                (self.capacity_cycles,self.phases,self.slots_per_cycle)!=self._layout or
                C.addressof(self._slot_owner)!=self._slot_base or
                C.sizeof(self._slot_owner)!=self.allocated_bytes or
                self._byte_destinations is not self._byte_destination_owner or
                (self._byte_destinations is not None and
                 (self._destinations is not self._destination_owner or
                  self.trace_copy_backend is not self._trace_copy_backend_owner))):
            raise ValueError('Native trace storage ownership/ABI changed')

    def _slot_index(self,cycle_index,phase,scope):
        if not 0<=cycle_index<self.capacity_cycles or phase not in self.phases:
            raise ValueError('Invalid trace cycle or phase')
        if scope not in _TRACE_SCOPE_INDEX:raise ValueError('Unknown trace bus')
        return (cycle_index*self.slots_per_cycle+
                self._phase_slot_offsets[phase]+_TRACE_SCOPE_INDEX[scope])

    def _slot(self,cycle_index,phase,scope):
        self._validate_storage()
        return self._destinations[self._slot_index(cycle_index,phase,scope)][0]

    def capture(self,cycle_index,row):
        self._validate_storage()
        buffer_copy = type(self.trace_copy_backend) is _OwnedBufferTraceCopy
        if buffer_copy: self.trace_copy_backend._verify_binding()
        copy_memory=C.memmove if self.trace_copy_backend is None else self.trace_copy_backend
        scopes={phase:tuple(row[phase]) for phase in self.phases}
        for phase in self.phases:
            for scope,(records,stats) in row[phase].items():
                count=len(records)
                if (not isinstance(records,C.Array) or type(records)._type_ is not native.Record or
                        not 1<=count<=12 or type(stats) is not native.Stats):
                    raise ValueError('Invalid native trace exchange')
                # Exact native arrays have a setup-verified size. Preserve
                # compatible subclasses, but verify their actual extent before
                # accepting len() as a bounded byte count.
                record_bytes=self._record_copy_bytes[count]
                if (type(records) is not self._record_array_types[count] and
                        C.sizeof(records)!=record_bytes):
                    raise ValueError('Invalid native trace exchange extent')
                index = self._slot_index(cycle_index,phase,scope)
                slot,records_destination,stats_destination=self._destinations[index]
                if buffer_copy:
                    self.trace_copy_backend.copy_exchange(
                        self._byte_destinations[index], records, stats, record_bytes)
                else:
                    copy_memory(records_destination,C.addressof(records),record_bytes)
                    copy_memory(stats_destination,C.addressof(stats),self._stats_bytes)
                slot.count=count
        metadata={k:v for k,v in row.items() if k not in self.phases}
        return _TraceRow(self,cycle_index,metadata,scopes)

    def evidence(self,cycle_index,phase,scope):
        slot=self._slot(cycle_index,phase,scope)
        if not 1<=slot.count<=12:raise ValueError('Missing trace exchange')
        return native.exchange_evidence(slot.records[:slot.count],slot.stats)


def snapshot_from_records(records_by_bus, sample, tick_ns, *, expected_voltage_by_bus=None,
                          feedback_decoders=None):
    """Validate raw replies; one input allocation, no event JSON/copy/hash pass.

    Raw record evidence stays owned by the run, directly or in a bounded
    preallocated trace, and is serialized after it ends.
    The existing observer independently validates source order/freshness/ranges
    and hashes the assembled input before inference.
    """
    if feedback_decoders is not None:
        from . import native_active_transport as active
        if (type(feedback_decoders) is not dict or set(feedback_decoders)!=set(dual.SCOPES) or
                any(type(d) is not active.NativeFeedbackBatchDecoder or d.available is not True
                    for d in feedback_decoders.values())):
            raise ValueError('Exact two-bus native feedback decoders required')
    motors, seen = [], set()
    oldest=latest=earliest_receive=None
    composite=False;voltages={}
    for scope, records in records_by_bus.items():
        if scope not in dual.SCOPES:raise ValueError('Unknown bus')
        native_rows=(feedback_decoders[scope].decode(records,dual.SCOPES[scope][0])
                     if feedback_decoders is not None else None)
        native_values=iter(native_rows.items()) if native_rows is not None else None
        for r in records:
            if not (r.written==r.received==17 and
                    0<r.start_ns<=r.finish_ns<=r.received_ns<r.deadline_ns and r.received_ns<=tick_ns):
                raise ValueError('Incomplete/noncausal native input')
            if native_values is not None:
                (mid,parameter),(value,request_ns,received_ns)=next(native_values)
                tx_raw,rx_raw=bytes(r.tx),bytes(r.rx)
                if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus input')
                if tx_raw!=_STOP_WIRES[mid]:raise ValueError('Identity is not cycle telemetry')
                if (parameter!='feedback' or request_ns!=r.start_ns or received_ns!=r.received_ns or
                        rx_raw[:7]!=_STOP_REPLY_HEADERS[mid] or rx_raw[7:10]==b'\x00\xc4\x56'):
                    raise ValueError('Invalid STOP composite response')
                composite=True
                pairs=(('position',value.protocol_position_rad),('velocity',value.velocity_rad_s))
            else:
                tx,rx=_native_record_frame(bytes(r.tx)),_native_record_frame(bytes(r.rx))
                mid=tx.destination
                if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus input')
                if tx.kind==4:
                    composite=True
                    if (bytes(r.tx)!=_STOP_WIRES[mid] or rx.flags!=4 or
                        rx.can_id!=((2<<24)|(mid<<8)|0xfd) or len(rx.data)!=8 or
                        rx.data[:3]==b'\x00\xc4\x56'):
                        raise ValueError('Invalid STOP composite response')
                    p,v,_,_=struct.unpack('>4H',rx.data)
                    pairs=(('position',p*(2.*12.57)/65535.-12.57),('velocity',v*100./65535.-50.))
                elif tx.kind==17:
                    parameter=next((name for name in ('position','velocity','voltage')
                        if bytes(r.tx)==_READ_WIRES[mid,name]),None)
                    if parameter is None:raise ValueError('Invalid Type17 request')
                    if bytes(r.tx)!=_READ_WIRES[mid,parameter]:raise ValueError('Invalid Type17 request')
                    decoded=codec.decode_reply(rx,mid,parameter)
                    if not decoded['ok']:raise ValueError('Rejected Type17 value')
                    if parameter=='voltage':
                        if expected_voltage_by_bus is None or scope in voltages or not math.isfinite(decoded['value']):
                            raise ValueError('Unexpected/invalid voltage cadence input')
                        voltages[scope]=(mid,decoded['value'])
                        # This read is timed as part of acquisition, but the
                        # observer's policy-input age/spread covers only the 24
                        # position/velocity values and IMU. Keep those summaries
                        # equal to its independent recomputation.
                        continue
                    pairs=((parameter,decoded['value']),)
                else:raise ValueError('Identity is not cycle telemetry')
            for parameter,value in pairs:
                key=(mid,parameter)
                if key in seen:raise ValueError('Duplicate input')
                seen.add(key)
                motors.append({'motor_id':mid,'parameter':parameter,'value':value,
                    'unit':'rad' if parameter=='position' else 'rad_s',
                    'request_ns':r.start_ns,'received_ns':r.received_ns,
                    'age_upper_bound_ns':tick_ns-r.start_ns})
                oldest=r.start_ns if oldest is None else min(oldest,r.start_ns)
                latest=r.received_ns if latest is None else max(latest,r.received_ns)
                earliest_receive=(r.received_ns if earliest_receive is None else
                                  min(earliest_receive,r.received_ns))
    if seen!={(i,p) for i in range(1,13) for p in ('position','velocity')}:
        raise ValueError('Missing full twelve-axis position/velocity inputs')
    if expected_voltage_by_bus is not None and {s:mid for s,(mid,_) in voltages.items()}!=expected_voltage_by_bus:
        raise ValueError('Missing or incorrect rotating voltage input')
    a,b=sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns']
    if not (0<a<=b<=tick_ns):raise ValueError('Noncausal IMU')
    for name in ('accel_m_s2','gyro_rad_s'):
        if len(sample[name])!=3 or not all(math.isfinite(x) for x in sample[name]):
            raise ValueError('Invalid IMU vector')
    oldest=min(oldest,a);latest=max(latest,b);earliest_receive=min(earliest_receive,b)
    if tick_ns-oldest>LIMIT_NS:raise ValueError('Expired inputs')
    return {'status':'DIAGNOSTIC_READY','output_allowed':False,'blocked_reasons':[],
        'tick_ns':tick_ns,'max_age_ns':LIMIT_NS,'max_spread_ns':LIMIT_NS,'motors':motors,
        'imu':{'frame':'raw_sensor','accel_m_s2':list(sample['accel_m_s2']),
               'gyro_rad_s':list(sample['gyro_rad_s']),'read_started_ns':a,'read_finished_ns':b,
               'age_upper_bound_ns':tick_ns-a},
        'oldest_observation_age_ns':tick_ns-oldest,'acquisition_spread_ns':latest-oldest,
        'receive_spread_ns':latest-earliest_receive,
        'voltage_by_bus':{s:{'motor_id':mid,'value_v':value} for s,(mid,value) in voltages.items()},
        'source_flags':{'native_diagnostic_transport':True,'sensor_type2_candidate':composite,
            'v3_voltage_cadence_proxy':expected_voltage_by_bus is not None,
            'velocity_scale_verified':False,'sensor_internal_sample_time_verified':False,
            'stop_feedback_state_changing':composite,'fresh_identity_match_verified':True,
            'approved_for_runtime':False,'output_allowed':False}}


def timing_row(acquired, sample, output, *, release_ns, gather_end_ns,
               prepare_end_ns, infer_end_ns, cycle_end_ns):
    input_records=[r for records,_ in acquired.values() for r in records]
    first=min(sample['read_started_monotonic_ns'],*(r.start_ns for r in input_records))
    input_end=max(sample['read_finished_monotonic_ns'],*(r.received_ns for r in input_records))
    sent=[r for records,_ in output.values() for r in records]
    final_write=max((r.finish_ns for r in sent),default=0)
    final_reply=max((r.received_ns for r in sent),default=0)
    return _timing_row_from_scalars(first,input_end,final_write,final_reply,bool(sent),
        release_ns=release_ns,gather_end_ns=gather_end_ns,
        prepare_end_ns=prepare_end_ns,infer_end_ns=infer_end_ns,cycle_end_ns=cycle_end_ns)


def _timing_row_from_scalars(first,input_end,final_write,final_reply,has_output,*,release_ns,
                             gather_end_ns,prepare_end_ns,infer_end_ns,cycle_end_ns):
    return {'release_ns':release_ns,'oldest_input_start_ns':first,
        'input_latest_reply_ns':input_end,'gather_end_ns':gather_end_ns,
        'prepare_end_ns':prepare_end_ns,'infer_end_ns':infer_end_ns,
        'final_host_write_ns':final_write or None,'last_proxy_reply_ns':final_reply or None,
        'cycle_end_ns':cycle_end_ns,'acquisition_ms':(gather_end_ns-first)/1e6,
        'prepare_ms':(prepare_end_ns-gather_end_ns)/1e6,'inference_ms':(infer_end_ns-prepare_end_ns)/1e6,
        'oldest_input_to_final_host_write_ms':(final_write-first)/1e6 if has_output else None,
        'oldest_input_to_last_reply_ms':(final_reply-first)/1e6 if has_output else None,
        'whole_iteration_ms':(cycle_end_ns-release_ns)/1e6,
        'host_deadline_met':bool(has_output and final_write-first<=PERIOD_NS),
        'iteration_deadline_met':cycle_end_ns-release_ns<=PERIOD_NS,
        'host_write_is_can_wire_completion':False,'learned_targets_sent':False}


def _timing_scalars(acquired,sample,output):
    """Keep only timestamps and output presence when a trace drops raw results."""
    first=sample['read_started_monotonic_ns']
    input_end=sample['read_finished_monotonic_ns']
    for records,_ in acquired.values():
        for r in records:
            first=min(first,r.start_ns)
            input_end=max(input_end,r.received_ns)
    final_write=final_reply=0;has_output=False
    for records,_ in output.values():
        for r in records:
            has_output=True
            final_write=max(final_write,r.finish_ns)
            final_reply=max(final_reply,r.received_ns)
    return first,input_end,final_write,final_reply,has_output


def _feedback_then_voltage(exchange,scope,feedback_wires,voltage_wire,feedback_ready,
                           proof=None,clock=None,*,publish_before_native=False):
    """One bus owner publishes six replies, then makes its separate voltage read.

    A single worker performs both calls in order; there is never a second task
    racing the same session. Both calls keep their own native Stats and records.
    Publishing in before_native leaves FD/argument preparation with this owner
    before the coordinator can begin inference, without gating the voltage read.
    """
    feedback=None
    def publish_feedback():
        if proof is not None:
            proof.setdefault('feedback_published_ns_by_bus',{})[scope]=clock()
        feedback_ready.set_result(feedback)
    try:
        if proof is not None:proof['feedback_dispatch_ns_by_bus'][scope]=clock()
        feedback=exchange(scope,feedback_wires)
        if proof is not None:
            proof['feedback_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in feedback[0])
            proof['feedback_ready_ns_by_bus'][scope]=clock()
        if not publish_before_native:publish_feedback()
        if proof is not None:proof['voltage_dispatch_ns_by_bus'][scope]=clock()
        if publish_before_native:
            voltage=exchange(scope,(voltage_wire,),before_native=publish_feedback)
            if not feedback_ready.done():
                raise RuntimeError('Native voltage call omitted feedback publication')
        else:voltage=exchange(scope,(voltage_wire,))
        if proof is not None:
            proof['voltage_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in voltage[0])
        return voltage
    except BaseException as error:
        if not feedback_ready.done():
            # FD/check/preparation failures must retain the complete feedback
            # already received; the separate voltage future still fails.
            if feedback is not None:feedback_ready.set_result(feedback)
            else:feedback_ready.set_exception(error)
        raise


def _feedback_then_gated_voltage(exchange,scope,feedback_wires,voltage_wire,
                                 feedback_ready,gate,cancelled,proof,clock):
    """Keep one FD owner across feedback and a coordinator-released voltage read.

    The coordinator validates the complete feedback/IMU snapshot before opening
    the gate. A failed snapshot cancels the waiting owners without issuing a
    voltage request or a subsequent normal STOP-proxy output batch.
    """
    try:
        proof['feedback_dispatch_ns_by_bus'][scope]=clock()
        feedback=exchange(scope,feedback_wires)
        proof['feedback_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in feedback[0])
        proof['feedback_ready_ns_by_bus'][scope]=clock()
        feedback_ready.set_result(feedback)
        gate.wait()
        if cancelled.is_set():raise RuntimeError('Voltage pipeline cancelled before read')
        proof['voltage_dispatch_ns_by_bus'][scope]=clock()
        voltage=exchange(scope,(voltage_wire,))
        proof['voltage_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in voltage[0])
        return voltage
    except BaseException as error:
        if not feedback_ready.done():feedback_ready.set_exception(error)
        raise


def _readiness_poll_target(now,deadline_ns):
    """Keep a final readiness opportunity without extending the deadline."""
    remaining=deadline_ns-now
    if remaining<=400_000:
        # With >=2 ns remaining, never deliberately sleep to the deadline.
        # Halving also bounds successive tail waits as the budget runs out.
        step=min(50_000,max(1,remaining//2))
    else:
        step=200_000
    return min(deadline_ns,now+step)


def _await_owned_ready(futures,validation_future,*,phase,deadline_ns,deadline_wait=None,
                         clock=time.monotonic_ns,check=lambda:None,
                         thread_clock=time.thread_time_ns, native_readiness_waiter=None):
    """Wait only for readiness; taking/validating results stays with the owner.

    The existing native release wait releases the GIL and spins for targets at
    most 200 us apart, reducing to <=50 us in the last 400 us and then
    halving the remaining budget. It never accepts a result at the deadline.
    It does not read an FD or publish/replace a source time.
    An explicit verified active-library notifier may wake on original Future
    hints instead; hints never certify readiness, take results or change source
    times. Without either callback, use one bounded condition wait. Result
    and frame/proof validation stay with the existing owners and dispatch gate.
    """
    if set(futures)!=set(dual.SCOPES) or any(not isinstance(f,Future) for f in futures.values()):
        raise ValueError('Exact '+phase.lower()+' owner futures required')
    if validation_future is not None and not isinstance(validation_future,Future):
        raise ValueError(phase+' validation/IMU future required')
    if type(deadline_ns) is not int or deadline_ns<=0 or (deadline_wait is not None and not callable(deadline_wait)):
        raise ValueError('Absolute '+phase.lower()+' join deadline required')
    owners=tuple(futures.values())+(() if validation_future is None else (validation_future,))
    if len({id(future) for future in owners})!=len(owners):
        raise ValueError('Distinct '+phase.lower()+' owner/validation futures required')
    if native_readiness_waiter is not None:
        from . import native_active_transport as active
        if (type(native_readiness_waiter) is not active._OwnedActiveWaiter or
                any(type(future) is not Future for future in owners)):
            raise ValueError('Exact active owned notifier and original Futures required')
    notification_selected=native_readiness_waiter is not None
    owner_count=len(owners)
    ready_flags=bytearray(owner_count)
    begin=clock();cpu_begin=thread_clock();calls=0
    if type(begin) is not int or begin<=0:
        raise ValueError('Causal '+phase.lower()+' join clock required')
    last_clock=begin;ready_count=0;decision_ns=None;stage='initial'
    last_poll_before_ns=None;last_poll_wake_ns=None;last_poll_returned_ns=None
    group=None;notification_group_created=False;notification_calls=0
    try:
        while True:
            stage='owner_readiness'
            decision_ns=None
            ready_count=0
            index=0
            while index<owner_count:
                ready_flags[index]=bool(owners[index].done())
                ready_count+=ready_flags[index]
                index+=1
            # A ready error wins over an unfinished second owner; never wait on it.
            index=0
            while index<owner_count:
                future=owners[index];ready=ready_flags[index];index+=1
                if not ready:continue
                if future.cancelled():raise RuntimeError(phase+' owner future cancelled')
                error=future.exception()
                if error is not None:raise error
            stage='guard_check'
            check()
            if notification_selected and native_readiness_waiter.future_readiness_available is not True:
                raise ValueError('Explicit native Future notification ABI is unavailable')
            stage='decision_clock'
            now=clock()
            if type(now) is not int or now<last_clock:
                raise ValueError('Noncausal '+phase.lower()+' join clock')
            last_clock=now
            decision_ns=now
            if now>=deadline_ns:
                raise TimeoutError(phase+' pipeline exceeded 20 ms hard deadline at '+phase.lower()+' join')
            if ready_count==owner_count:
                if group is not None:
                    group.close();group=None
                stage='completion_clock'
                cpu_end=thread_clock();end=clock()
                if type(end) is not int or end<now or cpu_end<cpu_begin:
                    raise ValueError('Noncausal '+phase.lower()+' join completion clock')
                last_clock=end
                decision_ns=end
                if end>=deadline_ns:
                    raise TimeoutError(phase+' pipeline exceeded 20 ms hard deadline at '+phase.lower()+' join')
                proof={'mode':('native_future_notification_v1' if notification_selected else
                                'native_readiness_poll_v1' if deadline_wait is not None else 'bounded_future_wait_v1'),
                        'native_tick_max_us':200 if deadline_wait is not None and not notification_selected else None,
                        'native_tail_window_us':400 if deadline_wait is not None and not notification_selected else None,
                        'native_tail_tick_max_us':50 if deadline_wait is not None and not notification_selected else None,
                        'wait_calls':calls,'begin_ns':begin,'end_ns':end,
                        'thread_cpu_begin_ns':cpu_begin,'thread_cpu_end_ns':cpu_end,
                        'future_results_taken_only_after_ready':True}
                if notification_selected:
                    proof.update(native_future_notification_selected=True,
                        native_notification_group_created=notification_group_created,
                        native_notification_wait_calls=notification_calls,
                        original_future_count=owner_count)
                return proof
            if group is None and notification_selected:
                group=native_readiness_waiter.readiness_group(owners)
                if group is None:
                    raise ValueError('Explicit native Future notification could not bind original Futures')
                notification_group_created=True
            if group is None and deadline_wait is None:
                stage='condition_wait'
                wait(owners,timeout=(deadline_ns-now)/1e9,return_when=FIRST_EXCEPTION)
            else:
                wake=deadline_ns if group is not None else _readiness_poll_target(now,deadline_ns)
                last_poll_before_ns=now;last_poll_wake_ns=wake;last_poll_returned_ns=None
                stage='native_wait';calls+=1
                try:
                    if group is not None:
                        notification_calls+=1
                        hint=group.wait(deadline_ns)
                        if (type(hint) is not dict or set(hint)!= {'kind','actual_ns'} or
                                hint['kind'] not in ('NOTIFIED','DEADLINE') or
                                type(hint['actual_ns']) is not int or
                                hint['actual_ns']<now or
                                (hint['kind']=='NOTIFIED' and hint['actual_ns']>=deadline_ns) or
                                (hint['kind']=='DEADLINE' and hint['actual_ns']<deadline_ns)):
                            raise ValueError('Invalid native Future notification hint')
                    else:deadline_wait(wake)
                except BaseException:
                    # Cancellation can wake the native wait after an owner failed.
                    # Retain that original owner error rather than hiding it with
                    # the cancellation notification raised by the wait callback.
                    for future in owners:
                        if future.done() and not future.cancelled():
                            error=future.exception()
                            if error is not None:raise error
                    raise
                stage='native_return_clock'
                returned=clock()
                last_poll_returned_ns=returned if type(returned) is int and returned>0 else None
                if type(returned) is not int or returned<now or (group is None and returned<wake):
                    raise ValueError('Native '+phase.lower()+' readiness wait returned before requested wake')
                last_clock=returned
            if group is None and deadline_wait is None:calls+=1
    except BaseException as error:
        try:
            error.readiness_poll_failure={
                'schema':'singularitydog.readiness-poll-failure.v1',
                'phase':phase,'stage':stage,'deadline_ns':deadline_ns,
                'begin_ns':begin,'last_checked_clock_ns':last_clock,
                'last_poll_before_ns':last_poll_before_ns,
                'last_poll_wake_ns':last_poll_wake_ns,
                'last_poll_returned_ns':last_poll_returned_ns,
                'decision_ns':decision_ns if stage in ('decision_clock','completion_clock') else None,
                'wait_calls_attempted':calls,'last_ready_count':ready_count,
                'owner_count':owner_count,'native_wait_selected':deadline_wait is not None,
                'native_tick_max_us':200 if deadline_wait is not None and not notification_selected else None,
                'native_tail_window_us':400 if deadline_wait is not None and not notification_selected else None,
                'native_tail_tick_max_us':50 if deadline_wait is not None and not notification_selected else None,
                'decision_is_owner_completion_time':False,
                'source_timestamps_changed':False}
            if notification_selected:
                error.readiness_poll_failure.update(native_future_notification_selected=True,
                    native_notification_group_created=notification_group_created,
                    native_notification_wait_calls=notification_calls,original_future_count=owner_count)
        except BaseException:
            pass
        raise

    finally:
        if group is not None:
            primary=sys.exception()
            try:group.close()
            except BaseException as cleanup:
                if primary is None:raise
                primary.add_note('Native Future notification cleanup failed: '+str(cleanup))

def _await_voltage_ready(futures,validation_future,**options):
    return _await_owned_ready(futures,validation_future,phase='Voltage',**options)


def _await_output_ready(futures,**options):
    return _await_owned_ready(futures,None,phase='Proxy output',**options)


def _output_join_failure_proof(error,stage,futures,clock):
    """Failure-only observation; no result takeout or native reply inference."""
    proof={'stage':stage,'reason':type(error).__name__+': '+str(error),
           'captured_ns':None,'owner_future_states':{},
           'capture_is_deadline_decision_time':False,
           'states_read_sequentially':True,'native_reply_time_inferred':False}
    try:
        poll_failure=getattr(error,'readiness_poll_failure',None)
        if type(poll_failure) is dict:
            proof['readiness_poll_failure']=dict(poll_failure)
    except BaseException:
        proof['readiness_poll_trace_unavailable']=True
    try:
        for scope,future in futures.items():
            proof['owner_future_states'][scope]={
                'done':future.done(),'cancelled':future.cancelled()}
        captured=clock()
        if type(captured) is not int or captured<=0:
            raise ValueError('Invalid failure observation clock')
        proof['captured_ns']=captured
    except BaseException as capture_error:
        # Observability must never replace the original failure or skip cleanup.
        proof['capture_error']=type(capture_error).__name__+': '+str(capture_error)
    return proof


def _settle_failed_decoded_output_owners(futures,record,primary_error,proof,clock):
    """Retain genuine owner outcomes only during already-failed cleanup.

    Result takeout is the existing blocking owner settlement, with no added
    allowance. Its actual observation time is never the earlier deadline
    decision or Future publication time. Full raw on an exception is distinct
    from a successfully decoded original result; neither admits another output.
    """
    settlement={'schema':'singularitydog.failed-output-owner-settlement.v1',
        'cleanup_only':True,'output_allowed':False,'timing_admission_eligible':False,
        'original_deadline_ns':proof.get('output_join_deadline_ns'),
        'primary_error':proof.get('output_join_failure_proof',{}).get('reason'),
        'capture_is_deadline_decision_time':False,
        'future_publication_time_inferred':False,'native_reply_time_inferred':False,
        'owners':{}}
    if settlement['primary_error'] is None:
        try:settlement['primary_error']=type(primary_error).__name__+': '+str(primary_error)
        except BaseException:settlement['primary_error_type']=type(primary_error).__name__
    proof['output_owner_settlement']=settlement
    for scope,future in futures.items():
        owner={'outcome':None,'settled_monotonic_ns':None,
               'raw_origin':None,'raw_retained':False}
        settlement['owners'][scope]=owner
        raw=None
        try:
            value=future.result()
        except BaseException as error:
            owner['outcome']='CANCELLED' if future.cancelled() else 'EXCEPTION'
            owner['exception_type']=type(error).__name__
            try:owner['exception_message']=str(error)
            except BaseException:owner['exception_message_unavailable']=True
            if hasattr(error,'records') and hasattr(error,'stats'):
                raw=(error.records,error.stats)
                owner['raw_origin']='original_future_exception'
        else:
            owner['outcome']='SUCCESS'
            owner['raw_origin']='original_future_result'
        try:
            settled=clock()
            if type(settled) is not int or settled<=0:
                raise ValueError('Invalid original output settlement observation clock')
            owner['settled_monotonic_ns']=settled
        except BaseException as capture_error:
            # Evidence failures must not replace the primary rejection or
            # prevent the other original owner from being joined.
            owner['capture_error_type']=type(capture_error).__name__
        try:
            if owner['outcome']=='SUCCESS':raw=value[0]
            if raw is not None:
                from .private_seven_request_bridge import diagnostic_result
                record['output'][scope]=diagnostic_result(*raw)
                owner['raw_retained']=True
        except BaseException as retention_error:
            owner['raw_retention_error_type']=type(retention_error).__name__
    return primary_error


def _proxy_output_join_deadline(release_ns,oldest_ns,*,startup=False):
    """Bound completed-result collection, independently of the dispatch gate.

    The existing explicit first-cycle diagnostic allowance permits at most
    21 ms elapsed, while checked input age remains at most 20 ms. This does
    not change native exchange deadlines or the pre-dispatch 20 ms gate.
    """
    return min(release_ns+PERIOD_NS+(1_000_000 if startup else 0),
               oldest_ns+PERIOD_NS)


def _await_acquisition_ready(futures,imu_future,**options):
    if not isinstance(imu_future,Future):
        raise ValueError('Current acquisition IMU Future required')
    return _await_owned_ready(futures,imu_future,phase='Acquisition',**options)


def _collection_deadline_wait(library,cancel_fd,spin_us):
    """Set up one coordinator-owned callback inside its FD/library lifetime.

    Default/fake collectors keep their existing condition or injected wait.
    The explicit native release option uses the same native routine, deadline,
    cancellation FD and spin setting with reusable scratch buffers.
    """
    if spin_us is None:return None
    return native.make_owned_waiter(library,cancel_fd,spin_us=spin_us)


def _require_future_notification_library(library):
    """Require the loaded pair library's exact optional notifier before I/O."""
    from . import native_active_transport as active
    abi=getattr(library,'sda_future_readiness_abi',None)
    waiter=getattr(library,'sda_wait_future_ready',None)
    if (not isinstance(abi,C._CFuncPtr) or abi._flags_!=C._FUNCFLAG_CDECL or
            tuple(abi.argtypes or ())!=() or abi.restype is not C.c_uint32 or
            getattr(abi,'errcheck',None) is not None or
            not isinstance(waiter,C._CFuncPtr) or waiter._flags_!=C._FUNCFLAG_CDECL or
            tuple(waiter.argtypes or ())!=active._FUTURE_READINESS_ARGUMENT_TYPES or
            waiter.restype is not C.c_int or getattr(waiter,'errcheck',None) is not None or
            abi()!=1):
        raise ValueError('Explicit native Future notification requires the exact active ABI1')
    return True


def _notification_join_evidence(records):
    """Post-collection counters include failed joins without converting raw slots."""
    result={phase:{'attempts':0,'groups_created':0,'wait_calls':0,'failed_joins':0}
            for phase in ('acquisition','voltage','output')}
    for row in records:
        metadata=row.metadata if type(row) is _TraceRow else row
        if type(metadata) is not dict:continue
        proof=metadata.get('voltage_fast_pipeline',{})
        for phase,counts in result.items():
            failure=proof.get(phase+'_join_failure_proof')
            value=proof.get(phase+'_join_wait')
            if type(failure) is dict:
                detail=failure.get('readiness_poll_failure',failure)
                if type(detail) is dict and detail.get('native_future_notification_selected') is True:
                    value=detail
            if type(value) is not dict or value.get('native_future_notification_selected') is not True:continue
            counts['attempts']+=1
            counts['groups_created']+=int(value['native_notification_group_created'])
            counts['wait_calls']+=value['native_notification_wait_calls']
            counts['failed_joins']+=int(type(failure) is dict)
    return result


def _settle_voltage(futures,record):
    """Keep each completed voltage exchange, even if inference failed first."""
    errors=[]
    for scope,future in futures.items():
        try:record['voltage'][scope]=future.result()
        except BaseException as error:
            errors.append(error)
            record.setdefault('voltage_errors_by_bus',{})[scope]=(
                type(error).__name__+': '+str(error))
    return errors


def _verify_final_proxy_stop_records(output,clock):
    """Account for all twelve disabled-only STOP replies in the fast trace."""
    if set(output)!=set(dual.SCOPES):raise ValueError('Incomplete final proxy STOP buses')
    reply_ends={}
    for scope,ids in dual.SCOPES.items():
        records,_=output[scope]
        if len(records)!=len(ids):raise ValueError('Incomplete final proxy STOP replies')
        for mid,row in zip(ids,records):
            if not (bytes(row.tx)==_STOP_WIRES[mid] and row.written==row.received==17 and
                    0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns):
                raise ValueError('Invalid final proxy STOP record')
            reply=bytes(row.rx)
            if (len(reply)!=17 or reply[:7]!=_STOP_REPLY_HEADERS[mid] or
                    reply[15:]!=b'\r\n' or reply[7:10]==b'\x00\xc4\x56'):
                raise ValueError('Invalid final proxy STOP reply')
        reply_ends[scope]=max(row.received_ns for row in records)
    verified_at=clock()
    if any(end>verified_at for end in reply_ends.values()):
        raise ValueError('Noncausal final proxy STOP reply')
    return reply_ends,verified_at


def _retain_submitted_feedback(feedback_ready,voltage_futures,record):
    """Preserve feedback from accepted tasks after a later submit was rejected."""
    for scope in voltage_futures:
        try:record['acquired'][scope]=feedback_ready[scope].result()
        except BaseException as error:
            record.setdefault('acquired_errors_by_bus',{})[scope]=(
                type(error).__name__+': '+str(error))


def _owned_record_images(owned,count):
    """Freeze complete native input records, including every header/timestamp.

    Each bus has already finished its exchange before these buffers are read.
    Whole-buffer comparison also catches mutations outside decoded SI values;
    no frame is reparsed just to prove the completed buffer is unchanged.
    """
    if set(owned)!=set(dual.SCOPES):raise ValueError('Incomplete voltage validation buses')
    images=[]
    for scope in dual.SCOPES:
        records,_=owned[scope]
        if (not isinstance(records,C.Array) or type(records)._type_ is not native.Record or
                len(records)!=count):
            raise ValueError('Incomplete native records before voltage validation')
        images.append(bytes(records))
    return tuple(images)


def _sample_signature(sample):
    return (sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns'],
            tuple(sample['accel_m_s2']),tuple(sample['gyro_rad_s']))


def _feedback_snapshot_signature(snapshot):
    # This locally constructed snapshot contains only bounded primitive fields.
    # Tuple ownership freezes its nested numeric values without a JSON hash or
    # recursive deepcopy. Keep units/ages/flags as well as the model SI values.
    imu_value=snapshot['imu']
    return (tuple((k,v) for k,v in snapshot.items()
                  if k not in ('motors','imu','source_flags','blocked_reasons','voltage_by_bus')),
            tuple(tuple(row.items()) for row in snapshot['motors']),
            tuple((k,tuple(v) if isinstance(v,list) else v) for k,v in imu_value.items()),
            tuple(snapshot['source_flags'].items()),tuple(snapshot['blocked_reasons']),
            tuple((s,tuple(v.items())) for s,v in snapshot['voltage_by_bus'].items()))


def _validated_feedback_for_voltage(acquired,sample,tick_ns,*,feedback_decoders=None):
    """Validate feedback once, then seal evidence before worker submission."""
    images=_owned_record_images(acquired,6)
    sample_value=_sample_signature(sample)
    snapshot=snapshot_from_records({s:x[0] for s,x in acquired.items()},sample,tick_ns,
                                   expected_voltage_by_bus=None,feedback_decoders=feedback_decoders)
    snapshot['source_flags']['v3_voltage_overlap_pending_at_inference']=True
    if images!=_owned_record_images(acquired,6) or sample_value!=_sample_signature(sample):
        raise ValueError('Feedback/IMU changed during initial voltage validation')
    return snapshot,_FeedbackProof(images,sample_value,_feedback_snapshot_signature(snapshot))


def _check_feedback_proof(acquired,sample,snapshot,proof):
    if (type(proof) is not _FeedbackProof or
            proof.images!=_owned_record_images(acquired,6) or
            proof.sample!=_sample_signature(sample) or
            proof.snapshot!=_feedback_snapshot_signature(snapshot)):
        raise ValueError('Feedback/IMU changed while voltage was pending')


def _voltage_from_records(voltage,expected_voltage_by_bus,tick_ns,voltage_max_v):
    """Decode only the two new voltage replies; retain the same wire checks."""
    if set(expected_voltage_by_bus)!=set(dual.SCOPES):
        raise ValueError('Missing or incorrect rotating voltage input')
    images=_owned_record_images(voltage,1);values={}
    for scope in dual.SCOPES:
        row=voltage[scope][0][0];mid=expected_voltage_by_bus[scope]
        if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus voltage input')
        if not (row.written==row.received==17 and
                0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns and
                row.received_ns<=tick_ns):
            raise ValueError('Incomplete/noncausal native voltage input')
        if bytes(row.tx)!=_READ_WIRES[mid,'voltage']:
            raise ValueError('Missing or incorrect rotating voltage input')
        rx=_native_record_frame(bytes(row.rx))
        decoded=codec.decode_reply(rx,mid,'voltage')
        if not decoded['ok']:raise ValueError('Rejected Type17 voltage value')
        value=decoded['value']
        if not math.isfinite(value) or not 35.<=value<=voltage_max_v:
            raise ValueError(f'Voltage outside 35..{voltage_max_v:g} V before proxy STOP')
        values[scope]={'motor_id':mid,'value_v':value}
    if images!=_owned_record_images(voltage,1):
        raise ValueError('Voltage records changed during validation')
    return values,_VoltageProof(images,tuple((s,mid['motor_id'],mid['value_v'])
                                           for s,mid in values.items()))


def _check_voltage_proof(voltage,full):
    proof=full.get('_validated_voltage_proof')
    if (type(proof) is not _VoltageProof or
            proof.images!=_owned_record_images(voltage,1) or
            proof.values!=tuple((s,v['motor_id'],v['value_v'])
                                for s,v in full['voltage_by_bus'].items())):
        raise ValueError('Voltage changed before proxy STOP')


def _verify_joined_voltage_proof(acquired,voltage,sample,snapshot,full,proof,clock):
    """Verify the joined worker result without a second fourteen-record walk."""
    _check_feedback_proof(acquired,sample,snapshot,proof)
    _check_voltage_proof(voltage,full)
    now=clock()
    if (not snapshot['tick_ns']<=full['tick_ns']<=now or
            now-(snapshot['tick_ns']-snapshot['oldest_observation_age_ns'])>LIMIT_NS):
        raise ValueError('Expired feedback/voltage/IMU before proxy STOP')
    return now


def _verify_voltage_with_feedback_proof(acquired,voltage,sample,feedback_snapshot,
                                       expected_voltage_by_bus,clock,voltage_max_v,proof):
    if type(proof) is not _FeedbackProof:raise ValueError('Missing validated feedback proof')
    tick=clock()
    values,voltage_proof=_voltage_from_records(voltage,expected_voltage_by_bus,tick,voltage_max_v)
    # Preserve the complete validation-snapshot schema and actual age fields.
    # Build from the immutable seal, not live mutable snapshot descendants.
    # A concurrent mutation cannot influence this validation result even if
    # it occurs while voltage parsing releases/interleaves the Python thread.
    metadata,motors,imu_fields,flags,blocked,_=proof.snapshot
    full=dict(metadata)
    original_tick=full['tick_ns'];oldest=original_tick-full['oldest_observation_age_ns']
    motor_rows=[dict(row) for row in motors]
    for row in motor_rows:row['age_upper_bound_ns']=tick-row['request_ns']
    imu_value=dict(imu_fields)
    for key in ('accel_m_s2','gyro_rad_s'):imu_value[key]=list(imu_value[key])
    imu_value['age_upper_bound_ns']=tick-imu_value['read_started_ns']
    source_flags=dict(flags);source_flags.pop('v3_voltage_overlap_pending_at_inference',None)
    source_flags['v3_voltage_cadence_proxy']=True
    full.update(tick_ns=tick,motors=motor_rows,imu=imu_value,voltage_by_bus=values,
                source_flags=source_flags,blocked_reasons=list(blocked),
                oldest_observation_age_ns=tick-oldest,_validated_voltage_proof=voltage_proof)
    # Frozen timestamps were causal at feedback validation; both new voltage
    # replies were checked against tick above. Recheck the buffer and actual
    # oldest age here; the coordinator still walks all fourteen timestamps at
    # its final pre-STOP gate. No source timestamp is updated or backdated.
    _check_feedback_proof(acquired,sample,feedback_snapshot,proof)
    verified=clock()
    if verified<tick or tick<original_tick or verified-oldest>LIMIT_NS:
        raise ValueError('Expired/noncausal feedback/voltage/IMU validation')
    return full,verified


def _verify_voltage_after_inference(acquired,voltage,sample,feedback_snapshot,
                                    expected_voltage_by_bus,clock,voltage_max_v=42,feedback_proof=None):
    """Revalidate retained feedback and both late replies before a proxy STOP.

    This second snapshot is a validation copy, never a replacement for the
    feedback-only snapshot whose canonical hash the observer consumed.
    """
    if feedback_proof is not None:
        return _verify_voltage_with_feedback_proof(acquired,voltage,sample,feedback_snapshot,
            expected_voltage_by_bus,clock,voltage_max_v,feedback_proof)
    combined={scope:list(acquired[scope][0])+list(voltage[scope][0])
              for scope in dual.SCOPES}
    validation_tick=clock()
    full=snapshot_from_records(combined,sample,validation_tick,
                               expected_voltage_by_bus=expected_voltage_by_bus)
    fields=('motor_id','parameter','value','request_ns','received_ns')
    if ([tuple(row[key] for key in fields) for row in full['motors']] !=
            [tuple(row[key] for key in fields) for row in feedback_snapshot['motors']]):
        raise ValueError('Feedback changed while voltage was pending')
    for key in ('accel_m_s2','gyro_rad_s','read_started_ns','read_finished_ns'):
        if full['imu'][key]!=feedback_snapshot['imu'][key]:
            raise ValueError('IMU changed while voltage was pending')
    # The upper limit is explicitly selected; 42 V remains the default.
    # This disabled-motor diagnostic screen is never an output approval.
    if any(not 35. <= row['value_v'] <= voltage_max_v
           for row in full['voltage_by_bus'].values()):
        raise ValueError(f'Voltage outside 35..{voltage_max_v:g} V before proxy STOP')
    return full,clock()


def _validate_voltage_during_inference(voltage_futures,acquired,sample,
                                       feedback_snapshot,expected_voltage_by_bus,clock,voltage_max_v=42,
                                       feedback_proof=None,result_reader=None):
    """Join both bus-owned voltage reads and validate on the free IMU worker.

    Submitted only after the IMU future has completed, so the three-worker pool
    has a free slot while the two bus workers finish their own serial reads.
    The main thread still joins this result and checks freshness before STOP.
    """
    voltage={scope:(future.result() if result_reader is None else result_reader(scope,future))
             for scope,future in voltage_futures.items()}
    started=clock()
    full,finished=_verify_voltage_after_inference(
        acquired,voltage,sample,feedback_snapshot,expected_voltage_by_bus,clock,voltage_max_v,
        feedback_proof)
    return full,started,finished


def _verify_voltage_final_freshness(acquired,voltage,sample,feedback_snapshot,
                                    full,expected_voltage_by_bus,clock,voltage_max_v=42,
                                    feedback_proof=None):
    """Check the worker proof against the actual post-inference STOP gate time.

    The worker already decoded every frame and compared feedback/IMU with the
    observer snapshot. Owned records stay unchanged until trace capture. This
    final gate checks their 14 native timestamps at the actual STOP gate time,
    without repeating frame parsing or nested equality work.
    """
    now=clock()
    if feedback_proof is not None:
        _check_feedback_proof(acquired,sample,feedback_snapshot,feedback_proof)
        _check_voltage_proof(voltage,full)
    if (full.get('status')!='DIAGNOSTIC_READY' or full.get('output_allowed') is not False or
            not 0<full.get('tick_ns',0)<=now or
            feedback_snapshot.get('source_flags',{}).get('v3_voltage_overlap_pending_at_inference') is not True or
            full.get('source_flags',{}).get('v3_voltage_cadence_proxy') is not True or
            set(acquired)!=set(dual.SCOPES) or set(voltage)!=set(dual.SCOPES)):
        raise ValueError('Voltage validation proof differs before proxy STOP')
    if (len(full.get('motors',()))!=len(feedback_snapshot.get('motors',())) or
            full.get('imu',{}).get('read_started_ns')!=
                feedback_snapshot.get('imu',{}).get('read_started_ns') or
            full.get('imu',{}).get('read_finished_ns')!=
                feedback_snapshot.get('imu',{}).get('read_finished_ns')):
        raise ValueError('Voltage validation snapshot differs before proxy STOP')
    if ({scope:row['motor_id'] for scope,row in full['voltage_by_bus'].items()}!=
            expected_voltage_by_bus or
            any(not 35.<=row['value_v']<=voltage_max_v
                for row in full['voltage_by_bus'].values())):
        raise ValueError('Voltage changed before proxy STOP')
    oldest=latest=earliest_receive=None
    for scope in dual.SCOPES:
        for phase,owned,expected_count in (
                ('feedback',acquired[scope][0],6),('voltage',voltage[scope][0],1)):
            if len(owned)!=expected_count:
                raise ValueError('Incomplete '+phase+' before proxy STOP')
            for row in owned:
                if not (row.written==row.received==17 and
                        0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns and
                        row.received_ns<=now):
                    raise ValueError('Noncausal '+phase+' before proxy STOP')
                oldest=row.start_ns if oldest is None else min(oldest,row.start_ns)
                latest=row.received_ns if latest is None else max(latest,row.received_ns)
                earliest_receive=(row.received_ns if earliest_receive is None else
                                  min(earliest_receive,row.received_ns))
    imu_start=sample['read_started_monotonic_ns']
    imu_end=sample['read_finished_monotonic_ns']
    if (not 0<imu_start<=imu_end<=now or
            full['imu']['read_started_ns']!=imu_start or
            full['imu']['read_finished_ns']!=imu_end):
        raise ValueError('Noncausal or changed IMU before proxy STOP')
    oldest=min(oldest,imu_start)
    latest=max(latest,imu_end)
    earliest_receive=min(earliest_receive,imu_end)
    # Proof/image comparisons and the fourteen-record walk can be descheduled.
    # A timestamp taken before them cannot certify their completion or a later
    # dispatch. Keep all source timestamps and measure the actual return time.
    verified=clock()
    if type(verified) is not int or verified<now:
        raise ValueError('Noncausal final feedback/voltage/IMU validation clock')
    if (verified-oldest>LIMIT_NS or latest-oldest>LIMIT_NS or
            latest-earliest_receive>LIMIT_NS):
        raise ValueError('Expired feedback/voltage/IMU before proxy STOP')
    return verified


def _prestart_workers(pool, check):
    """Start all three workers with bounded, no-I/O tasks before cycle release."""
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    ready=threading.Barrier(3)
    futures=[]
    def start_worker():
        ready.wait(timeout=max(0.,until-time.monotonic()))
        return threading.get_ident()
    try:
        check()
        for _ in range(3):futures.append(pool.submit(start_worker))
        pending=set(futures);workers=set()
        while pending:
            check()
            remaining=until-time.monotonic()
            if remaining<=0:raise TimeoutError('Diagnostic worker startup deadline exceeded')
            finished,pending=wait(pending,timeout=min(.01,remaining),return_when=FIRST_COMPLETED)
            for future in finished:workers.add(future.result())
        check()
        if len(workers)!=3:raise RuntimeError('Diagnostic startup requires three distinct workers')
    except BaseException as error:
        ready.abort()
        for future in futures:future.cancel()
        if isinstance(error,threading.BrokenBarrierError):
            raise TimeoutError('Diagnostic worker startup deadline exceeded') from error
        raise


def _verify_unpinned_workers(pool, check, expected):
    """Prove the three prestarted I/O workers did not inherit a main-thread pin."""
    barrier=threading.Barrier(3)
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    def sample():
        barrier.wait(timeout=max(0.,until-time.monotonic()))
        return threading.get_native_id(), sorted(os.sched_getaffinity(0))
    futures=[]
    try:
        check()
        for _ in range(3):futures.append(pool.submit(sample))
        values=[future.result(timeout=max(0.,until-time.monotonic())) for future in futures]
        check()
        if len({tid for tid,_ in values})!=3 or any(set(mask)!=expected for _,mask in values):
            raise RuntimeError('I/O worker affinity changed during main-thread pin')
        return [{'native_tid':tid,'cpus':mask} for tid,mask in values]
    except BaseException:
        barrier.abort()
        for future in futures:future.cancel()
        raise


def _transition_worker_affinity(pool, originals, target):
    """Set or restore all three prestarted workers, identified by native TID.

    Each task waits at a barrier so one executor worker cannot perform two
    transitions while another has not run. Errors are returned as rows, leaving
    the caller able to restore every saved original mask after partial setup.
    """
    barrier=threading.Barrier(3)
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    def change():
        tid=threading.get_native_id()
        row={'native_tid':tid,'before':None,'after':None,'error':None}
        try:
            barrier.wait(timeout=max(0.,until-time.monotonic()))
            original=originals.get(tid)
            if original is None:raise RuntimeError('Unknown I/O worker TID')
            if target is not None:
                row['before']=sorted(os.sched_getaffinity(0))
                if set(row['before'])!=original:
                    raise RuntimeError('I/O worker original affinity changed before setup')
            desired=original if target is None else target
            os.sched_setaffinity(0,desired)
            row['after']=sorted(os.sched_getaffinity(0))
            if set(row['after'])!=desired:
                raise RuntimeError('I/O worker affinity readback differs')
        except BaseException as error:
            row['error']=type(error).__name__+': '+str(error)
        return row
    futures=[]
    try:
        for _ in range(3):futures.append(pool.submit(change))
        rows=[future.result(timeout=max(0.,until-time.monotonic())) for future in futures]
    except BaseException:
        barrier.abort()
        for future in futures:future.cancel()
        raise
    if len({row['native_tid'] for row in rows})!=3 or set(originals)!={row['native_tid'] for row in rows}:
        raise RuntimeError('I/O worker TID set changed during affinity transition')
    return rows


def _reused_policy_input_tensors(run):
    """Fail closed unless the prime uses all six owned CPU float buffers."""
    sizes=(3,3,3,12,12,12)
    buffers=getattr(run,'_input_buffers',None)
    tensors=getattr(run,'_input_tensors',None)
    if (type(buffers) is not tuple or type(tensors) is not tuple or
            len(buffers)!=6 or len(tensors)!=6 or
            any(type(buf) is not array or len(buf)!=size or
                tuple(tensor.shape)!=(1,size) or
                tensor.data_ptr()!=buf.buffer_info()[0]
                for buf,tensor,size in zip(buffers,tensors,sizes))):
        raise ValueError('Post-pin prime requires six owned reused CPU float input buffers')
    return tensors


def collect(sessions, imu_device, policy_observer, *, mode, cycles, check=lambda:None,
            clock=time.monotonic_ns, sleep=time.sleep, worker_initializer=None,
            record_storage='objects', main_thread_cpu=None, output_dispatch_trace=False,
            defer_gc_during_cycles=False, pre_cycle_policy_prepare=None,
            post_pin_policy_prepare=None,v3_voltage_proxy=False,
            v3_voltage_overlap=False,v3_voltage_validation_overlap=False,
            v3_voltage_pipeline=False,v3_voltage_fast_pipeline=False,
            inference_thread_cpu_trace=False,absolute_epoch_cadence=False,
            exclude_policy_cpu_from_workers=False,startup_cycle_allowance=0,deadline_wait=None,
            voltage_max_v=42,trace_copy_backend=None,
            prepare_voltage_before_feedback_publication=False,native_phase_pair_candidate=None,
            native_pair_prime_before_cycles=False,native_future_notification_joins=False,
            private_seven_request_runtime=None,native_feedback_batch_decode=False,
            native_feedback_codec_selection=None,diagnostic_bus_workers_output=False,
            unpaired_output_future_notifications=False,diagnostic_cancel_io=None):
    """Finite no-catchup benchmark, injectable transports for failure testing."""
    if (mode not in ('type17','stop-proxy') or not 1<=cycles<=3000 or
            record_storage not in ('objects','encoded','trace')):
        raise ValueError('Invalid mode or cycle budget')
    if type(voltage_max_v) not in (int,float) or voltage_max_v not in (42,43):
        raise ValueError('Voltage maximum must be explicitly 42 or 43 V')
    if (type(startup_cycle_allowance) is not int or startup_cycle_allowance not in (0,1) or
            startup_cycle_allowance and (mode!='stop-proxy' or policy_observer is None or
                                         not 2<=cycles<=501)):
        raise ValueError('Startup allowance requires one recorded startup and 1..500 STOP-proxy inference cycles')
    bounded_cycles=500+startup_cycle_allowance
    if (type(native_future_notification_joins) is not bool or
            native_future_notification_joins and native_phase_pair_candidate is None):
        raise ValueError('Native Future notification joins require an explicit verified native pair candidate')
    if (type(native_pair_prime_before_cycles) is not bool or
            native_pair_prime_before_cycles and native_phase_pair_candidate is None):
        raise ValueError('Native pair priming requires explicit disabled native pair collection')
    if native_phase_pair_candidate is not None:
        if (type(native_phase_pair_candidate) is not _DisabledNativePairCandidate or
                any(sessions.get(scope) is not native_phase_pair_candidate.sessions[scope]
                    for scope in dual.SCOPES) or mode != 'stop-proxy' or cycles not in (5, 501) or
                policy_observer is None or record_storage != 'trace' or main_thread_cpu != 4 or
                not all((output_dispatch_trace, defer_gc_during_cycles, v3_voltage_proxy,
                    v3_voltage_overlap, v3_voltage_validation_overlap, v3_voltage_fast_pipeline,
                    prepare_voltage_before_feedback_publication, inference_thread_cpu_trace,
                    absolute_epoch_cadence, exclude_policy_cpu_from_workers)) or
                startup_cycle_allowance != 1 or
                (trace_copy_backend is not None and type(trace_copy_backend) is not _OwnedBufferTraceCopy) or
                v3_voltage_pipeline or not callable(deadline_wait) or
                not callable(worker_initializer) or not callable(pre_cycle_policy_prepare) or
                not callable(post_pin_policy_prepare) or voltage_max_v not in (42, 43)):
            raise ValueError('Native pair collection requires its exact disabled 5/501 best20 setup and owned sessions')
    if private_seven_request_runtime is not None:
        from .private_seven_request_bridge import UnpairedRuntime
        if (type(private_seven_request_runtime) is not UnpairedRuntime or
                native_phase_pair_candidate is not None or native_pair_prime_before_cycles or
                native_future_notification_joins or mode!='stop-proxy' or cycles not in (5,501) or
                policy_observer is None or record_storage!='trace' or main_thread_cpu!=4 or
                startup_cycle_allowance!=1 or voltage_max_v!=42 or trace_copy_backend is not None or
                not all((v3_voltage_proxy,v3_voltage_overlap,v3_voltage_validation_overlap,
                    v3_voltage_fast_pipeline,prepare_voltage_before_feedback_publication,
                    inference_thread_cpu_trace,absolute_epoch_cadence,exclude_policy_cpu_from_workers,
                    output_dispatch_trace,defer_gc_during_cycles)) or
                not all(callable(x) for x in (worker_initializer,pre_cycle_policy_prepare,
                    post_pin_policy_prepare,deadline_wait)) or
                any(sessions.get(k) is not private_seven_request_runtime.sessions[k] for k in dual.SCOPES)):
            raise ValueError('Exact private unpaired real-model collector required')
    if (type(diagnostic_bus_workers_output) is not bool or
            type(unpaired_output_future_notifications) is not bool):
        raise ValueError('Diagnostic runtime output selections must be boolean')
    if (unpaired_output_future_notifications and not diagnostic_bus_workers_output or
            diagnostic_bus_workers_output and (private_seven_request_runtime is None or
                private_seven_request_runtime.mode!='baseline6plus1' or
                native_phase_pair_candidate is not None or not callable(diagnostic_cancel_io))):
        raise ValueError('Diagnostic BusWorkers output requires original ordinary unpaired baseline owners and cancellation')
    if type(native_feedback_batch_decode) is not bool:
        raise ValueError('Native feedback batch decode selection must be boolean')
    if native_feedback_batch_decode and not diagnostic_bus_workers_output:
        raise ValueError('Selected codec producer must exercise actual diagnostic BusWorkers output')
    feedback_decoders=None;feedback_codec_proof=None
    if type(native_feedback_batch_decode) is not bool:
        raise ValueError('Native feedback codec selection must be boolean')
    if not native_feedback_batch_decode and native_feedback_codec_selection is not None:
        raise ValueError('Inactive diagnostic feedback codec cannot carry source selection')
    if native_feedback_batch_decode:
        if (private_seven_request_runtime is None or private_seven_request_runtime.mode!='baseline6plus1' or
                native_phase_pair_candidate is not None or native_future_notification_joins):
            raise ValueError('Selected diagnostic codec requires genuine ordinary unpaired baseline6plus1 owners')
        from .unpaired_native_feedback_codec import prepare_unpaired_decoders
        feedback_decoders,feedback_codec_proof=prepare_unpaired_decoders(
            private_seven_request_runtime.native_sessions,native_feedback_codec_selection)
    native_readiness_waiter=(native_phase_pair_candidate.future_readiness_waiter()
        if native_future_notification_joins else None)
    if trace_copy_backend is not None:
        if (type(trace_copy_backend) not in (_RetainedGILTraceCopy, _OwnedBufferTraceCopy) or mode!='stop-proxy' or
                policy_observer is None or not v3_voltage_proxy or record_storage!='trace' or
                cycles>bounded_cycles):
            raise ValueError('Retained-GIL trace copy requires bounded V3 STOP-proxy inference/trace')
        if type(trace_copy_backend) is _OwnedBufferTraceCopy and native_phase_pair_candidate is None:
            raise ValueError('Owned-buffer trace copy requires explicit native pair comparison')
        trace_copy_backend.verify()
    if deadline_wait is not None and (not callable(deadline_wait) or not absolute_epoch_cadence):
        raise ValueError('Native release wait requires absolute-epoch cadence')
    if type(v3_voltage_proxy) is not bool or (v3_voltage_proxy and
            (mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles)):
        raise ValueError('V3 voltage proxy requires at most 500 STOP-proxy inference cycles')
    if (type(v3_voltage_overlap) is not bool or v3_voltage_overlap and
            (not v3_voltage_proxy or mode!='stop-proxy' or policy_observer is None or
             cycles>bounded_cycles or record_storage!='trace')):
        raise ValueError('Voltage overlap requires bounded V3 STOP-proxy inference with trace storage')
    if (type(v3_voltage_validation_overlap) is not bool or
            v3_voltage_validation_overlap and not v3_voltage_overlap):
        raise ValueError('Voltage validation overlap requires voltage overlap')
    if (type(v3_voltage_pipeline) is not bool or v3_voltage_pipeline and not (
            v3_voltage_proxy and v3_voltage_overlap and v3_voltage_validation_overlap and
            mode=='stop-proxy' and policy_observer is not None and
            record_storage=='trace' and cycles<=bounded_cycles)):
        raise ValueError('Voltage pipeline requires bounded V3 STOP-proxy trace with voltage and validation overlap')
    if (type(v3_voltage_fast_pipeline) is not bool or v3_voltage_fast_pipeline and not (
            v3_voltage_proxy and v3_voltage_overlap and v3_voltage_validation_overlap and
            mode=='stop-proxy' and policy_observer is not None and
            record_storage=='trace' and cycles<=bounded_cycles) or
            v3_voltage_fast_pipeline and v3_voltage_pipeline):
        raise ValueError('Fast voltage pipeline requires bounded V3 STOP-proxy trace and excludes gated pipeline')
    if (type(prepare_voltage_before_feedback_publication) is not bool or
            prepare_voltage_before_feedback_publication and
            (not v3_voltage_fast_pipeline or
             (trace_copy_backend is not None and type(trace_copy_backend) is not _OwnedBufferTraceCopy))):
        raise ValueError('Prepared publication evidence requires the unchanged fast voltage STOP-proxy path')
    pipeline_key=('voltage_pipeline' if v3_voltage_pipeline else
                  'voltage_fast_pipeline' if v3_voltage_fast_pipeline else None)
    native_overlap_wait=v3_voltage_overlap and deadline_wait is not None
    if (type(inference_thread_cpu_trace) is not bool or
            inference_thread_cpu_trace and not v3_voltage_proxy):
        raise ValueError('Inference thread CPU trace requires bounded 26-request STOP-proxy inference')
    if (type(absolute_epoch_cadence) is not bool or absolute_epoch_cadence and
            (mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles)):
        raise ValueError('Absolute-epoch cadence requires at most 500 STOP-proxy inference cycles')
    if main_thread_cpu is not None and (type(main_thread_cpu) is not int or main_thread_cpu<0):
        raise ValueError('Invalid main-thread CPU')
    if (type(exclude_policy_cpu_from_workers) is not bool or
            exclude_policy_cpu_from_workers and not (
                v3_voltage_proxy and main_thread_cpu is not None and mode=='stop-proxy' and
                policy_observer is not None and cycles<=bounded_cycles)):
        raise ValueError('I/O worker CPU exclusion requires at most 500 V3 STOP-proxy cycles and a policy CPU pin')
    if pre_cycle_policy_prepare is not None and (not callable(pre_cycle_policy_prepare) or
            mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles):
        raise ValueError('Pre-cycle policy warmup requires at most 500 STOP-proxy cycles with policy inference')
    if post_pin_policy_prepare is not None and (
            not callable(post_pin_policy_prepare) or pre_cycle_policy_prepare is None or
            main_thread_cpu is None or mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles):
        raise ValueError('Post-pin policy priming requires pre-cycle warmup, CPU pin and at most 500 STOP-proxy cycles')
    if (type(output_dispatch_trace) is not bool or
            output_dispatch_trace and (mode!='stop-proxy' or policy_observer is None)):
        raise ValueError('Output dispatch trace requires STOP proxy with policy inference')
    if (type(defer_gc_during_cycles) is not bool or
            defer_gc_during_cycles and (mode!='stop-proxy' or policy_observer is None or
                                        cycles>bounded_cycles or record_storage!='trace' or
                                        not output_dispatch_trace)):
        raise ValueError('GC deferral requires at most 500 STOP-proxy cycles, trace storage and output dispatch trace')
    records=[];measurements=[];errors=[]
    dispatch_stride=len(_OUTPUT_DISPATCH_FIELDS)
    # Allocate storage before the measured loop; serialize it afterward.
    dispatch_values=(array('Q',[0])*(cycles*dispatch_stride)
                     if output_dispatch_trace else None)
    inference_cpu_values=(array('Q',[0])*(cycles*2)
                          if inference_thread_cpu_trace else None)
    gc_capacity=max(64,cycles*8) if output_dispatch_trace else 0
    if output_dispatch_trace:
        gc_times=array('Q',[0])*gc_capacity
        gc_tids=array('Q',[0])*gc_capacity
        gc_cycles=array('I',[0])*gc_capacity
        gc_generations=array('b',[0])*gc_capacity
        gc_phases=array('b',[0])*gc_capacity
    gc_count=gc_overflow=gc_errors=active_cycle=0
    gc_probe_installed=False
    gc_restore_required=False
    gc_state={'mode':'defer_automatic_during_cycles','before_enabled':None,
              'during_enabled':None,'after_enabled':None,'before_threshold':None,
              'after_threshold':None,'restored':None,'restore_attempts':0,
              'restore_errors':[]} if output_dispatch_trace else None
    def gc_probe(phase,info):
        nonlocal gc_count,gc_overflow,gc_errors
        if gc_count==gc_capacity:
            gc_overflow+=1
            return
        try:
            index=gc_count
            gc_times[index]=clock()
            gc_tids[index]=threading.get_native_id()
            gc_cycles[index]=active_cycle
            gc_generations[index]=info['generation']
            gc_phases[index]=0 if phase=='start' else 1
            gc_count=index+1
        except BaseException:
            # This diagnostic hook must never alter the collector's behavior.
            gc_errors+=1
    startup={'begin_ns':clock(),'end_ns':None,'duration_ms':None,'worker_count':3,'complete':False}
    last_imu=0;previous_release=None;previous_slot=None;cadence_epoch=None
    pool=None;workers_ready=False;storage_failure=None;trace=None
    diagnostic_output=None;diagnostic_output_proof=None;diagnostic_output_waiter=None
    policy_armed_before_cycles=False
    pair_prime = ({'schema': 'singularitydog.disabled-native-pair-prime.v1',
        'scope': 'administrative_stop_only_before_cadence', 'phase_count': 1,
        'request_count': 12, 'deadline_budget_ns': PERIOD_NS,
        'counted_as_measured_cycle': False, 'active_controller_qualification': False,
        'motor_enable_sent': False, 'learned_targets_sent': False, 'output_allowed': False,
        'attempted': False, 'complete': False, 'begin_ns': None, 'deadline_ns': None,
        'end_ns': None, 'errors': []} if native_pair_prime_before_cycles else None)
    original_affinity=None;original_worker_masks=None;worker_restore_required=False
    affinity={'requested_cpu':main_thread_cpu,'before':None,'during':None,
              'worker_masks_after_pin':None,'restored':None}
    worker_affinity={'enabled':exclude_policy_cpu_from_workers,
                     'excluded_cpu':main_thread_cpu if exclude_policy_cpu_from_workers else None,
                     'target_mask':None,'workers_before':None,'workers_during':None,
                     'workers_after':None,'restored':None,'restore_errors':[]}
    def settle_voltage(futures,record):
        if private_seven_request_runtime is not None and private_seven_request_runtime.mode=='split7':
            return private_seven_request_runtime.settle_voltage(futures,record)
        return _settle_voltage(futures,record)
    def read_imu():
        end=clock()+20_000_000
        while clock()<end:
            check();sample=imu_device.read_sample()
            if sample is not None:return sample
            sleep(.0005)
        raise TimeoutError('No new IMU within20ms')
    def exchange(scope,wires,dispatch_base=None,*,before_native=None):
        if dispatch_base is not None:
            dispatch_values[dispatch_base+(5 if scope=='front' else 9)]=clock()
        check()
        if dispatch_base is not None:
            dispatch_values[dispatch_base+(6 if scope=='front' else 10)]=clock()
        try:
            if before_native is not None:
                return sessions[scope].exchange(wires,before_native=before_native)
            return sessions[scope].exchange(wires)
        except native.ExchangeError as error:
            records.append({'failure_scope':scope,'native_failure':error})
            raise
    wires={scope:([native.stop_wire(i) for i in ids] if mode=='stop-proxy' else
        [codec.read_request(i,p) for p in ('position','velocity') for i in ids])
        for scope,ids in dual.SCOPES.items()}
    voltage_wires=({scope:tuple(tuple(wires[scope])+(_READ_WIRES[ids[phase],'voltage'],)
                                  for phase in range(6))
                    for scope,ids in dual.SCOPES.items()} if v3_voltage_proxy else None)
    try:
        if record_storage=='trace':trace=_RecordTrace(cycles,mode,
                    voltage_overlap=v3_voltage_overlap,trace_copy_backend=trace_copy_backend)
        pool_options={'max_workers':3,'thread_name_prefix':'native-bench'}
        if worker_initializer is not None:pool_options['initializer']=worker_initializer
        pool=ThreadPoolExecutor(**pool_options)
        _prestart_workers(pool,check)
        if private_seven_request_runtime is not None:
            private_seven_request_runtime.start_workers(pool,worker_initializer)
        if diagnostic_bus_workers_output:
            from . import native_active_transport as active
            from .diagnostic_runtime_output import make_borrowed_output
            diagnostic_output_waiter=active.make_owned_waiter(
                private_seven_request_runtime.native_sessions['front'].lib,
                private_seven_request_runtime.cancel_fd,spin_us=500)
            if unpaired_output_future_notifications and not diagnostic_output_waiter.future_readiness_available:
                raise ValueError('Selected diagnostic output notification needs authenticated full native readiness ABI')
            diagnostic_output=make_borrowed_output(pool,private_seven_request_runtime.native_sessions,
                diagnostic_cancel_io,clock=clock,native_feedback_batch_decode=native_feedback_batch_decode,
                native_feedback_codec_selection=native_feedback_codec_selection,
                unpaired_output_future_notifications=unpaired_output_future_notifications,
                notification_waiter=diagnostic_output_waiter)
        workers_ready=True
        startup['end_ns']=clock();startup['complete']=True
        # Torch may create native helper threads on its first policy call. Warm
        # them while the caller still has the full CPU mask; a thread created
        # after pinning the caller would inherit the single-CPU mask.
        if pre_cycle_policy_prepare is not None:
            check()
            pre_cycle_policy_prepare()
            check()
        if main_thread_cpu is not None:
            if not hasattr(os,'sched_getaffinity') or not hasattr(os,'sched_setaffinity'):
                raise RuntimeError('Main-thread affinity is unavailable')
            original_affinity=set(os.sched_getaffinity(0))
            affinity['before']=sorted(original_affinity)
            if main_thread_cpu not in original_affinity or len(original_affinity)<2:
                raise ValueError('Main-thread CPU unavailable or workers already pinned')
            if exclude_policy_cpu_from_workers and len(original_affinity-{main_thread_cpu})<3:
                raise ValueError('I/O worker CPU exclusion requires at least three other available CPUs')
            os.sched_setaffinity(0,{main_thread_cpu})
            affinity['during']=sorted(os.sched_getaffinity(0))
            if affinity['during']!=[main_thread_cpu]:
                raise RuntimeError('Main-thread affinity was not applied')
            affinity['worker_masks_after_pin']=_verify_unpinned_workers(pool,check,original_affinity)
            if exclude_policy_cpu_from_workers:
                original_worker_masks={row['native_tid']:set(row['cpus'])
                                       for row in affinity['worker_masks_after_pin']}
                worker_affinity['workers_before']=affinity['worker_masks_after_pin']
                target=original_affinity-{main_thread_cpu}
                worker_affinity['target_mask']=sorted(target)
                worker_restore_required=True
                check()
                worker_affinity['workers_during']=_transition_worker_affinity(
                    pool,original_worker_masks,target)
                if any(row['error'] is not None for row in worker_affinity['workers_during']):
                    raise RuntimeError('I/O worker affinity setup failed: '+str(
                        [row['error'] for row in worker_affinity['workers_during'] if row['error']]))
                check()
        if private_seven_request_runtime is not None:
            private_seven_request_runtime.configure_owners(original_affinity,main_thread_cpu)
            private_seven_request_runtime.verify_sources()
        if post_pin_policy_prepare is not None:
            check()
            post_pin_policy_prepare()
            check()
        if native_phase_pair_candidate is not None:
            check()
            native_phase_pair_candidate.configure_owners()
            check()
        if private_seven_request_runtime is not None:
            # Same administrative STOP prime for BOTH genuine-unpaired branches.
            # Its original20ms/raw replies stay separate from measured cycles.
            prime_begin=clock();private_seven_request_runtime.begin_cycle(prime_begin+PERIOD_NS)
            prime_futures={scope:pool.submit(exchange,scope,w) for scope,w in wires.items()}
            prime_output={scope:f.result() for scope,f in prime_futures.items()}
            prime_ends,prime_validated=_verify_final_proxy_stop_records(prime_output,clock)
            private_seven_request_runtime.prime={'begin_ns':prime_begin,'end_ns':clock(),
                'deadline_ns':prime_begin+PERIOD_NS,'reply_end_ns_by_bus':prime_ends,
                'validated_ns':prime_validated,'counted_as_measured_cycle':False,
                'request_count':12,'raw':{scope:native.exchange_evidence(*r) for scope,r in prime_output.items()}}
            private_seven_request_runtime.deadline_ns=None
        # Measured diagnostic ticks accept the real acquisition-completion
        # timestamp as their tick. Arm their schedule after all policy setup,
        # before the first timed release; all sensor/output cycles remain
        # timed and subject to the same deadline checks.
        if (not native_pair_prime_before_cycles and policy_observer is not None and
                getattr(policy_observer,'_measured_diagnostic_ticks',False) is True):
            check()
            policy_observer.arm_run(clock())
            check()
            policy_armed_before_cycles=True
        if output_dispatch_trace:
            gc.callbacks.append(gc_probe)
            gc_probe_installed=True
        if defer_gc_during_cycles:
            gc_state['before_enabled']=gc.isenabled()
            gc_state['before_threshold']=tuple(gc.get_threshold())
            if not gc_state['before_enabled']:
                raise RuntimeError('Automatic GC is already disabled before the diagnostic')
            gc_restore_required=True
            gc.disable()
            gc_state['during_enabled']=gc.isenabled()
            if gc_state['during_enabled']:
                raise RuntimeError('Automatic GC deferral was not applied')
        if pair_prime is not None:
            # Administrative STOP only: retain its raw phase separately, with
            # its own unchanged 20 ms deadline, before arming any cadence.
            try:
                check()
                native_phase_pair_candidate.verify_sources()
                pair_prime['attempted'] = True
                pair_prime['begin_ns'] = clock()
                pair_prime['deadline_ns'] = pair_prime['begin_ns'] + PERIOD_NS
                output = native_phase_pair_candidate.exchange_stop_proxy(wires,
                    deadline_ns=pair_prime['deadline_ns'])
                reply_ends, verified_at = _verify_final_proxy_stop_records(output, clock)
                pair_prime.update(reply_end_ns_by_bus=reply_ends,
                                  validated_ns=verified_at, complete=True)
                check()
            except BaseException as error:
                pair_prime['complete'] = False
                pair_prime['errors'].append(type(error).__name__ + ': ' + str(error))
                raise
            finally:
                pair_prime['end_ns'] = clock()
            if getattr(policy_observer, '_measured_diagnostic_ticks', False) is True:
                check()
                policy_observer.arm_run(clock())
                check()
                policy_armed_before_cycles = True
        # Exclude both selected preparation and optional CPU pin verification
        # from the finite measured schedule.
        release=(clock() if policy_armed_before_cycles or pre_cycle_policy_prepare is not None or main_thread_cpu is not None or
                 post_pin_policy_prepare is not None
                 else startup['end_ns'])
        if absolute_epoch_cadence:cadence_epoch=release
        deadline=release+int(cycles*.12*1e9)+2_000_000_000
        for cycle in range(cycles):
            if output_dispatch_trace:active_cycle=cycle+1
            check()
            if clock()>deadline:raise TimeoutError('Finite overall budget exhausted')
            wait_enter=clock();wait_return=wait_enter;wait_calls=0
            if absolute_epoch_cadence:
                slot,release=_absolute_epoch_slot(cadence_epoch,previous_slot,previous_release,clock())
                while clock()<release:
                    check()
                    if deadline_wait is None:sleep(max(0,release-clock())/1e9)
                    else:deadline_wait(release)
                    wait_return=clock();wait_calls+=1
                check()
            elif clock()<release:sleep((release-clock())/1e9)
            actual_release=clock()
            if absolute_epoch_cadence:
                # A sleep can wake a full slot (or more) late. Attribute the
                # work to its actual slot; never run its missed predecessors.
                slot,release=_absolute_epoch_slot(cadence_epoch,previous_slot,
                                                  previous_release,actual_release)
                skipped=slot if previous_slot is None else slot-previous_slot-1
            if private_seven_request_runtime is not None:
                private_seven_request_runtime.begin_cycle(actual_release+PERIOD_NS)
            acquisition_wires=({scope:voltage_wires[scope][cycle%6]
                                for scope in dual.SCOPES} if v3_voltage_proxy and not v3_voltage_overlap
                               else wires)
            voltage_futures={}
            voltage_gate=voltage_cancelled=None
            if v3_voltage_overlap:
                # Record before dispatch so a partially completed voltage read
                # remains attributable to this cycle on every failure path.
                record={'cycle':cycle+1,'acquired':{},'voltage':{},'imu':None,'output':{},
                        'voltage_overlap':{'status':'PENDING_AT_INFERENCE',
                                           'output_allowed':False}}
                if pipeline_key is None:
                    record['voltage_overlap']['feedback_publication']=_ORDINARY_FEEDBACK_PUBLICATION
                if pipeline_key is not None:
                    if v3_voltage_pipeline:
                        voltage_gate=threading.Event();voltage_cancelled=threading.Event()
                    record[pipeline_key]={
                        'status':'PENDING_AT_FEEDBACK','output_allowed':False,
                        **{name+'_ns_by_bus':{} for name in
                           ('feedback_dispatch','feedback_reply_end','feedback_ready',
                            'voltage_dispatch','voltage_reply_end')}}
                record_index=len(records);records.append(record)
                feedback_ready={scope:Future() for scope in dual.SCOPES}
                try:
                    for scope,ids in dual.SCOPES.items():
                        if private_seven_request_runtime is not None and private_seven_request_runtime.mode=='split7':
                            # Main constructs BOTH phases before either native
                            # task; their ORIGINAL full Futures run on this same
                            # existing three-worker pool alongside independent IMU.
                            if scope=='front':
                                voltage_futures=private_seven_request_runtime.start_split(
                                    {name:owned[cycle%6] for name,owned in dual.SCOPES.items()},
                                    feedback_ready,record['voltage_fast_pipeline'],cycle*2)
                        elif v3_voltage_pipeline:
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_gated_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope],
                                voltage_gate,voltage_cancelled,record['voltage_pipeline'],clock)
                        elif v3_voltage_fast_pipeline:
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope],
                                record['voltage_fast_pipeline'],clock,publish_before_native=True)
                        else:
                            # Ordinary overlap keeps its independent bus order:
                            # prepare the voltage call, publish this bus's six
                            # replies, then enter the GIL-releasing native call.
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope],
                                publish_before_native=True)
                    futures=feedback_ready
                    imu_future=pool.submit(read_imu)
                    if private_seven_request_runtime is not None and private_seven_request_runtime.mode=='split7':
                        private_seven_request_runtime.pump_feedback(feedback_ready)
                except BaseException:
                    if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                    settle_voltage(voltage_futures,record)
                    _retain_submitted_feedback(feedback_ready,voltage_futures,record)
                    raise
            else:
                futures={s:pool.submit(exchange,s,w) for s,w in acquisition_wires.items()}
                imu_future=pool.submit(read_imu)
            # Native-ready mode does not sleep sequentially on condition
            # notifications for front, rear and IMU. Readiness is not validation:
            # keep the original snapshot/causal/frame checks below unchanged.
            acquired={};failure=None
            if native_overlap_wait:
                wait_proof=record[pipeline_key] if pipeline_key is not None else record['voltage_overlap']
                try:
                    wait_proof['acquisition_join_wait']=_await_acquisition_ready(
                        futures,imu_future,deadline_ns=actual_release+PERIOD_NS,
                        deadline_wait=deadline_wait,clock=clock,check=check,
                        native_readiness_waiter=native_readiness_waiter)
                except BaseException as error:
                    failure=error
                    if native_future_notification_joins:
                        wait_proof['acquisition_join_failure_proof']=getattr(error,'readiness_poll_failure',None)
                    wait_proof.update(status='REJECTED_BEFORE_FEEDBACK_VALIDATION',
                        acquisition_join_error=type(error).__name__+': '+str(error))
                    if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
            # Every result is ready on success. Failure-only settlement retains
            # complete native evidence; it cannot reach inference or proxy STOP.
            for s,f in futures.items():
                try:acquired[s]=f.result()
                except BaseException as e:failure=failure or e
            try:sample=imu_future.result()
            except BaseException as e:sample=None;failure=failure or e
            if failure is None and native_overlap_wait:
                try:
                    check()
                    if clock()>=actual_release+PERIOD_NS:
                        raise TimeoutError('Acquisition result takeout exceeded 20 ms hard deadline')
                except BaseException as error:
                    failure=error
                    wait_proof.update(status='REJECTED_BEFORE_FEEDBACK_VALIDATION',
                        acquisition_join_error=type(error).__name__+': '+str(error))
            if v3_voltage_overlap:
                record['acquired']=acquired;record['imu']=sample
            else:
                record={'cycle':cycle+1,'acquired':acquired,'imu':sample,'output':{}}
                record_index=len(records);records.append(record)
            if failure:
                if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                if v3_voltage_overlap:settle_voltage(voltage_futures,record)
                raise failure
            gather_end=clock()
            if pipeline_key is not None:
                record[pipeline_key]['feedback_join_ns']=gather_end
            validation_future=None
            try:
                if sample['read_started_monotonic_ns']<=last_imu:
                    raise ValueError('IMU sample reused across cycles')
                last_imu=sample['read_started_monotonic_ns']
                expected_voltage=({scope:ids[cycle%6] for scope,ids in dual.SCOPES.items()}
                                  if v3_voltage_proxy else None)
                feedback_proof=None
                if v3_voltage_overlap:
                    snapshot,feedback_proof=_validated_feedback_for_voltage(acquired,sample,gather_end,
                        feedback_decoders=feedback_decoders)
                else:
                    snapshot=snapshot_from_records({s:x[0] for s,x in acquired.items()},sample,gather_end,
                                                  expected_voltage_by_bus=expected_voltage)
                if v3_voltage_overlap:
                    snapshot['source_flags']['v3_voltage_overlap_pending_at_inference']=True
                    if pipeline_key is not None or native_overlap_wait:
                        # STOP-proxy feedback and IMU are validated before
                        # inference. The fast path may already be reading
                        # voltage on its bus owners at this point.
                        oldest=min(sample['read_started_monotonic_ns'],
                                   *(r.start_ns for value in acquired.values() for r in value[0]))
                        pipeline_hard_end=min(actual_release+PERIOD_NS,oldest+PERIOD_NS)
                        proof=record[pipeline_key] if pipeline_key is not None else record['voltage_overlap']
                        proof['hard_deadline_ns']=pipeline_hard_end
                        snapshot_validated=clock()
                        if snapshot_validated>=pipeline_hard_end:
                            raise TimeoutError('Feedback exceeded 20 ms pipeline hard deadline')
                        proof['feedback_snapshot_validated_ns']=snapshot_validated
                        proof['status']='PENDING_AT_INFERENCE'
                        if v3_voltage_pipeline:
                            proof['voltage_gate_set_ns']=snapshot_validated
                            voltage_gate.set()
                    if v3_voltage_validation_overlap:
                        # The IMU worker is free after imu_future.result(). Its
                        # task waits on the two bus-owned voltage futures, then
                        # validates their immutable records during inference.
                        validation_future=pool.submit(_validate_voltage_during_inference,
                            voltage_futures,acquired,sample,snapshot,expected_voltage,clock,voltage_max_v,
                            feedback_proof,(private_seven_request_runtime.voltage_view
                                if private_seven_request_runtime is not None and private_seven_request_runtime.mode=='split7' else None))
                prepared=clock()
                if inference_cpu_values is not None:
                    inference_cpu_base=cycle*2
                    inference_cpu_values[inference_cpu_base]=time.thread_time_ns()
                observed=None
                if policy_observer is not None:
                    if cycle==0 and not policy_armed_before_cycles:policy_observer.arm_run(gather_end)
                    observed=policy_observer.consume(snapshot)
                if inference_cpu_values is not None:
                    inference_cpu_values[inference_cpu_base+1]=time.thread_time_ns()
                inferred=clock()
                if v3_voltage_overlap and output_dispatch_trace:
                    infer_thread_cpu_end=time.thread_time_ns()
            except BaseException:
                if v3_voltage_overlap:
                    if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                    settle_voltage(voltage_futures,record)
                    if validation_future is not None:
                        try:validation_future.result()
                        except BaseException as validation_error:
                            record['voltage_validation_error']=(
                                type(validation_error).__name__+': '+str(validation_error))
                raise
            if v3_voltage_overlap:
                # This scheduling comparison is selected only by the existing
                # native release-wait callback. Legacy no-callback diagnostic
                # sequencing and its failure-stage metadata stay unchanged.
                if native_overlap_wait:
                    wait_proof=record[pipeline_key] if pipeline_key is not None else record['voltage_overlap']
                    try:
                        wait_proof['voltage_join_wait']=_await_voltage_ready(
                            voltage_futures,validation_future,deadline_ns=pipeline_hard_end,
                            deadline_wait=deadline_wait,clock=clock,check=check,
                            native_readiness_waiter=native_readiness_waiter)
                    except BaseException as join_error:
                        if native_future_notification_joins:
                            wait_proof['voltage_join_failure_proof']=getattr(join_error,'readiness_poll_failure',None)
                        # Preserve every completed owner record on the failure
                        # path. This blocking settlement is cleanup, not an
                        # admission for another proxy STOP output batch.
                        settle_voltage(voltage_futures,record)
                        if validation_future is not None:
                            try:validation_future.result()
                            except BaseException as validation_error:
                                record['voltage_validation_error']=(
                                    type(validation_error).__name__+': '+str(validation_error))
                        wait_proof.update(
                            status='REJECTED_BEFORE_PROXY_STOP',
                            voltage_join_error=type(join_error).__name__+': '+str(join_error))
                        record.pop('observed',None)
                        raise
                voltage_errors=settle_voltage(voltage_futures,record)
                voltage_wait_end=clock()
                if pipeline_key is not None:
                    record[pipeline_key]['voltage_join_ns']=voltage_wait_end
                validation_started=validation_finished=final_freshness_checked=None
                if v3_voltage_validation_overlap:
                    try:
                        full_voltage,validation_started,validation_finished=(
                            validation_future.result())
                    except BaseException as validation_error:
                        record['voltage_validation_error']=(
                            type(validation_error).__name__+': '+str(validation_error))
                        if voltage_errors:raise voltage_errors[0]
                        raise
                    if voltage_errors:raise voltage_errors[0]
                    if pipeline_key is None:
                        try:
                            final_freshness_checked=_verify_voltage_final_freshness(
                                acquired,record['voltage'],sample,snapshot,full_voltage,
                                expected_voltage,clock,voltage_max_v,feedback_proof)
                        except BaseException as freshness_error:
                            record['voltage_freshness_error']=(
                                type(freshness_error).__name__+': '+str(freshness_error))
                            raise
                        verified_at=final_freshness_checked
                    else:
                        # The worker completed frame/mutation/age validation.
                        # This route has a second, actual dispatch gate below:
                        # walk fourteen timestamps once there, immediately
                        # before submitting STOP rather than twice in succession.
                        verified_at=_verify_joined_voltage_proof(
                            acquired,record['voltage'],sample,snapshot,full_voltage,feedback_proof,clock)
                else:
                    if voltage_errors:raise voltage_errors[0]
                    full_voltage,verified_at=_verify_voltage_after_inference(
                        acquired,record['voltage'],sample,snapshot,expected_voltage,clock,voltage_max_v,
                        feedback_proof)
                record['voltage_overlap']={**record['voltage_overlap'],
                    'status':'VALIDATED_BEFORE_PROXY_STOP',
                    'feedback_ready_ns':gather_end,'inference_end_ns':inferred,
                    'voltage_wait_end_ns':voltage_wait_end,
                    'voltage_verified_ns':verified_at,
                    'voltage_reply_end_ns_by_bus':{
                        scope:max(r.received_ns for r in value[0])
                        for scope,value in record['voltage'].items()},
                    'voltage_v_by_bus':{
                        scope:row['value_v'] for scope,row in full_voltage['voltage_by_bus'].items()},
                    'range_v':[35.,voltage_max_v], 'observer_snapshot_voltage_pending':True,
                    'output_allowed':False}
                if v3_voltage_validation_overlap:
                    record['voltage_overlap'].update(
                        validation_started_ns=validation_started,
                        validation_finished_ns=validation_finished,
                        final_freshness_checked_ns=final_freshness_checked)
                if pipeline_key is not None:
                    record[pipeline_key].update(
                        status='POST_INFERENCE_VALIDATED',
                        inference_end_ns=inferred,post_inference_verified_ns=verified_at,
                        range_v=[35.,voltage_max_v])
            if policy_observer is not None:record['observed']=observed
            if mode=='stop-proxy' and policy_observer is not None:
                if output_dispatch_trace:
                    dispatch_base=cycle*dispatch_stride
                    dispatch_values[dispatch_base]=inferred
                    dispatch_values[dispatch_base+13]=(
                        infer_thread_cpu_end if v3_voltage_overlap else time.thread_time_ns())
                    dispatch_values[dispatch_base+1]=clock()
                check()
                if pipeline_key is not None or native_overlap_wait:
                    gate_proof=record[pipeline_key] if pipeline_key is not None else record['voltage_overlap']
                    try:
                        # Mirror a pre-Type1 gate at the actual proxy
                        # dispatch point. STOP is the only possible output here.
                        final_gate=_verify_voltage_final_freshness(
                            acquired,record['voltage'],sample,snapshot,full_voltage,
                            expected_voltage,clock,voltage_max_v,feedback_proof)
                        if final_gate>=pipeline_hard_end:
                            raise TimeoutError('Voltage pipeline exceeded 20 ms hard deadline before proxy STOP')
                    except BaseException as gate_error:
                        gate_proof['status']='REJECTED_BEFORE_PROXY_STOP'
                        gate_proof['final_gate_error']=(
                            type(gate_error).__name__+': '+str(gate_error))
                        record.pop('observed',None)
                        raise
                    gate_proof.update(
                        status='VALIDATED_BEFORE_PROXY_STOP',voltage_verified_ns=final_gate)
                    record['voltage_overlap']['voltage_verified_ns']=final_gate
                    record['voltage_overlap']['final_freshness_checked_ns']=final_gate
                if output_dispatch_trace:dispatch_values[dispatch_base+2]=clock()
                futures={};failure=None;paired=None
                gated_proxy=pipeline_key is not None or native_overlap_wait
                try:
                    if diagnostic_output is not None:
                        submit_now=clock()
                        if not final_gate<=submit_now<pipeline_hard_end:
                            raise TimeoutError('Diagnostic runtime STOP submission exceeded original input/elapsed deadline')
                        gate_proof['proxy_submit_checked_ns_by_bus']={scope:submit_now for scope in dual.SCOPES}
                        diagnostic_output.set_dispatch_trace(
                            dispatch_values,dispatch_base,check) if output_dispatch_trace else None
                        futures=diagnostic_output.submit_decoded(wires,deadline_ns=pipeline_hard_end,
                            label='diagnostic_policy_output')
                        gate_proof['runtime_submit_completed_ns']=clock()
                        gate_proof['runtime_original_futures']=True
                        for scope in futures:
                            if output_dispatch_trace:
                                dispatch_values[dispatch_base+(3 if scope=='front' else 4)]=clock()
                    elif native_phase_pair_candidate is not None:
                        submit_now=clock()
                        if type(submit_now) is not int or not final_gate<=submit_now<pipeline_hard_end:
                            raise TimeoutError('Native pair STOP submission exceeded original input/elapsed deadline')
                        gate_proof['proxy_submit_checked_ns_by_bus']={scope:submit_now for scope in dual.SCOPES}
                        try:
                            paired=native_phase_pair_candidate.exchange_stop_proxy(wires,
                                deadline_ns=pipeline_hard_end)
                        except BaseException as pair_error:
                            for scope,raw in (getattr(pair_error,'native_pair_bus_results',None) or {}).items():
                                if isinstance(raw,BaseException):
                                    if not (hasattr(raw,'records') and hasattr(raw,'stats')):continue
                                    raw=(raw.records,raw.stats)
                                record['output'][scope]=_disabled_diagnostic_result(*raw,validate_stop=False)
                            record['native_phase_pair_phase']=getattr(pair_error,'native_pair_phase',None)
                            raise
                        # Save completed raw records before metadata takeout;
                        # a getter/tracing failure must not erase real replies.
                        record['output'].update(paired)
                        record['native_phase_pair_phase']=native_phase_pair_candidate.last_phase()
                        futures=native_phase_pair_candidate.last_completed_futures()
                        for scope in futures:
                            if output_dispatch_trace:
                                dispatch_values[dispatch_base+(3 if scope=='front' else 4)]=clock()
                    else:
                        for scope,scope_wires in wires.items():
                            if gated_proxy:
                                # Retain the legacy owners and submission checks.
                                submit_now=clock()
                                if type(submit_now) is not int or submit_now<final_gate:
                                    raise ValueError('Noncausal proxy STOP submission clock')
                                if submit_now>=pipeline_hard_end:
                                    raise TimeoutError('Voltage pipeline exceeded 20 ms hard deadline before proxy STOP submit')
                                gate_proof.setdefault('proxy_submit_checked_ns_by_bus',{})[scope]=submit_now
                            options=(dispatch_base,) if output_dispatch_trace else ()
                            futures[scope]=pool.submit(exchange,scope,scope_wires,*options)
                            if output_dispatch_trace:
                                dispatch_values[dispatch_base+(3 if scope=='front' else 4)]=clock()
                    if output_dispatch_trace:dispatch_values[dispatch_base+14]=time.thread_time_ns()
                except BaseException as submit_error:
                    failure=submit_error
                    if gated_proxy:
                        gate_proof.update(status='REJECTED_BEFORE_PROXY_STOP',
                            proxy_submit_error=type(submit_error).__name__+': '+str(submit_error))
                    record.pop('observed',None)
                if native_overlap_wait:
                    gate_proof['output_join_begin_ns']=clock()
                    output_join_deadline=_proxy_output_join_deadline(
                        actual_release,oldest,startup=cycle<startup_cycle_allowance)
                    gate_proof['output_join_deadline_ns']=output_join_deadline
                    gate_proof['output_join_startup_allowance']=cycle<startup_cycle_allowance
                    if failure is None:
                        try:
                            # Both original FD owners must finish before any
                            # result takeout. Match the acquisition/voltage
                            # readiness wait; do not alter native exchanges,
                            # the absolute input deadline, or source times.
                            if diagnostic_output is not None:
                                from .policy_output_runtime import _PendingCycleTiming
                                runtime_timing=_PendingCycleTiming()
                                groups_before=diagnostic_output.output_notification_groups
                                waits_before=diagnostic_output.output_notification_waits
                                try:
                                    diagnostic_output.collect_output(futures,deadline_ns=output_join_deadline,
                                        native_deadline_ns=pipeline_hard_end,
                                        deadline_wait=diagnostic_output_waiter,timing=runtime_timing)
                                finally:
                                    gate_proof['runtime_output_join_timing']={name:getattr(runtime_timing,name)
                                        for name in ('output_join_begin_ns','output_join_ready_ns',
                                            'output_takeout_end_ns','output_join_cpu_begin_ns','output_join_cpu_end_ns')}
                                    gate_proof['runtime_output_notification']={
                                        'selected':unpaired_output_future_notifications,
                                        'groups_created':diagnostic_output.output_notification_groups-groups_before,
                                        'wait_calls':diagnostic_output.output_notification_waits-waits_before}
                                gate_proof['output_join_wait']={'begin_ns':runtime_timing.output_join_begin_ns,
                                    'end_ns':clock(),'backend':'actual_busworkers_collect_output.v1',
                                    'original_futures_required':True,'deadline_ns':output_join_deadline,
                                    'notification_selected':unpaired_output_future_notifications}
                            else:
                                gate_proof['output_join_wait']=_await_output_ready(
                                    futures,deadline_ns=output_join_deadline,
                                    deadline_wait=deadline_wait,clock=clock,check=check,
                                    native_readiness_waiter=native_readiness_waiter)
                        except BaseException as join_error:
                            failure=join_error
                            gate_proof['output_join_failure_proof']=_output_join_failure_proof(
                                join_error,'readiness_join',futures,clock)
                # Retain any already submitted batch if a later submit fails.
                # A failed readiness wait also settles every submitted owner:
                # this is diagnostic cleanup, never a fresh admission. These
                # results and settlement time stay in the failed raw record.
                if diagnostic_output is not None and failure is not None:
                    failure=_settle_failed_decoded_output_owners(
                        futures,record,failure,gate_proof,clock)
                else:
                    for s,f in futures.items():
                        try:
                            if diagnostic_output is not None:
                                from .private_seven_request_bridge import diagnostic_result
                                record['output'][s]=diagnostic_result(*f.result()[0])
                            else:record['output'][s]=paired[s] if paired is not None else f.result()
                        except BaseException as e:
                            if diagnostic_output is not None and hasattr(e,'records') and hasattr(e,'stats'):
                                from .private_seven_request_bridge import diagnostic_result
                                record['output'][s]=diagnostic_result(e.records,e.stats)
                            failure=failure or e
                if native_overlap_wait:
                    gate_proof['output_join_settled_ns']=clock()
                    if failure is None:
                        try:
                            check()
                            joined=clock()
                            if type(joined) is not int or joined<gate_proof['output_join_wait']['end_ns']:
                                raise ValueError('Noncausal proxy output takeout clock')
                            gate_proof['output_join_checked_ns']=joined
                            if joined>=output_join_deadline:
                                raise TimeoutError('Proxy output result takeout exceeded its elapsed/input-age deadline')
                        except BaseException as takeout_error:
                            failure=takeout_error
                            gate_proof['output_join_failure_proof']=_output_join_failure_proof(
                                takeout_error,'result_takeout',futures,clock)
                    if failure is not None:
                        # Retain an earlier submit rejection and its reason.
                        if 'proxy_submit_error' not in gate_proof:
                            gate_proof['status']='FINAL_PROXY_OUTPUT_JOIN_REJECTED'
                        gate_proof['output_join_error']=type(failure).__name__+': '+str(failure)
                        gate_proof['output_join_cleanup_only']=True
                        record.pop('observed',None)
                if failure:raise failure
                if gated_proxy:
                    actual_starts={scope:min(r.start_ns for r in result[0])
                                   for scope,result in record['output'].items()}
                    gate_proof['proxy_actual_start_ns_by_bus']=actual_starts
                    if any(not 0<started<pipeline_hard_end for started in actual_starts.values()):
                        gate_proof['status']='PROXY_STOP_DISPATCH_DEADLINE_MISSED'
                        record.pop('observed',None)
                        raise TimeoutError('Proxy STOP actual dispatch exceeded 20 ms hard deadline')
                if v3_voltage_fast_pipeline:
                    try:
                        stop_reply_ends,stop_verified=_verify_final_proxy_stop_records(
                            record['output'],clock)
                    except BaseException as stop_error:
                        record['voltage_fast_pipeline']['status']='FINAL_STOP_REPLY_REJECTED'
                        record['voltage_fast_pipeline']['stop_reply_error']=(
                            type(stop_error).__name__+': '+str(stop_error))
                        raise
                    record['voltage_fast_pipeline'].update(
                        stop_reply_end_ns_by_bus=stop_reply_ends,
                        stop_reply_count=12,stop_reply_verified_ns=stop_verified)
                if output_dispatch_trace:
                    for scope,(scope_records,scope_stats) in record['output'].items():
                        offset=7 if scope=='front' else 11
                        dispatch_values[dispatch_base+offset]=scope_stats.begin_ns
                        dispatch_values[dispatch_base+offset+1]=scope_records[0].start_ns
            output=record['output']
            if record_storage=='trace':
                # Copy bounded native buffers into storage allocated before
                # worker startup. The copy and release precede cycle_end_ns.
                try:
                    traced=trace.capture(cycle,record)
                    timing_values=_timing_scalars(acquired,sample,output)
                except BaseException as error:
                    storage_failure={'storage_index':record_index,'cycle':cycle+1,
                        'error':type(error).__name__+': '+str(error)}
                    raise
                records[record_index]=traced
                record=traced=acquired=sample=output=snapshot=imu_future=f=None
                voltage_futures=validation_future=full_voltage=observed=None
                feedback_proof=None
                futures={};paired=None
            elif record_storage=='encoded':
                # Every output future is settled. Keep the raw in-flight row
                # until encoding succeeds, and include encoding/release in the
                # complete cycle rather than retaining its nested trees.
                try:
                    serialized=_serialize([record])
                    encoded=json.dumps(serialized,allow_nan=False)
                    # Timing needs only scalar timestamps. Drop owned record,
                    # native buffers, futures and observation trees before end.
                    timing_acquired={s:([_TimingRecord(r['start_ns'],r['finish_ns'],r['received_ns'])
                        for r in value['records']],None) for s,value in serialized[0]['acquired'].items()}
                    timing_output={s:([_TimingRecord(r['start_ns'],r['finish_ns'],r['received_ns'])
                        for r in value['records']],None) for s,value in serialized[0]['output'].items()}
                    timing_sample={k:sample[k] for k in
                        ('read_started_monotonic_ns','read_finished_monotonic_ns')}
                except BaseException as error:
                    storage_failure={'storage_index':record_index,'cycle':cycle+1,
                        'error':type(error).__name__+': '+str(error)}
                    raise
                records[record_index]=encoded
                record=serialized=acquired=sample=output=snapshot=imu_future=f=None
                futures={};paired=None
            else:
                timing_acquired,timing_sample,timing_output=acquired,sample,output
            end=clock()
            if record_storage=='trace':
                timing=_timing_row_from_scalars(*timing_values,release_ns=actual_release,
                    gather_end_ns=gather_end,prepare_end_ns=prepared,infer_end_ns=inferred,cycle_end_ns=end)
            else:
                timing=timing_row(timing_acquired,timing_sample,timing_output,release_ns=actual_release,
                    gather_end_ns=gather_end,prepare_end_ns=prepared,infer_end_ns=inferred,cycle_end_ns=end)
            timing['release_lateness_ms']=max(0,actual_release-release)/1e6
            timing['actual_release_interval_ms']=(actual_release-previous_release)/1e6 if previous_release is not None else None
            timing['timing_phase']='startup' if cycle<startup_cycle_allowance else 'steady'
            if absolute_epoch_cadence:
                timing.update(cadence_slot=slot,scheduled_release_ns=release,
                    skipped_slots_before=skipped,
                    wait_enter_ns=wait_enter,wait_return_ns=wait_return,wait_calls=wait_calls,
                    scheduled_completion_slack_ms=(release+PERIOD_NS-end)/1e6,
                    start_interval_over_20ms=(previous_release is not None and
                                              actual_release-previous_release>PERIOD_NS))
            measurements.append(timing);previous_release=actual_release
            if absolute_epoch_cadence:previous_slot=slot
            if output_dispatch_trace:active_cycle=0
            # Misses are retained; never run a catch-up burst or backdate source times.
            if not absolute_epoch_cadence:release=max(actual_release+PERIOD_NS,end)
    except BaseException as error:
        errors.append(type(error).__name__+': '+str(error))
        if policy_observer is not None:policy_observer.invalidate(errors[-1])
    finally:
        if gc_probe_installed:
            try:gc.callbacks.remove(gc_probe)
            except BaseException as error:
                errors.append('GC probe removal: '+type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if gc_restore_required:
            for attempt in range(3):
                gc_state['restore_attempts']=attempt+1
                try:gc.set_threshold(*gc_state['before_threshold'])
                except BaseException as error:
                    gc_state['restore_errors'].append('threshold: '+repr(error))
                try:
                    if gc_state['before_enabled']:gc.enable()
                    else:gc.disable()
                except BaseException as error:
                    gc_state['restore_errors'].append('enabled state: '+repr(error))
                try:
                    gc_state['after_enabled']=gc.isenabled()
                    gc_state['after_threshold']=tuple(gc.get_threshold())
                    gc_state['restored']=(gc_state['after_enabled']==gc_state['before_enabled'] and
                        gc_state['after_threshold']==gc_state['before_threshold'])
                except BaseException as error:
                    gc_state['restore_errors'].append('readback: '+repr(error))
                    gc_state['restored']=False
                if gc_state['restored']:break
            if not gc_state['restored']:
                errors.append('Automatic GC state/threshold restoration unconfirmed')
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if worker_restore_required:
            try:
                worker_affinity['workers_after']=_transition_worker_affinity(
                    pool,original_worker_masks,None)
                worker_affinity['restore_errors']=[row['error'] for row in worker_affinity['workers_after']
                                                    if row['error'] is not None]
                worker_affinity['restored']=not worker_affinity['restore_errors']
            except BaseException as error:
                worker_affinity['restored']=False
                worker_affinity['restore_errors'].append(type(error).__name__+': '+str(error))
            if not worker_affinity['restored']:
                errors.append('I/O worker affinity restoration unconfirmed')
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if original_affinity is not None:
            try:
                os.sched_setaffinity(0,original_affinity)
                affinity['restored']=set(os.sched_getaffinity(0))==original_affinity
                if not affinity['restored']:raise RuntimeError('Main-thread affinity restoration differs')
            except BaseException as error:
                affinity['restored']=False
                errors.append(type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if startup['end_ns'] is None:startup['end_ns']=clock()
        startup['duration_ms']=(startup['end_ns']-startup['begin_ns'])/1e6
        if pool is not None:pool.shutdown(wait=workers_ready,cancel_futures=not workers_ready)
        if diagnostic_output is not None:
            # Pool shutdown above joins normal output and any runtime emergency
            # STOP Futures. No second collector STOP races those FD owners.
            try:
                diagnostic_output_proof=diagnostic_output.evidence()
                diagnostic_output_proof['emergency_stop_original_results']={
                    scope:future.result() for scope,future in (diagnostic_output.stop_futures or {}).items()}
                diagnostic_output_proof['runtime_emergency_errors']=list(diagnostic_output.emergency_errors)
                diagnostic_output.close()
            except BaseException as error:
                errors.append('Diagnostic runtime output detach/evidence: '+type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if private_seven_request_runtime is not None:
            try:private_seven_request_runtime.close()
            except BaseException as error:
                errors.append('Private native owner close/restore: '+type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if native_phase_pair_candidate is not None:
            try:native_phase_pair_candidate.close()
            except BaseException as error:
                errors.append('Native pair cleanup: '+type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
    trace_copy_proof=None
    if trace_copy_backend is not None:
        try:trace_copy_proof=trace_copy_backend.verify()
        except BaseException as error:
            errors.append(type(error).__name__+': '+str(error))
            trace_copy_proof=trace_copy_backend.provenance()
            trace_copy_proof['source_files_unchanged']=False
            if policy_observer is not None:policy_observer.invalidate(errors[-1])
    summary=policy_observer.finish() if policy_observer is not None else None
    report={'status':'COMPLETE_DIAGNOSTIC' if not errors else 'ABORTED',
        'mode':mode,'errors':errors,'cycles_requested':cycles,'cycles_completed':len(measurements),
        'v3_voltage_proxy':v3_voltage_proxy,
        'voltage_max_v':voltage_max_v,'voltage_range_v':[35.,voltage_max_v],
        'motor_enable_sent':False,'learned_targets_sent':False,'approved_for_runtime':False,
        'full_controller_50Hz_verified':False,'worker_startup':startup,
        'input_acquisition_wait':('native_future_notification.v1' if native_future_notification_joins else 'native_ready_poll_200us.v1'
            if native_overlap_wait else 'legacy_result_collection.v1'),
        'voltage_join_wait':('native_future_notification.v1' if native_future_notification_joins else 'native_ready_poll_200us.v1'
            if native_overlap_wait else 'legacy_result_collection.v1'),
        'output_join_wait':('native_future_notification.v1' if native_future_notification_joins else 'native_ready_poll_200us.v1'
            if native_overlap_wait else 'legacy_result_collection.v1'),
        'main_thread_affinity':affinity,'worker_affinity':worker_affinity,
        'measurements':measurements,'observer':summary,
        'distributions_ms':{k:distribution([r[k] for r in measurements if r[k] is not None]) for k in
            ('acquisition_ms','prepare_ms','inference_ms','oldest_input_to_final_host_write_ms',
             'oldest_input_to_last_reply_ms','whole_iteration_ms','release_lateness_ms','actual_release_interval_ms')},
        'host_deadline_misses':sum(not r['host_deadline_met'] for r in measurements)
            if mode=='stop-proxy' and policy_observer is not None else None,
        'iteration_deadline_misses':sum(not r['iteration_deadline_met'] for r in measurements),
        'release_lateness_over_1ms':sum(r['release_lateness_ms']>1. for r in measurements),
        'release_intervals_over_21ms':sum((r['actual_release_interval_ms'] or 0)>21. for r in measurements)}
    if native_future_notification_joins:
        report['native_future_notification_joins']={
            'enabled':True,'abi':1,'wait_mode':'native_future_notification.v1',
            'scope':'disabled_stop_proxy_only','joins':_notification_join_evidence(records),
            'release_wait_backend':'unchanged_diagnostic_native_release',
            'source_timestamps_changed':False,'original_absolute_deadlines_unchanged':True,
            'hints_certify_readiness':False,'type1_speedup_verified':False}
    if diagnostic_bus_workers_output:
        report['output_join_wait']=('actual_busworkers_collect_output_native_future_notification.v1'
            if unpaired_output_future_notifications else
            'actual_busworkers_collect_output_native_ready_poll_200us.v1')
    if diagnostic_output_proof is not None:
        report['diagnostic_runtime_output']=diagnostic_output_proof
    if feedback_codec_proof is not None:
        report['native_feedback_batch_decode']={'enabled':True,
            'scope':'ordinary_unpaired_owners.v1','selected_buses':['front','rear'],
            'source_binding':feedback_codec_proof,'diagnostic_only':True,
            'active_controller_qualification':False,'hardware_timing_improvement_proven':False,
            'original_full_dynamic_dispatch_gate_unchanged':True}
        report['native_phase_pair']={'enabled':False,'diagnostic_only':True}
    if native_phase_pair_candidate is not None:
        candidate_evidence=native_phase_pair_candidate.evidence()
        report.update(native_phase_pair=True,native_phase_pair_evidence=candidate_evidence,
            active_controller_qualification=False,output_allowed=False)
        if pair_prime is not None:
            # Use only the exact administrative generation, including partial
            # records on failure; never attach an earlier phase as its proof.
            matching = [row for row in candidate_evidence['journal']
                        if row['deadline_ns'] == pair_prime['deadline_ns']]
            if len(matching) == 1:
                pair_prime.update({key: matching[0][key]
                    for key in ('raw_by_scope', 'joined', 'phase')})
                for error in matching[0]['errors']:
                    if error not in pair_prime['errors']:pair_prime['errors'].append(error)
            report['native_pair_prime_before_cycles'] = True
            report['native_phase_pair_prime'] = pair_prime
        if candidate_evidence['errors']:
            report['status']='ABORTED'
            for error in candidate_evidence['errors']:
                if error not in report['errors']:report['errors'].append(error)
        if not all((candidate_evidence['all_phases_joined'],
                candidate_evidence['owner_placement_verified'],candidate_evidence['owner_settings_restored'],
                candidate_evidence['coordinator_placement_verified'],candidate_evidence['coordinator_settings_restored'],
                candidate_evidence['source_files_unchanged'])):
            report['status']='ABORTED'
            report['errors'].append('Native pair source/join/owner restoration proof incomplete')
        if report['status']=='COMPLETE_DIAGNOSTIC' and len(measurements)==cycles:
            report['native_phase_pair_proof']={'mode':'persistent_dual_owner.v1',
                'request_count_per_cycle':26,'all_phases_joined':True,
                'owner_placement_verified':True,'owner_settings_restored':True,
                'coordinator_placement_verified':True,'coordinator_settings_restored':True,
                'active_deadlines_unchanged':True,'scope':'disabled_stop_proxy_only',
                'type1_or_enable_tested':False,'output_allowed':False}
    if trace_copy_proof is not None:report['trace_copy_provenance']=trace_copy_proof
    if prepare_voltage_before_feedback_publication:
        report['prepare_voltage_before_feedback_publication']=True
    if v3_voltage_overlap:
        report['v3_voltage_overlap']={'enabled':True,'voltage_range_v':[35.,voltage_max_v],
            'validation_overlap_enabled':v3_voltage_validation_overlap,
            'native_readiness_wait_enabled':native_overlap_wait,
            'voltage_dispatch_schedule':('after_complete_feedback_imu_snapshot'
                                         if v3_voltage_pipeline else 'after_each_bus_feedback'),
            'observer_snapshot_voltage_pending':True,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{
                'acquisition_ms':'feedback6_per_bus_and_imu_gather; voltage may run concurrently',
                'input_latest_reply_ns':'latest feedback reply or IMU read finish; excludes voltage replies',
                'voltage_verified_ns':'records[].voltage_overlap.voltage_verified_ns',
                'whole_iteration_ms':'actual release through voltage validation, STOP replies, and trace capture'},
            'diagnostic_only':True,'motor_output_allowed':False}
        if pipeline_key is None:
            report['v3_voltage_overlap']['feedback_publication']=_ORDINARY_FEEDBACK_PUBLICATION
        if v3_voltage_validation_overlap:
            report['v3_voltage_overlap']['timing_fields'].update(
                validation_started_ns='records[].voltage_overlap.validation_started_ns',
                validation_finished_ns='records[].voltage_overlap.validation_finished_ns',
                final_freshness_checked_ns='records[].voltage_overlap.final_freshness_checked_ns')
    if v3_voltage_pipeline:
        report['v3_voltage_pipeline']={
            'enabled':True,'schema':'feedback-then-voltage-proxy-v1',
            'period_ns':PERIOD_NS,'feedback_gate_before_voltage':True,
            'pre_gate_proof':'complete native STOP feedback, finite IMU, causal timestamps and 20 ms freshness',
            'active_feedback_safety_equivalent':False,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{name:'records[].voltage_pipeline.'+name for name in (
                'feedback_dispatch_ns_by_bus','feedback_reply_end_ns_by_bus',
                'feedback_ready_ns_by_bus','feedback_join_ns','hard_deadline_ns',
                'voltage_gate_set_ns',
                'voltage_dispatch_ns_by_bus','voltage_reply_end_ns_by_bus',
                'voltage_join_ns','post_inference_verified_ns','voltage_verified_ns',
                'inference_end_ns')},
            'diagnostic_only':True,'motor_output_allowed':False,
            'learned_targets_sent':False,
            'independent_emergency_stop_available':False,
            'failure_behavior':'Abort normal STOP-proxy batch; no Type1 path; native session errors poison the session'}
    if v3_voltage_fast_pipeline:
        report['v3_voltage_fast_pipeline']={
            'enabled':True,'schema':'immediate-feedback-voltage-proxy-v1',
            'period_ns':PERIOD_NS,'voltage_dispatch_schedule':'after_each_bus_feedback',
            'feedback_publication':'after_voltage_native_preparation',
            'voltage_may_precede_global_feedback_validation':True,
            'active_feedback_safety_equivalent':False,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{name:'records[].voltage_fast_pipeline.'+name for name in (
                'feedback_dispatch_ns_by_bus','feedback_reply_end_ns_by_bus',
                'feedback_ready_ns_by_bus','feedback_published_ns_by_bus',
                'feedback_join_ns','feedback_snapshot_validated_ns',
                'hard_deadline_ns','voltage_dispatch_ns_by_bus','voltage_reply_end_ns_by_bus',
                'voltage_join_ns','post_inference_verified_ns','voltage_verified_ns',
                'inference_end_ns','stop_reply_end_ns_by_bus','stop_reply_count',
                'stop_reply_verified_ns')},
            'diagnostic_only':True,'motor_output_allowed':False,
            'learned_targets_sent':False,'independent_emergency_stop_available':False,
            'failure_behavior':'Abort normal STOP-proxy batch; no Type1 path; native session errors poison the session'}
    if private_seven_request_runtime is not None:
        report['private_seven_request_experiment']=private_seven_request_runtime.evidence()
        report['private_seven_request_prime']=getattr(private_seven_request_runtime,'prime',None)
        report['v3_voltage_fast_pipeline']['voltage_dispatch_schedule']=(
            'combined_native7; seventh_write_can_precede_all_six_feedback_replies'
            if private_seven_request_runtime.mode=='split7' else 'separate_native6_then1_on_same_unpaired_executor')
        report['v3_voltage_fast_pipeline']['feedback_publication']=(
            'bridge_after_validated_immutable_native_partial7_prefix'
            if private_seven_request_runtime.mode=='split7' else 'after_voltage_native_preparation')
    report['steady_timing']=_steady_timing_summary(measurements,cycles,startup_cycle_allowance,
                                                report['status']=='COMPLETE_DIAGNOSTIC')
    if absolute_epoch_cadence:
        skipped_total=sum(r['skipped_slots_before'] for r in measurements)
        over_20=sum(r['start_interval_over_20ms'] for r in measurements)
        report['absolute_epoch_schedule']={
            'enabled':True,'epoch_ns':cadence_epoch,'period_ns':PERIOD_NS,
            'minimum_start_separation_ns':ABSOLUTE_MIN_START_SEPARATION_NS,
            'slots_skipped':skipped_total,'start_intervals_over_20ms':over_20,
            'strict_start_interval_20ms_met':bool(
                report['status']=='COMPLETE_DIAGNOSTIC' and len(measurements)==cycles and
                len(measurements)>1 and skipped_total==0 and over_20==0),
            'diagnostic_only':True,'learned_targets_sent':False}
    if output_dispatch_trace:
        report['output_dispatch_trace']={'schema':'native-output-dispatch-v1',
            'main_native_tid':threading.get_native_id(),
            'fields':list(_OUTPUT_DISPATCH_FIELDS),
            'rows':[list(dispatch_values[i*dispatch_stride:(i+1)*dispatch_stride])
                    for i in range(len(measurements))],
            'gc_events':[{'monotonic_ns':gc_times[i], 'native_tid':gc_tids[i],
                          'cycle':gc_cycles[i], 'generation':gc_generations[i],
                          'phase':'start' if gc_phases[i]==0 else 'stop'}
                         for i in range(gc_count)],
            'gc_overflow':gc_overflow,'gc_probe_errors':gc_errors}
    if inference_cpu_values is not None:
        cpu_rows=[]
        for index,measurement in enumerate(measurements):
            begin=inference_cpu_values[index*2]
            finish=inference_cpu_values[index*2+1]
            cpu_ns=finish-begin
            wall_ns=measurement['infer_end_ns']-measurement['prepare_end_ns']
            cpu_rows.append([index+1,begin,finish,cpu_ns,wall_ns,wall_ns-cpu_ns])
        report['inference_thread_cpu_trace']={
            'schema':'native-inference-thread-cpu-v1',
            'clock':'time.thread_time_ns',
            'scope':('collector main thread policy phase; arm_run completed before timed release'
                     if policy_armed_before_cycles else
                     'collector main thread policy phase; cycle 1 includes arm_run'),
            'helper_thread_cpu_included':False,
            'main_native_tid':threading.get_native_id(),
            'fields':['cycle','thread_cpu_begin_ns','thread_cpu_end_ns',
                      'thread_cpu_ns','inference_wall_ns','wall_minus_thread_cpu_ns'],
            'rows':cpu_rows}
    if dispatch_values is not None or inference_cpu_values is not None:
        # Only populated slots beyond completed measurements are partial evidence.
        # Build this after every worker has settled; no hot-loop work or invented
        # measurement, elapsed time, or successful qualification is added.
        incomplete=[]
        for index in range(len(measurements),cycles):
            dispatch=(list(dispatch_values[index*dispatch_stride:(index+1)*dispatch_stride])
                      if dispatch_values is not None else [])
            cpu=(list(inference_cpu_values[index*2:(index+1)*2])
                 if inference_cpu_values is not None else [])
            if not any(dispatch) and not any(cpu):continue
            row={'cycle':index+1,'complete_measurement':False,
                 'output_allowed':False,'reported_errors':list(errors)}
            if any(dispatch):
                row['output_dispatch_trace']={'fields':list(_OUTPUT_DISPATCH_FIELDS),
                    'row':[value if value else None for value in dispatch]}
            if any(cpu):
                row['inference_thread_cpu_trace']={
                    'clock':'time.thread_time_ns',
                    'fields':['thread_cpu_begin_ns','thread_cpu_end_ns'],
                    'row':[value if value else None for value in cpu]}
            for raw in records:
                if type(raw) is dict and raw.get('cycle')==index+1:
                    gate=raw.get(pipeline_key or 'voltage_overlap',{})
                    if 'output_join_failure_proof' in gate:
                        proof=gate['output_join_failure_proof']
                        row['output_join_failure_proof']={**proof,
                            'owner_future_states':{scope:dict(state) for scope,state
                                                   in proof['owner_future_states'].items()}}
                    if 'output_owner_settlement' in gate:
                        row['output_owner_settlement']=copy.deepcopy(gate['output_owner_settlement'])
                    break
            incomplete.append(row)
        report['incomplete_cycle_traces']=incomplete
    if defer_gc_during_cycles:report['cycle_gc_defer']=gc_state
    if record_storage=='encoded':
        report['record_storage']={'mode':'encoded','encoding_inside_whole_iteration':True,
            'decoded_after_collection':True,'completed_encoded_rows':sum(type(r) is str for r in records)}
        if storage_failure is not None:report['record_storage_failure']=storage_failure
    elif record_storage=='trace':
        report['record_storage']={'mode':'trace','copy_inside_whole_iteration':True,
            'serialized_after_collection':True,'capacity_cycles':cycles,
            'allocated_bytes':trace.allocated_bytes if trace is not None else 0,
            'pretouched_before_release_bytes':trace.pretouched_bytes if trace is not None else 0,
            'completed_trace_rows':sum(type(r) is _TraceRow for r in records)}
        if storage_failure is not None:report['record_storage_failure']=storage_failure
    return report,records


def _serialize(records):
    result=[]
    for row in records:
        if type(row) is _TraceRow:
            result.append(row.serialize());continue
        if type(row) is str:
            # Successful encoded rows are decoded only after collection. An
            # abort can also leave ordinary partial/native-failure rows here.
            decoded=json.loads(row)
            if type(decoded) is not list or len(decoded)!=1 or type(decoded[0]) is not dict:
                raise ValueError('Invalid encoded diagnostic record')
            result.extend(decoded)
            continue
        if 'native_failure' in row:
            e=row['native_failure'];result.append({'failure_scope':row['failure_scope'],
                'error':str(e),**native.exchange_evidence(e.records,e.stats)});continue
        phases=('acquired','voltage','output') if 'voltage' in row else ('acquired','output')
        result.append({**{k:v for k,v in row.items() if k not in phases},
            **{k:{s:native.exchange_evidence(*r) for s,r in row[k].items()} for k in phases}})
    return result


def _storage_invalid_fields(value):
    """Bounded failure inspection; never invoke arbitrary value repr/copy hooks."""
    issues=[];active=set();left=2000;truncated=False
    def typename(item):
        return type.__getattribute__(type(item),'__name__')[:128]
    def representation(item):
        kind=type(item)
        if kind is str:return str.__getitem__(item,slice(0,256))
        if kind is float:return float.__repr__(item)
        if kind is int:return '<integer bits='+str(int.bit_length(item))+'>'
        if kind is bool or item is None:return str(item)
        if kind is bytes:return bytes.hex(item[:64])
        return '<'+typename(item)+'>'
    def add(item,path):
        issues.append({'path':path[:256],'type':typename(item),'representation':representation(item)[:256]})
    def visit(recurse,item,path,depth):
        nonlocal left,truncated
        if left<=0 or depth>24 or len(issues)>=16:
            truncated=True;return
        left-=1
        if item is None or isinstance(item,(str,int,bool)):return
        if isinstance(item,float):
            if not math.isfinite(float.__float__(item)):add(item,path)
            return
        if not isinstance(item,(dict,list,tuple)):
            add(item,path);return
        identity=id(item)
        if identity in active:
            add(item,path);return
        active.add(identity)
        try:
            entries=dict.items(item) if isinstance(item,dict) else enumerate(item)
            for key,child in entries:
                if left<=0 or len(issues)>=16:
                    truncated=True;break
                if isinstance(item,dict):
                    if not (key is None or isinstance(key,(str,int,float,bool))):add(key,path+'.<key>')
                    elif isinstance(key,float) and not math.isfinite(float.__float__(key)):add(key,path+'.<key>')
                    label=(str.__getitem__(key,slice(0,80)) if isinstance(key,str) else representation(key))
                    child_path=path+'.'+label
                else:child_path=path+'['+str(key)+']'
                recurse(recurse,child,child_path,depth+1)
        finally:active.remove(identity)
    visit(visit,value,'$',0)
    return issues,truncated


def _storage_failure_artifact(row,index,error):
    metadata=({k:v for k,v in row.items() if k not in ('acquired','voltage','output','native_failure')}
              if type(row) is dict else row.metadata if type(row) is _TraceRow else row)
    invalid,truncated=_storage_invalid_fields(metadata)
    evidence={}
    if type(row) is _TraceRow:
        for phase,scopes in row.phase_scopes.items():
            evidence[phase]={}
            for scope in scopes:
                try:evidence[phase][scope]=row.storage.evidence(row.cycle_index,phase,scope)
                except Exception as failure:
                    evidence[phase][scope]={'evidence_error':type(failure).__name__+': '+str(failure)[:256]}
    elif type(row) is dict:
        for key in ('acquired','voltage','output'):
            if key not in row:continue
            evidence[key]={}
            for scope,result in row.get(key,{}).items():
                try:
                    if scope not in dual.SCOPES or not 1<=len(result[0])<=12:
                        raise ValueError('Invalid bounded native evidence')
                    evidence[key][scope]=native.exchange_evidence(*result)
                except Exception as failure:
                    evidence[key][scope]={'evidence_error':type(failure).__name__+': '+str(failure)[:256]}
        if isinstance(row.get('native_failure'),native.ExchangeError):
            failure=row['native_failure']
            if 1<=len(failure.records)<=12:
                evidence['native_failure']={'failure_scope':row.get('failure_scope'),
                    'error':str(failure)[:512],**native.exchange_evidence(failure.records,failure.stats)}
    return {'storage_index':index,'cycle':metadata.get('cycle') if type(metadata) is dict else None,
        'error':error[:512],'invalid_json_fields':invalid,'inspection_truncated':truncated,
        'native_evidence':evidence,'record_is_successful':False,'output_allowed':False,
        'approved_for_runtime':False}


def _encoded_records_for_output(records,encoding_failure=None):
    """Post-run conversion with explicit bounded failed-row evidence, never approval."""
    result=[];failures=[]
    for index,row in enumerate(records):
        try:
            value=_serialize([row])[0]
            if type(row) is not str:json.dumps(value,allow_nan=False)
        except Exception as error:
            failure=_storage_failure_artifact(row,index,type(error).__name__+': '+str(error))
            failures.append(failure)
            result.append({'cycle':failure['cycle'],'status':'RECORD_STORAGE_FAILED',
                'record_storage_failure_index':len(failures)-1,'native_evidence':failure['native_evidence'],
                'output_allowed':False,'approved_for_runtime':False})
        else:
            result.append(value)
            if encoding_failure is not None and encoding_failure['storage_index']==index:
                failures.append(_storage_failure_artifact(row,index,encoding_failure['error']))
    return result,failures


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    if _UNRELEASED_DISABLED_PAIR_RESOURCES:
        p.error('Native ownership remains unresolved; this interpreter must exit before another diagnostic')
    p.add_argument('--execute',action='store_true');p.add_argument('--supported-disabled',action='store_true')
    p.add_argument('--private-seven-request-experiment',choices=('baseline6plus1','split7'),
                   help='PRIVATE default-off real-model genuine-unpaired900 comparison; no output approval')
    p.add_argument('--private-seven-request-library');p.add_argument('--private-seven-request-library-sha256')
    p.add_argument('--private-seven-request-source-inventory');p.add_argument('--private-seven-request-source-inventory-sha256')
    p.add_argument('--provenance-mode',choices=('supported-geometric-preload-5s-v1',
                   'supported-policy-probe-v1', 'supported-policy-probe-2s-rare-jitter-v1',
                   'supported-policy-probe-10s-after-2s-v1', 'supported-policy-probe-20s-after-10s-v1',
                   'human-supported-partial-current-hold-audio-8s-v1'),
                   help='Pin current sources for the named supported scope in this disabled diagnostic; requires --power-epoch and grants no output approval')
    p.add_argument('--power-epoch',
                   help='Explicit current motor-power epoch assertion for --provenance-mode; never inferred from an earlier report')
    p.add_argument('--native-boot-guard-artifact',
                   help='Opt-in disabled STOP-proxy experiment: SHA-pinned fresh-pread C++ boot guard artifact')
    p.add_argument('--native-boot-guard-artifact-sha256')
    p.add_argument('--retain-gil-trace-copy',action='store_true',
                   help='Diagnostic-only: preserve the GIL during copies of owned traced STOP-proxy records; no active-controller qualification')
    p.add_argument('--owned-buffer-trace-copy',action='store_true',
                   help='Explicit native-pair disabled comparison: copy owned trace buffers using byte memoryviews; no active-controller qualification')
    p.add_argument('--mode',choices=('type17','stop-proxy'),default='type17')
    p.add_argument('--v3-voltage-proxy',action='store_true',
                   help='Disabled-only 26-request proxy: six STOP feedback plus one rotating voltage read per bus, then six STOP; never Type1 or motor enable')
    p.add_argument('--v3-voltage-overlap',action='store_true',
                   help='Diagnostic-only: infer from six feedback replies while each bus reads its separate voltage; verify selected voltage range on both buses before proxy STOP')
    p.add_argument('--voltage-max-v',type=int,choices=(42,43),default=42,
                   help='Explicit voltage upper bound; lower bound remains 35 V, default upper bound 42 V')
    p.add_argument('--v3-voltage-validation-overlap',action='store_true',
                   help='Diagnostic-only: validate completed voltage replies on the released IMU worker during inference; recheck all input timestamps before proxy STOP')
    p.add_argument('--v3-voltage-pipeline',action='store_true',
                   help='Diagnostic-only: keep each bus owner gated after six feedback replies; release separate voltage reads after the feedback/IMU snapshot, then validate before proxy STOP')
    p.add_argument('--v3-voltage-fast-pipeline',action='store_true',
                   help='Diagnostic-only: each bus owner reads voltage immediately after its six feedback replies; retain per-cycle timing proof and validate before proxy STOP')
    p.add_argument('--prepare-voltage-before-feedback-publication',action='store_true',
                   help='Record explicit use of the existing fast-path native-preparation publication order; does not certify the active transport API')
    p.add_argument('--cycles',type=int,choices=range(1,3001),metavar='1..3000')
    p.add_argument('--startup-cycle-allowance',type=int,choices=(0,1),default=0,
                   help='Record but separately judge first startup cycle; --cycles 501 gives one startup plus 500 steady cycles')
    p.add_argument('--request-window',type=int,choices=(1,2,3),default=3,
                   help='Maximum outstanding telemetry/STOP requests per bus')
    p.add_argument('--request-gap-us',type=int,default=600,metavar='600..5000',
                   help='Minimum gap after each write, in microseconds')
    p.add_argument('--timer-slack-ns',type=int,choices=thread_timer_slack.CHOICES_NS,
                   help='Opt-in per-thread Linux timer slack during three-worker collection only')
    p.add_argument('--main-thread-cpu',type=int,
                   help='Opt-in diagnostic: pin only the policy thread after I/O workers start; restore on exit')
    p.add_argument('--exclude-policy-cpu-from-workers',action='store_true',
                   help='Opt-in V3 STOP-proxy diagnostic: keep all three I/O workers off the pinned policy CPU; restore every worker mask')
    p.add_argument('--output-dispatch-trace',action='store_true',
                   help='Opt-in timestamps for output guard, submits and worker dispatch; collected in memory')
    p.add_argument('--inference-thread-cpu-trace',action='store_true',
                   help='Opt-in per-cycle main-thread CPU versus wall time for bounded 26-request STOP-proxy inference')
    p.add_argument('--absolute-epoch-cadence',action='store_true',
                   help='Diagnostic-only fixed 20 ms start slots; skip missed slots, never catch up in a burst')
    p.add_argument('--release-spin-us',type=int,choices=(200,500),
                   help='Opt-in native absolute wait with bounded final CPU spin; requires absolute-epoch cadence')
    p.add_argument('--defer-gc-during-cycles',action='store_true',
                   help='Opt-in bounded STOP-proxy comparison: defer automatic GC for at most 500 traced cycles')
    p.add_argument('--single-thread-math',action='store_true',
                   help='Opt in to OMP/OPENBLAS/MKL thread counts of 1 before NumPy/Torch import')
    p.add_argument('--require-pinned-fast-model',action='store_true',
                   help='Reject timing runs unless the SHA-pinned scalar C++ model is explicitly selected')
    p.add_argument('--setup-gc',choices=('before-warmup',),
                   help='Opt-in full garbage collection after UID/IMU startup and before policy warmup')
    p.add_argument('--pre-cycle-policy-warmup-calls',type=int,choices=range(10,101),metavar='10..100',
                   help='Opt-in bounded synthetic policy warmup after workers, before optional CPU pin and timed cycles')
    p.add_argument('--post-pin-policy-prime-calls',type=int,choices=range(1,101),metavar='1..100',
                   help='Opt-in synthetic model calls on reused observer input tensors after CPU pin, before timed cycles')
    p.add_argument('--record-storage',choices=('objects','encoded','trace'),default='objects',
                   help='Keep objects, encode rows, or copy into a preallocated native trace within each measured cycle')
    p.add_argument('--acquisition-only',action='store_true')
    p.add_argument('--apply-reviewed-accel-calibration',action='store_true',
                   help='Explicitly use the named acceleration review in the pinned gyro-bias document; default raw')
    p.add_argument('--accel-input-hypothesis',
                   help='SHA-pinned boxed input hypothesis for disabled STOP-proxy inference only')
    p.add_argument('--accel-input-hypothesis-sha256')
    p.add_argument('--active-fk-profile',help='Explicit file-only FK binding for supported STOP-only diagnosis; grants no active output qualification')
    p.add_argument('--active-fk-profile-sha256')
    p.add_argument('--checked-model-manifest',help='Explicit same-profile checked model file binding; grants no output')
    p.add_argument('--checked-model-manifest-sha256')
    p.add_argument('--diagnostic-bus-workers-output',action='store_true',
                   help='Default off: actual ordinary BusWorkers decoded STOP/output join on the original borrowed three-worker diagnostic pool')
    p.add_argument('--unpaired-output-future-notifications',action='store_true',
                   help='Default off: authenticated original-Future readiness hints only for the actual diagnostic BusWorkers output route')
    p.add_argument('--native-feedback-batch-decode',action='store_true',
                   help='Default off: pure six-feedback codec on the source-pinned genuine ordinary-unpaired disabled baseline; grants no Type1 output qualification')
    p.add_argument('--native-feedback-codec-selection')
    p.add_argument('--native-feedback-codec-selection-sha256')
    p.add_argument('--native-phase-pair',action='store_true',
                   help='Explicit bounded disabled-only pair for twelve output STOP requests; requires every best20 setting and externally pinned sources/library')
    p.add_argument('--native-future-notification-joins',action='store_true',
                   help='Default off: use the verified native pair library for original-Future readiness hints; retain diagnostic release waits and all original deadlines')
    p.add_argument('--native-pair-prime-before-cycles', action='store_true',
                   help='One separately retained STOP-only pair phase with a 20ms deadline before the measured cadence; requires --native-phase-pair')
    p.add_argument('--native-pair-active-library')
    p.add_argument('--native-pair-active-library-sha256')
    p.add_argument('--native-pair-source-inventory')
    p.add_argument('--native-pair-source-inventory-sha256')
    p.add_argument('--compare-feedback',action='store_true',
                   help='Separate Type17/STOP-Type2 comparison; no IMU or inference')
    for name in ('front-port','rear-port','expected-uids','library','output','calibration','mount','gyro-bias',
                 'bundle','native-policy-manifest','native-policy-manifest-sha256',
                 'view-cache-manifest','view-cache-manifest-sha256',
                 'scalar-step-manifest','scalar-step-manifest-sha256'):
        p.add_argument('--'+name)
    p.add_argument('--h-hypothesis',type=int,choices=(0,1),default=0)
    args=p.parse_args(argv)
    private_refs=(args.private_seven_request_library,args.private_seven_request_library_sha256,
                  args.private_seven_request_source_inventory,args.private_seven_request_source_inventory_sha256)
    if (args.private_seven_request_experiment is None and any(v is not None for v in private_refs) or
            args.private_seven_request_experiment is not None and not all(v is not None for v in private_refs)):
        p.error('Private library/source pins require explicit experiment and complete references')
    pair_references=(args.native_pair_active_library,args.native_pair_active_library_sha256,
                     args.native_pair_source_inventory,args.native_pair_source_inventory_sha256)
    if not args.native_phase_pair and any(value is not None for value in pair_references):
        p.error('Native pair library/source pins require explicit --native-phase-pair')
    if args.native_pair_prime_before_cycles and not args.native_phase_pair:
        p.error('--native-pair-prime-before-cycles requires explicit --native-phase-pair')
    if args.native_future_notification_joins and not args.native_phase_pair:
        p.error('--native-future-notification-joins requires explicit --native-phase-pair')
    if ((args.native_feedback_batch_decode or args.unpaired_output_future_notifications) and
            not args.diagnostic_bus_workers_output or
            args.diagnostic_bus_workers_output and (args.private_seven_request_experiment!='baseline6plus1' or
                args.native_phase_pair or args.native_future_notification_joins)):
        p.error('Diagnostic runtime output selection requires original unpaired baseline6plus1 and no native pair/split')
    codec_refs=(args.native_feedback_codec_selection,args.native_feedback_codec_selection_sha256)
    if (not args.native_feedback_batch_decode and any(value is not None for value in codec_refs) or
            args.native_feedback_batch_decode and not all(value is not None for value in codec_refs)):
        p.error('Explicit diagnostic feedback codec needs its complete selection reference; default carries none')
    if args.native_feedback_batch_decode and (args.private_seven_request_experiment!='baseline6plus1' or
            args.native_phase_pair or args.native_future_notification_joins):
        p.error('Diagnostic native feedback codec requires ordinary unpaired baseline6plus1, no split or pair')
    feedback_codec_selection=None
    if args.native_feedback_batch_decode:
        from .unpaired_native_feedback_codec import verify_source_selection
        try:
            from . import policy_live_profile as profiles
            feedback_codec_selection,_=profiles._read_json(args.native_feedback_codec_selection,
                digest=args.native_feedback_codec_selection_sha256)
            verify_source_selection(feedback_codec_selection)
            if feedback_codec_selection['references']['library']!={'path':str(Path(
                    args.private_seven_request_library).expanduser().absolute()),
                    'sha256':args.private_seven_request_library_sha256}:
                raise ValueError('Selected feedback codec differs from actual private ordinary active library')
        except (ValueError,OSError,KeyError,TypeError) as error:p.error(str(error))
    if bool(args.checked_model_manifest) != bool(args.checked_model_manifest_sha256):
        p.error('Checked model manifest path and SHA must be supplied together')
    if args.checked_model_manifest and not args.active_fk_profile:
        p.error('Checked model requires its own exact source-bound FK diagnostic profile')
    active_fk_context=None
    if args.active_fk_profile is not None or args.active_fk_profile_sha256 is not None:
        if not args.active_fk_profile or not args.active_fk_profile_sha256:
            p.error('--active-fk-profile and --active-fk-profile-sha256 must be supplied together')
        try:active_fk_context=_active_fk_diagnostic_context(args)
        except (ValueError,OSError,RuntimeError) as error:p.error(str(error))
    trace_copy_backend=None
    if args.retain_gil_trace_copy:
        if (args.native_boot_guard_artifact is not None or args.native_boot_guard_artifact_sha256 is not None):
            p.error('Trace-copy and native boot-guard experiments are mutually exclusive')
        if (args.mode!='stop-proxy' or not args.supported_disabled or not args.v3_voltage_proxy or
                args.record_storage!='trace' or args.acquisition_only or args.compare_feedback or
                not args.provenance_mode or not args.power_epoch):
            p.error('Retained-GIL trace copy requires source-pinned disabled V3 STOP-proxy inference/trace')
        try:trace_copy_backend=_retained_gil_trace_copy()
        except (ValueError,OSError,RuntimeError) as error:p.error(str(error))
    if args.owned_buffer_trace_copy:
        if (not args.native_phase_pair or args.retain_gil_trace_copy or
                args.native_boot_guard_artifact is not None or args.native_boot_guard_artifact_sha256 is not None):
            p.error('Owned-buffer trace copy requires explicit native pair and excludes other copy/guard experiments')
        try:trace_copy_backend=_OwnedBufferTraceCopy()
        except (ValueError,OSError,RuntimeError) as error:p.error(str(error))

    guard_reference = None
    guard_factory = None
    guard_plan = None
    if args.native_boot_guard_artifact is not None or args.native_boot_guard_artifact_sha256 is not None:
        digest = args.native_boot_guard_artifact_sha256
        if (not args.native_boot_guard_artifact or type(digest) is not str or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)):
            p.error('Native boot guard requires an artifact path and lowercase SHA256')
        if (args.mode != 'stop-proxy' or not args.supported_disabled or
                not args.v3_voltage_proxy or args.acquisition_only or args.compare_feedback or
                not args.provenance_mode or not args.power_epoch):
            p.error('Native boot guard is limited to source-pinned disabled V3 STOP-proxy inference')
        guard_reference = {'path':str(Path(args.native_boot_guard_artifact).expanduser().absolute()),
                           'sha256':digest}
        try:
            from .sourced_boot_guard import plan_sourced_boot_guard
            guard_plan = plan_sourced_boot_guard(guard_reference)
        except (ValueError, OSError, RuntimeError) as error:
            p.error(str(error))
    hypothesis_selected = (args.accel_input_hypothesis is not None or
                           args.accel_input_hypothesis_sha256 is not None)
    hypothesis_reference = None
    if hypothesis_selected:
        digest = args.accel_input_hypothesis_sha256
        if (not args.accel_input_hypothesis or type(digest) is not str or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)):
            p.error('Acceleration hypothesis requires a path and lowercase SHA256')
        if (args.apply_reviewed_accel_calibration or args.acquisition_only or
                args.compare_feedback or args.mode != 'stop-proxy' or
                not args.supported_disabled or not args.v3_voltage_proxy):
            p.error('Acceleration hypothesis requires disabled V3 STOP-proxy inference without other correction')
        hypothesis_reference = {'path':str(Path(args.accel_input_hypothesis).expanduser().absolute()),
                                'sha256':digest}
    if args.apply_reviewed_accel_calibration and (args.acquisition_only or args.compare_feedback or not args.gyro_bias):
        p.error('--apply-reviewed-accel-calibration requires full inference and --gyro-bias')
    try:math_startup=math_threads.configure_single_thread_math(args.single_thread_math)
    except math_threads.MathThreadStartupError as error:p.error(str(error))
    if not 600<=args.request_gap_us<=5000:
        p.error('--request-gap-us must be 600..5000')
    view_cache_selected=args.view_cache_manifest is not None or args.view_cache_manifest_sha256 is not None
    scalar_step_selected=args.scalar_step_manifest is not None or args.scalar_step_manifest_sha256 is not None
    if args.require_pinned_fast_model and not (scalar_step_selected and
            args.native_policy_manifest and args.native_policy_manifest_sha256 and
            args.mode=='stop-proxy' and not args.acquisition_only and not args.compare_feedback):
        p.error('--require-pinned-fast-model needs a full STOP-proxy inference run with pinned scalar and baseline manifests')
    if scalar_step_selected:
        if not args.scalar_step_manifest or not args.scalar_step_manifest_sha256:
            p.error('--scalar-step-manifest and --scalar-step-manifest-sha256 must be supplied together')
        if view_cache_selected:
            p.error('Scalar-step and cached-view selections are mutually exclusive')
        if args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback:
            p.error('Scalar-step diagnostic requires STOP proxy with policy inference')
        if not args.native_policy_manifest or not args.native_policy_manifest_sha256:
            p.error('Scalar-step diagnostic requires the pinned native baseline')
    if view_cache_selected:
        if not args.view_cache_manifest or not args.view_cache_manifest_sha256:
            p.error('--view-cache-manifest and --view-cache-manifest-sha256 must be supplied together')
        if args.acquisition_only or args.compare_feedback:
            p.error('Cached-view diagnostic requires policy inference, without acquisition-only or feedback comparison')
        if not args.native_policy_manifest or not args.native_policy_manifest_sha256:
            p.error('Cached-view diagnostic requires --native-policy-manifest and --native-policy-manifest-sha256')
    if args.setup_gc is not None and (args.acquisition_only or args.compare_feedback):
        p.error('--setup-gc requires policy inference, without acquisition-only or feedback comparison')
    if args.record_storage!='objects' and args.compare_feedback:
        p.error('--record-storage encoded/trace requires the diagnostic collector without feedback comparison')
    if args.output_dispatch_trace and (args.mode!='stop-proxy' or args.acquisition_only or
                                       args.compare_feedback):
        p.error('--output-dispatch-trace requires STOP proxy with policy inference')
    if args.cycles is None:args.cycles=3 if args.compare_feedback else 20
    if args.startup_cycle_allowance and (args.mode!='stop-proxy' or args.acquisition_only or
            args.compare_feedback or not 2<=args.cycles<=501):
        p.error('--startup-cycle-allowance requires one startup and 1..500 STOP-proxy inference cycles')
    bounded_cycles=500+args.startup_cycle_allowance
    if scalar_step_selected and args.cycles>bounded_cycles:
        p.error('Scalar-step diagnostic permits at most 500 steady cycles')
    if args.release_spin_us is not None and not args.absolute_epoch_cadence:
        p.error('--release-spin-us requires --absolute-epoch-cadence')
    if args.pre_cycle_policy_warmup_calls is not None and (
            args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--pre-cycle-policy-warmup-calls requires at most 500 STOP-proxy cycles with policy inference')
    if args.post_pin_policy_prime_calls is not None and (
            args.pre_cycle_policy_warmup_calls is None or args.main_thread_cpu is None or
            args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--post-pin-policy-prime-calls requires pre-cycle policy warmup, a pinned policy CPU and at most 500 STOP-proxy cycles')
    if args.defer_gc_during_cycles and (args.mode!='stop-proxy' or args.acquisition_only or
                                        args.compare_feedback or args.cycles>bounded_cycles or
                                        args.record_storage!='trace' or not args.output_dispatch_trace):
        p.error('--defer-gc-during-cycles requires at most 500 STOP-proxy cycles, trace storage and --output-dispatch-trace')
    if args.compare_feedback and (args.mode!='stop-proxy' or not args.acquisition_only):
        p.error('Feedback comparison requires --mode stop-proxy --acquisition-only')
    if args.compare_feedback and args.cycles>5:
        p.error('Feedback comparison uses 1..5 cycles')
    if args.compare_feedback and args.timer_slack_ns is not None:
        p.error('--timer-slack-ns requires the three-worker diagnostic collector')
    if args.v3_voltage_proxy and (args.mode!='stop-proxy' or args.acquisition_only or
                                  args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--v3-voltage-proxy requires at most 500 full STOP-proxy inference cycles')
    if args.v3_voltage_overlap and (not args.v3_voltage_proxy or args.mode!='stop-proxy' or
                                    args.acquisition_only or args.compare_feedback or
                                    args.cycles>bounded_cycles or args.record_storage!='trace'):
        p.error('--v3-voltage-overlap requires --v3-voltage-proxy, trace storage and at most 500 full STOP-proxy inference cycles')
    if args.v3_voltage_validation_overlap and not args.v3_voltage_overlap:
        p.error('--v3-voltage-validation-overlap requires --v3-voltage-overlap')
    if args.v3_voltage_pipeline and not (args.v3_voltage_proxy and args.v3_voltage_overlap and
                                         args.v3_voltage_validation_overlap and
                                         args.mode=='stop-proxy' and not args.acquisition_only and
                                         not args.compare_feedback and args.cycles<=bounded_cycles and
                                         args.record_storage=='trace'):
        p.error('--v3-voltage-pipeline requires bounded V3 STOP-proxy trace with --v3-voltage-overlap and --v3-voltage-validation-overlap')
    if args.v3_voltage_fast_pipeline and not (args.v3_voltage_proxy and args.v3_voltage_overlap and
                                              args.v3_voltage_validation_overlap and
                                              args.mode=='stop-proxy' and not args.acquisition_only and
                                              not args.compare_feedback and args.cycles<=bounded_cycles and
                                              args.record_storage=='trace' and not args.v3_voltage_pipeline):
        p.error('--v3-voltage-fast-pipeline requires bounded V3 STOP-proxy trace with overlap, and excludes gated pipeline')
    if args.prepare_voltage_before_feedback_publication and not (
            args.v3_voltage_fast_pipeline and args.supported_disabled and
            args.provenance_mode and args.power_epoch and
            not args.native_boot_guard_artifact and not args.retain_gil_trace_copy):
        p.error('--prepare-voltage-before-feedback-publication requires the disabled fast voltage path and explicit source/power provenance, without experimental runtimes')
    if args.inference_thread_cpu_trace and not args.v3_voltage_proxy:
        p.error('--inference-thread-cpu-trace requires bounded 26-request STOP-proxy inference with --v3-voltage-proxy')
    if args.absolute_epoch_cadence and (args.mode!='stop-proxy' or args.acquisition_only or
                                        args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--absolute-epoch-cadence requires at most 500 full STOP-proxy inference cycles')
    if args.main_thread_cpu is not None:
        if args.main_thread_cpu<0 or args.compare_feedback or args.acquisition_only:
            p.error('--main-thread-cpu requires a nonnegative CPU and policy inference')
        if not hasattr(os,'sched_getaffinity') or not hasattr(os,'sched_setaffinity'):
            p.error('--main-thread-cpu requires Linux thread affinity')
        if args.main_thread_cpu not in os.sched_getaffinity(0):
            p.error('--main-thread-cpu is outside the current CPU affinity')
    if args.exclude_policy_cpu_from_workers:
        if (not args.v3_voltage_proxy or args.mode!='stop-proxy' or args.acquisition_only or
                args.compare_feedback or args.cycles>bounded_cycles or args.main_thread_cpu is None):
            p.error('--exclude-policy-cpu-from-workers requires at most 500 V3 STOP-proxy inference cycles and --main-thread-cpu')
        if len(set(os.sched_getaffinity(0))-{args.main_thread_cpu})<3:
            p.error('--exclude-policy-cpu-from-workers requires at least three other available CPUs')
    private_seven_request_plan=None
    if args.private_seven_request_experiment is not None:
        from .private_seven_request_bridge import prepare_plan
        try:private_seven_request_plan=prepare_plan(args,active_fk_context)
        except (ValueError,OSError,RuntimeError) as error:p.error(str(error))
    native_pair_plan=None
    if args.native_phase_pair:
        try:native_pair_plan=_native_pair_diagnostic_plan(args,active_fk_context)
        except (ValueError,OSError,RuntimeError) as error:p.error(str(error))
    try:source_provenance=_start_source_provenance(args.provenance_mode,args.power_epoch,
                accel_input_hypothesis=hypothesis_selected,native_target_fk_cache=active_fk_context is not None,
                native_phase_pair=args.native_phase_pair,
                native_feedback_batch_decode=args.native_feedback_batch_decode,
                native_checked_policy_dispatch=bool(args.checked_model_manifest),
                unpaired_output_future_notifications=args.unpaired_output_future_notifications)
    except (ValueError,OSError) as error:p.error(str(error))
    if active_fk_context is not None and source_provenance['cadence_source_sha256']!=active_fk_context['profile']['cadence_source_sha256']:
        p.error('Active FK diagnostic current sources differ from its profile')
    plan={'diagnostic_bus_workers_output':args.diagnostic_bus_workers_output,
          'unpaired_output_future_notifications':args.unpaired_output_future_notifications,
          'native_feedback_batch_decode_selected':args.native_feedback_batch_decode,
          'native_feedback_codec_selection':feedback_codec_selection,
          'mode':args.mode,'cycles':args.cycles,'gap_ms':args.request_gap_us/1000,
          'startup_cycle_allowance':args.startup_cycle_allowance,
          'steady_cycles_requested':args.cycles-args.startup_cycle_allowance,
          'release_spin_us':args.release_spin_us,
          'v3_voltage_proxy':args.v3_voltage_proxy,
          'voltage_max_v':args.voltage_max_v,'voltage_range_v':[35.,args.voltage_max_v],
          'v3_voltage_pipeline':args.v3_voltage_pipeline,
          'v3_voltage_fast_pipeline':args.v3_voltage_fast_pipeline,
          'absolute_epoch_cadence':args.absolute_epoch_cadence,
          'absolute_epoch_min_start_separation_ms':(
              ABSOLUTE_MIN_START_SEPARATION_NS/1e6 if args.absolute_epoch_cadence else None),
          'requests_per_cycle':26 if args.v3_voltage_proxy else 24,
          'type1_requests_per_cycle':0,
          'request_gap_us':args.request_gap_us,'window':args.request_window,
          'timer_slack_ns':args.timer_slack_ns,
          'math_thread_startup':math_startup,
          'main_thread_cpu':args.main_thread_cpu,
          'exclude_policy_cpu_from_workers':args.exclude_policy_cpu_from_workers,
          'setup_gc':args.setup_gc,
          'record_storage':args.record_storage,
          'view_cache_variant':view_cache_selected,'view_cache_diagnostic_only':view_cache_selected,
          'policy_backend_requested':('pinned_active_fk_cache' if active_fk_context is not None else
                                      'pinned_scalar_cpp' if scalar_step_selected else
                                      'pinned_cached_view' if view_cache_selected else
                                      'pinned_native_baseline' if args.native_policy_manifest else
                                      'reference_bundle'),
          'require_pinned_fast_model':args.require_pinned_fast_model,
          'view_cache_manifest':args.view_cache_manifest,
          'view_cache_manifest_sha256':args.view_cache_manifest_sha256,
          'scalar_step_manifest':args.scalar_step_manifest,
          'scalar_step_manifest_sha256':args.scalar_step_manifest_sha256,
          'reused_policy_input_buffers':not args.acquisition_only,
          'startup_identity_batch_size':6 if args.compare_feedback else 1,
          'startup_identity_window':1,'startup_identity_retry':False,
          'compare_feedback':args.compare_feedback,
          'acquisition_only':args.acquisition_only,'enable_available':False,'learned_targets_sent':False,
          'apply_reviewed_accel_calibration':args.apply_reviewed_accel_calibration,
          'state_changing_stop':args.mode=='stop-proxy',
          'input_workers':(['front6+voltage1','rear6+voltage1','IMU'] if args.v3_voltage_overlap
                           else ['front7','rear7','IMU'] if args.v3_voltage_proxy
                           else ['front6','rear6','IMU']),
          'disk_io_during_cycles':False,'full_controller_50Hz_verified':False}
    if hypothesis_reference is not None:
        plan['accel_input_hypothesis'] = hypothesis_reference
    if active_fk_context is not None:
        plan.update(native_target_fk_cache=True,active_fk_profile=active_fk_context['reference'],
            target_fk_manifest=active_fk_context['profile']['artifacts']['target_fk_manifest'],
            active_fk_file_plan=active_fk_context['proof'],active_controller_qualification=False,
            diagnostic_local_reference_branch=active_fk_context['local_branch_provenance'])
    if args.checked_model_manifest:
        plan.update(native_checked_policy_dispatch=True,
            checked_model_manifest=active_fk_context['profile']['artifacts']['checked_model_manifest'],
            checked_model_plan=active_fk_context['profile']['_checked_model_plan'])
    if args.prepare_voltage_before_feedback_publication:
        plan['prepare_voltage_before_feedback_publication']=True
    if native_pair_plan is not None:
        plan.update(native_phase_pair=True,native_phase_pair_plan=native_pair_plan,
                    request_window=args.request_window,output_allowed=False,approved_for_runtime=False)
        if args.native_future_notification_joins:
            plan['native_future_notification_joins']=True
        if args.native_pair_prime_before_cycles:plan['native_pair_prime_before_cycles']=True
    if private_seven_request_plan is not None:
        plan['private_seven_request_experiment']=private_seven_request_plan
        plan['native_phase_pair']=False
        plan['input_workers']=['front_unpaired_owner','rear_unpaired_owner','IMU']
    if guard_plan is not None:
        plan['sourced_boot_guard'] = guard_plan
    if trace_copy_backend is not None:
        plan['trace_copy_provenance']=trace_copy_backend.provenance()
    if source_provenance is not None:
        plan['source_provenance']=source_provenance
    if args.v3_voltage_overlap:plan['v3_voltage_overlap']=True
    if args.v3_voltage_validation_overlap:plan['v3_voltage_validation_overlap']=True
    if args.output_dispatch_trace:plan['output_dispatch_trace']=True
    if args.inference_thread_cpu_trace:plan['inference_thread_cpu_trace']=True
    if args.defer_gc_during_cycles:plan['defer_gc_during_cycles']=True
    if args.pre_cycle_policy_warmup_calls is not None:
        plan['pre_cycle_policy_warmup_calls']=args.pre_cycle_policy_warmup_calls
    if args.post_pin_policy_prime_calls is not None:
        plan['post_pin_policy_prime_calls']=args.post_pin_policy_prime_calls
    if not args.execute:
        print(json.dumps(plan,indent=2));return 0
    required=['front_port','rear_port','expected_uids','library','output']
    if not args.acquisition_only:required+=['calibration','mount','bundle']
    if any(not getattr(args,k) for k in required):p.error('Missing execution paths: '+','.join(required))
    if args.mode=='stop-proxy' and not args.supported_disabled:
        p.error('STOP proxy requires independently supported, already disabled robot')
    if args.timer_slack_ns is not None:
        try:thread_timer_slack.require_supported_platform()
        except thread_timer_slack.TimerSlackError as error:p.error(str(error))
    out=Path(args.output).expanduser().resolve()
    if any((parent/'.git').exists() for parent in (out,*out.parents)):
        p.error('Raw diagnostic records must be saved outside Git')
    out.mkdir(mode=0o700,parents=True,exist_ok=False)
    timer_slack=thread_timer_slack.TimerSlack(args.timer_slack_ns)
    setup_gc={'mode':args.setup_gc,'scope':'setup_only',
              'position':'after_identity_and_imu_start_before_warmup' if args.setup_gc is not None else None,
              'generation':2 if args.setup_gc is not None else None,'attempted':False,'complete':False,
              'begin_ns':None,'end_ns':None,'duration_ms':None,'collected_objects':None,
              'changes_gc_settings':False}
    report={'status':'ABORTED','plan':plan,'math_thread_startup':math_startup,
            'timer_slack':timer_slack.report,
            'setup_gc':setup_gc,'errors':[]};saved=[];device=None
    if active_fk_context is not None:
        report.update(native_target_fk_cache=True,active_controller_qualification=False)
    if args.checked_model_manifest:
        report.update(native_checked_policy_dispatch=True,checked_model_plan=active_fk_context['profile']['_checked_model_plan'])
    if args.prepare_voltage_before_feedback_publication:
        report['prepare_voltage_before_feedback_publication']=True
    if native_pair_plan is not None:
        report.update(native_phase_pair=True,native_phase_pair_plan=native_pair_plan,
                      request_gap_us=args.request_gap_us,request_window=args.request_window,
                      active_controller_qualification=False,output_allowed=False)
    if trace_copy_backend is not None:
        report['trace_copy_provenance']=trace_copy_backend.provenance()
    if source_provenance is not None:
        report.update(motor_power_epoch=source_provenance['motor_power_epoch'],
                      cadence_source_sha256=source_provenance['cadence_source_sha256'],
                      source_provenance=source_provenance)
    calibration=None
    native_pair_candidate=None;private_seven_request_runtime=None
    cr,cw=os.pipe();handlers={};cancelled=[]
    def cancel(signum,frame):
        cancelled.append(signum)
        if len(cancelled)==1:os.write(cw,b'x')
    try:
        if guard_reference is not None:
            from .sourced_boot_guard import load_sourced_boot_guard_factory
            guard_factory = load_sourced_boot_guard_factory(guard_reference)
            report['sourced_boot_guard'] = guard_factory.provenance()
        report['input_sha256']={k:hashlib.sha256(Path(getattr(args,k)).read_bytes()).hexdigest()
            for k in ('expected_uids','calibration','mount','gyro_bias') if getattr(args,k)}
        if active_fk_context is not None:
            report['input_sha256']['target_fk_manifest']=active_fk_context['profile']['artifacts']['target_fk_manifest']['sha256']
            report['input_sha256']['local_reference_capture']=active_fk_context['local_branch_provenance']['capture']['sha256']
            report['diagnostic_local_reference_branch']=active_fk_context['local_branch_provenance']
        if hypothesis_reference is not None:
            actual = hashlib.sha256(Path(hypothesis_reference['path']).read_bytes()).hexdigest()
            if actual != hypothesis_reference['sha256']:
                raise ValueError('Acceleration input hypothesis SHA256 mismatch')
            report['input_sha256']['accel_input_hypothesis'] = actual
        lib=native.load_library(args.library)
        if native_pair_plan is not None:
            from . import native_active_transport as active
            verified_active_library=active.load_library(native_pair_plan['active_library']['path'],
                expected_sha256=native_pair_plan['active_library']['sha256'])
            if args.native_future_notification_joins:
                _require_future_notification_library(verified_active_library)
        if private_seven_request_plan is not None:
            from .private_seven_request_candidate import load_candidate
            verified_private_library=load_candidate(private_seven_request_plan['library']['path'])
        uids=dual.pipeline.validate_uids(shadow._json(Path(args.expected_uids).read_bytes()))
        run=None
        if not args.acquisition_only:
            calibration=shadow._json(Path(args.calibration).read_bytes())
            if calibration['identities']!={str(i):uids[i] for i in range(1,13)}:
                raise ValueError('Calibration UID binding mismatch')
            if active_fk_context is not None:
                if report['input_sha256']['calibration']!=active_fk_context['profile']['artifacts']['calibration']['sha256']:
                    raise ValueError('Active FK diagnostic calibration changed during load')
                calibration=active_fk_context['observer_calibration']
            mount=shadow._json(Path(args.mount).read_bytes())
            bias_raw=Path(args.gyro_bias).read_bytes() if args.gyro_bias else None
            if bias_raw is not None and hashlib.sha256(bias_raw).hexdigest()!=report['input_sha256']['gyro_bias']:
                raise ValueError('Gyro/acceleration bias artifact changed during diagnostic load')
            bias=shadow._json(bias_raw) if bias_raw is not None else None
            if args.single_thread_math:
                math_startup['before_torch_import_env']=math_threads.verify_before_math_import()
            else:
                math_startup['before_torch_import_env']=math_threads.effective_math_thread_env()
            import torch
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
            if active_fk_context is not None:
                from . import policy_active_fk
                policy,source=policy_active_fk.diagnostic_load(active_fk_context['profile'])
                report['scalar_step_model_source']=source['original_scalar_dependency']
                report['native_baseline_model_source']=source['baseline_provenance']
            elif args.native_policy_manifest:
                sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
                if scalar_step_selected:
                    from native_policy_overnight.model_call_fastpath.scalar_loader import load_file_only_verified
                    policy,source=load_file_only_verified(args.scalar_step_manifest,
                        expected_sha256=args.scalar_step_manifest_sha256,
                        baseline_manifest=args.native_policy_manifest,
                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['scalar_step_model_source']=source
                    report['native_baseline_model_source']=source['baseline_provenance']
                elif view_cache_selected:
                    from native_policy_overnight.view_cache.loader import load_file_only_verified
                    policy,source=load_file_only_verified(args.view_cache_manifest,
                        expected_sha256=args.view_cache_manifest_sha256,
                        baseline_manifest=args.native_policy_manifest,
                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['view_cache_model_source']=source
                    report['native_baseline_model_source']=source['baseline_provenance']
                else:
                    from native_policy_overnight import load_verified
                    policy,source=load_verified(args.native_policy_manifest,
                        expected_manifest_sha256=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['native_baseline_model_source']=source
            else:policy,source=shadow.load_policy(args.bundle)
            checked_wrapper=None
            if args.checked_model_manifest:
                from . import policy_checked_dispatch as checked
                policy,checked_wrapper,source=checked.load(active_fk_context['profile'],policy,source,active=False)
            report['model_source']=source
            run=observer.StatefulPolicyObserver(policy,calibration,imu_mount_candidate=mount,
                h_hypothesis=args.h_hypothesis,command=[0.,0.,0.],max_ticks=args.cycles,
                max_age_ns=LIMIT_NS,max_spread_ns=LIMIT_NS,torch_module=torch,
                gyro_bias_candidate=bias,profile_consume=True,measured_diagnostic_ticks=True,
                reuse_input_buffers=True,apply_reviewed_accel_calibration=args.apply_reviewed_accel_calibration,
                accel_input_hypothesis=hypothesis_reference,checked_dispatch_wrapper=checked_wrapper)
            if active_fk_context is not None:
                run=_ActiveFKDiagnosticObserver(run,active_fk_context)
        bindings=dual.validate_ports(args.front_port,args.rear_port)
        with ExitStack() as stack:
            candidate_slot={'value':None}
            def enter_owned_context(context):
                if native_pair_plan is None and private_seven_request_plan is None:
                    return stack.enter_context(context)
                context.__enter__()
                def release_context():
                    candidate=candidate_slot['value']
                    if candidate is not None and not candidate.fd_release_safe():
                        _UNRELEASED_DISABLED_PAIR_RESOURCES.append(context)
                        report['status']='ABORTED'
                        report['errors'].append('Retained process/port ownership: native owners not safely released')
                        return
                    context.__exit__(None,None,None)
                stack.callback(release_context)
            enter_owned_context(dual.pipeline.ownership_locks())
            if not args.compare_feedback:enter_owned_context(live.imu_ownership_lock())
            guard=guard_factory() if guard_factory is not None else dual.BootIdentityGuard()
            stack.callback(guard.close)
            report['boot_id']=guard.boot_id
            if active_fk_context is not None and active_fk_context['profile']['boot_id']!=guard.boot_id:
                raise ValueError('Active FK diagnostic profile must match the current boot')
            if calibration is not None and calibration.get('source_current_boot_id')!=guard.boot_id:
                raise ValueError('Capture-bound calibration must match the current Jetson boot; run capture again')
            def check():
                if cancelled:raise InterruptedError('Signal cancellation')
                guard.check()
            for sig in (signal.SIGINT,signal.SIGTERM):handlers[sig]=signal.signal(sig,cancel)
            import serial
            sessions={}
            pair_fds={};pair_boot_fds={}
            def guarded_transport_close(callback,*arguments):
                candidate=candidate_slot['value']
                if candidate is not None and not candidate.fd_release_safe():
                    report['status']='ABORTED'
                    report['errors'].append('Retained borrowed transport FD: native owners not safely released')
                    _UNRELEASED_DISABLED_PAIR_RESOURCES.append((callback,arguments,candidate))
                    return
                callback(*arguments)
            for scope,binding in bindings.items():
                enter_owned_context(dual.port_lock(binding['resolved']))
                port=serial.Serial(port=None,baudrate=921600,timeout=0,write_timeout=.1,exclusive=True)
                port.dtr=port.rts=False;port.port=binding['path'];port.open()
                stack.callback(guarded_transport_close,port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev!=binding['st_rdev']:
                    raise ValueError('Port binding changed')
                # A separate read-only proc fd per worker; not a shared Python guard lock.
                boot_fd=os.open('/proc/sys/kernel/random/boot_id',os.O_RDONLY|os.O_CLOEXEC)
                stack.callback(guarded_transport_close,os.close,boot_fd)
                if native_pair_plan is not None or private_seven_request_plan is not None:
                    pair_fds[scope]=port.fileno();pair_boot_fds[scope]=boot_fd
                else:
                    sessions[scope]=native.NativeSession(lib,port.fileno(),first_id=dual.SCOPES[scope][0],
                        cancel_fd=cr,boot_fd=boot_fd,boot_id=guard.boot_id,stop_proxy=args.mode=='stop-proxy',
                        gap_ns=args.request_gap_us*1000,window=args.request_window)
            if native_pair_plan is not None:
                native_pair_candidate=_DisabledNativePairCandidate(
                    library_path=native_pair_plan['active_library']['path'],
                    library_sha256=native_pair_plan['active_library']['sha256'],
                    source_sha256=native_pair_plan['source_sha256'],fd_by_scope=pair_fds,
                    boot_fd_by_scope=pair_boot_fds,cancel_fd=cr,boot_id=guard.boot_id,
                    motor_power_epoch=args.power_epoch,
                    request_gap_us=native_pair_plan['request_gap_us'])
                candidate_slot['value']=native_pair_candidate
                stack.callback(native_pair_candidate.close)
                sessions=native_pair_candidate.sessions
            if private_seven_request_plan is not None:
                from .private_seven_request_bridge import UnpairedRuntime
                private_seven_request_runtime=UnpairedRuntime(mode=args.private_seven_request_experiment,
                    library=verified_private_library,fd_by_scope=pair_fds,boot_fd_by_scope=pair_boot_fds,
                    cancel_fd=cr,boot_id=guard.boot_id,source_refs=private_seven_request_plan['source_refs'])
                candidate_slot['value']=private_seven_request_runtime
                stack.callback(private_seven_request_runtime.close)
                sessions=private_seven_request_runtime.sessions
            if args.compare_feedback:
                from .native_feedback_compare import collect_feedback_comparison
                result,saved=collect_feedback_comparison(sessions,uids,boot_id=guard.boot_id,
                    supported_disabled=args.supported_disabled,cycles=args.cycles,check=check)
                report.update(result)
            else:
                for scope,session in sessions.items():
                    captures=report.setdefault('identities',{}).setdefault(scope,[])
                    # Identity preflight is serialized; measured collection keeps the session window.
                    for mid in dual.SCOPES[scope]:
                        check()
                        capture=session.exchange([codec.read_request(mid)])
                        captures.append(native.exchange_evidence(*capture))
                        rows=native.records_as_events(capture[0],cycle=0)
                        if len(rows)!=1 or rows[0]['motor_id']!=mid or rows[0]['result']['mcu_uid_hex']!=uids[mid]:
                            raise ValueError('Fresh UID mismatch: ID'+str(mid))
                device=imu.ICM20948();stack.callback(device.close)
                report['imu_configuration']=device.start()
                prime=None
                if run is not None:
                    if args.setup_gc is not None:
                        import gc
                        setup_gc['begin_ns']=time.monotonic_ns();setup_gc['attempted']=True
                        try:
                            setup_gc['collected_objects']=gc.collect()
                            setup_gc['complete']=True
                        finally:
                            setup_gc['end_ns']=time.monotonic_ns()
                            setup_gc['duration_ms']=(setup_gc['end_ns']-setup_gc['begin_ns'])/1e6
                    pre_cycle_warmup=args.pre_cycle_policy_warmup_calls is not None
                    warmup={'position':('after_worker_startup_before_optional_main_thread_affinity' if pre_cycle_warmup else
                                        'after_identity_and_imu_start_before_worker_startup'),
                            'after_main_thread_affinity':False,
                            'iterations':args.pre_cycle_policy_warmup_calls if pre_cycle_warmup else 10,
                            'begin_ns':None,'end_ns':None,
                            'duration_ms':None,'complete':False}
                    report['setup_policy_warmup']=warmup
                    if args.post_pin_policy_prime_calls is not None:
                        prime={'position':'after_main_thread_affinity_before_timed_cycles',
                               'kind':'synthetic_model_calls_on_reused_input_tensors',
                               'iterations':args.post_pin_policy_prime_calls,
                               'begin_ns':None,'end_ns':None,'duration_ms':None,
                               'complete':False,'observer_reset_after':False,
                               'sensor_cycles':0,'stop_writes':0}
                        report['setup_policy_prime']=prime
                    def prepare_policy():
                        warmup['begin_ns']=time.monotonic_ns()
                        try:
                            replay.warmup_policy(policy,torch,args.h_hypothesis,warmup['iterations'],
                                                 checked_dispatch_wrapper=getattr(run,'_checked_dispatch_wrapper',None))
                            if prime is None:run.prepare_run(warmup_completed=True)
                            warmup['complete']=True
                        finally:
                            warmup['end_ns']=time.monotonic_ns()
                            warmup['duration_ms']=(warmup['end_ns']-warmup['begin_ns'])/1e6
                    def prime_policy():
                        prime['begin_ns']=time.monotonic_ns()
                        try:
                            replay.warmup_policy(policy,torch,args.h_hypothesis,prime['iterations'],
                                                 input_tensors=_reused_policy_input_tensors(run),
                                                 checked_dispatch_wrapper=getattr(run,'_checked_dispatch_wrapper',None))
                            run.prepare_run(warmup_completed=True)
                            prime['observer_reset_after']=True
                            prime['complete']=True
                        finally:
                            prime['end_ns']=time.monotonic_ns()
                            prime['duration_ms']=(prime['end_ns']-prime['begin_ns'])/1e6
                    if not pre_cycle_warmup:prepare_policy()
                with timer_slack:
                    options={'mode':args.mode,'cycles':args.cycles,'check':check,
                             'voltage_max_v':args.voltage_max_v}
                    if args.startup_cycle_allowance:
                        options['startup_cycle_allowance']=args.startup_cycle_allowance
                    if args.release_spin_us is not None:
                        options['deadline_wait']=_collection_deadline_wait(
                            lib,cr,args.release_spin_us)
                    if args.native_future_notification_joins:
                        options['native_future_notification_joins']=True
                    if args.v3_voltage_proxy:options['v3_voltage_proxy']=True
                    if args.v3_voltage_overlap:options['v3_voltage_overlap']=True
                    if args.v3_voltage_validation_overlap:
                        options['v3_voltage_validation_overlap']=True
                    if args.v3_voltage_pipeline:options['v3_voltage_pipeline']=True
                    if args.v3_voltage_fast_pipeline:options['v3_voltage_fast_pipeline']=True
                    if args.prepare_voltage_before_feedback_publication:
                        options['prepare_voltage_before_feedback_publication']=True
                    if args.absolute_epoch_cadence:options['absolute_epoch_cadence']=True
                    if args.timer_slack_ns is not None:options['worker_initializer']=timer_slack.worker_initializer
                    if args.main_thread_cpu is not None:options['main_thread_cpu']=args.main_thread_cpu
                    if args.exclude_policy_cpu_from_workers:
                        options['exclude_policy_cpu_from_workers']=True
                    if args.output_dispatch_trace:options['output_dispatch_trace']=True
                    if args.inference_thread_cpu_trace:options['inference_thread_cpu_trace']=True
                    if args.defer_gc_during_cycles:options['defer_gc_during_cycles']=True
                    if run is not None and pre_cycle_warmup:
                        options['pre_cycle_policy_prepare']=prepare_policy
                    if prime is not None:options['post_pin_policy_prepare']=prime_policy
                    if args.record_storage!='objects':options['record_storage']=args.record_storage
                    if trace_copy_backend is not None:options['trace_copy_backend']=trace_copy_backend
                    if native_pair_candidate is not None:
                        options['native_phase_pair_candidate']=native_pair_candidate
                        if args.native_pair_prime_before_cycles:
                            options['native_pair_prime_before_cycles']=True
                    if private_seven_request_runtime is not None:
                        options['private_seven_request_runtime']=private_seven_request_runtime
                    if args.native_feedback_batch_decode:
                        options['native_feedback_batch_decode']=True
                        options['native_feedback_codec_selection']=feedback_codec_selection
                    if args.diagnostic_bus_workers_output:
                        options['diagnostic_bus_workers_output']=True
                        options['unpaired_output_future_notifications']=args.unpaired_output_future_notifications
                        options['diagnostic_cancel_io']=lambda:os.write(cw,b'x')
                    result,saved=collect(sessions,device,run,**options)
                    report.update(result)
                    if result['status']=='COMPLETE_DIAGNOSTIC':timer_slack.verify_workers()
    except BaseException as error:
        report['status']='ABORTED';report.setdefault('errors',[]).append(type(error).__name__+': '+str(error))
        if isinstance(error,native.ExchangeError):report['startup_failure']=native.exchange_evidence(error.records,error.stats)
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        if ((native_pair_candidate is None or native_pair_candidate.fd_release_safe()) and
                (private_seven_request_runtime is None or private_seven_request_runtime.fd_release_safe())):
            os.close(cr);os.close(cw)
        else:
            report['status']='ABORTED'
            report['errors'].append('Retained cancellation FDs: native owners not safely released')
        report['imu_restore_status']=device.restore_status if device is not None else 'not_started'
        if device is not None and device.restore_status not in ('restored','not_needed'):
            report['status']='ABORTED';report['errors'].append('IMU restoration unconfirmed')
        _finish_source_provenance(report,source_provenance)
        _finish_active_fk_diagnostic(report,active_fk_context)
        if private_seven_request_runtime is not None:
            report['private_seven_request_experiment']=private_seven_request_runtime.evidence()
        if native_pair_candidate is not None:
            report['native_phase_pair_evidence']=native_pair_candidate.evidence()
        if native_pair_plan is not None:
            try:
                from . import policy_live_profile as profiles
                profiles._read_json(Path(native_pair_plan['source_inventory']['path']),
                    digest=native_pair_plan['source_inventory']['sha256'])
                library=Path(native_pair_plan['active_library']['path'])
                if hashlib.sha256(library.read_bytes()).hexdigest()!=native_pair_plan['active_library']['sha256']:
                    raise ValueError('Native pair library changed during diagnostic')
            except (ValueError,OSError) as error:
                report['status']='ABORTED';report['errors'].append(type(error).__name__+': '+str(error))
        if trace_copy_backend is not None:
            try:
                report['trace_copy_provenance'].update(trace_copy_backend.verify())
            except BaseException as error:
                report['status']='ABORTED'
                report['errors'].append(type(error).__name__+': '+str(error))
                report['trace_copy_provenance'].update(trace_copy_backend.provenance(),
                                                     source_files_unchanged=False)
        if guard_factory is not None:
            try:
                verified_guard = guard_factory.verify()
                report['sourced_boot_guard'].update(verified_guard,
                                                    files_unchanged_after_run=True)
            except BaseException as error:
                report['status']='ABORTED'
                report['errors'].append(type(error).__name__+': '+str(error))
                report['sourced_boot_guard']['files_unchanged_after_run']=False
        extra=[]
        if args.record_storage in ('encoded','trace'):
            values,failures=_encoded_records_for_output(saved,report.get('record_storage_failure'))
            if failures:
                report['status']='ABORTED'
                for failure in failures:
                    if failure['error'] not in report['errors']:report['errors'].append(failure['error'])
                report['record_storage_failure_artifact']='record-storage-failure.json'
                extra.append(('record-storage-failure.json',{
                    'schema':'native-diagnostic-record-storage-failure-v1','status':'ABORTED',
                    'failures':failures,'output_allowed':False,'approved_for_runtime':False}))
        else:values=saved if args.compare_feedback else _serialize(saved)
        for name,value in (*extra,('records.json',values),('report.json',report)):
            with os.fdopen(os.open(out/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as f:
                json.dump(value,f,allow_nan=False,ensure_ascii=False);f.write('\n')
            if name=='records.json' and (args.v3_voltage_pipeline or args.v3_voltage_fast_pipeline):
                # Bind the approval report to the exact on-disk evidence, not
                # an in-memory serialization that might differ by one byte.
                key='v3_voltage_pipeline' if args.v3_voltage_pipeline else 'v3_voltage_fast_pipeline'
                report.setdefault(key,{})['records_sha256']=(
                    hashlib.sha256((out/name).read_bytes()).hexdigest())
                if args.v3_voltage_fast_pipeline:
                    from . import policy_live_profile as profiles
                    contract = profiles._make_unpaired_codec_voltage_contract(report)
                    if contract is not None:
                        report['unpaired_codec_voltage_dispatch_contract'] = contract
        print(json.dumps({'status':report['status'],'output':str(out),'errors':report.get('errors',[]),
                          'distributions_ms':report.get('distributions_ms')},ensure_ascii=False))
    return 0 if report['status']=='COMPLETE_DIAGNOSTIC' else 2

if __name__=='__main__':raise SystemExit(main())
