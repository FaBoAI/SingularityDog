"""Default-PLAN four physical bus diagnostic; never enables or sends Type1.

Root/supervisor injects current source-bound topology, model, operating scopes
and caller-owned FDs. No legacy two-bus timing/profile approval is accepted.
"""
import argparse
from concurrent.futures import Future, ThreadPoolExecutor
import copy
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
import pickle
import sys
import threading
import time

from singularitydog_hw.policy_output_runtime import (feedback_sample, validate_measured,
    validate_imu_metadata, V3_VOLTAGE_MAX_AGE_NS)
from .transport_adapter import Batch, Group, ThreeAxisTransport, PORTS, IDS, GROUPS, PERIOD_NS

SCHEMA = 'singularitydog.four-bus-stop-proxy-pipeline.v1'
SCHEMA_30HZ = 'singularitydog.four-bus-stop-proxy-pipeline.v2'
_GRANTS = {'output_allowed': False, 'runtime_approved': False,
           'active_controller_qualification': False, 'timing_admission_eligible': False,
           'learned_targets_sent': False, 'motor_enable_sent': False,
           'positive_gain_sent': False}


class LinuxWorkerScope:
    """Explicit same-thread placement/slack only; no device or power writes."""
    def __init__(self, port, mask):
        if port not in (*PORTS, 'imu') or tuple(mask) not in ((0,), (1,), (2,), (3,), (0, 1, 2, 3)):
            raise ValueError('Explicit four-bus worker placement required')
        self.port, self.mask = port, tuple(mask)
        self.original = self.slack = self.tid = None

    def __enter__(self):
        if sys.platform != 'linux':
            raise RuntimeError('Real worker placement requires Linux; mocks must label their readbacks')
        from singularitydog_hw.thread_timer_slack import TimerSlack
        self.tid = threading.get_native_id()
        self.original = os.sched_getaffinity(0)
        try:
            os.sched_setaffinity(0, self.mask)
            if os.sched_getaffinity(0) != set(self.mask):
                raise RuntimeError('Worker CPU readback differs')
            self.slack = TimerSlack(1000)
            self.slack.__enter__()
            return {'native_tid': self.tid, 'cpu_mask': list(self.mask),
                    'timer_slack_ns': self.slack.report['parent']['during_ns'],
                    'file_only_mock_readback': False}
        except BaseException as primary:
            if self.slack is not None:
                try:
                    self.slack.__exit__(type(primary), primary, primary.__traceback__)
                except BaseException as cleanup:
                    primary.add_note('Timer slack rollback: '+str(cleanup))
            try:
                os.sched_setaffinity(0, self.original)
                if os.sched_getaffinity(0) != self.original:
                    raise RuntimeError('Exact original worker CPU rollback unconfirmed')
            except BaseException as cleanup:
                primary.add_note('Worker CPU rollback: '+str(cleanup))
            raise

    def __exit__(self, kind, value, trace):
        if threading.get_native_id() != self.tid:
            raise RuntimeError('Worker placement must restore on its actual owner thread')
        first = None
        try:
            self.slack.__exit__(kind, value, trace)
        except BaseException as error:
            first = error
        try:
            os.sched_setaffinity(0, self.original)
            if os.sched_getaffinity(0) != self.original:
                raise RuntimeError('Exact original worker CPU restoration unconfirmed')
        except BaseException as error:
            if first is None: first = error
            else: first.add_note('CPU restoration: '+str(error))
        if first is not None:
            if value is not None: value.add_note('Worker scope cleanup: '+str(first))
            else: raise first


def _real_operating_readback(value):
    if (not isinstance(value, dict) or value.get('main_cpu_mask') != [4] or
            value.get('nice') != -10 or value.get('timer_slack_ns') != 1000 or
            type(value.get('switch_interval_s')) is not float or
            abs(value['switch_interval_s'] - .0001) > 1e-12 or
            value.get('single_thread_math_verified') is not True or
            value.get('power_scope_verified') is not True or
            value.get('gc_deferred_during_cycles') is not True):
        raise ValueError('Current canonical main/CPU/C7/EMC/math/GC readback required before devices')


@dataclass(frozen=True)
class Config:
    groups: tuple
    profile: dict
    offsets: dict
    topology: dict
    cycles: int = 5
    combined_acquisition: bool = False
    boundary_current_checks: bool = False
    final_gate_input_identity: bool = False
    decode_once: bool = False
    control_clock: dict = None
    policy30_model: dict = None


def _control(config):
    from singularitydog_hw.policy_clock import DEFAULT_POLICY_CLOCK, validate_control_clock
    if config.control_clock is None:
        if config.policy30_model is not None:
            raise ValueError('30Hz model requires its explicit control clock')
        return DEFAULT_POLICY_CLOCK
    clock = validate_control_clock(config.control_clock)
    if clock.policy_hz == 30:
        from singularitydog_hw.policy_model_30hz import validate_plan
        validate_plan(config.policy30_model)
    elif config.policy30_model is not None:
        raise ValueError('30Hz model cannot use the legacy50Hz clock')
    return clock


def _verify_observer_clock(observer, control, *, allow_unadvertised):
    """Bind the actual observer cadence before any owner/device is created.

    Ordinary StatefulPolicyObserver explicitly advertises None for legacy
    50 Hz, or the immutable PolicyClock for 30 Hz. Older file-only injected
    mocks may omit that attribute; real execution cannot omit this proof.
    """
    from singularitydog_hw.policy_clock import DEFAULT_POLICY_CLOCK, PolicyClock
    absent = object()
    declared = getattr(observer, '_control_clock', absent)
    if declared is absent:
        if not allow_unadvertised:
            raise ValueError('Real diagnostic observer must advertise its control clock')
        return
    if declared is None:
        observer_clock = DEFAULT_POLICY_CLOCK
    elif type(declared) is PolicyClock:
        observer_clock = declared
    else:
        raise ValueError('Diagnostic observer control clock must be None or an immutable PolicyClock')
    if observer_clock != control:
        raise ValueError('Diagnostic observer and outer control clock differ')


def plan(config):
    if type(config) is not Config or type(config.cycles) is not int or not 1 <= config.cycles <= 501:
        raise ValueError('Explicit diagnostic Config with 1..501 cycles required')
    if type(config.combined_acquisition) is not bool:
        raise ValueError('Combined acquisition selection must be an exact bool')
    if type(config.boundary_current_checks) is not bool:
        raise ValueError('Boundary current-check selection must be an exact bool')
    if type(config.final_gate_input_identity) is not bool:
        raise ValueError('Final-gate input identity selection must be an exact bool')
    if type(config.decode_once) is not bool:
        raise ValueError('Decode-once selection must be an exact bool')
    control = _control(config)
    is30 = control.policy_hz == 30
    if (type(config.groups) is not tuple or len(config.groups) != 4 or
            tuple(group.port for group in config.groups) != PORTS or
            any(type(group) is not Group for group in config.groups) or
            {group.ids for group in config.groups} != set(GROUPS)):
        raise ValueError('Four distinct original same-half physical groups must cover twelve axes')
    if set(config.offsets) != set(IDS) or any(type(value) not in (int, float) or
            not math.isfinite(value) for value in config.offsets.values()):
        raise ValueError('Original finite all-axis model offsets required')
    profile = config.profile
    if not isinstance(profile, dict) or set(profile.get('axes', {})) != {str(mid) for mid in IDS}:
        raise ValueError('Original all-twelve-axis bounds required')
    for key in ('max_sample_age_ms',):
        if type(profile.get(key)) not in (int, float) or not 0 < profile[key] <= 20:
            raise ValueError('Original sample freshness must fit the absolute 20ms control deadline')
    from .topology import validate_topology
    topology = validate_topology(config.topology)
    if (not topology.get('motor_power_epoch') or
            any(tuple(topology['ids_by_port'][group.port]) != group.ids for group in config.groups)):
        raise ValueError('Current explicit power epoch and exact physical group binding required')
    value = {'schema': SCHEMA_30HZ if is30 else SCHEMA, 'status': 'PLAN', **_GRANTS, 'opens_devices': False,
            'cycles': config.cycles, 'request_gap_ns': 900_000, 'request_window': 3,
            'absolute_deadline_ns': control.cycle_budget_ns, 'per_cycle_requests': 28,
            'request_schedule': 'each_physical_3STOP_feedback_then_1voltage_then_3STOP_proxy',
            'physical_groups': {group.port: list(group.ids) for group in config.groups},
            'worker_topology': {'dedicated_can_owners': 4, 'imu_validation_workers': 1,
                                'main_cpu': 4, 'can_cpus': [0, 1, 2, 3], 'imu_mask': [0, 1, 2, 3]},
            'codec': 'original_python_parser_exact_three_or_one',
            'requires_new_four_bus_source_model_timing_proof': True}
    if config.combined_acquisition:
        value.update(schema=(SCHEMA_30HZ if is30 else
                     'singularitydog.four-bus-combined-acquisition-stop-proxy-pipeline.v1'),
                     combined_acquisition_selected=True,
                     request_schedule='each_physical_3STOP_plus_1voltage_one_native_exchange_then_3STOP_proxy',
                     acquisition_contract='original_four_requests_all_replies_and_owner_join_before_inference.v1',
                     codec='original_python_parser_exact_four_mixed_requests',
                     physical_acquisition_records_per_port=4, projected_feedback_records_per_port=3,
                     voltage_pending_during_inference=False)
    if config.boundary_current_checks:
        # The full boot/ancestor/port identity check runs once at each cycle
        # boundary instead of around every Future take. Native per-write FD and
        # boot guards, cancel FD, owner exceptions and the 20ms deadlines stay.
        value.update(boundary_current_checks_selected=True,
                     full_current_check_points=['cycle_start_before_submit', 'after_feedback_joins_before_inference',
                                                'after_final_gate_before_output'],
                     hot_path_check='cancel_flag_and_original_owner_exceptions')
    if config.final_gate_input_identity:
        # After inference the snapshot is not rebuilt. Every original raw byte
        # image is still re-verified/re-decoded, the IMU must equal its
        # pre-inference copy and the snapshot must equal its pre-inference copy.
        value.update(final_gate_input_identity_selected=True,
                     final_gate_snapshot_check='raw_bytes_redecoded_imu_and_snapshot_equal_pre_inference_copy')
    if config.decode_once:
        # Each exchange is decoded once by its owner into a read-only mapping of
        # frozen Type2 rows; later takeouts compare the raw byte images. Snapshot
        # frames are parsed from owned bytes, and plain-data copies use pickle.
        value.update(decode_once_selected=True,
                     takeout_check='raw_byte_images_compared_publication_rows_reused',
                     plain_data_copy='pickle_protocol5_roundtrip')
    if is30:
        value.update(control_clock=control.as_payload(), policy30_model=copy.deepcopy(config.policy30_model),
            input_to_stop_proxy_tx_budget_ns=PERIOD_NS,
            acquisition_budget_ns=PERIOD_NS, proxy_exchange_budget_ns=control.cycle_budget_ns,
            proxy_reply_budget_ns=control.cycle_budget_ns,
            proxy_write_deadline_abi_required=1, diagnostic_reply_budget_unchanged=False,
            diagnostic_type1_33ms_reply_qualification=False,
            absolute_schedule='epoch_plus_floor_index_1e9_over_30',
            extended_budget_scope='stop_proxy_replies_and_postprocessing_only_not_acquisition_or_host_writes',
            requires_fresh_30hz_model_and_current_hardware_measurement=True)
    return value


def _take(future, deadline, clock, check, *, deadline_label='Original 20ms'):
    if type(future) is not Future:
        raise ValueError('Genuine original Future required')
    check()
    # Existing owner exception wins over the coordinator's deadline.
    if future.done():
        result = future.result()
    else:
        remaining = deadline - clock()
        if remaining <= 0:
            raise TimeoutError(deadline_label+' Future join deadline')
        try:
            result = future.result(timeout=remaining / 1e9)
        except TimeoutError:
            check()  # A known peer failure retains precedence over a join timeout.
            raise
    check()
    if clock() >= deadline:
        raise TimeoutError(deadline_label+' Future takeout deadline')
    return result


def _proxy_host_writes(batch):
    """Original native write clocks, including partial failure evidence."""
    return [{'slot': slot, 'start_ns': row.start_ns, 'finish_ns': row.finish_ns,
             'written': row.written, 'reply_deadline_ns': row.deadline_ns}
            for slot, row in enumerate(batch)]


def _validate_proxy_host_writes(batch, host_deadline, reply_deadline):
    # Batch verification authenticates the same raw records and wire identities.
    # Every actual write needs the strict20ms before/after check, not merely a
    # coordinator dispatch timestamp or its last write's aggregate duration.
    rows = _proxy_host_writes(batch.records)
    if batch.write_deadline_ns != host_deadline or len(rows) != 3 or any(row['written'] != 17 or
            not 0 < row['start_ns'] <= row['finish_ns'] < host_deadline or
            row['reply_deadline_ns'] != reply_deadline for row in rows):
        raise TimeoutError('30Hz proxy host-write exceeded preserved20ms deadline or raw write proof differs')
    return rows


def _rows(feedback, adapters, *, combined_acquisition=False):
    rows = {}
    if set(feedback) != set(PORTS):
        raise ValueError('All four original feedback batches required')
    for port in PORTS:
        if combined_acquisition:
            voltage_keys = [key for key in feedback[port].rows if key[1] == 'voltage']
            if len(voltage_keys) != 1:
                raise ValueError('Original combined batch must contain exactly one voltage reply')
            all_rows = adapters[port].verify_combined_batch(feedback[port], voltage_keys[0][0])
            decoded = {key: value for key, value in all_rows.items() if key[1] == 'feedback'}
        else:
            decoded = adapters[port].verify_batch(feedback[port], 'feedback')
        if set(decoded) != {(mid, 'feedback') for mid in adapters[port].group.ids}:
            raise ValueError('Exact-three feedback rows required')
        if rows.keys() & decoded.keys():
            raise ValueError('Duplicate physical feedback')
        rows.update(decoded)
    return rows


def _voltage(batch, adapter, mid, now, label):
    if label == 'acquisition_combined4':
        rows = adapter.verify_combined_batch(batch, mid)
        voltage_keys = {key for key in rows if key[1] == 'voltage'}
    else:
        rows = adapter.verify_batch(batch, label)
        voltage_keys = set(rows)
    if voltage_keys != {(mid, 'voltage')}:
        raise ValueError('Current rotating same-group voltage required')
    value, begin, end = rows[mid, 'voltage']
    voltage = value.get('value')
    if (type(voltage) not in (int, float) or not math.isfinite(voltage) or
            not 35. <= voltage <= 42. or not 0 < begin <= end <= now or
            now-begin > V3_VOLTAGE_MAX_AGE_NS):
        raise ValueError('Original voltage range or freshness failed')
    return voltage, begin, end


def _fresh_cache(cache, now):
    if set(cache) != set(IDS):
        raise ValueError('Fresh all-twelve-axis voltage cache required')
    for mid, (value, begin, end) in cache.items():
        if not 35. <= value <= 42. or not 0 < begin <= end <= now or now-begin > V3_VOLTAGE_MAX_AGE_NS:
            raise ValueError('ID%d original voltage range/freshness failed' % mid)


def _final_gate(config, adapters, feedback, voltage, expected_voltage, imu, cache,
                snapshot, snapshot_builder, initial, previous, previous_imu, now, *, recorded_imu=None,
                release_ns=None):
    rows = _rows(feedback, adapters, combined_acquisition=config.combined_acquisition)
    sample = feedback_sample(rows, config.profile, config.offsets, now_ns=now,
                             previous=previous, required_mode=0)
    validate_measured(sample, config.profile, initial=initial)
    validate_imu_metadata(imu, now, config.profile, previous=previous_imu)
    for port in PORTS:
        _voltage(voltage[port], adapters[port], expected_voltage[port], now,
                 'acquisition_combined4' if config.combined_acquisition else 'voltage')
    _fresh_cache(cache, now)
    if recorded_imu is not None:
        # Raw feedback/voltage bytes were re-verified by _rows/_voltage above;
        # the builder is a pure function of those bytes, this IMU and the tick.
        if imu != recorded_imu:
            raise ValueError('Original IMU image changed after inference')
    elif snapshot_builder(feedback, imu, snapshot['tick_ns']) != snapshot:
        raise ValueError('Original sensor/snapshot image changed after inference')
    first = min(imu['read_started_monotonic_ns'],
                *(row.start_ns for batch in feedback.values() for row in batch.records),
                *(row.start_ns for batch in voltage.values() for row in batch.records))
    if config.control_clock is not None and _control(config).policy_hz == 30:
        if type(release_ns) is not int or release_ns <= 0:
            raise ValueError('30Hz proxy requires actual release clock')
        # A longer cadence never expands the release/input-to-write allowance.
        first = min(first, release_ns)
    return rows, min(first + PERIOD_NS, snapshot['_cycle_deadline_ns'])


def run(config, *, factory=None, imu_read=None, observer=None, snapshot_builder=None,
        check_current=None, cancel_io=None, model_setup=None, worker_scope=None,
        main_scope=None, backend_usage=None, release_wait=None, execute=False,
        check_cancelled=None, clock=time.monotonic_ns, control_clock=None, policy30_model=None):
    """All injected capabilities stay explicit; PLAN calls none of them.

    ``worker_scope(port,cpu_mask)`` and ``main_scope()`` are context managers
    responsible for real readback/restoration (or explicitly file-only mocks).
    ``model_setup(observer, pre_calls=10, post_calls=10)`` must warm the selected
    actual wrapper, reset its original state and return truthful provenance.
    """
    if control_clock is not None or policy30_model is not None:
        if type(config) is not Config:
            raise ValueError('Explicit diagnostic Config required')
        if (config.control_clock is not None and config.control_clock != control_clock or
                config.policy30_model is not None and config.policy30_model != policy30_model):
            raise ValueError('Run and Config30Hz selections differ')
        config = replace(config, control_clock=control_clock, policy30_model=policy30_model)
    planned = plan(config)
    control = _control(config)
    is30 = control.policy_hz == 30
    if execute is not True:
        return planned
    needed = (factory, imu_read, snapshot_builder, check_current, cancel_io, model_setup,
              worker_scope, main_scope, release_wait)
    if any(not callable(value) for value in needed) or not callable(getattr(observer, 'consume', None)):
        raise ValueError('Explicit source/current/owner/model/restore capabilities required')
    if config.boundary_current_checks and not callable(check_cancelled):
        raise ValueError('Boundary current checks require an explicit cancellation check')
    if not isinstance(backend_usage, dict) or backend_usage.get('kind') not in (
            'genuine_original_active_subset_cpp', 'injected_file_only_mock'):
        raise ValueError('Truthful explicit backend usage required')
    allow_unadvertised_observer = backend_usage['kind'] == 'injected_file_only_mock'
    _verify_observer_clock(observer, control, allow_unadvertised=allow_unadvertised_observer)
    config = copy.deepcopy(config)
    # Same plain-data deep copy; pickle is several times cheaper than deepcopy.
    plain_copy = ((lambda value: pickle.loads(pickle.dumps(value, protocol=5)))
                  if config.decode_once else copy.deepcopy)
    report = {**planned, 'status': 'STARTING', 'opens_devices': True,
              'backend_usage': copy.deepcopy(backend_usage), 'records': [],
              'setup_voltage': {}, 'worker_settings': {}, 'restoration': {},
              'cleanup': {}, 'primary_error': None, 'completed_cycles': 0,
              'owner_settlement': []}
    adapters, pools, owner_scopes, started = {}, {}, {}, []
    prefix_proof = {}
    initial = previous = None
    previous_imu = 0
    cache = {}
    primary = None
    main_context = None
    main_entered = False

    def owners():
        # A known current owner error wins even when another port's Future is
        # still pending. Success readiness still needs its own original join.
        for owner in started:
            if owner.done():
                if owner.cancelled():
                    raise RuntimeError('Original owner Future was cancelled')
                error = owner.exception()
                if error is not None:
                    raise error

    def check():
        check_current()
        owners()

    def light():
        check_cancelled()
        owners()

    # Inside a cycle only the selected boundary branch uses the light check.
    hot = light if config.boundary_current_checks else check

    def initialize(port, mask):
        scope = worker_scope(port, mask)
        value = scope.__enter__()
        owner_scopes[port] = scope
        report['worker_settings'][port] = copy.deepcopy(value)
        return threading.get_native_id()

    def restore(port):
        scope = owner_scopes.pop(port, None)
        if scope is None:
            report['restoration'][port] = {'scope_not_entered': True}
            return
        scope.__exit__(None, None, None)
        report['restoration'][port] = True

    def acquire(port, mid, future, deadline):
        if config.combined_acquisition:
            result = adapters[port].acquire_combined(mid, future, deadline_ns=deadline, check=hot)
            prefix_proof[port] = result
            return result
        result = adapters[port].acquire(mid, future, deadline_ns=deadline, check=hot)
        # Future.done is not readiness proof: the original owner result must
        # identify the prefix that that owner actually produced this cycle.
        prefix_proof[port] = result[0]
        return result

    def cleanup_error(error):
        nonlocal primary
        report['status'] = 'ABORTED'
        if primary is None:
            primary = error
            report['primary_error'] = {'type': type(error).__name__, 'message': str(error)}
        else:
            primary.add_note('Cleanup failure: '+str(error))

    try:
        main_context = main_scope()
        operating = main_context.__enter__()
        main_entered = True
        report['operating_settings'] = copy.deepcopy(operating)
        if backend_usage['kind'] == 'genuine_original_active_subset_cpp':
            _real_operating_readback(operating)
        check()
        for group in config.groups:
            adapter = factory(group)
            adapters[group.port] = adapter  # Retain returned resource even if its protocol is rejected.
            if adapter.group != group or not callable(getattr(adapter, 'recover_subset', None)):
                raise ValueError('Exact-group transport plus subset recovery capability required')
            if (getattr(adapter, 'combined_acquisition', False) is not config.combined_acquisition or
                    config.combined_acquisition and not callable(getattr(adapter, 'acquire_combined', None))):
                raise ValueError('Explicit Config/transport combined acquisition selection must match')
            if getattr(adapter, 'decode_once', False) is not config.decode_once:
                raise ValueError('Explicit Config/transport decode-once selection must match')
            if getattr(adapter, 'separate_write_deadline', False) is not is30:
                raise ValueError('Explicit30Hz proxy transport requires separate host-write/reply deadlines')
            if backend_usage['kind'] == 'genuine_original_active_subset_cpp':
                if type(adapter) is not ThreeAxisTransport:
                    raise ValueError('Real backend requires genuine original exact-three adapter')
                adapter._verify()
            pools[group.port] = ThreadPoolExecutor(max_workers=1, thread_name_prefix='four-can-'+group.port)
        pools['imu'] = ThreadPoolExecutor(max_workers=1, thread_name_prefix='four-imu-validation')
        readbacks = []
        for index, port in enumerate(PORTS):
            readbacks.append(pools[port].submit(initialize, port, (index,)))
            started.append(readbacks[-1])
        readbacks.append(pools['imu'].submit(initialize, 'imu', (0, 1, 2, 3)))
        started.append(readbacks[-1])
        for future in readbacks:
            future.result(timeout=1)
        started = []
        if len(set(report['worker_settings'][port]['native_tid'] for port in (*PORTS, 'imu'))) != 5:
            raise ValueError('Five distinct persistent CAN/IMU workers required')
        if backend_usage['kind'] == 'genuine_original_active_subset_cpp':
            for index, port in enumerate((*PORTS, 'imu')):
                value = report['worker_settings'][port]
                expected_mask = [index] if index < 4 else [0, 1, 2, 3]
                if (value.get('cpu_mask') != expected_mask or value.get('timer_slack_ns') != 1000 or
                        value.get('file_only_mock_readback') is not False):
                    raise ValueError('Actual original five-worker placement/slack readback required')
        report['model_setup'] = model_setup(observer, pre_calls=10, post_calls=10)
        if (report['model_setup'].get('pre_calls') != 10 or report['model_setup'].get('post_calls') != 10 or
                report['model_setup'].get('reset_verified') is not True):
            raise ValueError('Original selected-model 10+10 warmup and reset required')
        if backend_usage['kind'] == 'genuine_original_active_subset_cpp' and (
                report['model_setup'].get('real_model_verified') is not True or
                report['model_setup'].get('selected_model_warmup_verified') is not True or
                not report['model_setup'].get('model_artifact_sha256') or
                not report['model_setup'].get('model_source_sha256')):
            raise ValueError('Current actual selected model/source/warmup binding required')
        # Setup-only twelve voltage reads are separately retained. They are
        # not added to the per-cycle 28 or substituted for current replies.
        def prime(port):
            return [adapters[port].read_voltage(mid, deadline_ns=clock()+100_000_000, check=check)
                    for mid in adapters[port].group.ids]
        setup = {}
        for port in PORTS:
            setup[port] = pools[port].submit(prime, port)
            started.append(setup[port])
        for port, future in setup.items():
            batches = future.result(timeout=.35)
            report['setup_voltage'][port] = batches
            for mid, batch in zip(adapters[port].group.ids, batches):
                cache[mid] = _voltage(batch, adapters[port], mid, clock(), 'setup_voltage')
        started = []
        epoch = clock()
        for cycle in range(config.cycles):
            scheduled = control.release_ns(epoch, cycle)
            release = release_wait(scheduled)
            if type(release) is not int or release < scheduled or release > clock():
                raise ValueError('Release waiter must return actual current monotonic time')
            deadline = control.cycle_deadline_ns(release)
            acquisition_deadline = min(deadline, release + PERIOD_NS)
            check()
            record = {'cycle': cycle, 'scheduled_release_ns': scheduled, 'release_ns': release,
                      'original_deadline_ns': deadline, 'feedback': {}, 'voltage': {}, 'output': {},
                      'owner_settlement': {}, 'completed': False, 'cleanup_only': False}
            if is30:
                record.update(scheduled_slot_deadline_ns=control.slot_deadline_ns(epoch, cycle),
                    acquisition_deadline_ns=acquisition_deadline,
                    proxy_reply_deadline_ns=deadline,
                    proxy_exchange_budget_ns=control.cycle_budget_ns,
                    proxy_write_deadline_abi_required=1, diagnostic_reply_budget_unchanged=False)
            record['journal_start_index_by_port'] = {port: len(adapters[port].journal) for port in PORTS}
            report['records'].append(record)
            prefix_proof.clear()
            prefix = {port: Future() for port in PORTS}
            # Mark running prevents public cancel from substituting a
            # cancellation result before the original owner publication.
            for future in prefix.values():
                future.set_running_or_notify_cancel()
            expected = {port: adapters[port].group.ids[cycle % 3] for port in PORTS}
            full = {}
            started = []
            for port in PORTS:
                full[port] = pools[port].submit(acquire, port, expected[port], prefix[port], acquisition_deadline)
                started.append(full[port])
            imu_future = pools['imu'].submit(imu_read)
            started.append(imu_future)
            feedback = {port: _take(prefix[port], acquisition_deadline, clock, hot) for port in PORTS}
            if config.combined_acquisition:
                # These are original four-slot batches. No three/one exchange
                # Stats or independently completed voltage phase is invented.
                record['combined_acquisition'] = feedback
                record['feedback_record_indices'] = [0, 1, 2]
                record['voltage_record_index'] = 3
            else:
                record['feedback'] = feedback
            imu = _take(imu_future, acquisition_deadline, clock, hot)
            record['imu'] = plain_copy(imu)
            voltage = {}
            if config.combined_acquisition:
                # A prefix hint never substitutes for the original executor
                # Future. Full4 ownership and current voltage join BEFORE the
                # model, so it cannot consume a pending fourth reply.
                for port in PORTS:
                    full_batch = _take(full[port], acquisition_deadline, clock, hot)
                    if full_batch is not feedback[port] or prefix_proof.get(port) is not full_batch:
                        raise ValueError('Current exact original combined/full Future binding failed')
                    voltage[port] = full_batch
                    cache[expected[port]] = _voltage(full_batch, adapters[port], expected[port],
                                                   clock(), 'acquisition_combined4')
                _fresh_cache(cache, clock())
                record['voltage_join_end_ns'] = clock()
            now = clock()
            rows = _rows(feedback, adapters, combined_acquisition=config.combined_acquisition)
            sample = feedback_sample(rows, config.profile, config.offsets, now_ns=now,
                                     previous=previous, required_mode=0)
            validate_measured(sample, config.profile, initial=initial)
            validate_imu_metadata(imu, now, config.profile, previous=previous_imu)
            if initial is None:
                initial = sample
            snapshot = snapshot_builder(feedback, imu, now)
            original_snapshot = plain_copy(snapshot)
            record['snapshot'] = original_snapshot
            record['gather_end_ns'] = clock()
            check()
            if clock() >= acquisition_deadline:
                raise TimeoutError('Original deadline before inference')
            _verify_observer_clock(observer, control, allow_unadvertised=allow_unadvertised_observer)
            record['observed'] = observer.consume(snapshot)
            record['infer_end_ns'] = clock()
            # Model result is never encoded as Type1. It is still consumed
            # once per cycle using the same stateful actual observer.
            if not config.combined_acquisition:
                for port in PORTS:
                    actual_feedback, voltage[port] = _take(full[port], acquisition_deadline, clock, hot)
                    if actual_feedback is not feedback[port] or prefix_proof.get(port) is not feedback[port]:
                        raise ValueError('Current exact original feedback/full Future binding failed')
                    cache[expected[port]] = _voltage(voltage[port], adapters[port], expected[port], clock(), 'voltage')
                record['voltage'] = voltage
                record['voltage_join_end_ns'] = clock()
            if snapshot != original_snapshot:
                raise ValueError('Observer mutated the input snapshot')
            # The full dynamic gate runs after inference and full original
            # owner joins. Every raw feedback/voltage/IMU value is rechecked.
            final_snapshot = dict(original_snapshot, _cycle_deadline_ns=deadline)
            final_rows, host_deadline = _final_gate(config, adapters, feedback, voltage, expected,
                imu, cache, final_snapshot, lambda f, i, n: dict(snapshot_builder(f, i, n),
                _cycle_deadline_ns=deadline), initial, previous, previous_imu, clock(),
                recorded_imu=record['imu'] if config.final_gate_input_identity else None,
                release_ns=release if is30 else None)
            if is30:
                record['input_to_stop_proxy_tx_deadline_ns'] = host_deadline
            check()
            if clock() >= host_deadline:
                raise TimeoutError('Original input-age/20ms deadline before proxy output')
            output = {}
            started = []
            for port in PORTS:
                if is30:
                    output[port] = pools[port].submit(adapters[port].output_stop,
                        deadline_ns=deadline, write_deadline_ns=host_deadline, check=hot)
                else:
                    output[port] = pools[port].submit(adapters[port].output_stop,
                        deadline_ns=host_deadline, check=hot)
                started.append(output[port])
            for port in PORTS:
                batch = (_take(output[port], deadline, clock, hot,
                               deadline_label='30Hz STOP-proxy reply33ms') if is30 else
                         _take(output[port], host_deadline, clock, hot))
                decoded = adapters[port].verify_batch(batch, 'output_stop')
                if set(decoded) != {(mid, 'feedback') for mid in adapters[port].group.ids}:
                    raise ValueError('Exact-three original STOP output replies required')
                record['output'][port] = batch
                if is30:
                    record.setdefault('proxy_host_write_records', {})[port] = _validate_proxy_host_writes(
                        batch, host_deadline, deadline)
            if is30:
                record['final_proxy_host_write_ns'] = max(row.finish_ns for batch in
                    record['output'].values() for row in batch.records)
                if record['final_proxy_host_write_ns'] >= host_deadline:
                    raise TimeoutError('30Hz proxy host-write exceeded preserved20ms deadline')
                record['input_to_stop_proxy_tx_20ms_met'] = True
            record['cycle_end_ns'] = clock()
            if record['cycle_end_ns'] >= deadline:
                raise TimeoutError('30Hz whole-cycle deadline' if is30 else 'Original whole-cycle 20ms deadline')
            record['completed'] = True
            phases = ('combined_acquisition', 'output') if config.combined_acquisition else (
                'feedback', 'voltage', 'output')
            record['actual_request_count'] = sum(len(batch.records) for phase in
                phases for batch in record[phase].values())
            if record['actual_request_count'] != 28:
                raise ValueError('Original four-bus 28 transaction proof incomplete')
            report['completed_cycles'] += 1
            record['journal_end_index_by_port'] = {port: len(adapters[port].journal) for port in PORTS}
            previous, previous_imu = final_rows, imu['read_started_monotonic_ns']
            started = []
        report['status'] = 'COMPLETE_STOP_PROXY_DIAGNOSTIC'
        # Terminal STOP has its own raw record and is not inferred from the
        # final cycle's STOP-proxy feedback. The original owner queues it.
        for port in PORTS:
            report['cleanup'][port] = pools[port].submit(adapters[port].recover_subset).result(timeout=.6)
        if any(value.get('complete') is not True for value in report['cleanup'].values()):
            raise RuntimeError('Independent terminal three-axis STOP incomplete')
    except BaseException as error:
        primary = error
        report['status'] = 'ABORTED'
        report['primary_error'] = {'type': type(error).__name__, 'message': str(error)}
        try:
            cancel_io()
        except BaseException as cleanup:
            report['cancel_error'] = str(cleanup)
        # Genuine owners settle before any new STOP. These timestamps and
        # results are cleanup evidence, never retroactive deadline admission.
        for index, future in enumerate(started):
            value = {'cleanup_only': True, 'admission_eligible': False}
            try:
                future.result(timeout=.5)
                value['state'] = 'SUCCESS'
            except BaseException as cleanup:
                value.update(state='EXCEPTION' if future.done() else 'PENDING', error=str(cleanup))
            value['done_observed'] = future.done()
            value['observation_monotonic_ns'] = clock()
            if value['state'] != 'PENDING':
                value['post_settlement_ns'] = value['observation_monotonic_ns']
            if report['records']:
                report['records'][-1]['owner_settlement'][str(index)] = value
            report['owner_settlement'].append(value)
        for port in PORTS:
            if port not in adapters:
                continue
            try:
                report['cleanup'][port] = pools[port].submit(adapters[port].recover_subset).result(timeout=.6)
            except BaseException as cleanup:
                report['cleanup'][port] = {'complete': False, 'error': str(cleanup),
                                          'physical_cutoff_required': True, 'cleanup_only': True}
    finally:
        # Every existing pool, including queued recovery, settles before
        # session close or descriptor reuse. Same-thread scopes restore first.
        for port, pool in pools.items():
            # Queue unconditionally: an initializer may still be entering its
            # scope after a bounded settlement observation. The one-worker
            # FIFO guarantees restoration occurs AFTER that actual initializer
            # and AFTER every original exchange/recovery on this physical bus.
            restoration = pool.submit(restore, port)
            pool.shutdown(wait=True, cancel_futures=False)
            try:
                restoration.result()
            except BaseException as cleanup:
                report['restoration'][port] = {'error': str(cleanup)}
                cleanup_error(cleanup)
        report['all_original_workers_joined_monotonic_ns'] = clock()
        if report['records']:
            report['records'][-1]['journal_end_index_by_port'] = {
                port: len(adapter.journal) for port, adapter in adapters.items()}
        for port, adapter in adapters.items():
            try:
                adapter.close()
            except BaseException as cleanup:
                report['restoration'][port+'_session'] = {'error': str(cleanup)}
                cleanup_error(cleanup)
            else:
                report['restoration'][port+'_session'] = True
        # All journal entries survive decode/timeout errors and remain tied to
        # their actual physical port; an earlier cycle is never copied into a
        # failed current row as though it were new data.
        report['raw_journal'] = {port: list(adapter.journal) for port, adapter in adapters.items()}
        if is30:
            # Preserve real writes even when an ACK/owner fails before the last
            # takeout. This is failure evidence, never retroactive admission.
            for record in report['records']:
                retained = {}
                for port, adapter in adapters.items():
                    begin = record['journal_start_index_by_port'].get(port, 0)
                    end = record.get('journal_end_index_by_port', {}).get(port, len(adapter.journal))
                    selected = [raw for label, raw in adapter.journal[begin:end] if label == 'output_stop']
                    if selected:
                        retained[port] = [_proxy_host_writes(raw[0]) for raw in selected]
                record['retained_proxy_host_write_records'] = retained
                observed_writes = [row for batches in retained.values() for batch in batches for row in batch
                                   if row['start_ns'] > 0 and row['finish_ns'] >= row['start_ns']]
                finishes = [row['finish_ns'] for row in observed_writes]
                if finishes:
                    record['final_proxy_host_write_ns'] = max(finishes)
                complete_finishes = [row['finish_ns'] for row in observed_writes if row['written'] == 17]
                if complete_finishes:
                    record['final_complete_proxy_host_write_ns'] = max(complete_finishes)
                record['retained_proxy_host_writes_are_admission'] = False
        report['physical_cutoff_required'] = any(value.get('complete') is not True
                                                for value in report['cleanup'].values())
        if main_entered:
            try:
                main_context.__exit__(type(primary) if primary else None, primary, None)
                report['restoration']['main'] = True
            except BaseException as cleanup:
                report['restoration']['main'] = {'error': str(cleanup)}
                cleanup_error(cleanup)
    report['failure_retained'] = primary is not None
    return report


def evidence_report(report):
    """Serialize only after owners/cleanup have joined; preserve all raw slots."""
    from singularitydog_hw.native_diagnostic_transport import Record, exchange_evidence
    from singularitydog_hw.native_active_transport import Stats
    import ctypes as C
    def plain(value):
        if type(value) is Batch:
            return value.evidence()
        if (type(value) is tuple and len(value) == 2 and isinstance(value[0], C.Array)
                and getattr(type(value[0]), '_type_', None) is Record and type(value[1]) is Stats):
            raw = exchange_evidence(*value)
            raw.update(original_record_count=len(value[0]),
                       rejected_total_bytes=int(value[1].rejected_total),
                       rejected_truncated=value[1].rejected_total > value[1].rejected_size)
            return raw
        if isinstance(value, dict):
            return {str(key): plain(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [plain(item) for item in value]
        if value is None or type(value) in (str, bool, int, float):
            return value
        raise ValueError('Unsupported diagnostic evidence value: '+type(value).__name__)
    return plain(report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topology', required=True)
    parser.add_argument('--topology-sha256', required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--profile-sha256', required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--combined-acquisition', action='store_true')
    parser.add_argument('--boundary-current-checks', action='store_true')
    parser.add_argument('--final-gate-input-identity', action='store_true')
    parser.add_argument('--decode-once', action='store_true')
    parser.add_argument('--policy30-bundle', help='Absolute pinned30Hz bundle; PLAN only')
    args = parser.parse_args(argv)
    from pathlib import Path
    from .topology import read_topology
    raw = Path(args.profile).read_bytes()
    if hashlib.sha256(raw).hexdigest() != args.profile_sha256:
        raise ValueError('Profile input bytes changed')
    document = json.loads(raw)
    topology = read_topology(args.topology, args.topology_sha256)
    groups = tuple(Group(port, tuple(topology['ids_by_port'][port])) for port in PORTS)
    control_clock = policy30_model = None
    if args.policy30_bundle is not None:
        from singularitydog_hw import policy_model_30hz
        from singularitydog_hw.policy_clock import control_clock_payload
        if not Path(args.policy30_bundle).is_absolute():
            raise ValueError('Absolute30Hz bundle required')
        policy30_model = policy_model_30hz.plan(policy_model_30hz.bundle_artifacts(args.policy30_bundle))
        control_clock = control_clock_payload(30)
    result = plan(Config(groups, document['profile'],
                         {int(mid): value for mid, value in document['offsets'].items()},
                         topology, args.cycles, combined_acquisition=args.combined_acquisition,
                         boundary_current_checks=args.boundary_current_checks,
                         final_gate_input_identity=args.final_gate_input_identity,
                         decode_once=args.decode_once, control_clock=control_clock,
                         policy30_model=policy30_model))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
