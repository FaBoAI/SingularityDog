"""Compare two saved single-ID PVP probes; file access only, no device access.

Require the expected UID file, frozen source hashes and boot context separately.
A complete recording may have dropped slots. No measurement establishes sensor
sample time, physical velocity, STOP, a power epoch, origin accuracy or approval.
All phase cohorts are retained; no interpolation, threshold or velocity changes.
Run with PYTHONPATH=runtime. The resulting JSON contains private artifact paths.
"""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import struct
import uuid

from singularitydog_hw import can_readonly as codec

SCHEMA = 'singularitydog.stationary-velocity-probe.v1'
ANALYSIS_SCHEMA = 'singularitydog.stationary-velocity-probe-comparison.v1'
SOURCE_NAMES = {'can_readonly.py', 'can_timing_probe.py', 'dual_can_pipeline_benchmark.py',
                'sensor_pipeline_benchmark.py', 'stationary_velocity_probe.py'}
FLAGS = dict.fromkeys(('motor_output_allowed', 'approved_for_runtime', 'motor_enable_sent',
    'stop_sent', 'stop_confirmed', 'learned_targets_sent', 'settings_written', 'automatic_retry',
    'angle_wrap_applied', 'physical_stationarity_proven', 'calibration_approved',
    'sensor_sample_time_verified', 'velocity_replaced_by_position_derivative',
    'existing_comparison_gates_changed', 'full_controller_50Hz_verified', 'period_guaranteed'), False)
PORTS = {'front': '/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0',
         'rear': '/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0'}
IDS = {'front': list(range(1, 7)), 'rear': list(range(7, 13))}
KEYS = ('position_before', 'velocity', 'position_after')
READS = ('position', 'velocity', 'position')
MAX_BYTES, MAX_EVENTS = 4*1024*1024, 16384


def need(condition, message):
    if not condition:
        raise ValueError(message)


def strict_json(raw):
    def pairs(rows):
        result = {}
        for key, value in rows:
            need(key not in result, 'Duplicate JSON key'); result[key] = value
        return result
    def number(value):
        result = float(value); need(math.isfinite(result), 'Nonfinite JSON number'); return result
    def bad(_):
        raise ValueError('Nonfinite JSON constant')
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number, parse_constant=bad)


def exact(actual, expected, label):
    """JSON equality includes types; bool is never an integer count."""
    need(type(actual) is type(expected), label+' type')
    if isinstance(expected, dict):
        need(set(actual) == set(expected), label+' fields')
        for key in expected:
            exact(actual[key], expected[key], label+'.'+key)
    elif isinstance(expected, list):
        need(len(actual) == len(expected), label+' length')
        for a, b in zip(actual, expected):
            exact(a, b, label)
    else:
        need(actual == expected, label+' value')


def fields(value, required, optional=()):
    need(type(value) is dict and set(required) <= set(value) <= set(required) | set(optional), 'Unknown/missing fields')


def timestamp(value):
    need(type(value) is int and 0 < value < 2**63, 'Invalid host timestamp'); return value


def digest(value):
    need(type(value) is str and re.fullmatch('[0-9a-f]{64}', value), 'Invalid SHA256'); return value


def wall_metadata(report):
    exact(report['persistence_scope'], 'This report cannot certify its own finalization. Require the separate final CLI receipt with matching report SHA256.', 'Persistence scope')
    times = []
    for key in ('started_at', 'completed_at'):
        need(type(report[key]) is str, 'Invalid wall timestamp')
        value = datetime.datetime.fromisoformat(report[key]); need(value.tzinfo is not None, 'Missing wall time zone')
        times.append(value)
    need(times[1] >= times[0], 'Reversed wall times')


def rawhex(value):
    need(type(value) is str and len(value) <= 4096 and re.fullmatch('(?:[0-9a-f]{2})*', value), 'Noncanonical raw hex')
    return bytes.fromhex(value)


def read_file(path, maximum=MAX_BYTES, *, allow_empty=False):
    path = Path(path).expanduser().absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Input symlink refused')
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and (allow_empty or info.st_size > 0) and info.st_size <= maximum,
             'Expected bounded regular file (JSON must be nonempty)')
        raw = handle.read(maximum+1)
        named = path.lstat()
        need((info.st_dev, info.st_ino, info.st_size) == (named.st_dev, named.st_ino, named.st_size)
             and len(raw) == info.st_size, 'Input binding/size changed')
    return raw, {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'byte_count': len(raw)}


def canonical_plan(mid, count, period):
    need(type(mid) is int and 1 <= mid <= 12 and type(period) is int and period in (20, 200), 'Invalid ID/period')
    need(type(count) is int and 1 <= count <= (500 if period == 20 else 50), 'Invalid slot budget')
    queries = 12+3*count
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', **FLAGS, 'hardware_opened': False,
        'motor_id': mid, 'selected_bus': 'front' if mid <= 6 else 'rear', 'ports': PORTS, 'ids_by_bus': IDS,
        'samples_requested': count, 'period_ms': period, 'segment_duration_bound_s': count*period/1000,
        'maximum_segment_duration_s': 10, 'identity_stage_bound_s': 3, 'request_timeout_max_ms': 250,
        'request_order': list(READS), 'minimum_time_remaining_before_request_ms': 15,
        'whole_triplet_reserve_ms': 50.0,
        'insufficient_segment_budget_policy': 'DROP_REMAINING_SLOTS_BEFORE_ANY_PVP_TX',
        'maximum_queries': queries, 'allowed_can_types': [0, 17], 'one_outstanding_request': True,
        'catchup_available': False, 'trace_event_budget': MAX_EVENTS, 'trace_byte_budget': MAX_BYTES,
        'trace_events_minimum_estimate': queries*5, 'trace_events_two_receive_chunks_estimate': queries*6,
        'trace_estimate_is_not_a_completion_guarantee': True, 'motor_power_epoch': 'NOT_INFERRED_FROM_JETSON_BOOT',
        'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL', 'external_operator_support_and_idle_confirmation_required': True,
        'timing_scope': 'Finite descriptive reads; 20ms selection is no period, controller or output guarantee.'}


def statistics(values):
    if not values:
        return None
    mean = math.fsum(values)/len(values)
    ordered = sorted(values)
    return {'count': len(values), 'minimum': ordered[0], 'maximum': ordered[-1], 'span': ordered[-1]-ordered[0],
            'mean': mean, 'rms': math.sqrt(math.fsum(v*v for v in values)/len(values)),
            'population_std': math.sqrt(math.fsum((v-mean)**2 for v in values)/len(values)),
            'max_abs': max(abs(v) for v in values), 'p95': ordered[math.ceil(.95*len(values))-1]}


def core_stats(values):
    result = statistics(values)
    return None if result is None else {'samples': len(values), **{k: result[k] for k in ('minimum', 'maximum', 'mean', 'rms', 'max_abs')}}


def replay(events, plan):
    """Replay every byte, frame, write timing and decoded receipt without CAN I/O."""
    parsers = {b: codec.ATParser() for b in IDS}
    queued = {b: [] for b in IDS}; pending = {b: None for b in IDS}
    sequences = Counter(); exchanges, counts, issues = [], Counter(), []
    previous, last_complete, identity_seen = 0, 0, set()
    for event in events:
        need(type(event) is dict and event.get('bus') in IDS, 'Invalid trace bus')
        bus, kind, when = event['bus'], event.get('kind'), timestamp(event.get('monotonic_ns'))
        need(when >= previous, 'Noncausal trace time'); previous = when; counts[kind] += 1
        base = {'bus', 'kind', 'monotonic_ns'}
        if kind == 'can_tx':
            fields(event, base | {'sequence', 'motor_id', 'parameter', 'hex'})
            need(pending[bus] is None and all(pending[b] is None for b in IDS), 'Overlapping requests')
            need(not queued[bus] and not parsers[bus].buffer and not parsers[bus].discarded_bytes, 'Nonfresh request boundary')
            sequences[bus] += 1; exact(event['sequence'], sequences[bus], 'Sequence')
            mid, parameter = event['motor_id'], event['parameter']
            if sequences[bus] <= 6:
                exact(mid, IDS[bus][sequences[bus]-1], 'Identity order'); exact(parameter, 'identity', 'Identity read')
                exact(mid, len(exchanges)+1, 'Global identity order')
            else:
                need(identity_seen == set(range(1, 13)) and bus == plan['selected_bus'], 'Missing identity barrier/wrong bus')
                exact(mid, plan['motor_id'], 'PVP ID')
                exact(parameter, READS[(sequences[bus]-7) % 3], 'PVP order')
            exact(rawhex(event['hex']), codec.read_request(mid, None if parameter == 'identity' else parameter), 'Readonly request')
            need(when >= last_complete, 'Request precedes previous receipt')
            current = {'bus': bus, 'tx': event, 'write': None, 'frame': None, 'receipt': None, 'timeout': None}
            exchanges.append(current); pending[bus] = current
        elif kind == 'probe_write_timing':
            fields(event, base | {'write_started_monotonic_ns', 'write_finished_monotonic_ns'})
            current = pending[bus]; need(current is not None and current['write'] is None, 'Unbound/duplicate physical write')
            started, finished = timestamp(event['write_started_monotonic_ns']), timestamp(event['write_finished_monotonic_ns'])
            need(current['tx']['monotonic_ns'] <= started <= finished == when, 'Invalid physical write times')
            current['write'] = event
        elif kind == 'can_rx_bytes':
            fields(event, base | {'hex'}); raw = rawhex(event['hex']); need(raw, 'Empty receive chunk')
            counts['rx_byte_count'] += len(raw)
            queued[bus].extend((f, when) for f in parsers[bus].feed(raw))
        elif kind == 'can_rx_frame':
            need(queued[bus], 'Frame without receive bytes'); frame, received = queued[bus].pop(0)
            exact(event, {**frame.record(), 'bus': bus, 'kind': kind, 'monotonic_ns': received}, 'Raw frame')
            current = pending[bus]
            need(current is not None and current['write'] is not None, 'Reply without physical write')
            need(current['frame'] is None, 'Duplicate reply')
            current['frame'] = frame; current['frame_received_ns'] = when
            need(when >= current['write']['write_finished_monotonic_ns'], 'Reply precedes write completion')
            parameter = current['tx']['parameter']; parameter = None if parameter == 'identity' else parameter
            if not codec.matches(frame, current['tx']['motor_id'], parameter):
                issues.append('nonmatching_reply_frame')
        elif kind == 'motor_parameter':
            current = pending[bus]
            need(current is not None and current['frame'] is not None, 'Receipt without raw reply')
            tx = current['tx']; parameter = None if tx['parameter'] == 'identity' else tx['parameter']
            decoded = codec.decode_reply(current['frame'], tx['motor_id'], parameter)
            expected = {**decoded, 'bus': bus, 'kind': kind, 'sequence': tx['sequence'],
                        'request_monotonic_ns': tx['monotonic_ns'], 'monotonic_ns': when,
                        'round_trip_ms': (when-tx['monotonic_ns'])/1e6}
            exact(event, expected, 'Raw decoded receipt')
            need(tx['monotonic_ns'] < when and when-tx['monotonic_ns'] < 250_000_000, 'Late receipt')
            current['receipt'] = event; pending[bus] = None; last_complete = when
            if decoded['ok'] is True and parameter is None:
                identity_seen.add(tx['motor_id'])
            if decoded['ok'] is not True:
                issues.append('failed_decoded_receipt')
        elif kind == 'can_timeout':
            fields(event, base | {'motor_id', 'parameter'}); current = pending[bus]
            need(current is not None and current['write'] is not None and current['timeout'] is None, 'Unbound timeout')
            exact(event['motor_id'], current['tx']['motor_id'], 'Timeout ID')
            exact(event['parameter'], current['tx']['parameter'], 'Timeout parameter')
            current['timeout'] = event; issues.append('reply_timeout')
        else:
            raise ValueError('Unknown trace event kind')
    need(not any(queued.values()), 'Missing raw frame events')
    for bus, parser in parsers.items():
        if parser.buffer or parser.discarded_bytes:
            issues.append(bus+':partial_or_discarded_receive_bytes')
    return exchanges, dict(counts), issues


def receipt_from_exchange(item):
    receipt = item['receipt']
    need(receipt is not None, 'Report receipt missing from trace')
    return {**{k: v for k, v in receipt.items() if k != 'bus'},
            'raw_request_hex': item['tx']['hex'], 'raw_reply_hex': item['frame'].wire.hex()}


def raw_value_stats(receipts):
    words = [int.from_bytes(bytes.fromhex(r['raw_value_hex']), 'little') for r in receipts]
    values = [r['value'] for r in receipts]
    unique = sorted(set(values))
    spacing = [b-a for a, b in zip(unique, unique[1:])]
    changes = [abs(v-struct.unpack('<f', (w & ~15).to_bytes(4, 'little'))[0]) for w, v in zip(words, values)]
    bound = max(changes, default=0.)
    return {'count': len(values), 'unique_raw_words': len(set(words)),
            'adjacent_equal_raw_word_count': sum(a == b for a, b in zip(words, words[1:])),
            'minimum_positive_observed_value_spacing': min(spacing) if spacing else None,
            'spacing_scope': 'Observed reported-value spacing, not encoder LSB or sensor precision.',
            'low_four_bit_histogram': {format(i, 'x'): sum((w & 15) == i for w in words) for i in range(16)},
            'low_four_bit_zero_count': sum((w & 15) == 0 for w in words),
            'low_four_bit_mask_max_abs_value_change': bound,
            'two_endpoint_change_upper_bound_from_low_four_bit_mask': 2*bound}


def describe_rows(rows, requested):
    complete = [r for r in rows if r['complete']]
    velocities = [r['velocity'] for r in complete]
    positions = [r[k] for r in complete for k in ('position_before', 'position_after')]
    times = [r['velocity']['monotonic_ns'] for r in complete]
    diffs = [{'slot_index': r['slot_index'], 'position_change_rad': r['position_after']['value']-r['position_before']['value'],
              'host_position_midpoint_separation_ns': ((r['position_after']['request_monotonic_ns']+r['position_after']['monotonic_ns'])//2
                  -(r['position_before']['request_monotonic_ns']+r['position_before']['monotonic_ns'])//2),
              'sensor_sample_time_verified': False, 'velocity_ground_truth': False} for r in complete]
    for d in diffs:
        d['position_finite_difference_rad_s_host_midpoints'] = d['position_change_rad']*1e9/d['host_position_midpoint_separation_ns']
    return {'requested_slot_count': requested, 'complete_triplets': len(complete),
            'incomplete_triplets': len(rows)-len(complete), 'data_coverage': requested > 0 and len(complete) == requested,
            'has_complete_measurements': bool(complete),
            'reported_velocity_rad_s': statistics([r['value'] for r in velocities]),
            'reported_position_rad': statistics([r['value'] for r in positions]),
            'raw_velocity': raw_value_stats(velocities), 'raw_position': raw_value_stats(positions),
            'adjacent_velocity_sign_reversal_count': sum(a['value']*b['value'] < 0 for a, b in zip(velocities, velocities[1:])),
            'adjacent_velocity_value_change_rad_s': statistics([b['value']-a['value'] for a, b in zip(velocities, velocities[1:])]),
            'velocity_reply_gap_ms': statistics([(b-a)/1e6 for a, b in zip(times, times[1:])]),
            'pvp_duration_ms': statistics([(r['end_ns']-r['begin_ns'])/1e6 for r in complete]),
            'host_bracketing_position_differences': diffs}


REPORT_FIELDS = {'schema', 'status', 'plan', 'errors', 'samples', 'hardware_opened', 'hardware_open_attempted',
    'expected_boot_id', 'expected_uids_sha256', 'source_sha256', 'started_at', 'motor_power_epoch', 'stop_state',
    'all_device_contexts_closed', 'boot_id', 'identities', 'all12_identities_verified', 'requests', 'dropped_slots',
    'unacquired_slots', 'period_deadline_missed_slots', 'triplet_reserve_abort_before_write_count', 'write_attempts',
    'queries_sent', 'request_intents_logged', 'analysis', 'requested_slots', 'complete_triplets', 'incomplete_triplets',
    'dropped_slot_count', 'unacquired_slot_count', 'full_requested_slot_coverage', 'resource_leases_retained_until_process_exit',
    'trace_events', 'completed_at', 'data_collection_status', 'report_finalization_claimed', 'persistence_scope'} | set(FLAGS)

STARTUP_FIELDS = {'schema', 'status', 'plan', 'errors', 'samples', 'hardware_opened', 'hardware_open_attempted',
    'expected_boot_id', 'expected_uids_sha256', 'source_sha256', 'started_at', 'completed_at', 'motor_power_epoch',
    'stop_state', 'all_device_contexts_closed', 'resource_leases_retained_until_process_exit', 'trace_events',
    'data_collection_status', 'report_finalization_claimed', 'persistence_scope'} | set(FLAGS)


def audit_startup(report, receipt, events, bindings, uid_sha, source, boot, mid):
    """A correctly retained pre-collection failure is unavailable data, not success."""
    fields(report, STARTUP_FIELDS, {'boot_id'})
    need(not events and bindings['trace']['byte_count'] == 0, 'Startup schema cannot hide any CAN events')
    exact(report['schema'], SCHEMA, 'Startup schema'); exact(report['samples'], [], 'Startup samples')
    plan = report['plan']; exact(plan, canonical_plan(mid, plan['samples_requested'], plan['period_ms']), 'Startup plan')
    exact(report['source_sha256'], source, 'Startup source'); exact(report['expected_uids_sha256'], uid_sha, 'Startup UID hash')
    exact(report['expected_boot_id'], boot, 'Startup expected boot')
    if 'boot_id' in report: exact(report['boot_id'], boot, 'Startup observed boot')
    exact(report['status'], 'INCOMPLETE', 'Startup status'); exact(report['data_collection_status'], 'INCOMPLETE', 'Startup collection status')
    exact(report['report_finalization_claimed'], False, 'Startup finalization')
    wall_metadata(report)
    exact(report['motor_power_epoch'], plan['motor_power_epoch'], 'Startup power epoch scope')
    exact(report['stop_state'], plan['stop_state'], 'Startup STOP scope')
    for key, value in FLAGS.items(): exact(report[key], value, 'Startup '+key)
    need(type(report['errors']) is list and report['errors'] and all(type(e) is str for e in report['errors']), 'Startup errors required')
    for key in ('hardware_opened', 'hardware_open_attempted', 'all_device_contexts_closed', 'resource_leases_retained_until_process_exit'):
        need(type(report[key]) is bool, 'Startup flag type')
    trace = report['trace_events']; fields(trace, {'path', 'sha256', 'event_count', 'attempted_event_count', 'byte_count', 'complete', 'errors', 'status'})
    if trace['sha256'] is not None: exact(trace['sha256'], bindings['trace']['sha256'], 'Startup trace SHA')
    else: need(trace['complete'] is False and trace['errors'], 'Missing finalized startup trace SHA')
    for key in ('event_count', 'attempted_event_count', 'byte_count'): exact(trace[key], 0, 'Startup trace count')
    need(type(trace['complete']) is bool and type(trace['errors']) is list and all(type(e) is str for e in trace['errors']), 'Startup trace state')
    exact(trace['status'], 'COMPLETE_EVENT_TRACE' if trace['complete'] else 'INCOMPLETE_EVENT_TRACE', 'Startup trace status')
    need(not trace['complete'] or not trace['errors'], 'Complete startup trace has errors')
    counts = {'requested_slots': plan['samples_requested'], 'complete_triplets': 0, 'dropped_slot_count': 0,
              'incomplete_triplets': 0, 'unacquired_slot_count': plan['samples_requested'], 'full_requested_slot_coverage': False}
    fields(receipt, {'status', 'output', 'report_sha256', 'errors', *counts, *FLAGS})
    exact(receipt['status'], 'INCOMPLETE', 'Startup receipt'); exact(receipt['report_sha256'], None, 'Startup report binding')
    need(type(receipt['errors']) is list and all(type(e) is str for e in receipt['errors'])
         and receipt['errors'][:len(report['errors'])] == report['errors'], 'Startup final errors')
    for key, value in counts.items(): exact(receipt[key], value, 'Startup final count')
    for key, value in FLAGS.items(): exact(receipt[key], value, 'Startup final '+key)
    need(type(receipt['output']) is str and type(trace['path']) is str and Path(receipt['output']).is_absolute()
         and Path(trace['path']).is_absolute() and receipt['output'] != trace['path'], 'Startup artifact paths')
    result = describe_rows([], plan['samples_requested'])
    result.update(period_ms=plan['period_ms'], recording_complete=False, final_report_binding_verified=False,
        acquisition_state='UNACQUIRED_STARTUP_RECORD', boot_context_verified='boot_id' in report,
        dropped_slots=[], unacquired_slots=list(range(plan['samples_requested'])),
        missing_unclassified_slots=list(range(plan['samples_requested'])), period_deadline_missed_slots=[],
        trace_counts={}, trace_issues=[], trace_complete=trace['complete'], trace_errors=trace['errors'],
        trace_manifest_binding_verified=trace['sha256'] is not None,
        core_report_errors=report['errors'], core_final_receipt_errors=receipt['errors'], request_intents_without_physical_write=0,
        requests_without_receipts=[], pvp_receipts_not_retained_in_sample_rows=[],
        all_valid_velocity_receipts_including_partial_triplets=None,
        receipt_round_trip_ms_by_parameter={p: None for p in ('identity', 'position', 'velocity')},
        all_reply_timing_rows=[], all_sample_timing_rows=[], all_receipt_gap_ms_by_bus={b: None for b in IDS},
        segment_begin_ns=None, segment_deadline_ns=None, segment_end_ns=None, segment_overrun_ns=None,
        artifact_bindings=bindings, original_report_path=receipt['output'], original_trace_path=trace['path'])
    return result


def audit_run(report, receipt, events, bindings, uids, uid_sha, source, boot, mid):
    if type(report) is dict and 'identities' not in report:
        return audit_startup(report, receipt, events, bindings, uid_sha, source, boot, mid)
    fields(report, REPORT_FIELDS, {'segment_begin_ns', 'segment_deadline_ns', 'segment_end_ns'})
    exact(report['schema'], SCHEMA, 'Schema')
    for k, v in FLAGS.items(): exact(report[k], v, k)
    plan = report['plan']; fields(plan, canonical_plan(mid, 1, 20))
    exact(plan, canonical_plan(mid, plan['samples_requested'], plan['period_ms']), 'Canonical plan')
    exact(report['source_sha256'], source, 'Frozen source hashes')
    exact(report['expected_uids_sha256'], uid_sha, 'Expected UID file SHA')
    exact(report['boot_id'], boot, 'Boot context'); exact(report['expected_boot_id'], boot, 'Expected boot')
    exact(report['motor_power_epoch'], plan['motor_power_epoch'], 'Power epoch scope')
    exact(report['stop_state'], plan['stop_state'], 'STOP scope')
    need(type(report['errors']) is list and all(type(e) is str for e in report['errors']), 'Invalid errors')
    exact(report['report_finalization_claimed'], False, 'Self finalization')
    wall_metadata(report)
    need(report['data_collection_status'] in ('COMPLETE_READONLY_PROBE_CAPTURE', 'INCOMPLETE'), 'Invalid collection status')
    for key in ('hardware_opened', 'hardware_open_attempted', 'all12_identities_verified', 'all_device_contexts_closed',
                'resource_leases_retained_until_process_exit'):
        need(type(report[key]) is bool, 'Invalid state flag')
    trace = report['trace_events']
    fields(trace, {'path', 'sha256', 'event_count', 'attempted_event_count', 'byte_count', 'complete', 'errors', 'status'})
    if trace['sha256'] is not None: exact(trace['sha256'], bindings['trace']['sha256'], 'Trace SHA')
    else: need(trace['complete'] is False and trace['errors'], 'Missing finalized trace SHA')
    exact(trace['byte_count'], bindings['trace']['byte_count'], 'Trace bytes')
    exact(trace['event_count'], len(events), 'Trace event count')
    need(type(trace['attempted_event_count']) is int and trace['attempted_event_count'] >= len(events), 'Trace attempts')
    need(type(trace['complete']) is bool and type(trace['errors']) is list and all(type(e) is str for e in trace['errors']), 'Trace state')
    exact(trace['status'], 'COMPLETE_EVENT_TRACE' if trace['complete'] else 'INCOMPLETE_EVENT_TRACE', 'Trace status')
    need(not trace['complete'] or not trace['errors'], 'Complete trace has errors')
    if trace['complete']: exact(trace['attempted_event_count'], len(events), 'Complete trace attempts')
    exchanges, counts, issues = replay(events, plan)
    need(not exchanges or (report['hardware_opened'] and report['hardware_open_attempted']), 'Trace contradicts open state')
    exact(report['request_intents_logged'], counts.get('can_tx', 0), 'Request intent count')
    exact(report['write_attempts'], counts.get('probe_write_timing', 0), 'Physical write count')
    exact(report['queries_sent'], report['write_attempts'], 'Query count')
    need(len(exchanges) <= plan['maximum_queries'], 'Query budget exceeded')
    need(type(report['requests']) is list and len(exchanges) <= len(report['requests']) <= plan['maximum_queries'], 'Request timing count')
    # Failed calls may produce last_timing without a can_tx (e.g. budget guard).
    unmatched = list(exchanges); request_allowed = {'bus', 'motor_id', 'parameter', 'ok', 'residual_before', 'residual_after',
        'sequence', 'request_monotonic_ns', 'completed_monotonic_ns', 'round_trip_ms', 'write_started_monotonic_ns',
        'write_finished_monotonic_ns', 'received_monotonic_ns', 'wire_round_trip_ms', 'error', 'residual_inspection_error'}
    for row in report['requests']:
        fields(row, {'bus', 'motor_id', 'parameter', 'ok', 'write_started_monotonic_ns', 'write_finished_monotonic_ns',
                     'received_monotonic_ns', 'wire_round_trip_ms'}, request_allowed)
        need(type(row['ok']) is bool and row['bus'] in IDS, 'Request timing state')
        need(type(row['motor_id']) is int and row['motor_id'] in IDS[row['bus']]
             and (row['parameter'] == 'identity' or (row['bus'] == plan['selected_bus']
                  and row['motor_id'] == plan['motor_id'] and row['parameter'] in READS)), 'Timing request scope')
        for key in ('residual_before', 'residual_after'):
            if key in row:
                fields(row[key], {'serial_pending_bytes', 'parser_pending_bytes', 'parser_discarded_bytes'})
                need(all(type(v) is int and v >= 0 for v in row[key].values()), 'Invalid residual counts')
        item = unmatched[0] if unmatched else None
        if item and (row['bus'], row['motor_id'], row['parameter']) == (item['bus'], item['tx']['motor_id'], item['tx']['parameter']):
            unmatched.pop(0); write, answer = item['write'], item['receipt']
            for key in ('write_started_monotonic_ns', 'write_finished_monotonic_ns'):
                exact(row[key], write[key] if write else None, 'Write timing binding')
            received = item.get('frame_received_ns')
            exact(row['received_monotonic_ns'], received, 'Receive timing binding')
            exact(row['wire_round_trip_ms'], (received-write['write_started_monotonic_ns'])/1e6 if received and write else None, 'Wire RTT')
            if answer:
                for key, value in {'sequence': answer['sequence'], 'request_monotonic_ns': answer['request_monotonic_ns'],
                                   'completed_monotonic_ns': answer['monotonic_ns'], 'round_trip_ms': answer['round_trip_ms']}.items():
                    exact(row.get(key), value, 'Request receipt timing')
            if row['ok']:
                need(answer is not None and answer['ok'] is True and not row.get('error'), 'Successful request lacks valid reply')
                need(not any(row['residual_before'].values()) and not any(row['residual_after'].values()), 'Successful request residual')
        else:
            need(row['ok'] is False and row.get('error') and row['write_started_monotonic_ns'] is None, 'Unbound request timing')
    need(not unmatched, 'Trace request missing report timing')
    identities = report['identities']; need(type(identities) is dict and set(identities) <= set(uids), 'Identity schema')
    by_key = {(e['bus'], e['tx']['sequence']): e for e in exchanges}
    for key, answer in identities.items():
        motor = int(key); bus = 'front' if motor <= 6 else 'rear'; seq = IDS[bus].index(motor)+1
        exact(answer, receipt_from_exchange(by_key[(bus, seq)]), 'Identity receipt')
        exact(answer.get('mcu_uid_hex'), uids[key], 'UID identity')
    exact(report['all12_identities_verified'], set(identities) == set(uids), 'All12 identity barrier')
    need(type(report['samples']) is list and type(report['dropped_slots']) is list, 'Invalid slots')
    seen, dropped, differences, complete_values, receipt_keys = set(), set(), [], [], set()
    last_end = 0
    for row in report['samples']:
        fields(row, {'slot_index', 'requested_release_ns', 'begin_ns', 'end_ns', 'complete', 'state'}, {*KEYS, 'error'})
        index = row['slot_index']; need(type(index) is int and 0 <= index < plan['samples_requested'] and index not in seen, 'Invalid/repeated slot')
        need(not seen or index > max(seen), 'Reordered slots'); seen.add(index)
        need(type(row['complete']) is bool, 'Invalid triplet completion')
        exact(row['state'], 'COMPLETE_TRIPLET' if row['complete'] else 'INCOMPLETE_TRIPLET', 'Triplet state')
        release = report['segment_begin_ns']+index*plan['period_ms']*1_000_000
        exact(row['requested_release_ns'], release, 'Release time')
        need(release <= timestamp(row['begin_ns']) <= timestamp(row['end_ns']) and row['begin_ns'] >= last_end, 'PVP chronology')
        available = [k for k in KEYS if k in row]
        need(available == list(KEYS[:len(available)]), 'Nonprefix partial PVP')
        previous_finish, previous_sequence = row['begin_ns'], None
        for key, parameter in zip(available, READS):
            answer = row[key]; item = by_key.get((plan['selected_bus'], answer.get('sequence')))
            need(item is not None and item['tx']['parameter'] == parameter, 'Unbound PVP receipt')
            binding_key = (item['bus'], item['tx']['sequence']); need(binding_key not in receipt_keys, 'Reused receipt'); receipt_keys.add(binding_key)
            exact(answer, receipt_from_exchange(item), 'PVP raw receipt')
            need(answer['ok'] is True and previous_finish <= answer['request_monotonic_ns'] < answer['monotonic_ns'] <= row['end_ns'], 'Unbracketed PVP times')
            need(previous_sequence is None or answer['sequence'] == previous_sequence+1, 'PVP sequence gap')
            previous_finish, previous_sequence = answer['monotonic_ns'], answer['sequence']
        need(not row['complete'] or (len(available) == 3 and 'error' not in row), 'Incomplete marked complete')
        if row['complete']:
            b, a = row['position_before'], row['position_after']
            separation = (a['request_monotonic_ns']+a['monotonic_ns'])//2-(b['request_monotonic_ns']+b['monotonic_ns'])//2
            differences.append({'slot_index': index, 'position_change_rad': a['value']-b['value'],
                'host_position_midpoint_separation_ns': separation,
                'position_finite_difference_rad_s_host_midpoints': (a['value']-b['value'])*1e9/separation,
                'sensor_sample_time_verified': False, 'velocity_ground_truth': False})
            complete_values.append(row['velocity']['value'])
        last_end = row['end_ns']
    for row in report['dropped_slots']:
        fields(row, {'slot_index', 'state', 'reason'}); index = row['slot_index']
        need(type(index) is int and 0 <= index < plan['samples_requested'] and index not in seen | dropped, 'Invalid dropped slot')
        need(row['state'] == 'DROPPED' and row['reason'] in ('insufficient_segment_budget', 'release_elapsed_before_triplet_start'), 'Dropped state/reason')
        dropped.add(index)
    unacquired = sorted(set(range(plan['samples_requested']))-seen)
    complete = len(complete_values)
    counts_expected = {'requested_slots': plan['samples_requested'], 'complete_triplets': complete,
        'incomplete_triplets': len(seen)-complete, 'dropped_slot_count': len(dropped), 'unacquired_slot_count': len(unacquired),
        'full_requested_slot_coverage': complete == plan['samples_requested']}
    for key, value in counts_expected.items(): exact(report[key], value, 'Coverage '+key)
    exact(report['unacquired_slots'], unacquired, 'Unacquired slots')
    period_misses = [r['slot_index'] for r in report['samples'] if r['complete'] and r['end_ns'] > r['requested_release_ns']+plan['period_ms']*1_000_000]
    exact(report['period_deadline_missed_slots'], period_misses, 'Period misses')
    exact(report['analysis'], {'status': 'NO_COMPLETE_SAMPLES' if not complete else 'ALL_REQUESTED_TRIPLETS_DESCRIBED' if complete == plan['samples_requested'] else 'PARTIAL_TRIPLETS_DESCRIBED',
        'complete_triplets': complete, 'reported_velocity_rad_s': core_stats(complete_values),
        'requested_triplets': plan['samples_requested'], 'incomplete_triplets': len(seen)-complete,
        'dropped_slot_count': len(dropped), 'unacquired_slots': unacquired, 'full_requested_slot_coverage': complete == plan['samples_requested'],
        'host_bracketing_position_differences': differences, 'all_sample_rows_retained': True,
        'incomplete_triplets_excluded_from_statistics_but_retained': True, 'stationary_gate_applied': False,
        'calibration_approved': False, 'output_allowed': False,
        'scope': 'Separate host-time parameter reads, no sensor acquisition timestamps; position differences never replace measured velocity.'}, 'Saved descriptive analysis')
    if 'segment_begin_ns' in report:
        begin, deadline = timestamp(report['segment_begin_ns']), timestamp(report['segment_deadline_ns'])
        exact(deadline-begin, plan['samples_requested']*plan['period_ms']*1_000_000, 'Segment bound')
        if report['data_collection_status'] == 'COMPLETE_READONLY_PROBE_CAPTURE':
            need(last_end <= deadline, 'Complete PVP beyond segment cap')
        if 'segment_end_ns' in report:
            need(last_end <= timestamp(report['segment_end_ns']), 'Segment end precedes final row')
            if report['data_collection_status'] == 'COMPLETE_READONLY_PROBE_CAPTURE':
                need(report['segment_end_ns'] <= deadline, 'Complete segment beyond cap')
    fields(receipt, {'status', 'output', 'report_sha256', 'errors', *counts_expected, *FLAGS})
    for key, value in counts_expected.items(): exact(receipt[key], value, 'Final receipt coverage '+key)
    for k, v in FLAGS.items(): exact(receipt[k], v, 'Final receipt '+k)
    need(type(receipt['output']) is str and type(trace['path']) is str
         and Path(receipt['output']).is_absolute() and Path(trace['path']).is_absolute()
         and receipt['output'] != trace['path'], 'Invalid original artifact path')
    need(type(receipt['errors']) is list and all(type(e) is str for e in receipt['errors'])
         and receipt['errors'][:len(report['errors'])] == report['errors'], 'Final errors lost')
    saved = receipt['status'] == 'SAVED_READONLY_PROBE_REVIEW_REQUIRED'
    recording_complete = report['data_collection_status'] == 'COMPLETE_READONLY_PROBE_CAPTURE'
    exact(report['status'], 'RECORDED_REVIEW_REQUIRED' if recording_complete else 'INCOMPLETE', 'Report status')
    if saved:
        exact(receipt['report_sha256'], bindings['report']['sha256'], 'Final report SHA')
        need(recording_complete and complete > 0 and not report['errors'] and not receipt['errors'] and trace['complete']
             and report['all12_identities_verified'] and report['all_device_contexts_closed']
             and not report['resource_leases_retained_until_process_exit'] and not issues
             and seen | dropped == set(range(plan['samples_requested'])), 'Invalid final success receipt')
    else:
        exact(receipt['status'], 'INCOMPLETE', 'Final receipt status')
        need(receipt['report_sha256'] is None and receipt['errors'], 'Invalid incomplete finalization')
    unwritten = sum(e['write'] is None for e in exchanges)
    need(type(report['triplet_reserve_abort_before_write_count']) is int and 0 <= report['triplet_reserve_abort_before_write_count'] <= 1, 'Reserve abort count')
    need(unwritten <= report['triplet_reserve_abort_before_write_count'] or not saved, 'Unexplained unwritten intent')
    pvp_receipt_keys = {(e['bus'], e['tx']['sequence']) for e in exchanges
                        if e['tx']['parameter'] != 'identity' and e['receipt'] is not None}
    if saved:
        exact(pvp_receipt_keys, receipt_keys, 'All PVP receipts retained in sample rows')
        need(all(e is exchanges[-1] and e['tx']['parameter'] == 'position' and e['tx']['sequence'] % 3 == 1
                 for e in exchanges if e['write'] is None), 'Invalid reserve-abort intent')
    description = describe_rows(report['samples'], plan['samples_requested'])
    answers = [e['receipt'] for e in exchanges if e['receipt'] is not None]
    description.update(period_ms=plan['period_ms'], recording_complete=recording_complete,
        acquisition_state='COLLECTION_RECORD', boot_context_verified=True,
        final_report_binding_verified=saved, dropped_slots=report['dropped_slots'], unacquired_slots=unacquired,
        missing_unclassified_slots=sorted(set(unacquired)-dropped), period_deadline_missed_slots=period_misses,
        trace_counts=counts, trace_issues=issues, request_intents_without_physical_write=unwritten,
        trace_complete=trace['complete'], trace_errors=trace['errors'], core_report_errors=report['errors'],
        trace_manifest_binding_verified=trace['sha256'] is not None,
        core_final_receipt_errors=receipt['errors'],
        pvp_receipts_not_retained_in_sample_rows=[{'bus': b, 'sequence': s} for b, s in sorted(pvp_receipt_keys-receipt_keys)],
        requests_without_receipts=[{'bus': e['bus'], 'sequence': e['tx']['sequence'], 'parameter': e['tx']['parameter'],
                                    'physical_write_recorded': e['write'] is not None, 'timeout_recorded': e['timeout'] is not None}
                                   for e in exchanges if e['receipt'] is None],
        all_valid_velocity_receipts_including_partial_triplets=statistics([r['value'] for r in answers if r['parameter'] == 'velocity' and r['ok']]),
        receipt_round_trip_ms_by_parameter={p: statistics([r['round_trip_ms'] for r in answers if r['parameter'] == p]) for p in ('identity', 'position', 'velocity')},
        all_reply_timing_rows=[{'bus': e['bus'], 'sequence': e['tx']['sequence'], 'parameter': e['tx']['parameter'],
            'ok': e['receipt']['ok'], 'request_monotonic_ns': e['tx']['monotonic_ns'],
            'write_started_monotonic_ns': e['write']['write_started_monotonic_ns'],
            'write_finished_monotonic_ns': e['write']['write_finished_monotonic_ns'],
            'raw_frame_received_monotonic_ns': e['frame_received_ns'],
            'receipt_monotonic_ns': e['receipt']['monotonic_ns'], 'round_trip_ms': e['receipt']['round_trip_ms']}
            for e in exchanges if e['receipt'] is not None],
        all_receipt_gap_ms_by_bus={b: statistics([(z-a)/1e6 for a, z in zip(
            [r['monotonic_ns'] for r in answers if r['bus'] == b],
            [r['monotonic_ns'] for r in answers if r['bus'] == b][1:])]) for b in IDS},
        all_sample_timing_rows=[{k: r[k] for k in ('slot_index', 'requested_release_ns', 'begin_ns', 'end_ns', 'complete', 'state')}
                              for r in report['samples']],
        segment_begin_ns=report.get('segment_begin_ns'), segment_deadline_ns=report.get('segment_deadline_ns'),
        segment_end_ns=report.get('segment_end_ns'),
        segment_overrun_ns=max(0, max(last_end, report.get('segment_end_ns', 0))-report['segment_deadline_ns']) if 'segment_deadline_ns' in report else None,
        artifact_bindings=bindings,
        original_report_path=receipt['output'], original_trace_path=trace['path'])
    return description


def compare(paths, *, expected_uids, expected_sources, expected_boot_id, motor_id):
    need(type(expected_boot_id) is str and str(uuid.UUID(expected_boot_id)) == expected_boot_id, 'Canonical expected boot required')
    uid_raw, uid_binding = read_file(expected_uids, 16384); uids = strict_json(uid_raw)
    need(type(uids) is dict and set(uids) == {str(i) for i in range(1, 13)}
         and all(type(v) is str and re.fullmatch('[0-9a-f]{16}', v) for v in uids.values())
         and len(set(uids.values())) == 12, 'Invalid all12 UIDs')
    source_raw, source_binding = read_file(expected_sources, 16384); source = strict_json(source_raw)
    fields(source, SOURCE_NAMES)
    for value in source.values(): digest(value)
    exact(source['can_readonly.py'], hashlib.sha256(Path(codec.__file__).read_bytes()).hexdigest(), 'Loaded decoder SHA')
    need(type(paths) is list and len(paths) == 2, 'Two saved runs required')
    runs, reports = {}, {}
    for expected_period, paths_for_run in zip((20, 200), paths):
        fields(paths_for_run, {'report', 'receipt', 'trace'}); raw, bindings = {}, {}
        for key, path in paths_for_run.items(): raw[key], bindings[key] = read_file(path, allow_empty=key == 'trace')
        report, receipt = strict_json(raw['report']), strict_json(raw['receipt'])
        exact(report['plan']['period_ms'], expected_period, 'Input period label')
        need(not raw['trace'] or raw['trace'].endswith(b'\n'), 'Incomplete trace final line')
        lines = raw['trace'].splitlines(); need(len(lines) <= MAX_EVENTS and all(lines), 'Trace event budget/blank line')
        events = [strict_json(line) for line in lines]
        run = audit_run(report, receipt, events, bindings, uids, uid_binding['sha256'], source, expected_boot_id, motor_id)
        period = str(run['period_ms']); need(period not in runs, 'Periods must be distinct 20/200ms')
        runs[period], reports[period] = run, report
    need(set(runs) == {'20', '200'}, 'Expected 20ms and 200ms runs')
    fast = reports['20']; fast_unacquired = runs['20']['unacquired_slots']; cohorts = []
    for phase in range(10):
        requested = list(range(phase, fast['plan']['samples_requested'], 10))
        rows = [r for r in fast['samples'] if r['slot_index'] in requested]
        item = describe_rows(rows, len(requested))
        item.update(phase=phase, requested_slot_indices=requested,
            dropped_slot_indices=[r['slot_index'] for r in fast.get('dropped_slots', []) if r['slot_index'] in requested],
            unacquired_slot_indices=[i for i in fast_unacquired if i in requested])
        cohorts.append(item)
    common_duration = min(reports[p]['plan']['samples_requested']*reports[p]['plan']['period_ms']*1_000_000 for p in reports)
    common = {}
    for p in reports:
        begin = reports[p].get('segment_begin_ns')
        selected = [r for r in reports[p]['samples'] if r['requested_release_ns']-begin < common_duration
                    and (not r['complete'] or r['velocity']['monotonic_ns']-begin <= common_duration)]
        common[p] = describe_rows(selected, common_duration//(reports[p]['plan']['period_ms']*1_000_000))
        common[p]['complete_velocity_receipts_outside_common_host_reply_window'] = sum(
            r['complete'] and r['requested_release_ns']-begin < common_duration
            and r['velocity']['monotonic_ns']-begin > common_duration for r in reports[p]['samples'])
    complete = all(r['final_report_binding_verified'] and r['complete_triplets'] > 0 for r in runs.values())
    return {'schema': ANALYSIS_SCHEMA, 'status': 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED' if complete else 'INCOMPLETE_COMPARISON_RECORDS',
        **FLAGS, 'hardware_opened': False, 'automatic_approval': False, 'velocity_ground_truth': False,
        'absolute_origin_uncertainty_quantified': False, 'motor_id': motor_id,
        'expected_uid_file_binding': uid_binding, 'expected_source_manifest_binding': source_binding,
        'capture_source_sha256': source, 'decoder_sha256': source['can_readonly.py'],
        'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'boot_context_sha256': hashlib.sha256(expected_boot_id.encode()).hexdigest(),
        'runs': runs, 'all_requested_slot_coverage': all(r['data_coverage'] for r in runs.values()),
        'twenty_ms_all_ten_phase_cohorts': cohorts, 'common_nominal_elapsed_window_ns': common_duration,
        'common_nominal_window_statistics': common,
        'cause_hypotheses_status': 'UNRESOLVED; descriptive comparison only',
        'limitations': [
            '20ms is nominal host scheduling; report actual reply gaps, drops and period misses. It does not certify sensor or controller rate.',
            'The minimum observed position spacing is not encoder resolution or accuracy.',
            'Position quantization and unknown sensor timestamps cannot separate estimator variability from sub-quantization/high-frequency motion or aliasing.',
            'Position host-midpoint differences are descriptive and never replace Type17 velocity.',
            'Peak values depend on sample count. All ten phase cohorts, including empty/missing cohorts, are shown; none is selected as representative.',
            'Blocks differ in acquisition time/order; temperature, supply, support and internal firmware state are not measured by this PVP probe.',
            'Only Type0/17 is measured. Type2 cross-protocol update age/scale remains unmeasured.',
            'Constant angle offset cancels from velocity differences; sign changes direction, not absolute velocity or RMS.',
            'Same boot is context only, not motor power epoch, STOP or physical stationarity. Bubble-level alignment alone does not quantify absolute origin uncertainty.',
            'A failed final CLI receipt prevents verified artifact completion even if a saved JSON looks complete.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for period in (20, 200):
        for kind in ('report', 'receipt', 'trace'): parser.add_argument(f'--{kind}-{period}', required=True)
    parser.add_argument('--expected-uids', required=True)
    parser.add_argument('--expected-source-hashes', required=True)
    parser.add_argument('--expected-boot-id', required=True)
    parser.add_argument('--id', required=True, type=int, dest='motor_id')
    args = parser.parse_args(argv)
    try:
        result = compare([{kind: getattr(args, f'{kind}_{p}') for kind in ('report', 'receipt', 'trace')} for p in (20, 200)],
                         expected_uids=args.expected_uids, expected_sources=args.expected_source_hashes,
                         expected_boot_id=args.expected_boot_id, motor_id=args.motor_id)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(json.dumps({'schema': ANALYSIS_SCHEMA, 'status': 'INVALID_ARTIFACTS', 'error': str(error),
                          **FLAGS, 'hardware_opened': False, 'automatic_approval': False}, allow_nan=False))
        return 1
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['status'] == 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
