"""Analyze a saved fixed50/200ms/10s Type0/17 capture, without hardware.

Every saved receive byte and successful parameter receipt is accounted for.
Position slopes use host midpoints; they never replace the reported velocity.
Completion means file integrity only, not physical stationarity, STOP, joint
calibration, dynamic scale, the older fixed21 specification, or output approval.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import uuid

from singularitydog_hw import can_readonly as codec


SCHEMA = 'singularitydog.stationary-readonly-capture.v1'
ANALYSIS_SCHEMA = 'singularitydog.stationary-readonly-analysis.v1'
BUSES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
PARAMETERS = ('position', 'velocity', 'current', 'run_mode', 'voltage')
SWEEPS, PERIOD_NS, DURATION_NS = 50, 200_000_000, 10_000_000_000
REQUESTS_PER_BUS = 6 + SWEEPS * 30
MAX_FILE_BYTES = 32 * 1024 * 1024
SOURCE_NAMES = ('stationary_readonly_capture.py', 'can_readonly.py',
                'dual_can_pipeline_benchmark.py', 'sensor_pipeline_benchmark.py',
                'can_timing_probe.py', 'fixed_stance_readonly_capture.py',
                'motor_epoch_readonly_capture.py', 'post_charge_box_pose_check.py')
FALSE_FLAGS = ('motor_output_allowed', 'motor_enable_sent', 'stop_sent', 'stop_confirmed',
               'human_stance_verified', 'stationarity_verified', 'approved_for_runtime',
               'angle_wrap_applied', 'automatic_retry')
SCOPE_FLAGS = {**dict.fromkeys(FALSE_FLAGS, False), 'hardware_opened': False,
               'type2_comparison_verified': False, 'physical_stationarity_proven': False,
               'angle_calibration_verified': False, 'dynamic_scale_validated': False,
               'sensor_sample_time_verified': False, 'legacy_fixed21_evaluated': False,
               'comparison_gate_changed': False, 'automatic_approval': False,
               'velocity_replaced_by_position_derivative': False,
               'sample_filtering_applied': False}


def need(condition, message):
    if not condition:
        raise ValueError(message)


def stamp(value):
    need(type(value) is int and 0 < value < 2**63, 'Invalid host timestamp')
    return value


def sha256(value):
    need(type(value) is str and re.fullmatch('[0-9a-f]{64}', value), 'Invalid SHA256')
    return value


def boot_id(value):
    need(type(value) is str and str(uuid.UUID(value)) == value, 'Invalid canonical boot ID')
    return value


def uids(value):
    need(type(value) is dict and set(value) == {str(i) for i in range(1, 13)},
         'Expected all twelve UID keys')
    need(all(type(v) is str and re.fullmatch('[0-9a-f]{16}', v) for v in value.values())
         and len(set(value.values())) == 12, 'Invalid or duplicate expected UIDs')
    return value


def wire(value):
    need(type(value) is str and len(value) <= 4096 and len(value) % 2 == 0
         and re.fullmatch('[0-9a-fA-F]*', value), 'Invalid recorded receive bytes')
    return bytes.fromhex(value)


def request_key(sequence, bus):
    need(type(sequence) is int and 1 <= sequence <= REQUESTS_PER_BUS,
         'Invalid request sequence')
    if sequence <= 6:
        return BUSES[bus][sequence-1], 'identity', None
    sweep, offset = divmod(sequence-7, 30)
    mid, parameter = BUSES[bus][offset//5], PARAMETERS[offset % 5]
    return mid, parameter, sweep


def replay_events(events, bus):
    """Replay raw bytes and match canonical requests to parameter receipts.

    Type2/21 unsolicited frames are retained as counts and safety observations;
    they are never used as a requested STOP response or stationarity proof.
    """
    need(type(events) is list and len(events) <= 20000, 'Invalid saved event list')
    parser, queued, receipts, transmitted = codec.ATParser(), [], {}, set()
    current, candidate, previous_time, byte_count = None, None, 0, 0
    issues, feedback_count, timeouts = [], 0, 0
    pending_feedback, metadata_count = [], 0
    for event in events:
        need(type(event) is dict, 'Malformed saved event')
        when = stamp(event.get('monotonic_ns'))
        need(when >= previous_time, 'Noncausal saved event times')
        previous_time = when
        kind = event.get('kind')
        if kind == 'can_tx':
            sequence = event.get('sequence')
            mid, parameter, _ = request_key(sequence, bus)
            need(sequence not in transmitted and (not transmitted or sequence > max(transmitted)),
                 'Duplicate or reordered request sequence')
            need(current is None and not queued and not parser.buffer,
                 'Request lacks a fresh receive boundary')
            need(type(event.get('motor_id')) is int and event.get('motor_id') == mid and event.get('parameter') == parameter
                 and wire(event.get('hex')) == codec.read_request(mid, None if parameter == 'identity' else parameter),
                 'Noncanonical, reordered or non-readonly request')
            transmitted.add(sequence)
            current, candidate = event, None
        elif kind == 'can_rx_bytes':
            chunk = wire(event.get('hex'))
            need(bool(chunk), 'Empty receive-byte event')
            byte_count += len(chunk)
            queued.extend((frame, when) for frame in parser.feed(chunk))
        elif kind == 'can_rx_frame':
            need(bool(queued), 'Frame event lacks receive bytes')
            frame, received = queued.pop(0)
            need(when == received and all(event.get(k) == v for k, v in frame.record().items()),
                 'Frame event differs from raw bytes')
            need(not 1 <= frame.source <= 12 or frame.source in BUSES[bus], 'Cross-bus reply')
            metadata = codec.feedback_metadata(frame)
            if metadata is not None:
                feedback_count += 1
                pending_feedback.append((metadata, when))
                # Reevaluate the bytes themselves: deleting metadata cannot
                # hide contradictory enabled/fault feedback from file replay.
                if metadata['type'] == 21 or metadata.get('fault_bits') or metadata.get('mode_state'):
                    issues.append('unsolicited_enabled_or_fault_feedback')
            if current is not None:
                parameter = current['parameter']
                if codec.matches(frame, current['motor_id'], None if parameter == 'identity' else parameter):
                    if candidate is not None:
                        issues.append('multiple_matching_reply_frames')
                    else:
                        candidate = frame, received
        elif kind == 'motor_parameter':
            need(current is not None and candidate is not None, 'Receipt lacks fresh raw reply')
            frame, received = candidate
            mid, parameter = current['motor_id'], current['parameter']
            decoded = codec.decode_reply(frame, mid, None if parameter == 'identity' else parameter)
            need(all(event.get(k) == v and type(event.get(k)) is type(v) for k, v in decoded.items()),
                 'Parameter receipt differs from raw decoding')
            start = current['monotonic_ns']
            need(type(event.get('sequence')) is int and event.get('sequence') == current['sequence']
                 and event.get('request_monotonic_ns') == start and start < received <= when
                 and when-start < 250_000_000, 'Noncausal or late parameter receipt')
            rtt = event.get('round_trip_ms')
            need(type(rtt) in (int, float) and math.isfinite(rtt)
                 and math.isclose(rtt, (when-start)/1e6, rel_tol=0, abs_tol=1e-9),
                 'Round-trip time differs from host times')
            receipts[current['sequence']] = event
            if decoded['ok'] is not True:
                issues.append('failed_parameter_reply')
            current, candidate = None, None
        elif kind == 'motor_feedback':
            need(bool(pending_feedback), 'Unsolicited feedback lacks raw frame event')
            metadata, received = pending_feedback.pop(0)
            need(when == received and all(event.get(k) == v for k, v in metadata.items()),
                 'Unsolicited feedback metadata mismatch')
            metadata_count += 1
        elif kind == 'can_timeout':
            need(current is not None and event.get('motor_id') == current['motor_id']
                 and event.get('parameter') == current['parameter'], 'Unbound timeout')
            timeouts += 1
            issues.append('reply_timeout')
            # No later request is allowed after a poisoned request.
        else:
            raise ValueError('Unknown saved event kind')
    need(not queued, 'Receive frames are missing metadata events')
    if current is not None:
        issues.append('request_without_parameter_receipt')
    if parser.buffer or parser.discarded_bytes:
        issues.append('partial_or_discarded_receive_bytes')
    if pending_feedback:
        issues.append('missing_feedback_metadata_events')
    return {'receipts': receipts, 'queries_sent': len(transmitted), 'rx_bytes': byte_count,
            'parser_discarded_bytes': parser.discarded_bytes, 'parser_residual_hex': bytes(parser.buffer).hex(),
            'missing_raw_request_sequences': sorted(set(range(1, REQUESTS_PER_BUS+1))-transmitted),
            'unsolicited_feedback_count': feedback_count,
            'unsolicited_feedback_metadata_count': metadata_count,
            'missing_feedback_metadata_count': len(pending_feedback),
            'timeout_count': timeouts, 'issues': issues}


def statistics(values):
    if not values:
        return None
    mean = math.fsum(values)/len(values)
    return {'samples': len(values), 'minimum': min(values), 'maximum': max(values),
            'span': max(values)-min(values), 'mean': mean,
            'rms': math.sqrt(math.fsum(v*v for v in values)/len(values)),
            'max_abs': max(abs(v) for v in values)}


def observations(receipts, bus, mid, parameter):
    result = []
    for sequence, reply in sorted(receipts.items()):
        i, p, sweep = request_key(sequence, bus)
        if (i, p) == (mid, parameter) and reply['ok'] is True:
            start, end = reply['request_monotonic_ns'], reply['monotonic_ns']
            result.append({'sweep_index': sweep, 'value': reply['value'],
                           'request_monotonic_ns': start, 'reply_monotonic_ns': end,
                           'host_midpoint_ns': (start+end)//2})
    return result


def timing(points):
    gaps = [(b['host_midpoint_ns']-a['host_midpoint_ns'])/1e6 for a, b in zip(points, points[1:])]
    return {'sample_span_s': (points[-1]['host_midpoint_ns']-points[0]['host_midpoint_ns'])/1e9
            if len(points) > 1 else None,
            'minimum_host_midpoint_gap_ms': min(gaps) if gaps else None,
            'maximum_host_midpoint_gap_ms': max(gaps) if gaps else None,
            'adjacent_gaps': [{'before_sweep': a['sweep_index'], 'after_sweep': b['sweep_index'],
                               'missing_between': b['sweep_index']-a['sweep_index']-1,
                               'host_midpoint_gap_ms': (b['host_midpoint_ns']-a['host_midpoint_ns'])/1e6}
                              for a, b in zip(points, points[1:])]}


def motor_summary(receipts, bus, mid):
    points = {p: observations(receipts, bus, mid, p) for p in PARAMETERS}
    position, velocity = points['position'], points['velocity']
    slope = None
    if len(position) > 1:
        times = [(p['host_midpoint_ns']-position[0]['host_midpoint_ns'])/1e9 for p in position]
        mean_t, mean_p = math.fsum(times)/len(times), math.fsum(p['value'] for p in position)/len(position)
        denominator = math.fsum((t-mean_t)**2 for t in times)
        need(denominator > 0, 'Position times do not span an interval')
        slope = math.fsum((t-mean_t)*(p['value']-mean_p) for t, p in zip(times, position))/denominator
    # Retain all observations as well as every local absolute peak, including
    # ties and edge samples. Missing neighbors remain visible by sweep index.
    peaks = [p for i, p in enumerate(velocity)
             if (i == 0 or abs(p['value']) >= abs(velocity[i-1]['value']))
             and (i == len(velocity)-1 or abs(p['value']) >= abs(velocity[i+1]['value']))]
    maximum = max((abs(p['value']) for p in velocity), default=None)
    paired = {p['sweep_index']: p for p in position}
    return {'statistics': {p: statistics([row['value'] for row in rows]) for p, rows in points.items()},
            'position_OLS_slope_rad_s_host_midpoints': slope,
            'position_span_rad': max(p['value'] for p in position)-min(p['value'] for p in position) if position else None,
            'missing_parameter_sweep_indices': {p: sorted(set(range(SWEEPS))-{r['sweep_index'] for r in rows})
                                               for p, rows in points.items()},
            'parameter_host_timing': {p: timing(rows) for p, rows in points.items()},
            'position_samples': position, 'velocity_samples': velocity,
            'velocity_absolute_local_peaks': peaks,
            'velocity_absolute_maximum_samples': [p for p in velocity if abs(p['value']) == maximum],
            'velocity_peak_definition': 'absolute value >= each recorded neighbor; edges and ties included; all samples retained',
            'velocity_adjacent_changes': [{'before_sweep': a['sweep_index'], 'after_sweep': b['sweep_index'],
                                           'change_rad_s': b['value']-a['value'],
                                           'before_host_midpoint_ns': a['host_midpoint_ns'],
                                           'after_host_midpoint_ns': b['host_midpoint_ns']}
                                          for a, b in zip(velocity, velocity[1:])],
            'position_velocity_read_separation_ns': [
                {'sweep_index': p['sweep_index'], 'velocity_minus_position_host_midpoint_ns':
                 p['host_midpoint_ns']-paired[p['sweep_index']]['host_midpoint_ns']}
                for p in velocity if p['sweep_index'] in paired],
            'observed_current_all_zero': len(points['current']) == SWEEPS and all(p['value'] == 0 for p in points['current']),
            'observed_run_mode_all_zero': len(points['run_mode']) == SWEEPS and all(p['value'] == 0 for p in points['run_mode']),
            'current_nonzero_samples': [p for p in points['current'] if p['value'] != 0],
            'run_mode_nonzero_samples': [p for p in points['run_mode'] if p['value'] != 0]}


def analyze(report, expected_uids, expected_uids_sha256, expected_boot_id=None):
    expected_uids = uids(expected_uids)
    need(type(report) is dict and report.get('schema') == SCHEMA, 'Wrong capture schema')
    boot = boot_id(report.get('boot_id'))
    need(boot_id(report.get('expected_boot_id')) == boot, 'Capture boot binding mismatch')
    if expected_boot_id is not None:
        need(boot_id(expected_boot_id) == boot, 'External boot binding mismatch')
    need(sha256(report.get('expected_uids_sha256')) == sha256(expected_uids_sha256), 'Expected UID file hash mismatch')
    source = report.get('source_sha256')
    need(type(source) is dict and set(source) == set(SOURCE_NAMES), 'Incomplete capture source hash bindings')
    for value in source.values():
        sha256(value)
    need(all(report.get(flag) is False for flag in FALSE_FLAGS)
         and report.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL', 'Invalid capture authorization scope')
    need(report.get('status') in ('COMPLETE_STATIONARY_READONLY_CAPTURE', 'ABORTED_READONLY_CAPTURE'), 'Unknown capture status')
    need(type(report.get('errors')) is list and all(type(e) is str for e in report['errors']), 'Invalid capture errors')
    for key, value in (('duration_s', 10), ('sweeps', 50), ('period_ms', 200),
                       ('requests_per_bus', REQUESTS_PER_BUS), ('max_reply_timeout_ms', 250)):
        need(type(report.get(key)) is int and report[key] == value, 'Wrong fixed capture specification')
    need(report.get('parameters') == list(PARAMETERS) and report.get('allowed_can_types') == [0, 17]
         and report.get('catchup_available') is False, 'Wrong read-only capture plan')
    buses = report.get('buses')
    need(type(buses) is dict and set(buses).issubset(BUSES), 'Invalid capture bus groups')
    result = {'schema': ANALYSIS_SCHEMA, 'status': 'INCOMPLETE_FILE_DIAGNOSTIC', **SCOPE_FLAGS,
              'measurement_scope': 'saved fixed50/200ms/10s Type0/17 raw receipts; host timestamps only',
              'capture_reported_status': report['status'], 'capture_reported_error_count': len(report['errors']),
              'expected_uids_sha256': expected_uids_sha256, 'capture_source_sha256': source,
              'capture_source_contents_independently_verified': False,
              'boot_binding_verified': True, 'external_boot_binding_verified': expected_boot_id is not None,
              'boot_id_sha256': hashlib.sha256(boot.encode()).hexdigest(),
              'fixed_window': {'sweeps': SWEEPS, 'period_ms': 200, 'capture_duration_s': 10},
              'parameter_units': {p: codec.PARAMETERS[p][2] for p in PARAMETERS},
              'validation_errors': [], 'buses': {}, 'per_motor': {},
              'limitations': ['No stationary pass/fail specification is applied to these descriptive statistics.',
                  'The fixed50/10s record is not the fixed21/2s stationarity specification.',
                  'Host midpoint position OLS is not sensor-time velocity and never replaces measured velocity.',
                  'Separate position/velocity reads cannot prove motion between samples or sensor acquisition time.',
                  'run_mode=0 and current=0 do not prove STOP, physical stationarity or output safety.',
                  'No Type2 comparison, angle zero/direction/limits review, dynamic scale or automatic approval is produced.']}
    total_queries, epochs, owners = 0, [], []
    for bus, ids in BUSES.items():
        data = buses.get(bus)
        if data is None:
            result['validation_errors'].append(f'{bus}:missing_bus')
            for mid in ids:
                result['per_motor'][str(mid)] = motor_summary({}, bus, mid)
            continue
        need(type(data) is dict and type(data.get('errors')) is list
             and all(type(e) is str for e in data['errors']), 'Invalid bus capture errors')
        raw = replay_events(data.get('events'), bus)
        receipts = raw.pop('receipts')
        issues = raw['issues']
        total_queries += raw['queries_sent']
        identities = data.get('identities')
        need(type(identities) is dict and set(identities).issubset({str(i) for i in ids}), 'Invalid bus UID keys')
        missing_ids = sorted(set(ids)-{int(i) for i in identities})
        if missing_ids:
            issues.append('missing_identity_records')
        for mid, reply in identities.items():
            sequence = int(mid)-ids[0]+1
            need(type(reply) is dict and receipts.get(sequence) == reply
                 and reply.get('mcu_uid_hex') == expected_uids[mid], 'Raw UID mismatch or unbound identity record')
        for sequence, reply in receipts.items():
            mid, parameter, _ = request_key(sequence, bus)
            if parameter == 'identity':
                need(reply.get('mcu_uid_hex') == expected_uids[str(mid)], 'Raw UID differs from expected motor')
        owner = data.get('owner_thread_id')
        need(type(owner) is int and owner > 0, 'Missing bus owner identity')
        owners.append(owner)
        begin, end = data.get('capture_begin_ns'), data.get('capture_end_ns')
        if begin is None:
            issues.append('missing_capture_epoch')
        else:
            begin = stamp(begin); epochs.append(begin)
            if end is None:
                issues.append('missing_capture_end')
            else:
                end = stamp(end)
                need(end >= begin, 'Capture end precedes start')
                if end-begin < DURATION_NS:
                    issues.append('capture_shorter_than_10s')
            for sequence, reply in receipts.items():
                _, parameter, _ = request_key(sequence, bus)
                if parameter == 'identity':
                    need(reply['monotonic_ns'] <= begin, 'Identity received after capture epoch')
                else:
                    need(begin <= reply['request_monotonic_ns'] < reply['monotonic_ns'] < begin+DURATION_NS,
                         'Parameter receipt lies outside capture deadline')
        sweeps = data.get('sweeps')
        need(type(sweeps) is list and len(sweeps) <= SWEEPS, 'Invalid sweep list')
        indices, represented, scheduling, previous = set(), set(), [], None
        for row in sweeps:
            need(type(row) is dict and type(row.get('index')) is int and 0 <= row['index'] < SWEEPS
                 and row['index'] not in indices and (not indices or row['index'] > max(indices)), 'Duplicate/reordered sweep')
            index = row['index']; indices.add(index)
            released, started = stamp(row.get('requested_release_ns')), stamp(row.get('begin_ns'))
            need(begin is not None and begin+index*PERIOD_NS <= released <= started < begin+DURATION_NS,
                 'Noncausal sweep release/start')
            if index == 0:
                need(released == begin, 'First release differs from capture epoch')
            if previous is not None and index == previous['index']+1 and previous.get('end_ns') is not None:
                need(released == max(previous['begin_ns']+PERIOD_NS, previous['end_ns']),
                     'Sweep release violates no-catchup schedule')
            complete = row.get('complete')
            need(type(complete) is bool, 'Missing sweep completion flag')
            finished = row.get('end_ns')
            if complete:
                finished = stamp(finished)
                need(started <= finished and (end is None or finished <= end), 'Invalid sweep completion time')
            else:
                need(finished is None, 'Incomplete sweep claims completion time')
                issues.append('incomplete_sweep')
            scheduling.append({'sweep_index': index, 'start_ns': started,
                               'start_lateness_ms': (started-released)/1e6,
                               'previous_start_gap_ms': (started-previous['begin_ns'])/1e6 if previous else None,
                               'duration_ms': (finished-started)/1e6 if finished else None})
            samples = row.get('samples')
            need(type(samples) is dict and set(samples).issubset({str(i) for i in ids}), 'Invalid sweep motor keys')
            for mid, parameters in samples.items():
                need(type(parameters) is dict and set(parameters).issubset(PARAMETERS), 'Unknown sweep parameter')
                for parameter, reply in parameters.items():
                    sequence = 7+index*30+(int(mid)-ids[0])*5+PARAMETERS.index(parameter)
                    need(type(reply) is dict and receipts.get(sequence) == reply and reply.get('ok') is True,
                         'Sweep value differs from canonical raw receipt')
                    need(started <= reply['request_monotonic_ns'] < reply['monotonic_ns']
                         and (finished is None or reply['monotonic_ns'] <= finished), 'Receipt lies outside sweep times')
                    represented.add(sequence)
            previous = row
        missing_indices = sorted(set(range(SWEEPS))-indices)
        missing_saved = sorted(set(range(7, REQUESTS_PER_BUS+1))-represented)
        if missing_indices or missing_saved or raw['missing_raw_request_sequences']:
            issues.append('missing_sweeps_or_parameter_records')
        for key in ('queries_sent', 'rx_bytes', 'parser_discarded_bytes', 'parser_residual_hex'):
            if data.get(key) != raw[key] or type(data.get(key)) is not type(raw[key]):
                issues.append('reported_'+key+'_mismatch_or_missing')
        if data['errors']:
            issues.append('reported_bus_errors')
        for mid in ids:
            result['per_motor'][str(mid)] = motor_summary(receipts, bus, mid)
            if result['per_motor'][str(mid)]['run_mode_nonzero_samples']:
                issues.append('collector_required_run_mode_not_zero')
        result['buses'][bus] = {**raw, 'issues': sorted(set(issues)), 'missing_identity_ids': missing_ids,
                               'missing_sweep_indices': missing_indices,
                               'missing_saved_parameter_sequences': missing_saved,
                               'reported_error_count': len(data['errors']), 'sweep_scheduling': scheduling,
                               'capture_span_s': (end-begin)/1e9 if begin is not None and end is not None else None}
        result['validation_errors'].extend(f'{bus}:{issue}' for issue in sorted(set(issues)))
    if len(epochs) != 2 or len(set(epochs)) != 1:
        result['validation_errors'].append('missing_or_different_common_capture_epoch')
    if len(owners) != 2 or len(set(owners)) != 2:
        result['validation_errors'].append('missing_or_shared_bus_owner')
    if type(report.get('queries_sent')) is not int or report['queries_sent'] != total_queries:
        result['validation_errors'].append('total_query_count_mismatch_or_missing')
    if report['errors'] or report['status'] != 'COMPLETE_STATIONARY_READONLY_CAPTURE':
        result['validation_errors'].append('capture_reported_incomplete_or_errored')
    result['queries_replayed'] = total_queries
    result['capture_integrity_verified'] = not result['validation_errors']
    if result['capture_integrity_verified']:
        result['status'] = 'COMPLETE_FILE_DIAGNOSTIC'
    return result


def load_json(path, expected_sha256=None):
    if expected_sha256 is not None:
        sha256(expected_sha256)
    fd = os.open(Path(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= MAX_FILE_BYTES, 'Expected bounded regular JSON file')
        raw = handle.read(MAX_FILE_BYTES+1)
    need(0 < len(raw) <= MAX_FILE_BYTES, 'JSON file exceeds size limit')
    digest = hashlib.sha256(raw).hexdigest()
    need(expected_sha256 is None or expected_sha256 == digest, 'Saved JSON SHA256 mismatch')
    def pairs(rows):
        result = {}
        for key, value in rows:
            need(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def number(value):
        result = float(value)
        need(math.isfinite(result), 'Nonfinite JSON number')
        return result
    def constant(_):
        raise ValueError('Nonfinite JSON constant')
    return json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                      parse_float=number, parse_constant=constant), digest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--expected-sha256')
    parser.add_argument('--expected-uids', type=Path, required=True)
    parser.add_argument('--expected-boot-id')
    args = parser.parse_args(argv)
    try:
        report, digest = load_json(args.report, args.expected_sha256)
        expected, expected_hash = load_json(args.expected_uids)
        result = analyze(report, expected, expected_hash, args.expected_boot_id)
        result['source_report_sha256'] = digest
        result['analysis_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        print(json.dumps(result, sort_keys=True, allow_nan=False))
        return 0 if result['capture_integrity_verified'] else 1
    except (OSError, ValueError, TypeError, UnicodeError, OverflowError, KeyError) as error:
        print(json.dumps({'schema': ANALYSIS_SCHEMA, 'status': 'ANALYSIS_REJECTED',
                          'error_type': type(error).__name__, **SCOPE_FLAGS}))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
