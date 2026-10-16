"""Read-only correlation of sealed CPU/AF_UNIX probe and lossless RestoreThread trace.

This summarizes the probe's restore path, never qualifies a physical robot or
attributes a previous physical cycle to GIL waiting.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from . import trace_analyze as TA
from . import trace_scope as TS

SCHEMA = 'singularitydog.type1-latency-cause-summary.v1'
PROBE_SCHEMA = 'singularitydog.type1-latency-python-stages.v1'
TRACE_SCHEMA = 'singularitydog.type1-cause-trace.v1'
FALSE_FLAGS = ('CAN_IO', 'serial_IO', 'physical_type1_sent', 'motor_enable_sent',
               'positive_gain_sent', 'learned_targets_sent', 'output_allowed',
               'runtime_qualified', 'original_r3_report_relabeled', 'physical_current_guard_used')
COUNTERS = ('thread_cpu_ns', 'user_cpu_ns', 'system_cpu_ns', 'nvcsw', 'nivcsw',
            'minflt', 'majflt', 'sched_cpu_ns', 'sched_run_delay_ns', 'sched_timeslices')


def need(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def pinned(path, expected_sha):
    path = Path(path)
    need(path.is_absolute() and path.is_file() and not path.is_symlink(), 'Original regular absolute input required')
    raw = path.read_bytes()
    need(type(expected_sha) is str and digest(raw) == expected_sha, 'Input SHA differs: ' + path.name)
    return raw


def integer(value, name):
    need(type(value) is int and value >= 0, 'Nonnegative integer required: ' + name)
    return value


def percentiles(values):
    """Nearest-rank percentiles, explicitly retaining sample count and extrema."""
    values = sorted(values)
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'max': None, 'min': None}
    result = {'count': len(values), 'min': values[0], 'max': values[-1]}
    for q in (50, 95, 99):
        result['p' + str(q)] = values[max(0, math.ceil(len(values) * q / 100) - 1)]
    return result


def validate(probe, trace_report, trace_raw):
    need(type(probe) is dict and probe.get('schema') == PROBE_SCHEMA, 'CPU probe schema differs')
    need(probe.get('status') == 'COMPLETE_CPU_PROBE' and probe.get('errors') == [], 'Incomplete CPU probe')
    for name in FALSE_FLAGS:
        need(probe.get(name) is False, 'Hardware/qualification scope differs: ' + name)
    for name in ('instrumentation_restored', 'original_source_unchanged', 'virtual_last_output_seed', 'virtual_boot_fixture'):
        need(probe.get(name) is True, 'Probe restoration or explicit virtual scope missing: ' + name)
    need(probe.get('counter_detail') in ('minimal', 'full'), 'Counter detail differs')
    need(probe.get('take_mode') == 'exact_original_pipeline_take_with_counted_cpu_only_noop_hot', 'Exact take/no-guard scope differs')
    need(probe.get('native_stats_end_is_not_exact_function_return') is True and
         probe.get('gil_or_os_wait_diagnosed_from_wall_alone') is False, 'Native-end interpretation differs')
    pid = integer(probe.get('pid'), 'probe pid')
    tids = probe.get('owner_native_tids')
    need(type(tids) is list and len(tids) in (1, 4) and len(set(tids)) == len(tids) and
         all(type(t) is int and t > 0 for t in tids), 'Exact distinct owner native TIDs required')
    need(probe.get('ports') == len(tids), 'Port/TID count differs')
    main = probe.get('main_placement', {})
    need(main.get('supported') is True and main.get('restored') is True and
         main.get('requested_cpus') == main.get('during_cpus') == [4] and main.get('errors') == [],
         'Linux main CPU placement/restoration required')
    owners = probe.get('owner_placement', {})
    need(type(owners) is dict and len(owners) == len(tids), 'Owner placement reports required')
    for i, tid in enumerate(tids):
        row = owners.get('port' + str(i), {})
        need(row.get('native_tid') == tid and row.get('restored') is True and row.get('supported') is True and
             row.get('requested_cpus') == row.get('during_cpus') == [i] and row.get('errors') == [],
             'Owner CPU/TID/restoration differs')
    peer = probe.get('peer', {})
    pp = peer.get('placement', {})
    need(peer.get('errors') == [] and peer.get('motor_enable_sent') is False and peer.get('positive_gain_sent') is False and
         pp.get('restored') is True and pp.get('requested_cpus') == pp.get('during_cpus') == [0, 1, 2, 3] and
         pp.get('errors') == [], 'Peer scope/restoration differs')
    need(probe.get('gc_scope', {}).get('restored') is True and
         probe.get('python_switch_interval', {}).get('restored') is True and
         probe.get('python_switch_interval', {}).get('during_s') == .0001, 'GC/switch restoration missing')
    completed = integer(probe.get('completed_cycles'), 'completed cycles')
    requested = integer(probe.get('requested_cycles'), 'requested cycles')
    need(integer(probe.get('failed_cycles'), 'failed cycles') == 0 and completed == requested > 0,
         'Incomplete cycle counters')
    expected_ports = {'port' + str(i) for i in range(len(tids))}
    need(type(probe.get('groups')) is dict and set(probe['groups']) == expected_ports, 'Exact virtual groups required')
    markers = [event for event in probe.get('events', []) if event.get('stage') == 'native_first_python_statement']
    keys = [(event.get('port'), event.get('cycle'), event.get('exchange')) for event in markers]
    expected_keys = {(port, cycle, exchange) for port in expected_ports for cycle in range(completed)
                     for exchange in ('feedback_hold', 'voltage')}
    need(len(keys) == len(expected_keys) and set(keys) == expected_keys, 'Native marker coverage incomplete or duplicated')
    need(type(trace_report) is dict and trace_report.get('schema') == TRACE_SCHEMA, 'Trace scope schema differs')
    need(trace_report.get('status') == 'RECORDED_CPU_ONLY_TRACE', 'Complete trace scope required')
    need(trace_report.get('motor_hardware_opened') is False and trace_report.get('output_allowed') is False,
         'Trace hardware scope differs')
    need(trace_report.get('restored') is True and trace_report.get('source_unchanged') is True and
         trace_report.get('child_reaped') is True and trace_report.get('cleanup_errors') == [], 'Trace restoration incomplete')
    need(trace_report.get('child_exit_code') == 0 and trace_report.get('child_workload_completed') is True,
         'Trace child failed')
    need(trace_report.get('child_pid') == pid, 'Trace child pid differs from probe')
    need(trace_report.get('probe_sha256') == probe.get('source_sha256'), 'Probe source differs from traced source')
    points = trace_report.get('probe_points', {})
    semantic = 'After PACIASP before frame save, through take_gil, to authenticated pre-tail branch; excludes thread-state swap/return tail'
    need(trace_report.get('probe_kind') == points.get('kind') == 'arm64_pac_safe_gil_acquisition_path' and
         trace_report.get('endpoint_semantics') == points.get('semantics') == semantic and
         trace_report.get('lr_hijack_used') is points.get('lr_hijack_used') is False,
         'Reviewed PAC-safe acquisition endpoints required; legacy full-return semantics not combined')
    need(trace_report.get('python_sha256') == points.get('reviewed_executable_sha256') == TS.ARM64_PAC_EXEC_SHA256,
         'Reviewed native ELF SHA differs')
    offset = integer(trace_report.get('restore_thread_file_offset'), 'native function file offset')
    need(points.get('function_entry_file_offset') == offset, 'Native function endpoint offset differs')
    selected = points.get('points')
    need(type(selected) is list and len(selected) == 2 and
         [(p.get('kind'), p.get('name'), p.get('file_offset')) for p in selected] ==
         [('p', 'acquire_entry', offset + 4), ('p', 'acquire_exit', offset + 44)], 'Exact reviewed normal-probe points required')
    try:
        instruction_bytes = bytes.fromhex(points.get('reviewed_instruction_bytes_hex', ''))
        words = struct.unpack('<12I', instruction_bytes)
    except (ValueError, struct.error) as error:
        raise ValueError('Original reviewed instruction image required') from error
    fixed = {0: 0xd503233f, 1: 0xa9be7bfd, 2: 0x910003fd, 3: 0xf9000bf3,
             4: 0xb4000100, 5: 0xaa0003f3, 7: 0xaa1303e0, 8: 0xf9400bf3,
             9: 0xa8c27bfd, 10: 0xd50323bf}
    need(all(words[i] == value for i, value in fixed.items()) and words[6] & 0xfc000000 == 0x94000000 and
         words[11] & 0xfc000000 == 0x14000000 and selected[0].get('opcode_hex') == f'{words[1]:08x}' and
         selected[1].get('opcode_hex') == f'{words[11]:08x}', 'PAC/frame/take_gil/authenticated-tail image differs')
    need(trace_report.get('clock') == 'mono' and '[mono]' in trace_report.get('trace_clock_readback', ''),
         'Matching MONOTONIC trace clock required')
    need(type(trace_report.get('timestamp_text_precision_ns')) is int and
         trace_report['timestamp_text_precision_ns'] == 1000, 'Reviewed 1us trace text precision required')
    need(trace_report.get('trace_sha256') == digest(trace_raw) and trace_report.get('trace_bytes') == len(trace_raw),
         'Trace raw bytes differ from original scope report')
    need(type(trace_report.get('trace_buffer_loss_events')) is int and trace_report['trace_buffer_loss_events'] == 0,
         'Trace loss unknown or positive')
    stats = trace_report.get('per_cpu_stats')
    need(type(stats) is dict and stats, 'Per-CPU trace loss original data required')
    losses = [int(m.group(1)) for value in stats.values() for m in
              re.finditer(r'^(?:overrun|commit overrun|dropped events):\s*(\d+)', value, re.M)]
    need(losses and sum(losses) == 0, 'Original per-CPU trace loss unknown or positive')
    return set(tids)


def analyze_owner_trace(text, owner_tids):
    """Original parser/state splitter, indexed to avoid N-interval x N-event work."""
    events = TA.parse_trace(text)
    scheduler, stacks, paired, unmatched_returns = defaultdict(list), defaultdict(list), [], []
    for event in events:
        if event['kind'] == 'sched_switch':
            match = TA.SWITCH.search(event['payload'])
            if match:
                for tid in {int(match.group(1)), int(match.group(3))} & owner_tids:
                    scheduler[tid].append(event)
        elif event['kind'] == 'sched_wakeup':
            match = TA.WAKE.search(event['payload'])
            if match and int(match.group(1)) in owner_tids:
                scheduler[int(match.group(1))].append(event)
        elif event['tid'] in owner_tids:
            family = 'acquire_path' if event['kind'].startswith('acquire_') else 'full_restore'
            key = event['tid'], family
            if event['kind'] in ('restore_entry', 'acquire_entry'):
                stacks[key].append(event['ns'])
            elif event['kind'] in ('restore_return', 'acquire_exit'):
                if stacks[key]:
                    paired.append((event['tid'], family, stacks[key].pop(), event['ns']))
                else:
                    unmatched_returns.append(event)
    times = {tid: [event['ns'] for event in rows] for tid, rows in scheduler.items()}
    intervals = []
    for tid, family, begin, end in paired:
        relevant, clock = scheduler[tid], times.get(tid, [])
        rows = relevant[bisect_left(clock, begin):bisect_right(clock, end)]
        intervals.append({'tid': tid, 'begin_ns': begin, 'end_ns': end, 'duration_ns': end - begin,
                          'endpoint_kind': family,
                          'endpoint_semantics': ('Post-PAC frame entry through GIL acquisition to authenticated pre-tail branch; excludes thread-state swap/return tail'
                                                 if family == 'acquire_path' else
                                                 'PyEval_RestoreThread entry to function return; includes thread-state tail'),
                          'lr_hijack_used': family != 'acquire_path',
                          **TA.scheduler_parts(rows, tid, begin, end)})
    return {'intervals': intervals, 'interval_count': len(intervals),
            'unmatched_entries': [{'tid': tid, 'endpoint_kind': family, 'begin_ns': ns}
                                  for (tid, family), values in stacks.items() for ns in values],
            'unmatched_returns': unmatched_returns}


def correlate(probe, trace_report, trace_text):
    owner_tids = set(probe['owner_native_tids'])
    analysis = analyze_owner_trace(trace_text, owner_tids)
    precision = trace_report['timestamp_text_precision_ns']
    result, matches_used = [], defaultdict(list)
    by_tid = defaultdict(list)
    for index, interval in enumerate(analysis['intervals']):
        by_tid[interval['tid']].append((index, interval))
    for entries in by_tid.values():
        entries.sort(key=lambda pair: pair[1]['begin_ns'])
    entry_clocks = {tid: [row['begin_ns'] for _, row in entries] for tid, entries in by_tid.items()}
    for index, event in enumerate(probe['events']):
        if event.get('stage') != 'native_first_python_statement':
            continue
        tid = integer(event.get('tid'), 'native marker tid')
        need(tid in owner_tids, 'Native marker is not an admitted owner TID')
        begin = integer(event.get('native_stats_end_ns'), 'native end')
        end = integer(event.get('wall_ns'), 'first Python stamp')
        need(0 < begin <= end and event.get('port') in probe['groups'], 'Causal native/Python window required')
        need(probe['owner_placement'][event['port']]['native_tid'] == tid, 'Native marker port/TID binding differs')
        entries, clocks = by_tid[tid], entry_clocks.get(tid, [])
        candidates = [(i, row) for i, row in entries[bisect_left(clocks, begin - precision):bisect_right(clocks, end + precision)]
                      if row['end_ns'] <= end + precision and row['endpoint_kind'] == 'acquire_path']
        incomplete = [row for row in analysis['unmatched_entries'] if row['tid'] == tid and row['endpoint_kind'] == 'acquire_path' and
                      begin - precision <= row['begin_ns'] <= end + precision]
        incomplete += [row for row in analysis['unmatched_returns'] if row['tid'] == tid and row['kind'] == 'acquire_exit' and
                       begin - precision <= row['ns'] <= end + precision]
        row = {'event_index': index, 'port': event['port'], 'tid': tid,
               'cycle': event['cycle'], 'exchange': event['exchange'],
               'native_status': event['native_status'], 'native_stats_end_ns': begin,
               'first_python_statement_ns': end, 'native_to_python_wall_ns': end - begin,
               'trace_text_precision_ns': precision, 'candidate_count': len(candidates),
               'unmatched_inside_window_count': len(incomplete), 'status': 'UNPROVEN',
               'pure_gil_wait_proven': False, 'original_physical_cycle_cause_proven': False}
        if len(candidates) == 1 and not incomplete and event['native_status'] == 0:
            matched_index, interval = candidates[0]
            matches_used[matched_index].append(len(result))
            row.update(status='MATCHED_PROBE_RESTORE_PATH', restore_interval=interval,
                       restore_path_has_blocked_sleep_witness=interval['sleeping_ns'] > 0,
                       meaning=trace_report['endpoint_semantics'],
                       precision_expanded_edges=interval['begin_ns'] < begin or interval['end_ns'] > end)
        else:
            row['reason'] = ('No exact same-TID contained interval' if not candidates else
                             'Multiple contained intervals' if len(candidates) > 1 else
                             'Incomplete same-TID interval within window' if incomplete else 'Native call failed')
        result.append(row)
    for indices in matches_used.values():
        if len(indices) > 1:
            for index in indices:
                result[index].update(status='UNPROVEN', reason='One trace interval reused by multiple native windows')
                result[index].pop('restore_interval', None)
                result[index].pop('restore_path_has_blocked_sleep_witness', None)
    used = {i for i, indices in matches_used.items() if len(indices) == 1}
    return result, {'owner_restore_intervals_total': analysis['interval_count'],
                    'contained_unique_matched_intervals': len(used),
                    'other_owner_restore_intervals_not_aggregated': analysis['interval_count'] - len(used),
                    'unmatched_owner_entries': analysis['unmatched_entries'],
                    'unmatched_owner_returns': analysis['unmatched_returns'],
                    'unrelated_tids_excluded': True}


def stage_rows(probe):
    events = probe['events']
    rows = []
    starts = {'native_call_before': 'native_first_python_statement',
              'checked_decode_before': 'checked_decode_after',
              'batch_constructor_before': 'batch_constructor_after',
              'voltage_buffers_prepare_before': 'voltage_buffers_prepare_after',
              'voltage_preflight_hook_before': 'voltage_preflight_hook_after',
              'current_check_before': 'current_check_after',
              'prefix_set_result_before': 'prefix_set_result_after',
              'outer_set_result_before': 'outer_set_result_after'}
    pending = {}
    for event in events:
        integer(event['wall_ns'], 'event wall')
        for start, finish in starts.items():
            key = event['tid'], event['port'], event['cycle'], event['exchange'], start
            if event['stage'] == start:
                need(key not in pending, 'Duplicate unmatched stage start')
                pending[key] = event
            elif event['stage'] == finish and key in pending:
                previous = pending.pop(key)
                need(event['wall_ns'] >= previous['wall_ns'], 'Stage timestamps reversed')
                row = {'stage': start.removesuffix('_before'), 'port': event['port'], 'cycle': event['cycle'],
                       'exchange': event['exchange'], 'wall_ns': event['wall_ns'] - previous['wall_ns'],
                       'counter_window_start_ns': previous['sample_started_ns'],
                       'counter_window_end_ns': event['sample_finished_ns']}
                for name in COUNTERS:
                    a, b = previous.get(name), event.get(name)
                    if a is not None and b is not None:
                        integer(a, name); integer(b, name)
                        need(b >= a, 'Calling-thread counter reversed: ' + name)
                        row[name] = b - a
                    else:
                        row[name] = None
                rows.append(row)
    need(not pending, 'Stage starts without complete original end markers')
    # Original Batch.completed is the last constructor ARGUMENT stamp.
    by_key = defaultdict(list)
    for event in events:
        by_key[event['port'], event['cycle']].append(event)
    for (port, cycle), port_events in by_key.items():
        if port == 'main':
            continue
        for exchange in ('feedback_hold', 'voltage'):
            selected = {e['stage']: e for e in port_events if e['exchange'] == exchange}
            first, batch, registry = (selected.get(name) for name in
                ('native_first_python_statement', 'batch_constructor_after', 'batch_registry_complete'))
            need(first and batch and registry, 'Missing native/Batch/registry original marker')
            argument = integer(batch.get('batch_argument_stamp_ns'), 'Batch argument stamp')
            need(first['wall_ns'] <= argument <= batch['wall_ns'] <= registry['wall_ns'], 'Batch boundary order differs')
            for name, begin, end in (
                    ('first_python_to_batch_argument', first['wall_ns'], argument),
                    ('batch_argument_to_constructor_marker', argument, batch['wall_ns']),
                    ('constructor_marker_to_registry_complete', batch['wall_ns'], registry['wall_ns'])):
                rows.append({'stage': name, 'port': port, 'cycle': cycle, 'exchange': exchange,
                             'wall_ns': end - begin, 'cross_boundary_wall_only': True})
            publication = next((e for e in port_events if e['stage'] ==
                                ('prefix_set_result_after' if exchange == 'feedback_hold' else 'outer_set_result_after')), None)
            need(publication and publication['wall_ns'] >= registry['wall_ns'], 'Future publication marker missing/reversed')
            rows.append({'stage': 'batch_argument_to_' + ('prefix' if exchange == 'feedback_hold' else 'outer') + '_publication_after',
                         'port': port, 'cycle': cycle, 'exchange': exchange,
                         'wall_ns': publication['wall_ns'] - argument, 'cross_boundary_wall_only': True})
    return rows


def aggregate(rows, metric='wall_ns'):
    values = defaultdict(list)
    for row in rows:
        value = row.get(metric)
        if value is not None:
            values[row['stage'], row['port'], row.get('exchange')].append(value)
    return [{'stage': stage, 'port': port, 'exchange': exchange, 'metric': metric,
             'unit': 'counts' if metric in ('nvcsw', 'nivcsw', 'minflt', 'majflt', 'sched_timeslices') else 'ns',
             **percentiles(data)} for (stage, port, exchange), data in sorted(values.items(), key=str)]


def summarize(probe_path, probe_sha, trace_report_path, trace_report_sha, trace_path, trace_sha):
    raw_probe, raw_report, raw_trace = (pinned(probe_path, probe_sha), pinned(trace_report_path, trace_report_sha),
                                      pinned(trace_path, trace_sha))
    probe, trace_report = json.loads(raw_probe), json.loads(raw_report)
    validate(probe, trace_report, raw_trace)
    correlations, coverage = correlate(probe, trace_report, raw_trace.decode('utf-8', errors='strict'))
    rows = stage_rows(probe)
    matched = [row for row in correlations if row['status'] == 'MATCHED_PROBE_RESTORE_PATH']
    restore_rows = [dict(stage='matched_probe_restore_path', port=row['port'], exchange=row['exchange'],
                         wall_ns=row['restore_interval']['duration_ns'],
                         sleeping_ns=row['restore_interval']['sleeping_ns'],
                         runnable_wait_ns=row['restore_interval']['runnable_wait_ns'],
                         unknown_off_cpu_ns=row['restore_interval']['unknown_off_cpu_ns']) for row in matched]
    return {'schema': SCHEMA, 'status': 'READONLY_CPU_PROBE_SUMMARY',
        'inputs': {'probe': {'path': str(probe_path), 'sha256': probe_sha, 'bytes': len(raw_probe)},
                   'trace_report': {'path': str(trace_report_path), 'sha256': trace_report_sha, 'bytes': len(raw_report)},
                   'trace': {'path': str(trace_path), 'sha256': trace_sha, 'bytes': len(raw_trace)}},
        'probe_source_sha256': probe['source_sha256'], 'transport_source_sha256': probe['transport_source_sha256'],
        'python_elf_sha256': trace_report['python_sha256'], 'trace_probe_points': trace_report['probe_points'],
        'trace_endpoint_semantics': trace_report['endpoint_semantics'],
        'native_pins': probe['native_pins'], 'counter_detail': probe['counter_detail'],
        'completed_cycles': probe['completed_cycles'], 'ports': probe['ports'],
        'stage_statistics': aggregate(rows),
        'same_thread_counter_upper_window_statistics': {name: aggregate(rows, name) for name in COUNTERS},
        'restore_path_statistics': aggregate(restore_rows),
        'restore_path_scheduler_statistics': {name: aggregate(restore_rows, name) for name in
                                             ('sleeping_ns', 'runnable_wait_ns', 'unknown_off_cpu_ns')},
        'correlations': correlations, 'trace_coverage': coverage,
        'matched_windows': len(matched), 'unproven_windows': len(correlations) - len(matched),
        'original_scope_flags': {name: probe[name] for name in FALSE_FLAGS},
        'sampling_overhead_included': True, 'native_stats_end_is_exact_return': False,
        'counter_deltas_are_sampling_window_upper_bounds': True,
        'pure_gil_wait_proven': False, 'original_physical_cycle_cause_proven': False,
        'conclusion_scope': 'CPU/AF_UNIX probe restore path only; no past physical-cycle attribution',
        'CAN_IO': False, 'serial_IO': False, 'output_allowed': False, 'runtime_qualified': False,
        'summary_source_sha256': digest(Path(__file__).read_bytes()),
        'trace_analysis_source_sha256': digest(Path(TA.__file__).read_bytes()),
        'trace_analysis_engine': 'original parser/state splitter; same-TID time-indexed windows'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('probe', 'trace-report', 'trace'):
        parser.add_argument('--' + name, required=True, type=Path)
        parser.add_argument('--' + name + '-sha256', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    need(args.output.is_absolute() and not args.output.exists() and
         not any(p.is_symlink() for p in (args.output, *args.output.parents)), 'Fresh absolute summary output required')
    report = summarize(args.probe, args.probe_sha256, args.trace_report, args.trace_report_sha256,
                       args.trace, args.trace_sha256)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in ('status', 'matched_windows', 'unproven_windows',
                                                'pure_gil_wait_proven', 'output_allowed')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
