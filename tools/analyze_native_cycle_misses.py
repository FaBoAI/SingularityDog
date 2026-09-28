"""Classify saved STOP-proxy deadline misses from host timestamps only.

Thresholds locate diagnostic outliers; this analysis does not prove the cause
of a delay or certify motor-output timing.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        raise ValueError('No timing samples')
    return {'median': statistics.median(ordered),
            'p99': ordered[math.ceil(.99*len(ordered))-1],
            'max': ordered[-1]}


def analyze(report, records, *, inference_ms=5., dispatch_ms=1.):
    if (report.get('status') != 'COMPLETE_DIAGNOSTIC' or
            report.get('mode') != 'stop-proxy' or
            report.get('motor_enable_sent') is not False or
            report.get('learned_targets_sent') is not False or
            type(inference_ms) not in (int, float) or inference_ms <= 0 or
            type(dispatch_ms) not in (int, float) or dispatch_ms <= 0):
        raise ValueError('Require a complete no-enable STOP-proxy diagnostic')
    measurements = report.get('measurements')
    if (type(measurements) is not list or type(records) is not list or
            not measurements or len(measurements) != len(records) or
            report.get('cycles_completed') != len(records)):
        raise ValueError('Report/record cycle count mismatch')
    cycle_rows = []
    for index, (measurement, record) in enumerate(zip(measurements, records), 1):
        if record.get('cycle') != index:
            raise ValueError('Record cycle order mismatch')
        inferred = measurement.get('infer_end_ns')
        release = measurement.get('release_ns')
        end = measurement.get('cycle_end_ns')
        observed_inference = measurement.get('inference_ms')
        if (not all(type(t) is int and t > 0 for t in (release, inferred, end)) or
                not release <= inferred <= end or
                type(observed_inference) not in (int, float) or
                not math.isfinite(observed_inference) or observed_inference < 0):
            raise ValueError('Invalid cycle timing')
        begins = []
        native_to_write = []
        output_reply_times = []
        for bus in ('front', 'rear'):
            batch = record.get('output', {}).get(bus, {})
            begin = batch.get('stats', {}).get('begin_ns')
            frames = batch.get('records')
            if (type(begin) is not int or not isinstance(frames, list) or
                    len(frames) != 6 or not all(f.get('written') == f.get('received') == 17
                                                for f in frames)):
                raise ValueError('Missing complete output batch')
            if not all(type(f.get('start_ns')) is int and f['start_ns'] > 0
                       for f in frames):
                raise ValueError('Invalid output write time')
            for frame in frames:
                received = frame.get('received_ns')
                if received is not None:
                    if type(received) is not int or not frame['start_ns'] <= received <= end:
                        raise ValueError('Invalid output reply time')
                    output_reply_times.append(received)
            first_write = min(f.get('start_ns') for f in frames)
            if type(first_write) is not int or not inferred <= begin <= first_write <= end:
                raise ValueError('Noncausal output dispatch')
            begins.append(begin)
            native_to_write.append((first_write-begin)/1e6)
        pre_native = (min(begins)-inferred)/1e6
        whole = (end-release)/1e6
        if abs(whole-measurement.get('whole_iteration_ms', float('nan'))) > 1e-5:
            raise ValueError('Whole iteration differs from timestamp')
        observed = record.get('observed') or {}
        if type(observed) is not dict:
            raise ValueError('Invalid observed profile')
        profile = observed.get('consume_profile') or {}
        if type(profile) is not dict:
            raise ValueError('Invalid consume profile')
        model_call_ns = profile.get('durations_ns', {}).get('model_call')
        if model_call_ns is not None and (type(model_call_ns) is not int or
                not 0 <= model_call_ns <= end-release):
            raise ValueError('Invalid model-call duration')
        cycle_rows.append({'cycle': index, 'whole_iteration_ms': whole,
                           'inference_ms': observed_inference,
                           'model_call_ms': model_call_ns/1e6
                               if model_call_ns is not None else None,
                           'pre_native_output_ms': pre_native,
                           'native_to_first_write_ms_by_bus': native_to_write,
                           'release_ns': release, 'cycle_end_ns': end,
                           'post_last_output_reply_ms':
                               (end-max(output_reply_times))/1e6
                               if output_reply_times else None})
    misses = [row for row in cycle_rows if row['whole_iteration_ms'] > 20.]
    inferred_slow = [row['cycle'] for row in misses if row['inference_ms'] >= inference_ms]
    dispatch_slow = [row['cycle'] for row in misses if row['pre_native_output_ms'] > dispatch_ms]
    other = [row['cycle'] for row in misses if row['cycle'] not in
             set(inferred_slow) | set(dispatch_slow)]
    result = {'cycles': len(cycle_rows), 'whole_iteration_over_20ms': len(misses),
            'thresholds': {'inference_ms_at_least': inference_ms,
                           'pre_native_output_ms_over': dispatch_ms},
            'inference_slow_miss_cycles': inferred_slow,
            'pre_native_output_slow_miss_cycles': dispatch_slow,
            'other_miss_cycles': other,
            'pre_native_output_ms': distribution([r['pre_native_output_ms'] for r in cycle_rows]),
            'native_to_first_write_ms_by_bus': {
                bus: distribution([r['native_to_first_write_ms_by_bus'][i] for r in cycle_rows])
                for i, bus in enumerate(('front', 'rear'))},
            'pre_native_output_slow_rows': [r for r in cycle_rows
                                            if r['pre_native_output_ms'] > dispatch_ms],
            'interpretation': 'Host timing correlation only; GC and USB cause not established'}
    if all(row['model_call_ms'] is not None for row in cycle_rows):
        result['model_call_budget'] = {
            'model_call_ms': distribution([row['model_call_ms'] for row in cycle_rows]),
            'other_cycle_work_ms': distribution([
                row['whole_iteration_ms']-row['model_call_ms'] for row in cycle_rows]),
            'miss_rows': [{'cycle': row['cycle'],
                           'model_call_ms': row['model_call_ms'],
                           'available_model_budget_ms':
                               20.-(row['whole_iteration_ms']-row['model_call_ms']),
                           'minimum_saving_needed_ms': row['whole_iteration_ms']-20.}
                          for row in misses]}
    trace = report.get('output_dispatch_trace')
    if trace is not None:
        result['output_dispatch_detail'] = analyze_dispatch_trace(trace, cycle_rows, dispatch_ms)
    return result


def analyze_dispatch_trace(trace, cycles, slow_threshold_ms):
    """Decompose an opt-in native-output-dispatch-v1 trace, without causal claims."""
    required = ('infer_end_ns', 'main_check_start_ns', 'main_check_end_ns',
                'front_submit_end_ns', 'rear_submit_end_ns',
                'front_worker_enter_ns', 'front_worker_check_end_ns',
                'front_native_begin_ns', 'front_first_write_ns',
                'rear_worker_enter_ns', 'rear_worker_check_end_ns',
                'rear_native_begin_ns', 'rear_first_write_ns',
                'main_infer_thread_cpu_ns', 'main_submits_end_thread_cpu_ns')
    if (type(trace) is not dict or trace.get('schema') != 'native-output-dispatch-v1' or
            trace.get('fields') != list(required) or
            type(trace.get('rows')) is not list or len(trace['rows']) != len(cycles) or
            type(trace.get('gc_events')) is not list):
        raise ValueError('Invalid output-dispatch trace schema')
    detail = []
    for cycle, raw in zip(cycles, trace['rows']):
        if (type(raw) is not list or len(raw) != len(required) or
                not all(type(t) is int and t > 0 for t in raw)):
            raise ValueError('Incomplete output-dispatch row')
        row = dict(zip(required, raw))
        if not (row['infer_end_ns'] <= row['main_check_start_ns'] <=
                row['main_check_end_ns'] <= row['front_submit_end_ns'] <=
                row['rear_submit_end_ns'] and
                row['main_infer_thread_cpu_ns'] <= row['main_submits_end_thread_cpu_ns']):
            raise ValueError('Noncausal main output dispatch')
        for bus in ('front', 'rear'):
            if not (row[f'{bus}_worker_enter_ns'] <= row[f'{bus}_worker_check_end_ns'] <=
                    row[f'{bus}_native_begin_ns'] <= row[f'{bus}_first_write_ns']):
                raise ValueError('Noncausal worker output dispatch')
        if abs((min(row['front_native_begin_ns'],row['rear_native_begin_ns'])-
                row['infer_end_ns'])/1e6-cycle['pre_native_output_ms']) > 1e-5:
            raise ValueError('Dispatch trace differs from native record time')
        detail.append({'cycle': cycle['cycle'],
                       'main_check_ms': (row['main_check_end_ns']-row['main_check_start_ns'])/1e6,
                       'main_submit_ms': (row['rear_submit_end_ns']-row['main_check_end_ns'])/1e6,
                       'main_cpu_infer_to_submits_ms':
                           (row['main_submits_end_thread_cpu_ns']-
                            row['main_infer_thread_cpu_ns'])/1e6,
                       'front_worker_entry_minus_submit_ms':
                           (row['front_worker_enter_ns']-row['front_submit_end_ns'])/1e6,
                       'rear_worker_entry_minus_submit_ms':
                           (row['rear_worker_enter_ns']-row['rear_submit_end_ns'])/1e6,
                       'front_submit_to_native_ms':
                           (row['front_native_begin_ns']-row['front_submit_end_ns'])/1e6,
                       'rear_submit_to_native_ms':
                           (row['rear_native_begin_ns']-row['rear_submit_end_ns'])/1e6,
                       'front_worker_check_ms':
                           (row['front_worker_check_end_ns']-row['front_worker_enter_ns'])/1e6,
                       'rear_worker_check_ms':
                           (row['rear_worker_check_end_ns']-row['rear_worker_enter_ns'])/1e6,
                       'start_ns': row['infer_end_ns'],
                       'end_ns': min(row['front_native_begin_ns'],row['rear_native_begin_ns'])})
    gc_events = trace['gc_events']
    for event in gc_events:
        if (type(event) is not dict or type(event.get('monotonic_ns')) is not int or
                event.get('phase') not in ('start','stop') or
                type(event.get('generation')) is not int or
                type(event.get('native_tid')) is not int or
                type(event.get('cycle')) is not int):
            raise ValueError('Invalid GC trace event')
    active = {}
    gc_intervals = []
    for event in sorted(gc_events, key=lambda item: item['monotonic_ns']):
        key = event['native_tid'], event['generation']
        if event['phase'] == 'start':
            active[key] = event
        elif key in active:
            begin = active.pop(key)
            if event['monotonic_ns'] < begin['monotonic_ns']:
                raise ValueError('Noncausal GC trace interval')
            gc_intervals.append({'start_ns': begin['monotonic_ns'],
                                 'end_ns': event['monotonic_ns'],
                                 'duration_ms':
                                     (event['monotonic_ns']-begin['monotonic_ns'])/1e6,
                                 'native_tid': event['native_tid'],
                                 'generation': event['generation'],
                                 'start_cycle': begin['cycle']})
    slow = []
    for cycle, row in zip(cycles, detail):
        if cycle['pre_native_output_ms'] > slow_threshold_ms:
            slow.append({key:value for key,value in row.items() if key not in ('start_ns','end_ns')}
                        | {'gc_events_in_interval': [event for event in gc_events
                            if row['start_ns'] <= event['monotonic_ns'] <= row['end_ns']]})
    missed = []
    for cycle, row in zip(cycles, detail):
        if cycle['whole_iteration_ms'] <= 20.:
            continue
        overlaps = [dict(interval) for interval in gc_intervals
                    if interval['start_ns'] < cycle['cycle_end_ns'] and
                    interval['end_ns'] > cycle['release_ns']]
        missed.append({'cycle': cycle['cycle'],
                       'whole_iteration_ms': cycle['whole_iteration_ms'],
                       'inference_ms': cycle['inference_ms'],
                       'model_call_ms': cycle['model_call_ms'],
                       'front_submit_to_native_ms': row['front_submit_to_native_ms'],
                       'rear_submit_to_native_ms': row['rear_submit_to_native_ms'],
                       'post_last_output_reply_ms': cycle['post_last_output_reply_ms'],
                       'gc_intervals_overlapping_cycle': overlaps})
    return {'slow_dispatch_rows':slow,
            'whole_iteration_miss_rows': missed,
            'any_bus_submit_to_native_slow_miss_cycles': [row['cycle'] for row in missed
                if max(row['front_submit_to_native_ms'],
                       row['rear_submit_to_native_ms']) > slow_threshold_ms],
            'post_last_output_reply_slow_miss_cycles': [row['cycle'] for row in missed
                if row['post_last_output_reply_ms'] is not None and
                row['post_last_output_reply_ms'] > slow_threshold_ms],
            'gc_overlap_miss_cycles': [row['cycle'] for row in missed
                if row['gc_intervals_overlapping_cycle']],
            'main_check_ms':distribution([row['main_check_ms'] for row in detail]),
            'main_submit_ms':distribution([row['main_submit_ms'] for row in detail]),
            'gc_events_recorded':len(gc_events),
            'gc_event_overflow':trace.get('gc_overflow'),
            'gc_probe_errors':trace.get('gc_probe_errors'),
            'interpretation':'GC overlap is correlation only; callback adds bounded measurement cost'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('records', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    report_bytes, record_bytes = args.report.read_bytes(), args.records.read_bytes()
    result = analyze(json.loads(report_bytes), json.loads(record_bytes))
    result['source_report_sha256'] = hashlib.sha256(report_bytes).hexdigest()
    result['source_records_sha256'] = hashlib.sha256(record_bytes).hexdigest()
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        if args.output.exists():
            parser.error('Refusing to overwrite existing result')
        args.output.write_text(rendered)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
