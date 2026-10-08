"""Analyze SHA-pinned saved native-pair timing; no hardware or output admission.

Completed and failed raw cycles stay separate. Intervals use original integer
timestamps; missing inference starts remain unknown. The JSON summary omits
identities, boot IDs, thread IDs, private paths and arbitrary error prose.
Only file reads and an optional fresh JSON output exist here.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

SCHEMA = 'singularitydog.native-pair-latency-analysis.v1'
MAX_BYTES = 32*1024*1024
KNOWN_ERRORS = {'TimeoutError', 'ValueError', 'RuntimeError', 'ExchangeError',
                'OSError', 'InterruptedError', 'KeyboardInterrupt', 'SystemExit'}


def need(value, message):
    if not value:
        raise ValueError(message)


def hash_value(value):
    need(type(value) is str and len(value) == 64 and
         all(c in '0123456789abcdef' for c in value), 'Invalid lowercase SHA256')
    return value


def load_pinned(path, expected_sha256, *, max_bytes=MAX_BYTES):
    """Bound the actual read as well as stat; reject ambiguous JSON objects."""
    hash_value(expected_sha256)
    need(type(max_bytes) is int and 0 < max_bytes <= MAX_BYTES, 'Invalid bounded read size')
    path = Path(path)
    need(path.is_file() and not path.is_symlink(), 'Regular nonsymlink input required')
    need(path.stat().st_size <= max_bytes, 'Saved JSON exceeds byte bound')
    with path.open('rb') as stream:
        raw = stream.read(max_bytes+1)
    need(len(raw) <= max_bytes, 'Saved JSON grew beyond byte bound')
    need(hashlib.sha256(raw).hexdigest() == expected_sha256, 'Saved JSON SHA256 mismatch')
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def nonfinite(_):
        raise ValueError('Nonfinite JSON number')
    def finite_float(value):
        result = float(value)
        need(math.isfinite(result), 'Nonfinite JSON number')
        return result
    try:
        return json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                          parse_constant=nonfinite, parse_float=finite_float)
    except (UnicodeError, RecursionError) as error:
        raise ValueError('Saved JSON encoding or nesting exceeds parser limits') from error


def ns(value):
    return value if type(value) is int and 0 < value < 2**63 else None


def interval(end, begin):
    end, begin = ns(end), ns(begin)
    return end-begin if end is not None and begin is not None else None


def trace_row(trace, row):
    if type(trace) is not dict or type(row) is not list:
        return {}
    fields = trace.get('fields')
    need(type(fields) is list and all(type(name) is str for name in fields) and
         len(set(fields)) == len(fields) == len(row), 'Saved trace field/row shape differs')
    return dict(zip(fields, row))


def cpu_rows(report):
    result = {}
    trace = report.get('inference_thread_cpu_trace')
    if type(trace) is dict:
        for row in trace.get('rows', []):
            value = trace_row(trace, row)
            cycle = value.get('cycle')
            need(type(cycle) is int and cycle > 0 and cycle not in result,
                 'Saved CPU trace cycle coverage differs')
            result[cycle] = value
    partials = report.get('incomplete_cycle_traces', [])
    need(type(partials) is list, 'Invalid incomplete trace inventory')
    for partial in partials:
        need(type(partial) is dict and type(partial.get('cycle')) is int and partial['cycle'] > 0,
             'Invalid incomplete trace cycle')
        trace = partial.get('inference_thread_cpu_trace')
        if type(trace) is dict:
            value = trace_row(trace, trace.get('row'))
            need(partial['cycle'] not in result, 'Duplicate complete/incomplete CPU trace cycle')
            result[partial['cycle']] = value
    return result


def output_wires(row):
    output = row.get('output', {})
    need(type(output) is dict, 'Invalid saved output exchanges')
    wires = []
    for bus in ('front', 'rear'):
        exchange = output.get(bus, {})
        need(type(exchange) is dict and type(exchange.get('records', [])) is list,
             'Invalid saved bus records')
        for record in exchange.get('records', []):
            need(type(record) is dict, 'Invalid saved wire record')
            wires.append(record)
    return wires


def oldest_input(row):
    starts = []
    imu = row.get('imu')
    if type(imu) is dict and ns(imu.get('read_started_monotonic_ns')) is not None:
        starts.append(imu['read_started_monotonic_ns'])
    acquired = row.get('acquired', {})
    need(type(acquired) is dict, 'Invalid saved acquisition exchanges')
    for bus in ('front', 'rear'):
        exchange = acquired.get(bus, {})
        need(type(exchange) is dict and type(exchange.get('records', [])) is list,
             'Invalid saved acquisition records')
        for record in exchange.get('records', []):
            need(type(record) is dict, 'Invalid saved acquisition wire')
            stamp = ns(record.get('start_ns'))
            if stamp is not None:
                starts.append(stamp)
    return min(starts) if starts else None


def gate_proof(row):
    for key in ('voltage_fast_pipeline', 'voltage_pipeline', 'voltage_overlap'):
        if type(row.get(key)) is dict:
            return row[key]
    return {}


def error_stage(row):
    gate = gate_proof(row)
    proof = gate.get('output_join_failure_proof')
    if type(proof) is dict and proof.get('stage') in ('readiness_join', 'result_takeout'):
        return proof['stage']
    for key, stage in (('final_gate_error', 'final_gate'), ('proxy_submit_error', 'proxy_submit'),
                       ('acquisition_join_error', 'feedback_validation'),
                       ('post_inference_error', 'post_inference'),
                       ('voltage_join_error', 'voltage_join'), ('stop_reply_error', 'stop_reply')):
        if key in gate:
            return stage
    return 'unknown'


def distribution(values):
    values = sorted(value for value in values if type(value) is int)
    if not values:
        return dict(count=0, min_ns=None, median_ns=None, max_ns=None,
                    min_ms=None, median_ms=None, max_ms=None)
    median = statistics.median(values)
    return dict(count=len(values), min_ns=values[0], median_ns=median, max_ns=values[-1],
                min_ms=values[0]/1e6, median_ms=median/1e6, max_ms=values[-1]/1e6)


def analyze(report, records, *, report_sha256, records_sha256):
    """Read plain saved objects without mutation, device calls or permission tokens."""
    hash_value(report_sha256); hash_value(records_sha256)
    need(type(report) is dict and type(records) is list, 'Report object and raw row list required')
    measurements = report.get('measurements')
    completed, requested = report.get('cycles_completed'), report.get('cycles_requested')
    need(type(measurements) is list and type(completed) is int and type(requested) is int and
         0 <= completed == len(measurements) <= len(records) <= requested <= 10_000,
         'Saved completed/raw/requested cycle counts differ')
    cpu = cpu_rows(report)
    cycles = []
    seen = set()
    for index, raw in enumerate(records):
        need(type(raw) is dict and type(raw.get('cycle')) is int and
             1 <= raw['cycle'] <= requested and raw['cycle'] not in seen, 'Raw cycle ID invalid/duplicate')
        cycle = raw['cycle']; seen.add(cycle)
        need(cycle == index+1, 'Raw cycle numbering must retain original contiguous order')
        measurement = measurements[index] if index < completed else None
        need(measurement is None or type(measurement) is dict, 'Invalid original measurement')
        is_completed = measurement is not None
        oldest = ns(measurement.get('oldest_input_start_ns')) if is_completed else oldest_input(raw)
        wires = output_wires(raw)
        writes = [wire['finish_ns'] for wire in wires
                  if type(wire.get('written')) is int and wire['written'] > 0 and ns(wire.get('finish_ns')) is not None]
        replies = [wire['received_ns'] for wire in wires
                   if wire.get('received') == 17 and ns(wire.get('received_ns')) is not None]
        last_write = max(writes) if writes else (ns(measurement.get('final_host_write_ns')) if is_completed else None)
        last_reply = max(replies) if replies else (ns(measurement.get('last_proxy_reply_ns')) if is_completed else None)
        wall = interval(measurement.get('infer_end_ns'), measurement.get('prepare_end_ns')) if is_completed else None
        cpu_row = cpu.get(cycle, {})
        thread_cpu = interval(cpu_row.get('thread_cpu_end_ns'), cpu_row.get('thread_cpu_begin_ns'))
        if thread_cpu is not None:
            need(thread_cpu >= 0, 'CPU trace time reversed')
            if 'thread_cpu_ns' in cpu_row:
                need(type(cpu_row['thread_cpu_ns']) is int and cpu_row['thread_cpu_ns'] == thread_cpu,
                     'Saved CPU duration differs from original timestamps')
        if wall is not None:
            need(wall >= 0, 'Inference original time reversed')
        phase = raw.get('native_phase_pair_phase') or {}
        need(type(phase) is dict, 'Invalid saved native phase')
        finished = phase.get('owner_finished_ns', [None, None])
        need(type(finished) is list and len(finished) == 2, 'Saved native owner coverage differs')
        gate = gate_proof(raw)
        join = ns(gate.get('output_join_begin_ns'))
        offsets = {bus:interval(join, stamp) for bus, stamp in zip(('front', 'rear'), finished)}
        row = dict(cycle=cycle, status='completed' if is_completed else 'failed',
            error_stage=None if is_completed else error_stage(raw),
            inference_wall_ns=wall, inference_thread_cpu_ns=thread_cpu,
            inference_wall_minus_thread_cpu_ns=(wall-thread_cpu if wall is not None and thread_cpu is not None else None),
            oldest_input_to_final_host_write_ns=interval(last_write, oldest),
            oldest_input_to_last_reply_ns=interval(last_reply, oldest),
            native_owner_finished_to_output_join_begin_ns_by_bus=offsets,
            both_native_owners_finished_to_output_join_begin_ns=interval(join, max(finished))
                if all(ns(stamp) is not None for stamp in finished) else None,
            output_join_begin_after_deadline_ns=interval(join, gate.get('output_join_deadline_ns')),
            output_slot_count=len(wires), complete_output_write_count=sum(w.get('written') == 17 for w in wires),
            complete_output_reply_count=sum(w.get('received') == 17 for w in wires),
            partial_output_slot_count=sum(w.get('written') != 17 or w.get('received') != 17 for w in wires))
        for name in ('inference_wall', 'inference_thread_cpu', 'inference_wall_minus_thread_cpu',
                     'oldest_input_to_final_host_write', 'oldest_input_to_last_reply',
                     'both_native_owners_finished_to_output_join_begin', 'output_join_begin_after_deadline'):
            value = row[name+'_ns']; row[name+'_ms'] = value/1e6 if value is not None else None
        row['native_owner_finished_to_output_join_begin_ms_by_bus'] = {
            bus:value/1e6 if value is not None else None for bus,value in offsets.items()}
        cycles.append(row)
    metrics = ('inference_wall_ns', 'inference_thread_cpu_ns',
        'oldest_input_to_final_host_write_ns', 'oldest_input_to_last_reply_ns',
        'both_native_owners_finished_to_output_join_begin_ns', 'output_join_begin_after_deadline_ns')
    distributions = {status:{metric:distribution(row[metric] for row in cycles if row['status'] == status)
        for metric in metrics} for status in ('completed', 'failed')}
    errors = report.get('errors', [])
    need(type(errors) is list, 'Invalid saved report error inventory')
    error_types = Counter()
    for error in errors:
        name = error.split(':', 1)[0] if type(error) is str else None
        error_types[name if name in KNOWN_ERRORS else 'unknown'] += 1
    saved_status = report.get('status')
    if saved_status not in ('COMPLETE_DIAGNOSTIC', 'ABORTED', 'ABORTED_BEFORE_PREPARATION', 'ABORTED_BEFORE_MODEL'):
        saved_status = 'OTHER_SAVED_STATUS'
    return dict(schema=SCHEMA, status='FILE_ONLY_LATENCY_ANALYSIS', saved_run_status=saved_status,
        source_sha256=dict(report=report_sha256, records=records_sha256),
        counts=dict(requested_cycles=requested, completed_measurements=completed,
                    raw_rows=len(records), failed_raw_rows=len(records)-completed),
        unrepresented_requested_cycles=requested-len(records),
        error_types=dict(error_types), failed_error_stage_counts=dict(Counter(
            row['error_stage'] for row in cycles if row['status'] == 'failed')),
        cycles=cycles, distributions=distributions, hardware_opened=False,
        network_used=False, model_loaded=False, native_library_loaded=False,
        output_allowed=False, active_controller_qualification=False,
        timing_admission_evaluated=False, strict_20ms_qualification_claimed=False,
        source_timestamps_modified=False, failed_rows_discarded=False,
        notes=['Intervals use original integer timestamps; missing starts remain null.',
               'Completed and failed cycles have separate distributions.',
               'Host write completion is not physical CAN wire completion.',
               'This sanitized summary grants no output or timing admission.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('report', 'report-sha256', 'records', 'records-sha256'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--output')
    args = parser.parse_args(argv)
    try:
        report = load_pinned(args.report, args.report_sha256)
        records = load_pinned(args.records, args.records_sha256)
        result = analyze(report, records, report_sha256=args.report_sha256,
                         records_sha256=args.records_sha256)
        encoded = json.dumps(result, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False)+'\n'
        if args.output:
            output = Path(args.output)
            need(not output.exists() and not output.is_symlink(), 'Fresh analysis output required')
            with output.open('x', encoding='utf-8') as stream:
                stream.write(encoded)
        print(encoded, end='')
        return 0
    except (ValueError, OSError, TypeError, KeyError, AttributeError, OverflowError):
        # Preserve the original files; free-form errors may contain private paths.
        print(json.dumps(dict(schema=SCHEMA, status='BLOCKED_SAVED_FILE_ANALYSIS',
            output_allowed=False, active_controller_qualification=False,
            error='Pinned input or saved timing structure validation failed')))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
