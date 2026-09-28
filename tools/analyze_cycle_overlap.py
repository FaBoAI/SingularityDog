"""Replay saved STOP-proxy timing with speculative next-cycle acquisition.

This is a host-timestamp scheduling bound, not a hardware timing prediction.
The next CAN acquisition starts only after both previous output exchanges have
ended.  The main thread finishes the previous cycle's bookkeeping before it
consumes the next input.  Recorded stage durations are held constant.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


PERIOD_NS = 20_000_000


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        raise ValueError('No samples')
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p99': ordered[math.ceil(.99 * len(ordered)) - 1],
            'max': ordered[-1]}


def _timestamps(batch, label):
    if not isinstance(batch, dict) or not isinstance(batch.get('records'), list) or len(batch['records']) != 6:
        raise ValueError(f'{label}: require six complete CAN records')
    stats = batch.get('stats') or {}
    begin, end = stats.get('begin_ns'), stats.get('end_ns')
    if type(begin) is not int or type(end) is not int or not 0 < begin <= end:
        raise ValueError(f'{label}: invalid exchange timestamps')
    starts, writes, replies = [], [], []
    for frame in batch['records']:
        start, write, reply = (frame.get(k) for k in ('start_ns', 'finish_ns', 'received_ns'))
        if (not all(type(t) is int for t in (start, write, reply)) or
                not begin <= start <= write <= reply <= end or
                frame.get('written') != 17 or frame.get('received') != 17):
            raise ValueError(f'{label}: incomplete or noncausal CAN exchange')
        starts.append(start)
        writes.append(write)
        replies.append(reply)
    return min(starts), max(writes), max(replies), end


def build_profiles(report, records, expected_cycles=500):
    """Validate one complete disabled run and retain only timing offsets."""
    if (report.get('status') != 'COMPLETE_DIAGNOSTIC' or
            report.get('mode') != 'stop-proxy' or
            report.get('motor_enable_sent') is not False or
            report.get('learned_targets_sent') is not False or
            report.get('cycles_requested') != expected_cycles or
            report.get('cycles_completed') != expected_cycles or
            type(records) is not list or len(records) != expected_cycles or
            type(report.get('measurements')) is not list or
            len(report['measurements']) != expected_cycles):
        raise ValueError('Require a complete disabled STOP-proxy run')
    profiles = []
    for index, (row, record) in enumerate(zip(report['measurements'], records), 1):
        if record.get('cycle') != index:
            raise ValueError('Cycle order mismatch')
        release, gather, inferred, cycle_end = (row.get(k) for k in
            ('release_ns', 'gather_end_ns', 'infer_end_ns', 'cycle_end_ns'))
        if (not all(type(t) is int for t in (release, gather, inferred, cycle_end)) or
                not 0 < release <= gather <= inferred <= cycle_end):
            raise ValueError(f'Cycle {index}: invalid main-thread timestamps')
        imu = record.get('imu') or {}
        imu_start = imu.get('read_started_monotonic_ns')
        imu_end = imu.get('read_finished_monotonic_ns')
        if (type(imu_start) is not int or type(imu_end) is not int or
                not release <= imu_start <= imu_end <= gather):
            raise ValueError(f'Cycle {index}: invalid IMU timestamps')
        input_buses = {}
        output_buses = {}
        input_end = imu_end
        output_end = 0
        output_reply = 0
        output_write = 0
        oldest = imu_start
        for bus in ('front', 'rear'):
            acquired = _timestamps(record.get('acquired', {}).get(bus), f'{index} input {bus}')
            output = _timestamps(record.get('output', {}).get(bus), f'{index} output {bus}')
            if acquired[3] > gather or output[0] < inferred or output[3] > cycle_end:
                raise ValueError(f'Cycle {index}: exchange outside its stage')
            input_buses[bus] = {'start_offset': acquired[0] - release,
                                'duration': acquired[3] - acquired[0]}
            output_buses[bus] = {'write_after_infer': output[1] - inferred,
                                 'reply_after_infer': output[2] - inferred,
                                 'end_after_infer': output[3] - inferred}
            oldest = min(oldest, acquired[0])
            input_end = max(input_end, acquired[3])
            output_write = max(output_write, output[1])
            output_reply = max(output_reply, output[2])
            output_end = max(output_end, output[3])
        if (input_end > gather or output_end > cycle_end or
                oldest != row.get('oldest_input_start_ns') or
                output_write != row.get('final_host_write_ns') or
                output_reply != row.get('last_proxy_reply_ns') or
                abs((cycle_end - release) / 1e6 - row.get('whole_iteration_ms', float('nan'))) > 1e-6):
            raise ValueError(f'Cycle {index}: report and records disagree')
        profiles.append({'input_buses': input_buses, 'output_buses': output_buses,
                         'imu_start_offset': imu_start - release,
                         'imu_duration': imu_end - imu_start,
                         'gather_tail': gather - input_end,
                         'compute_duration': inferred - gather,
                         'cycle_tail': cycle_end - output_end,
                         'measured': {'release': release, 'output_end': output_end,
                                      'cycle_end': cycle_end, 'oldest': oldest,
                                      'write': output_write, 'reply': output_reply}})
    return profiles


def simulate(profiles, *, overlap, lead_ns=0, chained=False, period_ns=PERIOD_NS):
    """Deterministic stage replay; zero added dispatch cost is optimistic."""
    if not profiles or lead_ns < 0 or period_ns <= 0 or (chained and lead_ns):
        raise ValueError('Invalid replay schedule')
    rows = []
    for index, profile in enumerate(profiles):
        target = (rows[-1]['launch'] + period_ns if chained and rows else index * period_ns)
        previous_ready = 0
        if rows:
            previous_ready = (rows[-1]['output_end'] if overlap else rows[-1]['cycle_end'])
        launch = max(target - lead_ns, previous_ready)
        input_starts = [launch + profile['imu_start_offset']]
        input_ends = [input_starts[0] + profile['imu_duration']]
        for bus in ('front', 'rear'):
            entry = profile['input_buses'][bus]
            start = launch + entry['start_offset']
            if rows and start < rows[-1]['output_bus_end'][bus]:
                raise AssertionError('Same-bus output and next acquisition overlap')
            input_starts.append(start)
            input_ends.append(start + entry['duration'])
        oldest = min(input_starts)
        # The observer and trace-copy work both run on the main thread.
        main_ready = rows[-1]['cycle_end'] if rows else 0
        gathered = max(max(input_ends), main_ready) + profile['gather_tail']
        inferred = max(gathered, target) + profile['compute_duration']
        output_bus_end = {}
        writes = []
        replies = []
        for bus in ('front', 'rear'):
            output = profile['output_buses'][bus]
            output_bus_end[bus] = inferred + output['end_after_infer']
            writes.append(inferred + output['write_after_infer'])
            replies.append(inferred + output['reply_after_infer'])
        output_end = max(output_bus_end.values())
        cycle_end = output_end + profile['cycle_tail']
        rows.append({'target': target, 'launch': launch, 'oldest': oldest,
                     'write': max(writes), 'reply': max(replies),
                     'output_end': output_end, 'output_bus_end': output_bus_end,
                     'cycle_end': cycle_end})
    return rows


def summarize(rows, period_ns=PERIOD_NS):
    ms = lambda values: [value / 1e6 for value in values]
    intervals = ms([b['launch'] - a['launch'] for a, b in zip(rows, rows[1:])])
    ages = ms([row['write'] - row['oldest'] for row in rows])
    replies = ms([row['reply'] - row['oldest'] for row in rows])
    output_from_target = ms([row['write'] - row['target'] for row in rows])
    input_from_target = ms([row['oldest'] - row['target'] for row in rows])
    launch_delay = ms([max(0, row['launch'] - row['target']) for row in rows])
    phase_drift = ms([row['launch'] - index * period_ns
                      for index, row in enumerate(rows)])
    return {'cycles': len(rows), 'launch_interval_ms': distribution(intervals),
            'launch_intervals_over_20ms': sum(value > period_ns / 1e6 for value in intervals),
            'launch_intervals_over_21ms': sum(value > 21 for value in intervals),
            'max_launch_delay_vs_schedule_ms': max(launch_delay),
            'max_phase_drift_ms': max(phase_drift),
            'final_phase_drift_ms': phase_drift[-1],
            'oldest_input_to_host_write_ms': distribution(ages),
            'oldest_input_to_host_write_over_20ms': sum(value > 20 for value in ages),
            'oldest_input_to_last_reply_ms': distribution(replies),
            'oldest_input_start_minus_nominal_tick_ms': distribution(input_from_target),
            'oldest_input_started_before_tick_count': sum(value < 0 for value in input_from_target),
            'host_write_after_nominal_tick_ms': distribution(output_from_target),
            'host_write_after_nominal_tick_over_20ms':
                sum(value > period_ns / 1e6 for value in output_from_target)}


def analyze(report, records, expected_cycles=500, lead_ms=2.):
    profiles = build_profiles(report, records, expected_cycles)
    if not math.isfinite(lead_ms) or lead_ms < 0:
        raise ValueError('Invalid lookahead')
    lead_ns = round(lead_ms * 1e6)
    measured_headroom = [(p['measured']['release'] + PERIOD_NS -
                          p['measured']['output_end']) / 1e6 for p in profiles[:-1]]
    cleanup = [(p['measured']['cycle_end'] - p['measured']['output_end']) / 1e6
               for p in profiles]
    occupied = [(p['measured']['output_end'] - p['measured']['oldest']) / 1e6
                for p in profiles]
    schedules = {
        'serial_chained': simulate(profiles, overlap=False, chained=True),
        'overlap_chained': simulate(profiles, overlap=True, chained=True),
        'serial_phase_locked': simulate(profiles, overlap=False),
        'overlap_phase_locked': simulate(profiles, overlap=True),
        f'overlap_phase_locked_lead_{lead_ms:g}ms':
            simulate(profiles, overlap=True, lead_ns=lead_ns),
    }
    return {'scope': 'Offline host-timestamp replay of disabled STOP-proxy cycles',
            'assumptions': ['Recorded stage durations are unchanged by rescheduling',
                            'Both CAN input workers launch only after both prior STOP exchanges end',
                            'Previous-cycle main-thread cleanup ends before next input is consumed',
                            'Zero additional task-dispatch and contention cost',
                            'Nominal ticks remain 20ms apart; model step is never before its tick',
                            'Motor-internal sensor sample timestamps and CAN wire completion are unknown'],
            'measured_next_tick_minus_output_end_ms': distribution(measured_headroom),
            'measured_output_end_before_next_tick_count': sum(x >= 0 for x in measured_headroom),
            'measured_oldest_input_to_output_exchange_end_ms': distribution(occupied),
            'measured_oldest_input_to_output_exchange_end_over_20ms':
                sum(x > 20 for x in occupied),
            'measured_post_output_exchange_cleanup_ms': distribution(cleanup),
            'schedules': {name: summarize(rows) for name, rows in schedules.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('records', type=Path)
    parser.add_argument('--lead-ms', type=float, default=2.)
    args = parser.parse_args()
    report_bytes = args.report.read_bytes()
    records_bytes = args.records.read_bytes()
    result = analyze(json.loads(report_bytes), json.loads(records_bytes), lead_ms=args.lead_ms)
    result['source_sha256'] = {'report': hashlib.sha256(report_bytes).hexdigest(),
                               'records': hashlib.sha256(records_bytes).hexdigest()}
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
