"""Read saved failed-cycle timing; distinguish measured waits without assigning a cause.

All times describe host timestamps. Residuals may include scheduling, worker
completion, decoding and coordinator work; they do not establish an IMU/GIL cause.
No devices, model, approval, or output files are accessed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

STAMPS = ('release_ns', 'begin_ns', 'previous_candidate_ns', 'previous_sample_start_ns',
          'hold_checked_ns', 'combined_acquisition_wait_begin_ns',
          'combined_acquisition_wait_end_ns', 'feedback_collect_begin_ns', 'feedback_collect_end_ns',
          'imu_wait_begin_ns', 'imu_wait_end_ns', 'imu_read_started_ns', 'imu_read_finished_ns',
          'acquisition_complete_ns', 'sample_start_ns', 'policy_call_begin_ns',
          'policy_call_return_ns', 'target_ready_ns', 'voltage_owner_validated_ns',
          'voltage_join_complete_ns', 'candidate_ns', 'output_submit_ns',
          'output_return_ns', 'cycle_end_ns')


def _stamp(value, name):
    if value is not None and (type(value) is not int or value <= 0):
        raise ValueError('Invalid timestamp: ' + name)
    return value


def _elapsed(end, start, label):
    if end is None or start is None:
        return None
    if end < start:
        raise ValueError('Noncausal timestamps: ' + label)
    return (end - start) / 1e6


def analyze(report):
    if type(report) is not dict or type(report.get('failed_cycle_timing')) is not dict:
        raise ValueError('Expected a report with failed_cycle_timing')
    trace = report['failed_cycle_timing']
    times = {name: _stamp(trace.get(name), name) for name in STAMPS}
    # Missing intermediate stamps remain unknown; present ordering is still causal.
    order = ('release_ns', 'begin_ns', 'hold_checked_ns',
             'combined_acquisition_wait_begin_ns', 'combined_acquisition_wait_end_ns',
             'feedback_collect_begin_ns',
             'feedback_collect_end_ns', 'imu_wait_begin_ns', 'imu_wait_end_ns',
             'acquisition_complete_ns', 'policy_call_begin_ns', 'policy_call_return_ns',
             'target_ready_ns', 'voltage_join_complete_ns', 'candidate_ns',
             'output_submit_ns', 'output_return_ns', 'cycle_end_ns')
    present = [(name, times[name]) for name in order if times[name] is not None]
    for (old_name, old), (new_name, new) in zip(present, present[1:]):
        _elapsed(new, old, old_name + ' -> ' + new_name)
    measures = {}
    for label, end, start in (
        ('combined_acquisition_wait_ms', 'combined_acquisition_wait_end_ns', 'combined_acquisition_wait_begin_ns'),
        ('combined_wait_end_to_feedback_collect_begin_ms', 'feedback_collect_begin_ns', 'combined_acquisition_wait_end_ns'),
        ('combined_wait_end_to_acquisition_complete_ms', 'acquisition_complete_ns', 'combined_acquisition_wait_end_ns'),
        ('imu_read_finish_to_combined_wait_end_ms', 'combined_acquisition_wait_end_ns', 'imu_read_finished_ns'),
        ('feedback_collect_wait_ms', 'feedback_collect_end_ns', 'feedback_collect_begin_ns'),
        ('feedback_rows_preparation_ms', 'imu_wait_begin_ns', 'feedback_collect_end_ns'),
        ('feedback_to_imu_result_takeout_ms', 'imu_wait_begin_ns', 'feedback_collect_end_ns'),
        ('imu_future_wait_ms', 'imu_wait_end_ns', 'imu_wait_begin_ns'),
        ('imu_read_ms', 'imu_read_finished_ns', 'imu_read_started_ns'),
        ('imu_read_finish_to_future_return_ms', 'imu_wait_end_ns', 'imu_read_finished_ns'),
        ('imu_read_finish_to_acquisition_complete_ms', 'acquisition_complete_ns', 'imu_read_finished_ns'),
        ('begin_to_acquisition_complete_ms', 'acquisition_complete_ns', 'begin_ns'),
        ('feedback_collect_end_to_acquisition_complete_ms', 'acquisition_complete_ns', 'feedback_collect_end_ns'),
        ('imu_future_return_to_acquisition_complete_ms', 'acquisition_complete_ns', 'imu_wait_end_ns'),
        ('acquisition_to_policy_call_ms', 'policy_call_begin_ns', 'acquisition_complete_ns'),
        ('policy_call_ms', 'policy_call_return_ns', 'policy_call_begin_ns'),
        ('policy_return_to_target_ready_ms', 'target_ready_ns', 'policy_call_return_ns'),
        ('target_ready_to_voltage_join_ms', 'voltage_join_complete_ns', 'target_ready_ns'),
        ('voltage_owner_to_join_ms', 'voltage_join_complete_ns', 'voltage_owner_validated_ns'),
        ('voltage_join_to_candidate_ms', 'candidate_ns', 'voltage_join_complete_ns'),
        ('command_interval_ms', 'candidate_ns', 'previous_candidate_ns'),
        ('sample_interval_ms', 'sample_start_ns', 'previous_sample_start_ns'),
        ('candidate_sample_age_ms', 'candidate_ns', 'sample_start_ns'),
        ('sample_to_acquisition_complete_ms', 'acquisition_complete_ns', 'sample_start_ns')):
        measures[label] = _elapsed(times[end], times[start], label)
    journal = report.get('journal', [])
    if type(journal) is not list:
        raise ValueError('Expected journal list')
    buses = {bus: {'native_begin_ns': None, 'native_end_ns': None,
                   'native_end_to_acquisition_complete_ms': None,
                   'native_end_to_combined_wait_end_ms': None,
                   'native_end_to_feedback_collect_end_ms': None} for bus in ('front', 'rear')}
    begin, candidate = times['begin_ns'], times['candidate_ns']
    if begin is not None and candidate is not None:
        for row in journal:
            if type(row) is not dict:
                raise ValueError('Invalid journal row')
            if row.get('phase') != 'feedback_hold' or row.get('bus') not in buses:
                continue
            stats = row.get('stats')
            if stats is None:
                continue
            if type(stats) is not dict:
                raise ValueError('Invalid feedback journal stats')
            start = _stamp(stats.get('begin_ns'), 'journal native begin')
            end = _stamp(stats.get('end_ns'), 'journal native end')
            _elapsed(end, start, 'journal native exchange')
            if start is None or end is None or not begin <= start <= end <= candidate:
                continue
            bus = buses[row['bus']]
            if bus['native_end_ns'] is not None:
                raise ValueError('Ambiguous feedback journal batch: ' + row['bus'])
            bus.update(native_begin_ns=start, native_end_ns=end,
                       native_end_to_acquisition_complete_ms=_elapsed(times['acquisition_complete_ns'], end, 'native end -> acquisition'),
                       native_end_to_combined_wait_end_ms=_elapsed(times['combined_acquisition_wait_end_ns'], end, 'native end -> combined wait end'),
                       native_end_to_feedback_collect_end_ms=_elapsed(times['feedback_collect_end_ns'], end, 'native end -> collect end'))
    ends = [row['native_end_ns'] for row in buses.values()]
    measures['last_bus_native_end_to_acquisition_complete_ms'] = (
        _elapsed(times['acquisition_complete_ns'], max(ends), 'last native end -> acquisition')
        if all(end is not None for end in ends) else None)
    measures['last_bus_native_end_to_combined_wait_end_ms'] = (
        _elapsed(times['combined_acquisition_wait_end_ns'], max(ends), 'last native end -> combined wait end')
        if all(end is not None for end in ends) else None)
    basis = trace.get('command_gap_basis')
    if basis is not None and type(basis) is not str:
        raise ValueError('Invalid command_gap_basis')
    return {'schema': 'singularitydog.policy-cycle-wait-analysis.v1',
            'failed_cycle_index': trace.get('index'), 'failed_stage': trace.get('stage'),
            'command_gap_basis': basis, 'timing_ns': times, 'intervals_ms': measures,
            'feedback_native_by_bus': buses, 'root_cause_established': False,
            'acquisition_wait_layout': (
                'combined readiness wait, then ready CAN/IMU result takeout; '
                'rows merge follows IMU takeout; feedback_rows_preparation_ms is a legacy alias '
                'for feedback_to_imu_result_takeout_ms'
                if times['combined_acquisition_wait_begin_ns'] is not None else
                'legacy or sequential CAN/IMU collection; unrecorded intervals remain unknown'),
            'output_allowed': False, 'timestamp_scope': 'host timestamps; unseparated residual is not a root cause'}


def load_report(path, expected_sha256=None):
    if expected_sha256 is not None and not re.fullmatch('[0-9a-f]{64}', expected_sha256):
        raise ValueError('Expected SHA256 must be 64 lowercase hex characters')
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError('Expected a regular saved report file')
    if path.stat().st_size > 128*1024*1024:
        raise ValueError('Saved report exceeds 128 MiB')
    raw = path.read_bytes()
    if len(raw) > 128*1024*1024:
        raise ValueError('Saved report exceeds 128 MiB')
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError('Report SHA256 mismatch')
    def reject_constant(value):
        raise ValueError('Invalid JSON numeric constant: ' + value)
    return json.loads(raw, parse_constant=reject_constant), digest


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise ValueError(message)


def main(argv=None):
    parser = _Parser(description=__doc__, add_help=False)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--expected-sha256')
    try:
        args = parser.parse_args(argv)
        report, digest = load_report(args.report, args.expected_sha256)
        result = analyze(report)
        result['report_sha256'] = digest
        print(json.dumps(result, sort_keys=True, allow_nan=False))
        return 0
    except (OSError, ValueError) as error:
        print(json.dumps({'status': 'ANALYSIS_REJECTED', 'error': str(error),
                          'root_cause_established': False, 'output_allowed': False}, sort_keys=True))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
