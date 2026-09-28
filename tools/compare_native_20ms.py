"""Compare two finite STOP-proxy diagnostics without claiming live control.

Only complete 500-cycle no-enable runs are accepted by default. The cycle release
interval is checked separately from work duration; a short iteration after a
missed release does not make a 50 Hz controller. An explicit offline policy can
accept the already observed cadence of two immutable R17 reports. It never
changes hardware timing, freshness checks, STOP handling, or future acceptance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


OBSERVED_R17_POLICY = 'observed-r17-cadence-20260928'
# The user accepted these observed starts, not an unlimited jitter tolerance.
# Pin the original bytes so the exception cannot spread to later/edited runs.
OBSERVED_R17_REPORTS = frozenset((
    '1d0e49226ab095c007d5de63325b8e201467315234e43ed229546b3652cfb2d4',
    '7eaadb878a0ea6f98cfeae1b49312a2bcc4a63d71d4aeb8c9f43c6740afbe204',
))


def _number(value, label):
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
        raise ValueError('Invalid nonnegative timing: ' + label)
    return float(value)


def summarize(report, *, startup_cycle_allowance=0):
    if type(startup_cycle_allowance) is not int or startup_cycle_allowance not in (0, 1):
        raise ValueError('Startup allowance must be zero or one recorded cycle')
    requested = 500 + startup_cycle_allowance
    if (type(report) is not dict or report.get('status') != 'COMPLETE_DIAGNOSTIC'
            or report.get('errors') != [] or report.get('mode') != 'stop-proxy'
            or report.get('cycles_requested') != requested or report.get('cycles_completed') != requested
            or report.get('motor_enable_sent') is not False
            or report.get('learned_targets_sent') is not False
            or report.get('approved_for_runtime') is not False
            or report.get('full_controller_50Hz_verified') is not False
            or report.get('imu_restore_status') not in ('restored', 'not_needed')):
        raise ValueError('Requires complete 500 steady cycles in a disabled STOP-proxy report')
    if startup_cycle_allowance and report.get('plan', {}).get('startup_cycle_allowance') != 1:
        raise ValueError('The startup cycle must have been recorded explicitly')
    rows = report.get('measurements')
    if type(rows) is not list or len(rows) != requested:
        raise ValueError('Missing complete cycle measurements')
    metrics = ('acquisition_ms', 'prepare_ms', 'inference_ms',
               'oldest_input_to_final_host_write_ms',
               'oldest_input_to_last_reply_ms', 'whole_iteration_ms')
    values = {name: [] for name in metrics}
    intervals = []
    for index, row in enumerate(rows):
        if type(row) is not dict:
            raise ValueError('Invalid cycle row')
        for name in metrics:
            values[name].append(_number(row.get(name), name))
        interval = row.get('actual_release_interval_ms')
        if index == 0:
            if interval is not None:
                raise ValueError('First release interval must be absent')
        else:
            intervals.append(_number(interval, 'actual_release_interval_ms'))
        if row.get('iteration_deadline_met') is not (values['whole_iteration_ms'][-1] <= 20):
            raise ValueError('Iteration deadline flag differs from timing')
        if startup_cycle_allowance and row.get('timing_phase') != ('startup' if index == 0 else 'steady'):
            raise ValueError('Only the first recorded cycle may be classified as startup')
    affinity = report.get('main_thread_affinity')
    if affinity is not None and affinity.get('requested_cpu') is not None and affinity.get('restored') is not True:
        raise ValueError('Main-thread CPU affinity was not restored')
    plan = report.get('plan', {})
    requests_per_cycle = plan.get('requests_per_cycle')
    if requests_per_cycle is None and plan.get('input_workers') == ['front6', 'rear6', 'IMU']:
        # Older complete STOP-proxy reports predate the explicit count field.
        requests_per_cycle = 24
    if requests_per_cycle is not None and (type(requests_per_cycle) is not int or requests_per_cycle <= 0):
        raise ValueError('Invalid request count')
    result = {'boot_id': report.get('boot_id'),
              'request_gap_us': plan.get('request_gap_us'),
              'request_window': plan.get('window'),
              'requests_per_cycle': requests_per_cycle,
              'cycles': 500, 'stage_ms': {}, 'deadline_misses': {},
              'max_release_interval_ms': max(intervals[startup_cycle_allowance:])}
    if startup_cycle_allowance:
        result['startup'] = {
            'cycles': 1,
            'whole_iteration_ms': values['whole_iteration_ms'][0],
            'iteration_deadline_misses': int(values['whole_iteration_ms'][0] > 20),
            'startup_to_steady_interval_ms': intervals[0],
            'all_cycles_retained': len(rows) == requested,
            'total_cycles': requested,
        }
        # Keep the startup evidence above; steady-to-steady intervals begin at
        # rows[2]. Never discard a later processing miss as another warmup.
        values = {name: series[1:] for name, series in values.items()}
        intervals = intervals[1:]
    for name, series in values.items():
        sorted_values = sorted(series)
        result['stage_ms'][name] = {'median': statistics.median(series),
                                   'p99': sorted_values[494], 'max': sorted_values[-1]}
    result['deadline_misses'] = {
        'oldest_input_to_final_host_write': sum(x > 20 for x in values['oldest_input_to_final_host_write_ms']),
        'oldest_input_to_last_reply': sum(x > 20 for x in values['oldest_input_to_last_reply_ms']),
        'whole_iteration': sum(x > 20 for x in values['whole_iteration_ms']),
        'release_intervals_over_21ms': sum(x > 21 for x in intervals),
        'start_intervals_over_20ms': sum(x > 20 for x in intervals),
    }
    result['stop_proxy_diagnostic_gate'] = all(result['deadline_misses'][name] == 0 for name in (
        'oldest_input_to_final_host_write', 'oldest_input_to_last_reply',
        'whole_iteration', 'release_intervals_over_21ms'))
    result['strict_start_interval_20ms_met'] = result['deadline_misses']['start_intervals_over_20ms'] == 0
    if startup_cycle_allowance:
        steady = rows[1:]
        result['cadence_observations'] = {
            'max_release_lateness_ms': max(_number(row.get('release_lateness_ms'), 'release_lateness_ms') for row in steady),
            'scheduled_deadline_misses': sum(row['scheduled_completion_slack_ms'] < 0 for row in steady),
            'slots_skipped': sum(row['skipped_slots_before'] for row in steady),
        }
    return result


def compare(baseline, candidate, *, startup_cycle_allowance=0):
    left, right = (summarize(report, startup_cycle_allowance=startup_cycle_allowance)
                   for report in (baseline, candidate))
    return {'scope': 'Disabled STOP-proxy timing only; no learned target or active motor command',
            'baseline': left, 'candidate': right,
            'same_boot': left['boot_id'] == right['boot_id'],
            'same_pacing': (left['request_gap_us'], left['request_window']) ==
                           (right['request_gap_us'], right['request_window']),
            'same_request_count': (left['requests_per_cycle'] is not None
                                   and left['requests_per_cycle'] == right['requests_per_cycle']),
            'whole_iteration_max_change_ms':
                right['stage_ms']['whole_iteration_ms']['max'] - left['stage_ms']['whole_iteration_ms']['max'],
            'diagnostic_gate_pass': right['stop_proxy_diagnostic_gate'],
            'live_policy_20ms_verified': False}


def compare_sources(baseline_raw, candidate_raw, *, acceptance_policy=None):
    """File-only adoption judgment; exact source bytes bound the exception."""
    hashes = {name: hashlib.sha256(raw).hexdigest()
              for name, raw in (('baseline', baseline_raw), ('candidate', candidate_raw))}
    if acceptance_policy not in (None, OBSERVED_R17_POLICY):
        raise ValueError('Unknown acceptance policy')
    if acceptance_policy and not all(digest in OBSERVED_R17_REPORTS for digest in hashes.values()):
        raise ValueError('Observed-cadence acceptance applies only to the two original R17 report hashes')
    result = compare(json.loads(baseline_raw), json.loads(candidate_raw),
                     startup_cycle_allowance=1 if acceptance_policy else 0)
    result['source_sha256'] = hashes
    if acceptance_policy:
        assessments = {}
        for name in ('baseline', 'candidate'):
            summary = result[name]
            # Cadence acceptance never excuses late processing or stale input
            # through the final proxy reply. Strict cadence remains reported.
            processing_ok = all(summary['deadline_misses'][key] == 0 for key in (
                'oldest_input_to_final_host_write', 'oldest_input_to_last_reply', 'whole_iteration'))
            assessments[name] = {
                'accepted_for_diagnostic_continuation': processing_ok,
                'steady_processing_20ms_met': summary['deadline_misses']['whole_iteration'] == 0,
                'observed_cadence_accepted_by_operator': True,
                'strict_start_interval_20ms_met': summary['strict_start_interval_20ms_met'],
            }
        result['operator_acceptance'] = {
            'policy': acceptance_policy,
            'scope': 'Only these two existing disabled R17 reports; first cycle separately recorded, observed start jitter accepted',
            'reports': assessments,
            'accepted_for_diagnostic_continuation': all(
                entry['accepted_for_diagnostic_continuation'] for entry in assessments.values()),
            'future_reports_covered': False,
            'runtime_safety_limits_changed': False,
            'approved_for_runtime': False,
            'live_policy_20ms_verified': False,
        }
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--acceptance-policy', choices=(OBSERVED_R17_POLICY,),
                        help='Offline acceptance of the two hash-pinned R17 runs only; preserves strict timing results')
    args = parser.parse_args(argv)
    source = [path.read_bytes() for path in (args.baseline, args.candidate)]
    result = compare_sources(*source, acceptance_policy=args.acceptance_policy)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        if args.output.exists():
            parser.error('Refusing to overwrite existing result')
        args.output.write_text(rendered)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
