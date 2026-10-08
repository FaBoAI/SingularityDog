"""Offline re-analysis of motor-free harness reports (``harness-report.json``); opens no device.

Per run: command/sample interval distributions (max, std, count > 20.8 and > 21 ms),
release -> first hold write, release -> natural gate (final-gate end) and
release -> command time separately, output last reply after the cycle start,
cycle end (post-reply admission) -> next release (F3: must leave max(L, pre-arm work)),
the runner status/abort, and a per-violation cause attribution separating (a) the
recorded-hardware reply tail drawn by the peer, (b) synthetic-peer artifacts,
(c) host-side stalls and (d) deterministic injections.

Per configuration (opt-in options, gap mode, latency, observer, power-scope
emulation, injections; seeds pooled): the same distributions, the aborts and the
synthesis acceptance criteria evaluated as HARNESS EMULATION ONLY (never a timing
qualification or an output approval). Older reports without the option fields
are read as the default configuration.

    python3 -B analyze.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""
import argparse
import json
from pathlib import Path
import statistics

PORTS = ('port0', 'port1', 'port2', 'port3')
EDGES = (('begin_to_hold_first_write', 'begin_ns', 'hold_first_write_ns'),
         ('hold_first_write_to_last_reply', 'hold_first_write_ns', 'hold_last_reply_ns'),
         ('hold_last_reply_to_acquired', 'hold_last_reply_ns', 'acquired_ns'),
         ('acquired_to_gather', 'acquired_ns', 'gather_ns'),
         ('gather_to_infer_end', 'gather_ns', 'infer_end_ns'),
         ('infer_end_to_voltage_join', 'infer_end_ns', 'voltage_join_ns'),
         ('voltage_join_to_final_gate', 'voltage_join_ns', 'final_gate_ns'))
STAGES = {'release_lateness': ('release_ns', 'begin_ns'), **{name: (a, b) for name, a, b in EDGES},
          'begin_to_final_gate': ('begin_ns', 'final_gate_ns'), 'final_gate_to_encode_end': ('final_gate_ns', 'encode_end_ns'),
          'output_submit_to_first_write': ('output_submit_ns', 'output_first_write_ns'),
          'output_submit_to_reply_return': ('output_submit_ns', 'reply_return_ns'),
          'iteration': ('begin_ns', 'cycle_end_ns'), 'imu_read': ('imu_read_started_ns', 'imu_read_finished_ns'),
          'observer_total': ('observer_start_ns', 'observer_end_ns'),
          'observer_model': ('observer_model_start_ns', 'observer_model_end_ns'),
          'check1_end_to_last_hold_owner_begin': None, 'release_to_last_hold_owner_begin': None}
LIMIT = 21.
MARGIN = 20.8
CYCLES_PER_RUN = 989  # A complete 20 s run: indices 0..988; index 0 has no interval.
CYCLES_REQUIRED = 3*(CYCLES_PER_RUN-1)  # Synthesis: three complete 20 s runs per configuration (2964 steady).
LATE_OWNER_MS = 1.6  # Default: hold owner native begin after Boundary-1 end.
LATE_PREARMED_OWNER_MS = .5  # F3: owners begin natively at the release; Boundary 1 ends before the wake lead.
PREARM_WORK_MS = 1.25  # type1_profile.PREARM_WORK_US: Boundary 1, hold gates and owner prep before the release.


def quantile(values, fraction):
    values = sorted(values)
    position = (len(values)-1)*fraction
    low = int(position); high = min(low+1, len(values)-1)
    return values[low]+(values[high]-values[low])*(position-low)


def describe(values):
    values = [v for v in values if v is not None]
    if not values:
        return {'n': 0}
    return {'n': len(values), 'median': quantile(values, .5), 'p90': quantile(values, .9),
            'p99': quantile(values, .99), 'p999': quantile(values, .999), 'max': max(values), 'min': min(values),
            'stdev': statistics.pstdev(values)}


def ms(a, b):
    return None if a is None or b is None else (b-a)/1e6


def stage(row, name):
    if name in ('check1_end_to_last_hold_owner_begin', 'release_to_last_hold_owner_begin'):
        begins = [row.get(f'hold_{p}_owner_begin_ns') for p in PORTS]
        base = row.get('current_check1_end_ns' if name.startswith('check1') else 'release_ns')
        if base is None or None in begins:
            return None
        return ms(base, max(begins))
    a, b = STAGES[name]
    return ms(row.get(a), row.get(b))


def natural_gate(row):
    """Release -> natural gate (final-gate end); F1 records the same instant as natural_gate_ns."""
    return ms(row.get('release_ns'), row.get('natural_gate_ns', row.get('final_gate_ns')))


def command_time(row):
    """Release -> the command time given to the envelope (F1: paced; default: the natural gate)."""
    return ms(row.get('release_ns'), row.get('command_ns', row.get('final_gate_ns')))


def injected(row):
    return {k: v for k, v in row.items() if k.startswith('inject_')}


def port_flags(row, stage_name, threshold=.5):
    flags = []
    for port in PORTS:
        get = lambda name: row.get(f'{stage_name}_{port}_{name}') or 0.
        if row.get(f'inject_{stage_name}_tail_{port}_ms'):
            flags.append(('injected_%s_tail' % stage_name, port, row[f'inject_{stage_name}_tail_{port}_ms']))
        elif get('modelled_delay_excess_ms') > threshold:
            flags.append(('device_reply_tail_from_recorded_hardware', port, get('modelled_delay_excess_ms')))
        if get('peer_send_late_ms') > threshold or get('write_to_peer_arrival_ms') > threshold:
            flags.append(('harness_peer_artifact', port, max(get('peer_send_late_ms'), get('write_to_peer_arrival_ms'))))
        if get('host_read_lag_ms') > threshold or get('write_spacing_max_ms') > .92+threshold:
            flags.append(('host_owner_stalled_inside_exchange', port,
                          max(get('host_read_lag_ms'), get('write_spacing_max_ms')-.92)))
    return sorted(flags, key=lambda item: -item[2])


def classify(now, before):
    deltas = {}
    for name, a, b in EDGES:
        x, y = ms(now.get(a), now.get(b)), ms(before.get(a), before.get(b))
        deltas[name] = None if x is None or y is None else x-y
    primary = max((k for k in deltas if deltas[k] is not None), key=lambda k: deltas[k], default=None)
    hold, voltage = port_flags(now, 'hold'), port_flags(now, 'voltage')
    # F3 (pre-arm wake recorded): an owner's native hold begins at the release, so lateness is from the release.
    base, late = (('release_ns', LATE_PREARMED_OWNER_MS) if now.get('prearm_wake_ns') is not None
                  else ('current_check1_end_ns', LATE_OWNER_MS))
    late_owners = [p for p in PORTS if (ms(now.get(base), now.get(f'hold_{p}_owner_begin_ns')) or 0) > late]
    if now.get('inject_host_stall_ms') or before.get('inject_host_stall_ms'):
        cause = 'injected_host_stall_'+(now.get('inject_host_stall_mode') or before.get('inject_host_stall_mode'))
    elif primary == 'hold_first_write_to_last_reply' and hold:
        cause = hold[0][0]
    elif primary == 'infer_end_to_voltage_join' and voltage:
        cause = voltage[0][0]
    elif primary in ('begin_to_hold_first_write', 'hold_first_write_to_last_reply') and late_owners:
        cause = 'host_owner_start_late_%d_of_4_ports' % len(late_owners)
    elif primary == 'hold_last_reply_to_acquired':
        cause = 'host_stall_owner_publish_or_main_wakeup'
    elif primary in ('acquired_to_gather', 'gather_to_infer_end', 'voltage_join_to_final_gate', 'infer_end_to_voltage_join'):
        cause = 'host_main_thread_stall_'+primary
    else:
        cause = 'host_unattributed_'+str(primary)
    if injected(before) and not injected(now) and not cause.startswith('injected'):
        cause = 'after_injected_cycle_'+cause  # Interval shortened/lengthened by the previous injected cycle.
    return {'index': now['index'], 'command_interval_ms': now.get('command_interval_ms'),
            'sample_interval_ms': now.get('sample_interval_ms'), 'primary_stage': primary,
            'primary_delta_ms': deltas.get(primary), 'cause': cause, 'late_owner_ports': late_owners,
            'injected': injected(now), 'hold_port_flags': hold, 'voltage_port_flags': voltage,
            'natural_gate_ms': natural_gate(now), 'command_ms': command_time(now),
            'deltas_ms': {k: v for k, v in deltas.items() if v is not None and abs(v) > .2}}


def options_of(report):
    value = report.get('options') or {}
    pacing = dict(value.get('pacing_options') or {})
    return pacing, bool(value.get('timing_evidence'))


def observer_of(report):
    value = report.get('observer') or {}
    return value.get('kind') or ('torch_policy' if 'profile_path' in value else None)


def config_key(report):
    """Everything but the seed: options, gap mode, latency, observer, emulation, duration, injections."""
    pacing, evidence = options_of(report)
    names = (('command_phase_offset_us', 'F1'), ('decode_once', 'F2b'), ('prearmed_hold_lead_us', 'F3'), ('gc_freeze', 'F4'))
    parts = [name+('' if pacing[k] is True else '=%s' % pacing[k]) for k, name in names if k in pacing]
    if evidence:
        parts.append('F0')
    argv = report.get('argv') or []
    duration = argv[argv.index('--duration')+1] if '--duration' in argv[:-1] else '20'
    selected = report.get('injections_selected') or {}
    inject = ['%s:%s@%d+%g' % (t['stage'], t['port'], t['cycle'], t['extra_ms']) for t in selected.get('tails', ())]
    inject += ['stall@%d:%g:%s' % (s['cycle'], s['ms'], s['mode']) for s in selected.get('stalls', ())]
    observer = observer_of(report)
    return ' | '.join(['+'.join(parts) or 'default', report.get('envelope_gap_mode') or '?',
                       (report.get('peer') or {}).get('latency_mode') or '?', str(observer),
                       report.get('power_scope_emulation') or 'no_emulation', 'duration=%s' % duration,
                       'inject=' + (','.join(inject) or 'none')])


def seed_of(report):
    argv = report.get('argv') or []
    return int(argv[argv.index('--seed')+1]) if '--seed' in argv[:-1] else 1


def end_to_next_release(rows):
    by_index = {row['index']: row for row in rows}
    return [ms(row.get('cycle_end_ns'), by_index[row['index']+1].get('release_ns'))
            for row in rows if row['index']+1 in by_index]


def analyze(directory):
    report = json.loads((Path(directory)/'harness-report.json').read_text())
    rows = report['cycles']
    runner = report['runner']
    by_index = {row['index']: row for row in rows}
    steady = [r for r in rows if r.get('command_interval_ms') is not None and r['index'] > 0]
    command = [r['command_interval_ms'] for r in steady]
    sample = [r['sample_interval_ms'] for r in steady]
    violations = [r for r in steady if r['command_interval_ms'] > LIMIT or r['sample_interval_ms'] > LIMIT]
    attributed = [classify(r, by_index[r['index']-1]) for r in violations if r['index']-1 in by_index]
    causes = {}
    for item in attributed:
        causes[item['cause']] = causes.get(item['cause'], 0)+1
    gate = [stage(r, 'begin_to_final_gate') for r in steady]
    normal = [r for r in rows if r.get('label') == 'zero_gain_timing']
    pacing, evidence = options_of(report)
    status = runner.get('status') or ''
    return {'run': Path(directory).name, 'path': str(directory), 'config': config_key(report), 'seed': seed_of(report),
            'pacing_options': pacing, 'timing_evidence': evidence,
            'injections_selected': report.get('injections_selected'),
            'injections_performed': {k: (report.get('injections') or {}).get(k) for k in
                                     ('tails_applied_by_peer', 'tails_not_applied', 'stalls_performed')}
            if report.get('injections') else None,
            'runner_status': status, 'complete': status.startswith('COMPLETE'),
            'primary_error': (runner.get('primary_error') or {}).get('message'),
            'completed_cycles': runner.get('completed_cycles'), 'steady_cycles': len(steady),
            'post_reply_late_cycles': runner.get('post_reply_late_cycles'),
            'envelope_gap_mode': report.get('envelope_gap_mode'), 'observer': observer_of(report),
            'latency': report.get('peer', {}).get('latency_mode'),
            'power_scope_emulation': report.get('power_scope_emulation'),
            'command_interval_ms': describe(command), 'sample_interval_ms': describe(sample),
            'command_over_20_8': sum(1 for v in command if v > MARGIN), 'sample_over_20_8': sum(1 for v in sample if v > MARGIN),
            'command_over_21': sum(1 for v in command if v > LIMIT), 'sample_over_21': sum(1 for v in sample if v > LIMIT),
            'command_over_20_5': sum(1 for v in command if v > 20.5),
            'command_within_20_00_pm_0_01': sum(1 for v in command if abs(v-20.) <= .01),
            'release_to_first_hold_write_ms': describe([ms(r.get('release_ns'), r.get('hold_first_write_ns')) for r in rows]),
            'release_to_natural_gate_ms': describe([natural_gate(r) for r in rows]),
            'release_to_command_ms': describe([command_time(r) for r in rows]),
            'output_last_reply_after_begin_ms': describe([ms(r.get('begin_ns'), r.get('output_last_reply_ns')) for r in rows]),
            'iteration_ms': describe([ms(r.get('begin_ns'), r.get('cycle_end_ns')) for r in rows]),
            'cycle_end_to_next_release_ms': describe(end_to_next_release(rows)),
            'violations': len(violations), 'violation_rate': len(violations)/len(steady) if steady else None,
            'first_violation_index': violations[0]['index'] if violations else None,
            'cycles_between_violations': [b['index']-a['index'] for a, b in zip(violations, violations[1:])],
            'delta_begin_to_final_gate_ms': describe([b-a for a, b in zip(gate, gate[1:]) if a is not None and b is not None]),
            'stage_ms': {name: describe([stage(r, name) for r in normal]) for name in STAGES},
            'cause_counts': causes, 'violations_attributed': attributed,
            'peer_send_late_over_100us': report.get('peer', {}).get('send_lateness_over_100us'),
            '_values': {'command': command, 'sample': sample,
                        'first_write': [ms(r.get('release_ns'), r.get('hold_first_write_ns')) for r in rows],
                        'natural': [natural_gate(r) for r in rows], 'command_time': [command_time(r) for r in rows],
                        'output_last': [ms(r.get('begin_ns'), r.get('output_last_reply_ns')) for r in rows],
                        'iteration': [ms(r.get('begin_ns'), r.get('cycle_end_ns')) for r in rows],
                        'end_to_next_release': end_to_next_release(rows)}}


def criterion(value, limit, passed):
    return {'value': value, 'limit': limit, 'pass': bool(passed)}


def acceptance(summary):
    """Synthesis acceptance thresholds, as harness emulation only (never a qualification)."""
    pacing = summary['pacing_options']
    offset = pacing.get('command_phase_offset_us')
    prearmed = 'prearmed_hold_lead_us' in pacing
    get = lambda key, field: summary[key].get(field)
    late = summary['post_reply_late_cycles_total']
    result = {
        'all_runs_complete': criterion(summary['aborts'], 0, summary['aborts'] == 0),
        'command_over_20_8': criterion(summary['command_over_20_8'], 0, summary['command_over_20_8'] == 0),
        'sample_over_20_8': criterion(summary['sample_over_20_8'], 0, summary['sample_over_20_8'] == 0),
        'command_max_ms': criterion(get('command_interval_ms', 'max'), 20.5, (get('command_interval_ms', 'max') or 99) <= 20.5),
        'sample_max_ms': criterion(get('sample_interval_ms', 'max'), 20.2 if prearmed else 20.5,
                                   (get('sample_interval_ms', 'max') or 99) <= (20.2 if prearmed else 20.5)),
        'release_to_first_hold_write_max_ms': criterion(
            get('release_to_first_hold_write_ms', 'max'), .2 if prearmed else 1.3,
            (get('release_to_first_hold_write_ms', 'max') or 99) <= (.2 if prearmed else 1.3)),
        'output_last_reply_after_begin_max_ms': criterion(
            get('output_last_reply_after_begin_ms', 'max'), 19., (get('output_last_reply_after_begin_ms', 'max') or 99) <= 19.),
        'post_reply_late_cycles': criterion(late, 0, late == 0),
        'iteration_max_ms': criterion(get('iteration_ms', 'max'), 18.5, (get('iteration_ms', 'max') or 99) <= 18.5),
        'steady_cycles': criterion(summary['steady_cycles'], CYCLES_REQUIRED, summary['steady_cycles'] >= CYCLES_REQUIRED)}
    gate_limit = offset/1000-.25 if offset is not None else 11.
    result['natural_gate_p999_ms'] = criterion(get('release_to_natural_gate_ms', 'p999'), gate_limit,
                                                (get('release_to_natural_gate_ms', 'p999') or 99) <= gate_limit)
    if offset is not None:
        result['natural_gate_max_ms'] = criterion(get('release_to_natural_gate_ms', 'max'), offset/1000+.5,
                                                  (get('release_to_natural_gate_ms', 'max') or 99) <= offset/1000+.5)
        fraction = summary['command_within_20_00_pm_0_01']/max(1, summary['command_interval_ms'].get('n', 0))
        result['command_within_20_00_pm_0_01_fraction'] = criterion(fraction, .995, fraction >= .995)
    if prearmed:
        # The next cycle wakes at release-L and must finish Boundary 1, the gates and owner prep before it.
        window = max(pacing['prearmed_hold_lead_us']/1000, PREARM_WORK_MS)
        result['cycle_end_to_next_release_min_ms'] = criterion(get('cycle_end_to_next_release_ms', 'min'), window,
            (get('cycle_end_to_next_release_ms', 'min') or -1) >= window)
        result['sample_stdev_ms'] = criterion(get('sample_interval_ms', 'stdev'), .05,
                                              (get('sample_interval_ms', 'stdev') or 99) <= .05)
    if summary['injected']:
        # Injection replays: only "no fault" applies (complete, no >21 ms would-be violation).
        no_fault = summary['aborts'] == 0 and summary['command_over_21'] == 0 and summary['sample_over_21'] == 0
        result = {'no_fault': criterion({'aborts': summary['aborts'], 'command_over_21': summary['command_over_21'],
                                         'sample_over_21': summary['sample_over_21']}, 0, no_fault)}
    return {'harness_emulation_only_not_a_timing_qualification': True, 'criteria': result,
            'all_pass': all(item['pass'] for item in result.values())}


def by_config(results):
    groups = {}
    for value in results:
        groups.setdefault(value['config'], []).append(value)
    summaries = []
    for config, runs in groups.items():
        pooled = {key: [v for run in runs for v in run['_values'][key] if v is not None] for key in runs[0]['_values']}
        command, sample = pooled['command'], pooled['sample']
        summary = {'config': config, 'runs': [run['run'] for run in runs], 'seeds': sorted(run['seed'] for run in runs),
                   'pacing_options': runs[0]['pacing_options'], 'timing_evidence': runs[0]['timing_evidence'],
                   'injected': runs[0]['injections_selected'] is not None,
                   'statuses': [run['runner_status'] for run in runs],
                   'aborts': sum(1 for run in runs if not run['complete']),
                   'abort_errors': [{'run': run['run'], 'completed_cycles': run['completed_cycles'],
                                     'error': run['primary_error']} for run in runs if not run['complete']],
                   'steady_cycles': sum(run['steady_cycles'] for run in runs),
                   'post_reply_late_cycles_total': sum(run['post_reply_late_cycles'] or 0 for run in runs),
                   'command_interval_ms': describe(command), 'sample_interval_ms': describe(sample),
                   'command_over_20_8': sum(1 for v in command if v > MARGIN),
                   'command_over_21': sum(1 for v in command if v > LIMIT),
                   'sample_over_20_8': sum(1 for v in sample if v > MARGIN),
                   'sample_over_21': sum(1 for v in sample if v > LIMIT),
                   'command_within_20_00_pm_0_01': sum(1 for v in command if abs(v-20.) <= .01),
                   'release_to_first_hold_write_ms': describe(pooled['first_write']),
                   'release_to_natural_gate_ms': describe(pooled['natural']),
                   'release_to_command_ms': describe(pooled['command_time']),
                   'output_last_reply_after_begin_ms': describe(pooled['output_last']),
                   'iteration_ms': describe(pooled['iteration']),
                   'cycle_end_to_next_release_ms': describe(pooled.get('end_to_next_release', [])),
                   'cause_counts': {}}
        for run in runs:
            for cause, count in run['cause_counts'].items():
                summary['cause_counts'][cause] = summary['cause_counts'].get(cause, 0)+count
        summary['acceptance'] = acceptance(summary)
        summaries.append(summary)
    return summaries


def _line(name, v):
    if not v.get('n'):
        return '  %-34s n=0' % name
    return '  %-34s n=%d med=%.3f p99=%.3f p99.9=%.3f max=%.3f sd=%.4f' % (
        name, v['n'], v['median'], v['p99'], v['p999'], v['max'], v['stdev'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--json')
    args = parser.parse_args(argv)
    results = [analyze(run) for run in args.runs]
    for value in results:
        print('== %(run)s  seed=%(seed)s  %(runner_status)s  cycles=%(completed_cycles)s  error=%(primary_error)s' % value)
        print('   config:', value['config'])
        for key in ('command_interval_ms', 'sample_interval_ms', 'release_to_first_hold_write_ms',
                    'release_to_natural_gate_ms', 'release_to_command_ms', 'output_last_reply_after_begin_ms'):
            print(_line(key, value[key]))
        print('  >20.8ms command=%d sample=%d  >21ms command=%d sample=%d  violations=%d (%.2f%%) first=%s' % (
            value['command_over_20_8'], value['sample_over_20_8'], value['command_over_21'], value['sample_over_21'],
            value['violations'], 100*(value['violation_rate'] or 0), value['first_violation_index']))
        print('  causes', value['cause_counts'])
    configs = by_config(results)
    print('\n#### per configuration (harness emulation only; not a timing qualification)')
    for summary in configs:
        print('== %s\n   runs=%s seeds=%s aborts=%d steady=%d' % (summary['config'], len(summary['runs']), summary['seeds'],
                                                              summary['aborts'], summary['steady_cycles']))
        for item in summary['abort_errors']:
            print('   ABORT %(run)s at %(completed_cycles)s: %(error)s' % item)
        for key in ('command_interval_ms', 'sample_interval_ms', 'release_to_first_hold_write_ms',
                    'release_to_natural_gate_ms', 'release_to_command_ms', 'output_last_reply_after_begin_ms'):
            print(_line(key, summary[key]))
        print('  >20.8ms command=%d sample=%d  >21ms command=%d sample=%d' % (
            summary['command_over_20_8'], summary['sample_over_20_8'], summary['command_over_21'], summary['sample_over_21']))
        failed = [name for name, item in summary['acceptance']['criteria'].items() if not item['pass']]
        print('  acceptance (emulation):', 'ALL PASS' if not failed else 'FAIL '+', '.join(failed))
    if args.json:
        for value in results:
            value.pop('_values')
        Path(args.json).write_text(json.dumps({'runs': results, 'configurations': configs}, indent=1))
    return results, configs


if __name__ == '__main__':
    main()
