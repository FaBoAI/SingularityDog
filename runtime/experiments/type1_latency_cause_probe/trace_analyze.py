"""Pair RestoreThread probes and retain scheduler wait states as evidence."""
from __future__ import annotations

import argparse
from decimal import Decimal
import json
from pathlib import Path
import re

LINE = re.compile(r'^.*?-(\d+)\s+\[\d+\].*?\s(\d+\.\d+):\s+(?:\w+:)?'
                  r'(acquire_entry|acquire_exit|restore_entry|restore_return|sched_switch|sched_wakeup):\s*(.*)$')
SWITCH = re.compile(r'prev_pid=(\d+).*?prev_state=(\S+).*?next_pid=(\d+)')
WAKE = re.compile(r'\bpid=(\d+)')


def parse_trace(text):
    events = []
    for sequence, line in enumerate(text.splitlines()):
        match = LINE.match(line)
        if match:
            tid, seconds, kind, payload = match.groups()
            events.append({'tid': int(tid), 'ns': int(Decimal(seconds) * 1_000_000_000),
                           'kind': kind, 'payload': payload, 'sequence': sequence})
    return sorted(events, key=lambda e: (e['ns'], e['sequence']))


def scheduler_parts(events, tid, begin, end):
    """Split off-CPU time into sleeping and runnable; unknown stays unknown."""
    off_since, sleep_since = None, None
    sleeping, runnable, unknown = 0, 0, 0
    for event in events:
        now = event['ns']
        if now < begin or now > end:
            continue
        if event['kind'] == 'sched_switch':
            match = SWITCH.search(event['payload'])
            if not match:
                continue
            prev, state, nxt = match.groups()
            if int(prev) == tid:
                if off_since is not None:
                    unknown += max(0, now - off_since)
                off_since = now
                sleep_since = None if state.startswith('R') else now
            if int(nxt) == tid and off_since is not None:
                if sleep_since is not None:
                    # No wakeup witness: do not count the whole interval as GIL sleep.
                    unknown += now - off_since
                else:
                    runnable += now - off_since
                off_since, sleep_since = None, None
        elif event['kind'] == 'sched_wakeup' and off_since is not None:
            match = WAKE.search(event['payload'])
            if match and int(match.group(1)) == tid and sleep_since is not None:
                sleeping += now - sleep_since
                off_since, sleep_since = now, None
    if off_since is not None:
        unknown += max(0, end - off_since)
    total = end - begin
    return {'sleeping_ns': sleeping, 'runnable_wait_ns': runnable,
            'unknown_off_cpu_ns': unknown,
            'on_cpu_or_unobserved_ns': total - sleeping - runnable - unknown}


def analyze(text, tids=None):
    events = parse_trace(text)
    stacks, intervals, unmatched_returns = {}, [], []
    for event in events:
        if tids is not None and event['tid'] not in tids:
            continue
        family = 'acquire_path' if event['kind'].startswith('acquire_') else 'full_restore'
        key = event['tid'], family
        if event['kind'] in ('restore_entry', 'acquire_entry'):
            stacks.setdefault(key, []).append(event['ns'])
        elif event['kind'] in ('restore_return', 'acquire_exit'):
            stack = stacks.get(key, [])
            if not stack:
                unmatched_returns.append(event)
                continue
            begin = stack.pop()
            intervals.append({'tid': event['tid'], 'begin_ns': begin, 'end_ns': event['ns'],
                'duration_ns': event['ns'] - begin,
                'endpoint_kind': family,
                'endpoint_semantics': ('Post-PAC frame entry through GIL acquisition to authenticated pre-tail branch; excludes thread-state swap/return tail'
                                       if family == 'acquire_path' else
                                       'PyEval_RestoreThread entry to function return; includes thread-state tail'),
                'lr_hijack_used': family != 'acquire_path',
                **scheduler_parts(events, event['tid'], begin, event['ns'])})
    unmatched_entries = [{'tid': tid, 'endpoint_kind': family, 'begin_ns': value}
                         for (tid, family), values in stacks.items() for value in values]
    return {'schema': 'singularitydog.restore-thread-trace-analysis.v1',
            'status': 'DIAGNOSTIC_ONLY', 'output_allowed': False,
            'duration_meaning': 'Probe-point path includes scheduling; each interval retains its endpoint semantics',
            'unmatched_entries': unmatched_entries, 'unmatched_returns': unmatched_returns,
            'interval_count': len(intervals), 'intervals': intervals,
            'pure_gil_wait_proven': False,
            'requires_native_exit_to_first_python_correlation': True,
            'trace_buffer_loss_must_be_checked_separately': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', required=True, type=Path)
    parser.add_argument('--tid', action='append', type=int)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Fresh output required')
    report = analyze(args.trace.read_text(), None if args.tid is None else set(args.tid))
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'intervals'}))


if __name__ == '__main__':
    main()
