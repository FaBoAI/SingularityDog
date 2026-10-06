#!/usr/bin/env python3
"""Explain saved startup/STOP evidence only; never access or retry hardware.

An observed mode-zero Type2 frame cannot erase an outstanding Type1/Type3
transaction: these replies have no transaction number. Missing evidence stays
unknown. The report describes the saved run, not the robot's current state.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

SCHEMA = 'singularitydog.saved-supported-startup-failure-analysis.v1'
BUSES = {'front': set(range(1, 7)), 'rear': set(range(7, 13))}
STARTUP_PHASES = ('startup_enable', 'startup_zero_gain')


def _integer(value, low=0, high=2**63-1):
    return type(value) is int and low <= value <= high


def _frame(value):
    if type(value) is not str or len(value) != 34:
        raise ValueError('canonical wire missing')
    raw = bytes.fromhex(value)
    if (len(raw) != 17 or raw[:2] != b'AT' or raw[6] != 8 or
            raw[-2:] != b'\r\n' or raw[5] & 7 != 4):
        raise ValueError('canonical framing missing')
    can = int.from_bytes(raw[2:6], 'big') >> 3
    return dict(kind=can >> 24, destination=can & 255, source=(can >> 8) & 255,
                middle=(can >> 8) & 65535,
                mode=(can >> 22) & 3, fault=(can >> 16) & 63,
                ordinary=raw[7:10] != b'\x00\xc4\x56', payload=raw[7:15])


def _record(row, bus, phase, stats=None):
    """Expose only canonical IDs/types and original numeric receipt fields."""
    issues = []
    bus = bus if type(bus) is str and bus in BUSES else None
    context = dict(bus=bus,
        phase=phase if phase in (*STARTUP_PHASES, 'emergency_stop') else None,
        motor_id=None, request_type=None, written_bytes=None, reply_bytes=None,
        full_write=None, no_reply_read=None, original_timestamps_ns={},
        deadline_from_request_ms=None, reply_delay_ms=None,
        mode_state=None, fault_bits=None, evidence_status='UNKNOWN')
    if type(row) is not dict:
        return context, ['record object missing']
    try:
        tx = _frame(row.get('tx_hex'))
        if bus not in BUSES or tx['destination'] not in BUSES[bus]:
            raise ValueError('bus/ID mismatch')
        if phase == 'emergency_stop' and (tx['kind'] != 4 or
                tx['middle'] != 253 or tx['payload'] != bytes(8)):
            raise ValueError('ordinary STOP request missing')
        if phase in STARTUP_PHASES and tx['kind'] != (3 if phase == 'startup_enable' else 1):
            raise ValueError('startup request type mismatch')
        if phase == 'startup_enable' and (tx['middle'] != 253 or tx['payload'] != bytes(8)):
            raise ValueError('canonical enable request missing')
        if phase == 'startup_zero_gain' and (tx['middle'] != 32767 or
                tx['payload'][2:4] != b'\x7f\xff' or tx['payload'][4:] != bytes(4)):
            raise ValueError('canonical zero-gain request missing')
        context.update(motor_id=tx['destination'], request_type=tx['kind'])
    except (ValueError, TypeError):
        tx = None
        issues.append('canonical request/bus/type invalid')
    for key, output in (('written', 'written_bytes'), ('received', 'reply_bytes')):
        if _integer(row.get(key), 0, 17):
            context[output] = row[key]
        else:
            issues.append('invalid byte count: '+key)
    keys = ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns')
    stamps = {key: row.get(key) if _integer(row.get(key)) else None for key in keys}
    context['original_timestamps_ns'] = stamps
    timing_valid = all(value is not None for value in stamps.values()) and stamps['deadline_ns'] > 0
    if timing_valid:
        start, finish, read, received, deadline = (stamps[k] for k in keys)
        if context['written_bytes'] == 0:
            timing_valid = start == finish == 0
        else:
            timing_valid = 0 < start <= finish < deadline
        if context['reply_bytes'] == 0:
            timing_valid = timing_valid and read == received == 0
        elif context['reply_bytes'] == 17:
            timing_valid = timing_valid and start <= read <= received < deadline and finish <= received
        else:
            timing_valid = False
        if type(stats) is dict:
            begin, end = stats.get('begin_ns'), stats.get('end_ns')
            timing_valid = (timing_valid and _integer(begin, 1) and _integer(end, begin) and
                (not start or begin <= start <= end) and finish <= end and received <= end)
        if timing_valid and start:
            context['deadline_from_request_ms'] = (deadline-start)/1e6
            if received:
                context['reply_delay_ms'] = (received-start)/1e6
    if not timing_valid:
        issues.append('original timestamps missing/noncausal/late')
    if context['reply_bytes'] == 17:
        try:
            rx = _frame(row.get('rx_hex'))
            if (tx is None or rx['kind'] != 2 or rx['source'] != tx['destination'] or
                    rx['destination'] != 253 or not rx['ordinary']):
                raise ValueError('ordinary matching Type2 missing')
            context.update(mode_state=rx['mode'], fault_bits=rx['fault'])
        except (ValueError, TypeError):
            issues.append('canonical ordinary reply/ID invalid')
    if not issues:
        context.update(full_write=context['written_bytes'] == 17,
            no_reply_read=context['reply_bytes'] == 0, evidence_status='VERIFIED_RECORD_CONTEXT')
    return context, issues


def _ids(value, bus):
    return (type(value) is list and all(_integer(x, 1, 12) and x in BUSES[bus] for x in value)
            and len(value) == len(set(value)))


def analyze_report(report, report_sha256):
    if (type(report) is not dict or type(report_sha256) is not str or len(report_sha256) != 64 or
            any(c not in '0123456789abcdef' for c in report_sha256)):
        raise ValueError('Report object and exact source SHA256 required')
    issues, startup, failed, outstanding = [], [], [], set()
    journal = report.get('journal')
    if type(journal) is not list:
        journal = []
        issues.append('original journal missing')
    for item in journal:
        if type(item) is not dict or item.get('phase') not in STARTUP_PHASES:
            continue
        rows = item.get('records')
        if type(rows) is not list or not rows:
            issues.append('original startup records missing')
            continue
        for row in rows:
            context, errors = _record(row, item.get('bus'), item['phase'], item.get('stats'))
            context['data_quality_errors'] = errors
            startup.append(context)
            if (context['evidence_status'] == 'VERIFIED_RECORD_CONTEXT' and
                    context['written_bytes'] > 0 and context['reply_bytes'] == 0):
                outstanding.add(context['motor_id'])
            if item.get('error') is not None or context['full_write'] is not True or context['reply_bytes'] != 17:
                failed.append(context)
    transition = report.get('zero_gain_enable_transition')
    transition = transition if type(transition) is dict else {}
    completed = transition.get('completed_axes')
    complete_startup = (transition.get('complete') is True and type(completed) is list and
        all(_integer(x, 1, 12) for x in completed) and
        transition.get('completed_axes') == [x['motor_id'] for x in startup if x['phase'] == 'startup_zero_gain'] and
        set(completed) == set(range(1, 13)) and len(startup) == 24 and
        all({x['motor_id'] for x in startup if x['phase'] == phase} == set(range(1, 13)) and
            sum(x['phase'] == phase for x in startup) == 12 for phase in STARTUP_PHASES) and
        not failed and all(x['evidence_status'] == 'VERIFIED_RECORD_CONTEXT' and x['fault_bits'] == 0 and
            x['mode_state'] in ((0, 2) if x['phase'] == 'startup_enable' else (2,)) for x in startup))
    context = failed[0] if failed else None
    if context is None and not complete_startup:
        axis = transition.get('current_axis')
        axis = axis if type(axis) is dict else {}
        bus, mid = axis.get('bus'), axis.get('motor_id')
        bus = bus if type(bus) is str and bus in BUSES else None
        context = dict(bus=bus if bus in BUSES else None,
            motor_id=mid if bus in BUSES and _integer(mid, 1, 12) and mid in BUSES[bus] else None,
            phase={'enable': 'startup_enable', 'zero_gain': 'startup_zero_gain'}.get(transition.get('current_stage')),
            request_type=None, written_bytes=None, reply_bytes=None, full_write=None,
            no_reply_read=None, original_timestamps_ns={}, deadline_from_request_ms=None,
            evidence_status='UNKNOWN_METADATA_ONLY', data_quality_errors=['original startup request evidence unavailable'])
        issues.append('complete startup or original failing request not established')
    observed, healthy, confirmed, ambiguous, unconfirmed = set(), set(), set(), set(), set()
    stop_issues, per_bus = [], {}
    if not startup or any(x['evidence_status'] != 'VERIFIED_RECORD_CONTEXT' for x in startup):
        stop_issues.append('startup pending-request provenance incomplete')
    summaries = report.get('stop_reports')
    summaries = summaries if type(summaries) is dict else {}
    for bus, bus_ids in BUSES.items():
        summary = summaries.get(bus)
        if type(summary) is not dict:
            stop_issues.append(bus+': STOP summary missing')
            continue
        declared_valid = all(_ids(summary.get(k), bus) for k in
                             ('confirmed_ids', 'ambiguous_ids', 'unconfirmed_ids'))
        if not declared_valid:
            stop_issues.append(bus+': STOP ID lists missing/invalid')
        declared = set(summary.get('confirmed_ids', [])) if declared_valid else set()
        amb = set(summary.get('ambiguous_ids', [])) if declared_valid else set()
        unc = set(summary.get('unconfirmed_ids', [])) if declared_valid else set()
        attempts = summary.get('attempts', [summary])
        if type(attempts) is not list or not attempts:
            attempts = []
            stop_issues.append(bus+': STOP attempts missing')
        bus_observed, bus_faults, count = set(), {}, 0
        for attempt in attempts:
            evidence = attempt.get('evidence') if type(attempt) is dict else None
            rows = evidence.get('records') if type(evidence) is dict else None
            if type(rows) is not list or not rows:
                stop_issues.append(bus+': original STOP records missing')
                continue
            for row in rows:
                receipt, errors = _record(row, bus, 'emergency_stop', evidence.get('stats'))
                if errors:
                    stop_issues.append(bus+': invalid original STOP receipt')
                    continue
                count += 1
                if receipt['full_write'] and receipt['mode_state'] == 0:
                    bus_observed.add(receipt['motor_id'])
                    mid = receipt['motor_id']
                    bus_faults[mid] = bus_faults.get(mid, 0) | receipt['fault_bits']
        # Repeated STOP retains every fault, even if a later frame is healthy.
        bus_healthy = {mid for mid, bits in bus_faults.items() if bits == 0}
        reported_faults = summary.get('fault_by_id')
        faults_valid = (type(reported_faults) is dict and
            set(reported_faults) == {str(mid) for mid in bus_faults} and
            all(_integer(bits, 0, 63) for bits in reported_faults.values()))
        expected_faults = {str(mid): bits for mid, bits in bus_faults.items()}
        if not faults_valid or reported_faults != expected_faults:
            stop_issues.append(bus+': retained fault summary missing/contradicts original STOP receipts')
        reported_faulted = ({int(mid) for mid, bits in reported_faults.items() if bits > 0}
                            if faults_valid else set())
        # Preserve the distinction even if a derived summary erases ambiguity.
        pending = outstanding & bus_ids
        if declared & (amb | pending) or declared & unc or declared | unc != bus_ids:
            stop_issues.append(bus+': confirmation contradicts pending/ambiguous/unconfirmed IDs')
        if declared - bus_healthy:
            stop_issues.append(bus+': reported confirmation lacks valid mode-zero fault-zero receipt')
        verified = declared & bus_healthy - amb - pending - reported_faulted
        observed |= bus_observed; healthy |= bus_healthy; confirmed |= verified
        ambiguous |= amb | pending; unconfirmed |= bus_ids - verified
        per_bus[bus] = dict(raw_STOP_records_validated=count, mode_zero_observed_ids=sorted(bus_observed),
            reported_confirmed_ids=sorted(declared), reported_ambiguous_ids=sorted(amb),
            reported_unconfirmed_ids=sorted(unc), independently_preserved_pending_ids=sorted(pending),
            retained_raw_fault_bits_by_id=expected_faults,
            matching_reported_confirmed_ids=sorted(verified),
            reported_complete=summary.get('complete') if type(summary.get('complete')) is bool else None,
            attempts_recorded=len(attempts))
    stop_status = ('CONFIRMED_AS_REPORTED_WITH_MATCHING_RAW' if not stop_issues and
        confirmed == set(range(1, 13)) and not ambiguous and report.get('stop_confirmed') is True and
        all(x['reported_complete'] is True for x in per_bus.values()) else
        'UNCONFIRMED' if ambiguous or report.get('stop_confirmed') is False else 'UNKNOWN')
    cycles = report.get('cycles')
    return dict(schema=SCHEMA, source_report_sha256=report_sha256,
        tool_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='saved_report_only_not_current_physical_state', hardware_access=False,
        automatic_retry=False, grants_motor_output=False, changes_runtime_limits=False,
        source_report_status=report.get('status') if type(report.get('status')) is str else None,
        startup_classification='STARTUP_REQUEST_FAILURE_OBSERVED' if failed else
            'NO_STARTUP_FAILURE_OBSERVED_IN_SAVED_REPORT' if complete_startup else 'UNKNOWN',
        failed_request_context=context, original_startup_records_checked=len(startup),
        cycles_recorded=len(cycles) if type(cycles) is list else None,
        actual_model_calls=report.get('actual_model_calls') if _integer(report.get('actual_model_calls')) else None,
        learned_targets_sent=report.get('learned_targets_sent') if type(report.get('learned_targets_sent')) is bool else None,
        failure_cause='UNKNOWN_FROM_SAVED_HOST_RECEIPTS',
        stop=dict(mode_zero_observed_ids=sorted(observed), mode_zero_fault_zero_observed_ids=sorted(healthy),
            matching_reported_confirmed_ids=sorted(confirmed), ambiguous_ids=sorted(ambiguous),
            unconfirmed_ids=sorted(unconfirmed), confirmation_status=stop_status, buses=per_bus,
            data_quality_errors=sorted(set(stop_issues)), later_mode_zero_clears_pending_attribution=False),
        data_quality_errors=sorted(set(issues)),
        trial_success_or_extension_eligibility_inferred=False)


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key')
        result[key] = value
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--report-sha256')
    parser.add_argument('--output', required=True, type=Path, help='Fresh analysis JSON; existing files are refused')
    args = parser.parse_args(argv)
    try:
        if args.report.is_symlink() or not args.report.is_file():
            raise ValueError('Regular saved report required')
        raw = args.report.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if args.report_sha256 is not None and digest != args.report_sha256:
            raise ValueError('Report SHA256 differs')
        report = json.loads(raw, object_pairs_hook=_object,
            parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON value')))
        result = analyze_report(report, digest)
        if hashlib.sha256(args.report.read_bytes()).hexdigest() != digest:
            raise ValueError('Report changed during analysis')
        with args.output.open('x', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write('\n')
        print(json.dumps(dict(status='FILE_ONLY_ANALYSIS_SAVED', source_report_sha256=digest,
            startup_classification=result['startup_classification'],
            stop_confirmation_status=result['stop']['confirmation_status'])))
        return 0
    except (ValueError, OSError, TypeError) as error:
        print('File-only analysis failed: '+str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
