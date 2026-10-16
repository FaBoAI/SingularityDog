"""Project two existing R3 nested deadline fields into a fresh report copy.

This file-only compatibility operation changes neither source bytes nor the
original report. It accepts only a complete same-contract 30Hz Type1 report
with complete original per-cycle wire/clock evidence and terminal STOP. The
two missing top-level fields are copied verbatim from measurement; no status,
claim, flag, threshold or original sample is changed. This grants no output.
Run with PYTHONPATH pointing to the intended (possibly frozen) runtime.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import analyze_four_bus_type1_failure as raw_audit

FIELDS = ('inference_tx_budget_ns', 'cycle_budget_ns')
METADATA = 'compatibility_deadline_field_projection'


def profile_module():
    runtime = Path(__file__).resolve().parents[1]/'runtime'
    if runtime.is_dir() and str(runtime) not in sys.path:
        sys.path.append(str(runtime))
    from experiments.four_bus_type1 import type1_profile
    return type1_profile


def need(value, message):
    raw_audit.need(value, message)


def raw_cycles(report, contract):
    """Recheck all 28 requests per cycle and their matching original journals."""
    measurement = report['measurement']
    clock = contract['control_clock']
    topology = contract['topology_by_port']
    cycles = measurement.get('cycles')
    need(type(cycles) is list and len(cycles) == report['completed_cycles'] > 0,
         'Complete original cycle list required')
    epoch = raw_audit.integer(measurement.get('epoch_ns'), 'epoch')
    need(epoch == report['first_release_monotonic_ns'], 'Original epoch/first release differ')
    need(measurement.get('normal_ramp_completed') is True and measurement.get('errors') == [] and
         measurement.get('failure_retained') is False, 'Original runner must have completed normally')
    journals, deadlines = measurement['raw_journal'], measurement.get('write_deadline_journal')
    need(type(deadlines) is dict and set(deadlines) == set(topology), 'Original native write-cutoff journal required')
    indexes = {}
    for port in topology:
        need(type(deadlines[port]) is list, 'Original write-cutoff entries required')
        entries = {}
        for entry in deadlines[port]:
            need(type(entry) is list and len(entry) == 4 and all(type(x) is int and x >= 0 for x in entry),
                 'Original native cutoff entry required')
            ji, reply, write, dispatched = entry
            need(ji not in entries and ji < len(journals[port]), 'Duplicate/out-of-range cutoff journal index')
            entries[ji] = (reply, write, dispatched)
        indexes[port] = entries
    maxima = {'input_to_last_host_write_ns': 0, 'begin_to_last_host_reply_ns': 0}
    raw_output_count = 0
    for index, row in enumerate(cycles):
        need(type(row) is dict and row.get('index') == row.get('slot') == index and row.get('completed') is True,
             'Original cycle order/completion differs')
        release, begin, first, hard, write_cap, end = (raw_audit.integer(row.get(key), key) for key in
            ('release_ns', 'begin_ns', 'first_ns', 'hard_end_ns', 'write_deadline_ns', 'cycle_end_ns'))
        need(release == epoch+index*1_000_000_000//30 and release <= begin and
             hard == begin+clock['cycle_budget_ns'] and write_cap == min(begin, first)+clock['inference_tx_budget_ns'],
             'Original rational release/write/reply cutoff differs')
        need(row.get('request_count') == 28 and row.get('decision', {}).get('accepted') is True,
             'Original cycle is not an accepted 28-request cycle')
        need(row['decision'].get('checked_ns') == end, 'Original post-reply clock differs')
        starts, finishes, received = [], [], []
        for name, count, kind in (('hold', 3, 1), ('voltage', 1, 17), ('output', 3, 1)):
            batches = row.get(name)
            need(type(batches) is dict and set(batches) == set(topology), 'All four original '+name+' batches required')
            for port, ids in topology.items():
                batch = batches[port]
                need(batch.get('physical_port') == port and batch.get('physical_ids') == ids and
                     batch.get('phase') == {'hold': 'feedback_hold', 'voltage': 'voltage', 'output': 'output'}[name] and
                     batch.get('original_record_count') == count and len(batch.get('records', [])) == count,
                     'Original physical batch shape differs')
                label = batch['phase']
                matches = [ji for ji, entry in enumerate(journals[port]) if entry[0] == label and
                           entry[1].get('records') == batch['records'] and entry[1].get('stats') == batch.get('stats')]
                need(len(matches) == 1, 'Original cycle batch does not match exactly one native journal entry')
                if name == 'output':
                    need(matches[0] in indexes[port], 'Original output has no native write-cutoff record')
                    reply, write, dispatched = indexes[port][matches[0]]
                    need(reply == hard and write == write_cap and begin <= dispatched < write_cap,
                         'Original native output cutoff differs')
                actual_ids = []
                for record in batch['records']:
                    tx, rx = raw_audit.wire(record['tx_hex']), raw_audit.wire(record['rx_hex'])
                    mid = tx['destination']; actual_ids.append(mid)
                    start, finish, read_begin, arrived, deadline = (raw_audit.integer(record.get(key), key) for key in
                        ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns'))
                    need(record.get('written') == record.get('received') == 17 and tx['kind'] == kind and
                         mid in ids and rx['source'] == mid and start > 0 and
                         begin <= start <= finish <= read_begin <= arrived < deadline == hard and arrived <= end,
                         'Incomplete/noncausal/out-of-deadline original transaction')
                    if kind == 1:
                        need(rx['kind'] == 2 and rx['mode'] == 2 and rx['fault'] == 0, 'Original Type1 feedback mode/fault differs')
                    else:
                        need(tx['data'][:2] == rx['data'][:2] == bytes.fromhex('1c70'), 'Original voltage request/reply differs')
                    if name != 'output':
                        starts.append(start)
                    else:
                        need(finish < write_cap, 'Original Type1 host write reached/exceeded20ms')
                        finishes.append(finish); received.append(arrived); raw_output_count += 1
                if kind == 1:
                    need(actual_ids == ids, 'Original Type1 batch does not cover ordered physical three')
        imu = row.get('imu')
        need(type(imu) is dict, 'Original IMU host interval required')
        imu_begin, imu_end = (raw_audit.integer(imu.get(key), key) for key in
                            ('read_started_monotonic_ns', 'read_finished_monotonic_ns'))
        need(begin <= imu_begin <= imu_end <= row['infer_end_ns'], 'Original IMU interval differs')
        starts.append(imu_begin)
        need(first == min(starts), 'Oldest input timestamp differs from original feedback/IMU')
        need(row.get('final_write_ns') == max(finishes) and row.get('output_last_reply_ns') == max(received),
             'Original aggregate last-write/reply differs')
        maxima['input_to_last_host_write_ns'] = max(maxima['input_to_last_host_write_ns'], max(finishes)-first)
        maxima['begin_to_last_host_reply_ns'] = max(maxima['begin_to_last_host_reply_ns'], max(received)-begin)
    return {'complete_original_cycles_checked': len(cycles), 'original_Type1_output_records_checked': raw_output_count,
            **maxima, 'all_original_writes20ms_and_replies33ms_met': True}


def normalize(report, profile, *, original_reference, profile_reference, expected_source_sha256):
    P = profile_module()
    need(not any(key in report for key in (*FIELDS, METADATA)), 'Only the original missing-field R3 report can be projected')
    P.validate_profile(profile)
    contract = profile['contract']
    need('control_clock' in contract and report.get('schema') == P.REPORT_SCHEMA_30, 'Original explicit30Hz contract/report required')
    need(report.get('profile_canonical_sha256') == P.profile_canonical_sha256(profile) and
         report.get('mode') == profile['mode'] and report.get('duration_s') == profile['duration_s'],
         'Original report names a different profile/mode/duration')
    need(contract['source_manifest']['sha256'] == expected_source_sha256 == report.get('source_manifest_sha256'),
         'Expected original source manifest differs')
    measurement = report.get('measurement')
    need(type(measurement) is dict, 'Original nested measurement required')
    for key in ('control_clock', 'policy30_model'):
        need(report.get(key) == measurement.get(key) == contract[key], 'Original clock/new-model contradiction')
    for key in FIELDS:
        need(type(measurement.get(key)) is int and measurement[key] == contract['control_clock'][key],
             'Missing/contradictory original nested deadline')
    for key in ('mode', 'duration_s', 'boot_id', 'motor_power_epoch', 'contract_sha256', 'pacing',
                'status', 'completed_cycles', 'motor_enable_sent', 'type1_sent',
                'positive_gain_sent', 'learned_targets_attempted'):
        need(report.get(key) == measurement.get(key), 'Original nested run identity/contract differs')
    need(report.get('source_and_input_pins_unchanged') is True, 'Original source/input integrity failed')
    need(report.get('input_sha256', {}).get(contract['source_manifest']['path']) == expected_source_sha256,
         'Original source manifest pin missing')
    need(all(report['input_sha256'].get(ref['path']) == ref['sha256']
             for ref in contract['policy30_model']['artifacts'].values()),
         'Original new30Hz model artifact pins missing/different')
    backend = measurement.get('backend_usage')
    need(type(backend) is dict and backend.get('kind') == 'genuine_subset_active_cpp' and
         backend.get('source_manifest_sha256') == expected_source_sha256 and
         all(backend.get(key) in report['input_sha256'].values() for key in ('library_sha256', 'build_record_sha256')),
         'Original native backend/source/library proof differs')
    audited = raw_audit.analyze(report)
    need(audited['failed_requests'] == [] and audited['terminal_STOP_owner_summary']['stop_confirmed'] is True and
         audited['terminal_STOP_owner_summary']['unconfirmed_ids'] == audited['terminal_STOP_owner_summary']['ambiguous_ids'] == [],
         'Original raw exchange/STOP failure cannot be normalized')
    proof = raw_cycles(report, contract)
    derived = copy.deepcopy(report)
    for key in FIELDS:
        derived[key] = measurement[key]
    # The genuine unchanged consumer verifies status, flags, same contract/model
    # and terminal STOP. Only the absent serialization aliases are supplied.
    summary = P.validate_type1_report(derived, contract=contract, contract_digest=P.contract_sha256(contract),
                                     mode=report['mode'], duration_s=report['duration_s'])
    derived[METADATA] = {'schema': 'singularitydog.original-deadline-field-projection.v1',
        'original_report': copy.deepcopy(original_reference), 'original_profile': copy.deepcopy(profile_reference),
        'fields_projected_verbatim_from_measurement': list(FIELDS),
        'original_source_manifest_sha256': expected_source_sha256, 'original_report_modified': False,
        'original_status_flags_raw_records_or_thresholds_changed': False, 'hardware_opened': False,
        'output_approval_granted_here': False, 'timing_qualification_granted_here': False,
        'sticky_STOP_ambiguity_cleared': False, 'consumer_summary': summary, 'raw_validation': proof}
    for key in FIELDS:
        need(derived[key] == report['measurement'][key], 'Derived field differs')
    need({key: value for key, value in derived.items() if key not in (*FIELDS, METADATA)} == report,
         'Original data changed during field projection')
    return derived


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('report', 'profile'):
        parser.add_argument('--'+name, type=Path, required=True)
        parser.add_argument('--'+name+'-sha256', required=True)
    parser.add_argument('--expected-source-manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    source = raw_audit.load_report(args.report, args.report_sha256)
    profile = raw_audit.load_report(args.profile, args.profile_sha256)
    derived = normalize(source, profile,
        original_reference={'path': str(args.report), 'sha256': args.report_sha256},
        profile_reference={'path': str(args.profile), 'sha256': args.profile_sha256},
        expected_source_sha256=args.expected_source_manifest_sha256)
    raw_audit.write_fresh(args.output, derived)
    print(json.dumps({'status': 'ORIGINAL_DEADLINE_FIELDS_PROJECTED', 'output': str(args.output),
        'sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
        'original_report_sha256': args.report_sha256, 'output_approval_granted_here': False,
        'validation': derived[METADATA]['raw_validation']}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
