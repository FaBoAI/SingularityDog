"""Classify SHA-pinned four-bus Type1 raw failures without opening hardware.

Setup Type3 replies, zero-gain handshakes, initial hold, cyclic output and STOP
are separate stages. Host writes are not CAN delivery; a missing reply cannot
identify the physical loss layer. Later mode-zero bytes never clear an owner's
sticky STOP ambiguity. The original report is never modified.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import struct

MAX_REPORT_BYTES = 128 * 1024 * 1024  # Original cyclic journals may exceed 16 MiB.
PORTS = ('port0', 'port1', 'port2', 'port3')
SCHEMAS = ('singularitydog.four-bus-type1-boxed-run.v1',
           'singularitydog.four-bus-type1-boxed-run.v2')
KINDS = {'identify': 0, 'stop': 4, 'version': 4, 'params': 17,
         'watchdog_setup': 18, 'enable': 3, 'zero_gain': 1,
         'feedback_hold': 1, 'voltage': 17, 'output': 1, 'stop_subset': 4}


def need(value, message):
    if not value:
        raise ValueError(message)


def integer(value, name):
    need(type(value) is int and 0 <= value < 2**63, 'Invalid '+name)
    return value


def wire(value):
    need(type(value) is str and re.fullmatch('[0-9a-fA-F]{34}', value), 'Invalid recorded AT wire')
    raw = bytes.fromhex(value)
    need(raw[:2] == b'AT' and raw[5] & 7 == 4 and raw[6] == 8 and raw[15:] == b'\r\n',
         'Noncanonical recorded AT wire')
    cid = int.from_bytes(raw[2:6], 'big') >> 3
    return {'kind': (cid >> 24) & 31, 'source': (cid >> 8) & 255,
            'destination': cid & 255, 'mode': (cid >> 22) & 3,
            'fault': (cid >> 16) & 63, 'cid': cid, 'data': raw[7:15]}


def stage(label, batch_begin, first_release):
    if label == 'output':
        return ('SETUP_INITIAL_ZERO_GAIN_HOLD' if first_release is None or batch_begin < first_release
                else 'CYCLE_TYPE1_OUTPUT')
    return {'enable': 'SETUP_ENABLE_TYPE3', 'zero_gain': 'SETUP_SINGLE_AXIS_ZERO_GAIN',
            'feedback_hold': 'CYCLE_FEEDBACK_HOLD', 'voltage': 'CYCLE_VOLTAGE_READ',
            'stop_subset': 'TERMINAL_STOP', 'stop': 'SETUP_STOP',
            'version': 'SETUP_FIRMWARE_PROBE', 'params': 'SETUP_PARAMETER_READ',
            'identify': 'SETUP_IDENTITY', 'watchdog_setup': 'SETUP_WATCHDOG_WRITE'}[label]


def analyze(report):
    need(type(report) is dict and report.get('schema') in SCHEMAS, 'Four-bus Type1 foreground report required')
    measurement, topology = report.get('measurement'), report.get('topology_by_port')
    need(type(measurement) is dict and type(topology) is dict and set(topology) == set(PORTS),
         'Measurement and four physical ports required')
    for ids in topology.values():
        need(type(ids) is list and len(ids) == 3 and all(type(mid) is int for mid in ids), 'Three IDs per port required')
    need(sorted(mid for ids in topology.values() for mid in ids) == list(range(1, 13)), 'Exact twelve-axis topology required')
    journals = measurement.get('raw_journal')
    need(type(journals) is dict and set(journals) == set(PORTS), 'All four original owner journals required')
    first = report.get('first_release_monotonic_ns')
    need(first is None or integer(first, 'first release') > 0, 'Invalid first release')
    completed = integer(report.get('completed_cycles'), 'completed cycle count')
    need(first is not None or completed == 0, 'Completed cycles without a first release')
    clock = report.get('control_clock')
    is30 = report['schema'].endswith('.v2')
    if is30:
        need(type(clock) is dict and clock.get('policy_hz') == 30 and
             clock.get('inference_tx_budget_ns') == 20_000_000 and
             clock.get('cycle_budget_ns') == 33_333_334, 'Explicit 30Hz / 20ms write / 33.333ms reply clock required')
    else:
        need(clock is None, 'Legacy schema cannot claim a new 30Hz clock')
    failures, counts, stop_observations = [], Counter(), set()
    requests = replies = batches = 0
    for port in PORTS:
        need(type(journals[port]) is list, 'Original journal list required')
        for index, entry in enumerate(journals[port]):
            need(type(entry) in (list, tuple) and len(entry) == 2, 'Labelled original exchange required')
            label, batch = entry
            need(label in KINDS and type(batch) is dict, 'Unknown Type1 exchange label')
            rows, stats = batch.get('records'), batch.get('stats')
            need(type(rows) is list and type(stats) is dict, 'Original records and native stats required')
            begin, end = (integer(stats.get(key), key) for key in ('begin_ns', 'end_ns'))
            need(0 < begin <= end, 'Noncausal native exchange interval')
            writes, receive_bytes, reads = (integer(stats.get(key), key) for key in ('writes', 'bytes', 'reads'))
            retained = batch.get('rejected_total_bytes')
            need(type(retained) is int and retained >= 0 and type(batch.get('rejected_truncated')) is bool,
                 'Explicit native rejected-byte evidence required')
            rejected = batch.get('rejected_hex')
            need(type(rejected) is str and re.fullmatch('[0-9a-fA-F]*', rejected) and len(rejected) % 2 == 0,
                 'Invalid retained receive bytes')
            need(len(rejected)//2 <= retained and (batch['rejected_truncated'] or len(rejected)//2 == retained),
                 'Retained-byte count differs')
            written_rows = matched_bytes = 0
            for slot, row in enumerate(rows):
                need(type(row) is dict, 'Original native record required')
                written, received = (integer(row.get(key), key) for key in ('written', 'received'))
                need(written <= 17 and received in (0, 17), 'Invalid native byte count')
                if written == received == 0 and row.get('tx_hex') == '00'*17:
                    continue  # Unselected slots in the native six-slot STOP record.
                tx = wire(row.get('tx_hex'))
                mid = tx['destination']
                need(mid in topology[port] and tx['kind'] == KINDS[label], 'Request ID/type differs from its physical stage')
                if label in ('identify', 'enable', 'stop', 'stop_subset', 'version'):
                    payload = bytes.fromhex('00c4000000000000') if label == 'version' else bytes(8)
                    need(tx['cid'] == (tx['kind'] << 24) | (253 << 8) | mid and tx['data'] == payload,
                         'Noncanonical setup/control request')
                if tx['kind'] == 1:
                    q, velocity, kp, kd = struct.unpack('>4H', tx['data'])
                    need(tx['cid'] == (1 << 24) | (32767 << 8) | mid and velocity == 32767,
                         'Noncanonical Type1 torque/velocity fields')
                    if label == 'zero_gain':
                        need(kp == kd == 0, 'Single-axis zero-gain stage contains positive gains')
                start, finish, read_begin, received_at, deadline = (
                    integer(row.get(key), key) for key in ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns'))
                need(deadline > begin, 'Native reply deadline precedes exchange')
                if written:
                    need(begin <= start <= finish <= end and start > 0, 'Noncausal host write')
                    written_rows += 1
                    requests += 1
                    counts[str(tx['kind'])] += 1
                else:
                    need(start == finish == read_begin == received_at == received == 0,
                         'Unwritten request claims host I/O')
                if received:
                    need(written == 17 and finish <= read_begin <= received_at <= end and read_begin > 0,
                         'Noncausal complete reply')
                    need(received_at < deadline, 'Recorded complete reply is outside its native deadline')
                    rx = wire(row.get('rx_hex'))
                    need(rx['source'] == mid and rx['kind'] == (2 if tx['kind'] in (1, 3, 4, 18) else tx['kind']),
                         'Reply ID/type differs from request')
                    if tx['kind'] == 17:
                        need(rx['data'][:2] == tx['data'][:2], 'Reply parameter differs from request')
                    if label == 'stop_subset' and rx['mode'] == 0 and rx['fault'] == 0:
                        stop_observations.add(mid)  # Observation, never STOP attribution.
                    matched_bytes += received
                    replies += 1
                else:
                    need(read_begin == received_at == 0 and row.get('rx_hex') == '00'*17,
                         'Incomplete native record claims a received frame')
                    state = ('NOT_WRITTEN' if written == 0 else 'PARTIAL_HOST_WRITE' if written < 17 else
                             'FULL_WRITE_NO_HOST_RX' if receive_bytes == 0 else 'FULL_WRITE_NO_COMPLETE_REPLY')
                    item = {'port': port, 'journal_index': index, 'record_slot': slot, 'motor_id': mid,
                            'request_kind': tx['kind'], 'stage': stage(label, begin, first),
                            'state': state, 'native_reply_budget_ms': (deadline-begin)/1e6,
                            'native_elapsed_ms': (end-begin)/1e6,
                            'write_to_reply_deadline_ms': (deadline-finish)/1e6 if written else None,
                            'host_rx_bytes_in_exchange': receive_bytes, 'host_read_calls_in_exchange': reads,
                            'retained_receive_bytes': retained,
                            'physical_packet_loss_proven': False, 'skip_or_retry_approved': False}
                    failures.append(item)
            need(written_rows == writes and matched_bytes+retained == receive_bytes,
                 'Native counters differ from the original raw records')
            batches += 1
    transition = measurement.get('zero_gain_enable_transition')
    primary_setup = [row for row in failures if row['stage'] in ('SETUP_ENABLE_TYPE3', 'SETUP_SINGLE_AXIS_ZERO_GAIN')]
    if first is None and primary_setup and transition is not None:
        need(type(transition) is dict and transition.get('complete') is False, 'Failed transition claims completion')
        need(any(transition.get('current_axis') == {'port': row['port'], 'motor_id': row['motor_id']} and
                 transition.get('current_stage') == ('enable' if row['request_kind'] == 3 else 'zero_gain')
                 for row in primary_setup),
             'Transition stage differs from failed native request')
    terminal = report.get('terminal_stop')
    need(type(terminal) is dict and type(terminal.get('stop_confirmed')) is bool, 'Original terminal STOP summary required')
    stop_ids = {}
    for name in ('confirmed_ids', 'unconfirmed_ids', 'ambiguous_ids'):
        value = terminal.get(name)
        need(type(value) is list and len(value) == len(set(value)) and
             all(type(mid) is int and 1 <= mid <= 12 for mid in value), 'Invalid terminal '+name)
        stop_ids[name] = sorted(value)
    need(type(terminal.get('physical_cutoff_required')) is bool, 'Original cutoff requirement required')
    pending = set(stop_ids['unconfirmed_ids']) | set(stop_ids['ambiguous_ids'])
    return {'schema': 'singularitydog.four-bus-type1-failure-analysis.v1', 'status': 'ANALYZED_SAVED_RAW_JOURNALS',
            'hardware_opened': False, 'output_allowed': False, 'approved_for_runtime': False,
            'original_report_modified': False, 'runtime_source_modified': False,
            'source_trial_success_inferred': False, 'completed_cycles': completed,
            'first_cycle_started': first is not None, 'policy_hz': 30 if is30 else 50,
            'inference_tx_budget_ms': 20, 'cycle_reply_budget_ms': 33.333334 if is30 else 20,
            'raw_batches': batches, 'host_written_requests': requests, 'complete_recorded_replies': replies,
            'request_kinds': dict(sorted(counts.items())), 'failed_requests': failures,
            'terminal_STOP_owner_summary': {'stop_confirmed': terminal['stop_confirmed'], **stop_ids,
                'physical_cutoff_required': terminal['physical_cutoff_required'],
                'contradictory_complete_claim': terminal['stop_confirmed'] and bool(pending),
                'mode_zero_bytes_observed_for_unconfirmed_ids': sorted(stop_observations & pending)},
            'sticky_STOP_ambiguity_cleared': False, 'stop_confirmation_created': False,
            'physical_layer_root_cause_confirmed': False,
            'root_cause': 'UNRESOLVED_MOTOR_CAN_ADAPTER_USB_OR_HOST_LATENCY',
            'measurement_scope': 'host native write/read records; no CAN-wire arrival or physical-stop inference'}


def strict_pairs(pairs):
    value = {}
    for key, item in pairs:
        need(key not in value, 'Duplicate JSON key')
        value[key] = item
    return value


def load_report(path, expected_sha256):
    need(type(expected_sha256) is str and re.fullmatch('[0-9a-f]{64}', expected_sha256), 'Exact lowercase report SHA256 required')
    path = Path(path)
    need(path.is_absolute(), 'Absolute report path required')
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'rb') as handle:
        before = os.fstat(handle.fileno())
        need(stat.S_ISREG(before.st_mode) and 0 < before.st_size <= MAX_REPORT_BYTES, 'Bounded regular report required')
        raw = handle.read(MAX_REPORT_BYTES+1)
        after = os.fstat(handle.fileno())
    need((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
         (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) and len(raw) == before.st_size,
         'Report changed while reading')
    need(hashlib.sha256(raw).hexdigest() == expected_sha256, 'Report SHA256 mismatch')
    value = json.loads(raw, object_pairs_hook=strict_pairs,
                       parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON value')))
    return value


def write_fresh(path, value):
    path = Path(path)
    need(path.is_absolute() and path.parent.is_dir() and
         not any(p.is_symlink() or (p/'.git').exists() for p in path.parents),
         'Fresh private absolute output outside Git required')
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600), 'w') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--report-sha256', required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    result = analyze(load_report(args.report, args.report_sha256))
    result['source_report_sha256'] = args.report_sha256
    if args.output:
        write_fresh(args.output, result)
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
