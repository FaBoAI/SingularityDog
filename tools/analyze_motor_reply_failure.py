"""Inspect saved zero-gain commissioning exchanges without opening hardware.

The report records host writes and reads, not CAN-wire activity. A complete
host write cannot identify whether a reply was lost at the motor, CAN bridge,
USB, or host. Type2 has no transaction nonce; a later reset observation never
erases the session owner's ambiguous STOP result. Output contains no raw UID,
boot, power label, file location, frame bytes, or approval.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.motor_version_probe import VERSION_PREFIX, version_request

BUSES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
PARAMETERS = ('identity', 'run_mode', 'voltage', 'can_timeout')
MAX_FILE_BYTES = 32 * 1024 * 1024


def need(value, message):
    if not value:
        raise ValueError(message)


def stamp(value):
    need(type(value) is int and 0 < value < 2**63, 'Invalid host timestamp')
    return value


def wire_bytes(value):
    need(type(value) is str and len(value) <= 8192 and len(value) % 2 == 0 and
         re.fullmatch('[0-9a-fA-F]*', value) is not None, 'Invalid recorded wire bytes')
    return bytes.fromhex(value)


def ids(value, bus):
    need(type(value) is list and all(type(mid) is int and mid in BUSES[bus] for mid in value)
         and len(value) == len(set(value)),
         'Invalid bus ID list')
    return sorted(value)


def expected_wire(mid, step):
    if step in PARAMETERS:
        return codec.read_request(mid, None if step == 'identity' else step)
    if step == 'version':
        return version_request(mid)
    phase = {'enable': protocol.TrialPhase.ENABLE, 'stop': protocol.TrialPhase.STOP,
             'watchdog_write': protocol.TrialPhase.WATCHDOG_SETUP}.get(step)
    if step == 'enable':
        return protocol.enable_request(phase=phase, motor_id=mid)
    if step == 'stop':
        return protocol.stop_request(phase=phase, motor_id=mid)
    if step == 'watchdog_write':
        return protocol.watchdog_setup_request(phase=phase, motor_id=mid)
    if step == 'zero':
        return None  # The bounded center is not repeated in the public summary.
    raise ValueError('Unknown commissioning step')


def is_candidate(frame, mid, step):
    if step in PARAMETERS:
        return codec.matches(frame, mid, None if step == 'identity' else step)
    if step == 'version':
        return (frame.kind == 2 and frame.source == mid and
                frame.destination == codec.HOST_ID and frame.data[:3] == VERSION_PREFIX)
    return (frame.kind == 2 and frame.source == mid and
            frame.destination == codec.HOST_ID and frame.data[:3] != VERSION_PREFIX)


def analyze_bus(events, stop_report, bus):
    need(type(events) is list and len(events) <= 4096, 'Invalid event list')
    need(type(stop_report) is dict, 'Missing owner STOP result')
    confirmed = ids(stop_report.get('confirmed_ids'), bus)
    unconfirmed = ids(stop_report.get('unconfirmed_ids'), bus)
    ambiguous = ids(stop_report.get('ambiguous_ids'), bus)
    need(not set(confirmed).intersection(unconfirmed + ambiguous), 'Contradictory STOP ID sets')
    need(set(ambiguous).issubset(unconfirmed), 'Ambiguous STOP IDs must remain unconfirmed')
    need(set(confirmed + unconfirmed) == set(BUSES[bus]), 'Incomplete STOP ID accounting')
    need(type(stop_report.get('complete')) is bool, 'Missing STOP completion flag')
    parser = codec.ATParser()
    transactions = []
    current = None
    previous_time = 0
    rejected_chunks = 0
    unknown_time_chunks = 0
    orphan_rx_bytes = 0
    for event in events:
        need(type(event) is dict, 'Malformed commissioning event')
        kind = event.get('kind')
        if kind == 'tx':
            mid, step = event.get('motor_id'), event.get('step')
            need(type(mid) is int and mid in BUSES[bus] and type(step) is str,
                 'Cross-bus/invalid request')
            start, finish = stamp(event.get('start_ns')), stamp(event.get('finish_ns'))
            need(previous_time <= start <= finish, 'Noncausal request timestamps')
            raw = wire_bytes(event.get('hex'))
            expected = expected_wire(mid, step)
            if expected is not None:
                need(raw == expected, 'Request step differs from recorded wire')
            else:
                outgoing = codec.ATParser()
                frames = outgoing.feed(raw)
                need(len(frames) == 1 and not outgoing.buffer and not outgoing.discarded_bytes and
                     len(raw) == 17 and frames[0].flags == 4 and frames[0].kind == 1 and
                     frames[0].destination == mid and
                     (frames[0].can_id >> 8) & 65535 == 32767 and
                     frames[0].data[2:4] == b'\x7f\xff' and frames[0].data[4:] == bytes(4),
                     'Invalid zero-gain request')
            returned = event.get('returned_bytes')
            need(returned is None or type(returned) is int and 0 <= returned <= len(raw),
                 'Invalid write byte count')
            if current is not None:
                current['observed_until_next_tx_ms'] = (start - current.pop('_start_ns')) / 1e6
            current = {'motor_id': mid, 'step': step, '_start_ns': start,
                       'write_wall_ms': (finish - start) / 1e6,
                       'host_write_complete': returned == len(raw), 'received_bytes': 0,
                       'reply_candidate_count': 0, 'first_reply_after_write_ms': None,
                       'observed_reset': False, 'rx_partial_before_send_bytes': len(parser.buffer),
                       'rejected_receive_chunk_count': 0, 'causal_confirmation_inferred': False}
            transactions.append(current)
            previous_time = finish
        elif kind in ('rx_bytes', 'rx_rejected'):
            raw = wire_bytes(event.get('hex'))
            received = event.get('received_ns')
            if received is not None:
                received = stamp(received)
                need(received >= previous_time, 'Noncausal receive timestamps')
                previous_time = received
            else:
                need(kind == 'rx_rejected', 'Missing receive timestamp')
                unknown_time_chunks += 1
            if kind == 'rx_rejected':
                rejected_chunks += 1
            if current is None:
                orphan_rx_bytes += len(raw)
            else:
                current['received_bytes'] += len(raw)
                current['rejected_receive_chunk_count'] += int(kind == 'rx_rejected')
            frames = parser.feed(raw)
            for frame in frames:
                if current is None or received is None:
                    continue
                need(frame.flags == 4 and len(frame.data) == 8, 'Noncanonical recorded reply')
                if not is_candidate(frame, current['motor_id'], current['step']):
                    continue
                current['reply_candidate_count'] += 1
                if current['first_reply_after_write_ms'] is None:
                    current['first_reply_after_write_ms'] = (received - current['_start_ns']) / 1e6
                if frame.kind == 2 and frame.data[:3] != VERSION_PREFIX:
                    feedback = protocol.decode_type2(frame, motor_id=current['motor_id'])
                    current['observed_reset'] |= feedback.mode_state == 0 and feedback.fault_bits == 0
        else:
            raise ValueError('Unknown recorded event kind')
    for row in transactions:
        row.pop('_start_ns', None)
        if not row['host_write_complete']:
            row['classification'] = 'HOST_WRITE_INCOMPLETE'
        elif row['reply_candidate_count']:
            row['classification'] = 'REPLY_CANDIDATE_OBSERVED_NOT_CAUSAL_CONFIRMATION'
        elif row['received_bytes']:
            row['classification'] = 'RX_BYTES_WITHOUT_MATCHING_COMPLETE_REPLY'
        else:
            row['classification'] = 'FULL_HOST_WRITE_WITHOUT_RECORDED_RX'
    suspicious = [row for row in transactions if not row['reply_candidate_count'] or
                  not row['host_write_complete'] or row['rx_partial_before_send_bytes'] or
                  row['rejected_receive_chunk_count']]
    stop_suspect = bool(unconfirmed or ambiguous or not stop_report['complete'] or
                        stop_report.get('errors') or stop_report.get('error') or
                        stop_report.get('sticky_boundary_uncertain'))
    return {'transactions': len(transactions), 'suspicious_exchanges': suspicious,
            'confirmed_stop_ids_from_owner': confirmed,
            'unconfirmed_stop_ids_from_owner': unconfirmed,
            'ambiguous_stop_ids_from_owner': ambiguous,
            'owner_stop_incomplete_or_errored': stop_suspect,
            'reset_observed_during_stop_ids': sorted({row['motor_id'] for row in transactions
                 if row['step'] == 'stop' and row['observed_reset']}),
            'parser_partial_bytes_at_end': len(parser.buffer),
            'parser_discarded_bytes': parser.discarded_bytes,
            'rejected_receive_chunks': rejected_chunks,
            'receive_chunks_without_valid_time': unknown_time_chunks,
            'orphan_receive_bytes': orphan_rx_bytes}


def analyze(report):
    need(type(report) is dict and type(report.get('stop_confirmed')) is bool,
         'Expected commissioning report with explicit STOP summary')
    events, stops = report.get('events_by_bus'), report.get('stop_reports')
    need(type(events) is dict and type(stops) is dict and set(events) == set(stops) == set(BUSES),
         'Expected both commissioning buses and owner STOP results')
    buses = {bus: analyze_bus(events[bus], stops[bus], bus) for bus in BUSES}
    owner_unconfirmed = any(row['owner_stop_incomplete_or_errored'] for row in buses.values())
    return {'schema': 'singularitydog.motor-reply-failure-analysis.v1',
            'status': 'ANALYZED_SAVED_EVENTS', 'hardware_opened': False,
            'output_allowed': False, 'root_cause_established': False,
            'stop_confirmation_created': False,
            'reported_stop_confirmed': report['stop_confirmed'],
            'contradictory_stop_summary': report['stop_confirmed'] and owner_unconfirmed,
            'unresolved_stop_evidence': owner_unconfirmed, 'buses': buses,
            'measurement_scope': 'host write/read only; no CAN-wire or physical-stop inference',
            'next_diagnostic': ['Compare the failed ID with a successful ID at identical firmware/settings',
                                'Record host serial counters and USB/kernel events during a zero-gain run',
                                'Keep earlier missing same-key replies ambiguous through repeated STOP']}


def load_report(path, expected_sha256=None):
    need(expected_sha256 is None or type(expected_sha256) is str and
         re.fullmatch('[0-9a-f]{64}', expected_sha256), 'Invalid expected SHA256')
    # fstat applies to the opened file; O_NOFOLLOW rejects a final-path symlink.
    fd = os.open(Path(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= MAX_FILE_BYTES,
             'Expected a bounded regular saved report')
        raw = handle.read(MAX_FILE_BYTES + 1)
    need(0 < len(raw) <= MAX_FILE_BYTES, 'Saved report exceeds size limit')
    sha = hashlib.sha256(raw).hexdigest()
    need(expected_sha256 is None or sha == expected_sha256, 'Saved report SHA256 mismatch')
    def pairs(rows):
        value = {}
        for key, item in rows:
            need(key not in value, 'Duplicate saved JSON key')
            value[key] = item
        return value
    def number(value):
        result = float(value)
        need(math.isfinite(result), 'Nonfinite saved JSON number')
        return result
    def constant(value):
        raise ValueError('Nonfinite saved JSON constant')
    return json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                      parse_float=number, parse_constant=constant), sha


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--expected-sha256')
    args = parser.parse_args(argv)
    try:
        report, sha = load_report(args.report, args.expected_sha256)
        result = analyze(report)
        result['report_sha256'] = sha
        print(json.dumps(result, sort_keys=True, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError, UnicodeError, OverflowError) as error:
        # Avoid copying private paths or original exception text into the summary.
        print(json.dumps({'status': 'ANALYSIS_REJECTED', 'error_type': type(error).__name__,
                          'output_allowed': False, 'root_cause_established': False}))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
