"""Classify saved active-transport failures without opening any hardware.

Host write completion is not CAN delivery. Missing-at-deadline does not prove
physical packet loss. A STOP receive buffer is evidence, never a recovered ACK.
"""
import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import statistics


def frame(wire_hex):
    raw = bytes.fromhex(wire_hex)
    if (len(raw) != 17 or raw[:2] != b'AT' or raw[5] & 7 != 4 or
            raw[6] != 8 or raw[15:] != b'\r\n'):
        raise ValueError('Noncanonical 17-byte AT frame')
    cid = int.from_bytes(raw[2:6], 'big') >> 3
    return {'kind': cid >> 24, 'source': (cid >> 8) & 255,
            'destination': cid & 255, 'mode': (cid >> 22) & 3,
            'fault': (cid >> 16) & 63}


def distribution(values):
    return ({'count': len(values), 'median': statistics.median(values),
             'max': max(values)} if values else {'count': 0})


def analyze(report):
    journal = report.get('journal')
    if not isinstance(journal, list) or not journal:
        raise ValueError('Nonempty active transport journal required')
    groups = defaultdict(lambda: {'writes': 0, 'replies': 0, 'latency_ms': []})
    failures = []
    for batch_index, batch in enumerate(journal):
        bus = batch['bus']
        if bus not in ('front', 'rear'):
            raise ValueError('Unknown bus')
        missing = []
        for row in batch['records']:
            if not row['written']:
                continue
            tx = frame(row['tx_hex'])
            mid = tx['destination']
            if mid not in (range(1, 7) if bus == 'front' else range(7, 13)):
                raise ValueError('Request ID on wrong bus')
            if not 0 < row['start_ns'] <= row['finish_ns']:
                raise ValueError('Invalid write timestamps')
            key = (bus, tx['kind'], mid)
            group = groups[key]
            group['writes'] += 1
            if row['written'] != 17:
                missing.append({'id': mid, 'kind': tx['kind'], 'state': 'partial_write'})
            elif row['received'] == 17:
                rx = frame(row['rx_hex'])
                if rx['source'] != mid or not row['finish_ns'] <= row['read_start_ns'] <= row['received_ns']:
                    raise ValueError('Mismatched/noncausal recorded reply')
                group['replies'] += 1
                group['latency_ms'].append((row['received_ns'] - row['finish_ns']) / 1e6)
            else:
                if row['received'] != 0:
                    raise ValueError('Native record must contain zero or seventeen received bytes')
                missing.append({'id': mid, 'kind': tx['kind'], 'state': 'no_complete_reply_at_deadline',
                                'write_to_deadline_ms': (row['deadline_ns'] - row['finish_ns']) / 1e6})
        if not batch.get('error') and not missing:
            continue
        rejected = bytes.fromhex(batch.get('rejected_hex', ''))
        item = {'journal_index': batch_index, 'bus': bus, 'phase': batch['phase'],
                'error': batch.get('error'), 'unresolved': missing,
                'rx_bytes': batch['stats']['bytes'], 'rx_reads': batch['stats']['reads'],
                'unparsed_bytes': len(rejected), 'unparsed_hex': rejected.hex(),
                'late_tail_candidate': None, 'skip_recovery_proven': False}
        stop = report.get('stop_reports', {}).get(bus, {})
        attempts = stop.get('attempts', [stop])
        # Only the first following STOP buffer can be a contiguous continuation.
        # This is a reconstruction candidate, not attribution or exact arrival time.
        if 7 <= len(rejected) < 17 and rejected[:2] == b'AT' and attempts:
            evidence = attempts[0].get('evidence', {})
            tail = bytes.fromhex(evidence.get('rejected_hex', ''))
            needed = 17 - len(rejected)
            if (not batch.get('rejected_truncated', False) and
                    not evidence.get('rejected_truncated', False) and len(tail) >= needed and
                    evidence.get('stats', {}).get('begin_ns', 0) >= batch['stats']['end_ns']):
                try:
                    decoded = frame((rejected + tail[:needed]).hex())
                except ValueError:
                    decoded = None
                pending = {r['id'] for r in missing if r['kind'] == 1}
                if (decoded and decoded['kind'] == 2 and decoded['source'] in pending and
                        decoded['destination'] == 0xfd and decoded['mode'] == 2 and not decoded['fault']):
                    item['late_tail_candidate'] = {
                        'id': decoded['source'], 'prefix_bytes': len(rejected),
                        'later_bytes_hex': tail[:needed].hex(),
                        'observed_during': 'first STOP collection; individual byte timestamp unavailable',
                        'arrival_within_one_extra_cycle_proven': False}
        failures.append(item)
    rows = [{'bus': bus, 'kind': kind, 'id': mid, 'writes': g['writes'],
             'replies': g['replies'], 'incomplete_records': g['writes'] - g['replies'],
             'write_return_to_reply_ms': distribution(g['latency_ms'])}
            for (bus, kind, mid), g in sorted(groups.items())]
    return {'schema': 'can-reply-loss-analysis-v1', 'source_status': report.get('status'),
            'completed_cycles': len(report.get('cycles', [])), 'request_groups': rows,
            'failed_batches': failures,
            'unresolved_counts_by_id': dict(sorted(Counter(str(r['id']) for b in failures
                                                         for r in b['unresolved']).items())),
            'root_cause': 'UNRESOLVED_USB_UART_ADAPTER_CAN_OR_DEVICE',
            'scope': 'host application records; aborted trials are not an unbiased loss-rate sample',
            'hardware_opened': False, 'runtime_skip_approved': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    result = []
    for path in args.reports:
        raw = path.read_bytes()
        row = analyze(json.loads(raw))
        row.update(source_name=path.parent.name, source_sha256=hashlib.sha256(raw).hexdigest())
        result.append(row)
    text = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
