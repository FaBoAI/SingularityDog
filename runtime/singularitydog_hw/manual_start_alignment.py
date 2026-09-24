"""Finite read-only RR manual alignment display; no drive/calibration API exists.

Default CLI is plan-only. Execute requires an explicit operator declaration of
relaxed motors, supported body and free legs. Type0/17 reads do not prove motor
disable or make manual contact safe. A matching display is guidance only; the
separate powered trial must still repeat its own fresh gates.
"""
import argparse
from contextlib import ExitStack, contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import sys
import time
import uuid

from . import can_readonly as readonly

IDS = (7, 8, 9)
PERIOD_NS = 200_000_000
MAX_AGE_NS = 100_000_000
TOLERANCE_RAD = math.radians(.5)
MAX_SECONDS = 120
MAX_REQUESTS = 1803
MAX_EVENTS = 50_000
_RETAINED_LOCKS = []
FALSE_FLAGS = {'motor_output_available': False, 'output_allowed': False,
               'approved_for_runtime': False, 'calibration_modified': False,
               'angle_wrapping_applied': False, 'disabled_mode_verified': False,
               'stationarity_verified': False, 'automatic_next_action': False}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strict_json(text):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def invalid(value):
        raise ValueError('Nonfinite JSON: ' + value)
    return json.loads(text, object_pairs_hook=pairs, parse_constant=invalid)


def digest_valid(value):
    return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def seconds_valid(seconds):
    require(type(seconds) is int and 1 <= seconds <= MAX_SECONDS, 'seconds must be integer1..120')
    return seconds


def identities(value):
    require(type(value) is dict and set(value) == {str(i) for i in range(1, 13)}, 'Twelve expected IDs required')
    require(all(type(v) is str and len(v) == 16 and all(c in '0123456789abcdef' for c in v)
                for v in value.values()) and len(set(value.values())) == 12, 'Malformed/duplicate UID')
    return value


def validate_reference(reference, expected, expected_digest):
    require(reference.get('schema') == 'singularitydog.private-matched-start-reference.v1'
            and reference.get('leg') == 'RR' and reference.get('ids') == list(IDS), 'RR matched reference required')
    boot = reference.get('boot_id')
    require(type(boot) is str and str(uuid.UUID(boot)) == boot, 'Canonical pinned boot required')
    require(identities(reference.get('expected_uids')) == expected
            and reference.get('expected_uids_file_sha256') == expected_digest, 'Reference identity mismatch')
    ports = reference.get('ports', {})
    require(type(ports) is dict and set(ports) == {'front', 'rear'} and len(set(ports.values())) == 2,
            'Two distinct pinned by-path mappings required')
    require(all(type(p) is str and Path(p).is_absolute() and Path(p).parent == Path('/dev/serial/by-path')
                for p in ports.values()), 'Explicit by-path names required')
    require(all(reference.get(k) is False for k in ('output_allowed', 'runtime_approved',
            'calibration_modified', 'angle_wrapping_applied', 'old_r2_AB_equivalent')), 'Reference approval flags invalid')
    source = reference.get('source', {})
    require(source.get('stage') == 'before' and source.get('raw_readonly_pairs_verified') == 492
            and all(digest_valid(source.get(k)) for k in ('summary_sha256', 'events_sha256', 'wrapper_sha256')),
            'Reference source provenance missing')
    positions, targets = reference.get('positions_rad'), reference.get('absolute_targets_rad')
    require(type(positions) is dict and type(targets) is dict
            and set(positions) == set(targets) == {'7', '8', '9'}, 'Exact reference axes required')
    rows = reference.get('source_before_positions', {})
    require(set(rows) == set(positions), 'Reference timestamps missing')
    for mid, q in positions.items():
        require(type(q) in (int, float) and math.isfinite(q) and -12.57 <= q <= 12.57,
                'Invalid raw reference position')
        target = targets[mid]
        require(type(target) in (int, float) and math.isfinite(target)
                and target == q - (math.radians(4) if mid == '9' else 0)
                and -12.57 <= target <= 12.57, 'Unexpected matched diagnostic target')
        row = rows[mid]
        request, received = row.get('request_monotonic_ns'), row.get('received_monotonic_ns')
        require(row.get('raw_position_rad') == q and type(request) is int and type(received) is int
                and 0 <= request < received, 'Invalid reference source time')
    return reference


def load_inputs(reference_path, reference_digest, uid_path, uid_digest):
    require(digest_valid(reference_digest) and digest_valid(uid_digest), 'Explicit pinned input hashes required')
    for path, digest in ((reference_path, reference_digest), (uid_path, uid_digest)):
        require(Path(path).is_file() and not Path(path).is_symlink() and sha(path) == digest, 'Input hash mismatch')
    expected = identities(strict_json(Path(uid_path).read_text()))
    reference = validate_reference(strict_json(Path(reference_path).read_text()), expected, uid_digest)
    return reference, expected


def validate_bindings(ports):
    result = {}
    for bus, name in ports.items():
        path = Path(name)
        resolved = path.resolve(strict=True)
        info = resolved.stat()
        require(stat.S_ISCHR(info.st_mode), 'CAN port is not a character device')
        result[bus] = {'path': str(path), 'resolved': str(resolved), 'st_rdev': info.st_rdev}
    require(result['front']['resolved'] != result['rear']['resolved']
            and result['front']['st_rdev'] != result['rear']['st_rdev'], 'Physical CAN aliases collide')
    return result


def check_binding(binding, raw=None):
    resolved = Path(binding['path']).resolve(strict=True)
    info = resolved.stat()
    require(str(resolved) == binding['resolved'] and stat.S_ISCHR(info.st_mode)
            and info.st_rdev == binding['st_rdev'], 'CAN binding changed')
    if raw is not None:
        opened = os.fstat(raw.fileno())
        require(stat.S_ISCHR(opened.st_mode) and opened.st_rdev == binding['st_rdev'], 'Opened FD mismatch')


@contextmanager
def ownership(bindings, report):
    import fcntl
    root = Path.home() / '.cache' / 'singularitydog'
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    names = [root / 'manual-calibration.lock', root / 'can-readonly.lock']
    names += [Path('/tmp/singularitydog-can-port-' + hashlib.sha256(bindings[b]['resolved'].encode()).hexdigest()[:24] + '.lock')
              for b in ('front', 'rear')]
    stack, entered = ExitStack(), False
    try:
        for name in names:
            handle = stack.enter_context(name.open('a+'))
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        entered = True
        report['locks_acquired'] = True
        yield
    finally:
        if entered and report.get('serial_closed') is False:
            _RETAINED_LOCKS.append(stack)
            report['locks_released'] = False
            report['errors'].append('Serial close unconfirmed; locks retained until process exit')
        else:
            stack.close()
            report['locks_released'] = entered


def serial_factory():
    import serial
    return serial.Serial(port=None, baudrate=921600, bytesize=8, parity='N', stopbits=1,
                         timeout=.003, write_timeout=.1, exclusive=True,
                         xonxoff=False, rtscts=False, dsrdtr=False)


class GuardedSerial:
    """Final byte boundary accepts only the current canonical read exactly once."""
    def __init__(self, raw, check, clock, deadline_ns, emit, max_requests):
        self.raw, self.check, self.clock, self.deadline_ns, self.emit = raw, check, clock, deadline_ns, emit
        self.max_requests, self.write_count = max_requests, 0
        self.expected_wire, self.written, self.query_deadline_ns = None, False, deadline_ns
        self.allowlist = {readonly.read_request(mid, parameter) for mid in IDS for parameter in (None, 'position')}

    def remaining(self):
        self.check()
        remaining = min(self.deadline_ns, self.query_deadline_ns) - self.clock()
        require(remaining > 0, 'Finite alignment/query deadline expired')
        return remaining / 1e9

    @property
    def in_waiting(self):
        self.remaining()
        return self.raw.in_waiting

    def read(self, count):
        self.raw.timeout = min(.003, self.remaining())
        data = self.raw.read(count)
        self.remaining()
        return data

    def write(self, wire):
        require(type(wire) is bytes and wire in self.allowlist and wire == self.expected_wire
                and not self.written and self.write_count < self.max_requests <= MAX_REQUESTS,
                'Write blocked: not the one canonical pending RR read or request limit reached')
        self.remaining()
        self.emit({'kind': 'alignment_write_ready', 'monotonic_ns': self.clock(), 'next_write_count': self.write_count+1})
        # The synchronous event sink may block. Recheck immediately before I/O.
        self.raw.write_timeout = min(.1, self.remaining())
        self.write_count += 1
        self.written = True
        count = self.raw.write(wire)
        self.emit({'kind': 'alignment_write_returned', 'monotonic_ns': self.clock(),
                   'write_count': self.write_count, 'bytes_written': count})
        self.remaining()
        require(type(count) is int and count == len(wire), 'Partial serial write; no retry')
        return count


class StrictReads:
    def __init__(self, raw, check, emit, clock, deadline_ns, max_requests):
        self.check, self.emit, self.clock = check, emit, clock
        self.proxy = GuardedSerial(raw, check, clock, deadline_ns, emit, max_requests)
        self.pending = None
        self.can = readonly.ReadOnlyCAN(serial_port=self.proxy, timeout_s=.1, event_sink=self.sink, clock=clock)

    def sink(self, event):
        self.emit(event)
        state = self.pending
        require(state is not None, 'Unsolicited data outside a pending read')
        if event['kind'] == 'can_tx':
            require(not state['tx'] and event['hex'] == self.proxy.expected_wire.hex(), 'Noncanonical/duplicate read TX')
            state['tx'] = True
        elif event['kind'] == 'can_rx_frame':
            mid, parameter = state['key']
            require(state['tx'] and self.proxy.written and state['frames'] == 0
                    and event['flags'] == 4 and event['source_id'] == mid
                    and event['destination_id'] == (254 if parameter is None else 253)
                    and event['type'] == (0 if parameter is None else 17), 'Wrong/duplicate/unsolicited CAN frame')
            data = bytes.fromhex(event['data_hex'])
            require(len(data) == 8 and (parameter is None or int.from_bytes(data[:2], 'little')
                    == readonly.PARAMETERS['position'][0]), 'Wrong parameter reply')
            state['frames'] += 1

    def query(self, mid, parameter=None):
        require(type(mid) is int and mid in IDS and parameter in (None, 'position'), 'Only RR identity/position reads')
        self.check()
        self.proxy.query_deadline_ns = self.proxy.deadline_ns
        remaining_ns = self.proxy.deadline_ns - self.clock()
        require(remaining_ns >= 10_000_000, 'Insufficient query budget; no request sent')
        require(not self.can.parser.buffer and not self.can.parser.discarded_bytes
                and not self.proxy.in_waiting, 'Residual input before request')
        self.proxy.expected_wire = readonly.read_request(mid, parameter)
        self.proxy.written = False
        self.proxy.query_deadline_ns = min(self.proxy.deadline_ns, self.clock() + MAX_AGE_NS)
        self.can.timeout_s = min(.1, remaining_ns / 1e9)
        self.pending = {'key': (mid, parameter), 'tx': False, 'frames': 0}
        result = self.can.query(mid, parameter)
        require(result.get('ok') is True and self.pending['tx'] and self.pending['frames'] == 1,
                'Missing/invalid fresh reply')
        require(not self.can.parser.buffer and not self.can.parser.discarded_bytes
                and not self.proxy.in_waiting, 'Residual input after reply')
        self.check()
        require(type(result['request_monotonic_ns']) is int and type(result['monotonic_ns']) is int
                and result['request_monotonic_ns'] < result['monotonic_ns'] <= self.clock()
                and self.clock() - result['request_monotonic_ns'] <= MAX_AGE_NS, 'Stale/invalid reply time')
        self.pending = None
        self.proxy.expected_wire = None
        return result


def alignment_row(reference, readings, index, checked_ns):
    require(set(readings) == set(IDS), 'All three new readings required')
    joints = {}
    for mid in IDS:
        result = readings[mid]
        q = result.get('value')
        request, received = result['request_monotonic_ns'], result['monotonic_ns']
        require(type(q) in (float, int) and math.isfinite(q) and -12.57 <= q <= 12.57
                and type(request) is int and type(received) is int and request < received <= checked_ns
                and checked_ns - request <= MAX_AGE_NS, 'Invalid/stale display sample')
        ref = reference['positions_rad'][str(mid)]
        joints[str(mid)] = {'raw_position_rad': q, 'reference_rad': ref,
            'raw_delta_rad': q-ref, 'raw_delta_deg': math.degrees(q-ref),
            'within_candidate_tolerance': ref-TOLERANCE_RAD <= q <= ref+TOLERANCE_RAD,
            'request_ns': request, 'received_ns': received, 'age_upper_bound_ns': checked_ns-request}
    return {'kind': 'manual_alignment_sample', 'sample_index': index, 'checked_ns': checked_ns,
            'joints': joints, 'all_three_within_candidate_tolerance': all(r['within_candidate_tolerance'] for r in joints.values()),
            'acquisition_span_ns': max(r['received_ns'] for r in joints.values()) - min(r['request_ns'] for r in joints.values()),
            'tolerance_candidate_deg': .5, **FALSE_FLAGS}


def display_row(row):
    values = '  '.join('ID' + mid + ' 差' + format(j['raw_delta_deg'], '+.2f') + '°'
                       for mid, j in row['joints'].items())
    matched = '3軸が候補範囲内（表示のみ）' if row['all_three_within_candidate_tolerance'] else '手動調整中'
    print(values + '  ' + matched, flush=True)


def observe(reference, expected, bindings, seconds, emit, check_external, report, *,
            factory=serial_factory, clock=time.monotonic_ns, wait=time.sleep, fd_check=check_binding,
            display=display_row):
    """Single finite session; declaration/locks/log lifecycle are owned by main."""
    seconds_valid(seconds)
    max_requests = 3 + 3 * seconds * 5
    start = clock()
    deadline = start + seconds * 1_000_000_000
    raw = None
    reader = None
    report.update(max_requests=max_requests, acquisition_started_ns=start, acquisition_deadline_ns=deadline,
                  serial_closed=None, samples_completed=0, matched_display_samples=0, tx_count=0)
    def check():
        check_external()
        require(clock() < deadline, 'Finite alignment deadline exceeded')
        fd_check(bindings['rear'], raw)
    try:
        check_external()
        raw = factory()
        report['serial_closed'] = False
        raw.dtr = raw.rts = False
        raw.port = bindings['rear']['path']
        raw.open()
        check()
        reader = StrictReads(raw, check, emit, clock, deadline, max_requests)
        for mid in IDS:
            result = reader.query(mid)
            require(result['mcu_uid_hex'] == expected[str(mid)], 'ID' + str(mid) + ' identity mismatch')
        report['selected_identities_verified'] = True
        schedule_start = clock()
        previous_received = {mid: None for mid in IDS}
        for index in range(seconds * 5):
            due = schedule_start + index * PERIOD_NS
            if due + MAX_AGE_NS >= deadline:
                break
            wait(max(0, due-clock()) / 1e9)
            check()
            require(clock()-due <= MAX_AGE_NS, 'Alignment loop late; no catch-up')
            readings = {mid: reader.query(mid, 'position') for mid in IDS}
            for mid, value in readings.items():
                require(previous_received[mid] is None or value['request_monotonic_ns'] > previous_received[mid],
                        'Reused/overlapping previous source sample')
            row = alignment_row(reference, readings, index, clock())
            emit(row)
            # A synchronous log write must not turn an old sample into a live display.
            check()
            display_ns = clock()
            require(all(0 <= display_ns-value['request_monotonic_ns'] <= MAX_AGE_NS
                        for value in readings.values()), 'Sample became stale before display')
            display(row)
            check()
            for mid, value in readings.items(): previous_received[mid] = value['monotonic_ns']
            report['samples_completed'] += 1
            report['matched_display_samples'] += int(row['all_three_within_candidate_tolerance'])
        require(report['samples_completed'] > 0, 'No complete current-position display samples')
        report['observation_completed'] = True
    finally:
        if reader is not None: report['tx_count'] = reader.proxy.write_count
        if raw is not None:
            raw.close()
            report['serial_closed'] = True
        report['acquisition_finished_ns'] = clock()


def make_plan(seconds):
    seconds_valid(seconds)
    return {'leg': 'RR', 'ids': list(IDS), 'seconds': seconds, 'nominal_display_hz': 5,
            'max_requests': 3+15*seconds, 'allowed_can_types': [0, 17],
            'allowed_parameters': ['identity', 'position'], 'candidate_tolerance_deg': .5,
            'operator_moves_joints': True, 'reference_recentered': False,
            'read_mode_is_not_disable_proof': True, **FALSE_FLAGS}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-readonly', action='store_true')
    ap.add_argument('--operator-confirmed-relaxed-supported', action='store_true')
    ap.add_argument('--reference', type=Path, required=True)
    ap.add_argument('--reference-sha256', required=True)
    ap.add_argument('--expected-uids', type=Path, required=True)
    ap.add_argument('--expected-uids-sha256', required=True)
    ap.add_argument('--seconds', type=int, default=90)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args(argv)
    plan = make_plan(args.seconds)
    if not args.execute_readonly:
        print(json.dumps({'status': 'PLAN_ONLY', **plan}, indent=2)); return 0
    if not args.operator_confirmed_relaxed_supported:
        ap.error('Operator must explicitly confirm relaxed motors, supported body and free legs')
    reference, expected = load_inputs(args.reference, args.reference_sha256, args.expected_uids, args.expected_uids_sha256)
    output = args.output
    if (output is None or not output.is_absolute() or output.exists() or output.is_symlink()
            or output.resolve() != output
            or any((p/'.git').exists() for p in (output, *output.parents))):
        ap.error('A new absolute private output directory outside Git is required')
    os.umask(0o077)
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    report = {'status': 'INCOMPLETE', 'plan': plan, 'errors': [], 'signals': [],
              'locks_released': False, 'serial_closed': None, 'events_flush_confirmed': False,
              'operator_declared_relaxed_supported': True, 'observation_completed': False,
              'reference_sha256': args.reference_sha256, 'expected_uids_sha256': args.expected_uids_sha256,
              'boot_id': reference['boot_id'], 'source_sha256': {
                  'manual_start_alignment.py': sha(__file__), 'can_readonly.py': sha(readonly.__file__)}, **FALSE_FLAGS}
    handlers, log, bindings, events = {}, None, None, 0
    def check_external():
        require(not report['signals'], 'Manual alignment interrupted')
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == reference['boot_id'], 'Boot changed')
        if bindings:
            for binding in bindings.values(): check_binding(binding)
    def emit(event):
        nonlocal events
        events += 1
        require(events <= MAX_EVENTS, 'Event bound exceeded')
        log.write(json.dumps({'wall_time_ns': time.time_ns(), **event}, allow_nan=False)+'\n')
    try:
        log = (output/'events.jsonl').open('x', buffering=1)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, lambda n, _: report['signals'].append(n))
        check_external()
        bindings = validate_bindings(reference['ports'])
        report['bindings'] = bindings
        print('右後脚を手で合わせます。自動駆動・原点変更はありません。', flush=True)
        print('支持台上で脚が自由に動き、モーターが脱力していることを操作者が確認してください。', flush=True)
        print('差は現在値−固定参照の生角度です。上下方向は表しません。', flush=True)
        print('値の絶対値が小さくなる側へ手でゆっくり。抵抗・引っ掛かりには力を加えずCtrl+C。', flush=True)
        print('±0.5°表示は駆動許可・静止判定ではありません。揃っても自動実行しません。', flush=True)
        print('手を離すとずれる場合は報告し、無理に繰り返し合わせないでください。', flush=True)
        with ownership(bindings, report):
            observe(reference, expected, bindings, args.seconds, emit, check_external, report)
        require(report['locks_released'] and report['serial_closed'] is True, 'Cleanup not confirmed')
        report['status'] = 'READONLY_ALIGNMENT_OBSERVATION_FINISHED'
    except BaseException as error:
        report['errors'].append(repr(error))
    finally:
        for sig, old in handlers.items():
            try: signal.signal(sig, old)
            except BaseException as error: report['errors'].append('SIGNAL_RESTORE_FAILED: ' + repr(error))
        if log is not None:
            try:
                log.flush(); os.fsync(log.fileno()); log.close()
                report['events_flush_confirmed'] = True
                report['events_sha256'] = sha(output/'events.jsonl')
            except BaseException as error:
                report['errors'].append('EVENT_LOG_SAVE_FAILED: ' + repr(error))
                try: log.close()
                except BaseException: pass
        report['event_count'] = events
        if report['signals'] or report['errors'] or not report['events_flush_confirmed']:
            report['status'] = 'INCOMPLETE'
        try:
            with (output/'summary.json').open('x') as stream:
                json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
                stream.flush(); os.fsync(stream.fileno())
        except BaseException as error:
            print('SUMMARY_SAVE_FAILED', repr(error), flush=True); return 2
    print(json.dumps({'status': report['status'], 'samples_completed': report.get('samples_completed', 0),
                      'tx_count': report.get('tx_count', 0), 'errors': report['errors'], 'output_allowed': False}), flush=True)
    return 0 if report['status'] == 'READONLY_ALIGNMENT_OBSERVATION_FINISHED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
