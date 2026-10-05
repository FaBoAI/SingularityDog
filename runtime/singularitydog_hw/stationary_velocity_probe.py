"""Finite single-ID position/velocity/position reads, never motor output.

Default is a device-free PLAN. Explicit execution pins both configured by-path
buses, all twelve identities and the Jetson boot; boot is not motor power epoch.
Host position differences are descriptive, not sensor-time velocity ground truth.
This module does not change any existing stationarity gate or collector.
"""
from contextlib import ExitStack
import argparse
import datetime
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import threading
import time
import uuid

from . import can_readonly as codec
from . import can_timing_probe as timing
from . import dual_can_pipeline_benchmark as dual
from . import sensor_pipeline_benchmark as sensor


SCHEMA = 'singularitydog.stationary-velocity-probe.v1'
PORTS = {'front': '/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0',
         'rear': '/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0'}
IDS = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
READS = ('position', 'velocity', 'position')
MAX_EVENTS, MAX_TRACE_BYTES = 16384, 4 * 1024 * 1024
STARTUP_NS, MAX_SEGMENT_NS = 3_000_000_000, 10_000_000_000
TRIPLET_RESERVE_NS = 50_000_000  # Three inherited 15ms request guards plus 5ms headroom.
# An unknown live serial descriptor must keep every cooperating lease alive.
# CLI process termination releases those leases; this module never retries reads.
_UNCLOSED_DEVICE_LEASES = []
FLAGS = dict.fromkeys(('motor_output_allowed', 'approved_for_runtime', 'motor_enable_sent',
    'stop_sent', 'stop_confirmed', 'learned_targets_sent', 'settings_written', 'automatic_retry',
    'angle_wrap_applied', 'physical_stationarity_proven', 'calibration_approved',
    'sensor_sample_time_verified', 'velocity_replaced_by_position_derivative',
    'existing_comparison_gates_changed', 'full_controller_50Hz_verified', 'period_guaranteed'), False)


def need(value, message):
    if not value:
        raise ValueError(message)


class TripletBudgetUnavailable(TimeoutError):
    """No first physical PVP write may consume the whole-triplet reserve."""


def make_plan(motor_id, samples=50, period_ms=200):
    need(type(motor_id) is int and 1 <= motor_id <= 12, 'Select exactly one ID 1..12')
    need(type(period_ms) is int and period_ms in (20, 200), 'Period is exactly 20 or 200 ms')
    maximum = 500 if period_ms == 20 else 50
    need(type(samples) is int and 1 <= samples <= maximum, 'Samples exceed the ten-second segment bound')
    requests = 12 + 3*samples
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', **FLAGS, 'hardware_opened': False,
            'motor_id': motor_id, 'selected_bus': 'front' if motor_id <= 6 else 'rear',
            'ports': dict(PORTS), 'ids_by_bus': {k: list(v) for k, v in IDS.items()},
            'samples_requested': samples, 'period_ms': period_ms,
            'segment_duration_bound_s': samples*period_ms/1000,
            'maximum_segment_duration_s': 10, 'identity_stage_bound_s': 3,
            'request_timeout_max_ms': 250, 'request_order': list(READS),
            'minimum_time_remaining_before_request_ms': 15,
            'whole_triplet_reserve_ms': TRIPLET_RESERVE_NS/1_000_000,
            'insufficient_segment_budget_policy': 'DROP_REMAINING_SLOTS_BEFORE_ANY_PVP_TX',
            'maximum_queries': requests, 'allowed_can_types': [0, 17],
            'one_outstanding_request': True, 'catchup_available': False,
            'trace_event_budget': MAX_EVENTS, 'trace_byte_budget': MAX_TRACE_BYTES,
            'trace_events_minimum_estimate': requests*5,
            'trace_events_two_receive_chunks_estimate': requests*6,
            'trace_estimate_is_not_a_completion_guarantee': True,
            'motor_power_epoch': 'NOT_INFERRED_FROM_JETSON_BOOT',
            'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
            'external_operator_support_and_idle_confirmation_required': True,
            'timing_scope': 'Finite descriptive reads; 20ms selection is no period, controller or output guarantee.'}


def _strict_json(raw):
    def pairs(rows):
        value = {}
        for key, item in rows:
            need(key not in value, 'Duplicate JSON key')
            value[key] = item
        return value
    def number(value):
        result = float(value); need(math.isfinite(result), 'Nonfinite JSON number'); return result
    def bad(_):
        raise ValueError('Nonfinite JSON constant')
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number, parse_constant=bad)


def _no_symlinks(path):
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Path must not contain symlinks')


def private_path(value):
    path = Path(value).expanduser().absolute()
    _no_symlinks(path)
    need(path.parent.is_dir() and not path.exists(), 'A fresh path with existing parent is required')
    need(not any((p/'.git').exists() for p in path.parents), 'Private raw logs must stay outside Git')
    return path


def load_uids(path):
    path = Path(path).expanduser().absolute(); _no_symlinks(path)
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= 16384, 'Expected bounded regular UID file')
        raw = handle.read(16385)
    need(0 < len(raw) <= 16384, 'UID file exceeds bound')
    return timing.validate_uids(_strict_json(raw)), hashlib.sha256(raw).hexdigest()


def source_hashes():
    return {Path(m.__file__).name: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
            for m in (codec, timing, dual, sensor)} | {
                Path(__file__).name: hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


class PrivateFile:
    """Reserve the artifact before device access; never overwrite a named file."""
    def __init__(self, path):
        self.path = private_path(path)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            os.fchmod(fd, 0o600)
            info = os.fstat(fd)
            self.identity = (info.st_dev, info.st_ino)
            self.stream = os.fdopen(fd, 'w+b', buffering=0)
        except BaseException:
            os.close(fd)
            raise
        self.sha256, self.saved_bytes = None, None

    def binding(self):
        _no_symlinks(self.path)
        opened, named = os.fstat(self.stream.fileno()), self.path.lstat()
        need(stat.S_ISREG(named.st_mode) and (opened.st_dev, opened.st_ino) == (named.st_dev, named.st_ino),
             'Artifact path differs from reserved file')

    def write(self, raw):
        offset = 0
        while offset < len(raw):
            n = self.stream.write(raw[offset:])
            need(type(n) is int and 0 < n <= len(raw)-offset, 'Artifact write made no progress')
            offset += n

    def report(self, value):
        self.binding()
        self.write((json.dumps(value, indent=2, allow_nan=False)+'\n').encode())
        self.stream.flush(); os.fsync(self.stream.fileno()); self.binding()
        raw = os.pread(self.stream.fileno(), self.stream.tell()+1, 0)
        need(len(raw) == self.stream.tell(), 'Incomplete saved report')
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.saved_bytes = len(raw)

    def finalized_sha256(self):
        need(self.stream.closed and self.sha256 is not None, 'Report has not finalized')
        _no_symlinks(self.path)
        fd = os.open(self.path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as handle:
            info = os.fstat(handle.fileno())
            need(stat.S_ISREG(info.st_mode) and (info.st_dev, info.st_ino) == self.identity
                 and info.st_size == self.saved_bytes, 'Saved report binding changed')
            raw = handle.read(self.saved_bytes+1)
            need(hashlib.sha256(raw).hexdigest() == self.sha256, 'Saved report bytes changed')
            named = self.path.lstat()
            need((named.st_dev, named.st_ino) == self.identity, 'Saved report path changed')
        return self.sha256

    def close(self):
        self.stream.close()


class EventTrace(PrivateFile):
    """Independent 16,384-event/4MiB writer; existing writer caps are untouched."""
    def __init__(self, path):
        super().__init__(path)
        self.event_count = self.attempted_event_count = self.byte_count = 0
        self.errors, self.failure, self.sha256, self.closed = [], None, None, False

    def record(self, bus, event):
        self.attempted_event_count += 1
        if self.failure is not None:
            raise self.failure
        try:
            need(not self.closed and self.event_count < MAX_EVENTS, 'Raw event budget exceeded')
            raw = (json.dumps({**event, 'bus': bus}, separators=(',', ':'), allow_nan=False)+'\n').encode()
            need(len(raw) <= MAX_TRACE_BYTES-self.byte_count, 'Raw byte budget exceeded')
            before = self.stream.tell()
            try:
                self.write(raw)
            finally:
                self.byte_count += self.stream.tell()-before
            self.event_count += 1
        except BaseException as error:
            self.failure = error; self.errors.append(type(error).__name__+': '+str(error)); raise

    def close(self):
        if self.closed:
            return self.summary()
        try:
            self.stream.flush(); os.fsync(self.stream.fileno()); self.binding()
            raw = os.pread(self.stream.fileno(), MAX_TRACE_BYTES+1, 0)
            self.sha256 = hashlib.sha256(raw).hexdigest()
            need(len(raw) <= MAX_TRACE_BYTES and len(raw) == self.byte_count
                 and raw.count(b'\n') == self.event_count, 'Raw final count mismatch')
        except BaseException as error:
            self.failure = self.failure or error; self.errors.append(type(error).__name__+': '+str(error))
        finally:
            try:
                super().close()
            except BaseException as error:
                self.failure = self.failure or error; self.errors.append(type(error).__name__+': '+str(error))
            self.closed = True
        return self.summary()

    def summary(self):
        return {'path': str(self.path), 'sha256': self.sha256, 'event_count': self.event_count,
                'attempted_event_count': self.attempted_event_count, 'byte_count': self.byte_count,
                'complete': self.closed and self.failure is None, 'errors': list(self.errors),
                'status': 'COMPLETE_EVENT_TRACE' if self.closed and self.failure is None else 'INCOMPLETE_EVENT_TRACE'}


class ProbeCAN(timing.TimingCAN):
    """Existing strict read transport and physical-write guard, scoped to one bus."""
    def __init__(self, bus, plan, sink, *, check=lambda: None, clock=time.monotonic_ns, serial_port=None):
        need(plan == make_plan(plan['motor_id'], plan['samples_requested'], plan['period_ms']), 'Noncanonical transport plan')
        self.bus, self.selected = bus, plan['motor_id']
        need(bus in IDS, 'Unknown bus')
        limit = 6 + (3*plan['samples_requested'] if bus == plan['selected_bus'] else 0)
        self.request_intents_logged = 0
        def audited_sink(event):
            sink(event)
            if event['kind'] == 'can_tx':
                self.request_intents_logged += 1
        super().__init__(audited_sink, max_requests=limit, max_seconds=13, check_interrupt=check,
                         clock=clock, serial_port=serial_port)
        self.port_name = PORTS[bus]
        self.reply_wire_hex = None
        self.starting_triplet = False

    def record(self, event):
        super().record(event)
        if event['kind'] == 'can_rx_frame':
            self.reply_wire_hex = event['wire_hex']

    def validate_wire(self, wire):
        need(self.pending is not None and self.pending[0] in IDS[self.bus]
             and (self.pending[1] is None or (self.pending[0] == self.selected
                  and self.pending[1] in ('position', 'velocity'))), 'Physical write outside selected bus/ID')
        super().validate_wire(wire)
        if self.starting_triplet and self.deadline_ns-self.clock() < TRIPLET_RESERVE_NS:
            raise TripletBudgetUnavailable('Insufficient segment budget before first PVP physical write')

    def query(self, motor_id, parameter=None):
        need(type(motor_id) is int and motor_id in IDS[self.bus], 'Cross-bus request')
        need(parameter is None or (motor_id == self.selected and parameter in ('position', 'velocity')),
             'Only selected-ID position/velocity parameters are available')
        self.reply_wire_hex = None
        reply = super().query(motor_id, parameter)
        need(self.reply_wire_hex is not None, 'Missing raw matching reply')
        return {**reply, 'raw_request_hex': codec.read_request(motor_id, parameter).hex(),
                'raw_reply_hex': self.reply_wire_hex}


def _stats(values):
    if not values:
        return None
    mean = math.fsum(values)/len(values)
    return {'samples': len(values), 'minimum': min(values), 'maximum': max(values), 'mean': mean,
            'rms': math.sqrt(math.fsum(v*v for v in values)/len(values)),
            'max_abs': max(abs(v) for v in values)}


def validate_receipt(reply, mid, parameter):
    """Reject unknown fields and re-decode exactly one canonical raw reply."""
    need(type(reply) is dict, 'Receipt must be an object')
    request, answer = reply.get('raw_request_hex'), reply.get('raw_reply_hex')
    need(type(request) is str and request == codec.read_request(mid, parameter).hex(), 'Changed raw request')
    need(type(answer) is str, 'Missing raw reply')
    raw = bytes.fromhex(answer)
    need(answer == raw.hex(), 'Noncanonical raw hex')
    parser = codec.ATParser(); frames = parser.feed(raw)
    need(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes and frames[0].wire == raw,
         'Raw reply must contain exactly one canonical frame')
    decoded = codec.decode_reply(frames[0], mid, parameter)
    need(decoded['ok'] is True, 'Invalid raw reply')
    extras = {'kind', 'sequence', 'request_monotonic_ns', 'monotonic_ns', 'round_trip_ms',
              'raw_request_hex', 'raw_reply_hex'}
    need(set(reply) == set(decoded) | extras, 'Unknown/missing receipt field')
    need(all(type(reply[k]) is type(v) and reply[k] == v for k, v in decoded.items()), 'Raw/decoded mismatch')
    need(reply['kind'] == 'motor_parameter' and type(reply['sequence']) is int and reply['sequence'] >= 1,
         'Invalid receipt label/sequence')
    started, finished = reply['request_monotonic_ns'], reply['monotonic_ns']
    need(type(started) is int and type(finished) is int and 0 < started < finished, 'Invalid sample times')
    need(type(reply['round_trip_ms']) in (float, int) and math.isfinite(reply['round_trip_ms'])
         and reply['round_trip_ms'] == (finished-started)/1e6, 'Invalid receipt RTT')


def describe_samples(report):
    """All complete triplets described; partial rows remain in the report/trace."""
    need(report.get('schema') == SCHEMA, 'Wrong probe schema')
    plan = report['plan']; need(plan == make_plan(plan['motor_id'], plan['samples_requested'], plan['period_ms']), 'Changed probe plan')
    indices, values, differences = set(), [], []
    need(type(report['samples']) is list and len(report['samples']) <= plan['samples_requested'], 'Invalid samples list')
    for row in report['samples']:
        need(type(row) is dict, 'Sample must be an object')
        index = row['slot_index']
        need(type(index) is int and 0 <= index < plan['samples_requested'] and index not in indices, 'Duplicate/invalid slot')
        indices.add(index)
        need(type(row['complete']) is bool, 'Missing sample completion state')
        fields = {'slot_index', 'requested_release_ns', 'begin_ns', 'end_ns', 'complete', 'state'}
        available = [k for k in ('position_before', 'velocity', 'position_after') if k in row]
        need(set(row) == fields | set(available) | ({'error'} if 'error' in row else set()), 'Unknown/missing sample field')
        need(row['state'] == ('COMPLETE_TRIPLET' if row['complete'] else 'INCOMPLETE_TRIPLET'), 'Changed sample state')
        need(type(row['requested_release_ns']) is int and type(row['begin_ns']) is int
             and type(row['end_ns']) is int and 0 < row['requested_release_ns'] <= row['begin_ns'] <= row['end_ns'], 'Invalid slot times')
        for key, parameter in zip(('position_before', 'velocity', 'position_after'), READS):
            if key in row:
                validate_receipt(row[key], plan['motor_id'], parameter)
        if not row['complete']:
            continue
        need(len(available) == 3 and 'error' not in row, 'Incomplete marked complete')
        before, speed, after = [row[k] for k in ('position_before', 'velocity', 'position_after')]
        need(row['begin_ns'] <= before['request_monotonic_ns'] and after['monotonic_ns'] <= row['end_ns']
             and before['monotonic_ns'] <= speed['request_monotonic_ns']
             and speed['monotonic_ns'] <= after['request_monotonic_ns']
             and before['sequence']+1 == speed['sequence'] and speed['sequence']+1 == after['sequence'], 'Unbracketed sample')
        b = (before['request_monotonic_ns']+before['monotonic_ns'])//2
        a = (after['request_monotonic_ns']+after['monotonic_ns'])//2
        differences.append({'slot_index': index, 'position_change_rad': after['value']-before['value'],
            'host_position_midpoint_separation_ns': a-b,
            'position_finite_difference_rad_s_host_midpoints': (after['value']-before['value'])*1e9/(a-b),
            'sensor_sample_time_verified': False, 'velocity_ground_truth': False})
        values.append(speed['value'])
    dropped = set()
    for row in report['dropped_slots']:
        need(type(row) is dict and set(row) == {'slot_index', 'state', 'reason'}, 'Invalid dropped-slot schema')
        index = row['slot_index']
        need(type(index) is int and 0 <= index < plan['samples_requested'] and index not in indices | dropped,
             'Duplicate/invalid dropped slot')
        need(row['state'] == 'DROPPED' and row['reason'] in ('insufficient_segment_budget', 'release_elapsed_before_triplet_start'),
             'Invalid dropped-slot reason')
        dropped.add(index)
    unacquired = sorted(set(range(plan['samples_requested']))-indices)
    need(report['unacquired_slots'] == unacquired, 'Changed unacquired-slot accounting')
    return {'status': ('NO_COMPLETE_SAMPLES' if not values else 'ALL_REQUESTED_TRIPLETS_DESCRIBED'
                      if len(values) == plan['samples_requested'] else 'PARTIAL_TRIPLETS_DESCRIBED'),
            'complete_triplets': len(values), 'reported_velocity_rad_s': _stats(values),
            'requested_triplets': plan['samples_requested'],
            'incomplete_triplets': len(report['samples'])-len(values),
            'dropped_slot_count': len(dropped), 'unacquired_slots': unacquired,
            'full_requested_slot_coverage': len(values) == plan['samples_requested'],
            'host_bracketing_position_differences': differences,
            'all_sample_rows_retained': True, 'incomplete_triplets_excluded_from_statistics_but_retained': True,
            'stationary_gate_applied': False, 'calibration_approved': False, 'output_allowed': False,
            'scope': 'Separate host-time parameter reads, no sensor acquisition timestamps; position differences never replace measured velocity.'}


def collect(cans, expected_uids, plan, *, check=lambda: None, clock=time.monotonic_ns, wait=time.sleep):
    """Caller owns both opened bus contexts and all locks through actual close."""
    need(plan == make_plan(plan['motor_id'], plan['samples_requested'], plan['period_ms']), 'Noncanonical plan')
    expected = timing.validate_uids(expected_uids)
    need(set(cans) == set(IDS) and cans['front'] is not cans['rear'], 'Two distinct bus owners required')
    report = {'schema': SCHEMA, 'status': 'INCOMPLETE', **FLAGS, 'plan': plan, 'errors': [],
              'identities': {}, 'all12_identities_verified': False, 'requests': [], 'samples': [],
              'dropped_slots': [], 'unacquired_slots': [], 'period_deadline_missed_slots': [],
              'triplet_reserve_abort_before_write_count': 0,
              'motor_power_epoch': 'NOT_INFERRED_FROM_JETSON_BOOT', 'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL'}
    deadline = clock()+STARTUP_NS
    def guard():
        check()
        if clock() >= deadline:
            raise TimeoutError('Finite probe deadline exhausted')
    def query(bus, mid, parameter=None):
        guard(); can = cans[bus]; can.deadline_ns = deadline
        try:
            reply = can.query(mid, parameter)
            need(reply.get('ok') is True, 'Rejected parameter reply')
            guard()
            return dict(reply)
        finally:
            if can.last_timing is not None:
                report['requests'].append({'bus': bus, **can.last_timing})
    try:
        for bus, ids in IDS.items():
            for mid in ids:
                reply = query(bus, mid)
                report['identities'][str(mid)] = reply
                need(reply['mcu_uid_hex'] == expected[mid], 'Fresh UID mismatch')
        report['all12_identities_verified'] = True
        begin = clock(); deadline = begin+plan['samples_requested']*plan['period_ms']*1_000_000
        report['segment_begin_ns'], report['segment_deadline_ns'] = begin, deadline
        period, cursor, not_before = plan['period_ms']*1_000_000, 0, begin
        while cursor < plan['samples_requested']:
            target = max(begin+cursor*period, not_before)
            while clock() < target:
                guard(); wait(max(0., min(.01, (target-clock())/1e9)))
            guard(); started = clock()
            slot = max(cursor, (started-begin)//period)
            report['dropped_slots'].extend({'slot_index': i, 'state': 'DROPPED', 'reason': 'release_elapsed_before_triplet_start'}
                                          for i in range(cursor, min(slot, plan['samples_requested'])))
            if slot >= plan['samples_requested']:
                break
            if deadline-started < TRIPLET_RESERVE_NS:
                report['dropped_slots'].extend({'slot_index': i, 'state': 'DROPPED', 'reason': 'insufficient_segment_budget'}
                                              for i in range(slot, plan['samples_requested']))
                break
            row = {'slot_index': slot, 'requested_release_ns': begin+slot*period,
                   'begin_ns': started, 'complete': False, 'state': 'INCOMPLETE_TRIPLET'}
            report['samples'].append(row)
            selected = cans[plan['selected_bus']]; attempts_before = selected.write_attempts
            selected.starting_triplet = True
            try:
                row['position_before'] = query(plan['selected_bus'], plan['motor_id'], 'position')
            except TripletBudgetUnavailable:
                need(selected.write_attempts == attempts_before, 'Reserve abort followed a physical PVP write')
                report['samples'].pop(); report['triplet_reserve_abort_before_write_count'] += 1
                report['dropped_slots'].extend({'slot_index': i, 'state': 'DROPPED', 'reason': 'insufficient_segment_budget'}
                                              for i in range(slot, plan['samples_requested']))
                break
            finally:
                selected.starting_triplet = False
            for name, parameter in (('velocity', 'velocity'), ('position_after', 'position')):
                row[name] = query(plan['selected_bus'], plan['motor_id'], parameter)
            finished = clock(); row.update(end_ns=finished, complete=True, state='COMPLETE_TRIPLET')
            if finished > begin+(slot+1)*period:
                report['period_deadline_missed_slots'].append(slot)
            not_before = max(started+period, finished); cursor = slot+1
        report['segment_end_ns'] = clock()
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
        if report['samples'] and not report['samples'][-1]['complete']:
            report['samples'][-1].update(end_ns=clock(), error=report['errors'][-1])
        for can in cans.values():
            can.poisoned = True
    finally:
        present = {row['slot_index'] for row in report['samples']}
        report['unacquired_slots'] = sorted(set(range(plan['samples_requested']))-present)
        report['write_attempts'] = sum(can.write_attempts for can in cans.values())
        report['queries_sent'] = report['write_attempts']
        report['request_intents_logged'] = sum(can.request_intents_logged for can in cans.values())
        try:
            report['analysis'] = describe_samples(report)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            report['errors'].append('Invalid sample schema: '+str(error))
            report['analysis'] = {'status': 'INVALID_SAMPLE_SCHEMA', 'complete_triplets': 0,
                                  'reported_velocity_rad_s': None, 'output_allowed': False,
                                  'calibration_approved': False, 'stationary_gate_applied': False}
        complete = sum(row['complete'] for row in report['samples'])
        report.update(requested_slots=plan['samples_requested'], complete_triplets=complete,
                      incomplete_triplets=len(report['samples'])-complete,
                      dropped_slot_count=len(report['dropped_slots']), unacquired_slot_count=len(report['unacquired_slots']),
                      full_requested_slot_coverage=complete == plan['samples_requested'])
        accounted = {row['slot_index'] for row in report['samples']} | {row['slot_index'] for row in report['dropped_slots']}
        if (not report['errors'] and complete > 0 and all(r['complete'] for r in report['samples'])
                and accounted == set(range(plan['samples_requested']))):
            report['status'] = 'COMPLETE_READONLY_PROBE_CAPTURE'
        elif complete == 0:
            report['errors'].append('No complete triplets; raw/identity trace is not a successful velocity capture')
    return report


def close_devices(opened):
    """Positive fd closure evidence is necessary before releasing shared leases."""
    errors, all_closed = [], True
    for item in reversed(opened):
        can, fd = item['can'], item['fd']
        can.poisoned = True
        if fd is None and can.serial is not None:
            try:
                fd = can.serial.fileno()
            except BaseException as error:
                errors.append('Cannot identify opened '+item['bus']+' descriptor: '+repr(error))
        try:
            can.__exit__(None, None, None)
        except BaseException as error:
            errors.append('Device close '+item['bus']+': '+repr(error))
        closed = can.serial is None
        if fd is not None:
            try:
                os.fstat(fd)
                closed = False
            except OSError as error:
                closed = error.errno == errno.EBADF
        if not closed:
            all_closed = False
            errors.append('Unconfirmed device descriptor closure: '+item['bus'])
    return all_closed, errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, required=True, dest='motor_id')
    parser.add_argument('--samples', type=int, default=50)
    parser.add_argument('--period-ms', type=int, choices=(20, 200), default=200)
    parser.add_argument('--execute-readonly', action='store_true')
    for name in ('expected-uids', 'expected-boot-id', 'output', 'trace-events'):
        parser.add_argument('--'+name)
    args = parser.parse_args(argv)
    try:
        plan = make_plan(args.motor_id, args.samples, args.period_ms)
    except ValueError as error:
        parser.error(str(error))
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2)); return 0
    if not all((args.expected_uids, args.expected_boot_id, args.output, args.trace_events)):
        parser.error('Execution requires all12 UID file, explicit boot, fresh report and raw JSONL paths')
    try:
        need(str(uuid.UUID(args.expected_boot_id)) == args.expected_boot_id, 'Canonical expected boot UUID required')
        expected, uid_sha = load_uids(args.expected_uids)
        output, trace_path = private_path(args.output), private_path(args.trace_events)
        need(output != trace_path, 'Report and raw trace paths must differ')
        bindings = dual.validate_ports(PORTS['front'], PORTS['rear'])
    except (OSError, ValueError) as error:
        parser.error(str(error))
    cancel = threading.Event(); handlers = {}; trace = report_file = None
    leases, opened, report_sha = ExitStack(), [], None
    result = {'schema': SCHEMA, 'status': 'INCOMPLETE', **FLAGS, 'plan': plan, 'errors': [],
              'samples': [], 'hardware_opened': False, 'hardware_open_attempted': False,
              'expected_boot_id': args.expected_boot_id, 'expected_uids_sha256': uid_sha,
              'source_sha256': source_hashes(), 'started_at': datetime.datetime.now().astimezone().isoformat(),
              'motor_power_epoch': 'NOT_INFERRED_FROM_JETSON_BOOT', 'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
              'all_device_contexts_closed': False}
    try:
        report_file = PrivateFile(output); trace = EventTrace(trace_path)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda *_: cancel.set())
        leases.enter_context(timing.ownership_locks())
        for bus in IDS:
            leases.enter_context(dual.port_lock(bindings[bus]['resolved']))
        boot = dual.BootIdentityGuard(); leases.callback(boot.close)
        result['boot_id'] = boot.boot_id
        need(boot.boot_id == args.expected_boot_id, 'Current boot differs from context pin')
        def check():
            if cancel.is_set():
                raise InterruptedError('Probe interrupted')
            boot.check()
            need(all(dual.binding_matches(b) for b in bindings.values()), 'Pinned bus binding changed')
            for item in opened:
                if item['fd'] is not None:
                    info = os.fstat(item['fd'])
                    need(item['can'].serial.fileno() == item['fd'] and stat.S_ISCHR(info.st_mode)
                         and info.st_rdev == bindings[item['bus']]['st_rdev'], 'Opened descriptor binding changed')
        cans = {}
        for bus in IDS:
            check(); result['hardware_open_attempted'] = True
            can = ProbeCAN(bus, plan, lambda e, b=bus: trace.record(b, e), check=check)
            item = {'bus': bus, 'can': can, 'fd': None}; opened.append(item)
            can.__enter__(); result['hardware_opened'] = True
            item['fd'] = can.serial.fileno(); info = os.fstat(item['fd'])
            need(stat.S_ISCHR(info.st_mode) and info.st_rdev == bindings[bus]['st_rdev'], 'Opened bus differs from pin')
            cans[bus] = can
        result.update(collect(cans, expected, plan, check=check))
        check()
        need(source_hashes() == result['source_sha256'], 'Probe/codec source changed during capture')
    except BaseException as error:
        result['status'] = 'INCOMPLETE'; result['errors'].append(type(error).__name__+': '+str(error))
    finally:
        closed, close_errors = close_devices(opened)
        result['all_device_contexts_closed'] = closed
        if close_errors:
            result['status'] = 'INCOMPLETE'; result['errors'].extend(close_errors)
        result['resource_leases_retained_until_process_exit'] = not closed
        if closed:
            try:
                leases.close()
            except BaseException as error:
                result['status'] = 'INCOMPLETE'; result['errors'].append('Lease close: '+repr(error))
        else:
            _UNCLOSED_DEVICE_LEASES.append((leases, opened))
        for sig, handler in handlers.items():
            try:
                signal.signal(sig, handler)
            except BaseException as error:
                result['status'] = 'INCOMPLETE'; result['errors'].append('Signal restore: '+repr(error))
        if trace is not None:
            result['trace_events'] = trace.close()
            if not result['trace_events']['complete']:
                result['status'] = 'INCOMPLETE'; result['errors'].extend(result['trace_events']['errors'])
        result['completed_at'] = datetime.datetime.now().astimezone().isoformat()
        result['data_collection_status'] = result['status']
        result['status'] = ('RECORDED_REVIEW_REQUIRED' if result['status'] == 'COMPLETE_READONLY_PROBE_CAPTURE' else 'INCOMPLETE')
        result['report_finalization_claimed'] = False
        result['persistence_scope'] = 'This report cannot certify its own finalization. Require the separate final CLI receipt with matching report SHA256.'
        try:
            if report_file is None:
                raise RuntimeError('Report destination was not reserved')
            report_file.report(result)
        except BaseException as error:
            result['status'] = 'INCOMPLETE'; result['errors'].append(type(error).__name__+': '+str(error))
        finally:
            if report_file is not None:
                try:
                    report_file.close()
                except BaseException as error:
                    result['status'] = 'INCOMPLETE'; result['errors'].append(type(error).__name__+': '+str(error))
        if report_file is not None and not result['errors']:
            try:
                report_sha = report_file.finalized_sha256()
            except BaseException as error:
                result['status'] = 'INCOMPLETE'; result['errors'].append(type(error).__name__+': '+str(error))
    success = result['status'] == 'RECORDED_REVIEW_REQUIRED' and report_sha is not None and not result['errors']
    print(json.dumps({'status': 'SAVED_READONLY_PROBE_REVIEW_REQUIRED' if success else 'INCOMPLETE',
                      'output': str(output), 'report_sha256': report_sha, 'errors': result['errors'],
                      'requested_slots': plan['samples_requested'], 'complete_triplets': result.get('complete_triplets', 0),
                      'dropped_slot_count': result.get('dropped_slot_count', 0),
                      'incomplete_triplets': result.get('incomplete_triplets', 0),
                      'unacquired_slot_count': result.get('unacquired_slot_count', plan['samples_requested']),
                      'full_requested_slot_coverage': result.get('full_requested_slot_coverage', False), **FLAGS}))
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
