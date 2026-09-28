"""Supported, zero-gain RS05 command-loss commissioning; default is PLAN_ONLY.

This independent entry point does not require an approved learned-output profile.
It sends no learned target, positive gain, flash-save or firmware-update command.
Nominal zero references still have uint16 quantization bias. Mechanical support
and a person at the physical cutoff remain necessary. A command-loss result is
NOT a physical USB-unplug test or proof that a loaded robot cannot fall.
"""
import argparse
from contextlib import ExitStack
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from . import can_readonly as codec
from . import rs05_trial_protocol as protocol
from .motor_version_probe import VERSION_PREFIX, decode_version, version_request, validate_uids

IDS = tuple(range(1, 13))
BUSES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
GROUPS = {'calf': (1, 4, 7, 10), 'thigh': (2, 5, 8, 11),
          'hip': (3, 6, 9, 12), 'all': IDS}
SILENCE_NS = 210_000_000
MAX_DISABLE_UPPER_BOUND_NS = 250_000_000
REQUEST_NS = 250_000_000
TOTAL_NS = 25_000_000_000


def plan(group='all'):
    if group not in GROUPS:
        raise ValueError('Choose one fixed joint group or all twelve axes')
    return {'status': 'PLAN_ONLY', 'hardware_opened': False, 'group': group,
            'selected_ids': list(GROUPS[group]), 'kp': 0, 'kd': 0,
            'nominal_velocity_reference': 0, 'nominal_feedforward_reference': 0,
            'quantized_zero_bias_present': True, 'watchdog_ticks': 4000,
            'configured_timeout_ms': 200, 'silent_interval_ms': 210,
            'accepted_disable_upper_bound_ms': 250, 'automatic_retry': False,
            'learned_targets_available': False, 'positive_gains_available': False,
            'usb_disconnect_tested': False, 'approved_for_runtime': False,
            'sequence': ['identity/STOP/version/mode/voltage for all12',
                         '200ms watchdog write and readback for selected group',
                         'zero-gain frame while disabled: must stay disabled',
                         'one announced run; each selected axis: enable and verify zero-gain reply',
                         'per axis: no CAN transmission for at least210ms',
                         'per axis: zero-gain probe must report disabled within250ms upper bound',
                         'STOP every axis; preserve ambiguous acknowledgements']}


def _need(condition, message):
    if not condition:
        raise RuntimeError(message)


class Channel:
    """One bus owner, exact outgoing allowlist, finite requests, no retries.

    Recording is in memory until STOP; this is a commissioning tool, not a
    20ms control loop. A timed-out mode command cannot provide a causal STOP
    acknowledgement later: cleanup records it as unconfirmed.
    """
    def __init__(self, port, ids, *, clock=time.monotonic_ns, check=lambda: None, reader=None):
        from .serial_deadline_reader import DeadlineSerialReader
        _need(tuple(ids) in BUSES.values(), 'Channel must own exactly one fixed six-axis bus')
        self.port, self.ids, self.clock, self.check = port, tuple(ids), clock, check
        self.parser = codec.ATParser()
        self.pending = {}
        self.events = []
        self.failed = False
        self.reader = reader or DeadlineSerialReader(port, clock=clock, check=check)

    def event(self, value):
        _need(len(self.events) < 2048, 'Commissioning event budget exhausted')
        self.events.append(value)

    def _wire(self, mid, step, center=0.):
        _need(type(mid) is int and mid in self.ids, 'Cross-bus motor request rejected')
        if step in ('identity', 'run_mode', 'voltage', 'can_timeout'):
            return codec.read_request(mid, None if step == 'identity' else step)
        if step == 'stop':
            return protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=mid)
        if step == 'version':
            return version_request(mid)
        if step == 'watchdog_write':
            return protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid)
        if step == 'enable':
            return protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=mid)
        if step == 'zero':
            return protocol.motion_request(phase=protocol.TrialPhase.ZERO_GAIN, center_rad=center, motor_id=mid)
        raise ValueError('Unknown commissioning step; arbitrary commands are unavailable')

    def _read(self, wake, hard):
        try:
            raw, received = self.reader.read_until(wake, hard)
        except BaseException as error:
            rejected = getattr(error, 'serial_read_evidence', None)
            if rejected is not None:
                try:
                    self.event({'kind': 'rx_rejected', **rejected.record(),
                                'error': type(error).__name__+': '+str(error)})
                except BaseException:
                    self.unlogged_receive_failure = True
            raise
        if not raw:
            return [], received
        self.event({'kind': 'rx_bytes', 'received_ns': received, 'hex': raw.hex()})
        frames = self.parser.feed(raw)
        _need(not self.parser.discarded_bytes, 'Malformed serial bytes')
        for frame in frames:
            _need(frame.flags == 4 and len(frame.data) == 8, 'Noncanonical CAN reply')
            _need(not (frame.kind == 21 and frame.source in self.ids), 'Motor fault-detail reply')
            if frame.kind == 2 and frame.source in self.ids and frame.data[:3] != VERSION_PREFIX:
                fb = protocol.decode_type2(frame, motor_id=frame.source)
                _need(fb.fault_bits == 0, 'Motor fault in feedback')
        return frames, received

    def _boundary(self):
        # Outstanding replies may never be silently discarded and reused.
        _need(not self.pending and not self.parser.buffer and not self.port.in_waiting,
              'No fresh request boundary: pending/partial/backlogged reply')

    def _send(self, mid, step, center=0.):
        self.check()
        wire = self._wire(mid, step, center)
        start = self.clock()
        self.pending[mid] = step
        returned = None
        try:
            returned = self.port.write(wire)
        finally:
            finish = self.clock()
            self.event({'kind': 'tx', 'motor_id': mid, 'step': step, 'hex': wire.hex(),
                        'start_ns': start, 'finish_ns': finish, 'returned_bytes': returned})
        _need(returned == len(wire) and type(returned) is int, 'Partial write; no retry')
        return start, finish

    def exchange(self, mid, step, *, center=0.):
        _need(not self.failed, 'Failed channel cannot continue commissioning')
        try:
            self._boundary()
            start, finish = self._send(mid, step, center)
            deadline = start + REQUEST_NS
            while self.clock() < deadline:
                frames, received = self._read(deadline, deadline)
                found = None
                for frame in frames:
                    if step in ('identity', 'run_mode', 'voltage', 'can_timeout'):
                        parameter = None if step == 'identity' else step
                        if not codec.matches(frame, mid, parameter):
                            raise RuntimeError('Unexpected reply during single outstanding request')
                        value = codec.decode_reply(frame, mid, parameter)
                        _need(value['ok'], 'Rejected parameter')
                    elif step == 'version':
                        value = decode_version(frame, mid)
                    else:
                        fb = protocol.decode_type2(frame, motor_id=mid)
                        _need(fb.fault_bits == 0, 'Fault in state reply')
                        if step == 'watchdog_write':
                            # Current RS05 firmware emits one Type2 state
                            # response to this Type18 write. Consume it before
                            # the separate Type17 readback. A state reply alone
                            # does NOT prove the parameter was stored.
                            _need(fb.mode_state == 0, 'Watchdog setup must remain disabled')
                        value = asdict(fb)
                    _need(found is None, 'Duplicate matching reply')
                    found = value
                if found is not None:
                    _need(not self.parser.buffer, 'Partial residual reply')
                    del self.pending[mid]
                    return {**found, 'request_start_ns': start, 'write_finish_ns': finish,
                            'received_ns': received}
            raise TimeoutError('No reply within fixed commissioning deadline')
        except BaseException:
            self.failed = True
            raise

    def stop_all(self):
        """Attempt all six STOPs even after cancellation, parser or write error."""
        from .serial_deadline_reader import DeadlineSerialReader
        ambiguous = {mid for mid, step in self.pending.items()
                     if step in ('zero', 'enable', 'stop', 'watchdog_write')}
        self.pending.clear()
        # A broken receive path must not prevent six physical STOP attempts.
        errors, sent, confirmed = [], {}, set()
        boundary_ok = False
        try:
            boundary_ok = not self.parser.buffer and not self.port.in_waiting
        except BaseException as error:
            errors.append('STOP boundary: '+type(error).__name__+': '+str(error))
        self.parser = codec.ATParser()
        old_check, old_reader = self.check, self.reader
        self.check = lambda: None
        acknowledged = set()
        def consume(frames, received):
            for frame in frames:
                if frame.kind != 2 or frame.source not in sent or frame.data[:3] == VERSION_PREFIX:
                    continue
                fb = protocol.decode_type2(frame, motor_id=frame.source)
                if received > sent[frame.source] and fb.mode_state == 0 and fb.fault_bits == 0:
                    _need(frame.source not in acknowledged, 'Duplicate STOP acknowledgement')
                    acknowledged.add(frame.source)
                    if boundary_ok and frame.source not in ambiguous:
                        confirmed.add(frame.source)
        try:
            reader_ready = False
            try:
                self.reader = DeadlineSerialReader(self.port, clock=self.clock, check=lambda: None)
                reader_ready = True
            except BaseException as error:
                errors.append('STOP reader: '+type(error).__name__+': '+str(error))
            last_stop_finish = None
            for mid in self.ids:
                try:
                    if last_stop_finish is not None:
                        gap_end = last_stop_finish+800_000
                        while self.clock() < gap_end:
                            time.sleep(min(.0008, max(0., (gap_end-self.clock())/1e9)))
                    start, finish = self._send(mid, 'stop')
                    sent[mid] = finish
                    last_stop_finish = finish
                except BaseException as error:
                    errors.append(f'ID{mid}: {type(error).__name__}: {error}')
                if mid in sent and reader_ready:
                    # Cleanup has no 20ms budget. Await this axis before the
                    # next STOP, instead of overfilling the serial adapter with
                    # a batch. Timeout or receive failure still proceeds to
                    # every remaining physical STOP exactly once.
                    axis_deadline = self.clock()+REQUEST_NS
                    try:
                        while self.clock() < axis_deadline and mid not in acknowledged:
                            frames, received = self._read(axis_deadline, axis_deadline)
                            consume(frames, received)
                    except BaseException as error:
                        errors.append(f'ID{mid} STOP wait: '+type(error).__name__+': '+str(error))
            deadline = self.clock()+REQUEST_NS
            while reader_ready and self.clock() < deadline and len(confirmed) < len(self.ids):
                try:
                    frames, received = self._read(deadline, deadline)
                    consume(frames, received)
                except BaseException as error:
                    errors.append(type(error).__name__+': '+str(error)); break
            try:
                _need(not self.parser.buffer and not self.port.in_waiting, 'STOP has trailing/partial/backlogged bytes')
            except BaseException as error:
                errors.append(type(error).__name__+': '+str(error))
        except BaseException as error:
            errors.append(type(error).__name__+': '+str(error))
        finally:
            self.check, self.reader = old_check, old_reader
            self.pending.clear()
        return {'confirmed_ids': sorted(confirmed), 'unconfirmed_ids': sorted(set(self.ids)-confirmed),
                'ambiguous_ids': sorted(ambiguous), 'errors': errors,
                'complete': boundary_ok and not errors and len(confirmed)==len(self.ids)}


def run(channels, expected_uids, *, group='all', check=lambda: None,
        clock=time.monotonic_ns, wait=time.sleep, announce=lambda: None):
    """Independent finite command-loss experiment; channels may be fake in tests."""
    expected = validate_uids(expected_uids)
    selected = GROUPS[plan(group)['group']]
    _need(set(channels) == set(BUSES), 'Exactly two independently owned buses required')
    _need(channels['front'] is not channels['rear'], 'Shared transport rejected')
    report = {**plan(group), 'status': 'ABORTED', 'errors': [], 'axes': {},
              'motor_enable_sent': False, 'positive_gain_sent': False,
              'learned_targets_sent': False, 'usb_disconnect_tested': False,
              'stop_confirmed': False, 'approved_for_runtime': False}
    started = clock()
    def guard():
        check()
        _need(started <= clock() < started+TOTAL_NS, 'Commissioning total deadline exceeded')
    def owner(mid):
        return channels['front' if mid <= 6 else 'rear']
    try:
        # No enable occurs until every expected actuator has been checked.
        for mid in IDS:
            guard(); channel = owner(mid)
            uid = channel.exchange(mid, 'identity')
            _need(uid['mcu_uid_hex'] == expected[mid], f'ID{mid} identity mismatch')
            stopped = channel.exchange(mid, 'stop')
            _need(stopped['mode_state'] == 0 and stopped['fault_bits'] == 0, 'Initial STOP unconfirmed')
            version = channel.exchange(mid, 'version')
            mode = channel.exchange(mid, 'run_mode')['value']
            volts = channel.exchange(mid, 'voltage')['value']
            _need(mode == 0, 'MIT run_mode0 required')
            _need(type(volts) in (int, float) and math.isfinite(volts) and 35 <= volts <= 42,
                  'Voltage outside supported commissioning range35..42V')
            _need(math.isfinite(stopped['protocol_position_rad']) and
                  -12.57 <= stopped['protocol_position_rad'] <= 12.57,
                  'Invalid Type2 center')
            _need(-10 <= stopped['temperature_c'] < 60, 'Temperature outside commissioning range')
            report['axes'][str(mid)] = {'uid': expected[mid], 'version': version,
                'center_rad': stopped['protocol_position_rad'], 'voltage_v': volts,
                'selected': mid in selected, 'command_loss_tested': False,
                'usb_disconnect_tested': False, 'disabled_on_command_loss': False}
        for mid in selected:
            guard(); channel = owner(mid)
            setup = channel.exchange(mid, 'watchdog_write')
            readback = channel.exchange(mid, 'can_timeout')
            _need(readback['value'] == 4000, 'Watchdog readback mismatch')
            report['axes'][str(mid)].update(watchdog_setup_state=setup, watchdog_readback=readback)
            fb = channel.exchange(mid, 'zero', center=report['axes'][str(mid)]['center_rad'])
            _need(fb['mode_state'] == 0 and fb['fault_bits'] == 0,
                  'Zero-gain query re-enabled a disabled actuator; probe method is invalid')
        guard(); announce(); guard()
        for mid in selected:
            guard(); axis = report['axes'][str(mid)]
            # One axis per silence interval, all in one invocation. This avoids
            # treating sequential bus delays as a device timeout measurement.
            report['motor_enable_sent'] = True  # Before an ambiguous write.
            enabled = owner(mid).exchange(mid, 'enable')
            _need(enabled['mode_state'] in (0, 2) and enabled['fault_bits'] == 0, 'Enable rejected')
            fb = owner(mid).exchange(mid, 'zero', center=axis['center_rad'])
            _need(fb['mode_state'] == 2 and fb['fault_bits'] == 0, 'Zero-gain active reply not confirmed')
            _need(abs(fb['protocol_position_rad']-axis['center_rad']) <= math.radians(3) and
                  abs(fb['velocity_rad_s']) <= .5 and -10 <= fb['temperature_c'] < 60,
                  'Unexpected motion/temperature in zero-gain commissioning')
            axis['last_zero_write_finish_ns'] = fb['write_finish_ns']
            axis['last_zero_write_start_ns'] = fb['request_start_ns']
            # No request on either bus, including watchdog-resetting polls.
            silent_until = clock()+SILENCE_NS
            while clock() < silent_until:
                guard(); wait(min(.002, max(0., (silent_until-clock())/1e9)))
            fb = owner(mid).exchange(mid, 'zero', center=axis['center_rad'])
            # The adapter may transmit before write() returns. The entry time
            # gives a conservative host-side bound without subtracting any
            # unmeasured time that could already count toward device expiry.
            elapsed = fb['received_ns']-axis['last_zero_write_start_ns']
            _need(SILENCE_NS <= elapsed <= MAX_DISABLE_UPPER_BOUND_NS,
                  f'ID{mid} stopped-state reply exceeded250ms conservative upper bound')
            _need(fb['mode_state'] == 0 and fb['fault_bits'] == 0,
                  f'ID{mid} did not report disabled after command silence')
            axis.update(command_loss_tested=True, disabled_on_command_loss=True,
                        disable_reply_upper_bound_ms=elapsed/1e6,
                        disable_upper_bound_origin='last_zero_host_write_started_ns',
                        stop_probe=fb, configured_timeout_ms=200)
        report['status'] = 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC'
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        stops = {}
        for scope, channel in channels.items():
            try:
                stops[scope] = channel.stop_all()
            except BaseException as error:
                stops[scope] = {'confirmed_ids': [], 'unconfirmed_ids': list(BUSES[scope]),
                                'errors': [type(error).__name__+': '+str(error)]}
        report['stop_reports'] = stops
        report['stop_confirmed'] = all(item.get('complete') is True and not item['unconfirmed_ids']
                                       and not item['errors'] for item in stops.values())
        if not report['stop_confirmed']:
            report['status'] = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
            report['errors'].append('Physically switch motor power Off; STOP acknowledgement incomplete')
        report['hardware_opened'] = True
        report['elapsed_s'] = (clock()-started)/1e9
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expected-uids', required=True)
    p.add_argument('--group', choices=GROUPS, default='all')
    p.add_argument('--execute-supported-zero-gain', action='store_true')
    for name in ('support-in-place', 'cutoff-ready', 'rs05-model-confirmed'):
        p.add_argument('--'+name, action='store_true')
    for name in ('front-port', 'rear-port', 'power-epoch', 'output', 'audio', 'audio-sha256', 'audio-device'):
        p.add_argument('--'+name)
    a = p.parse_args(argv)
    source = Path(a.expected_uids).read_bytes()
    expected = validate_uids(json.loads(source))
    if not a.execute_supported_zero_gain:
        print(json.dumps(plan(a.group), ensure_ascii=False, indent=2)); return 0
    if not all((a.support_in_place, a.cutoff_ready, a.rs05_model_confirmed)):
        p.error('Current mechanical support, physical cutoff and actual RS05 model confirmation required')
    if not all(getattr(a, key) for key in ('front_port', 'rear_port', 'power_epoch', 'output',
                                          'audio', 'audio_sha256', 'audio_device')):
        p.error('Explicit ports, motor-power epoch, new private output and pinned audio required')
    audio = Path(a.audio).resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest() != a.audio_sha256:
        p.error('Announcement file hash mismatch')
    out = Path(a.output).expanduser().resolve()
    if any((ancestor/'.git').exists() for ancestor in (out, *out.parents)):
        p.error('Raw logs must be outside Git')
    out.mkdir(parents=True, mode=0o700, exist_ok=False)
    from . import dual_can_pipeline_benchmark as dual
    from .sensor_pipeline_benchmark import BootIdentityGuard
    cancelled = []
    handlers = {}
    report = {'status': 'ABORTED_BEFORE_ENABLE', 'errors': [], 'motor_enable_sent': False}
    channels = {}
    try:
        bindings = dual.validate_ports(a.front_port, a.rear_port)
        answer = input('支持台は残し、脚から手を離し、即時40V Offを確認。'
                       'ゼロゲイン通信断診断を一括実行するならEnter、中止はq: ')
        if answer.strip():
            raise InterruptedError('Operator did not confirm commissioning start')
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.append(number))
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks())
            boot = BootIdentityGuard(); stack.callback(boot.close)
            def check():
                if cancelled:
                    raise InterruptedError('Operator interrupted commissioning')
                boot.check()
            import serial
            for scope, binding in bindings.items():
                stack.enter_context(dual.port_lock(binding['resolved']))
                port = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.02, exclusive=True)
                port.dtr = port.rts = False
                port.port = binding['path']; port.open(); stack.callback(port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev != binding['st_rdev']:
                    raise RuntimeError('USB identity changed')
                channels[scope] = Channel(port, BUSES[scope], check=check)
            def announce():
                check()
                subprocess.run(['aplay', '-D', a.audio_device, str(audio)], check=True, timeout=8.,
                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            report = run(channels, expected, group=a.group, check=check, announce=announce)
            report.update(boot_id=boot.boot_id, motor_power_epoch=a.power_epoch,
                          expected_uids_sha256=hashlib.sha256(source).hexdigest())
            report['events_by_bus'] = {scope: channel.events for scope, channel in channels.items()}
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        with os.fdopen(os.open(out/'report.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'w') as f:
            json.dump(report, f, ensure_ascii=False, allow_nan=False); f.write('\n')
    print(json.dumps({key: report.get(key) for key in ('status', 'errors', 'motor_enable_sent',
                                                       'stop_confirmed', 'usb_disconnect_tested')}, ensure_ascii=False))
    return 0 if report['status'] == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' else 2


if __name__ == '__main__':
    raise SystemExit(main())
