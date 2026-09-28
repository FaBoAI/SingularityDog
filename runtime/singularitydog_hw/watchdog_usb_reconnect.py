"""Finite, supported zero-gain USB detach/reconnect diagnostic; PLAN by default.

One selected six-axis adapter is unplugged while zero-gain replies are active.
After a confirmed absence, a newly opened channel checks UID, firmware and the
disabled state before any Type4 query, STOP or Enable on that reopened channel.
No packet is sent during the planned absence window; abort cleanup attempts STOP. USB-outage disable time is NOT observable
through the unplugged adapter; the command-loss timing source remains separate.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from . import watchdog_commissioning as loss

SCHEMA = 'singularitydog.watchdog-usb-reconnect.v1'
UNPLUG_WAIT_NS = 15_000_000_000
MIN_ABSENT_NS = 2_000_000_000
RECONNECT_WAIT_NS = 30_000_000_000
TOTAL_NS = 90_000_000_000
RECENT_ACTIVE_NS = 100_000_000
KEEPALIVE_PERIOD_NS = 50_000_000


class USBChannel(loss.Channel):
    """Bounded extra trace capacity for <=15s of six-axis50ms keepalives."""
    def event(self, value):
        need(len(self.events) < 8192, 'USB commissioning event budget exhausted')
        self.events.append(value)


def need(condition, message):
    if not condition:
        raise RuntimeError(message)


def plan(scope):
    need(scope in loss.BUSES, 'Select exactly one of front/rear')
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', 'hardware_opened': False,
        'selected_bus': scope, 'selected_ids': list(loss.BUSES[scope]),
        'kp': 0, 'kd': 0, 'nominal_velocity_reference': 0,
        'nominal_feedforward_reference': 0, 'quantized_zero_bias_present': True,
        'positive_gains_available': False, 'learned_targets_available': False,
        'automatic_retry': False, 'unplug_wait_s': UNPLUG_WAIT_NS/1e9,
        'minimum_observed_absence_s': MIN_ABSENT_NS/1e9,
        'reconnect_wait_s': RECONNECT_WAIT_NS/1e9, 'total_run_limit_s': TOTAL_NS/1e9,
        'usb_outage_disable_timing_verified': False,
        'usb_outage_disable_upper_bound_ms': None, 'approved_for_runtime': False,
        'sequence': ['require successful current-boot/current-power command-loss source',
            'all12 UID/STOP/FW/mode/voltage, selected6 timeout200ms readback',
            'disabled zero-gain query must not enable; announced selected6 enable/zero',
            'keep zero-gain requests current while operator unplugs selected USB only',
            'observe adapter absence for >=2s without transmitting on either bus',
            'reopen same physical USB path; UID, disabled feedback, then FW before ordinary STOP',
            'all12 STOP; separately record operator physical-unplug/power-continuity statement']}


def validate_command_loss(report, expected_uids, boot_id, power_epoch):
    """Validate stored proof; never turn it into a USB test or physical approval."""
    expected = loss.validate_uids(expected_uids)
    need(type(report) is dict and report.get('status') == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC'
         and report.get('group') == 'all' and report.get('stop_confirmed') is True,
         'Completed all12 command-loss source with confirmed STOP required')
    need(report.get('boot_id') == boot_id and report.get('motor_power_epoch') == power_epoch,
         'Command-loss source boot/power epoch differs from this test')
    need(report.get('motor_enable_sent') is True and report.get('positive_gain_sent') is False
         and report.get('learned_targets_sent') is False,
         'Expected zero-gain-only command-loss evidence')
    axes = report.get('axes')
    need(type(axes) is dict and set(axes) == {str(i) for i in loss.IDS}, 'All12 source axes required')
    for mid in loss.IDS:
        axis = axes[str(mid)]
        need(type(axis) is dict and axis.get('uid') == expected[mid]
             and axis.get('command_loss_tested') is True and axis.get('disabled_on_command_loss') is True
             and axis.get('configured_timeout_ms') == 200, f'ID{mid} command-loss source mismatch')
        bound = axis.get('disable_reply_upper_bound_ms')
        need(type(bound) in (int, float) and math.isfinite(bound) and 0 < bound <= 250,
             f'ID{mid} missing bounded command-loss disabled reply')
        need(axis.get('disable_upper_bound_origin') == 'last_zero_host_write_started_ns',
             f'ID{mid} conservative host-start command-loss bound required')
        version = axis.get('version', {}).get('version_bytes_hex')
        need(type(version) is str and len(version) == 8 and all(c in '0123456789abcdef' for c in version),
             f'ID{mid} command-loss firmware bytes missing')
    return expected


def run(channels, expected_uids, command_loss_report, *, scope, boot_id, power_epoch,
        detached, reopen, close_detached, check=lambda: None, announce=lambda: None,
        emit=lambda text: None, clock=time.monotonic_ns, wait=time.sleep):
    """Callbacks own only USB presence/reopening, never CAN traffic or approval.

    ``detached()`` checks that the selected device disappeared and the other
    adapter still matches. ``reopen(deadline_ns)`` waits finitely and returns a
    NEW channel. Both ownership leases remain held for this entire function.
    """
    expected = validate_command_loss(command_loss_report, expected_uids, boot_id, power_epoch)
    selected = loss.BUSES[plan(scope)['selected_bus']]
    need(set(channels) == set(loss.BUSES) and channels['front'] is not channels['rear'],
         'Exactly two independently owned channels required')
    report = {**plan(scope), 'status': 'ABORTED', 'errors': [], 'axes': {},
        'boot_id': boot_id, 'motor_power_epoch': power_epoch, 'hardware_opened': True,
        'motor_enable_sent': False, 'positive_gain_sent': False, 'learned_targets_sent': False,
        'usb_disconnect_observed_by_os': False, 'disabled_after_reconnect_verified': False,
        'physical_usb_unplug_confirmed_by_operator': False, 'motor_power_continuity_confirmed': False,
        'usb_disconnect_test_passed': False, 'stop_confirmed': False,
        'command_loss_disable_upper_bound_ms_by_id': {
            str(i): command_loss_report['axes'][str(i)]['disable_reply_upper_bound_ms'] for i in selected}}
    started = clock()
    old_channel = channels[scope]
    def guard():
        check()
        need(started <= clock() < started+TOTAL_NS, 'USB diagnostic total deadline exceeded')
    def absent():
        guard()
        value = detached()
        need(type(value) is bool, 'USB presence callback must return bool')
        return value
    def owner(mid):
        return channels['front' if mid <= 6 else 'rear']
    active = {}
    last_present = None
    try:
        # All preflight STOPs precede the detach experiment. On reopening, UID,
        # version and zero-gain state probes are the only permitted requests.
        for mid in loss.IDS:
            guard(); channel = owner(mid)
            uid = channel.exchange(mid, 'identity')
            need(uid['mcu_uid_hex'] == expected[mid], f'ID{mid} UID differs')
            stopped = channel.exchange(mid, 'stop')
            need(stopped['mode_state'] == 0 and stopped['fault_bits'] == 0, f'ID{mid} initial STOP missing')
            version = channel.exchange(mid, 'version')
            expected_version = command_loss_report['axes'][str(mid)]['version']['version_bytes_hex']
            need(version['version_bytes_hex'] == expected_version, f'ID{mid} firmware changed since command-loss test')
            need(channel.exchange(mid, 'run_mode')['value'] == 0, f'ID{mid} not MIT mode0')
            voltage = channel.exchange(mid, 'voltage')['value']
            need(type(voltage) in (int, float) and math.isfinite(voltage) and 35 <= voltage <= 42,
                 f'ID{mid} voltage outside35..42V')
            center = stopped['protocol_position_rad']
            need(math.isfinite(center) and -12.57 <= center <= 12.57 and -10 <= stopped['temperature_c'] < 60,
                 f'ID{mid} invalid center/temperature')
            report['axes'][str(mid)] = {'uid': expected[mid], 'version_bytes_hex': expected_version,
                'selected': mid in selected, 'center_rad': center, 'voltage_v': voltage,
                'disabled_after_reconnect': False}
        for mid in selected:
            guard(); channel = owner(mid)
            need(channel.exchange(mid, 'can_timeout')['value'] == 4000,
                 f'ID{mid} current watchdog no longer200ms')
            zero = channel.exchange(mid, 'zero', center=report['axes'][str(mid)]['center_rad'])
            need(zero['mode_state'] == 0 and zero['fault_bits'] == 0,
                 f'ID{mid} disabled zero probe re-enabled motor')
        guard(); announce(); guard()
        for mid in selected:
            guard(); report['motor_enable_sent'] = True
            enabled = owner(mid).exchange(mid, 'enable')
            need(enabled['mode_state'] in (0, 2) and enabled['fault_bits'] == 0, f'ID{mid} enable rejected')
            active[mid] = _active_probe(owner(mid), mid, report['axes'][str(mid)]['center_rad'])
        guard()
        need(not absent(), 'USB absent before unplug instruction')
        last_present = clock()
        report['unplug_window_opened_ns'] = last_present
        emit('USB_UNPLUG_NOW '+scope+'：このUSBだけ抜いてください。40Vと反対バスはそのまま。')
        end = clock()+UNPLUG_WAIT_NS
        while True:
            if absent():
                break
            last_present = clock()
            need(clock() < end, 'No USB disappearance within15s; no automatic retry')
            cycle_start = clock()
            for mid in selected:
                if absent():
                    break
                last_present = clock()
                try:
                    active[mid] = _active_probe(owner(mid), mid, report['axes'][str(mid)]['center_rad'])
                except Exception as error:
                    if not absent():
                        raise
                    report['detach_exchange_error'] = type(error).__name__+': '+str(error)
                    break
            else:
                while clock() < cycle_start+KEEPALIVE_PERIOD_NS:
                    if absent():
                        break
                    last_present = clock()
                    wait(min(.002, max(0., (cycle_start+KEEPALIVE_PERIOD_NS-clock())/1e9)))
                continue
            if absent():
                break
        detected = clock()
        report.update(usb_disconnect_observed_by_os=True, device_absence_detected_ns=detected,
                      last_device_presence_checked_ns=last_present)
        report['last_active_reply_by_id'] = {str(i): active[i] for i in selected}
        need(all(0 <= detected-active[i]['received_ns'] <= RECENT_ACTIVE_NS for i in selected),
             'Not all six active replies were recent at USB disappearance')
        close_detached(old_channel)
        report['detached_channel_closed_ns'] = clock()
        # No calls to either channel during this interval, including STOP.
        while clock() < detected+MIN_ABSENT_NS:
            need(absent(), 'USB reconnected before minimum2s absence')
            wait(min(.01, max(0., (detected+MIN_ABSENT_NS-clock())/1e9)))
        emit('USB_RECONNECT_NOW '+scope+'：同じUSB差込口へ戻してください。40VはOnのまま。')
        report['reconnect_window_opened_ns'] = clock()
        reopened = reopen(min(started+TOTAL_NS, clock()+RECONNECT_WAIT_NS))
        need(reopened is not old_channel and reopened is not channels['rear' if scope == 'front' else 'front'],
             'Reconnection requires a newly owned selected-bus channel')
        channels[scope] = reopened
        guard(); report['new_channel_opened_ns'] = clock()
        report['reconnect_probes'] = []
        # Type4/00c4 firmware queries share the STOP command family. Check
        # disabled state BEFORE even this special query, so its possible stop
        # side effect cannot make a failed USB watchdog look successful.
        for mid in selected:
            guard(); uid = reopened.exchange(mid, 'identity')
            need(uid['mcu_uid_hex'] == expected[mid], f'Reconnected ID{mid} UID mismatch')
            report['reconnect_probes'].append({'motor_id': mid, 'uid': expected[mid]})
        for mid in selected:
            guard(); state = reopened.exchange(mid, 'zero', center=report['axes'][str(mid)]['center_rad'])
            need(state['mode_state'] == 0 and state['fault_bits'] == 0,
                 f'ID{mid} not disabled after reconnect; STOP will now be attempted')
            report['axes'][str(mid)].update(disabled_after_reconnect=True, reconnect_state=state)
        for mid in selected:
            guard(); version = reopened.exchange(mid, 'version')
            need(version['version_bytes_hex'] == report['axes'][str(mid)]['version_bytes_hex'],
                 f'Reconnected ID{mid} firmware mismatch')
            report['reconnect_probes'][selected.index(mid)]['version'] = version
        report['disabled_after_reconnect_verified'] = True
        report['status'] = 'RECORDED_USB_RECONNECT_REVIEW_REQUIRED'
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        report['pre_disconnect_events'] = list(getattr(old_channel, 'events', []))
        stops = {}
        for bus, channel in channels.items():
            try:
                stops[bus] = channel.stop_all()
            except BaseException as error:
                stops[bus] = {'complete': False, 'confirmed_ids': [], 'unconfirmed_ids': list(loss.BUSES[bus]),
                              'errors': [type(error).__name__+': '+str(error)]}
        report['stop_reports'] = stops
        report['stop_confirmed'] = all(v.get('complete') is True and not v.get('unconfirmed_ids')
                                       and not v.get('errors') for v in stops.values())
        if not report['stop_confirmed']:
            report['status'] = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
            report['errors'].append('Keep body supported and physically switch motor power Off')
        report['events_by_bus'] = {bus: list(getattr(channel, 'events', [])) for bus, channel in channels.items()}
        report['elapsed_s'] = (clock()-started)/1e9
    return report


def _active_probe(channel, mid, center):
    fb = channel.exchange(mid, 'zero', center=center)
    need(fb['mode_state'] == 2 and fb['fault_bits'] == 0, f'ID{mid} fresh active zero-gain reply absent')
    need(abs(fb['protocol_position_rad']-center) <= math.radians(3) and
         abs(fb['velocity_rad_s']) <= .5 and -10 <= fb['temperature_c'] < 60,
         f'ID{mid} unexpected zero-gain motion/temperature')
    return fb


def operator_review(report, answer):
    """An explicit post-STOP statement is separate from observed serial state."""
    if report.get('status') != 'RECORDED_USB_RECONNECT_REVIEW_REQUIRED':
        return report
    need(report.get('stop_confirmed') is True and not report.get('errors')
         and report.get('usb_disconnect_observed_by_os') is True
         and report.get('disabled_after_reconnect_verified') is True,
         'Incomplete USB diagnostic cannot accept physical confirmation')
    if answer.strip().lower() != 'yes':
        report['errors'].append('Physical USB-only unplug and uninterrupted40V not confirmed')
        return report
    report.update(physical_usb_unplug_confirmed_by_operator=True,
                  motor_power_continuity_confirmed=True, usb_disconnect_test_passed=True,
                  status='COMPLETE_USB_RECONNECT_DISABLED_DIAGNOSTIC')
    return report


def finite_input(prompt, seconds=30, *, check=lambda: None):
    print(prompt, flush=True)
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        check()
        if select.select([sys.stdin], [], [], min(.1, max(0., end-time.monotonic())))[0]:
            answer = sys.stdin.readline()
            if not answer:
                raise EOFError('Operator input closed')
            return answer.rstrip('\r\n')
    raise TimeoutError('Operator input timed out after'+str(seconds)+'s')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bus', choices=loss.BUSES, required=True)
    p.add_argument('--expected-uids', required=True)
    p.add_argument('--command-loss-report')
    p.add_argument('--execute-supported-zero-gain', action='store_true')
    for name in ('support-in-place', 'cutoff-ready', 'rs05-model-confirmed'):
        p.add_argument('--'+name, action='store_true')
    for name in ('front-port', 'rear-port', 'power-epoch', 'output', 'audio', 'audio-sha256', 'audio-device'):
        p.add_argument('--'+name)
    a = p.parse_args(argv)
    uid_raw = Path(a.expected_uids).read_bytes()
    expected = loss.validate_uids(json.loads(uid_raw))
    if not a.execute_supported_zero_gain:
        print(json.dumps(plan(a.bus), ensure_ascii=False, indent=2)); return 0
    if not a.command_loss_report:
        p.error('Successful command-loss report required before USB test')
    source_raw = Path(a.command_loss_report).read_bytes()
    source = json.loads(source_raw)
    if not all((a.support_in_place, a.cutoff_ready, a.rs05_model_confirmed)) or not all(
            getattr(a, k) for k in ('front_port', 'rear_port', 'power_epoch', 'output',
                                    'audio', 'audio_sha256', 'audio_device')):
        p.error('Explicit support/cutoff/model, ports, epoch, fresh output and pinned audio required')
    audio = Path(a.audio).resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest() != a.audio_sha256:
        p.error('Announcement SHA256 differs')
    from .i2s_announcement import validate_announcement
    validate_announcement(audio)
    out = Path(a.output).expanduser().resolve()
    if any((folder/'.git').exists() for folder in (out, *out.parents)):
        p.error('Keep raw device logs outside Git')
    out.mkdir(parents=True, mode=0o700, exist_ok=False)
    from . import dual_can_pipeline_benchmark as dual
    from .sensor_pipeline_benchmark import BootIdentityGuard
    report = {'schema': SCHEMA, 'status': 'ABORTED_BEFORE_ENABLE', 'errors': [], 'motor_enable_sent': False,
              'usb_disconnect_test_passed': False, 'usb_outage_disable_timing_verified': False}
    cancelled = []; handlers = {}; channels = {}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.append(number))
        with ExitStack() as stack:
            boot = BootIdentityGuard(); stack.callback(boot.close)
            def check():
                if cancelled:
                    raise InterruptedError('Operator interrupted USB diagnostic')
                boot.check()
            validate_command_loss(source, expected, boot.boot_id, a.power_epoch)
            answer = finite_input('箱を残し4足接地・手離し・即時40V Off。'+a.bus+
                ' USBだけに届く状態で、合図まで抜かず待てますか [Enter開始 / q中止]:', check=check)
            if answer.strip():
                raise InterruptedError('USB diagnostic cancelled before setup')
            stack.enter_context(dual.pipeline.ownership_locks())
            bindings = dual.validate_ports(a.front_port, a.rear_port)
            import serial
            def open_channel(bus, binding):
                stack.enter_context(dual.port_lock(binding['resolved']))
                port = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.02, exclusive=True)
                port.dtr = port.rts = False; port.port = binding['path']
                port.open(); stack.callback(port.close)
                need(dual.binding_matches(binding) and os.fstat(port.fileno()).st_rdev == binding['st_rdev'],
                     'USB binding changed while opening')
                return USBChannel(port, loss.BUSES[bus], check=check)
            for bus, binding in bindings.items():
                channels[bus] = open_channel(bus, binding)
            other = 'rear' if a.bus == 'front' else 'front'
            def detached():
                check()
                need(dual.binding_matches(bindings[other]), 'Unselected USB changed/disappeared')
                try:
                    need(dual.binding_matches(bindings[a.bus]), 'Selected USB rebound without observed absence')
                    return False
                except FileNotFoundError:
                    return True
            def reopen(deadline):
                # The original per-port lock stays held. Reusing the same tty
                # path requires no second flock of the same lock file.
                while time.monotonic_ns() < deadline:
                    check()
                    need(dual.binding_matches(bindings[other]), 'Unselected USB changed during reconnect')
                    if Path(bindings[a.bus]['path']).exists():
                        new = dual.validate_ports(a.front_port, a.rear_port)[a.bus]
                        port = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.02, exclusive=True)
                        port.dtr = port.rts = False; port.port = new['path']
                        if new['resolved'] != bindings[a.bus]['resolved']:
                            stack.enter_context(dual.port_lock(new['resolved']))
                        port.open(); stack.callback(port.close)
                        need(dual.binding_matches(new) and os.fstat(port.fileno()).st_rdev == new['st_rdev'],
                             'Reopened USB binding differs')
                        return USBChannel(port, loss.BUSES[a.bus], check=check)
                    time.sleep(.02)
                raise TimeoutError('Selected USB did not reconnect within30s')
            def announce():
                subprocess.run(['aplay', '-D', a.audio_device, str(audio)], check=True, timeout=8,
                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            report = run(channels, expected, source, scope=a.bus, boot_id=boot.boot_id,
                power_epoch=a.power_epoch, detached=detached, reopen=reopen,
                close_detached=lambda channel: channel.port.close(), check=check, announce=announce,
                emit=lambda message: print(message, flush=True))
            if report['status'] == 'RECORDED_USB_RECONNECT_REVIEW_REQUIRED':
                # All STOP acknowledgements precede this finite operator wait.
                answer = finite_input('全12軸STOP確認済み。このUSBケーブルだけを実際に抜き差しし、'
                    '試験中40VはOnを維持しましたか [yes / それ以外=未確認]:', check=check)
                operator_review(report, answer)
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        report.update(command_loss_source_sha256=hashlib.sha256(source_raw).hexdigest(),
                      expected_uids_sha256=hashlib.sha256(uid_raw).hexdigest())
        for sig, old in handlers.items():
            signal.signal(sig, old)
        with os.fdopen(os.open(out/'report.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'w') as f:
            json.dump(report, f, ensure_ascii=False, allow_nan=False); f.write('\n')
    print(json.dumps({key: report.get(key) for key in ('status', 'errors', 'motor_enable_sent',
        'stop_confirmed', 'usb_disconnect_test_passed', 'usb_outage_disable_timing_verified')}, ensure_ascii=False))
    return 0 if report['status'] == 'COMPLETE_USB_RECONNECT_DISABLED_DIAGNOSTIC' else 2


if __name__ == '__main__':
    raise SystemExit(main())
