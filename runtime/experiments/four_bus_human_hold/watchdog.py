"""Four independent three-axis owners for the existing zero-gain watchdog protocol.

No positive-gain or learned command exists in this module. The original strict
Channel request/parser/deadline/STOP ambiguity rules are inherited unchanged;
only its constructor is scoped to one fixed three-axis owner. A global CAN
silence interval is measured one actuator at a time, never parallelized.
"""
import copy
import math
import time

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw import watchdog_commissioning as original
from singularitydog_hw.motor_version_probe import validate_uids
from . import profile as HP

IDS = HP.IDS
PORTS = HP.PORTS
BUS_MEMBERS = tuple(tuple(range(first, first+3)) for first in (1, 4, 7, 10))
SILENCE_NS = original.SILENCE_NS
MAX_DISABLE_UPPER_BOUND_NS = original.MAX_DISABLE_UPPER_BOUND_NS
TOTAL_NS = original.TOTAL_NS
_need = original._need
_error_text = original._error_text


class Channel(original.Channel):
    """Original exact request/read/stop engine, restricted to one three-axis port."""
    def __init__(self, port, ids, *, clock=time.monotonic_ns, check=lambda: None, reader=None):
        from singularitydog_hw.serial_deadline_reader import DeadlineSerialReader
        _need(tuple(ids) in BUS_MEMBERS, 'Exactly one fixed three-axis owner required')
        self.port, self.ids, self.clock, self.check = port, tuple(ids), clock, check
        self.parser = codec.ATParser()
        self.pending = {}
        self.events = []
        self.failed = False
        self._ambiguous_ids = set()
        self._unresolved_state_steps = {}
        self._stop_boundary_uncertain = False
        self.reader = reader or DeadlineSerialReader(port, clock=clock, check=check)

    def exchange(self, mid, step, *, center=0.):
        begin = len(self.events)
        result = super().exchange(mid, step, center=center)
        incoming = b''.join(bytes.fromhex(row['hex']) for row in self.events[begin:]
                            if row.get('kind') == 'rx_bytes')
        _need(len(incoming) == 17, 'Exchange returned without its one original reply frame')
        return {**result, 'original_reply_hex': incoming.hex()}


def plan(contract):
    HP.validate_contract(copy.deepcopy(contract))
    base = contract['base_numerical_contract']
    return {'schema': 'singularitydog.four-can-watchdog-plan.v1', 'status': 'PLAN_ONLY',
        'hardware_opened': False, 'opens_devices': False, 'selected_ids': list(IDS),
        'topology_by_port': copy.deepcopy(base['topology_by_port']),
        'boot_id': base['boot_id'], 'motor_power_epoch': base['motor_power_epoch'],
        'source_manifest_sha256': base['source_manifest']['sha256'],
        'configured_timeout_ms': 200, 'watchdog_ticks': 4000,
        'silent_interval_ms': 210, 'accepted_disable_upper_bound_ms': 250,
        'voltage_range_v': [35, 42], 'kp': 0, 'kd': 0,
        'automatic_retry': False, 'usb_disconnect_tested': False,
        'positive_gain_sent': False, 'learned_targets_sent': False,
        'approved_for_runtime': False, 'output_allowed': False,
        'all_CAN_owners_silent_during_each_probe': True,
        'existing_two_bus_watchdog_changed': False}


def _checked_feedback(axis, mid, phase, reply, profile, *, mode):
    """Keep a received observation before applying the unchanged human caps."""
    axis.setdefault('zero_gain_observations', []).append(dict(
        motor_id=mid, phase=phase, reply=copy.deepcopy(reply),
        position_delta_rad=reply['protocol_position_rad']-axis['center_rad'],
        original_reply_hex=reply['original_reply_hex']))
    return HP._feedback(reply['original_reply_hex'], mid, mode=mode, profile=profile)


def run(channels, contract, firmware_by_id, *, check=lambda: None,
        clock=time.monotonic_ns, wait=time.sleep, announce=lambda: None, execute=False):
    """Default is file-only PLAN. The dedicated foreground owns all actual capabilities."""
    planned = plan(contract)
    if execute is not True:
        return planned
    _need(type(execute) is bool, 'Explicit execution boolean required')
    base, profile = contract['base_numerical_contract'], contract['runtime_profile']
    expected = validate_uids(base['uids_by_id'])
    _need(set(channels) == set(PORTS) and len({id(ch) for ch in channels.values()}) == 4,
          'Four independently owned channels required')
    _need(all(type(channels[port]) is Channel and tuple(channels[port].ids) == tuple(ids)
              for port, ids in base['topology_by_port'].items()), 'Exact current three-axis channel owners required')
    _need(type(firmware_by_id) is dict and set(firmware_by_id) == {str(mid) for mid in IDS},
          'All twelve independently reviewed firmware bytes required')
    report = {**planned, 'status': 'ABORTED', 'errors': [], 'axes': {},
              'motor_enable_sent': False, 'stop_confirmed': False}
    started = clock()
    owners = {mid: channels[port] for port, ids in base['topology_by_port'].items() for mid in ids}
    def guard():
        check()
        _need(started <= clock() < started+TOTAL_NS, 'Commissioning total deadline exceeded')
    def owner(mid):
        return owners[mid]
    try:
        for mid in IDS:
            guard(); channel = owner(mid)
            uid = channel.exchange(mid, 'identity')
            _need(uid['mcu_uid_hex'] == expected[mid], f'ID{mid} identity mismatch')
            stopped = channel.exchange(mid, 'stop')
            HP._feedback(stopped['original_reply_hex'], mid, mode=0, profile=profile)
            version = channel.exchange(mid, 'version')
            _need(version['version_bytes_hex'] == firmware_by_id[str(mid)], f'ID{mid} reviewed firmware changed')
            mode = channel.exchange(mid, 'run_mode')['value']
            volts = channel.exchange(mid, 'voltage')['value']
            _need(mode == 0, 'MIT run_mode0 required')
            _need(type(volts) in (int, float) and math.isfinite(volts) and 35 <= volts <= 42,
                  f'ID{mid} voltage outside supported35..42V')
            report['axes'][str(mid)] = {'uid': expected[mid], 'version': version,
                'center_rad': stopped['protocol_position_rad'], 'voltage_v': volts,
                'selected': True, 'command_loss_tested': False,
                'usb_disconnect_tested': False, 'disabled_on_command_loss': False}
        for mid in IDS:
            guard(); channel, axis = owner(mid), report['axes'][str(mid)]
            setup = channel.exchange(mid, 'watchdog_write')
            HP._feedback(setup['original_reply_hex'], mid, mode=0, profile=profile)
            readback = channel.exchange(mid, 'can_timeout')
            _need(readback['value'] == 4000, 'Watchdog readback mismatch')
            axis.update(watchdog_setup_state=setup, watchdog_readback=readback)
            fb = channel.exchange(mid, 'zero', center=axis['center_rad'])
            _checked_feedback(axis, mid, 'disabled_zero_before_enable', fb, profile, mode=0)
        guard(); announce(); guard()
        for mid in IDS:
            guard(); axis = report['axes'][str(mid)]
            report['motor_enable_sent'] = True  # Retain before a possibly partial/ambiguous write.
            enabled = owner(mid).exchange(mid, 'enable')
            _need(enabled['mode_state'] in (0, 2), 'Enable state rejected')
            HP._feedback(enabled['original_reply_hex'], mid, mode=enabled['mode_state'], profile=profile)
            fb = owner(mid).exchange(mid, 'zero', center=axis['center_rad'])
            _checked_feedback(axis, mid, 'active_zero_before_silence', fb, profile, mode=2)
            axis['last_zero_write_finish_ns'] = fb['write_finish_ns']
            axis['last_zero_write_start_ns'] = fb['request_start_ns']
            # This one foreground owner is the sole writer on all four ports.
            # Guard checks only source/FD/boot/cancel; it must never poll CAN.
            silent_until = clock()+SILENCE_NS
            while clock() < silent_until:
                guard(); wait(min(.002, max(0., (silent_until-clock())/1e9)))
            fb = owner(mid).exchange(mid, 'zero', center=axis['center_rad'])
            elapsed = fb['received_ns']-axis['last_zero_write_start_ns']
            axis.update(stop_probe=fb, disable_reply_upper_bound_ms=elapsed/1e6,
                        disable_upper_bound_origin='last_zero_host_write_started_ns', configured_timeout_ms=200)
            _checked_feedback(axis, mid, 'disabled_zero_after_silence', fb, profile, mode=0)
            _need(SILENCE_NS <= elapsed <= MAX_DISABLE_UPPER_BOUND_NS,
                  f'ID{mid} disabled-state reply exceeded250ms conservative upper bound')
            axis.update(command_loss_tested=True, disabled_on_command_loss=True)
        report['status'] = 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC'
    except BaseException as error:
        report['errors'].append(_error_text(error))
    finally:
        stops = {}
        for port, channel in channels.items():
            try:
                stops[port] = channel.stop_all()
            except BaseException as error:
                stops[port] = {'complete': False, 'confirmed_ids': [],
                    'unconfirmed_ids': list(base['topology_by_port'][port]),
                    'ambiguous_ids': list(base['topology_by_port'][port]), 'errors': [_error_text(error)]}
        report['stop_reports'] = stops
        report['stop_confirmed'] = all(item.get('complete') is True and not item['unconfirmed_ids'] and
                                       not item['errors'] and not item.get('ambiguous_ids') for item in stops.values())
        if not report['stop_confirmed']:
            report['status'] = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
            report['errors'].append('Physically switch motor power Off; STOP acknowledgement incomplete')
        report['hardware_opened'] = True
        report['opens_devices'] = True
        try:
            report['elapsed_s'] = (clock()-started)/1e9
        except BaseException as error:
            report['elapsed_s'] = None
            report['errors'].append('Final clock: '+_error_text(error))
            if report['status'].startswith('COMPLETE'):report['status']='ABORTED'
        report['diagnostic_storage_by_port'] = {port: {
            'unlogged_receive_failure': getattr(channel, 'unlogged_receive_failure', False),
            'unlogged_exchange_failure': getattr(channel, 'unlogged_exchange_failure', False)}
            for port, channel in channels.items()}
        if any(any(row.values()) for row in report['diagnostic_storage_by_port'].values()):
            report['errors'].append('Original watchdog event storage failure retained')
            if report['status'].startswith('COMPLETE'): report['status'] = 'ABORTED'
    return report


def trace(report, channels):
    """Detach original raw events only after all STOPs and final report metadata."""
    return {'schema': 'singularitydog.four-can-watchdog-trace.v1',
        'report_canonical_sha256': HP.canonical_sha256(report),
        'events_by_port': {port: copy.deepcopy(channel.events) for port, channel in channels.items()},
        'failed_receive_or_partial_events_filtered': False, 'physical_USB_disconnect_tested': False}
