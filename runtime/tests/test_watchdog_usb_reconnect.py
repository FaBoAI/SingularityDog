"""Deterministic USB lifecycle simulation only; no devices or motor output."""
from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import watchdog_usb_reconnect as usb
from singularitydog_hw import watchdog_commissioning as loss


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def wait(self, seconds): self.now += max(1, round(seconds*1e9))


def uids():
    return {mid: bytes([mid]*8).hex() for mid in loss.IDS}


def source():
    return {'status': 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC', 'group': 'all',
        'stop_confirmed': True, 'boot_id': 'test-boot', 'motor_power_epoch': 'test-power',
        'motor_enable_sent': True, 'positive_gain_sent': False, 'learned_targets_sent': False,
        'axes': {str(i): {'uid': uids()[i], 'command_loss_tested': True,
            'disabled_on_command_loss': True, 'configured_timeout_ms': 200,
            'disable_reply_upper_bound_ms': 213.5,
            'disable_upper_bound_origin': 'last_zero_host_write_started_ns', 'version': {'version_bytes_hex': '0005000d'}}
            for i in loss.IDS}}


class Channel:
    def __init__(self, ids, clock, *, bad=None):
        self.ids, self.clock, self.bad = ids, clock, bad
        self.calls = []; self.events = []; self.enabled = set(); self.closed = False
    def exchange(self, mid, step, center=0.):
        if self.closed: raise OSError('USB fd gone')
        start = self.clock()
        self.calls.append((start, mid, step)); self.events.append({'mid': mid, 'step': step})
        self.clock.now += 1_000_000
        if step == 'identity': return {'mcu_uid_hex': '00'*8 if self.bad == 'uid' else uids()[mid]}
        if step == 'version': return {'version_bytes_hex': '0005000c' if self.bad == 'version' else '0005000d'}
        if step == 'voltage': return {'value': 40.}
        if step == 'run_mode': return {'value': 0}
        if step == 'can_timeout': return {'value': 4000}
        if step == 'stop': self.enabled.discard(mid)
        elif step == 'enable': self.enabled.add(mid)
        elif step != 'zero': raise AssertionError('Unexpected '+step)
        return {'mode_state': 2 if mid in self.enabled else 0, 'fault_bits': 0,
            'protocol_position_rad': 0., 'velocity_rad_s': 0., 'temperature_c': 25.,
            'request_start_ns': start, 'write_finish_ns': start+10_000, 'received_ns': self.clock()}
    def stop_all(self):
        self.calls.append((self.clock(), None, 'cleanup_stop'))
        self.enabled.clear()
        return {'complete': not self.closed, 'confirmed_ids': [] if self.closed else list(self.ids),
            'unconfirmed_ids': list(self.ids) if self.closed else [],
            'errors': ['USB gone'] if self.closed else []}


class Tests(unittest.TestCase):
    def run_case(self, scope='front', failure=None, command_loss=None):
        clock = Clock(); messages = []; opened = []; state = {}
        channels = {name: Channel(ids, clock) for name, ids in loss.BUSES.items()}
        old = channels[scope]
        def emit(text):
            messages.append(text)
            if text.startswith('USB_UNPLUG_NOW'): state['unplug_open'] = clock()
        def detached():
            if 'unplug_open' not in state: return False
            if failure == 'wrong_bus': raise RuntimeError('Unselected USB disappeared')
            if failure == 'no_detach': return False
            value = clock() >= state['unplug_open']+400_000_000
            if value: state.setdefault('detected', clock())
            if failure == 'early_reconnect' and value and clock() > state['detected']+1_000_000_000:
                return False
            return value
        def close(channel):
            self.assertIs(channel, old); channel.closed = True
        def reopen(deadline):
            self.assertGreater(deadline, clock())
            if failure == 'reconnect_timeout': raise TimeoutError('No reconnection')
            clock.wait(1.)
            new = Channel(loss.BUSES[scope], clock,
                          bad=failure if failure in ('uid', 'version') else None)
            if failure == 'still_active': new.enabled.update(new.ids)
            opened.append(new)
            return old if failure == 'same_channel' else new
        if failure == 'old_version': old.bad = 'version'
        report = usb.run(channels, uids(), command_loss or source(), scope=scope,
            boot_id='test-boot', power_epoch='test-power', detached=detached,
            reopen=reopen, close_detached=close, clock=clock, wait=clock.wait, emit=emit)
        return report, channels, old, opened, messages

    def test_each_bus_only_six_enabled_and_all_stopped_after_reconnect(self):
        for scope in loss.BUSES:
            with self.subTest(scope=scope):
                report, channels, old, opened, messages = self.run_case(scope)
                self.assertEqual(report['status'], 'RECORDED_USB_RECONNECT_REVIEW_REQUIRED', report['errors'])
                self.assertTrue(report['stop_confirmed'])
                self.assertTrue(report['usb_disconnect_observed_by_os'])
                self.assertTrue(report['disabled_after_reconnect_verified'])
                self.assertEqual([i for _, i, s in old.calls if s == 'enable'], list(loss.BUSES[scope]))
                other = channels['rear' if scope == 'front' else 'front']
                self.assertFalse(any(s == 'enable' for _, _, s in other.calls))
                self.assertFalse(report['positive_gain_sent']); self.assertFalse(report['learned_targets_sent'])
                self.assertFalse(report['usb_disconnect_test_passed'])
                self.assertIsNone(report['usb_outage_disable_upper_bound_ms'])
                self.assertFalse(report['usb_outage_disable_timing_verified'])
                self.assertTrue(any(m.startswith('USB_RECONNECT_NOW') for m in messages))

    def test_reopen_checks_uid_then_disabled_before_even_type4_firmware_and_stop(self):
        report, _, _, opened, _ = self.run_case()
        calls = [(i, step) for _, i, step in opened[0].calls]
        self.assertEqual(calls[:6], [(i, 'identity') for i in loss.BUSES['front']])
        self.assertEqual(calls[6:12], [(i, 'zero') for i in loss.BUSES['front']])
        self.assertEqual(calls[12:18], [(i, 'version') for i in loss.BUSES['front']])
        self.assertEqual(calls[18:], [(None, 'cleanup_stop')])
        self.assertTrue(report['disabled_after_reconnect_verified'])

    def test_no_can_transmissions_between_absence_and_new_channel(self):
        report, channels, old, _, _ = self.run_case()
        start, end = report['device_absence_detected_ns'], report['new_channel_opened_ns']
        self.assertGreaterEqual(end-start, usb.MIN_ABSENT_NS)
        for c in [old, *channels.values()]:
            self.assertFalse(any(start < ns < end for ns, _, _ in c.calls))

    def test_physical_confirmation_is_separate_and_never_claims_outage_timing(self):
        report, *_ = self.run_case()
        denied = usb.operator_review(copy.deepcopy(report), 'no')
        self.assertFalse(denied['usb_disconnect_test_passed'])
        approved = usb.operator_review(report, 'yes')
        self.assertEqual(approved['status'], 'COMPLETE_USB_RECONNECT_DISABLED_DIAGNOSTIC')
        self.assertTrue(approved['usb_disconnect_test_passed'])
        self.assertFalse(approved['usb_outage_disable_timing_verified'])
        self.assertIsNone(approved['usb_outage_disable_upper_bound_ms'])
        self.assertFalse(approved['approved_for_runtime'])
        self.assertEqual(approved['command_loss_disable_upper_bound_ms_by_id']['1'], 213.5)

    def test_no_detach_is_finite_abort_and_stops_both_buses(self):
        report, _, _, _, _ = self.run_case(failure='no_detach')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['usb_disconnect_observed_by_os'])
        self.assertLess(report['elapsed_s'], 16.)

    def test_wrong_bus_is_abort_not_selected_usb_success(self):
        report, *_ = self.run_case(failure='wrong_bus')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertFalse(report['disabled_after_reconnect_verified'])

    def test_preflight_firmware_change_never_enables(self):
        report, _, old, *_ = self.run_case(failure='old_version')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(any(s == 'enable' for _, _, s in old.calls))

    def test_reconnected_wrong_uid_or_fw_stops_without_enable(self):
        for failure in ('uid', 'version'):
            with self.subTest(failure=failure):
                report, _, _, opened, _ = self.run_case(failure=failure)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertTrue(report['stop_confirmed'])
                self.assertFalse(report['disabled_after_reconnect_verified'])
                self.assertFalse(any(s == 'enable' for _, _, s in opened[0].calls))
                if failure == 'uid': self.assertFalse(any(s == 'zero' for _, _, s in opened[0].calls))

    def test_motor_still_active_after_reconnect_fails_and_stops(self):
        report, _, _, opened, _ = self.run_case(failure='still_active')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['disabled_after_reconnect_verified'])
        self.assertEqual(sum(s == 'zero' for _, _, s in opened[0].calls), 1)
        self.assertFalse(any(s == 'version' for _, _, s in opened[0].calls))

    def test_early_reconnect_missing_reconnect_or_reused_channel_preserves_stop_uncertainty(self):
        for failure in ('early_reconnect', 'reconnect_timeout', 'same_channel'):
            with self.subTest(failure=failure):
                report, *_ = self.run_case(failure=failure)
                self.assertEqual(report['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
                self.assertFalse(report['stop_confirmed'])
                self.assertFalse(report['disabled_after_reconnect_verified'])
                self.assertFalse(usb.operator_review(report, 'yes')['usb_disconnect_test_passed'])

    def test_source_requires_same_power_boot_uid_firmware_and_actual_loss(self):
        for change in ('status', 'boot', 'power', 'uid', 'firmware', 'loss', 'bound', 'bound_origin'):
            with self.subTest(change=change):
                value = source()
                if change == 'status': value['status'] = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
                if change == 'boot': value['boot_id'] = 'new-boot'
                if change == 'power': value['motor_power_epoch'] = 'new-power'
                if change == 'uid': value['axes']['1']['uid'] = '00'*8
                if change == 'firmware': value['axes']['1']['version']['version_bytes_hex'] = None
                if change == 'loss': value['axes']['1']['disabled_on_command_loss'] = False
                if change == 'bound': value['axes']['1']['disable_reply_upper_bound_ms'] = 251
                if change == 'bound_origin': value['axes']['1']['disable_upper_bound_origin'] = 'write_finish'
                with self.assertRaises(RuntimeError):
                    self.run_case(command_loss=value)

    def test_default_plan_does_not_need_loss_report_or_open_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'uids.json'; path.write_text(json.dumps(uids()))
            stream = io.StringIO()
            with redirect_stdout(stream), patch.object(usb, 'run') as run:
                self.assertEqual(usb.main(['--bus','front','--expected-uids',str(path)]), 0)
                run.assert_not_called()
            self.assertFalse(json.loads(stream.getvalue())['hardware_opened'])

    def test_trace_budget_is_finite_and_fits_maximum_fifty_ms_window(self):
        channel = object.__new__(usb.USBChannel); channel.events = []
        # 15s *20 rounds/s *6 axes *2 raw TX/RX events plus preflight room.
        for _ in range(3800): channel.event({})
        for _ in range(8192-3800): channel.event({})
        with self.assertRaises(RuntimeError): channel.event({})


if __name__ == '__main__': unittest.main()
