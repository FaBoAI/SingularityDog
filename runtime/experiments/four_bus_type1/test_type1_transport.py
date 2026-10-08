"""Four-bus Type1 transport: mock-session unit tests and genuine C++ over sockets; no hardware."""
from concurrent.futures import Future
import ctypes as C
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import shutil
import tempfile
import threading
import time
from types import MappingProxyType, SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.motor_version_probe import version_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw.policy_output_runtime import decode_records
from experiments.four_bus_diagnostic import build as stop_build
from experiments.four_bus_diagnostic import transport_adapter
from experiments.four_bus_diagnostic.transport_adapter import Batch, Group
from . import build
from . import type1_transport as T

BOOT = '11111111-2222-3333-4444-555555555555'
SEAL = object()
ARMED = SimpleNamespace(exchange_at=('sda_subset_exchange_at_abi', 'sda_subset_exchange_at'))


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def reply(raw, *, type1_mode=2, enable_mode=2, stop_mode=0, fault=0, timeout=4000, run_mode=0, voltage=40.):
    value = ATParser().feed(raw)[0]; mid = value.destination
    if value.kind == 0:
        return frame((mid << 8)|0xfe, bytes([mid])*8)
    if value.kind == 17:
        index = int.from_bytes(value.data[:2], 'little')
        payload = (struct.pack('<I', timeout) if index == 0x7028 else bytes((run_mode, 0, 0, 0))
                   if index == 0x7005 else struct.pack('<f', voltage if index == 0x701c else .01))
        return frame((17 << 24)|(mid << 8)|0xfd, value.data[:4]+payload)
    if value.kind == 4 and value.data[:2] == b'\x00\xc4':
        return frame((2 << 24)|(mid << 8)|0xfd, b'\x00\xc4\x56\x01\x02\x03\x04\x07')
    mode = {1: type1_mode, 3: enable_mode, 4: stop_mode, 18: stop_mode}.get(value.kind, 0)
    return frame((2 << 24)|(mode << 22)|(fault << 16)|(mid << 8)|0xfd,
                 struct.pack('>4H', 32767, 32767, 32767, 250))


def kind(wire):
    return int.from_bytes(wire[2:6], 'big') >> 27


def bounds(ids, width=.1):
    return {mid: (-width/2, width/2) for mid in ids}


def caps(ids, value):
    return {mid: value for mid in ids}


class MockLib:
    """Records every native call; ordinary/fixed-six entry points are forbidden."""
    def __init__(self):
        self.calls, self.forbidden, self.stop_script, self.observe = [], [], [], None
        self.reply, self.status, self.armed = reply, 0, []

    def sda_subset_exchange_at(self, handle, mask, raw, count, send_only, not_before, deadline, records, stats,
                               error, size):
        # Stand-in for the native wait: same window rule, then the ordinary mock exchange.
        called = time.monotonic_ns()
        self.armed.append((called, not_before, deadline, send_only))
        if not (send_only == 0 and called < not_before < deadline and not_before-called <= 5_000_000):
            error.value = b'Pre-armed exchange requires now<not_before<deadline'; return -1
        while time.monotonic_ns() < not_before:
            pass
        return self.sda_subset_exchange(handle, mask, raw, count, send_only, deadline, records, stats, error, size)

    def sda_subset_exchange(self, handle, mask, raw, count, send_only, deadline, records, stats, error, size):
        wires = [bytes(raw)[i*17:(i+1)*17] for i in range(count)]
        self.calls.append(('exchange', mask, wires, send_only, deadline,
                           self.observe() if self.observe else None))
        stats = stats._obj; stats.begin_ns = time.monotonic_ns()
        for record, wire in zip(records, wires):
            now = time.monotonic_ns()
            record.tx[:] = wire; record.deadline_ns = deadline
            record.start_ns, record.finish_ns, record.read_start_ns = now, now+1, now+2
            record.written = 17
            response = self.reply(wire)
            if response:
                record.rx[:] = response; record.received_ns = now+3; record.received = 17
        stats.end_ns = time.monotonic_ns()+4; stats.writes = count
        if self.status:
            error.value = b'Injected native subset failure'; return -1
        return 0

    def sda_subset_validate(self, handle, mask, raw, count, error, size):
        self.calls.append(('validate', mask, [bytes(raw)[i*17:(i+1)*17] for i in range(count)]))
        return 0

    def sda_emergency_stop_subset(self, handle, mask, deadline, records, stats, result, error, size):
        self.calls.append(('stop_subset', mask, deadline))
        row = self.stop_script.pop(0) if self.stop_script else {}
        result = result._obj
        result.attempted_mask = row.get('attempted', mask); result.confirmed_mask = row.get('confirmed', mask)
        result.ambiguous_mask = row.get('ambiguous', 0)
        for slot, bits in row.get('fault', {}).items(): result.fault[slot] = bits
        first = 1 if mask == 0x07 or handle == 1 else 7
        for slot in range(6):
            if mask & (1 << slot) and result.confirmed_mask & (1 << slot):
                records[slot].tx[:] = stop_wire(first+slot); records[slot].written = records[slot].received = 17
        error.value = row.get('message', b'scripted')
        return row.get('status', 0 if result.confirmed_mask == mask and not result.ambiguous_mask else 1)

    def sda_exchange(self, *args):
        self.forbidden.append('sda_exchange'); raise AssertionError('ordinary sda_exchange called')

    def sda_emergency_stop(self, *args):
        self.forbidden.append('sda_emergency_stop'); raise AssertionError('fixed-six STOP called')


class MockSession:
    created = []

    def __init__(self, library, fd, *, first_id, cancel_fd, boot_fd, boot_id, raw_lower_by_id,
                 raw_upper_by_id, kp_max_by_id, kd_max_by_id, gap_ns, window):
        self.lib, self.fd, self.first_id, self._handle = library, fd, first_id, first_id
        self.busy, self.poisoned, self._phase_pair = threading.Lock(), False, None
        self.settings = dict(cancel_fd=cancel_fd, boot_fd=boot_fd, boot_id=boot_id, gap_ns=gap_ns, window=window)
        self._limits = active.Limits()
        for name, values in (('lower', raw_lower_by_id), ('upper', raw_upper_by_id),
                             ('kp', kp_max_by_id), ('kd', kd_max_by_id)):
            assert set(values) == set(range(first_id, first_id+6))
            for index in range(6): getattr(self._limits, name)[index] = values[first_id+index]
        self.closed = False
        MockSession.created.append(self)

    def close(self): self._handle = None; self.closed = True
    def exchange(self, *args, **kwargs): self.lib.forbidden.append('exchange'); raise AssertionError
    def emergency_stop(self, *args, **kwargs): self.lib.forbidden.append('emergency_stop'); raise AssertionError
    def emergency_stop_repeated(self, *args, **kwargs):
        self.lib.forbidden.append('emergency_stop_repeated'); raise AssertionError


class MockTransportTests(unittest.TestCase):
    def setUp(self):
        self.seal = SEAL
        self.patches = [patch.object(T.active, 'ActiveSession', MockSession),
                        patch.object(T, 'verify_library', lambda library: self.seal),
                        patch.object(T.active, 'verified_active_session_creation', lambda session: ('c', 'b'))]
        for value in self.patches: value.start()
        MockSession.created = []
        self.lib = MockLib(); self.cancels = []

    def tearDown(self):
        for value in reversed(self.patches): value.stop()
        self.assertEqual(self.lib.forbidden, [])

    def transport(self, group=Group('port3', (1, 2, 3)), *, kp=3., kd=.15, cancel_all=None, **overrides):
        arguments = dict(group=group, cancel_fd=10, boot_fd=11, boot_id=BOOT,
                         axis_raw_bounds=bounds(group.ids), kp_cap_by_id=caps(group.ids, kp),
                         kd_cap_by_id=caps(group.ids, kd),
                         cancel_all=cancel_all or (lambda: self.cancels.append(time.monotonic_ns())))
        arguments.update(overrides)
        return T.Type1Transport.create(self.lib, 9, **arguments)

    def deadline(self, ms=50):
        return time.monotonic_ns()+ms*1_000_000

    def ready(self, transport):
        for mid in transport.group.ids:
            transport.enable(mid, deadline_ns=self.deadline())
            transport.zero_gain(mid, 0., deadline_ns=self.deadline())
        return transport

    def sent(self):
        return [call[2] for call in self.lib.calls if call[0] == 'exchange']

    def assert_rejected(self, transport, action, pattern=None, *, native_calls=None):
        before = len(self.lib.calls) if native_calls is None else native_calls
        cancels = len(self.cancels)
        with self.assertRaisesRegex(Exception, pattern or '.'):
            action()
        self.assertEqual(len(self.lib.calls), before)
        self.assertTrue(transport.poisoned); self.assertTrue(transport._session.poisoned)
        self.assertEqual(len(self.cancels), cancels+1)
        with self.assertRaisesRegex(Exception, 'poisoned|binding changed'):
            transport.stop(deadline_ns=self.deadline())
        self.assertEqual(len(self.lib.calls), before)

    def test_member_caps_and_unreachable_nonmember_windows(self):
        for group, members in ((Group('port2', (4, 5, 6)), (4, 5, 6)), (Group('port0', (7, 8, 9)), (7, 8, 9))):
            transport = self.transport(group, kp=2.5, kd=.1)
            session = transport._session; limits = session._limits
            self.assertEqual((session.first_id, session.settings['gap_ns'], session.settings['window']),
                             (group.first_id, 900_000, 3))
            for slot in range(6):
                mid = group.first_id+slot
                if mid in members:
                    self.assertEqual((limits.lower[slot], limits.upper[slot], limits.kp[slot], limits.kd[slot]),
                                     (-.05, .05, 2.5, .1))
                else:
                    lo, hi = limits.lower[slot], limits.upper[slot]
                    self.assertEqual((limits.kp[slot], limits.kd[slot]), (0., 0.))
                    self.assertTrue(-12.57 < lo < hi < 12.57)
                    self.assertFalse(any(lo <= T.decoded_q(code) <= hi for code in range(65536)))

    def test_create_rejects_unsafe_limits_without_session(self):
        group = Group('port3', (1, 2, 3))
        for overrides, pattern in (
                (dict(kp_cap_by_id=caps((1, 2, 3), 3.01)), 'kp cap'),
                (dict(kd_cap_by_id=caps((1, 2, 3), .16)), 'kd cap'),
                (dict(kp_cap_by_id=caps((1, 2, 3), -.1)), 'kp cap'),
                (dict(kp_cap_by_id={1: 1., 2: 1., 3: math.nan}), 'kp cap'),
                (dict(kp_cap_by_id=caps((1, 2, 3, 4), 1.)), 'exactly'),
                (dict(axis_raw_bounds=bounds((1, 2))), 'exactly'),
                (dict(axis_raw_bounds=bounds((1, 2, 3), width=math.radians(6.01))), 'six degrees'),
                (dict(axis_raw_bounds={1: (.1, .1), 2: (0., .1), 3: (0., .1)}), 'ordered'),
                (dict(axis_raw_bounds={1: [0., .1], 2: (0., .1), 3: (0., .1)}), 'tuple'),
                (dict(axis_raw_bounds={1: (12.5, 12.6), 2: (0., .1), 3: (0., .1)}), 'range'),
                (dict(cancel_all=None), 'cancel_all')):
            with self.subTest(pattern=pattern, overrides=overrides):
                arguments = dict(overrides)
                if 'cancel_all' in arguments:
                    arguments['cancel_all'] = 'not callable'
                with self.assertRaisesRegex(ValueError, pattern):
                    T.Type1Transport.create(self.lib, 9, group=group, cancel_fd=10, boot_fd=11, boot_id=BOOT,
                        **{'axis_raw_bounds': bounds(group.ids), 'kp_cap_by_id': caps(group.ids, 3.),
                           'kd_cap_by_id': caps(group.ids, .15), 'cancel_all': lambda: None, **arguments})
        self.assertEqual(MockSession.created, [])
        with self.assertRaisesRegex(ValueError, 'creation factory'):
            T.Type1Transport(MockSession(self.lib, 9, first_id=1, cancel_fd=1, boot_fd=2, boot_id=BOOT,
                raw_lower_by_id=dict.fromkeys(range(1, 7), -1.), raw_upper_by_id=dict.fromkeys(range(1, 7), 1.),
                kp_max_by_id=dict.fromkeys(range(1, 7), 0.), kd_max_by_id=dict.fromkeys(range(1, 7), 0.),
                gap_ns=900_000, window=3), group, cancel_all=lambda: None)

    def test_full_handshake_uses_exact_wires_group_mask_and_truthful_flags(self):
        group = Group('port1', (10, 11, 12))
        transport = self.transport(group)
        d = self.deadline
        self.assertEqual(transport.identify(deadline_ns=d()), {mid: bytes([mid])*8 for mid in group.ids})
        stopped = transport.stop(deadline_ns=d())
        self.assertEqual({mid: (value.mode_state, value.fault_bits) for mid, value in stopped.items()},
                         dict.fromkeys(group.ids, (0, 0)))
        self.assertEqual(transport.version_probe(deadline_ns=d()), dict.fromkeys(group.ids, b'\x01\x02\x03\x04'))
        params = transport.read_params(['run_mode', 'voltage'], deadline_ns=d())
        self.assertEqual(params, {**{(mid, 'run_mode'): 0 for mid in group.ids},
                                  **{(mid, 'voltage'): 40. for mid in group.ids}})
        self.assertEqual(set(transport.write_watchdog(4000, deadline_ns=d())), set(group.ids))
        self.assertEqual(transport.read_params(('can_timeout',), deadline_ns=d()),
                         {(mid, 'can_timeout'): 4000 for mid in group.ids})
        self.assertEqual(transport.attempts, dict.fromkeys(transport.attempts, False))
        transport.enable(10, deadline_ns=d())
        self.assertEqual(transport.attempts, {'motor_enable_sent': True, 'type1_sent': False,
                                              'positive_gain_sent': False})
        self.assertEqual(transport.zero_gain(10, .01, deadline_ns=d()).mode_state, 2)
        self.assertEqual(transport.attempts['type1_sent'], True)
        for mid in (11, 12):
            transport.enable(mid, deadline_ns=d()); transport.zero_gain(mid, 0., deadline_ns=d())
        zero = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in group.ids)
        transport.output(zero, deadline_ns=d(), check=lambda: None)
        self.assertFalse(transport.attempts['positive_gain_sent'])
        positive = tuple(active.encode_motion(mid, .02, 3., .15) for mid in group.ids)
        batch = transport.output(positive, deadline_ns=d(), check=lambda: None)
        self.assertTrue(transport.attempts['positive_gain_sent'])
        self.assertIs(type(batch), Batch); self.assertEqual(transport.verify_batch(batch, 'output'), batch.rows)
        future = Future(); future.set_running_or_notify_cancel()
        hold, voltage = transport.hold_then_voltage(positive, 11, future, deadline_ns=d(), check=lambda: None)
        self.assertIs(future.result(timeout=0), hold)
        self.assertEqual(set(voltage.rows), {(11, 'voltage')})
        expected = [list(read_request(mid) for mid in group.ids), [stop_wire(mid) for mid in group.ids],
                    [version_request(mid) for mid in group.ids],
                    [read_request(mid, name) for name in ('run_mode', 'voltage') for mid in group.ids],
                    [protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid)
                     for mid in group.ids], [read_request(mid, 'can_timeout') for mid in group.ids],
                    [protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=10)],
                    [active.encode_motion(10, .01, 0., 0.)]]
        self.assertEqual(self.sent()[:8], expected)
        self.assertEqual(self.sent()[-3:], [list(positive), list(positive), [read_request(11, 'voltage')]])
        self.assertEqual({call[1] for call in self.lib.calls}, {group.mask})
        self.assertEqual({call[3] for call in self.lib.calls}, {0})
        self.assertEqual([label for label, _ in transport.journal][-3:], ['output', 'feedback_hold', 'voltage'])
        self.assertEqual(self.cancels, []); self.assertFalse(transport.poisoned)

    def test_whitelist_rejects_before_native_and_cancels_siblings(self):
        group = Group('port3', (1, 2, 3))
        d = self.deadline
        good = tuple(active.encode_motion(mid, 0., 1., .1) for mid in group.ids)
        vref = bytearray(good[1]); vref[9:11] = b'\x00\x00'
        cases = (
            (lambda t: t.enable(4, deadline_ns=d()), 'Same-group'),
            (lambda t: t.zero_gain(1, 0., deadline_ns=d()), 'enabled'),
            (lambda t: t.output(good, deadline_ns=d(), check=lambda: None), 'zero-gain handshake'),
            (lambda t: t.write_watchdog(3000, deadline_ns=d()), '4000'),
            (lambda t: t.read_params(['current'], deadline_ns=d()), 'allowlisted'),
            (lambda t: t.read_params(['voltage', 'run_mode', 'position'], deadline_ns=d()), 'allowlisted'),
            (lambda t: t._exchange('stop', (stop_wire(1), stop_wire(2), stop_wire(4)), d()), 'exact three stop'),
            (lambda t: t._exchange('enable', (stop_wire(1),), d()), 'Type3'),
            (lambda t: t._exchange('identify', (read_request(1),), d()), 'identify'),
            (lambda t: t._exchange('fixed_six', (stop_wire(1),), d()), 'Unknown'),
            (lambda t: t._exchange('voltage', (read_request(4, 'voltage'),), d()), 'voltage'),
            (lambda t: t._exchange('params', (read_request(1, 'current'),)*3, d()), 'allowlisted'))
        for action, pattern in cases:
            with self.subTest(pattern=pattern):
                transport = self.transport(group)
                self.assert_rejected(transport, lambda: action(transport), pattern)
        ready_cases = (
            (good[::-1], 'destination'), (good[:2], 'exactly three'), (good+good[:1], 'exactly three'),
            ((good[0], active.encode_motion(4, 0., 1., .1), good[2]), 'destination'),
            ((good[0], active.encode_motion(2, 0., 3.1, .1), good[2]), 'above its cap'),
            ((good[0], active.encode_motion(2, 0., 1., .16), good[2]), 'above its cap'),
            ((good[0], active.encode_motion(2, .06, 1., .1), good[2]), 'raw window'),
            ((good[0], active.encode_motion(2, -.06, 1., .1), good[2]), 'raw window'),
            ((good[0], bytes(vref), good[2]), 'Noncanonical'),
            ((good[0], stop_wire(2), good[2]), 'Noncanonical'))
        for wires, pattern in ready_cases:
            with self.subTest(pattern=pattern, wires=[w.hex() for w in wires]):
                transport = self.ready(self.transport(group))
                with self.assertRaisesRegex(ValueError, pattern):
                    transport.check_output_wires(wires)
                self.assertFalse(transport.poisoned)
                self.assert_rejected(transport, lambda: transport.output(wires, deadline_ns=d(), check=lambda: None),
                                     pattern)
        transport = self.ready(self.transport(group))
        self.assert_rejected(transport, lambda: transport.zero_gain(2, 0.06, deadline_ns=d()), 'raw window')
        transport = self.ready(self.transport(group))
        future = Future()
        self.assert_rejected(transport, lambda: transport.hold_then_voltage(good, 1, future, deadline_ns=d(),
                                                                            check=lambda: None), 'last validated')
        self.assertIsInstance(future.exception(timeout=0), ValueError)
        transport = self.ready(self.transport(group))
        transport.output(good, deadline_ns=d(), check=lambda: None)
        changed = (good[0], active.encode_motion(2, .001, 1., .1), good[2])
        self.assert_rejected(transport, lambda: transport.hold_then_voltage(changed, 1, Future(), deadline_ns=d(),
                                                                            check=lambda: None), 'byte for byte')
        transport = self.ready(self.transport(group))
        transport.output(good, deadline_ns=d(), check=lambda: None)
        self.assert_rejected(transport, lambda: transport.hold_then_voltage(good, 4, Future(), deadline_ns=d(),
                                                                            check=lambda: None), 'physical group')

    def test_required_reply_modes_and_faults(self):
        d = self.deadline
        def run(method, mutate, pattern, prepare=lambda t: None):
            transport = self.transport()
            prepare(transport)
            self.lib.reply = mutate
            calls = len(self.lib.calls)
            with self.assertRaisesRegex((ValueError, RuntimeError), pattern):
                method(transport)
            self.lib.reply = reply
            self.assertEqual(len(self.lib.calls), calls+1)
            self.assertTrue(transport.poisoned); self.assertEqual(transport.journal[-1][0], method.label)
            return transport
        def labelled(label, function):
            function.label = label; return function
        stop = labelled('stop', lambda t: t.stop(deadline_ns=d()))
        run(stop, lambda w: reply(w, stop_mode=2), 'mode 0')
        run(stop, lambda w: reply(w, fault=1), 'fault zero')
        enable = labelled('enable', lambda t: t.enable(2, deadline_ns=d()))
        run(enable, lambda w: reply(w, enable_mode=1), 'mode 0/2')
        run(enable, lambda w: reply(w, fault=4), 'fault zero')
        self.assertEqual(self.transport().enable(2, deadline_ns=d()).mode_state, 2)
        self.lib.reply = lambda w: reply(w, enable_mode=0)
        self.assertEqual(self.transport().enable(2, deadline_ns=d()).mode_state, 0)
        self.lib.reply = reply
        zero = labelled('zero_gain', lambda t: t.zero_gain(2, 0., deadline_ns=d()))
        run(zero, lambda w: reply(w, type1_mode=0), 'mode 2', lambda t: t.enable(2, deadline_ns=d()))
        watchdog = labelled('watchdog_setup', lambda t: t.write_watchdog(4000, deadline_ns=d()))
        run(watchdog, lambda w: reply(w, stop_mode=2), 'mode 0')
        output = labelled('output', lambda t: t.output(tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3)),
                                                       deadline_ns=d(), check=lambda: None))
        run(output, lambda w: reply(w, type1_mode=0 if w[5] >> 3 & 31 == 3 else 2), 'ID3 output requires mode 2',
            self.ready)
        run(output, lambda w: reply(w, fault=2), 'fault zero', self.ready)
        missing = labelled('stop', lambda t: t.stop(deadline_ns=d()))
        run(missing, lambda w: None if w == stop_wire(2) else reply(w), 'Incomplete')
        self.assertEqual(len(self.cancels), 9)  # One per rejected method call.

    def test_native_failure_journals_raw_poisons_and_cancels(self):
        transport = self.transport(); self.lib.status = -1
        with self.assertRaisesRegex(active.ExchangeError, 'Injected native'):
            transport.stop(deadline_ns=self.deadline())
        self.assertEqual(transport.journal[-1][0], 'stop'); self.assertEqual(len(transport.journal[-1][1][0]), 3)
        self.assertEqual(len(self.cancels), 1); self.assertTrue(transport.poisoned)
        self.assertIn('Injected native', transport.failures[0])

    def test_invalid_deadlines_rejected_before_native(self):
        for deadline in (time.monotonic_ns()-1, time.monotonic_ns()+260_000_000, float(time.monotonic_ns()+10**7)):
            with self.subTest(deadline=deadline):
                transport = self.transport()
                self.assert_rejected(transport, lambda: transport.stop(deadline_ns=deadline), 'deadline')

    def test_single_owner_thread(self):
        transport = self.transport()
        transport.stop(deadline_ns=self.deadline())
        errors = []
        def other():
            try: transport.stop(deadline_ns=self.deadline())
            except BaseException as error: errors.append(error)
        calls = len(self.lib.calls)
        thread = threading.Thread(target=other); thread.start(); thread.join(timeout=1)
        self.assertRegex(str(errors[0]), 'retain ownership'); self.assertEqual(len(self.lib.calls), calls)
        self.assertTrue(transport.poisoned); self.assertEqual(len(self.cancels), 1)
        errors = []
        thread = threading.Thread(target=lambda: errors.append(
            self.assertRaises(RuntimeError, transport.stop_repeated))); thread.start(); thread.join(timeout=1)
        self.assertEqual(len(self.lib.calls), calls)

    def test_binding_changes_are_rejected(self):
        transport = self.transport()
        transport._wires = dict(transport._wires, stop=tuple(active.encode_motion(m, 0., 0., 0.) for m in (1, 2, 3)))
        self.assert_rejected(transport, lambda: transport.stop(deadline_ns=self.deadline()), 'binding changed')
        transport = self.transport()
        transport._session._limits.kp[0] = 30.
        self.assert_rejected(transport, lambda: transport.stop(deadline_ns=self.deadline()), 'binding changed')
        transport = self.transport()
        transport._cancel_all = lambda: None
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            transport.stop(deadline_ns=self.deadline())

    def test_cancel_all_failure_is_noted_and_original_error_raised(self):
        def broken(): raise OSError('cancel pipe closed')
        transport = self.transport(cancel_all=broken)
        with self.assertRaisesRegex(ValueError, 'Same-group') as raised:
            transport.enable(7, deadline_ns=self.deadline())
        self.assertIn('Shared cancel_all failed: cancel pipe closed', raised.exception.__notes__)
        self.assertTrue(transport.poisoned); self.assertEqual(len(transport.failures), 2)

    def test_hold_publishes_prefix_before_voltage_and_propagates_failure(self):
        transport = self.ready(self.transport())
        wires = tuple(active.encode_motion(mid, 0., 1., .1) for mid in (1, 2, 3))
        transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        future = Future(); future.set_running_or_notify_cancel()
        self.lib.observe = lambda: future.done()
        checks = []
        hold, voltage = transport.hold_then_voltage(wires, 3, future, deadline_ns=self.deadline(),
                                                    check=lambda: checks.append(future.done()))
        self.assertEqual([call[5] for call in self.lib.calls[-2:]], [False, True])
        self.assertEqual(checks, [False, False, True])
        self.assertEqual(transport.verify_batch(hold, 'feedback_hold'), hold.rows)
        self.assertEqual([bytes(record.tx) for record in hold.records], list(wires))
        future = Future(); self.lib.reply = lambda w: reply(w, type1_mode=0)
        with self.assertRaisesRegex(ValueError, 'mode 2'):
            transport.hold_then_voltage(wires, 1, future, deadline_ns=self.deadline(), check=lambda: None)
        self.assertIsInstance(future.exception(timeout=0), ValueError)
        self.assertEqual(len(self.cancels), 1)
        self.lib.reply = reply; transport = self.ready(self.transport())
        transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        future = Future(); sibling = RuntimeError('sibling port failed')
        def check():
            if future.done(): raise sibling
        with self.assertRaisesRegex(RuntimeError, 'sibling'):
            transport.hold_then_voltage(wires, 1, future, deadline_ns=self.deadline(), check=check)
        self.assertEqual(self.sent()[-1], list(wires))  # Voltage never reached native.
        self.assertTrue(transport.poisoned)

    def test_validate_output_uses_native_validate_without_exchange(self):
        transport = self.transport()
        wires = tuple(active.encode_motion(mid, 0., 1., .1) for mid in (1, 2, 3))
        self.assertEqual(transport.validate_output(wires), wires)
        self.assertEqual(self.lib.calls, [('validate', 0x07, list(wires))])
        self.assert_rejected(transport, lambda: transport.validate_output(wires[::-1]), 'destination')

    def test_stop_repeated_confirms_first_round(self):
        transport = self.transport(Group('port2', (4, 5, 6)))
        result = transport.stop_repeated()
        self.assertTrue(result['complete'], result)
        self.assertEqual((result['confirmed_ids'], result['unconfirmed_ids'], result['ambiguous_ids']),
                         ([4, 5, 6], [], []))
        self.assertFalse(result['physical_cutoff_required']); self.assertEqual(len(result['rounds']), 1)
        self.assertEqual([call[:2] for call in self.lib.calls], [('stop_subset', 0x38)])
        self.assertTrue(transport.poisoned); self.assertEqual(self.cancels, [])
        self.assertEqual(transport.journal[-1][0], 'stop_subset')

    def test_stop_repeated_retries_missing_axis_and_keeps_ambiguity_sticky(self):
        transport = self.transport()
        self.lib.stop_script = [{'confirmed': 0b011}, {}]
        result = transport.stop_repeated()
        self.assertTrue(result['complete'], result); self.assertEqual(len(result['rounds']), 2)
        transport = self.transport()
        self.lib.stop_script = [{'confirmed': 0b011, 'ambiguous': 0b100}, {}, {}]
        result = transport.stop_repeated(total_budget_ns=900_000_000)
        self.assertFalse(result['complete']); self.assertTrue(result['physical_cutoff_required'])
        self.assertEqual((result['ambiguous_ids'], result['unconfirmed_ids']), ([3], [3]))
        self.assertEqual(len(result['rounds']), 3)
        later = transport.stop_repeated(rounds=1)
        self.assertEqual((later['complete'], later['ambiguous_ids']), (False, [3]))
        self.assertEqual([call[0] for call in self.lib.calls], ['stop_subset']*6)

    def test_stop_repeated_fault_is_not_complete_and_not_retried(self):
        transport = self.transport(Group('port0', (7, 8, 9)))
        self.lib.stop_script = [{'fault': {1: 4}}]
        result = transport.stop_repeated()
        self.assertFalse(result['complete']); self.assertTrue(result['physical_cutoff_required'])
        self.assertEqual(result['faults'], {'7': 0, '8': 4, '9': 0}); self.assertEqual(len(result['rounds']), 1)

    def test_stop_repeated_budget_rounds_and_busy_owner(self):
        transport = self.transport()
        for kwargs in ({'rounds': 0}, {'rounds': 4}, {'total_budget_ns': 20_000_000},
                       {'total_budget_ns': 1_000_000_001}, {'rounds': True}):
            with self.assertRaisesRegex(ValueError, 'rounds'):
                transport.stop_repeated(**kwargs)
        self.assertEqual(self.lib.calls, [])
        transport._session.busy.acquire()
        try:
            result = transport.stop_repeated(total_budget_ns=100_000_000)
        finally:
            transport._session.busy.release()
        self.assertFalse(result['complete']); self.assertEqual(self.lib.calls, [])
        self.assertRegex(result['rounds'][0]['error'], 'Join original owner')
        result = transport.stop_repeated(total_budget_ns=21_000_000)  # No 21 ms reserve remains.
        self.assertEqual((result['rounds'], result['complete'], result['unconfirmed_ids']), ([], False, [1, 2, 3]))
        self.assertTrue(result['physical_cutoff_required']); self.assertEqual(self.lib.calls, [])

    def test_close_releases_session(self):
        transport = self.transport(); transport.close(); transport.close()
        self.assertTrue(transport._session.closed)
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            transport.stop(deadline_ns=self.deadline())

    # Opt-in selections (F2b decode_once, F3 prearmed_hold) and the F2a owner cleanup.
    def cycle(self, transport, *, not_before=None):
        """Handshake, two outputs and three holds; returns comparable evidence."""
        d = self.deadline
        self.ready(transport)
        futures = []
        for wires, voltage_id in ((tuple(active.encode_motion(m, 0., 0., 0.) for m in (1, 2, 3)), 1),
                                  (tuple(active.encode_motion(m, .01*m, 2., .1) for m in (1, 2, 3)), 2),
                                  (None, 3)):
            if wires is not None:
                transport.output(wires, deadline_ns=d(), check=lambda: None)
                last = wires
            future = Future(); future.set_running_or_notify_cancel(); futures.append(future)
            hold, _ = transport.hold_then_voltage(last, voltage_id, future, deadline_ns=d(), check=lambda: None,
                **({} if not_before is None else {'not_before_ns': time.monotonic_ns()+not_before}))
            self.assertIs(future.result(timeout=0), hold)
        return {'calls': [call[:4] for call in self.lib.calls],
                'journal': [(label, [bytes(r.tx) for r in raw[0]], [(r.written, r.received) for r in raw[0]])
                            for label, raw in transport.journal],
                'rows': [{key: value[0] for key, value in batch.rows.items()} for batch in transport._batches],
                'attempts': dict(transport.attempts), 'cancels': len(self.cancels), 'poisoned': transport.poisoned,
                'last_output': transport._last_output}

    def test_selections_are_exact_sealed_and_default_off(self):
        transport = self.transport()
        self.assertEqual((transport.decode_once, transport.prearmed_hold), (False, False))
        self.assertEqual(T._TRANSPORTS[transport][6:], (False, False))
        for options, pattern in (({'decode_once': 1}, 'Decode-once'), ({'prearmed_hold': None}, 'Pre-armed'),
                                 ({'decode_once': 'true'}, 'exact bool'),
                                 ({'prearmed_hold': True}, 'sda_subset_exchange_at')):
            with self.subTest(options=options):
                MockSession.created = []
                with self.assertRaisesRegex(ValueError, pattern):
                    self.transport(**options)
                self.assertEqual(MockSession.created, [])  # Rejected before any session exists.
        self.seal = ARMED
        for name in ('decode_once', 'prearmed_hold'):
            with self.subTest(name=name):
                transport = self.transport(**{name: True})
                self.assertIs(getattr(transport, name), True)
                setattr(transport, name, False)
                self.assert_rejected(transport, lambda: transport.stop(deadline_ns=self.deadline()), 'binding changed')
        with self.assertRaisesRegex(ValueError, 'creation factory'):
            T.Type1Transport(self.transport()._session, Group('port3', (1, 2, 3)), cancel_all=lambda: None,
                             decode_once=True)

    def test_unused_selections_send_identical_bytes_journal_rows_and_flags(self):
        self.seal = ARMED
        baseline = self.cycle(self.transport())
        for options in ({'decode_once': True}, {'prearmed_hold': True}, {'decode_once': True, 'prearmed_hold': True}):
            with self.subTest(options=options):
                self.lib = MockLib(); self.cancels = []
                self.assertEqual(self.cycle(self.transport(**options)), baseline)
                self.assertEqual(self.lib.armed, [])  # No not_before given: ordinary export only.
        self.assertEqual(baseline['attempts'], dict.fromkeys(baseline['attempts'], True))
        self.assertEqual(baseline['journal'][-2:], [('feedback_hold', list(baseline['last_output']), [(17, 17)]*3),
                                                    ('voltage', [read_request(3, 'voltage')], [(17, 17)])])

    def test_prearmed_hold_uses_exchange_at_for_the_hold_only_with_identical_bytes(self):
        self.seal = ARMED
        baseline = self.cycle(self.transport())
        self.lib = MockLib(); self.cancels = []
        transport = self.transport(prearmed_hold=True)
        self.assertEqual(self.cycle(transport, not_before=2_000_000), baseline)
        exchanges = [call for call in self.lib.calls if call[0] == 'exchange']
        self.assertEqual(len(exchanges), len(transport.journal))
        holds = [call for call, (label, _) in zip(exchanges, transport.journal) if label == 'feedback_hold']
        self.assertEqual((len(holds), len(self.lib.armed)), (3, 3))  # Voltage and output stay ordinary.
        for (called, not_before, deadline, send_only), call, batch in zip(
                self.lib.armed, holds, [b for b in transport._batches if b.label == 'feedback_hold']):
            self.assertTrue(called < not_before < deadline and send_only == 0)
            self.assertEqual(call[4], deadline); self.assertGreaterEqual(batch.records[0].start_ns, not_before)

    def test_not_before_requires_selection_and_a_valid_window_before_any_write(self):
        def held(transport):
            wires = tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3))
            self.ready(transport).output(wires, deadline_ns=self.deadline(), check=lambda: None)
            return wires
        transport = self.transport(); wires = held(transport); future = Future()
        self.assert_rejected(transport, lambda: transport.hold_then_voltage(wires, 1, future,
            deadline_ns=self.deadline(), check=lambda: None, not_before_ns=time.monotonic_ns()+1_000_000),
            'explicit prearmed_hold selection')
        self.assertIsInstance(future.exception(timeout=0), ValueError)
        self.seal = ARMED
        now = time.monotonic_ns
        for value, pattern in ((lambda: float(now()+1_000_000), 'integer not_before_ns'),
                               (lambda: True, 'integer not_before_ns'), (lambda: 0, 'integer not_before_ns'),
                               (lambda: now()+60_000_000, 'integer not_before_ns'),
                               (lambda: now()-1, 'ahead'), (lambda: now()+6_000_000, 'at most 5 ms')):
            with self.subTest(pattern=pattern):
                self.lib = MockLib(); self.cancels = []
                transport = self.transport(prearmed_hold=True); wires = held(transport); future = Future()
                self.assert_rejected(transport, lambda: transport.hold_then_voltage(wires, 1, future,
                    deadline_ns=self.deadline(), check=lambda: None, not_before_ns=value()), pattern)
                self.assertEqual(self.lib.armed, []); self.assertIsInstance(future.exception(timeout=0), Exception)
        transport = self.transport(prearmed_hold=True); wires = held(transport)
        self.assert_rejected(transport, lambda: transport._exchange('output', wires, self.deadline(),
            not_before_ns=now()+1_000_000), 'Only the feedback hold')

    def test_owner_cleanup_parses_once_verifies_once_and_reuses_the_cached_voltage_wire(self):
        transport = self.ready(self.transport())
        wires = tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3))
        original = T.type1_fields
        counted = []
        def fields(wire):
            counted.append(wire); return original(wire)
        verify = transport._verify
        verified = []
        transport._verify = lambda: (verified.append(1), verify())[1]
        with patch.object(T, 'type1_fields', fields):
            transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
            self.assertEqual((counted, len(verified)), (list(wires), 1))
            del counted[:]; del verified[:]
            for voltage_id in (1, 2, 3):
                with patch.object(T, 'read_request', side_effect=AssertionError('rebuilt')):
                    transport.hold_then_voltage(wires, voltage_id, Future(), deadline_ns=self.deadline(),
                                                check=lambda: None)
            self.assertEqual((counted, len(verified)), ([], 6))  # Was 9 type1_fields and 9 verifies.
        del transport._verify
        self.assertEqual(self.sent()[-6:], [list(wires), [read_request(1, 'voltage')], list(wires),
                                            [read_request(2, 'voltage')], list(wires), [read_request(3, 'voltage')]])
        self.assertEqual(transport._wires['voltage'], tuple(read_request(m, 'voltage') for m in (1, 2, 3)))

    def test_mark_with_cached_fields_matches_the_original_parse(self):
        def reference(wires):
            flags = dict.fromkeys(('motor_enable_sent', 'type1_sent', 'positive_gain_sent'), False)
            for wire in wires:
                kind_ = int.from_bytes(wire[2:6], 'big') >> 27
                if kind_ == 3:
                    flags['motor_enable_sent'] = True
                elif kind_ == 1:
                    flags['type1_sent'] = True
                    fields = T.type1_fields(wire)
                    if fields is None or fields[2] or fields[3]:
                        flags['positive_gain_sent'] = True
            return flags
        vref = bytearray(active.encode_motion(2, 0., 0., 0.)); vref[9:11] = b'\x00\x00'
        pool = (active.encode_motion(1, 0., 0., 0.), active.encode_motion(2, .01, 0., 0.),
                active.encode_motion(3, 0., 0., .01), active.encode_motion(1, 0., .01, 0.), bytes(vref),
                stop_wire(1), protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=2),
                read_request(3, 'voltage'), version_request(1))
        for first in pool:
            for second in pool:
                for third in pool:
                    wires = (first, second, third)
                    for cached in (None, tuple(T.type1_fields(wire) for wire in wires)):
                        transport = self.transport()
                        transport._mark(wires, cached)
                        self.assertEqual(transport.attempts, reference(wires), [w.hex() for w in wires])

    def test_binding_change_before_hold_keeps_its_error_and_writes_nothing(self):
        transport = self.ready(self.transport())
        wires = tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3))
        transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        transport._wires = dict(transport._wires, voltage=(stop_wire(1),)*3)
        future = Future()
        self.assert_rejected(transport, lambda: transport.hold_then_voltage(wires, 1, future,
            deadline_ns=self.deadline(), check=lambda: None), 'Original four-bus Type1 owner/session/source binding changed')
        self.assertRegex(str(future.exception(timeout=0)), 'binding changed')
        errors = []
        def other():
            try: transport.hold_then_voltage(wires, 1, Future(), deadline_ns=self.deadline(), check=lambda: None)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=other); thread.start(); thread.join(timeout=1)
        self.assertRegex(str(errors[0]), 'retain ownership')  # Thread ownership is still checked first.

    def test_voltage_buffers_are_fresh_per_cycle_and_bound_to_the_batch(self):
        transport = self.ready(self.transport())
        wires = tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3))
        transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        first = transport.hold_then_voltage(wires, 1, Future(), deadline_ns=self.deadline(), check=lambda: None)[1]
        second = transport.hold_then_voltage(wires, 1, Future(), deadline_ns=self.deadline(), check=lambda: None)[1]
        self.assertIsNot(first.records, second.records); self.assertIsNot(first.stats, second.stats)
        self.assertEqual(transport.verify_batch(first, 'voltage'), first.rows)
        calls = len(self.lib.calls)
        with self.assertRaisesRegex(ValueError, 'Prepared native buffers'):
            transport._native((read_request(1, 'voltage'),), self.deadline(), None, None, T.Type1Transport._buffers(2))
        self.assertEqual(len(self.lib.calls), calls); self.assertTrue(transport.poisoned)

    def test_decode_once_publishes_read_only_rows_reused_only_for_the_exact_mapping(self):
        from experiments.four_bus_diagnostic import transport_adapter as adapter
        transport = self.ready(self.transport(decode_once=True))
        wires = tuple(active.encode_motion(m, 0., 1., .1) for m in (1, 2, 3))
        batch = transport.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        self.assertIs(type(batch.rows), MappingProxyType)
        self.assertEqual(batch.rows, decode_records((batch.records, batch.stats)))
        with self.assertRaises(TypeError):
            batch.rows[(1, 'feedback')] = batch.rows[(2, 'feedback')]
        with patch.object(adapter, 'decode_records', side_effect=AssertionError('re-decoded')), \
                patch.object(T, 'decode_records', side_effect=AssertionError('re-decoded')):
            self.assertIs(transport.verify_batch(batch, 'output'), batch.rows)
        hold, voltage = transport.hold_then_voltage(wires, 2, Future(), deadline_ns=self.deadline(), check=lambda: None)
        self.assertIs(transport.verify_batch(hold, 'feedback_hold'), hold.rows)
        decoded = []
        original = adapter.decode_records
        with patch.object(adapter, 'decode_records', lambda raw: (decoded.append(1), original(raw))[1]):
            self.assertEqual(transport.verify_batch(voltage, 'voltage'), voltage.rows)  # Dict values: full re-decode.
        self.assertEqual(decoded, [1])
        hold.records[1].rx[9] ^= 1
        with self.assertRaisesRegex(ValueError, 'raw batch'):
            transport.verify_batch(hold, 'feedback_hold')
        hold.records[1].rx[9] ^= 1
        rows = dict(hold.rows); value, start, received = rows[(1, 'feedback')]
        from dataclasses import replace
        rows[(1, 'feedback')] = (replace(value, torque_nm=value.torque_nm+1.), start, received)
        for substitute in (rows, MappingProxyType(rows)):
            object.__setattr__(hold, 'rows', substitute)
            with self.assertRaisesRegex(ValueError, 'decoded rows changed'):
                transport.verify_batch(hold, 'feedback_hold')
        forged = Batch(batch.group, batch.label, batch.records, batch.stats, batch.rows, batch.record_image,
                       batch.stats_image, batch.completed_ns)
        for candidate, label in ((forged, 'output'), (batch, 'feedback_hold')):
            with self.assertRaisesRegex(ValueError, 'Genuine'):
                transport.verify_batch(candidate, label)
        self.assertEqual((len(transport._batches), len(transport._batch_index)), (9, 9))
        default = self.ready(self.transport())
        batch = default.output(wires, deadline_ns=self.deadline(), check=lambda: None)
        self.assertIs(type(batch.rows), dict)
        with patch.object(adapter, 'decode_records', side_effect=AssertionError('re-decoded')):
            with self.assertRaisesRegex(AssertionError, 're-decoded'):
                default.verify_batch(batch, 'output')  # Default always re-decodes.


def sha(path):
    return T.sha(path)


class NativeType1TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        selected = os.environ.get('FOUR_BUS_TYPE1_TEST_LIBRARY')
        cls.library_path = (Path(selected).resolve() if selected else
                            build.build(Path(cls.directory.name)/'build'))
        cls.pins = cls.pins_for(cls.library_path)
        cls.lib = T.load_library(cls.library_path, **cls.pins)

    @staticmethod
    def pins_for(path):
        record_path = path.parent/'build-record.json'
        scope = json.loads(record_path.read_bytes()).get('four_bus_subset_active', {})
        return dict(expected_sha256=sha(path), ordinary_source_sha256=scope.get('ordinary_source_sha256'),
                    subset_stop_source_sha256=scope.get('subset_stop_source_sha256'),
                    extension_source_sha256=scope.get('extension_source_sha256'),
                    build_record_sha256=sha(record_path))

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.host, self.peer = socket.socketpair(); self.host.setblocking(False)
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile(); self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.transports = []; self.thread = None; self.seen = []; self.peer_error = None; self.cancels = 0

    def tearDown(self):
        for transport in self.transports: transport.close()
        self.host.close(); self.peer.close()
        if self.thread: self.thread.join(timeout=1)
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()
        if self.peer_error: raise self.peer_error

    def cancel_all(self):
        self.cancels += 1; os.write(self.cancel_write, b'x')

    def transport(self, group=Group('port2', (4, 5, 6)), kp=3., kd=.15, *, lib=None, **options):
        result = T.Type1Transport.create(lib or self.lib, self.host.fileno(), group=group,
            cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
            axis_raw_bounds=bounds(group.ids), kp_cap_by_id=caps(group.ids, kp),
            kd_cap_by_id=caps(group.ids, kd), cancel_all=self.cancel_all, **options)
        self.transports.append(result); return result

    def peer_loop(self, count, *, mutate=reply):
        if self.thread:
            self.thread.join(timeout=1)
            self.assertFalse(self.thread.is_alive(), 'Previous synthetic peer remains active')
        ready = threading.Event()
        def run():
            parser = ATParser(); seen = 0; ready.set()
            try:
                while seen < count:
                    if not select.select([self.peer], [], [], 1)[0]: return
                    raw = self.peer.recv(4096)
                    if not raw: return
                    for value in parser.feed(raw):
                        self.seen.append(value); seen += 1
                        response = mutate(value.wire)
                        if response: self.peer.sendall(response)
            except OSError:
                pass
            except BaseException as error:
                self.peer_error = error
        self.thread = threading.Thread(target=run, name='four-type1-transport-peer')
        self.thread.start(); self.assertTrue(ready.wait(timeout=1))

    def deadline(self, ms=100):
        return time.monotonic_ns()+ms*1_000_000

    def assert_silent(self):
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def cancelled(self):
        return bool(select.select([self.cancel_read], [], [], 0)[0])

    def test_loader_requires_subset_active_receipt_and_pins(self):
        self.assertIs(T.verify_library(self.lib).active_binding, active.verified_active_source_binding(self.lib))
        stop_library = stop_build.build(Path(self.directory.name)/f'stop-{time.monotonic_ns()}')
        with self.assertRaisesRegex(ValueError, 'library name|STOP-only'):
            T.load_library(stop_library, **self.pins_for(stop_library))
        with self.assertRaisesRegex(ValueError, 'pins required'):
            transport_adapter.load_library(self.library_path, expected_sha256=self.pins['expected_sha256'],
                ordinary_source_sha256=self.pins['ordinary_source_sha256'],
                extension_source_sha256=self.pins['extension_source_sha256'],
                build_record_sha256=self.pins['build_record_sha256'])
        for key in ('ordinary_source_sha256', 'subset_stop_source_sha256', 'extension_source_sha256',
                    'build_record_sha256'):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'pins|receipt differs'):
                T.load_library(self.library_path, **dict(self.pins, **{key: '0'*64}))
        with self.assertRaisesRegex(ValueError, 'differs from the pinned'):
            T.load_library(self.library_path, **dict(self.pins, expected_sha256='0'*64))
        plain = active.load_library(self.library_path, expected_sha256=self.pins['expected_sha256'])
        with self.assertRaisesRegex(ValueError, 'Genuine authenticated four-bus Type1'):
            T.verify_library(plain)
        with self.assertRaisesRegex(ValueError, 'Genuine authenticated subset library'):
            transport_adapter.verify_library(self.lib)
        original = self.lib.sda_subset_exchange
        try:
            self.lib.sda_subset_exchange = self.lib['sda_subset_exchange']
            with self.assertRaisesRegex(ValueError, 'function binding changed'):
                T.verify_library(self.lib)
        finally:
            self.lib.sda_subset_exchange = original
        T.verify_library(self.lib)

    def test_full_preflight_enable_output_hold_and_terminal_stop(self):
        group = Group('port2', (4, 5, 6))
        transport = self.transport(group)
        d = self.deadline
        self.peer_loop(3*5+6*2+2*3+3+3+4+3)
        self.assertEqual(transport.identify(deadline_ns=d()), {mid: bytes([mid])*8 for mid in group.ids})
        self.assertEqual(set(transport.stop(deadline_ns=d())), set(group.ids))
        self.assertEqual(transport.version_probe(deadline_ns=d(250)), dict.fromkeys(group.ids, b'\x01\x02\x03\x04'))
        modes = transport.read_params(('run_mode', 'voltage'), deadline_ns=d())
        self.assertEqual({key: value for key, value in modes.items() if key[1] == 'run_mode'},
                         {(mid, 'run_mode'): 0 for mid in group.ids})
        transport.write_watchdog(4000, deadline_ns=d())
        self.assertEqual(set(transport.read_params(('can_timeout',), deadline_ns=d()).values()), {4000})
        transport.read_params(('position', 'velocity'), deadline_ns=d())
        for mid in group.ids:
            self.assertEqual(transport.enable(mid, deadline_ns=d(30)).mode_state, 2)
            self.assertEqual(transport.zero_gain(mid, 0., deadline_ns=d(20)).mode_state, 2)
        zero = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in group.ids)
        transport.output(zero, deadline_ns=d(20), check=lambda: None)
        wires = tuple(active.encode_motion(mid, .01, 3., .15) for mid in group.ids)
        transport.validate_output(wires)
        batch = transport.output(wires, deadline_ns=d(20), check=lambda: None)
        self.assertEqual([record.received for record in batch.records], [17]*3)
        future = Future(); future.set_running_or_notify_cancel()
        hold, voltage = transport.hold_then_voltage(wires, 5, future, deadline_ns=d(20), check=lambda: None)
        self.assertIs(future.result(timeout=0), hold)
        self.assertEqual([bytes(record.tx) for record in hold.records], list(wires))
        self.assertEqual(set(voltage.rows), {(5, 'voltage')})
        self.assertEqual(transport.attempts, dict.fromkeys(transport.attempts, True))
        result = transport.stop_repeated()
        self.assertTrue(result['complete'], result)
        self.assertEqual((result['confirmed_ids'], result['ambiguous_ids'], result['faults']),
                         ([4, 5, 6], [], {'4': 0, '5': 0, '6': 0}))
        self.assertEqual({value.destination for value in self.seen}, set(group.ids))
        self.assertEqual([value.kind for value in self.seen][-3:], [4, 4, 4])
        self.assertFalse(self.cancelled()); self.assertEqual(self.cancels, 0)
        with self.assertRaisesRegex(RuntimeError, 'poisoned'):
            transport.output(wires, deadline_ns=d(20), check=lambda: None)
        self.assert_silent()

    def ready(self, transport):
        self.peer_loop(6)
        for mid in transport.group.ids:
            transport.enable(mid, deadline_ns=self.deadline(30))
            transport.zero_gain(mid, 0., deadline_ns=self.deadline(20))
        self.thread.join(timeout=1)
        return transport

    def test_native_mask_and_limits_back_the_python_whitelist(self):
        for wire, pattern in ((active.encode_motion(4, 0., 0., 0.), "outside this port's three-axis mask"),
                              (stop_wire(5), "outside this port's three-axis mask"),
                              (active.encode_motion(2, 0., 3.2, 0.), 'out-of-bounds'),
                              (active.encode_motion(2, .06, 1., 0.), 'out-of-bounds')):
            with self.subTest(wire=wire.hex()):
                transport = self.transport(Group('port3', (1, 2, 3)))
                with self.assertRaisesRegex(active.ExchangeError, pattern):
                    transport._native((wire,), self.deadline(), None)  # Bypass Python only in this test.
                self.assert_silent(); self.assertTrue(transport.poisoned)
                transport.close(); self.transports.remove(transport)
        self.assertFalse(self.cancelled())  # _native alone is below the abort scope.

    def test_python_whitelist_writes_nothing_and_cancels(self):
        transport = self.ready(self.transport(Group('port0', (7, 8, 9))))
        wires = (active.encode_motion(7, 0., 1., .1), active.encode_motion(9, 0., 1., .1),
                 active.encode_motion(8, 0., 1., .1))
        with self.assertRaisesRegex(ValueError, 'destination'):
            transport.output(wires, deadline_ns=self.deadline(20), check=lambda: None)
        self.assert_silent(); self.assertTrue(self.cancelled()); self.assertEqual(self.cancels, 1)
        self.assertTrue(transport.poisoned)

    def test_mode0_output_reply_rejected_cancels_and_stop_reports_ambiguous(self):
        group = Group('port1', (10, 11, 12))
        transport = self.ready(self.transport(group))
        wires = tuple(active.encode_motion(mid, 0., 1., .1) for mid in group.ids)
        self.peer_loop(3, mutate=lambda raw: reply(raw, type1_mode=0 if ATParser().feed(raw)[0].destination == 12 else 2))
        with self.assertRaisesRegex(active.ExchangeError, 'Unmatched/duplicate/fault/mode'):
            transport.output(wires, deadline_ns=self.deadline(20), check=lambda: None)
        self.assertTrue(self.cancelled()); self.assertEqual(self.cancels, 1)
        self.assertEqual(transport.journal[-1][0], 'output'); self.assertEqual(transport.journal[-1][1][0][2].received, 0)
        self.assertFalse(transport.attempts['positive_gain_sent'] is False)
        self.thread.join(timeout=1); self.peer_loop(9)
        result = transport.stop_repeated(total_budget_ns=900_000_000)
        self.assertFalse(result['complete']); self.assertTrue(result['physical_cutoff_required'])
        self.assertEqual((result['ambiguous_ids'], result['confirmed_ids']), ([12], [10, 11]))
        self.assertEqual(len(result['rounds']), 3)
        self.assertTrue(all(row['native_status'] == 1 for row in result['rounds']))
        self.assertEqual([value.kind for value in self.seen][-9:], [4]*9)

    def test_late_hold_reply_fails_prefix_and_cancels(self):
        group = Group('port3', (1, 2, 3))
        transport = self.ready(self.transport(group))
        wires = tuple(active.encode_motion(mid, 0., 1., .1) for mid in group.ids)
        self.peer_loop(3); transport.output(wires, deadline_ns=self.deadline(20), check=lambda: None)
        self.thread.join(timeout=1)
        self.peer_loop(3, mutate=lambda raw: None if ATParser().feed(raw)[0].destination == 2 else reply(raw))
        future = Future(); future.set_running_or_notify_cancel()
        with self.assertRaisesRegex(active.ExchangeError, 'deadline'):
            transport.hold_then_voltage(wires, 1, future, deadline_ns=self.deadline(20), check=lambda: None)
        self.assertIsInstance(future.exception(timeout=0), active.ExchangeError)
        self.assertTrue(self.cancelled())
        self.thread.join(timeout=1); self.peer_loop(9)
        result = transport.stop_repeated(total_budget_ns=900_000_000)
        self.assertEqual(result['ambiguous_ids'], [2]); self.assertFalse(result['complete'])

    def output_ready(self, group=Group('port3', (1, 2, 3)), **options):
        transport = self.ready(self.transport(group, **options))
        wires = tuple(active.encode_motion(mid, .01, 2., .1) for mid in group.ids)
        self.peer_loop(3); transport.output(wires, deadline_ns=self.deadline(20), check=lambda: None)
        self.thread.join(timeout=1)
        return transport, wires

    def test_loader_resolves_and_seals_the_optional_exchange_at(self):
        seal = T.verify_library(self.lib)
        self.assertEqual(len(seal.exchange_at), 2)
        self.assertEqual(self.lib.sda_subset_exchange_at_abi(), 1)
        self.assertEqual(self.lib.sda_subset_exchange_at.argtypes, list(T._AT_ARGS[1]))
        self.assertNotIn('sda_subset_exchange_at', T._SYMBOLS)
        original = self.lib.sda_subset_exchange_at
        try:
            self.lib.sda_subset_exchange_at = self.lib['sda_subset_exchange_at']
            with self.assertRaisesRegex(ValueError, 'function binding changed'):
                T.verify_library(self.lib)
        finally:
            self.lib.sda_subset_exchange_at = original
        T.verify_library(self.lib)

    def test_prearmed_hold_first_write_is_at_or_after_not_before(self):
        transport, wires = self.output_ready(prearmed_hold=True)
        for not_before in (3_000_000, None):
            with self.subTest(not_before=not_before):
                self.seen = []; self.peer_loop(4)
                future = Future(); future.set_running_or_notify_cancel()
                release = None if not_before is None else time.monotonic_ns()+not_before
                hold, voltage = transport.hold_then_voltage(wires, 2, future, deadline_ns=self.deadline(20),
                                                            check=lambda: None, not_before_ns=release)
                self.thread.join(timeout=1)
                self.assertIs(future.result(timeout=0), hold)
                if release is not None:
                    self.assertGreaterEqual(hold.stats.begin_ns, release)
                    self.assertGreaterEqual(hold.records[0].start_ns, release)
                    self.assertLess(voltage.records[0].start_ns-release, 20_000_000)
                self.assertEqual([value.wire for value in self.seen], list(wires)+[read_request(2, 'voltage')])
                self.assertEqual([bytes(record.tx) for record in hold.records], list(wires))
                self.assertEqual(set(voltage.rows), {(2, 'voltage')})
                self.assertEqual(transport.verify_batch(hold, 'feedback_hold'), hold.rows)
        self.assertFalse(transport.poisoned); self.assertEqual(self.cancels, 0); self.assertFalse(self.cancelled())

    def test_prearmed_cancel_during_wait_writes_nothing_and_fails_the_prefix(self):
        transport, wires = self.output_ready(prearmed_hold=True)
        timer = threading.Timer(.001, lambda: os.write(self.cancel_write, b'x')); timer.start()
        self.addCleanup(timer.cancel)
        future = Future(); future.set_running_or_notify_cancel()
        release = time.monotonic_ns()+4_500_000
        with self.assertRaisesRegex(active.ExchangeError, 'Cancelled'):
            transport.hold_then_voltage(wires, 1, future, deadline_ns=self.deadline(20), check=lambda: None,
                                        not_before_ns=release)
        self.assertLess(time.monotonic_ns(), release)  # Returned during the wait.
        self.assert_silent(); self.assertIsInstance(future.exception(timeout=0), active.ExchangeError)
        self.assertTrue(transport.poisoned); self.assertEqual(self.cancels, 1)
        label, (records, _) = transport.journal[-1]
        self.assertEqual((label, [record.written for record in records]), ('feedback_hold', [0, 0, 0]))
        self.assertTrue(transport.attempts['positive_gain_sent'])  # Set before the call, never reset.

    def test_prearmed_release_already_passed_is_rejected_without_a_write(self):
        transport, wires = self.output_ready(prearmed_hold=True)
        future = Future()
        with self.assertRaisesRegex(TimeoutError, 'ahead'):
            transport.hold_then_voltage(wires, 1, future, deadline_ns=self.deadline(20), check=lambda: None,
                                        not_before_ns=time.monotonic_ns()+1)
        self.assert_silent(); self.assertTrue(transport.poisoned); self.assertTrue(self.cancelled())

    def test_not_before_without_the_selection_is_rejected_without_a_write(self):
        default, wires = self.output_ready(Group('port1', (10, 11, 12)))
        with self.assertRaisesRegex(ValueError, 'explicit prearmed_hold selection'):
            default.hold_then_voltage(wires, 10, Future(), deadline_ns=self.deadline(20), check=lambda: None,
                                      not_before_ns=time.monotonic_ns()+2_000_000)
        self.assert_silent(); self.assertTrue(default.poisoned); self.assertTrue(self.cancelled())

    def test_decode_once_rows_are_reused_and_tamper_is_rejected(self):
        transport, wires = self.output_ready(decode_once=True)
        self.peer_loop(4)
        hold, voltage = transport.hold_then_voltage(wires, 3, Future(), deadline_ns=self.deadline(20),
                                                    check=lambda: None)
        self.thread.join(timeout=1)
        self.assertIs(type(hold.rows), MappingProxyType)
        self.assertEqual(hold.rows, decode_records((hold.records, hold.stats)))
        with patch.object(transport_adapter, 'decode_records', side_effect=AssertionError('re-decoded')):
            self.assertIs(transport.verify_batch(hold, 'feedback_hold'), hold.rows)
        self.assertEqual(transport.verify_batch(voltage, 'voltage'), decode_records((voltage.records, voltage.stats)))
        hold.stats.writes += 1
        with self.assertRaisesRegex(ValueError, 'raw batch'):
            transport.verify_batch(hold, 'feedback_hold')

    def test_library_without_exchange_at_keeps_the_default_path_and_rejects_prearming(self):
        source = build.EXTENSION.read_text()
        marker = 'extern "C" uint32_t sda_subset_exchange_at_abi'
        self.assertIn(marker, source)
        root = Path(self.directory.name)/f'legacy-{time.monotonic_ns()}'; root.mkdir()
        (root/'subset_active.cpp').write_text(source[:source.index(marker)])
        with patch.object(build, 'EXTENSION', root/'subset_active.cpp'):
            legacy = build.build(root/'build')
        with self.assertRaisesRegex(ValueError, 'library lacks it'):
            T.load_library(legacy, **self.pins_for(legacy))  # Receipt claims the export: inconsistent.
        record_path = legacy.parent/'build-record.json'
        record = json.loads(record_path.read_bytes()); del record['four_bus_subset_active']['exchange_at_abi']
        record_path.write_text(json.dumps(record))
        lib = T.load_library(legacy, **self.pins_for(legacy))
        self.assertEqual(T.verify_library(lib).exchange_at, ())
        self.assertNotIn('sda_subset_exchange_at', vars(lib))
        with self.assertRaisesRegex(ValueError, 'sda_subset_exchange_at'):
            self.transport(lib=lib, prearmed_hold=True)
        transport = self.transport(lib=lib); self.peer_loop(3)
        self.assertEqual(set(transport.stop(deadline_ns=self.deadline())), {4, 5, 6})
        self.thread.join(timeout=1)
        # A current binary whose receipt predates the key is also never pre-armed.
        current = Path(self.directory.name)/f'unrecorded-{time.monotonic_ns()}'
        shutil.copytree(self.library_path.parent, current)
        record = json.loads((current/'build-record.json').read_bytes())
        del record['four_bus_subset_active']['exchange_at_abi']
        (current/'build-record.json').write_text(json.dumps(record))
        path = current/build.LIBRARY_NAME
        lib = T.load_library(path, **self.pins_for(path))
        self.assertEqual(T.verify_library(lib).exchange_at, ())
        with self.assertRaisesRegex(ValueError, 'sda_subset_exchange_at'):
            self.transport(lib=lib, prearmed_hold=True)

    def test_cancel_from_sibling_blocks_exchange_but_not_stop(self):
        transport = self.ready(self.transport(Group('port2', (4, 5, 6))))
        os.write(self.cancel_write, b'x')
        with self.assertRaisesRegex(active.ExchangeError, 'Cancelled'):
            transport.stop(deadline_ns=self.deadline())
        self.assert_silent(); self.peer_loop(3)
        result = transport.stop_repeated()
        self.assertTrue(result['complete'], result)


if __name__ == '__main__': unittest.main()
