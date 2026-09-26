"""Six-axis bus transport validation and fault injection without hardware I/O."""
import math
import struct
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw.can_readonly import ATParser, PARAMETERS, read_request
from singularitydog_hw.rs05_bus_transport import (
    BusTrialTransport, EXCHANGE_TIMEOUT_S, MIN_TX_INTERVAL_S)
from singularitydog_hw.rs05_joint_trial import check_feedback
from singularitydog_hw.rs05_trial_protocol import (
    TrialPhase, Type2Feedback, enable_request, motion_request, stop_request,
    watchdog_setup_request)
from test_rs05_joint_trial import FakeClock
from test_rs05_leg_pacing import TimedSerial
from test_rs05_leg_transport import Serial, feedback, motion, wire


BUSES = {'front': (1, 2, 3, 4, 5, 6), 'rear': (7, 8, 9, 10, 11, 12)}
CLOCK_PATCH = 'singularitydog_hw.rs05_bus_transport.time.monotonic'


def changed_wire(command, *, can_id=None, payload=None):
    frame = ATParser().feed(command)[0]
    return wire(frame.can_id if can_id is None else can_id,
                frame.data if payload is None else payload)


class BusFixture(unittest.TestCase):
    def fixture(self, bus='front', *, responder=None, fail_write_id=None,
                emit=lambda _: None, timed=False, **kwargs):
        clock = FakeClock()
        if timed:
            port = TimedSerial(clock, **kwargs)
            if responder is not None:
                port.responder = responder
        else:
            port = Serial(clock, responder, fail_write_id)
        transport = BusTrialTransport(port, emit, ids=BUSES[bus], wait=clock.wait)
        return clock, port, transport

    def assert_gaps(self, port):
        for before, after in zip(port.finishes, port.starts[1:]):
            self.assertGreaterEqual(after - before, MIN_TX_INTERVAL_S - 1e-12)


class BusTransportValidationTests(BusFixture):
    def test_front_hip_kp12_six_axis_batch_passes_before_any_real_write(self):
        clock, port, transport = self.fixture('front')
        commands = [motion_request(
            phase=(TrialPhase.POSITION_ROLE_FRONT_HIP_KP12 if mid in (3, 6)
                   else TrialPhase.POSITION_STEP5_KP4 if mid == 4
                   else TrialPhase.POSITION_STEP5),
            center_rad=0., offset_rad=(math.radians(1.) if mid == 3 else
                                      -math.radians(1.) if mid == 6 else 0.),
            motor_id=mid) for mid in BUSES['front']]
        with patch(CLOCK_PATCH, clock):
            found = transport.feedback_many(commands, BUSES['front'])
        self.assertEqual(set(found), set(BUSES['front']))
        self.assertEqual([frame.destination for frame in port.sent], list(BUSES['front']))
        self.assertEqual(len(port.attempts), 6)

    def test_four_upper_leg_kp12_and_four_hip_hold_kp12_frames_pass_validation(self):
        for bus, ids in (('front', (2, 5)), ('rear', (8, 11))):
            _, port, transport = self.fixture(bus)
            for mid in ids:
                command = motion_request(phase=TrialPhase.POSITION_ROLE_THIGH_KP12,
                                         center_rad=0., offset_rad=.1, motor_id=mid)
                frame = transport._validated_wire(command)
                self.assertEqual((frame.kind, frame.destination), (1, mid))
            hips = (3, 6) if bus == 'front' else (9, 12)
            for mid in hips:
                command = motion_request(phase=TrialPhase.POSITION_ROLE_HIP_HOLD_KP12,
                                         center_rad=0., motor_id=mid)
                frame = transport._validated_wire(command)
                self.assertEqual((frame.kind, frame.destination), (1, mid))
            self.assertFalse(port.attempts)

    def test_only_exact_ordered_front_and_rear_buses_are_constructed(self):
        for name, ids in BUSES.items():
            with self.subTest(bus=name):
                _, _, transport = self.fixture(name)
                self.assertEqual(transport.ids, ids)
                self.assertEqual(transport.bus_name, name)
        invalid = ((), (1, 2, 3), tuple(range(1, 13)), (1, 2, 3, 4, 5, 7),
                   (6, 5, 4, 3, 2, 1), (7, 8, 9, 10, 12, 11),
                   (1, 2, 3, 4, 5, 5), (True, 2, 3, 4, 5, 6),
                   (1., 2, 3, 4, 5, 6), ('1', 2, 3, 4, 5, 6))
        for ids in invalid:
            clock = FakeClock(); port = Serial(clock)
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                BusTrialTransport(port, lambda _: None, ids=ids, wait=clock.wait)
            self.assertFalse(port.attempts)

    def test_shared_parser_returns_all_six_fresh_replies_on_either_bus(self):
        for name, ids in BUSES.items():
            clock, port, transport = self.fixture(name)
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                result = transport.feedback_many([motion(mid) for mid in ids], ids)
            self.assertEqual(set(result), set(ids))
            self.assertEqual(set(transport.latest), set(ids))
            self.assertEqual([f.destination for f in port.sent], list(ids))
            self.assertTrue(all(isinstance(fb, Type2Feedback) for fb, _ in result.values()))
            self.assertTrue(all(0 <= clock() - when <= .1 for _, when in result.values()))

    def test_identity_and_parameter_reads_use_selected_bus_ids(self):
        def responder(frame):
            mid = frame.destination
            if frame.kind == 0:
                return wire((mid << 8) | 0xFE, mid.to_bytes(8, 'big'))
            return wire((17 << 24) | (mid << 8) | 0xFD,
                        frame.data[:4] + struct.pack('<f', 1.25))

        for name, ids in BUSES.items():
            clock, _, transport = self.fixture(name, responder=responder)
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                for mid in ids:
                    result = transport.parameter(mid)
                    self.assertEqual(result['mcu_uid_hex'], f'{mid:016x}')
                    self.assertTrue(result['ok'])
                result = transport.parameter(ids[-1], 'position')
                self.assertEqual(result['value'], 1.25)
                self.assertEqual(result['motor_id'], ids[-1])

    def test_other_bus_command_and_invalid_subsets_reject_before_writes(self):
        for name, ids in BUSES.items():
            other = 7 if name == 'front' else 1
            for operation in ('send', 'parameter', 'batch'):
                clock, port, transport = self.fixture(name)
                with self.subTest(bus=name, operation=operation), patch(CLOCK_PATCH, clock):
                    with self.assertRaises(ValueError):
                        if operation == 'send':
                            transport.send(motion(other))
                        elif operation == 'parameter':
                            transport.parameter(other)
                        else:
                            transport.feedback_many([motion(ids[0]), motion(other)], (ids[0],))
                self.assertFalse(port.attempts)
            for subset in ((), (other,), (ids[0], other), (ids[0], ids[0]),
                           (float(ids[0]),), (str(ids[0]),), (True,)):
                for operation in ('exchange', 'stop'):
                    clock, port, transport = self.fixture(name)
                    with self.subTest(bus=name, subset=subset, operation=operation), patch(CLOCK_PATCH, clock):
                        with self.assertRaises(ValueError):
                            if operation == 'exchange':
                                transport.exchange_many([motion(ids[0])], subset, lambda _: None)
                            else:
                                transport.stop_all(subset)
                    self.assertFalse(port.attempts)

    def test_stop_defaults_to_all_six_and_accepts_unique_same_bus_subset(self):
        for name, ids in BUSES.items():
            for subset in (None, (ids[0], ids[-1])):
                clock, port, transport = self.fixture(name)
                with self.subTest(bus=name, subset=subset), patch(CLOCK_PATCH, clock):
                    reports = transport.stop_all() if subset is None else transport.stop_all(subset)
                selected = ids if subset is None else subset
                self.assertEqual(tuple(reports), selected)
                self.assertEqual([f.destination for f in port.sent], list(selected))
                self.assertTrue(all(report['confirmed'] for report in reports.values()))


class CanonicalFrameTests(BusFixture):
    def test_all_allowed_frame_classes_and_fixed_gain_phases_are_canonical(self):
        for name, ids in BUSES.items():
            clock, port, transport = self.fixture(name)
            mid = ids[0]
            commands = [read_request(mid),
                        enable_request(phase=TrialPhase.ENABLE, motor_id=mid),
                        stop_request(phase=TrialPhase.STOP, motor_id=mid),
                        watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid)]
            commands.extend(read_request(mid, name) for name in PARAMETERS)
            phases = (TrialPhase.ZERO_GAIN, TrialPhase.POSITION, TrialPhase.POSITION_STEP2,
                      TrialPhase.POSITION_VISIBLE, TrialPhase.POSITION_STEP5,
                      TrialPhase.POSITION_STEP5_KP4)
            commands.extend(motion_request(phase=phase, center_rad=0., motor_id=mid)
                            for phase in phases)
            if name == 'rear':
                commands.append(motion_request(phase=TrialPhase.POSITION_STEP5_RR_HIP_KP6,
                                               center_rad=0., motor_id=9))
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                for command in commands:
                    transport.send(command)
            self.assertEqual(len(port.sent), len(commands))
            self.assertEqual({f.kind for f in port.sent}, {0, 1, 3, 4, 17, 18})

    def test_noncanonical_identity_enable_stop_read_and_watchdog_frames_reject(self):
        mid = 1
        bad = []
        for command in (read_request(mid), enable_request(phase=TrialPhase.ENABLE, motor_id=mid),
                        stop_request(phase=TrialPhase.STOP, motor_id=mid),
                        read_request(mid, 'position'),
                        watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid)):
            frame = ATParser().feed(command)[0]
            bad.append(changed_wire(command, can_id=(frame.can_id & ~0xFF00) | (0xFC << 8)))
            bad.append(changed_wire(command, payload=frame.data[:-1] + b'\x01'))
        bad.extend((wire((17 << 24) | (0xFD << 8) | mid, struct.pack('<H2xI', 0x1234, 0)),
                    wire((17 << 24) | (0xFD << 8) | mid, struct.pack('<HBBI', 0x7019, 1, 0, 0)),
                    wire((18 << 24) | (0xFD << 8) | mid, struct.pack('<H2xI', 0x7028, 4001)),
                    wire((18 << 24) | (0xFD << 8) | mid, struct.pack('<H2xI', 0x7005, 0)),
                    wire((6 << 24) | (0xFD << 8) | mid, bytes(8))))
        for command in bad:
            clock, port, transport = self.fixture()
            with self.subTest(command=command.hex()), patch(CLOCK_PATCH, clock):
                with self.assertRaises(ValueError):
                    transport.send(command)
            self.assertFalse(port.attempts)

    def test_type1_rejects_nonzero_feedforward_velocity_and_arbitrary_gain_pairs(self):
        canonical = motion(1, active=True)
        frame = ATParser().feed(canonical)[0]
        position, velocity, kp, kd = struct.unpack('>4H', frame.data)
        bad = [changed_wire(canonical, can_id=frame.can_id + (1 << 8)),
               changed_wire(canonical, payload=struct.pack('>4H', position, velocity + 1, kp, kd)),
               changed_wire(canonical, payload=struct.pack('>4H', position, velocity, kp + 1, kd)),
               changed_wire(canonical, payload=struct.pack('>4H', position, velocity, kp, kd + 1)),
               changed_wire(canonical, payload=struct.pack('>4H', position, velocity, 0, kd))]
        hip6 = motion_request(phase=TrialPhase.POSITION_STEP5_RR_HIP_KP6, center_rad=0., motor_id=9)
        hip_frame = ATParser().feed(hip6)[0]
        bad.append(changed_wire(hip6, can_id=(hip_frame.can_id & ~255) | 1))
        for command in bad:
            clock, port, transport = self.fixture()
            with self.subTest(command=command.hex()), patch(CLOCK_PATCH, clock):
                with self.assertRaises(ValueError):
                    transport.send(command)
            self.assertFalse(port.attempts)

    def test_noncanonical_at_framing_is_rejected_before_write(self):
        command = motion(1)
        bad_flags = bytearray(command)
        bad_flags[5] &= ~7
        for bad in (b'garbage' + command, command + b'AT', command + command,
                    command[:-1], bytes(bad_flags), command[:6] + b'\x07' + command[7:]):
            clock, port, transport = self.fixture()
            with self.subTest(command=bad.hex()), patch(CLOCK_PATCH, clock):
                with self.assertRaises(ValueError):
                    transport.send(bad)
            self.assertFalse(port.attempts)


class BusTransportFaultTests(BusFixture):
    def test_sibling_type21_or_feedback_fault_latches_and_still_allows_all_six_stops(self):
        for name, ids in BUSES.items():
            first, sibling = ids[0], ids[-1]
            fault21 = wire((21 << 24) | (sibling << 8) | 0xFD, bytes(8))
            for bad in (fault21, feedback(sibling, fault=1), feedback(sibling, velocity=1)):
                def responder(frame):
                    if frame.kind == 4:
                        return feedback(frame.destination, mode=0)
                    return bad + feedback(first)

                clock, port, transport = self.fixture(name, responder=responder)
                transport.feedback_guard = lambda value, when, mid: check_feedback(value, 0., when, clock())
                with self.subTest(bus=name, bad=bad.hex()), patch(CLOCK_PATCH, clock):
                    with self.assertRaises(RuntimeError):
                        transport.feedback_many([motion(first)], (first,))
                    self.assertIsNotNone(transport.fault_latched)
                    count = len(port.attempts)
                    with self.assertRaises(RuntimeError):
                        transport.send(motion(sibling))
                    self.assertEqual(len(port.attempts), count)
                    reports = transport.stop_all()
                self.assertEqual([f.destination for f in port.sent if f.kind == 4], list(ids))
                self.assertTrue(all(report['confirmed'] for report in reports.values()))

    def test_slow_receive_logger_cannot_refresh_received_timestamps(self):
        for event_kind in ('can_rx_bytes', 'can_rx_frame'):
            clock, _, transport = self.fixture()
            transport.emit = lambda event: clock.wait(.11) if event['kind'] == event_kind else None
            transport.feedback_guard = lambda value, when, mid: check_feedback(value, 0., when, clock())
            with self.subTest(event_kind=event_kind), patch(CLOCK_PATCH, clock):
                with self.assertRaises((RuntimeError, TimeoutError)):
                    transport.feedback_many([motion(1)], (1,))

    def test_feedback_summary_logging_cannot_return_stale_feedback_without_guard(self):
        clock, _, transport = self.fixture()
        transport.emit = lambda event: clock.wait(.11) if event['kind'] == 'bus_trial_feedback' else None
        with patch(CLOCK_PATCH, clock), self.assertRaisesRegex(RuntimeError, 'Stale'):
            transport.feedback_many([motion(1)], (1,))
        self.assertIsNotNone(transport.fault_latched)

    def test_other_bus_replies_latch_and_invalidate_stop_confirmation(self):
        for name, ids in BUSES.items():
            other = 7 if name == 'front' else 1
            replies = (wire((other << 8) | 0xFE, bytes(8)), feedback(other),
                       wire((17 << 24) | (other << 8) | 0xFD, struct.pack('<H2xf', 0x7019, 0.)),
                       wire((21 << 24) | (other << 8) | 0xFD, bytes(8)))
            for bad in replies:
                clock, port, transport = self.fixture(
                    name, responder=lambda frame: feedback(frame.destination, mode=0) + bad)
                with self.subTest(bus=name, reply=bad.hex()), patch(CLOCK_PATCH, clock):
                    with self.assertRaisesRegex(RuntimeError, 'another bus'):
                        transport.feedback_many([motion(ids[0])], (ids[0],))
                    self.assertIsNotNone(transport.fault_latched)
                    reports = transport.stop_all()
                self.assertEqual([f.destination for f in port.sent if f.kind == 4], list(ids))
                self.assertFalse(any(report['confirmed'] for report in reports.values()))

    def test_stop_reply_aged_by_slow_logger_is_not_confirmed(self):
        clock, port, transport = self.fixture()
        transport.emit = lambda event: clock.wait(.11) if event['kind'] == 'can_rx_bytes' else None
        with patch(CLOCK_PATCH, clock):
            reports = transport.stop_all()
        self.assertEqual([f.destination for f in port.sent], list(BUSES['front']))
        self.assertFalse(any(report['confirmed'] for report in reports.values()))

    def test_each_active_write_rechecks_sibling_freshness_after_slow_log(self):
        clock, port, transport = self.fixture()
        latest = {mid: (Type2Feedback(2, 0, 32767, 0., 0., 0., 30.), clock())
                  for mid in transport.ids}

        def guard():
            for value, when in latest.values():
                check_feedback(value, 0., when, clock())

        transport.pre_send_guard = guard
        transport.emit = lambda event: clock.wait(.11) if event['kind'] == 'can_tx' else None
        with patch(CLOCK_PATCH, clock):
            transport.send(motion(1, active=True))
            with self.assertRaisesRegex(RuntimeError, 'Stale'):
                transport.send(motion(6, active=True))
        self.assertEqual([f.destination for f in port.sent], [1])

    def test_poisoned_boundary_attempts_all_six_stops_but_confirms_none(self):
        for poison in ('partial', 'discarded'):
            clock, port, transport = self.fixture()
            if poison == 'partial':
                transport.parser.buffer = bytearray(b'AT')
            else:
                transport.parser.discarded_bytes = 1
            with self.subTest(poison=poison), patch(CLOCK_PATCH, clock):
                reports = transport.stop_all()
            self.assertEqual([f.destination for f in port.attempts], list(BUSES['front']))
            self.assertFalse(any(report['confirmed'] for report in reports.values()))

    def test_first_stop_failure_does_not_prevent_other_five_attempts(self):
        for name, ids in BUSES.items():
            clock, port, transport = self.fixture(name, fail_write_id=ids[0])
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                reports = transport.stop_all()
            self.assertEqual([f.destination for f in port.attempts], list(ids))
            self.assertFalse(reports[ids[0]]['confirmed'])
            self.assertTrue(all(reports[mid]['confirmed'] for mid in ids[1:]))

    def test_expired_deadline_interrupt_and_failed_log_never_prevent_stop_burst(self):
        clock, port, transport = self.fixture(emit=Mock(side_effect=IOError('log failed')))
        transport.active_deadline = clock() - .01
        transport.check_interrupt = Mock(side_effect=InterruptedError('operator'))
        with patch(CLOCK_PATCH, clock):
            reports = transport.stop_all()
        self.assertEqual([f.destination for f in port.sent], list(BUSES['front']))
        self.assertTrue(all(report['confirmed'] for report in reports.values()))
        transport.check_interrupt.assert_not_called()

    def test_later_running_feedback_or_receive_error_invalidates_stop_confirmation(self):
        clock, _, transport = self.fixture(
            responder=lambda f: feedback(f.destination, mode=0) +
            (feedback(6, mode=2) if f.destination == 6 else b''))
        with patch(CLOCK_PATCH, clock):
            reports = transport.stop_all()
        self.assertFalse(reports[6]['confirmed'])
        clock, port, transport = self.fixture()
        original_receive = transport.receive
        calls = []

        def broken_receive():
            if calls:
                raise IOError('later read failed')
            calls.append(True)
            return original_receive()

        transport.receive = broken_receive
        with patch(CLOCK_PATCH, clock):
            reports = transport.stop_all()
        self.assertEqual(len(port.attempts), 6)
        self.assertFalse(any(report['confirmed'] for report in reports.values()))


class BusTransportPacingTests(BusFixture):
    def test_every_frame_class_waits_five_ms_after_previous_completed_write(self):
        clock, port, transport = self.fixture(timed=True)
        commands = [read_request(1), read_request(2, 'position'),
                    watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=3),
                    enable_request(phase=TrialPhase.ENABLE, motor_id=4), motion(5, active=True),
                    stop_request(phase=TrialPhase.STOP, motor_id=6)]
        with patch(CLOCK_PATCH, clock):
            for command in commands:
                transport.send(command)
        self.assertEqual(MIN_TX_INTERVAL_S, .005)
        self.assertEqual(len(port.starts), 6)
        self.assert_gaps(port)
        self.assertAlmostEqual(port.starts[1] - port.starts[0], .008)

    def test_early_wakeup_does_not_skip_remaining_pacing_wait(self):
        clock, port, transport = self.fixture(timed=True)
        waits = []

        def early_once(seconds):
            waits.append(seconds)
            clock.wait(seconds / 2 if len(waits) == 1 else seconds)

        transport.wait = early_once
        with patch(CLOCK_PATCH, clock):
            transport.send(read_request(1)); transport.send(read_request(2))
        self.assertEqual(len(waits), 2)
        self.assert_gaps(port)

    def test_failed_or_partial_first_write_still_spaces_all_six_stop_attempts(self):
        for name, ids in BUSES.items():
            for failure in ('exception', 'partial'):
                clock, port, transport = self.fixture(name, timed=True, failure=failure)
                with self.subTest(bus=name, failure=failure), patch(CLOCK_PATCH, clock):
                    reports = transport.stop_all()
                self.assertEqual(len(port.starts), 6)
                self.assert_gaps(port)
                self.assertGreaterEqual(port.read_starts[0], port.finishes[-1])
                self.assertFalse(reports[ids[0]]['confirmed'])
                self.assertTrue(all(reports[mid]['confirmed'] for mid in ids[1:]))

    def test_failed_or_partial_send_latches_until_all_six_paced_stops(self):
        for failure in ('exception', 'partial'):
            clock, port, transport = self.fixture(timed=True, failure=failure)
            with self.subTest(failure=failure), patch(CLOCK_PATCH, clock):
                with self.assertRaises(IOError):
                    transport.send(motion(1))
                self.assertIsNotNone(transport.fault_latched)
                with self.assertRaises(RuntimeError):
                    transport.send(motion(2))
                self.assertEqual(len(port.starts), 1)
                reports = transport.stop_all()
            self.assertEqual(len(port.starts), 7)
            self.assert_gaps(port)
            self.assertEqual([f.destination for f in port.sent if f.kind == 4], list(BUSES['front']))
            self.assertTrue(all(report['confirmed'] for report in reports.values()))

    def test_deadline_and_freshness_are_checked_again_after_pacing(self):
        for cause in ('deadline', 'freshness'):
            for command in (enable_request(phase=TrialPhase.ENABLE, motor_id=6), motion(6, active=True)):
                clock, port, transport = self.fixture(timed=True)
                with self.subTest(cause=cause, command=command.hex()), patch(CLOCK_PATCH, clock):
                    transport.send(read_request(1))
                    if cause == 'deadline':
                        transport.active_deadline = clock() + .004
                    else:
                        when = clock() - .098

                        def guard():
                            check_feedback(Type2Feedback(2, 0, 32767, 0., 0., 0., 30.), 0., when, clock())

                        transport.pre_enable_guard = transport.pre_send_guard = guard
                    with self.assertRaises(RuntimeError):
                        transport.send(command)
                self.assertEqual(len(port.starts), 1)

    def test_missing_sibling_reply_times_out_eighty_ms_after_all_sends_without_retry(self):
        for name, ids in BUSES.items():
            clock, port, transport = self.fixture(
                name, timed=True,
                responder=lambda frame: b'' if frame.destination == ids[-1] else feedback(frame.destination))
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                with self.assertRaises(TimeoutError) as caught:
                    transport.feedback_many([motion(mid) for mid in ids], ids)
            error = caught.exception
            self.assertIs(type(error), TimeoutError)
            diagnostics = error.diagnostics
            self.assertEqual(diagnostics['bus'], name)
            self.assertEqual(diagnostics['deadline_source'], 'reply_timeout')
            self.assertEqual(diagnostics['received_ids'], list(ids[:-1]))
            self.assertEqual(diagnostics['missing_ids'], [ids[-1]])
            self.assertAlmostEqual(diagnostics['batch_completed_monotonic_s'], port.finishes[-1])
            self.assertAlmostEqual(diagnostics['effective_reply_budget_s'], .08)
            self.assertAlmostEqual(diagnostics['effective_deadline_monotonic_s'], port.finishes[-1] + .08)
            self.assertAlmostEqual(diagnostics['timed_out_monotonic_s'], clock())
            last_received = max(when for _, when in transport.latest.values())
            self.assertAlmostEqual(diagnostics['quiet_elapsed_s'], clock() - last_received)
            self.assertEqual(diagnostics['quiet_required_s'], .004)
            self.assertIn('reply_timeout', str(error))
            self.assertIn(str(list(ids[:-1])), str(error))
            self.assertIn(str([ids[-1]]), str(error))
            self.assertEqual(EXCHANGE_TIMEOUT_S, .08)
            self.assertEqual([f.destination for f in port.sent], list(ids))
            self.assertEqual(len(port.starts), 6)
            self.assert_gaps(port)
            self.assertGreaterEqual(clock() - port.finishes[-1], .08 - 1e-12)
            self.assertLess(clock() - port.finishes[-1], .082)
            self.assertIsNotNone(transport.fault_latched)
            with patch(CLOCK_PATCH, clock), self.assertRaises(RuntimeError):
                transport.send(motion(ids[0]))
            self.assertEqual(len(port.starts), 6)

    def test_active_deadline_reports_all_replies_when_quiet_interval_is_incomplete(self):
        for name, ids in BUSES.items():
            clock, port, transport = self.fixture(name, timed=True)
            # Six 3ms writes and five 5ms gaps leave just3ms for replies.
            transport.active_deadline = clock() + 6 * .003 + 5 * .005 + .003
            with self.subTest(bus=name), patch(CLOCK_PATCH, clock):
                with self.assertRaises(TimeoutError) as caught:
                    transport.feedback_many([motion(mid) for mid in ids], ids)
            error = caught.exception
            self.assertIs(type(error), TimeoutError)
            diagnostics = error.diagnostics
            self.assertEqual(diagnostics['bus'], name)
            self.assertEqual(diagnostics['deadline_source'], 'active_deadline')
            self.assertEqual(diagnostics['received_ids'], list(ids))
            self.assertEqual(diagnostics['missing_ids'], [])
            self.assertEqual(set(transport.latest), set(ids))
            self.assertAlmostEqual(diagnostics['batch_completed_monotonic_s'], port.finishes[-1])
            self.assertAlmostEqual(diagnostics['effective_reply_budget_s'], .003)
            self.assertEqual(diagnostics['effective_deadline_monotonic_s'], transport.active_deadline)
            self.assertAlmostEqual(diagnostics['timed_out_monotonic_s'], clock())
            self.assertGreaterEqual(clock(), transport.active_deadline)
            self.assertLessEqual(clock() - transport.active_deadline, .001 + 1e-12)
            self.assertEqual(diagnostics['quiet_required_s'], .004)
            self.assertGreaterEqual(diagnostics['quiet_elapsed_s'], 0.)
            self.assertLess(diagnostics['quiet_elapsed_s'], diagnostics['quiet_required_s'])
            self.assertIn('active_deadline', str(error))
            self.assertIn(str(list(ids)), str(error))
            self.assertIn('[]', str(error))
            self.assertEqual([frame.destination for frame in port.sent], list(ids))
            self.assertEqual(len(port.starts), 6)
            self.assert_gaps(port)
            self.assertIsNotNone(transport.fault_latched)

    def test_no_replies_timeout_reports_no_quiet_start_and_all_ids_missing(self):
        clock, port, transport = self.fixture(timed=True, responder=lambda _: b'')
        with patch(CLOCK_PATCH, clock), self.assertRaises(TimeoutError) as caught:
            transport.feedback_many([motion(mid) for mid in transport.ids], transport.ids)
        diagnostics = caught.exception.diagnostics
        self.assertEqual(diagnostics['deadline_source'], 'reply_timeout')
        self.assertEqual(diagnostics['received_ids'], [])
        self.assertEqual(diagnostics['missing_ids'], list(transport.ids))
        self.assertIsNone(diagnostics['quiet_elapsed_s'])
        self.assertEqual(diagnostics['quiet_required_s'], .004)
        self.assertAlmostEqual(diagnostics['effective_reply_budget_s'], .08)
        self.assertEqual(len(port.starts), 6)
        self.assert_gaps(port)


if __name__ == '__main__':
    unittest.main()
