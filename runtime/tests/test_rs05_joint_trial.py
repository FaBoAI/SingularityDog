import contextlib
from dataclasses import replace
import io
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import Type2Feedback, TrialPhase, motion_request, stop_request
from singularitydog_hw.rs05_joint_trial import (run_trial, trajectory_offset, check_feedback,
                                               TrialTransport, main, MAX_DURATION_S)

UID = "0123456789abcdef"


def feedback(mode=2, **changes):
    return replace(Type2Feedback(mode, 0, 32767, 0., 0., 0., 25.), **changes)


class FakeClock:
    now = 10.
    def __call__(self):
        return self.now
    def wait(self, dt):
        self.now += dt


class FakeTransport:
    def __init__(self, failure=None):
        self.motor_id = 1
        self.clock = FakeClock()
        self.frames = []
        self.failure = failure
        self.watchdog = 0
        self.enabled = False
        self.stopped_final = False
    def parameter(self, name=None):
        values = {None: {"mcu_uid_hex": UID}, "run_mode": {"value": 0},
                  "position": {"value": 0}, "current": {"value": 0}, "voltage": {"value": 40},
                  "can_timeout": {"value": self.watchdog}}
        return values[name]
    def fresh_boundary(self):
        pass
    def send(self, wire):
        f = ATParser().feed(wire)[0]
        self.frames.append(f)
        if f.kind == 18:
            self.watchdog = 4000
    def feedback(self, wires, stopping=False):
        for wire in wires:
            self.send(wire)
        if stopping:
            self.stopped_final = True
            if self.failure == "stop":
                raise TimeoutError("stop disconnected")
            return feedback(0), self.clock()
        kinds = [ATParser().feed(w)[0].kind for w in wires]
        if 3 in kinds:
            self.enabled = True
        if self.enabled and self.failure == "communication":
            raise TimeoutError("lost feedback")
        if self.enabled and self.failure == "overspeed":
            return feedback(velocity_rad_s=1.), self.clock()
        self.clock.wait(.005)
        return feedback(2 if self.enabled else 0), self.clock()


class TrialTests(unittest.TestCase):
    def test_explicit_second_motor_never_commands_first_or_others(self):
        t = FakeTransport()
        t.motor_id = 2
        result = run_trial(t, UID, lambda: None, lambda _: None,
                           clock=t.clock, wait=t.clock.wait, motor_id=2)
        self.assertEqual(result["motor_id"], 2)
        self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
        self.assertEqual({f.destination for f in t.frames}, {2})
        self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)
        with self.assertRaises(ValueError):
            self.execute(t)

    def test_visible_profile_is_bounded_three_degrees(self):
        t = FakeTransport()
        result = run_trial(t, UID, lambda: None, lambda _: None,
                           clock=t.clock, wait=t.clock.wait, gain_profile="visible")
        self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
        words = [struct.unpack(">4H", f.data) for f in t.frames if f.kind == 1]
        self.assertEqual({v[2:] for v in words}, {(0, 0), (393, 1966)})
        self.assertLessEqual(max(v[0] for v in words)*25.14/65535-12.57, math.radians(3))
        self.assertAlmostEqual(trajectory_offset(MAX_DURATION_S/2, 3), math.radians(3))
        with self.assertRaises(RuntimeError):
            check_feedback(feedback(protocol_position_rad=math.radians(5.1)), 0, 0, 0,
                           max_drift_rad=math.radians(5))
        check_feedback(feedback(protocol_position_rad=math.radians(4)), 0, 0, 0,
                       max_drift_rad=math.radians(5))

    def test_step2_is_explicit_fixed_profile_and_never_reenables(self):
        t = FakeTransport()
        result = run_trial(t, UID, lambda: None, lambda _: None,
                           clock=t.clock, wait=t.clock.wait, gain_profile="step2")
        self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
        self.assertEqual(result["gain_profile"], "step2")
        words = [struct.unpack(">4H", f.data) for f in t.frames if f.kind == 1]
        self.assertEqual({v[2:] for v in words}, {(0, 0), (655, 655)})
        self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)

    def execute(self, transport, interrupt=lambda: None):
        return run_trial(transport, UID, interrupt, lambda _: None,
                         clock=transport.clock, wait=transport.clock.wait)

    def test_fixed_trajectory_does_not_increase_gains_if_stationary(self):
        t = FakeTransport()
        result = self.execute(t)
        self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
        self.assertEqual(result["peak_observed_delta_rad"], 0)
        self.assertTrue(t.stopped_final)
        self.assertEqual({f.destination for f in t.frames}, {1})
        self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)
        motion = [struct.unpack(">4H", f.data) for f in t.frames if f.kind == 1]
        self.assertEqual({v[2] for v in motion}, {0, 65})
        self.assertEqual({v[3] for v in motion}, {0, 262})
        self.assertLess(t.clock.now, 14)

    def test_communication_failure_or_guard_failure_still_stops(self):
        for failure in ("communication", "overspeed"):
            t = FakeTransport(failure)
            result = self.execute(t)
            self.assertEqual(result["status"], "ABORTED")
            self.assertTrue(result["stop_confirmed"])
            self.assertEqual(t.frames[-1].kind, 4)
            self.assertFalse(result["motion_completed"])

    def test_signal_after_enable_stops_without_reenable(self):
        t = FakeTransport()
        def interrupt():
            if t.enabled:
                raise InterruptedError("signal")
        result = self.execute(t, interrupt)
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["motion_completed"])
        self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)

    def test_stop_timeout_never_reports_completed_stop(self):
        t = FakeTransport("stop")
        result = self.execute(t)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(any("STOP_UNCONFIRMED" in e for e in result["errors"]))

    def test_uid_mismatch_never_sends_command(self):
        t = FakeTransport()
        result = run_trial(t, "f"*16, lambda: None, lambda _: None, clock=t.clock, wait=t.clock.wait)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(t.frames)

    def test_feedback_guards(self):
        for fb, when, now in ((feedback(mode=0), 0, 0), (feedback(fault_bits=1), 0, 0),
                             (feedback(protocol_position_rad=.06), 0, 0),
                             (feedback(velocity_rad_s=.6), 0, 0), (feedback(temperature_c=50), 0, 0),
                             (feedback(), 0, .101), (feedback(), 1, 0),
                             (feedback(protocol_position_rad=math.nan), 0, 0)):
            with self.assertRaises(RuntimeError):
                check_feedback(fb, 0, when, now)
        check_feedback(feedback(), 0, 10, 10.01)

    def test_trajectory_boundaries_and_peak(self):
        self.assertEqual(trajectory_offset(0), 0)
        self.assertEqual(trajectory_offset(MAX_DURATION_S), 0)
        self.assertAlmostEqual(trajectory_offset(MAX_DURATION_S / 2), math.radians(1))
        with self.assertRaises(ValueError):
            trajectory_offset(MAX_DURATION_S+.001)

    def test_cli_default_and_missing_physical_readiness_do_not_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/"capture"
            args = ["--expected-uid", UID, "--output", str(out)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            self.assertFalse(out.exists())
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args+["--execute", "--supported"])
            self.assertFalse(out.exists())


def feedback_wire(velocity=0., fault=0, mode=2):
    cid = (2 << 24) | (mode << 22) | (fault << 16) | (1 << 8) | 0xFD
    data = struct.pack(">4H", 32767, int((velocity+50)*65535/100), 32767, 250)
    return b"AT"+((cid<<3)|4).to_bytes(4,"big")+b"\x08"+data+b"\r\n"


class SerialFake:
    def __init__(self, rx=b""):
        self.rx, self.sent = rx, []
    @property
    def in_waiting(self):
        return len(self.rx)
    def read(self, count):
        data, self.rx = self.rx[:count], self.rx[count:]
        return data
    def write(self, data):
        self.sent.append(data)
        return len(data)


class TransportGuards(unittest.TestCase):
    def test_selected_transport_rejects_a_different_target(self):
        port = SerialFake()
        t = TrialTransport(port, lambda _: None, motor_id=2)
        with self.assertRaises(ValueError):
            t.send(motion_request(phase=TrialPhase.POSITION, center_rad=0, motor_id=1))
        self.assertFalse(port.sent)
        t.send(stop_request(phase=TrialPhase.STOP, motor_id=2))
        self.assertEqual(ATParser().feed(port.sent[0])[0].destination, 2)

    def test_type21_fault_is_not_hidden_by_healthy_feedback(self):
        cid = (21 << 24) | (1 << 8) | 0xFD
        wire = b"AT"+((cid<<3)|4).to_bytes(4,"big")+b"\x08"+bytes(8)+b"\r\n"
        t = TrialTransport(SerialFake(wire+feedback_wire()), lambda _: None)
        with self.assertRaisesRegex(RuntimeError, "Type21"):
            t.receive()

    def test_slow_logger_does_not_refresh_timestamp(self):
        clock = FakeClock()
        t = TrialTransport(SerialFake(feedback_wire()), lambda _: clock.wait(.2))
        t.feedback_guard = lambda fb, ts: check_feedback(fb, 0, ts, clock())
        with patch("singularitydog_hw.rs05_joint_trial.time.monotonic", clock), self.assertRaisesRegex(RuntimeError, "Stale"):
            t.receive()

    def test_signal_immediately_before_send_blocks_motion(self):
        port = SerialFake()
        def interrupt():
            raise InterruptedError()
        t = TrialTransport(port, lambda _: None, interrupt)
        with self.assertRaises(InterruptedError):
            t.send(motion_request(phase=TrialPhase.POSITION, center_rad=0))
        self.assertFalse(port.sent)
        t.send(stop_request(phase=TrialPhase.STOP))
        self.assertEqual(len(port.sent), 1)

    def test_invalid_boundary_sends_stop_but_cannot_confirm_it(self):
        port = SerialFake(feedback_wire(mode=0))
        t = TrialTransport(port, lambda _: None)
        with patch.object(t, "fresh_boundary", side_effect=RuntimeError("backlog")), self.assertRaisesRegex(RuntimeError, "boundary"):
            t.feedback([stop_request(phase=TrialPhase.STOP)], stopping=True)
        self.assertEqual(ATParser().feed(port.sent[0])[0].kind, 4)

    def test_unsafe_then_healthy_in_same_chunk_latches(self):
        for unsafe in (feedback_wire(velocity=1), feedback_wire(fault=1)):
            port = SerialFake(unsafe+feedback_wire())
            t = TrialTransport(port, lambda _: None)
            t.feedback_guard = lambda fb, ts: check_feedback(fb, 0, ts, ts)
            with self.assertRaises(RuntimeError):
                t.receive()
            self.assertFalse(port.sent)

    def test_pending_overspeed_blocks_new_command(self):
        port = SerialFake(feedback_wire(velocity=1))
        t = TrialTransport(port, lambda _: None)
        t.feedback_guard = lambda fb, ts: check_feedback(fb, 0, ts, ts)
        with self.assertRaises(RuntimeError):
            t.feedback([motion_request(phase=TrialPhase.POSITION, center_rad=0)])
        self.assertFalse(port.sent)

    def test_deadline_prevents_late_command(self):
        port = SerialFake()
        t = TrialTransport(port, lambda _: None)
        t.active_deadline = 1
        with patch("singularitydog_hw.rs05_joint_trial.time.monotonic", return_value=2), self.assertRaises(RuntimeError):
            t.send(motion_request(phase=TrialPhase.POSITION, center_rad=0))
        self.assertFalse(port.sent)


if __name__ == "__main__":
    unittest.main()
