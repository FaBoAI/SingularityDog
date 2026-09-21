"""Offline serial-fault injection for the shared three-axis transport."""
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_joint_trial import check_feedback
from singularitydog_hw.rs05_leg_trial import LegTrialTransport, main
from singularitydog_hw.rs05_trial_protocol import TrialPhase, Type2Feedback, motion_request
from test_rs05_joint_trial import FakeClock


def wire(cid, payload):
    return b"AT" + ((cid << 3) | 4).to_bytes(4, "big") + b"\x08" + payload + b"\r\n"


def feedback(mid, *, mode=2, fault=0, velocity=0):
    payload = struct.pack(">4H", 32767, int((velocity + 50) * 65535 / 100), 32767, 300)
    return wire((2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 0xFD, payload)


def motion(mid=1, *, active=False):
    return motion_request(phase=TrialPhase.POSITION_STEP5 if active else TrialPhase.ZERO_GAIN,
                          center_rad=0, motor_id=mid)


class Serial:
    def __init__(self, clock, responder=None, fail_write_id=None):
        self.clock, self.responder = clock, responder
        self.fail_write_id, self.rx, self.attempts, self.sent = fail_write_id, b"", [], []

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        self.clock.wait(.001)
        value, self.rx = self.rx[:size], self.rx[size:]
        return value

    def write(self, data):
        frame = ATParser().feed(data)[0]
        self.attempts.append(frame)
        if frame.destination == self.fail_write_id:
            raise IOError("injected write failure")
        self.sent.append(frame)
        self.rx += (self.responder(frame) if self.responder else
                    feedback(frame.destination, mode=0 if frame.kind == 4 else 2))
        return len(data)


class LegTransportTests(unittest.TestCase):
    def fixture(self, responder=None, fail_write_id=None, emit=lambda _: None):
        clock = FakeClock()
        port = Serial(clock, responder, fail_write_id)
        transport = LegTrialTransport(port, emit, ids=(1, 2, 3), wait=clock.wait)
        return clock, port, transport

    def test_sibling_fault_cannot_hide_behind_awaited_healthy_reply(self):
        fault21 = wire((21 << 24) | (2 << 8) | 0xFD, bytes(8))
        for bad in (fault21, feedback(2, fault=1), feedback(2, velocity=1)):
            with self.subTest(bad=bad.hex()):
                clock, port, t = self.fixture(lambda _: bad + feedback(1))
                t.feedback_guard = lambda value, received, mid: check_feedback(value, 0, received, clock())
                with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
                    with self.assertRaises(RuntimeError):
                        t.feedback_many([motion()], (1,))
                    self.assertIsNotNone(t.fault_latched)
                    count = len(port.sent)
                    with self.assertRaises(RuntimeError):
                        t.send(motion(3))
                    self.assertEqual(len(port.sent), count)

    def test_shared_parser_returns_all_three_fresh_replies(self):
        clock, port, t = self.fixture()
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            result = t.feedback_many([motion(i) for i in (1, 2, 3)], (1, 2, 3))
        self.assertEqual(set(result), {1, 2, 3})
        self.assertEqual(set(t.latest), {1, 2, 3})
        self.assertEqual([f.destination for f in port.sent], [1, 2, 3])

    def test_missing_sibling_reply_times_out_without_retries(self):
        clock, port, t = self.fixture(lambda f: b"" if f.destination == 2 else feedback(f.destination))
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            with self.assertRaises(TimeoutError):
                t.feedback_many([motion(i) for i in (1, 2, 3)], (1, 2, 3))
        self.assertEqual(len(port.sent), 3)

    def test_slow_logger_cannot_falsely_refresh_received_feedback(self):
        clock, port, t = self.fixture()
        t.emit = lambda event: clock.wait(.2) if event["kind"] == "can_rx_bytes" else None
        t.feedback_guard = lambda value, received, mid: check_feedback(value, 0, received, clock())
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock), self.assertRaisesRegex(RuntimeError, "Stale"):
            t.feedback_many([motion()], (1,))

    def test_active_deadline_blocks_motion_but_never_stop_burst(self):
        clock, port, t = self.fixture()
        t.active_deadline = clock() - .01
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            with self.assertRaises(RuntimeError):
                t.send(motion())
            stopped = t.stop_all((1, 2, 3))
        self.assertEqual([f.kind for f in port.sent], [4, 4, 4])
        self.assertTrue(all(v["confirmed"] for v in stopped.values()))

    def test_each_active_write_checks_freshness_after_previous_slow_log(self):
        clock, port, t = self.fixture()
        t.latest = {i: (Type2Feedback(2, 0, 32767, 0, 0, 0, 30), clock()) for i in (1, 2, 3)}
        def guard():
            for value, received in t.latest.values():
                check_feedback(value, 0, received, clock())
        t.pre_send_guard = guard
        t.emit = lambda event: clock.wait(.11) if event["kind"] == "can_tx" else None
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            t.send(motion(1, active=True))
            with self.assertRaisesRegex(RuntimeError, "Stale"):
                t.send(motion(2, active=True))
        self.assertEqual([f.destination for f in port.sent], [1])

    def test_poisoned_boundary_still_sends_all_stops_without_confirmation(self):
        clock, port, t = self.fixture()
        t.parser.buffer = bytearray(b"AT")
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            stopped = t.stop_all((1, 2, 3))
        self.assertEqual([f.destination for f in port.sent], [1, 2, 3])
        self.assertFalse(any(v["confirmed"] for v in stopped.values()))

    def test_first_stop_write_failure_does_not_prevent_remaining_attempts(self):
        clock, port, t = self.fixture(fail_write_id=1)
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            stopped = t.stop_all((1, 2, 3))
        self.assertEqual([f.destination for f in port.attempts], [1, 2, 3])
        self.assertFalse(stopped[1]["confirmed"])
        self.assertTrue(stopped[2]["confirmed"] and stopped[3]["confirmed"])

    def test_log_failure_and_operator_interrupt_cannot_prevent_stop_burst(self):
        clock, port, t = self.fixture(emit=Mock(side_effect=IOError("log failed")))
        t.check_interrupt = Mock(side_effect=InterruptedError("operator"))
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            stopped = t.stop_all((1, 2, 3))
        self.assertEqual([f.destination for f in port.sent], [1, 2, 3])
        self.assertTrue(all(v["confirmed"] for v in stopped.values()))
        t.check_interrupt.assert_not_called()

    def test_later_running_feedback_invalidates_earlier_reset_confirmation(self):
        clock, port, t = self.fixture(lambda f: feedback(f.destination, mode=0) +
                                    (feedback(2, mode=2) if f.destination == 2 else b""))
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            stopped = t.stop_all((1, 2, 3))
        self.assertFalse(stopped[2]["confirmed"])

    def test_stop_receive_failure_invalidates_earlier_confirmations(self):
        clock, port, t = self.fixture()
        original = t.receive
        called = []
        def broken_receive():
            if called:
                raise IOError("later read failed")
            called.append(True)
            return original()
        t.receive = broken_receive
        with patch("singularitydog_hw.rs05_leg_trial.time.monotonic", clock):
            stopped = t.stop_all((1, 2, 3))
        self.assertFalse(any(v["confirmed"] for v in stopped.values()))
        self.assertEqual(len(port.sent), 3)

    def test_other_leg_and_non_trial_writes_are_blocked(self):
        clock, port, t = self.fixture()
        bad = [motion(4), wire((4 << 24) | (0xFD << 8) | 1, b"\x01" + bytes(7)),
               wire((18 << 24) | (0xFD << 8) | 1, struct.pack("<H2xI", 0x7005, 0))]
        for command in bad:
            with self.assertRaises(ValueError):
                t.send(command)
        self.assertFalse(port.sent)


class LegCLITests(unittest.TestCase):
    def test_dry_run_and_missing_readiness_never_open_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            ids = Path(tmp) / "ids.json"
            ids.write_text(json.dumps({i: f"{i:016x}" for i in (1, 2, 3)}))
            output = Path(tmp) / "unused"
            serial = Mock(side_effect=AssertionError("must not open hardware"))
            args = ["--leg", "FR", "--directions", "-1", "-1", "1",
                    "--expected-uids", str(ids), "--output", str(output)]
            with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=serial)}):
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    self.assertEqual(main(args), 0)
                plan = json.loads(out.getvalue())
                self.assertEqual(plan["motor_ids"], [1, 2, 3])
                self.assertEqual(plan["target_offsets_deg"], [-5, -5, 5])
                self.assertEqual(plan["cycle_s"], .05)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(args + ["--execute", "--supported"])
                self.assertFalse(output.exists())
                serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
