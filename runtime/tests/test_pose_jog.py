"""Offline, synthetic checks for a single bounded pose-adjustment jog.

These tests exercise wire targets and stop behavior. They do not establish
physical tracking, retention after reset, joint calibration, or stop latency.
"""
import contextlib
from dataclasses import replace
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw.rs05_joint_trial import jog_offset, main, run_trial
from test_rs05_joint_trial import FakeTransport, UID


POSITION_STEP_RAD = 25.14 / 65535
AMPLITUDE_RAD = math.radians(1)


class AuditedTransport(FakeTransport):
    def __init__(self, failure=None):
        super().__init__(failure)
        self.parameter_names = []
        self.timed_frames = []

    def parameter(self, name=None):
        self.parameter_names.append(name)
        return super().parameter(name)

    def send(self, wire):
        super().send(wire)
        self.timed_frames.append((self.clock(), self.frames[-1]))


def execute(transport, *, direction=1, interrupt=lambda: None, **kwargs):
    return run_trial(transport, UID, interrupt, lambda _: None,
                     clock=transport.clock, wait=transport.clock.wait,
                     gain_profile="step2", trajectory_kind="jog",
                     direction=direction, motor_id=transport.motor_id, **kwargs)


class JogTrajectoryTests(unittest.TestCase):
    def test_signed_ramp_finishes_at_two_seconds_and_holds_until_three(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.assertEqual(jog_offset(0, direction), 0)
                self.assertAlmostEqual(jog_offset(1, direction), direction * AMPLITUDE_RAD / 2)
                for elapsed in (2, 2.25, 2.5, 2.99, 3):
                    self.assertAlmostEqual(jog_offset(elapsed, direction), direction * AMPLITUDE_RAD)
                projected = [direction * jog_offset(i / 100, direction) for i in range(301)]
                self.assertTrue(all(0 <= value <= AMPLITUDE_RAD for value in projected))
                self.assertTrue(all(a <= b for a, b in zip(projected, projected[1:])))

    def test_jog_rejects_invalid_time_and_noninteger_direction(self):
        for elapsed in (-.0001, 3.0001, math.nan, math.inf, -math.inf):
            with self.subTest(elapsed=elapsed), self.assertRaises(ValueError):
                jog_offset(elapsed)
        for direction in (0, 2, -2, True, False, 1.0, -1.0, "1", None):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                jog_offset(1, direction)


class JogRunnerTests(unittest.TestCase):
    def test_nonzero_fresh_motor_center_is_not_replaced_by_zero_or_previous_target(self):
        class CenteredTransport(AuditedTransport):
            def __init__(self, center):
                super().__init__()
                self.center = center

            def parameter(self, name=None):
                value = super().parameter(name)
                return {"value": self.center} if name == "position" else value

            def feedback(self, wires, stopping=False):
                value, timestamp = super().feedback(wires, stopping=stopping)
                return replace(value, protocol_position_rad=self.center), timestamp

        for center, direction in ((1.25, 1), (-2.0, -1)):
            with self.subTest(center=center, direction=direction):
                transport = CenteredTransport(center)
                result = execute(transport, direction=direction)
                self.assertEqual(result["center_rad"], center)
                self.assertTrue(result["motion_completed"])
                targets = [struct.unpack(">4H", frame.data)[0] * POSITION_STEP_RAD - 12.57
                           for frame in transport.frames if frame.kind == 1]
                self.assertAlmostEqual(targets[0], center, delta=POSITION_STEP_RAD)
                self.assertAlmostEqual(targets[-1], center + direction * AMPLITUDE_RAD,
                                       delta=POSITION_STEP_RAD)
                self.assertEqual(transport.parameter_names.count("position"), 1)
                self.assertEqual(sum(frame.kind == 3 for frame in transport.frames), 1)

    def test_signed_wire_targets_are_bounded_and_hold_without_reenable(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                transport = AuditedTransport()
                transport.motor_id = 3
                result = execute(transport, direction=direction)
                self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
                self.assertEqual(result["trajectory_kind"], "jog")
                self.assertEqual(result["direction"], direction)
                self.assertAlmostEqual(result["target_final_offset_rad"], direction * AMPLITUDE_RAD)
                self.assertEqual(result["final_observed_delta_rad"], 0)
                self.assertAlmostEqual(result["final_tracking_error_rad"], -direction * AMPLITUDE_RAD)
                self.assertFalse(result["joint_calibration_verified"])
                self.assertTrue(result["stop_confirmed"])
                self.assertEqual({frame.destination for frame in transport.frames}, {3})
                self.assertEqual(sum(frame.kind == 3 for frame in transport.frames), 1)
                self.assertEqual(transport.frames[-1].kind, 4)
                self.assertEqual(transport.frames[-1].data, bytes(8))  # No fault clear.

                motion = [(when, frame, struct.unpack(">4H", frame.data))
                          for when, frame in transport.timed_frames if frame.kind == 1]
                self.assertEqual({words[2:] for _, _, words in motion}, {(0, 0), (655, 655)})
                active = [(when, frame, words) for when, frame, words in motion if words[2] != 0]
                self.assertGreaterEqual(len(active), 100)
                self.assertLessEqual(len(active), 151)
                # Actual protocol quantization is floor, so allow one position LSB.
                positions = [words[0] * POSITION_STEP_RAD - 12.57 for _, _, words in active]
                self.assertTrue(all(abs(value) <= AMPLITUDE_RAD + POSITION_STEP_RAD for value in positions))
                projected = [direction * value for value in positions]
                self.assertTrue(all(a <= b for a, b in zip(projected, projected[1:])))
                self.assertGreater(projected[-1], AMPLITUDE_RAD - POSITION_STEP_RAD)
                hold = [(when, words) for when, _, words in active
                        if when - active[0][0] >= 2.001]
                self.assertGreaterEqual(len(hold), 45)
                self.assertEqual(len({words[0] for _, words in hold}), 1)
                self.assertLess(active[-1][0] - active[0][0], 3)
                self.assertLess(transport.clock(), 14)
                self.assertEqual({words[1] for _, _, words in motion}, {32767})
                self.assertEqual({(frame.can_id >> 8) & 0xffff for _, frame, _ in motion}, {32767})

    def test_stationary_feedback_does_not_trigger_gain_escalation_or_extra_jog(self):
        transport = AuditedTransport()
        result = execute(transport)
        self.assertTrue(result["motion_completed"])
        self.assertEqual(result["peak_observed_delta_rad"], 0)
        self.assertEqual(sum(frame.kind == 3 for frame in transport.frames), 1)
        self.assertEqual(sum(frame.kind == 18 for frame in transport.frames), 1)
        self.assertEqual(transport.parameter_names.count("position"), 1)
        self.assertEqual({struct.unpack(">4H", frame.data)[2:]
                          for frame in transport.frames if frame.kind == 1}, {(0, 0), (655, 655)})
        self.assertTrue(transport.stopped_final)

    def test_failures_still_stop_and_never_claim_completed_target(self):
        for failure in ("communication", "overspeed", "stop"):
            with self.subTest(failure=failure):
                transport = AuditedTransport(failure)
                result = execute(transport, direction=-1)
                self.assertEqual(result["status"], "ABORTED")
                self.assertEqual(result["motion_completed"], failure == "stop")
                if failure != "stop":
                    self.assertNotIn("final_tracking_error_rad", result)
                self.assertEqual(transport.frames[-1].kind, 4)
                self.assertLessEqual(sum(frame.kind == 3 for frame in transport.frames), 1)
                self.assertEqual(result["stop_confirmed"], failure != "stop")

    def test_operator_interrupt_after_enable_stops_instead_of_completing_hold(self):
        transport = AuditedTransport()
        def interrupt():
            if transport.enabled:
                raise InterruptedError("operator interrupt")
        result = execute(transport, interrupt=interrupt)
        self.assertFalse(result["motion_completed"])
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(sum(frame.kind == 3 for frame in transport.frames), 1)
        self.assertEqual(transport.frames[-1].kind, 4)

    def test_invalid_combinations_fail_before_queries_or_commands(self):
        invalid = [
            ("initial", "jog", 1), ("visible", "jog", 1),
            ("step2", "jog", 0), ("step2", "jog", 2),
            ("step2", "jog", True), ("step2", "jog", 1.0),
            ("step2", "jog", None), ("step2", "unknown", 1),
            ("initial", "return", -1), ("step2", "return", -1),
            ("visible", "return", -1), ("initial", "return", True),
        ]
        for gain, trajectory, direction in invalid:
            with self.subTest(gain=gain, trajectory=trajectory, direction=direction):
                transport = AuditedTransport()
                with self.assertRaises(ValueError):
                    run_trial(transport, UID, lambda: None, lambda _: None,
                              clock=transport.clock, wait=transport.clock.wait,
                              gain_profile=gain, trajectory_kind=trajectory, direction=direction)
                self.assertFalse(transport.parameter_names)
                self.assertFalse(transport.frames)

    def test_default_return_keeps_identical_wire_trajectory(self):
        implicit, explicit = AuditedTransport(), AuditedTransport()
        default_result = run_trial(implicit, UID, lambda: None, lambda _: None,
                                   clock=implicit.clock, wait=implicit.clock.wait)
        explicit_result = run_trial(explicit, UID, lambda: None, lambda _: None,
                                    clock=explicit.clock, wait=explicit.clock.wait,
                                    trajectory_kind="return", direction=1)
        self.assertEqual([frame.wire for frame in implicit.frames],
                         [frame.wire for frame in explicit.frames])
        for result in (default_result, explicit_result):
            self.assertEqual(result["gain_profile"], "initial")
            self.assertEqual(result["trajectory_kind"], "return")
            self.assertEqual(result["direction"], 1)
            self.assertEqual(result["target_final_offset_rad"], 0)
            self.assertEqual(result["final_tracking_error_rad"], 0)


class JogCLITests(unittest.TestCase):
    def test_valid_jog_and_default_dry_runs_never_touch_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "capture"
            serial_constructor = Mock(side_effect=AssertionError("hardware must not open"))
            base = ["--expected-uid", UID, "--output", str(output)]
            for extra, expected in (([], ("return", "initial", 1)),
                    (["--trajectory-kind", "jog", "--gain-profile", "step2", "--direction", "-1"],
                     ("jog", "step2", -1))):
                text = io.StringIO()
                with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=serial_constructor)}), \
                        contextlib.redirect_stdout(text):
                    self.assertEqual(main(base + extra), 0)
                plan = json.loads(text.getvalue())
                self.assertEqual((plan["trajectory_kind"], plan["gain_profile"], plan["direction"]), expected)
                self.assertFalse(plan["automatic_gain_increase"])
                self.assertFalse(plan["automatic_reenable"])
                self.assertEqual(plan["trajectory_duration_s"], 3)
                self.assertFalse(output.exists())
            serial_constructor.assert_not_called()

    def test_invalid_cli_combinations_never_create_output_or_open_serial(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "capture"
            serial_constructor = Mock(side_effect=AssertionError("hardware must not open"))
            base = ["--expected-uid", UID, "--output", str(output), "--execute",
                    "--supported", "--operator-power-cut-ready"]
            for extra in (["--trajectory-kind", "jog"],
                    ["--trajectory-kind", "jog", "--gain-profile", "visible"],
                    ["--trajectory-kind", "return", "--direction", "-1"],
                    ["--trajectory-kind", "jog", "--gain-profile", "step2", "--direction", "0"]):
                with self.subTest(extra=extra), \
                        patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=serial_constructor)}), \
                        contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(base + extra)
                self.assertFalse(output.exists())
            serial_constructor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
