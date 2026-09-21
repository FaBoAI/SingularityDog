"""Offline bounds/failure checks; no proof of physical tracking or stop latency."""
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

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_joint_trial import main, run_trial, step5_jog_offset
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from test_pose_jog import AuditedTransport, POSITION_STEP_RAD
from test_rs05_joint_trial import UID


class CenteredTransport(AuditedTransport):
    def __init__(self, center=5.5, failure=None):
        super().__init__(failure)
        self.center = center
        self.enable_time = None

    def parameter(self, name=None):
        value = super().parameter(name)
        return {"value": self.center} if name == "position" else value

    def feedback(self, wires, stopping=False):
        if any(ATParser().feed(w)[0].kind == 3 for w in wires):
            self.enable_time = self.clock()
        value, timestamp = super().feedback(wires, stopping=stopping)
        return replace(value, protocol_position_rad=self.center), timestamp


def execute(t, **kwargs):
    return run_trial(t, UID, kwargs.pop("interrupt", lambda: None), lambda _: None,
                     clock=t.clock, wait=t.clock.wait, gain_profile="step5",
                     trajectory_kind="jog", **kwargs)


class Step5Tests(unittest.TestCase):
    def test_fixed_signed_four_second_ramp_then_one_second_hold(self):
        for direction in (-1, 1):
            self.assertEqual(step5_jog_offset(0, direction), 0)
            self.assertAlmostEqual(math.degrees(step5_jog_offset(2, direction)), direction * 2.5)
            values = [direction * step5_jog_offset(i / 100, direction) for i in range(501)]
            self.assertTrue(all(0 <= v <= math.radians(5) for v in values))
            self.assertTrue(all(a <= b for a, b in zip(values, values[1:])))
            self.assertEqual(len(set(values[400:])), 1)
            self.assertAlmostEqual(values[-1], math.radians(5))
        for elapsed in (-.001, 5.001, True, False, math.nan, math.inf):
            with self.assertRaises(ValueError):
                step5_jog_offset(elapsed)
        for direction in (0, 2, True, 1.0, None):
            with self.assertRaises(ValueError):
                step5_jog_offset(1, direction)

    def test_codec_fixed_gains_headroom_and_old_bounds(self):
        for offset in (-math.radians(5), 0, math.radians(5)):
            f = ATParser().feed(motion_request(phase=TrialPhase.POSITION_STEP5,
                                   center_rad=5.5, offset_rad=offset, motor_id=1))[0]
            p, v, kp, kd = struct.unpack(">4H", f.data)
            self.assertEqual((f.destination, v, kp, kd), (1, 32767, 393, 1966))
            self.assertEqual((f.can_id >> 8) & 0xffff, 32767)
            self.assertAlmostEqual(p * POSITION_STEP_RAD - 12.57, 5.5 + offset, delta=POSITION_STEP_RAD)
        for phase in (TrialPhase.POSITION, TrialPhase.POSITION_STEP2, TrialPhase.POSITION_VISIBLE):
            with self.assertRaises(ValueError):
                motion_request(phase=phase, center_rad=0, offset_rad=math.radians(5))
        for center in (-12.57, 12.57, math.nan, True):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=center)
        for offset in (math.nextafter(math.radians(5), math.inf), -math.nextafter(math.radians(5), math.inf)):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=0, offset_rad=offset)

    def test_one_enable_five_second_current_relative_signed_wire_trajectory(self):
        for direction in (-1, 1):
            t = CenteredTransport()
            result = execute(t, direction=direction)
            self.assertEqual(result["status"], "MOTION_FINISHED_RESET_CONFIRMED")
            self.assertAlmostEqual(math.degrees(result["target_final_offset_rad"]), direction * 5)
            self.assertFalse(result["joint_calibration_verified"])
            self.assertEqual({f.destination for f in t.frames}, {1})
            self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)
            self.assertEqual((t.frames[-1].kind, t.frames[-1].data), (4, bytes(8)))
            self.assertAlmostEqual(t.active_deadline - t.enable_time, 6)
            active = [(when, struct.unpack(">4H", f.data)) for when, f in t.timed_frames
                      if f.kind == 1 and struct.unpack(">4H", f.data)[2] != 0]
            self.assertTrue(249 <= len(active) <= 251)
            self.assertEqual({v[2:] for _, v in active}, {(393, 1966)})
            deltas = [v[0] * POSITION_STEP_RAD - 12.57 - t.center for _, v in active]
            self.assertTrue(all(abs(v) <= math.radians(5) + POSITION_STEP_RAD for v in deltas))
            self.assertAlmostEqual(deltas[-1], direction * math.radians(5), delta=POSITION_STEP_RAD)
            hold = [v[0] for when, v in active if when - active[0][0] > 4.001]
            self.assertGreaterEqual(len(hold), 45)
            self.assertEqual(len(set(hold)), 1)
            self.assertLess(active[-1][0] - active[0][0], 5)
            self.assertGreater(t.clock() - t.enable_time, 5)
            self.assertLess(t.clock() - t.enable_time, 5.1)

    def test_drift_guard_remains_active_during_extended_hold(self):
        class Drift(CenteredTransport):
            def feedback(self, wires, stopping=False):
                fb, ts = super().feedback(wires, stopping=stopping)
                if not stopping and self.enable_time is not None and self.clock() - self.enable_time > 4.2:
                    fb = replace(fb, protocol_position_rad=self.center + math.radians(7.1))
                return fb, ts
        t = Drift()
        r = execute(t)
        self.assertEqual(r["status"], "ABORTED")
        self.assertFalse(r["motion_completed"])
        self.assertTrue(r["stop_confirmed"])
        self.assertIn("departed", r["errors"][0])
        self.assertEqual(t.frames[-1].kind, 4)

    def test_bad_headroom_prevents_enable_and_watchdog_write(self):
        t = CenteredTransport(center=12.56)
        r = execute(t)
        self.assertEqual(r["status"], "ABORTED")
        self.assertTrue(r["stop_confirmed"])
        self.assertFalse(any(f.kind in (1, 3, 18) for f in t.frames))

    def test_failures_and_interruption_still_stop_without_retry(self):
        for failure in ("communication", "overspeed", "stop"):
            t = CenteredTransport(failure=failure)
            r = execute(t)
            self.assertEqual(r["status"], "ABORTED")
            self.assertEqual(r["stop_confirmed"], failure != "stop")
            self.assertEqual(t.frames[-1].kind, 4)
            self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)
        t = CenteredTransport()
        def interrupt():
            if t.enabled:
                raise InterruptedError("operator")
        r = execute(t, interrupt=interrupt)
        self.assertFalse(r["motion_completed"])
        self.assertTrue(r["stop_confirmed"])
        self.assertEqual(sum(f.kind == 3 for f in t.frames), 1)

    def test_return_combination_rejected_before_transport_use(self):
        t = CenteredTransport()
        with self.assertRaises(ValueError):
            run_trial(t, UID, lambda: None, lambda _: None, gain_profile="step5", trajectory_kind="return")
        self.assertFalse(t.frames)
        self.assertFalse(t.parameter_names)

    def test_cli_dry_plan_matches_fixed_limits_and_never_opens_device(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "unused"
            serial = Mock(side_effect=AssertionError("no serial"))
            args = ["--expected-uid", UID, "--gain-profile", "step5", "--output", str(output)]
            with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=serial)}):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(args)
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    self.assertEqual(main(args + ["--trajectory-kind", "jog"]), 0)
                plan = json.loads(out.getvalue())
                for key, value in {"target_max_offset_deg": 5, "target_final_offset_deg": 5,
                         "trajectory_duration_s": 5, "jog_ramp_s": 4, "jog_hold_s": 1,
                         "active_command_budget_s": 6, "Kp": 3, "Kd": .15,
                         "max_observed_drift_deg": 7, "max_observed_speed_rad_s": .5,
                         "automatic_gain_increase": False, "automatic_reenable": False}.items():
                    self.assertEqual(plan[key], value)
                self.assertFalse(output.exists())
                serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
