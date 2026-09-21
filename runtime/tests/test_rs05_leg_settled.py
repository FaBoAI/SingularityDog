"""Synthetic fixed-window stationarity tests; no hardware or private identities."""
from copy import deepcopy
from dataclasses import asdict, replace
import math
import unittest
from unittest.mock import patch

from singularitydog_hw.rs05_leg_trial import evaluate_settled_window
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import Type2Feedback
from test_rs05_leg_trial import FakeLegTransport, execute

CENTERS = {1: 5.5, 2: 4.4, 3: 5.4}


def samples(position=lambda t: 0., velocity=lambda t: 0.):
    return {mid: [{"sample_index": n, "received_monotonic_s": 10+n*.1,
                  "checked_monotonic_s": 10+n*.1+.004,
                  "feedback": asdict(Type2Feedback(0, 0, 32767, center+position(n*.1),
                                                    velocity(n*.1), 0., 30.))}
                 for n in range(21)] for mid, center in CENTERS.items()}


class WindowTests(unittest.TestCase):
    def check_failed(self, rows):
        report = evaluate_settled_window(rows, CENTERS)
        self.assertFalse(report["passed"])
        self.assertTrue(report["errors"])
        return report

    def test_stationary_velocity_noise_uses_all_samples_and_RMS(self):
        rows = samples(velocity=lambda t: .07 if t == 1. else .02)
        result = evaluate_settled_window(rows, CENTERS)
        self.assertTrue(result["passed"])
        for mid in CENTERS:
            self.assertEqual(result["motors"][mid]["sample_count"], 21)
            self.assertEqual(result["motors"][mid]["max_abs_velocity_rad_s"], .07)
            self.assertEqual(result["motors"][mid]["samples"], rows[mid])

    def test_creep_and_returning_oscillation_rejected_despite_low_speed(self):
        for position, velocity in ((lambda t: .003*t, lambda t: .003),
                                  (lambda t: .0011*t, lambda t: .0011),
                                  (lambda t: .0012*math.sin(math.pi*t),
                                   lambda t: .0012*math.pi*math.cos(math.pi*t))):
            with self.subTest(position=position):
                self.check_failed(samples(position, velocity))

    def test_velocity_signs_cannot_cancel_RMS(self):
        self.check_failed(samples(velocity=lambda t: .06 if round(t*10)%2 else -.06))

    def test_single_spikes_and_static_wrong_center_rejected(self):
        self.check_failed(samples(position=lambda t: .003 if t == 1. else 0.))
        self.check_failed(samples(velocity=lambda t: .51 if t == 1. else 0.))
        self.check_failed(samples(position=lambda t: .021))

    def test_missing_extra_wrong_ids_and_missing_sample_not_trimmed(self):
        rows = samples(); rows.pop(2); self.check_failed(rows)
        rows = samples(); rows[4] = rows.pop(3); self.check_failed(rows)
        rows = samples(); rows[2].pop(10); self.check_failed(rows)
        rows = samples(); rows[2].append(deepcopy(rows[2][-1])); self.check_failed(rows)

    def test_timing_failures_rejected(self):
        for failure in ("duplicate", "reorder", "gap", "short", "long", "stale", "future"):
            with self.subTest(failure=failure):
                rows = samples()
                if failure in ("short", "long"):
                    step = .09 if failure == "short" else .12
                    for n, row in enumerate(rows[1]):
                        row["received_monotonic_s"] = 10+n*step
                        row["checked_monotonic_s"] = 10+n*step+.004
                else:
                    row = rows[1][10]
                    if failure == "duplicate": row["received_monotonic_s"] = 10.9
                    if failure == "reorder": row["sample_index"] = 9
                    if failure == "gap": row["received_monotonic_s"] += .06
                    if failure == "stale": row["checked_monotonic_s"] += .11
                    if failure == "future": row["checked_monotonic_s"] = 10.99
                self.check_failed(rows)

    def test_fault_mode_and_nonfinite_or_bool_rejected(self):
        for field, value in (("fault_bits", 1), ("mode_state", 2), ("mode_state", False),
                             ("protocol_position_rad", float('nan')),
                             ("velocity_rad_s", float('inf')), ("velocity_rad_s", True),
                             ("torque_nm", float('nan')), ("temperature_c", 50.)):
            with self.subTest(field=field, value=value):
                rows = samples(); rows[3][10]["feedback"][field] = value
                self.check_failed(rows)


class WindowTransport(FakeLegTransport):
    def __init__(self, change=None):
        super().__init__()
        self.change = change
        self.window_count = 0

    def feedback_many(self, wires, expected_ids):
        # The first disabled batch is before watchdog setup; window follows it.
        window = (not self.enabled and all(v == 4000 for v in self.watchdogs.values())
                  and not any(ATParser().feed(w)[0].kind == 3 for w in wires))
        if window:
            self.window_count += 1
        found = super().feedback_many(wires, expected_ids)
        if window and self.change:
            return self.change(self, found)
        return found


class WindowRunnerTests(unittest.TestCase):
    def test_fixed_window_once_before_enable_and_no_gain_during_window(self):
        t = WindowTransport()
        result = execute(t)
        self.assertTrue(result["settled_window"]["passed"])
        self.assertEqual(t.window_count, 21)
        self.assertTrue(result["motion_completed"])
        first_enable = next(n for n, (_, f) in enumerate(t.frames) if f.kind == 3)
        for _, frame in t.frames[:first_enable]:
            if frame.kind == 1:
                self.assertEqual(frame.data[4:8], bytes(4))
        self.assertGreater(t.enable_time, 2.)
        self.assertLess(t.clock()-t.enable_time, 5.1)

    def test_motion_or_fault_or_missing_or_delay_window_never_enables(self):
        def change(failure):
            def apply(t, found):
                n = t.window_count
                fb, ts = found[3]
                if failure == "creep": found[3] = (replace(fb, protocol_position_rad=fb.protocol_position_rad+n*.0003), ts)
                if failure == "fault" and n == 10: found[3] = (replace(fb, fault_bits=1), ts)
                if failure == "missing" and n == 10: found.pop(3)
                if failure == "gap" and n == 10: t.clock.wait(.16)
                return found
            return apply
        for failure in ("creep", "fault", "missing", "gap"):
            with self.subTest(failure=failure):
                t = WindowTransport(change(failure)); result = execute(t)
                self.assertEqual(result["status"], "ABORTED")
                self.assertFalse(result["settled_window"]["passed"])
                self.assertFalse(any(f.kind == 3 for _, f in t.frames))
                self.assertLessEqual(t.window_count, 21)
                self.assertEqual(t.stop_calls[-1], (1, 2, 3))

    def test_report_delay_rechecks_last_samples_before_enable(self):
        from singularitydog_hw.rs05_leg_trial import run_leg_trial
        from test_rs05_leg_trial import UIDS, DIRECTIONS
        t = WindowTransport()
        def emit(event):
            if event["kind"] == "leg_trial_settled_window": t.clock.wait(.11)
        result = run_leg_trial(t, UIDS, lambda: None, emit, directions=DIRECTIONS,
                               clock=t.clock, wait=t.clock.wait)
        self.assertEqual(result["status"], "ABORTED")
        self.assertTrue(result["settled_window"]["passed"])
        self.assertFalse(any(f.kind == 3 for _, f in t.frames))
        self.assertEqual(t.stop_calls[-1], (1, 2, 3))

    def test_changed_latest_after_window_is_not_substituted_for_measured_sample(self):
        from singularitydog_hw.rs05_leg_trial import run_leg_trial
        from test_rs05_leg_trial import UIDS, DIRECTIONS
        for change in ("time", "position"):
            t = WindowTransport()
            def emit(event):
                if event["kind"] == "leg_trial_settled_window":
                    fb, ts = t.latest[2]
                    if change == "time": ts += .0001
                    else: fb = replace(fb, protocol_position_rad=fb.protocol_position_rad+.001)
                    t.latest[2] = (fb, ts)
            result = run_leg_trial(t, UIDS, lambda: None, emit, directions=DIRECTIONS,
                                   clock=t.clock, wait=t.clock.wait)
            self.assertEqual(result["status"], "ABORTED")
            self.assertTrue(result["settled_window"]["passed"])
            self.assertTrue(any("changed after fixed" in e for e in result["errors"]))
            self.assertFalse(any(f.kind == 3 for _, f in t.frames))
            self.assertEqual(t.stop_calls[-1], (1, 2, 3))


class WindowEnableTransportTests(unittest.TestCase):
    def test_each_enable_rechecks_freshness_after_previous_slow_log(self):
        from singularitydog_hw.rs05_leg_trial import LegTrialTransport
        from singularitydog_hw.rs05_joint_trial import check_feedback
        from singularitydog_hw.rs05_trial_protocol import TrialPhase, enable_request
        from test_rs05_joint_trial import FakeClock
        from test_rs05_leg_pacing import TimedSerial
        clock = FakeClock()
        port = TimedSerial(clock)
        def emit(event):
            if event['kind'] == 'can_tx' and event['type'] == 3:
                clock.wait(.11)
        t = LegTrialTransport(port, emit, ids=(1, 2, 3), wait=clock.wait)
        final_samples = {i: (Type2Feedback(0, 0, 32767, 0., 0., 0., 30.), clock()) for i in t.ids}
        def guard():
            for fb, when in final_samples.values():
                check_feedback(fb, 0., when, clock(), required_mode=0, max_drift_rad=.02)
        t.pre_enable_guard = guard
        with patch('singularitydog_hw.rs05_leg_trial.time.monotonic', clock):
            t.send(enable_request(phase=TrialPhase.ENABLE, motor_id=1))
            with self.assertRaisesRegex(RuntimeError, 'Stale'):
                t.send(enable_request(phase=TrialPhase.ENABLE, motor_id=2))
        self.assertEqual(len(port.starts), 1)


if __name__ == '__main__':
    unittest.main()
