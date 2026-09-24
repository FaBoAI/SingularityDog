"""Offline tests of three-axis batching, latching failures, and stopping all axes."""
from dataclasses import asdict, replace
import math
import struct
import unittest

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import Type2Feedback, TrialPhase, stop_request
from singularitydog_hw.rs05_leg_trial import run_leg_trial
from test_rs05_joint_trial import FakeClock

UIDS = {mid: f"{mid:016x}" for mid in (1, 2, 3)}
DIRECTIONS = (-1, -1, 1)
LSB = 25.14 / 65535


class FakeLegTransport:
    def __init__(self, failure=None):
        self.ids = (1, 2, 3)
        self.clock = FakeClock()
        self.centers = {1: 5.5, 2: 4.4, 3: 5.4}
        self.latest = {}
        self.enabled = set()
        self.frames = []
        self.calls = []
        self.stop_calls = []
        self.watchdogs = {mid: 0 for mid in self.ids}
        self.feedback_guard = None
        self.active_deadline = None
        self.failure = failure
        self.enable_time = None
        self.motion_batches = []

    def fresh_boundary(self):
        pass

    def parameter(self, mid, name=None):
        self.calls.append(("parameter", mid, name, self.clock()))
        self.clock.wait(.004)
        if name is None:
            return {"mcu_uid_hex": "f" * 16 if self.failure == "identity" and mid == 2 else UIDS[mid]}
        if name == "position":
            return {"value": 12.56 if self.failure == "headroom" and mid == 3 else self.centers[mid]}
        if name == "can_timeout":
            if self.failure == "prior_watchdog_rejected" and mid == 3:
                raise RuntimeError("ID3 parameter rejected: can_timeout")
            value = self.watchdogs[mid]
            if self.failure == "watchdog" and mid == 3:
                value = 0
            return {"value": value}
        return {"value": {"run_mode": 0, "current": 0, "voltage": 40}[name]}

    def send(self, wire):
        frame = ATParser().feed(wire)[0]
        mid = frame.destination
        if frame.kind == 3 and mid == 2 and self.failure == "partial_enable":
            raise IOError("second enable failed")
        if frame.kind in (1, 3) and self.active_deadline is not None and self.clock() >= self.active_deadline:
            raise RuntimeError("active deadline")
        self.frames.append((self.clock(), frame))
        if frame.kind == 18:
            self.watchdogs[mid] = 4000
        elif frame.kind == 3:
            self.enabled.add(mid)
            if self.enable_time is None:
                self.enable_time = self.clock()
        elif frame.kind == 4:
            self.enabled.discard(mid)

    def value(self, mid):
        fb = Type2Feedback(2 if mid in self.enabled else 0, 0, 0,
                           self.centers[mid], 0, 0, 30)
        if self.enabled and self.failure == "overspeed" and mid == 2:
            fb = replace(fb, velocity_rad_s=.6)
        if self.enabled and self.failure == "fault" and mid == 3:
            fb = replace(fb, fault_bits=1)
        if self.enable_time is not None and self.clock() - self.enable_time > 4.2 and mid == 2 and self.failure == "late_drift":
            fb = replace(fb, protocol_position_rad=self.centers[mid] + math.radians(7.1))
        return fb

    def feedback_many(self, wires, expected_ids):
        frames = [ATParser().feed(w)[0] for w in wires]
        active = [f for f in frames if f.kind == 1 and struct.unpack(">4H", f.data)[2]]
        if active:
            self.motion_batches.append((self.clock(), active))
        for wire in wires:
            self.send(wire)
        if self.enabled and self.failure == "communication":
            raise TimeoutError("one sibling did not reply")
        self.clock.wait(.007)
        result = {}
        for mid in expected_ids:
            fb, ts = self.value(mid), self.clock()
            if self.enabled and self.failure == "stale" and mid == 1:
                ts -= .11
            if self.feedback_guard is not None:
                self.feedback_guard(fb, ts, mid)
            result[mid] = (fb, ts)
            self.latest[mid] = (fb, ts)
        return result

    def stop_all(self, ids):
        ids = tuple(ids)
        self.stop_calls.append(ids)
        result = {}
        for mid in ids:
            self.send(stop_request(phase=TrialPhase.STOP, motor_id=mid))
        for mid in ids:
            fb = Type2Feedback(0, 0, 0, self.centers[mid], 0, 0, 30)
            self.latest[mid] = (fb, self.clock())
            failed = self.failure == "stop" and self.enable_time is not None and mid == 2
            result[mid] = {"confirmed": not failed, "feedback": None if failed else asdict(fb),
                           "error": "no stop reply" if failed else None}
        self.clock.wait(.005)
        return result


def execute(t, interrupt=lambda: None):
    return run_leg_trial(t, UIDS, interrupt, lambda _: None, directions=DIRECTIONS,
                         clock=t.clock, wait=t.clock.wait)


class LegRunnerTests(unittest.TestCase):
    def test_all_three_share_phase_and_all_watchdogs_precede_enable(self):
        t = FakeLegTransport()
        r = execute(t)
        self.assertEqual(r["status"], "MOTION_FINISHED_RESET_CONFIRMED")
        self.assertTrue(r["motion_completed"])
        self.assertTrue(r["stop_confirmed"])
        self.assertEqual(r["errors"], [])
        self.assertEqual({f.destination for _, f in t.frames}, {1, 2, 3})
        self.assertEqual([f.destination for _, f in t.frames if f.kind == 3], [1, 2, 3])
        first_enable = next(i for i, (_, f) in enumerate(t.frames) if f.kind == 3)
        self.assertEqual({f.destination for _, f in t.frames[:first_enable] if f.kind == 18}, {1, 2, 3})
        first_watchdog_write = next(when for when, f in t.frames if f.kind == 18)
        prior_reads = [call for call in t.calls if call[2] == "can_timeout"][:3]
        self.assertEqual([call[1] for call in prior_reads], [1, 2, 3])
        self.assertTrue(all(call[3] < first_watchdog_write for call in prior_reads))
        for mid in t.ids:
            self.assertEqual(r["motors"][mid]["watchdog_previous_ticks"], 0)
            self.assertEqual(r["motors"][mid]["watchdog_readback_ticks"], 4000)
        self.assertEqual(t.stop_calls[-1], (1, 2, 3))
        self.assertEqual([f.kind for _, f in t.frames[-3:]], [4, 4, 4])
        self.assertTrue(99 <= len(t.motion_batches) <= 101)
        times = [when for when, _ in t.motion_batches]
        self.assertTrue(all(abs(b-a-.05) < 1e-8 for a, b in zip(times, times[1:])))
        for _, frames in t.motion_batches:
            self.assertEqual([f.destination for f in frames], [1, 2, 3])
            progress = []
            for f, direction in zip(frames, DIRECTIONS):
                p, v, kp, kd = struct.unpack(">4H", f.data)
                self.assertEqual((v, kp, kd), (32767, 393, 1966))
                delta = p * LSB - 12.57 - t.centers[f.destination]
                self.assertLessEqual(abs(delta), math.radians(5) + LSB)
                progress.append(direction * delta)
            self.assertLessEqual(max(progress)-min(progress), 2*LSB)
        for f, direction in zip(t.motion_batches[-1][1], DIRECTIONS):
            delta = struct.unpack(">4H", f.data)[0]*LSB-12.57-t.centers[f.destination]
            self.assertAlmostEqual(delta, direction*math.radians(5), delta=LSB)
        self.assertGreaterEqual(t.clock()-t.enable_time, 5)
        self.assertLess(t.clock()-t.enable_time, 5.1)

    def test_identity_failure_does_not_enable_or_write_settings(self):
        t = FakeLegTransport("identity")
        r = execute(t)
        self.assertEqual(r["status"], "ABORTED")
        self.assertFalse(any(f.kind in (1, 3, 18) for _, f in t.frames))

    def test_all_headroom_and_watchdog_checks_finish_before_any_enable(self):
        for failure in ("headroom", "watchdog"):
            with self.subTest(failure=failure):
                t = FakeLegTransport(failure)
                r = execute(t)
                self.assertEqual(r["status"], "ABORTED")
                self.assertFalse(any(f.kind == 3 for _, f in t.frames))
                self.assertEqual(t.stop_calls[-1], (1, 2, 3))

    def test_rejected_third_prior_watchdog_read_writes_nothing_and_stops_all(self):
        t = FakeLegTransport("prior_watchdog_rejected")
        r = execute(t)
        self.assertEqual(r["status"], "ABORTED")
        self.assertFalse(r["motion_completed"])
        self.assertIsNot(r.get("enable_confirmed"), True)
        self.assertTrue(r["stop_confirmed"])
        self.assertTrue(any("parameter rejected: can_timeout" in e for e in r["errors"]))
        self.assertEqual([call[1] for call in t.calls if call[2] == "can_timeout"], [1, 2, 3])
        self.assertFalse(any(f.kind in (3, 18) for _, f in t.frames))
        self.assertEqual(t.watchdogs, {1: 0, 2: 0, 3: 0})
        self.assertEqual(t.motion_batches, [])
        self.assertEqual(t.stop_calls, [(1, 2, 3), (1, 2, 3)])
        self.assertEqual([f.destination for _, f in t.frames[-3:]], [1, 2, 3])
        self.assertEqual([f.kind for _, f in t.frames[-3:]], [4, 4, 4])
        for mid in (1, 2):
            self.assertEqual(r["motors"][mid]["watchdog_previous_ticks"], 0)
        self.assertNotIn("watchdog_previous_ticks", r["motors"][3])
        self.assertTrue(all("watchdog_readback_ticks" not in m for m in r["motors"].values()))

    def test_any_sibling_failure_stops_all_without_retry(self):
        for failure in ("partial_enable", "communication", "overspeed", "fault", "stale", "late_drift"):
            with self.subTest(failure=failure):
                t = FakeLegTransport(failure)
                r = execute(t)
                self.assertEqual(r["status"], "ABORTED")
                self.assertFalse(r["motion_completed"])
                self.assertEqual(t.stop_calls[-1], (1, 2, 3))
                for mid in t.ids:
                    self.assertLessEqual(sum(f.kind == 3 and f.destination == mid for _, f in t.frames), 1)

    def test_one_missing_stop_never_reports_success(self):
        t = FakeLegTransport("stop")
        r = execute(t)
        self.assertEqual(r["status"], "ABORTED")
        self.assertFalse(r["stop_confirmed"])
        self.assertEqual(t.stop_calls[-1], (1, 2, 3))
        self.assertEqual([f.destination for _, f in t.frames[-3:]], [1, 2, 3])

    def test_operator_interrupt_stops_all_axes(self):
        t = FakeLegTransport()
        def interrupt():
            if t.enabled:
                raise InterruptedError("operator")
        r = execute(t, interrupt)
        self.assertEqual(r["status"], "ABORTED")
        self.assertFalse(r["motion_completed"])
        self.assertEqual(t.stop_calls[-1], (1, 2, 3))

    def test_invalid_plan_rejected_before_queries(self):
        for directions in ((1, 1), (1, 0, 1), (1, True, 1), (1., 1, 1), (1, 1, 1, 1)):
            t = FakeLegTransport()
            with self.assertRaises(ValueError):
                run_leg_trial(t, UIDS, lambda:None, lambda _:None, directions=directions)
            self.assertFalse(t.calls)
            self.assertFalse(t.frames)


if __name__ == "__main__":
    unittest.main()
