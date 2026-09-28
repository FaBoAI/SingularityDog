"""Offline command/catch-window properties; no hardware or model imports."""

from dataclasses import asdict
import json
import math
import random
import unittest

from singularitydog_hw.ground_trial_trajectory import (
    DECISION_MARGIN_S, GroundTimeline, GroundTrajectoryError,
)


def timeline(stage="walk", **changes):
    kwargs = dict(stage=stage, duration_s=9., initial_hold_s=1.5,
                  active_duration_s=3., forward_velocity_m_s=.05 if stage == "walk" else 0.,
                  ramp_up_s=.5 if stage == "walk" else 0., ramp_down_s=.75 if stage == "walk" else 0.,
                  final_stationary_s=.5, resupport_window_s=1., shutdown_reserve_s=1.)
    kwargs.update(changes)
    return GroundTimeline(**kwargs)


class GroundTimelineTests(unittest.TestCase):
    def test_walk_zero_outside_active_and_forward_only_everywhere(self):
        trial = timeline()
        self.assertEqual(trial.command_at(0), (0., 0., 0.))
        self.assertEqual(trial.command_at(1.5), (0., 0., 0.))
        self.assertEqual(trial.command_at(4.5), (0., 0., 0.))
        self.assertEqual(trial.command_at(100), (0., 0., 0.))
        for tick in range(1001):
            command = trial.command_at(tick / 100.)
            self.assertGreaterEqual(command[0], 0.)
            self.assertLessEqual(command[0], .05)
            self.assertEqual(command[1:], (0., 0.))
        self.assertEqual(trial.command_at(3), (.05, 0., 0.))

    def test_quintic_joins_are_position_velocity_acceleration_continuous(self):
        trial = timeline()
        h = .00001
        f = lambda t: trial.command_at(t)[0]
        for boundary in (1.5, 2., 3.75, 4.5):
            left = (f(boundary) - f(boundary-h)) / h
            right = (f(boundary+h) - f(boundary)) / h
            second_left = (f(boundary)-2*f(boundary-h)+f(boundary-2*h)) / h**2
            second_right = (f(boundary+2*h)-2*f(boundary+h)+f(boundary)) / h**2
            self.assertLess(abs(left-right), 1e-7)
            self.assertLess(abs(second_left-second_right), .001)

    def test_nonwalk_stages_always_zero_not_automatic_balance(self):
        for stage in ("supported_stance", "partial_load", "stand"):
            trial = timeline(stage)
            for t in (0, 1.5, 2, 4.5, 5, 7.9):
                decision = trial.step(t)
                self.assertEqual(decision.command, (0., 0., 0.))
                self.assertFalse(decision.request_normal_stop)
            result = trial.step(8)
            self.assertEqual(result.request_normal_stop, stage == "supported_stance")
            self.assertEqual(result.emergency_stop, stage != "supported_stance")

    def test_unconfirmed_resupport_stops_at_deadline_without_waiting(self):
        trial = timeline()
        self.assertEqual(trial.step(5).phase, "resupport_hold")
        self.assertFalse(trial.step(7.99).request_normal_stop)
        stopped = trial.step(8)
        self.assertTrue(stopped.emergency_stop)
        self.assertFalse(stopped.request_normal_stop)
        self.assertEqual(stopped.reason, "resupport_not_confirmed")
        # A later human confirmation cannot revive a failed trial.
        still_stopped = trial.step(8.1, resupport_ack_s=8)
        self.assertTrue(still_stopped.emergency_stop)
        self.assertFalse(still_stopped.resupport_ack_accepted)

    def test_fresh_ack_is_recorded_but_gains_wait_until_reserved_stop_time(self):
        trial = timeline()
        accepted = trial.step(5.2, resupport_ack_s=5.1)
        self.assertTrue(accepted.resupport_ack_accepted)
        self.assertFalse(accepted.request_normal_stop)
        self.assertEqual(trial.resupport_ack_s, 5.1)
        final = trial.step(8.01)
        self.assertTrue(final.request_normal_stop)
        self.assertFalse(final.emergency_stop)
        self.assertEqual(final.command, (0., 0., 0.))

    def test_stale_late_future_nonfinite_ack_cannot_authorize_gain_down(self):
        trial = timeline()
        early = trial.step(5.1, resupport_ack_s=4.999)
        self.assertEqual(early.reason, "resupport_ack_before_window")
        self.assertFalse(early.resupport_ack_accepted)
        late = trial.step(8.01, resupport_ack_s=8.001)
        self.assertTrue(late.emergency_stop)
        self.assertIsNone(trial.resupport_ack_s)
        for ack in (5.11, float("nan"), float("inf"), True, "5"):
            with self.subTest(ack=ack), self.assertRaises(GroundTrajectoryError):
                timeline().step(5.1, resupport_ack_s=ack)

    def test_deadline_skipped_fails_even_with_ack_and_supported_stage(self):
        for stage in ("walk", "supported_stance"):
            trial = timeline(stage)
            result = trial.step(8 + DECISION_MARGIN_S + .001, resupport_ack_s=6.)
            self.assertTrue(result.emergency_stop)
            self.assertEqual(result.reason, "normal_stop_deadline_missed")
            self.assertFalse(result.request_normal_stop)

    def test_cues_have_exact_crossings_no_repeated_announcements(self):
        trial = timeline()
        self.assertEqual([c.key for c in trial.cues_between(None, 0)], ["initial_hold"])
        self.assertEqual([c.key for c in trial.cues_between(1.49, 1.5)], ["active_window_open"])
        self.assertEqual(trial.cues_between(1.5, 1.5), ())
        self.assertEqual([c.key for c in trial.step(0).cues], ["initial_hold"])
        self.assertEqual(trial.step(0).cues, ())
        keys = [c.key for c in trial.step(5).cues]
        self.assertEqual(keys, ["active_window_open", "active_window_close", "resupport_window_open"])
        self.assertEqual(trial.step(5).cues, ())
        # Frozen dataclasses can be serialized in the caller's runtime report.
        self.assertIn("resupport_hold", json.dumps(asdict(trial.step(5.1))))

    def test_graceful_request_before_active_skips_movement_not_initial_hold(self):
        trial = timeline()
        effective = trial.request_stop(.5)
        self.assertEqual(effective["active_end_s"], 1.5)
        self.assertEqual(effective["resupport_window_open_s"], 2.)
        self.assertEqual(effective["latest_stop_start_s"], 3.)
        self.assertEqual(trial.step(1.).phase, "initial_hold")
        for t in (1.5, 2., 2.5, 3.):
            self.assertEqual(trial.command_at(t), (0., 0., 0.))
        self.assertIn("active_window_skipped", [c.key for c in trial.step(2).cues])
        self.assertTrue(trial.step(3, resupport_ack_s=2.2).request_normal_stop)

    def test_graceful_request_during_rampup_preserves_joins_and_bounded_speed(self):
        trial = timeline()
        before = [trial.command_at(t / 100.) for t in range(151, 176)]
        effective = trial.request_stop(1.75)
        self.assertEqual(effective["ramp_down_start_s"], 2.)
        self.assertEqual(effective["active_end_s"], 2.75)
        self.assertEqual([trial.command_at(t / 100.) for t in range(151, 176)], before)
        self.assertAlmostEqual(trial.command_at(2)[0], .05)
        self.assertEqual(trial.command_at(2.75), (0., 0., 0.))
        self.assertEqual(effective["latest_stop_start_s"], 4.25)

    def test_graceful_request_in_cruise_decelerates_now_without_extending_trial(self):
        trial = timeline()
        effective = trial.request_stop(3)
        self.assertEqual(effective["ramp_down_start_s"], 3.)
        self.assertEqual(effective["active_end_s"], 3.75)
        self.assertLess(trial.command_at(3.01)[0], .05)
        self.assertEqual(trial.command_at(3.75)[0], 0.)
        # Duplicate requests do not postpone resupport or termination.
        self.assertEqual(trial.request_stop(4), effective)

    def test_request_during_deceleration_keeps_existing_smooth_command(self):
        before = timeline()
        trial = timeline()
        effective = trial.request_stop(4.)
        self.assertEqual(effective["active_end_s"], 4.5)
        for t in (4, 4.1, 4.3, 4.5):
            self.assertEqual(trial.command_at(t), before.command_at(t))

    def test_early_nonwalk_request_requires_new_resupport_window(self):
        trial = timeline("stand")
        trial.step(5.2, resupport_ack_s=5.1)
        effective = trial.request_stop(5.3)
        self.assertEqual(effective["resupport_window_open_s"], 5.3)
        self.assertEqual(effective["latest_stop_start_s"], 6.3)
        self.assertIsNone(trial.resupport_ack_s)
        self.assertFalse(trial.step(5.4, resupport_ack_s=5.1).resupport_ack_accepted)
        self.assertTrue(trial.step(6.3).emergency_stop)

    def test_late_graceful_request_cannot_shorten_remaining_resupport_window(self):
        trial = timeline()
        original = trial.timing
        effective = trial.request_stop(7.5)
        for key in ("active_end_s", "resupport_window_open_s", "latest_stop_start_s"):
            self.assertEqual(effective[key], original[key])

    def test_many_early_requests_remain_nonnegative_bounded_and_finite(self):
        rng = random.Random(404)
        for _ in range(100):
            trial = timeline()
            requested = rng.uniform(0., 9.)
            effective = trial.request_stop(requested)
            self.assertLessEqual(effective["latest_stop_start_s"], 8.)
            self.assertLessEqual(effective["active_end_s"], 4.5)
            self.assertGreaterEqual(effective["latest_stop_start_s"] - effective["resupport_window_open_s"], 1. - 1e-12)
            for tick in range(101):
                command = trial.command_at(tick / 10.)
                self.assertTrue(0. <= command[0] <= .05)
                self.assertEqual(command[1:], (0., 0.))

    def test_invalid_plans_and_nonfinite_times_fail_before_output(self):
        changes = ({"duration_s": 10.001}, {"initial_hold_s": 0}, {"active_duration_s": .9},
                   {"forward_velocity_m_s": .051}, {"forward_velocity_m_s": -.01},
                   {"ramp_up_s": .49}, {"ramp_down_s": 3}, {"final_stationary_s": .49},
                   {"resupport_window_s": .99}, {"shutdown_reserve_s": .04},
                   {"duration_s": 5}, {"duration_s": True}, {"duration_s": math.inf},
                   {"initial_hold_s": math.nan}, {"duration_s": 10**1000})
        for change in changes:
            with self.subTest(change=change), self.assertRaises(GroundTrajectoryError):
                timeline(**change)
        for change in ({"ramp_up_s": .1}, {"forward_velocity_m_s": .001}):
            with self.assertRaises(GroundTrajectoryError):
                timeline("stand", **change)
        for time in (-1, True, math.nan, math.inf, "1"):
            for method in ("step", "command_at", "request_stop"):
                with self.subTest(time=time, method=method), self.assertRaises(GroundTrajectoryError):
                    getattr(timeline(), method)(time)
        trial = timeline()
        trial.step(1)
        with self.assertRaises(GroundTrajectoryError):
            trial.step(.5)
        trial.request_stop(2)
        with self.assertRaises(GroundTrajectoryError):
            trial.step(1.99)
        with self.assertRaises(GroundTrajectoryError):
            trial.cues_between(3, 2)


if __name__ == "__main__":
    unittest.main()
