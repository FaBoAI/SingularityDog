"""Offline rational-clock tests. No motor, USB, SSH, or wall-clock activity."""

from dataclasses import FrozenInstanceError
import copy
import math
import unittest

from singularitydog_hw.policy_clock import (
    DEFAULT_POLICY_CLOCK, PolicyClock, Pure30Budget, control_clock_payload,
    policy_clock_settings, validate_control_clock,
)


class PolicyClockContractTests(unittest.TestCase):
    def test_legacy_profile_defaults_to_unchanged_50hz(self):
        self.assertIs(policy_clock_settings({}), DEFAULT_POLICY_CLOCK)
        clock = policy_clock_settings({"other": "legacy"})
        self.assertEqual(clock.policy_hz, 50)
        self.assertEqual(clock.cycle_budget_ns, 20_000_000)
        self.assertEqual(clock.command_gap_budget_ns, 21_000_000)
        self.assertEqual(clock.minimum_start_separation_ns, 15_000_000)
        self.assertEqual(clock.model_dt_s, .02)

    def test_30hz_changes_cadence_only_not_freshness_or_watchdogs(self):
        clock = validate_control_clock(control_clock_payload(30))
        self.assertEqual(clock.cycle_budget_ns, 33_333_334)
        self.assertEqual(clock.command_gap_budget_ns, 34_333_334)
        self.assertAlmostEqual(clock.cadence_gap_ms, 34.333333333333336)
        self.assertEqual(clock.model_dt_s, 1 / 30)
        self.assertEqual(clock.freshness_budget_ns, 20_000_000)
        self.assertEqual(clock.inference_tx_budget_ns, 20_000_000)
        self.assertEqual(clock.host_watchdog_ns, 40_000_000)
        self.assertEqual(clock.motor_watchdog_ns, 200_000_000)

    def test_clock_is_immutable_and_detached_from_profile(self):
        payload = control_clock_payload(30)
        clock = policy_clock_settings({"control_clock": payload})
        payload["policy_hz"] = 50
        self.assertEqual(clock.policy_hz, 30)
        with self.assertRaises(FrozenInstanceError):
            clock.policy_hz = 50
        copy_payload = clock.as_payload()
        copy_payload["input_freshness_budget_ns"] = 40_000_000
        self.assertEqual(clock.freshness_budget_ns, 20_000_000)

    def test_explicit_invalid_clock_never_falls_back_to_legacy(self):
        for value in (None, False, 30, [], {}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                policy_clock_settings({"control_clock": value})
        for value in (None, False, [], 30):
            with self.subTest(value=value), self.assertRaises(ValueError):
                policy_clock_settings(value)

    def test_unsupported_rates_and_bool_are_rejected(self):
        for rate in (0, -30, 20, 60, 30.0, "30", True, False, None, math.nan, math.inf):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                PolicyClock(rate)

    def test_nan_inf_wrong_dt_and_field_types_are_rejected(self):
        alterations = [
            ("model_dt_s", math.nan), ("model_dt_s", math.inf),
            ("model_dt_s", -.02), ("model_dt_s", 0.0),
            ("model_dt_s", .02), ("model_dt_s", 1 / 30 + 1e-14),
            ("model_dt_s", True), ("policy_hz", 30.0),
            ("cycle_budget_ns", 33_333_333),
            ("inference_tx_budget_ns", 33_333_334),
            ("input_freshness_budget_ns", True),
            ("host_watchdog_ns", 60_000_000),
            ("motor_watchdog_ns", 250_000_000),
            ("period_denominator", 50), ("period_numerator_ns", 1e9),
            ("release_rule", "rounded_period_addition"),
        ]
        for name, value in alterations:
            payload = control_clock_payload(30)
            payload[name] = value
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                validate_control_clock(payload)

    def test_missing_and_extra_fields_are_rejected(self):
        payload = control_clock_payload(30)
        for name in payload:
            truncated = copy.deepcopy(payload)
            del truncated[name]
            with self.subTest(missing=name), self.assertRaises(ValueError):
                validate_control_clock(truncated)
        payload["period_ns"] = 33_333_333
        with self.assertRaises(ValueError):
            validate_control_clock(payload)


class RationalPolicyScheduleTests(unittest.TestCase):
    def test_30hz_intervals_follow_integer_rational_clock(self):
        clock = PolicyClock(30)
        epoch = 5_000_000_007
        releases = [clock.release_ns(epoch, i) for i in range(7)]
        self.assertEqual(releases, [epoch + v for v in (
            0, 33_333_333, 66_666_666, 100_000_000,
            133_333_333, 166_666_666, 200_000_000)])
        self.assertEqual({b - a for a, b in zip(releases, releases[1:])},
                         {33_333_333, 33_333_334})

    def test_one_hour_and_one_day_have_no_rounded_period_drift(self):
        epoch = 9_007_199_254_740_993
        for hz in (30, 50):
            clock = PolicyClock(hz)
            for seconds in (3600, 3601, 86400):
                with self.subTest(hz=hz, seconds=seconds):
                    self.assertEqual(clock.release_ns(epoch, seconds * hz),
                                     epoch + seconds * 1_000_000_000)
            # Every release remains floor(exact rational), including a large
            # monotonic epoch beyond float's exact-integer precision.
            for index in (1, 7, 108001, 2592001):
                self.assertEqual(clock.release_ns(epoch, index) - epoch,
                                 index * 1_000_000_000 // hz)

    def test_slot_deadline_is_distinct_from_elapsed_budget_and_prefix(self):
        clock = PolicyClock(30)
        epoch = 1_000_000_000
        begin = epoch + 20_000
        oldest = begin + 100_000
        self.assertEqual(clock.slot_deadline_ns(epoch, 0), epoch + 33_333_333)
        self.assertEqual(clock.cycle_deadline_ns(begin), begin + 33_333_334)
        self.assertEqual(clock.input_tx_deadline_ns(begin, oldest), begin + 20_000_000)
        self.assertEqual(clock.input_tx_deadline_ns(begin, begin - 100_000),
                         begin - 100_000 + 20_000_000)
        self.assertLess(clock.input_tx_deadline_ns(begin, oldest),
                        clock.slot_deadline_ns(epoch, 0))

    def test_release_boundary_inverse_handles_floor_exactly(self):
        clock = PolicyClock(30)
        epoch = 1_000_000_000
        for index in range(1, 1000):
            boundary = clock.release_ns(epoch, index)
            self.assertEqual(clock.select_slot(epoch, None, None, boundary), (index, boundary))
            self.assertEqual(clock.select_slot(epoch, None, None, boundary - 1)[0], index - 1)

    def test_late_wake_identifies_skip_without_duplicate_or_catchup(self):
        clock = PolicyClock(30)
        epoch = 1_000_000_000
        self.assertEqual(clock.select_slot(epoch, None, None, epoch), (0, epoch))
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 70_000, epoch + 31_000_000),
                         (1, epoch + 33_333_333))
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 70_000, epoch + 102_000_000),
                         (3, epoch + 100_000_000))
        self.assertEqual(clock.select_slot(epoch, 3, epoch + 102_000_000, epoch + 103_000_000),
                         (4, epoch + 133_333_333))

    def test_late_first_cycle_cannot_trigger_short_catchup_burst(self):
        clock = PolicyClock(30)
        epoch = 1_000_000_000
        # A first begin at +30 ms leaves too little separation before slot 1.
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 30_000_000, epoch + 31_000_000),
                         (2, epoch + 66_666_666))

    def test_50hz_matches_existing_absolute_slot_examples(self):
        clock = PolicyClock(50)
        epoch = 1_000_000_000
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 70_000, epoch + 18_500_000),
                         (1, epoch + 20_000_000))
        self.assertEqual(clock.select_slot(epoch, 1, epoch + 20_110_000, epoch + 39_000_000),
                         (2, epoch + 40_000_000))
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 80_000, epoch + 60_100_000),
                         (3, epoch + 60_000_000))
        self.assertEqual(clock.select_slot(epoch, 0, epoch + 18_000_000, epoch + 19_000_000),
                         (2, epoch + 40_000_000))

    def test_invalid_schedule_arguments_rejected(self):
        clock = PolicyClock(30)
        epoch = 1_000_000_000
        invalid = (
            (epoch, 1, None, epoch), (epoch, None, epoch, epoch),
            (epoch, 0, epoch, epoch - 1), (epoch, True, epoch, epoch),
            (True, None, None, epoch), (epoch, None, None, math.nan),
            (epoch, 1, epoch, epoch + 40_000_000),
            (epoch, 0, epoch + 1, epoch),
        )
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                clock.select_slot(*args)
        for invalid_index in (-1, True, 1.0, math.inf):
            with self.subTest(index=invalid_index), self.assertRaises(ValueError):
                clock.release_ns(epoch, invalid_index)
        self.assertEqual(clock.release_ns(0, 0), 0)


class ThirtyHzDeadlineBudgetTests(unittest.TestCase):
    def setUp(self):
        self.settings = dict(mode="bounded_post_reply_v1", max_lateness_ms=1.,
                             max_consecutive_misses=1, rolling_window_cycles=100,
                             max_misses_per_window=1)
        self.clock = PolicyClock(30)
        self.budget = Pure30Budget(self.settings, self.clock)

    def admit(self, index=0, *, elapsed=33_000_000, **changes):
        begin = 1_000_000_000 + index * 40_000_000
        options = dict(index=index, begin_ns=begin, oldest_input_ns=begin + 1_000_000,
                       final_write_ns=begin + 17_000_000, last_reply_ns=begin + 32_000_000,
                       output_sample_start_ns=begin + 14_000_000,
                       checked_ns=begin + elapsed, sample_age_ns=20_000_000)
        options.update(changes)
        return self.budget.admit(**options)

    def test_reply_after_20ms_preserves_send_freshness_and_feedback_freshness(self):
        result = self.admit()
        self.assertTrue(result["input_to_tx_20ms_met"])
        self.assertEqual(result["input_sample_age_at_tx_ms"], 16.)
        self.assertEqual(result["completed_input_age_at_post_reply_ms"], 32.)
        self.assertEqual(result["output_feedback_sample_age_ms"], 19.)
        self.assertFalse(result["cycle_deadline_missed"])
        self.assertEqual(result["cycle_budget_ns"], 33_333_334)

    def test_exact_tx_and_reply_boundaries_are_admitted(self):
        result = self.admit(elapsed=33_333_334, final_write_ns=1_020_000_000,
                            last_reply_ns=1_033_333_334)
        self.assertFalse(result["allowance_used"])

    def test_late_tx_and_reply_never_receive_bookkeeping_allowance(self):
        for changes, reason in (
            ({"final_write_ns": 1_020_000_001}, "hard20ms"),
            ({"last_reply_ns": 1_033_333_335}, "reply exceeded"),
            ({"oldest_input_ns": 1_001_000_000, "final_write_ns": 1_019_000_000,
              "sample_age_ns": 17_000_000}, "hard20ms"),
        ):
            with self.subTest(reason=reason), self.assertRaisesRegex(RuntimeError, reason):
                self.admit(elapsed=34_000_000, startup_allowed=True, **changes)
        self.assertEqual(self.budget.previous_index, -1)
        self.assertEqual(self.budget.accepted_misses, 0)

    def test_output_feedback_cannot_age_beyond_20ms(self):
        with self.assertRaisesRegex(RuntimeError, "sample-age"):
            self.admit(output_sample_start_ns=1_012_000_000, elapsed=33_000_001)

    def test_one_ms_late_bookkeeping_is_bounded_separately(self):
        on_time = self.admit()
        self.assertFalse(on_time["allowance_used"])
        result = self.admit(1, elapsed=34_333_334,
                            output_sample_start_ns=1_054_333_334)
        self.assertEqual(result["lateness_ms"], 1.)
        self.assertTrue(result["allowance_used"])
        self.assertEqual(self.budget.accepted_misses, 1)

    def test_lateness_cap_does_not_extend_reply_or_feedback_limits(self):
        with self.assertRaisesRegex(RuntimeError, "lateness"):
            self.admit(elapsed=34_333_335, output_sample_start_ns=1_015_000_000)

    def test_consecutive_and_rolling_misses_are_rejected_without_state_commit(self):
        self.admit(elapsed=33_433_334)
        with self.assertRaisesRegex(RuntimeError, "consecutive"):
            self.admit(1, elapsed=33_433_334)
        self.assertEqual(self.budget.previous_index, 0)
        self.admit(1)
        with self.assertRaisesRegex(RuntimeError, "rolling"):
            self.admit(2, elapsed=33_433_334)
        self.assertEqual(self.budget.previous_index, 1)
        self.assertEqual(tuple(self.budget.misses), (0,))

    def test_rolling_window_counts_100_cycles_and_startup_is_separate(self):
        result = self.admit(elapsed=33_433_334, startup_allowed=True)
        self.assertTrue(result["startup_allowance_used"])
        self.assertFalse(result["allowance_used"])
        self.admit(1, elapsed=33_433_334)
        for index in range(2, 101):
            self.admit(index)
        self.assertTrue(self.admit(101, elapsed=33_433_334)["allowance_used"])
        self.assertEqual(self.budget.accepted_misses, 2)

    def test_missing_duplicate_noncausal_bool_and_bad_age_rejected(self):
        changes = (
            {"index": 1}, {"index": True}, {"sample_age_ns": True},
            {"sample_age_ns": 0}, {"sample_age_ns": 20_000_001},
            {"last_reply_ns": 0}, {"last_reply_ns": math.nan},
            {"final_write_ns": 1_000_000_000},
            {"startup_allowed": "yes"},
        )
        for options in changes:
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.admit(**options)
        self.admit()
        with self.assertRaisesRegex(RuntimeError, "nonsequential"):
            self.admit()

    def test_only_explicit_30hz_and_unchanged_v1_settings_supported(self):
        for clock in (PolicyClock(50), None, 30):
            with self.subTest(clock=clock), self.assertRaises(ValueError):
                Pure30Budget(self.settings, clock)
        for key, value in (("max_lateness_ms", True), ("max_lateness_ms", math.nan),
                           ("max_lateness_ms", 1.0001), ("max_consecutive_misses", 2),
                           ("rolling_window_cycles", 99), ("max_misses_per_window", True),
                           ("mode", "bounded_post_reply_input_age_v2")):
            settings = {**self.settings, key: value}
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                Pure30Budget(settings, self.clock)
        self.settings["max_lateness_ms"] = .5
        self.assertEqual(self.budget.settings["max_lateness_ms"], 1.)
        with self.assertRaises(TypeError):
            self.budget.settings["max_lateness_ms"] = 2.
        with self.assertRaises(AttributeError):
            self.budget.settings = {**self.settings, "max_lateness_ms": 2.}
        with self.assertRaises(AttributeError):
            self.budget.clock = PolicyClock(50)


if __name__ == "__main__":
    unittest.main()
