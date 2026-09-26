"""Bounded observer copying retains guard and state contracts; no hardware."""
import copy
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer as observer
from singularitydog_hw.policy_snapshot_copy_replay import benchmark_snapshot
from test_policy_observer import make, snapshot, Policy


class PolicySnapshotCopyTests(unittest.TestCase):
    def test_owned_copy_matches_baseline_and_does_not_share_mutable_children(self):
        source = snapshot()
        nested = ["original", {"ok": True}]
        source["source_flags"] = {"first": nested, "second": nested}
        actual = observer._snapshot_copy(source)
        expected = copy.deepcopy(source)
        self.assertEqual(actual, expected)
        source["motors"][0]["value"] = 99
        source["imu"]["gyro_rad_s"].clear()
        nested[1]["ok"] = False
        self.assertEqual(actual, expected)
        actual["source_flags"]["first"][1]["ok"] = "changed"
        self.assertTrue(actual["source_flags"]["second"][1]["ok"])

    def test_full_observer_records_and_state_match_old_copy_for_both_hypotheses(self):
        for h in (0, 1):
            baseline, candidate = make(h_hypothesis=h, max_ticks=3), make(h_hypothesis=h, max_ticks=3)
            for run in (baseline, candidate):
                run.reset_run(1_000_000_000, warmup_completed=True)
            for index in range(3):
                source = snapshot(1_000_000_000+index*observer.DT_NS)
                source["source_flags"] = {"sequence": index, "nested": [False, {"candidate": True}]}
                source["imu"]["gyro_rad_s"][0] += .01*index
                with patch.object(observer, "_snapshot_copy", copy.deepcopy):
                    expected = baseline.consume(source)
                actual = candidate.consume(source)
                self.assertEqual(actual, expected)
                self.assertEqual(candidate._policy.previous, baseline._policy.previous)
                self.assertEqual(candidate._last_sources, baseline._last_sources)
                self.assertEqual(candidate.summary(), baseline.summary())

    def test_non_json_nonfinite_cyclic_and_oversized_sources_fail_before_forward(self):
        class CopyHook:
            def __deepcopy__(self, memo):
                raise AssertionError("Arbitrary deepcopy hook must not run")
        cycle = []
        cycle.append(cycle)
        deep = []
        for _ in range(30):
            deep = [deep]
        variants = (CopyHook(), math.nan, math.inf, cycle, deep, "x"*100_000,
                    [0]*20_001, {1: "non-string key"})
        for value in variants:
            with self.subTest(kind=type(value).__name__):
                source = snapshot()
                source["source_flags"] = {"bad": value}
                policy = Policy()
                run = make(policy, profile_consume=True)
                run.reset_run(source["tick_ns"], warmup_completed=True)
                with self.assertRaises(observer.ObserverError):
                    run.consume(source)
                self.assertEqual(policy.calls, [])
                self.assertEqual(run.ticks_completed, 0)
                self.assertEqual(run.status, "INCOMPLETE")
                self.assertEqual(run.summary()["last_consume_profile"]["failed_stage"], "snapshot_copy")

    def test_bounded_copy_failure_cannot_reuse_active_run(self):
        run = make()
        run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot()
        source["imu"]["accel_m_s2"][0] = math.nan
        with self.assertRaises(observer.ObserverError):
            run.consume(source)
        with self.assertRaisesRegex(observer.ObserverError, "inactive or invalid"):
            run.consume(snapshot())

    def test_cpu_replay_keeps_original_values_and_records_both_costs(self):
        source = snapshot()
        original = copy.deepcopy(source)
        result = benchmark_snapshot(source, repeats=4, warmup=0)
        self.assertEqual(source, original)
        self.assertTrue(result["snapshots_exactly_equal"])
        self.assertTrue(result["original_timestamps_preserved"])
        self.assertEqual(set(result["timings"]), {"deepcopy", "bounded_json_copy"})
        self.assertTrue(all(value["wall"]["samples"] == 4 for value in result["timings"].values()))
        for repeats in (True, 0, 100_001):
            with self.assertRaises(ValueError):
                benchmark_snapshot(source, repeats=repeats)


if __name__ == "__main__":
    unittest.main()
