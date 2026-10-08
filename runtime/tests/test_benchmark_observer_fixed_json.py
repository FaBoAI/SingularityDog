"""Offline benchmark selection/fallback and capture checks; no live device IO."""
import copy
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer as observer
import test_policy_observer as fixtures


PATH = Path(__file__).resolve().parents[2]/"tools"/"benchmark_observer_fixed_json.py"
SPEC = importlib.util.spec_from_file_location("observer_fixed_json_benchmark", PATH)
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


class FixedJsonBenchmarkTests(unittest.TestCase):
    def test_exact_copy_checks_and_report_do_not_grant_runtime_approval(self):
        shared = [{"unknown": False, "float": -0.0}]
        value = {"tuple": (shared,), "another": shared, "immutable": "source"}
        rows = [{"calibration_source_flags": copy.deepcopy(value)} for _ in range(3)]
        before = copy.deepcopy(rows)
        report = benchmark.benchmark_copies(rows, iterations=2, trials=2)
        self.assertEqual(rows, before)
        copied = report["calibration_source_flags"]
        self.assertTrue(copied["factory_selected"])
        self.assertEqual(copied["records_exact_checked"], 3)
        self.assertEqual(copied["canonical_json_sha256"], observer._digest(value))
        self.assertEqual(set(copied["ns_per_copy_trials"]), {"marshal", "literal_factory"})
        self.assertTrue(all(len(v) == 2 for v in copied["ns_per_copy_trials"].values()))
        self.assertEqual(copied["measurement_order"], [["marshal", "literal_factory"],
                                                      ["literal_factory", "marshal"]])

    def test_large_legacy_copy_fallback_is_reported_truthfully(self):
        result = benchmark.benchmark_copies([{"gyro_bias_hypothesis": {"notes": "x"*4097}}],
                                            iterations=2, trials=2)
        self.assertFalse(result["gyro_bias_hypothesis"]["factory_selected"])
        self.assertEqual(result["gyro_bias_hypothesis"]["records_exact_checked"], 1)

    def test_changed_selection_or_static_float_bits_fail_before_timing(self):
        for rows in ([{"calibration_source_flags": {}}, {}],
                     [{"calibration_source_flags": {"zero": 0.0}},
                      {"calibration_source_flags": {"zero": -0.0}}]):
            with self.assertRaises(ValueError):
                benchmark.benchmark_copies(rows, iterations=2, trials=2)

    def test_copy_shape_alias_or_negative_zero_mismatch_is_detected(self):
        original = {"calibration_source_flags": {"zero": -0.0, "child": [1]}}
        with patch.object(observer, "_frozen_json_literal_factory", return_value=lambda: {"zero": 0.0, "child": [1]}):
            with self.assertRaisesRegex(ValueError, "float bits"):
                benchmark.benchmark_copies([original], iterations=2, trials=2)
        owned = original["calibration_source_flags"]
        with patch.object(observer, "_frozen_json_literal_factory", return_value=lambda: owned):
            with self.assertRaisesRegex(ValueError, "source alias"):
                benchmark.benchmark_copies([original], iterations=2, trials=2)
        repeated = {"zero": -0.0, "child": [1]}
        with patch.object(observer, "_frozen_json_literal_factory", return_value=lambda: repeated):
            with self.assertRaisesRegex(ValueError, "prior-record alias"):
                benchmark.benchmark_copies([original, copy.deepcopy(original)], iterations=2, trials=2)

    def test_timing_counts_are_bounded_before_any_copy_plan(self):
        for iterations, trials in ((0, 2), (1_000_001, 2), (2, 0), (2, 32), (True, 2)):
            with patch.object(observer, "_frozen_json_literal_factory") as factory:
                with self.assertRaises(ValueError):
                    benchmark.benchmark_copies([{}], iterations=iterations, trials=trials)
                factory.assert_not_called()

    def test_restore_checks_frames_unsigned_fields_and_the_saved_snapshot_sha(self):
        snapshot = fixtures.snapshot()
        snapshot["source_flags"] = {}
        def saved():
            records = [{"start_ns": 1, "finish_ns": 2, "read_start_ns": 3, "received_ns": 4,
                        "deadline_ns": 5, "written": 17, "received": 17,
                        "tx_hex": "00"*17, "rx_hex": "00"*17} for _ in range(6)]
            result = {"acquired": {scope: {"records": copy.deepcopy(records)} for scope in ("front", "rear")},
                      "imu": {}, "observed": {"tick_ns": snapshot["tick_ns"], "provenance": {}}}
            s = copy.deepcopy(snapshot); s["source_flags"] = {"v3_voltage_overlap_pending_at_inference": True}
            result["observed"]["provenance"]["snapshot_canonical_json_sha256"] = observer._digest(s)
            return result
        with patch.object(benchmark.pipeline, "snapshot_from_records", side_effect=lambda *a: copy.deepcopy(snapshot)):
            restored = benchmark.restore_snapshot(saved())
            self.assertIs(restored["source_flags"]["v3_voltage_overlap_pending_at_inference"], True)
            row = saved(); row["observed"]["provenance"]["snapshot_canonical_json_sha256"] = "0"*64
            with self.assertRaisesRegex(ValueError, "canonical SHA"):
                benchmark.restore_snapshot(row)
            for key, value in (("start_ns", True), ("received_ns", 2**64), ("written", -1)):
                row = saved(); row["acquired"]["front"]["records"][0][key] = value
                with self.assertRaisesRegex(ValueError, "native field"):
                    benchmark.restore_snapshot(row)
            row = saved(); row["acquired"]["front"]["records"][0]["rx_hex"] = "00"*16
            with self.assertRaisesRegex(ValueError, "Complete saved AT"):
                benchmark.restore_snapshot(row)


if __name__ == "__main__":
    unittest.main()
