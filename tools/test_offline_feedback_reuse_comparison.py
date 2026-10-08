"""Synthetic saved-trace scenarios, without real identity or hardware claims."""
import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "tests"))
from test_feedback_reuse_cadence import original_records, wire
import offline_feedback_reuse_comparison as comparison


def fixture():
    records, measurements = [], []
    for cycle in range(1, 4):
        release = 1_000_000_000 + (cycle - 1) * 20_000_000
        output = original_records(first=release + 10_000_000, mode=0, kind=4)
        records.append({"cycle": cycle, "output": {bus: {"records": rows}
            for bus, rows in output.items()}, "imu": {
                "sequence": cycle, "read_started_monotonic_ns": release + 100_000,
                "read_finished_monotonic_ns": release + 900_000,
                "accel_m_s2": [0., 0., -9.81], "gyro_rad_s": [0., 0., 0.]}})
        measurements.append({"release_ns": release,
            "final_host_write_ns": max(row["finish_ns"] for rows in output.values() for row in rows)})
    identities = {"front": [], "rear": []}
    for bus, ids in comparison.cadence.BUSES.items():
        for mid in ids:
            row = {"tx_hex": wire(0xFD << 8 | mid, bytes(8)).hex(),
                   "rx_hex": wire(mid << 8 | 0xFE, mid.to_bytes(8, "big")).hex(),
                   "start_ns": 1_000 + mid * 100, "finish_ns": 1_010 + mid * 100,
                   "received_ns": 1_020 + mid * 100, "deadline_ns": 100_000,
                   "written": 17, "received": 17}
            identities[bus].append({"records": [row]})
    report = {"measurements": measurements, "cycles_completed": 3,
        "status": "SYNTHETIC_NO_HARDWARE", "mode": "stop-proxy",
        "source_provenance": {"source_files_unchanged": True,
                              "cadence_source_sha256": {"synthetic.py": "a" * 64}},
        "identities": identities, "boot_id": "synthetic-boot",
        "motor_power_epoch": "synthetic-power",
        "plan": {"active_fk_profile": {"sha256": "b" * 64}}}
    return records, report


class OfflineFeedbackComparisonTests(unittest.TestCase):
    def test_naive_previous_cycle_copy_fails_but_conditional_projection_is_distinct(self):
        records, report = fixture()
        before = copy.deepcopy((records, report))
        result = comparison.compare(records, report, work_to_last_write_budget_ns=8_000_000)
        self.assertEqual((records, report), before)
        self.assertEqual(result["original_time_reuse_20ms_misses"], 2)
        self.assertEqual(result["independent_conditional_decision_counts"]["REUSE_14"], 2)
        self.assertFalse(result["steady_14_request_50hz_sequence_validated"])
        self.assertFalse(result["hardware_14_request_path_measured"])
        self.assertFalse(result["declared_budget_was_measured_on_14_request_path"])
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["original_timestamps_changed"])

    def test_inadequate_remaining_budget_chooses_all_fresh_requests(self):
        records, report = fixture()
        result = comparison.compare(records, report, work_to_last_write_budget_ns=12_000_000)
        self.assertEqual(result["independent_conditional_decision_counts"]["REFRESH_26"], 2)
        self.assertEqual(result["independent_conditional_decision_counts"]["REUSE_14"], 0)

    def test_incomplete_cycle_original_is_retained_without_promoting_completion(self):
        records, report = fixture()
        report["measurements"].pop()
        report["cycles_completed"] = 2
        report["status"] = "ABORTED"
        records[-1]["output"]["rear"]["records"][0]["received"] = 0
        records[-1]["output"]["front"]["records"] = []
        result = comparison.compare(records, report, work_to_last_write_budget_ns=8_000_000)
        self.assertEqual(result["incomplete_raw_cycles_not_promoted"], 1)
        self.assertEqual(result["recorded_raw_cycles_retained"], 3)
        self.assertEqual(result["independent_transitions_evaluated"], 1)
        self.assertEqual(result["original_report_status"], "ABORTED")
        self.assertEqual(result["incomplete_cycle_numbers_retained_not_evaluated"], [3])

    def test_fault_or_malformed_completed_prefix_cannot_be_hidden_as_incomplete(self):
        records, report = fixture()
        records[1]["output"]["rear"]["records"][0]["received"] = 0
        with self.assertRaises(ValueError):
            comparison.compare(records, report, work_to_last_write_budget_ns=8_000_000)

    def test_raw_final_write_mismatch_and_bad_identity_are_not_skipped(self):
        for identity in (False, True):
            records, report = fixture()
            if identity:
                report["identities"]["front"][0]["records"][0]["rx_hex"] = report["identities"]["front"][1]["records"][0]["rx_hex"]
            else:
                report["measurements"][1]["final_host_write_ns"] += 1
            with self.subTest(identity=identity), self.assertRaises(ValueError):
                comparison.compare(records, report, work_to_last_write_budget_ns=8_000_000)

    def test_missing_pins_no_completed_timings_or_relaxed_limit_rejected(self):
        for change in ("pin", "timing", "source", "budget"):
            records, report = fixture()
            budget = 8_000_000
            if change == "pin":
                report["plan"]["active_fk_profile"]["sha256"] = "not-a-pin"
            elif change == "timing":
                report["measurements"] = report["measurements"][:1]
            elif change == "source":
                report["source_provenance"]["source_files_unchanged"] = False
            else:
                budget = 20_000_001
            with self.subTest(change=change), self.assertRaises(ValueError):
                comparison.compare(records, report, work_to_last_write_budget_ns=budget)

    def test_cli_without_input_only_shows_usage_and_never_opens_hardware(self):
        result = subprocess.run([sys.executable, "-B", comparison.__file__, "--help"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn("conditional budget", result.stdout)

    def test_regular_bounded_json_reader_rejects_devices_symlinks_fifo_and_nan(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            regular = root / "records.json"
            regular.write_text('{"value":1}')
            raw, parsed = comparison._read_json(regular)
            self.assertEqual(parsed, {"value": 1})
            self.assertEqual(raw, regular.read_bytes())
            link = root / "link.json"
            link.symlink_to(regular)
            fifo = root / "fifo.json"
            os.mkfifo(fifo)
            for path in (Path("/dev/null"), link, fifo):
                with self.subTest(path=path), self.assertRaises(ValueError):
                    comparison._read_json(path)
            regular.write_text('{"value":NaN}')
            with self.assertRaises(ValueError):
                comparison._read_json(regular)


if __name__ == "__main__":
    unittest.main()
