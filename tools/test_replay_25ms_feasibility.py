"""Synthetic, file-only checks for the 25 ms threshold comparison."""

import copy
import hashlib
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from tools.replay_25ms_feasibility import analyze, main


def saved_report():
    rows = []
    for index, (duration, output_age, reply) in enumerate((
            (19.5, 16.0, 18.7), (20.2, 17.0, 19.9),
            (24.4, 19.8, 23.9), (25.1, 24.8, 24.9))):
        begin = 1_000_000_000 + index * 20_000_000
        rows.append({"index": index, "begin_ns": begin,
                     "end_ns": begin + int(duration * 1e6),
                     "output_reply_end_ns": begin + int(reply * 1e6),
                     "iteration_ms": duration,
                     "oldest_input_to_final_host_write_ms": output_age})
    return {
        "status": "ABORTED", "cycles": rows,
        "telemetry_cadence": {"voltage_rotation_length_cycles": 6},
        "voltage_guard": {"maximum_age_ms": 126.0},
    }


class ReplayTests(unittest.TestCase):
    def test_separates_measured_20ms_misses_from_counterfactual_25ms_threshold(self):
        source = saved_report()
        before = copy.deepcopy(source)
        result = analyze(source)
        self.assertEqual(source, before)
        self.assertEqual(result["recorded_cycles"], 4)
        measured = result["measurements"]
        self.assertEqual(measured["begin_to_cycle_end"]["over_20ms"], 3)
        self.assertEqual(measured["begin_to_cycle_end"]["over_25ms"], 1)
        self.assertEqual(measured["oldest_input_to_final_host_write"]["over_20ms"], 1)
        self.assertEqual(measured["oldest_input_to_final_host_write"]["over_25ms"], 0)
        self.assertEqual(measured["begin_to_last_output_reply"]["over_20ms"], 2)
        self.assertEqual(measured["begin_to_next_begin"]["count"], 3)
        self.assertFalse(result["actual_25ms_cadence_measured"])
        self.assertFalse(result["learned_policy_40hz_validated"])
        self.assertFalse(result["supported_25ms_output_approved"])

    def test_model_profile_sample_and_voltage_blockers_remain_even_if_all_under_25ms(self):
        source = saved_report()
        source["cycles"] = source["cycles"][:2]
        result = analyze(source)
        blockers = {row["code"]: row for row in result["blockers"]}
        self.assertIn("PINNED_MODEL_DT_20MS", blockers)
        self.assertIn("SUPPORTED_PROFILE_PERIOD_20MS", blockers)
        self.assertEqual(blockers["REVIEWED_INPUT_AGE_20MS"]["reviewed_max_sample_age_ms"], 20)
        self.assertEqual(blockers["REVIEWED_SAMPLE_GAP_21MS"]["candidate_gap_ms"], 25)
        self.assertEqual(blockers["VOLTAGE_ROTATION_25MS_EXCEEDS_CACHE_AGE"]["projected_refresh_interval_ms"], 150)
        self.assertEqual(blockers["VOLTAGE_ROTATION_25MS_EXCEEDS_CACHE_AGE"]["reviewed_maximum_age_ms"], 126.0)
        self.assertEqual(result["measurements"]["begin_to_cycle_end"]["over_25ms"], 0)
        self.assertFalse(result["supported_25ms_output_approved"])

    def test_missing_voltage_evidence_is_unknown_not_assumed_safe(self):
        source = saved_report()
        source.pop("voltage_guard")
        result = analyze(source)
        self.assertIn("VOLTAGE_CADENCE_UNKNOWN", [row["code"] for row in result["blockers"]])

    def test_rejects_impossible_timestamps_or_forged_duration(self):
        for mutation in (lambda r: r["cycles"][0].update(output_reply_end_ns=r["cycles"][0]["end_ns"] + 1),
                         lambda r: r["cycles"][1].update(iteration_ms=18.0),
                         lambda r: r["cycles"][2].update(begin_ns=r["cycles"][1]["begin_ns"]),
                         lambda r: r["cycles"][0].update(oldest_input_to_final_host_write_ms=float("nan"))):
            source = saved_report()
            mutation(source)
            with self.subTest(source=source["cycles"]), self.assertRaises(ValueError):
                analyze(source)

    def test_cli_reads_only_saved_path_and_emits_digest(self):
        source = saved_report()
        raw = json.dumps(source).encode()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            path.write_bytes(raw)
            output = io.StringIO()
            with redirect_stdout(output):
                main([str(path)])
            self.assertEqual(path.read_bytes(), raw)
        result = json.loads(output.getvalue())
        self.assertEqual(result["input_report_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertFalse(result["supported_25ms_output_approved"])


if __name__ == "__main__":
    unittest.main()
