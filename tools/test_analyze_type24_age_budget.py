"""Synthetic timing checks for file-only Type24 age-budget arithmetic."""

import copy
import unittest

from analyze_type24_age_budget import summarize


def fixture():
    # Millisecond timestamps expressed in nanoseconds. Post-gather work is
    # 10 ms to final write, 12 ms to reply, and 13 ms to cycle completion.
    row = dict(zip((
        "release_ns", "oldest_input_start_ns", "input_latest_reply_ns", "gather_end_ns",
        "prepare_end_ns", "infer_end_ns", "final_host_write_ns",
        "last_proxy_reply_ns", "cycle_end_ns",
    ), (1, 1, 7, 8, 9, 12, 18, 20, 21)))
    row = {key: value * 1_000_000 for key, value in row.items()}
    return {
        "status": "COMPLETE_DIAGNOSTIC", "mode": "stop-proxy",
        "motor_enable_sent": False, "learned_targets_sent": False,
        "cycles_completed": 1, "identities": {"front": {}, "rear": {}},
        "measurements": [row], "host_deadline_misses": 0,
        "iteration_deadline_misses": 0,
    }


class Type24AgeBudgetTest(unittest.TestCase):
    def test_counterfactual_keeps_actual_acquisition_and_deadlines_distinct(self):
        result = summarize(fixture())
        self.assertEqual(result["baseline_acquisition_ms"]["median"], 7)
        self.assertEqual(result["maximum_oldest_report_age_at_gather_end_ms"]["oldest_report_to_cycle_end"]["median"], 7)
        self.assertEqual(result["hypothetical_deadline_misses_at_fixed_oldest_report_age_ms"]["oldest_report_to_cycle_end"]["8"], 1)
        self.assertEqual(result["hypothetical_deadline_misses_at_fixed_oldest_report_age_ms"]["oldest_report_to_cycle_end"]["7"], 0)
        self.assertFalse(result["type24_stream_measured"])
        self.assertFalse(result["type24_full_cycle_verified"])

    def test_active_or_incomplete_run_is_rejected(self):
        report = fixture()
        report["motor_enable_sent"] = True
        with self.assertRaisesRegex(ValueError, "disabled"):
            summarize(report)
        report = fixture()
        report["cycles_completed"] = 2
        with self.assertRaisesRegex(ValueError, "cycle count"):
            summarize(report)

    def test_noncausal_or_inconsistent_timing_is_rejected(self):
        report = fixture()
        report["measurements"][0]["last_proxy_reply_ns"] = 17_000_000
        with self.assertRaisesRegex(ValueError, "Noncausal"):
            summarize(report)
        report = copy.deepcopy(fixture())
        report["iteration_deadline_misses"] = 1
        with self.assertRaisesRegex(ValueError, "disagree"):
            summarize(report)


if __name__ == "__main__":
    unittest.main()
