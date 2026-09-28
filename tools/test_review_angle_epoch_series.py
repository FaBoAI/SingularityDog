"""File-only tests of the twelve-axis power-epoch numeric comparison."""

import json
import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from audit_angle_calibration import load_profile
from review_angle_epoch_series import review
from test_audit_angle_calibration import capture_fixture, profile_fixture


def capture_series(root, updates):
    source = json.loads(capture_fixture(root).read_text())
    paths = []
    for index, changes in enumerate(updates):
        value = json.loads(json.dumps(source))
        # Distinct labels identify files, but are not evidence of a power cycle.
        value["motor_power_epoch"] = f"synthetic-capture-{index}"
        for mid, position in changes.items():
            value["telemetry"]["rows"][str(mid)]["median_position_rad"] = position
        path = root / f"series-{index}.json"
        path.write_text(json.dumps(value))
        paths.append(path)
    return paths


class EpochSeriesTests(unittest.TestCase):
    def test_positive_and_negative_full_turns_are_visible_without_approval(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            first = capture_fixture(root)
            second = root / "second.json"
            value = json.loads(first.read_text())
            value["boot_id"] = "different-boot"
            value["telemetry"]["rows"]["3"]["median_position_rad"] += 2 * math.pi
            value["telemetry"]["rows"]["9"]["median_position_rad"] -= 2 * math.pi
            second.write_text(json.dumps(value))
            result = review(axes, [first, second])
            self.assertEqual(result["status"], "NUMERIC_COMPARISON_ONLY")
            self.assertEqual(result["rows_by_id"]["3"]["first_to_last_branch_turn_delta"], 1)
            self.assertEqual(result["rows_by_id"]["9"]["first_to_last_branch_turn_delta"], -1)
            self.assertAlmostEqual(result["rows_by_id"]["3"]["first_to_last_model_delta_deg_if_numeric"], 0)
            self.assertAlmostEqual(result["rows_by_id"]["9"]["first_to_last_model_delta_deg_if_numeric"], 0)
            self.assertFalse(result["pose_equivalence_verified"])
            self.assertFalse(result["physical_zero_direction_or_limits_verified_by_this_tool"])
            self.assertFalse(result["output_allowed"])

    def test_five_transitions_include_full_turn_return_hidden_by_endpoints(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, evidence = profile_fixture(root)
            _, axes = load_profile(profile)
            captures = capture_series(root, [{}, {3: .1 + 2 * math.pi}, {}, {}, {}, {}])
            original = {path: path.read_bytes() for path in [profile, evidence, *captures]}
            original_axes = axes.copy()
            result = review(axes, captures)
            row = result["rows_by_id"]["3"]
            transitions = row["adjacent_transitions"]
            self.assertEqual(len(transitions), 5)
            self.assertEqual([pair["branch_turn_delta_if_numeric"] for pair in transitions],
                             [1, -1, 0, 0, 0])
            for pair, expected in zip(transitions, [360, -360, 0, 0, 0]):
                self.assertAlmostEqual(pair["raw_delta_deg"], expected)
                self.assertAlmostEqual(pair["model_delta_deg_if_numeric"], 0)
                self.assertTrue(pair["unique_numeric_branch_in_both_captures"])
                self.assertEqual(pair["from_capture_sha256"], result["capture_sources"][
                    pair["from_capture_index"]]["sha256"])
                self.assertEqual(pair["to_capture_sha256"], result["capture_sources"][
                    pair["to_capture_index"]]["sha256"])
            self.assertEqual(row["first_to_last_branch_turn_delta"], 0)
            self.assertAlmostEqual(row["first_to_last_raw_delta_deg"], 0)
            summary = row["series_summary"]
            self.assertEqual(summary["whole_turn_changed_transition_count"], 2)
            self.assertEqual(summary["whole_turn_changed_transition_indexes"], [0, 1])
            self.assertEqual(summary["numeric_adjacent_transition_count"], 5)
            self.assertEqual(summary["numeric_comparison_to_first_count"], 5)
            self.assertAlmostEqual(summary["max_abs_adjacent_model_delta_deg_if_numeric"], 0)
            self.assertAlmostEqual(summary["max_abs_model_delta_vs_first_deg_if_numeric"], 0)
            series = result["series_summary"]
            self.assertEqual(series["capture_count"], 6)
            self.assertEqual(series["adjacent_transition_count"], 5)
            self.assertEqual(series["whole_turn_changed_transition_count"], 2)
            self.assertEqual(series["whole_turn_change_axis_event_count"], 2)
            self.assertEqual([pair["whole_turn_changed_ids"] for pair in series["adjacent_transitions"]],
                             [[3], [3], [], [], []])
            for flag in ("power_off_on_observed_by_this_tool", "pose_equivalence_verified",
                         "physical_zero_direction_or_limits_verified_by_this_tool",
                         "motor_targets_generated", "approved_for_runtime", "output_allowed"):
                self.assertFalse(result[flag])
            self.assertEqual({path: path.read_bytes() for path in original}, original)
            self.assertEqual(axes, original_axes)

    def test_whole_turn_and_real_motion_keep_signed_deltas_and_intermediate_maxima(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            axes[7] = replace(axes[7], sign=-1)
            captures = capture_series(root, [{}, {7: .3 + 2 * math.pi},
                                             {7: -.2 + 2 * math.pi}, {}])
            result = review(axes, captures)
            row = result["rows_by_id"]["7"]
            self.assertAlmostEqual(row["first_to_last_model_delta_deg_if_numeric"], 0)
            for pair, expected in zip(row["adjacent_transitions"], [-.2, .5, -.3]):
                self.assertAlmostEqual(pair["model_delta_deg_if_numeric"], math.degrees(expected))
            self.assertEqual([pair["branch_turn_delta_if_numeric"] for pair in row["adjacent_transitions"]],
                             [1, 0, -1])
            self.assertAlmostEqual(row["adjacent_transitions"][0]["raw_delta_deg"],
                                   360 + math.degrees(.2))
            for summary in (row["series_summary"], result["series_summary"]):
                self.assertAlmostEqual(summary["max_abs_adjacent_model_delta_deg_if_numeric"],
                                       math.degrees(.5))
                self.assertAlmostEqual(summary["max_abs_model_delta_vs_first_deg_if_numeric"],
                                       math.degrees(.3))

    def test_mismatched_first_uid_invalidates_one_pair_without_hiding_later_pairs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            captures = capture_series(root, [{}, {}, {4: .1 + 2 * math.pi}, {}])
            value = json.loads(captures[0].read_text())
            value["identities"]["4"]["mcu_uid_hex"] = "f" * 16
            captures[0].write_text(json.dumps(value))
            result = review(axes, captures)
            row = result["rows_by_id"]["4"]
            pairs = row["adjacent_transitions"]
            self.assertEqual([pair["uid_matches_both_captures"] for pair in pairs], [False, True, True])
            self.assertEqual([pair["branch_turn_delta_if_numeric"] for pair in pairs], [None, 1, -1])
            self.assertIsNone(pairs[0]["model_delta_deg_if_numeric"])
            self.assertAlmostEqual(pairs[0]["raw_delta_deg"], 0)
            self.assertFalse(row["unique_numeric_branch_in_every_capture"])
            self.assertIsNone(row["first_to_last_branch_turn_delta"])
            self.assertIsNone(row["first_to_last_model_delta_deg_if_numeric"])
            summary = row["series_summary"]
            self.assertEqual(summary["uid_mismatch_capture_indexes"], [0])
            self.assertEqual(summary["invalid_adjacent_transition_indexes"], [0])
            self.assertEqual(summary["numeric_adjacent_transition_count"], 2)
            self.assertEqual(summary["whole_turn_changed_transition_count"], 2)
            self.assertEqual(summary["numeric_comparison_to_first_count"], 0)
            self.assertEqual(summary["invalid_comparison_to_first_capture_indexes"], [1, 2, 3])
            self.assertIsNone(summary["max_abs_model_delta_vs_first_deg_if_numeric"])
            self.assertEqual(result["uid_mismatch_ids"], [4])
            self.assertEqual([pair["uid_mismatch_ids"] for pair in result["series_summary"]["adjacent_transitions"]],
                             [[4], [], []])

    def test_invalid_middle_capture_only_invalidates_touching_pairs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            captures = capture_series(root, [{}, {3: .495, 4: 1.}, {}, {3: .3, 4: .3}])
            # ID3 has one candidate whose uncertainty crosses a limit; ID4 has none.
            result = review(axes, captures)
            self.assertEqual(result["nonunique_branch_ids"], [3, 4])
            for mid in (3, 4):
                row = result["rows_by_id"][str(mid)]
                pairs = row["adjacent_transitions"]
                self.assertEqual([pair["branch_turn_delta_if_numeric"] for pair in pairs], [None, None, 0])
                self.assertTrue(all(pair["uid_matches_both_captures"] for pair in pairs))
                self.assertAlmostEqual(pairs[2]["model_delta_deg_if_numeric"], math.degrees(.2))
                summary = row["series_summary"]
                self.assertEqual(summary["invalid_adjacent_transition_indexes"], [0, 1])
                self.assertEqual(summary["nonunique_numeric_branch_capture_indexes"], [1])
                self.assertEqual(summary["numeric_adjacent_transition_count"], 1)
                self.assertEqual(summary["numeric_comparison_to_first_count"], 2)
                self.assertEqual(summary["invalid_comparison_to_first_capture_indexes"], [1])
                self.assertAlmostEqual(summary["max_abs_model_delta_vs_first_deg_if_numeric"], math.degrees(.2))

    def test_ambiguous_branches_have_no_numeric_maximum(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            axes[1] = replace(axes[1], lower_rad=-7., upper_rad=7.)
            captures = capture_series(root, [{}, {1: .3}])
            result = review(axes, captures)
            row = result["rows_by_id"]["1"]
            self.assertTrue(all(len(capture["branch_candidates"]) > 1 for capture in row["captures"]))
            pair = row["adjacent_transitions"][0]
            self.assertAlmostEqual(pair["raw_delta_deg"], math.degrees(.2))
            self.assertIsNone(pair["branch_turn_delta_if_numeric"])
            self.assertIsNone(pair["model_delta_deg_if_numeric"])
            summary = row["series_summary"]
            self.assertEqual(summary["numeric_adjacent_transition_count"], 0)
            self.assertEqual(summary["numeric_comparison_to_first_count"], 0)
            self.assertIsNone(summary["max_abs_adjacent_model_delta_deg_if_numeric"])
            self.assertIsNone(summary["max_abs_model_delta_vs_first_deg_if_numeric"])
            self.assertEqual(result["nonunique_branch_ids"], [1])

    def test_changed_uid_and_real_pose_delta_remain_distinct(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            first = capture_fixture(root)
            second = root / "second.json"
            value = json.loads(first.read_text())
            value["identities"]["4"]["mcu_uid_hex"] = "f" * 16
            value["telemetry"]["rows"]["7"]["median_position_rad"] += .2
            second.write_text(json.dumps(value))
            result = review(axes, [first, second])
            self.assertEqual(result["uid_mismatch_ids"], [4])
            self.assertEqual(result["nonunique_branch_ids"], [4])
            self.assertAlmostEqual(result["rows_by_id"]["7"]["first_to_last_model_delta_deg_if_numeric"],
                                   math.degrees(.2))
            self.assertIsNone(result["rows_by_id"]["4"]["first_to_last_branch_turn_delta"])

    def test_repeated_capture_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, axes = load_profile(profile)
            capture = capture_fixture(root)
            with self.assertRaisesRegex(ValueError, "repeated"):
                review(axes, [capture, capture])


if __name__ == "__main__":
    unittest.main()
