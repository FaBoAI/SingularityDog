"""Synthetic saved-file tests for the offline STOP-proxy range report."""

import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw import stop_proxy_range_report as ranges


def calibration():
    rows = []
    for index, mid in enumerate(ranges.MODEL_ORDER):
        leg, joint = ranges.EXPECTED_LABELS[mid]
        midpoint = (ranges.LOWER[index] + ranges.UPPER[index]) / 2
        rows.append({"motor_id": mid, "leg": leg, "joint": joint,
                     "sign_candidate": 1, "offset_candidate_rad": midpoint - 0.1,
                     "approved_for_runtime": False})
    return {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
            "formula": "q_model = sign * raw + offset; rad; no wrapping",
            "model_can_order_candidate": list(ranges.MODEL_ORDER),
            "identities": {str(mid): f"{mid:016x}" for mid in range(1, 13)},
            "approved_for_runtime": False, "candidates": rows}


def reply(mid, *, cycle=1, position=0.1):
    return {"kind": "pipeline_reply", "cycle": cycle, "parameter": "stop_feedback",
            "motor_id": mid, "ok": True,
            "result": {"motor_id": mid, "parameter": "stop_feedback",
                       "ok": True, "position_rad_candidate": position}}


def write_inputs(root, front=None, rear=None, candidate=None):
    root = Path(root)
    paths = [root / "front.jsonl", root / "rear.jsonl", root / "calibration.json"]
    if front is None:
        front = [reply(mid) for mid in range(1, 7)]
    if rear is None:
        rear = [reply(mid) for mid in range(7, 13)]
    for path, rows in zip(paths[:2], (front, rear)):
        path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    paths[2].write_text(json.dumps(calibration() if candidate is None else candidate))
    return paths


def by_id(report):
    return {row["motor_id"]: row for row in report["joints"]}


class StopProxyRangeReportTests(unittest.TestCase):
    def test_exact_model_order_limits_and_unverified_status(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = ranges.build_report(*write_inputs(tmp))
        self.assertEqual([row["motor_id"] for row in report["joints"]],
                         [6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7])
        self.assertEqual(report["blocked_motor_ids"], [])
        self.assertEqual(report["input_blockers"], [])
        self.assertEqual(report["status"], "OFFLINE_RANGE_WITHIN_LIMITS_UNVERIFIED")
        self.assertTrue(report["range_screen_passed"])
        self.assertFalse(report["calibration_verified"])
        self.assertFalse(report["output_allowed"])
        self.assertFalse(report["angle_wrapping_applied"])
        self.assertFalse(report["clipping_applied"])
        for index, row in enumerate(report["joints"]):
            self.assertEqual(row["raw_angle_rad"], 0.1)
            self.assertAlmostEqual(row["calibrated_angle_rad"],
                                   (ranges.LOWER[index] + ranges.UPPER[index]) / 2)
            self.assertEqual((row["fixed_policy_lower_rad"], row["fixed_policy_upper_rad"]),
                             (ranges.LOWER[index], ranges.UPPER[index]))
            self.assertTrue(row["within_fixed_policy_limits"])

    def test_outside_limits_are_reported_without_turn_correction(self):
        front = [reply(mid) for mid in range(1, 7)]
        rear = [reply(mid) for mid in range(7, 13)]
        front[0] = reply(1, position=2 * math.pi + 0.2)
        front[5] = reply(6, position=1.0)
        rear[8 - 7] = reply(8, position=-3.5)
        with tempfile.TemporaryDirectory() as tmp:
            report = ranges.build_report(*write_inputs(tmp, front, rear))
        rows = by_id(report)
        self.assertEqual(rows[1]["raw_angle_rad"], 2 * math.pi + 0.2)
        self.assertAlmostEqual(rows[1]["calibrated_angle_rad"],
                               2 * math.pi + 0.2 + calibration()["candidates"][5]["offset_candidate_rad"])
        self.assertAlmostEqual(rows[1]["raw_angle_deg"], 360 + math.degrees(0.2))
        self.assertIn("above_fixed_policy_upper_limit", rows[1]["blockers"])
        self.assertIn("above_fixed_policy_upper_limit", rows[6]["blockers"])
        self.assertIn("below_fixed_policy_lower_limit", rows[8]["blockers"])
        self.assertFalse(rows[1]["within_fixed_policy_limits"])
        self.assertEqual(set(report["blocked_motor_ids"]), {1, 6, 8})
        self.assertEqual(report["status"], "OFFLINE_RANGE_BLOCKED")
        self.assertFalse(report["range_screen_passed"])

    def test_missing_duplicate_wrong_bus_and_unsuccessful_are_explicit(self):
        front = [reply(mid) for mid in range(1, 7) if mid != 2]
        front.append(reply(7))  # Recorded on the wrong bus.
        rear = [reply(mid) for mid in range(8, 13)]
        rear[0]["ok"] = False
        rear.append(copy.deepcopy(rear[-1]))
        with tempfile.TemporaryDirectory() as tmp:
            report = ranges.build_report(*write_inputs(tmp, front, rear))
        rows = by_id(report)
        self.assertEqual(rows[2]["blockers"], ["missing_cycle_1_stop_feedback"])
        self.assertIn("wrong_bus_for_motor_id", rows[7]["blockers"])
        self.assertIn("unsuccessful_or_missing_decoded_result", rows[8]["blockers"])
        self.assertEqual(rows[12]["blockers"], ["duplicate_cycle_1_stop_feedback"])
        self.assertIsNone(rows[12]["calibrated_angle_rad"])

    def test_later_proxy_cycle_cannot_fill_missing_input(self):
        front = [reply(mid) for mid in range(1, 6)] + [reply(6, cycle=2)]
        with tempfile.TemporaryDirectory() as tmp:
            report = ranges.build_report(*write_inputs(tmp, front))
        self.assertEqual(by_id(report)[6]["blockers"], ["missing_cycle_1_stop_feedback"])

    def test_decoded_motor_id_must_be_an_integer(self):
        for invalid in (True, 1.0):
            front = [reply(mid) for mid in range(1, 7)]
            front[0]["result"]["motor_id"] = invalid
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as tmp:
                report = ranges.build_report(*write_inputs(tmp, front))
                self.assertIn("decoded_result_id_or_parameter_mismatch",
                              by_id(report)[1]["blockers"])

    def test_candidate_schema_mismatch_and_nonfinite_log_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = calibration()
            candidate["candidates"][0]["joint"] = "calf"
            paths = write_inputs(tmp, candidate=candidate)
            with self.assertRaisesRegex(ValueError, "leg/joint"):
                ranges.build_report(*paths)
            front = [reply(mid) for mid in range(1, 7)]
            front[0]["result"]["position_rad_candidate"] = float("nan")
            paths = write_inputs(tmp, front=front)
            with self.assertRaisesRegex(ValueError, "Nonfinite JSON number"):
                ranges.build_report(*paths)

    def test_cli_only_prints_report_and_leaves_saved_files_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_inputs(tmp)
            before = {path.name: path.read_bytes() for path in paths}
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                status = ranges.main(["--front-jsonl", str(paths[0]),
                                      "--rear-jsonl", str(paths[1]),
                                      "--calibration", str(paths[2])])
            self.assertEqual(status, 0)
            self.assertEqual(set(Path(tmp).iterdir()), set(paths))
            self.assertEqual({path.name: path.read_bytes() for path in paths}, before)
            self.assertFalse(json.loads(output.getvalue())["output_allowed"])


if __name__ == "__main__":
    unittest.main()
