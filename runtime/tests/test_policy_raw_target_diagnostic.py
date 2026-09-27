"""The model-to-raw bridge stays a bounded, file-only numerical diagnostic."""

import copy
import math
import unittest

from singularitydog_hw import policy_raw_target_diagnostic as bridge
from singularitydog_hw import policy_shadow as shadow
from test_policy_observer import calibration, make, reviewed_branch_snapshot, snapshot


def observed_pair(*, calib=None):
    calib = calib or calibration()
    original, review = reviewed_branch_snapshot(calib=calib)
    run = make(calibration=calib, max_ticks=1,
               power_epoch_branch_comparison=review)
    run.reset_run(original["tick_ns"], warmup_completed=True)
    tick = run.consume(original)
    return tick, original, calib


class PolicyRawTargetDiagnosticTests(unittest.TestCase):
    def test_real_observer_branch_tick_maps_back_to_current_raw_epoch_without_mutation(self):
        tick, original, calib = observed_pair()
        before = copy.deepcopy((tick, original, calib))
        report = bridge.project_raw_targets(tick, original, calib,
                                            max_abs_delta_rad=.2)
        self.assertEqual((tick, original, calib), before)
        self.assertEqual(report["status"], "RAW_TARGET_CANDIDATES_NO_OUTPUT")
        self.assertFalse(report["output_allowed"])
        self.assertFalse(report["motor_output_available"])
        self.assertFalse(report["can_frames_constructed"])
        self.assertEqual([row["motor_id"] for row in report["motor_rows_model_order"]],
                         shadow.CAN_ORDER)
        by_id = {row["motor_id"]: row for row in report["motor_rows_model_order"]}
        self.assertEqual(by_id[3]["reviewed_branch_turns"], 1)
        self.assertAlmostEqual(by_id[3]["raw_target_rad_diagnostic_only"],
                               2*math.pi + .03)
        self.assertAlmostEqual(by_id[3]["raw_delta_rad_diagnostic_only"],
                               by_id[3]["raw_target_rad_diagnostic_only"]
                               - by_id[3]["raw_current_rad"])
        self.assertEqual(report["source_snapshot_sha256"],
                         tick["provenance"]["snapshot_canonical_json_sha256"])

    def test_rejects_missing_or_changed_snapshot_and_power_epoch_binding(self):
        tick, original, calib = observed_pair()
        altered = copy.deepcopy(original)
        next(row for row in altered["motors"] if row["motor_id"] == 3
             and row["parameter"] == "position")["value"] += .01
        with self.assertRaisesRegex(ValueError, "exact snapshot"):
            bridge.project_raw_targets(tick, altered, calib, max_abs_delta_rad=.2)
        altered = copy.deepcopy(original)
        altered["source_flags"]["power_epoch_branch_capture"]["uid_read_motor_power_epoch"] = "old"
        tick = copy.deepcopy(tick)
        tick["provenance"]["snapshot_source_flags"] = copy.deepcopy(altered["source_flags"])
        tick["provenance"]["snapshot_canonical_json_sha256"] = bridge._digest(altered)
        with self.assertRaisesRegex(ValueError, "fresh UID or motor power-epoch"):
            bridge.project_raw_targets(tick, altered, calib, max_abs_delta_rad=.2)

    def test_rejects_calibration_mismatch_and_missing_branch_review(self):
        tick, original, calib = observed_pair()
        changed = copy.deepcopy(calib)
        changed["candidates"][0]["offset_candidate_rad"] += .01
        with self.assertRaisesRegex(ValueError, "calibration and exact snapshot"):
            bridge.project_raw_targets(tick, original, changed, max_abs_delta_rad=.2)
        plain = snapshot(calib=calib)
        run = make(calibration=calib, max_ticks=1)
        run.reset_run(plain["tick_ns"], warmup_completed=True)
        plain_tick = run.consume(plain)
        with self.assertRaisesRegex(ValueError, "Reviewed power-epoch"):
            bridge.project_raw_targets(plain_tick, plain, calib, max_abs_delta_rad=.2)

    def test_rejects_nonfinite_model_target_joint_range_and_excessive_delta(self):
        tick, original, calib = observed_pair()
        altered = copy.deepcopy(tick)
        altered["q_target_rad_diagnostic_only"][0] = math.nan
        with self.assertRaisesRegex(ValueError, "Nonfinite model target"):
            bridge.project_raw_targets(altered, original, calib, max_abs_delta_rad=.2)
        altered["q_target_rad_diagnostic_only"][0] = 2.
        with self.assertRaisesRegex(ValueError, "registered joint range"):
            bridge.project_raw_targets(altered, original, calib, max_abs_delta_rad=.2)
        with self.assertRaisesRegex(ValueError, "delta ceiling"):
            bridge.project_raw_targets(tick, original, calib, max_abs_delta_rad=.01)

    def test_rejects_raw_protocol_range_even_when_model_angle_is_in_range(self):
        calib = calibration()
        next(row for row in calib["candidates"] if row["motor_id"] == 6)[
            "offset_candidate_rad"] = 13.
        tick, original, calib = observed_pair(calib=calib)
        with self.assertRaisesRegex(ValueError, "RS05 encoding range"):
            bridge.project_raw_targets(tick, original, calib,
                                       max_abs_delta_rad=.2)


if __name__ == "__main__":
    unittest.main()
