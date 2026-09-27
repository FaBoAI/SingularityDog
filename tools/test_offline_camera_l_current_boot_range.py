"""Synthetic file-only checks for the cross-boot camera-L range review."""

import copy
import math
import unittest

from tools import offline_camera_l_current_boot_range as screen


def fixture():
    uids = {str(i): f"{i:016x}" for i in screen.IDS}
    cameras = {}
    for leg, ids in screen.IDS_BY_LEG.items():
        cameras[leg] = {
            "status": f"READ_ONLY_{leg}_CAMERA_L", "boot_id": "old-boot",
            "motor_output_allowed": False,
            "rows": {str(i): {"uid": uids[str(i)], "run_mode": 0,
                              "current_A": 0.0, "position_median_rad": 0.0,
                              "position_span_deg": 0.01} for i in ids},
        }
    current = {
        "status": "RECORDED_REVIEW_REQUIRED", "errors": [],
        "motor_output_allowed": False, "angle_wrap_applied": False,
        "boot_id": "new-boot", "motor_power_epoch": "unverified-epoch",
        "identities": {key: {"mcu_uid_hex": uid} for key, uid in uids.items()},
        "telemetry": {"rows": {str(i): {"run_mode": 0, "current": 0.0,
                                             "position_span_deg": 0.01,
                                             "median_position_rad": 0.0}
                               for i in screen.IDS}},
    }
    current["telemetry"]["rows"]["3"]["median_position_rad"] = (
        screen.TWO_PI + math.radians(2))
    historical = {
        "status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
        "formula": "q_model = sign * raw + offset; rad; no wrapping",
        "model_can_order_candidate": screen.MODEL_CAN_ORDER,
        "identities": uids,
        "candidates": [{"motor_id": i, "sign_candidate": -1 if i == 3 else 1}
                       for i in screen.IDS],
    }
    hashes = {**{leg: leg.lower() for leg in screen.IDS_BY_LEG},
              "current": "current", "historical_signs": "historical"}
    return cameras, current, historical, hashes


class CameraLCurrentBootRangeTests(unittest.TestCase):
    def review(self, camera, current, historical, hashes, *, claim=True):
        return screen.build_review(
            camera, current, historical, operator_no_turn_id3=claim,
            operator_statement="Operator saw no complete ID3 output-shaft turn.",
            source_hashes=hashes)

    def test_branch_is_comparison_only_and_raw_remains_unchanged(self):
        camera, current, historical, hashes = fixture()
        before = copy.deepcopy(current)
        report = self.review(camera, current, historical, hashes)
        id3 = report["rows_by_id"]["3"]
        self.assertEqual(current, before)
        self.assertAlmostEqual(id3["raw_current_unmodified_rad"],
                               screen.TWO_PI + math.radians(2))
        self.assertAlmostEqual(id3["raw_comparison_copy_rad"], math.radians(2))
        self.assertEqual(report["direct_out_of_range_ids"], [3])
        self.assertEqual(report["conditional_out_of_range_ids"], [])
        self.assertFalse(report["cross_boot_angle_continuity_verified"])
        self.assertFalse(report["motor_supply_off_on_evidence_complete"])
        self.assertFalse(report["approved_for_runtime"])
        self.assertFalse(report["motor_targets_generated"])

    def test_requires_explicit_claim_and_rejects_unrelated_branch(self):
        camera, current, historical, hashes = fixture()
        with self.assertRaisesRegex(ValueError, "Explicit operator"):
            self.review(camera, current, historical, hashes, claim=False)
        current["telemetry"]["rows"]["3"]["median_position_rad"] += math.radians(20)
        with self.assertRaisesRegex(ValueError, r"reviewed \+one-turn"):
            self.review(camera, current, historical, hashes)

    def test_rejects_uid_or_quiet_state_mismatch(self):
        camera, current, historical, hashes = fixture()
        current["identities"]["3"]["mcu_uid_hex"] = "ffffffffffffffff"
        with self.assertRaisesRegex(ValueError, "UIDs differ"):
            self.review(camera, current, historical, hashes)
        current["identities"]["3"]["mcu_uid_hex"] = "0000000000000003"
        current["telemetry"]["rows"]["10"]["current"] = 0.2
        with self.assertRaisesRegex(ValueError, "quiet and stationary"):
            self.review(camera, current, historical, hashes)


if __name__ == "__main__":
    unittest.main()
