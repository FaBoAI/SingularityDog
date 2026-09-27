"""Synthetic no-hardware checks for the policy input candidate generator."""

import copy
import math
import unittest

from tools import offline_camera_l_current_boot_range as range_screen
from tools import offline_camera_l_policy_candidate as candidate
from tools.test_offline_camera_l_current_boot_range import fixture
from singularitydog_hw import policy_shadow


def inputs():
    camera, current, historical, hashes = fixture()
    hashes["current"] = "c" * 64
    review = range_screen.build_review(
        camera, current, historical, operator_no_turn_id3=True,
        operator_statement="Operator confirms no full ID3 physical turn.",
        source_hashes=hashes)
    return review, current


class OfflinePolicyCandidateTests(unittest.TestCase):
    def build(self, review, current):
        return candidate.build_candidate(
            review, current, review_sha256="a" * 64,
            current_sha256="c" * 64)

    def test_exact_offset_reproduces_comparison_and_policy_schema(self):
        review, current = inputs()
        before = copy.deepcopy(current)
        result = self.build(review, current)
        rows = policy_shadow.validate_calibration(result)
        self.assertEqual(len(rows), 12)
        self.assertEqual(current, before)
        id3 = rows[3]
        self.assertEqual(id3["sign_candidate"], -1)
        self.assertAlmostEqual(id3["offset_candidate_rad"],
                               range_screen.TWO_PI)
        raw = current["telemetry"]["rows"]["3"]["median_position_rad"]
        self.assertAlmostEqual(id3["sign_candidate"] * raw
                               + id3["offset_candidate_rad"],
                               -math.radians(2))
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(result["motor_targets_generated"])
        self.assertFalse(result["motor_supply_off_on_evidence_complete"])

    def test_rejects_review_without_range_pass_or_matching_source(self):
        review, current = inputs()
        review["model_range_screen_diagnostic_only_passed"] = False
        with self.assertRaisesRegex(ValueError, "unapproved conditional range review"):
            self.build(review, current)
        review, current = inputs()
        review["source_sha256"]["current"] = "wrong"
        with self.assertRaisesRegex(ValueError, "source hashes differ"):
            self.build(review, current)

    def test_rejects_raw_or_boot_mismatch(self):
        review, current = inputs()
        current["telemetry"]["rows"]["3"]["median_position_rad"] += 0.1
        with self.assertRaisesRegex(ValueError, "raw angle differs"):
            self.build(review, current)
        review, current = inputs()
        current["boot_id"] = "another-boot"
        with self.assertRaisesRegex(ValueError, "does not match the review"):
            self.build(review, current)


if __name__ == "__main__":
    unittest.main()
