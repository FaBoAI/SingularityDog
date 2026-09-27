"""No-hardware checks for power-epoch-bound static angle comparison."""

import copy
import math
import unittest

from singularitydog_hw.angle_branch_comparison import (
    BranchComparisonError, StaticBranchComparison, TWO_PI,
)


IDS = tuple(str(i) for i in range(1, 13))
UIDS = {mid: f"{int(mid):016x}" for mid in IDS}


def snapshot(epoch, hash_digit, raw=None, *, boot="jetson-boot"):
    return {"boot_id": boot, "motor_power_epoch": epoch,
            "uid_read_boot_id": boot, "uid_read_motor_power_epoch": epoch,
            "capture_sha256": hash_digit * 64,
            "uid_capture_sha256": hash_digit * 63 + "f",
            "uids_by_id": dict(UIDS),
            "raw_rad_by_id": dict(raw or {mid: .1 * int(mid) for mid in IDS}),
            "disabled_zero_current_by_id": {mid: True for mid in IDS},
            "motor_output_allowed": False}


def evidence(reference, current):
    return {"motor_supply_off_on_observed": True,
            "reference_capture_sha256": reference["capture_sha256"],
            "current_capture_sha256": current["capture_sha256"],
            "evidence_sha256": "e" * 64,
            "physical_pose_observation_by_id": {
                "3": "ID3 output mark observed without a full turn during Off/On"},
            "no_full_physical_turn_by_id": {"3": True}}


class BranchComparisonTests(unittest.TestCase):
    def setUp(self):
        self.reference = snapshot("motor-epoch-1", "a")
        raw = dict(self.reference["raw_rad_by_id"])
        raw["3"] += TWO_PI + math.radians(2)
        raw["9"] -= TWO_PI + math.radians(1)
        self.current = snapshot("motor-epoch-2", "b", raw)
        self.no_turn = evidence(self.reference, self.current)

    def test_plus_one_turn_is_comparison_only_and_other_axes_keep_raw(self):
        original = copy.deepcopy(self.current)
        review = StaticBranchComparison(self.reference, self.current, self.no_turn)
        result = review.comparison()
        self.assertEqual(self.current, original)
        self.assertEqual(result["rows"]["3"]["branch_turns_for_comparison"], 1)
        self.assertTrue(result["rows"]["3"]["branch_reviewed"])
        self.assertFalse(result["rows"]["9"]["branch_reviewed"])
        self.assertEqual(result["rows"]["9"]["branch_turns_for_comparison"], 0)
        self.assertEqual(result["rows"]["9"]["comparison_raw_rad"],
                         original["raw_rad_by_id"]["9"])
        self.assertAlmostEqual(result["rows"]["3"]["comparison_delta_rad"], math.radians(2))
        self.assertAlmostEqual(result["rows"]["3"]["direct_delta_rad"],
                               TWO_PI + math.radians(2))
        self.assertFalse(result["raw_targets_changed"])
        self.assertFalse(result["motor_output_allowed"])
        self.assertFalse(result["approved_for_runtime"])

    def test_minus_one_turn_with_small_real_pose_delta(self):
        current = copy.deepcopy(self.current)
        current["raw_rad_by_id"]["3"] = (self.reference["raw_rad_by_id"]["3"]
                                           - TWO_PI - math.radians(1))
        review = StaticBranchComparison(self.reference, current,
                                        evidence(self.reference, current))
        row = review.comparison()["rows"]["3"]
        self.assertEqual(row["branch_turns_for_comparison"], -1)
        self.assertAlmostEqual(row["comparison_delta_rad"], -math.radians(1))

    def test_reported_id3_raw_pair_uses_exact_arithmetic(self):
        reference, current = copy.deepcopy(self.reference), copy.deepcopy(self.current)
        reference["raw_rad_by_id"]["3"] = -.043743
        current["raw_rad_by_id"]["3"] = 6.262798
        review = StaticBranchComparison(reference, current, evidence(reference, current))
        row = review.comparison()["rows"]["3"]
        self.assertAlmostEqual(math.degrees(row["comparison_delta_rad"]),
                               1.338182626213717, places=10)
        self.assertEqual(row["branch_turns_for_comparison"], 1)

    def test_post_charge_box_capture_id3_residual_is_1_307_degrees(self):
        reference, current = copy.deepcopy(self.reference), copy.deepcopy(self.current)
        reference["raw_rad_by_id"]["3"] = -0.04319906234741211
        current["raw_rad_by_id"]["3"] = 6.262798309326172
        row = StaticBranchComparison(reference, current,
                                     evidence(reference, current)).comparison()["rows"]["3"]
        self.assertAlmostEqual(math.degrees(row["direct_delta_rad"]),
                               361.3070350174863, places=10)
        self.assertAlmostEqual(math.degrees(row["comparison_delta_rad"]),
                               1.3070350174863, places=10)

    def test_epoch_offset_is_algebraic_observation_candidate_only(self):
        review = StaticBranchComparison(self.reference, self.current, self.no_turn)
        signs = {mid: 1 for mid in IDS}
        signs["3"] = -1
        offsets = {mid: 0.0 for mid in IDS}
        offsets["3"] = .3
        candidate = review.comparison_offset_candidates(signs, offsets)
        self.assertEqual(set(candidate["candidate_by_id"]), {"3"})
        self.assertAlmostEqual(candidate["candidate_by_id"]["3"]
                               ["comparison_offset_candidate_rad"], .3 + TWO_PI)
        self.assertFalse(candidate["motor_output_allowed"])
        self.assertFalse(candidate["approved_for_runtime"])

    def test_validated_binding_is_copied_and_never_approves_output(self):
        review = StaticBranchComparison(self.reference, self.current, self.no_turn)
        binding = review.validated_current_binding()
        self.assertEqual(binding["reviewed_branch_turns_by_id"], {"3": 1})
        self.assertEqual(binding["motor_power_epoch"], "motor-epoch-2")
        self.assertEqual(binding["uid_capture_sha256"], self.current["uid_capture_sha256"])
        self.assertFalse(binding["motor_output_allowed"])
        self.assertFalse(binding["approved_for_runtime"])
        binding["raw_rad_by_id"]["3"] = 42.
        binding["uids_by_id"]["3"] = "ffffffffffffffff"
        self.assertEqual(review.validated_current_binding()["raw_rad_by_id"]["3"],
                         self.current["raw_rad_by_id"]["3"])
        self.assertEqual(review.validated_current_binding()["uids_by_id"]["3"],
                         self.current["uids_by_id"]["3"])

    def test_multi_turn_and_ambiguous_pose_are_rejected(self):
        for delta in (2 * TWO_PI + .01, math.pi, TWO_PI + math.radians(11)):
            with self.subTest(delta=delta):
                current = copy.deepcopy(self.current)
                current["raw_rad_by_id"]["3"] = self.reference["raw_rad_by_id"]["3"] + delta
                with self.assertRaises(BranchComparisonError):
                    StaticBranchComparison(self.reference, current,
                                           evidence(self.reference, current))

    def test_missing_physical_or_power_epoch_evidence_rejected(self):
        for key in ("motor_supply_off_on_observed", "no_full_physical_turn_by_id"):
            with self.subTest(key=key):
                proof = copy.deepcopy(self.no_turn)
                if key == "motor_supply_off_on_observed":
                    proof[key] = False
                else:
                    proof[key]["3"] = False
                with self.assertRaises(BranchComparisonError):
                    StaticBranchComparison(self.reference, self.current, proof)
        same_epoch = copy.deepcopy(self.current)
        same_epoch["motor_power_epoch"] = self.reference["motor_power_epoch"]
        same_epoch["uid_read_motor_power_epoch"] = same_epoch["motor_power_epoch"]
        with self.assertRaisesRegex(BranchComparisonError, "distinct observed motor power epoch"):
            StaticBranchComparison(self.reference, same_epoch,
                                   evidence(self.reference, same_epoch))

    def test_unproven_axis_cannot_be_requested_for_branch_review(self):
        proof = copy.deepcopy(self.no_turn)
        proof["no_full_physical_turn_by_id"]["9"] = False
        with self.assertRaisesRegex(BranchComparisonError, "requested shaft"):
            StaticBranchComparison(self.reference, self.current, proof)
        proof["no_full_physical_turn_by_id"]["9"] = True
        with self.assertRaisesRegex(BranchComparisonError, "physical pose observation"):
            StaticBranchComparison(self.reference, self.current, proof)

    def test_fresh_uid_and_boot_binding_rejected_if_wrong(self):
        wrong_uid = copy.deepcopy(self.current)
        wrong_uid["uids_by_id"]["7"] = "ffffffffffffffff"
        with self.assertRaisesRegex(BranchComparisonError, "UID or ID assignment changed"):
            StaticBranchComparison(self.reference, wrong_uid, self.no_turn)
        stale_uid = copy.deepcopy(self.current)
        stale_uid["uid_read_motor_power_epoch"] = self.reference["motor_power_epoch"]
        with self.assertRaisesRegex(BranchComparisonError, "freshly read"):
            StaticBranchComparison(self.reference, stale_uid, self.no_turn)
        wrong_boot = copy.deepcopy(self.current)
        wrong_boot["uid_read_boot_id"] = "other-boot"
        with self.assertRaisesRegex(BranchComparisonError, "freshly read"):
            StaticBranchComparison(self.reference, wrong_boot, self.no_turn)

    def test_within_epoch_jump_aborts_and_cannot_be_recovered(self):
        review = StaticBranchComparison(self.reference, self.current, self.no_turn)
        later = snapshot("motor-epoch-2", "c", self.current["raw_rad_by_id"])
        later["raw_rad_by_id"]["3"] += TWO_PI
        with self.assertRaisesRegex(BranchComparisonError, "within-epoch raw angle discontinuity"):
            review.observe_same_epoch(later)
        with self.assertRaisesRegex(BranchComparisonError, "aborted"):
            review.comparison()
        with self.assertRaisesRegex(BranchComparisonError, "aborted"):
            review.observe_same_epoch(self.current)

    def test_small_same_epoch_static_read_uses_fixed_branch(self):
        review = StaticBranchComparison(self.reference, self.current, self.no_turn)
        later = snapshot("motor-epoch-2", "c", self.current["raw_rad_by_id"])
        later["raw_rad_by_id"]["3"] += math.radians(1)
        result = review.observe_same_epoch(later)
        self.assertEqual(result["rows"]["3"]["branch_turns_for_comparison"], 1)
        self.assertAlmostEqual(result["rows"]["3"]["comparison_delta_rad"], math.radians(3))


if __name__ == "__main__":
    unittest.main()
