"""The standing-route review must expose old-boot and collision blockers."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw.staged_stance_review import review_historical_stand


ROOT = Path(__file__).resolve().parents[2]


class StagedStanceReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.current = json.loads((ROOT / "evidence/stance-rest-after-manual-20260926-r1.json").read_text())
        cls.historical = json.loads((ROOT / "evidence/stand10-geometry-20260926.json").read_text())

    def test_actual_captures_block_old_target_and_known_contact(self):
        review = review_historical_stand(self.current, self.historical)
        self.assertEqual(review["status"], "BLOCKED_FOR_ACTUATION_REVIEW_ONLY")
        self.assertFalse(review["output_allowed"])
        self.assertFalse(review["same_boot"])
        self.assertTrue(review["start_angle_rebase_can_invalidate_clearance"])
        self.assertEqual([stage["motor_ids"] for stage in review["stages"]],
                         [[3, 6], [9, 12], [2, 5], [8, 11], [1, 4, 7, 10]])
        front_thighs = review["stages"][2]
        self.assertEqual(front_thighs["motor_ids"], [2, 5])
        self.assertIn("known_carbon_clamp_contact_in_direct_route", front_thighs["blockers"])
        self.assertIn("hip_first_full_swept_clearance_unverified", front_thighs["blockers"])
        self.assertIn("target_is_from_another_boot", review["stages"][0]["blockers"])
        self.assertGreaterEqual(review["stages"][0]["review_intervals_at_most_10deg"], 1)
        self.assertTrue(review["raw_turn_ambiguous_motor_ids"])
        json.dumps(review, allow_nan=False)

    def test_same_boot_does_not_claim_clearance_or_authorize_output(self):
        current = copy.deepcopy(self.current)
        current["boot_id"] = self.historical["boot_id"]
        review = review_historical_stand(current, self.historical)
        self.assertTrue(review["same_boot"])
        self.assertFalse(review["output_allowed"])
        self.assertFalse(review["collision_clearance_verified"])
        self.assertTrue(review["start_angle_rebase_can_invalidate_clearance"])
        self.assertIn("known_carbon_clamp_contact_in_direct_route",
                      review["stages"][2]["blockers"])

    def test_no_auto_wrap_on_turn_disagreement(self):
        review = review_historical_stand(self.current, self.historical)
        ambiguous = set(review["raw_turn_ambiguous_motor_ids"])
        self.assertIn(8, ambiguous)
        self.assertIn("raw_turn_ambiguity_no_automatic_wrapping", review["stages"][3]["blockers"])
        self.assertGreater(abs(review["stages"][3]["historical_delta_deg_by_id"]["8"]), 180)

    def test_rejects_incomplete_or_non_read_only_current_capture(self):
        for mutation in (
            lambda x: x.update(read_only=False),
            lambda x: x.update(motor_enable_sent=True),
            lambda x: x["motors"].pop("12"),
            lambda x: x["motors"]["2"].update(uid_match=False),
            lambda x: x["motors"]["2"].update(position_last_rad=float("nan")),
            lambda x: x["motors"]["2"].update(position_last_rad=100),
        ):
            current = copy.deepcopy(self.current)
            mutation(current)
            with self.subTest(current=current.get("read_only")), self.assertRaises(ValueError):
                review_historical_stand(current, self.historical)

    def test_rejects_inconsistent_or_duplicate_historical_rows(self):
        for mutation in (
            lambda x: x.update(status="LIVE_TARGET"),
            lambda x: x["joint_rows"].pop(),
            lambda x: x["joint_rows"][0].update(id=2),
            lambda x: x["joint_rows"][0].update(raw_step_deg=11),
            lambda x: x["joint_rows"][0].update(remaining_to_nominal_raw_deg=float("inf")),
        ):
            historical = copy.deepcopy(self.historical)
            mutation(historical)
            with self.subTest(status=historical.get("status")), self.assertRaises(ValueError):
                review_historical_stand(self.current, historical)

    def test_review_interval_is_only_bounded_analysis_input(self):
        for bad in (0, -1, 10.01, True, "10", float("nan")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                review_historical_stand(self.current, self.historical, review_step_deg=bad)

    def test_cli_writes_review_without_command_fields(self):
        from importlib.util import module_from_spec, spec_from_file_location
        spec = spec_from_file_location("review_standing_route", ROOT / "tools/review_standing_route.py")
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "review.json"
            code = module.main(["--current", str(ROOT / "evidence/stance-rest-after-manual-20260926-r1.json"),
                                "--historical", str(ROOT / "evidence/stand10-geometry-20260926.json"),
                                "--output", str(result)])
            self.assertEqual(code, 0)
            document = json.loads(result.read_text())
            self.assertFalse(document["output_allowed"])
            self.assertNotIn("command_frames", document)
            self.assertNotIn("trajectory_samples", document)


if __name__ == "__main__":
    unittest.main()
