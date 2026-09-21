"""A verified replacement identity permits new observations, never auto-adoption."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import manual_calibration as manual
from test_manual_calibration import synthetic_pose, complete_records


RETIRED_UID = "000000000000000b"
REPLACEMENT_UID = "abcde0123456789f"
OTHER_NEW_UID = "fedcb9876543210a"
POLICY = {"schema_version": 1, "motor_id": 11,
          "retired_uid": RETIRED_UID, "replacement_uid": REPLACEMENT_UID,
          "eligibility": "candidate_only"}


def replacement_records(stages, uid=REPLACEMENT_UID):
    records = complete_records(stages)
    for record in records:
        record["motors"]["11"]["mcu_uid_hex"] = uid
    return records


def id11_candidate(stages, records, policy=POLICY):
    return next(candidate for candidate in manual.build_candidates(stages, records, replacement=policy)
                if candidate["motor_id"] == 11)


class ReplacementCandidateTests(unittest.TestCase):
    def setUp(self):
        self.stages = manual.make_stages(["RL"])

    def assert_no_candidate(self, records, policy=POLICY):
        candidate = id11_candidate(self.stages, records, policy)
        self.assertIsNone(candidate["sign_candidate"])
        self.assertIsNone(candidate["offset_candidate_rad"])
        self.assertIs(candidate["approved_for_runtime"], False)
        self.assertIs(candidate["zero_verified"], False)
        self.assertIs(candidate["sign_verified"], False)
        return candidate

    def test_consistent_replacement_baseline_direction_and_return_allow_unverified_candidate(self):
        records = replacement_records(self.stages)
        candidate = id11_candidate(self.stages, records)
        self.assertEqual(candidate["status"], "MANUAL_NOMINAL_CANDIDATE_REVIEW_REQUIRED")
        self.assertEqual(candidate["sign_candidate"], 1)
        self.assertAlmostEqual(candidate["offset_candidate_rad"], -1.1)
        self.assertAlmostEqual(candidate["observed_delta_deg"], -8)
        self.assertEqual(candidate["nominal_delta_deg"], -10)
        self.assertEqual(candidate["evidence_paths"], {
            "baseline": "/synthetic/observations/rl-l",
            "direction": "/synthetic/observations/rl-thigh",
            "return": "/synthetic/observations/rl-return",
        })
        for flag in ("approved_for_runtime", "zero_verified", "sign_verified", "measured_model_angles"):
            self.assertIs(candidate[flag], False)

    def test_missing_policy_still_withholds_replacement_candidate(self):
        candidate = self.assert_no_candidate(replacement_records(self.stages), policy=None)
        self.assertEqual(candidate["status"], "KNOWN_POSITION_JUMP_REQUIRES_REVIEW")

    def test_retired_and_unregistered_new_identity_cannot_supply_candidate(self):
        for uid in (RETIRED_UID, OTHER_NEW_UID):
            with self.subTest(uid=uid):
                candidate = self.assert_no_candidate(replacement_records(self.stages, uid))
                self.assertEqual(candidate["status"], "IDENTITY_NOT_ELIGIBLE_FOR_CANDIDATE")

    def test_any_mixed_identity_in_required_evidence_blocks_candidate(self):
        for index in (0, 2, 4):
            for uid in (RETIRED_UID, OTHER_NEW_UID):
                with self.subTest(stage=self.stages[index]["id"], uid=uid):
                    records = replacement_records(self.stages)
                    records[index]["motors"]["11"]["mcu_uid_hex"] = uid
                    candidate = self.assert_no_candidate(records)
                    self.assertEqual(candidate["status"], "IDENTITY_NOT_ELIGIBLE_FOR_CANDIDATE")

    def test_replacement_policy_does_not_remove_return_requirement(self):
        records = replacement_records(self.stages)
        self.assert_no_candidate(records[:-1])
        records[-1]["accepted"] = False
        self.assert_no_candidate(records)

    def test_replacement_policy_does_not_override_current_or_position_stability_checks(self):
        for index in (0, 2, 4):
            for failure in ("current", "position"):
                with self.subTest(stage=self.stages[index]["id"], failure=failure):
                    records = replacement_records(self.stages)
                    motor = records[index]["motors"]["11"]
                    if failure == "current":
                        motor["max_abs_current_A"] = .051
                    else:
                        motor["position"]["peak_to_peak_rad"] = .03
                    pose = {"motors": records[index]["motors"]}
                    baseline = {"motors": records[0]["motors"]} if index else None
                    quality = manual.analyse(self.stages[index], pose, baseline, replacement=POLICY)
                    self.assertEqual(quality["status"], "RETRY_RECOMMENDED")
                    self.assertTrue(quality["blockers"])
                    records[index].update(quality=quality, accepted=not quality["blockers"])
                    self.assert_no_candidate(records)

    def test_replacement_with_bad_return_or_new_value_jump_still_has_no_candidate(self):
        for index, extra_degrees in ((2, 46), (4, 5)):
            with self.subTest(stage=self.stages[index]["id"]):
                records = replacement_records(self.stages)
                motor = records[index]["motors"]["11"]
                motor["median_deg_raw_shaft"] += extra_degrees
                motor["position"]["median_rad"] += math.radians(extra_degrees)
                quality = manual.analyse(self.stages[index], {"motors": records[index]["motors"]},
                                         {"motors": records[0]["motors"]}, replacement=POLICY)
                self.assertTrue(quality["blockers"])
                records[index].update(quality=quality, accepted=False)
                self.assert_no_candidate(records)


class ReplacementSessionTests(unittest.TestCase):
    def test_new_session_uses_only_fresh_replacement_observations_for_id11(self):
        stages = manual.make_stages(["RL"])
        poses = [{"motors": record["motors"], "completed_at": "2026-09-21T10:00:00+09:00"}
                 for record in replacement_records(stages)]
        with tempfile.TemporaryDirectory() as tmp:
            session = manual.Session(Path(tmp) / "session", stages,
                                     get_boot=lambda: "replacement-boot", replacement=copy.deepcopy(POLICY))
            with patch.object(manual.joint_snapshot, "main", return_value=0) as snapshot, \
                    patch.object(manual, "save_record", side_effect=poses) as save_pose:
                for stage in stages:
                    self.assertTrue(session.capture(stage)["accepted"])
            self.assertEqual(snapshot.call_count, 5)
            self.assertEqual(save_pose.call_count, 5)
            data = json.loads((session.path / "session.json").read_text())
            self.assertEqual(data["replacement_evidence"], POLICY)
            self.assertEqual(data["identities"]["11"], REPLACEMENT_UID)
            self.assertTrue(all(record["motors"]["11"]["mcu_uid_hex"] == REPLACEMENT_UID
                                for record in data["records"]))
            candidate = next(item for item in data["candidates"] if item["motor_id"] == 11)
            self.assertEqual(candidate["status"], "MANUAL_NOMINAL_CANDIDATE_REVIEW_REQUIRED")
            self.assertTrue(all(Path(path).is_relative_to(session.path)
                                for path in candidate["evidence_paths"].values()))
            for flag in ("approved_for_runtime", "zero_verified", "sign_verified"):
                self.assertIs(data[flag], False)
                self.assertIs(candidate[flag], False)

    def test_wrong_id11_identity_fails_capture_before_accepting_any_baseline(self):
        for uid in (RETIRED_UID, OTHER_NEW_UID):
            with self.subTest(uid=uid):
                pose = synthetic_pose()
                pose["motors"]["11"]["mcu_uid_hex"] = uid
                self.assert_capture_identity_failure(pose)

    def test_retired_motor_at_another_can_id_also_fails_capture(self):
        pose = synthetic_pose()
        pose["motors"]["11"]["mcu_uid_hex"] = REPLACEMENT_UID
        pose["motors"]["1"]["mcu_uid_hex"] = RETIRED_UID
        self.assert_capture_identity_failure(pose)

    def assert_capture_identity_failure(self, pose):
        # The identity policy applies across all twelve motors, even during an FR pose.
        stages = manual.make_stages(["FR"])
        with tempfile.TemporaryDirectory() as tmp:
            session = manual.Session(Path(tmp) / "session", stages,
                                     get_boot=lambda: "replacement-boot", replacement=copy.deepcopy(POLICY))
            with patch.object(manual.joint_snapshot, "main", return_value=0) as snapshot, \
                    patch.object(manual, "save_record", return_value=pose):
                with self.assertRaisesRegex(RuntimeError, "交換記録"):
                    session.capture(stages[0])
            snapshot.assert_called_once()
            data = json.loads((session.path / "session.json").read_text())
            self.assertEqual(len(data["records"]), 1)
            self.assertEqual(data["records"][0]["status"], "FAILED")
            self.assertFalse(data["records"][0]["accepted"])
            self.assertIn("交換記録", data["records"][0]["error"])
            self.assertIsNone(data["identities"])
            self.assertFalse(session.baselines)
            self.assertTrue(all(item["sign_candidate"] is None for item in data["candidates"]))


if __name__ == "__main__":
    unittest.main()
