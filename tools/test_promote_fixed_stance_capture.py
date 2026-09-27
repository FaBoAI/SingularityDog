"""File-only provenance and physical-review tests; no CAN device is opened."""

import json
from pathlib import Path
import statistics
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import promote_fixed_stance_capture as promotion
from singularitydog_hw.fixed_stance_readonly_capture import draft_manifest


UIDS = {str(mid): f"{mid:016x}" for mid in range(1, 13)}
BOOT = "current-test-boot"
ENTER = 1_000_000_000
START = ENTER + 100
END = START + 10_000


def put(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + "\n")


def example_summary(uid_sha):
    samples = {}
    raw = {}
    for mid in range(1, 13):
        rows = []
        values = [mid / 10 + index * .001 for index in range(3)]
        for index, position in enumerate(values):
            row = {}
            for parameter_index, parameter in enumerate(promotion.PARAMETERS):
                request = START + index * 1000 + parameter_index * 100
                value = {"position": position, "velocity": .01,
                         "current": 0., "voltage": 40., "run_mode": 0.}[parameter]
                row[parameter] = {"value": value, "request_monotonic_ns": request,
                                  "reply_monotonic_ns": request + 1}
            rows.append(row)
        samples[str(mid)] = rows
        raw[str(mid)] = statistics.median(values)
    identities = {str(mid): {"mcu_uid_hex": UIDS[str(mid)],
                             "request_monotonic_ns": ENTER - 20,
                             "reply_monotonic_ns": ENTER - 10}
                  for mid in range(1, 13)}
    return {
        "schema": promotion.READONLY_SCHEMA,
        "status": "RECORDED_REVIEW_REQUIRED", "boot_id": BOOT,
        "errors": [], "output_allowed": False, "approved_for_runtime": False,
        "stop_state": promotion.UNVERIFIED_STOP,
        "expected_uids_sha256": uid_sha,
        "source_sha256": {name: promotion.digest((promotion.ROOT / "runtime" /
                            "singularitydog_hw" / name).read_bytes()) for name in
                          ("fixed_stance_readonly_capture.py", "can_readonly.py")},
        "plan": {"ids_by_bus": {"front": list(range(1, 7)),
                                 "rear": list(range(7, 13))},
                 "allowed_can_types": [0, 17],
                 "parameters": ["identity", *promotion.PARAMETERS],
                 "motor_output_available": False, "output_allowed": False,
                 "stop_command_available": False, "stop_state_verifiable": False,
                 "sweeps": 3},
        "operator_enter_monotonic_ns": ENTER,
        "identities": identities,
        "pose": {"samples": samples, "raw_rad_by_id": raw,
                 "started_monotonic_ns": START, "ended_monotonic_ns": END,
                 "capture_span_ms": (END - START) / 1e6,
                 "sampling_issues": [], "sampling_stability_heuristic_passed": True,
                 "stationarity_verified": False},
    }


class PromotionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.expected_path = self.root / "expected-uids.json"
        self.summary_path = self.root / "summary.json"
        self.draft_path = self.root / "capture-draft.json"
        self.review_path = self.root / "physical-review.json"
        self.output = self.root / "capture.json"
        put(self.expected_path, UIDS)
        self.summary = example_summary(promotion.digest(self.expected_path.read_bytes()))
        self.refresh()

    def refresh(self):
        put(self.summary_path, self.summary)
        summary_sha = promotion.digest(self.summary_path.read_bytes())
        self.draft = draft_manifest(self.summary, summary_sha)
        put(self.draft_path, self.draft)

    def physical_review(self):
        return {
            "schema": promotion.PHYSICAL_REVIEW_SCHEMA,
            "boot_id": BOOT,
            "motor_uids": UIDS,
            "raw_rad_by_id": self.summary["pose"]["raw_rad_by_id"],
            "readonly_summary_sha256": promotion.digest(self.summary_path.read_bytes()),
            "readonly_draft_sha256": promotion.digest(self.draft_path.read_bytes()),
            "supported_pose_placed_by_operator": True,
            "simultaneous_physical_stance_verified": True,
            "stand_removed": False,
            "pose_class": promotion.FLOOR_STANCE_CLASS,
            "foot_support_kind": "floor",
            "foot_support_height_cm": 0,
            "operator_note": "All four legs were held in one supported pose throughout capture.",
            "evidence_reference": "private/front-and-side-pose-video.mp4",
        }

    def promote(self, review=None):
        return promotion.promote(self.summary_path, self.draft_path,
                                 self.expected_path, self.output,
                                 physical_review_path=review)

    def test_valid_capture_stays_blocked_without_separate_physical_review(self):
        template_path = self.root / "physical-review-template.json"
        status = promotion.promote(self.summary_path, self.draft_path,
                                   self.expected_path, self.output,
                                   review_template_path=template_path)
        output = json.loads(self.output.read_text())
        template = json.loads(template_path.read_text())
        self.assertEqual(status["status"], "READONLY_CAPTURE_VALIDATED_REVIEW_REQUIRED")
        self.assertEqual(status["review_template"], str(template_path.resolve()))
        self.assertFalse(template["supported_pose_placed_by_operator"])
        self.assertFalse(template["simultaneous_physical_stance_verified"])
        self.assertIsNone(template["pose_class"])
        self.assertIsNone(template["foot_support_kind"])
        self.assertIsNone(template["foot_support_height_cm"])
        self.assertEqual(template["raw_rad_by_id"], output["raw_rad_by_id"])
        self.assertEqual(output["schema"], promotion.CAPTURE_SCHEMA)
        self.assertEqual(output["raw_rad_by_id"], self.summary["pose"]["raw_rad_by_id"])
        self.assertEqual(output["motor_uids"], UIDS)
        self.assertFalse(output["supported_pose_placed_by_operator"])
        self.assertFalse(output["simultaneous_physical_stance_verified"])
        self.assertIsNone(output["pose_class"])
        self.assertIsNone(output["foot_support_kind"])
        self.assertFalse(output["output_allowed"])
        self.assertFalse(output["approved_for_runtime"])
        self.assertFalse(output["same_boot_12_axis_hold_passed"])
        self.assertFalse(output["all_segment_sweeps_physically_reviewed"])
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o600)
        self.assertEqual(output["readonly_summary_sha256"],
                         promotion.digest(self.summary_path.read_bytes()))

    def test_exact_separate_review_only_confirms_supported_pose(self):
        put(self.review_path, self.physical_review())
        status = self.promote(self.review_path)
        output = json.loads(self.output.read_text())
        self.assertEqual(status["status"], "PHYSICAL_POSE_REVIEWED_DISABLED_ONLY")
        self.assertTrue(output["supported_pose_placed_by_operator"])
        self.assertTrue(output["simultaneous_physical_stance_verified"])
        self.assertEqual(output["pose_class"], promotion.FLOOR_STANCE_CLASS)
        self.assertEqual(output["foot_support_kind"], "floor")
        self.assertEqual(output["foot_support_height_cm"], 0)
        self.assertFalse(output["all_segment_sweeps_physically_reviewed"])
        self.assertFalse(output["front_upper_leg_carbon_clamp_clearance_verified"])
        self.assertFalse(output["output_allowed"])
        self.assertFalse(output["self_supported_standing_verified"])
        self.assertEqual(output["physical_pose_review_sha256"],
                         promotion.digest(self.review_path.read_bytes()))

    def test_tampered_draft_or_expected_identity_rejected_without_output(self):
        self.draft["raw_rad_by_id"]["7"] += .01
        put(self.draft_path, self.draft)
        with self.assertRaisesRegex(ValueError, "draft differs"):
            self.promote()
        self.assertFalse(self.output.exists())
        self.refresh()
        changed = dict(UIDS)
        changed["7"] = "f" * 16
        put(self.expected_path, changed)
        with self.assertRaisesRegex(ValueError, "Expected UID file differs"):
            self.promote()
        self.assertFalse(self.output.exists())

    def test_uncorroborated_sample_stability_and_median_rejected(self):
        self.summary["pose"]["samples"]["8"][2]["position"]["value"] += .03
        self.refresh()
        with self.assertRaisesRegex(ValueError, "ID8 sampled pose is unstable"):
            self.promote()
        self.assertFalse(self.output.exists())
        self.summary = example_summary(promotion.digest(self.expected_path.read_bytes()))
        self.summary["pose"]["raw_rad_by_id"]["8"] += .005
        self.refresh()
        with self.assertRaisesRegex(ValueError, "ID8 raw position differs"):
            self.promote()
        self.assertFalse(self.output.exists())

    def test_velocity_heuristic_and_summary_sha_link_are_enforced(self):
        self.summary["pose"]["samples"]["6"][1]["velocity"]["value"] = .101
        self.refresh()
        with self.assertRaisesRegex(ValueError, "ID6 sampled pose is unstable"):
            self.promote()
        self.assertFalse(self.output.exists())
        self.summary = example_summary(promotion.digest(self.expected_path.read_bytes()))
        self.refresh()
        self.draft["stance_capture_sha256"] = "f" * 64
        put(self.draft_path, self.draft)
        with self.assertRaisesRegex(ValueError, "draft differs"):
            self.promote()
        self.assertFalse(self.output.exists())

    def test_stale_or_incomplete_sample_rejected(self):
        sample = self.summary["pose"]["samples"]["9"][0]["position"]
        sample["request_monotonic_ns"] = ENTER - 1
        self.refresh()
        with self.assertRaisesRegex(ValueError, "stale or out of sequence"):
            self.promote()
        self.assertFalse(self.output.exists())
        self.summary = example_summary(promotion.digest(self.expected_path.read_bytes()))
        self.summary["pose"]["samples"]["9"].pop()
        self.refresh()
        with self.assertRaisesRegex(ValueError, "incomplete sample group"):
            self.promote()
        self.assertFalse(self.output.exists())

    def test_physical_review_must_pin_exact_capture_and_cannot_authorize_output(self):
        review = self.physical_review()
        review["readonly_summary_sha256"] = "f" * 64
        put(self.review_path, review)
        with self.assertRaisesRegex(ValueError, "Separate physical pose review"):
            self.promote(self.review_path)
        self.assertFalse(self.output.exists())
        review = self.physical_review()
        review["output_allowed"] = True
        put(self.review_path, review)
        with self.assertRaisesRegex(ValueError, "Separate physical pose review"):
            self.promote(self.review_path)
        self.assertFalse(self.output.exists())

    def test_l_calibration_or_raised_foot_platform_cannot_be_promoted(self):
        for field, value in (("pose_class", "calibration_L"),
                             ("foot_support_kind", "platform"),
                             ("foot_support_height_cm", 12)):
            with self.subTest(field=field):
                review = self.physical_review()
                review[field] = value
                put(self.review_path, review)
                with self.assertRaisesRegex(ValueError, "Separate physical pose review"):
                    self.promote(self.review_path)
                self.assertFalse(self.output.exists())

    def test_readonly_source_hash_mismatch_rejected(self):
        self.summary["source_sha256"]["fixed_stance_readonly_capture.py"] = "f" * 64
        self.refresh()
        with self.assertRaisesRegex(ValueError, "source differs"):
            self.promote()
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
