"""File-only tests for a supported, current-boot raw standing candidate."""

import math
import unittest

from singularitydog_hw.fixed_stance_candidate import (
    HOLD_TICKS, IDS, MAX_SEGMENT_DEG, RAMP_TICKS, REQUIRED_REVIEWS,
    SCHEMA, prepare_candidate,
)


BOOT = "current-boot"
UIDS = {str(i): f"uid-{i}" for i in IDS}
START = {str(i): .25 * i for i in IDS}


def at_offset(degrees):
    return {str(i): START[str(i)] + math.radians(degrees) for i in IDS}


def reviewed(*, degrees=5.):
    target = at_offset(degrees)
    return {
        "schema": SCHEMA, "boot_id": BOOT, "motor_uids": dict(UIDS),
        "stance_capture_boot_id": BOOT,
        "stance_capture_motor_uids": dict(UIDS),
        "stance_capture_raw_rad_by_id": dict(target),
        "hold_summary_sha256": "a"*64,
        "stance_capture_sha256": "b"*64,
        "physical_route_review_sha256": "c"*64,
        "segment_clearance_sha256": ["d"*64],
        "clearance_reviewed_start_envelope_deg": 3.,
        "reviewed_raw_corridor_by_id": {str(i): {
            'min_rad': START[str(i)] - math.radians(4.),
            'max_rad': START[str(i)] + math.radians(100.),
        } for i in IDS},
        "raw_corridor_physical_source_note": "Virtual reviewed raw corridor; no hardware claim.",
        "hold_status": "CURRENT_HOLD_COMPLETED_RESET_CONFIRMED",
        "hold_stop_confirmed": True,
        "old_d17_target_reused": False,
        "learned_policy_allowed": False,
        "automatic_retry_allowed": False,
        "start_raw_rad_by_id": dict(START),
        "waypoints_raw_rad_by_id": [target],
        "fixed_stance_raw_rad_by_id": target,
        **{flag: True for flag in REQUIRED_REVIEWS},
    }


class CandidateTests(unittest.TestCase):
    def prepare(self, review=None, *, boot=BOOT, uids=None, fresh=None):
        return prepare_candidate(reviewed() if review is None else review,
                                 current_boot_id=boot,
                                 current_motor_uids=dict(UIDS) if uids is None else uids,
                                 fresh_raw_rad_by_id=dict(START) if fresh is None else fresh)

    def test_one_finite_segment_has_exact_endpoints_and_no_output(self):
        result = self.prepare()
        self.assertEqual(result["sample_count"], RAMP_TICKS + HOLD_TICKS)
        self.assertEqual(result["duration_s"], 9.)
        self.assertEqual(result["samples"][0]["raw_rad_by_id"], START)
        self.assertEqual(result["samples"][-1]["raw_rad_by_id"], at_offset(5.))
        self.assertEqual(result["status"], "OFFLINE_FINITE_STANCE_CANDIDATE_ONLY")
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["live_runner_available"])
        self.assertFalse(result["policy_handoff_allowed"])
        self.assertTrue(result["support_required"])
        self.assertFalse(result["self_supported_standing_verified"])
        self.assertTrue(all(earlier["elapsed_s"] < later["elapsed_s"]
                            for earlier, later in zip(result["samples"], result["samples"][1:])))

    def test_two_explicit_segments_are_contiguous_and_within_limits(self):
        review = reviewed(degrees=15.)
        review["waypoints_raw_rad_by_id"] = [at_offset(8.), at_offset(15.)]
        review["segment_clearance_sha256"] = ["d"*64, "e"*64]
        result = self.prepare(review)
        self.assertEqual(result["waypoint_count"], 2)
        self.assertEqual(result["sample_count"], 2 * (RAMP_TICKS + HOLD_TICKS))
        self.assertEqual(result["samples"][RAMP_TICKS + HOLD_TICKS - 1]["raw_rad_by_id"],
                         result["samples"][RAMP_TICKS + HOLD_TICKS]["raw_rad_by_id"])
        self.assertEqual(result["samples"][-1]["raw_rad_by_id"], at_offset(15.))

    def test_large_gap_requires_nine_separately_reviewed_segments(self):
        review = reviewed(degrees=90.)
        review["waypoints_raw_rad_by_id"] = [at_offset(10.*step) for step in range(1, 10)]
        review["segment_clearance_sha256"] = [f"{index:064x}" for index in range(1, 10)]
        result = self.prepare(review)
        self.assertEqual(result["waypoint_count"], 9)
        self.assertEqual(result["duration_s"], 81.)
        self.assertFalse(result["output_allowed"])

    def test_stale_boot_uid_or_start_fails_before_plan(self):
        with self.assertRaisesRegex(ValueError, "boot IDs differ"):
            self.prepare(boot="prior-boot")
        uids = dict(UIDS)
        uids["7"] = "replacement"
        with self.assertRaisesRegex(ValueError, "identities differ"):
            self.prepare(uids=uids)
        fresh = dict(START)
        fresh["7"] += math.radians(3.01)
        with self.assertRaisesRegex(ValueError, "ID7 left reviewed start envelope"):
            self.prepare(fresh=fresh)

    def test_old_d17_or_unreviewed_contact_fails(self):
        review = reviewed()
        review["old_d17_target_reused"] = True
        with self.assertRaisesRegex(ValueError, "not D17"):
            self.prepare(review)
        review = reviewed()
        review["front_upper_leg_carbon_clamp_clearance_verified"] = False
        with self.assertRaisesRegex(ValueError, "front_upper_leg_carbon_clamp"):
            self.prepare(review)
        review = reviewed()
        review["clearance_reviewed_start_envelope_deg"] = 5.
        with self.assertRaisesRegex(ValueError, "exact three-degree"):
            self.prepare(review)

    def test_each_segment_and_clearance_evidence_is_required(self):
        review = reviewed(degrees=MAX_SEGMENT_DEG + .01)
        with self.assertRaisesRegex(ValueError, "exceeds ten raw degrees"):
            self.prepare(review)
        review = reviewed()
        review["segment_clearance_sha256"] = []
        with self.assertRaisesRegex(ValueError, "Each segment needs"):
            self.prepare(review)
        review = reviewed()
        review["segment_clearance_sha256"] = ["not-a-digest"]
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            self.prepare(review)

    def test_hold_stop_target_integrity_and_policy_refused(self):
        review = reviewed()
        review["hold_stop_confirmed"] = False
        with self.assertRaisesRegex(ValueError, "all twelve STOPs"):
            self.prepare(review)
        review = reviewed()
        review["fixed_stance_raw_rad_by_id"] = at_offset(4.)
        with self.assertRaisesRegex(ValueError, "Final waypoint differs"):
            self.prepare(review)
        review = reviewed()
        review["stance_capture_raw_rad_by_id"] = at_offset(4.)
        with self.assertRaisesRegex(ValueError, "Final waypoint differs"):
            self.prepare(review)
        review = reviewed()
        review["learned_policy_allowed"] = True
        with self.assertRaisesRegex(ValueError, "Policy handoff"):
            self.prepare(review)

    def test_malformed_or_extra_motor_rejected(self):
        review = reviewed()
        review["waypoints_raw_rad_by_id"][0]["13"] = 0.
        with self.assertRaisesRegex(ValueError, "exactly string IDs"):
            self.prepare(review)
        review = reviewed()
        review["waypoints_raw_rad_by_id"][0]["2"] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            self.prepare(review)

    def test_every_sample_stays_inside_reviewed_raw_corridor(self):
        review = reviewed()
        review['reviewed_raw_corridor_by_id']['7']['max_rad'] = (
            START['7'] + math.radians(4.))
        with self.assertRaisesRegex(ValueError, 'ID7 leaves reviewed raw corridor'):
            self.prepare(review)
        review = reviewed()
        review['reviewed_raw_corridor_by_id']['7']['min_rad'] = START['7'] + .001
        with self.assertRaisesRegex(ValueError, 'fresh start outside reviewed raw corridor'):
            self.prepare(review)


if __name__ == "__main__":
    unittest.main()
