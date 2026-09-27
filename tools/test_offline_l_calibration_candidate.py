"""Synthetic, file-only checks for the nominal L audit."""

import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from tools import offline_l_calibration_candidate as audit


def fixtures():
    uids = {str(i): f"{i:016x}" for i in audit.IDS}
    raw_l = {str(i): float(i) / 10 for i in audit.IDS}

    def capture(raw):
        return {"schema": audit.CAPTURE_SCHEMA, "status": "RECORDED_REVIEW_REQUIRED",
                "boot_id": "boot-a", "errors": [], "output_allowed": False,
                "approved_for_runtime": False, "source_sha256": {"capture": "test"},
                "plan": {"ids_by_bus": {"front": list(range(1, 7)),
                                        "rear": list(range(7, 13))},
                         "ports": {"front": "front", "rear": "rear"}},
                "identities": {mid: {"mcu_uid_hex": uid} for mid, uid in uids.items()},
                "pose": {"raw_rad_by_id": raw, "sampling_issues": [],
                         "sampling_stability_heuristic_passed": True}}

    box_raw = dict(raw_l)
    box_raw["10"] += math.radians(40)
    box_raw["11"] += math.radians(15)
    old = {"boot_id": "boot-old", "approved_for_runtime": False,
           "sign_revalidated": False, "motor_power_cycle_continuity_verified": False,
           "physical_angle_accuracy_verified": False,
           "candidates": [{"motor_id": i, "uid": uids[str(i)],
                           "sign_candidate": 1, "raw_L_median_rad": raw_l[str(i)]}
                          for i in audit.IDS]}

    def ab(label, id10):
        return {"status": f"READ_ONLY_{label}", "boot_id": "boot-a",
                "motor_output_allowed": False,
                "rows": {str(i): {"position_rad": id10 if i == 10 else float(i),
                                  "disabled_mode": True} for i in (10, 11, 12)}}

    a, b = ab("A", 3.), ab("B", 3. - math.radians(92))
    b["delta_deg_from_A"] = {str(i): math.degrees(
        b["rows"][str(i)]["position_rad"] - a["rows"][str(i)]["position_rad"])
        for i in (10, 11, 12)}
    return capture(raw_l), capture(box_raw), old, a, b


class OfflineNominalLTests(unittest.TestCase):
    def test_two_sign_hypotheses_remain_unverified_and_use_direct_deltas(self):
        data = fixtures()
        report = audit.build_review(*data)
        self.assertEqual(report["status"], "REVIEW_REQUIRED_NO_RUNTIME_PROMOTION")
        self.assertFalse(report["output_allowed"])
        self.assertFalse(report["sign_revalidated"])
        self.assertEqual(report["priority_branch_review_ids"], [1, 3, 9])
        self.assertEqual(report["all_direct_deltas_over_180_deg_ids"], [])
        row = report["rows"][9]
        self.assertAlmostEqual(row["box_minus_l_direct_deg"], 40)
        self.assertEqual(report["fk_hypotheses"]["historical_id10_plus"]["id10_sign"], 1)
        self.assertEqual(report["fk_hypotheses"]["id10_reversed_for_review"]["id10_sign"], -1)
        self.assertNotAlmostEqual(
            report["fk_hypotheses"]["historical_id10_plus"]["box_rl_to_other_three_foot_center_plane_mm"],
            report["fk_hypotheses"]["id10_reversed_for_review"]["box_rl_to_other_three_foot_center_plane_mm"])

    def test_rejects_boot_uid_and_id10_ab_mismatch(self):
        l, box, old, a, b = fixtures()
        box["boot_id"] = "boot-b"
        with self.assertRaisesRegex(ValueError, "boot, UIDs"):
            audit.build_review(l, box, old, a, b)
        l, box, old, a, b = fixtures()
        box["identities"]["12"]["mcu_uid_hex"] = "f"*16
        with self.assertRaisesRegex(ValueError, "boot, UIDs"):
            audit.build_review(l, box, old, a, b)
        l, box, old, a, b = fixtures()
        b["rows"]["11"]["position_rad"] += math.radians(4)
        b["delta_deg_from_A"]["11"] = 4.
        with self.assertRaisesRegex(ValueError, "isolation"):
            audit.build_review(l, box, old, a, b)

    def test_after_pose_requires_sha_identities_and_all_twenty_cycles(self):
        l, *_ = fixtures()
        uids = {i: l["identities"][str(i)]["mcu_uid_hex"] for i in audit.IDS}
        streams = {}
        for bus, ids in (("front", range(1, 7)), ("rear", range(7, 13))):
            events = []
            for i in ids:
                events.append({"kind": "pipeline_reply", "ok": True, "cycle": 0,
                               "motor_id": i, "parameter": "identity",
                               "result": {"mcu_uid_hex": uids[i]}})
                for cycle in range(1, 21):
                    events.append({"kind": "pipeline_reply", "ok": True,
                                   "cycle": cycle, "motor_id": i, "parameter": "position",
                                   "result": {"index": 0x7019, "unit": "rad_output_shaft",
                                              "value": float(i)}})
            streams[bus] = ("\n".join(json.dumps(e) for e in events) + "\n").encode()
        summary = {"status": "COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT",
                   "boot_id": "boot-a", "errors": [], "output_allowed": False,
                   "approved_for_runtime": False, "plan": {"cycles_per_bus": 20},
                   "events_sha256": {bus: hashlib.sha256(blob).hexdigest()
                                     for bus, blob in streams.items()}}
        pose, span = audit._after_pose(summary, streams, "boot-a", uids)
        self.assertEqual(pose[10], 10.)
        self.assertEqual(span[10], 0.)
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            audit._after_pose(summary, {**streams, "rear": streams["rear"] + b" "},
                              "boot-a", uids)
        truncated = streams["rear"].rsplit(b"\n", 2)[0] + b"\n"
        summary["events_sha256"]["rear"] = hashlib.sha256(truncated).hexdigest()
        with self.assertRaisesRegex(ValueError, "incomplete"):
            audit._after_pose(summary, {**streams, "rear": truncated}, "boot-a", uids)

    def test_private_output_path_refuses_git_and_existing_file(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.assertEqual(audit._private_new_path(folder / "review.json"),
                             (folder / "review.json").resolve())
            (folder / "review.json").write_text("existing")
            with self.assertRaisesRegex(ValueError, "fresh"):
                audit._private_new_path(folder / "review.json")
            (folder / ".git").mkdir()
            with self.assertRaisesRegex(ValueError, "outside Git"):
                audit._private_new_path(folder / "other.json")


if __name__ == "__main__":
    unittest.main()
