"""Synthetic, file-only validation of the nominal box-rise geometry candidate."""

from contextlib import redirect_stdout
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from tools import offline_box_rise_candidate as candidate
from tools import screen_box_lift_offline as screen


URDF = (Path(__file__).resolve().parents[2] / "FaBoRobotDog_L17_physical_r1"
        / "assets/urdf/fabo_robotdog_d17_hardware_r1.urdf")


def fixture():
    uids = {str(i): f"{i:016x}" for i in candidate.IDS}
    hashes = {leg: f"{index:064x}" for index, leg in enumerate(screen.LEGS, 1)}
    q_l = {i: -math.pi/2 if i in (1, 4, 7, 10) else 0.
           for i in candidate.IDS}
    q_floor = {i: (-1.4 if i in (1, 4, 7, 10)
                   else .2 if i in (2, 5, 8, 11) else 0.)
               for i in candidate.IDS}
    raw_floor = {i: q_floor[i]-q_l[i] for i in candidate.IDS}
    cameras = {}
    for leg, ids in screen.LEGS.items():
        cameras[leg] = {
            "status": f"READ_ONLY_{leg}_CAMERA_L", "boot_id": "boot-a",
            "created_ns": 100, "motor_output_allowed": False,
            "rows": {str(i): {"uid": uids[str(i)], "run_mode": 0,
                              "current_A": 0., "position_median_rad": 0.,
                              "position_span_deg": 0.01} for i in ids},
        }
    floor = {
        "status": "READ_ONLY_BOX_AFTER_ALL_CAMERA_L", "boot_id": "boot-a",
        "created_ns": 200, "motor_output_allowed": False,
        "rows": {str(i): {"uid": uids[str(i)], "run_mode": 0,
                          "current_A": 0., "median_position_rad": raw_floor[i],
                          "span_deg": .01} for i in candidate.IDS},
    }
    angles = {
        "schema": "singularitydog.box-rise-angle-review.v1",
        "boot_id": "boot-a", "motor_uids": uids,
        "camera_l_sha256_by_leg": dict(hashes),
        "axes": {str(i): {"sign": 1, "sign_physically_revalidated": True,
                          "l_model_angle_rad": q_l[i],
                          "l_model_angle_measured": True} for i in candidate.IDS},
    }
    physical = {
        "schema": "singularitydog.box-rise-physical-review.v1",
        "boot_id": "boot-a", "motor_uids": uids,
        "floor_snapshot_sha256": "f"*64,
        "all_four_paws_on_floor_confirmed": True,
        "box_directly_under_torso_confirmed": True,
        "torso_supported_by_box_confirmed": True,
        "full_nonfoot_sweep_measured": True,
        "floor_contact_evidence_ref": "synthetic video",
        "clearance_evidence_ref": "synthetic measured clearance",
        "raw_corridor_evidence_ref": "synthetic measured range",
        "reviewed_floor_start_raw_rad_by_id": {
            str(i): raw_floor[i] for i in candidate.IDS},
        "measured_raw_corridor_by_id": {
            str(i): {"min_rad": -1., "max_rad": 1.} for i in candidate.IDS},
        "minimum_measured_nonfoot_clearance_mm_by_leg": {
            leg: 30. for leg in screen.LEGS},
        "clearance_measurement_uncertainty_mm": 1.,
    }
    return {
        "floor_snapshot": floor, "floor_snapshot_sha256": "f"*64,
        "camera_l_by_leg": cameras, "camera_l_sha256_by_leg": hashes,
        "angle_review": angles, "physical_review": physical,
        "geometry": screen.parse_d17(URDF.read_bytes()),
        "current_boot_id": "boot-a", "current_motor_uids": uids,
        "rise_mm": 3.,
    }


class OfflineBoxRiseCandidateTests(unittest.TestCase):
    def test_geometry_path_keeps_four_feet_fixed_relative_to_floor(self):
        args = fixture()
        result = candidate.build_candidate(**args)
        self.assertEqual(result["status"], "OFFLINE_GEOMETRY_CANDIDATE_ONLY")
        self.assertFalse(result["motor_output_allowed"])
        self.assertFalse(result["live_runner_available"])
        self.assertFalse(result["angle_wrapping_applied"])
        self.assertEqual(result["sample_count"], 101)
        initial = None
        for sample in result["samples"]:
            model = {i: (-math.pi/2 if i in (1, 4, 7, 10) else 0.)
                     + sample["raw_rad_by_id"][str(i)] for i in candidate.IDS}
            feet = screen.foot_centers(model, args["geometry"])
            if initial is None:
                initial = feet
            for leg in screen.LEGS:
                self.assertAlmostEqual(feet[leg][0], initial[leg][0], places=4)
                self.assertAlmostEqual(feet[leg][1], initial[leg][1], places=4)
                self.assertAlmostEqual(feet[leg][2],
                                       initial[leg][2]-sample["body_rise_mm"]/1000.,
                                       places=4)
        self.assertAlmostEqual(result["samples"][50]["body_rise_mm"], 3.)
        self.assertAlmostEqual(result["samples"][-1]["body_rise_mm"], 0.)
        for mid in candidate.IDS:
            self.assertAlmostEqual(
                result["samples"][0]["raw_rad_by_id"][str(mid)],
                result["samples"][-1]["raw_rad_by_id"][str(mid)], places=5)

    def test_rejects_missing_sign_or_angle_zero(self):
        for change in (
            lambda x: x["angle_review"]["axes"].pop("4"),
            lambda x: x["angle_review"]["axes"]["4"].pop("sign"),
            lambda x: x["angle_review"]["axes"]["4"].update(
                sign_physically_revalidated=False),
            lambda x: x["angle_review"]["axes"]["4"].pop("l_model_angle_rad"),
            lambda x: x["angle_review"]["axes"]["4"].update(
                l_model_angle_measured=False),
        ):
            args = fixture()
            change(args)
            with self.assertRaises(ValueError):
                candidate.build_candidate(**args)

    def test_rejects_stale_or_unsupported_floor_pose(self):
        args = fixture()
        args["physical_review"]["reviewed_floor_start_raw_rad_by_id"]["8"] += math.radians(1.)
        with self.assertRaisesRegex(ValueError, "supported start envelope"):
            candidate.build_candidate(**args)
        args = fixture()
        args["floor_snapshot"]["rows"]["8"]["run_mode"] = 2
        with self.assertRaisesRegex(ValueError, "disabled mode"):
            candidate.build_candidate(**args)
        args = fixture()
        args["physical_review"]["all_four_paws_on_floor_confirmed"] = False
        with self.assertRaisesRegex(ValueError, "four_paws"):
            candidate.build_candidate(**args)

    def test_rejects_joint_limit_and_clearance_shortfalls(self):
        args = fixture()
        for i in (1, 4, 7, 10):
            args["angle_review"]["axes"][str(i)]["l_model_angle_rad"] = -2.38
        with self.assertRaisesRegex(ValueError, "joint limit"):
            candidate.build_candidate(**args)
        args = fixture()
        args["physical_review"]["minimum_measured_nonfoot_clearance_mm_by_leg"]["FL"] = 8.
        with self.assertRaisesRegex(ValueError, "clearance"):
            candidate.build_candidate(**args)
        args = fixture()
        args["physical_review"]["clearance_measurement_uncertainty_mm"] = 4.
        with self.assertRaisesRegex(ValueError, "uncertainty"):
            candidate.build_candidate(**args)

    def test_rejects_boot_uid_and_camera_source_mismatch(self):
        mutations = (
            lambda x: x.update(current_boot_id="different"),
            lambda x: x["camera_l_by_leg"]["RR"].update(boot_id="different"),
            lambda x: x["floor_snapshot"]["rows"]["3"].update(uid="f"*16),
            lambda x: x["angle_review"]["motor_uids"].update({"3": "f"*16}),
            lambda x: x["angle_review"]["camera_l_sha256_by_leg"].update(
                {"FR": "9"*64}),
        )
        for change in mutations:
            args = fixture()
            change(args)
            with self.assertRaises(ValueError):
                candidate.build_candidate(**args)

    def test_rejects_oversize_rise(self):
        args = fixture()
        args["rise_mm"] = 6.
        with self.assertRaisesRegex(ValueError, "2–5"):
            candidate.build_candidate(**args)

    def test_rejects_target_outside_type1_codec_headroom(self):
        args = fixture()
        for i in candidate.IDS:
            args["floor_snapshot"]["rows"][str(i)]["median_position_rad"] += 12.5
            args["physical_review"]["reviewed_floor_start_raw_rad_by_id"][str(i)] += 12.5
            args["physical_review"]["measured_raw_corridor_by_id"][str(i)] = {
                "min_rad": 12., "max_rad": 13.}
        with self.assertRaisesRegex(ValueError, "Type1 position range"):
            candidate.build_candidate(**args)

    def test_fixed_hip_plan_uses_only_eight_calibrated_axes(self):
        args = fixture()
        args['rise_mm'] = 2.
        args['fixed_hip'] = True
        args['physical_review']['max_abs_hip_angle_deg'] = 17.
        for mid in candidate.HIP_IDS:
            args['angle_review']['axes'][str(mid)] = {}
        result = candidate.build_candidate(**args)
        self.assertTrue(result['fixed_hip'])
        self.assertIsNone(result['initial_foot_plane_residual_mm'])
        self.assertLess(result['hip_lateral_paw_drift_bound_mm'], .75)
        first = result['samples'][0]['raw_rad_by_id']
        for sample in result['samples']:
            for mid in candidate.HIP_IDS:
                self.assertEqual(sample['raw_rad_by_id'][str(mid)], first[str(mid)])
        self.assertAlmostEqual(result['samples'][50]['body_rise_mm'], 2.)

    def test_fixed_hip_rejects_excessive_lateral_drift_assumption(self):
        args = fixture()
        args['rise_mm'] = 5.
        args['fixed_hip'] = True
        args['physical_review']['max_abs_hip_angle_deg'] = 20.
        with self.assertRaisesRegex(ValueError, 'lateral paw drift'):
            candidate.build_candidate(**args)

    def test_cli_hashes_sources_and_writes_private_offline_output(self):
        args = fixture()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            files = {}

            def save(label, data):
                path = root / (label + ".json")
                content = json.dumps(data).encode()
                path.write_bytes(content)
                files[label] = path
                return hashlib.sha256(content).hexdigest()

            floor_hash = save("floor-snapshot", args["floor_snapshot"])
            args["physical_review"]["floor_snapshot_sha256"] = floor_hash
            for leg in screen.LEGS:
                digest = save(leg.lower() + "-camera-l", args["camera_l_by_leg"][leg])
                args["angle_review"]["camera_l_sha256_by_leg"][leg] = digest
            save("angle-review", args["angle_review"])
            save("physical-review", args["physical_review"])
            save("current-uids", args["current_motor_uids"])
            output = root / "candidate.json"
            command = []
            for label in ("floor-snapshot", "fr-camera-l", "fl-camera-l",
                          "rr-camera-l", "rl-camera-l", "angle-review",
                          "physical-review", "current-uids"):
                command += ["--" + label, str(files[label])]
            command += ["--urdf", str(URDF), "--current-boot-id", "boot-a",
                        "--rise-mm", "3", "--output", str(output)]
            with redirect_stdout(io.StringIO()):
                self.assertEqual(candidate.main(command), 0)
            result = json.loads(output.read_text())
            self.assertEqual(result["floor_snapshot_sha256"], floor_hash)
            self.assertFalse(result["motor_output_allowed"])
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
