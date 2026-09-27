"""Synthetic, file-only checks for the box-supported lift screen."""

import hashlib
import math
from pathlib import Path
import tempfile
import unittest

from tools import screen_box_lift_offline as screen


URDF = (Path(__file__).resolve().parents[2] / "FaBoRobotDog_L17_physical_r1"
        / "assets/urdf/fabo_robotdog_d17_hardware_r1.urdf")


def fixtures():
    signs = {i: 1 for i in screen.IDS}
    signs.update({3: -1, 5: -1, 6: -1, 11: -1})
    uids = {i: f"{i:016x}" for i in screen.IDS}
    review = {"schema": "singularitydog.offline-nominal-l-review.v1",
              "status": "REVIEW_REQUIRED_NO_RUNTIME_PROMOTION",
              "boot_id": "boot-a", "d17_urdf_sha256": screen.URDF_SHA256,
              "sign_revalidated": False, "physical_angle_accuracy_verified": False,
              "output_allowed": False, "approved_for_runtime": False,
              "command_bytes_generated": False,
              "id10_manual_b_minus_a_deg": {"10": -92.0},
              "rows": [{"motor_id": i, "uid": uids[i], "raw_l_rad": 0.,
                        "historical_sign": signs[i],
                        "nominal_l_rad": -math.pi/2 if i in (1, 4, 7, 10) else 0.}
                       for i in screen.IDS]}
    raw = {i: 0. for i in screen.IDS}
    raw.update({3: -2*math.pi, 9: 2*math.pi, 10: .65, 11: .25, 12: .1})
    snapshot = {"status": "READ_ONLY_12_BOX_AFTER_ID10_MANUAL",
                "boot_id": "boot-a", "motor_output_allowed": False,
                "rows": {str(i): {"uid": uids[i], "disabled_mode": True,
                                  "current_A": 0.0, "positions_rad": [raw[i]]*3,
                                  "median_position_rad": raw[i]}
                         for i in screen.IDS}}
    return snapshot, review


def camera_fixtures():
    snapshot, review = fixtures()
    snapshot["status"] = "READ_ONLY_BOX_AFTER_ALL_CAMERA_L"
    snapshot["created_ns"] = 50
    for row in snapshot["rows"].values():
        row["run_mode"] = 0
        row["span_deg"] = .01
        del row["disabled_mode"]
        del row["positions_rad"]

    def leg_record(leg, created_ns, status=None):
        ids = screen.LEGS[leg]
        return {"status": status or f"READ_ONLY_{leg}_CAMERA_L",
                "boot_id": "boot-a", "created_ns": created_ns,
                "motor_output_allowed": False,
                "rows": {str(i): {"uid": snapshot["rows"][str(i)]["uid"],
                                  "run_mode": 0, "current_A": 0.0,
                                  "position_median_rad": snapshot["rows"][str(i)]["median_position_rad"],
                                  "position_span_deg": .01}
                         for i in ids}}

    rl_l = leg_record("RL", 20)
    rl_match = leg_record("RL", 10, "READ_ONLY_RL_CAMERA_MATCH")
    rl_match["rows"]["10"]["position_median_rad"] -= math.radians(65)
    others = {leg: leg_record(leg, time)
              for leg, time in (("FR", 30), ("FL", 35), ("RR", 40))}
    return snapshot, review, rl_l, rl_match, others


class BoxLiftOfflineScreenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = URDF.read_bytes()
        cls.geometry = screen.parse_d17(cls.urdf)

    def test_actual_d17_limits_and_equal_foot_geometry(self):
        self.assertEqual(hashlib.sha256(self.urdf).hexdigest(), screen.URDF_SHA256)
        for leg in screen.LEGS:
            g = self.geometry[leg]
            self.assertAlmostEqual(g["upper_m"], .12)
            self.assertAlmostEqual(g["lower_m"], .12)
            self.assertAlmostEqual(g["foot_radius_m"], .024)
            self.assertEqual(g["limits"], ((-2.2, -.08), (-.9, 1.2), (-.5, .5)))

    def test_direct_turn_branch_both_sign_ranges_and_contact_residual(self):
        snapshot, review = fixtures()
        report = screen.build_report(snapshot, review, self.geometry)
        self.assertEqual(report["status"], "NOT_READY")
        self.assertFalse(report["motor_commands_generated"])
        self.assertFalse(report["angle_wrapping_applied"])
        self.assertAlmostEqual(report["direct_raw_minus_l_deg_by_id"]["3"], -360)
        self.assertAlmostEqual(report["direct_raw_minus_l_deg_by_id"]["9"], 360)
        self.assertEqual([reason["code"] for reason in report["reasons"]], [
            "UNRESOLVED_360_DEG_BRANCH_ID3_ID9",
            "ID10_SIGN_AND_L_ORIGIN_UNVERIFIED",
            "FOUR_FOOT_CONTACT_GEOMETRY_RESIDUAL",
            "LIFT_TRAJECTORY_AND_LOAD_SUPPORT_UNREVIEWED"])
        old = report["hypotheses"]["historical_id10_plus"]
        flipped = report["hypotheses"]["id10_flipped_for_review"]
        self.assertEqual(old["out_of_range_ids"], [3, 9])
        self.assertEqual(flipped["out_of_range_ids"], [3, 9, 10])
        self.assertGreater(old["rl_to_other_three_foot_center_plane_mm"], 5)
        self.assertGreater(flipped["rl_to_other_three_foot_center_plane_mm"], 5)
        self.assertNotAlmostEqual(old["rl_to_other_three_foot_center_plane_mm"],
                                  flipped["rl_to_other_three_foot_center_plane_mm"])

    def test_rejects_boot_uid_disabled_current_and_changed_median(self):
        for change, error in (
            (lambda s, r: s.update(boot_id="other"), "boot IDs"),
            (lambda s, r: s["rows"]["7"].update(uid="f"*16), "ID7 UID"),
            (lambda s, r: s["rows"]["8"].update(disabled_mode=False), "ID8 is not"),
            (lambda s, r: s["rows"]["9"].update(current_A=.01), "ID9 is not"),
            (lambda s, r: s["rows"]["9"].update(current_A=False), "ID9 is not"),
            (lambda s, r: s["rows"]["10"].update(median_position_rad=1.2), "ID10 median"),
        ):
            with self.subTest(error=error):
                snapshot, review = fixtures()
                change(snapshot, review)
                with self.assertRaisesRegex(ValueError, error):
                    screen.build_report(snapshot, review, self.geometry)

    def test_even_coplanar_hypotheses_cannot_authorize_lift(self):
        snapshot, review = fixtures()
        for i in screen.IDS:
            snapshot["rows"][str(i)]["positions_rad"] = [0.]*3
            snapshot["rows"][str(i)]["median_position_rad"] = 0.
        report = screen.build_report(snapshot, review, self.geometry)
        self.assertEqual(report["status"], "NOT_READY")
        self.assertEqual([reason["code"] for reason in report["reasons"]],
                         ["ID10_SIGN_AND_L_ORIGIN_UNVERIFIED",
                          "LIFT_TRAJECTORY_AND_LOAD_SUPPORT_UNREVIEWED"])
        self.assertFalse(report["approved_for_runtime"])
        self.assertFalse(report["physical_contact_verified_by_this_screen"])

    def test_unmeasured_side_view_adds_context_without_approving_hypothesis(self):
        snapshot, review = fixtures()
        note = {"schema": "singularitydog.rl-sideview-note.v1",
                "status": "QUALITATIVE_UNVERIFIED", "leg": "RL",
                "motor_ids": [10, 11, 12], "source_image_saved": False,
                "physical_joint_angle_measured": False,
                "leg_identified_by_operator": True,
                "approximate_image_landmarks_px": {
                    "proximal_center": [960, 300], "knee_center": [790, 665],
                    "paw_center": [805, 920]},
                "fresh_rear_read_summary": {
                    "source_record_checked_by_screen": False,
                    "raw_deg_by_id": {"10": 181.4, "11": 23.6, "12": 92.5}}}
        report = screen.build_report(snapshot, review, self.geometry, visual_note=note)
        self.assertEqual(report["status"], "NOT_READY")
        self.assertFalse(report["visual_note"]["physical_joint_angle_measured"])
        self.assertIn("SIDE_VIEW_IS_NOT_ANGLE_METROLOGY",
                      [reason["code"] for reason in report["reasons"]])
        note["physical_joint_angle_measured"] = True
        with self.assertRaisesRegex(ValueError, "unmeasured"):
            screen.build_report(snapshot, review, self.geometry, visual_note=note)

    def test_four_same_boot_camera_l_references_resolve_branches_not_admission(self):
        snapshot, review, rl_l, rl_match, others = camera_fixtures()
        report = screen.build_report(snapshot, review, self.geometry,
                                     rl_camera_l=rl_l, rl_camera_match=rl_match,
                                     other_camera_l=others)
        selected = report["hypotheses"]["available_camera_l_rebased_for_review"]
        self.assertEqual(report["available_operator_identified_l_legs"],
                         ["FL", "FR", "RL", "RR"])
        self.assertEqual(selected["out_of_range_ids"], [])
        self.assertAlmostEqual(selected["rl_to_other_three_foot_center_plane_mm"], 0)
        self.assertAlmostEqual(selected["foot_center_z_spread_mm"], 0)
        self.assertAlmostEqual(selected["fr_fl_rr_plane_tilt_from_body_horizontal_deg"], 0)
        self.assertEqual(report["status"], "NOT_READY")
        self.assertFalse(report["simultaneous_fullbody_l_pose_verified"])
        self.assertNotIn("UNRESOLVED_360_DEG_BRANCH_ID3_ID9",
                         [reason["code"] for reason in report["reasons"]])
        self.assertFalse(report["approved_for_runtime"])
        self.assertFalse(report["motor_commands_generated"])

    def test_camera_l_rejects_uid_mode_current_and_missing_leg(self):
        for change, error in (
            (lambda s, l, m, o: o["FR"]["rows"]["1"].update(uid="f"*16),
             "FR camera L ID1"),
            (lambda s, l, m, o: o["FL"]["rows"]["4"].update(run_mode=1),
             "FL camera L ID4"),
            (lambda s, l, m, o: l["rows"]["10"].update(current_A=.01),
             "RL l ID10"),
            (lambda s, l, m, o: o.pop("RR"),
             "requires all four"),
        ):
            with self.subTest(error=error):
                snapshot, review, rl_l, rl_match, others = camera_fixtures()
                change(snapshot, rl_l, rl_match, others)
                with self.assertRaisesRegex(ValueError, error):
                    screen.build_report(snapshot, review, self.geometry,
                                        rl_camera_l=rl_l, rl_camera_match=rl_match,
                                        other_camera_l=others)

    def test_plane_residual_can_hide_large_torso_tilt(self):
        points = {"FR": (0., 0., 0.), "FL": (0., 1., -.12),
                  "RR": (1., 0., 0.), "RL": (1., 1., -.12)}
        self.assertAlmostEqual(screen.rl_plane_residual_mm(points), 0)
        self.assertAlmostEqual(screen.foot_height_spread_mm(points), 120)
        self.assertGreater(screen.reference_plane_tilt_deg(points), 6)

    def test_uncalibrated_imu_slope_agreement_is_context_only(self):
        snapshot, review, rl_l, rl_match, others = camera_fixtures()
        imu = {"status": "RECORDED_NOT_CALIBRATED", "restore_status": "restored",
               "errors": [], "plan": {"can_opened": False,
                                        "calibration_applied": False,
                                        "orientation_verified_by_software": False},
               "summary": {"samples": 968, "calibration_applied": False,
                           "stillness_or_orientation_confirmed": False,
                           "accel_mean_m_s2": [0., 0., -10.67],
                           "gravity_norm_deviation_percent": 8.8}}
        mount = {"status": "IMU_MOUNT_CANDIDATE_ONLY",
                 "approved_for_runtime": False, "raw_driver_axes_verified": False,
                 "R_body_from_sensor": [[0, 1, 0], [1, 0, 0], [0, 0, -1]]}
        report = screen.build_report(snapshot, review, self.geometry,
                                     rl_camera_l=rl_l, rl_camera_match=rl_match,
                                     other_camera_l=others,
                                     imu_summary=imu, imu_mount=mount)
        comparison = report["imu_comparison"]
        self.assertAlmostEqual(comparison["absolute_pitch_roll_difference_deg"][0], 0)
        self.assertFalse(comparison["calibration_applied"])
        self.assertFalse(comparison["time_synchronized_to_joint_capture"])
        self.assertEqual(report["status"], "NOT_READY")
        self.assertIn("IMU_CALIBRATION_AND_MOUNT_UNVERIFIED",
                      [reason["code"] for reason in report["reasons"]])
        mount["approved_for_runtime"] = True
        with self.assertRaisesRegex(ValueError, "mount"):
            screen.build_report(snapshot, review, self.geometry,
                                rl_camera_l=rl_l, rl_camera_match=rl_match,
                                other_camera_l=others,
                                imu_summary=imu, imu_mount=mount)

    def test_pinned_source_and_private_fresh_path(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            source = directory / "source.json"
            source.write_bytes(b"{}")
            self.assertEqual(screen.read_pinned(source, hashlib.sha256(b"{}").hexdigest(),
                                                "test"), b"{}")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                screen.read_pinned(source, "0"*64, "test")
            report = directory / "report.json"
            self.assertEqual(screen.private_new_path(report), report.resolve())
            report.write_text("occupied")
            with self.assertRaisesRegex(ValueError, "fresh"):
                screen.private_new_path(report)
            (directory / ".git").mkdir()
            with self.assertRaisesRegex(ValueError, "outside Git"):
                screen.private_new_path(directory / "next.json")


if __name__ == "__main__":
    unittest.main()
