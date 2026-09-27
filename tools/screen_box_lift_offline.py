"""File-only D17 screen for a box-supported, few-mm torso lift.

This reads saved evidence and URDF geometry. It has no hardware imports, does
not generate motor targets, and always leaves runtime admission closed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET


IDS = tuple(range(1, 13))
LEGS = {"FR": (1, 2, 3), "FL": (4, 5, 6),
        "RR": (7, 8, 9), "RL": (10, 11, 12)}
SNAPSHOT_SHA256 = "4a230d221c841905e863a1aca339c7dd22d179c3174c3ca2c93d0d966c21c04d"
LATEST_SNAPSHOT_SHA256 = "5fcfdf4d2ef3405c9b6a516f072c5da715af1e053c94dbc4bbf230638f2d2755"
FINAL_SNAPSHOT_SHA256 = "1706efbe3bd315e30e9e0063ab49f71b47a6c2b36275ca97ba1b90a8bef0c995"
REVIEW_SHA256 = "748405d5ffa0c48d605faf03007a9d4abb2630c7b173fb9d6075b49c8654f6fa"
URDF_SHA256 = "ba77462679268455d547848e76925dcc1f75a9b497fe4814c92ac1e7492496c8"
RL_CAMERA_L_SHA256 = "937c865cb9954a361fbd815b6d5508b155349ff38b84a985af90ddf1ba55781e"
RL_CAMERA_MATCH_SHA256 = "abc47bc9dc91d00feacb44504fd3b40579bee2a88ea3d8101496b45615c43ce5"
FR_CAMERA_L_SHA256 = "de367f55f3776d04f4ad2acb65014a950ebe1b99e9c630c8bee202891c8cd949"
FL_CAMERA_L_SHA256 = "3f5c96335005ff283902c2ee58aa0e8a314523f75ec5b19e4a6c6cdaf574e611"
RR_CAMERA_L_SHA256 = "979e6239cddc8e5027097c19f7fb7f80b8563ac904ea7d85c64bed3c08676999"
IMU_SUMMARY_SHA256 = "35ad500b48d92dc482958e0cc7c5389e2fdd8ec53291134ab1c41a3ca370401c"
IMU_MOUNT_SHA256 = "b9f6814e13141e82efca0fbb64772847ae7ad91434282b8c9b1e038b3c42b420"
FEW_MM = 5.0  # Comparison scale, not a measured contact tolerance.


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), f"{label} must be finite")
    return float(value)


def read_pinned(path, expected_sha256, label):
    content = Path(path).read_bytes()
    allowed = (expected_sha256,) if type(expected_sha256) is str else expected_sha256
    require(hashlib.sha256(content).hexdigest() in allowed,
            f"{label} SHA-256 mismatch")
    return content


def _xyz(element, label):
    require(element is not None and type(element.get("xyz")) is str, f"{label} xyz missing")
    parts = element.get("xyz").split()
    require(len(parts) == 3, f"{label} xyz length invalid")
    return tuple(finite(float(part), label) for part in parts)


def _joint(root, name, axis):
    joint = root.find(f".//joint[@name='{name}']")
    require(joint is not None, f"D17 {name} missing")
    origin = joint.find("origin")
    xyz = _xyz(origin, name)
    require(origin.get("rpy") == "0 0 0", f"D17 {name} rotated origin")
    if axis is not None:
        require(_xyz(joint.find("axis"), name) == axis, f"D17 {name} axis differs")
        limit = joint.find("limit")
        require(limit is not None, f"D17 {name} limit missing")
        lower, upper = (finite(float(limit.get(key)), name + " " + key)
                        for key in ("lower", "upper"))
        require(lower < upper, f"D17 {name} limit inverted")
        return xyz, (lower, upper)
    return xyz, None


def parse_d17(urdf_bytes):
    """Extract only the origins, axes, limits and equal foot radii used by FK."""
    root = ET.fromstring(urdf_bytes)
    legs = {}
    for leg, ids in LEGS.items():
        hip, hip_limits = _joint(root, f"{leg}_hip_joint", (1., 0., 0.))
        offset, thigh_limits = _joint(root, f"{leg}_thigh_joint", (0., 1., 0.))
        upper, calf_limits = _joint(root, f"{leg}_calf_joint", (0., 1., 0.))
        lower, _ = _joint(root, f"{leg}_foot_fixed", None)
        foot = root.find(f".//link[@name='{leg}_foot']")
        spheres = [] if foot is None else foot.findall("./collision/geometry/sphere")
        require(len(spheres) == 1, f"D17 {leg} foot sphere missing")
        radius = finite(float(spheres[0].get("radius")), leg + " radius")
        require(radius > 0 and abs(hip[2]) < 1e-10
                and abs(offset[0]) < 1e-10 and abs(offset[2]) < 1e-10
                and all(abs(x) < 1e-10 for x in upper[:2] + lower[:2])
                and upper[2] < 0 and lower[2] < 0,
                f"D17 {leg} geometry differs from FK assumptions")
        legs[leg] = {"ids": ids, "hip": hip, "offset_y": offset[1],
                     "upper_m": -upper[2], "lower_m": -lower[2],
                     "foot_radius_m": radius,
                     "limits": (calf_limits, thigh_limits, hip_limits)}
    require(len({round(leg["foot_radius_m"], 10) for leg in legs.values()}) == 1,
            "D17 foot radii differ; centers need not be coplanar")
    return legs


def validate_evidence(snapshot, review, urdf_sha256):
    require(type(review) is dict and review.get("schema") == "singularitydog.offline-nominal-l-review.v1"
            and review.get("status") == "REVIEW_REQUIRED_NO_RUNTIME_PROMOTION"
            and review.get("d17_urdf_sha256") == urdf_sha256
            and review.get("sign_revalidated") is False
            and review.get("physical_angle_accuracy_verified") is False
            and review.get("output_allowed") is False
            and review.get("approved_for_runtime") is False
            and review.get("command_bytes_generated") is False,
            "Nominal-L review is not the unapproved D17 evidence")
    require(type(snapshot) is dict
            and snapshot.get("status") in (
                "READ_ONLY_12_BOX_AFTER_ID10_MANUAL", "READ_ONLY_BOX_AFTER_CAMERA_L",
                "READ_ONLY_BOX_AFTER_ALL_CAMERA_L")
            and snapshot.get("motor_output_allowed") is False,
            "Latest pose is not the read-only box snapshot")
    newer = snapshot["status"] != "READ_ONLY_12_BOX_AFTER_ID10_MANUAL"
    boot = snapshot.get("boot_id")
    require(type(boot) is str and boot and review.get("boot_id") == boot,
            "Snapshot and review boot IDs differ")
    review_rows = review.get("rows")
    require(type(review_rows) is list and len(review_rows) == 12,
            "Review needs twelve rows")
    by_id = {row.get("motor_id"): row for row in review_rows if type(row) is dict}
    require(set(by_id) == set(IDS), "Review IDs differ")
    rows = snapshot.get("rows")
    require(type(rows) is dict and set(rows) == {str(i) for i in IDS},
            "Snapshot needs exactly twelve axes")
    require(all(type(by_id[i].get("uid")) is str for i in IDS)
            and len({by_id[i]["uid"] for i in IDS}) == 12,
            "Review UIDs are not unique")
    raw, l_raw, signs, nominal, span_deg = {}, {}, {}, {}, {}
    for i in IDS:
        row = rows[str(i)]
        require(type(row) is dict and row.get("uid") == by_id[i].get("uid")
                and type(row.get("uid")) is str and len(row["uid"]) == 16,
                f"ID{i} UID differs")
        require((row.get("run_mode") == 0 if newer else row.get("disabled_mode") is True)
                and type(row.get("current_A")) in (int, float)
                and row["current_A"] == 0.0,
                f"ID{i} is not recorded disabled at zero current")
        median = finite(row.get("median_position_rad"), f"ID{i} median")
        if newer:
            span_deg[i] = finite(row.get("span_deg"), f"ID{i} span")
            require(0 <= span_deg[i] <= 0.1, f"ID{i} samples are too spread for this screen")
        else:
            samples = row.get("positions_rad")
            require(type(samples) is list and len(samples) == 3,
                    f"ID{i} needs three position samples")
            values = [finite(value, f"ID{i} sample") for value in samples]
            require(abs(median - statistics.median(values)) < 1e-8,
                    f"ID{i} median differs from samples")
            span_deg[i] = math.degrees(max(values) - min(values))
            require(span_deg[i] <= 0.1, f"ID{i} samples are too spread for this screen")
        raw[i] = median
        l_raw[i] = finite(by_id[i].get("raw_l_rad"), f"ID{i} nominal-L raw")
        sign = by_id[i].get("historical_sign")
        require(type(sign) is int and sign in (-1, 1), f"ID{i} sign invalid")
        signs[i] = sign
        q_l = finite(by_id[i].get("nominal_l_rad"), f"ID{i} nominal-L angle")
        require(abs(q_l - (-math.pi / 2 if i in (1, 4, 7, 10) else 0.)) < 1e-9,
                f"ID{i} nominal-L angle differs")
        nominal[i] = q_l
    require(signs[10] == 1, "Review ID10 historical sign differs")
    ab = review.get("id10_manual_b_minus_a_deg")
    require(type(ab) is dict and -110 < finite(ab.get("10"), "ID10 manual delta") < -70,
            "ID10 manual direction evidence missing")
    return boot, raw, l_raw, signs, nominal, span_deg


def validate_rl_camera_pair(rl_l, rl_match, snapshot):
    """Validate saved same-boot rear reads; the physical L shape remains operator evidence."""
    expected = {"l": (rl_l, "READ_ONLY_RL_CAMERA_L"),
                "match": (rl_match, "READ_ONLY_RL_CAMERA_MATCH")}
    values, spans = {}, {}
    for label, (data, status) in expected.items():
        require(type(data) is dict and data.get("status") == status
                and data.get("boot_id") == snapshot.get("boot_id")
                and data.get("motor_output_allowed") is False,
                f"RL {label} is not a same-boot read-only record")
        rows = data.get("rows")
        require(type(rows) is dict and set(rows) == {"10", "11", "12"},
                f"RL {label} needs IDs 10/11/12")
        values[label], spans[label] = {}, {}
        for i in (10, 11, 12):
            row = rows[str(i)]
            require(type(row) is dict and row.get("uid") == snapshot["rows"][str(i)]["uid"]
                    and type(row.get("run_mode")) is int and row["run_mode"] == 0
                    and type(row.get("current_A")) in (int, float)
                    and row["current_A"] == 0.0,
                    f"RL {label} ID{i} identity or disabled/current evidence differs")
            values[label][i] = finite(row.get("position_median_rad"), f"RL {label} ID{i}")
            spans[label][i] = finite(row.get("position_span_deg"), f"RL {label} ID{i} span")
            require(0 <= spans[label][i] <= 0.1, f"RL {label} ID{i} span too large")
    require(type(rl_match.get("created_ns")) is int
            and type(rl_l.get("created_ns")) is int
            and type(snapshot.get("created_ns")) is int
            and rl_match["created_ns"] < rl_l["created_ns"] < snapshot["created_ns"],
            "RL camera observations and box pose are out of order")
    return values, spans


def validate_leg_camera_l(data, leg, snapshot):
    require(leg in LEGS and type(data) is dict
            and data.get("status") == f"READ_ONLY_{leg}_CAMERA_L"
            and data.get("boot_id") == snapshot.get("boot_id")
            and data.get("motor_output_allowed") is False,
            f"{leg} camera L is not a same-boot read-only record")
    ids = LEGS[leg]
    rows = data.get("rows")
    require(type(rows) is dict and set(rows) == {str(i) for i in ids},
            f"{leg} camera L needs three axes")
    values, spans = {}, {}
    for i in ids:
        row = rows[str(i)]
        require(type(row) is dict and row.get("uid") == snapshot["rows"][str(i)]["uid"]
                and type(row.get("run_mode")) is int and row["run_mode"] == 0
                and type(row.get("current_A")) in (int, float)
                and row["current_A"] == 0.0,
                f"{leg} camera L ID{i} identity or disabled/current evidence differs")
        values[i] = finite(row.get("position_median_rad"), f"{leg} camera L ID{i}")
        spans[i] = finite(row.get("position_span_deg"), f"{leg} camera L ID{i} span")
        require(0 <= spans[i] <= 0.1, f"{leg} camera L ID{i} span too large")
    return values, spans


def foot_centers(angles, geometry):
    points = {}
    for leg, g in geometry.items():
        calf_id, thigh_id, hip_id = g["ids"]
        calf, thigh, hip = angles[calf_id], angles[thigh_id], angles[hip_id]
        down = -g["upper_m"] * math.cos(thigh) - g["lower_m"] * math.cos(thigh + calf)
        points[leg] = (
            g["hip"][0] - g["upper_m"] * math.sin(thigh)
            - g["lower_m"] * math.sin(thigh + calf),
            g["hip"][1] + g["offset_y"] * math.cos(hip) - down * math.sin(hip),
            g["offset_y"] * math.sin(hip) + down * math.cos(hip),
        )
    return points


def rl_plane_residual_mm(points):
    a, b, c, d = (points[leg] for leg in ("FR", "FL", "RR", "RL"))
    u = tuple(b[k] - a[k] for k in range(3))
    v = tuple(c[k] - a[k] for k in range(3))
    w = tuple(d[k] - a[k] for k in range(3))
    n = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    norm = math.sqrt(sum(value*value for value in n))
    require(norm > 1e-9, "Three reference feet do not define a plane")
    return abs(sum(n[k]*w[k] for k in range(3))) / norm * 1000.


def foot_height_spread_mm(points):
    heights = [point[2] for point in points.values()]
    return (max(heights) - min(heights)) * 1000.


def reference_plane_tilt_deg(points):
    a, b, c = (points[leg] for leg in ("FR", "FL", "RR"))
    u = tuple(b[k] - a[k] for k in range(3))
    v = tuple(c[k] - a[k] for k in range(3))
    n = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    norm = math.sqrt(sum(value*value for value in n))
    require(norm > 1e-9, "Three reference feet do not define a plane")
    return math.degrees(math.acos(min(1., abs(n[2]) / norm)))


def reference_plane_slopes(points):
    """z = a*x + b*y + c through FR/FL/RR centers, in the body frame."""
    a, b, c = (points[leg] for leg in ("FR", "FL", "RR"))
    u = tuple(b[k] - a[k] for k in range(3))
    v = tuple(c[k] - a[k] for k in range(3))
    n = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    require(abs(n[2]) > 1e-9, "Reference foot plane is vertical")
    return -n[0] / n[2], -n[1] / n[2]


def compare_uncalibrated_imu(imu, mount, foot_slopes):
    require(type(imu) is dict and imu.get("status") == "RECORDED_NOT_CALIBRATED"
            and imu.get("restore_status") == "restored" and imu.get("errors") == []
            and imu.get("plan", {}).get("can_opened") is False
            and imu.get("plan", {}).get("calibration_applied") is False
            and imu.get("plan", {}).get("orientation_verified_by_software") is False,
            "IMU summary is not an uncalibrated restored read-only capture")
    summary = imu.get("summary")
    require(type(summary) is dict and summary.get("samples", 0) >= 900
            and summary.get("calibration_applied") is False
            and summary.get("stillness_or_orientation_confirmed") is False,
            "IMU summary quality or verification differs")
    accel = summary.get("accel_mean_m_s2")
    require(type(accel) is list and len(accel) == 3, "IMU acceleration mean missing")
    accel = [finite(value, "IMU acceleration") for value in accel]
    require(type(mount) is dict and mount.get("status") == "IMU_MOUNT_CANDIDATE_ONLY"
            and mount.get("approved_for_runtime") is False
            and mount.get("raw_driver_axes_verified") is False
            and mount.get("R_body_from_sensor") == [[0, 1, 0], [1, 0, 0], [0, 0, -1]],
            "IMU mount is not the unverified fixed candidate")
    body = [accel[1], accel[0], -accel[2]]
    require(body[2] > 0, "IMU candidate points gravity downward in body frame")
    imu_slopes = (-body[0] / body[2], -body[1] / body[2])
    fk_angles = [math.degrees(math.atan(value)) for value in foot_slopes]
    imu_angles = [math.degrees(math.atan(value)) for value in imu_slopes]
    return {"body_accel_mean_m_s2_from_unverified_mount": body,
            "imu_inferred_plane_slopes": {"a_x": imu_slopes[0], "b_y": imu_slopes[1]},
            "fk_plane_slopes": {"a_x": foot_slopes[0], "b_y": foot_slopes[1]},
            "imu_inferred_pitch_roll_deg": imu_angles,
            "fk_pitch_roll_deg": fk_angles,
            "absolute_pitch_roll_difference_deg": [abs(fk_angles[i] - imu_angles[i])
                                                   for i in (0, 1)],
            "accel_gravity_norm_deviation_percent": finite(
                summary.get("gravity_norm_deviation_percent"), "gravity norm deviation"),
            "calibration_applied": False, "mount_verified": False,
            "time_synchronized_to_joint_capture": False,
            "interpretation": "Small slope differences are qualitative only; sensor bias and mount axes are unverified."}


def validate_visual_note(note):
    """Keep an unsaved-camera transcription visibly separate from measured angles."""
    require(type(note) is dict
            and note.get("schema") == "singularitydog.rl-sideview-note.v1"
            and note.get("status") == "QUALITATIVE_UNVERIFIED"
            and note.get("leg") == "RL" and note.get("motor_ids") == [10, 11, 12]
            and note.get("source_image_saved") is False
            and note.get("physical_joint_angle_measured") is False,
            "Side-view note must be an unmeasured RL observation")
    landmarks = note.get("approximate_image_landmarks_px")
    require(type(landmarks) is dict and set(landmarks) == {
        "proximal_center", "knee_center", "paw_center"},
        "Side-view landmarks missing")
    for label, point in landmarks.items():
        require(type(point) is list and len(point) == 2,
                f"Side-view {label} must have two coordinates")
        for value in point:
            finite(value, f"Side-view {label}")
    summary = note.get("fresh_rear_read_summary")
    require(type(summary) is dict and summary.get("source_record_checked_by_screen") is False
            and type(summary.get("raw_deg_by_id")) is dict
            and set(summary["raw_deg_by_id"]) == {"10", "11", "12"},
            "Side-view rear read must remain an unchecked summary")
    for mid, value in summary["raw_deg_by_id"].items():
        finite(value, f"Side-view ID{mid} reported raw angle")
    return {"classification": "qualitative_screen_projection_only",
            "leg_identified_by_operator": note.get("leg_identified_by_operator") is True,
            "approximate_image_landmarks_px": landmarks,
            "upper_and_lower_links_appear_downward": True,
            "physical_joint_angle_measured": False,
            "source_image_saved": False,
            "fresh_rear_read_summary": summary,
            "interpretation": "Both visible links slope downward in the image; perspective and uncertain centers prevent a physical knee angle or ID10 origin inference. A different nominal L origin could explain the apparent bend, but remains unmeasured."}


def build_report(snapshot, review, geometry, *, urdf_sha256=URDF_SHA256,
                 source_sha256=None, visual_note=None, rl_camera_l=None,
                 rl_camera_match=None, other_camera_l=None,
                 imu_summary=None, imu_mount=None):
    boot, raw, l_raw, signs, nominal, spans = validate_evidence(snapshot, review, urdf_sha256)
    require((rl_camera_l is None) == (rl_camera_match is None),
            "RL camera L and match records must be provided together")
    require((imu_summary is None) == (imu_mount is None),
            "IMU summary and mount candidate must be provided together")
    require(snapshot["status"] == "READ_ONLY_12_BOX_AFTER_ID10_MANUAL" or rl_camera_l is not None,
            "Latest box pose requires RL camera references")
    rl_pair = (validate_rl_camera_pair(rl_camera_l, rl_camera_match, snapshot)
               if rl_camera_l is not None else None)
    other_camera_l = other_camera_l or {}
    require(type(other_camera_l) is dict and set(other_camera_l) <= {"FR", "FL", "RR"},
            "Unexpected camera-L leg")
    if snapshot["status"] == "READ_ONLY_BOX_AFTER_ALL_CAMERA_L":
        require(set(other_camera_l) == {"FR", "FL", "RR"},
                "Final box pose requires all four camera-L references")
    other_refs = {leg: validate_leg_camera_l(data, leg, snapshot)
                  for leg, data in other_camera_l.items()}
    if snapshot["status"] == "READ_ONLY_BOX_AFTER_ALL_CAMERA_L":
        require(all(type(data.get("created_ns")) is int
                    and data["created_ns"] < snapshot["created_ns"]
                    for data in other_camera_l.values()),
                "Camera-L references must precede final box pose")
    available_l = ({**rl_pair[0]["l"]} if rl_pair is not None else {})
    for values, _ in other_refs.values():
        available_l.update(values)
    direct = {i: math.degrees(raw[i] - l_raw[i]) for i in IDS}
    hypotheses = {}
    specs = [("historical_id10_plus", signs[10], l_raw),
             ("id10_flipped_for_review", -signs[10], l_raw)]
    if rl_pair is not None:
        true_l = rl_pair[0]["l"]
        specs.extend((
            ("id10_camera_l_reversed_for_review", -signs[10], {**l_raw, 10: true_l[10]}),
            ("rl_camera_l_rebased_for_review", -signs[10],
             {**l_raw, **true_l}),
        ))
    if other_refs:
        specs.append(("available_camera_l_rebased_for_review", -signs[10],
                      {**l_raw, **available_l}))
    selected_name = specs[-1][0]
    for name, id10_sign, l_reference in specs:
        candidate_signs = {**signs, 10: id10_sign}
        model = {i: nominal[i] + candidate_signs[i] * (raw[i] - l_reference[i]) for i in IDS}
        ranges = {}
        for leg, g in geometry.items():
            for i, (lower, upper) in zip(g["ids"], g["limits"]):
                q = model[i]
                ranges[str(i)] = {"model_deg": math.degrees(q),
                                  "limit_deg": [math.degrees(lower), math.degrees(upper)],
                                  "in_range": lower <= q <= upper,
                                  "violation_deg": math.degrees(max(lower - q, q - upper, 0.))}
        feet = foot_centers(model, geometry)
        residual = rl_plane_residual_mm(feet)
        hypotheses[name] = {"id10_sign_hypothesis": id10_sign,
                            "model_range_by_id": {str(i): ranges[str(i)] for i in IDS},
                            "out_of_range_ids": [i for i in IDS if not ranges[str(i)]["in_range"]],
                            "rl_to_other_three_foot_center_plane_mm": residual,
                            "foot_center_z_mm_by_leg": {leg: feet[leg][2]*1000 for leg in LEGS},
                            "foot_center_z_spread_mm": foot_height_spread_mm(feet),
                            "fr_fl_rr_plane_tilt_from_body_horizontal_deg":
                                reference_plane_tilt_deg(feet),
                            "fr_fl_rr_plane_slopes": dict(zip(
                                ("a_x", "b_y"), reference_plane_slopes(feet))),
                            "verified": False}
    selected_imu = (compare_uncalibrated_imu(
        imu_summary, imu_mount,
        tuple(hypotheses[selected_name]["fr_fl_rr_plane_slopes"][key]
              for key in ("a_x", "b_y"))) if imu_summary is not None else None)
    reasons = []
    branch_ids = [i for i in (3, 9)
                  if abs(math.degrees(raw[i] - available_l.get(i, l_raw[i]))) >= 300]
    if branch_ids:
        reasons.append({"code": "UNRESOLVED_360_DEG_BRANCH_ID3_ID9",
                        "ids": branch_ids,
                        "direct_delta_deg": {
                            str(i): math.degrees(raw[i] - available_l.get(i, l_raw[i]))
                            for i in branch_ids}})
    if rl_pair is None:
        reasons.append({"code": "ID10_SIGN_AND_L_ORIGIN_UNVERIFIED",
                        "detail": "Manual relative rotation conflicts with the old sign; nominal L angles were not measured."})
    else:
        reasons.append({"code": "CAMERA_L_PHYSICAL_ANGLE_UNVERIFIED",
                        "detail": "Operator-identified RL L and the negative raw delta support ID10 sign -1, but exact physical L angles were not measured."})
        reasons.append({"code": "OTHER_AXIS_SIGNS_NOT_REVALIDATED",
                        "detail": "The remaining eleven joint signs still come from an unapproved historical review."})
        reasons.append({"code": "SEQUENTIAL_L_REFERENCES_NOT_SIMULTANEOUS",
                        "detail": "Per-leg L captures were taken at separate moments, without a measured common torso attitude."})
    if all(4 in item["out_of_range_ids"] for item in hypotheses.values()):
        reasons.append({"code": "MODEL_RANGE_OVERFLOW_ID4",
                        "detail": "The nominal-L hypothesis puts ID4 outside its D17 calf range."})
    if visual_note is not None:
        reasons.append({"code": "SIDE_VIEW_IS_NOT_ANGLE_METROLOGY",
                        "detail": "Unsaved side-view image and approximate landmarks do not settle the knee angle or ID10 origin."})
    if hypotheses[selected_name]["rl_to_other_three_foot_center_plane_mm"] > FEW_MM:
        reasons.append({"code": "FOUR_FOOT_CONTACT_GEOMETRY_RESIDUAL",
                        "comparison_scale_mm": FEW_MM,
                        "detail": "A modeled residual exceeds an illustrative few-mm comparison scale; floor contact geometry is not resolved."})
    if other_refs and hypotheses["available_camera_l_rebased_for_review"]["foot_center_z_spread_mm"] > 20:
        reasons.append({"code": "MODELED_FOOT_HEIGHT_SPREAD_NEEDS_BODY_ATTITUDE",
                        "detail": "Small plane residual alone can hide a strongly tilted plane; torso attitude was not measured."})
    reasons.append({"code": "LIFT_TRAJECTORY_AND_LOAD_SUPPORT_UNREVIEWED",
                    "detail": "These saved positions do not establish a finite all-axis lift path or load-bearing behavior."})
    if selected_imu is not None:
        reasons.append({"code": "IMU_CALIBRATION_AND_MOUNT_UNVERIFIED",
                        "detail": "Stationary gravity direction is qualitatively consistent with FK, but magnitude and mount are uncalibrated."})
    return {"schema": "singularitydog.box-lift-offline-screen.v1",
            "status": "NOT_READY", "boot_id": boot,
            "input_sha256": source_sha256,
            "snapshot": {"matching_uid_count": 12, "disabled_zero_current_count": 12,
                         "maximum_three_sample_span_deg": max(spans.values())},
            "rl_camera_pair": ({
                "matching_uid_count": 3, "disabled_zero_current_count_each": 3,
                "maximum_recorded_span_deg": max(value for m in rl_pair[1].values()
                                                 for value in m.values()),
                "l_to_near_straight_direct_raw_delta_deg_by_id": {
                    str(i): math.degrees(rl_pair[0]["match"][i] - rl_pair[0]["l"][i])
                    for i in (10, 11, 12)},
                "id10_near_straight_reverse_sign_model_deg": math.degrees(
                    nominal[10] - (rl_pair[0]["match"][10] - rl_pair[0]["l"][10])),
                "physical_L_angle_accuracy_verified": False,
            } if rl_pair is not None else None),
            "available_operator_identified_l_legs": sorted(({"RL"} if rl_pair else set())
                                                           | set(other_refs)),
            "simultaneous_fullbody_l_pose_verified": False,
            "other_camera_l": {leg: {"matching_uid_count": 3,
                                      "disabled_zero_current_count": 3,
                                      "maximum_recorded_span_deg": max(spans_by_id.values()),
                                      "physical_L_angle_accuracy_verified": False}
                               for leg, (_, spans_by_id) in other_refs.items()},
            "direct_raw_minus_l_deg_by_id": {str(i): direct[i] for i in IDS},
            "selected_model_hypothesis": selected_name,
            "imu_comparison": selected_imu,
            "hypotheses": hypotheses, "reasons": reasons,
            "visual_note": validate_visual_note(visual_note) if visual_note is not None else None,
            "interpretation": "Equal-radius foot-center coplanarity is necessary, not proof of contact or load support. Angles are unwrapped hypotheses only.",
            "angle_wrapping_applied": False, "turn_correction_applied": False,
            "physical_contact_verified_by_this_screen": False,
            "approved_for_runtime": False, "motor_commands_generated": False}


def private_new_path(path):
    requested = Path(path).expanduser()
    require(not requested.is_symlink(), "Report path cannot be a symlink")
    path = requested.resolve()
    require(path.parent.is_dir() and not path.exists(),
            "Report needs a fresh path in an existing directory")
    require(not any((parent / ".git").exists() for parent in (path.parent, *path.parents)),
            "Private report must be outside Git")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("snapshot", "nominal-l-review", "urdf", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--visual-note", type=Path,
                        help="Optional private, qualitative side-view transcription")
    parser.add_argument("--rl-camera-l", type=Path,
                        help="Saved read-only rear pose at operator-identified RL L")
    parser.add_argument("--rl-camera-match", type=Path,
                        help="Saved read-only rear pose at prior camera alignment")
    parser.add_argument("--fr-camera-l", type=Path)
    parser.add_argument("--fl-camera-l", type=Path)
    parser.add_argument("--rr-camera-l", type=Path)
    parser.add_argument("--imu-summary", type=Path)
    parser.add_argument("--imu-mount-candidate", type=Path)
    args = parser.parse_args(argv)
    try:
        require((args.rl_camera_l is None) == (args.rl_camera_match is None),
                "Both RL camera records are required together")
        require((args.imu_summary is None) == (args.imu_mount_candidate is None),
                "IMU summary and mount candidate are required together")
        snapshot_bytes = read_pinned(args.snapshot,
                                     (SNAPSHOT_SHA256, LATEST_SNAPSHOT_SHA256,
                                      FINAL_SNAPSHOT_SHA256), "snapshot")
        review_bytes = read_pinned(args.nominal_l_review, REVIEW_SHA256, "nominal-L review")
        urdf = read_pinned(args.urdf, URDF_SHA256, "D17 URDF")
        visual_bytes = args.visual_note.read_bytes() if args.visual_note else None
        visual_note = json.loads(visual_bytes) if visual_bytes is not None else None
        rl_l_bytes = (read_pinned(args.rl_camera_l, RL_CAMERA_L_SHA256, "RL camera L")
                      if args.rl_camera_l else None)
        rl_match_bytes = (read_pinned(args.rl_camera_match, RL_CAMERA_MATCH_SHA256,
                                      "RL camera match") if args.rl_camera_match else None)
        other_camera_bytes = {}
        for leg, path, digest in (("FR", args.fr_camera_l, FR_CAMERA_L_SHA256),
                                  ("FL", args.fl_camera_l, FL_CAMERA_L_SHA256),
                                  ("RR", args.rr_camera_l, RR_CAMERA_L_SHA256)):
            if path is not None:
                other_camera_bytes[leg] = read_pinned(path, digest, f"{leg} camera L")
        imu_bytes = (read_pinned(args.imu_summary, IMU_SUMMARY_SHA256, "IMU summary")
                     if args.imu_summary else None)
        mount_bytes = (read_pinned(args.imu_mount_candidate, IMU_MOUNT_SHA256,
                                   "IMU mount candidate") if args.imu_mount_candidate else None)
        sources = {"snapshot": hashlib.sha256(snapshot_bytes).hexdigest(),
                   "nominal_l_review": hashlib.sha256(review_bytes).hexdigest(),
                   "d17_urdf": hashlib.sha256(urdf).hexdigest()}
        if visual_bytes is not None:
            sources["visual_note"] = hashlib.sha256(visual_bytes).hexdigest()
        if rl_l_bytes is not None:
            sources["rl_camera_l"] = hashlib.sha256(rl_l_bytes).hexdigest()
            sources["rl_camera_match"] = hashlib.sha256(rl_match_bytes).hexdigest()
        for leg, content in other_camera_bytes.items():
            sources[f"{leg.lower()}_camera_l"] = hashlib.sha256(content).hexdigest()
        if imu_bytes is not None:
            sources["imu_summary"] = hashlib.sha256(imu_bytes).hexdigest()
            sources["imu_mount_candidate"] = hashlib.sha256(mount_bytes).hexdigest()
        report = build_report(json.loads(snapshot_bytes), json.loads(review_bytes),
                              parse_d17(urdf), source_sha256=sources,
                              visual_note=visual_note,
                              rl_camera_l=json.loads(rl_l_bytes) if rl_l_bytes else None,
                              rl_camera_match=json.loads(rl_match_bytes) if rl_match_bytes else None,
                              other_camera_l={leg: json.loads(content)
                                              for leg, content in other_camera_bytes.items()},
                              imu_summary=json.loads(imu_bytes) if imu_bytes else None,
                              imu_mount=json.loads(mount_bytes) if mount_bytes else None)
        output = private_new_path(args.output)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        with os.fdopen(os.open(output, flags, 0o600), "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print("NOT_READY: file-only D17 box-lift screen")
        for name, hypothesis in report["hypotheses"].items():
            print(f"{name}: out-of-range IDs {hypothesis['out_of_range_ids']}; "
                  f"RL plane residual {hypothesis['rl_to_other_three_foot_center_plane_mm']:.1f} mm")
        print("Reasons: " + ", ".join(reason["code"] for reason in report["reasons"]))
        print(f"Private report: {output}")
    except (OSError, ValueError, ET.ParseError, json.JSONDecodeError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
