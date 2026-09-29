"""File-only, nominal 2–5 mm box-supported body-rise candidate.

There are deliberately no CAN, serial, motor-protocol, or live-runtime imports.
The result is a geometry calculation, not an executable motor plan.  A box must
remain directly beneath the torso; contact force and swept clearance still
require separate physical evidence.  No angle wrapping or branch repair occurs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

from tools.screen_box_lift_offline import (LEGS, URDF_SHA256, foot_centers,
                                           parse_d17, private_new_path)


IDS = tuple(range(1, 13))
HIP_IDS = (3, 6, 9, 12)
MODEL_MARGIN_RAD = math.radians(2.)
START_ENVELOPE_RAD = math.radians(.5)
MAX_TOTAL_RAW_EXCURSION_RAD = math.radians(12.)
MAX_STEP_RAW_RAD = math.radians(.5)
MIN_CLEARANCE_MARGIN_MM = 5.
MAX_CLEARANCE_UNCERTAINTY_MM = 2.
MAX_FOOT_PLANE_RESIDUAL_MM = 2.
RAW_PROTOCOL_MAX_RAD = 12.57
TYPE1_HEADROOM_RAD = math.radians(5.)
SAMPLE_PERIOD_S = .08
SAMPLE_COUNT = 101  # Four-second rise and four-second return to the box pose.


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _finite(value, label):
    _require(type(value) in (int, float) and math.isfinite(value),
             f"{label} must be finite")
    return float(value)


def _raw_map(value, label):
    _require(type(value) is dict and set(value) == {str(i) for i in IDS},
             f"{label} needs exactly IDs 1..12")
    return {i: _finite(value[str(i)], f"{label} ID{i}") for i in IDS}


def _uids(value, label):
    _require(type(value) is dict and set(value) == {str(i) for i in IDS},
             f"{label} needs twelve UIDs")
    _require(all(type(uid) is str and len(uid) == 16
                 and all(c in "0123456789abcdefABCDEF" for c in uid)
                 for uid in value.values()), f"{label} invalid UID")
    _require(len(set(uid.lower() for uid in value.values())) == 12,
             f"{label} UIDs are not unique")
    return {i: value[str(i)].lower() for i in IDS}


def _sha(value, label):
    _require(type(value) is str and len(value) == 64
             and all(c in "0123456789abcdef" for c in value),
             f"{label} needs a SHA-256 digest")
    return value


def _snapshot(value, label, boot, uids):
    _require(type(value) is dict and type(value.get("status")) is str
             and value["status"].startswith("READ_ONLY_BOX_")
             and value.get("motor_output_allowed") is False
             and value.get("boot_id") == boot
             and type(value.get("created_ns")) is int,
             f"{label} is not a same-boot read-only box snapshot")
    rows = value.get("rows")
    _require(type(rows) is dict and set(rows) == {str(i) for i in IDS},
             f"{label} needs twelve rows")
    raw = {}
    for i in IDS:
        row = rows[str(i)]
        _require(type(row) is dict and type(row.get("uid")) is str
                 and row["uid"].lower() == uids[i]
                 and type(row.get("run_mode")) is int and row["run_mode"] == 0
                 and type(row.get("current_A")) in (int, float)
                 and row["current_A"] == 0.,
                 f"{label} ID{i} UID, disabled mode or current differs")
        raw[i] = _finite(row.get("median_position_rad"), f"{label} ID{i} raw")
        span = _finite(row.get("span_deg"), f"{label} ID{i} span")
        _require(0 <= span <= .1, f"{label} ID{i} is not stationary")
    return raw


def _camera_references(records, hashes, boot, uids, floor_created_ns):
    _require(type(records) is dict and set(records) == set(LEGS)
             and type(hashes) is dict and set(hashes) == set(LEGS),
             "All four camera-confirmed L records are required")
    raw_l = {}
    for leg, ids in LEGS.items():
        data = records[leg]
        _sha(hashes[leg], f"{leg} camera L hash")
        _require(type(data) is dict and data.get("status") == f"READ_ONLY_{leg}_CAMERA_L"
                 and data.get("motor_output_allowed") is False
                 and data.get("boot_id") == boot
                 and type(data.get("created_ns")) is int
                 and data["created_ns"] < floor_created_ns,
                 f"{leg} camera L does not precede same-boot floor snapshot")
        rows = data.get("rows")
        _require(type(rows) is dict and set(rows) == {str(i) for i in ids},
                 f"{leg} camera L needs its three axes")
        for i in ids:
            row = rows[str(i)]
            _require(type(row) is dict and type(row.get("uid")) is str
                     and row["uid"].lower() == uids[i]
                     and type(row.get("run_mode")) is int and row["run_mode"] == 0
                     and type(row.get("current_A")) in (int, float)
                     and row["current_A"] == 0.,
                     f"{leg} camera L ID{i} UID, disabled mode or current differs")
            span = _finite(row.get("position_span_deg"), f"{leg} camera L ID{i} span")
            _require(0 <= span <= .1, f"{leg} camera L ID{i} is not stationary")
            raw_l[i] = _finite(row.get("position_median_rad"), f"{leg} camera L ID{i}")
    return raw_l


def _calibration(value, boot, uids, hashes, *, fixed_hip):
    _require(type(value) is dict and value.get("schema") ==
             "singularitydog.box-rise-angle-review.v1"
             and value.get("boot_id") == boot
             and value.get("camera_l_sha256_by_leg") == hashes,
             "Angle review is not bound to this boot and all four camera L records")
    _require(_uids(value.get("motor_uids"), "angle review") == uids,
             "Angle review UIDs differ")
    rows = value.get("axes")
    _require(type(rows) is dict and set(rows) == {str(i) for i in IDS},
             "Angle review needs all twelve axes")
    signs, q_l = {}, {}
    for i in IDS:
        if fixed_hip and i in HIP_IDS:
            signs[i] = None
            q_l[i] = None
            continue
        row = rows[str(i)]
        _require(type(row) is dict and type(row.get("sign")) is int
                 and row["sign"] in (-1, 1)
                 and row.get("sign_physically_revalidated") is True
                 and row.get("l_model_angle_measured") is True,
                 f"ID{i} lacks verified sign or L-angle zero")
        signs[i] = row["sign"]
        q_l[i] = _finite(row.get("l_model_angle_rad"), f"ID{i} L-angle zero")
    return signs, q_l


def _physical_review(value, boot, uids, floor_hash, fresh_raw, rise_mm, *, fixed_hip):
    _require(type(value) is dict and value.get("schema") ==
             "singularitydog.box-rise-physical-review.v1"
             and value.get("boot_id") == boot
             and value.get("floor_snapshot_sha256") == floor_hash,
             "Physical review is not bound to the floor snapshot and boot")
    _require(_uids(value.get("motor_uids"), "physical review") == uids,
             "Physical review UIDs differ")
    for name in ("all_four_paws_on_floor_confirmed", "box_directly_under_torso_confirmed",
                 "torso_supported_by_box_confirmed", "full_nonfoot_sweep_measured"):
        _require(value.get(name) is True, f"Physical review missing {name}")
    for name in ("floor_contact_evidence_ref", "clearance_evidence_ref",
                 "raw_corridor_evidence_ref"):
        _require(type(value.get(name)) is str and value[name].strip(),
                 f"Physical review missing {name}")
    reference = _raw_map(value.get("reviewed_floor_start_raw_rad_by_id"),
                         "reviewed floor start")
    for i in IDS:
        _require(abs(fresh_raw[i] - reference[i]) <= START_ENVELOPE_RAD,
                 f"ID{i} is outside the supported start envelope; no wrapping")
    corridor = value.get("measured_raw_corridor_by_id")
    _require(type(corridor) is dict and set(corridor) == {str(i) for i in IDS},
             "Measured raw corridor needs all twelve axes")
    bounds = {}
    for i in IDS:
        row = corridor[str(i)]
        _require(type(row) is dict and set(row) == {"min_rad", "max_rad"},
                 f"ID{i} raw corridor needs min/max")
        low = _finite(row["min_rad"], f"ID{i} raw min")
        high = _finite(row["max_rad"], f"ID{i} raw max")
        _require(low < high and low <= fresh_raw[i] <= high,
                 f"ID{i} start outside measured raw corridor")
        bounds[i] = (low, high)
    clearances = value.get("minimum_measured_nonfoot_clearance_mm_by_leg")
    _require(type(clearances) is dict and set(clearances) == set(LEGS),
             "Measured nonfoot clearances need all four legs")
    uncertainty = _finite(value.get("clearance_measurement_uncertainty_mm"),
                          "clearance uncertainty")
    _require(0 <= uncertainty <= MAX_CLEARANCE_UNCERTAINTY_MM,
             "Clearance uncertainty exceeds two millimeters")
    minimum = {leg: _finite(clearances[leg], f"{leg} clearance") for leg in LEGS}
    _require(all(value >= rise_mm + uncertainty + MIN_CLEARANCE_MARGIN_MM
                 for value in minimum.values()),
             "Measured clearance is insufficient even before swept displacement")
    hip_bound_deg = None
    if fixed_hip:
        hip_bound_deg = _finite(value.get("max_abs_hip_angle_deg"),
                                "fixed-hip absolute angle bound")
        _require(0 <= hip_bound_deg <= 20.,
                 "Fixed-hip angle bound exceeds twenty degrees")
        _require(rise_mm*math.tan(math.radians(hip_bound_deg)) <= .75,
                 "Fixed-hip lateral paw drift bound exceeds 0.75 mm")
    return bounds, minimum, uncertainty, hip_bound_deg


def _point(q, g):
    calf, thigh, hip = q
    down = -g["upper_m"]*math.cos(thigh) - g["lower_m"]*math.cos(thigh + calf)
    return (g["hip"][0] - g["upper_m"]*math.sin(thigh)
            - g["lower_m"]*math.sin(thigh + calf),
            g["hip"][1] + g["offset_y"]*math.cos(hip) - down*math.sin(hip),
            g["offset_y"]*math.sin(hip) + down*math.cos(hip))


def _key_points(q, g):
    calf, thigh, hip = q
    h = (g["hip"][0], g["hip"][1], 0.)
    t = (h[0], h[1] + g["offset_y"]*math.cos(hip),
         g["offset_y"]*math.sin(hip))
    k = (t[0] - g["upper_m"]*math.sin(thigh),
         t[1] + g["upper_m"]*math.cos(thigh)*math.sin(hip),
         t[2] - g["upper_m"]*math.cos(thigh)*math.cos(hip))
    return (h, t, k, _point(q, g))


def _planar_point(q, g):
    calf, thigh = q
    return (-g["upper_m"]*math.sin(thigh)
            - g["lower_m"]*math.sin(thigh+calf),
            -g["upper_m"]*math.cos(thigh)
            - g["lower_m"]*math.cos(thigh+calf))


def _planar_key_points(q, g):
    calf, thigh = q
    knee = (-g["upper_m"]*math.sin(thigh),
            -g["upper_m"]*math.cos(thigh))
    return ((0., 0.), knee, _planar_point(q, g))


def _ik2_near(previous, target, g):
    q = tuple(previous)
    for _ in range(20):
        actual = _planar_point(q, g)
        ex, ez = target[0]-actual[0], target[1]-actual[1]
        if math.hypot(ex, ez) <= 1e-5:
            return q
        calf, thigh = q
        upper, lower = g["upper_m"], g["lower_m"]
        c = math.cos(thigh+calf)
        s = math.sin(thigh+calf)
        a, b = -lower*c, -upper*math.cos(thigh)-lower*c
        c2, d = lower*s, upper*math.sin(thigh)+lower*s
        determinant = a*d-b*c2
        _require(abs(determinant) > 1e-8,
                 "Fixed-hip inverse kinematics is near a singular pose")
        dq = ((d*ex-b*ez)/determinant,
              (-c2*ex+a*ez)/determinant)
        scale = min(1., .2/max(abs(v) for v in dq))
        previous_error = math.hypot(ex, ez)
        accepted = False
        for _ in range(10):
            trial = (q[0]+scale*dq[0], q[1]+scale*dq[1])
            point = _planar_point(trial, g)
            if math.dist(point, target) < previous_error:
                q, accepted = trial, True
                break
            scale *= .5
        _require(accepted, "Fixed-hip inverse kinematics did not converge")
    raise ValueError("Fixed-hip inverse kinematics did not converge")


def _solve3(a, b):
    rows = [list(a[i]) + [b[i]] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        _require(abs(rows[pivot][col]) > 1e-9,
                 "Body-rise inverse kinematics is near a singular pose")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        factor = rows[col][col]
        for j in range(col, 4):
            rows[col][j] /= factor
        for row in range(3):
            if row == col:
                continue
            factor = rows[row][col]
            for j in range(col, 4):
                rows[row][j] -= factor*rows[col][j]
    return tuple(rows[i][3] for i in range(3))


def _ik_near(previous, target, g, *, tolerance_m=1e-5):
    _require(type(tolerance_m) in (int, float) and math.isfinite(tolerance_m)
             and 0 < tolerance_m <= 1e-5, 'IK tolerance must be positive and at most 0.01mm')
    q = tuple(previous)
    for _ in range(20):
        actual = _point(q, g)
        error = tuple(target[k] - actual[k] for k in range(3))
        if math.sqrt(sum(v*v for v in error)) <= tolerance_m:
            return q
        eps = 1e-6
        derivatives = []
        for j in range(3):
            plus = list(q)
            minus = list(q)
            plus[j] += eps
            minus[j] -= eps
            p_plus, p_minus = _point(plus, g), _point(minus, g)
            derivatives.append(tuple((p_plus[k] - p_minus[k])/(2*eps) for k in range(3)))
        correction = _solve3([[derivatives[j][k] for j in range(3)]
                              for k in range(3)], error)
        scale = min(1., .2/max(abs(c) for c in correction))
        old_norm = math.sqrt(sum(v*v for v in error))
        accepted = False
        for _ in range(10):
            trial = tuple(q[j] + scale*correction[j] for j in range(3))
            residual = math.sqrt(sum((target[k] - _point(trial, g)[k])**2
                                     for k in range(3)))
            if residual < old_norm:
                q, accepted = trial, True
                break
            scale *= .5
        _require(accepted, "Body-rise inverse kinematics did not converge")
    raise ValueError("Body-rise inverse kinematics did not converge")


def _quintic(u):
    return u*u*u*(10 + u*(-15 + 6*u))


def _plane_residual_mm(points):
    a, b, c, d = (points[leg] for leg in ("FR", "FL", "RR", "RL"))
    u, v, w = ([b[k]-a[k] for k in range(3)],
               [c[k]-a[k] for k in range(3)],
               [d[k]-a[k] for k in range(3)])
    normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],
              u[0]*v[1]-u[1]*v[0])
    magnitude = math.sqrt(sum(x*x for x in normal))
    _require(magnitude > 1e-9, "First three feet do not define a plane")
    return abs(sum(normal[k]*w[k] for k in range(3)))/magnitude*1000.


def build_candidate(*, floor_snapshot, floor_snapshot_sha256, camera_l_by_leg,
                    camera_l_sha256_by_leg, angle_review, physical_review, geometry,
                    current_boot_id, current_motor_uids, rise_mm, fixed_hip=False):
    """Return a nominal finite path only after all static evidence is complete.

    This function never checks the live bus, so even a returned candidate is
    unsuitable for direct execution.  Geometry is the pinned D17 URDF parsed
    by ``screen_box_lift_offline.parse_d17``.
    """
    rise_mm = _finite(rise_mm, "requested body rise")
    _require(2. <= rise_mm <= 5., "Body rise must be 2–5 millimeters")
    _require(type(fixed_hip) is bool, "Fixed-hip choice must be explicit")
    _sha(floor_snapshot_sha256, "floor snapshot hash")
    _require(type(current_boot_id) is str and current_boot_id,
             "Current boot ID is required")
    uids = _uids(current_motor_uids, "current identities")
    raw_start = _snapshot(floor_snapshot, "fresh floor", current_boot_id, uids)
    for i in IDS:
        _require(-RAW_PROTOCOL_MAX_RAD + TYPE1_HEADROOM_RAD <= raw_start[i]
                 <= RAW_PROTOCOL_MAX_RAD - TYPE1_HEADROOM_RAD,
                 f"ID{i} start exceeds Type1 position range with headroom")
    raw_l = _camera_references(camera_l_by_leg, camera_l_sha256_by_leg,
                               current_boot_id, uids, floor_snapshot["created_ns"])
    signs, q_l = _calibration(angle_review, current_boot_id, uids,
                              camera_l_sha256_by_leg, fixed_hip=fixed_hip)
    bounds, minimum_clearance, uncertainty, hip_bound_deg = _physical_review(
        physical_review, current_boot_id, uids, floor_snapshot_sha256,
        raw_start, rise_mm, fixed_hip=fixed_hip)
    _require(type(geometry) is dict and set(geometry) == set(LEGS),
             "Pinned D17 geometry for four legs is required")

    moving_ids = tuple(i for i in IDS if not fixed_hip or i not in HIP_IDS)
    model_start = {i: q_l[i] + signs[i]*(raw_start[i] - raw_l[i]) for i in moving_ids}
    if fixed_hip:
        initial_q = {leg: tuple(model_start[i] for i in ids[:2])
                     for leg, ids in LEGS.items()}
        initial_feet = {leg: _planar_point(initial_q[leg], geometry[leg])
                        for leg in LEGS}
        residual_mm = None  # Hip axes are fixed; their model angles are not inferred.
        initial_key_points = {leg: _planar_key_points(initial_q[leg], geometry[leg])
                              for leg in LEGS}
    else:
        initial_feet = foot_centers(model_start, geometry)
        residual_mm = _plane_residual_mm(initial_feet)
        _require(residual_mm <= MAX_FOOT_PLANE_RESIDUAL_MM,
                 "Supported initial pose contradicts four-foot floor geometry")
        initial_q = {leg: tuple(model_start[i] for i in ids) for leg, ids in LEGS.items()}
        initial_key_points = {leg: _key_points(initial_q[leg], geometry[leg]) for leg in LEGS}
    previous_q = dict(initial_q)
    max_swept_mm = {leg: 0. for leg in LEGS}
    previous_raw = dict(raw_start)
    samples = []
    for tick in range(SAMPLE_COUNT):
        half = (SAMPLE_COUNT-1)//2
        progress = (tick if tick <= half else SAMPLE_COUNT-1-tick)/half
        fraction = _quintic(progress)
        q_by_leg = {}
        raw = {}
        for leg, ids in LEGS.items():
            foot = initial_feet[leg]
            desired = ((foot[0], foot[1] - rise_mm/1000.*fraction) if fixed_hip
                       else (foot[0], foot[1], foot[2] - rise_mm/1000.*fraction))
            q = (initial_q[leg] if tick in (0, SAMPLE_COUNT-1)
                 else (_ik2_near(previous_q[leg], desired, geometry[leg]) if fixed_hip
                       else _ik_near(previous_q[leg], desired, geometry[leg])))
            for index, i in enumerate(ids[:2] if fixed_hip else ids):
                lower, upper = geometry[leg]["limits"][index]
                _require(lower + MODEL_MARGIN_RAD <= q[index] <= upper - MODEL_MARGIN_RAD,
                         f"ID{i} violates D17 joint limit or two-degree margin")
                target = raw_l[i] + signs[i]*(q[index] - q_l[i])
                _require(-RAW_PROTOCOL_MAX_RAD + TYPE1_HEADROOM_RAD <= target
                         <= RAW_PROTOCOL_MAX_RAD - TYPE1_HEADROOM_RAD,
                         f"ID{i} target exceeds Type1 position range with headroom")
                low, high = bounds[i]
                _require(low <= target <= high,
                         f"ID{i} leaves measured raw corridor")
                _require(abs(target - raw_start[i]) <= MAX_TOTAL_RAW_EXCURSION_RAD,
                         f"ID{i} exceeds finite total raw excursion")
                _require(abs(target - previous_raw[i]) <= MAX_STEP_RAW_RAD + 1e-12,
                         f"ID{i} exceeds finite per-sample raw step")
                raw[i] = target
            if fixed_hip:
                i = ids[2]
                raw[i] = raw_start[i]
                low, high = bounds[i]
                _require(low <= raw[i] <= high, f"ID{i} fixed hip outside raw corridor")
            points = (_planar_key_points(q, geometry[leg]) if fixed_hip
                      else _key_points(q, geometry[leg]))
            for old, new in zip(initial_key_points[leg], points):
                movement_mm = 1000*math.dist(old, new)
                max_swept_mm[leg] = max(max_swept_mm[leg], movement_mm)
            q_by_leg[leg] = q
        samples.append({"tick": tick, "elapsed_s": tick*SAMPLE_PERIOD_S,
                        "body_rise_mm": rise_mm*fraction,
                        "raw_rad_by_id": {str(i): raw[i] for i in IDS}})
        previous_q = q_by_leg
        previous_raw = raw
    for leg in LEGS:
        _require(minimum_clearance[leg] >= max_swept_mm[leg] + uncertainty
                 + MIN_CLEARANCE_MARGIN_MM,
                 f"{leg} measured clearance is insufficient for swept path")
    return {
        "schema": "singularitydog.offline-box-rise-candidate.v1",
        "status": "OFFLINE_GEOMETRY_CANDIDATE_ONLY",
        "boot_id": current_boot_id,
        "motor_uids": {str(i): uids[i] for i in IDS},
        "floor_snapshot_sha256": floor_snapshot_sha256,
        "camera_l_sha256_by_leg": dict(camera_l_sha256_by_leg),
        "requested_body_rise_mm": rise_mm,
        "sample_period_s": SAMPLE_PERIOD_S,
        "sample_count": SAMPLE_COUNT,
        "duration_s": (SAMPLE_COUNT-1)*SAMPLE_PERIOD_S,
        "initial_foot_plane_residual_mm": residual_mm,
        "fixed_hip": fixed_hip,
        "hip_raw_targets_equal_fresh_start": fixed_hip,
        "hip_lateral_paw_drift_bound_mm": (rise_mm*math.tan(math.radians(hip_bound_deg))
                                            if fixed_hip else None),
        "max_nonfoot_keypoint_displacement_mm_by_leg": max_swept_mm,
        "samples": samples,
        "motor_output_allowed": False,
        "live_runner_available": False,
        "load_transfer_verified": False,
        "self_supported_stance_verified": False,
        "swept_clearance_verified_by_calculation": False,
        "angle_wrapping_applied": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("floor-snapshot", "fr-camera-l", "fl-camera-l", "rr-camera-l",
                 "rl-camera-l", "angle-review", "physical-review", "urdf",
                 "current-uids", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--current-boot-id", required=True)
    parser.add_argument("--rise-mm", type=float, required=True)
    parser.add_argument("--fixed-hip", action="store_true")
    args = parser.parse_args(argv)
    try:
        def read_json(path):
            content = path.read_bytes()
            return json.loads(content), hashlib.sha256(content).hexdigest()

        floor, floor_hash = read_json(args.floor_snapshot)
        cameras = {}
        camera_hashes = {}
        for leg in LEGS:
            cameras[leg], camera_hashes[leg] = read_json(
                getattr(args, f"{leg.lower()}_camera_l"))
        angle_review, _ = read_json(args.angle_review)
        physical_review, _ = read_json(args.physical_review)
        current_uids, _ = read_json(args.current_uids)
        urdf_bytes = args.urdf.read_bytes()
        _require(hashlib.sha256(urdf_bytes).hexdigest() == URDF_SHA256,
                 "D17 URDF SHA-256 differs")
        report = build_candidate(
            floor_snapshot=floor, floor_snapshot_sha256=floor_hash,
            camera_l_by_leg=cameras, camera_l_sha256_by_leg=camera_hashes,
            angle_review=angle_review, physical_review=physical_review,
            geometry=parse_d17(urdf_bytes), current_boot_id=args.current_boot_id,
            current_motor_uids=current_uids, rise_mm=args.rise_mm,
            fixed_hip=args.fixed_hip)
        output = private_new_path(args.output)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        with os.fdopen(os.open(output, flags, 0o600), "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"OFFLINE_GEOMETRY_CANDIDATE_ONLY: {output}")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
