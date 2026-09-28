"""Offline, fixed-mount IMU commissioning audit; no device or motor I/O.

Combines two independently recorded stationary captures and explicitly labelled
outbound/return movements. A proper rotation and a bias candidate are useful
diagnostics, but never constitute runtime/actuation approval. One static pose
cannot identify accelerometer bias, scale, absolute level, or yaw.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics

from . import imu_fixed_mount_baseline as baseline
from .policy_shadow import validate_imu_mount_candidate

MOUNT_ROTATION = [[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]]
MOVEMENTS = {"nose_up": (1, -1), "left_side_up": (0, 1), "turn_left": (2, 1)}
AXIS_LIMITS = {"minimum_excursion_deg": 3., "maximum_excursion_deg": 60.,
               "minimum_axis_dominance": 1.5, "maximum_return_fraction_error": .7,
               "minimum_window_s": .5, "maximum_window_s": 15.}


def mount_candidate():
    return {"schema_version": 1, "status": "IMU_MOUNT_CANDIDATE_ONLY",
            "input_frame": "sensor", "output_frame": "body_x_forward_y_left_z_up",
            "R_body_from_sensor": [list(row) for row in MOUNT_ROTATION],
            "raw_driver_axes_verified": False, "approved_for_runtime": False,
            "provenance": {"mount_assertion": "sensor X points dog-left; component face down",
                           "use": "offline candidate; physical directions require movement records"}}


def rotate(rotation, vector):
    return [sum(row[i] * vector[i] for i in range(3)) for row in rotation]


def _window(rows, window):
    baseline._require(isinstance(window, list) and len(window) == 2
                      and all(baseline._finite(v) for v in window), "window requires two finite seconds")
    start, end = window
    baseline._require(0 <= start < end and AXIS_LIMITS["minimum_window_s"] <= end-start
                      <= AXIS_LIMITS["maximum_window_s"], "movement window length outside diagnostic limits")
    first = rows[0]["monotonic_ns"]
    baseline._require(end <= (rows[-1]["monotonic_ns"] - first)/1e9,
                      "movement window exceeds captured interval")
    selected = [row for row in rows if start <= (row["monotonic_ns"]-first)/1e9 <= end]
    baseline._require(len(selected) >= 20, "movement window has fewer than 20 samples")
    baseline._require((selected[-1]["monotonic_ns"]-selected[0]["monotonic_ns"])/1e9
                      >= (end-start)-.25, "movement window incompletely covered")
    return selected


def motion_window(rows, window, rotation, bias):
    selected = _window(rows, window)
    rates = [rotate(rotation, [v-b for v, b in zip(row["gyro_rad_s"], bias)]) for row in selected]
    integral = [0., 0., 0.]
    absolute_integral = [0., 0., 0.]
    for a, b, wa, wb in zip(selected, selected[1:], rates, rates[1:]):
        dt = (b["monotonic_ns"] - a["monotonic_ns"])/1e9
        for axis in range(3):
            integral[axis] += (wa[axis] + wb[axis]) * .5 * dt
            absolute_integral[axis] += (abs(wa[axis]) + abs(wb[axis])) * .5 * dt
    return {"requested_window_s": list(window), "samples": len(selected),
            "actual_duration_s": (selected[-1]["monotonic_ns"]-selected[0]["monotonic_ns"])/1e9,
            "body_gyro_component_integral_deg": [math.degrees(v) for v in integral],
            "body_gyro_absolute_component_integral_deg": [math.degrees(v) for v in absolute_integral],
            "interpretation": "component integrals for axis/sign screening, not Euler angles or ground truth"}


def audit_movement(rows, item, rotation, bias):
    name = item.get("movement")
    baseline._require(name in MOVEMENTS, "unknown movement label")
    confirmed = item.get("operator_direction_confirmed", False)
    baseline._require(type(confirmed) is bool, "direction confirmation must be boolean")
    outgoing, returning = item.get("outbound_s"), item.get("return_s")
    out = motion_window(rows, outgoing, rotation, bias)
    back = motion_window(rows, returning, rotation, bias)
    baseline._require(outgoing[1] <= returning[0], "outbound and return windows overlap or reverse")
    axis, sign = MOVEMENTS[name]
    o, r = out["body_gyro_component_integral_deg"], back["body_gyro_component_integral_deg"]
    dominance = []
    for vector in (o, r):
        cross = math.hypot(*(vector[i] for i in range(3) if i != axis))
        dominance.append(abs(vector[axis])/max(cross, 1e-9))
    path_dominance = []
    for result in (out, back):
        vector = result["body_gyro_absolute_component_integral_deg"]
        path_dominance.append(vector[axis]/max(math.hypot(*(vector[i] for i in range(3) if i != axis)), 1e-9))
    out_abs = out["body_gyro_absolute_component_integral_deg"][axis]
    ret_abs = back["body_gyro_absolute_component_integral_deg"][axis]
    gates = {
        "operator_direction_confirmed": confirmed,
        "outbound_sign": sign*o[axis] > 0,
        "return_sign": sign*r[axis] < 0,
        "outbound_excursion": AXIS_LIMITS["minimum_excursion_deg"] <= abs(o[axis]) <= AXIS_LIMITS["maximum_excursion_deg"],
        "return_excursion": AXIS_LIMITS["minimum_excursion_deg"] <= abs(r[axis]) <= AXIS_LIMITS["maximum_excursion_deg"],
        "primary_axis_dominates": min(dominance) >= AXIS_LIMITS["minimum_axis_dominance"],
        "off_axis_path_does_not_dominate": min(path_dominance) >= 1.,
        "mostly_one_direction_per_window": abs(o[axis]) >= .7*out_abs and abs(r[axis]) >= .7*ret_abs,
        "return_is_comparable": abs(o[axis]+r[axis])/max(abs(o[axis]), 1e-9)
                                 <= AXIS_LIMITS["maximum_return_fraction_error"],
    }
    return {"movement": name, "body_axis": "xyz"[axis], "expected_outbound_sign": sign,
            "outbound": out, "return": back, "axis_dominance": dominance, "absolute_path_axis_dominance": path_dominance,
            "diagnostic_gates": gates, "axis_sign_supported": all(gates.values()),
            "failed_gates": [k for k, v in gates.items() if not v]}


def _same_setup(reference, other):
    for key in ("source_sha256", "configuration", "register_audit_before"):
        baseline._require(reference[key] == other[key], "capture setup mismatch: " + key)
    for key in ("bus", "address"):
        baseline._require(reference["plan"][key] == other["plan"][key], "capture device mismatch: " + key)


def _static_body(rows, rotation, bias):
    accel = [rotate(rotation, row["accel_m_s2"]) for row in rows]
    gyro = [rotate(rotation, [v-b for v, b in zip(row["gyro_rad_s"], bias)]) for row in rows]
    mean_a = [statistics.fmean(v[i] for v in accel) for i in range(3)]
    norm = math.hypot(*mean_a)
    return {"samples": len(rows), "specific_force_body_mean_m_s2": mean_a,
            "gyro_body_corrected_mean_rad_s": [statistics.fmean(v[i] for v in gyro) for i in range(3)],
            "gyro_body_corrected_norm_max_rad_s": max(math.hypot(*v) for v in gyro),
            "gravity_direction_candidate_body": [-v/norm for v in mean_a],
            "raw_norm_mean_m_s2": statistics.fmean(math.hypot(*v) for v in accel),
            "normalization_is_acceleration_calibration": False,
            "gravity_direction_valid_under_dynamic_acceleration": False}


def audit_manifest(manifest, *, root=Path(".")):
    """Return batch diagnostics. Malformed/incomplete movements remain unresolved."""
    baseline._require(isinstance(manifest, dict) and type(manifest.get("schema_version")) is int
                      and manifest["schema_version"] == 1,
                      "require commissioning manifest schema 1")
    root = Path(root).resolve()
    def path(value):
        baseline._require(isinstance(value, str) and value.strip(), "capture path must be nonempty string")
        p = Path(value).expanduser()
        return p if p.is_absolute() else root/p
    mount = validate_imu_mount_candidate(manifest.get("mount_candidate", mount_candidate()))
    rotation = mount["R_body_from_sensor"]
    static = manifest.get("stationary", {})
    baseline._require(isinstance(static, dict), "stationary description must be object")
    a, b = path(static.get("a")), path(static.get("b"))
    base = baseline.compare_captures(a, b, operator_confirmed_stationary=static.get("operator_confirmed", False))
    ma, ra, _, pa = baseline._load_capture(a)
    _, rb, _, pb = baseline._load_capture(b)
    baseline._require(pa == base["provenance"]["a"] and pb == base["provenance"]["b"],
                      "static capture changed during audit")
    bias = base["gyro_bias_candidate_rad_s"]
    motions = manifest.get("movements", [])
    baseline._require(isinstance(motions, list), "movements must be list")
    seen, sequences, motion_reports = set(), set(), []
    for item in motions:
        baseline._require(isinstance(item, dict), "movement must be object")
        name = item.get("movement")
        baseline._require(name in MOVEMENTS and name not in seen, "unknown/duplicate movement")
        seen.add(name)
        try:
            meta, rows, _, prov = baseline._load_capture(path(item.get("capture")))
            _same_setup(ma, meta)
            monotonic_gap = (prov["monotonic_interval_ns"][0]-pb["monotonic_interval_ns"][1])/1e9
            wall_gap = (prov["wall_interval_ns"][0]-pb["wall_interval_ns"][1])/1e9
            baseline._require(monotonic_gap > 0 and wall_gap > 0 and
                              abs(monotonic_gap-wall_gap) <= baseline.LIMITS["clock_agreement_s"],
                              "movement must follow static B in the same clock epoch")
            baseline._require(prov["measurement_sequence_sha256"] not in
                              (pa["measurement_sequence_sha256"], pb["measurement_sequence_sha256"]),
                              "movement cannot reuse a static recording")
            baseline._require(prov["measurement_sequence_sha256"] not in sequences,
                              "different movements cannot reuse one measurement sequence")
            sequences.add(prov["measurement_sequence_sha256"])
            baseline._require(bias is not None, "no eligible stationary gyro candidate")
            report = audit_movement(rows, item, rotation, bias)
            report["provenance"] = prov
        except (OSError, ValueError) as error:
            report = {"movement": name, "axis_sign_supported": False, "error": str(error)}
        motion_reports.append(report)
    checks = {"gyro_bias_candidate": base["gyro_bias_candidate_eligible"],
              "proper_rotation": True,
              "raw_accel_norm_within_3_percent": all(abs(base["captures"][name]["gravity_norm_deviation_percent"]) <= 3.
                                                     for name in ("a", "b")),
              **{name+"_axis_sign": any(r["movement"] == name and r["axis_sign_supported"]
                                       for r in motion_reports) for name in MOVEMENTS},
              "accel_bias_and_scale_identified": False,
              "absolute_level_and_heading_validated": False}
    return {"schema_version": 1, "kind": "fixed_mount_commissioning_audit",
            "status": "OFFLINE_CANDIDATE_REVIEW_REQUIRED", "approved_for_runtime": False,
            "motor_output_available": False, "hardware_opened": False, "automatically_applied": False,
            "mount_candidate": mount, "stationary": base,
            "heldout_body_diagnostic": _static_body(rb, rotation, bias) if bias is not None else None,
            "movements": motion_reports, "checks": checks, "axis_limits": dict(AXIS_LIMITS),
            "gyro_bias_candidate_sensor_rad_s": bias,
            "transform_if_validated": "omega_body = R * (omega_sensor - bias_sensor); specific_force_body = R * raw_accel",
            "unresolved": [key for key, passed in checks.items() if not passed],
            "limitations": [
                "All motion directions and stationarity are operator assertions; this audit does not observe the robot.",
                "A proper rotation does not establish correct physical mount alignment.",
                "One static pose cannot separate accelerometer bias and scale; raw norm anomaly remains visible.",
                "Gyro bias is fitted only from A and checked on B; temperature/boot changes need fresh evidence.",
                "Signed component integrals test axis/sign, not precise attitude; absolute yaw is not recovered.",
                "Motion-window thresholds are screening heuristics, not motor safety limits or deployment approval.",
                "Normalized acceleration supplies only a gravity-direction candidate while stationary; dynamic acceleration needs a validated estimator.",
            ]}


def write_audit(manifest_path, output):
    source = Path(manifest_path).expanduser().resolve()
    raw = source.read_bytes()
    manifest = baseline._json(raw)
    result = audit_manifest(manifest, root=source.parent)
    target = Path(output).expanduser().resolve()
    baseline._require(not any((p/".git").exists() for p in (target.parent, *target.parents)),
                      "audit output must be outside Git")
    result["manifest_sha256"] = hashlib.sha256(raw).hexdigest()
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True, help="new private JSON outside Git")
    args = parser.parse_args(argv)
    try:
        result = write_audit(args.manifest, args.output)
    except (OSError, ValueError) as error:
        parser.exit(2, "IMU audit rejected: %s\n" % error)
    print(json.dumps({"status": result["status"], "checks": result["checks"],
                      "output": str(Path(args.output).resolve()), "approved_for_runtime": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
