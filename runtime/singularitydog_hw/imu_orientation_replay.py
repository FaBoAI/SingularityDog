"""Offline gyro-only relative orientation replay of fixed-mount capture B.

Revalidates original captures A/B and fits a candidate gyro bias to A only.
Uses Hamilton unit quaternions [w, x, y, z] mapping sensor_current vectors to
sensor_at_t0: v_t0 = q * [0, v_current] * conjugate(q). Sensor-frame angular
rates therefore compose on the right: q_next = q * exp(omega_midpoint * dt/2).
Identity at t0 defines a relative reference, not a horizontal body frame.

This is numerical gyro integration, not sensor fusion or true attitude. It
cannot validate yaw, gyro scale, mounting rotation, or acceleration calibration.
No hardware is opened, no motor output exists, and nothing is runtime-approved.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path

from . import imu_fixed_mount_baseline as baseline


def _multiply(a, b):
    """Hamilton product, scalar first; composition applies b before a."""
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def _angle_deg(q):
    """Shortest rotation from identity, invariant under q -> -q (0..180)."""
    return math.degrees(2*math.atan2(math.hypot(*q[1:]), abs(q[0])))


def _finite_vector(vector):
    return (isinstance(vector, (list, tuple)) and len(vector) == 3
            and all(baseline._finite(v) for v in vector))


def integrate_gyro(timestamps_ns, rates_rad_s, *, bias_rad_s=(0., 0., 0.)):
    """Integrate sensor-frame rates at actual timestamps; no attitude reference.

    Trapezoidal angular rates feed an exponential step, followed by unit-norm
    normalization. This midpoint approximation does not correct coning or
    reconstruct motion between samples. No nominal sampling rate is assumed.
    """
    baseline._require(len(timestamps_ns) == len(rates_rad_s) >= 2,
                      "matching timestamp/rate sequences with at least two samples required")
    baseline._require(_finite_vector(bias_rad_s), "invalid/nonfinite gyro bias")
    baseline._require(all(type(t) is int and 0 < t < 2**63 for t in timestamps_ns),
                      "invalid integration timestamp")
    baseline._require(all(_finite_vector(v) for v in rates_rad_s), "invalid/nonfinite gyro rate")
    rates = [[v-b for v, b in zip(rate, bias_rad_s)] for rate in rates_rad_s]
    baseline._require(all(_finite_vector(v) for v in rates), "nonfinite bias-subtracted gyro rate")
    q, series = [1., 0., 0., 0.], []
    for index, timestamp in enumerate(timestamps_ns):
        if index:
            dt = (timestamp-timestamps_ns[index-1])/1e9
            baseline._require(0 < dt <= baseline.LIMITS["max_gap_s"],
                              "duplicate/noncontinuous integration timestamps")
            midpoint = [a/2+b/2 for a, b in zip(rates[index-1], rates[index])]
            rotation = [rate*dt for rate in midpoint]
            theta = math.hypot(*rotation)
            baseline._require(math.isfinite(theta), "nonfinite integration step")
            scale = math.sin(theta/2)/theta if theta else .5
            step = [math.cos(theta/2), *(v*scale for v in rotation)]
            q = _multiply(q, step)
            norm = math.hypot(*q)
            baseline._require(math.isfinite(norm) and norm > 0, "invalid integrated quaternion")
            q = [v/norm for v in q]
        series.append({"monotonic_ns": timestamp,
                       "elapsed_s": (timestamp-timestamps_ns[0])/1e9,
                       "quaternion_wxyz": list(q),
                       "relative_shortest_rotation_deg": _angle_deg(q)})
    return {"time_series": series,
            "final_relative_shortest_rotation_deg": series[-1]["relative_shortest_rotation_deg"],
            "max_relative_shortest_rotation_deg": max(r["relative_shortest_rotation_deg"] for r in series)}


def _direction_angle(a, b):
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    return math.degrees(math.atan2(math.hypot(*cross), sum(x*y for x, y in zip(a, b))))


def _acceleration_monitor(rows, stats):
    series = []
    initial = None
    for index, row in enumerate(rows):
        accel = row["accel_m_s2"]
        norm = math.hypot(*accel)
        direction = [v/norm for v in accel] if norm else None
        if not index:
            initial = direction
        series.append({"monotonic_ns": row["monotonic_ns"],
                       "elapsed_s": (row["monotonic_ns"]-rows[0]["monotonic_ns"])/1e9,
                       "raw_accel_lsb": list(row["raw_accel"]),
                       "uncorrected_accel_m_s2": list(accel),
                       "norm_m_s2": norm,
                       "norm_deviation_from_1g_percent": 100*(norm/baseline.GRAVITY-1),
                       "specific_force_direction_sensor_unit": direction,
                       "direction_change_from_first_sample_deg":
                           _direction_angle(initial, direction) if initial and direction else None,
                       "temperature_c": row["temperature_c"]})
    changes = [r["direction_change_from_first_sample_deg"] for r in series
               if r["direction_change_from_first_sample_deg"] is not None]
    return {"used_for_orientation_correction": False,
            "reference": "uncorrected specific force in sensor axes; not body gravity",
            "norm_mean_m_s2": stats["accel_norm_mean_m_s2"],
            "norm_min_m_s2": stats["accel_norm_min_m_s2"],
            "norm_max_m_s2": stats["accel_norm_max_m_s2"],
            "norm_deviation_from_1g_percent": stats["gravity_norm_deviation_percent"],
            "max_direction_change_from_first_sample_deg": max(changes) if changes else None,
            "time_series": series}


def replay_captures(base_a, base_b, *, operator_confirmed_stationary=False):
    """Recompute A-only candidate, validate B independently, then replay B."""
    baseline._require(operator_confirmed_stationary is True,
                      "explicit operator-confirmed stationary assertion for both captures required")
    comparison = baseline.compare_captures(
        base_a, base_b, operator_confirmed_stationary=operator_confirmed_stationary)
    baseline._require(comparison["gyro_bias_candidate_eligible"],
                      "gyro bias candidate ineligible: " + ", ".join(comparison["failed_diagnostic_gates"]))
    # Reuse the same raw/SI, configuration, provenance and timestamp validator.
    # Refuse a changed B between comparison and loading the rows for integration.
    _, rows, stats, provenance = baseline._load_capture(base_b)
    baseline._require(provenance == comparison["provenance"]["b"], "capture B changed during replay")
    bias = comparison["gyro_bias_candidate_rad_s"]
    timestamps, rates = [r["monotonic_ns"] for r in rows], [r["gyro_rad_s"] for r in rows]
    return {
        "schema_version": 1, "kind": "gyro_only_orientation_replay", "status": "OFFLINE_DIAGNOSTIC_ONLY",
        "approved_for_runtime": False, "automatically_applied": False, "hardware_opened": False,
        "motor_output_available": False, "calibration_verified": False,
        "operator_confirmed_stationary": True, "stationarity_verified_by_software": False,
        "accel_bias_estimated": False, "accel_scale_estimated": False,
        "mount_rotation_applied": False, "absolute_level_verified": False,
        "sensor_fusion_applied": False, "absolute_attitude_verified": False,
        "frame": "sensor", "axis_order": ["x", "y", "z"],
        "quaternion_convention": {
            "order": ["w", "x", "y", "z"], "algebra": "Hamilton",
            "maps_from": "sensor_current", "maps_to": "sensor_at_t0",
            "vector_transform": "v_t0 = q * [0, v_current] * conjugate(q)",
            "step": "q_next = normalize(q * exp(omega_midpoint * actual_dt / 2))",
            "initial": [1., 0., 0., 0.],
            "initial_reference": "relative sensor frame at B start; no horizontal/body alignment",
            "angle": "shortest rotation from initial identity, 0..180 degrees; not accumulated path length"},
        "gyro_bias_candidate_rad_s": bias, "gyro_bias_fit_capture": "a", "replayed_capture": "b",
        "correction_formula": "corrected_sensor_rad_s = raw_sensor_rad_s - A_mean_sensor_rad_s",
        "samples": stats["samples"], "actual_duration_s": stats["duration_s"],
        "temperature_c": stats["temperature_c"],
        "raw": integrate_gyro(timestamps, rates),
        "subtract_a_bias": integrate_gyro(timestamps, rates, bias_rad_s=bias),
        "acceleration_monitor": _acceleration_monitor(rows, stats),
        "baseline_validation": comparison,
        "provenance": {**comparison["provenance"], "replay_source_sha256": {
            Path(path).name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
            for path in (__file__, baseline.__file__)}},
        "warnings": comparison["warnings"],
        "limitations": [
            "Gyro-only numerical integration is not sensor fusion, true attitude or drift-free yaw.",
            "Identity at B start is an arbitrary relative reference, not a horizontal body pose.",
            "A alone fits the bias; B is a held-out recording, not independent physical attitude truth.",
            "Shorter residual rotation after subtraction does not validate bias under motion or other temperatures.",
            "Acceleration is monitored unchanged and never supplies orientation correction.",
            "Host sample timestamps approximate measurement times; missed conversions and intra-step motion remain unknown.",
            "Midpoint exponential integration does not correct coning, sensor scale or cross-axis error.",
        ]}


def write_replay(base_a, base_b, output, *, operator_confirmed_stationary=False):
    """Exclusively create mode-0600 JSON outside Git after all validation."""
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output already exists: " + str(output))
    output = output.resolve()
    baseline._require(not any((p/".git").exists() or (p/".git").is_symlink()
                              for p in (output.parent, *output.parent.parents)), "output must be outside Git")
    result = replay_captures(base_a, base_b, operator_confirmed_stationary=operator_confirmed_stationary)
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    serialized = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n"
    descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(serialized)
    except BaseException:
        output.unlink(missing_ok=True)
        raise
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-a", required=True, help="original A imu_capture directory; fits bias")
    parser.add_argument("--base-b", required=True, help="original later B imu_capture directory; replay only")
    parser.add_argument("--output", required=True, help="new private JSON outside Git; parent must exist")
    parser.add_argument("--operator-confirmed-stationary", action="store_true", required=True,
                        help="assert both recordings stationary; never grants runtime approval")
    args = parser.parse_args(argv)
    try:
        result = write_replay(args.base_a, args.base_b, args.output,
                              operator_confirmed_stationary=args.operator_confirmed_stationary)
    except (OSError, ValueError) as error:
        parser.exit(2, "Replay rejected: %s\n" % error)
    print(json.dumps({"output": str(Path(args.output).expanduser().resolve()), "status": result["status"],
                      "actual_duration_s": result["actual_duration_s"],
                      "raw_final_rotation_deg": result["raw"]["final_relative_shortest_rotation_deg"],
                      "subtract_a_bias_final_rotation_deg":
                          result["subtract_a_bias"]["final_relative_shortest_rotation_deg"],
                      "approved_for_runtime": False, "motor_output_available": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
