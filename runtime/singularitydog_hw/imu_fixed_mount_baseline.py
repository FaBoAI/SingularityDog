"""Offline comparison of two unchanged-mount imu_capture directories.

Never opens hardware, rotates samples, estimates acceleration calibration, or
installs corrections. An eligible gyro bias remains an unapproved candidate.
Use 30 seconds per capture after settling; 9.5 seconds is the acceptance floor
for the existing capture tool's minimum 10-second request. All gates below are
diagnostic heuristics, not validated physical safety limits.
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


GRAVITY = 9.80665
LIMITS = {
    "min_samples": 100,
    "min_duration_s": 9.5,
    "max_gap_s": 0.25,
    "clock_agreement_s": 0.25,
    "accel_std_max_g": 0.03,
    "gyro_std_max_rad_s": 0.03,
    "gyro_mean_max_rad_s": 0.15,
    "gyro_residual_mean_max_rad_s": 0.03,
    "accel_repeatability_max_g": 0.03,
    "direction_repeatability_max_deg": 3.0,
    "temperature_span_max_c": 2.0,
    "temperature_mean_change_max_c": 2.0,
    "temperature_drift_max_c": 1.0,
    "accel_norm_warning_relative": 0.03,
}
_TRIM_KEYS = {f"bank{bank}:0x{address:02X}" for bank, addresses in {
    1: (0x02, 0x03, 0x04, 0x0E, 0x0F, 0x10, 0x14, 0x15, 0x17, 0x18, 0x1A, 0x1B, 0x28),
    2: (0x03, 0x04, 0x05, 0x06, 0x07, 0x08),
}.items() for address in addresses}


class BaselineError(ValueError):
    """Capture provenance, structure or unmodified measurements are invalid."""


def _require(condition, message):
    if not condition:
        raise BaselineError(message)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _reject_constant(value):
    raise BaselineError("nonfinite JSON constant: " + value)


def _object(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, "duplicate JSON key: " + key)
        result[key] = value
    return result


def _json(text):
    try:
        value = json.loads(text, parse_constant=_reject_constant, object_pairs_hook=_object)
    except (ValueError, RecursionError, UnicodeError) as error:
        raise BaselineError("invalid JSON: " + str(error)) from error
    _require(isinstance(value, dict), "JSON record must be an object")
    return value


def _hash(data):
    return hashlib.sha256(data).hexdigest()


def _vector(record, key, *, raw=False):
    value = record.get(key)
    _require(isinstance(value, list) and len(value) == 3, "invalid vector: " + key)
    if raw:
        _require(all(type(v) is int and -32768 <= v <= 32767 for v in value),
                 "invalid signed16 vector: " + key)
    else:
        _require(all(_finite(v) for v in value), "nonfinite vector: " + key)
    return value


def _mean(rows, key):
    return [statistics.fmean(row[key][i] for row in rows) for i in range(3)]


def _std(rows, key):
    return [statistics.pstdev(row[key][i] for row in rows) for i in range(3)]


def _delta(a, b):
    return [x-y for x, y in zip(a, b)]


def _direction(vector):
    norm = math.hypot(*vector)
    _require(norm > 0.1 * GRAVITY, "acceleration mean has no useful reference direction")
    return [v/norm for v in vector]


def _close(actual, expected):
    return _finite(actual) and math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-10)


def _uncorrected(record):
    for key in ("calibration_applied", "orientation_applied", "mount_rotation_applied",
                "accel_bias_subtracted", "accel_scale_corrected", "gyro_bias_subtracted"):
        _require(key not in record or record[key] is False, "previous correction: " + key)


def _configuration(config):
    _require(isinstance(config, dict), "configuration missing")
    _require(config.get("who_am_i") == 0xEA and config.get("address") in (0x68, 0x69),
             "invalid sensor identity/address")
    _require(config.get("frame") == "sensor" and config.get("axis_order") == ["x", "y", "z"]
             and config.get("orientation_applied") is False, "original sensor frame required")
    _uncorrected(config)
    registers = config.get("registers")
    _require(isinstance(registers, dict) and all(type(v) is int and 0 <= v <= 255
             for v in registers.values()), "invalid configuration readback")
    _require(all(k in registers for k in ("bank2:0x14", "bank2:0x01", "bank0:0x06")),
             "missing range/power readback")
    accel_fs, gyro_fs = (registers["bank2:0x14"] >> 1) & 3, (registers["bank2:0x01"] >> 1) & 3
    accel_scale = GRAVITY / (16384., 8192., 4096., 2048.)[accel_fs]
    gyro_scale = math.pi / 180 / (131., 65.5, 32.8, 16.4)[gyro_fs]
    _require(config.get("accel_range_g") == (2, 4, 8, 16)[accel_fs]
             and config.get("gyro_range_dps") == (250, 500, 1000, 2000)[gyro_fs]
             and _close(config.get("accel_m_s2_per_lsb"), accel_scale)
             and _close(config.get("gyro_rad_s_per_lsb"), gyro_scale), "range/scale readback mismatch")
    _require(not registers["bank0:0x06"] & 8, "temperature measurement is required")
    return accel_scale, gyro_scale


def _trim(audit):
    _require(isinstance(audit, dict) and audit.get("offset_registers_written") is False
             and audit.get("self_test_executed") is False, "unmodified trim audit required")
    values = audit.get("raw_registers")
    _require(isinstance(values, dict) and set(values) == _TRIM_KEYS and all(
        type(v) is int and 0 <= v <= 255 for v in values.values()), "missing/invalid trim registers")


def _statistics(rows):
    norms = [math.hypot(*r["accel_m_s2"]) for r in rows]
    temperatures = [r["temperature_c"] for r in rows]
    duration = (rows[-1]["monotonic_ns"]-rows[0]["monotonic_ns"])/1e9
    quarter = max(1, len(rows)//4)
    first, last = rows[:quarter], rows[-quarter:]
    mean_accel = _mean(rows, "accel_m_s2")
    return {
        "samples": len(rows), "duration_s": duration, "rate_hz": (len(rows)-1)/duration,
        "accel_mean_m_s2": mean_accel, "accel_std_m_s2": _std(rows, "accel_m_s2"),
        "gyro_mean_rad_s": _mean(rows, "gyro_rad_s"), "gyro_std_rad_s": _std(rows, "gyro_rad_s"),
        "accel_norm_mean_m_s2": statistics.fmean(norms),
        "accel_norm_std_m_s2": statistics.pstdev(norms),
        "accel_norm_min_m_s2": min(norms), "accel_norm_max_m_s2": max(norms),
        "gravity_norm_deviation_percent": 100*(statistics.fmean(norms)/GRAVITY-1),
        "specific_force_direction_sensor_unit": _direction(mean_accel),
        "temperature_c": {"mean": statistics.fmean(temperatures), "min": min(temperatures),
                          "max": max(temperatures), "first": temperatures[0], "last": temperatures[-1]},
        "first_to_last_quarter": {
            "accel_mean_change_m_s2": _delta(_mean(last, "accel_m_s2"), _mean(first, "accel_m_s2")),
            "gyro_mean_change_rad_s": _delta(_mean(last, "gyro_rad_s"), _mean(first, "gyro_rad_s")),
            "temperature_mean_change_c": statistics.fmean(r["temperature_c"] for r in last)
                - statistics.fmean(r["temperature_c"] for r in first)},
    }


def _load_capture(directory):
    directory = Path(directory).expanduser().resolve()
    summary_bytes, event_bytes = (directory/"summary.json").read_bytes(), (directory/"events.jsonl").read_bytes()
    metadata = _json(summary_bytes)
    _require(metadata.get("status") == "RECORDED_NOT_CALIBRATED" and metadata.get("errors") == []
             and metadata.get("restore_status") == "restored", "capture incomplete or restoration not verified")
    plan = metadata.get("plan")
    _require(isinstance(plan, dict) and plan.get("can_opened") is False
             and plan.get("calibration_applied") is False and plan.get("face_label") == "unverified",
             "unmodified fixed-mount capture plan required")
    _require(isinstance(plan.get("bus"), str) and bool(plan["bus"])
             and type(plan.get("address")) is int and plan["address"] in (0x68, 0x69),
             "capture device path/address missing")
    _require(type(plan.get("capture_seconds")) is int and 10 <= plan["capture_seconds"] <= 120
             and type(plan.get("settle_seconds")) is int and plan["settle_seconds"] >= 1,
             "invalid capture/settling duration")
    sources = metadata.get("source_sha256")
    _require(isinstance(sources, dict) and set(sources) == {"imu.py", "imu_capture.py"}
             and all(isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)
                     for v in sources.values()), "source hashes missing/invalid")
    accel_scale, gyro_scale = _configuration(metadata.get("configuration"))
    _require(plan["address"] == metadata["configuration"]["address"], "plan/configuration address mismatch")
    for key in ("register_audit_before", "register_audit_after"):
        _trim(metadata.get(key))
    _require(metadata["register_audit_before"] == metadata["register_audit_after"], "trim audit changed")
    records = [_json(line) for line in event_bytes.splitlines() if line.strip()]
    _require(records and records[0] == {"kind": "capture_metadata", **plan}, "capture metadata event mismatch")
    rows = records[1:]
    _require(len(rows) >= LIMITS["min_samples"], "insufficient samples")
    for row in rows:
        _require(row.get("kind") == "imu" and row.get("frame") == "sensor", "unexpected event/frame")
        _uncorrected(row)
        for key in ("sequence", "monotonic_ns", "wall_time_ns", "read_started_monotonic_ns",
                    "read_finished_monotonic_ns"):
            _require(type(row.get(key)) is int and 0 < row[key] < 2**63, "invalid timestamp/sequence: " + key)
        start, end = row["read_started_monotonic_ns"], row["read_finished_monotonic_ns"]
        _require(start <= row["monotonic_ns"] <= end and end-start <= 250_000_000,
                 "invalid read interval")
        _require(type(row.get("data_ready_status")) is int and row["data_ready_status"] & 1,
                 "sample lacks data ready")
        for raw_key, si_key, scale in (("raw_accel", "accel_m_s2", accel_scale),
                                       ("raw_gyro", "gyro_rad_s", gyro_scale)):
            raw, si = _vector(row, raw_key, raw=True), _vector(row, si_key)
            _require(all(_close(v, r*scale) for r, v in zip(raw, si)), "raw/SI conversion mismatch: " + si_key)
        raw_temp, temp = row.get("raw_temperature"), row.get("temperature_c")
        _require(type(raw_temp) is int and -32768 <= raw_temp <= 32767
                 and _close(temp, raw_temp/333.87+21.) and -40 <= temp <= 85,
                 "raw/temperature conversion mismatch or out of range")
    for a, b in zip(rows, rows[1:]):
        _require(b["sequence"] == a["sequence"]+1, "noncontiguous sample sequence")
        delta_m, delta_w = (b["monotonic_ns"]-a["monotonic_ns"])/1e9, (b["wall_time_ns"]-a["wall_time_ns"])/1e9
        _require(0 < delta_m <= LIMITS["max_gap_s"] and delta_w > 0, "duplicate/noncontinuous timestamps")
        _require(abs(delta_w-delta_m) <= LIMITS["clock_agreement_s"], "clock discontinuity")
        _require(b["read_started_monotonic_ns"] >= a["read_finished_monotonic_ns"], "overlapping read intervals")
    duration = (rows[-1]["monotonic_ns"]-rows[0]["monotonic_ns"])/1e9
    wall_duration = (rows[-1]["wall_time_ns"]-rows[0]["wall_time_ns"])/1e9
    _require(duration >= LIMITS["min_duration_s"] and duration >= plan["capture_seconds"]-.5
             and duration <= plan["capture_seconds"]+.25, "insufficient/inconsistent capture duration")
    _require(abs(wall_duration-duration) <= LIMITS["clock_agreement_s"], "wall/monotonic duration mismatch")
    stats = _statistics(rows)
    saved = metadata.get("summary")
    _require(isinstance(saved, dict) and saved.get("sensor_frame") is True
             and saved.get("calibration_applied") is False, "summary frame/calibration mismatch")
    for key in ("samples", "duration_s", "rate_hz", "accel_norm_mean_m_s2", "gravity_norm_deviation_percent"):
        _require(_close(saved.get(key), stats[key]), "summary/event mismatch: " + key)
    for key in ("accel_mean_m_s2", "accel_std_m_s2", "gyro_mean_rad_s", "gyro_std_rad_s"):
        _require(all(_close(a, b) for a, b in zip(_vector(saved, key), stats[key])), "summary/event mismatch: " + key)
    _require(isinstance(saved.get("temperature_c"), dict) and all(
        _close(saved["temperature_c"].get(k), v) for k, v in stats["temperature_c"].items()),
        "summary/event temperature mismatch")
    measurement_digest = _hash(json.dumps([[r["raw_accel"], r["raw_gyro"]]
                              for r in rows], separators=(",", ":")).encode())
    provenance = {"directory": str(directory), "summary_sha256": _hash(summary_bytes),
                  "events_sha256": _hash(event_bytes), "measurement_sequence_sha256": measurement_digest,
                  "source_sha256": sources,
                  "monotonic_interval_ns": [rows[0]["monotonic_ns"], rows[-1]["monotonic_ns"]],
                  "wall_interval_ns": [rows[0]["wall_time_ns"], rows[-1]["wall_time_ns"]]}
    return metadata, rows, stats, provenance


def compare_captures(base_a, base_b, *, operator_confirmed_stationary=False):
    """Validate two captures and return diagnostics; never writes/applies anything."""
    _require(type(operator_confirmed_stationary) is bool, "stationary confirmation must be boolean")
    a, ra, sa, pa = _load_capture(base_a)
    b, rb, sb, pb = _load_capture(base_b)
    _require(pa["directory"] != pb["directory"] and pa["events_sha256"] != pb["events_sha256"]
             and pa["measurement_sequence_sha256"] != pb["measurement_sequence_sha256"], "reused capture/measurement sequence")
    _require(a["source_sha256"] == b["source_sha256"], "capture source mismatch")
    _require(a["configuration"] == b["configuration"], "capture configuration mismatch")
    _require(a["register_audit_before"] == b["register_audit_before"], "trim differs between captures")
    _require(a["plan"]["bus"] == b["plan"]["bus"] and a["plan"]["address"] == b["plan"]["address"],
             "capture device path/address mismatch")
    gap_m = (rb[0]["read_started_monotonic_ns"]-ra[-1]["read_finished_monotonic_ns"])/1e9
    gap_w = (rb[0]["wall_time_ns"]-ra[-1]["wall_time_ns"])/1e9
    _require(gap_m > 0 and gap_w > 0, "overlapping/reversed capture intervals")
    _require(abs(gap_m-gap_w) <= LIMITS["clock_agreement_s"], "capture clocks differ; use one uninterrupted session")
    bias = sa["gyro_mean_rad_s"]
    residual = _delta(sb["gyro_mean_rad_s"], bias)
    accel_change = _delta(sb["accel_mean_m_s2"], sa["accel_mean_m_s2"])
    cosine = sum(x*y for x, y in zip(sa["specific_force_direction_sensor_unit"], sb["specific_force_direction_sensor_unit"]))
    angle = math.degrees(math.acos(max(-1., min(1., cosine))))
    temperature_change = sb["temperature_c"]["mean"]-sa["temperature_c"]["mean"]
    gates = {}
    for label, stats in (("a", sa), ("b", sb)):
        drift, temp = stats["first_to_last_quarter"], stats["temperature_c"]
        gates.update({
            label+"_accel_variation": max(stats["accel_std_m_s2"]) <= LIMITS["accel_std_max_g"]*GRAVITY,
            label+"_gyro_variation": max(stats["gyro_std_rad_s"]) <= LIMITS["gyro_std_max_rad_s"],
            label+"_gyro_mean": math.hypot(*stats["gyro_mean_rad_s"]) <= LIMITS["gyro_mean_max_rad_s"],
            label+"_temperature_span": temp["max"]-temp["min"] <= LIMITS["temperature_span_max_c"],
            label+"_temperature_drift": abs(drift["temperature_mean_change_c"]) <= LIMITS["temperature_drift_max_c"],
            label+"_gyro_drift": math.hypot(*drift["gyro_mean_change_rad_s"]) <= LIMITS["gyro_residual_mean_max_rad_s"],
            label+"_accel_drift": math.hypot(*drift["accel_mean_change_m_s2"]) <= LIMITS["accel_repeatability_max_g"]*GRAVITY})
    gates.update({"gyro_repeatability": math.hypot(*residual) <= LIMITS["gyro_residual_mean_max_rad_s"],
                  "accel_repeatability": math.hypot(*accel_change) <= LIMITS["accel_repeatability_max_g"]*GRAVITY,
                  "direction_repeatability": angle <= LIMITS["direction_repeatability_max_deg"],
                  "temperature_repeatability": abs(temperature_change) <= LIMITS["temperature_mean_change_max_c"]})
    eligible = operator_confirmed_stationary and all(gates.values())
    warnings = [f"{label}: raw acceleration norm differs from 1g; not corrected or calibrated"
                for label, stats in (("a", sa), ("b", sb)) if
                abs(stats["gravity_norm_deviation_percent"])/100 > LIMITS["accel_norm_warning_relative"]]
    return {
        "schema_version": 1, "kind": "fixed_mount_baseline",
        "status": "GYRO_BIAS_CANDIDATE" if eligible else "DIAGNOSTIC_ONLY",
        "approved_for_runtime": False, "automatically_applied": False, "hardware_opened": False,
        "motor_output_available": False, "calibration_verified": False,
        "frame": "sensor", "axis_order": ["x", "y", "z"],
        "operator_confirmed_stationary": operator_confirmed_stationary,
        "stationarity_verified_by_software": False, "gyro_bias_candidate_eligible": eligible,
        "gyro_bias_candidate_rad_s": list(bias) if eligible else None,
        "gyro_bias_formula_if_later_validated": "corrected_sensor[i] = raw_sensor[i] - bias_sensor[i]",
        "accel_bias_estimated": False, "accel_scale_estimated": False,
        "mount_rotation_applied": False, "absolute_level_verified": False,
        "captures": {"a": sa, "b": sb}, "provenance": {"a": pa, "b": pb},
        "comparison": {"gyro_b_residual_after_subtracting_a_mean_rad_s": residual,
                       "gyro_b_residual_std_rad_s": sb["gyro_std_rad_s"],
                       "accel_b_minus_a_mean_m_s2": accel_change,
                       "specific_force_direction_change_deg": angle,
                       "temperature_b_minus_a_mean_c": temperature_change},
        "diagnostic_limits": dict(LIMITS), "diagnostic_gates": gates,
        "failed_diagnostic_gates": [key for key, value in gates.items() if not value],
        "warnings": warnings,
        "limitations": [
            "Diagnostic gates are heuristic, not physical safety limits or deployment approval.",
            "The stationary assertion covers both captures; low variance cannot exclude constant rotation.",
            "Capture hashes detect exact copied sequences, not fabricated data or physical sensor identity.",
            "One pose does not identify acceleration bias/scale, mounting yaw, or absolute level.",
            "Reference direction is normalized specific force in sensor axes, not validated body gravity.",
            "Gyro scale, cross-axis error, temperature compensation and dynamic behavior remain unverified.",
            "A and B are separate captures, not proof of physically independent calibration truth.",
        ]}


def write_baseline(base_a, base_b, output, *, operator_confirmed_stationary=False):
    """Exclusively create a private JSON report outside Git; preserve all inputs."""
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output already exists: " + str(output))
    output = output.resolve()
    _require(not any((p/".git").exists() for p in (output.parent, *output.parent.parents)),
             "output must be outside Git")
    result = compare_captures(base_a, base_b, operator_confirmed_stationary=operator_confirmed_stationary)
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
    parser.add_argument("--base-a", required=True, help="first imu_capture directory")
    parser.add_argument("--base-b", required=True, help="later, separate imu_capture directory")
    parser.add_argument("--output", required=True, help="new private JSON outside Git; parent must exist")
    parser.add_argument("--operator-confirmed-stationary", action="store_true",
                        help="assert both recordings were stationary; never grants runtime approval")
    args = parser.parse_args(argv)
    try:
        result = write_baseline(args.base_a, args.base_b, args.output,
                                operator_confirmed_stationary=args.operator_confirmed_stationary)
    except (OSError, BaselineError) as error:
        parser.exit(2, "Baseline rejected: %s\n" % error)
    print(json.dumps({"output": str(Path(args.output).expanduser().resolve()), "status": result["status"],
                      "gyro_bias_candidate_eligible": result["gyro_bias_candidate_eligible"],
                      "approved_for_runtime": False, "automatically_applied": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
