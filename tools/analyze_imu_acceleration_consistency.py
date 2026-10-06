#!/usr/bin/env python3
"""Audit saved ICM-20948 captures without opening hardware or fitting a calibration.

The returned hypothetical models only explain why one gravity direction cannot
identify offset versus gain. Their coefficients are not calibration candidates.
Host data-ready races are analyzed separately and never treated as conversion
timestamps. No profile, input file, trim register, or runtime limit is changed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

G = 9.80665
LSB_PER_G = (16384.0, 8192.0, 4096.0, 2048.0)
DATASHEET = "https://invensense.tdk.com/wp-content/uploads/2021/10/DS-000189-ICM-20948-v1.5.pdf"


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite number required")
    return float(value)


def vector(value, raw=False):
    require(isinstance(value, list) and len(value) == 3, "three components required")
    if raw:
        require(all(type(v) is int and -32768 <= v <= 32767 for v in value),
                "signed16 raw components required")
    return [number(v) for v in value]


def mean_vector(rows):
    return [statistics.fmean(v[i] for v in rows) for i in range(3)]


def norm_summary(rows):
    values = [math.hypot(*v) for v in rows]
    mean = mean_vector(rows)
    mean_norm = statistics.fmean(values)
    norm_of_mean = math.hypot(*mean)
    # Jensen's observed gap is exact for this dataset, not a model of all noise.
    gap = mean_norm - norm_of_mean
    excess = mean_norm - G
    return {
        "samples": len(rows), "axis_mean_m_s2": mean,
        "axis_std_m_s2": [statistics.pstdev(v[i] for v in rows) for i in range(3)],
        "norm_mean_m_s2": mean_norm, "norm_of_mean_vector_m_s2": norm_of_mean,
        "norm_std_m_s2": statistics.pstdev(values), "norm_min_m_s2": min(values),
        "norm_max_m_s2": max(values), "norm_excess_m_s2": excess,
        "norm_excess_percent": excess / G * 100,
        "observed_norm_averaging_gap_m_s2": gap,
        "observed_norm_averaging_gap_fraction_of_excess": gap / excess if excess > 0 else None,
        "observed_fluctuation_explanation_only": True,
    }


def audit_rows(rows, configuration):
    require(len(rows) >= 2, "at least two IMU samples required")
    require(configuration.get("frame") == "sensor" and
            configuration.get("orientation_applied") is False, "unrotated sensor configuration required")
    require(configuration.get("who_am_i") == 0xEA, "ICM-20948 identity required")
    registers = configuration.get("registers", {})
    config = registers.get("bank2:0x14")
    require(type(config) is int and 0 <= config <= 255, "ACCEL_CONFIG readback required")
    fs = (config >> 1) & 3
    scale = G / LSB_PER_G[fs]
    require(configuration.get("accel_range_g") == (2, 4, 8, 16)[fs], "range/readback mismatch")
    require(abs(number(configuration.get("accel_m_s2_per_lsb")) - scale) <= 1e-15,
            "SI scale/readback mismatch")
    timestamps, accel, raw, race_groups, temperatures = [], [], [], {False: [], True: []}, []
    max_error = 0.0
    for row in rows:
        require(row.get("frame") == "sensor", "sensor frame sample required")
        for field in ("calibration_applied", "orientation_applied", "accel_bias_subtracted", "accel_scale_corrected"):
            require(row.get(field, False) is False, "already corrected sample: " + field)
        a, counts = vector(row.get("accel_m_s2")), vector(row.get("raw_accel"), raw=True)
        stamp = row.get("monotonic_ns")
        require(type(stamp) is int and stamp > 0, "positive host timestamp required")
        if timestamps:
            require(stamp > timestamps[-1], "host timestamps must increase")
        timestamps.append(stamp)
        error = max(abs(a[i] - counts[i] * scale) for i in range(3))
        require(error <= 1e-12, "saved raw/SI conversion mismatch")
        max_error = max(max_error, error)
        accel.append(a)
        raw.append(counts)
        if "data_ready_during_read" in row:
            require(type(row["data_ready_during_read"]) is bool, "invalid ready-race flag")
            race_groups[row["data_ready_during_read"]].append(a)
        if row.get("temperature_c") is not None:
            temperatures.append(number(row["temperature_c"]))
    result = norm_summary(accel)
    result.update({
        "duration_s": (timestamps[-1] - timestamps[0]) / 1e9,
        "accel_range_g": (2, 4, 8, 16)[fs], "accel_config_readback": config,
        "raw_to_si_max_abs_error_m_s2": max_error,
        "raw_axis_mean_counts": mean_vector(raw),
        "raw_saturated_component_count": sum(abs(v) >= 32767 for r in raw for v in r),
        "data_ready_race_groups": {str(flag).lower(): norm_summary(group) if group else None
                                   for flag, group in race_groups.items()},
        "data_ready_race_is_not_proof_of_torn_sample": True,
        "temperature_c": {"min": min(temperatures), "max": max(temperatures),
                          "mean": statistics.fmean(temperatures)} if temperatures else None,
        "hardware_sample_completeness_proven": False,
    })
    return result


def audit_capture(directory):
    directory = Path(directory)
    summary_path, samples_path = directory / "summary.json", directory / "events.jsonl"
    summary_bytes, samples_bytes = summary_path.read_bytes(), samples_path.read_bytes()
    summary = json.loads(summary_bytes)
    require(summary.get("status") == "RECORDED_NOT_CALIBRATED", "capture did not complete")
    require(summary.get("errors") == [], "capture errors present")
    require(summary.get("restore_status") == "restored", "register restoration not verified")
    require(summary.get("plan", {}).get("calibration_applied") is False, "raw capture required")
    rows = [r for line in samples_bytes.splitlines() if line.strip()
            for r in [json.loads(line)] if "raw_accel" in r]
    require(summary.get("summary", {}).get("samples") == len(rows), "sample count mismatch")
    result = audit_rows(rows, summary["configuration"])
    before, after = summary.get("register_audit_before"), summary.get("register_audit_after")
    require((before is None) == (after is None), "one-sided trim audit")
    if before is not None:
        require(before == after, "trim/register audit changed during capture")
        require(before.get("offset_registers_written") is False, "trim write was reported")
    result.update({
        "source": str(directory), "summary_sha256": digest(summary_bytes),
        "samples_sha256": digest(samples_bytes), "started_at": summary.get("started_at"),
        "source_sha256": summary.get("source_sha256"),
        "original_registers": summary.get("original_registers"),
        "trim_audit": before, "trim_unchanged": True if before is not None else None,
        "operator_stationarity_from_summary": summary.get("summary", {}).get("stillness_or_orientation_confirmed"),
        "stationarity_not_inferred_from_small_variance": True,
    })
    return result


def audit_range_crosscheck(directory):
    directory = Path(directory)
    report_bytes = (directory / "report.json").read_bytes()
    sample_bytes = (directory / "samples.jsonl").read_bytes()
    report = json.loads(report_bytes)
    require(report.get("status") == "CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED",
            "range comparison did not complete consistently")
    require(report.get("errors") == [], "range comparison errors present")
    rows = [json.loads(line) for line in sample_bytes.splitlines() if line.strip()]
    sessions, covered = [], set()
    for session in report["sessions"]:
        require(session.get("restore_status") == "restored", "range session not restored")
        require(session.get("requested_range_g") == session.get("configuration", {}).get("accel_range_g"),
                "range request/readback mismatch")
        start, end = (session["summary"][key] for key in ("first_monotonic_ns", "last_monotonic_ns"))
        subset = []
        for i, row in enumerate(rows):
            if start <= row["monotonic_ns"] <= end:
                require(i not in covered, "overlapping range sessions")
                require(row.get("range_g") == session["requested_range_g"], "row range mismatch")
                covered.add(i)
                subset.append(row)
        require(len(subset) == session["summary"]["sample_count"], "range sample count mismatch")
        sessions.append(audit_rows(subset, session["configuration"]))
    require(len(covered) == len(rows), "unassigned range samples")
    require([s["accel_range_g"] for s in sessions] == [2, 4, 2], "2g/4g/2g comparison required")
    two = statistics.fmean([sessions[0]["norm_mean_m_s2"], sessions[2]["norm_mean_m_s2"]])
    four = sessions[1]["norm_mean_m_s2"]
    return {"source": str(directory), "report_sha256": digest(report_bytes),
            "samples_sha256": digest(sample_bytes), "sessions": sessions,
            "four_vs_bracket_two_relative_difference_percent": (four / two - 1) * 100,
            "common_gain_or_bias_not_excluded": True}


def hypotheses(mean):
    """Two incompatible sensor models fitting the same vector, not calibration."""
    mean = vector(mean)
    n = math.hypot(*mean)
    require(n > 0, "nonzero mean required")
    axis = [v / n for v in mean]
    gain = n / G
    bias = [(n - G) * u for u in axis]
    # Hypothesis A: isotropic gain, zero offset. B: unit gain, radial offset.
    predictions = []
    for angle in (0, 10, 15, 30, 60, 90, 180):
        c = math.cos(math.radians(angle))
        bias_norm = math.sqrt(G * G + (n - G)**2 + 2 * G * (n - G) * c)
        predictions.append({"angle_from_current_gravity_deg": angle,
                            "gain_only_norm_m_s2": n, "radial_bias_only_norm_m_s2": bias_norm,
                            "predicted_norm_separation_m_s2": abs(n - bias_norm),
                            "radial_bias_normalized_direction_error_deg": abs(math.degrees(math.atan2(
                                (n-G)*math.sin(math.radians(angle)), G+(n-G)*c)))})
    return {"status": "NONIDENTIFIABILITY_EXAMPLES_NOT_CALIBRATION",
            "assumptions": "static 1g, mean direction provisionally treated as true gravity direction",
            "gain_only": {"hypothetical_gain": gain, "hypothetical_correction": 1 / gain},
            "radial_bias_only": {"hypothetical_bias_m_s2": bias},
            "prediction_by_orientation": predictions,
            "both_exactly_fit_reference_mean": True, "real_bias_or_scale_identified": False,
            "uniform_positive_scalar_does_not_change_normalized_gravity_direction": True,
            "bias_can_change_normalized_gravity_direction_away_from_reference": True,
            "coefficients_must_not_be_used_by_runtime": True}


def analyze(captures, range_crosscheck=None):
    require(bool(captures), "at least one capture required")
    summaries = [audit_capture(path) for path in captures]
    require(len({s["samples_sha256"] for s in summaries}) == len(summaries), "duplicate capture evidence")
    directions = [[v / s["norm_of_mean_vector_m_s2"] for v in s["axis_mean_m_s2"]] for s in summaries]
    max_angle = max(math.degrees(math.acos(max(-1, min(1, sum(x*y for x, y in zip(a, b))))))
                    for a in directions for b in directions)
    trim = [s["trim_audit"] for s in summaries if s["trim_audit"] is not None]
    return {"schema": "singularitydog.imu-acceleration-consistency.v1",
            "status": "OFFLINE_CONSISTENCY_ANALYSIS_NOT_CALIBRATION", "captures": summaries,
            "maximum_mean_direction_separation_deg": max_angle,
            "trim_consistent_across_available_audits": all(t == trim[0] for t in trim) if trim else None,
            "captures_without_trim_audit": len(summaries) - len(trim),
            "last_capture_hypotheses": hypotheses(summaries[-1]["axis_mean_m_s2"]),
            "range_crosscheck": audit_range_crosscheck(range_crosscheck) if range_crosscheck else None,
            "proper_rotation_preserves_norm": True,
            "remaining_identification": "independent gravity orientations plus held-out repeats; one pose cannot distinguish offset from gain",
            "sensor_frame_is_not_body_calibration": True,
            "datasheet": {"url": DATASHEET, "sections": ["3.2", "9.7–9.12", "10.15"]},
            "approved_for_runtime": False, "output_allowed": False,
            "hardware_opened": False, "profile_changed": False, "calibration_created": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, action="append", required=True)
    parser.add_argument("--range-crosscheck", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.capture, args.range_crosscheck)
    # Exclusive creation prevents replacing measurements or earlier analyses.
    with args.output.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "captures": len(result["captures"]),
                      "output": str(args.output), "approved_for_runtime": False}))


if __name__ == "__main__":
    main()
