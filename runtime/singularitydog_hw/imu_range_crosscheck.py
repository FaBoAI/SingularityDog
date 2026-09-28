"""IMU-only, supported-body comparison of ±2 g and ±4 g readback.

This diagnoses whether the observed acceleration excess depends on the full-scale
register/host conversion. It cannot identify bias versus scale from one pose and
never writes a calibration or opens CAN. Every short IMU session restores the
exact original registers before the next one begins.
"""
from __future__ import annotations

import argparse
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

from .imu import ICM20948

G = 9.80665


def summarize(samples):
    if len(samples) < 100:
        raise ValueError("fewer than 100 fresh IMU samples")
    accel = [row["accel_m_s2"] for row in samples]
    gyro = [row["gyro_rad_s"] for row in samples]
    raw_norm = [math.sqrt(sum(v * v for v in row["raw_accel"])) for row in samples]
    si_norm = [math.sqrt(sum(v * v for v in values)) for values in accel]
    return {
        "sample_count": len(samples),
        "raw_count_norm_mean": statistics.fmean(raw_norm),
        "accel_norm_mean_m_s2": statistics.fmean(si_norm),
        "accel_axis_std_m_s2": [statistics.pstdev(row[i] for row in accel)
                                 for i in range(3)],
        "gyro_axis_mean_rad_s": [statistics.fmean(row[i] for row in gyro)
                                  for i in range(3)],
        "first_monotonic_ns": samples[0]["monotonic_ns"],
        "last_monotonic_ns": samples[-1]["monotonic_ns"],
    }


def evaluate(sessions):
    if [item["requested_range_g"] for item in sessions] != [2, 4, 2]:
        raise ValueError("expected 2g, 4g, 2g sessions")
    if len(sessions) != 3:
        raise ValueError("expected three sessions")
    ranges_ok = all(item["configuration"]["accel_range_g"] == expected
                    for item, expected in zip(sessions, (2, 4, 2)))
    restored = all(item["restore_status"] == "restored" for item in sessions)
    same_original = all(item["original_registers"] == sessions[0]["original_registers"]
                        for item in sessions[1:])
    summaries = [item["summary"] for item in sessions]
    stationary = all(max(s["accel_axis_std_m_s2"]) <= 0.12 and
                     math.sqrt(sum(v * v for v in s["gyro_axis_mean_rad_s"])) <= 0.10
                     for s in summaries)
    two_g = statistics.fmean((summaries[0]["accel_norm_mean_m_s2"],
                              summaries[2]["accel_norm_mean_m_s2"]))
    four_g = summaries[1]["accel_norm_mean_m_s2"]
    two_g_counts = statistics.fmean((summaries[0]["raw_count_norm_mean"],
                                     summaries[2]["raw_count_norm_mean"]))
    four_g_counts = summaries[1]["raw_count_norm_mean"]
    bracket_drift = abs(summaries[0]["accel_norm_mean_m_s2"] -
                        summaries[2]["accel_norm_mean_m_s2"])
    checks = {
        "range_register_readback": ranges_ok,
        "all_sessions_restored": restored,
        "same_original_registers": same_original,
        "stationary": stationary,
        "two_g_bracket_stable": bracket_drift <= 0.10,
        "raw_count_ratio_two_to_four_near_two": 1.9 <= two_g_counts / four_g_counts <= 2.1,
        "si_norm_range_invariant": abs(two_g - four_g) <= 0.10,
        "raw_si_norm_within_3_percent_of_g": abs(two_g / G - 1) <= 0.03,
    }
    diagnostic_ok = all(value for key, value in checks.items()
                        if key != "raw_si_norm_within_3_percent_of_g")
    return {
        "status": ("CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED" if diagnostic_ok and
                   not checks["raw_si_norm_within_3_percent_of_g"] else
                   "CONFIG_AND_NORM_PLAUSIBLE_STILL_UNCALIBRATED" if diagnostic_ok else
                   "REVIEW_REQUIRED"),
        "checks": checks,
        "two_g_mean_m_s2": two_g,
        "four_g_mean_m_s2": four_g,
        "two_g_raw_count_norm_mean": two_g_counts,
        "four_g_raw_count_norm_mean": four_g_counts,
        "raw_norm_deviation_percent": 100.0 * (two_g / G - 1.0),
        "approved_for_runtime": False,
        "inference_limit": "One fixed pose cannot separate accelerometer offset from scale or establish body alignment.",
    }


def capture_range(range_g, *, seconds=4.0, settle_seconds=1.0):
    device = ICM20948(accel_range_g=range_g)
    samples = []
    item = {"requested_range_g": range_g}
    try:
        with device:
            item["configuration"] = device.configuration
            item["original_registers"] = device.original_registers
            until = time.monotonic() + settle_seconds
            while time.monotonic() < until:
                device.read_sample()
                time.sleep(0.01)
            until = time.monotonic() + seconds
            next_poll = time.monotonic()
            last_fresh = next_poll
            while time.monotonic() < until:
                value = device.read_sample()
                if value is not None:
                    samples.append(value)
                    last_fresh = time.monotonic()
                elif time.monotonic() - last_fresh > 0.5:
                    raise RuntimeError("No fresh IMU sample for 0.5 seconds")
                next_poll += 0.01
                time.sleep(max(0.0, next_poll - time.monotonic()))
        item["restore_status"] = device.restore_status
        item["summary"] = summarize(samples)
        return item, samples
    finally:
        device.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--motor-power-off", action="store_true")
    parser.add_argument("--body-supported", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    plan = {"ranges_g": [2, 4, 2], "capture_seconds_each": 4,
            "settle_seconds_each": 1, "can_opened": False,
            "motor_commands_sent": False, "calibration_applied": False,
            "approved_for_runtime": False}
    if not args.execute:
        print(json.dumps({"status": "PLAN_ONLY", "plan": plan}, indent=2))
        return 0
    if not args.motor_power_off or not args.body_supported:
        parser.error("real IMU range comparison requires motor power off and body support")
    output = args.output.expanduser().resolve()
    if any((path / ".git").exists() for path in (output, *output.parents)):
        parser.error("capture output must be outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    result = {"status": "INCOMPLETE", "plan": plan,
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "sessions": [], "errors": [], "approved_for_runtime": False,
              "source_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (Path(__file__), Path(__file__).with_name("imu.py"))}}
    try:
        with open("/tmp/singularitydog-imu-i2c7-68.lock", "a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with (output / "samples.jsonl").open("x") as stream:
                for range_g in (2, 4, 2):
                    item, samples = capture_range(range_g)
                    result["sessions"].append(item)
                    for sample in samples:
                        stream.write(json.dumps({"range_g": range_g, **sample},
                                                allow_nan=False) + "\n")
                    stream.flush()
        result.update(evaluate(result["sessions"]))
    except BaseException as error:
        result["errors"].append(repr(error))
    result["completed_at"] = datetime.datetime.now().astimezone().isoformat()
    (output / "report.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": result["status"], "checks": result.get("checks"),
                      "raw_norm_deviation_percent": result.get("raw_norm_deviation_percent"),
                      "output": str(output), "errors": result["errors"]}, indent=2))
    return 0 if result["status"].startswith("CONFIG_") else 1


if __name__ == "__main__":
    raise SystemExit(main())
