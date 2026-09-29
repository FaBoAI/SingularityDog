"""Compare saved policy-output timing with 20/25 ms limits, without hardware I/O.

The 25 ms column is a counterfactual threshold comparison of a 20 ms run. It
does not replay the learned controller at 40 Hz or approve a different profile.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import median


PERIODS_MS = (20, 25)
MODEL_DT_MS = 20
REVIEWED_SAMPLE_AGE_MS = 20
REVIEWED_SAMPLE_GAP_MS = 21


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError(f"Invalid {label}")
    return value


def _stamp(value, label):
    if type(value) is not int or value < 0:
        raise ValueError(f"Invalid {label}")
    return value


def _stats(values):
    if not values:
        return None
    return {
        "count": len(values),
        "median_ms": median(values),
        "max_ms": max(values),
        **{f"over_{limit}ms": sum(value > limit for value in values)
           for limit in PERIODS_MS},
    }


def analyze(report):
    """Return measured bounds and explicit blockers; never grant output approval."""
    if type(report) is not dict or type(report.get("cycles")) is not list or not report["cycles"]:
        raise ValueError("Expected a policy-output report with completed cycles")
    whole, output_age, reply_from_begin, starts = [], [], [], []
    previous_begin = None
    for expected_index, row in enumerate(report["cycles"]):
        if type(row) is not dict or type(row.get("index")) is not int or row["index"] != expected_index:
            raise ValueError("Cycles must have ordered zero-based indices")
        begin = _stamp(row.get("begin_ns"), "cycle begin_ns")
        end = _stamp(row.get("end_ns"), "cycle end_ns")
        reply = _stamp(row.get("output_reply_end_ns"), "output_reply_end_ns")
        if not begin <= reply <= end or (previous_begin is not None and begin <= previous_begin):
            raise ValueError("Cycle timestamps are out of order")
        duration = (end - begin) / 1e6
        claimed = row.get("iteration_ms")
        if claimed is not None and abs(_number(claimed, "iteration_ms") - duration) > 0.001:
            raise ValueError("Cycle duration disagrees with begin/end timestamps")
        whole.append(duration)
        output_age.append(_number(row.get("oldest_input_to_final_host_write_ms"),
                                  "oldest_input_to_final_host_write_ms"))
        reply_from_begin.append((reply - begin) / 1e6)
        if previous_begin is not None:
            starts.append((begin - previous_begin) / 1e6)
        previous_begin = begin

    blockers = [
        {
            "code": "PINNED_MODEL_DT_20MS",
            "model_dt_ms": MODEL_DT_MS,
            "detail": "The trained model advances its internal phase, filters and previous action once per 20 ms call; 25 ms calls change real-time behavior.",
            "source": "runtime/experiments/native_policy_overnight/lean_swing_core.py",
        },
        {
            "code": "SUPPORTED_PROFILE_PERIOD_20MS",
            "detail": "The supported profile and output runner require a 20 ms target period; hard_cycle_ms is not a 25 ms period switch.",
            "source": "runtime/singularitydog_hw/policy_live_profile.py",
        },
        {
            "code": "REVIEWED_INPUT_AGE_20MS",
            "reviewed_max_sample_age_ms": REVIEWED_SAMPLE_AGE_MS,
            "detail": "A 25 ms cycle threshold does not extend the independent 20 ms oldest-input age limit.",
            "source": "runtime/singularitydog_hw/policy_live_profile.py",
        },
        {
            "code": "REVIEWED_SAMPLE_GAP_21MS",
            "candidate_gap_ms": 25,
            "reviewed_max_sample_gap_ms": REVIEWED_SAMPLE_GAP_MS,
            "detail": "A true 25 ms command and sample cadence exceeds the current 21 ms gap limit.",
            "source": "runtime/singularitydog_hw/policy_motion_envelope.py",
        },
    ]
    cadence = report.get("telemetry_cadence")
    voltage = report.get("voltage_guard")
    rotation = cadence.get("voltage_rotation_length_cycles") if type(cadence) is dict else None
    maximum_age = voltage.get("maximum_age_ms") if type(voltage) is dict else None
    if type(rotation) is int and rotation > 0 and maximum_age is not None:
        maximum_age = _number(maximum_age, "voltage maximum_age_ms")
        projected = rotation * 25
        blockers.append({
            "code": "VOLTAGE_ROTATION_25MS_EXCEEDS_CACHE_AGE" if projected > maximum_age else "VOLTAGE_ROTATION_REQUIRES_REVIEW",
            "rotation_cycles": rotation,
            "projected_refresh_interval_ms": projected,
            "reviewed_maximum_age_ms": maximum_age,
            "detail": "The saved voltage rotation must be revalidated at 25 ms; nominal refresh spacing alone excludes the current 6-cycle/126 ms setup." if projected > maximum_age else "Voltage freshness at 25 ms is not established by this 20 ms recording.",
            "source": "runtime/singularitydog_hw/policy_output_runtime.py",
        })
    else:
        blockers.append({
            "code": "VOLTAGE_CADENCE_UNKNOWN",
            "detail": "The report lacks voltage rotation or cache-age limits needed to assess 25 ms freshness.",
        })

    return {
        "schema": "singularitydog.offline-25ms-threshold-replay.v1",
        "report_status": report.get("status"),
        "recorded_cycles": len(whole),
        "measured_under_original_20ms_schedule": True,
        "measurements": {
            "begin_to_cycle_end": _stats(whole),
            "oldest_input_to_final_host_write": _stats(output_age),
            "begin_to_last_output_reply": _stats(reply_from_begin),
            "begin_to_next_begin": _stats(starts),
        },
        "reply_age_note": "The report does not contain each cycle's oldest input timestamp; begin-to-reply is a cycle-start measure, not exact oldest-input-to-reply age.",
        "blockers": blockers,
        "actual_25ms_cadence_measured": False,
        "learned_policy_40hz_validated": False,
        "supported_25ms_output_approved": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="Saved policy-output report JSON (read only)")
    args = parser.parse_args(argv)
    raw = args.report.read_bytes()
    result = analyze(json.loads(raw))
    result["input_report_sha256"] = hashlib.sha256(raw).hexdigest()
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
