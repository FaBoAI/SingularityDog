"""Estimate the host-report age Type24 would need in a saved STOP-proxy run.

This is file-only counterfactual arithmetic. It holds the measured time after
``gather_end_ns`` fixed and substitutes a hypothetical age for the oldest
Type24 host report at that point. It does not contain a Type24 stream, establish
the motor's internal sample time, or prove that acquisition can be skipped.
"""

import argparse
import hashlib
import json
from pathlib import Path
import statistics


PERIOD_MS = 20.0
AGE_CANDIDATES_MS = (6, 7, 8, 9, 10)
TARGETS = {
    "final_host_write": "final_host_write_ns",
    "last_stop_reply": "last_proxy_reply_ns",
    "oldest_report_to_cycle_end": "cycle_end_ns",
}
TIMESTAMP_ORDER = (
    "oldest_input_start_ns", "input_latest_reply_ns", "gather_end_ns",
    "prepare_end_ns", "infer_end_ns", "final_host_write_ns",
    "last_proxy_reply_ns", "cycle_end_ns",
)


def _distribution(values):
    ordered = sorted(values)
    return {
        "minimum": min(values),
        "p01": ordered[max(0, int(0.01 * (len(ordered) - 1)))],
        "median": statistics.median(values),
        "maximum": max(values),
    }


def summarize(report):
    if (not isinstance(report, dict) or report.get("status") != "COMPLETE_DIAGNOSTIC"
            or report.get("mode") != "stop-proxy"
            or report.get("motor_enable_sent") is not False
            or report.get("learned_targets_sent") is not False):
        raise ValueError("Complete disabled STOP-proxy report required")
    rows = report.get("measurements")
    if (not isinstance(rows, list) or not rows
            or type(report.get("cycles_completed")) is not int
            or len(rows) != report["cycles_completed"]):
        raise ValueError("Completed cycle count mismatch")
    if not isinstance(report.get("identities"), dict) or set(report["identities"]) != {"front", "rear"}:
        raise ValueError("Both motor identity groups required")

    acquisition_ms = []
    age_budgets = {name: [] for name in TARGETS}
    measured_misses = {
        "oldest_input_to_final_host_write": 0,
        "oldest_input_to_last_stop_reply": 0,
        "oldest_input_to_cycle_end": 0,
        "release_to_cycle_end": 0,
    }
    measured_keys = {
        "final_host_write": "oldest_input_to_final_host_write",
        "last_stop_reply": "oldest_input_to_last_stop_reply",
        "oldest_report_to_cycle_end": "oldest_input_to_cycle_end",
    }
    for row in rows:
        if (not isinstance(row, dict) or any(type(row.get(key)) is not int for key in TIMESTAMP_ORDER)
                or type(row.get("release_ns")) is not int):
            raise ValueError("Missing cycle timestamps")
        times = [row[key] for key in TIMESTAMP_ORDER]
        if (any(a > b for a, b in zip(times, times[1:])) or times[0] <= 0
                or not 0 < row["release_ns"] <= times[0]):
            raise ValueError("Noncausal cycle timestamps")
        beginning, gathered = row["oldest_input_start_ns"], row["gather_end_ns"]
        acquisition_ms.append((gathered - beginning) / 1e6)
        for name, key in TARGETS.items():
            age_budgets[name].append(PERIOD_MS - (row[key] - gathered) / 1e6)
            measured_misses[measured_keys[name]] += (row[key] - beginning) / 1e6 > PERIOD_MS
        measured_misses["release_to_cycle_end"] += (
            row["cycle_end_ns"] - row["release_ns"]) / 1e6 > PERIOD_MS

    if (measured_misses["oldest_input_to_final_host_write"] != report.get("host_deadline_misses")
            or measured_misses["release_to_cycle_end"] != report.get("iteration_deadline_misses")):
        raise ValueError("Measured deadline counts disagree with report")

    return {
        "scope": "saved disabled STOP-proxy timing; hypothetical Type24 host-report age only",
        "cycles": len(rows),
        "baseline_acquisition_ms": _distribution(acquisition_ms),
        "measured_stop_proxy_deadline_misses": measured_misses,
        "maximum_oldest_report_age_at_gather_end_ms": {
            name: _distribution(values) for name, values in age_budgets.items()
        },
        "hypothetical_deadline_misses_at_fixed_oldest_report_age_ms": {
            name: {str(age): sum(age > budget for budget in values)
                   for age in AGE_CANDIDATES_MS}
            for name, values in age_budgets.items()
        },
        "type24_stream_measured": False,
        "per_motor_type24_freshness_verified": False,
        "concurrent_reporting_effect_on_stop_reply_measured": False,
        "motor_internal_sample_time_known": False,
        "age_budget_is_measured_saving": False,
        "type24_full_cycle_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    raw = args.report.read_bytes()
    result = summarize(json.loads(raw))
    result["source_report_sha256"] = hashlib.sha256(raw).hexdigest()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
