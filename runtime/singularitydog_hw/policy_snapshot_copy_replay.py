"""Compare observer input copies on original saved snapshots; no device/model.

Usage: python -m singularitydog_hw.policy_snapshot_copy_replay --report report.json
Repeat --report for separate captures. Files are loaded once before timing.
The copied snapshots retain their original tick and source times; repeating
them measures CPU copying only and is never counted as new sensor acquisition.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import statistics
import time

from .policy_observer import _snapshot_copy


def _stats(values):
    values = sorted(values)
    return {"samples": len(values), "median_ms": statistics.median(values),
            "p95_ms": values[(95*len(values)+99)//100-1],
            "min_ms": values[0], "max_ms": values[-1]}


def benchmark_snapshot(snapshot, *, repeats=1000, warmup=50):
    if type(repeats) is not int or not 1 <= repeats <= 100_000:
        raise ValueError("repeats must be 1..100000")
    if type(warmup) is not int or not 0 <= warmup <= 1000:
        raise ValueError("warmup must be 0..1000")
    expected = copy.deepcopy(snapshot)
    if _snapshot_copy(snapshot) != expected:
        raise ValueError("Bounded and baseline snapshots differ")
    operations = (("deepcopy", copy.deepcopy), ("bounded_json_copy", _snapshot_copy))
    for _ in range(warmup):
        for _, operation in operations:
            operation(snapshot)
    timings = {name: {"wall": [], "thread_cpu": []} for name, _ in operations}
    for index in range(repeats):
        for name, operation in (operations if index % 2 == 0 else operations[::-1]):
            wall, cpu = time.perf_counter_ns(), time.thread_time_ns()
            actual = operation(snapshot)
            elapsed_cpu = time.thread_time_ns()-cpu
            elapsed_wall = time.perf_counter_ns()-wall
            # Equality checks, JSON and reporting are outside measured copying.
            if actual != expected or snapshot != expected:
                raise ValueError("Snapshot value or timestamp changed during copying")
            timings[name]["wall"].append(elapsed_wall/1e6)
            timings[name]["thread_cpu"].append(elapsed_cpu/1e6)
    canonical = json.dumps(expected, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return {"snapshots_exactly_equal": True, "original_timestamps_preserved": True,
            "snapshot_canonical_json_sha256": hashlib.sha256(canonical.encode()).hexdigest(),
            "timings": {name: {kind: _stats(values) for kind, values in parts.items()}
                        for name, parts in timings.items()}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=50)
    args = parser.parse_args(argv)
    captures = []
    for path in args.report:
        raw = path.read_bytes()
        report = json.loads(raw)
        snapshot = report.get("snapshot")
        if type(snapshot) is not dict or snapshot.get("status") != "DIAGNOSTIC_READY":
            raise ValueError("Report must contain an original diagnostic snapshot")
        captures.append((hashlib.sha256(raw).hexdigest(), snapshot))
    results = [dict(report_sha256=digest,
                    **benchmark_snapshot(snapshot, repeats=args.repeats, warmup=args.warmup))
               for digest, snapshot in captures]
    print(json.dumps({"kind": "offline_policy_snapshot_copy_comparison",
        "execution_platform": {"system": platform.system(), "machine": platform.machine(),
                               "python_version": platform.python_version()},
        "output_allowed": False, "hardware_access": False, "model_inference_measured": False,
        "full_observer_consume_measured": False, "live_throughput_verified": False,
        "full_pipeline_20ms_verified": False, "files_loaded_outside_timed_sections": True,
        "original_timestamps_reused_for_cpu_replay_only": True,
        "fresh_sample_count_is_not_replay_iteration_count": True,
        "captures": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
