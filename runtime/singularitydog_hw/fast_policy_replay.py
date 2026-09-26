"""Compare Type2 assembly on saved inputs, without hardware or model loading.

Example (paths are supplied explicitly; no device discovery is performed)::

    python -m singularitydog_hw.fast_policy_replay --baseline-adapter candidate_snapshot.py \
        --capture captures/01 --capture captures/02 --capture captures/03 --repeats 100

Files are loaded once. Every iteration uses the original recorded tick and
source timestamps. Preparation and preparation+assembly are both reported so
the hot prepared-record timing cannot be mistaken for complete live work.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import time
import types

from .fast_policy_inputs import prepare_cycle


def load_baseline(path):
    """Load the explicitly selected pure adapter without writing its pycache."""
    source = Path(path).read_bytes()
    module = types.ModuleType("fast_policy_selected_offline_baseline")
    module.__file__ = str(Path(path).resolve())
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    if not callable(getattr(module, "build_snapshot", None)):
        raise ValueError("Baseline must expose build_snapshot")
    return module.build_snapshot, hashlib.sha256(source).hexdigest()


def load_capture(directory):
    root = Path(directory)
    report = json.loads((root / "report.json").read_text())
    if report.get("status") != "COMPLETE_INTEGRATED_STOP_PROXY":
        raise ValueError("Require a completed integrated STOP proxy capture")
    rows = []
    for name in ("front", "rear"):
        for text in (root / (name + ".jsonl")).read_text().splitlines():
            row = json.loads(text)
            if row.get("kind") == "pipeline_reply" and row.get("cycle") == 1:
                rows.append(row)
    snapshot = report["snapshot"]
    return rows, report["imu_sample"], snapshot["tick_ns"], snapshot


def _stats(values):
    ordered = sorted(values)
    return {"median_ms": statistics.median(ordered),
            "p95_ms": ordered[max(0, (95*len(ordered)+99)//100-1)],
            "min_ms": ordered[0], "max_ms": ordered[-1], "samples": len(ordered)}


def benchmark_capture(data, baseline, *, repeats=100, warmup=10):
    if type(repeats) is not int or not 1 <= repeats <= 10_000:
        raise ValueError("repeats must be 1..10000")
    if type(warmup) is not int or not 0 <= warmup <= 100:
        raise ValueError("warmup must be 0..100")
    rows, imu, tick, recorded = data
    prepared = prepare_cycle(rows, imu)
    expected = baseline(rows, imu, tick, cycle=1)
    if prepared.snapshot(tick) != expected or expected != recorded:
        raise ValueError("Prepared/baseline/recorded snapshots differ")
    operations = {
        "baseline_assembly": lambda: baseline(rows, imu, tick, cycle=1),
        "prepare_immutable_cycle": lambda: prepare_cycle(rows, imu),
        "prepared_assembly_only": lambda: prepared.snapshot(tick),
        "prepare_and_assemble": lambda: prepare_cycle(rows, imu).snapshot(tick),
    }
    for _ in range(warmup):
        for operation in operations.values():
            operation()
    timings = {name: {"wall": [], "thread_cpu": []} for name in operations}
    names = list(operations)
    for iteration in range(repeats):
        # Rotate the first condition instead of always warming one with another.
        order = names[iteration % len(names):] + names[:iteration % len(names)]
        for name in order:
            wall, cpu = time.perf_counter_ns(), time.thread_time_ns()
            value = operations[name]()
            elapsed_cpu, elapsed_wall = time.thread_time_ns()-cpu, time.perf_counter_ns()-wall
            timings[name]["wall"].append(elapsed_wall/1e6)
            timings[name]["thread_cpu"].append(elapsed_cpu/1e6)
            # Equality and reporting deliberately happen outside the timer.
            actual = value.snapshot(tick) if name == "prepare_immutable_cycle" else value
            if actual != expected:
                raise ValueError("Output changed during replay: " + name)
    return {"original_tick_ns": tick, "snapshots_exactly_equal": True,
            "original_source_timestamps_preserved": True,
            "iterations_per_condition": repeats,
            "timings": {name: {kind: _stats(values) for kind, values in parts.items()}
                        for name, parts in timings.items()}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-adapter", required=True, type=Path)
    parser.add_argument("--capture", action="append", required=True, type=Path)
    parser.add_argument("--repeats", default=100, type=int)
    parser.add_argument("--warmup", default=10, type=int)
    args = parser.parse_args(argv)
    baseline, baseline_hash = load_baseline(args.baseline_adapter)
    # Loading and validation of all capture files precede every timed block.
    captures = [load_capture(path) for path in args.capture]
    results = [benchmark_capture(data, baseline, repeats=args.repeats, warmup=args.warmup)
               for data in captures]
    print(json.dumps({"kind": "offline_saved_input_assembly_comparison", "output_allowed": False,
        "execution_platform": {"system": platform.system(), "machine": platform.machine(),
                               "python_version": platform.python_version()},
        "hardware_access": False, "model_inference_measured": False,
        "live_throughput_verified": False, "full_pipeline_20ms_verified": False,
        "original_timestamps_reused_for_cpu_replay_only": True,
        "fresh_sample_count_is_not_replay_iteration_count": True,
        "preparation_required_for_every_new_source_cycle": True,
        "files_loaded_outside_timed_sections": True,
        "baseline_adapter_sha256": baseline_hash, "captures": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
