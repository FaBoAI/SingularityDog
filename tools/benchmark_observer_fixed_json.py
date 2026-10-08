#!/usr/bin/env python3
"""Benchmark owned fixed-provenance copies from saved raw telemetry only.

No device, network, model weights, inference or motor command is opened. Each
saved snapshot's original canonical SHA is independently reconstructed before
timing four static copy categories. This is a local component benchmark, not a
new acquisition/20ms/active-output qualification.
"""
import argparse
import hashlib
import json
import marshal
from pathlib import Path
import statistics
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"runtime"))
from singularitydog_hw import policy_observer as observer
from singularitydog_hw.native_diagnostic_transport import Record
from singularitydog_hw import native_pipeline_benchmark as pipeline


COPY_KEYS = ("calibration_source_flags", "imu_mount_candidate", "gyro_bias_hypothesis",
             "accel_input_hypothesis", "reviewed_accel_calibration")


def tree_bits(value):
    kind = type(value)
    if kind is dict:
        return "dict", tuple((key, tree_bits(child)) for key, child in value.items())
    if kind in (list, tuple):
        return kind.__name__, tuple(tree_bits(child) for child in value)
    if kind is float:
        return "float", struct.pack(">d", value)
    return kind.__name__, value


def mutable_nodes(value):
    result = set()
    if type(value) in (dict, list, tuple):
        if type(value) in (dict, list):
            result.add(id(value))
        for child in value.values() if type(value) is dict else value:
            result.update(mutable_nodes(child))
    return result


def restore_snapshot(row):
    records = {}
    for scope in ("front", "rear"):
        saved = row["acquired"][scope]["records"]
        if type(saved) is not list or len(saved) != 6:
            raise ValueError("Exactly six saved feedback records per bus required")
        owned = (Record*6)()
        for source, target in zip(saved, owned):
            for key in ("start_ns", "finish_ns", "read_start_ns", "received_ns", "deadline_ns", "written", "received"):
                value = source[key]
                maximum = 2**32 if key in ("written", "received") else 2**64
                if type(value) is not int or not 0 <= value < maximum:
                    raise ValueError("Invalid saved native field: "+key)
                setattr(target, key, value)
            tx, rx = bytes.fromhex(source["tx_hex"]), bytes.fromhex(source["rx_hex"])
            if len(tx) != 17 or len(rx) != 17:
                raise ValueError("Complete saved AT frames required")
            target.tx[:], target.rx[:] = tx, rx
        records[scope] = owned
    observed = row["observed"]
    snapshot = pipeline.snapshot_from_records(records, row["imu"], observed["tick_ns"])
    snapshot["source_flags"]["v3_voltage_overlap_pending_at_inference"] = True
    if observer._digest(snapshot) != observed["provenance"]["snapshot_canonical_json_sha256"]:
        raise ValueError("Reconstructed saved snapshot canonical SHA differs")
    return snapshot


def benchmark_copies(provenance_rows, *, iterations, trials):
    """Interleave old/new copy trials after all fixed-record equality checks."""
    if type(iterations) is not int or not 1 <= iterations <= 100_000 or type(trials) is not int or not 1 <= trials <= 31:
        raise ValueError("Bounded positive benchmark counts required")
    if not provenance_rows:
        raise ValueError("Saved provenance required")
    results = {}
    for name in COPY_KEYS:
        present = [row[name] for row in provenance_rows if name in row]
        if not present:
            continue
        if len(present) != len(provenance_rows):
            raise ValueError("Provenance selection changed within saved run")
        owned = observer.snapshot_event(present[0])
        blob = marshal.dumps(owned)
        factory = observer._frozen_json_literal_factory(owned)
        reference = lambda: marshal.loads(blob)
        candidate = reference if factory is None else factory
        signature = tree_bits(owned)
        previous = None
        for value in present:
            if tree_bits(observer.snapshot_event(value)) != signature:
                raise ValueError("Fixed provenance changed within saved run: "+name)
            fresh = candidate()
            if tree_bits(fresh) != signature:
                raise ValueError("Literal copy changed type, order or float bits")
            if mutable_nodes(fresh) & (mutable_nodes(value) | mutable_nodes(owned)):
                raise ValueError("Literal copy retained a mutable source alias")
            if previous is not None and mutable_nodes(fresh) & mutable_nodes(previous):
                raise ValueError("Literal copy retained a prior-record alias")
            previous = fresh
        measured = {"marshal": [], "literal_factory": []}
        measurement_order = []
        for trial in range(trials):
            methods = (("marshal", reference), ("literal_factory", candidate))
            if trial % 2:
                methods = methods[::-1]
            measurement_order.append([name for name, _ in methods])
            for label, copier in methods:
                started = time.perf_counter_ns()
                for _ in range(iterations):
                    copier()
                measured[label].append((time.perf_counter_ns()-started)/iterations)
        results[name] = {
            "factory_selected": factory is not None, "records_exact_checked": len(present),
            "canonical_json_sha256": observer._digest(owned), "marshal_byte_count": len(blob),
            "ns_per_copy_trials": measured, "measurement_order": measurement_order,
            "median_ns_per_copy": {k: statistics.median(v) for k, v in measured.items()}}
    return results


def run(records_path, output_path, *, iterations=2000, trials=7):
    records_path, output_path = Path(records_path).resolve(strict=True), Path(output_path).resolve()
    if output_path.exists() or not output_path.parent.exists() or any((p/".git").exists() for p in output_path.parents):
        raise ValueError("Fresh output in an existing private directory outside Git required")
    if not 0 < records_path.stat().st_size <= 256*1024*1024:
        raise ValueError("Bounded saved records file required")
    with records_path.open("rb") as handle:
        raw = handle.read(256*1024*1024+1)
    if len(raw) > 256*1024*1024:
        raise ValueError("Saved records grew beyond the byte bound")
    sources = {"policy_observer.py": Path(observer.__file__),
               "native_pipeline_benchmark.py": Path(pipeline.__file__),
               "benchmark_observer_fixed_json.py": Path(__file__)}
    source_pins = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}
    rows = json.loads(raw)
    if type(rows) is not list or not 1 <= len(rows) <= 30_000:
        raise ValueError("Bounded saved record list required")
    for row in rows:
        restore_snapshot(row)
    copies = benchmark_copies([row["observed"]["provenance"] for row in rows],
                              iterations=iterations, trials=trials)
    sections = {}
    profiles = [row["observed"].get("consume_profile") for row in rows]
    if all(type(profile) is dict and profile.get("complete") is True for profile in profiles):
        for stage in profiles[0]["durations_ns"]:
            values = sorted(profile["durations_ns"][stage] for profile in profiles)
            sections[stage] = {"median_ns": statistics.median(values),
                               "p95_ns": values[int((len(values)-1)*.95)], "max_ns": values[-1]}
    if source_pins != {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}:
        raise ValueError("Benchmark sources changed during measurement")
    report = {
        "status": "OFFLINE_FIXED_JSON_COPY_BENCHMARK_COMPLETE", "hardware_opened": False,
        "output_allowed": False, "active_output_qualified": False, "new_20ms_run_measured": False,
        "scope": "local CPU static copy components; saved snapshot/provenance equality only",
        "saved_records_sha256": hashlib.sha256(raw).hexdigest(),
        "saved_snapshot_sha_exact_count": len(rows), "iterations_per_trial": iterations, "trials": trials,
        "source_sha256": source_pins, "sources_unchanged_during_measurement": True,
        "copies": copies, "original_saved_consume_sections": sections}
    with output_path.open("x") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--trials", type=int, default=7)
    args = parser.parse_args()
    report = run(args.records, args.output, iterations=args.iterations, trials=args.trials)
    print(json.dumps({k: report[k] for k in ("status", "hardware_opened", "output_allowed",
                                           "saved_snapshot_sha_exact_count")}))


if __name__ == "__main__":
    main()
