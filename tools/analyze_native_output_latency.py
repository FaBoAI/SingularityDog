"""Summarize saved native STOP-proxy output timing without opening hardware.

This is a diagnostic of host timestamps. A write returning does not prove that
the corresponding CAN frame has finished on the wire, and a read start does not
identify where a reply waited before reaching the host process.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        raise ValueError("No timing samples")
    return {
        "count": len(ordered),
        "median": statistics.median(ordered),
        "p95": ordered[math.ceil(0.95 * len(ordered)) - 1],
        "p99": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max": ordered[-1],
    }


def summarize(records):
    if not isinstance(records, list) or not records:
        raise ValueError("Expected a nonempty list of completed cycles")
    result = {}
    for bus in ("front", "rear"):
        measures = {
            "first_to_last_write_ms": [],
            "last_write_to_read_start_ms": [],
            "last_write_to_last_reply_ms": [],
            "read_start_to_received_ms": [],
            "between_write_gap_ms": [],
            "write_call_ms": [],
            "reads_per_exchange": [],
            "waits_per_exchange": [],
        }
        for index, cycle in enumerate(records, 1):
            if cycle.get("cycle") != index:
                raise ValueError("Cycle order or number mismatch")
            batch = cycle.get("output", {}).get(bus)
            if not isinstance(batch, dict):
                raise ValueError("Missing output bus")
            frames = batch.get("records")
            if not isinstance(frames, list) or len(frames) != 6:
                raise ValueError("Expected six output frames per bus")
            for frame in frames:
                if frame.get("written") != 17 or frame.get("received") != 17:
                    raise ValueError("Incomplete frame")
                times = tuple(frame.get(name) for name in (
                    "start_ns", "finish_ns", "read_start_ns", "received_ns"
                ))
                if not all(type(value) is int and value > 0 for value in times):
                    raise ValueError("Invalid timestamp")
                if not times[0] <= times[1] <= times[2] <= times[3]:
                    raise ValueError("Noncausal timestamp")
                measures["write_call_ms"].append((times[1] - times[0]) / 1e6)
            ordered = sorted(frames, key=lambda frame: frame["start_ns"])
            last = ordered[-1]
            last_reply = max(frames, key=lambda frame: frame["received_ns"])
            measures["first_to_last_write_ms"].append(
                (last["finish_ns"] - ordered[0]["finish_ns"]) / 1e6
            )
            measures["last_write_to_read_start_ms"].append(
                (last_reply["read_start_ns"] - last["finish_ns"]) / 1e6
            )
            measures["last_write_to_last_reply_ms"].append(
                (last_reply["received_ns"] - last["finish_ns"]) / 1e6
            )
            measures["read_start_to_received_ms"].append(
                (last_reply["received_ns"] - last_reply["read_start_ns"]) / 1e6
            )
            for before, after in zip(ordered, ordered[1:]):
                gap = (after["start_ns"] - before["finish_ns"]) / 1e6
                if gap < 0:
                    raise ValueError("Overlapping output writes")
                measures["between_write_gap_ms"].append(gap)
            stats = batch.get("stats", {})
            for name in ("reads", "waits"):
                value = stats.get(name)
                if type(value) is not int or value < 0:
                    raise ValueError("Invalid native stats")
                measures[f"{name}_per_exchange"].append(value)
        result[bus] = {name: distribution(values) for name, values in measures.items()}
    return {"cycles": len(records), "buses": result,
            "timestamp_scope": "host write return and host read; not CAN wire completion"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    raw = args.records.read_bytes()
    report = summarize(json.loads(raw))
    report["source_records_sha256"] = hashlib.sha256(raw).hexdigest()
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        if args.output.exists():
            parser.error("Refusing to overwrite an existing output")
        args.output.write_text(rendered)
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
