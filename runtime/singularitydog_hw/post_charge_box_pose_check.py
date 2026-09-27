"""One bounded, read-only comparison after charging the 40 V motor supply.

Only canonical Type0 identity and Type17 parameter reads are available here.
The result is private raw-shaft telemetry for review, not a lift, a STOP check,
an angle calibration, or permission to enable any motor. No wrap is applied.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import sys
import threading
import time

from . import dual_can_pipeline_benchmark as dual
from .can_readonly import PARAMETERS, ReadOnlyCAN, read_request
from .can_timing_probe import ownership_locks, validate_uids
from .fixed_stance_readonly_capture import IDS_BY_BUS, _checked_query, capture_identities

DEFAULT_BASELINE = Path(
    "/home/jetson/singularitydog-logs/RO-box-pose-after-all-camera-L-20260927.json"
)
BASELINE_STATUS = "READ_ONLY_BOX_AFTER_ALL_CAMERA_L"
SCHEMA = "singularitydog.post-charge-box-pose-check.v1"
READS = ("run_mode", "current", "voltage", "position")
POSITION_SWEEPS = 3
MAX_CAPTURE_SECONDS = 30.0
ALL_IDS = {str(i) for i in range(1, 13)}


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _strict_pairs(pairs):
    value = {}
    for key, item in pairs:
        _require(key not in value, "Duplicate key in private baseline")
        value[key] = item
    return value


def _bad_constant(value):
    raise ValueError(f"Non-finite JSON constant {value}")


def load_baseline(path):
    """Validate the exact twelve-axis box snapshot without publishing its values."""
    source = Path(path).read_bytes()
    data = json.loads(source, object_pairs_hook=_strict_pairs,
                      parse_constant=_bad_constant)
    _require(type(data) is dict and data.get("status") == BASELINE_STATUS
             and data.get("motor_output_allowed") is False,
             "Expected the read-only box pose after all camera L captures")
    boot = data.get("boot_id")
    _require(type(boot) is str and bool(boot.strip()), "Baseline has no boot ID")
    _require(type(data.get("created_ns")) is int and data["created_ns"] > 0,
             "Baseline has no creation timestamp")
    rows = data.get("rows")
    _require(type(rows) is dict and set(rows) == ALL_IDS,
             "Baseline needs exactly IDs 1..12")
    uids = validate_uids({mid: rows[mid].get("uid", "").lower()
                          if type(rows[mid]) is dict and type(rows[mid].get("uid")) is str
                          else None for mid in ALL_IDS})
    raw = {}
    for mid in sorted(ALL_IDS, key=int):
        row = rows[mid]
        _require(type(row.get("run_mode")) is int and row["run_mode"] == 0
                 and type(row.get("current_A")) in (int, float)
                 and row["current_A"] == 0.,
                 f"Baseline ID{mid} was not recorded in quiet mode")
        value, span = row.get("median_position_rad"), row.get("span_deg")
        _require(type(value) in (int, float) and math.isfinite(value)
                 and type(span) in (int, float) and math.isfinite(span)
                 and 0 <= span <= 0.1,
                 f"Baseline ID{mid} has invalid raw position or position span")
        raw[int(mid)] = float(value)
    return {"boot_id": boot, "uids": uids, "raw_rad_by_id": raw,
            "sha256": hashlib.sha256(source).hexdigest()}


def guard_event(bus, event):
    """Reject unexpected writes and contradictory feedback during the read."""
    kind = event.get("kind")
    if kind == "can_tx":
        mid, parameter = event.get("motor_id"), event.get("parameter")
        if type(mid) is not int or mid not in IDS_BY_BUS[bus] or parameter not in (
                "identity", *READS):
            raise RuntimeError("Unexpected CAN transmission")
        expected = read_request(mid, None if parameter == "identity" else parameter)
        if event.get("hex") != expected.hex():
            raise RuntimeError("Noncanonical CAN transmission")
    elif kind == "can_rx_frame":
        source = event.get("source_id")
        if type(source) is int and 1 <= source <= 12 and source not in IDS_BY_BUS[bus]:
            raise RuntimeError("Motor replied on the wrong USB2CAN bus")
    elif kind == "motor_feedback":
        if event.get("type") == 21 or event.get("fault_bits", 0) or event.get("mode_state", 0):
            raise RuntimeError("Enabled or fault feedback during read-only check")


def collect_telemetry(cans, check):
    """Read 3 telemetry parameters and 3 independent positions per motor."""
    _require(type(cans) is dict and set(cans) == set(IDS_BY_BUS)
             and cans["front"] is not cans["rear"], "Two CAN owners are required")
    started = time.monotonic_ns()
    abort = threading.Event()

    def work(bus):
        result = {}
        try:
            for mid in IDS_BY_BUS[bus]:
                row = {}
                for parameter in (*READS[:-1], *("position",) * POSITION_SWEEPS):
                    def checked():
                        if abort.is_set():
                            raise RuntimeError("Peer bus read failed")
                        check()
                    reply = _checked_query(cans[bus], bus, mid, parameter, checked)
                    if (reply.get("index") != PARAMETERS[parameter][0]
                            or reply.get("unit") != PARAMETERS[parameter][2]):
                        raise RuntimeError(f"ID{mid} {parameter} parameter mismatch")
                    if parameter == "position":
                        row.setdefault("position_samples", []).append({
                            "rad": reply["value"],
                            "request_monotonic_ns": reply["request_monotonic_ns"],
                            "reply_monotonic_ns": reply["monotonic_ns"]})
                    else:
                        value = reply["value"]
                        if parameter == "run_mode" and type(value) is not int:
                            raise RuntimeError(f"ID{mid} run_mode is not an integer")
                        row[parameter] = value
                positions = [sample["rad"] for sample in row["position_samples"]]
                row["median_position_rad"] = statistics.median(positions)
                row["position_span_deg"] = math.degrees(max(positions) - min(positions))
                result[str(mid)] = row
        except BaseException:
            abort.set()
            raise
        return result

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {bus: pool.submit(work, bus) for bus in IDS_BY_BUS}
        rows = {mid: row for future in futures.values()
                for mid, row in future.result().items()}
    ended = time.monotonic_ns()
    _require((ended - started) / 1e9 <= MAX_CAPTURE_SECONDS,
             "Post-charge read exceeded its 30-second bound")
    _require(set(rows) == ALL_IDS, "All twelve telemetry rows are required")
    return {"rows": rows, "started_monotonic_ns": started,
            "ended_monotonic_ns": ended, "capture_span_ms": (ended - started) / 1e6}


def compare(capture, baseline):
    """Subtract raw shaft radians directly; even a 360-degree jump is retained."""
    rows = capture["rows"]
    _require(type(rows) is dict and set(rows) == ALL_IDS,
             "Complete twelve-axis telemetry is required")
    result = {}
    for mid in sorted(ALL_IDS, key=int):
        value = rows[mid]["median_position_rad"]
        _require(type(value) in (int, float) and math.isfinite(value),
                 f"ID{mid} position is invalid")
        delta_rad = value - baseline["raw_rad_by_id"][int(mid)]
        _require(math.isfinite(delta_rad), f"ID{mid} direct delta is invalid")
        result[mid] = {"direct_delta_rad": delta_rad,
                       "direct_delta_deg": math.degrees(delta_rad)}
    return result


def private_output_path(path):
    requested = Path(path).expanduser()
    _require(not requested.is_symlink(), "Output must not be a symlink")
    output = requested.resolve()
    _require(output.parent.is_dir(), "Output parent directory does not exist")
    _require(not output.exists() and not output.is_symlink(), "Output already exists")
    _require(not any((parent / ".git").exists()
                     for parent in (output.parent, *output.parents)),
             "Private UID and angle record must be outside Git")
    return output


def _write_private(path, payload):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def _boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    ap.add_argument("--front-port", required=True)
    ap.add_argument("--rear-port", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--execute-readonly", action="store_true")
    args = ap.parse_args(argv)
    try:
        baseline = load_baseline(args.baseline)
        output = private_output_path(args.output)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        ap.error(str(error))
    plan = {"schema": SCHEMA, "baseline_sha256": baseline["sha256"],
            "ports": {"front": args.front_port, "rear": args.rear_port},
            "ids_by_bus": {bus: list(ids) for bus, ids in IDS_BY_BUS.items()},
            "allowed_can_types": [0, 17],
            "parameters": ["identity", *READS[:-1], "position x3"],
            "automatic_retry": False, "motor_output_available": False,
            "max_capture_seconds": MAX_CAPTURE_SECONDS}
    if not args.execute_readonly:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    if _boot_id() != baseline["boot_id"]:
        ap.error("Jetson boot differs from the box-pose baseline")
    try:
        bindings = dual.validate_ports(args.front_port, args.rear_port)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    result = {"schema": SCHEMA, "status": "INCOMPLETE", "errors": [],
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "boot_id": baseline["boot_id"], "baseline_sha256": baseline["sha256"],
              "plan": plan, "identities": None, "telemetry": None,
              "direct_delta_by_id": None, "angle_wrap_applied": False,
              "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
              "motor_output_allowed": False, "approved_for_runtime": False}
    cancel = threading.Event()
    old_handlers = {}

    def check():
        if cancel.is_set():
            raise InterruptedError("Post-charge read interrupted")
        if _boot_id() != baseline["boot_id"]:
            raise RuntimeError("Jetson boot changed during read")
        if any(not dual.binding_matches(binding) for binding in bindings.values()):
            raise RuntimeError("USB2CAN port binding changed")

    def interrupted(_sig, _frame):
        cancel.set()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupted)
        with ExitStack() as stack:
            stack.enter_context(ownership_locks())
            for bus in IDS_BY_BUS:
                stack.enter_context(dual.port_lock(bindings[bus]["resolved"]))
            cans = {}
            for bus in IDS_BY_BUS:
                can = stack.enter_context(ReadOnlyCAN(
                    port=bindings[bus]["path"],
                    event_sink=lambda event, b=bus: guard_event(b, event)))
                if os.fstat(can.serial.fileno()).st_rdev != bindings[bus]["st_rdev"]:
                    raise RuntimeError(f"{bus} opened USB2CAN differs from verified binding")
                cans[bus] = can
            check()
            result["identities"] = capture_identities(cans, baseline["uids"], check)
            check()
            result["telemetry"] = collect_telemetry(cans, check)
            for can in cans.values():
                if can.parser.discarded_bytes or can.parser.buffer:
                    raise RuntimeError("CAN parser has discarded or partial bytes")
            check()
            result["direct_delta_by_id"] = compare(result["telemetry"], baseline)
            result["status"] = "RECORDED_REVIEW_REQUIRED"
    except BaseException as error:
        result["errors"].append(repr(error))
    finally:
        for sig, previous in old_handlers.items():
            signal.signal(sig, previous)
    result["completed_at"] = datetime.datetime.now().astimezone().isoformat()
    try:
        _write_private(output, result)
    except OSError as error:
        print(f"Private output could not be saved: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"output": str(output), "status": result["status"],
                      "errors": result["errors"], "angle_wrap_applied": False,
                      "approved_for_runtime": False}, ensure_ascii=False))
    return 0 if result["status"] == "RECORDED_REVIEW_REQUIRED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
