"""Capture all twelve raw motor angles in the current boot, without output.

The record is private telemetry for a later power-epoch review. It does not
identify a motor power epoch, verify STOP, or authorize motor actuation.
Only canonical Type0 UID and Type17 parameter requests can reach the buses.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import threading

from . import dual_can_pipeline_benchmark as dual
from .can_readonly import ReadOnlyCAN
from .can_timing_probe import ownership_locks, validate_uids
from .fixed_stance_readonly_capture import IDS_BY_BUS, capture_identities
from .post_charge_box_pose_check import (
    _bad_constant, _boot_id, _strict_pairs, _write_private,
    collect_telemetry, guard_event, private_output_path,
)


SCHEMA = "singularitydog.readonly-12-angle-capture.v1"


def _expected_uids(path):
    source = Path(path).read_bytes()
    expected = validate_uids(json.loads(source, object_pairs_hook=_strict_pairs,
                                        parse_constant=_bad_constant))
    return expected, hashlib.sha256(source).hexdigest()


def _check_quiet(telemetry):
    """Check the observed quiet values, without calling them a STOP proof."""
    rows = telemetry["rows"]
    if set(rows) != {str(i) for i in range(1, 13)}:
        raise RuntimeError("Exactly twelve telemetry rows required")
    for mid, row in rows.items():
        if row.get("run_mode") != 0 or row.get("current") != 0:
            raise RuntimeError(f"ID{mid} was not observed quiet")
        if row.get("position_span_deg", float("inf")) > 0.1:
            raise RuntimeError(f"ID{mid} position was not static")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--front-port", required=True)
    ap.add_argument("--rear-port", required=True)
    ap.add_argument("--expected-uids", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--execute-readonly", action="store_true")
    args = ap.parse_args(argv)
    try:
        expected, expected_hash = _expected_uids(args.expected_uids)
        output = private_output_path(args.output)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        ap.error(str(error))
    plan = {"allowed_can_types": [0, 17], "automatic_retry": False,
            "motor_output_available": False,
            "ports": {"front": args.front_port, "rear": args.rear_port},
            "ids_by_bus": {bus: list(ids) for bus, ids in IDS_BY_BUS.items()},
            "parameters": ["identity", "run_mode", "current", "voltage", "position x3"]}
    if not args.execute_readonly:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    try:
        bindings = dual.validate_ports(args.front_port, args.rear_port)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    boot = _boot_id()
    result = {"schema": SCHEMA, "status": "INCOMPLETE", "errors": [],
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "boot_id": boot, "expected_uids_sha256": expected_hash,
              "plan": plan, "identities": None, "telemetry": None,
              "angle_wrap_applied": False,
              "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
              "motor_output_allowed": False, "approved_for_runtime": False,
              "motor_power_epoch": "NOT_INFERRED_FROM_JETSON_BOOT"}
    cancel = threading.Event()
    old_handlers = {}

    def check():
        if cancel.is_set():
            raise InterruptedError("Read-only capture interrupted")
        if _boot_id() != boot:
            raise RuntimeError("Jetson boot changed during capture")
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
            result["identities"] = capture_identities(cans, expected, check)
            check()
            result["telemetry"] = collect_telemetry(cans, check)
            for can in cans.values():
                if can.parser.discarded_bytes or can.parser.buffer:
                    raise RuntimeError("CAN parser has discarded or partial bytes")
            check()
            _check_quiet(result["telemetry"])
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
