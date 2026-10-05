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
import stat
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
MAX_TRACE_EVENTS = 4096
MAX_TRACE_BYTES = 2 * 1024 * 1024


def _trace_output_path(path):
    requested = Path(path).expanduser().absolute()
    if any(parent.is_symlink() for parent in (requested, *requested.parents)):
        raise ValueError("Trace path must not contain symlinks")
    return private_output_path(requested)


def _trace_note(primary, secondary):
    if primary is not secondary and hasattr(primary, "add_note"):
        primary.add_note("Event trace also failed: " + repr(secondary))


class _EventTrace:
    """One bounded unbuffered JSONL stream shared by the two CAN owners.

    O_EXCL creates the private destination atomically. Failed/partial traces
    remain available; complete refers to event recording, not capture success.
    """
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.event_count = self.byte_count = self.attempted_event_count = 0
        self.errors = []
        self.failure = None
        self.closed = False
        self.sha256 = None
        flags = os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(path, flags, 0o600)
        try:
            os.fchmod(fd, 0o600)
            self.stream = os.fdopen(fd, "w+b", buffering=0)
        except BaseException as error:
            try:
                os.close(fd)
            except BaseException as secondary:
                _trace_note(error, secondary)
            raise

    def _failed(self, error):
        self.errors.append(repr(error))
        if self.failure is None:
            self.failure = error

    def record(self, bus, event):
        with self.lock:
            self.attempted_event_count += 1
            if self.failure is not None:
                raise self.failure
            try:
                if self.closed or self.event_count >= MAX_TRACE_EVENTS:
                    raise RuntimeError("Event trace closed or exceeded 4096-event budget")
                payload = (json.dumps({**event, "bus": bus}, ensure_ascii=False,
                                      allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")
                if len(payload) > MAX_TRACE_BYTES - self.byte_count:
                    raise RuntimeError("Event trace exceeded 2MB byte budget")
                offset = 0
                while offset < len(payload):
                    written = self.stream.write(payload[offset:])
                    if type(written) is not int or not 0 < written <= len(payload)-offset:
                        raise OSError("Event trace write made no progress")
                    offset += written
                    self.byte_count += written
                self.event_count += 1
            except BaseException as error:
                self._failed(error)
                raise

    def close(self):
        with self.lock:
            if self.closed:
                return self.summary()
            try:
                self.stream.flush()
                os.fsync(self.stream.fileno())
            except BaseException as error:
                self._failed(error)
            try:
                fd = self.stream.fileno()
                raw = os.pread(fd, MAX_TRACE_BYTES + 1, 0)
                opened, named = os.fstat(fd), self.path.lstat()
                if (not stat.S_ISREG(named.st_mode) or
                        (opened.st_dev, opened.st_ino) != (named.st_dev, named.st_ino)):
                    raise RuntimeError("Event trace final path differs from opened file")
                bound = True
                if (len(raw) > MAX_TRACE_BYTES or len(raw) != self.byte_count or
                        raw.count(b"\n") != self.event_count):
                    raise RuntimeError("Event trace final bytes differ from recorded events")
                self.sha256 = hashlib.sha256(raw).hexdigest()
            except BaseException as error:
                self._failed(error)
                # Partial writes still have an exact hash and full-line count.
                if 'bound' in locals() and len(raw) <= MAX_TRACE_BYTES:
                    self.sha256 = hashlib.sha256(raw).hexdigest()
                    self.byte_count, self.event_count = len(raw), raw.count(b"\n")
            finally:
                try:
                    self.stream.close()
                except BaseException as error:
                    self._failed(error)
                self.closed = True
            return self.summary()

    def summary(self):
        complete = self.closed and self.failure is None
        return {"path": str(self.path), "sha256": self.sha256,
                "event_count": self.event_count, "byte_count": self.byte_count,
                "attempted_event_count": self.attempted_event_count,
                "status": "COMPLETE_EVENT_TRACE" if complete else "FAILED_EVENT_TRACE",
                "complete": complete, "errors": list(self.errors)}


def _trace_event(trace, bus, event):
    """Preserve the rejected event too, without replacing a guard exception."""
    try:
        guard_event(bus, event)
    except BaseException as guard_error:
        try:
            trace.record(bus, event)
        except BaseException as trace_error:
            _trace_note(guard_error, trace_error)
        raise
    trace.record(bus, event)


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
    ap.add_argument("--trace-events", type=Path,
                    help="Optional fresh private bounded raw-event JSONL (no automatic retry)")
    ap.add_argument("--execute-readonly", action="store_true")
    args = ap.parse_args(argv)
    try:
        expected, expected_hash = _expected_uids(args.expected_uids)
        output = private_output_path(args.output)
        trace_output = _trace_output_path(args.trace_events) if args.trace_events is not None else None
        if trace_output == output:
            raise ValueError("Trace and capture output must be independent files")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        ap.error(str(error))
    plan = {"allowed_can_types": [0, 17], "automatic_retry": False,
            "motor_output_available": False,
            "ports": {"front": args.front_port, "rear": args.rear_port},
            "ids_by_bus": {bus: list(ids) for bus, ids in IDS_BY_BUS.items()},
            "parameters": ["identity", "run_mode", "current", "voltage", "position x3"]}
    if trace_output is not None:
        plan["trace_events"] = {"path": str(trace_output), "max_events": MAX_TRACE_EVENTS,
                                "max_bytes": MAX_TRACE_BYTES}
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
    trace = None
    capture_error = None
    if trace_output is not None:
        result["trace_events"] = {"path": str(trace_output), "sha256": None,
            "event_count": 0, "byte_count": 0, "attempted_event_count": 0,
            "status": "FAILED_EVENT_TRACE", "complete": False, "errors": []}

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
        if trace_output is not None:
            trace = _EventTrace(trace_output)
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
                    event_sink=(lambda event, b=bus: _trace_event(trace, b, event)) if trace is not None
                    else (lambda event, b=bus: guard_event(b, event))))
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
        capture_error = error
        result["status"] = "INCOMPLETE"
        result["errors"].append(repr(error))
    finally:
        try:
            for sig, previous in old_handlers.items():
                signal.signal(sig, previous)
        finally:
            if trace is not None:
                result["trace_events"] = trace.close()
                if trace.failure is not None:
                    result["status"] = "INCOMPLETE"
                    if capture_error is not None:
                        _trace_note(capture_error, trace.failure)
                    result["errors"].extend("Event trace: " + error for error in trace.errors)
            elif trace_output is not None and capture_error is not None:
                result["trace_events"]["errors"].append(repr(capture_error))
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
