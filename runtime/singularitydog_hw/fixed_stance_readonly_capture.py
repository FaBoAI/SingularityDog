"""Capture one operator-held, twelve-axis raw pose without motor output.

Only Type 0 identity and Type 17 parameter reads are sent, through ReadOnlyCAN.
The operator must hold all four legs in the *same* pose throughout the capture;
sequential one-leg poses cannot be combined into a full-body stance target.
Read-only replies cannot prove the present STOP/enable state, physical clearance,
model joint zero/sign, load bearing, or self-standing. The output is a private
draft for later human review, never a motor-command authorization.
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

from .can_readonly import ReadOnlyCAN
from .can_timing_probe import ownership_locks, validate_uids
from . import dual_can_pipeline_benchmark as dual

IDS_BY_BUS = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}
PARAMETERS = ("position", "velocity", "current", "voltage", "run_mode")
SWEEPS = 3
MAX_CAPTURE_S = 15.
STABILITY_SPAN_RAD = .02  # Reporting heuristic, not a mechanical limit.
MAX_ABS_VELOCITY_RAD_S = .1
SCHEMA = "singularitydog.fixed-stance-readonly-capture.v1"


def _require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def _checked_query(can, bus, motor_id, parameter, check):
    _require(motor_id in IDS_BY_BUS[bus], f"ID{motor_id} assigned to wrong bus")
    check()
    result = can.query(motor_id, parameter)
    _require(type(result) is dict and result.get("ok") is True,
             f"ID{motor_id} {parameter or 'identity'} read failed")
    requested, received = result.get("request_monotonic_ns"), result.get("monotonic_ns")
    _require(type(requested) is int and type(received) is int and
             0 < requested < received, f"ID{motor_id} has no fresh reply timestamp")
    _require(not can.parser.discarded_bytes and not can.parser.buffer,
             f"{bus} parser has discarded or partial bytes")
    if parameter is None:
        uid = result.get("mcu_uid_hex")
        _require(type(uid) is str and len(uid) == 16 and
                 all(ch in "0123456789abcdef" for ch in uid),
                 f"ID{motor_id} malformed UID")
    else:
        value = result.get("value")
        _require(type(value) in (int, float) and math.isfinite(value),
                 f"ID{motor_id} {parameter} nonfinite value")
        if parameter == "position":
            _require(result.get("unit") == "rad_output_shaft" and
                     result.get("index") == 0x7019,
                     f"ID{motor_id} position is not raw output-shaft radians")
    check()
    return result


def capture_identities(cans, expected, check):
    """Read all twelve UIDs with a separate serial owner per bus."""
    _require(type(cans) is dict and set(cans) == set(IDS_BY_BUS),
             "Two separate bus sessions are required")
    expected = validate_uids(expected)
    _require(cans["front"] is not cans["rear"], "Buses must not share a CAN owner")

    abort = threading.Event()

    def work(bus):
        rows = {}
        try:
            for mid in IDS_BY_BUS[bus]:
                _require(not abort.is_set(), "Peer bus identity read failed")
                result = _checked_query(cans[bus], bus, mid, None, check)
                _require(result["mcu_uid_hex"] == expected[mid], f"ID{mid} UID mismatch")
                rows[str(mid)] = {"mcu_uid_hex": result["mcu_uid_hex"],
                                  "request_monotonic_ns": result["request_monotonic_ns"],
                                  "reply_monotonic_ns": result["monotonic_ns"]}
        except BaseException:
            abort.set()
            raise
        return rows

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {bus: pool.submit(work, bus) for bus in IDS_BY_BUS}
        identities = {key: row for future in futures.values()
                      for key, row in future.result().items()}
    _require(set(identities) == {str(i) for i in range(1, 13)} and
             len({row["mcu_uid_hex"] for row in identities.values()}) == 12,
             "Twelve distinct fresh motor identities are required")
    return identities


def capture_pose(cans, check, *, enter_monotonic_ns, sweeps=SWEEPS):
    """Three fresh read sweeps per motor after the operator's Enter timestamp."""
    _require(type(sweeps) is int and 3 <= sweeps <= 5,
             "Pose capture needs three to five sweeps")
    _require(type(enter_monotonic_ns) is int and enter_monotonic_ns > 0,
             "A real operator Enter timestamp is required")
    _require(type(cans) is dict and set(cans) == set(IDS_BY_BUS),
             "Two separate bus sessions are required")
    started = time.monotonic_ns()

    abort = threading.Event()

    def work(bus):
        rows = {str(mid): [] for mid in IDS_BY_BUS[bus]}
        try:
            for _ in range(sweeps):
                for mid in IDS_BY_BUS[bus]:
                    sample = {}
                    for parameter in PARAMETERS:
                        _require(not abort.is_set(), "Peer bus pose read failed")
                        reply = _checked_query(cans[bus], bus, mid, parameter, check)
                        _require(reply["request_monotonic_ns"] >= enter_monotonic_ns,
                                 f"ID{mid} reply predates operator confirmation")
                        sample[parameter] = {"value": reply["value"],
                                             "request_monotonic_ns": reply["request_monotonic_ns"],
                                             "reply_monotonic_ns": reply["monotonic_ns"]}
                    rows[str(mid)].append(sample)
        except BaseException:
            abort.set()
            raise
        return rows

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {bus: pool.submit(work, bus) for bus in IDS_BY_BUS}
        samples = {key: row for future in futures.values()
                   for key, row in future.result().items()}
    ended = time.monotonic_ns()
    _require(ended - started <= MAX_CAPTURE_S * 1e9,
             "Full-body pose sampling exceeded the 15-second bound")
    _require(set(samples) == {str(i) for i in range(1, 13)},
             "All twelve fresh position groups are required")
    raw = {}
    issues = []
    for mid in map(str, range(1, 13)):
        values = [row["position"]["value"] for row in samples[mid]]
        velocities = [abs(row["velocity"]["value"]) for row in samples[mid]]
        span = max(values) - min(values)
        raw[mid] = statistics.median(values)
        if span > STABILITY_SPAN_RAD or max(velocities) > MAX_ABS_VELOCITY_RAD_S:
            issues.append({"motor_id": int(mid), "position_span_rad": span,
                           "maximum_abs_velocity_rad_s": max(velocities)})
    return {"samples": samples, "raw_rad_by_id": raw,
            "started_monotonic_ns": started, "ended_monotonic_ns": ended,
            "capture_span_ms": (ended-started)/1e6,
            "sampling_issues": issues,
            "stationarity_verified": False,
            "sampling_stability_heuristic_passed": not issues}


def draft_manifest(summary, summary_sha256):
    """Carry the numbers forward without making any physical review assertion."""
    _require(summary.get("status") == "RECORDED_REVIEW_REQUIRED" and
             type(summary.get("pose")) is dict, "A complete capture is required")
    return {"schema": "singularitydog.fixed-stance-capture-draft.v1",
            "boot_id": summary["boot_id"],
            "motor_uids": {mid: row["mcu_uid_hex"]
                           for mid, row in summary["identities"].items()},
            "raw_rad_by_id": summary["pose"]["raw_rad_by_id"],
            "stance_capture_sha256": summary_sha256,
            "capture_span_ms": summary["pose"]["capture_span_ms"],
            "sampling_issues": summary["pose"]["sampling_issues"],
            "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
            "simultaneous_physical_stance_verified": False,
            "all_segment_sweeps_physically_reviewed": False,
            "front_upper_leg_carbon_clamp_clearance_verified": False,
            "same_boot_12_axis_hold_passed": False,
            "output_allowed": False,
            "approved_for_runtime": False,
            "learned_policy_allowed": False,
            "self_supported_standing_verified": False,
            "limitations": [
                "Operator Enter is a timing marker, not proof of a simultaneous four-leg pose.",
                "Read-only Type0/17 replies do not confirm current motor STOP/enable state.",
                "The raw shaft positions are not calibrated model joint angles.",
                "Sequential one-leg captures must not be merged into this full-body target.",
                "Physical swept-path clearance and supported load testing remain separate.",
            ]}


def _read_boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def _check_output_path(parser, output):
    path = output.expanduser().resolve()
    if any((parent / ".git").exists() for parent in (path, *path.parents)):
        parser.error("Private UID/pose logs must be outside Git")
    if path.exists():
        parser.error("Output directory already exists")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--front-port", required=True)
    parser.add_argument("--rear-port", required=True)
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute-readonly", action="store_true")
    args = parser.parse_args(argv)
    try:
        expected_source = args.expected_uids.read_bytes()
        expected = validate_uids(json.loads(expected_source))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    plan = {"schema": SCHEMA, "ids_by_bus": {bus: list(ids)
            for bus, ids in IDS_BY_BUS.items()}, "sweeps": SWEEPS,
            "parameters": ["identity", *PARAMETERS],
            "allowed_can_types": [0, 17], "automatic_retry": False,
            "output_allowed": False, "motor_output_available": False,
            "stop_command_available": False, "stop_state_verifiable": False,
            "physical_stance_verifiable": False,
            "ports": {"front": args.front_port, "rear": args.rear_port}}
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2))
        return 0
    if not sys.stdin.isatty():
        parser.error("Run interactively on Jetson; piped Enter is not accepted")
    output = _check_output_path(parser, args.output)
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    boot = _read_boot_id()
    result = {"schema": SCHEMA, "status": "INCOMPLETE", "boot_id": boot,
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "plan": plan, "expected_uids_sha256": hashlib.sha256(expected_source).hexdigest(),
              "source_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                 for path in (Path(__file__), Path(__file__).with_name("can_readonly.py"))},
              "identities": {}, "pose": None, "errors": [],
              "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
              "output_allowed": False, "approved_for_runtime": False}
    cancel = threading.Event()
    old_handlers = {}

    def check():
        if cancel.is_set():
            raise InterruptedError("Capture cancelled")
        if _read_boot_id() != boot:
            raise RuntimeError("Jetson boot changed during capture")
        for binding in bindings.values():
            if not dual.binding_matches(binding):
                raise RuntimeError("USB2CAN port binding changed")

    def interrupted(signum, _frame):
        cancel.set()
        raise InterruptedError(f"Signal {signum} cancelled capture")

    event_lock = threading.Lock()
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, interrupted)
        with ExitStack() as stack:
            stack.enter_context(ownership_locks())
            for bus in IDS_BY_BUS:
                stack.enter_context(dual.port_lock(bindings[bus]["resolved"]))
            log = stack.enter_context((output / "events.jsonl").open("x", buffering=1))

            def emit(bus, event):
                source = event.get("source_id") if event.get("kind") == "can_rx_frame" else None
                if type(source) is int and 1 <= source <= 12 and source not in IDS_BY_BUS[bus]:
                    raise RuntimeError(f"ID{source} replied on {bus} bus")
                if event.get("kind") == "motor_feedback" and (
                        event.get("type") == 21 or event.get("fault_bits", 0) or
                        event.get("mode_state", 0) == 2):
                    raise RuntimeError("Enabled or fault feedback during read-only capture")
                with event_lock:
                    log.write(json.dumps({"wall_time_ns": time.time_ns(),
                                          "bus": bus, **event}, allow_nan=False) + "\n")

            cans = {}
            for bus in IDS_BY_BUS:
                can = stack.enter_context(ReadOnlyCAN(
                    port=bindings[bus]["path"], event_sink=lambda event, b=bus: emit(b, event)))
                _require(os.fstat(can.serial.fileno()).st_rdev == bindings[bus]["st_rdev"],
                         f"{bus} opened USB2CAN differs from verified binding")
                cans[bus] = can
            result["identities"] = capture_identities(cans, expected, check)
            print("12軸UID確認完了。四脚を同時に目標姿勢で保持してください。", flush=True)
            print("1脚ずつ順番に姿勢を作る場合は、この全身立位記録を使わず q で中止してください。", flush=True)
            print("モーターは脱力し、別の駆動ツールを停止してください。読取りだけでSTOP状態は証明できません。", flush=True)
            answer = input("四脚を同時に保持できたら Enter、無理なら q: ").strip().lower()
            if answer != "":
                raise InterruptedError("Operator did not confirm a single held full-body pose")
            enter_ns = time.monotonic_ns()
            check()
            result["operator_enter_monotonic_ns"] = enter_ns
            result["pose"] = capture_pose(cans, check, enter_monotonic_ns=enter_ns)
            for can in cans.values():
                _require(not can.parser.discarded_bytes and not can.parser.buffer,
                         "CAN parser contains discarded or incomplete bytes")
            check()
            result["status"] = "RECORDED_REVIEW_REQUIRED"
    except BaseException as error:
        result["status"] = "INCOMPLETE"
        result["errors"].append(repr(error))
    finally:
        for signum, old in old_handlers.items():
            signal.signal(signum, old)
    result["completed_at"] = datetime.datetime.now().astimezone().isoformat()
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    if result["status"] == "RECORDED_REVIEW_REQUIRED" and not result["errors"]:
        draft = draft_manifest(result, hashlib.sha256(summary_path.read_bytes()).hexdigest())
        (output / "capture-draft.json").write_text(
            json.dumps(draft, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(output), "status": result["status"],
                      "errors": result["errors"],
                      "stop_state": result["stop_state"]}, ensure_ascii=False, indent=2))
    return int(bool(result["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
