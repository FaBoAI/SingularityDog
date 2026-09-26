"""Finite, read-only ID7 position stream for inspecting raw angle continuity.

The only transmitted CAN frames are Type0 identity and Type17 parameter reads.
This does not enable, stop, zero, calibrate, or move a motor. It cannot make a
powered motor safe: run it only while the motor is already disabled.
"""

import argparse
import datetime
import fcntl
import json
import math
from pathlib import Path
import sys
import time

from .can_readonly import ReadOnlyCAN


REAR_PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0"
MAX_SECONDS = 20.0
MIN_SECONDS = 1.0
MAX_SAMPLES = 1000


def validate_plan(port, seconds, expected_uids):
    if port != REAR_PORT:
        raise ValueError(f"ID7 requires the verified rear USB2CAN by-path: {REAR_PORT}")
    if not math.isfinite(seconds) or not MIN_SECONDS <= seconds <= MAX_SECONDS:
        raise ValueError("--seconds must be between 1 and 20")
    if not isinstance(expected_uids, dict) or set(expected_uids) != {str(i) for i in range(1, 13)}:
        raise ValueError("Expected UID file must contain exactly IDs 1..12")
    if any(not isinstance(uid, str) or len(uid) != 16 or
           any(c not in "0123456789abcdef" for c in uid)
           for uid in expected_uids.values()):
        raise ValueError("Invalid expected UID")
    return {"motor_id": 7, "port": port, "seconds": seconds,
            "allowed_can_types": [0, 17], "motor_output_available": False,
            "calibration_applied": False}


def continuity_metrics(rows):
    """Report raw consecutive changes; do not unwrap or guess gear ratio."""
    if not rows:
        return {"samples": 0}
    changes = []
    for a, b in zip(rows, rows[1:]):
        dt = (b["monotonic_ns"] - a["monotonic_ns"]) / 1e9
        if dt <= 0:
            raise ValueError("Non-increasing position timestamps")
        changes.append({"from_sample": a["sample"], "to_sample": b["sample"],
                        "delta_deg": math.degrees(b["position_rad"] - a["position_rad"]),
                        "gap_ms": dt * 1000})
    largest = max(changes, key=lambda x: abs(x["delta_deg"])) if changes else None
    positions = [row["position_rad"] for row in rows]
    return {"samples": len(rows),
            "elapsed_s": (rows[-1]["monotonic_ns"] - rows[0]["monotonic_ns"]) / 1e9,
            "raw_start_rad": positions[0], "raw_end_rad": positions[-1],
            "raw_net_change_deg": math.degrees(positions[-1] - positions[0]),
            "raw_range_deg": math.degrees(max(positions) - min(positions)),
            "largest_adjacent_change": largest,
            "adjacent_changes_over_5deg": sum(abs(x["delta_deg"]) > 5 for x in changes),
            "interpretation": "Raw telemetry only; no automatic gear-ratio, wrap, or motion approval."}


def collect(can, seconds, emit, *, clock=time.monotonic_ns, sleep=time.sleep):
    start = clock()
    deadline = start + int(seconds * 1e9)
    rows = []
    next_sample = start
    while len(rows) < MAX_SAMPLES and clock() + int(can.timeout_s * 1e9) < deadline:
        reply = can.query(7, "position")
        if not reply.get("ok") or not math.isfinite(reply.get("value", math.nan)):
            raise RuntimeError("ID7 position read failed")
        if can.parser.discarded_bytes or can.parser.buffer:
            raise RuntimeError("Corrupt or incomplete CAN frame")
        row = {"kind": "id7_position_sample", "sample": len(rows),
               "monotonic_ns": reply["monotonic_ns"],
               "position_rad": reply["value"],
               "raw_value_hex": reply["raw_value_hex"],
               "round_trip_ms": reply["round_trip_ms"]}
        emit(row)
        rows.append(row)
        next_sample += 20_000_000
        remaining_ns = min(next_sample, deadline) - clock()
        if remaining_ns > 0:
            sleep(remaining_ns / 1e9)
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--start-immediately", action="store_true",
                        help="Start without the operator Enter cue, for supervised remote execution")
    parser.add_argument("--port", required=True)
    parser.add_argument("--expected-uids", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=15.0)
    args = parser.parse_args(argv)
    expected = json.loads(args.expected_uids.read_text())
    plan = validate_plan(args.port, args.seconds, expected)
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2))
        return 0
    output = args.output.expanduser().resolve()
    if any((p / ".git").exists() for p in (output, *output.parents)):
        parser.error("Private log output must be outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    result = {"started_at": datetime.datetime.now().astimezone().isoformat(),
              "plan": plan, "status": "INCOMPLETE", "approved_for_runtime": False}
    try:
        lock_dir = Path.home() / ".cache" / "singularitydog"
        lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        with (lock_dir / "manual-calibration.lock").open("a+") as motion_lock, \
             (lock_dir / "can-readonly.lock").open("a+") as can_lock, \
             (output / "events.jsonl").open("x", buffering=1) as events:
            fcntl.flock(motion_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(can_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            def emit(event):
                events.write(json.dumps({"wall_time_ns": time.time_ns(), **event},
                                        allow_nan=False) + "\n")
                if event.get("kind") == "motor_feedback" and (
                        event.get("type") == 21 or event.get("fault_bits", 0) or
                        event.get("mode_state", 0)):
                    raise RuntimeError("Enabled/fault feedback while observing")

            emit({"kind": "id7_position_stream_plan", **plan})
            if not args.start_immediately:
                if not sys.stdin.isatty():
                    raise RuntimeError("Interactive terminal required; or use --start-immediately")
                answer = input("ID7を手で動かす準備ができたらEnter（中止はq）: ").strip().lower()
                if answer == "q":
                    raise InterruptedError("Operator cancelled before recording")
                if answer:
                    raise ValueError("Only Enter or q is accepted")
            with ReadOnlyCAN(port=args.port, event_sink=emit) as can:
                identity = can.query(7)
                if not identity.get("ok") or identity.get("mcu_uid_hex") != expected["7"]:
                    raise RuntimeError("ID7 UID mismatch")
                print(f"ID7読み取り開始：{args.seconds:g}秒以内。足先側を無理なく動かしてください。", flush=True)
                rows = collect(can, args.seconds, emit)
            result.update(status="RECORDED_REVIEW_REQUIRED",
                          metrics=continuity_metrics(rows))
    except BaseException as error:
        result["error"] = repr(error)
    finally:
        result["completed_at"] = datetime.datetime.now().astimezone().isoformat()
        (output / "summary.json").write_text(json.dumps(result, indent=2,
                                                         ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["status"] == "RECORDED_REVIEW_REQUIRED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
