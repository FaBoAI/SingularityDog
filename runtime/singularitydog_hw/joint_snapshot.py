"""Bounded, read-only RS05 joint snapshots for before/after observations.

No motor enable, stop, setting write, zeroing, or IMU configuration is available.
Captured shaft positions are not robot joint zeros or verified joint directions.
"""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import time

from .can_readonly import ReadOnlyCAN


def collect(can, emit, *, sweeps=10, check_interrupt=lambda: None, wait=time.sleep):
    """Require all 12 unique identities and complete, finite core read replies."""
    if type(sweeps) is not int or not 3 <= sweeps <= 60:
        raise ValueError("sweeps must be an integer in 3..60")
    identities = {}
    positions = {i: [] for i in range(1, 13)}
    for mid in positions:
        check_interrupt()
        value = can.query(mid)
        if not value.get("ok"):
            raise RuntimeError(f"Identity read failed: ID {mid}")
        uid = value.get("mcu_uid_hex", "")
        if len(uid) != 16 or any(c not in "0123456789abcdef" for c in uid):
            raise RuntimeError(f"Invalid identity: ID {mid}")
        if uid in identities.values():
            raise RuntimeError("Duplicate motor identity")
        identities[mid] = uid
    for sweep in range(sweeps):
        for mid in positions:
            for parameter in ("position", "current", "velocity", "voltage"):
                check_interrupt()
                result = can.query(mid, parameter)
                v = result.get("value")
                if not result.get("ok") or type(v) not in (int, float) or not math.isfinite(v):
                    raise RuntimeError(f"Failed finite read: ID {mid} {parameter}")
                if parameter == "position":
                    positions[mid].append(v)
        emit({"kind": "joint_snapshot_sweep", "monotonic_ns": time.monotonic_ns(),
              "sweep": sweep + 1})
        if sweep + 1 < sweeps:
            wait(.05)
    check_interrupt()
    return {"joint_calibration_verified": False, "identities": identities,
            "position_unit": "rad_output_shaft", "automatic_wrap_applied": False,
            "positions": {str(i): {"samples": len(v), "mean": statistics.fmean(v),
                                     "min": min(v), "max": max(v), "last": v[-1]}
                          for i, v in positions.items()}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--sweeps", type=int, default=10)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    if not 3 <= args.sweeps <= 60:
        ap.error("sweeps must be 3..60")
    plan = {"ids": list(range(1, 13)), "sweeps": args.sweeps,
            "allowed_can_types": [0, 17], "motor_output_available": False,
            "parameters": ["identity", "position", "current", "velocity", "voltage"],
            "imu_opened": False, "joint_calibration_verified": False}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    output = args.output.expanduser().resolve()
    if any((p / ".git").exists() for p in (output, *output.parents)):
        ap.error("Private logs must be outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    metadata = {"started_at": datetime.datetime.now().astimezone().isoformat(), "plan": plan,
                "errors": [], "source_sha256": {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (Path(__file__), Path(__file__).with_name("can_readonly.py"))}}
    signals, handlers = [], {}
    def interrupted(signum, _frame):
        signals.append(signum)
    def check_interrupt():
        if signals:
            raise InterruptedError(f"signal {signals[0]}")
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, interrupted)
        with (output / "events.jsonl").open("x", buffering=1) as log:
            def emit(event):
                log.write(json.dumps({"wall_time_ns": time.time_ns(), **event}, allow_nan=False) + "\n")
            emit({"kind": "capture_metadata", "monotonic_ns": time.monotonic_ns(), **plan})
            try:
                with ReadOnlyCAN(event_sink=emit) as can:
                    metadata["summary"] = collect(can, emit, sweeps=args.sweeps,
                                                  check_interrupt=check_interrupt)
                    metadata["tx_count"] = can.tx_count
                    metadata["discarded_rx_bytes"] = can.parser.discarded_bytes
                    if can.parser.discarded_bytes:
                        raise RuntimeError("Discarded serial bytes; snapshot integrity not established")
                check_interrupt()
            except BaseException as error:
                metadata["errors"].append(repr(error))
                emit({"kind": "capture_error", "monotonic_ns": time.monotonic_ns(),
                      "ok": False, "error": repr(error)})
    except BaseException as error:
        metadata["errors"].append(repr(error))
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if signals and not any("signal " in e for e in metadata["errors"]):
        metadata["errors"].append(f"signal {signals[0]}")
    metadata.update(completed_at=datetime.datetime.now().astimezone().isoformat(),
                    status="INCOMPLETE" if metadata["errors"] else "RECORDED_NOT_CALIBRATED")
    (output / "summary.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(output), "status": metadata["status"],
                      "errors": metadata["errors"], "tx_count": metadata.get("tx_count")}, indent=2))
    return int(bool(metadata["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
