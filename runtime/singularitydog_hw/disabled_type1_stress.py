"""Bounded Type1 reply stress while every RS05 remains disabled and at zero gain.

This is a transport diagnostic, not a holding or locomotion controller. It
never sends enable, positive gains, feedforward torque, or learned targets.
The firmware's disabled-state Type1 reply is checked after every request.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from dataclasses import asdict

from . import watchdog_commissioning as watchdog
from . import rs05_trial_protocol as protocol
from .motor_version_probe import validate_uids


MAX_CYCLES = 100
MAX_DENSE_CYCLES = 50
PERIOD_NS = 20_000_000
DENSE_DIAGNOSTIC_DEADLINE_NS = 25_000_000
GAP_NS = 800_000
WINDOW = 3


def plan(cycles: int, *, dense: bool = False) -> dict:
    if type(dense) is not bool or type(cycles) is not int or not 1 <= cycles <= (
            MAX_DENSE_CYCLES if dense else MAX_CYCLES):
        raise ValueError("Type1 stress is limited to 100 ordinary or 50 dense cycles")
    return {"schema": "disabled-type1-stress-v1", "status": "PLAN_ONLY",
            "cycles": cycles, "period_ms": 20, "type1_requests_per_cycle": 24 if dense else 12,
            "voltage_requests_per_cycle": 2 if dense else 0,
            "dense_disabled_26": dense,
            "diagnostic_reply_deadline_ms": 25 if dense else None,
            "request_gap_us": 800 if dense else None,
            "request_window": 3 if dense else 1,
            "kp": 0, "kd": 0, "motor_enable_available": False,
            "learned_targets_available": False, "hardware_opened": False}


def run(channels, expected_uids, *, cycles: int, announce=lambda: None,
        check=lambda: None, clock=time.monotonic_ns, wait=time.sleep,
        dense=False) -> dict:
    result = {**plan(cycles, dense=dense), "status": "ABORTED", "errors": [],
              "cycles_completed": 0, "motor_enable_sent": False,
              "positive_gain_sent": False, "learned_targets_sent": False,
              "per_cycle": [], "stop_confirmed": False}
    expected = validate_uids(expected_uids)
    if set(channels) != set(watchdog.BUSES) or channels["front"] is channels["rear"]:
        raise ValueError("Exactly two independent bus owners are required")
    centers = {}
    started = clock()

    def guard():
        check()
        if clock() - started >= 20_000_000_000:
            raise TimeoutError("Bounded Type1 diagnostic exceeded 20 seconds")

    try:
        for scope, ids in watchdog.BUSES.items():
            owner = channels[scope]
            for mid in ids:
                guard()
                if owner.exchange(mid, "identity")["mcu_uid_hex"] != expected[mid]:
                    raise ValueError(f"ID{mid} UID mismatch")
                stopped = owner.exchange(mid, "stop")
                if stopped["mode_state"] != 0 or stopped["fault_bits"] != 0:
                    raise RuntimeError(f"ID{mid} initial STOP unconfirmed")
                if owner.exchange(mid, "run_mode")["value"] != 0:
                    raise RuntimeError(f"ID{mid} MIT mode not confirmed")
                volts = owner.exchange(mid, "voltage")["value"]
                if not 35 <= volts <= 42:
                    raise RuntimeError(f"ID{mid} voltage outside 35..42V")
                centers[mid] = stopped["protocol_position_rad"]
                if not math.isfinite(centers[mid]):
                    raise RuntimeError(f"ID{mid} invalid disabled center")
                probe = owner.exchange(mid, "zero", center=centers[mid])
                _check_zero(probe, mid, centers[mid])
        guard()
        announce()
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="disabled-type1") as pool:
            next_release = clock()
            for cycle in range(cycles):
                guard()
                while clock() < next_release:
                    guard()
                    wait(min(.001, max(0., (next_release-clock())/1e9)))
                begin = clock()
                futures = {scope: pool.submit(_sweep_dense if dense else _sweep,
                                               channels[scope], ids, centers, guard,
                                               cycle, begin+(DENSE_DIAGNOSTIC_DEADLINE_NS if dense else PERIOD_NS))
                           for scope, ids in watchdog.BUSES.items()}
                failures = []
                for scope, future in futures.items():
                    try:
                        future.result()
                    except BaseException as error:
                        failures.append(f"{scope}: {type(error).__name__}: {error}")
                end = clock()
                result["per_cycle"].append({"cycle": cycle+1,
                    "begin_ns": begin, "end_ns": end,
                    "elapsed_ms": (end-begin)/1e6,
                    "release_lateness_ms": max(0, begin-next_release)/1e6})
                if failures:
                    raise RuntimeError("; ".join(failures))
                result["cycles_completed"] = cycle+1
                next_release = max(next_release+PERIOD_NS, end)
        result["status"] = "COMPLETE_DISABLED_TYPE1_DIAGNOSTIC"
    except BaseException as error:
        result["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        stops = {}
        # Both physical CAN buses receive their STOP attempts concurrently.
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="disabled-stop") as pool:
            stop_futures = {scope: pool.submit(owner.stop_all) for scope, owner in channels.items()}
            for scope, future in stop_futures.items():
                try:
                    stops[scope] = future.result()
                except BaseException as error:
                    stops[scope] = {"complete": False, "unconfirmed_ids": list(watchdog.BUSES[scope]),
                                    "errors": [f"{type(error).__name__}: {error}"]}
        result["stop_reports"] = stops
        result["stop_confirmed"] = all(stops[s].get("complete") is True and
            not stops[s].get("unconfirmed_ids") and not stops[s].get("errors")
            for s in watchdog.BUSES)
        if not result["stop_confirmed"]:
            result["status"] = "STOP_UNCONFIRMED_POWER_OFF_REQUIRED"
            result["errors"].append("Physically switch motor power Off")
        result["elapsed_s"] = (clock()-started)/1e9
    return result


def _check_zero(reply, mid, center, *, expected_mode=0):
    if expected_mode not in (0, 2):
        raise ValueError("Only disabled or active zero-gain feedback is permitted")
    if (reply["mode_state"] != expected_mode or reply["fault_bits"] != 0 or
            abs(reply["protocol_position_rad"]-center) > math.radians(3) or
            abs(reply["velocity_rad_s"]) > .5 or not -10 <= reply["temperature_c"] < 60):
        raise RuntimeError(f"ID{mid} did not remain safely disabled at zero gain")


def _sweep(owner, ids, centers, check, cycle=None, deadline=None):
    for mid in ids:
        check()
        _check_zero(owner.exchange(mid, "zero", center=centers[mid]), mid, centers[mid])


def _sweep_dense(owner, ids, centers, check, cycle, deadline):
    """Two six-axis zero-gain phases plus one voltage read, all while disabled."""
    _zero_batch(owner, ids, centers, check, deadline)
    check()
    volts = owner.exchange(ids[cycle % len(ids)], "voltage")["value"]
    if not 35 <= volts <= 42:
        raise RuntimeError("Rotating voltage outside 35..42V")
    _zero_batch(owner, ids, centers, check, deadline)


def _zero_batch(owner, ids, centers, check, deadline, *, expected_mode=0):
    """One bus owner. Three outstanding Type1 commands at 0.8ms spacing.

    No Type1 can be discarded: a missing frame leaves its request pending and
    makes the later STOP acknowledgement for that axis ambiguous.
    """
    if not isinstance(owner, watchdog.Channel):
        raise TypeError("Dense mode requires the audited Channel implementation")
    try:
        owner._boundary()
        sent, completed, next_send = 0, set(), owner.clock()
        while len(completed) < len(ids):
            check()
            now = owner.clock()
            if now >= deadline:
                raise TimeoutError("Dense disabled Type1 diagnostic reply deadline exceeded")
            if sent < len(ids) and len(owner.pending) < WINDOW and now >= next_send:
                _, finish = owner._send(ids[sent], "zero", centers[ids[sent]])
                sent += 1
                next_send = finish + GAP_NS
                continue
            wake = min(deadline, next_send) if sent < len(ids) and len(owner.pending) < WINDOW else deadline
            frames, _ = owner._read(wake, deadline)
            for frame in frames:
                mid = frame.source
                if frame.kind != 2 or mid not in owner.pending or owner.pending[mid] != "zero" or mid in completed:
                    raise RuntimeError("Unexpected, duplicate, or unowned Type1 response")
                reply = asdict(protocol.decode_type2(frame, motor_id=mid))
                _check_zero(reply, mid, centers[mid], expected_mode=expected_mode)
                completed.add(mid)
                del owner.pending[mid]
        owner._boundary()
    except BaseException:
        owner.failed = True
        raise


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cycles", type=int, default=100)
    p.add_argument("--dense-disabled-26", action="store_true")
    p.add_argument("--execute-supported-disabled", action="store_true")
    for name in ("support-in-place", "cutoff-ready", "rs05-model-confirmed"):
        p.add_argument("--"+name, action="store_true")
    for name in ("front-port", "rear-port", "expected-uids", "boot-id", "power-epoch",
                 "output", "audio", "audio-sha256", "audio-device"):
        p.add_argument("--"+name)
    a = p.parse_args(argv)
    intended = plan(a.cycles, dense=a.dense_disabled_26)
    if not a.execute_supported_disabled:
        print(json.dumps(intended, indent=2)); return 0
    if not all((a.support_in_place, a.cutoff_ready, a.rs05_model_confirmed)):
        p.error("Current support, physical cutoff and RS05 confirmation required")
    required = ("front_port", "rear_port", "expected_uids", "boot_id", "power_epoch",
                "output", "audio", "audio_sha256", "audio_device")
    if any(not getattr(a, name) for name in required):
        p.error("All explicit hardware, epoch, output and announcement inputs required")
    audio = Path(a.audio).resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest() != a.audio_sha256:
        p.error("Audio file hash mismatch")
    out = Path(a.output).expanduser().resolve()
    if any((parent/".git").exists() for parent in (out, *out.parents)):
        p.error("Raw output must stay outside Git")
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    from . import dual_can_pipeline_benchmark as dual
    from .sensor_pipeline_benchmark import BootIdentityGuard
    source = Path(a.expected_uids).read_bytes()
    expected = validate_uids(json.loads(source))
    channels, handlers, cancelled = {}, {}, []
    report = {**intended, "status": "ABORTED_BEFORE_OPEN", "errors": [],
              "motor_enable_sent": False, "learned_targets_sent": False}
    try:
        if Path("/proc/sys/kernel/random/boot_id").read_text().strip() != a.boot_id:
            raise RuntimeError("Jetson boot changed")
        bindings = dual.validate_ports(a.front_port, a.rear_port)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.append(number))
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks())
            boot = BootIdentityGuard(); stack.callback(boot.close)
            if boot.boot_id != a.boot_id:
                raise RuntimeError("Jetson boot changed")
            def check():
                if cancelled:
                    raise InterruptedError("Operator interrupted Type1 diagnostic")
                boot.check()
            import serial
            for scope, binding in bindings.items():
                stack.enter_context(dual.port_lock(binding["resolved"]))
                port = serial.Serial(port=None, baudrate=921600, timeout=0,
                                     write_timeout=.02, exclusive=True)
                port.dtr = port.rts = False
                port.port = binding["path"]; port.open(); stack.callback(port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev != binding["st_rdev"]:
                    raise RuntimeError("USB binding changed")
                channels[scope] = watchdog.Channel(port, watchdog.BUSES[scope], check=check)
            def announce():
                check()
                subprocess.run(["aplay", "-D", a.audio_device, str(audio)], check=True,
                    timeout=8., stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE)
            report = run(channels, expected, cycles=a.cycles, announce=announce,
                         check=check, dense=a.dense_disabled_26)
            report.update(boot_id=boot.boot_id, motor_power_epoch=a.power_epoch,
                expected_uids_sha256=hashlib.sha256(source).hexdigest(),
                events_by_bus={scope: channel.events for scope, channel in channels.items()})
    except BaseException as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        with os.fdopen(os.open(out/"report.json", os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), "w") as handle:
            json.dump(report, handle, allow_nan=False); handle.write("\n")
    print(json.dumps({key: report.get(key) for key in
        ("status", "errors", "cycles_completed", "stop_confirmed", "motor_enable_sent")},
        ensure_ascii=False))
    return 0 if report["status"] == "COMPLETE_DISABLED_TYPE1_DIAGNOSTIC" else 2


if __name__ == "__main__":
    raise SystemExit(main())
