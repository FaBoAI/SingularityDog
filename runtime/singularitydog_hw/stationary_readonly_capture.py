"""Finite 10-second raw telemetry capture; no enable, STOP, or motion writes.

Fifty sweeps start at least 200 ms apart on each independent bus owner. These
readings do not prove a human-held stance, STOP, or readiness for motor output.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time

from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from .can_timing_probe import ownership_locks, validate_uids
from .fixed_stance_readonly_capture import IDS_BY_BUS, _checked_query
from .motor_epoch_readonly_capture import _expected_uids
from .post_charge_box_pose_check import private_output_path, _write_private


SCHEMA = "singularitydog.stationary-readonly-capture.v1"
PARAMETERS = ("position", "velocity", "current", "run_mode", "voltage")
SWEEPS = 50
PERIOD_NS = 200_000_000
DURATION_NS = 10_000_000_000
FLAGS = {"motor_output_allowed": False, "motor_enable_sent": False,
         "stop_sent": False, "stop_confirmed": False, "human_stance_verified": False,
         "stationarity_verified": False, "approved_for_runtime": False,
         "angle_wrap_applied": False, "automatic_retry": False,
         "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL"}


def make_plan():
    return {"schema": SCHEMA, "status": "PLAN_ONLY", "hardware_opened": False,
            "duration_s": 10, "sweeps": SWEEPS, "period_ms": 200,
            "parameters": list(PARAMETERS), "allowed_can_types": [0, 17],
            "requests_per_bus": 6 + SWEEPS * 6 * len(PARAMETERS),
            "catchup_available": False, "max_reply_timeout_ms": 250, **FLAGS}


def guard_event(bus, event):
    kind = event.get("kind")
    if kind == "can_tx":
        mid, parameter = event.get("motor_id"), event.get("parameter")
        if mid not in IDS_BY_BUS[bus] or parameter not in ("identity", *PARAMETERS):
            raise RuntimeError("Unexpected stationary-capture CAN transmission")
        expected = codec.read_request(mid, None if parameter == "identity" else parameter)
        if event.get("hex") != expected.hex():
            raise RuntimeError("Noncanonical stationary-capture CAN transmission")
    elif kind == "can_rx_frame":
        source = event.get("source_id")
        if type(source) is int and 1 <= source <= 12 and source not in IDS_BY_BUS[bus]:
            raise RuntimeError("Motor replied on the wrong USB2CAN bus")
    elif kind == "motor_feedback":
        if event.get("type") == 21 or event.get("fault_bits", 0) or event.get("mode_state", 0):
            raise RuntimeError("Enabled or fault feedback during read-only capture")


def capture(can_factory, expected_uids, *, check=lambda: None,
            clock=time.monotonic_ns, wait=time.sleep):
    """Open/query/close each bus on its sole worker; retain partial raw evidence."""
    expected = validate_uids(expected_uids)
    abort = threading.Event()
    epoch = []
    ready = threading.Barrier(2, action=lambda: epoch.append(clock()))
    result = {**make_plan(), "status": "ABORTED_READONLY_CAPTURE", "errors": [],
              "buses": {bus: {"identities": {}, "sweeps": [], "events": [], "errors": []}
                        for bus in IDS_BY_BUS}}

    def work(bus):
        data = result["buses"][bus]
        data["owner_thread_id"] = threading.get_ident()
        can = None
        startup_end = clock() + 3_000_000_000

        def checked(deadline=None):
            if abort.is_set():
                raise InterruptedError("Peer failure or capture cancellation")
            check()
            if deadline is not None and clock() >= deadline:
                raise TimeoutError("Finite read-only capture deadline reached")

        def sink(event):
            data["events"].append(dict(event))
            guard_event(bus, event)

        def query(mid, parameter, deadline):
            checked(deadline)
            remaining = (deadline-clock())/1e9
            if remaining < .01:
                raise TimeoutError("Insufficient remaining read-only reply budget")
            can.timeout_s = min(.25, remaining)
            reply = _checked_query(can, bus, mid, parameter, lambda: checked(deadline))
            if parameter is not None:
                index, _, unit = codec.PARAMETERS[parameter]
                if reply.get("index") != index or reply.get("unit") != unit:
                    raise RuntimeError(f"ID{mid} {parameter} index/unit mismatch")
                if parameter == "run_mode" and (type(reply["value"]) is not int or reply["value"] != 0):
                    raise RuntimeError(f"ID{mid} run_mode={reply['value']!r}; required 0")
            return reply

        def until(target, deadline):
            while clock() < target:
                checked(deadline)
                wait(min(.01, max(0., (target-clock())/1e9)))

        try:
            checked(startup_end)
            with can_factory(bus, sink) as can:
                for mid in IDS_BY_BUS[bus]:
                    reply = query(mid, None, startup_end)
                    if reply["mcu_uid_hex"] != expected[mid]:
                        raise RuntimeError(f"ID{mid} UID mismatch")
                    data["identities"][str(mid)] = reply
                ready.wait(timeout=3.)
                start = epoch[0]
                end = start + DURATION_NS
                data["capture_begin_ns"] = start
                next_release = start
                for index in range(SWEEPS):
                    until(next_release, end)
                    checked(end)
                    begin = clock()
                    row = {"index": index, "begin_ns": begin, "requested_release_ns": next_release,
                           "complete": False, "samples": {}}
                    data["sweeps"].append(row)
                    for mid in IDS_BY_BUS[bus]:
                        row["samples"][str(mid)] = {}
                        for parameter in PARAMETERS:
                            row["samples"][str(mid)][parameter] = query(mid, parameter, end)
                    row.update(end_ns=clock(), complete=True)
                    next_release = max(begin + PERIOD_NS, row["end_ns"])
                # Preserve a ten-second observation window even after the last read.
                until(end, None)
                checked()
                if (len(data["identities"]) != 6 or len(data["sweeps"]) != SWEEPS or
                        not all(row["complete"] for row in data["sweeps"]) or
                        can.tx_count != make_plan()["requests_per_bus"]):
                    raise RuntimeError("Incomplete finite read-only request/sweep evidence")
                if can.parser.discarded_bytes or can.parser.buffer:
                    raise RuntimeError("Final CAN parser has discarded or partial bytes")
                data["capture_end_ns"] = clock()
        except BaseException as error:
            abort.set()
            ready.abort()
            if can is not None:
                can.poisoned = True
            data["errors"].append(type(error).__name__ + ": " + str(error))
        finally:
            data["session_poisoned"] = can.poisoned if can is not None else None
            data["queries_sent"] = can.tx_count if can is not None else 0
            data["rx_bytes"] = can.rx_bytes if can is not None else 0
            if can is not None:
                data["parser_discarded_bytes"] = can.parser.discarded_bytes
                data["parser_residual_hex"] = bytes(can.parser.buffer).hex()

    with ThreadPoolExecutor(max_workers=2, thread_name_prefix="stationary-readonly") as pool:
        futures = [pool.submit(work, bus) for bus in IDS_BY_BUS]
        for future in futures:
            future.result()
    for bus, data in result["buses"].items():
        result["errors"].extend(f"{bus}: {error}" for error in data["errors"])
    result["queries_sent"] = sum(data["queries_sent"] for data in result["buses"].values())
    if not result["errors"]:
        result["status"] = "COMPLETE_STATIONARY_READONLY_CAPTURE"
    return result


def source_hashes():
    directory = Path(__file__).parent
    names = (Path(__file__).name, "can_readonly.py", "dual_can_pipeline_benchmark.py",
             "sensor_pipeline_benchmark.py", "can_timing_probe.py",
             "fixed_stance_readonly_capture.py", "motor_epoch_readonly_capture.py",
             "post_charge_box_pose_check.py")
    return {name: hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in names}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    for name in ("front-port", "rear-port", "expected-uids", "boot-id", "output"):
        parser.add_argument("--" + name)
    args = parser.parse_args(argv)
    if not args.execute_readonly:
        print(json.dumps(make_plan(), indent=2)); return 0
    if not all(getattr(args, name) for name in ("front_port", "rear_port", "expected_uids", "boot_id", "output")):
        parser.error("Explicit ports, expected UIDs, current boot ID and new private output required")
    expected, expected_hash = _expected_uids(args.expected_uids)
    output = private_output_path(args.output)
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    cancelled = threading.Event()
    handlers = {}
    result = {**make_plan(), "status": "ABORTED_READONLY_CAPTURE", "errors": []}
    metadata = {"expected_uids_sha256": expected_hash, "source_sha256": source_hashes(),
                "expected_boot_id": args.boot_id,
                "hardware_open_attempted": False, "hardware_opened": False,
                "started_at": datetime.datetime.now().astimezone().isoformat()}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda *_: cancelled.set())
        with ExitStack() as stack:
            stack.enter_context(ownership_locks())
            for bus in IDS_BY_BUS:
                stack.enter_context(dual.port_lock(bindings[bus]["resolved"]))
            boot = dual.BootIdentityGuard(); stack.callback(boot.close)
            metadata["boot_id"] = boot.boot_id
            if boot.boot_id != args.boot_id:
                raise RuntimeError("Current boot differs from requested boot ID")
            def check():
                if cancelled.is_set():
                    raise InterruptedError("Read-only capture interrupted")
                boot.check()
                if any(not dual.binding_matches(binding) for binding in bindings.values()):
                    raise RuntimeError("USB2CAN port binding changed")
            @contextmanager
            def factory(bus, sink):
                check()
                metadata["hardware_open_attempted"] = True
                with codec.ReadOnlyCAN(port=bindings[bus]["path"], event_sink=sink) as can:
                    metadata["hardware_opened"] = True
                    if os.fstat(can.serial.fileno()).st_rdev != bindings[bus]["st_rdev"]:
                        raise RuntimeError(f"{bus} opened USB2CAN differs from verified binding")
                    yield can
            result = capture(factory, expected, check=check)
    except BaseException as error:
        result["errors"].append(type(error).__name__ + ": " + str(error))
    finally:
        for sig, previous in handlers.items():
            signal.signal(sig, previous)
    result.update(metadata, completed_at=datetime.datetime.now().astimezone().isoformat())
    _write_private(output, result)
    print(json.dumps({"status": result["status"], "errors": result["errors"], "output": str(output), **FLAGS}))
    return 0 if result["status"] == "COMPLETE_STATIONARY_READONLY_CAPTURE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
