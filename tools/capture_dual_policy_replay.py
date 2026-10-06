#!/usr/bin/env python3
"""Bounded dual-CAN + IMU data for the existing read-only diagnose contract.

Default PLAN opens no devices, locks or output files. Explicit execution reads
Type0/17 only, on one CAN worker across both buses; IMU acquisition is concurrent.
Original source times and raw wire events stay intact. This is saved data for
offline policy comparison, never a motor command or calibration approval.
"""
import argparse
from contextlib import ExitStack
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import threading
import time

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import can_timing_probe as timing
from singularitydog_hw import diagnose
from singularitydog_hw import dual_can_pipeline_benchmark as dual
from singularitydog_hw import imu
from singularitydog_hw import policy_observer_live as live
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.post_charge_box_pose_check import _boot_id

SCHEMA = "singularitydog.dual-readonly-diagnose-capture.v1"
IDS_BY_BUS = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}
READS = ("position", "velocity", "current", "voltage", "run_mode")
MAX_QUERIES = 4096
MAX_EVENTS, MAX_BYTES = 65536, 32 * 1024 * 1024
FLAGS = dict(motor_output_available=False, motor_enable_sent=False,
             stop_sent=False, learned_targets_sent=False, approved_for_runtime=False,
             calibration_applied=False, automatically_applied=False,
             physical_stationarity_proven=False, motor_power_epoch_verified=False)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def make_plan(seconds=10., timeout_s=.25, front_port=None, rear_port=None):
    require(type(seconds) in (int, float) and math.isfinite(seconds)
            and 1 <= seconds <= 10, "seconds must be finite and1..10")
    require(type(timeout_s) in (int, float) and math.isfinite(timeout_s)
            and .01 <= timeout_s <= .25, "timeout must be finite and0.01..0.25s")
    return {"status": "PLAN_ONLY", "producer": SCHEMA, **FLAGS,
            "ids_bottom_to_top": diagnose.LEG_IDS,
            "ids_by_bus": {b: list(ids) for b, ids in IDS_BY_BUS.items()},
            "ports": {"front": front_port, "rear": rear_port},
            "allowed_can_types": [0, 17], "one_outstanding_request_across_both_buses": True,
            "automatic_retry": False, "duration_seconds": seconds,
            "can_reply_timeout_s": timeout_s, "max_can_queries": MAX_QUERIES,
            "initial_checks": ["all12 expected UID", "run_mode == 0", "current == 0"],
            "coverage_parameters": list(READS),
            "parameters_not_requested": ["can_timeout", "zero_state"],
            "position_velocity_order": "ID1..12, position then velocity",
            "imu_poll_hz": 100, "imu_configuration_written_and_restored": True,
            "pure_hardware_readonly": False,
            "imu_setup": "existing ICM20948 accel2g/gyro250dps; original registers restored",
            "timestamps": "original host CAN/IMU monotonic times; sensor conversion time unknown",
            "deadline_scope": "acquisition after IMU setup; last in-flight CAN read may finish within timeout",
            "partial_final_sweep": "preserved; frame builder excludes incomplete tail; stateful replay retains original per-source availability",
            "no_policy_inference_or_motor_output": True}


def private_directory(path):
    requested = Path(path).expanduser().absolute()
    require(not any(p.is_symlink() for p in (requested, *requested.parents)),
            "Output path must not contain symlinks")
    output = requested.resolve()
    require(output.parent.is_dir() and not output.exists(), "Choose a fresh directory with existing parent")
    require(not any((p / ".git").exists() for p in (output, *output.parents)),
            "Raw UID and telemetry files must be outside Git")
    return output


class Recorder:
    """One synchronized private stream; original event fields are not retimed."""
    def __init__(self, directory):
        self.records, self.bytes = [], 0
        self.lock = threading.Lock()
        fd = os.open(Path(directory) / "events.jsonl", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.stream = os.fdopen(fd, "w", buffering=1)

    def emit(self, event):
        with self.lock:
            record = {"wall_time_ns": time.time_ns(), **event}
            payload = json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n"
            length = len(payload.encode())
            require(len(self.records) < MAX_EVENTS and self.bytes + length <= MAX_BYTES,
                    "Capture event/byte budget exhausted")
            written = self.stream.write(payload)
            require(written == len(payload), "Short event trace write")
            self.bytes += length
            self.records.append(record)

    def close(self):
        try:
            self.stream.flush()
            os.fsync(self.stream.fileno())
        finally:
            self.stream.close()


def guard_can_event(bus, event):
    if event.get("kind") == "can_tx":
        mid, parameter = event.get("motor_id"), event.get("parameter")
        require(mid in IDS_BY_BUS[bus] and parameter in ("identity", *READS),
                "Unexpected CAN request scope")
        wire = codec.read_request(mid, None if parameter == "identity" else parameter)
        require(event.get("hex") == wire.hex(), "Noncanonical read-only CAN request")
    elif event.get("kind") == "can_rx_frame":
        source = event.get("source_id")
        require(not (type(source) is int and 1 <= source <= 12 and source not in IDS_BY_BUS[bus]),
                "Motor replied on the wrong bus")
    elif event.get("kind") == "motor_feedback":
        require(event.get("type") != 21 and not event.get("fault_bits", 0)
                and not event.get("mode_state", 0), "Enabled/fault feedback during read-only capture")


def verify_binding(can, binding):
    info = os.fstat(can.serial.fileno())
    require(dual.binding_matches(binding) and stat.S_ISCHR(info.st_mode)
            and info.st_rdev == binding["st_rdev"], "Opened USB2CAN binding changed")


def coverage_errors(summary, expected, state):
    errors = []
    if summary["imu"]["samples"] < 2:
        errors.append({"component": "coverage", "error": "Fewer than two IMU samples"})
    for mid, row in summary["motors"].items():
        if row["identities"] != [expected[int(mid)]]:
            errors.append({"component": "coverage", "error": "Missing/inconsistent expected UID: ID" + mid})
        missing = [name for name in READS if row["parameters"][name]["last"] is None]
        if missing:
            errors.append({"component": "coverage", "error": f"ID{mid}: missing {','.join(missing)}"})
    if not state["initial_quiet_verified"] or state["complete_sweeps"] < 1:
        errors.append({"component": "coverage", "error": "Initial quiet checks or complete all12 sweep missing"})
    return errors


def capture(plan, bindings, expected, recorder, *, stop=None,
            can_factory=codec.ReadOnlyCAN, imu_factory=imu.ICM20948,
            common_lock=timing.ownership_locks, port_lock=dual.port_lock,
            imu_lock=live.imu_ownership_lock, binding_check=verify_binding,
            clock=time.monotonic_ns):
    """Hold all leases through worker join, both serial closes and IMU restore."""
    stop = stop if stop is not None else threading.Event()
    errors, warnings = [], []
    state = {"initial_quiet_verified": False, "identities_verified": False,
             "complete_sweeps": 0, "partial_final_sweep_queries": 0,
             "queries": 0, "can_worker_joined": False, "can_ports_closed": False}
    report = {"schema": SCHEMA, **FLAGS,
              "plan": {**plan, "status": "EXECUTED_READONLY_ACQUISITION_PLAN"}, "errors": errors,
              "warnings": warnings, "acquisition": state,
              "started_at": datetime.datetime.now(datetime.timezone.utc).isoformat()}
    device, worker = None, None
    ready = threading.Event()
    try:
        with ExitStack() as leases:
            leases.enter_context(common_lock())
            for bus in IDS_BY_BUS:
                leases.enter_context(port_lock(bindings[bus]["resolved"]))
            leases.enter_context(imu_lock())
            require(not stop.is_set(), "Capture cancelled before hardware access")
            device = imu_factory()
            try:
                report["imu_configuration"] = device.start()
                report["imu_original_registers"] = device.original_registers
                report["imu_register_audit_before"] = device.diagnostic_registers()
                recorder.emit({"kind": "imu_configured", "monotonic_ns": clock(),
                               "configuration": report["imu_configuration"]})
                started = clock()
                deadline = started + int(plan["duration_seconds"] * 1e9)
                report["acquisition_started_monotonic_ns"] = started
                report["acquisition_deadline_monotonic_ns"] = deadline

                def collect_can():
                    cans = {}
                    try:
                        with ExitStack() as ports:
                            for bus in IDS_BY_BUS:
                                def emit(event, b=bus):
                                    guard_can_event(b, event)
                                    recorder.emit({**event, "bus": b})
                                can = ports.enter_context(can_factory(port=bindings[bus]["path"],
                                    timeout_s=plan["can_reply_timeout_s"], event_sink=emit))
                                cans[bus] = can
                                binding_check(can, bindings[bus])

                            def query(mid, parameter=None, *, initial=False):
                                if stop.is_set() or clock() >= deadline:
                                    require(not initial, "Capture ended before initial coverage")
                                    return None
                                require(state["queries"] < MAX_QUERIES, "CAN query budget exhausted")
                                bus = "front" if mid <= 6 else "rear"
                                can = cans[bus]
                                binding_check(can, bindings[bus])
                                state["queries"] += 1
                                result = can.query(mid, parameter)
                                require(not can.parser.buffer and not can.parser.discarded_bytes,
                                        "CAN parser has partial/discarded bytes")
                                require(result.get("ok") is True, "Invalid CAN parameter reply")
                                if parameter is None:
                                    require(result.get("mcu_uid_hex") == expected[mid], f"ID{mid} expected UID mismatch")
                                else:
                                    require(result.get("index") == codec.PARAMETERS[parameter][0]
                                            and result.get("unit") == codec.PARAMETERS[parameter][2],
                                            "Parameter index/unit mismatch")
                                    if parameter in ("run_mode", "current"):
                                        require(result.get("value") == 0, f"ID{mid} {parameter} is not zero")
                                return result

                            for mid in range(1, 13):
                                query(mid, initial=True)
                            state["identities_verified"] = True
                            for mid in range(1, 13):
                                for parameter in ("run_mode", "current", "voltage"):
                                    query(mid, parameter, initial=True)
                            state["initial_quiet_verified"] = True
                            while not ready.is_set() and not stop.is_set() and clock() < deadline:
                                stop.wait(.005)
                            while not stop.is_set() and clock() < deadline:
                                state["partial_final_sweep_queries"] = 0
                                for mid in range(1, 13):
                                    for parameter in READS:
                                        if query(mid, parameter) is None:
                                            break
                                        state["partial_final_sweep_queries"] += 1
                                    if stop.is_set() or clock() >= deadline:
                                        break
                                if state["partial_final_sweep_queries"] != 12 * len(READS):
                                    break
                                state["complete_sweeps"] += 1
                                state["partial_final_sweep_queries"] = 0
                    except BaseException as error:
                        errors.append({"component": "can", "error": repr(error)})
                        stop.set()
                    finally:
                        state["can_ports_opened"] = len(cans)
                        state["can_ports_closed"] = all(
                            getattr(can.serial, "is_open", None) is False for can in cans.values())
                        state["can_worker_exited"] = True

                worker = threading.Thread(target=collect_can, name="dual-readonly-can")
                worker.start()
                next_poll = last_sample = clock()
                while not stop.is_set() and clock() < deadline:
                    sample = device.read_sample()
                    now = clock()
                    if sample is not None:
                        recorder.emit({"kind": "imu", **sample})
                        last_sample = now
                        ready.set()
                    else:
                        require(now - last_sample <= 500_000_000, "No fresh IMU sample for0.5s")
                    next_poll += 10_000_000
                    if next_poll < now - 10_000_000:
                        next_poll = now
                    stop.wait(max(0., (next_poll - now) / 1e9))
                if stop.is_set() and not errors:
                    raise InterruptedError("Read-only capture cancelled")
            finally:
                stop.set()
                if worker is not None and worker.ident is not None:
                    worker.join()
                    state["can_worker_joined"] = not worker.is_alive()
                if device is not None:
                    try:
                        report["imu_register_audit_after"] = device.diagnostic_registers()
                        require(report.get("imu_register_audit_before") == report["imu_register_audit_after"],
                                "IMU offset/trim audit changed")
                    except BaseException as error:
                        errors.append({"component": "imu_audit", "error": repr(error)})
                    try:
                        device.close()
                    except BaseException as error:
                        errors.append({"component": "imu_restore", "error": repr(error)})
                    report["imu_restore_status"] = device.restore_status
                    if device.restore_status != "restored":
                        errors.append({"component": "imu_restore", "error": "Restoration was not confirmed"})
                    try:
                        recorder.emit({"kind": "imu_restored", "monotonic_ns": clock(),
                                       "restore_status": device.restore_status})
                    except BaseException as error:
                        errors.append({"component": "event_trace", "error": repr(error)})
    except BaseException as error:
        errors.append({"component": "main_or_imu", "error": repr(error)})
    summary = diagnose.summarize(recorder.records)
    errors.extend(coverage_errors(summary, expected, state))
    if state["partial_final_sweep_queries"]:
        warnings.append({"component": "coverage", "message": "Original partial final sweep retained; consumers must distinguish complete-frame and per-source replay",
                         "parameter_queries": state["partial_final_sweep_queries"]})
    report.update(summary=summary, completed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  acquisition_finished_monotonic_ns=clock(),
                  status="INCOMPLETE" if errors else "COMPLETE_WITH_WARNINGS" if warnings else "COMPLETE")
    report["required_coverage_complete"] = not coverage_errors(summary, expected, state)
    report["diagnostic_contract"] = "diagnose summary/events; five named parameters, original source-time/raw-wire contract"
    return report


def source_pins():
    paths = [Path(__file__), *(Path(module.__file__) for module in
             (codec, timing, diagnose, dual, imu, live, shadow))]
    return {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute-readonly", action="store_true")
    ap.add_argument("--front-port")
    ap.add_argument("--rear-port")
    ap.add_argument("--expected-uids", type=Path)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--seconds", type=float, default=10.)
    ap.add_argument("--timeout", type=float, default=.25)
    args = ap.parse_args(argv)
    try:
        plan = make_plan(args.seconds, args.timeout, args.front_port, args.rear_port)
        if not args.execute_readonly:
            print(json.dumps(plan, indent=2)); return 0
        require(all(x is not None for x in (args.front_port, args.rear_port, args.expected_uids, args.output)),
                "Execution requires both explicit ports, expected-uids and a fresh private output directory")
        output = private_directory(args.output)
        expected_bytes = args.expected_uids.read_bytes()
        expected_hash = hashlib.sha256(expected_bytes).hexdigest()
        expected = timing.validate_uids(shadow._json(expected_bytes.decode()))
        bindings = dual.validate_ports(args.front_port, args.rear_port)
        boot = _boot_id()
        pins = source_pins()
        output.mkdir(mode=0o700)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    recorder = Recorder(output)
    stop, signals, handlers = threading.Event(), [], {}
    def interrupted(signum, _frame):
        signals.append(signum)
        stop.set()
    report = None
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, interrupted)
        try:
            report = capture(plan, bindings, expected, recorder, stop=stop)
        finally:
            try:
                recorder.close()
            except BaseException as error:
                if report is None:
                    raise
                report["errors"].append({"component": "event_trace", "error": repr(error)})
        if signals:
            report["errors"].append({"component": "process", "error": "Signals: " + repr(signals)})
        try:
            require(_boot_id() == boot, "Jetson boot changed during capture")
            require(args.expected_uids.read_bytes() == expected_bytes, "Expected UID source changed during capture")
            require(source_pins() == pins, "Acquisition source changed during capture")
        except BaseException as error:
            report["errors"].append({"component": "provenance", "error": repr(error)})
        if report["errors"]:
            report["status"] = "INCOMPLETE"
        report.update(boot_id=boot, source_sha256=pins, expected_uids_sha256=expected_hash,
                      ports=bindings, events_sha256=hashlib.sha256((output / "events.jsonl").read_bytes()).hexdigest())
        payload = json.dumps(report, allow_nan=False, indent=2) + "\n"
        fd = os.open(output / "summary.json", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        print(json.dumps({"output": str(output), "status": report["status"], "errors": report["errors"],
                          "imu_samples": report["summary"]["imu"]["samples"],
                          "complete_sweeps": report["acquisition"]["complete_sweeps"], **FLAGS}, indent=2))
        return 1 if report["errors"] else 0
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
