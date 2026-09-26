"""One finite dual-bus STOP batching comparison; no enable or learned target.

The default only prints a plan. Each selected bus owns its serial FD throughout
UID/voltage/disabled-state preflight, the group write experiment and closure.
Reporting must be OFF. No inference, IMU, continuous control, or motor motion.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time

from .active_report_probe import ActiveProbe, FLAGS, require
from . import dual_can_pipeline_benchmark as dual
from .can_timing_probe import ownership_locks, validate_uids
from .sensor_pipeline_benchmark import BootIdentityGuard
from .stop_batch_probe import StopBatchProbe

_HELD_LOCKS = []


def make_plan(stage="front", group_size=1):
    require(stage in ("front", "rear", "both"), "Unknown scope")
    require(type(group_size) is int and group_size in (1, 2, 3, 6), "Select group size1/2/3/6")
    scopes = {s: list(ids) for s, ids in dual.SCOPES.items() if stage in (s, "both")}
    return {**FLAGS, "output_allowed": False, "stage": stage,
            "group_size": group_size, "ids_by_scope": scopes,
            "passes": 1, "allowed_can_types": [0, 4, 17],
            "reporting_switch_available": False, "period_write_available": False,
            "uart_baudrate": 921600, "known_reporting_off_required": True,
            "scope": "preflight then one all-zero STOP group-write feedback pass",
            "host_write_is_not_can_wire_completion": True,
            "per_frame_transmit_timestamps_available": False,
            "replies_do_not_prove_physical_stop_time": True}


def execute(plan, bindings, expected, boot_id, *, cancelled, pipeline_factory=StopBatchProbe,
            preflight_factory=ActiveProbe, serial_factory=None,
            clock=time.monotonic_ns, boot_reader=None):
    """Own all hardware resources; injected factories are only for offline tests."""
    require(plan == make_plan(plan.get("stage"), plan.get("group_size")), "Plan changed after validation")
    guard, outcome = None, None
    try:
        if boot_reader is None:
            guard = BootIdentityGuard()
            require(guard.boot_id == boot_id, "Boot mismatch")
            boot_check = guard.check
        else:
            # Preserve the explicit offline injection without opening a real FD.
            require(boot_reader() == boot_id, "Boot mismatch")
            def boot_check():
                require(boot_reader() == boot_id, "Boot changed")
        outcome = _execute(plan, bindings, expected, boot_id, cancelled=cancelled,
            pipeline_factory=pipeline_factory, preflight_factory=preflight_factory,
            serial_factory=serial_factory, clock=clock, boot_check=boot_check)
    finally:
        # _execute joins all workers, including startup failures, before returning.
        if guard is not None:
            try:
                guard.close()
            except BaseException as exc:
                if outcome is None:
                    raise
                outcome[0]["status"] = "INCOMPLETE"
                outcome[0]["errors"].append("Boot monitor close failed: " + repr(exc))
    return outcome


def _execute(plan, bindings, expected, boot_id, *, cancelled, pipeline_factory,
             preflight_factory, serial_factory, clock, boot_check):
    expected = validate_uids(expected)
    scopes = plan["ids_by_scope"]
    results, probes, preflights, errors, stamps = {}, {}, {}, [], {}
    lock = threading.Lock()
    deadline = clock() + 20_000_000_000
    def release():
        require(not cancelled.is_set(), "Experiment cancelled")
        stamps["batch_release_ns"] = clock()
    barrier = threading.Barrier(len(scopes), action=release)
    lease = dual.CommonLease(ownership_locks)
    def worker(scope):
        raw, stack = None, ExitStack()
        result = {"status": "INCOMPLETE", "port_closed": False}
        try:
            stack.enter_context(dual.port_lock(bindings[scope]["resolved"]))
            def check():
                require(not cancelled.is_set(), "Experiment cancelled")
                require(clock() < deadline, "Finite20s coordinator budget exhausted")
                boot_check()
                require(dual.binding_matches(bindings[scope]), "Port binding changed")
            check()
            if serial_factory is None:
                import serial
                factory = serial.Serial
            else:
                factory = serial_factory
            raw = factory(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
                          timeout=0, write_timeout=.1, exclusive=True,
                          xonxoff=False, rtscts=False, dsrdtr=False)
            raw.dtr = raw.rts = False
            raw.port = bindings[scope]["path"]
            raw.open()
            require(os.fstat(raw.fileno()).st_rdev == bindings[scope]["st_rdev"],
                    "Opened FD differs")
            preflight = preflight_factory(raw, scopes[scope], expected, seconds=1,
                preflight_only=True, period_policy="observe-current", clock=clock,
                check=lambda cleaning=False: check())
            preflights[scope] = preflight
            result["preflight"] = preflight.run()
            require(result["preflight"]["status"] == "PREFLIGHT_COMPLETE", "Preflight incomplete")
            check()
            barrier.wait(timeout=5)
            check()
            probe = pipeline_factory(raw, tuple(scopes[scope]), group_size=plan["group_size"],
                                     clock=clock, check=check)
            probes[scope] = probe
            result["batch"] = probe.run()
            require(result["batch"]["status"] == "STOP_BATCH_OBSERVATION_COMPLETE", "Batch incomplete")
            check()
            result["status"] = "STOP_BATCH_OBSERVATION_COMPLETE"
        except BaseException as exc:
            result["failure"] = repr(exc)
            cancelled.set(); barrier.abort()
        finally:
            if raw is not None:
                try:
                    raw.close()
                    result["port_closed"] = not raw.is_open
                except BaseException as exc:
                    result.update(status="INCOMPLETE", close_failure=repr(exc))
            else:
                result["port_closed"] = True
            if result["port_closed"]:
                try:
                    stack.close(); lease.release()
                except BaseException as exc:
                    result.update(status="INCOMPLETE", lock_release_failure=repr(exc))
            else:
                _HELD_LOCKS.extend((stack, lease))
            with lock:
                results[scope] = result
    threads = []
    try:
        for scope in scopes:
            lease.retain()
            thread = threading.Thread(target=worker, args=(scope,), name="stop-batch-"+scope)
            try:
                thread.start()
            except BaseException:
                lease.release()
                raise
            threads.append(thread)
        for thread in threads:
            thread.join()
    except BaseException as exc:
        errors.append(repr(exc))
        cancelled.set(); barrier.abort()
    finally:
        for thread in threads:
            thread.join()
        lease.release()
    complete = (not errors and not cancelled.is_set() and set(results) == set(scopes)
                and lease.released and all(r["port_closed"] and
                    r["status"] == "STOP_BATCH_OBSERVATION_COMPLETE" for r in results.values()))
    report = {**plan, "status": "COMPLETE_STOP_BATCH_EXPERIMENT" if complete else "INCOMPLETE",
              "results": results, "errors": errors, "stamps": stamps,
              "boot_id": boot_id, "bindings": bindings, "locks_released": lease.released}
    return report, preflights, probes


def _save(path, value):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as out:
        json.dump(value, out, indent=2, allow_nan=False)
        out.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("front", "rear", "both"), default="front")
    parser.add_argument("--group-size", type=int, choices=(1, 2, 3, 6), default=1)
    parser.add_argument("--front-port")
    parser.add_argument("--rear-port")
    parser.add_argument("--expected-uids", type=Path)
    parser.add_argument("--expected-boot-id")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--known-reporting-off", action="store_true")
    parser.add_argument("--execute-no-motion", action="store_true")
    args = parser.parse_args(argv)
    plan = make_plan(args.stage, args.group_size)
    if not args.execute_no_motion:
        print(json.dumps(plan, indent=2)); return 0
    require(args.known_reporting_off and all((args.front_port, args.rear_port,
            args.expected_uids, args.expected_boot_id, args.output)), "Pinned live arguments required")
    output = args.output.expanduser().resolve()
    require(not any((p / ".git").exists() for p in (output, *output.parents)), "Private output outside Git required")
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    expected = validate_uids(json.loads(args.expected_uids.read_text()))
    output.mkdir(mode=0o700, exist_ok=False)
    cancelled, handlers = threading.Event(), {}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.set())
        report, preflights, probes = execute(plan, bindings, expected, args.expected_boot_id,
                                            cancelled=cancelled)
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    for category, owners in (("preflight", preflights), ("batch", probes)):
        for scope, probe in owners.items():
            _save(output / (scope+"-"+category+"-tx.json"), probe.tx_log)
            _save(output / (scope+"-"+category+"-raw.json"), [
                {"read_started_ns": begin, "received_ns": end, "hex": chunk.hex()}
                for begin, end, chunk in probe.raw_log])
    report["source_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sorted(Path(__file__).parent.glob("*.py"))}
    _save(output / "summary.json", report)
    print(json.dumps({"status": report["status"], "output": str(output), "output_allowed": False}))
    return 0 if report["status"] == "COMPLETE_STOP_BATCH_EXPERIMENT" else 2


if __name__ == "__main__":
    raise SystemExit(main())
