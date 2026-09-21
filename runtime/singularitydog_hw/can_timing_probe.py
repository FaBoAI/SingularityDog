"""Bounded sequential read-only CAN timing probe; not a controller benchmark.

Only identity, position and velocity reads exist. No stop, enable, setting,
report-frequency change or automatic retry is available. A read cannot stop a
motor: the operator must arrange an idle, exclusively owned bus before use.
"""
import argparse
from contextlib import contextmanager
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import time

from .can_readonly import ATParser, ReadOnlyCAN, matches, read_request

IDS = tuple(range(1, 13))
READS = ("position", "velocity")


def validate_uids(values):
    if not isinstance(values, dict) or len(values) != 12:
        raise ValueError("Expected exactly twelve private motor identities")
    if any(type(k) not in (str, int) or str(k) not in {str(i) for i in IDS} for k in values):
        raise ValueError("Expected identity keys1..12")
    values = {int(k): v for k, v in values.items()}
    if set(values) != set(IDS) or any(not isinstance(v, str) or len(v) != 16
            or any(c not in '0123456789abcdef' for c in v) for v in values.values()):
        raise ValueError("Malformed private motor identities")
    if len(set(values.values())) != 12:
        raise ValueError("Expected identities must be unique")
    return values


def make_plan(cycles=10, max_seconds=30.):
    if type(cycles) is not int or not 1 <= cycles <= 100:
        raise ValueError("cycles must be an integer1..100")
    if type(max_seconds) not in (float, int) or not math.isfinite(max_seconds) or not 1 <= max_seconds <= 120:
        raise ValueError("max_seconds must be finite and1..120")
    return {"ids": list(IDS), "cycles": cycles, "reads_per_cycle": 24,
            "max_requests": 12+24*cycles, "max_seconds": max_seconds,
            "parameters": ["identity", *READS], "allowed_can_types": [0, 17],
            "one_outstanding_request": True, "automatic_retry": False,
            "artificial_inter_request_delay": False, "motor_output_available": False,
            "report_frequency_changed": False, "full_controller_50Hz_verified": False}


def distribution(values):
    if not values:
        return None
    values = sorted(values)
    return {"count": len(values), "min": values[0], "median": statistics.median(values),
            "mean": statistics.fmean(values), "p95": values[math.ceil(.95*len(values))-1],
            "max": values[-1]}


class _ReadOnlyPort:
    """Validate the exact bytes again at the physical-write boundary."""
    def __init__(self, port, probe):
        self.port, self.probe = port, probe

    def __getattr__(self, name):
        return getattr(self.port, name)

    def write(self, wire):
        p = self.probe
        p.check_budget()
        p.validate_wire(wire)
        p.write_attempts += 1
        p.write_started_ns = p.clock()
        try:
            return self.port.write(wire)
        finally:
            p.write_finished_ns = p.clock()
            p.output({"kind": "probe_write_timing", "monotonic_ns": p.write_finished_ns,
                      "write_started_monotonic_ns": p.write_started_ns,
                      "write_finished_monotonic_ns": p.write_finished_ns})


class TimingCAN(ReadOnlyCAN):
    """Uses the existing request/reply codec and transport, with stricter auditing."""
    def __init__(self, emit, *, max_requests, max_seconds, check_interrupt=lambda: None,
                 serial_port=None, clock=time.monotonic_ns):
        make_plan(1, max_seconds)
        if type(max_requests) is not int or not 1 <= max_requests <= 2412:
            raise ValueError("max_requests must be an integer1..2412")
        self.output, self.check_interrupt = emit, check_interrupt
        self.started_ns = clock()
        self.deadline_ns = self.started_ns + int(max_seconds*1e9)
        self.max_requests, self.write_attempts = max_requests, 0
        self.pending, self.reply_ns, self.last_timing = None, None, None
        self.write_started_ns = self.write_finished_ns = None
        super().__init__(event_sink=self.record, serial_port=serial_port, clock=clock)

    def __enter__(self):
        super().__enter__()
        self.serial = _ReadOnlyPort(self.serial, self)
        return self

    def check_budget(self):
        self.check_interrupt()
        if self.clock() >= self.deadline_ns:
            raise TimeoutError("Probe wall-time budget exhausted")
        if self.write_attempts >= self.max_requests:
            raise RuntimeError("Probe request-count budget exhausted")

    def validate_wire(self, wire):
        if self.pending is None or self.pending[1] not in (None, *READS):
            raise ValueError("No permitted outstanding read")
        if wire != read_request(*self.pending):
            raise ValueError("Probe only permits canonical identity/position/velocity reads")

    def record(self, event):
        self.output(event)
        self.check_interrupt()
        if event["kind"] == "can_tx":
            self.validate_wire(bytes.fromhex(event["hex"]))
        elif event["kind"] == "can_rx_frame":
            frame = ATParser().feed(bytes.fromhex(event["wire_hex"]))[0]
            if self.pending is None or not matches(frame, *self.pending):
                raise RuntimeError("Unexpected CAN reply during single outstanding read")
            if self.reply_ns is not None:
                raise RuntimeError("Duplicate reply; fresh correlation not established")
            self.reply_ns = event["monotonic_ns"]

    def residual(self):
        return {"serial_pending_bytes": self.serial.in_waiting,
                "parser_pending_bytes": len(self.parser.buffer),
                "parser_discarded_bytes": self.parser.discarded_bytes}

    def query(self, motor_id, parameter=None):
        # No arbitrary-index or alternate-parameter API is exposed by this probe.
        if type(motor_id) is not int or motor_id not in IDS or parameter not in (None, *READS):
            raise ValueError("Only IDs1..12 identity/position/velocity are permitted")
        if self.pending is not None:
            raise RuntimeError("An outstanding query already exists")
        self.last_timing = {"motor_id": motor_id, "parameter": parameter or "identity", "ok": False}
        self.reply_ns = self.write_started_ns = self.write_finished_ns = None
        self.pending = (motor_id, parameter)
        try:
            self.check_budget()
            before = self.residual()
            self.last_timing["residual_before"] = before
            if any(before.values()):
                raise RuntimeError("Nonempty or corrupted initial reply boundary")
            remaining_s = (self.deadline_ns-self.clock())/1e9
            if remaining_s <= .015:
                raise TimeoutError("Insufficient remaining probe time for another bounded query")
            self.timeout_s = min(.25, remaining_s-.005)
            result = super().query(motor_id, parameter)
            self.last_timing.update(sequence=result["sequence"],
                request_monotonic_ns=result["request_monotonic_ns"],
                completed_monotonic_ns=result["monotonic_ns"],
                round_trip_ms=result["round_trip_ms"])
            if self.clock() >= self.deadline_ns:
                raise TimeoutError("Response completed after probe time budget")
            if not result["ok"]:
                raise RuntimeError(f"Rejected or invalid read: ID{motor_id} {parameter}")
            if self.reply_ns is None or self.write_started_ns is None or self.reply_ns < self.write_started_ns:
                raise RuntimeError("Missing physical write/receive timing")
            after = self.residual()
            self.last_timing["residual_after"] = after
            if any(after.values()):
                raise RuntimeError("Residual bytes or parse corruption after reply")
            self.last_timing["ok"] = True
            return result
        except BaseException as error:
            self.poisoned = True
            self.last_timing["error"] = repr(error)
            raise
        finally:
            self.last_timing.update(write_started_monotonic_ns=self.write_started_ns,
                write_finished_monotonic_ns=self.write_finished_ns, received_monotonic_ns=self.reply_ns,
                wire_round_trip_ms=((self.reply_ns-self.write_started_ns)/1e6
                    if self.reply_ns is not None and self.write_started_ns is not None else None))
            try:
                self.last_timing["residual_after"] = self.residual()
            except BaseException as error:
                self.last_timing["residual_inspection_error"] = repr(error)
            self.pending = None


def collect(can, expected_uids, emit, *, cycles):
    make_plan(cycles, 1.)
    expected_uids = validate_uids(expected_uids)
    report = {"status": "INCOMPLETE", "errors": [], "requests": [], "cycles": [],
              "identities_verified": False, "full_controller_50Hz_verified": False}
    def query(mid, parameter=None):
        try:
            return can.query(mid, parameter)
        finally:
            if can.last_timing is not None:
                row = dict(can.last_timing)
                report["requests"].append(row)
                emit({"kind": "probe_request_timing", **row})
    try:
        for mid in IDS:
            result = query(mid)
            if result["mcu_uid_hex"] != expected_uids[mid]:
                raise RuntimeError(f"Identity mismatch: ID{mid}")
        report["identities_verified"] = True
        for cycle in range(cycles):
            started = can.clock()
            rows = []
            for mid in IDS:
                for parameter in READS:
                    query(mid, parameter)
                    rows.append(report["requests"][-1])
            finished = can.clock()
            received = [r["received_monotonic_ns"] for r in rows]
            item = {"cycle": cycle+1, "read_count": len(rows),
                    "started_monotonic_ns": started, "finished_monotonic_ns": finished,
                    "duration_ms": (finished-started)/1e6,
                    "oldest_received_monotonic_ns": min(received),
                    "newest_received_monotonic_ns": max(received),
                    "oldest_newest_spread_ms": (max(received)-min(received))/1e6,
                    "oldest_request_to_newest_reply_ms": (max(received)-rows[0]["write_started_monotonic_ns"])/1e6,
                    "residual": can.residual()}
            report["cycles"].append(item)
            emit({"kind": "probe_cycle", **item})
        report["status"] = "READONLY_TIMING_COMPLETE"
    except BaseException as error:
        report["errors"].append(repr(error))
    report["write_attempts"] = can.write_attempts
    report["elapsed_s"] = (can.clock()-can.started_ns)/1e9
    report["cycles_completed"] = len(report["cycles"])
    try:
        report["residual_final"] = can.residual()
        if any(report["residual_final"].values()):
            report["errors"].append("Residual bytes or parse corruption at end")
    except BaseException as error:
        report["errors"].append("Cannot inspect final receive boundary: " + repr(error))
    report["statistics_ms"] = {
        "request_RTT": distribution([r["round_trip_ms"] for r in report["requests"] if r["ok"]]),
        "parameter_request_RTT": distribution([r["round_trip_ms"] for r in report["requests"]
                                               if r["ok"] and r["parameter"] in READS]),
        "wire_RTT": distribution([r["wire_round_trip_ms"] for r in report["requests"] if r["ok"]]),
        "cycle_duration": distribution([c["duration_ms"] for c in report["cycles"]]),
        "cycle_oldest_newest_spread": distribution([c["oldest_newest_spread_ms"] for c in report["cycles"]])}
    if report["cycles"]:
        span = report["cycles"][-1]["finished_monotonic_ns"]-report["cycles"][0]["started_monotonic_ns"]
        report["observed_complete_cycle_rate_hz"] = len(report["cycles"])*1e9/span if span > 0 else None
    if report["errors"]:
        report["status"] = "INCOMPLETE"
    return report


@contextmanager
def ownership_locks():
    """Cooperating-process locks; serial exclusive remains necessary as well."""
    directory = Path.home()/".cache"/"singularitydog"
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    handles = []
    try:
        for name in ("manual-calibration.lock", "can-readonly.lock"):
            handle = (directory/name).open("a")
            handles.append(handle)
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        for handle in reversed(handles):
            handle.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute-readonly", action="store_true")
    ap.add_argument("--cycles", type=int, default=10)
    ap.add_argument("--max-seconds", type=float, default=30.)
    ap.add_argument("--expected-uids", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    try:
        plan = make_plan(args.cycles, args.max_seconds)
        expected_source = args.expected_uids.read_bytes()
        expected = validate_uids(json.loads(expected_source))
    except (ValueError, OSError) as error:
        ap.error(str(error))
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2))
        return 0
    output = args.output.expanduser().resolve()
    if any((p/".git").exists() for p in (output, *output.parents)) or Path(__file__).resolve().parent.parent in output.parents:
        ap.error("Use a new private output directory outside Git/runtime")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {"status": "INCOMPLETE", "errors": [], "plan": plan,
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "expected_uid_file_sha256": hashlib.sha256(expected_source).hexdigest(),
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (Path(__file__), Path(__file__).with_name("can_readonly.py"))}}
    signals, handlers = [], {}
    def check_interrupt():
        if signals:
            raise InterruptedError(f"signal {signals[0]}")
    old_umask = os.umask(0o077)
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: signals.append(number))
        with (output/"events.jsonl").open("x", buffering=1) as log:
            def emit(event):
                log.write(json.dumps({"wall_time_ns": time.time_ns(), **event}, allow_nan=False)+"\n")
            emit({"kind": "probe_metadata", "monotonic_ns": time.monotonic_ns(), **plan})
            with ownership_locks():
                check_interrupt()
                with TimingCAN(emit, max_requests=plan["max_requests"], max_seconds=args.max_seconds,
                               check_interrupt=check_interrupt) as can:
                    report.update(collect(can, expected, emit, cycles=args.cycles))
            check_interrupt()
    except BaseException as error:
        report["errors"].append(repr(error))
        report["status"] = "INCOMPLETE"
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        report["completed_at"] = datetime.datetime.now().astimezone().isoformat()
        try:
            (output/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        finally:
            os.umask(old_umask)
    print(json.dumps({"output": str(output), "status": report["status"], "errors": report["errors"],
                      "cycles_completed": report.get("cycles_completed", 0),
                      "statistics_ms": report.get("statistics_ms"),
                      "full_controller_50Hz_verified": False}, indent=2))
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
