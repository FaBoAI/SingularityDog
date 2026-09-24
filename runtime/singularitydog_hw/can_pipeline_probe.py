"""Bounded read-only CAN pipeline experiment; no motor control or configuration.

One invocation tests one window/gap profile. The protocol has no transaction
sequence number, so a late prior-cycle reply with an identical key cannot be
distinguished after that key is sent again. This is not a controller benchmark.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

from .can_readonly import ATParser, PARAMETERS, decode_reply, matches, read_request
from .can_timing_probe import distribution, ownership_locks, validate_uids

IDS = tuple(range(1, 13))
READS = ("position", "velocity")
REQUEST_TIMEOUT_NS = 250_000_000


def make_plan(window=1, gap_ms=0., cycles=1, max_seconds=10., request_order="interleaved"):
    if type(window) is not int or not 1 <= window <= 12:
        raise ValueError("window must be an integer1..12")
    if type(cycles) is not int or not 1 <= cycles <= 20:
        raise ValueError("cycles must be an integer1..20")
    if request_order not in ("interleaved", "by-parameter"):
        raise ValueError("request_order must be interleaved or by-parameter")
    for value, low, high, name in ((gap_ms, 0, 5, "gap_ms"),
                                   (max_seconds, 1, 30, "max_seconds")):
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{name} must be finite and{low}..{high}")
    return {"ids": list(IDS), "window": window, "gap_ms": gap_ms,
            "request_order": request_order,
            "cycles": cycles, "reads_per_cycle": 24, "max_requests": 12+24*cycles,
            "max_seconds": max_seconds, "request_timeout_seconds": .25,
            "allowed_can_types": [0, 17], "parameters": ["identity", *READS],
            "identity_requests_sequential": True, "automatic_retry": False,
            "motor_output_available": False, "can_bitrate_changed": False,
            "full_controller_50Hz_verified": False,
            "late_same_key_previous_cycle_disambiguation": False}


class _ReadOnlyPort:
    """Canonical allowlist check immediately at the actual serial-write boundary."""
    def __init__(self, port, probe):
        self.port, self.probe = port, probe

    def __getattr__(self, name):
        return getattr(self.port, name)

    def write(self, wire):
        self.probe.check_budget()
        self.probe.validate_wire(wire)
        row = self.probe.pending[self.probe.current_write]
        row["write_call_entered"] = True
        returned = self.port.write(wire)
        row["write_returned_bytes"] = returned
        return returned


class PipelineCAN:
    def __init__(self, emit, *, window=1, gap_ms=0., cycles=1, max_seconds=10.,
                 request_order="interleaved", serial_port=None, clock=time.monotonic_ns,
                 check_interrupt=lambda: None, receive_mode="serial"):
        if receive_mode not in ("serial", "select"):
            raise ValueError("receive_mode must be serial or select")
        self.plan = make_plan(window, gap_ms, cycles, max_seconds, request_order)
        self.plan["receive_mode"] = receive_mode
        self.emit, self.clock, self.check_interrupt = emit, clock, check_interrupt
        self.receive_mode, self.deadline_reader = receive_mode, None
        self._receive_wake_ns = None
        self.read_profile = {"calls": 0, "wall_ns": 0, "thread_cpu_ns": 0}
        self.started_ns = clock()
        self.deadline_ns = self.started_ns + int(max_seconds*1e9)
        self.raw_port, self.serial = serial_port, None
        self.parser = ATParser()
        self.pending, self.completed_keys = {}, set()
        self.current_write = None
        self.write_attempts = self.replies = self.rx_bytes = 0
        self.max_observed_pending = 0
        self.last_write_finished_ns = None
        self.rows, self.cycle_rows = [], []
        self.poisoned = False
        self.identities_verified = False

    def __enter__(self):
        self.check_budget()
        if self.raw_port is None:
            import serial
            port = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N",
                                 stopbits=1, timeout=.003, write_timeout=.1,
                                 xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
            port.dtr = port.rts = False
            port.port = "/dev/robstride-usb2can"
            self.raw_port = port
            try:
                port.open()
            except BaseException:
                port.close()
                raise
        self.serial = _ReadOnlyPort(self.raw_port, self)
        try:
            self.configure_receiver()
            self.check_budget()
        except BaseException:
            self.raw_port.close()
            self.poisoned = True
            raise
        return self

    def __exit__(self, *_):
        self.poisoned = True
        if self.raw_port is not None:
            self.raw_port.close()

    def configure_receiver(self):
        if self.receive_mode == "select":
            from .serial_deadline_reader import DeadlineSerialReader
            self.deadline_reader = DeadlineSerialReader(
                self.raw_port, clock=self.clock, check=self.check_budget)

    def _read_chunk(self, timeout_s):
        """Preserve RX time and, in select mode, the scheduler's absolute wake."""
        started, cpu_started = self.clock(), time.thread_time_ns()
        try:
            self.check_budget()
            now = self.clock()
            nearest = min([self.deadline_ns,
                           *(r["deadline_monotonic_ns"] for r in self.pending.values())])
            if self.receive_mode == "select":
                if self.deadline_reader is None:
                    raise RuntimeError("Deadline receiver not configured")
                wake = min(nearest, started + max(0, int(timeout_s*1e9)))
                if self._receive_wake_ns is not None:
                    wake = min(wake, self._receive_wake_ns)
                return self.deadline_reader.read_until(wake, nearest)
            read_timeout = min(timeout_s, max(0., (nearest-now)/1e9))
            if self.raw_port.timeout != read_timeout:
                self.raw_port.timeout = read_timeout
            chunk = self.serial.read(min(max(self.raw_port.in_waiting, 1), 4096))
            return chunk, self.clock()
        finally:
            self.read_profile["calls"] += 1
            self.read_profile["wall_ns"] += self.clock()-started
            self.read_profile["thread_cpu_ns"] += time.thread_time_ns()-cpu_started

    def receiver_profile(self):
        return {"mode": self.receive_mode, "read_wrapper": dict(self.read_profile),
                "deadline_reader": self.deadline_reader.stats() if self.deadline_reader else None}

    def check_budget(self):
        self.check_interrupt()
        if self.poisoned:
            raise RuntimeError("Pipeline session is closed or failed")
        now = self.clock()
        if now >= self.deadline_ns:
            raise TimeoutError("Pipeline wall-time budget exhausted")
        if any(now >= row["deadline_monotonic_ns"] for row in self.pending.values()):
            raise TimeoutError("Missing reply within request deadline")

    def residual(self):
        return {"serial_pending_bytes": self.raw_port.in_waiting,
                "parser_pending_bytes": len(self.parser.buffer),
                "parser_discarded_bytes": self.parser.discarded_bytes}

    def clean_boundary(self):
        self.check_budget()
        if self.pending or any(self.residual().values()):
            raise RuntimeError("Nonempty or corrupted cycle boundary")

    def validate_wire(self, wire):
        key = self.current_write
        if key is None or key not in self.pending or key[1] not in (None, *READS):
            raise ValueError("No permitted pending read at write boundary")
        if type(key[0]) is not int or key[0] not in IDS or wire != read_request(*key):
            raise ValueError("Only canonical identity/position/velocity reads are permitted")

    def _send(self, key, cycle):
        self.check_budget()
        mid, parameter = key
        if type(mid) is not int or mid not in IDS or parameter not in (None, *READS):
            raise ValueError("Only IDs1..12 identity/position/velocity reads exist")
        if key in self.pending or key in self.completed_keys:
            raise RuntimeError("Repeated request key in the same cycle")
        if self.write_attempts >= self.plan["max_requests"]:
            raise RuntimeError("Pipeline request-count budget exhausted")
        wire = read_request(mid, parameter)
        row = {"sequence": self.write_attempts+1, "cycle": cycle, "motor_id": mid,
               "parameter": parameter or "identity", "ok": False,
               "write_expected_bytes": len(wire), "write_returned_bytes": None,
               "write_call_entered": False,
               "write_started_monotonic_ns": None, "write_finished_monotonic_ns": None,
               "received_monotonic_ns": None}
        self.emit({"kind": "pipeline_tx_intent", "monotonic_ns": self.clock(),
                   **row, "wire_hex": wire.hex()})
        self.check_budget()  # A slow logger or an interrupt must not permit a late write.
        started = self.clock()
        row["write_started_monotonic_ns"] = started
        row["deadline_monotonic_ns"] = min(self.deadline_ns, started+REQUEST_TIMEOUT_NS)
        nearest = min([row["deadline_monotonic_ns"],
                       *(r["deadline_monotonic_ns"] for r in self.pending.values())])
        write_timeout = min(.1, (nearest-started)/1e9)
        # pyserial reconfigures an open port even for an unchanged timeout.
        # Read the public property rather than caching it; deadline reductions
        # (and a setting changed by a caller) must still take effect.
        if self.raw_port.write_timeout != write_timeout:
            self.raw_port.write_timeout = write_timeout
        self.pending[key] = row
        self.rows.append(row)
        self.current_write = key
        self.write_attempts += 1
        self.max_observed_pending = max(self.max_observed_pending, len(self.pending))
        try:
            # The wrapper revalidates these bytes at the actual I/O boundary.
            returned = self.serial.write(wire)
            if type(returned) is not int or returned != len(wire):
                raise IOError("Partial serial write")
        finally:
            finished = self.clock()
            row["write_finished_monotonic_ns"] = finished
            self.last_write_finished_ns = finished
            self.current_write = None
            self.emit({"kind": "pipeline_write_timing", **row})
        self.check_budget()

    def _receive(self, timeout_s, expected_uids):
        chunk, received = self._read_chunk(timeout_s)
        if not chunk:
            self.check_budget()
            return
        self.rx_bytes += len(chunk)
        self.emit({"kind": "pipeline_rx_bytes", "monotonic_ns": received, "hex": chunk.hex()})
        frames = self.parser.feed(chunk)
        if self.parser.discarded_bytes:
            raise RuntimeError("Malformed or discarded serial bytes")
        for frame in frames:
            self.emit({"kind": "pipeline_rx_frame", "monotonic_ns": received, **frame.record()})
            if frame.kind == 0:
                key = (frame.source, None)
            elif frame.kind == 17 and len(frame.data) == 8:
                index = int.from_bytes(frame.data[:2], "little")
                name = next((p for p in READS if PARAMETERS[p][0] == index), None)
                key = (frame.source, name) if name is not None else None
            else:
                key = None
            if key in self.completed_keys:
                raise RuntimeError("Duplicate reply to an already completed key")
            if key not in self.pending or not matches(frame, *key):
                raise RuntimeError("Unsolicited, invalid or unmatched reply")
            row = self.pending[key]
            if received >= row["deadline_monotonic_ns"]:
                raise TimeoutError("Reply arrived after its deadline")
            result = decode_reply(frame, *key)
            if not result["ok"]:
                raise RuntimeError("Rejected or nonfinite parameter reply")
            if key[1] is None and result["mcu_uid_hex"] != expected_uids[key[0]]:
                raise RuntimeError(f"Identity mismatch: ID{key[0]}")
            row.update(ok=True, received_monotonic_ns=received,
                       wire_round_trip_ms=(received-row["write_started_monotonic_ns"])/1e6,
                       write_duration_ms=(row["write_finished_monotonic_ns"]-row["write_started_monotonic_ns"])/1e6)
            self.emit({"kind": "pipeline_reply", **row, "result": result})
            self.replies += 1
            self.completed_keys.add(key)
            del self.pending[key]
        if self.parser.buffer and not self.pending:
            raise RuntimeError("Partial or residual reply with no outstanding request")
        self.check_budget()

    def _batch(self, keys, *, window, cycle, expected_uids):
        self.clean_boundary()
        self.completed_keys = set()
        next_index = 0
        while next_index < len(keys) or self.pending:
            self.check_budget()
            self._receive_wake_ns = None
            # Drain available replies before generating another request, including duplicates.
            if self.raw_port.in_waiting:
                self._receive(0., expected_uids)
                continue
            now = self.clock()
            eligible = ((self.last_write_finished_ns or now) + int(self.plan["gap_ms"]*1e6)
                        if self.last_write_finished_ns is not None else now)
            can_send = next_index < len(keys) and len(self.pending) < window
            if can_send and now >= eligible:
                self._send(keys[next_index], cycle)
                next_index += 1
                continue
            wait_s = .003
            if can_send:
                wait_s = min(wait_s, max(0., (eligible-now)/1e9))
                self._receive_wake_ns = eligible
            self._receive(wait_s, expected_uids)
        self._receive_wake_ns = None
        self.clean_boundary()

    def collect(self, expected_uids):
        report = {"status": "INCOMPLETE", "errors": [], "plan": self.plan,
                  "requests": self.rows, "cycles": self.cycle_rows,
                  "full_controller_50Hz_verified": False}
        current_cycle = None
        try:
            expected_uids = validate_uids(expected_uids)
            self._batch([(i, None) for i in IDS], window=1, cycle=0, expected_uids=expected_uids)
            self.identities_verified = True
            for number in range(1, self.plan["cycles"]+1):
                self.check_budget()
                started = self.clock()
                current_cycle = {"cycle": number, "status": "INCOMPLETE",
                                 "started_monotonic_ns": started}
                self.cycle_rows.append(current_cycle)
                keys = ([(i, p) for p in READS for i in IDS]
                        if self.plan["request_order"] == "by-parameter"
                        else [(i, p) for i in IDS for p in READS])
                self._batch(keys, window=self.plan["window"],
                            cycle=number, expected_uids=expected_uids)
                rows = [r for r in self.rows if r["cycle"] == number]
                received = [r["received_monotonic_ns"] for r in rows]
                finished = self.clock()
                current_cycle.update(status="COMPLETE", finished_monotonic_ns=finished,
                    requests=len(rows), replies=sum(r["ok"] for r in rows),
                    duration_ms=(finished-started)/1e6,
                    oldest_newest_spread_ms=(max(received)-min(received))/1e6,
                    oldest_request_to_newest_reply_ms=(max(received)-rows[0]["write_started_monotonic_ns"])/1e6,
                    residual=self.residual())
                self.emit({"kind": "pipeline_cycle", **current_cycle})
                current_cycle = None
            self.check_budget()
            report["status"] = "READONLY_PIPELINE_COMPLETE"
        except BaseException as error:
            self.poisoned = True
            report["errors"].append(repr(error))
            for row in self.pending.values():
                row.setdefault("error", repr(error))
            if current_cycle is not None:
                current_cycle.update(error=repr(error), finished_monotonic_ns=self.clock(),
                                     requests=sum(r["cycle"] == current_cycle["cycle"] for r in self.rows),
                                     replies=sum(r["cycle"] == current_cycle["cycle"] and r["ok"] for r in self.rows))
        report.update(identities_verified=self.identities_verified, write_attempts=self.write_attempts,
                      replies=self.replies, rx_bytes=self.rx_bytes, max_observed_pending=self.max_observed_pending,
                      elapsed_s=(self.clock()-self.started_ns)/1e9,
                      pending_at_end=[{"motor_id": k[0], "parameter": k[1] or "identity"} for k in self.pending])
        report["receiver_profile"] = self.receiver_profile()
        try:
            report["residual_final"] = self.residual()
            if any(report["residual_final"].values()):
                report["errors"].append("Residual bytes or parse corruption at final boundary")
        except BaseException as error:
            report["errors"].append("Residual inspection failed: "+repr(error))
        complete = [c for c in self.cycle_rows if c["status"] == "COMPLETE"]
        report["cycles_completed"] = len(complete)
        report["statistics_ms"] = {
            "request_wire_RTT": distribution([r["wire_round_trip_ms"] for r in self.rows if r["ok"]]),
            "parameter_wire_RTT": distribution([r["wire_round_trip_ms"] for r in self.rows if r["ok"] and r["cycle"]]),
            "cycle_duration": distribution([c["duration_ms"] for c in complete]),
            "cycle_oldest_newest_spread": distribution([c["oldest_newest_spread_ms"] for c in complete])}
        if report["errors"]:
            report["status"] = "INCOMPLETE"
        return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--window", type=int, default=1)
    parser.add_argument("--gap-ms", type=float, default=0.)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--max-seconds", type=float, default=10.)
    parser.add_argument("--request-order", choices=("interleaved", "by-parameter"),
                        default="interleaved")
    parser.add_argument("--receive-mode", choices=("serial", "select"), default="serial")
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        plan = make_plan(args.window, args.gap_ms, args.cycles, args.max_seconds,
                         args.request_order)
        plan["receive_mode"] = args.receive_mode
        uid_bytes = args.expected_uids.expanduser().read_bytes()
        expected = validate_uids(json.loads(uid_bytes))
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2))
        return 0
    output = args.output.expanduser().resolve()
    if any((p/".git").exists() for p in (output, *output.parents)) or Path(__file__).resolve().parent.parent in output.parents:
        parser.error("Use a new private output directory outside Git/runtime")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {"status": "INCOMPLETE", "errors": [], "plan": plan,
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "expected_uid_file_sha256": hashlib.sha256(uid_bytes).hexdigest(),
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (Path(__file__), Path(__file__).with_name("can_readonly.py"), Path(__file__).with_name("can_timing_probe.py"),
                  Path(__file__).with_name("serial_deadline_reader.py"))}}
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
            with ownership_locks():
                check_interrupt()
                with PipelineCAN(emit, window=args.window, gap_ms=args.gap_ms, cycles=args.cycles,
                                 max_seconds=args.max_seconds, request_order=args.request_order,
                                 check_interrupt=check_interrupt, receive_mode=args.receive_mode) as probe:
                    report.update(probe.collect(expected))
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
    # Raw frames and identities are private. Even arbitrary exception text stays in the private report.
    print(json.dumps({"output": str(output), "status": report["status"],
                      "error_count": len(report["errors"]), "cycles_completed": report.get("cycles_completed", 0),
                      "write_attempts": report.get("write_attempts", 0), "replies": report.get("replies", 0),
                      "statistics_ms": report.get("statistics_ms"),
                      "full_controller_50Hz_verified": False}, indent=2))
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
