"""Finite recovery of reporting left ON by an identified failed no-motion run.

Sends one paced reporting-OFF per selected ID. Only after100ms silence may
identity reads and all-zero STOP be sent. Never ON, Type1/3/18, or automatic
retry. Type2 observations after OFF are recorded, never labelled STOP ACKs.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import time

from . import active_report_probe as active
from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from .active_report_protocol import reporting_request
from .can_timing_probe import ownership_locks, validate_uids
from .rs05_trial_protocol import decode_type2
from .serial_deadline_reader import DeadlineSerialReader


OFF_SPACING_NS = 20_000_000
OFF_OBSERVATION_NS = 50_000_000
MAX_RUN_NS = 8_000_000_000
_HELD_LOCKS = []
require = active.require


def validate_evidence(summary, tx, *, scope, ids, boot_id):
    """Validate frozen failed-run evidence; do not infer missing port provenance."""
    require(scope in dual.SCOPES, "Unknown recovery scope")
    require(type(ids) is tuple and ids and tuple(sorted(set(ids))) == ids
            and all(type(mid) is int and mid in dual.SCOPES[scope] for mid in ids),
            "Recovery IDs must be a sorted unique selected-bus subset")
    require(summary.get("status") == "INCOMPLETE" and summary.get("boot_id") == boot_id,
            "Failed evidence or boot does not match")
    result = summary.get("results", {}).get(scope, {})
    require(result.get("identities_verified") is True and result.get("port_closed") is True,
            "Failed scope must have verified identities and a confirmed closed port")
    require(result.get("transport_poisoned") is False,
            "Cannot recover an uncertain UART frame boundary")
    require(summary.get("common_locks_released") is True,
            "Prior common ownership release is unconfirmed")
    require(summary.get("plan", {}).get("period_policy") == "observe-current",
            "Recovery cannot restore or conceal changed period settings")
    require(set(ids) <= set(result.get("ids", [])), "IDs exceed failed scope")
    require(type(tx) is list and len(tx) <= 200, "Malformed prior TX evidence")
    attempted = set()
    for row in tx:
        if row.get("action") == "report_on":
            mid = row.get("motor_id")
            require(type(mid) is int and mid in dual.SCOPES[scope], "Invalid ON evidence ID")
            require(row.get("wire_hex") == reporting_request(mid, True).hex()
                    and type(row.get("returned_bytes")) is int and row["returned_bytes"] == 17,
                    "Prior ON frame/complete-write evidence differs")
            attempted.add(mid)
    require(set(ids) <= attempted, "Recovery ID has no prior canonical reporting-ON attempt")
    return {"scope": scope, "ids": list(ids), "boot_id": boot_id,
            "previous_port_binding_recorded": False,
            "port_basis": "explicit current by-path mapping; historical paths absent from failed summary"}


class RecoveryProbe(active.ActiveProbe):
    """One owner, no retry; a failed/partial write forbids every later write."""
    def __init__(self, raw, ids, expected, *, clock=time.monotonic_ns,
                 check=lambda cleaning=False: None, reader_factory=DeadlineSerialReader):
        self.recovery_phase = "off"
        self.off_attempted = set()
        super().__init__(raw, ids, expected, seconds=1, expected_kind=24,
                         period_policy="observe-current", clock=clock, check=check,
                         reader_factory=reader_factory)
        self.deadline = self.started + MAX_RUN_NS
        self.recovery_parser = codec.ATParser()
        self.off_windows, self.off_observations = {}, {}
        self.periodic_counts = dict.fromkeys(self.ids, 0)
        self.report = {**active.FLAGS, "status": "INCOMPLETE", "ids": list(self.ids),
                       "off_observations_are_stop_ack": False,
                       "reporting_switch_readback_available": False, "cleanup": {}}

    def _send(self, mid, action, value=None):
        require(value is None, "Recovery has no parameter-write interface")
        if self.recovery_phase == "off":
            require(action == "report_off" and mid not in self.off_attempted,
                    "Only one reporting-OFF per ID is permitted before quiet")
            self.off_attempted.add(mid)
        else:
            require(self.recovery_phase == "verified_quiet" and action in ("identity", "stop"),
                    "Only identity and STOP are permitted after verified silence")
        return super()._send(mid, action)

    def _receive_recovery(self, wake):
        chunk, started, received = self._read(wake)
        frames = self.recovery_parser.feed(chunk)
        require(not self.recovery_parser.discarded_bytes, "Malformed recovery stream")
        for frame in frames:
            require(frame.flags == 4 and len(frame.data) == 8 and frame.source in self.ids
                    and frame.destination == codec.HOST_ID and frame.kind in (2, 24),
                    "Unexpected recovery frame addressing/type/DLC")
            require(((frame.can_id >> 22) & 3) == 0 and ((frame.can_id >> 16) & 63) == 0,
                    "Recovery requires mode0/fault0")
            if frame.kind == 24:
                self.periodic_counts[frame.source] += 1
            else:
                decode_type2(frame, motor_id=frame.source)
                require(frame.source in self.off_windows, "Type2 before this ID's OFF attempt")
                lower, upper = self.off_windows[frame.source]
                require(lower <= received < upper and frame.source not in self.off_observations,
                        "Duplicate or late Type2 OFF observation")
                self.off_observations[frame.source] = {"motor_id": frame.source,
                    "received_ns": received, "wire_hex": frame.wire.hex(),
                    "stop_ack": False}
        return bool(chunk), received

    def run(self):
        try:
            for mid in self.ids:
                begin = self.clock()
                self.off_windows[mid] = (begin, begin + OFF_OBSERVATION_NS)
                self._send(mid, "report_off")
                # Always receive for20ms after write completion, even if Type2
                # arrives immediately. A missing observation gets at most50ms.
                earliest_next = self.clock() + OFF_SPACING_NS
                # Bound the observation from completed host writing. A slow
                # successful write must not put a later receive wake in the past.
                self.off_windows[mid] = (begin, self.clock() + OFF_OBSERVATION_NS)
                while self.clock() < earliest_next or (mid not in self.off_observations
                                                      and self.clock() < self.off_windows[mid][1]):
                    self._receive_recovery(min(self.off_windows[mid][1],
                                               max(earliest_next, self.clock() + 1_000_000)))
            quiet_deadline = min(self.deadline - 1, self.clock() + 1_000_000_000)
            quiet_until = self.clock() + active.QUIET_NS
            while self.clock() < quiet_until:
                require(self.clock() < quiet_deadline, "Reporting did not stop within finite drain")
                data, received = self._receive_recovery(min(quiet_until, quiet_deadline))
                if data:
                    quiet_until = received + active.QUIET_NS
            require(not self.recovery_parser.buffer, "Partial frame at quiet boundary")
            require(set(self.off_observations) == set(self.ids), "Missing OFF Type2 observation")
            self.report["quiet_verified_ns"] = self.clock()
            self.recovery_phase = "verified_quiet"
            for mid in self.ids:
                require(self.query(mid, "identity")["mcu_uid_hex"] == self.expected[mid],
                        "Post-quiet UID mismatch")
            self.report["identities_reverified"] = True
            for mid in self.ids:
                self.query(mid, "stop")
            self.quiet(initial=True)
            self.report.update(status="REPORTING_OFF_RECOVERY_COMPLETE", reporting_off_observed=True)
        except BaseException as error:
            self.report["failure"] = repr(error)
        self.report.update(off_attempted=sorted(self.off_attempted),
            off_observations=list(self.off_observations.values()), periodic_counts=self.periodic_counts,
            stop_observations=self.stop_observations, transport_poisoned=self.transport_poisoned,
            raw_bytes=self.raw_bytes, tx_attempts=len(self.tx_log),
            elapsed_s=(self.clock() - self.started) / 1e9)
        return self.report


def _save(path, value):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--failed-summary", type=Path, required=True)
    parser.add_argument("--scope", choices=("front", "rear"), required=True)
    parser.add_argument("--ids", nargs="+", type=int, required=True)
    parser.add_argument("--front-port", required=True)
    parser.add_argument("--rear-port", required=True)
    parser.add_argument("--expected-boot-id", required=True)
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute-no-motion", action="store_true")
    args = parser.parse_args(argv)
    source = args.failed_summary.read_bytes()
    tx_path = args.failed_summary.parent / (args.scope + "-tx.json")
    tx_source = tx_path.read_bytes()
    evidence = validate_evidence(json.loads(source), json.loads(tx_source), scope=args.scope,
                                 ids=tuple(args.ids), boot_id=args.expected_boot_id)
    expected_source = args.expected_uids.read_bytes()
    expected = validate_uids(json.loads(expected_source))
    plan = {**active.FLAGS, **evidence, "allowed_can_types": [0, 4, 24],
            "off_spacing_ms": 20, "off_observation_limit_ms": 50, "quiet_ms": 100,
            "max_seconds": 8, "automatic_retry": False,
            "failed_summary_sha256": hashlib.sha256(source).hexdigest(),
            "failed_tx_sha256": hashlib.sha256(tx_source).hexdigest(),
            "expected_uids_sha256": hashlib.sha256(expected_source).hexdigest()}
    if not args.execute_no_motion:
        print(json.dumps(plan, indent=2)); return 0
    require(args.output is not None, "New private output directory is required")
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    boot_path = Path("/proc/sys/kernel/random/boot_id")
    require(boot_path.read_text().strip() == args.expected_boot_id, "Boot mismatch")
    output = args.output.expanduser().resolve()
    require(not any((p / ".git").exists() for p in (output, *output.parents)), "Private output outside Git required")
    output.mkdir(mode=0o700, exist_ok=False)
    result = {"status": "INCOMPLETE", "plan": plan, "bindings": bindings,
              "port_closed": False, "locks_released": False}
    signals, handlers, raw, recovery = [], {}, None, None
    stack = ExitStack()
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: signals.append(number))
        stack.enter_context(ownership_locks())
        stack.enter_context(dual.port_lock(bindings[args.scope]["resolved"]))
        def check(cleaning=False):
            require(not signals, "Recovery interrupted; no automatic retry")
            require(boot_path.read_text().strip() == args.expected_boot_id, "Boot changed")
            require(dual.binding_matches(bindings[args.scope]), "Port mapping changed")
        check()
        import serial
        raw = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
                            timeout=0, write_timeout=.1, exclusive=True,
                            xonxoff=False, rtscts=False, dsrdtr=False)
        raw.dtr = raw.rts = False
        raw.port = bindings[args.scope]["path"]
        raw.open()
        require(os.fstat(raw.fileno()).st_rdev == bindings[args.scope]["st_rdev"], "Opened FD differs")
        recovery = RecoveryProbe(raw, tuple(args.ids), expected, check=check)
        result.update(recovery.run())
    except BaseException as error:
        result.update(status="INCOMPLETE", failure=repr(error))
    finally:
        if raw is not None:
            try:
                raw.close()
                result["port_closed"] = not raw.is_open
            except BaseException as error:
                result.update(status="INCOMPLETE", close_failure=repr(error))
        else:
            result["port_closed"] = True
        if result["port_closed"]:
            try:
                stack.close()
                result["locks_released"] = True
            except BaseException as error:
                result.update(status="INCOMPLETE", lock_failure=repr(error))
        else:
            _HELD_LOCKS.append(stack)
            result["status"] = "INCOMPLETE"
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if recovery is not None:
        _save(output / "tx.json", recovery.tx_log)
        fd = os.open(output / "raw.jsonl", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            for start, end, chunk in recovery.raw_log:
                stream.write(json.dumps({"read_started_ns": start, "received_ns": end, "hex": chunk.hex()}) + "\n")
            stream.flush(); os.fsync(stream.fileno())
    result["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    _save(output / "summary.json", result)
    print(json.dumps({"status": result["status"], "output": str(output)}))
    return 0 if result["status"] == "REPORTING_OFF_RECOVERY_COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
