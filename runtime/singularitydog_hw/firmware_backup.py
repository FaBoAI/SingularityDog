"""Finite known-parameter backup before a separately managed firmware update.

Run the frozen r7 ActiveProbe preflight-only/read-versions immediately beforehand
to establish disabled mode0/fault0 and reporting OFF. This tool does not establish
disabled state and cannot send STOP, enable, settings, ID/zero changes, or flash.
run_mode is a control-mode enum, not an enable-state observation; zero_state is
an enum, not an export of motor zero offsets or factory calibration. These reads
are not a full configuration/NVM backup and cannot restore a flashed motor.

Default CLI prints a plan. Execution owns front then rear sequentially, with one
outstanding request and no retry. Raw evidence is buffered and saved after all
port owners finish; a failed port close retains its port and common locks.
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

from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from .can_timing_probe import ownership_locks, validate_uids
from .sensor_pipeline_benchmark import BootIdentityGuard
from .serial_deadline_reader import DeadlineSerialReader, record_failed_read

PARAMETERS = ("run_mode", "position", "current", "velocity", "voltage", "can_timeout", "zero_state")
IDS_BY_SCOPE = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}
QUERY_NS = 250_000_000
MAX_RUN_NS = 20_000_000_000
QUIET_NS = 100_000_000
MAX_RAW_BYTES = 65_536
MAX_RAW_CHUNKS = 1024
MAX_READ_CALLS = 4096
FLAGS = dict(output_allowed=False, motor_enabling_available=False, motion_command_available=False,
             stop_command_available=False, settings_write_available=False, firmware_update_available=False,
             full_configuration_backup=False, zero_calibration_backup=False,
             disabled_state_verified_by_backup=False, calibration_modified=False,
             automatic_retry=False, pure_read_only=True, prior_r7_disabled_preflight_required=True)
_HELD_LOCKS = []


def require(condition, message):
    if not condition:
        raise ValueError(message)


def make_plan():
    return {**FLAGS, "ids_by_scope": {s: list(ids) for s, ids in IDS_BY_SCOPE.items()},
            "parameters": list(PARAMETERS), "allowed_can_types": [0, 17],
            "max_seconds": 20, "query_timeout_ms": 250, "uart_baudrate": 921600,
            "requests_per_bus": 48, "total_request_limit": 96,
            "bus_order": ["front", "rear"], "per_id_order": ["identity", *PARAMETERS],
            "required_initial_and_final_quiet_ms": 100,
            "run_mode_is_enable_state": False, "zero_state_is_zero_offset": False}


class BackupProbe:
    """One finite bus pass; all I/O dependencies are injectable for offline tests."""
    def __init__(self, raw, ids, expected, *, clock=time.monotonic_ns, check=lambda: None,
                 deadline_ns=None, reader_factory=DeadlineSerialReader):
        require(type(ids) is tuple and bool(ids) and all(type(mid) is int for mid in ids)
                and ids == tuple(sorted(set(ids)))
                and any(set(ids) <= set(scope) for scope in IDS_BY_SCOPE.values()), "Invalid bus IDs")
        self.expected = validate_uids(expected)
        self.raw, self.ids, self.clock, self.check = raw, ids, clock, check
        self.deadline_ns, self.reader_factory = deadline_ns, reader_factory
        self.parser = codec.ATParser()
        self.raw_log, self.tx_log = [], []
        self.raw_bytes = self.read_calls = self._next = 0
        self._schedule = tuple((mid, name) for mid in ids for name in (None, *PARAMETERS))
        self._ran = self.tx_prohibited = self.transport_poisoned = False
        self._last_ns = None
        self.report = {**FLAGS, "status": "INCOMPLETE", "ids": list(ids), "motors": {},
                       "initial_quiet_observed": False, "final_quiet_observed": False}

    def _now(self):
        now = self.clock()
        require(type(now) is int and 0 <= now < 2**63
                and (self._last_ns is None or now >= self._last_ns), "Invalid monotonic clock")
        self._last_ns = now
        return now

    def _guard(self):
        self.check()
        now = self._now()
        if now >= self.deadline_ns:
            raise TimeoutError("Backup overall20s deadline exhausted")
        return now

    def _read(self, wake):
        begin = self._guard()
        self.read_calls += 1
        require(self.read_calls <= MAX_READ_CALLS, "Bounded receive-call budget exhausted")
        try:
            chunk, received = self.reader.read_until(min(wake, self.deadline_ns - 1), self.deadline_ns)
        except BaseException as error:
            self.raw_bytes = record_failed_read(error, report=self.report, raw_log=self.raw_log,
                raw_bytes=self.raw_bytes, read_started_ns=begin,
                max_raw_bytes=MAX_RAW_BYTES, max_raw_chunks=MAX_RAW_CHUNKS)
            raise
        require(type(chunk) is bytes and type(received) is int and begin <= received <= self._now(),
                "Invalid host read interval")
        if chunk:
            self.raw_bytes += len(chunk)
            if self.raw_bytes > MAX_RAW_BYTES or len(self.raw_log) >= MAX_RAW_CHUNKS:
                self.report.update(raw_log_overflow=True, unlogged_chunk_bytes=len(chunk))
                raise RuntimeError("Bounded raw evidence buffer exhausted")
            self.raw_log.append((begin, received, chunk))
        self._guard()
        return chunk, received

    def _quiet(self):
        require(not self.parser.buffer, "Partial frame at quiet boundary")
        end = self._guard() + QUIET_NS
        while self._guard() < end:
            chunk, _ = self._read(end)
            if chunk:
                self.parser.feed(chunk)
            require(not chunk, "Unsolicited bytes during backup quiet boundary")

    def _send(self, mid, parameter):
        require(self._ran and not self.tx_prohibited and not self.transport_poisoned,
                "Further backup transmission prohibited")
        require(self._next < len(self._schedule) and (mid, parameter) == self._schedule[self._next],
                "Request outside fixed backup schedule")
        require(type(mid) is int and parameter in (None, *PARAMETERS), "Read-only allowlist violation")
        require(not self.parser.buffer, "Partial frame before next query")
        # Poll for late/unsolicited bytes before starting a new transaction.
        chunk, _ = self._read(self._guard())
        if chunk:
            self.parser.feed(chunk)
        require(not chunk, "Unexpected bytes before next query")
        wire = codec.read_request(mid, parameter)
        kind = 0 if parameter is None else 17
        payload = bytes(8) if parameter is None else codec.PARAMETERS[parameter][0].to_bytes(2, "little") + bytes(6)
        canonical = (b"AT" + ((((kind << 24) | (codec.HOST_ID << 8) | mid) << 3) | 4).to_bytes(4, "big")
                     + b"\x08" + payload + b"\r\n")
        require(type(wire) is bytes and wire == canonical, "Noncanonical read at physical write boundary")
        now = self._guard()
        self.raw.write_timeout = min(.1, (self.deadline_ns - now) / 1e9)
        self._guard()
        row = {"motor_id": mid, "parameter": parameter or "identity", "wire_hex": wire.hex(),
               "started_ns": self._now(), "finished_ns": None, "returned_bytes": None}
        self.tx_log.append(row)
        self._next += 1
        try:
            row["returned_bytes"] = self.raw.write(wire)
            if type(row["returned_bytes"]) is not int or row["returned_bytes"] != len(wire):
                raise IOError("Partial UART write; further transmission prohibited")
        except BaseException:
            self.transport_poisoned = self.tx_prohibited = True
            raise
        finally:
            row["finished_ns"] = self._now()
        self._guard()
        return row

    def _query(self, mid, parameter):
        row = self._send(mid, parameter)
        end = min(row["started_ns"] + QUERY_NS, self.deadline_ns)
        row["deadline_ns"] = end
        while self._guard() < end:
            chunk, received = self._read(end)
            if not chunk:
                continue
            require(received < end, "Query reply arrived after deadline")
            frames = self.parser.feed(chunk)
            require(not self.parser.discarded_bytes, "Malformed/discarded reply bytes")
            if frames:
                require(len(frames) == 1 and not self.parser.buffer, "Duplicate/coalesced or partial extra reply")
                frame = frames[0]
                require(codec.matches(frame, mid, parameter), "Wrong reply type, ID, host or parameter")
                if parameter is None:
                    require((frame.can_id >> 16) == 0, "Noncanonical identity reply")
                result = codec.decode_reply(frame, mid, parameter)
                result.update(received_ns=received, raw_frame=frame.record())
                return result
        raise TimeoutError(f"No fresh reply: ID{mid} {parameter or 'identity'}")

    def run(self):
        if self._ran:
            raise RuntimeError("Backup probe permits one finite pass only")
        self._ran = True
        try:
            started = self._now()
            self.report["started_ns"] = started
            if self.deadline_ns is None:
                self.deadline_ns = started + MAX_RUN_NS
            require(type(self.deadline_ns) is int and started < self.deadline_ns <= started + MAX_RUN_NS,
                    "Invalid finite backup deadline")
            self._guard()
            self.reader = self.reader_factory(self.raw, clock=self.clock, check=self._guard)
            self._quiet()
            self.report["initial_quiet_observed"] = True
            for mid in self.ids:
                motor = {"parameters": {}}
                self.report["motors"][str(mid)] = motor
                motor["identity"] = self._query(mid, None)
                require(motor["identity"]["mcu_uid_hex"] == self.expected[mid], "Motor UID mismatch")
                for name in PARAMETERS:
                    result = self._query(mid, name)
                    motor["parameters"][name] = result
                    require(result["ok"], f"Rejected parameter: ID{mid} {name}")
                    if name == "voltage":
                        require(35 <= result["value"] <= 45, "Voltage outside backup35..45V range")
            self._quiet()
            self.report.update(final_quiet_observed=True, status="FIRMWARE_BACKUP_COMPLETE")
        except BaseException as exc:
            self.report["failure"] = repr(exc)
        finally:
            self.tx_prohibited = True
            self.report.update(tx_attempts=len(self.tx_log), raw_bytes=self.raw_bytes,
                read_calls=self.read_calls, residual_hex=bytes(self.parser.buffer).hex(),
                discarded_bytes=self.parser.discarded_bytes, last_host_clock_ns=self._last_ns,
                transport_poisoned=self.transport_poisoned, further_tx_prohibited=True)
        return self.report


def execute(bindings, expected, boot_id, *, cancelled, serial_factory=None, clock=time.monotonic_ns,
            boot_guard_factory=None, reader_factory=DeadlineSerialReader):
    """Own sequential bus sessions; injected dependencies are for offline tests."""
    expected = validate_uids(expected)
    results, probes, errors = {}, {}, []
    guard = lease = None
    deadline = clock() + MAX_RUN_NS
    try:
        guard = (boot_guard_factory or BootIdentityGuard)()
        require(guard.boot_id == boot_id, "Boot mismatch")
        lease = dual.CommonLease(ownership_locks)
        for scope, ids in IDS_BY_SCOPE.items():
            raw, stack = None, ExitStack()
            result = {"status": "INCOMPLETE", "port_closed": False}
            results[scope] = result
            lease.retain()
            try:
                def check():
                    require(not cancelled.is_set(), "Backup cancelled")
                    require(clock() < deadline, "Backup overall20s deadline exhausted")
                    guard.check()
                    require(dual.binding_matches(bindings[scope]), "Port binding changed")
                stack.enter_context(dual.port_lock(bindings[scope]["resolved"]))
                check()
                factory = serial_factory
                if factory is None:
                    import serial
                    factory = serial.Serial
                raw = factory(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
                              timeout=0, write_timeout=.1, exclusive=True,
                              xonxoff=False, rtscts=False, dsrdtr=False)
                raw.dtr = raw.rts = False
                raw.port = bindings[scope]["path"]
                raw.open()
                require(os.fstat(raw.fileno()).st_rdev == bindings[scope]["st_rdev"], "Opened FD differs")
                probe = BackupProbe(raw, ids, expected, clock=clock, check=check,
                                    deadline_ns=deadline, reader_factory=reader_factory)
                probes[scope] = probe
                result.update(probe.run())
                require(result["status"] == "FIRMWARE_BACKUP_COMPLETE", "Bus backup incomplete")
                check()
            except BaseException as exc:
                result.update(status="INCOMPLETE", failure=result.get("failure", repr(exc)))
                cancelled.set()
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
                        stack.close()
                        lease.release()
                    except BaseException as exc:
                        result.update(status="INCOMPLETE", lock_release_failure=repr(exc))
                else:
                    result["status"] = "INCOMPLETE"
                    _HELD_LOCKS.extend((stack, lease))
            if result["status"] != "FIRMWARE_BACKUP_COMPLETE":
                break
    except BaseException as exc:
        errors.append(repr(exc))
    finally:
        try:
            if lease is not None:
                lease.release()
        except BaseException as exc:
            errors.append("Common lock release failed: " + repr(exc))
        finally:
            if guard is not None:
                try:
                    guard.close()
                except BaseException as exc:
                    errors.append("Boot monitor close failed: " + repr(exc))
    complete = (not errors and not cancelled.is_set() and set(results) == set(IDS_BY_SCOPE)
                and lease is not None and lease.released and all(r["port_closed"]
                    and r["status"] == "FIRMWARE_BACKUP_COMPLETE" for r in results.values()))
    return ({**make_plan(), "status": "FIRMWARE_BACKUP_COMPLETE" if complete else "INCOMPLETE",
             "results": results, "errors": errors, "boot_id": boot_id, "bindings": bindings,
             "locks_released": bool(lease and lease.released)}, probes)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "Duplicate JSON identity key")
        result[key] = value
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-read-only", action="store_true")
    for name in ("front-port", "rear-port", "expected-uids", "expected-boot-id", "output"):
        parser.add_argument("--" + name)
    args = parser.parse_args(argv)
    if not args.execute_read_only:
        print(json.dumps(make_plan(), indent=2))
        return 0
    require(all((args.front_port, args.rear_port, args.expected_uids, args.expected_boot_id, args.output)),
            "Execution requires pinned ports, all UID identities, boot and private output")
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    uid_bytes = Path(args.expected_uids).expanduser().read_bytes()
    expected = validate_uids(json.loads(uid_bytes, object_pairs_hook=_unique_object))
    original_output = Path(args.output).expanduser()
    require(not original_output.exists() and not original_output.is_symlink(), "Output directory must be new")
    output = original_output.resolve()
    require(not any((p / ".git").exists() for p in (output, *output.parents)), "Private output outside Git required")
    output.mkdir(mode=0o700, exist_ok=False)
    cancelled, handlers = threading.Event(), {}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.set())
        report, probes = execute(bindings, expected, args.expected_boot_id, cancelled=cancelled)
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    report["expected_uid_file_sha256"] = hashlib.sha256(uid_bytes).hexdigest()
    report["source_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(Path(__file__).parent.glob("*.py"))}
    for scope, probe in probes.items():
        for category, rows in (("tx", probe.tx_log), ("raw", [
                {"read_started_ns": start, "received_ns": end, "hex": chunk.hex()}
                for start, end, chunk in probe.raw_log])):
            with (output / (scope + "-" + category + ".json")).open("x") as stream:
                json.dump(rows, stream, indent=2, allow_nan=False)
                stream.write("\n")
    with (output / "summary.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": report["status"], "output": str(output), **FLAGS}))
    return 0 if report["status"] == "FIRMWARE_BACKUP_COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
