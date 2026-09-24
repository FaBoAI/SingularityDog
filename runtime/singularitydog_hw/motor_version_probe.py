"""Bounded RS05 stop-and-version probe, fixed IDs1..12; NOT pure read-only.

Each ID must match its expected UID, acknowledge an ordinary Type4 stop with
canonical Type2 mode0/fault0, then receives ONE Type4 version query. Only the
documented 00 c4 56 Type2 reply is a version. Raw bytes3..6 are retained without
guessing semantic firmware numbers; byte7 is unspecified and retained.

Primary source: RS05User Manual260713.pdf, printed English pages45--46
(PDF pages47--48), SHA256 recorded in MANUAL below. Boot continuity and physical
readiness belong to the caller. No enabling, settings, zero/save, Type26 fallback
or automatic retry exists. A timeout poisons the session before later IDs.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import time

from . import can_readonly as codec
from . import rs05_trial_protocol as protocol
from . import can_timing_probe as timing

IDS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
TIMEOUT_NS = 250_000_000
GAP_NS = 5_000_000
DRAIN_NS = 50_000_000
MAX_RUN_NS = 15_000_000_000
VERSION_PAYLOAD = b"\x00\xc4" + bytes(6)
VERSION_PREFIX = b"\x00\xc4\x56"
MANUAL = {"url": "https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf",
          "sha256": "1b2c61a6daa8331b9a348455005d10d6e425165c22e6f64c39cbca33f6360e82",
          "printed_english_pages": [45, 46], "pdf_pages_one_based": [47, 48]}


class VersionUnknown(TimeoutError):
    """The sole version query had no matching version before its deadline."""


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def version_request(motor_id):
    _require(type(motor_id) is int and motor_id in IDS, "ID outside fixed version-probe scope")
    can_id = (4 << 24) | (codec.HOST_ID << 8) | motor_id
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + VERSION_PAYLOAD + b"\r\n"


def decode_version(frame, motor_id):
    _require(motor_id in IDS and type(motor_id) is int, "ID outside fixed version-probe scope")
    protocol._reply(frame, 2, motor_id)  # Canonical29-bit AT, flags4, DLC8, source/destination.
    _require(frame.data[:3] == VERSION_PREFIX, "Normal Type2 feedback is not a version reply")
    _require(((frame.can_id >> 22) & 3) == 0 and ((frame.can_id >> 16) & 63) == 0,
             "Version reply reports running/reserved mode or fault")
    return {"version_bytes_hex": frame.data[3:7].hex(), "version_bytes": list(frame.data[3:7]),
            "byte_order": "as received; high-order first per manual",
            "semantic_firmware_version": None, "unspecified_byte7": frame.data[7],
            "mode_state": 0, "fault_bits": 0, "raw_frame": frame.record()}


def validate_uids(data):
    _require(isinstance(data, dict), "Expected UID JSON must be an object")
    keys = {str(k) for k in data}
    _require(all(type(k) in (str, int) for k in data) and len(keys) == len(data)
             and keys == {str(i) for i in IDS},
             "Expected UID keys must be all fixed IDs1..12")
    _require(all(isinstance(v, str) and len(v) == 16 and all(c in "0123456789abcdef" for c in v)
                 for v in data.values()) and len(set(data.values())) == len(data), "Invalid/duplicate UID")
    normalized = {int(k): value for k, value in data.items()}
    return {mid: normalized[mid] for mid in IDS}


def make_plan():
    return {"ids": list(IDS), "per_id_sequence": ["fresh_identity", "ordinary_stop_confirmed", "version_once"],
            "allowed_can_types": [0, 4], "max_write_attempts": 36, "max_version_queries": 12,
            "version_request_payload_hex": VERSION_PAYLOAD.hex(), "ordinary_stop_payload_hex": bytes(8).hex(),
            "reply_timeout_ms": 250, "minimum_after_write_ms": 5, "fresh_boundary_max_ms": 50,
            "max_run_s": 15, "pure_read_only": False, "stop_command_available": True,
            "motor_enabling_available": False, "motion_command_available": False,
            "settings_available": False, "automatic_retry": False, "type26_fallback": False,
            "approved_for_runtime": False, "boot_verified_by_tool": False,
            "manual": dict(MANUAL)}


class VersionProbe:
    """One finite session; validated bytes only at the physical write boundary."""
    def __init__(self, port, emit, *, check_interrupt=lambda: None, clock=time.monotonic_ns):
        self.port, self.emit, self.check_interrupt, self.clock = port, emit, check_interrupt, clock
        self.parser = codec.ATParser()
        self.started_ns = self.last_clock_ns = clock()
        self.last_write_finished_ns = None
        self.write_attempts = 0
        self.poisoned = self.collected = False
        self.schedule = [(mid, step) for mid in IDS for step in ("identity", "stop", "version")]

    def _check(self):
        self.check_interrupt()
        now = self.clock()
        _require(type(now) is int and self.last_clock_ns <= now, "Monotonic clock moved backward")
        self.last_clock_ns = now
        if self.poisoned or now-self.started_ns >= MAX_RUN_NS:
            raise RuntimeError("Probe poisoned or total time budget exhausted")
        return now

    def _receive(self, deadline, phase):
        now = self._check()
        self.port.timeout = max(0., min(.002, (deadline-now)/1e9,
                                       (self.started_ns+MAX_RUN_NS-now)/1e9))
        chunk = self.port.read(min(max(self.port.in_waiting, 1), 4096))
        received = self.clock()  # Raw read completion, before emit/parse work.
        if not chunk:
            self._check()
            return [], received
        self.emit({"kind": "version_rx_bytes", "monotonic_ns": received, "phase": phase, "hex": chunk.hex()})
        frames = self.parser.feed(chunk)
        _require(self.parser.discarded_bytes == 0, "Malformed/discarded serial bytes")
        for frame in frames:
            self.emit({"kind": "version_rx_frame", "monotonic_ns": received, "phase": phase, **frame.record()})
            _require(frame.flags == 4 and len(frame.data) == 8, "Noncanonical flags or DLC")
            if frame.kind == 2 and frame.destination == codec.HOST_ID and frame.source in IDS:
                if frame.data[:3] == VERSION_PREFIX:
                    decode_version(frame, frame.source)  # Also rejects fault/running version-shaped packets.
                else:
                    fb = protocol.decode_type2(frame, motor_id=frame.source)
                    _require(fb.mode_state == 0 and fb.fault_bits == 0, "Running mode or fault in Type2 feedback")
            elif frame.kind == 21 and frame.source in IDS:
                raise RuntimeError("Motor fault-detail frame received")
        self._check()
        return frames, received

    def _boundary(self):
        """Discard/log old frames until5ms quiet, within a50ms total drain bound."""
        start = self._check()
        deadline, quiet = start+DRAIN_NS, start+GAP_NS
        if self.last_write_finished_ns is not None:
            quiet = max(quiet, self.last_write_finished_ns+GAP_NS)
        while self._check() < quiet or self.port.in_waiting:
            if self.clock() >= deadline:
                raise RuntimeError("CAN backlog prevents a fresh request boundary")
            frames, received = self._receive(min(quiet, deadline), "pre_send_discard")
            if frames or self.parser.buffer:
                quiet = received+GAP_NS
        _require(not self.parser.buffer, "Partial old frame at request boundary")

    def _exchange(self, motor_id, step):
        self._check()
        _require(self.write_attempts < len(self.schedule)
                 and (motor_id, step) == self.schedule[self.write_attempts], "Invalid request order/count")
        self._boundary()
        wire = (codec.read_request(motor_id) if step == "identity" else
                protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=motor_id)
                if step == "stop" else version_request(motor_id))
        allowed = {codec.read_request(motor_id),
                   protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=motor_id),
                   version_request(motor_id)}
        _require(wire in allowed, "Outgoing wire not permitted")
        self.emit({"kind": "version_tx_intent", "monotonic_ns": self.clock(),
                   "motor_id": motor_id, "step": step, "hex": wire.hex()})
        started = self._check()
        deadline = started+TIMEOUT_NS
        self.port.write_timeout = .02
        returned = None
        self.write_attempts += 1
        try:
            returned = self.port.write(wire)
        finally:
            self.last_write_finished_ns = self.clock()
            self.emit({"kind": "version_tx_write", "motor_id": motor_id, "step": step, "hex": wire.hex(),
                       "write_started_monotonic_ns": started,
                       "write_finished_monotonic_ns": self.last_write_finished_ns,
                       "expected_bytes": len(wire), "returned_bytes": returned,
                       "deadline_monotonic_ns": deadline})
        _require(type(returned) is int and returned == len(wire), "Partial serial write; no retry")
        while self._check() < deadline:
            frames, received = self._receive(deadline, step)
            if received >= deadline:
                break
            match = None
            for frame in frames:
                if step == "identity" and codec.matches(frame, motor_id, None):
                    value = codec.decode_reply(frame, motor_id, None)
                elif frame.kind == 2 and frame.source == motor_id and frame.destination == codec.HOST_ID:
                    if step == "version" and frame.data[:3] == VERSION_PREFIX:
                        value = decode_version(frame, motor_id)
                    elif step == "stop" and frame.data[:3] != VERSION_PREFIX:
                        fb = protocol.decode_type2(frame, motor_id=motor_id)
                        value = {"mode_state": fb.mode_state, "fault_bits": fb.fault_bits,
                                 "raw_frame": frame.record()}
                    else:
                        continue
                else:
                    continue
                _require(match is None, "Duplicate matching reply in one serial read")
                match = value
            if match is not None:
                _require(not self.parser.buffer, "Partial residual frame after matching response")
                match.update(request_started_monotonic_ns=started, received_monotonic_ns=received)
                self.emit({"kind": "version_exchange_result", "motor_id": motor_id, "step": step, **match})
                return match
        self.emit({"kind": "version_deadline", "motor_id": motor_id, "step": step,
                   "monotonic_ns": self.clock(), "deadline_monotonic_ns": deadline})
        if step == "version":
            raise VersionUnknown("No matching version reply within250ms; no fallback or later ID")
        raise TimeoutError("No fresh " + step + " response within250ms")

    def collect(self, expected_uids):
        _require(not self.collected, "Probe session cannot be reused")
        expected_uids = validate_uids(expected_uids)
        self.collected = True
        report = {"status": "ABORTED", "errors": [], "motors": [
            {"motor_id": mid, "status": "NOT_ATTEMPTED", "identity_verified": False,
             "stop_confirmed": False, "version": None} for mid in IDS], "plan": make_plan()}
        current = None
        try:
            for current in report["motors"]:
                mid = current["motor_id"]
                current["status"] = "IN_PROGRESS"
                identity = self._exchange(mid, "identity")
                _require(identity["mcu_uid_hex"] == expected_uids[mid], "Fresh motor identity mismatch")
                current["identity_verified"] = True
                current["stop_feedback"] = self._exchange(mid, "stop")
                current["stop_confirmed"] = True
                current["version"] = self._exchange(mid, "version")
                current["status"] = "VERSION_RECEIVED"
            self._boundary()
            report["status"] = "VERSION_PROBE_COMPLETE"
        except BaseException as error:
            unknown = isinstance(error, VersionUnknown)
            report["status"] = "UNKNOWN" if unknown else "ABORTED"
            report["errors"].append(type(error).__name__ + ": " + str(error))
            if current is not None and current["status"] == "IN_PROGRESS":
                current["status"] = "UNKNOWN" if unknown else "ABORTED"
        finally:
            self.poisoned = True
        report.update(write_attempts=self.write_attempts,
                      elapsed_s=(self.clock()-self.started_ns)/1e9, approved_for_runtime=False,
                      motor_enabling_available=False, motion_command_available=False,
                      settings_available=False, pure_read_only=False, boot_verified_by_tool=False)
        return report


def open_serial():
    import serial
    port = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
                         timeout=.002, write_timeout=.02, xonxoff=False, rtscts=False,
                         dsrdtr=False, exclusive=True)
    port.dtr = port.rts = False
    port.port = "/dev/robstride-usb2can"
    try:
        port.open()
    except BaseException:
        port.close()
        raise
    return port


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, "Duplicate UID JSON key")
        result[key] = value
    return result


def _sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Send ordinary Type4 stop and shared Type4 version query")
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        uid_bytes = args.expected_uids.expanduser().read_bytes()
        expected = validate_uids(json.loads(uid_bytes, object_pairs_hook=_strict_object))
        if not args.execute:
            print(json.dumps(make_plan(), indent=2))
            return 0
        original = args.output.expanduser()
        _require(not original.exists() and not original.is_symlink(), "Output directory must be new")
        output = original.resolve()
        _require(not any((p/".git").exists() or (p/".git").is_symlink() for p in (output, *output.parents))
                 and Path(__file__).resolve().parent.parent not in output.parents, "Output must be outside Git/runtime")
        output.mkdir(mode=0o700, exist_ok=False)
        _sync_directory(output.parent)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    report = {"status": "ABORTED", "errors": [], "plan": make_plan(),
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "expected_uid_file_sha256": hashlib.sha256(uid_bytes).hexdigest(),
              "source_sha256": {Path(p).name: hashlib.sha256(Path(p).read_bytes()).hexdigest()
                  for p in (__file__, codec.__file__, protocol.__file__, timing.__file__)}}
    signals, handlers, port = [], {}, None
    def check_interrupt():
        if signals:
            raise InterruptedError("signal " + str(signals[0]))
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: signals.append(number))
        descriptor = os.open(output/"events.jsonl", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            def emit(event):
                stream.write(json.dumps({"wall_time_ns": time.time_ns(), **event}, allow_nan=False)+"\n")
                stream.flush()
                os.fsync(stream.fileno())
            emit({"kind": "version_probe_metadata", "monotonic_ns": time.monotonic_ns(), **make_plan()})
            with timing.ownership_locks():
                check_interrupt()
                port = open_serial()
                try:
                    report.update(VersionProbe(port, emit, check_interrupt=check_interrupt).collect(expected))
                finally:
                    port.close()
                    port = None
                check_interrupt()
    except BaseException as error:
        report["status"] = "ABORTED"
        report["errors"].append(type(error).__name__ + ": " + str(error))
    finally:
        if port is not None:
            try:
                port.close()
            except BaseException as error:
                report["errors"].append("Port close failed: " + repr(error))
                report["status"] = "ABORTED"
        try:
            report["signals"] = signals
            report["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
            events_path = output/"events.jsonl"
            report["events_sha256"] = hashlib.sha256(events_path.read_bytes()).hexdigest() if events_path.exists() else None
            serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n"
            descriptor = os.open(output/"summary.json", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(serialized)
                stream.flush()
                os.fsync(stream.fileno())
            _sync_directory(output)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    print(json.dumps({"output": str(output), "status": report["status"],
                      "write_attempts": report.get("write_attempts", 0),
                      "motor_enabling_available": False, "pure_read_only": False,
                      "approved_for_runtime": False}))
    return 0 if report["status"] == "VERSION_PROBE_COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
