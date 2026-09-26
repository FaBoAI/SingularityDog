"""One finite, injectable STOP-only batch observation on an already owned bus.

The caller establishes identity, voltage, stopped state, and a reporting-OFF
quiet boundary before creating this probe, and owns/closes the port and locks.
This module has no CLI, device discovery, serial import, retry, or motion path.
Group writes contain only prevalidated ordinary Type 4 all-zero STOP frames.
Host write/read intervals do not establish individual CAN transmit times or
physical stop completion. Feedback values are retained as raw uint16 fields.
"""
from dataclasses import dataclass
import struct
import time

from .can_readonly import ATParser, HOST_ID
from .rs05_trial_protocol import TrialPhase, stop_request
from .serial_deadline_reader import DeadlineSerialReader, record_failed_read


GROUP_TIMEOUT_NS = 250_000_000
MAX_RUN_NS = 3_000_000_000
FINAL_QUIET_NS = 100_000_000
MAX_RAW_BYTES = 65_536
MAX_RAW_CHUNKS = 1024
MAX_READ_CALLS = 4096
FLAGS = {
    "output_allowed": False,
    "motor_enabling_available": False,
    "motion_command_available": False,
    "automatic_retry": False,
    "pure_read_only": False,
    "physical_stop_time_verified": False,
    "can_wire_completion_verified": False,
    "per_frame_physical_send_timestamps_available": False,
    "motor_internal_sample_time_verified": False,
    "full_controller_50Hz_verified": False,
    "full_pipeline_20ms_verified": False,
    "velocity_scale_verified": False,
    "calibration_verified": False,
}


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _timestamp(value, name):
    _require(type(value) is int and 0 <= value < 2**63,
             f"{name} must be integer nanoseconds in 0..2**63-1")
    return value


def _canonical_stop(mid):
    can_id = (4 << 24) | (HOST_ID << 8) | mid
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + bytes(8) + b"\r\n"


@dataclass(frozen=True)
class _Group:
    ids: tuple[int, ...]
    wire: bytes


class StopBatchProbe:
    """An exactly-once pass; errors are returned in ``report`` without retry.

    ``ids`` is a sorted unique tuple on either the front (1..6) or rear (7..12)
    bus. Group size is 1, 2, 3, or 6; a final smaller group retains its explicit
    actual IDs/count. ``run`` is single-use even after a failed pass. No method
    accepts caller-provided transmit bytes, and port ownership remains external.
    ``raw_log`` contains (read_started_ns, received_ns, bytes) tuples;
    ``tx_log`` contains the host write interval and feedback for each group.
    """

    def __init__(self, raw, ids, *, group_size=1, clock=time.monotonic_ns,
                 check=lambda: None, reader_factory=DeadlineSerialReader):
        _require(type(ids) is tuple and bool(ids)
                 and all(type(mid) is int and 1 <= mid <= 12 for mid in ids)
                 and ids == tuple(sorted(set(ids))), "Select sorted unique integer IDs in 1..12")
        _require(all(mid <= 6 for mid in ids) or all(mid >= 7 for mid in ids),
                 "One STOP batch probe may observe only one configured bus")
        _require(type(group_size) is int and group_size in (1, 2, 3, 6),
                 "Select group_size 1, 2, 3, or 6")
        frames = []
        for mid in ids:
            wire = stop_request(phase=TrialPhase.STOP, motor_id=mid)
            _require(type(wire) is bytes and wire == _canonical_stop(mid),
                     "STOP codec differs from canonical all-zero Type 4")
            parser = ATParser()
            parsed = parser.feed(wire)
            _require(len(parsed) == 1 and not parser.buffer and not parser.discarded_bytes
                     and parsed[0].kind == 4 and parsed[0].source == HOST_ID
                     and parsed[0].destination == mid and parsed[0].flags == 4
                     and parsed[0].data == bytes(8), "STOP frame prevalidation failed")
            frames.append(wire)
        self.ids, self.group_size = ids, group_size
        self._groups = tuple(_Group(ids[start:start + group_size],
                                   b"".join(frames[start:start + group_size]))
                             for start in range(0, len(ids), group_size))
        self.raw, self.clock, self.check = raw, clock, check
        self._reader_factory = reader_factory
        self.reader = None
        self.parser = ATParser()
        self.raw_log, self.tx_log = [], []
        self.raw_bytes = self.read_calls = 0
        self.transport_poisoned = False
        self.tx_prohibited = False
        self._ran = False
        self._phase = "new"
        self._next_group = 0
        self._last_clock_ns = None
        self._partial_started_ns = None
        self.started_ns = self.deadline_ns = None
        self.report = {
            **FLAGS, "status": "INCOMPLETE", "ids": list(ids), "group_size": group_size,
            "groups_planned": [list(group.ids) for group in self._groups],
            "group_timeout_ms": GROUP_TIMEOUT_NS / 1e6, "max_run_ms": MAX_RUN_NS / 1e6,
            "required_final_quiet_ms": FINAL_QUIET_NS / 1e6,
            "identity_verified_by_probe": False, "preflight_is_caller_precondition": True,
            "groups": self.tx_log, "final_quiet_observed": False,
        }

    def _now(self):
        now = _timestamp(self.clock(), "clock")
        _require(self._last_clock_ns is None or now >= self._last_clock_ns,
                 "Monotonic clock moved backward")
        self._last_clock_ns = now
        return now

    def _guard(self):
        self.check()
        now = self._now()
        if now >= self.deadline_ns:
            raise TimeoutError("STOP batch overall deadline exhausted")
        return now

    def _read(self, wake_ns):
        started = self._guard()
        self.read_calls += 1
        _require(self.read_calls <= MAX_READ_CALLS, "Bounded receive-call budget exhausted")
        try:
            chunk, received = self.reader.read_until(min(wake_ns, self.deadline_ns - 1), self.deadline_ns)
        except BaseException as error:
            self.raw_bytes = record_failed_read(error, report=self.report, raw_log=self.raw_log,
                raw_bytes=self.raw_bytes, read_started_ns=started,
                max_raw_bytes=MAX_RAW_BYTES, max_raw_chunks=MAX_RAW_CHUNKS)
            raise
        _require(type(chunk) is bytes, "Reader must return bytes")
        _timestamp(received, "received_ns")
        now = self._now()
        _require(started <= received <= now, "Invalid host read interval")
        if chunk:
            self.raw_bytes += len(chunk)
            if self.raw_bytes > MAX_RAW_BYTES or len(self.raw_log) >= MAX_RAW_CHUNKS:
                self.report["raw_log_overflow"] = True
                self.report["unlogged_chunk_bytes"] = len(chunk)
                raise RuntimeError("Bounded raw evidence buffer exhausted")
            self.raw_log.append((started, received, chunk))
        self._guard()
        return chunk, started, received

    def _empty_boundary(self):
        _require(not self.parser.buffer, "Partial frame at group boundary")
        chunk, _, _ = self._read(self._guard())
        _require(not chunk, "Unexpected bytes at group boundary")

    def _write_next_group(self):
        _require(self._phase == "ready" and not self.tx_prohibited and not self.transport_poisoned,
                 "Further STOP transmission is prohibited")
        _require(self._next_group < len(self._groups), "All STOP groups were already attempted")
        group = self._groups[self._next_group]
        expected_ids = self.ids[self._next_group * self.group_size:
                                (self._next_group + 1) * self.group_size]
        _require(group.ids == expected_ids and type(group.wire) is bytes
                 and group.wire == b"".join(_canonical_stop(mid) for mid in expected_ids),
                 "Batch must contain only its exact prevalidated STOP frames")
        _require(not self.parser.buffer, "Partial frame before STOP write")
        now = self._guard()
        self.raw.write_timeout = min(.1, (self.deadline_ns - now) / 1e9)
        self._guard()
        started = self._now()
        row = {"group_index": self._next_group + 1, "ids": list(group.ids),
               "frame_count": len(group.ids), "wire_hex": group.wire.hex(),
               "write_started_ns": started, "write_finished_ns": None,
               "deadline_ns": min(started + GROUP_TIMEOUT_NS, self.deadline_ns),
               "returned_bytes": None, "status": "ATTEMPTED", "feedback_by_id": {}}
        self.tx_log.append(row)
        self._phase = "receiving"  # A failed/partial write can never repeat this group.
        try:
            row["returned_bytes"] = self.raw.write(group.wire)
            if type(row["returned_bytes"]) is not int or row["returned_bytes"] != len(group.wire):
                raise IOError("Partial or invalid UART write result; no further writes permitted")
        except BaseException:
            self.transport_poisoned = True
            raise
        finally:
            row["write_finished_ns"] = self._now()
        self._guard()
        if row["write_finished_ns"] >= row["deadline_ns"]:
            raise TimeoutError("STOP group write completed after its deadline")
        return group, row

    def _feedback(self, chunk, read_started_ns, received_ns, group, row):
        if received_ns >= row["deadline_ns"]:
            raise TimeoutError("STOP feedback arrived at or after the group deadline")
        offset = 0
        while offset < len(chunk):
            if not self.parser.buffer:
                self._partial_started_ns = read_started_ns
            take = min(17 - len(self.parser.buffer), len(chunk) - offset)
            frames = self.parser.feed(chunk[offset:offset + take])
            offset += take
            _require(not self.parser.discarded_bytes, "Malformed/discarded STOP feedback bytes")
            for frame in frames:
                _require(frame.flags == 4 and len(frame.data) == 8 and frame.kind == 2
                         and frame.destination == HOST_ID, "Noncanonical STOP feedback")
                _require(frame.source in group.ids, "Unexpected ID for current STOP group")
                key = str(frame.source)
                _require(key not in row["feedback_by_id"], "Duplicate STOP feedback ID")
                _require(not frame.data.startswith(b"\x00\xc4\x56"),
                         "Version-shaped Type 2 is not STOP feedback")
                mode_state, fault_bits = (frame.can_id >> 22) & 3, (frame.can_id >> 16) & 63
                _require(mode_state == 0 and fault_bits == 0, "STOP feedback is not mode0/fault0")
                row["feedback_by_id"][key] = {
                    "motor_id": frame.source, "kind": 2,
                    "raw_u16": list(struct.unpack(">4H", frame.data)),
                    "mode_state": mode_state, "fault_bits": fault_bits,
                    "first_seen_ns": self._partial_started_ns,
                    "read_started_ns": read_started_ns, "received_ns": received_ns,
                    "wire_hex": frame.wire.hex(), "physical_stop_time_verified": False,
                }
            if not self.parser.buffer:
                self._partial_started_ns = None

    def _receive_group(self, group, row):
        while len(row["feedback_by_id"]) < len(group.ids):
            if self._guard() >= row["deadline_ns"]:
                raise TimeoutError("Missing STOP feedback before group deadline")
            chunk, started, received = self._read(row["deadline_ns"])
            if chunk:
                self._feedback(chunk, started, received, group, row)
        self._empty_boundary()
        row["last_feedback_received_ns"] = max(
            feedback["received_ns"] for feedback in row["feedback_by_id"].values())
        row["host_group_round_trip_ms"] = (row["last_feedback_received_ns"] - row["write_started_ns"]) / 1e6
        row["status"] = "OBSERVED"

    def _final_quiet(self):
        _require(not self.parser.buffer, "Partial frame before final quiet")
        started = self._guard()
        end = started + FINAL_QUIET_NS
        self.report["quiet_started_ns"] = started
        while self._guard() < end:
            chunk, _, _ = self._read(end)
            _require(not chunk, "Unexpected feedback during final quiet")
        _require(not self.parser.buffer, "Partial frame after final quiet")
        self.report.update(quiet_finished_ns=self._guard(), final_quiet_observed=True)

    def run(self):
        if self._ran:
            raise RuntimeError("STOP batch probe permits only one finite pass")
        self._ran = True
        try:
            self.started_ns = self._now()
            self.deadline_ns = self.started_ns + MAX_RUN_NS
            _timestamp(self.deadline_ns, "overall_deadline_ns")
            self._guard()
            self.reader = self._reader_factory(self.raw, clock=self.clock, check=self._guard)
            self._empty_boundary()
            for _ in self._groups:
                self._phase = "ready"
                group, row = self._write_next_group()
                self._receive_group(group, row)
                self._next_group += 1
            self._phase = "quiet"
            self._final_quiet()
            self.report["status"] = "STOP_BATCH_OBSERVATION_COMPLETE"
            self._phase = "finished"
        except BaseException as exc:
            self._phase = "failed"
            self.report["failure"] = repr(exc)
            if self.tx_log and self.tx_log[-1]["status"] != "OBSERVED":
                self.tx_log[-1]["failure"] = repr(exc)
        finally:
            self.tx_prohibited = True
            self.report.update(started_ns=self.started_ns, overall_deadline_ns=self.deadline_ns,
                last_host_clock_ns=self._last_clock_ns, transport_poisoned=self.transport_poisoned,
                further_tx_prohibited=True, write_attempts=len(self.tx_log),
                groups_observed=sum(row["status"] == "OBSERVED" for row in self.tx_log),
                raw_bytes=self.raw_bytes, read_calls=self.read_calls,
                residual_hex=bytes(self.parser.buffer).hex(), discarded_bytes=self.parser.discarded_bytes)
        return self.report
