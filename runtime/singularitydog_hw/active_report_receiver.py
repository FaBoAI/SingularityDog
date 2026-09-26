"""Offline, bounded receiver for a stopped RS05 Active Reporting observation.

Timestamps describe host reads, not motor sampling. Neither host sequences nor
arrival gaps establish physical packet loss or suitability for 20 ms control.
This module performs no I/O and deliberately retains only raw uint16 values.
"""
from dataclasses import dataclass
import math
import struct

from .can_readonly import ATParser, HOST_ID


@dataclass(frozen=True)
class Sample:
    motor_id: int
    kind: int
    raw_u16: tuple[int, int, int, int]
    mode_state: int
    fault_bits: int
    wire: bytes
    host_sequence: int
    first_seen_ns: int
    received_ns: int


class ReceiverError(ValueError):
    """A stream error permanently invalidates this observation session."""


def _timestamp(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer nanosecond timestamp")


def _percentile(values, fraction):
    """Linearly interpolate the sorted host interarrival observations."""
    if not values:
        return None
    ordered = sorted(values)
    at = (len(ordered) - 1) * fraction
    lower = math.floor(at)
    upper = math.ceil(at)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (at - lower)


class StreamReceiver:
    """One serial reader's observations, with a fail-closed protocol boundary.

    Every successful read (including empty reads) may be supplied to ``feed``.
    Read intervals must not overlap or move backward. Equal timestamps are
    allowed, including multiple complete frames returned by one read. A partial
    frame retains the start time of the first read that contributed its bytes.

    ``expected_kind=None`` discovers Type 2 or Type 24 independently per ID;
    a change of kind for an already observed ID invalidates the session. No
    deduplication is possible because these frames lack a device sequence.

    ``activation_type2_prefix=True`` explicitly selects one observed firmware
    behavior for an otherwise Type 24 stream. Each ID must first be armed using
    ``expect_activation`` before the caller's reporting-on write. Exactly one
    stopped Type 2 in that host interval is retained in ``activation_feedback``;
    it is neither a periodic sample nor proof of STOP/configuration completion.
    Activation records have host_sequence=0 and are bounded to one per ID.
    ``expect_deactivation`` similarly selects one stopped Type 2 during an
    explicitly bounded reporting-off transition, while retaining Type 24 tails.

    A ReceiverError leaves accepted prefix samples available in ``samples`` and
    permanently poisons the receiver. The caller should retain each entire raw
    read before feeding it: an invalid read's unprocessed suffix is counted,
    not buffered. The parser buffer and rejected frame evidence stay bounded.
    """

    def __init__(self, ids: tuple[int, ...], expected_kind: int | None,
                 max_frames=100000, *, activation_type2_prefix=False):
        if (type(ids) is not tuple or not ids or
                any(type(mid) is not int or not 1 <= mid <= 12 for mid in ids) or
                tuple(sorted(set(ids))) != ids):
            raise ValueError("ids must be a nonempty sorted unique tuple from IDs 1..12")
        if expected_kind is not None and (type(expected_kind) is not int or
                                          expected_kind not in (2, 24)):
            raise ValueError("expected_kind must be explicitly 2, 24, or None for discovery")
        if type(max_frames) is not int or max_frames < 1:
            raise ValueError("max_frames must be a positive integer")
        if type(activation_type2_prefix) is not bool:
            raise ValueError("activation_type2_prefix must be bool")
        if activation_type2_prefix and expected_kind != 24:
            raise ValueError("activation_type2_prefix requires expected_kind=24")
        self.ids = ids
        self.expected_kind = expected_kind
        self.max_frames = max_frames
        self.activation_type2_prefix = activation_type2_prefix
        self.activation_pending: dict[int, tuple[int, int]] = {}
        self.activation_feedback: dict[int, Sample] = {}
        self.deactivation_pending: dict[int, tuple[int, int]] = {}
        self.deactivation_feedback: dict[int, Sample] = {}
        self.parser = ATParser()
        self.samples: list[Sample] = []
        self.detected_kinds: dict[int, int] = {}
        self.errors: list[str] = []
        self.poisoned = False
        self.received_bytes = 0
        self.unprocessed_bytes = 0
        self.rejected_wire_hex: str | None = None
        self._sequences = dict.fromkeys(ids, 0)
        self._partial_started_ns = None
        self._first_read_started_ns = None
        self._last_received_ns = None

    def _fail(self, reason, wire=None):
        self.poisoned = True
        self.errors.append(reason)
        if wire is not None:
            self.rejected_wire_hex = wire.hex()
        raise ReceiverError(reason)

    def expect_activation(self, mid, not_before_ns, deadline_ns):
        """Arm one selected ID before its reporting-on write; never re-arm it.

        Bounds apply to the first contributing host read's start and the read
        completing the frame. They do not establish a device timestamp or an
        acknowledged command transaction. An unfinished expectation invalidates
        summary and expires on a feed whose completion is beyond its deadline.
        """
        self._expect_transition("activation", mid, not_before_ns, deadline_ns)

    def expect_deactivation(self, mid, not_before_ns, deadline_ns):
        """Arm one reporting-off transition after that ID's activation prefix.

        Type 24 tails remain periodic samples before and after its single Type 2
        record. This record is not a STOP acknowledgment or proof that reporting
        ceased; the caller must independently establish its quiet interval.
        """
        self._expect_transition("deactivation", mid, not_before_ns, deadline_ns)

    def _expect_transition(self, phase, mid, not_before_ns, deadline_ns):
        if self.poisoned:
            raise ReceiverError("receiver_already_poisoned")
        if not self.activation_type2_prefix:
            self._fail("activation_type2_prefix_not_selected")
        if type(mid) is not int or mid not in self.ids:
            self._fail(f"{phase}_motor_id_not_selected")
        if phase == "deactivation" and mid not in self.activation_feedback:
            self._fail("deactivation_requires_activation_prefix")
        pending = self.activation_pending if phase == "activation" else self.deactivation_pending
        recorded = self.activation_feedback if phase == "activation" else self.deactivation_feedback
        if mid in pending or mid in recorded:
            self._fail(f"{phase}_already_armed")
        try:
            _timestamp(not_before_ns, "not_before_ns")
            _timestamp(deadline_ns, "deadline_ns")
            if not_before_ns >= deadline_ns:
                raise ValueError(f"{phase}_interval_empty_or_reversed")
            if self._last_received_ns is not None and not_before_ns < self._last_received_ns:
                raise ValueError(f"{phase}_interval_precedes_last_read")
        except ValueError as exc:
            self._fail(str(exc))
        pending[mid] = (not_before_ns, deadline_ns)

    def _transition_sample(self, phase, frame, first_seen_ns, received_ns):
        pending = self.activation_pending if phase == "activation" else self.deactivation_pending
        recorded = self.activation_feedback if phase == "activation" else self.deactivation_feedback
        not_before_ns, deadline_ns = pending[frame.source]
        if first_seen_ns < not_before_ns or received_ns > deadline_ns:
            self._fail(f"{phase}_type2_prefix_outside_interval", frame.wire)
        recorded[frame.source] = Sample(
            frame.source, frame.kind, struct.unpack(">4H", frame.data),
            0, 0, frame.wire, 0, first_seen_ns, received_ns)
        del pending[frame.source]

    def _sample(self, frame, first_seen_ns, received_ns):
        if frame.flags != 4 or len(frame.data) != 8:
            self._fail("noncanonical_flags_or_dlc", frame.wire)
        if frame.source not in self.ids:
            self._fail("unknown_motor_id", frame.wire)
        if frame.destination != HOST_ID:
            self._fail("unexpected_destination", frame.wire)
        if frame.kind not in (2, 24):
            self._fail("unexpected_frame_type", frame.wire)
        if frame.kind == 2 and frame.data.startswith(b"\x00\xc4\x56"):
            self._fail("version_reply_is_not_active_feedback", frame.wire)
        mode_state = (frame.can_id >> 22) & 3
        fault_bits = (frame.can_id >> 16) & 63
        if mode_state != 0 or fault_bits != 0:
            self._fail("nonzero_mode_or_fault", frame.wire)
        if self.activation_type2_prefix:
            if frame.source in self.activation_pending:
                if frame.kind != 2:
                    self._fail("activation_type2_prefix_required", frame.wire)
                self._transition_sample("activation", frame, first_seen_ns, received_ns)
                return None
            if frame.source not in self.activation_feedback:
                self._fail("activation_not_armed", frame.wire)
            if frame.kind == 2:
                if frame.source in self.deactivation_pending:
                    self._transition_sample("deactivation", frame, first_seen_ns, received_ns)
                    return None
                if frame.source in self.deactivation_feedback:
                    self._fail("deactivation_type2_prefix_duplicate", frame.wire)
                self._fail("activation_type2_prefix_duplicate", frame.wire)
        if self.expected_kind is not None and frame.kind != self.expected_kind:
            self._fail("expected_frame_type_mismatch", frame.wire)
        if (frame.source in self.detected_kinds and
                self.detected_kinds[frame.source] != frame.kind):
            self._fail("per_id_frame_type_changed", frame.wire)
        if len(self.samples) >= self.max_frames:
            self._fail("frame_capacity_exceeded", frame.wire)
        self.detected_kinds[frame.source] = frame.kind
        self._sequences[frame.source] += 1
        sample = Sample(frame.source, frame.kind, struct.unpack(">4H", frame.data),
                        mode_state, fault_bits, frame.wire,
                        self._sequences[frame.source], first_seen_ns, received_ns)
        self.samples.append(sample)
        return sample

    def feed(self, chunk: bytes, read_started_ns: int, received_ns: int) -> list[Sample]:
        if self.poisoned:
            raise ReceiverError("receiver_already_poisoned")
        try:
            if type(chunk) is not bytes:
                raise ValueError("chunk must be bytes")
            _timestamp(read_started_ns, "read_started_ns")
            _timestamp(received_ns, "received_ns")
            if read_started_ns > received_ns:
                raise ValueError("read_interval_reversed")
            if self._last_received_ns is not None and read_started_ns < self._last_received_ns:
                raise ValueError("read_clock_moved_backward_or_intervals_overlap")
        except ValueError as exc:
            self._fail(str(exc))
        if self._first_read_started_ns is None:
            self._first_read_started_ns = read_started_ns
        self._last_received_ns = received_ns
        self.received_bytes += len(chunk)
        accepted = []
        offset = 0
        try:
            while offset < len(chunk):
                if not self.parser.buffer:
                    self._partial_started_ns = read_started_ns
                # At most one canonical DLC8 frame is parsed in each step. This
                # avoids building an unbounded temporary list from a large read.
                take = min(17 - len(self.parser.buffer), len(chunk) - offset)
                piece = chunk[offset:offset + take]
                offset += take
                frames = self.parser.feed(piece)
                if self.parser.discarded_bytes:
                    self._fail("malformed_or_discarded_bytes")
                for frame in frames:
                    sample = self._sample(frame, self._partial_started_ns, received_ns)
                    if sample is not None:
                        accepted.append(sample)
                if not self.parser.buffer:
                    self._partial_started_ns = None
        except ReceiverError:
            self.unprocessed_bytes += len(chunk) - offset
            raise
        for phase, pending in (("activation", self.activation_pending),
                               ("deactivation", self.deactivation_pending)):
            for mid, (_, deadline_ns) in pending.items():
                if received_ns > deadline_ns:
                    self._fail(f"{phase}_prefix_deadline_exceeded:{mid}")
        return accepted

    def summary(self, start_ns, end_ns, max_gap_ms=20):
        """Summarize one host observation interval, including its edges.

        ``ok`` requires every selected channel to be present, no protocol or
        residual-frame errors, and all host silence intervals <= max_gap_ms.
        Only samples wholly inside the window are included; warmup samples and
        frames crossing its start are retained in ``samples`` but excluded here.
        The caller must provide a completed observation window; these bounds
        are not inferred from the first and last frames. Empty read polls are
        optional. Actual supplied read bounds are included as separate evidence.
        A residual frame first seen after the window is reported without making
        that earlier observation fail. Other unresolved residuals invalidate it.
        Frames completed by a later drain are counted at the crossed boundary.
        Missing channels have the full window as silence and still fail even
        when that window is shorter than the limit. Percentiles are linearly
        interpolated; they are None when there are fewer than two samples.
        """
        _timestamp(start_ns, "start_ns")
        _timestamp(end_ns, "end_ns")
        if start_ns >= end_ns:
            raise ValueError("summary requires start_ns < end_ns")
        if type(max_gap_ms) not in (int, float):
            raise ValueError("max_gap_ms must be a finite positive number")
        try:
            max_gap_ms = float(max_gap_ms)
        except OverflowError as exc:
            raise ValueError("max_gap_ms must be a finite positive number") from exc
        if not math.isfinite(max_gap_ms) or max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be a finite positive number")
        errors = list(self.errors)
        for mid in self.activation_pending:
            errors.append(f"activation_prefix_pending:{mid}")
        for mid in self.deactivation_pending:
            errors.append(f"deactivation_prefix_pending:{mid}")
        if self.activation_type2_prefix:
            for mid in self.ids:
                if mid not in self.activation_pending and mid not in self.activation_feedback:
                    errors.append(f"activation_not_armed:{mid}")
        residual_in_window = bool(self.parser.buffer) and self._partial_started_ns <= end_ns
        if residual_in_window:
            errors.append("partial_frame_at_end")
        by_id = {mid: [] for mid in self.ids}
        excluded_before = excluded_after = excluded_crossing_start = excluded_crossing_end = 0
        for sample in self.samples:
            excluded_crossing_end += sample.first_seen_ns <= end_ns < sample.received_ns
            if sample.first_seen_ns < start_ns:
                excluded_before += 1
                excluded_crossing_start += sample.received_ns >= start_ns
                continue
            if sample.received_ns > end_ns:
                excluded_after += 1
                continue
            by_id[sample.motor_id].append(sample.received_ns)
        channels = {}
        for mid, arrivals in by_id.items():
            gaps = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
            duration_ms = (end_ns - start_ns) / 1e6
            initial = (arrivals[0] - start_ns) / 1e6 if arrivals else duration_ms
            tail = (end_ns - arrivals[-1]) / 1e6 if arrivals else duration_ms
            max_silence = max(initial, tail, max(gaps, default=0.0))
            present = bool(arrivals)
            within_limit = max_silence <= max_gap_ms
            if not present:
                errors.append(f"missing_motor_id:{mid}")
            if not within_limit:
                errors.append(f"host_silence_limit_exceeded:{mid}")
            channels[str(mid)] = {
                "frame_count": len(arrivals),
                "interarrival_ms": {"p50": _percentile(gaps, .50),
                                    "p95": _percentile(gaps, .95),
                                    "p99": _percentile(gaps, .99),
                                    "max": max(gaps) if gaps else None},
                "initial_gap_ms": initial, "tail_gap_ms": tail,
                "max_silence_ms": max_silence,
                "present": present, "within_gap_limit": within_limit,
                "ok": present and within_limit,
            }
        return {
            "ok": not errors, "errors": errors, "ids": channels,
            "start_ns": start_ns, "end_ns": end_ns, "max_gap_ms": max_gap_ms,
            "first_read_started_ns": self._first_read_started_ns,
            "last_read_received_ns": self._last_received_ns,
            "samples_received": sum(len(arrivals) for arrivals in by_id.values()),
            "total_samples_received": len(self.samples), "max_frames": self.max_frames,
            "excluded_before_window": excluded_before,
            "excluded_after_window": excluded_after,
            "excluded_crossing_start": excluded_crossing_start,
            "excluded_crossing_end": excluded_crossing_end,
            "expected_kind": self.expected_kind,
            "activation_type2_prefix": self.activation_type2_prefix,
            "activation_pending": {
                str(mid): {"not_before_ns": bounds[0], "deadline_ns": bounds[1]}
                for mid, bounds in self.activation_pending.items()},
            "activation_feedback": {
                str(mid): {"motor_id": sample.motor_id, "kind": sample.kind,
                           "raw_u16": list(sample.raw_u16), "mode_state": sample.mode_state,
                           "fault_bits": sample.fault_bits, "wire_hex": sample.wire.hex(),
                           "host_sequence": sample.host_sequence,
                           "first_seen_ns": sample.first_seen_ns,
                           "received_ns": sample.received_ns}
                for mid, sample in self.activation_feedback.items()},
            "activation_stop_ack_proven": False,
            "activation_configuration_ack_proven": False,
            "deactivation_pending": {
                str(mid): {"not_before_ns": bounds[0], "deadline_ns": bounds[1]}
                for mid, bounds in self.deactivation_pending.items()},
            "deactivation_feedback": {
                str(mid): {"motor_id": sample.motor_id, "kind": sample.kind,
                           "raw_u16": list(sample.raw_u16), "mode_state": sample.mode_state,
                           "fault_bits": sample.fault_bits, "wire_hex": sample.wire.hex(),
                           "host_sequence": sample.host_sequence,
                           "first_seen_ns": sample.first_seen_ns,
                           "received_ns": sample.received_ns}
                for mid, sample in self.deactivation_feedback.items()},
            "deactivation_stop_ack_proven": False,
            "deactivation_reporting_off_proven": False,
            "detected_kinds": {str(mid): kind for mid, kind in self.detected_kinds.items()},
            "received_bytes": self.received_bytes,
            "discarded_bytes": self.parser.discarded_bytes,
            "unprocessed_bytes": self.unprocessed_bytes,
            "residual_hex": bytes(self.parser.buffer).hex(),
            "residual_bytes": len(self.parser.buffer),
            "residual_in_window": residual_in_window,
            "residual_first_seen_ns": self._partial_started_ns,
            "rejected_wire_hex": self.rejected_wire_hex,
            "host_observation_only": True,
            "physical_packet_loss_proven": False,
            "motor_sample_time_proven": False,
            "control_20ms_proven": False,
            "scaled_feedback_verified": False,
        }
