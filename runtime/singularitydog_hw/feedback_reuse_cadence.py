"""Pure admission planning for a 26-request / 14-request feedback cadence.

An output Type 2 reply can be the next policy input only while its *original*
request interval leaves enough time to finish every new host write within
20 ms. Publication/receive time never replaces that request timestamp. This
module neither opens hardware nor authorizes output. The supervisor must keep
its existing UID preflight, angle-branch, physical, watchdog, voltage, calibrated
motion and final STOP checks, and must validate an actual selected path anew.

The explicit opt-in selector falls back to the ordinary twelve fresh feedback
requests when reuse cannot fit. Faults, changed source context and unconsumed
generations block reuse; they are not silently converted to a fresh admission.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
import struct


PERIOD_NS = INPUT_MAX_AGE_NS = 20_000_000
IDS = tuple(range(1, 13))
BUSES = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}
HOST_ID = 0xFD


def _integer(value, name, *, minimum=0, maximum=2**63 - 1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"Invalid {name}")
    return value


def _text(value, name):
    if type(value) is not str or not value.strip() or len(value) > 256:
        raise ValueError(f"Invalid {name}")


def _sha(value, name):
    if type(value) is not str or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError(f"Invalid {name}")


@dataclass(frozen=True)
class FeedbackContext:
    """Source binding supplied by the supervisor's current verified preflight.

    Type 2 does not carry a UID. These bindings must come from the same current
    boot/power/descriptor generation's identity reads, never from Type 2 itself.
    A valid object is a binding, not evidence that preflight was performed.
    """

    boot_id: str
    power_epoch: str
    source_sha256: str
    profile_sha256: str
    identity_evidence_sha256: str
    expected_uids: tuple
    transport_generation: int

    def __post_init__(self):
        for name in ("boot_id", "power_epoch"):
            _text(getattr(self, name), name)
        for name in ("source_sha256", "profile_sha256", "identity_evidence_sha256"):
            _sha(getattr(self, name), name)
        _integer(self.transport_generation, "transport_generation", minimum=1)
        if (type(self.expected_uids) is not tuple or len(self.expected_uids) != 12 or
                tuple(row[0] for row in self.expected_uids if type(row) is tuple and len(row) == 2) != IDS):
            raise ValueError("Expected immutable ID1..12 UID bindings")
        values = []
        for mid, uid in self.expected_uids:
            _integer(mid, "UID motor ID", minimum=1, maximum=12)
            if type(uid) is not str or len(uid) != 16 or any(c not in "0123456789abcdef" for c in uid):
                raise ValueError("Expected eight-byte UID hex")
            values.append(uid)
        if len(set(values)) != 12:
            raise ValueError("Duplicate UID bindings")


@dataclass(frozen=True)
class Type2Feedback:
    motor_id: int
    bus: str
    request_ns: int
    write_finished_ns: int
    received_ns: int
    request_deadline_ns: int
    tx_wire: bytes
    rx_wire: bytes
    request_kind: int
    mode_state: int
    fault_bits: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float

    def __post_init__(self):
        # Even direct construction cannot substitute decoded values or mutable
        # byte arrays for the immutable wire image retained by the factory.
        _integer(self.motor_id, "feedback motor ID", minimum=1, maximum=12)
        if type(self.bus) is not str or self.bus not in BUSES or self.motor_id not in BUSES[self.bus]:
            raise ValueError("Cross-bus feedback")
        for name in ("request_ns", "write_finished_ns", "received_ns", "request_deadline_ns"):
            _integer(getattr(self, name), name, minimum=1)
        if not self.request_ns <= self.write_finished_ns <= self.received_ns < self.request_deadline_ns:
            raise ValueError("Incomplete or noncausal feedback interval")
        decoded = _decode_wires(self.tx_wire, self.rx_wire, self.motor_id)
        for name, expected in decoded.items():
            if type(getattr(self, name)) is not type(expected) or getattr(self, name) != expected:
                raise ValueError(f"Decoded feedback differs from raw wire: {name}")

    @property
    def protocol_position_rad(self):
        return self.position_rad


def _wire_id(wire):
    if (type(wire) is not bytes or len(wire) != 17 or wire[:2] != b"AT" or
            wire[6] != 8 or wire[-2:] != b"\r\n" or wire[5] & 7 != 4):
        raise ValueError("Noncanonical fixed17B AT envelope")
    return int.from_bytes(wire[2:6], "big") >> 3


def _decode_wires(tx, rx, mid):
    tx_id, rx_id = _wire_id(tx), _wire_id(rx)
    kind = tx_id >> 24 & 31
    if tx_id & 255 != mid or kind not in (1, 4):
        raise ValueError("Only causal Type1/Type4 feedback records are accepted")
    if kind == 4 and (tx_id != (4 << 24 | HOST_ID << 8 | mid) or tx[7:15] != bytes(8)):
        raise ValueError("Version or noncanonical STOP request is not telemetry")
    if (rx_id >> 24 & 31 != 2 or rx_id >> 8 & 255 != mid or rx_id & 255 != HOST_ID or
            rx[7:10] == b"\x00\xc4\x56"):
        raise ValueError("Type2 feedback ID/version mismatch")
    if kind == 4 and rx_id >> 22 & 3 != 0:
        raise ValueError("STOP feedback must confirm stopped mode")
    p, v, torque, temperature = struct.unpack(">4H", rx[7:15])
    return {"request_kind": kind, "mode_state": rx_id >> 22 & 3,
            "fault_bits": rx_id >> 16 & 63,
            "position_rad": p * 25.14 / 65535. - 12.57,
            "velocity_rad_s": v * 100. / 65535. - 50.,
            "torque_nm": torque * 11. / 65535. - 5.5,
            "temperature_c": temperature / 10.}


@dataclass(frozen=True)
class FeedbackSnapshot:
    """One whole exchange, with a generation independent of cycle index.

    Use a single monotonically increasing exchange generation for both fresh
    acquisition and output feedback. A fresh fallback input consumes its own
    generation; the following output is a different, next generation. Do not
    label both snapshots with one cycle number or mark output as policy-consumed
    merely because its motion limits were already checked.
    """
    context: FeedbackContext
    generation: int
    samples: tuple
    published_ns: int

    def __post_init__(self):
        if type(self.context) is not FeedbackContext:
            raise ValueError("Verified preflight context binding required")
        _integer(self.generation, "feedback generation", minimum=1)
        _integer(self.published_ns, "feedback publication", minimum=1)
        if (type(self.samples) is not tuple or len(self.samples) != 12 or
                not all(type(row) is Type2Feedback for row in self.samples) or
                tuple(row.motor_id for row in self.samples) != IDS):
            raise ValueError("An immutable complete ID1..12 generation is required")
        if self.published_ns < max(row.received_ns for row in self.samples):
            raise ValueError("Publication precedes complete reception")


def freeze_type2_snapshot(records_by_bus, *, context, generation, published_ns):
    """Copy one complete output exchange's native records into immutable input.

    Accepts saved record mappings or native fixed-record objects. All raw bytes
    are copied, matched and decoded; a caller's decoded/ok fields are ignored.
    The original request start is retained, including for a previous cycle.
    """
    if not isinstance(records_by_bus, Mapping) or set(records_by_bus) != set(BUSES):
        raise ValueError("Two complete buses are required")
    samples = []
    for bus in BUSES:
        records = records_by_bus[bus]
        if not isinstance(records, (tuple, list)) or len(records) != 6:
            raise ValueError("Six original records per bus are required")
        for record in records:
            if isinstance(record, Mapping):
                get = record.__getitem__
                try:
                    tx, rx = bytes.fromhex(get("tx_hex")), bytes.fromhex(get("rx_hex"))
                except (ValueError, TypeError, KeyError) as error:
                    raise ValueError("Invalid original wire hex") from error
            else:
                get = lambda name: getattr(record, name)
                tx, rx = bytes(get("tx")), bytes(get("rx"))
            if (type(get("written")) is not int or type(get("received")) is not int or
                    get("written") != 17 or get("received") != 17):
                raise ValueError("Incomplete fixed17B native record")
            mid = _wire_id(tx) & 255
            decoded = _decode_wires(tx, rx, mid)
            samples.append(Type2Feedback(mid, bus, get("start_ns"), get("finish_ns"),
                get("received_ns"), get("deadline_ns"), tx, rx, **decoded))
    return FeedbackSnapshot(context, generation,
                            tuple(sorted(samples, key=lambda row: row.motor_id)), published_ns)


@dataclass(frozen=True)
class IMUInterval:
    read_started_ns: int
    read_finished_ns: int
    sequence: int
    accel_m_s2: tuple
    gyro_rad_s: tuple

    def __post_init__(self):
        _integer(self.read_started_ns, "IMU request", minimum=1)
        _integer(self.read_finished_ns, "IMU reception", minimum=self.read_started_ns)
        _integer(self.sequence, "IMU sequence", minimum=1)
        for name in ("accel_m_s2", "gyro_rad_s"):
            vector = getattr(self, name)
            if (type(vector) is not tuple or len(vector) != 3 or
                    not all(type(v) in (int, float) and math.isfinite(v) for v in vector)):
                raise ValueError("Immutable finite IMU vectors required")


@dataclass(frozen=True)
class FeedbackSelection:
    mode: str
    reasons: tuple
    snapshot: FeedbackSnapshot | None
    context: FeedbackContext
    imu: IMUInterval
    release_ns: int
    now_ns: int
    oldest_input_request_ns: int | None
    input_source_deadline_ns: int | None
    final_write_deadline_ns: int
    projected_final_write_ns: int
    acquisition_spread_ns: int | None
    remaining_budget_ns: int | None

    def __post_init__(self):
        if (type(self.mode) is not str or self.mode not in ("REUSE_14", "REFRESH_26", "BLOCK") or
                type(self.reasons) is not tuple or not all(type(r) is str for r in self.reasons) or
                type(self.context) is not FeedbackContext or
                type(self.imu) is not IMUInterval):
            raise ValueError("Invalid immutable cadence decision")
        for name in ("release_ns", "now_ns", "final_write_deadline_ns", "projected_final_write_ns"):
            _integer(getattr(self, name), name, minimum=1)
        if self.now_ns < self.release_ns or self.projected_final_write_ns <= self.now_ns:
            raise ValueError("Noncausal cadence decision")
        if self.mode == "REUSE_14":
            if (type(self.snapshot) is not FeedbackSnapshot or self.snapshot.context != self.context or
                    self.snapshot.published_ns > self.now_ns or self.imu.read_finished_ns > self.now_ns or
                    self.reasons):
                raise ValueError("Incomplete feedback-reuse decision")
            oldest = min(self.imu.read_started_ns, *(s.request_ns for s in self.snapshot.samples))
            latest = max(self.imu.read_finished_ns, *(s.received_ns for s in self.snapshot.samples))
            deadline = min(self.release_ns + PERIOD_NS, oldest + INPUT_MAX_AGE_NS)
            if (self.oldest_input_request_ns != oldest or self.input_source_deadline_ns != oldest + INPUT_MAX_AGE_NS or
                    self.final_write_deadline_ns != deadline or self.acquisition_spread_ns != latest - oldest or
                    self.remaining_budget_ns != deadline - self.now_ns or self.projected_final_write_ns > deadline or
                    latest - oldest > INPUT_MAX_AGE_NS or any(s.fault_bits for s in self.snapshot.samples) or
                    len({s.mode_state for s in self.snapshot.samples}) != 1 or
                    self.snapshot.samples[0].mode_state not in (0, 2)):
                raise ValueError("Original source-bound reuse decision differs from immutable inputs")

    @property
    def requests_per_cycle(self):
        return {"REUSE_14": 14, "REFRESH_26": 26, "BLOCK": 0}[self.mode]

    @property
    def output_allowed(self):
        return False


def select_feedback_acquisition(snapshot, *, context, release_ns, now_ns, imu,
                                work_to_last_write_budget_ns, last_consumed_generation,
                                previous_imu_sequence, bootstrap_complete, opt_in=False,
                                required_mode=2, max_acquisition_spread_ns=INPUT_MAX_AGE_NS):
    """Choose reuse only when all original inputs fit a declared remaining budget.

    ``work_to_last_write_budget_ns`` includes inference, all validation/encoding,
    scheduling and both buses' final writes. It is a conservative caller budget,
    not a latency estimate derived by this module. Equality at 20 ms is allowed;
    no startup age extension or timestamp re-stamping exists here. A runtime
    must recheck before output and verify actual final writes afterward.
    """
    if type(context) is not FeedbackContext or type(imu) is not IMUInterval:
        raise ValueError("Immutable current context and IMU interval required")
    for name, value in (("release", release_ns), ("now", now_ns)):
        _integer(value, name, minimum=1)
    if now_ns < release_ns or imu.read_finished_ns > now_ns:
        raise ValueError("Noncausal current selection time")
    _integer(work_to_last_write_budget_ns, "remaining write budget", minimum=1, maximum=PERIOD_NS)
    _integer(last_consumed_generation, "last consumed generation")
    _integer(previous_imu_sequence, "previous IMU sequence")
    _integer(max_acquisition_spread_ns, "acquisition spread", minimum=1, maximum=INPUT_MAX_AGE_NS)
    if type(required_mode) is not int or required_mode not in (0, 2):
        raise ValueError("Exact stopped or active feedback mode required")
    if type(opt_in) is not bool or type(bootstrap_complete) is not bool:
        raise ValueError("Explicit boolean cadence selections required")
    projected = now_ns + work_to_last_write_budget_ns
    cycle_deadline = release_ns + PERIOD_NS
    oldest = deadline = spread = remaining = None

    def result(mode, *reasons):
        return FeedbackSelection(mode, tuple(reasons), snapshot, context, imu, release_ns,
            now_ns, oldest, deadline,
            min(cycle_deadline, deadline or cycle_deadline) if mode == "REUSE_14" else cycle_deadline,
            projected, spread, remaining)

    if imu.sequence <= previous_imu_sequence:
        return result("BLOCK", "repeated_imu_sequence")
    if now_ns - imu.read_started_ns > INPUT_MAX_AGE_NS:
        return result("BLOCK", "stale_current_imu")
    if snapshot is None:
        return result("REFRESH_26", "bootstrap_missing_feedback")
    if type(snapshot) is not FeedbackSnapshot:
        raise ValueError("Immutable all-axis snapshot required")
    if snapshot.context != context:
        return result("BLOCK", "changed_boot_power_source_uid_or_transport_generation")
    if snapshot.published_ns > now_ns:
        return result("BLOCK", "feedback_not_yet_published")
    if any(row.fault_bits or row.mode_state != required_mode for row in snapshot.samples):
        return result("BLOCK", "feedback_fault_or_mode")
    if snapshot.generation <= last_consumed_generation:
        return result("BLOCK", "repeated_feedback_generation")
    if snapshot.generation != last_consumed_generation + 1:
        return result("REFRESH_26", "feedback_generation_gap_requires_refresh")
    oldest = min(imu.read_started_ns, *(row.request_ns for row in snapshot.samples))
    latest = max(imu.read_finished_ns, *(row.received_ns for row in snapshot.samples))
    spread = latest - oldest
    deadline = oldest + INPUT_MAX_AGE_NS
    remaining = min(cycle_deadline, deadline) - now_ns
    if not opt_in:
        return result("REFRESH_26", "reuse_not_selected")
    if not bootstrap_complete:
        return result("REFRESH_26", "fresh_bootstrap_required")
    if spread > max_acquisition_spread_ns:
        return result("REFRESH_26", "original_input_interval_spread_exceeded")
    if projected > min(cycle_deadline, deadline):
        return result("REFRESH_26", "original_input_to_final_write_budget_exceeded")
    return result("REUSE_14")


def validate_final_write(selection, *, final_host_write_ns, context, generation):
    """Validate the actual write result; this cannot undo an already-late write.

    Before dispatch the supervisor must bound/recheck remaining budget and pass
    this original absolute deadline to the transport. This postcondition retains
    any miss as a miss and never changes request timestamps or resets a budget.
    """
    if type(selection) is not FeedbackSelection or selection.mode != "REUSE_14":
        raise ValueError("No admitted feedback-reuse plan")
    _integer(generation, "final feedback generation", minimum=1)
    _integer(final_host_write_ns, "final host write", minimum=selection.now_ns)
    if context != selection.context or generation != selection.snapshot.generation:
        raise ValueError("Context or generation changed before final write")
    if final_host_write_ns > selection.final_write_deadline_ns:
        raise TimeoutError("Original input-to-final-write 20 ms deadline exceeded")
    return final_host_write_ns - selection.oldest_input_request_ns


def feedback_rows(snapshot):
    """Adapt immutable samples to the existing feedback_sample row interface.

    Its unchanged calibrated motion, discontinuity and mode/fault validators
    still must run. Track the *policy-consumed* previous input separately from
    the already-validated output cache; validating a reply once as output does
    not mean it was already consumed by a model. No original time is changed.
    """
    if type(snapshot) is not FeedbackSnapshot:
        raise ValueError("Immutable all-axis feedback required")
    return {(row.motor_id, "feedback"): (row, row.request_ns, row.received_ns)
            for row in snapshot.samples}


def recheck_before_dispatch(selection, *, now_ns, remaining_write_budget_ns,
                            context, generation):
    """Return the original absolute deadline, without restarting its budget."""
    if type(selection) is not FeedbackSelection or selection.mode != "REUSE_14":
        raise ValueError("No admitted feedback-reuse plan")
    _integer(now_ns, "dispatch time", minimum=selection.now_ns)
    _integer(generation, "dispatch feedback generation", minimum=1)
    _integer(remaining_write_budget_ns, "remaining dispatch budget", minimum=1, maximum=PERIOD_NS)
    if context != selection.context or generation != selection.snapshot.generation:
        raise ValueError("Context or generation changed before dispatch")
    if now_ns + remaining_write_budget_ns > selection.final_write_deadline_ns:
        raise TimeoutError("Original remaining final-write budget exhausted")
    return selection.final_write_deadline_ns
