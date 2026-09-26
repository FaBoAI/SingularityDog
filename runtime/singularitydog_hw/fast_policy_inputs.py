"""Immutable, offline Type2 input preparation; no transport or model output.

Each fresh source cycle must be prepared again. Preparation validates raw wire
bytes and captures owned evidence hashes; it is not a free operation. Replaying
one prepared capture measures assembly CPU only, never fresh/live throughput.
The existing policy observer still applies calibration and target-range checks.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import struct

from .event_snapshot import snapshot_event

MAX_AGE_NS = MAX_SPREAD_NS = 100_000_000
HOST_ID = 0xFD
UNVERIFIED_FLAGS = (
    "approved_for_runtime", "calibration_verified", "velocity_scale_verified",
    "full_pipeline_20ms_verified", "full_controller_50Hz_verified",
    "motor_enabling_available", "motion_command_available", "configuration_available",
    "automatic_retry", "can_bitrate_verified", "late_same_key_previous_cycle_disambiguation",
    "sensor_internal_sample_time_verified", "unwrapping_applied",
)


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _stamp(value, label):
    _require(type(value) is int and 0 <= value < 2**63, "Invalid " + label)
    return value


def _finite(value, label):
    _require(type(value) in (int, float), "Invalid " + label)
    try:
        valid = math.isfinite(value)
    except (OverflowError, ValueError):
        valid = False
    _require(valid, "Nonfinite " + label)
    return value


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True, slots=True)
class _Motor:
    motor_id: int
    can_id: int
    flags: int
    position: float
    velocity: float
    start: int
    finish: int
    received: int
    deadline: int
    available: int

    def check(self, mid, tick):
        _require(type(self.motor_id) is int and self.motor_id == mid,
                 "Missing, repeated or invalid motor ID")
        _require(self.flags == 4 and self.can_id == (2 << 24) | (mid << 8) | HOST_ID,
                 "Type2 must match ID/host and mode0/fault0")
        _require(self.start <= self.finish <= self.received <= self.available <= tick
                 and self.received < self.deadline <= self.start+250_000_000,
                 "Noncausal or expired source interval")
        _require(math.isfinite(self.position) and -12.57 <= self.position <= 12.57
                 and math.isfinite(self.velocity) and -50. <= self.velocity <= 50.,
                 "Candidate position or velocity outside declared range")


def _motor(row, cycle):
    _require(type(row) is dict and row.get("kind") == "pipeline_reply"
             and row.get("ok") is True and type(row.get("cycle")) is int
             and row["cycle"] == cycle and row.get("parameter") == "stop_feedback",
             "Expected successful combined reply in the selected cycle")
    mid = row.get("motor_id")
    _require(type(mid) is int and 1 <= mid <= 12, "Invalid motor ID")
    _require(row.get("write_call_entered") is True
             and type(row.get("write_expected_bytes")) is int and row["write_expected_bytes"] == 17
             and type(row.get("write_returned_bytes")) is int and row["write_returned_bytes"] == 17,
             "Require a complete canonical request write")
    result = row.get("result")
    _require(type(result) is dict and result.get("ok") is True
             and type(result.get("motor_id")) is int and result["motor_id"] == mid
             and result.get("parameter") == "stop_feedback", "Mismatched decoded result")
    _require(all(result.get(flag) is False for flag in UNVERIFIED_FLAGS),
             "Require explicit unverified diagnostic flags")
    _require(result.get("position_velocity_in_one_reply") is True,
             "Expected position and velocity in one reply")
    raw = result.get("raw_frame")
    _require(type(raw) is dict, "Missing original Type2 raw frame")
    wire_hex = raw.get("wire_hex")
    _require(type(wire_hex) is str and len(wire_hex) == 34, "Invalid raw wire hex")
    try:
        wire = bytes.fromhex(wire_hex)
    except ValueError as error:
        raise ValueError("Invalid raw wire hex") from error
    _require(wire.hex() == wire_hex and len(wire) == 17 and wire[:2] == b"AT"
             and wire[6] == 8 and wire[-2:] == b"\r\n", "Noncanonical raw Type2 wire")
    encoded = int.from_bytes(wire[2:6], "big")
    can_id, flags = encoded >> 3, encoded & 7
    _require(flags == 4 and can_id == (2 << 24) | (mid << 8) | HOST_ID,
             "Type2 must match ID/host and mode0/fault0")
    data = wire[7:15]
    _require(data[:3] != b"\x00\xc4\x56", "Version-shaped reply is not ordinary Type2")
    expected_raw = {"can_id": can_id, "type": 2, "source_id": mid,
                    "destination_id": HOST_ID, "flags": flags,
                    "data_hex": data.hex(), "wire_hex": wire.hex()}
    _require(set(raw) == set(expected_raw)
             and all(type(raw[key]) is type(value) and raw[key] == value
                     for key, value in expected_raw.items()), "Raw-frame metadata differs from wire")
    p, v, torque, temp = struct.unpack(">4H", data)
    for name, value in (("mode_state", 0), ("fault_bits", 0), ("position_u16", p),
                        ("velocity_u16", v), ("torque_u16", torque), ("temperature_u16", temp)):
        _require(type(result.get(name)) is int and result[name] == value,
                 "Decoded mode/fault or raw16 differs from wire")
    scale = lambda value, limit: value*(2.*limit)/65535.-limit
    expected = {"position_rad_candidate": scale(p, 12.57),
                "velocity_rad_s_candidate": scale(v, 50.),
                "torque_nm_candidate": scale(torque, 5.5), "temperature_c": temp/10.}
    for name, value in expected.items():
        _require(_finite(result.get(name), name) == value, "Candidate scale differs from raw16")
    start, finish, received, deadline = (
        _stamp(row.get(key), key) for key in ("write_started_monotonic_ns",
            "write_finished_monotonic_ns", "received_monotonic_ns", "deadline_monotonic_ns"))
    available = _stamp(row.get("available_monotonic_ns", received), "reply availability")
    motor = _Motor(mid, can_id, flags, expected["position_rad_candidate"],
                   expected["velocity_rad_s_candidate"], start, finish, received, deadline, available)
    motor.check(mid, available)
    return motor, expected_raw


@dataclass(frozen=True, slots=True)
class PreparedCycle:
    """Owned immutable source values. Prepare once for EACH incoming cycle.

    Use ``prepare_cycle`` rather than constructing this record directly. No
    mutable input or output object is retained. A new tick recomputes ages and
    rejects stale values; source timestamps are never moved forward.
    """
    motors: tuple
    accel: tuple
    gyro: tuple
    imu_start: int
    imu_end: int
    imu_available: int
    cycle: int
    evidence_hashes: tuple

    def snapshot(self, tick_ns):
        tick = _stamp(tick_ns, "tick")
        _require(type(self.cycle) is int and 1 <= self.cycle <= 20
                 and len(self.motors) == 12, "Invalid fixed cycle")
        intervals = []
        motors = []
        for mid, motor in enumerate(self.motors, 1):
            motor.check(mid, tick)
            intervals.append((motor.start, motor.received))
            for name, value, unit in (("position", motor.position, "rad"),
                                       ("velocity", motor.velocity, "rad_s")):
                motors.append({"motor_id": mid, "parameter": name, "value": value,
                    "unit": unit, "request_ns": motor.start, "received_ns": motor.received,
                    "age_upper_bound_ns": tick-motor.start})
        _require(self.imu_start <= self.imu_end <= self.imu_available <= tick,
                 "Noncausal IMU interval or availability")
        _require(len(self.accel) == len(self.gyro) == 3
                 and all(type(x) in (int, float) and math.isfinite(x)
                         for vector in (self.accel, self.gyro) for x in vector), "Invalid IMU vector")
        intervals.append((self.imu_start, self.imu_end))
        earliest, latest = min(a for a, _ in intervals), max(b for _, b in intervals)
        age, spread = tick-earliest, latest-earliest
        _require(age <= MAX_AGE_NS and spread <= MAX_SPREAD_NS,
                 "Stale or excessive diagnostic acquisition spread")
        flags = {"sensor_type2_candidate": True, "motor_can_reply_type": 2,
            "type17_fabricated": False, "stop_feedback_state_changing": True,
            "cycle": self.cycle, "velocity_unverified": True, "velocity_scale_verified": False,
            "position_range_rad_candidate": [-12.57, 12.57],
            "velocity_range_rad_s_candidate": [-50., 50.],
            "calibration_unverified": True, "calibration_verified": False,
            "sensor_internal_sample_time_verified": False,
            "fresh_identity_match_verified": False, "source_timestamps_changed": False,
            "unwrapping_applied": False, "clipping_applied": False,
            "output_allowed": False, "approved_for_runtime": False,
            "full_pipeline_20ms_verified": False,
            "raw_type2_frames_canonical_json_sha256": self.evidence_hashes[0],
            "source_reply_rows_canonical_json_sha256": self.evidence_hashes[1],
            "source_imu_event_canonical_json_sha256": self.evidence_hashes[2]}
        return {"status": "DIAGNOSTIC_READY", "output_allowed": False, "tick_ns": tick,
            "blocked_reasons": [], "motors": motors,
            "imu": {"frame": "raw_sensor", "accel_m_s2": list(self.accel), "gyro_rad_s": list(self.gyro),
                "read_started_ns": self.imu_start, "read_finished_ns": self.imu_end,
                "age_upper_bound_ns": tick-self.imu_start},
            "oldest_observation_age_ns": age, "acquisition_spread_ns": spread,
            "receive_spread_ns": latest-min(b for _, b in intervals),
            "max_age_ns": MAX_AGE_NS, "max_spread_ns": MAX_SPREAD_NS, "source_flags": flags}


def prepare_cycle(reply_rows, imu_sample, *, cycle=1):
    """Validate a whole captured cycle, own its values, and compute evidence.

    This full preparation cost MUST be reported separately from hot assembly.
    It runs for every new source cycle, even when numbers happen to be equal.
    The saved capture may then be replayed without re-decoding or re-hashing.
    """
    _require(type(cycle) is int and 1 <= cycle <= 20, "Invalid fixed cycle")
    _require(type(reply_rows) in (list, tuple) and len(reply_rows) == 12,
             "Exactly twelve successful same-cycle replies are required")
    rows, imu = snapshot_event(reply_rows), snapshot_event(imu_sample)
    records = {}
    for row in rows:
        motor, raw = _motor(row, cycle)
        _require(motor.motor_id not in records, "Repeated motor ID")
        records[motor.motor_id] = (motor, raw, row)
    _require(set(records) == set(range(1, 13)), "All twelve IDs must be present")
    _require(type(imu) is dict and imu.get("kind") == "imu"
             and imu.get("frame") in ("sensor", "raw_sensor"), "Require original sensor IMU event")
    vectors = []
    for key in ("accel_m_s2", "gyro_rad_s"):
        value = imu.get(key)
        _require(type(value) in (list, tuple) and len(value) == 3, "Invalid IMU vector")
        vectors.append(tuple(_finite(x, key) for x in value))
    start = _stamp(imu.get("read_started_monotonic_ns"), "IMU start")
    end = _stamp(imu.get("read_finished_monotonic_ns"), "IMU finish")
    available = _stamp(imu.get("available_monotonic_ns", end), "IMU availability")
    _require(start <= end <= available, "Noncausal IMU interval or availability")
    ordered = tuple(records[i] for i in range(1, 13))
    prepared = PreparedCycle(tuple(x[0] for x in ordered), *vectors, start, end, available, cycle,
        (_digest([x[1] for x in ordered]), _digest([x[2] for x in ordered]), _digest(imu)))
    # Check the source interval without allocating a throwaway snapshot or
    # claiming that any historical timestamp represents the current clock.
    earliest = min(start, *(m.start for m in prepared.motors))
    latest = max(end, *(m.received for m in prepared.motors))
    newest_available = max(available, *(m.available for m in prepared.motors))
    _require(newest_available-earliest <= MAX_AGE_NS and latest-earliest <= MAX_SPREAD_NS,
             "Stale or excessive diagnostic acquisition spread")
    return prepared
