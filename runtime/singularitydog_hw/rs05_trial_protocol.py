"""Offline codec for a supported, explicitly selected single-joint RS05 trial.

There is no transport, runner, clock, or hardware access in this module. Explicit
phases label individual frames; they do NOT implement an arming state machine.
The caller must establish support, a physical power cutoff, fresh measurements,
firmware scaling, watchdog behavior, and a verified stop before enabling motion.
Each call targets one integer motor ID in 1..12, defaulting to ID 1. Broadcasts,
automatic motor selection, and automatic progression to another motor are absent.
The caller must independently confirm that the selected actuator is an RS05;
neither its CAN ID nor its UID establishes the model.

Type 1 uses the current RS05 manual/official SDK profile: +/-12.57 rad,
+/-50 rad/s, +/-5.5 Nm, Kp 0..500, Kd 0..5. Older firmware may differ.
POSITION fixes Kp=0.5/Kd=0.02 and POSITION_STEP2 fixes Kp=5.0/Kd=0.05,
both with +/-1 degree targets. Separately selected POSITION_VISIBLE permits
+/-3 degrees and POSITION_STEP5 permits +/-5 degrees, both with Kp=3.0/Kd=0.15.
POSITION_STEP5_KP4 is an explicitly selected diagnostic with the same5-degree
bound and Kd=0.15, changing Kp only to4.0. There is no automatic escalation.
POSITION_STEP5_RR_HIP_KP6 permits Kp=6.0 only for explicit motor ID9, with the
same bounds and Kd. The runner additionally requires the fixed RR hip-only plan.
POSITION_ROLE_FRONT_HIP_KP6 permits Kp=6.0 only for front-hip IDs3/6, with
the same 5-degree bound and Kd; the role-group runner fixes their signed plan.
POSITION_ROLE_FRONT_HIP_KP12 selects Kp=12.0 only for the same two IDs and
5-degree bound; it requires a separate reviewed role-group package.
These are trial choices, not vendor safety
limits or automatic gain tuning. No gain set or zero velocity/torque reference imposes a physical
speed or total torque cap. Even nominal zero has uint16 quantization bias.

Sources: RS05User Manual260713 sections 3.3.6, 4.1.2--5, 4.1.8, 4.4;
RobStride/EDULITE_A3 el_a3_sdk/{protocol,can_driver,utils}.py.
"""
from dataclasses import dataclass
from enum import Enum
import math
import struct

from .can_readonly import Frame


MOTOR_ID = 1
HOST_ID = 0xFD
WATCHDOG_TICKS = 4000  # 50 us/tick = 200 ms; volatile Type 18 only.
MAX_OFFSET_RAD = math.pi / 180.0
VISIBLE_MAX_OFFSET_RAD = math.radians(3.0)
STEP5_MAX_OFFSET_RAD = math.radians(5.0)
POSITION_MIN = -12.57
POSITION_MAX = 12.57
TRIAL_KP = 0.5
TRIAL_KD = 0.02
STEP2_KP = 5.0
STEP2_KD = 0.05
VISIBLE_KP = 3.0
VISIBLE_KD = 0.15
STEP5_KP4_DIAGNOSTIC_KP = 4.0
STEP5_RR_HIP_KP6_DIAGNOSTIC_KP = 6.0
STEP5_FR_HIP_KP6_DIAGNOSTIC_KP = 6.0
ROLE_FRONT_HIP_KP6_DIAGNOSTIC_KP = 6.0
ROLE_FRONT_HIP_KP12_DIAGNOSTIC_KP = 12.0
FR_CURRENT_KP12_DIAGNOSTIC_KP = 12.0
FR_STEP4_KP12_DIAGNOSTIC_KP = 12.0
FR_STEP4_THIGH_KP18_DIAGNOSTIC_KP = 18.0
FR_STEP4_MAX_OFFSET_RAD = math.radians(4.5)


class TrialPhase(Enum):
    WATCHDOG_SETUP = "watchdog_setup"
    WATCHDOG_READBACK = "watchdog_readback"
    ENABLE = "enable"
    ZERO_GAIN = "zero_gain"
    POSITION = "position"
    POSITION_STEP2 = "position_step2"
    POSITION_VISIBLE = "position_visible"
    POSITION_STEP5 = "position_step5"
    POSITION_STEP5_KP4 = "position_step5_kp4"
    POSITION_ROLE_THIGH_KP6 = "position_role_thigh_kp6"
    POSITION_ROLE_THIGH_KP12 = "position_role_thigh_kp12"
    POSITION_ROLE_HIP_HOLD_KP12 = "position_role_hip_hold_kp12"
    POSITION_STEP5_RR_HIP_KP6 = "position_step5_rr_hip_kp6"
    POSITION_STEP5_FR_HIP_KP6 = "position_step5_fr_hip_kp6"
    POSITION_ROLE_FRONT_HIP_KP6 = "position_role_front_hip_kp6"
    POSITION_ROLE_FRONT_HIP_KP12 = "position_role_front_hip_kp12"
    POSITION_ROLE_FRONT_HIP_KP12_STEP10 = "position_role_front_hip_kp12_step10"
    POSITION_CURRENT_FR_THIGH_KP12 = "position_current_fr_thigh_kp12"
    POSITION_CURRENT_FR_HIP_KP12 = "position_current_fr_hip_kp12"
    POSITION_STEP4_FR_THIGH_KP12 = "position_step4_fr_thigh_kp12"
    POSITION_STEP4_FR_THIGH_KP18 = "position_step4_fr_thigh_kp18"
    POSITION_STEP4_FR_HIP_KP12 = "position_step4_fr_hip_kp12"
    STOP = "stop"


@dataclass(frozen=True)
class Type2Feedback:
    """Declared-profile feedback; position is not unwrapped or a joint zero."""
    mode_state: int
    fault_bits: int
    position_u16: int
    protocol_position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float


@dataclass(frozen=True)
class DirectPosition:
    """Type 17 load-side float, kept separate from Type 2 cyclic position."""
    direct_position_rad: float


def _selected_id(motor_id):
    if type(motor_id) is not int or not 1 <= motor_id <= 12:
        raise ValueError("Select exactly one integer motor ID in 1..12; no broadcast")


def _phase_is(phase, expected):
    if type(phase) is not TrialPhase or phase is not expected:
        raise ValueError(f"Explicit phase {expected.value} is required")


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be a finite real number, not bool")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} is too large") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _u16(value, minimum, maximum):
    # The public API fixes all ranges. No silent saturation of unsafe inputs.
    value = _number(value, "encoded value")
    if not minimum <= value <= maximum:
        raise ValueError("Value is outside the declared RS05 encoding range")
    return int((value - minimum) * 65535.0 / (maximum - minimum))


def _wire(can_id, data):
    return (b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08"
            + data + b"\r\n")


def enable_request(*, phase, motor_id=MOTOR_ID):
    _selected_id(motor_id)
    _phase_is(phase, TrialPhase.ENABLE)
    return _wire((3 << 24) | (HOST_ID << 8) | motor_id, bytes(8))


def stop_request(*, phase, motor_id=MOTOR_ID):
    """Stop with byte 0 = 0; never clear a fault or query firmware implicitly."""
    _selected_id(motor_id)
    _phase_is(phase, TrialPhase.STOP)
    return _wire((4 << 24) | (HOST_ID << 8) | motor_id, bytes(8))


def watchdog_setup_request(*, phase, motor_id=MOTOR_ID):
    """Encode only a volatile 200 ms watchdog value, never a flash-save frame."""
    _selected_id(motor_id)
    _phase_is(phase, TrialPhase.WATCHDOG_SETUP)
    data = struct.pack("<H2xI", 0x7028, WATCHDOG_TICKS)
    return _wire((18 << 24) | (HOST_ID << 8) | motor_id, data)


def watchdog_readback_request(*, phase, motor_id=MOTOR_ID):
    """A successful readback is not proof that physical cutoff was exercised."""
    _selected_id(motor_id)
    _phase_is(phase, TrialPhase.WATCHDOG_READBACK)
    return _wire((17 << 24) | (HOST_ID << 8) | motor_id,
                 struct.pack("<H", 0x7028) + bytes(6))


def motion_request(*, phase, center_rad, offset_rad=0.0, motor_id=MOTOR_ID):
    """Encode zero-gain initialization or an explicitly bounded fixed-gain target.

    center_rad must be a fresh, independently validated motor-coordinate
    reference. This function cannot check freshness, sign, support, or calibration.
    No modulo conversion or unwrapping is performed. POSITION and POSITION_STEP2
    require +/-1 degree headroom; POSITION_VISIBLE requires +/-3 degrees and
    All POSITION_STEP5 phases require +/-5 degrees headroom; the explicitly
    selected four-thigh diagnostic allows +/-10 degrees. Each phase must be explicitly selected; no
    measurement or unsuccessful motion automatically changes gains or bounds.
    """
    _selected_id(motor_id)
    if type(phase) is not TrialPhase or phase not in (
            TrialPhase.ZERO_GAIN, TrialPhase.POSITION, TrialPhase.POSITION_STEP2,
            TrialPhase.POSITION_VISIBLE, TrialPhase.POSITION_STEP5, TrialPhase.POSITION_STEP5_KP4,
            TrialPhase.POSITION_ROLE_THIGH_KP6, TrialPhase.POSITION_ROLE_THIGH_KP12,
            TrialPhase.POSITION_ROLE_HIP_HOLD_KP12,
            TrialPhase.POSITION_STEP5_RR_HIP_KP6, TrialPhase.POSITION_STEP5_FR_HIP_KP6,
            TrialPhase.POSITION_ROLE_FRONT_HIP_KP6,
            TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
            TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10,
            TrialPhase.POSITION_CURRENT_FR_THIGH_KP12, TrialPhase.POSITION_CURRENT_FR_HIP_KP12,
            TrialPhase.POSITION_STEP4_FR_THIGH_KP12, TrialPhase.POSITION_STEP4_FR_THIGH_KP18,
            TrialPhase.POSITION_STEP4_FR_HIP_KP12):
        raise ValueError("Motion requires an explicit zero-gain or position trial phase")
    if phase is TrialPhase.POSITION_STEP5_RR_HIP_KP6 and motor_id != 9:
        raise ValueError("RR hip Kp6 diagnostic requires explicit motor ID9")
    if phase is TrialPhase.POSITION_STEP5_FR_HIP_KP6 and motor_id != 3:
        raise ValueError("FR hip Kp6 diagnostic requires explicit motor ID3")
    if phase in (TrialPhase.POSITION_ROLE_FRONT_HIP_KP6,
                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10) and motor_id not in (3, 6):
        raise ValueError("Front-hip pair diagnostic requires motor ID3 or ID6")
    if phase is TrialPhase.POSITION_ROLE_THIGH_KP6 and motor_id not in (2, 5, 8, 11):
        raise ValueError("Role-thigh Kp6 diagnostic requires an upper-leg motor ID")
    if phase is TrialPhase.POSITION_ROLE_THIGH_KP12 and motor_id not in (2, 5, 8, 11):
        raise ValueError("Role-thigh Kp12 diagnostic requires an upper-leg motor ID")
    if phase is TrialPhase.POSITION_ROLE_HIP_HOLD_KP12 and motor_id not in (3, 6, 9, 12):
        raise ValueError("Role-hip Kp12 hold requires a hip motor ID")
    current_id = {TrialPhase.POSITION_CURRENT_FR_THIGH_KP12: 2,
                  TrialPhase.POSITION_CURRENT_FR_HIP_KP12: 3}.get(phase)
    if current_id is not None and motor_id != current_id:
        raise ValueError("FR current Kp12 phase requires its exact selected motor ID")
    step4_id = {TrialPhase.POSITION_STEP4_FR_THIGH_KP12: 2,
                TrialPhase.POSITION_STEP4_FR_THIGH_KP18: 2,
                TrialPhase.POSITION_STEP4_FR_HIP_KP12: 3}.get(phase)
    if step4_id is not None and motor_id != step4_id:
        raise ValueError("FR step4 diagnostic phase requires its exact selected motor ID")
    center = _number(center_rad, "center_rad")
    offset = _number(offset_rad, "offset_rad")
    if not POSITION_MIN <= center <= POSITION_MAX:
        raise ValueError("Center requires an unambiguous in-range motor reference")
    max_offset = {TrialPhase.POSITION_VISIBLE: VISIBLE_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_STEP5: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_STEP5_KP4: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_ROLE_THIGH_KP6: math.radians(10.),
                  TrialPhase.POSITION_ROLE_THIGH_KP12: math.radians(10.),
                  TrialPhase.POSITION_ROLE_HIP_HOLD_KP12: 0.,
                  TrialPhase.POSITION_STEP5_RR_HIP_KP6: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_STEP5_FR_HIP_KP6: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_ROLE_FRONT_HIP_KP6: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_ROLE_FRONT_HIP_KP12: STEP5_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10: math.radians(10.),
                  TrialPhase.POSITION_CURRENT_FR_THIGH_KP12: 0.,
                  TrialPhase.POSITION_CURRENT_FR_HIP_KP12: 0.,
                  TrialPhase.POSITION_STEP4_FR_THIGH_KP12: FR_STEP4_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_STEP4_FR_THIGH_KP18: FR_STEP4_MAX_OFFSET_RAD,
                  TrialPhase.POSITION_STEP4_FR_HIP_KP12: FR_STEP4_MAX_OFFSET_RAD}.get(phase, MAX_OFFSET_RAD)
    if abs(offset) > max_offset + (1e-12 if phase in (
            TrialPhase.POSITION_ROLE_THIGH_KP6,
            TrialPhase.POSITION_ROLE_THIGH_KP12) else 0.):
        raise ValueError("Trial offset exceeds the selected phase's bound")
    if phase is TrialPhase.ZERO_GAIN:
        if offset != 0.0:
            raise ValueError("Zero-gain initialization cannot include an offset")
        kp, kd = 0.0, 0.0
    else:
        if not (POSITION_MIN + max_offset <= center
                <= POSITION_MAX - max_offset):
            raise ValueError("Center has insufficient headroom for the selected phase")
        if phase in (TrialPhase.POSITION_VISIBLE, TrialPhase.POSITION_STEP5):
            kp, kd = VISIBLE_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_STEP5_KP4:
            kp, kd = STEP5_KP4_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_ROLE_THIGH_KP6:
            kp, kd = 6.0, VISIBLE_KD
        elif phase is TrialPhase.POSITION_ROLE_THIGH_KP12:
            kp, kd = 12.0, VISIBLE_KD
        elif phase is TrialPhase.POSITION_ROLE_HIP_HOLD_KP12:
            kp, kd = 12.0, VISIBLE_KD
        elif phase is TrialPhase.POSITION_STEP5_RR_HIP_KP6:
            kp, kd = STEP5_RR_HIP_KP6_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_STEP5_FR_HIP_KP6:
            kp, kd = STEP5_FR_HIP_KP6_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_ROLE_FRONT_HIP_KP6:
            kp, kd = ROLE_FRONT_HIP_KP6_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase in (TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                      TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10):
            kp, kd = ROLE_FRONT_HIP_KP12_DIAGNOSTIC_KP, VISIBLE_KD
        elif current_id is not None:
            kp, kd = FR_CURRENT_KP12_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_STEP4_FR_THIGH_KP18:
            kp, kd = FR_STEP4_THIGH_KP18_DIAGNOSTIC_KP, VISIBLE_KD
        elif step4_id is not None:
            kp, kd = FR_STEP4_KP12_DIAGNOSTIC_KP, VISIBLE_KD
        elif phase is TrialPhase.POSITION_STEP2:
            kp, kd = STEP2_KP, STEP2_KD
        else:
            kp, kd = TRIAL_KP, TRIAL_KD
    target = center + offset
    torque_raw = _u16(0.0, -5.5, 5.5)
    data = struct.pack(">4H", _u16(target, POSITION_MIN, POSITION_MAX),
                       _u16(0.0, -50.0, 50.0),
                       _u16(kp, 0.0, 500.0), _u16(kd, 0.0, 5.0))
    # Type 1's middle 16 bits hold torque, NOT the host ID.
    return _wire((1 << 24) | (torque_raw << 8) | motor_id, data)


def _reply(frame, kind, motor_id):
    _selected_id(motor_id)
    if not isinstance(frame, Frame):
        raise ValueError("A parsed AT Frame is required")
    if type(frame.can_id) is not int or not 0 <= frame.can_id <= 0x1FFFFFFF:
        raise ValueError("CAN ID must be an unsigned 29-bit integer")
    if type(frame.flags) is not int or frame.flags != 4:
        raise ValueError("Only extended CAN data frames are accepted")
    if type(frame.data) is not bytes or len(frame.data) != 8:
        raise ValueError("Reply DLC must be exactly 8")
    if type(frame.wire) is not bytes or frame.wire != _wire(frame.can_id, frame.data):
        raise ValueError("Reply must have a consistent canonical AT wire frame")
    if (frame.kind != kind or frame.source != motor_id
            or frame.destination != HOST_ID):
        raise ValueError("Reply type/source/destination does not match selected motor / host FD")


def decode_type2(frame, *, motor_id=MOTOR_ID):
    """Decode this declared RS05 profile; never infer or repair position wraps."""
    _reply(frame, 2, motor_id)
    if frame.data[:3] == b"\x00\xc4\x56":
        raise ValueError("Version-shaped Type 2 reply is not position feedback")
    mode = (frame.can_id >> 22) & 3
    if mode == 3:
        raise ValueError("Reserved Type 2 mode state")
    p, v, torque, temp = struct.unpack(">4H", frame.data)
    return Type2Feedback(
        mode_state=mode, fault_bits=(frame.can_id >> 16) & 63,
        position_u16=p,
        protocol_position_rad=p * 25.14 / 65535.0 + POSITION_MIN,
        velocity_rad_s=v * 100.0 / 65535.0 - 50.0,
        torque_nm=torque * 11.0 / 65535.0 - 5.5,
        temperature_c=temp / 10.0)


def decode_type17_position(frame, *, motor_id=MOTOR_ID):
    """Preserve finite direct floats, including multi-turn values outside Type 2.

    Failure status or nonzero reserved bytes cannot yield a usable position.
    The original Frame remains available to the caller for raw failure logging.
    """
    _reply(frame, 17, motor_id)
    if frame.data[:4] != b"\x19\x70\x00\x00":
        raise ValueError("Expected position index 0x7019 and zero reserved bytes")
    if (frame.can_id >> 16) & 255:
        raise ValueError("Type 17 position read failed; payload is not a value")
    value = struct.unpack("<f", frame.data[4:8])[0]
    if not math.isfinite(value):
        raise ValueError("Type 17 position is nonfinite")
    return DirectPosition(value)
