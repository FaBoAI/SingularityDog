"""Offline, default-deny commissioning record validator; no hardware is opened.

An empty blocker list means that the supplied record is complete and internally
consistent. It does not independently verify its evidence or authorize motion.
Evidence entries and IMU artifact paths are references, not files executed or
automatically trusted by this module. A responsible reviewer must check them.

Calibration convention: joint_rad = sign * motor_output_rad + offset_rad.
Measured ROM is expressed in that model-joint coordinate system. No unmeasured
physical limit, control gain, latency limit, or joint-role ordering is supplied.
Rotation tolerance is numerical validation only, not a physical accuracy bound.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


EXPECTED_LEGS = {"FR": [1, 2, 3], "FL": [4, 5, 6],
                 "RR": [7, 8, 9], "RL": [10, 11, 12]}
JOINT_ROLES = frozenset(("knee", "hip_pitch", "hip_roll"))
CALIBRATION_CONVENTION = "joint_rad = sign * motor_output_rad + offset_rad"
ROTATION_TOLERANCE = 1e-6


def _number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def _vector(value, size):
    return isinstance(value, list) and len(value) == size and all(_number(v) for v in value)


def validate_commissioning(document) -> tuple[str, ...]:
    """Return every detected blocker, including malformed or missing record data.

    Verification flags must be literal true and accompanied by evidence. Reported
    CAN timeout values never waive the separate per-motor communication-loss test.
    This function contains no arming, command encoding or device access.
    """
    blockers = []

    def block(path, explanation):
        blockers.append(path + ": " + explanation)

    def obj(value, path, required, optional=()):
        if not isinstance(value, dict):
            block(path, "must be an object")
            return {}
        missing = set(required) - value.keys()
        extra = value.keys() - set(required) - set(optional)
        if missing:
            block(path, "missing fields " + ", ".join(sorted(missing)))
        if extra:
            block(path, "unknown fields " + ", ".join(sorted(str(k) for k in extra)))
        return value

    def evidence(value, path):
        if not isinstance(value, list) or not value or not all(_text(v) for v in value):
            block(path, "nonempty source evidence references are required")

    def verified(value, path):
        if value.get("verified") is not True:
            block(path + ".verified", "verification is required")
        evidence(value.get("evidence"), path + ".evidence")

    document = obj(document, "record", (
        "schema_version", "motor_model", "id_order", "legs", "calibration_convention",
        "motors", "imu", "physical"), ("notes",))
    if type(document.get("schema_version")) is not int or document.get("schema_version") != 1:
        block("schema_version", "expected integer 1")
    if document.get("motor_model") != "RS05":
        block("motor_model", "this record requires the user-confirmed RS05 model")
    if document.get("id_order") != "foot_to_body":
        block("id_order", "expected foot_to_body; semantic joint roles require separate verification")
    if document.get("calibration_convention") != CALIBRATION_CONVENTION:
        block("calibration_convention", "unsupported or unspecified coordinate convention")
    legs = document.get("legs")
    if (not isinstance(legs, dict) or set(legs) != set(EXPECTED_LEGS) or
            any(not isinstance(legs.get(leg), list) or
                any(type(mid) is not int for mid in legs.get(leg, [])) or
                legs.get(leg) != ids for leg, ids in EXPECTED_LEGS.items())):
        block("legs", "expected FR[1,2,3], FL[4,5,6], RR[7,8,9], RL[10,11,12] in foot-to-body order")

    motors = document.get("motors")
    if not isinstance(motors, list):
        block("motors", "must be a list containing each ID 1..12 exactly once")
        motors = []
    ids, joint_names, roles = [], [], {}
    for index, entry in enumerate(motors):
        path = "motors[%d]" % index
        motor = obj(entry, path, ("id", "model", "joint", "calibration",
                                  "observed_can_timeout", "communication_loss_stop"))
        mid = motor.get("id")
        if type(mid) is not int or not 1 <= mid <= 12:
            block(path + ".id", "expected an integer motor ID 1..12")
        else:
            ids.append(mid)
        if motor.get("model") != "RS05":
            block(path + ".model", "expected RS05")

        joint = obj(motor.get("joint"), path + ".joint",
                    ("model_joint_name", "semantic_role", "verified", "evidence"))
        verified(joint, path + ".joint")
        name = joint.get("model_joint_name")
        if not _text(name) or name != name.strip():
            block(path + ".joint.model_joint_name", "a verified model joint name is required")
        else:
            joint_names.append(name)
        role = joint.get("semantic_role")
        if not isinstance(role, str) or role not in JOINT_ROLES:
            block(path + ".joint.semantic_role", "verify knee, hip_pitch or hip_roll; no order is inferred")
        elif type(mid) is int and 1 <= mid <= 12:
            roles[mid] = role

        calibration = obj(motor.get("calibration"), path + ".calibration",
                          ("verified", "offset_rad", "sign", "measured_rom_rad", "evidence"))
        verified(calibration, path + ".calibration")
        if not _number(calibration.get("offset_rad")):
            block(path + ".calibration.offset_rad", "a finite measured offset is required")
        sign = calibration.get("sign")
        if type(sign) is not int or sign not in (-1, 1):
            block(path + ".calibration.sign", "verified sign must be integer -1 or 1")
        rom = obj(calibration.get("measured_rom_rad"), path + ".calibration.measured_rom_rad", ("min", "max"))
        lower, upper = rom.get("min"), rom.get("max")
        if not _number(lower) or not _number(upper) or lower >= upper:
            block(path + ".calibration.measured_rom_rad", "finite measured min < max in joint coordinates is required")

        timeout = obj(motor.get("observed_can_timeout"), path + ".observed_can_timeout",
                      ("value_ticks", "read_status", "evidence"))
        evidence(timeout.get("evidence"), path + ".observed_can_timeout.evidence")
        ticks, status = timeout.get("value_ticks"), timeout.get("read_status")
        if type(status) is not int or not 0 <= status <= 255:
            block(path + ".observed_can_timeout.read_status", "record the actual Type17 status byte")
        elif status == 0:
            if type(ticks) is not int or not 0 <= ticks <= 0xFFFFFFFF:
                block(path + ".observed_can_timeout.value_ticks", "successful read requires uint32 ticks")
        elif ticks is not None:
            block(path + ".observed_can_timeout.value_ticks", "a rejected read must remain null/unknown")
        # Neither zero nor an unknown timeout is accepted as proof of stopping.
        stop = obj(motor.get("communication_loss_stop"), path + ".communication_loss_stop",
                   ("verified", "measured_stop_latency_s", "evidence"))
        verified(stop, path + ".communication_loss_stop")
        latency = stop.get("measured_stop_latency_s")
        if not _number(latency) or latency <= 0:
            block(path + ".communication_loss_stop.measured_stop_latency_s", "positive measured stop latency is required")

    if sorted(ids) != list(range(1, 13)):
        block("motors", "IDs 1..12 must be unique and complete")
    if len(joint_names) != len(set(joint_names)):
        block("motors.joint.model_joint_name", "model joint names must be unique")
    for leg, mids in EXPECTED_LEGS.items():
        if {roles.get(mid) for mid in mids} != JOINT_ROLES:
            block("legs." + leg, "three independently verified, distinct joint roles are required")

    imu = obj(document.get("imu"), "imu", ("rotation", "bias_and_scale"))
    rotation = obj(imu.get("rotation"), "imu.rotation", ("verified", "matrix_sensor_to_body", "artifact", "evidence"))
    verified(rotation, "imu.rotation")
    if not _text(rotation.get("artifact")):
        block("imu.rotation.artifact", "a reviewed mounting-rotation artifact is required")
    matrix = rotation.get("matrix_sensor_to_body")
    if not isinstance(matrix, list) or len(matrix) != 3 or not all(_vector(row, 3) for row in matrix):
        block("imu.rotation.matrix_sensor_to_body", "a finite 3x3 matrix is required")
    else:
        orthogonal = all(abs(sum(matrix[k][i] * matrix[k][j] for k in range(3)) - (i == j))
                         <= ROTATION_TOLERANCE for i in range(3) for j in range(3))
        a, b, c = matrix
        determinant = (a[0] * (b[1] * c[2] - b[2] * c[1]) -
                       a[1] * (b[0] * c[2] - b[2] * c[0]) +
                       a[2] * (b[0] * c[1] - b[1] * c[0]))
        if not orthogonal or not _number(determinant) or abs(determinant - 1) > ROTATION_TOLERANCE:
            block("imu.rotation.matrix_sensor_to_body", "must be orthonormal and right-handed (determinant +1)")
    bias = obj(imu.get("bias_and_scale"), "imu.bias_and_scale",
               ("verified", "accel_bias_m_s2", "accel_scale", "gyro_bias_rad_s", "artifact", "evidence"))
    verified(bias, "imu.bias_and_scale")
    if not _text(bias.get("artifact")):
        block("imu.bias_and_scale.artifact", "a reviewed calibration artifact is required")
    for name in ("accel_bias_m_s2", "gyro_bias_rad_s", "accel_scale"):
        value = bias.get(name)
        if not _vector(value, 3) or (name == "accel_scale" and any(v <= 0 for v in value)):
            block("imu.bias_and_scale." + name, "three finite measured values are required; scales must be positive")

    physical = obj(document.get("physical"), "physical", ("power_cut", "support"))
    for name in ("power_cut", "support"):
        item = obj(physical.get(name), "physical." + name, ("verified", "evidence"))
        verified(item, "physical." + name)
    return tuple(blockers)


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def load_commissioning(path):
    """Read strict JSON. Duplicate keys and non-JSON NaN/Infinity are rejected."""
    def invalid_constant(value):
        raise ValueError("non-JSON numeric constant: " + value)
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=_reject_duplicate_keys,
                      parse_constant=invalid_constant)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        blockers = validate_commissioning(load_commissioning(args.config))
    except (OSError, UnicodeError, ValueError, RecursionError) as error:
        blockers = ("record: could not validate configuration: " + str(error),)
    print(json.dumps({"blockers": blockers}, indent=2, allow_nan=False))
    return 1 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
