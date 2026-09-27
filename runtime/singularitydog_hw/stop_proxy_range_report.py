"""Offline angle and fixed-policy-range report for saved integrated STOP-proxy logs.

Run from ``runtime``::

    python -B -m singularitydog_hw.stop_proxy_range_report \
        --front-jsonl /path/to/front.jsonl --rear-jsonl /path/to/rear.jsonl \
        --calibration /path/to/calibration.json

This module reads three regular files and prints JSON. It has no transport,
device discovery, motor output, or output-file option. Cycle 1 STOP feedback is
the saved input observation; later STOP-proxy replies are never substituted.
The calibration is an unverified candidate. Angles are neither wrapped nor
clipped, and an in-range row is not a motion or standing authorization.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import stat

from . import policy_shadow


SCHEMA = "singularitydog.offline-stop-proxy-range-report.v1"
# Pin the reviewed order and limits. A policy change requires an explicit review
# of this diagnostic instead of silently moving the comparison thresholds.
MODEL_ORDER = (6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7)
LOWER = (-0.5, -0.9, -2.2) * 4
UPPER = (0.5, 1.2, -0.08) * 4
EXPECTED_BUS = {mid: "front" if mid <= 6 else "rear" for mid in range(1, 13)}
EXPECTED_LABELS = {
    mid: (leg, joint)
    for leg, ids in (("FL", (6, 5, 4)), ("FR", (3, 2, 1)),
                     ("RL", (12, 11, 10)), ("RR", (9, 8, 7)))
    for mid, joint in zip(ids, ("hip", "thigh", "calf"))
}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _regular_bytes(path):
    path = Path(path)
    _require(path.is_file() and stat.S_ISREG(path.stat().st_mode),
             "Input must be a saved regular file: " + str(path))
    return path.read_bytes()


def _json(text):
    return policy_shadow._json(text)


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


def _degrees(value):
    if value is None:
        return None
    converted = math.degrees(value)
    _require(_finite(converted), "Angle cannot be represented in degrees")
    return converted


def _calibration(raw):
    data = _json(raw.decode("utf-8"))
    _require(type(data) is dict, "Calibration must be a JSON object")
    _require(type(data.get("candidates")) is list
             and all(type(row) is dict for row in data["candidates"]),
             "Calibration candidates must be an array of objects")
    candidates = policy_shadow.validate_calibration(data)
    _require(data.get("approved_for_runtime") is False,
             "Calibration must remain unapproved for runtime")
    for mid in range(1, 13):
        row = candidates[mid]
        leg, joint = EXPECTED_LABELS[mid]
        _require(row.get("leg") == leg and row.get("joint") == joint,
                 "Calibration leg/joint does not match fixed model order for ID" + str(mid))
        _require(row.get("approved_for_runtime") is False,
                 "Calibration row must remain unapproved for runtime")
    return candidates


def _selected_rows(raw, bus):
    rows = []
    for line_number, line in enumerate(raw.decode("utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = _json(line)
        _require(type(row) is dict, f"{bus} JSONL line {line_number} is not an object")
        if (row.get("kind") == "pipeline_reply" and type(row.get("cycle")) is int
                and row["cycle"] == 1 and row.get("parameter") == "stop_feedback"):
            rows.append((bus, line_number, row))
    return rows


def build_report(front_jsonl, rear_jsonl, calibration):
    """Read saved files only; return a diagnostic report without UID values."""
    _require((tuple(policy_shadow.CAN_ORDER), tuple(policy_shadow.LOWER),
              tuple(policy_shadow.UPPER)) == (MODEL_ORDER, LOWER, UPPER),
             "Reviewed fixed order or policy limits differ from policy_shadow")
    front_raw = _regular_bytes(front_jsonl)
    rear_raw = _regular_bytes(rear_jsonl)
    calibration_raw = _regular_bytes(calibration)
    candidates = _calibration(calibration_raw)
    selected = _selected_rows(front_raw, "front") + _selected_rows(rear_raw, "rear")
    by_id = {mid: [] for mid in range(1, 13)}
    input_blockers = []
    for bus, line, row in selected:
        mid = row.get("motor_id")
        if type(mid) is not int or mid not in by_id:
            input_blockers.append({"bus": bus, "line": line,
                                   "reason": "invalid_selected_motor_id"})
        else:
            by_id[mid].append((bus, line, row))

    joints = []
    for index, mid in enumerate(MODEL_ORDER):
        candidate = candidates[mid]
        leg, joint = EXPECTED_LABELS[mid]
        lower, upper = LOWER[index], UPPER[index]
        blockers = []
        raw_angle = calibrated = None
        source = by_id[mid]
        if not source:
            blockers.append("missing_cycle_1_stop_feedback")
        elif len(source) != 1:
            blockers.append("duplicate_cycle_1_stop_feedback")
        else:
            bus, _line, row = source[0]
            if bus != EXPECTED_BUS[mid]:
                blockers.append("wrong_bus_for_motor_id")
            result = row.get("result")
            if (row.get("ok") is not True or type(result) is not dict
                    or result.get("ok") is not True):
                blockers.append("unsuccessful_or_missing_decoded_result")
            elif (type(result.get("motor_id")) is not int or result["motor_id"] != mid
                  or result.get("parameter") != "stop_feedback"):
                blockers.append("decoded_result_id_or_parameter_mismatch")
            if not blockers:
                value = result.get("position_rad_candidate")
                if not _finite(value):
                    blockers.append("invalid_raw_position")
                else:
                    raw_angle = value
                    calibrated = candidate["sign_candidate"] * value + candidate["offset_candidate_rad"]
                    if not _finite(calibrated):
                        calibrated = None
                        blockers.append("invalid_calibrated_angle")
                    if abs(value) > 12.57:
                        blockers.append("raw_position_outside_declared_type2_range")
                    if calibrated is not None:
                        if calibrated < lower:
                            blockers.append("below_fixed_policy_lower_limit")
                        elif calibrated > upper:
                            blockers.append("above_fixed_policy_upper_limit")

        joints.append({
            "model_index": index, "motor_id": mid, "bus": EXPECTED_BUS[mid],
            "leg": leg, "joint": joint, "source_rows": len(source),
            "source_locations": [{"bus": bus, "line": line}
                                 for bus, line, _row in source],
            "raw_angle_rad": raw_angle,
            "raw_angle_deg": _degrees(raw_angle),
            "sign_candidate": candidate["sign_candidate"],
            "offset_candidate_rad": candidate["offset_candidate_rad"],
            "calibrated_angle_rad": calibrated,
            "calibrated_angle_deg": _degrees(calibrated),
            "fixed_policy_lower_rad": lower, "fixed_policy_upper_rad": upper,
            "fixed_policy_lower_deg": _degrees(lower),
            "fixed_policy_upper_deg": _degrees(upper),
            "within_fixed_policy_limits": (lower <= calibrated <= upper
                                           if calibrated is not None else None),
            "blockers": blockers,
        })

    blocked_ids = [row["motor_id"] for row in joints if row["blockers"]]
    range_screen_passed = not blocked_ids and not input_blockers
    return {
        "schema": SCHEMA,
        "status": ("OFFLINE_RANGE_WITHIN_LIMITS_UNVERIFIED" if range_screen_passed
                   else "OFFLINE_RANGE_BLOCKED"),
        "range_screen_passed": range_screen_passed,
        "selected_cycle": 1, "selected_parameter": "stop_feedback",
        "formula": "q_model = sign * raw + offset; rad; no wrapping",
        "model_can_order": list(MODEL_ORDER),
        "fixed_policy_limits_rad": {
            "hip": [-0.5, 0.5], "thigh": [-0.9, 1.2], "calf": [-2.2, -0.08],
        },
        "joints": joints, "blocked_motor_ids": blocked_ids,
        "input_blockers": input_blockers,
        "global_blockers": [
            "calibration_candidate_unverified",
            "physical_angle_accuracy_unverified",
            "fresh_uid_match_not_checked_from_these_inputs",
            "integrated_capture_completion_not_checked_from_jsonl_only",
            "decoded_position_not_independently_checked_against_wire",
            "angle_turn_branch_not_verified_from_single_pose",
            "saved_observation_not_live_readiness",
        ],
        "sources_sha256": {
            "front_jsonl": hashlib.sha256(front_raw).hexdigest(),
            "rear_jsonl": hashlib.sha256(rear_raw).hexdigest(),
            "calibration": hashlib.sha256(calibration_raw).hexdigest(),
        },
        "calibration_verified": False, "physical_angle_accuracy_verified": False,
        "fresh_identity_match_verified": False, "angle_wrapping_applied": False,
        "clipping_applied": False, "output_allowed": False,
        "approved_for_runtime": False, "motor_command_available": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--front-jsonl", required=True, type=Path)
    parser.add_argument("--rear-jsonl", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = build_report(args.front_jsonl, args.rear_jsonl, args.calibration)
    except (OSError, UnicodeError, ValueError, TypeError, KeyError, OverflowError) as error:
        parser.exit(2, "Offline range report rejected: " + str(error) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
