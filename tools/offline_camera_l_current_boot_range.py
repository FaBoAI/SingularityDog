"""File-only, review-only D17 joint-range screen for the 2026-09-27 captures.

The saved motor angles are never changed. ID3 has a separate comparison copy
with one turn subtracted only when the operator explicitly attests that its
physical output shaft made no full turn since the camera-L recording. This is
not a calibration, a motor target, or permission to operate the robot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path


IDS_BY_LEG = {"FR": (1, 2, 3), "FL": (4, 5, 6),
              "RR": (7, 8, 9), "RL": (10, 11, 12)}
IDS = tuple(range(1, 13))
MODEL_CAN_ORDER = [6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7]
CAMERA_HASHES = {
    "FR": "de367f55f3776d04f4ad2acb65014a950ebe1b99e9c630c8bee202891c8cd949",
    "FL": "3f5c96335005ff283902c2ee58aa0e8a314523f75ec5b19e4a6c6cdaf574e611",
    "RR": "979e6239cddc8e5027097c19f7fb7f80b8563ac904ea7d85c64bed3c08676999",
    "RL": "937c865cb9954a361fbd815b6d5508b155349ff38b84a985af90ddf1ba55781e",
}
CURRENT_HASH = "78810636c90d55f8eac8063c12f04c4867a0f52d7db7c7bd39fd042140a30d48"
HISTORICAL_SIGNS_HASH = "68d139e3bf6ec329fd941b80bdc711f20c4280ca065859daf2d8a9ffc4051371"
TWO_PI = 2.0 * math.pi
ID3_COMPARISON_RESIDUAL_LIMIT_DEG = 10.0


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value, name):
    require(type(value) in (int, float) and math.isfinite(value), f"Invalid {name}")
    return float(value)


def strict_json(source):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "Duplicate JSON key")
            result[key] = value
        return result

    def nonfinite(value):
        raise ValueError("Nonfinite JSON value: " + value)

    return json.loads(source, object_pairs_hook=pairs, parse_constant=nonfinite)


def read_pinned(path, expected_hash, name):
    source = Path(path).read_bytes()
    require(hashlib.sha256(source).hexdigest() == expected_hash,
            f"{name} SHA-256 differs from the pinned review source")
    return strict_json(source)


def limits(mid):
    kind = (mid - 1) % 3
    return ((-2.2, -0.08) if kind == 0 else
            (-0.9, 1.2) if kind == 1 else (-0.5, 0.5))


def model_angle(mid, raw, camera_l_raw, sign):
    nominal_l = -math.pi / 2 if (mid - 1) % 3 == 0 else 0.0
    return nominal_l + sign * (raw - camera_l_raw)


def build_review(camera_l, current, historical, *, operator_no_turn_id3,
                 operator_statement, source_hashes):
    """Calculate two model-angle screens; never modify any caller object."""
    require(operator_no_turn_id3 is True and type(operator_statement) is str
            and bool(operator_statement.strip()),
            "Explicit operator ID3 no-full-turn statement is required")
    require(type(source_hashes) is dict and set(source_hashes) ==
            {"FR", "FL", "RR", "RL", "current", "historical_signs"},
            "Complete source hashes required")
    require(type(camera_l) is dict and set(camera_l) == set(IDS_BY_LEG),
            "Four camera-L records required")
    require(type(current) is dict
            and current.get("status") == "RECORDED_REVIEW_REQUIRED"
            and current.get("errors") == []
            and current.get("motor_output_allowed") is False
            and current.get("angle_wrap_applied") is False,
            "Current capture must be an unwrapped, error-free read-only record")
    require(type(historical) is dict
            and historical.get("status") == "MANUAL_NOMINAL_CANDIDATES_ONLY"
            and historical.get("formula") == "q_model = sign * raw + offset; rad; no wrapping"
            and historical.get("model_can_order_candidate") == MODEL_CAN_ORDER,
            "Historical sign candidate has an incompatible schema")
    boot = current.get("boot_id")
    require(type(boot) is str and bool(boot.strip()), "Current boot missing")
    old_boots = {data.get("boot_id") for data in camera_l.values() if type(data) is dict}
    require(len(old_boots) == 1 and all(type(b) is str and b for b in old_boots)
            and boot not in old_boots,
            "Expected pinned camera-L records from one earlier Jetson boot")
    current_rows = current.get("telemetry", {}).get("rows")
    current_ids = current.get("identities")
    require(type(current_rows) is dict and set(current_rows) == {str(i) for i in IDS}
            and type(current_ids) is dict and set(current_ids) == {str(i) for i in IDS},
            "Current capture lacks twelve telemetry and UID rows")
    old_ids = historical.get("identities")
    old_candidates = historical.get("candidates")
    require(type(old_ids) is dict and set(old_ids) == {str(i) for i in IDS}
            and type(old_candidates) is list and len(old_candidates) == 12,
            "Historical sign candidate lacks twelve axes")
    signs = {}
    for row in old_candidates:
        require(type(row) is dict and type(row.get("motor_id")) is int
                and row["motor_id"] in IDS and row["motor_id"] not in signs
                and type(row.get("sign_candidate")) is int
                and row["sign_candidate"] in (-1, 1),
                "Invalid historical sign candidate")
        signs[row["motor_id"]] = row["sign_candidate"]
    require(set(signs) == set(IDS) and signs[10] == 1,
            "Expected twelve historical signs including ID10 +1")
    signs[10] = -1  # Review hypothesis only, supported by the isolated ID10 A/B direction.

    camera_raw = {}
    camera_uids = {}
    for leg, ids in IDS_BY_LEG.items():
        data = camera_l[leg]
        require(data.get("status") == f"READ_ONLY_{leg}_CAMERA_L"
                and data.get("motor_output_allowed") is False
                and data.get("boot_id") in old_boots,
                f"{leg} is not the expected read-only camera-L record")
        rows = data.get("rows")
        require(type(rows) is dict and set(rows) == {str(i) for i in ids},
                f"{leg} camera-L rows are incomplete")
        for mid in ids:
            key = str(mid)
            row = rows[key]
            require(type(row) is dict and row.get("run_mode") == 0
                    and row.get("current_A") == 0
                    and 0 <= number(row.get("position_span_deg"), "camera span") <= 0.1,
                    f"{leg} ID{mid} was not quiet and stationary")
            camera_raw[mid] = number(row.get("position_median_rad"), "camera raw")
            camera_uids[mid] = row.get("uid")
    require(len(set(camera_uids.values())) == 12 and all(
        type(camera_uids[mid]) is str and len(camera_uids[mid]) == 16
        and camera_uids[mid] == old_ids[str(mid)]
        and camera_uids[mid] == current_ids[str(mid)].get("mcu_uid_hex")
        for mid in IDS), "Camera, historical and current UIDs differ")

    rows = {}
    for mid in IDS:
        key = str(mid)
        row = current_rows[key]
        require(type(row) is dict and row.get("run_mode") == 0
                and row.get("current") == 0
                and 0 <= number(row.get("position_span_deg"), "current span") <= 0.1,
                f"Current ID{mid} was not quiet and stationary")
        raw = number(row.get("median_position_rad"), "current raw")
        direct_delta_deg = math.degrees(raw - camera_raw[mid])
        if mid == 3:
            require(abs(direct_delta_deg - 360.0) <= ID3_COMPARISON_RESIDUAL_LIMIT_DEG,
                    "ID3 is not the reviewed +one-turn raw comparison candidate")
        comparison = raw - (TWO_PI if mid == 3 else 0.0)
        direct_q = model_angle(mid, raw, camera_raw[mid], signs[mid])
        conditional_q = model_angle(mid, comparison, camera_raw[mid], signs[mid])
        lower, upper = limits(mid)
        rows[key] = {
            "raw_camera_l_rad": camera_raw[mid],
            "raw_current_unmodified_rad": raw,
            "raw_direct_delta_deg": direct_delta_deg,
            "comparison_turns_subtracted": 1 if mid == 3 else 0,
            "raw_comparison_copy_rad": comparison,
            "sign_hypothesis": signs[mid],
            "nominal_l_model_rad": -math.pi / 2 if (mid - 1) % 3 == 0 else 0.0,
            "direct_model_deg": math.degrees(direct_q),
            "conditional_model_deg": math.degrees(conditional_q),
            "model_limit_deg": [math.degrees(lower), math.degrees(upper)],
            "conditional_lower_margin_deg": math.degrees(conditional_q - lower),
            "conditional_upper_margin_deg": math.degrees(upper - conditional_q),
            "direct_in_model_range": lower <= direct_q <= upper,
            "conditional_in_model_range": lower <= conditional_q <= upper,
        }
    return {
        "schema": "singularitydog.camera-l-current-boot-range-review.v1",
        "status": "REVIEW_ONLY_NO_RUNTIME_PROMOTION",
        "source_sha256": dict(source_hashes),
        "camera_l_boot_id": next(iter(old_boots)),
        "current_boot_id": boot,
        "current_motor_power_epoch_label": current.get("motor_power_epoch"),
        "matched_uid_count": 12,
        "operator_id3_no_full_turn_asserted": True,
        "operator_statement": operator_statement.strip(),
        "operator_statement_independently_verified": False,
        "motor_supply_off_on_evidence_complete": False,
        "cross_boot_angle_continuity_verified": False,
        "camera_l_physical_angle_accuracy_verified": False,
        "all_joint_signs_physically_verified": False,
        "id10_sign_reversal_verified_for_runtime": False,
        "sequential_camera_l_not_simultaneous_fullbody_pose": True,
        "id3_comparison_residual_limit_deg_diagnostic_only": ID3_COMPARISON_RESIDUAL_LIMIT_DEG,
        "raw_angles_modified": False,
        "branch_subtraction_comparison_only": {"id": 3, "turns": 1,
                                                "radians": TWO_PI},
        "rows_by_id": rows,
        "direct_out_of_range_ids": [mid for mid in IDS
                                    if not rows[str(mid)]["direct_in_model_range"]],
        "conditional_out_of_range_ids": [mid for mid in IDS
                                         if not rows[str(mid)]["conditional_in_model_range"]],
        "model_range_screen_diagnostic_only_passed": all(
            rows[str(mid)]["conditional_in_model_range"] for mid in IDS),
        "calibration_verified": False,
        "approved_for_runtime": False,
        "motor_output_available": False,
        "output_allowed": False,
        "motor_targets_generated": False,
    }


def private_new_path(path):
    requested = Path(path).expanduser()
    require(not requested.is_symlink(), "Private output cannot be a symlink")
    output = requested.resolve()
    require(output.parent.is_dir() and not output.exists(),
            "Private output needs a fresh path in an existing directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Private angle output must be outside Git")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for leg in IDS_BY_LEG:
        parser.add_argument(f"--{leg.lower()}-camera-l", type=Path, required=True)
    parser.add_argument("--current-r2", type=Path, required=True)
    parser.add_argument("--historical-signs", type=Path, required=True)
    parser.add_argument("--operator-id3-no-full-turn", action="store_true")
    parser.add_argument("--operator-statement", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        output = private_new_path(args.output)
        paths = {leg: getattr(args, f"{leg.lower()}_camera_l") for leg in IDS_BY_LEG}
        camera = {leg: read_pinned(paths[leg], CAMERA_HASHES[leg], leg + " camera L")
                  for leg in IDS_BY_LEG}
        current = read_pinned(args.current_r2, CURRENT_HASH, "current boot r2")
        historical = read_pinned(args.historical_signs, HISTORICAL_SIGNS_HASH,
                                 "historical sign candidates")
        review = build_review(camera, current, historical,
                              operator_no_turn_id3=args.operator_id3_no_full_turn,
                              operator_statement=args.operator_statement,
                              source_hashes={**CAMERA_HASHES, "current": CURRENT_HASH,
                                             "historical_signs": HISTORICAL_SIGNS_HASH})
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = os.open(output, flags, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(review, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(output), "status": review["status"],
                      "direct_out_of_range_ids": review["direct_out_of_range_ids"],
                      "conditional_out_of_range_ids": review["conditional_out_of_range_ids"],
                      "approved_for_runtime": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
