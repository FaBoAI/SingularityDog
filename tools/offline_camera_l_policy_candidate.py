"""Derive one unapproved policy input candidate from two pinned private files.

This is file-only. It embeds the reviewed ID3 comparison turn in a candidate
offset so that a no-output policy observer can consume current raw readings.
It does not verify physical calibration or construct any motor command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path


REVIEW_SHA256 = "d8238c120b3b305292bd6baf5579d6951061e95331b2a2ede03976b4e759d710"
CURRENT_SHA256 = "78810636c90d55f8eac8063c12f04c4867a0f52d7db7c7bd39fd042140a30d48"
MODEL_CAN_ORDER = [6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7]
IDS = tuple(range(1, 13))
TWO_PI = 2.0 * math.pi


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value),
            "Invalid " + label)
    return float(value)


def strict_json(source):
    def pairs(items):
        data = {}
        for key, value in items:
            require(key not in data, "Duplicate JSON key")
            data[key] = value
        return data

    def bad_constant(value):
        raise ValueError("Nonfinite JSON value: " + value)

    return json.loads(source, object_pairs_hook=pairs, parse_constant=bad_constant)


def read_pinned(path, expected_hash, label):
    raw = Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected_hash,
            label + " SHA-256 differs from the pinned source")
    return strict_json(raw)


def build_candidate(review, current, *, review_sha256, current_sha256):
    """Return a policy_shadow schema candidate with all runtime flags closed."""
    require(type(review) is dict
            and review.get("schema") == "singularitydog.camera-l-current-boot-range-review.v1"
            and review.get("status") == "REVIEW_ONLY_NO_RUNTIME_PROMOTION"
            and review.get("approved_for_runtime") is False
            and review.get("output_allowed") is False
            and review.get("motor_targets_generated") is False
            and review.get("raw_angles_modified") is False
            and review.get("operator_id3_no_full_turn_asserted") is True
            and review.get("model_range_screen_diagnostic_only_passed") is True
            and review.get("direct_out_of_range_ids") == [3]
            and review.get("conditional_out_of_range_ids") == []
            and review.get("cross_boot_angle_continuity_verified") is False
            and review.get("motor_supply_off_on_evidence_complete") is False,
            "Require the exact unapproved conditional range review")
    source = review.get("source_sha256")
    require(type(source) is dict and source.get("current") == current_sha256,
            "Review/current capture source hashes differ")
    require(type(review_sha256) is str and len(review_sha256) == 64
            and type(current_sha256) is str and len(current_sha256) == 64,
            "Missing source digests")
    statement = review.get("operator_statement")
    require(type(statement) is str and bool(statement.strip())
            and review.get("operator_statement_independently_verified") is False,
            "Missing bounded operator no-turn statement")
    branch = review.get("branch_subtraction_comparison_only")
    require(type(branch) is dict and branch.get("id") == 3
            and branch.get("turns") == 1
            and finite(branch.get("radians"), "branch radians") == TWO_PI,
            "Only reviewed ID3 minus one turn is accepted")
    require(type(current) is dict
            and current.get("status") == "RECORDED_REVIEW_REQUIRED"
            and current.get("errors") == []
            and current.get("angle_wrap_applied") is False
            and current.get("motor_output_allowed") is False
            and current.get("boot_id") == review.get("current_boot_id")
            and current.get("motor_power_epoch") ==
                review.get("current_motor_power_epoch_label"),
            "Current read-only capture does not match the review")
    rows = review.get("rows_by_id")
    telemetry = current.get("telemetry", {}).get("rows")
    identities = current.get("identities")
    expected_keys = {str(i) for i in IDS}
    require(type(rows) is dict and set(rows) == expected_keys
            and type(telemetry) is dict and set(telemetry) == expected_keys
            and type(identities) is dict and set(identities) == expected_keys,
            "Twelve reviewed, telemetry and UID rows are required")
    uid_map = {}
    candidates = []
    for mid in IDS:
        key = str(mid)
        row = rows[key]
        live = telemetry[key]
        identity = identities[key]
        require(type(row) is dict and type(live) is dict
                and type(identity) is dict and live.get("run_mode") == 0
                and live.get("current") == 0,
                f"ID{mid} does not have quiet read-only telemetry")
        uid = identity.get("mcu_uid_hex")
        require(type(uid) is str and len(uid) == 16
                and all(ch in "0123456789abcdef" for ch in uid),
                f"ID{mid} has no valid UID")
        uid_map[key] = uid
        raw = finite(live.get("median_position_rad"), f"ID{mid} raw")
        require(raw == finite(row.get("raw_current_unmodified_rad"),
                              f"ID{mid} reviewed raw"),
                f"ID{mid} raw angle differs from exact reviewed capture")
        turns = row.get("comparison_turns_subtracted")
        require(type(turns) is int and turns == (1 if mid == 3 else 0),
                f"ID{mid} has an unreviewed branch turn")
        camera_l = finite(row.get("raw_camera_l_rad"), f"ID{mid} camera L")
        nominal_l = finite(row.get("nominal_l_model_rad"), f"ID{mid} nominal L")
        require(nominal_l == (-math.pi / 2 if (mid - 1) % 3 == 0 else 0.0),
                f"ID{mid} nominal L differs")
        sign = row.get("sign_hypothesis")
        require(type(sign) is int and sign in (-1, 1),
                f"ID{mid} sign hypothesis invalid")
        offset = nominal_l - sign * (camera_l + turns * TWO_PI)
        q = sign * raw + offset
        require(math.isclose(math.degrees(q),
                             finite(row.get("conditional_model_deg"),
                                    f"ID{mid} conditional model angle"),
                             rel_tol=0.0, abs_tol=1e-8)
                and row.get("conditional_in_model_range") is True,
                f"ID{mid} model angle does not reproduce the reviewed screen")
        require(math.isclose(raw - turns * TWO_PI,
                             finite(row.get("raw_comparison_copy_rad"),
                                    f"ID{mid} comparison copy"),
                             rel_tol=0.0, abs_tol=1e-12),
                f"ID{mid} comparison copy differs")
        candidates.append({
            "motor_id": mid, "sign_candidate": sign,
            "offset_candidate_rad": offset,
            "reviewed_branch_turns_embedded_in_offset": turns,
            "physical_angle_accuracy_verified": False,
            "sign_revalidated_for_runtime": False,
            "approved_for_runtime": False,
        })
    require(len(set(uid_map.values())) == 12, "Motor UIDs are not distinct")
    return {
        "status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
        "candidate_subtype": "CROSS_BOOT_CAMERA_L_NO_OUTPUT_ONLY",
        "formula": "q_model = sign * raw + offset; rad; no wrapping",
        "offset_derivation":
            "nominal_L - sign * (camera_L_raw + reviewed_branch_turns * 2*pi)",
        "model_can_order_candidate": list(MODEL_CAN_ORDER),
        "identities": uid_map,
        "candidates": candidates,
        "source_sha256": {"range_review": review_sha256,
                          "current_capture": current_sha256,
                          "camera_l_by_leg": {leg: source[leg]
                                              for leg in ("FR", "FL", "RR", "RL")},
                          "historical_sign_candidates": source["historical_signs"]},
        "source_camera_l_boot_id": review["camera_l_boot_id"],
        "source_current_boot_id": current["boot_id"],
        "source_current_motor_power_epoch_label": current["motor_power_epoch"],
        "operator_id3_no_full_turn_statement": statement,
        "operator_statement_independently_verified": False,
        "id3_branch_adjustment_comparison_only": True,
        "raw_angles_modified": False,
        "cross_boot_angle_continuity_verified": False,
        "motor_supply_off_on_evidence_complete": False,
        "calibration_verified": False,
        "physical_joint_limits_verified": False,
        "live_50hz_verified": False,
        "motor_targets_generated": False,
        "approved_for_runtime": False,
        "motor_output_available": False,
        "output_allowed": False,
    }


def private_new_path(path):
    requested = Path(path).expanduser()
    require(not requested.is_symlink(), "Private output cannot be a symlink")
    output = requested.resolve()
    require(output.parent.is_dir() and not output.exists(),
            "Private output needs a fresh path in an existing directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Private calibration candidate must be outside Git")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review", type=Path, required=True)
    parser.add_argument("--current-r2", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        output = private_new_path(args.output)
        review = read_pinned(args.review, REVIEW_SHA256, "private range review")
        current = read_pinned(args.current_r2, CURRENT_SHA256, "current boot r2")
        candidate = build_candidate(review, current, review_sha256=REVIEW_SHA256,
                                    current_sha256=CURRENT_SHA256)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = os.open(output, flags, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(candidate, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(output), "status": candidate["status"],
                      "candidate_subtype": candidate["candidate_subtype"],
                      "approved_for_runtime": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
