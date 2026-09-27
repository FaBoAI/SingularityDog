"""Validate a read-only twelve-axis capture and prepare a private stance record.

Without a separate physical-pose review, the result remains a blocked draft.
Even with that review it is only an input to the disabled package builder: this
tool has no CAN access, motor output, route authorization, or live trial path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "runtime"))

from singularitydog_hw.can_timing_probe import validate_uids
from singularitydog_hw.fixed_stance_readonly_capture import (
    IDS_BY_BUS, MAX_ABS_VELOCITY_RAD_S, MAX_CAPTURE_S, PARAMETERS,
    SCHEMA as READONLY_SCHEMA, STABILITY_SPAN_RAD, draft_manifest,
)
from singularitydog_hw.rs05_trial_protocol import POSITION_MAX, POSITION_MIN


IDS = tuple(range(1, 13))
KEYS = {str(mid) for mid in IDS}
DRAFT_SCHEMA = "singularitydog.fixed-stance-capture-draft.v1"
CAPTURE_SCHEMA = "singularitydog.supported-fixed-stance-capture.v2"
PHYSICAL_REVIEW_SCHEMA = "singularitydog.fixed-stance-physical-pose-review.v2"
FLOOR_STANCE_CLASS = "four_foot_floor_supported_stance"
UNVERIFIED_STOP = "UNVERIFIED_BY_READ_ONLY_PROTOCOL"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def is_digest(value):
    return (type(value) is str and len(value) == 64
            and all(char in "0123456789abcdef" for char in value))


def pairs_unique(rows):
    result = {}
    for key, value in rows:
        require(key not in result, "Duplicate JSON key: " + key)
        result[key] = value
    return result


def read_json(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "Missing or symlinked input: " + str(path))
    raw = path.read_bytes()
    value = json.loads(raw, object_pairs_hook=pairs_unique,
                       parse_constant=lambda word: (_ for _ in ()).throw(
                           ValueError("Nonfinite JSON token: " + word)))
    json.dumps(value, allow_nan=False)  # Also rejects overflowing JSON floats.
    require(type(value) is dict, "Input root must be a JSON object: " + str(path))
    return value, digest(raw)


def finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), label + " is not finite")
    return float(value)


def integer(value, label):
    require(type(value) is int and value > 0, label + " must be a positive integer")
    return value


def raw_map(value, label):
    require(type(value) is dict and set(value) == KEYS,
            label + " must contain exactly IDs 1..12")
    result = {}
    for mid in KEYS:
        number = finite(value[mid], f"{label} ID{mid}")
        require(POSITION_MIN <= number <= POSITION_MAX,
                f"{label} ID{mid} is outside RS05 raw position range")
        result[mid] = number
    return result


def verified_capture(summary, draft, expected_uids, *, summary_sha,
                     expected_uids_sha):
    """Recompute every recorded pose statistic before making a blocked record."""
    require(summary.get("schema") == READONLY_SCHEMA
            and summary.get("status") == "RECORDED_REVIEW_REQUIRED"
            and summary.get("errors") == []
            and summary.get("output_allowed") is False
            and summary.get("approved_for_runtime") is False
            and summary.get("stop_state") == UNVERIFIED_STOP,
            "Read-only summary is incomplete or makes an output/STOP claim")
    boot = summary.get("boot_id")
    require(type(boot) is str and bool(boot.strip()), "Capture boot ID missing")
    require(summary.get("expected_uids_sha256") == expected_uids_sha,
            "Expected UID file differs from the capture session")
    source_hashes = summary.get("source_sha256")
    require(type(source_hashes) is dict
            and set(source_hashes) == {"fixed_stance_readonly_capture.py", "can_readonly.py"}
            and all(is_digest(value) for value in source_hashes.values()),
            "Read-only source hash record missing")
    for name, recorded_hash in source_hashes.items():
        trusted_path = ROOT / "runtime" / "singularitydog_hw" / name
        require(recorded_hash == digest(trusted_path.read_bytes()),
                "Capture source differs from trusted local copy: " + name)
    plan = summary.get("plan")
    require(type(plan) is dict
            and plan.get("ids_by_bus") == {bus: list(ids) for bus, ids in IDS_BY_BUS.items()}
            and plan.get("allowed_can_types") == [0, 17]
            and plan.get("parameters") == ["identity", *PARAMETERS]
            and plan.get("motor_output_available") is False
            and plan.get("output_allowed") is False
            and plan.get("stop_command_available") is False
            and plan.get("stop_state_verifiable") is False,
            "Capture plan is not the twelve-axis read-only plan")
    sweeps = plan.get("sweeps")
    require(type(sweeps) is int and 3 <= sweeps <= 5, "Capture sweep count invalid")

    enter = integer(summary.get("operator_enter_monotonic_ns"), "Operator Enter timestamp")
    identities = summary.get("identities")
    require(type(identities) is dict and set(identities) == KEYS,
            "Capture requires twelve identity rows")
    uids = {str(mid): expected_uids[mid] for mid in IDS}
    for mid in IDS:
        row = identities[str(mid)]
        require(type(row) is dict and row.get("mcu_uid_hex") == uids[str(mid)],
                f"ID{mid} does not match expected UID")
        requested = integer(row.get("request_monotonic_ns"), f"ID{mid} UID request")
        received = integer(row.get("reply_monotonic_ns"), f"ID{mid} UID reply")
        require(requested < received < enter, f"ID{mid} UID reply is not before Enter")

    pose = summary.get("pose")
    require(type(pose) is dict and pose.get("sampling_issues") == []
            and pose.get("sampling_stability_heuristic_passed") is True
            and pose.get("stationarity_verified") is False,
            "Pose sampling is incomplete, unstable, or overclaims stationarity")
    started = integer(pose.get("started_monotonic_ns"), "Pose start")
    ended = integer(pose.get("ended_monotonic_ns"), "Pose end")
    require(enter <= started < ended and ended - started <= MAX_CAPTURE_S * 1e9,
            "Pose timestamps exceed the capture window")
    span_ms = finite(pose.get("capture_span_ms"), "Capture span")
    require(math.isclose(span_ms, (ended - started) / 1e6, rel_tol=0., abs_tol=1e-6),
            "Reported capture span differs from timestamps")
    samples = pose.get("samples")
    require(type(samples) is dict and set(samples) == KEYS,
            "Pose requires twelve sample groups")
    raw = raw_map(pose.get("raw_rad_by_id"), "Pose raw positions")
    for mid in IDS:
        rows = samples[str(mid)]
        require(type(rows) is list and len(rows) == sweeps,
                f"ID{mid} has an incomplete sample group")
        positions, velocities = [], []
        previous_reply = 0
        for index, sample in enumerate(rows):
            require(type(sample) is dict and set(sample) == set(PARAMETERS),
                    f"ID{mid} sample {index} has missing parameters")
            for parameter in PARAMETERS:
                item = sample[parameter]
                require(type(item) is dict and set(item) == {
                    "value", "request_monotonic_ns", "reply_monotonic_ns"},
                    f"ID{mid} {parameter} sample is malformed")
                requested = integer(item["request_monotonic_ns"], f"ID{mid} {parameter} request")
                received = integer(item["reply_monotonic_ns"], f"ID{mid} {parameter} reply")
                require(started <= requested < received <= ended and requested > previous_reply,
                        f"ID{mid} {parameter} reply is stale or out of sequence")
                previous_reply = received
                value = finite(item["value"], f"ID{mid} {parameter}")
                if parameter == "position":
                    require(POSITION_MIN <= value <= POSITION_MAX,
                            f"ID{mid} sampled raw angle outside RS05 range")
                    positions.append(value)
                elif parameter == "velocity":
                    velocities.append(abs(value))
                elif parameter == "run_mode":
                    require(value == 0., f"ID{mid} was not in disabled run mode")
        require(max(positions) - min(positions) <= STABILITY_SPAN_RAD
                and max(velocities) <= MAX_ABS_VELOCITY_RAD_S,
                f"ID{mid} sampled pose is unstable")
        require(math.isclose(raw[str(mid)], statistics.median(positions),
                             rel_tol=0., abs_tol=1e-10),
                f"ID{mid} raw position differs from sampled median")

    require(draft.get("schema") == DRAFT_SCHEMA
            and draft.get("boot_id") == boot
            and draft.get("motor_uids") == uids
            and raw_map(draft.get("raw_rad_by_id"), "Draft raw positions") == raw
            and draft.get("stance_capture_sha256") == summary_sha
            and draft == draft_manifest(summary, summary_sha),
            "Capture draft differs from the exact read-only summary")
    return boot, uids, raw, span_ms


def validate_physical_review(review, *, boot, uids, raw, summary_sha, draft_sha):
    keys = {"schema", "boot_id", "motor_uids", "raw_rad_by_id",
            "readonly_summary_sha256", "readonly_draft_sha256",
            "supported_pose_placed_by_operator", "simultaneous_physical_stance_verified",
            "stand_removed", "pose_class", "foot_support_kind",
            "foot_support_height_cm", "operator_note", "evidence_reference"}
    require(type(review) is dict and set(review) == keys
            and review["schema"] == PHYSICAL_REVIEW_SCHEMA
            and review["boot_id"] == boot
            and review["motor_uids"] == uids
            and raw_map(review["raw_rad_by_id"], "Physical review raw positions") == raw
            and review["readonly_summary_sha256"] == summary_sha
            and review["readonly_draft_sha256"] == draft_sha
            and review["supported_pose_placed_by_operator"] is True
            and review["simultaneous_physical_stance_verified"] is True
            and review["stand_removed"] is False
            and review["pose_class"] == FLOOR_STANCE_CLASS
            and review["foot_support_kind"] == "floor"
            and type(review["foot_support_height_cm"]) in (int, float)
            and review["foot_support_height_cm"] == 0
            and type(review["operator_note"]) is str and bool(review["operator_note"].strip())
            and type(review["evidence_reference"]) is str
            and bool(review["evidence_reference"].strip()),
            "Separate physical pose review is incomplete or differs from read-only evidence")


def review_template(boot, uids, raw, summary_sha, draft_sha):
    """Make a separate evidence-linked form without asserting a physical fact."""
    return {
        "schema": PHYSICAL_REVIEW_SCHEMA,
        "boot_id": boot,
        "motor_uids": uids,
        "raw_rad_by_id": raw,
        "readonly_summary_sha256": summary_sha,
        "readonly_draft_sha256": draft_sha,
        "supported_pose_placed_by_operator": False,
        "simultaneous_physical_stance_verified": False,
        "stand_removed": False,
        "pose_class": None,
        "foot_support_kind": None,
        "foot_support_height_cm": None,
        "operator_note": "",
        "evidence_reference": "",
    }


def fresh_private_output(path):
    raw_path = Path(path).expanduser()
    output = raw_path.resolve()
    require(not output.exists() and not raw_path.is_symlink() and output.parent.is_dir(),
            "Use a fresh output file in an existing directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Private UID and pose output must remain outside Git")
    return output


def write_private_json(path, value):
    data = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(data)
    return digest(data)


def promote(summary_path, draft_path, expected_uids_path, output_path,
            *, physical_review_path=None, review_template_path=None):
    output = fresh_private_output(output_path)
    template_output = (fresh_private_output(review_template_path)
                       if review_template_path is not None else None)
    require(template_output is None or template_output != output,
            "Capture and review-template outputs must differ")
    require(not (physical_review_path is not None and template_output is not None),
            "Generate a review template before supplying a completed physical review")
    summary, summary_sha = read_json(summary_path)
    draft, draft_sha = read_json(draft_path)
    expected, expected_sha = read_json(expected_uids_path)
    expected_uids = validate_uids(expected)
    boot, uids, raw, span_ms = verified_capture(
        summary, draft, expected_uids, summary_sha=summary_sha,
        expected_uids_sha=expected_sha)
    physical_review_sha = None
    physical_review = None
    if physical_review_path is not None:
        physical_review, physical_review_sha = read_json(physical_review_path)
        validate_physical_review(physical_review, boot=boot, uids=uids, raw=raw,
                                 summary_sha=summary_sha, draft_sha=draft_sha)
    reviewed = physical_review is not None
    result = {
        "schema": CAPTURE_SCHEMA,
        "boot_id": boot,
        "motor_uids": uids,
        "raw_rad_by_id": raw,
        "read_only": True,
        "motor_enable_sent": False,
        "supported_pose_placed_by_operator": reviewed,
        "simultaneous_physical_stance_verified": reviewed,
        "stand_removed": False,
        "pose_class": physical_review["pose_class"] if reviewed else None,
        "foot_support_kind": physical_review["foot_support_kind"] if reviewed else None,
        "foot_support_height_cm": physical_review["foot_support_height_cm"] if reviewed else None,
        "operator_note": physical_review["operator_note"] if reviewed else "",
        "evidence_reference": physical_review["evidence_reference"] if reviewed else "",
        "readonly_summary_sha256": summary_sha,
        "readonly_draft_sha256": draft_sha,
        "expected_uids_sha256": expected_sha,
        "physical_pose_review_sha256": physical_review_sha,
        "capture_span_ms": span_ms,
        "sampling_stability_heuristic_passed": True,
        "stop_state": UNVERIFIED_STOP,
        "same_boot_12_axis_hold_passed": False,
        "all_segment_sweeps_physically_reviewed": False,
        "front_upper_leg_carbon_clamp_clearance_verified": False,
        "output_allowed": False,
        "approved_for_runtime": False,
        "learned_policy_allowed": False,
        "self_supported_standing_verified": False,
    }
    if template_output is not None:
        write_private_json(template_output,
                           review_template(boot, uids, raw, summary_sha, draft_sha))
    capture_sha = write_private_json(output, result)
    return {"status": "PHYSICAL_POSE_REVIEWED_DISABLED_ONLY" if reviewed
            else "READONLY_CAPTURE_VALIDATED_REVIEW_REQUIRED",
            "output": str(output), "boot_id": boot, "output_allowed": False,
            "physical_pose_reviewed": reviewed, "capture_sha256": capture_sha,
            "review_template": str(template_output) if template_output is not None else None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--capture-draft", type=Path, required=True)
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--physical-review", type=Path)
    parser.add_argument("--review-template", type=Path,
                        help="Write a separate, false-by-default physical pose review form")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(promote(args.summary, args.capture_draft, args.expected_uids,
                             args.output, physical_review_path=args.physical_review,
                             review_template_path=args.review_template), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
