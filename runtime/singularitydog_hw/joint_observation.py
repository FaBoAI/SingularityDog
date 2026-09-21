"""Compare existing diagnostic logs after an operator moved a joint.

This offline tool does not open hardware, send commands, infer a joint role/sign/
zero, or update commissioning records. Positions are output-shaft mechPos radians;
all changes are direct differences, with no modulo/angle wrapping.

Default heuristics: median change >=0.03rad is visible movement; within-capture
peak-to-peak >0.02rad suggests the pose was not settled; a direct change or sample
jump >=pi rad requires discontinuity/large-motion investigation. These are
comparison heuristics, not measured joint limits or proof of physical safety.
Even an unambiguous result is only a candidate requiring physical confirmation.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import statistics


IDS = tuple(range(1, 13))


class ObservationError(ValueError):
    """The supplied records cannot support a joint-observation comparison."""


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class ObservationLimits:
    movement_threshold_rad: float = 0.03
    settled_span_max_rad: float = 0.02
    large_delta_threshold_rad: float = math.pi

    def __post_init__(self):
        if any(not _finite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("Observation thresholds must be finite and positive")
        if self.large_delta_threshold_rad <= self.movement_threshold_rad:
            raise ValueError("Large-delta threshold must exceed movement threshold")


def _timestamp(record, field, description):
    value = record.get(field)
    if type(value) is not int or value <= 0:
        raise ObservationError(description + ": " + field + " must be a positive integer")
    return value


def _capture(records, label):
    if not isinstance(records, (list, tuple)) or not records:
        raise ObservationError(label + ": a nonempty sequence of diagnostic events is required")
    identities = {}
    positions = {mid: [] for mid in IDS}
    clocks = {"monotonic_ns": [], "wall_time_ns": []}
    last_parameter_time = {"monotonic_ns": None, "wall_time_ns": None}
    for index, record in enumerate(records):
        description = "%s event%d" % (label, index)
        if (not isinstance(record, dict) or not isinstance(record.get("kind"), str) or
                not record["kind"].strip()):
            raise ObservationError(description + ": malformed diagnostic event")
        kind = record["kind"]
        if (any(word in kind.lower() for word in ("error", "timeout", "fault")) or
                record.get("error") or record.get("errors") or
                ("ok" in record and record["ok"] is not True) or
                record.get("fault_bits", 0) != 0 or record.get("fault_detail_u32", 0) != 0):
            raise ObservationError(description + ": errors, faults or timeouts invalidate the capture")
        for field in clocks:
            if field in record:
                clocks[field].append(_timestamp(record, field, description))
        if kind != "motor_parameter":
            continue
        mid = record.get("motor_id")
        if type(mid) is not int or mid not in IDS or record.get("ok") is not True:
            raise ObservationError(description + ": invalid motor ID or unsuccessful parameter result")
        if ("status" in record and (type(record["status"]) is not int or record["status"] != 0) or
                "value" in record and not _finite(record["value"])):
            raise ObservationError(description + ": contradictory or nonfinite parameter result")
        # CAN results are serially produced even when IMU events are interleaved.
        for field in last_parameter_time:
            now = _timestamp(record, field, description)
            previous = last_parameter_time[field]
            if previous is not None and now <= previous:
                raise ObservationError(description + ": parameter timestamps must strictly increase")
            last_parameter_time[field] = now
        parameter = record.get("parameter")
        if parameter == "identity":
            uid = record.get("mcu_uid_hex")
            if (not isinstance(uid, str) or len(uid) != 16 or
                    any(c not in "0123456789abcdefABCDEF" for c in uid)):
                raise ObservationError(description + ": identity requires an eight-byte MCU UID")
            uid = uid.lower()
            if mid in identities and identities[mid] != uid:
                raise ObservationError(description + ": inconsistent identity within capture")
            identities[mid] = uid
        elif parameter == "position":
            if (record.get("unit") != "rad_output_shaft" or record.get("index") != 0x7019 or
                    type(record.get("status")) is not int or record["status"] != 0 or
                    not _finite(record.get("value"))):
                raise ObservationError(description + ": malformed/nonfinite mechPos result")
            positions[mid].append(float(record["value"]))
        elif not isinstance(parameter, str) or not parameter:
            raise ObservationError(description + ": missing parameter name")
    if set(identities) != set(IDS):
        raise ObservationError(label + ": identity coverage must include every ID1..12")
    if len(set(identities.values())) != len(IDS):
        raise ObservationError(label + ": MCU UIDs must be distinct across IDs")
    for mid, values in positions.items():
        if len(values) < 3:
            raise ObservationError("%s: ID%d requires at least three position values" % (label, mid))
    intervals = {field: {"start": min(values), "end": max(values)}
                 for field, values in clocks.items()}
    return identities, positions, intervals


def _statistics(values):
    try:
        result = {"samples": len(values), "mean_rad": statistics.fmean(values),
                  "median_rad": statistics.median(values), "min_rad": min(values),
                  "max_rad": max(values), "peak_to_peak_rad": max(values) - min(values),
                  "max_adjacent_step_rad": max(abs(b - a) for a, b in zip(values, values[1:]))}
    except (OverflowError, ValueError) as error:
        raise ObservationError("Position statistics overflow or are invalid") from error
    if not all(_finite(v) for v in result.values()):
        raise ObservationError("Position statistics must remain finite")
    return result


def compare_captures(before_records, after_records, expected_id, *, limits=None):
    """Validate two ordered captures and return a physically unconfirmed candidate.

    Current diagnose.py logs provide both clocks on parameter events. Both are
    required, and both capture intervals must be ordered and nonoverlapping.
    A missing/stepped-back clock fails closed. These fields alone cannot prove
    clock continuity across host reboots or fabricated source records.
    """
    if type(expected_id) is not int or expected_id not in IDS:
        raise ObservationError("expected_id must be an integer1..12")
    if limits is None:
        limits = ObservationLimits()
    if not isinstance(limits, ObservationLimits):
        raise ObservationError("limits must be validated ObservationLimits")
    before_uid, before, before_time = _capture(before_records, "before")
    after_uid, after, after_time = _capture(after_records, "after")
    if before_uid != after_uid:
        raise ObservationError("Motor ID to MCU UID mapping changed between captures")
    if any(before_time[field]["end"] >= after_time[field]["start"] for field in before_time):
        raise ObservationError("Before and after capture intervals overlap or are reversed")
    motors, changed, unsettled, large = {}, [], [], []
    for mid in IDS:
        pre, post = _statistics(before[mid]), _statistics(after[mid])
        median_delta = post["median_rad"] - pre["median_rad"]
        mean_delta = post["mean_rad"] - pre["mean_rad"]
        if not _finite(median_delta) or not _finite(mean_delta):
            raise ObservationError("Derived position changes must remain finite")
        is_changed = abs(median_delta) >= limits.movement_threshold_rad
        is_unsettled = max(pre["peak_to_peak_rad"], post["peak_to_peak_rad"]) > limits.settled_span_max_rad
        is_large = max(abs(median_delta), pre["max_adjacent_step_rad"],
                       post["max_adjacent_step_rad"]) >= limits.large_delta_threshold_rad
        if is_changed:
            changed.append(mid)
        if is_unsettled:
            unsettled.append(mid)
        if is_large:
            large.append(mid)
        motors[str(mid)] = {"mcu_uid_hex": before_uid[mid], "before": pre, "after": post,
                            "median_change_rad": median_delta, "mean_change_rad": mean_delta,
                            "change_exceeds_heuristic": is_changed,
                            "unsettled_capture": is_unsettled,
                            "large_raw_change_or_discontinuity": is_large}
    other_changed = [mid for mid in changed if mid != expected_id]
    issues = []
    if expected_id not in changed:
        issues.append("Expected ID has no median change above the comparison threshold")
    if other_changed:
        issues.append("Other motor IDs also changed: " + ",".join(map(str, other_changed)))
    if unsettled:
        issues.append("Within-capture motion/noise exceeds settled-span heuristic: " + ",".join(map(str, unsettled)))
    if large:
        issues.append("Large raw changes/discontinuities require investigation; no wrapping applied: " + ",".join(map(str, large)))
    classification = ("expected_change_candidate" if not issues else
                      "no_change" if not changed and not unsettled and not large else "ambiguous")
    return {"schema_version": 1, "status": "candidate", "classification": classification,
            "expected_motor_id": expected_id, "changed_motor_ids": changed,
            "other_changed_motor_ids": other_changed, "issues": issues,
            "motors": motors, "capture_intervals": {"before": before_time, "after": after_time},
            "heuristics": asdict(limits), "heuristics_are_mechanical_limits": False,
            "requires_physical_confirmation": True, "approved_for_runtime": False,
            "semantic_joint_role_inferred": False, "direction_sign_inferred": False,
            "zero_inferred": False, "angle_wrapping_applied": False,
            "commissioning_modified": False,
            "limitations": ["The expected motor ID is an operator hypothesis, not a verified joint mapping.",
                            "Both captures must describe settled poses; nearby changes can remain below thresholds.",
                            "A large delta may be real multi-turn motion or a discontinuity; this tool cannot decide.",
                            "Clock fields and source records are supplied evidence, not independent physical proof."]}


def _strict_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ObservationError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def _load(path):
    path = Path(path)
    content = path.read_bytes()
    records = []
    def bad_constant(value):
        raise ObservationError("Non-JSON numeric value: " + value)
    try:
        for line in content.decode("utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line, object_pairs_hook=_strict_pairs,
                                          parse_constant=bad_constant))
    except (UnicodeError, ValueError, RecursionError) as error:
        raise ObservationError("Malformed JSONL in " + str(path) + ": " + str(error)) from error
    return records, {"path": str(path.resolve()), "sha256": hashlib.sha256(content).hexdigest(),
                     "bytes": len(content)}


def compare_jsonl_files(before, after, expected_id, output, *, limits=None):
    """Create a new0600 candidate JSON outside Git, without modifying input logs."""
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("Output already exists: " + str(output))
    resolved = output.resolve()
    if any((p / ".git").exists() for p in (resolved, *resolved.parents)):
        raise ObservationError("UID-containing reports must be outside a Git checkout")
    before_records, before_source = _load(before)
    after_records, after_source = _load(after)
    if before_source["sha256"] == after_source["sha256"]:
        raise ObservationError("Before and after are identical captures")
    report = compare_captures(before_records, after_records, expected_id, limits=limits)
    report["sources"] = {"before": before_source, "after": after_source}
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(encoded)
    except BaseException:
        output.unlink(missing_ok=True)
        raise
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--expected-id", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = compare_jsonl_files(args.before, args.after, args.expected_id, args.output)
    except (OSError, ObservationError, ValueError) as error:
        parser.exit(2, "Observation rejected: " + str(error) + "\n")
    print(json.dumps({"output": str(args.output.resolve()), "status": report["status"],
                      "classification": report["classification"], "issues": report["issues"],
                      "requires_physical_confirmation": True}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
