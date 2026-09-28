"""File-only operator-attested motor-power epoch manifest for static review.

Two complete read-only Type0/17 captures must bracket operator-recorded 40 V
Off/On markers on the same Jetson monotonic clock. The operator reports an
external bus-voltage reading and physical pose evidence. This module cannot
sense the motor rail, verify a meter, verify STOP, or authorize motor output.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics

from .angle_branch_comparison import IDS, StaticBranchComparison


SCHEMA = "singularitydog.operator-power-epoch-manifest.v1"
EVENT_SCHEMA = "singularitydog.operator-power-events.v1"
CAPTURE_SCHEMA = "singularitydog.post-charge-box-pose-check.v1"
GENERIC_CAPTURE_SCHEMA = "singularitydog.readonly-12-angle-capture.v1"
PHASES = ("reference_on", "supply_off", "current_on")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _hash(value, label):
    _require(type(value) is str and len(value) == 64
             and all(ch in "0123456789abcdef" for ch in value), "Invalid " + label)
    return value


def _stamp(value, label):
    _require(type(value) is int and 0 < value < 2**63, "Invalid " + label)
    return value


def _finite(value, label):
    _require(type(value) in (int, float) and math.isfinite(value), "Invalid " + label)
    return float(value)


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, "Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("Nonfinite JSON constant: " + value)


def _decode(source, label):
    _require(type(source) is bytes, label + " source must be bytes")
    data = json.loads(source.decode("utf-8"), object_pairs_hook=_unique_pairs,
                      parse_constant=_reject_constant)
    _require(type(data) is dict, label + " must be a JSON object")
    return data, hashlib.sha256(source).hexdigest()


def _capture(source, label):
    data, digest = _decode(source, label)
    source_schema = data.get("schema")
    _require(source_schema in (CAPTURE_SCHEMA, GENERIC_CAPTURE_SCHEMA)
             and data.get("status") == "RECORDED_REVIEW_REQUIRED"
             and data.get("errors") == []
             and data.get("motor_output_allowed") is False
             and data.get("approved_for_runtime") is False
             and data.get("angle_wrap_applied") is False
             and data.get("stop_state") == "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
             label + " is not a complete unwrapped read-only capture")
    plan = data.get("plan")
    _require(type(plan) is dict and plan.get("allowed_can_types") == [0, 17]
             and plan.get("automatic_retry") is False
             and plan.get("motor_output_available") is False,
             label + " read-only plan differs")
    boot = data.get("boot_id")
    _require(type(boot) is str and boot.strip(), label + " boot ID missing")
    identities = data.get("identities")
    _require(type(identities) is dict and set(identities) == set(IDS),
             label + " needs twelve fresh Type0 identities")
    uids, uid_starts, uid_ends = {}, [], []
    for mid in IDS:
        row = identities[mid]
        _require(type(row) is dict, label + " ID" + mid + " identity malformed")
        uid = row.get("mcu_uid_hex")
        _require(type(uid) is str and len(uid) == 16
                 and all(ch in "0123456789abcdef" for ch in uid),
                 label + " ID" + mid + " UID malformed")
        start = _stamp(row.get("request_monotonic_ns"), "UID request")
        end = _stamp(row.get("reply_monotonic_ns"), "UID reply")
        _require(start < end, label + " UID reply precedes request")
        uids[mid] = uid
        uid_starts.append(start)
        uid_ends.append(end)
    _require(len(set(uids.values())) == 12, label + " UIDs are duplicated")
    telemetry = data.get("telemetry")
    _require(type(telemetry) is dict and type(telemetry.get("rows")) is dict
             and set(telemetry["rows"]) == set(IDS),
             label + " needs twelve Type17 telemetry rows")
    tele_start = _stamp(telemetry.get("started_monotonic_ns"), "telemetry start")
    tele_end = _stamp(telemetry.get("ended_monotonic_ns"), "telemetry end")
    _require(max(uid_ends) < tele_start < tele_end,
             label + " UID and telemetry chronology invalid")
    direct = data.get("direct_delta_by_id")
    if source_schema == CAPTURE_SCHEMA:
        _require(type(direct) is dict and set(direct) == set(IDS),
                 label + " needs twelve original direct-delta rows")
        baseline_sha256 = _hash(data.get("baseline_sha256"), "baseline hash")
    else:
        _require(direct is None and data.get("baseline_sha256") is None,
                 label + " generic capture must contain raw values without a baseline")
        baseline_sha256 = None
    raw, quiet, direct_delta = {}, {}, {}
    for mid in IDS:
        row = telemetry["rows"][mid]
        _require(type(row) is dict and type(row.get("run_mode")) is int
                 and row["run_mode"] == 0
                 and _finite(row.get("current"), "current") == 0.0
                 and 35.0 <= _finite(row.get("voltage"), "voltage") <= 43.0,
                 label + " ID" + mid + " not quiet or 40 V On")
        samples = row.get("position_samples")
        _require(type(samples) is list and len(samples) == 3,
                 label + " ID" + mid + " needs three positions")
        values = []
        for sample in samples:
            _require(type(sample) is dict, label + " position sample malformed")
            start = _stamp(sample.get("request_monotonic_ns"), "position request")
            end = _stamp(sample.get("reply_monotonic_ns"), "position reply")
            _require(tele_start <= start < end <= tele_end,
                     label + " ID" + mid + " position chronology invalid")
            values.append(_finite(sample.get("rad"), "position"))
        median = _finite(row.get("median_position_rad"), "median position")
        span = _finite(row.get("position_span_deg"), "position span")
        _require(abs(median-statistics.median(values)) <= 1e-9
                 and abs(span-math.degrees(max(values)-min(values))) <= 1e-6
                 and 0.0 <= span <= 0.1,
                 label + " ID" + mid + " position summary/stability invalid")
        raw[mid], quiet[mid] = median, True
        if direct is not None:
            delta = direct[mid]
            _require(type(delta) is dict
                     and abs(_finite(delta.get("direct_delta_deg"), "direct delta deg")
                             - math.degrees(_finite(delta.get("direct_delta_rad"),
                                                    "direct delta rad"))) <= 1e-8,
                     label + " ID" + mid + " direct delta summary invalid")
            direct_delta[mid] = delta["direct_delta_rad"]
    uid_digest = hashlib.sha256(json.dumps(identities, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    return {"boot_id": boot, "source_schema": source_schema,
            "baseline_sha256": baseline_sha256, "capture_sha256": digest,
            "uid_capture_sha256": uid_digest, "uids_by_id": uids,
            "raw_rad_by_id": raw, "direct_delta_rad_by_id": direct_delta,
            "disabled_zero_current_by_id": quiet,
            "first_uid_request_ns": min(uid_starts), "last_telemetry_reply_ns": tele_end}


def _events(source, reference, current):
    data, digest = _decode(source, "operator events")
    _require(data.get("schema") == EVENT_SCHEMA
             and data.get("motor_output_allowed") is False
             and data.get("clock_source") == "Jetson time.monotonic_ns"
             and data.get("boot_id") == reference["boot_id"] == current["boot_id"],
             "Operator events must use the capture Jetson boot and monotonic clock")
    _require(data.get("reference_capture_sha256") == reference["capture_sha256"]
             and data.get("current_capture_sha256") == current["capture_sha256"],
             "Operator event capture hash mismatch")
    rows = data.get("events")
    _require(type(rows) is list and len(rows) == 3
             and [row.get("phase") if type(row) is dict else None for row in rows] == list(PHASES),
             "Require reference On, physical Off, current On markers")
    times = []
    for phase, row in zip(PHASES, rows):
        time_ns = _stamp(row.get("observed_monotonic_ns"), phase + " time")
        voltage = _finite(row.get("operator_reported_external_bus_voltage_v"),
                          phase + " external voltage")
        if phase == "supply_off":
            _require(0.0 <= voltage <= 1.0, "Operator Off voltage is not near zero")
        else:
            _require(35.0 <= voltage <= 43.0, "Operator On voltage is outside 35..43 V")
        _hash(row.get("external_evidence_sha256"), phase + " external evidence hash")
        observation = row.get("physical_switch_observation")
        _require(type(observation) is str and observation.strip()
                 and row.get("operator_confirmed") is True,
                 phase + " needs operator physical observation")
        times.append(time_ns)
    _require(times[0] < reference["first_uid_request_ns"]
             and reference["last_telemetry_reply_ns"] < times[1]
             and times[1] < times[2] < current["first_uid_request_ns"],
             "Power event and capture chronology invalid")
    no_turn = data.get("no_full_physical_turn_by_id")
    observations = data.get("physical_pose_observation_by_id")
    pose_hashes = data.get("physical_pose_evidence_sha256_by_id")
    pose_intervals = data.get("physical_pose_observation_interval_ns_by_id")
    _require(type(no_turn) is dict and bool(no_turn)
             and set(no_turn).issubset(IDS)
             and all(no_turn[mid] is True for mid in no_turn)
             and type(observations) is dict and set(observations) == set(no_turn)
             and type(pose_hashes) is dict and set(pose_hashes) == set(no_turn)
             and type(pose_intervals) is dict and set(pose_intervals) == set(no_turn),
             "Missing per-axis no-full-turn evidence")
    for mid in no_turn:
        _require(type(observations[mid]) is str and observations[mid].strip(),
                 "Missing ID" + mid + " physical pose observation")
        _hash(pose_hashes[mid], "ID" + mid + " physical pose evidence hash")
        interval = pose_intervals[mid]
        _require(type(interval) is dict
                 and _stamp(interval.get("start_ns"), "ID" + mid + " pose start") <= times[1]
                 and _stamp(interval.get("end_ns"), "ID" + mid + " pose end") >= times[2],
                 "ID" + mid + " physical pose observation must span Off/On")
    _require(digest not in (reference["capture_sha256"], current["capture_sha256"],
                            reference["uid_capture_sha256"],
                            current["uid_capture_sha256"]),
             "Operator event provenance must be independent")
    return {"sha256": digest, "events": rows, "no_turn": no_turn,
            "observations": observations, "pose_evidence_sha256_by_id": pose_hashes,
            "pose_observation_interval_ns_by_id": pose_intervals}


def build_manifest(reference_source, current_source, operator_events_source):
    """Build a review artifact from three immutable private JSON byte strings."""
    reference = _capture(reference_source, "reference")
    current = _capture(current_source, "current")
    _require(reference["capture_sha256"] != current["capture_sha256"]
             and reference["uid_capture_sha256"] != current["uid_capture_sha256"]
             and reference["uids_by_id"] == current["uids_by_id"],
             "Captures or twelve UID assignments differ")
    if reference["baseline_sha256"] is not None and current["baseline_sha256"] is not None:
        _require(reference["baseline_sha256"] == current["baseline_sha256"],
                 "Post-charge captures use different baselines")
        _require(all(abs((current["direct_delta_rad_by_id"][mid]
                          - reference["direct_delta_rad_by_id"][mid])
                         - (current["raw_rad_by_id"][mid]
                            - reference["raw_rad_by_id"][mid])) <= 1e-8 for mid in IDS),
                 "Direct deltas disagree with raw angles under the common baseline")
    events = _events(operator_events_source, reference, current)
    epoch_id = lambda phase: "operator-attested:" + events["sha256"] + ":" + phase
    snapshots = []
    for capture, phase in ((reference, "reference_on"), (current, "current_on")):
        epoch = epoch_id(phase)
        snapshots.append({"boot_id": capture["boot_id"],
            "motor_power_epoch": epoch,
            "epoch_evidence_sha256": events["sha256"],
            "uid_read_boot_id": capture["boot_id"],
            "uid_read_motor_power_epoch": epoch,
            "capture_sha256": capture["capture_sha256"],
            "uid_capture_sha256": capture["uid_capture_sha256"],
            "monotonic_ns": capture["last_telemetry_reply_ns"],
            "uids_by_id": capture["uids_by_id"],
            "raw_rad_by_id": capture["raw_rad_by_id"],
            "disabled_zero_current_by_id": capture["disabled_zero_current_by_id"],
            "motor_output_allowed": False})
    proof = {"motor_supply_off_on_observed": True,
             "reference_capture_sha256": reference["capture_sha256"],
             "current_capture_sha256": current["capture_sha256"],
             "evidence_sha256": events["sha256"],
             "no_full_physical_turn_by_id": events["no_turn"],
             "physical_pose_observation_by_id": events["observations"]}
    comparison = StaticBranchComparison(*snapshots, proof).comparison()
    return {"schema": SCHEMA, "status": "OPERATOR_ATTESTED_REVIEW_REQUIRED",
            "boot_id": reference["boot_id"],
            "source_sha256": {"reference_capture": reference["capture_sha256"],
                              "current_capture": current["capture_sha256"],
                              "operator_events": events["sha256"]},
            "source_capture_schema": {"reference": reference["source_schema"],
                                      "current": current["source_schema"]},
            "operator_event_markers": events["events"],
            "physical_pose_evidence_sha256_by_id": events["pose_evidence_sha256_by_id"],
            "physical_pose_observation_interval_ns_by_id": (
                events["pose_observation_interval_ns_by_id"]),
            "reference_snapshot": snapshots[0], "current_snapshot": snapshots[1],
            "no_turn_evidence": proof, "branch_comparison": comparison,
            "external_voltage_software_verified": False,
            "motor_power_state_software_sensed": False,
            "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
            "motor_output_allowed": False, "approved_for_runtime": False}


def _private_output(path):
    requested = Path(path).expanduser()
    _require(not requested.is_symlink(), "Output path must not be a symlink")
    output = requested.resolve()
    _require(output.parent.is_dir() and not output.exists(),
             "Output parent must exist and output must be new")
    _require(not any((parent / ".git").exists()
                     for parent in (output.parent, *output.parents)),
             "Private UID and raw-angle manifest must be outside Git")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--current", type=Path, required=True)
    parser.add_argument("--operator-events", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = build_manifest(args.reference.read_bytes(), args.current.read_bytes(),
                                  args.operator_events.read_bytes())
        output = _private_output(args.output)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = os.open(output, flags, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
    except (OSError, ValueError, UnicodeError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(output), "status": manifest["status"],
                      "reviewed_motor_ids": sorted(
                          manifest["no_turn_evidence"]["no_full_physical_turn_by_id"], key=int),
                      "motor_output_allowed": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
