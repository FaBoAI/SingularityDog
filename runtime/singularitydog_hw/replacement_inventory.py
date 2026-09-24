"""Validate replacement inventory from saved joint snapshots, entirely offline.

Example: python3 -m singularitydog_hw.replacement_inventory --config /private/plan.json
    --output /private/new-inventory

Strict config (absolute capture paths; lowercase SHA256 of BOTH source files):
  {"schema_version": 1, "expected_changed_ids": [6, 9, 10],
   "previous": {"capture": "/private/before", "events_sha256": "...",
                "summary_sha256": "..."},
   "current": {"capture": "/private/after", "events_sha256": "...",
               "summary_sha256": "..."}}

Each capture must be a complete, error-free joint_snapshot (3..60 sweeps).
Wall-clock intervals must be ordered; monotonic clocks may reset after reboot.
Exactly the declared IDs may change, all other identities (including ID11 when
not selected) must stay unchanged, and retired identities may not move to any ID.
New/current position spread and adjacent jumps <=.02rad and current <=.05A are
short observation gates, not calibration, motor-off, durability or motion proof.
Velocity and voltage are reported but are not replacement-identity gates.

Writes private inventory.json and, ONLY on validation success, expected-uids.json
(plain ID1..12 -> UID mapping for later fresh identity comparison, not permission).
No overwrites, no --force, no calibration copying, device or network access.
Exit 0 means candidate inventory validated; blocked inventory or bad inputs exit 2.
Malformed inputs/hash mismatch create no output. Valid captures with inventory or
quality failures produce a blocked report without expected-uids.json. Never mix
old zeros or the old RR overlay with replacement hardware.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re

from .joint_observation import _strict_pairs
from .pose_record import load_snapshot


IDS = tuple(range(1, 13))
QUALITY_LIMITS = {"minimum_samples": 3, "maximum_position_span_rad": .02,
                  "maximum_adjacent_step_rad": .02, "maximum_abs_current_A": .05}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _reject_constant(value):
    raise ValueError("Invalid JSON constant: " + value)


def _config(path):
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    data = json.loads(raw, object_pairs_hook=_strict_pairs, parse_constant=_reject_constant)
    _require(isinstance(data, dict) and set(data) == {
        "schema_version", "expected_changed_ids", "previous", "current"}, "Invalid config fields")
    _require(type(data["schema_version"]) is int and data["schema_version"] == 1,
             "Require schema_version 1")
    selected = data["expected_changed_ids"]
    _require(isinstance(selected, list) and selected and
             all(type(i) is int and i in IDS for i in selected) and
             selected == sorted(set(selected)), "Expected changed IDs must be unique, sorted integers 1..12")
    for label in ("previous", "current"):
        source = data[label]
        _require(isinstance(source, dict) and set(source) == {
            "capture", "events_sha256", "summary_sha256"}, "Invalid capture pin fields: " + label)
        _require(isinstance(source["capture"], str) and Path(source["capture"]).is_absolute(),
                 "Capture paths must be absolute")
        for name in ("events_sha256", "summary_sha256"):
            _require(isinstance(source[name], str) and re.fullmatch(r"[0-9a-f]{64}", source[name]),
                     "Require lowercase SHA256 pins")
    return data, {"path": str(path), "sha256": _sha(raw)}


def _load_pinned(source):
    directory = Path(source["capture"]).resolve()
    # Check pins before parsing, then verify the exact bytes used by the strict
    # loader. Changed files cannot silently acquire an unpinned provenance.
    for name in ("events", "summary"):
        suffix = ".jsonl" if name == "events" else ".json"
        _require(_sha((directory / (name + suffix)).read_bytes()) == source[name + "_sha256"],
                 "Capture source hash mismatch: " + name)
    result = load_snapshot(directory)
    _require(set(result["motors"]) == {str(i) for i in IDS}, "Require all twelve motors")
    for name in ("events", "summary"):
        _require(result["sources"][name]["sha256"] == source[name + "_sha256"],
                 "Capture changed during validation: " + name)
    identities = {str(i): result["motors"][str(i)]["mcu_uid_hex"] for i in IDS}
    _require(all(isinstance(v, str) and re.fullmatch(r"[0-9a-f]{16}", v)
                 for v in identities.values()) and len(set(identities.values())) == 12,
             "Require twelve distinct valid identities")
    return result, identities


def build_inventory(config_path):
    """Read pinned files and return a private candidate-only report. No devices."""
    config, config_source = _config(config_path)
    previous, old = _load_pinned(config["previous"])
    current, new = _load_pinned(config["current"])
    expected = config["expected_changed_ids"]
    changed = [i for i in IDS if old[str(i)] != new[str(i)]]
    retired = {old[str(i)] for i in expected}
    retired_present = [i for i in IDS if new[str(i)] in retired]
    old_end = previous["capture_intervals"]["wall_time_ns"]["end"]
    new_start = current["capture_intervals"]["wall_time_ns"]["start"]
    _require(type(old_end) is int and type(new_start) is int and old_end > 0 and new_start > 0,
             "Invalid capture wall-clock intervals")
    gates = {"exact_expected_ids_changed": changed == expected,
             "other_identities_unchanged": all(old[str(i)] == new[str(i)] for i in IDS if i not in expected),
             "retired_identities_absent_everywhere": not retired_present,
             "current_capture_after_previous": old_end < new_start}
    blockers = [{"gate": name} for name, passed in gates.items() if not passed]
    quality, mapping = [], []
    for mid in IDS:
        motor = current["motors"][str(mid)]
        stats = motor["position"]
        samples = stats["samples"]
        span, adjacent, amps = stats["peak_to_peak_rad"], stats["max_adjacent_step_rad"], motor["max_abs_current_A"]
        _require(type(samples) is int and samples >= QUALITY_LIMITS["minimum_samples"] and
                 all(_finite(v) and v >= 0 for v in (span, adjacent, amps)),
                 "Missing or malformed current stability/current evidence")
        qgates = {"position_span": span <= QUALITY_LIMITS["maximum_position_span_rad"],
                  "adjacent_step": adjacent <= QUALITY_LIMITS["maximum_adjacent_step_rad"],
                  "current": amps <= QUALITY_LIMITS["maximum_abs_current_A"]}
        for name, passed in qgates.items():
            if not passed:
                blockers.append({"motor_id": mid, "gate": name})
        quality.append({"motor_id": mid, "samples": samples, "position_span_rad": span,
                        "maximum_adjacent_step_rad": adjacent, "maximum_abs_current_A": amps,
                        "maximum_abs_velocity_rad_s": motor["max_abs_velocity_rad_s"],
                        "voltage_minmax_V": motor["voltage_minmax_V"],
                        "sampling_issues": motor["sampling_issues"], "gates": qgates})
        mapping.append({"motor_id": mid, "previous_uid": old[str(mid)], "current_uid": new[str(mid)],
                        "expected_replaced": mid in expected, "identity_changed": mid in changed,
                        "calibration_copied": False})
    validated = not blockers
    expected_bytes = _json_bytes(new)
    return {"schema": "singularitydog.replacement-inventory.v1",
            "status": "CANDIDATE_INVENTORY_VALIDATED" if validated else "BLOCKED_REVIEW_REQUIRED",
            "inventory_candidate_validated": validated,
            "expected_changed_ids": expected, "observed_changed_ids": changed,
            "required_recalibration_ids": changed,
            "unchanged_ids": [i for i in IDS if i not in changed],
            "retired_uid_present_at_ids": retired_present,
            "identity_mapping": mapping, "identity_and_chronology_gates": gates,
            "current_observation_quality": quality, "quality_limits": dict(QUALITY_LIMITS),
            "blockers": blockers,
            "expected_uids_export": {"available": validated,
                "filename": "expected-uids.json" if validated else None,
                "sha256": _sha(expected_bytes) if validated else None,
                "purpose": "Comparison data for future fresh identity checks; no motor permission"},
            "sources": {"config": config_source, "previous": previous["sources"],
                        "current": current["sources"], "checker_sha256": _sha(Path(__file__).read_bytes())},
            "wall_time_ns": {"previous_end": old_end, "current_start": new_start},
            "limitations": ["Saved captures only; no live identity or motor-off verification.",
                "Short stability observations do not certify hardware health or calibration.",
                "Monotonic clocks and raw shaft angles are not compared across replacement/reboot.",
                "Velocity and voltage remain reported observations, not motion authorization.",
                "Decoded-event validation does not independently redecode received CAN wire bytes.",
                "Retired identities here are the replaced IDs from the pinned previous capture; older history is not inferred."],
            "calibration_copied": False, "prior_rr_overlay_inherited": False,
            "zero_verified": False, "sign_verified": False,
            "approved_for_runtime": False, "output_allowed": False, "motor_output_available": False,
            "live_readiness": False, "motor_enable_state_verified": False,
            "motor_power_cycle_continuity_verified": False, "received_wire_redecoded": False}


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")


def save_inventory(config_path, output):
    """Publish a new private report; publish UID comparison data only if valid."""
    output = Path(output).expanduser()
    _require(not output.is_symlink(), "Output must not be a symlink")
    output = output.resolve()
    if output.exists():
        raise FileExistsError("Inventory output already exists")
    _require(not any((p / ".git").exists() for p in (output, *output.parents)),
             "Private inventory must be outside Git")
    report = build_inventory(config_path)
    # Serialize both before creating any output. No old calibration fields enter
    # the plain UID map; downstream code must still require fresh identity reads.
    report_bytes = _json_bytes(report)
    uid_bytes = _json_bytes({str(r["motor_id"]): r["current_uid"] for r in report["identity_mapping"]})
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    for name, data in (("inventory.json", report_bytes), ("expected-uids.json", uid_bytes)):
        if name == "expected-uids.json" and not report["inventory_candidate_validated"]:
            continue
        fd = os.open(output / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = save_inventory(args.config, args.output)
    except (ValueError, OSError, TypeError, KeyError, OverflowError) as error:
        parser.exit(2, "Replacement inventory rejected: " + str(error) + "\n")
    print(json.dumps({"status": report["status"], "expected_changed_ids": report["expected_changed_ids"],
                      "observed_changed_ids": report["observed_changed_ids"], "blockers": report["blockers"],
                      "expected_uids_exported": report["inventory_candidate_validated"],
                      "output_allowed": False, "approved_for_runtime": False}, indent=2))
    return 0 if report["inventory_candidate_validated"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
