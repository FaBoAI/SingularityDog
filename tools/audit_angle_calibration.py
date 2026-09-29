#!/usr/bin/env python3
"""File-only twelve-axis zero/sign/periodic-branch audit and reference fit.

Modes: replay the saved September 27 evidence, or apply an explicit calibration
profile to a new motor_epoch_readonly_capture JSON.  No serial, CAN, SSH or motor
control module is imported.  A profile is never installed into runtime settings.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from singularitydog_hw.angle_calibration_audit import (  # noqa: E402
    IDS, TAU, UNKNOWN_EPOCHS, MODEL_CAN_ORDER, AxisCalibration, audit_twelve_axes,
    fit_reference_observations,
)


PROFILE_SCHEMA = "singularitydog.angle-calibration-review-profile.v1"
CURRENT_NAME = "RO-motor-epoch-current-boot-20260927-r2.json"
CANDIDATE_NAME = "RO-policy-candidate-current-boot-20260927-r1.json"
IDS_BY_LEG = {"FR": (1, 2, 3), "FL": (4, 5, 6), "RR": (7, 8, 9), "RL": (10, 11, 12)}


def need(condition, message):
    if not condition:
        raise ValueError(message)


def strict_json(source):
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, "Duplicate JSON key")
            result[key] = value
        return result

    def constant(value):
        raise ValueError("Nonfinite JSON constant: " + value)

    return json.loads(source, object_pairs_hook=pairs, parse_constant=constant)


def read(path, expected_sha=None):
    raw = Path(path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    need(expected_sha is None or digest == expected_sha, "Source SHA-256 mismatch: " + str(path))
    data = strict_json(raw)
    need(type(data) is dict, "JSON object required")
    return data, digest


def current_values(data):
    need(data.get("status") == "RECORDED_REVIEW_REQUIRED" and data.get("errors") == []
         and data.get("motor_output_allowed") is False and data.get("angle_wrap_applied") is False,
         "An error-free unwrapped no-output current capture is required")
    need(data.get("plan", {}).get("allowed_can_types") == [0, 17],
         "Only the Type0/17 current capture schema is accepted")
    identities, rows = data.get("identities"), data.get("telemetry", {}).get("rows")
    keys = {str(mid) for mid in IDS}
    need(type(identities) is dict and set(identities) == keys
         and type(rows) is dict and set(rows) == keys, "Current capture needs twelve axes")
    raw, uids = {}, {}
    for key in keys:
        row = rows[key]
        need(row.get("run_mode") == 0 and row.get("current") == 0,
             "Current capture not observed quiet: ID" + key)
        span = row.get("position_span_deg")
        need(type(span) in (int, float) and math.isfinite(span) and 0 <= span <= .1,
             "Current raw samples not static: ID" + key)
        raw[key], uids[key] = row.get("median_position_rad"), identities[key].get("mcu_uid_hex")
    return raw, uids


def history_profile(root):
    """Import existing hypotheses without converting old false flags to approval."""
    root = Path(root)
    logs = root if root.name == "singularitydog-logs" else root / "singularitydog-logs"
    candidate, candidate_hash = read(logs / CANDIDATE_NAME)
    need(candidate.get("status") == "MANUAL_NOMINAL_CANDIDATES_ONLY"
         and candidate.get("candidate_subtype") == "CROSS_BOOT_CAMERA_L_NO_OUTPUT_ONLY"
         and candidate.get("approved_for_runtime") is False
         and candidate.get("calibration_verified") is False,
         "Expected the unapproved September 27 policy candidate")
    current, current_hash = read(logs / CURRENT_NAME, candidate["source_sha256"]["current_capture"])
    raw, uids = current_values(current)
    need(candidate.get("identities") == uids, "Candidate/current UIDs differ")
    by_id = {row["motor_id"]: row for row in candidate["candidates"]}
    need(len(candidate["candidates"]) == 12 and set(by_id) == set(IDS), "Candidate must have twelve unique IDs")
    historical, historical_hash = read(
        logs.parent / "singularitydog-tests/fr-toward-stance-r11/stance-reference/calibration-candidates.json",
        candidate["source_sha256"]["historical_sign_candidates"])
    need(historical.get("identities") == uids, "Historical direction UIDs differ")
    old_signs = {row["motor_id"]: row for row in historical["candidates"]}
    need(len(historical["candidates"]) == 12 and set(old_signs) == set(IDS),
         "Historical direction list is incomplete")
    direction_support = {}
    for mid in IDS:
        evidence = old_signs[mid]["retained_sign_source"]
        observed, nominal = evidence["observed_delta_deg"], evidence["nominal_delta_deg"]
        need(type(observed) in (int, float) and math.isfinite(observed)
             and type(nominal) in (int, float) and math.isfinite(nominal)
             and observed != 0 and nominal != 0, "Invalid historical direction values")
        inferred = 1 if observed * nominal > 0 else -1
        need(inferred == old_signs[mid]["sign_candidate"], "Historical sign algebra mismatch")
        direction_support[str(mid)] = {
            "historical_observed_delta_deg": observed,
            "historical_nominal_delta_deg": nominal,
            "historical_sign_candidate": inferred,
            "matches_current_hypothesis": inferred == by_id[mid]["sign_candidate"],
            "current_physical_revalidation": False,
            "source_sha256": historical_hash}
    camera_raw, camera_hashes = {}, {}
    for leg, ids in IDS_BY_LEG.items():
        pose, digest = read(logs / f"RO-{leg.lower()}-camera-L-20260927.json",
                            candidate["source_sha256"]["camera_l_by_leg"][leg])
        need(pose.get("motor_output_allowed") is False and set(pose.get("rows", {})) == {str(mid) for mid in ids},
             "Incomplete camera-L source")
        camera_hashes[leg] = digest
        for mid in ids:
            row = pose["rows"][str(mid)]
            need(row.get("uid") == uids[str(mid)] and row.get("run_mode") == 0
                 and row.get("current_A") == 0, "Camera-L UID or quiet-state mismatch")
            camera_raw[mid] = row["position_median_rad"]
    axes = []
    for mid in IDS:
        old = by_id[mid]
        sign = old["sign_candidate"]
        need(type(sign) is int and sign in (-1, 1), "Invalid candidate sign")
        nominal = -math.pi / 2 if (mid - 1) % 3 == 0 else 0.
        offset = nominal - sign * camera_raw[mid]
        # The old policy-only candidate embedded the ID3 turn into its offset.
        # Recover the reference calibration algebra without changing any file.
        embedded = old["reviewed_branch_turns_embedded_in_offset"]
        need(type(embedded) is int and embedded == (1 if mid == 3 else 0)
             and math.isclose(old["offset_candidate_rad"] + sign * embedded * TAU,
                              offset, rel_tol=0, abs_tol=1e-10),
             "Old candidate's branch/offset does not match its camera source")
        lower, upper = ((-2.2, -.08) if (mid - 1) % 3 == 0 else
                        (-.9, 1.2) if (mid - 1) % 3 == 1 else (-.5, .5))
        axes.append({"motor_id": mid, "uid": uids[str(mid)], "sign": sign,
                     "offset_rad": offset, "lower_rad": lower, "upper_rad": upper,
                     "uncertainty_rad": 0., "zero_reviewed": False,
                     "direction_reviewed": False, "physical_limits_reviewed": False,
                     "zero_evidence_sha256": None, "direction_evidence_sha256": None,
                     "physical_limits_evidence_sha256": None})
    profile = {"schema": PROFILE_SCHEMA, "axes": axes, "evidence_files": {},
               "assembly_revision": "UNREVIEWED_CAMERA_L_20260927",
               "physical_uncertainty_known": False,
               "uncertainty_note": "0 is diagnostic arithmetic only; physical camera-L uncertainty is UNKNOWN",
               "limits_note": "D17 model limits are an arithmetic screen, not verified mechanical limits",
               "source_sha256": {"candidate": candidate_hash, "current": current_hash,
                                  "camera_l_by_leg": camera_hashes,
                                  "historical_direction": historical_hash},
               "historical_direction_support_by_id": direction_support,
               "approved_for_runtime": False, "motor_output_available": False}
    digest = hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode()).hexdigest()
    contracts = {row["motor_id"]: AxisCalibration(**row, calibration_sha256=digest) for row in axes}
    return profile, contracts, current, raw, uids


def load_profile(path):
    profile, digest = read(path)
    need(profile.get("schema") == PROFILE_SCHEMA
         and profile.get("approved_for_runtime") is False
         and profile.get("motor_output_available") is False, "Unapproved review profile required")
    need(type(profile.get("assembly_revision")) is str and profile["assembly_revision"].strip(),
         "Explicit assembly revision required")
    rows = profile.get("axes")
    need(type(rows) is list and len(rows) == 12, "Twelve axis profiles required")
    verified_files = profile.get("evidence_files", {})
    need(type(verified_files) is dict, "Evidence file map required")
    contracts = {}
    for row in rows:
        need(type(row) is dict and "calibration_sha256" not in row,
             "Calibration hash is computed from this profile, not supplied by a row")
        contract = AxisCalibration(**row, calibration_sha256=digest)
        need(contract.motor_id not in contracts, "Duplicate profile motor ID")
        for name in ("zero", "direction", "physical_limits"):
            if getattr(contract, name + "_reviewed"):
                evidence = getattr(contract, name + "_evidence_sha256")
                need(evidence in verified_files, "Reviewed evidence file missing: " + name)
                source = Path(verified_files[evidence]).expanduser()
                if not source.is_absolute():
                    source = Path(path).resolve().parent / source
                need(hashlib.sha256(source.read_bytes()).hexdigest() == evidence,
                     "Reviewed evidence file changed: " + name)
        if contract.zero_reviewed:
            need(profile.get("physical_uncertainty_known") is True
                 and contract.uncertainty_rad > 0, "Reviewed zero needs explicit nonzero error bound")
        contracts[contract.motor_id] = contract
    need(set(contracts) == set(IDS), "Profile motor IDs must be 1..12")
    return profile, contracts


def write_private(path, value):
    path = Path(path).expanduser()
    need(not path.is_symlink(), "Output must not be a symlink")
    path = path.resolve()
    need(path.parent.is_dir(), "Create a private output directory first")
    need(not any((p / ".git").exists() for p in (path.parent, *path.parents)),
         "Telemetry output must be outside a Git checkout")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def policy_input_candidate(contracts, current, *, capture_sha256):
    """Adapt unique numeric branches to the existing no-output policy schema.

    This deliberately does not require physical review to run a diagnostic
    inference.  It cannot create an EpochAngleMap or become a motor calibration.
    The source boot/raw/capture hash remains attached to every generated offset.
    """
    raw, uids = current_values(current)
    report = audit_twelve_axes(contracts, raw, uids)
    need(not report["uid_changed_ids"], "UID mismatch prevents policy diagnostic candidate")
    boot = current.get("boot_id")
    need(type(boot) is str and boot.strip(), "Source capture boot ID missing")
    need(type(capture_sha256) is str and len(capture_sha256) == 64
         and all(c in "0123456789abcdef" for c in capture_sha256), "Source capture hash missing")
    candidates, models = [], {}
    for mid in IDS:
        axis = contracts[mid]
        options = report["rows_by_id"][str(mid)]["periodic_branch_candidates"]
        need(len(options) == 1 and options[0]["whole_uncertainty_inside_limits"],
             f"ID{mid}: no unique in-range numeric branch for diagnostic policy input")
        branch = options[0]
        offset = axis.offset_rad - axis.sign * branch["turns"] * TAU
        q = axis.sign * raw[str(mid)] + offset
        need(math.isclose(q, branch["model_rad"], rel_tol=0, abs_tol=1e-10),
             "Diagnostic branch algebra mismatch")
        models[str(mid)] = q
        candidates.append({"motor_id": mid, "sign_candidate": axis.sign,
                           "offset_candidate_rad": offset,
                           "diagnostic_branch_turns_embedded_in_offset": branch["turns"],
                           "physical_angle_accuracy_verified": False,
                           "sign_revalidated_for_runtime": False,
                           "approved_for_runtime": False})
    return {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
            "candidate_subtype": "UNIQUE_PERIODIC_BRANCH_NO_OUTPUT_DIAGNOSTIC_ONLY",
            "formula": "q_model = sign * raw + offset; rad; no wrapping",
            "offset_derivation": "base_offset - sign * unique_numeric_turns * 2*pi",
            "model_can_order_candidate": list(MODEL_CAN_ORDER),
            "identities": dict(uids), "candidates": candidates,
            "source_current_boot_id": boot,
            "source_current_motor_power_epoch_label": current.get("motor_power_epoch"),
            "source_capture_sha256": capture_sha256,
            "source_calibration_sha256_by_id": {
                str(mid): contracts[mid].calibration_sha256 for mid in IDS},
            "source_raw_rad_by_id": dict(raw),
            "model_rad_at_source_capture_by_id": models,
            "calibration_evidence_blockers_by_id": {
                str(mid): report["rows_by_id"][str(mid)]["blockers"] for mid in IDS},
            "diagnostic_inference_only": True, "epoch_binding_created": False,
            "physical_joint_limits_verified": False, "calibration_verified": False,
            "cross_boot_angle_continuity_verified": False,
            "motor_supply_off_on_evidence_complete": False,
            "raw_angles_modified": False, "live_50hz_verified": False,
            "motor_targets_generated": False, "approved_for_runtime": False,
            "motor_output_available": False, "output_allowed": False}


def batch_review_plan(report):
    """Separate reusable numeric/history evidence from missing physical review.

    This is a file-only work queue.  A matching old direction sign does not
    approve that direction; a unique nominal branch does not prove an encoder
    power epoch or the joint's physical zero and limits.
    """
    need(report.get("schema") == "singularitydog.twelve-angle-calibration-audit.v1"
         and report.get("approved_for_runtime") is False
         and report.get("output_allowed") is False,
         "Unapproved twelve-axis angle audit required")
    rows = report.get("rows_by_id")
    need(type(rows) is dict and set(rows) == {str(mid) for mid in IDS},
         "Twelve audited angle rows required")
    support = report.get("historical_direction_support_by_id", {})
    need(type(support) is dict and (not support or set(support) == set(rows)),
         "Historical direction support must cover all twelve axes if present")
    numeric, turns, history_matches, history_conflicts = [], {}, [], []
    for mid in IDS:
        key, row = str(mid), rows[str(mid)]
        options = row.get("periodic_branch_candidates")
        need(type(options) is list, f"ID{mid}: missing branch candidates")
        if row.get("identity_matches") is True and len(options) == 1 \
                and options[0].get("whole_uncertainty_inside_limits") is True:
            numeric.append(mid)
            turns[key] = options[0]["turns"]
        if support:
            matched = support[key].get("matches_current_hypothesis")
            need(type(matched) is bool, f"ID{mid}: historical support malformed")
            (history_matches if matched else history_conflicts).append(mid)
    epoch = report.get("motor_power_epoch")
    epoch_label_missing = type(epoch) is not str or epoch.strip() in UNKNOWN_EPOCHS
    return {
        "status": "BATCH_REVIEW_PLAN_NOT_APPROVAL",
        "numeric_branch_screen_pass_ids": numeric,
        "numeric_branch_turns_by_id": turns,
        "historical_direction_agrees_ids": history_matches,
        "historical_direction_conflicts_ids": history_conflicts,
        "historical_direction_record_review_ids": [
            mid for mid in history_matches if rows[str(mid)].get("direction_reviewed") is False],
        "priority_physical_direction_recheck_ids": history_conflicts,
        "direction_without_historical_record_ids": [] if support else [
            mid for mid in IDS if rows[str(mid)].get("direction_reviewed") is False],
        "physical_zero_error_review_ids": list(report.get("needs_zero_review_ids", [])),
        "physical_direction_review_ids": list(report.get("needs_direction_review_ids", [])),
        "physical_limit_review_ids": list(report.get("needs_physical_limit_review_ids", [])),
        "motor_power_epoch_label_missing": epoch_label_missing,
        "motor_power_epoch_attestation_verified_by_this_audit": False,
        "power_epoch_manifest_link_needed": True,
        "physical_uncertainty_known": report.get("physical_uncertainty_known") is True,
        "historical_consistency_is_not_physical_approval": True,
        "nominal_branch_is_not_physical_approval": True,
        "motor_targets_generated": False,
        "approved_for_runtime": False,
        "output_allowed": False,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--history-root", type=Path, help="Saved Jetson home or singularitydog-logs directory")
    mode.add_argument("--profile", type=Path, help="Explicit twelve-axis reviewed-evidence profile")
    ap.add_argument("--capture", type=Path, help="Fresh motor_epoch_readonly_capture JSON, with --profile")
    ap.add_argument("--references", type=Path, help="Optional per-ID external angle observation lists to fit")
    ap.add_argument("--profile-output", type=Path, help="Write imported candidate template; does not approve it")
    ap.add_argument("--policy-candidate-output", type=Path,
                    help="Write policy_shadow-compatible input candidate; diagnostic inference only")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    try:
        if args.history_root:
            need(args.capture is None, "History mode already identifies its pinned current capture")
            profile, contracts, current, raw, uids = history_profile(args.history_root)
            current_hash = profile["source_sha256"]["current"]
        else:
            need(args.capture is not None, "--profile requires --capture")
            profile, contracts = load_profile(args.profile)
            current, current_hash = read(args.capture)
            raw, uids = current_values(current)
        report = audit_twelve_axes(contracts, raw, uids)
        report["source_sha256"] = dict(profile.get("source_sha256", {}))
        if not args.history_root:
            # Keep the calibration profile's original evidence references, but
            # make "current" identify the fresh capture actually audited here.
            report["profile_source_sha256"] = dict(report["source_sha256"])
            report["source_sha256"]["current"] = current_hash
            report["current_capture_sha256"] = current_hash
        report["current_boot_id"] = current.get("boot_id")
        report["motor_power_epoch"] = current.get("motor_power_epoch")
        report["physical_uncertainty_known"] = profile.get("physical_uncertainty_known", False)
        report["assembly_revision"] = profile["assembly_revision"]
        support = profile.get("historical_direction_support_by_id", {})
        if support:
            report["historical_direction_support_by_id"] = support
            report["priority_direction_recheck_ids"] = [
                mid for mid in IDS if not support[str(mid)]["matches_current_hypothesis"]]
            report["direction_review_note"] = (
                "Unreviewed does not mean untested. Reuse matching historical evidence; "
                "recheck conflicting axes first, then review the retained per-axis sources.")
        report["epoch_binding_created"] = False
        if args.references:
            references, reference_hash = read(args.references)
            fits = {}
            for key, observations in references.items():
                need(key in {str(mid) for mid in IDS}, "Reference ID must be 1..12")
                try:
                    fit = fit_reference_observations(observations)
                    need(fit["uid"] == contracts[int(key)].uid, "Reference UID differs from this axis")
                    fits[key] = fit
                except ValueError as error:
                    fits[key] = {"status": "REMEASURE_THIS_AXIS", "error": str(error)}
            report["reference_fits_by_id"] = fits
            report["reference_source_sha256"] = reference_hash
            report["reference_fits_applied_to_profile"] = False
        report["batch_review_plan"] = batch_review_plan(report)
        candidate = (policy_input_candidate(contracts, current, capture_sha256=current_hash)
                     if args.policy_candidate_output else None)
        write_private(args.output, report)
        if args.profile_output:
            write_private(args.profile_output, profile)
        if args.policy_candidate_output:
            write_private(args.policy_candidate_output, candidate)
    except (OSError, ValueError, KeyError, TypeError) as error:
        ap.error(str(error))
    print(json.dumps({"output": str(args.output), "status": report["status"],
                      "needs_zero_review_ids": report["needs_zero_review_ids"],
                      "needs_direction_review_ids": report["needs_direction_review_ids"],
                      "uid_changed_ids": report["uid_changed_ids"],
                      "policy_candidate_output": str(args.policy_candidate_output) if args.policy_candidate_output else None,
                      "approved_for_runtime": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
