"""One saved-snapshot policy observation, with no hardware or motor-output path.

Run from SingularityDog with ``PYTHONPATH=runtime python3 -B tools/saved_policy_once.py``.
The normal mode accepts a completed integrated STOP-proxy report containing its
original snapshot. ``--range-stopped-capture`` is a separate, tightly limited
mode for a saved capture stopped by the original joint-range guard. It decodes
the twelve Type2 replies again and uses the *recorded snapshot end*, not the
unknown original tick, as a new offline tick. The source remains INCOMPLETE.
The observer checks all 24 motor values, IMU timestamps,
candidate joint limits, and policy target limits before returning a result.
Three synthetic warmup calls are followed by one reset and at most one call on
the saved input. The result is diagnostic only, even if that call succeeds.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import stat

from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_observer_replay as replay
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw import fast_policy_inputs


MAX_DIAGNOSTIC_AGE_NS = 100_000_000


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _saved_json(path):
    path = Path(path)
    _require(path.is_file() and stat.S_ISREG(path.stat().st_mode),
             "Require a saved regular JSON file: " + str(path))
    raw = path.read_bytes()
    return shadow._json(raw.decode("utf-8")), hashlib.sha256(raw).hexdigest()


def _saved_jsonl(path):
    path = Path(path)
    _require(path.is_file() and stat.S_ISREG(path.stat().st_mode),
             "Require a saved regular JSONL file: " + str(path))
    raw = path.read_bytes()
    return [shadow._json(line) for line in raw.decode("utf-8").splitlines() if line.strip()], \
        hashlib.sha256(raw).hexdigest()


def _calibration(candidate):
    _require(type(candidate) is dict, "Invalid calibration candidate")
    rows = shadow.validate_calibration(candidate)
    _require(candidate.get("approved_for_runtime") is False,
             "Require an unapproved calibration candidate")
    unverified = ("calibration_verified", "physical_angle_accuracy_verified",
                  "zero_verified", "sign_verified", "sign_revalidated",
                  "motor_power_cycle_continuity_verified", "angle_wrapping_applied",
                  "turn_correction_applied")
    for key in unverified:
        _require(candidate.get(key) is not True, "Calibration claims verified " + key)
    for row in rows.values():
        _require(row.get("approved_for_runtime") is False,
                 "Require all calibration rows to remain unapproved")
        for key in unverified:
            _require(row.get(key) is not True, "Calibration row claims verified " + key)
    return candidate


def _snapshot(report):
    _require(type(report) is dict and report.get("status") in (
        "COMPLETE_INTEGRATED_STOP_PROXY", "OFFLINE_RECONSTRUCTED_RANGE_STOP_ONLY"),
        "Require a completed report or explicitly reconstructed range-stopped source")
    snapshot = report.get("snapshot")
    _require(type(snapshot) is dict and snapshot.get("status") == "DIAGNOSTIC_READY"
             and snapshot.get("output_allowed") is False,
             "Require a saved or explicitly reconstructed no-output diagnostic snapshot")
    for name in ("max_age_ns", "max_spread_ns"):
        value = snapshot.get(name)
        _require(type(value) is int and 0 <= value <= MAX_DIAGNOSTIC_AGE_NS,
                 "Invalid or excessive diagnostic " + name)
    return snapshot


def reconstruct_range_stopped_capture(directory, calibration):
    """Rebuild a diagnostic input from one exact saved range-stop case only.

    The original unrecorded tick and original snapshot cannot be reproduced.
    ``snapshot_end_ns`` is a later, recorded host instant and therefore a new
    offline tick. Source request/read times are left untouched.
    """
    root = Path(directory)
    report, report_hash = _saved_json(root / "report.json")
    _require(type(report) is dict and report.get("status") == "INCOMPLETE"
             and report.get("cycles") == 1 and report.get("output_allowed") is False
             and report.get("learned_target_sent") is False
             and report.get("calibration_verified") is False
             and report.get("h_measured") is False,
             "Require the exact no-output, one-cycle incomplete capture scope")
    errors = report.get("errors")
    _require(type(errors) is list and len(errors) == 4
             and all(type(e) is dict for e in errors)
             and errors[0].get("scope") == "coordinator"
             and errors[0].get("error") ==
                "ObserverError('Calibrated position outside registered joint range; no clipping')"
             and {(e.get("scope"), e.get("error")) for e in errors[1:]} == {
                 (name, "InterruptedError('Integrated collection cancelled')")
                 for name in ("front", "rear", "imu")},
             "Capture did not stop solely at the registered joint-range guard")
    stamps = report.get("stamps")
    _require(type(stamps) is dict, "Missing saved snapshot interval")
    ready, start, end = (stamps.get(name) for name in (
        "inputs_ready_ns", "snapshot_start_ns", "snapshot_end_ns"))
    _require(all(type(value) is int and 0 < value < 2**63
                 for value in (ready, start, end)) and ready <= start <= end,
             "Invalid saved snapshot interval")
    _require(report.get("imu_sample") is not None, "Missing original IMU sample")
    selected, hashes = [], {"report_sha256": report_hash}
    identities = {}
    for bus in ("front", "rear"):
        events, hashes[bus + "_jsonl_sha256"] = _saved_jsonl(root / (bus + ".jsonl"))
        expected_ids = range(1, 7) if bus == "front" else range(7, 13)
        for event in events:
            if type(event) is not dict or event.get("kind") != "pipeline_reply":
                continue
            mid = event.get("motor_id")
            if event.get("cycle") == 0 and event.get("parameter") == "identity":
                _require(type(mid) is int and mid in expected_ids and mid not in identities
                         and event.get("ok") is True and type(event.get("result")) is dict
                         and event["result"].get("ok") is True
                         and event["result"].get("mcu_uid_hex") ==
                             calibration["identities"][str(mid)],
                         "Saved identity does not match calibration candidate")
                identities[mid] = True
            elif event.get("cycle") == 1 and event.get("parameter") == "stop_feedback":
                _require(type(mid) is int and mid in expected_ids,
                         "Saved stop feedback came from the wrong bus")
                selected.append(event)
    _require(set(identities) == set(range(1, 13)), "Require twelve saved matching identities")
    prepared = fast_policy_inputs.prepare_cycle(selected, report["imu_sample"], cycle=1)
    latest_available = max(prepared.imu_available,
                           *(motor.available for motor in prepared.motors))
    _require(latest_available <= ready <= start,
             "Source became available after the recorded snapshot start")
    snapshot = prepared.snapshot(end)
    wrapper = {"status": "OFFLINE_RECONSTRUCTED_RANGE_STOP_ONLY",
               "source_capture_status": "INCOMPLETE", "snapshot": snapshot,
               "reconstruction": {
                   "original_snapshot_saved": False, "original_tick_known": False,
                   "offline_tick_source": "recorded_snapshot_end_ns",
                   "offline_tick_ns": end, "original_tick_interval_ns": [start, end],
                   "source_timestamps_changed": False, "raw_type2_redecoded": True,
                   "decoded_identity_candidate_match_only": True,
                   "source_capture_range_guard_stopped": True,
                   "current_pose_after_capture_verified": False,
                   "source_capture_approved_for_runtime": False}}
    return wrapper, hashes


def _range_screen(snapshot, calibration):
    """Reject an out-of-range saved pose before even the synthetic warmup."""
    motors = snapshot.get("motors")
    _require(type(motors) is list and len(motors) == 24,
             "Require 24 saved motor observations")
    positions = {}
    for row in motors:
        _require(type(row) is dict, "Malformed saved motor observation")
        if row.get("parameter") != "position":
            continue
        mid, value = row.get("motor_id"), row.get("value")
        _require(type(mid) is int and 1 <= mid <= 12 and mid not in positions
                 and row.get("unit") == "rad" and type(value) in (int, float),
                 "Invalid or repeated saved position")
        try:
            finite = math.isfinite(value)
        except (OverflowError, ValueError):
            finite = False
        _require(finite, "Nonfinite saved position")
        positions[mid] = value
    _require(set(positions) == set(range(1, 13)), "Missing saved joint position")
    rows = shadow.validate_calibration(calibration)
    blocked = []
    for mid, lower, upper in zip(shadow.CAN_ORDER, shadow.LOWER, shadow.UPPER):
        row = rows[mid]
        q = row["sign_candidate"] * positions[mid] + row["offset_candidate_rad"]
        _require(shadow.finite(q), "Nonfinite candidate model angle")
        if not lower <= q <= upper:
            blocked.append(mid)
    _require(not blocked, "Calibrated position outside registered joint range; no clipping: motor IDs "
             + ",".join(map(str, blocked)))


def infer_once(report, calibration, mount, policy, torch_module, *, h_hypothesis):
    """Apply the existing guarded observer to one saved report; never write data."""
    snapshot = _snapshot(report)
    calibration = _calibration(calibration)
    shadow.validate_imu_mount_candidate(mount)
    _range_screen(snapshot, calibration)
    _require(type(h_hypothesis) is int and h_hypothesis in (0, 1),
             "Choose an explicit h=0 or h=1 diagnostic hypothesis")
    run = observer.StatefulPolicyObserver(
        policy, calibration, imu_mount_candidate=mount, h_hypothesis=h_hypothesis,
        command=[0., 0., 0.], max_ticks=1, max_age_ns=snapshot["max_age_ns"],
        max_spread_ns=snapshot["max_spread_ns"], torch_module=torch_module)
    replay.warmup_policy(policy, torch_module, h_hypothesis, 3)
    run.reset_run(snapshot["tick_ns"], warmup_completed=True)
    tick = run.consume(snapshot)  # Input bounds are checked before the saved-input model call.
    summary = run.finish()
    return {"status": "OFFLINE_SAVED_POLICY_OBSERVED_NO_OUTPUT",
            "output_allowed": False, "motor_output_available": False,
            "hardware_opened": False, "approved_for_runtime": False,
            "calibration_verified": False, "physical_angle_accuracy_verified": False,
            "sensor_alignment_verified": False, "raw_driver_axes_verified": False,
            "h_measured": False, "live_50hz_verified": False,
            "source_wire_revalidated_by_this_tool": bool(
                report.get("reconstruction", {}).get("raw_type2_redecoded")),
            "fresh_identity_match_verified_by_this_tool": False,
            "source_capture_status": report.get("source_capture_status", report["status"]),
            "reconstruction": report.get("reconstruction"),
            "saved_input_policy_calls": 1, "synthetic_warmup_calls": 3,
            "source_timestamps_changed": False, "clipping_applied": False,
            "angle_wrapping_applied": False, "observer_summary": summary, "tick": tick}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--report", type=Path)
    source.add_argument("--range-stopped-capture", type=Path)
    for name in ("calibration", "imu-mount-candidate", "bundle"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--h-hypothesis", required=True, type=int, choices=(0, 1))
    args = parser.parse_args(argv)
    source_hashes = {}
    try:
        calibration, source_hashes["calibration_sha256"] = _saved_json(args.calibration)
        mount, source_hashes["imu_mount_candidate_sha256"] = _saved_json(args.imu_mount_candidate)
        _calibration(calibration)
        shadow.validate_imu_mount_candidate(mount)
        if args.report is not None:
            report, source_hashes["report_sha256"] = _saved_json(args.report)
            _require(type(report) is dict and report.get("status") ==
                     "COMPLETE_INTEGRATED_STOP_PROXY",
                     "--report requires an original completed integrated report")
        else:
            report, capture_hashes = reconstruct_range_stopped_capture(
                args.range_stopped_capture, calibration)
            source_hashes.update(capture_hashes)
        _snapshot(report)
        _range_screen(report["snapshot"], calibration)
        policy, model_source = shadow.load_policy(args.bundle)
        import torch
        result = infer_once(report, calibration, mount, policy, torch,
                            h_hypothesis=args.h_hypothesis)
        result["sources_sha256"] = source_hashes
        result["model"] = model_source
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0
    except (OSError, UnicodeError, ValueError, TypeError, KeyError, RuntimeError,
            ImportError, OverflowError) as error:
        print(json.dumps({"status": "OFFLINE_SAVED_POLICY_BLOCKED",
                          "reason": type(error).__name__ + ": " + str(error),
                          "output_allowed": False, "hardware_opened": False,
                          "approved_for_runtime": False,
                          "calibration_verified": False, "live_50hz_verified": False,
                          "sources_sha256": source_hashes}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
