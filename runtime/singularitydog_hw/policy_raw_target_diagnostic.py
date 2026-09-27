"""Pure, no-output inverse mapping of one reviewed policy observation.

This consumes an actual observer tick and the exact snapshot that produced it.
It only accepts the observer's reviewed motor-power branch provenance.  The
result is a numerical candidate, never an RS05 frame or an actuation permit.
The caller's delta ceiling is a diagnostic screen, not a verified motion limit.
"""

import hashlib
import json
import math
import struct

from . import policy_shadow as shadow
from .angle_branch_comparison import IDS, MAX_STATIC_POSE_DELTA_RAD, TWO_PI
from .rs05_trial_protocol import POSITION_MIN, POSITION_MAX


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def _digest(value):
    try:
        encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                             allow_nan=False).encode()
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("Non-JSON or nonfinite diagnostic input") from error
    return hashlib.sha256(encoded).hexdigest()


def _number(value, label):
    _need(type(value) in (int, float), "Invalid " + label)
    try:
        converted = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError("Invalid " + label) from error
    _need(math.isfinite(converted), "Nonfinite " + label)
    return converted


def _same(a, b):
    return math.isclose(a, b, rel_tol=0., abs_tol=1e-8)


def project_raw_targets(observer_tick, snapshot, calibration, *, max_abs_delta_rad):
    """Screen one observer tick against its original, same-epoch raw snapshot.

    A `StaticBranchComparison` must already have been supplied to the observer;
    its copied provenance binds the snapshot to fresh UID and power-epoch
    assertions.  No modulo, clipping, alternate branch search, gain selection,
    CAN encoding, hardware I/O, or persistent calibration change occurs here.
    """
    _need(type(observer_tick) is dict and type(snapshot) is dict
          and type(calibration) is dict, "Require observer, snapshot and calibration objects")
    _need(observer_tick.get("status") == "TICK_OBSERVED_NO_OUTPUT"
          and observer_tick.get("output_allowed") is False
          and observer_tick.get("motor_output_available") is False
          and observer_tick.get("approved_for_runtime") is False,
          "Require a no-output policy observer tick")
    _need(calibration.get("approved_for_runtime") is False,
          "Require an explicitly unapproved calibration candidate")
    rows = shadow.validate_calibration(calibration)
    _need(all(row.get("approved_for_runtime") is not True for row in rows.values()),
          "Approved calibration rows are outside this diagnostic contract")
    provenance = observer_tick.get("provenance")
    _need(type(provenance) is dict
          and provenance.get("calibration_canonical_json_sha256") == _digest(calibration)
          and provenance.get("snapshot_canonical_json_sha256") == _digest(snapshot)
          and provenance.get("model_can_order_candidate") == shadow.CAN_ORDER,
          "Observer provenance does not bind this calibration and exact snapshot")
    _need(snapshot.get("status") == "DIAGNOSTIC_READY"
          and snapshot.get("output_allowed") is False
          and snapshot.get("blocked_reasons") == []
          and snapshot.get("tick_ns") == observer_tick.get("tick_ns")
          and type(snapshot.get("tick_ns")) is int and snapshot["tick_ns"] > 0,
          "Snapshot is blocked or from another tick")
    age_limit = snapshot.get("max_age_ns")
    _need(type(age_limit) is int and 0 < age_limit <= 1_000_000_000,
          "Missing finite snapshot freshness limit")
    source_flags = snapshot.get("source_flags", {})
    _need(type(source_flags) is dict
          and source_flags == provenance.get("snapshot_source_flags"),
          "Observer and snapshot source flags differ")
    source = source_flags.get("power_epoch_branch_capture")
    overlay = provenance.get("power_epoch_branch_overlay")
    _need(type(source) is dict and type(overlay) is dict
          and overlay.get("kind") == "power_epoch_static_branch_observation_only"
          and overlay.get("motor_output_allowed") is False
          and overlay.get("approved_for_runtime") is False,
          "Reviewed power-epoch observation is required")
    binding = overlay.get("reviewed_capture_binding")
    _need(type(binding) is dict and binding.get("motor_output_allowed") is False
          and binding.get("approved_for_runtime") is False,
          "Missing reviewed capture binding")
    for key in ("boot_id", "motor_power_epoch", "capture_sha256",
                "uid_capture_sha256", "evidence_sha256", "uids_by_id", "raw_rad_by_id"):
        _need(source.get(key) == binding.get(key), "Power-epoch binding mismatch: " + key)
    boot, epoch = source.get("boot_id"), source.get("motor_power_epoch")
    _need(type(boot) is str and bool(boot.strip())
          and type(epoch) is str and bool(epoch.strip())
          and source.get("uid_read_boot_id") == boot
          and source.get("uid_read_motor_power_epoch") == epoch
          and source.get("motor_output_allowed") is False
          and source.get("motor_supply_off_on_observed") is True,
          "Missing fresh UID or motor power-epoch assertion")
    uids = source.get("uids_by_id")
    _need(type(uids) is dict and set(uids) == set(IDS)
          and uids == calibration["identities"] and len(set(uids.values())) == 12,
          "Motor UIDs do not match calibration")
    quiet = source.get("disabled_zero_current_by_id")
    _need(type(quiet) is dict and set(quiet) == set(IDS)
          and all(quiet[mid] is True for mid in IDS),
          "Missing disabled zero-current capture assertions")
    capture_raw = source.get("raw_rad_by_id")
    _need(type(capture_raw) is dict and set(capture_raw) == set(IDS),
          "Incomplete reviewed raw capture")
    turns = binding.get("reviewed_branch_turns_by_id")
    _need(type(turns) is dict and bool(turns) and set(turns).issubset(IDS)
          and all(type(turn) is int and turn in (-1, 0, 1) for turn in turns.values()),
          "Missing or invalid reviewed branch turns")
    observed = overlay.get("raw_position_rad_by_id")
    compared = overlay.get("comparison_position_rad_by_id")
    _need(type(observed) is dict and set(observed) == set(IDS)
          and type(compared) is dict and set(compared) == set(IDS),
          "Incomplete branch observation")
    motors = snapshot.get("motors")
    _need(type(motors) is list and len(motors) == 24,
          "Exact observer snapshot needs 24 motor readings")
    positions = {}
    for row in motors:
        _need(type(row) is dict, "Malformed motor reading")
        if row.get("parameter") != "position":
            continue
        mid = row.get("motor_id")
        _need(type(mid) is int and str(mid) in IDS and mid not in positions
              and row.get("unit") == "rad", "Invalid or duplicate raw position")
        start, end = row.get("request_ns"), row.get("received_ns")
        _need(type(start) is int and type(end) is int
              and 0 < start <= end <= snapshot["tick_ns"]
              and snapshot["tick_ns"] - start <= age_limit
              and row.get("age_upper_bound_ns") == snapshot["tick_ns"] - start,
              "Raw position is stale or time-inconsistent")
        positions[mid] = _number(row.get("value"), "raw position")
    _need(set(positions) == set(range(1, 13)), "Missing raw position")
    inputs = observer_tick.get("inputs")
    target = observer_tick.get("q_target_rad_diagnostic_only")
    _need(type(inputs) is dict and type(inputs.get("q_model_rad")) is list
          and len(inputs["q_model_rad"]) == 12
          and type(target) is list and len(target) == 12,
          "Missing twelve model current/target values")
    ceiling = _number(max_abs_delta_rad, "diagnostic delta ceiling")
    _need(0. < ceiling <= math.pi, "Diagnostic delta ceiling must be in (0, pi] rad")

    result_rows = []
    for index, mid in enumerate(shadow.CAN_ORDER):
        key = str(mid)
        raw = positions[mid]
        turn = turns.get(key, 0)
        capture_value = _number(capture_raw[key], "reviewed capture raw")
        _need(abs(raw - capture_value) <= MAX_STATIC_POSE_DELTA_RAD,
              "Raw position left reviewed static branch window: ID" + key)
        comparison_raw = raw - turn * TWO_PI
        _need(_same(_number(observed[key], "observed raw"), raw)
              and _same(_number(compared[key], "comparison raw"), comparison_raw),
              "Observer raw/branch provenance mismatch: ID" + key)
        sign = rows[mid]["sign_candidate"]
        offset = _number(rows[mid]["offset_candidate_rad"], "calibration offset")
        current_q = _number(inputs["q_model_rad"][index], "model current")
        goal_q = _number(target[index], "model target")
        lower = struct.unpack("<f", struct.pack("<f", shadow.LOWER[index]))[0]
        upper = struct.unpack("<f", struct.pack("<f", shadow.UPPER[index]))[0]
        _need(_same(current_q, sign * comparison_raw + offset)
              and lower <= current_q <= upper,
              "Model current disagrees with reviewed raw branch: ID" + key)
        _need(lower <= goal_q <= upper,
              "Model target outside registered joint range: ID" + key)
        # sign is +/-1, so it is its own inverse. Reapply the *current* raw
        # power-epoch branch only after inverting the model-space observation.
        raw_goal = sign * (goal_q - offset) + turn * TWO_PI
        _need(math.isfinite(raw_goal)
              and POSITION_MIN <= raw <= POSITION_MAX
              and POSITION_MIN <= raw_goal <= POSITION_MAX,
              "Raw current/target outside RS05 encoding range: ID" + key)
        delta = raw_goal - raw
        _need(math.isfinite(delta) and abs(delta) <= ceiling,
              "Candidate target exceeds explicit delta ceiling: ID" + key)
        result_rows.append({"motor_id": mid, "model_current_rad": current_q,
                            "model_target_rad": goal_q, "model_lower_rad": lower,
                            "model_upper_rad": upper, "raw_current_rad": raw,
                            "raw_target_rad_diagnostic_only": raw_goal,
                            "raw_delta_rad_diagnostic_only": delta,
                            "raw_protocol_lower_rad": POSITION_MIN,
                            "raw_protocol_upper_rad": POSITION_MAX,
                            "diagnostic_delta_ceiling_rad": ceiling,
                            "reviewed_branch_turns": turn,
                            "model_limit_screen_passed": True,
                            "raw_protocol_limit_screen_passed": True,
                            "delta_screen_passed": True})
    return {"status": "RAW_TARGET_CANDIDATES_NO_OUTPUT",
            "motor_output_available": False, "output_allowed": False,
            "approved_for_runtime": False, "calibration_verified": False,
            "physical_joint_limits_verified": False, "live_50hz_verified": False,
            "hardware_opened": False, "can_frames_constructed": False,
            "source_observer_tick_ns": observer_tick["tick_ns"],
            "source_snapshot_sha256": _digest(snapshot),
            "source_calibration_sha256": _digest(calibration),
            "source_boot_id": boot, "source_motor_power_epoch": epoch,
            "source_uid_capture_sha256": source["uid_capture_sha256"],
            "source_raw_capture_sha256": source["capture_sha256"],
            "motor_rows_model_order": result_rows,
            "maximum_abs_raw_delta_rad": max(abs(row["raw_delta_rad_diagnostic_only"])
                                             for row in result_rows)}
