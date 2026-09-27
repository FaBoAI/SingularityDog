"""Offline review of a historical stand target against a later raw snapshot.

The old D17 geometry target is useful for estimating work, but it is not a
current motor target. This module intentionally returns no trajectory samples,
motor commands, or actuation approval. In particular, a different raw-angle
turn is never silently normalized by +/-2*pi.
"""

import math

from .rs05_trial_protocol import POSITION_MAX, POSITION_MIN


IDS = tuple(range(1, 13))
STAGES = (
    ("front_hips_first", (3, 6)),
    ("rear_hips", (9, 12)),
    ("front_thighs_after_clearance_review", (2, 5)),
    ("rear_thighs", (8, 11)),
    ("calves", (1, 4, 7, 10)),
)
KNOWN_CONTACT_IDS = (2, 5)


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError(f"{label} must be finite") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _boot(value, label):
    if type(value) is not str or not value.strip():
        raise ValueError(f"{label} must identify the capture boot")
    return value


def _current(current):
    if (type(current) is not dict or current.get("read_only") is not True
            or current.get("motor_enable_sent") is not False):
        raise ValueError("Current input must be a read-only, non-actuating capture")
    boot = _boot(current.get("boot_id"), "current boot_id")
    motors = current.get("motors")
    if type(motors) is not dict or set(motors) != {str(i) for i in IDS}:
        raise ValueError("Current capture must include exactly twelve motor IDs")
    positions = {}
    for i in IDS:
        row = motors[str(i)]
        if type(row) is not dict or row.get("uid_match") is not True:
            raise ValueError(f"ID{i} is not identity-verified")
        value = _number(row.get("position_last_rad"), f"ID{i} current position")
        if not POSITION_MIN <= value <= POSITION_MAX:
            raise ValueError(f"ID{i} current position outside protocol range")
        positions[i] = value
    return boot, positions


def _historical_target(historical):
    if (type(historical) is not dict
            or historical.get("status") != "OFFLINE_GEOMETRY_ONLY"
            or historical.get("current_read_only") is not True
            or historical.get("motor_enable_sent") is not False):
        raise ValueError("Stand target must come from non-actuating offline geometry")
    boot = _boot(historical.get("boot_id"), "historical boot_id")
    rows = historical.get("joint_rows")
    if type(rows) is not list or len(rows) != len(IDS):
        raise ValueError("Historical target must include exactly twelve joint rows")
    result = {}
    for row in rows:
        if type(row) is not dict or type(row.get("id")) is not int:
            raise ValueError("Historical row has an invalid motor ID")
        i = row["id"]
        if i not in IDS or i in result:
            raise ValueError("Historical target has a duplicate or invalid motor ID")
        start = _number(row.get("start_raw_rad"), f"ID{i} historical start")
        step_deg = _number(row.get("raw_step_deg"), f"ID{i} historical step")
        step_end = _number(row.get("target_raw_rad"), f"ID{i} historical step end")
        remaining = _number(row.get("remaining_to_nominal_raw_deg"), f"ID{i} remaining")
        if abs(step_end - start - math.radians(step_deg)) > 1e-7:
            raise ValueError(f"ID{i} historical step is internally inconsistent")
        target = step_end + math.radians(remaining)
        if not POSITION_MIN <= target <= POSITION_MAX:
            raise ValueError(f"ID{i} inferred historical target outside protocol range")
        result[i] = target
    if set(result) != set(IDS):
        raise ValueError("Historical target must include every motor ID")
    return boot, result


def review_historical_stand(current, historical, *, review_step_deg=10.0):
    """Summarize staged angle gaps and blockers; never produce drive targets.

    The returned angle differences are diagnostics only. Stage ordering makes
    the known front-thigh/carbon-clamp conflict visible; it does not certify that
    moving hips first clears the full route. `review_step_deg` is for counting
    inspection intervals, not a motor amplitude or a runtime threshold.
    """
    step = _number(review_step_deg, "review_step_deg")
    if not 0 < step <= 10:
        raise ValueError("Review interval must be positive and at most ten degrees")
    current_boot, starts = _current(current)
    historical_boot, old_targets = _historical_target(historical)
    delta = {i: math.degrees(old_targets[i] - starts[i]) for i in IDS}
    turn_ambiguous = [i for i in IDS if abs(delta[i]) > 180]
    stages = []
    for name, ids in STAGES:
        gaps = {str(i): round(delta[i], 3) for i in ids}
        largest = max(abs(delta[i]) for i in ids)
        intervals = math.ceil(largest / step)
        blockers = []
        if current_boot != historical_boot:
            blockers.append("target_is_from_another_boot")
        if any(i in turn_ambiguous for i in ids):
            blockers.append("raw_turn_ambiguity_no_automatic_wrapping")
        if name == "front_thighs_after_clearance_review":
            blockers.append("known_carbon_clamp_contact_in_direct_route")
            blockers.append("hip_first_full_swept_clearance_unverified")
        stages.append({"stage": name, "motor_ids": list(ids),
                       "historical_delta_deg_by_id": gaps,
                       "largest_abs_delta_deg": round(largest, 3),
                       "review_intervals_at_most_10deg": intervals,
                       "blockers": blockers})
    return {"schema": "singularitydog.historical-stance-route-review.v1",
            "status": "BLOCKED_FOR_ACTUATION_REVIEW_ONLY",
            "output_allowed": False, "live_runner_available": False,
            "collision_clearance_verified": False, "calibration_verified": False,
            "start_angle_rebase_can_invalidate_clearance": True,
            "current_boot_id": current_boot, "historical_target_boot_id": historical_boot,
            "same_boot": current_boot == historical_boot,
            "known_contact_motor_ids": list(KNOWN_CONTACT_IDS),
            "raw_turn_ambiguous_motor_ids": turn_ambiguous,
            "review_interval_deg": step,
            "stages": stages,
            "required_before_motor_route": [
                "Capture all 12 current raw angles and UID on the execution boot",
                "Recheck full swept clearance after every start-angle rebase, even on the same boot",
                "Establish current calibration and raw turns without automatic wrapping",
                "Recompute nominal target from the current geometry and calibration",
                "Verify the complete hip-first/front-thigh swept path on the assembled robot",
                "Use a separately reviewed finite all-axis runner with STOP/watchdog",
            ]}
