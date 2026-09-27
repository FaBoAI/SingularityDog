"""File-only candidate for a finite, supported twelve-axis standing transition.

This module deliberately has no serial/CAN imports or motor output.  A
supported fixed-standing candidate can be captured as twelve raw positions
on the execution boot; this does not need an inferred model-angle target.
The existing role-group diagnostics do not establish such a current-boot
target or clearance of the known front upper-leg/carbon-clamp contact. A
trajectory is useful for reviewing the whole swept path, but is *not* a grant
to drive it. The supported pose also does not prove that the dog can bear its
own weight after the stand is removed. The separate trial runtime remains
disabled until its frozen sources and physical evidence are reviewed.
"""

from __future__ import annotations

import math

from .rs05_trial_protocol import POSITION_MAX, POSITION_MIN

SCHEMA = "singularitydog.fixed-stance-candidate.v1"
IDS = tuple(range(1, 13))
SAMPLE_PERIOD_S = .05
RAMP_TICKS = 160
HOLD_TICKS = 20
MAX_SEGMENTS = 12  # Old ID10 target gap was about 90 raw degrees.
MAX_SEGMENT_DEG = 10.
# The physical-route review must cover every pose in this envelope; this is
# not an independent tolerance that an operator can simply relax in software.
REVIEWED_START_ENVELOPE_DEG = 3.
REQUIRED_REVIEWS = (
    "same_boot_12_axis_hold_passed",
    "same_boot_physical_stance_capture_verified",
    "all_segment_sweeps_physically_reviewed",
    "front_upper_leg_carbon_clamp_clearance_verified",
    "support_and_foot_clearance_verified",
    "attended_40v_cutoff_ready",
)


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be a finite number")
    try:
        value = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{label} must be finite") from error
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _raw_map(value, label):
    if type(value) is not dict or set(value) != {str(i) for i in IDS}:
        raise ValueError(f"{label} requires exactly string IDs 1..12")
    result = {i: _number(value[str(i)], f"{label}[{i}]") for i in IDS}
    if any(not POSITION_MIN <= angle <= POSITION_MAX for angle in result.values()):
        raise ValueError(f"{label} outside RS05 protocol position range")
    return result


def _identity_map(value):
    if type(value) is not dict or set(value) != {str(i) for i in IDS}:
        raise ValueError("Twelve exact motor identities are required")
    if any(type(uid) is not str or not uid.strip() for uid in value.values()):
        raise ValueError("Every motor identity must be a nonempty string")
    if len(set(value.values())) != len(IDS):
        raise ValueError("Motor identities must be unique")
    return dict(value)


def _corridor(value):
    if type(value) is not dict or set(value) != {str(i) for i in IDS}:
        raise ValueError('Reviewed raw corridor requires all twelve IDs')
    result = {}
    for mid in IDS:
        row = value[str(mid)]
        if type(row) is not dict or set(row) != {'min_rad', 'max_rad'}:
            raise ValueError(f'ID{mid} raw corridor requires min_rad and max_rad')
        low = _number(row['min_rad'], f'ID{mid} corridor minimum')
        high = _number(row['max_rad'], f'ID{mid} corridor maximum')
        if not POSITION_MIN <= low < high <= POSITION_MAX:
            raise ValueError(f'ID{mid} raw corridor outside protocol position range')
        result[mid] = (low, high)
    return result


def _digest(value, label):
    if (type(value) is not str or len(value) != 64
            or any(char not in "0123456789abcdef" for char in value)):
        raise ValueError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _quintic(u):
    return u*u*u*(10. + u*(-15. + 6.*u))


def prepare_candidate(review, *, current_boot_id, current_motor_uids,
                      fresh_raw_rad_by_id):
    """Validate evidence and return bounded samples; never enable output.

    The reviewed starts and all raw waypoints are bound to one exact boot and
    all twelve UIDs. The physically reviewed start envelope must be exactly
    three degrees, including axes nominally held still.
    Every waypoint segment is limited to ten raw degrees per axis with no
    wrapping.  This does not prove physical swept clearance; that is a
    separate on-machine review whose evidence hash is pinned here.
    """
    if type(review) is not dict or review.get("schema") != SCHEMA:
        raise ValueError("A fixed-stance candidate review is required")
    if (type(current_boot_id) is not str or not current_boot_id
            or review.get("boot_id") != current_boot_id):
        raise ValueError("Current and reviewed boot IDs differ")
    identities = _identity_map(current_motor_uids)
    if _identity_map(review.get("motor_uids")) != identities:
        raise ValueError("Current and reviewed motor identities differ")
    for name in ("hold_summary_sha256", "stance_capture_sha256",
                 "physical_route_review_sha256"):
        _digest(review.get(name), name)
    if (review.get("stance_capture_boot_id") != current_boot_id
            or _identity_map(review.get("stance_capture_motor_uids")) != identities
            or review.get("old_d17_target_reused") is not False):
        raise ValueError("Stance target must come from a current-boot physical capture, not D17")
    captured_stance = _raw_map(review.get("stance_capture_raw_rad_by_id"),
                               "physical stance capture")
    if review.get("hold_status") != "CURRENT_HOLD_COMPLETED_RESET_CONFIRMED":
        raise ValueError("A completed current-boot twelve-axis hold is required")
    if review.get("hold_stop_confirmed") is not True:
        raise ValueError("The prerequisite hold must confirm all twelve STOPs")
    if any(review.get(flag) is not True for flag in REQUIRED_REVIEWS):
        missing = [flag for flag in REQUIRED_REVIEWS if review.get(flag) is not True]
        raise ValueError("Required reviews missing: " + ", ".join(missing))
    if (type(review.get("clearance_reviewed_start_envelope_deg")) not in (int, float)
            or review["clearance_reviewed_start_envelope_deg"]
            != REVIEWED_START_ENVELOPE_DEG):
        raise ValueError("Physical clearance review must cover the exact three-degree start envelope")
    if (review.get("learned_policy_allowed") is not False
            or review.get("automatic_retry_allowed") is not False):
        raise ValueError("Policy handoff and automatic retry are outside this trial")

    reviewed_start = _raw_map(review.get("start_raw_rad_by_id"), "reviewed start")
    fresh_start = _raw_map(fresh_raw_rad_by_id, "fresh start")
    corridor = _corridor(review.get('reviewed_raw_corridor_by_id'))
    if (type(review.get('raw_corridor_physical_source_note')) is not str
            or not review['raw_corridor_physical_source_note'].strip()):
        raise ValueError('Raw corridor needs a physical source note')
    maximum_start_error = math.radians(REVIEWED_START_ENVELOPE_DEG)
    for mid in IDS:
        if abs(fresh_start[mid] - reviewed_start[mid]) > maximum_start_error:
            raise ValueError(f"ID{mid} left reviewed start envelope; no automatic wrapping")
        low, high = corridor[mid]
        if not low <= fresh_start[mid] <= high:
            raise ValueError(f'ID{mid} fresh start outside reviewed raw corridor')

    raw_waypoints = review.get("waypoints_raw_rad_by_id")
    if (type(raw_waypoints) is not list or not 1 <= len(raw_waypoints) <= MAX_SEGMENTS):
        raise ValueError("Require one to twelve explicitly reviewed twelve-axis waypoints")
    waypoints = [_raw_map(row, f"waypoint {index}")
                 for index, row in enumerate(raw_waypoints)]
    if (waypoints[-1] != captured_stance
            or waypoints[-1] != _raw_map(review.get("fixed_stance_raw_rad_by_id"),
                                        "fixed stance target")):
        raise ValueError("Final waypoint differs from current-boot physical stance capture")
    if (type(review.get("segment_clearance_sha256")) is not list
            or len(review["segment_clearance_sha256"]) != len(waypoints)):
        raise ValueError("Each segment needs separately pinned clearance evidence")
    for index, digest in enumerate(review["segment_clearance_sha256"]):
        _digest(digest, f"segment {index} clearance")

    samples = []
    previous = fresh_start
    bound = math.radians(MAX_SEGMENT_DEG)
    for segment, target in enumerate(waypoints):
        for mid in IDS:
            if abs(target[mid] - previous[mid]) > bound + 1e-12:
                raise ValueError(f"Segment {segment} ID{mid} exceeds ten raw degrees")
            low, high = corridor[mid]
            if not low <= target[mid] <= high:
                raise ValueError(f'Segment {segment} ID{mid} leaves reviewed raw corridor')
        for tick in range(RAMP_TICKS + HOLD_TICKS):
            blend = _quintic(min(tick / RAMP_TICKS, 1.))
            angles = {str(mid): (previous[mid] if tick == 0 else target[mid] if tick >= RAMP_TICKS
                                else previous[mid] + (target[mid] - previous[mid])*blend)
                      for mid in IDS}
            samples.append({"segment": segment, "tick": tick,
                            "elapsed_s": len(samples)*SAMPLE_PERIOD_S,
                            "raw_rad_by_id": angles})
        previous = target
    return {
        "schema": SCHEMA,
        "status": "OFFLINE_FINITE_STANCE_CANDIDATE_ONLY",
        "boot_id": current_boot_id,
        "motor_uids": identities,
        "waypoint_count": len(waypoints),
        "sample_period_s": SAMPLE_PERIOD_S,
        "sample_count": len(samples),
        "duration_s": len(samples)*SAMPLE_PERIOD_S,
        "samples": samples,
        "output_allowed": False,
        "live_runner_available": False,
        "policy_handoff_allowed": False,
        "support_required": True,
        "self_supported_standing_verified": False,
        "raw_to_model_mapping_verified": False,
        "physical_standing_verified": False,
        "automatic_retry_allowed": False,
        "remaining_before_drive": [
            "Review frozen dual-bus runtime, packet gate and active package authorization",
            "Confirm per-cycle feedback and all-twelve STOP on the current hardware",
            "Confirm current physical support, raw corridor, swept clearance and attended cutoff",
            "Verify load transfer and self-supported balance separately after supported standing",
        ],
    }
