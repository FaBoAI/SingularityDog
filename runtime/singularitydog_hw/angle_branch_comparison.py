"""Read-only, static-pose angle-branch comparison across a motor power cycle.

This module never opens hardware, changes calibration, or constructs motor
targets. A one-turn raw offset is removed only from a *comparison copy* after
independent evidence says the physical shafts did not turn through a revolution.
The raw value and inferred branch are always retained in the result.
"""

from __future__ import annotations

import math


IDS = tuple(str(i) for i in range(1, 13))
TWO_PI = 2.0 * math.pi
# This is a static-pose comparison window, not a joint limit or motion allowance.
MAX_STATIC_POSE_DELTA_RAD = math.radians(10.0)


class BranchComparisonError(ValueError):
    """The evidence cannot distinguish an encoder branch from physical motion."""


def _require(condition, message):
    if not condition:
        raise BranchComparisonError(message)


def _hash(value, name):
    _require(type(value) is str and len(value) == 64
             and all(ch in "0123456789abcdef" for ch in value), f"Invalid {name}")
    return value


def _uid_map(value):
    _require(type(value) is dict and set(value) == set(IDS), "Fresh twelve UIDs required")
    _require(all(type(uid) is str and len(uid) == 16
                 and all(ch in "0123456789abcdef" for ch in uid)
                 for uid in value.values()) and len(set(value.values())) == 12,
             "Malformed or duplicate motor UID")
    return dict(value)


def _snapshot(value):
    """Validate a caller-asserted, freshly identified quiet read-only capture.

    The caller must bind the motor power epoch to an independently observed
    supply Off/On event and independently establish STOP. The booleans here
    cannot prove physical disablement; run_mode=0 alone is not STOP evidence.
    A Jetson boot ID alone cannot identify a motor power epoch.
    """
    _require(type(value) is dict, "Invalid angle snapshot")
    boot, epoch = value.get("boot_id"), value.get("motor_power_epoch")
    _require(type(boot) is str and bool(boot.strip())
             and type(epoch) is str and bool(epoch.strip()),
             "Boot and motor power epoch are required")
    _require(value.get("uid_read_boot_id") == boot
             and value.get("uid_read_motor_power_epoch") == epoch,
             "UIDs were not freshly read in this boot and motor power epoch")
    _require(value.get("motor_output_allowed") is False,
             "Only a read-only capture is accepted")
    quiet = value.get("disabled_zero_current_by_id")
    _require(type(quiet) is dict and set(quiet) == set(IDS)
             and all(quiet[mid] is True for mid in IDS),
             "All twelve motors must be disabled at zero current")
    uids = _uid_map(value.get("uids_by_id"))
    raw = value.get("raw_rad_by_id")
    _require(type(raw) is dict and set(raw) == set(IDS),
             "Exactly twelve raw shaft angles required")
    _require(all(type(raw[mid]) in (int, float) and math.isfinite(raw[mid])
                 for mid in IDS), "Invalid raw shaft angle")
    return {"boot_id": boot, "motor_power_epoch": epoch,
            "capture_sha256": _hash(value.get("capture_sha256"), "capture hash"),
            "uid_capture_sha256": _hash(value.get("uid_capture_sha256"), "UID capture hash"),
            "uids_by_id": uids, "raw_rad_by_id": {mid: float(raw[mid]) for mid in IDS}}


def _no_turn_evidence(value, reference, current):
    _require(type(value) is dict and value.get("motor_supply_off_on_observed") is True,
             "Explicit motor supply Off/On evidence required")
    _require(value.get("reference_capture_sha256") == reference["capture_sha256"]
             and value.get("current_capture_sha256") == current["capture_sha256"],
             "No-turn evidence is not bound to both angle captures")
    evidence_hash = _hash(value.get("evidence_sha256"), "no-turn evidence hash")
    _require(evidence_hash not in (reference["capture_sha256"], current["capture_sha256"],
                                   reference["uid_capture_sha256"],
                                   current["uid_capture_sha256"]),
             "No-turn observation must have independent provenance")
    no_turn = value.get("no_full_physical_turn_by_id")
    _require(type(no_turn) is dict and bool(no_turn)
             and set(no_turn).issubset(IDS)
             and all(no_turn[mid] is True for mid in no_turn),
             "Each requested shaft needs explicit no-full-turn evidence")
    observations = value.get("physical_pose_observation_by_id")
    _require(type(observations) is dict and set(observations) == set(no_turn)
             and all(type(observations[mid]) is str and observations[mid].strip()
                     for mid in no_turn),
             "Each requested shaft needs a physical pose observation")
    return frozenset(no_turn)


def _comparison(reference, current, turns):
    rows = {}
    for mid in IDS:
        raw = current["raw_rad_by_id"][mid]
        equivalent = raw - turns.get(mid, 0) * TWO_PI
        rows[mid] = {"uid": current["uids_by_id"][mid],
                     "reference_raw_rad": reference["raw_rad_by_id"][mid],
                     "current_raw_rad": raw,
                     "direct_delta_rad": raw-reference["raw_rad_by_id"][mid],
                     "branch_reviewed": mid in turns,
                     "branch_turns_for_comparison": turns.get(mid, 0),
                     "comparison_raw_rad": equivalent,
                     "comparison_delta_rad": equivalent-reference["raw_rad_by_id"][mid]}
    return {"status": "STATIC_BRANCH_COMPARISON_ONLY", "rows": rows,
            "reference_boot_id": reference["boot_id"],
            "current_boot_id": current["boot_id"],
            "reference_motor_power_epoch": reference["motor_power_epoch"],
            "current_motor_power_epoch": current["motor_power_epoch"],
            "reference_capture_sha256": reference["capture_sha256"],
            "current_capture_sha256": current["capture_sha256"],
            "raw_targets_changed": False, "motor_output_allowed": False,
            "approved_for_runtime": False}


class StaticBranchComparison:
    """One static pose review. A failed later capture permanently aborts it."""

    def __init__(self, reference_capture, current_capture, no_turn_evidence):
        reference, current = _snapshot(reference_capture), _snapshot(current_capture)
        _require(reference["motor_power_epoch"] != current["motor_power_epoch"],
                 "A distinct observed motor power epoch is required")
        _require(reference["capture_sha256"] != current["capture_sha256"]
                 and reference["uid_capture_sha256"] != current["uid_capture_sha256"],
                 "Captures and fresh UID reads must be distinct")
        _require(reference["uids_by_id"] == current["uids_by_id"],
                 "Motor UID or ID assignment changed")
        reviewed = _no_turn_evidence(no_turn_evidence, reference, current)
        turns = {}
        for mid in reviewed:
            delta = current["raw_rad_by_id"][mid] - reference["raw_rad_by_id"][mid]
            turns[mid] = round(delta / TWO_PI)
            _require(abs(turns[mid]) <= 1, f"ID{mid} multi-turn difference is ambiguous")
            remainder = delta - turns[mid] * TWO_PI
            _require(abs(remainder) <= MAX_STATIC_POSE_DELTA_RAD,
                     f"ID{mid} physical pose difference or branch is ambiguous")
        self._reference = reference
        self._current = current
        self._turns = turns
        self._reviewed = reviewed
        self._evidence_sha256 = no_turn_evidence["evidence_sha256"]
        self._aborted = False

    def comparison(self):
        _require(not self._aborted, "Static branch comparison aborted")
        return _comparison(self._reference, self._current, self._turns)

    def validated_current_binding(self):
        """Copy the reviewed current epoch identity for diagnostic consumers.

        This is not an actuation token. A consumer must separately bind every
        later raw observation and fresh UID read to this epoch and still apply
        its own joint-range checks.
        """
        _require(not self._aborted, "Static branch comparison aborted")
        return {"boot_id": self._current["boot_id"],
                "motor_power_epoch": self._current["motor_power_epoch"],
                "capture_sha256": self._current["capture_sha256"],
                "uid_capture_sha256": self._current["uid_capture_sha256"],
                "evidence_sha256": self._evidence_sha256,
                "uids_by_id": dict(self._current["uids_by_id"]),
                "raw_rad_by_id": dict(self._current["raw_rad_by_id"]),
                "reviewed_branch_turns_by_id": dict(self._turns),
                "motor_output_allowed": False, "approved_for_runtime": False}

    def comparison_offset_candidates(self, signs_by_id, old_offsets_by_id):
        """Return algebraic observation offsets for proven axes, never targets.

        For raw_new = raw_old + k·2π, sign·raw_new + b_new equals the old
        model angle when b_new = b_old - sign·k·2π. This does not validate sign,
        physical zero, joint limits, or an actuator command branch.
        """
        _require(not self._aborted, "Static branch comparison aborted")
        _require(type(signs_by_id) is dict and set(signs_by_id) == set(IDS)
                 and all(type(signs_by_id[mid]) is int and signs_by_id[mid] in (-1, 1)
                         for mid in IDS), "Twelve explicit candidate signs required")
        _require(type(old_offsets_by_id) is dict and set(old_offsets_by_id) == set(IDS)
                 and all(type(old_offsets_by_id[mid]) in (int, float)
                         and math.isfinite(old_offsets_by_id[mid]) for mid in IDS),
                 "Twelve finite candidate offsets required")
        return {"status": "OBSERVATION_OFFSET_CANDIDATES_ONLY",
                "candidate_by_id": {
                    mid: {"branch_turns_for_comparison": self._turns[mid],
                          "comparison_offset_candidate_rad": (
                              old_offsets_by_id[mid]
                              - signs_by_id[mid] * self._turns[mid] * TWO_PI)}
                    for mid in sorted(self._reviewed, key=int)},
                "formula": "b_new = b_old - sign * branch_turns * 2*pi",
                "motor_output_allowed": False, "approved_for_runtime": False}

    def observe_same_epoch(self, capture):
        """Check a later static read; never infer a new branch within an epoch."""
        _require(not self._aborted, "Static branch comparison aborted")
        try:
            next_capture = _snapshot(capture)
            previous = self._current
            _require(next_capture["boot_id"] == previous["boot_id"]
                     and next_capture["motor_power_epoch"] == previous["motor_power_epoch"]
                     and next_capture["uids_by_id"] == previous["uids_by_id"],
                     "Boot, power epoch or UID changed within comparison")
            _require(next_capture["capture_sha256"] != previous["capture_sha256"]
                     and next_capture["uid_capture_sha256"] != previous["uid_capture_sha256"],
                     "Repeated capture or stale UID read")
            for mid in self._reviewed:
                step = next_capture["raw_rad_by_id"][mid] - previous["raw_rad_by_id"][mid]
                _require(abs(step) <= MAX_STATIC_POSE_DELTA_RAD,
                         f"ID{mid} within-epoch raw angle discontinuity")
                residual = (next_capture["raw_rad_by_id"][mid]
                            - self._turns[mid] * TWO_PI
                            - self._reference["raw_rad_by_id"][mid])
                _require(abs(residual) <= MAX_STATIC_POSE_DELTA_RAD,
                         f"ID{mid} no longer matches the static reference pose")
            self._current = next_capture
            return self.comparison()
        except BaseException:
            self._aborted = True
            raise
