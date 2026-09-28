"""Pure angle calibration and epoch branch diagnostics; no CAN or motor output.

An encoder turn is not a joint zero.  Modulo arithmetic is useful for comparing
orientations, while a command must remain on the current encoder branch.  This
module makes that distinction explicit.  Physical calibration and mechanical
limits are supplied as reviewed evidence, never inferred from an URDF or from
the fact that a policy input happens to be in range.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


IDS = tuple(range(1, 13))
TAU = 2 * math.pi
UNKNOWN_EPOCHS = {"", "UNKNOWN", "NOT_INFERRED_FROM_JETSON_BOOT"}
MODEL_CAN_ORDER = (6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7)


class AngleEvidenceError(ValueError):
    """A candidate is ambiguous or is not bound to sufficient evidence."""


def _need(value, message):
    if not value:
        raise AngleEvidenceError(message)


def _finite(value, name):
    _need(type(value) in (int, float), f"Invalid {name}")
    try:
        converted = float(value)
    except (OverflowError, ValueError) as error:
        raise AngleEvidenceError(f"Invalid {name}") from error
    _need(math.isfinite(converted), f"Nonfinite {name}")
    return converted


def _sha(value, name):
    _need(type(value) is str and len(value) == 64
          and all(ch in "0123456789abcdef" for ch in value), f"Invalid {name}")
    return value


def _uid(value):
    _need(type(value) is str and len(value) == 16
          and all(ch in "0123456789abcdef" for ch in value), "Invalid motor UID")
    return value


def signed_periodic_delta_rad(value, reference):
    """Shortest signed orientation difference; exactly half a turn is ambiguous.

    This function must not be used to hide jumps in a live encoder sequence or
    to turn a modulo result directly into a motor target.
    """
    difference = _finite(value, "angle") - _finite(reference, "reference")
    _need(math.isfinite(difference), "Angle difference overflow")
    delta = math.remainder(difference, TAU)
    _need(not math.isclose(abs(delta), math.pi, rel_tol=0, abs_tol=1e-12),
          "Half-turn orientation difference is ambiguous")
    return delta


@dataclass(frozen=True)
class AxisCalibration:
    """A reviewable q = sign * (raw - turns * 2*pi) + offset contract.

    ``lower/upper`` must be physical joint limits in the model convention.
    ``uncertainty`` is a bound, not a standard deviation.  A visual L pose with
    unknown error cannot be supplied as a zero with zero uncertainty.
    """

    motor_id: int
    uid: str
    sign: int
    offset_rad: float
    lower_rad: float
    upper_rad: float
    uncertainty_rad: float
    calibration_sha256: str
    zero_reviewed: bool = False
    direction_reviewed: bool = False
    physical_limits_reviewed: bool = False
    zero_evidence_sha256: str | None = None
    direction_evidence_sha256: str | None = None
    physical_limits_evidence_sha256: str | None = None

    def __post_init__(self):
        _need(type(self.motor_id) is int and self.motor_id in IDS, "Invalid motor ID")
        _uid(self.uid)
        _need(type(self.sign) is int and self.sign in (-1, 1), "Sign must be +1 or -1")
        for name in ("offset_rad", "lower_rad", "upper_rad", "uncertainty_rad"):
            _finite(getattr(self, name), name)
        _need(self.lower_rad < self.upper_rad and self.uncertainty_rad >= 0,
              "Invalid angle interval or uncertainty")
        _sha(self.calibration_sha256, "calibration hash")
        for name in ("zero", "direction", "physical_limits"):
            flag = getattr(self, name + "_reviewed")
            _need(type(flag) is bool, "Evidence review flag must be boolean")
            if flag:
                _sha(getattr(self, name + "_evidence_sha256"), name + " evidence hash")

    def evidence_blockers(self):
        blockers = []
        if not self.zero_reviewed:
            blockers.append("ZERO_ACCURACY_NOT_REVIEWED")
        if not self.direction_reviewed:
            blockers.append("DIRECTION_NOT_REVIEWED")
        if not self.physical_limits_reviewed:
            blockers.append("MECHANICAL_LIMITS_NOT_REVIEWED")
        if self.upper_rad - self.lower_rad >= TAU:
            blockers.append("MECHANICAL_INTERVAL_AT_LEAST_ONE_TURN")
        return blockers


def equivalent_branch_candidates(raw_rad, calibration):
    """Enumerate all encoder branches whose uncertainty intersects the interval.

    This is an arithmetic screen, including for unreviewed calibration.  An
    intersection at a limit is reported as marginal and cannot be bound by
    ``resolve_current_branch``.  No nearest-branch heuristic or clipping occurs.
    """
    _need(isinstance(calibration, AxisCalibration), "AxisCalibration required")
    return _branch_candidates(raw_rad, calibration.sign, calibration.offset_rad,
                              calibration.lower_rad, calibration.upper_rad,
                              calibration.uncertainty_rad)


def _branch_candidates(raw_rad, sign, offset_rad, lower_rad, upper_rad, uncertainty_rad):
    """Shared arithmetic for offline review and the reviewed output preflight."""
    _need(type(sign) is int and sign in (-1, 1), "Sign must be +1 or -1")
    raw = _finite(raw_rad, "raw angle")
    offset = _finite(offset_rad, "zero offset")
    lower = _finite(lower_rad, "physical lower limit")
    upper = _finite(upper_rad, "physical upper limit")
    uncertainty = _finite(uncertainty_rad, "angle uncertainty")
    _need(lower < upper and uncertainty >= 0, "Invalid physical angle interval")
    base = sign * raw + offset
    _need(math.isfinite(base), "Model angle overflow")
    # model = base - sign * turns * 2*pi
    expanded_lo = lower - uncertainty
    expanded_hi = upper + uncertainty
    ends = ((base - expanded_lo) / (sign * TAU),
            (base - expanded_hi) / (sign * TAU))
    _need(all(math.isfinite(value) and abs(value) < 2**40 for value in ends),
          "Raw branch index is not numerically bounded")
    first, last = math.ceil(min(ends) - 1e-12), math.floor(max(ends) + 1e-12)
    _need(last - first < 100, "Mechanical interval is too broad")
    result = []
    for turns in range(first, last + 1):
        q = sign * (raw - turns * TAU) + offset
        result.append({"turns": turns, "model_rad": q,
                       "model_interval_rad": [q - uncertainty, q + uncertainty],
                       "whole_uncertainty_inside_limits": (
                           q - uncertainty >= lower - 1e-12
                           and q + uncertainty <= upper + 1e-12)})
    return result


def resolve_unique_numeric_branch(raw_rad, *, sign, offset_rad, lower_rad,
                                  upper_rad, uncertainty_rad):
    """Resolve one in-range branch without claiming physical evidence approval.

    The caller must separately establish that zero, sign, physical limits and
    uncertainty were reviewed.  A width below one turn, one candidate, and its
    entire uncertainty interval inside the limits are all mandatory.  This is
    the same arithmetic used by ``EpochAngleMap`` and the active preflight.
    """
    lower = _finite(lower_rad, "physical lower limit")
    upper = _finite(upper_rad, "physical upper limit")
    _need(lower < upper and upper - lower < TAU,
          "Physical interval must be narrower than one turn")
    candidates = _branch_candidates(raw_rad, sign, offset_rad, lower, upper,
                                    uncertainty_rad)
    _need(len(candidates) == 1,
          f"Expected one physical branch, got {len(candidates)}")
    _need(candidates[0]["whole_uncertainty_inside_limits"],
          "Angle uncertainty crosses physical limit")
    return dict(candidates[0])


def resolve_current_branch(raw_rad, calibration):
    """Bind only a unique branch supported by reviewed zero/sign/physical limits.

    A uniquely limited mechanical joint cannot have two physically valid full
    turns.  Thus a fresh motor-power epoch can be reconciled without asking the
    operator to repeat a no-full-revolution statement on every power cycle.
    Missing physical-limit evidence remains a blocker; URDF limits alone do not
    supply it.  The result remains a numerical diagnostic, not an output permit.
    """
    blockers = calibration.evidence_blockers()
    _need(not blockers, f"ID{calibration.motor_id}: " + ", ".join(blockers))
    try:
        return resolve_unique_numeric_branch(
            raw_rad, sign=calibration.sign, offset_rad=calibration.offset_rad,
            lower_rad=calibration.lower_rad, upper_rad=calibration.upper_rad,
            uncertainty_rad=calibration.uncertainty_rad)
    except AngleEvidenceError as error:
        raise AngleEvidenceError(f"ID{calibration.motor_id}: {error}") from error


def _snapshot(value):
    _need(type(value) is dict, "Snapshot must be an object")
    boot, epoch = value.get("boot_id"), value.get("motor_power_epoch")
    _need(type(boot) is str and bool(boot.strip()), "Boot identity required")
    _need(type(epoch) is str and epoch.strip() not in UNKNOWN_EPOCHS,
          "Explicit motor-power epoch required; Jetson boot alone is insufficient")
    _sha(value.get("epoch_evidence_sha256"), "motor-power epoch evidence")
    _need(value.get("uid_read_boot_id") == boot
          and value.get("uid_read_motor_power_epoch") == epoch,
          "Fresh UID read is not bound to this boot and motor-power epoch")
    _sha(value.get("uid_capture_sha256"), "UID capture hash")
    _sha(value.get("capture_sha256"), "angle capture hash")
    _need(value.get("motor_output_allowed") is False, "Only no-output snapshots accepted")
    stamp = value.get("monotonic_ns")
    _need(type(stamp) is int and 0 < stamp < 2**63, "Monotonic capture time required")
    raw, uids = value.get("raw_rad_by_id"), value.get("uids_by_id")
    keys = {str(mid) for mid in IDS}
    _need(type(raw) is dict and set(raw) == keys
          and type(uids) is dict and set(uids) == keys, "Exactly twelve axes required")
    result = dict(value)
    result["raw_rad_by_id"] = {key: _finite(raw[key], "raw angle") for key in keys}
    result["uids_by_id"] = {key: _uid(uids[key]) for key in keys}
    _need(len(set(uids.values())) == 12, "Duplicate motor UID")
    return result


class EpochAngleMap:
    """Fixed, twelve-axis branch for one motor-power epoch; no hardware access.

    A new epoch requires a new instance and a fresh UID capture.  Once initialized,
    readings use direct differences, never modulo.  A stale sample, changed UID,
    excessive step or failed range check permanently invalidates this instance.
    """

    def __init__(self, calibrations, snapshot, *, max_speed_rad_s,
                 noise_margin_rad, max_sample_gap_s):
        _need(isinstance(calibrations, Mapping)
              and set(calibrations) == set(IDS), "Twelve calibration objects required")
        for mid in IDS:
            _need(isinstance(calibrations[mid], AxisCalibration)
                  and calibrations[mid].motor_id == mid, "Calibration ID mismatch")
        self._calibrations = dict(calibrations)
        self._current = _snapshot(snapshot)
        self._speed = _finite(max_speed_rad_s, "maximum physical speed")
        self._margin = _finite(noise_margin_rad, "readout noise margin")
        self._gap = _finite(max_sample_gap_s, "maximum sample gap")
        _need(0 < self._speed and 0 <= self._margin < math.pi
              and 0 < self._gap and self._speed * self._gap + self._margin < math.pi,
              "Continuity bound must distinguish a half-turn discontinuity")
        self._aborted = False
        self._branches = {}
        for mid in IDS:
            axis, key = self._calibrations[mid], str(mid)
            _need(axis.uid == self._current["uids_by_id"][key], f"ID{mid}: UID changed")
            self._branches[mid] = resolve_current_branch(
                self._current["raw_rad_by_id"][key], axis)["turns"]
        self._values = self._convert(self._current)

    def _convert(self, snapshot):
        values = {}
        for mid in IDS:
            axis = self._calibrations[mid]
            q = (axis.sign * (snapshot["raw_rad_by_id"][str(mid)]
                              - self._branches[mid] * TAU) + axis.offset_rad)
            _need(q - axis.uncertainty_rad >= axis.lower_rad - 1e-12
                  and q + axis.uncertainty_rad <= axis.upper_rad + 1e-12,
                  f"ID{mid}: fixed-branch angle outside mechanical interval")
            values[str(mid)] = q
        return values

    def report(self):
        _need(not self._aborted, "Angle map was invalidated")
        return {"schema": "singularitydog.epoch-angle-map-diagnostic.v1",
                "boot_id": self._current["boot_id"],
                "motor_power_epoch": self._current["motor_power_epoch"],
                "epoch_evidence_sha256": self._current["epoch_evidence_sha256"],
                "capture_sha256": self._current["capture_sha256"],
                "uid_capture_sha256": self._current["uid_capture_sha256"],
                "turns_by_id": {str(mid): self._branches[mid] for mid in IDS},
                "model_rad_by_id": dict(self._values),
                "raw_rad_by_id": dict(self._current["raw_rad_by_id"]),
                "calibration_sha256_by_id": {
                    str(mid): self._calibrations[mid].calibration_sha256 for mid in IDS},
                "raw_angles_modified": False, "approved_for_runtime": False,
                "motor_output_available": False, "output_allowed": False}

    def observe(self, snapshot):
        _need(not self._aborted, "Angle map was invalidated")
        try:
            current = _snapshot(snapshot)
            for key in ("boot_id", "motor_power_epoch", "epoch_evidence_sha256",
                        "uids_by_id", "uid_capture_sha256"):
                _need(current[key] == self._current[key], f"Epoch binding changed: {key}")
            dt = (current["monotonic_ns"] - self._current["monotonic_ns"]) / 1e9
            _need(0 < dt <= self._gap, "Out-of-order or stale angle sample")
            _need(current["capture_sha256"] != self._current["capture_sha256"],
                  "Repeated angle capture")
            allowed_step = self._speed * dt + self._margin
            for mid in IDS:
                key = str(mid)
                step = current["raw_rad_by_id"][key] - self._current["raw_rad_by_id"][key]
                _need(abs(step) <= allowed_step,
                      f"ID{mid}: within-epoch discontinuity; modulo is not allowed")
            values = self._convert(current)
            self._current, self._values = current, values
            return self.report()
        except BaseException:
            self._aborted = True
            raise

    def raw_target_candidates(self, model_rad_by_id, *, max_delta_rad):
        """Numerical inverse on the current branch, with a caller-bounded delta.

        No enable, gain, frame or transport object can be created here.  Command
        encoding, RS05 range, collision and load limits remain separate checks.
        """
        _need(not self._aborted, "Angle map was invalidated")
        _need(type(model_rad_by_id) is dict
              and set(model_rad_by_id) == {str(mid) for mid in IDS},
              "Exactly twelve model targets required")
        ceiling = _finite(max_delta_rad, "target delta ceiling")
        _need(0 < ceiling < math.pi, "Target delta ceiling must be below half a turn")
        targets = {}
        for mid in IDS:
            key, axis = str(mid), self._calibrations[mid]
            q = _finite(model_rad_by_id[key], "model target")
            _need(q - axis.uncertainty_rad >= axis.lower_rad - 1e-12
                  and q + axis.uncertainty_rad <= axis.upper_rad + 1e-12,
                  f"ID{mid}: target outside mechanical interval")
            raw = (q - axis.offset_rad) / axis.sign + self._branches[mid] * TAU
            _need(abs(raw - self._current["raw_rad_by_id"][key]) <= ceiling + 1e-12,
                  f"ID{mid}: inverse target delta exceeds bound")
            targets[key] = raw
        return {"raw_target_candidates_rad_by_id": targets,
                "source_capture_sha256": self._current["capture_sha256"],
                "boot_id": self._current["boot_id"],
                "motor_power_epoch": self._current["motor_power_epoch"],
                "numerical_inverse_only": True, "approved_for_runtime": False,
                "motor_output_available": False, "output_allowed": False}


def fit_reference_observations(observations):
    """Fit sign and an offset interval from externally observed joint angles.

    Each observation contains the same UID/boot/power epoch, raw angle, external
    model-convention angle, an explicit angular error bound and source digest.
    No full turns are inferred within the measurement sequence.  A known-angle
    span of at least five degrees distinguishes direction from readout noise.
    The supplied error bounds must make exactly one of sign +/-1 feasible.
    """
    _need(type(observations) is list and len(observations) >= 2,
          "At least two physical reference observations required")
    reference, rows = observations[0], []
    _need(type(reference) is dict, "Observation object required")
    uid = _uid(reference.get("uid"))
    boot, epoch = reference.get("boot_id"), reference.get("motor_power_epoch")
    _need(type(boot) is str and bool(boot.strip()) and type(epoch) is str
          and epoch.strip() not in UNKNOWN_EPOCHS, "Known boot and motor epoch required")
    for row in observations:
        _need(type(row) is dict and row.get("uid") == uid
              and row.get("boot_id") == boot and row.get("motor_power_epoch") == epoch,
              "Observation identity or power epoch changed")
        _sha(row.get("source_sha256"), "physical observation source")
        _need(row.get("relative_output_shaft_observed") is True,
              "Relative output shaft must be observed, not whole-leg movement")
        method = row.get("physical_angle_method")
        _need(method in ("angle_gauge", "fixture", "calibrated_image", "visual_bounded"),
              "Explicit physical angle method required")
        raw = _finite(row.get("raw_rad"), "reference raw angle")
        q = _finite(row.get("model_rad"), "physical reference angle")
        error = _finite(row.get("uncertainty_rad"), "physical angle uncertainty")
        _need(0 < error < math.pi / 2, "A nonzero bounded physical-angle error is required")
        if rows:
            _need(abs(raw - rows[-1][0]) < math.pi,
                  "Within-epoch reference jump is ambiguous; do not wrap it")
        rows.append((raw, q, error))
    _need(len({row["source_sha256"] for row in observations}) == len(observations),
          "Reference observations must have independent sources")
    _need(max(row[1] for row in rows) - min(row[1] for row in rows) >= math.radians(5),
          "Physical reference span below five degrees")
    candidates = []
    for sign in (-1, 1):
        lower = max(q - sign * raw - error for raw, q, error in rows)
        upper = min(q - sign * raw + error for raw, q, error in rows)
        if lower <= upper:
            candidates.append({"sign_candidate": sign,
                               "offset_candidate_rad": (lower + upper) / 2,
                               "offset_interval_rad": [lower, upper],
                               "uncertainty_rad": (upper - lower) / 2})
    _need(len(candidates) == 1,
          f"Physical observations do not identify one consistent sign: {len(candidates)} candidates")
    return {**candidates[0], "status": "PHYSICAL_REFERENCE_FIT_REVIEW_REQUIRED",
            "uid": uid, "boot_id": boot, "motor_power_epoch": epoch,
            "source_sha256": [row["source_sha256"] for row in observations],
            "observations": len(rows), "approved_for_runtime": False,
            "motor_output_available": False, "output_allowed": False}


def audit_twelve_axes(calibrations, raw_rad_by_id, uids_by_id):
    """Batch a reusable zero/direction/limit checklist and branch arithmetic.

    Calibration evidence stays useful across boots and power cycles only for
    the same motor UID and assembly.  This screen cannot certify that assembly
    has not changed; any reinstallation needs a new calibration revision.
    """
    _need(set(calibrations) == set(IDS), "Twelve calibration objects required")
    _need(type(raw_rad_by_id) is dict and type(uids_by_id) is dict
          and set(raw_rad_by_id) == set(uids_by_id) == {str(mid) for mid in IDS},
          "Twelve current UID and raw-angle rows required")
    _need(len(set(uids_by_id.values())) == 12, "Duplicate current motor UID")
    rows = {}
    for mid in IDS:
        axis, key = calibrations[mid], str(mid)
        _need(isinstance(axis, AxisCalibration) and axis.motor_id == mid,
              "Calibration ID mismatch")
        _uid(uids_by_id[key])
        blockers = axis.evidence_blockers()
        if axis.uid != uids_by_id[key]:
            blockers.append("UID_CHANGED_RECAPTURE_ONLY_THIS_AXIS")
        candidates = equivalent_branch_candidates(raw_rad_by_id[key], axis)
        if len(candidates) != 1:
            blockers.append("NO_UNIQUE_PERIODIC_MODEL_ANGLE")
        elif not candidates[0]["whole_uncertainty_inside_limits"]:
            blockers.append("ANGLE_UNCERTAINTY_CROSSES_LIMIT")
        rows[key] = {"motor_id": mid,
                     "zero_reviewed": axis.zero_reviewed,
                     "direction_reviewed": axis.direction_reviewed,
                     "physical_limits_reviewed": axis.physical_limits_reviewed,
                     "identity_matches": axis.uid == uids_by_id[key],
                     "raw_rad_unmodified": raw_rad_by_id[key],
                     "candidate_sign": axis.sign,
                     "periodic_branch_candidates": candidates,
                     "evidence_ready_for_epoch_binding": not blockers,
                     "blockers": blockers}
    return {"schema": "singularitydog.twelve-angle-calibration-audit.v1",
            "status": ("EVIDENCE_READY_FOR_EPOCH_BINDING"
                       if all(not row["blockers"] for row in rows.values())
                       else "INCOMPLETE_PHYSICAL_EVIDENCE"),
            "rows_by_id": rows,
            "needs_zero_review_ids": [mid for mid in IDS if not rows[str(mid)]["zero_reviewed"]],
            "needs_direction_review_ids": [mid for mid in IDS if not rows[str(mid)]["direction_reviewed"]],
            "needs_physical_limit_review_ids": [mid for mid in IDS if not rows[str(mid)]["physical_limits_reviewed"]],
            "uid_changed_ids": [mid for mid in IDS if not rows[str(mid)]["identity_matches"]],
            "raw_angles_modified": False, "approved_for_runtime": False,
            "motor_output_available": False, "output_allowed": False}
