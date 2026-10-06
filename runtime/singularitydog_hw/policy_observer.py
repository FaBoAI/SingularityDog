"""Pure stateful policy observation; no device, network or motor-output path.

The caller supplies audited telemetry snapshots and a separate policy instance
for each fixed h=0 or h=1 diagnostic hypothesis. h is twelve load-history
inputs, not measured temperature or height. Caller warmup is followed by one
explicit reset per run. A blocked/missed tick permanently invalidates that run.
"""
from array import array
import copy
import hashlib
import json
import marshal
import math
from pathlib import Path
import struct
import time
import weakref

from . import policy_shadow as shadow
from .angle_branch_comparison import (IDS as BRANCH_IDS, MAX_STATIC_POSE_DELTA_RAD,
                                      StaticBranchComparison, TWO_PI)
from .event_snapshot import snapshot_event
from . import imu_accel_input_hypothesis as accel_hypotheses
from .imu_calibration_review import reviewed_acceleration

DT_NS = 20_000_000
RAW_IMU_CORRECTION_FLAGS = (
    "calibration_applied", "orientation_applied", "mount_rotation_applied",
    "gyro_bias_subtracted", "accel_bias_subtracted", "accel_scale_corrected",
    "mount_correction_applied", "gyro_bias_correction_applied")
_FLOAT32_MAX = float.fromhex("0x1.fffffep+127")
_EXPECTED_MOTOR_KEYS = frozenset((i, p) for i in range(1, 13)
                               for p in ("position", "velocity"))
_OWNERS = weakref.WeakKeyDictionary()
_ACCEL_HYPOTHESIS_CLASS = accel_hypotheses.AccelInputHypothesis
_ACCEL_PROVENANCE_METHOD = _ACCEL_HYPOTHESIS_CLASS.provenance
_ACCEL_PROVENANCE_PARSER = accel_hypotheses.strict_json
# Compare float32 policy output with the same representable endpoint, without
# adding a physical margin or clipping the diagnostic target itself.
_TARGET_LOWER = [struct.unpack("<f", struct.pack("<f", x))[0] for x in shadow.LOWER]
_TARGET_UPPER = [struct.unpack("<f", struct.pack("<f", x))[0] for x in shadow.UPPER]


class ObserverError(ValueError):
    """This diagnostic run cannot consume another tick without explicit reset."""


def _require(condition, message):
    if not condition:
        raise ObserverError(message)


def _vector(value, size, label):
    _require(isinstance(value, list) and len(value) == size
             and all(shadow.finite(x) for x in value), "Invalid " + label)
    return list(value)


def _stamp(value, label):
    _require(type(value) is int and 0 <= value < 2**63, "Invalid " + label)
    return value


def _raw_imu_corrections(value):
    # Original drivers predate these flags, so absence retains the raw-frame
    # contract. An explicit unknown/nonboolean value cannot establish it.
    for flag in RAW_IMU_CORRECTION_FLAGS:
        _require(value.get(flag, False) is False,
                 "Invalid raw IMU correction state: " + flag)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _snapshot_copy(snapshot):
    """Own the finite JSON input without arbitrary deepcopy dispatch/hooks.

    This only changes copying: _inputs still validates every dynamic field,
    timestamp and calibrated range, and the full input digest is recomputed.
    Oversized, cyclic or non-JSON inputs fail before model execution.
    """
    try:
        return snapshot_event(snapshot)
    except (TypeError, ValueError) as error:
        raise ObserverError("Invalid bounded JSON snapshot: " + str(error)) from error


def _provenance_copier(value):
    """Select the bounded JSON copier without narrowing static metadata input.

    Static validators historically allow JSON metadata beyond the event bounds
    (including numeric keys). Such metadata keeps its existing deepcopy path.
    Selection is once per fixed configuration; every result is still copied in
    the measured provenance section, with the bounded checks included there.
    """
    try:
        snapshot_event(value)
    except (TypeError, ValueError):
        return copy.deepcopy
    return snapshot_event


def _frozen_accel_provenance(correction):
    """Cache only the original frozen hypothesis's bounded, owned JSON tree.

    Custom/subclass provenance remains dynamic. Unusual JSON or a parse failure
    keeps the existing per-tick path, including its original error timing.
    The cache never substitutes for correct() or its loader/norm checks.
    """
    if (type(correction) is not _ACCEL_HYPOTHESIS_CLASS
            or accel_hypotheses.AccelInputHypothesis is not _ACCEL_HYPOTHESIS_CLASS
            or getattr(correction.provenance, "__func__", None) is not _ACCEL_PROVENANCE_METHOD
            or accel_hypotheses.strict_json is not _ACCEL_PROVENANCE_PARSER
            or type(correction._provenance_json) is not str):
        return None
    raw, proof = correction._provenance_json, correction._proof
    try:
        blob = marshal.dumps(snapshot_event(correction.provenance()))
    except (TypeError, ValueError, RecursionError):
        return None
    return correction, raw, proof, blob


def _mount(candidate):
    if isinstance(candidate, (str, Path)):
        return copy.deepcopy(shadow.load_imu_mount_candidate(candidate))
    _require(isinstance(candidate, dict), "An explicit IMU mount candidate is required")
    candidate = copy.deepcopy(shadow.validate_imu_mount_candidate(candidate))
    return {**candidate, "source": {"kind": "inline_candidate",
                                    "canonical_json_sha256": _digest(candidate)}}


def _bias(candidate):
    """Accept an explicitly supplied A-fit report only as an unverified hypothesis."""
    if candidate is None:
        return {"applied_as_hypothesis": False, "bias_sensor_rad_s": [0., 0., 0.],
                "measured_bias_verified": False, "approved_for_runtime": False,
                "source": None}
    c = copy.deepcopy(candidate)
    _require(isinstance(c, dict) and c.get("schema_version") == 1
             and c.get("kind") == "fixed_mount_baseline"
             and c.get("status") == "GYRO_BIAS_CANDIDATE"
             and c.get("frame") == "sensor" and c.get("axis_order") == ["x", "y", "z"]
             and c.get("gyro_bias_candidate_eligible") is True
             and c.get("operator_confirmed_stationary") is True
             and c.get("approved_for_runtime") is False
             and c.get("automatically_applied") is False
             and c.get("mount_rotation_applied") is False,
             "Require an explicit unapproved sensor-frame A-fit gyro candidate")
    bias = _vector(c.get("gyro_bias_candidate_rad_s"), 3, "gyro bias")
    fitted = _vector(c.get("captures", {}).get("a", {}).get("gyro_mean_rad_s"), 3,
                     "capture A gyro mean")
    _require(bias == fitted, "Gyro candidate must equal capture A mean")
    provenance = c.get("provenance")
    _require(isinstance(provenance, dict), "Missing A/B bias provenance")
    for name in ("a", "b"):
        p = provenance.get(name)
        _require(isinstance(p, dict), "Missing bias capture provenance")
        for key in ("summary_sha256", "events_sha256"):
            value = p.get(key)
            _require(isinstance(value, str) and len(value) == 64
                     and all(x in "0123456789abcdef" for x in value), "Invalid bias source hash")
    _require(provenance["a"]["events_sha256"] != provenance["b"]["events_sha256"],
             "Bias A/B captures must be distinct")
    return {"applied_as_hypothesis": True, "bias_sensor_rad_s": bias,
            "fit_capture": "a", "measured_bias_verified": False,
            "approved_for_runtime": False, "source": copy.deepcopy(provenance),
            "candidate_canonical_json_sha256": _digest(c),
            "source_flags": {k: c.get(k) for k in (
                "status", "operator_confirmed_stationary", "stationarity_verified_by_software",
                "approved_for_runtime", "calibration_verified", "absolute_level_verified")}}


def _power_epoch_branch_overlay(candidate, calibration):
    """Freeze one reviewed static comparison for observation input only.

    The comparison constructor checked caller-attested Off/On, quiet disabled
    zero-current, no-full-turn and fresh UID evidence. Those assertions do not
    prove physical STOP. A tick still needs an explicit binding to that capture
    and power epoch.
    """
    if candidate is None:
        return None
    _require(type(candidate) is StaticBranchComparison,
             "Require a reviewed StaticBranchComparison for branch overlay")
    try:
        binding = candidate.validated_current_binding()
        comparison = candidate.comparison()
    except ValueError as error:
        raise ObserverError("Invalid static branch comparison: " + str(error)) from error
    _require(binding["motor_output_allowed"] is False
             and binding["approved_for_runtime"] is False
             and comparison["motor_output_allowed"] is False
             and comparison["approved_for_runtime"] is False
             and comparison["raw_targets_changed"] is False
             and comparison["status"] == "STATIC_BRANCH_COMPARISON_ONLY",
             "Branch review must be diagnostic only")
    _require(comparison["current_boot_id"] == binding["boot_id"]
             and comparison["current_motor_power_epoch"] == binding["motor_power_epoch"]
             and comparison["current_capture_sha256"] == binding["capture_sha256"],
             "Branch capture binding mismatch")
    _require(binding["uids_by_id"] == calibration["identities"],
             "Branch UIDs do not match calibrated motor identities")
    turns = binding["reviewed_branch_turns_by_id"]
    _require(type(turns) is dict and bool(turns) and set(turns).issubset(BRANCH_IDS)
             and all(type(k) is int and abs(k) <= 1 for k in turns.values()),
             "Invalid reviewed branch turns")
    rows = comparison["rows"]
    _require(type(rows) is dict and set(rows) == set(BRANCH_IDS)
             and set(binding["raw_rad_by_id"]) == set(BRANCH_IDS),
             "Incomplete static branch comparison")
    for mid in BRANCH_IDS:
        row = rows[mid]
        _require(row["uid"] == binding["uids_by_id"][mid]
                 and row["current_raw_rad"] == binding["raw_rad_by_id"][mid]
                 and row["branch_reviewed"] is (mid in turns)
                 and row["branch_turns_for_comparison"] == turns.get(mid, 0),
                 "Static comparison and validated binding disagree")
    return {"binding": copy.deepcopy(binding),
            "reference_raw_rad_by_id": {mid: rows[mid]["reference_raw_rad"] for mid in BRANCH_IDS}}


def _branch_observation(snapshot, values, overlay, previous_raw):
    """Check a tick against the same quiet capture; return comparison copies."""
    flags = snapshot.get("source_flags")
    _require(type(flags) is dict, "Branch overlay requires snapshot source flags")
    source = flags.get("power_epoch_branch_capture")
    _require(type(source) is dict, "Missing power-epoch branch capture evidence")
    binding = overlay["binding"]
    for key in ("boot_id", "motor_power_epoch", "capture_sha256",
                "uid_capture_sha256", "evidence_sha256", "uids_by_id", "raw_rad_by_id"):
        _require(source.get(key) == binding[key], "Branch capture evidence mismatch: " + key)
    _require(source.get("uid_read_boot_id") == binding["boot_id"]
             and source.get("uid_read_motor_power_epoch") == binding["motor_power_epoch"],
             "Branch UIDs were not freshly read in the bound boot and motor power epoch")
    _require(source.get("motor_output_allowed") is False
             and source.get("motor_supply_off_on_observed") is True,
             "Branch capture needs read-only motor power-cycle evidence")
    disabled = source.get("disabled_zero_current_by_id")
    _require(type(disabled) is dict and set(disabled) == set(BRANCH_IDS)
             and all(disabled[mid] is True for mid in BRANCH_IDS),
             "Branch capture needs twelve disabled zero-current assertions")
    raw = {mid: values[(int(mid), "position")] for mid in BRANCH_IDS}
    adjusted = {}
    turns = binding["reviewed_branch_turns_by_id"]
    for mid in BRANCH_IDS:
        current = raw[mid]
        if previous_raw is not None:
            _require(abs(current-previous_raw[mid]) <= MAX_STATIC_POSE_DELTA_RAD,
                     "ID" + mid + " within-epoch raw angle discontinuity")
        _require(abs(current-binding["raw_rad_by_id"][mid]) <= MAX_STATIC_POSE_DELTA_RAD,
                 "ID" + mid + " differs from the reviewed static capture")
        equivalent = current-turns.get(mid, 0)*TWO_PI
        if mid in turns:
            _require(abs(equivalent-overlay["reference_raw_rad_by_id"][mid])
                     <= MAX_STATIC_POSE_DELTA_RAD,
                     "ID" + mid + " no longer matches the reviewed static reference")
        adjusted[mid] = equivalent
    provenance = {"kind": "power_epoch_static_branch_observation_only",
                  "formula": "q_model = sign * (raw - branch_turns * 2*pi) + offset",
                  "reviewed_capture_binding": copy.deepcopy(binding),
                  "raw_position_rad_by_id": dict(raw),
                  "comparison_position_rad_by_id": adjusted,
                  "static_pose_max_delta_rad": MAX_STATIC_POSE_DELTA_RAD,
                  "motor_output_allowed": False, "approved_for_runtime": False}
    return adjusted, provenance, raw


def _tensor_row(value, count, label):
    rows = value.detach().cpu().tolist()
    _require(isinstance(rows, list) and len(rows) == 1, "Invalid " + label + " batch")
    return _vector(rows[0], count, label)


class _ConsumeProfile:
    """Host section timings only; never a deadline or hardware approval."""

    def __init__(self, clock):
        self.clock = clock
        self.started = self.last = _stamp(clock(), "profiling monotonic clock")
        self.stage = "snapshot_copy"
        self.durations = {}

    def next(self, stage):
        now = _stamp(self.clock(), "profiling monotonic clock")
        _require(now >= self.last, "Profiling monotonic clock moved backward")
        self.durations[self.stage] = now-self.last
        self.last, self.stage = now, stage

    def finish(self, *, complete):
        failed_stage = None if complete else self.stage
        self.next(None)
        return {"kind": "host_consume_sections", "complete": complete,
                "started_monotonic_ns": self.started, "finished_monotonic_ns": self.last,
                "measured_total_ns": self.last-self.started,
                "durations_ns": dict(self.durations), "failed_stage": failed_stage,
                "includes_instrumentation_overhead": True,
                "model_call_is_host_elapsed_not_device_kernel_timing": True,
                "provenance_serialization_includes_canonical_json_hashing": True,
                "excludes": ["profile_metadata_build_and_tick_commit", "caller_json_serialization_and_io",
                             "device_acquisition", "motor_command_transmission"],
                "wall_clock_timing_verified": False, "output_allowed": False}


class StatefulPolicyObserver:
    """Finite, ordered 50Hz model ticks with diagnostic-only candidate inputs.

    No warmup is performed here. prepare_run(..., warmup_completed=True) resets
    policy state without assigning a deadline; arm_run(first_tick_ns) assigns
    the schedule after preparation. reset_run combines both for replay callers.
    Tick values are model deadlines, not a claim of measured wall time.

    CPU buffer reuse is opt-in, matching the live adapter's input storage. Such
    policies must own any retained recurrent state rather than retain input views.
    Returned diagnostic records still own their lists and snapshot provenance.
    """

    def __init__(self, policy, calibration, *, imu_mount_candidate,
                 h_hypothesis, command, max_ticks, max_age_ns, max_spread_ns,
                 torch_module=None, gyro_bias_candidate=None,
                 profile_consume=False, monotonic_ns=None,
                 power_epoch_branch_comparison=None, measured_diagnostic_ticks=False,
                 reuse_input_buffers=False, apply_reviewed_accel_calibration=False,
                 accel_input_hypothesis=None):
        _require(type(reuse_input_buffers) is bool, "reuse_input_buffers must be boolean")
        _require(type(measured_diagnostic_ticks) is bool, "measured_diagnostic_ticks must be boolean")
        self._measured_diagnostic_ticks = measured_diagnostic_ticks
        _require(type(profile_consume) is bool, "profile_consume must be an explicit boolean")
        _require(monotonic_ns is None or callable(monotonic_ns),
                 "monotonic_ns must be a callable clock")
        self._profile_consume = profile_consume
        self._profile_clock = time.monotonic_ns if monotonic_ns is None else monotonic_ns
        self._last_consume_profile = None
        _require(type(h_hypothesis) in (int, float) and h_hypothesis in (0, 1),
                 "Explicit h hypothesis must be 0 or 1; it is not a measurement")
        _require(type(max_ticks) is int and 1 <= max_ticks <= 30_000, "max_ticks must be 1..30000")
        self._calibration = copy.deepcopy(calibration)
        self._rows = shadow.validate_calibration(self._calibration)
        self._branch_overlay = _power_epoch_branch_overlay(power_epoch_branch_comparison,
                                                           self._calibration)
        self._mount = _mount(imu_mount_candidate)
        self._bias = _bias(gyro_bias_candidate)
        self._accel_calibration = reviewed_acceleration(gyro_bias_candidate,
            self._mount["R_body_from_sensor"], enabled=apply_reviewed_accel_calibration)
        _require(accel_input_hypothesis is None or not apply_reviewed_accel_calibration,
                 "Acceleration hypothesis and reviewed calibration are mutually exclusive")
        self._accel_input_hypothesis = None
        if accel_input_hypothesis is not None:
            from .imu_accel_input_hypothesis import load_accel_input_hypothesis
            self._accel_input_hypothesis = load_accel_input_hypothesis(
                accel_input_hypothesis, self._mount["R_body_from_sensor"])
            self._accel_calibration = self._accel_input_hypothesis
        self._accel_provenance_cache = _frozen_accel_provenance(self._accel_calibration)
        # Private validated configuration is fixed for the observer's lifetime.
        # Only its digest/lookup plan is reused; live snapshot values and their
        # complete canonical digest are still recomputed for every tick.
        self._calibration_sha256 = _digest(self._calibration)
        self._calibration_source_flags = {
            k: copy.deepcopy(v) for k, v in self._calibration.items()
            if k not in ("identities", "candidates")}
        self._static_provenance_copiers = tuple(_provenance_copier(value) for value in
            (self._calibration_source_flags, self._mount, self._bias))
        # The bounded built-in JSON branches are fixed configuration. Freeze an
        # alias-free validated copy once, then decode a fresh owned tree for
        # every returned record. Legacy metadata keeps its deepcopy fallback.
        self._static_provenance_blobs = tuple(
            marshal.dumps(snapshot_event(value)) if copier is snapshot_event else None
            for value, copier in zip((self._calibration_source_flags, self._mount, self._bias),
                                     self._static_provenance_copiers))
        self._can_order = tuple(shadow.CAN_ORDER)
        self._ordered_calibration = tuple(
            (i, self._rows[i]["sign_candidate"], self._rows[i]["offset_candidate_rad"])
            for i in self._can_order)
        self._rotation = tuple(tuple(row) for row in self._mount["R_body_from_sensor"])
        self._gyro_bias_values = tuple(self._bias["bias_sensor_rad_s"])
        self._command = _vector(command, 3, "explicit diagnostic command")
        vx, vy, yaw = self._command
        _require(-.12 <= vx <= shadow.OPTIONS["forward_command_limit"] and abs(vy) <= .12
                 and abs(yaw) <= .25 and not (vx and vy) and not ((vx or vy) and yaw),
                 "Command outside registered cardinal/pure-yaw/stop domain")
        self._max_age_ns = _stamp(max_age_ns, "max age")
        self._max_spread_ns = _stamp(max_spread_ns, "max spread")
        if torch_module is None:
            import torch as torch_module
        self._torch = torch_module
        self._policy = policy
        self._input_buffers = None
        self._input_tensors = None
        if reuse_input_buffers:
            _require(callable(getattr(self._torch, "frombuffer", None)) and array("f").itemsize == 4,
                     "CPU float buffer reuse requires torch.frombuffer and 32-bit floats")
            self._input_buffers = tuple(array("f", [0.]*n) for n in (3, 3, 3, 12, 12, 12))
            with self._torch.inference_mode():
                self._input_tensors = tuple(
                    self._torch.frombuffer(buf, dtype=self._torch.float32).reshape(1, len(buf))
                    for buf in self._input_buffers)
        owner = _OWNERS.get(policy)
        _require(owner is None or owner() is None, "Policy instance already belongs to an observer")
        _OWNERS[policy] = weakref.ref(self)
        self._h = float(h_hypothesis)
        self.max_ticks = max_ticks
        self.run_number = 0
        self.reset_count = 0
        self.status = "NOT_STARTED"
        self.failure = None
        self.ticks_completed = 0
        self._next_tick_ns = None
        self._last_sources = {}
        self._last_branch_raw = None

    def _flags(self):
        return {"motor_output_available": False, "output_allowed": False,
                "approved_for_runtime": False, "live_50hz_verified": False,
                "calibration_verified": False, "gravity_fusion_verified": False,
                "raw_driver_axes_verified": False, "sensor_alignment_verified": False,
                "gyro_bias_verified": False,
                "fresh_identity_match_verified": False, "wall_clock_timing_verified": False,
                "h_measured": False, "h_hypothesis": self._h,
                "h_semantics": "fixed twelve-joint load-history sensitivity hypothesis",
                "independent_cold_reset": False, "dt_s": .02}

    def _copy_static_provenance(self, index, value):
        copier = self._static_provenance_copiers[index]
        blob = self._static_provenance_blobs[index]
        return marshal.loads(blob) if blob is not None and copier is snapshot_event else copier(value)

    def _copy_accel_provenance(self):
        correction = self._accel_calibration
        cache = self._accel_provenance_cache
        if cache is not None:
            source, raw, proof, blob = cache
            if (correction is source
                    and type(correction) is _ACCEL_HYPOTHESIS_CLASS
                    and accel_hypotheses.AccelInputHypothesis is _ACCEL_HYPOTHESIS_CLASS
                    and getattr(correction.provenance, "__func__", None) is _ACCEL_PROVENANCE_METHOD
                    and accel_hypotheses.strict_json is _ACCEL_PROVENANCE_PARSER
                    and correction._provenance_json is raw and correction._proof is proof):
                return marshal.loads(blob)
        return correction.provenance()

    def summary(self):
        result = {**self._flags(), "status": self.status, "run_number": self.run_number,
                "ticks_completed": self.ticks_completed, "ticks_requested": self.max_ticks,
                "reset_count": self.reset_count, "failure": self.failure,
                "incomplete": self.status != "COMPLETE_NO_OUTPUT_DIAGNOSTIC"}
        if self._profile_consume:
            result["last_consume_profile"] = copy.deepcopy(self._last_consume_profile)
        if self._accel_input_hypothesis is not None:
            result["accel_input_hypothesis"] = self._accel_input_hypothesis.provenance()
        return result

    def prepare_run(self, *, warmup_completed):
        """Reset exactly once before the caller arms its wall-clock schedule."""
        _require(self.status not in ("ACTIVE", "PREPARED"),
                 "Finish or invalidate the active/prepared run before resetting")
        _require(warmup_completed is True, "Caller warmup must be explicitly completed before reset")
        self.run_number += 1
        self.ticks_completed = 0
        self.failure = None
        self._next_tick_ns = None
        self._last_sources = {}
        self._last_branch_raw = None
        self._last_consume_profile = None
        self.status = "INCOMPLETE"
        try:
            with self._torch.inference_mode():
                self._policy.reset(self._torch.tensor([0], dtype=self._torch.long))
            self.reset_count += 1
            self.status = "PREPARED"
        except BaseException as error:
            self.failure = "Reset failed: " + type(error).__name__
            raise
        return self.summary()

    def arm_run(self, first_tick_ns):
        """Assign the first deadline without resetting or running the policy."""
        _require(self.status == "PREPARED", "Prepare exactly once before arming")
        _stamp(first_tick_ns, "first tick")
        self._next_tick_ns = first_tick_ns
        self.status = "ACTIVE"
        return self.summary()

    def reset_run(self, first_tick_ns, *, warmup_completed):
        """Compatibility API for callers that already have a replay time axis."""
        _stamp(first_tick_ns, "first tick")
        self.prepare_run(warmup_completed=warmup_completed)
        return self.arm_run(first_tick_ns)

    def invalidate(self, reason):
        self.status = "INCOMPLETE"
        self.failure = str(reason)
        return self.summary()

    def finish(self):
        if self.status == "ACTIVE":
            self.status = ("COMPLETE_NO_OUTPUT_DIAGNOSTIC" if self.ticks_completed == self.max_ticks
                           else "INCOMPLETE")
            if self.status == "INCOMPLETE":
                self.failure = "Run ended before all requested ticks"
        return self.summary()

    def consume(self, snapshot):
        """Consume one tick; optional host profiling never relaxes validation.

        Profile sections are contiguous and include marker overhead. Canonical
        provenance JSON hashing is measured here; JSON encoding/writing of the
        returned record is the caller's separate responsibility. A CUDA policy
        may launch asynchronous work in model_call and synchronize during output
        conversion, so these are host durations, not GPU kernel timings.
        """
        _require(self.status == "ACTIVE", "Run inactive or invalid; explicit reset_run required")
        profile = None
        try:
            _require(self.ticks_completed < self.max_ticks, "Tick budget exhausted")
            if self._profile_consume:
                self._last_consume_profile = None
                profile = _ConsumeProfile(self._profile_clock)
            snapshot = _snapshot_copy(snapshot)
            if profile is not None:
                profile.next("source_validation")
            inputs, provenance, selected_sources, branch_raw = self._inputs(snapshot, profile)
            if profile is not None:
                profile.next("tensor_conversion")
            # Finite JSON/Python doubles can overflow when narrowed to the
            # policy's float32 inputs. Reject before touching recurrent state,
            # consistently for newly allocated and reused input tensors.
            _require(all(abs(x) <= _FLOAT32_MAX for row in inputs for x in row),
                     "Policy input is not finite-representable in float32")
            with self._torch.inference_mode():
                if self._input_buffers is None:
                    tensors = tuple(self._torch.tensor([x], dtype=self._torch.float32) for x in inputs)
                else:
                    # Float32 conversion and all 45 writes stay in this measured
                    # section. Diagnostic input/result lists never borrow these buffers.
                    for buf, row in zip(self._input_buffers, inputs):
                        for index, value in enumerate(row):
                            buf[index] = value
                    tensors = self._input_tensors
                if profile is not None:
                    profile.next("model_call")
                target = self._policy(*tensors)
                if profile is not None:
                    profile.next("output_conversion_validation")
                target = _tensor_row(target, 12, "target")
                actor = _tensor_row(self._policy.last_actor_output, 12, "actor output")
                observation = _tensor_row(self._policy.last_observation, 74, "observation")
            _require(all(lo <= q <= hi for q, lo, hi in zip(target, _TARGET_LOWER, _TARGET_UPPER)),
                     "Policy target outside registered joint range")
            if profile is not None:
                profile.next("result_build")
            result = {**self._flags(), "status": "TICK_OBSERVED_NO_OUTPUT",
                    "run_number": self.run_number, "tick_index": self.ticks_completed,
                    "tick_ns": snapshot["tick_ns"], "observation74": observation,
                    "actor_residual12": actor, "q_target_rad_diagnostic_only": target,
                    "inputs": dict(zip(("gyro_body_rad_s", "gravity_body_unit", "command",
                                         "q_model_rad", "dq_model_rad_s", "h_hypothesis12"), inputs)),
                    "provenance": provenance}
            if profile is not None:
                self._last_consume_profile = profile.finish(complete=True)
                # The fixed successful profile has only these two mutable
                # components. Keep result ownership without recursive dispatch.
                result["consume_profile"] = {
                    **self._last_consume_profile,
                    "durations_ns": dict(self._last_consume_profile["durations_ns"]),
                    "excludes": list(self._last_consume_profile["excludes"])}
            self._last_sources = selected_sources
            self._last_branch_raw = branch_raw
            self.ticks_completed += 1
            self._next_tick_ns = (snapshot["tick_ns"]+1 if self._measured_diagnostic_ticks
                                  else self._next_tick_ns+DT_NS)
            if self._measured_diagnostic_ticks:
                result["timing_scope"] = "actual acquisition completion; model dt remains 20ms; diagnostic only"
            return result
        except BaseException as error:
            if profile is not None:
                try:
                    self._last_consume_profile = profile.finish(complete=False)
                except BaseException as timing_error:
                    # A broken diagnostic clock must not mask the original
                    # guard/model failure or turn it into an accepted tick.
                    self._last_consume_profile = {
                        "kind": "host_consume_sections", "complete": False,
                        "measurement_error": type(timing_error).__name__ + ": " + str(timing_error),
                        "wall_clock_timing_verified": False, "output_allowed": False}
            self.invalidate(type(error).__name__ + ": " + str(error))
            raise

    def _inputs(self, snapshot, profile=None):
        _require(isinstance(snapshot, dict), "Require DiagnosticSnapshot.as_dict()")
        _require(snapshot.get("status") == "DIAGNOSTIC_READY"
                 and snapshot.get("output_allowed") is False
                 and snapshot.get("blocked_reasons") == [], "Blocked or non-diagnostic snapshot")
        tick = _stamp(snapshot.get("tick_ns"), "snapshot tick")
        if self._measured_diagnostic_ticks:
            _require(tick >= self._next_tick_ns, "Repeated/backward measured diagnostic tick")
        else:
            _require(tick == self._next_tick_ns, "Missed, repeated or out-of-order 20ms tick; no catchup")
        _require(type(snapshot.get("max_age_ns")) is int
                 and snapshot["max_age_ns"] == self._max_age_ns
                 and type(snapshot.get("max_spread_ns")) is int
                 and snapshot["max_spread_ns"] == self._max_spread_ns,
                 "Snapshot limits differ from the fixed observer profile")
        motors = snapshot.get("motors")
        _require(isinstance(motors, list) and len(motors) == 24, "Missing 24 motor values")
        values, selected_sources = {}, {}
        oldest = latest = earliest_receive = None
        for row in motors:
            _require(isinstance(row, dict), "Invalid motor observation")
            mid, parameter = row.get("motor_id"), row.get("parameter")
            _require(type(mid) is int and 1 <= mid <= 12
                     and isinstance(parameter, str) and parameter in ("position", "velocity"),
                     "Invalid motor key")
            key = (mid, parameter)
            _require(key not in values, "Duplicate motor key")
            _require(row.get("unit") == ("rad" if parameter == "position" else "rad_s")
                     and shadow.finite(row.get("value")), "Invalid motor SI value")
            start = _stamp(row.get("request_ns"), "motor request")
            end = _stamp(row.get("received_ns"), "motor receive")
            _require(start <= end <= tick, "Noncausal motor observation")
            _require(type(row.get("age_upper_bound_ns")) is int
                     and row["age_upper_bound_ns"] == tick-start, "Invalid motor age")
            values[key] = row["value"]
            oldest = start if oldest is None else min(oldest, start)
            latest = end if latest is None else max(latest, end)
            earliest_receive = end if earliest_receive is None else min(earliest_receive, end)
            selected_sources[key] = (start, end, row["value"])
        _require(values.keys() == _EXPECTED_MOTOR_KEYS, "Missing motor input; no zero filling")
        imu = snapshot.get("imu")
        _require(isinstance(imu, dict) and imu.get("frame") == "raw_sensor", "Missing raw sensor IMU")
        _raw_imu_corrections(imu)
        accel = _vector(imu.get("accel_m_s2"), 3, "raw acceleration")
        gyro = _vector(imu.get("gyro_rad_s"), 3, "raw gyro")
        start = _stamp(imu.get("read_started_ns"), "IMU read start")
        end = _stamp(imu.get("read_finished_ns"), "IMU read finish")
        _require(start <= end <= tick, "Noncausal IMU observation")
        _require(type(imu.get("age_upper_bound_ns")) is int
                 and imu["age_upper_bound_ns"] == tick-start, "Invalid IMU age")
        oldest = min(oldest, start)
        latest = max(latest, end)
        earliest_receive = min(earliest_receive, end)
        selected_sources["imu"] = (start, end, (tuple(accel), tuple(gyro)))
        for key, current in selected_sources.items():
            previous = self._last_sources.get(key)
            if previous is None:
                continue
            if current[:2] == previous[:2]:
                _require(current[2] == previous[2], "Held source timestamps changed values")
            else:
                _require(current[0] > previous[0] and current[1] > previous[1],
                         "Source acquisition/read timestamps moved backward or partly repeated")
        age = tick-oldest
        spread = latest-oldest
        receive_spread = latest-earliest_receive
        for key, actual in (("oldest_observation_age_ns", age), ("acquisition_spread_ns", spread),
                            ("receive_spread_ns", receive_spread)):
            _require(type(snapshot.get(key)) is int and snapshot[key] == actual,
                     "Snapshot timing summary mismatch: " + key)
        _require(age <= self._max_age_ns and spread <= self._max_spread_ns,
                 "Stale observation or excessive acquisition spread")
        if profile is not None:
            profile.next("input_conversion")
        branch_provenance, branch_raw = None, None
        if self._branch_overlay is not None:
            adjusted, branch_provenance, branch_raw = _branch_observation(
                snapshot, values, self._branch_overlay, self._last_branch_raw)
            q = [sign*adjusted[str(i)]+offset
                 for i, sign, offset in self._ordered_calibration]
        else:
            q = [sign*values[(i, "position")]+offset
                 for i, sign, offset in self._ordered_calibration]
        dq = [sign*values[(i, "velocity")] for i, sign, _ in self._ordered_calibration]
        _require(all(shadow.finite(x) for x in q+dq), "Nonfinite calibrated input")
        _require(all(lo <= value <= hi for value, lo, hi in zip(q, shadow.LOWER, shadow.UPPER)),
                 "Calibrated position outside registered joint range; no clipping")
        rotation = self._rotation
        corrected_gyro = [g-b for g, b in zip(gyro, self._gyro_bias_values)]
        norm = math.hypot(*accel)
        _require(math.isfinite(norm) and norm > 1e-9, "Invalid acceleration gravity-proxy norm")
        corrected_accel, corrected_norm = (accel, norm) if self._accel_calibration is None else self._accel_calibration.correct(accel)
        accel_body = [sum(r[j]*corrected_accel[j] for j in range(3)) for r in rotation]
        gyro_body = [sum(r[j]*corrected_gyro[j] for j in range(3)) for r in rotation]
        gravity = [-x/corrected_norm for x in accel_body]
        _require(all(shadow.finite(x) for x in gyro_body+gravity), "Nonfinite corrected IMU input")
        if profile is not None:
            profile.next("provenance_serialization")
        source_flags = snapshot.get("source_flags", {})
        _require(isinstance(source_flags, dict), "Invalid snapshot source flags")
        provenance = {"snapshot_status": snapshot["status"], "snapshot_output_allowed": False,
            # _snapshot_copy independently owns every mutable descendant; no
            # snapshot container is retained by the observer after this tick.
            "snapshot_source_flags": source_flags,
            "snapshot_canonical_json_sha256": _digest(snapshot),
            "oldest_observation_age_ns": age, "acquisition_spread_ns": spread,
            "receive_spread_ns": receive_spread, "max_age_ns": self._max_age_ns,
            "max_spread_ns": self._max_spread_ns,
            "calibration_canonical_json_sha256": self._calibration_sha256,
            "calibration_source_flags": self._copy_static_provenance(0, self._calibration_source_flags),
            "identity_binding": "calibration manifest syntax only; fresh UID matching is upstream and unverified here",
            "imu_mount_candidate": self._copy_static_provenance(1, self._mount),
            "gyro_bias_hypothesis": self._copy_static_provenance(2, self._bias),
            "raw_accel_m_s2": accel, "raw_gyro_rad_s": gyro,
            "raw_accel_norm_m_s2": norm, "raw_accel_norm_relative_deviation": norm/9.80665-1.,
            "accel_bias_subtracted": self._accel_calibration is not None,
            "accel_scale_corrected": self._accel_calibration is not None,
            "gravity_source": "negative normalized specific force; hypothesis only, not validated fusion",
            "command_source": "explicit configured diagnostic command",
            "model_can_order_candidate": list(self._can_order)}
        if self._accel_calibration is not None:
            provenance["corrected_accel_sensor_m_s2"] = list(corrected_accel)
            provenance["corrected_accel_norm_m_s2"] = corrected_norm
            key = ("accel_input_hypothesis" if self._accel_input_hypothesis is not None
                   else "reviewed_accel_calibration")
            provenance[key] = self._copy_accel_provenance()
        if branch_provenance is not None:
            provenance["power_epoch_branch_overlay"] = branch_provenance
        return ((gyro_body, gravity, list(self._command), q, dq, [self._h]*12),
                provenance, selected_sources, branch_raw)
