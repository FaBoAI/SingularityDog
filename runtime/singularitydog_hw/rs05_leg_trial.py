"""Supported, supervised three-joint RS05 leg trial; not a pose or RL controller.

All three targets share a four-second five-degree ramp and one-second hold.
Directions are explicit raw motor-coordinate signs. Stop removes active control.
Physical watchdog latency and total torque limits remain unverified.
Pre-enable stationarity is a single fixed observation window, not proof of
absolute rest: sub-tolerance creep and motion aliased by sampling can escape it.
"""
import argparse
from dataclasses import asdict
import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from .can_readonly import ATParser, decode_reply, matches, read_request
from .rs05_trial_protocol import (POSITION_MAX, POSITION_MIN, Type2Feedback, TrialPhase, decode_type2, enable_request,
    motion_request, stop_request, watchdog_setup_request)
from .rs05_joint_trial import check_feedback, step5_jog_offset
from .position_response_evidence import load_position_response_evidence
from .current_hold_review import (PROFILE as CURRENT_HOLD_PROFILE,
                                 load_current_hold_review, validated_reviewed_ids)

LEGS = {"FR": (1, 2, 3), "FL": (4, 5, 6), "RR": (7, 8, 9), "RL": (10, 11, 12)}
DURATION_S, ACTIVE_BUDGET_S, CYCLE_S = 5., 6., .05
MAX_DRIFT_RAD = math.radians(7)
MIN_TX_INTERVAL_S = .005  # Provisional adapter pacing; physical reliability is not verified.
SETTLED_COUNT, SETTLED_PERIOD_S = 21, .1
SETTLED_LIMITS = {"position_range_rad": .002, "abs_OLS_slope_rad_s": .002,
                  "velocity_RMS_rad_s": .05, "minimum_span_s": 1.9,
                  "maximum_span_s": 2.3, "maximum_gap_s": .15,
                  "maximum_center_drift_rad": .02}

POSITION_LIMITS = {**SETTLED_LIMITS, "position_range_rad": .001,
                   "abs_OLS_slope_rad_s": .0005,
                   "tail_position_range_rad": .0008, "tail_abs_OLS_slope_rad_s": .001,
                   # The Type2 velocity estimate can retain a small signed bias
                   # while the independently measured position remains still.
                   "abs_velocity_mean_rad_s": .05, "tail_count": 6,
                   "tail_minimum_span_s": .45, "tail_maximum_span_s": .60}
FULLBODY_POSITION_PROFILE = "position-v2-all"
# The full-body hold has a narrower signed-velocity gate than the existing
# single-leg position-v2 profile. Its one exception is a bounded Type2 velocity
# bias when the raw position remains within one encoder count, including a
# one-count net change. This does not waive any position, tail, timing, per-sample
# speed, mode, fault, or freshness check.
POSITION_COUNT_RAD = (POSITION_MAX - POSITION_MIN) / 65535
FULLBODY_POSITION_LIMITS = {**POSITION_LIMITS, "abs_velocity_mean_rad_s": .025,
                            "static_position_bias_max_abs_mean_rad_s": .04,
                            "static_position_bias_max_RMS_rad_s": .08,
                            "static_position_bias_max_count_span": 1}
REVIEWED_CURRENT_HOLD_POSITION_LIMITS = {**POSITION_LIMITS,
                                         "abs_velocity_mean_rad_s": .025}
PROFILES = ("legacy-rms-v1", "position-v2", FULLBODY_POSITION_PROFILE)


def profile_limits(profile):
    if profile not in PROFILES:
        raise ValueError("Unknown stationarity profile")
    return dict(FULLBODY_POSITION_LIMITS if profile == FULLBODY_POSITION_PROFILE
                else POSITION_LIMITS if profile == "position-v2" else SETTLED_LIMITS)


def selected_ids(ids):
    ids = tuple(ids)
    if any(type(i) is not int for i in ids) or ids not in LEGS.values():
        raise ValueError("Select exactly one leg, ordered foot/thigh/hip")
    return ids


def validated_uids(values, ids):
    if not isinstance(values, dict):
        raise ValueError("Expected UIDs must be an object keyed by motor ID")
    if any(type(k) not in (str, int) or str(k) not in {str(i) for i in range(1, 13)} for k in values):
        raise ValueError("Invalid expected-UID motor ID")
    normalized = {int(k): v for k, v in values.items()}
    if len(normalized) != len(values) or set(normalized) not in (set(ids), set(range(1, 13))):
        raise ValueError("Supply exactly the selected three identities or all twelve")
    if any(not isinstance(v, str) or len(v) != 16 or any(c not in '0123456789abcdef' for c in v)
           for v in normalized.values()) or len(set(normalized.values())) != len(normalized):
        raise ValueError("Identities must be unique sixteen-character lowercase hex strings")
    return {i: normalized[i] for i in ids}


def evaluate_settled_window(samples, centers, *, profile="legacy-rms-v1", reviewed_motor_ids=None):
    """Evaluate every sample in one fixed window; never trim, wrap, or retry.

    This deliberately changes the former instantaneous 0.05 rad/s test to a
    position/time/RMS conjunction. The instantaneous 0.5 rad/s guard remains.
    Both times are required so a delayed measurement cannot look stationary.
    This pure calculation grants no motor permission. run_leg_trial separately
    requires the UID-bound evidence file before any position-v2 transport call.
    Current-hold reviewed axes come from the runner's validated review. Omitting
    that set retains the original ID6-only FL or ID8-only RR calculation.
    """
    ids = selected_ids(centers)
    if profile == CURRENT_HOLD_PROFILE:
        reviewed = validated_reviewed_ids(ids, reviewed_motor_ids)
        legacy = evaluate_settled_window(samples, centers, profile='legacy-rms-v1')
        position = evaluate_settled_window(samples, centers, profile='position-v2')
        for mid in reviewed:
            entry = position['motors'][mid]
            # The earlier reviewed-axis current-hold grant retains its original
            # .025 signed-mean check. Only the separate full-body profile has
            # the narrowly bounded static-position noise exception.
            mean = entry.get('velocity_mean_rad_s')
            if mean is not None and abs(mean) > REVIEWED_CURRENT_HOLD_POSITION_LIMITS['abs_velocity_mean_rad_s']:
                entry['errors'].append('abs_velocity_mean_rad_s exceeds 0.025')
            legacy['motors'][mid] = entry
        legacy.update(profile=profile, reviewed_motor_ids=list(reviewed),
                      stationarity_spec=('fixed21_reviewed_single_axis_position_tail_mean_v1'
                          if len(reviewed) == 1 else 'fixed21_reviewed_axis_set_position_tail_mean_v1'))
        if len(reviewed) == 1:
            legacy['reviewed_motor_id'] = reviewed[0]
        legacy['effective_profile_by_motor'] = {
            i: 'position-v2' if i in reviewed else 'legacy-rms-v1' for i in ids}
        legacy['limits_by_motor'] = {
            i: dict(REVIEWED_CURRENT_HOLD_POSITION_LIMITS) if i in reviewed
            else profile_limits('legacy-rms-v1') for i in ids}
        legacy.pop('limits')
        legacy['errors'] = ([] if set(samples) == set(ids) and all(type(i) is int for i in samples)
                            else ['Window motor IDs differ from selected leg'])
        legacy['warnings'] = []
        for i, entry in legacy['motors'].items():
            legacy['errors'].extend(f'ID{i}: {e}' for e in entry['errors'])
            legacy['warnings'].extend(f'ID{i}: {e}' for e in entry['warnings'])
        legacy['passed'] = not legacy['errors']
        return legacy
    if reviewed_motor_ids is not None:
        raise ValueError('Reviewed motor sets require the current-hold profile')
    limits = profile_limits(profile)
    report = {"passed": False, "sample_count_required": SETTLED_COUNT,
              "sample_period_s": SETTLED_PERIOD_S, "limits": limits, "profile": profile, "warnings": [],
              "errors": [], "motors": {}, "absolute_rest_proven": False,
              "joint_calibration_verified": False,
              "stationarity_spec": ("fixed21_position_OLS_velocity_RMS_v1" if profile == "legacy-rms-v1"
                                    else "fixed21_position_tail_quantized_bias_v5"
                                    if profile == FULLBODY_POSITION_PROFILE
                                    else "fixed21_position_tail_mean_v2")}
    if any(type(i) is not int for i in samples) or set(samples) != set(ids):
        report["errors"].append("Window motor IDs differ from selected leg")
    for mid in ids:
        rows = samples.get(mid, [])
        entry = {"sample_count": len(rows), "samples": rows, "errors": [], "warnings": []}
        report["motors"][mid] = entry
        if len(rows) != SETTLED_COUNT:
            entry["errors"].append(f"Expected exactly {SETTLED_COUNT} samples")
        times, positions, position_counts, velocities = [], [], [], []
        for index, row in enumerate(rows):
            try:
                if type(row["sample_index"]) is not int or row["sample_index"] != index:
                    raise ValueError("Missing, duplicate, or reordered sample index")
                fb = Type2Feedback(**row["feedback"])
                received, checked = row["received_monotonic_s"], row["checked_monotonic_s"]
                real = (centers[mid], received, checked, fb.protocol_position_rad,
                        fb.velocity_rad_s, fb.torque_nm, fb.temperature_c)
                if not all(type(v) in (int, float) and math.isfinite(v) for v in real):
                    raise ValueError("Nonfinite or nonnumeric sample")
                if (type(fb.mode_state) is not int or type(fb.fault_bits) is not int
                        or type(fb.position_u16) is not int or not 0 <= fb.position_u16 <= 65535):
                    raise ValueError("Malformed integer feedback fields")
                check_feedback(fb, centers[mid], received, checked, required_mode=0,
                               max_drift_rad=SETTLED_LIMITS["maximum_center_drift_rad"])
                if times and (received <= times[-1] or received-times[-1] > .15):
                    raise ValueError("Non-increasing sample time or gap over150ms")
                times.append(received)
                positions.append(fb.protocol_position_rad)
                position_counts.append(fb.position_u16)
                velocities.append(fb.velocity_rad_s)
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                entry["errors"].append(f"sample{index}: {error}")
        if len(times) == SETTLED_COUNT and not entry["errors"]:
            # Subtract the first timestamp before regression for numerical stability.
            times = [t-times[0] for t in times]
            tm, pm = sum(times)/len(times), sum(positions)/len(positions)
            span = times[-1]
            slope = sum((t-tm)*(p-pm) for t, p in zip(times, positions))/sum((t-tm)**2 for t in times)
            entry.update(span_s=span, maximum_gap_s=max(b-a for a, b in zip(times, times[1:])),
                         position_range_rad=max(positions)-min(positions), OLS_slope_rad_s=slope,
                         velocity_RMS_rad_s=math.sqrt(sum(v*v for v in velocities)/len(velocities)),
                         max_abs_velocity_rad_s=max(abs(v) for v in velocities))
            if not 1.9 <= span <= 2.3:
                entry["errors"].append("Observation span outside1.9..2.3s")
            checks = [("position_range_rad", entry["position_range_rad"]),
                      ("abs_OLS_slope_rad_s", abs(slope))]
            if profile == "legacy-rms-v1":
                checks.append(("velocity_RMS_rad_s", entry["velocity_RMS_rad_s"]))
            else:
                tt, pp = times[-6:], positions[-6:]
                tmean, pmean = sum(tt)/6, sum(pp)/6
                tail_slope = sum((t-tmean)*(p-pmean) for t, p in zip(tt, pp))/sum((t-tmean)**2 for t in tt)
                entry.update(velocity_mean_rad_s=sum(velocities)/len(velocities),
                             tail_span_s=tt[-1]-tt[0], tail_position_range_rad=max(pp)-min(pp),
                             tail_OLS_slope_rad_s=tail_slope)
                if not .45 <= entry["tail_span_s"] <= .60:
                    entry["errors"].append("Last six samples span outside .45.. .60s")
                checks.extend((("tail_position_range_rad", entry["tail_position_range_rad"]),
                               ("tail_abs_OLS_slope_rad_s", abs(tail_slope))))
                count_span = max(position_counts) - min(position_counts)
                # Compare relative positions: the absolute center may be
                # arbitrary in synthetic tests, but every observed count step
                # must agree with the Type2 decoder's one-count resolution.
                count_position_consistent = all(abs((p - positions[0])
                    - (c - position_counts[0]) * POSITION_COUNT_RAD) <= 1e-10
                    for p, c in zip(positions, position_counts))
                static_bias_exception = (profile == FULLBODY_POSITION_PROFILE
                    and count_span <= limits["static_position_bias_max_count_span"]
                    and count_position_consistent
                    and abs(entry["velocity_mean_rad_s"]) <= limits["static_position_bias_max_abs_mean_rad_s"]
                    and entry["velocity_RMS_rad_s"] <= limits["static_position_bias_max_RMS_rad_s"])
                if profile == FULLBODY_POSITION_PROFILE:
                    entry["position_count_span"] = count_span
                    entry["position_count_returned_to_start"] = position_counts[0] == position_counts[-1]
                    entry["position_count_consistent"] = count_position_consistent
                    entry["static_position_bias_exception_applied"] = bool(
                        static_bias_exception
                        and abs(entry["velocity_mean_rad_s"]) > limits["abs_velocity_mean_rad_s"])
                if not static_bias_exception:
                    checks.append(("abs_velocity_mean_rad_s", abs(entry["velocity_mean_rad_s"])))
                elif entry["static_position_bias_exception_applied"]:
                    warning = "bounded Type2 velocity bias accepted only with at-most-one-count quantized position"
                    entry["warnings"].append(warning)
                    report["warnings"].append(f"ID{mid}: {warning}")
                if entry["velocity_RMS_rad_s"] > SETTLED_LIMITS["velocity_RMS_rad_s"]:
                    warning = "velocity_RMS_rad_s exceeds legacy .05; position-v2 uses stricter position, tail and signed-mean gates"
                    entry["warnings"].append(warning)
                    report["warnings"].append(f"ID{mid}: {warning}")
            for key, value in checks:
                if value > limits[key]:
                    entry["errors"].append(f"{key} exceeds {limits[key]}")
        report["errors"].extend(f"ID{mid}: {error}" for error in entry["errors"])
    report["passed"] = not report["errors"]
    return report


class LegTrialTransport:
    """One serial owner and one receive parser route feedback from all selected IDs."""
    def __init__(self, serial_port, emit, check_interrupt=lambda: None, *, ids, wait=None):
        self.ids = selected_ids(ids)
        self.serial, self.emit, self.check_interrupt = serial_port, emit, check_interrupt
        self.parser, self.latest = ATParser(), {}
        self.relaxed_log, self.active_deadline, self.feedback_guard = False, None, None
        self.pre_send_guard = None
        self.pre_enable_guard = None
        self.fault_latched = None
        self.wait = (lambda seconds: time.sleep(seconds)) if wait is None else wait
        self.last_write_finished_s = None

    def pace_transmit(self):
        """Measure spacing after write completion, including partial/failed writes."""
        if self.last_write_finished_s is None:
            return
        next_write = self.last_write_finished_s + MIN_TX_INTERVAL_S
        while True:
            remaining = next_write - time.monotonic()
            if remaining <= 0:
                return
            self.wait(remaining)

    def log(self, event):
        try:
            self.emit({"monotonic_ns": time.monotonic_ns(), **event})
        except BaseException:
            if not self.relaxed_log:
                raise

    def receive(self):
        chunk = self.serial.read(min(max(self.serial.in_waiting, 1), 2048))
        when = time.monotonic()
        if not chunk:
            return []
        self.log({"kind": "can_rx_bytes", "hex": chunk.hex()})
        frames = self.parser.feed(chunk)
        for frame in frames:
            self.log({"kind": "can_rx_frame", **frame.record()})
            if frame.source not in self.ids or frame.kind not in (2, 21):
                continue
            try:
                if frame.kind == 21:
                    raise RuntimeError(f"ID{frame.source} reported Type21 fault")
                value = decode_type2(frame, motor_id=frame.source)
                self.latest[frame.source] = (value, when)
                if value.fault_bits:
                    raise RuntimeError(f"ID{frame.source} feedback fault")
                if self.feedback_guard is not None and not self.relaxed_log:
                    self.feedback_guard(value, when, frame.source)
            except BaseException as error:
                self.fault_latched = self.fault_latched or repr(error)
                if not self.relaxed_log:
                    raise
        if self.parser.discarded_bytes and not self.relaxed_log:
            self.fault_latched = self.fault_latched or "Parser discarded bytes"
            raise RuntimeError(self.fault_latched)
        return [(frame, when) for frame in frames]

    def fresh_boundary(self):
        if self.fault_latched and not self.relaxed_log:
            raise RuntimeError(self.fault_latched)
        end = time.monotonic() + .03
        while self.serial.in_waiting:
            self.receive()
            if time.monotonic() > end:
                raise RuntimeError("Input backlog")
        if self.parser.buffer or self.parser.discarded_bytes:
            raise RuntimeError("Partial or discarded serial frame before command")

    def send(self, wire):
        parser = ATParser()
        frames = parser.feed(wire)
        if (len(frames) != 1 or parser.buffer or parser.discarded_bytes
                or frames[0].flags != 4 or len(frames[0].data) != 8
                or frames[0].destination not in self.ids or frames[0].kind not in (0, 1, 3, 4, 17, 18)):
            raise ValueError("Only selected-leg canonical trial frames are allowed")
        frame = frames[0]
        if frame.kind in (0, 3, 4, 18):
            canonical = {0: lambda: read_request(frame.destination),
                3: lambda: enable_request(phase=TrialPhase.ENABLE, motor_id=frame.destination),
                4: lambda: stop_request(phase=TrialPhase.STOP, motor_id=frame.destination),
                18: lambda: watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=frame.destination)}
            if wire != canonical[frame.kind]():
                raise ValueError("Only canonical identity/enable/stop/volatile-watchdog frames are allowed")
        if frame.kind != 4:
            self.check_interrupt()
            if self.fault_latched:
                raise RuntimeError(self.fault_latched)
        self.pace_transmit()
        # A signal, deadline, or stale sibling sample can arise during pacing.
        # Stop bypasses active-trial guards; it still uses the same TX spacing.
        if frame.kind != 4:
            self.check_interrupt()
            if self.fault_latched:
                raise RuntimeError(self.fault_latched)
        if frame.kind in (1, 3) and self.active_deadline is not None and time.monotonic() >= self.active_deadline:
            raise RuntimeError("Active trial deadline reached")
        if frame.kind == 1 and frame.data[4:8] != bytes(4) and self.pre_send_guard is not None:
            self.pre_send_guard()
        if frame.kind == 3 and self.pre_enable_guard is not None:
            self.pre_enable_guard()
        if frame.kind in (1, 3) and self.active_deadline is not None and time.monotonic() >= self.active_deadline:
            raise RuntimeError("Active trial deadline reached after pre-send guards")
        try:
            written = self.serial.write(wire)
        finally:
            self.last_write_finished_s = time.monotonic()
        if written != len(wire):
            raise IOError("Partial serial write")
        self.log({"kind": "can_tx", "hex": wire.hex(), "type": frame.kind, "motor_id": frame.destination})

    def exchange_many(self, wires, expected_ids, accept):
        expected_ids = tuple(expected_ids)
        if not expected_ids or len(set(expected_ids)) != len(expected_ids) or not set(expected_ids) <= set(self.ids):
            raise ValueError("Response IDs must be a nonempty unique selected-leg subset")
        self.fresh_boundary()
        for wire in wires:
            self.send(wire)
        deadline = time.monotonic() + .08
        if self.active_deadline is not None:
            deadline = min(deadline, self.active_deadline)
        found = {}
        while time.monotonic() < deadline:
            self.check_interrupt()
            for frame, when in self.receive():
                if frame.source in expected_ids:
                    value = accept(frame)
                    if value is not None:
                        found[frame.source] = (value, when)
            if set(found) == set(expected_ids) and time.monotonic() - max(t for _, t in found.values()) >= .004:
                return found
        raise TimeoutError("Not all selected replies arrived fresh within80ms")

    def parameter(self, motor_id, name=None):
        def accept(frame):
            if matches(frame, motor_id, name):
                value = decode_reply(frame, motor_id, name)
                if not value["ok"]:
                    raise RuntimeError(f"ID{motor_id} parameter rejected: {name}")
                return value
        return self.exchange_many([read_request(motor_id, name)], (motor_id,), accept)[motor_id][0]

    def feedback_many(self, wires, expected_ids):
        def accept(frame):
            if frame.kind == 2:
                return decode_type2(frame, motor_id=frame.source)
        found = self.exchange_many(wires, expected_ids, accept)
        for motor_id, (value, when) in found.items():
            self.log({"kind": "leg_trial_feedback", "motor_id": motor_id,
                      "received_monotonic_s": when, **asdict(value)})
        return found

    def stop_all(self, ids):
        """Attempt EVERY paced stop before receiving; failures remain isolated per ID."""
        ids = tuple(ids)
        if len(set(ids)) != len(ids) or not set(ids) <= set(self.ids):
            raise ValueError("Stop targets must be identified selected-leg IDs")
        self.relaxed_log, self.feedback_guard, self.pre_send_guard = True, None, None
        self.pre_enable_guard = None
        reports = {i: {"confirmed": False, "feedback": None, "error": None} for i in ids}
        boundary_valid = True
        try:
            self.fresh_boundary()
        except BaseException:
            boundary_valid, self.parser = False, ATParser()
        sent = {}
        for motor_id in ids:
            try:
                self.send(stop_request(phase=TrialPhase.STOP, motor_id=motor_id))
                sent[motor_id] = time.monotonic()
            except BaseException as error:
                reports[motor_id]["error"] = "Stop write failed: " + repr(error)
        deadline = time.monotonic() + .08
        while time.monotonic() < deadline:
            try:
                received = self.receive()
            except BaseException as error:
                for entry in reports.values():
                    entry["confirmed"] = False
                    entry["error"] = entry["error"] or "Stop read failed: " + repr(error)
                break
            for frame, when in received:
                i = frame.source
                if i not in reports or frame.kind not in (2, 21):
                    continue
                try:
                    if frame.kind == 21:
                        raise RuntimeError("Type21 fault during stop")
                    value = decode_type2(frame, motor_id=i)
                    reports[i]["feedback"] = asdict(value)
                    if value.fault_bits:
                        raise RuntimeError("Fault bits during stop")
                    reports[i]["confirmed"] = bool(boundary_valid and i in sent and when >= sent[i]
                        and value.mode_state == 0 and reports[i]["error"] is None)
                except BaseException as error:
                    reports[i]["confirmed"] = False
                    reports[i]["error"] = repr(error)
        for entry in reports.values():
            if not boundary_valid or self.parser.discarded_bytes or self.parser.buffer:
                entry["confirmed"] = False
                entry["error"] = entry["error"] or "Stop sent; fresh clean reply boundary unavailable"
            if not entry["confirmed"]:
                entry["error"] = entry["error"] or "No fresh reset0/fault0 confirmation"
        self.relaxed_log = False
        return reports


def run_leg_trial(transport, expected_uids, check_interrupt, emit, *, directions,
                  clock=time.monotonic, wait=time.sleep, profile="legacy-rms-v1",
                  position_response_evidence=None, position_response_evidence_sha256=None):
    """Original relative5-degree trial; its default path and limits are unchanged."""
    return _run_leg_trial(transport, expected_uids, check_interrupt, emit,
        directions=directions, clock=clock, wait=wait, profile=profile,
        position_response_evidence=position_response_evidence,
        position_response_evidence_sha256=position_response_evidence_sha256)


def run_bounded_pose_trial(transport, expected_uids, check_interrupt, emit, *, absolute_targets,
                          clock=time.monotonic, wait=time.sleep, profile="position-v2",
                          position_response_evidence=None, position_response_evidence_sha256=None,
                          gain_profile="kp3", matched_start_positions=None,
                          matched_start_tolerance_rad=math.radians(.5), observation_profile=None,
                          enable_profile="legacy-burst"):
    """Explicit absolute raw targets within5degrees; no CLI, wrapping or chaining.

    This does not validate a stored L reference's identity/turn continuity or its
    physical accuracy. The caller must establish those and existing physical
    readiness. Success describes a finite diagnostic arrival/hold candidate;
    stop still removes control, and never permits learned-policy deployment or a persistent hold.
    fr_hip_kp6_diagnostic requires FR, position-v2 evidence, matched references,
    sequential enable and targets within4.5degrees of those references. Only
    ID3 uses Kp6; ID1/2 retain Kp3, and all three use the0.5Nm feedback monitor.
    fr_current_kp12_diagnostic holds fixed matched raw references for5s with
    Kp3/12/12. It requires targets equal those references and sequential enable;
    the existing0.5degree fresh-start match permits a small initial correction.
    No ramp or added target offset is sent; final1s acceptance remains unchanged.
    fr_step4_kp12_diagnostic separately permits one4s cosine ramp plus1s hold
    at Kp3/12/12, with targets within4degrees of matched references and at most
    4.5degrees from fresh centers. It retains all arrival and torque checks.
    fr_step4_kp12_torque1_diagnostic uses that same trajectory and gains, with
    an explicit1.0Nm ID2 feedback monitor; IDs1/3 retain0.5Nm. The separately
    selected fr_step4_kp12_peak_burst_diagnostic permits at most three fresh
    active ID2 samples over1.0Nm before aborting; 1.5Nm is the absolute
    feedback ceiling. Neither profile sets a physical motor torque cap.
    fr_step4_thigh_kp18_peak_burst_diagnostic changes only ID2 Kp from12 to18
    for a separately selected fixed4-degree FR experiment; all other limits,
    timing, Kd, torque monitors, and explicit stop behavior stay the same.
    kp4_diagnostic must be selected explicitly. Its0.5Nm Type2-feedback monitor
    is a diagnostic abort threshold, not an actuator or physical torque cap.
    Optional matched_start_positions compares all three fresh disabled raw centers
    to an explicit fixed reference within the fixed0.5-degree candidate tolerance.
    A mismatch rejects enable; it never moves the leg back to the reference.
    rr_hip_kp6_diagnostic is a separate explicit position-v2-only RR diagnostic:
    matched references are mandatory, ID7/8 targets must equal their references
    at Kp4, and ID9 must target reference minus4degrees at Kp6. No other target,
    leg, automatic gain change or CLI selection is permitted for that profile.
    Explicit rr_settling_1s and fr_settling_1s observation profiles keep their
    respective fixed RR hip and FR4-degree peak-burst experiments. They permit
    observation of up to2-degree target error for the nominal last1s, aborting
    if any axis worsens more than0.25degree from the first complete hold batch.
    The strict1-degree arrival/hold evaluation stays unchanged; observation
    completion is a distinct status, not target success or visible motion proof.
    sequential-confirmed is an explicit bounded/matched-reference enable
    profile: each selected axis must return fresh mode2 before the next
    enable, and all axes must still be fresh mode2 before nonzero gain.
    """
    from .bounded_pose_plan import normalize_absolute_targets
    ids = selected_ids(transport.ids)
    leg = next(name for name in LEGS if LEGS[name] == ids)
    targets = normalize_absolute_targets(leg, absolute_targets)
    return _run_leg_trial(transport, expected_uids, check_interrupt, emit,
        directions=(1, 1, 1), clock=clock, wait=wait, profile=profile,
        position_response_evidence=position_response_evidence,
        position_response_evidence_sha256=position_response_evidence_sha256,
        absolute_targets=targets, gain_profile=gain_profile,
        matched_start_positions=matched_start_positions,
        matched_start_tolerance_rad=matched_start_tolerance_rad,
        observation_profile=observation_profile, enable_profile=enable_profile)


def _run_leg_trial(transport, expected_uids, check_interrupt, emit, *, directions,
                   clock=time.monotonic, wait=time.sleep, profile="legacy-rms-v1",
                   position_response_evidence=None, position_response_evidence_sha256=None,
                   absolute_targets=None, gain_profile="kp3", matched_start_positions=None,
                   matched_start_tolerance_rad=math.radians(.5), observation_profile=None,
                   enable_profile="legacy-burst"):
    if type(enable_profile) is not str or enable_profile not in ("legacy-burst", "sequential-confirmed"):
        raise ValueError("Explicit enable profile must be legacy-burst or sequential-confirmed")
    if enable_profile == "sequential-confirmed" and (absolute_targets is None or matched_start_positions is None):
        raise ValueError("Sequential enable requires bounded absolute targets and matched references")
    if observation_profile is not None and (type(observation_profile) is not str
            or observation_profile not in ("rr_settling_1s", "fr_settling_1s")):
        raise ValueError("Observation profile must be None, rr_settling_1s, or fr_settling_1s")
    rr_settling_observation = observation_profile == "rr_settling_1s"
    fr_settling_observation = observation_profile == "fr_settling_1s"
    settling_observation = rr_settling_observation or fr_settling_observation
    if type(gain_profile) is not str or gain_profile not in (
            "kp3", "kp4_diagnostic", "rr_hip_kp6_diagnostic", "fr_hip_kp6_diagnostic",
            "fr_current_kp12_diagnostic", "fr_step4_kp12_diagnostic",
            "fr_step4_kp12_torque1_diagnostic", "fr_step4_kp12_peak_burst_diagnostic",
            "fr_step4_thigh_kp18_peak_burst_diagnostic"):
        raise ValueError("Select an explicit supported fixed diagnostic gain profile")
    if absolute_targets is None and gain_profile != "kp3":
        raise ValueError("Diagnostic gains are available only for explicit bounded absolute targets")
    if absolute_targets is None and matched_start_positions is not None:
        raise ValueError("Matched start requires explicit bounded absolute targets")
    kp4 = gain_profile == "kp4_diagnostic"
    rr_hip_kp6 = gain_profile == "rr_hip_kp6_diagnostic"
    fr_hip_kp6 = gain_profile == "fr_hip_kp6_diagnostic"
    fr_current_kp12 = gain_profile == "fr_current_kp12_diagnostic"
    fr_step4_kp18 = gain_profile == "fr_step4_thigh_kp18_peak_burst_diagnostic"
    fr_step4_burst = gain_profile in ("fr_step4_kp12_peak_burst_diagnostic",
                                      "fr_step4_thigh_kp18_peak_burst_diagnostic")
    fr_step4_torque1 = gain_profile == "fr_step4_kp12_torque1_diagnostic" or fr_step4_burst
    fr_step4_kp12 = gain_profile in ("fr_step4_kp12_diagnostic", "fr_step4_kp12_torque1_diagnostic",
                                   "fr_step4_kp12_peak_burst_diagnostic",
                                   "fr_step4_thigh_kp18_peak_burst_diagnostic")
    if rr_settling_observation and not rr_hip_kp6:
        raise ValueError("RR settling observation requires the fixed RR hip Kp6 diagnostic")
    if fr_settling_observation and not fr_step4_burst:
        raise ValueError("FR settling observation requires the fixed FR4-degree peak-burst diagnostic")
    torque_monitor = kp4 or rr_hip_kp6 or fr_hip_kp6 or fr_current_kp12 or fr_step4_kp12
    ids = selected_ids(transport.ids)
    torque_limits = {i: (1.5 if fr_step4_burst else 1.) if fr_step4_torque1 and i == 2 else .5 for i in ids}
    motion_phases = {i: TrialPhase.POSITION_STEP5_KP4 if kp4 or rr_hip_kp6
                     else TrialPhase.POSITION_STEP5 for i in ids}
    pose = None
    if absolute_targets is not None:
        from . import bounded_pose_plan as pose
        leg = next(name for name in LEGS if LEGS[name] == ids)
        absolute_targets = pose.normalize_absolute_targets(leg, absolute_targets)
        matched_start_tolerance_rad = pose.validate_matched_start_tolerance(matched_start_tolerance_rad)
        if matched_start_positions is not None:
            matched_start_positions = pose.normalize_matched_start_positions(leg, matched_start_positions)
    if rr_hip_kp6:
        # This profile is an explicit fixed-reference hip experiment, not a
        # general Kp6 option. Reject scope/target changes before any transport I/O.
        if ids != (7, 8, 9) or matched_start_positions is None or profile != "position-v2":
            raise ValueError("RR hip Kp6 requires exact IDs7/8/9, matched references and position-v2 evidence")
        required_targets = {7: matched_start_positions[7], 8: matched_start_positions[8],
                            9: matched_start_positions[9] - math.radians(4)}
        if absolute_targets != required_targets:
            raise ValueError("RR hip Kp6 requires fixed ID7/8 references and ID9 reference minus4degrees; no recenter/wrap")
        motion_phases[9] = TrialPhase.POSITION_STEP5_RR_HIP_KP6
    if fr_hip_kp6:
        if (ids != (1, 2, 3) or matched_start_positions is None or profile != "position-v2"
                or enable_profile != "sequential-confirmed"):
            raise ValueError("FR hip Kp6 requires IDs1/2/3, matched references, position-v2 evidence and sequential enable")
        limit = math.radians(4.5)
        if any(not matched_start_positions[i]-limit <= absolute_targets[i] <= matched_start_positions[i]+limit
               for i in ids):
            raise ValueError("FR hip Kp6 targets must remain within4.5degrees of matched raw references; no wrapping")
        motion_phases[3] = TrialPhase.POSITION_STEP5_FR_HIP_KP6
    if fr_current_kp12:
        if (ids != (1, 2, 3) or matched_start_positions is None or profile != "position-v2"
                or enable_profile != "sequential-confirmed"):
            raise ValueError("FR current Kp12 requires IDs1/2/3, matched references, position-v2 evidence and sequential enable")
        if absolute_targets != matched_start_positions:
            raise ValueError("FR current Kp12 targets must exactly equal matched raw references; no added motion")
        motion_phases[2] = TrialPhase.POSITION_CURRENT_FR_THIGH_KP12
        motion_phases[3] = TrialPhase.POSITION_CURRENT_FR_HIP_KP12
    if fr_step4_kp12:
        if (ids != (1, 2, 3) or matched_start_positions is None or profile != "position-v2"
                or enable_profile != "sequential-confirmed"):
            raise ValueError("FR step4 Kp12 requires IDs1/2/3, matched references, position-v2 evidence and sequential enable")
        limit = math.radians(4)
        if any(not matched_start_positions[i]-limit <= absolute_targets[i] <= matched_start_positions[i]+limit
               for i in ids):
            raise ValueError("FR step4 Kp12 targets must remain within4degrees of matched raw references; no wrapping")
        motion_phases[2] = TrialPhase.POSITION_STEP4_FR_THIGH_KP12
        motion_phases[3] = TrialPhase.POSITION_STEP4_FR_HIP_KP12
        if fr_step4_kp18:
            motion_phases[2] = TrialPhase.POSITION_STEP4_FR_THIGH_KP18
    if profile != CURRENT_HOLD_PROFILE:
        profile_limits(profile)
    expected_uids = validated_uids(expected_uids, ids)
    response_evidence = None
    reviewed_motor_ids = None
    if profile == CURRENT_HOLD_PROFILE:
        response_evidence = load_current_hold_review(
            position_response_evidence, expected_uids, ids=ids,
            absolute_targets=absolute_targets, matched_start_positions=matched_start_positions,
            matched_start_tolerance_rad=matched_start_tolerance_rad, gain_profile=gain_profile,
            observation_profile=observation_profile, expected_sha256=position_response_evidence_sha256)
        # Capture only the validated set before any I/O or externally supplied logging.
        reviewed_motor_ids = validated_reviewed_ids(ids, response_evidence['reviewed_motor_ids'])
    elif profile == "position-v2":
        if position_response_evidence is None:
            raise ValueError("Position-v2 requires a position-response evidence file")
        leg = next(name for name in LEGS if LEGS[name] == ids)
        response_evidence = load_position_response_evidence(
            position_response_evidence, expected_uids, leg=leg,
            expected_sha256=position_response_evidence_sha256)
    directions = tuple(directions)
    if len(directions) != 3 or any(type(d) is not int or d not in (-1, 1) for d in directions):
        raise ValueError("Three explicit integer directions -1/+1 are required")
    signs = dict(zip(ids, directions))
    result = {"motor_ids": list(ids), "directions": list(directions), "errors": [],
              "enable_profile": enable_profile,
              "position_response_evidence": response_evidence,
              "stationarity_profile": profile, "motion_completed": False, "stop_confirmed": False, "joint_calibration_verified": False,
              "watchdog_physical_latency_verified": False, "watchdog_persisted": False, "motors": {}}
    pose_plan, pose_start_ns, pose_end_ns, pose_hold_start = None, None, None, None
    pose_requests, pose_samples = {}, []
    settling_baseline = {}
    if pose is not None:
        gain_plan = {"gain_profile": gain_profile, "Kp": None if rr_hip_kp6 or fr_hip_kp6 or fr_current_kp12 or fr_step4_kp12 else (4. if kp4 else 3.), "Kd": .15,
                     "torque_feedforward_nm": 0.,
                     "max_abs_torque_feedback_candidate_nm": .5 if torque_monitor and not fr_step4_torque1 else None,
                     "max_abs_torque_feedback_candidate_nm_by_motor": dict(torque_limits) if torque_monitor else None,
                     "physical_torque_cap_verified": False,
                     "torque_monitor_semantics": "declared-profile Type2 feedback diagnostic abort; not physical cap",
                     "automatic_gain_increase": False}
        if rr_hip_kp6:
            gain_plan.update(Kp_by_motor={7: 4., 8: 4., 9: 6.},
                             motion_phase_by_motor={i: motion_phases[i].value for i in ids})
        if fr_hip_kp6:
            gain_plan.update(Kp_by_motor={1: 3., 2: 3., 3: 6.},
                             motion_phase_by_motor={i: motion_phases[i].value for i in ids})
        if fr_current_kp12:
            gain_plan.update(Kp_by_motor={1: 3., 2: 12., 3: 12.},
                motion_phase_by_motor={i: motion_phases[i].value for i in ids},
                current_position_only=True, target_ramp_applied=False, added_target_offset_rad=0.,
                initial_reference_correction_limit_rad=matched_start_tolerance_rad,
                command_trajectory='constant matched raw reference for5s; final1s acceptance')
        if fr_step4_kp12:
            gain_plan.update(Kp_by_motor={1: 3., 2: 18. if fr_step4_kp18 else 12., 3: 12.},
                motion_phase_by_motor={i: motion_phases[i].value for i in ids},
                current_position_only=False, target_ramp_applied=True,
                maximum_reference_delta_rad=math.radians(4),
                maximum_fresh_center_delta_rad=math.radians(4.5),
                command_trajectory='one4s cosine ramp plus1s target hold; no chaining')
        if fr_step4_burst:
            gain_plan.update(id2_above_one_nm_max_active_fresh_samples=3,
                             id2_above_one_nm_nominal_sample_budget_s=.15,
                             id2_burst_monitor_semantics='fresh Type2 feedback samples after first active command; not a physical torque cap')
            result.update(id2_above_one_nm_active_fresh_samples=0,
                          id2_active_peak_abs_torque_feedback_nm=0.)
        if settling_observation:
            gain_plan.update(observation_profile=observation_profile, observation_only=True,
                max_abs_target_error_observation_rad=pose.SETTLING_MAX_ERROR_RAD,
                max_abs_error_worsening_observation_rad=pose.SETTLING_MAX_WORSENING_RAD,
                arrival_threshold_unchanged_rad=pose.ARRIVAL_ERROR_CANDIDATE_RAD)
            result.update(settling_observation_completed=False, settling_observation_samples=pose_samples)
        result.update(directions=None, absolute_targets_rad=absolute_targets, trajectory_elapsed=False,
                      arrival_candidate_met=False, hold_candidate_met=False, **pose.FLAGS, **gain_plan,
                      request_timestamp_semantics="conservative batch-start lower bound; not individual UART write time",
                      timestamp_source="monotonic seconds rounded to integer nanoseconds")
        if matched_start_positions is not None:
            result.update(matched_start_positions_rad=dict(matched_start_positions),
                          matched_start_tolerance_rad=matched_start_tolerance_rad,
                          automatic_repositioning=False)
    def ns(seconds):
        return int(round(seconds * 1_000_000_000))
    identified, centers = [], {}
    id2_burst_last_received_ns = None
    id2_burst_first_high_ns = None
    def torque_guard(torque, motor_id):
        limit = torque_limits[motor_id]
        if torque_monitor and (type(torque) not in (float, int) or not math.isfinite(torque) or abs(torque) > limit):
            raise RuntimeError(f"ID{motor_id} Type2 torque feedback exceeds{limit:g}Nm diagnostic monitor or is nonfinite")
    def guard(value, received, motor_id, required_mode=2):
        nonlocal id2_burst_last_received_ns, id2_burst_first_high_ns
        torque_guard(value.torque_nm, motor_id)
        if (fr_step4_burst and motor_id == 2 and required_mode == 2
                and pose_start_ns is not None and ns(received) >= pose_start_ns):
            stamp = ns(received)
            if id2_burst_last_received_ns is None or stamp > id2_burst_last_received_ns:
                id2_burst_last_received_ns = stamp
                result['id2_active_peak_abs_torque_feedback_nm'] = max(
                    result.get('id2_active_peak_abs_torque_feedback_nm', 0.), abs(value.torque_nm))
                if abs(value.torque_nm) > 1.:
                    if id2_burst_first_high_ns is None:
                        id2_burst_first_high_ns = stamp
                    count = result.get('id2_above_one_nm_active_fresh_samples', 0) + 1
                    result['id2_above_one_nm_active_fresh_samples'] = count
                    result['id2_first_above_one_nm_received_ns'] = id2_burst_first_high_ns
                    result['id2_last_above_one_nm_received_ns'] = stamp
                    if count > 3 or stamp - id2_burst_first_high_ns >= 200_000_000:
                        raise RuntimeError('ID2 Type2 torque feedback above1.0Nm exceeded brief peak sample/time budget')
        check_feedback(value, centers[motor_id], received, clock(), required_mode=required_mode,
                       max_drift_rad=MAX_DRIFT_RAD)
        if (pose is not None and pose_hold_start is not None and required_mode == 2
                and received >= pose_hold_start):
            target = absolute_targets[motor_id]
            error = abs(value.protocol_position_rad-target)
            if settling_observation:
                if not target-pose.SETTLING_MAX_ERROR_RAD <= value.protocol_position_rad <= target+pose.SETTLING_MAX_ERROR_RAD:
                    raise RuntimeError(f"ID{motor_id} target error exceeds2-degree observation limit")
                if settling_baseline and error > settling_baseline[motor_id]+pose.SETTLING_MAX_WORSENING_RAD:
                    raise RuntimeError(f"ID{motor_id} target error worsened over0.25-degree observation limit")
            elif error > pose.ARRIVAL_ERROR_CANDIDATE_RAD:
                raise RuntimeError(f"ID{motor_id} bounded pose target error exceeds1-degree candidate")
    def batch(wires, required_mode):
        requested = clock() if pose is not None else None
        found = transport.feedback_many(wires, ids)
        if set(found) != set(ids):
            raise RuntimeError("Missing selected-leg feedback")
        transport.latest.update(found)
        for i, (value, received) in found.items():
            guard(value, received, i, required_mode)
        if pose is not None:
            pose_requests.update({i: requested for i in ids})
        return found
    def neutral(i):
        return motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=centers[i], motor_id=i)
    def active_request(i, center, offset):
        # The current-only phases never encode an offset. Their independently
        # matched absolute reference is the one constant target, even when the
        # last disabled Type2 center differs within the existing0.5degree gate.
        if fr_step4_kp12:
            limit = math.radians(4.5)
            target = center + offset
            if not center-limit <= target <= center+limit:
                raise ValueError("FR step4 Kp12 exceeds4.5degree fresh-center envelope")
            # Normalize only exact endpoints, without an epsilon or expanded
            # bound, when subtraction lost precision near a nonzero center.
            if target == center-limit: offset = -limit
            elif target == center+limit: offset = limit
        return motion_request(phase=motion_phases[i],
            center_rad=absolute_targets[i] if fr_current_kp12 else center,
            offset_rad=0. if fr_current_kp12 else offset, motor_id=i)
    def check_parameters(i):
        values = [transport.parameter(i, p)["value"] for p in ("position", "current", "voltage")]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise RuntimeError(f"ID{i} nonfinite or malformed initial parameters")
        position, current, voltage = values
        if abs(current) > .05 or not 35 <= voltage <= 43:
            raise RuntimeError(f"ID{i} current/voltage outside trial envelope")
        motion_request(phase=motion_phases[i], center_rad=position, motor_id=i)
        return position, current, voltage
    try:
        for i in ids:
            check_interrupt()
            if transport.parameter(i)["mcu_uid_hex"] != expected_uids[i]:
                raise RuntimeError(f"ID{i} identity differs from supported baseline")
            identified.append(i)
        initial = transport.stop_all(ids)
        if any(not initial[i]["confirmed"] for i in ids):
            raise RuntimeError("Initial all-leg reset/fault confirmation failed")
        if torque_monitor:
            for i in ids:
                torque_guard(initial[i]["feedback"]["torque_nm"], i)
        for i in ids:
            if transport.parameter(i, "run_mode")["value"] != 0:
                raise RuntimeError(f"ID{i} must already be in operation mode0")
            centers[i], current, voltage = check_parameters(i)
            if abs(centers[i] - initial[i]["feedback"]["protocol_position_rad"]) > .02:
                raise RuntimeError(f"ID{i} Type17/Type2 position mismatch; no wrap guessed")
            result["motors"][i] = {"center_rad": centers[i], "initial_current_A": current,
                "initial_voltage_V": voltage, "target_final_offset_rad": signs[i] * math.radians(5)}
            if rr_hip_kp6 or fr_hip_kp6 or fr_current_kp12 or fr_step4_kp12:
                result["motors"][i].update(Kp=gain_plan["Kp_by_motor"][i], Kd=.15,
                                          torque_feedforward_nm=0., motion_phase=motion_phases[i].value)
            if pose is not None:
                # Reject distant/ambiguous raw targets before watchdog setup,
                # then repeat against the final fresh disabled references.
                active_request(i, centers[i], pose.bounded_offset(centers[i],absolute_targets[i]))
        transport.feedback_guard = lambda v, t, i: guard(v, t, i, 0)
        batch([neutral(i) for i in ids], 0)
        # Validate every selected motor's prior watchdog read before changing any
        # setting. A rejected sibling read must leave all watchdogs untouched.
        for i in ids:
            result["motors"][i]["watchdog_previous_ticks"] = transport.parameter(i, "can_timeout")["value"]
        for i in ids:
            transport.fresh_boundary()
            transport.send(watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=i))
        wait(.015)
        for i in ids:
            if transport.parameter(i, "can_timeout")["value"] != 4000:
                raise RuntimeError(f"ID{i} watchdog readback mismatch")
            result["motors"][i]["watchdog_readback_ticks"] = 4000
        # Refresh references after the setup delay while all motors remain disabled.
        for i in ids:
            position, _, _ = check_parameters(i)
            if abs(position - centers[i]) > .02:
                raise RuntimeError(f"ID{i} moved during disabled setup")
            centers[i] = position
            result["motors"][i]["center_rad"] = position
        samples = {i: [] for i in ids}
        collection_error = None
        def settled_guard(value, received, motor_id):
            try:
                torque_guard(value.torque_nm, motor_id)
                check_feedback(value, centers[motor_id], received, clock(), required_mode=0,
                               max_drift_rad=SETTLED_LIMITS["maximum_center_drift_rad"])
            except Exception as error:
                raise RuntimeError(f"ID{motor_id} settled-window guard: {error}") from error
        transport.feedback_guard = settled_guard
        window_start = clock()
        try:
            for sample_index in range(SETTLED_COUNT):
                due = window_start + sample_index*SETTLED_PERIOD_S
                wait(max(0., due-clock()))
                check_interrupt()
                if clock() > due+.05:
                    raise RuntimeError("Settled-window sample scheduling missed by over50ms")
                found = batch([neutral(i) for i in ids], 0)
                for i, (value, received) in found.items():
                    samples[i].append({"sample_index": sample_index,
                        "received_monotonic_s": received, "checked_monotonic_s": clock(),
                        "feedback": asdict(value)})
                    settled_guard(value, received, i)
        except BaseException as error:
            collection_error = error
        result["settled_window"] = evaluate_settled_window(
            samples, centers, profile=profile, reviewed_motor_ids=reviewed_motor_ids)
        if collection_error is not None:
            result["settled_window"]["errors"].append("Collection failed: " + repr(collection_error))
            result["settled_window"]["passed"] = False
        emit({"kind": "leg_trial_settled_window", "report": result["settled_window"]})
        if collection_error is not None:
            raise collection_error
        if not result["settled_window"]["passed"]:
            raise RuntimeError("Leg stationary window rejected: " + "; ".join(result["settled_window"]["errors"]))
        check_interrupt()
        transport.fresh_boundary()
        def guard_last_disabled():
            for i in ids:
                value, received = transport.latest[i]
                last = samples[i][-1]
                if received != last["received_monotonic_s"] or asdict(value) != last["feedback"]:
                    raise RuntimeError(f"ID{i} feedback changed after fixed settled window; no resampling")
                settled_guard(*transport.latest[i], i)
                if pose is not None:
                    checked = ns(clock())
                    if not (0 <= checked-ns(pose_requests[i]) <= pose.MAX_AGE_NS
                            and 0 <= checked-ns(received) <= pose.MAX_AGE_NS):
                        raise RuntimeError(f"ID{i} bounded pose center request/receive aged before enable")
            if pose_plan is not None and matched_start_positions is not None:
                matched = pose.evaluate_matched_start(pose_plan, matched_start_positions,
                    now_ns=ns(clock()), tolerance_rad=matched_start_tolerance_rad)
                result["matched_start_check"] = matched
                if not matched["passed"]:
                    raise RuntimeError("Matched start rejected: " + "; ".join(matched["errors"]))
        guard_last_disabled()
        if pose is not None:
            pose_plan = pose.build_plan(leg,
                {i: {"position_rad": transport.latest[i][0].protocol_position_rad,
                     "request_ns": ns(pose_requests[i]), "received_ns": ns(transport.latest[i][1])} for i in ids},
                absolute_targets, now_ns=ns(clock()))
            for i, center in zip(ids, pose_plan.centers):
                centers[i] = center.position_rad
                offset = pose.bounded_offset(center.position_rad,absolute_targets[i])
                # Validate the exact codec center/offset used later, before ANY
                # enable. Do not mix old Type17 centers with fresh Type2 centers.
                active_request(i, center.position_rad, offset)
                result["motors"][i].update(center_rad=center.position_rad,
                    target_final_offset_rad=offset)
            result["bounded_pose_plan"] = {**pose_plan.as_dict(), **gain_plan}
            if fr_current_kp12:
                result["bounded_pose_plan"].update(ramp_ns=0, hold_ns=pose.DURATION_NS,
                    final_evaluation_start_ns=pose.RAMP_NS)
            if matched_start_positions is not None:
                result["bounded_pose_plan"].update(matched_start_positions_rad=dict(matched_start_positions),
                    matched_start_tolerance_rad=matched_start_tolerance_rad, automatic_repositioning=False)
            guard_last_disabled()
        # The legacy burst keeps checking the final disabled snapshots before
        # every Enable. The sequential profile tracks each confirmed mode2
        # reply while the not-yet-enabled siblings remain in mode0.
        transport.active_deadline = clock() + ACTIVE_BUDGET_S
        if enable_profile == "sequential-confirmed":
            # A burst can leave one RS05 in mode0 even when its siblings enter
            # mode2. Confirm each Enable with its own fresh reply; never send
            # an extra neutral burst or any nonzero gain until all three pass.
            confirmed = set()
            transitioning = None
            def enable_guard_all():
                check_interrupt()
                for mid in ids:
                    guard(*transport.latest[mid], mid, 2 if mid in confirmed else 0)
            def transition_guard(value, received, mid):
                mode = 2 if mid in confirmed else 0
                if mid == transitioning:
                    if value.mode_state not in (0, 2):
                        raise RuntimeError(f"ID{mid} unexpected enable transition mode")
                    mode = value.mode_state
                guard(value, received, mid, mode)
            transport.pre_enable_guard = enable_guard_all
            transport.feedback_guard = transition_guard
            result['sequential_enable_confirmed_ids'] = []
            for mid in ids:
                enable_guard_all()
                transitioning = mid
                requested = clock()
                found = transport.feedback_many([enable_request(phase=TrialPhase.ENABLE, motor_id=mid)], (mid,))
                if set(found) != {mid}:
                    raise RuntimeError(f"ID{mid} missing sequential enable reply")
                value, received = found[mid]
                if not requested <= received <= clock():
                    raise RuntimeError(f"ID{mid} sequential enable reply is not fresh")
                guard(value, received, mid, 2)
                transport.latest.update(found)
                confirmed.add(mid)
                result['sequential_enable_confirmed_ids'].append(mid)
                transitioning = None
            enable_guard_all()
            transport.feedback_guard = lambda v, t, i: guard(v, t, i)
        else:
            transport.pre_enable_guard = guard_last_disabled
            transport.feedback_guard = lambda v, t, i: guard(v, t, i)
            wires = [wire for i in ids for wire in (enable_request(phase=TrialPhase.ENABLE, motor_id=i), neutral(i))]
            batch(wires, 2)
        transport.pre_enable_guard = None
        result["enable_confirmed"] = True
        def guard_all():
            for i in ids:
                guard(*transport.latest[i], i)
        transport.pre_send_guard = guard_all
        start = next_send = clock()
        if pose is not None:
            pose_start_ns = ns(start)
            if any(pose_start_ns-c.request_ns > pose.MAX_AGE_NS for c in pose_plan.centers):
                raise RuntimeError("Bounded pose centers aged before active trajectory; no nonzero command")
        peaks = {i: 0. for i in ids}
        while True:
            check_interrupt()
            now = clock()
            for i in ids:
                guard(*transport.latest[i], i)
            elapsed_ns = ns(now)-pose_start_ns if pose is not None else None
            if now - start >= DURATION_S or (pose is not None and elapsed_ns >= pose.DURATION_NS):
                break
            if now > next_send + .02:
                raise RuntimeError("Control scheduling missed by over20ms")
            if pose is None:
                offsets = {i: step5_jog_offset(now - start, signs[i]) for i in ids}
            else:
                offsets = ({i: absolute_targets[i]-centers[i] for i in ids} if fr_current_kp12
                           else pose.offsets_at(pose_plan, elapsed_ns))
                if elapsed_ns >= pose.RAMP_NS and pose_hold_start is None:
                    pose_hold_start = now
            found = batch([active_request(i, centers[i], offsets[i]) for i in ids], 2)
            if pose is not None and pose_hold_start is not None:
                pose_samples.append({"sample_index": len(pose_samples), "checked_ns": ns(clock()),
                    "joints": {i: {"position_rad": value.protocol_position_rad,
                                   "request_ns": ns(pose_requests[i]), "received_ns": ns(received)}
                               for i, (value, received) in found.items()}})
                if settling_observation and not settling_baseline:
                    # Capture all3 first-hold values atomically, only after a
                    # complete fresh batch requested in the actual hold interval.
                    first = pose_samples[-1]
                    for i, item in first['joints'].items():
                        if not (pose_start_ns+pose.RAMP_NS <= item['request_ns'] < item['received_ns']
                                <= first['checked_ns'] <= pose_start_ns+pose.DURATION_NS
                                and first['checked_ns']-item['request_ns'] <= pose.MAX_AGE_NS):
                            raise RuntimeError(f"ID{i} first settling batch has invalid or stale hold source")
                    settling_baseline.update({i: abs(item['position_rad']-absolute_targets[i])
                                              for i,item in first['joints'].items()})
                    result['settling_first_hold_batch'] = {'checked_ns': first['checked_ns'],
                        'joints': {i: dict(item) for i,item in first['joints'].items()},
                        'abs_target_error_rad': dict(settling_baseline)}
            for i, (value, _) in found.items():
                delta = value.protocol_position_rad - centers[i]
                peaks[i] = max(peaks[i], abs(delta))
                motion_sample = {"kind": "leg_trial_motion_sample", "motor_id": i, "elapsed_s": clock()-start,
                      "target_offset_rad": offsets[i], "observed_offset_rad": delta,
                      "velocity_rad_s": value.velocity_rad_s, "torque_feedback_nm": value.torque_nm}
                if rr_hip_kp6 or fr_hip_kp6 or fr_current_kp12 or fr_step4_kp12:
                    motion_sample.update(gain_profile=gain_profile, Kp=gain_plan["Kp_by_motor"][i],
                                         Kd=.15, torque_feedforward_nm=0., motion_phase=motion_phases[i].value)
                if settling_observation:
                    motion_sample.update(observation_profile=observation_profile,
                        final_target_error_rad=value.protocol_position_rad-absolute_targets[i],
                        first_hold_abs_error_rad=settling_baseline.get(i))
                emit(motion_sample)
            next_send += CYCLE_S
            if clock() > next_send + .02:
                raise RuntimeError("Control scheduling missed by over20ms")
            wait(max(0., next_send-clock()))
        result["motion_completed"] = True
        if pose is not None:
            pose_end_ns = ns(clock())
            evaluation = pose.evaluate_hold(pose_plan, pose_samples, run_start_ns=pose_start_ns, ended_ns=pose_end_ns)
            result.update(hold_evaluation=evaluation, trajectory_elapsed=evaluation["elapsed_completed"],
                          arrival_candidate_met=evaluation["arrival_candidate_met"],
                          hold_candidate_met=evaluation["hold_candidate_met"])
            if settling_observation:
                evaluator = (pose.evaluate_fr_settling_observation if fr_settling_observation
                             else pose.evaluate_settling_observation)
                observation = evaluator(pose_plan, pose_samples,
                    run_start_ns=pose_start_ns, ended_ns=pose_end_ns)
                result['settling_observation_evaluation'] = observation
                if not observation['data_complete']:
                    raise RuntimeError("Settling observation incomplete: " + "; ".join(observation['errors']))
            elif not evaluation["hold_candidate_met"]:
                raise RuntimeError("Bounded pose hold candidate rejected: " + "; ".join(evaluation["errors"]))
        for i in ids:
            delta = transport.latest[i][0].protocol_position_rad - centers[i]
            result["motors"][i].update(peak_observed_delta_rad=peaks[i], final_observed_delta_rad=delta,
                final_tracking_error_rad=delta-result["motors"][i]["target_final_offset_rad"])
    except BaseException as error:
        result["errors"].append(repr(error))
    finally:
        if pose is not None and pose_start_ns is not None and pose_end_ns is None:
            try:
                pose_end_ns = ns(clock())
            except BaseException as error:
                result["errors"].append("Pose timing unavailable: " + repr(error))
        if identified:
            try:
                result["stops"] = transport.stop_all(identified)
                result["stop_confirmed"] = all(result["stops"][i]["confirmed"] for i in identified)
                if not result["stop_confirmed"]:
                    result["errors"].append("STOP_UNCONFIRMED: cut motor power immediately")
            except BaseException as error:
                result["errors"].append("STOP_UNCONFIRMED: " + repr(error))
        if pose is not None and pose_plan is not None and pose_start_ns is not None and pose_end_ns is not None and "hold_evaluation" not in result:
            # Stop comes first. A diagnostic exception cannot bypass stopping.
            try:
                evaluation = pose.evaluate_hold(pose_plan, pose_samples, run_start_ns=pose_start_ns,
                                                ended_ns=pose_end_ns)
                result.update(hold_evaluation=evaluation, trajectory_elapsed=evaluation["elapsed_completed"],
                              arrival_candidate_met=evaluation["arrival_candidate_met"],
                              hold_candidate_met=evaluation["hold_candidate_met"])
            except BaseException as error:
                result["errors"].append("Pose diagnostic failed: " + repr(error))
        if (settling_observation and pose_plan is not None and pose_start_ns is not None
                and pose_end_ns is not None and 'settling_observation_evaluation' not in result):
            try:
                evaluator = (pose.evaluate_fr_settling_observation if fr_settling_observation
                             else pose.evaluate_settling_observation)
                result['settling_observation_evaluation'] = evaluator(
                    pose_plan, pose_samples, run_start_ns=pose_start_ns, ended_ns=pose_end_ns)
            except BaseException as error:
                result['errors'].append('Settling diagnostic failed: ' + repr(error))
    result["status"] = ("MOTION_FINISHED_RESET_CONFIRMED" if result["motion_completed"]
                        and result["stop_confirmed"] and not result["errors"] else "ABORTED")
    if settling_observation:
        result['settling_observation_completed'] = bool(result['status'] != 'ABORTED'
            and result.get('settling_observation_evaluation',{}).get('data_complete'))
        result['status'] = (('FR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED'
            if fr_settling_observation else 'RR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED')
            if result['settling_observation_completed'] else 'ABORTED')
    elif pose is not None and result["status"] != "ABORTED":
        result["status"] = "BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED"
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--leg", choices=LEGS, required=True)
    ap.add_argument("--directions", type=int, nargs=3, choices=(-1, 1), required=True)
    ap.add_argument("--expected-uids", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--stationarity-profile", choices=PROFILES, default="legacy-rms-v1")
    ap.add_argument("--position-response-evidence", type=Path)
    for flag in ("execute", "supported", "operator-power-cut-ready"):
        ap.add_argument("--"+flag, action="store_true")
    args = ap.parse_args(argv)
    ids = LEGS[args.leg]
    try:
        expected = validated_uids(json.loads(args.expected_uids.read_text()), ids)
        response_evidence = None
        if args.stationarity_profile == "position-v2":
            if args.position_response_evidence is None:
                raise ValueError("Position-v2 requires selected-leg --position-response-evidence")
            response_evidence = load_position_response_evidence(
                args.position_response_evidence, expected, leg=args.leg)
    except (ValueError, OSError) as error:
        ap.error(str(error))
    plan = {"stationarity_profile": args.stationarity_profile,
            "position_response_evidence": response_evidence, "leg": args.leg, "motor_ids": ids, "directions": args.directions, "order": "foot/thigh/hip",
            "trajectory_duration_s": 5, "ramp_s": 4, "hold_s": 1, "cycle_s": CYCLE_S,
            "active_command_budget_s": 6, "target_offsets_deg": [5*d for d in args.directions],
            "minimum_tx_interval_after_write_s": MIN_TX_INTERVAL_S,
            "adapter_pacing_reliability_verified": False,
            "stationarity_window": {"samples": SETTLED_COUNT, "period_s": SETTLED_PERIOD_S,
                                     "limits": profile_limits(args.stationarity_profile), "automatic_retry": False},
            "Kp": 3., "Kd": .15, "torque_feedforward_nm": 0., "max_drift_deg": 7,
            "max_feedback_age_s": .1, "max_speed_rad_s": .5, "max_temperature_C": 50,
            "watchdog_requested_ms": 200, "watchdog_physical_latency_verified": False,
            "absolute_pose_replay": False, "persistent_hold_after_stop": False,
            "automatic_next_leg": False, "automatic_reenable": False,
            "user_supported": args.supported, "user_at_power_cut": args.operator_power_cut_ready}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    if not args.supported or not args.operator_power_cut_ready:
        ap.error("Supported free legs and present operator at physical power cut are required")
    output = args.output.expanduser().resolve()
    if any((p/".git").exists() for p in (output, *output.parents)):
        ap.error("Use a new private output directory outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    import serial
    signals, handlers, port = [], {}, None
    def interrupted(signum, _frame):
        signals.append(signum)
    def check_interrupt():
        if signals:
            raise InterruptedError(f"signal {signals[0]}")
    sources = [Path(__file__), *(Path(__file__).with_name(n) for n in
               ("rs05_joint_trial.py", "rs05_trial_protocol.py", "can_readonly.py",
                "position_response_evidence.py"))]
    report = {"started_at": datetime.datetime.now().astimezone().isoformat(), "plan": plan,
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, interrupted)
        with (output/"events.jsonl").open("x", buffering=1) as log:
            def emit(event):
                log.write(json.dumps({"wall_time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(),
                                      **event}, allow_nan=False)+"\n")
            check_interrupt()
            port = serial.Serial(port=None, baudrate=921600, timeout=.002, write_timeout=.02,
                                 exclusive=True, rtscts=False, dsrdtr=False, xonxoff=False)
            port.dtr = port.rts = False
            port.port = "/dev/robstride-usb2can"
            port.open()
            report["result"] = run_leg_trial(LegTrialTransport(port, emit, check_interrupt, ids=ids),
                expected, check_interrupt, emit, directions=args.directions, profile=args.stationarity_profile,
                position_response_evidence=args.position_response_evidence,
                position_response_evidence_sha256=response_evidence["sha256"] if response_evidence else None)
    except BaseException as error:
        report["host_error"] = repr(error)
    finally:
        if port is not None:
            try:
                port.close()
            except BaseException as error:
                report["close_error"] = repr(error)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    report.update(signals=signals, completed_at=datetime.datetime.now().astimezone().isoformat())
    (output/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report.get("result", {"status": "HOST_ERROR", "error": report.get("host_error")}), indent=2))
    return int(report.get("result", {}).get("status") != "MOTION_FINISHED_RESET_CONFIRMED")


if __name__ == "__main__":
    raise SystemExit(main())
