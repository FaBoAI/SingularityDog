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
from .rs05_trial_protocol import (Type2Feedback, TrialPhase, decode_type2, enable_request,
    motion_request, stop_request, watchdog_setup_request)
from .rs05_joint_trial import check_feedback, step5_jog_offset

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
                   "abs_velocity_mean_rad_s": .025, "tail_count": 6,
                   "tail_minimum_span_s": .45, "tail_maximum_span_s": .60}
PROFILES = ("legacy-rms-v1", "position-v2")


def profile_limits(profile):
    if profile not in PROFILES:
        raise ValueError("Unknown stationarity profile")
    return dict(POSITION_LIMITS if profile == "position-v2" else SETTLED_LIMITS)


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


def evaluate_settled_window(samples, centers, *, profile="legacy-rms-v1"):
    """Evaluate every sample in one fixed window; never trim, wrap, or retry.

    This deliberately changes the former instantaneous 0.05 rad/s test to a
    position/time/RMS conjunction. The instantaneous 0.5 rad/s guard remains.
    Both times are required so a delayed measurement cannot look stationary.
    """
    ids = selected_ids(centers)
    limits = profile_limits(profile)
    if profile == "position-v2" and ids != LEGS["FR"]:
        raise ValueError("Position-v2 is restricted to FR")
    report = {"passed": False, "sample_count_required": SETTLED_COUNT,
              "sample_period_s": SETTLED_PERIOD_S, "limits": limits, "profile": profile, "warnings": [],
              "errors": [], "motors": {}, "absolute_rest_proven": False,
              "joint_calibration_verified": False,
              "stationarity_spec": ("fixed21_position_OLS_velocity_RMS_v1" if profile == "legacy-rms-v1"
                                    else "fixed21_position_tail_mean_v2")}
    if any(type(i) is not int for i in samples) or set(samples) != set(ids):
        report["errors"].append("Window motor IDs differ from selected leg")
    for mid in ids:
        rows = samples.get(mid, [])
        entry = {"sample_count": len(rows), "samples": rows, "errors": [], "warnings": []}
        report["motors"][mid] = entry
        if len(rows) != SETTLED_COUNT:
            entry["errors"].append(f"Expected exactly {SETTLED_COUNT} samples")
        times, positions, velocities = [], [], []
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
                               ("tail_abs_OLS_slope_rad_s", abs(tail_slope)),
                               ("abs_velocity_mean_rad_s", abs(entry["velocity_mean_rad_s"]))))
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
                  clock=time.monotonic, wait=time.sleep, profile="legacy-rms-v1"):
    ids = selected_ids(transport.ids)
    profile_limits(profile)
    if profile == "position-v2" and ids != LEGS["FR"]:
        raise ValueError("Position-v2 is restricted to FR")
    expected_uids = validated_uids(expected_uids, ids)
    directions = tuple(directions)
    if len(directions) != 3 or any(type(d) is not int or d not in (-1, 1) for d in directions):
        raise ValueError("Three explicit integer directions -1/+1 are required")
    signs = dict(zip(ids, directions))
    result = {"motor_ids": list(ids), "directions": list(directions), "errors": [],
              "stationarity_profile": profile, "motion_completed": False, "stop_confirmed": False, "joint_calibration_verified": False,
              "watchdog_physical_latency_verified": False, "watchdog_persisted": False, "motors": {}}
    identified, centers = [], {}
    def guard(value, received, motor_id, required_mode=2):
        check_feedback(value, centers[motor_id], received, clock(), required_mode=required_mode,
                       max_drift_rad=MAX_DRIFT_RAD)
    def batch(wires, required_mode):
        found = transport.feedback_many(wires, ids)
        if set(found) != set(ids):
            raise RuntimeError("Missing selected-leg feedback")
        transport.latest.update(found)
        for i, (value, received) in found.items():
            guard(value, received, i, required_mode)
        return found
    def neutral(i):
        return motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=centers[i], motor_id=i)
    def check_parameters(i):
        values = [transport.parameter(i, p)["value"] for p in ("position", "current", "voltage")]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise RuntimeError(f"ID{i} nonfinite or malformed initial parameters")
        position, current, voltage = values
        if abs(current) > .05 or not 35 <= voltage <= 43:
            raise RuntimeError(f"ID{i} current/voltage outside trial envelope")
        motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=position, motor_id=i)
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
        for i in ids:
            if transport.parameter(i, "run_mode")["value"] != 0:
                raise RuntimeError(f"ID{i} must already be in operation mode0")
            centers[i], current, voltage = check_parameters(i)
            if abs(centers[i] - initial[i]["feedback"]["protocol_position_rad"]) > .02:
                raise RuntimeError(f"ID{i} Type17/Type2 position mismatch; no wrap guessed")
            result["motors"][i] = {"center_rad": centers[i], "initial_current_A": current,
                "initial_voltage_V": voltage, "target_final_offset_rad": signs[i] * math.radians(5)}
        transport.feedback_guard = lambda v, t, i: guard(v, t, i, 0)
        batch([neutral(i) for i in ids], 0)
        for i in ids:
            result["motors"][i]["watchdog_previous_ticks"] = transport.parameter(i, "can_timeout")["value"]
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
        result["settled_window"] = evaluate_settled_window(samples, centers, profile=profile)
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
        guard_last_disabled()
        # No receives occur inside the enable/neutral write burst. Keep checking
        # the final disabled snapshots before EVERY enable, including after a
        # slow previous write/log operation, until all enabled replies arrive.
        transport.pre_enable_guard = guard_last_disabled
        transport.active_deadline = clock() + ACTIVE_BUDGET_S
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
        peaks = {i: 0. for i in ids}
        while True:
            check_interrupt()
            now = clock()
            for i in ids:
                guard(*transport.latest[i], i)
            if now - start >= DURATION_S:
                break
            if now > next_send + .02:
                raise RuntimeError("Control scheduling missed by over20ms")
            offsets = {i: step5_jog_offset(now - start, signs[i]) for i in ids}
            found = batch([motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=centers[i],
                           offset_rad=offsets[i], motor_id=i) for i in ids], 2)
            for i, (value, _) in found.items():
                delta = value.protocol_position_rad - centers[i]
                peaks[i] = max(peaks[i], abs(delta))
                emit({"kind": "leg_trial_motion_sample", "motor_id": i, "elapsed_s": clock()-start,
                      "target_offset_rad": offsets[i], "observed_offset_rad": delta,
                      "velocity_rad_s": value.velocity_rad_s, "torque_feedback_nm": value.torque_nm})
            next_send += CYCLE_S
            if clock() > next_send + .02:
                raise RuntimeError("Control scheduling missed by over20ms")
            wait(max(0., next_send-clock()))
        result["motion_completed"] = True
        for i in ids:
            delta = transport.latest[i][0].protocol_position_rad - centers[i]
            result["motors"][i].update(peak_observed_delta_rad=peaks[i], final_observed_delta_rad=delta,
                final_tracking_error_rad=delta-result["motors"][i]["target_final_offset_rad"])
    except BaseException as error:
        result["errors"].append(repr(error))
    finally:
        if identified:
            try:
                result["stops"] = transport.stop_all(identified)
                result["stop_confirmed"] = all(result["stops"][i]["confirmed"] for i in identified)
                if not result["stop_confirmed"]:
                    result["errors"].append("STOP_UNCONFIRMED: cut motor power immediately")
            except BaseException as error:
                result["errors"].append("STOP_UNCONFIRMED: " + repr(error))
    result["status"] = ("MOTION_FINISHED_RESET_CONFIRMED" if result["motion_completed"]
                        and result["stop_confirmed"] and not result["errors"] else "ABORTED")
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
            if args.leg != "FR" or args.position_response_evidence is None:
                raise ValueError("Position-v2 requires FR and --position-response-evidence")
            from .position_response_evidence import load_position_response_evidence
            response_evidence = load_position_response_evidence(args.position_response_evidence, expected)
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
               ("rs05_joint_trial.py", "rs05_trial_protocol.py", "can_readonly.py"))]
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
                expected, check_interrupt, emit, directions=args.directions, profile=args.stationarity_profile)
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
