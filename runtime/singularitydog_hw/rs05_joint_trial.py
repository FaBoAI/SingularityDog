"""Supervised single-ID, supported RS05 micro-motion trial; never a walking controller.

Physical support and an operator at motor-power cutoff are mandatory. Limits below
are conservative trial choices, not proven motor torque/speed limits. Watchdog
readback is not proof of physical communication-loss stop latency.
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
from .rs05_trial_protocol import (TrialPhase, enable_request, stop_request,
    watchdog_setup_request, motion_request, decode_type2)

MAX_DRIFT_RAD = math.radians(3)
MAX_SPEED_RAD_S = .5
MAX_TEMP_C = 50
MAX_FEEDBACK_AGE_S = .1
MAX_DURATION_S = 3.0
ACTIVE_BUDGET_S = 4.0
STEP5_DURATION_S = 5.0
STEP5_ACTIVE_BUDGET_S = 6.0


def check_feedback(feedback, center, received_at, now, *, required_mode=2,
                   max_drift_rad=MAX_DRIFT_RAD, max_age_s=MAX_FEEDBACK_AGE_S):
    """Checks precede the next nonzero-gain frame; no modulo normalization."""
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("Feedback age limit must be positive and finite")
    values = (feedback.protocol_position_rad, feedback.velocity_rad_s,
              feedback.temperature_c, center, received_at, now)
    if not all(math.isfinite(x) for x in values):
        raise RuntimeError("Nonfinite feedback or time")
    if not 0 <= now - received_at <= max_age_s:
        raise RuntimeError("Stale feedback")
    if feedback.mode_state != required_mode or feedback.fault_bits:
        raise RuntimeError("Unexpected motor mode or fault")
    if abs(feedback.protocol_position_rad - center) > max_drift_rad:
        raise RuntimeError("Position departed from trial center beyond selected threshold")
    if abs(feedback.velocity_rad_s) > MAX_SPEED_RAD_S:
        raise RuntimeError("Speed exceeded trial threshold")
    if not -10 <= feedback.temperature_c < MAX_TEMP_C:
        raise RuntimeError("Temperature outside trial threshold")


def trajectory_offset(elapsed, amplitude_deg=1):
    if not math.isfinite(elapsed) or not 0 <= elapsed <= MAX_DURATION_S:
        raise ValueError("Outside fixed3s trajectory")
    # Smooth half-cosine, +1deg peak at1.5s, returns at3s.
    if amplitude_deg not in (1, 3) or type(amplitude_deg) is not int:
        raise ValueError("Only explicit1deg or3deg trajectory amplitudes")
    return math.radians(amplitude_deg) * .5 * (1 - math.cos(2 * math.pi * elapsed / MAX_DURATION_S))


def jog_offset(elapsed, direction=1):
    """One signed1deg jog:2s smooth ramp then1s bounded hold, followed by stop.

    This is a motor-coordinate direction probe, not a calibrated pose or a
    persistent hold. The final reset removes active control of this joint.
    """
    if type(elapsed) not in (int, float) or not math.isfinite(elapsed) or not 0 <= elapsed <= MAX_DURATION_S:
        raise ValueError("Outside fixed3s jog")
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("Jog direction must be integer-1 or+1")
    progress = min(elapsed / 2., 1.)
    return direction * math.radians(1) * .5 * (1 - math.cos(math.pi * progress))


def validate_trajectory(gain_profile, trajectory_kind, direction):
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("Direction must be integer-1 or+1")
    if trajectory_kind not in ("return", "jog"):
        raise ValueError("Unknown trajectory kind")
    if trajectory_kind == "return" and direction != 1:
        raise ValueError("Return trial keeps its original positive trajectory")
    if trajectory_kind == "jog" and gain_profile not in ("step2", "step5"):
        raise ValueError("Jog requires explicit fixed step2 or step5 profile")
    if gain_profile == "step5" and trajectory_kind != "jog":
        raise ValueError("Step5 permits only a fixed5deg jog,4s ramp+1s hold")


def step5_jog_offset(elapsed, direction=1):
    """Explicit five-degree jog,4s ramp+1s hold; never chosen automatically."""
    if type(elapsed) not in (int, float) or not math.isfinite(elapsed) or not 0 <= elapsed <= STEP5_DURATION_S:
        raise ValueError("Outside fixed5s jog")
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("Jog direction must be integer-1 or+1")
    progress = min(elapsed / 4., 1.)
    return direction * math.radians(5) * .5 * (1 - math.cos(math.pi * progress))


class TrialTransport:
    """Single owner. No retry of failed command; stopping remains independent."""
    def __init__(self, serial_port, emit, check_interrupt=lambda: None, *, motor_id=1):
        if type(motor_id) is not int or not 1 <= motor_id <= 12:
            raise ValueError("Select exactly one motor ID in1..12")
        self.motor_id = motor_id
        self.serial, self.emit = serial_port, emit
        self.parser = ATParser()
        self.relaxed_log = False
        self.active_deadline = None
        self.feedback_guard = None
        self.check_interrupt = check_interrupt

    def log(self, event):
        try:
            self.emit({"monotonic_ns": time.monotonic_ns(), **event})
        except BaseException:
            if not self.relaxed_log:
                raise

    def receive(self):
        chunk = self.serial.read(min(max(self.serial.in_waiting, 1), 2048))
        received_at = time.monotonic()
        if not chunk:
            return []
        self.log({"kind": "can_rx_bytes", "hex": chunk.hex()})
        frames = self.parser.feed(chunk)
        for f in frames:
            self.log({"kind": "can_rx_frame", **f.record()})
            if f.kind == 21 and f.source == self.motor_id and not self.relaxed_log:
                raise RuntimeError("Selected motor reported a Type21 fault")
            if f.kind == 2 and f.source == self.motor_id and not self.relaxed_log:
                value = decode_type2(f, motor_id=self.motor_id)
                if value.fault_bits:
                    raise RuntimeError("Fault in received feedback")
                if self.feedback_guard is not None:
                    self.feedback_guard(value, received_at)
        if self.parser.discarded_bytes and not self.relaxed_log:
            raise RuntimeError("Serial parser discarded bytes")
        return [(f, received_at) for f in frames]

    def fresh_boundary(self):
        end = time.monotonic() + .03
        while self.serial.in_waiting:
            for f, _ in self.receive():
                if f.kind == 2 and f.source == self.motor_id:
                    if decode_type2(f, motor_id=self.motor_id).fault_bits:
                        raise RuntimeError("Fault in pending feedback")
            if time.monotonic() > end:
                raise RuntimeError("Input backlog")
        if self.parser.buffer:
            raise RuntimeError("Partial frame before command")

    def send(self, wire):
        frames = ATParser().feed(wire)
        if len(frames) != 1 or frames[0].destination != self.motor_id or frames[0].kind not in (0, 1, 3, 4, 17, 18):
            raise ValueError("Trial transport permits only the selected motor ID")
        if frames[0].kind != 4:
            self.check_interrupt()
        if (frames[0].kind in (1, 3) and self.active_deadline is not None
                and time.monotonic() >= self.active_deadline):
            raise RuntimeError("Active trial deadline reached before command")
        if self.serial.write(wire) != len(wire):
            raise IOError("Partial write")
        self.log({"kind": "can_tx", "hex": wire.hex(), "type": frames[0].kind, "motor_id": self.motor_id})

    def exchange(self, wires, accept, *, stopping=False):
        boundary_valid = True
        if stopping:
            # Never allow a poisoned parser, backlog, interrupted wait, or log failure
            # to prevent a bounded physical stop attempt.
            self.relaxed_log = True
            try:
                self.fresh_boundary()
            except BaseException:
                self.parser = ATParser()
                boundary_valid = False
        else:
            self.fresh_boundary()
        for wire in wires:
            self.send(wire)
        deadline = time.monotonic() + .08
        if self.active_deadline is not None and not stopping:
            deadline = min(deadline, self.active_deadline)
        last = None
        last_at = None
        while time.monotonic() < deadline:
            for frame, received_at in self.receive():
                value = accept(frame)
                if value is not None:
                    last, last_at = value, received_at
            if last is not None and time.monotonic() - last_at >= .004:
                if not boundary_valid:
                    raise RuntimeError("Stop sent but fresh reply boundary could not be established")
                return last, last_at
        raise TimeoutError("No fresh accepted response within80ms")

    def parameter(self, name=None):
        def accept(frame):
            if matches(frame, self.motor_id, name):
                result = decode_reply(frame, self.motor_id, name)
                if not result["ok"]:
                    raise RuntimeError(f"Parameter rejected: {name}")
                return result
        value, _ = self.exchange([read_request(self.motor_id, name)], accept)
        return value

    def feedback(self, wires, *, stopping=False):
        def accept(frame):
            if frame.kind == 2 and frame.source == self.motor_id:
                value = decode_type2(frame, motor_id=self.motor_id)
                if stopping and value.mode_state != 0:
                    return None
                return value
        value, when = self.exchange(wires, accept, stopping=stopping)
        self.log({"kind": "trial_feedback", "received_monotonic_s": when, **asdict(value)})
        return value, when


def run_trial(transport, expected_uid, check_interrupt, emit, *, clock=time.monotonic, wait=time.sleep,
              gain_profile="initial", motor_id=1, trajectory_kind="return", direction=1):
    if type(motor_id) is not int or not 1 <= motor_id <= 12 or motor_id != transport.motor_id:
        raise ValueError("Trial and transport must select the same single ID1..12")
    if gain_profile not in ("initial", "step2", "visible", "step5"):
        raise ValueError("Unknown explicit gain profile")
    validate_trajectory(gain_profile, trajectory_kind, direction)
    position_phase = {"initial": TrialPhase.POSITION, "step2": TrialPhase.POSITION_STEP2,
                      "visible": TrialPhase.POSITION_VISIBLE, "step5": TrialPhase.POSITION_STEP5}[gain_profile]
    amplitude_deg = {"visible": 3, "step5": 5}.get(gain_profile, 1)
    max_drift_rad = math.radians({"visible": 5, "step5": 7}.get(gain_profile, 3))
    duration_s = STEP5_DURATION_S if gain_profile == "step5" else MAX_DURATION_S
    active_budget_s = STEP5_ACTIVE_BUDGET_S if gain_profile == "step5" else ACTIVE_BUDGET_S
    def guard(value, received, now, required_mode=2):
        check_feedback(value, center, received, now, required_mode=required_mode, max_drift_rad=max_drift_rad)
    result = {"motor_id": motor_id, "errors": [], "stop_confirmed": False,
              "gain_profile": gain_profile,
              "trajectory_kind": trajectory_kind, "direction": direction,
              "target_final_offset_rad": direction * math.radians(amplitude_deg) if trajectory_kind == "jog" else 0.,
              "joint_calibration_verified": False, "watchdog_physical_latency_verified": False,
              "watchdog_persisted": False, "motion_completed": False}
    identified = False
    try:
        check_interrupt()
        uid = transport.parameter()["mcu_uid_hex"]
        if uid != expected_uid:
            raise RuntimeError("Selected ID identity differs from supported baseline")
        identified = True
        feedback, when = transport.feedback([stop_request(phase=TrialPhase.STOP, motor_id=motor_id)])
        check_interrupt()
        if feedback.mode_state != 0 or feedback.fault_bits:
            raise RuntimeError("Initial reset/fault check failed")
        if transport.parameter("run_mode")["value"] != 0:
            raise RuntimeError("Selected motor must already be in operation mode0; no mode rewrite")
        center = transport.parameter("position")["value"]
        current = transport.parameter("current")["value"]
        voltage = transport.parameter("voltage")["value"]
        check_interrupt()
        if abs(current) > .05 or not 35 <= voltage <= 43:
            raise RuntimeError("Initial current/voltage outside trial envelope")
        if abs(center - feedback.protocol_position_rad) > .02:
            raise RuntimeError("Type17/Type2 position conventions do not match; no wrap guessed")
        result.update(center_rad=center, initial_current_A=current, initial_voltage_V=voltage)
        # Validate the complete trial's encoding headroom while still stopped.
        motion_request(phase=position_phase, center_rad=center, motor_id=motor_id)
        neutral = motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=center, motor_id=motor_id)
        # A disabled-state neutral frame is an extra precaution, never a claim that
        # firmware retains it across enable. Repeat immediately after Type3.
        transport.feedback_guard = lambda value, received: guard(value, received, clock(), required_mode=0)
        feedback, when = transport.feedback([neutral])
        guard(feedback, when, clock(), required_mode=0)
        check_interrupt()
        result["watchdog_previous_ticks"] = transport.parameter("can_timeout")["value"]
        transport.fresh_boundary()
        transport.send(watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=motor_id))
        wait(.015)
        if transport.parameter("can_timeout")["value"] != 4000:
            raise RuntimeError("Watchdog200ms readback mismatch")
        result["watchdog_readback_ticks"] = 4000
        check_interrupt()
        transport.active_deadline = clock() + active_budget_s
        transport.feedback_guard = lambda value, received: guard(value, received, clock())
        feedback, when = transport.feedback([enable_request(phase=TrialPhase.ENABLE, motor_id=motor_id), neutral])
        guard(feedback, when, clock())
        result["enable_confirmed"] = True
        start = clock()
        next_send = start
        peaks = [abs(feedback.protocol_position_rad - center)]
        while True:
            check_interrupt()
            now = clock()
            if now - start >= duration_s:
                break
            guard(feedback, when, now)
            offset = (step5_jog_offset(now - start, direction) if gain_profile == "step5"
                      else jog_offset(now - start, direction) if trajectory_kind == "jog"
                      else trajectory_offset(now - start, amplitude_deg))
            wire = motion_request(phase=position_phase, center_rad=center, offset_rad=offset, motor_id=motor_id)
            feedback, when = transport.feedback([wire])
            guard(feedback, when, clock())
            peaks.append(abs(feedback.protocol_position_rad - center))
            emit({"kind": "trial_motion_sample", "elapsed_s": clock()-start,
                  "target_offset_rad": offset, "observed_offset_rad": feedback.protocol_position_rad-center,
                  "velocity_rad_s": feedback.velocity_rad_s, "torque_feedback_nm": feedback.torque_nm})
            next_send += .02
            if clock() > next_send + .02:
                raise RuntimeError("Control scheduling missed by over20ms")
            wait(max(0, next_send-clock()))
        result.update(motion_completed=True, peak_observed_delta_rad=max(peaks),
                      final_observed_delta_rad=feedback.protocol_position_rad-center,
                      final_tracking_error_rad=feedback.protocol_position_rad-center-result["target_final_offset_rad"])
    except BaseException as error:
        result["errors"].append(repr(error))
    finally:
        if identified:
            try:
                feedback, _ = transport.feedback([stop_request(phase=TrialPhase.STOP, motor_id=motor_id)], stopping=True)
                result["stop_feedback"] = asdict(feedback)
                result["stop_confirmed"] = feedback.mode_state == 0
                if feedback.fault_bits:
                    result["errors"].append("Motor fault reported at final stop")
            except BaseException as error:
                result["errors"].append("STOP_UNCONFIRMED: " + repr(error))
    result["status"] = ("MOTION_FINISHED_RESET_CONFIRMED" if result["motion_completed"] and result["stop_confirmed"]
                        and not result["errors"] else "ABORTED")
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--supported", action="store_true")
    ap.add_argument("--operator-power-cut-ready", action="store_true")
    ap.add_argument("--expected-uid", required=True)
    ap.add_argument("--motor-id", type=int, choices=range(1, 13), default=1)
    ap.add_argument("--gain-profile", choices=("initial", "step2", "visible", "step5"), default="initial",
                    help="Fixed profile chosen explicitly after reviewing the preceding trial; never automatic")
    ap.add_argument("--trajectory-kind", choices=("return", "jog"), default="return")
    ap.add_argument("--direction", type=int, choices=(-1, 1), default=1,
                    help="Motor-coordinate sign for jog only; does not identify a robot joint direction")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    if len(args.expected_uid) != 16 or any(c not in "0123456789abcdef" for c in args.expected_uid):
        ap.error("expected-uid must be16 lowercase hex digits from the selected motor in prior read-only baseline")
    try:
        validate_trajectory(args.gain_profile, args.trajectory_kind, args.direction)
    except ValueError as error:
        ap.error(str(error))
    amplitude_deg = {"visible": 3, "step5": 5}.get(args.gain_profile, 1)
    step5 = args.gain_profile == "step5"
    plan = {"motor_id": args.motor_id, "trajectory_duration_s": 5 if step5 else 3,
            "active_command_budget_s": 6 if step5 else 4,
            "physical_stop_delivery_latency_verified": False,
            "target_max_offset_deg": amplitude_deg,
            "gain_profile": args.gain_profile,
            "trajectory_kind": args.trajectory_kind, "direction": args.direction,
            "jog_ramp_s": (4 if step5 else 2) if args.trajectory_kind == "jog" else None,
            "jog_hold_s": 1 if args.trajectory_kind == "jog" else None,
            "persistent_hold_after_stop": False,
            "absolute_pose_replay": False,
            "target_final_offset_deg": args.direction * amplitude_deg if args.trajectory_kind == "jog" else 0,
            "Kp": {"initial": .5, "step2": 5., "visible": 3., "step5": 3.}[args.gain_profile],
            "Kd": {"initial": .02, "step2": .05, "visible": .15, "step5": .15}[args.gain_profile], "torque_feedforward_nm": 0,
            "voltage_range_V": [35, 43], "max_feedback_age_s": .1,
            "max_observed_drift_deg": {"visible": 5, "step5": 7}.get(args.gain_profile, 3), "max_observed_speed_rad_s": .5,
            "max_temperature_C": 50, "watchdog_requested_ms": 200,
            "hard_torque_or_speed_cap_verified": False,
            "user_supported": args.supported, "user_at_power_cut": args.operator_power_cut_ready,
            "automatic_gain_increase": False, "automatic_reenable": False}
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
    signals, handlers = [], {}
    def interrupted(signum, _frame):
        signals.append(signum)
    def check_interrupt():
        if signals:
            raise InterruptedError(f"signal {signals[0]}")
    report = {"started_at": datetime.datetime.now().astimezone().isoformat(), "plan": plan,
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (Path(__file__), Path(__file__).with_name("rs05_trial_protocol.py"),
                            Path(__file__).with_name("can_readonly.py"))}}
    port = None
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
            report["result"] = run_trial(TrialTransport(port, emit, check_interrupt, motor_id=args.motor_id), args.expected_uid,
                                         check_interrupt, emit, gain_profile=args.gain_profile, motor_id=args.motor_id,
                                         trajectory_kind=args.trajectory_kind, direction=args.direction)
    except BaseException as error:
        report["host_error"] = repr(error)
    finally:
        if port is not None:
            port.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    report["signals"] = signals
    report["completed_at"] = datetime.datetime.now().astimezone().isoformat()
    (output/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report.get("result", {"status": "HOST_ERROR", "error": report.get("host_error")}), indent=2))
    return int(report.get("result", {}).get("status") != "MOTION_FINISHED_RESET_CONFIRMED")


if __name__ == "__main__":
    raise SystemExit(main())
