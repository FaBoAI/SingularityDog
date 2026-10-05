"""Bracketed Type17 vs Type2 diagnostics over caller-owned native sessions.

No device is opened here. The caller holds bus/common locks and boot guards.
STOP is state changing: collect requires an independently supported, already
disabled robot. No enable, motion, settings, zeroing, or automatic retries.

Five requests per motor: position/velocity before, STOP feedback, then both
parameters after. Front/rear run concurrently within each phase. This is a
comparison experiment, NOT a 20ms loop or dynamic scale calibration.
"""
from concurrent.futures import ThreadPoolExecutor
import math
import struct
import uuid

from . import can_readonly as codec
from . import native_diagnostic_transport as native
from .can_timing_probe import validate_uids
from .dual_can_pipeline_benchmark import SCOPES

LIMITS = {"maximum_bracket_ms": 100., "maximum_position_drift_deg": .5,
          "maximum_endpoint_speed_rad_s": .15, "maximum_speed_change_rad_s": .1,
          "position_comparison_deg": .2, "velocity_comparison_rad_s": .2}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _frame(value):
    _require(isinstance(value, str), "wire must be hex string")
    try:
        wire = bytes.fromhex(value)
    except ValueError as error:
        raise ValueError("invalid wire hex") from error
    parser = codec.ATParser()
    frames = parser.feed(wire)
    _require(len(wire) == 17 and len(frames) == 1 and parser.discarded_bytes == 0
             and not parser.buffer and frames[0].flags == 4 and len(frames[0].data) == 8,
             "malformed/noncanonical AT frame")
    return frames[0]


def _record(record, scope, phase):
    _require(isinstance(record, dict), "record must be object")
    for name in ("start_ns", "finish_ns", "read_start_ns", "received_ns", "deadline_ns"):
        _require(type(record.get(name)) is int and record[name] > 0, "invalid record time: " + name)
    start, finish, read, received, deadline = [record[n] for n in
        ("start_ns", "finish_ns", "read_start_ns", "received_ns", "deadline_ns")]
    _require(start <= finish <= read <= received < deadline and
             record.get("written") == record.get("received") == 17, "incomplete/noncausal native record")
    tx, rx = _frame(record.get("tx_hex")), _frame(record.get("rx_hex"))
    mid = tx.destination
    _require(mid in SCOPES[scope], "cross-bus motor ID")
    if phase == "identity":
        _require(tx.wire == codec.read_request(mid) and rx.can_id == (mid << 8 | 0xfe),
                 "identity request/reply mismatch")
        key, value = (mid, "identity"), rx.data.hex()
    elif phase == "feedback":
        _require(tx.wire == native.stop_wire(mid) and rx.can_id == (2 << 24 | mid << 8 | 0xfd)
                 and rx.data[:3] != b"\x00\xc4\x56", "STOP reply fault/mode/version/protocol mismatch")
        p, v, torque, temp = struct.unpack(">4H", rx.data)
        key, value = (mid, "feedback"), {
            "position_rad_candidate": p*(2.*12.57)/65535.-12.57,
            "velocity_rad_s_candidate": v*100./65535.-50.,
            "position_u16": p, "velocity_u16": v, "torque_u16": torque,
            "temperature_c": temp/10., "fault_bits": 0, "mode_state": 0}
    else:
        parameter = {b"\x19\x70": "position", b"\x1b\x70": "velocity"}.get(tx.data[:2])
        _require(parameter is not None and tx.wire == codec.read_request(mid, parameter)
                 and rx.can_id == (17 << 24 | mid << 8 | 0xfd), "Type17 request/reply mismatch")
        decoded = codec.decode_reply(rx, mid, parameter)
        _require(decoded["ok"], "Type17 status/reserved/value rejected")
        key, value = (mid, parameter), decoded["value"]
    return key, {"value": value, "request_ns": start, "write_finished_ns": finish,
                 "received_ns": received, "host_midpoint_ns": (start+received)//2}


def _phase(evidence, name):
    _require(isinstance(evidence, dict) and set(evidence) == set(SCOPES), "phase requires both buses")
    result = {}
    for scope, exchange in evidence.items():
        _require(isinstance(exchange, dict) and exchange.get("rejected_hex") == ""
                 and isinstance(exchange.get("records"), list), "failed/incomplete exchange evidence")
        for raw in exchange["records"]:
            key, row = _record(raw, scope, name)
            _require(key not in result, "duplicate phase record")
            result[key] = row
    fields = ("identity",) if name == "identity" else ("feedback",) if name == "feedback" else ("position", "velocity")
    _require(set(result) == {(i, p) for i in range(1, 13) for p in fields}, "phase missing motor or parameter")
    return result


def _extent(records):
    return min(v["request_ns"] for v in records.values()), max(v["received_ns"] for v in records.values())


def _interpolate(before, feedback, after):
    """Host midpoint interpolation + possible timing interval under a linear model.

    Host intervals are not sensor timestamps. The returned spread cannot bound
    arbitrary acceleration, jumps, or sensor-internal buffering delays.
    """
    b, f, a = before["host_midpoint_ns"], feedback["host_midpoint_ns"], after["host_midpoint_ns"]
    _require(before["received_ns"] <= feedback["request_ns"]
             and feedback["received_ns"] <= after["request_ns"] and b < f < a,
             "feedback not bracketed by independent Type17 observations")
    alpha = (f-b)/(a-b)
    lo = (feedback["request_ns"]-before["received_ns"])/(after["received_ns"]-before["received_ns"])
    hi = (feedback["received_ns"]-before["request_ns"])/(after["request_ns"]-before["request_ns"])
    _require(0 <= lo <= alpha <= hi <= 1, "invalid host interpolation fractions")
    value = before["value"] + alpha*(after["value"]-before["value"])
    candidates = [before["value"] + x*(after["value"]-before["value"]) for x in (lo, hi)]
    return {"before": before, "after": after, "host_midpoint_fraction": alpha,
            "host_interval_fraction_range": [lo, hi], "interpolated_type17": value,
            "host_time_linear_model_spread": max(abs(v-value) for v in candidates),
            "bracket_ms": (after["received_ns"]-before["request_ns"])/1e6}


def _sample_statistics(values):
    """Describe every saved sample; never select a passing subset or fit a scale."""
    count = len(values)
    mean = math.fsum(values)/count
    return {"samples": count, "minimum": min(values), "maximum": max(values),
            "range": max(values)-min(values), "mean": mean,
            "rms": math.sqrt(math.fsum(v*v for v in values)/count),
            "population_std": math.sqrt(math.fsum((v-mean)**2 for v in values)/count)}


def _host_endpoint_diagnostics(position, velocity):
    """Keep separately read host endpoints distinct from sensor-time velocity."""
    position_dt = position["after"]["host_midpoint_ns"]-position["before"]["host_midpoint_ns"]
    velocity_dt = velocity["after"]["host_midpoint_ns"]-velocity["before"]["host_midpoint_ns"]
    return {
        "position_midpoint_delta_ns": position_dt,
        "velocity_midpoint_delta_ns": velocity_dt,
        "position_finite_difference_rad_s":
            (position["after"]["value"]-position["before"]["value"])*1e9/position_dt,
        "velocity_minus_position_before_midpoint_ns":
            velocity["before"]["host_midpoint_ns"]-position["before"]["host_midpoint_ns"],
        "velocity_minus_position_after_midpoint_ns":
            velocity["after"]["host_midpoint_ns"]-position["after"]["host_midpoint_ns"],
        "position_and_velocity_read_together": False, "sensor_sample_time_verified": False,
        "affects_comparison_result": False,
        "scope": "Position difference divided by host midpoint separation; position and velocity "
                 "are read separately. This is a descriptive estimate, not sensor-time velocity "
                 "or velocity calibration.",
    }


def analyze_feedback_comparison(evidence, expected_uids):
    """Pure saved-evidence replay. No branch correction or physical approval."""
    expected = validate_uids(expected_uids)
    _require(isinstance(evidence, dict) and evidence.get("kind") == "native_feedback_comparison"
             and evidence.get("supported_disabled") is True
             and evidence.get("motor_enable_sent") is False and evidence.get("learned_targets_sent") is False,
             "explicit supported-disabled diagnostic context required")
    boot = evidence.get("boot_id")
    _require(isinstance(boot, str) and str(uuid.UUID(boot)) == boot, "invalid boot binding")
    identities = _phase(evidence.get("identity"), "identity")
    _require(all(identities[i, "identity"]["value"] == expected[i] for i in range(1, 13)),
             "fresh UID mismatch")
    cycles = evidence.get("cycles")
    _require(isinstance(cycles, list) and 1 <= len(cycles) <= 5, "require one to five complete comparison cycles")
    rows, timings = [], []
    previous_end = _extent(identities)[1]
    for n, cycle in enumerate(cycles, 1):
        _require(cycle.get("cycle") == n, "comparison cycle sequence invalid")
        before, feedback, after = (_phase(cycle.get(p), p) for p in ("before", "feedback", "after"))
        bs, be = _extent(before); fs, fe = _extent(feedback); ats, ate = _extent(after)
        _require(previous_end <= bs <= be <= fs <= fe <= ats <= ate, "phase order/UID epoch chronology invalid")
        previous_end = ate
        timings.append({"cycle": n, "before_ms": (be-bs)/1e6, "feedback_ms": (fe-fs)/1e6,
                        "after_ms": (ate-ats)/1e6, "three_phase_span_ms": (ate-bs)/1e6})
        for mid in range(1, 13):
            fb = feedback[mid, "feedback"]
            position = _interpolate(before[mid, "position"], fb, after[mid, "position"])
            velocity = _interpolate(before[mid, "velocity"], fb, after[mid, "velocity"])
            pd = after[mid, "position"]["value"]-before[mid, "position"]["value"]
            vd = after[mid, "velocity"]["value"]-before[mid, "velocity"]["value"]
            pos_error = fb["value"]["position_rad_candidate"]-position["interpolated_type17"]
            vel_error = fb["value"]["velocity_rad_s_candidate"]-velocity["interpolated_type17"]
            wrapped = math.atan2(math.sin(pos_error), math.cos(pos_error))
            gates = {
                "short_host_bracket": max(position["bracket_ms"], velocity["bracket_ms"]) <= LIMITS["maximum_bracket_ms"],
                "position_endpoints_stable": abs(math.degrees(pd)) <= LIMITS["maximum_position_drift_deg"],
                "speed_endpoints_small": max(abs(before[mid, "velocity"]["value"]), abs(after[mid, "velocity"]["value"]))
                                         <= LIMITS["maximum_endpoint_speed_rad_s"],
                "speed_endpoints_stable": abs(vd) <= LIMITS["maximum_speed_change_rad_s"],
            }
            eligible = all(gates.values())
            modulo_agrees = abs(math.degrees(wrapped)) <= LIMITS["position_comparison_deg"] and abs(vel_error) <= LIMITS["velocity_comparison_rad_s"]
            direct_agrees = abs(math.degrees(pos_error)) <= LIMITS["position_comparison_deg"] and abs(vel_error) <= LIMITS["velocity_comparison_rad_s"]
            rows.append({"cycle": n, "motor_id": mid, "position": position, "velocity": velocity,
                "feedback": fb, "position_endpoint_change_deg": math.degrees(pd),
                "velocity_endpoint_change_rad_s": vd, "direct_position_error_deg": math.degrees(pos_error),
                "mod_2pi_position_error_deg": math.degrees(wrapped), "velocity_error_rad_s": vel_error,
                "diagnostic_gates": gates, "endpoint_stationarity_heuristic_passed": eligible,
                "failed_diagnostic_gates": sorted(name for name, passed in gates.items() if not passed),
                "host_endpoint_diagnostics": _host_endpoint_diagnostics(position, velocity),
                "comparison_result": ("STATIC_CANDIDATE_AGREES" if direct_agrees else
                                      "STATIC_MODULO_ONLY_BRANCH_UNRESOLVED" if modulo_agrees else "STATIC_CANDIDATE_DIFFERS")
                                     if eligible else "INCONCLUSIVE_MOTION_OR_TIMING",
                "direct_comparison_agrees": eligible and direct_agrees,
                "modulo_comparison_agrees": eligible and modulo_agrees,
                "branch_adjustment_applied": False, "dynamic_scale_validated": False})
    per_id = {}
    for mid in range(1, 13):
        selected = [r for r in rows if r["motor_id"] == mid]
        # Include inconclusive/different cycles. Type17 endpoints and Type2
        # feedback remain separate measured quantities; no pooled estimator.
        statistics = {
            "all_cycles_included": True, "sample_filtering_applied": False,
            "affects_comparison_result": False,
            "type17_position_endpoints_rad": _sample_statistics(
                [r["position"][p]["value"] for r in selected for p in ("before", "after")]),
            "type17_velocity_endpoints_rad_s": _sample_statistics(
                [r["velocity"][p]["value"] for r in selected for p in ("before", "after")]),
            "type2_feedback_velocity_rad_s_candidate": _sample_statistics(
                [r["feedback"]["value"]["velocity_rad_s_candidate"] for r in selected]),
            "failed_gate_counts": {name: sum(not r["diagnostic_gates"][name] for r in selected)
                                   for name in selected[0]["diagnostic_gates"]},
            "scope": "Statistics of all existing decoded samples, including failed gates. "
                     "Population standard deviation describes this finite record; no scale, "
                     "stationarity or runtime approval is inferred.",
        }
        per_id[str(mid)] = {"samples": len(selected),
            "static_bracket_samples": sum(r["endpoint_stationarity_heuristic_passed"] for r in selected),
            "max_abs_direct_position_error_deg": max(abs(r["direct_position_error_deg"]) for r in selected),
            "max_abs_mod_2pi_position_error_deg": max(abs(r["mod_2pi_position_error_deg"]) for r in selected),
            "max_abs_velocity_error_rad_s": max(abs(r["velocity_error_rad_s"]) for r in selected),
            "max_bracket_ms": max(max(r[p]["bracket_ms"] for p in ("position", "velocity")) for r in selected),
            "all_sample_statistics": statistics,
            "all_direct_static_comparisons_agree": all(r["direct_comparison_agrees"] for r in selected),
            "all_modulo_static_comparisons_agree": all(r["modulo_comparison_agrees"] for r in selected)}
    return {"status": "COMPLETE_DIAGNOSTIC", "kind": "native_feedback_comparison_report",
            "boot_id": boot, "cycles_completed": len(cycles), "per_motor": per_id, "rows": rows,
            "phase_timings": timings, "limits": dict(LIMITS),
            "type2_candidate_ranges": {"position_rad": [-12.57, 12.57], "velocity_rad_s": [-50., 50.]},
            "motor_enable_sent": False, "learned_targets_sent": False, "approved_for_runtime": False,
            "stop_proxy_changes_motor_state": True, "sensor_sample_time_verified": False,
            "dynamic_scale_validated": False, "full_controller_50Hz_verified": False,
            "limitations": [
                "STOP responses change motor state; this is only for already disabled, independently supported hardware.",
                "Static agreement cannot validate velocity scale, motor drive mode, or full-cycle 20ms performance.",
                "Interpolation uses host request/receive midpoints, not sensor acquisition time.",
                "Reported timing spread assumes linear change; it does not bound unknown motion, acceleration, or buffering.",
                "Endpoint stability does not prove the joint stayed still between samples.",
                "Modulo-2pi comparison is reported separately; no raw value, calibration, or commanded angle is corrected.",
                "Candidate ranges are fixed; no regression, fitted scale, learned target or runtime approval is produced."]}


def collect_feedback_comparison(sessions, expected_uids, *, boot_id, supported_disabled,
                                cycles=3, check=lambda: None):
    """Use caller-owned native sessions. Return (report, serializable evidence).

    Fresh identity checks precede any STOP; all phase futures are joined before
    proceeding. One failure ends the run, retaining partial exchange evidence.
    The caller retains ownership of serial/boot/cancel descriptors and locks.
    """
    expected = validate_uids(expected_uids)
    _require(supported_disabled is True, "STOP comparison requires supported, already disabled robot")
    _require(type(cycles) is int and 1 <= cycles <= 5, "comparison cycles must be 1..5")
    _require(set(sessions) == set(SCOPES) and str(uuid.UUID(boot_id)) == boot_id, "session scopes/boot binding invalid")
    for scope, session in sessions.items():
        _require(session.first_id == SCOPES[scope][0] and session.stop_proxy is True
                 and session.boot_id == boot_id.encode("ascii") and session.boot_fd >= 0,
                 "native session scope/STOP permission/boot binding mismatch")
    evidence = {"kind": "native_feedback_comparison", "boot_id": boot_id,
                "supported_disabled": True, "motor_enable_sent": False, "learned_targets_sent": False,
                "cycles_requested": cycles, "identity": {}, "cycles": []}
    def one(scope, wires):
        check()
        return sessions[scope].exchange(wires)
    try:
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="feedback-compare") as pool:
            def phase(target, kind):
                futures = {}
                for scope, ids in SCOPES.items():
                    wires = ([codec.read_request(i) for i in ids] if kind == "identity" else
                             [native.stop_wire(i) for i in ids] if kind == "feedback" else
                             [codec.read_request(i, p) for p in ("position", "velocity") for i in ids])
                    futures[scope] = pool.submit(one, scope, wires)
                failure = None
                for scope, future in futures.items():
                    try:
                        target[scope] = native.exchange_evidence(*future.result())
                    except BaseException as error:
                        target[scope] = {"error": type(error).__name__+": "+str(error)}
                        if isinstance(error, native.ExchangeError):
                            target[scope]["failed_native_exchange"] = native.exchange_evidence(error.records, error.stats)
                        failure = failure or error
                if failure:
                    raise failure
                check()
            phase(evidence["identity"], "identity")
            identities = _phase(evidence["identity"], "identity")
            _require(all(identities[i, "identity"]["value"] == expected[i] for i in range(1, 13)), "fresh UID mismatch")
            for n in range(1, cycles+1):
                cycle = {"cycle": n, "before": {}, "feedback": {}, "after": {}}
                evidence["cycles"].append(cycle)
                for name in ("before", "feedback", "after"):
                    phase(cycle[name], name)
        report = analyze_feedback_comparison(evidence, expected)
        report["errors"] = []
    except BaseException as error:
        report = {"status": "ABORTED", "errors": [type(error).__name__+": "+str(error)],
                  "motor_enable_sent": False, "learned_targets_sent": False,
                  "approved_for_runtime": False, "dynamic_scale_validated": False,
                  "full_controller_50Hz_verified": False}
    return report, evidence
