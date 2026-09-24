"""Screen a SAVED joint_snapshot against explicit calibration candidates.

Offline only: no device, transport, policy, calibration write, or motor command.
Example from runtime/ (all inputs and output are local private files):

    python -m singularitydog_hw.prestand_check --capture /private/snapshot \
        --calibration /private/calibration-candidates.json \
        --rr-overlay /private/rr-hip-zero-candidate.json \
        --output /private/new-prestand-report.json

Omit --rr-overlay to use the original candidates unchanged. Output must be new
and outside Git; no --force option exists. Return 0 means an offline report was
written, even when its screen is blocked; malformed input returns 2. A PASS
describes these saved samples only, never live readiness or motor permission.
Nominal differences are model-angle comparisons, not movement steps. Snapshot
medians come from sequential acquisition and are not a synchronous policy input.
load_snapshot validates decoded events/requests; this tool does not independently
redecode received wire bytes or measure physical angles/firmware scaling.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

from . import policy_shadow
from .pose_record import load_snapshot


SWING_CORE_SHA256 = "7a4f6ee4d1f4ae165356d91ab160fef5ac67d2958e8d717846ebc10be9d4811a"
D17_URDF_SHA256 = "ba77462679268455d547848e76925dcc1f75a9b497fe4814c92ac1e7492496c8"
# Audited SwingCore q0, FL/FR/RL/RR x hip/thigh/calf. Not rounded +/-0.28.
NOMINAL = (-.2800237445733691, .3837207788252659, -.7674415576505315,
           .2800237445733691, .3837207788252659, -.7674415576505315) * 2
JOINTS = tuple(f"{leg}_{kind}_joint" for leg in ("FL", "FR", "RL", "RR")
               for kind in ("hip", "thigh", "calf"))
VOLTAGE_MIN, VOLTAGE_MAX, CURRENT_MAX = 35.0, 43.0, .05
OVERLAY_FALSE_FLAGS = ("motor_output_allowed", "approved_for_runtime",
                       "controller_config_changed", "encoder_zero_written",
                       "zero_verified", "sign_revalidated_today",
                       "motor_power_cycle_continuity_verified")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _finite_tree(value):
    if isinstance(value, dict):
        for child in value.values():
            _finite_tree(child)
    elif isinstance(value, list):
        for child in value:
            _finite_tree(child)
    elif type(value) in (int, float):
        _require(policy_shadow.finite(value), "Nonfinite JSON number")


def _read_object(path):
    raw = Path(path).read_bytes()
    data = policy_shadow._json(raw.decode("utf-8"))
    _require(isinstance(data, dict), "Require a JSON object")
    _finite_tree(data)
    return data, _sha(raw)


def _overlay(data, calibration, calibration_sha, rows):
    _require(data.get("schema") == "singularitydog.manual-zero-overlay-candidate.v1"
             and data.get("status") == "OFFLINE_CANDIDATE_ONLY",
             "Require a standalone unapproved RR zero overlay")
    _require(type(data.get("motor_id")) is int and data["motor_id"] == 9
             and data.get("leg") == "RR" and data.get("model_joint") == "RR_hip_joint",
             "Overlay may target only RR hip ID9")
    _require(data.get("base_calibration_sha256") == calibration_sha,
             "Overlay original calibration hash mismatch")
    _require(data.get("identity") == calibration["identities"]["9"],
             "Overlay ID9 identity mismatch")
    _require(type(data.get("candidate_sign")) is int
             and data["candidate_sign"] == rows[9]["sign_candidate"] == 1,
             "RR zero overlay cannot change the original +1 sign")
    _require(policy_shadow.finite(data.get("base_offset_rad"))
             and data["base_offset_rad"] == rows[9]["offset_candidate_rad"]
             and policy_shadow.finite(data.get("candidate_offset_rad")),
             "Invalid or mismatched overlay offset")
    _require(data.get("formula") == "q_model = raw + candidate_offset_rad; radians; no wrapping"
             and type(data.get("nominal_model_angle_rad")) in (int, float)
             and data["nominal_model_angle_rad"] == 0,
             "Overlay requires explicit zero reference and no wrapping")
    _require(all(data.get(k) is False for k in OVERLAY_FALSE_FLAGS)
             and data.get("reference_external_measurement") is False,
             "Overlay cannot carry runtime, calibration, or measurement approval")
    return data["candidate_offset_rad"]


def build_report(capture, calibration, rr_overlay=None):
    """Pure file read. Return diagnostic q/gates, without raw UIDs or commands."""
    source, cal_sha = _read_object(calibration)
    rows = policy_shadow.validate_calibration(source)
    snapshot = load_snapshot(capture)
    _require(all(snapshot["motors"][str(i)]["mcu_uid_hex"] == source["identities"][str(i)]
                 for i in range(1, 13)), "Snapshot/calibration UID mismatch")
    offsets = {i: r["offset_candidate_rad"] for i, r in rows.items()}
    overlay_source = None
    if rr_overlay is not None:
        data, digest = _read_object(rr_overlay)
        offsets[9] = _overlay(data, source, cal_sha, rows)
        overlay_source = {"sha256": digest, "motor_id": 9,
                          "original_offset_rad": rows[9]["offset_candidate_rad"],
                          "candidate_offset_rad": offsets[9]}
    joints, blockers = [], []
    for index, mid in enumerate(policy_shadow.CAN_ORDER):
        m = snapshot["motors"][str(mid)]
        p, sign = m["position"], rows[mid]["sign_candidate"]
        original = sign * p["median_rad"] + rows[mid]["offset_candidate_rad"]
        q = sign * p["median_rad"] + offsets[mid]
        q_range = sorted(sign * v + offsets[mid] for v in (p["min_rad"], p["max_rad"]))
        delta = NOMINAL[index] - q
        _require(all(policy_shadow.finite(v) for v in (original, q, delta, *q_range)),
                 "Candidate arithmetic overflow/nonfinite")
        lower, upper = policy_shadow.LOWER[index], policy_shadow.UPPER[index]
        volts = m["voltage_minmax_V"]
        gates = {"model_range_all_samples": lower <= q_range[0] <= q_range[1] <= upper,
                 "voltage_all_samples": VOLTAGE_MIN <= volts[0] <= volts[1] <= VOLTAGE_MAX,
                 "current_all_samples": m["max_abs_current_A"] <= CURRENT_MAX,
                 "sampling_heuristics": not m["sampling_issues"]}
        failed = [key for key, passed in gates.items() if not passed]
        if failed:
            blockers.append({"motor_id": mid, "reasons": failed})
        joints.append({"model_index": index, "motor_id": mid, "joint": JOINTS[index],
                       "raw_position_rad": dict(p), "sign_candidate": sign,
                       "original_offset_rad": rows[mid]["offset_candidate_rad"],
                       "used_offset_rad": offsets[mid], "q_original_median_rad": original,
                       "q_candidate_median_rad": q, "q_candidate_minmax_rad": q_range,
                       "model_lower_rad": lower, "model_upper_rad": upper,
                       "nominal_reference_rad": NOMINAL[index],
                       "nominal_minus_candidate_rad": delta,
                       "voltage_minmax_V": list(volts),
                       "max_abs_current_A": m["max_abs_current_A"],
                       "max_abs_velocity_rad_s": m["max_abs_velocity_rad_s"],
                       "sampling_issues": list(m["sampling_issues"]), "gates": gates})
    return {"schema": "singularitydog.offline-prestand-check.v1",
            "status": "OFFLINE_SCREEN_BLOCKED" if blockers else "OFFLINE_SCREEN_PASS",
            "blockers": blockers, "joint_table": joints,
            "model_can_order": list(policy_shadow.CAN_ORDER),
            "capture_started_at": snapshot["started_at"],
            "capture_completed_at": snapshot["completed_at"],
            "capture_intervals": snapshot["capture_intervals"],
            "sampling_issues": snapshot["sampling_issues"],
            "capture_uid_match_all12": True,
            "screen_limits": {"voltage_min_V": VOLTAGE_MIN, "voltage_max_V": VOLTAGE_MAX,
                              "max_abs_current_A": CURRENT_MAX},
            "sources": {"capture": snapshot["sources"], "original_calibration_sha256": cal_sha,
                        "rr_overlay": overlay_source, "checker_sha256": _sha(Path(__file__).read_bytes()),
                        "nominal_SwingCore_sha256": SWING_CORE_SHA256,
                        "registered_limits_D17_URDF_sha256": D17_URDF_SHA256,
                        "policy_shadow_sha256": _sha(Path(policy_shadow.__file__).read_bytes())},
            "limitations": ["Saved sequential samples are never fresh live20ms input.",
                            "Nominal differences are not drive steps or raw targets.",
                            "Model q and optional RR overlay remain unverified hypotheses.",
                            "No clipping, wrapping, inferred pose, or automatic offset change.",
                            "Sampling heuristics and low current do not prove motors are disabled.",
                            "Received wire bytes and physical angles are not independently checked here.",
                            "Declared nominal/URDF hashes document constants; model files are not loaded."],
            "motor_output_available": False, "motor_output_allowed": False,
            "output_allowed": False, "approved_for_runtime": False,
            "live_readiness": False, "fresh_identity_match_verified": False,
            "calibration_verified": False, "zero_verified": False, "sign_verified": False,
            "motor_power_cycle_continuity_verified": False,
            "received_wire_redecoded": False, "controller_config_changed": False,
            "encoder_zero_written": False, "angle_wrapping_applied": False}


def save_report(capture, calibration, output, rr_overlay=None):
    """Write a new private report only after every input is validated."""
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("Prestand report already exists")
    resolved = output.resolve()
    _require(not any((p / ".git").exists() for p in (resolved, *resolved.parents)),
             "Private prestand reports must be outside Git")
    report = build_report(capture, calibration, rr_overlay)
    payload = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True, type=Path)
    ap.add_argument("--calibration", required=True, type=Path)
    ap.add_argument("--rr-overlay", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args(argv)
    try:
        report = save_report(args.capture, args.calibration, args.output, args.rr_overlay)
    except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
        ap.exit(2, "Offline prestand rejected: " + str(error) + "\n")
    print(json.dumps({"status": report["status"], "blockers": report["blockers"],
                      "output": str(args.output.resolve()), "output_allowed": False,
                      "approved_for_runtime": False, "live_readiness": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
