"""Name and compare existing joint_snapshot captures, entirely offline.

No device is opened and no motor command, calibration or pose replay is produced.
The output contains private motor UIDs and is restricted to a new directory
outside Git. Arbitrary shaft positions are never converted into model angles.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re

from .can_readonly import PARAMETERS, read_request
from .joint_observation import (
    IDS, ObservationError, _capture, _finite, _load, _statistics, _strict_pairs,
)


CORE = ("position", "current", "velocity", "voltage")
HEURISTICS = {"position_span_rad": 0.02, "abs_velocity_rad_s": 0.1,
              "abs_current_A": 0.05, "large_direct_delta_rad": math.pi}


def load_snapshot(directory):
    """Check completed read-only coverage; recompute statistics from raw events."""
    directory = Path(directory).resolve()
    summary_bytes = (directory / "summary.json").read_bytes()
    def bad_constant(value):
        raise ObservationError("Non-JSON number: " + value)
    summary = json.loads(summary_bytes, object_pairs_hook=_strict_pairs,
                         parse_constant=bad_constant)
    if (not isinstance(summary, dict) or summary.get("status") != "RECORDED_NOT_CALIBRATED"
            or summary.get("errors") != [] or summary.get("discarded_rx_bytes") != 0):
        raise ObservationError("A completed, error-free joint_snapshot is required")
    plan = summary.get("plan", {})
    sweeps = plan.get("sweeps")
    if (type(sweeps) is not int or not 3 <= sweeps <= 60 or
            plan.get("allowed_can_types") != [0, 17] or
            plan.get("motor_output_available") is not False or
            plan.get("ids") != list(IDS) or plan.get("parameters") != ["identity", *CORE]):
        raise ObservationError("Not a supported read-only joint_snapshot plan")
    records, events_source = _load(directory / "events.jsonl")
    identities, positions, intervals = _capture(records, "pose")
    metadata = [r for r in records if r["kind"] == "capture_metadata"]
    if (len(metadata) != 1 or records[0] is not metadata[0] or
            any(metadata[0].get(k) != v for k, v in plan.items())):
        raise ObservationError("Capture metadata missing or inconsistent")
    sweeps_seen, outstanding, previous_time, replies_seen = [], None, None, 0
    for event in records:
        if event["kind"] not in ("capture_metadata", "can_tx", "motor_parameter", "joint_snapshot_sweep"):
            continue
        clocks = tuple(event.get(c) for c in ("monotonic_ns", "wall_time_ns"))
        if (any(type(c) is not int or c <= 0 for c in clocks) or
                previous_time is not None and any(a <= b for a, b in zip(clocks, previous_time))):
            raise ObservationError("Capture chronology is inconsistent")
        previous_time = clocks
        if event["kind"] == "can_tx":
            if outstanding is not None:
                raise ObservationError("Another request precedes the previous reply")
            outstanding = event
        elif event["kind"] == "motor_parameter":
            if outstanding is None or event.get("request_monotonic_ns") != outstanding["monotonic_ns"]:
                raise ObservationError("Reply has no matching preceding request")
            outstanding = None
            replies_seen += 1
        elif event["kind"] == "joint_snapshot_sweep":
            if outstanding is not None or replies_seen != 12 + 48 * (len(sweeps_seen) + 1):
                raise ObservationError("Sweep marker is out of sequence")
            sweeps_seen.append(event.get("sweep"))
    if outstanding is not None or sweeps_seen != list(range(1, sweeps + 1)):
        raise ObservationError("Incomplete sweep markers or unanswered request")
    expected = [(mid, "identity") for mid in IDS] + [
        (mid, parameter) for _ in range(sweeps) for mid in IDS for parameter in CORE]
    txs = [r for r in records if r["kind"] == "can_tx"]
    replies = [r for r in records if r["kind"] == "motor_parameter"]
    if len(txs) != len(expected) or len(replies) != len(expected) or summary.get("tx_count") != len(expected):
        raise ObservationError("Incomplete request/reply coverage")
    values = {mid: {parameter: [] for parameter in CORE} for mid in IDS}
    for sequence, ((mid, parameter), tx, reply) in enumerate(zip(expected, txs, replies), 1):
        for event in (tx, reply):
            if (event.get("motor_id"), event.get("parameter"), event.get("sequence")) != (mid, parameter, sequence):
                raise ObservationError("Request/reply ordering or ID mismatch")
        if tx.get("hex") != read_request(mid, None if parameter == "identity" else parameter).hex():
            raise ObservationError("Non-read-only or mismatched request bytes")
        if parameter != "identity":
            if (reply.get("index") != PARAMETERS[parameter][0] or
                    reply.get("unit") != PARAMETERS[parameter][2] or
                    type(reply.get("status")) is not int or reply["status"] != 0 or
                    not _finite(reply.get("value"))):
                raise ObservationError("Invalid core telemetry")
            values[mid][parameter].append(reply["value"])
    recorded = summary.get("summary", {})
    if (recorded.get("identities") != {str(i): identities[i] for i in IDS} or
            recorded.get("position_unit") != "rad_output_shaft" or
            recorded.get("automatic_wrap_applied") is not False):
        raise ObservationError("Summary identities or units disagree with events")
    motors, issues = {}, []
    for mid in IDS:
        stats = _statistics(positions[mid])
        expected_stats = {"samples": sweeps, "mean": stats["mean_rad"],
                          "min": stats["min_rad"], "max": stats["max_rad"],
                          "last": positions[mid][-1]}
        saved_stats = recorded.get("positions", {}).get(str(mid), {})
        if (set(saved_stats) != set(expected_stats) or
                type(saved_stats.get("samples")) is not int or saved_stats["samples"] != sweeps or
                any(not _finite(saved_stats[k]) or not math.isclose(saved_stats[k], v, rel_tol=0, abs_tol=1e-12)
                    for k, v in expected_stats.items() if k != "samples")):
            raise ObservationError("Summary position statistics disagree with events")
        current = max(abs(v) for v in values[mid]["current"])
        velocity = max(abs(v) for v in values[mid]["velocity"])
        reasons = []
        if stats["peak_to_peak_rad"] > HEURISTICS["position_span_rad"]:
            reasons.append("position_spread")
        if velocity > HEURISTICS["abs_velocity_rad_s"]:
            reasons.append("velocity")
        if current > HEURISTICS["abs_current_A"]:
            reasons.append("current")
        if reasons:
            issues.append({"motor_id": mid, "reasons": reasons})
        motors[str(mid)] = {"mcu_uid_hex": identities[mid], "position": stats,
                            "median_deg_raw_shaft": math.degrees(stats["median_rad"]),
                            "max_abs_current_A": current, "max_abs_velocity_rad_s": velocity,
                            "voltage_minmax_V": [min(values[mid]["voltage"]), max(values[mid]["voltage"])],
                            "sampling_issues": reasons}
    return {"motors": motors, "capture_intervals": intervals,
            "started_at": summary.get("started_at"), "completed_at": summary.get("completed_at"),
            "sampling_issues": issues, "sources": {
                "events": events_source,
                "summary": {"path": str(directory / "summary.json"), "bytes": len(summary_bytes),
                            "sha256": hashlib.sha256(summary_bytes).hexdigest()}}}


def build_record(capture, name, note, reference=None):
    if not isinstance(name, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,63}", name):
        raise ObservationError("Name must be 1..64 lowercase ASCII letters, digits, '-' or '_'")
    if not isinstance(note, str) or not note.strip() or len(note) > 2000:
        raise ObservationError("A short description of the physical pose is required")
    pose = load_snapshot(capture)
    pose.update(schema_version=1, name=name, operator_note=note,
                status="RECORDED_NOT_CALIBRATED", approved_for_runtime=False,
                zero_inferred=False, sign_inferred=False, angle_wrapping_applied=False,
                semantic_mapping_inferred=False, motor_enable_state_verified=False,
                motor_power_cycle_continuity_verified=False, pose_replay_available=False,
                telemetry_evidence="decoded_motor_parameter_events_with_read_request_and_time_checks",
                received_wire_redecoded=False,
                measured_model_joint_angles_rad=None, heuristics=HEURISTICS,
                heuristics_are_mechanical_limits=False,
                sampling_quality="review_required" if pose["sampling_issues"] else "stationary_candidate")
    if reference is not None:
        before = load_snapshot(reference)
        if before["sources"]["events"]["sha256"] == pose["sources"]["events"]["sha256"]:
            raise ObservationError("Reference and current capture must be distinct")
        if any(before["capture_intervals"][c]["end"] >= pose["capture_intervals"][c]["start"]
               for c in before["capture_intervals"]):
            raise ObservationError("Reference must precede current capture on both clocks")
        changes, large = {}, []
        for mid in map(str, IDS):
            pre, post = before["motors"][mid], pose["motors"][mid]
            if pre["mcu_uid_hex"] != post["mcu_uid_hex"]:
                raise ObservationError("Motor ID to UID mapping changed")
            delta = post["position"]["median_rad"] - pre["position"]["median_rad"]
            if not _finite(delta):
                raise ObservationError("Nonfinite direct difference")
            changes[mid] = {"median_delta_rad": delta, "median_delta_deg": math.degrees(delta)}
            if abs(delta) >= HEURISTICS["large_direct_delta_rad"]:
                large.append(int(mid))
        pose["reference_comparison"] = {
            "sources": before["sources"], "changes": changes,
            "reference_sampling_issues": before["sampling_issues"],
            "large_raw_change_or_discontinuity_ids": large,
            "power_cycle_continuity_verified": False,
            "status": "raw_difference_requires_review"}
    return pose


def markdown(pose):
    note = pose["operator_note"].replace("\n", " ").replace("|", "\\|")
    lines = ["# 保存ポーズ: " + pose["name"], "", note, "",
             f"取得: {pose['started_at']} — {pose['completed_at']}", "",
             "出力軸の生角度です。モデルの関節角・原点・可動域・駆動指令ではありません。",
             "低電流や小さいばらつきは、モーター無効状態の証明ではありません。", "",
             "| ID | 生角度 rad（中央値） | 生角度 ° | ばらつき ° | 基準との差 ° |",
             "|---:|---:|---:|---:|---:|"]
    comparison = pose.get("reference_comparison", {}).get("changes", {})
    for mid in map(str, IDS):
        m = pose["motors"][mid]
        delta = f"{comparison[mid]['median_delta_deg']:+.4f}" if mid in comparison else "—"
        lines.append(f"| {mid} | {m['position']['median_rad']:.8f} | {m['median_deg_raw_shaft']:.4f} | "
                     f"{math.degrees(m['position']['peak_to_peak_rad']):.4f} | {delta} |")
    lines += ["", "記録品質: " + pose["sampling_quality"], "",
              "差分は角度の折り返し補正なし。モーター電源の再投入・原点変更をまたぐ連続性は未検証です。",
              "任意のポーズ名だけでは原点・正方向を決められません。既知の実角度とモデル軸の照合が必要です。",
              "原点設定・自動復帰・ポーズ再生は行いません。"]
    if pose["sampling_issues"]:
        lines += ["", "再測定候補: " + json.dumps(pose["sampling_issues"], ensure_ascii=False)]
    if "reference_comparison" in pose:
        comp = pose["reference_comparison"]
        lines += ["", "基準の再測定候補: " + json.dumps(comp["reference_sampling_issues"]),
                  "大きな変化・不連続の確認対象ID: " + json.dumps(comp["large_raw_change_or_discontinuity_ids"])]
    return "\n".join(lines) + "\n"


def save_record(capture, name, note, output, reference=None):
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("Pose output already exists")
    resolved = output.resolve()
    if any((p / ".git").exists() for p in (resolved, *resolved.parents)):
        raise ObservationError("Private pose records must be outside Git")
    pose = build_record(capture, name, note, reference)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    for filename, text in (("pose.json", json.dumps(pose, indent=2, ensure_ascii=False, allow_nan=False) + "\n"),
                           ("POSE.md", markdown(pose))):
        fd = os.open(output / filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
    return pose


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--capture", required=True, type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--note", required=True)
    ap.add_argument("--reference", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args(argv)
    try:
        pose = save_record(args.capture, args.name, args.note, args.output, args.reference)
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
        ap.exit(2, "Pose record rejected: " + str(error) + "\n")
    print(json.dumps({"output": str(args.output.resolve()), "name": pose["name"],
                      "status": pose["status"], "sampling_quality": pose["sampling_quality"],
                      "approved_for_runtime": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
