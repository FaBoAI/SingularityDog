"""Audit nominal L-pose calibration hypotheses from saved files only.

This program has no hardware imports. Its output contains private raw positions,
UIDs and candidate offsets, so it writes only to a fresh path outside Git. The
output is a review artifact, never a runtime calibration or motor target.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics


IDS = tuple(range(1, 13))
LEGS = {"FR": (1, 2, 3), "FL": (4, 5, 6),
        "RR": (7, 8, 9), "RL": (10, 11, 12)}
CAPTURE_SCHEMA = "singularitydog.fixed-stance-readonly-capture.v1"
URDF_SHA256 = "ba77462679268455d547848e76925dcc1f75a9b497fe4814c92ac1e7492496c8"
PINNED_SHA256 = {
    "l": "c40a9f8c5735e8525dd4cbdefc6e2b8cf97da5aba4474398ae9fc7ae4bfacd77",
    "box": "7ca29605e7b142d8a49e8ca1431731c29b1df5d476176f1b7b80bb00b071bc6b",
    "historical": "68d139e3bf6ec329fd941b80bdc711f20c4280ca065859daf2d8a9ffc4051371",
    "id10_a": "afe3e134ceab5ef0a83c9d9b92a89881e0034a8d05e6de1b0a82796207054819",
    "id10_b": "c70f505d6b5c9b972c5ae48f8aad9eb63f247f92e9da78a0147f9b5f47adb67d",
}


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _finite(value, label):
    _require(type(value) in (float, int) and math.isfinite(value), f"{label} must be finite")
    return float(value)


def _read_pinned(path, digest, label):
    content = Path(path).read_bytes()
    _require(hashlib.sha256(content).hexdigest() == digest, f"{label} SHA-256 mismatch")
    return json.loads(content)


def _raw_map(value, label):
    _require(type(value) is dict and set(value) == {str(i) for i in IDS},
             f"{label} must contain exactly IDs 1..12")
    return {i: _finite(value[str(i)], f"{label} ID{i}") for i in IDS}


def _capture(data, label):
    _require(type(data) is dict and data.get("schema") == CAPTURE_SCHEMA
             and data.get("status") == "RECORDED_REVIEW_REQUIRED"
             and data.get("errors") == [] and data.get("output_allowed") is False
             and data.get("approved_for_runtime") is False,
             f"{label} is not a complete non-output capture")
    pose, identities, plan = data.get("pose"), data.get("identities"), data.get("plan")
    _require(type(pose) is dict and pose.get("sampling_issues") is not None,
             f"{label} pose quality is missing")
    _require(type(identities) is dict and set(identities) == {str(i) for i in IDS},
             f"{label} needs twelve identities")
    uids = {i: identities[str(i)].get("mcu_uid_hex") for i in IDS}
    _require(all(type(uid) is str and len(uid) == 16 for uid in uids.values())
             and len(set(uids.values())) == 12, f"{label} identities are invalid")
    _require(type(plan) is dict and plan.get("ids_by_bus") == {
        "front": list(range(1, 7)), "rear": list(range(7, 13))},
        f"{label} bus map differs")
    _require(type(data.get("boot_id")) is str and data["boot_id"],
             f"{label} boot ID is missing")
    return {"boot_id": data["boot_id"], "uids": uids,
            "raw": _raw_map(pose.get("raw_rad_by_id"), label),
            "stability_passed": pose.get("sampling_stability_heuristic_passed") is True,
            "sampling_issues": pose["sampling_issues"],
            "source_sha256": data.get("source_sha256"), "ports": plan.get("ports")}


def _historical(data, uids, boot):
    _require(type(data) is dict and data.get("boot_id") != boot
             and data.get("approved_for_runtime") is False
             and data.get("sign_revalidated") is False
             and data.get("motor_power_cycle_continuity_verified") is False
             and data.get("physical_angle_accuracy_verified") is False,
             "Historical candidate must remain unverified and from another boot")
    rows = data.get("candidates")
    _require(type(rows) is list and len(rows) == 12, "Historical candidate needs twelve rows")
    by_id = {row.get("motor_id"): row for row in rows if type(row) is dict}
    _require(set(by_id) == set(IDS), "Historical candidate IDs differ")
    for i, row in by_id.items():
        _require(row.get("uid") == uids[i], f"Historical ID{i} UID mismatch")
        _require(type(row.get("sign_candidate")) is int
                 and row["sign_candidate"] in (-1, 1), f"Historical ID{i} sign invalid")
        _finite(row.get("raw_L_median_rad"), f"Historical ID{i} L")
    return by_id


def _id10_ab(a, b, boot):
    for label, data in (("A", a), ("B", b)):
        _require(type(data) is dict and data.get("boot_id") == boot
                 and data.get("status") == f"READ_ONLY_{label}"
                 and data.get("motor_output_allowed") is False,
                 f"ID10 {label} is not a same-boot read-only record")
        rows = data.get("rows")
        _require(type(rows) is dict and set(rows) == {"10", "11", "12"},
                 f"ID10 {label} needs three axes")
        for i in (10, 11, 12):
            _require(rows[str(i)].get("disabled_mode") is True,
                     f"ID10 {label} ID{i} disabled state missing")
            _finite(rows[str(i)].get("position_rad"), f"ID10 {label} ID{i} position")
    delta = {i: math.degrees(b["rows"][str(i)]["position_rad"]
                             - a["rows"][str(i)]["position_rad"])
             for i in (10, 11, 12)}
    reported = b.get("delta_deg_from_A")
    _require(type(reported) is dict and set(reported) == {"10", "11", "12"}
             and all(abs(delta[i] - _finite(reported[str(i)], f"ID{i} reported delta")) < 1e-8
                     for i in delta), "ID10 A/B reported deltas differ")
    _require(-110 < delta[10] < -70 and max(abs(delta[i]) for i in (11, 12)) < 3,
             "ID10 A/B isolation or direction needs review")
    return delta


def _after_pose(summary, events_by_bus, expected_boot, uids):
    _require(summary.get("status") == "COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT"
             and summary.get("boot_id") == expected_boot and summary.get("errors") == []
             and summary.get("output_allowed") is False
             and summary.get("approved_for_runtime") is False,
             "After-ID10 summary is not a complete same-boot read-only run")
    _require(summary.get("plan", {}).get("cycles_per_bus") == 20,
             "After-ID10 pose needs twenty cycles")
    values = {i: {} for i in IDS}
    seen_uids = {}
    for bus, content in events_by_bus.items():
        _require(bus in ("front", "rear"), "Unexpected bus")
        _require(hashlib.sha256(content).hexdigest() == summary["events_sha256"][bus],
                 f"After-ID10 {bus} events SHA-256 mismatch")
        expected_ids = set(range(1, 7)) if bus == "front" else set(range(7, 13))
        for line in content.splitlines():
            event = json.loads(line)
            if event.get("kind") != "pipeline_reply" or event.get("ok") is not True:
                continue
            i, parameter = event.get("motor_id"), event.get("parameter")
            if parameter not in ("identity", "position"):
                continue
            _require(i in expected_ids and type(event.get("result")) is dict,
                     "After-ID10 reply bus or content invalid")
            result = event["result"]
            if parameter == "identity":
                _require(event.get("cycle") == 0 and i not in seen_uids,
                         "Duplicate or late identity")
                seen_uids[i] = result.get("mcu_uid_hex")
            else:
                cycle = event.get("cycle")
                _require(type(cycle) is int and 1 <= cycle <= 20
                         and cycle not in values[i] and result.get("index") == 0x7019
                         and result.get("unit") == "rad_output_shaft",
                         f"After-ID10 ID{i} position cycle invalid")
                values[i][cycle] = _finite(result.get("value"), f"After-ID10 ID{i} value")
    _require(seen_uids == uids, "After-ID10 identities differ from L capture")
    _require(all(set(values[i]) == set(range(1, 21)) for i in IDS),
             "After-ID10 twenty-cycle position set incomplete")
    return {i: statistics.median(values[i].values()) for i in IDS}, {
        i: math.degrees(max(values[i].values()) - min(values[i].values())) for i in IDS}


def _foot_centers(raw, l_raw, signs):
    """D17 URDF joint origins: hip ±155/110 mm, offset ±64, links 120/120."""
    points = {}
    for leg, (calf_id, thigh_id, hip_id) in LEGS.items():
        side = 1 if leg in ("FL", "RL") else -1
        front = 1 if leg in ("FR", "FL") else -1
        calf = -math.pi / 2 + signs[calf_id] * (raw[calf_id] - l_raw[calf_id])
        thigh = signs[thigh_id] * (raw[thigh_id] - l_raw[thigh_id])
        hip = signs[hip_id] * (raw[hip_id] - l_raw[hip_id])
        down = -.12 * (math.cos(thigh) + math.cos(thigh + calf))
        points[leg] = (front * .155 - .12 * (math.sin(thigh) + math.sin(thigh + calf)),
                       side * .11 + side * .064 * math.cos(hip) - down * math.sin(hip),
                       side * .064 * math.sin(hip) + down * math.cos(hip))
    return points


def _rl_plane_residual_mm(points):
    a, b, c, d = (points[leg] for leg in ("FR", "FL", "RR", "RL"))
    u, v, w = ([p[k] - a[k] for k in range(3)] for p in (b, c, d))
    n = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    norm = math.sqrt(sum(x*x for x in n))
    _require(norm > 1e-9, "Reference foot-center plane is degenerate")
    return abs(sum(n[k]*w[k] for k in range(3))) / norm * 1000


def build_review(l_data, box_data, historical_data, a_data, b_data,
                 *, after=None, urdf_sha256=URDF_SHA256):
    _require(urdf_sha256 == URDF_SHA256, "D17 URDF SHA-256 mismatch")
    l, box = _capture(l_data, "L"), _capture(box_data, "box")
    _require(l["boot_id"] == box["boot_id"] and l["uids"] == box["uids"]
             and l["ports"] == box["ports"] and l["source_sha256"] == box["source_sha256"],
             "L and box captures differ in boot, UIDs, ports or capture source")
    old = _historical(historical_data, l["uids"], l["boot_id"])
    ab_delta = _id10_ab(a_data, b_data, l["boot_id"])
    signs = {i: old[i]["sign_candidate"] for i in IDS}
    after_raw, after_span = after if after is not None else (None, None)
    rows = []
    for i in IDS:
        nominal = -math.pi / 2 if i in (1, 4, 7, 10) else 0.
        row = {"motor_id": i, "uid": l["uids"][i], "raw_l_rad": l["raw"][i],
               "raw_box_rad": box["raw"][i], "nominal_l_rad": nominal,
               "historical_sign": signs[i],
               "historical_to_current_l_direct_deg": math.degrees(
                   l["raw"][i] - old[i]["raw_L_median_rad"]),
               "box_minus_l_direct_deg": math.degrees(box["raw"][i] - l["raw"][i]),
               "retained_sign_offset_hypothesis_rad": nominal - signs[i]*l["raw"][i],
               "box_model_angle_retained_sign_hypothesis_deg": math.degrees(
                   nominal + signs[i]*(box["raw"][i] - l["raw"][i])),
               "sign_revalidated": False, "physical_angle_accuracy_verified": False}
        if after_raw is not None:
            row["raw_after_id10_rad"] = after_raw[i]
            row["after_id10_minus_l_direct_deg"] = math.degrees(after_raw[i] - l["raw"][i])
            row["after_id10_model_angle_retained_sign_hypothesis_deg"] = math.degrees(
                nominal + signs[i]*(after_raw[i] - l["raw"][i]))
            row["after_id10_twenty_cycle_span_deg"] = after_span[i]
        rows.append(row)
    hypotheses = {}
    for name, id10_sign in (("historical_id10_plus", signs[10]),
                            ("id10_reversed_for_review", -signs[10])):
        candidate_signs = {**signs, 10: id10_sign}
        hypotheses[name] = {
            "id10_sign": id10_sign,
            "id10_offset_rad": -math.pi/2 - id10_sign*l["raw"][10],
            "box_rl_to_other_three_foot_center_plane_mm": _rl_plane_residual_mm(
                _foot_centers(box["raw"], l["raw"], candidate_signs)),
            "after_id10_rl_to_other_three_foot_center_plane_mm": (
                _rl_plane_residual_mm(_foot_centers(after_raw, l["raw"], candidate_signs))
                if after_raw is not None else None),
            "verified": False,
        }
    direct_over_180 = sorted(row["motor_id"] for row in rows if any(
        abs(row[key]) > 180 for key in (
            "historical_to_current_l_direct_deg", "box_minus_l_direct_deg",
            "after_id10_minus_l_direct_deg") if key in row))
    return {"schema": "singularitydog.offline-nominal-l-review.v1",
            "status": "REVIEW_REQUIRED_NO_RUNTIME_PROMOTION", "boot_id": l["boot_id"],
            "source_sha256": dict(PINNED_SHA256), "d17_urdf_sha256": urdf_sha256,
            "l_stability_heuristic_passed": l["stability_passed"],
            "l_sampling_issues": l["sampling_issues"],
            "box_stability_heuristic_passed": box["stability_passed"],
            "box_sampling_issues": box["sampling_issues"],
            "id10_manual_b_minus_a_deg": {str(i): ab_delta[i] for i in (10, 11, 12)},
            "id10_physical_direction_operator_observation":
                "faceward horizontal to downward approximately 90 degrees; not metrology",
            "id10_manual_ab_uid_independently_recorded": False,
            "id10_sign_evidence": {
                "historical_plus": "in tension with observed negative raw delta for downward rotation",
                "reversed_minus": "consistent with observed direction, pending physical angle and FK checks",
            },
            "priority_branch_review_ids": [1, 3, 9],
            "all_direct_deltas_over_180_deg_ids": direct_over_180,
            "rows": rows,
            "fk_hypotheses": hypotheses,
            "fk_interpretation": "Coplanarity is necessary for four equal-radius feet on one plane, not proof of contact or sign. FK is periodic and hides raw turn discontinuities.",
            "sign_revalidated": False, "physical_angle_accuracy_verified": False,
            "motor_power_cycle_continuity_verified": False,
            "angle_wrapping_applied": False, "turn_correction_applied": False,
            "motor_output_available": False, "output_allowed": False,
            "approved_for_runtime": False, "command_bytes_generated": False}


def _private_new_path(path):
    path = Path(path).expanduser()
    _require(not path.is_symlink(), "Output cannot be a symlink")
    path = path.resolve()
    _require(path.parent.is_dir() and not path.exists(), "Output needs a fresh path in an existing directory")
    _require(not any((parent / ".git").exists() for parent in (path.parent, *path.parents)),
             "Private output must be outside Git")
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    for label in ("l", "box", "historical", "id10-a", "id10-b", "urdf", "output"):
        ap.add_argument("--" + label, type=Path, required=True)
    for label in ("after-summary", "events-front", "events-rear"):
        ap.add_argument("--" + label, type=Path)
    args = ap.parse_args(argv)
    after_paths = (args.after_summary, args.events_front, args.events_rear)
    if any(after_paths) and not all(after_paths):
        ap.error("After-ID10 review requires summary and both event streams")
    try:
        data = {key: _read_pinned(getattr(args, key), digest, key)
                for key, digest in PINNED_SHA256.items()}
        _require(hashlib.sha256(args.urdf.read_bytes()).hexdigest() == URDF_SHA256,
                 "D17 URDF SHA-256 mismatch")
        after = None
        if all(after_paths):
            summary = json.loads(args.after_summary.read_text())
            l = _capture(data["l"], "L")
            after = _after_pose(summary, {
                bus: path.read_bytes() for bus, path in
                (("front", args.events_front), ("rear", args.events_rear))},
                l["boot_id"], l["uids"])
        review = build_review(data["l"], data["box"], data["historical"],
                              data["id10_a"], data["id10_b"], after=after)
        if all(after_paths):
            review["after_id10_sources_sha256"] = {
                "summary": hashlib.sha256(args.after_summary.read_bytes()).hexdigest(),
                **summary["events_sha256"],
            }
        output = _private_new_path(args.output)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        with os.fdopen(os.open(output, flags, 0o600), "w", encoding="utf-8") as stream:
            json.dump(review, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"Saved private offline review: {output}")
        for name, row in review["fk_hypotheses"].items():
            print(f"{name}: old-box residual {row['box_rl_to_other_three_foot_center_plane_mm']:.1f} mm"
                  + (f", after-ID10 residual {row['after_id10_rl_to_other_three_foot_center_plane_mm']:.1f} mm"
                     if after is not None else ""))
        print("Review required; no runtime calibration or motor target produced.")
    except (ValueError, OSError, json.JSONDecodeError) as error:
        ap.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
