"""Compare read-only twelve-axis captures without promoting a calibration.

Use the same, hashed, unapproved angle profile for every capture.  A numerical
whole-turn branch is useful for locating a power-cycle readout jump, but
different captures are *not* assumed to have the same physical pose.  This
tool neither opens CAN nor produces motor targets or approval flags.

Capture and transition indexes are zero-based.  Adjacent comparisons use only
their two captures; an invalid capture does not hide later valid comparisons.
Reported maxima cover numeric comparisons only, with invalid indexes retained.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from audit_angle_calibration import (
    IDS, load_profile, read, current_values, write_private,
)
from singularitydog_hw.angle_calibration_audit import equivalent_branch_candidates


def _numeric_candidate(capture):
    candidates = capture["branch_candidates"]
    if (capture["uid_matches"] and len(candidates) == 1
            and candidates[0]["whole_uncertainty_inside_limits"]):
        return candidates[0]
    return None


def _compare(before, after, before_index, after_index):
    first, second = _numeric_candidate(before), _numeric_candidate(after)
    numeric = first is not None and second is not None
    return {"from_capture_index": before_index, "to_capture_index": after_index,
            "from_capture_sha256": before["capture_sha256"],
            "to_capture_sha256": after["capture_sha256"],
            "uid_matches_both_captures": before["uid_matches"] and after["uid_matches"],
            "unique_numeric_branch_in_both_captures": numeric,
            "raw_delta_deg": math.degrees(after["raw_rad"] - before["raw_rad"]),
            "branch_turn_delta_if_numeric": second["turns"] - first["turns"] if numeric else None,
            "model_delta_deg_if_numeric": (
                math.degrees(second["model_rad"] - first["model_rad"]) if numeric else None)}


def _max_abs_numeric(comparisons):
    return max((abs(row["model_delta_deg_if_numeric"]) for row in comparisons
                if row["model_delta_deg_if_numeric"] is not None), default=None)


def review(profile, sources):
    if len(sources) < 2:
        raise ValueError("At least two independent read-only captures required")
    captures = []
    for path in sources:
        current, digest = read(path)
        raw, uids = current_values(current)
        captures.append({"path": str(Path(path).resolve()), "sha256": digest,
                         "boot_id": current.get("boot_id"),
                         "motor_power_epoch_label": current.get("motor_power_epoch"),
                         "raw": raw, "uids": uids})
    if len({row["sha256"] for row in captures}) != len(captures):
        raise ValueError("A capture was repeated")
    rows = {}
    first, last = captures[0], captures[-1]
    for mid in IDS:
        key = str(mid)
        axis = profile[mid]
        per_capture = []
        for entry in captures:
            raw = entry["raw"][key]
            uid_matches = entry["uids"][key] == axis.uid
            candidates = equivalent_branch_candidates(raw, axis) if uid_matches else []
            per_capture.append({"capture_sha256": entry["sha256"],
                                "uid_matches": uid_matches,
                                "raw_rad": raw,
                                "branch_candidates": candidates})
        unique = all(_numeric_candidate(row) is not None for row in per_capture)
        adjacent = [dict(transition_index=index,
                         **_compare(before, after, index, index + 1))
                    for index, (before, after) in enumerate(zip(per_capture, per_capture[1:]))]
        from_first = [_compare(per_capture[0], after, 0, index)
                      for index, after in enumerate(per_capture[1:], start=1)]
        changed = [row["transition_index"] for row in adjacent
                   if row["branch_turn_delta_if_numeric"] not in (None, 0)]
        raw_delta = last["raw"][key] - first["raw"][key]
        branch_turn_delta = (per_capture[-1]["branch_candidates"][0]["turns"]
                             - per_capture[0]["branch_candidates"][0]["turns"]
                             if unique else None)
        rows[key] = {"uid": axis.uid, "calibration_evidence_blockers": axis.evidence_blockers(),
                     "captures": per_capture, "unique_numeric_branch_in_every_capture": unique,
                     "adjacent_transitions": adjacent,
                     "comparisons_to_first_capture": from_first,
                     "series_summary": {
                         "whole_turn_changed_transition_count": len(changed),
                         "whole_turn_changed_transition_indexes": changed,
                         "numeric_adjacent_transition_count": sum(
                             row["unique_numeric_branch_in_both_captures"] for row in adjacent),
                         "invalid_adjacent_transition_indexes": [
                             row["transition_index"] for row in adjacent
                             if not row["unique_numeric_branch_in_both_captures"]],
                         "max_abs_adjacent_model_delta_deg_if_numeric": _max_abs_numeric(adjacent),
                         "numeric_comparison_to_first_count": sum(
                             row["unique_numeric_branch_in_both_captures"] for row in from_first),
                         "invalid_comparison_to_first_capture_indexes": [
                             row["to_capture_index"] for row in from_first
                             if not row["unique_numeric_branch_in_both_captures"]],
                         "max_abs_model_delta_vs_first_deg_if_numeric": _max_abs_numeric(from_first),
                         "uid_mismatch_capture_indexes": [
                             index for index, row in enumerate(per_capture) if not row["uid_matches"]],
                         "nonunique_numeric_branch_capture_indexes": [
                             index for index, row in enumerate(per_capture)
                             if _numeric_candidate(row) is None]},
                     "first_to_last_raw_delta_deg": math.degrees(raw_delta),
                     "first_to_last_branch_turn_delta": branch_turn_delta,
                     "first_to_last_model_delta_deg_if_numeric": (
                         math.degrees(per_capture[-1]["branch_candidates"][0]["model_rad"]
                                      - per_capture[0]["branch_candidates"][0]["model_rad"])
                         if unique else None)}
    transition_summaries = []
    for index in range(len(captures) - 1):
        pairs = {mid: rows[str(mid)]["adjacent_transitions"][index] for mid in IDS}
        transition_summaries.append({
            "transition_index": index, "from_capture_index": index, "to_capture_index": index + 1,
            "whole_turn_changed_ids": [mid for mid, row in pairs.items()
                                       if row["branch_turn_delta_if_numeric"] not in (None, 0)],
            "uid_mismatch_ids": [mid for mid, row in pairs.items()
                                 if not row["uid_matches_both_captures"]],
            "nonunique_branch_ids": [mid for mid, row in pairs.items()
                                     if not row["unique_numeric_branch_in_both_captures"]]})
    return {"schema": "singularitydog.angle-epoch-series-review.v1",
            "status": "NUMERIC_COMPARISON_ONLY",
            "capture_sources": [{key: row[key] for key in
                                 ("path", "sha256", "boot_id", "motor_power_epoch_label")}
                                for row in captures],
            "rows_by_id": rows,
            "series_summary": {
                "capture_count": len(captures), "adjacent_transition_count": len(captures) - 1,
                "capture_and_transition_index_base": 0,
                "adjacent_transitions": transition_summaries,
                "whole_turn_changed_transition_count": sum(
                    bool(row["whole_turn_changed_ids"]) for row in transition_summaries),
                "whole_turn_change_axis_event_count": sum(
                    len(row["whole_turn_changed_ids"]) for row in transition_summaries),
                "max_abs_adjacent_model_delta_deg_if_numeric": _max_abs_numeric(
                    [pair for row in rows.values() for pair in row["adjacent_transitions"]]),
                "max_abs_model_delta_vs_first_deg_if_numeric": _max_abs_numeric(
                    [pair for row in rows.values() for pair in row["comparisons_to_first_capture"]])},
            "uid_mismatch_ids": [mid for mid in IDS if any(
                not row["uid_matches"] for row in rows[str(mid)]["captures"])],
            "nonunique_branch_ids": [mid for mid in IDS if not rows[str(mid)][
                "unique_numeric_branch_in_every_capture"]],
            "pose_equivalence_verified": False,
            "physical_zero_direction_or_limits_verified_by_this_tool": False,
            "power_off_on_observed_by_this_tool": False,
            "motor_targets_generated": False,
            "approved_for_runtime": False,
            "output_allowed": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--capture", type=Path, action="append", required=True,
                        help="Read-only capture, in time order; repeat at least twice")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        _, axes = load_profile(args.profile)
        result = review(axes, args.capture)
        write_private(args.output, result)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(json.dumps({"status": result["status"], "output": str(args.output),
                      "uid_mismatch_ids": result["uid_mismatch_ids"],
                      "nonunique_branch_ids": result["nonunique_branch_ids"],
                      "approved_for_runtime": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
