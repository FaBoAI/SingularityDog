#!/usr/bin/env python3
"""File-only L-reference and direct return diagnostics, never a zero calibration.

An explicit input manifest selects only the axes actually placed in a nominal L
pose. Other axes in a twelve-axis capture are not reference observations. Saved
captures and operator notes are SHA-pinned. Cross-boot full turns are displayed
as orientation arithmetic only; within-boot differences are never wrapped.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_angle_calibration import (  # noqa: E402
    IDS, TAU, UNKNOWN_EPOCHS, AxisCalibration, current_values, finite_number, load_profile,
    need, read, signed_periodic_delta_rad, write_private,
)
from singularitydog_hw.angle_calibration_audit import equivalent_branch_candidates  # noqa: E402

INPUT_SCHEMA = "singularitydog.joint-reference-repeatability-input.v1"
REPORT_SCHEMA = "singularitydog.joint-reference-repeatability-analysis.v1"


def _text(value, name):
    need(type(value) is str and bool(value.strip()), "Nonempty " + name + " required")
    return value


def _ids(value):
    need(type(value) is list and bool(value)
         and all(type(mid) is int and mid in IDS for mid in value)
         and len(set(value)) == len(value), "Explicit unique reference IDs required")
    return value


class Sources:
    def __init__(self, root):
        self.root, self.pins = Path(root), {}

    def source(self, pin):
        need(type(pin) is dict, "Source pin object required")
        name, digest = _text(pin.get("path"), "source path"), pin.get("sha256")
        need(type(digest) is str and len(digest) == 64
             and all(c in "0123456789abcdef" for c in digest), "Source SHA256 required")
        path = Path(name).expanduser()
        if not path.is_absolute():
            path = self.root / path
        need(not path.is_symlink() and path.is_file(), "Regular source file required")
        path = path.resolve()
        data, actual = read(path, digest)
        need(str(path) not in self.pins or self.pins[str(path)] == actual,
             "One source path was selected with conflicting hashes")
        self.pins[str(path)] = actual
        return data, {"path": str(path), "sha256": actual}

    def capture(self, pin, axes):
        data, source = self.source(pin)
        raw, uids = current_values(data)
        need(all(uids[str(mid)] == axes[mid].uid for mid in IDS), "Capture UID differs from profile")
        boot = _text(data.get("boot_id"), "capture boot ID")
        need(len(set(uids.values())) == 12, "Duplicate motor UID")
        return {"data": data, "source": source, "raw": raw, "boot": boot}

    def recheck(self):
        for name, digest in self.pins.items():
            path = Path(name)
            need(not path.is_symlink() and path.is_file()
                 and hashlib.sha256(path.read_bytes()).hexdigest() == digest,
                 "Source changed during analysis: " + name)


def _observation(sources, pin, captures):
    if pin is None:
        return {"source": None, "operator_report_available": False}
    data, source = sources.source(pin)
    event = data.get("power_event")
    need(type(event) is dict and event.get("inferred_from_boot") is False,
         "Operator observation must not infer motor power epoch from boot")
    hashes = event.get("capture_sha256")
    phases = ("l",) if len(captures) == 1 else ("baseline", "moved", "return")
    need(len(captures) == len(phases) and type(hashes) is dict
         and all(hashes.get(phase) == capture["source"]["sha256"]
                 for phase, capture in zip(phases, captures)),
         "Operator observation does not bind the selected captures")
    need(all(c["boot"] == event.get("boot_id") for c in captures), "Operator observation boot differs")
    return {"source": source, "operator_report_available": True,
            "operator_l_pose_reported": data.get("l_pose_observed") is True,
            "operator_return_reported": data.get("returned_to_baseline") is True,
            "operator_power_unchanged_reported": event.get("unchanged_since_seed") is True,
            "independent_physical_verification": False}


def _nominal(mid):
    return -math.pi / 2 if (mid - 1) % 3 == 0 else 0.


def _difference(a, b, mid):
    """Compare b-a; modulo is only a cross-boot descriptive decomposition."""
    direct = finite_number(b["raw"][str(mid)] - a["raw"][str(mid)], "direct difference")
    same_boot = a["boot"] == b["boot"]
    result = {"from_capture_sha256": a["source"]["sha256"],
              "to_capture_sha256": b["source"]["sha256"],
              "same_boot": same_boot, "direct_raw_delta_deg": finite_number(math.degrees(direct), "degree difference"),
              "motor_power_epoch_inferred": False, "epoch_branch_binding_created": False}
    if same_boot:
        result.update(orientation_delta_deg=None, full_turn_difference=None,
                      within_boot_half_turn_or_larger=abs(direct) >= math.pi)
    else:
        try:
            orientation = signed_periodic_delta_rad(b["raw"][str(mid)], a["raw"][str(mid)])
            result.update(orientation_delta_deg=math.degrees(orientation),
                          full_turn_difference=round((direct - orientation) / TAU),
                          cross_boot_orientation_ambiguous=False)
        except ValueError:
            result.update(orientation_delta_deg=None, full_turn_difference=None,
                          cross_boot_orientation_ambiguous=True)
    return result


def _capture_time(capture):
    samples = [sample for row in capture["data"]["telemetry"]["rows"].values()
               for sample in row.get("position_samples", [])]
    need(bool(samples), "Return trial needs saved position timestamps")
    for sample in samples:
        need(type(sample.get("request_monotonic_ns")) is int
             and type(sample.get("reply_monotonic_ns")) is int
             and 0 <= sample["request_monotonic_ns"] <= sample["reply_monotonic_ns"],
             "Invalid saved position timestamps")
    return min(s["request_monotonic_ns"] for s in samples), max(s["reply_monotonic_ns"] for s in samples)


def analyze(manifest, profile, axes, sources):
    need(manifest.get("schema") == INPUT_SCHEMA, "Repeatability input schema required")
    references, trials = manifest.get("references", []), manifest.get("return_trials", [])
    need(type(references) is list and type(trials) is list and bool(references or trials),
         "Reference observations or return trials required")
    groups, seen, reference_rows = {}, set(), []
    for item in references:
        need(type(item) is dict, "Reference selection object required")
        method = _text(item.get("method"), "reference method")
        label = _text(item.get("label"), "reference label")
        ids = _ids(item.get("ids"))
        capture = sources.capture(item.get("capture"), axes)
        observation = _observation(sources, item.get("observation"), [capture])
        for mid in ids:
            key = (mid, method)
            token = (mid, capture["source"]["sha256"])
            need(token not in seen, "Repeated capture cannot be an independent reference")
            seen.add(token)
            axis, nominal = axes[mid], _nominal(mid)
            residual = signed_periodic_delta_rad(axis.sign * capture["raw"][str(mid)] + axis.offset_rad, nominal)
            row = {"motor_id": mid, "label": label, "method": method, "source": capture["source"],
                   "boot_id": capture["boot"], "raw_rad_unmodified": capture["raw"][str(mid)],
                   "nominal_model_deg_diagnostic_only": math.degrees(nominal),
                   "residual_from_nominal_deg": math.degrees(residual),
                   "offset_change_to_nominal_rad_diagnostic_only": -residual,
                   "sampling_span_deg": finite_number(capture["data"]["telemetry"]["rows"][str(mid)]["position_span_deg"], "span"),
                   "observation": observation, "absolute_origin_error_rad": None,
                   "nominal_angle_measured": False}
            reference_rows.append(row)
            groups.setdefault(key, []).append((capture, row))
    group_rows = []
    for (mid, method), points in sorted(groups.items()):
        pairs = [_difference(a[0], b[0], mid) for n, a in enumerate(points) for b in points[n + 1:]]
        ambiguous = any(p.get("within_boot_half_turn_or_larger", False)
                        or p.get("cross_boot_orientation_ambiguous", False) for p in pairs)
        differences = []
        for c, _ in points:
            if c["boot"] == points[0][0]["boot"]:
                differences.append(finite_number(c["raw"][str(mid)] - points[0][0]["raw"][str(mid)], "reference difference"))
            else:
                try:
                    differences.append(signed_periodic_delta_rad(c["raw"][str(mid)], points[0][0]["raw"][str(mid)]))
                except ValueError:
                    ambiguous = True
        if differences and max(differences) - min(differences) >= math.pi:
            # A group covering half a turn cannot define one small reference
            # neighborhood from an anchor. Keep the raw/pair evidence only.
            ambiguous = True
        group_rows.append({"motor_id": mid, "method": method, "reference_count": len(points),
                           "boot_count": len({c["boot"] for c, _ in points}), "pairs": pairs,
                           "observed_reference_span_deg": (math.degrees(max(differences) - min(differences))
                                                           if len(points) > 1 and not ambiguous else None),
                           "ambiguity_or_discontinuity_present": ambiguous,
                           "max_sampling_span_deg": max(r["sampling_span_deg"] for _, r in points),
                           "absolute_accuracy_bound_rad": None,
                           "observed_span_is_error_bound": False})
    return_rows = []
    for trial in trials:
        need(type(trial) is dict and type(trial.get("motor_id")) is int
             and trial["motor_id"] in IDS, "Return trial motor ID required")
        mid, label = trial["motor_id"], _text(trial.get("label"), "trial label")
        captures = [sources.capture(trial.get(phase), axes) for phase in ("baseline", "moved", "returned")]
        need(len({c["source"]["sha256"] for c in captures}) == 3, "Three independent trial captures required")
        need(len({c["boot"] for c in captures}) == 1, "Return trial boot changed; do not wrap")
        times = [_capture_time(c) for c in captures]
        need(times[0][1] < times[1][0] and times[1][1] < times[2][0], "Trial captures not in timestamp order")
        epochs = [c["data"].get("motor_power_epoch") for c in captures]
        known = [e for e in epochs if type(e) is str and e not in UNKNOWN_EPOCHS]
        need(not known or len(known) == 3 and len(set(known)) == 1, "Recorded motor power epoch changed")
        observation = _observation(sources, trial.get("observation"), captures)
        base, moved, returned = (c["raw"] for c in captures)
        movement = {str(i): finite_number(math.degrees(moved[str(i)] - base[str(i)]), "move difference") for i in IDS}
        back = {str(i): finite_number(math.degrees(returned[str(i)] - base[str(i)]), "return difference") for i in IDS}
        recovery = {str(i): finite_number(math.degrees(returned[str(i)] - moved[str(i)]), "recovery difference") for i in IDS}
        return_rows.append({"motor_id": mid, "label": label, "sources": [c["source"] for c in captures],
                            "boot_id": captures[0]["boot"], "recorded_motor_epoch_known": bool(known),
                            "observation": observation, "direct_raw_move_deg_by_id": movement,
                            "direct_raw_return_deg_by_id": back, "direct_raw_recovery_deg_by_id": recovery,
                            "selected_model_return_deg": axes[mid].sign * back[str(mid)],
                            "half_turn_or_larger_present": any(abs(value) >= 180 for value in [*movement.values(), *back.values(), *recovery.values()]),
                            "return_tolerance_or_approval_created": False})
    current_rows = {}
    if manifest.get("current") is not None:
        current = sources.capture(manifest["current"], axes)
        for mid, axis in axes.items():
            center = (axis.lower_rad + axis.upper_rad) / 2
            q = center + signed_periodic_delta_rad(axis.sign * current["raw"][str(mid)] + axis.offset_rad, center)
            correction = min(max(q, axis.lower_rad), axis.upper_rad) - q
            replacements = []
            for ref in reference_rows:
                if ref["motor_id"] == mid:
                    adjusted = center + signed_periodic_delta_rad(q + ref["offset_change_to_nominal_rad_diagnostic_only"], center)
                    replacements.append({"reference_sha256": ref["source"]["sha256"], "method": ref["method"],
                                         "model_rad_if_nominal_L_replaced_zero": adjusted,
                                         "inside_numeric_interval": axis.lower_rad <= adjusted <= axis.upper_rad})
            current_rows[str(mid)] = {"source": current["source"], "raw_rad_unmodified": current["raw"][str(mid)],
                                     "existing_periodic_branch_candidates": equivalent_branch_candidates(current["raw"][str(mid)], axis),
                                     "orientation_representative_rad_diagnostic_only": q,
                                     "numeric_interval_rad": [axis.lower_rad, axis.upper_rad],
                                     "minimum_offset_change_rad_to_touch_numeric_interval": correction,
                                     "minimum_offset_change_deg_to_touch_numeric_interval": math.degrees(correction),
                                     "nominal_L_zero_replacement_diagnostics": replacements,
                                     "absolute_origin_error_rad": None,
                                     "range_failure_explained_by_absolute_origin_error": None}
    return {"schema": REPORT_SCHEMA, "status": "FILE_ONLY_REPEATABILITY_DIAGNOSTIC",
            "assembly_revision_from_profile": profile["assembly_revision"],
            "same_assembly_independently_verified": False, "reference_rows": reference_rows,
            "reference_groups": group_rows, "return_trials": return_rows, "current_by_id": current_rows,
            "absolute_origin_error_rad": None, "absolute_accuracy_bound_rad": None,
            "profile_changed": False, "raw_angles_modified": False, "calibration_approved_for_runtime": False,
            "epoch_branch_binding_created": False, "motor_output_available": False, "output_allowed": False,
            "limitations": ["Observed reference spread and return differences are not absolute accuracy bounds.",
                            "UID equality does not independently prove unchanged assembly or motor power epoch.",
                            "Cross-boot turns are orientation decompositions, never live sequence wrapping.",
                            "Numeric model intervals are not independently verified mechanical limits.",
                            "Nominal L angles and alternative zero diagnostics are not measured physical truth."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        need(not args.input.is_symlink() and not args.profile.is_symlink(), "Regular input files required")
        manifest, manifest_sha = read(args.input)
        profile, axes = load_profile(args.profile)
        profile_sha = axes[1].calibration_sha256
        sources = Sources(args.input.resolve().parent)
        sources.pins.update({str(args.input.resolve()): manifest_sha, str(args.profile.resolve()): profile_sha})
        module_root = Path(__file__).resolve().parents[1]
        for relative in ("tools/analyze_joint_reference_repeatability.py", "tools/audit_angle_calibration.py",
                         "runtime/singularitydog_hw/angle_calibration_audit.py"):
            path = module_root / relative
            sources.pins[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        report = analyze(manifest, profile, axes, sources)
        report["source_sha256"] = dict(sources.pins)
        sources.recheck()
        write_private(args.output, report)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(json.dumps({"status": report["status"], "output": str(args.output),
                      "sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(), "output_allowed": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
