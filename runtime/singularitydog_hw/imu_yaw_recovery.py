"""Recover an overlong yaw window from a complete, restored IMU-only capture.

File-only. The failed session, four Enter markers, motion capture and prior
stationary A/B remain immutable inputs. A fresh audit is never runtime approval.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re

from . import imu_commissioning_audit as audit
from . import imu_commissioning_capture as guided
from . import imu_fixed_mount_baseline as baseline
from . import imu_yaw_retake as retake


_OLD_WINDOW_ERROR = re.compile(
    r"ValueError\('(outbound|return) window \d+\.\d+s: allowed 0\.50\.\.10\.00s'\)"
)


def _source(path, pinned, *, parse=True):
    path = Path(path).expanduser().absolute()
    baseline._require(not path.is_symlink() and path.is_file(), "regular source file required: " + str(path))
    path = path.resolve()
    raw = path.read_bytes()
    pinned[path] = hashlib.sha256(raw).hexdigest()
    return baseline._json(raw) if parse else raw


def _same(a, b, label):
    canonical = lambda value: json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    baseline._require(canonical(a) == canonical(b), label + " mismatch")


def _provenance_matches(actual, expected, label):
    baseline._require(isinstance(expected, dict), label + " provenance missing")
    for key in ("summary_sha256", "events_sha256", "measurement_sequence_sha256",
                "source_sha256", "monotonic_interval_ns", "wall_interval_ns"):
        _same(actual[key], expected.get(key), label + " " + key)


def _marker_windows(path, rows, capture_seconds, pinned):
    path = Path(path).expanduser().absolute()
    baseline._require(not path.is_symlink() and path.is_file(), "regular marker sidecar required")
    path = path.resolve()
    raw = path.read_bytes()
    pinned[path] = hashlib.sha256(raw).hexdigest()
    lines = raw.splitlines()
    baseline._require(len(lines) == 6 and all(line.strip() for line in lines),
                      "marker sidecar requires exactly six complete records")
    records = [baseline._json(line) for line in lines]
    first, last = rows[0]["monotonic_ns"], rows[-1]["monotonic_ns"]
    _same(records[0], {"kind": "first_sample", "monotonic_ns": first,
                       "capture_seconds_requested": capture_seconds}, "first marker record")
    _same(records[-1], {"kind": "last_sample", "monotonic_ns": last,
                        "elapsed_from_first_sample_s": (last-first)/1e9}, "last marker record")
    markers = {}
    for record, (name, _) in zip(records[1:-1], retake.MARKERS):
        baseline._require(set(record) == {"kind", "name", "monotonic_ns", "elapsed_from_first_sample_s"}
                          and record["kind"] == "operator_marker" and record["name"] == name,
                          "marker order or fields invalid")
        stamp = record["monotonic_ns"]
        baseline._require(type(stamp) is int and first < stamp < last,
                          "marker timestamp outside capture")
        _same(record["elapsed_from_first_sample_s"], (stamp-first)/1e9,
              "marker elapsed timestamp")
        markers[name] = stamp
    return markers, retake.marker_windows(markers, first, last)


def _check_pins(pinned):
    for path, expected in pinned.items():
        baseline._require(not path.is_symlink() and path.is_file()
                          and hashlib.sha256(path.read_bytes()).hexdigest() == expected,
                          "source changed during yaw recovery: " + str(path))


def evaluate(session, static_manifest, *, static_a=None, static_b=None,
             operator_direction_confirmed):
    """Validate and recompute in memory; return manifest, audit and pinned hashes."""
    baseline._require(type(operator_direction_confirmed) is bool,
                      "explicit operator direction confirmation required")
    session = Path(session).expanduser().resolve()
    pinned = {}
    receipt = _source(session / "session-incomplete.json", pinned)
    error = receipt.get("error")
    baseline._require(receipt.get("status") == "INCOMPLETE" and isinstance(error, str)
                      and _OLD_WINDOW_ERROR.fullmatch(error) is not None,
                      "recovery requires only the prior 10-second yaw-window rejection")
    original = receipt.get("manifest_so_far")
    baseline._require(isinstance(original, dict) and original.get("schema_version") == 1
                      and original.get("movements") == [] and original.get("runtime_approval") is False,
                      "incomplete session manifest invalid")
    assertions = original.get("operator_assertions", {})
    baseline._require(isinstance(assertions, dict) and all(assertions.get(key) is True for key in
                      ("qdd_power_off", "body_supported", "imu_mount_unchanged_since_static")),
                      "incomplete session lacks power-off/support/unchanged-mount assertions")
    source_info = original.get("stationary_source")
    baseline._require(isinstance(source_info, dict)
                      and isinstance(source_info.get("manifest"), str)
                      and bool(source_info["manifest"].strip()),
                      "stationary source provenance missing")
    source_manifest = _source(static_manifest, pinned)
    source_path = Path(static_manifest).expanduser().absolute().resolve()
    _same(pinned[source_path], source_info.get("manifest_sha256"), "stationary manifest SHA-256")
    baseline._require(source_manifest.get("schema_version") == 1, "stationary manifest schema invalid")
    source_static = source_manifest.get("stationary", {})
    source_assertions = source_manifest.get("operator_assertions", {})
    baseline._require(isinstance(source_static, dict) and source_static.get("operator_confirmed") is True
                      and isinstance(source_assertions, dict)
                      and source_assertions.get("qdd_power_off") is True
                      and source_assertions.get("body_supported") is True,
                      "stationary source lacks operator assertions")
    _same(source_manifest.get("mount_candidate", audit.mount_candidate()),
          original.get("mount_candidate"), "mount candidate")
    resolved_static = {}
    for key, alternate in (("a", static_a), ("b", static_b)):
        name = source_static.get(key)
        baseline._require(isinstance(name, str) and bool(name.strip()), "stationary source path missing")
        origin = Path(name).expanduser()
        origin = origin if origin.is_absolute() else Path(source_info["manifest"]).parent / origin
        expected = source_info.get("static_" + key, {})
        _same(str(origin), expected.get("directory"), "original static " + key + " path")
        _same(str(origin), original.get("stationary", {}).get(key),
              "incomplete static " + key + " path")
        resolved_static[key] = Path(alternate).expanduser().resolve() if alternate is not None else origin
        for filename in ("summary.json", "events.jsonl"):
            _source(resolved_static[key] / filename, pinned, parse=filename == "summary.json")
        _, _, _, provenance = baseline._load_capture(resolved_static[key])
        _provenance_matches(provenance, expected, "static " + key)

    capture = session / "turn_left"
    for filename in ("summary.json", "events.jsonl"):
        _source(capture / filename, pinned, parse=filename == "summary.json")
    metadata, rows, _, capture_provenance = baseline._load_capture(capture)
    markers, windows = _marker_windows(session / "yaw-markers.jsonl", rows,
                                       metadata["plan"]["capture_seconds"], pinned)
    movement = {"movement": "turn_left", "capture": str(capture), **windows,
                "operator_direction_confirmed": operator_direction_confirmed,
                "marker_monotonic_ns": markers,
                "marker_method": "operator_enter_at_motion_boundaries"}
    evidence = {"failed_session_receipt": str(session / "session-incomplete.json"),
                "failed_session_receipt_sha256": pinned[(session / "session-incomplete.json").resolve()],
                "marker_sidecar": str(session / "yaw-markers.jsonl"),
                "marker_sidecar_sha256": pinned[(session / "yaw-markers.jsonl").resolve()],
                "stationary_manifest": str(source_path),
                "stationary_manifest_sha256": pinned[source_path],
                "static_a": source_info["static_a"], "static_b": source_info["static_b"],
                "turn_left_capture": capture_provenance}
    manifest = {"schema_version": 1, "mount_candidate": original["mount_candidate"],
                "stationary": {"a": str(resolved_static["a"]), "b": str(resolved_static["b"]),
                               "operator_confirmed": True},
                "stationary_source": source_info, "movements": [movement],
                "recovery_evidence": evidence, "hardware_scope": "file-only IMU yaw recovery; no CAN",
                "operator_assertions": assertions, "runtime_approval": False}
    result = audit.audit_manifest(manifest)
    baseline._require(result["checks"]["gyro_bias_candidate"], "stationary A/B no longer eligible")
    motion = result["movements"][0]
    baseline._require("error" not in motion, "recovered motion audit error: " + motion.get("error", ""))
    _provenance_matches(motion["provenance"], capture_provenance, "turn-left capture")
    _provenance_matches(result["stationary"]["provenance"]["a"], source_info["static_a"], "audit static a")
    _provenance_matches(result["stationary"]["provenance"]["b"], source_info["static_b"], "audit static b")
    _check_pins(pinned)
    return manifest, result, pinned


def recover(session, static_manifest, output, *, static_a=None, static_b=None,
            operator_direction_confirmed):
    """Write exclusive, private, unapproved manifest/audit outside Git."""
    manifest, result, pinned = evaluate(
        session, static_manifest, static_a=static_a, static_b=static_b,
        operator_direction_confirmed=operator_direction_confirmed)
    output = Path(output).expanduser().resolve()
    retake._outside_git(output)
    baseline._require(not output.exists(), "new output directory required")
    _check_pins(pinned)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    guided._write_new(output / "manifest.json", manifest)
    manifest_raw = (output / "manifest.json").read_bytes()
    result["manifest_sha256"] = hashlib.sha256(manifest_raw).hexdigest()
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    guided._write_new(output / "audit.json", result)
    _check_pins(pinned)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, help="original incomplete r5 session directory")
    parser.add_argument("--static-manifest", required=True, help="original stationary A/B manifest")
    parser.add_argument("--static-a", help="relocated static A capture directory")
    parser.add_argument("--static-b", help="relocated static B capture directory")
    parser.add_argument("--output", required=True, help="new private directory outside Git")
    parser.add_argument("--direction-confirmation", choices=("yes", "no"),
                        help="explicit operator observation of nose-left motion and return")
    args = parser.parse_args(argv)
    direction = args.direction_confirmation
    if direction is None:
        try:
            direction = input("記録境界に合わせ、鼻先を犬の左へ動かして元へ戻しましたか [y/n]: ").strip().lower()
        except EOFError:
            parser.error("explicit operator direction confirmation required")
        if direction not in ("y", "n"):
            parser.error("direction confirmation must be y or n")
    try:
        result = recover(args.session, args.static_manifest, args.output,
                         static_a=args.static_a, static_b=args.static_b,
                         operator_direction_confirmed=direction in ("y", "yes"))
    except (OSError, ValueError) as error:
        parser.exit(2, "IMU yaw recovery rejected: %s\n" % error)
    motion = result["movements"][0]
    print(json.dumps({"turn_left_axis_sign": result["checks"]["turn_left_axis_sign"],
                      "failed_gates": motion.get("failed_gates", []),
                      "approved_for_runtime": False,
                      "output": str(Path(args.output).expanduser().resolve())}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
