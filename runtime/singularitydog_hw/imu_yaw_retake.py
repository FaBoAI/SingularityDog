"""Operator-paced, IMU-only yaw retake using a prior stationary A/B pair.

This never opens CAN, drives a motor, changes calibration, or approves runtime
use. The existing imu_capture child owns I2C and register restoration. The
operator presses Enter at each actual movement boundary, so capture windows
are not inferred from when a terminal prompt happened to appear.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time

from . import imu_commissioning_audit as audit
from . import imu_commissioning_capture as guided
from . import imu_fixed_mount_baseline as baseline


DEFAULT_CAPTURE_SECONDS = 60
INITIAL_STILL_SECONDS = 1.0
BETWEEN_MOVEMENTS_STILL_SECONDS = 1.0
MARKERS = (
    ("outbound_start", "左旋回を始める直前に Enter。押したら鼻先を犬の左へ5〜15°ゆっくり動かします。"),
    ("outbound_end", "左へ動かし終えたら、その姿勢を保って Enter。"),
    ("return_start", "元へ戻し始める直前に Enter。押したらゆっくり戻します。"),
    ("return_end", "元の姿勢で静止したら Enter。そのまま収録終了まで支えてください。"),
)


def _outside_git(path):
    if any((parent / ".git").exists() for parent in (path, *path.parents)):
        raise ValueError("capture output must be outside Git")


def _stationary_source(manifest_path):
    """Resolve and re-audit prior A/B, retaining their original byte hashes."""
    source = Path(manifest_path).expanduser().resolve()
    raw = source.read_bytes()
    original = baseline._json(raw)
    baseline._require(original.get("schema_version") == 1, "require commissioning manifest schema 1")
    stationary = original.get("stationary")
    baseline._require(isinstance(stationary, dict) and stationary.get("operator_confirmed") is True,
                      "source A/B lack operator-confirmed stationarity")
    assertions = original.get("operator_assertions", {})
    baseline._require(assertions.get("qdd_power_off") is True and assertions.get("body_supported") is True,
                      "source A/B lack power-off and support assertions")
    resolved = {}
    for key in ("a", "b"):
        name = stationary.get(key)
        baseline._require(isinstance(name, str) and bool(name.strip()), "source A/B path missing")
        p = Path(name).expanduser()
        resolved[key] = str((p if p.is_absolute() else source.parent / p).resolve())
    candidate = original.get("mount_candidate", audit.mount_candidate())
    audit_input = {"schema_version": 1, "mount_candidate": candidate,
                   "stationary": {**resolved, "operator_confirmed": True}, "movements": []}
    prior = audit.audit_manifest(audit_input, root=source.parent)
    baseline._require(prior["checks"]["gyro_bias_candidate"], "source A/B do not support a gyro bias candidate")
    provenance = prior["stationary"]["provenance"]
    return audit_input["stationary"], prior["mount_candidate"], {
        "manifest": str(source), "manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "static_a": provenance["a"],
        "static_b": provenance["b"],
    }


def marker_windows(markers, first_sample_ns, last_sample_ns):
    """Turn actual Enter timestamps into audit windows, rejecting weak timing."""
    names = [name for name, _ in MARKERS]
    if set(markers) != set(names):
        raise ValueError("four complete yaw markers are required")
    times = [markers[name] for name in names]
    if any(type(t) is not int for t in times) or not all(a < b for a, b in zip(times, times[1:])):
        raise ValueError("yaw markers must be ordered monotonic timestamps")
    seconds = [(t - first_sample_ns) / 1e9 for t in times]
    duration = (last_sample_ns - first_sample_ns) / 1e9
    lo = audit.AXIS_LIMITS["minimum_window_s"]
    hi = audit.AXIS_LIMITS["maximum_window_s"]
    if seconds[0] < .5:
        raise ValueError(f"outbound_start {seconds[0]:.2f}s: need at least 0.50s after first sample")
    for label, start, end in (("outbound", seconds[0], seconds[1]),
                              ("return", seconds[2], seconds[3])):
        length = end - start
        if not lo <= length <= hi:
            raise ValueError(f"{label} window {length:.2f}s: allowed {lo:.2f}..{hi:.2f}s")
    hold = seconds[2] - seconds[1]
    if hold < .5:
        raise ValueError(f"hold between movements {hold:.2f}s: need at least 0.50s")
    tail = duration - seconds[3]
    if tail < .5:
        raise ValueError(f"still tail after return_end {tail:.2f}s: need at least 0.50s")
    return {"outbound_s": seconds[:2], "return_s": seconds[2:]}


def _child_alive(process, deadline):
    if process.poll() is not None:
        raise RuntimeError("IMU capture ended before all yaw markers were recorded")
    if time.monotonic() >= deadline:
        raise TimeoutError("IMU yaw capture timed out")


def _mark(process, deadline, name, prompt):
    print("\a" + prompt + " [Enterで記録 / qで中止]: ", end="", flush=True)
    while True:
        _child_alive(process, deadline)
        if not select.select([sys.stdin], [], [], .05)[0]:
            continue
        response = sys.stdin.readline()
        if not response or response.strip().lower() == "q":
            raise InterruptedError("operator cancelled yaw retake")
        if response.strip():
            print("Enterのみを押してください。qで中止できます。", flush=True)
            continue
        stamp = time.monotonic_ns()
        print("\a記録しました。", flush=True)
        return name, stamp


def _wait_still_after(marker_ns, seconds, process, deadline):
    earliest = marker_ns + int(seconds * 1e9)
    while time.monotonic_ns() < earliest:
        _child_alive(process, deadline)
        time.sleep(.02)


def capture_yaw(output, *, capture_seconds=DEFAULT_CAPTURE_SECONDS):
    """Collect one complete yaw recording and four operator boundary markers."""
    if type(capture_seconds) is not int or not 20 <= capture_seconds <= 120:
        raise ValueError("capture_seconds must be 20..120")
    command = [sys.executable, "-m", "singularitydog_hw.imu_capture", "--execute",
               "--face", "unverified", "--settle-seconds", "3", "--seconds", str(capture_seconds),
               "--output", str(output)]
    transcript = output.parent / (output.name + "-capture.stdout")
    process = None
    markers = {}
    started = time.monotonic()
    deadline = started + capture_seconds + 18
    marker_sidecar = output.parent / "yaw-markers.jsonl"
    with transcript.open("x") as stream, marker_sidecar.open("x") as marker_log:
        def record_marker(event):
            marker_log.write(json.dumps(event, allow_nan=False) + "\n")
            marker_log.flush()
        try:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            first = None
            while first is None:
                _child_alive(process, deadline)
                first = guided._first_sample(output / "events.jsonl")
                if first is None:
                    time.sleep(.02)
            record_marker({"kind": "first_sample", "monotonic_ns": first,
                           "capture_seconds_requested": capture_seconds})
            print("\a収録開始。最初の1秒は動かさず、次の合図を待ってください。", flush=True)
            _wait_still_after(first, INITIAL_STILL_SECONDS, process, deadline)
            print("各動作の直前・直後にEnterで時刻を記録します。", flush=True)
            for name, prompt in MARKERS:
                key, stamp = _mark(process, deadline, name, prompt)
                markers[key] = stamp
                record_marker({"kind": "operator_marker", "name": key, "monotonic_ns": stamp,
                               "elapsed_from_first_sample_s": (stamp - first) / 1e9})
                if key == "outbound_end":
                    print("その姿勢で1秒静止してください。", flush=True)
                    _wait_still_after(stamp, BETWEEN_MOVEMENTS_STILL_SECONDS, process, deadline)
            while process.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError("IMU yaw capture timed out")
                time.sleep(.05)
            if process.returncode:
                raise RuntimeError("IMU capture failed; inspect " + str(transcript))
        finally:
            if process is not None and process.poll() is None:
                process.terminate()  # imu_capture handles SIGTERM and restores settings.
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                    print("IMU process required SIGKILL; restoration is NOT confirmed.", file=sys.stderr)
        metadata, rows, _, _ = baseline._load_capture(output)
        baseline._require(metadata.get("restore_status") == "restored", "IMU restoration not confirmed")
        record_marker({"kind": "last_sample", "monotonic_ns": rows[-1]["monotonic_ns"],
                       "elapsed_from_first_sample_s":
                           (rows[-1]["monotonic_ns"] - rows[0]["monotonic_ns"]) / 1e9})
        windows = marker_windows(markers, rows[0]["monotonic_ns"], rows[-1]["monotonic_ns"])
    return {"movement": "turn_left", "capture": output.name, **windows,
            "operator_direction_confirmed": False, "marker_monotonic_ns": markers,
            "marker_method": "operator_enter_at_motion_boundaries"}


def run_session(output, *, reuse_static_from, capture_seconds=DEFAULT_CAPTURE_SECONDS):
    stationary, mount, source = _stationary_source(reuse_static_from)
    output = Path(output).expanduser().resolve()
    _outside_git(output)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    manifest = {"schema_version": 1, "mount_candidate": mount, "stationary": stationary,
                "stationary_source": source, "movements": [], "hardware_scope": "IMU only; no CAN",
                "operator_assertions": {"qdd_power_off": True, "body_supported": True,
                                        "imu_mount_unchanged_since_static": True},
                "runtime_approval": False}
    try:
        print("QDDの40VはOffのまま。胴体を支え、IMUの取付を変えません。", flush=True)
        print("左旋回だけを再収録します。開始・終了の4箇所でEnterを押し、各移動は15秒以内、動作中は水平に近い姿勢を保ちます。", flush=True)
        guided._enter("抵抗・接触がなく、支えたまま鼻先を左へ5〜15°動かして戻せる状態にしてください")
        item = capture_yaw(output / "turn_left", capture_seconds=capture_seconds)
        item["operator_direction_confirmed"] = input(
            "記録した境界に合わせ、鼻先を犬の左へ動かして元へ戻しましたか [y/n]: ").strip().lower() == "y"
        manifest["movements"].append(item)
        if hashlib.sha256(Path(source["manifest"]).read_bytes()).hexdigest() != source["manifest_sha256"]:
            raise RuntimeError("source stationary manifest changed during yaw retake")
        guided._write_new(output / "manifest.json", manifest)
        result = audit.write_audit(output / "manifest.json", output / "audit.json")
        if (result["stationary"]["provenance"]["a"] != source["static_a"]
                or result["stationary"]["provenance"]["b"] != source["static_b"]):
            raise RuntimeError("source stationary capture changed during yaw retake")
        guided._write_new(output / "imu-mount-candidate.json", result["mount_candidate"])
        if result["checks"]["gyro_bias_candidate"]:
            guided._write_new(output / "gyro-bias-candidate.json", result["stationary"])
        motion = next(r for r in result["movements"] if r["movement"] == "turn_left")
        review = {"status": result["status"], "approved_for_runtime": False,
                  "gyro_bias_candidate": result["checks"]["gyro_bias_candidate"],
                  "turn_left_axis_sign": result["checks"]["turn_left_axis_sign"],
                  "operator_direction_confirmed": item["operator_direction_confirmed"],
                  "operator_motion_audit_discrepancy": item["operator_direction_confirmed"]
                      and not result["checks"]["turn_left_axis_sign"],
                  "failed_gates": motion.get("failed_gates", []), "audit_error": motion.get("error"),
                  "stationary_source_manifest_sha256": source["manifest_sha256"]}
        guided._write_new(output / "retake-review.json", review)
        return review
    except BaseException as error:
        guided._write_new(output / "session-incomplete.json", {
            "error": repr(error), "manifest_so_far": manifest,
            "status": "INCOMPLETE", "approved_for_runtime": False})
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-static-from", required=True, help="prior commissioning manifest.json")
    parser.add_argument("--output", required=True, help="new private directory outside Git")
    parser.add_argument("--capture-seconds", type=int, default=DEFAULT_CAPTURE_SECONDS,
                        help="yaw recording length; default 60 s")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--motor-power-off", action="store_true")
    parser.add_argument("--body-supported", action="store_true")
    parser.add_argument("--mount-unchanged", action="store_true")
    args = parser.parse_args(argv)
    if not 20 <= args.capture_seconds <= 120:
        parser.error("capture-seconds must be 20..120")
    if not args.execute:
        print(json.dumps({"mode": "PLAN_ONLY", "CAN_opened": False,
                          "captures": ["turn_left"], "static_reused_from": args.reuse_static_from,
                          "yaw_recording_seconds": args.capture_seconds,
                          "operator_boundary_markers": [name for name, _ in MARKERS],
                          "runtime_approval": False}))
        return 0
    if not (args.motor_power_off and args.body_supported and args.mount_unchanged):
        parser.error("yaw retake requires --motor-power-off --body-supported --mount-unchanged")
    try:
        result = run_session(args.output, reuse_static_from=args.reuse_static_from,
                             capture_seconds=args.capture_seconds)
    except (OSError, ValueError, RuntimeError, InterruptedError, KeyboardInterrupt) as error:
        parser.exit(2, "IMU yaw retake incomplete: %s\n" % error)
    print(json.dumps({**result, "output": str(Path(args.output).expanduser().resolve())},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
