"""One guided, IMU-only session; never opens CAN, enables a motor, or applies bias.

Default is a plan. --execute requires QDD power off and physical body support.
Each acquisition delegates to the existing audited imu_capture subprocess and
waits for restoration. Prompt times are recorded against the first real sample;
late/missed prompts invalidate a movement rather than silently moving windows.
The yaw-only option records fresh A/B and the missing yaw direction; old pitch
and roll evidence remains separate and is never promoted by this short session.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from . import imu_commissioning_audit as audit
from . import imu_fixed_mount_baseline as baseline

DIRECTIONS = {
    "nose_up": "鼻先を上へ少し傾ける（機体Yの負方向）",
    "left_side_up": "犬の左側を少し持ち上げる（機体Xの正方向）",
    "turn_left": "水平に近いまま鼻先を犬の左へ少し向ける（機体Zの正方向）",
}
PROMPTS = ((3., "outbound_start", "動作開始：ゆっくり5〜15°だけ指定の方向へ動かしてください。"),
           (7., "outbound_end", "その姿勢を保ってください。"),
           (10., "return_start", "戻してください：ゆっくり元の姿勢へ戻します。"),
           (14., "return_end", "戻す動作は終了です。支えたまま静止してください。"))


def _write_new(path, obj):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(obj, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def _first_sample(path):
    if not path.exists():
        return None
    # The existing recorder flushes complete JSONL lines. Partial tail is ignored.
    with path.open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                break
            if row.get("kind") == "imu":
                return row["monotonic_ns"]
    return None


def movement_windows(prompt_times, first_sample_ns):
    if set(prompt_times) != {key for _, key, _ in PROMPTS}:
        raise RuntimeError("Not all motion prompts were delivered; motion cannot be labelled")
    seconds = {key: (value-first_sample_ns)/1e9 for key, value in prompt_times.items()}
    for desired, key, _ in PROMPTS:
        if not desired <= seconds[key] <= desired+.25:
            raise RuntimeError("Late/missed motion prompt: " + key)
    return {"outbound_s": [seconds["outbound_start"], seconds["outbound_end"]],
            "return_s": [seconds["return_start"], seconds["return_end"]]}


def capture_one(output, *, movement=None):
    """Child owns I2C/lock/restoration; parent owns prompts and timeout only."""
    command = [sys.executable, "-m", "singularitydog_hw.imu_capture", "--execute",
               "--face", "unverified", "--settle-seconds", "3", "--seconds", "20",
               "--output", str(output)]
    transcript = output.parent/(output.name+"-capture.stdout")
    process = None
    prompts, first = {}, None
    started = time.monotonic()
    # File avoids stdout pipe filling/deadlock while the parent is giving cues.
    with transcript.open("x") as stream:
        try:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            while process.poll() is None:
                if time.monotonic()-started > 40:
                    raise TimeoutError("IMU capture exceeded 40 seconds")
                if movement and first is None:
                    first = _first_sample(output/"events.jsonl")
                    if first is not None:
                        print("収録開始。最初は静止したまま待ってください。", flush=True)
                if movement and first is not None:
                    elapsed = (time.monotonic_ns()-first)/1e9
                    for target, name, text in PROMPTS:
                        if name not in prompts and elapsed >= target:
                            # A stalled controller must not fire multiple stale cues.
                            if elapsed > target+.25:
                                raise RuntimeError("Motion cue missed its deadline: " + name)
                            prompts[name] = time.monotonic_ns()
                            print(text, flush=True)
                time.sleep(.01)
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
    if movement:
        windows = movement_windows(prompts, rows[0]["monotonic_ns"])
        return {"movement": movement, "capture": output.name, **windows,
                "operator_direction_confirmed": False, "cue_monotonic_ns": prompts}
    return None


def _enter(message):
    value = input(message + " [Enterで開始 / qで中止]: ").strip().lower()
    if value != "":
        raise InterruptedError("operator cancelled session")


def run_session(output, *, static_only=False, movements=None):
    if static_only and movements is not None:
        raise ValueError("static_only and selected movements cannot be combined")
    selected = () if static_only else tuple(DIRECTIONS) if movements is None else tuple(movements)
    if len(set(selected)) != len(selected) or any(name not in DIRECTIONS for name in selected):
        raise ValueError("selected movements must be distinct known directions")
    output = Path(output).expanduser().resolve()
    if any((p/".git").exists() for p in (output, *output.parents)):
        raise ValueError("capture output must be outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    manifest = {"schema_version": 1, "mount_candidate": audit.mount_candidate(),
                "stationary": {"a": "static-a", "b": "static-b", "operator_confirmed": False},
                "movements": [], "hardware_scope": "IMU only; no CAN",
                "operator_assertions": {"qdd_power_off": True, "body_supported": True},
                "runtime_approval": False}
    try:
        print("QDDの40VはOffのまま。機体を支え、IMUの取付を変えません。", flush=True)
        for name in ("static-a", "static-b"):
            _enter(name + "：機体を静止させ、約23秒触れずに待てる状態にしてください")
            capture_one(output/name)
        answer = input("A/Bの両収録中、機体は静止していましたか [y/n]: ").strip().lower()
        manifest["stationary"]["operator_confirmed"] = answer == "y"
        if selected:
            for name in selected:
                description = DIRECTIONS[name]
                print("次の方向：" + description + "。元の姿勢へ戻すところまで約23秒です。", flush=True)
                _enter("抵抗・接触がなく、支えたまま小さく動かせる状態にしてください")
                item = capture_one(output/name, movement=name)
                item["operator_direction_confirmed"] = input(
                    "合図に合わせ、指定方向へ動かして戻しましたか [y/n]: ").strip().lower() == "y"
                manifest["movements"].append(item)
        _write_new(output/"manifest.json", manifest)
        result = audit.write_audit(output/"manifest.json", output/"audit.json")
        _write_new(output/"imu-mount-candidate.json", result["mount_candidate"])
        if result["checks"]["gyro_bias_candidate"]:
            # Existing no-output PolicyObserver accepts this schema unchanged.
            _write_new(output/"gyro-bias-candidate.json", result["stationary"])
        return result
    except BaseException as error:
        _write_new(output/"session-incomplete.json", {"error": repr(error), "manifest_so_far": manifest,
                                                     "status": "INCOMPLETE", "approved_for_runtime": False})
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="new private directory outside Git")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--motor-power-off", action="store_true")
    parser.add_argument("--body-supported", action="store_true")
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--yaw-only", action="store_true",
                        help="capture stationary A/B and left turn only; existing pitch/roll evidence needs separate review")
    args = parser.parse_args(argv)
    if args.static_only and args.yaw_only:
        parser.error("--static-only and --yaw-only cannot be combined")
    movements = () if args.static_only else ("turn_left",) if args.yaw_only else tuple(DIRECTIONS)
    if not args.execute:
        print(json.dumps({"mode": "PLAN_ONLY", "CAN_opened": False,
                          "captures": ["static-a", "static-b", *movements],
                          "movement_directions_not_measured": [name for name in DIRECTIONS if name not in movements],
                          "minimum_recording_seconds": 46 + 23*len(movements),
                          "operator_positioning_time_additional": True, "runtime_approval": False}))
        return 0
    if not args.motor_power_off or not args.body_supported:
        parser.error("IMU motion session requires --motor-power-off and --body-supported")
    try:
        result = run_session(args.output, static_only=args.static_only,
                             movements=None if args.static_only else movements)
    except (OSError, ValueError, RuntimeError, InterruptedError, KeyboardInterrupt) as error:
        parser.exit(2, "IMU session incomplete: %s\n" % error)
    print(json.dumps({"status": result["status"], "checks": result["checks"],
                      "output": str(Path(args.output).expanduser().resolve()), "approved_for_runtime": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
