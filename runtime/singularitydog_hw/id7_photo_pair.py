"""Pair two operator photographs with read-only RS05 ID7 position snapshots.

The only CAN requests are Type0 identity and Type17 position (0x7019) reads.
This tool cannot enable, disable, zero, calibrate, or move a motor. A photograph
is taken with an external camera; its optional filename and acknowledgement time
are recorded so the physical marker angle can be measured separately.
"""

import argparse
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import time

from .can_readonly import ReadOnlyCAN


REAR_PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0"
REAR_IDS = (7, 8, 9)
POSES = ("A", "B")
SAMPLES_PER_POSE = 3
SAMPLE_GAP_S = 0.05


def validate_expected_uids(value):
    if not isinstance(value, dict) or set(value) != {str(i) for i in range(1, 13)}:
        raise ValueError("Expected UID file must contain IDs 1..12")
    if any(not isinstance(uid, str) or len(uid) != 16 or
           any(ch not in "0123456789abcdef" for ch in uid)
           for uid in value.values()):
        raise ValueError("Invalid expected UID")
    if len(set(value.values())) != 12:
        raise ValueError("Duplicate expected UID")
    return value


def validate_port(port):
    if port != REAR_PORT:
        raise ValueError(f"ID7 requires verified rear USB2CAN path: {REAR_PORT}")
    return port


def read_boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def checked_query(can, motor_id, parameter, check):
    check()
    reply = can.query(motor_id, parameter)
    if can.parser.discarded_bytes or can.parser.buffer:
        raise RuntimeError("Corrupt or incomplete CAN data")
    if not reply.get("ok"):
        raise RuntimeError(f"ID{motor_id} {parameter or 'identity'} read failed")
    if parameter == "position" and (
            type(reply.get("value")) not in (float, int) or
            not math.isfinite(reply["value"])):
        raise RuntimeError("Invalid ID7 position")
    check()
    return reply


def capture_pose(can, pose, expected_uids, emit, check, *, sleep=time.sleep):
    """Verify rear identities and sample raw position; never infer output scale."""
    if pose not in POSES:
        raise ValueError("Pose must be A or B")
    uids = {}
    for motor_id in REAR_IDS:
        reply = checked_query(can, motor_id, None, check)
        uid = reply.get("mcu_uid_hex")
        if uid != expected_uids[str(motor_id)]:
            raise RuntimeError(f"ID{motor_id} UID mismatch")
        uids[str(motor_id)] = uid
        emit({"kind": "photo_pair_uid", "pose": pose,
              "motor_id": motor_id, "mcu_uid_hex": uid,
              "monotonic_ns": reply.get("monotonic_ns")})
    readings = []
    for index in range(SAMPLES_PER_POSE):
        reply = checked_query(can, 7, "position", check)
        reading = {"kind": "photo_pair_position", "pose": pose,
                   "sample": index, "monotonic_ns": reply["monotonic_ns"],
                   "position_rad": reply["value"],
                   "raw_value_hex": reply["raw_value_hex"]}
        emit(reading)
        readings.append(reading)
        if index + 1 < SAMPLES_PER_POSE:
            sleep(SAMPLE_GAP_S)
    if any(b["monotonic_ns"] <= a["monotonic_ns"]
           for a, b in zip(readings, readings[1:])):
        raise RuntimeError("Position timestamps did not increase")
    values = [r["position_rad"] for r in readings]
    return {"verified_uids": uids, "samples": readings,
            "median_position_rad": statistics.median(values),
            "position_range_deg": math.degrees(max(values) - min(values)),
            "stationarity_certified": False}


def photo_ack(pose, check, *, ask=input, clock_ns=time.monotonic_ns,
              wall_ns=time.time_ns):
    """The acknowledgement follows the photo; the robot pose stays unchanged."""
    while True:
        check()
        photo_ref = ask(
            f"\n姿勢{pose}：回転軸を正面から見て、黒いQDD本体と黄色い出力部の目印を同じ画面で撮影。"
            "姿勢を保ったまま写真名を入力（不明ならEnter、qで中止）: ").strip()
        check()
        if photo_ref.lower() == "q":
            raise InterruptedError("Operator cancelled photo pairing")
        if len(photo_ref) <= 128 and all(ord(ch) >= 32 for ch in photo_ref):
            return {"pose": pose, "photo_reference": photo_ref or None,
                    "ack_wall_time_ns": wall_ns(),
                    "ack_monotonic_ns": clock_ns()}
        print("写真名は128文字以内で入力してください。", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--port", default=REAR_PORT)
    parser.add_argument("--expected-uids", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        port = validate_port(args.port)
        expected_bytes = args.expected_uids.expanduser().read_bytes()
        expected = validate_expected_uids(json.loads(expected_bytes))
    except (ValueError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
    plan = {"motor_id": 7, "verified_ids": list(REAR_IDS), "poses": list(POSES),
            "position_samples_per_pose": SAMPLES_PER_POSE, "port": port,
            "allowed_can_types": [0, 17], "position_parameter": "0x7019",
            "motor_output_available": False, "calibration_applied": False}
    if not args.execute_readonly:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    if not sys.stdin.isatty():
        parser.error("Run in an interactive Jetson terminal; no piped confirmation")
    output = args.output.expanduser().resolve()
    if any((candidate / ".git").exists() for candidate in (output, *output.parents)):
        parser.error("Private photo pairing data must be outside Git")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    boot_id = read_boot_id()
    result = {"started_at": datetime.datetime.now().astimezone().isoformat(),
              "boot_id": boot_id, "plan": plan,
              "expected_uids_sha256": hashlib.sha256(expected_bytes).hexdigest(),
              "source_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                  for name, path in (("id7_photo_pair.py", Path(__file__)),
                                     ("can_readonly.py", Path(__file__).with_name("can_readonly.py")))},
              "poses": {}, "status": "INCOMPLETE", "approved_for_runtime": False,
              "calibration_applied": False, "errors": []}
    interrupted = []
    old_handlers = {}

    def check():
        if interrupted:
            raise InterruptedError("Operator interrupted photo pairing")
        if read_boot_id() != boot_id:
            raise RuntimeError("Jetson boot changed during photo pairing")

    def stop(signum, _frame):
        interrupted.append(signum)
        raise InterruptedError("Operator interrupted photo pairing")

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, stop)
        lock_dir = Path.home() / ".cache" / "singularitydog"
        lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        with (lock_dir / "manual-calibration.lock").open("a+") as motion_lock, \
             (lock_dir / "can-readonly.lock").open("a+") as can_lock, \
             (output / "events.jsonl").open("x", buffering=1) as events:
            fcntl.flock(motion_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(can_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            def emit(event):
                events.write(json.dumps({"wall_time_ns": time.time_ns(), **event},
                                        allow_nan=False) + "\n")
                if event.get("kind") == "motor_feedback" and (
                        event.get("type") == 21 or event.get("fault_bits", 0) or
                        event.get("mode_state", 0)):
                    raise RuntimeError("Enabled or fault feedback during read-only capture")

            emit({"kind": "photo_pair_plan", **plan})
            print("写真A/Bを回転軸の正面、同じ向き・距離から撮影してください。QDDは脱力し、機体を支持台へ載せます。", flush=True)
            print("各写真の撮影後もその位置を保ち、EnterするとUIDと角度を3回読みます。自動駆動はありません。", flush=True)
            with ReadOnlyCAN(port=port, event_sink=emit) as can:
                for pose in POSES:
                    check()
                    ack = photo_ack(pose, check)
                    emit({"kind": "photo_pair_ack", **ack})
                    snapshot = capture_pose(can, pose, expected, emit, check)
                    result["poses"][pose] = {**ack, **snapshot}
                    (output / "summary.json").write_text(json.dumps(
                        result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
                    print(f"姿勢{pose}を保存：生角度中央値 "
                          f"{math.degrees(snapshot['median_position_rad']):.3f}°。"
                          "次の姿勢へ移動してから撮影してください。", flush=True)
            a = result["poses"]["A"]["median_position_rad"]
            b = result["poses"]["B"]["median_position_rad"]
            result["raw_position_change_deg"] = math.degrees(b - a)
            result["interpretation"] = (
                "Raw position change only. Compare paired photos independently; "
                "no automatic gear-ratio correction or runtime approval.")
            result["status"] = "RECORDED_REVIEW_REQUIRED"
    except BaseException as error:
        result["errors"].append(repr(error))
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        result["completed_at"] = datetime.datetime.now().astimezone().isoformat()
        (output / "summary.json").write_text(json.dumps(
            result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(f"保存先：{output}\n結果：{result['status']}。実角度との照合が必要です。", flush=True)
    if result["errors"]:
        print(" / ".join(result["errors"]), flush=True)
    return 0 if result["status"] == "RECORDED_REVIEW_REQUIRED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
