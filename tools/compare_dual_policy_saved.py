"""Compare fixed h=0/1 on one saved dual-policy-once capture, without I/O to devices.

Run from SingularityDog with ``PYTHONPATH=runtime python3 -B
tools/compare_dual_policy_saved.py --capture CAPTURE --calibration CANDIDATE
--imu-mount-candidate MOUNT --bundle BUNDLE``. The original h=0 result must
reproduce, with only a bounded CPU-dependent actor float32 difference, before
an h=1 sensitivity result is reported. This command
only reads files and prints JSON; it has no motor command or capture path.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

from singularitydog_hw import dual_policy_once as once
from singularitydog_hw import policy_shadow as shadow


CAPTURE_FILES = ("dual-report.json", "events-front.json", "events-rear.json",
                 "events-imu.json")
FALSE_FLAGS = ("output_allowed", "motor_output_available", "approved_for_runtime",
               "learned_target_sent", "live_50hz_verified", "calibration_verified",
               "h_measured")
TICK_FALSE_FLAGS = tuple(key for key in FALSE_FLAGS if key != "learned_target_sent")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    path = Path(path)
    require(path.is_file(), "Missing saved JSON: " + str(path))
    raw = path.read_bytes()
    return shadow._json(raw.decode("utf-8")), hashlib.sha256(raw).hexdigest()


def require_false_flags(value, flags, label):
    require(type(value) is dict and all(value.get(key) is False for key in flags),
            label + " does not retain no-output/unapproved flags")


def load_saved(capture, calibration_path, mount_path):
    """Read original files only after checking capture and candidate digests."""
    capture = Path(capture)
    summary, summary_hash = read_json(capture / "summary.json")
    require(summary.get("status") == "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT",
            "Require the completed original single-inference report")
    require_false_flags(summary, FALSE_FLAGS, "Original summary")
    require(summary.get("motor_power_epoch_attested", False) is False,
            "This comparison is limited to an un-attested capture")
    require(summary.get("failure") is None, "Original report contains a failure")
    observation = summary.get("observation")
    require_false_flags(observation, FALSE_FLAGS, "Original observation")
    require(observation.get("status") == "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT"
            and observation.get("inference_calls_on_captured_input") == 1
            and observation.get("motor_power_epoch_attested") is False,
            "Require one original un-attested inference")
    original_tick = observation.get("observer_tick")
    require_false_flags(original_tick, TICK_FALSE_FLAGS, "Original observer tick")
    require(original_tick.get("h_hypothesis") == 0.0
            and original_tick.get("status") == "TICK_OBSERVED_NO_OUTPUT",
            "Require the saved h=0 baseline tick")
    require(type(summary.get("boot_id")) is str
            and observation.get("boot_id") == summary["boot_id"]
            and observation.get("motor_power_epoch_label") ==
                summary.get("plan", {}).get("motor_power_epoch_label"),
            "Original boot or motor-power label differs")

    capture_hashes = summary.get("capture_file_sha256")
    require(type(capture_hashes) is dict and set(capture_hashes) == set(CAPTURE_FILES),
            "Missing exact original capture file digests")
    loaded = {}
    for name in CAPTURE_FILES:
        loaded[name], actual_hash = read_json(capture / name)
        require(actual_hash == capture_hashes[name], name + " differs from original capture")
    calibration, calibration_hash = read_json(calibration_path)
    mount, mount_hash = read_json(mount_path)
    input_hashes = summary.get("input_sha256")
    require(type(input_hashes) is dict
            and input_hashes.get("calibration") == calibration_hash
            and input_hashes.get("imu_mount_candidate") == mount_hash,
            "Candidate file differs from original inference")
    require_false_flags(calibration, ("approved_for_runtime", "calibration_verified",
                                       "output_allowed", "motor_output_available"),
                        "Calibration candidate")
    rows = shadow.validate_calibration(calibration)
    require(all(row.get("approved_for_runtime") is False
                and row.get("physical_angle_accuracy_verified") is False
                for row in rows.values()), "Candidate row claims runtime/physical approval")
    shadow.validate_imu_mount_candidate(mount)
    return summary, loaded, calibration, mount, {
        "summary_sha256": summary_hash, "capture_file_sha256": capture_hashes,
        "calibration_sha256": calibration_hash,
        "imu_mount_candidate_sha256": mount_hash}


def compare_saved(summary, loaded, calibration, mount, policy_factory, torch_module,
                  *, model_sha256):
    """Replay the original validated Type0/17+IMU streams with fresh policies."""
    require(model_sha256 == summary.get("model_source", {}).get("sha256"),
            "Policy bundle differs from original inference")
    original = summary["observation"]
    args = (loaded["dual-report.json"],
            {"front": loaded["events-front.json"],
             "rear": loaded["events-rear.json"]},
            loaded["events-imu.json"], calibration, mount)
    policies = [policy_factory(), policy_factory()]
    require(policies[0] is not policies[1], "Require independent policy instances")
    results = [once.observe_one(*args, policy, torch_module, h_hypothesis=h,
                                boot_id=summary["boot_id"],
                                motor_power_epoch=original["motor_power_epoch_label"])
               for h, policy in enumerate(policies)]
    require(results[0]["snapshot"] == original["snapshot"],
            "Saved snapshot did not reproduce from original source streams")
    replay_tick, saved_tick = results[0]["observer_tick"], original["observer_tick"]
    require(type(replay_tick) is dict and type(saved_tick) is dict
            and set(replay_tick) == set(saved_tick),
            "Saved h=0 observation did not reproduce its field set")
    # The same TorchScript CPU model on ARM and x86 can differ by a few float32
    # ULPs in actor_residual12. Everything else, including the model targets,
    # source timestamps, and all 74 observation values, must match exactly.
    actor_name = "actor_residual12"
    replay_actor, saved_actor = replay_tick.get(actor_name), saved_tick.get(actor_name)
    require(type(replay_actor) is list and type(saved_actor) is list
            and len(replay_actor) == len(saved_actor) == 12
            and all(type(x) in (int, float) and math.isfinite(x)
                    for x in replay_actor + saved_actor),
            "Saved h=0 actor residual is invalid")
    actor_max_abs_difference = max(abs(a-b) for a, b in zip(replay_actor, saved_actor))
    require({k: v for k, v in replay_tick.items() if k != actor_name}
            == {k: v for k, v in saved_tick.items() if k != actor_name}
            and actor_max_abs_difference <= 1e-6
            and results[0]["observer_summary"] == original["observer_summary"],
            "Saved h=0 observation did not reproduce within float32 actor tolerance")
    for h, result in enumerate(results):
        require_false_flags(result, FALSE_FLAGS, "Replayed observation")
        require_false_flags(result["observer_tick"], TICK_FALSE_FLAGS, "Replayed tick")
        require(result["observer_tick"]["h_hypothesis"] == float(h)
                and result["observer_summary"]["status"] ==
                    "COMPLETE_NO_OUTPUT_DIAGNOSTIC",
                "Replayed hypothesis did not complete exactly one diagnostic tick")
    ticks = [result["observer_tick"] for result in results]
    q = ticks[0]["inputs"]["q_model_rad"]
    t0, t1 = (tick["q_target_rad_diagnostic_only"] for tick in ticks)
    require(ticks[0]["tick_ns"] == ticks[1]["tick_ns"]
            and results[0]["snapshot"] == results[1]["snapshot"]
            and all(ticks[0]["inputs"][key] == ticks[1]["inputs"][key]
                    for key in ("gyro_body_rad_s", "gravity_body_unit", "command",
                                "q_model_rad", "dq_model_rad_s"))
            and ticks[0]["inputs"]["h_hypothesis12"] == [0.]*12
            and ticks[1]["inputs"]["h_hypothesis12"] == [1.]*12,
            "Hypotheses did not use identical recorded inputs apart from fixed h")
    rows = [{"motor_id": mid, "q_model_rad": q[i], "h0_target_rad": t0[i],
             "h1_target_rad": t1[i], "h1_minus_h0_rad": t1[i]-t0[i]}
            for i, mid in enumerate(shadow.CAN_ORDER)]
    max_difference = max(abs(row["h1_minus_h0_rad"]) for row in rows)
    return {"status": "OFFLINE_DUAL_H_SENSITIVITY_NO_OUTPUT",
            **{key: False for key in FALSE_FLAGS},
            "hardware_opened": False, "fresh_hardware_access": False,
            "motor_power_epoch_attested": False,
            "physical_angle_accuracy_verified": False,
            "sensor_alignment_verified": False,
            "h_is_fixed_load_history_hypothesis": True,
            "source_capture_status": summary["status"],
            "boot_id": summary["boot_id"],
            "capture_tick_ns": ticks[0]["tick_ns"],
            "source_timestamps_changed": False,
            "raw_angles_modified": False,
            "clipping_applied": False,
            "angle_wrapping_applied": False,
            "h0_exact_reproduction": replay_tick == saved_tick,
            "h0_non_actor_fields_exact": True,
            "h0_actor_max_abs_difference": actor_max_abs_difference,
            "h0_actor_abs_tolerance": 1e-6,
            "saved_input_policy_calls_per_hypothesis": 1,
            "synthetic_warmup_calls_per_hypothesis": 3,
            "max_h0_target_current_gap_rad": max(abs(a-b) for a, b in zip(t0, q)),
            "max_h1_target_current_gap_rad": max(abs(a-b) for a, b in zip(t1, q)),
            "max_h1_minus_h0_target_abs_rad": max_difference,
            "max_h1_minus_h0_target_abs_deg": math.degrees(max_difference),
            "rows_model_can_order": rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--imu-mount-candidate", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        summary, loaded, calibration, mount, hashes = load_saved(
            args.capture, args.calibration, args.imu_mount_candidate)
        policy0, source0 = shadow.load_policy(args.bundle)
        import torch
        policy1, source1 = shadow.load_policy(args.bundle)
        require(source0["sha256"] == source1["sha256"],
                "Independent policy loads differ")
        policies = iter((policy0, policy1))
        result = compare_saved(summary, loaded, calibration, mount,
                               lambda: next(policies), torch,
                               model_sha256=source0["sha256"])
        result["sources_sha256"] = hashes
        result["model_sha256"] = source0["sha256"]
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0
    except (OSError, UnicodeError, ValueError, TypeError, KeyError, RuntimeError,
            ImportError, OverflowError, StopIteration) as error:
        print(json.dumps({"status": "OFFLINE_DUAL_H_SENSITIVITY_BLOCKED",
                          "reason": type(error).__name__ + ": " + str(error),
                          **{key: False for key in FALSE_FLAGS},
                          "hardware_opened": False}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
