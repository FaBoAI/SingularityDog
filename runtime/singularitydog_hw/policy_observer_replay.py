"""Offline stateful replay of complete, wire-validated diagnose captures.

Source timestamps stay unchanged. A 20ms diagnostic grid starts when every
available source and all twelve identities have first completed. Missing sources
block the first tick; unavailable future values never enter the buffer. This
file has no device/network/output-to-motor path and makes no live50Hz claim.
"""
import argparse
import json
import os
from pathlib import Path

from . import policy_shadow as shadow
from . import policy_observer as observer
from . import telemetry_snapshot as telemetry


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _stamp(value):
    _require(type(value) is int and 0 < value < 2**63, "Invalid original monotonic timestamp")
    return value


def _ingest(buffer, item):
    if item["source"] == "imu":
        return buffer.ingest_imu(**item["values"])
    return buffer.ingest_motor(**item["values"])


def translate_records(records, calibration):
    """Translate already wire-validated events; never infer source timestamps.

    The CLI MUST call shadow.load_capture first. This helper checks identities,
    SI fields and per-source chronology, but does not itself decode CAN wire.
    """
    shadow.validate_calibration(calibration)
    identities, items, first = {}, [], {}
    checker = telemetry.TelemetrySnapshotBuffer(history_per_key=1)
    for index, row in enumerate(records):
        _require(isinstance(row, dict), "Capture event must be an object")
        if row.get("kind") == "motor_parameter":
            mid, parameter = row.get("motor_id"), row.get("parameter")
            _require(type(mid) is int and 1 <= mid <= 12, "Invalid motor source")
            if parameter == "identity":
                _require(mid not in identities, "Duplicate capture identity")
                _require(row.get("ok") is True and row.get("mcu_uid_hex") ==
                         calibration["identities"][str(mid)], "Capture/calibration identity mismatch")
                identities[mid] = _stamp(row.get("monotonic_ns"))
                continue
            if parameter not in ("position", "velocity"):
                continue
            start, finish = _stamp(row.get("request_monotonic_ns")), _stamp(row.get("monotonic_ns"))
            _require(mid in identities and identities[mid] <= start,
                     "Motor telemetry precedes its validated identity")
            _require(row.get("ok") is True and type(row.get("status")) is int and row["status"] == 0,
                     "Require successful position/velocity read")
            _require(row.get("unit") == shadow.PARAMETERS[parameter][2], "Invalid original motor SI unit")
            values = dict(can_type=17, motor_id=mid, parameter=parameter, value=row.get("value"),
                          unit=row["unit"], request_ns=start, received_ns=finish)
            source, key = "motor", (mid, parameter)
        elif row.get("kind") == "imu":
            _require(row.get("frame") == "sensor", "Require original sensor-frame IMU")
            for flag in ("calibration_applied", "orientation_applied", "mount_rotation_applied",
                         "gyro_bias_subtracted", "accel_bias_subtracted", "accel_scale_corrected"):
                _require(flag not in row or row[flag] is False, "Already corrected IMU: " + flag)
            start, midpoint, finish = (_stamp(row.get(k)) for k in (
                "read_started_monotonic_ns", "monotonic_ns", "read_finished_monotonic_ns"))
            _require(start <= midpoint <= finish, "Invalid original IMU read interval")
            values = dict(accel_m_s2=row.get("accel_m_s2"), gyro_rad_s=row.get("gyro_rad_s"),
                          read_started_ns=start, read_finished_ns=finish)
            source, key = "imu", "imu"
        else:
            continue
        item = {"source": source, "available_ns": finish, "source_record_index": index, "values": values}
        _ingest(checker, item)  # Validate original per-key order before sorting across threads.
        first.setdefault(key, finish)
        items.append(item)
    _require(set(identities) == set(range(1, 13)), "Require all twelve matching capture identities")
    _require(items, "No replay telemetry")
    items.sort(key=lambda x: (x["available_ns"], x["source_record_index"]))
    first_tick = max(*first.values(), *identities.values())
    return items, {"first_tick_ns": first_tick, "last_available_ns": items[-1]["available_ns"],
                   "grid_step_ns": observer.DT_NS, "source_count": len(items),
                   "first_source_completion_ns": min(first.values()),
                   "initialization_wait_ns": first_tick-min(first.values()),
                   "missing_sources": [str(k) for k in (*telemetry.MOTOR_KEYS, "imu") if k not in first],
                   "capture_identity_match_verified": True, "fresh_live_identity_match_verified": False,
                   "grid_origin": "latest first completion of available required sources and twelve identities",
                   "source_timestamps_changed": False}


def warmup_policy(policy, torch, h, count):
    """Synthetic shape/state warmup only; never substitute these for telemetry."""
    _require(type(count) is int and 1 <= count <= 100, "warmup_ticks must be 1..100")
    inputs = ([0., 0., 0.], [0., 0., -1.], [0., 0., 0.],
              [0., .4, -.8]*4, [0.]*12, [float(h)]*12)
    with torch.inference_mode():
        for _ in range(count):
            output = policy(*(torch.tensor([x], dtype=torch.float32) for x in inputs))
            observer._tensor_row(output, 12, "warmup target")
            observer._tensor_row(policy.last_actor_output, 12, "warmup actor")
            observer._tensor_row(policy.last_observation, 74, "warmup observation")


def replay_records(records, calibration, policies, *, imu_mount_candidate,
                   max_ticks, max_age_ns, max_spread_ns, torch_module,
                   gyro_bias_candidate=None, command=(0., 0., 0.), warmup_ticks=3,
                   emit=None):
    """Replay two independent policy instances; return summary, emit JSON rows.

    Inputs must already have passed wire-level capture validation. Synthetic
    tests may use this helper directly. Stops each hypothesis at its first
    blocked/model-failure tick, without resetting or catching up that run.
    """
    _require(isinstance(policies, (list, tuple)) and len(policies) == 2
             and policies[0] is not policies[1], "Require independent h=0/h=1 policy instances")
    items, timeline = translate_records(records, calibration)
    runs = [observer.StatefulPolicyObserver(
        policy, calibration, imu_mount_candidate=imu_mount_candidate, h_hypothesis=h,
        command=list(command), max_ticks=max_ticks, max_age_ns=max_age_ns, max_spread_ns=max_spread_ns,
        torch_module=torch_module, gyro_bias_candidate=gyro_bias_candidate)
        for h, policy in enumerate(policies)]
    emit = emit or (lambda row: None)
    results = []
    for h, (policy, run) in enumerate(zip(policies, runs)):
        buffer = telemetry.TelemetrySnapshotBuffer(history_per_key=2, max_age_ns=max_age_ns,
                                                    max_spread_ns=max_spread_ns)
        failure, cursor = None, 0
        try:
            warmup_policy(policy, torch_module, h, warmup_ticks)
            run.reset_run(timeline["first_tick_ns"], warmup_completed=True)
        except Exception as error:
            run.invalidate("Warmup/reset failed: " + type(error).__name__ + ": " + str(error))
            failure = {"phase": "warmup_or_reset", "tick_ns": None, "tick_index": None,
                       "reason": run.failure}
        if failure is None:
            for index in range(max_ticks):
                tick = timeline["first_tick_ns"] + index*observer.DT_NS
                while cursor < len(items) and items[cursor]["available_ns"] <= tick:
                    _ingest(buffer, items[cursor])
                    cursor += 1
                snapshot = buffer.snapshot(tick).as_dict()
                snapshot["source_flags"] = {
                    "capture_identity_match_verified": True, "fresh_identity_match_verified": False,
                    "source": "complete diagnose capture; availability by original read completion",
                    "original_timestamps_preserved": True, "live_50hz_verified": False}
                reason = ("capture_exhausted" if tick > timeline["last_available_ns"] else
                          "; ".join(snapshot["blocked_reasons"]))
                if reason:
                    run.invalidate(reason)
                else:
                    try:
                        tick_result = run.consume(snapshot)
                    except Exception as error:
                        reason = type(error).__name__ + ": " + str(error)
                    else:
                        emit({"kind": "policy_observer_tick", **tick_result})
                if reason:
                    failure = {"phase": "replay", "tick_ns": tick, "tick_index": index,
                               "reason": reason, "snapshot": snapshot}
                    emit({"kind": "policy_observer_blocked", "h_hypothesis": h,
                          "output_allowed": False, "approved_for_runtime": False, **failure})
                    break
        results.append({**run.finish(), "first_blocked_tick": failure,
                        "warmup_ticks_requested": warmup_ticks,
                        "warmup_source": "explicit synthetic inputs; not captured telemetry"})
    complete = all(r["status"] == "COMPLETE_NO_OUTPUT_DIAGNOSTIC" for r in results)
    return {"schema_version": 1, "kind": "policy_observer_replay",
            "status": "COMPLETE_NO_OUTPUT_DIAGNOSTIC" if complete else "INCOMPLETE",
            "motor_output_available": False, "output_allowed": False, "hardware_opened": False,
            "approved_for_runtime": False, "live_50hz_verified": False,
            "calibration_verified": False, "incomplete_capture_override_available": False,
            "timeline": timeline, "max_age_ns": max_age_ns, "max_spread_ns": max_spread_ns,
            "limits_are_diagnostic_only": True, "command": list(command), "hypotheses": results,
            "limitations": ["A 20ms offline grid does not establish live execution at50Hz.",
                "Host request/read completion intervals do not establish sensor conversion simultaneity.",
                "Snapshot values may be held only within explicit age/spread limits; never zero-filled or clipped.",
                "Mount, gyro bias, calibration and h remain unapproved diagnostic hypotheses.",
                "Source coverage ends at the last recorded relevant read completion; no extrapolated tail."]}


def _output_path(path):
    original = Path(path).expanduser()
    _require(not original.exists() and not original.is_symlink(), "Output directory must be new")
    output = original.resolve()
    _require(not any((p/".git").exists() or (p/".git").is_symlink()
                     for p in (output, *output.parents)), "Output must be outside Git")
    return output


def _milliseconds(text):
    # Reject limits that would require rounding to integer nanoseconds.
    from decimal import Decimal, DecimalException
    try:
        ns = Decimal(text)*1_000_000
        if not ns.is_finite() or ns < 0 or ns >= 2**63 or ns != ns.to_integral_value():
            raise ValueError
        return int(ns)
    except (DecimalException, ValueError):
        raise argparse.ArgumentTypeError("Use nonnegative finite milliseconds with integer-nanosecond precision")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "calibration", "bundle", "imu-mount-candidate", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--gyro-bias-candidate", type=Path)
    parser.add_argument("--max-age-ms", type=_milliseconds, required=True)
    parser.add_argument("--max-spread-ms", type=_milliseconds, required=True)
    parser.add_argument("--max-ticks", type=int, required=True)
    parser.add_argument("--warmup-ticks", type=int, default=3)
    parser.add_argument("--command", type=float, nargs=3, default=[0., 0., 0.], metavar=("VX", "VY", "YAW"))
    args = parser.parse_args(argv)
    output_created, phase, capture_validated = False, "output", False
    capture_hashes = {}
    try:
        output = _output_path(args.output)
        output.mkdir(mode=0o700, exist_ok=False)
        output_created = True
        phase = "capture_validation"
        capture_hashes = {name: shadow.sha(args.capture/name) for name in ("summary.json", "events.jsonl")}
        records, capture_source = shadow.load_capture(args.capture)  # Complete-only; no override exists.
        _require(capture_hashes == {name: shadow.sha(args.capture/name) for name in capture_hashes}
                 and capture_hashes["summary.json"] == capture_source["summary_sha256"]
                 and capture_hashes["events.jsonl"] == capture_source["events_sha256"], "Capture changed during load")
        capture_validated = True
        phase = "candidate_and_timeline_validation"
        calibration_bytes = args.calibration.read_bytes()
        mount_bytes = args.imu_mount_candidate.read_bytes()
        calibration = shadow._json(calibration_bytes)
        mount = shadow.validate_imu_mount_candidate(shadow._json(mount_bytes))
        bias_bytes = args.gyro_bias_candidate.read_bytes() if args.gyro_bias_candidate else None
        bias = shadow._json(bias_bytes) if bias_bytes is not None else None
        translate_records(records, calibration)
        _require(1 <= args.max_ticks <= 30_000 and 1 <= args.warmup_ticks <= 100, "Invalid finite tick/warmup cap")
        phase = "model_loading"
        policies, model_sources = zip(*(shadow.load_policy(args.bundle) for _ in range(2)))
        import torch
        phase = "replay"
        with (output/"events.jsonl").open("x", encoding="utf-8") as stream:
            os.chmod(output/"events.jsonl", 0o600)
            def emit(row):
                stream.write(json.dumps(row, allow_nan=False)+"\n")
            result = replay_records(records, calibration, policies, imu_mount_candidate=mount,
                max_ticks=args.max_ticks, max_age_ns=args.max_age_ms, max_spread_ns=args.max_spread_ms,
                torch_module=torch, gyro_bias_candidate=bias, command=args.command,
                warmup_ticks=args.warmup_ticks, emit=emit)
            stream.flush()
            os.fsync(stream.fileno())
        import hashlib
        result["provenance"] = {"capture": capture_source, "models": list(model_sources),
            "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
            "imu_mount_candidate_sha256": hashlib.sha256(mount_bytes).hexdigest(),
            "gyro_bias_candidate_sha256": hashlib.sha256(bias_bytes).hexdigest() if bias_bytes is not None else None,
            "runner_sources_sha256": {Path(p).name: shadow.sha(p)
                for p in (__file__, shadow.__file__, observer.__file__, telemetry.__file__)},
            "output_events_sha256": shadow.sha(output/"events.jsonl")}
        with (output/"summary.json").open("x", encoding="utf-8") as stream:
            os.chmod(output/"summary.json", 0o600)
            stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
            stream.flush()
            os.fsync(stream.fileno())
        print(json.dumps({"output": str(output), "status": result["status"],
                          "hypotheses": [{k: r[k] for k in ("h_hypothesis", "ticks_completed", "failure")}
                                         for r in result["hypotheses"]],
                          "approved_for_runtime": False, "live_50hz_verified": False}))
        return 0 if result["status"] == "COMPLETE_NO_OUTPUT_DIAGNOSTIC" else 2
    except Exception as error:
        if output_created and not (output/"summary.json").exists():
            failure = {"schema_version": 1, "kind": "policy_observer_replay", "status": "INCOMPLETE",
                       "failure_phase": phase, "failure": type(error).__name__ + ": " + str(error),
                       "capture_validation_passed": capture_validated,
                       "ticks_completed": None if phase == "replay" else 0,
                       "tick_count_unknown_due_to_runner_failure": phase == "replay",
                       "approved_for_runtime": False, "output_allowed": False,
                       "hardware_opened": False, "motor_output_available": False,
                       "live_50hz_verified": False, "incomplete_capture_override_available": False,
                       "provenance": {"capture_file_sha256": capture_hashes,
                           "runner_sources_sha256": {Path(p).name: shadow.sha(p)
                               for p in (__file__, shadow.__file__, observer.__file__, telemetry.__file__)}}}
            if not (output/"events.jsonl").exists():
                descriptor = os.open(output/"events.jsonl", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                os.close(descriptor)
            descriptor = os.open(output/"summary.json", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(json.dumps(failure, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
                stream.flush()
                os.fsync(stream.fileno())
        parser.exit(2, "Observer replay rejected: %s\n" % error)


if __name__ == "__main__":
    raise SystemExit(main())
