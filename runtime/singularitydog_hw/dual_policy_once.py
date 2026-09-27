"""Bounded two-USB2CAN + IMU capture, then one no-output policy observation.

CAN is restricted by the existing dual pipeline to Type0 UID and Type17
position/velocity reads. The IMU reader temporarily configures its own sensor
and restores it. Inference uses a completed capture's original host timestamps;
it is deliberately post-capture and establishes no 20 ms control performance.
No RS05 enable, STOP, setting write, motion frame, or command transport exists.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import threading
import time

from . import dual_can_pipeline_benchmark as dual
from . import policy_observer as observer
from . import policy_observer_live as live
from . import policy_observer_replay as replay
from . import policy_shadow as shadow
from . import telemetry_snapshot as telemetry
from .angle_branch_comparison import StaticBranchComparison


FLAGS = {"motor_output_available": False, "output_allowed": False,
         "approved_for_runtime": False, "learned_target_sent": False,
         "live_50hz_verified": False, "calibration_verified": False,
         "h_measured": False, "can_allowed_types": [0, 17]}
MAX_SECONDS = 10.
MAX_IMU_ROWS = 1200


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def reviewed_branch_manifest(data, boot_id, motor_power_epoch, expected_uids):
    """Reconstruct the existing static review object from a saved manifest."""
    if data is None:
        return None
    _need(motor_power_epoch != "UNVERIFIED",
          "A branch manifest needs an independently observed motor power epoch")
    _need(type(data) is dict and data.get("status") == "OPERATOR_ATTESTED_REVIEW_REQUIRED"
          and data.get("motor_output_allowed") is False
          and data.get("approved_for_runtime") is False
          and data.get("boot_id") == boot_id,
          "Branch manifest is not a matching no-output boot review")
    comparison = StaticBranchComparison(data.get("reference_snapshot"),
                                        data.get("current_snapshot"),
                                        data.get("no_turn_evidence"))
    _need(comparison.comparison() == data.get("branch_comparison"),
          "Branch manifest comparison differs from its snapshots")
    binding = comparison.validated_current_binding()
    _need(binding["boot_id"] == boot_id
          and binding["motor_power_epoch"] == motor_power_epoch
          and binding["uids_by_id"] == {str(i): expected_uids[i] for i in range(1, 13)},
          "Branch manifest UID/boot/motor-power epoch binding changed")
    return comparison


def observe_one(dual_report, events_by_bus, imu_events, calibration, mount, policy,
                torch_module, *, h_hypothesis, boot_id, motor_power_epoch,
                branch_comparison=None,
                max_age_ns=100_000_000, max_spread_ns=100_000_000,
                warmup_completed=False):
    """Pure post-capture adapter for validated Type0/17 and raw IMU events."""
    _need(type(boot_id) is str and bool(boot_id.strip()), "Missing capture boot ID")
    _need(type(motor_power_epoch) is str and bool(motor_power_epoch.strip()),
          "Missing motor power epoch label; use UNVERIFIED without a manifest")
    _need(type(h_hypothesis) is int and h_hypothesis in (0, 1),
          "Only an explicit diagnostic h=0/1 hypothesis is accepted")
    _need(type(max_age_ns) is int and 0 < max_age_ns <= 100_000_000
          and type(max_spread_ns) is int and 0 < max_spread_ns <= 100_000_000,
          "Diagnostic age/spread must be at most 100 ms")
    _need(type(dual_report) is dict
          and dual_report.get("status") == "COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT"
          and dual_report.get("output_allowed") is False
          and dual_report.get("producer_threads_exited") is True
          and dual_report.get("common_lock_released") is True
          and dual_report.get("errors") == [],
          "Require completed, closed, no-output two-bus capture")
    _need(type(events_by_bus) is dict and set(events_by_bus) == {"front", "rear"}
          and all(type(events_by_bus[bus]) is list for bus in events_by_bus),
          "Require both original bus event streams")
    _need(type(imu_events) is list and 0 < len(imu_events) <= MAX_IMU_ROWS,
          "Require bounded IMU source events")
    rows = shadow.validate_calibration(calibration)
    shadow.validate_imu_mount_candidate(mount)
    _need(calibration.get("approved_for_runtime") is False
          and all(row.get("approved_for_runtime") is not True for row in rows.values()),
          "Only unapproved calibration candidates are accepted")
    expected = calibration["identities"]
    seen_uids, seen_values = {}, set()
    last_finishes = []
    buffer = telemetry.TelemetrySnapshotBuffer(history_per_key=32,
        max_age_ns=max_age_ns, max_spread_ns=max_spread_ns)
    for bus, ids in dual.SCOPES.items():
        for event in events_by_bus[bus]:
            if event.get("kind") != "pipeline_reply":
                continue
            mid, parameter, cycle = (event.get(key) for key in
                                      ("motor_id", "parameter", "cycle"))
            result = event.get("result")
            _need(type(mid) is int and mid in ids and type(result) is dict
                  and event.get("ok") is True and result.get("ok") is True,
                  "Invalid dual-bus reply or motor assignment")
            if parameter == "identity":
                _need(cycle == 0 and mid not in seen_uids
                      and result.get("mcu_uid_hex") == expected[str(mid)],
                      "Fresh identity does not match calibration")
                seen_uids[mid] = result["mcu_uid_hex"]
            elif parameter in ("position", "velocity"):
                _need(mid in seen_uids and type(cycle) is int and 1 <= cycle <= 20
                      and (cycle, mid, parameter) not in seen_values,
                      "Telemetry missing identity or repeated within a cycle")
                seen_values.add((cycle, mid, parameter))
                buffer.ingest_pipeline_reply(event)
                if cycle == 20:
                    last_finishes.append(event["received_monotonic_ns"])
            else:
                raise ValueError("Unexpected CAN parameter in no-output capture")
    _need(set(seen_uids) == set(range(1, 13)) and len(seen_values) == 20*24
          and len(last_finishes) == 24,
          "Incomplete twelve-axis identity or position/velocity capture")
    previous_sequence = 0
    for event in imu_events:
        _need(type(event) is dict and event.get("kind") == "imu"
              and event.get("frame") == "sensor"
              and type(event.get("sequence")) is int
              and event["sequence"] == previous_sequence + 1,
              "IMU source sequence/frame invalid")
        previous_sequence += 1
        buffer.ingest_imu(accel_m_s2=event.get("accel_m_s2"),
            gyro_rad_s=event.get("gyro_rad_s"),
            read_started_ns=event.get("read_started_monotonic_ns"),
            read_finished_ns=event.get("read_finished_monotonic_ns"))
    tick_ns = max(last_finishes)
    snapshot = buffer.snapshot(tick_ns).as_dict()
    _need(snapshot["status"] == "DIAGNOSTIC_READY",
          "Captured inputs are incomplete or stale: " + ",".join(snapshot["blocked_reasons"]))
    if branch_comparison is not None:
        _need(motor_power_epoch != "UNVERIFIED",
              "A branch comparison needs an independently observed motor power epoch")
        _need(type(branch_comparison) is StaticBranchComparison,
              "Require an existing StaticBranchComparison")
        binding = branch_comparison.validated_current_binding()
        _need(binding["boot_id"] == boot_id
              and binding["motor_power_epoch"] == motor_power_epoch
              and binding["uids_by_id"] == expected,
              "Reviewed branch belongs to another boot, motor-power epoch or UID set")
        snapshot["source_flags"] = {"power_epoch_branch_capture": {
            **binding, "uid_read_boot_id": binding["boot_id"],
            "uid_read_motor_power_epoch": binding["motor_power_epoch"],
            "motor_supply_off_on_observed": True,
            "disabled_zero_current_by_id": {str(i): True for i in range(1, 13)}}}
    if not warmup_completed:
        replay.warmup_policy(policy, torch_module, h_hypothesis, 3)
    run = observer.StatefulPolicyObserver(policy, calibration, imu_mount_candidate=mount,
        h_hypothesis=h_hypothesis, command=[0., 0., 0.], max_ticks=1,
        max_age_ns=max_age_ns, max_spread_ns=max_spread_ns,
        torch_module=torch_module, power_epoch_branch_comparison=branch_comparison)
    run.reset_run(tick_ns, warmup_completed=True)
    observed = run.consume(snapshot)
    summary = run.finish()
    _need(summary["status"] == "COMPLETE_NO_OUTPUT_DIAGNOSTIC",
          "Single policy diagnostic did not finish")
    return {**FLAGS, "status": "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT",
            "boot_id": boot_id, "motor_power_epoch_label": motor_power_epoch,
            "capture_tick_ns": tick_ns,
            "capture_is_postprocessed": True,
            "fresh_identity_match_with_calibration": True,
            "motor_power_epoch_attested": branch_comparison is not None,
            "imu_samples": len(imu_events), "can_cycles_per_bus": 20,
            "inference_calls_on_captured_input": 1,
            "snapshot": snapshot, "observer_tick": observed,
            "observer_summary": summary}


def collect_once(bindings, expected_uids, *, max_seconds=MAX_SECONDS,
                 receive_mode="select",
                 audit_status=None,
                 check_external=lambda: None):
    """Hardware entry point. Parent caller owns signal handling and boot guard."""
    _need(type(max_seconds) in (int, float) and 1 <= max_seconds <= MAX_SECONDS,
          "Acquisition budget must be 1..10 seconds")
    _need(receive_mode in ("serial", "select"), "Invalid CAN receive mode")
    bus = live.SessionBus()
    imu_deadline = bus.clock() + int(max_seconds*1e9)
    worker = threading.Thread(target=live.imu_producer,
        args=(bus, imu_deadline), kwargs={"check_external": check_external},
        name="dual-policy-once-imu", daemon=True)
    buffers = {scope: dual.EventBuffer(scope) for scope in dual.SCOPES}
    report = None
    try:
        worker.start()
        report = dual.collect_dual(bindings, expected_uids,
            {scope: holder.emit for scope, holder in buffers.items()},
            max_seconds, check_external=check_external,
            paired_cycle_sync=True, receive_mode=receive_mode)
    finally:
        bus.stop.set()
        if worker.ident is not None:
            worker.join(timeout=2.)
        imu_status = {"imu_worker_exited": worker.ident is not None and not worker.is_alive(),
                      "imu_restore_status": bus.producer_status.get("imu", {}).get(
                          "restore_status", "unconfirmed")}
        if audit_status is not None:
            audit_status.update(imu_status)
    _need(imu_status["imu_worker_exited"] and not bus.errors
          and imu_status["imu_restore_status"] in
              ("restored", "not_needed"),
          "IMU acquisition/restore or worker exit failed")
    imu_events = []
    while True:
        try:
            row = bus.logs.get_nowait()
        except queue.Empty:
            break
        if row.get("kind") == "imu":
            imu_events.append(row)
    return report, {scope: holder.rows for scope, holder in buffers.items()}, imu_events


def _write_private(path, value):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-no-output", action="store_true")
    for name in ("expected-uids", "front-port", "rear-port", "calibration",
                 "imu-mount-candidate", "bundle", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--branch-manifest", type=Path)
    parser.add_argument("--receive-mode", choices=("serial", "select"), default="select")
    parser.add_argument("--current-motor-power-epoch", required=True)
    parser.add_argument("--h-hypothesis", type=int, choices=(0, 1), required=True)
    args = parser.parse_args(argv)
    plan = {**FLAGS, "max_seconds": MAX_SECONDS, "can_cycles_per_bus": 20,
            "receive_mode": args.receive_mode,
            "paired_cycle_sync": True,
            "imu_polling": "existing ICM20948 10 ms loop",
            "inference_calls_on_captured_input": 1,
            "capture_is_postprocessed": True,
            "branch_manifest_supplied": args.branch_manifest is not None,
            "motor_power_epoch_label": args.current_motor_power_epoch,
            "motor_power_epoch_attested": False}
    if not args.execute_no_output:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    output = replay._output_path(args.output)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {**FLAGS, "status": "INCOMPLETE", "plan": plan,
              "imu_worker_exited": False, "imu_restore_status": "not_started",
              "started_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}
    previous_handlers, signals, guard = {}, [], None
    try:
        expected_source = args.expected_uids.read_bytes()
        expected_uids = dual.pipeline.validate_uids(shadow._json(expected_source))
        calibration = shadow._json(args.calibration.read_bytes())
        shadow.validate_calibration(calibration)
        _need(calibration["identities"] ==
              {str(i): expected_uids[i] for i in range(1, 13)},
              "Expected UIDs do not match calibration")
        mount = shadow.validate_imu_mount_candidate(
            shadow._json(args.imu_mount_candidate.read_bytes()))
        policy, model_source = shadow.load_policy(args.bundle)
        import torch
        replay.warmup_policy(policy, torch, args.h_hypothesis, 3)
        report["model_source"] = model_source
        report["input_sha256"] = {"expected_uids": hashlib.sha256(expected_source).hexdigest(),
            "calibration": shadow.sha(args.calibration),
            "imu_mount_candidate": shadow.sha(args.imu_mount_candidate),
            "branch_manifest": shadow.sha(args.branch_manifest) if args.branch_manifest else None}
        bindings = dual.validate_ports(args.front_port, args.rear_port)
        guard = dual.BootIdentityGuard()
        report["boot_id"] = guard.boot_id
        comparison = reviewed_branch_manifest(
            shadow._json(args.branch_manifest.read_bytes()) if args.branch_manifest else None,
            guard.boot_id, args.current_motor_power_epoch, expected_uids)
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[number] = signal.signal(number,
                lambda signum, _: signals.append(signum))
        def check():
            if signals:
                raise InterruptedError("Signal " + str(signals[0]))
            guard.check()
        dual_report, events, imu_events = collect_once(bindings, expected_uids,
            receive_mode=args.receive_mode, audit_status=report,
            check_external=check)
        report["dual_capture_status"] = dual_report.get("status")
        report["imu_event_count"] = len(imu_events)
        # Keep original reads even if a later calibration/range gate rejects
        # inference. All files stay in the caller's new private directory.
        _write_private(output / "dual-report.json", dual_report)
        for scope in dual.SCOPES:
            _write_private(output / ("events-" + scope + ".json"), events[scope])
        _write_private(output / "events-imu.json", imu_events)
        report["capture_file_sha256"] = {
            name: shadow.sha(output / name) for name in
            ("dual-report.json", "events-front.json", "events-rear.json", "events-imu.json")}
        _need(guard.boot_id == report["boot_id"], "Boot changed")
        report["observation"] = observe_one(dual_report, events, imu_events,
            calibration, mount, policy, torch, h_hypothesis=args.h_hypothesis,
            boot_id=guard.boot_id, motor_power_epoch=args.current_motor_power_epoch,
            branch_comparison=comparison,
            warmup_completed=True)
        report["status"] = "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT"
    except BaseException as error:
        report["failure"] = type(error).__name__ + ": " + str(error)
    finally:
        if guard is not None:
            guard.close()
        for number, previous in previous_handlers.items():
            signal.signal(number, previous)
        report["completed_at_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        _write_private(output / "summary.json", report)
    print(json.dumps({"status": report["status"], "output": str(output), **FLAGS},
                     ensure_ascii=False))
    return 0 if report["status"] == "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT" else 2


if __name__ == "__main__":
    raise SystemExit(main())
