#!/usr/bin/env python3
"""Ground supervisor + actual policy + C++ transport over TWO LOCAL socketpairs.

This is a software integration test, not contact/load/balance simulation. The
instantaneously tracking plant, upright IMU and operator acknowledgment are all
synthetic. No physical review, stage progression permission, real-time hardware
claim, serial device, network connection or SSH operation is produced.

The in-memory test fixtures deliberately bypass physical approval gates. In
particular their relaxed Mac timing limits are NOT approved for hardware walk.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import offline_policy_output_smoke as support

from singularitydog_hw.ground_trial_output import GroundTrialExecution
from singularitydog_hw import ground_trial_trajectory as trajectory


CASES = ("supported_stance", "walk", "walk-missing-ack")


def local_deadline_sleep(seconds):
    """Reduce Mac timer coalescing for this local test, not a real-time claim.

    All timestamps still come from the actual local monotonic clock. The last
    two milliseconds are a bounded busy wait; runtime and its guards are not
    changed. This keeps the resupport fault test distinct from a late timer.
    """
    deadline = time.monotonic_ns() + max(0, round(seconds * 1e9))
    remaining = (deadline - time.monotonic_ns()) / 1e9
    if remaining > .002:
        time.sleep(remaining - .002)
    while time.monotonic_ns() < deadline:
        pass


def source_hashes():
    pins = support.source_hashes()
    pins["support_smoke_tool"] = pins.pop("tool")
    pins["tool"] = support.digest(__file__)
    folder = support.REPO / "runtime/singularitydog_hw"
    for name in ("ground_trial_output", "ground_trial_trajectory", "ground_trial_plan", "ground_trial_review"):
        pins[name] = support.digest(folder / (name + ".py"))
    return pins


def synthetic_fixtures(stage):
    """In-memory bypass for sockets only; no approval or catch review is invented."""
    support.require(stage in ("supported_stance", "walk"), "Unknown simulated stage")
    profile = support.synthetic_profile()
    profile.update(duration_s=4., stop_duration_s=.2,
                   assembly_id="SIMULATED_NO_PHYSICAL_ROBOT", boot_id=support.BOOT,
                   motor_power_epoch="SIMULATED_NO_MOTOR_POWER",
                   profile_sha256=hashlib.sha256(b"offline-ground-in-memory-profile-only").hexdigest())
    reserve = max(a["max_command_velocity_rad_s"] / a["max_command_acceleration_rad_s2"]
                  for a in profile["axes"].values()) + profile["stop_duration_s"] + .04
    plan = {"simulation_only": True, "execution_plan_validated": True,
            "physical_validation_bypassed_for_local_sockets": True,
            "physical_review_generated": False,
            "stage": stage, "base_profile_sha256": profile["profile_sha256"],
            "assembly_id": profile["assembly_id"], "boot_id": profile["boot_id"],
            "motor_power_epoch": profile["motor_power_epoch"],
            "trajectory": {"stage": stage, "duration_s": profile["duration_s"],
                "initial_hold_s": profile["startup_duration_s"] + profile["policy_ramp_s"],
                "active_duration_s": 1.2, "forward_velocity_m_s": .05 if stage == "walk" else 0.,
                "ramp_up_s": .5 if stage == "walk" else 0.,
                "ramp_down_s": .5 if stage == "walk" else 0.,
                "final_stationary_s": .5, "resupport_window_s": 1., "shutdown_reserve_s": reserve}}
    return profile, plan


class RecordedGroundModel:
    """Record exact model inputs including the command override for eager replay."""
    def __init__(self, live):
        self.live = live
        self.rows = []

    @property
    def calls(self): return self.live.calls
    @property
    def provenance(self): return self.live.provenance
    @property
    def last_validation(self): return self.live.last_validation
    def validate_inputs(self, *args): return self.live.validate_inputs(*args)

    def __call__(self, sample, imu, now_ns, *, command_override=None):
        values = self.live.validate_inputs(sample, imu, now_ns)
        if command_override is not None:
            values = (*values[:2], list(command_override), *values[3:])
        target = tuple(self.live(sample, imu, now_ns, command_override=command_override))
        self.rows.append({"simulation_only": True, "call_monotonic_ns": now_ns,
                          "inputs": values, "target_can_order": list(target)})
        return target


class SimulatedGroundExecution(GroundTrialExecution):
    """Explicit synthetic operator, instantiated only by this socket smoke tool."""
    def __init__(self, plan, *, acknowledge):
        self.synthetic_ack_enabled = acknowledge
        self.synthetic_ack_due_ns = None
        self.displayed = []
        super().__init__(plan, hashlib.sha256(b"SIMULATION_ONLY_NO_PHYSICAL_PLAN").hexdigest(),
                         execution_kind="simulation", input_stream=None, emit=self._record_cue)

    def _record_cue(self, text):
        now = time.monotonic_ns()
        self.displayed.append({"simulation_only": True, "monotonic_ns": now, "text": text})
        if text.startswith("resupport_window_open ") and self.synthetic_ack_enabled:
            # Never acknowledge in the cue-emission cycle. The actual supervision
            # receives a new event on a later cycle after the visible cue.
            self.synthetic_ack_due_ns = now + 25_000_000

    def before_cycle(self, now_ns, *, stop_requested=False):
        if self.synthetic_ack_due_ns is not None and now_ns >= self.synthetic_ack_due_ns:
            self.acknowledge_resupport(now_ns)
            self.events.append({"key": "SIMULATED_OPERATOR_ACK_INJECTED", "monotonic_ns": now_ns,
                                "simulation_only": True, "physical_readiness": False})
            self.synthetic_ack_due_ns = None
        return super().before_cycle(now_ns, stop_requested=stop_requested)


def audit_case(report, peers, execution, recorded, case):
    ground = report["ground_report"]
    raw = ground["runtime_report"]
    support.require(ground["simulation_only"] and ground["execution_kind"] == "simulation",
                    "Synthetic execution mislabeled as physical")
    support.require(ground["dependency_eligible"] is False and ground["physical_result"] == "UNREVIEWED",
                    "Simulation acquired physical stage approval")
    support.require(all(not p.errors and not p.parser.buffer and not p.parser.discarded_bytes
                        and not p.thread.is_alive() for p in peers.values()), "Socket peer failed")
    support.require(recorded.calls == len(recorded.rows) > 20, "Real model path not exercised")
    support.require(raw["learned_targets_sent"], "Learned targets did not reach native output")
    support.require(raw["stop_confirmed"], "All-axis attributable STOP required in clean socket trial")
    support.require(all(not peer.enabled for peer in peers.values()), "Synthetic actuator remains enabled")
    tx_rows = [row for peer in peers.values() for row in peer.rows]
    support.require({r["motor_id"] for r in tx_rows if r["kind"] == 3} == set(range(1, 13)),
                    "All twelve explicit enables were not exercised")
    support.require(all(r["kind"] in (0, 1, 3, 4, 17, 18) for r in tx_rows), "Unexpected native request type")
    for peer in peers.values():
        last_motion = max(i for i, row in enumerate(peer.rows) if row["kind"] == 1)
        support.require([(r["kind"], r["motor_id"]) for r in peer.rows[last_motion+1:]] ==
                        [(4, mid) for mid in peer.ids], "Terminal STOP is not a single all-axis pass")
    commands = [row["inputs"][2] for row in recorded.rows]
    support.require(all(len(c) == 3 and 0 <= c[0] <= .05 and c[1:] == [0., 0.] for c in commands),
                    "Forward-only command domain violated")
    for row in recorded.rows:
        t = (row["call_monotonic_ns"] - execution.started_ns) / 1e9
        support.require(tuple(row["inputs"][2]) == execution.timeline.command_at(t),
                        "Model did not receive exact supervised command")
    if execution.plan["stage"] == "walk":
        positive = [i for i, c in enumerate(commands) if c[0] > 0]
        support.require(bool(positive) and positive[0] > 0 and positive[-1] < len(commands)-1,
                        "Missing initial hold, walking, or final zero-command segment")
        support.require(any(0 < c[0] < .05 for c in commands[:positive[0]+12]), "Ramp-up was not exercised")
        support.require(max(c[0] for c in commands) == .05, "Walk cruise was not exercised")
        support.require(any(0 < c[0] < .05 for c in commands[positive[-1]-12:]), "Ramp-down was not exercised")
    else:
        support.require(all(c == [0., 0., 0.] for c in commands), "Supported stance commanded locomotion")
    cues = ground["cues"]
    open_cue = next(c for c in cues if c["key"] == "resupport_window_open")
    audit = {"status": "PASS", "real_model_calls": recorded.calls,
             "maximum_forward_command_m_s": max(c[0] for c in commands),
             "physical_review_generated": False, "dependency_eligible": False,
             "stop_confirmed_ids": sorted(mid for stop in raw["stop_reports"].values()
                                          for mid in stop["confirmed_ids"])}
    if case == "walk-missing-ack":
        support.require(ground["status"] == "ABORTED", "Missing acknowledgment did not abort")
        support.require(any("resupport_not_confirmed" in e for e in raw["errors"]), "Wrong abort reason")
        support.require(not raw["normal_ramp_completed"], "Missing acknowledgment used graceful gain-down")
        support.require(not any(c["phase"] in ("stopping", "stopped") for c in raw["cycles"]),
                        "Fault used a normal stop ramp")
        support.require(ground["resupport_ack"]["accepted_elapsed_s"] is None, "Missing acknowledgment synthesized")
        support.require(not any(e["key"] == "OPERATOR_RESUPPORT_ACK" for e in cues), "Unexpected acknowledgment")
        audit["missing_ack_immediate_emergency"] = True
    else:
        support.require(ground["status"] == "COMPLETE_BOUNDED_GROUND_TRIAL", str(raw["errors"]))
        support.require(raw["normal_ramp_completed"] and raw["cycles"][-1]["command"]["gain_scale"] == 0.,
                        "Normal finite gain-down incomplete")
        audit["native_success_audit"] = support.audit_case(
            {"runtime": raw}, peers, "real-success", SimpleNamespace(calls=recorded.rows))
        if case == "walk":
            ack = ground["resupport_ack"]
            support.require(ack["accepted_elapsed_s"] is not None and
                            ack["monotonic_ns"] > open_cue["monotonic_ns"], "No fresh post-cue acknowledgment")
            support.require(ack["monotonic_ns"] > next(r["monotonic_ns"] for r in execution.displayed
                            if r["text"].startswith("resupport_window_open ")), "Acknowledgment preceded display")
            audit["fresh_simulated_resupport_ack_after_visible_cue"] = True
        audit["normal_stop_gain_zero"] = True
    return audit


def run_case(case, lib, out, *, state, bundle, expected_sources=None):
    support.require(case in CASES, "Unknown ground smoke case")
    out = Path(out); out.mkdir(mode=0o700)
    stage = "walk" if case.startswith("walk") else "supported_stance"
    profile, plan = synthetic_fixtures(stage)
    peers, hosts, devices, sessions, cancel_fds = {}, [], [], {}, []
    report = {"status": "INCOMPLETE_OFFLINE_GROUND_SIMULATION", "case": case,
        "simulation_only": True, "hardware_opened": False, "approved_for_runtime": False,
        "output_allowed": False, "physical_review_generated": False, "dependency_eligible": False,
        "physical_motor_enable_sent": False, "physical_learned_targets_sent": False,
        "jetson_latency_measurement": False, "real_controller_50Hz_verified": False,
        "physical_stage_progression_authorized": False,
        "physical_validation_bypassed_for_local_sockets": True,
        "simulated_profile_has_relaxed_timing_not_valid_for_hardware_walk": True,
        "socket_transport": "Exactly two local AF_UNIX socketpairs; no /dev, SSH or network",
        "plant_description": "Instantaneous position tracking; no dynamics, contact, load, balance or watchdog emulation",
        "local_sleep": "Host monotonic deadline; sleep bulk then busy wait final2ms; simulation only, no real-time claim",
        "all_new_timestamps_are_simulated": True, "new_robot_samples": 0,
        "errors": [], "source_sha256": expected_sources or source_hashes()}
    boot = tempfile.TemporaryFile(); boot.write((support.BOOT + "\n").encode()); boot.flush()
    recorded = execution = None
    try:
        support.require(report["source_sha256"] == source_hashes(), "Source changed before this simulated run")
        helper_recorded, provenance = support.learned_policy(profile, state, bundle, out)
        recorded = RecordedGroundModel(helper_recorded.policy)
        report["real_policy_provenance"] = provenance
        execution = SimulatedGroundExecution(plan, acknowledge=case == "walk")
        execution.bind_profile(profile, active=True)
        model = execution.wrap_model(recorded)
        imu = support.SyntheticIMU()
        for scope, ids in support.SCOPES.items():
            host, device = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
            hosts.append(host); devices.append(device); host.setblocking(False)
            cr, cw = os.pipe(); cancel_fds.extend((cr, cw))
            peer = support.SocketPlant(device, ids)
            peers[scope] = peer; peer.thread.start()
            axis = lambda key: {i: profile["axes"][str(i)][key] for i in ids}
            sessions[scope] = support.native.ActiveSession(lib, host.fileno(), first_id=ids[0], cancel_fd=cr,
                boot_fd=boot.fileno(), boot_id=support.BOOT, raw_lower_by_id=axis("lower_rad"),
                raw_upper_by_id=axis("upper_rad"), kp_max_by_id=axis("kp"), kd_max_by_id=axis("kd"))
        cancelled = threading.Event()
        def cancel():
            if not cancelled.is_set():
                cancelled.set()
                for fd in cancel_fds[1::2]: os.write(fd, b"x")
        execution.connect_cancel(cancel)
        raw_report = support.runtime.run_supported_policy(profile, sessions, imu, model,
            cancel_io=cancel, supervision=execution, sleep=local_deadline_sleep)
        report["ground_report"] = execution.decorate_report(raw_report)
        report["simulated_imu_reads"] = imu.calls
        report["owner_cancel_invoked"] = cancelled.is_set()
        report["displayed_cues"] = execution.displayed
        if recorded.rows:
            report["eager_parity"] = support.eager_parity(
                SimpleNamespace(calls=recorded.rows, policy=recorded.live), bundle)
    except BaseException as error:
        report["errors"].append(type(error).__name__ + ": " + str(error))
    finally:
        for session in sessions.values(): session.close()
        for host in hosts:
            try: host.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            host.close()
        for peer in peers.values(): peer.thread.join(timeout=1.)
        for device in devices: device.close()
        for fd in cancel_fds: os.close(fd)
        boot.close()
        report["all_local_fds_closed"] = all(x.fileno() == -1 for x in hosts + devices) and boot.closed
        report["peers"] = {scope: peer.evidence() for scope, peer in peers.items()}
        report["policy_calls"] = [] if recorded is None else recorded.rows
        report["total_native_tx_frames"] = sum(len(p.rows) for p in peers.values())
        report["total_native_rx_frames"] = sum(bool(r["reply_hex"]) for p in peers.values() for r in p.rows)
        if not report["errors"]:
            try:
                report["audit"] = audit_case(report, peers, execution, recorded, case)
                report["status"] = "PASS_OFFLINE_GROUND_SIMULATION"
            except BaseException as error: report["errors"].append(type(error).__name__ + ": " + str(error))
        if report["source_sha256"] != source_hashes():
            report["status"] = "INCOMPLETE_OFFLINE_GROUND_SIMULATION"
            report["errors"].append("Source changed during this simulated run")
        support.write_json(out / "report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("library", "state", "bundle", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--case", choices=("all", *CASES), default="all")
    args = parser.parse_args(argv)
    out = args.output.absolute()
    support.require(not out.exists() and out.parent.is_dir(), "Use a new output directory with an existing parent")
    support.require(not any((p / ".git").exists() for p in (out, *out.parents)), "Raw reports must stay outside Git")
    out.mkdir(mode=0o700)
    import torch
    torch.set_num_threads(1); torch.set_num_interop_threads(1)
    lib = support.native.load_library(args.library)
    cases = CASES if args.case == "all" else (args.case,)
    pins = source_hashes()
    reports = {case: run_case(case, lib, out / case, state=args.state, bundle=args.bundle,
                             expected_sources=pins) for case in cases}
    summary = {"simulation_only": True, "hardware_opened": False, "approved_for_runtime": False,
               "physical_review_generated": False, "dependency_eligible": False,
               "physical_stage_progression_authorized": False, "jetson_latency_measurement": False,
               "library_sha256": support.digest(args.library), "source_sha256": pins,
               "all_sources_unchanged": pins == source_hashes(), "cases": {}}
    for case, report in reports.items():
        ground = report.get("ground_report", {})
        summary["cases"][case] = {"status": report["status"], "errors": report["errors"],
            "ground_status": ground.get("status"), "tx_frames": report["total_native_tx_frames"],
            "rx_frames": report["total_native_rx_frames"], "real_model_calls": len(report["policy_calls"]),
            "stop_confirmed": ground.get("stop_confirmed"),
            "eager_parity": report.get("eager_parity"), "report_sha256": support.digest(out / case / "report.json")}
    summary["status"] = "PASS" if summary["all_sources_unchanged"] and all(
        r["status"] == "PASS_OFFLINE_GROUND_SIMULATION" for r in reports.values()) else "FAILED"
    support.write_json(out / "summary.json", summary)
    print(json.dumps(summary))
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
