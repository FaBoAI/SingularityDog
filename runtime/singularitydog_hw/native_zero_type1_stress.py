"""Finite native Type1 transport comparison; PLAN_ONLY unless explicitly armed.

The existing active C++ transport owns both serial streams from open through
STOP. No Python serial parser, ownership handoff, learned model, positive gain,
motion retry, or change to active-control timing limits is involved. The 25 ms
reply budget belongs only to this zero-gain diagnostic; 20 ms is reported
separately and is never claimed as learned-control validation.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

from . import can_readonly as codec
from . import native_active_transport as native
from . import rs05_trial_protocol as protocol
from . import thread_timer_slack as timer_slack
from .motor_version_probe import validate_uids
from .native_diagnostic_transport import exchange_evidence


CONDITIONS = {"800us-window3": (800, 3), "800us-window2": (800, 2),
              "900us-window3": (900, 3), "1000us-window3": (1000, 3)}
BUSES = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}
PERIOD_NS = 20_000_000
DIAGNOSTIC_DEADLINE_NS = 25_000_000
TOTAL_NS = 25_000_000_000
ENABLE_TOTAL_NS = 120_000_000


def plan(cycles=5, *, condition="800us-window3", timer_slack_ns=None):
    if type(cycles) is not int or cycles not in (5, 50, 500):
        raise ValueError("Choose exactly 5, 50, or 500 finite diagnostic cycles")
    if type(condition) is not str or condition not in CONDITIONS:
        raise ValueError("Choose one explicit native transport condition")
    if timer_slack_ns is not None and (type(timer_slack_ns) is not int or timer_slack_ns != 1000):
        raise ValueError("Optional bus-owner timer slack must be exactly 1000 ns")
    gap, window = CONDITIONS[condition]
    return {"schema": "native-zero-type1-stress-v1", "status": "PLAN_ONLY",
            "condition": condition, "cycles": cycles, "period_ms": 20,
            "diagnostic_reply_deadline_ms": 25, "total_budget_s": 25,
            "request_gap_us": gap, "request_window": window,
            "timer_slack_ns": timer_slack_ns,
            "type1_requests_per_cycle": 24, "voltage_requests_per_cycle": 2,
            "kp": 0, "kd": 0, "feedforward_torque_nm": 0,
            "motor_enable_available": True, "positive_gain_available": False,
            "learned_targets_available": False, "motion_retry_allowed": False,
            "transport": "native_active_transport", "parser_handoff": False,
            "hardware_opened": False, "actual_policy_output_20ms_verified": False}


class ZeroGainSession(native.ActiveSession):
    """Construct immutable native zero-gain caps; there is no gain argument."""
    def __init__(self, library, fd, *, first_id, cancel_fd, boot_fd, boot_id,
                 condition):
        plan(condition=condition)
        gap, window = CONDITIONS[condition]
        ids = range(first_id, first_id+6)
        super().__init__(library, fd, first_id=first_id, cancel_fd=cancel_fd,
            boot_fd=boot_fd, boot_id=boot_id,
            raw_lower_by_id={mid: -12.57 for mid in ids},
            raw_upper_by_id={mid: 12.57 for mid in ids},
            kp_max_by_id={mid: 0. for mid in ids},
            kd_max_by_id={mid: 0. for mid in ids}, gap_ns=gap*1000, window=window)
        self.condition = condition


def _need(condition, message):
    if not condition:
        raise RuntimeError(message)


def _feedback(rows, ids, centers=None, *, mode):
    _need(set(rows) == {(mid, "feedback") for mid in ids},
          "Incomplete or cross-bus native feedback")
    found = {}
    for mid in ids:
        value = rows[mid, "feedback"][0]
        q = value.protocol_position_rad
        _need(value.mode_state == mode and value.fault_bits == 0,
              f"ID{mid} native feedback mode/fault mismatch")
        _need(math.isfinite(q),
              f"ID{mid} non-finite position_rad={q!r}; required finite within [-12.57, 12.57]")
        _need(-12.57 <= q <= 12.57,
              f"ID{mid} position_rad={q!r} outside [-12.57, 12.57]")
        velocity = value.velocity_rad_s
        _need(math.isfinite(velocity),
              f"ID{mid} non-finite velocity_rad_s={velocity!r}; required finite with abs <= 0.5")
        _need(abs(velocity) <= .5,
              f"ID{mid} velocity_rad_s={velocity!r} exceeds abs limit 0.5 rad/s")
        temperature = value.temperature_c
        _need(math.isfinite(temperature),
              f"ID{mid} non-finite temperature_c={temperature!r}; required finite within [-10, 60)")
        _need(-10 <= temperature < 60,
              f"ID{mid} temperature_c={temperature!r} outside [-10, 60) C")
        if centers is not None:
            _need(abs(q-centers[mid]) <= math.radians(3),
                  f"ID{mid} moved beyond zero-gain diagnostic envelope")
        found[mid] = q
    return found


def _voltage(rows, ids):
    _need(set(rows) == {(mid, "voltage") for mid in ids},
          "Incomplete native voltage response")
    for mid in ids:
        value = rows[mid, "voltage"][0]["value"]
        _need(math.isfinite(value) and 35 <= value <= 42,
              f"ID{mid} voltage outside 35..42V")


def _summary(journal):
    """Materialize timing evidence only after both owners have stopped."""
    def distribution(values):
        return {"count": len(values), "min": min(values) if values else None,
                "median": statistics.median(values) if values else None,
                "max": max(values) if values else None}
    result = {}
    for scope in BUSES:
        entries = [row for row in journal if row["bus"] == scope]
        cyclic = [row for row in entries if row["phase"].startswith("cycle_")]
        records = sorted((r for row in cyclic for r in row["records"] if r["start_ns"]),
                         key=lambda r: r["start_ns"])
        gaps = [(b["start_ns"]-a["finish_ns"])/1000 for a, b in zip(records, records[1:])]
        latencies = [(r["received_ns"]-r["finish_ns"])/1e6
                     for r in records if r["received"] == 17]
        result[scope] = {
            "journal_writes": sum(row["stats"]["writes"] for row in entries),
            "journal_tx_bytes": sum(r["written"] for row in entries for r in row["records"]),
            "journal_rx_bytes": sum(row["stats"]["bytes"] for row in entries),
            "cyclic_writes": sum(row["stats"]["writes"] for row in cyclic),
            "cyclic_rx_bytes": sum(row["stats"]["bytes"] for row in cyclic),
            "cyclic_complete_replies": sum(r["received"] == 17 for r in records),
            "cyclic_write_gap_us": distribution(gaps),
            "cyclic_reply_latency_ms": distribution(latencies),
            "stop_traffic_is_in_stop_reports": True}
    return result


def run(sessions, expected_uids, *, cycles=5, condition="800us-window3",
        cancel_io, announce=lambda: None, check=lambda: None,
        clock=time.monotonic_ns, wait=time.sleep, deadline_wait=None, timer_slack_ns=None):
    """Use existing owners and sticky STOP logic; never open a device here."""
    intended = plan(cycles, condition=condition, timer_slack_ns=timer_slack_ns)
    expected = validate_uids(expected_uids)
    if (set(sessions) != set(BUSES) or sessions["front"] is sessions["rear"] or
            any(not isinstance(session, ZeroGainSession) or
                session.first_id != BUSES[scope][0] or session.condition != condition
                for scope, session in sessions.items())):
        raise ValueError("Two native ZeroGainSession owners matching the selected condition required")
    from .policy_output_runtime import BusWorkers, OutputWatchdog
    workers = BusWorkers(sessions, cancel_io, clock)
    watcher = OutputWatchdog(workers, 40_000_000, clock)
    result = {**intended, "status": "ABORTED", "errors": [], "per_cycle": [],
              "cycles_completed": 0, "motor_enable_attempted": False,
              "positive_gain_sent": False, "learned_targets_sent": False,
              "stop_confirmed": False, "centers_rad_by_id": {},
              "requested_cycles": cycles, "native_release_wait": deadline_wait is not None}
    started = clock()
    centers = {}
    slack_scopes = ({scope: timer_slack.TimerSlack(timer_slack_ns) for scope in BUSES}
                    if timer_slack_ns is not None else {})
    result["timer_slack"] = {
        "scope": "front_rear_bus_owners_only", "requested_ns": timer_slack_ns,
        "enabled": timer_slack_ns is not None, "apply_verified": False,
        "restoration_complete": None, "status": "not_applied" if slack_scopes else "inactive",
        "owners": {scope: value.report for scope, value in slack_scopes.items()}}

    def guard():
        check()
        _need(not workers.aborted.is_set(), "Native zero diagnostic cancelled")
        _need(started <= clock() < started+TOTAL_NS, "Native zero diagnostic total deadline exceeded")

    def exchange(wires, label, *, timeout_ns=100_000_000, limit=None):
        guard()
        deadline = min(clock()+timeout_ns, started+TOTAL_NS,
                       limit if limit is not None else started+TOTAL_NS)
        values = workers.collect(workers.submit_decoded(wires, deadline_ns=deadline, label=label))
        return {scope: value[1] for scope, value in values.items()}

    def all_read(parameter, label):
        return exchange({scope: [codec.read_request(mid, parameter) for mid in ids]
                         for scope, ids in BUSES.items()}, label)

    def stop_pose(label):
        rows = exchange({scope: [protocol.stop_request(phase=protocol.TrialPhase.STOP,
                            motor_id=mid) for mid in ids] for scope, ids in BUSES.items()}, label)
        pose = {}
        for scope, ids in BUSES.items():
            pose.update(_feedback(rows[scope], ids, centers or None, mode=0))
        return pose

    def verify_parameters(label):
        rows = all_read("run_mode", label+"_run_mode")
        for scope, ids in BUSES.items():
            _need(set(rows[scope]) == {(mid, "run_mode") for mid in ids}, "Missing MIT mode reply")
            for mid in ids:
                _need(rows[scope][mid, "run_mode"][0]["value"] == 0,
                      f"ID{mid} MIT mode not confirmed")
        rows = all_read("voltage", label+"_voltage")
        for scope, ids in BUSES.items():
            _voltage(rows[scope], ids)

    def verify_watchdog(label):
        rows = all_read("can_timeout", label)
        for scope, ids in BUSES.items():
            _need(set(rows[scope]) == {(mid, "can_timeout") for mid in ids},
                  "Missing watchdog readback")
            for mid in ids:
                _need(rows[scope][mid, "can_timeout"][0]["value"] == 4000,
                      f"ID{mid} watchdog readback mismatch")

    def cycle_owner(scope, cycle, wires, deadline):
        # Validation runs on the FD owner: any bad reply cancels the other bus
        # before a coordinator blocked on another future can miss the failure.
        try:
            ids = BUSES[scope]
            for phase in ("input", "output"):
                guard()
                decoded = workers._exchange_decoded(scope, wires, deadline,
                                                     f"cycle_{cycle+1}_{phase}")
                _feedback(decoded[1], ids, centers, mode=2)
                if phase == "input":
                    guard()
                    mid = ids[cycle % 6]
                    decoded = workers._exchange_decoded(scope, [codec.read_request(mid, "voltage")],
                                                         deadline, f"cycle_{cycle+1}_voltage")
                    _voltage(decoded[1], (mid,))
            return clock()
        except BaseException as error:
            workers.emergency(type(error).__name__+": "+str(error))
            raise

    try:
        if slack_scopes:
            # TimerSlack's parent is this individual FD owner. Do not use its
            # three-worker inheritance API or change the coordinator's slack.
            workers.collect({scope: workers.pools[scope].submit(value.__enter__)
                             for scope, value in slack_scopes.items()})
            parents = [value.report["parent"] for value in slack_scopes.values()]
            _need(len({row["native_tid"] for row in parents}) == 2 and
                  all(row["during_ns"] == timer_slack_ns for row in parents),
                  "Two distinct native bus-owner timer slack readbacks required")
            result["timer_slack"].update(apply_verified=True, status="applied")
        identity = all_read(None, "preflight_identity")
        for scope, ids in BUSES.items():
            _need(set(identity[scope]) == {(mid, "identity") for mid in ids}, "Missing UID reply")
            for mid in ids:
                _need(identity[scope][mid, "identity"][0]["mcu_uid_hex"] == expected[mid],
                      f"ID{mid} UID mismatch")
        centers.update(stop_pose("preflight_stop"))
        verify_parameters("preflight")
        setup = exchange({scope: [protocol.watchdog_setup_request(
            phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid) for mid in ids]
            for scope, ids in BUSES.items()}, "preflight_watchdog_setup")
        for scope, ids in BUSES.items():
            _feedback(setup[scope], ids, centers, mode=0)
        verify_watchdog("preflight_watchdog_readback")
        guard(); announce(); guard()
        # Audio can last seconds. Revalidate the pose, voltage and watchdog
        # immediately before enabling, using the same native stream owner.
        centers.update(stop_pose("after_announcement_stop"))
        verify_parameters("after_announcement")
        verify_watchdog("after_announcement_watchdog_readback")
        result["centers_rad_by_id"] = {str(mid): q for mid, q in centers.items()}
        wires = {scope: tuple(native.encode_motion(mid, centers[mid], 0., 0.) for mid in ids)
                 for scope, ids in BUSES.items()}
        transition_end = clock()+ENABLE_TOTAL_NS
        watcher.kick()
        for index in range(6):
            result["motor_enable_attempted"] = True
            enabled = exchange({scope: [protocol.enable_request(phase=protocol.TrialPhase.ENABLE,
                                   motor_id=ids[index])] for scope, ids in BUSES.items()},
                               "startup_enable", timeout_ns=30_000_000, limit=transition_end)
            for scope, ids in BUSES.items():
                row = enabled[scope][ids[index], "feedback"][0]
                _need(row.mode_state in (0, 2) and row.fault_bits == 0, "Enable rejected")
            zero = exchange({scope: [wires[scope][index]] for scope in BUSES},
                            "startup_zero", timeout_ns=20_000_000, limit=transition_end)
            for scope, ids in BUSES.items():
                _feedback(zero[scope], (ids[index],), centers, mode=2)
            watcher.kick()
        guard()
        _need(clock() < transition_end, "Native zero-enable transition exceeded 120 ms")
        next_release = clock()
        previous_begin = None
        for cycle in range(cycles):
            guard()
            while clock() < next_release:
                guard()
                if deadline_wait is not None:
                    deadline_wait(next_release)
                else:
                    wait(min(.001, max(0., (next_release-clock())/1e9)))
            begin = clock()
            deadline = min(begin+DIAGNOSTIC_DEADLINE_NS, started+TOTAL_NS)
            row = {"cycle": cycle+1, "begin_ns": begin, "deadline_ns": deadline,
                   "requested_release_ns": next_release,
                   "release_lateness_ms": max(0, begin-next_release)/1e6,
                   "start_interval_ms": None if previous_begin is None else (begin-previous_begin)/1e6,
                   "complete": False}
            result["per_cycle"].append(row)
            with workers.lock:
                _need(not workers.aborted.is_set(), "Cancelled before native cycle submission")
                futures = {scope: workers.pools[scope].submit(cycle_owner, scope, cycle,
                            wires[scope], deadline) for scope in BUSES}
            try:
                workers.collect(futures)
            finally:
                row["end_ns"] = clock()
                row["elapsed_ms"] = (row["end_ns"]-begin)/1e6
                row["deadline20ms_missed"] = row["end_ns"]-begin > PERIOD_NS
            guard()
            _need(row["end_ns"] < deadline, "Native diagnostic cycle exceeded 25 ms")
            row["complete"] = True
            result["cycles_completed"] = cycle+1
            watcher.kick()
            # No shortened catch-up interval after a diagnostic overrun.
            previous_begin = begin
            next_release = max(begin+PERIOD_NS, row["end_ns"])
        result["status"] = "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC"
    except BaseException as error:
        result["errors"].append(type(error).__name__+": "+str(error))
        workers.emergency(result["errors"][-1])
    finally:
        try:
            try:
                stops = workers.finish_stops()
            except BaseException as error:
                result["errors"].append("STOP collection: "+repr(error))
                stops = {scope: {"complete": False, "unconfirmed_ids": list(ids)}
                         for scope, ids in BUSES.items()}
            result["stop_reports"] = stops
            result["stop_confirmed"] = all(stops[scope].get("complete") is True and
                stops[scope].get("confirmed_ids") == list(ids) and
                not stops[scope].get("ambiguous_ids") and not stops[scope].get("unconfirmed_ids")
                for scope, ids in BUSES.items())
        finally:
            try:
                watcher.close()
            except BaseException as error:
                result["errors"].append("Host watchdog close: "+repr(error))
                if result["status"] == "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC":
                    result["status"] = "ABORTED_WATCHDOG_CLOSE"
            try:
                # STOP completes before restoration; the pools remain alive
                # so each original owner restores and verifies its own value.
                restores = {scope: workers.pools[scope].submit(value.__exit__, None, None, None)
                    for scope, value in slack_scopes.items()
                    if type(value.report["parent"]["original_ns"]) is int and
                    value.report["parent"]["original_ns"] > 0 and
                    value.report["parent"]["restored"] is not True}
                for scope, future in restores.items():
                    try:
                        future.result()
                    except BaseException as error:
                        result["errors"].append(scope+" timer slack restore: "+repr(error))
                if slack_scopes:
                    restored = all(value.report["parent"]["original_ns"] is None or
                                   value.report["parent"]["restored"] is True
                                   for value in slack_scopes.values())
                    result["timer_slack"]["restoration_complete"] = restored
                    result["timer_slack"]["status"] = (
                        "restored" if restored and result["timer_slack"]["apply_verified"] else "failed")
                    if not restored and result["status"] == "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC":
                        result["status"] = "ABORTED_TIMER_SLACK_RESTORE"
            finally:
                workers.close()
        result["stop_faults_by_id"] = {mid: bits for stop in stops.values()
            for mid, bits in stop.get("fault_by_id", {}).items() if bits}
        if result["stop_faults_by_id"]:
            result["status"] = "ABORTED_STOP_FAULT"
            result["errors"].append("Fault reported during STOP")
        if not result["stop_confirmed"]:
            result["status"] = "STOP_UNCONFIRMED_POWER_OFF_REQUIRED"
            result["errors"].append("Physically switch motor power Off")
        result["host_watchdog_reason"] = workers.reason
        result["journal"] = [{"bus": scope, "phase": label, "error": error,
            **exchange_evidence(*value), "rejected_total": value[1].rejected_total,
            "rejected_truncated": value[1].rejected_total > value[1].rejected_size}
            for scope, value, error, label in workers.journal]
        result["transport_summary"] = _summary(result["journal"])
        result["deadline20ms_misses"] = sum(row.get("deadline20ms_missed", False)
                                             for row in result["per_cycle"])
        # Preserve the active report/analyze_can_reply_loss completed-cycle shape.
        result["cycles"] = [row for row in result["per_cycle"] if row["complete"]]
        result["elapsed_s"] = (clock()-started)/1e9
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cycles", type=int, choices=(5, 50, 500), default=5)
    p.add_argument("--condition", choices=tuple(CONDITIONS))
    p.add_argument("--timer-slack-ns", type=int, choices=(1000,),
                   help="Opt in to 1us on the two native bus owners; restore after STOP")
    p.add_argument("--execute-supported-zero-gain", action="store_true")
    for name in ("support-in-place", "cutoff-ready", "rs05-model-confirmed"):
        p.add_argument("--"+name, action="store_true")
    for name in ("front-port", "rear-port", "library", "expected-uids", "boot-id",
                 "power-epoch", "output", "audio", "audio-sha256", "audio-device"):
        p.add_argument("--"+name)
    a = p.parse_args(argv)
    intended = plan(a.cycles, condition=a.condition or "800us-window3", timer_slack_ns=a.timer_slack_ns)
    if not a.execute_supported_zero_gain:
        print(json.dumps(intended, indent=2)); return 0
    if not all((a.support_in_place, a.cutoff_ready, a.rs05_model_confirmed)):
        p.error("Current support, physical cutoff and RS05 confirmation required")
    required = ("condition", "front_port", "rear_port", "library", "expected_uids",
                "boot_id", "power_epoch", "output", "audio", "audio_sha256", "audio_device")
    if any(not getattr(a, name) for name in required):
        p.error("One explicit condition, native library, ports, epoch, output and audio inputs required")
    if a.timer_slack_ns is not None:
        timer_slack.require_supported_platform()
    audio = Path(a.audio).expanduser().resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest() != a.audio_sha256:
        p.error("Audio file hash mismatch")
    source = Path(a.expected_uids).expanduser().read_bytes()
    expected = validate_uids(json.loads(source))
    library_path = Path(a.library).expanduser().resolve(strict=True)
    library = native.load_library(library_path)  # Verify source/binary before any serial open.
    if getattr(library, "sda_wait_until", None) is None:
        p.error("Native diagnostic requires the cancellation-aware release waiter")
    provenance = json.loads((library_path.parent/"build-record.json").read_text())
    out = Path(a.output).expanduser().resolve()
    if any((parent/".git").exists() for parent in (out, *out.parents)):
        p.error("Raw output must stay outside Git")
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    from . import dual_can_pipeline_benchmark as dual
    from .policy_output import SignalState
    from .sensor_pipeline_benchmark import BootIdentityGuard
    cr, cw = os.pipe()
    signals = SignalState(cw)
    handlers = {}
    report = {**intended, "status": "ABORTED_BEFORE_OPEN", "errors": [],
              "motor_enable_attempted": False, "stop_confirmed": False,
              "positive_gain_sent": False, "learned_targets_sent": False}
    try:
        if Path("/proc/sys/kernel/random/boot_id").read_text().strip() != a.boot_id:
            raise RuntimeError("Jetson boot changed")
        bindings = dual.validate_ports(a.front_port, a.rear_port)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, signals.handler)
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks())
            boot = BootIdentityGuard(); stack.callback(boot.close)
            if boot.boot_id != a.boot_id:
                raise RuntimeError("Jetson boot changed")
            def check():
                if signals.cancelled:
                    raise InterruptedError("Operator interrupted native zero diagnostic")
                boot.check()
            import serial
            sessions = {}
            for scope, binding in bindings.items():
                check()
                stack.enter_context(dual.port_lock(binding["resolved"]))
                port = serial.Serial(port=None, baudrate=921600, timeout=0,
                                     write_timeout=.02, exclusive=True)
                port.dtr = port.rts = False
                port.port = binding["path"]; port.open(); stack.callback(port.close)
                report.update(hardware_opened=True, status="ABORTED_DURING_SETUP")
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev != binding["st_rdev"]:
                    raise RuntimeError("USB binding changed")
                boot_fd = os.open("/proc/sys/kernel/random/boot_id", os.O_RDONLY|os.O_CLOEXEC)
                stack.callback(os.close, boot_fd)
                sessions[scope] = ZeroGainSession(library, port.fileno(), first_id=BUSES[scope][0],
                    cancel_fd=cr, boot_fd=boot_fd, boot_id=boot.boot_id, condition=a.condition)
                stack.callback(sessions[scope].close)
            def announce():
                check()
                subprocess.run(["aplay", "-D", a.audio_device, str(audio)], check=True,
                    timeout=8., stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE)
            report = run(sessions, expected, cycles=a.cycles, condition=a.condition,
                         cancel_io=signals.cancel, announce=announce, check=check,
                         deadline_wait=lambda target: native.wait_until(library, cr, target, spin_us=500),
                         timer_slack_ns=a.timer_slack_ns)
            report["hardware_opened"] = True
    except BaseException as error:
        report["errors"].append(type(error).__name__+": "+str(error))
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        os.close(cr); os.close(cw)
        report.update(boot_id=a.boot_id, motor_power_epoch=a.power_epoch,
            expected_uids_sha256=hashlib.sha256(source).hexdigest(), native_build=provenance)
        with os.fdopen(os.open(out/"report.json", os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), "w") as f:
            json.dump(report, f, ensure_ascii=False, allow_nan=False); f.write("\n")
    print(json.dumps({key: report.get(key) for key in
        ("status", "errors", "condition", "cycles_completed", "deadline20ms_misses",
         "motor_enable_attempted", "stop_confirmed")}, ensure_ascii=False))
    return 0 if report["status"] == "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC" else 2


if __name__ == "__main__":
    raise SystemExit(main())
