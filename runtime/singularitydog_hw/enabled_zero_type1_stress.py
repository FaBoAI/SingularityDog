"""Finite supported Type1 density diagnostic with enabled RS05s and zero gains.

This diagnostic never sends a positive gain, feedforward torque, or learned
target. The torso must stay on a support. A missing reply prevents further
Type1 traffic and may require physical motor-power cutoff.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from . import disabled_type1_stress as disabled
from . import watchdog_commissioning as watchdog
from .motor_version_probe import validate_uids


MAX_CYCLES = 500
PERIOD_NS = 20_000_000
DIAGNOSTIC_DEADLINE_NS = 25_000_000
TOTAL_NS = 25_000_000_000
EVENT_TAIL = 1024


class EvidenceChannel(watchdog.Channel):
    """Keep failure-adjacent raw traffic and exact totals without an event cap."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.events = deque(maxlen=EVENT_TAIL)
        self.event_total = 0
        self.tx_steps = Counter()
        self.rx_bytes = 0
        self.rejected_rx_bytes = 0

    def event(self, value):
        self.event_total += 1
        if value.get("kind") == "tx":
            self.tx_steps[value.get("step")] += 1
        elif value.get("kind") == "rx_bytes":
            self.rx_bytes += len(bytes.fromhex(value["hex"]))
        elif value.get("kind") == "rx_rejected":
            self.rejected_rx_bytes += len(bytes.fromhex(value["hex"]))
        self.events.append(value)

    def evidence(self):
        return {"event_total": self.event_total,
                "event_tail_count": len(self.events),
                "event_tail_truncated": self.event_total > len(self.events),
                "tx_steps": dict(self.tx_steps), "accepted_rx_bytes": self.rx_bytes,
                "rejected_rx_bytes": self.rejected_rx_bytes,
                "rx_bytes": self.rx_bytes+self.rejected_rx_bytes,
                "event_tail": list(self.events)}


def plan(cycles: int) -> dict:
    if type(cycles) is not int or not 1 <= cycles <= MAX_CYCLES:
        raise ValueError("Enabled zero-gain diagnostic is limited to 500 cycles")
    return {"schema": "enabled-zero-type1-stress-v1", "status": "PLAN_ONLY",
            "cycles": cycles, "period_ms": 20, "diagnostic_reply_deadline_ms": 25,
            "type1_requests_per_cycle": 24, "voltage_requests_per_cycle": 2,
            "request_gap_us": 800, "request_window": 3,
            "kp": 0, "kd": 0, "feedforward_torque_nm": 0,
            "motor_enable_available": True, "positive_gain_available": False,
            "learned_targets_available": False, "hardware_opened": False}


def run(channels, expected_uids, *, cycles: int, announce=lambda: None,
        check=lambda: None, clock=time.monotonic_ns, wait=time.sleep) -> dict:
    result = {**plan(cycles), "status": "ABORTED", "errors": [],
              "cycles_completed": 0, "motor_enable_attempted": False,
              "positive_gain_sent": False, "learned_targets_sent": False,
              "per_cycle": [], "stop_confirmed": False}
    expected = validate_uids(expected_uids)
    if set(channels) != set(watchdog.BUSES) or channels["front"] is channels["rear"]:
        raise ValueError("Exactly two independent bus owners are required")
    centers = {}
    started = clock()

    def guard():
        check()
        if clock() - started >= TOTAL_NS:
            raise TimeoutError("Enabled zero-gain diagnostic exceeded 25 seconds")

    def active_batch(owner, ids, cycle, deadline):
        disabled._zero_batch(owner, ids, centers, guard, deadline, expected_mode=2)
        guard()
        volts = owner.exchange(ids[cycle % len(ids)], "voltage")["value"]
        if not 35 <= volts <= 42:
            raise RuntimeError("Rotating voltage outside 35..42V")
        disabled._zero_batch(owner, ids, centers, guard, deadline, expected_mode=2)

    try:
        # Qualify all twelve before any Enable. A zero-gain Type1 must remain
        # disabled at this stage; the 200ms firmware timeout is read back.
        for scope, ids in watchdog.BUSES.items():
            owner = channels[scope]
            for mid in ids:
                guard()
                if owner.exchange(mid, "identity")["mcu_uid_hex"] != expected[mid]:
                    raise ValueError(f"ID{mid} UID mismatch")
                stopped = owner.exchange(mid, "stop")
                if stopped["mode_state"] != 0 or stopped["fault_bits"] != 0:
                    raise RuntimeError(f"ID{mid} initial STOP unconfirmed")
                if owner.exchange(mid, "run_mode")["value"] != 0:
                    raise RuntimeError(f"ID{mid} MIT mode not confirmed")
                volts = owner.exchange(mid, "voltage")["value"]
                if not 35 <= volts <= 42:
                    raise RuntimeError(f"ID{mid} voltage outside 35..42V")
                center = stopped["protocol_position_rad"]
                if not math.isfinite(center):
                    raise RuntimeError(f"ID{mid} invalid position")
                centers[mid] = center
                owner.exchange(mid, "watchdog_write")
                if owner.exchange(mid, "can_timeout")["value"] != 4000:
                    raise RuntimeError(f"ID{mid} watchdog readback mismatch")
                disabled._check_zero(owner.exchange(mid, "zero", center=center), mid, center)
        guard(); announce(); guard()

        # The final zero-gain reply must prove mode2 before each axis joins the
        # density stream. No active cycle can run on an unconfirmed Enable.
        for scope, ids in watchdog.BUSES.items():
            owner = channels[scope]
            for mid in ids:
                guard()
                result["motor_enable_attempted"] = True
                enabled = owner.exchange(mid, "enable")
                if enabled["mode_state"] not in (0, 2) or enabled["fault_bits"] != 0:
                    raise RuntimeError(f"ID{mid} Enable rejected")
                disabled._check_zero(owner.exchange(mid, "zero", center=centers[mid]),
                                     mid, centers[mid], expected_mode=2)

        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="enabled-zero") as pool:
            next_release = clock()
            for cycle in range(cycles):
                guard()
                while clock() < next_release:
                    guard()
                    wait(min(.001, max(0., (next_release-clock())/1e9)))
                begin = clock()
                futures = {scope: pool.submit(active_batch, channels[scope], ids, cycle,
                                               begin+DIAGNOSTIC_DEADLINE_NS)
                           for scope, ids in watchdog.BUSES.items()}
                failures = []
                for scope, future in futures.items():
                    try: future.result()
                    except BaseException as error:
                        failures.append(f"{scope}: {type(error).__name__}: {error}")
                end = clock()
                result["per_cycle"].append({"cycle": cycle+1, "begin_ns": begin,
                    "end_ns": end, "elapsed_ms": (end-begin)/1e6,
                    "release_lateness_ms": max(0, begin-next_release)/1e6})
                if failures:
                    raise RuntimeError("; ".join(failures))
                result["cycles_completed"] = cycle+1
                next_release = max(next_release+PERIOD_NS, end)
        result["status"] = "COMPLETE_ENABLED_ZERO_TYPE1_DIAGNOSTIC"
    except BaseException as error:
        result["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        stops = {}
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="enabled-stop") as pool:
            futures = {scope: pool.submit(owner.stop_all) for scope, owner in channels.items()}
            for scope, future in futures.items():
                try: stops[scope] = future.result()
                except BaseException as error:
                    stops[scope] = {"complete": False,
                        "unconfirmed_ids": list(watchdog.BUSES[scope]),
                        "errors": [f"{type(error).__name__}: {error}"]}
        result["stop_reports"] = stops
        result["stop_confirmed"] = all(stops[s].get("complete") is True and
            not stops[s].get("unconfirmed_ids") and not stops[s].get("errors")
            for s in watchdog.BUSES)
        if not result["stop_confirmed"]:
            result["status"] = "STOP_UNCONFIRMED_POWER_OFF_REQUIRED"
            result["errors"].append("Physically switch motor power Off")
        result["elapsed_s"] = (clock()-started)/1e9
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cycles", type=int, default=5)
    p.add_argument("--execute-supported-zero-gain", action="store_true")
    for name in ("support-in-place", "cutoff-ready", "rs05-model-confirmed"):
        p.add_argument("--"+name, action="store_true")
    for name in ("front-port", "rear-port", "expected-uids", "boot-id", "power-epoch",
                 "output", "audio", "audio-sha256", "audio-device"):
        p.add_argument("--"+name)
    a = p.parse_args(argv)
    intended = plan(a.cycles)
    if not a.execute_supported_zero_gain:
        print(json.dumps(intended, indent=2)); return 0
    if not all((a.support_in_place, a.cutoff_ready, a.rs05_model_confirmed)):
        p.error("Current support, physical cutoff and RS05 confirmation required")
    required = ("front_port", "rear_port", "expected_uids", "boot_id", "power_epoch",
                "output", "audio", "audio_sha256", "audio_device")
    if any(not getattr(a, name) for name in required):
        p.error("All explicit hardware, epoch, output and announcement inputs required")
    audio = Path(a.audio).resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest() != a.audio_sha256:
        p.error("Audio file hash mismatch")
    out = Path(a.output).expanduser().resolve()
    if any((parent/".git").exists() for parent in (out, *out.parents)):
        p.error("Raw output must stay outside Git")
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    from . import dual_can_pipeline_benchmark as dual
    from .sensor_pipeline_benchmark import BootIdentityGuard
    source = Path(a.expected_uids).read_bytes()
    expected = validate_uids(json.loads(source))
    channels, handlers, cancelled = {}, {}, []
    report = {**intended, "status": "ABORTED_BEFORE_OPEN", "errors": [],
              "motor_enable_attempted": False, "positive_gain_sent": False,
              "learned_targets_sent": False}
    try:
        if Path("/proc/sys/kernel/random/boot_id").read_text().strip() != a.boot_id:
            raise RuntimeError("Jetson boot changed")
        bindings = dual.validate_ports(a.front_port, a.rear_port)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: cancelled.append(number))
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks())
            boot = BootIdentityGuard(); stack.callback(boot.close)
            if boot.boot_id != a.boot_id:
                raise RuntimeError("Jetson boot changed")
            def check():
                if cancelled:
                    raise InterruptedError("Operator interrupted enabled-zero diagnostic")
                boot.check()
            import serial
            for scope, binding in bindings.items():
                stack.enter_context(dual.port_lock(binding["resolved"]))
                port = serial.Serial(port=None, baudrate=921600, timeout=0,
                                     write_timeout=.02, exclusive=True)
                port.dtr = port.rts = False
                port.port = binding["path"]; port.open(); stack.callback(port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev != binding["st_rdev"]:
                    raise RuntimeError("USB binding changed")
                channels[scope] = EvidenceChannel(port, watchdog.BUSES[scope], check=check)
            def announce():
                check()
                subprocess.run(["aplay", "-D", a.audio_device, str(audio)], check=True,
                    timeout=8., stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE)
            report = run(channels, expected, cycles=a.cycles, announce=announce, check=check)
            report.update(hardware_opened=True, boot_id=boot.boot_id, motor_power_epoch=a.power_epoch,
                expected_uids_sha256=hashlib.sha256(source).hexdigest(),
                evidence_by_bus={scope: channel.evidence() for scope, channel in channels.items()})
    except BaseException as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        for sig, handler in handlers.items(): signal.signal(sig, handler)
        with os.fdopen(os.open(out/"report.json", os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), "w") as f:
            json.dump(report, f, ensure_ascii=False, allow_nan=False); f.write("\n")
    print(json.dumps({key: report.get(key) for key in
        ("status", "errors", "cycles_completed", "motor_enable_attempted",
         "stop_confirmed")}, ensure_ascii=False))
    return 0 if report["status"] == "COMPLETE_ENABLED_ZERO_TYPE1_DIAGNOSTIC" else 2


if __name__ == "__main__":
    raise SystemExit(main())
