"""Live, read-only comparison with the private box-supported 12-axis pose.

Only Type0 identity and Type17 position requests are sent on the two USB2CAN
buses. The display is a positioning aid for a continuously human-supported
robot. It cannot verify STOP state, physical clearance, or active-hold admission.
No motor command or automatic retry exists in this module.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

from .can_readonly import ReadOnlyCAN, read_request
from .can_timing_probe import ownership_locks, validate_uids
from . import dual_can_pipeline_benchmark as dual
from .fixed_stance_readonly_capture import (
    IDS_BY_BUS, SCHEMA as CAPTURE_SCHEMA, _checked_query, capture_identities,
)

DEFAULT_BASELINE = Path(
    "/home/jetson/singularitydog-tests/load-transfer-2s-partial-r5/"
    "evidence/box-baseline-summary.json"
)
EXPECTED_BASELINE_SHA256 = "7ca29605e7b142d8a49e8ca1431731c29b1df5d476176f1b7b80bb00b071bc6b"
LEGS = {"FR": (1, 2, 3), "FL": (4, 5, 6),
        "RR": (7, 8, 9), "RL": (10, 11, 12)}
ALL_IDS = set(range(1, 13))
MAX_SECONDS = 90.0
POSITION_PERIOD_S = 0.4
OBSERVATION_LIMIT_DEG = 3.0


class WindowEnded(Exception):
    """The bounded display window expired before another complete sweep."""


def load_baseline(path, *, expected_sha256=None):
    """Return only private data needed in memory; never print it or persist it."""
    if expected_sha256 is None:
        expected_sha256 = EXPECTED_BASELINE_SHA256
    source = Path(path).read_bytes()
    if hashlib.sha256(source).hexdigest() != expected_sha256:
        raise ValueError("Box baseline SHA-256 differs from the reviewed capture")
    data = json.loads(source)
    if (type(data) is not dict or data.get("schema") != CAPTURE_SCHEMA
            or data.get("status") != "RECORDED_REVIEW_REQUIRED"
            or data.get("errors") != []):
        raise ValueError("Box baseline is not a complete read-only capture")
    boot = data.get("boot_id")
    if type(boot) is not str or not boot:
        raise ValueError("Box baseline has no boot identity")
    plan = data.get("plan")
    ports = plan.get("ports") if type(plan) is dict else None
    if (type(ports) is not dict or set(ports) != set(IDS_BY_BUS)
            or any(type(port) is not str for port in ports.values())
            or plan.get("ids_by_bus") != {bus: list(ids) for bus, ids in IDS_BY_BUS.items()}):
        raise ValueError("Box baseline has no exact two-bus assignment")
    identities = data.get("identities")
    if type(identities) is not dict or set(identities) != {str(i) for i in ALL_IDS}:
        raise ValueError("Box baseline lacks twelve identities")
    expected = validate_uids({mid: row.get("mcu_uid_hex") if type(row) is dict else None
                              for mid, row in identities.items()})
    pose = data.get("pose")
    if (type(pose) is not dict or pose.get("sampling_issues") != []
            or pose.get("sampling_stability_heuristic_passed") is not True):
        raise ValueError("Box baseline pose did not pass capture stability checks")
    raw = pose.get("raw_rad_by_id")
    if type(raw) is not dict or set(raw) != {str(i) for i in ALL_IDS}:
        raise ValueError("Box baseline lacks twelve raw positions")
    if any(type(value) not in (int, float) or not math.isfinite(value)
           for value in raw.values()):
        raise ValueError("Box baseline has invalid raw positions")
    return {"boot_id": boot, "ports": ports, "uids": expected,
            "raw_rad_by_id": {int(mid): value for mid, value in raw.items()}}


def guard_event(bus, event):
    """Reject any outgoing request other than the exact two read-only frames."""
    if event.get("kind") == "can_tx":
        mid, parameter = event.get("motor_id"), event.get("parameter")
        if (type(mid) is not int or mid not in IDS_BY_BUS[bus]
                or parameter not in ("identity", "position")):
            raise RuntimeError("Unexpected CAN transmission in alignment observer")
        expected = read_request(mid, None if parameter == "identity" else "position")
        if event.get("hex") != expected.hex():
            raise RuntimeError("Noncanonical CAN transmission in alignment observer")
    elif event.get("kind") == "can_rx_frame":
        source = event.get("source_id")
        if type(source) is int and source in ALL_IDS and source not in IDS_BY_BUS[bus]:
            raise RuntimeError("Motor replied on the wrong USB2CAN bus")
    elif event.get("kind") == "motor_feedback":
        if (event.get("type") == 21 or event.get("fault_bits", 0)
                or event.get("mode_state", 0) == 2):
            raise RuntimeError("Enabled or fault feedback during read-only alignment")


def collect_positions(cans, check):
    """Read one fresh Type17 position for every axis, concurrently by bus."""
    if type(cans) is not dict or set(cans) != set(IDS_BY_BUS) or cans["front"] is cans["rear"]:
        raise ValueError("Two separate CAN owners are required")
    abort = threading.Event()
    failed = []
    failed_lock = threading.Lock()

    def work(bus):
        rows = {}
        try:
            for mid in IDS_BY_BUS[bus]:
                if abort.is_set():
                    raise RuntimeError("Peer bus position read failed")
                def checked():
                    if abort.is_set():
                        raise RuntimeError("Peer bus position read failed")
                    check()
                reply = _checked_query(cans[bus], bus, mid, "position", checked)
                rows[mid] = reply["value"]
        except BaseException as error:
            with failed_lock:
                if not failed:
                    failed.append(error)
            abort.set()
            raise
        return rows

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {bus: pool.submit(work, bus) for bus in IDS_BY_BUS}
        readings = {}
        for future in futures.values():
            try:
                readings.update(future.result())
            except BaseException:
                pass
    if failed:
        raise failed[0]
    if set(readings) != ALL_IDS:
        raise RuntimeError("Incomplete twelve-axis sweep")
    return readings


def signed_deltas_deg(readings, baseline):
    if set(readings) != ALL_IDS or set(baseline) != ALL_IDS:
        raise ValueError("Twelve positions are required")
    if any(type(value) not in (int, float) or not math.isfinite(value)
           for value in (*readings.values(), *baseline.values())):
        raise ValueError("Nonfinite position")
    return {mid: math.degrees(readings[mid] - baseline[mid]) for mid in sorted(ALL_IDS)}


def format_screen(deltas, elapsed_s, sweep, best_max_abs_deg, rate_hz):
    if set(deltas) != ALL_IDS:
        raise ValueError("Incomplete display sweep")
    minimum = min(deltas, key=deltas.get)
    maximum = max(deltas, key=deltas.get)
    largest = max(deltas, key=lambda mid: abs(deltas[mid]))
    max_abs = abs(deltas[largest])
    lines = [f"箱上姿勢との差（Type17、度） {elapsed_s:.1f}s / 最大90s  周期{sweep}  実測{rate_hz:.1f}Hz",
             "q または Ctrl+C で終了。胴体を支え続けてください。"]
    for leg, ids in LEGS.items():
        lines.append(f"{leg}  " + "  ".join(f"ID{mid:02d} {deltas[mid]:+6.2f}°" for mid in ids))
    lines.append(f"最小Δ ID{minimum:02d} {deltas[minimum]:+.2f}°  "
                 f"最大Δ ID{maximum:02d} {deltas[maximum]:+.2f}°")
    lines.append(f"現在 最大|Δ| ID{largest:02d} {max_abs:.2f}°  "
                 f"瞬間最良 {best_max_abs_deg:.2f}°  "
                 f"逐次読取り全軸±3°: {'はい' if max_abs <= OBSERVATION_LIMIT_DEG else 'いいえ'}")
    lines.append("各軸は順番に読取り。位置合わせの目安であり、駆動試験の開始判定ではありません。")
    return "\n".join(lines)


@contextmanager
def _single_key_terminal():
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield fd
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def _quit_key(fd, timeout_s):
    if select.select([fd], [], [], max(0.0, timeout_s))[0]:
        return os.read(fd, 1).lower() == b"q"
    return False


def _boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def private_summary_path(path):
    """Require a fresh path outside every Git checkout before touching hardware."""
    requested = path.expanduser()
    if requested.is_symlink():
        raise ValueError("Summary path is a symlink")
    output = requested.resolve()
    if not output.parent.is_dir():
        raise ValueError("Summary parent directory does not exist")
    if output.exists() or output.is_symlink():
        raise ValueError("Summary path already exists")
    if any((parent / ".git").exists() for parent in (output.parent, *output.parents)):
        raise ValueError("Private alignment summary must be outside Git")
    return output


def write_private_summary(path, status, sweeps, last_deltas, best_deltas):
    """Create a 0600 delta-only record; never persist baseline or read replies."""
    if (last_deltas is not None and set(last_deltas) != ALL_IDS
            or best_deltas is not None and set(best_deltas) != ALL_IDS):
        raise ValueError("Cannot persist an incomplete position sweep")
    payload = {"status": status, "complete_sweeps": sweeps,
               "last_delta_deg_by_id": ({str(mid): last_deltas[mid] for mid in sorted(ALL_IDS)}
                                        if last_deltas is not None else None),
               "best_sweep_delta_deg_by_id": ({str(mid): best_deltas[mid] for mid in sorted(ALL_IDS)}
                                               if best_deltas is not None else None),
               "last_max_abs_delta_deg": (max(map(abs, last_deltas.values()))
                                          if last_deltas is not None else None),
               "best_max_abs_delta_deg": (max(map(abs, best_deltas.values()))
                                          if best_deltas is not None else None)}
    serialized = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(serialized)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    ap.add_argument("--seconds", type=float, default=MAX_SECONDS)
    ap.add_argument("--period-s", type=float, default=POSITION_PERIOD_S)
    ap.add_argument("--summary", type=Path,
                    help="Fresh private JSON path outside Git; omitted by default")
    ap.add_argument("--execute-readonly", action="store_true")
    args = ap.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1.0 <= args.seconds <= MAX_SECONDS:
        ap.error("--seconds must be 1..90")
    if not math.isfinite(args.period_s) or not .2 <= args.period_s <= .5:
        ap.error("--period-s must be 0.2..0.5")
    try:
        baseline = load_baseline(args.baseline)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        ap.error(str(error))
    plan = {"scope": {bus: list(ids) for bus, ids in IDS_BY_BUS.items()},
            "ports": baseline["ports"], "allowed_can_types": [0, 17],
            "parameters": ["identity", "position"], "identity_reads": 12,
            "period_s": args.period_s, "max_seconds": args.seconds,
            "automatic_retry": False, "motor_output_available": False,
            "persist_private_data": False}
    if not args.execute_readonly:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        ap.error("Run interactively on the Jetson terminal")
    try:
        summary_path = private_summary_path(args.summary) if args.summary is not None else None
    except ValueError as error:
        ap.error(str(error))
    try:
        bindings = dual.validate_ports(baseline["ports"]["front"],
                                       baseline["ports"]["rear"])
    except (OSError, ValueError) as error:
        ap.error(str(error))
    if _boot_id() != baseline["boot_id"]:
        ap.error("Jetson boot differs from the box baseline")

    cancel = threading.Event()
    handlers = {}
    def interrupted(_signum, _frame):
        cancel.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        handlers[sig] = signal.signal(sig, interrupted)
    status, last_max_abs, best_max_abs = "INCOMPLETE", None, math.inf
    last_deltas = best_deltas = None
    sweep = 0
    try:
        def check():
            if cancel.is_set():
                raise InterruptedError("Operator ended alignment")
            if _boot_id() != baseline["boot_id"]:
                raise RuntimeError("Jetson boot changed")
            if any(not dual.binding_matches(binding) for binding in bindings.values()):
                raise RuntimeError("USB2CAN port binding changed")
            if deadline is not None and time.monotonic() >= deadline:
                raise WindowEnded()

        deadline = None
        with ExitStack() as stack:
            stack.enter_context(ownership_locks())
            for bus in IDS_BY_BUS:
                stack.enter_context(dual.port_lock(bindings[bus]["resolved"]))
            cans = {}
            for bus in IDS_BY_BUS:
                can = stack.enter_context(ReadOnlyCAN(
                    port=bindings[bus]["path"],
                    event_sink=lambda event, b=bus: guard_event(b, event)))
                if os.fstat(can.serial.fileno()).st_rdev != bindings[bus]["st_rdev"]:
                    raise RuntimeError("Opened USB2CAN differs from verified binding")
                cans[bus] = can
            check()
            capture_identities(cans, baseline["uids"], check)
            print("12軸UID確認完了。位置差を表示します。", flush=True)
            with _single_key_terminal() as fd:
                started = time.monotonic()
                deadline = started + args.seconds
                first_completed = None
                while True:
                    if _quit_key(fd, 0):
                        status = "OPERATOR_EXIT"
                        break
                    try:
                        check()
                        readings = collect_positions(cans, check)
                    except WindowEnded:
                        status = "TIME_LIMIT"
                        break
                    deltas = signed_deltas_deg(readings, baseline["raw_rad_by_id"])
                    sweep += 1
                    last_deltas = deltas
                    last_max_abs = max(abs(delta) for delta in deltas.values())
                    if last_max_abs < best_max_abs:
                        best_max_abs = last_max_abs
                        best_deltas = deltas
                    completed = time.monotonic()
                    if first_completed is None:
                        first_completed = completed
                    elapsed = completed - started
                    rate_hz = ((sweep - 1) / (completed - first_completed)
                               if completed > first_completed else 0.0)
                    print("\033[H\033[2J" + format_screen(
                        deltas, elapsed, sweep, best_max_abs, rate_hz), flush=True)
                    if _quit_key(fd, max(0.0, min(args.period_s * sweep - elapsed,
                                                    deadline - time.monotonic()))):
                        status = "OPERATOR_EXIT"
                        break
    except InterruptedError:
        status = "OPERATOR_EXIT"
    except BaseException as error:
        print(f"位置表示を中止: {error}", file=sys.stderr, flush=True)
        status = "ABORTED"
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if summary_path is not None:
        try:
            write_private_summary(summary_path, status, sweep, last_deltas, best_deltas)
            print(f"差分記録: {summary_path}", flush=True)
        except (OSError, ValueError) as error:
            print(f"差分記録を保存できませんでした: {error}", file=sys.stderr, flush=True)
            status = "ABORTED"
    print(f"終了: {status}; 最終 最大|Δ|="
          f"{last_max_abs:.2f}°" if last_max_abs is not None else f"終了: {status}; 有効な全軸読取りなし",
          flush=True)
    if last_max_abs is not None:
        print(f"瞬間最良 最大|Δ|={best_max_abs:.2f}°; Type17参考値。全支持を維持してください。", flush=True)
    return 0 if status in ("OPERATOR_EXIT", "TIME_LIMIT") else 1


if __name__ == "__main__":
    raise SystemExit(main())
