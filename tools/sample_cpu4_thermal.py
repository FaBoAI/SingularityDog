"""Read-only Jetson CPU4 frequency and thermal sampler for a separate process.

The native 500-cycle benchmark is never imported or launched here. Samples are
kept in memory during collection and written to a new JSONL file afterward, so
this tool does not add file I/O to the benchmark's timed loop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import signal
import time
from typing import Callable, Sequence


DEFAULT_SYSFS = Path("/sys")
DEFAULT_BOOT_ID = Path("/proc/sys/kernel/random/boot_id")
_ZONE_NAME = re.compile(r"thermal_zone[0-9]+\Z")
_BOOT_ID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\Z")


class ProbeError(RuntimeError):
    """Required read-only sensor files are unavailable or malformed."""


@dataclass(frozen=True)
class ThermalZone:
    name: str
    kind: str
    path: Path


@dataclass(frozen=True)
class Sources:
    frequency_path: Path
    frequency_kind: str
    zones: tuple[ThermalZone, ...]


def _read_integer(path: Path, *, positive: bool) -> int:
    try:
        value = int(path.read_text(encoding="ascii").strip())
    except (OSError, UnicodeError, ValueError) as exc:
        raise ProbeError(f"Cannot read integer from {path}: {exc}") from exc
    if positive and value <= 0:
        raise ProbeError(f"Nonpositive frequency at {path}: {value}")
    return value


def discover_sources(sysfs_root: Path = DEFAULT_SYSFS) -> Sources:
    """Prefer the hardware-current cpufreq attribute; label a policy fallback."""
    root = Path(sysfs_root)
    cpu4 = root / "devices/system/cpu/cpu4/cpufreq"
    failures: list[str] = []
    selected: tuple[Path, str] | None = None
    for kind in ("cpuinfo_cur_freq", "scaling_cur_freq"):
        path = cpu4 / kind
        try:
            _read_integer(path, positive=True)
        except ProbeError as exc:
            failures.append(str(exc))
        else:
            selected = (path, kind)
            break
    if selected is None:
        raise ProbeError("No readable CPU4 current frequency: " + "; ".join(failures))

    thermal_root = root / "class/thermal"
    try:
        paths = sorted(
            (p for p in thermal_root.iterdir() if _ZONE_NAME.fullmatch(p.name)),
            key=lambda p: int(p.name.removeprefix("thermal_zone")),
        )
    except OSError as exc:
        raise ProbeError(f"Cannot list thermal zones at {thermal_root}: {exc}") from exc
    if not paths or len(paths) > 128:
        raise ProbeError(f"Expected 1..128 thermal zones at {thermal_root}; found {len(paths)}")
    zones: list[ThermalZone] = []
    usable_temperatures = 0
    for zone in paths:
        try:
            kind = (zone / "type").read_text(encoding="ascii").strip()
            if not kind:
                raise ValueError("empty thermal type")
        except (OSError, UnicodeError, ValueError) as exc:
            raise ProbeError(f"Cannot discover {zone.name}: {exc}") from exc
        # Some Jetson thermal zones exist but return ENODATA until a sensor is
        # active. Keep their identity and record each read error in the trace.
        try:
            _read_integer(zone / "temp", positive=False)
        except ProbeError:
            pass
        else:
            usable_temperatures += 1
        zones.append(ThermalZone(zone.name, kind, zone / "temp"))
    if not usable_temperatures:
        raise ProbeError("No readable thermal-zone temperature")
    return Sources(selected[0], selected[1], tuple(zones))


def _sensor_read(path: Path, *, positive: bool) -> tuple[int | None, str | None]:
    try:
        return _read_integer(path, positive=positive), None
    except ProbeError as exc:
        return None, str(exc)


def read_boot_id(path: Path = DEFAULT_BOOT_ID) -> str:
    try:
        value = Path(path).read_text(encoding="ascii").strip().lower()
    except (OSError, UnicodeError) as exc:
        raise ProbeError(f"Cannot read boot ID from {path}: {exc}") from exc
    if not _BOOT_ID.fullmatch(value):
        raise ProbeError(f"Invalid boot ID at {path}")
    return value


def collect_samples(
    sources: Sources,
    *,
    duration_ns: int,
    period_ns: int,
    thermal_period_ns: int,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    interrupted: Callable[[], bool] = lambda: False,
) -> tuple[int, int, str, list[dict[str, object]]]:
    """Collect bounded samples without changing settings or writing during sampling."""
    if duration_ns <= 0 or period_ns < 5_000_000 or thermal_period_ns < period_ns:
        raise ValueError("Invalid duration or sampling period")
    if duration_ns // period_ns > 25_000:
        raise ValueError("Sampling bound exceeds 25,000 rows")
    started_ns = clock_ns()
    deadline_ns = started_ns + duration_ns
    next_frequency_ns = started_ns
    next_thermal_ns = started_ns
    rows: list[dict[str, object]] = []
    reason = "duration_elapsed"
    while True:
        if interrupted():
            reason = "signal"
            break
        now_ns = clock_ns()
        if now_ns >= deadline_ns:
            break
        if now_ns < next_frequency_ns:
            sleep((min(next_frequency_ns, deadline_ns) - now_ns) / 1e9)
            continue

        read_start_ns = clock_ns()
        frequency, error = _sensor_read(sources.frequency_path, positive=True)
        read_end_ns = clock_ns()
        rows.append({
            "kind": "frequency", "scheduled_ns": next_frequency_ns,
            "read_start_ns": read_start_ns, "read_end_ns": read_end_ns,
            "cpu4_khz": frequency, "error": error,
        })
        if read_end_ns >= next_thermal_ns:
            values: list[dict[str, object]] = []
            for zone in sources.zones:
                zone_start_ns = clock_ns()
                temperature, zone_error = _sensor_read(zone.path, positive=False)
                zone_end_ns = clock_ns()
                values.append({
                    "zone": zone.name, "millidegrees_c": temperature,
                    "read_start_ns": zone_start_ns, "read_end_ns": zone_end_ns,
                    "error": zone_error,
                })
            rows.append({"kind": "thermal", "zones": values})
            thermal_completed_ns = clock_ns()
            next_thermal_ns += max(
                1, (thermal_completed_ns - next_thermal_ns) // thermal_period_ns + 1
            ) * thermal_period_ns

        # Skip missed probe slots; never spin to catch up or affect the runner.
        completed_ns = clock_ns()
        next_frequency_ns += max(
            1, (completed_ns - next_frequency_ns) // period_ns + 1
        ) * period_ns
    return started_ns, clock_ns(), reason, rows


def write_jsonl(
    output: Path,
    sources: Sources,
    *,
    started_ns: int,
    ended_ns: int,
    reason: str,
    rows: Sequence[dict[str, object]],
    period_ns: int,
    thermal_period_ns: int,
    boot_id: str,
) -> None:
    """Create a new artifact after collection; never overwrite earlier evidence."""
    metadata = {
        "kind": "metadata", "schema": "jetson-cpu4-thermal-v1",
        "clock": "time.monotonic_ns", "boot_id": boot_id, "read_only": True,
        "frequency_path": str(sources.frequency_path),
        "frequency_kind": sources.frequency_kind,
        "frequency_scope": (
            "CPU4 hardware-current attribute if supported by driver"
            if sources.frequency_kind == "cpuinfo_cur_freq"
            else "CPU4 cpufreq policy-reported current value; may be shared with CPU5"
        ),
        "thermal_zones": [
            {"zone": zone.name, "type": zone.kind, "temp_path": str(zone.path)}
            for zone in sources.zones
        ],
        "period_ns": period_ns, "thermal_period_ns": thermal_period_ns,
        "started_ns": started_ns,
    }
    footer = {
        "kind": "end", "ended_ns": ended_ns, "reason": reason,
        "frequency_samples": sum(row["kind"] == "frequency" for row in rows),
        "thermal_samples": sum(row["kind"] == "thermal" for row in rows),
    }
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    descriptor = os.open(output, flags, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        for item in (metadata, *rows, footer):
            stream.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New JSONL file; parent directory must exist")
    parser.add_argument("--duration-s", type=float, default=30.0,
                        help="Maximum collection time; stop sooner with SIGTERM (default: 30)")
    parser.add_argument("--period-ms", type=int, default=10,
                        help="CPU4 frequency sample period, at least 5 ms (default: 10)")
    parser.add_argument("--thermal-period-ms", type=int, default=100,
                        help="Thermal sample period, at least CPU period (default: 100)")
    parser.add_argument("--sysfs-root", type=Path, default=DEFAULT_SYSFS,
                        help="Read-only sysfs root; mainly for offline fixture tests")
    parser.add_argument("--boot-id-file", type=Path, default=DEFAULT_BOOT_ID,
                        help="Read-only Linux boot ID file; mainly for offline fixture tests")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 120:
        parser.error("--duration-s must be finite and in (0, 120]")
    if not 5 <= args.period_ms <= 1000:
        parser.error("--period-ms must be in 5..1000")
    if not args.period_ms <= args.thermal_period_ms <= 5000:
        parser.error("--thermal-period-ms must be between the CPU period and 5000")
    duration_ns = round(args.duration_s * 1e9)
    period_ns = args.period_ms * 1_000_000
    if duration_ns // period_ns > 25_000:
        parser.error("At most 25,000 frequency samples are allowed")
    if os.path.lexists(args.output):
        parser.error(f"Output exists: {args.output}")
    if not args.output.parent.is_dir():
        parser.error(f"Output parent directory does not exist: {args.output.parent}")
    try:
        sources = discover_sources(args.sysfs_root)
        boot_id = read_boot_id(args.boot_id_file)
    except ProbeError as exc:
        parser.error(str(exc))

    cancelled = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal cancelled
        cancelled = True

    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for sig in old_handlers:
            signal.signal(sig, stop)
        started_ns, ended_ns, reason, rows = collect_samples(
            sources, duration_ns=duration_ns, period_ns=period_ns,
            thermal_period_ns=args.thermal_period_ms * 1_000_000,
            interrupted=lambda: cancelled,
        )
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    try:
        write_jsonl(
            args.output, sources, started_ns=started_ns, ended_ns=ended_ns,
            reason=reason, rows=rows, period_ns=period_ns,
            thermal_period_ns=args.thermal_period_ms * 1_000_000,
            boot_id=boot_id,
        )
    except OSError as exc:
        parser.error(f"Cannot write {args.output}: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
