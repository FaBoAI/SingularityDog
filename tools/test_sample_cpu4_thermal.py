"""Offline fixture tests; never read the host's real sysfs or start a benchmark."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.sample_cpu4_thermal import (
    ProbeError,
    collect_samples,
    discover_sources,
    main,
)


def fake_sysfs(root: Path) -> Path:
    sysfs = root / "sys"
    cpufreq = sysfs / "devices/system/cpu/cpu4/cpufreq"
    cpufreq.mkdir(parents=True)
    (cpufreq / "cpuinfo_cur_freq").write_text("1728000\n", encoding="ascii")
    (cpufreq / "scaling_cur_freq").write_text("1497600\n", encoding="ascii")
    for index, (kind, value) in enumerate((("CPU-therm", 42125), ("SOC0-therm", 39750))):
        zone = sysfs / "class/thermal" / f"thermal_zone{index}"
        zone.mkdir(parents=True)
        (zone / "type").write_text(kind + "\n", encoding="ascii")
        (zone / "temp").write_text(f"{value}\n", encoding="ascii")
    return sysfs


class Clock:
    def __init__(self) -> None:
        self.now = 100_000_000_000

    def read(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += round(seconds * 1e9)


class Cpu4ThermalSidecarTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sysfs = fake_sysfs(self.root)
        self.boot_id = self.root / "boot_id"
        self.boot_id.write_text("12345678-1234-1234-1234-123456789abc\n", encoding="ascii")

    def test_source_priority_fallback_and_thermal_identity(self) -> None:
        sources = discover_sources(self.sysfs)
        self.assertEqual(sources.frequency_kind, "cpuinfo_cur_freq")
        self.assertEqual([(z.name, z.kind) for z in sources.zones], [
            ("thermal_zone0", "CPU-therm"), ("thermal_zone1", "SOC0-therm")
        ])
        sources.frequency_path.unlink()
        fallback = discover_sources(self.sysfs)
        self.assertEqual(fallback.frequency_kind, "scaling_cur_freq")
        (self.sysfs / "devices/system/cpu/cpu4/cpufreq/scaling_cur_freq").unlink()
        with self.assertRaisesRegex(ProbeError, "No readable CPU4"):
            discover_sources(self.sysfs)

    def test_unreadable_zone_is_retained_when_another_zone_is_usable(self) -> None:
        zone = self.sysfs / "class/thermal/thermal_zone1/temp"
        zone.unlink()
        sources = discover_sources(self.sysfs)
        self.assertEqual(len(sources.zones), 2)
        self.assertEqual(sources.zones[1].name, "thermal_zone1")
        (self.sysfs / "class/thermal/thermal_zone0/temp").unlink()
        with self.assertRaisesRegex(ProbeError, "No readable thermal"):
            discover_sources(self.sysfs)

    def test_timestamped_samples_and_missing_read_are_recorded(self) -> None:
        clock = Clock()
        sources = discover_sources(self.sysfs)
        reads = 0

        def sleep(seconds: float) -> None:
            nonlocal reads
            clock.sleep(seconds)
            reads += 1
            if reads == 1:
                # A zone can disappear after discovery; keep the timeline.
                sources.zones[1].path.unlink()

        started, ended, reason, rows = collect_samples(
            sources, duration_ns=25_000_000, period_ns=10_000_000,
            thermal_period_ns=20_000_000, clock_ns=clock.read, sleep=sleep,
        )
        frequency = [row for row in rows if row["kind"] == "frequency"]
        thermal = [row for row in rows if row["kind"] == "thermal"]
        self.assertEqual((started, ended, reason), (100_000_000_000, 100_025_000_000, "duration_elapsed"))
        self.assertEqual([row["scheduled_ns"] - started for row in frequency],
                         [0, 10_000_000, 20_000_000])
        self.assertEqual([row["cpu4_khz"] for row in frequency], [1728000] * 3)
        self.assertEqual(len(thermal), 2)
        self.assertEqual(thermal[0]["zones"][0]["millidegrees_c"], 42125)
        self.assertIsNone(thermal[1]["zones"][1]["millidegrees_c"])
        self.assertIn("Cannot read integer", thermal[1]["zones"][1]["error"])
        self.assertTrue(all(row["read_start_ns"] <= row["read_end_ns"] for row in frequency))

    def test_cli_writes_new_file_only_and_preserves_fake_sysfs(self) -> None:
        output = self.root / "sidecar.jsonl"
        before = {
            path.relative_to(self.sysfs): path.read_bytes()
            for path in self.sysfs.rglob("*") if path.is_file()
        }
        args = ["--sysfs-root", str(self.sysfs), "--boot-id-file", str(self.boot_id),
                "--output", str(output),
                "--duration-s", "0.025", "--period-ms", "10",
                "--thermal-period-ms", "20"]
        self.assertEqual(main(args), 0)
        lines = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(lines[0]["schema"], "jetson-cpu4-thermal-v1")
        self.assertEqual(lines[0]["boot_id"], "12345678-1234-1234-1234-123456789abc")
        self.assertEqual(lines[0]["frequency_kind"], "cpuinfo_cur_freq")
        self.assertEqual(lines[-1]["reason"], "duration_elapsed")
        self.assertGreaterEqual(lines[-1]["frequency_samples"], 2)
        self.assertEqual(output.stat().st_mode & 0o777, 0o600)
        self.assertEqual(before, {
            path.relative_to(self.sysfs): path.read_bytes()
            for path in self.sysfs.rglob("*") if path.is_file()
        })
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main(args)
        self.assertEqual(lines, [json.loads(line) for line in output.read_text().splitlines()])

    def test_cli_rejects_missing_frequency_before_creating_output(self) -> None:
        for path in (self.sysfs / "devices/system/cpu/cpu4/cpufreq").iterdir():
            path.unlink()
        output = self.root / "missing.jsonl"
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main(["--sysfs-root", str(self.sysfs), "--boot-id-file", str(self.boot_id),
                  "--output", str(output)])
        self.assertFalse(output.exists())

    def test_late_wakeup_skips_probe_slots_and_signal_stops(self) -> None:
        clock = Clock()
        sources = discover_sources(self.sysfs)
        def late_sleep(_seconds: float) -> None:
            clock.now += 35_000_000

        def stop_after_two_samples() -> bool:
            return clock.now >= 100_070_000_000

        started, ended, reason, rows = collect_samples(
            sources, duration_ns=100_000_000, period_ns=10_000_000,
            thermal_period_ns=100_000_000, clock_ns=clock.read,
            sleep=late_sleep, interrupted=stop_after_two_samples,
        )
        frequency = [row for row in rows if row["kind"] == "frequency"]
        self.assertEqual([row["scheduled_ns"] - started for row in frequency], [0, 10_000_000])
        self.assertEqual((ended, reason), (100_070_000_000, "signal"))


if __name__ == "__main__":
    unittest.main()
