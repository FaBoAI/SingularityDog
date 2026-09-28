"""File-only tests: never touch the host's real cpufreq sysfs."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from tools.jetson_cpu_performance_scope import (
    CpuPerformanceScope,
    CpuScopeError,
    parse_cpu_list,
    run_command,
)


def create_fake_sysfs(base: Path) -> Path:
    root = base / "cpufreq"
    for name, cpus, minimum, maximum in (
        ("policy0", "0 1 2 3", 729600, 1500000),
        ("policy4", "4 5", 918000, 2100000),
    ):
        policy = root / name
        policy.mkdir(parents=True)
        (policy / "related_cpus").write_text(cpus + "\n", encoding="ascii")
        (policy / "scaling_min_freq").write_text(f"{minimum}\n", encoding="ascii")
        (policy / "scaling_max_freq").write_text(f"{maximum}\n", encoding="ascii")
        # Linux also exposes one symlink per CPU.  A correct implementation
        # must not snapshot/write through these six aliases.
        for cpu in parse_cpu_list(cpus):
            link = base / f"cpu{cpu}" / "cpufreq"
            link.parent.mkdir()
            link.symlink_to(policy, target_is_directory=True)
    return root


def read_min(root: Path, name: str) -> int:
    return int((root / name / "scaling_min_freq").read_text(encoding="ascii"))


class CpuScopeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = create_fake_sysfs(self.base)
        self.lock = self.base / "scope.lock"

    def scope(self, **kwargs: object) -> CpuPerformanceScope:
        settings: dict[str, object] = {
            "boost_timeout_s": 0.3,
            "restore_timeout_s": 0.8,
            "restore_quiet_s": 0.2,
            "poll_s": 0.01,
            "max_restore_writes": 8,
        }
        settings.update(kwargs)
        return CpuPerformanceScope(self.root, self.lock, frozenset(range(6)), **settings)

    def test_cpu_lists_and_invalid_overlap(self) -> None:
        self.assertEqual(parse_cpu_list("0-3,4 5"), frozenset(range(6)))
        with self.assertRaises(CpuScopeError):
            parse_cpu_list("0-3 3")
        (self.root / "policy4" / "related_cpus").write_text("3 4 5\n", encoding="ascii")
        with self.assertRaisesRegex(CpuScopeError, "multiple policies"):
            with self.scope():
                self.fail("scope should not start")

    def test_shared_policies_are_saved_once_and_restored_exactly(self) -> None:
        writes: list[tuple[str, int]] = []

        def writer(path: Path, value: int) -> None:
            writes.append((path.parent.name, value))
            path.write_text(f"{value}\n", encoding="ascii")

        scope = self.scope(writer=writer)
        with scope:
            self.assertEqual(read_min(self.root, "policy0"), 1500000)
            self.assertEqual(read_min(self.root, "policy4"), 2100000)
            self.assertEqual(len(scope.snapshots), 2)
            self.assertEqual([s.original_min for s in scope.snapshots], [729600, 918000])
        self.assertTrue(scope.restored)
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)
        self.assertEqual(writes, [
            ("policy0", 1500000), ("policy4", 2100000),
            ("policy4", 918000), ("policy4", 918000),
            ("policy0", 729600), ("policy0", 729600),
        ])

    def test_delayed_boost_does_not_outlive_failed_setup_rollback(self) -> None:
        """Reproduces the Jetson stale read that left policy0 boosted."""
        timers: list[threading.Timer] = []
        writes: list[tuple[str, int]] = []

        def writer(path: Path, value: int) -> None:
            writes.append((path.parent.name, value))
            if path.parent.name == "policy0" and value == 1500000:
                # Defer the accepted boost until after rollback has begun.
                return
            path.write_text(f"{value}\n", encoding="ascii")
            if (path.parent.name == "policy0" and value == 729600
                    and writes.count(("policy0", 729600)) == 1):
                timer = threading.Timer(
                    0.05, lambda: path.write_text("1500000\n", encoding="ascii")
                )
                timers.append(timer)
                timer.start()

        marker = self.base / "child-must-not-run"
        try:
            with self.assertRaisesRegex(CpuScopeError, "Boost verification timed out"):
                run_command(
                    [sys.executable, "-c", f"from pathlib import Path; Path({str(marker)!r}).touch()"],
                    self.scope(writer=writer, boost_timeout_s=0.04,
                               restore_timeout_s=0.7, restore_quiet_s=0.2),
                )
        finally:
            for timer in timers:
                timer.join(timeout=1)
        self.assertFalse(marker.exists())
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)
        self.assertGreaterEqual(writes.count(("policy0", 729600)), 3)

    def test_delayed_boost_within_bound_is_verified_before_child(self) -> None:
        timers: list[threading.Timer] = []

        def writer(path: Path, value: int) -> None:
            if path.parent.name == "policy0" and value == 1500000:
                timer = threading.Timer(
                    0.07, lambda: path.write_text(f"{value}\n", encoding="ascii")
                )
                timers.append(timer)
                timer.start()
            else:
                path.write_text(f"{value}\n", encoding="ascii")

        scope = self.scope(writer=writer)
        try:
            with scope:
                self.assertEqual(read_min(self.root, "policy0"), 1500000)
                self.assertEqual(read_min(self.root, "policy4"), 2100000)
        finally:
            for timer in timers:
                timer.join(timeout=1)
        self.assertTrue(scope.restored)
        self.assertEqual(read_min(self.root, "policy0"), 729600)

    def test_final_all_policy_check_catches_earlier_policy_rebound(self) -> None:
        timers: list[threading.Timer] = []
        writes: list[tuple[str, int]] = []

        def writer(path: Path, value: int) -> None:
            writes.append((path.parent.name, value))
            path.write_text(f"{value}\n", encoding="ascii")
            if path.parent.name == "policy4" and value == 2100000:
                # policy4 is restored first, then a stale boost lands while
                # policy0 is being restored.  Its own quiet check has passed.
                timer = threading.Timer(
                    0.28, lambda: path.write_text(f"{value}\n", encoding="ascii")
                )
                timers.append(timer)
                timer.start()

        scope = self.scope(writer=writer)
        try:
            with scope:
                self.assertEqual(read_min(self.root, "policy4"), 2100000)
        finally:
            for timer in timers:
                timer.join(timeout=1)
        self.assertTrue(scope.restored)
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)
        self.assertGreaterEqual(writes.count(("policy4", 918000)), 3)

    def test_setup_error_rolls_back_partial_change_and_never_runs_child(self) -> None:
        def writer(path: Path, value: int) -> None:
            if path.parent.name == "policy4" and value == 2100000:
                raise CpuScopeError("injected policy4 write failure")
            path.write_text(f"{value}\n", encoding="ascii")

        marker = self.base / "child-ran"
        with self.assertRaisesRegex(CpuScopeError, "injected"):
            run_command(
                [sys.executable, "-c", f"from pathlib import Path; Path({str(marker)!r}).touch()"],
                self.scope(writer=writer),
            )
        self.assertFalse(marker.exists())
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)

    def test_missing_required_cpu_fails_before_any_write(self) -> None:
        (self.root / "policy4" / "related_cpus").write_text("4\n", encoding="ascii")
        with self.assertRaisesRegex(CpuScopeError, "Required CPUs"):
            with self.scope():
                self.fail("scope should not start")
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)

    def test_exclusive_lock_fails_closed(self) -> None:
        with self.scope():
            with self.assertRaisesRegex(CpuScopeError, "exclusive CPU scope lock"):
                with self.scope():
                    self.fail("second scope should not start")
        self.assertEqual(read_min(self.root, "policy0"), 729600)

    def test_command_sees_raised_values_and_restoration_follows_exit(self) -> None:
        observed = self.base / "observed.json"
        source = (
            "import json,sys; from pathlib import Path; "
            "r=Path(sys.argv[1]); "
            "values=[int((r/n/'scaling_min_freq').read_text()) "
            "for n in ('policy0','policy4')]; "
            "Path(sys.argv[2]).write_text(json.dumps(values))"
        )
        scope = self.scope()
        code, caught = run_command(
            [sys.executable, "-c", source, str(self.root), str(observed)], scope
        )
        self.assertEqual((code, caught), (0, None))
        self.assertEqual(json.loads(observed.read_text()), [1500000, 2100000])
        self.assertTrue(scope.restored)
        self.assertEqual(read_min(self.root, "policy0"), 729600)
        self.assertEqual(read_min(self.root, "policy4"), 918000)

    def test_restoration_failure_is_reported_after_child(self) -> None:
        def writer(path: Path, value: int) -> None:
            if path.parent.name == "policy0" and value == 729600:
                raise CpuScopeError("injected restore failure")
            path.write_text(f"{value}\n", encoding="ascii")

        scope = self.scope(writer=writer)
        with self.assertRaisesRegex(CpuScopeError, "original_min=729600"):
            run_command([sys.executable, "-c", "pass"], scope)
        self.assertFalse(scope.restored)
        # Even after one restoration fails, the other policy is still restored.
        self.assertEqual(read_min(self.root, "policy0"), 1500000)
        self.assertEqual(read_min(self.root, "policy4"), 918000)

    @unittest.skipUnless(sys.platform.startswith("linux"), "orphan group signaling is restricted on macOS")
    def test_child_background_worker_is_stopped_before_restoration(self) -> None:
        marker = self.base / "background-survived"
        child = (
            "import subprocess,sys; "
            "subprocess.Popen([sys.executable,'-c',"
            "'import sys,time; from pathlib import Path; time.sleep(0.8); '"
            "+'Path(sys.argv[1]).touch()',sys.argv[1]])"
        )
        scope = self.scope()
        code, caught = run_command([sys.executable, "-c", child, str(marker)], scope)
        self.assertEqual((code, caught), (0, None))
        self.assertTrue(scope.restored)
        time.sleep(0.9)
        self.assertFalse(marker.exists())

    def test_sigterm_stops_child_before_restoring(self) -> None:
        ready = self.base / "ready"
        marker = self.base / "should-not-appear"
        child = (
            "import sys,time; from pathlib import Path; "
            "Path(sys.argv[1]).touch(); time.sleep(3); Path(sys.argv[2]).touch()"
        )
        wrapper = Path(__file__).with_name("jetson_cpu_performance_scope.py")
        process = subprocess.Popen(
            [sys.executable, str(wrapper), "--sysfs-root", str(self.root),
             "--lock-file", str(self.lock), "--", sys.executable, "-c", child,
             str(ready), str(marker)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            deadline = time.monotonic() + 5
            while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(ready.exists(), "child never started")
            self.assertEqual(read_min(self.root, "policy0"), 1500000)
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=8)
            self.assertEqual(stderr, "")
            self.assertEqual(process.returncode, 128 + signal.SIGTERM)
            result = json.loads(stdout.strip().splitlines()[-1])
            self.assertTrue(result["restored"])
            self.assertEqual(result["caught_signal"], signal.SIGTERM)
            self.assertFalse(marker.exists())
            self.assertEqual(read_min(self.root, "policy0"), 729600)
            self.assertEqual(read_min(self.root, "policy4"), 918000)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


if __name__ == "__main__":
    unittest.main()
