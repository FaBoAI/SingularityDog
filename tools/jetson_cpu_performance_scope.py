"""Run one foreground command with reversible Jetson cpufreq policy minima.

This tool changes each *policy* once.  On the target Jetson, CPUs 0-3 share
policy0 and CPUs 4-5 share policy4; writing through every cpuN/cpufreq link
would overwrite the original minimum with a value already raised by this tool.

Example::

    sudo python3 tools/jetson_cpu_performance_scope.py -- \
        python3 -m singularitydog_hw.native_pipeline_benchmark --help

The child stays in the foreground.  The wrapper waits for it, terminates its
process group on interruption, and restores and verifies every original
scaling_min_freq before exiting.  It cannot recover from SIGKILL, power loss,
or a child that deliberately escapes its process group.  Restoration is
verified over a bounded quiet period; no software check can rule out a driver
write that appears only after that bound.
"""

from __future__ import annotations

import argparse
import dataclasses
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Callable, Sequence


DEFAULT_SYSFS = Path("/sys/devices/system/cpu/cpufreq")
DEFAULT_LOCK = Path("/run/lock/singularitydog-cpu-performance-scope.lock")
_POLICY_NAME = re.compile(r"policy[0-9]+\Z")


class CpuScopeError(RuntimeError):
    """The requested performance scope was not safely established or closed."""


@dataclasses.dataclass(frozen=True)
class PolicySnapshot:
    name: str
    path: Path
    related_cpus: frozenset[int]
    original_min: int
    target_max: int


def parse_cpu_list(raw: str) -> frozenset[int]:
    """Parse a Linux cpulist (including space-separated sysfs related_cpus)."""
    result: set[int] = set()
    tokens = raw.replace(",", " ").split()
    if not tokens:
        raise CpuScopeError("Empty related_cpus or required CPU list")
    for token in tokens:
        if "-" in token:
            parts = token.split("-")
            if len(parts) != 2 or not all(part.isdecimal() for part in parts):
                raise CpuScopeError(f"Invalid CPU range: {token!r}")
            first, last = (int(part) for part in parts)
            if first > last or last - first > 4096:
                raise CpuScopeError(f"Invalid CPU range: {token!r}")
            cpus = range(first, last + 1)
        elif token.isdecimal():
            cpus = (int(token),)
        else:
            raise CpuScopeError(f"Invalid CPU number: {token!r}")
        for cpu in cpus:
            if cpu in result:
                raise CpuScopeError(f"Repeated CPU in list: {cpu}")
            result.add(cpu)
    return frozenset(result)


def _read_frequency(path: Path) -> int:
    try:
        value = int(path.read_text(encoding="ascii").strip())
    except (OSError, ValueError) as exc:
        raise CpuScopeError(f"Cannot read integer frequency from {path}: {exc}") from exc
    if value <= 0:
        raise CpuScopeError(f"Nonpositive frequency at {path}: {value}")
    return value


def _write_frequency(path: Path, value: int) -> None:
    try:
        path.write_text(f"{value}\n", encoding="ascii")
    except OSError as exc:
        raise CpuScopeError(f"Cannot set {path} to {value}: {exc}") from exc


class CpuPerformanceScope:
    """Exclusive, reversible scope over unique cpufreq policy directories."""

    def __init__(
        self,
        sysfs_root: Path = DEFAULT_SYSFS,
        lock_path: Path = DEFAULT_LOCK,
        required_cpus: frozenset[int] = frozenset(range(6)),
        *,
        writer: Callable[[Path, int], None] = _write_frequency,
        boost_timeout_s: float = 2.0,
        restore_timeout_s: float = 4.0,
        restore_quiet_s: float = 1.0,
        poll_s: float = 0.02,
        max_restore_writes: int = 20,
    ) -> None:
        if not (boost_timeout_s > 0 and restore_timeout_s > restore_quiet_s > 0
                and poll_s > 0 and max_restore_writes >= 2):
            raise ValueError("Invalid CPU scope settle/restore timing")
        self.sysfs_root = Path(sysfs_root)
        self.lock_path = Path(lock_path)
        self.required_cpus = frozenset(required_cpus)
        self.writer = writer
        self.boost_timeout_s = boost_timeout_s
        self.restore_timeout_s = restore_timeout_s
        self.restore_quiet_s = restore_quiet_s
        self.poll_s = poll_s
        self.max_restore_writes = max_restore_writes
        self.snapshots: tuple[PolicySnapshot, ...] = ()
        self._lock_fd: int | None = None
        self.restored = False

    def _acquire_lock(self) -> None:
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(self.lock_path, flags, 0o600)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if "fd" in locals():
                os.close(fd)
            raise CpuScopeError(f"Cannot acquire exclusive CPU scope lock {self.lock_path}: {exc}") from exc
        self._lock_fd = fd

    def _release_lock(self) -> None:
        if self._lock_fd is not None:
            fd, self._lock_fd = self._lock_fd, None
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    def _discover(self) -> tuple[PolicySnapshot, ...]:
        if not self.required_cpus:
            raise CpuScopeError("At least one required CPU must be specified")
        try:
            candidates = sorted(
                (path for path in self.sysfs_root.iterdir() if _POLICY_NAME.fullmatch(path.name)),
                key=lambda path: int(path.name.removeprefix("policy")),
            )
        except OSError as exc:
            raise CpuScopeError(f"Cannot list cpufreq policies at {self.sysfs_root}: {exc}") from exc
        if not candidates:
            raise CpuScopeError(f"No cpufreq policies found in {self.sysfs_root}")
        snapshots: list[PolicySnapshot] = []
        covered: set[int] = set()
        for directory in candidates:
            if not directory.is_dir():
                raise CpuScopeError(f"Policy path is not a directory: {directory}")
            try:
                related = parse_cpu_list((directory / "related_cpus").read_text(encoding="ascii"))
            except OSError as exc:
                raise CpuScopeError(f"Cannot read related CPUs for {directory}: {exc}") from exc
            overlap = covered.intersection(related)
            if overlap:
                raise CpuScopeError(f"CPU belongs to multiple policies: {sorted(overlap)}")
            covered.update(related)
            original_min = _read_frequency(directory / "scaling_min_freq")
            target_max = _read_frequency(directory / "scaling_max_freq")
            if original_min > target_max:
                raise CpuScopeError(f"Policy {directory.name} has min above max")
            snapshots.append(
                PolicySnapshot(directory.name, directory, related, original_min, target_max)
            )
        missing = self.required_cpus - covered
        if missing:
            raise CpuScopeError(f"Required CPUs have no cpufreq policy: {sorted(missing)}")
        return tuple(snapshots)

    def _await_boost(self, snapshot: PolicySnapshot) -> None:
        path = snapshot.path / "scaling_min_freq"
        deadline = time.monotonic() + self.boost_timeout_s
        quiet_required = min(0.1, self.boost_timeout_s / 2)
        quiet_since: float | None = None
        observed = None
        while time.monotonic() < deadline:
            observed = _read_frequency(path)
            if _read_frequency(snapshot.path / "scaling_max_freq") != snapshot.target_max:
                raise CpuScopeError(f"Maximum changed while boosting {snapshot.name}")
            now = time.monotonic()
            if observed == snapshot.target_max:
                if quiet_since is None:
                    quiet_since = now
                elif now - quiet_since >= quiet_required:
                    return
            else:
                quiet_since = None
            time.sleep(self.poll_s)
        raise CpuScopeError(
            f"Boost verification timed out for {snapshot.name}: "
            f"{observed} != {snapshot.target_max}"
        )

    def _restore_policy(self, snapshot: PolicySnapshot) -> None:
        """Always reassert the saved minimum, even when an old read looks right.

        Some Jetson cpufreq writes become visible after write(2) returns.  A
        stale original read therefore cannot prove that a failed boost has
        finished.  Observe a quiet period, reassert on any rebound, and fail
        rather than claiming restoration when the bound is exceeded.
        """
        path = snapshot.path / "scaling_min_freq"
        deadline = time.monotonic() + self.restore_timeout_s
        stable_since: float | None = None
        writes = 0
        last_error: str | None = None
        observed: int | None = None
        last_write_at = float("-inf")
        retry_interval = min(0.2, self.restore_timeout_s / (self.max_restore_writes + 1))

        def write_original() -> None:
            nonlocal writes, last_error, last_write_at
            if writes >= self.max_restore_writes:
                return
            writes += 1
            last_write_at = time.monotonic()
            try:
                self.writer(path, snapshot.original_min)
                last_error = None
            except (CpuScopeError, OSError) as exc:
                last_error = str(exc)

        # Required even if an immediate read still shows the old minimum:
        # the earlier boost can remain in flight and appear after this read.
        write_original()
        while time.monotonic() < deadline:
            try:
                observed = _read_frequency(path)
            except CpuScopeError as exc:
                last_error = str(exc)
                observed = None
            now = time.monotonic()
            if observed == snapshot.original_min:
                if stable_since is None:
                    # A second write is ordered after the visible old value;
                    # then require a period with no conflicting readback.
                    if writes == 1 or now - last_write_at >= retry_interval:
                        write_original()
                    if last_error is None and writes >= 2:
                        stable_since = now
                elif now - stable_since >= self.restore_quiet_s:
                    maximum = _read_frequency(snapshot.path / "scaling_max_freq")
                    if maximum != snapshot.target_max:
                        raise CpuScopeError(
                            f"Maximum changed for {snapshot.name}: {maximum}, "
                            f"originally {snapshot.target_max}"
                        )
                    if last_error is None and _read_frequency(path) == snapshot.original_min:
                        return
            else:
                stable_since = None
                # A delayed boost may have arrived after an earlier restore.
                # Retry the saved value, but never loop or write indefinitely.
                if now - last_write_at >= retry_interval:
                    write_original()
            time.sleep(self.poll_s)
        raise CpuScopeError(
            f"Timed out restoring {snapshot.name}: observed={observed}, "
            f"expected={snapshot.original_min}, writes={writes}, last_error={last_error}"
        )

    def _verify_all_restored_quiet(self) -> None:
        """Catch a late write on a policy restored earlier than another."""
        deadline = time.monotonic() + self.restore_timeout_s
        quiet_since: float | None = None
        retries = {snapshot.name: 0 for snapshot in self.snapshots}
        last_retry_at = {snapshot.name: float("-inf") for snapshot in self.snapshots}
        retry_interval = min(0.2, self.restore_timeout_s / (self.max_restore_writes + 1))
        last_observed: dict[str, int] = {}
        last_write_errors: dict[str, str] = {}
        while time.monotonic() < deadline:
            mismatches: list[PolicySnapshot] = []
            for snapshot in self.snapshots:
                actual = _read_frequency(snapshot.path / "scaling_min_freq")
                last_observed[snapshot.name] = actual
                if actual != snapshot.original_min:
                    mismatches.append(snapshot)
                maximum = _read_frequency(snapshot.path / "scaling_max_freq")
                if maximum != snapshot.target_max:
                    raise CpuScopeError(
                        f"Maximum changed for {snapshot.name}: {maximum}, "
                        f"originally {snapshot.target_max}"
                    )
            now = time.monotonic()
            if mismatches:
                quiet_since = None
                for snapshot in mismatches:
                    if now - last_retry_at[snapshot.name] < retry_interval:
                        continue
                    retries[snapshot.name] += 1
                    last_retry_at[snapshot.name] = now
                    if retries[snapshot.name] > self.max_restore_writes:
                        raise CpuScopeError(
                            f"Late boost persisted for {snapshot.name}: "
                            f"{last_observed[snapshot.name]} != {snapshot.original_min}"
                        )
                    try:
                        self.writer(snapshot.path / "scaling_min_freq", snapshot.original_min)
                        last_write_errors.pop(snapshot.name, None)
                    except (CpuScopeError, OSError) as exc:
                        last_write_errors[snapshot.name] = str(exc)
            elif quiet_since is None:
                quiet_since = now
            elif now - quiet_since >= self.restore_quiet_s:
                if last_write_errors:
                    raise CpuScopeError(f"Final restoration writes failed: {last_write_errors}")
                return
            time.sleep(self.poll_s)
        raise CpuScopeError(
            f"Final all-policy restoration did not remain quiet: "
            f"{last_observed}; write_errors={last_write_errors}"
        )

    def _restore(self) -> None:
        failures: list[str] = []
        for snapshot in reversed(self.snapshots):
            try:
                self._restore_policy(snapshot)
            except (CpuScopeError, OSError) as exc:
                failures.append(
                    f"{snapshot.name} original_min={snapshot.original_min}: {exc}"
                )
        try:
            self._verify_all_restored_quiet()
        except (CpuScopeError, OSError) as exc:
            failures.append(f"final all-policy check: {exc}")
        self.restored = not failures
        if failures:
            raise CpuScopeError("CPU minimum restoration could not be verified: " + "; ".join(failures))

    def __enter__(self) -> "CpuPerformanceScope":
        self._acquire_lock()
        try:
            self.snapshots = self._discover()  # Capture each shared policy exactly once.
            for snapshot in self.snapshots:
                if _read_frequency(snapshot.path / "scaling_max_freq") != snapshot.target_max:
                    raise CpuScopeError(f"Maximum changed before setting {snapshot.name}")
                self.writer(snapshot.path / "scaling_min_freq", snapshot.target_max)
                self._await_boost(snapshot)
            return self
        except BaseException as original:
            try:
                if self.snapshots:
                    self._restore()
            except CpuScopeError as restore_error:
                raise CpuScopeError(f"Setup failed ({original}); {restore_error}") from original
            finally:
                self._release_lock()
            raise

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        try:
            self._restore()
        finally:
            self._release_lock()


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError as exc:
        raise CpuScopeError(f"Cannot inspect child process group {pgid}: {exc}") from exc
    return True


def _signal_group(pgid: int, sig: int) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass
    except PermissionError as exc:
        raise CpuScopeError(f"Cannot signal child process group {pgid}: {exc}") from exc


def _terminate_group(pgid: int, child: subprocess.Popen[bytes], grace_s: float) -> None:
    """Keep the CPU scope raised until the foreground process group is gone."""
    if not _group_exists(pgid):
        child.wait()
        return
    _signal_group(pgid, signal.SIGTERM)
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        try:
            child.wait(timeout=0.05)
        except subprocess.TimeoutExpired:
            pass
        if not _group_exists(pgid):
            return
    _signal_group(pgid, signal.SIGKILL)
    try:
        child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired as exc:
        raise CpuScopeError(f"Child process {child.pid} did not stop") from exc
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if not _group_exists(pgid):
            return
        time.sleep(0.05)
    raise CpuScopeError(f"Child process group {pgid} could not be confirmed stopped")


def _child_identity() -> dict[str, object]:
    """Avoid unintentionally running a diagnostic as root under sudo."""
    if os.geteuid() != 0 or "SUDO_UID" not in os.environ:
        return {}
    import pwd

    try:
        uid = int(os.environ["SUDO_UID"])
        gid = int(os.environ["SUDO_GID"])
        user = pwd.getpwuid(uid)
        groups = os.getgrouplist(user.pw_name, gid)
    except (KeyError, ValueError, OSError) as exc:
        raise CpuScopeError(f"Cannot recover invoking user for child: {exc}") from exc
    environment = os.environ.copy()
    environment.update(HOME=user.pw_dir, USER=user.pw_name, LOGNAME=user.pw_name)
    return {"user": uid, "group": gid, "extra_groups": groups, "env": environment}


def run_command(
    command: Sequence[str], scope: CpuPerformanceScope, *, grace_s: float = 3.0
) -> tuple[int, int | None]:
    if not command:
        raise CpuScopeError("No child command was provided")
    pending_signal: int | None = None
    watched = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {sig: signal.getsignal(sig) for sig in watched}

    def catch(sig: int, _frame: object) -> None:
        nonlocal pending_signal
        if pending_signal is None:
            pending_signal = sig

    for sig in watched:
        signal.signal(sig, catch)
    child: subprocess.Popen[bytes] | None = None
    result_code: int | None = None
    try:
        with scope:
            if pending_signal is None:
                try:
                    child = subprocess.Popen(
                        list(command), start_new_session=True, close_fds=True, **_child_identity()
                    )
                except OSError as exc:
                    raise CpuScopeError(f"Cannot start child command: {exc}") from exc
                try:
                    while True:
                        if pending_signal is not None:
                            _terminate_group(child.pid, child, grace_s)
                            break
                        try:
                            code = child.wait(timeout=0.1)
                            result_code = 128 - code if code < 0 else code
                            break
                        except subprocess.TimeoutExpired:
                            pass
                    # A command must not leave background workers using an unscoped CPU.
                    if _group_exists(child.pid):
                        _terminate_group(child.pid, child, grace_s)
                finally:
                    if child.poll() is None or _group_exists(child.pid):
                        _terminate_group(child.pid, child, grace_s)
        # Check after restoration too: a signal may arrive while sysfs is restored.
        if pending_signal is not None:
            return 128 + pending_signal, pending_signal
        if result_code is None:
            raise CpuScopeError("Child command ended without an exit status")
        return result_code, None
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sysfs-root", type=Path, default=DEFAULT_SYSFS)
    parser.add_argument("--lock-file", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--require-cpus", default="0-5", help="CPU list that policies must cover")
    parser.add_argument("--term-grace-seconds", type=float, default=3.0)
    parser.add_argument("command", nargs=argparse.REMAINDER, help="foreground command after --")
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command and args.command[0] == "--" else args.command
    if not command:
        parser.error("a foreground command is required after --")
    if not 0 < args.term_grace_seconds <= 30:
        parser.error("--term-grace-seconds must be in (0, 30]")
    try:
        required_cpus = parse_cpu_list(args.require_cpus)
        scope = CpuPerformanceScope(args.sysfs_root, args.lock_file, required_cpus)
        code, caught_signal = run_command(command, scope, grace_s=args.term_grace_seconds)
    except (CpuScopeError, ValueError) as exc:
        print(f"CPU_SCOPE_FAILED: {exc}", file=sys.stderr, flush=True)
        return 125
    print(
        json.dumps(
            {"status": "RESTORED", "restored": scope.restored, "child_exit_code": code,
             "caught_signal": caught_signal,
             "restoration_quiet_s": scope.restore_quiet_s,
             "policies": [
                 {"name": item.name, "related_cpus": sorted(item.related_cpus),
                  "original_min": item.original_min, "raised_min": item.target_max}
                 for item in scope.snapshots
             ]},
            sort_keys=True,
        ),
        flush=True,
    )
    return code


if __name__ == "__main__":
    raise SystemExit(main())
