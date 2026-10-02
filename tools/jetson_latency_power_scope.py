"""Temporarily disable Jetson C7 and raise EMC for one foreground comparison.

Run with ``--diagnostic -- COMMAND ...`` for a no-output diagnostic, or explicitly
``--supported-characterization`` for a supported trial approved by the child's
existing profile loader. This wrapper grants no motor-output approval and does
not inspect CAN commands. It saves all online
CPU idle settings and EMC min/max before writing, changes only C7 states named
``c7`` with latency >= 1000 us, and leaves WFI and every other idle state alone.
The existing CPU scope runner owns the child process group and waits for its
cleanup before restoration. SIGINT/SIGTERM/SIGHUP, command failure and Python
exceptions all restore the saved values with readback. SIGKILL, power loss and
children escaping their process group cannot be recovered by this wrapper.

By default it preserves CPU frequency minima. ``--with-cpu-performance`` also
enters the existing CPU-minimum scope within the same child lifecycle. Power
mode, governors and EMC maximum remain unchanged. Use an identical workload
and CPU-frequency scope for the baseline/comparison.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import dataclasses
import fcntl
import json
import os
from pathlib import Path
import re
import sys
import time
from typing import Callable, Sequence

try:  # Both direct script execution and repository imports are supported.
    from tools.jetson_cpu_performance_scope import (
        CpuPerformanceScope, CpuScopeError, DEFAULT_SYSFS as DEFAULT_CPU_FREQ_ROOT,
        parse_cpu_list, run_command,
    )
except ModuleNotFoundError:
    from jetson_cpu_performance_scope import (
        CpuPerformanceScope, CpuScopeError, DEFAULT_SYSFS as DEFAULT_CPU_FREQ_ROOT,
        parse_cpu_list, run_command,
    )


DEFAULT_CPU_ROOT = Path('/sys/devices/system/cpu')
DEFAULT_EMC_ROOT = Path('/sys/class/devfreq/bwmgr')
DEFAULT_LOCK = Path('/run/lock/singularitydog-latency-power-scope.lock')
_STATE_NAME = re.compile(r'state[0-9]+\Z')


class LatencyPowerScopeError(CpuScopeError):
    """The diagnostic-only power scope was not established or restored."""


def _read_integer(path: Path, *, minimum: int = 0) -> int:
    try:
        raw = path.read_text(encoding='ascii').strip()
        if not raw.isdecimal():
            raise ValueError('expected a nonnegative decimal integer')
        value = int(raw)
        if value < minimum:
            raise ValueError(f'expected >= {minimum}')
        return value
    except (OSError, ValueError) as error:
        raise LatencyPowerScopeError(f'Cannot read {path}: {error}') from error


def _write_integer(path: Path, value: int) -> None:
    try:
        path.write_text(f'{value}\n', encoding='ascii')
    except OSError as error:
        raise LatencyPowerScopeError(f'Cannot set {path} to {value}: {error}') from error


@dataclasses.dataclass(frozen=True)
class IdleSnapshot:
    cpu: int
    path: Path
    name: str
    latency_us: int
    original_disable: int

    @property
    def selected(self) -> bool:
        return self.name == 'c7' and self.latency_us >= 1000


@dataclasses.dataclass(frozen=True)
class SettingSnapshot:
    name: str
    path: Path
    original: int
    target: int


class LatencyPowerScope:
    """Exclusive reversible C7/EMC settings; no unsupported partial scope."""

    def __init__(self, cpu_root: Path = DEFAULT_CPU_ROOT,
                 emc_root: Path = DEFAULT_EMC_ROOT, lock_path: Path = DEFAULT_LOCK,
                 *, writer: Callable[[Path, int], None] = _write_integer,
                 boost_timeout_s: float = 2., restore_timeout_s: float = 4.,
                 restore_quiet_s: float = 1., poll_s: float = .02,
                 max_restore_writes: int = 20, emit: Callable[[dict], None] | None = None,
                 scope_kind: str = 'diagnostic'):
        if not (boost_timeout_s > 0 and restore_timeout_s > restore_quiet_s > 0
                and poll_s > 0 and max_restore_writes >= 2):
            raise ValueError('Invalid power scope settle/restore timing')
        if scope_kind not in ('diagnostic', 'supported-characterization'):
            raise ValueError('An explicit diagnostic/supported-characterization scope is required')
        self.cpu_root, self.emc_root, self.lock_path = map(Path, (cpu_root, emc_root, lock_path))
        self.writer, self.emit = writer, emit
        self.scope_kind = scope_kind
        self.boost_timeout_s, self.restore_timeout_s = boost_timeout_s, restore_timeout_s
        self.restore_quiet_s, self.poll_s = restore_quiet_s, poll_s
        self.max_restore_writes = max_restore_writes
        self.online_cpus: frozenset[int] = frozenset()
        self.idle: tuple[IdleSnapshot, ...] = ()
        self.settings: tuple[SettingSnapshot, ...] = ()
        self.emc_max: int | None = None
        self._lock_fd: int | None = None
        self.restored = False
        self.established_ns: int | None = None
        self.restored_ns: int | None = None
        self.restore_errors: list[str] = []

    def _online(self) -> frozenset[int]:
        try:
            return parse_cpu_list((self.cpu_root / 'online').read_text(encoding='ascii'))
        except (OSError, CpuScopeError) as error:
            raise LatencyPowerScopeError(f'Cannot read online CPUs: {error}') from error

    def _acquire_lock(self) -> None:
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0)
        try:
            fd = os.open(self.lock_path, flags, 0o600)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            if 'fd' in locals(): os.close(fd)
            raise LatencyPowerScopeError(f'Cannot acquire exclusive power scope lock: {error}') from error
        self._lock_fd = fd

    def _release_lock(self) -> None:
        if self._lock_fd is not None:
            fd, self._lock_fd = self._lock_fd, None
            try: fcntl.flock(fd, fcntl.LOCK_UN)
            finally: os.close(fd)

    def _discover(self) -> None:
        online = self._online()
        idle: list[IdleSnapshot] = []
        settings: list[SettingSnapshot] = []
        # Build everything in locals: unsupported platforms fail before writes.
        for cpu in sorted(online):
            directory = self.cpu_root / f'cpu{cpu}' / 'cpuidle'
            try:
                states = sorted((p for p in directory.iterdir() if _STATE_NAME.fullmatch(p.name)),
                                key=lambda p: int(p.name[5:]))
                for state in states:
                    name = (state / 'name').read_text(encoding='ascii').strip()
                    item = IdleSnapshot(cpu, state, name, _read_integer(state / 'latency'),
                                        _read_integer(state / 'disable'))
                    if item.original_disable not in (0, 1):
                        raise LatencyPowerScopeError(f'Invalid idle disable at {state}')
                    idle.append(item)
                    if item.selected:
                        settings.append(SettingSnapshot(f'cpu{cpu}/{state.name}/{name}',
                                                        state / 'disable', item.original_disable, 1))
            except OSError as error:
                raise LatencyPowerScopeError(f'Unsupported CPU idle layout for cpu{cpu}: {error}') from error
            if not any(item.cpu == cpu and item.selected for item in idle):
                raise LatencyPowerScopeError(f'cpu{cpu} has no c7 state with latency >= 1000 us')
            if not any(item.cpu == cpu and item.name.casefold() == 'wfi' for item in idle):
                raise LatencyPowerScopeError(f'cpu{cpu} has no WFI state to preserve')
        minimum = _read_integer(self.emc_root / 'min_freq')
        maximum = _read_integer(self.emc_root / 'max_freq', minimum=1)
        if minimum > maximum:
            raise LatencyPowerScopeError('EMC minimum exceeds maximum')
        settings.append(SettingSnapshot('emc/min_freq', self.emc_root / 'min_freq', minimum, maximum))
        if online != self._online():
            raise LatencyPowerScopeError('Online CPU set changed during discovery')
        self.online_cpus, self.idle, self.settings, self.emc_max = online, tuple(idle), tuple(settings), maximum

    def _guard_errors(self) -> list[str]:
        errors: list[str] = []
        try:
            if self._online() != self.online_cpus:
                errors.append('Online CPU set changed during scope')
        except CpuScopeError as error: errors.append(str(error))
        for item in self.idle:
            try:
                if ((item.path / 'name').read_text(encoding='ascii').strip() != item.name or
                        _read_integer(item.path / 'latency') != item.latency_us):
                    errors.append(f'Idle state identity changed: {item.path}')
                if not item.selected and _read_integer(item.path / 'disable') != item.original_disable:
                    errors.append(f'Unselected idle state changed: {item.path}')
            except (CpuScopeError, OSError) as error: errors.append(str(error))
        try:
            if _read_integer(self.emc_root / 'max_freq', minimum=1) != self.emc_max:
                errors.append('EMC maximum changed during scope')
        except CpuScopeError as error: errors.append(str(error))
        return errors

    def _await_active(self) -> None:
        deadline = time.monotonic() + self.boost_timeout_s
        stable_since = None
        while time.monotonic() < deadline:
            errors = self._guard_errors()
            if errors: raise LatencyPowerScopeError('; '.join(errors))
            matching = all(_read_integer(item.path) == item.target for item in self.settings)
            now = time.monotonic()
            if matching:
                if stable_since is None: stable_since = now
                elif now - stable_since >= min(.1, self.boost_timeout_s / 2): return
            else: stable_since = None
            time.sleep(self.poll_s)
        raise LatencyPowerScopeError('C7/EMC setup readback did not settle before child')

    def _restore(self) -> None:
        """Reassert saved values and verify all together over a quiet period.

        A delayed setup write may become visible after a stale original-value
        read. Two ordered restore writes plus a bounded all-setting quiet check
        catch that rebound; each setting is retried independently on error.
        """
        self.restored = False
        self.restore_errors = []
        states = {item.name: {'writes': 0, 'last_write': float('-inf'), 'error': None}
                  for item in self.settings}
        retry_s = min(.2, self.restore_timeout_s / (self.max_restore_writes + 1))
        deadline = time.monotonic() + self.restore_timeout_s
        quiet_since = None

        def restore(item: SettingSnapshot) -> None:
            state = states[item.name]
            if state['writes'] >= self.max_restore_writes: return
            state['writes'] += 1
            state['last_write'] = time.monotonic()
            try:
                self.writer(item.path, item.original)
                state['error'] = None
            except (CpuScopeError, OSError) as error: state['error'] = str(error)

        for item in reversed(self.settings): restore(item)
        while time.monotonic() < deadline:
            all_matching = True
            now = time.monotonic()
            for item in reversed(self.settings):
                state = states[item.name]
                try: matching = _read_integer(item.path) == item.original
                except CpuScopeError as error:
                    state['error'] = str(error); matching = False
                if not matching or state['writes'] < 2 or state['error']:
                    all_matching = False
                    if state['writes'] == 1 or now - state['last_write'] >= retry_s:
                        restore(item)
            if all_matching:
                if quiet_since is None: quiet_since = now
                elif now - quiet_since >= self.restore_quiet_s:
                    errors = self._guard_errors()
                    # A final read after guard checking closes the readback gate.
                    for item in self.settings:
                        try:
                            if _read_integer(item.path) != item.original:
                                errors.append(f'Final restore mismatch: {item.name}')
                        except CpuScopeError as error: errors.append(str(error))
                    if errors:
                        self.restore_errors = errors
                        raise LatencyPowerScopeError('Restoration/unchanged-state check failed: ' + '; '.join(errors))
                    self.restored = True
                    self.restored_ns = time.monotonic_ns()
                    return
            else: quiet_since = None
            time.sleep(self.poll_s)
        self.restore_errors = [f"{item.name}: original={item.original}, writes={states[item.name]['writes']}, "
                               f"last_error={states[item.name]['error']}" for item in self.settings]
        raise LatencyPowerScopeError('C7/EMC restoration could not be verified: ' + '; '.join(self.restore_errors))

    def summary(self) -> dict:
        label = ('no_output_diagnostic_c7_emc_only' if self.scope_kind == 'diagnostic'
                 else 'supported_characterization_c7_emc_only')
        return {'scope': label, 'output_permission_granted_by_scope': False,
                'online_cpus': sorted(self.online_cpus),
                'established_ns': self.established_ns, 'restored_ns': self.restored_ns,
                'restored': self.restored, 'restoration_quiet_s': self.restore_quiet_s,
                'settings': [{'name': item.name, 'path': str(item.path), 'original': item.original,
                              'scoped': item.target} for item in self.settings],
                'unchanged_idle_states': [{'cpu': item.cpu, 'state': item.path.name,
                    'name': item.name, 'latency_us': item.latency_us, 'disable': item.original_disable}
                    for item in self.idle if not item.selected],
                'emc_max_unchanged': self.emc_max, 'restore_errors': self.restore_errors}

    def __enter__(self) -> 'LatencyPowerScope':
        self._acquire_lock()
        try:
            self._discover()
            for item in self.settings:
                errors = self._guard_errors()
                if errors: raise LatencyPowerScopeError('; '.join(errors))
                self.writer(item.path, item.target)
            self._await_active()
            self.established_ns = time.monotonic_ns()
            if self.emit: self.emit({'kind': 'latency_power_scope_established', **self.summary()})
            return self
        except BaseException as original:
            try:
                if self.settings: self._restore()
            except CpuScopeError as restore_error:
                raise LatencyPowerScopeError(f'Setup failed ({original}); {restore_error}') from original
            finally: self._release_lock()
            raise

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        try: self._restore()
        finally: self._release_lock()


class CombinedDiagnosticScope:
    """One child session, with reverse-order CPU then C7/EMC restoration.

    Nesting the CPU wrapper as a subprocess would create a second session that
    the outer wrapper cannot terminate. Enter its scope here instead; the sole
    run_command owns the actual diagnostic and inherited worker process group.
    """
    def __init__(self, power: LatencyPowerScope, cpu: CpuPerformanceScope | None):
        self.power, self.cpu = power, cpu
        self._stack: ExitStack | None = None

    def __enter__(self) -> 'CombinedDiagnosticScope':
        stack = ExitStack()
        try:
            stack.enter_context(self.power)
            if self.cpu is not None:
                stack.enter_context(self.cpu)
                if self.power.emit:
                    self.power.emit({'kind': 'latency_cpu_performance_established',
                        'scope': self.power.summary()['scope'],
                        'cpu_policies': [{'name': item.name, 'related_cpus': sorted(item.related_cpus),
                            'original_min': item.original_min, 'scoped_min': item.target_max}
                            for item in self.cpu.snapshots]})
        except BaseException:
            stack.close()
            raise
        self._stack = stack
        return self

    def __exit__(self, *args: object) -> None:
        stack, self._stack = self._stack, None
        if stack is not None: stack.__exit__(*args)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--diagnostic', action='store_true',
                      help='declare the child is a no-output diagnostic')
    mode.add_argument('--supported-characterization', action='store_true',
                      help='declare a supported trial; child profile loader must approve all output')
    parser.add_argument('--cpu-root', type=Path, default=DEFAULT_CPU_ROOT)
    parser.add_argument('--emc-root', type=Path, default=DEFAULT_EMC_ROOT)
    parser.add_argument('--lock-file', type=Path, default=DEFAULT_LOCK)
    parser.add_argument('--with-cpu-performance', action='store_true',
                        help='enter the existing CPU-minimum scope without a nested child session')
    parser.add_argument('--cpu-frequency-root', type=Path, default=DEFAULT_CPU_FREQ_ROOT)
    parser.add_argument('--term-grace-seconds', type=float, default=3.)
    parser.add_argument('command', nargs=argparse.REMAINDER, help='one foreground command after --')
    args = parser.parse_args(argv)
    if not args.command or args.command[0] != '--' or len(args.command) == 1:
        parser.error('a foreground command is required after an explicit --')
    if not 0 < args.term_grace_seconds <= 30:
        parser.error('--term-grace-seconds must be in (0, 30]')
    emit = lambda row: print(json.dumps(row, sort_keys=True), flush=True)
    scope = LatencyPowerScope(args.cpu_root, args.emc_root, args.lock_file, emit=emit,
        scope_kind='diagnostic' if args.diagnostic else 'supported-characterization')
    cpu_scope = None
    try:
        if args.with_cpu_performance:
            cpu_scope = CpuPerformanceScope(args.cpu_frequency_root,
                required_cpus=scope._online())
        combined = CombinedDiagnosticScope(scope, cpu_scope)
        code, caught_signal = run_command(args.command[1:], combined, grace_s=args.term_grace_seconds)
    except KeyboardInterrupt:
        code, caught_signal = 130, 2
    except (CpuScopeError, OSError, ValueError) as error:
        emit({'kind': 'latency_power_scope_exit', 'status': 'FAILED', 'error': str(error),
              'cpu_performance_restored': cpu_scope.restored if cpu_scope else None, **scope.summary()})
        print(f'LATENCY_POWER_SCOPE_FAILED: {error}', file=sys.stderr, flush=True)
        return 125
    emit({'kind': 'latency_power_scope_exit', 'status': 'RESTORED', 'child_exit_code': code,
          'caught_signal': caught_signal,
          'cpu_performance_restored': cpu_scope.restored if cpu_scope else None,
          'cpu_performance_policies': [
              {'name': item.name, 'related_cpus': sorted(item.related_cpus),
               'original_min': item.original_min, 'scoped_min': item.target_max}
              for item in cpu_scope.snapshots] if cpu_scope else [], **scope.summary()})
    return code


if __name__ == '__main__':
    raise SystemExit(main())
