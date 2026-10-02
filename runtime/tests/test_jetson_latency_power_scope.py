"""File-only C7/EMC tests: never read or write the host's real sysfs."""

import io
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
from contextlib import redirect_stderr, redirect_stdout
from unittest.mock import patch

from tools import jetson_cpu_performance_scope as cpu
from tools import jetson_latency_power_scope as power


def fake_platform(base):
    root = base / 'cpu'
    root.mkdir()
    (root / 'online').write_text('0-5\n')
    for index in range(7):  # cpu6 is offline and must remain untouched.
        for state, name, latency, disabled in (
                (0, 'WFI', 1, 0), (1, 'c7', 5000, int(index == 3)), (2, 'c6', 1200, 0)):
            directory = root / f'cpu{index}' / 'cpuidle' / f'state{state}'
            directory.mkdir(parents=True)
            for key, value in {'name': name, 'latency': latency, 'disable': disabled}.items():
                (directory / key).write_text(f'{value}\n')
    emc = base / 'bwmgr'
    emc.mkdir()
    (emc / 'min_freq').write_text('0\n')  # Zero is a valid saved EMC floor.
    (emc / 'max_freq').write_text('3199000000\n')
    return root, emc


def read(path):
    return int(path.read_text())


class LatencyPowerScopeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.root, self.emc = fake_platform(self.base)
        self.lock = self.base / 'power.lock'
        self.timers = []
        self.addCleanup(lambda: [timer.join(2) for timer in self.timers])

    def scope(self, **settings):
        config = dict(boost_timeout_s=.12, restore_timeout_s=.4,
                      restore_quiet_s=.06, poll_s=.005, max_restore_writes=10)
        config.update(settings)
        return power.LatencyPowerScope(self.root, self.emc, self.lock, **config)

    def c7(self, index):
        return self.root / f'cpu{index}' / 'cpuidle/state1/disable'

    def assert_original(self):
        self.assertEqual([read(self.c7(i)) for i in range(7)], [0, 0, 0, 1, 0, 0, 0])
        self.assertEqual(read(self.emc / 'min_freq'), 0)
        self.assertEqual(read(self.emc / 'max_freq'), 3199000000)

    def test_only_all_online_c7_and_emc_are_changed_then_exactly_restored(self):
        writes = []
        def writer(path, value):
            writes.append((path, value)); path.write_text(f'{value}\n')
        scope = self.scope(writer=writer)
        with scope:
            self.assertEqual([read(self.c7(i)) for i in range(7)], [1] * 6 + [0])
            self.assertEqual(read(self.emc / 'min_freq'), 3199000000)
            self.assertEqual(scope.online_cpus, frozenset(range(6)))
        self.assertTrue(scope.restored)
        self.assert_original()
        self.assertEqual({path for path, _ in writes}, {self.c7(i) for i in range(6)} | {self.emc / 'min_freq'})
        self.assertTrue(all(read(self.root / f'cpu{i}/cpuidle/state0/disable') == 0 for i in range(7)))
        self.assertTrue(all(read(self.root / f'cpu{i}/cpuidle/state2/disable') == 0 for i in range(7)))
        self.assertEqual(len(scope.summary()['unchanged_idle_states']), 12)

    def test_unsupported_layout_fails_before_writes_or_child(self):
        for failure in ('no_c7', 'fast_c7', 'no_wfi', 'bad_disable', 'no_emc', 'min_above_max'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temporary:
                base = Path(temporary); root, emc = fake_platform(base); writes = []
                if failure == 'no_c7': (root / 'cpu5/cpuidle/state1/name').write_text('c6\n')
                if failure == 'fast_c7': (root / 'cpu5/cpuidle/state1/latency').write_text('999\n')
                if failure == 'no_wfi': (root / 'cpu5/cpuidle/state0/name').write_text('other\n')
                if failure == 'bad_disable': (root / 'cpu5/cpuidle/state1/disable').write_text('2\n')
                if failure == 'no_emc': (emc / 'min_freq').unlink()
                if failure == 'min_above_max': (emc / 'min_freq').write_text('4000000000\n')
                marker = base / 'child-ran'
                with self.assertRaises(power.LatencyPowerScopeError):
                    power.run_command([sys.executable, '-c', f'from pathlib import Path;Path({str(marker)!r}).touch()'],
                        power.LatencyPowerScope(root, emc, base / 'lock', writer=lambda p, v: writes.append((p, v))))
                self.assertFalse(marker.exists()); self.assertEqual(writes, [])

    def test_partial_setup_write_that_changes_then_raises_is_rolled_back(self):
        marker = self.base / 'child-ran'
        def writer(path, value):
            path.write_text(f'{value}\n')
            if path == self.c7(4) and value == 1:
                raise power.LatencyPowerScopeError('injected after-write setup failure')
        scope = self.scope(writer=writer)
        with self.assertRaisesRegex(power.LatencyPowerScopeError, 'after-write'):
            power.run_command([sys.executable, '-c', f'from pathlib import Path;Path({str(marker)!r}).touch()'], scope)
        self.assertTrue(scope.restored); self.assertFalse(marker.exists()); self.assert_original()

    def test_each_setup_write_failure_rolls_back_including_emc(self):
        for failing_path in [self.c7(i) for i in range(6)] + [self.emc / 'min_freq']:
            with self.subTest(path=failing_path):
                failed = False
                def writer(path, value):
                    nonlocal failed
                    path.write_text(f'{value}\n')
                    if path == failing_path and not failed:
                        failed = True
                        raise OSError('injected write error')
                scope = self.scope(writer=writer)
                with self.assertRaisesRegex(OSError, 'injected'):
                    with scope: self.fail('failed setup must not admit child')
                self.assertTrue(scope.restored); self.assert_original()

    def test_delayed_setup_writes_cannot_escape_rollback_or_quiet_check(self):
        counts = {}
        def writer(path, value):
            counts[path, value] = counts.get((path, value), 0) + 1
            if path == self.c7(0) and value == 1:
                return  # Accepted setup write is still pending.
            path.write_text(f'{value}\n')
            if path == self.c7(0) and value == 0 and counts[path, value] == 1:
                timer = threading.Timer(.025, lambda: path.write_text('1\n'))
                self.timers.append(timer); timer.start()
        scope = self.scope(writer=writer, boost_timeout_s=.02)
        with self.assertRaisesRegex(power.LatencyPowerScopeError, 'setup readback'):
            with scope: self.fail('child scope must not be established')
        self.assertTrue(scope.restored); self.assert_original()
        self.assertGreaterEqual(counts[self.c7(0), 0], 3)

    def test_late_emc_rebound_resets_all_setting_quiet_period(self):
        counts = {}
        def writer(path, value):
            counts[path, value] = counts.get((path, value), 0) + 1
            path.write_text(f'{value}\n')
            if path == self.emc / 'min_freq' and value == 0 and counts[path, value] == 2:
                timer = threading.Timer(.025, lambda: path.write_text('3199000000\n'))
                self.timers.append(timer); timer.start()
        scope = self.scope(writer=writer)
        with scope: pass
        self.assertTrue(scope.restored); self.assert_original()
        self.assertGreaterEqual(counts[self.emc / 'min_freq', 0], 3)

    def test_body_error_and_keyboardinterrupt_both_restore(self):
        for error in (RuntimeError('body failed'), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                scope = self.scope()
                with self.assertRaises(type(error)):
                    with scope: raise error
                self.assertTrue(scope.restored); self.assert_original()

    def test_wfi_or_maximum_drift_is_reported_without_writing_unselected_state(self):
        for path, value in ((self.root / 'cpu2/cpuidle/state0/disable', 1),
                            (self.emc / 'max_freq', 3200000000)):
            original = path.read_text(); writes = []
            def writer(p, v): writes.append(p); p.write_text(f'{v}\n')
            scope = self.scope(writer=writer)
            with self.assertRaisesRegex(power.LatencyPowerScopeError, 'check failed'):
                with scope: path.write_text(f'{value}\n')
            self.assertFalse(scope.restored); self.assertNotIn(path, writes)
            self.assertEqual(read(self.emc / 'min_freq'), 0)
            path.write_text(original)
            self.assert_original()

    def test_online_cpu_hotplug_is_detected_and_saved_settings_restored(self):
        scope = self.scope()
        with self.assertRaisesRegex(power.LatencyPowerScopeError, 'Online CPU set'):
            with scope: (self.root / 'online').write_text('0-4\n')
        self.assertFalse(scope.restored); self.assert_original()

    def test_persistent_restore_failure_reports_failure_but_restores_other_settings(self):
        def writer(path, value):
            if path == self.c7(0) and value == 0:
                raise power.LatencyPowerScopeError('restore forbidden')
            path.write_text(f'{value}\n')
        scope = self.scope(writer=writer, restore_timeout_s=.15)
        with self.assertRaisesRegex(power.LatencyPowerScopeError, 'restoration could not be verified'):
            with scope: pass
        self.assertFalse(scope.restored)
        self.assertEqual(read(self.c7(0)), 1)
        self.assertEqual(read(self.emc / 'min_freq'), 0)
        self.assertEqual([read(self.c7(i)) for i in range(1, 6)], [0, 0, 1, 0, 0])

    def test_emc_restore_failure_still_restores_all_c7_settings(self):
        def writer(path, value):
            if path == self.emc / 'min_freq' and value == 0:
                raise power.LatencyPowerScopeError('emc restore failed')
            path.write_text(f'{value}\n')
        scope = self.scope(writer=writer, restore_timeout_s=.15)
        with self.assertRaisesRegex(power.LatencyPowerScopeError, 'emc restore failed'):
            with scope: pass
        self.assertFalse(scope.restored)
        self.assertEqual([read(self.c7(i)) for i in range(7)], [0, 0, 0, 1, 0, 0, 0])

    def test_child_spawn_failure_and_keyboardinterrupt_restore_scope(self):
        handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        for error in (OSError('spawn forbidden'), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                scope = self.scope()
                with patch.object(cpu.subprocess, 'Popen', side_effect=error):
                    with self.assertRaises((cpu.CpuScopeError, KeyboardInterrupt)):
                        power.run_command(['child'], scope)
                self.assertTrue(scope.restored); self.assert_original()
                self.assertEqual({sig: signal.getsignal(sig) for sig in handlers}, handlers)

    def test_signal_while_restoring_is_deferred_until_all_readback_completes(self):
        caught = False
        def writer(path, value):
            nonlocal caught
            path.write_text(f'{value}\n')
            if path == self.emc / 'min_freq' and value == 0 and not caught:
                caught = True
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        scope = self.scope(writer=writer)
        self.assertEqual(power.run_command([sys.executable, '-c', 'pass'], scope), (143, signal.SIGTERM))
        self.assertTrue(scope.restored); self.assert_original()

    def test_exclusive_lock_rejects_second_scope(self):
        with self.scope():
            with self.assertRaisesRegex(power.LatencyPowerScopeError, 'exclusive'):
                with self.scope(): self.fail('second scope started')
        self.assert_original()

    def test_child_sees_both_changes_and_failure_exit_still_restores(self):
        observed = self.base / 'observed'
        source = ('from pathlib import Path;import json,sys;'
            f'Path({str(observed)!r}).write_text(json.dumps(['
            f'int(Path({str(self.c7(0))!r}).read_text()),int(Path({str(self.emc / "min_freq")!r}).read_text())]));'
            'sys.exit(7)')
        scope = self.scope()
        self.assertEqual(power.run_command([sys.executable, '-c', source], scope), (7, None))
        self.assertEqual(json.loads(observed.read_text()), [1, 3199000000])
        self.assertTrue(scope.restored); self.assert_original()

    def test_combined_cpu_scope_uses_single_runner_and_reverse_restoration(self):
        frequency = self.base / 'cpufreq'; policy = frequency / 'policy0'; policy.mkdir(parents=True)
        for name, value in {'related_cpus': '0-5', 'scaling_min_freq': '729600', 'scaling_max_freq': '1728000'}.items():
            (policy / name).write_text(f'{value}\n')
        writes = []
        def writer(path, value): writes.append((path, value)); path.write_text(f'{value}\n')
        cpu_scope = cpu.CpuPerformanceScope(frequency, self.base / 'cpu.lock', frozenset(range(6)),
            writer=writer, boost_timeout_s=.12, restore_timeout_s=.3, restore_quiet_s=.04, poll_s=.005)
        power_scope = self.scope(writer=writer)
        combined = power.CombinedDiagnosticScope(power_scope, cpu_scope)
        with combined:
            self.assertEqual(read(policy / 'scaling_min_freq'), 1728000)
            self.assertEqual(read(self.c7(0)), 1)
        self.assertTrue(power_scope.restored); self.assertTrue(cpu_scope.restored); self.assert_original()
        restores = [path for path, value in writes if value in (729600, 0)]
        self.assertEqual(restores[0], policy / 'scaling_min_freq')
        self.assertEqual(read(policy / 'scaling_min_freq'), 729600)

    def test_combined_cpu_setup_failure_restores_power_before_no_child(self):
        scope = self.scope()
        cpu_scope = cpu.CpuPerformanceScope(self.base / 'missing', self.base / 'cpu.lock')
        marker = self.base / 'child-ran'
        with self.assertRaises(cpu.CpuScopeError):
            power.run_command([sys.executable, '-c', f'from pathlib import Path;Path({str(marker)!r}).touch()'],
                              power.CombinedDiagnosticScope(scope, cpu_scope))
        self.assertFalse(marker.exists()); self.assertTrue(scope.restored); self.assert_original()

    def test_cpu_restore_failure_does_not_prevent_c7_emc_restoration(self):
        frequency = self.base / 'cpufreq'; policy = frequency / 'policy0'; policy.mkdir(parents=True)
        for name, value in {'related_cpus': '0-5', 'scaling_min_freq': '729600', 'scaling_max_freq': '1728000'}.items():
            (policy / name).write_text(f'{value}\n')
        def writer(path, value):
            if value == 729600: raise cpu.CpuScopeError('injected CPU restore failure')
            path.write_text(f'{value}\n')
        cpu_scope = cpu.CpuPerformanceScope(frequency, self.base / 'cpu.lock', frozenset(range(6)),
            writer=writer, boost_timeout_s=.12, restore_timeout_s=.15, restore_quiet_s=.04, poll_s=.005)
        power_scope = self.scope()
        with self.assertRaisesRegex(cpu.CpuScopeError, 'CPU restore failure'):
            with power.CombinedDiagnosticScope(power_scope, cpu_scope): pass
        self.assertFalse(cpu_scope.restored); self.assertTrue(power_scope.restored); self.assert_original()

    def test_cli_requires_diagnostic_and_explicit_separator(self):
        for arguments in (['--', sys.executable, '-c', 'pass'],
                          ['--diagnostic', sys.executable, '-c', 'pass'], ['--diagnostic', '--'],
                          ['--diagnostic', '--supported-characterization', '--', sys.executable, '-c', 'pass']):
            with self.subTest(arguments=arguments), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exit:
                    power.main(arguments)
                self.assertEqual(exit.exception.code, 2)
        self.assert_original()

    def test_supported_scope_label_grants_no_output_permission(self):
        scope = self.scope(scope_kind='supported-characterization')
        with redirect_stdout(io.StringIO()) as output, patch.object(power, 'LatencyPowerScope', return_value=scope):
            self.assertEqual(power.main(['--supported-characterization', '--', sys.executable, '-c', 'pass']), 0)
        final = json.loads(output.getvalue().splitlines()[-1])
        self.assertEqual(final['scope'], 'supported_characterization_c7_emc_only')
        self.assertFalse(final['output_permission_granted_by_scope'])
        self.assertTrue(final['restored']); self.assert_original()

    def signal_child_cleanup(self, signum):
        ready = self.base / 'ready'; cleanup = self.base / 'cleanup-observed'
        source = ('import signal,time;from pathlib import Path;'
            f'ready=Path({str(ready)!r});out=Path({str(cleanup)!r});'
            f'p=Path({str(self.c7(0))!r});'
            'signal.signal(signal.SIGTERM,lambda s,f:(out.write_text(p.read_text()),exit(0)));'
            'ready.touch();time.sleep(10)')
        wrapper = Path(power.__file__)
        process = subprocess.Popen([sys.executable, str(wrapper), '--diagnostic',
            '--cpu-root', str(self.root), '--emc-root', str(self.emc), '--lock-file', str(self.lock),
            '--', sys.executable, '-c', source], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 4
            while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(ready.exists(), 'child failed to start')
            process.send_signal(signum)
            stdout, stderr = process.communicate(timeout=8)
            self.assertEqual(process.returncode, 128 + signum, stderr)
            final = json.loads(stdout.splitlines()[-1])
            self.assertEqual(final['caught_signal'], signum)
            self.assertTrue(final['restored']); self.assertEqual(cleanup.read_text().strip(), '1')
            self.assert_original()
        finally:
            if process.poll() is None: process.kill(); process.wait()

    def test_signal_child_cleanup_precedes_restore_for_all_watched_signals(self):
        for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            with self.subTest(signum=signum):
                self.signal_child_cleanup(signum)
                for path in (self.base / 'ready', self.base / 'cleanup-observed'): path.unlink()


if __name__ == '__main__':
    unittest.main()
