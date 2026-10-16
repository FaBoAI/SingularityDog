from pathlib import Path
import hashlib
import json
import os
import shutil
import signal
import struct
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

from . import trace_scope as scope
from .trace_scope import elf_offset
from .trace_analyze import analyze


class TraceTests(unittest.TestCase):
    def test_reviewed_arm64_pac_uses_only_normal_instruction_probes(self):
        header = bytearray(128)
        header[:6] = b'\x7fELF\x02\x01'
        struct.pack_into('<H', header, 18, 183)
        words = [0xd503233f, 0xa9be7bfd, 0x910003fd, 0xf9000bf3,
                 0xb4000100, 0xaa0003f3, 0x97fffd56, 0xaa1303e0,
                 0xf9400bf3, 0xa8c27bfd, 0xd50323bf, 0x1400390d]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'synthetic-arm64-elf'
            path.write_bytes(header + struct.pack('<12I', *words))
            # Artificial instruction fixture is never executed or confused with
            # the actual binary; only this isolated test pins its fake digest.
            with patch.object(scope, 'sha256', return_value=scope.ARM64_PAC_EXEC_SHA256):
                result = scope.probe_points(path, 128)
                self.assertFalse(result['lr_hijack_used'])
                self.assertEqual([(p['kind'], p['name'], p['file_offset']) for p in result['points']],
                                 [('p', 'acquire_entry', 132), ('p', 'acquire_exit', 172)])
                self.assertIn('excludes thread-state', result['semantics'])
                words[1] = 0xd503201f
                path.write_bytes(header + struct.pack('<12I', *words))
                with self.assertRaisesRegex(ValueError, 'instruction layout differs'):
                    scope.probe_points(path, 128)
            with self.assertRaisesRegex(ValueError, 'not independently reviewed'):
                scope.probe_points(path, 128)

    def test_kernel_command_is_one_write_without_text_append_seek(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'synthetic-events'
            path.write_bytes(b'existing\n')
            with patch.object(Path, 'open', side_effect=AssertionError('Text append seek used')):
                scope.kernel_command(path, 'p:synthetic/entry /synthetic:0x4\n')
            self.assertEqual(path.read_text(), 'existing\np:synthetic/entry /synthetic:0x4\n')
            with patch.object(scope.os, 'write', return_value=1) as write:
                with self.assertRaisesRegex(OSError, 'Incomplete atomic'):
                    scope.kernel_command(path, '-:synthetic/entry\n')
                self.assertEqual(write.call_count, 1)
            with patch.object(scope.os, 'open') as opened:
                with self.assertRaisesRegex(ValueError, 'Exactly one'):
                    scope.kernel_command(path, 'one\ntwo\n')
                opened.assert_not_called()

    def test_acquisition_and_legacy_return_points_never_cross_pair(self):
        trace = '''worker-11 [001] d... 1.000000: acquire_entry: (0x1)
worker-11 [001] d... 1.000100: restore_return: (0x2)
worker-11 [001] d... 1.000200: acquire_exit: (0x3)
'''
        report = analyze(trace)
        self.assertEqual(report['interval_count'], 1)
        row = report['intervals'][0]
        self.assertEqual(row['duration_ns'], 200_000)
        self.assertEqual(row['endpoint_kind'], 'acquire_path')
        self.assertFalse(row['lr_hijack_used'])
        self.assertIn('excludes thread-state', row['endpoint_semantics'])
        self.assertEqual(len(report['unmatched_returns']), 1)

    def test_elf_symbol_address_is_not_a_file_offset(self):
        header = bytearray(64)
        header[:6] = b'\x7fELF\x02\x01'
        struct.pack_into('<Q', header, 32, 64)
        struct.pack_into('<HH', header, 54, 56, 1)
        segment = struct.pack('<IIQQQQQQ', 1, 5, 0x1000, 0x401000, 0, 0x1000, 0x1000, 0x1000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'elf'
            path.write_bytes(header + segment)
            self.assertEqual(elf_offset(path, 0x401234), 0x1234)
            with self.assertRaises(ValueError):
                elf_offset(path, 0x501234)

    def test_blocked_and_runnable_are_separate(self):
        trace = '''worker-11 [001] d... 1.000000: restore_entry: (0x1)
worker-11 [001] d... 1.000100: sched_switch: prev_comm=worker prev_pid=11 prev_prio=120 prev_state=S ==> next_comm=x next_pid=12 next_prio=120
x-12 [001] d... 1.000600: sched_wakeup: comm=worker pid=11 prio=120 target_cpu=001
x-12 [001] d... 1.000800: sched_switch: prev_comm=x prev_pid=12 prev_prio=120 prev_state=R ==> next_comm=worker next_pid=11 next_prio=120
worker-11 [001] d... 1.001000: restore_return: (0x2 <- 0x1)
'''
        report = analyze(trace, {11})
        row = report['intervals'][0]
        self.assertEqual(row['duration_ns'], 1_000_000)
        self.assertEqual(row['sleeping_ns'], 500_000)
        self.assertEqual(row['runnable_wait_ns'], 200_000)
        self.assertEqual(row['on_cpu_or_unobserved_ns'], 300_000)
        self.assertFalse(report['pure_gil_wait_proven'])

    def test_missing_wakeup_stays_unknown(self):
        trace = '''worker-11 [001] d... 1.000000: restore_entry: (0x1)
worker-11 [001] d... 1.000100: sched_switch: prev_pid=11 prev_state=S ==> next_pid=12
x-12 [001] d... 1.000800: sched_switch: prev_pid=12 prev_state=R ==> next_pid=11
worker-11 [001] d... 1.001000: restore_return: (0x2)
worker-11 [001] d... 1.002000: restore_return: (0x2)
worker-11 [001] d... 1.003000: restore_entry: (0x1)
'''
        report = analyze(trace)
        self.assertEqual(report['intervals'][0]['unknown_off_cpu_ns'], 700_000)
        self.assertEqual(len(report['unmatched_returns']), 1)
        self.assertEqual(len(report['unmatched_entries']), 1)


class ReversibleScopeTests(unittest.TestCase):
    """Fake tracefs and fake child only: no root, tracing or process is changed."""
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        self.tracing = self.root / 'tracefs'
        (self.tracing / 'instances').mkdir(parents=True)
        (self.tracing / 'uprobe_events').write_text('p:unrelated/event /unrelated:0x1\n')
        self.schedstats = self.root / 'schedstats'
        self.schedstats.write_text('0\n')
        self.runtime = self.root / 'runtime'
        source = self.runtime / 'experiments/type1_latency_cause_probe/probe.py'
        source.parent.mkdir(parents=True)
        source.write_text('# SYNTHETIC test marker; never executed\n')
        self.source = source
        self.python = self.root / 'python'
        self.python.write_bytes(b'SYNTHETIC interpreter; never executed')
        self.args = SimpleNamespace(output=str(self.root / 'fresh-output'),
            runtime_root=str(self.runtime), python=str(self.python),
            probe_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            execute_cpu_trace=True, timeout_s=1.)

    def fake_run(self, failure=None):
        original_mkdir = Path.mkdir
        original_write = scope.write
        original_kernel_command = scope.kernel_command
        original_rmdir = Path.rmdir
        order, stop_writes = [], [0]
        child = SimpleNamespace(pid=999991, running=True, exit_code=None, wait_calls=0)

        def mkdir(path, *args, **kwargs):
            existed = path.exists()
            result = original_mkdir(path, *args, **kwargs)
            if not existed and path.parent == self.tracing / 'instances':
                members = ['tracing_on', 'buffer_size_kb', 'trace_clock', 'events/enable',
                    'events/sched/sched_switch/enable', 'events/sched/sched_wakeup/enable',
                    f'events/{path.name}/restore_entry/enable', f'events/{path.name}/restore_return/enable',
                    'trace', 'per_cpu/cpu0/stats']
                for relative in members:
                    member = path / relative
                    member.parent.mkdir(parents=True, exist_ok=True)
                    member.write_text('overrun: 0\ncommit overrun: 0\ndropped events: 0\n'
                                      if relative.endswith('/stats') else '0\n')
            return result

        def write(path, value):
            path = Path(path)
            order.append(('write', path.name, str(value)))
            if path.name == 'tracing_on' and value == 0:
                stop_writes[0] += 1
                if failure == 'cleanup_stop_write' and stop_writes[0] == 3:
                    raise OSError('Synthetic tracing_on cleanup failure')
            original_write(path, '[mono]' if path.name == 'trace_clock' else value)

        def rmdir(path):
            if path.parent == self.tracing / 'instances':
                order.append(('remove_instance', path.name))
                shutil.rmtree(path)
            else:
                original_rmdir(path)

        def kernel_command(path, command):
            if failure == 'register_probe' and command.startswith('p:'):
                raise OSError(22, 'Synthetic tracefs command failure')
            original_kernel_command(path, command)

        def wait(timeout=None):
            child.wait_calls += 1
            if failure == 'signal_and_ignored_term':
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                # The second signal during cleanup must not interrupt restoration.
                if child.running:
                    raise subprocess.TimeoutExpired('synthetic CPU child', timeout)
            elif failure == 'timeout_and_ignored_term' and child.wait_calls <= 2:
                raise subprocess.TimeoutExpired('synthetic CPU child', timeout)
            if failure == 'source_removed':
                self.source.unlink(missing_ok=True)
            child.running = False
            child.exit_code = 7 if failure == 'child_failed' else child.exit_code or 0
            return child.exit_code

        def killpg(pid, sig):
            self.assertEqual(pid, child.pid)
            order.append(('kill', sig))
            if sig == signal.SIGKILL:
                child.running = False
                child.exit_code = -signal.SIGKILL

        child.wait = wait
        child.poll = lambda: None if child.running else child.exit_code
        original_handlers = {sig: signal.getsignal(sig) for sig in
                             (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        with patch.object(scope, 'TRACE_ROOT', self.tracing), \
             patch.object(scope, 'SCHEDSTATS', self.schedstats), \
             patch.object(scope, 'symbol_offset', return_value=0x1234), \
             patch.object(scope.os, 'geteuid', return_value=0), \
             patch.dict(scope.os.environ, {'SUDO_UID': '12345'}), \
             patch.object(scope.pwd, 'getpwuid', return_value=SimpleNamespace(
                 pw_gid=12345, pw_dir='/SYNTHETIC', pw_name='synthetic')), \
             patch.object(scope.os, 'chown') as chown, \
             patch.object(scope.os, 'killpg', side_effect=killpg), \
             patch.object(scope.subprocess, 'Popen', return_value=child) as spawned, \
             patch.object(Path, 'mkdir', mkdir), patch.object(Path, 'rmdir', rmdir), \
             patch.object(scope, 'kernel_command', side_effect=kernel_command), \
             patch.object(scope, 'write', side_effect=write):
            report = scope.run(self.args, ['--execute-cpu'])
        if failure != 'register_probe':
            self.assertEqual(spawned.call_args.args[0][0], self.args.python)
        else:
            spawned.assert_not_called()
        self.assertEqual(self.schedstats.read_text().strip(), '0')
        self.assertFalse(list((self.tracing / 'instances').iterdir()))
        self.assertEqual({sig: signal.getsignal(sig) for sig in original_handlers}, original_handlers)
        self.assertEqual(json.loads((Path(self.args.output) / 'trace-report.json').read_text()), report)
        self.assertTrue((Path(self.args.output) / 'trace-recovery.json').exists())
        self.assertTrue(all(call.kwargs.get('follow_symlinks') is False
                            for call in chown.call_args_list[1:]))
        events = (self.tracing / 'uprobe_events').read_text()
        self.assertTrue(events.startswith('p:unrelated/event '))
        self.assertNotIn('-:unrelated/event', events)
        if failure != 'register_probe':
            self.assertIn('-:dogcause_', events)
        return report, order

    def test_normal_scope_restores_only_owned_resources_and_private_report(self):
        report, _ = self.fake_run()
        self.assertEqual(report['status'], 'RECORDED_CPU_ONLY_TRACE')
        self.assertTrue(report['restored'] and report['child_reaped'])
        self.assertTrue(report['source_unchanged'] and report['child_workload_completed'])
        self.assertFalse(report['output_allowed'] or report['motor_hardware_opened'])
        self.assertEqual(os.stat(Path(self.args.output) / 'trace-report.json').st_mode & 0o777, 0o600)

    def test_venv_invocation_keeps_symlink_path_but_probes_resolved_elf(self):
        invoked = self.root / 'venv/bin/python'
        invoked.parent.mkdir(parents=True)
        invoked.symlink_to(self.python)
        self.args.python = str(invoked)
        report, _ = self.fake_run()
        self.assertEqual(report['invoked_python'], str(invoked))
        self.assertEqual(report['elf_python'], str(self.python))
        self.assertTrue(report['invoked_python_differs_from_elf'])
        events = (self.tracing / 'uprobe_events').read_text()
        self.assertIn(str(self.python) + ':0x1234', events)
        self.assertNotIn(str(invoked) + ':0x1234', events)

    def test_sigterm_and_repeated_signal_reap_ignored_child_and_restore_handlers(self):
        report, order = self.fake_run('signal_and_ignored_term')
        self.assertEqual(report['status'], 'FAILED_CPU_ONLY_TRACE')
        self.assertTrue(report['restored'] and report['child_reaped'])
        self.assertGreaterEqual(len(report['signals_received']), 2)
        self.assertIn(('kill', signal.SIGTERM), order)
        self.assertIn(('kill', signal.SIGKILL), order)

    def test_timeout_escalates_to_sigkill_before_restoring_tracing(self):
        report, order = self.fake_run('timeout_and_ignored_term')
        self.assertEqual(report['status'], 'FAILED_CPU_ONLY_TRACE')
        self.assertTrue(report['restored'] and report['child_reaped'])
        self.assertIn(('kill', signal.SIGKILL), order)
        self.assertLess(order.index(('kill', signal.SIGKILL)),
                        next(i for i, row in enumerate(order) if row[0] == 'remove_instance'))

    def test_failed_cleanup_write_still_disables_removes_and_restores_schedstats(self):
        report, order = self.fake_run('cleanup_stop_write')
        self.assertFalse(report['restored'])
        self.assertTrue(any('instance stop' in error for error in report['cleanup_errors']))
        self.assertIn(('write', 'enable', '0'), order)
        self.assertTrue(any(row[0] == 'remove_instance' for row in order))

    def test_missing_final_source_retains_failure_report_after_restoration(self):
        report, _ = self.fake_run('source_removed')
        self.assertTrue(report['restored'])
        self.assertFalse(report['source_unchanged'])
        self.assertEqual(report['status'], 'FAILED_CPU_ONLY_TRACE')
        self.assertIn('source_recheck_error', report)

    def test_failed_child_trace_is_retained_without_success_exit(self):
        report, _ = self.fake_run('child_failed')
        self.assertEqual(report['status'], 'RECORDED_CPU_ONLY_TRACE_CHILD_FAILED')
        self.assertTrue(report['restored'])
        self.assertFalse(report['child_workload_completed'])

    def test_registration_failure_retains_exact_step_and_traceback(self):
        report, _ = self.fake_run('register_probe')
        self.assertEqual(report['status'], 'FAILED_CPU_ONLY_TRACE')
        self.assertEqual(report['setup_step'], 'register_probe_restore_entry')
        self.assertIn('Synthetic tracefs command failure', report['traceback'])
        self.assertTrue(report['restored'])

    def test_combined_hardware_flags_rejected_before_trace_mutation(self):
        for flag in ('--front-port=/dev/SYNTHETIC', '--execute-supported=true'):
            with self.subTest(flag=flag), patch.object(scope, 'write') as mutated:
                with self.assertRaisesRegex(ValueError, 'Hardware execution'):
                    scope.run(self.args, ['--execute-cpu', flag])
                mutated.assert_not_called()
                self.assertFalse(Path(self.args.output).exists())

    def test_plan_opens_no_trace_child_or_signal_scope(self):
        self.args.execute_cpu_trace = False
        with patch.object(scope, 'symbol_offset', return_value=0x1234), \
             patch.object(scope, 'write') as mutated, \
             patch.object(scope.subprocess, 'Popen') as child, \
             patch.object(scope.signal, 'signal') as handlers:
            result = scope.run(self.args, ['--execute-cpu'])
        self.assertEqual(result['status'], 'PLAN')
        for value in (mutated, child, handlers):
            value.assert_not_called()
        self.assertFalse(Path(self.args.output).exists())


if __name__ == '__main__':
    unittest.main()
