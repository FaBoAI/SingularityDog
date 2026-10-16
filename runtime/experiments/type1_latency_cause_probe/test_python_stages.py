"""Real original C++ over AF_UNIX, instrumentation/negative tests; no hardware."""
import contextlib
import io
import json
import gc
import os
from pathlib import Path
import socket
import sys
import time
import tempfile
import unittest
from unittest.mock import patch

from experiments.four_bus_type1 import build as TB
from experiments.four_bus_type1 import type1_transport as T
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.rs05_trial_protocol import enable_request, TrialPhase
from . import probe as P


class PythonStagesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        selected = os.environ.get('FOUR_BUS_TYPE1_TEST_LIBRARY')
        cls.library = Path(selected).resolve() if selected else TB.build(Path(cls.tmp.name) / 'build')
        cls.record = json.loads((cls.library.parent / 'build-record.json').read_bytes())
        cls.scope = cls.record['four_bus_subset_active']

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def args(self, *, ports=1, cycles=2, drop=False):
        values = ['--library', str(self.library), '--library-sha256', P.sha(self.library),
                  '--build-record-sha256', P.sha(self.library.parent / 'build-record.json'),
                  '--transport-source-sha256', P.sha(T.__file__), '--ports', str(ports),
                  '--cycles', str(cycles), '--reply-delay-us', '500', '--start-delay-s', '0',
                  '--output', str(Path(self.tmp.name) / ('run-' + str(len(os.listdir(self.tmp.name)))))]
        for name in ('ordinary_source', 'subset_stop_source', 'extension_source'):
            values += ['--' + name.replace('_', '-') + '-sha256', self.scope[name + '_sha256']]
        if drop:
            values += ['--drop-first-reply']
        return P.parser().parse_args(values)

    def run_probe(self, **kwargs):
        args = self.args(**kwargs)
        args.execute_cpu = True
        with contextlib.redirect_stdout(io.StringIO()):
            report = P.run(args, P.prepare(args))
        self.assertTrue((args.output / 'report.json').is_file())
        return report

    def test_plan_has_no_library_socket_or_thread_capabilities(self):
        args = self.args()
        with patch.object(T, 'load_library', side_effect=AssertionError('CDLL used')), \
             patch.object(socket, 'socketpair', side_effect=AssertionError('socket used')), \
             patch.object(P.threading, 'Thread', side_effect=AssertionError('thread used')):
            pins = P.prepare(args)
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(P.main([*self._argv(args)]), 0)
            self.assertEqual(json.loads(output.getvalue())['status'], 'PLAN')
        self.assertEqual(pins['expected_sha256'], P.sha(self.library))
        self.assertFalse(args.output.exists())

    def _argv(self, args):
        result = []
        for key, value in vars(args).items():
            if value is None or value is False:
                continue
            option = '--' + key.replace('_', '-')
            result.extend([option] if value is True else [option, str(value)])
        return result

    def test_real_original_cpp_single_port_stage_order_and_truthful_scope(self):
        report = self.run_probe()
        self.assertEqual(report['errors'], [])
        self.assertEqual((report['completed_cycles'], report['failed_cycles']), (2, 0))
        self.assertTrue(report['instrumentation_restored'])
        for flag in ('CAN_IO', 'serial_IO', 'motor_enable_sent', 'positive_gain_sent',
                     'learned_targets_sent', 'output_allowed', 'runtime_qualified'):
            self.assertIs(report[flag], False)
        self.assertEqual({f['kind'] for f in report['peer']['frames']}, {1, 17})
        self.assertEqual(len(report['peer']['frames']), 8)
        events = [e for e in report['events'] if e['cycle'] == 0 and e['port'] == 'port0']
        stages = [e['stage'] for e in events]
        required = ['owner_dispatch_enter', 'voltage_buffers_prepare_before', 'voltage_buffers_prepare_after',
                    'checked_exchange_enter', 'native_call_before', 'native_first_python_statement',
                    'checked_decode_before', 'checked_decode_after', 'batch_constructor_before',
                    'batch_constructor_after', 'batch_registry_complete', 'voltage_preflight_hook_before',
                    'prefix_set_result_before', 'prefix_set_result_after', 'voltage_preflight_hook_after',
                    'owner_call_returned', 'outer_set_result_before', 'outer_set_result_after', 'owner_cycle_finally']
        for stage in required:
            self.assertIn(stage, stages)
        position = [stages.index(stage) for stage in required]
        self.assertEqual(position, sorted(position))
        first = next(e for e in events if e['stage'] == 'native_first_python_statement')
        self.assertGreaterEqual(first['wall_ns'], first['native_stats_end_ns'])
        batch = next(e for e in events if e['stage'] == 'batch_constructor_after')
        before = next(e for e in events if e['stage'] == 'batch_constructor_before')
        self.assertLessEqual(before['wall_ns'], batch['batch_argument_stamp_ns'])
        self.assertLessEqual(batch['batch_argument_stamp_ns'], batch['wall_ns'])
        self.assertTrue(all(e['sample_finished_ns'] >= e['sample_started_ns'] for e in events))
        self.assertEqual(len({e['tid'] for e in events}), 1)
        decode = next(r for r in report['intervals'] if r['stage'] == 'checked_decode')
        self.assertGreaterEqual(decode['thread_cpu_ns_delta'], 0)
        self.assertIn('upper bound', decode['counter_scope'])

    def test_four_ports_stable_owner_threads_and_no_enabled_calls(self):
        with patch.object(T.Type1Transport, 'enable', side_effect=AssertionError('enable forbidden')), \
             patch.object(T.Type1Transport, 'zero_gain', side_effect=AssertionError('handshake forbidden')), \
             patch.object(T.Type1Transport, 'output', side_effect=AssertionError('physical output forbidden')):
            report = self.run_probe(ports=4, cycles=1)
        self.assertEqual(report['errors'], [])
        self.assertEqual(len(report['owner_native_tids']), 4)
        self.assertEqual(len(set(report['owner_native_tids'])), 4)
        self.assertEqual(len(report['peer']['frames']), 16)
        self.assertEqual(report['status'], 'COMPLETE_CPU_PROBE')
        self.assertTrue(report['main_placement']['restored'])
        self.assertTrue(all(p['restored'] for p in report['owner_placement'].values()))
        self.assertTrue(report['peer']['placement']['restored'])
        self.assertTrue(report['python_switch_interval']['restored'])
        self.assertTrue(report['gc_scope']['restored'])
        self.assertFalse(report['physical_current_guard_used'])
        self.assertIn('exact_original', report['take_mode'])

    def test_native_timeout_retains_failed_cycle_counters_and_raw_boundary(self):
        report = self.run_probe(drop=True, cycles=2)
        self.assertEqual(report['status'], 'ABORTED_CPU_PROBE')
        self.assertEqual((report['completed_cycles'], report['failed_cycles']), (0, 1))
        stages = {e['stage'] for e in report['events']}
        self.assertIn('native_first_python_statement', stages)
        self.assertIn('owner_cycle_failed', stages)
        self.assertIn('owner_cycle_finally', stages)
        self.assertIn('main_cycle_finally', stages)
        self.assertTrue(report['instrumentation_restored'])
        self.assertTrue(report['owner_raw_journals']['port0'])
        raw = report['owner_raw_journals']['port0'][0]['raw']
        self.assertIn('records', raw)

    def test_instrumentation_restores_methods_after_exception_and_bad_pin(self):
        originals = {n: getattr(T.Type1Transport, n) for n in ('_native', '_checked_exchange', 'hold_then_voltage')}
        recorder = P.Recorder()
        with self.assertRaisesRegex(RuntimeError, 'test failure'):
            with P.instrument(recorder, P.sha(T.__file__)) as ledger:
                self.assertEqual(len(ledger), 3)
                self.assertIsNot(T.Type1Transport._native, originals['_native'])
                raise RuntimeError('test failure')
        self.assertEqual({n: getattr(T.Type1Transport, n) for n in originals}, originals)
        with self.assertRaisesRegex(ValueError, 'SHA differs'):
            with P.instrument(recorder, '0' * 64):
                self.fail('Bad original source entered')
        recorder.close()

    def test_peer_rejects_enable_positive_gain_and_non_voltage_requests(self):
        invalid = (enable_request(phase=TrialPhase.ENABLE, motor_id=1),
                   active.encode_motion(1, 0., 1., 0.), read_request(1, 'position'))
        for raw in invalid:
            with self.subTest(raw=raw.hex()), self.assertRaises(ValueError):
                P.virtual_reply(raw)

    def test_ready_fd_device_and_source_pin_fail_before_socket_creation(self):
        args = self.args()
        with open('/dev/null', 'wb') as device:
            args.ready_fd = device.fileno()
            with patch.object(socket, 'socketpair', side_effect=AssertionError('socket used')):
                with self.assertRaisesRegex(ValueError, 'Ready FD'):
                    P.prepare(args)
        args.ready_fd = None
        args.transport_source_sha256 = '0' * 64
        with self.assertRaisesRegex(ValueError, 'source SHA differs'):
            P.prepare(args)

    def test_counter_disabled_schedstats_never_claims_os_wait_zero(self):
        recorder = P.Recorder(); recorder.bind('test')
        recorder.schedstats_enabled = False
        row = recorder.mark('check')
        self.assertIsNone(row['sched_run_delay_ns'])
        self.assertFalse(row['sched_run_delay_inference_usable'])
        recorder.close()

    def test_minimal_markers_no_counter_sampling_ready_pipe_and_restore(self):
        args = self.args(cycles=1); args.execute_cpu = True; args.counter_detail = 'minimal'
        read, write = os.pipe(); args.ready_fd = write
        old_switch, old_gc = sys.getswitchinterval(), gc.isenabled()
        try:
            gc.disable()
            with contextlib.redirect_stdout(io.StringIO()):
                report = P.run(args, P.prepare(args))
            ready = json.loads(os.read(read, 4096))
            self.assertEqual(ready['pid'], os.getpid())
            self.assertEqual(ready['owner_native_tids'], report['owner_native_tids'])
            self.assertEqual(report['errors'], [])
            self.assertFalse(gc.isenabled())
            self.assertEqual(sys.getswitchinterval(), old_switch)
            self.assertTrue(all(e['thread_cpu_ns'] is None and e['nvcsw'] is None and
                                e['sched_run_delay_ns'] is None for e in report['events']))
            self.assertEqual(report['gc_scope']['before_enabled'], False)
        finally:
            os.close(read); os.close(write)
            if old_gc:
                gc.enable()
            else:
                gc.disable()

    def test_direct_default_plan_cannot_run(self):
        args = self.args()
        with patch.object(socket, 'socketpair', side_effect=AssertionError('socket used')):
            with self.assertRaisesRegex(ValueError, 'execute-cpu'):
                P.run(args, P.prepare(args))

    def test_zero_start_delay_waits_for_peer_ready_before_owner_native_work(self):
        original_ready, original_owner = P.await_peer_ready, P.Owner.__init__
        ready = {}
        def wait(*args):
            result = original_ready(*args)
            ready['time_ns'], ready['message'] = time.monotonic_ns(), result
            return result
        def create_owner(owner, *args):
            self.assertIn('time_ns', ready, 'Owner must not begin before cold-spawn peer is ready')
            original_owner(owner, *args)
        with patch.object(P, 'await_peer_ready', wait), patch.object(P.Owner, '__init__', create_owner):
            report = self.run_probe(ports=4, cycles=1)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['peer_ready']['record_kind'], 'PEER_READY')
        self.assertEqual(report['peer']['record_kind'], 'PEER_COMPLETE_REPORT')
        self.assertEqual(report['peer_ready']['ports'], 4)
        self.assertTrue(all(e['wall_ns'] >= ready['time_ns'] for e in report['events']
                            if e['stage'] == 'native_call_before'))
        self.assertEqual(report['diagnostic_absolute_budget_ns'], 200_000_000)

    def test_peer_readiness_is_bounded_and_checks_exact_identity_and_scope(self):
        class Connection:
            def __init__(self, available, message=None):
                self.available, self.message, self.timeout = available, message, None
            def poll(self, timeout):
                self.timeout = timeout
                return self.available
            def recv(self):
                return self.message
        connection = Connection(False)
        with self.assertRaisesRegex(TimeoutError, '5s startup'):
            P.await_peer_ready(connection, 123, 4)
        self.assertEqual(connection.timeout, 5.)
        valid = {'record_kind': 'PEER_READY', 'pid': 123, 'ports': 4,
                 'CAN_IO': False, 'serial_IO': False, 'output_allowed': False}
        for override in ({'pid': 124}, {'ports': 1}, {'CAN_IO': True}, {'record_kind': 'PEER_COMPLETE_REPORT'}):
            with self.subTest(override=override), self.assertRaises(P.PeerStartupError):
                P.await_peer_ready(Connection(True, dict(valid, **override)), 123, 4)


if __name__ == '__main__':
    unittest.main()
