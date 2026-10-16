"""CPU-only native-group benchmark checks; all peers use AF_UNIX sockets."""
import contextlib
from concurrent.futures import Future
import gc
import io
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

from experiments.four_bus_type1 import type1_transport as T
from singularitydog_hw import policy_clock
from . import benchmark, build, client


def _malformed_peer_then_hang(sockets, delay_us, portable, connection):
    connection.send({'kind': 'READY', 'pid': os.getpid(), 'ports': [],
                     'serial_io': False, 'can_io': False})
    time.sleep(30)


class BenchmarkTests(unittest.TestCase):
    def test_default_plan_never_loads_library_or_opens_socket(self):
        output = io.StringIO()
        with (mock.patch.object(benchmark, '_expected_pins', side_effect=AssertionError('library loaded')),
              mock.patch.object(benchmark.socket, 'socketpair', side_effect=AssertionError('socket opened')),
              mock.patch.object(benchmark.mp, 'get_context', side_effect=AssertionError('process started')),
              contextlib.redirect_stdout(output)):
            self.assertEqual(benchmark.main([]), 0)
        row = json.loads(output.getvalue())
        self.assertEqual(row['status'], 'PLAN')
        self.assertFalse(row['loads_library'])
        self.assertFalse(row['starts_threads'])
        self.assertFalse(row['live_qualification'])

    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.library = build.build(Path(cls.temporary.name)/'group')
        record = json.loads((cls.library.parent/'build-record.json').read_text())
        sources = record['four_bus_native_group']['included_source_sha256']
        cls.arguments = [
            '--library', str(cls.library),
            '--expected-sha256', benchmark._sha(cls.library),
            '--ordinary-source-sha256', sources['ordinary'],
            '--subset-stop-source-sha256', sources['subset_stop'],
            '--subset-active-source-sha256', sources['subset_active'],
            '--group-source-sha256', sources['native_group'],
            '--build-record-sha256', benchmark._sha(cls.library.parent/'build-record.json'),
            '--type1-python-sha256', benchmark._sha(T.__file__),
            '--client-python-sha256', benchmark._sha(client.__file__),
            '--policy-clock-python-sha256', benchmark._sha(policy_clock.__file__),
        ]
        cls.lib = client.load_group_library(cls.library,
            expected_sha256=benchmark._sha(cls.library),
            ordinary_source_sha256=sources['ordinary'],
            subset_stop_source_sha256=sources['subset_stop'],
            subset_active_source_sha256=sources['subset_active'],
            group_source_sha256=sources['native_group'],
            build_record_sha256=benchmark._sha(cls.library.parent/'build-record.json'))

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_bad_python_pin_rejected_before_socket_creation(self):
        target = Path(self.temporary.name)/'bad-pin-output'
        args = self.arguments.copy()
        index = args.index('--client-python-sha256')+1
        args[index] = '0'*64
        with mock.patch.object(benchmark.socket, 'socketpair', side_effect=AssertionError('socket opened')):
            with self.assertRaisesRegex(ValueError, 'client source pin differs'):
                benchmark.main(['--execute-cpu', '--portable-socket-fixture', '--output', str(target),
                                '--cycles', '1', *args])
        self.assertFalse(target.exists())

    def test_bad_native_pin_rejected_before_socket_creation(self):
        target = Path(self.temporary.name)/'bad-native-pin-output'
        args = self.arguments.copy()
        args[args.index('--group-source-sha256')+1] = '0'*64
        with mock.patch.object(benchmark.socket, 'socketpair', side_effect=AssertionError('socket opened')):
            with self.assertRaisesRegex(ValueError, 'four native group source pins'):
                benchmark.main(['--execute-cpu', '--portable-socket-fixture', '--output', str(target),
                                '--cycles', '1', *args])
        self.assertFalse(target.exists())

    def test_two_cycle_socket_fixture_has_identical_bytes_and_terminal_stop(self):
        target = Path(self.temporary.name)/'two-cycle-output'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(benchmark.main(['--execute-cpu', '--portable-socket-fixture',
                '--output', str(target), '--cycles', '2', *self.arguments]), 0)
        report = json.loads((target/'report.json').read_text())
        self.assertEqual(report['status'], 'COMPLETE_CPU_FIXTURE_ONLY')
        self.assertFalse(report['live_qualification'])
        self.assertFalse(report['serial_or_can_io'])
        self.assertTrue(report['command_bytes']['identical'])
        for mode in ('python', 'native'):
            variant = report['variants'][mode]
            self.assertEqual(variant['status'], 'COMPLETE')
            self.assertEqual(variant['cycles_completed'], 2)
            self.assertEqual(len(variant['rows']), 2)
            self.assertEqual(variant['metrics']['last_reply']['count'], 2)
            self.assertEqual(variant['metrics']['start_interval']['count'], 1)
            self.assertEqual(variant['metrics']['hold_reply_to_voltage_native_begin']['count'], 8)
            self.assertGreaterEqual(variant['metrics']['hold_reply_to_voltage_native_begin']['median_ms'], 0)
            self.assertTrue(all(row['write_deadline_met'] and row['reply_deadline_met']
                                for row in variant['rows']))
            self.assertTrue(all(item['complete'] for item in variant['stop'].values()))
            pacing = variant['common_pacing']
            self.assertEqual(pacing['before'], pacing['after'])
            self.assertTrue(pacing['restored'])
            self.assertIsNotNone(pacing['pre_cycle_gc_collect']['collected_objects'])
            self.assertFalse(pacing['during']['gc_enabled'])
            self.assertAlmostEqual(pacing['during']['switch_interval_s'], .0001, places=12)
            self.assertFalse(variant['peer']['errors'])
            self.assertTrue(all(value is None for value in variant['owner_errors'].values()))
        self.assertTrue(report['variants']['native']['native_group_close']['snapshot_complete'])
        self.assertTrue(report['post_run_source_pins']['unchanged'])
        self.assertEqual(report['reply_cap_ns'], policy_clock.PolicyClock(30).cycle_budget_ns)
        self.assertEqual(report['period_rule'], 'PolicyClock(30).release_ns(epoch,index)')
        for mode in ('python', 'native'):
            row = report['variants'][mode]['rows'][0]
            self.assertEqual(len(row['hold']['port0']['rx_hex']), 3)
            self.assertEqual(len(row['voltage']['port0']['rx_hex']), 1)
            self.assertEqual(len(row['output']['port0']['rx_hex']), 3)

    def test_owner_setup_error_joins_peer_and_returns_abort_receipt(self):
        before = {process.pid for process in mp.active_children()}
        with mock.patch.object(benchmark, '_Owner', side_effect=RuntimeError('injected setup failure')):
            report = benchmark._run_variant('python', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('injected setup failure', report['error'])
        self.assertEqual(report['cycles_not_started'], 1)
        self.assertEqual(report['peer']['kind'], 'COMPLETE')
        self.assertEqual({process.pid for process in mp.active_children()}, before)

    def test_native_output_failure_retains_partial_cycle_raw_and_restores(self):
        with mock.patch.object(client.NativeGroup, 'output', side_effect=RuntimeError('injected output failure')):
            report = benchmark._run_variant('native', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('injected output failure', report['error'])
        self.assertEqual(report['cycles_partially_observed'], 1)
        self.assertEqual(report['cycles_not_started'], 0)
        self.assertIn('hold', report['rows'][0])
        self.assertIn('voltage', report['rows'][0])
        self.assertNotIn('output', report['rows'][0])
        self.assertTrue(report['native_group_raw_journal'])
        self.assertTrue(all(value['complete'] for value in report['stop'].values()))
        self.assertTrue(report['main_placement']['restored'])
        self.assertTrue(report['common_pacing']['restored'])

    def test_pacing_preserves_pre_disabled_gc_state(self):
        original_enabled = gc.isenabled()
        gc.disable()
        try:
            report = benchmark._run_variant('native', self.lib, cycles=1, delay_us=0, portable=True)
            pacing = report['common_pacing']
            self.assertFalse(pacing['before']['gc_enabled'])
            self.assertFalse(pacing['during']['gc_enabled'])
            self.assertFalse(pacing['after']['gc_enabled'])
            self.assertTrue(pacing['restored'])
            self.assertFalse(gc.isenabled())
        finally:
            if original_enabled:
                gc.enable()

    def test_pacing_restore_readback_failure_aborts_and_retains_raw_and_stop(self):
        original = sys.getswitchinterval()
        real_set = sys.setswitchinterval
        real_set(.004)
        changed = sys.getswitchinterval()
        restore_value = math.nextafter(changed, math.inf)
        def omit_restoration(value):
            if value == restore_value:
                return None
            return real_set(value)
        try:
            with mock.patch.object(benchmark.sys, 'setswitchinterval', side_effect=omit_restoration):
                report = benchmark._run_variant('native', self.lib, cycles=1,
                                                delay_us=0, portable=True)
            self.assertEqual(report['status'], 'ABORTED')
            self.assertIn('Common pacing restoration', report['error'])
            self.assertFalse(report['common_pacing']['restored'])
            self.assertAlmostEqual(report['common_pacing']['after']['switch_interval_s'],
                                   .0001, places=12)
            self.assertEqual(report['common_pacing']['after']['gc_enabled'],
                             report['common_pacing']['before']['gc_enabled'])
            self.assertTrue(report['native_group_raw_journal'])
            self.assertTrue(all(value['complete'] for value in report['stop'].values()))
            self.assertTrue(report['main_placement']['restored'])
        finally:
            real_set(math.nextafter(original, math.inf))
        self.assertEqual(sys.getswitchinterval(), original)

    def test_pacing_restores_even_when_main_placement_exit_raises(self):
        original_placement = benchmark._placement
        class FailingMainPlacement(benchmark._NoPlacement):
            def __exit__(self, *args):
                raise RuntimeError('injected main placement restore failure')
        def placement(cpus, portable):
            return FailingMainPlacement() if cpus == (4,) else original_placement(cpus, portable)
        with mock.patch.object(benchmark, '_placement', side_effect=placement):
            report = benchmark._run_variant('native', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('injected main placement restore failure', report['error'])
        self.assertTrue(report['common_pacing']['restored'])
        self.assertEqual(report['common_pacing']['before'], report['common_pacing']['after'])
        self.assertTrue(report['native_group_raw_journal'])
        self.assertTrue(all(value['complete'] for value in report['stop'].values()))

    def test_native_close_error_does_not_skip_stop_or_main_restoration(self):
        original = client.NativeGroup.close
        def close_then_raise(group):
            original(group)
            raise RuntimeError('injected close-report failure')
        with mock.patch.object(client.NativeGroup, 'close', close_then_raise):
            report = benchmark._run_variant('native', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('injected close-report failure', report['error'])
        self.assertTrue(all(value['complete'] for value in report['stop'].values()))
        self.assertTrue(report['main_placement']['restored'])

    def test_native_restore_failure_report_rejects_completion_but_still_stops(self):
        original = client.NativeGroup.close
        def close_with_failed_restore_report(group):
            report = original(group).copy()
            report['restored'] = False
            report['error'] = 'injected restore failure'
            return report
        with mock.patch.object(client.NativeGroup, 'close', close_with_failed_restore_report):
            report = benchmark._run_variant('native', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('Native group close/restoration/snapshot incomplete', report['error'])
        self.assertFalse(report['native_group_close']['restored'])
        self.assertTrue(all(value['complete'] for value in report['stop'].values()))
        self.assertTrue(report['main_placement']['restored'])

    def test_one_stop_future_failure_preserves_other_stop_results(self):
        original = benchmark._Owner.submit
        def fail_one_stop(owner, name, *arguments):
            if name == 'stop' and owner.port == 'port0':
                future = Future()
                future.set_exception(RuntimeError('injected stop failure'))
                return future
            return original(owner, name, *arguments)
        with mock.patch.object(benchmark._Owner, 'submit', fail_one_stop):
            report = benchmark._run_variant('python', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(set(report['stop']), {'port0', 'port1', 'port2', 'port3'})
        self.assertFalse(report['stop']['port0']['complete'])
        self.assertIn('injected stop failure', report['stop']['port0']['error'])
        self.assertTrue(all(report['stop'][port]['complete'] for port in ('port1', 'port2', 'port3')))
        self.assertTrue(report['main_placement']['restored'])

    def test_malformed_peer_is_bounded_and_terminated(self):
        before = {process.pid for process in mp.active_children()}
        with (mock.patch.object(benchmark, '_serve_peer', _malformed_peer_then_hang),
              mock.patch.object(benchmark, 'PEER_COMPLETE_TIMEOUT_S', .1),
              mock.patch.object(benchmark, 'PEER_JOIN_TIMEOUT_S', .1)):
            report = benchmark._run_variant('python', self.lib, cycles=1, delay_us=0, portable=True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('startup identity differs', report['error'])
        self.assertIn('Virtual peer required termination', report['cleanup_errors'])
        self.assertEqual({process.pid for process in mp.active_children()}, before)

    def test_aborted_first_variant_does_not_hide_second_or_partial_receipt(self):
        target = Path(self.temporary.name)/'partial-output'
        modes = []
        def aborted(mode, library, **kwargs):
            modes.append(mode)
            return {'mode': mode, 'status': 'ABORTED', 'error': 'injected virtual failure',
                    'cycles_requested': 1, 'cycles_completed': 0,
                    'cycles_partially_observed': 1, 'cycles_not_started': 0,
                    'rows': [{'cycle': 0, 'hold': {'port0': {'rx_hex': ['00']}}}],
                    'owner_raw_journal': {'port0': [{'label': 'feedback_hold'}]}}
        with mock.patch.object(benchmark, '_run_variant', side_effect=aborted):
            with self.assertRaisesRegex(ValueError, 'variants aborted'):
                benchmark.main(['--execute-cpu', '--portable-socket-fixture',
                    '--output', str(target), '--cycles', '1', *self.arguments])
        self.assertEqual(modes, ['python', 'native'])
        report = json.loads((target/'report.json').read_text())
        self.assertEqual(report['status'], 'ABORTED_CPU_FIXTURE_ONLY')
        self.assertEqual(set(report['variants']), {'python', 'native'})
        self.assertEqual(report['variants']['native']['rows'][0]['hold']['port0']['rx_hex'], ['00'])
        self.assertFalse(report['command_bytes']['identical'])


if __name__ == '__main__':
    unittest.main()
