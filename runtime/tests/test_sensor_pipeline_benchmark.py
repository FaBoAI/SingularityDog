"""Synthetic wires/clocks only: no CAN, I2C, model weights or SSH."""
import contextlib
import copy
import io
import json
from pathlib import Path
import signal
import tempfile
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import sensor_pipeline_benchmark as bench
from test_can_pipeline_probe import Clock as WireClock, Serial, UIDS
from test_policy_observer_live import Clock, seed, logs


class BootGuardTests(unittest.TestCase):
    BOOT = '388804e8-4730-4348-9ef6-519bb9480672'

    def test_one_open_fresh_pread_every_check_then_close_without_fd_reuse(self):
        with patch.object(bench.os, 'open', return_value=123) as opened, \
             patch.object(bench.os, 'pread', return_value=(self.BOOT+'\n').encode()) as read, \
             patch.object(bench.os, 'close') as closed:
            guard = bench.BootIdentityGuard()
            guard.check(); guard.check()
            self.assertEqual(opened.call_count, 1)
            self.assertEqual(read.call_count, 3)
            self.assertTrue(all(c.args == (123, 80, 0) for c in read.call_args_list))
            guard.close(); guard.close()
            closed.assert_called_once_with(123)
            with self.assertRaisesRegex(ValueError, 'closed'): guard.check()
            self.assertEqual(read.call_count, 3)

    def test_invalid_initial_read_and_open_failure_cleanup(self):
        for raw in (b'', b'123', b'\xff'*36, b'388804E8-4730-4348-9ef6-519bb9480672'):
            with self.subTest(raw=raw), patch.object(bench.os, 'open', return_value=123), \
                 patch.object(bench.os, 'pread', return_value=raw), patch.object(bench.os, 'close') as close:
                with self.assertRaises(ValueError): bench.BootIdentityGuard()
                close.assert_called_once_with(123)
        with patch.object(bench.os, 'open', side_effect=OSError('open failed')), \
             patch.object(bench.os, 'close') as close:
            with self.assertRaises(OSError): bench.BootIdentityGuard()
            close.assert_not_called()

    def test_changed_short_and_failed_read_do_not_use_cached_value(self):
        for following in (b'00000000-0000-0000-0000-000000000000', b'', OSError('pread failed')):
            with self.subTest(following=following), patch.object(bench.os, 'open', return_value=123), \
                 patch.object(bench.os, 'pread', side_effect=[self.BOOT.encode(), following]), \
                 patch.object(bench.os, 'close') as close:
                guard = bench.BootIdentityGuard()
                with self.assertRaises((ValueError, OSError)): guard.check()
                guard.close(); close.assert_called_once_with(123)


class PipelineTests(unittest.TestCase):
    def run_worker(self, failure=None, close_error=False):
        clock = WireClock()
        bus, done = bench.live.SessionBus(clock=clock), threading.Event()
        serial = Serial(clock, failure=failure)
        order = []
        if close_error:
            def close():
                order.append('close_failed')
                raise OSError('close failed')
            serial.close = close
        @contextlib.contextmanager
        def lock():
            order.append('lock')
            try: yield
            finally: order.append('unlock')
        def factory(emit, **kwargs):
            self.assertEqual((kwargs['window'], kwargs['gap_ms'], kwargs['cycles'], kwargs['request_order']),
                             (4, .5, 20, 'by-parameter'))
            return bench.pipeline.PipelineCAN(emit, serial_port=serial, **kwargs)
        bench.can_producer(bus, done, UIDS, bench.make_plan(), clock()+10**10,
                           pipeline_factory=factory, lock_factory=lock)
        return bus, done, serial, order, clock

    def test_exact492_reads_and_original_timestamps_pass_through_after_identity_barrier(self):
        bus, done, serial, order, clock = self.run_worker()
        self.assertEqual(bus.errors, [])
        self.assertTrue(done.is_set()); self.assertFalse(bus.stop.is_set())
        self.assertTrue(serial.closed)
        self.assertEqual(order, ['lock', 'unlock'])
        report = bus.producer_status['can']['report']
        bench.require_complete(report)
        self.assertEqual([f.kind for f in serial.writes], [0]*12+[17]*480)
        self.assertTrue(all(r['write_returned_bytes'] == 17 for r in report['requests']))
        rows = logs(bus)
        barrier = next(i for i, r in enumerate(rows) if r['kind'] == 'identity_all12_verified')
        replies = {r['sequence']: r for r in rows if r['kind'] == 'pipeline_reply'}
        telemetry = [r for r in rows if r['kind'] == 'motor_parameter']
        self.assertEqual(len(telemetry), 480)
        self.assertTrue(all(i > barrier for i, r in enumerate(rows) if r['kind'] == 'motor_parameter'))
        for r in telemetry:
            source = replies[r['sequence']]
            self.assertEqual(r['request_monotonic_ns'], source['write_started_monotonic_ns'])
            self.assertEqual(r['monotonic_ns'], source['received_monotonic_ns'])
            self.assertLessEqual(r['monotonic_ns'], r['available_monotonic_ns'])
            self.assertEqual(r['value'], source['result']['value'])

    def test_partial_write_duplicate_timeout_uid_and_residual_fail_closed(self):
        for failure in ('shortwrite', 'duplicate', 'timeout', 'uid', 'boundary_partial'):
            with self.subTest(failure=failure):
                bus, done, serial, order, _ = self.run_worker(failure)
                self.assertTrue(done.is_set()); self.assertTrue(bus.stop.is_set())
                self.assertTrue(bus.errors); self.assertTrue(serial.closed)
                self.assertEqual(order[-1], 'unlock')
                self.assertEqual(bus.producer_status['can']['report']['status'], 'INCOMPLETE')
                self.assertLess(len(serial.writes), 492)

    def test_close_failure_after_complete_collection_is_not_success(self):
        bus, done, _, order, _ = self.run_worker(close_error=True)
        self.assertTrue(bus.errors); self.assertTrue(done.is_set())
        self.assertFalse(bus.producer_status['can']['device_context_exited'])
        self.assertEqual(order, ['lock', 'close_failed', 'unlock'])

    def test_complete_flag_alone_cannot_hide_partial_rows(self):
        bus, _, _, _, _ = self.run_worker()
        original = bus.producer_status['can']['report']
        for change in ('row', 'cycle', 'pending', 'partial'):
            report = copy.deepcopy(original)
            if change == 'row': report['requests'].pop()
            elif change == 'cycle': report['cycles'][0]['replies'] = 23
            elif change == 'pending': report['pending_at_end'] = [{'motor_id': 1}]
            else: report['requests'][12]['write_returned_bytes'] = 16
            with self.assertRaises(ValueError): bench.require_complete(report)

    def test_adapter_rejects_source_mismatch_and_telemetry_before_identity(self):
        bus, _, _, _, _ = self.run_worker()
        source = next(r for r in logs(bus) if r['kind'] == 'pipeline_reply' and r['cycle'] == 1)
        for mutation in ('barrier', 'id', 'index', 'future_write', 'unit'):
            bridge = bench.PipelineEvents(bench.live.SessionBus(clock=Clock()), UIDS)
            if mutation != 'barrier': bridge.identities = set(range(1, 13))
            row = copy.deepcopy(source)
            if mutation == 'id': row['result']['motor_id'] = 12
            elif mutation == 'index': row['result']['index'] = 0x701C
            elif mutation == 'future_write': row['write_finished_monotonic_ns'] = row['received_monotonic_ns']+1
            elif mutation == 'unit': row['result']['unit'] = 'degrees'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): bridge(row)


class ObservationTests(unittest.TestCase):
    def observe(self, *, wait_hook=None, omit=None, max_age=None):
        clock, done = Clock(), threading.Event()
        bus = bench.live.SessionBus(clock=clock)
        plan = bench.make_plan()
        if max_age is not None: plan['max_age_ns'] = max_age
        seed(bus, omit=omit)
        def wait(seconds):
            clock.wait(seconds)
            if wait_hook: wait_hook(clock, bus, done)
            elif clock() >= 1_070_000_000: done.set()
        result = bench.observe(bus, done, plan, clock()+10**10, wait=wait)
        return result, logs(bus)

    def test_no_update_is_reported_not_filled_or_counted_as_initial_success(self):
        result, rows = self.observe()
        self.assertEqual(result['status'], 'SENSOR_OBSERVATIONS_COMPLETE')
        self.assertEqual(result['ticks_completed'], 3)
        self.assertEqual(result['update_comparisons_excluding_initial_baseline'], 2)
        self.assertEqual(result['all24_updated_comparisons'], 0)
        self.assertEqual(result['imu_updated_comparisons'], 0)
        ticks = [r for r in rows if r['kind'] == 'sensor_benchmark_tick']
        self.assertIsNone(ticks[0]['all24_motor_keys_updated'])
        self.assertIsNone(ticks[0]['imu_updated_since_previous_tick'])
        self.assertEqual(ticks[1]['updated_motor_keys_since_previous_tick'], [])
        self.assertEqual(result['inference_calls'], 0)
        self.assertFalse(result['output_allowed'])

    def test_same_values_with_new_timestamps_count_as_updates(self):
        supplied = False
        def wait(clock, bus, done):
            nonlocal supplied
            if clock() >= 1_030_000_000 and not supplied:
                seed(bus, tick=clock(), identity=False, imu_sequence=2)
                supplied = True
            if clock() >= 1_050_000_000: done.set()
        result, rows = self.observe(wait_hook=wait)
        self.assertEqual(result['all24_updated_comparisons'], 1)
        self.assertEqual(result['imu_updated_comparisons'], 1)
        self.assertEqual(result['update_comparisons_excluding_initial_baseline'], 1)

    def test_late_arrival_is_not_used_at_earlier_tick(self):
        supplied = False
        def wait(clock, bus, done):
            nonlocal supplied
            if clock() >= 1_020_000_000 and not supplied:
                clock.now += 1_000_000
                bus.publish({'kind': 'motor_parameter', 'motor_id': 1, 'parameter': 'position',
                    'ok': True, 'status': 0, 'value': 100., 'unit': 'rad',
                    'request_monotonic_ns': 1_019_000_000, 'monotonic_ns': 1_020_000_000},
                    telemetry_input=True)
                supplied = True
            if clock() >= 1_050_000_000: done.set()
        result, rows = self.observe(wait_hook=wait)
        ticks = [r for r in rows if r['kind'] == 'sensor_benchmark_tick']
        get = lambda t: next(r['value'] for r in t['snapshot']['motors'] if r['motor_id']==1 and r['parameter']=='position')
        self.assertNotEqual(get(ticks[0]), 100.)
        self.assertEqual(get(ticks[1]), 100.)  # diagnostic raw sensor, no calibrated pose gate
        self.assertEqual(ticks[1]['updated_motor_keys_since_previous_tick'], [{'motor_id': 1, 'parameter': 'position'}])

    def test_missing_input_stale_and_late_ticks_block_without_catchup(self):
        result, _ = self.observe(omit=(12, 'velocity'))
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertEqual(result['ticks_completed'], 0)
        result, _ = self.observe(max_age=10_000_000)
        self.assertIn('stale', result['failure'])
        def late(clock, bus, done): clock.now += 30_000_000
        result, rows = self.observe(wait_hook=late)
        self.assertIn('deadline missed', result['failure'])
        self.assertEqual(result['ticks_completed'], 0)
        self.assertTrue(any(r['kind']=='sensor_tick_start_check' for r in rows))

    def test_done_does_not_hide_producer_failure(self):
        def fail(clock, bus, done):
            bus.fail('can_pipeline', 'missing reply'); done.set()
        result, _ = self.observe(wait_hook=fail)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertEqual(result['ticks_completed'], 0)

    def test_processing_overrun_does_not_enter_completed_tick_statistics(self):
        clock, done = Clock(), threading.Event()
        bus = bench.live.SessionBus(clock=clock)
        seed(bus)
        publish = bus.publish
        def slow(event, **kwargs):
            publish(event, **kwargs)
            if event['kind'] == 'sensor_benchmark_tick' and event['tick_index'] == 1:
                clock.now += 20_000_000
        bus.publish = slow
        result = bench.observe(bus, done, bench.make_plan(), clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('processing exceeded20ms', result['failure'])
        self.assertEqual(result['ticks_completed'], 1)
        self.assertEqual(result['update_comparisons_excluding_initial_baseline'], 0)
        self.assertEqual(result['statistics_ms']['start_lateness']['count'], 1)
        self.assertIsNone(result['updated_motor_key_count'])


class AcquisitionTests(unittest.TestCase):
    def test_final_gate_requires_can_complete_close_and_imu_restore(self):
        worker = PipelineTests()
        captured, _, _, _, _ = worker.run_worker()
        complete = captured.producer_status['can']['report']
        for mutation in ('none', 'close', 'restore', 'report', 'observer', 'error'):
            with self.subTest(mutation=mutation):
                bus = bench.live.SessionBus()
                def can(bus, done, *args, **kwargs):
                    report = copy.deepcopy(complete)
                    if mutation == 'report': report['replies'] -= 1
                    bus.producer_status['can'] = {'exited': True,
                        'device_context_exited': mutation != 'close',
                        'cross_process_locks_acquired': True, 'report': report}
                    if mutation == 'error': bus.fail('test', 'worker failure')
                    done.set()
                def imu(bus, *args, **kwargs):
                    bus.producer_status['imu'] = {'exited': True,
                        'restore_status': 'failed' if mutation == 'restore' else 'restored'}
                with patch.object(bench, 'can_producer', can), \
                     patch.object(bench.live, 'imu_producer', imu), \
                     patch.object(bench, 'observe', return_value={'status': 'INCOMPLETE' if mutation == 'observer'
                                                                 else 'SENSOR_OBSERVATIONS_COMPLETE'}):
                    result = bench.run_acquisition(bus, UIDS, bench.make_plan())
                self.assertEqual(result['status'], 'COMPLETE_SENSOR_BENCHMARK_NO_OUTPUT'
                                 if mutation == 'none' else 'INCOMPLETE')
                self.assertTrue(result['producer_threads_exited'])

    def test_thread_start_failure_stops_and_joins_started_worker(self):
        bus = bench.live.SessionBus()
        entered, exited = threading.Event(), threading.Event()
        def can(bus, *args, **kwargs):
            entered.set()
            bus.stop.wait(1.)
            exited.set()
        original = threading.Thread.start
        def start(thread):
            if thread.name == 'sensor-pipeline-imu':
                self.assertTrue(entered.wait(1.))
                raise RuntimeError('second thread unavailable')
            return original(thread)
        with patch.object(bench, 'can_producer', can), patch.object(threading.Thread, 'start', start):
            with self.assertRaisesRegex(RuntimeError, 'second thread'):
                bench.run_acquisition(bus, UIDS, bench.make_plan())
        self.assertTrue(bus.stop.is_set())
        self.assertTrue(exited.is_set())


class CLITests(unittest.TestCase):
    def test_default_plan_is_fixed_without_file_device_or_model_access(self):
        with patch.object(bench, 'run_acquisition', side_effect=AssertionError('hardware')), \
             patch.object(bench.live.shadow, 'load_policy', side_effect=AssertionError('model')), \
             contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(bench.main(['--expected-uids', '/missing', '--output', '/missing']), 0)
        plan = json.loads(out.getvalue())
        self.assertEqual([plan['can'][x] for x in ('cycles', 'window', 'gap_ms', 'request_order')],
                         [20, 4, .5, 'by-parameter'])
        self.assertEqual(plan['inference_calls'], 0)
        self.assertFalse(plan['output_allowed'])

    def test_budget_rejects_nonfinite_and_above10seconds(self):
        for value in (0, 11, float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError): bench.make_plan(value)

    def test_complete_cleanup_signal_and_start_failure_are_durable_without_model(self):
        for mode in ('complete', 'signal', 'start_failure', 'writer_close_failure'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                root = Path(directory); uid = root/'uids.json'; uid.write_text(json.dumps(UIDS))
                original_read, original_close = Path.read_text, bench.live.AuditWriter.close
                def read(path, *a, **kw):
                    if str(path) == '/proc/sys/kernel/random/boot_id': return 'fake-boot'
                    return original_read(path, *a, **kw)
                def close(writer):
                    if mode == 'signal': signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                    result = original_close(writer)
                    if mode == 'writer_close_failure': raise OSError('writer close failure')
                    return result
                def acquire(bus, *a, **kw):
                    bus.publish({'kind': 'synthetic_test_only'})
                    return {'status': 'COMPLETE_SENSOR_BENCHMARK_NO_OUTPUT'}
                start = patch.object(threading.Thread, 'start', side_effect=RuntimeError('no thread')) \
                    if mode == 'start_failure' else contextlib.nullcontext()
                with patch.object(Path, 'read_text', read), \
                     patch.object(bench, 'BootIdentityGuard') as guard_type, \
                     patch.object(bench.live.AuditWriter, 'close', close), \
                     patch.object(bench, 'run_acquisition', acquire), start, \
                     patch.object(bench.live.shadow, 'load_policy', side_effect=AssertionError('model')), \
                     contextlib.redirect_stdout(io.StringIO()):
                    guard_type.return_value.boot_id = BootGuardTests.BOOT
                    rc = bench.main(['--execute-no-output', '--expected-uids', str(uid), '--output', str(root/'out')])
                    guard_type.return_value.close.assert_called_once()
                report = json.loads((root/'out/summary.json').read_text())
                self.assertEqual(rc, 0 if mode == 'complete' else 2)
                self.assertEqual(report['status'], 'COMPLETE_SENSOR_BENCHMARK_NO_OUTPUT' if mode == 'complete' else 'INCOMPLETE')
                self.assertEqual((root/'out/summary.json').stat().st_mode & 0o777, 0o600)
                self.assertEqual(report['inference_calls'], 0)
                self.assertFalse(report['output_allowed'])


if __name__ == '__main__':
    unittest.main()
