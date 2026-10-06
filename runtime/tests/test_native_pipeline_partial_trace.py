"""Synthetic transports only: failed slots remain separate from measurements."""
from concurrent.futures import wait
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device
from test_native_voltage_fast_pipeline import PreparedSession
from test_native_voltage_overlap import OverlapObserver


class PartialCycleTraceTests(unittest.TestCase):
    def collect(self, *, cycles=2, inference_failure=False):
        started = (threading.Event(), threading.Event())
        release = threading.Event()
        sessions = {scope: PreparedSession(started[i], release)
                    for i, scope in enumerate(('front', 'rear'))}
        observer = OverlapObserver(started, release, fail=inference_failure)
        report, raw = bench.collect(sessions, Device(), observer,
            mode='stop-proxy', cycles=cycles, record_storage='trace',
            v3_voltage_proxy=True, v3_voltage_overlap=True,
            v3_voltage_validation_overlap=True, v3_voltage_fast_pipeline=True,
            output_dispatch_trace=True, inference_thread_cpu_trace=True,
            absolute_epoch_cadence=True,
            deadline_wait=lambda target: time.sleep(max(0, target-time.monotonic_ns())/1e9))
        return report, bench._serialize(raw)

    def test_failed_second_join_keeps_partial_traces_without_completed_measurement(self):
        original = bench._await_output_ready
        joins = []
        def fail_second(futures, **options):
            joins.append(options['deadline_ns'])
            if len(joins) == 1:
                return original(futures, **options)
            # Test-only failure injection after both synthetic owners finish.
            _, pending = wait(tuple(futures.values()), timeout=.5)
            self.assertFalse(pending)
            raise TimeoutError('synthetic second output join failure')
        with patch.object(bench, '_await_output_ready', side_effect=fail_second):
            report, raw = self.collect()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 1)
        self.assertEqual(len(report['measurements']), 1)
        self.assertEqual(len(report['output_dispatch_trace']['rows']), 1)
        self.assertEqual(len(report['inference_thread_cpu_trace']['rows']), 1)
        self.assertEqual(report['record_storage']['completed_trace_rows'], 1)
        self.assertEqual(len(raw), 2)
        self.assertNotIn('observed', raw[1])
        self.assertEqual(len(report['incomplete_cycle_traces']), 1)
        partial = report['incomplete_cycle_traces'][0]
        self.assertEqual(partial['cycle'], 2)
        self.assertIs(partial['complete_measurement'], False)
        self.assertIs(partial['output_allowed'], False)
        dispatch = dict(zip(partial['output_dispatch_trace']['fields'],
                            partial['output_dispatch_trace']['row']))
        for scope in ('front', 'rear'):
            self.assertGreater(dispatch[scope+'_worker_enter_ns'], 0)
            self.assertGreaterEqual(dispatch[scope+'_worker_check_end_ns'],
                                    dispatch[scope+'_worker_enter_ns'])
            # These existing slots are populated only after successful takeout.
            self.assertIsNone(dispatch[scope+'_native_begin_ns'])
            self.assertIsNone(dispatch[scope+'_first_write_ns'])
        cpu = partial['inference_thread_cpu_trace']['row']
        self.assertGreater(cpu[0], 0)
        self.assertGreaterEqual(cpu[1], cpu[0])
        self.assertNotIn('inference_wall_ns', partial)
        failure = partial['output_join_failure_proof']
        self.assertEqual(failure['stage'], 'readiness_join')
        self.assertIn('synthetic second output join failure', failure['reason'])
        self.assertGreater(failure['captured_ns'], 0)
        self.assertIs(failure['capture_is_deadline_decision_time'], False)
        self.assertIs(failure['native_reply_time_inferred'], False)
        self.assertEqual(failure['owner_future_states'],
                         {scope: {'done': True, 'cancelled': False}
                          for scope in ('front', 'rear')})
        self.assertEqual(raw[1]['voltage_fast_pipeline']['output_join_failure_proof'], failure)
        failure['owner_future_states']['front']['done'] = False
        self.assertIs(raw[1]['voltage_fast_pipeline']['output_join_failure_proof']
                      ['owner_future_states']['front']['done'], True)
        for key in ('motor_enable_sent', 'learned_targets_sent',
                    'approved_for_runtime', 'full_controller_50Hz_verified'):
            self.assertIs(report[key], False)

    def test_early_failure_preserves_unset_cpu_end_and_success_counts_stay_complete(self):
        failed, raw = self.collect(cycles=1, inference_failure=True)
        self.assertEqual(failed['status'], 'ABORTED')
        self.assertEqual(failed['cycles_completed'], 0)
        self.assertEqual(failed['measurements'], [])
        self.assertEqual(failed['output_dispatch_trace']['rows'], [])
        self.assertEqual(failed['inference_thread_cpu_trace']['rows'], [])
        partial = failed['incomplete_cycle_traces'][0]
        self.assertEqual(partial['cycle'], 1)
        self.assertNotIn('output_dispatch_trace', partial)
        self.assertGreater(partial['inference_thread_cpu_trace']['row'][0], 0)
        self.assertIsNone(partial['inference_thread_cpu_trace']['row'][1])
        self.assertNotIn('output_join_failure_proof', partial)
        self.assertEqual(raw[0]['output'], {})
        complete, raw = self.collect()
        self.assertEqual(complete['status'], 'COMPLETE_DIAGNOSTIC', complete['errors'])
        self.assertEqual(complete['cycles_completed'], 2)
        self.assertEqual(len(complete['measurements']), 2)
        self.assertEqual(len(complete['output_dispatch_trace']['rows']), 2)
        self.assertEqual(len(complete['inference_thread_cpu_trace']['rows']), 2)
        self.assertEqual(complete['incomplete_cycle_traces'], [])
        self.assertEqual(complete['record_storage']['completed_trace_rows'], 2)
        self.assertEqual(len(raw), 2)
        self.assertFalse(complete['motor_enable_sent'])
        self.assertFalse(complete['learned_targets_sent'])


if __name__ == '__main__':
    unittest.main()
