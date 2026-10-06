"""No-device coverage for STOP-proxy output readiness and retained cleanup."""
from concurrent.futures import Future, ThreadPoolExecutor
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device
from test_native_voltage_fast_pipeline import PreparedSession
from test_native_voltage_overlap import OverlapObserver


class Clock:
    def __init__(self): self.now = 1000
    def __call__(self): return self.now


class NativeOutputReadinessTests(unittest.TestCase):
    def owners(self, ready=False):
        values = {scope:Future() for scope in ('front', 'rear')}
        if ready:
            for scope, future in values.items(): future.set_result(scope)
        return values

    def test_ready_join_never_takes_or_replaces_owner_results(self):
        owners = self.owners(True); native = Mock()
        with (patch.object(owners['front'], 'result', side_effect=AssertionError('No takeout')),
              patch.object(owners['rear'], 'result', side_effect=AssertionError('No takeout'))):
            proof = bench._await_output_ready(owners, deadline_ns=1_000_000,
                clock=Clock(), deadline_wait=native, thread_clock=lambda:10)
        native.assert_not_called()
        self.assertEqual(proof['wait_calls'], 0)
        self.assertEqual(owners['front'].result(), 'front')

    def test_native_wait_requires_both_owners_with_bounded_targets(self):
        owners = self.owners(); clock = Clock(); targets = []
        def native(target):
            self.assertLessEqual(target-clock.now, 200_000)
            targets.append(target); clock.now = target
            owners['front' if len(targets) == 1 else 'rear'].set_result('done')
        with patch.object(bench, 'wait', side_effect=AssertionError('No condition wait')):
            proof = bench._await_output_ready(owners, deadline_ns=1_000_000,
                clock=clock, deadline_wait=native)
        self.assertEqual(targets, [201000, 401000])
        self.assertTrue(proof['future_results_taken_only_after_ready'])

    def test_deadline_is_clipped_and_readiness_at_equality_rejected(self):
        owners = self.owners(); clock = Clock(); targets = []
        def native(target):
            targets.append(target); clock.now = target
            if target == 301000:
                for future in owners.values(): future.set_result('done')
        with self.assertRaises(TimeoutError):
            bench._await_output_ready(owners, deadline_ns=301000, clock=clock, deadline_wait=native)
        self.assertLess(len(targets), 32)
        self.assertEqual(targets[-1], 301000)
        self.assertTrue(all(b > a for a, b in zip([1000]+targets, targets)))
        self.assertTrue(all(b-a <= 50_000 for a, b in zip([1000]+targets, targets)))

    def test_owner_failure_or_cancellation_preempts_other_unfinished_owner(self):
        for cancel in (False, True):
            owners = self.owners(); native = Mock()
            if cancel: owners['rear'].cancel()
            else: owners['rear'].set_exception(OSError('rear failed'))
            with self.subTest(cancel=cancel), self.assertRaises((RuntimeError, OSError)):
                bench._await_output_ready(owners, deadline_ns=1_000_000, clock=Clock(), deadline_wait=native)
            native.assert_not_called(); self.assertFalse(owners['front'].done())

    def test_signal_check_and_native_cancellation_preserve_original_failure(self):
        for published in (False, True):
            owners = self.owners(); failure = OSError('owner failed')
            def native(target):
                if published: owners['front'].set_exception(failure)
                raise KeyboardInterrupt('cancelled')
            with self.subTest(published=published), self.assertRaises(OSError if published else KeyboardInterrupt):
                bench._await_output_ready(owners, deadline_ns=1_000_000, clock=Clock(), deadline_wait=native)
        with self.assertRaisesRegex(RuntimeError, 'signal'):
            bench._await_output_ready(self.owners(True), deadline_ns=1_000_000,
                clock=Clock(), check=Mock(side_effect=RuntimeError('signal cancellation')))

    def test_early_backdated_or_overslept_native_return_is_rejected(self):
        for returned in (200999, 999, 1_000_000):
            owners = self.owners(); clock = Clock()
            def native(target):
                clock.now = returned
                for future in owners.values(): future.set_result('done')
            with self.subTest(returned=returned), self.assertRaises((ValueError, TimeoutError)):
                bench._await_output_ready(owners, deadline_ns=1_000_000, clock=clock, deadline_wait=native)

    def test_wrong_bus_nonfuture_alias_and_invalid_deadline_rejected(self):
        owners = self.owners(True)
        for values, deadline in (({'front':owners['front']}, 1_000_000),
                                ({'front':owners['front'], 'wrong':owners['rear']}, 1_000_000),
                                ({'front':owners['front'], 'rear':object()}, 1_000_000),
                                ({'front':owners['front'], 'rear':owners['front']}, 1_000_000),
                                (owners, True)):
            with self.subTest(deadline=deadline), self.assertRaises(ValueError):
                bench._await_output_ready(values, deadline_ns=deadline, clock=Clock())

    def test_fallback_is_finite_and_completion_clock_is_not_backdated(self):
        owners = self.owners(); clock = Clock()
        def complete(values, *, timeout, return_when):
            self.assertEqual(set(values), set(owners.values()))
            self.assertEqual(return_when, bench.FIRST_EXCEPTION)
            self.assertAlmostEqual(timeout, .000999)
            for future in values: future.set_result('done')
        with patch.object(bench, 'wait', side_effect=complete) as waiter:
            bench._await_output_ready(owners, deadline_ns=1_000_000, clock=clock)
        self.assertEqual(waiter.call_count, 1)
        with self.assertRaisesRegex(ValueError, 'Noncausal'):
            bench._await_output_ready(owners, deadline_ns=1_000_000,
                clock=Mock(side_effect=[1000, 1100, 1099]))

    def test_explicit_startup_deadline_keeps_sample_age_20ms_and_elapsed_21ms(self):
        release = 1_000_000; oldest = release+600_000
        self.assertEqual(bench._proxy_output_join_deadline(release, oldest), release+20_000_000)
        self.assertEqual(bench._proxy_output_join_deadline(release, oldest, startup=True), oldest+20_000_000)
        self.assertEqual(bench._proxy_output_join_deadline(release, release+2_000_000,
            startup=True), release+21_000_000)

    def test_r5_first_join_can_be_collected_without_claiming_complete_iteration(self):
        release = 3682752316229; oldest = 3682752947250; settled = 3682772361182
        clock = Mock(return_value=settled); owners = self.owners(True)
        startup_deadline = bench._proxy_output_join_deadline(release, oldest, startup=True)
        proof = bench._await_output_ready(owners, deadline_ns=startup_deadline, clock=clock)
        self.assertEqual(proof['end_ns'], settled)
        self.assertLess(settled-oldest, 20_000_000)
        with self.assertRaises(TimeoutError):
            bench._await_output_ready(owners,
                deadline_ns=bench._proxy_output_join_deadline(release, oldest), clock=clock)

    def test_startup_never_accepts_expired_input_or_elapsed_budget(self):
        release = 1_000_000
        for oldest in (release+600_000, release+2_000_000):
            limit = bench._proxy_output_join_deadline(release, oldest, startup=True)
            for late in (limit, limit+1):
                with self.subTest(oldest=oldest, late=late), self.assertRaises(TimeoutError):
                    bench._await_output_ready(self.owners(True), deadline_ns=limit,
                        clock=Mock(return_value=late))


class NativeOutputIntegrationTests(unittest.TestCase):
    def integration(self, *, native=True, pipeline=True, session_type=PreparedSession,
                    check=lambda:None, clock=time.monotonic_ns, startup_cycle_allowance=0):
        started = (threading.Event(), threading.Event()); release = threading.Event()
        sessions = {scope:session_type(started[i], release) for i, scope in enumerate(('front', 'rear'))}
        observer = OverlapObserver(started, release)
        options = dict(mode='stop-proxy', cycles=2 if startup_cycle_allowance else 1,
            startup_cycle_allowance=startup_cycle_allowance, v3_voltage_proxy=True,
            v3_voltage_overlap=True, v3_voltage_validation_overlap=True,
            v3_voltage_fast_pipeline=pipeline, record_storage='trace', clock=clock)
        if native:
            options.update(absolute_epoch_cadence=True,
                deadline_wait=lambda target:time.sleep(max(0, (target-clock())/1e9)))
        report, raw = bench.collect(sessions, Device(), observer, check=check, **options)
        return report, bench._serialize(raw), sessions, observer

    def test_selected_path_preserves_26_requests_original_times_and_final_stop_check(self):
        for pipeline in (True, False):
            with self.subTest(pipeline=pipeline):
                report, rows, sessions, _ = self.integration(pipeline=pipeline)
                self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
                self.assertEqual(report['output_join_wait'], 'native_ready_poll_200us.v1')
                row = rows[0]; proof = row['voltage_fast_pipeline' if pipeline else 'voltage_overlap']
                wait = proof['output_join_wait']
                self.assertLessEqual(proof['output_join_begin_ns'], wait['begin_ns'])
                self.assertLessEqual(wait['end_ns'], proof['output_join_settled_ns'])
                self.assertLessEqual(proof['output_join_settled_ns'], proof['output_join_checked_ns'])
                self.assertLess(proof['output_join_checked_ns'], proof['hard_deadline_ns'])
                self.assertEqual(sum(len(row[phase][scope]['records'])
                    for phase in ('acquired', 'voltage', 'output') for scope in sessions), 26)
                self.assertTrue(all(s.phases == ['feedback', 'voltage', 'output'] for s in sessions.values()))
                if pipeline:
                    self.assertEqual(proof['stop_reply_count'], 12)
                    self.assertLessEqual(proof['output_join_checked_ns'], proof['stop_reply_verified_ns'])
                self.assertFalse(report['motor_enable_sent']); self.assertFalse(report['learned_targets_sent'])

    def test_no_callback_leaves_legacy_output_collection_unchanged(self):
        with patch.object(bench, '_await_output_ready', side_effect=AssertionError('Legacy changed')):
            report, rows, _, _ = self.integration(native=False)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['output_join_wait'], 'legacy_result_collection.v1')
        self.assertNotIn('output_join_wait', rows[0]['voltage_fast_pipeline'])

    def test_startup_only_changes_completed_result_join_not_dispatch_or_steady_deadline(self):
        deadlines = []; original = bench._await_output_ready
        def ready(*args, **kwargs):
            deadlines.append(kwargs['deadline_ns']); return original(*args, **kwargs)
        with patch.object(bench, '_await_output_ready', side_effect=ready):
            report, rows, _, _ = self.integration(startup_cycle_allowance=1)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 2)
        for index, (row, measure, passed) in enumerate(zip(rows, report['measurements'], deadlines)):
            proof = row['voltage_fast_pipeline']; release = measure['release_ns']
            oldest = measure['oldest_input_start_ns']
            self.assertEqual(proof['hard_deadline_ns'], min(release, oldest)+20_000_000)
            self.assertEqual(proof['output_join_startup_allowance'], index == 0)
            self.assertEqual(proof['output_join_deadline_ns'], passed)
            self.assertEqual(passed, min(release+(21_000_000 if index == 0 else 20_000_000),
                                         oldest+20_000_000))
            self.assertTrue(all(t < proof['hard_deadline_ns']
                for t in proof['proxy_submit_checked_ns_by_bus'].values()))
            self.assertTrue(all(t < proof['hard_deadline_ns']
                for t in proof['proxy_actual_start_ns_by_bus'].values()))
            self.assertEqual(measure['timing_phase'], 'startup' if index == 0 else 'steady')

    def test_default_zero_startup_allowance_has_no_new_join_budget(self):
        report, rows, _, _ = self.integration()
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        proof = rows[0]['voltage_fast_pipeline']
        self.assertFalse(proof['output_join_startup_allowance'])
        self.assertEqual(proof['output_join_deadline_ns'], proof['hard_deadline_ns'])

    def test_startup_allowance_does_not_admit_late_proxy_dispatch(self):
        class LateSession(PreparedSession):
            def exchange(self, wires, **kwargs):
                rows, stats = super().exchange(wires, **kwargs)
                if self.phases[-1] == 'output':
                    for row in rows:
                        for name in ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns'):
                            setattr(row, name, getattr(row, name)+21_000_000)
                return rows, stats
        report, rows, _, _ = self.integration(startup_cycle_allowance=1, session_type=LateSession)
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(rows[0]['voltage_fast_pipeline']['status'], 'PROXY_STOP_DISPATCH_DEADLINE_MISSED')

    def test_startup_result_takeout_still_rejects_age_above_20ms(self):
        original = bench._await_output_ready
        def delayed(*args, **kwargs):
            proof = original(*args, **kwargs); time.sleep(.021); return proof
        with patch.object(bench, '_await_output_ready', side_effect=delayed):
            report, rows, _, _ = self.integration(startup_cycle_allowance=1)
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        proof = rows[0]['voltage_fast_pipeline']
        self.assertGreaterEqual(proof['output_join_settled_ns'], proof['output_join_deadline_ns'])
        self.assertTrue(proof['output_join_cleanup_only'])

    def test_join_error_keeps_all_submitted_output_records_as_cleanup_only(self):
        with patch.object(bench, '_await_output_ready', side_effect=RuntimeError('output wait cancelled')):
            report, rows, sessions, observer = self.integration()
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        row = rows[0]; proof = row['voltage_fast_pipeline']
        self.assertEqual(set(row['output']), {'front', 'rear'})
        self.assertEqual(proof['status'], 'FINAL_PROXY_OUTPUT_JOIN_REJECTED')
        self.assertTrue(proof['output_join_cleanup_only']); self.assertIn('cancelled', proof['output_join_error'])
        self.assertGreaterEqual(proof['output_join_settled_ns'], proof['output_join_begin_ns'])
        self.assertNotIn('observed', row); self.assertTrue(observer.invalid)
        self.assertTrue(all(s.phases == ['feedback', 'voltage', 'output'] for s in sessions.values()))

    def test_ready_takeout_delay_is_rejected_without_hiding_settlement_time(self):
        original = bench._await_output_ready
        def delayed(*args, **kwargs):
            proof = original(*args, **kwargs); time.sleep(.021); return proof
        with patch.object(bench, '_await_output_ready', side_effect=delayed):
            report, rows, _, _ = self.integration()
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        proof = rows[0]['voltage_fast_pipeline']
        self.assertGreaterEqual(proof['output_join_settled_ns'], proof['hard_deadline_ns'])
        self.assertIn('result takeout', proof['output_join_error'])
        self.assertTrue(proof['output_join_cleanup_only']); self.assertNotIn('observed', rows[0])

    def test_cancel_after_ready_still_settles_original_outputs_without_next_cycle(self):
        cancelled = threading.Event(); original = bench._await_output_ready
        def ready(*args, **kwargs):
            result = original(*args, **kwargs); cancelled.set(); return result
        def check():
            if cancelled.is_set(): raise RuntimeError('signal after ready')
        with patch.object(bench, '_await_output_ready', side_effect=ready):
            report, rows, _, _ = self.integration(check=check)
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(set(rows[0]['output']), {'front', 'rear'})
        self.assertIn('signal after ready', rows[0]['voltage_fast_pipeline']['output_join_error'])

    def test_one_output_owner_failure_retains_other_owner_and_invalidates(self):
        class FailedSession(PreparedSession):
            def exchange(self, wires, **kwargs):
                result = super().exchange(wires, **kwargs)
                if self.phases[-1] == 'output' and self is failed[0]: raise OSError('output owner failed')
                return result
        failed = []
        class Session(FailedSession):
            def __init__(self, *args):
                super().__init__(*args)
                if not failed: failed.append(self)
        report, rows, _, observer = self.integration(session_type=Session)
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(set(rows[0]['output']), {'rear'})
        self.assertIn('output owner failed', rows[0]['voltage_fast_pipeline']['output_join_error'])
        self.assertTrue(observer.invalid)

    def test_second_submit_failure_retains_first_output_and_original_rejection(self):
        class FailedSubmitPool(ThreadPoolExecutor):
            def submit(self, fn, *args, **kwargs):
                if fn.__name__ == 'exchange' and args[0] == 'rear': raise RuntimeError('rear submit failed')
                return super().submit(fn, *args, **kwargs)
        with patch.object(bench, 'ThreadPoolExecutor', FailedSubmitPool):
            report, rows, _, _ = self.integration()
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(set(rows[0]['output']), {'front'})
        proof = rows[0]['voltage_fast_pipeline']
        self.assertEqual(proof['status'], 'REJECTED_BEFORE_PROXY_STOP')
        self.assertIn('rear submit failed', proof['proxy_submit_error'])
        self.assertTrue(proof['output_join_cleanup_only']); self.assertNotIn('output_join_wait', proof)

    def test_final_stop_reply_validation_is_not_replaced_by_ready_futures(self):
        class BadReplySession(PreparedSession):
            def exchange(self, wires, **kwargs):
                rows, stats = super().exchange(wires, **kwargs)
                if self.phases[-1] == 'output': rows[0].received = 0
                return rows, stats
        report, rows, _, _ = self.integration(session_type=BadReplySession)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(rows[0]['voltage_fast_pipeline']['status'], 'FINAL_STOP_REPLY_REJECTED')
        self.assertEqual(set(rows[0]['output']), {'front', 'rear'})

    def test_trace_copy_cost_is_still_inside_full_iteration(self):
        original = bench._RecordTrace.capture
        def slow_capture(trace, *args, **kwargs):
            result = original(trace, *args, **kwargs); time.sleep(.021); return result
        with patch.object(bench._RecordTrace, 'capture', slow_capture):
            report, rows, _, _ = self.integration()
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 1)
        self.assertGreater(report['measurements'][0]['whole_iteration_ms'], 20.)
        self.assertEqual(report['iteration_deadline_misses'], 1)
        self.assertLess(rows[0]['voltage_fast_pipeline']['output_join_checked_ns'],
                        report['measurements'][0]['cycle_end_ns'])


if __name__ == '__main__': unittest.main()
