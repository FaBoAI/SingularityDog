"""Output readiness and unchanged coordinator deadlines with synthetic Futures only."""
from concurrent.futures import CancelledError, Future
import threading
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_output_runtime as runtime


START = 1_000_000_000
DEADLINE = START + 1_000_000


class Clock:
    def __init__(self): self.now = START
    def __call__(self): return self.now


class ObservedFuture(Future):
    def __init__(self, name, events):
        super().__init__()
        self.name, self.events = name, events
    def result(self, timeout=None):
        self.events.append(('result', self.name, self.done()))
        if not self.done(): raise AssertionError('Read unfinished output proof')
        return super().result(timeout=timeout)


class OutputJoinTests(unittest.TestCase):
    def fixture(self):
        clock, events = Clock(), []
        workers = runtime.BusWorkers.__new__(runtime.BusWorkers)
        workers.clock, workers.aborted, workers.reason = clock, threading.Event(), None
        def emergency(reason):
            events.append(('emergency', str(reason)))
            workers.reason = str(reason); workers.aborted.set()
        workers.emergency = Mock(side_effect=emergency)
        workers.collect = runtime.BusWorkers.collect.__get__(workers)
        futures = {scope: ObservedFuture(scope, events) for scope in ('front', 'rear')}
        values = {scope: (object(), {scope: object()}, START) for scope in futures}
        return workers, clock, events, futures, values

    def collect(self, workers, futures, waiter=None, timing=None):
        return workers.collect_output(futures, deadline_ns=DEADLINE,
                                       deadline_wait=waiter, timing=timing)

    def assert_no_pending_result(self, events):
        self.assertFalse(any(row[0] == 'result' and not row[2] for row in events), events)

    def test_already_ready_returns_same_owner_proofs_without_wait(self):
        workers, _, events, futures, values = self.fixture()
        for scope in futures: futures[scope].set_result(values[scope])
        native = Mock()
        with patch.object(runtime, 'wait') as fallback:
            result = self.collect(workers, futures, native)
        fallback.assert_not_called(); native.assert_not_called()
        for scope in values: self.assertIs(result[scope], values[scope])
        workers.emergency.assert_not_called(); self.assert_no_pending_result(events)

    def test_native_wait_ticks_are_bounded_and_takeout_is_after_both_ready(self):
        workers, clock, events, futures, values = self.fixture()
        ticks = []
        def native(wake):
            self.assertGreater(wake, clock.now)
            self.assertLessEqual(wake - clock.now, 200_000)
            self.assertLessEqual(wake, DEADLINE)
            ticks.append(wake); clock.now = wake
            events.append(('native', len(ticks)))
            if len(ticks) == 1: futures['rear'].set_result(values['rear'])
            if len(ticks) == 3: futures['front'].set_result(values['front'])
        with patch.object(runtime, 'wait') as fallback:
            result = self.collect(workers, futures, native)
        fallback.assert_not_called()
        self.assertEqual(ticks, [START + 200_000, START + 400_000, START + 600_000])
        self.assertTrue(all(row[0] != 'result' for row in events[:3]))
        for scope in values: self.assertIs(result[scope], values[scope])
        workers.emergency.assert_not_called(); self.assert_no_pending_result(events)

    def test_each_ready_failure_propagates_before_other_pending_or_wait(self):
        for scope in ('front', 'rear'):
            workers, _, events, futures, _ = self.fixture()
            failure = OSError('Invalid owner output: ' + scope)
            futures[scope].set_exception(failure); native = Mock()
            with self.assertRaises(OSError) as caught:
                self.collect(workers, futures, native)
            self.assertIs(caught.exception, failure); native.assert_not_called()
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_owner_failure_during_native_wait_wins_over_cancelled_wait_error(self):
        for scope in ('front', 'rear'):
            workers, clock, events, futures, _ = self.fixture()
            failure = ValueError('Owner guard failure: ' + scope)
            def native(wake):
                clock.now = wake; futures[scope].set_exception(failure)
                workers.aborted.set(); workers.reason = str(failure)
                raise RuntimeError('Native wait cancellation')
            with self.assertRaises(ValueError) as caught:
                self.collect(workers, futures, native)
            self.assertIs(caught.exception, failure)
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_cancelled_future_before_and_during_wait_never_reads_pending_peer(self):
        for during in (False, True):
            workers, clock, events, futures, _ = self.fixture()
            if not during: futures['rear'].cancel()
            def native(wake): clock.now = wake; futures['rear'].cancel()
            with self.assertRaises(CancelledError): self.collect(workers, futures, native)
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_native_wait_failure_and_owner_abort_cannot_continue(self):
        workers, _, events, futures, _ = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'wait failure'):
            self.collect(workers, futures, Mock(side_effect=RuntimeError('wait failure')))
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)
        workers, clock, events, futures, values = self.fixture()
        def abort(wake):
            clock.now = wake; workers.aborted.set(); workers.reason = 'Asynchronous owner abort'
            for scope in futures: futures[scope].set_result(values[scope])
        with self.assertRaisesRegex(RuntimeError, 'Asynchronous owner abort'):
            self.collect(workers, futures, abort)
        self.assertFalse(any(row[0] == 'result' for row in events))
        workers.emergency.assert_called_once()

    def test_deadline_with_pending_proofs_does_not_take_unfinished_result(self):
        workers, clock, events, futures, values = self.fixture()
        futures['front'].set_result(values['front']); ticks = []
        def native(wake): ticks.append(wake); clock.now = wake
        with self.assertRaises(TimeoutError): self.collect(workers, futures, native)
        self.assertEqual(ticks[-1], DEADLINE)
        self.assertEqual(len(ticks), 5)
        self.assertFalse(any(row[0] == 'result' for row in events))
        workers.emergency.assert_called_once()

    def test_ready_proofs_at_or_after_deadline_are_rejected_before_takeout(self):
        for lateness in (0, 1):
            workers, clock, events, futures, values = self.fixture()
            def late(wake):
                clock.now = DEADLINE + lateness
                for scope in futures: futures[scope].set_result(values[scope])
            with self.assertRaises(TimeoutError): self.collect(workers, futures, late)
            self.assertFalse(any(row[0] == 'result' for row in events))
            workers.emergency.assert_called_once()

    def test_result_takeout_crossing_deadline_is_rejected(self):
        workers, clock, events, futures, values = self.fixture()
        for scope in futures: futures[scope].set_result(values[scope])
        original = futures['rear'].result
        def late_result(*args, **kwargs):
            result = original(*args, **kwargs); clock.now = DEADLINE; return result
        futures['rear'].result = late_result
        with self.assertRaises(TimeoutError): self.collect(workers, futures)
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_fallback_is_one_first_exception_wait_with_original_deadline(self):
        workers, clock, events, futures, values = self.fixture()
        def fallback(inputs, **kwargs):
            self.assertEqual(set(inputs), set(futures.values()))
            self.assertEqual(kwargs['return_when'], runtime.FIRST_EXCEPTION)
            self.assertEqual(kwargs['timeout'], (DEADLINE - START)/1e9)
            clock.now += 100_000
            for scope in futures: futures[scope].set_result(values[scope])
            return set(inputs), set()
        with patch.object(runtime, 'wait', side_effect=fallback) as wait:
            result = self.collect(workers, futures)
        wait.assert_called_once(); self.assertEqual(result, values)
        workers.emergency.assert_not_called(); self.assert_no_pending_result(events)

    def test_fallback_timeout_or_failure_propagates_without_pending_result(self):
        for fail in (False, True):
            workers, clock, events, futures, _ = self.fixture()
            failure = OSError('Rear validation rejected')
            def fallback(inputs, **kwargs):
                if fail: futures['rear'].set_exception(failure)
                else: clock.now = DEADLINE
                return ({futures['rear']} if fail else set()), {futures['front']}
            with patch.object(runtime, 'wait', side_effect=fallback), \
                    self.assertRaises(OSError if fail else TimeoutError) as caught:
                self.collect(workers, futures)
            if fail: self.assertIs(caught.exception, failure)
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_missing_duplicate_or_invalid_deadline_proofs_fail_before_wait(self):
        for defect in ('missing', 'duplicate', 'bool_deadline', 'bad_waiter', 'expired', 'wrong_future'):
            workers, clock, events, futures, _ = self.fixture(); deadline = DEADLINE; native = Mock()
            if defect == 'missing': del futures['rear']
            if defect == 'duplicate': futures['rear'] = futures['front']
            if defect == 'wrong_future': futures['rear'] = object()
            if defect == 'bool_deadline': deadline = True
            if defect == 'bad_waiter': native = True
            if defect == 'expired': clock.now = deadline
            with self.assertRaises((RuntimeError, TimeoutError)):
                workers.collect_output(futures, deadline_ns=deadline, deadline_wait=native)
            if callable(native): native.assert_not_called()
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_backdated_native_wait_is_rejected_without_reading_pending_results(self):
        workers, _, events, futures, _ = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'before its deadline'):
            self.collect(workers, futures, lambda wake: wake)
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_wall_and_cpu_stamps_are_recorded_on_success_and_failure_and_reset(self):
        for fail in (False, True):
            workers, _, _, futures, values = self.fixture()
            timing = runtime._PendingCycleTiming(); timing.begin(0, START, START, None, None)
            for scope in futures: futures[scope].set_result(values[scope])
            if fail: futures['rear'] = ObservedFuture('rear', []); futures['rear'].set_exception(OSError('failure'))
            with patch.object(runtime.time, 'thread_time_ns', side_effect=(1_000, 13_000)) as cpu:
                if fail:
                    with self.assertRaises(OSError): self.collect(workers, futures, timing=timing)
                else: self.collect(workers, futures, timing=timing)
            self.assertEqual(cpu.call_count, 2)
            self.assertEqual(timing.output_join_cpu_begin_ns, 1_000)
            self.assertEqual(timing.output_join_begin_ns, START)
            if not fail:
                self.assertEqual(timing.output_join_ready_ns, START)
                self.assertEqual(timing.output_takeout_end_ns, START)
            self.assertEqual(timing.output_join_cpu_end_ns, 13_000)
            snapshot = timing.snapshot({'hard_cycle_ms': 20., 'max_sample_age_ms': 20.})
            self.assertEqual(snapshot['output_join_cpu_ms'], .012)
            self.assertEqual(snapshot['command_gap_basis'], 'validated_target_computation_not_transport_write')
            timing.begin(1, START, START, None, None)
            self.assertIsNone(timing.output_join_cpu_begin_ns)
            self.assertIsNone(timing.output_join_cpu_end_ns)


    def test_keyboard_interrupt_requests_stop_without_pending_takeout(self):
        workers, _, events, futures, _ = self.fixture()
        with self.assertRaises(KeyboardInterrupt):
            self.collect(workers, futures, Mock(side_effect=KeyboardInterrupt()))
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_late_host_proof_fails_even_if_native_result_itself_was_on_time(self):
        workers, clock, events, futures, values = self.fixture()
        # The reply data may say it arrived early; host completion still matters.
        for scope in futures: futures[scope].set_result(values[scope])
        clock.now = DEADLINE + 1
        with self.assertRaises(TimeoutError): self.collect(workers, futures)
        self.assertFalse(any(row[0] == 'result' for row in events))
        workers.emergency.assert_called_once()

    def test_abort_during_ready_result_takeout_cannot_publish_outputs(self):
        workers, _, events, futures, values = self.fixture()
        for scope in futures: futures[scope].set_result(values[scope])
        original = futures['front'].result
        def abort(*args, **kwargs):
            value = original(*args, **kwargs)
            workers.reason = 'STOP during takeout'; workers.aborted.set()
            return value
        futures['front'].result = abort
        with self.assertRaisesRegex(RuntimeError, 'STOP during takeout'):
            self.collect(workers, futures)
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)


class OutputJoinRuntimeTests(unittest.TestCase):
    def test_existing_post_reply_allowance_is_not_applied_to_native_deadline(self):
        from test_policy_post_reply_timing import CoordinatorTests
        fixture = CoordinatorTests(); self.addCleanup(fixture.doCleanups)
        report, _, _ = fixture.run_timing(delayed={2:20_010_000})
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertEqual(report['post_reply_deadline_allowance_uses'], 1)
        for row in report['cycles']:
            self.assertEqual(row['output_native_deadline_ns'], row['begin_ns'] + 20_000_000)
            self.assertLessEqual(row['output_reply_end_ns'], row['output_native_deadline_ns'])
            self.assertGreaterEqual(row['output_join_deadline_ns'], row['output_native_deadline_ns'])
            self.assertLessEqual(row['output_join_deadline_ns'], row['begin_ns'] + 21_000_000)

    def test_default_runtime_joins_both_decoded_owners_and_records_boundaries(self):
        from test_policy_output_runtime import OutputRuntimeTests
        calls=[]; original=runtime.BusWorkers.collect_output
        def join(workers,*args,**kwargs):
            calls.append(dict(kwargs))
            return original(workers,*args,**kwargs)
        with patch.object(runtime.BusWorkers,'collect_output',autospec=True,side_effect=join):
            report,_=OutputRuntimeTests.run_case(self)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(len(calls),len(report['cycles']))
        self.assertEqual(report['output_join_wait'],'all_ready_first_exception.v1')
        self.assertTrue(report['stop_confirmed'])
        for row,call in zip(report['cycles'],calls):
            self.assertIsNone(call['deadline_wait'])
            self.assertEqual(call['deadline_ns'],row['output_native_deadline_ns'])
            stamps=[row[key] for key in ('output_join_begin_ns','output_join_ready_ns',
                                         'output_takeout_end_ns','output_exchange_return_ns')]
            self.assertEqual(stamps,sorted(stamps))
            self.assertLess(row['output_exchange_return_ns'],row['output_join_deadline_ns'])
            self.assertGreaterEqual(row['output_join_cpu_ms'],0.)

    def test_selected_native_wait_is_passed_for_output_and_all_guards_remain(self):
        from test_policy_output_runtime import OutputRuntimeTests,SimulatedClock,FakeSession,FakeIMU
        import time
        clock=SimulatedClock(); waiting=[]; selected=[]
        original_output=runtime.BusWorkers.collect_output
        original_acquisition=runtime.BusWorkers.collect_acquisition
        def native_wait(target):
            limit=time.monotonic()+1.
            while waiting and not all(future.done() for future in waiting):
                self.assertLess(time.monotonic(),limit,'Synthetic owner did not finish')
                time.sleep(0)
            clock.advance_to(target); time.sleep(0)
        def acquire(workers,*args,**kwargs):
            waiting[:]=[*args[0].values(),args[1]]
            try:return original_acquisition(workers,*args,**kwargs)
            finally:waiting.clear()
        def output(workers,*args,**kwargs):
            selected.append(kwargs['deadline_wait']);waiting[:]=args[0].values()
            try:return original_output(workers,*args,**kwargs)
            finally:waiting.clear()
        with patch.object(runtime.BusWorkers,'collect_acquisition',autospec=True,side_effect=acquire), \
             patch.object(runtime.BusWorkers,'collect_output',autospec=True,side_effect=output):
            report,_=OutputRuntimeTests.run_case(self,absolute_epoch_cadence=True,
                deadline_wait=native_wait,clock=clock,sleep=clock.sleep,
                front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),imu=FakeIMU(clock=clock))
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(report['output_join_wait'],'native_ready_poll_200us.v1')
        self.assertTrue(selected);self.assertTrue(all(item is native_wait for item in selected))
        self.assertTrue(report['stop_confirmed'])
        for row in report['cycles']:
            self.assertLessEqual(row['output_reply_end_ns'],row['output_native_deadline_ns'])
            self.assertLess(row['output_exchange_return_ns'],row['output_join_deadline_ns'])


if __name__ == '__main__':
    unittest.main()
