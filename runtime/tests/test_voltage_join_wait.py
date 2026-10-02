"""Voltage readiness/absolute deadlines with fake clocks and Futures only."""
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
        if not self.done(): raise AssertionError('Read unfinished voltage proof')
        return super().result(timeout=timeout)


class VoltageJoinTests(unittest.TestCase):
    def fixture(self):
        clock, events = Clock(), []
        workers = runtime.BusWorkers.__new__(runtime.BusWorkers)
        workers.clock, workers.aborted, workers.reason = clock, threading.Event(), None
        def emergency(reason):
            events.append(('emergency', str(reason)))
            workers.reason = str(reason); workers.aborted.set()
        workers.emergency = Mock(side_effect=emergency)
        futures = {scope: ObservedFuture(scope, events) for scope in ('front', 'rear')}
        values = {scope: (object(), {scope: object()}, START) for scope in futures}
        return workers, clock, events, futures, values

    def collect(self, workers, futures, waiter=None, timing=None):
        return workers.collect_voltage(futures, deadline_ns=DEADLINE,
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
            failure = OSError('Invalid owner voltage: ' + scope)
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
        for defect in ('missing', 'duplicate', 'bool_deadline', 'bad_waiter', 'expired'):
            workers, clock, events, futures, _ = self.fixture(); deadline = DEADLINE; native = Mock()
            if defect == 'missing': del futures['rear']
            if defect == 'duplicate': futures['rear'] = futures['front']
            if defect == 'bool_deadline': deadline = True
            if defect == 'bad_waiter': native = True
            if defect == 'expired': clock.now = deadline
            with self.assertRaises((RuntimeError, TimeoutError)):
                workers.collect_voltage(futures, deadline_ns=deadline, deadline_wait=native)
            if callable(native): native.assert_not_called()
            workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_backdated_native_wait_is_rejected_without_reading_pending_results(self):
        workers, _, events, futures, _ = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'before its deadline'):
            self.collect(workers, futures, lambda wake: wake)
        workers.emergency.assert_called_once(); self.assert_no_pending_result(events)

    def test_two_cpu_stamps_are_recorded_on_success_and_failure_and_reset(self):
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
            self.assertEqual(timing.voltage_join_cpu_begin_ns, 1_000)
            self.assertEqual(timing.voltage_join_cpu_end_ns, 13_000)
            snapshot = timing.snapshot({'hard_cycle_ms': 20., 'max_sample_age_ms': 20.})
            self.assertEqual(snapshot['voltage_join_cpu_ms'], .012)
            self.assertEqual(snapshot['command_gap_basis'], 'validated_target_computation_not_transport_write')
            timing.begin(1, START, START, None, None)
            self.assertIsNone(timing.voltage_join_cpu_begin_ns)
            self.assertIsNone(timing.voltage_join_cpu_end_ns)


if __name__ == '__main__':
    unittest.main()
