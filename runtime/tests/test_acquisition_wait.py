"""Joint CAN/IMU acquisition waits using synthetic Futures, without hardware.

These check ordering, failure, and absolute-deadline semantics. They do not
measure host scheduling or establish physical 20ms operation.
"""
from concurrent.futures import CancelledError, Future, wait as real_wait
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_output_runtime as runtime

START_NS = 1_000_000_000
DEADLINE_NS = START_NS + 20_000_000


class Clock:
    def __init__(self):
        self.now = START_NS

    def __call__(self):
        return self.now


class ObservedFuture(Future):
    """Any result access to unfinished input is a test failure, not a wait."""
    def __init__(self, name, events):
        super().__init__()
        self.name = name
        self.events = events

    def result(self, timeout=None):
        self.events.append(('result', self.name, self.done()))
        if not self.done():
            raise AssertionError('Unfinished acquisition result was read: ' + self.name)
        return super().result(timeout=timeout)


class AcquisitionWaitTests(unittest.TestCase):
    def fixture(self):
        clock, events = Clock(), []
        workers = runtime.BusWorkers.__new__(runtime.BusWorkers)
        workers.clock = clock
        workers.aborted = threading.Event()
        workers.reason = None
        def emergency(reason):
            events.append(('emergency', str(reason)))
            workers.reason = str(reason)
            workers.aborted.set()
        workers.emergency = Mock(side_effect=emergency)
        futures = {name: ObservedFuture(name, events) for name in ('front', 'rear', 'imu')}
        return workers, clock, events, futures

    def collect(self, workers, futures):
        return workers.collect_acquisition(
            {name: futures[name] for name in ('front', 'rear')}, futures['imu'],
            deadline_ns=DEADLINE_NS)

    def assert_no_unfinished_result(self, events):
        self.assertFalse(any(event[0] == 'result' and not event[2] for event in events), events)

    def test_all_success_has_one_joint_wait_and_returns_original_values(self):
        workers, _, events, futures = self.fixture()
        values = {name: object() for name in futures}
        for name, future in futures.items():
            future.set_result(values[name])
        with patch.object(runtime, 'wait', wraps=real_wait) as wait:
            buses, imu = self.collect(workers, futures)
        wait.assert_called_once()
        args, kwargs = wait.call_args
        self.assertEqual(set(args[0]), set(futures.values()))
        self.assertEqual(kwargs.get('return_when'), runtime.FIRST_EXCEPTION)
        self.assertGreater(kwargs.get('timeout', 0), 0)
        self.assertLessEqual(kwargs['timeout'], .02)
        self.assertIs(buses['front'], values['front'])
        self.assertIs(buses['rear'], values['rear'])
        self.assertIs(imu, values['imu'])
        workers.emergency.assert_not_called()
        self.assert_no_unfinished_result(events)

    def test_successful_join_stamps_and_reads_follow_the_completed_wait(self):
        workers, clock, events, futures = self.fixture()
        timing = runtime._PendingCycleTiming()
        timing.begin(0, START_NS, START_NS, None, None)
        def complete(fs, **kwargs):
            events.append(('wait', 'entered'))
            clock.now += 7_000_000
            for future in fs:
                future.set_result(object())
            events.append(('wait', 'all_complete'))
            return set(fs), set()
        with patch.object(runtime, 'wait', side_effect=complete) as wait:
            workers.collect_acquisition(
                {name: futures[name] for name in ('front', 'rear')}, futures['imu'],
                deadline_ns=DEADLINE_NS, timing=timing)
        wait.assert_called_once()
        self.assertEqual(events[:2], [('wait', 'entered'), ('wait', 'all_complete')])
        self.assertTrue(all(event[0] == 'result' and event[2] for event in events[2:]))
        stamps = [getattr(timing, name) for name in (
            'combined_acquisition_wait_begin_ns', 'combined_acquisition_wait_end_ns',
            'feedback_collect_begin_ns', 'feedback_collect_end_ns',
            'imu_wait_begin_ns', 'imu_wait_end_ns')]
        self.assertEqual(stamps, sorted(stamps))
        self.assertEqual(stamps[0], START_NS)
        self.assertEqual(stamps[1], START_NS + 7_000_000)
        self.assertTrue(all(stamp == START_NS + 7_000_000 for stamp in stamps[2:]))
        workers.emergency.assert_not_called()

    def test_each_failure_stops_before_waiting_on_other_unfinished_inputs(self):
        for failed in ('front', 'rear', 'imu'):
            with self.subTest(failed=failed):
                workers, _, events, futures = self.fixture()
                failure = OSError('Injected ' + failed + ' acquisition failure')
                futures[failed].set_exception(failure)
                with patch.object(runtime, 'wait', wraps=real_wait) as wait:
                    with self.assertRaises(OSError) as caught:
                        self.collect(workers, futures)
                self.assertIs(caught.exception, failure)
                wait.assert_called_once()
                workers.emergency.assert_called_once()
                self.assertIn(failed, str(workers.emergency.call_args))
                self.assert_no_unfinished_result(events)
                self.assertTrue(all(not future.done() for name, future in futures.items() if name != failed))

    def test_cancelled_future_stops_without_reading_other_pending_results(self):
        # A Future cancelled before its executor notifies waiters must also STOP;
        # otherwise concurrent.futures.wait may leave it in the pending set.
        for cancelled in ('front', 'rear', 'imu'):
            with self.subTest(cancelled=cancelled):
                workers, _, events, futures = self.fixture()
                self.assertTrue(futures[cancelled].cancel())
                with self.assertRaises(CancelledError):
                    self.collect(workers, futures)
                workers.emergency.assert_called_once()
                self.assert_no_unfinished_result(events)

    def test_timeout_stops_without_result_access_to_unfinished_inputs(self):
        workers, clock, events, futures = self.fixture()
        futures['front'].set_result(object())
        def timed_out(fs, **kwargs):
            self.assertEqual(set(fs), set(futures.values()))
            clock.now = DEADLINE_NS
            return {futures['front']}, {futures['rear'], futures['imu']}
        with patch.object(runtime, 'wait', side_effect=timed_out) as wait:
            with self.assertRaises((TimeoutError, RuntimeError)):
                self.collect(workers, futures)
        wait.assert_called_once()
        workers.emergency.assert_called_once()
        self.assert_no_unfinished_result(events)

    def test_completed_inputs_at_or_after_absolute_deadline_are_rejected(self):
        for lateness in (0, 1):
            with self.subTest(lateness=lateness):
                workers, clock, events, futures = self.fixture()
                for future in futures.values():
                    future.set_result(object())
                def late_complete(fs, **kwargs):
                    clock.now = DEADLINE_NS + lateness
                    return set(fs), set()
                with patch.object(runtime, 'wait', side_effect=late_complete) as wait:
                    with self.assertRaises((TimeoutError, RuntimeError)):
                        self.collect(workers, futures)
                wait.assert_called_once()
                workers.emergency.assert_called_once()
                self.assert_no_unfinished_result(events)

    def test_expired_deadline_rejects_before_wait_or_pending_result_reads(self):
        workers, clock, events, futures = self.fixture()
        clock.now = DEADLINE_NS
        with patch.object(runtime, 'wait') as wait:
            with self.assertRaises((TimeoutError, RuntimeError)):
                self.collect(workers, futures)
        wait.assert_not_called()
        workers.emergency.assert_called_once()
        self.assert_no_unfinished_result(events)

    def test_wait_failure_or_interruption_stops_before_any_result_access(self):
        for failure in (OSError('Injected wait failure'), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__):
                workers, _, events, futures = self.fixture()
                with patch.object(runtime, 'wait', side_effect=failure):
                    with self.assertRaises(type(failure)) as caught:
                        self.collect(workers, futures)
                self.assertIs(caught.exception, failure)
                workers.emergency.assert_called_once()
                self.assertFalse(any(event[0] == 'result' for event in events))


class NativeAcquisitionWaitTests(unittest.TestCase):
    """The selected native waiter cannot admit pending, stale or failed input."""
    fixture = AcquisitionWaitTests.fixture
    assert_no_unfinished_result = AcquisitionWaitTests.assert_no_unfinished_result

    def collect(self, workers, futures, native_wait, *, timing=None, deadline_ns=DEADLINE_NS):
        return workers.collect_acquisition(
            {name: futures[name] for name in ('front', 'rear')}, futures['imu'],
            deadline_ns=deadline_ns, deadline_wait=native_wait, timing=timing)

    def complete(self, futures):
        values = {name: object() for name in futures}
        for name, future in futures.items():
            future.set_result(values[name])
        return values

    def test_ready_inputs_return_original_values_without_native_or_condition_wait(self):
        workers, _, events, futures = self.fixture()
        values = self.complete(futures); native_wait = Mock()
        with patch.object(runtime, 'wait', side_effect=AssertionError('Condition wait is forbidden')):
            buses, imu = self.collect(workers, futures, native_wait)
        native_wait.assert_not_called()
        self.assertIs(buses['front'], values['front'])
        self.assertIs(buses['rear'], values['rear'])
        self.assertIs(imu, values['imu'])
        self.assert_no_unfinished_result(events)

    def test_poll_waits_for_current_imu_and_requests_at_most_200us(self):
        workers, clock, events, futures = self.fixture(); targets = []
        values = {name: object() for name in futures}
        def native_wait(target):
            self.assertGreater(target, clock.now)
            self.assertLessEqual(target-clock.now, 200_000)
            targets.append(target); clock.now = target
            if len(targets) == 1:
                for name in ('front', 'rear'): futures[name].set_result(values[name])
            else: futures['imu'].set_result(values['imu'])
        with patch.object(runtime, 'wait', side_effect=AssertionError('Condition wait is forbidden')):
            buses, imu = self.collect(workers, futures, native_wait)
        self.assertEqual(targets, [START_NS+200_000, START_NS+400_000])
        self.assertIs(imu, values['imu']); self.assertIs(buses['rear'], values['rear'])
        self.assert_no_unfinished_result(events)

    def test_error_on_each_input_preempts_other_pending_inputs(self):
        for failed in ('front', 'rear', 'imu'):
            with self.subTest(failed=failed):
                workers, _, events, futures = self.fixture(); native_wait = Mock()
                error = OSError('current '+failed+' failed'); futures[failed].set_exception(error)
                with self.assertRaises(OSError) as caught:
                    self.collect(workers, futures, native_wait)
                self.assertIs(caught.exception, error); native_wait.assert_not_called()
                workers.emergency.assert_called_once(); self.assert_no_unfinished_result(events)

    def test_error_published_during_native_cancel_preserves_original_error(self):
        workers, _, events, futures = self.fixture(); error = OSError('rear decode failed')
        def native_wait(target):
            futures['rear'].set_exception(error)
            raise RuntimeError('native wait cancelled')
        with self.assertRaises(OSError) as caught:
            self.collect(workers, futures, native_wait)
        self.assertIs(caught.exception, error); self.assert_no_unfinished_result(events)

    def test_cancelled_future_before_or_during_native_wait_stops(self):
        for during in (False, True):
            with self.subTest(during=during):
                workers, clock, events, futures = self.fixture(); calls = []
                if not during: futures['imu'].cancel()
                def native_wait(target):
                    calls.append(target); clock.now = target; futures['imu'].cancel()
                with self.assertRaises(CancelledError):
                    self.collect(workers, futures, native_wait)
                self.assertEqual(len(calls), int(during)); workers.emergency.assert_called_once()
                self.assert_no_unfinished_result(events)

    def test_abort_before_or_during_wait_does_not_read_pending_inputs(self):
        for during in (False, True):
            with self.subTest(during=during):
                workers, clock, events, futures = self.fixture(); calls = []
                if not during: workers.aborted.set()
                def native_wait(target):
                    calls.append(target); clock.now = target; workers.aborted.set()
                with self.assertRaisesRegex(RuntimeError, 'aborted'):
                    self.collect(workers, futures, native_wait)
                self.assertEqual(len(calls), int(during)); self.assert_no_unfinished_result(events)

    def test_native_last_target_clipped_to_deadline_and_equality_rejected(self):
        workers, clock, events, futures = self.fixture(); targets = []
        deadline = START_NS+300_000
        def native_wait(target):
            targets.append(target); clock.now = target
            if target == deadline: self.complete(futures)
        with self.assertRaises(TimeoutError):
            self.collect(workers, futures, native_wait, deadline_ns=deadline)
        self.assertEqual(targets, [START_NS+200_000, deadline])
        self.assertFalse(any(event[0] == 'result' for event in events))

    def test_native_oversleep_not_backdated_even_when_all_inputs_are_ready(self):
        workers, clock, events, futures = self.fixture()
        def native_wait(target):
            self.complete(futures); clock.now = DEADLINE_NS+1
        with self.assertRaises(TimeoutError): self.collect(workers, futures, native_wait)
        self.assertFalse(any(event[0] == 'result' for event in events))

    def test_native_early_or_backdated_return_rejected(self):
        for offset in (-1, -200_001):
            with self.subTest(offset=offset):
                workers, clock, events, futures = self.fixture()
                def native_wait(target): clock.now = target+offset
                with self.assertRaisesRegex(RuntimeError, 'before its deadline'):
                    self.collect(workers, futures, native_wait)
                self.assert_no_unfinished_result(events)

    def test_aliases_non_futures_and_invalid_deadlines_fail_before_wait(self):
        for invalid in ('can-alias', 'imu-alias', 'non-future', 'missing-bus', 'zero', 'boolean', 'callback'):
            with self.subTest(invalid=invalid):
                workers, _, events, futures = self.fixture(); native_wait = Mock()
                buses = {name: futures[name] for name in ('front', 'rear')}; imu = futures['imu']
                deadline = DEADLINE_NS
                if invalid == 'can-alias': buses['rear'] = buses['front']
                if invalid == 'imu-alias': imu = buses['front']
                if invalid == 'non-future': imu = object()
                if invalid == 'missing-bus': del buses['rear']
                if invalid == 'zero': deadline = 0
                if invalid == 'boolean': deadline = True
                if invalid == 'callback': native_wait = object()
                with self.assertRaises(RuntimeError):
                    workers.collect_acquisition(buses, imu, deadline_ns=deadline, deadline_wait=native_wait)
                if isinstance(native_wait, Mock): native_wait.assert_not_called()
                self.assertFalse(any(event[0] == 'result' for event in events))

    def test_ready_result_takeout_crossing_deadline_or_aborting_is_rejected(self):
        for abort in (False, True):
            for native in (False, True):
                with self.subTest(abort=abort, native=native):
                    workers, clock, events, futures = self.fixture(); self.complete(futures)
                    original = futures['imu'].result
                    def result(timeout=None):
                        value = original(timeout)
                        if abort: workers.aborted.set()
                        else: clock.now = DEADLINE_NS
                        return value
                    with patch.object(futures['imu'], 'result', side_effect=result):
                        with self.assertRaises((RuntimeError, TimeoutError)):
                            self.collect(workers, futures, Mock() if native else None)
                    workers.emergency.assert_called_once(); self.assert_no_unfinished_result(events)

    def test_wall_and_cpu_timing_record_actual_ready_and_takeout_boundaries(self):
        workers, clock, _, futures = self.fixture(); timing = runtime._PendingCycleTiming()
        timing.begin(0, START_NS, START_NS, None, None)
        def native_wait(target): clock.now = target; self.complete(futures)
        with patch.object(runtime.time, 'thread_time_ns', side_effect=[1000, 3000]):
            self.collect(workers, futures, native_wait, timing=timing)
        self.assertEqual(timing.combined_acquisition_wait_begin_ns, START_NS)
        self.assertEqual(timing.combined_acquisition_wait_end_ns, START_NS+200_000)
        self.assertEqual(timing.combined_acquisition_wait_cpu_begin_ns, 1000)
        self.assertEqual(timing.combined_acquisition_wait_cpu_end_ns, 3000)
        self.assertAlmostEqual(timing.snapshot({'hard_cycle_ms':20.,'max_sample_age_ms':20.})['acquisition_join_cpu_ms'], .002)

    def test_failure_cpu_stamp_retained_but_completed_ready_stamp_is_unset(self):
        workers, _, _, futures = self.fixture(); timing = runtime._PendingCycleTiming()
        timing.begin(0, START_NS, START_NS, None, None)
        with patch.object(runtime.time, 'thread_time_ns', side_effect=[1000, 4000]):
            with self.assertRaises(OSError):
                self.collect(workers, futures, Mock(side_effect=OSError('wait failed')), timing=timing)
        self.assertEqual(timing.combined_acquisition_wait_cpu_end_ns, 4000)
        self.assertIsNone(timing.combined_acquisition_wait_end_ns)
        self.assertAlmostEqual(timing.snapshot({'hard_cycle_ms':20.,'max_sample_age_ms':20.})['acquisition_join_cpu_ms'], .003)


class NativeAcquisitionRuntimeIntegrationTests(unittest.TestCase):
    def test_selected_runtime_passes_same_native_waiter_and_records_cpu_time(self):
        from test_policy_output_runtime import OutputRuntimeTests, SimulatedClock, FakeSession, FakeIMU
        original = runtime.BusWorkers.collect_acquisition
        original_output = runtime.BusWorkers.collect_output
        selected = []; waiting=[]; clock=SimulatedClock()
        # Give real executor workers CPU while advancing shared causal fixture
        # time, without making macOS wake jitter decide an argument-wire test.
        def native_wait(target):
            limit=time.monotonic()+1.
            while waiting and not all(future.done() for future in waiting):
                self.assertLess(time.monotonic(),limit,'Synthetic executor input did not complete')
                time.sleep(0)
            clock.advance_to(target); time.sleep(0)
        def collect(workers, *args, **kwargs):
            selected.append(kwargs.get('deadline_wait'))
            waiting[:]=[*args[0].values(),args[1]]
            try: return original(workers,*args,**kwargs)
            finally: waiting.clear()
        def collect_output(workers, *args, **kwargs):
            # The same native waiter now also joins output Futures. Let their
            # real fixture workers finish before advancing synthetic time.
            waiting[:]=args[0].values()
            try: return original_output(workers,*args,**kwargs)
            finally: waiting.clear()
        with patch.object(runtime.BusWorkers,'collect_acquisition',side_effect=collect,autospec=True), \
             patch.object(runtime.BusWorkers,'collect_output',side_effect=collect_output,autospec=True):
            report,sessions=OutputRuntimeTests.run_case(self,absolute_epoch_cadence=True,
                deadline_wait=native_wait,clock=clock,sleep=clock.sleep,
                front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),imu=FakeIMU(clock=clock))
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(selected); self.assertTrue(all(wait is native_wait for wait in selected))
        self.assertEqual(report['input_acquisition_wait'],'native_ready_poll_200us.v1')
        self.assertTrue(all(row['acquisition_join_cpu_ms']>=0 for row in report['cycles']))
        self.assertTrue(report['stop_confirmed']); self.assertEqual(set(sessions),{'front','rear'})


if __name__ == '__main__':
    unittest.main()
