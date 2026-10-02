"""Joint CAN/IMU acquisition waits using synthetic Futures, without hardware.

These check ordering, failure, and absolute-deadline semantics. They do not
measure host scheduling or establish physical 20ms operation.
"""
from concurrent.futures import CancelledError, Future, wait as real_wait
import threading
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


if __name__ == '__main__':
    unittest.main()
