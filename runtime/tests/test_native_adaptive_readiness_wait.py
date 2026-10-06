"""Synthetic clocks/Futures only; no native library, model or hardware."""
from concurrent.futures import Future
import json
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench


class Clock:
    def __init__(self, now):
        self.now = now

    def __call__(self):
        return self.now


class AdaptiveReadinessTests(unittest.TestCase):
    def owners(self, ready=False):
        result = {name: Future() for name in ('front', 'rear')}
        if ready:
            for name, future in result.items():
                future.set_result(name)
        return result

    def run_wait(self, owners, clock, native=None, check=lambda: None):
        return bench._await_output_ready(owners, deadline_ns=1_000_000,
            deadline_wait=native, clock=clock, check=check, thread_clock=lambda: 10)

    def test_completion_with_eighty_microseconds_left_is_observed_before_deadline(self):
        owners = self.owners(); clock = Clock(850_000); targets = []
        def native(target):
            targets.append(target); clock.now = target
            if target >= 920_000:
                for future in owners.values(): future.set_result('ready')
        with patch.object(bench, 'wait', side_effect=AssertionError('No condition wait')):
            proof = self.run_wait(owners, clock, native)
        self.assertEqual(targets, [900_000, 950_000])
        self.assertEqual(proof['end_ns'], 950_000)
        self.assertEqual(proof['wait_calls'], 2)
        self.assertEqual(proof['native_tail_window_us'], 400)
        self.assertEqual(proof['native_tail_tick_max_us'], 50)
        self.assertEqual(owners['rear'].result(), 'ready')

    def test_remaining_below_fifty_microseconds_still_gets_earlier_observation(self):
        owners = self.owners(); clock = Clock(960_000); targets = []
        def native(target):
            targets.append(target); clock.now = target
            for future in owners.values(): future.set_result('ready')
        proof = self.run_wait(owners, clock, native)
        self.assertEqual(targets, [980_000]); self.assertLess(proof['end_ns'], 1_000_000)

    def test_normal_poll_targets_remain_two_hundred_microseconds(self):
        owners = self.owners(); clock = Clock(1_000); targets = []
        def native(target):
            targets.append(target); clock.now = target
            if len(targets) == 2:
                for future in owners.values(): future.set_result('ready')
        self.run_wait(owners, clock, native)
        self.assertEqual(targets, [201_000, 401_000])

    def test_never_targets_deadline_with_at_least_two_nanoseconds_left(self):
        for remaining in (2, 3, 19, 49_999, 50_000, 99_999, 100_000, 400_000):
            now = 1_000_000-remaining
            with self.subTest(remaining=remaining):
                target = bench._readiness_poll_target(now, 1_000_000)
                self.assertLess(now, target); self.assertLess(target, 1_000_000)
                self.assertLessEqual(target-now, 50_000)
        self.assertEqual(bench._readiness_poll_target(999_999, 1_000_000), 1_000_000)

    def test_unfinished_tail_is_finite_and_retains_actual_failure_poll(self):
        owners = self.owners(); clock = Clock(600_000); targets = []
        def native(target): targets.append(target); clock.now = target
        with self.assertRaises(TimeoutError) as caught:
            self.run_wait(owners, clock, native)
        self.assertLess(len(targets), 32); self.assertEqual(targets[-1], 1_000_000)
        proof = caught.exception.readiness_poll_failure
        self.assertEqual(proof['last_poll_before_ns'], 999_999)
        self.assertEqual(proof['last_poll_wake_ns'], 1_000_000)
        self.assertEqual(proof['last_poll_returned_ns'], 1_000_000)
        self.assertEqual(proof['decision_ns'], 1_000_000)
        self.assertEqual(proof['wait_calls_attempted'], len(targets))
        self.assertEqual(proof['last_ready_count'], 0)

    def test_equality_and_oversleep_reject_even_when_both_ready(self):
        for returned in (1_000_000, 1_000_001, 1_300_000):
            owners = self.owners(); clock = Clock(920_000)
            def native(target):
                clock.now = returned
                for future in owners.values(): future.set_result('late')
            with self.subTest(returned=returned), self.assertRaises(TimeoutError) as caught:
                self.run_wait(owners, clock, native)
            self.assertEqual(caught.exception.readiness_poll_failure['decision_ns'], returned)
            self.assertEqual(caught.exception.readiness_poll_failure['last_ready_count'], 2)

    def test_completion_clock_cannot_extend_deadline(self):
        owners = self.owners(True)
        with self.assertRaises(TimeoutError) as caught:
            self.run_wait(owners, Mock(side_effect=[990_000, 999_999, 1_000_000]))
        trace = caught.exception.readiness_poll_failure
        self.assertEqual(trace['stage'], 'completion_clock')
        self.assertEqual(trace['decision_ns'], 1_000_000)

    def test_guard_check_can_reject_ready_result_and_has_no_invented_decision_time(self):
        failure = RuntimeError('guard cancelled')
        with self.assertRaises(RuntimeError) as caught:
            self.run_wait(self.owners(True), Clock(900_000), check=Mock(side_effect=failure))
        self.assertIs(caught.exception, failure)
        self.assertEqual(failure.readiness_poll_failure['stage'], 'guard_check')
        self.assertIsNone(failure.readiness_poll_failure['decision_ns'])

    def test_owner_error_identity_wins_over_native_cancellation(self):
        owners = self.owners(); failure = OSError('original owner failure')
        def native(target):
            owners['front'].set_exception(failure)
            raise KeyboardInterrupt('native cancel')
        with self.assertRaises(OSError) as caught:
            self.run_wait(owners, Clock(900_000), native)
        self.assertIs(caught.exception, failure)
        self.assertEqual(failure.readiness_poll_failure['wait_calls_attempted'], 1)
        self.assertIsNone(failure.readiness_poll_failure['last_poll_returned_ns'])

    def test_ready_error_wins_over_deadline_and_other_pending_owner(self):
        owners = self.owners(); failure = OSError('owner')
        owners['front'].set_exception(failure); native = Mock()
        with self.assertRaises(OSError) as caught:
            self.run_wait(owners, Clock(1_000_000), native)
        self.assertIs(caught.exception, failure); native.assert_not_called()

    def test_owner_cancelled_is_not_ready_success(self):
        owners = self.owners(); owners['rear'].cancel(); native = Mock()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.run_wait(owners, Clock(900_000), native)
        native.assert_not_called()

    def test_native_cancel_identity_preserved(self):
        failure = KeyboardInterrupt('cancel')
        with self.assertRaises(KeyboardInterrupt) as caught:
            self.run_wait(self.owners(), Clock(900_000), Mock(side_effect=failure))
        self.assertIs(caught.exception, failure)

    def test_early_native_return_and_clock_reversal_are_rejected(self):
        for returned in (949_999, 899_999, True, float('nan')):
            owners = self.owners(); clock = Clock(900_000)
            def native(target): clock.now = returned
            with self.subTest(returned=returned), self.assertRaises(ValueError) as caught:
                self.run_wait(owners, clock, native)
            json.dumps(caught.exception.readiness_poll_failure, allow_nan=False)
        # Return was valid, but the next decision clock goes backwards.
        with self.assertRaisesRegex(ValueError, 'Noncausal'):
            self.run_wait(self.owners(), Mock(side_effect=[900_000, 900_000, 950_000, 949_999]), lambda _: None)

    def test_fallback_condition_wait_keeps_original_finite_deadline(self):
        owners = self.owners(); clock = Clock(900_000)
        def complete(values, *, timeout, return_when):
            self.assertAlmostEqual(timeout, .0001)
            for future in values: future.set_result('done')
        with patch.object(bench, 'wait', side_effect=complete) as wait:
            proof = self.run_wait(owners, clock)
        self.assertEqual(wait.call_count, 1); self.assertEqual(proof['wait_calls'], 1)
        self.assertIsNone(proof['native_tail_tick_max_us'])

    def test_failure_observer_copies_trace_and_keeps_distinct_observation_time(self):
        owners = self.owners(True); clock = Clock(1_000_001)
        with self.assertRaises(TimeoutError) as caught:
            self.run_wait(owners, clock)
        original = dict(caught.exception.readiness_poll_failure)
        proof = bench._output_join_failure_proof(caught.exception, 'readiness_join', owners, lambda: 1_010_000)
        self.assertEqual(proof['captured_ns'], 1_010_000)
        self.assertEqual(proof['readiness_poll_failure']['decision_ns'], 1_000_001)
        self.assertFalse(proof['capture_is_deadline_decision_time'])
        proof['readiness_poll_failure']['decision_ns'] = 1
        self.assertEqual(caught.exception.readiness_poll_failure, original)

    def test_exception_annotation_failure_never_replaces_original_error(self):
        class ImmutableError(RuntimeError):
            def __setattr__(self, key, value): raise RuntimeError('no annotation')
        failure = ImmutableError('original'); owners = self.owners()
        owners['front'].set_exception(failure)
        with self.assertRaises(ImmutableError) as caught:
            self.run_wait(owners, Clock(900_000))
        self.assertIs(caught.exception, failure)

    def test_bad_exception_trace_accessor_does_not_hide_owner_state(self):
        class BadTraceError(RuntimeError):
            @property
            def readiness_poll_failure(self): raise RuntimeError('unavailable')
        proof = bench._output_join_failure_proof(BadTraceError(), 'readiness_join', self.owners(True), lambda: 1_000_000)
        self.assertTrue(proof['readiness_poll_trace_unavailable'])
        self.assertTrue(proof['owner_future_states']['front']['done'])
        self.assertEqual(proof['captured_ns'], 1_000_000)

    def test_collector_preserves_failure_poll_trace_and_settles_both_owners(self):
        import test_native_output_join_wait as existing
        original = bench._await_output_ready
        def expire(*args, **kwargs):
            kwargs['clock'] = lambda: kwargs['deadline_ns']
            return original(*args, **kwargs)
        with patch.object(bench, '_await_output_ready', side_effect=expire):
            report, rows, sessions, _ = existing.NativeOutputIntegrationTests().integration()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 0)
        gate = rows[0]['voltage_fast_pipeline']
        self.assertTrue(gate['output_join_cleanup_only'])
        self.assertEqual(set(rows[0]['output']), {'front', 'rear'})
        trace = gate['output_join_failure_proof']['readiness_poll_failure']
        self.assertEqual(trace['decision_ns'], gate['output_join_deadline_ns'])
        self.assertEqual(trace['wait_calls_attempted'], 0)
        self.assertNotIn('observed', rows[0])
        self.assertTrue(all(s.phases == ['feedback', 'voltage', 'output'] for s in sessions.values()))


if __name__ == '__main__':
    unittest.main()
