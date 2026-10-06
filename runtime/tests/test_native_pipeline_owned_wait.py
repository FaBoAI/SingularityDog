"""Synthetic C callbacks and fake sessions; no device or native library load."""
from concurrent.futures import Future
import ctypes as C
import threading
import unittest
import weakref
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import native_diagnostic_transport as native
from test_native_pipeline_benchmark import Device, Observer, Session


WAIT = C.CFUNCTYPE(C.c_int, C.c_int, C.c_uint64, C.c_uint32,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)


class Library:
    def __init__(self, callback):
        self.calls = []
        self.abi_calls = 0
        def execute(fd, target, spin, actual, error, size):
            self.calls.append((fd, target, spin, C.addressof(actual.contents),
                               C.cast(error, C.c_void_p).value))
            return callback(target, actual, error)
        self.sd_wait_until = WAIT(execute)
    def sd_abi(self):
        self.abi_calls += 1
        return 1


class CollectionOwnedWaitTests(unittest.TestCase):
    def run_three_phases(self, *, owned):
        state = {'now': 1000, 'phase_calls': 0}
        checks = []
        def advance(target, actual, error):
            state['now'] = target
            actual.contents.value = target
            state['phase_calls'] += 1
            if state['phase_calls'] == 1:
                state['owners']['front'].set_result('front')
            else:
                state['owners']['rear'].set_result('rear')
                if state['validation'] is not None:
                    state['validation'].set_result('validated')
            return 0
        library = Library(advance)
        if owned:
            callback = bench._collection_deadline_wait(library, 23, 500)
        else:
            callback = lambda target: native.wait_until(library, 23, target, spin_us=500)
        proofs = []
        # This stands in for one setup retaining the callback across all phases.
        context = (patch.object(native.C, 'create_string_buffer',
                                side_effect=AssertionError('per-poll allocation'))
                   if owned else patch.object(bench, '_collection_deadline_wait'))
        with context:
            for phase in ('acquisition', 'voltage', 'output'):
                state['owners'] = {scope: Future() for scope in ('front', 'rear')}
                state['validation'] = None if phase == 'output' else Future()
                state['phase_calls'] = 0
                options = {'deadline_ns': state['now']+1_000_000,
                           'deadline_wait': callback, 'clock': lambda: state['now'],
                           'check': lambda: checks.append(state['now']),
                           'thread_clock': lambda: 10}
                if phase == 'output':
                    proof = bench._await_output_ready(state['owners'], **options)
                elif phase == 'acquisition':
                    proof = bench._await_acquisition_ready(
                        state['owners'], state['validation'], **options)
                else:
                    proof = bench._await_voltage_ready(
                        state['owners'], state['validation'], **options)
                proofs.append(proof)
                self.assertEqual([f.result() for f in state['owners'].values()],
                                 ['front', 'rear'])
                self.assertEqual(proof['wait_calls'], 2)
        return proofs, checks, library

    def test_same_native_deadlines_spin_guard_frequency_and_three_phase_results(self):
        old, old_checks, old_library = self.run_three_phases(owned=False)
        new, new_checks, library = self.run_three_phases(owned=True)
        self.assertEqual(new, old)
        self.assertEqual(new_checks, old_checks)
        self.assertEqual(len(new_checks), 9)
        self.assertEqual([r[:3] for r in library.calls],
                         [r[:3] for r in old_library.calls])
        self.assertEqual(library.abi_calls, 1)
        self.assertEqual(len(library.calls), 6)
        self.assertEqual(len({r[3] for r in library.calls}), 1)
        self.assertEqual(len({r[4] for r in library.calls}), 1)

    def test_default_and_fake_sessions_keep_condition_wait_without_native_factory(self):
        with patch.object(native, 'make_owned_waiter',
                          side_effect=AssertionError('native selection forbidden')):
            self.assertIsNone(bench._collection_deadline_wait(object(), -1, None))
            sessions = {scope: Session() for scope in ('front', 'rear')}
            report, rows = bench.collect(sessions, Device(), Observer(),
                                        mode='stop-proxy', cycles=2)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 2)
        self.assertEqual(len(rows), 2)
        self.assertEqual([s.calls for s in sessions.values()], [4, 4])
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])

    def test_callback_retains_verified_library_and_rejects_wrong_coordinator_thread(self):
        def wake(target, actual, error):
            actual.contents.value = target
            return 0
        library = Library(wake)
        retained = weakref.ref(library)
        callback = bench._collection_deadline_wait(library, 23, 200)
        del library
        self.assertIsNotNone(retained())
        self.assertEqual(callback(1000), 1000)
        failures = []
        def other_owner():
            try: callback(2000)
            except BaseException as error: failures.append(error)
        thread = threading.Thread(target=other_owner)
        thread.start(); thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], native.WaitError)
        self.assertEqual(len(retained().calls), 1)
        self.assertEqual(callback(3000), 3000)

    def test_native_cancellation_preserves_original_owner_error_and_stops_waiting(self):
        owners = {scope: Future() for scope in ('front', 'rear')}
        original = OSError('synthetic session closed/owner failure')
        def cancelled(target, actual, error):
            owners['rear'].set_exception(original)
            error[0] = b'x'
            return 1
        library = Library(cancelled)
        callback = bench._collection_deadline_wait(library, 23, 500)
        with self.assertRaises(OSError) as caught:
            bench._await_output_ready(owners, deadline_ns=1_000_000,
                                      deadline_wait=callback, clock=lambda: 1000)
        self.assertIs(caught.exception, original)
        self.assertFalse(owners['front'].done())
        self.assertEqual(len(library.calls), 1)
        self.assertEqual(callback._actual.value, 0)
        self.assertEqual(callback._error.raw, bytes(256))

    def test_200us_poll_clips_to_hard_deadline_and_equal_completion_is_rejected(self):
        owners = {scope: Future() for scope in ('front', 'rear')}
        state = {'now': 1000}
        def wake(target, actual, error):
            state['now'] = target
            actual.contents.value = target
            if target == 301000:
                for future in owners.values(): future.set_result('late')
            return 0
        library = Library(wake)
        callback = bench._collection_deadline_wait(library, 23, 500)
        with self.assertRaises(TimeoutError):
            bench._await_output_ready(owners, deadline_ns=301000,
                deadline_wait=callback, clock=lambda: state['now'])
        targets = [r[1] for r in library.calls]
        self.assertLess(len(targets), 32)
        self.assertEqual(targets[-1], 301000)
        self.assertTrue(all(0 < b-a <= 50_000 for a,b in zip([1000]+targets, targets)))

    def test_setup_failure_does_not_fall_back_or_retry_native_selection(self):
        original = native.WaitError('optional symbol/ABI unavailable')
        with patch.object(native, 'make_owned_waiter', side_effect=original) as factory:
            with self.assertRaises(native.WaitError) as caught:
                bench._collection_deadline_wait(object(), 23, 500)
        self.assertIs(caught.exception, original)
        self.assertEqual(factory.call_count, 1)

    def test_readiness_snapshot_precedes_errors_and_original_failure_preempts_guard(self):
        events = []
        class ObservedFuture(Future):
            def __init__(self, name):
                super().__init__()
                self.name = name
            def done(self):
                events.append(('done', self.name))
                return super().done()
            def exception(self):
                events.append(('exception', self.name))
                return super().exception()
        owners = {scope: ObservedFuture(scope) for scope in ('front', 'rear')}
        original = OSError('front owner failed')
        owners['front'].set_exception(original)
        with self.assertRaises(OSError) as caught:
            bench._await_output_ready(owners, deadline_ns=1_000_000,
                clock=lambda: 1000, check=lambda: events.append(('guard', None)))
        self.assertIs(caught.exception, original)
        self.assertEqual(events, [('done', 'front'), ('done', 'rear'),
                                  ('exception', 'front')])


if __name__ == '__main__':
    unittest.main()
