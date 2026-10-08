"""Deterministic stdlib-only queue/lifetime tests; no CAN, model or native code."""
from concurrent.futures import CancelledError, Future
from contextlib import redirect_stderr
import io
import gc
import threading
import unittest
import weakref
from unittest.mock import patch

from experiments.diagnostic_dedicated_owners import owners


class DedicatedOwnerTests(unittest.TestCase):
    def submit(self, pool, owner, fn, *args, **kwargs):
        return pool.submit(owner, fn, *args, task_deadline_ns=20_000_001,
                           task_label='test', **kwargs)

    def test_default_off_plan_and_constructor_start_no_workers(self):
        with patch.object(owners, 'ThreadPoolExecutor', side_effect=AssertionError('started')):
            p = owners.DedicatedOwners()
            self.assertEqual(p.topology()['actual_workers_started'], 0)
            with self.assertRaisesRegex(RuntimeError, 'Explicit'):
                p.start()
            self.assertFalse(owners.plan()['selected'])
            self.assertEqual(owners.plan(enabled=True)['workers_started'], 0)
            p.close()

    def test_three_prestarted_persistent_distinct_workers_and_genuine_futures(self):
        with owners.DedicatedOwners(enabled=True) as p:
            self.assertEqual(p.topology()['actual_workers_alive'], 3)
            identities = {}
            for name in owners.OWNERS:
                a = self.submit(p, name, threading.get_native_id)
                b = self.submit(p, name, threading.get_native_id)
                self.assertIs(type(a), Future)
                self.assertEqual(a.result(timeout=3), b.result(timeout=3))
                self.assertIs(p.origin(a).future, a)
                self.assertEqual(p.origin(a).owner, name)
                identities[name] = a.result()
            self.assertEqual(len(set(identities.values())), 3)
            self.assertEqual(identities, p.topology()['native_thread_id_by_owner'])
            threads = dict(p.worker_threads)
        self.assertTrue(all(not t.is_alive() for t in threads.values()))

    def test_blocked_imu_and_queued_validation_do_not_delay_either_bus_queue(self):
        entered, release, validation = threading.Event(), threading.Event(), threading.Event()
        with owners.DedicatedOwners(enabled=True) as p:
            def imu():
                entered.set()
                self.assertTrue(release.wait(timeout=3))
                return 'imu'
            first = self.submit(p, 'imu_validation', imu)
            self.assertTrue(entered.wait(timeout=3))
            queued = self.submit(p, 'imu_validation', validation.set)
            try:
                front = self.submit(p, 'front', lambda: 'front')
                rear = self.submit(p, 'rear', lambda: 'rear')
                self.assertEqual(front.result(timeout=3), 'front')
                self.assertEqual(rear.result(timeout=3), 'rear')
                self.assertFalse(first.done())
                self.assertFalse(queued.done())
                self.assertFalse(validation.is_set())
            finally:
                release.set()
            self.assertEqual(first.result(timeout=3), 'imu')
            queued.result(timeout=3)

    def test_front_fifo_does_not_share_rear_queue(self):
        entered, release = threading.Event(), threading.Event()
        order = []
        with owners.DedicatedOwners(enabled=True) as p:
            def first():
                entered.set()
                release.wait(timeout=3)
                order.append(1)
            a = self.submit(p, 'front', first)
            self.assertTrue(entered.wait(timeout=3))
            b = self.submit(p, 'front', lambda: order.append(2))
            try:
                self.assertEqual(self.submit(p, 'rear', lambda: 42).result(timeout=3), 42)
                self.assertFalse(b.done())
            finally:
                release.set()
            a.result(timeout=3); b.result(timeout=3)
            self.assertEqual(order, [1, 2])

    def test_original_absolute_deadline_and_argument_objects_are_unchanged(self):
        wire, captured = bytearray(b'raw'), []
        deadline = 123456789
        def task(value, *, deadline_ns):
            captured.append((value, deadline_ns))
            return value
        with owners.DedicatedOwners(enabled=True) as p:
            f = p.submit('front', task, wire, deadline_ns=deadline,
                         task_deadline_ns=deadline, task_label='original')
            self.assertIs(f.result(timeout=3), wire)
            self.assertIs(captured[0][0], wire)
            self.assertEqual(captured[0][1], deadline)
            self.assertEqual(p.origin(f).deadline_ns, deadline)

    def test_original_task_deadline_failure_and_raw_exception_are_not_admitted_or_wrapped(self):
        error = TimeoutError('original deadline expired')
        error.records, error.stats = (b'partial',), {'reply_count': 1}
        def task(*, deadline_ns, now_ns):
            if now_ns >= deadline_ns:
                raise error
            self.fail('deadline widened')
        with owners.DedicatedOwners(enabled=True) as p:
            f = p.submit('rear', task, deadline_ns=10, now_ns=10,
                         task_deadline_ns=10, task_label='expired')
            with self.assertRaises(TimeoutError) as caught:
                f.result(timeout=3)
            self.assertIs(caught.exception, error)
            self.assertIs(f.exception(), error)
            self.assertEqual(error.records, (b'partial',))

    def test_queued_cancellation_keeps_original_future_and_running_owner(self):
        entered, release, invoked = threading.Event(), threading.Event(), threading.Event()
        with owners.DedicatedOwners(enabled=True) as p:
            def running():
                entered.set()
                release.wait(timeout=3)
                return 'settled'
            a = self.submit(p, 'front', running)
            self.assertTrue(entered.wait(timeout=3))
            b = self.submit(p, 'front', invoked.set)
            try:
                cancelled = p.cancel_pending()
                self.assertEqual([row.future for row in cancelled], [b])
                self.assertIs(p.origin(b).future, b)
                self.assertFalse(a.cancel())
                with self.assertRaises(CancelledError):
                    b.result()
                self.assertFalse(invoked.is_set())
            finally:
                release.set()
            self.assertEqual(a.result(timeout=3), 'settled')

    def test_original_cancel_exception_has_priority_without_scheduler_reordering(self):
        error = RuntimeError('original cancellation wins')
        error.records = ('original evidence',)
        def original_check(*, cancelled, deadline_ns, now_ns):
            if cancelled:
                raise error
            if now_ns >= deadline_ns:
                raise TimeoutError('deadline')
        with owners.DedicatedOwners(enabled=True) as p:
            f = p.submit('rear', original_check, cancelled=True, deadline_ns=10, now_ns=10,
                         task_deadline_ns=10, task_label='cancel-priority')
            with self.assertRaises(RuntimeError) as caught:
                f.result(timeout=3)
            self.assertIs(caught.exception, error)
            self.assertEqual(error.records, ('original evidence',))

    def test_same_thread_finalizers_after_tasks_then_all_owners_joined(self):
        initialized, finalized, done = {}, {}, set()
        def init(owner):
            initialized[owner] = threading.get_native_id()
        def finish(owner):
            self.assertIn(owner, done)
            finalized[owner] = threading.get_native_id()
        p = owners.DedicatedOwners(enabled=True, worker_initializer=init,
                                   worker_finalizer=finish).start()
        for name in owners.OWNERS:
            self.submit(p, name, lambda owner: done.add(owner), name).result(timeout=3)
        threads = dict(p.worker_threads)
        p.close()
        self.assertEqual(initialized, finalized)
        self.assertEqual(set(finalized), set(owners.OWNERS))
        self.assertTrue(all(not t.is_alive() for t in threads.values()))
        self.assertEqual(p.topology()['actual_workers_alive'], 0)
        with self.assertRaises(RuntimeError):
            self.submit(p, 'front', lambda: None)
        p.close()

    def test_initializer_failure_joins_already_started_owner(self):
        initialized = {}
        error = RuntimeError('original initializer')
        def init(owner):
            initialized[owner] = threading.current_thread()
            if owner == 'rear':
                raise error
        p = owners.DedicatedOwners(enabled=True, worker_initializer=init)
        with redirect_stderr(io.StringIO()), self.assertRaises(RuntimeError):
            p.start()
        self.assertTrue(all(not t.is_alive() for t in initialized.values()))
        self.assertEqual(p.topology()['state'], 'CLOSED')

    def test_cleanup_failure_still_joins_all_owners(self):
        error = RuntimeError('restore failure')
        finalized = []
        def finish(owner):
            finalized.append(owner)
            if owner == 'front':
                raise error
        p = owners.DedicatedOwners(enabled=True, worker_finalizer=finish).start()
        threads = dict(p.worker_threads)
        with self.assertRaises(RuntimeError) as caught:
            p.close()
        self.assertIs(caught.exception, error)
        self.assertEqual(set(finalized), set(owners.OWNERS))
        self.assertTrue(all(not t.is_alive() for t in threads.values()))

    def test_context_preserves_primary_error_if_restore_also_fails(self):
        primary, cleanup = ValueError('original task'), RuntimeError('restore')
        def finish(owner):
            if owner == 'front':
                raise cleanup
        with self.assertRaises(ValueError) as caught:
            with owners.DedicatedOwners(enabled=True, worker_finalizer=finish):
                raise primary
        self.assertIs(caught.exception, primary)
        self.assertTrue(any('restore' in note for note in primary.__notes__))

    def test_foreign_future_and_wrong_owner_are_rejected(self):
        with owners.DedicatedOwners(enabled=True) as p:
            with self.assertRaises(ValueError):
                p.origin(Future())
            with self.assertRaises(ValueError):
                self.submit(p, 'shared', lambda: None)

    def test_identity_retirement_rejects_pending_foreign_or_repeated_futures_atomically(self):
        entered, release = threading.Event(), threading.Event()
        with owners.DedicatedOwners(enabled=True) as p:
            def blocked():
                entered.set()
                release.wait(timeout=3)
            a = self.submit(p, 'front', blocked)
            self.assertTrue(entered.wait(timeout=3))
            b = self.submit(p, 'rear', lambda: b'raw')
            b.result(timeout=3)
            try:
                with self.assertRaises(RuntimeError):
                    p.release_settled((b, a))
                self.assertIs(p.origin(b).future, b)
                with self.assertRaises(ValueError):
                    p.release_settled((b, Future()))
                self.assertIs(p.origin(b).future, b)
                with self.assertRaises(ValueError):
                    p.release_settled((b, b))
            finally:
                release.set()
            a.result(timeout=3)
            self.assertEqual(p.release_settled((a, b)), 2)
            self.assertEqual(b.result(), b'raw')
            with self.assertRaises(ValueError):
                p.origin(a)

    def test_retired_future_and_owned_result_have_no_scheduler_reference_leak(self):
        class Raw:
            pass
        with owners.DedicatedOwners(enabled=True) as p:
            payload = Raw()
            raw_ref = weakref.ref(payload)
            f = self.submit(p, 'front', lambda value: value, payload)
            self.assertIs(f.result(timeout=3), payload)
            future_ref = weakref.ref(f)
            # The next same-owner task proves the previous work item was released.
            barrier = self.submit(p, 'front', lambda: None)
            barrier.result(timeout=3)
            p.release_settled((f, barrier))
            del f, barrier, payload
            gc.collect()
            self.assertIsNone(future_ref())
            self.assertIsNone(raw_ref())

    def test_cross_thread_submit_is_rejected_before_adding_a_task(self):
        with owners.DedicatedOwners(enabled=True) as p:
            errors = []
            def other():
                try:
                    self.submit(p, 'front', lambda: None)
                except BaseException as error:
                    errors.append(error)
            t = threading.Thread(target=other)
            t.start(); t.join(timeout=3)
            self.assertFalse(t.is_alive())
            self.assertEqual(len(errors), 1)
            self.assertIsInstance(errors[0], RuntimeError)
            self.assertEqual(sum(p.topology()['submitted_tasks_by_owner'].values()), 0)


if __name__ == '__main__':
    unittest.main()
