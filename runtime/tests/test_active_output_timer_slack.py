"""Offline active-output timer-slack scope tests; no prctl or robot devices."""

from concurrent.futures import ThreadPoolExecutor
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import active_output_timer_slack as active


class FakePrctl:
    def __init__(self, *, original=73_000, fail_second_apply=False,
                 bad_during_once=False, fail_first_restore=False,
                 fail_main_apply=False, bad_main_readback=False,
                 fail_main_restore=False):
        self.original = original
        self.fail_second_apply = fail_second_apply
        self.bad_during_once = bad_during_once
        self.fail_first_restore = fail_first_restore
        self.fail_main_apply = fail_main_apply
        self.bad_main_readback = bad_main_readback
        self.fail_main_restore = fail_main_restore
        self.main_tid = threading.get_native_id()
        self.values = {}
        self.calls = []
        self.apply_count = 0
        self.restore_count = 0
        self.lock = threading.Lock()

    def get(self):
        tid = threading.get_native_id()
        with self.lock:
            self.calls.append(("get", tid))
            value = self.values.get(tid, self.original)
            if self.bad_main_readback and tid == self.main_tid and value == active.OPT_IN_NS:
                self.bad_main_readback = False
                return self.original
            if self.bad_during_once and value == active.OPT_IN_NS:
                self.bad_during_once = False
                return self.original
            return value

    def set(self, value):
        tid = threading.get_native_id()
        with self.lock:
            self.calls.append(("set", tid, value))
            self.values[tid] = value
            if value == active.OPT_IN_NS:
                self.apply_count += 1
                if self.fail_second_apply and self.apply_count == 2:
                    raise active.ActiveTimerSlackError("Injected ambiguous apply failure")
                if self.fail_main_apply and tid == self.main_tid:
                    raise active.ActiveTimerSlackError("Injected ambiguous main apply failure")
            else:
                self.restore_count += 1
                if self.fail_first_restore and self.restore_count == 1:
                    self.values[tid] = active.OPT_IN_NS
                    raise active.ActiveTimerSlackError("Injected restore failure")
                if self.fail_main_restore and tid == self.main_tid:
                    self.fail_main_restore = False
                    self.values[tid] = active.OPT_IN_NS
                    raise active.ActiveTimerSlackError("Injected main restore failure")


class ThreePools:
    def __enter__(self):
        self.front = ThreadPoolExecutor(max_workers=1)
        self.rear = ThreadPoolExecutor(max_workers=1)
        self.imu = ThreadPoolExecutor(max_workers=1)
        self.workers = SimpleNamespace(pools={"front": self.front, "rear": self.rear})
        return self

    def __exit__(self, *_):
        self.front.shutdown(wait=True)
        self.rear.shutdown(wait=True)
        self.imu.shutdown(wait=True)


class InterruptedSuccessfulFuture:
    """Interrupt the caller after the real worker has already changed state."""
    def __init__(self, future):
        self.future = future
        self.interrupted = False

    def result(self):
        value = self.future.result()
        if not self.interrupted:
            self.interrupted = True
            raise KeyboardInterrupt('Injected caller signal after worker completion')
        return value

    def done(self):
        return self.future.done()

    def cancelled(self):
        return self.future.cancelled()

    def exception(self):
        return self.future.exception()


class ActiveOutputTimerSlackTests(unittest.TestCase):
    def test_default_is_a_true_noop_even_without_pools_or_linux(self):
        scope = active.ActiveOutputTimerSlack()
        with patch.object(active.base, "require_supported_platform") as platform, \
                patch.object(active.base, "_load_prctl") as loader:
            scope.apply()
            scope.restore()
        platform.assert_not_called()
        loader.assert_not_called()
        self.assertEqual(scope.report["status"], "inactive")
        self.assertEqual(scope.report["workers"], {})
        self.assertIsNone(scope.report["main"])

    def test_only_explicit_1us_is_allowed(self):
        for value in (0, 999, 50_000, True, 1_000.0, "1000"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                active.ActiveOutputTimerSlack(value)

    def test_main_and_three_workers_restore_their_own_exact_values(self):
        backend = FakePrctl()
        main_tid = threading.get_native_id()
        with ThreePools() as pools, \
                patch.object(active.base, "require_supported_platform"), \
                patch.object(active.base, "_load_prctl", return_value=backend):
            originals = {"front": 73_000, "rear": 81_000, "imu": 54_000}
            for role, pool in (("front", pools.front), ("rear", pools.rear), ("imu", pools.imu)):
                backend.values[pool.submit(threading.get_native_id).result()] = originals[role]
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            rows = scope.report["workers"]
            self.assertEqual(set(rows), set(active.ROLES))
            self.assertEqual(len({row["native_tid"] for row in rows.values()}), 3)
            self.assertTrue(scope.report["apply_verified"])
            self.assertTrue(all(row["original_ns"] == originals[role] and row["during_ns"] == 1_000
                                and row["applied"] for role, row in rows.items()))
            self.assertTrue(all(backend.values[row["native_tid"]] == 1_000 for row in rows.values()))
            self.assertEqual(scope.report['main']['native_tid'], main_tid)
            self.assertEqual(scope.report['main']['original_ns'], 73_000)
            self.assertEqual(backend.values[main_tid], 1_000)
            self.assertEqual(scope.report['schema'], 'singularitydog.active-output-timer-slack.v2')
            self.assertEqual(scope.report['scope'], 'main_thread_and_three_io_workers')
            set_calls = [call for call in backend.calls if call[0] == 'set']
            self.assertEqual([call[1] == main_tid for call in set_calls], [False, False, False, True])
            scope.restore()
            self.assertTrue(scope.report["restoration_complete"])
            self.assertEqual(scope.report["status"], "restored")
            self.assertTrue(all(row["before_restore_ns"] == 1_000 and
                                row["after_ns"] == originals[role] and row["restored"]
                                for role, row in rows.items()))
            self.assertTrue(all(backend.values[row["native_tid"]] == originals[role]
                                for role, row in rows.items()))
            self.assertTrue(scope.report['main']['restored'])
            self.assertEqual(scope.report['main']['after_ns'], 73_000)
            self.assertEqual(backend.values[main_tid], 73_000)

    def test_ambiguous_partial_apply_rolls_back_every_changed_worker(self):
        backend = FakePrctl(fail_second_apply=True)
        with ThreePools() as pools, \
                patch.object(active.base, "require_supported_platform"), \
                patch.object(active.base, "_load_prctl", return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            with self.assertRaisesRegex(active.ActiveTimerSlackError, "Three distinct"):
                scope.apply(pools.workers, pools.imu)
            self.assertFalse(scope.report["apply_verified"])
            self.assertTrue(scope.report["restoration_complete"])
            self.assertEqual(scope.report["status"], "apply_failed_restored")
            self.assertEqual(len(scope.report["workers"]), 3)
            self.assertTrue(all(row["restored"] and backend.values[row["native_tid"]] == 73_000
                                for row in scope.report["workers"].values()))

    def test_wrong_apply_readback_also_rolls_back(self):
        backend = FakePrctl(bad_during_once=True)
        with ThreePools() as pools, \
                patch.object(active.base, "require_supported_platform"), \
                patch.object(active.base, "_load_prctl", return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.apply(pools.workers, pools.imu)
            self.assertTrue(scope.report["restoration_complete"])
            self.assertTrue(all(value == 73_000 for value in backend.values.values()))

    def test_restore_failure_attempts_all_roles_and_can_be_retried(self):
        backend = FakePrctl(fail_first_restore=True)
        with ThreePools() as pools, \
                patch.object(active.base, "require_supported_platform"), \
                patch.object(active.base, "_load_prctl", return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            with self.assertRaisesRegex(active.ActiveTimerSlackError, "restoration unconfirmed"):
                scope.restore()
            self.assertFalse(scope.report["restoration_complete"])
            self.assertEqual(backend.restore_count, 4)
            self.assertEqual(sum(row["restored"] is True for row in scope.report["workers"].values()), 2)
            # A retry still records the earlier failure; it cannot silently
            # turn incomplete first-attempt evidence into a successful run.
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertTrue(all(value == 73_000 for value in backend.values.values()))
            self.assertFalse(scope.report["restoration_complete"])

    def test_main_ambiguous_apply_failure_restores_all_four_threads(self):
        for options in ({'fail_main_apply': True}, {'bad_main_readback': True}):
            with self.subTest(options=options), ThreePools() as pools, \
                    patch.object(active.base, 'require_supported_platform'), \
                    patch.object(active.base, '_load_prctl', return_value=FakePrctl(**options)):
                scope = active.ActiveOutputTimerSlack(1_000)
                with self.assertRaises(active.ActiveTimerSlackError):
                    scope.apply(pools.workers, pools.imu)
                self.assertFalse(scope.report['apply_verified'])
                self.assertTrue(scope.report['restoration_complete'])
                self.assertTrue(scope.report['main']['restored'])
                self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))

    def test_main_restore_failure_is_visible_and_retry_restores_original(self):
        backend = FakePrctl(fail_main_restore=True)
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertFalse(scope.report['main']['restored'])
            self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertEqual(backend.values[backend.main_tid], 73_000)
            self.assertFalse(scope.report['restoration_complete'])

    def test_main_drift_restores_original_without_claiming_verified_completion(self):
        backend = FakePrctl()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            backend.values[backend.main_tid] = 50_000
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertEqual(backend.values[backend.main_tid], 73_000)
            self.assertTrue(scope.report['main']['restored'])
            self.assertFalse(scope.report['restoration_complete'])

    def test_wrong_thread_restore_does_not_write_main_original_to_wrong_thread(self):
        backend = FakePrctl()
        with ThreePools() as pools, ThreadPoolExecutor(max_workers=1) as wrong, \
                patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            wrong_tid = wrong.submit(threading.get_native_id).result()
            with self.assertRaises(active.ActiveTimerSlackError):
                wrong.submit(scope.restore).result()
            self.assertNotIn(wrong_tid, [call[1] for call in backend.calls])
            self.assertEqual(backend.values[backend.main_tid], 1_000)
            self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))
            # A retry on the owner restores the value while preserving failure history.
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertEqual(backend.values[backend.main_tid], 73_000)

    def test_interrupted_apply_result_is_reaped_before_rollback(self):
        backend = FakePrctl()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            original_submit = pools.front.submit
            def submit(function, *args):
                result = original_submit(function, *args)
                return InterruptedSuccessfulFuture(result) if function.__name__ == '_apply_one' else result
            with patch.object(pools.front, 'submit', side_effect=submit):
                scope = active.ActiveOutputTimerSlack(1_000)
                with self.assertRaises(active.ActiveTimerSlackError):
                    scope.apply(pools.workers, pools.imu)
            self.assertIsNone(scope.report['main'])
            self.assertFalse(scope.report['apply_verified'])
            self.assertTrue(scope.report['restoration_complete'])
            self.assertEqual(len(scope.report['workers']), 3)
            self.assertTrue(all(v == 73_000 for v in backend.values.values()))
            self.assertTrue(any('KeyboardInterrupt' in e for e in scope.report['errors']))

    def test_interrupted_restore_result_is_reaped_and_main_restoration_still_attempted(self):
        backend = FakePrctl()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            original_submit = pools.front.submit
            with patch.object(pools.front, 'submit', side_effect=lambda fn, *args:
                              InterruptedSuccessfulFuture(original_submit(fn, *args))):
                with self.assertRaises(active.ActiveTimerSlackError):
                    scope.restore()
            self.assertTrue(scope.report['main']['restored'])
            self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))
            self.assertTrue(all(v == 73_000 for v in backend.values.values()))
            self.assertFalse(scope.report['restoration_complete'])

    def test_main_original_read_failure_never_sets_main_and_rolls_back_workers(self):
        backend = FakePrctl()
        original_get = backend.get
        def read():
            if threading.get_native_id() == backend.main_tid:
                raise active.ActiveTimerSlackError('Injected main initial read failure')
            return original_get()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend), \
                patch.object(backend, 'get', side_effect=read):
            scope = active.ActiveOutputTimerSlack(1_000)
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.apply(pools.workers, pools.imu)
            self.assertFalse(scope.report['main']['set_attempted'])
            self.assertTrue(scope.report['restoration_complete'])
            self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))
            self.assertNotIn(backend.main_tid, backend.values)

    def test_apply_submit_interruption_after_enqueue_recovers_owner_row_without_future(self):
        backend = FakePrctl()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            original_submit = pools.front.submit
            def submit(fn, *args):
                future = original_submit(fn, *args)
                if fn.__name__ == '_apply_one':
                    future.result()
                    raise KeyboardInterrupt('Interrupted after enqueue, before returning Future')
                return future
            scope = active.ActiveOutputTimerSlack(1_000)
            with patch.object(pools.front, 'submit', side_effect=submit):
                with self.assertRaises(active.ActiveTimerSlackError):
                    scope.apply(pools.workers, pools.imu)
            self.assertEqual(set(scope.report['workers']), set(active.ROLES))
            self.assertTrue(all(row['restored'] for row in scope.report['workers'].values()))
            self.assertTrue(all(v == 73_000 for v in backend.values.values()))
            self.assertTrue(scope.report['restoration_complete'])
            self.assertEqual(scope.report['status'], 'apply_failed_restored')
            self.assertIsNone(scope.report['main'])

    def test_lost_apply_future_and_rejected_restore_barrier_never_claims_completion(self):
        backend = FakePrctl()
        with ThreePools() as pools, patch.object(active.base, 'require_supported_platform'), \
                patch.object(active.base, '_load_prctl', return_value=backend):
            original_submit = pools.front.submit
            def submit(fn, *args):
                if fn.__name__ == '_apply_one':
                    original_submit(fn, *args).result()
                    raise KeyboardInterrupt('Lost apply Future')
                raise RuntimeError('Restore queue unavailable')
            scope = active.ActiveOutputTimerSlack(1_000)
            with patch.object(pools.front, 'submit', side_effect=submit):
                with self.assertRaises(active.ActiveTimerSlackError):
                    scope.apply(pools.workers, pools.imu)
            self.assertFalse(scope.report['restoration_complete'])
            self.assertEqual(scope.report['status'], 'restore_failed')
            # After recovering the queue, physical restoration can be retried,
            # but the failed first restoration remains visible.
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertTrue(all(v == 73_000 for v in backend.values.values()))
            self.assertFalse(scope.report['restoration_complete'])

    def test_external_worker_drift_is_restored_but_never_reported_as_verified(self):
        backend = FakePrctl()
        with ThreePools() as pools, \
                patch.object(active.base, "require_supported_platform"), \
                patch.object(active.base, "_load_prctl", return_value=backend):
            scope = active.ActiveOutputTimerSlack(1_000)
            scope.apply(pools.workers, pools.imu)
            front_tid = scope.report["workers"]["front"]["native_tid"]
            backend.values[front_tid] = 50_000
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertFalse(scope.report["restoration_complete"])
            self.assertEqual(backend.values[front_tid], 73_000)
            self.assertEqual(scope.report["workers"]["front"]["before_restore_ns"], 50_000)

    def test_opt_in_off_linux_fails_before_libc_or_worker_mutation(self):
        with ThreePools() as pools, \
                patch.object(active.base.sys, "platform", "darwin"), \
                patch.object(active.base, "_load_prctl") as loader:
            scope = active.ActiveOutputTimerSlack(1_000)
            with self.assertRaises(active.base.TimerSlackError):
                scope.apply(pools.workers, pools.imu)
            self.assertEqual(scope.report["status"], "apply_failed")
            self.assertTrue(scope.report["restoration_complete"])
        loader.assert_not_called()

    def test_pool_shape_rejected_before_any_os_access(self):
        with ThreadPoolExecutor(max_workers=2) as wrong, \
                patch.object(active.base, "_load_prctl") as loader:
            scope = active.ActiveOutputTimerSlack(1_000)
            workers = SimpleNamespace(pools={"front": wrong, "rear": wrong})
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.apply(workers, wrong)
        loader.assert_not_called()
        self.assertEqual(scope.report["status"], "apply_failed")


if __name__ == "__main__":
    unittest.main()
