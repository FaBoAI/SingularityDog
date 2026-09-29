"""Offline active-output timer-slack scope tests; no prctl or robot devices."""

from concurrent.futures import ThreadPoolExecutor
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import active_output_timer_slack as active


class FakePrctl:
    def __init__(self, *, original=73_000, fail_second_apply=False,
                 bad_during_once=False, fail_first_restore=False):
        self.original = original
        self.fail_second_apply = fail_second_apply
        self.bad_during_once = bad_during_once
        self.fail_first_restore = fail_first_restore
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
            else:
                self.restore_count += 1
                if self.fail_first_restore and self.restore_count == 1:
                    self.values[tid] = active.OPT_IN_NS
                    raise active.ActiveTimerSlackError("Injected restore failure")


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

    def test_only_explicit_1us_is_allowed(self):
        for value in (0, 999, 50_000, True, 1_000.0, "1000"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                active.ActiveOutputTimerSlack(value)

    def test_three_distinct_workers_restore_their_own_exact_values(self):
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
            scope.restore()
            self.assertTrue(scope.report["restoration_complete"])
            self.assertEqual(scope.report["status"], "restored")
            self.assertTrue(all(row["before_restore_ns"] == 1_000 and
                                row["after_ns"] == originals[role] and row["restored"]
                                for role, row in rows.items()))
            self.assertTrue(all(backend.values[row["native_tid"]] == originals[role]
                                for role, row in rows.items()))
        self.assertNotIn(main_tid, [call[1] for call in backend.calls])

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
            self.assertEqual(backend.restore_count, 3)
            self.assertEqual(sum(row["restored"] is True for row in scope.report["workers"].values()), 2)
            # A retry still records the earlier failure; it cannot silently
            # turn incomplete first-attempt evidence into a successful run.
            with self.assertRaises(active.ActiveTimerSlackError):
                scope.restore()
            self.assertTrue(all(value == 73_000 for value in backend.values.values()))
            self.assertFalse(scope.report["restoration_complete"])

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
