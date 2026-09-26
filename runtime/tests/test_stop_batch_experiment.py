"""Finite experiment coordination and ownership; fake devices only."""
import contextlib
import io
import json
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import stop_batch_experiment as exp

UIDS = {i: f"{i:016x}" for i in range(1, 13)}
BINDINGS = {s: {"path": s, "resolved": s, "st_rdev": 42} for s in ("front", "rear")}


class FakeBootGuard:
    def __init__(self, events, *, boot_id="boot", fail_check=None, fail_close=False):
        self.boot_id = boot_id
        self.events = events
        self.fail_check, self.fail_close = fail_check, fail_close
        self.checks = self.closes = 0

    def check(self):
        assert not self.closes, "Check after owned boot FD closed"
        self.checks += 1
        self.events.append("boot-check")
        if self.checks == self.fail_check:
            raise OSError("fresh boot read failed")

    def close(self):
        self.closes += 1
        self.events.append("boot-close")
        if self.fail_close:
            raise OSError("boot close failed")


class WrapperTests(unittest.TestCase):
    def run_fake(self, *, stage="both", preflight_fail=None, batch_fail=None,
                 close_failure=None, boot="boot", binding=True, signals=False,
                 guard=None, fail_second_start=False):
        events = [] if guard is None else guard.events
        @contextlib.contextmanager
        def common_lock():
            events.append("common-open")
            try: yield
            finally: events.append("common-close")
        @contextlib.contextmanager
        def port_lock(name):
            events.append("port-open-"+name)
            try: yield
            finally: events.append("port-close-"+name)
        class Raw:
            def __init__(self, **kwargs): self.is_open = False
            def open(self): self.is_open = True; events.append("open-"+self.port)
            def close(self):
                events.append("close-"+self.port)
                if self.port == close_failure: raise OSError("close failed")
                self.is_open = False
            def fileno(self): return 0
        class Preflight:
            def __init__(self, raw, ids, expected, **kwargs):
                assert kwargs["preflight_only"] and kwargs["period_policy"] == "observe-current"
                self.scope = raw.port
                self.tx_log, self.raw_log = [], []
            def run(self):
                events.append("preflight-"+self.scope)
                return {"status": "INCOMPLETE" if self.scope == preflight_fail else "PREFLIGHT_COMPLETE"}
        class Batch:
            def __init__(self, raw, ids, **kwargs):
                self.scope = raw.port
                self.tx_log, self.raw_log = [], []
                assert type(ids) is tuple and len(ids) == 6
            def run(self):
                events.append("batch-"+self.scope)
                return {"status": "INCOMPLETE" if self.scope == batch_fail else "STOP_BATCH_OBSERVATION_COMPLETE"}
        cancelled = threading.Event()
        if signals: cancelled.set()
        def boot_reader():
            events.append("boot-read")
            return boot
        starts = 0
        original_start = threading.Thread.start
        def start(thread):
            nonlocal starts
            starts += 1
            if fail_second_start and starts == 2:
                raise RuntimeError("thread start failed")
            return original_start(thread)
        with patch.object(exp, "ownership_locks", common_lock), \
             patch.object(exp.dual, "port_lock", port_lock), \
             patch.object(exp.dual, "binding_matches", return_value=binding), \
             patch.object(exp.os, "fstat", return_value=SimpleNamespace(st_rdev=42)), \
             patch.object(exp, "BootIdentityGuard", return_value=guard) as factory, \
             patch.object(exp.threading.Thread, "start", start):
            report, _, _ = exp.execute(exp.make_plan(stage, 3), BINDINGS, UIDS, "boot",
                cancelled=cancelled, preflight_factory=Preflight, pipeline_factory=Batch,
                serial_factory=Raw, boot_reader=None if guard is not None else boot_reader)
            if guard is None:
                factory.assert_not_called()
            else:
                factory.assert_called_once_with()
        return report, events

    def test_default_cli_does_not_open_hardware(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), patch.object(exp, "execute") as execute:
            self.assertEqual(exp.main([]), 0)
        execute.assert_not_called()
        plan = json.loads(out.getvalue())
        self.assertEqual(plan["group_size"], 1)
        self.assertEqual(plan["allowed_can_types"], [0, 4, 17])
        self.assertFalse(plan["output_allowed"])

    def test_group_and_scope_plan_reject_invalid_values(self):
        for g in (True, 0, 4, 7, 2.):
            with self.subTest(g=g), self.assertRaises(ValueError): exp.make_plan("front", g)
        with self.assertRaises(ValueError): exp.make_plan("unknown", 1)

    def test_dual_preflight_completes_before_any_batch_and_close_before_unlock(self):
        r, e = self.run_fake()
        self.assertEqual(r["status"], "COMPLETE_STOP_BATCH_EXPERIMENT")
        self.assertEqual(e.count("boot-read"), 9)  # Initial plus four per worker.
        self.assertTrue(r["locks_released"])
        self.assertLess(max(e.index("preflight-"+s) for s in ("front", "rear")),
                        min(e.index("batch-"+s) for s in ("front", "rear")))
        for s in ("front", "rear"):
            self.assertLess(e.index("close-"+s), e.index("port-close-"+s))
            self.assertLess(e.index("port-close-"+s), e.index("common-close"))

    def test_failed_preflight_cancels_both_without_batch(self):
        r, e = self.run_fake(preflight_fail="front")
        self.assertEqual(r["status"], "INCOMPLETE")
        self.assertFalse(any(x.startswith("batch-") for x in e))
        self.assertTrue(r["locks_released"])

    def test_failed_batch_stays_incomplete_and_closes(self):
        r, e = self.run_fake(batch_fail="front")
        self.assertEqual(r["status"], "INCOMPLETE")
        self.assertTrue(r["locks_released"])
        self.assertTrue(all(v["port_closed"] for v in r["results"].values()))

    def test_close_failure_retains_common_and_port_ownership(self):
        before = len(exp._HELD_LOCKS)
        guard = FakeBootGuard([])
        r, e = self.run_fake(stage="front", close_failure="front", guard=guard)
        self.assertEqual(r["status"], "INCOMPLETE")
        self.assertFalse(r["locks_released"])
        self.assertFalse(r["results"]["front"]["port_closed"])
        self.assertNotIn("port-close-front", e)
        self.assertNotIn("common-close", e)
        self.assertEqual(len(exp._HELD_LOCKS), before+2)
        self.assertEqual(guard.closes, 1)
        # Only fake locks are released after assertions; never a device fallback.
        stack, lease = exp._HELD_LOCKS[before:]
        stack.close(); lease.release()
        del exp._HELD_LOCKS[before:]

    def test_boot_or_binding_change_prevents_batch(self):
        with self.assertRaisesRegex(ValueError, "Boot mismatch"):
            self.run_fake(boot="different")
        r, e = self.run_fake(binding=False)
        self.assertEqual(r["status"], "INCOMPLETE")
        self.assertFalse(any(x.startswith("open-") or x.startswith("batch-") for x in e))

    def test_cancelled_before_open_does_not_open(self):
        r, e = self.run_fake(signals=True)
        self.assertEqual(r["status"], "INCOMPLETE")
        self.assertFalse(any(x.startswith("open-") for x in e))

    def test_mutated_plan_rejected_before_locking(self):
        plan = exp.make_plan(); plan["ids_by_scope"]["front"] = [99]
        with patch.object(exp, "ownership_locks") as locks, \
             patch.object(exp, "BootIdentityGuard") as guard, \
             self.assertRaisesRegex(ValueError, "Plan changed"):
            exp.execute(plan, BINDINGS, UIDS, "boot", cancelled=threading.Event())
        locks.assert_not_called()
        guard.assert_not_called()

    def test_one_owned_boot_guard_checks_every_existing_site_and_closes_after_workers(self):
        guard = FakeBootGuard([])
        report, events = self.run_fake(guard=guard)
        self.assertEqual(report["status"], "COMPLETE_STOP_BATCH_EXPERIMENT")
        # Entry, pre-barrier, post-barrier and post-batch per worker. The fake
        # probes perform no inner checks; those callbacks retain the same guard.
        self.assertEqual(guard.checks, 8)
        self.assertEqual(guard.closes, 1)
        for scope in ("front", "rear"):
            self.assertLess(events.index("close-" + scope), events.index("boot-close"))

    def test_owned_boot_mismatch_closes_before_serial_or_locks(self):
        guard = FakeBootGuard([], boot_id="different")
        with self.assertRaisesRegex(ValueError, "Boot mismatch"):
            self.run_fake(guard=guard)
        self.assertEqual(guard.events, ["boot-close"])
        self.assertEqual(guard.closes, 1)

    def test_owned_boot_guard_closes_after_failed_fresh_read(self):
        guard = FakeBootGuard([], fail_check=2)
        report, events = self.run_fake(stage="front", guard=guard)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertIn("fresh boot read failed", report["results"]["front"]["failure"])
        self.assertFalse(any(e.startswith("batch-") for e in events))
        self.assertLess(events.index("close-front"), events.index("boot-close"))
        self.assertEqual(guard.closes, 1)

    def test_owned_boot_guard_closes_on_uid_and_lease_setup_failure(self):
        for bad_uids in (True, False):
            guard = FakeBootGuard([])
            with self.subTest(bad_uids=bad_uids), \
                 patch.object(exp, "BootIdentityGuard", return_value=guard), \
                 patch.object(exp.dual, "CommonLease", side_effect=OSError("lock unavailable")), \
                 self.assertRaises((ValueError, OSError)):
                exp.execute(exp.make_plan(), BINDINGS, {} if bad_uids else UIDS, "boot",
                            cancelled=threading.Event())
            self.assertEqual(guard.closes, 1)

    def test_owned_boot_guard_stays_open_until_started_worker_exits(self):
        guard = FakeBootGuard([])
        report, events = self.run_fake(guard=guard, fail_second_start=True)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertTrue(any("thread start failed" in e for e in report["errors"]))
        self.assertLess(events.index("port-close-front"), events.index("boot-close"))
        self.assertEqual(guard.closes, 1)

    def test_owned_boot_close_failure_is_not_success(self):
        guard = FakeBootGuard([], fail_close=True)
        report, _ = self.run_fake(stage="front", guard=guard)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertTrue(any("Boot monitor close failed" in e for e in report["errors"]))


if __name__ == "__main__":
    unittest.main()
