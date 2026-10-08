"""Python coordinator placement on per-thread mocks and isolated socket pairs."""
from contextlib import ExitStack
import os
import sys
import threading
import time
import unittest
from unittest.mock import patch

import test_native_active_phase_pair as fixture
from singularitydog_hw import native_active_transport as native
from singularitydog_hw import thread_timer_slack as slack
from singularitydog_hw.native_diagnostic_transport import stop_wire


class ThreadPlacement:
    """Model Linux's calling-thread-only affinity and prctl operations."""
    def __init__(self):
        self.cpus = {}; self.slacks = {}; self.calls = []
        self.fail_apply_slack = False
        self.fail_restore_affinity = False
        self.ignore_apply_affinity = False
        self.restore_started = threading.Event()

    def get_affinity(self, pid):
        assert pid == 0
        tid = threading.get_native_id()
        self.calls.append(('get_affinity', tid))
        return set(self.cpus.setdefault(tid, set(range(8))))

    def set_affinity(self, pid, cpus):
        assert pid == 0
        tid = threading.get_native_id(); cpus = set(cpus)
        self.calls.append(('set_affinity', tid, cpus))
        if cpus == set(range(8)):
            self.restore_started.set()
            if self.fail_restore_affinity:
                raise RuntimeError('synthetic affinity restoration failure')
        if self.ignore_apply_affinity and cpus == {0, 1, 2, 3}:
            return
        self.cpus[tid] = cpus

    def get(self):
        tid = threading.get_native_id()
        self.calls.append(('get_slack', tid))
        return self.slacks.setdefault(tid, 50_000)

    def set(self, value):
        tid = threading.get_native_id()
        self.calls.append(('set_slack', tid, value))
        if self.fail_apply_slack and value == 1000:
            raise RuntimeError('synthetic slack application failure')
        self.slacks[tid] = value


class NativePairCoordinatorSettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NativePhasePairTests.setUpClass.__func__(cls)

    def setUp(self):
        self.placement = ThreadPlacement(); self.stack = ExitStack()
        self.stack.enter_context(patch.object(native.os, 'sched_getaffinity',
                                             self.placement.get_affinity, create=True))
        self.stack.enter_context(patch.object(native.os, 'sched_setaffinity',
                                             self.placement.set_affinity, create=True))
        self.load_prctl = self.stack.enter_context(patch.object(slack, '_load_prctl',
                                                               return_value=self.placement))
        self.stack.enter_context(patch.object(slack, 'require_supported_platform'))
        fixture.NativePhasePairTests.setUp(self)
        self.native_setting_calls = []; self.native_unsupported = False
        self.stack.enter_context(patch.object(self.lib, 'sda_pair_owner_settings',
                                             side_effect=self.native_settings))

    def tearDown(self):
        try: fixture.NativePhasePairTests.tearDown(self)
        finally: self.stack.close()

    device = fixture.NativePhasePairTests.device
    no_write = fixture.NativePhasePairTests.no_write

    def native_settings(self, handle, cpu_mask, timer_slack_ns, restore, rows, error, size):
        self.native_setting_calls.append((cpu_mask, timer_slack_ns, restore))
        for index, row in enumerate(rows):
            row.status = -1 if self.native_unsupported else 0
            if not self.native_unsupported:
                row.native_tid = 1_000_001 + index
                row.cpu_mask = 255 if restore else cpu_mask
                row.timer_slack_ns = 50_000 if restore else timer_slack_ns
                row.original_cpu_mask = 255; row.original_timer_slack_ns = 50_000
                row.configured = 1; row.restored = int(restore)
        error.value = b'Native owner settings unsupported' if self.native_unsupported else b''
        return -1 if self.native_unsupported else 0

    @staticmethod
    def batches():
        return {'front': [stop_wire(1)], 'rear': [stop_wire(7)]}

    def configure(self):
        return self.pair.configure_owners((0, 1, 2, 3), timer_slack_ns=1000)

    def test_default_starts_worker_without_reading_or_changing_linux_settings(self):
        self.assertTrue(self.pair._coordinator_thread.is_alive())
        self.assertIsNone(self.pair.coordinator_settings)
        self.device('front', 1); self.device('rear', 1)
        futures = self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
        for future in futures.values(): self.assertEqual(future.result(timeout=.5)[1].writes, 1)
        self.pair.close()
        self.load_prctl.assert_not_called()
        self.assertEqual(self.placement.calls, [])
        self.assertEqual(self.native_setting_calls, [])
        self.assertEqual(self.pair.coordinator_settings_history, [])

    def test_configures_existing_coordinator_before_phase_and_restores_same_tid(self):
        caller = threading.get_native_id()
        coordinator = self.pair._coordinator_thread.native_id
        self.assertNotEqual(caller, coordinator)
        # The pair's worker already exists when the policy caller moves to CPU4.
        self.placement.cpus[caller] = {4}; self.placement.slacks[caller] = 70_000
        owners = self.configure()
        self.assertEqual(set(owners), {'front', 'rear'})
        row = self.pair.coordinator_settings
        self.assertEqual(row, {'native_tid': coordinator, 'cpu_mask': 15,
            'timer_slack_ns': 1000, 'original_cpu_mask': 255,
            'original_timer_slack_ns': 50_000, 'status': 0,
            'configured': 1, 'restored': 0, 'applied': True})
        row['status'] = -99
        self.assertEqual(self.pair.coordinator_settings['status'], 0)
        exchange = self.lib.sda_pair_exchange
        observed = []
        def verified_exchange(*args):
            observed.append((threading.get_native_id(), self.placement.get_affinity(0),
                             self.placement.get()))
            return exchange(*args)
        self.device('front', 1); self.device('rear', 1)
        with patch.object(self.lib, 'sda_pair_exchange', side_effect=verified_exchange):
            futures = self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
            for future in futures.values(): self.assertEqual(future.result(timeout=.5)[1].writes, 1)
        self.assertEqual(observed, [(coordinator, {0, 1, 2, 3}, 1000)])
        self.pair.close()
        restored = self.pair.coordinator_settings
        self.assertEqual(restored['native_tid'], coordinator)
        self.assertEqual(restored['cpu_mask'], 255)
        self.assertEqual(restored['timer_slack_ns'], 50_000)
        self.assertEqual(restored['status'], 0); self.assertEqual(restored['restored'], 1)
        self.assertFalse(restored['applied'])
        self.assertEqual(self.placement.cpus[caller], {4})
        self.assertEqual(self.placement.slacks[caller], 70_000)
        self.assertEqual([r['restore'] for r in self.pair.coordinator_settings_history], [False, True])
        self.assertTrue(all(call[1] == coordinator for call in self.placement.calls))
        self.assertTrue(all(not session._phase_pair for session in self.sessions.values()))

    def test_partial_application_failure_is_retained_blocks_phases_and_restores(self):
        self.placement.fail_apply_slack = True
        with self.assertRaisesRegex(RuntimeError, 'slack application') as caught:
            self.configure()
        row = self.pair.coordinator_settings
        self.assertEqual(row['status'], -1); self.assertEqual(row['configured'], 1)
        self.assertEqual(row['cpu_mask'], 15); self.assertEqual(row['timer_slack_ns'], 50_000)
        self.assertEqual(caught.exception.native_pair_coordinator_settings, row)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
        self.no_write(); self.pair.close()
        self.assertEqual(self.pair.coordinator_settings['cpu_mask'], 255)
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)
        self.assertTrue(any('slack application' in r['error']
                            for r in self.pair.coordinator_settings_history))

    def test_syscall_success_without_exact_readback_is_a_failure(self):
        self.placement.ignore_apply_affinity = True
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            self.configure()
        self.assertEqual(self.pair.coordinator_settings['cpu_mask'], 255)
        self.assertEqual(self.pair.coordinator_settings['timer_slack_ns'], 1000)
        self.assertEqual(self.pair.coordinator_settings['status'], -1)
        self.no_write(); self.pair.close()
        self.assertEqual(self.pair.coordinator_settings['timer_slack_ns'], 50_000)
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)

    def test_failed_restoration_propagates_and_keeps_partial_readback(self):
        self.configure(); tid = self.pair.coordinator_settings['native_tid']
        self.placement.fail_restore_affinity = True
        with self.assertRaisesRegex(RuntimeError, 'affinity restoration'):
            self.pair.close()
        row = self.pair.coordinator_settings
        self.assertEqual(row['native_tid'], tid)
        self.assertEqual(row['status'], -1); self.assertEqual(row['restored'], 0)
        self.assertEqual(row['cpu_mask'], 15)
        # The other original is attempted even when affinity restoration fails.
        self.assertEqual(row['timer_slack_ns'], 50_000)
        self.assertTrue(self.pair._closed)
        self.assertTrue(all(session._phase_pair is None for session in self.sessions.values()))
        self.assertTrue(any(r['restore'] and 'affinity restoration' in r['error']
                            for r in self.pair.coordinator_settings_history))

    def test_restoration_waits_for_entire_python_coordinator_job(self):
        self.configure(); completed = threading.Event(); release = threading.Event()
        original = self.pair._exchange
        def exchange_then_wait(*args):
            original(*args)
            completed.set()
            if not release.wait(.8): raise AssertionError('Coordinator test release missing')
        failures = []
        def close():
            try: self.pair.close()
            except BaseException as error: failures.append(error)
        self.device('front', 1); self.device('rear', 1)
        with patch.object(self.pair, '_exchange', side_effect=exchange_then_wait):
            futures = self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
            for future in futures.values(): future.result(timeout=.5)
            self.assertTrue(completed.wait(.5))
            closer = threading.Thread(target=close); closer.start()
            try:
                self.assertFalse(self.placement.restore_started.wait(.03))
                self.assertTrue(closer.is_alive())
            finally:
                release.set(); closer.join(timeout=.8)
        self.assertFalse(closer.is_alive()); self.assertEqual(failures, [])
        self.assertTrue(self.placement.restore_started.is_set())
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)

    def test_lost_phase_and_restoration_futures_keep_raw_writes_and_restore(self):
        self.configure()
        self.device('front', 1, lambda _: b''); self.device('rear', 1, lambda _: b'')
        original = self.pair._executor.submit
        def enqueue_then_fail(function, *args, **kwargs):
            original(function, *args, **kwargs)
            if function == self.pair._exchange:
                until = time.monotonic()+.3
                while not all(self.seen.values()) and time.monotonic() < until: time.sleep(.001)
                self.assertTrue(all(self.seen.values()))
            raise RuntimeError('synthetic lost Future after enqueue')
        with patch.object(self.pair._executor, 'submit', side_effect=enqueue_then_fail):
            with self.assertRaisesRegex(RuntimeError, 'lost Future') as caught:
                self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+200_000_000)
        evidence = caught.exception.native_pair_bus_results
        self.assertEqual(set(evidence), {'front', 'rear'})
        for result in evidence.values():
            self.assertIsInstance(result, native.ExchangeError)
            self.assertEqual(result.records[0].written, 17)
            self.assertEqual(result.stats.writes, 1)
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)
        self.assertEqual(self.pair.coordinator_settings['cpu_mask'], 255)
        self.assertFalse(self.pair._coordinator_thread.is_alive())
        self.assertFalse(self.pair._busy.locked()); self.pair.close()

    def test_lost_settings_future_before_apply_still_restores_after_join(self):
        original = self.pair._executor.submit
        release = threading.Event(); entered = threading.Event()
        change = self.pair._change_coordinator_settings
        def delayed_change(*args):
            if not args[2]:
                entered.set()
                if not release.wait(.8): raise AssertionError('Settings test release missing')
            return change(*args)
        def enqueue_then_fail(*args, **kwargs):
            original(*args, **kwargs)
            self.assertTrue(entered.wait(.5))
            raise RuntimeError('synthetic settings Future lost before apply')
        with patch.object(self.pair, '_change_coordinator_settings', side_effect=delayed_change), \
             patch.object(self.pair._executor, 'submit', side_effect=enqueue_then_fail):
            with self.assertRaisesRegex(RuntimeError, 'lost before apply'):
                self.configure()
            self.assertIsNone(self.pair.coordinator_settings)
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
        release.set(); self.pair.close(); self.no_write()
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)
        self.assertEqual(self.pair.coordinator_settings['cpu_mask'], 255)
        self.assertEqual(self.pair.coordinator_settings['timer_slack_ns'], 50_000)
        self.assertTrue(any('lost before apply' in row['error']
                            for row in self.pair.coordinator_settings_history))

    def test_native_unsupported_never_accesses_or_changes_coordinator_settings(self):
        self.native_unsupported = True
        with self.assertRaisesRegex(RuntimeError, 'unsupported'):
            self.configure()
        self.pair.close(); self.no_write()
        self.load_prctl.assert_not_called()
        self.assertEqual(self.placement.calls, [])
        self.assertIsNone(self.pair.coordinator_settings)
        self.assertEqual(self.pair.coordinator_settings_history, [])

    def test_interrupted_join_after_lost_phase_future_retains_inflight_borrow(self):
        self.configure()
        native_entered = threading.Event(); release = threading.Event()
        exchange = self.lib.sda_pair_exchange
        submit = self.pair._executor.submit
        def blocked_exchange(*args):
            native_entered.set()
            if not release.wait(.8): raise AssertionError('Native test release missing')
            return exchange(*args)
        def enqueue_then_fail(function, *args, **kwargs):
            submit(function, *args, **kwargs)
            if function == self.pair._exchange:
                self.assertTrue(native_entered.wait(.5))
                raise RuntimeError('synthetic lost phase Future before join')
            raise RuntimeError('synthetic restoration Future lost before join')
        try:
            with patch.object(self.lib, 'sda_pair_exchange', side_effect=blocked_exchange), \
                 patch.object(self.pair._executor, 'submit', side_effect=enqueue_then_fail), \
                 patch.object(self.pair._executor, 'shutdown',
                              side_effect=KeyboardInterrupt('synthetic interrupted join')), \
                 patch.object(self.lib, 'sda_pair_destroy', wraps=self.lib.sda_pair_destroy) as destroy:
                with self.assertRaisesRegex(RuntimeError, 'lost phase Future') as caught:
                    self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+200_000_000)
                self.assertIsInstance(caught.exception.native_pair_coordinator_restoration_error,
                                      KeyboardInterrupt)
                self.assertFalse(self.pair._idle.is_set())
                self.assertTrue(self.pair._busy.locked())
                self.assertFalse(self.pair._executor_closed)
                with self.assertRaisesRegex(TimeoutError, 'did not join'):
                    self.pair.wait_idle(timeout=.001)
                with self.assertRaisesRegex(KeyboardInterrupt, 'interrupted join'):
                    self.pair.close()
                destroy.assert_not_called()
                self.assertFalse(self.pair._closed)
                self.assertTrue(all(session._phase_pair is self.pair for session in self.sessions.values()))
                with self.assertRaisesRegex(RuntimeError, 'borrowing native pair'):
                    self.sessions['front'].close()
        finally: release.set()
        self.pair.close(); self.no_write()
        self.assertTrue(self.pair._closed); self.assertTrue(self.pair._executor_closed)
        self.assertFalse(self.pair._busy.locked())
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)
        self.assertEqual(set(self.pair.last_completed_bus_results), {'front', 'rear'})

    def test_interrupted_join_before_enqueue_can_be_retried_without_forging_idle(self):
        self.configure()
        with patch.object(self.pair._executor, 'submit',
                          side_effect=RuntimeError('synthetic submit failed before enqueue')), \
             patch.object(self.pair._executor, 'shutdown',
                          side_effect=KeyboardInterrupt('synthetic interrupted empty join')):
            with self.assertRaisesRegex(RuntimeError, 'before enqueue') as caught:
                self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
            self.assertIsNone(caught.exception.native_pair_bus_results)
            self.assertFalse(self.pair._idle.is_set()); self.assertTrue(self.pair._busy.locked())
            with self.assertRaisesRegex(KeyboardInterrupt, 'empty join'): self.pair.close()
            self.assertFalse(self.pair._closed)
        self.pair.close(); self.no_write()
        self.assertTrue(self.pair._idle.is_set()); self.assertFalse(self.pair._busy.locked())
        self.assertEqual(self.pair.coordinator_settings['restored'], 1)

    def test_normal_close_interrupted_join_keeps_handle_until_python_job_finishes(self):
        finished_phase = threading.Event(); release = threading.Event()
        exchange = self.pair._exchange
        def exchange_then_wait(*args):
            exchange(*args); finished_phase.set()
            if not release.wait(.8): raise AssertionError('Coordinator close test release missing')
        self.device('front', 1); self.device('rear', 1)
        try:
            with patch.object(self.pair, '_exchange', side_effect=exchange_then_wait):
                futures = self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)
                for future in futures.values(): future.result(timeout=.5)
                self.assertTrue(finished_phase.wait(.5))
                with patch.object(self.pair._executor, 'shutdown',
                                  side_effect=KeyboardInterrupt('synthetic close join interruption')), \
                     patch.object(self.lib, 'sda_pair_destroy', wraps=self.lib.sda_pair_destroy) as destroy:
                    with self.assertRaisesRegex(KeyboardInterrupt, 'close join'): self.pair.close()
                    destroy.assert_not_called()
                    self.assertFalse(self.pair._closed); self.assertIsNotNone(self.pair._handle)
                    self.assertTrue(all(session._phase_pair is self.pair for session in self.sessions.values()))
        finally: release.set()
        self.pair.close()
        self.assertTrue(self.pair._closed)
        self.assertFalse(self.pair._coordinator_thread.is_alive())


@unittest.skipUnless(sys.platform == 'linux', 'Actual Linux coordinator controls only')
class NativePairCoordinatorLinuxTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NativePhasePairTests.setUpClass.__func__(cls)

    setUp = fixture.NativePhasePairTests.setUp
    tearDown = fixture.NativePhasePairTests.tearDown
    device = fixture.NativePhasePairTests.device

    def test_actual_linux_worker_readbacks_and_socket_phase_leave_caller_unchanged(self):
        allowed = os.sched_getaffinity(0)
        if not allowed or max(allowed) >= 64:
            self.skipTest('Native ABI requires an original affinity mask within CPUs0..63')
        original_slack = slack._load_prctl().get()
        owners = self.pair.configure_owners((min(allowed),))
        row = self.pair.coordinator_settings
        self.assertNotIn(row['native_tid'], [threading.get_native_id()] +
                         [owner['native_tid'] for owner in owners.values()])
        self.assertEqual(os.sched_getaffinity(row['native_tid']), {min(allowed)})
        self.assertEqual(row['timer_slack_ns'], 1000)
        self.assertEqual(os.sched_getaffinity(0), allowed)
        self.assertEqual(slack._load_prctl().get(), original_slack)
        self.device('front', 1); self.device('rear', 1)
        futures = self.pair.submit({'front': [stop_wire(1)], 'rear': [stop_wire(7)]},
                                  deadline_ns=time.monotonic_ns()+100_000_000)
        for future in futures.values(): self.assertEqual(future.result(timeout=.5)[1].writes, 1)
        self.pair.close(); restored = self.pair.coordinator_settings
        self.assertEqual(restored['native_tid'], row['native_tid'])
        self.assertEqual(restored['cpu_mask'], row['original_cpu_mask'])
        self.assertEqual(restored['timer_slack_ns'], row['original_timer_slack_ns'])
        self.assertEqual(restored['restored'], 1)
        self.assertEqual(os.sched_getaffinity(0), allowed)
        self.assertEqual(slack._load_prctl().get(), original_slack)


if __name__ == '__main__': unittest.main()
