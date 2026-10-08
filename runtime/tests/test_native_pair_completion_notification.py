"""Completion hints and original Future publication on local socket buses only."""
from concurrent.futures import Future
import os
import threading
import time
from types import MappingProxyType
import unittest
from unittest.mock import Mock, patch

import test_native_active_phase_pair as fixture
from singularitydog_hw import native_active_transport as native
from singularitydog_hw.native_diagnostic_transport import stop_wire


class NativeCompletionNotificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NativePhasePairTests.setUpClass.__func__(cls)

    setUp = fixture.NativePhasePairTests.setUp
    tearDown = fixture.NativePhasePairTests.tearDown
    device = fixture.NativePhasePairTests.device
    take = fixture.NativePhasePairTests.take

    def phase(self, **kwargs):
        self.device('front', 1); self.device('rear', 1)
        self.latest_deadline = time.monotonic_ns() + 200_000_000
        return self.pair.submit({'front': [stop_wire(1)], 'rear': [stop_wire(7)]},
                               deadline_ns=self.latest_deadline, **kwargs)

    def settle(self, futures):
        rows = self.take(futures); self.pair.wait_published(futures)
        return rows

    def drain(self):
        while True:
            try:
                if not os.read(self.pair._notification_fds[0], 256): break
            except BlockingIOError: break

    def wait(self, futures, delta=5_000_000):
        stamp = time.monotonic_ns()
        return self.pair.wait_completion(futures, tick_ns=stamp + delta,
                                        deadline_ns=stamp + 100_000_000)

    def test_transform_has_stable_immutable_originals_once_and_uses_original_futures(self):
        calls = []
        def transform(raw):
            self.assertIsInstance(raw, MappingProxyType)
            self.assertTrue(self.pair._idle.is_set())
            self.assertTrue(self.pair._busy.locked())
            self.assertTrue(all(not session.busy.locked() for session in self.sessions.values()))
            with self.assertRaises(TypeError): raw['front'] = None
            calls.append(raw)
            return {scope: (row, 'decorated') for scope, row in raw.items()}
        futures = self.phase(result_transform=transform)
        rows = self.settle(futures)
        self.assertEqual(len(calls), 1)
        self.assertIs(self.pair.last_completed_bus_results, calls[0])
        for scope in ('front', 'rear'):
            self.assertIs(futures[scope], self.pair._current_futures[scope])
            self.assertIs(rows[scope][0], calls[0][scope])
            self.assertEqual(rows[scope][1], 'decorated')

    def test_malformed_transform_finishes_both_futures_and_retains_raw(self):
        futures = self.phase(result_transform=lambda raw: {'front': 'missing rear'})
        for future in futures.values():
            with self.assertRaisesRegex(ValueError, 'exact front/rear') as caught:
                future.result(timeout=.5)
            self.assertIs(caught.exception.native_pair_bus_results, self.pair.last_completed_bus_results)
            self.assertEqual(caught.exception.native_pair_bus_results['front'][0][0].received, 17)
        self.pair.wait_published(futures)
        self.assertFalse(self.pair._busy.locked())

    def test_transform_cannot_hide_original_native_bus_errors(self):
        futures = self.pair.submit({'front': [stop_wire(1)], 'rear': [stop_wire(1)]},
            deadline_ns=time.monotonic_ns() + 100_000_000,
            result_transform=lambda raw: {'front': 'success', 'rear': 'success'})
        for scope, future in futures.items():
            with self.assertRaises(native.ExchangeError) as caught: future.result(timeout=.5)
            self.assertIs(caught.exception, self.pair.last_completed_bus_results[scope])
            self.assertEqual(caught.exception.stats.writes, 0)
        self.pair.wait_published(futures)

    def test_native_hint_is_not_ready_and_post_publication_hint_wakes_again(self):
        entered, release = threading.Event(), threading.Event()
        def transform(raw):
            entered.set()
            if not release.wait(.5): raise RuntimeError('Test transformation gate timeout')
            return raw
        futures = self.phase(result_transform=transform)
        try:
            self.assertTrue(entered.wait(.3))
            self.assertTrue(self.pair._idle.is_set())
            self.assertFalse(any(f.done() for f in futures.values()))
            self.assertFalse(self.pair.publication_complete(futures))
            self.assertEqual(self.wait(futures)['kind'], 'NOTIFIED')
            self.assertFalse(any(f.done() for f in futures.values()))
            with self.assertRaisesRegex(RuntimeError, 'in-flight generation'):
                self.pair.submit({'front': [stop_wire(1)], 'rear': [stop_wire(7)]},
                                 deadline_ns=time.monotonic_ns() + 100_000_000)
            release.set(); self.settle(futures)
            self.assertEqual(self.wait(futures)['kind'], 'NOTIFIED')
            self.assertTrue(self.pair.publication_complete(futures))
        finally: release.set()

    def test_hint_and_tick_return_actual_timestamps_without_changing_deadline(self):
        futures = self.phase(); self.settle(futures)
        self.assertTrue(self.pair.completion_notification_available)
        before = time.monotonic_ns(); row = self.wait(futures)
        self.assertEqual(row['kind'], 'NOTIFIED'); self.assertGreaterEqual(row['actual_ns'], before)
        self.drain(); before = time.monotonic_ns(); tick = before + 1_000_000; deadline = before + 50_000_000
        row = self.pair.wait_completion(futures, tick_ns=tick, deadline_ns=deadline)
        self.assertEqual(row['kind'], 'TICK'); self.assertGreaterEqual(row['actual_ns'], tick)
        self.assertLess(row['actual_ns'], deadline)
        self.assertTrue(all(raw[0][0].deadline_ns == self.latest_deadline
                            for raw in self.pair.last_completed_bus_results.values()))

    def test_external_cancel_has_priority_over_already_readable_notification(self):
        futures = self.phase(); self.settle(futures)
        os.write(self.cancel_write, b'!')
        with self.assertRaisesRegex(native.ActiveWaitError, 'Cancelled'):
            self.wait(futures)

    def test_pair_cancel_wakes_a_native_wait(self):
        futures = self.phase(); self.settle(futures); self.drain()
        entered, errors = threading.Event(), []
        def wait():
            entered.set()
            try: self.wait(futures, 50_000_000)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=wait); thread.start()
        self.assertTrue(entered.wait(.1)); self.pair.cancel(); thread.join(.2)
        self.assertFalse(thread.is_alive()); self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], native.ActiveWaitError)
        self.assertIn('Cancelled', str(errors[0]))

    def test_expired_original_hard_deadline_is_not_restarted_for_a_hint(self):
        futures = self.phase(); self.settle(futures)
        expired = time.monotonic_ns() - 1
        with self.assertRaises(native.ActiveWaitError):
            self.pair.wait_completion(futures, tick_ns=expired, deadline_ns=expired)

    def test_foreign_and_old_generation_futures_never_admitted(self):
        old = self.phase(); self.settle(old)
        foreign = {'front': Future(), 'rear': old['rear']}
        self.assertFalse(self.pair.owns_futures(foreign))
        with self.assertRaisesRegex(ValueError, 'Exact current'): self.wait(foreign)
        current = self.phase(); self.settle(current)
        self.assertFalse(self.pair.owns_futures(old))
        with self.assertRaisesRegex(ValueError, 'Exact current'): self.wait(old)
        self.assertTrue(self.pair.owns_futures(current))
        # Undrained old hints do not change the current Future identities/data.
        self.assertEqual(self.wait(current)['kind'], 'NOTIFIED')
        self.assertGreater(self.pair.last_completed_bus_results['front'][0][0].start_ns,
                           old['front'].result()[0][0].start_ns)

    def test_missing_all_optional_symbols_keeps_true_future_fallback(self):
        self.pair.close()
        with patch.multiple(self.lib, **dict.fromkeys(native._NOTIFICATION_SYMBOLS)):
            self.pair = native.ActivePhasePair(self.sessions['front'], self.sessions['rear'])
            self.assertFalse(self.pair.completion_notification_available)
            futures = self.phase(); self.settle(futures)
            with self.assertRaisesRegex(native.ActiveWaitError, 'unavailable'): self.wait(futures)

    def test_partial_optional_abi_and_changed_waiter_fail_closed(self):
        with patch.object(self.lib, 'sda_pair_wait_completion', Mock()):
            futures = self.phase(); self.settle(futures)
            with self.assertRaisesRegex(ValueError, 'Exact GIL-releasing'): self.wait(futures)
        self.pair.close()
        with patch.object(self.lib, 'sda_pair_wait_completion', None):
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                native.ActivePhasePair(self.sessions['front'], self.sessions['rear'])
        self.assertTrue(all(session._phase_pair is None for session in self.sessions.values()))

    def test_future_callback_baseexception_finishes_peer_and_surfaces_publication_error(self):
        entered, release = threading.Event(), threading.Event()
        def transform(raw):
            entered.set(); release.wait(.5); return raw
        futures = self.phase(result_transform=transform)
        try:
            self.assertTrue(entered.wait(.3))
            def interrupted(_): raise KeyboardInterrupt('User callback interrupted publication')
            futures['front'].add_done_callback(interrupted)
            release.set(); self.take(futures)
            with self.assertRaisesRegex(KeyboardInterrupt, 'callback') as caught:
                self.pair.wait_published(futures)
            self.assertIs(caught.exception.native_pair_bus_results, self.pair.last_completed_bus_results)
            with self.assertRaises(KeyboardInterrupt): self.pair.publication_complete(futures)
            self.assertTrue(all(f.done() for f in futures.values()))
            self.assertFalse(self.pair._busy.locked())
        finally: release.set()

    def test_final_hint_failure_is_observable_after_successful_raw_results(self):
        with patch.object(self.pair, '_signal_published_completion', side_effect=OSError('Hint pipe failed')):
            futures = self.phase(); self.take(futures)
            with self.assertRaisesRegex(OSError, 'Hint pipe failed') as caught:
                self.pair.wait_published(futures)
            self.assertIs(caught.exception.native_pair_bus_results, self.pair.last_completed_bus_results)
        self.assertFalse(self.pair._busy.locked())

    def test_close_retains_pipe_until_active_waiter_returns(self):
        futures = self.phase(); self.settle(futures); self.drain()
        descriptors = self.pair._notification_fds
        original, entered, errors = self.pair._notification_waiter, threading.Event(), []
        # Wait through the real GIL-releasing C function. Pair close cancels it
        # and takes the waiter fence before native destroy/descriptor release.
        def wait():
            entered.set()
            try: self.wait(futures, 50_000_000)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=wait); thread.start()
        self.assertTrue(entered.wait(.1))
        until = time.monotonic() + .1
        while not self.pair._notification_wait_lock.locked() and time.monotonic() < until:
            time.sleep(.0001)
        self.assertTrue(self.pair._notification_wait_lock.locked())
        for fd in descriptors: os.fstat(fd)
        self.pair.close(); thread.join(.2)
        self.assertFalse(thread.is_alive()); self.assertTrue(errors)
        self.assertIsInstance(errors[0], native.ActiveWaitError)
        for fd in descriptors:
            with self.assertRaises(OSError): os.fstat(fd)
        self.assertIs(original, self.lib.sda_pair_wait_completion)

    def test_unexpected_closed_writer_fails_before_an_eof_hint(self):
        futures = self.phase(); self.settle(futures); self.drain()
        write_fd = self.pair._notification_fds[1]; retained = os.dup(write_fd)
        try:
            os.close(write_fd)
            with self.assertRaisesRegex(native.ActiveWaitError, 'Invalid bounded|binding changed'):
                self.wait(futures)
        finally:
            os.dup2(retained, write_fd); os.set_blocking(write_fd, False); os.close(retained)

    def test_saturated_hint_pipe_never_blocks_native_or_publication(self):
        while True:
            try: os.write(self.pair._notification_fds[1], b'!' * 4096)
            except BlockingIOError: break
        futures = self.phase(); rows = self.settle(futures)
        self.assertTrue(all(row[0][0].received == 17 for row in rows.values()))
        self.assertTrue(self.pair.publication_complete(futures))
        self.assertEqual(self.wait(futures)['kind'], 'NOTIFIED')


if __name__ == '__main__': unittest.main()
