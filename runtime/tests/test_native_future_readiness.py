"""Original-Future hints on private pipes; no motor, serial, or network I/O.

The native fixture is built in a private temporary directory, so this module
does not race another test's shared active-transport binary/build record.
"""
from concurrent.futures import Future
import ctypes as C
import gc
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import signal
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import weakref
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native


ROOT = Path(__file__).resolve().parents[1] / 'experiments/native_active_transport'
READY = C.CFUNCTYPE(C.c_int, *native._FUTURE_READINESS_ARGUMENT_TYPES)
ABI = C.CFUNCTYPE(C.c_uint32)


class NativeFutureReadinessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='dog-future-readiness-')
        cls.build_root = Path(cls.temporary.name)
        for name in ('transport.cpp', 'build.py'):
            shutil.copy2(ROOT / name, cls.build_root / name)
        spec = importlib.util.spec_from_file_location('private_future_readiness_build', cls.build_root / 'build.py')
        builder = importlib.util.module_from_spec(spec); spec.loader.exec_module(builder)
        cls.library_path = builder.build()
        cls.lib = native.load_library(cls.library_path)
        cls.source_sha256 = hashlib.sha256((ROOT / 'transport.cpp').read_bytes()).hexdigest()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.pipes = []
        self.threads = []
        self.cancel = self.pipe()
        self.waiter = native.make_owned_waiter(self.lib, self.cancel[0])

    def tearDown(self):
        if self.waiter._readiness_group is not None:
            try: self.waiter._readiness_group.close()
            except native.ActiveWaitError: pass
        for thread in self.threads:
            thread.join(1)
            self.assertFalse(thread.is_alive())
        for endpoints in reversed(self.pipes):
            for fd in endpoints:
                try: os.close(fd)
                except OSError: pass

    def pipe(self):
        endpoints = os.pipe()
        for fd in endpoints: os.set_blocking(fd, False)
        self.pipes.append(endpoints)
        return endpoints

    def worker(self, function):
        thread = threading.Thread(target=function)
        self.threads.append(thread); thread.start()
        return thread

    def raw_wait(self, reader, *, cancel=None, deadline=None):
        actual, error = C.c_uint64(), C.create_string_buffer(256)
        before = time.monotonic_ns()
        if deadline is None: deadline = before + 50_000_000
        status = self.lib.sda_wait_future_ready(self.cancel[0] if cancel is None else cancel,
            reader, deadline, C.byref(actual), error, 256)
        after = time.monotonic_ns()
        return status, actual.value, error.value, before, after

    def proxy(self, **overrides):
        base = self.lib
        class Proxy:
            def __getattr__(self, name):
                if name in overrides:
                    value = overrides[name]
                    if value is None: raise AttributeError(name)
                    return value
                return getattr(base, name)
        return Proxy()

    def test_native_hint_is_actual_early_wake_and_does_not_consume(self):
        hint = self.pipe(); os.write(hint[1], b'x')
        status, actual, error, before, after = self.raw_wait(hint[0])
        self.assertEqual(status, 0); self.assertEqual(error, b'')
        self.assertLessEqual(before, actual); self.assertLessEqual(actual, after)
        self.assertEqual(os.read(hint[0], 1), b'x')

    def test_native_absolute_deadline_and_past_deadline_are_not_backdated(self):
        hint = self.pipe(); deadline = time.monotonic_ns() + 8_000_000
        status, actual, error, before, after = self.raw_wait(hint[0], deadline=deadline)
        self.assertEqual(status, 1); self.assertEqual(error, b'')
        self.assertLessEqual(before, actual); self.assertLessEqual(deadline, actual)
        self.assertLessEqual(actual, after)
        os.write(hint[1], b'x')
        status, actual, error, _, after = self.raw_wait(hint[0], deadline=deadline)
        self.assertEqual(status, 1); self.assertGreaterEqual(actual, deadline)
        self.assertEqual(os.read(hint[0], 1), b'x')

    def test_cancel_has_priority_over_hint_and_expired_deadline(self):
        hint = self.pipe()
        os.write(hint[1], b'x'); os.write(self.cancel[1], b'x')
        for deadline in (time.monotonic_ns() + 50_000_000, time.monotonic_ns() - 1):
            status, actual, error, _, _ = self.raw_wait(hint[0], deadline=deadline)
            self.assertEqual(status, -1); self.assertEqual(actual, 0)
            self.assertIn(b'Cancelled', error)
        self.assertEqual(os.read(hint[0], 1), b'x')
        self.assertEqual(os.read(self.cancel[0], 1), b'x')

    def test_native_wait_releases_gil_and_observes_cancellation_during_wait(self):
        hint = self.pipe()
        def cancel_later():
            time.sleep(.003); os.write(self.cancel[1], b'x')
        self.worker(cancel_later)
        status, actual, error, _, _ = self.raw_wait(hint[0])
        self.assertEqual(status, -1); self.assertEqual(actual, 0)
        self.assertIn(b'Cancelled', error)

    def test_native_eof_is_an_error_without_consuming_a_byte(self):
        hint = self.pipe(); os.close(hint[1])
        status, actual, error, _, _ = self.raw_wait(hint[0])
        self.assertEqual(status, -1); self.assertEqual(actual, 0)
        self.assertIn(b'EOF', error)

    def test_native_refuses_regular_socket_write_endpoint_blocking_and_alias(self):
        hint = self.pipe()
        left, right = socket.socketpair()
        with tempfile.TemporaryFile() as regular, left as sock, right:
            for reader in (regular.fileno(), sock.fileno(), hint[1], -1, 2048, self.cancel[0]):
                with self.subTest(reader=reader):
                    status, actual, error, _, _ = self.raw_wait(reader)
                    self.assertEqual(status, -1); self.assertEqual(actual, 0); self.assertTrue(error)
        os.set_blocking(hint[0], True)
        status, actual, error, _, _ = self.raw_wait(hint[0])
        self.assertEqual(status, -1); self.assertEqual(actual, 0); self.assertTrue(error)

    def test_native_reused_read_fd_during_wait_fails_binding(self):
        hint, replacement = self.pipe(), self.pipe()
        # Keep the originally selected file description live long enough to
        # wake pselect after its descriptor number is deliberately reused.
        held = os.dup(hint[0]); self.addCleanup(os.close, held)
        def reuse():
            time.sleep(.004); os.dup2(replacement[0], hint[0]); os.write(hint[1], b'x')
        self.worker(reuse)
        status, actual, error, _, _ = self.raw_wait(hint[0])
        self.assertEqual(status, -1); self.assertEqual(actual, 0)
        self.assertIn(b'binding', error)

    def test_native_eintr_is_bounded_without_a_new_relative_deadline(self):
        hint = self.pipe(); old_handler = signal.getsignal(signal.SIGALRM)
        old_timer = signal.getitimer(signal.ITIMER_REAL)
        signal.signal(signal.SIGALRM, lambda *args: None)
        try:
            signal.setitimer(signal.ITIMER_REAL, .0005, .0005)
            status, actual, error, _, _ = self.raw_wait(hint[0], deadline=time.monotonic_ns()+100_000_000)
        finally:
            signal.setitimer(signal.ITIMER_REAL, *old_timer)
            signal.signal(signal.SIGALRM, old_handler)
        self.assertEqual(status, -1); self.assertEqual(actual, 0)
        self.assertIn(b'interrupted', error)

    def test_original_already_done_future_callback_is_safe_and_hint_is_not_peer_readiness(self):
        first, second = Future(), Future(); first.set_result('first')
        with self.waiter.readiness_group((first, second)) as group:
            deadline = time.monotonic_ns()+8_000_000
            self.assertEqual(group.wait(deadline)['kind'], 'NOTIFIED')
            self.assertEqual(first.result(), 'first'); self.assertFalse(second.done())
            # The first hint has been drained, so it cannot busy-spin while
            # waiting for the still-pending peer or reset its absolute budget.
            result = group.wait(deadline)
            self.assertEqual(result['kind'], 'DEADLINE')
            self.assertGreaterEqual(result['actual_ns'], deadline)
            self.assertFalse(second.done())

    def test_two_async_original_future_publications_each_wake_without_wrappers(self):
        futures = (Future(), Future())
        with self.waiter.readiness_group(futures) as group:
            self.assertIs(group._futures, futures)
            for future, value in zip(futures, ('front', 'rear')):
                self.worker(lambda f=future, v=value: (time.sleep(.003), f.set_result(v)))
                result = group.wait(time.monotonic_ns()+50_000_000)
                self.assertEqual(result['kind'], 'NOTIFIED')
                self.assertEqual(future.result(), value)

    def test_group_deadline_cancel_and_wrong_future_hints_never_certify_success(self):
        future = Future()
        with self.waiter.readiness_group((future,)) as group:
            group._publish_hint(Future())
            self.assertEqual(group.wait(time.monotonic_ns()+5_000_000)['kind'], 'DEADLINE')
            self.assertFalse(future.done())
            future.set_exception(RuntimeError('original failure'))
            os.write(self.cancel[1], b'x')
            with self.assertRaisesRegex(native.ActiveWaitError, 'Cancelled'):
                group.wait(time.monotonic_ns()+50_000_000)
            with self.assertRaisesRegex(RuntimeError, 'original failure'): future.result()

    def test_legacy_abi_absent_fallback_and_partial_or_malformed_abi_fail_closed(self):
        absent = self.proxy(sda_future_readiness_abi=None, sda_wait_future_ready=None)
        with patch.object(native.C, 'CDLL', return_value=absent):
            legacy = native.load_library(self.library_path)
        owned = native.make_owned_waiter(legacy, self.cancel[0])
        self.assertFalse(owned.future_readiness_available)
        with patch.object(native.os, 'pipe', side_effect=AssertionError('no allocation')):
            self.assertIsNone(owned.readiness_group((Future(),)))
        for overrides in ({'sda_future_readiness_abi': None}, {'sda_wait_future_ready': None},
                          {'sda_future_readiness_abi': ABI(lambda: 2)}):
            proxy = self.proxy(**overrides)
            with self.subTest(overrides=overrides), patch.object(native.C, 'CDLL', return_value=proxy):
                with self.assertRaises(ValueError): native.load_library(self.library_path)
        absent.sda_future_readiness_abi = self.lib.sda_future_readiness_abi
        with self.assertRaisesRegex(ValueError, 'capability changed'): owned.future_readiness_available

    def test_unsupported_future_container_subclass_duplicates_fall_back(self):
        class Derived(Future): pass
        one = Future()
        for value in ([], [one], (), (one, one), (Derived(),), (object(),), (one,)*17):
            with self.subTest(value=value): self.assertIsNone(self.waiter.readiness_group(value))
        self.assertIsNone(self.waiter._readiness_group)

    def test_owner_and_busy_fences_leave_pipe_open_until_wait_finishes(self):
        group = self.waiter.readiness_group((Future(),)); errors = []
        def foreign():
            for action in (lambda: group.wait(time.monotonic_ns()+50_000_000), group.close,
                           lambda: self.waiter.readiness_group((Future(),))):
                try: action()
                except BaseException as error: errors.append(error)
        self.worker(foreign).join(1)
        self.assertEqual(len(errors), 3)
        self.assertTrue(all(isinstance(error, native.ActiveWaitError) for error in errors))
        self.assertTrue(group._active)
        with self.assertRaisesRegex(native.ActiveWaitError, 'Overlapping'):
            self.waiter.readiness_group((Future(),))
        self.waiter._busy.acquire()
        try:
            with self.assertRaisesRegex(native.ActiveWaitError, 'Reentrant'):
                group.wait(time.monotonic_ns()+50_000_000)
        finally: self.waiter._busy.release()
        group.close(); group.close()

    def test_late_callback_does_not_write_after_close_and_fd_reuse(self):
        future = Future(); group = self.waiter.readiness_group((future,))
        old_writer = group._fds[1]; group.close()
        other = self.pipe()
        if other[1] != old_writer:
            os.dup2(other[1], old_writer); self.addCleanup(os.close, old_writer)
        future.set_result('late')
        with self.assertRaises(BlockingIOError): os.read(other[0], 1)
        self.assertEqual(future.result(), 'late')

    def test_reused_endpoint_is_rejected_and_cleanup_preserves_foreign_fd(self):
        future = Future(); group = self.waiter.readiness_group((future,))
        replacement = self.pipe(); old_reader = group._fds[0]
        os.dup2(replacement[0], old_reader)
        with self.assertRaisesRegex(native.ActiveWaitError, 'binding'):
            group.wait(time.monotonic_ns()+50_000_000)
        with self.assertRaises(native.ActiveWaitError): group.close()
        os.write(replacement[1], b'x'); self.assertEqual(os.read(old_reader, 1), b'x')
        os.close(old_reader)

    def test_callback_failure_is_reported_on_wait_and_close_not_on_worker(self):
        future = Future(); group = self.waiter.readiness_group((future,))
        with patch.object(native.os, 'write', side_effect=OSError('injected write failure')):
            future.set_result('still the original result')
        self.assertEqual(future.result(), 'still the original result')
        with self.assertRaisesRegex(native.ActiveWaitError, 'callback failed'):
            group.wait(time.monotonic_ns()+50_000_000)
        with self.assertRaisesRegex(native.ActiveWaitError, 'cleanup/publication failed'): group.close()

    def test_python_drain_eof_and_error_never_claim_original_future_readiness(self):
        for effect in (lambda *args: b'', OSError('injected drain failure')):
            future = Future(); future.set_result('original')
            with self.waiter.readiness_group((future,)) as group:
                with patch.object(native.os, 'read', side_effect=effect):
                    with self.assertRaises(native.ActiveWaitError):
                        group.wait(time.monotonic_ns()+50_000_000)
                self.assertEqual(future.result(), 'original')

    def test_success_with_error_and_semantically_wrong_status_clock_are_rejected(self):
        def contradictory(fd, hint, deadline, actual, error, size):
            actual.contents.value = time.monotonic_ns(); error[0] = b'x'; return 0
        def fake_deadline(fd, hint, deadline, actual, error, size):
            actual.contents.value = time.monotonic_ns(); return 1
        for callback in (contradictory, fake_deadline):
            proxy = self.proxy(sda_wait_future_ready=READY(callback))
            proxy.sda_wait_future_ready.argtypes = list(native._FUTURE_READINESS_ARGUMENT_TYPES)
            proxy.sda_wait_future_ready.restype = C.c_int
            owned = native.make_owned_waiter(proxy, self.cancel[0])
            with owned.readiness_group((Future(),)) as group:
                with self.assertRaisesRegex(native.ActiveWaitError, 'wake/status'):
                    group.wait(time.monotonic_ns()+50_000_000)

    def test_selected_function_abi_signature_and_endpoint_mode_mutations_fail_closed(self):
        function = self.lib.sda_wait_future_ready
        for field, value in (('argtypes', (C.c_int,)), ('restype', C.c_uint64),
                             ('errcheck', lambda result, *args: result)):
            old = getattr(function, field, None)
            try:
                setattr(function, field, value)
                with self.assertRaises(ValueError): self.waiter.future_readiness_available
            finally:
                if old is None and field == 'errcheck': del function.errcheck
                else: setattr(function, field, old)
        for endpoint in (0, 1):
            group = self.waiter.readiness_group((Future(),))
            os.set_blocking(group._fds[endpoint], True)
            with self.assertRaisesRegex(native.ActiveWaitError, 'endpoint/mode'):
                group.wait(time.monotonic_ns()+50_000_000)
            os.set_blocking(group._fds[endpoint], False); group.close()

    def test_gil_held_replacement_signature_and_invalid_status_or_clock_fail_closed(self):
        for callback in (
                lambda fd, hint, deadline, actual, error, size: 3,
                lambda fd, hint, deadline, actual, error, size: 0):
            proxy = self.proxy(sda_wait_future_ready=READY(callback))
            proxy.sda_wait_future_ready.argtypes = list(native._FUTURE_READINESS_ARGUMENT_TYPES)
            proxy.sda_wait_future_ready.restype = C.c_int
            owned = native.make_owned_waiter(proxy, self.cancel[0])
            with owned.readiness_group((Future(),)) as group:
                with self.assertRaises(native.ActiveWaitError): group.wait(time.monotonic_ns()+50_000_000)
        held = C.PYFUNCTYPE(C.c_int, *native._FUTURE_READINESS_ARGUMENT_TYPES)(lambda *args: 0)
        with self.assertRaises(ValueError):
            native.make_owned_waiter(self.proxy(sda_wait_future_ready=held), self.cancel[0])
        proxy = self.proxy(); owned = native.make_owned_waiter(proxy, self.cancel[0])
        proxy.sda_wait_future_ready = READY(lambda *args: 0)
        with self.assertRaisesRegex(ValueError, 'capability changed'): owned.future_readiness_available

    def test_partial_callback_registration_rollback_is_fenced_and_preserves_primary(self):
        first, second = Future(), Future(); entered, release = threading.Event(), threading.Event()
        original_add = Future.add_done_callback
        original_write = os.write
        captured = {}
        def write(fd, data):
            group = self.waiter._readiness_group
            if group is not None and fd == group._fds[1]:
                captured['group'] = group; captured['fds'] = group._fds
                entered.set(); release.wait(1)
                captured['published'] = original_write(fd, data)
                return captured['published']
            return original_write(fd, data)
        def add(future, callback):
            if future is first:
                original_add(future, callback)
                self.worker(lambda: first.set_result('ready'))
                self.assertTrue(entered.wait(1))
            else:
                # Rollback must wait for this active publisher's lock rather
                # than closing endpoints while its publication is in flight.
                self.worker(lambda: (time.sleep(.01), release.set()))
                raise MemoryError('injected registration failure')
        with patch.object(Future, 'add_done_callback', new=add), \
             patch.object(native.os, 'write', new=write):
            with self.assertRaisesRegex(MemoryError, 'injected registration failure'):
                self.waiter.readiness_group((first, second))
        group = captured['group']
        self.assertTrue(group._closed); self.assertFalse(group._active)
        self.assertEqual(group._futures, ()); self.assertIsNone(group._fds)
        self.assertIsNone(self.waiter._readiness_group)
        self.assertEqual(captured['published'], 1)
        for fd in captured['fds']:
            with self.assertRaises(OSError): os.fstat(fd)

    def test_partial_registration_external_fd_reuse_and_context_cleanup_keep_primary(self):
        first, second = Future(), Future(); captured = {}; original_add = Future.add_done_callback
        other = self.pipe()
        def add(future, callback):
            if future is first:
                captured['group'] = self.waiter._readiness_group
                original_add(future, callback)
            else:
                group = captured['group']; captured['reader'] = group._fds[0]
                os.dup2(other[0], captured['reader'])
                raise MemoryError('registration primary')
        with patch.object(Future, 'add_done_callback', new=add):
            with self.assertRaisesRegex(MemoryError, 'registration primary') as caught:
                self.waiter.readiness_group((first, second))
        self.assertTrue(caught.exception.__notes__)
        os.write(other[1], b'x'); self.assertEqual(os.read(captured['reader'], 1), b'x')
        os.close(captured['reader'])
        with self.assertRaisesRegex(ValueError, 'primary') as caught:
            with self.waiter.readiness_group((Future(),)) as group:
                group._failure = RuntimeError('cleanup secondary')
                raise ValueError('primary')
        self.assertTrue(caught.exception.__notes__)

    def test_no_group_future_callback_cycle_or_fd_leak_when_gc_is_deferred(self):
        before = len(os.listdir('/dev/fd')); references = []; retained_futures = []
        enabled = gc.isenabled(); gc.disable()
        try:
            for _ in range(200):
                futures = (Future(), Future())
                group = self.waiter.readiness_group(futures)
                references.append(weakref.ref(group)); retained_futures.extend(futures)
                group.close(); del group
            self.assertTrue(all(reference() is None for reference in references))
            self.assertEqual(len(os.listdir('/dev/fd')), before)
            for future in retained_futures: future.set_result('late')
            self.assertEqual(len(os.listdir('/dev/fd')), before)
        finally:
            if enabled: gc.enable()


if __name__ == '__main__':
    unittest.main()
