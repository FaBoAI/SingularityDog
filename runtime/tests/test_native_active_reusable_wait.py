"""Pure synthetic active-wait callbacks; no native library or motor access."""
import ctypes as C
import gc
from types import SimpleNamespace
import threading
import unittest
import weakref
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native


WAIT = C.CFUNCTYPE(C.c_int, C.c_int, C.c_uint64, C.c_uint32,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)


class Library:
    pass


class OwnedActiveWaitTests(unittest.TestCase):
    def fixture(self, callback=None, *, abi=1, spin_us=500):
        calls = []
        def execute(fd, target, spin, actual, error, size):
            calls.append({'fd': fd, 'target': target, 'spin': spin,
                          'actual_address': C.addressof(actual.contents),
                          'error_address': C.cast(error, C.c_void_p).value,
                          'initial_actual': actual.contents.value,
                          'initial_error': C.string_at(error, size), 'size': size})
            if callback is not None:
                return callback(fd, target, spin, actual, error, size)
            actual.contents.value = target+7
            return 0
        library = Library()
        library.sda_wait_until = WAIT(execute)
        library.sda_abi = lambda: abi
        owned = native.make_owned_waiter(library, 23, spin_us=spin_us)
        return owned, library, calls

    def clean(self, owned):
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))

    def test_same_legacy_absolute_deadline_spin_fd_and_actual_return(self):
        for spin in (200, 500):
            with self.subTest(spin=spin):
                owned, library, calls = self.fixture(spin_us=spin)
                self.assertEqual(native.wait_until(library, 23, 1000, spin_us=spin), 1007)
                self.assertEqual(owned(1000), 1007)
                self.assertEqual([(r['fd'], r['target'], r['spin'], r['size'])
                                  for r in calls], [(23, 1000, spin, 256)]*2)
                self.clean(owned)

    def test_one_coordinator_reuses_buffers_without_per_call_ctypes_allocations(self):
        owned, _, calls = self.fixture()
        with patch.object(native.C, 'create_string_buffer', side_effect=AssertionError('new buffer')), \
             patch.object(native.C, 'byref', side_effect=AssertionError('new pointer')):
            for target in (1000, 2000, 3000): self.assertEqual(owned(target), target+7)
        self.assertEqual(len({r['actual_address'] for r in calls}), 1)
        self.assertEqual(len({r['error_address'] for r in calls}), 1)
        self.assertTrue(all(r['initial_actual'] == 0 and r['initial_error'] == bytes(256)
                            for r in calls))
        self.clean(owned)

    def test_library_lifetime_is_retained_without_an_fd_or_library_close(self):
        owned, library, calls = self.fixture()
        held = weakref.ref(library)
        del library
        gc.collect()
        self.assertIsNotNone(held())
        self.assertEqual(owned(1000), 1007)
        self.assertEqual(len(calls), 1)

    def test_nonowner_thread_fails_before_native_and_owner_can_continue(self):
        owned, _, calls = self.fixture()
        errors = []
        def other():
            try: owned(1000)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=other); thread.start(); thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], native.ActiveWaitError)
        self.assertEqual(calls, [])
        self.assertEqual(owned(2000), 2007)
        self.clean(owned)

    def test_reentrancy_rejected_without_corrupting_outer_native_buffers(self):
        holder, errors = {}, []
        def execute(fd, target, spin, actual, error, size):
            actual.contents.value = 777
            error[0] = b'x'
            try: holder['owned'](target+1)
            except BaseException as failure: errors.append(failure)
            self.assertEqual(actual.contents.value, 777)
            self.assertEqual(error[0], b'x')
            error[0] = b'\0'; actual.contents.value = target+7
            return 0
        owned, _, calls = self.fixture(execute); holder['owned'] = owned
        self.assertEqual(owned(1000), 1007)
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], native.ActiveWaitError)
        self.assertEqual(len(calls), 1)
        self.clean(owned)

    def test_invalid_fd_spin_and_deadlines_do_not_reach_native(self):
        owned, library, calls = self.fixture()
        for fd in (True, -1, 2**31, 1.0):
            with self.subTest(fd=fd), self.assertRaises(ValueError):
                native.make_owned_waiter(library, fd)
        for spin in (True, 0, 199, 501, 500.0):
            with self.subTest(spin=spin), self.assertRaises(ValueError):
                native.make_owned_waiter(library, 23, spin_us=spin)
        for target in (True, 0, -1, 2**64, 1.0):
            with self.subTest(target=target), self.assertRaises(ValueError): owned(target)
        self.assertEqual(calls, [])
        self.clean(owned)

    def test_missing_symbol_wrong_abi_and_python_function_rejected_at_setup(self):
        with self.assertRaises(native.ActiveWaitError):
            native.make_owned_waiter(SimpleNamespace(sda_abi=lambda: 1), 23)
        with self.assertRaises(ValueError):
            native.make_owned_waiter(SimpleNamespace(sda_abi=lambda: 1,
                                                     sda_wait_until=lambda *a: 0), 23)
        for abi in (True, 0, 2, 1.0, None):
            with self.subTest(abi=abi), self.assertRaises(ValueError): self.fixture(abi=abi)

    def test_python_api_gil_held_callback_not_accepted_as_cdll_abi(self):
        function = C.PYFUNCTYPE(C.c_int, *native._WAIT_ARGUMENT_TYPES)(lambda *a: 0)
        with self.assertRaises(ValueError):
            native.make_owned_waiter(SimpleNamespace(sda_abi=lambda: 1,
                                                     sda_wait_until=function), 23)

    def test_function_binding_signature_restype_and_errcheck_mutations_rejected(self):
        for kind in ('replace', 'args', 'return', 'errcheck'):
            with self.subTest(kind=kind):
                owned, library, calls = self.fixture()
                if kind == 'replace': library.sda_wait_until = WAIT(lambda *a: 0)
                elif kind == 'args': library.sda_wait_until.argtypes = (C.c_int,)
                elif kind == 'return': library.sda_wait_until.restype = C.c_uint64
                else: library.sda_wait_until.errcheck = lambda result, *args: result
                with self.assertRaises(ValueError): owned(1000)
                self.assertEqual(calls, [])
                self.clean(owned)

    def test_cancellation_status_keeps_original_message_and_clears_scratch(self):
        def cancelled(fd, target, spin, actual, error, size):
            actual.contents.value = target
            C.memmove(error, b'Cancelled\0', 10)
            return 1
        owned, library, calls = self.fixture(cancelled)
        for wait in (lambda: native.wait_until(library, 23, 1000), lambda: owned(1000)):
            with self.assertRaisesRegex(native.ActiveWaitError, '^Cancelled$'): wait()
        self.assertEqual(len(calls), 2)
        self.clean(owned)

    def test_empty_native_error_status_still_fails_and_does_not_retry(self):
        owned, _, calls = self.fixture(lambda *args: 1)
        with self.assertRaisesRegex(native.ActiveWaitError, 'without an error message'): owned(1000)
        self.assertEqual(len(calls), 1)
        self.clean(owned)

    def test_success_with_error_rejected_not_used_as_a_wake(self):
        def invalid(fd, target, spin, actual, error, size):
            actual.contents.value = target
            error[0] = b'x'
            return 0
        owned, _, calls = self.fixture(invalid)
        with self.assertRaisesRegex(native.ActiveWaitError, 'success with an error'): owned(1000)
        self.assertEqual(len(calls), 1)
        self.clean(owned)

    def test_backdated_or_unwritten_wake_rejected_without_reusing_prior_success(self):
        state = {'bad': False}
        def wake(fd, target, spin, actual, error, size):
            if not state['bad']: actual.contents.value = target+7
            return 0
        owned, _, calls = self.fixture(wake)
        self.assertEqual(owned(1000), 1007)
        state['bad'] = True
        with self.assertRaisesRegex(native.ActiveWaitError, 'backdated'): owned(500)
        self.assertEqual(calls[1]['initial_actual'], 0)
        self.clean(owned)

    def test_validation_exception_releases_lock_and_clears_then_owner_can_retry_wait(self):
        owned, _, calls = self.fixture()
        with patch.object(owned, '_verify_function', side_effect=ValueError('binding changed')):
            with self.assertRaisesRegex(ValueError, 'binding changed'): owned(1000)
        self.assertEqual(calls, [])
        self.clean(owned)
        self.assertEqual(owned(2000), 2007)


if __name__ == '__main__':
    unittest.main()
