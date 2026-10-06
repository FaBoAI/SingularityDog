"""Owned scratch wait ABI/causality tests; fake C callbacks, no devices."""
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import native_diagnostic_transport as native


WAIT = C.CFUNCTYPE(C.c_int, C.c_int, C.c_uint64, C.c_uint32,
    C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)


class OwnedWaitTests(unittest.TestCase):
    def fixture(self, callback=None, *, abi=1, spin_us=200):
        calls = []

        def execute(fd, deadline, spin, actual, error, size):
            calls.append({'fd': fd, 'deadline': deadline, 'spin': spin,
                          'actual_address': C.addressof(actual.contents),
                          'error_address': C.cast(error, C.c_void_p).value,
                          'initial_actual': actual.contents.value,
                          'initial_error': C.string_at(error, size), 'size': size})
            if callback is not None:
                return callback(fd, deadline, spin, actual, error, size)
            actual.contents.value = deadline + 7
            return 0

        library = SimpleNamespace(sd_wait_until=WAIT(execute), sd_abi=lambda: abi)
        owned = native.make_owned_waiter(library, 8, spin_us=spin_us)
        return owned, library, calls

    def assert_clean(self, owned):
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))

    def test_exact_deadline_fd_spin_and_actual_wake_preserved(self):
        owned, _, calls = self.fixture(spin_us=500)
        self.assertEqual(owned(123456), 123463)
        self.assertEqual([(r['fd'], r['deadline'], r['spin'], r['size']) for r in calls],
                         [(8, 123456, 500, 256)])
        self.assert_clean(owned)

    def test_each_call_reuses_same_owned_buffer_addresses(self):
        owned, _, calls = self.fixture()
        for deadline in (100, 200, 300):
            self.assertEqual(owned(deadline), deadline + 7)
        self.assertEqual(len({r['actual_address'] for r in calls}), 1)
        self.assertEqual(len({r['error_address'] for r in calls}), 1)
        self.assertTrue(all(r['initial_actual'] == 0 and
                            r['initial_error'] == bytes(256) for r in calls))

    def test_no_per_call_output_or_error_buffer_constructor(self):
        owned, _, _ = self.fixture()
        with patch.object(native.C, 'create_string_buffer', side_effect=AssertionError('allocated')):
            self.assertEqual(owned(100), 107)
            self.assertEqual(owned(200), 207)

    def test_wrong_thread_rejected_before_native_or_buffer_access(self):
        owned, _, calls = self.fixture()
        failures = []
        thread = threading.Thread(target=lambda: self.record_failure(failures, owned, 100))
        thread.start(); thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], native.WaitError)
        self.assertIn('another thread', str(failures[0]))
        self.assertEqual(calls, [])
        self.assert_clean(owned)
        self.assertEqual(owned(100), 107)

    @staticmethod
    def record_failure(failures, function, *args):
        try:
            function(*args)
        except BaseException as error:
            failures.append(error)

    def test_reentrant_call_rejected_without_resetting_inflight_buffers(self):
        holder, failures = {}, []

        def execute(fd, deadline, spin, actual, error, size):
            actual.contents.value = 777
            error[0] = b'x'
            self.record_failure(failures, holder['owned'], deadline + 1)
            self.assertEqual(actual.contents.value, 777)
            self.assertEqual(error[0], b'x')
            error[0] = b'\0'; actual.contents.value = deadline
            return 0

        owned, _, calls = self.fixture(execute); holder['owned'] = owned
        self.assertEqual(owned(100), 100)
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(failures), 1)
        self.assertIn('Reentrant', str(failures[0]))
        self.assert_clean(owned)

    def test_bad_deadlines_rejected_without_native_call(self):
        owned, _, calls = self.fixture()
        for deadline in (True, False, None, 0, -1, 2**64, 1., '100'):
            with self.subTest(deadline=deadline), self.assertRaises(ValueError):
                owned(deadline)
        self.assertEqual(calls, [])
        self.assert_clean(owned)

    def test_bad_factory_fd_or_spin_rejected(self):
        _, library, calls = self.fixture()
        for fd in (True, False, None, -1, 2**31, 1., '8'):
            with self.subTest(fd=fd), self.assertRaises(ValueError):
                native.make_owned_waiter(library, fd)
        for spin in (True, None, 0, 100, 201, 500., '500'):
            with self.subTest(spin=spin), self.assertRaises(ValueError):
                native.make_owned_waiter(library, 8, spin_us=spin)
        self.assertEqual(calls, [])

    def test_missing_optional_native_waiter_rejected(self):
        with self.assertRaises(native.WaitError):
            native.make_owned_waiter(SimpleNamespace(sd_abi=lambda: 1), 8)

    def test_wrong_native_abi_rejected_before_wait(self):
        _, library, calls = self.fixture()
        for abi in (0, 2, None, '1', True, 1.):
            library.sd_abi = lambda value=abi: value
            with self.subTest(abi=abi), self.assertRaises(ValueError):
                native.make_owned_waiter(library, 8)
        self.assertEqual(calls, [])

    def test_python_callable_not_native_prototype_rejected(self):
        with self.assertRaises(ValueError):
            native.make_owned_waiter(SimpleNamespace(sd_wait_until=lambda *a: 0,
                                                     sd_abi=lambda: 1), 8)

    def test_gil_retaining_prototype_rejected(self):
        prototype = C.PYFUNCTYPE(C.c_int, *native._WAIT_ARGUMENT_TYPES)
        function = prototype(lambda *args: 0)
        with self.assertRaises(ValueError):
            native.make_owned_waiter(SimpleNamespace(sd_wait_until=function,
                                                     sd_abi=lambda: 1), 8)

    def test_prototype_or_function_mutation_rejected_before_native(self):
        for mutation in ('arguments', 'result', 'replacement', 'error_handler'):
            with self.subTest(mutation=mutation):
                owned, library, calls = self.fixture()
                if mutation == 'arguments': library.sd_wait_until.argtypes = [C.c_int]
                elif mutation == 'result': library.sd_wait_until.restype = C.c_uint64
                elif mutation == 'error_handler': library.sd_wait_until.errcheck = lambda *args: 0
                else: library.sd_wait_until = WAIT(lambda *args: 0)
                with self.assertRaises(ValueError): owned(100)
                self.assertEqual(calls, [])
                self.assert_clean(owned)

    def test_cancel_error_is_preserved_and_scratch_is_cleared(self):
        def cancel(fd, deadline, spin, actual, error, size):
            actual.contents.value = deadline + 99
            message = b'Cancelled during diagnostic wait\0'
            C.memmove(error, message, len(message))
            return -1
        owned, _, _ = self.fixture(cancel)
        with self.assertRaisesRegex(native.WaitError, 'Cancelled during diagnostic wait'):
            owned(100)
        self.assert_clean(owned)

    def test_partial_failed_actual_cannot_be_reused_on_next_success(self):
        count = [0]
        def execute(fd, deadline, spin, actual, error, size):
            count[0] += 1
            if count[0] == 1:
                actual.contents.value = 100000
                error[0] = b'e'
                return -1
            if count[0] == 3: actual.contents.value = deadline
            return 0
        owned, _, calls = self.fixture(execute)
        with self.assertRaises(native.WaitError): owned(100)
        with self.assertRaisesRegex(native.WaitError, 'backdated'): owned(200)
        self.assertEqual(owned(300), 300)
        self.assertTrue(all(r['initial_actual'] == 0 and
                            r['initial_error'] == bytes(256) for r in calls))
        self.assert_clean(owned)

    def test_success_with_error_or_backdated_actual_is_rejected(self):
        for defect in ('error', 'backdated', 'zero'):
            def execute(fd, deadline, spin, actual, error, size):
                actual.contents.value = deadline if defect == 'error' else deadline-1 if defect == 'backdated' else 0
                if defect == 'error': error[0] = b'x'
                return 0
            with self.subTest(defect=defect):
                owned, _, _ = self.fixture(execute)
                with self.assertRaises(native.WaitError): owned(100)
                self.assert_clean(owned)

    def test_native_failure_without_message_has_explicit_error(self):
        owned, _, _ = self.fixture(lambda *args: -1)
        with self.assertRaisesRegex(native.WaitError, 'without an error message'):
            owned(100)
        self.assert_clean(owned)

    def test_non_utf8_error_does_not_hide_native_failure(self):
        def execute(fd, deadline, spin, actual, error, size):
            error[0] = b'\xff'
            return 1
        owned, _, _ = self.fixture(execute)
        with self.assertRaises(native.WaitError): owned(100)
        self.assert_clean(owned)

    def test_existing_wait_until_still_allocates_independent_buffers(self):
        _, library, _ = self.fixture()
        with patch.object(native.C, 'create_string_buffer', wraps=C.create_string_buffer) as allocate:
            self.assertEqual(native.wait_until(library, 8, 100), 107)
            self.assertEqual(native.wait_until(library, 8, 200), 207)
        self.assertEqual(allocate.call_count, 2)


@unittest.skipUnless(shutil.which('c++'), 'Portable diagnostic C++ compiler unavailable')
class RealOwnedWaitTests(unittest.TestCase):
    """Same C routine with only a local cancellation pipe; no serial/IMU FD."""
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='dog-owned-wait-test-')
        cls.addClassCleanup(cls.directory.cleanup)
        root = Path(cls.directory.name)
        source = root/'transport.cpp'
        source.write_bytes((Path(__file__).resolve().parents[1]/
                            'experiments/native_transport/transport.cpp').read_bytes())
        binary = root/'libdog_transport.so'
        subprocess.run(['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-fPIC', '-shared', str(source), '-o', str(binary)], check=True)
        (root/'build-record.json').write_text(json.dumps({
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}))
        cls.library = native.load_library(binary)

    def setUp(self):
        self.read_fd, self.write_fd = os.pipe()
        self.addCleanup(os.close, self.write_fd)
        self.addCleanup(os.close, self.read_fd)

    def test_real_native_absolute_wake_and_caller_fd_ownership(self):
        owned = native.make_owned_waiter(self.library, self.read_fd, spin_us=500)
        target = time.monotonic_ns()+300_000
        actual = owned(target)
        self.assertGreaterEqual(actual, target)
        self.assertLessEqual(actual, time.monotonic_ns())
        os.fstat(self.read_fd)  # Factory/call never closes the caller's FD.
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))

    def test_real_native_cancel_before_wait_clears_actual_and_error(self):
        owned = native.make_owned_waiter(self.library, self.read_fd)
        os.write(self.write_fd, b'x')
        with self.assertRaisesRegex(native.WaitError, 'Cancelled'):
            owned(time.monotonic_ns()+300_000)
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))

    def test_real_native_cancel_during_sleep_is_not_hidden_by_reuse(self):
        owned = native.make_owned_waiter(self.library, self.read_fd)
        sender = threading.Thread(target=lambda: (time.sleep(.002), os.write(self.write_fd, b'x')))
        sender.start()
        try:
            with self.assertRaisesRegex(native.WaitError, 'Cancelled'):
                owned(time.monotonic_ns()+50_000_000)
        finally:
            sender.join(1)
        self.assertFalse(sender.is_alive())
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))

    def test_real_native_oversized_wait_preserves_original_validation(self):
        owned = native.make_owned_waiter(self.library, self.read_fd)
        with self.assertRaisesRegex(native.WaitError, 'Invalid bounded diagnostic wait arguments'):
            owned(time.monotonic_ns()+2_000_000_000)
        self.assertEqual(owned._actual.value, 0)
        self.assertEqual(owned._error.raw, bytes(256))


if __name__ == '__main__':
    unittest.main()
