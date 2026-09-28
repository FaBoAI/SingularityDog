"""Real native clock/cancellation tests with local pipes; no robot devices."""

import importlib.util
import hashlib
import json
import os
from pathlib import Path
import select
import socket
import threading
import time
import tempfile
import types
import unittest
from unittest.mock import patch

from singularitydog_hw import native_diagnostic_transport as native


ROOT = Path(__file__).resolve().parents[1] / 'experiments/native_transport'


class NativeDeadlineWaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('build_native_wait', ROOT / 'build.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.library = native.load_library(module.build())

    def setUp(self):
        self.cancel_read, self.cancel_write = os.pipe()

    def tearDown(self):
        os.close(self.cancel_read)
        os.close(self.cancel_write)

    def test_deadline_reached_with_actual_monotonic_time_and_no_port_action(self):
        host, device = socket.socketpair()
        host.setblocking(False)
        device.setblocking(False)
        try:
            for spin_us in (200, 500):
                before = time.monotonic_ns()
                target = before + 5_000_000
                actual = native.wait_until(self.library, self.cancel_read, target,
                                           spin_us=spin_us)
                after = time.monotonic_ns()
                self.assertGreaterEqual(actual, target)
                self.assertLessEqual(actual, after + 1_000)
                self.assertFalse(select.select([host, device], [], [], 0)[0])
            # A slightly late caller gets the true current time, not target.
            old_target = time.monotonic_ns() - 1_000_000
            self.assertGreater(native.wait_until(self.library, self.cancel_read, old_target),
                               old_target)
        finally:
            host.close()
            device.close()

    def test_cancel_before_and_during_sleep(self):
        os.write(self.cancel_write, b'x')
        with self.assertRaisesRegex(native.WaitError, 'Cancelled'):
            native.wait_until(self.library, self.cancel_read, time.monotonic_ns() + 50_000_000)
        os.read(self.cancel_read, 1)

        def cancel_later():
            time.sleep(0.01)
            os.write(self.cancel_write, b'x')

        writer = threading.Thread(target=cancel_later)
        writer.start()
        try:
            with self.assertRaisesRegex(native.WaitError, 'Cancelled'):
                native.wait_until(self.library, self.cancel_read,
                                  time.monotonic_ns() + 200_000_000, spin_us=500)
        finally:
            writer.join(timeout=1)
        self.assertFalse(writer.is_alive())

    def test_invalid_bounds_and_spin_are_rejected_without_waiting(self):
        now = time.monotonic_ns()
        for target in (0, now + 1_100_000_000, now - 1_100_000_000):
            with self.subTest(target=target), self.assertRaises((ValueError, native.WaitError)):
                native.wait_until(self.library, self.cancel_read, target)
        for spin in (0, 100, 501, True):
            with self.subTest(spin=spin), self.assertRaises(ValueError):
                native.wait_until(self.library, self.cancel_read, now + 10_000_000,
                                  spin_us=spin)
        with self.assertRaises(native.WaitError):
            native.wait_until(self.library, -1, now + 10_000_000)
        with self.assertRaises(ValueError):
            native.wait_until(self.library, self.cancel_read, True)


class OptionalWaitSymbolTests(unittest.TestCase):
    def test_old_verified_exchange_library_loads_until_wait_is_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, binary = root / 'transport.cpp', root / 'libdog_transport.so'
            source.write_text('legacy exchange source', encoding='ascii')
            binary.write_bytes(b'legacy exchange binary')
            (root / 'build-record.json').write_text(json.dumps({
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            }), encoding='ascii')
            def abi(): return 1
            def now(): return time.monotonic_ns()
            def exchange(*_args): return 0
            old_library = types.SimpleNamespace(sd_abi=abi, sd_now_ns=now,
                                                sd_exchange=exchange)
            with patch.object(native.C, 'CDLL', return_value=old_library):
                loaded = native.load_library(binary)
            self.assertIs(loaded, old_library)
            self.assertIsNotNone(loaded.sd_exchange.argtypes)
            with self.assertRaisesRegex(native.WaitError, 'unavailable'):
                native.wait_until(loaded, 0, time.monotonic_ns() + 1_000_000)


if __name__ == '__main__':
    unittest.main()
