"""Owned result publication and fault masks on isolated native socket pairs."""
import time
import unittest
from unittest.mock import patch

import test_native_active_phase_pair as fixture
from singularitydog_hw import native_active_transport as native
from singularitydog_hw.native_diagnostic_transport import stop_wire


class NativePairResultPublicationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NativePhasePairTests.setUpClass.__func__(cls)

    setUp = fixture.NativePhasePairTests.setUp
    tearDown = fixture.NativePhasePairTests.tearDown
    device = fixture.NativePhasePairTests.device
    batches = fixture.NativePhasePairTests.batches
    take = fixture.NativePhasePairTests.take
    no_write = fixture.NativePhasePairTests.no_write

    def counting_errors(self):
        allocations = []
        original = native.ExchangeError

        class CountedExchangeError(original):
            def __init__(self, message, records, stats):
                allocations.append((message, records, stats))
                super().__init__(message, records, stats)

        return CountedExchangeError, allocations

    def test_success_publishes_original_results_without_allocating_fallback_errors(self):
        for scope in self.peers: self.device(scope, 6)
        counted, allocations = self.counting_errors()
        deadline = time.monotonic_ns()+100_000_000
        with patch.object(native, 'ExchangeError', counted):
            results = self.take(self.pair.submit(self.batches(), deadline_ns=deadline))
        self.assertEqual(allocations, [])
        for scope, (records, stats) in results.items():
            self.assertIs(self.pair.last_completed_bus_results[scope], results[scope])
            self.assertTrue(all(row.written == row.received == 17 and
                                row.deadline_ns == deadline for row in records))
            self.assertEqual((stats.writes, stats.bytes), (6, 102))
        with self.assertRaises(TypeError): self.pair.last_completed_bus_results['front'] = None

    def test_native_failures_allocate_only_the_two_real_bus_errors(self):
        batches = self.batches(); batches['rear'][0] = stop_wire(1)
        counted, allocations = self.counting_errors()
        with patch.object(native, 'ExchangeError', counted):
            futures = self.pair.submit(batches, deadline_ns=time.monotonic_ns()+100_000_000)
            for scope, future in futures.items():
                with self.assertRaises(counted) as caught: future.result(timeout=.5)
                self.assertIs(caught.exception, self.pair.last_completed_bus_results[scope])
                self.assertEqual(caught.exception.stats.writes, 0)
        self.assertEqual(len(allocations), 2); self.no_write()

    def test_interruption_after_native_return_keeps_both_filled_raw_slots(self):
        for scope in self.peers: self.device(scope, 1)
        original = self.lib.sda_pair_exchange
        counted, allocations = self.counting_errors()

        def completed_then_interrupted(*args):
            original(*args)
            raise RuntimeError('Result normalization interrupted after native completion')

        with patch.object(self.lib, 'sda_pair_exchange', side_effect=completed_then_interrupted),\
             patch.object(native, 'ExchangeError', counted):
            futures = self.pair.submit({'front':[stop_wire(1)], 'rear':[stop_wire(7)]},
                                       deadline_ns=time.monotonic_ns()+100_000_000)
            for scope, future in futures.items():
                with self.assertRaisesRegex(counted, 'normalization interrupted') as caught:
                    future.result(timeout=.5)
                error = caught.exception
                self.assertIs(error, self.pair.last_completed_bus_results[scope])
                self.assertEqual((error.records[0].written, error.records[0].received), (17, 17))
                self.assertEqual(error.stats.writes, 1)
        self.assertEqual(len(allocations), 2)
        self.assertTrue(all(session.poisoned for session in self.sessions.values()))
        self.assertGreater(self.pair.last_phase['generation'], 0)

    def test_interruption_after_first_bus_keeps_success_and_one_raw_fallback(self):
        for scope in self.peers: self.device(scope, 1)
        counted, allocations = self.counting_errors()
        checks = iter((False, RuntimeError('Second bus normalization interrupted')))

        def inspect(_):
            value = next(checks)
            if isinstance(value, BaseException): raise value
            return value

        with patch.object(native, '_stop_reply_has_fault', side_effect=inspect),\
             patch.object(native, 'ExchangeError', counted):
            futures = self.pair.submit({'front':[stop_wire(1)], 'rear':[stop_wire(7)]},
                                       deadline_ns=time.monotonic_ns()+100_000_000)
            front = futures['front'].result(timeout=.5)
            with self.assertRaisesRegex(counted, 'Second bus') as caught:
                futures['rear'].result(timeout=.5)
        self.assertEqual(len(allocations), 1)
        self.assertIs(front, self.pair.last_completed_bus_results['front'])
        self.assertIs(caught.exception, self.pair.last_completed_bus_results['rear'])
        self.assertEqual((front[0][0].received, caught.exception.records[0].received), (17, 17))
        self.assertTrue(all(session.poisoned for session in self.sessions.values()))


class StopFaultHeaderTests(unittest.TestCase):
    def test_all_types_fault_bits_modes_ids_and_receive_lengths_match_previous_decoder(self):
        record = native.Record()
        for communication_type in range(32):
            for fault in range(64):
                for mode in range(3):
                    for mid in (1, 6, 7, 12):
                        tx = ((((communication_type << 24) | (0xfd << 8) | mid) << 3) | 4)
                        rx = ((((2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 0xfd) << 3) | 4)
                        record.tx[:] = b'AT'+tx.to_bytes(4, 'big')+b'\x08'+bytes(8)+b'\r\n'
                        record.rx[:] = b'AT'+rx.to_bytes(4, 'big')+b'\x08'+bytes(8)+b'\r\n'
                        for received in (0, 11, 16, 17):
                            record.received = received
                            previous = bool(received == 17 and
                                (int.from_bytes(bytes(record.tx)[2:6], 'big') >> 27) == 4 and
                                (int.from_bytes(bytes(record.rx)[2:6], 'big') >> 19) & 63)
                            self.assertIs(native._stop_reply_has_fault(record), previous)


if __name__ == '__main__': unittest.main()
