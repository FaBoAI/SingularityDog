"""Observe native owner wake coalescing on isolated, instrumented local buses.

Counters exist only in this private test-copy translation unit. The shipped
ABI/layout/timestamps and implementation have no instrumentation additions.
"""
import ctypes as C
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest

from singularitydog_hw import native_active_transport as native
from singularitydog_hw.native_diagnostic_transport import stop_wire
import test_native_active_phase_pair as fixture
from test_native_active_transport import reply


ROOT = Path(__file__).resolve().parents[1] / 'experiments/native_active_transport'
CONDITION = 'finished==2||settings_job||job.status||cancelled.load()||closing'
COUNTERS = '''
std::atomic<uint64_t> test_owner_counters[8]{};
void test_owner_completed(uint32_t count) {
    ++test_owner_counters[count==1?0:1];
}
void test_owner_notified(uint32_t count,bool settings,int status,bool cancel,bool closing) {
    ++test_owner_counters[7];
    if(settings)++test_owner_counters[5];
    else if(status||cancel||closing)++test_owner_counters[4];
    else ++test_owner_counters[count==1?2:3];
}
'''
EXPORT = '''
extern "C" uint64_t test_owner_counter(unsigned index) {
    return index<8?test_owner_counters[index].load():0;
}
extern "C" void test_owner_counters_reset() {
    for(auto &counter:test_owner_counters)counter.store(0);
}
'''


class NativePairCompletionCoalescingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='dog-pair-coalesced-')
        root = Path(cls.temporary.name)
        source = (ROOT/'transport.cpp').read_text()
        cls.production_sha256 = hashlib.sha256(source.encode()).hexdigest()
        needle = '++finished;\n            // A normal first owner'
        if source.count(needle) != 1 or source.count('if('+CONDITION+')') != 1:
            raise AssertionError('Exact production completion predicate required')
        # Observe the unchanged native mutex/condition-variable control flow,
        # not a separate Python model of the notification decision.
        instrumented = source.replace('namespace {\n', 'namespace {\n'+COUNTERS, 1)
        instrumented = instrumented.replace(needle,
            '++finished;\n            test_owner_completed(finished);\n'
            '            // A normal first owner', 1)
        notification = 'if('+CONDITION+')\n                completion.notify_all();'
        if instrumented.count(notification) != 1:
            raise AssertionError('Exact production owner notification call required')
        instrumented = instrumented.replace(notification,
            'if('+CONDITION+') {\n'
            '                test_owner_notified(finished,settings_job,job.status,cancelled.load(),closing);\n'
            '                completion.notify_all();\n            }', 1)
        instrumented = instrumented.replace('release.notify_all();completion.notify_all();',
            '++test_owner_counters[6];release.notify_all();completion.notify_all();', 1)
        instrumented += EXPORT
        (root/'transport.cpp').write_text(instrumented)
        shutil.copy2(ROOT/'build.py', root/'build.py')
        spec = importlib.util.spec_from_file_location('private_coalesced_pair_build', root/'build.py')
        builder = importlib.util.module_from_spec(spec); spec.loader.exec_module(builder)
        cls.lib = native.load_library(builder.build())
        cls.lib.test_owner_counter.argtypes = [C.c_uint]; cls.lib.test_owner_counter.restype = C.c_uint64
        cls.lib.test_owner_counters_reset.argtypes = []; cls.lib.test_owner_counters_reset.restype = None
        cls.instrumented_sha256 = hashlib.sha256(instrumented.encode()).hexdigest()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        fixture.NativePhasePairTests.setUp(self)
        self.lib.test_owner_counters_reset()
        self.release = threading.Event()

    def tearDown(self):
        self.release.set()
        fixture.NativePhasePairTests.tearDown(self)

    device = fixture.NativePhasePairTests.device
    no_write = fixture.NativePhasePairTests.no_write

    def counter(self, index): return self.lib.test_owner_counter(index)

    def until(self, predicate):
        end = time.monotonic()+.1
        while not predicate() and time.monotonic()<end: time.sleep(.0005)
        self.assertTrue(predicate())

    def phase(self):
        return self.pair.submit({'front':[stop_wire(1)], 'rear':[stop_wire(7)]},
            deadline_ns=time.monotonic_ns()+200_000_000)

    def take(self, futures):
        rows = {scope:future.result(timeout=.5) for scope,future in futures.items()}
        self.pair.wait_published(futures)
        return rows

    def test_normal_first_owner_has_no_wake_then_second_owner_wakes_once(self):
        def delayed(wire):
            if not self.release.wait(.15): raise AssertionError('Second owner was not released')
            return reply(wire)
        self.device('front', 1); self.device('rear', 1, delayed)
        futures = self.phase()
        self.until(lambda:self.counter(0)==1)
        self.assertEqual(self.counter(1), 0)
        self.assertEqual(self.counter(2), 0)
        self.assertEqual(self.counter(7), 0)
        self.assertTrue(all(not future.done() for future in futures.values()))
        self.release.set(); rows = self.take(futures)
        self.assertEqual(self.counter(1), 1)
        self.assertEqual(self.counter(3), 1)
        self.assertEqual(self.counter(7), 1)
        self.assertEqual(self.counter(6), 0)
        for records, stats in rows.values():
            self.assertEqual(stats.writes, 1); self.assertEqual(stats.bytes, 17)
            self.assertEqual(records[0].received, 17)
            self.assertLess(records[0].received_ns, records[0].deadline_ns)

    def test_second_generation_and_raw_buffers_remain_independent(self):
        generations, records = [], []
        for _ in range(2):
            self.device('front', 1); self.device('rear', 1)
            rows = self.take(self.phase())
            generations.append(self.pair.last_phase['generation'])
            records.append(rows['front'][0])
            for thread in self.threads: thread.join(.3)
        self.assertGreater(generations[1], generations[0])
        self.assertIsNot(records[0], records[1])
        self.assertEqual(self.counter(0), 2); self.assertEqual(self.counter(1), 2)
        self.assertEqual(self.counter(2), 0); self.assertEqual(self.counter(3), 2)
        self.assertEqual(self.counter(7), 2)

    def test_error_keeps_explicit_cancel_and_both_owner_notifications(self):
        self.device('front', 1, lambda wire:b'XX'+reply(wire)[2:])
        self.device('rear', 1, lambda wire:b'')
        futures = self.phase()
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.pair.wait_published(futures)
        self.assertEqual(self.counter(0), 1); self.assertEqual(self.counter(1), 1)
        self.assertEqual(self.counter(4), 2); self.assertEqual(self.counter(7), 2)
        self.assertGreaterEqual(self.counter(6), 1)
        self.assertGreater(self.pair.last_phase['cancel_requested_ns'], 0)

    def test_external_cancel_joins_writers_and_keeps_every_error_wake(self):
        self.device('front', 1, lambda wire:b''); self.device('rear', 1, lambda wire:b'')
        futures = self.phase()
        self.until(lambda:all(self.seen.values()))
        os.write(self.cancel_write, b'x')
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.pair.wait_published(futures); self.pair.wait_idle()
        self.assertEqual(self.counter(4), 2); self.assertEqual(self.counter(7), 2)
        self.assertGreaterEqual(self.counter(6), 1)
        self.assertTrue(all(not session.busy.locked() for session in self.sessions.values()))

    def test_settings_keeps_both_notifications_on_supported_or_unsupported_host(self):
        output = (native.PairOwnerSettings*2)(); error = C.create_string_buffer(256)
        status = self.lib.sda_pair_owner_settings(self.pair._handle, 1, 1000, 0, output, error, 256)
        self.assertIn(status, (0, -1))
        self.assertEqual(self.counter(0), 1); self.assertEqual(self.counter(1), 1)
        self.assertEqual(self.counter(5), 2); self.assertEqual(self.counter(7), 2)
        self.no_write()
        # Always restore before destroying native owners, including partial
        # scheduler application on Linux. macOS explicitly reports unsupported.
        restored = self.lib.sda_pair_owner_settings(self.pair._handle, 0, 0, 1, output, error, 256)
        self.assertIn(restored, (0, -1))
        self.assertEqual(self.counter(5), 4); self.assertEqual(self.counter(7), 4)

    def test_idle_close_preserves_explicit_cancellation_notification(self):
        self.pair.close()
        self.assertGreaterEqual(self.counter(6), 1)
        self.assertEqual(self.counter(0), 0); self.assertEqual(self.counter(1), 0)
        self.no_write()


if __name__=='__main__':unittest.main()
