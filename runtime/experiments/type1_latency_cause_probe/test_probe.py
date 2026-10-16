"""CPU-only ABI, clock, controlled GIL and same-CPU scheduler counter tests.

Artificial pressure validates diagnostic signatures; it is not evidence of the
cause of either original Type1 failure and creates no motor qualification.
"""
import ctypes as C
import os
from pathlib import Path
import statistics
import sys
import tempfile
import threading
import time
import unittest

from . import build as B


class NativeReturnProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.directory=Path(cls.tmp.name).resolve()/'native'
        cls.manifest=B.build(cls.directory)
        cls.library=B.load_library(cls.directory)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def wait(self,wait_ns=2_000_000,spin_ns=0):
        record=B.Record();error=C.create_string_buffer(256)
        status=self.library.cdll.gp_wait(wait_ns,spin_ns,C.byref(record),error,len(error))
        first_python_ns=time.monotonic_ns()  # First Python statement after the real CDLL return.
        after=self.library.snapshot()
        self.assertEqual(status,0,error.value.decode())
        return record,first_python_ns,after

    def test_actual_abi_clocks_and_calling_thread_counters_are_causal(self):
        record,first,post=self.wait()
        self.assertEqual((C.sizeof(B.Snapshot),C.sizeof(B.Record)),(120,296))
        self.assertEqual(record.before.native_tid,threading.get_native_id())
        self.assertEqual(record.after.native_tid,record.before.native_tid)
        self.assertEqual(post.native_tid,record.before.native_tid)
        self.assertLessEqual(record.call_started_ns,record.before.sample_started_ns)
        self.assertLessEqual(record.before.sample_finished_ns,record.wait_started_ns)
        self.assertGreaterEqual(record.wait_finished_ns-record.wait_started_ns,record.wait_requested_ns)
        self.assertLessEqual(record.wait_finished_ns,record.after.sample_started_ns)
        self.assertLessEqual(record.after.sample_finished_ns,record.native_ready_to_return_ns)
        self.assertLessEqual(record.native_ready_to_return_ns,first)
        self.assertLessEqual(first,post.sample_started_ns)
        self.assertLess(post.sample_finished_ns-first,500_000_000)
        self.assertFalse(self.manifest['actual_transport_probe'])
        self.assertFalse(self.manifest['CAN_IO_available'])

    def test_unavailable_counters_are_none_not_measured_zero(self):
        empty=B.Snapshot()
        document=B.snapshot_to_dict(empty)
        for name in ('thread_cpu_ns','user_cpu_ns','minflt','nvcsw','sched_cpu_ns','sched_run_delay_ns','cpu','policy'):
            self.assertIsNone(document[name])
        self.assertTrue(document['zero_unavailable_counters_are_not_measurements'])

    @unittest.skipUnless(sys.platform.startswith('linux'),'Explicit Linux schedstats setting unavailable')
    def test_schedstat_validity_follows_explicit_kernel_enable_setting(self):
        path=Path('/proc/sys/kernel/sched_schedstats')
        if not path.is_file():
            self.skipTest('Explicit kernel schedstats control is unavailable')
        before=int(path.read_text().strip())
        snapshot=self.library.snapshot()
        after=int(path.read_text().strip())
        if before!=after:
            self.skipTest('External kernel setting changed across the snapshot')
        if before==0:
            # Retained runtime/timeslices, if any, cannot turn disabled run-delay
            # accounting into an observed zero. No kernel writes happen here.
            self.assertFalse(snapshot.valid_bits&B.SCHEDSTAT)
            self.assertIsNone(B.snapshot_to_dict(snapshot)['sched_run_delay_ns'])
        elif before==1:
            self.assertTrue(snapshot.valid_bits&B.SCHEDSTAT)
            self.assertIsNotNone(B.snapshot_to_dict(snapshot)['sched_run_delay_ns'])
        else:
            self.fail('Unexpected kernel schedstats enable value')

    def test_invalid_unbounded_or_inconsistent_wait_cannot_run(self):
        for wait,spin in ((0,0),(1_000_000_001,0),(1_000,1_001)):
            with self.subTest(wait=wait,spin=spin):
                record=B.Record();error=C.create_string_buffer(256)
                status=self.library.cdll.gp_wait(wait,spin,C.byref(record),error,len(error))
                self.assertEqual(status,-1);self.assertTrue(error.value)
                self.assertEqual(record.wait_started_ns,0)

    def test_changed_native_bytes_or_receipt_cannot_be_loaded(self):
        with tempfile.TemporaryDirectory() as tmp:
            copy=Path(tmp).resolve()/'copied';copy.mkdir()
            for path in self.directory.iterdir():
                if path.is_file():(copy/path.name).write_bytes(path.read_bytes())
            (copy/self.manifest['binary_name']).write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'artifact changed'):
                B.load_library(copy)
        with self.assertRaisesRegex(ValueError,'Expected CPU-only binary SHA'):
            B.load_library(self.directory,expected_binary_sha256='0'*64)

    def test_controlled_gil_holder_delays_return_without_matching_thread_cpu_work(self):
        baseline=[]
        for _ in range(3):
            record,first,_=self.wait()
            baseline.append(first-record.native_ready_to_return_ns)
        # A Python actor starts only after gp_wait entered natively and released
        # the GIL. A long switch interval is confined to this test and restored.
        old_interval=sys.getswitchinterval();self.library.cdll.gp_reset_entered()
        actor_times=[];errors=[]
        def hold():
            original=None
            try:
                if hasattr(os,'sched_getaffinity'):
                    original=os.sched_getaffinity(0)
                    if len(original)>1:
                        os.sched_setaffinity(0,{max(original)})
                deadline=time.monotonic()+1.
                while not self.library.cdll.gp_entered():
                    if time.monotonic()>deadline:raise RuntimeError('Native entry not observed')
                    time.sleep(.0001)
                actor_times.append(time.monotonic_ns())
                end=time.monotonic_ns()+150_000_000
                while time.monotonic_ns()<end:pass
                actor_times.append(time.monotonic_ns())
            except BaseException as error:errors.append(error)
            finally:
                if original is not None:os.sched_setaffinity(0,original)
        actor=threading.Thread(target=hold,daemon=True)
        try:
            sys.setswitchinterval(.08)
            actor.start();record,first,post=self.wait(5_000_000)
        finally:
            sys.setswitchinterval(old_interval)
            actor.join(timeout=2.)
        self.assertFalse(actor.is_alive());self.assertEqual(errors,[])
        self.assertEqual(len(actor_times),2)
        self.assertLess(actor_times[0],record.native_ready_to_return_ns)
        gap=first-record.native_ready_to_return_ns
        self.assertGreater(gap,20_000_000)
        self.assertGreater(gap,5*min(baseline))
        if record.after.valid_bits&B.THREAD_CPU and post.valid_bits&B.THREAD_CPU:
            # Broad post-sampling CPU window includes native tail and Python
            # setup. Even this upper window has little CPU compared with wall.
            self.assertLess(post.thread_cpu_ns-record.after.thread_cpu_ns,gap//2)
        self.assertEqual(sys.getswitchinterval(),old_interval)

    @unittest.skipUnless(hasattr(os,'sched_setaffinity'),'Same-CPU Linux affinity unavailable')
    def test_same_cpu_native_contention_exposes_runnable_delay_in_its_counter_window(self):
        original=os.sched_getaffinity(0);cpu=min(original);os.sched_setaffinity(0,{cpu})
        try:
            # Do not turn a disabled schedstats zero into a causal claim.
            sample=self.library.snapshot()
            if not sample.valid_bits&B.SCHEDSTAT:
                self.skipTest('Kernel schedstats disabled/unavailable; enable reversibly outside the probe')
            stop=threading.Event();ready=threading.Event();errors=[]
            def busy():
                previous=os.sched_getaffinity(0)
                try:
                    os.sched_setaffinity(0,{cpu});ready.set()
                    while not stop.is_set():
                        record=B.Record();error=C.create_string_buffer(256)
                        if self.library.cdll.gp_wait(80_000_000,80_000_000,C.byref(record),error,len(error)):
                            raise RuntimeError(error.value.decode())
                except BaseException as error:errors.append(error)
                finally:os.sched_setaffinity(0,previous)
            actor=threading.Thread(target=busy,daemon=True);actor.start();self.assertTrue(ready.wait(1.))
            values=[]
            try:
                for _ in range(12):
                    # A sleeping probe can be dispatched immediately whenever
                    # it wakes, giving genuine zero runnable delay despite the
                    # competitor. Sustained native work forces shared-CPU CFS
                    # quanta; retain the same positive-delay assertion below.
                    record,first,post=self.wait(20_000_000,20_000_000)
                    if record.before.valid_bits&B.SCHEDSTAT and post.valid_bits&B.SCHEDSTAT:
                        values.append(post.sched_run_delay_ns-record.before.sched_run_delay_ns)
            finally:
                stop.set();actor.join(timeout=2.)
            self.assertFalse(actor.is_alive());self.assertEqual(errors,[])
            self.assertTrue(values)
            # The full before->post counter window (not exact return-gap wall)
            # records runnable waiting against a genuine native CPU competitor.
            self.assertGreater(max(values),50_000)
        finally:
            os.sched_setaffinity(0,original)
        self.assertEqual(os.sched_getaffinity(0),original)


if __name__=='__main__':unittest.main()
