"""Real C++ dual owners on local socket pairs; no robot or network access."""
import importlib.util
import os
from pathlib import Path
import select
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw.can_readonly import ATParser, read_request
from test_native_active_transport import BOOT, ROOT, reply
from singularitydog_hw.native_diagnostic_transport import stop_wire


class NativePhasePairTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('build_phase_pair_native', ROOT/'build.py')
        builder = importlib.util.module_from_spec(spec); spec.loader.exec_module(builder)
        cls.lib = native.load_library(builder.build())

    def setUp(self):
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile()
        self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.hosts = {}; self.peers = {}; self.sessions = {}; self.threads = []; self.errors = []
        self.seen = {scope: [] for scope in ('front', 'rear')}
        for scope, first in (('front', 1), ('rear', 7)):
            host, peer = socket.socketpair(); host.setblocking(False)
            self.hosts[scope] = host; self.peers[scope] = peer
            ids = range(first, first+6)
            self.sessions[scope] = native.ActiveSession(self.lib, host.fileno(), first_id=first,
                cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
                raw_lower_by_id={i:-1. for i in ids}, raw_upper_by_id={i:1. for i in ids},
                kp_max_by_id={i:12. for i in ids}, kd_max_by_id={i:.25 for i in ids},
                gap_ns=900_000, window=3)
        self.pair = native.ActivePhasePair(self.sessions['front'], self.sessions['rear'])

    def tearDown(self):
        self.pair.close()
        for session in self.sessions.values(): session.close()
        for host in self.hosts.values(): host.close()
        for peer in self.peers.values(): peer.close()
        for thread in self.threads: thread.join(timeout=.8)
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()
        if self.errors: raise self.errors[0]

    def device(self, scope, count, mutate=reply):
        def run():
            parser = ATParser(); seen = 0
            try:
                while seen < count:
                    if not select.select([self.peers[scope]], [], [], .5)[0]: return
                    chunk = self.peers[scope].recv(4096)
                    if not chunk: return
                    for frame in parser.feed(chunk):
                        self.seen[scope].append(frame); seen += 1
                        outgoing = mutate(frame.wire)
                        if outgoing: self.peers[scope].sendall(outgoing)
            except OSError: pass
            except BaseException as error: self.errors.append(error)
        thread = threading.Thread(target=run); thread.start(); self.threads.append(thread)

    def batches(self):
        return {'front': [stop_wire(i) for i in range(1,7)],
                'rear': [stop_wire(i) for i in range(7,13)]}

    def take(self, futures):
        return {scope: future.result(timeout=.5) for scope, future in futures.items()}

    def no_write(self):
        for peer in self.peers.values(): self.assertFalse(select.select([peer], [], [], 0)[0])

    def test_two_generations_keep_original_records_and_individual_gaps(self):
        generations = []; outputs = []
        for _ in range(2):
            self.device('front',6); self.device('rear',6)
            outputs.append(self.take(self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+100_000_000)))
            phase = dict(self.pair.last_phase); generations.append(phase['generation'])
            self.assertLessEqual(phase['submitted_ns'], phase['validated_ns'])
            self.assertLessEqual(phase['validated_ns'], phase['released_ns'])
            for started, finished in zip(phase['owner_started_ns'], phase['owner_finished_ns']):
                self.assertLessEqual(phase['released_ns'], started); self.assertLessEqual(started, finished)
            for records, stats in outputs[-1].values():
                self.assertEqual(stats.writes,6); self.assertEqual(stats.bytes,102)
                self.assertEqual([r.received for r in records],[17]*6)
                self.assertTrue(all(b.start_ns-a.finish_ns >= 900_000 for a,b in zip(records,records[1:])))
            for thread in self.threads: thread.join(timeout=.6)
        self.assertGreater(generations[1],generations[0])
        self.assertLess(outputs[0]['front'][0][0].start_ns, outputs[1]['front'][0][0].start_ns)
        self.assertIsNot(outputs[0]['front'][0], outputs[1]['front'][0])

    def test_invalid_rear_frame_prevents_even_front_first_write(self):
        batches = self.batches(); batches['rear'][0] = stop_wire(1)
        futures = self.pair.submit(batches, deadline_ns=time.monotonic_ns()+100_000_000)
        for future in futures.values():
            with self.assertRaises(native.ExchangeError) as error: future.result(timeout=.5)
            self.assertEqual(error.exception.stats.writes,0)
        self.no_write(); self.assertEqual(self.pair.last_phase['released_ns'],0)
        self.assertTrue(all(s.poisoned for s in self.sessions.values()))

    def test_duplicate_type2_key_rejects_complete_phase(self):
        batches=self.batches(); batches['rear']=[stop_wire(7), stop_wire(7)]
        futures=self.pair.submit(batches, deadline_ns=time.monotonic_ns()+100_000_000)
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.no_write()

    def test_absolute_deadline_consumed_by_queue_not_restarted(self):
        self.pair._executor.submit(lambda: time.sleep(.03))
        futures=self.pair.submit(self.batches(), deadline_ns=time.monotonic_ns()+15_000_000)
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.no_write()

    def test_changed_boot_blocks_both_buses_before_write(self):
        self.boot.seek(0); self.boot.write(('f'*36+'\n').encode()); self.boot.flush()
        futures=self.pair.submit(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000)
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.no_write()

    def test_malformed_front_cancels_waiting_rear_and_preserves_raw(self):
        self.device('front',6,lambda wire: b'XX'+reply(wire)[2:])
        self.device('rear',6,lambda wire: b'')
        begin=time.monotonic_ns()
        futures=self.pair.submit(self.batches(),deadline_ns=begin+200_000_000)
        failures={}
        for scope,future in futures.items():
            with self.assertRaises(native.ExchangeError) as failure: future.result(timeout=.5)
            failures[scope]=failure.exception
        self.assertLess(time.monotonic_ns()-begin,100_000_000)
        front=failures['front']
        self.assertGreater(front.stats.rejected_size,0)
        self.assertGreater(front.records[0].written,0)
        self.assertGreater(self.pair.last_phase['cancel_requested_ns'],0)

    def test_missing_reply_is_never_retried_and_stop_ambiguity_survives_pair_close(self):
        self.device('front',1,lambda _: b''); self.device('rear',1,lambda _: b'')
        batches={'front':[stop_wire(1)],'rear':[stop_wire(7)]}
        futures=self.pair.submit(batches,deadline_ns=time.monotonic_ns()+25_000_000)
        for future in futures.values():
            with self.assertRaises(native.ExchangeError) as failure: future.result(timeout=.5)
            self.assertEqual(failure.exception.stats.writes,1)
        for thread in self.threads: thread.join(timeout=.6)
        self.pair.close()
        self.device('front',6); self.device('rear',6)
        results={}
        def stop(scope): results[scope]=self.sessions[scope].emergency_stop()
        stops=[threading.Thread(target=stop,args=(scope,)) for scope in ('front','rear')]
        for thread in stops: thread.start()
        for thread in stops: thread.join(timeout=.6)
        self.assertEqual(results['front']['ambiguous_ids'],[1])
        self.assertEqual(results['rear']['ambiguous_ids'],[7])
        self.assertFalse(results['front']['complete']); self.assertFalse(results['rear']['complete'])

    def test_external_cancel_joins_before_any_session_close(self):
        self.device('front',1,lambda _:b''); self.device('rear',1,lambda _:b'')
        futures=self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                 deadline_ns=time.monotonic_ns()+200_000_000)
        until=time.monotonic()+.2
        while not all(self.seen.values()) and time.monotonic()<until: time.sleep(.001)
        with self.assertRaisesRegex(RuntimeError,'borrowing native pair'): self.sessions['front'].close()
        self.pair.cancel(); self.pair.wait_idle(); self.pair.close()
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)
        self.sessions['front'].close(); self.sessions['rear'].close()

    def test_reentrant_generation_is_rejected_without_queuing_writes(self):
        self.device('front',1,lambda _: b''); self.device('rear',1,lambda _: b'')
        batches={'front':[stop_wire(1)],'rear':[stop_wire(7)]}
        futures=self.pair.submit(batches,deadline_ns=time.monotonic_ns()+100_000_000)
        with self.assertRaisesRegex(RuntimeError,'in-flight generation'):
            self.pair.submit(batches,deadline_ns=time.monotonic_ns()+100_000_000)
        self.pair.cancel(); self.pair.wait_idle()
        for future in futures.values():
            with self.assertRaises(native.ExchangeError): future.result(timeout=.5)

    def test_between_generations_legacy_session_still_works(self):
        self.device('front',1)
        records,_=self.sessions['front'].exchange([read_request(1)])
        self.assertEqual(native.decode_record(records[0])['motor_id'],1)
        self.device('front',1); self.device('rear',1)
        self.take(self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                   deadline_ns=time.monotonic_ns()+100_000_000))

    def test_stopped_fault_is_returned_and_sibling_is_cancelled(self):
        self.device('front',1,lambda wire:reply(wire,fault=1)); self.device('rear',1,lambda _:b'')
        futures=self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                 deadline_ns=time.monotonic_ns()+100_000_000)
        records,_=futures['front'].result(timeout=.5)
        self.assertEqual(native.decode_record(records[0])['fault_bits'],1)
        with self.assertRaises(native.ExchangeError): futures['rear'].result(timeout=.5)
        self.assertTrue(self.sessions['front'].poisoned)

    def test_session_close_and_double_pair_borrow_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'borrowing native pair'): self.sessions['rear'].close()
        with self.assertRaisesRegex(RuntimeError,'borrowed'):
            native.ActivePhasePair(self.sessions['front'],self.sessions['rear'])
        self.pair.close(); self.pair.close(); self.no_write()

    def test_future_is_not_an_independent_motion_cancel_handle(self):
        self.device('front',1); self.device('rear',1)
        futures=self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                 deadline_ns=time.monotonic_ns()+100_000_000)
        self.assertFalse(futures['front'].cancel()); self.take(futures)

    def test_python_invalid_deadline_or_missing_bus_never_submits(self):
        with self.assertRaises(ValueError): self.pair.submit({'front':[stop_wire(1)]},deadline_ns=time.monotonic_ns()+100_000_000)
        with self.assertRaises(ValueError): self.pair.submit(self.batches(),deadline_ns=time.monotonic_ns()-1)
        self.no_write()

    def test_enqueue_then_interruption_joins_lost_future_before_return(self):
        original=self.pair._executor.submit
        def enqueue_then_fail(*args,**kwargs):
            original(*args,**kwargs)
            raise KeyboardInterrupt('Queued phase Future was lost')
        with patch.object(self.pair._executor,'submit',side_effect=enqueue_then_fail):
            with self.assertRaisesRegex(KeyboardInterrupt,'lost'):
                self.pair.submit(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000)
        self.pair.wait_idle()
        self.assertFalse(self.pair._busy.locked())
        self.assertTrue(self.pair._cancelled)
        self.assertTrue(all(not session.busy.locked() for session in self.sessions.values()))
        self.pair.close()

    def test_lost_submit_future_after_write_retains_exact_bus_originals(self):
        self.device('front',1,lambda _:b'');self.device('rear',1,lambda _:b'')
        original=self.pair._executor.submit
        def enqueue_then_fail(*args,**kwargs):
            original(*args,**kwargs)
            until=time.monotonic()+.2
            while not all(self.seen.values()) and time.monotonic()<until:time.sleep(.001)
            if not all(self.seen.values()):raise AssertionError('Synthetic write was not observed')
            raise RuntimeError('Lost Future after real socket writes')
        with patch.object(self.pair._executor,'submit',side_effect=enqueue_then_fail):
            with self.assertRaisesRegex(RuntimeError,'Lost Future') as caught:
                self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                 deadline_ns=time.monotonic_ns()+200_000_000)
        self.pair.wait_idle()
        evidence=caught.exception.native_pair_bus_results
        self.assertEqual(set(evidence),{'front','rear'})
        for error in evidence.values():
            self.assertIsInstance(error,native.ExchangeError)
            self.assertEqual(error.records[0].written,17)
            self.assertEqual(error.stats.writes,1)
            self.assertEqual(error.records[0].received,0)
        with self.assertRaises(TypeError):evidence['front']=None

    def test_python_exception_after_native_return_keeps_filled_output_slots(self):
        self.device('front',1);self.device('rear',1)
        original=self.lib.sda_pair_exchange
        def native_return_then_fail(*args):
            original(*args)
            raise RuntimeError('Interrupted after the native phase populated raw outputs')
        with patch.object(self.lib,'sda_pair_exchange',side_effect=native_return_then_fail):
            futures=self.pair.submit({'front':[stop_wire(1)],'rear':[stop_wire(7)]},
                                     deadline_ns=time.monotonic_ns()+100_000_000)
            for future in futures.values():
                with self.assertRaises(native.ExchangeError) as caught:future.result(timeout=.5)
                self.assertEqual(caught.exception.records[0].written,17)
                self.assertEqual(caught.exception.records[0].received,17)
                self.assertEqual(caught.exception.stats.writes,1)
        self.assertGreater(self.pair.last_phase['generation'],0)

    def test_owner_placement_is_real_readback_or_explicitly_unsupported(self):
        if sys.platform=='darwin':
            with self.assertRaisesRegex(RuntimeError,'unsupported'):
                self.pair.configure_owners((0,1,2,3))
            self.assertFalse(self.pair._settings_applied)
            self.no_write(); return
        allowed=sorted(os.sched_getaffinity(0))
        if not allowed or max(allowed)>=64:
            self.skipTest('64-bit explicit test owner mask unavailable on this host')
        rows=self.pair.configure_owners((allowed[0],))
        self.assertNotEqual(rows['front']['native_tid'],rows['rear']['native_tid'])
        for row in rows.values():
            self.assertEqual(row['cpu_mask'],1<<allowed[0]); self.assertEqual(row['timer_slack_ns'],1000)
        restored=self.pair.restore_owners()
        for scope,row in restored.items():
            self.assertEqual(row['native_tid'],rows[scope]['native_tid'])
            self.assertEqual(row['cpu_mask'],row['original_cpu_mask']); self.assertEqual(row['restored'],1)
        self.no_write()


if __name__=='__main__': unittest.main()
