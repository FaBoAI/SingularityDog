"""Native dual owners through the real coordinator, on local sockets only."""
from concurrent.futures import Future
import time
import threading
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import test_native_active_phase_pair as fixture
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import native_active_transport as native


class NativePairCoordinatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NativePhasePairTests.setUpClass.__func__(cls)

    def setUp(self):
        fixture.NativePhasePairTests.setUp(self)
        self.pair.close()
        self.workers=runtime.BusWorkers(self.sessions,lambda:None,native_phase_pair=True)
        self.pair=self.workers.native_pair

    def tearDown(self):
        self.workers.close()
        fixture.NativePhasePairTests.tearDown(self)

    device=fixture.NativePhasePairTests.device
    batches=fixture.NativePhasePairTests.batches

    def test_decoded_pair_and_normal_stop_keep_every_bus_record(self):
        self.device('front',12);self.device('rear',12)
        deadline=time.monotonic_ns()+100_000_000
        futures=self.workers.submit_decoded(self.batches(),deadline_ns=deadline,label='policy_output')
        self.assertTrue(all(isinstance(f,Future) for f in futures.values()))
        rows=self.workers.collect_output(futures,deadline_ns=deadline)
        self.assertEqual(set(rows),{'front','rear'})
        for scope in runtime.BUSES:
            result,decoded,last_write,last_receive=rows[scope]
            self.assertEqual(set(decoded),{(i,'feedback') for i in runtime.BUSES[scope]})
            self.assertLess(last_write,last_receive)
            self.assertTrue(all(r.deadline_ns==deadline for r in result[0]))
        self.assertEqual(len(self.workers.journal),2)
        stops=self.workers.finish_stops()
        self.assertTrue(all(set(stops[s]['confirmed_ids'])==set(runtime.BUSES[s]) for s in runtime.BUSES))
        self.assertFalse(self.workers.emergency_errors)

    def test_fault_keeps_partial_raw_and_joins_before_stop(self):
        seen=[0]
        def first_bad(wire):
            seen[0]+=1
            value=fixture.reply(wire)
            return b'XX'+value[2:] if seen[0]==1 else value
        self.device('front',12,first_bad);self.device('rear',12)
        deadline=time.monotonic_ns()+100_000_000
        futures=self.workers.submit_decoded(self.batches(),deadline_ns=deadline,label='policy_output')
        with self.assertRaises(native.ExchangeError):self.workers.collect(futures)
        stops=self.workers.finish_stops()
        self.assertEqual(set(stops),set(runtime.BUSES))
        self.assertTrue(self.workers.aborted.is_set())
        self.assertEqual({scope for scope,_,_,label in self.workers.journal if label=='policy_output'},set(runtime.BUSES))
        self.assertTrue(any(error for _,_,error,_ in self.workers.journal))
        self.assertFalse(any(e['stage']=='native_pair_join' for e in self.workers.emergency_errors))

    def test_native_batch_decodes_acquisition_before_parameter_codec_fallback(self):
        from test_policy_output_runtime import profile
        self.device('front',13,lambda wire:fixture.reply(wire,value=40.))
        self.device('rear',13,lambda wire:fixture.reply(wire,value=40.))
        wires={scope:[native.encode_motion(i,.1,0.,0.) for i in ids]
               for scope,ids in runtime.BUSES.items()}
        deadline=[time.monotonic_ns()+100_000_000]
        with patch.object(runtime,'_fixed_record_frame',wraps=runtime._fixed_record_frame) as legacy:
            feedback,voltage=self.workers.submit_feedback_then_voltage(wires,
                {'front':1,'rear':7},profile(),deadline_ns=deadline)
            feedback_rows=self.workers.collect(feedback)
            voltage_rows=self.workers.collect(voltage)
        # Only the two one-record voltage queries need Python tx/rx frames;
        # the twelve feedback replies use their native six-record slots.
        self.assertEqual(legacy.call_count,4)
        for scope in runtime.BUSES:
            raw,decoded=feedback_rows[scope]
            self.assertEqual(set(decoded),{(i,'feedback') for i in runtime.BUSES[scope]})
            self.assertTrue(all(r.deadline_ns==deadline[0] for r in raw[0]))
            self.assertEqual(voltage_rows[scope][1][runtime.BUSES[scope][0]][0],40.)
        self.assertEqual(len(self.workers.journal),4)
        stops=self.workers.finish_stops()
        self.assertTrue(all(set(stops[s]['confirmed_ids'])==set(runtime.BUSES[s]) for s in runtime.BUSES))

    def test_two_phases_do_not_overwrite_old_journal(self):
        self.device('front',12);self.device('rear',12)
        old=self.workers.exchange(self.batches(),label='first')
        old_start=old['front'][0][0].start_ns
        old_raw=bytes(old['front'][0][0].rx)
        current=self.workers.exchange(self.batches(),label='second')
        self.assertEqual(old_start,old['front'][0][0].start_ns)
        self.assertEqual(old_raw,bytes(old['front'][0][0].rx))
        self.assertGreater(current['front'][0][0].start_ns,old_start)
        self.assertEqual([label for _,_,_,label in self.workers.journal],['first','first','second','second'])

    def test_real_type1_890us_keeps_original_futures_and_has_no_success_relay(self):
        # Recreate only owned handles with the selected 890us/window3 settings;
        # these FDs remain local sockets and cannot address motor hardware.
        self.workers.close()
        for session in self.sessions.values():session.close()
        for scope,first in (('front',1),('rear',7)):
            ids=runtime.BUSES[scope]
            self.sessions[scope]=native.ActiveSession(self.lib,self.hosts[scope].fileno(),
                first_id=first,cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=fixture.BOOT,
                raw_lower_by_id={i:-1. for i in ids},raw_upper_by_id={i:1. for i in ids},
                kp_max_by_id={i:12. for i in ids},kd_max_by_id={i:.25 for i in ids},
                gap_ns=890_000,window=3)
        self.workers=runtime.BusWorkers(self.sessions,lambda:None,native_phase_pair=True)
        self.pair=self.workers.native_pair
        self.device('front',12);self.device('rear',12)
        wires={scope:[native.encode_motion(i,.1,3.,.15) for i in ids] for scope,ids in runtime.BUSES.items()}
        original_submit=self.pair.submit;original_futures={}
        def keep_original(*args,**kwargs):
            result=original_submit(*args,**kwargs);original_futures.update(result);return result
        deadline=time.monotonic_ns()+20_000_000
        self.assertTrue(all(decoder.available for decoder in self.workers.native_feedback_decoders.values()))
        with patch.object(self.pair,'submit',side_effect=keep_original), \
             patch.object(runtime,'_fixed_record_frame',side_effect=AssertionError('Per-record Python frame relay')), \
             patch.object(self.workers.pools['front'],'submit',side_effect=AssertionError('Secondary takeout enqueue')), \
             patch.object(self.workers.pools['rear'],'submit',side_effect=AssertionError('Secondary takeout enqueue')):
            futures=self.workers.submit_decoded(wires,deadline_ns=deadline,label='policy_output')
            rows=self.workers.collect_output(futures,deadline_ns=deadline)
        for scope in runtime.BUSES:
            self.assertIs(futures[scope],original_futures[scope])
            raw,decoded,last_write,last_receive=rows[scope]
            self.assertEqual([bytes(r.tx) for r in raw[0]],wires[scope])
            self.assertEqual([f.kind for f in self.seen[scope]],[1]*6)
            self.assertTrue(all(r.written==r.received==17 and r.deadline_ns==deadline for r in raw[0]))
            self.assertTrue(all(b.start_ns-a.finish_ns>=890_000 for a,b in zip(raw[0],raw[0][1:])))
            self.assertTrue(all(value[0].mode_state==2 and value[0].fault_bits==0 for value in decoded.values()))
            self.assertLess(last_write,last_receive);self.assertLess(last_receive,deadline)
        self.assertEqual(len(self.workers.journal),2)
        stops=self.workers.finish_stops()
        self.assertTrue(all(set(stops[s]['confirmed_ids'])==set(runtime.BUSES[s]) for s in runtime.BUSES))

    def test_transform_failure_keeps_both_raw_and_error_handoff_stops_without_collect(self):
        self.device('front',12);self.device('rear',12)
        original=runtime.decode_records
        def fail_front(result,**kwargs):
            if runtime.codec.ATParser().feed(bytes(result[0][0].tx))[0].destination==1:
                raise ValueError('Synthetic decoded front proof rejected')
            return original(result,**kwargs)
        with patch.object(runtime,'decode_records',side_effect=fail_front):
            futures=self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
                label='policy_output')
            self.assertTrue(self.workers.aborted.wait(.5),'Original failure publication must hand off STOP')
        self.assertIsInstance(futures['front'].exception(timeout=.5),ValueError)
        self.assertIsNone(futures['rear'].exception(timeout=.5))
        self.workers.finish_stops()
        rows=[row for row in self.workers.journal if row[3]=='policy_output']
        self.assertEqual(len(rows),2)
        self.assertEqual({row[0] for row in rows},set(runtime.BUSES))
        self.assertTrue(all(all(r.written==r.received==17 for r in row[1][0]) for row in rows))
        self.assertEqual(next(row[2] for row in rows if row[0]=='front'),'Synthetic decoded front proof rejected')
        self.assertFalse(any(e['stage']=='native_pair_join' for e in self.workers.emergency_errors))

    def test_failure_handoff_enqueue_error_still_propagates_raw_and_collect_stops(self):
        self.device('front',12);self.device('rear',12)
        pool=self.workers.pools['front'];submit=pool.submit
        def fail_handoff(fn,*args,**kwargs):
            if fn==self.workers.emergency:raise RuntimeError('Synthetic failure handoff enqueue lost')
            return submit(fn,*args,**kwargs)
        with patch.object(runtime,'decode_records',side_effect=ValueError('Synthetic decode failure')), \
             patch.object(pool,'submit',side_effect=fail_handoff):
            futures=self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
                label='policy_output')
            with self.assertRaisesRegex(ValueError,'Synthetic decode failure'):self.workers.collect(futures)
            self.workers.finish_stops()
        self.assertEqual(len(self.workers.journal),2)
        self.assertTrue(any(e['stage']=='native_pair_failure_handoff' for e in self.workers.emergency_errors))
        self.assertTrue(all(set(f.result()['confirmed_ids'])==set(runtime.BUSES[s])
                            for s,f in self.workers.stop_futures.items()))

    def test_native_enqueue_then_raise_after_transform_has_no_duplicate_raw_journal(self):
        self.device('front',12);self.device('rear',12)
        transformed=threading.Event();transform=self.workers._transform_native_pair_results
        def signal_transformed(*args,**kwargs):
            result=transform(*args,**kwargs);transformed.set();return result
        pool=self.pair._executor;submit=pool.submit
        def enqueue_then_raise(*args,**kwargs):
            submit(*args,**kwargs)
            self.assertTrue(transformed.wait(.5),'Force publication transform before lost enqueue result')
            raise RuntimeError('Synthetic lost native enqueue after transform')
        with patch.object(self.workers,'_transform_native_pair_results',side_effect=signal_transformed), \
             patch.object(pool,'submit',side_effect=enqueue_then_raise):
            with self.assertRaisesRegex(RuntimeError,'lost native enqueue after transform'):
                self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
                    label='policy_output')
        self.workers.finish_stops()
        self.assertEqual(len(self.workers.journal),2)
        self.assertEqual({r[0] for r in self.workers.journal},set(runtime.BUSES))

    def test_public_futures_cannot_drop_one_bus_by_cancellation(self):
        self.device('front',6);self.device('rear',6)
        futures=self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
            label='policy_output')
        self.assertFalse(futures['front'].cancel())
        self.assertFalse(futures['rear'].cancel())
        self.assertEqual(set(self.workers.collect(futures)),set(runtime.BUSES))

    def test_native_enqueue_then_raise_keeps_written_raw_in_coordinator_journal(self):
        self.device('front',20);self.device('rear',20)
        pool=self.pair._executor;original=pool.submit;lost=[False]
        def lose_after_first_writes(*args,**kwargs):
            value=original(*args,**kwargs)
            if not lost[0]:
                lost[0]=True
                until=time.monotonic()+.5
                while time.monotonic()<until and not all(self.seen[s] for s in runtime.BUSES):time.sleep(.0001)
                self.assertTrue(all(self.seen[s] for s in runtime.BUSES))
                raise RuntimeError('synthetic native Future lost after writes')
            return value
        with patch.object(pool,'submit',side_effect=lose_after_first_writes):
            with self.assertRaisesRegex(RuntimeError,'native Future lost'):
                self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
                    label='policy_output')
        self.workers.finish_stops()
        rows=[row for row in self.workers.journal if row[3]=='policy_output']
        self.assertEqual(len(rows),2)
        self.assertEqual({row[0] for row in rows},set(runtime.BUSES))
        self.assertTrue(all(any(r.written==17 for r in result[0]) for _,result,_,_ in rows))

    def test_submit_fails_before_enqueue_without_raw_keeps_original_error_and_stop(self):
        self.device('front',6);self.device('rear',6)
        error=RuntimeError('synthetic submit failed before enqueue')
        error.native_pair_bus_results=None
        with patch.object(self.pair,'submit',side_effect=error):
            with self.assertRaisesRegex(RuntimeError,'failed before enqueue'):
                self.workers.submit_decoded(self.batches(),deadline_ns=time.monotonic_ns()+100_000_000,
                    label='policy_output')
        self.assertFalse(self.workers.journal)
        stops=self.workers.finish_stops()
        self.assertTrue(all(set(stops[s]['confirmed_ids'])==set(runtime.BUSES[s]) for s in runtime.BUSES))


class NativePairGateTests(unittest.TestCase):
    def test_default_has_no_native_pair(self):
        sessions={scope:SimpleNamespace() for scope in runtime.BUSES}
        with patch.object(native,'ActivePhasePair') as factory:
            workers=runtime.BusWorkers(sessions,lambda:None)
            try:self.assertIsNone(workers.native_pair);factory.assert_not_called()
            finally:workers.close()

    def test_unjoined_native_writer_cannot_race_stop(self):
        stop=Mock();pair=Mock();pair.wait_idle.side_effect=TimeoutError('still writing')
        sessions={scope:SimpleNamespace(emergency_stop=stop) for scope in runtime.BUSES}
        with patch.object(native,'ActivePhasePair',return_value=pair):
            workers=runtime.BusWorkers(sessions,lambda:None,native_phase_pair=True)
            try:
                rows=workers.finish_stops()
                stop.assert_not_called()
                self.assertTrue(all(not r['confirmed_ids'] for r in rows.values()))
                self.assertEqual(workers.emergency_errors[0]['stage'],'native_pair_join')
            finally:workers.close()

    def test_selection_without_review_never_constructs_owner(self):
        from test_policy_output_runtime import profile,FakeIMU,FakeSession
        data=profile();data['native_phase_pair']=True
        with patch.object(native,'ActivePhasePair') as factory:
            with self.assertRaises((RuntimeError,ValueError)):
                runtime.run_supported_policy(data,{'front':FakeSession(1),'rear':FakeSession(7)},
                    FakeIMU(),lambda *_:(0.,)*12,cancel_io=lambda:None,native_phase_pair=True)
            factory.assert_not_called()


if __name__=='__main__':unittest.main()
