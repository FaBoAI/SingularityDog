import copy
from array import array
import contextlib
import gc
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from contextlib import ExitStack
import struct
import io
import threading
import time
import unittest
import tempfile
import json
from pathlib import Path
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw import thread_timer_slack as slack
from test_thread_timer_slack import FakePrctl
from test_policy_observer import make, snapshot


class Device:
    def __init__(self,barrier=None):self.barrier=barrier;self.reuse=None
    def read_sample(self):
        if self.barrier:self.barrier.wait(timeout=1)
        if self.reuse:return self.reuse
        a=time.monotonic_ns()
        return {'frame':'sensor','accel_m_s2':[0.,0.,-9.81],'gyro_rad_s':[0.,0.,0.],
                'read_started_monotonic_ns':a,'read_finished_monotonic_ns':time.monotonic_ns()}


class Session:
    def __init__(self,barrier=None):self.calls=0;self.barrier=barrier
    def exchange(self,wires):
        self.calls+=1
        if self.barrier:self.barrier.wait(timeout=1)
        rows=(native.Record*len(wires))()
        for r,wire in zip(rows,wires):
            f=ATParser().feed(wire)[0];mid=f.destination
            data=(struct.pack('>4H',32768,32768,32768,250) if f.kind==4 else
                  f.data[:4]+struct.pack('<f',.2))
            can_id=((2 if f.kind==4 else 17)<<24)|mid<<8|0xfd
            rx=b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
            r.tx[:]=wire;r.rx[:]=rx;r.written=r.received=17
            r.start_ns=time.monotonic_ns();r.finish_ns=r.start_ns+1
            r.read_start_ns=r.finish_ns;r.received_ns=r.finish_ns+1;r.deadline_ns=r.start_ns+100000000
        stats=native.Stats();stats.writes=len(wires)
        return rows,stats


class Observer:
    def __init__(self):self.calls=0;self.invalid=False
    def arm_run(self,tick):self.first=tick
    def consume(self,s):self.calls+=1;return {'q_target_rad_diagnostic_only':[.1]*12}
    def invalidate(self,error):self.invalid=True
    def finish(self):return {'calls':self.calls,'invalid':self.invalid}


class StartupTrackingPool(ThreadPoolExecutor):
    """Real workers with controlled startup delay, capacity, or submit failure."""
    def __init__(self,*,max_workers,thread_name_prefix,startup_delay=0.,worker_limit=None,fail_submit_at=None):
        self.startup_delay=startup_delay;self.fail_submit_at=fail_submit_at
        self.submitted=0;self.startup_futures=[];self.shutdown_calls=[]
        self.startup_threads={};self.startup_finished={};self.track_lock=threading.Lock()
        self.started=[threading.Event() for _ in range(3)]
        super().__init__(max_workers=worker_limit or max_workers,thread_name_prefix=thread_name_prefix)

    def submit(self,fn,*args,**kwargs):
        self.submitted+=1;number=self.submitted
        if number==self.fail_submit_at:
            for event in self.started[:number-1]:
                if not event.wait(timeout=.25):raise RuntimeError('Fixture worker did not start')
            raise RuntimeError('Injected startup submit failure')
        if number<=3:
            def tracked_startup():
                with self.track_lock:self.startup_threads[number]=threading.get_ident()
                self.started[number-1].set()
                try:
                    if self.startup_delay:time.sleep(self.startup_delay)
                    return fn(*args,**kwargs)
                finally:
                    with self.track_lock:self.startup_finished[number]=time.monotonic_ns()
            future=super().submit(tracked_startup)
            self.startup_futures.append(future)
            return future
        return super().submit(fn,*args,**kwargs)

    def shutdown(self,wait=True,*,cancel_futures=False):
        self.shutdown_calls.append((wait,cancel_futures))
        return super().shutdown(wait=wait,cancel_futures=cancel_futures)


class NativePipelineTests(unittest.TestCase):
    def test_reused_input_prime_rejects_unowned_or_missing_buffers(self):
        buffers=tuple(array('f',[0.]*n) for n in (3,3,3,12,12,12))
        tensors=tuple(Mock(shape=(1,len(buf)),
                           data_ptr=Mock(return_value=buf.buffer_info()[0]))
                      for buf in buffers)
        run=Mock(_input_buffers=buffers,_input_tensors=tensors)
        self.assertIs(bench._reused_policy_input_tensors(run),tensors)
        run._input_tensors=tensors[:-1]
        with self.assertRaisesRegex(ValueError,'six owned'):
            bench._reused_policy_input_tensors(run)
        run._input_tensors=tensors
        run._input_buffers=None
        with self.assertRaisesRegex(ValueError,'six owned'):
            bench._reused_policy_input_tensors(run)

    def collect_with_startup_pool(self,*,pool_options=None,check=lambda:None,startup_timeout=None):
        pools=[];barriers=[];events=[];result={};finished=threading.Event()
        real_barrier=threading.Barrier

        def make_pool(**kwargs):
            pool=StartupTrackingPool(**kwargs,**(pool_options or {}));pools.append(pool)
            return pool

        def make_barrier(*args,**kwargs):
            barrier=real_barrier(*args,**kwargs);barriers.append(barrier)
            return barrier

        def measured_call(kind,method):
            def call(*args,**kwargs):
                pool=pools[0]
                with pool.track_lock:workers=len(set(pool.startup_threads.values()))
                events.append((kind,time.monotonic_ns(),workers,
                               len(pool.startup_futures)==3 and all(f.done() for f in pool.startup_futures)))
                return method(*args,**kwargs)
            return call

        sessions={}
        for scope in ('front','rear'):
            session=Session();sessions[scope]=Mock(wraps=session)
            sessions[scope].exchange.side_effect=measured_call('exchange',session.exchange)
        imu=Device();device=Mock(wraps=imu)
        device.read_sample.side_effect=measured_call('imu',imu.read_sample)
        policy=Observer();observer=Mock(wraps=policy)
        observer.arm_run.side_effect=measured_call('arm',policy.arm_run)
        observer.consume.side_effect=measured_call('inference',policy.consume)

        def run():
            begin=time.monotonic()
            try:result['collected']=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=1,check=check)
            except BaseException as error:result['error']=error
            finally:result['elapsed']=time.monotonic()-begin;finished.set()

        with ExitStack() as stack:
            stack.enter_context(patch.object(bench,'ThreadPoolExecutor',side_effect=make_pool))
            stack.enter_context(patch.object(bench.threading,'Barrier',side_effect=make_barrier))
            if startup_timeout is not None:
                stack.enter_context(patch.object(bench,'WORKER_STARTUP_TIMEOUT_S',startup_timeout))
            thread=threading.Thread(target=run,daemon=True);thread.start()
            returned=False;settled=False
            try:
                returned=finished.wait(timeout=1.)
                if returned and pools:
                    settled=True
                    for future in pools[0].startup_futures:
                        try:future.result(timeout=.25)
                        except FutureTimeoutError:settled=False
                        except BaseException:pass
            finally:
                # Rescue only the fixture's workers if a regression failed to abort its barrier.
                for barrier in barriers:barrier.abort()
                for pool in pools:ThreadPoolExecutor.shutdown(pool,wait=False,cancel_futures=True)
                thread.join(timeout=.25)
        self.assertTrue(returned,'Collector did not return within the bounded fixture budget')
        self.assertNotIn('error',result,str(result.get('error')))
        self.assertEqual(len(pools),1)
        report,records=result['collected']
        return report,records,pools[0],sessions,device,observer,events,result['elapsed'],settled

    def assert_failed_worker_startup(self,report,records,pool,sessions,device,observer):
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0);self.assertEqual(report['measurements'],[])
        self.assertEqual(records,[])
        self.assertFalse(report['worker_startup']['complete'])
        self.assertEqual(report['worker_startup']['worker_count'],3)
        for session in sessions.values():session.exchange.assert_not_called()
        device.read_sample.assert_not_called()
        observer.arm_run.assert_not_called();observer.consume.assert_not_called()
        observer.invalidate.assert_called_once();observer.finish.assert_called_once()
        self.assertEqual(report['observer'],{'calls':0,'invalid':True})
        self.assertEqual(pool.shutdown_calls,[(False,True)])

    def test_delayed_worker_startup_precedes_release_and_all_measured_calls(self):
        report,records,pool,sessions,device,observer,events,_,settled=self.collect_with_startup_pool(
            pool_options={'startup_delay':.04})
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC');self.assertTrue(settled)
        startup=report['worker_startup']
        self.assertTrue(startup['complete']);self.assertEqual(startup['worker_count'],3)
        self.assertEqual(len(set(pool.startup_threads.values())),3)
        self.assertEqual(len(pool.startup_finished),3)
        self.assertGreaterEqual(startup['duration_ms'],40.)
        self.assertAlmostEqual(startup['duration_ms'],(startup['end_ns']-startup['begin_ns'])/1e6)
        self.assertLessEqual(max(pool.startup_finished.values()),startup['end_ns'])
        self.assertTrue(events)
        self.assertTrue(all(when>=startup['end_ns'] and workers==3 and ready
                            for _,when,workers,ready in events))
        self.assertEqual(report['cycles_completed'],1);self.assertEqual(len(records),1)
        for session in sessions.values():self.assertEqual(session.exchange.call_count,2)
        device.read_sample.assert_called_once();observer.arm_run.assert_called_once()
        observer.consume.assert_called_once();observer.invalidate.assert_not_called();observer.finish.assert_called_once()
        self.assertEqual(report['observer'],{'calls':1,'invalid':False})
        self.assertEqual(pool.shutdown_calls,[(True,False)])
        row=report['measurements'][0]
        self.assertGreaterEqual(row['release_ns'],startup['end_ns'])
        self.assertAlmostEqual(row['whole_iteration_ms'],(row['cycle_end_ns']-row['release_ns'])/1e6)
        total_ms=(row['cycle_end_ns']-startup['begin_ns'])/1e6
        self.assertGreaterEqual(total_ms-row['whole_iteration_ms'],startup['duration_ms']-.000001)
        for acquired,_ in records[0]['acquired'].values():
            for record in acquired:
                self.assertLessEqual(row['release_ns'],record.start_ns)
                self.assertLessEqual(record.start_ns,record.finish_ns)
                self.assertLessEqual(record.finish_ns,record.received_ns)
                self.assertLessEqual(record.received_ns,row['gather_end_ns'])
        sample=records[0]['imu']
        self.assertLessEqual(row['release_ns'],sample['read_started_monotonic_ns'])
        self.assertLessEqual(sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns'])
        self.assertLessEqual(sample['read_finished_monotonic_ns'],row['gather_end_ns'])
        self.assertEqual(set(records[0]['output']),{'front','rear'})
        self.assertTrue(all(record.start_ns>=row['infer_end_ns'] for output,_ in records[0]['output'].values()
                            for record in output))

    def test_guard_cancellation_during_worker_startup_has_no_measured_work(self):
        checks=[]
        def cancelled():
            checks.append(time.monotonic_ns())
            if len(checks)==2:raise InterruptedError('Injected cancellation during worker startup')
        report,records,pool,sessions,device,observer,events,elapsed,settled=self.collect_with_startup_pool(
            pool_options={'startup_delay':.08},check=cancelled)
        self.assert_failed_worker_startup(report,records,pool,sessions,device,observer)
        self.assertIn('Injected cancellation',report['errors'][0])
        self.assertEqual(len(checks),2);self.assertEqual(events,[])
        self.assertLess(elapsed,.5);self.assertTrue(settled)

    def test_partial_worker_submit_failure_releases_started_tasks_without_join(self):
        report,records,pool,sessions,device,observer,events,elapsed,settled=self.collect_with_startup_pool(
            pool_options={'fail_submit_at':3})
        self.assert_failed_worker_startup(report,records,pool,sessions,device,observer)
        self.assertIn('Injected startup submit failure',report['errors'][0])
        self.assertEqual(pool.submitted,3);self.assertEqual(len(pool.startup_futures),2)
        self.assertEqual(len(pool.startup_threads),2);self.assertEqual(len(pool.startup_finished),2)
        self.assertEqual(events,[]);self.assertLess(elapsed,.5);self.assertTrue(settled)

    def test_missing_third_worker_times_out_and_settles_pending_startup_tasks(self):
        checks=[]
        report,records,pool,sessions,device,observer,events,elapsed,settled=self.collect_with_startup_pool(
            pool_options={'worker_limit':2},startup_timeout=.05,check=lambda:checks.append(time.monotonic_ns()))
        self.assert_failed_worker_startup(report,records,pool,sessions,device,observer)
        self.assertIn('worker startup deadline exceeded',report['errors'][0])
        self.assertEqual(pool.submitted,3);self.assertEqual(len(pool.startup_futures),3)
        self.assertEqual(len(set(pool.startup_threads.values())),2)
        self.assertEqual(set(pool.startup_finished),set(pool.startup_threads))
        third=pool.startup_futures[2]
        if not third.cancelled():
            self.assertIsInstance(third.exception(timeout=.1),threading.BrokenBarrierError)
        self.assertGreaterEqual(len(checks),3)
        self.assertEqual(events,[]);self.assertLess(elapsed,.5);self.assertTrue(settled)

    def test_three_input_workers_are_concurrent(self):
        barrier=threading.Barrier(3)
        sessions={s:Session(barrier) for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(barrier),None,mode='type17',cycles=2)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(report['cycles_completed'],2)
        self.assertIsNone(report['host_deadline_misses'])
        self.assertTrue(all(r['final_host_write_ns'] is None for r in report['measurements']))

    def test_main_thread_affinity_keeps_workers_unpinned_and_restores(self):
        main_tid=threading.get_ident();masks={}
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')}
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,_=bench.collect(sessions,Device(),Observer(),mode='stop-proxy',cycles=2,
                                   main_thread_cpu=4)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        affinity=report['main_thread_affinity']
        self.assertEqual(affinity['before'],[0,4])
        self.assertEqual(affinity['during'],[4])
        self.assertTrue(affinity['restored'])
        self.assertEqual({tuple(row['cpus']) for row in affinity['worker_masks_after_pin']},{(0,4)})
        self.assertEqual(len({row['native_tid'] for row in affinity['worker_masks_after_pin']}),3)
        self.assertEqual(masks[main_tid],{0,4})
        self.assertTrue(all(session.calls==4 for session in sessions.values()))

    def test_v3_worker_exclusion_composes_with_initializer_and_restores_every_tid(self):
        main_tid=threading.get_ident();masks={};initialized=[]
        original={0,1,2,3,4};target={0,1,2,3}
        def get_affinity(_pid):return masks.get(threading.get_ident(),original)
        def set_affinity(_pid,cpus):masks[threading.get_ident()]=set(cpus)
        def initializer():initialized.append(threading.get_native_id())
        sessions={s:Session() for s in ('front','rear')}
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,_=bench.collect(sessions,Device(),Observer(),mode='stop-proxy',cycles=2,
                main_thread_cpu=4,v3_voltage_proxy=True,
                exclude_policy_cpu_from_workers=True,worker_initializer=initializer)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(len(set(initialized)),3)
        worker=report['worker_affinity']
        self.assertEqual(worker['target_mask'],sorted(target))
        self.assertEqual({row['native_tid'] for row in worker['workers_before']},set(initialized))
        self.assertEqual({tuple(row['after']) for row in worker['workers_during']},{tuple(sorted(target))})
        self.assertEqual({tuple(row['after']) for row in worker['workers_after']},{tuple(sorted(original))})
        self.assertTrue(worker['restored'])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertEqual(masks[main_tid],original)
        self.assertEqual({tuple(sorted(masks[tid])) for tid in masks if tid!=main_tid},
                         {tuple(sorted(original))})

    def test_v3_worker_partial_setup_failure_restores_before_any_input(self):
        main_tid=threading.get_ident();masks={};sets=0
        original={0,1,2,3,4};target={0,1,2,3}
        def get_affinity(_pid):return masks.get(threading.get_ident(),original)
        def set_affinity(_pid,cpus):
            nonlocal sets
            if threading.get_ident()!=main_tid and set(cpus)==target:
                sets+=1
                if sets==2:raise RuntimeError('injected worker setup failure')
            masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=1,
                main_thread_cpu=4,v3_voltage_proxy=True,exclude_policy_cpu_from_workers=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(records,[])
        self.assertIn('injected worker setup failure',report['errors'][0])
        self.assertTrue(report['worker_affinity']['restored'])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertTrue(observer.invalid)
        self.assertEqual({tuple(sorted(mask)) for mask in masks.values()},{tuple(sorted(original))})
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_v3_worker_restore_mismatch_invalidates_complete_run(self):
        main_tid=threading.get_ident();masks={};skipped=set()
        original={0,1,2,3,4};target={0,1,2,3}
        def get_affinity(_pid):return masks.get(threading.get_ident(),original)
        def set_affinity(_pid,cpus):
            tid=threading.get_ident()
            if tid!=main_tid and set(cpus)==original and not skipped:
                skipped.add(tid);return
            masks[tid]=set(cpus)
        observer=Observer()
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,_=bench.collect({s:Session() for s in ('front','rear')},Device(),observer,
                mode='stop-proxy',cycles=1,main_thread_cpu=4,v3_voltage_proxy=True,
                exclude_policy_cpu_from_workers=True)
        self.assertEqual(report['cycles_completed'],1)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['worker_affinity']['restored'])
        self.assertIn('readback differs',report['worker_affinity']['restore_errors'][0])
        self.assertIn('restoration unconfirmed',report['errors'][-1])
        self.assertTrue(observer.invalid)

    def test_worker_exclusion_requires_v3_stop_policy_pin_and_three_other_cpus(self):
        sessions={s:Session() for s in ('front','rear')}
        for options in ({}, {'main_thread_cpu':4},
                        {'main_thread_cpu':4,'v3_voltage_proxy':True,'mode':'type17'},
                        {'main_thread_cpu':4,'v3_voltage_proxy':True,'cycles':501}):
            with self.subTest(options=options),self.assertRaisesRegex(ValueError,'requires'):
                bench.collect(sessions,Device(),Observer(),mode=options.get('mode','stop-proxy'),
                    cycles=options.get('cycles',1),main_thread_cpu=options.get('main_thread_cpu'),
                    v3_voltage_proxy=options.get('v3_voltage_proxy',False),
                    exclude_policy_cpu_from_workers=True)
        with patch.object(bench.os,'sched_getaffinity',return_value={0,1,4},create=True), \
                patch.object(bench.os,'sched_setaffinity',create=True):
            report,_=bench.collect(sessions,Device(),Observer(),mode='stop-proxy',cycles=1,
                main_thread_cpu=4,v3_voltage_proxy=True,exclude_policy_cpu_from_workers=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('at least three other',report['errors'][0])
        self.assertEqual(report['cycles_completed'],0)

    def test_pre_cycle_policy_prepare_runs_before_pin_and_any_input(self):
        main_tid=threading.get_ident();masks={};events=[]
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):
            events.append(('pin' if set(cpus)=={4} else 'restore',time.monotonic_ns()))
            masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};device=Device();observer=Observer()
        def prepared():
            events.append(('prepare',time.monotonic_ns()))
            self.assertEqual(threading.get_ident(),main_tid)
            self.assertEqual(get_affinity(0),{0,4})
            self.assertTrue(all(session.calls==0 for session in sessions.values()))
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,_=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=1,
                main_thread_cpu=4,pre_cycle_policy_prepare=prepared)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual([kind for kind,_ in events],['prepare','pin','restore'])
        self.assertGreaterEqual(events[0][1],report['worker_startup']['end_ns'])
        self.assertLessEqual(events[1][1],report['measurements'][0]['release_ns'])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertEqual(masks[main_tid],{0,4})

    def test_pre_cycle_warmup_rebases_first_release_and_finite_deadline(self):
        fake_ns=1_000_000_000;lock=threading.Lock();prepared=[]
        def fake_clock():
            nonlocal fake_ns
            with lock:
                fake_ns+=10_000
                return fake_ns
        def prepare():
            nonlocal fake_ns
            with lock:
                fake_ns+=3_000_000_000
                prepared.append(fake_ns)
        sessions={s:Session() for s in ('front','rear')}
        with patch.object(bench.time,'monotonic_ns',side_effect=fake_clock):
            report,_=bench.collect(sessions,Device(),Observer(),mode='stop-proxy',cycles=1,
                clock=fake_clock,pre_cycle_policy_prepare=prepare)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(report['cycles_completed'],1)
        self.assertEqual(len(prepared),1)
        first=report['measurements'][0]
        self.assertGreaterEqual(first['release_ns'],prepared[0])
        self.assertGreater(first['release_ns']-report['worker_startup']['end_ns'],3_000_000_000)
        self.assertLess(first['release_lateness_ms'],1.)

    def test_pre_cycle_policy_prepare_failure_aborts_before_inputs_and_pin(self):
        main_tid=threading.get_ident();masks={};calls=[]
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):
            calls.append('pin')
            masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        def failed_prepare():
            calls.append(1)
            self.assertEqual(get_affinity(0),{0,4})
            raise RuntimeError('injected pre-cycle warmup failure')
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=1,
                main_thread_cpu=4,pre_cycle_policy_prepare=failed_prepare)
        self.assertEqual(calls,[1])
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(records,[])
        self.assertIn('injected pre-cycle warmup failure',report['errors'][0])
        self.assertTrue(observer.invalid)
        self.assertIsNone(report['main_thread_affinity']['restored'])
        self.assertNotIn(main_tid,masks)
        self.assertEqual(get_affinity(0),{0,4})
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_post_pin_policy_prepare_precedes_first_fresh_measured_cycle(self):
        main_tid=threading.get_ident();masks={};events=[]
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):
            events.append(('pin' if set(cpus)=={4} else 'restore',time.monotonic_ns()))
            masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};device=Device();observer=Observer()
        def before_pin():
            events.append(('warmup',time.monotonic_ns()))
            self.assertEqual(get_affinity(0),{0,4})
        def after_pin():
            events.append(('prime',time.monotonic_ns()))
            self.assertEqual(get_affinity(0),{4})
            self.assertTrue(all(session.calls==0 for session in sessions.values()))
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,records=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=2,
                main_thread_cpu=4,pre_cycle_policy_prepare=before_pin,
                post_pin_policy_prepare=after_pin)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual([kind for kind,_ in events],['warmup','pin','prime','restore'])
        self.assertEqual(len(records),2)
        self.assertGreaterEqual(report['measurements'][0]['release_ns'],events[2][1])
        self.assertLess(records[0]['imu']['read_started_monotonic_ns'],
                        records[1]['imu']['read_started_monotonic_ns'])
        self.assertEqual(masks[main_tid],{0,4})
        self.assertTrue(all(session.calls==4 for session in sessions.values()))

    def test_measured_observer_arms_before_release_and_counts_slow_first_cycle(self):
        class MeasuredObserver(Observer):
            _measured_diagnostic_ticks=True
            def arm_run(self,tick):
                self.armed_at=time.monotonic_ns()
                super().arm_run(tick)
            def consume(self,snapshot):
                self.asserted_tick=snapshot['tick_ns']>=self.first
                return super().consume(snapshot)

        class SlowFirstInput(Session):
            def exchange(self,wires):
                if self.calls==0:time.sleep(.022)
                return super().exchange(wires)

        observer=MeasuredObserver()
        sessions={s:SlowFirstInput() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=2,
            v3_voltage_proxy=True,absolute_epoch_cadence=True,record_storage='trace',
            inference_thread_cpu_trace=True)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertTrue(observer.asserted_tick)
        self.assertLess(observer.armed_at,report['measurements'][0]['release_ns'])
        self.assertEqual(report['cycles_requested'],2)
        self.assertEqual(report['cycles_completed'],2)
        self.assertEqual(len(records),2)
        self.assertEqual(observer.calls,2)
        self.assertEqual(report['iteration_deadline_misses'],1)
        self.assertFalse(report['measurements'][0]['iteration_deadline_met'])
        self.assertGreater(report['measurements'][0]['whole_iteration_ms'],20.)
        self.assertTrue(report['measurements'][1]['iteration_deadline_met'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertFalse(report['absolute_epoch_schedule']['strict_start_interval_20ms_met'])
        self.assertIn('arm_run completed before timed release',
                      report['inference_thread_cpu_trace']['scope'])
        for row in records:
            evidence=row.serialize()
            self.assertEqual([len(evidence['acquired'][scope]['records']) for scope in ('front','rear')],[7,7])
            self.assertEqual([len(evidence['output'][scope]['records']) for scope in ('front','rear')],[6,6])
        self.assertTrue(all(session.calls==4 for session in sessions.values()))

    def test_trace_pages_are_zeroed_before_release_without_losing_500_cycles(self):
        touched=[];real_memset=bench.C.memset
        def observe_touch(address,value,size):
            result=real_memset(address,value,size)
            touched.append((time.monotonic_ns(),value,size,
                            not any(bench.C.string_at(address,size))))
            return result
        observer=Observer()
        sessions={scope:Session() for scope in ('front','rear')}
        # A no-op injected sleep keeps this a fast storage/accounting test; it
        # is not a simulated 50 Hz timing result.
        with patch.object(bench.C,'memset',side_effect=observe_touch):
            report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',
                cycles=500,record_storage='trace',v3_voltage_proxy=True,sleep=lambda _:None)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual((report['cycles_requested'],report['cycles_completed'],len(records),observer.calls),
                         (500,500,500,500))
        self.assertEqual(len(touched),1)
        touched_at,value,size,zeroed=touched[0]
        self.assertEqual(value,0)
        self.assertTrue(zeroed)
        self.assertLess(touched_at,report['measurements'][0]['release_ns'])
        storage=records[0].storage
        self.assertEqual(size,storage.allocated_bytes)
        self.assertEqual(len(storage.slots),500*4)
        self.assertEqual(sum(slot.count for slot in storage.slots),500*26)
        self.assertEqual(report['record_storage']['allocated_bytes'],size)
        self.assertEqual(report['record_storage']['pretouched_before_release_bytes'],size)
        self.assertEqual(report['record_storage']['completed_trace_rows'],500)
        self.assertTrue(report['record_storage']['copy_inside_whole_iteration'])
        self.assertEqual(report['iteration_deadline_misses'],
                         sum(not row['iteration_deadline_met'] for row in report['measurements']))
        for index in (0,499):
            row=records[index].serialize()
            self.assertEqual([len(row['acquired'][scope]['records']) for scope in ('front','rear')],
                             [7,7])
            self.assertEqual([len(row['output'][scope]['records']) for scope in ('front','rear')],
                             [6,6])

    def test_failed_post_pin_prime_restores_affinity_without_measured_inputs(self):
        main_tid=threading.get_ident();masks={}
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};device=Mock(wraps=Device());observer=Observer()
        def fail():raise RuntimeError('injected post-pin prime failure')
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,records=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=1,
                main_thread_cpu=4,pre_cycle_policy_prepare=lambda:None,
                post_pin_policy_prepare=fail)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(records,[])
        self.assertIn('injected post-pin prime failure',report['errors'][0])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertEqual(masks[main_tid],{0,4})
        self.assertTrue(all(session.calls==0 for session in sessions.values()))
        device.read_sample.assert_not_called()
        self.assertTrue(observer.invalid)

    def test_main_thread_affinity_worker_mismatch_aborts_before_inputs(self):
        main_tid=threading.get_ident();masks={}
        def get_affinity(_pid):
            tid=threading.get_ident()
            return masks.get(tid,{0,4} if tid==main_tid else {4})
        def set_affinity(_pid,cpus):masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};device=Device();observer=Observer()
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,records=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=1,
                                         main_thread_cpu=4)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(records,[])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertIn('worker affinity changed',report['errors'][0])
        self.assertTrue(observer.invalid)
        self.assertEqual(masks[main_tid],{0,4})
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_main_thread_affinity_restore_mismatch_invalidates_complete_run(self):
        main_tid=threading.get_ident();masks={}
        def get_affinity(_pid):return masks.get(threading.get_ident(),{0,4})
        def set_affinity(_pid,cpus):
            # Simulate an OS restore that returns success but leaves the main
            # thread pinned. A complete data path must not be reported valid.
            if threading.get_ident()==main_tid and set(cpus)=={0,4}:return
            masks[threading.get_ident()]=set(cpus)
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        with patch.object(bench.os,'sched_getaffinity',get_affinity,create=True), \
                patch.object(bench.os,'sched_setaffinity',set_affinity,create=True):
            report,_=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=1,
                                   main_thread_cpu=4)
        self.assertEqual(report['cycles_completed'],1)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['main_thread_affinity']['restored'])
        self.assertTrue(observer.invalid)
        self.assertIn('restoration differs',report['errors'][-1])

    def test_timer_slack_worker_checks_finish_before_any_measured_input(self):
        backend=FakePrctl();scope=slack.TimerSlack(1_000)
        checked=[];inputs=[]
        def initialized():
            scope.worker_initializer();checked.append(time.monotonic_ns())
        def measured(method):
            def call(*args,**kwargs):
                self.assertTrue(scope.report['worker_verification_complete'])
                self.assertEqual(len(checked),3)
                inputs.append(time.monotonic_ns())
                return method(*args,**kwargs)
            return call
        sessions={}
        for name in ('front','rear'):
            session=Session();sessions[name]=Mock(wraps=session)
            sessions[name].exchange.side_effect=measured(session.exchange)
        imu=Device();device=Mock(wraps=imu)
        device.read_sample.side_effect=measured(imu.read_sample)
        with patch.object(slack.sys,'platform','linux'),patch.object(slack,'_load_prctl',return_value=backend):
            with scope:
                report,records=bench.collect(sessions,device,None,mode='type17',cycles=1,
                                             worker_initializer=initialized)
                scope.verify_workers()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(len(records),1);self.assertEqual(len(inputs),3)
        self.assertEqual(len({row['native_tid'] for row in scope.report['workers']}),3)
        self.assertLessEqual(max(checked),report['worker_startup']['end_ns'])
        self.assertLessEqual(report['worker_startup']['end_ns'],min(inputs))
        self.assertTrue(scope.report['parent']['restored'])
        for session in sessions.values():session.exchange.assert_called_once()
        device.read_sample.assert_called_once()
        self.assertTrue(all(row[1]==backend.parent_tid for row in backend.calls if row[0]=='set'))

    def test_timer_slack_worker_mismatch_aborts_startup_without_inputs_or_output(self):
        backend=FakePrctl(inherited=50_000);scope=slack.TimerSlack(1_000)
        sessions={name:Mock(wraps=Session()) for name in ('front','rear')}
        device=Mock(wraps=Device());observer=Mock(wraps=Observer())
        pools=[]
        def make_pool(**options):
            pool=ThreadPoolExecutor(**options);pools.append(pool);return pool
        with patch.object(slack.sys,'platform','linux'),patch.object(slack,'_load_prctl',return_value=backend), \
                patch.object(bench,'ThreadPoolExecutor',side_effect=make_pool), \
                contextlib.redirect_stderr(io.StringIO()):
            with scope:
                report,records=bench.collect(sessions,device,observer,mode='stop-proxy',cycles=1,
                                             worker_initializer=scope.worker_initializer)
                # Join only the fixture's finite, pure failing initializers after collect has returned.
                for pool in pools:pool.shutdown(wait=True,cancel_futures=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['worker_startup']['complete'])
        self.assertEqual(report['measurements'],[]);self.assertEqual(records,[])
        for session in sessions.values():session.exchange.assert_not_called()
        device.read_sample.assert_not_called()
        observer.arm_run.assert_not_called();observer.consume.assert_not_called()
        observer.invalidate.assert_called_once();observer.finish.assert_called_once()
        self.assertTrue(scope.report['workers'])
        self.assertTrue(all(not row['verified'] for row in scope.report['workers']))
        self.assertFalse(scope.report['worker_verification_complete'])
        self.assertTrue(scope.report['parent']['restored'])

    def test_real_inference_order_followed_only_by_stop_proxy(self):
        sessions={s:Session() for s in ('front','rear')};o=Observer()
        report,records=bench.collect(sessions,Device(),o,mode='stop-proxy',cycles=3)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(o.calls,3)
        self.assertTrue(all(s.calls==6 for s in sessions.values()))
        for row in records:
            for s,(rs,_) in row['output'].items():
                self.assertTrue(all(ATParser().feed(bytes(r.tx))[0].kind==4 for r in rs))
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertFalse(report['learned_targets_sent'])

    def test_output_dispatch_trace_locates_guard_submit_workers_and_gc_without_changing_stop(self):
        class TimedSession(Session):
            def exchange(self,wires):
                records,stats=super().exchange(wires)
                stats.begin_ns=records[0].start_ns-1
                return records,stats
        class CollectingObserver(Observer):
            def consume(self,snapshot):
                result=super().consume(snapshot)
                gc.collect(0)
                return result
        original_callbacks=tuple(gc.callbacks)
        sessions={s:TimedSession() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),CollectingObserver(),
            mode='stop-proxy',cycles=2,record_storage='trace',output_dispatch_trace=True)
        self.assertEqual(tuple(gc.callbacks),original_callbacks)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])
        trace=report['output_dispatch_trace']
        self.assertEqual(trace['schema'],'native-output-dispatch-v1')
        self.assertEqual(len(trace['rows']),2)
        self.assertEqual(trace['gc_probe_errors'],0)
        self.assertEqual(trace['gc_overflow'],0)
        for cycle,row in enumerate(trace['rows'],1):
            times=dict(zip(trace['fields'],row))
            self.assertEqual(times['infer_end_ns'],report['measurements'][cycle-1]['infer_end_ns'])
            self.assertLessEqual(times['infer_end_ns'],times['main_check_start_ns'])
            self.assertLessEqual(times['main_check_start_ns'],times['main_check_end_ns'])
            self.assertLessEqual(times['main_check_end_ns'],times['front_submit_end_ns'])
            self.assertLessEqual(times['front_submit_end_ns'],times['rear_submit_end_ns'])
            self.assertLessEqual(times['main_infer_thread_cpu_ns'],
                                 times['main_submits_end_thread_cpu_ns'])
            for scope in ('front','rear'):
                self.assertLessEqual(times[scope+'_worker_enter_ns'],times[scope+'_worker_check_end_ns'])
                self.assertLessEqual(times[scope+'_worker_check_end_ns'],times[scope+'_native_begin_ns'])
                self.assertLessEqual(times[scope+'_native_begin_ns'],times[scope+'_first_write_ns'])
            for scope in ('front','rear'):
                for record in records[cycle-1].serialize()['output'][scope]['records']:
                    self.assertEqual(ATParser().feed(bytes.fromhex(record['tx_hex']))[0].kind,4)
        for cycle in (1,2):
            events=[event for event in trace['gc_events']
                    if event['cycle']==cycle and event['generation']==0]
            self.assertTrue(any(event['phase']=='start' for event in events))
            self.assertTrue(any(event['phase']=='stop' for event in events))
        plain,_=bench.collect({s:TimedSession() for s in ('front','rear')},Device(),Observer(),
                              mode='stop-proxy',cycles=1)
        self.assertNotIn('output_dispatch_trace',plain)

    def test_output_dispatch_trace_preserves_model_rejection_before_output(self):
        class Reject(Observer):
            def consume(self,snapshot):raise ValueError('range rejected')
        callbacks=tuple(gc.callbacks)
        sessions={s:Session() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),Reject(),mode='stop-proxy',cycles=1,
                                     output_dispatch_trace=True)
        self.assertEqual(tuple(gc.callbacks),callbacks)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['output_dispatch_trace']['rows'],[])
        self.assertTrue(all(session.calls==1 for session in sessions.values()))
        self.assertEqual(records[0]['output'],{})

    def test_output_dispatch_gc_ring_is_bounded(self):
        class ManyCollections(Observer):
            def consume(self,snapshot):
                result=super().consume(snapshot)
                for _ in range(40):gc.collect(0)
                return result
        callbacks=tuple(gc.callbacks)
        report,_=bench.collect({s:Session() for s in ('front','rear')},Device(),
            ManyCollections(),mode='stop-proxy',cycles=1,output_dispatch_trace=True)
        self.assertEqual(tuple(gc.callbacks),callbacks)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        trace=report['output_dispatch_trace']
        self.assertEqual(len(trace['gc_events']),64)
        self.assertGreater(trace['gc_overflow'],0)
        self.assertEqual(trace['gc_probe_errors'],0)

    def test_bounded_gc_deferral_restores_state_and_keeps_stop_trace(self):
        self.assertTrue(gc.isenabled())
        original=gc.get_threshold()
        class CheckDisabled(Observer):
            def consume(self,snapshot):
                self_test.assertFalse(gc.isenabled())
                return super().consume(snapshot)
        self_test=self
        sessions={s:Session() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),CheckDisabled(),mode='stop-proxy',
            cycles=2,record_storage='trace',output_dispatch_trace=True,
            defer_gc_during_cycles=True)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(report['cycles_completed'],2)
        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_threshold(),original)
        state=report['cycle_gc_defer']
        self.assertTrue(state['before_enabled'])
        self.assertFalse(state['during_enabled'])
        self.assertTrue(state['after_enabled'])
        self.assertEqual(state['before_threshold'],original)
        self.assertEqual(state['after_threshold'],original)
        self.assertTrue(state['restored'])
        self.assertEqual(state['restore_attempts'],1)
        self.assertEqual(len(report['output_dispatch_trace']['rows']),2)
        self.assertTrue(all(session.calls==4 for session in sessions.values()))
        for row in records:
            for scope in ('front','rear'):
                self.assertTrue(all(ATParser().feed(bytes.fromhex(r['tx_hex']))[0].kind==4
                    for r in row.serialize()['output'][scope]['records']))

    def test_bounded_gc_deferral_restores_enabled_state_and_threshold_after_rejection(self):
        self.assertTrue(gc.isenabled())
        original=gc.get_threshold()
        class Reject(Observer):
            def consume(self,snapshot):
                gc.set_threshold(1,2,3)
                raise ValueError('model rejection')
        observer=Reject();sessions={s:Session() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',
            cycles=500,record_storage='trace',output_dispatch_trace=True,
            defer_gc_during_cycles=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('model rejection',report['errors'][0])
        self.assertTrue(report['cycle_gc_defer']['restored'])
        self.assertEqual(report['cycle_gc_defer']['after_threshold'],original)
        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_threshold(),original)
        self.assertTrue(observer.invalid)
        self.assertEqual(records[0]['output'],{})
        self.assertTrue(all(session.calls==1 for session in sessions.values()))

    def test_gc_deferral_transition_failure_still_restores_and_blocks_output(self):
        self.assertTrue(gc.isenabled())
        original=gc.get_threshold();real_disable=gc.disable
        def fail_after_disabling():
            real_disable()
            raise RuntimeError('injected gc disable failure')
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        with patch.object(bench.gc,'disable',side_effect=fail_after_disabling):
            report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',
                cycles=2,record_storage='trace',output_dispatch_trace=True,
                defer_gc_during_cycles=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('injected gc disable failure',report['errors'][0])
        self.assertTrue(report['cycle_gc_defer']['restored'])
        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_threshold(),original)
        self.assertTrue(observer.invalid)
        self.assertEqual(records,[])
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_gc_deferral_unconfirmed_restoration_aborts_diagnostic(self):
        self.assertTrue(gc.isenabled())
        original=gc.get_threshold();real_enable=gc.enable
        observer=Observer()
        try:
            with patch.object(bench.gc,'enable',return_value=None):
                report,_=bench.collect({s:Session() for s in ('front','rear')},Device(),observer,
                    mode='stop-proxy',cycles=1,record_storage='trace',
                    output_dispatch_trace=True,defer_gc_during_cycles=True)
        finally:
            real_enable()
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('restoration unconfirmed',report['errors'][-1])
        self.assertFalse(report['cycle_gc_defer']['restored'])
        self.assertEqual(report['cycle_gc_defer']['restore_attempts'],3)
        self.assertTrue(observer.invalid)
        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_threshold(),original)

    def test_gc_deferral_rejects_untraced_or_unbounded_collection_before_work(self):
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        for options in ({'cycles':501,'record_storage':'trace','output_dispatch_trace':True},
                        {'cycles':2,'record_storage':'objects','output_dispatch_trace':True},
                        {'cycles':2,'record_storage':'trace','output_dispatch_trace':False}):
            with self.assertRaisesRegex(ValueError,'GC deferral requires'):
                bench.collect(sessions,Device(),observer,mode='stop-proxy',
                              defer_gc_during_cycles=True,**options)
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_reused_imu_stops_before_another_inference_or_output(self):
        d=Device();d.reuse=d.read_sample();o=Observer()
        report,_=bench.collect({s:Session() for s in ('front','rear')},d,o,mode='stop-proxy',cycles=2)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(o.calls,1)
        self.assertIn('reused',report['errors'][0])

    def test_model_rejection_sends_no_output_proxy(self):
        class Reject(Observer):
            def consume(self,s):raise ValueError('range rejected')
        sessions={s:Session() for s in ('front','rear')}
        report,records=bench.collect(sessions,Device(),Reject(),mode='stop-proxy',cycles=2)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(all(s.calls==1 for s in sessions.values()))
        self.assertEqual(records[0]['output'],{})

    def test_v3_voltage_proxy_rotates_one_read_per_bus_and_never_enables(self):
        class TimingObserver(Observer):
            def consume(self,snapshot):
                starts=[r['request_ns'] for r in snapshot['motors']]
                ends=[r['received_ns'] for r in snapshot['motors']]
                starts.append(snapshot['imu']['read_started_ns'])
                ends.append(snapshot['imu']['read_finished_ns'])
                self_test.assertEqual(snapshot['oldest_observation_age_ns'],
                                      snapshot['tick_ns']-min(starts))
                self_test.assertEqual(snapshot['acquisition_spread_ns'],max(ends)-min(starts))
                self_test.assertEqual(snapshot['receive_spread_ns'],max(ends)-min(ends))
                self_test.assertEqual(set(snapshot['voltage_by_bus']),{'front','rear'})
                return super().consume(snapshot)
        self_test=self
        sessions={s:Session() for s in ('front','rear')};observer=TimingObserver()
        report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=7,
                                     record_storage='trace',v3_voltage_proxy=True)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertTrue(report['v3_voltage_proxy'])
        self.assertEqual((report['cycles_completed'],observer.calls),(7,7))
        self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(len(records),7)
        for cycle,row in enumerate(records):
            value=row.serialize()
            for scope,ids in bench.dual.SCOPES.items():
                inputs=value['acquired'][scope]['records']
                outputs=value['output'][scope]['records']
                self.assertEqual((len(inputs),len(outputs)),(7,6))
                input_frames=[ATParser().feed(bytes.fromhex(r['tx_hex']))[0] for r in inputs]
                output_frames=[ATParser().feed(bytes.fromhex(r['tx_hex']))[0] for r in outputs]
                self.assertEqual([f.kind for f in input_frames],[4]*6+[17])
                self.assertEqual([f.kind for f in output_frames],[4]*6)
                self.assertEqual(input_frames[-1].destination,ids[cycle%6])
                self.assertEqual(input_frames[-1].data[:2],b'\x1c\x70')
        self.assertEqual([sessions[s].calls for s in ('front','rear')],[14,14])

    def test_inference_thread_cpu_trace_is_bounded_per_cycle_and_opt_in(self):
        sessions={s:Session() for s in ('front','rear')}
        with patch.object(bench.time,'thread_time_ns',side_effect=[10_000,10_100,11_000,11_200]) as cpu:
            report,records=bench.collect(sessions,Device(),Observer(),mode='stop-proxy',
                cycles=2,record_storage='trace',v3_voltage_proxy=True,
                inference_thread_cpu_trace=True)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(cpu.call_count,4)
        trace=report['inference_thread_cpu_trace']
        self.assertEqual(trace['schema'],'native-inference-thread-cpu-v1')
        self.assertEqual(trace['clock'],'time.thread_time_ns')
        self.assertFalse(trace['helper_thread_cpu_included'])
        self.assertEqual(trace['fields'],['cycle','thread_cpu_begin_ns','thread_cpu_end_ns',
                         'thread_cpu_ns','inference_wall_ns','wall_minus_thread_cpu_ns'])
        self.assertEqual([row[:4] for row in trace['rows']],
                         [[1,10_000,10_100,100],[2,11_000,11_200,200]])
        for index,row in enumerate(trace['rows']):
            measurement=report['measurements'][index]
            self.assertEqual(row[4],measurement['infer_end_ns']-measurement['prepare_end_ns'])
            self.assertEqual(row[5],row[4]-row[3])
        self.assertEqual([sessions[s].calls for s in ('front','rear')],[4,4])
        self.assertTrue(all(len(row.serialize()['acquired'][s]['records'])==7 and
                            len(row.serialize()['output'][s]['records'])==6
                            for row in records for s in ('front','rear')))
        json.dumps(report,allow_nan=False)

        with patch.object(bench.time,'thread_time_ns',side_effect=AssertionError('unrequested CPU clock')):
            plain,_=bench.collect({s:Session() for s in ('front','rear')},Device(),Observer(),
                                  mode='stop-proxy',cycles=1,v3_voltage_proxy=True)
        self.assertEqual(plain['status'],'COMPLETE_DIAGNOSTIC')
        self.assertNotIn('inference_thread_cpu_trace',plain)

    def test_inference_thread_cpu_trace_rejection_keeps_output_empty(self):
        class Reject(Observer):
            def consume(self,snapshot):raise ValueError('range rejected')
        sessions={s:Session() for s in ('front','rear')}
        with patch.object(bench.time,'thread_time_ns',side_effect=[10_000]) as cpu:
            report,records=bench.collect(sessions,Device(),Reject(),mode='stop-proxy',
                cycles=2,v3_voltage_proxy=True,inference_thread_cpu_trace=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(report['inference_thread_cpu_trace']['rows'],[])
        self.assertEqual(cpu.call_count,1)
        self.assertEqual(records[0]['output'],{})
        self.assertTrue(all(session.calls==1 for session in sessions.values()))

    def test_inference_thread_cpu_trace_requires_26_request_inference_before_work(self):
        sessions={s:Session() for s in ('front','rear')};observer=Observer()
        for options in ({'v3_voltage_proxy':False,'inference_thread_cpu_trace':True},
                        {'v3_voltage_proxy':True,'inference_thread_cpu_trace':1}):
            with self.subTest(options=options),self.assertRaisesRegex(ValueError,
                    'Inference thread CPU trace requires'):
                bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=1,**options)
        self.assertTrue(all(session.calls==0 for session in sessions.values()))
        self.assertEqual(observer.calls,0)

    def test_v3_voltage_proxy_missing_read_prevents_inference_and_output(self):
        class DropVoltage(Session):
            def exchange(self,wires):
                rows,stats=super().exchange(wires)
                return (rows[:-1] if len(wires)==7 else rows),stats
        sessions={s:DropVoltage() for s in ('front','rear')};observer=Observer()
        report,records=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=2,
                                     v3_voltage_proxy=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Missing or incorrect rotating voltage',report['errors'][0])
        self.assertEqual(observer.calls,0)
        self.assertEqual(records[0]['output'],{})
        self.assertEqual([sessions[s].calls for s in ('front','rear')],[1,1])

    def test_v3_voltage_proxy_requires_finite_bounded_inference_run(self):
        sessions={s:Session() for s in ('front','rear')}
        for mode,observer,cycles in (('type17',Observer(),1),('stop-proxy',None,1),
                                     ('stop-proxy',Observer(),501)):
            with self.subTest(mode=mode,cycles=cycles):
                with self.assertRaisesRegex(ValueError,'V3 voltage proxy requires'):
                    bench.collect(sessions,Device(),observer,mode=mode,cycles=cycles,
                                  v3_voltage_proxy=True)
        self.assertEqual([sessions[s].calls for s in ('front','rear')],[0,0])

    def test_snapshot_rejects_fault_missing_wrong_bus_and_keeps_original_times(self):
        acquired={s:Session().exchange([native.stop_wire(i) for i in ids])[0]
                  for s,ids in bench.dual.SCOPES.items()}
        sample=Device().read_sample();tick=time.monotonic_ns()
        s=bench.snapshot_from_records(acquired,sample,tick)
        self.assertEqual(s['motors'][0]['request_ns'],acquired['front'][0].start_ns)
        self.assertEqual(s['imu']['read_started_ns'],sample['read_started_monotonic_ns'])
        with self.assertRaisesRegex(ValueError,'Missing'):
            bench.snapshot_from_records({'front':acquired['front']},sample,tick)
        with self.assertRaisesRegex(ValueError,'Cross-bus'):
            bench.snapshot_from_records({'rear':acquired['front'],'front':acquired['rear']},sample,tick)
        acquired['front'][0].rx[3] |= 8
        with self.assertRaisesRegex(ValueError,'Invalid STOP'):
            bench.snapshot_from_records(acquired,sample,tick)

    def test_fixed_native_frame_decoder_matches_stream_parser_and_rejects_corruption(self):
        for mode in ('type17','stop-proxy'):
            with self.subTest(mode=mode):
                wires={scope:([native.stop_wire(i) for i in ids] if mode=='stop-proxy' else
                    [bench.codec.read_request(i,p) for p in ('position','velocity') for i in ids])
                    for scope,ids in bench.dual.SCOPES.items()}
                acquired={scope:Session().exchange(batch)[0] for scope,batch in wires.items()}
                sample=Device().read_sample();tick=time.monotonic_ns()
                actual=bench.snapshot_from_records(acquired,sample,tick)
                expected_keys=[]
                for scope,records in acquired.items():
                    for record in records:
                        for wire in (bytes(record.tx),bytes(record.rx)):
                            self.assertEqual(bench._native_record_frame(wire),ATParser().feed(wire)[0])
                        mid=bench._native_record_frame(bytes(record.tx)).destination
                        parameters=(('position','velocity') if mode=='stop-proxy' else
                                    (('position',) if bytes(record.tx)[7:9]==b'\x19\x70'
                                     else ('velocity',)))
                        expected_keys.extend((mid,p) for p in parameters)
                self.assertEqual([(row['motor_id'],row['parameter']) for row in actual['motors']],
                                 expected_keys)
                self.assertEqual(actual['oldest_observation_age_ns'],
                    tick-min([sample['read_started_monotonic_ns']]+[
                        r.start_ns for records in acquired.values() for r in records]))
                self.assertEqual(actual['receive_spread_ns'],
                    max([sample['read_finished_monotonic_ns']]+[
                        r.received_ns for records in acquired.values() for r in records])-
                    min([sample['read_finished_monotonic_ns']]+[
                        r.received_ns for records in acquired.values() for r in records]))
                # One corrupted fixed ABI wire must never become a source value.
                for offset,value in ((0,ord('X')),(6,7),(15,ord('X'))):
                    with self.subTest(mode=mode,offset=offset):
                        records=acquired['front'];old=records[0].rx[offset]
                        records[0].rx[offset]=value
                        try:
                            with self.assertRaisesRegex(ValueError,'Malformed native record'):
                                bench.snapshot_from_records(acquired,sample,tick)
                        finally:records[0].rx[offset]=old

    def test_timing_counts_oldest_imu_and_reply_tail_separately(self):
        r=native.Record();r.start_ns=10_000_000;r.received_ns=13_000_000
        out=native.Record();out.finish_ns=22_000_000;out.received_ns=25_000_000
        row=bench.timing_row({'front':([r],None)},
            {'read_started_monotonic_ns':1_000_000,'read_finished_monotonic_ns':2_000_000},
            {'front':([out],None)},release_ns=0,gather_end_ns=14000000,
            prepare_end_ns=15000000,infer_end_ns=18000000,cycle_end_ns=26000000)
        self.assertEqual(row['oldest_input_to_final_host_write_ms'],21.)
        self.assertEqual(row['oldest_input_to_last_reply_ms'],24.)
        self.assertFalse(row['host_deadline_met']);self.assertFalse(row['iteration_deadline_met'])

    def test_default_observer_schedule_is_unchanged(self):
        o=make();o.reset_run(1_000_000_000,warmup_completed=True)
        with self.assertRaisesRegex(ValueError,'20ms tick'):
            o.consume(snapshot(1_001_000_000))

    def test_diagnostic_observer_keeps_state_with_measured_ticks(self):
        o=make(max_ticks=3,measured_diagnostic_ticks=True)
        o.reset_run(1_000_000_000,warmup_completed=True)
        first=o.consume(snapshot(1_001_000_000));second=o.consume(snapshot(1_026_000_000))
        self.assertEqual(second['tick_index'],1)
        self.assertIn('actual acquisition',first['timing_scope'])
        with self.assertRaisesRegex(ValueError,'Repeated/backward'):
            o.consume(snapshot(1_026_000_000))

    def test_missing_input_preserves_failure_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'capture'
            rc=bench.main(['--execute','--mode','type17','--acquisition-only',
                '--expected-uids',str(Path(tmp)/'missing.json'),'--library',str(Path(tmp)/'missing.so'),
                '--front-port','/dev/serial/by-path/front','--rear-port','/dev/serial/by-path/rear',
                '--output',str(out)])
            self.assertEqual(rc,2)
            self.assertEqual(json.loads((out/'report.json').read_text())['status'],'ABORTED')
            self.assertEqual(json.loads((out/'records.json').read_text()),[])

if __name__=='__main__':unittest.main()
