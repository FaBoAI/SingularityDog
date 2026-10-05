"""No-device acquisition readiness tests; these establish no OS latency bound."""
from concurrent.futures import Future
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device
from test_native_voltage_fast_pipeline import PreparedSession
from test_native_voltage_overlap import OverlapObserver


class Clock:
    def __init__(self): self.now = 1000
    def __call__(self): return self.now


class NativeAcquisitionJoinWaitTests(unittest.TestCase):
    def futures(self, ready=False):
        owners = {scope:Future() for scope in ('front','rear')}; imu = Future()
        if ready:
            for scope,future in owners.items(): future.set_result(scope)
            imu.set_result('imu')
        return owners, imu

    def test_ready_values_remain_with_original_owners_and_imu(self):
        owners,imu = self.futures(True); native_wait = Mock()
        with (patch.object(owners['front'],'result',side_effect=AssertionError('No takeout')),
              patch.object(owners['rear'],'result',side_effect=AssertionError('No takeout')),
              patch.object(imu,'result',side_effect=AssertionError('No IMU takeout'))):
            proof = bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                clock=Clock(),deadline_wait=native_wait,thread_clock=lambda:12)
        native_wait.assert_not_called(); self.assertEqual(proof['wait_calls'],0)
        self.assertTrue(proof['future_results_taken_only_after_ready'])
        self.assertEqual(owners['rear'].result(),'rear'); self.assertEqual(imu.result(),'imu')

    def test_native_targets_bounded_and_wait_includes_this_cycle_imu(self):
        owners,imu = self.futures(); clock=Clock(); targets=[]
        def native_wait(target):
            self.assertLessEqual(target-clock.now,200_000)
            targets.append(target); clock.now=target
            if len(targets)==1:
                for scope,future in owners.items(): future.set_result(scope)
            else: imu.set_result('fresh imu')
        with patch.object(bench,'wait',side_effect=AssertionError('No condition wait')):
            proof=bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                clock=clock,deadline_wait=native_wait,thread_clock=lambda:50)
        self.assertEqual(targets,[201000,401000]); self.assertEqual(proof['wait_calls'],2)
        self.assertEqual(proof['end_ns'],401000); self.assertTrue(imu.done())

    def test_poll_clips_to_deadline_and_ready_at_equality_is_rejected(self):
        owners,imu=self.futures(); clock=Clock(); targets=[]
        def native_wait(target):
            targets.append(target); clock.now=target
            if target==301000:
                for f in (*owners.values(),imu): f.set_result('done')
        with self.assertRaisesRegex(TimeoutError,'20 ms hard deadline'):
            bench._await_acquisition_ready(owners,imu,deadline_ns=301000,
                clock=clock,deadline_wait=native_wait)
        self.assertEqual(targets,[201000,301000])

    def test_each_ready_owner_error_preempts_pending_others(self):
        for failed in ('front','rear','imu'):
            with self.subTest(failed=failed):
                owners,imu=self.futures(); failure=OSError(failed+' native acquisition failed')
                (imu if failed=='imu' else owners[failed]).set_exception(failure); native_wait=Mock()
                with self.assertRaises(OSError) as caught:
                    bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                        clock=Clock(),deadline_wait=native_wait)
                self.assertIs(caught.exception,failure); native_wait.assert_not_called()

    def test_owner_error_after_native_cancellation_is_not_hidden(self):
        owners,imu=self.futures(); failure=OSError('current IMU read failed')
        def native_wait(target): imu.set_exception(failure); raise RuntimeError('native wait cancelled')
        with self.assertRaises(OSError) as caught:
            bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                clock=Clock(),deadline_wait=native_wait)
        self.assertIs(caught.exception,failure)

    def test_future_or_external_cancellation_does_not_wait_for_other_inputs(self):
        for future_cancel in (True,False):
            with self.subTest(future_cancel=future_cancel):
                owners,imu=self.futures(); clock=Clock(); calls=[]
                def native_wait(target):
                    calls.append(target); clock.now=target
                    if future_cancel: imu.cancel()
                def check():
                    if calls and not future_cancel: raise RuntimeError('signal cancellation')
                with self.assertRaisesRegex(RuntimeError,'cancel'):
                    bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                        clock=clock,deadline_wait=native_wait,check=check)
                self.assertEqual(len(calls),1)

    def test_callback_early_backdated_and_oversleep_are_rejected(self):
        for returned in ('early','backdated','late'):
            with self.subTest(returned=returned):
                owners,imu=self.futures(); clock=Clock()
                def native_wait(target):
                    clock.now={'early':target-1,'backdated':999,'late':1_000_000}[returned]
                    if returned=='late':
                        for f in (*owners.values(),imu): f.set_result('done')
                with self.assertRaises((ValueError,TimeoutError)):
                    bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                        clock=clock,deadline_wait=native_wait)

    def test_missing_non_future_aliased_and_invalid_deadline_are_rejected(self):
        owners,imu=self.futures(True)
        cases=[({'front':owners['front']},imu,1_000_000),
               ({'front':owners['front'],'rear':owners['front']},imu,1_000_000),
               (owners,owners['rear'],1_000_000),(owners,None,1_000_000),
               (owners,object(),1_000_000),(owners,imu,True),(owners,imu,0)]
        for buses,sample,deadline in cases:
            with self.subTest(deadline=deadline),self.assertRaises(ValueError):
                bench._await_acquisition_ready(buses,sample,deadline_ns=deadline,clock=Clock())

    def test_noncausal_ready_completion_wall_or_cpu_clock_is_rejected(self):
        owners,imu=self.futures(True)
        for clock,thread_clock in ((Mock(side_effect=[1000,1100,1099]),lambda:50),
                                   (Clock(),Mock(side_effect=[50,49]))):
            with self.assertRaisesRegex(ValueError,'Noncausal'):
                bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,
                    clock=clock,thread_clock=thread_clock)

    def test_finite_condition_fallback_still_waits_for_three_owners(self):
        owners,imu=self.futures(); clock=Clock()
        def complete(fs,*,timeout,return_when):
            self.assertEqual(set(fs),set(owners.values())|{imu})
            self.assertEqual(return_when,bench.FIRST_EXCEPTION); self.assertAlmostEqual(timeout,.000999)
            for f in fs: f.set_result('done')
            clock.now+=100; return set(fs),set()
        with patch.object(bench,'wait',side_effect=complete) as called:
            proof=bench._await_acquisition_ready(owners,imu,deadline_ns=1_000_000,clock=clock)
        self.assertEqual(called.call_count,1); self.assertEqual(proof['end_ns'],1100)


class NativeAcquisitionIntegrationTests(unittest.TestCase):
    def integration(self, *, native=True, release_voltage=False, device=None, check=lambda:None,
                    pipeline=True):
        started=(threading.Event(),threading.Event()); release=threading.Event()
        if release_voltage: release.set()
        sessions={scope:PreparedSession(started[i],release) for i,scope in enumerate(('front','rear'))}
        observer=OverlapObserver(started,release)
        options={'mode':'stop-proxy','cycles':1,'v3_voltage_proxy':True,
                 'v3_voltage_overlap':True,'v3_voltage_validation_overlap':True,
                 'v3_voltage_fast_pipeline':pipeline,'record_storage':'trace'}
        if native:
            options.update(absolute_epoch_cadence=True,
                deadline_wait=lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)))
        report,raw=bench.collect(sessions,device or Device(),observer,check=check,**options)
        return report,bench._serialize(raw),sessions,observer

    def test_selected_route_keeps_26_frames_and_final_14_timestamp_validation(self):
        report,rows,sessions,_=self.integration()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['input_acquisition_wait'],'native_ready_poll_200us.v1')
        proof=rows[0]['voltage_fast_pipeline']; wait=proof['acquisition_join_wait']
        self.assertLessEqual(wait['end_ns'],proof['feedback_join_ns'])
        self.assertLessEqual(wait['thread_cpu_begin_ns'],wait['thread_cpu_end_ns'])
        self.assertEqual(proof['stop_reply_count'],12)
        self.assertLess(proof['voltage_verified_ns'],proof['hard_deadline_ns'])
        self.assertEqual(sum(len(rows[0][phase][scope]['records'])
            for phase in ('acquired','voltage','output') for scope in ('front','rear')),26)
        self.assertTrue(all(s.phases==['feedback','voltage','output'] for s in sessions.values()))
        self.assertFalse(report['motor_enable_sent']); self.assertFalse(report['learned_targets_sent'])

    def test_ordinary_overlap_uses_both_native_waits_without_either_pipeline_variant(self):
        report,rows,sessions,_=self.integration(pipeline=False)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['input_acquisition_wait'],'native_ready_poll_200us.v1')
        self.assertEqual(report['voltage_join_wait'],'native_ready_poll_200us.v1')
        self.assertTrue(report['v3_voltage_overlap']['native_readiness_wait_enabled'])
        self.assertNotIn('v3_voltage_pipeline',report)
        self.assertNotIn('v3_voltage_fast_pipeline',report)
        proof=rows[0]['voltage_overlap']
        for name in ('acquisition_join_wait','voltage_join_wait'):
            self.assertEqual(proof[name]['mode'],'native_readiness_poll_v1')
            self.assertEqual(proof[name]['native_tick_max_us'],200)
        self.assertLess(proof['voltage_verified_ns'],proof['hard_deadline_ns'])
        self.assertEqual(sum(len(rows[0][phase][scope]['records'])
            for phase in ('acquired','voltage','output') for scope in ('front','rear')),26)
        self.assertTrue(all(s.phases==['feedback','voltage','output'] for s in sessions.values()))
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])

    def test_ordinary_overlap_acquisition_error_keeps_raw_evidence_and_never_infers(self):
        with patch.object(bench,'_await_acquisition_ready',side_effect=RuntimeError('acquisition cancelled')):
            report,rows,sessions,observer=self.integration(pipeline=False,release_voltage=True)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(observer.calls,0)
        self.assertEqual(set(rows[0]['acquired']),{'front','rear'})
        self.assertEqual(rows[0]['output'],{})
        self.assertEqual(rows[0]['voltage_overlap']['status'],'REJECTED_BEFORE_FEEDBACK_VALIDATION')
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_ordinary_overlap_takeout_crossing_deadline_cannot_reach_inference(self):
        original=bench._await_acquisition_ready
        def ready_then_delay(*args,**kwargs):
            result=original(*args,**kwargs);time.sleep(.021);return result
        with patch.object(bench,'_await_acquisition_ready',side_effect=ready_then_delay):
            report,rows,_,observer=self.integration(pipeline=False,release_voltage=True)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(observer.calls,0)
        self.assertEqual(rows[0]['output'],{})
        self.assertTrue(any('Acquisition result takeout' in error for error in report['errors']))

    def test_ordinary_overlap_without_native_callback_keeps_legacy_waits(self):
        with (patch.object(bench,'_await_acquisition_ready',side_effect=AssertionError('Legacy acquisition changed')),
              patch.object(bench,'_await_voltage_ready',side_effect=AssertionError('Legacy voltage changed'))):
            report,rows,_,_=self.integration(native=False,pipeline=False)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertFalse(report['v3_voltage_overlap']['native_readiness_wait_enabled'])
        self.assertNotIn('acquisition_join_wait',rows[0]['voltage_overlap'])
        self.assertNotIn('voltage_join_wait',rows[0]['voltage_overlap'])

    def test_join_failure_preserves_raw_inputs_but_never_runs_inference_or_proxy_output(self):
        with patch.object(bench,'_await_acquisition_ready',side_effect=RuntimeError('acquisition cancelled')):
            report,rows,sessions,observer=self.integration(release_voltage=True)
        self.assertEqual(report['status'],'ABORTED'); self.assertEqual(observer.calls,0)
        self.assertEqual(set(rows[0]['acquired']),{'front','rear'}); self.assertIn('imu',rows[0])
        self.assertEqual(rows[0]['output'],{})
        self.assertEqual(rows[0]['voltage_fast_pipeline']['status'],'REJECTED_BEFORE_FEEDBACK_VALIDATION')
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_ready_result_takeout_crossing_deadline_never_reaches_policy(self):
        original=bench._await_acquisition_ready
        def ready_then_delay(*args,**kwargs):
            result=original(*args,**kwargs)
            time.sleep(.021)
            return result
        with patch.object(bench,'_await_acquisition_ready',side_effect=ready_then_delay):
            report,rows,sessions,observer=self.integration(release_voltage=True)
        self.assertEqual(report['status'],'ABORTED'); self.assertEqual(observer.calls,0)
        self.assertEqual(rows[0]['output'],{})
        self.assertTrue(any('Acquisition result takeout' in error for error in report['errors']))
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_no_callback_route_keeps_original_acquisition_and_metadata(self):
        with patch.object(bench,'_await_acquisition_ready',side_effect=AssertionError('Legacy acquisition changed')):
            report,rows,_,_=self.integration(native=False)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['input_acquisition_wait'],'legacy_result_collection.v1')
        self.assertNotIn('acquisition_join_wait',rows[0]['voltage_fast_pipeline'])
        self.assertEqual(rows[0]['voltage_fast_pipeline']['stop_reply_count'],12)

    def test_cancellation_after_ready_boundary_stops_before_policy_and_output(self):
        cancelled=threading.Event(); original=bench._await_acquisition_ready
        def ready_then_cancel(*args,**kwargs):
            value=original(*args,**kwargs); cancelled.set(); return value
        def check():
            if cancelled.is_set(): raise RuntimeError('Operator cancelled after acquisition ready')
        with patch.object(bench,'_await_acquisition_ready',side_effect=ready_then_cancel):
            report,rows,_,observer=self.integration(release_voltage=True,check=check)
        self.assertEqual(report['status'],'ABORTED'); self.assertEqual(observer.calls,0)
        self.assertEqual(rows[0]['output'],{})
        self.assertTrue(any('cancelled after acquisition ready' in error for error in report['errors']))


if __name__=='__main__': unittest.main()
