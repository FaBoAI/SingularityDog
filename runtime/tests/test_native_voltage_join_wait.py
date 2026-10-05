"""No-device tests for bounded readiness before taking voltage-owner results."""
import threading
import time
import unittest
from concurrent.futures import Future, FIRST_EXCEPTION, ThreadPoolExecutor
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device
from test_native_voltage_fast_pipeline import PreparedSession
from test_native_voltage_overlap import OverlapObserver


class Clock:
    def __init__(self):self.now=1000
    def __call__(self):return self.now


class VoltageJoinWaitTests(unittest.TestCase):
    def futures(self,ready=False):
        values={scope:Future() for scope in ('front','rear')};validation=Future()
        if ready:
            for scope,future in values.items():future.set_result(scope)
            validation.set_result('validated')
        return values,validation

    def test_ready_results_not_taken_or_replaced_by_join(self):
        futures,validation=self.futures(True);clock=Clock()
        with patch.object(futures['front'],'result',side_effect=AssertionError('result must remain owner')),patch.object(futures['rear'],'result',side_effect=AssertionError('result must remain owner')),patch.object(validation,'result',side_effect=AssertionError('result must remain owner')):
            result=bench._await_voltage_ready(futures,validation,deadline_ns=1000000,clock=clock,thread_clock=lambda:123)
        self.assertEqual(result['mode'],'bounded_future_wait_v1');self.assertEqual(result['wait_calls'],0)
        self.assertEqual(futures['front'].result(),'front');self.assertEqual(validation.result(),'validated')

    def test_native_poll_targets_at_most_200us_and_all_validation_ready_required(self):
        futures,validation=self.futures();clock=Clock();targets=[]
        def native_wait(target):
            self.assertLessEqual(target-clock.now,200000);targets.append(target);clock.now=target
            if len(targets)==1:
                for scope,future in futures.items():future.set_result(scope)
            else:validation.set_result('validated')
        result=bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=native_wait,clock=clock,thread_clock=lambda:55)
        self.assertEqual(result['wait_calls'],2);self.assertEqual(result['mode'],'native_readiness_poll_v1')
        self.assertEqual(result['end_ns'],401000);self.assertTrue(validation.done())

    def test_final_native_poll_clips_to_absolute_deadline_and_rejects(self):
        futures,validation=self.futures();clock=Clock();targets=[]
        def wait(target):targets.append(target);clock.now=target
        with self.assertRaisesRegex(TimeoutError,'20 ms hard deadline'):
            bench._await_voltage_ready(futures,validation,deadline_ns=301000,deadline_wait=wait,clock=clock)
        self.assertEqual(targets,[201000,301000]);self.assertFalse(futures['front'].done())

    def test_published_error_does_not_wait_for_unfinished_other_bus(self):
        futures,validation=self.futures();error=ValueError('owner validation failed');futures['rear'].set_exception(error);wait=Mock()
        with self.assertRaises(ValueError)as caught:
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=Clock())
        self.assertIs(caught.exception,error);wait.assert_not_called();self.assertFalse(futures['front'].done())

    def test_error_after_poll_is_rethrown_without_another_wait(self):
        futures,validation=self.futures();clock=Clock();error=RuntimeError('native owner failed');calls=[]
        def wait(target):calls.append(target);clock.now=target;futures['front'].set_exception(error)
        with self.assertRaises(RuntimeError)as caught:
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=clock)
        self.assertIs(caught.exception,error);self.assertEqual(len(calls),1)

    def test_cancellation_check_stops_before_another_native_wait(self):
        futures,validation=self.futures();clock=Clock();calls=[];error=RuntimeError('cancelled')
        def check():
            if calls:raise error
        def wait(target):calls.append(target);clock.now=target
        with self.assertRaises(RuntimeError)as caught:
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=clock,check=check)
        self.assertIs(caught.exception,error);self.assertEqual(len(calls),1)

    def test_condition_fallback_wait_is_bounded_and_joins_validation(self):
        futures,validation=self.futures();clock=Clock()
        def wait(owners,*,timeout,return_when):
            self.assertEqual(set(owners),set(futures.values())|{validation});self.assertEqual(return_when,FIRST_EXCEPTION)
            self.assertAlmostEqual(timeout,.000999)
            for future in owners:future.set_result('done')
            clock.now+=100
            return set(owners),set()
        with patch.object(bench,'wait',side_effect=wait)as called:
            result=bench._await_voltage_ready(futures,validation,deadline_ns=1000000,clock=clock)
        self.assertEqual(called.call_count,1);self.assertEqual(result['end_ns'],1100)

    def test_already_ready_after_absolute_deadline_never_admitted(self):
        futures,validation=self.futures(True)
        with self.assertRaises(TimeoutError):bench._await_voltage_ready(futures,validation,deadline_ns=1000,clock=Clock())

    def test_clock_crossing_deadline_during_ready_check_never_admitted(self):
        futures,validation=self.futures(True);clock=Mock(side_effect=[1000,1100,2000])
        with self.assertRaises(TimeoutError):bench._await_voltage_ready(futures,validation,deadline_ns=2000,clock=clock)

    def test_cancelled_future_rejected_without_result_or_wait(self):
        futures,validation=self.futures();futures['front'].cancel();wait=Mock()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=Clock())
        wait.assert_not_called()

    def test_native_wait_cancellation_does_not_hide_published_owner_error(self):
        futures,validation=self.futures();clock=Clock();owner_error=ValueError('invalid voltage owner proof')
        def wait(target):
            futures['rear'].set_exception(owner_error)
            raise RuntimeError('native wait cancelled')
        with self.assertRaises(ValueError)as caught:
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=clock)
        self.assertIs(caught.exception,owner_error)

    def test_native_callback_early_or_backdated_return_rejected(self):
        for difference in (-1,-200000):
            futures,validation=self.futures();clock=Clock()
            def wait(target):clock.now=target+difference
            with self.subTest(difference=difference),self.assertRaisesRegex(ValueError,'before requested wake'):
                bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=clock)

    def test_native_oversleep_is_not_backdated_or_admitted_even_if_ready(self):
        futures,validation=self.futures();clock=Clock()
        def wait(target):
            clock.now=1000000
            for future in futures.values():future.set_result('done')
            validation.set_result('validated')
        with self.assertRaises(TimeoutError):
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,deadline_wait=wait,clock=clock)

    def test_aliased_owner_or_validation_future_rejected(self):
        futures,validation=self.futures(True)
        cases=[({'front':futures['front'],'rear':futures['front']},validation),(futures,futures['rear'])]
        for owners,checked in cases:
            with self.subTest(owners=owners),self.assertRaisesRegex(ValueError,'Distinct'):
                bench._await_voltage_ready(owners,checked,deadline_ns=1000000,clock=Clock())

    def test_noncausal_ready_completion_clock_rejected(self):
        futures,validation=self.futures(True)
        with self.assertRaisesRegex(ValueError,'Noncausal'):
            bench._await_voltage_ready(futures,validation,deadline_ns=1000000,clock=Mock(side_effect=[1000,1100,1099]))

    def test_exact_owner_set_and_deadline_required(self):
        futures,validation=self.futures(True)
        invalid=[({'front':futures['front']},validation,1000000),({'front':futures['front'],'wrong':futures['rear']},validation,1000000),(futures,object(),1000000),(futures,validation,True)]
        for owners,checked,deadline in invalid:
            with self.subTest(deadline=deadline),self.assertRaises(ValueError):
                bench._await_voltage_ready(owners,checked,deadline_ns=deadline,clock=Clock())

    def integration(self,wait=None,*,pipeline=True,validation_overlap=True,
                    clock=None,output_dispatch_trace=False):
        started=(threading.Event(),threading.Event());release=threading.Event()
        sessions={scope:PreparedSession(started[index],release)for index,scope in enumerate(('front','rear'))}
        policy=OverlapObserver(started,release)
        options={'mode':'stop-proxy','cycles':1,'v3_voltage_proxy':True,'v3_voltage_overlap':True,
                 'v3_voltage_validation_overlap':validation_overlap,
                 'v3_voltage_fast_pipeline':pipeline,'record_storage':'trace',
                 'output_dispatch_trace':output_dispatch_trace}
        if clock is not None:options['clock']=clock
        if wait is not None:options.update(absolute_epoch_cadence=True,deadline_wait=wait)
        report,raw=bench.collect(sessions,Device(),policy,**options)
        return report,bench._serialize(raw),sessions,policy

    def test_native_fast_route_records_join_proof_and_retains_final_14timestamp_gate(self):
        report,rows,sessions,policy=self.integration(lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)))
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors']);proof=rows[0]['voltage_fast_pipeline']
        self.assertEqual(proof['voltage_join_wait']['mode'],'native_readiness_poll_v1')
        self.assertLessEqual(proof['voltage_join_wait']['end_ns'],proof['voltage_join_ns'])
        self.assertLess(proof['voltage_verified_ns'],proof['hard_deadline_ns']);self.assertEqual(proof['stop_reply_count'],12)
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])

    def test_ordinary_overlap_voltage_join_failure_retains_both_owners_and_stops_output(self):
        with patch.object(bench,'_await_voltage_ready',side_effect=RuntimeError('synthetic readiness failure')):
            report,rows,sessions,policy=self.integration(
                lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)),pipeline=False)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(rows[0]['output'],{})
        self.assertEqual(set(rows[0]['voltage']),{'front','rear'});self.assertTrue(policy.invalid)
        self.assertEqual(rows[0]['voltage_overlap']['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_ordinary_overlap_current_dispatch_deadline_is_checked_after_ready_voltage(self):
        original=bench._verify_voltage_final_freshness
        calls=0
        def final_gate(*args,**kwargs):
            nonlocal calls
            calls+=1
            if calls==2: time.sleep(.021)
            return original(*args,**kwargs)
        with patch.object(bench,'_verify_voltage_final_freshness',side_effect=final_gate):
            report,rows,sessions,policy=self.integration(
                lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)),pipeline=False)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(rows[0]['output'],{})
        self.assertEqual(calls,2);self.assertTrue(policy.invalid)
        proof=rows[0]['voltage_overlap']
        self.assertEqual(proof['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertIn('final_gate_error',proof)
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_validation_traversal_cannot_certify_an_entry_time_after_clock_advanced(self):
        for pipeline in (False,True):
            for advance in (21_000_000,101_000_000):
                with self.subTest(pipeline=pipeline,advance=advance):
                    offset=0;gates=0
                    original_gate=bench._verify_voltage_final_freshness
                    original_proof=bench._check_voltage_proof
                    def clock(): return time.monotonic_ns()+offset
                    def gate(*args,**kwargs):
                        nonlocal gates
                        gates+=1;return original_gate(*args,**kwargs)
                    def proof(*args,**kwargs):
                        nonlocal offset
                        result=original_proof(*args,**kwargs)
                        if gates==(1 if pipeline else 2): offset+=advance
                        return result
                    with (patch.object(bench,'_verify_voltage_final_freshness',side_effect=gate),
                          patch.object(bench,'_check_voltage_proof',side_effect=proof)):
                        report,rows,sessions,policy=self.integration(
                            lambda target:time.sleep(max(0,(target-clock())/1e9)),
                            pipeline=pipeline,clock=clock)
                    self.assertEqual(report['status'],'ABORTED')
                    self.assertEqual(rows[0]['output'],{})
                    self.assertEqual(set(rows[0]['acquired']),{'front','rear'})
                    self.assertEqual(set(rows[0]['voltage']),{'front','rear'})
                    state=rows[0]['voltage_fast_pipeline' if pipeline else 'voltage_overlap']
                    self.assertEqual(state['status'],'REJECTED_BEFORE_PROXY_STOP')
                    self.assertIn('final_gate_error',state)
                    self.assertTrue(policy.invalid)
                    self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_clock_advance_after_validation_return_is_rejected_before_first_submit(self):
        offset=0;gates=0;original=bench._verify_voltage_final_freshness
        def clock(): return time.monotonic_ns()+offset
        def gate(*args,**kwargs):
            nonlocal gates,offset
            gates+=1;result=original(*args,**kwargs)
            if gates==2: offset+=21_000_000
            return result
        with patch.object(bench,'_verify_voltage_final_freshness',side_effect=gate):
            report,rows,sessions,policy=self.integration(
                lambda target:time.sleep(max(0,(target-clock())/1e9)),
                pipeline=False,clock=clock,output_dispatch_trace=True)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(rows[0]['output'],{})
        state=rows[0]['voltage_overlap']
        self.assertEqual(state['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertIn('proxy_submit_error',state);self.assertTrue(policy.invalid)
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_first_submit_clock_advance_retains_accepted_output_and_rejects_second_bus(self):
        offset=0;advanced=False
        def clock(): return time.monotonic_ns()+offset
        class PausingSubmitPool(ThreadPoolExecutor):
            def submit(self,fn,*args,**kwargs):
                nonlocal offset,advanced
                future=super().submit(fn,*args,**kwargs)
                if fn.__name__=='exchange' and not advanced:
                    future.result(timeout=.5)
                    advanced=True;offset+=21_000_000
                return future
        with patch.object(bench,'ThreadPoolExecutor',PausingSubmitPool):
            report,rows,sessions,policy=self.integration(
                lambda target:time.sleep(max(0,(target-clock())/1e9)),
                pipeline=False,clock=clock)
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(set(rows[0]['output']),{'front'})
        self.assertEqual(sessions['front'].phases,['feedback','voltage','output'])
        self.assertEqual(sessions['rear'].phases,['feedback','voltage'])
        self.assertIn('proxy_submit_error',rows[0]['voltage_overlap'])
        self.assertTrue(policy.invalid);self.assertNotIn('observed',rows[0])

    def test_late_recorded_worker_dispatch_cannot_complete_a_diagnostic(self):
        class LateStartSession(PreparedSession):
            def exchange(self,wires,**kwargs):
                records,stats=super().exchange(wires,**kwargs)
                if self.phases[-1]=='output':
                    # Model an owner that began after queueing for 21 ms.
                    # Every request keeps its write/read causal ordering.
                    for row in records:
                        for field in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                            setattr(row,field,getattr(row,field)+21_000_000)
                return records,stats
        with patch(__name__+'.PreparedSession',LateStartSession):
            report,rows,_,policy=self.integration(
                lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)),pipeline=False)
        self.assertEqual(report['status'],'ABORTED')
        state=rows[0]['voltage_overlap']
        self.assertEqual(state['status'],'PROXY_STOP_DISPATCH_DEADLINE_MISSED')
        self.assertTrue(all(start>=state['hard_deadline_ns']
                            for start in state['proxy_actual_start_ns_by_bus'].values()))
        self.assertEqual(set(rows[0]['output']),{'front','rear'})
        self.assertTrue(policy.invalid);self.assertNotIn('observed',rows[0])

    def test_ordinary_overlap_waits_without_validation_worker_and_keeps_actual_dispatch_gate(self):
        report,rows,_,_=self.integration(
            lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)),
            pipeline=False,validation_overlap=False)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        proof=rows[0]['voltage_overlap']
        self.assertEqual(proof['voltage_join_wait']['mode'],'native_readiness_poll_v1')
        self.assertLess(proof['voltage_verified_ns'],proof['hard_deadline_ns'])

    def test_join_failure_keeps_native_voltages_and_no_later_output(self):
        with patch.object(bench,'_await_voltage_ready',side_effect=RuntimeError('synthetic readiness failure')):
            report,rows,sessions,policy=self.integration(lambda target:time.sleep(max(0,(target-time.monotonic_ns())/1e9)))
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(rows[0]['output'],{})
        self.assertEqual(set(rows[0]['voltage']),{'front','rear'});self.assertTrue(policy.invalid)
        self.assertEqual(rows[0]['voltage_fast_pipeline']['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertTrue(all(s.phases==['feedback','voltage']for s in sessions.values()))

    def test_no_callback_route_keeps_legacy_join_metadata_and_gates(self):
        with patch.object(bench,'_await_voltage_ready',side_effect=AssertionError('legacy path must stay unchanged')):
            report,rows,sessions,_=self.integration()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertNotIn('voltage_join_wait',rows[0]['voltage_fast_pipeline'])
        self.assertEqual(rows[0]['voltage_fast_pipeline']['stop_reply_count'],12)

    def test_native_route_uses_existing_callback_without_new_serial_commands(self):
        calls=[]
        def native_wait(target):
            calls.append(target);time.sleep(max(0,(target-time.monotonic_ns())/1e9))
        report,rows,sessions,_=self.integration(native_wait)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(rows[0]['voltage_fast_pipeline']['voltage_join_wait']['mode'],'native_readiness_poll_v1')
        self.assertTrue(all(s.phases==['feedback','voltage','output']for s in sessions.values()))

if __name__=='__main__':unittest.main()
