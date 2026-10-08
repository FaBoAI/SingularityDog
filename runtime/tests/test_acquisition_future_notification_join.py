"""Three current acquisition owners are proof; native notifications are hints.

Deterministic clocks and original Futures establish ordering/error semantics,
not OS wake latency or a physical 20 ms loop. No devices or network are used.
"""
from concurrent.futures import CancelledError, Future, wait as real_wait
import threading
import unittest
from unittest.mock import Mock, PropertyMock, patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_output_runtime as runtime


START=1_000_000_000
DEADLINE=START+1_000_000


class Clock:
    def __init__(self):self.now=START
    def __call__(self):return self.now


class AcquisitionNotificationJoinTests(unittest.TestCase):
    def fixture(self):
        workers=runtime.BusWorkers.__new__(runtime.BusWorkers)
        clock=Clock();workers.clock=clock
        workers.aborted=threading.Event();workers.reason=None
        workers.emergency=Mock()
        futures={scope:Future() for scope in ('front','rear')};imu=Future()
        values={scope:object() for scope in futures};imu_value=object()
        group=Mock();waiter=Mock()
        return workers,clock,futures,imu,values,imu_value,group,waiter

    def collect(self,fixture,*,timing=None):
        workers,_,futures,imu,_,_,group,waiter=fixture
        with patch.object(runtime,'_owned_future_readiness_group',return_value=group) as create:
            value=workers.collect_acquisition(futures,imu,deadline_ns=DEADLINE,
                deadline_wait=waiter,timing=timing)
        return value,create

    def finish(self,fixture):
        _,_,futures,imu,values,imu_value,_,_=fixture
        for scope,future in futures.items():
            if not future.done():future.set_result(values[scope])
        if not imu.done():imu.set_result(imu_value)

    def successful_hint(self,fixture):
        def notified(deadline):
            self.assertEqual(deadline,DEADLINE)
            fixture[1].now+=10_000;self.finish(fixture)
            return {'kind':'NOTIFIED','actual_ns':fixture[1].now}
        return notified

    def test_three_original_futures_required_before_any_success_takeout(self):
        fixture=self.fixture();workers,clock,futures,imu,values,imu_value,group,waiter=fixture
        originals=(*futures.values(),imu);calls=[];taken=[]
        def notify(deadline):
            calls.append(deadline);clock.now+=10_000
            if len(calls)==1:futures['rear'].set_result(values['rear'])
            if len(calls)==2:futures['front'].set_result(values['front'])
            if len(calls)==4:imu.set_result(imu_value)
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=notify
        def takeout(name,future,original_result):
            def read(*args,**kwargs):
                self.assertTrue(all(f.done() for f in originals),'Hint read an unfinished owner')
                taken.append(name);return original_result(*args,**kwargs)
            return read
        with (patch.object(futures['front'],'result',side_effect=takeout('front',futures['front'],futures['front'].result)),
              patch.object(futures['rear'],'result',side_effect=takeout('rear',futures['rear'],futures['rear'].result)),
              patch.object(imu,'result',side_effect=takeout('imu',imu,imu.result)),
              patch.object(runtime,'wait',side_effect=AssertionError('Notification route cannot condition wait'))):
            (result,sample),create=self.collect(fixture)
        self.assertIs(sample,imu_value);self.assertEqual(result,values)
        create.assert_called_once_with(waiter,originals)
        self.assertEqual(calls,[DEADLINE]*4);self.assertEqual(taken,['front','rear','imu'])
        self.assertEqual(workers.acquisition_notification_groups,1)
        self.assertEqual(workers.acquisition_notification_waits,4)
        group.close.assert_called_once();waiter.assert_not_called();workers.emergency.assert_not_called()

    def test_all_ready_avoids_group_registration_and_native_wait(self):
        fixture=self.fixture();self.finish(fixture)
        (result,sample),create=self.collect(fixture)
        self.assertEqual(result,fixture[4]);self.assertIs(sample,fixture[5])
        create.assert_not_called();fixture[6].wait.assert_not_called();fixture[6].close.assert_not_called()
        fixture[7].assert_not_called();fixture[0].emergency.assert_not_called()

    def test_registration_that_completes_all_owners_is_rechecked_without_wait(self):
        fixture=self.fixture()
        def register(*_):self.finish(fixture);return fixture[6]
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=register):
            result,sample=fixture[0].collect_acquisition(fixture[2],fixture[3],
                deadline_ns=DEADLINE,deadline_wait=fixture[7])
        self.assertEqual(result,fixture[4]);self.assertIs(sample,fixture[5])
        fixture[6].wait.assert_not_called();fixture[6].close.assert_called_once()
        self.assertEqual(fixture[0].acquisition_notification_groups,1)
        self.assertEqual(getattr(fixture[0],'acquisition_notification_waits',0),0)

    def test_each_original_failure_precedes_registration_wait_and_result_takeout(self):
        for failed in ('front','rear','imu'):
            with self.subTest(failed=failed):
                fixture=self.fixture();original=OSError('Original '+failed+' owner failed')
                (fixture[3] if failed=='imu' else fixture[2][failed]).set_exception(original)
                with patch.object(runtime,'_owned_future_readiness_group') as create, \
                     self.assertRaises(OSError) as caught:
                    fixture[0].collect_acquisition(fixture[2],fixture[3],
                        deadline_ns=DEADLINE,deadline_wait=fixture[7])
                self.assertIs(caught.exception,original);create.assert_not_called()
                fixture[7].assert_not_called();fixture[0].emergency.assert_called_once()

    def test_registration_failure_keeps_original_owner_failure_priority(self):
        for failed in ('front','rear','imu'):
            with self.subTest(failed=failed):
                fixture=self.fixture();original=ValueError('Original '+failed+' failed')
                def register(*_):
                    (fixture[3] if failed=='imu' else fixture[2][failed]).set_exception(original)
                    raise OSError('Registration failed')
                with patch.object(runtime,'_owned_future_readiness_group',side_effect=register), \
                     self.assertRaises(ValueError) as caught:
                    fixture[0].collect_acquisition(fixture[2],fixture[3],
                        deadline_ns=DEADLINE,deadline_wait=fixture[7])
                self.assertIs(caught.exception,original);fixture[7].assert_not_called()
                fixture[0].emergency.assert_called_once()

    def test_registration_error_without_owner_failure_preserves_pending_inputs(self):
        fixture=self.fixture();original=OSError('Optional registration failed')
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=original), \
             self.assertRaises(OSError) as caught:
            fixture[0].collect_acquisition(fixture[2],fixture[3],
                deadline_ns=DEADLINE,deadline_wait=fixture[7])
        self.assertIs(caught.exception,original)
        self.assertTrue(all(not f.done() for f in (*fixture[2].values(),fixture[3])))
        fixture[0].emergency.assert_called_once()

    def test_registration_rechecks_original_deadline_and_aborted_state(self):
        for late in (False,True):
            with self.subTest(late=late):
                fixture=self.fixture()
                def register(*_):
                    if late:fixture[1].now=DEADLINE
                    else:fixture[0].aborted.set()
                    return fixture[6]
                with patch.object(runtime,'_owned_future_readiness_group',side_effect=register), \
                     self.assertRaises((RuntimeError,TimeoutError)):
                    fixture[0].collect_acquisition(fixture[2],fixture[3],
                        deadline_ns=DEADLINE,deadline_wait=fixture[7])
                fixture[6].wait.assert_not_called();fixture[6].close.assert_called_once()
                fixture[0].emergency.assert_called_once()

    def test_each_owner_failure_has_priority_over_hint_error_bad_event_and_lateness(self):
        for failed in ('front','rear','imu'):
            for kind in ('error','bad','late'):
                with self.subTest(failed=failed,kind=kind):
                    fixture=self.fixture();original=OSError('Current '+failed+' failed')
                    def notify(_):
                        fixture[1].now=DEADLINE if kind=='late' else START+1_000
                        (fixture[3] if failed=='imu' else fixture[2][failed]).set_exception(original)
                        if kind=='error':raise RuntimeError('Native cancellation wake')
                        return {'kind':'BAD' if kind=='bad' else 'NOTIFIED','actual_ns':fixture[1].now}
                    fixture[6].wait.side_effect=notify
                    with self.assertRaises(OSError) as caught:self.collect(fixture)
                    self.assertIs(caught.exception,original);fixture[6].close.assert_called_once()
                    fixture[0].emergency.assert_called_once()

    def test_cancelled_original_owner_is_not_replaced_by_hint(self):
        for cancelled in ('front','rear','imu'):
            with self.subTest(cancelled=cancelled):
                fixture=self.fixture()
                def notify(_):
                    fixture[1].now+=1_000
                    (fixture[3] if cancelled=='imu' else fixture[2][cancelled]).cancel()
                    return {'kind':'NOTIFIED','actual_ns':fixture[1].now}
                fixture[6].wait.side_effect=notify
                with self.assertRaises(CancelledError):self.collect(fixture)
                fixture[6].close.assert_called_once();fixture[0].emergency.assert_called_once()

    def test_invalid_notification_cannot_take_any_result(self):
        for defect in ('kind','bool','backdated','future','early_deadline','not_dict'):
            with self.subTest(defect=defect):
                fixture=self.fixture()
                def notify(_):
                    fixture[1].now+=1_000;event={'kind':'NOTIFIED','actual_ns':fixture[1].now}
                    if defect=='kind':event['kind']='READY'
                    if defect=='bool':event['actual_ns']=True
                    if defect=='backdated':event['actual_ns']=START-1
                    if defect=='future':event['actual_ns']=fixture[1].now+1
                    if defect=='early_deadline':event['kind']='DEADLINE'
                    if defect=='not_dict':return (0,fixture[1].now)
                    return event
                fixture[6].wait.side_effect=notify
                with patch.object(fixture[0],'collect',side_effect=AssertionError('No takeout')), \
                     patch.object(fixture[3],'result',side_effect=AssertionError('No IMU takeout')), \
                     self.assertRaises(RuntimeError):self.collect(fixture)
                self.assertTrue(all(not f.done() for f in (*fixture[2].values(),fixture[3])))
                fixture[6].close.assert_called_once();fixture[0].emergency.assert_called_once()

    def test_original_deadline_rejects_no_hint_timeout_and_late_all_ready_hint(self):
        for ready in (False,True):
            with self.subTest(ready=ready):
                fixture=self.fixture()
                def notify(deadline):
                    fixture[1].now=deadline
                    if ready:self.finish(fixture)
                    return {'kind':'NOTIFIED' if ready else 'DEADLINE','actual_ns':fixture[1].now}
                fixture[6].wait.side_effect=notify
                with patch.object(fixture[0],'collect',side_effect=AssertionError('No late takeout')), \
                     self.assertRaisesRegex(TimeoutError,'Input acquisition hard deadline'):self.collect(fixture)
                fixture[6].close.assert_called_once();fixture[0].emergency.assert_called_once()

    def test_aborted_after_hint_cannot_take_out_three_ready_values(self):
        fixture=self.fixture()
        def notify(_):
            self.finish(fixture);fixture[0].aborted.set()
            return {'kind':'NOTIFIED','actual_ns':fixture[1].now}
        fixture[6].wait.side_effect=notify
        with patch.object(fixture[0],'collect',side_effect=AssertionError('No aborted takeout')), \
             self.assertRaisesRegex(RuntimeError,'aborted'):self.collect(fixture)
        fixture[6].close.assert_called_once()

    def test_takeout_deadline_and_abort_still_reject_before_input_validation(self):
        for abort in (False,True):
            for target in ('can','imu'):
                with self.subTest(abort=abort,target=target):
                    fixture=self.fixture();fixture[6].wait.side_effect=self.successful_hint(fixture)
                    future=fixture[2]['rear'] if target=='can' else fixture[3];original=future.result
                    def read(*args,**kwargs):
                        result=original(*args,**kwargs)
                        if abort:fixture[0].aborted.set()
                        else:fixture[1].now=DEADLINE
                        return result
                    with patch.object(future,'result',side_effect=read), \
                         self.assertRaises((RuntimeError,TimeoutError)):self.collect(fixture)
                    fixture[6].close.assert_called_once();fixture[0].emergency.assert_called_once()

    def test_original_failure_survives_cleanup_error_with_note(self):
        fixture=self.fixture();original=ValueError('Original IMU failed')
        def notify(_):
            fixture[3].set_exception(original)
            return {'kind':'NOTIFIED','actual_ns':fixture[1].now}
        fixture[6].wait.side_effect=notify;fixture[6].close.side_effect=OSError('Cleanup failed')
        with self.assertRaises(ValueError) as caught:self.collect(fixture)
        self.assertIs(caught.exception,original)
        self.assertTrue(any('Acquisition readiness cleanup' in n and 'Cleanup failed' in n
                            for n in original.__notes__))
        fixture[0].emergency.assert_called_once()

    def test_cleanup_remains_inside_original_deadline_and_abort_guard(self):
        for abort in (False,True):
            with self.subTest(abort=abort):
                fixture=self.fixture();fixture[6].wait.side_effect=self.successful_hint(fixture)
                def close():
                    if abort:fixture[0].aborted.set()
                    else:fixture[1].now=DEADLINE
                fixture[6].close.side_effect=close
                with self.assertRaisesRegex((RuntimeError,TimeoutError),'acquisition readiness cleanup'):
                    self.collect(fixture)
                fixture[6].close.assert_called_once();fixture[0].emergency.assert_called_once()

    def test_original_acquisition_timing_order_and_cpu_end_include_cleanup(self):
        fixture=self.fixture();timing=runtime._PendingCycleTiming()
        timing.begin(0,START,START,None,None)
        fixture[6].wait.side_effect=self.successful_hint(fixture)
        fixture[6].close.side_effect=lambda:setattr(fixture[1],'now',fixture[1].now+5_000)
        with patch.object(runtime.time,'thread_time_ns',side_effect=[1000,6000]):
            self.collect(fixture,timing=timing)
        names=('combined_acquisition_wait_begin_ns','combined_acquisition_wait_end_ns',
               'feedback_collect_begin_ns','feedback_collect_end_ns','imu_wait_begin_ns','imu_wait_end_ns')
        self.assertEqual([getattr(timing,n) for n in names],[START]+[START+10_000]*5)
        self.assertEqual(fixture[1].now,START+15_000)
        self.assertEqual(timing.combined_acquisition_wait_cpu_begin_ns,1000)
        self.assertEqual(timing.combined_acquisition_wait_cpu_end_ns,6000)

    def test_failure_cpu_end_is_recorded_but_ready_stage_stays_unset(self):
        fixture=self.fixture();timing=runtime._PendingCycleTiming();timing.begin(0,START,START,None,None)
        fixture[6].wait.side_effect=OSError('Native hint failed')
        with patch.object(runtime.time,'thread_time_ns',side_effect=[1000,7000]), \
             self.assertRaises(OSError):self.collect(fixture,timing=timing)
        self.assertEqual(timing.combined_acquisition_wait_cpu_end_ns,7000)
        self.assertIsNone(timing.combined_acquisition_wait_end_ns)
        fixture[6].close.assert_called_once()

    def test_unavailable_group_retains_same_200us_poll_and_original_imu(self):
        fixture=self.fixture();workers,clock,futures,imu,values,imu_value,group,waiter=fixture;targets=[]
        def wait(target):
            self.assertLessEqual(target-clock.now,200_000);targets.append(target);clock.now=target
            if len(targets)==1:
                for scope,future in futures.items():future.set_result(values[scope])
            else:imu.set_result(imu_value)
        waiter.side_effect=wait
        with patch.object(runtime,'_owned_future_readiness_group',return_value=None) as create:
            result,sample=workers.collect_acquisition(futures,imu,deadline_ns=DEADLINE,deadline_wait=waiter)
        create.assert_called_once_with(waiter,(*futures.values(),imu))
        self.assertEqual(targets,[START+200_000,START+400_000])
        self.assertEqual(result,values);self.assertIs(sample,imu_value)
        group.wait.assert_not_called();group.close.assert_not_called()
        self.assertEqual(getattr(workers,'acquisition_notification_groups',0),0)

    def test_no_waiter_retains_one_finite_first_exception_wait_without_group(self):
        fixture=self.fixture();workers,clock,futures,imu,values,imu_value,_,_=fixture;originals=(*futures.values(),imu)
        def wait(inputs,*,timeout,return_when):
            self.assertEqual(inputs,originals);self.assertEqual(return_when,runtime.FIRST_EXCEPTION)
            self.assertEqual(timeout,.001);clock.now+=10_000;self.finish(fixture)
            return set(inputs),set()
        with patch.object(runtime,'wait',side_effect=wait) as waited, \
             patch.object(runtime,'_owned_future_readiness_group') as create:
            result,sample=workers.collect_acquisition(futures,imu,deadline_ns=DEADLINE)
        waited.assert_called_once();create.assert_not_called()
        self.assertEqual(result,values);self.assertIs(sample,imu_value)

    def test_generic_callable_and_nonexact_future_keep_legacy_capability(self):
        fixture=self.fixture();inputs=(*fixture[2].values(),fixture[3])
        self.assertIsNone(runtime._owned_future_readiness_group(fixture[7],inputs))
        waiter=active._OwnedActiveWaiter.__new__(active._OwnedActiveWaiter)
        waiter._owner=threading.current_thread();waiter._readiness_group=None;waiter._busy=threading.Lock()
        class CustomFuture(Future):pass
        with patch.object(active._OwnedActiveWaiter,'future_readiness_available',new_callable=PropertyMock,return_value=True):
            self.assertIsNone(runtime._owned_future_readiness_group(waiter,(inputs[0],inputs[1],CustomFuture())))
        with patch.object(active._OwnedActiveWaiter,'future_readiness_available',new_callable=PropertyMock,return_value=False):
            self.assertIsNone(runtime._owned_future_readiness_group(waiter,inputs))

    def test_invalid_aliased_owners_rejected_before_optional_registration(self):
        for invalid in ('missing','can-alias','imu-alias','nonfuture','bool-deadline'):
            with self.subTest(invalid=invalid):
                fixture=self.fixture();futures=dict(fixture[2]);imu=fixture[3];deadline=DEADLINE
                if invalid=='missing':del futures['rear']
                if invalid=='can-alias':futures['rear']=futures['front']
                if invalid=='imu-alias':imu=futures['front']
                if invalid=='nonfuture':imu=object()
                if invalid=='bool-deadline':deadline=True
                with patch.object(runtime,'_owned_future_readiness_group') as create, \
                     self.assertRaises(RuntimeError):
                    fixture[0].collect_acquisition(futures,imu,deadline_ns=deadline,deadline_wait=fixture[7])
                create.assert_not_called();fixture[7].assert_not_called();fixture[0].emergency.assert_called_once()


class AcquisitionNotificationReportTests(unittest.TestCase):
    def test_new_workers_initialize_both_acquisition_counters_without_io(self):
        workers=runtime.BusWorkers({'front':object(),'rear':object()},Mock())
        try:
            self.assertEqual(workers.acquisition_notification_groups,0)
            self.assertEqual(workers.acquisition_notification_waits,0)
            self.assertEqual(workers.journal,[]);self.assertIsNone(workers.stop_futures)
        finally:workers.close()

    def run_case(self,*,notifications):
        from test_policy_output_runtime import OutputRuntimeTests,SimulatedClock,FakeSession,FakeIMU,profile
        clock=SimulatedClock();gate=threading.Event();groups=[];waiting=[];coordinator=threading.current_thread()
        class GatedIMU(FakeIMU):
            def __call__(self):
                if threading.current_thread() is not coordinator and not gate.wait(1.):
                    raise AssertionError('Acquisition notification did not release test IMU')
                return super().__call__()
        class Group:
            def __init__(self,inputs):self.inputs=inputs;self.waits=0;self.closed=False
            def wait(self,deadline):
                self.assert_open();self.waits+=1;gate.set()
                _,unfinished=real_wait(self.inputs,timeout=1.)
                if unfinished:raise AssertionError('Test acquisition owners did not complete')
                return {'kind':'NOTIFIED','actual_ns':clock()}
            def assert_open(self):
                if self.closed:raise AssertionError('Closed acquisition group reused')
            def close(self):self.closed=True;gate.clear()
        def create(waiter,inputs):
            self.assertEqual(len(inputs),3)
            self.assertTrue(all(type(future) is Future for future in inputs))
            group=Group(inputs);groups.append(group);return group
        original_output=runtime.BusWorkers.collect_output
        def output(workers,*args,**kwargs):
            waiting[:]=args[0].values()
            try:return original_output(workers,*args,**kwargs)
            finally:waiting.clear()
        def native_wait(target):
            if waiting:
                _,unfinished=real_wait(waiting,timeout=1.)
                self.assertFalse(unfinished,'Test output owners did not complete')
            clock.advance_to(target)
        data=profile()
        imu=GatedIMU(clock=clock) if notifications else FakeIMU(clock=clock)
        with (patch.object(runtime,'_owned_future_readiness_group',side_effect=create if notifications else None) as created,
              patch.object(runtime.BusWorkers,'collect_output',side_effect=output,autospec=True)):
            if not notifications:created.return_value=None
            report,_=OutputRuntimeTests.run_case(self,profile_data=data,
                absolute_epoch_cadence=notifications,deadline_wait=native_wait if notifications else None,
                clock=clock,sleep=clock.sleep,front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),imu=imu)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        return report,groups,created

    def test_legacy_report_keeps_original_selection_and_zero_notification_counts(self):
        report,groups,created=self.run_case(notifications=False)
        self.assertEqual(report['input_acquisition_wait'],'all_inputs_first_exception.v1')
        self.assertEqual(report['acquisition_future_notification'],{
            'groups_created':0,'wait_calls':0,'original_futures_required':True,
            'absolute_deadlines_unchanged':True,'hardware_timing_improvement_proven':False})
        self.assertEqual(groups,[]);created.assert_not_called()

    def test_selected_report_counts_actual_groups_and_three_owner_hint_waits(self):
        report,groups,_=self.run_case(notifications=True)
        self.assertTrue(groups);self.assertTrue(all(group.closed for group in groups))
        self.assertEqual(report['input_acquisition_wait'],'native_future_notification_or_ready_poll_200us.v1')
        proof=report['acquisition_future_notification']
        self.assertEqual(proof['groups_created'],len(groups))
        self.assertEqual(proof['wait_calls'],sum(group.waits for group in groups))
        self.assertTrue(proof['original_futures_required']);self.assertTrue(proof['absolute_deadlines_unchanged'])
        self.assertFalse(proof['hardware_timing_improvement_proven'])
        self.assertTrue(report['stop_confirmed'])


if __name__=='__main__':unittest.main()
