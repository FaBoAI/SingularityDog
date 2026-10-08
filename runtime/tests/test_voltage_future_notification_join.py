"""Current voltage Futures remain the proof; hints only reduce bounded waiting."""
from concurrent.futures import CancelledError, Future
import threading
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_output_runtime as runtime


START=1_000_000_000
DEADLINE=START+1_000_000


class Clock:
    def __init__(self):self.now=START
    def __call__(self):return self.now


class NotificationJoinTests(unittest.TestCase):
    def fixture(self):
        workers=runtime.BusWorkers.__new__(runtime.BusWorkers)
        clock=Clock();workers.clock=clock
        workers.aborted=threading.Event();workers.reason=None
        workers.emergency=Mock()
        futures={scope:Future() for scope in ('front','rear')}
        values={scope:object() for scope in futures}
        group=Mock();waiter=Mock()
        return workers,clock,futures,values,group,waiter

    def collect(self,workers,futures,group,waiter):
        with patch.object(runtime,'_owned_future_readiness_group',return_value=group) as create:
            value=workers.collect_voltage(futures,deadline_ns=DEADLINE,deadline_wait=waiter)
        return value,create

    def test_notification_is_hint_until_both_original_futures_are_ready(self):
        workers,clock,futures,values,group,waiter=self.fixture();calls=[]
        def notify(deadline):
            calls.append(deadline);clock.now+=10_000
            if len(calls)==1:futures['rear'].set_result(values['rear'])
            if len(calls)==3:futures['front'].set_result(values['front'])
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=notify
        result,create=self.collect(workers,futures,group,waiter)
        create.assert_called_once_with(waiter,tuple(futures.values()))
        self.assertEqual(result,values);self.assertEqual(calls,[DEADLINE]*3)
        group.close.assert_called_once();waiter.assert_not_called()
        workers.emergency.assert_not_called()
        self.assertEqual(workers.voltage_notification_groups,1)
        self.assertEqual(workers.voltage_notification_waits,3)

    def test_already_done_avoids_pipe_and_notification_registration(self):
        workers,_,futures,values,group,waiter=self.fixture()
        for scope in futures:futures[scope].set_result(values[scope])
        result,create=self.collect(workers,futures,group,waiter)
        self.assertEqual(result,values);create.assert_not_called()
        group.wait.assert_not_called();group.close.assert_not_called()

    def test_callback_registration_rechecks_failure_before_native_wait(self):
        workers,_,futures,_,group,waiter=self.fixture()
        error=ValueError('original voltage owner failed')
        def register(*_):futures['rear'].set_exception(error);return group
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=register), \
             self.assertRaises(ValueError) as caught:
            workers.collect_voltage(futures,deadline_ns=DEADLINE,deadline_wait=waiter)
        self.assertIs(caught.exception,error);group.wait.assert_not_called()
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_registration_cannot_extend_original_deadline(self):
        workers,clock,futures,_,group,waiter=self.fixture()
        def register(*_):clock.now=DEADLINE;return group
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=register), \
             self.assertRaisesRegex(TimeoutError,'Voltage join hard cycle deadline'):
            workers.collect_voltage(futures,deadline_ns=DEADLINE,deadline_wait=waiter)
        group.wait.assert_not_called();group.close.assert_called_once()

    def test_no_hint_timeout_keeps_pending_futures_pending(self):
        workers,clock,futures,_,group,waiter=self.fixture()
        def timeout(deadline):clock.now=deadline;return {'kind':'DEADLINE','actual_ns':clock.now}
        group.wait.side_effect=timeout
        with self.assertRaises(TimeoutError):self.collect(workers,futures,group,waiter)
        self.assertTrue(all(not future.done() for future in futures.values()))
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_late_hint_with_both_ready_is_not_a_late_success(self):
        workers,clock,futures,values,group,waiter=self.fixture()
        def late(_):
            clock.now=DEADLINE
            for scope in futures:futures[scope].set_result(values[scope])
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=late
        with self.assertRaises(TimeoutError):self.collect(workers,futures,group,waiter)
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_bad_hint_is_rejected_before_any_pending_result_takeout(self):
        for defect in ('kind','bool','backdated','future','early_deadline','not_dict'):
            workers,clock,futures,_,group,waiter=self.fixture()
            def bad(_):
                clock.now+=1_000;event={'kind':'NOTIFIED','actual_ns':clock.now}
                if defect=='kind':event['kind']='READY'
                if defect=='bool':event['actual_ns']=True
                if defect=='backdated':event['actual_ns']=START-1
                if defect=='future':event['actual_ns']=clock.now+1
                if defect=='early_deadline':event['kind']='DEADLINE'
                if defect=='not_dict':return (0,clock.now)
                return event
            group.wait.side_effect=bad
            with self.subTest(defect=defect),self.assertRaises(RuntimeError):
                self.collect(workers,futures,group,waiter)
            self.assertTrue(all(not future.done() for future in futures.values()))
            group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_owner_failure_has_priority_over_malformed_hint_and_cancel_error(self):
        for malformed in (False,True):
            workers,clock,futures,_,group,waiter=self.fixture()
            original=OSError('original rear validation failed')
            def failed(_):
                clock.now+=1_000;futures['rear'].set_exception(original)
                if malformed:return {'kind':'BAD','actual_ns':False}
                raise RuntimeError('native cancellation wake')
            group.wait.side_effect=failed
            with self.subTest(malformed=malformed),self.assertRaises(OSError) as caught:
                self.collect(workers,futures,group,waiter)
            self.assertIs(caught.exception,original);group.close.assert_called_once()
            workers.emergency.assert_called_once()

    def test_cancelled_original_future_is_not_replaced_by_hint(self):
        workers,clock,futures,_,group,waiter=self.fixture()
        def cancelled(_):
            clock.now+=1_000;futures['front'].cancel()
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=cancelled
        with self.assertRaises(CancelledError):self.collect(workers,futures,group,waiter)
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_cleanup_error_does_not_replace_original_owner_failure(self):
        workers,clock,futures,_,group,waiter=self.fixture();original=ValueError('original proof')
        def failed(_):
            clock.now+=1_000;futures['front'].set_exception(original)
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=failed;group.close.side_effect=OSError('cleanup problem')
        with self.assertRaises(ValueError) as caught:self.collect(workers,futures,group,waiter)
        self.assertIs(caught.exception,original)
        self.assertTrue(any('cleanup problem' in note for note in original.__notes__))
        workers.emergency.assert_called_once()

    def test_cleanup_is_inside_original_cycle_budget(self):
        workers,clock,futures,values,group,waiter=self.fixture()
        def notified(_):
            clock.now+=1_000
            for scope in futures:futures[scope].set_result(values[scope])
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=notified;group.close.side_effect=lambda:setattr(clock,'now',DEADLINE)
        with self.assertRaisesRegex(TimeoutError,'readiness cleanup hard cycle deadline'):
            self.collect(workers,futures,group,waiter)
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_cleanup_aborted_state_cannot_publish_voltage_proofs(self):
        workers,clock,futures,values,group,waiter=self.fixture()
        def notified(_):
            clock.now+=1_000
            for scope in futures:futures[scope].set_result(values[scope])
            return {'kind':'NOTIFIED','actual_ns':clock.now}
        group.wait.side_effect=notified
        group.close.side_effect=lambda:workers.aborted.set()
        with self.assertRaisesRegex(RuntimeError,'voltage readiness cleanup'):
            self.collect(workers,futures,group,waiter)
        group.close.assert_called_once();workers.emergency.assert_called_once()

    def test_legacy_callable_has_no_native_notification_capability(self):
        _,_,futures,_,_,waiter=self.fixture()
        self.assertIsNone(runtime._owned_future_readiness_group(waiter,tuple(futures.values())))

    def test_factory_failure_preserves_pending_futures_and_requests_stop(self):
        workers,_,futures,_,_,waiter=self.fixture()
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=ValueError('Bad optional ABI')), \
             self.assertRaisesRegex(ValueError,'Bad optional ABI'):
            workers.collect_voltage(futures,deadline_ns=DEADLINE,deadline_wait=waiter)
        self.assertTrue(all(not future.done() for future in futures.values()))
        workers.emergency.assert_called_once()

    def test_original_owner_failure_precedes_registration_error(self):
        workers,_,futures,_,_,waiter=self.fixture();original=OSError('original proof')
        def failed(*_):
            futures['rear'].set_exception(original)
            raise ValueError('registration error')
        with patch.object(runtime,'_owned_future_readiness_group',side_effect=failed), \
             self.assertRaises(OSError) as caught:
            workers.collect_voltage(futures,deadline_ns=DEADLINE,deadline_wait=waiter)
        self.assertIs(caught.exception,original);workers.emergency.assert_called_once()


if __name__=='__main__':unittest.main()
