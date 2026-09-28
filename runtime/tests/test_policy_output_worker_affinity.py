"""Affinity comparisons use per-thread fake masks and simulated time, no OS changes."""
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_output_runtime as runtime
from test_policy_output_runtime import FakeSession, FakeIMU, SimulatedClock, encode_motion, profile


class WorkerAffinityTests(unittest.TestCase):
    def run_case(self,*,full=None,selection=True,setup_failure=False,
                 setup_readback_failure=False,restore_failure=False,policy_failure=False):
        full=set(range(5)) if full is None else set(full)
        main=threading.get_ident();masks={};events=[];lock=threading.Lock()
        clock=SimulatedClock();sessions={s:FakeSession(i,clock=clock) for s,i in (('front',1),('rear',7))}
        def get_mask(_pid):
            self.assertEqual(_pid,0)
            with lock:return set(masks.get(threading.get_ident(),full))
        def set_mask(_pid,value):
            self.assertEqual(_pid,0)
            ident=threading.get_ident();name=threading.current_thread().name;value=set(value)
            with lock:
                restoring=ident!=main and value==full
                events.append((ident,name,value,restoring,sum(len(s.stop_times) for s in sessions.values())))
                if restoring and restore_failure and name.startswith('policy-front'):
                    raise OSError('Synthetic worker restore failure')
                if ident!=main and value==full-{4} and setup_readback_failure:
                    return  # Successful set call but wrong readback must still fail.
                masks[ident]=value
                if ident!=main and value==full-{4} and setup_failure and name.startswith('policy-front'):
                    raise OSError('Synthetic partial worker setup failure')
        class Startup:
            def pre_pin_warmup(self_inner):self.assertEqual(get_mask(0),full)
            def post_pin_prime(self_inner):self.assertEqual(get_mask(0),{4})
            def finish_startup(self_inner):self.assertEqual(get_mask(0),{4})
            def __call__(self_inner,*args):
                if policy_failure:raise RuntimeError('Synthetic policy failure')
                return (.04,)*12
        startup=Startup()
        kwargs={} if selection is None else {'exclude_policy_cpu_from_workers':selection}
        with patch.object(runtime.os,'sched_getaffinity',side_effect=get_mask,create=True), \
             patch.object(runtime.os,'sched_setaffinity',side_effect=set_mask,create=True):
            report=runtime.run_supported_policy(profile(),sessions,FakeIMU(clock=clock),startup,
                cancel_io=lambda:None,clock=clock,sleep=clock.sleep,encode_motion=encode_motion,
                startup_model=startup,main_thread_cpu=4,pre_cycle_policy_warmup_calls=10,
                post_pin_policy_prime_calls=10,**kwargs)
        return report,sessions,masks,events,main,full

    def test_default_preserves_original_worker_masks(self):
        report,sessions,masks,events,main,full=self.run_case(selection=None)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertFalse(report['worker_affinity']['enabled'])
        self.assertTrue(all(ident==main for ident,*_ in events))
        self.assertTrue(all(row['cpus']==sorted(full) for row in
                            report['main_thread_affinity']['worker_masks_after_pin'].values()))

    def test_selected_masks_restore_after_stop_on_same_owners(self):
        report,sessions,masks,events,main,full=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        state=report['worker_affinity']
        self.assertEqual(state['target_mask'],[0,1,2,3]);self.assertTrue(state['restored'])
        self.assertEqual(state['restore_errors'],[]);self.assertTrue(report['stop_confirmed'])
        for scope,before in state['workers_before'].items():
            self.assertEqual(state['workers_during'][scope]['native_tid'],before['native_tid'])
            self.assertEqual(state['workers_during'][scope]['cpus'],[0,1,2,3])
            self.assertEqual(state['workers_after'][scope]['native_tid'],before['native_tid'])
            self.assertEqual(state['workers_after'][scope]['cpus'],sorted(full))
        self.assertEqual(sum(restoring for *_,restoring,_ in events),3)
        self.assertTrue(all(stops==2 for *_,restoring,stops in events if restoring))
        self.assertTrue(all(mask==full for mask in masks.values()))

    def test_partial_setup_and_wrong_readback_restore_before_enable(self):
        for selection in ({'setup_failure':True},{'setup_readback_failure':True}):
            with self.subTest(selection=selection):
                report,sessions,masks,events,main,full=self.run_case(**selection)
                self.assertEqual(report['status'],'ABORTED')
                self.assertIn('I/O worker affinity setup failed',report['errors'][0])
                self.assertFalse(report['motor_enable_sent']);self.assertTrue(report['stop_confirmed'])
                self.assertTrue(report['worker_affinity']['restored'])
                self.assertTrue(all(mask==full for mask in masks.values()))
                self.assertTrue(all(stops==2 for *_,restoring,stops in events if restoring))

    def test_policy_failure_restores_even_after_emergency_cancellation(self):
        report,sessions,masks,events,main,full=self.run_case(policy_failure=True)
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
        self.assertIn('Synthetic policy failure',report['errors'][0])
        self.assertTrue(report['worker_affinity']['restored'])
        self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertTrue(all(stops==2 for *_,restoring,stops in events if restoring))

    def test_restore_failure_cannot_be_reported_as_completed_output(self):
        report,*_=self.run_case(restore_failure=True)
        self.assertEqual(report['status'],'ABORTED_WORKER_AFFINITY_RESTORE')
        self.assertTrue(report['stop_confirmed']);self.assertFalse(report['worker_affinity']['restored'])
        self.assertIn('Synthetic worker restore failure',str(report['worker_affinity']['restore_errors']))
        self.assertTrue(report['main_thread_affinity']['restored'])

    def test_too_few_other_cpus_rejects_before_enable_or_worker_mutation(self):
        report,sessions,masks,events,main,full=self.run_case(full={0,1,4})
        self.assertEqual(report['status'],'ABORTED');self.assertFalse(report['motor_enable_sent'])
        self.assertIn('at least three other available CPUs',report['errors'][0])
        self.assertTrue(all(ident==main for ident,*_ in events));self.assertTrue(report['stop_confirmed'])

    def test_invalid_selection_and_missing_r22_reject_before_hardware(self):
        for selection in (True,1,'yes',None):
            with self.subTest(selection=selection):
                sessions={s:FakeSession(i) for s,i in (('front',1),('rear',7))}
                with self.assertRaisesRegex(RuntimeError,'I/O worker CPU exclusion'):
                    runtime.run_supported_policy(profile(),sessions,FakeIMU(),lambda *_:(.04,)*12,
                        cancel_io=lambda:None,exclude_policy_cpu_from_workers=selection)
                self.assertTrue(all(not session.calls for session in sessions.values()))


if __name__=='__main__':unittest.main()
