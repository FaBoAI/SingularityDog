"""File-only tests for the opt-in, disabled-motor voltage overlap diagnostic."""

import copy
from concurrent.futures import ThreadPoolExecutor
import io
import json
import struct
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw.can_readonly import ATParser
from test_native_pipeline_benchmark import Device, Observer, Session


class OverlapSession(Session):
    def __init__(self, voltage_started, voltage_release, *, voltage_v=40.,
                 fail_voltage=None):
        super().__init__()
        self.voltage_started=voltage_started
        self.voltage_release=voltage_release
        self.voltage_v=voltage_v
        self.fail_voltage=fail_voltage
        self.phases=[]
        self.active=threading.Lock()

    def exchange(self,wires):
        if not self.active.acquire(blocking=False):
            raise AssertionError('Concurrent exchanges on one bus')
        try:
            kind=ATParser().feed(wires[0])[0].kind
            phase='voltage' if len(wires)==1 and kind==17 else (
                'feedback' if len(wires)==6 and self.calls==0 else 'output')
            self.phases.append(phase)
            if phase=='voltage':
                self.voltage_started.set()
                if not self.voltage_release.wait(.5):
                    raise TimeoutError('Voltage phase was never released')
            rows,stats=super().exchange(wires)
            if phase=='voltage':
                rows[0].rx[11:15]=struct.pack('<f',self.voltage_v)
                if self.fail_voltage=='missing':
                    rows[0].received=0
                    raise native.ExchangeError('synthetic missing voltage',rows,stats)
                if self.fail_voltage=='cancel':
                    rows[0].received=0
                    raise native.ExchangeError('synthetic voltage cancellation',rows,stats)
            return rows,stats
        finally:
            self.active.release()


class OverlapObserver(Observer):
    def __init__(self, voltage_started, voltage_release, *, fail=False):
        super().__init__()
        self.voltage_started=voltage_started
        self.voltage_release=voltage_release
        self.fail=fail
        self.inferred_while_voltage_pending=False
        self.snapshot=None

    def consume(self,snapshot):
        if not all(event.wait(.5) for event in self.voltage_started):
            raise AssertionError('Voltage workers did not start before inference')
        self.inferred_while_voltage_pending=not self.voltage_release.is_set()
        self.snapshot=copy.deepcopy(snapshot)
        self.voltage_release.set()
        if self.fail:
            raise RuntimeError('synthetic inference failure')
        return super().consume(snapshot)


class NativeVoltageOverlapTests(unittest.TestCase):
    def run_case(self, *, voltages=(40.,40.), failures=(None,None),
                 inference_failure=False):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        sessions={scope:OverlapSession(started[index],release,
                    voltage_v=voltages[index],fail_voltage=failures[index])
                  for index,scope in enumerate(('front','rear'))}
        observer=OverlapObserver(started,release,fail=inference_failure)
        report,rows=bench.collect(sessions,Device(),observer,mode='stop-proxy',
            cycles=1,v3_voltage_proxy=True,v3_voltage_overlap=True,
            record_storage='trace')
        return report,bench._serialize(rows),sessions,observer

    def test_inference_overlaps_two_serial_voltage_reads_and_retains_26_records(self):
        report,rows,sessions,observer=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertTrue(observer.inferred_while_voltage_pending)
        self.assertEqual(observer.snapshot['voltage_by_bus'],{})
        self.assertTrue(observer.snapshot['source_flags']['v3_voltage_overlap_pending_at_inference'])
        self.assertFalse(observer.snapshot['source_flags']['v3_voltage_cadence_proxy'])
        self.assertEqual(len(rows),1)
        row=rows[0]
        self.assertEqual(row['voltage_overlap']['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertLessEqual(row['voltage_overlap']['feedback_ready_ns'],
                             row['voltage_overlap']['voltage_verified_ns'])
        timing_fields=report['v3_voltage_overlap']['timing_fields']
        self.assertIn('feedback6_per_bus_and_imu_gather',timing_fields['acquisition_ms'])
        self.assertIn('excludes voltage replies',timing_fields['input_latest_reply_ns'])
        self.assertEqual(timing_fields['voltage_verified_ns'],
                         'records[].voltage_overlap.voltage_verified_ns')
        self.assertIn('voltage validation',timing_fields['whole_iteration_ms'])
        timing=report['measurements'][0]
        self.assertLess(timing['input_latest_reply_ns'],
                        max(row['voltage_overlap']['voltage_reply_end_ns_by_bus'].values()))
        self.assertLessEqual(row['voltage_overlap']['voltage_verified_ns'],timing['cycle_end_ns'])
        self.assertIn('observed',row)
        for scope in ('front','rear'):
            self.assertEqual(sessions[scope].phases,['feedback','voltage','output'])
            self.assertEqual(len(row['acquired'][scope]['records']),6)
            self.assertEqual(len(row['voltage'][scope]['records']),1)
            self.assertEqual(len(row['output'][scope]['records']),6)
            self.assertEqual(row['acquired'][scope]['stats']['writes'],6)
            self.assertEqual(row['voltage'][scope]['stats']['writes'],1)
            self.assertEqual(row['output'][scope]['stats']['writes'],6)
        self.assertEqual(sum(len(row[phase][scope]['records'])
                             for phase in ('acquired','voltage','output')
                             for scope in ('front','rear')),26)
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])

    def test_low_voltage_discards_inference_and_sends_no_proxy_stop(self):
        report,rows,sessions,observer=self.run_case(voltages=(34.,40.))
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Voltage outside 35..42' in error for error in report['errors']))
        self.assertTrue(observer.invalid)
        self.assertTrue(observer.inferred_while_voltage_pending)
        row=next(row for row in rows if row.get('cycle')==1)
        self.assertNotIn('observed',row)
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertEqual(row['voltage_overlap']['status'],'PENDING_AT_INFERENCE')
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_missing_or_cancelled_voltage_settles_both_workers_and_keeps_partial(self):
        for failure in ('missing','cancel'):
            with self.subTest(failure=failure):
                report,rows,sessions,observer=self.run_case(failures=(failure,None))
                self.assertEqual(report['status'],'ABORTED')
                self.assertTrue(observer.invalid)
                row=next(row for row in rows if row.get('cycle')==1)
                self.assertNotIn('observed',row)
                self.assertIn('front',row['voltage_errors_by_bus'])
                self.assertEqual(len(row['voltage']['rear']['records']),1)
                native_failures=[row for row in rows if row.get('failure_scope')=='front']
                self.assertEqual(len(native_failures),1)
                self.assertEqual(len(native_failures[0]['records']),1)
                self.assertTrue(all(session.phases==['feedback','voltage']
                                    for session in sessions.values()))

    def test_inference_exception_still_settles_and_retains_both_voltage_reads(self):
        report,rows,sessions,observer=self.run_case(inference_failure=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic inference failure' in error for error in report['errors']))
        row=next(row for row in rows if row.get('cycle')==1)
        self.assertNotIn('observed',row)
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))
        self.assertTrue(observer.invalid)

    def test_later_submit_rejection_keeps_feedback_from_every_accepted_bus(self):
        for rejection,accepted in ((5,('front',)),(6,('front','rear'))):
            with self.subTest(rejection=rejection):
                class RejectingPool(ThreadPoolExecutor):
                    def __init__(self,**options):
                        super().__init__(**options)
                        self.submissions=0

                    def submit(self,fn,*args,**kwargs):
                        self.submissions+=1
                        if self.submissions==rejection:
                            raise RuntimeError('synthetic cycle submit rejection')
                        return super().submit(fn,*args,**kwargs)

                started=(threading.Event(),threading.Event())
                release=threading.Event();release.set()
                sessions={scope:OverlapSession(started[index],release)
                          for index,scope in enumerate(('front','rear'))}
                observer=Observer()
                with patch.object(bench,'ThreadPoolExecutor',RejectingPool):
                    report,raw=bench.collect(sessions,Device(),observer,
                        mode='stop-proxy',cycles=1,v3_voltage_proxy=True,
                        v3_voltage_overlap=True,record_storage='trace')
                row=next(row for row in bench._serialize(raw) if row.get('cycle')==1)
                self.assertEqual(report['status'],'ABORTED')
                self.assertTrue(any('synthetic cycle submit rejection' in error
                                    for error in report['errors']))
                self.assertEqual(set(row['acquired']),set(accepted))
                self.assertEqual(set(row['voltage']),set(accepted))
                self.assertNotIn('observed',row)
                self.assertEqual(observer.calls,0)
                self.assertTrue(observer.invalid)
                for scope in accepted:
                    self.assertEqual(len(row['acquired'][scope]['records']),6)
                    self.assertEqual(len(row['voltage'][scope]['records']),1)
                    self.assertEqual(sessions[scope].phases,['feedback','voltage'])

    def test_dispatch_trace_marks_inference_cpu_before_voltage_validation(self):
        validation_cpu=[]
        original=bench._verify_voltage_after_inference
        def capture_validation(*args):
            validation_cpu.append(bench.time.thread_time_ns())
            return original(*args)
        with patch.object(bench,'_verify_voltage_after_inference',capture_validation):
            started=(threading.Event(),threading.Event())
            release=threading.Event()
            sessions={scope:OverlapSession(started[index],release)
                      for index,scope in enumerate(('front','rear'))}
            report,_=bench.collect(sessions,Device(),
                OverlapObserver(started,release),mode='stop-proxy',cycles=1,
                v3_voltage_proxy=True,v3_voltage_overlap=True,record_storage='trace',
                output_dispatch_trace=True,inference_thread_cpu_trace=True)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        fields=report['output_dispatch_trace']['fields']
        dispatch=report['output_dispatch_trace']['rows'][0]
        self.assertLessEqual(dispatch[fields.index('main_infer_thread_cpu_ns')],
                             validation_cpu[0])
        self.assertEqual(report['inference_thread_cpu_trace']['rows'][0][2]
                         <=validation_cpu[0],True)

    def test_flag_is_opt_in_and_501_means_one_startup_plus_500_steady(self):
        arguments=['--mode','stop-proxy','--v3-voltage-proxy',
                   '--v3-voltage-overlap','--record-storage','trace',
                   '--cycles','501','--startup-cycle-allowance','1']
        stream=io.StringIO()
        with patch('sys.stdout',stream):
            self.assertEqual(bench.main(arguments),0)
        plan=json.loads(stream.getvalue())
        self.assertTrue(plan['v3_voltage_overlap'])
        self.assertEqual(plan['requests_per_cycle'],26)
        self.assertEqual(plan['steady_cycles_requested'],500)
        self.assertEqual(plan['input_workers'],['front6+voltage1','rear6+voltage1','IMU'])
        for bad in (['--v3-voltage-overlap'],arguments[:-2],
                    [*arguments[:arguments.index('--cycles')],'--cycles','502',
                     '--startup-cycle-allowance','1']):
            with self.subTest(bad=bad),patch('sys.stderr',io.StringIO()):
                with self.assertRaises(SystemExit):bench.main(bad)

    def test_validation_future_starts_after_snapshot_overlaps_inference_and_joins_before_stop(self):
        snapshot_ready=threading.Event()
        validation_submitted=threading.Event()
        validation_started=threading.Event()
        inference_done=threading.Event()
        validation_release=threading.Event()
        validation_joined=threading.Event()
        freshness_checked=threading.Event()
        voltage_started=(threading.Event(),threading.Event())
        voltage_release=threading.Event()
        original_snapshot=bench.snapshot_from_records
        original_validate=bench._validate_voltage_during_inference
        original_verify=bench._verify_voltage_after_inference
        original_freshness=bench._verify_voltage_final_freshness

        def snapshot_spy(*args,**kwargs):
            value=original_snapshot(*args,**kwargs)
            if kwargs.get('expected_voltage_by_bus') is None:
                snapshot_ready.set()
            return value

        def verify_spy(*args,**kwargs):
            validation_started.set()
            return original_verify(*args,**kwargs)

        def validate_spy(*args,**kwargs):
            result=original_validate(*args,**kwargs)
            if not validation_release.wait(.5):
                raise TimeoutError('Validation join gate was never released')
            validation_joined.set()
            return result

        def freshness_spy(*args,**kwargs):
            result=original_freshness(*args,**kwargs)
            freshness_checked.set()
            return result

        class TrackingPool(ThreadPoolExecutor):
            def submit(self,fn,*args,**kwargs):
                if fn is validate_spy:
                    if not snapshot_ready.is_set():
                        raise AssertionError('Validation submitted before feedback snapshot')
                    validation_submitted.set()
                return super().submit(fn,*args,**kwargs)

        class GuardSession(OverlapSession):
            def exchange(self,wires):
                if len(wires)==6 and self.calls>0:
                    if not (validation_joined.is_set() and freshness_checked.is_set()):
                        raise AssertionError('STOP submitted before validation and freshness')
                return super().exchange(wires)

        class TrackingObserver(OverlapObserver):
            def consume(self,snapshot):
                if not validation_submitted.is_set():
                    raise AssertionError('Inference began before validation future submission')
                self.snapshot=copy.deepcopy(snapshot)
                self.inferred_while_voltage_pending=not voltage_release.is_set()
                voltage_release.set()
                if not validation_started.wait(.5):
                    raise TimeoutError('Validation did not overlap inference')
                inference_done.set()
                return Observer.consume(self,snapshot)

        sessions={scope:GuardSession(voltage_started[index],voltage_release)
                  for index,scope in enumerate(('front','rear'))}
        observer=TrackingObserver(voltage_started,voltage_release)
        result={}
        def run():
            try:
                result['value']=bench.collect(sessions,Device(),observer,
                    mode='stop-proxy',cycles=1,v3_voltage_proxy=True,
                    v3_voltage_overlap=True,v3_voltage_validation_overlap=True,
                    record_storage='trace')
            except BaseException as error:
                result['error']=error
        runner=threading.Thread(target=run,daemon=True)
        try:
            with (patch.object(bench,'snapshot_from_records',snapshot_spy),
                  patch.object(bench,'_verify_voltage_after_inference',verify_spy),
                  patch.object(bench,'_validate_voltage_during_inference',validate_spy),
                  patch.object(bench,'_verify_voltage_final_freshness',freshness_spy),
                  patch.object(bench,'ThreadPoolExecutor',TrackingPool)):
                runner.start()
                self.assertTrue(validation_submitted.wait(1),'Validation future not submitted')
                self.assertTrue(inference_done.wait(1),'Inference did not overlap validation')
                self.assertFalse(validation_joined.is_set())
                self.assertTrue(all('output' not in session.phases for session in sessions.values()))
                validation_release.set()
                runner.join(2)
        finally:
            voltage_release.set()
            validation_release.set()
            runner.join(2)
        self.assertFalse(runner.is_alive(),'Three-worker validation path deadlocked')
        if 'error' in result:raise result['error']
        report,raw=result['value']
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertTrue(observer.inferred_while_voltage_pending)
        self.assertTrue(freshness_checked.is_set())
        row=bench._serialize(raw)[0]
        proof=row['voltage_overlap']
        self.assertEqual(proof['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertLessEqual(proof['feedback_ready_ns'],proof['validation_started_ns'])
        self.assertLessEqual(proof['validation_started_ns'],proof['validation_finished_ns'])
        self.assertLessEqual(proof['validation_finished_ns'],proof['final_freshness_checked_ns'])
        self.assertLess(proof['validation_started_ns'],proof['inference_end_ns'])
        self.assertIn('observed',row)
        self.assertTrue(all(session.phases==['feedback','voltage','output']
                            for session in sessions.values()))

    def test_final_freshness_rejects_stale_voltage_after_worker_validation(self):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        validated=threading.Event()
        original_validate=bench._validate_voltage_during_inference

        def validated_worker(*args,**kwargs):
            result=original_validate(*args,**kwargs)
            validated.set()
            return result

        class SlowObserver(OverlapObserver):
            def consume(self,snapshot):
                release.set()
                if not validated.wait(.5):
                    raise TimeoutError('Voltage worker validation did not finish')
                time.sleep(bench.LIMIT_NS/1e9+.025)
                return Observer.consume(self,snapshot)

        sessions={scope:OverlapSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        observer=SlowObserver(started,release)
        with patch.object(bench,'_validate_voltage_during_inference',validated_worker):
            report,raw=bench.collect(sessions,Device(),observer,mode='stop-proxy',
                cycles=1,v3_voltage_proxy=True,v3_voltage_overlap=True,
                v3_voltage_validation_overlap=True,record_storage='trace')
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Expired feedback/voltage/IMU before proxy STOP' in error
                            for error in report['errors']),report['errors'])
        self.assertTrue(observer.invalid)
        row=next(row for row in bench._serialize(raw) if row.get('cycle')==1)
        self.assertIn('voltage_freshness_error',row)
        self.assertNotIn('observed',row)
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_validation_worker_error_retains_partial_trace_and_discards_observation(self):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        sessions={scope:OverlapSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        observer=OverlapObserver(started,release)
        with patch.object(bench,'_verify_voltage_after_inference',
                          side_effect=RuntimeError('synthetic worker validation failure')):
            report,raw=bench.collect(sessions,Device(),observer,mode='stop-proxy',
                cycles=1,v3_voltage_proxy=True,v3_voltage_overlap=True,
                v3_voltage_validation_overlap=True,record_storage='trace')
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic worker validation failure' in error
                            for error in report['errors']),report['errors'])
        self.assertTrue(observer.invalid)
        row=next(row for row in bench._serialize(raw) if row.get('cycle')==1)
        self.assertIn('synthetic worker validation failure',row['voltage_validation_error'])
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertTrue(all(len(row['voltage'][scope]['records'])==1
                            for scope in ('front','rear')))
        self.assertNotIn('observed',row)
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_inference_exception_joins_validation_future_and_retains_voltage_trace(self):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        inference_failed=threading.Event()
        validation_ready=threading.Event()
        validation_release=threading.Event()
        validation_done=threading.Event()
        original_validate=bench._validate_voltage_during_inference

        def held_validation(*args,**kwargs):
            result=original_validate(*args,**kwargs)
            validation_ready.set()
            if not validation_release.wait(.5):
                raise TimeoutError('Inference failure did not join validation')
            validation_done.set()
            return result

        class RejectingObserver(OverlapObserver):
            def consume(self,snapshot):
                release.set()
                inference_failed.set()
                raise RuntimeError('synthetic inference rejection')

        sessions={scope:OverlapSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        observer=RejectingObserver(started,release)
        result={}
        def run():
            try:
                result['value']=bench.collect(sessions,Device(),observer,
                    mode='stop-proxy',cycles=1,v3_voltage_proxy=True,
                    v3_voltage_overlap=True,v3_voltage_validation_overlap=True,
                    record_storage='trace')
            except BaseException as error:
                result['error']=error
        runner=threading.Thread(target=run,daemon=True)
        try:
            with patch.object(bench,'_validate_voltage_during_inference',held_validation):
                runner.start()
                self.assertTrue(inference_failed.wait(1))
                self.assertTrue(validation_ready.wait(1))
                self.assertTrue(runner.is_alive(),'Collector returned before joining validation')
                self.assertTrue(all('output' not in session.phases for session in sessions.values()))
                validation_release.set()
                runner.join(2)
        finally:
            release.set()
            validation_release.set()
            runner.join(2)
        self.assertFalse(runner.is_alive(),'Inference failure left validation unresolved')
        if 'error' in result:raise result['error']
        report,raw=result['value']
        self.assertTrue(validation_done.is_set())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic inference rejection' in error
                            for error in report['errors']),report['errors'])
        row=next(row for row in bench._serialize(raw) if row.get('cycle')==1)
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertNotIn('observed',row)
        self.assertEqual(row['output'],{})
        self.assertTrue(observer.invalid)

    def test_validation_submit_rejection_settles_bus_futures_after_feedback_snapshot(self):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        snapshot_ready=threading.Event()
        original_snapshot=bench.snapshot_from_records

        def snapshot_spy(*args,**kwargs):
            value=original_snapshot(*args,**kwargs)
            if kwargs.get('expected_voltage_by_bus') is None:
                snapshot_ready.set()
            return value

        class RejectValidationPool(ThreadPoolExecutor):
            def submit(self,fn,*args,**kwargs):
                if fn is bench._validate_voltage_during_inference:
                    release.set()
                    if not snapshot_ready.is_set():
                        raise AssertionError('Validation submit preceded feedback snapshot')
                    raise RuntimeError('synthetic validation submit rejection')
                return super().submit(fn,*args,**kwargs)

        sessions={scope:OverlapSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        observer=Observer()
        with (patch.object(bench,'snapshot_from_records',snapshot_spy),
              patch.object(bench,'ThreadPoolExecutor',RejectValidationPool)):
            report,raw=bench.collect(sessions,Device(),observer,mode='stop-proxy',
                cycles=1,v3_voltage_proxy=True,v3_voltage_overlap=True,
                v3_voltage_validation_overlap=True,record_storage='trace')
        self.assertTrue(snapshot_ready.is_set())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic validation submit rejection' in error
                            for error in report['errors']),report['errors'])
        row=next(row for row in bench._serialize(raw) if row.get('cycle')==1)
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual(set(row['voltage']),{'front','rear'})
        self.assertEqual(row['output'],{})
        self.assertNotIn('observed',row)
        self.assertEqual(observer.calls,0)
        self.assertTrue(observer.invalid)
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_validation_overlap_flag_requires_bounded_trace_overlap_and_cli_plans_it(self):
        sessions={scope:Session() for scope in ('front','rear')}
        observer=Observer()
        valid={'mode':'stop-proxy','cycles':1,'v3_voltage_proxy':True,
               'v3_voltage_overlap':True,'record_storage':'trace'}
        for options in ({'v3_voltage_overlap':False},
                        {'v3_voltage_validation_overlap':1},
                        {'record_storage':'objects'},
                        {'cycles':501}):
            with self.subTest(options=options),self.assertRaises(ValueError):
                bench.collect(sessions,Device(),observer,
                    **{**valid,'v3_voltage_validation_overlap':True,**options})
        self.assertTrue(all(session.calls==0 for session in sessions.values()))
        self.assertEqual(observer.calls,0)

        arguments=['--mode','stop-proxy','--v3-voltage-proxy',
                   '--v3-voltage-overlap','--v3-voltage-validation-overlap',
                   '--record-storage','trace','--cycles','1']
        stream=io.StringIO()
        with patch('sys.stdout',stream):
            self.assertEqual(bench.main(arguments),0)
        plan=json.loads(stream.getvalue())
        self.assertTrue(plan['v3_voltage_validation_overlap'])
        self.assertTrue(plan['v3_voltage_overlap'])
        self.assertEqual(plan['requests_per_cycle'],26)
        without_overlap=[arg for arg in arguments if arg!='--v3-voltage-overlap']
        with patch('sys.stderr',io.StringIO()):
            with self.assertRaises(SystemExit):bench.main(without_overlap)


if __name__=='__main__':
    unittest.main()
