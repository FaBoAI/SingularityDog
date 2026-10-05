"""Ordinary overlap publication order and complete failure evidence; no hardware."""
import ctypes as C
from concurrent.futures import Future
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device, Observer, Session
from test_native_voltage_overlap import OverlapObserver, OverlapSession


class OrdinaryVoltagePublicationTests(unittest.TestCase):
    def test_real_session_publishes_after_fd_and_argument_preparation_before_native(self):
        ready=Future();events=[];source=Session();expected=[]
        st=SimpleNamespace(st_dev=1,st_ino=2,st_rdev=3)
        def native_call(*args):
            phase='feedback' if args[5]==6 else 'voltage'
            events.append((phase+'_native',ready.done()))
            self.assertEqual(ready.done(),phase=='voltage')
            raw=bytes(args[4]);wires=tuple(raw[i:i+17] for i in range(0,len(raw),17))
            records,stats=source.exchange(wires)
            C.memmove(C.addressof(args[12]),C.addressof(records),C.sizeof(records))
            C.memmove(C.addressof(C.cast(args[13],C.POINTER(native.Stats)).contents),
                      C.addressof(stats),C.sizeof(stats))
            expected.append(native.exchange_evidence(records,stats))
            return 0
        library=SimpleNamespace(sd_exchange=Mock(side_effect=native_call))
        with (patch.object(native.os,'fstat',return_value=st),
              patch.object(native.os,'get_blocking',return_value=False)):
            session=native.NativeSession(library,123,first_id=1,cancel_fd=-1,stop_proxy=True)
            create_buffer=native.C.create_string_buffer
            def prepared_buffer(*args):
                events.append(('buffer',ready.done()))
                return create_buffer(*args)
            def fd_check(_):
                events.append(('fstat',ready.done()))
                return st
            ready.add_done_callback(lambda _:events.append(('published',ready.done())))
            with (patch.object(native.os,'fstat',side_effect=fd_check),
                  patch.object(native.C,'create_string_buffer',side_effect=prepared_buffer)):
                voltage=bench._feedback_then_voltage(
                    lambda scope,wires,**kw:session.exchange(wires,**kw),'front',
                    tuple(native.stop_wire(i) for i in range(1,7)),
                    bench._READ_WIRES[1,'voltage'],ready,publish_before_native=True)
        self.assertEqual(events,[('fstat',False),('buffer',False),('feedback_native',False),
                                 ('fstat',False),('buffer',False),('published',True),
                                 ('voltage_native',True)])
        self.assertEqual(native.exchange_evidence(*ready.result(timeout=.01)),expected[0])
        self.assertEqual(native.exchange_evidence(*voltage),expected[1])
        self.assertEqual(library.sd_exchange.call_count,2)
        self.assertFalse(session.poisoned)
        self.assertFalse(session.busy.locked())

    def test_ordinary_buses_publish_independently_after_preparation_and_keep_26_records(self):
        case=self;rear_preparation=threading.Event();rear_release=threading.Event()
        voltage_release=threading.Event();started=(threading.Event(),threading.Event())
        class PreparedSession(OverlapSession):
            def __init__(self,scope,*args):
                super().__init__(*args);self.scope=scope;self.ready=None
                self.published=threading.Event();self.feedback=None
            def exchange(self,wires,*,before_native=None):
                if len(wires)==1:
                    case.assertTrue(callable(before_native),'ordinary voltage omitted native hook')
                    case.assertFalse(self.ready.done(),'feedback published before voltage preparation')
                    if self.scope=='rear':
                        rear_preparation.set()
                        if not rear_release.wait(.5):raise TimeoutError('Rear preparation not released')
                    def publish():
                        before_native()
                        case.assertIs(self.ready.result(),self.feedback)
                        self.published.set()
                    return super().exchange(wires,before_native=publish)
                result=super().exchange(wires,before_native=before_native)
                if self.calls==1:self.feedback=result
                return result
        sessions={scope:PreparedSession(scope,started[i],voltage_release)
                  for i,scope in enumerate(('front','rear'))}
        original=bench._feedback_then_voltage
        def owner(exchange,scope,wires,voltage,ready,*args,**kw):
            sessions[scope].ready=ready
            return original(exchange,scope,wires,voltage,ready,*args,**kw)
        policy=OverlapObserver(started,voltage_release);result={}
        def run():
            result['value']=bench.collect(sessions,Device(),policy,mode='stop-proxy',cycles=1,
                v3_voltage_proxy=True,v3_voltage_overlap=True,record_storage='trace')
        runner=threading.Thread(target=run,daemon=True)
        with patch.object(bench,'_feedback_then_voltage',side_effect=owner):
            try:
                runner.start()
                self.assertTrue(rear_preparation.wait(1))
                self.assertTrue(sessions['front'].published.wait(1))
                self.assertFalse(sessions['rear'].ready.done())
                self.assertFalse(started[1].is_set())
                rear_release.set();runner.join(2)
            finally:
                rear_release.set();voltage_release.set();runner.join(2)
        self.assertFalse(runner.is_alive(),'ordinary publication left an unresolved owner')
        report,raw=result['value'];self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['v3_voltage_overlap']['voltage_dispatch_schedule'],'after_each_bus_feedback')
        self.assertEqual(report['v3_voltage_overlap']['feedback_publication'],'after_voltage_native_preparation')
        self.assertNotIn('v3_voltage_fast_pipeline',report);self.assertNotIn('v3_voltage_pipeline',report)
        row=bench._serialize(raw)[0]
        self.assertEqual(row['voltage_overlap']['feedback_publication'],'after_voltage_native_preparation')
        self.assertEqual(sum(len(row[phase][scope]['records']) for phase in ('acquired','voltage','output')
                             for scope in ('front','rear')),26)
        for scope in sessions:
            self.assertEqual(row['acquired'][scope],native.exchange_evidence(*sessions[scope].feedback))
            self.assertEqual(sessions[scope].phases,['feedback','voltage','output'])
        for flag in ('motor_enable_sent','learned_targets_sent','approved_for_runtime','full_controller_50Hz_verified'):
            self.assertIs(report[flag],False)

    def test_preparation_failure_publishes_received_feedback_without_deadlock_or_output(self):
        case=self
        class FailedPreparation(OverlapSession):
            def exchange(self,wires,*,before_native=None):
                if len(wires)==1:
                    case.assertTrue(callable(before_native))
                    raise ValueError('synthetic voltage FD preparation failure')
                result=super().exchange(wires,before_native=before_native)
                self.feedback=result
                return result
        sessions={scope:FailedPreparation(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        policy=Observer();result={}
        def run():
            result['value']=bench.collect(sessions,Device(),policy,mode='stop-proxy',cycles=1,
                v3_voltage_proxy=True,v3_voltage_overlap=True,record_storage='trace')
        runner=threading.Thread(target=run,daemon=True);runner.start();runner.join(2)
        self.assertFalse(runner.is_alive(),'preparation failure left feedback unpublished')
        report,raw=result['value'];row=bench._serialize(raw)[0]
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(policy.invalid)
        self.assertTrue(any('FD preparation failure' in e for e in report['errors']))
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual(set(row['voltage_errors_by_bus']),{'front','rear'})
        self.assertEqual(row['voltage'],{});self.assertEqual(row['output'],{})
        self.assertNotIn('observed',row)
        for scope in sessions:
            self.assertEqual(row['acquired'][scope],native.exchange_evidence(*sessions[scope].feedback))
            self.assertEqual(sessions[scope].phases,['feedback'])

    def test_native_failure_after_publication_keeps_original_partial_voltage_and_other_bus(self):
        class CapturedFailure(OverlapSession):
            def exchange(self,wires,*,before_native=None):
                try:return super().exchange(wires,before_native=before_native)
                except native.ExchangeError as error:
                    self.failed=error
                    raise
        release=threading.Event();release.set()
        sessions={'front':CapturedFailure(threading.Event(),release,fail_voltage='missing'),
                  'rear':OverlapSession(threading.Event(),release)}
        policy=Observer()
        report,raw=bench.collect(sessions,Device(),policy,mode='stop-proxy',cycles=1,
            v3_voltage_proxy=True,v3_voltage_overlap=True,record_storage='trace')
        rows=bench._serialize(raw);row=next(r for r in rows if r.get('cycle')==1)
        failed=next(r for r in rows if r.get('failure_scope')=='front')
        original=native.exchange_evidence(sessions['front'].failed.records,sessions['front'].failed.stats)
        self.assertEqual({k:failed[k] for k in original},original)
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(policy.invalid)
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual(len(row['voltage']['rear']['records']),1)
        self.assertEqual(row['output'],{});self.assertNotIn('observed',row)
        self.assertEqual(original['records'][0]['received'],0)
        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))


if __name__=='__main__':unittest.main()
