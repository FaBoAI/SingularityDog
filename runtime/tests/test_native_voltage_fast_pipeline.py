"""Offline proof for immediate bus-owned voltage reads in the STOP proxy."""

import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device, Observer, Session
from test_native_voltage_overlap import OverlapObserver, OverlapSession


class PreparedSession(OverlapSession):
    def exchange(self,wires,*,before_native=None):
        if before_native is not None:before_native()
        return super().exchange(wires)


class WaitForVoltageDevice(Device):
    def __init__(self,started):
        super().__init__()
        self.started=started

    def read_sample(self):
        if not all(event.wait(.5) for event in self.started):
            raise TimeoutError('Voltage was not dispatched before IMU join')
        return super().read_sample()


class NativeVoltageFastPipelineTests(unittest.TestCase):
    def options(self,**extra):
        return {'mode':'stop-proxy','cycles':1,'v3_voltage_proxy':True,
                'v3_voltage_overlap':True,'v3_voltage_validation_overlap':True,
                'v3_voltage_fast_pipeline':True,'record_storage':'trace',**extra}

    def run_case(self,*,voltages=(40.,40.),inference_delay_s=0.,inference_failure=False):
        started=(threading.Event(),threading.Event())
        release=threading.Event()
        sessions={scope:PreparedSession(started[index],release,voltage_v=voltages[index])
                  for index,scope in enumerate(('front','rear'))}
        class DelayedObserver(OverlapObserver):
            def consume(self,snapshot):
                result=super().consume(snapshot)
                if inference_delay_s:time.sleep(inference_delay_s)
                return result
        policy=DelayedObserver(started,release,fail=inference_failure)
        report,raw=bench.collect(sessions,WaitForVoltageDevice(started),policy,
                                 **self.options())
        return report,bench._serialize(raw),sessions,policy

    def test_immediate_dispatch_precedes_global_feedback_join(self):
        report,rows,sessions,policy=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['v3_voltage_overlap']['voltage_dispatch_schedule'],
                         'after_each_bus_feedback')
        self.assertTrue(report['v3_voltage_fast_pipeline']['voltage_may_precede_global_feedback_validation'])
        self.assertEqual(report['v3_voltage_fast_pipeline']['period_ns'],20_000_000)
        self.assertFalse(report['learned_targets_sent'])
        self.assertFalse(report['motor_enable_sent'])
        self.assertTrue(policy.inferred_while_voltage_pending)
        row=rows[0];proof=row['voltage_fast_pipeline']
        self.assertEqual(proof['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertFalse(proof['output_allowed'])
        self.assertNotIn('voltage_pipeline',row)
        self.assertEqual(row['voltage_overlap']['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertLessEqual(proof['feedback_join_ns'],proof['feedback_snapshot_validated_ns'])
        self.assertLessEqual(proof['feedback_snapshot_validated_ns'],proof['inference_end_ns'])
        self.assertLessEqual(proof['voltage_join_ns'],proof['post_inference_verified_ns'])
        self.assertLessEqual(proof['post_inference_verified_ns'],proof['voltage_verified_ns'])
        self.assertLess(proof['voltage_verified_ns'],proof['hard_deadline_ns'])
        self.assertLessEqual(proof['voltage_verified_ns'],
                             min(value['records'][0]['start_ns'] for value in row['output'].values()))
        self.assertEqual(proof['stop_reply_count'],12)
        self.assertLessEqual(max(proof['stop_reply_end_ns_by_bus'].values()),
                             proof['stop_reply_verified_ns'])
        for scope in ('front','rear'):
            self.assertLessEqual(proof['feedback_dispatch_ns_by_bus'][scope],
                                 proof['feedback_reply_end_ns_by_bus'][scope])
            self.assertLessEqual(proof['feedback_reply_end_ns_by_bus'][scope],
                                 proof['feedback_ready_ns_by_bus'][scope])
            self.assertLessEqual(proof['feedback_ready_ns_by_bus'][scope],
                                 proof['voltage_dispatch_ns_by_bus'][scope])
            self.assertLessEqual(proof['voltage_dispatch_ns_by_bus'][scope],
                                 proof['feedback_published_ns_by_bus'][scope])
            self.assertLessEqual(proof['feedback_published_ns_by_bus'][scope],
                                 proof['feedback_join_ns'])
            self.assertLessEqual(proof['feedback_published_ns_by_bus'][scope],
                                 row['voltage'][scope]['records'][0]['start_ns'])
            self.assertLess(proof['voltage_dispatch_ns_by_bus'][scope],
                            proof['feedback_join_ns'])
            self.assertLessEqual(proof['voltage_dispatch_ns_by_bus'][scope],
                                 proof['voltage_reply_end_ns_by_bus'][scope])
            self.assertLessEqual(proof['voltage_reply_end_ns_by_bus'][scope],
                                 proof['voltage_join_ns'])
            self.assertEqual(sessions[scope].phases,['feedback','voltage','output'])
            self.assertEqual([len(row[phase][scope]['records'])
                              for phase in ('acquired','voltage','output')],[6,1,6])
        self.assertEqual(sum(len(row[phase][scope]['records'])
                             for phase in ('acquired','voltage','output')
                             for scope in ('front','rear')),26)

    def test_low_voltage_or_inference_failure_never_sends_later_stop_batch(self):
        for kwargs,needle in (({'voltages':(34.,40.)},'Voltage outside 35..42'),
                              ({'inference_failure':True},'synthetic inference failure')):
            with self.subTest(kwargs=kwargs):
                report,rows,sessions,policy=self.run_case(**kwargs)
                self.assertEqual(report['status'],'ABORTED')
                self.assertTrue(any(needle in error for error in report['errors']))
                self.assertTrue(policy.invalid)
                row=rows[0]
                self.assertEqual(row['output'],{})
                self.assertNotIn('observed',row)
                self.assertEqual(set(row['voltage']),{'front','rear'})
                self.assertTrue(all(session.phases==['feedback','voltage']
                                    for session in sessions.values()))

    def test_20ms_final_gate_rejects_delayed_inference(self):
        report,rows,sessions,policy=self.run_case(inference_delay_s=.025)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('20 ms hard deadline' in error for error in report['errors']))
        self.assertTrue(policy.invalid)
        row=rows[0];proof=row['voltage_fast_pipeline']
        self.assertEqual(proof['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertEqual(row['output'],{})
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_incomplete_final_stop_reply_aborts_and_keeps_trace(self):
        started=(threading.Event(),threading.Event());release=threading.Event()
        class BadFinalSession(PreparedSession):
            def exchange(self,wires,**kwargs):
                rows,stats=super().exchange(wires,**kwargs)
                if self.phases[-1]=='output':rows[0].received=0
                return rows,stats
        sessions={scope:BadFinalSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        policy=OverlapObserver(started,release)
        report,raw=bench.collect(sessions,WaitForVoltageDevice(started),policy,
                                 **self.options())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Invalid final proxy STOP record' in error
                            for error in report['errors']))
        row=bench._serialize(raw)[0]
        self.assertEqual(row['voltage_fast_pipeline']['status'],'FINAL_STOP_REPLY_REJECTED')
        self.assertEqual(set(row['output']),{'front','rear'})
        self.assertTrue(all(len(row['output'][scope]['records'])==6
                            for scope in ('front','rear')))

    def test_feedback_is_published_once_after_voltage_preparation(self):
        for failure in (None,'feedback','preparation','native'):
            with self.subTest(failure=failure):
                ready=Future();feedback=object();voltage=object();calls=[]
                def exchange(scope,wires,*,before_native=None):
                    if wires==('feedback',):
                        if failure=='feedback':raise RuntimeError('feedback failure')
                        return feedback
                    self.assertFalse(ready.done())
                    calls.append('voltage prepared')
                    if failure=='preparation':raise RuntimeError('preparation failure')
                    before_native()
                    self.assertIs(ready.result(),feedback)
                    calls.append('native')
                    if failure=='native':raise RuntimeError('native failure')
                    return voltage
                with patch.object(ready,'set_result',wraps=ready.set_result) as publish:
                    if failure is None:
                        self.assertIs(bench._feedback_then_voltage(exchange,'front',
                            ('feedback',),'voltage',ready,publish_before_native=True),voltage)
                    else:
                        with self.assertRaisesRegex(RuntimeError,failure+' failure'):
                            bench._feedback_then_voltage(exchange,'front',('feedback',),
                                'voltage',ready,publish_before_native=True)
                    self.assertTrue(ready.done())
                    if failure=='feedback':
                        self.assertEqual(publish.call_count,0)
                        self.assertIsInstance(ready.exception(),RuntimeError)
                    else:
                        self.assertEqual(publish.call_count,1)
                        self.assertIs(ready.result(),feedback)
                    self.assertEqual(calls,([] if failure=='feedback' else
                        ['voltage prepared'] if failure=='preparation' else
                        ['voltage prepared','native']))

    def test_voltage_preparation_failure_retains_all_feedback_without_output(self):
        class FailedPreparationSession(PreparedSession):
            def exchange(self,wires,*,before_native=None):
                if before_native is not None:
                    raise ValueError('synthetic changed voltage FD binding')
                return super().exchange(wires)
        sessions={scope:FailedPreparationSession(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        policy=Observer()
        report,raw=bench.collect(sessions,Device(),policy,**self.options())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('changed voltage FD binding' in error for error in report['errors']))
        row=bench._serialize(raw)[0]
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual(set(row['voltage_errors_by_bus']),{'front','rear'})
        self.assertEqual(row['voltage'],{})
        self.assertEqual(row['output'],{})
        self.assertTrue(policy.invalid)
        for scope in ('front','rear'):
            self.assertEqual(len(row['acquired'][scope]['records']),6)
            self.assertTrue(all(r['received']==17 for r in row['acquired'][scope]['records']))
            self.assertEqual(sessions[scope].phases,['feedback'])

    def test_publication_timestamp_failure_still_retains_completed_feedback(self):
        ready=Future();feedback=([SimpleNamespace(received_ns=2)],None)
        native_entered=[]
        def exchange(scope,wires,*,before_native=None):
            if before_native is None:return feedback
            before_native()
            native_entered.append(True)
        proof={name+'_ns_by_bus':{} for name in
               ('feedback_dispatch','feedback_reply_end','feedback_ready','voltage_dispatch')}
        clock=Mock(side_effect=[1,3,4,RuntimeError('publication clock failure')])
        with self.assertRaisesRegex(RuntimeError,'publication clock failure'):
            bench._feedback_then_voltage(exchange,'front',('feedback',),'voltage',
                ready,proof,clock,publish_before_native=True)
        self.assertIs(ready.result(timeout=.01),feedback)
        self.assertEqual(native_entered,[])

    def test_explicit_voltage_limit_propagates_through_all_overlap_routes(self):
        for route in ('overlap','worker','gated','fast'):
            for maximum,value,complete in ((42,42.247,False),(43,42.247,True),
                    (43,43.,True),(43,35.,True),(43,43.01,False),(43,34.99,False),
                    (43,float('nan'),False),(43,float('inf'),False)):
                with self.subTest(route=route,maximum=maximum,value=value):
                    started=(threading.Event(),threading.Event());release=threading.Event()
                    sessions={scope:PreparedSession(started[index],release,voltage_v=value)
                              for index,scope in enumerate(('front','rear'))}
                    policy=OverlapObserver(started,release)
                    report,raw=bench.collect(sessions,Device(),policy,**self.options(
                        v3_voltage_validation_overlap=route!='overlap',
                        v3_voltage_pipeline=route=='gated',
                        v3_voltage_fast_pipeline=route=='fast',voltage_max_v=maximum))
                    self.assertEqual(report['status'],
                        'COMPLETE_DIAGNOSTIC' if complete else 'ABORTED',report['errors'])
                    self.assertEqual(report['voltage_max_v'],maximum)
                    self.assertEqual(report['v3_voltage_overlap']['voltage_range_v'],[35.,maximum])
                    row=bench._serialize(raw)[0]
                    if complete:
                        self.assertEqual(row['voltage_overlap']['range_v'],[35.,maximum])
                        if route in ('gated','fast'):
                            key='voltage_pipeline' if route=='gated' else 'voltage_fast_pipeline'
                            self.assertEqual(row[key]['range_v'],[35.,maximum])
                    else:
                        self.assertEqual(row['output'],{})
                        self.assertTrue(all(s.phases==['feedback','voltage'] for s in sessions.values()))

    def test_final_freshness_rejects_voltage_changed_after_worker_validation(self):
        original=bench._validate_voltage_during_inference
        def changed(*args,**kwargs):
            full,start,end=original(*args,**kwargs)
            full['voltage_by_bus']['front']['value_v']=43.01
            return full,start,end
        started=(threading.Event(),threading.Event());release=threading.Event()
        sessions={scope:PreparedSession(started[index],release,voltage_v=42.247)
                  for index,scope in enumerate(('front','rear'))}
        with patch.object(bench,'_validate_voltage_during_inference',changed):
            report,raw=bench.collect(sessions,Device(),OverlapObserver(started,release),
                                     **self.options(voltage_max_v=43))
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Voltage changed before proxy STOP' in e for e in report['errors']))
        self.assertEqual(bench._serialize(raw)[0]['output'],{})

    def test_voltage_limit_cli_defaults_and_rejects_unapproved_limits(self):
        for flags,maximum in (([],42),(['--voltage-max-v','43'],43)):
            stream=io.StringIO()
            with patch('sys.stdout',stream):self.assertEqual(bench.main(flags),0)
            plan=json.loads(stream.getvalue())
            self.assertEqual(plan['voltage_max_v'],maximum)
            self.assertEqual(plan['voltage_range_v'],[35.,maximum])
        for value in ('41','44','nan','inf'):
            with self.subTest(value=value),patch('sys.stderr',io.StringIO()):
                with self.assertRaises(SystemExit):bench.main(['--voltage-max-v',value])
        for value in (41,44,True,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'Voltage maximum'):
                bench.collect({},Device(),Observer(),**self.options(voltage_max_v=value))

    def test_opt_in_flags_and_gated_route_are_distinct(self):
        flags=['--mode','stop-proxy','--v3-voltage-proxy','--v3-voltage-overlap',
               '--v3-voltage-validation-overlap','--v3-voltage-fast-pipeline',
               '--record-storage','trace','--cycles','5','--startup-cycle-allowance','1']
        stream=io.StringIO()
        with patch('sys.stdout',stream):self.assertEqual(bench.main(flags),0)
        plan=json.loads(stream.getvalue())
        self.assertTrue(plan['v3_voltage_fast_pipeline'])
        self.assertFalse(plan['v3_voltage_pipeline'])
        self.assertTrue(plan['v3_voltage_overlap'])
        self.assertTrue(plan['v3_voltage_validation_overlap'])
        self.assertEqual(plan['steady_cycles_requested'],4)
        self.assertEqual(plan['type1_requests_per_cycle'],0)
        for missing in ('--v3-voltage-proxy','--v3-voltage-overlap',
                        '--v3-voltage-validation-overlap','trace'):
            with self.subTest(missing=missing),patch('sys.stderr',io.StringIO()):
                with self.assertRaises(SystemExit):bench.main([x for x in flags if x!=missing])
        with patch('sys.stderr',io.StringIO()):
            with self.assertRaises(SystemExit):bench.main([*flags,'--v3-voltage-pipeline'])
        sessions={scope:OverlapSession(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        observer=Observer()
        for invalid in ({'v3_voltage_pipeline':True},
                        {'v3_voltage_fast_pipeline':1},
                        {'v3_voltage_validation_overlap':False}):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):
                bench.collect(sessions,Device(),observer,**self.options(**invalid))
        self.assertTrue(all(session.calls==0 for session in sessions.values()))

    def test_report_hash_binds_actual_records_bytes_on_startup_abort(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);out=root/'capture'
            flags=['--mode','stop-proxy','--v3-voltage-proxy','--v3-voltage-overlap',
                   '--v3-voltage-validation-overlap','--v3-voltage-fast-pipeline',
                   '--record-storage','trace','--cycles','1','--execute',
                   '--supported-disabled','--front-port','/dev/null',
                   '--rear-port','/dev/null','--expected-uids',str(root/'missing-uids.json'),
                   '--library',str(root/'missing.so'),'--calibration',str(root/'missing-cal.json'),
                   '--mount',str(root/'missing-mount.json'),'--bundle',str(root/'missing-bundle'),
                   '--output',str(out)]
            with patch('sys.stdout',io.StringIO()):self.assertEqual(bench.main(flags),2)
            report=json.loads((out/'report.json').read_text())
            self.assertEqual(report['status'],'ABORTED')
            self.assertEqual(report['v3_voltage_fast_pipeline']['records_sha256'],
                             hashlib.sha256((out/'records.json').read_bytes()).hexdigest())


def legacy_final_stop_verifier(output,clock):
    """Frozen pre-optimization verifier for byte-level acceptance parity."""
    if set(output)!=set(bench.dual.SCOPES):raise ValueError('Incomplete final proxy STOP buses')
    reply_ends={}
    for scope,ids in bench.dual.SCOPES.items():
        records,_=output[scope]
        if len(records)!=len(ids):raise ValueError('Incomplete final proxy STOP replies')
        for mid,row in zip(ids,records):
            if not (bytes(row.tx)==bench._STOP_WIRES[mid] and row.written==row.received==17 and
                    0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns):
                raise ValueError('Invalid final proxy STOP record')
            reply=bench._native_record_frame(bytes(row.rx))
            if (reply.flags!=4 or reply.can_id!=((2<<24)|(mid<<8)|0xfd) or
                    reply.data[:3]==b'\x00\xc4\x56'):
                raise ValueError('Invalid final proxy STOP reply')
        reply_ends[scope]=max(row.received_ns for row in records)
    verified_at=clock()
    if any(end>verified_at for end in reply_ends.values()):
        raise ValueError('Noncausal final proxy STOP reply')
    return reply_ends,verified_at


class FinalStopHeaderTests(unittest.TestCase):
    def setUp(self):
        self.output={scope:Session().exchange([bench._STOP_WIRES[mid] for mid in ids])
                     for scope,ids in bench.dual.SCOPES.items()}
        self.tick=max(row.received_ns for rows,_ in self.output.values() for row in rows)+1

    def result(self,fn):
        try:return fn(self.output,lambda:self.tick)
        except ValueError:return 'rejected'

    def test_canonical_all12_preserves_result_without_frame_objects(self):
        expected=self.result(legacy_final_stop_verifier)
        with patch.object(bench,'_native_record_frame',side_effect=AssertionError('Frame allocation')):
            self.assertEqual(self.result(bench._verify_final_proxy_stop_records),expected)
        self.assertNotEqual(expected,'rejected')

    def test_every_single_byte_value_has_identical_legacy_acceptance(self):
        row=self.output['front'][0][0]
        original=bytes(row.rx)
        for offset in range(17):
            for value in range(256):
                row.rx[offset]=value
                self.assertEqual(self.result(bench._verify_final_proxy_stop_records),
                                 self.result(legacy_final_stop_verifier),(offset,value))
            row.rx[offset]=original[offset]

    def test_version_prefix_on_every_axis_remains_rejected(self):
        for rows,_ in self.output.values():
            for row in rows:
                original=bytes(row.rx)
                row.rx[7:10]=b'\x00\xc4\x56'
                self.assertEqual(self.result(legacy_final_stop_verifier),'rejected')
                self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')
                row.rx[:]=original

    def test_record_count_tx_completeness_and_timestamp_guards_remain(self):
        row=self.output['rear'][0][-1]
        for name,value in (('written',16),('received',16),('start_ns',0),
                ('finish_ns',row.start_ns-1),('received_ns',row.deadline_ns)):
            original=getattr(row,name)
            setattr(row,name,value)
            self.assertEqual(self.result(legacy_final_stop_verifier),'rejected')
            self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')
            setattr(row,name,original)
        row.tx[0]=0
        self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')
        row.tx[0]=ord('A')
        self.tick=1
        self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')
        self.tick=max(r.received_ns for rows,_ in self.output.values() for r in rows)+1
        saved=self.output['rear']
        self.output['rear']=(saved[0][:-1],saved[1])
        self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')
        self.output.pop('rear')
        self.assertEqual(self.result(bench._verify_final_proxy_stop_records),'rejected')


if __name__=='__main__':unittest.main()
