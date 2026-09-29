"""Offline proof for immediate bus-owned voltage reads in the STOP proxy."""

import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device, Observer
from test_native_voltage_overlap import OverlapObserver, OverlapSession


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
        sessions={scope:OverlapSession(started[index],release,voltage_v=voltages[index])
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
        class BadFinalSession(OverlapSession):
            def exchange(self,wires):
                rows,stats=super().exchange(wires)
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


if __name__=='__main__':unittest.main()
