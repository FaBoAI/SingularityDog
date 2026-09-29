"""Offline tests for the gated feedback-then-voltage STOP-proxy diagnostic."""

import io
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
import tempfile
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw.can_readonly import ATParser
from test_native_pipeline_benchmark import Device, Observer
from test_native_voltage_overlap import OverlapObserver, OverlapSession


class GatedSession(OverlapSession):
    def __init__(self,*args,snapshot_ready=None,**kwargs):
        super().__init__(*args,**kwargs)
        self.snapshot_ready=snapshot_ready

    def exchange(self,wires):
        if (len(wires)==1 and ATParser().feed(wires[0])[0].kind==17 and
                self.snapshot_ready is not None and not self.snapshot_ready.is_set()):
            raise AssertionError('Voltage dispatched before complete feedback/IMU snapshot')
        return super().exchange(wires)


class NativeVoltagePipelineTests(unittest.TestCase):
    def options(self,**extra):
        return {'mode':'stop-proxy','cycles':1,'v3_voltage_proxy':True,
                'v3_voltage_overlap':True,'v3_voltage_validation_overlap':True,
                'v3_voltage_pipeline':True,'record_storage':'trace',**extra}

    def run_case(self,*,voltages=(40.,40.),observer_failure=False,
                 inference_delay_s=0.):
        snapshot_ready=threading.Event()
        voltage_started=(threading.Event(),threading.Event())
        voltage_release=threading.Event()
        sessions={scope:GatedSession(voltage_started[index],voltage_release,
                    voltage_v=voltages[index],snapshot_ready=snapshot_ready)
                  for index,scope in enumerate(('front','rear'))}
        class DelayedObserver(OverlapObserver):
            def consume(self,snapshot):
                result=super().consume(snapshot)
                if inference_delay_s:time.sleep(inference_delay_s)
                return result
        observer=DelayedObserver(voltage_started,voltage_release,fail=observer_failure)
        original=bench.snapshot_from_records

        def snapshot(*args,**kwargs):
            value=original(*args,**kwargs)
            if kwargs.get('expected_voltage_by_bus') is None:snapshot_ready.set()
            return value

        with patch.object(bench,'snapshot_from_records',snapshot):
            report,raw=bench.collect(sessions,Device(),observer,**self.options())
        return report,bench._serialize(raw),sessions,observer

    def test_pipeline_gates_voltage_after_feedback_snapshot_and_before_stop(self):
        report,rows,sessions,observer=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['v3_voltage_overlap']['voltage_dispatch_schedule'],
                         'after_complete_feedback_imu_snapshot')
        self.assertEqual(report['v3_voltage_pipeline']['period_ns'],20_000_000)
        self.assertFalse(report['learned_targets_sent'])
        self.assertFalse(report['motor_enable_sent'])
        self.assertTrue(observer.inferred_while_voltage_pending)
        row=rows[0];proof=row['voltage_pipeline']
        self.assertEqual(proof['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertFalse(proof['output_allowed'])
        self.assertEqual(row['voltage_overlap']['status'],'VALIDATED_BEFORE_PROXY_STOP')
        self.assertLessEqual(proof['feedback_join_ns'],proof['voltage_gate_set_ns'])
        self.assertLessEqual(proof['voltage_gate_set_ns'],proof['inference_end_ns'])
        self.assertLessEqual(proof['voltage_join_ns'],proof['voltage_verified_ns'])
        self.assertLessEqual(proof['voltage_verified_ns'],
                             min(entry['records'][0]['start_ns'] for entry in row['output'].values()))
        for scope in ('front','rear'):
            self.assertLessEqual(proof['feedback_dispatch_ns_by_bus'][scope],
                                 proof['feedback_reply_end_ns_by_bus'][scope])
            self.assertLessEqual(proof['feedback_reply_end_ns_by_bus'][scope],
                                 proof['feedback_ready_ns_by_bus'][scope])
            self.assertLessEqual(proof['feedback_ready_ns_by_bus'][scope],
                                 proof['feedback_join_ns'])
            self.assertLessEqual(proof['voltage_gate_set_ns'],
                                 proof['voltage_dispatch_ns_by_bus'][scope])
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

    def test_snapshot_failure_releases_waiting_owners_without_voltage_or_output(self):
        sessions={scope:GatedSession(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        observer=Observer()
        with patch.object(bench,'snapshot_from_records',
                          side_effect=ValueError('synthetic feedback rejection')):
            report,raw=bench.collect(sessions,Device(),observer,**self.options())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(observer.invalid)
        row=bench._serialize(raw)[0]
        self.assertEqual(row['voltage_pipeline']['status'],'PENDING_AT_FEEDBACK')
        self.assertEqual(row['voltage'],{})
        self.assertEqual(row['output'],{})
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual([session.phases for session in sessions.values()],
                         [['feedback'],['feedback']])

    def test_low_voltage_and_inference_error_fail_closed(self):
        for kwargs,needle in (({'voltages':(34.,40.)},'Voltage outside 35..42'),
                              ({'observer_failure':True},'synthetic inference failure')):
            with self.subTest(kwargs=kwargs):
                report,rows,sessions,observer=self.run_case(**kwargs)
                self.assertEqual(report['status'],'ABORTED')
                self.assertTrue(any(needle in error for error in report['errors']))
                self.assertTrue(observer.invalid)
                row=rows[0]
                self.assertNotIn('observed',row)
                self.assertEqual(row['output'],{})
                self.assertEqual(set(row['voltage']),{'front','rear'})
                self.assertTrue(all(session.phases==['feedback','voltage']
                                    for session in sessions.values()))

    def test_delayed_inference_rechecks_20ms_deadline_at_proxy_gate(self):
        report,rows,sessions,observer=self.run_case(inference_delay_s=.025)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('20 ms hard deadline' in error for error in report['errors']))
        self.assertTrue(observer.invalid)
        row=rows[0];proof=row['voltage_pipeline']
        self.assertEqual(proof['status'],'REJECTED_BEFORE_PROXY_STOP')
        self.assertGreaterEqual(proof['post_inference_verified_ns'],
                                proof['hard_deadline_ns'])
        self.assertEqual(row['output'],{})
        self.assertTrue(all(session.phases==['feedback','voltage']
                            for session in sessions.values()))

    def test_delayed_imu_rejects_before_voltage_dispatch(self):
        class SlowDevice(Device):
            def read_sample(self):
                time.sleep(.025)
                return super().read_sample()
        sessions={scope:GatedSession(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        observer=Observer()
        report,raw=bench.collect(sessions,SlowDevice(),observer,**self.options())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Feedback exceeded 20 ms pipeline hard deadline' in error
                            for error in report['errors']))
        row=bench._serialize(raw)[0]
        self.assertEqual(row['voltage'],{})
        self.assertEqual(row['output'],{})
        self.assertNotIn('voltage_gate_set_ns',row['voltage_pipeline'])
        self.assertTrue(all(session.phases==['feedback'] for session in sessions.values()))

    def test_validation_submit_failure_settles_released_bus_owners(self):
        release=threading.Event();release.set()
        sessions={scope:GatedSession(threading.Event(),release)
                  for scope in ('front','rear')}
        class RejectValidationPool(ThreadPoolExecutor):
            def submit(self,fn,*args,**kwargs):
                if fn is bench._validate_voltage_during_inference:
                    raise RuntimeError('synthetic validation submit rejection')
                return super().submit(fn,*args,**kwargs)
        observer=Observer()
        with patch.object(bench,'ThreadPoolExecutor',RejectValidationPool):
            report,raw=bench.collect(sessions,Device(),observer,**self.options())
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic validation submit rejection' in error
                            for error in report['errors']))
        self.assertTrue(observer.invalid)
        row=bench._serialize(raw)[0]
        self.assertEqual(set(row['acquired']),{'front','rear'})
        self.assertEqual(set(row['voltage'])|set(row.get('voltage_errors_by_bus',{})),
                         {'front','rear'})
        self.assertEqual(row['output'],{})
        self.assertEqual(observer.calls,0)

    def test_cli_requires_existing_overlap_profile_and_marks_new_plan(self):
        flags=['--mode','stop-proxy','--v3-voltage-proxy','--v3-voltage-overlap',
               '--v3-voltage-validation-overlap','--v3-voltage-pipeline',
               '--record-storage','trace','--cycles','2']
        stream=io.StringIO()
        with patch('sys.stdout',stream):self.assertEqual(bench.main(flags),0)
        plan=json.loads(stream.getvalue())
        self.assertTrue(plan['v3_voltage_pipeline'])
        self.assertTrue(plan['v3_voltage_overlap'])
        self.assertTrue(plan['v3_voltage_validation_overlap'])
        self.assertEqual(plan['requests_per_cycle'],26)
        self.assertEqual(plan['type1_requests_per_cycle'],0)
        self.assertEqual(plan['input_workers'],['front6+voltage1','rear6+voltage1','IMU'])
        for missing in ('--v3-voltage-proxy','--v3-voltage-overlap',
                        '--v3-voltage-validation-overlap'):
            with self.subTest(missing=missing),patch('sys.stderr',io.StringIO()):
                with self.assertRaises(SystemExit):bench.main([x for x in flags if x!=missing])
        with patch('sys.stderr',io.StringIO()):
            with self.assertRaises(SystemExit):bench.main([x for x in flags if x!='trace'])

    def test_collect_rejects_invalid_pipeline_config_before_io(self):
        sessions={scope:GatedSession(threading.Event(),threading.Event())
                  for scope in ('front','rear')}
        observer=Observer()
        for change in ({'v3_voltage_overlap':False},
                       {'v3_voltage_validation_overlap':False},
                       {'record_storage':'objects'},
                       {'v3_voltage_pipeline':1}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                bench.collect(sessions,Device(),observer,**self.options(**change))
        self.assertTrue(all(session.calls==0 for session in sessions.values()))
        self.assertEqual(observer.calls,0)

    def test_report_hash_binds_actual_records_bytes_even_on_startup_abort(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);out=root/'capture'
            flags=['--mode','stop-proxy','--v3-voltage-proxy','--v3-voltage-overlap',
                   '--v3-voltage-validation-overlap','--v3-voltage-pipeline',
                   '--record-storage','trace','--cycles','1','--execute',
                   '--supported-disabled','--front-port','/dev/null',
                   '--rear-port','/dev/null','--expected-uids',str(root/'missing-uids.json'),
                   '--library',str(root/'missing.so'),'--calibration',str(root/'missing-cal.json'),
                   '--mount',str(root/'missing-mount.json'),'--bundle',str(root/'missing-bundle'),
                   '--output',str(out)]
            with patch('sys.stdout',io.StringIO()):self.assertEqual(bench.main(flags),2)
            report=json.loads((out/'report.json').read_text())
            self.assertEqual(report['status'],'ABORTED')
            self.assertEqual(report['v3_voltage_pipeline']['records_sha256'],
                             hashlib.sha256((out/'records.json').read_bytes()).hexdigest())


if __name__=='__main__':unittest.main()
