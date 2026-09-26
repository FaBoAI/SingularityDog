import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from singularitydog_hw import manual_motion_check as module
from singularitydog_hw.manual_motion_check import collect, identities, main, metrics, numeric_query, check_feedback_event, confirm_observed_motion, selected_port, wait_for_ready


class FakeTime:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


class FakeCAN:
    def __init__(self, clock, fail_at=None):
        self.clock, self.calls, self.fail_at = clock, [], fail_at
        self.parser=SimpleNamespace(discarded_bytes=0,buffer=b'')
    def query(self, mid, param):
        self.calls.append((mid,param))
        self.clock.t += .001
        if len(self.calls) == self.fail_at:
            raise TimeoutError('No reply')
        return {'ok':True, 'value':.02*self.clock.t if param=='position' else 0.,
                'monotonic_ns':int(self.clock.t*1e9)}


class ManualMotionTests(unittest.TestCase):
    def test_ready_prompt_retries_invalid_and_accepts_enter_or_yes_without_can(self):
        for final in ('','y'):
            with patch('singularitydog_hw.manual_motion_check.fresh_input',side_effect=['wrong','n',final]) as prompt, \
                 patch('singularitydog_hw.manual_motion_check.ReadOnlyCAN') as can, \
                 contextlib.redirect_stdout(io.StringIO()):
                wait_for_ready('Ready?',lambda:None)
                self.assertEqual(prompt.call_count,3)
                can.assert_not_called()
    def test_ready_quit_and_eof_do_not_become_consent(self):
        with patch('singularitydog_hw.manual_motion_check.fresh_input',return_value='q'):
            with self.assertRaises(InterruptedError): wait_for_ready('Ready?',lambda:None)
        with patch('singularitydog_hw.manual_motion_check.fresh_input',side_effect=EOFError):
            with self.assertRaises(EOFError): wait_for_ready('Ready?',lambda:None)
    def test_single_fr_joint_plan_is_explicit_and_readonly(self):
        with patch('singularitydog_hw.manual_motion_check.ReadOnlyCAN') as can, \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(['--motor-id','3','--expected-uids','missing','--output','unused']),0)
            plan=json.loads(output.getvalue())
            self.assertEqual(plan['ids'],[3])
            self.assertEqual(plan['leg'],'FR')
            self.assertEqual(plan['identity_check_ids'],[1,2,3])
            self.assertEqual(plan['port'],module.DEFAULT_FRONT_PORT)
            self.assertFalse(plan['motor_output_available'])
            can.assert_not_called()
    def test_selection_cannot_expand_to_other_legs(self):
        for value in ('0','4','all','1,2'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['--motor-id',value,'--expected-uids','missing','--output','unused'])
    def test_leg_selection_plan_and_single_joint_scope(self):
        for leg, ids in [('FR',[1,2,3]),('FL',[4,5,6]),('RR',[7,8,9]),('RL',[10,11,12])]:
            for single in (None,ids[-1]):
                argv=['--leg',leg,'--expected-uids','missing','--output','unused']
                if leg in ('RR','RL'):
                    argv += ['--port',f'/dev/serial/by-path/{leg.lower()}-usb2can']
                if single is not None: argv += ['--motor-id',str(single)]
                with patch.object(module,'ReadOnlyCAN') as can, contextlib.redirect_stdout(io.StringIO()) as output:
                    self.assertEqual(main(argv),0)
                plan=json.loads(output.getvalue())
                self.assertEqual(plan['ids'],ids if single is None else [single])
                self.assertEqual(plan['identity_check_ids'],ids)
                self.assertEqual(plan['port'], (f'/dev/serial/by-path/{leg.lower()}-usb2can'
                                                if leg in ('RR','RL') else module.DEFAULT_FRONT_PORT))
                self.assertEqual(plan['allowed_can_types'],[0,17])
                self.assertEqual([plan[k] for k in ('before_seconds_per_joint','moving_seconds_per_joint','released_seconds_per_joint')],[3,8,3])
                can.assert_not_called()
        for leg, mid in [('FL','1'),('FL','7'),('RR','6'),('RL','9'),('FR','4')]:
            with patch.object(module,'ReadOnlyCAN') as can, contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['--leg',leg,'--motor-id',mid,'--expected-uids','missing','--output','unused'])
            can.assert_not_called()

    def test_rear_leg_requires_explicit_by_path_port_before_any_hardware(self):
        invalid = (None, 'rear-usb2can', '/dev/ttyUSB1', '/dev/serial/by-path/..',
                   '/dev/serial/by-path/../ttyUSB1')
        for port in invalid:
            argv=['--leg','RR','--motor-id','7','--expected-uids','missing','--output','unused']
            if port is not None:
                argv += ['--port',port]
            with self.subTest(port=port), patch.object(module,'ReadOnlyCAN') as can, \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(argv)
            can.assert_not_called()
        self.assertEqual(selected_port('RR','/dev/serial/by-path/rear-usb2can'),
                         '/dev/serial/by-path/rear-usb2can')

    def execute_fl_fixture(self, *, single=True, bad_uid=None, bad_current=None,
                           changed_boot=False, leg='FL', motor_id=None, port=None):
        clock=FakeTime(); calls=[]; ports=[]; refs={str(i):f'{i:016x}' for i in range(1,13)}
        leg_ids=module.LEGS[leg][1]
        class ObservationCAN:
            def __init__(self,port,event_sink):
                self.parser=SimpleNamespace(discarded_bytes=0,buffer=b''); self.closed=False
                self.port=port
                ports.append(self)
            def __enter__(self): return self
            def __exit__(self,*_): self.closed=True
            def query(self,mid,param=None):
                calls.append((mid,param)); clock.t += .001
                if param is None:
                    return {'ok':True,'mcu_uid_hex':'f'*16 if mid==bad_uid else refs[str(mid)]}
                value=.02*clock.t if param=='position' else .06 if param=='current' and mid==bad_current else 0.
                return {'ok':True,'value':value,'monotonic_ns':int(clock.t*1e9)}
        def fake_read(path,*args,**kwargs):
            if str(path)=='/proc/sys/kernel/random/boot_id':
                return 'new-boot' if changed_boot and len(calls)>=6 else 'original-boot'
            return original_read(path,*args,**kwargs)
        def fast_collect(*args,**kwargs):
            return collect(*args,**kwargs,clock=clock.now,wait=clock.sleep)
        original_read=Path.read_text
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); uids=root/'uids.json'; uids.write_text(json.dumps(refs)); out=root/'capture'
            argv=['--execute-readonly','--leg',leg,'--expected-uids',str(uids),'--output',str(out)]
            if port is not None: argv += ['--port',port]
            if single: argv += ['--motor-id',str(motor_id if motor_id is not None else leg_ids[-1])]
            with patch.object(module,'ReadOnlyCAN',ObservationCAN), patch.object(module,'collect',side_effect=fast_collect), \
                 patch.object(module,'fresh_input',return_value='y'), patch('sys.stdin.isatty',return_value=True), \
                 patch.object(Path,'home',return_value=root), patch.object(Path,'read_text',fake_read), \
                 patch.object(module.fcntl,'flock',wraps=module.fcntl.flock) as locks, \
                 contextlib.redirect_stdout(io.StringIO()) as printed:
                rc=main(argv)
            summary=json.loads((out/'summary.json').read_text())
            events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
            lock_names=[Path(c.args[0].name).name for c in locks.call_args_list]
        self.assertTrue(all(p.closed for p in ports))
        self.assertTrue(all(p.port == (port or module.DEFAULT_FRONT_PORT) for p in ports))
        self.assertEqual(lock_names,['manual-calibration.lock','can-readonly.lock'])
        return rc,summary,events,calls,printed.getvalue()

    def test_id7_uses_explicit_rear_port_and_checks_rear_bus_identities(self):
        rear_port='/dev/serial/by-path/rear-usb2can'
        rc,summary,events,calls,_=self.execute_fl_fixture(
            leg='RR',motor_id=7,port=rear_port)
        self.assertEqual(rc,0)
        self.assertEqual(summary['plan']['port'],rear_port)
        self.assertEqual(summary['plan']['ids'],[7])
        self.assertEqual(calls[:6],[(mid,param) for mid in (7,8,9)
                                    for param in (None,'current')])
        self.assertEqual({row['motor_id'] for row in events
                          if row['kind']=='manual_motion_sample'},{7})
        self.assertTrue(all(mid in (7,8,9) for mid,_ in calls))

        rc,summary,events,calls,_=self.execute_fl_fixture(
            leg='RR',motor_id=7,port=rear_port,bad_uid=8)
        self.assertEqual(rc,1)
        self.assertEqual(summary['status'],'INCOMPLETE')
        self.assertEqual(summary['joints'],{})
        self.assertFalse(any(row['kind']=='manual_motion_sample' for row in events))
        self.assertFalse(any(parameter in ('position','velocity') for _,parameter in calls))

    def test_fl_execution_checks_all_three_identities_currents_then_selected_windows(self):
        for single in (True,False):
            with self.subTest(single=single):
                rc,summary,events,calls,printed=self.execute_fl_fixture(single=single)
                self.assertEqual(rc,0)
                self.assertEqual(summary['status'],'RECORDED_REVIEW_REQUIRED')
                self.assertFalse(summary['approved_for_runtime'])
                self.assertEqual(set(summary['joints']),{'6'} if single else {'4','5','6'})
                checks=[(mid,param) for mid in (4,5,6) for param in (None,'current')]
                self.assertEqual(calls[:6],checks)
                self.assertEqual(sum(param is None for _,param in calls),3 if single else 9)
                self.assertTrue(all(mid in (4,5,6) and param in (None,'position','velocity','current') for mid,param in calls))
                samples=[r for r in events if r['kind']=='manual_motion_sample']
                self.assertEqual({r['motor_id'] for r in samples},{6} if single else {4,5,6})
                self.assertEqual({r['phase'] for r in samples},{'before','moving','released'})
                self.assertIn('左前脚',printed); self.assertNotIn('右前脚',printed)
                for entry in summary['joints'].values():
                    for phase in ('before','moving','released'):
                        self.assertIsNone(entry[phase]['stationarity_pass'])
                        self.assertFalse(entry[phase]['approved_for_runtime'])

    def test_fl_wrong_sibling_identity_current_or_boot_aborts_before_observation(self):
        for failure in ({'bad_uid':4},{'bad_current':5},{'changed_boot':True}):
            with self.subTest(failure=failure):
                rc,summary,events,calls,_=self.execute_fl_fixture(**failure)
                self.assertEqual(rc,1); self.assertEqual(summary['status'],'INCOMPLETE')
                self.assertEqual(summary['joints'],{})
                self.assertTrue(summary['errors'])
                self.assertFalse(any(param in ('position','velocity') for _,param in calls))
                self.assertFalse(any(r['kind']=='manual_motion_sample' for r in events))
    def test_blank_and_invalid_answers_retry_without_any_can_read(self):
        with patch('singularitydog_hw.manual_motion_check.fresh_input',side_effect=['','maybe','y']) as prompt, \
             patch('singularitydog_hw.manual_motion_check.ReadOnlyCAN') as can, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertIs(confirm_observed_motion(1,lambda:None),True)
            self.assertEqual(prompt.call_count,3)
            can.assert_not_called()
    def test_unconfirmed_motion_stays_false_and_explicit_quit_aborts(self):
        with patch('singularitydog_hw.manual_motion_check.fresh_input',return_value='n'):
            self.assertIs(confirm_observed_motion(1,lambda:None),False)
        with patch('singularitydog_hw.manual_motion_check.fresh_input',return_value='q'):
            with self.assertRaises(InterruptedError): confirm_observed_motion(1,lambda:None)
    def test_interrupt_during_answer_retry_propagates(self):
        with patch('singularitydog_hw.manual_motion_check.fresh_input',return_value='') as prompt, \
             contextlib.redirect_stdout(io.StringIO()):
            def check():
                if prompt.call_count: raise InterruptedError('interrupted')
            with self.assertRaises(InterruptedError): confirm_observed_motion(1,check)
            self.assertEqual(prompt.call_count,1)
    def test_fixed_read_only_window_and_motion_metrics(self):
        clock=FakeTime(); can=FakeCAN(clock); emitted=[]
        rows=collect(can,1,3,'released',emitted.append,lambda:None,clock=clock.now,wait=clock.sleep)
        self.assertEqual(len(rows),150)
        self.assertEqual(len(can.calls),450)
        self.assertEqual(set(can.calls),{(1,'position'),(1,'velocity'),(1,'current')})
        report=metrics(rows)
        self.assertAlmostEqual(report['position_OLS_rad_s'],.02)
        self.assertGreater(report['position_range_deg'],3)
        self.assertFalse(report['approved_for_runtime'])
        self.assertIsNone(report['stationarity_pass'])
    def test_timeout_has_no_retry(self):
        clock=FakeTime(); can=FakeCAN(clock,4)
        with self.assertRaises(TimeoutError):
            collect(can,1,3,'released',lambda x:None,lambda:None,clock=clock.now,wait=clock.sleep)
        self.assertEqual(len(can.calls),4)
    def test_interrupt_stops_before_next_read(self):
        clock=FakeTime(); can=FakeCAN(clock)
        def check():
            if len(can.calls)>=2: raise InterruptedError()
        with self.assertRaises(InterruptedError):
            collect(can,1,3,'released',lambda x:None,check,clock=clock.now,wait=clock.sleep)
        self.assertEqual(len(can.calls),2)
    def test_bad_current_status_and_nonfinite_stop(self):
        for r in [{'ok':True,'value':.1},{'ok':False,'value':0},
                  {'ok':True,'value':math.nan},{'ok':True,'value':True}]:
            with self.subTest(r=r), patch.object(FakeCAN,'query',return_value=r):
                with self.assertRaises(RuntimeError):
                    numeric_query(FakeCAN(FakeTime()),1,'current',lambda:None)
    def test_dry_run_no_hardware_no_output_no_private_file_read(self):
        with tempfile.TemporaryDirectory() as tmp, patch('singularitydog_hw.manual_motion_check.ReadOnlyCAN') as can:
            p=Path(tmp)/'new'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--expected-uids',str(Path(tmp)/'absent'),'--output',str(p)]),0)
            can.assert_not_called(); self.assertFalse(p.exists())
    def test_noninteractive_execution_rejected_before_hardware(self):
        with patch('sys.stdin.isatty',return_value=False), patch('singularitydog_hw.manual_motion_check.ReadOnlyCAN') as can:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['--execute-readonly','--expected-uids','missing','--output','unused'])
            can.assert_not_called()
    def test_reference_identity_shape_uniqueness(self):
        refs={str(i):f'{i:016x}' for i in range(1,13)}
        self.assertEqual(identities(refs),refs)
        for bad in [{}, {**refs,'1':refs['2']},{**refs,'1':'Z'*16},{**refs,'13':'d'*16}]:
            with self.assertRaises(ValueError): identities(bad)
    def test_position_timestamps_cannot_be_reused(self):
        row={'position_read_ns':10,'position_rad':0,'velocity_rad_s':0}
        with self.assertRaises(ValueError): metrics([row,row])
    def test_corrupt_or_partial_serial_frame_aborts_immediately(self):
        for discarded,buffer in [(1,b''),(0,b'A')]:
            can=FakeCAN(FakeTime())
            can.parser.discarded_bytes,can.parser.buffer=discarded,buffer
            with self.assertRaises(RuntimeError): numeric_query(can,1,'position',lambda:None)
            self.assertEqual(len(can.calls),1)
    def test_fault_or_enabled_unsolicited_feedback_aborts(self):
        for values in [{'type':21},{'type':2,'mode_state':2,'fault_bits':0},
                       {'type':2,'mode_state':0,'fault_bits':8}]:
            with self.assertRaises(RuntimeError):
                check_feedback_event({'kind':'motor_feedback',**values})
        check_feedback_event({'kind':'motor_feedback','type':2,'mode_state':0,'fault_bits':0})
    def test_window_selection_is_bounded(self):
        for mid, seconds, phase in [(4,3,'released'),(1,60,'released'),(1,3,'active')]:
            with self.assertRaises(ValueError):
                collect(None,mid,seconds,phase,None,None)
        for mid,leg in [(1,'FL'),(4,'FR'),(6,'RR'),(True,'FR'),(1,'unknown')]:
            with self.assertRaises(ValueError):
                collect(None,mid,3,'before',None,None,leg=leg)


if __name__=='__main__': unittest.main()
