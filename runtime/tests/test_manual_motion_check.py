import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from singularitydog_hw.manual_motion_check import collect, identities, main, metrics, numeric_query, check_feedback_event, confirm_observed_motion, wait_for_ready


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
            self.assertFalse(plan['motor_output_available'])
            can.assert_not_called()
    def test_selection_cannot_expand_to_other_legs(self):
        for value in ('0','4','all','1,2'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['--motor-id',value,'--expected-uids','missing','--output','unused'])
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


if __name__=='__main__': unittest.main()
