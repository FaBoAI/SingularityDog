import contextlib
import copy
import io
import json
import os
from pathlib import Path
import tempfile
import signal
import subprocess
import unittest
from unittest.mock import Mock, patch

import imu_six_face_session as session

MOUNT = {'R_body_from_sensor': [[0, 1, 0], [1, 0, 0], [0, 0, -1]]}


class PoseTests(unittest.TestCase):
    def test_body_directions_use_selected_mount_not_default_identity(self):
        poses = session.pose_plan(MOUNT)
        self.assertEqual([p['sensor_face'] for p in poses], ['z-', 'z+', 'x+', 'x-', 'y+', 'y-'])
        identity = session.pose_plan({'R_body_from_sensor': [[1,0,0],[0,1,0],[0,0,1]]})
        self.assertEqual([p['sensor_face'] for p in identity], ['z+', 'z-', 'y+', 'y-', 'x+', 'x-'])

    def test_invalid_mounts_do_not_suggest_physical_poses(self):
        matrices = ([[1,0,0],[0,1,0],[0,0,-1]], [[1,0,0],[1,0,0],[0,0,1]],
                    [[True,0,0],[0,1,0],[0,0,1]], [[.9,.1,0],[0,1,0],[0,0,1]],
                    [[1,0,0],[0,1,0]], None)
        for matrix in matrices:
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                session.pose_plan({'R_body_from_sensor':matrix})
        for value in (None, [], 'mount'):
            with self.subTest(mount=value), self.assertRaisesRegex(ValueError, 'object'):
                session.pose_plan(value)

    def test_commands_are_imu_only_and_have_bounded_duration(self):
        command = session.capture_command('/python', Path('/capture'), 'x+')
        self.assertEqual(command[3], 'singularitydog_hw.imu_capture')
        self.assertIn('--execute', command)
        self.assertEqual(command[command.index('--seconds')+1], '10')
        self.assertEqual(command[command.index('--settle-seconds')+1], '3')
        self.assertFalse(any('can' in arg or 'policy' in arg for arg in command))

    def test_plan_does_not_import_device_or_play_audio_or_create_output(self):
        with tempfile.TemporaryDirectory() as temp:
            mount = Path(temp)/'mount.json'; mount.write_text(json.dumps(MOUNT))
            output = Path(temp)/'does-not-exist'
            with patch.object(session, 'audio_player', side_effect=AssertionError('audio opened')), \
                 patch.object(session, 'run_capture', side_effect=AssertionError('capture opened')), \
                 patch.object(session.signal, 'signal', side_effect=AssertionError('signal handler changed')), \
                 contextlib.redirect_stdout(io.StringIO()) as stdout:
                self.assertEqual(session.main(['--mount',str(mount),'--output-root',str(output)]),0)
            plan=json.loads(stdout.getvalue())
            self.assertEqual(plan['status'],'PLAN_ONLY')
            self.assertFalse(output.exists())
            self.assertFalse(plan['can_opened'])


class SessionTests(unittest.TestCase):
    def run_case(self, directory, *, answers=None, fail_capture=None, fail_audit=None,
                 fail_verify=None, fail_audio=None, fit_result=None, mutate_at_fit=None,
                 mutate_after_last_capture=None):
        captures, audits, spoken, fits = [], [], [], []
        answers = iter(answers if answers is not None else ['y','y']+['']*6+['y']+['']*6+['y'])
        checks=0
        def announce(key):
            spoken.append(key)
            if key==fail_audio:raise RuntimeError('audio error')
        def capture(path, face):
            captures.append((path,face))
            path.mkdir()
            (path/'raw.json').write_text('{}')
            (path/'summary.json').write_text('{"fixture":true}')
            (path/'events.jsonl').write_text('{"fixture":true}\n')
            if len(captures)==fail_capture:raise RuntimeError('capture incomplete')
        def audit(path, face):
            audits.append((path,face))
            if len(audits)==fail_audit:raise RuntimeError('restore not verified')
            result = {'samples':1000, 'provenance':{
                'events_sha256':session.file_sha256(path/'events.jsonl'),
                'summary_sha256':session.file_sha256(path/'summary.json')}}
            if len(audits)==12 and mutate_after_last_capture:
                mutate_after_last_capture(Path(directory))
            return result
        def fit(a,b):
            fits.append((copy.deepcopy(a),copy.deepcopy(b)))
            if mutate_at_fit:mutate_at_fit(Path(directory))
            return fit_result if fit_result is not None else {'status':'candidate',**session.FLAGS}
        def verify():
            nonlocal checks
            checks+=1
            if checks==fail_verify:raise RuntimeError('source changed')
        report=session.run_session(Path(directory),session.pose_plan(MOUNT),announce=announce,
            capture=capture,audit=audit,fit=fit,verify=verify,read=lambda:next(answers),write=lambda _:None)
        self.assertEqual(json.loads((Path(directory)/'session.json').read_text()),report)
        return report,captures,audits,spoken,fits

    def test_fit_and_validation_are_distinct_and_final_candidate_unapproved(self):
        with tempfile.TemporaryDirectory() as temp:
            result,captures,audits,spoken,fits=self.run_case(temp)
            self.assertEqual(result['status'],'SIX_FACE_CANDIDATE_REVIEW_REQUIRED')
            self.assertEqual(len(captures),12);self.assertEqual(len(audits),12)
            self.assertEqual(result['operator_confirmed_partitions'],['fit','validation'])
            a,b=fits[0]
            self.assertEqual(set(a),{'x+','x-','y+','y-','z+','z-'})
            self.assertEqual(set(a),set(b));self.assertFalse(set(a.values())&set(b.values()))
            self.assertEqual(spoken.count('independent'),1)
            self.assertTrue((Path(temp)/'calibration-candidate.json').exists())
            self.assertFalse(result['approved_for_runtime']);self.assertFalse(result['can_opened'])
            self.assertEqual(len(result['capture_file_bindings']),12)

    def test_cancel_before_power_confirmation_never_captures(self):
        for answers in (['q'], ['y','q']):
            with tempfile.TemporaryDirectory() as temp:
                result,captures,_,_,fits=self.run_case(temp,answers=answers)
                self.assertEqual(result['status'],'INCOMPLETE_SIX_FACE_SESSION')
                self.assertEqual(captures,[]);self.assertEqual(fits,[])

    def test_blank_does_not_confirm_power_off(self):
        with tempfile.TemporaryDirectory() as temp:
            result,captures,_,_,_=self.run_case(temp,answers=['y','','q'])
            self.assertEqual(captures,[])
            self.assertNotIn('operator_confirmed_motor_power_off_and_support',result)

    def test_no_quiet_confirmation_preserves_six_captures_without_fit(self):
        with tempfile.TemporaryDirectory() as temp:
            result,captures,_,_,fits=self.run_case(temp,answers=['y','y']+['']*6+['n'])
            self.assertEqual(len(captures),6);self.assertEqual(fits,[])
            self.assertEqual(result['operator_confirmed_partitions'],[])
            self.assertFalse((Path(temp)/'calibration-candidate.json').exists())

    def test_failed_capture_or_restore_audit_stops_before_next_pose(self):
        for kind in ('fail_capture','fail_audit'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as temp:
                result,captures,_,_,fits=self.run_case(temp,**{kind:3})
                self.assertEqual(len(captures),3);self.assertEqual(len(result['captures']),2)
                self.assertEqual(fits,[])
                self.assertEqual(len(list(Path(temp).glob('*/raw.json'))),3)

    def test_source_changed_before_first_capture_stops(self):
        with tempfile.TemporaryDirectory() as temp:
            result,captures,_,_,fits=self.run_case(temp,fail_verify=3)
            self.assertEqual(captures,[]);self.assertEqual(fits,[])
            self.assertEqual(result['status'],'INCOMPLETE_SIX_FACE_SESSION')

    def test_audio_failure_prevents_first_capture(self):
        with tempfile.TemporaryDirectory() as temp:
            result,captures,_,_,_=self.run_case(temp,fail_audio='start')
            self.assertEqual(captures,[])
            self.assertEqual(result['status'],'INCOMPLETE_SIX_FACE_SESSION')

    def test_missing_or_approved_fit_cannot_publish_candidate(self):
        for result in ({},{'approved_for_runtime':True,'automatically_applied':False}):
            with tempfile.TemporaryDirectory() as temp:
                report,*_=self.run_case(temp,fit_result=result)
                self.assertFalse((Path(temp)/'calibration-candidate.json').exists())
                self.assertEqual(report['status'],'INCOMPLETE_SIX_FACE_SESSION')

    def test_existing_receipt_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'data.json';session.save(path,{'old':True})
            with self.assertRaises(FileExistsError):session.save(path,{'new':True})
            self.assertEqual(json.loads(path.read_text()),{'old':True})

    def test_modified_earlier_capture_prevents_fit(self):
        for filename in ('summary.json','events.jsonl'):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as temp:
                def mutate(root):
                    (root/'fit-z-'/filename).write_text('changed')
                result,_,_,_,fits=self.run_case(temp,mutate_after_last_capture=mutate)
                self.assertEqual(fits,[])
                self.assertIn('Confirmed capture or receipt changed',result['errors'][0])
                self.assertFalse((Path(temp)/'calibration-candidate.json').exists())

    def test_modified_receipt_prevents_fit(self):
        with tempfile.TemporaryDirectory() as temp:
            def mutate(root):(root/'fit-z--receipt.json').write_text('{}')
            result,_,_,_,fits=self.run_case(temp,mutate_after_last_capture=mutate)
            self.assertEqual(fits,[])
            self.assertEqual(result['status'],'INCOMPLETE_SIX_FACE_SESSION')

    def test_changed_inputs_or_receipt_during_fit_prevent_candidate(self):
        for filename in ('fit-z-/summary.json','fit-z-/events.jsonl','fit-z--receipt.json'):
            with self.subTest(filename=filename),tempfile.TemporaryDirectory() as temp:
                def mutate(root):(root/filename).write_text('{}')
                result,_,_,_,fits=self.run_case(temp,mutate_at_fit=mutate)
                self.assertEqual(len(fits),1)
                self.assertEqual(result['status'],'INCOMPLETE_SIX_FACE_SESSION')
                self.assertFalse((Path(temp)/'calibration-candidate.json').exists())


class CaptureCancellationTests(unittest.TestCase):
    def run_child(self, outcomes, *, initial_signal=None):
        child=Mock()
        child.wait.side_effect=outcomes
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(session.subprocess,'Popen',return_value=child) as popen:
            command=['/fixture-python','-m','fixture']
            if initial_signal:
                def create(*args,**kwargs):
                    os.kill(os.getpid(),initial_signal)
                    return child
                popen.side_effect=create
            try:
                with session.operator_signal_handlers():
                    session.run_capture(command,{},Path(temp)/'capture.log')
            except BaseException as error:
                return child,popen,error
        return child,popen,None

    def test_capture_uses_separate_session_and_inert_stdin(self):
        child,popen,error=self.run_child([0])
        self.assertIsNone(error)
        self.assertTrue(popen.call_args.kwargs['start_new_session'])
        self.assertEqual(popen.call_args.kwargs['stdin'],subprocess.DEVNULL)
        self.assertFalse(child.send_signal.called)

    def test_second_interrupt_waits_for_restoration_and_reaps(self):
        child,_,error=self.run_child([KeyboardInterrupt(),KeyboardInterrupt(),1])
        self.assertIsInstance(error,KeyboardInterrupt)
        self.assertEqual(child.wait.call_count,3)
        child.send_signal.assert_called_once_with(signal.SIGINT)
        self.assertFalse(child.kill.called)

    def test_timeout_kills_and_reaps_despite_further_interrupts(self):
        timeout=subprocess.TimeoutExpired('fixture',35)
        child,_,error=self.run_child([timeout,subprocess.TimeoutExpired('fixture',10),KeyboardInterrupt(),-9])
        self.assertIs(error,timeout)
        child.kill.assert_called_once()
        self.assertEqual(child.wait.call_count,4)

    def test_parent_term_or_hangup_at_creation_cancels_after_child_is_owned(self):
        for sig in (signal.SIGTERM,signal.SIGHUP):
            with self.subTest(sig=sig):
                child,_,error=self.run_child([1],initial_signal=sig)
                self.assertIsInstance(error,session.SessionCancelled)
                child.send_signal.assert_called_once_with(signal.SIGINT)
                self.assertEqual(child.wait.call_count,1)

    def test_parent_signal_during_active_wait_enters_cleanup(self):
        for sig in (signal.SIGTERM,signal.SIGHUP):
            with self.subTest(sig=sig):
                child=Mock()
                def first_wait(*args,**kwargs):
                    os.kill(os.getpid(),sig)
                child.wait.side_effect=first_wait
                def sent(*args):child.wait.side_effect=None;child.wait.return_value=1
                child.send_signal.side_effect=sent
                with tempfile.TemporaryDirectory() as temp,patch.object(session.subprocess,'Popen',return_value=child):
                    with self.assertRaises(session.SessionCancelled),session.operator_signal_handlers():
                        session.run_capture(['/fixture'],{},Path(temp)/'log')
                self.assertEqual(child.wait.call_count,2)

    def test_original_signal_handlers_restored_after_success_and_exception(self):
        original={sig:signal.getsignal(sig) for sig in session.OPERATOR_SIGNALS}
        for fail in (False,True):
            try:
                with session.operator_signal_handlers():
                    if fail:raise RuntimeError('fixture')
            except RuntimeError:pass
            self.assertEqual({sig:signal.getsignal(sig) for sig in session.OPERATOR_SIGNALS},original)

    def test_repeated_signals_do_not_interrupt_cleanup(self):
        child=Mock()
        def restore_wait(*args,**kwargs):
            for sig in session.OPERATOR_SIGNALS:os.kill(os.getpid(),sig)
            return 1
        child.wait.side_effect=restore_wait
        with session.operator_signal_handlers():
            session.cancel_capture_child(child)
        self.assertEqual(child.wait.call_count,1)


if __name__=='__main__':unittest.main()
