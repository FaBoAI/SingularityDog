"""Failure cleanup with synthetic children only; no hardware job is executed."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import bounded_job as job


class BoundedJobFailureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root/'request.json').write_text('{}')
        self.lock = self.root/'lock'
        self.completion = self.root/'done.json'
        self.request = dict(job_id='synthetic', argv=['/bin/true'], cwd=str(self.root),
                            timeout_s=1., completion_file=str(self.completion),
                            expected_completion={'status':'COMPLETE'})

    def run_child(self, child, *, fail_save=False, kill=None, group_wait=None):
        original = job.save
        def save(path, state):
            if fail_save and state['status']=='RUNNING':
                raise OSError('synthetic receipt write failure')
            original(path,state)
        with patch.object(job,'validated_request',return_value=self.request), \
             patch.object(job.subprocess,'Popen',return_value=child), \
             patch.object(job,'save',side_effect=save), \
             patch.object(job,'require_group_monitor'), \
             patch.object(job,'_child_exit_status',return_value=None), \
             patch.object(job,'wait_child_unreaped',side_effect=lambda c,t:c.wait(timeout=t)), \
             patch.object(job,'_group_live_members',return_value=[child.pid] if child.poll() is None else []), \
             patch.object(job,'_wait_group_empty',side_effect=group_wait or (lambda *_:True)), \
             patch.object(job.os,'killpg',side_effect=kill) as killed:
            status = job.supervise(self.root,self.lock)
        return status,json.loads((self.root/'status.json').read_text()),killed.call_args_list

    def test_receipt_write_failure_terminates_live_child_before_unlock(self):
        child=Mock(pid=1000000);child.poll.return_value=None;child.wait.return_value=-15
        def still_locked(pid,signum):
            with self.lock.open('a') as lock:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        code,state,kills=self.run_child(child,fail_save=True,kill=still_locked)
        self.assertEqual(code,2);self.assertEqual(state['status'],'FAILED')
        self.assertEqual(kills[0].args,(child.pid,signal.SIGTERM))
        child.wait.assert_called_once_with(timeout=5)

    def test_keyboard_interrupt_and_signal_leave_interrupted_receipt(self):
        for error,wanted in ((KeyboardInterrupt(),130),(job.JobInterrupted(signal.SIGTERM),143)):
            with self.subTest(error=type(error).__name__):
                child=Mock(pid=1000000);child.poll.return_value=None
                child.wait.side_effect=[error,-15]
                code,state,kills=self.run_child(child)
                self.assertEqual(code,wanted);self.assertEqual(state['status'],'INTERRUPTED')
                self.assertFalse(state['automatic_retry']);self.assertEqual(len(kills),1)

    def test_timeout_cleanup_escalates_to_kill_with_finite_wait(self):
        child=Mock(pid=1000000);child.poll.return_value=None
        child.wait.side_effect=[subprocess.TimeoutExpired('fixture',1),-9]
        code,state,kills=self.run_child(child,group_wait=[False,True])
        self.assertEqual(code,2);self.assertIn('timeout',state['error'])
        self.assertEqual([call.args[1] for call in kills],[signal.SIGTERM,signal.SIGKILL])
        self.assertEqual([call.kwargs['timeout'] for call in child.wait.call_args_list],[1.,5])

    def test_completed_child_is_not_signalled_and_receipt_hashes_parsed_bytes(self):
        self.completion.write_text('{"status":"COMPLETE"}')
        child=Mock(pid=1000000);child.poll.return_value=0;child.wait.return_value=0
        code,state,kills=self.run_child(child)
        self.assertEqual(code,0);self.assertEqual(kills,[])
        self.assertEqual(state['completion_sha256'],job.digest(self.completion))

    def test_missing_null_bool_int_and_nested_type_mismatches_do_not_complete(self):
        for expected,result in (({'proof':None},{}),({'count':1},{'count':True}),
                                ({'nested':{'ok':True}},{'nested':{'ok':1}})):
            with self.subTest(expected=expected):
                self.request['expected_completion']=expected
                self.completion.write_text(json.dumps(result))
                child=Mock(pid=1000000);child.poll.return_value=0;child.wait.return_value=0
                code,state,_=self.run_child(child)
                self.assertEqual(code,2);self.assertIn('Strict completion mismatch',state['error'])

    def test_duplicate_completion_key_is_not_silently_replaced(self):
        self.completion.write_text('{"status":"FAIL","status":"COMPLETE"}')
        child=Mock(pid=1000000);child.poll.return_value=0;child.wait.return_value=0
        code,state,_=self.run_child(child)
        self.assertEqual(code,2);self.assertIn('Duplicate',state['error'])

    def test_handlers_restore_and_second_interrupt_cannot_abort_cleanup(self):
        old={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
        with job.interrupt_signals():
            handler=signal.getsignal(signal.SIGTERM)
            with self.assertRaises(job.JobInterrupted):handler(signal.SIGTERM,None)
            handler(signal.SIGINT,None)
        self.assertEqual({s:signal.getsignal(s) for s in old},old)

    def test_nonfinite_json_is_rejected(self):
        for value in ('NaN','Infinity','-Infinity','1e9999','-1e9999'):
            with self.subTest(value=value),self.assertRaises(ValueError):
                job.parse_json('{"timeout_s":'+value+'}')

    def test_first_signal_during_receipt_failure_cleanup_does_not_skip_wait(self):
        child=Mock(pid=1000000);child.poll.return_value=None;child.wait.return_value=-15
        def interrupted_cleanup(*_):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
            with self.lock.open('a') as lock:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        code,state,kills=self.run_child(child,fail_save=True,kill=interrupted_cleanup)
        self.assertEqual(code,143);self.assertEqual(state['status'],'INTERRUPTED')
        child.wait.assert_called_once_with(timeout=5)
        self.assertEqual(len(kills),1)

    def test_monitor_unavailable_prevents_any_child_start(self):
        with patch.object(job,'validated_request',return_value=self.request),\
             patch.object(job,'require_group_monitor',side_effect=OSError('ps unavailable')),\
             patch.object(job.subprocess,'Popen') as child:
            code=job.supervise(self.root,self.lock)
        child.assert_not_called();self.assertEqual(code,2)
        self.assertIn('ps unavailable',json.loads((self.root/'status.json').read_text())['error'])

    def test_automatic_child_reaping_cannot_disable_pid_reservation(self):
        with patch.object(job.signal,'getsignal',return_value=signal.SIG_IGN),\
             patch.object(job,'_group_live_members') as monitor:
            with self.assertRaisesRegex(RuntimeError,'non-reaping SIGCHLD'):
                job.require_group_monitor()
            monitor.assert_not_called()

    def test_reaped_leader_is_never_used_to_signal_a_possibly_reused_group(self):
        child=Mock(pid=1000000,returncode=0)
        with patch.object(job.os,'killpg') as kill:
            with self.assertRaisesRegex(RuntimeError,'reaped before group cleanup'):
                job.terminate_child(child)
            kill.assert_not_called()

    def test_group_monitor_excludes_zombies_and_rejects_bad_output(self):
        with patch.object(job.subprocess,'run',return_value=Mock(stdout='11 10 S\n10 10 Z\n12 12 R\n')) as run:
            self.assertEqual(job._group_live_members(10),[11])
            self.assertEqual(run.call_args.kwargs['timeout'],1)
        with patch.object(job.subprocess,'run',return_value=Mock(stdout='bad output')):
            with self.assertRaisesRegex(RuntimeError,'Cannot parse'):job._group_live_members(10)

    def test_same_group_descendant_is_terminated_even_after_parent_exit(self):
        # POSIX integration fixture: a real parent exits and leaves a sleeping
        # descendant in its isolated group. Neither process touches hardware.
        pid_path=self.root/'descendant.json'
        script=("import os,time,json; from pathlib import Path; "
                "pid=os.fork(); "
                "time.sleep(60) if pid==0 else None; "
                "Path("+repr(str(pid_path))+").write_text(json.dumps({'leader':os.getpid(),'child':pid})) if pid else None; "
                "Path("+repr(str(self.completion))+").write_text('{\\\"status\\\":\\\"COMPLETE\\\"}') if pid else None")
        self.request.update(argv=[sys.executable,'-c',script],timeout_s=3.)
        try:
            job.require_group_monitor()
        except (OSError,subprocess.SubprocessError) as error:
            self.skipTest('Process-table access is unavailable in this sandbox: '+str(error))
        try:
            with patch.object(job,'validated_request',return_value=self.request):
                code=job.supervise(self.root,self.lock)
            self.assertEqual(code,0)
            recorded=json.loads(pid_path.read_text())
            self.assertGreater(recorded['child'],0)
            self.assertEqual(job._group_live_members(recorded['leader']),[])
            with self.lock.open('a') as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        finally:
            if pid_path.exists():
                recorded=json.loads(pid_path.read_text())
                # Cleanup only this still-live descendant if an assertion or
                # implementation failure occurred; never signal a reused PGID.
                try:
                    if (os.getpgid(recorded['child'])==recorded['leader']
                            and recorded['child'] in job._group_live_members(recorded['leader'])):
                        os.kill(recorded['child'],signal.SIGKILL)
                except ProcessLookupError:pass


if __name__=='__main__':unittest.main()
