"""Command-line gating tests; no serial, I2C, speech or network access."""
from contextlib import ExitStack, nullcontext, redirect_stdout, redirect_stderr
import hashlib
import io
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

from singularitydog_hw import policy_output as cli
from singularitydog_hw.policy_live_profile import template, ProfileError, SCHEMA_V1, SCHEMA_V2


class MockSerialPort:
    """Owns a regular temporary file only; never opens a tty or network FD."""
    def __init__(self, path):
        self.path=path;self.fd=None;self.closed=False
    def open(self):
        self.fd=os.open(self.path,os.O_CREAT|os.O_RDWR,0o600)
    def fileno(self):return self.fd
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
        self.closed=True


class PolicyOutputCLITests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.path = self.base/'profile.json'; self.path.write_text(json.dumps(template()))
        self.audio = self.base/'test.wav'; self.audio.write_bytes(b'synthetic audio, never played')
        self.digest = hashlib.sha256(self.audio.read_bytes()).hexdigest()
        self.out = self.base/'out'
        self.fake = dict(template(), output_allowed=True, profile_sha256='1'*64,
                         approved_for_supported_policy_output=True, blockers=[], motor_power_epoch='test-epoch')

    def args(self):
        return ['--profile', str(self.path), '--execute-supported', '--support-in-place', '--cutoff-ready',
                '--front-port', '/DO_NOT_OPEN/front', '--rear-port', '/DO_NOT_OPEN/rear',
                '--library', '/DO_NOT_LOAD/native.so', '--output', str(self.out),
                '--audio', str(self.audio), '--audio-sha256', self.digest, '--audio-device', 'DO_NOT_PLAY',
                '--power-epoch', 'test-epoch']

    def run_quiet(self, args):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return cli.main(args)

    def expected_transport(self,schema,*,gap_us=600,window=3):
        return {'request_gap_us':gap_us,'request_window':window,'source_profile_schema':schema,
                'emergency_stop_uses_same_gap':True}

    def approved_profile(self,schema,*,gap_us=600,window=3):
        value=dict(template(schema=schema),output_allowed=True,profile_sha256='1'*64,
                   approved_for_supported_policy_output=True,blockers=[],motor_power_epoch='test-epoch',
                   boot_id='00000000-0000-0000-0000-000000000001')
        if schema==SCHEMA_V2:value.update(request_gap_us=gap_us,request_window=window)
        for axis in value['axes'].values():axis.update(kp=4.,kd=.2)
        return value

    def execute_mocked_profile(self,profile,*,status,extra_args=(),expect_deferred=False):
        ports=[];sessions=[];boot_fds=[];real_open=os.open
        boot=self.base/'boot-id';boot.write_text(profile['boot_id']+'\n')
        guard=Mock(boot_id=profile['boot_id'])
        device=Mock(restore_status='restored');device.start.return_value={'source':'mock-imu'}
        model=SimpleNamespace(provenance={'kind':'mock-model'},calls=0)
        outcome={'status':status,'errors':[] if status=='COMPLETE_SUPPORTED_OUTPUT' else ['synthetic runtime abort'],
                 'motor_enable_sent':False,'learned_targets_sent':False}

        def open_owned(path,flags,*args,**kwargs):
            if os.fspath(path)=='/proc/sys/kernel/random/boot_id':
                fd=real_open(boot,flags,*args,**kwargs);boot_fds.append(fd);return fd
            return real_open(path,flags,*args,**kwargs)

        def make_port(**kwargs):
            self.assertIsNone(kwargs['port'])
            port=MockSerialPort(self.base/('mock-port-'+str(len(ports))));ports.append(port)
            return port

        def make_session(*args,**kwargs):
            session=Mock();sessions.append(session);return session

        torch=SimpleNamespace(set_num_threads=Mock(),set_num_interop_threads=Mock())
        bindings={scope:{'path':'DO_NOT_OPEN_'+scope,'resolved':'DO_NOT_OPEN_'+scope,'st_rdev':0}
                  for scope in ('front','rear')}
        try:
            with ExitStack() as stack:
                stack.enter_context(patch.object(cli,'load_profile',return_value=profile))
                stack.enter_context(patch.dict('sys.modules',{'torch':torch,'serial':SimpleNamespace(Serial=make_port)}))
                stack.enter_context(patch.object(cli.os,'open',side_effect=open_owned))
                stack.enter_context(patch.object(cli.signal,'signal',return_value=None))
                model_ctor=stack.enter_context(patch('singularitydog_hw.policy_output_model.LivePolicyModel',return_value=model))
                stack.enter_context(patch('singularitydog_hw.native_active_transport.load_library',return_value=object()))
                constructor=stack.enter_context(patch('singularitydog_hw.native_active_transport.ActiveSession',side_effect=make_session))
                stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.validate_ports',return_value=bindings))
                stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.binding_matches',return_value=True))
                stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.BootIdentityGuard',return_value=guard))
                stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.pipeline.ownership_locks',side_effect=nullcontext))
                stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.port_lock',side_effect=lambda _:nullcontext()))
                stack.enter_context(patch('singularitydog_hw.policy_observer_live.imu_ownership_lock',side_effect=nullcontext))
                stack.enter_context(patch('singularitydog_hw.imu.ICM20948',return_value=device))
                runner=stack.enter_context(patch('singularitydog_hw.policy_output_runtime.run_supported_policy',return_value=outcome))
                audio=stack.enter_context(patch.object(cli.subprocess,'run'))
                code=self.run_quiet(self.args()+list(extra_args))
            model_ctor.assert_called_once_with(profile,**({'defer_warmup':True} if expect_deferred else {}))
            self.assertTrue(all(port.closed for port in ports))
            for session in sessions:session.close.assert_called_once_with()
            guard.close.assert_called_once_with();device.close.assert_called_once_with()
            audio.assert_not_called()
            for fd in boot_fds:
                with self.assertRaises(OSError):os.fstat(fd)
            return code,json.loads((self.out/'report.json').read_text()),constructor,runner,sessions
        finally:
            for port in ports:
                if port.fd is not None:port.close()
            for fd in boot_fds:
                try:os.close(fd)
                except OSError:pass

    def test_default_plan_reads_template_without_loading_model_or_opening_devices(self):
        with patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
             patch('singularitydog_hw.native_active_transport.load_library') as native, \
             patch.object(cli.subprocess, 'run') as audio, redirect_stdout(io.StringIO()) as out:
            code = cli.main(['--profile', str(self.path)])
        self.assertEqual(code, 0)
        report = json.loads(out.getvalue())
        self.assertEqual(report['status'], 'PLAN_ONLY')
        self.assertFalse(report['hardware_opened']); self.assertFalse(report['output_allowed'])
        self.assertFalse(report['exclude_policy_cpu_from_workers'])
        self.assertFalse(report['math_thread_startup']['selected'])
        model.assert_not_called(); native.assert_not_called(); audio.assert_not_called()

    def test_single_thread_math_plan_starts_early_and_late_import_fails_closed(self):
        environment=dict(os.environ,OMP_NUM_THREADS='8',OPENBLAS_NUM_THREADS='4',MKL_NUM_THREADS='2',
                         PYTHONPATH=str(Path(__file__).resolve().parents[1]))
        command=[sys.executable,'-m','singularitydog_hw.policy_output',
                 '--profile',str(self.path),'--single-thread-math']
        result=subprocess.run(command,env=environment,text=True,capture_output=True,check=False)
        self.assertEqual(result.returncode,0,result.stderr)
        plan=json.loads(result.stdout)
        self.assertEqual(plan['math_thread_startup']['effective_env'],
                         {'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1'})
        self.assertTrue(plan['math_thread_startup']['verified_before_math_import'])
        self.assertFalse(plan['hardware_opened'])
        late=subprocess.run([sys.executable,'-c',
            'import sys,types; sys.modules["numpy"]=types.ModuleType("numpy"); '
            'from singularitydog_hw import policy_output; policy_output.main(sys.argv[1:])',
            '--profile',str(self.path),'--single-thread-math'],env=environment,
            text=True,capture_output=True,check=False)
        self.assertEqual(late.returncode,2)
        self.assertIn('before NumPy/Torch import',late.stderr)
        self.assertFalse(self.out.exists())

    def test_absolute_epoch_selection_is_explicit_and_file_only_in_plan(self):
        flags=['--absolute-epoch-cadence','--release-spin-us','500',
               '--active-timer-slack-ns','1000']
        with patch('singularitydog_hw.native_active_transport.load_library') as native, \
             redirect_stdout(io.StringIO()) as out:
            self.assertEqual(cli.main(['--profile',str(self.path),*flags]),0)
        plan=json.loads(out.getvalue())
        self.assertTrue(plan['absolute_epoch_cadence'])
        self.assertEqual(plan['release_spin_us'],500)
        self.assertEqual(plan['active_timer_slack_ns'],1000)
        self.assertFalse(plan['hardware_opened']);native.assert_not_called()
        with patch.object(cli,'load_profile') as load,redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                cli.main(['--profile',str(self.path),'--release-spin-us','500'])
        load.assert_not_called()

    def test_absolute_epoch_and_worker_slack_forward_to_runtime_only_when_selected(self):
        code,report,_,runner,_=self.execute_mocked_profile(
            self.approved_profile(SCHEMA_V2),status='COMPLETE_SUPPORTED_OUTPUT',
            extra_args=['--absolute-epoch-cadence','--active-timer-slack-ns','1000'])
        self.assertEqual(code,0)
        self.assertTrue(report['absolute_epoch_cadence'])
        self.assertEqual(report['active_timer_slack_ns'],1000)
        self.assertTrue(runner.call_args.kwargs['absolute_epoch_cadence'])
        self.assertEqual(runner.call_args.kwargs['active_timer_slack_ns'],1000)

    def test_r22_plan_and_invalid_flag_pairs_stay_file_only(self):
        flags=['--pre-cycle-policy-warmup-calls','10','--main-thread-cpu','4',
               '--post-pin-policy-prime-calls','10','--defer-gc-during-cycles']
        with patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
             patch('singularitydog_hw.native_active_transport.load_library') as native, \
             redirect_stdout(io.StringIO()) as out:
            self.assertEqual(cli.main(['--profile',str(self.path),*flags]),0)
        plan=json.loads(out.getvalue())
        self.assertTrue(plan['r22_startup_selected'])
        self.assertEqual(plan['post_pin_policy_prime_calls'],10)
        self.assertTrue(plan['defer_gc_during_cycles']);self.assertFalse(plan['hardware_opened'])
        model.assert_not_called();native.assert_not_called();self.assertFalse(self.out.exists())
        bad=(('--pre-cycle-policy-warmup-calls','10'),('--main-thread-cpu','4'),
             ('--post-pin-policy-prime-calls','10'),('--main-thread-cpu','3',
              '--pre-cycle-policy-warmup-calls','10'))
        for selection in bad:
            with self.subTest(selection=selection),patch.object(cli,'load_profile') as load, \
                 patch.object(cli.Path,'mkdir') as mkdir,redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    cli.main(['--profile',str(self.path),*selection])
                self.assertEqual(raised.exception.code,2)
            load.assert_not_called();mkdir.assert_not_called()

    def test_r22_execute_forwards_unwrapped_model_and_selected_gc(self):
        profile=self.approved_profile(SCHEMA_V2)
        flags=['--pre-cycle-policy-warmup-calls','10','--main-thread-cpu','4',
               '--post-pin-policy-prime-calls','10','--defer-gc-during-cycles']
        code,report,_,runner,_=self.execute_mocked_profile(profile,status='COMPLETE_SUPPORTED_OUTPUT',
            extra_args=flags,expect_deferred=True)
        self.assertEqual(code,0);self.assertTrue(report['r22_startup_selected'])
        self.assertEqual(report['post_pin_policy_prime_calls'],10)
        self.assertTrue(report['defer_gc_during_cycles'])
        kwargs=runner.call_args.kwargs
        self.assertIs(kwargs['startup_model'],runner.call_args.args[3])
        self.assertEqual((kwargs['main_thread_cpu'],kwargs['pre_cycle_policy_warmup_calls'],
                          kwargs['post_pin_policy_prime_calls']),(4,10,10))
        self.assertTrue(kwargs['defer_gc_during_cycles'])

    def test_worker_exclusion_is_explicit_and_requires_r22_supported_only(self):
        flags=['--pre-cycle-policy-warmup-calls','10','--main-thread-cpu','4',
               '--exclude-policy-cpu-from-workers']
        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(cli.main(['--profile',str(self.path),*flags]),0)
        plan=json.loads(out.getvalue())
        self.assertTrue(plan['exclude_policy_cpu_from_workers']);self.assertFalse(plan['hardware_opened'])
        for args,execution in ((['--exclude-policy-cpu-from-workers'],None),(flags,Mock())):
            with self.subTest(args=args,execution=execution),patch.object(cli,'load_profile') as load, \
                 redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    cli.main(['--profile',str(self.path),*args],execution=execution)
                load.assert_not_called()
        code,report,_,runner,_=self.execute_mocked_profile(self.approved_profile(SCHEMA_V2),
            status='COMPLETE_SUPPORTED_OUTPUT',extra_args=flags,expect_deferred=True)
        self.assertEqual(code,0);self.assertTrue(report['exclude_policy_cpu_from_workers'])
        self.assertTrue(runner.call_args.kwargs['exclude_policy_cpu_from_workers'])

    def test_v1_and_v2_plans_report_profile_bound_transport_settings(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            candidate=template(schema=schema)
            if schema==SCHEMA_V2:candidate.update(request_gap_us=gap,request_window=window)
            self.path.write_text(json.dumps(candidate))
            for overrides in ((),('--request-gap-us',str(gap),'--request-window',str(window))):
                with self.subTest(schema=schema,overrides=overrides), \
                     patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
                     patch('singularitydog_hw.native_active_transport.load_library') as native, \
                     patch('singularitydog_hw.native_active_transport.ActiveSession') as session, \
                     patch('singularitydog_hw.imu.ICM20948') as imu, \
                     patch.object(cli.subprocess,'run') as audio,redirect_stdout(io.StringIO()) as stream:
                    self.assertEqual(cli.main(['--profile',str(self.path),*overrides]),0)
                payload=json.loads(stream.getvalue())
                self.assertEqual(payload['transport_settings'],self.expected_transport(schema,gap_us=gap,window=window))
                self.assertEqual(payload['status'],'PLAN_ONLY');self.assertFalse(payload['hardware_opened'])
                self.assertFalse(payload['output_allowed']);self.assertFalse(self.out.exists())
                for method in (model,native,session,imu,audio):method.assert_not_called()

    def test_profile_transport_settings_reach_both_sessions_and_success_or_abort_report(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            for status in ('COMPLETE_SUPPORTED_OUTPUT','ABORTED'):
                with self.subTest(schema=schema,status=status):
                    self.out=self.base/('result-'+schema.rsplit('.',1)[-1]+'-'+status)
                    profile=self.approved_profile(schema,gap_us=gap,window=window)
                    code,report,constructor,runner,sessions=self.execute_mocked_profile(profile,status=status)
                    self.assertEqual(code,0 if status=='COMPLETE_SUPPORTED_OUTPUT' else 2)
                    self.assertEqual(report['status'],status)
                    self.assertEqual(report['transport_settings'],self.expected_transport(schema,gap_us=gap,window=window))
                    self.assertEqual(report['imu_restore_status'],'restored')
                    self.assertEqual(constructor.call_count,2)
                    self.assertEqual([call.kwargs['first_id'] for call in constructor.call_args_list],[1,7])
                    for call in constructor.call_args_list:
                        self.assertEqual(call.kwargs['gap_ns'],gap*1000)
                        self.assertEqual(call.kwargs['window'],window)
                    runner.assert_called_once()
                    self.assertIs(runner.call_args.args[0],profile)
                    self.assertEqual(runner.call_args.args[1],dict(zip(('front','rear'),sessions)))
                    if status=='ABORTED':self.assertEqual(report['errors'],['synthetic runtime abort'])

    def test_invalid_transport_overrides_rejected_before_profile_or_setup(self):
        invalid=(('--request-gap-us','599'),('--request-gap-us','5001'),('--request-gap-us','not-an-int'),
                 ('--request-window','0'),('--request-window','4'),('--request-window','not-an-int'))
        for execute in (False,True):
            for flag,value in invalid:
                execution=Mock()
                with self.subTest(execute=execute,flag=flag,value=value), \
                     patch.object(cli,'load_profile') as load, \
                     patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
                     patch('singularitydog_hw.native_active_transport.load_library') as native, \
                     patch('singularitydog_hw.native_active_transport.ActiveSession') as session, \
                     patch('singularitydog_hw.imu.ICM20948') as imu, \
                     patch.object(cli.subprocess,'run') as audio, \
                     patch.object(cli.Path,'mkdir') as mkdir,redirect_stderr(io.StringIO()):
                    args=self.args() if execute else ['--profile',str(self.path)]
                    with self.assertRaises(SystemExit) as raised:
                        cli.main(args+[flag,value],execution=execution)
                    self.assertEqual(raised.exception.code,2)
                for method in (load,model,native,session,imu,audio,mkdir,execution.bind_profile):method.assert_not_called()
                self.assertFalse(self.out.exists())

    def test_bounded_transport_mismatch_rejected_in_plan_and_execute_before_binding_or_setup(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            profile=self.approved_profile(schema,gap_us=gap,window=window)
            for execute in (False,True):
                for flag,value in (('--request-gap-us','800' if gap==600 else '600'),
                                   ('--request-window','2' if window==3 else '3')):
                    execution=Mock()
                    with self.subTest(schema=schema,execute=execute,flag=flag), \
                         patch.object(cli,'load_profile',return_value=profile) as load, \
                         patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
                         patch('singularitydog_hw.native_active_transport.load_library') as native, \
                         patch('singularitydog_hw.native_active_transport.ActiveSession') as session, \
                         patch('singularitydog_hw.imu.ICM20948') as imu, \
                         patch.object(cli.subprocess,'run') as audio, \
                         patch.object(cli.Path,'mkdir') as mkdir,redirect_stderr(io.StringIO()):
                        args=self.args() if execute else ['--profile',str(self.path)]
                        with self.assertRaises(SystemExit) as raised:
                            cli.main(args+[flag,value],execution=execution)
                        self.assertEqual(raised.exception.code,2)
                    load.assert_called_once_with(str(self.path),require_approved=execute)
                    for method in (model,native,session,imu,audio,mkdir,execution.bind_profile):method.assert_not_called()
                    self.assertFalse(self.out.exists())

    def test_unapproved_real_output_rejected_before_model_and_native(self):
        with patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
             patch('singularitydog_hw.native_active_transport.load_library') as native:
            with self.assertRaises(ProfileError): self.run_quiet(self.args())
        self.assertFalse(self.out.exists()); model.assert_not_called(); native.assert_not_called()

    def test_missing_support_cutoff_or_power_epoch_stops_before_setup(self):
        for flag in ('--support-in-place', '--cutoff-ready', '--power-epoch'):
            args = self.args(); index = args.index(flag); del args[index:index+(2 if flag == '--power-epoch' else 1)]
            with self.subTest(flag=flag), patch.object(cli, 'load_profile', return_value=self.fake), \
                 patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model:
                with self.assertRaises(SystemExit) as raised: self.run_quiet(args)
                self.assertEqual(raised.exception.code, 2); model.assert_not_called()
        self.assertFalse(self.out.exists())

    def test_mismatched_epoch_rejected_before_audio_or_setup(self):
        args = self.args(); args[-1] = 'previous-epoch'
        with patch.object(cli, 'load_profile', return_value=self.fake), \
             patch.object(cli.subprocess, 'run') as audio:
            with self.assertRaises(SystemExit): self.run_quiet(args)
        self.assertFalse(self.out.exists()); audio.assert_not_called()

    def test_audio_digest_mismatch_rejected_before_setup(self):
        args = self.args(); args[args.index('--audio-sha256')+1] = '0'*64
        with patch.object(cli, 'load_profile', return_value=self.fake), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
             patch('singularitydog_hw.native_active_transport.load_library') as native:
            with self.assertRaises(SystemExit): self.run_quiet(args)
        self.assertFalse(self.out.exists()); model.assert_not_called(); native.assert_not_called()

    def test_preexisting_output_not_overwritten(self):
        self.out.mkdir(); (self.out/'report.json').write_text('preserve')
        with patch.object(cli, 'load_profile', return_value=self.fake), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model:
            with self.assertRaises(FileExistsError): self.run_quiet(self.args())
        self.assertEqual((self.out/'report.json').read_text(), 'preserve'); model.assert_not_called()

    def fixed_catch_args(self):
        args=self.args();args.remove('--execute-supported');args.remove('--support-in-place')
        return args+['--execute-fixed-catch','--fixed-catch-ready']

    def test_fixed_catch_invalid_admission_never_takes_terminal_ownership(self):
        profile=dict(self.fake,scope='fixed_catch_current_hold_only')
        cases=('cutoff','power','digest','existing','wrong_scope')
        for case in cases:
            args=self.fixed_catch_args();selected=profile
            if case=='cutoff':args.remove('--cutoff-ready')
            elif case=='power':args[args.index('--power-epoch')+1]='old-power'
            elif case=='digest':args[args.index('--audio-sha256')+1]='0'*64
            elif case=='existing':self.out.mkdir()
            else:selected=self.fake
            try:
                with self.subTest(case=case),patch.object(cli,'load_profile',return_value=selected), \
                     patch('singularitydog_hw.fixed_catch_hold.FixedCatchExecution') as supervisor, \
                     patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model:
                    with self.assertRaises((SystemExit,FileExistsError)):
                        self.run_quiet(args)
                    supervisor.assert_not_called();model.assert_not_called()
            finally:
                if case=='existing':self.out.rmdir()

    def test_fixed_catch_setup_failure_restores_tty_descriptors_and_saves_report(self):
        from singularitydog_hw.fixed_catch_hold import FixedCatchExecution
        master,reader=pty.openpty();writer=os.open(os.ttyname(reader),os.O_RDWR)
        self.addCleanup(os.close,master);self.addCleanup(os.close,reader);self.addCleanup(os.close,writer)
        original=[os.get_blocking(fd) for fd in (reader,writer)];owners=[]
        def supervisor(*args,**kwargs):
            execution=FixedCatchExecution(reader,write_fd=writer);owners.append(execution)
            return execution
        torch=SimpleNamespace(set_num_threads=Mock(),set_num_interop_threads=Mock())
        with patch.object(cli,'load_profile',return_value=dict(self.fake,scope='fixed_catch_current_hold_only')), \
             patch('singularitydog_hw.fixed_catch_hold.FixedCatchExecution',side_effect=supervisor), \
             patch.object(FixedCatchExecution,'bind_profile'),patch.dict('sys.modules',{'torch':torch}), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel',side_effect=RuntimeError('synthetic model failure')), \
             patch('singularitydog_hw.native_active_transport.load_library') as native:
            self.assertEqual(self.run_quiet(self.fixed_catch_args()),2)
        self.assertEqual([os.get_blocking(fd) for fd in (reader,writer)],original)
        self.assertEqual(len(owners),1);self.assertTrue(owners[0].closed)
        saved=json.loads((self.out/'report.json').read_text())
        self.assertEqual(saved['status'],'ABORTED_BEFORE_OUTPUT')
        self.assertIn('synthetic model failure',saved['errors'][0]);native.assert_not_called()

    def test_setup_model_failure_is_preserved_as_before_output_report(self):
        torch = SimpleNamespace(set_num_threads=Mock(), set_num_interop_threads=Mock())
        with patch.object(cli, 'load_profile', return_value=self.fake), patch.dict('sys.modules', {'torch': torch}), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel', side_effect=RuntimeError('synthetic model failure')), \
             patch('singularitydog_hw.native_active_transport.load_library') as native, \
             patch('singularitydog_hw.dual_can_pipeline_benchmark.validate_ports') as ports:
            self.assertEqual(self.run_quiet(self.args()), 2)
        saved = json.loads((self.out/'report.json').read_text())
        self.assertEqual(saved['status'], 'ABORTED_BEFORE_OUTPUT')
        self.assertFalse(saved['motor_enable_sent']); self.assertFalse(saved['learned_targets_sent'])
        self.assertIn('synthetic model failure', saved['errors'][0])
        self.assertEqual(saved['imu_restore_status'], 'not_started')
        self.assertEqual(saved['transport_settings'],self.expected_transport(SCHEMA_V2))
        self.assertEqual((self.out/'report.json').stat().st_mode & 0o777, 0o600)
        native.assert_not_called(); ports.assert_not_called()

    def test_setup_native_failure_also_preserves_evidence_before_ports_open(self):
        torch = SimpleNamespace(set_num_threads=Mock(), set_num_interop_threads=Mock())
        with patch.object(cli, 'load_profile', return_value=self.fake), patch.dict('sys.modules', {'torch': torch}), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel', return_value=object()), \
             patch('singularitydog_hw.native_active_transport.load_library', side_effect=ValueError('synthetic ABI mismatch')), \
             patch('singularitydog_hw.dual_can_pipeline_benchmark.validate_ports') as ports:
            self.assertEqual(self.run_quiet(self.args()), 2)
        saved = json.loads((self.out/'report.json').read_text())
        self.assertIn('synthetic ABI mismatch', saved['errors'][0]); ports.assert_not_called()

    def test_repeated_sigint_is_level_triggered_without_lock_reentrancy(self):
        read_fd, write_fd = os.pipe()
        try:
            state = cli.SignalState(write_fd)
            original_write = os.write
            nested = []
            def write_with_signal(fd, value):
                if not nested:
                    nested.append(True)
                    state.handler(signal.SIGINT, None)
                return original_write(fd, value)
            with patch.object(cli.os, 'write', side_effect=write_with_signal):
                state.handler(signal.SIGINT, None)
            self.assertTrue(state.cancelled)
            self.assertEqual(os.read(read_fd, 2), b'xx')
            state.handler(signal.SIGTERM, None)
            self.assertTrue(state.cancelled)
            self.assertEqual(os.read(read_fd, 1), b'x')
        finally:
            os.close(read_fd); os.close(write_fd)

    def test_full_cancel_pipe_never_blocks_signal_handler(self):
        read_fd, write_fd = os.pipe()
        try:
            state = cli.SignalState(write_fd)
            self.assertFalse(os.get_blocking(write_fd))
            while True:
                try: os.write(write_fd, b'x'*4096)
                except BlockingIOError: break
            state.handler(signal.SIGINT, None)
            state.handler(signal.SIGINT, None)
            self.assertTrue(state.cancelled)
            self.assertEqual(os.read(read_fd, 1), b'x')
        finally:
            os.close(read_fd); os.close(write_fd)

    def test_graceful_request_never_clears_emergency_cancellation(self):
        read_fd, write_fd = os.pipe()
        try:
            state = cli.SignalState(write_fd)
            state.handler(signal.SIGUSR1, None)
            self.assertTrue(state.is_set()); self.assertFalse(state.cancelled)
            state.handler(signal.SIGINT, None)
            state.handler(signal.SIGUSR1, None)
            self.assertTrue(state.cancelled)
        finally:
            os.close(read_fd); os.close(write_fd)


if __name__ == '__main__':
    unittest.main()
