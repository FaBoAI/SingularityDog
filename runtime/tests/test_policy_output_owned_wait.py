"""Synthetic CLI execution; all device/model/audio/native loads are replaced."""
from contextlib import ExitStack, nullcontext, redirect_stdout
import io
import json
import os
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_output as cli
from singularitydog_hw import native_active_transport as native
from singularitydog_hw.policy_live_profile import SCHEMA_V2
import test_policy_output_cli as cli_fixtures
from test_native_active_reusable_wait import WAIT, Library


class PolicyOutputOwnedWaitTests(unittest.TestCase):
    def fixture(self):
        case = cli_fixtures.PolicyOutputCLITests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        return case

    def execute(self, *, selected=True, abort=False, factory_failure=False):
        case = self.fixture()
        profile = case.approved_profile(SCHEMA_V2)
        ports, sessions, calls = [], [], []
        boot = case.base/'boot-id'; boot.write_text(profile['boot_id']+'\n')
        real_open = os.open
        library = Library()
        def wake(fd, target, spin, actual, error, size):
            os.fstat(fd)  # Caller-owned private cancellation pipe is still open.
            calls.append((fd, target, spin))
            actual.contents.value = target+7
            return 0
        library.sda_wait_until = WAIT(wake); library.sda_abi = lambda: 1
        device = Mock(restore_status='restored')
        device.start.return_value = {'source': 'synthetic-only'}
        guard = Mock(boot_id=profile['boot_id'])
        def opened(path, flags, *args, **kwargs):
            return real_open(boot if os.fspath(path)=='/proc/sys/kernel/random/boot_id'
                             else path, flags, *args, **kwargs)
        def port(**kwargs):
            value = cli_fixtures.MockSerialPort(case.base/('fake-port-'+str(len(ports))))
            ports.append(value)
            return value
        def session(*args, **kwargs):
            value = Mock(); sessions.append(value)
            return value
        retained = []
        def runtime(*args, **kwargs):
            if selected:
                callback = kwargs['deadline_wait']; retained.append(callback)
                self.assertEqual(callback(1000), 1007)
                self.assertEqual(callback(2000), 2007)
            else: self.assertNotIn('deadline_wait', kwargs)
            return {'status': 'ABORTED' if abort else 'COMPLETE_SUPPORTED_OUTPUT',
                    'errors': ['synthetic runtime abort'] if abort else [],
                    'motor_enable_sent': False, 'learned_targets_sent': False}
        bindings = {scope: {'path': 'DO_NOT_OPEN_'+scope,
                           'resolved': 'DO_NOT_OPEN_'+scope, 'st_rdev': 0}
                    for scope in ('front', 'rear')}
        with ExitStack() as stack:
            stack.enter_context(patch.object(cli, 'load_profile', return_value=profile))
            stack.enter_context(patch.dict('sys.modules', {'serial': SimpleNamespace(Serial=port),
                'torch': SimpleNamespace(set_num_threads=Mock(), set_num_interop_threads=Mock())}))
            stack.enter_context(patch.object(cli.os, 'open', side_effect=opened))
            stack.enter_context(patch.object(cli.signal, 'signal', return_value=None))
            stack.enter_context(patch('singularitydog_hw.policy_output_model.LivePolicyModel',
                return_value=SimpleNamespace(provenance={'source': 'synthetic-only'}, calls=0)))
            stack.enter_context(patch.object(native, 'load_library', return_value=library))
            stack.enter_context(patch.object(native, 'ActiveSession', side_effect=session))
            stack.enter_context(patch.object(native, 'wait_until',
                                            side_effect=AssertionError('legacy per-call buffers')))
            factory = stack.enter_context(patch.object(native, 'make_owned_waiter',
                **({'side_effect': native.ActiveWaitError('synthetic setup failure')}
                   if factory_failure else {'wraps': native.make_owned_waiter})))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.validate_ports', return_value=bindings))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.binding_matches', return_value=True))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.BootIdentityGuard', return_value=guard))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.pipeline.ownership_locks', side_effect=nullcontext))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.port_lock', side_effect=lambda _: nullcontext()))
            stack.enter_context(patch('singularitydog_hw.policy_observer_live.imu_ownership_lock', side_effect=nullcontext))
            stack.enter_context(patch('singularitydog_hw.imu.ICM20948', return_value=device))
            runner = stack.enter_context(patch('singularitydog_hw.policy_output_runtime.run_supported_policy', side_effect=runtime))
            audio = stack.enter_context(patch.object(cli.subprocess, 'run'))
            flags = ['--absolute-epoch-cadence', '--release-spin-us', '500'] if selected else []
            code = case.run_quiet(case.args()+flags)
        self.assertTrue(all(value.closed for value in ports))
        for value in sessions: value.close.assert_called_once_with()
        device.close.assert_called_once_with(); guard.close.assert_called_once_with()
        audio.assert_not_called()
        if selected:
            self.assertEqual(factory.call_count, 1)
            fd = factory.call_args.args[1]
            with self.assertRaises(OSError): os.fstat(fd)
        else: factory.assert_not_called()
        return code, json.loads((case.out/'report.json').read_text()), runner, calls, retained

    def test_selected_factory_once_callback_reused_inside_fd_scope_and_cleanup(self):
        code, report, runner, calls, retained = self.execute()
        self.assertEqual(code, 0)
        runner.assert_called_once()
        self.assertIs(runner.call_args.kwargs['deadline_wait'], retained[0])
        self.assertEqual([row[1:] for row in calls], [(1000, 500), (2000, 500)])
        self.assertEqual(len({row[0] for row in calls}), 1)
        self.assertFalse(report['actual_policy_output_20ms_verified'])

    def test_unselected_fake_session_execution_keeps_default_without_waiter(self):
        code, report, runner, calls, retained = self.execute(selected=False)
        self.assertEqual(code, 0)
        self.assertNotIn('deadline_wait', runner.call_args.kwargs)
        self.assertEqual(calls, []); self.assertEqual(retained, [])
        self.assertFalse(report['actual_policy_output_20ms_verified'])

    def test_runtime_abort_after_wait_keeps_cleanup_and_does_not_retry(self):
        code, report, runner, calls, _ = self.execute(abort=True)
        self.assertEqual(code, 2)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(len(calls), 2)
        runner.assert_called_once()
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])

    def test_factory_setup_failure_keeps_cleanup_and_never_enters_runtime(self):
        code, report, runner, calls, _ = self.execute(factory_failure=True)
        self.assertEqual(code, 2)
        self.assertEqual(report['status'], 'ABORTED_BEFORE_OUTPUT')
        self.assertIn('synthetic setup failure', ' '.join(report['errors']))
        runner.assert_not_called(); self.assertEqual(calls, [])

    def test_plan_with_explicit_selection_still_does_not_construct_or_load_waiter(self):
        case = self.fixture()
        with patch.object(native, 'load_library') as load, \
             patch.object(native, 'make_owned_waiter') as factory, redirect_stdout(io.StringIO()) as out:
            self.assertEqual(cli.main(['--profile', str(case.path),
                '--absolute-epoch-cadence', '--release-spin-us', '500']), 0)
        load.assert_not_called(); factory.assert_not_called()
        self.assertFalse(json.loads(out.getvalue())['hardware_opened'])


if __name__ == '__main__':
    unittest.main()
