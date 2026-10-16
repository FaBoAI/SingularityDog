"""File, mock transport and PTY checks of the actual independent source.

Synthetic primary stubs are explicit test seams. They never create a Prepared
execution proof or count as hardware timing, physical support or output review.
"""
import copy
import hashlib
import io
import json
import os
import pty
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch, Mock

from experiments.four_bus_type1 import type1_transport as boxed_transport
from experiments.four_bus_type1 import type1_runner as boxed_runner
from experiments.four_bus_type1.test_type1_30hz import admitted30
from experiments.four_bus_diagnostic.transport_adapter import Group
from . import foreground as F, profile as P, type1_runner as R, type1_transport as T
from . import test_runner_proposal as proposal_cases

HERE = Path(__file__).resolve().parent


class MaterializedRunnerFlowTests(proposal_cases.RunnerProposalTests):
    """Reuse reviewed failure scenarios against the placed actual runner, not diff execution."""
    def setUp(self):
        super().setUp()
        self.proposed = R

    def test_materialized_sources_match_frozen_proposals_without_mutating_current_boxed(self):
        from ._proposal_fixture import reconstruct_proposal
        for label, file, key, diff in (
                ('runner', 'type1_runner.py', 'proposed_runner_sha256', 'type1_runner_hooks.proposed.patch'),
                ('transport', 'type1_transport.py', 'proposed_transport_sha256', 'transport_hooks.proposed.patch')):
            receipt = json.loads((HERE/(label+'-proposal-manifest.json')).read_text())
            self.assertEqual(hashlib.sha256((HERE/file).read_bytes()).hexdigest(), receipt[key])
            current = HERE.parent/'four_bus_type1'/file
            before = current.read_bytes()
            frozen = reconstruct_proposal(HERE/(label+'-proposal-manifest.json'),
                                           HERE/diff, HERE/file, key)
            self.assertEqual(hashlib.sha256(frozen.base).hexdigest(), receipt['base_sha256'])
            self.assertEqual(current.read_bytes(), before)
        self.assertEqual(boxed_transport.KP_CAP, 3.)
        self.assertEqual(T.KP_CAP, 6.)

    def test_new_current_hold_cannot_continue_without_owned_supervisor(self):
        from .test_runner_proposal import synthetic_spec
        with patch.object(R, 'validate_admitted', return_value=synthetic_spec('human_current_hold')), \
             patch.object(P, 'verify_runner_source'):
            with self.assertRaisesRegex(ValueError, 'Dedicated genuine'):
                R.run({'synthetic_test_only': True}, factory=self.factory, imu_read=self.imu,
                    observer=SimpleNamespace(consume=lambda *_a, **_k: None), check_current=self.current, check_cancelled=self.check,
                    cancel_io=self.cancel_io, model_setup=lambda *_a, **_k: None,
                    worker_scope=self.worker_scope, main_scope=self.main_scope,
                    release_wait=self.release_wait, announce=lambda: None, execute=True,
                    backend_usage={'kind': 'injected_file_only_mock'})
        self.assertEqual(self.adapters, {})


class ForegroundNegativeAndWiringTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='human30-independent-foreground-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()

    def cli(self, plan):
        file = self.root/'plan.json'
        raw = (json.dumps(plan, indent=2)+'\n').encode()
        file.write_bytes(raw)
        return SimpleNamespace(plan=str(file), plan_sha256=hashlib.sha256(raw).hexdigest(),
            output=str(self.root/'fresh-output'), audio_device='plughw:SYNTHETIC',
            holder_receipt=None, holder_receipt_sha256=None, timing_evidence=False)

    def plan(self):
        base = admitted30()
        # Genuine numerical boxed contract builder is covered by test_profile;
        # this mock exists only to exercise foreground file/no-capability wiring.
        contract = {'synthetic_contract_only': True}
        return dict(schema=P.SCHEMA, status='PLAN_ONLY', mode='human_current_hold', duration_s=8,
                    contract=contract, contract_sha256=P.canonical_sha256(contract), blockers=[],
                    **P.NO_GRANTS)

    def test_default_plan_never_loads_libraries_torch_terminal_model_or_hardware(self):
        cli = self.cli(self.plan())
        with patch.object(P, 'validate_contract'), patch.object(P, 'prepare_from_files') as primary, \
             patch.object(F.LinuxEnvironment, 'load_libraries') as library, \
             patch.object(F.LinuxEnvironment, 'load_model') as model, \
             patch.object(F.LinuxEnvironment, 'verify_terminal') as terminal:
            result = F.file_plan(cli)
        for mock in (primary, library, model, terminal): mock.assert_not_called()
        self.assertFalse(result['opens_devices'])
        self.assertFalse(result['loads_library_model_or_torch'])
        self.assertFalse(result['output_allowed'])
        self.assertIn('EXECUTION_PRIMARY_FILES_NOT_RECHECKED_IN_DEFAULT_PLAN', result['blockers'])
        self.assertFalse(Path(cli.output).exists())

    def test_plan_hash_mutation_rejected(self):
        cli = self.cli(self.plan()); Path(cli.plan).write_text('{}')
        with self.assertRaises(ValueError): F.file_plan(cli)

    def test_public_grants_and_wrong_mode_or_duration_rejected(self):
        for field, value in (('output_allowed', True), ('admitted', True),
                             ('mode', 'learned_boxed'), ('duration_s', 20)):
            with self.subTest(field=field):
                plan = self.plan(); plan[field] = value
                with patch.object(P, 'validate_contract'), self.assertRaises(ValueError):
                    F.file_plan(self.cli(plan))

    def test_missing_primary_files_rejected_before_terminal_model_or_library(self):
        cli = self.cli(self.plan())
        with patch.object(P, 'prepare_from_files', side_effect=ValueError('Missing named primary human evidence')), \
             patch.object(F.LinuxEnvironment, 'load_model') as model, \
             patch.object(F.LinuxEnvironment, 'load_libraries') as library, \
             patch.object(F.LinuxEnvironment, 'verify_terminal') as terminal, self.assertRaises(ValueError):
            F.prepare(cli)
        for mock in (model, library, terminal): mock.assert_not_called()

    def test_missing_integration_receipt_execution_proof_rejected_before_capabilities(self):
        cli = self.cli(self.plan())
        with patch.object(P, 'prepare_from_files', return_value=object()), \
             patch.object(P, 'runtime_spec', side_effect=ValueError('Execution unavailable')), \
             patch.object(F.LinuxEnvironment, 'load_libraries') as library, self.assertRaisesRegex(ValueError, 'Execution unavailable'):
            F.prepare(cli)
        library.assert_not_called()

    def test_direct_execute_cannot_use_a_genuine_but_plan_only_prepared(self):
        from .test_profile import ContractPlanTests
        fixture = ContractPlanTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        # Explicit test construction of an immutable genuine token holder with
        # no execution token. It never authenticates fake primary files or grants.
        plan = P.build_review_plan(fixture.base)
        value = P.Prepared(P._PREPARED_TOKEN, plan, P.Pins(), {}, {}, None)
        self.assertIs(P.verify_prepared(value), value)
        env = Mock()
        with self.assertRaisesRegex(ValueError, 'Execution unavailable'):
            F.execute(SimpleNamespace(), {'admitted': value}, environment=env)
        self.assertEqual(env.mock_calls, [])

    def test_opaque_runner_refuses_old_admitted_and_plain_dictionary_grants(self):
        for candidate in (admitted30(), {'admitted': True, 'output_allowed': True}):
            with self.subTest(candidate=type(candidate)), self.assertRaises(ValueError):
                P.runtime_spec(candidate)

    def test_human_observer_exact_duration_and_30hz(self):
        clock = admitted30()['control_clock']
        self.assertEqual(F.observer_ticks(2, clock), 62)
        self.assertEqual(F.observer_ticks(8, clock), 242)
        for value in (True, 2., 3, 10, 20):
            with self.subTest(value=value), self.assertRaises(ValueError): F.observer_ticks(value, clock)

    def test_native_limits_pin_all_nonmembers_and_do_not_change_old_adapter(self):
        group = Group('port3', (1, 2, 3))
        contract = {'axes': {str(mid): {'raw_lower_rad': -.03, 'raw_upper_rad': .03} for mid in range(1,13)}}
        bounds, kp, kd = F.transport_limits(contract, group, 'human_current_hold')
        lower, upper, raw_kp, raw_kd = T.session_limits(group, bounds, kp, kd)
        self.assertEqual([raw_kp[mid] for mid in group.ids], [6.]*3)
        self.assertEqual(boxed_transport.KP_CAP, 3.)
        self.assertNotEqual(T.Type1Transport, boxed_transport.Type1Transport)
        for mid in (4,5,6):
            self.assertEqual(raw_kp[mid], 0.)
            self.assertEqual(raw_kd[mid], 0.)
            self.assertEqual((lower[mid], upper[mid]), T.unreachable_window())
        _, kp, kd = F.transport_limits(contract, group, 'human_zero_gain_timing')
        self.assertFalse(any(kp.values()) or any(kd.values()))
        with self.assertRaises(ValueError): F.transport_limits(contract, group, 'learned_boxed')

    def test_piped_terminal_rejected_before_any_native_loading(self):
        with patch.object(os, 'isatty', return_value=False), \
             patch.object(os, 'open') as open_fd, self.assertRaisesRegex(ValueError, 'visible terminal'):
            F.LinuxEnvironment().verify_terminal()
        open_fd.assert_not_called()

    def test_inherited_actual_pty_survives_outer_new_process_session(self):
        """Regression for wrapper setsid(): inherited stdio works without /dev/tty."""
        master, slave = pty.openpty()
        try:
            script = ('import json; from experiments.four_bus_human_hold.foreground import LinuxEnvironment; '
                      'print(json.dumps(LinuxEnvironment().verify_terminal(), sort_keys=True))')
            result = subprocess.run([sys.executable, '-B', '-c', script], start_new_session=True,
                stdin=slave, stdout=slave, stderr=subprocess.PIPE, check=False, timeout=5,
                env={**os.environ, 'PYTHONPATH': str(HERE.parents[1])})
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            observed = os.read(master, 8192).decode()
            self.assertIn('"separate_session_supported": true', observed)
            self.assertIn('"source": "inherited_actual_stdin_stdout"', observed)
            self.assertIn('"audibility_or_visibility_inferred": false', observed)
        finally:
            os.close(master); os.close(slave)

    def test_audio_bind_failure_restores_actual_terminal_and_preserves_primary_error(self):
        from . import supervision
        for broken_close in (False,True):
            with self.subTest(broken_close=broken_close):
                master,slave=pty.openpty();before=os.get_blocking(slave);created=[]
                try:
                    class Player:
                        def __init__(self,*_a,**_k):self.closed=False
                        def close(self):
                            self.closed=True
                            if broken_close:raise OSError('Synthetic audio close failure')
                    def make_supervisor(*_a,**kwargs):
                        value=supervision.HumanSupported30Execution(slave,write_fd=slave,
                            stage_audio=kwargs['stage_audio']);created.append(value);return value
                    with patch('singularitydog_hw.policy_output.HumanStageAudioPlayer',Player), \
                         patch.object(P,'validated_audio',return_value={'clips':{}}), \
                         patch.object(F,'HumanSupported30Execution',side_effect=make_supervisor), \
                         patch.object(supervision,'supervisor_view',side_effect=ValueError('Synthetic primary bind failure')):
                        # Stages are never read by the fake owned audio actor.
                        with patch('singularitydog_hw.policy_output.HUMAN_AUDIO_STAGES',()):
                            with self.assertRaisesRegex(ValueError,'Synthetic primary bind failure') as caught:
                                F.LinuxEnvironment().human_supervisor({'admitted':object(),'audio':{'device':'SYNTHETIC'}})
                    self.assertTrue(created[0].closed)
                    self.assertEqual(os.get_blocking(slave),before)
                    if broken_close:self.assertTrue(caught.exception.__notes__)
                    else:self.assertTrue(created[0].restored)
                finally:
                    os.close(master);os.close(slave)

    def test_summary_accepts_only_complete_independent_mode_and_keeps_stop(self):
        groups = tuple(Group('port'+str(i), tuple(range(i*3+1,i*3+4))) for i in range(4))
        report = {'mode':'human_zero_gain_timing'}
        measured = dict(status=P.COMPLETE_STATUS['human_zero_gain_timing'],normal_ramp_completed=True,
            failure_retained=False, cycles=[{'completed':True,'release_ns':1000}],completed_cycles=1,
            stop_confirmed=True, physical_cutoff_required=False,
            stop_results={group.port: {'confirmed_ids':list(group.ids),'ambiguous_ids':[],
                'faults':{str(mid):0 for mid in group.ids}} for group in groups})
        F.summarize(report, measured, {}, groups)
        self.assertTrue(report['all_cycles_passed'])
        self.assertTrue(report['normal_ramp_completed'])
        self.assertTrue(report['terminal_stop']['stop_confirmed'])
        measured['failure_retained'] = True
        F.summarize(report, measured, {}, groups)
        self.assertFalse(report['all_cycles_passed'])

    def test_partial_stop_is_never_granted_by_normal_completion(self):
        groups = tuple(Group('port'+str(i), tuple(range(i*3+1,i*3+4))) for i in range(4))
        report = {'mode':'human_current_hold'}
        measured = dict(status=P.COMPLETE_STATUS['human_current_hold'], normal_ramp_completed=True,
            failure_retained=False, cycles=[{'completed':True,'release_ns':1000}], completed_cycles=1,
            stop_confirmed=False, physical_cutoff_required=True, stop_results={})
        F.summarize(report, measured, {}, groups)
        self.assertFalse(report['terminal_stop']['stop_confirmed'])
        self.assertTrue(report['physical_cutoff_required'])


if __name__ == '__main__': unittest.main()
