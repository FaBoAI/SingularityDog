"""Boundary tests; synthetic files/mocks never qualify hardware or motor output."""
import copy
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import dog_supported_fk_trial as tool


class ForegroundTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name).resolve()
        pins = {}
        for name in ('profile', 'library', 'audio'):
            path = self.base / name; path.write_bytes(b'SYNTHETIC NEVER EXECUTED ' + name.encode())
            pins[name] = (str(path), hashlib.sha256(path.read_bytes()).hexdigest())
        self.argv = []
        self.argv.extend(['--launcher-sha256', hashlib.sha256(Path(tool.__file__).read_bytes()).hexdigest()])
        self.profile_path = pins['profile'][0]
        for name in pins:
            self.argv.extend(['--' + name, pins[name][0], '--' + name + '-sha256', pins[name][1]])
        self.argv.extend(['--front-port', '/dev/serial/by-path/front', '--rear-port', '/dev/serial/by-path/rear',
            '--audio-device', 'synthetic', '--output-root', str(self.base), '--power-epoch', 'SYNTHETIC'])
        self.profile = {'profile_sha256': pins['profile'][1], 'motor_power_epoch': 'SYNTHETIC',
            'request_gap_us': 900, 'request_window': 3, 'period_ms': 20, 'duration_s': 2.,
            'scope': 'supported_characterization_only', 'boot_id': 'SYNTHETIC',
            'blockers': ['SYNTHETIC_UNREVIEWED'], 'output_allowed': False}
        self.load = self.enterContext(patch.object(tool.live, 'load_profile', side_effect=lambda *a, **k: copy.deepcopy(self.profile)))
        self.enterContext(patch.object(tool.fk, 'selected', return_value=True))
        self.enterContext(patch.object(tool.fk, 'plan', return_value={'synthetic': True}))
        self.run = self.enterContext(patch.object(tool, 'run_switched_module'))

    def invoke(self, extra=()):
        with redirect_stdout(io.StringIO()) as stdout:
            code = tool.main(self.argv + list(extra))
        return code, json.loads(stdout.getvalue())

    def test_default_is_file_only_and_does_not_read_boot_or_start_child(self):
        before = set(self.base.iterdir())
        with patch.object(Path, 'read_text', side_effect=AssertionError('No /proc read in PLAN')):
            code, plan = self.invoke()
        self.assertEqual(code, 0); self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['hardware_opened']); self.assertFalse(plan['child_started'])
        self.assertEqual(set(self.base.iterdir()), before); self.run.assert_not_called()
        self.load.assert_called_once_with(self.profile_path, require_approved=False)
        self.assertEqual(plan['command'][8:10], ['--profile', self.profile_path])
        self.assertIn('--absolute-epoch-cadence', plan['command'])
        self.assertEqual(plan['command'][plan['command'].index('--request-gap-us') + 1], '900')
        self.assertEqual(plan['command'][plan['command'].index('--release-spin-us') + 1], '200')
        self.assertEqual(plan['timing_selection']['request_gap_us'], 900)
        self.assertEqual(plan['timing_selection']['release_spin_us'], 200)
        self.assertEqual(plan['timing_selection']['request_window'], 3)
        self.assertEqual(plan['timing_selection']['period_ms'], 20)
        self.assertEqual(plan['timing_selection']['reason_source'], 'existing_default')
        self.assertFalse(plan['timing_selection']['differs_from_foreground_defaults'])
        self.assertTrue(plan['timing_selection']['reason'])
        self.assertFalse(plan['native_phase_pair'])
        self.assertNotIn('--native-phase-pair', plan['command'])

    def test_explicit_candidate_pacing_is_bound_and_recorded_in_file_only_plan(self):
        self.profile['request_gap_us'] = 890
        reason = 'Compare the new 890-us profile with the saved 900-us timing baseline; select spin500 explicitly.'
        before = set(self.base.iterdir())
        with patch.object(Path, 'read_text', side_effect=AssertionError('No /proc read in PLAN')):
            code, plan = self.invoke(['--request-gap-us', '890', '--release-spin-us', '500',
                                      '--timing-selection-reason', reason])
        self.assertEqual(code, 0); self.run.assert_not_called()
        self.assertEqual(set(self.base.iterdir()), before)
        self.load.assert_called_once_with(self.profile_path, require_approved=False)
        self.assertEqual(plan['command'][plan['command'].index('--request-gap-us') + 1], '890')
        self.assertEqual(plan['command'][plan['command'].index('--release-spin-us') + 1], '500')
        self.assertEqual(plan['timing_selection'], {
            'request_gap_us': 890, 'request_window': 3, 'period_ms': 20, 'release_spin_us': 500,
            'reason': reason, 'reason_source': 'explicit_cli', 'differs_from_foreground_defaults': True,
            'foreground_defaults': {'request_gap_us': 900, 'release_spin_us': 200}})
        self.assertFalse(plan['hardware_opened']); self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['native_phase_pair'])
        self.assertNotIn('--native-phase-pair', plan['command'])

    def test_unapproved_native_profile_can_plan_only_its_explicit_spin500_route(self):
        self.profile.update(request_gap_us=890, native_phase_pair=True)
        with patch.object(tool.live, 'native_phase_pair_settings',
                          side_effect=AssertionError('Unapproved PLAN cannot qualify execution')) as proof:
            _, plan = self.invoke(['--request-gap-us', '890', '--release-spin-us', '500',
                                   '--timing-selection-reason', 'Candidate native-owner comparison'])
        self.assertTrue(plan['native_phase_pair'])
        self.assertIn('--native-phase-pair', plan['command'])
        self.assertFalse(plan['profile_reviewed']); self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['child_started']); self.run.assert_not_called(); proof.assert_not_called()

    def test_native_profile_rejects_default_spin200_in_plan(self):
        self.profile.update(request_gap_us=890, native_phase_pair=True)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(['--request-gap-us', '890', '--timing-selection-reason', 'Named comparison'])
        self.run.assert_not_called()

    def test_native_execution_requires_immutable_complete_loader_proof(self):
        self.profile.update(request_gap_us=890, native_phase_pair=True, output_allowed=True, blockers=[])
        with patch.object(tool.live, 'native_phase_pair_settings',
                          side_effect=tool.live.ProfileError('Native phase pair requires complete loader proof')) as proof:
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.invoke(['--execute-supported', '--support-in-place', '--cutoff-ready',
                             '--request-gap-us', '890', '--release-spin-us', '500',
                             '--timing-selection-reason', 'Named comparison'])
        proof.assert_called_once(); self.run.assert_not_called()

    def test_cli_cannot_promote_generic_profile_to_native_phase_pair(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(['--native-phase-pair'])
        self.load.assert_not_called(); self.run.assert_not_called()

    def test_candidate_profile_does_not_silently_change_default_pacing(self):
        self.profile['request_gap_us'] = 890
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke()
        self.run.assert_not_called()

    def test_explicit_gap_must_match_complete_loaded_profile(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(['--request-gap-us', '890', '--timing-selection-reason', 'Named comparison'])
        self.run.assert_not_called()

    def test_both_nondefault_timing_values_require_an_explicit_reason(self):
        for extra, gap in ((['--request-gap-us', '890'], 890), (['--release-spin-us', '500'], 900)):
            with self.subTest(extra=extra):
                self.profile['request_gap_us'] = gap
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke(extra)
                self.run.assert_not_called()

    def test_explicit_default_selection_reason_is_preserved(self):
        _, plan = self.invoke(['--request-gap-us', '900', '--release-spin-us', '200',
                               '--timing-selection-reason', 'Repeat the existing foreground defaults'])
        self.assertEqual(plan['timing_selection']['reason'], 'Repeat the existing foreground defaults')
        self.assertEqual(plan['timing_selection']['reason_source'], 'explicit_cli')
        self.assertFalse(plan['timing_selection']['differs_from_foreground_defaults'])

    def test_request_gap_cli_range_matches_policy_output(self):
        for value in ('599', '5001', '890.0'):
            with self.subTest(value=value), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.invoke(['--request-gap-us', value])
        self.load.assert_not_called(); self.run.assert_not_called()
        for value in (600, 5000):
            with self.subTest(value=value):
                self.profile['request_gap_us'] = value
                _, plan = self.invoke(['--request-gap-us', str(value),
                                       '--timing-selection-reason', 'Synthetic CLI boundary only'])
                self.assertEqual(plan['timing_selection']['request_gap_us'], value)
                self.run.assert_not_called()

    def test_release_spin_cli_choices_match_policy_output(self):
        for value in ('0', '400', '501'):
            with self.subTest(value=value), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.invoke(['--release-spin-us', value])
        self.load.assert_not_called(); self.run.assert_not_called()

    def test_empty_nonprintable_or_excessive_selection_reason_is_rejected(self):
        for reason in (' ', 'comparison\nother', 'x' * 1025):
            with self.subTest(reason=reason[:30]), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.invoke(['--release-spin-us', '500', '--timing-selection-reason', reason])
        self.run.assert_not_called()

    def test_window_and_period_remain_fixed(self):
        for key, value in (('request_window', 2), ('period_ms', 21)):
            original = self.profile[key]
            try:
                self.profile[key] = value
                with self.subTest(key=key), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    self.invoke()
                self.run.assert_not_called()
            finally:
                self.profile[key] = original

    def test_complete_loader_failure_is_not_replaced_by_timing_selection(self):
        self.load.side_effect = tool.live.ProfileError('SYNTHETIC full profile contract rejected')
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(['--request-gap-us', '890', '--release-spin-us', '500',
                         '--timing-selection-reason', 'Named comparison'])
        self.run.assert_not_called()

    def test_missing_physical_confirmation_rejects_before_loading_profile(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(['--execute-supported', '--support-in-place'])
        self.load.assert_not_called(); self.run.assert_not_called()

    def test_wrong_profile_hash_never_reaches_loader(self):
        Path(self.profile_path).write_bytes(b'CHANGED')
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke()
        self.load.assert_not_called(); self.run.assert_not_called()

    def test_wrong_motor_power_epoch_is_not_inferred(self):
        self.profile['motor_power_epoch'] = 'OTHER'
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke()
        self.run.assert_not_called()

    def test_wrong_gap_rejected(self):
        self.profile['request_gap_us'] = 850
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke()
        self.run.assert_not_called()

    def test_same_port_rejected(self):
        self.argv[self.argv.index('--rear-port') + 1] = '/dev/serial/by-path/front'
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke()
        self.run.assert_not_called()

    def test_boot_change_rejects_before_policy_module(self):
        self.profile.update(output_allowed=True, blockers=[])
        with patch.object(Path, 'read_text', return_value='OTHER'), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): self.invoke(['--execute-supported', '--support-in-place', '--cutoff-ready'])
        self.run.assert_not_called()

    def test_existing_switch_and_policy_finally_share_one_foreground_process(self):
        self.profile.update(output_allowed=True, blockers=[], prepare_voltage_before_feedback_publication=True,
                            request_gap_us=890, native_phase_pair=True)
        before = dict(os.environ)
        self.run.side_effect = SystemExit(2)
        with patch.object(Path, 'read_text', return_value='SYNTHETIC'), redirect_stdout(io.StringIO()), \
             patch.object(tool.live, 'native_phase_pair_settings', return_value=True) as proof:
            with self.assertRaises(SystemExit) as error:
                tool.main(self.argv + ['--execute-supported', '--support-in-place', '--cutoff-ready',
                                       '--request-gap-us', '890', '--release-spin-us', '500',
                                       '--timing-selection-reason', 'Named comparison'])
        self.assertEqual(error.exception.code, 2); self.assertEqual(dict(os.environ), before)
        self.assertEqual(self.run.call_args.args[0], 'singularitydog_hw.policy_output')
        self.assertEqual(self.run.call_args.kwargs, {'interval_us': 100})
        self.assertIn('--prepare-voltage-before-feedback-publication', self.run.call_args.args[1])
        self.assertIn('--native-phase-pair', self.run.call_args.args[1])
        child = self.run.call_args.args[1]
        self.assertEqual(child[child.index('--request-gap-us') + 1], '890')
        self.assertEqual(child[child.index('--release-spin-us') + 1], '500')
        self.assertNotIn('--timing-selection-reason', child)
        self.assertEqual(self.load.call_count, 2)
        self.assertEqual(proof.call_count, 2)
        self.assertTrue(all(call.kwargs['require_approved'] for call in self.load.call_args_list))

    def test_terminal_hangup_reaches_existing_term_cancellation_and_restores_handler(self):
        self.profile.update(output_allowed=True, blockers=[])
        import signal
        before_hup = signal.getsignal(signal.SIGHUP)
        called = []
        def run(*args, **kwargs):
            signal.signal(signal.SIGTERM, lambda number, frame: called.append(number))
            signal.getsignal(signal.SIGHUP)(signal.SIGHUP, None)
        before_term = signal.getsignal(signal.SIGTERM)
        self.run.side_effect = run
        try:
            with patch.object(Path, 'read_text', return_value='SYNTHETIC'), redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(self.argv + ['--execute-supported', '--support-in-place', '--cutoff-ready']), 0)
        finally:
            signal.signal(signal.SIGTERM, before_term)
        self.assertEqual(called, [signal.SIGTERM]); self.assertIs(signal.getsignal(signal.SIGHUP), before_hup)


if __name__ == '__main__': unittest.main()
