"""Scalar candidate selection cannot bypass the disabled diagnostic boundary."""
import contextlib
import io
import types
import unittest
from unittest.mock import Mock, patch

from test_native_benchmark_cli import NativeBenchmarkCLITests


class ScalarCLITests(NativeBenchmarkCLITests):
    def setUp(self):
        super().setUp()
        self.scalar_policy = object()
        self.scalar_source = {**self.cached_source, 'schema': 'native-scalar-step-diagnostic-loader-v1'}
        self.scalar_loader = Mock(return_value=(self.scalar_policy, self.scalar_source))
        package = types.ModuleType('native_policy_overnight.model_call_fastpath')
        package.__path__ = []
        loader = types.ModuleType('native_policy_overnight.model_call_fastpath.scalar_loader')
        loader.load_file_only_verified = self.scalar_loader
        self.patches.enter_context(patch.dict('sys.modules', {
            package.__name__: package, loader.__name__: loader}))

    def scalar_flags(self):
        return ['--scalar-step-manifest', str(self.root / 'scalar.json'),
                '--scalar-step-manifest-sha256', 'f'*64]

    def test_selects_only_pinned_scalar_model_and_retains_diagnostic_provenance(self):
        self.ready_imu()
        self.normal_collect.return_value = ({'status': 'COMPLETE_DIAGNOSTIC', 'errors': [],
            'motor_enable_sent': False, 'learned_targets_sent': False}, [])
        self.assertEqual(self.call_main(self.policy_args()+self.native_baseline_flags()+self.scalar_flags()), 0)
        self.scalar_loader.assert_called_once_with(str(self.root / 'scalar.json'),
            expected_sha256='f'*64, baseline_manifest=str(self.root / 'native-baseline.json'),
            baseline_sha='b'*64, bundle=str(self.root / 'unused-bundle'))
        self.assertIs(self.observer.call_args.args[0], self.scalar_policy)
        self.cached_loader.assert_not_called(); self.load_policy.assert_not_called()
        report, _ = self.saved()
        self.assertEqual(report['scalar_step_model_source'], self.scalar_source)
        self.assertFalse(report['plan']['enable_available'])
        self.assertFalse(report['plan']['learned_targets_sent'])
        self.assert_closed()

    def test_rejects_incomplete_conflicting_or_noninference_flags_before_ports(self):
        cases = [self.policy_args()+self.scalar_flags(),
            self.policy_args()+self.native_baseline_flags()+self.scalar_flags()[:2],
            self.policy_args()+self.native_baseline_flags()+self.scalar_flags()[2:],
            self.policy_args()+self.native_baseline_flags()+self.scalar_flags()+self.cached_variant_flags(),
            self.acquisition_args()+self.native_baseline_flags()+self.scalar_flags(),
            self.compare_args()+self.native_baseline_flags()+self.scalar_flags(),
            self.policy_args()+self.native_baseline_flags()+self.scalar_flags()+['--cycles','501']]
        for args in cases:
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:self.call_main(args)
                self.assertEqual(error.exception.code, 2)
        self.scalar_loader.assert_not_called()
        self.serial_constructor.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_verification_failure_prevents_ports_imu_and_inference(self):
        self.scalar_loader.side_effect = ValueError('Candidate parity evidence mismatch')
        self.assertEqual(self.call_main(self.policy_args()+self.native_baseline_flags()+self.scalar_flags()), 2)
        self.serial_constructor.assert_not_called(); self.imu_constructor.assert_not_called()
        self.observer.assert_not_called(); self.normal_collect.assert_not_called()
        report, _ = self.saved()
        self.assertTrue(any('parity evidence mismatch' in x for x in report['errors']))

    def test_dry_plan_records_one_startup_plus_500_without_loading_candidate(self):
        args = ['--mode','stop-proxy','--cycles','501','--startup-cycle-allowance','1']
        self.assertEqual(self.call_main(args+self.native_baseline_flags()+self.scalar_flags()), 0)
        self.scalar_loader.assert_not_called(); self.serial_constructor.assert_not_called()


def load_tests(loader, tests, pattern):
    # Reuse the existing fixture without rerunning its inherited test methods.
    return unittest.TestSuite(ScalarCLITests(name) for name in ScalarCLITests.__dict__
                              if name.startswith('test_'))


if __name__ == '__main__':unittest.main()
