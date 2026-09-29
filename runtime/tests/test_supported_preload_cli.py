"""Dedicated preload CLI gates, without serial, I2C or motor output."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_output as cli
from singularitydog_hw.policy_live_profile import (
    SCHEMA_V3, SUPPORTED_PRELOAD_5S, cadence_source_hashes, template,
)
import test_policy_output_cli as common


def profile(*, preload=True):
    value = dict(template(schema=SCHEMA_V3), profile_sha256='1'*64,
                 output_allowed=False, blockers=['Synthetic candidate requires admission'],
                 motor_power_epoch='test-epoch')
    if preload:
        value.update(diagnostic_timing_acceptance=SUPPORTED_PRELOAD_5S,
                     duration_s=5., policy_weight=0.)
        value['cadence_source_sha256'] = cadence_source_hashes(value)
    return value


class SupportedPreloadCLITests(unittest.TestCase):
    def quiet_call(self, arguments, data, *, expect_error=None, execution=None):
        output, error = io.StringIO(), io.StringIO()
        with ExitStack() as stack:
            load = stack.enter_context(patch.object(cli, 'load_profile', return_value=data))
            mocks = [stack.enter_context(patch(name)) for name in (
                'singularitydog_hw.policy_output_model.LivePolicyModel',
                'singularitydog_hw.native_active_transport.load_library',
                'singularitydog_hw.native_active_transport.ActiveSession',
                'singularitydog_hw.imu.ICM20948',
            )]
            mocks.append(stack.enter_context(patch.object(cli.subprocess, 'run')))
            mocks.append(stack.enter_context(patch.object(cli.Path, 'mkdir')))
            with redirect_stdout(output), redirect_stderr(error):
                if expect_error is not None:
                    with self.assertRaises(SystemExit) as raised:
                        cli.main(['--profile', '/DO_NOT_READ/profile.json', *arguments], execution=execution)
                    self.assertEqual(raised.exception.code, 2)
                else:
                    self.assertEqual(cli.main(['--profile', '/DO_NOT_READ/profile.json', *arguments]), 0)
            for call in mocks:
                call.assert_not_called()
        if expect_error is not None:
            self.assertIn(expect_error, error.getvalue())
        return output.getvalue(), load

    def test_default_preload_plan_never_loads_model_or_opens_devices(self):
        output, load = self.quiet_call([], profile())
        result = json.loads(output)
        self.assertEqual(result['status'], 'PLAN_ONLY')
        self.assertFalse(result['hardware_opened'])
        self.assertFalse(result['output_allowed'])
        self.assertEqual(result['duration_s'], 5.)
        load.assert_called_once_with('/DO_NOT_READ/profile.json', require_approved=False)

    def test_preload_profile_cannot_use_generic_supported_or_fixed_catch_flag(self):
        for flags in (['--execute-supported'], ['--execute-fixed-catch', '--fixed-catch-ready']):
            with self.subTest(flags=flags):
                _, load = self.quiet_call(flags, profile(), expect_error='must match')
                load.assert_called_once_with('/DO_NOT_READ/profile.json', require_approved=True)

    def test_ordinary_profile_cannot_use_preload_execute_flag(self):
        self.quiet_call(['--execute-supported-preload', '--absolute-epoch-cadence'],
                        profile(preload=False), expect_error='must match')

    def test_preload_requires_absolute_cadence_before_model_or_devices(self):
        self.quiet_call(['--execute-supported-preload'], profile(),
                        expect_error='requires --absolute-epoch-cadence')

    def test_conflicting_executors_fail_before_profile_or_hardware_access(self):
        selections = (
            (['--execute-supported-preload', '--execute-supported'], None),
            (['--execute-supported-preload', '--execute-fixed-catch', '--fixed-catch-ready'], None),
            (['--execute-supported-preload'], Mock()),
        )
        for flags, execution in selections:
            with self.subTest(flags=flags, execution=execution):
                _, load = self.quiet_call(flags, profile(), expect_error='dedicated', execution=execution)
                load.assert_not_called()

    def test_support_and_cutoff_are_still_required(self):
        for readiness in ([], ['--support-in-place'], ['--cutoff-ready']):
            with self.subTest(readiness=readiness):
                self.quiet_call(['--execute-supported-preload', '--absolute-epoch-cadence', *readiness],
                                profile(), expect_error='support or fixed catch and immediate cutoff')

    def test_dedicated_execution_flag_forwards_absolute_runner_selection(self):
        # Reuse the established CLI resource fixture: it owns regular temp
        # files as fake serial FDs, and replaces all hardware constructors.
        helper = common.PolicyOutputCLITests(methodName='test_default_plan_reads_template_without_loading_model_or_opening_devices')
        helper.setUp()
        self.addCleanup(helper.doCleanups)
        original_args = helper.args
        helper.args = lambda: [('--execute-supported-preload' if v == '--execute-supported' else v)
                               for v in original_args()]
        candidate = helper.approved_profile(SCHEMA_V3)
        candidate.update(diagnostic_timing_acceptance=SUPPORTED_PRELOAD_5S,
                         duration_s=5., policy_weight=0.)
        candidate['cadence_source_sha256'] = cadence_source_hashes(candidate)
        code, report, _, runner, _ = helper.execute_mocked_profile(
            candidate, status='COMPLETE_SUPPORTED_OUTPUT', extra_args=['--absolute-epoch-cadence'])
        self.assertEqual(code, 0)
        self.assertTrue(report['absolute_epoch_cadence'])
        self.assertTrue(runner.call_args.kwargs['absolute_epoch_cadence'])
        self.assertEqual(runner.call_args.args[0]['diagnostic_timing_acceptance'], SUPPORTED_PRELOAD_5S)
        self.assertIsNone(runner.call_args.kwargs.get('supervision'))


if __name__ == '__main__':
    unittest.main()
