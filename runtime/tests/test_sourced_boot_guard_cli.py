"""The retained-GIL experiment is opt-in and cannot select motor output."""
import contextlib
import io
import importlib
import json
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench


class SourcedBootGuardCliTests(unittest.TestCase):
    def selected(self):
        return ['--mode', 'stop-proxy', '--supported-disabled', '--v3-voltage-proxy',
                '--cycles', '5', '--provenance-mode', 'supported-policy-probe-2s-rare-jitter-v1',
                '--power-epoch', 'operator-confirmed-test-only',
                '--native-boot-guard-artifact', '/private/test-guard.json',
                '--native-boot-guard-artifact-sha256', 'a'*64]

    def reject(self, argv):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
            bench.main(argv)
        self.assertEqual(caught.exception.code, 2)

    def test_selection_requires_both_path_and_sha(self):
        self.reject(['--cycles', '5', '--native-boot-guard-artifact', '/private/test.json'])
        self.reject(['--cycles', '5', '--native-boot-guard-artifact-sha256', 'a'*64])
        self.reject(['--cycles', '5', '--native-boot-guard-artifact', '/private/test.json',
                     '--native-boot-guard-artifact-sha256', 'A'*64])

    def test_selection_requires_disabled_v3_with_explicit_source_and_power_epoch(self):
        argv = self.selected()
        for flag in ('--supported-disabled', '--v3-voltage-proxy', '--provenance-mode', '--power-epoch'):
            changed = list(argv); index = changed.index(flag)
            del changed[index:index+(2 if flag in ('--provenance-mode', '--power-epoch') else 1)]
            with self.subTest(flag=flag): self.reject(changed)
        for flag in ('--acquisition-only', '--compare-feedback'):
            with self.subTest(flag=flag): self.reject(argv+[flag])
        changed = list(argv); changed[changed.index('--mode')+1] = 'type17'
        self.reject(changed)

    def test_plan_keeps_selection_metadata_without_importing_native_code(self):
        sourced_boot_guard = importlib.import_module('singularitydog_hw.sourced_boot_guard')
        proof = {'scope': 'disabled_stop_proxy_diagnostic_only', 'native_loaded': False}
        with (patch.object(sourced_boot_guard, 'plan_sourced_boot_guard', return_value=proof) as plan,
              patch.object(sourced_boot_guard, 'load_sourced_boot_guard_factory',
                           side_effect=AssertionError('PLAN cannot load native code')),
              patch.object(bench, '_start_source_provenance', return_value=None),
              contextlib.redirect_stdout(io.StringIO()) as stdout):
            self.assertEqual(bench.main(self.selected()), 0)
        value = json.loads(stdout.getvalue())
        self.assertEqual(value['sourced_boot_guard'], proof)
        self.assertFalse(value['enable_available'])
        self.assertFalse(value['learned_targets_sent'])
        self.assertEqual(value['type1_requests_per_cycle'], 0)
        plan.assert_called_once_with({'path': '/private/test-guard.json', 'sha256': 'a'*64})

    def test_artifact_failure_stops_before_execution(self):
        sourced_boot_guard = importlib.import_module('singularitydog_hw.sourced_boot_guard')
        with (patch.object(sourced_boot_guard, 'plan_sourced_boot_guard',
                           side_effect=ValueError('Changed artifact')),
              patch.object(bench.dual, 'validate_ports',
                           side_effect=AssertionError('Do not touch ports'))):
            self.reject(self.selected()+['--execute'])


if __name__ == '__main__': unittest.main()
