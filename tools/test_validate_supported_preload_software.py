"""Evidence-runner fault gates; mocked child results avoid rerunning motor mocks."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from tools import validate_supported_preload_software as runner


class ValidateSupportedPreloadSoftwareTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.tests = self.base/'runtime/tests'
        self.tests.mkdir(parents=True)
        self.test = self.tests/'test_supported_preload_case.py'
        self.test.write_text('# synthetic selected test\n')
        self.helper = self.tests/'fixture_helper.py'
        self.helper.write_text('# helper imported by a selected test\n')
        self.dependency = self.base/'runtime/singularitydog_hw/policy_motion_envelope.py'
        self.dependency.parent.mkdir(parents=True)
        self.dependency.write_text('# transitive runtime safety dependency\n')
        self.runner_source = self.base/'validate.py'
        self.runner_source.write_text('# synthetic runner pin\n')
        self.required = sorted({test for tests in runner.CHECK_TESTS.values() for test in tests})

    def result(self, **updates):
        data = dict(passed=self.required.copy(), testsRun=len(self.required),
                    skipped=[], expectedFailures=[], wasSuccessful=lambda: True)
        data.update(updates)
        return SimpleNamespace(**data)

    def validate(self, *, result=None, during=None, changed_cadence=False):
        result = result or self.result()
        def child(text_runner, suite):
            self.assertIsInstance(suite, unittest.TestSuite)
            text_runner.stream.write('Synthetic child log, no hardware opened.\n')
            if during:
                during()
            return result
        with ExitStack() as stack:
            stack.enter_context(patch.object(runner, 'ROOT', self.base))
            stack.enter_context(patch.object(runner, 'RUNNER', self.runner_source))
            pins = {'pinned.py': 'a'*64}
            after = {'pinned.py': 'b'*64} if changed_cadence else pins
            stack.enter_context(patch.object(live, 'cadence_source_hashes', side_effect=[pins, after]))
            stack.enter_context(patch.object(unittest.TestLoader, 'discover', return_value=unittest.TestSuite()))
            stack.enter_context(patch.object(unittest.TextTestRunner, 'run', autospec=True, side_effect=child))
            before_path = list(sys.path)
            report, log = runner.run_validation()
            self.assertEqual(sys.path, before_path)
        return report, log

    def assert_failed(self, report, phrase):
        self.assertEqual(report['status'], 'FAIL_FILE_ONLY_TESTS')
        self.assertIn(phrase, ' '.join(report['errors']))
        self.assertFalse(report['hardware_opened'])
        self.assertFalse(report['motor_output_allowed'])
        self.assertFalse(report['approved_for_runtime'])

    def test_only_all_passing_complete_cases_produce_software_evidence(self):
        report, log = self.validate()
        self.assertEqual(report['status'], 'PASS_FILE_ONLY_TESTS')
        self.assertEqual(report['errors'], [])
        self.assertTrue(all(report['checks'].values()))
        self.assertEqual(report['tests_passed'], len(self.required))
        self.assertEqual(report['test_output_sha256'], hashlib.sha256(log.encode()).hexdigest())
        self.assertIn('runtime/tests/fixture_helper.py', report['test_source_sha256'])
        self.assertIn('runtime/singularitydog_hw/policy_motion_envelope.py', report['dependency_source_sha256'])

    def test_failure_skip_expected_failure_or_zero_tests_cannot_pass(self):
        for change in (
            {'wasSuccessful': lambda: False},
            {'skipped': [('case', 'reason')]},
            {'expectedFailures': [('case', 'expected but not accepted')]},
            {'testsRun': 0, 'passed': []},
            {'testsRun': len(self.required)+1},
        ):
            with self.subTest(change=change):
                report, _ = self.validate(result=self.result(**change))
                self.assert_failed(report, 'must pass')

    def test_missing_case_or_same_method_in_wrong_class_cannot_satisfy_gate(self):
        for replacement in (None, 'unrelated.WrongClass.'+self.required[0].rsplit('.', 1)[-1]):
            passed = self.required[1:]
            if replacement:
                passed.append(replacement)
            with self.subTest(replacement=replacement):
                report, _ = self.validate(result=self.result(passed=passed, testsRun=len(passed)))
                self.assert_failed(report, 'required fault/return scenario')

    def test_duplicate_case_ids_are_not_independent_proof(self):
        passed = [*self.required, self.required[0]]
        report, _ = self.validate(result=self.result(passed=passed, testsRun=len(passed)))
        self.assert_failed(report, 'Duplicate test IDs')

    def test_changed_cadence_sources_cannot_pass(self):
        report, _ = self.validate(changed_cadence=True)
        self.assert_failed(report, 'Runtime sources changed')

    def test_changed_or_added_test_helper_cannot_pass(self):
        for mutate in (lambda: self.helper.write_text('# changed helper\n'),
                       lambda: (self.tests/'another_helper.py').write_text('# new helper\n'),
                       lambda: self.test.unlink()):
            with self.subTest(mutate=mutate):
                report, _ = self.validate(during=mutate)
                self.assert_failed(report, 'fixture inventory changed')

    def test_changed_transitive_runtime_dependency_cannot_pass(self):
        report, _ = self.validate(during=lambda: self.dependency.write_text('# changed safety limits\n'))
        self.assert_failed(report, 'Runtime dependency sources')

    def test_changed_runner_cannot_claim_its_new_hash_for_old_execution(self):
        old = hashlib.sha256(self.runner_source.read_bytes()).hexdigest()
        report, _ = self.validate(during=lambda: self.runner_source.write_text('# altered runner\n'))
        self.assert_failed(report, 'Validation runner changed')
        self.assertEqual(report['runner_sha256'], old)

    def test_report_or_log_existing_prevents_tests_and_never_overwrites(self):
        for existing_log in (False, True):
            output = self.base/('result-'+str(existing_log)+'.json')
            existing = output.with_suffix('.json.log') if existing_log else output
            existing.write_text('preserve this evidence')
            with self.subTest(existing_log=existing_log), \
                 patch.object(runner, 'run_validation') as validate, redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    runner.main(['--output', str(output)])
                self.assertEqual(raised.exception.code, 2)
                validate.assert_not_called()
            self.assertEqual(existing.read_text(), 'preserve this evidence')

    def test_failure_report_is_saved_and_returns_failure_exit_status(self):
        report, log = self.validate(result=self.result(skipped=[('case', 'skip')]))
        output = self.base/'failed-report.json'
        with patch.object(runner, 'run_validation', return_value=(report, log)), redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(['--output', str(output)]), 1)
        saved = json.loads(output.read_text())
        self.assertEqual(saved['status'], 'FAIL_FILE_ONLY_TESTS')
        self.assertEqual((self.base/saved['test_output_file']).read_text(), log)


if __name__ == '__main__':
    unittest.main()
