"""Run offline preload fault tests and pin their actual result to source hashes.

This report is software evidence only. It cannot authorize an output profile,
certify contact/load or substitute for a current-power hardware diagnostic.
"""
import argparse
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
RUNNER = Path(__file__).resolve()
RUNTIME_CASE = 'test_supported_preload_runtime.SupportedPreloadRuntimeTests.'
PATH_CASE = 'test_supported_preload_path.SupportedPreloadPathTests.'
PROFILE_CASE = 'test_supported_preload_profile.SupportedPreloadProfileTests.'
CHECK_TESTS = {
    'normal_extend_return_and_stop': (
        RUNTIME_CASE+'test_five_second_path_returns_before_normal_gain_down_and_stops_both_buses',
        RUNTIME_CASE+'test_real_profile_loader_admission_executes_without_accessor_substitution'),
    'fault_stop_both_buses': (RUNTIME_CASE+'test_usb_write_failure_uses_existing_stop_owners',
        RUNTIME_CASE+'test_unconfirmed_stop_never_reports_completion_or_safe_shutdown'),
    'cancellation_stop': (RUNTIME_CASE+'test_operator_cancellation_stops_immediately_without_a_forced_return',),
    'stale_input_stop': (RUNTIME_CASE+'test_stale_imu_and_invalid_gravity_stop_without_inference',
        RUNTIME_CASE+'test_stale_bus_feedback_stops_both_buses'),
    'path_mutation_and_replay_rejected': (PATH_CASE+'test_path_mutation_after_validation_cannot_change_loaded_targets',
        PATH_CASE+'test_repeated_reversed_or_skipped_slots_are_rejected',
        PROFILE_CASE+'test_token_is_not_reusable_with_changed_targets_gains_epoch_or_artifacts'),
    'return_target_and_measured_confirmation': (RUNTIME_CASE+'test_measured_return_is_required_before_gain_down',),
    'wire_reference_quantization_checked': (
        PATH_CASE+'test_float_path_at_limit_cannot_hide_out_of_bounds_encoded_reference',
        PATH_CASE+'test_wire_position_matches_canonical_python_at_grid_boundaries',
        PATH_CASE+'test_wire_position_matches_cpp_byte_encoder_for_complete_signed_branch_path'),
}


class RecordedResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.passed = []

    def addSuccess(self, test):
        super().addSuccess(test)
        self.passed.append(test.id())


def fingerprint(paths):
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def source_inventories():
    # Pin fixture helpers as well as discovered tests. Transitive imports can
    # change assertions or simulated transport without editing the test case.
    tests = sorted((ROOT/'runtime/tests').rglob('*.py'))
    dependencies = sorted(path for path in (ROOT/'runtime/singularitydog_hw').rglob('*')
                          if path.is_file() and path.suffix in ('.py', '.cpp', '.h', '.hpp'))
    return tests, dependencies


def _run_validation():
    from singularitydog_hw.policy_live_profile import cadence_source_hashes, SUPPORTED_PRELOAD_5S
    selection = {'diagnostic_timing_acceptance': SUPPORTED_PRELOAD_5S}
    sources = cadence_source_hashes(selection)
    tests, dependencies = source_inventories()
    test_hashes = fingerprint(tests)
    dependency_hashes = fingerprint(dependencies)
    runner_hash = hashlib.sha256(RUNNER.read_bytes()).hexdigest()
    log = io.StringIO()
    with redirect_stdout(log), redirect_stderr(log):
        suite = unittest.TestLoader().discover(str(ROOT/'runtime/tests'),
                                               pattern='test_supported_preload*.py')
        result = unittest.TextTestRunner(stream=log, verbosity=2,
                                        resultclass=RecordedResult).run(suite)
    errors = []
    if (not result.wasSuccessful() or result.skipped or result.expectedFailures or
            result.testsRun == 0 or len(result.passed) != result.testsRun):
        errors.append('All discovered preload tests must pass without skips or expected failures')
    if sources != cadence_source_hashes(selection):
        errors.append('Runtime sources changed during validation')
    try:
        after_tests, after_dependencies = source_inventories()
        if test_hashes != fingerprint(after_tests):
            errors.append('Test sources or fixture inventory changed during validation')
        if dependency_hashes != fingerprint(after_dependencies):
            errors.append('Runtime dependency sources or inventory changed during validation')
        if runner_hash != hashlib.sha256(RUNNER.read_bytes()).hexdigest():
            errors.append('Validation runner changed during validation')
    except OSError as error:
        errors.append('Cannot revalidate source fingerprints: '+str(error))
    passed = set(result.passed)
    if len(passed) != len(result.passed):
        errors.append('Duplicate test IDs cannot count as independent passing cases')
    checks = {key: all(name in passed for name in required)
              for key, required in CHECK_TESTS.items()}
    if not all(checks.values()):
        errors.append('A required fault/return scenario did not pass')
    text = log.getvalue()
    report = dict(schema='singularitydog.supported-preload-source-validation.v1',
        status='PASS_FILE_ONLY_TESTS' if not errors else 'FAIL_FILE_ONLY_TESTS',
        hardware_opened=False, motor_output_allowed=False, approved_for_runtime=False,
        errors=errors, source_sha256=sources, test_source_sha256=test_hashes,
        dependency_source_sha256=dependency_hashes, runner_sha256=runner_hash,
        test_command='python3 tools/validate_supported_preload_software.py --output REPORT.json',
        test_output_sha256=hashlib.sha256(text.encode()).hexdigest(),
        tests_passed=len(result.passed), tests_run=result.testsRun,
        checks=checks, passed_test_ids=result.passed,
        limitations=['Uses simulated buses and a shared virtual clock',
                    'Not hardware timing, contact, weight-bearing or motor authorization'])
    return report, text


def run_validation():
    original_path = list(sys.path)
    try:
        sys.path[:0] = [str(ROOT/'runtime/tests'), str(ROOT/'runtime')]
        return _run_validation()
    finally:
        sys.path[:] = original_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    log_path = args.output.with_suffix(args.output.suffix+'.log')
    if args.output.exists() or log_path.exists():
        parser.error('Report and log paths must both be new')
    report, text = run_validation()
    with log_path.open('x') as stream:
        stream.write(text)
    report['test_output_file'] = log_path.name
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in
        ('status', 'tests_passed', 'tests_run', 'checks', 'errors', 'hardware_opened')}))
    return 0 if report['status'] == 'PASS_FILE_ONLY_TESTS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
