"""File provenance only; these tests never open a serial port or IMU."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as benchmark
from singularitydog_hw import policy_live_profile as profiles


class NativePipelineProvenanceTests(unittest.TestCase):
    def test_legacy_invocation_remains_explicitly_unbound(self):
        self.assertIsNone(benchmark._start_source_provenance(None, None))
        report = {'status':'COMPLETE_DIAGNOSTIC','errors':[]}
        original = copy.deepcopy(report)
        benchmark._finish_source_provenance(report, None)
        self.assertEqual(report, original)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(benchmark.main([]), 0)
        plan = json.loads(output.getvalue())
        self.assertNotIn('source_provenance', plan)
        self.assertNotIn('motor_power_epoch', plan)

    def test_epoch_is_explicit_never_inferred(self):
        mode = profiles.SUPPORTED_PRELOAD_5S
        for epoch in (None, '', ' ', ' epoch', 'epoch ', 'x\ny', 4, False, 'x'*257):
            with self.subTest(epoch=epoch), self.assertRaises(ValueError):
                benchmark._start_source_provenance(mode, epoch)
        for mode in (None, 'wrong-mode'):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                benchmark._start_source_provenance(mode, 'new-epoch')

    def test_actual_files_match_extended_profile_source_set(self):
        value = benchmark._start_source_provenance(profiles.SUPPORTED_PRELOAD_5S, 'epoch-current')
        expected = profiles.cadence_source_paths(
            {'diagnostic_timing_acceptance':profiles.SUPPORTED_PRELOAD_5S})
        root = Path(profiles.__file__).resolve().parents[1]
        self.assertEqual(value['cadence_source_sha256'], {
            name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in expected})
        self.assertIn('singularitydog_hw/supported_preload_path.py', expected)
        self.assertNotIn('singularitydog_hw/supported_preload_path.py', profiles.cadence_source_paths())
        self.assertEqual(value['motor_power_epoch'], 'epoch-current')
        self.assertEqual(value['power_epoch_source'], 'explicit_operator_argument_not_hardware_detected')
        self.assertIsNone(value['source_files_unchanged'])
        self.assertFalse(value['output_allowed'])
        self.assertFalse(value['approved_for_runtime'])

    def test_unchanged_files_preserve_status_without_approval(self):
        value = benchmark._start_source_provenance(profiles.SUPPORTED_PRELOAD_5S, 'epoch-current')
        report = {'status':'COMPLETE_DIAGNOSTIC','errors':[]}
        with patch.object(profiles, 'cadence_source_hashes', return_value=value['cadence_source_sha256']):
            benchmark._finish_source_provenance(report, value)
        self.assertEqual(report, {'status':'COMPLETE_DIAGNOSTIC','errors':[]})
        self.assertTrue(value['source_files_unchanged'])
        self.assertFalse(value['approved_for_runtime'])

    def test_human_hold_pins_its_supervisor_without_changing_legacy_sources(self):
        mode = profiles.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S
        value = benchmark._start_source_provenance(mode, 'human-epoch')
        self.assertEqual(value['cadence_source_sha256'], profiles.cadence_source_hashes(
            {'diagnostic_timing_acceptance': mode}))
        self.assertIn('singularitydog_hw/human_supported_hold.py', value['cadence_source_sha256'])
        for legacy in (None, {'diagnostic_timing_acceptance': profiles.SUPPORTED_PRELOAD_5S}):
            self.assertNotIn('singularitydog_hw/human_supported_hold.py', profiles.cadence_source_paths(legacy))
        self.assertFalse(value['output_allowed'])
        self.assertFalse(value['approved_for_runtime'])
        self.assertEqual(value['power_epoch_source'], 'explicit_operator_argument_not_hardware_detected')

    def test_human_epoch_and_source_drift_fail_before_approval(self):
        mode = profiles.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S
        for epoch in (None, '', ' ', ' epoch', 'epoch ', 'x\ny', False):
            with self.subTest(epoch=epoch), self.assertRaises(ValueError):
                benchmark._start_source_provenance(mode, epoch)
        value = benchmark._start_source_provenance(mode, 'human-epoch')
        changed = dict(value['cadence_source_sha256'])
        changed['singularitydog_hw/human_supported_hold.py'] = '0'*64
        report = {'status':'COMPLETE_DIAGNOSTIC','errors':[]}
        with patch.object(profiles, 'cadence_source_hashes', return_value=changed):
            benchmark._finish_source_provenance(report, value)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertFalse(value['source_files_unchanged'])
        self.assertFalse(value['output_allowed'])

    def test_human_plan_only_is_accepted_without_device_access(self):
        mode = profiles.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch.object(benchmark.native, 'load_library') as load:
            self.assertEqual(benchmark.main(['--provenance-mode',mode,'--power-epoch','human-epoch']), 0)
        load.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertEqual(plan['source_provenance']['mode'], mode)
        self.assertFalse(plan['enable_available'])
        self.assertFalse(plan['learned_targets_sent'])
        self.assertFalse(plan['source_provenance']['approved_for_runtime'])

    def test_changed_or_missing_sources_abort_successful_diagnostic(self):
        original = benchmark._start_source_provenance(profiles.SUPPORTED_PRELOAD_5S, 'epoch-current')
        changed = dict(original['cadence_source_sha256'])
        changed['singularitydog_hw/native_pipeline_benchmark.py'] = '0'*64
        for response in (changed, OSError('source removed')):
            value = copy.deepcopy(original)
            report = {'status':'COMPLETE_DIAGNOSTIC','errors':[]}
            kwargs = {'side_effect':response} if isinstance(response, Exception) else {'return_value':response}
            with self.subTest(response=type(response).__name__), patch.object(profiles, 'cadence_source_hashes', **kwargs):
                benchmark._finish_source_provenance(report, value)
            self.assertEqual(report['status'], 'ABORTED')
            self.assertFalse(value['source_files_unchanged'])
            self.assertEqual(len(report['errors']), 1)
            self.assertEqual(value['cadence_source_sha256'], original['cadence_source_sha256'])

    def test_existing_failure_is_not_promoted_by_matching_files(self):
        value = benchmark._start_source_provenance(profiles.SUPPORTED_PRELOAD_5S, 'epoch-current')
        report = {'status':'ABORTED','errors':['injected transport error']}
        with patch.object(profiles, 'cadence_source_hashes', return_value=value['cadence_source_sha256']):
            benchmark._finish_source_provenance(report, value)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['errors'], ['injected transport error'])

    def test_plan_only_pins_sources_but_never_marks_runtime_approval(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch.object(benchmark.native, 'load_library') as load:
            result = benchmark.main(['--provenance-mode',profiles.SUPPORTED_PRELOAD_5S,
                                     '--power-epoch','new-power-epoch'])
        self.assertEqual(result, 0)
        load.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertFalse(plan['enable_available'])
        self.assertFalse(plan['learned_targets_sent'])
        self.assertIsNone(plan['source_provenance']['source_files_unchanged'])
        self.assertFalse(plan['source_provenance']['approved_for_runtime'])

    def test_invalid_cli_epoch_fails_before_any_device_access(self):
        for args in (['--provenance-mode',profiles.SUPPORTED_PRELOAD_5S],
                     ['--power-epoch','epoch-without-mode']):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), \
                    patch.object(benchmark.native, 'load_library') as load, self.assertRaises(SystemExit) as error:
                benchmark.main(args)
            self.assertEqual(error.exception.code, 2)
            load.assert_not_called()

    def test_saved_failed_report_binds_real_files_and_caller_epoch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            uids = root/'uids.json'
            uids.write_text('{}')
            output = root/'result'
            with contextlib.redirect_stdout(io.StringIO()), \
                    patch.object(benchmark.native, 'load_library', side_effect=RuntimeError('fixture stops before devices')):
                result = benchmark.main(['--execute','--mode','type17','--acquisition-only',
                    '--front-port','unused-front','--rear-port','unused-rear',
                    '--expected-uids',str(uids),'--library','unused','--output',str(output),
                    '--provenance-mode',profiles.SUPPORTED_PRELOAD_5S,'--power-epoch','epoch-recorded'])
            self.assertEqual(result, 2)
            report = json.loads((output/'report.json').read_text())
            self.assertEqual(report['status'], 'ABORTED')
            self.assertEqual(report['motor_power_epoch'], 'epoch-recorded')
            self.assertEqual(report['cadence_source_sha256'], profiles.cadence_source_hashes(
                {'diagnostic_timing_acceptance':profiles.SUPPORTED_PRELOAD_5S}))
            self.assertTrue(report['source_provenance']['source_files_unchanged'])
            self.assertFalse(report['source_provenance']['approved_for_runtime'])
            self.assertEqual(json.loads((output/'records.json').read_text()), [])


if __name__ == '__main__':
    unittest.main()
