"""Synthetic saved-timestamp analysis; no individual device logs or model calls."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from analyze_policy_cycle_waits import analyze, load_report, main

MS = 1_000_000
B = 1_000_000_000


def fixture(new=False):
    trace = {'index': 3, 'stage': 'motion_envelope', 'release_ns': B,
             'begin_ns': B + 10_000, 'hold_checked_ns': B + 20_000,
             'previous_candidate_ns': B - 9*MS, 'previous_sample_start_ns': B - 20*MS,
             'acquisition_complete_ns': B + 11*MS, 'sample_start_ns': B + MS,
             'policy_call_begin_ns': B + 12*MS, 'policy_call_return_ns': B + 14*MS,
             'target_ready_ns': B + 14*MS + 100_000,
             'voltage_owner_validated_ns': B + 10*MS,
             'voltage_join_complete_ns': B + 14*MS + 200_000,
             'candidate_ns': B + 14*MS + 300_000,
             'output_submit_ns': None, 'output_return_ns': None, 'cycle_end_ns': None,
             'command_gap_basis': 'validated_target_computation_not_transport_write'}
    if new:
        trace.update(feedback_collect_begin_ns=B + 100_000,
                     feedback_collect_end_ns=B + 8*MS,
                     imu_wait_begin_ns=B + 8*MS + 100_000,
                     imu_wait_end_ns=B + 10*MS + 900_000,
                     imu_read_started_ns=B + 2*MS, imu_read_finished_ns=B + 10*MS)
    return {'failed_cycle_timing': trace,
            'journal': [{'phase': 'feedback_hold', 'bus': bus,
                         'stats': {'begin_ns': B + 200_000, 'end_ns': B + end*MS}}
                        for bus,end in (('front',7),('rear',8))]}


def combined_fixture():
    report = fixture(new=True)
    report['failed_cycle_timing'].update(
        combined_acquisition_wait_begin_ns=B + 100_000,
        combined_acquisition_wait_end_ns=B + 10*MS + 100_000,
        feedback_collect_begin_ns=B + 10*MS + 200_000,
        feedback_collect_end_ns=B + 10*MS + 300_000,
        imu_wait_begin_ns=B + 10*MS + 400_000,
        imu_wait_end_ns=B + 10*MS + 500_000)
    return report


class CycleWaitTests(unittest.TestCase):
    def test_old_log_keeps_unmeasured_waits_none_and_gaps_separate(self):
        result = analyze(fixture())
        values = result['intervals_ms']
        self.assertIsNone(values['feedback_collect_wait_ms'])
        self.assertIsNone(values['imu_future_wait_ms'])
        self.assertIsNone(values['imu_read_ms'])
        self.assertIsNone(values['combined_acquisition_wait_ms'])
        self.assertIsNone(values['combined_wait_end_to_feedback_collect_begin_ms'])
        self.assertIsNone(values['combined_wait_end_to_acquisition_complete_ms'])
        self.assertIsNone(values['last_bus_native_end_to_combined_wait_end_ms'])
        self.assertIsNone(values['imu_read_finish_to_combined_wait_end_ms'])
        self.assertIsNone(result['timing_ns']['combined_acquisition_wait_begin_ns'])
        self.assertIsNone(result['timing_ns']['combined_acquisition_wait_end_ns'])
        self.assertEqual(values['command_interval_ms'], 23.3)
        self.assertEqual(values['sample_interval_ms'], 21.)
        self.assertEqual(values['policy_call_ms'], 2.)
        self.assertEqual(values['last_bus_native_end_to_acquisition_complete_ms'], 3.)
        self.assertEqual(result['command_gap_basis'], fixture()['failed_cycle_timing']['command_gap_basis'])
        self.assertFalse(result['root_cause_established'])
        self.assertFalse(result['output_allowed'])

    def test_combined_readiness_wait_is_separate_from_ready_result_takeout(self):
        result = analyze(combined_fixture())
        values = result['intervals_ms']
        self.assertEqual(values['combined_acquisition_wait_ms'], 10.)
        self.assertEqual(values['combined_wait_end_to_feedback_collect_begin_ms'], .1)
        self.assertEqual(values['combined_wait_end_to_acquisition_complete_ms'], .9)
        self.assertEqual(values['last_bus_native_end_to_combined_wait_end_ms'], 2.1)
        self.assertEqual(values['imu_read_finish_to_combined_wait_end_ms'], .1)
        self.assertEqual(values['feedback_collect_wait_ms'], .1)
        self.assertEqual(values['feedback_rows_preparation_ms'], .1)
        self.assertEqual(values['feedback_to_imu_result_takeout_ms'], .1)
        self.assertEqual(values['imu_future_wait_ms'], .1)
        self.assertEqual(values['imu_read_ms'], 8.)
        self.assertIn('rows merge follows IMU takeout', result['acquisition_wait_layout'])
        self.assertFalse(result['root_cause_established'])

    def test_partial_combined_wait_keeps_unreached_end_and_takeout_unknown(self):
        report = combined_fixture(); trace = report['failed_cycle_timing']
        for key in list(trace):
            if key.endswith('_ns') and key not in (
                    'release_ns', 'begin_ns', 'hold_checked_ns',
                    'previous_candidate_ns', 'previous_sample_start_ns',
                    'combined_acquisition_wait_begin_ns'):
                trace[key] = None
        trace['stage'] = 'input_acquisition'
        result = analyze(report)
        self.assertEqual(result['timing_ns']['combined_acquisition_wait_begin_ns'], B+100_000)
        for name in ('combined_acquisition_wait_ms',
                     'combined_wait_end_to_feedback_collect_begin_ms',
                     'combined_wait_end_to_acquisition_complete_ms',
                     'feedback_collect_wait_ms', 'imu_future_wait_ms'):
            self.assertIsNone(result['intervals_ms'][name], name)

    def test_combined_wait_rejects_reversed_or_late_or_invalid_stamps(self):
        mutations = [('combined_acquisition_wait_begin_ns', B),
                     ('combined_acquisition_wait_end_ns', B+50_000),
                     ('combined_acquisition_wait_end_ns', B+10*MS+250_000),
                     ('combined_acquisition_wait_begin_ns', True),
                     ('combined_acquisition_wait_end_ns', 1.5)]
        for key,value in mutations:
            with self.subTest(key=key, value=value):
                report = combined_fixture(); report['failed_cycle_timing'][key] = value
                with self.assertRaises(ValueError): analyze(report)

    def test_combined_success_boundary_cannot_precede_known_worker_completion(self):
        for worker in ('imu', 'front', 'rear'):
            with self.subTest(worker=worker):
                report = combined_fixture()
                after_ready = B+10*MS+150_000
                if worker == 'imu':
                    report['failed_cycle_timing']['imu_read_finished_ns'] = after_ready
                else:
                    next(row for row in report['journal'] if row['bus']==worker)['stats']['end_ns'] = after_ready
                with self.assertRaisesRegex(ValueError, 'combined'):
                    analyze(report)

    def test_complete_trace_exposes_different_waits_without_inventing_a_cause(self):
        result = analyze(fixture(new=True))
        values = result['intervals_ms']
        self.assertEqual(values['feedback_collect_wait_ms'], 7.9)
        self.assertEqual(values['feedback_rows_preparation_ms'], .1)
        self.assertEqual(values['imu_future_wait_ms'], 2.8)
        self.assertEqual(values['imu_read_ms'], 8.)
        self.assertEqual(values['imu_read_finish_to_future_return_ms'], .9)
        self.assertEqual(values['imu_future_return_to_acquisition_complete_ms'], .1)
        self.assertEqual(result['feedback_native_by_bus']['front']['native_end_to_feedback_collect_end_ms'], 1.)
        self.assertEqual(result['feedback_native_by_bus']['rear']['native_end_to_feedback_collect_end_ms'], 0.)

    def test_partial_collection_failure_keeps_later_stages_none(self):
        report = fixture(new=True); trace = report['failed_cycle_timing']
        for key in list(trace):
            if key.endswith('_ns') and key not in ('release_ns', 'begin_ns', 'hold_checked_ns',
                'previous_candidate_ns', 'previous_sample_start_ns', 'feedback_collect_begin_ns'):
                trace[key] = None
        trace['stage'] = 'input_acquisition'
        result = analyze(report)
        for name in ('feedback_collect_wait_ms', 'imu_future_wait_ms', 'command_interval_ms',
                     'last_bus_native_end_to_acquisition_complete_ms'):
            self.assertIsNone(result['intervals_ms'][name], name)
        self.assertTrue(all(row['native_end_ns'] is None for row in result['feedback_native_by_bus'].values()))

    def test_invalid_and_noncausal_stamps_are_rejected_without_clamping(self):
        mutations = [('feedback_collect_end_ns', B), ('imu_read_finished_ns', B + 12*MS),
                     ('sample_start_ns', B - 30*MS), ('candidate_ns', True),
                     ('imu_read_started_ns', 1.5), ('policy_call_return_ns', -1)]
        for key,value in mutations:
            with self.subTest(key=key):
                report = fixture(new=True); report['failed_cycle_timing'][key] = value
                with self.assertRaises(ValueError): analyze(report)

    def test_journal_only_selects_this_cycle_and_cannot_hide_negative_residual(self):
        report = fixture(new=True)
        report['journal'].insert(0, {'phase':'feedback_hold','bus':'front',
                                   'stats':{'begin_ns':B-20*MS,'end_ns':B-12*MS}})
        report['journal'].append({'phase':'overlapped_voltage','bus':'rear',
                                 'stats':{'begin_ns':B+8*MS,'end_ns':B+13*MS}})
        self.assertEqual(analyze(report)['feedback_native_by_bus']['front']['native_end_ns'], B+7*MS)
        report['journal'][1]['stats']['end_ns'] = B+12*MS
        with self.assertRaisesRegex(ValueError, 'native end'): analyze(report)

    def test_missing_and_ambiguous_feedback_journal(self):
        report = fixture(); report['journal'] = []
        self.assertIsNone(analyze(report)['intervals_ms']['last_bus_native_end_to_acquisition_complete_ms'])
        report = fixture(); report['journal'].append(copy.deepcopy(report['journal'][0]))
        with self.assertRaisesRegex(ValueError, 'Ambiguous'): analyze(report)

    def test_sha_checked_before_json_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'report.json'; raw = json.dumps(fixture()).encode(); path.write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            doc, actual = load_report(path, digest)
            self.assertEqual(actual, digest); self.assertEqual(doc, fixture())
            for sha in ('0'*64, 'A'*64, 'bad'):
                with self.subTest(sha=sha), self.assertRaises(ValueError): load_report(path, sha)

    def test_cli_emits_json_only_for_success_or_failure_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'report.json'; path.write_text(json.dumps(fixture()))
            original=path.read_bytes()
            for argv, code in ((['--report',str(path)],0),
                               (['--report',str(path),'--expected-sha256','0'*64],2),([],2)):
                stream=io.StringIO()
                with contextlib.redirect_stdout(stream): self.assertEqual(main(argv),code)
                result=json.loads(stream.getvalue()); self.assertFalse(result['output_allowed'])
            self.assertEqual(path.read_bytes(),original)
            self.assertEqual(list(Path(directory).iterdir()),[path])

    def test_missing_trace_and_nonfinite_json_are_rejected(self):
        with self.assertRaises(ValueError): analyze({'journal':[]})
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'report.json'; path.write_text('{"failed_cycle_timing":NaN}')
            with self.assertRaises(ValueError): load_report(path)

    def test_saved_report_rejects_directories_symlinks_and_oversize_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, 'regular'): load_report(root)
            path = root/'report.json'; path.write_text(json.dumps(fixture()))
            link = root/'link.json'; link.symlink_to(path)
            with self.assertRaisesRegex(ValueError, 'regular'): load_report(link)
            # Sparse synthetic file checks the limit before reading its bytes.
            large = root/'large.json'
            with large.open('wb') as stream: stream.truncate(128*1024*1024+1)
            with self.assertRaisesRegex(ValueError, '128 MiB'): load_report(large)


if __name__ == '__main__': unittest.main()
