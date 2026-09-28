"""File-only checks for the conservative 20 ms STOP-proxy comparison."""
import copy
import hashlib
import json
import unittest
from unittest.mock import patch

from tools.compare_native_20ms import (
    OBSERVED_R17_POLICY, compare, compare_sources, summarize,
)


def report(*, long_cycle=None, long_reply=None, long_interval=None):
    rows = []
    for index in range(500):
        whole = 21.2 if index == long_cycle else 18.8
        reply = 20.6 if index == long_reply else 18.3
        interval = None if index == 0 else 21.3 if index == long_interval else 20.0
        rows.append({'acquisition_ms': 7.0, 'prepare_ms': .2,
                     'inference_ms': 3.8,
                     'oldest_input_to_final_host_write_ms': 15.7,
                     'oldest_input_to_last_reply_ms': reply,
                     'whole_iteration_ms': whole,
                     'actual_release_interval_ms': interval,
                     'iteration_deadline_met': whole <= 20})
    return {'status': 'COMPLETE_DIAGNOSTIC', 'errors': [], 'mode': 'stop-proxy',
            'cycles_requested': 500, 'cycles_completed': 500,
            'motor_enable_sent': False, 'learned_targets_sent': False,
            'approved_for_runtime': False, 'full_controller_50Hz_verified': False,
            'imu_restore_status': 'restored', 'boot_id': 'boot-a',
            'plan': {'request_gap_us': 800, 'window': 3},
            'measurements': rows}


class ComparisonTests(unittest.TestCase):
    def test_reports_all_deadlines_separately(self):
        baseline = report(long_cycle=14, long_reply=28, long_interval=30)
        result = compare(baseline, report())
        self.assertEqual(result['baseline']['deadline_misses'], {
            'oldest_input_to_final_host_write': 0,
            'oldest_input_to_last_reply': 1,
            'whole_iteration': 1,
            'release_intervals_over_21ms': 1,
            'start_intervals_over_20ms': 1})
        self.assertTrue(result['diagnostic_gate_pass'])
        self.assertFalse(result['baseline']['strict_start_interval_20ms_met'])
        self.assertFalse(result['live_policy_20ms_verified'])
        self.assertTrue(result['same_boot'])

    def test_different_boot_is_visible(self):
        candidate = report()
        candidate['boot_id'] = 'boot-b'
        self.assertFalse(compare(report(), candidate)['same_boot'])

    def test_request_count_and_start_cadence_are_separate(self):
        baseline = report()
        baseline['plan']['input_workers'] = ['front6', 'rear6', 'IMU']
        candidate = report()
        candidate['plan']['requests_per_cycle'] = 26
        candidate['measurements'][1]['actual_release_interval_ms'] = 20.01
        result = compare(baseline, candidate)
        self.assertEqual(result['baseline']['requests_per_cycle'], 24)
        self.assertEqual(result['candidate']['requests_per_cycle'], 26)
        self.assertFalse(result['same_request_count'])
        self.assertTrue(result['diagnostic_gate_pass'])
        self.assertFalse(result['candidate']['strict_start_interval_20ms_met'])

    def test_refuses_short_or_active_runs(self):
        for change in ({'cycles_completed': 499}, {'motor_enable_sent': True},
                       {'learned_targets_sent': True}, {'status': 'ABORTED'}):
            bad = report()
            bad.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                summarize(bad)

    def test_refuses_fake_deadline_flag_or_missing_restoration(self):
        bad = report()
        bad['measurements'][3]['iteration_deadline_met'] = False
        with self.assertRaises(ValueError):
            summarize(bad)
        bad = report()
        bad['main_thread_affinity'] = {'requested_cpu': 4, 'restored': False}
        with self.assertRaises(ValueError):
            summarize(bad)

    def test_no_mutation(self):
        source = report()
        before = copy.deepcopy(source)
        summarize(source)
        self.assertEqual(source, before)


def startup_report(*, steady_miss=None, stale_reply=None):
    result = report(long_cycle=steady_miss, long_reply=stale_reply)
    for row in result['measurements']:
        row.update(timing_phase='steady', release_lateness_ms=.01,
                   scheduled_completion_slack_ms=1.1, skipped_slots_before=0)
    result['measurements'][0]['actual_release_interval_ms'] = 20.654463
    result['measurements'][349].update(actual_release_interval_ms=21.552385,
                                      release_lateness_ms=1.566252,
                                      scheduled_completion_slack_ms=-.357238)
    first = copy.deepcopy(result['measurements'][0])
    first.update(timing_phase='startup', whole_iteration_ms=20.60489,
                 iteration_deadline_met=False, actual_release_interval_ms=None)
    result['measurements'].insert(0, first)
    result.update(cycles_requested=501, cycles_completed=501)
    result['plan']['startup_cycle_allowance'] = 1
    return result


class ObservedCadenceAcceptanceTests(unittest.TestCase):
    @staticmethod
    def encode(result):
        return json.dumps(result, sort_keys=True).encode()

    def assess_fixture(self, result):
        raw = self.encode(result)
        # Test fixtures replace the production allowlist only inside this test.
        # The command-line interface cannot introduce a new accepted digest.
        with patch('tools.compare_native_20ms.OBSERVED_R17_REPORTS',
                   frozenset((hashlib.sha256(raw).hexdigest(),))):
            return compare_sources(raw, raw, acceptance_policy=OBSERVED_R17_POLICY)

    def test_explicit_policy_accepts_observed_cadence_and_preserves_strict_failures(self):
        source = startup_report()
        before = copy.deepcopy(source)
        result = self.assess_fixture(source)
        acceptance = result['operator_acceptance']
        self.assertTrue(acceptance['accepted_for_diagnostic_continuation'])
        self.assertFalse(acceptance['future_reports_covered'])
        self.assertFalse(acceptance['runtime_safety_limits_changed'])
        self.assertFalse(acceptance['approved_for_runtime'])
        self.assertFalse(acceptance['live_policy_20ms_verified'])
        candidate = result['candidate']
        self.assertFalse(candidate['stop_proxy_diagnostic_gate'])
        self.assertFalse(candidate['strict_start_interval_20ms_met'])
        self.assertEqual(candidate['cadence_observations']['max_release_lateness_ms'], 1.566252)
        self.assertEqual(candidate['cadence_observations']['scheduled_deadline_misses'], 1)
        self.assertEqual(candidate['max_release_interval_ms'], 21.552385)
        self.assertEqual(candidate['startup']['whole_iteration_ms'], 20.60489)
        self.assertEqual(candidate['startup']['total_cycles'], 501)
        self.assertTrue(candidate['startup']['all_cycles_retained'])
        self.assertEqual(source, before)

    def test_startup_exception_is_never_inferred(self):
        source = startup_report()
        raw = self.encode(source)
        with self.assertRaises(ValueError):
            compare_sources(raw, raw)
        source['plan']['startup_cycle_allowance'] = 0
        with self.assertRaises(ValueError):
            summarize(source, startup_cycle_allowance=1)
        source['plan']['startup_cycle_allowance'] = 1
        source['measurements'][2]['timing_phase'] = 'startup'
        with self.assertRaises(ValueError):
            summarize(source, startup_cycle_allowance=1)

    def test_later_processing_and_oldest_input_misses_are_not_waived(self):
        for source in (startup_report(steady_miss=499), startup_report(stale_reply=499)):
            with self.subTest(source=source['measurements'][-1]):
                result = self.assess_fixture(source)
                self.assertFalse(result['operator_acceptance']['accepted_for_diagnostic_continuation'])
                self.assertEqual(result['candidate']['startup']['iteration_deadline_misses'], 1)

    def test_future_and_edited_logs_are_not_covered(self):
        raw = self.encode(startup_report())
        with self.assertRaisesRegex(ValueError, 'original R17'):
            compare_sources(raw, raw, acceptance_policy=OBSERVED_R17_POLICY)
        with patch('tools.compare_native_20ms.OBSERVED_R17_REPORTS',
                   frozenset((hashlib.sha256(raw).hexdigest(),))):
            with self.assertRaisesRegex(ValueError, 'original R17'):
                compare_sources(raw, raw + b'\n', acceptance_policy=OBSERVED_R17_POLICY)

    def test_errors_or_output_cannot_be_accepted(self):
        for change in ({'errors': ['timeout']}, {'learned_targets_sent': True},
                       {'motor_enable_sent': True}, {'imu_restore_status': 'failed'}):
            source = startup_report()
            source.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.assess_fixture(source)

    def test_unknown_policy_is_rejected(self):
        raw = self.encode(report())
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            compare_sources(raw, raw, acceptance_policy='unlimited-jitter')


if __name__ == '__main__':
    unittest.main()
