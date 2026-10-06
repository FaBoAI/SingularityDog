"""Synthetic v2 budget/admission contracts; no operator facts or hardware proof."""
import copy
import json
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw.policy_post_reply_timing import (
    POST_REPLY_POLICY, POST_REPLY_POLICY_V2, PostReplyDeadlineBudget)
import test_prepared_voltage_publication_profile as prepared
import test_policy_post_reply_timing as v1


def settings():
    return {**v1.settings(), 'mode': POST_REPLY_POLICY_V2,
            'post_reply_input_age_budget_ms': 1.}


def args(index=0, elapsed=20_100_000, **changes):
    begin = 1_000_000_000+index*25_000_000
    value = dict(index=index, begin_ns=begin, oldest_input_ns=begin+200_000,
        final_write_ns=begin+17_000_000, last_reply_ns=begin+19_800_000,
        output_sample_start_ns=begin+12_000_000, checked_ns=begin+elapsed,
        sample_age_ns=20_000_000)
    value.update(changes)
    return value


class InputAgeBudgetTests(unittest.TestCase):
    def test_v1_rejects_same_input_age_v2_accepts_with_explicit_proof(self):
        a = args(elapsed=20_500_000)
        with self.assertRaisesRegex(RuntimeError, 'sample-age'):
            PostReplyDeadlineBudget(v1.settings()).admit(**a)
        row = PostReplyDeadlineBudget(settings()).admit(**a)
        self.assertEqual(row['input_sample_age_ms'], 20.3)
        self.assertTrue(row['input_age_allowance_used'])
        self.assertEqual(row['mode'], POST_REPLY_POLICY_V2)
        self.assertTrue(row['allowance_used'])

    def test_original_r11_failed_timestamps_are_only_counterfactual_input(self):
        a = dict(index=0, begin_ns=7933104201816, oldest_input_ns=7933104486113,
            final_write_ns=7933121274241, last_reply_ns=7933124052731,
            output_sample_start_ns=7933116559016, checked_ns=7933124615022,
            sample_age_ns=20_000_000, startup_allowed=True)
        row = PostReplyDeadlineBudget(settings()).admit(**a)
        self.assertEqual(row['input_sample_age_ms'], 20.128909)
        self.assertTrue(row['startup_allowance_used'])
        self.assertTrue(row['input_age_allowance_used'])
        self.assertFalse(row['allowance_used'])
        # This is arithmetic on old timestamps, never a changed original report.

    def test_exact_21ms_age_boundary_and_one_nanosecond_over(self):
        a = args(elapsed=21_000_000, oldest_input_ns=1_000_000_000)
        self.assertTrue(PostReplyDeadlineBudget(settings()).admit(**a)['accepted'])
        with self.assertRaisesRegex(RuntimeError, 'sample-age'):
            PostReplyDeadlineBudget(settings()).admit(**{**a, 'checked_ns':a['checked_ns']+1})

    def test_output_feedback_age_remains_20ms(self):
        a = args(elapsed=20_500_000, output_sample_start_ns=1_000_200_000)
        with self.assertRaisesRegex(RuntimeError, 'sample-age'):
            PostReplyDeadlineBudget(settings()).admit(**a)

    def test_native_write_and_reply_deadlines_remain_20ms(self):
        for values in (dict(last_reply_ns=1_020_000_001),
                       dict(final_write_ns=1_020_000_001, last_reply_ns=1_020_000_002)):
            with self.subTest(values=values), self.assertRaisesRegex(RuntimeError, 'write/reply'):
                PostReplyDeadlineBudget(settings()).admit(**args(elapsed=20_500_000, **values))

    def test_coordinator_lateness_independently_remains_one_ms(self):
        with self.assertRaisesRegex(RuntimeError, 'lateness'):
            PostReplyDeadlineBudget(settings()).admit(**args(
                elapsed=21_000_001, oldest_input_ns=1_002_000_000))

    def test_settings_reject_bool_nan_infinity_extra_and_widened_budgets(self):
        for key, values in (('max_lateness_ms', (True, float('nan'), float('inf'), 0, 1.001)),
                            ('post_reply_input_age_budget_ms', (True, float('nan'), float('inf'), 0, 1.001)),
                            ('max_consecutive_misses', (True, 2)),
                            ('rolling_window_cycles', (True, 99)),
                            ('max_misses_per_window', (True, 2))):
            for value in values:
                s = settings(); s[key] = value
                with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                    PostReplyDeadlineBudget(s)
        s = settings(); s['unreviewed'] = True
        with self.assertRaises(RuntimeError): PostReplyDeadlineBudget(s)

    def test_noninteger_boolean_and_nonfinite_timestamps_are_rejected(self):
        for key in args():
            for value in (True, 1., float('nan'), float('inf')):
                with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                    PostReplyDeadlineBudget(settings()).admit(**args(**{key:value}))
        with self.assertRaises(RuntimeError):
            PostReplyDeadlineBudget(settings()).admit(**args(sample_age_ns=20_000_001))

    def test_settings_tampering_and_downgrade_cannot_bypass_bound(self):
        for key, value in (('mode', POST_REPLY_POLICY), ('post_reply_input_age_budget_ms', .5),
                           ('max_lateness_ms', 2), ('max_misses_per_window', 2)):
            b = PostReplyDeadlineBudget(settings()); b.settings[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError): b.admit(**args())

    def test_steady_consecutive_and_one_in_100_quotas_include_age_use(self):
        b = PostReplyDeadlineBudget(settings()); b.admit(**args(elapsed=20_500_000))
        with self.assertRaisesRegex(RuntimeError, 'consecutive'): b.admit(**args(1, 20_500_000))
        b.admit(**args(1, 20_000_000))
        with self.assertRaisesRegex(RuntimeError, 'rolling'): b.admit(**args(2, 20_500_000))
        for index in range(2, 100): b.admit(**args(index, 20_000_000))
        self.assertTrue(b.admit(**args(100, 20_500_000))['allowance_used'])

    def test_first_cycle_review_is_separate_but_cannot_repeat(self):
        b = PostReplyDeadlineBudget(settings())
        self.assertTrue(b.admit(**args(elapsed=20_500_000), startup_allowed=True)['startup_allowance_used'])
        self.assertEqual(b.accepted_misses, 0)
        with self.assertRaisesRegex(RuntimeError, 'startup'): b.admit(**args(1), startup_allowed=True)
        self.assertTrue(b.admit(**args(1, 20_500_000))['allowance_used'])
        with self.assertRaisesRegex(RuntimeError, 'consecutive'): b.admit(**args(2, 20_500_000))

    def test_rejected_cycle_does_not_advance_sequence_or_consume_quota(self):
        b = PostReplyDeadlineBudget(settings())
        with self.assertRaises(RuntimeError): b.admit(**args(elapsed=22_000_000))
        self.assertEqual((b.previous_index, b.accepted_misses, b.consecutive), (-1, 0, 0))
        self.assertFalse(b.admit(**args(elapsed=20_000_000))['allowance_used'])


class InputAgeProfileTests(unittest.TestCase):
    select = prepared.PreparedVoltageProfileTests.select

    def setUp(self):
        prepared.PreparedVoltageProfileTests.setUp(self)
        self.data['post_reply_deadline_policy'] = settings()
        h = self.docs['hardware_review']
        for name in ('rare_jitter_diagnostic_acceptance', 'voltage_pipeline_acceptance',
                     'prepared_voltage_publication_acceptance', 'post_reply_deadline_acceptance'):
            row = h[name]
            row.pop('hard_output_and_freshness_limits_unchanged', None)
            row.pop('live_deadline_policy_unchanged', None)
            row.update(pre_send_input_and_native_output_limits_unchanged=True,
                output_feedback_sample_age_limit_unchanged=True, post_reply_input_age_budget_ms=1.)
        h['post_reply_deadline_acceptance'].update(
            schema='singularitydog.post-reply-input-age-review.v2', settings=settings())
        h['post_reply_deadline_acceptance']['review']['decision'] = 'ACCEPT_BOUNDED_POST_REPLY_INPUT_AGE_V2'

    def seal(self):
        path = prepared.PreparedVoltageProfileTests.seal(self)
        self.docs['hardware_review']['post_reply_deadline_acceptance']['diagnostic_sha256'] = self.data['artifacts']['pipeline_diagnostic']['sha256']
        return prepared.PreparedVoltageProfileTests.seal(self)

    def load(self):
        return live.load_profile(self.seal())

    def test_full_loader_explicit_selection_bound_and_no_active_timing_claim(self):
        p = self.load()
        self.assertEqual(live.post_reply_deadline_settings(p), settings())
        self.assertTrue(live.prepared_voltage_publication_settings(p))
        self.assertFalse(p['actual_policy_output_20ms_verified'])
        self.assertEqual((p['hard_cycle_ms'], p['max_sample_age_ms'], p['max_sample_gap_ms']), (20, 20, 21))

    def test_missing_wrong_or_stale_named_review_rejected(self):
        row = self.docs['hardware_review']['post_reply_deadline_acceptance']
        for key, value in (('schema', 'wrong'), ('diagnostic_sha256', 'a'*64),
                           ('post_reply_input_age_budget_ms', .5)):
            old = row.get(key); row[key] = value
            # seal() refreshes only diagnostic SHA; test that field afterwards.
            path = self.seal()
            if key == 'diagnostic_sha256':
                doc = json.loads((self.base/'hardware_review.json').read_bytes())
                doc['post_reply_deadline_acceptance'][key] = value
                self.docs['hardware_review'] = doc
                from test_policy_local_profile import seal_local
                seal_local(self.base, self.data, self.docs)
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live.load_profile(path)
            row[key] = old; self.docs['hardware_review']['post_reply_deadline_acceptance'] = row
        row['review']['decision'] = 'ACCEPT_BOUNDED_POST_REPLY_DEADLINE'
        with self.assertRaises(live.ProfileError): self.load()

    def test_unchanged_freshness_claim_cannot_be_copied_into_any_v2_review(self):
        for name in ('rare_jitter_diagnostic_acceptance', 'voltage_pipeline_acceptance',
                     'prepared_voltage_publication_acceptance', 'post_reply_deadline_acceptance'):
            row = self.docs['hardware_review'][name]
            row['hard_output_and_freshness_limits_unchanged'] = True
            with self.subTest(name=name), self.assertRaises(live.ProfileError): self.load()
            del row['hard_output_and_freshness_limits_unchanged']

    def test_pre_send_and_output_feedback_unchanged_flags_are_required(self):
        row = self.docs['hardware_review']['post_reply_deadline_acceptance']
        for key in ('pre_send_input_and_native_output_limits_unchanged', 'output_feedback_sample_age_limit_unchanged'):
            for value in (False, 1, None):
                row[key] = value
                with self.subTest(key=key, value=value), self.assertRaises(live.ProfileError): self.load()
            row[key] = True

    def test_v2_does_not_admit_10s_or_hold_ground_human_gain_routes(self):
        for mode in (live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S, live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                     live.CURRENT_HOLD_AFTER_SUPPORTED_10S, live.SUPPORTED_POLICY_GAIN_STEP_3S,
                     live.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S, live.FIXED_CATCH_CURRENT_HOLD_30S):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(live.ProfileError): live._post_reply_policy(data)

    def test_caps_and_nonfinite_settings_cannot_change(self):
        for key, value in (('duration_s', 2.001), ('policy_weight', .00501), ('max_sample_age_ms', 20.001),
                           ('max_sample_gap_ms', 21.001), ('hard_cycle_ms', 20.001)):
            data = copy.deepcopy(self.data); data[key] = value
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live._post_reply_policy(data)
        for key, value in (('kp', 3.001), ('kd', .15001), ('max_displacement_from_start_rad', math.radians(1.001))):
            data = copy.deepcopy(self.data); data['axes']['1'][key] = value
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live._post_reply_policy(data)
        for value in (True, float('nan'), float('inf'), 1.001):
            data = copy.deepcopy(self.data); data['post_reply_deadline_policy']['post_reply_input_age_budget_ms'] = value
            with self.subTest(value=value), self.assertRaises(live.ProfileError): live._post_reply_policy(data)

    def test_raw_unapproved_and_mutated_loaded_profiles_cannot_reuse_proof(self):
        with self.assertRaises(live.ProfileError): live.post_reply_deadline_settings(self.data)
        p = self.load()
        for target, key, value in ((p['post_reply_deadline_policy'], 'mode', POST_REPLY_POLICY),
                                   (p, 'output_allowed', False), (p, 'motor_power_epoch', 'other'),
                                   (p['axes']['1'], 'kp', 2.5), (p['artifacts']['calibration'], 'sha256', 'a'*64)):
            old = target[key]; target[key] = value
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live.post_reply_deadline_settings(p)
            target[key] = old
        draft = copy.deepcopy(self.data)
        draft.update(approved_for_supported_policy_output=False, blockers=['SYNTHETIC PENDING'], review=None)
        path = self.base/'unapproved.json'; path.write_text(json.dumps(draft))
        parsed = live.load_profile(path, require_approved=False)
        self.assertFalse(parsed['output_allowed'])
        with self.assertRaises(live.ProfileError): live.post_reply_deadline_settings(parsed)

    def test_startup_keeps_separate_named_review_and_precise_v2_scope(self):
        self.data['startup_cycle_allowance'] = live.FIRST_CYCLE_POST_REPLY
        row = dict(mode=live.FIRST_CYCLE_POST_REPLY, scope=self.data['scope'], first_cycle_only=True,
            steady_miss_budget_unchanged=True, pre_send_input_and_native_output_limits_unchanged=True,
            output_feedback_sample_age_limit_unchanged=True, post_reply_input_age_budget_ms=1.,
            review={**self.data['review'], 'decision':'ACCEPT_FIRST_CYCLE_POST_REPLY'})
        self.docs['hardware_review']['startup_cycle_acceptance'] = row
        self.assertTrue(live.reviewed_startup_cycle_allowance(self.load()))
        row['hard_output_and_freshness_limits_unchanged'] = True
        with self.assertRaises(live.ProfileError): self.load()


class InputAgeCoordinatorTests(unittest.TestCase):
    # Existing in-memory owner fixture explicitly bypasses profile admission;
    # full reviewed admission is independently covered above.
    run_case = v1.CoordinatorTests.run_case
    run_timing = v1.CoordinatorTests.run_timing

    def execute(self, **kwargs):
        with patch.object(runtime, 'post_reply_deadline_settings', return_value=settings()):
            return self.run_timing(**kwargs)

    def test_one_post_reply_age_miss_is_counted_without_50hz_or_guard_claim(self):
        report, _, _ = self.execute(delayed={2:20_950_000})
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        row = report['cycles'][2]['post_reply_deadline']
        self.assertTrue(row['input_age_allowance_used'])
        self.assertEqual(report['post_reply_deadline_allowance_uses'], 1)
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertTrue(report['stop_confirmed'])

    def test_repeated_post_reply_age_misses_still_stop(self):
        for delayed in ({2:20_950_000, 3:20_950_000}, {2:20_950_000, 4:20_950_000}):
            report, _, _ = self.execute(delayed=delayed)
            self.assertEqual(report['status'], 'ABORTED')
            self.assertTrue(report['stop_confirmed'])

    def test_late_missing_reply_and_owner_failure_preserve_abort_stop(self):
        for values in ({'late_reply':True}, {'missing_reply':True}, {'front_options':{'fail_motion':True}}):
            report, _, _ = self.execute(**values)
            self.assertEqual(report['status'], 'ABORTED')
            self.assertTrue(report['stop_confirmed'])

    def test_pre_send_encoding_and_stale_hold_are_not_waived(self):
        for values in ({'late_encoding':True}, {'wake_late':True}):
            report, _, _ = self.execute(**values)
            self.assertEqual(report['status'], 'ABORTED')
            self.assertTrue(report['stop_confirmed'])


if __name__ == '__main__':
    unittest.main()
