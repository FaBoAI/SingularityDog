"""Finite diagnostic admission contracts; synthetic data is never robot evidence."""
import copy
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from singularitydog_hw.policy_post_reply_timing import POST_REPLY_POLICY, PostReplyDeadlineBudget
from test_policy_local_profile import local_fixture, seal_local
from test_policy_live_profile import _write


class RareJitterProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.data, self.docs, pins = local_fixture(self.base)
        self.enterContext(patch.object(profile.shadow, 'SOURCE_HASHES', pins))
        self.data.update(model_backend=profile.SCALAR_BACKEND, voltage_overlap=True,
            diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            startup_damping_duration_s=.08,
            post_reply_deadline_policy=dict(mode=POST_REPLY_POLICY, max_lateness_ms=1.,
                max_consecutive_misses=1, rolling_window_cycles=100, max_misses_per_window=1))
        for row in self.data['axes'].values(): row['max_measured_velocity_rad_s'] = .35
        scalar = dict(schema='native-step-scalar-file-only-v1', status='PASS_FILE_ONLY_COMPARE',
            baseline_manifest_sha256=self.data['artifacts']['model_manifest']['sha256'],
            hardware_opened=False, output_allowed=False, approved_for_runtime=False, live_50hz_verified=False)
        self.docs['scalar_step_manifest'] = scalar
        self.data['artifacts']['scalar_step_manifest'] = _write(self.base/'scalar_step_manifest.json', scalar)
        report = self.docs['pipeline_diagnostic']
        report.update(boot_id=self.data['boot_id'], approved_for_runtime=False, imu_restore_status='restored',
            cycles_completed=501, cycles_requested=501, model_source=dict(
                manifest_sha256=self.data['artifacts']['scalar_step_manifest']['sha256'],
                baseline_provenance={'manifest_sha256':self.data['artifacts']['model_manifest']['sha256']}))
        report['plan'].update(startup_cycle_allowance=1, steady_cycles_requested=500,
            absolute_epoch_cadence=True, v3_voltage_overlap=True, v3_voltage_validation_overlap=True)
        report['observer'].update(ticks_completed=501, ticks_requested=501)
        seed = report['measurements'][0]
        epoch = seed['release_ns']
        rows = []
        for index in range(501):
            offset = index*20_000_000+(600_000 if index else 0)
            row = {key:value+offset if key.endswith('_ns') else value for key,value in seed.items()}
            row.update(oldest_input_start_ns=row['release_ns']+2_000_000,
                timing_phase='steady' if index else 'startup', cadence_slot=index,
                scheduled_release_ns=epoch+index*20_000_000, skipped_slots_before=0)
            if index == 0: row['cycle_end_ns'] = row['release_ns']+20_500_000
            rows.append(row)
        report['measurements'] = rows
        report['absolute_epoch_schedule'] = dict(enabled=True, epoch_ns=epoch, period_ns=20_000_000)
        self.docs['hardware_review']['post_reply_deadline_acceptance'] = dict(
            settings=copy.deepcopy(self.data['post_reply_deadline_policy']), scope=self.data['scope'],
            strict_50hz_not_established=True, hard_output_and_freshness_limits_unchanged=True,
            review={**self.data['review'], 'decision':'ACCEPT_BOUNDED_POST_REPLY_DEADLINE'})
        self.acceptance = dict(mode=profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            scope=self.data['scope'], strict_50hz_not_established=True, live_deadline_policy_unchanged=True,
            review={**self.data['review'],
                    'decision':'ACCEPT_RARE_JITTER_DIAGNOSTIC_FOR_2S_SUPPORTED_PROBE'})
        self.docs['hardware_review']['rare_jitter_diagnostic_acceptance'] = self.acceptance

    def seal(self):
        seal_local(self.base, self.data, self.docs)
        self.acceptance['diagnostic_sha256'] = self.data['artifacts']['pipeline_diagnostic']['sha256']
        seal_local(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def miss(self, index, elapsed_ns=20_100_000):
        rows = self.docs['pipeline_diagnostic']['measurements']
        rows[index]['cycle_end_ns'] = rows[index]['release_ns']+elapsed_ns
        if index+1 < len(rows):
            shift = max(0, rows[index]['cycle_end_ns']+1-rows[index+1]['release_ns'])
            for key in rows[index+1]:
                if key.endswith('_ns') and key != 'scheduled_release_ns': rows[index+1][key] += shift

    def timing(self):
        return profile._timing(self.docs['pipeline_diagnostic'], self.data)

    def test_five_isolated_misses_and_two_per100_are_explicit_admission_only(self):
        for index in (20, 90, 220, 290, 450): self.miss(index)
        loaded = profile.load_profile(self.seal())
        self.assertEqual(loaded['timing_review']['twenty_ms_misses'], 5)
        self.assertEqual(loaded['timing_review']['kind'], 'supported_policy_2s_rare_jitter_admission_only')
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertEqual(profile.post_reply_deadline_settings(loaded)['max_misses_per_window'], 1)
        self.assertEqual(loaded['hard_cycle_ms'], 20.)
        self.assertEqual(loaded['max_sample_age_ms'], 20.)

    def test_six_total_three_per100_and_consecutive_misses_rejected(self):
        baseline = copy.deepcopy(self.docs['pipeline_diagnostic'])
        for indices in ((20,90,180,250,360,450), (20,50,90), (20,21)):
            self.docs['pipeline_diagnostic'] = copy.deepcopy(baseline)
            for index in indices: self.miss(index)
            with self.subTest(indices=indices), self.assertRaisesRegex(profile.ProfileError, 'isolated-miss budget'):
                self.timing()

    def test_scheduled_carryover_counts_and_old_modes_keep_one_per100(self):
        rows = self.docs['pipeline_diagnostic']['measurements']
        rows[1]['cycle_end_ns'] = rows[1]['scheduled_release_ns']+20_100_000
        self.miss(90)
        self.assertEqual(self.timing()['twenty_ms_misses'], 2)
        for mode in (profile.SUPPORTED_POLICY_PROBE, profile.SUPPORTED_POLICY_PROBE_5S):
            self.data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaisesRegex(profile.ProfileError, 'one miss per100'):
                self.timing()

    def test_hard_reply_checked_age_and_whole_boundaries(self):
        report = self.docs['pipeline_diagnostic']; original = copy.deepcopy(report['measurements'][-1])
        row = report['measurements'][-1]; release = row['release_ns']
        row.update(oldest_input_start_ns=release+1_000_000, last_proxy_reply_ns=release+20_000_000,
                   cycle_end_ns=release+21_000_000)
        self.assertEqual(self.timing()['twenty_ms_misses'], 1)
        for key, value in (('last_proxy_reply_ns',release+20_000_001),
                           ('oldest_input_start_ns',release+999_999),
                           ('cycle_end_ns',release+21_000_001)):
            saved = row[key]; row[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.timing()
            row[key] = saved
        row.update(original)
        # Startup has separate whole-cycle allowance, but never a stale input.
        startup = report['measurements'][0]
        startup['oldest_input_start_ns'] = startup['release_ns']+499_999
        with self.assertRaisesRegex(profile.ProfileError, 'checked sample-age'): self.timing()

    def test_exact_count_boot_and_schedule_required(self):
        baseline = copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes = (lambda r:r.update(cycles_requested=500), lambda r:r.update(boot_id='different'),
            lambda r:r['measurements'][2].update(skipped_slots_before=1),
            lambda r:r['measurements'][2].update(cadence_slot=3))
        for change in changes:
            self.docs['pipeline_diagnostic'] = copy.deepcopy(baseline)
            change(self.docs['pipeline_diagnostic'])
            with self.assertRaises(profile.ProfileError): self.timing()

    def test_named_review_pins_mode_report_and_unchanged_live_policy(self):
        path = self.seal()
        for key, value in (('mode',profile.SUPPORTED_POLICY_PROBE_5S), ('diagnostic_sha256','f'*64),
                           ('scope','ground'), ('strict_50hz_not_established',False),
                           ('live_deadline_policy_unchanged',False)):
            old = self.acceptance[key]; self.acceptance[key] = value
            seal_local(self.base,self.data,self.docs)
            with self.subTest(key=key), self.assertRaisesRegex(profile.ProfileError, 'rare-jitter diagnostic acceptance'):
                profile.load_profile(path)
            self.acceptance[key] = old
        self.acceptance['review']['decision'] = 'APPROVED_SUPPORTED_CHARACTERIZATION'
        with self.assertRaisesRegex(profile.ProfileError, 'Review has not approved'):
            profile.load_profile(self.seal())

    def test_scope_caps_and_live_budget_cannot_expand(self):
        for key,value in (('duration_s',2.001), ('policy_weight',.005001), ('startup_damping_duration_s',.079),
                           ('hard_cycle_ms',21), ('max_sample_age_ms',20.001)):
            old = self.data[key]; self.data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): profile.load_profile(self.seal())
            self.data[key] = old
        for key,value in (('kp',3.001), ('kd',.150001), ('max_measured_velocity_rad_s',.350001),
                           ('max_displacement_from_start_rad',math.radians(1)+1e-6)):
            old = self.data['axes']['1'][key]; self.data['axes']['1'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): profile.load_profile(self.seal())
            self.data['axes']['1'][key] = old
        self.data['post_reply_deadline_policy']['max_misses_per_window'] = 2
        with self.assertRaisesRegex(profile.ProfileError, 'max_misses_per_window'):
            profile.load_profile(self.seal())

    def test_live_second_miss_still_stops(self):
        budget = PostReplyDeadlineBudget(self.data['post_reply_deadline_policy'])
        for index in range(90):
            start = 1_000_000_000+index*25_000_000
            args = dict(index=index, begin_ns=start, oldest_input_ns=start+1_000_000,
                final_write_ns=start+16_000_000, last_reply_ns=start+19_000_000,
                output_sample_start_ns=start+14_000_000, checked_ns=start+20_100_000,
                sample_age_ns=20_000_000)
            if index not in (1,89): args['checked_ns'] = start+19_900_000
            if index == 89:
                with self.assertRaisesRegex(RuntimeError, 'rolling miss budget'): budget.admit(**args)
            else: budget.admit(**args)


if __name__ == '__main__': unittest.main()
