"""Synthetic file-only timing evidence; never hardware or physical certification."""
import copy
import json
import math
import unittest

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
from test_policy_local_profile import seal_local
import test_policy_rare_jitter_profile as rare_fixture
import test_policy_supported_extension_profile as extension_fixture


class HoldAfterSupportedProfileTests(unittest.TestCase):
    def setUp(self):
        extension_fixture.SupportedExtensionProfileTests.setUp(self)
        # Preserve the exact historical diagnostic, including its two misses.
        for index in (20, 90): rare_fixture.RareJitterProfileTests.miss(self, index)
        diagnostic_pin = _write(self.base/'pipeline_diagnostic.json', self.docs['pipeline_diagnostic'])
        self.data['artifacts']['pipeline_diagnostic'] = diagnostic_pin
        self.docs['prior_supported_profile']['artifacts']['pipeline_diagnostic'] = copy.deepcopy(diagnostic_pin)
        prior_path = extension_fixture.SupportedExtensionProfileTests.seal(self)
        profile.load_profile(prior_path)
        prior = json.loads(prior_path.read_text())
        prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'b'*64
        report = self.docs['prior_supported_report']
        report.update(cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
            execution_settings=profile.execution_settings(prior), current_position_hold_only=False,
            cyclic_inference_skipped=False, actual_model_calls=468,
            post_reply_deadline_policy=copy.deepcopy(prior['post_reply_deadline_policy']),
            model_provenance=dict(manifest_sha256=prior['artifacts']['scalar_step_manifest']['sha256'],
                baseline_provenance=dict(manifest_sha256=prior['artifacts']['model_manifest']['sha256'])))
        rows = []
        for index in range(494):
            begin = 1_000_000_000+index*20_000_000+(300_000 if index == 287 else 0)
            missed = index == 286
            end = begin+(20_189_507 if missed else 19_000_000)
            rows.append(dict(index=index, begin_ns=begin, output_reply_end_ns=begin+18_000_000,
                end_ns=end, phase='starting' if index == 0 else 'stopped' if index == 493 else 'active',
                effective_policy_weight=0. if index in (0,493) else .005,
                deadline20ms_missed=missed, steady_deadline20ms_missed=missed,
                startup_20ms_allowance_used=False,
                post_reply_deadline=dict(accepted=True, checked_ns=end, allowance_used=missed,
                    startup_allowance_used=False, consecutive_misses=int(missed),
                    rolling_misses=int(286 <= index < 386))))
        report.update(cycles=rows, deadline20ms_misses=1, steady_deadline20ms_misses=1,
            post_reply_deadline_allowance_uses=1, startup_20ms_allowance_uses=0)
        self.docs['prior_supported_profile'] = prior
        self.data.update(diagnostic_timing_acceptance=profile.CURRENT_HOLD_AFTER_SUPPORTED_10S,
            duration_s=3., policy_weight=0., startup_duration_s=1., assembly_id='synthetic-current-hold')
        for key in ('post_reply_deadline_policy', 'startup_cycle_allowance', 'startup_damping_duration_s'):
            self.data.pop(key, None)
        for axis in self.data['axes'].values():
            axis.update(kp=12., max_estimated_pd_torque_nm=.5, max_measured_velocity_rad_s=.25)
        self.docs['hardware_review']['assembly_id'] = self.data['assembly_id']
        self.acceptance = dict(mode=profile.CURRENT_HOLD_AFTER_SUPPORTED_10S, scope=self.data['scope'],
            strict_50hz_not_established=True, support_must_remain=True, load_bearing_not_established=True,
            strict_current_hold_deadline=True,
            review={**self.data['review'], 'decision':'ACCEPT_CURRENT_HOLD_AFTER_SUPPORTED_10S'})
        self.docs['hardware_review']['current_hold_after_supported_acceptance'] = self.acceptance

    def seal(self):
        for name in ('prior_supported_profile', 'prior_supported_report', 'prior_supported_observation'):
            if name == 'prior_supported_report':
                self.docs[name]['profile_sha256'] = self.data['artifacts']['prior_supported_profile']['sha256']
            if name == 'prior_supported_observation':
                self.docs[name]['report_sha256'] = self.data['artifacts']['prior_supported_report']['sha256']
            self.data['artifacts'][name] = _write(self.base/(name+'.json'), self.docs[name])
        seal_local(self.base, self.data, self.docs)
        for field, name in (('prior_profile_sha256','prior_supported_profile'),
                            ('prior_report_sha256','prior_supported_report'),
                            ('prior_observation_sha256','prior_supported_observation'),
                            ('diagnostic_sha256','pipeline_diagnostic')):
            self.acceptance[field] = self.data['artifacts'][name]['sha256']
        seal_local(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def load(self):
        return profile.load_profile(self.seal())

    def test_completed_bounded_ten_second_run_admits_only_strict_three_second_hold(self):
        loaded = self.load()
        self.assertTrue(profile.current_position_hold_only(loaded))
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertEqual(loaded['timing_review']['twenty_ms_misses'], 2)
        self.assertEqual(loaded['timing_review']['kind'], 'current_hold_after_supported_10s_admission_only')
        self.assertEqual((loaded['duration_s'], loaded['startup_duration_s'], loaded['hard_cycle_ms']), (3.,1.,20.))
        self.assertIsNone(profile.post_reply_deadline_settings(loaded))
        self.assertFalse(profile.reviewed_startup_cycle_allowance(loaded))
        with self.assertRaisesRegex(profile.ProfileError, 'loader proof'):
            profile.current_position_hold_only(self.data)

    def test_old_hold_keeps_original_one_per100_diagnostic_limit(self):
        self.data['diagnostic_timing_acceptance'] = profile.CURRENT_HOLD_PROBE
        with self.assertRaisesRegex(profile.ProfileError, 'one miss per100'):
            profile._timing(self.docs['pipeline_diagnostic'], self.data)

    def test_new_mode_keeps_rare_diagnostic_bound_and_ground_rejection(self):
        from singularitydog_hw.ground_trial_plan import GroundPlanError, validate_ground_plan
        from test_ground_trial_plan import fixture
        loaded = self.load()
        for stage in ('supported_stance', 'partial_load', 'stand', 'walk'):
            plan, _, records = fixture(stage)
            with self.subTest(stage=stage), self.assertRaisesRegex(GroundPlanError, 'Supported-only'):
                validate_ground_plan(plan, loaded, records)
        rare_fixture.RareJitterProfileTests.miss(self, 50)
        with self.assertRaisesRegex(profile.ProfileError, 'isolated-miss budget'):
            profile._timing(self.docs['pipeline_diagnostic'], self.data)

    def test_duration_gains_displacement_and_live_deadlines_cannot_expand(self):
        for key, value in (('duration_s',3.001), ('duration_s',30.), ('startup_duration_s',.999),
                           ('policy_weight',.001), ('hard_cycle_ms',21.), ('max_sample_age_ms',20.001),
                           ('max_sample_gap_ms',21.001), ('max_consecutive_20ms_misses',1)):
            old = self.data[key]; self.data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.load()
            self.data[key] = old
        for key, value in (('kp',12.001), ('kd',.151), ('max_estimated_pd_torque_nm',.501),
                           ('max_measured_velocity_rad_s',.251),
                           ('max_displacement_from_start_rad',math.radians(1.01))):
            old = self.data['axes']['1'][key]; self.data['axes']['1'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.load()
            self.data['axes']['1'][key] = old
        for key, value in (('startup_cycle_allowance',profile.FIRST_CYCLE_POST_REPLY),
                           ('startup_damping_duration_s',.08),
                           ('post_reply_deadline_policy',self.docs['prior_supported_profile']['post_reply_deadline_policy'])):
            self.data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.load()
            del self.data[key]

    def test_prior_runtime_session_identity_and_artifact_changes_rejected(self):
        baseline = copy.deepcopy(self.docs['prior_supported_profile'])
        changes = (lambda p:p.update(boot_id='changed'), lambda p:p.update(motor_power_epoch='changed'),
            lambda p:p.update(duration_s=9.), lambda p:p.update(request_gap_us=1000),
            lambda p:p['axes']['1'].update(offset_rad=.123), lambda p:p['axes']['1'].update(uid='wrong'),
            lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64}),
            lambda p:p['artifacts']['model_manifest'].update(sha256='f'*64),
            lambda p:p['artifacts']['calibration'].update(sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_profile'] = copy.deepcopy(baseline)
            change(self.docs['prior_supported_profile'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_predecessor_failures_partial_run_false_counts_and_late_reply_rejected(self):
        baseline = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r.update(status='ABORTED'), lambda r:r.update(errors=['failure']),
            lambda r:r.update(normal_ramp_completed=False), lambda r:r.update(learned_targets_sent=False),
            lambda r:r.update(actual_model_calls=0), lambda r:r.update(cyclic_inference_skipped=True),
            lambda r:r.update(deadline20ms_misses=0), lambda r:r.update(post_reply_deadline_allowance_uses=0),
            lambda r:r.update(cycles=r['cycles'][:474]),
            lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1]),
            lambda r:r['stop_reports']['rear']['fault_by_id'].update({'12':1}),
            lambda r:r['cycles'][286].update(output_reply_end_ns=r['cycles'][286]['begin_ns']+20_000_001),
            lambda r:r['cycles'][286].update(end_ns=r['cycles'][286]['begin_ns']+21_000_001),
            lambda r:r['cycles'][-1].update(phase='active'))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(baseline)
            change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_second_predecessor_miss_within100_cycles_rejected(self):
        report = self.docs['prior_supported_report']
        row = report['cycles'][290]
        row.update(end_ns=row['begin_ns']+20_100_000, deadline20ms_missed=True, steady_deadline20ms_missed=True)
        row['post_reply_deadline'].update(checked_ns=row['end_ns'], allowance_used=True, consecutive_misses=1, rolling_misses=2)
        with self.assertRaisesRegex(profile.ProfileError, 'bounded live timing'): self.load()

    def test_matching_observation_hashes_and_review_are_required(self):
        baseline = copy.deepcopy(self.docs['prior_supported_observation'])
        for key, value in (('audio_heard',False), ('abnormal_noise_vibration_slip_sinking_contact',True),
                           ('box_support_maintained',False), ('autonomous_standing_or_walking_observed',True),
                           ('observed_by','inferred'), ('user_statement','')):
            self.docs['prior_supported_observation'] = copy.deepcopy(baseline)
            self.docs['prior_supported_observation'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.load()
        self.docs['prior_supported_observation'] = baseline
        for key in ('support_must_remain','load_bearing_not_established','strict_current_hold_deadline',
                    'strict_50hz_not_established'):
            self.acceptance[key] = False
            with self.subTest(key=key), self.assertRaisesRegex(profile.ProfileError, 'hash-bound'): self.load()
            self.acceptance[key] = True
        path = self.seal()
        for key in ('prior_profile_sha256', 'prior_report_sha256', 'prior_observation_sha256', 'diagnostic_sha256'):
            old = self.acceptance[key]; self.acceptance[key] = 'f'*64
            seal_local(self.base, self.data, self.docs)
            with self.subTest(key=key), self.assertRaisesRegex(profile.ProfileError, 'hash-bound'):
                profile.load_profile(path)
            self.acceptance[key] = old
        path = self.seal()
        target = self.base/'prior_supported_report.json'
        target.write_text(target.read_text()+' ')
        with self.assertRaisesRegex(profile.ProfileError, 'SHA256'): profile.load_profile(path)


if __name__ == '__main__': unittest.main()
