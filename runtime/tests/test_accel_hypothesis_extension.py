"""File-only admission tests. Synthetic receipts are never robot evidence."""
import copy
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
import test_policy_supported_extension_profile as extension_fixture


class AccelHypothesisExtensionTests(unittest.TestCase):
    def setUp(self):
        extension_fixture.SupportedExtensionProfileTests.setUp(self)
        self.data['accel_input_hypothesis'] = True
        self.data['cadence_source_sha256'] = profile.cadence_source_hashes(self.data)
        self.docs['accel_input_hypothesis'] = {'SYNTHETIC_UNIT_TEST_ONLY': True}
        self.reference = _write(self.base/'accel_input_hypothesis.json',
                                self.docs['accel_input_hypothesis'])
        self.reference['path'] = str(self.base/'accel_input_hypothesis.json')
        self.data['artifacts']['accel_input_hypothesis'] = copy.deepcopy(self.reference)
        self.provenance = {
            'kind': 'singularitydog.supported-accel-input-hypothesis.v1',
            'scope': 'boxed_small_mix_only', 'hypothesis_sha256': self.reference['sha256'],
            'candidate_sha256': '3'*64, 'manifest_sha256': '4'*64,
            'raw_norm_bounds_m_s2': [self.data['imu_accel_norm_min_m_s2'],
                                    self.data['imu_accel_norm_max_m_s2']],
            'formal_calibration_approved': False, 'absolute_orientation_error_bound_rad': None,
            'fit_and_independent_captures_reaudited': True, 'grants_motor_output': False}

        class SyntheticAuditedInput:
            def provenance(inner): return copy.deepcopy(self.provenance)
        self.enterContext(patch.object(profile, '_load_accel_input_hypothesis',
                                      return_value=SyntheticAuditedInput()))
        prior = self.docs['prior_supported_profile']
        prior['accel_input_hypothesis'] = True
        prior['artifacts']['accel_input_hypothesis'] = copy.deepcopy(self.reference)
        prior['cadence_source_sha256'] = copy.deepcopy(self.data['cadence_source_sha256'])
        prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'a'*64
        actual = self.docs['prior_supported_report']
        actual.update(cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
                      current_position_hold_only=False, cyclic_inference_skipped=False,
                      actual_model_calls=68)
        for row in actual['cycles']:
            row['imu_body'] = dict(accel_bias_subtracted=True, accel_scale_corrected=True,
                                  accel_input_hypothesis=copy.deepcopy(self.provenance))
        diagnostic = self.docs['pipeline_diagnostic']
        diagnostic.update(motor_power_epoch=self.data['motor_power_epoch'],
                          cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']))
        diagnostic['plan']['accel_input_hypothesis'] = copy.deepcopy(self.reference)
        diagnostic['input_sha256']['accel_input_hypothesis'] = self.reference['sha256']
        diagnostic['observer']['accel_input_hypothesis'] = copy.deepcopy(self.provenance)
        # Re-measurement follows the successful prior run. Keep every relative
        # raw timing interval intact while assigning synthetic causal epochs.
        shift = actual['cycles'][-1]['end_ns'] + 1_000_000 - diagnostic['measurements'][0]['release_ns']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += shift
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift

    seal = extension_fixture.SupportedExtensionProfileTests.seal
    load = extension_fixture.SupportedExtensionProfileTests.load

    def test_fresh_diagnostic_and_completed_same_hypothesis_two_seconds_admit_only_boxed_ten_seconds(self):
        loaded = self.load()
        prior = self.docs['prior_supported_profile']
        self.assertNotEqual(prior['artifacts']['pipeline_diagnostic']['sha256'],
                            loaded['artifacts']['pipeline_diagnostic']['sha256'])
        self.assertEqual(loaded['duration_s'], 10.)
        self.assertEqual(profile.accel_input_hypothesis_settings(loaded), self.reference)
        self.assertTrue(loaded['output_allowed'])
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['formal_calibration_approved'])
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['grants_motor_output'])
        self.assertIsNone(loaded['_accel_input_hypothesis_provenance']['absolute_orientation_error_bound_rad'])
        self.assertEqual((loaded['hard_cycle_ms'], loaded['max_sample_age_ms'],
                          loaded['max_sample_gap_ms']), (20., 20., 21.))
        self.assertEqual(loaded['policy_weight'], .005)
        self.assertTrue(all(a['kp'] == 3. and a['kd'] == .15 and
                            a['max_displacement_from_start_rad'] == math.radians(1)
                            for a in loaded['axes'].values()))

    def test_loading_does_not_rewrite_prior_profile_report_or_observation(self):
        path = self.seal()
        files = [self.base/(name+'.json') for name in (
            'prior_supported_profile', 'prior_supported_report', 'prior_supported_observation')]
        before = [p.read_bytes() for p in files]
        profile.load_profile(path)
        self.assertEqual(before, [p.read_bytes() for p in files])

    def test_missing_failed_short_or_noninferred_prior_run_is_rejected(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r.update(status='ABORTED'), lambda r:r.update(learned_targets_sent=False),
                   lambda r:r.update(normal_ramp_completed=False), lambda r:r.update(cycles=r['cycles'][:79]),
                   lambda r:r.update(actual_model_calls=0), lambda r:r.update(actual_model_calls=True),
                   lambda r:r.update(current_position_hold_only=True),
                   lambda r:r.update(cyclic_inference_skipped=True),
                   lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1]))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_prior_raw_input_or_different_hypothesis_cannot_be_relabelled(self):
        prior = self.docs['prior_supported_profile']; saved = copy.deepcopy(prior)
        changes = (lambda p:p.update(accel_input_hypothesis=False),
                   lambda p:p['artifacts']['accel_input_hypothesis'].update(sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_profile'] = copy.deepcopy(saved); change(self.docs['prior_supported_profile'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_actual_prior_input_provenance_is_checked_on_every_cycle(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r['cycles'][30]['imu_body'].pop('accel_input_hypothesis'),
                   lambda r:r['cycles'][30]['imu_body']['accel_input_hypothesis'].update(hypothesis_sha256='f'*64),
                   lambda r:r['cycles'][-1]['imu_body'].update(accel_bias_subtracted=False),
                   lambda r:r['cycles'][-1]['imu_body'].update(reviewed_accel_calibration={}),
                   lambda r:r['cycles'][0]['imu_body']['accel_input_hypothesis'].update(formal_calibration_approved=True))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaisesRegex(profile.ProfileError, 'actual input provenance'):
                self.load()

    def test_new_diagnostic_must_bind_exact_current_sources_power_and_hypothesis(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes = (lambda r:r.update(motor_power_epoch='old-power'),
                   lambda r:r.update(cadence_source_sha256=copy.deepcopy(self.docs['prior_supported_profile']['cadence_source_sha256'])),
                   lambda r:r['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64}),
                   lambda r:r['plan']['accel_input_hypothesis'].update(sha256='f'*64),
                   lambda r:r['input_sha256'].update(accel_input_hypothesis='f'*64),
                   lambda r:r['observer']['accel_input_hypothesis'].update(candidate_sha256='f'*64))
        for change in changes:
            self.docs['pipeline_diagnostic'] = copy.deepcopy(saved); change(self.docs['pipeline_diagnostic'])
            with self.subTest(change=change), self.assertRaisesRegex(profile.ProfileError, 'hypothesis'):
                self.load()

    def test_changed_diagnostic_must_be_acquired_after_the_prior_run(self):
        diagnostic = self.docs['pipeline_diagnostic']
        first = diagnostic['measurements'][0]['release_ns']
        target = self.docs['prior_supported_report']['cycles'][-1]['end_ns']
        shift = target-first
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += shift
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift
        with self.assertRaisesRegex(profile.ProfileError, 'must follow'): self.load()

    def test_other_inputs_model_calibration_and_runtime_cannot_change(self):
        saved = copy.deepcopy(self.docs['prior_supported_profile'])
        changes = [lambda p:p.update(boot_id='changed'), lambda p:p.update(motor_power_epoch='changed'),
                   lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64})]
        for key in ('calibration', 'mount', 'bias', 'model_manifest', 'scalar_step_manifest', 'command_loss_report'):
            changes.append(lambda p, key=key:p['artifacts'][key].update(sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_profile'] = copy.deepcopy(saved); change(self.docs['prior_supported_profile'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_no_higher_gains_motion_velocity_or_timing_limits(self):
        saved = copy.deepcopy(self.data)
        changes = [lambda p:p.update(policy_weight=.005001), lambda p:p.update(hard_cycle_ms=20.001),
                   lambda p:p.update(max_sample_age_ms=20.001), lambda p:p.update(max_sample_gap_ms=21.001)]
        for key, value in (('kp',3.001), ('kd',.150001), ('max_measured_velocity_rad_s',.350001),
                           ('max_displacement_from_start_rad',math.radians(1.001)),
                           ('max_estimated_pd_torque_nm',.10001)):
            changes.append(lambda p, key=key, value=value:p['axes']['8'].update({key:value}))
        for change in changes:
            self.data = copy.deepcopy(saved); change(self.data)
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_human_hold_gain_twenty_seconds_and_generic_duration_remain_rejected(self):
        for mode in (profile.CURRENT_HOLD_PROBE, profile.CURRENT_HOLD_AFTER_SUPPORTED_10S,
                     profile.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S, profile.FIXED_CATCH_CURRENT_HOLD_30S,
                     profile.SUPPORTED_POLICY_GAIN_STEP_3S, profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                     profile.SUPPORTED_PRELOAD_5S, profile.SUPPORTED_POLICY_MIX_STEP_10PCT):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(profile.ProfileError):
                profile._accel_input_hypothesis_scope(data)
        for duration in (2., 10.001, 20., True, float('nan')):
            data = copy.deepcopy(self.data); data['duration_s'] = duration
            with self.subTest(duration=duration), self.assertRaises(profile.ProfileError): profile._settings(data)
        for mode in (profile.SUPPORTED_POLICY_PROBE, profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(profile.ProfileError): profile._settings(data)

    def test_new_named_extension_review_and_actual_physical_observation_still_required(self):
        self.docs['prior_supported_observation']['box_support_maintained'] = False
        with self.assertRaisesRegex(profile.ProfileError, 'operator observation'): self.load()
        self.docs['prior_supported_observation']['box_support_maintained'] = True
        self.extension['review']['decision'] = 'APPROVED_SUPPORTED_CHARACTERIZATION'
        with self.assertRaisesRegex(profile.ProfileError, 'Review has not approved'): self.load()

    def test_nonhypothesis_extension_still_requires_unchanged_diagnostic(self):
        fixture = extension_fixture.SupportedExtensionProfileTests()
        fixture.setUp(); self.addCleanup(fixture.doCleanups)
        fixture.docs['pipeline_diagnostic']['SYNTHETIC_DIAGNOSTIC_CHANGED'] = True
        with self.assertRaisesRegex(profile.ProfileError, 'changes prior execution'):
            fixture.load()


if __name__ == '__main__':
    unittest.main()
