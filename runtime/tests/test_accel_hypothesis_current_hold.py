"""Synthetic file-only admission for a boxed current hold; no hardware I/O."""
import copy
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
import test_policy_hold_after_supported_profile as hold_fixture


class AccelHypothesisCurrentHoldTests(unittest.TestCase):
    def setUp(self):
        hold_fixture.HoldAfterSupportedProfileTests.setUp(self)
        self.data['accel_input_hypothesis'] = True
        self.data['cadence_source_sha256'] = profile.cadence_source_hashes(self.data)
        for axis in self.data['axes'].values():
            axis.update(kp=6., kd=.15, max_estimated_pd_torque_nm=.2)
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
        prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'b'*64
        actual = self.docs['prior_supported_report']
        actual['cadence_source_sha256'] = copy.deepcopy(prior['cadence_source_sha256'])
        for row in actual['cycles']:
            row['imu_body'] = dict(accel_bias_subtracted=True, accel_scale_corrected=True,
                                  accel_input_hypothesis=copy.deepcopy(self.provenance))
        diagnostic = self.docs['pipeline_diagnostic']
        diagnostic.update(motor_power_epoch=self.data['motor_power_epoch'],
                          cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']))
        diagnostic['plan']['accel_input_hypothesis'] = copy.deepcopy(self.reference)
        diagnostic['input_sha256']['accel_input_hypothesis'] = self.reference['sha256']
        diagnostic['observer']['accel_input_hypothesis'] = copy.deepcopy(self.provenance)
        shift = actual['cycles'][-1]['end_ns'] + 1_000_000 - diagnostic['measurements'][0]['release_ns']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += shift
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift

    seal = hold_fixture.HoldAfterSupportedProfileTests.seal
    load = hold_fixture.HoldAfterSupportedProfileTests.load

    def test_same_hypothesis_successful_ten_seconds_and_fresh_diagnostic_admit_exact_boxed_hold(self):
        loaded = self.load()
        self.assertEqual((loaded['duration_s'], loaded['policy_weight'], loaded['startup_duration_s']), (3.,0.,1.))
        self.assertTrue(profile.current_position_hold_only(loaded))
        self.assertTrue(loaded['support_must_remain'])
        self.assertTrue(all(a['kp']==6. and a['kd']==.15 and a['max_estimated_pd_torque_nm']==.2 and
                            a['uncertainty_rad'] is None for a in loaded['axes'].values()))
        self.assertEqual(profile.accel_input_hypothesis_settings(loaded), self.reference)
        self.assertIsNone(profile.post_reply_deadline_settings(loaded))
        self.assertFalse(profile.reviewed_startup_cycle_allowance(loaded))
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['formal_calibration_approved'])
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['grants_motor_output'])
        self.assertIsNone(loaded['_accel_input_hypothesis_provenance']['absolute_orientation_error_bound_rad'])

    def test_generic_hold_without_successful_ten_second_predecessor_is_rejected(self):
        prior = self.docs['prior_supported_profile']
        prior['duration_s'] = 2.
        with self.assertRaisesRegex(profile.ProfileError, 'ten-second supported predecessor'): self.load()
        self.data['diagnostic_timing_acceptance'] = profile.CURRENT_HOLD_PROBE
        with self.assertRaisesRegex(profile.ProfileError, 'Acceleration input hypothesis'):
            profile._settings(self.data)

    def test_prior_raw_or_other_hypothesis_cannot_be_substituted(self):
        saved = copy.deepcopy(self.docs['prior_supported_profile'])
        for change in (lambda p:p.update(accel_input_hypothesis=False),
                       lambda p:p['artifacts']['accel_input_hypothesis'].update(sha256='f'*64)):
            self.docs['prior_supported_profile'] = copy.deepcopy(saved); change(self.docs['prior_supported_profile'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_every_prior_active_and_stopping_cycle_must_use_same_noncertifying_correction(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r['cycles'][25]['imu_body'].pop('accel_input_hypothesis'),
                   lambda r:r['cycles'][400]['imu_body']['accel_input_hypothesis'].update(candidate_sha256='f'*64),
                   lambda r:r['cycles'][-1]['imu_body'].update(accel_scale_corrected=False),
                   lambda r:r['cycles'][-1]['imu_body'].update(reviewed_accel_calibration={}),
                   lambda r:r['cycles'][0]['imu_body']['accel_input_hypothesis'].update(formal_calibration_approved=True))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaisesRegex(profile.ProfileError, 'actual input provenance'):
                self.load()

    def test_failed_incomplete_or_ambiguous_prior_output_is_rejected(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r.update(status='ABORTED'), lambda r:r.update(normal_ramp_completed=False),
                   lambda r:r.update(learned_targets_sent=False), lambda r:r.update(cyclic_inference_skipped=True),
                   lambda r:r.update(cycles=r['cycles'][:474]), lambda r:r.update(actual_model_calls=399),
                   lambda r:r['stop_reports']['rear'].update(ambiguous_ids=[10]))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_new_diagnostic_requires_current_source_graph_power_and_same_input(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes = (lambda r:r.update(motor_power_epoch='other-epoch'),
                   lambda r:r.update(cadence_source_sha256=self.docs['prior_supported_profile']['cadence_source_sha256']),
                   lambda r:r['plan']['accel_input_hypothesis'].update(sha256='f'*64),
                   lambda r:r['observer']['accel_input_hypothesis'].update(hypothesis_sha256='f'*64))
        for change in changes:
            self.docs['pipeline_diagnostic'] = copy.deepcopy(saved); change(self.docs['pipeline_diagnostic'])
            with self.subTest(change=change), self.assertRaisesRegex(profile.ProfileError, 'hypothesis'):
                self.load()

    def test_new_diagnostic_must_follow_ten_second_run(self):
        diagnostic = self.docs['pipeline_diagnostic']
        shift = self.docs['prior_supported_report']['cycles'][-1]['end_ns']-diagnostic['measurements'][0]['release_ns']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += shift
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift
        with self.assertRaisesRegex(profile.ProfileError, 'must follow'): self.load()

    def test_exact_gains_zero_mix_strict_live_timing_and_no_new_envelope(self):
        saved = copy.deepcopy(self.data)
        changes = [lambda p:p.update(duration_s=3.001), lambda p:p.update(policy_weight=.0001),
                   lambda p:p.update(hard_cycle_ms=20.001), lambda p:p.update(max_sample_age_ms=20.001),
                   lambda p:p.update(startup_cycle_allowance=profile.FIRST_CYCLE_POST_REPLY),
                   lambda p:p.update(post_reply_deadline_policy=self.docs['prior_supported_profile']['post_reply_deadline_policy'])]
        for key, value in (('kp',3.),('kp',6.001),('kd',.1),('kd',.150001),
                           ('max_estimated_pd_torque_nm',.200001),('max_measured_velocity_rad_s',.250001),
                           ('max_displacement_from_start_rad',math.radians(1.001))):
            changes.append(lambda p,key=key,value=value:p['axes']['6'].update({key:value}))
        for change in changes:
            self.data = copy.deepcopy(saved); change(self.data)
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_model_calibration_other_sources_boot_and_power_cannot_change(self):
        saved = copy.deepcopy(self.docs['prior_supported_profile'])
        changes = [lambda p:p.update(boot_id='changed'), lambda p:p.update(motor_power_epoch='changed'),
                   lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64})]
        for key in ('calibration','mount','bias','model_manifest','scalar_step_manifest','command_loss_report'):
            changes.append(lambda p,key=key:p['artifacts'][key].update(sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_profile'] = copy.deepcopy(saved); change(self.docs['prior_supported_profile'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError): self.load()

    def test_human_ground_fixed_catch_gain_or_twenty_second_permission_never_follows(self):
        for mode in (profile.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S, profile.FIXED_CATCH_CURRENT_HOLD_30S,
                     profile.SUPPORTED_POLICY_GAIN_STEP_3S, profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                     profile.SUPPORTED_PRELOAD_5S, profile.SUPPORTED_POLICY_MIX_STEP_10PCT):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(profile.ProfileError): profile._settings(data)
        for key,value in (('scope','human_supported_partial_current_hold_only'),
                          ('scope','ground'),('human_supported_hold',{}),('fixed_catch',{})):
            data = copy.deepcopy(self.data); data[key]=value
            with self.subTest(key=key,value=value), self.assertRaises(profile.ProfileError): profile._settings(data)

    def test_named_hold_acceptance_and_operator_observation_remain_required(self):
        self.docs['prior_supported_observation']['box_support_maintained'] = False
        with self.assertRaisesRegex(profile.ProfileError, 'operator observation'): self.load()
        self.docs['prior_supported_observation']['box_support_maintained'] = True
        self.acceptance['review']['decision'] = 'APPROVED_SUPPORTED_CHARACTERIZATION'
        with self.assertRaisesRegex(profile.ProfileError, 'Review has not approved'): self.load()

    def test_old_no_hypothesis_hold_still_cannot_replace_diagnostic(self):
        fixture=hold_fixture.HoldAfterSupportedProfileTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.docs['pipeline_diagnostic']['SYNTHETIC_CHANGED'] = True
        with self.assertRaisesRegex(profile.ProfileError, 'changes predecessor execution'): fixture.load()

    def test_loading_keeps_all_historical_bytes(self):
        path=self.seal()
        paths=[self.base/(key+'.json') for key in ('prior_supported_profile','prior_supported_report','prior_supported_observation')]
        before=[p.read_bytes() for p in paths]
        profile.load_profile(path)
        self.assertEqual(before,[p.read_bytes() for p in paths])


if __name__ == '__main__':
    unittest.main()
