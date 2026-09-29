"""Synthetic loader checks for a short, supported learned gain increase."""

import copy
import unittest

from singularitydog_hw import policy_live_profile as profile
from test_policy_supported_extension_profile import SupportedExtensionProfileTests


class SupportedGainStepProfileTests(unittest.TestCase):
    def setUp(self):
        SupportedExtensionProfileTests.setUp(self)
        self.data.update(
            assembly_id='synthetic-gain-step', duration_s=3.,
            startup_duration_s=1., policy_weight=.01,
            diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_GAIN_STEP_3S)
        self.docs['hardware_review']['assembly_id'] = self.data['assembly_id']
        self.docs['prior_supported_report'].update(
            startup_20ms_allowance_uses=0, steady_deadline20ms_misses=0,
            motor_enable_sent=True, actual_model_calls=90)
        for axis in self.data['axes'].values():
            axis.update(kp=6., max_estimated_pd_torque_nm=.2)
        for mid, axis in self.data['axes'].items():
            self.docs['hardware_review']['angles'][mid].update(
                physical_lower_rad=axis['physical_lower_rad'],
                physical_upper_rad=axis['physical_upper_rad'])
        self.acceptance = dict(
            mode=profile.SUPPORTED_POLICY_GAIN_STEP_3S,
            scope=self.data['scope'], support_must_remain=True,
            load_bearing_not_established=True, walking_allowed=False,
            review={**self.data['review'],
                    'decision': 'ACCEPT_3S_SUPPORTED_LEARNED_GAIN_STEP'})
        self.docs['hardware_review'].pop('supported_extension_acceptance', None)
        self.docs['hardware_review']['supported_gain_step_acceptance'] = self.acceptance

    def load(self):
        # The shared fixture seals all three predecessor files and binds their
        # hashes to the current profile and hardware review.
        path = SupportedExtensionProfileTests.seal(self)
        for field, name in (
                ('prior_profile_sha256', 'prior_supported_profile'),
                ('prior_report_sha256', 'prior_supported_report'),
                ('prior_observation_sha256', 'prior_supported_observation')):
            self.acceptance[field] = self.data['artifacts'][name]['sha256']
        path = SupportedExtensionProfileTests.seal(self)
        return profile.load_profile(path)

    def test_bounded_gain_step_from_completed_learned_predecessor(self):
        loaded = self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertEqual(loaded['timing_review']['kind'],
                         'supported_policy_gain_step_3s_admission_only')
        self.assertEqual((loaded['duration_s'], loaded['policy_weight']), (3., .01))
        self.assertEqual(loaded['axes']['1']['kp'], 6.)
        self.assertTrue(loaded['support_must_remain'])

    def test_gain_step_rejects_unreviewed_escalation(self):
        self.load()
        for key, value in (('kp', 12.001),
                           ('max_estimated_pd_torque_nm', .501),
                           ('max_displacement_from_start_rad', .1)):
            old = self.data['axes']['1'][key]
            self.data['axes']['1'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError):
                self.load()
            self.data['axes']['1'][key] = old
        for key, value in (('duration_s', 3.001),
                           ('policy_weight', .011),
                           ('startup_duration_s', .999)):
            old = self.data[key]
            self.data[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError):
                self.load()
            self.data[key] = old

    def test_predecessor_output_and_observation_are_required(self):
        self.load()
        report = self.docs['prior_supported_report']
        original = copy.deepcopy(report)
        for key, value in (('status', 'ABORTED'), ('stop_confirmed', False),
                           ('learned_targets_sent', False),
                           ('steady_deadline20ms_misses', 1)):
            report[key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError):
                self.load()
            report.clear(); report.update(copy.deepcopy(original))
        self.docs['prior_supported_observation']['box_support_maintained'] = False
        with self.assertRaises(profile.ProfileError):
            self.load()


if __name__ == '__main__':
    unittest.main()
