"""Synthetic extension evidence only; no device, native loading or output."""
import copy
import hashlib
import json
from pathlib import Path
import unittest

from singularitydog_hw import policy_live_profile as profile
from test_policy_live_profile import _write
from test_policy_local_profile import seal_local
import test_policy_rare_jitter_profile as rare_fixture


class SupportedExtensionProfileTests(unittest.TestCase):
    def setUp(self):
        rare_fixture.RareJitterProfileTests.setUp(self)
        binary = Path(self.data['bundle_path'])/'synthetic-encoder.so'
        if not binary.is_absolute(): binary = self.base/binary
        binary.write_bytes(b'SYNTHETIC file-only encoder pin; never loaded')
        binary_sha = hashlib.sha256(binary.read_bytes()).hexdigest()
        self.data['native_batch_encoder'] = dict(path=binary.name, sha256=binary_sha)
        self.docs['hardware_review']['native_batch_encoder_acceptance'] = dict(
            binary_sha256=binary_sha, scope=self.data['scope'], hard_output_and_freshness_limits_unchanged=True,
            review={**self.data['review'], 'decision':'ACCEPT_NATIVE_BATCH_ENCODER'})
        prior_path = rare_fixture.RareJitterProfileTests.seal(self)
        profile.load_profile(prior_path)
        prior = json.loads(prior_path.read_text())
        self.data.update(duration_s=10., assembly_id='synthetic-extension',
            diagnostic_timing_acceptance=profile.SUPPORTED_POLICY_PROBE_10S_AFTER_2S)
        self.docs['hardware_review']['assembly_id']=self.data['assembly_id']
        # The old loader's bytes are retained; its admission-only change must
        # not require recursive loading against the new loader source hash.
        prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'a'*64
        rows = []
        for index in range(92):
            begin = 1_000_000_000+index*20_000_000
            end = begin+19_000_000
            rows.append(dict(index=index, begin_ns=begin, output_reply_end_ns=begin+18_000_000,
                end_ns=end, phase='starting' if index==0 else 'stopped' if index==91 else 'active',
                effective_policy_weight=0. if index in (0,91) else .005,
                post_reply_deadline=dict(accepted=True, checked_ns=end, allowance_used=False,
                                        startup_allowance_used=False)))
        report = dict(status='COMPLETE_SUPPORTED_OUTPUT', errors=[], scope=prior['scope'],
            boot_id=prior['boot_id'], motor_power_epoch=prior['motor_power_epoch'],
            cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
            normal_ramp_completed=True, learned_targets_sent=True, stop_confirmed=True,
            deadline20ms_misses=0, post_reply_deadline_allowance_uses=0, post_reply_deadline_rejections=[],
            trial_displacement_origin='final_pre_enable_feedback', cycles=rows,
            execution_settings=profile.execution_settings(prior),
            native_batch_encoder=dict(enabled=True,binary_sha256=binary_sha),
            transport_settings=dict(request_gap_us=prior['request_gap_us'],request_window=prior['request_window']),
            stop_reports={scope:dict(complete=True,confirmed_ids=ids,unconfirmed_ids=[],ambiguous_ids=[],
                                    fault_by_id={str(mid):0 for mid in ids})
                for scope,ids in (('front',list(range(1,7))),('rear',list(range(7,13))))})
        observed = dict(user_statement='SYNTHETIC operator observation only', observed_by='operator',
            audio_heard=True, abnormal_noise_vibration_slip_sinking_contact=False,
            box_support_maintained=True, autonomous_standing_or_walking_observed=False)
        self.docs.update(prior_supported_profile=prior, prior_supported_report=report,
                         prior_supported_observation=observed)
        self.extension = dict(mode=profile.SUPPORTED_POLICY_PROBE_10S_AFTER_2S, scope=self.data['scope'],
            only_duration_extended=True,live_limits_unchanged=True,
            review={**self.data['review'],'decision':'ACCEPT_10S_SUPPORTED_AFTER_2S'})
        self.docs['hardware_review']['supported_extension_acceptance'] = self.extension

    def seal(self):
        for name in ('prior_supported_profile','prior_supported_report','prior_supported_observation'):
            if name=='prior_supported_report':
                self.docs[name]['profile_sha256']=self.data['artifacts']['prior_supported_profile']['sha256']
            if name=='prior_supported_observation':
                self.docs[name]['report_sha256']=self.data['artifacts']['prior_supported_report']['sha256']
            self.data['artifacts'][name]=_write(self.base/(name+'.json'),self.docs[name])
        for field,name in (('prior_profile_sha256','prior_supported_profile'),
                           ('prior_report_sha256','prior_supported_report'),
                           ('prior_observation_sha256','prior_supported_observation')):
            self.extension[field]=self.data['artifacts'][name]['sha256']
        seal_local(self.base,self.data,self.docs)
        return self.base/'profile.json'

    def load(self):
        return profile.load_profile(self.seal())

    def test_ten_seconds_requires_completed_two_second_run_and_keeps_live_limits(self):
        loaded=self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertEqual(loaded['timing_review']['kind'],'supported_policy_10s_after_2s_admission_only')
        self.assertEqual((loaded['duration_s'],loaded['hard_cycle_ms'],loaded['max_sample_age_ms']),(10.,20.,20.))
        self.assertEqual(loaded['post_reply_deadline_policy']['max_misses_per_window'],1)
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertTrue(loaded['support_must_remain'])

    def test_legacy_mode_caps_and_ten_second_cap_remain(self):
        self.data['duration_s']=10.001
        with self.assertRaises(profile.ProfileError): self.load()
        for mode,cap in ((profile.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,2.),
                         (profile.SUPPORTED_POLICY_PROBE_5S,5.)):
            self.data.update(diagnostic_timing_acceptance=mode,duration_s=cap+.001)
            with self.subTest(mode=mode),self.assertRaises(profile.ProfileError): profile._settings(self.data)

    def test_prior_session_encoder_runtime_and_numerical_contract_changes_rejected(self):
        prior=self.docs['prior_supported_profile']; original=copy.deepcopy(prior)
        changes=(lambda p:p.update(boot_id='changed'),lambda p:p.update(motor_power_epoch='changed'),
            lambda p:p.update(request_gap_us=1000),lambda p:p.update(policy_weight=.004),
            lambda p:p.update(model_backend='native_baseline'),lambda p:p['axes']['1'].update(kp=2.),
            lambda p:p['axes']['1'].update(offset_rad=.123),lambda p:p['axes']['1'].update(uid='changed'),
            lambda p:p['native_batch_encoder'].update(sha256='f'*64),
            lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64}),
            lambda p:p['artifacts']['model_manifest'].update(sha256='f'*64))
        for change in changes:
            self.docs['prior_supported_profile']=copy.deepcopy(original);change(self.docs['prior_supported_profile'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError): self.load()

    def test_failed_partial_timing_or_ambiguous_stop_cannot_admit_extension(self):
        baseline=copy.deepcopy(self.docs['prior_supported_report'])
        changes=(lambda r:r.update(status='ABORTED'),lambda r:r.update(errors=['failure']),
            lambda r:r.update(normal_ramp_completed=False),lambda r:r.update(learned_targets_sent=False),
            lambda r:r.update(deadline20ms_misses=1),lambda r:r.update(post_reply_deadline_allowance_uses=1),
            lambda r:r.update(cycles=r['cycles'][:79]),
            lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1]),
            lambda r:r['stop_reports']['front']['fault_by_id'].update({'1':1}),
            lambda r:r['cycles'][4].update(end_ns=r['cycles'][4]['begin_ns']+20_000_001),
            lambda r:r['cycles'][-1].update(phase='active'),
            lambda r:[row.update(effective_policy_weight=0.) for row in r['cycles']])
        for change in changes:
            self.docs['prior_supported_report']=copy.deepcopy(baseline);change(self.docs['prior_supported_report'])
            with self.subTest(change=change),self.assertRaises(profile.ProfileError): self.load()

    def test_matching_actual_operator_observation_required(self):
        observed=self.docs['prior_supported_observation'];baseline=copy.deepcopy(observed)
        for key,value in (('audio_heard',False),('abnormal_noise_vibration_slip_sinking_contact',True),
                          ('box_support_maintained',False),('observed_by','inferred'),('user_statement',''),
                          ('autonomous_standing_or_walking_observed',True)):
            self.docs['prior_supported_observation']=copy.deepcopy(baseline)
            self.docs['prior_supported_observation'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError): self.load()

    def test_artifact_and_review_hash_tampering_rejected(self):
        path=self.seal()
        report_path=self.base/'prior_supported_report.json'
        report_path.write_text(report_path.read_text()+' ')
        with self.assertRaisesRegex(profile.ProfileError,'SHA256'):profile.load_profile(path)
        for key,value in (('prior_report_sha256','f'*64),('prior_profile_sha256','f'*64),
                          ('prior_observation_sha256','f'*64),('live_limits_unchanged',False),
                          ('only_duration_extended',False)):
            self.seal();old=self.extension[key];self.extension[key]=value
            seal_local(self.base,self.data,self.docs)
            with self.subTest(key=key),self.assertRaisesRegex(profile.ProfileError,'extension review'):
                profile.load_profile(path)
            self.extension[key]=old


if __name__ == '__main__':unittest.main()
