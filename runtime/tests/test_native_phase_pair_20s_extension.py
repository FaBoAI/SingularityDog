"""Synthetic full native 2/10/20 graph; no hardware or operator approvals.

Only the pre-existing disabled-diagnostic wire converter is mocked by the
reused fixture. Actual predecessor CAN/IMU records and admission proofs are
replayed by the production validators, with hash-bound file-only receipts.
"""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw.policy_post_reply_timing import POST_REPLY_POLICY
from test_policy_live_profile import _write
from test_policy_local_profile import seal_local
import test_post_reply_input_age_extensions as wire_fixture

wire_cycles = wire_fixture.wire_cycles


def _diagnostic_at(diagnostic, begin):
    shift = begin-diagnostic['measurements'][0]['release_ns']
    for row in diagnostic['measurements']:
        for key in row:
            if key.endswith('_ns'):
                row[key] += shift
    diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift


def _native_report(report, prior):
    report['execution_settings'] = live.execution_settings(prior)
    report['cadence_source_sha256'] = copy.deepcopy(prior['cadence_source_sha256'])
    report['transport_settings'] = dict(request_gap_us=prior['request_gap_us'],
                                        request_window=prior['request_window'])
    report['native_phase_pair'] = dict(enabled=True, mode='persistent_dual_owner.v1',
        paired_phases='ordinary_exchange_and_output',
        prepared_feedback_voltage_owners='existing_python_bus_owners',
        request_count_per_cycle=26, active_deadlines_unchanged=True,
        hardware_timing_improvement_proven=False)


class NativePairTwentySecondExtensionTests(unittest.TestCase):
    def setUp(self):
        fixture = wire_fixture.V2TwentySecondTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.base, self.data, self.docs = fixture.base, fixture.data, fixture.docs
        self.provenance = fixture.provenance
        self.ten = self.docs['prior_supported_profile']
        self.two = json.loads(Path(self.ten['artifacts']['prior_supported_profile']['path']).read_text())
        self.two_report = json.loads(Path(self.ten['artifacts']['prior_supported_report']['path']).read_text())
        self.two_observation = json.loads(Path(self.ten['artifacts']['prior_supported_observation']['path']).read_text())
        self.two_base = self.base/'native-two'
        self.ten_base = self.base/'native-ten'
        self.two_base.mkdir(); self.ten_base.mkdir()
        # Use exactly the same immutable model/encoder inputs at every depth;
        # the fixture's toy encoder is a pinned file and is never dlopened.
        for prior in (self.two, self.ten, self.data):
            prior.update(native_phase_pair=True, request_gap_us=890, request_window=3,
                         bundle_path=str(self.base/'bundle'))
            prior['cadence_source_sha256'] = live.cadence_source_hashes(prior)
        self.two['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = 'a'*64
        self.two_docs = {key: copy.deepcopy(self.docs[key]) for key in live.artifact_names(self.two)}
        self.ten_docs = {key: copy.deepcopy(self.docs[key]) for key in live.artifact_names(self.ten)}
        self.ten_docs.update(prior_supported_profile=self.two,
            prior_supported_report=self.two_report, prior_supported_observation=self.two_observation)
        self.ten_docs['hardware_review']['supported_extension_acceptance'].update(
            mode=live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
            review={**self.ten['review'], 'decision':'ACCEPT_10S_SUPPORTED_AFTER_2S'})
        for prior, docs in ((self.two, self.two_docs), (self.ten, self.ten_docs),
                            (self.data, self.docs)):
            self._select_stage(prior, docs)
        _diagnostic_at(self.two_docs['pipeline_diagnostic'], 1_000_000)
        self._move_report(self.two_report, 12_000_000_000, self.two)
        _diagnostic_at(self.ten_docs['pipeline_diagnostic'], 14_000_000_000)
        self._move_report(self.docs['prior_supported_report'], 26_000_000_000, self.ten)
        _diagnostic_at(self.docs['pipeline_diagnostic'], 36_000_000_000)
        self._seal_stage(self.two_base, self.two, self.two_docs)
        self._seal_stage(self.ten_base, self.ten, self.ten_docs)

    def _move_report(self, report, begin, prior):
        shift = begin-report['cycles'][0]['begin_ns']
        for row in report['cycles']:
            row['begin_ns'] += shift
            row['end_ns'] += shift
        wire_cycles(report, prior, self.provenance)
        _native_report(report, prior)

    def _select_stage(self, prior, docs):
        diagnostic = docs['pipeline_diagnostic']
        diagnostic.update(native_phase_pair=True,
            cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']))
        diagnostic['plan'].update(native_phase_pair=True, request_gap_us=890, request_window=3)
        diagnostic['source_provenance']['cadence_source_sha256'] = copy.deepcopy(prior['cadence_source_sha256'])
        diagnostic['native_phase_pair_proof'] = dict(mode='persistent_dual_owner.v1',
            request_count_per_cycle=26, all_phases_joined=True,
            owner_placement_verified=True, owner_settings_restored=True,
            coordinator_placement_verified=True, coordinator_settings_restored=True,
            active_deadlines_unchanged=True)
        hardware = docs['hardware_review']
        hardware['assembly_id'] = prior['assembly_id']
        for row in hardware['source_captures']:
            row['path'] = str(self.base/Path(row['path']).name)
        hardware['native_phase_pair_acceptance'] = dict(
            schema='singularitydog.native-phase-pair-review.v1', mode='persistent_dual_owner.v1',
            scope=prior['scope'], request_count_per_cycle=26, active_deadlines_unchanged=True,
            stop_proxy_does_not_certify_active_api_latency=True,
            cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
            pre_send_input_and_native_output_limits_unchanged=True,
            output_feedback_sample_age_limit_unchanged=True, post_reply_input_age_budget_ms=1.,
            review={**prior['review'], 'decision':'ACCEPT_NATIVE_PHASE_PAIR'})

    def _seal_stage(self, base, prior, docs):
        if 'prior_supported_profile' in docs:
            for name in ('prior_supported_profile', 'prior_supported_report', 'prior_supported_observation'):
                if name == 'prior_supported_report':
                    docs[name]['profile_sha256'] = prior['artifacts']['prior_supported_profile']['sha256']
                elif name == 'prior_supported_observation':
                    docs[name]['report_sha256'] = prior['artifacts']['prior_supported_report']['sha256']
                ref = _write(base/(name+'.json'), docs[name])
                prior['artifacts'][name] = dict(path=str(base/(name+'.json')), sha256=ref['sha256'])
        seal_local(base, prior, docs)
        for ref in prior['artifacts'].values():
            if not Path(ref['path']).is_absolute():
                ref['path'] = str(base/ref['path'])
        docs['pipeline_diagnostic']['plan']['accel_input_hypothesis'] = copy.deepcopy(
            prior['artifacts']['accel_input_hypothesis'])
        # The diagnostic changed only this pinned path; update its digest before
        # binding the dedicated reviews below.
        prior['artifacts']['pipeline_diagnostic'] = _write(base/'pipeline_diagnostic.json',
                                                          docs['pipeline_diagnostic'])
        hardware = docs['hardware_review']
        for key in ('post_reply_deadline_acceptance', 'rare_jitter_diagnostic_acceptance',
                    'voltage_pipeline_acceptance', 'prepared_voltage_publication_acceptance',
                    'native_phase_pair_acceptance'):
            hardware[key]['diagnostic_sha256'] = prior['artifacts']['pipeline_diagnostic']['sha256']
        if 'prior_supported_profile' in docs:
            acceptance = hardware['supported_extension_acceptance']
            for field, key in (('prior_profile_sha256','prior_supported_profile'),
                               ('prior_report_sha256','prior_supported_report'),
                               ('prior_observation_sha256','prior_supported_observation')):
                acceptance[field] = prior['artifacts'][key]['sha256']
        seal_local(base, prior, docs)
        # seal_local writes new relative references; bind every copied graph to
        # its own directory before its parent snapshots these original bytes.
        for ref in prior['artifacts'].values():
            if not Path(ref['path']).is_absolute():
                ref['path'] = str(base/ref['path'])
        return _write(base/'profile.json', prior)

    def seal(self):
        self._seal_stage(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def load(self):
        return live.load_profile(self.seal())

    def reseal_history(self):
        self._seal_stage(self.two_base, self.two, self.two_docs)
        self._seal_stage(self.ten_base, self.ten, self.ten_docs)

    def test_complete_loader_admits_only_bounded_twenty_and_binds_runtime_selection(self):
        value = self.load()
        self.assertTrue(live.native_phase_pair_settings(value))
        self.assertTrue(live.prepared_voltage_publication_settings(value))
        self.assertEqual(value['timing_review']['kind'], 'supported_policy_20s_after_10s_admission_only')
        self.assertEqual((value['duration_s'],value['policy_weight'],value['hard_cycle_ms'],
                          value['max_sample_age_ms'],value['request_gap_us'],value['request_window']),
                         (20.,.005,20.,20.,890,3))
        self.assertTrue(value['support_must_remain'])
        self.assertFalse(value['actual_policy_output_20ms_verified'])
        self.assertIsNone(value['axes']['1']['uncertainty_rad'])

    def test_raw_or_rebound_profile_cannot_mint_runtime_selection(self):
        with self.assertRaisesRegex(live.ProfileError, 'complete loader proof'):
            live.native_phase_pair_settings(self.data)
        value = self.load(); value['duration_s'] = 10.
        with self.assertRaises(live.ProfileError):live.native_phase_pair_settings(value)

    def test_scope_does_not_expand_duration_motion_pacing_or_freshness(self):
        for key, bad in (('duration_s',19.999),('duration_s',20.001),('duration_s',30.),
                         ('diagnostic_timing_acceptance',live.SUPPORTED_POLICY_PROBE_60S_AFTER_20S),
                         ('diagnostic_timing_acceptance',live.SUPPORTED_POLICY_PROBE_30S_PREAUTHORIZED),
                         ('policy_weight',.0051),('request_gap_us',891),('request_window',2),
                         ('hard_cycle_ms',21),('max_sample_age_ms',21),
                         ('max_sample_gap_ms',21.001),('max_consecutive_20ms_misses',1),
                         ('preauthorized_boxed_sequence',True)):
            candidate = copy.deepcopy(self.data); candidate[key] = bad
            with self.subTest(key=key, bad=bad), self.assertRaises(live.ProfileError):
                live._settings(candidate)
        for key, bad in (('kp',3.001),('kd',.151),('max_estimated_pd_torque_nm',.101),
                         ('max_displacement_from_start_rad',math.radians(1.001))):
            candidate = copy.deepcopy(self.data); candidate['axes']['1'][key] = bad
            with self.subTest(axis_key=key), self.assertRaises(live.ProfileError):live._settings(candidate)

    def test_two_and_ten_actual_native_route_required_not_just_execution_label(self):
        for report in (self.two_report, self.docs['prior_supported_report']):
            saved = copy.deepcopy(report['native_phase_pair'])
            for key, bad in (('enabled',False),('mode','legacy'),('paired_phases','output_only'),
                             ('prepared_feedback_voltage_owners','replacement_reader'),
                             ('request_count_per_cycle',24),('active_deadlines_unchanged',False)):
                report['native_phase_pair'] = copy.deepcopy(saved); report['native_phase_pair'][key] = bad
                self.reseal_history()
                with self.subTest(seconds=2 if report is self.two_report else 10,key=key), self.assertRaisesRegex(
                        live.ProfileError, 'selected actual native predecessor'): self.load()
            report['native_phase_pair'] = saved

    def test_new_diagnostic_must_follow_ten_seconds_and_keep_original_sources(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        for mutate in (lambda d:_diagnostic_at(d,1_000_000),
                       lambda d:d.update(boot_id='different-boot'),
                       lambda d:d.update(motor_power_epoch='different-power'),
                       lambda d:d['cadence_source_sha256'].update({'singularitydog_hw/native_active_transport.py':'f'*64}),
                       lambda d:d['plan'].update(request_gap_us=880),
                       lambda d:d['native_phase_pair_proof'].update(all_phases_joined=False)):
            self.docs['pipeline_diagnostic'] = copy.deepcopy(saved); mutate(self.docs['pipeline_diagnostic'])
            with self.subTest(mutate=mutate), self.assertRaises(live.ProfileError): self.load()

    def test_missing_two_or_ten_original_wire_cannot_be_replaced_with_success_summaries(self):
        for report in (self.two_report, self.docs['prior_supported_report']):
            saved = copy.deepcopy(report)
            for mutate in (lambda r:r['journal'].__setitem__(0,{**r['journal'][0],'records':[]}),
                           lambda r:r['journal'][0]['records'][0].update(received=0),
                           lambda r:r['journal'][0]['records'][0].update(rx_hex='00'),
                           lambda r:r['journal'][0]['records'][0].update(deadline_ns=1),
                           lambda r:r['cycles'][1]['imu'].update(read_started_monotonic_ns=1),
                           lambda r:r['cycles'][1].update(oldest_input_to_final_host_write_ms=0.)):
                report.clear(); report.update(copy.deepcopy(saved)); mutate(report)
                self.reseal_history()
                with self.subTest(seconds=2 if report is self.two_report else 10,mutate=mutate), self.assertRaises(
                        live.ProfileError): self.load()
            report.clear(); report.update(saved)

    def test_prior_selection_encoder_or_runtime_source_cannot_be_added_later(self):
        for prior in (self.two, self.ten):
            saved = copy.deepcopy(prior)
            for mutate in (lambda p:p.update(native_phase_pair=False),
                           lambda p:p.update(request_gap_us=900),
                           lambda p:p['native_batch_encoder'].update(sha256='f'*64),
                           lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64})):
                prior.clear(); prior.update(copy.deepcopy(saved)); mutate(prior)
                self.reseal_history()
                with self.subTest(seconds=prior['duration_s'],mutate=mutate), self.assertRaises(live.ProfileError):self.load()
            prior.clear(); prior.update(saved)

    def test_historical_two_and_ten_native_review_and_diagnostic_proof_are_not_waived(self):
        for docs in (self.two_docs, self.ten_docs):
            review = docs['hardware_review']['native_phase_pair_acceptance']; saved = copy.deepcopy(review)
            for mutate in (lambda a:a['review'].update(decision='APPROVED_SUPPORTED_CHARACTERIZATION'),
                           lambda a:a['cadence_source_sha256'].update({'singularitydog_hw/native_active_transport.py':'f'*64}),
                           lambda a:a.update(active_deadlines_unchanged=False)):
                review.clear(); review.update(copy.deepcopy(saved)); mutate(review); self.reseal_history()
                with self.subTest(docs='two' if docs is self.two_docs else 'ten',mutate=mutate), self.assertRaisesRegex(
                        live.ProfileError, 'native phase pair|Review has not approved'):self.load()
            review.clear(); review.update(saved)
            proof = docs['pipeline_diagnostic']['native_phase_pair_proof']
            proof['owner_settings_restored'] = False; self.reseal_history()
            with self.assertRaisesRegex(live.ProfileError, 'own current diagnostic'):self.load()
            proof['owner_settings_restored'] = True

    def test_stops_normal_ramp_and_direct_physical_confirmation_remain_required(self):
        report = self.docs['prior_supported_report']; saved = copy.deepcopy(report)
        for mutate in (lambda r:r.update(normal_ramp_completed=False),
                       lambda r:r.update(stop_confirmed=False),
                       lambda r:r['stop_reports']['rear'].update(confirmed_ids=list(range(7,12))),
                       lambda r:r['stop_reports']['front'].update(ambiguous_ids=[1])):
            report.clear();report.update(copy.deepcopy(saved));mutate(report)
            with self.subTest(mutate=mutate), self.assertRaises(live.ProfileError): self.load()
        report.clear();report.update(saved)
        observation = self.docs['prior_supported_observation']
        for key, bad in (('observed_by','summary_inference'),('audio_heard',False),
                         ('box_support_maintained',False),('abnormal_noise_vibration_slip_sinking_contact',True)):
            old = observation[key]; observation[key] = bad
            with self.subTest(key=key), self.assertRaises(live.ProfileError):self.load()
            observation[key] = old

    def test_legacy_v1_still_replays_original_wire_for_native_extension(self):
        prior = copy.deepcopy(self.ten); report = copy.deepcopy(self.docs['prior_supported_report'])
        prior['post_reply_deadline_policy']['mode'] = POST_REPLY_POLICY
        prior['post_reply_deadline_policy'].pop('post_reply_input_age_budget_ms')
        with patch('test_post_reply_input_age_extensions.settings',
                   return_value=copy.deepcopy(prior['post_reply_deadline_policy'])):
            wire_cycles(report, prior, {})
        live._native_phase_pair_20s_predecessor(report, prior)
        report.pop('journal')
        with self.assertRaisesRegex(live.ProfileError, 'original cycle and wire'):
            live._native_phase_pair_20s_predecessor(report, prior)


if __name__ == '__main__': unittest.main()
