"""Synthetic file-only V2 2 -> 10 -> 20 contracts; never physical evidence."""
import copy
import json
import math
from pathlib import Path
import shutil
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.policy_post_reply_timing import PostReplyDeadlineBudget
from test_post_reply_input_age_v2 import settings
import test_accel_hypothesis_extension as hypothesis_fixture
import test_policy_supported_extension_profile as extension_fixture


def select_v2(data, hardware):
    data['post_reply_deadline_policy'] = settings()
    data['voltage_pipeline'] = True
    for key in ('post_reply_deadline_acceptance', 'rare_jitter_diagnostic_acceptance',
                'voltage_pipeline_acceptance', 'native_batch_encoder_acceptance',
                'startup_cycle_acceptance', 'prepared_voltage_publication_acceptance'):
        if key not in hardware:
            continue
        row = hardware[key]
        row.pop('hard_output_and_freshness_limits_unchanged', None)
        row.pop('live_deadline_policy_unchanged', None)
        row.update(pre_send_input_and_native_output_limits_unchanged=True,
                   output_feedback_sample_age_limit_unchanged=True,
                   post_reply_input_age_budget_ms=1.)
    hardware['post_reply_deadline_acceptance'].update(
        schema='singularitydog.post-reply-input-age-review.v2', settings=settings())
    hardware['post_reply_deadline_acceptance']['review']['decision'] = 'ACCEPT_BOUNDED_POST_REPLY_INPUT_AGE_V2'


def wire_cycles(report, prior, provenance):
    """Create canonical toy original records, then derive admissions from them."""
    journal = []
    prepared = []
    budget = PostReplyDeadlineBudget(settings())
    for index, row in enumerate(report['cycles']):
        begin, end = row['begin_ns'], row['end_ns']
        inputs, outputs = [], []
        for bus, first in (('front', 1), ('rear', 7)):
            for phase, target in (('feedback_hold', inputs),
                                  ('graceful_stop' if row['phase'] == 'stopped' else
                                   'startup_hold' if row['phase'] == 'starting' else 'policy_output', outputs)):
                raw = []
                for ordinal, mid in enumerate(range(first, first+6)):
                    output = phase != 'feedback_hold'
                    start = begin+(12_000_000 if output else 100_000)+ordinal*800_000
                    received = begin+(18_000_000 if output else 6_000_000)+ordinal*100_000
                    tx = protocol._wire((1 << 24) | (32767 << 8) | mid,
                                        struct.pack('>4H', 32767, 32767, 0, 0))
                    rx = protocol._wire((2 << 24) | (2 << 22) | (mid << 8) | 0xFD,
                                        struct.pack('>4H', 32767, 32767, 32767, 300))
                    raw.append(dict(tx_hex=tx.hex(), rx_hex=rx.hex(), start_ns=start,
                        finish_ns=start+10_000, read_start_ns=received-1000,
                        received_ns=received, deadline_ns=begin+20_000_000, written=17, received=17))
                target.extend(raw)
                journal.append(dict(bus=bus, phase=phase, records=raw, error=None,
                    rejected_total=0, rejected_hex='', rejected_truncated=False))
            if prior.get('prepare_voltage_before_feedback_publication') is True:
                mid = first+index%6
                request = begin+9_030_000
                received = begin+10_000_000
                tx = codec.read_request(mid, 'voltage')
                rx = protocol._wire((17 << 24) | (mid << 8) | 0xFD,
                                    struct.pack('<HBBf', 0x701C, 0, 0, 40.))
                journal.append(dict(bus=bus, phase='overlapped_voltage', error=None,
                    stats=dict(begin_ns=begin+9_020_000), records=[dict(
                        tx_hex=tx.hex(), rx_hex=rx.hex(), start_ns=request,
                        finish_ns=request+10_000, read_start_ns=received-1000,
                        received_ns=received, deadline_ns=begin+20_000_000, written=17, received=17)]))
                prepared.append(dict(bus=bus, bus_cycle_index=index, voltage_motor_id=mid,
                    status='VALIDATED', error=None, feedback_validated_ns=begin+8_000_000,
                    prepared_before_publish_ns=begin+9_000_000,
                    publication_checked_after_ns=begin+9_010_000,
                    voltage_native_begin_ns=begin+9_020_000, voltage_first_request_ns=request,
                    voltage_validated_ns=begin+10_100_000, effective_deadline_ns=begin+20_000_000,
                    submitted_deadline_ns=begin+20_000_000))
        oldest = min(r['start_ns'] for r in inputs)
        final_write = max(r['finish_ns'] for r in outputs)
        replied = max(r['received_ns'] for r in outputs)
        row.update(index=index, output_reply_end_ns=replied,
            oldest_input_to_final_host_write_ms=(final_write-oldest)/1e6,
            imu=dict(read_started_monotonic_ns=begin+2_000_000,
                     read_finished_monotonic_ns=begin+2_100_000),
            imu_body=dict(accel_bias_subtracted=True, accel_scale_corrected=True,
                          accel_input_hypothesis=copy.deepcopy(provenance)))
        decision = budget.admit(index=index, begin_ns=begin, oldest_input_ns=oldest,
            final_write_ns=final_write, last_reply_ns=replied,
            output_sample_start_ns=min(r['start_ns'] for r in outputs), checked_ns=end,
            sample_age_ns=20_000_000,
            startup_allowed=index == 0 and prior.get('startup_cycle_allowance') == live.FIRST_CYCLE_POST_REPLY)
        row.update(post_reply_deadline=decision, deadline20ms_missed=end-begin > 20_000_000,
            steady_deadline20ms_missed=decision['allowance_used'],
            startup_20ms_allowance_used=decision['startup_allowance_used'])
    report.update(journal=journal, post_reply_deadline_policy=settings(),
        deadline20ms_misses=sum(r['deadline20ms_missed'] for r in report['cycles']),
        steady_deadline20ms_misses=budget.accepted_misses,
        post_reply_deadline_allowance_uses=budget.accepted_misses,
        startup_20ms_allowance_enabled=prior.get('startup_cycle_allowance') == live.FIRST_CYCLE_POST_REPLY,
        startup_20ms_allowance_uses=sum(r['startup_20ms_allowance_used'] for r in report['cycles']))
    if prior.get('prepare_voltage_before_feedback_publication') is True:
        report.update(prepare_voltage_before_feedback_publication=True,
            prepared_voltage_publication=dict(schema='singularitydog.active-prepared-voltage-publication.v1',
                mode='validate_feedback_prepare_voltage_publish_then_native',
                transport_capability='singularitydog.active-prepared-exchange.v1',
                selection_bound_to_reviewed_profile=True,
                cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
                changes_deadline_or_cancellation_guards=False, records=prepared))


class V2TenSecondTests(unittest.TestCase):
    def setUp(self):
        hypothesis_fixture.AccelHypothesisExtensionTests.setUp(self)
        # Wire validation of the unchanged disabled diagnostic is covered by its
        # dedicated suite. Only that existing converter is mocked in this toy
        # fixture; every new actual-predecessor admission uses real wire bytes.
        self.enterContext(patch.object(live, '_voltage_fast_pipeline_trace'))
        self.docs['hardware_review']['voltage_pipeline_acceptance'] = dict(
            pipeline='feedback_then_voltage.fast_v1', scope=self.data['scope'],
            review={**self.data['review'], 'decision':'ACCEPT_FEEDBACK_THEN_VOLTAGE'})
        self.docs['hardware_review']['prepared_voltage_publication_acceptance'] = dict(
            schema='singularitydog.prepared-voltage-publication-review.v1',
            mode=live.PREPARED_VOLTAGE_PUBLICATION_MODE, scope=self.data['scope'],
            stop_proxy_does_not_certify_active_api_latency=True,
            review={**self.data['review'], 'decision':'ACCEPT_PREPARED_VOLTAGE_PUBLICATION'})
        self.data['prepare_voltage_before_feedback_publication'] = True
        select_v2(self.data, self.docs['hardware_review'])
        prior = self.docs['prior_supported_profile']
        prior.update(post_reply_deadline_policy=settings(), voltage_pipeline=True,
                     prepare_voltage_before_feedback_publication=True)
        actual = self.docs['prior_supported_report']
        actual['execution_settings'] = live.execution_settings(prior)
        wire_cycles(actual, prior, self.provenance)
        diagnostic = self.docs['pipeline_diagnostic']
        diagnostic.update(prepare_voltage_before_feedback_publication=True,
            boot_id=self.data['boot_id'], source_provenance=dict(
                source_files_unchanged=True,
                cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
                motor_power_epoch=self.data['motor_power_epoch']))
        diagnostic['plan'].update(v3_voltage_fast_pipeline=True, v3_voltage_pipeline=False,
                                 prepare_voltage_before_feedback_publication=True)

    def seal(self):
        path = extension_fixture.SupportedExtensionProfileTests.seal(self)
        for key in ('post_reply_deadline_acceptance', 'voltage_pipeline_acceptance',
                    'prepared_voltage_publication_acceptance'):
            self.docs['hardware_review'][key]['diagnostic_sha256'] = self.data['artifacts']['pipeline_diagnostic']['sha256']
        return extension_fixture.SupportedExtensionProfileTests.seal(self)

    def load(self):
        return live.load_profile(self.seal())

    def test_full_loader_ten_seconds_preserves_v2_caps_and_bound_token(self):
        loaded = self.load()
        self.assertTrue(loaded['output_allowed'])
        self.assertTrue(loaded['support_must_remain'])
        self.assertEqual(loaded['duration_s'], 10.)
        self.assertEqual(live.post_reply_deadline_settings(loaded), settings())
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])
        self.assertEqual(loaded['axes'], {m:loaded['axes'][m] for m in live.IDS})

    def test_initial_age_allowance_only_is_recomputed_and_admits_existing_two_seconds(self):
        prior = self.docs['prior_supported_profile']
        prior['startup_cycle_allowance'] = self.data['startup_cycle_allowance'] = live.FIRST_CYCLE_POST_REPLY
        self.docs['hardware_review']['startup_cycle_acceptance'] = dict(
            mode=live.FIRST_CYCLE_POST_REPLY, scope=self.data['scope'], first_cycle_only=True,
            steady_miss_budget_unchanged=True, pre_send_input_and_native_output_limits_unchanged=True,
            output_feedback_sample_age_limit_unchanged=True, post_reply_input_age_budget_ms=1.,
            review={**self.data['review'], 'decision':'ACCEPT_FIRST_CYCLE_POST_REPLY'})
        actual = self.docs['prior_supported_report']
        actual['execution_settings'] = live.execution_settings(prior)
        actual['cycles'][0]['end_ns'] = actual['cycles'][0]['begin_ns']+20_500_000
        for row in actual['cycles'][1:]:
            row['begin_ns'] += 1_000_000; row['end_ns'] += 1_000_000
        diagnostic = self.docs['pipeline_diagnostic']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += 2_000_000
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += 2_000_000
        wire_cycles(actual, prior, self.provenance)
        self.assertTrue(actual['cycles'][0]['post_reply_deadline']['input_age_allowance_used'])
        self.assertEqual(actual['post_reply_deadline_allowance_uses'], 0)
        self.assertTrue(self.load()['output_allowed'])

    def test_pose_physical_limits_and_local_capture_must_remain_exact(self):
        original = copy.deepcopy(self.data)
        for change in (lambda p:p['axes']['1'].update(physical_lower_rad=p['axes']['1']['physical_lower_rad']-.0001),
                       lambda p:p.update(start_pose_bounds={'1':[0.,.1]}),
                       lambda p:p['artifacts']['local_reference_capture'].update(sha256='f'*64)):
            data = copy.deepcopy(original); change(data)
            with self.subTest(change=change), self.assertRaises(live.ProfileError):
                live._v2_supported_extension_context(self.docs, data, self.docs['prior_supported_profile'],
                                                      self.docs['prior_supported_report'])

    def test_false_summary_or_missing_cancelled_late_wire_cannot_admit(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r['cycles'][4]['post_reply_deadline'].update(input_sample_age_ms=0.),
                   lambda r:r['cycles'][4]['post_reply_deadline'].update(output_feedback_sample_age_ms=0.),
                   lambda r:r['cycles'][4].update(output_reply_end_ns=r['cycles'][4]['output_reply_end_ns']-1),
                   lambda r:r['journal'][0].update(error='cancelled'),
                   lambda r:r['journal'][0].update(rejected_total=1),
                   lambda r:r['journal'][0].update(rejected_total=False),
                   lambda r:r['journal'].pop(),
                   lambda r:r['journal'][1]['records'][-1].update(received_ns=r['cycles'][0]['begin_ns']+20_000_001),
                   lambda r:r['cycles'][0].update(index=False),
                   lambda r:r.update(startup_20ms_allowance_uses=True))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(live.ProfileError): self.load()

    def test_wrong_current_diagnostic_source_power_or_epoch_order_rejected(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        for change in (lambda d:d.update(motor_power_epoch='old'),
                       lambda d:d['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py':'f'*64}),
                       lambda d:d['measurements'][0].update(release_ns=1)):
            self.docs['pipeline_diagnostic'] = copy.deepcopy(saved); change(self.docs['pipeline_diagnostic'])
            with self.subTest(change=change), self.assertRaises(live.ProfileError): self.load()

    def test_steady_allowance_in_two_second_predecessor_remains_ineligible(self):
        actual = self.docs['prior_supported_report']
        actual['cycles'][4]['end_ns'] = actual['cycles'][4]['begin_ns']+20_500_000
        # Preserve causal next-cycle starts in this toy schedule.
        for row in actual['cycles'][5:]:
            row['begin_ns'] += 1_000_000; row['end_ns'] += 1_000_000
        wire_cycles(actual, self.docs['prior_supported_profile'], self.provenance)
        with self.assertRaises(live.ProfileError): self.load()


class V2TwentySecondTests(V2TenSecondTests):
    def setUp(self):
        V2TenSecondTests.setUp(self)
        self.load()
        self.history = self.base/'history-ten'
        self.history.mkdir()
        for p in self.base.iterdir():
            if p.is_file(): shutil.copyfile(p, self.history/p.name)
        ten = copy.deepcopy(self.data)
        for name, ref in ten['artifacts'].items():
            if name == 'accel_input_hypothesis':
                # Preserve the diagnostic's selected absolute artifact path.
                ref['path'] = str(self.base/Path(ref['path']).name)
            else:
                ref['path'] = str(self.history/Path(ref['path']).name)
        self.ten = ten
        self.data.update(duration_s=20., assembly_id='SYNTHETIC V2 twenty seconds',
            diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)
        self.docs['hardware_review']['assembly_id'] = self.data['assembly_id']
        origins = {mid:ten['axes'][mid]['sign']*self.docs['local_reference_capture']['telemetry']['rows'][mid]['median_position_rad']+ten['axes'][mid]['offset_rad'] for mid in live.IDS}
        q = [origins[str(mid)] for mid in live.shadow.CAN_ORDER]
        report = copy.deepcopy(self.docs['prior_supported_report'])
        rows = []
        for index in range(493):
            begin = 15_000_000_000+index*20_000_000
            phase = 'starting' if index == 0 else 'stopped' if index == 492 else 'active'
            gain = 1. if phase == 'active' else 0.
            rows.append(dict(index=index, begin_ns=begin, end_ns=begin+19_000_000,
                phase=phase, effective_policy_weight=.005*gain,
                command=dict(phase=phase, q_model_rad=q[:], kp=[3.*gain]*12, kd=[.15*gain]*12,
                    velocity_reference_rad_s=[0.]*12, feedforward_torque_nm=[0.]*12,
                    command_velocity_rad_s=[0.]*12, tracking_error_rad=[0.]*12, estimated_pd_torque_nm=[0.]*12),
                feedback=dict(q_model_rad=q[:], velocity_rad_s=[0.]*12, torque_nm=[0.]*12, temperature_c=[30.]*12)))
        report.update(cycles=rows, actual_model_calls=468,
            cadence_source_sha256=copy.deepcopy(ten['cadence_source_sha256']),
            execution_settings=live.execution_settings(ten), motor_enable_sent=True,
            motion_gain_sent=True, command_output_sent=True,
            trial_origin_model_rad_by_id=origins,
            model_provenance=dict(manifest_sha256=ten['artifacts']['scalar_step_manifest']['sha256'],
                baseline_provenance=dict(manifest_sha256=ten['artifacts']['model_manifest']['sha256'])))
        wire_cycles(report, ten, self.provenance)
        self.docs.update(prior_supported_profile=ten, prior_supported_report=report)
        # The new twenty-second review uses a second, later disabled diagnostic.
        diagnostic = self.docs['pipeline_diagnostic']
        shift = rows[-1]['end_ns']+1_000_000-diagnostic['measurements'][0]['release_ns']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += shift
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += shift
        self.extension = dict(mode=live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S, scope=self.data['scope'],
            only_duration_extended=True, live_limits_unchanged=True, support_must_remain=True,
            load_bearing_not_established=True, walking_allowed=False,
            review={**self.data['review'], 'decision':'ACCEPT_20S_SUPPORTED_AFTER_10S'})
        self.docs['hardware_review']['supported_extension_acceptance'] = self.extension

    def test_full_loader_ten_seconds_preserves_v2_caps_and_bound_token(self):
        loaded = self.load()
        self.assertEqual(loaded['duration_s'], 20.)
        self.assertEqual(live.post_reply_deadline_settings(loaded), settings())
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(loaded['_accel_input_hypothesis_provenance']['formal_calibration_approved'])
        self.assertIsNone(loaded['axes']['1']['uncertainty_rad'])

    def test_initial_age_allowance_only_is_recomputed_and_admits_existing_two_seconds(self):
        self.assertTrue(self.load()['output_allowed'])

    def test_steady_allowance_in_two_second_predecessor_remains_ineligible(self):
        report = self.docs['prior_supported_report']
        for index in (10, 110):
            report['cycles'][index]['end_ns'] = report['cycles'][index]['begin_ns']+20_500_000
            for row in report['cycles'][index+1:]:
                row['begin_ns'] += 1_000_000; row['end_ns'] += 1_000_000
        wire_cycles(report, self.ten, self.provenance)
        diagnostic = self.docs['pipeline_diagnostic']
        for row in diagnostic['measurements']:
            for key in row:
                if key.endswith('_ns'): row[key] += 2_000_000
        diagnostic['absolute_epoch_schedule']['epoch_ns'] += 2_000_000
        self.assertEqual(report['post_reply_deadline_allowance_uses'], 2)
        self.assertTrue(self.load()['output_allowed'])

    def test_nested_two_second_original_tamper_and_observation_rejected(self):
        path = Path(self.ten['artifacts']['prior_supported_report']['path'])
        path.write_bytes(path.read_bytes()+b' ')
        with self.assertRaisesRegex(live.ProfileError, 'SHA256'): self.load()

    def test_actual_hypothesis_must_match_on_all_ten_second_rows(self):
        self.docs['prior_supported_report']['cycles'][-1]['imu_body']['accel_input_hypothesis']['hypothesis_sha256'] = 'f'*64
        with self.assertRaisesRegex(live.ProfileError, 'actual input provenance'): self.load()

    def test_selected_prepared_proof_is_checked_on_ten_seconds(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        for change in (lambda r:r.pop('prepared_voltage_publication'),
                       lambda r:r.update(prepare_voltage_before_feedback_publication=False),
                       lambda r:r['prepared_voltage_publication']['records'][-1].update(status='FAILED'),
                       lambda r:r['prepared_voltage_publication']['records'][-1].update(effective_deadline_ns=1)):
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(live.ProfileError): self.load()


class V2ExtensionScopeTests(unittest.TestCase):
    def setUp(self):
        self.fixture = V2TenSecondTests()
        self.fixture.setUp(); self.addCleanup(self.fixture.doCleanups)
        self.data = copy.deepcopy(self.fixture.data)

    def test_only_exact_two_ten_twenty_seconds_allowed(self):
        for mode, valid in ((live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, 2.),
                            (live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S, 10.),
                            (live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S, 20.)):
            p = copy.deepcopy(self.data); p.update(diagnostic_timing_acceptance=mode, duration_s=valid)
            self.assertEqual(live._post_reply_policy(p), settings())
            live._accel_input_hypothesis_scope(p)
            for value in (True, float('nan'), float('inf'), valid-.001, valid+.001):
                p['duration_s'] = value
                with self.subTest(mode=mode, duration=value), self.assertRaises(live.ProfileError): live._post_reply_policy(p)

    def test_no_hold_human_ground_gain_or_larger_caps(self):
        for mode in (live.CURRENT_HOLD_AFTER_SUPPORTED_10S, live.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
                     live.FIXED_CATCH_CURRENT_HOLD_30S, live.SUPPORTED_POLICY_GAIN_STEP_3S):
            p = copy.deepcopy(self.data); p['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(live.ProfileError): live._post_reply_policy(p)
        for key,value in (('kp',3.001),('kd',.15001),('max_displacement_from_start_rad',math.radians(1.001)),
                           ('max_estimated_pd_torque_nm',.10001),('max_measured_velocity_rad_s',.35001)):
            p=copy.deepcopy(self.data);p['axes']['6'][key]=value
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live._post_reply_policy(p)
        for key,value in (('scope','ground'),('policy_weight',.00501),('max_sample_age_ms',20.001)):
            p=copy.deepcopy(self.data);p[key]=value
            with self.subTest(key=key), self.assertRaises(live.ProfileError): live._post_reply_policy(p)


if __name__ == '__main__':
    unittest.main()
