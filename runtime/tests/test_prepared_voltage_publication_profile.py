"""Synthetic admission/binding tests; no native load, model, device or approvals.

The pre-existing fast-pipeline wire validator is explicitly mocked here. Its
original tests cover raw records; this file isolates the new selection contract.
"""
import copy
import json
import math
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as profile
from singularitydog_hw import can_readonly as codec
from test_policy_local_profile import seal_local
import test_policy_rare_jitter_profile as rare
import test_accel_hypothesis_extension as extension


KEY = 'prepare_voltage_before_feedback_publication'


class PreparedVoltageProfileTests(unittest.TestCase):
    def setUp(self):
        rare.RareJitterProfileTests.setUp(self)
        self.enterContext(patch.object(profile, '_voltage_fast_pipeline_trace'))
        self.select()

    def select(self):
        self.data[KEY] = True
        self.data['voltage_pipeline'] = True
        diagnostic = self.docs['pipeline_diagnostic']
        diagnostic[KEY] = True
        diagnostic['plan'].update({KEY: True, 'v3_voltage_fast_pipeline': True})
        diagnostic.update(motor_power_epoch=self.data['motor_power_epoch'],
                          cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']))
        diagnostic['source_provenance'] = dict(source_files_unchanged=True,
            cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
            motor_power_epoch=self.data['motor_power_epoch'])
        hardware = self.docs['hardware_review']
        hardware['voltage_pipeline_acceptance'] = dict(
            pipeline='feedback_then_voltage.fast_v1', scope=self.data['scope'],
            hard_output_and_freshness_limits_unchanged=True,
            review={**self.data['review'], 'decision': 'ACCEPT_FEEDBACK_THEN_VOLTAGE'})
        hardware['prepared_voltage_publication_acceptance'] = dict(
            schema='singularitydog.prepared-voltage-publication-review.v1',
            mode=profile.PREPARED_VOLTAGE_PUBLICATION_MODE, scope=self.data['scope'],
            hard_output_and_freshness_limits_unchanged=True,
            stop_proxy_does_not_certify_active_api_latency=True,
            review={**self.data['review'], 'decision': 'ACCEPT_PREPARED_VOLTAGE_PUBLICATION'})

    def seal(self):
        seal_local(self.base, self.data, self.docs)
        hardware = self.docs['hardware_review']
        for name in ('rare_jitter_diagnostic_acceptance', 'voltage_pipeline_acceptance',
                     'prepared_voltage_publication_acceptance'):
            if name in hardware:
                hardware[name]['diagnostic_sha256'] = self.data['artifacts']['pipeline_diagnostic']['sha256']
        seal_local(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def load(self):
        return profile.load_profile(self.seal())

    def test_complete_loader_exposes_selection_without_claiming_active_latency(self):
        parsed = self.load()
        self.assertIs(profile.prepared_voltage_publication_settings(parsed), True)
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])
        self.assertTrue(parsed['support_must_remain'])
        self.assertEqual(set(profile.execution_settings(parsed)),
            {'model_backend', 'voltage_overlap', 'voltage_pipeline', 'diagnostic_timing_acceptance'})
        self.assertEqual((parsed['hard_cycle_ms'], parsed['max_sample_age_ms']), (20., 20.))

    def test_absent_and_false_preserve_historical_execution_settings(self):
        for value in ('absent', False):
            data = profile.template()
            if value is False:
                data['schema'] = profile.SCHEMA_V3
                data[KEY] = False
            self.assertIs(profile.prepared_voltage_publication_settings(data), False)
            self.assertEqual(profile.execution_settings(data), dict(model_backend='native_baseline',
                voltage_overlap=False, voltage_pipeline=False, diagnostic_timing_acceptance=None))
            self.assertNotIn('_prepared_voltage_publication_token', data)

    def test_raw_json_and_unapproved_plan_cannot_mint_selected_token(self):
        with self.assertRaisesRegex(profile.ProfileError, 'complete loader proof'):
            profile.prepared_voltage_publication_settings(self.data)
        self.data.update(approved_for_supported_policy_output=False,
                         blockers=['SYNTHETIC NOT REVIEWED'], review=None)
        # Serialize the valid structure directly; never invent a review for PLAN.
        path = self.base/'unapproved.json'
        path.write_text(json.dumps(self.data))
        parsed = profile.load_profile(path, require_approved=False)
        self.assertFalse(parsed['output_allowed'])
        with self.assertRaisesRegex(profile.ProfileError, 'complete loader proof'):
            profile.prepared_voltage_publication_settings(parsed)

    def test_false_or_removed_selection_after_loading_is_rejected(self):
        for remove in (False, True):
            parsed = self.load()
            if remove:
                parsed.pop(KEY)
            else:
                parsed[KEY] = False
            with self.assertRaisesRegex(profile.ProfileError, 'changed after loading'):
                profile.prepared_voltage_publication_settings(parsed)

    def test_nonboolean_and_legacy_selected_values_are_rejected(self):
        for value in (None, 0, 1, '', [], {}):
            data = copy.deepcopy(self.data); data[KEY] = value
            with self.subTest(value=value), self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)
        for schema in (profile.SCHEMA_V1, profile.SCHEMA_V2):
            data = copy.deepcopy(self.data); data['schema'] = schema
            with self.subTest(schema=schema), self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)

    def test_only_boxed_short_policy_pipeline_routes_are_eligible(self):
        for key, value in (('voltage_pipeline', False), ('voltage_overlap', False),
                           ('model_backend', 'native_baseline'), ('local_characterization', None),
                           ('scope', profile.FIXED_CATCH_SCOPE), ('duration_s', 2.001),
                           ('policy_weight', 0), ('policy_weight', .005001),
                           ('hard_cycle_ms', 20.001), ('max_sample_age_ms', 20.001),
                           ('max_sample_gap_ms', 21.001), ('max_consecutive_20ms_misses', 1)):
            data = copy.deepcopy(self.data); data[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)
        for mode in (profile.CURRENT_HOLD_PROBE, profile.CURRENT_HOLD_AFTER_SUPPORTED_10S,
                     profile.SUPPORTED_POLICY_GAIN_STEP_3S, profile.SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                     profile.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
                     profile.FIXED_CATCH_CURRENT_HOLD_30S, profile.SUPPORTED_PRELOAD_5S):
            data = copy.deepcopy(self.data); data['diagnostic_timing_acceptance'] = mode
            with self.subTest(mode=mode), self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)

    def test_selected_route_does_not_expand_gains_or_motion_monitors(self):
        for key, value in (('kp', 3.001), ('kd', .15001),
                           ('max_displacement_from_start_rad', math.radians(1.001)),
                           ('max_command_velocity_rad_s', math.radians(1.001)),
                           ('max_estimated_pd_torque_nm', .10001),
                           ('max_measured_torque_nm', 1.001), ('max_temperature_c', 45.01)):
            data = copy.deepcopy(self.data); data['axes']['7'][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)

    def test_matching_selection_required_in_both_diagnostic_locations(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        for location in ('report', 'plan'):
            for value in ('absent', False, None, 1):
                diagnostic = copy.deepcopy(saved)
                target = diagnostic if location == 'report' else diagnostic['plan']
                if value == 'absent': target.pop(KEY)
                else: target[KEY] = value
                self.docs['pipeline_diagnostic'] = diagnostic
                with self.subTest(location=location, value=value), self.assertRaisesRegex(
                        profile.ProfileError, 'publication selection'):
                    self.load()

    def test_selected_diagnostic_cannot_qualify_unselected_profile(self):
        for value in ('absent', False):
            if value == 'absent': self.data.pop(KEY, None)
            else: self.data[KEY] = value
            with self.subTest(value=value), self.assertRaisesRegex(profile.ProfileError, 'publication selection'):
                self.load()

    def test_null_diagnostic_plan_fails_with_profile_error(self):
        self.docs['pipeline_diagnostic']['plan'] = None
        with self.assertRaises(profile.ProfileError):
            profile._timing(self.docs['pipeline_diagnostic'], self.data)

    def test_current_sources_boot_epoch_and_finish_recheck_required(self):
        saved = copy.deepcopy(self.docs['pipeline_diagnostic'])
        changes = (lambda r:r.update(boot_id='different'),
                   lambda r:r.update(motor_power_epoch='different'),
                   lambda r:r['cadence_source_sha256'].update({'singularitydog_hw/policy_output_runtime.py': 'f'*64}),
                   lambda r:r['source_provenance'].update(source_files_unchanged=False),
                   lambda r:r['source_provenance'].update(motor_power_epoch='different'),
                   lambda r:r.pop('source_provenance'))
        for change in changes:
            self.docs['pipeline_diagnostic'] = copy.deepcopy(saved); change(self.docs['pipeline_diagnostic'])
            with self.subTest(change=change), self.assertRaisesRegex(profile.ProfileError, 'exact current sources'):
                self.load()

    def test_unchanged_experimental_admission_rejections_remain(self):
        for key in ('trace_copy_provenance', 'sourced_boot_guard'):
            for location in ('report', 'plan'):
                target = self.docs['pipeline_diagnostic']
                if location == 'plan': target = target['plan']
                target[key] = {}
                with self.subTest(key=key, location=location), self.assertRaisesRegex(
                        profile.ProfileError, 'Experimental'):
                    self.load()
                target.pop(key)

    def test_named_acceptance_and_active_latency_boundary_are_required(self):
        acceptance = self.docs['hardware_review']['prepared_voltage_publication_acceptance']
        for key, value in (('schema', 'different'), ('mode', 'different'),
                           ('scope', 'different'), ('hard_output_and_freshness_limits_unchanged', False),
                           ('stop_proxy_does_not_certify_active_api_latency', False)):
            old = acceptance[key]; acceptance[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(profile.ProfileError, 'publication acceptance'):
                self.load()
            acceptance[key] = old
        acceptance['review']['decision'] = 'APPROVED_SUPPORTED_CHARACTERIZATION'
        with self.assertRaisesRegex(profile.ProfileError, 'Review has not approved'):
            self.load()

    def test_missing_or_old_diagnostic_acceptance_is_rejected(self):
        self.docs['hardware_review'].pop('prepared_voltage_publication_acceptance')
        with self.assertRaisesRegex(profile.ProfileError, 'publication acceptance'):
            self.load()
        self.select(); path = self.seal()
        doc = self.docs['hardware_review']
        doc['prepared_voltage_publication_acceptance']['diagnostic_sha256'] = 'f'*64
        # Bind the changed hardware artifact without repairing its bad inner reference.
        from test_policy_live_profile import _write
        self.data['artifacts']['hardware_review'] = _write(self.base/'hardware_review.json', doc)
        _write(path, self.data)
        with self.assertRaisesRegex(profile.ProfileError, 'publication acceptance'):
            profile.load_profile(path)

    def test_mutation_of_executable_input_review_source_or_epoch_revokes_token(self):
        changes = (lambda p:p.update(boot_id='different'), lambda p:p.update(motor_power_epoch='different'),
                   lambda p:p.update(approved_for_supported_policy_output=False),
                   lambda p:p.update(output_allowed=False), lambda p:p['axes']['1'].update(offset_rad=.001),
                   lambda p:p['artifacts']['calibration'].update(sha256='f'*64),
                   lambda p:p['cadence_source_sha256'].update({'singularitydog_hw/policy_output.py':'f'*64}),
                   lambda p:p['review'].update(rationale='changed'),
                   lambda p:p.update(_prepared_voltage_publication_token=object()))
        for change in changes:
            parsed = self.load(); change(parsed)
            with self.subTest(change=change), self.assertRaises(profile.ProfileError):
                profile.prepared_voltage_publication_settings(parsed)

    def test_selection_is_bound_into_named_review_settings(self):
        old = profile.reviewed_settings_sha256(self.data)
        self.data[KEY] = False
        self.assertNotEqual(profile.reviewed_settings_sha256(self.data), old)


class PreparedVoltageExtensionTests(unittest.TestCase):
    select = PreparedVoltageProfileTests.select
    load = PreparedVoltageProfileTests.load

    def setUp(self):
        extension.AccelHypothesisExtensionTests.setUp(self)
        self.enterContext(patch.object(profile, '_voltage_fast_pipeline_trace'))
        self.select()
        prior = self.docs['prior_supported_profile']
        prior[KEY] = True; prior['voltage_pipeline'] = True
        self.docs['prior_supported_report'].update({KEY: True,
            'execution_settings': profile.execution_settings(prior)})
        actual = self.docs['prior_supported_report']
        proof = dict(schema='singularitydog.active-prepared-voltage-publication.v1',
            mode='validate_feedback_prepare_voltage_publish_then_native',
            transport_capability='singularitydog.active-prepared-exchange.v1',
            selection_bound_to_reviewed_profile=True,
            cadence_source_sha256=copy.deepcopy(prior['cadence_source_sha256']),
            changes_deadline_or_cancellation_guards=False, hardware_timing_improvement_proven=False,
            records=[])
        actual['prepared_voltage_publication'] = proof
        actual['journal'] = []
        for index, cycle in enumerate(actual['cycles']):
            begin = cycle['begin_ns']
            for bus, first in (('front', 1), ('rear', 7)):
                mid = first+index%6
                proof['records'].append(dict(bus=bus, bus_cycle_index=index, voltage_motor_id=mid,
                    feedback_validated_ns=begin+9_000_000, prepared_before_publish_ns=begin+10_000_000,
                    publication_checked_after_ns=begin+11_000_000, voltage_native_begin_ns=begin+12_000_000,
                    voltage_first_request_ns=begin+12_100_000, voltage_validated_ns=begin+14_100_000,
                    effective_deadline_ns=begin+20_000_000, submitted_deadline_ns=begin+20_000_000,
                    status='VALIDATED', error=None))
                can_id = (17 << 24) | (mid << 8) | codec.HOST_ID
                rx = b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+bytes([8])+struct.pack('<Hxxf', 0x701c, 40.)+b'\r\n'
                actual['journal'].append(dict(bus=bus, phase='overlapped_voltage', error=None,
                    stats=dict(begin_ns=begin+12_000_000), records=[dict(
                        tx_hex=codec.read_request(mid, 'voltage').hex(), rx_hex=rx.hex(),
                        written=17, received=17, start_ns=begin+12_100_000,
                        finish_ns=begin+12_200_000, received_ns=begin+14_000_000,
                        deadline_ns=begin+20_000_000)]))

    def seal(self):
        extension.AccelHypothesisExtensionTests.seal(self)
        hardware = self.docs['hardware_review']
        for name in ('voltage_pipeline_acceptance', 'prepared_voltage_publication_acceptance'):
            hardware[name]['diagnostic_sha256'] = self.data['artifacts']['pipeline_diagnostic']['sha256']
        seal_local(self.base, self.data, self.docs)
        return self.base/'profile.json'

    def test_selected_evidenced_ten_second_extension_loads(self):
        parsed = self.load()
        self.assertIs(profile.prepared_voltage_publication_settings(parsed), True)
        self.assertEqual(parsed['duration_s'], 10.)
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])

    def test_legacy_or_unselected_actual_predecessor_cannot_be_relabelled(self):
        report = self.docs['prior_supported_report']
        for value in ('absent', False):
            if value == 'absent': report.pop(KEY, None)
            else: report[KEY] = value
            with self.subTest(value=value), self.assertRaisesRegex(profile.ProfileError, 'selected actual predecessor'):
                self.load()

    def test_prior_profile_selection_must_be_identical(self):
        self.docs['prior_supported_profile'][KEY] = False
        with self.assertRaisesRegex(profile.ProfileError, 'changes prior execution'):
            self.load()

    def test_aborted_prior_output_cannot_enable_extension(self):
        self.docs['prior_supported_report']['status'] = 'ABORTED'
        with self.assertRaisesRegex(profile.ProfileError, 'successful same-session'):
            self.load()

    def test_boolean_only_predecessor_lacks_production_edge_proof(self):
        self.docs['prior_supported_report'].pop('prepared_voltage_publication')
        with self.assertRaisesRegex(profile.ProfileError, 'production-edge proof'):
            self.load()

    def test_failed_duplicate_or_missing_owner_proof_rejected(self):
        actual = self.docs['prior_supported_report']; saved = copy.deepcopy(actual)
        changes = (lambda r:r['prepared_voltage_publication']['records'].pop(),
                   lambda r:r['prepared_voltage_publication']['records'][0].update(status='FAILED'),
                   lambda r:r['prepared_voltage_publication']['records'].__setitem__(1,
                       copy.deepcopy(r['prepared_voltage_publication']['records'][0])),
                   lambda r:r['prepared_voltage_publication'].update(transport_capability='duck-typed'),
                   lambda r:r['prepared_voltage_publication']['cadence_source_sha256'].update(
                       {'singularitydog_hw/native_active_transport.py':'f'*64}))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved)
            change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError):
                self.load()

    def test_prepared_publication_causality_and_tighten_only_deadline_rejected(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        for key, value in (('prepared_before_publish_ns', 1),
                           ('effective_deadline_ns', saved['cycles'][0]['begin_ns']+20_000_001),
                           ('submitted_deadline_ns', saved['cycles'][0]['begin_ns']+20_000_001),
                           ('voltage_motor_id', 7), ('voltage_first_request_ns', True)):
            self.docs['prior_supported_report'] = copy.deepcopy(saved)
            self.docs['prior_supported_report']['prepared_voltage_publication']['records'][0][key] = value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError):
                self.load()

    def test_matching_raw_voltage_journal_is_required(self):
        saved = copy.deepcopy(self.docs['prior_supported_report'])
        changes = (lambda r:r['journal'].pop(),
                   lambda r:r['journal'][0]['records'][0].update(deadline_ns=1),
                   lambda r:r['journal'][0]['records'][0].update(tx_hex=codec.read_request(2, 'voltage').hex()),
                   lambda r:r['journal'][0]['records'][0].update(rx_hex='00'),
                   lambda r:r['journal'][0]['stats'].update(begin_ns=1),
                   lambda r:r['journal'][0].update(error='failed'))
        for change in changes:
            self.docs['prior_supported_report'] = copy.deepcopy(saved); change(self.docs['prior_supported_report'])
            with self.subTest(change=change), self.assertRaises(profile.ProfileError):
                self.load()

    def test_valid_frame_with_voltage_below_existing_floor_rejected(self):
        raw = self.docs['prior_supported_report']['journal'][0]['records'][0]
        wire = bytearray.fromhex(raw['rx_hex']); wire[11:15] = struct.pack('<f', 34.9)
        raw['rx_hex'] = wire.hex()
        with self.assertRaisesRegex(profile.ProfileError, 'voltage proof is out of scope'):
            self.load()


if __name__ == '__main__':
    unittest.main()
