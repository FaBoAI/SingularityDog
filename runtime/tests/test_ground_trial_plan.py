"""Synthetic, file-only gate tests. No fixture is real commissioning evidence."""

import copy
import hashlib
import json
import math
import unittest

from singularitydog_hw.ground_trial_plan import (
    GroundPlanError, STAGES, ground_plan_settings_sha256, prerequisites,
    template_ground_plan, validate_ground_plan,
)


def digest(value):
    return hashlib.sha256(value.encode()).hexdigest()


def base_profile():
    return {'output_allowed': True, 'approved_for_supported_policy_output': True,
            'scope': 'supported_characterization_only', 'blockers': [],
            'profile_sha256': digest('SYNTHETIC UNIT TEST PROFILE'),
            'assembly_id': 'UNIT_TEST_ONLY', 'boot_id': 'UNIT_TEST_BOOT',
            'motor_power_epoch': 'UNIT_TEST_EPOCH', 'duration_s': 6.,
            'startup_duration_s': .5, 'policy_ramp_s': .5, 'stop_duration_s': .4,
            'policy_weight': 1.,
            'hard_cycle_ms': 20., 'max_consecutive_20ms_misses': 0,
            'max_sample_age_ms': 20., 'timing_review': {'twenty_ms_misses': 0},
            'axes': {str(i): {'uid': f'unit-test-uid-{i}',
                             'max_command_velocity_rad_s': .1,
                             'max_command_acceleration_rad_s2': 1.}
                     for i in range(1, 13)}}


def evaluation(stage, base):
    hashes = {'report': digest('SYNTHETIC REPORT '+stage),
              'plan': digest('SYNTHETIC PLAN '+stage),
              'profile': base['profile_sha256'],
              'video': digest('NO REAL VIDEO '+stage)}
    return {'schema': 'singularitydog.ground-trial-evaluation.v1', 'stage': stage,
            'status': 'PASS_REVIEWED_STAGE', 'decision': 'PASS_REVIEWED_HARDWARE_STAGE',
            'dependency_eligible': True, 'genuine_hardware_capture': True,
            'simulated': False, 'replayed': False, 'learned_policy_used': True,
            'fault_free': True, 'all_axis_stop_confirmed': True,
            'physical_review_complete': True,
            'independent_body_catch_used': stage != 'supported_stance',
            'actual_controller_20ms_pass': True, 'max_active_cycle_ms': 18.7,
            'assembly_id': base['assembly_id'], 'base_profile_sha256': base['profile_sha256'],
            'uids_by_id': {mid: row['uid'] for mid, row in base['axes'].items()},
            'report_sha256': hashes['report'], 'stage_plan_sha256': hashes['plan'],
            'video_sha256': hashes['video'], 'physical_review_sha256': digest('SYNTHETIC REVIEW'),
            'reviewed_by': 'UNIT_TEST_ONLY', 'reviewed_at': '2026-09-28T00:00:00+09:00',
            'association': {'references': {key: {'path': '/synthetic/'+key,
                                                 'sha256': value}
                                           for key, value in hashes.items()},
                            'sync': {'trial_start_ns': 1000000000,
                                     'trial_end_ns': 7000000000,
                                     'video_start_s': 1., 'video_end_s': 7.,
                                     'uncertainty_ms': 20.}}}


def seal(plan):
    plan['review'] = {'reviewer': 'UNIT_TEST_ONLY',
                      'reviewed_at': '2026-09-28T00:00:00+09:00',
                      'decision': 'ALLOW_BOUNDED_GROUND_TRIAL',
                      'rationale': 'Synthetic validator fixture, no physical approval',
                      'settings_sha256': ground_plan_settings_sha256(plan)}


def replace_prior(plan, records, stage, report):
    raw = json.dumps(report, sort_keys=True).encode()
    records[stage] = raw
    plan['prior_evaluations'][stage] = {'path': '/synthetic/'+stage+'.json',
                                       'sha256': hashlib.sha256(raw).hexdigest()}
    seal(plan)


def fixture(stage='supported_stance'):
    base = base_profile()
    plan = template_ground_plan(stage, base)
    plan['approved_for_ground_trial'] = True
    plan['blockers'] = []
    plan['catch'] = {'kind': 'support_in_place' if stage == 'supported_stance' else 'two_operators',
                     'operator_count': 2, 'full_weight_capacity_reviewed': True,
                     'motor_disabled_recovery_reviewed': True, 'roles_separated': True,
                     'review_note': 'SYNTHETIC fixture; cannot establish real physical readiness'}
    records = {}
    for prior in prerequisites(stage):
        replace_prior(plan, records, prior, evaluation(prior, base))
    seal(plan)
    return plan, base, records


class GroundTrialPlanTests(unittest.TestCase):
    def test_boxed_acceleration_hypothesis_cannot_enter_any_ground_stage(self):
        for stage in STAGES:
            plan, base, records = fixture(stage)
            base['accel_input_hypothesis'] = True
            with self.subTest(stage=stage), self.assertRaisesRegex(GroundPlanError, 'Boxed acceleration'):
                validate_ground_plan(plan, base, records)

    def test_supported_only_usb_waiver_cannot_enter_ground_execution(self):
        plan,base,records=fixture()
        base['watchdog_review_policy']='command_loss_only_supported_trial'
        with self.assertRaisesRegex(GroundPlanError,'Supported-only'):
            validate_ground_plan(plan,base,records)

    def test_validated_four_stages_never_grant_live_output(self):
        for stage in STAGES:
            with self.subTest(stage=stage):
                plan, base, records = fixture(stage)
                result = validate_ground_plan(plan, base, records)
                self.assertTrue(result['execution_plan_validated'])
                self.assertFalse(result['output_allowed'])
                self.assertTrue(result['physical_arming_required'])
                self.assertEqual(result['trajectory'], plan['trajectory'])

    def test_template_default_is_inert_and_does_not_invent_catch(self):
        for stage in STAGES:
            with self.subTest(stage=stage):
                plan = template_ground_plan(stage)
                result = validate_ground_plan(plan, None, require_approved=False)
                self.assertFalse(result['execution_plan_validated'])
                self.assertFalse(result['output_allowed'])
                self.assertIsNone(plan['review'])
                self.assertIsNone(plan['catch']['operator_count'])
                with self.assertRaises(GroundPlanError):
                    validate_ground_plan(plan, base_profile())

    def test_result_and_inputs_are_detached(self):
        plan, base, records = fixture('walk')
        original = copy.deepcopy((plan, base, records))
        result = validate_ground_plan(plan, base, records)
        result['trajectory']['forward_velocity_m_s'] = .9
        result['catch']['operator_count'] = 0
        self.assertEqual((plan, base, records), original)

    def test_unapproved_or_unresolved_base_never_releases_ground(self):
        for key, value in [('output_allowed', False), ('approved_for_supported_policy_output', False),
                           ('blockers', ['pending physical review']), ('scope', 'walking')]:
            plan, base, records = fixture()
            base[key] = value
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_all_identity_and_epoch_bindings_are_exact(self):
        for key in ('base_profile_sha256', 'assembly_id', 'boot_id', 'motor_power_epoch'):
            plan, base, records = fixture()
            plan[key] = digest('different') if key.endswith('sha256') else 'changed'
            seal(plan)
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_tampered_settings_require_new_review(self):
        plan, base, records = fixture('walk')
        plan['trajectory']['forward_velocity_m_s'] = .03
        with self.assertRaisesRegex(GroundPlanError, 'changed after review'):
            validate_ground_plan(plan, base, records)

    def test_named_review_timezone_and_decision_required(self):
        for key, value in [('reviewer', ''), ('reviewed_at', '2026-09-28T00:00:00'),
                           ('decision', 'ALLOW_ALL'), ('rationale', ''),
                           ('settings_sha256', digest('different'))]:
            plan, base, records = fixture()
            plan['review'][key] = value
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_unknown_fields_cannot_smuggle_motion_or_physical_arming(self):
        for parent, key, value in [(None, 'output_allowed', True),
                                   ('trajectory', 'lateral_velocity_m_s', .01),
                                   ('trajectory', 'yaw_velocity_rad_s', .01),
                                   ('catch', 'current_ready', True)]:
            plan, base, records = fixture('walk')
            (plan if parent is None else plan[parent])[key] = value
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_fixed_independent_catch_allows_one_cutoff_operator(self):
        plan, base, records = fixture('stand')
        plan['catch'].update(kind='fixed_catch', operator_count=1, roles_separated=False)
        seal(plan)
        result = validate_ground_plan(plan, base, records)
        self.assertTrue(result['execution_plan_validated'])
        self.assertFalse(result['output_allowed'])

    def test_two_people_roles_cannot_be_collapsed_to_one(self):
        for count, separated in [(1, True), (2, False), (True, True), (0, True)]:
            plan, base, records = fixture('partial_load')
            plan['catch'].update(operator_count=count, roles_separated=separated)
            seal(plan)
            with self.subTest(count=count, separated=separated), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_full_weight_disabled_motor_recovery_are_independent_requirements(self):
        for kind in ('fixed_catch', 'two_operators'):
            for key in ('full_weight_capacity_reviewed', 'motor_disabled_recovery_reviewed'):
                plan, base, records = fixture('stand')
                plan['catch']['kind'] = kind
                plan['catch'][key] = False
                seal(plan)
                with self.subTest(kind=kind, key=key), self.assertRaises(GroundPlanError):
                    validate_ground_plan(plan, base, records)

    def test_support_in_place_is_not_an_independent_ground_catch(self):
        for stage in ('partial_load', 'stand', 'walk'):
            plan, base, records = fixture(stage)
            plan['catch']['kind'] = 'support_in_place'
            seal(plan)
            with self.subTest(stage=stage), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_forward_speed_and_ramps_are_bounded(self):
        for key, value in [('forward_velocity_m_s', -.01), ('forward_velocity_m_s', .050001),
                           ('forward_velocity_m_s', 0), ('ramp_up_s', .499),
                           ('ramp_down_s', .499), ('active_duration_s', .99),
                           ('active_duration_s', 5.001)]:
            plan, base, records = fixture('walk')
            plan['trajectory'][key] = value
            seal(plan)
            with self.subTest(key=key, value=value), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)
        plan, base, records = fixture('walk')
        plan['trajectory']['forward_velocity_m_s'] = .05
        seal(plan)
        self.assertTrue(validate_ground_plan(plan, base, records)['execution_plan_validated'])

    def test_walk_ramps_must_fit_active_interval(self):
        plan, base, records = fixture('walk')
        plan['trajectory']['ramp_up_s'] = .6
        seal(plan)
        with self.assertRaises(GroundPlanError):
            validate_ground_plan(plan, base, records)

    def test_stationary_stages_never_command_locomotion(self):
        for stage in STAGES[:-1]:
            for key in ('forward_velocity_m_s', 'ramp_up_s', 'ramp_down_s'):
                plan, base, records = fixture(stage)
                plan['trajectory'][key] = .01
                seal(plan)
                with self.subTest(stage=stage, key=key), self.assertRaises(GroundPlanError):
                    validate_ground_plan(plan, base, records)

    def test_timing_never_silently_extends_duration_or_shortens_recovery(self):
        for key, value in [('duration_s', 7.), ('initial_hold_s', .999),
                           ('final_stationary_s', .499), ('resupport_window_s', .999),
                           ('shutdown_reserve_s', .539), ('shutdown_reserve_s', .6),
                           ('active_duration_s', 4.)]:
            plan, base, records = fixture()
            plan['trajectory'][key] = value
            seal(plan)
            with self.subTest(key=key, value=value), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_shutdown_reserve_uses_slowest_axis_braking(self):
        plan, base, records = fixture('walk')
        base['axes']['12']['max_command_acceleration_rad_s2'] = .2
        with self.assertRaisesRegex(GroundPlanError, 'braking'):
            validate_ground_plan(plan, base, records)
        plan['trajectory']['shutdown_reserve_s'] = .94
        seal(plan)
        self.assertTrue(validate_ground_plan(plan, base, records)['execution_plan_validated'])

    def test_every_numeric_trajectory_field_rejects_bool_nan_infinity(self):
        for key in template_ground_plan('walk')['trajectory']:
            if key == 'stage':
                continue
            for value in (True, math.nan, math.inf, -math.inf, '1'):
                plan, base, records = fixture('walk')
                plan['trajectory'][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(GroundPlanError):
                    # NaN cannot even be signed as finite JSON.
                    seal(plan)
                    validate_ground_plan(plan, base, records)

    def test_all_preceding_stages_and_only_those_are_required(self):
        self.assertEqual(prerequisites('walk'), ('supported_stance', 'partial_load', 'stand'))
        plan, base, records = fixture('walk')
        del records['partial_load']
        with self.assertRaises(GroundPlanError):
            validate_ground_plan(plan, base, records)
        plan, base, records = fixture('supported_stance')
        records['walk'] = b'{}'
        with self.assertRaises(GroundPlanError):
            validate_ground_plan(plan, base, records)

    def test_pinned_evaluation_is_exact_bytes_not_dictionary(self):
        for value in (evaluation('supported_stance', base_profile()), b'{}', b'{"x":1}'):
            plan, base, records = fixture('partial_load')
            records['supported_stance'] = value
            with self.subTest(value=type(value)), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_replay_simulation_or_missing_hardware_fact_cannot_release_stage(self):
        mutations = [('genuine_hardware_capture', False), ('simulated', True),
                     ('replayed', True), ('fault_free', False), ('all_axis_stop_confirmed', False),
                     ('physical_review_complete', False), ('dependency_eligible', False),
                     ('status', 'RECORDED_REVIEW_REQUIRED'), ('decision', 'PASS'),
                     ('schema', 'unknown'), ('stage', 'stand')]
        for key, value in mutations:
            plan, base, records = fixture('partial_load')
            report = evaluation('supported_stance', base)
            report[key] = value
            replace_prior(plan, records, 'supported_stance', report)
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_prior_catch_and_full_learned_policy_are_required_before_walk(self):
        for stage, key in [('partial_load', 'independent_body_catch_used'),
                           ('stand', 'independent_body_catch_used'), ('stand', 'learned_policy_used')]:
            plan, base, records = fixture('walk')
            report = evaluation(stage, base)
            report[key] = False
            replace_prior(plan, records, stage, report)
            with self.subTest(stage=stage, key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_partial_policy_blend_is_not_named_autonomous_stand_or_walk(self):
        for stage in ('stand', 'walk'):
            plan, base, records = fixture(stage)
            base['policy_weight'] = .1
            with self.subTest(stage=stage), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_walk_rejects_supported_diagnostic_deadline_waiver(self):
        for key, value in [('hard_cycle_ms', 60.), ('max_consecutive_20ms_misses', 1),
                           ('max_sample_age_ms', 21.), ('timing_review', {'twenty_ms_misses': 1})]:
            plan, base, records = fixture('walk')
            base[key] = value
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_walk_requires_actual_stand_mit_timing_not_only_proxy_timing(self):
        for key, value in [('actual_controller_20ms_pass', False), ('max_active_cycle_ms', 20.001),
                           ('max_active_cycle_ms', None)]:
            plan, base, records = fixture('walk')
            report = evaluation('stand', base)
            report[key] = value
            replace_prior(plan, records, 'stand', report)
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_walk_has_separate_physical_distance_bound(self):
        for value in (None, 0, .049, .301, True, '0.1'):
            plan, base, records = fixture('walk')
            plan['maximum_measured_distance_m'] = value
            seal(plan)
            with self.subTest(value=value), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_prior_robot_profile_uids_and_recording_hashes_are_bound(self):
        for key, value in [('assembly_id', 'other robot'), ('base_profile_sha256', digest('other profile')),
                           ('uids_by_id', {}), ('report_sha256', None), ('video_sha256', ''),
                           ('physical_review_sha256', None), ('stage_plan_sha256', digest('other plan')),
                           ('reviewed_by', ''), ('reviewed_at', 'not a time')]:
            plan, base, records = fixture('partial_load')
            report = evaluation('supported_stance', base)
            report[key] = value
            replace_prior(plan, records, 'supported_stance', report)
            with self.subTest(key=key), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_association_references_and_sync_cannot_be_omitted_or_swapped(self):
        for mutation in ('missing_video', 'video_hash', 'interval', 'sync_missing', 'profile_hash'):
            plan, base, records = fixture('partial_load')
            report = evaluation('supported_stance', base)
            assoc = report['association']
            if mutation == 'missing_video':
                del assoc['references']['video']
            elif mutation in ('video_hash', 'profile_hash'):
                assoc['references'][mutation.split('_')[0]]['sha256'] = digest('swapped')
            elif mutation == 'interval':
                assoc['sync']['video_end_s'] = 0
            else:
                del assoc['sync']
            replace_prior(plan, records, 'supported_stance', report)
            with self.subTest(mutation=mutation), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)

    def test_duplicate_json_keys_and_overflow_float_fail_before_release(self):
        for raw in (b'{"status":"FAIL","status":"PASS_REVIEWED_STAGE"}',
                    b'{"unused":1e999}', b'{"unused":NaN}'):
            plan, base, records = fixture('partial_load')
            records['supported_stance'] = raw
            plan['prior_evaluations']['supported_stance']['sha256'] = hashlib.sha256(raw).hexdigest()
            seal(plan)
            with self.subTest(raw=raw), self.assertRaises(GroundPlanError):
                validate_ground_plan(plan, base, records)


if __name__ == '__main__':
    unittest.main()
