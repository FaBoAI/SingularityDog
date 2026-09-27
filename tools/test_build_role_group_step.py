"""File-only checks for bounded upper-leg diagnostic plans."""
import copy
import math
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import build_role_group_step as builder


class RoleGroupDirectionTests(unittest.TestCase):
    def setUp(self):
        self.centers = {str(mid): .1 * mid for mid in range(1, 13)}

    def prepared_candidate(self, profile='mirrored-thigh', group='thigh'):
        directions = builder.directions_for(group, profile)
        amplitude = builder.amplitude_for(group)
        return {
            'role_group': group, 'direction_profile': profile,
            'raw_direction_by_id': directions,
            'rows': [
                {'id': mid, 'start_raw_rad': self.centers[str(mid)],
                 'candidate_raw_step_deg': amplitude * directions[str(mid)],
                 'candidate_end_raw_rad': (self.centers[str(mid)]
                                           + math.radians(amplitude * directions[str(mid)]))}
                for mid in range(1, 13)
            ],
        }

    def test_default_keeps_all_four_raw_plus(self):
        directions = builder.directions_for('thigh', 'raw-plus')
        self.assertEqual([directions[str(mid)] for mid in (2, 5, 8, 11)], [1] * 4)
        self.assertEqual(sum(abs(value) for value in directions.values()), 4)
        legacy = self.prepared_candidate('raw-plus')
        del legacy['direction_profile']
        del legacy['raw_direction_by_id']
        self.assertEqual(builder.validate_candidate_directions(legacy, 'thigh',
                                                               self.centers)[0], 'raw-plus')

    def test_prepare_mirrored_candidate_pins_signed_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'source'
            source.mkdir()
            hold_path = base / 'hold.json'
            builder.write_json(source / 'fullbody-review.json',
                               {'boot_id': 'boot', 'motor_uids': {str(i): f'u{i}'
                                                                 for i in range(1, 13)}})
            builder.write_json(hold_path, {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'})
            with patch.object(builder.disabled_base, 'validate_source'), patch.object(
                    builder.disabled_base, 'validate_hold', return_value=self.centers):
                result = builder.prepare(source, hold_path, 'thigh', base / 'prepared',
                                         'mirrored-thigh')
            candidate = builder.read_json(base / 'prepared' / 'offline-raw-step2-candidate.json')
            self.assertEqual(result['direction_profile'], 'mirrored-thigh')
            self.assertEqual([candidate['raw_direction_by_id'][str(mid)]
                              for mid in (2, 5, 8, 11)], [1, -1, 1, -1])
            self.assertEqual([candidate['rows'][mid - 1]['candidate_raw_step_deg']
                              for mid in (2, 5, 8, 11)], [10., -10., 10., -10.])
            builder.validate_candidate_directions(candidate, 'thigh', self.centers)

    def test_continuous_front_hip_candidate_requires_exact_opt_in(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'source'
            source.mkdir()
            hold_path = base / 'hold.json'
            builder.write_json(source / 'fullbody-review.json',
                               {'boot_id': 'boot', 'motor_uids': {str(i): f'u{i}'
                                                                 for i in range(1, 13)}})
            builder.write_json(hold_path, {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'})
            with patch.object(builder.disabled_base, 'validate_source'), patch.object(
                    builder.disabled_base, 'validate_hold', return_value=self.centers):
                builder.prepare(source, hold_path, 'front-hip', base / 'prepared',
                                'front-hip-mirrored', 10.,
                                builder.CONTINUOUS_FRONT_HIP_PROFILE)
                with self.assertRaisesRegex(ValueError, 'Unsupported continuous'):
                    builder.prepare(source, hold_path, 'front-hip', base / 'invalid',
                                    'front-hip-mirrored', 5.,
                                    builder.CONTINUOUS_FRONT_HIP_PROFILE)
            candidate = builder.read_json(base / 'prepared' / 'offline-raw-step2-candidate.json')
            self.assertEqual(candidate['continuous_profile'], builder.CONTINUOUS_FRONT_HIP_PROFILE)
            self.assertEqual(candidate['continuous_waypoints_deg'], [5., 10.])
            builder.validate_candidate_directions(candidate, 'front-hip', self.centers)
            candidate['continuous_waypoints_deg'] = [10., 5.]
            with self.assertRaisesRegex(ValueError, 'Unsupported continuous'):
                builder.validate_candidate_directions(candidate, 'front-hip', self.centers)

    def test_mirrored_profile_rejects_wrong_group_and_changed_sign(self):
        with self.assertRaisesRegex(ValueError, 'only for the four upper-leg'):
            builder.directions_for('toe', 'mirrored-thigh')
        candidate = self.prepared_candidate()
        candidate['raw_direction_by_id']['5'] = 1
        with self.assertRaisesRegex(ValueError, 'raw directions differ'):
            builder.validate_candidate_directions(candidate, 'thigh', self.centers)
        candidate = self.prepared_candidate()
        candidate['rows'][10]['candidate_raw_step_deg'] = 10.
        with self.assertRaisesRegex(ValueError, 'ID11 candidate differs'):
            builder.validate_candidate_directions(candidate, 'thigh', self.centers)

    def test_front_thigh_profile_moves_only_two_toward_face(self):
        profile, group = 'front-thigh-toward-face', 'front-thigh'
        directions = builder.directions_for(group, profile)
        self.assertEqual({mid: value for mid, value in directions.items() if value},
                         {'2': -1, '5': 1})
        candidate = self.prepared_candidate(profile, group)
        self.assertEqual([candidate['rows'][mid - 1]['candidate_raw_step_deg']
                          for mid in (2, 5, 8, 11)], [-10., 10., 0., 0.])
        self.assertEqual(builder.validate_candidate_directions(candidate, group,
                                                               self.centers)[0], profile)
        for wrong_group, wrong_profile in (('thigh', profile),
                                           ('front-thigh', 'raw-plus'),
                                           ('front-thigh', 'mirrored-thigh')):
            with self.subTest(group=wrong_group, profile=wrong_profile), self.assertRaises(ValueError):
                builder.directions_for(wrong_group, wrong_profile)
        for change in ('missing_map', 'reversed_front_sign', 'rear_row_moves'):
            wrong = copy.deepcopy(candidate)
            if change == 'missing_map':
                del wrong['raw_direction_by_id']
            elif change == 'reversed_front_sign':
                wrong['raw_direction_by_id']['2'] = 1
            else:
                wrong['rows'][7]['candidate_raw_step_deg'] = 10.
            with self.subTest(change=change), self.assertRaises(ValueError):
                builder.validate_candidate_directions(wrong, group, self.centers)

    def test_front_hip_profile_moves_only_two_with_exact_mirror_signs(self):
        profile, group = 'front-hip-mirrored', 'front-hip'
        directions = builder.directions_for(group, profile)
        self.assertEqual({mid: value for mid, value in directions.items() if value},
                         {'3': 1, '6': -1})
        candidate = self.prepared_candidate(profile, group)
        self.assertEqual([candidate['rows'][mid - 1]['candidate_raw_step_deg']
                          for mid in (3, 6, 9, 12)], [5., -5., 0., 0.])
        self.assertEqual(builder.validate_candidate_directions(candidate, group,
                                                               self.centers)[0], profile)
        for wrong_group, wrong_profile in (('hip', profile), ('front-hip', 'raw-plus'),
                                           ('front-hip', 'front-thigh-toward-face')):
            with self.subTest(group=wrong_group, profile=wrong_profile), self.assertRaises(ValueError):
                builder.directions_for(wrong_group, wrong_profile)
        for change in ('missing_map', 'reversed_left', 'rear_row_moves', 'over_five'):
            wrong = copy.deepcopy(candidate)
            if change == 'missing_map':
                del wrong['raw_direction_by_id']
            elif change == 'reversed_left':
                wrong['raw_direction_by_id']['6'] = 1
            elif change == 'rear_row_moves':
                wrong['rows'][8]['candidate_raw_step_deg'] = 5.
            else:
                wrong['rows'][2]['candidate_raw_step_deg'] = 5.1
            with self.subTest(change=change), self.assertRaises(ValueError):
                builder.validate_candidate_directions(wrong, group, self.centers)

    def test_front_thigh_disabled_package_pins_two_axis_scope(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source, prepared, output = (base / 'source', base / 'prepared', base / 'disabled')
            source.mkdir()
            (source / 'singularitydog_hw').mkdir()
            (source / 'prepared_fullbody.py').write_text('PINS = {}\n')
            uids = {str(i): f'u{i}' for i in range(1, 13)}
            builder.write_json(source / 'fullbody-review.json',
                               {'boot_id': 'boot', 'motor_uids': uids})
            hold_path = base / 'hold.json'
            builder.write_json(hold_path,
                               {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                                'result': {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'}})
            with patch.object(builder.disabled_base, 'validate_source'), patch.object(
                    builder.disabled_base, 'validate_hold', return_value=self.centers), patch.object(
                    builder.disabled_base, 'adapt_wrapper', side_effect=lambda text, *_: text), patch.object(
                    builder, 'validate_frozen_feedback_age'):
                builder.prepare(source, hold_path, 'front-thigh', prepared,
                                'front-thigh-toward-face')
                builder.disabled(source, hold_path, prepared, output)
            candidate = builder.read_json(prepared / 'offline-raw-step2-candidate.json')
            review = builder.read_json(output / 'step2-review.json')
            self.assertEqual(candidate['moving_motor_ids'], [2, 5])
            self.assertEqual(review['scope'],
                             'supported-front-thigh-two-axis-raw-bounded-other10-hold-50ms-diagnostic')
            self.assertEqual(review['amplitude_deg'], 10.)
            self.assertEqual(review['gain_profile'],
                             'role-front-thigh-kp12-front2-hip-kp12-all4-id4-id10-kp4')
            self.assertEqual(review['raw_direction_by_id'],
                             {str(mid): (-1 if mid == 2 else 1 if mid == 5 else 0)
                              for mid in range(1, 13)})

    def test_front_hip_disabled_package_pins_five_degree_scope(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source, prepared, output = (base / 'source', base / 'prepared', base / 'disabled')
            source.mkdir()
            (source / 'singularitydog_hw').mkdir()
            (source / 'prepared_fullbody.py').write_text('PINS = {}\n')
            uids = {str(i): f'u{i}' for i in range(1, 13)}
            builder.write_json(source / 'fullbody-review.json',
                               {'boot_id': 'boot', 'motor_uids': uids})
            hold_path = base / 'hold.json'
            builder.write_json(hold_path,
                               {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                                'result': {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'}})
            with patch.object(builder.disabled_base, 'validate_source'), patch.object(
                    builder.disabled_base, 'validate_hold', return_value=self.centers), patch.object(
                    builder.disabled_base, 'adapt_wrapper', side_effect=lambda text, *_: text), patch.object(
                    builder, 'validate_frozen_feedback_age'):
                builder.prepare(source, hold_path, 'front-hip', prepared, 'front-hip-mirrored')
                builder.disabled(source, hold_path, prepared, output)
            candidate = builder.read_json(prepared / 'offline-raw-step2-candidate.json')
            review = builder.read_json(output / 'step2-review.json')
            self.assertEqual(candidate['moving_motor_ids'], [3, 6])
            self.assertEqual(review['scope'],
                             'supported-front-hip-two-axis-raw-bounded-other10-hold-50ms-diagnostic')
            self.assertEqual(review['amplitude_deg'], 5.)
            self.assertEqual(review['gain_profile'],
                             'role-front-hip-kp12-front2-id4-id10-kp4')
            self.assertEqual(review['raw_direction_by_id'],
                             {str(mid): (1 if mid == 3 else -1 if mid == 6 else 0)
                              for mid in range(1, 13)})
            self.assertEqual(builder.read_json(output / 'manifest.json')
                             ['singularitydog_hw/rs05_joint_trial.py'],
                             builder.sha(builder.RUNTIME / 'rs05_joint_trial.py'))

    def test_frozen_feedback_guard_requires_pinned_125ms_behavior(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'frozen'
            shutil.copytree(builder.RUNTIME, output / 'singularitydog_hw',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            names = ('singularitydog_hw/rs05_fullbody_step2.py',
                     'singularitydog_hw/rs05_joint_trial.py')
            manifest = {name: builder.sha(output / name) for name in names}
            builder.write_json(output / 'active-manifest.json', manifest)
            builder.validate_frozen_feedback_age(output, 'active-manifest.json')

            guard = output / names[1]
            guard.write_text(guard.read_text().replace(
                'now - received_at <= max_age_s',
                'now - received_at <= MAX_FEEDBACK_AGE_S'))
            manifest[names[1]] = builder.sha(guard)
            builder.write_json(output / 'active-manifest.json', manifest)
            with self.assertRaisesRegex(ValueError, 'Frozen feedback age guard rejected'):
                builder.validate_frozen_feedback_age(output, 'active-manifest.json')

    def test_frozen_front_hip_package_transport_accepts_both_kp12_wires(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'frozen'
            shutil.copytree(builder.RUNTIME, output / 'singularitydog_hw',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            transport = output / 'singularitydog_hw' / 'rs05_bus_transport.py'
            protocol = output / 'singularitydog_hw' / 'rs05_trial_protocol.py'
            runner = output / 'singularitydog_hw' / 'rs05_fullbody_step2.py'
            manifest = {str(path.relative_to(output)): builder.sha(path)
                        for path in (transport, protocol, runner)}
            builder.write_json(output / 'active-manifest.json', manifest)
            builder.validate_frozen_front_hip_transport(output)

            text = transport.read_text()
            old = '                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,\n'
            self.assertIn(old, text)
            transport.write_text(text.replace(old, '', 1))
            manifest[str(transport.relative_to(output))] = builder.sha(transport)
            builder.write_json(output / 'active-manifest.json', manifest)
            with self.assertRaisesRegex(ValueError, 'rejected before UART'):
                builder.validate_frozen_front_hip_transport(output)

    def test_frozen_ten_degree_runner_and_transport_select_interleaved_feedback(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'frozen'
            shutil.copytree(builder.RUNTIME, output / 'singularitydog_hw',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            names = ('rs05_bus_transport.py', 'rs05_trial_protocol.py', 'rs05_fullbody_step2.py')
            manifest = {'singularitydog_hw/'+name: builder.sha(output / 'singularitydog_hw' / name)
                        for name in names}
            builder.write_json(output / 'active-manifest.json', manifest)
            builder.write_json(output / 'step2-active-review.json', {'amplitude_deg': 10.})
            builder.validate_frozen_front_hip_transport(output)
            runner = output / 'singularitydog_hw' / 'rs05_fullbody_step2.py'
            text = runner.read_text()
            old = "and review.get('amplitude_deg') == 10."
            self.assertIn(old, text)
            runner.write_text(text.replace(old, "and review.get('amplitude_deg') == 5.", 1))
            manifest['singularitydog_hw/rs05_fullbody_step2.py'] = builder.sha(runner)
            builder.write_json(output / 'active-manifest.json', manifest)
            with self.assertRaisesRegex(ValueError, 'rejected before UART'):
                builder.validate_frozen_front_hip_transport(output)

    def test_disabled_package_pins_mirrored_review(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source, prepared, output = (base / 'source', base / 'prepared', base / 'disabled')
            source.mkdir()
            (source / 'singularitydog_hw').mkdir()
            (source / 'prepared_fullbody.py').write_text('PINS = {}\n')
            uids = {str(i): f'u{i}' for i in range(1, 13)}
            builder.write_json(source / 'fullbody-review.json',
                               {'boot_id': 'boot', 'motor_uids': uids})
            hold_path = base / 'hold.json'
            builder.write_json(hold_path,
                               {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                                'result': {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'}})
            with patch.object(builder.disabled_base, 'validate_source'), patch.object(
                    builder.disabled_base, 'validate_hold', return_value=self.centers), patch.object(
                    builder.disabled_base, 'adapt_wrapper', side_effect=lambda text, *_: text), patch.object(
                    builder, 'validate_frozen_feedback_age'):
                builder.prepare(source, hold_path, 'thigh', prepared, 'mirrored-thigh')
                result = builder.disabled(source, hold_path, prepared, output)
            review = builder.read_json(output / 'step2-review.json')
            self.assertEqual(result['direction_profile'], 'mirrored-thigh')
            self.assertEqual([review['raw_direction_by_id'][str(i)]
                              for i in (2, 5, 8, 11)], [1, -1, 1, -1])
            self.assertEqual(builder.read_json(output / 'manifest.json')
                             ['step2-review.json'], builder.sha(output / 'step2-review.json'))

    def test_active_requires_exact_signed_physical_review(self):
        candidate = self.prepared_candidate()
        directions = candidate['raw_direction_by_id']
        review = {'role_group': 'thigh', 'direction_profile': 'mirrored-thigh',
                  'raw_direction_by_id': directions}
        physical = {'direction_profile': 'mirrored-thigh',
                    'raw_direction_by_id': copy.deepcopy(directions),
                    'scope': 'supported-four-thigh-current-raw-mirrored-10deg-diagnostic-only'}
        self.assertEqual(builder.validate_active_directions(review, candidate, physical,
                                                             self.centers), 'mirrored-thigh')
        physical['raw_direction_by_id']['11'] = 1
        with self.assertRaisesRegex(ValueError, 'exact raw directions'):
            builder.validate_active_directions(review, candidate, physical, self.centers)
        physical['raw_direction_by_id']['11'] = -1
        physical['scope'] = 'supported-four-thigh-current-raw-plus-10deg-diagnostic-only'
        with self.assertRaisesRegex(ValueError, 'exact raw directions'):
            builder.validate_active_directions(review, candidate, physical, self.centers)

    def test_front_thigh_active_requires_its_exact_physical_scope(self):
        candidate = self.prepared_candidate('front-thigh-toward-face', 'front-thigh')
        directions = candidate['raw_direction_by_id']
        review = {'role_group': 'front-thigh', 'direction_profile': 'front-thigh-toward-face',
                  'raw_direction_by_id': directions}
        physical = {'direction_profile': 'front-thigh-toward-face',
                    'raw_direction_by_id': copy.deepcopy(directions),
                    'scope': 'supported-front-thigh-current-raw-toward-face-10deg-diagnostic-only'}
        self.assertEqual(builder.validate_active_directions(review, candidate, physical,
                                                             self.centers), 'front-thigh-toward-face')
        for change in ('old_scope', 'wrong_sign', 'wrong_rear'):
            wrong = copy.deepcopy(physical)
            if change == 'old_scope':
                wrong['scope'] = 'supported-four-thigh-current-raw-mirrored-10deg-diagnostic-only'
            elif change == 'wrong_sign':
                wrong['raw_direction_by_id']['2'] = 1
            else:
                wrong['raw_direction_by_id']['8'] = 1
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'exact raw directions'):
                builder.validate_active_directions(review, candidate, wrong, self.centers)

    def test_front_hip_active_requires_its_exact_physical_scope(self):
        candidate = self.prepared_candidate('front-hip-mirrored', 'front-hip')
        directions = candidate['raw_direction_by_id']
        review = {'role_group': 'front-hip', 'direction_profile': 'front-hip-mirrored',
                  'raw_direction_by_id': directions}
        physical = {'direction_profile': 'front-hip-mirrored',
                    'raw_direction_by_id': copy.deepcopy(directions),
                    'scope': 'supported-front-hip-current-raw-mirrored-5deg-diagnostic-only'}
        self.assertEqual(builder.validate_active_directions(review, candidate, physical,
                                                             self.centers), 'front-hip-mirrored')
        for change in ('old_scope', 'wrong_sign', 'wrong_rear'):
            wrong = copy.deepcopy(physical)
            if change == 'old_scope':
                wrong['scope'] = 'supported-front-thigh-current-raw-toward-face-10deg-diagnostic-only'
            elif change == 'wrong_sign':
                wrong['raw_direction_by_id']['6'] = 1
            else:
                wrong['raw_direction_by_id']['9'] = 1
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'exact raw directions'):
                builder.validate_active_directions(review, candidate, wrong, self.centers)

    def test_continuous_candidate_cannot_be_downgraded_by_dropping_review_profile(self):
        candidate = self.prepared_candidate('front-hip-mirrored', 'front-hip')
        candidate['amplitude_deg'] = 10.
        candidate['continuous_profile'] = builder.CONTINUOUS_FRONT_HIP_PROFILE
        candidate['continuous_waypoints_deg'] = [5., 10.]
        for row in candidate['rows']:
            mid = row['id']
            step = 10. * candidate['raw_direction_by_id'][str(mid)]
            row['candidate_raw_step_deg'] = step
            row['candidate_end_raw_rad'] = self.centers[str(mid)] + math.radians(step)
        review = {'role_group': 'front-hip', 'direction_profile': 'front-hip-mirrored',
                  'raw_direction_by_id': candidate['raw_direction_by_id'], 'amplitude_deg': 10.}
        physical = {'direction_profile': 'front-hip-mirrored',
                    'raw_direction_by_id': copy.deepcopy(candidate['raw_direction_by_id']),
                    'scope': 'supported-front-hip-current-raw-mirrored-10deg-diagnostic-only'}
        with self.assertRaisesRegex(ValueError, 'Continuous front-hip profile differs'):
            builder.validate_active_directions(review, candidate, physical, self.centers)

    def test_preflight_must_belong_to_exact_disabled_role_package(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            disabled = base / 'disabled'
            disabled.mkdir()
            wrapper = disabled / 'prepared_fullbody.py'
            wrapper.write_text('PINS = {}\n')
            review = {'role_group': 'front-thigh', 'gain_profile': 'front-profile'}
            builder.write_json(disabled / 'step2-review.json', review)
            summary_path, events_path = base / 'summary.json', base / 'events.jsonl'
            events_path.write_text('{}\n')
            summary = {'wrapper_sha256': builder.sha(wrapper),
                       'result': {'review': review, 'moving_motor_ids': [2, 5],
                                  'gain_profile': 'front-profile', 'raw_diagnostic_only': True}}
            with patch.object(builder.active_base, 'validate_step2_preflight'):
                builder.write_json(summary_path, summary)
                builder.validate_role_group_preflight(disabled, summary_path, events_path, 'boot')
                for change in ('old_wrapper', 'old_review', 'old_group'):
                    wrong = copy.deepcopy(summary)
                    if change == 'old_wrapper':
                        wrong['wrapper_sha256'] = 'a' * 64
                    elif change == 'old_review':
                        wrong['result']['review'] = {'role_group': 'thigh'}
                    else:
                        wrong['result']['moving_motor_ids'] = [2, 5, 8, 11]
                    builder.write_json(summary_path, wrong)
                    with self.subTest(change=change), self.assertRaisesRegex(ValueError,
                                                                             'exact role-group'):
                        builder.validate_role_group_preflight(disabled, summary_path,
                                                              events_path, 'boot')

    def test_front_hip_ten_degree_clearance_binds_all_preflight_centers(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary = Path(tmp) / 'summary.json'
            builder.write_json(summary, {'result': {'workers': {
                'front': {'centers': {str(i): self.centers[str(i)] for i in range(1, 7)}},
                'rear': {'centers': {str(i): self.centers[str(i)] for i in range(7, 13)}}}}})
            physical = {
                'clearance_reference_raw_rad_by_id': dict(self.centers),
                'clearance_reference_preflight_summary_sha256': builder.sha(summary),
                'start_tolerance_clearance_verified_deg': 3.,
                'start_tolerance_clearance_note': 'All twelve joints clear through the 10° sweep '
                                                  'from every start within ±3° of this pose.',
            }
            review = {'role_group': 'front-hip', 'amplitude_deg': 10.}
            extras = builder.front_hip_step10_clearance_extras(review, physical, summary)
            self.assertEqual(extras['clearance_reference_raw_rad_by_id'], self.centers)
            self.assertEqual(extras['start_tolerance_clearance_verified_deg'], 3.)
            self.assertEqual(builder.front_hip_step10_clearance_extras(
                {'role_group': 'front-hip', 'amplitude_deg': 5.}, {}, summary), {})
            for change in ('held_axis', 'moving_axis', 'summary_hash', 'margin', 'note'):
                wrong = copy.deepcopy(physical)
                if change == 'held_axis':
                    wrong['clearance_reference_raw_rad_by_id']['8'] += .1
                elif change == 'moving_axis':
                    wrong['clearance_reference_raw_rad_by_id']['3'] += .1
                elif change == 'summary_hash':
                    wrong['clearance_reference_preflight_summary_sha256'] = 'a' * 64
                elif change == 'margin':
                    wrong['start_tolerance_clearance_verified_deg'] = 5.
                else:
                    wrong['start_tolerance_clearance_note'] = ' '
                with self.subTest(change=change), self.assertRaisesRegex(
                        ValueError, 'physical review must cover'):
                    builder.front_hip_step10_clearance_extras(review, wrong, summary)
    def test_audio_gate_keeps_nested_active_runner_valid(self):
        wrapper = ("def run():\n"
                   "    from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2\n"
                   "    modules = ('rs05_step2_packet_gate',)\n"
                   "    try:\n"
                   "        report['result'] = run_fullbody_step2(transports, expected, check, emit,\n"
                   "            validated_review=review)\n"
                   "    finally:\n"
                   "        close()\n")
        gated = builder.attach_announcement(wrapper)
        self.assertEqual(gated.count("play_test_start(BASE / 'test-start-ja.wav')"), 1)
        self.assertLess(gated.index('play_test_start'), gated.index("report['result']"))
        self.assertLess(gated.index('from singularitydog_hw.i2s_announcement'),
                        gated.index('modules ='))


if __name__ == '__main__':
    unittest.main()
