"""Offline checks for narrowly reviewed current-reference holds; no hardware I/O."""
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import replace
import hashlib
import inspect
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.current_hold_review import PROFILE, load_current_hold_review
from singularitydog_hw import rs05_leg_trial as trial
from test_rs05_bounded_pose_trial import PoseTransport
from test_rs05_leg_settled import CENTERS, samples


BOOT_ID = '01234567-89ab-cdef-0123-456789abcdef'
REVIEWED = {'FL': 6, 'RR': 8}


def review_data(leg='FL', *, motor_id=None):
    mid = REVIEWED[leg] if motor_id is None else motor_id
    return {'schema': 'rs05-current-hold-review-v1',
            'scope': 'supported-current-reference-hold-5s',
            'reviewed_motor_id': mid, 'motor_uid_hex': f'{mid:016x}',
            'boot_id': BOOT_ID, 'firmware': '0.5.0.13',
            'physical_large_motion_confirmed': True, 'review_complete': True,
            'calibration_verified': False, 'learned_policy_allowed': False,
            'metrics': {'position_span_deg': 2., 'position_velocity_correlation': .95,
                        'position_unique_values': 20},
            'source_sha256': {'events.jsonl': 'a' * 64, 'summary.json': 'b' * 64,
                              'audit.json': 'c' * 64}}


def review_set_data():
    return {'schema': 'rs05-current-hold-review-set-v1',
            'scope': 'supported-current-reference-hold-5s',
            'reviews': {str(mid): review_data(motor_id=mid) for mid in (5, 6)}}


def invalid_review_sets():
    """Each case invalidates one axis; a good peer must never mask its failure."""
    for ids in ((), (5,), (6,), (4, 5, 6), (5, 6, 7), (7, 8), (8,)):
        data = review_set_data()
        data['reviews'] = {str(mid): review_data(motor_id=mid) for mid in ids}
        yield f'wrong_members_{ids}', data
    for key, value in (('schema', 'rs05-current-hold-review-set-v2'),
                       ('scope', 'all-legs'), ('reviews', []), ('reviews', None)):
        data = review_set_data(); data[key] = value
        yield f'outer_{key}_{value}', data
    for key in ('schema', 'scope', 'reviews'):
        data = review_set_data(); del data[key]
        yield f'missing_outer_{key}', data
    data = review_set_data(); data['reviews']['05'] = data['reviews'].pop('5')
    yield 'noncanonical_member_id', data
    for mid in (5, 6):
        for key in review_data(motor_id=mid):
            data = review_set_data(); del data['reviews'][str(mid)][key]
            yield f'ID{mid}_missing_{key}', data
        changes = (('schema', 'rs05-current-hold-review-set-v1'), ('scope', 'all-legs'),
                   ('reviewed_motor_id', 6 if mid == 5 else 5),
                   ('reviewed_motor_id', str(mid)), ('reviewed_motor_id', float(mid)),
                   ('motor_uid_hex', f'{6 if mid == 5 else 5:016x}'),
                   ('boot_id', 'previous-boot'), ('firmware', '0.5.0.12'),
                   ('review_complete', False), ('review_complete', 1),
                   ('physical_large_motion_confirmed', False),
                   ('physical_large_motion_confirmed', 1),
                   ('calibration_verified', True), ('calibration_verified', 0),
                   ('learned_policy_allowed', True), ('learned_policy_allowed', 0),
                   ('metrics', None), ('source_sha256', []))
        for key, value in changes:
            data = review_set_data(); data['reviews'][str(mid)][key] = value
            yield f'ID{mid}_{key}_{value!r}', data
        for key, value in (('position_span_deg', .99), ('position_span_deg', True),
                           ('position_span_deg', float('nan')),
                           ('position_velocity_correlation', .89),
                           ('position_velocity_correlation', 1.01),
                           ('position_velocity_correlation', float('inf')),
                           ('position_unique_values', 9), ('position_unique_values', 10.),
                           ('position_unique_values', True)):
            data = review_set_data(); data['reviews'][str(mid)]['metrics'][key] = value
            yield f'ID{mid}_metric_{key}_{value!r}', data
        for filename in ('events.jsonl', 'summary.json', 'audit.json'):
            data = review_set_data(); del data['reviews'][str(mid)]['source_sha256'][filename]
            yield f'ID{mid}_missing_source_{filename}', data
            data = review_set_data(); data['reviews'][str(mid)]['source_sha256'][filename] = 'z' * 64
            yield f'ID{mid}_bad_source_{filename}', data
        data = review_set_data(); data['reviews'][str(mid)]['source_sha256']['extra.json'] = 'd' * 64
        yield f'ID{mid}_extra_source', data


@contextmanager
def review_file(leg='FL', data=None, *, raw=None):
    original_read = Path.read_text

    def read_text(path, *args, **kwargs):
        if str(path) == '/proc/sys/kernel/random/boot_id':
            return BOOT_ID + '\n'
        return original_read(path, *args, **kwargs)

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'review.json'
        path.write_text(json.dumps(review_data(leg) if data is None else data) if raw is None else raw)
        pin = hashlib.sha256(path.read_bytes()).hexdigest()
        with patch.object(Path, 'read_text', autospec=True, side_effect=read_text):
            yield path, pin


def load(path, pin, leg='FL', **overrides):
    ids = trial.LEGS[leg]
    centers = dict(zip(ids, CENTERS.values()))
    arguments = dict(ids=ids, absolute_targets=centers,
                     matched_start_positions=dict(centers),
                     matched_start_tolerance_rad=math.radians(.5),
                     gain_profile='kp3', observation_profile=None, expected_sha256=pin)
    expected = overrides.pop('expected_uids', {i: f'{i:016x}' for i in ids})
    arguments.update(overrides)
    return load_current_hold_review(path, expected, **arguments)


class ReviewValidationTests(unittest.TestCase):
    def test_only_explicit_fl_and_rr_current_boot_reviews_are_accepted(self):
        for leg in REVIEWED:
            with self.subTest(leg=leg), review_file(leg) as (path, pin):
                result = load(path, pin, leg)
                self.assertEqual(result['sha256'], pin)
                self.assertEqual(result['reviewed_motor_id'], REVIEWED[leg])
                self.assertEqual(result['reviewed_motor_ids'], [REVIEWED[leg]])
                self.assertFalse(result['calibration_verified'])
                self.assertFalse(result['learned_policy_allowed'])
                other = 'RR' if leg == 'FL' else 'FL'
                with self.assertRaises(ValueError):
                    load(path, pin, other)
        for leg in ('FR', 'RL'):
            with self.subTest(leg=leg), review_file() as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin, leg)

    def test_record_identity_boot_firmware_and_review_flags_fail_closed(self):
        changes = {'schema': 'other', 'scope': 'all-legs', 'reviewed_motor_id': 5,
                   'motor_uid_hex': 'f' * 16, 'boot_id': 'another-boot',
                   'firmware': '0.5.0.12', 'physical_large_motion_confirmed': False,
                   'review_complete': False, 'calibration_verified': True,
                   'learned_policy_allowed': True}
        for key, value in changes.items():
            data = review_data(); data[key] = value
            with self.subTest(key=key), review_file(data=data) as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin)
        for expected in ({4: '4' * 16, 5: '5' * 16},
                         {4: '4' * 16, 5: '5' * 16, 6: 'Z' * 16},
                         {4: f'{6:016x}', 5: f'{5:016x}', 6: f'{6:016x}'},
                         {4: f'{4:016x}', 5: f'{5:016x}', 6: f'{6:016x}', '6': f'{6:016x}'}):
            with self.subTest(expected=expected), review_file() as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin, expected_uids=expected)

    def test_missing_boot_file_fails_closed(self):
        with review_file() as (path, pin):
            with patch.object(Path, 'read_text', side_effect=OSError('boot unavailable')):
                with self.assertRaises((ValueError, OSError)):
                    load(path, pin)

    def test_hash_pin_is_mandatory_and_covers_exact_review_bytes(self):
        with review_file() as (path, pin):
            for bad_pin in (None, '', 'z' * 64, '0' * 64):
                with self.subTest(pin=bad_pin), self.assertRaises(ValueError):
                    load(path, bad_pin)
            path.write_bytes(path.read_bytes() + b'\n')
            with self.assertRaises(ValueError):
                load(path, pin)
        for hashes in ({}, {'events.jsonl': 'a' * 64, 'summary.json': 'b' * 64},
                       {**review_data()['source_sha256'], 'audit.json': 'z' * 64}):
            data = review_data(); data['source_sha256'] = hashes
            with review_file(data=data) as (path, pin), self.assertRaises(ValueError):
                load(path, pin)

    def test_insufficient_nonnumeric_and_nonfinite_metrics_fail_closed(self):
        failures = (('position_span_deg', .99), ('position_span_deg', True),
                    ('position_span_deg', float('nan')),
                    ('position_velocity_correlation', .89),
                    ('position_velocity_correlation', float('inf')),
                    ('position_unique_values', 9), ('position_unique_values', 10.),
                    ('position_unique_values', True))
        for key, value in failures:
            data = review_data(); data['metrics'][key] = value
            with self.subTest(key=key, value=value), review_file(data=data) as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin)

    def test_only_exact_reference_kp3_and_at_most_half_degree_match(self):
        centers = dict(zip(trial.LEGS['FL'], CENTERS.values()))
        failures = ({'absolute_targets': None}, {'matched_start_positions': None},
                    {'absolute_targets': {**centers, 6: centers[6] + .00001}},
                    {'matched_start_tolerance_rad': math.radians(.5001)},
                    {'matched_start_tolerance_rad': float('nan')},
                    {'gain_profile': 'kp4_diagnostic'},
                    {'gain_profile': 'rr_hip_kp6_diagnostic'},
                    {'observation_profile': 'rr_settling_1s'})
        for override in failures:
            with self.subTest(override=override), review_file() as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin, **override)


class ReviewSetValidationTests(unittest.TestCase):
    def test_complete_fl_set_returns_both_axes_and_cannot_be_used_for_other_legs(self):
        for order in ((5, 6), (6, 5)):
            data = review_set_data()
            data['reviews'] = {str(mid): data['reviews'][str(mid)] for mid in order}
            with self.subTest(order=order), review_file(data=data) as (path, pin):
                result = load(path, pin)
                self.assertEqual(result['sha256'], pin)
                self.assertEqual(result['reviewed_motor_ids'], [5, 6])
                self.assertNotIn('reviewed_motor_id', result)
                self.assertTrue(result['manual_response_reviewed'])
                self.assertFalse(result['calibration_verified'])
                self.assertFalse(result['learned_policy_allowed'])
                self.assertFalse(result['physical_angle_accuracy_verified'])
                for leg in ('FR', 'RR', 'RL'):
                    with self.subTest(leg=leg), self.assertRaises(ValueError):
                        load(path, pin, leg)

    def test_invalid_members_or_either_incomplete_review_rejects_entire_set(self):
        for name, data in invalid_review_sets():
            with self.subTest(case=name), review_file(data=data) as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin)

    def test_duplicate_json_members_are_rejected_at_every_level(self):
        data = review_set_data()
        raw = json.dumps(data)
        duplicates = (
            raw.replace('"reviews": {', '"reviews": {"5": ' + json.dumps(data['reviews']['5']) + ', ', 1),
            raw.replace('"reviewed_motor_id": 5', '"reviewed_motor_id": 5, "reviewed_motor_id": 5', 1),
            raw.replace('"position_span_deg": 2.0', '"position_span_deg": 2.0, "position_span_deg": 2.0', 1),
        )
        for raw in duplicates:
            with self.subTest(raw=raw), review_file(raw=raw) as (path, pin):
                with self.assertRaises(ValueError):
                    load(path, pin)

    def test_bundle_pin_covers_each_nested_review_and_serialization(self):
        for mid in (5, 6):
            data = review_set_data()
            with self.subTest(mid=mid), review_file(data=data) as (path, pin):
                data['reviews'][str(mid)]['source_sha256']['events.jsonl'] = 'd' * 64
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load(path, pin)
        with review_file(data=review_set_data()) as (path, pin):
            path.write_bytes(path.read_bytes() + b'\n')
            with self.assertRaises(ValueError):
                load(path, pin)


class CompositeWindowTests(unittest.TestCase):
    def evaluate(self, leg, rows, **kwargs):
        ids = trial.LEGS[leg]
        return trial.evaluate_settled_window(dict(zip(ids, rows.values())),
            dict(zip(ids, CENTERS.values())), profile=PROFILE, **kwargs)

    def test_only_reviewed_motor_may_use_position_gate_over_legacy_rms(self):
        noisy = samples(velocity=lambda t: .06 if round(t * 10) % 2 else -.06)
        for leg, reviewed in REVIEWED.items():
            for mid in trial.LEGS[leg]:
                rows = samples()
                source_id = trial.LEGS[leg].index(mid) + 1
                rows[source_id] = deepcopy(noisy[source_id])
                with self.subTest(leg=leg, mid=mid):
                    result = self.evaluate(leg, rows)
                    self.assertEqual(result['passed'], mid == reviewed)
                    self.assertFalse(result['absolute_rest_proven'])
                    self.assertFalse(result['joint_calibration_verified'])
                # The other two retain the legacy position limits as well as RMS.
                rows = samples()
                rows[source_id][10]['feedback']['protocol_position_rad'] += .0015
                with self.subTest(leg=leg, mid=mid, signal='position'):
                    self.assertEqual(self.evaluate(leg, rows)['passed'], mid != reviewed)

    def test_review_does_not_override_mean_drift_fault_stale_or_speed_guards(self):
        for leg, reviewed in REVIEWED.items():
            index = trial.LEGS[leg].index(reviewed) + 1
            for failure in ('mean', 'drift', 'fault', 'stale', 'instant', 'nonfinite'):
                rows = samples(); selected = rows[index]
                if failure == 'mean':
                    for row in selected: row['feedback']['velocity_rad_s'] = .06
                if failure == 'drift':
                    for n, row in enumerate(selected):
                        row['feedback']['protocol_position_rad'] += n * .00006
                if failure == 'fault': selected[10]['feedback']['fault_bits'] = 1
                if failure == 'stale': selected[10]['checked_monotonic_s'] += .11
                if failure == 'instant': selected[10]['feedback']['velocity_rad_s'] = .51
                if failure == 'nonfinite': selected[10]['feedback']['velocity_rad_s'] = float('nan')
                with self.subTest(leg=leg, failure=failure):
                    self.assertFalse(self.evaluate(leg, rows)['passed'])
        for leg in ('FR', 'RL'):
            with self.subTest(leg=leg), self.assertRaises(ValueError):
                self.evaluate(leg, samples())

    def test_fl_set_uses_position_only_for_both_reviewed_axes(self):
        noisy = samples(velocity=lambda t: .06 if round(t * 10) % 2 else -.06)
        for noisy_ids in ((5,), (6,), (5, 6), (4,), (4, 5, 6)):
            rows = samples()
            for mid in noisy_ids:
                rows[mid - 3] = deepcopy(noisy[mid - 3])
            with self.subTest(noisy_ids=noisy_ids):
                result = self.evaluate('FL', rows, reviewed_motor_ids=[5, 6])
                self.assertEqual(result['passed'], 4 not in noisy_ids, result['errors'])
                self.assertEqual(result['reviewed_motor_ids'], [5, 6])
                self.assertEqual(result['effective_profile_by_motor'],
                                 {4: 'legacy-rms-v1', 5: 'position-v2', 6: 'position-v2'})
                self.assertFalse(result['absolute_rest_proven'])
                self.assertFalse(result['joint_calibration_verified'])
        rows = samples(); rows[1][10]['feedback']['protocol_position_rad'] += .0015
        self.assertTrue(self.evaluate('FL', rows, reviewed_motor_ids=(5, 6))['passed'])
        rows[1][10]['feedback']['protocol_position_rad'] += .001
        self.assertFalse(self.evaluate('FL', rows, reviewed_motor_ids=(5, 6))['passed'])

    def test_explicit_singletons_and_invalid_selections_cannot_expand_review_scope(self):
        rows = samples(velocity=lambda t: .06 if round(t * 10) % 2 else -.06)
        only_id5_noisy = samples(); only_id5_noisy[2] = rows[2]
        for reviewed in (None, (6,), [6]):
            with self.subTest(reviewed=reviewed):
                self.assertFalse(self.evaluate('FL', only_id5_noisy,
                                               reviewed_motor_ids=reviewed)['passed'])
        for leg, reviewed in REVIEWED.items():
            with self.subTest(leg=leg):
                result = self.evaluate(leg, samples(), reviewed_motor_ids=[reviewed])
                self.assertTrue(result['passed'])
                self.assertEqual(result['reviewed_motor_id'], reviewed)
        invalid = ([], [5], [4, 5, 6], [5, 6, 6], [6, 5], [5, 6, 7],
                   ['5', '6'], [5., 6.], {5, 6}, {'5': True, '6': True}, '5,6', True)
        for reviewed in invalid:
            with self.subTest(reviewed=reviewed), self.assertRaises(ValueError):
                self.evaluate('FL', samples(), reviewed_motor_ids=reviewed)
        for leg in ('FR', 'RR', 'RL'):
            with self.subTest(leg=leg), self.assertRaises(ValueError):
                self.evaluate(leg, samples(), reviewed_motor_ids=[5, 6])

    def test_each_set_axis_retains_fixed_samples_mean_position_tail_and_feedback_guards(self):
        for mid in (5, 6):
            for failure in ('mean', 'drift', 'span', 'tail', 'fault', 'stale', 'instant',
                            'nonfinite_velocity', 'nonfinite_position', 'missing', 'extra', 'gap'):
                rows = samples(); selected = rows[mid - 3]
                if failure == 'mean':
                    for row in selected: row['feedback']['velocity_rad_s'] = .026
                if failure == 'drift':
                    for n, row in enumerate(selected):
                        row['feedback']['protocol_position_rad'] += n * .00006
                if failure == 'span':
                    selected[10]['feedback']['protocol_position_rad'] += .0011
                if failure == 'tail':
                    selected[-1]['feedback']['protocol_position_rad'] += .0009
                if failure == 'fault': selected[10]['feedback']['fault_bits'] = 1
                if failure == 'stale': selected[10]['checked_monotonic_s'] += .11
                if failure == 'instant': selected[10]['feedback']['velocity_rad_s'] = .51
                if failure == 'nonfinite_velocity': selected[10]['feedback']['velocity_rad_s'] = float('nan')
                if failure == 'nonfinite_position': selected[10]['feedback']['protocol_position_rad'] = float('inf')
                if failure == 'missing': selected.pop(10)
                if failure == 'extra': selected.append(deepcopy(selected[-1]))
                if failure == 'gap': selected[10]['received_monotonic_s'] += .06
                with self.subTest(mid=mid, failure=failure):
                    result = self.evaluate('FL', rows, reviewed_motor_ids=[5, 6])
                    self.assertFalse(result['passed'])
                    self.assertTrue(result['motors'][mid]['errors'])


class CurrentHoldRunnerTests(unittest.TestCase):
    def test_finite_fake_hold_accepts_only_reviewed_axis_noise_and_always_stops(self):
        self.assertNotIn(PROFILE, trial.PROFILES)
        for leg, reviewed in REVIEWED.items():
            for noisy_mid in trial.LEGS[leg]:
                t = PoseTransport(leg)

                def change(transport, found):
                    fb, timestamp = found[noisy_mid]
                    velocity = .06 if transport.window_count % 2 else -.06
                    found[noisy_mid] = replace(fb, velocity_rad_s=velocity), timestamp
                    return found

                t.change = change
                with self.subTest(leg=leg, noisy_mid=noisy_mid), review_file(leg) as (path, pin):
                    result = trial.run_bounded_pose_trial(t,
                        {i: f'{i:016x}' for i in t.ids}, lambda: None, lambda _: None,
                        absolute_targets=dict(t.centers), matched_start_positions=dict(t.centers),
                        profile=PROFILE, position_response_evidence=path,
                        position_response_evidence_sha256=pin, clock=t.clock, wait=t.clock.wait)
                accepted = noisy_mid == reviewed
                self.assertEqual(result['motion_completed'], accepted, result['errors'])
                self.assertEqual(any(f.kind == 3 for _, f in t.frames), accepted)
                self.assertTrue(result['stop_confirmed'])
                self.assertEqual(t.stop_calls[-1], t.ids)
                self.assertEqual(t.window_count, 21)
                self.assertFalse(t.enabled)
                if accepted:
                    self.assertTrue(5. <= t.clock() - t.enable_time < 5.1)
                    for mid in t.ids:
                        self.assertAlmostEqual(t.commanded[mid], t.centers[mid], delta=25.14 / 65535)

    def test_jog_gain_offset_and_observation_reject_before_any_transport_io(self):
        for leg in REVIEWED:
            for failure in ('jog', 'gain', 'offset', 'observation', 'no_pin', 'no_review'):
                t = PoseTransport(leg)
                expected = {i: f'{i:016x}' for i in t.ids}
                with self.subTest(leg=leg, failure=failure), review_file(leg) as (path, pin):
                    arguments = dict(profile=PROFILE, position_response_evidence=path,
                                     position_response_evidence_sha256=pin)
                    with self.assertRaises(ValueError):
                        if failure == 'jog':
                            trial.run_leg_trial(t, expected, lambda: None, lambda _: None,
                                                directions=(1, 1, 1), **arguments)
                        else:
                            targets = dict(t.centers)
                            if failure == 'offset': targets[t.ids[0]] += .00001
                            if failure == 'gain': arguments['gain_profile'] = 'kp4_diagnostic'
                            if failure == 'observation': arguments['observation_profile'] = 'rr_settling_1s'
                            if failure == 'no_pin': arguments['position_response_evidence_sha256'] = None
                            if failure == 'no_review': arguments['position_response_evidence'] = None
                            trial.run_bounded_pose_trial(t, expected, lambda: None, lambda _: None,
                                absolute_targets=targets, matched_start_positions=dict(t.centers),
                                clock=t.clock, wait=t.clock.wait, **arguments)
                self.assertFalse(t.calls or t.frames or t.stop_calls)

    def test_review_set_runs_one_finite_fixed_hold_with_all_axes_stopped(self):
        for noisy_ids in ((5,), (6,), (5, 6), (4,), (4, 5, 6)):
            t = PoseTransport('FL')

            def change(transport, found):
                for mid in noisy_ids:
                    fb, timestamp = found[mid]
                    velocity = .06 if transport.window_count % 2 else -.06
                    found[mid] = replace(fb, velocity_rad_s=velocity), timestamp
                return found

            t.change = change
            with self.subTest(noisy_ids=noisy_ids), review_file(data=review_set_data()) as (path, pin):
                result = trial.run_bounded_pose_trial(t,
                    {i: f'{i:016x}' for i in t.ids}, lambda: None, lambda _: None,
                    absolute_targets=dict(t.centers), matched_start_positions=dict(t.centers),
                    profile=PROFILE, position_response_evidence=path,
                    position_response_evidence_sha256=pin, clock=t.clock, wait=t.clock.wait)
            accepted = 4 not in noisy_ids
            self.assertEqual(result['motion_completed'], accepted, result['errors'])
            self.assertEqual(sum(f.kind == 3 for _, f in t.frames), 3 if accepted else 0)
            self.assertEqual(result['settled_window']['reviewed_motor_ids'], [5, 6])
            self.assertEqual(t.window_count, 21)
            self.assertTrue(result['stop_confirmed'])
            self.assertEqual(t.stop_calls[-1], (4, 5, 6))
            self.assertFalse(t.enabled)
            if accepted:
                self.assertTrue(5. <= t.clock() - t.enable_time < 5.1)
                for mid in t.ids:
                    self.assertAlmostEqual(t.commanded[mid], t.centers[mid], delta=25.14 / 65535)

    def test_every_invalid_bundle_rejects_before_transport_io(self):
        for name, data in invalid_review_sets():
            t = PoseTransport('FL')
            with self.subTest(case=name), review_file(data=data) as (path, pin):
                with self.assertRaises(ValueError):
                    trial.run_bounded_pose_trial(t,
                        {i: f'{i:016x}' for i in t.ids}, lambda: None, lambda _: None,
                        absolute_targets=dict(t.centers), matched_start_positions=dict(t.centers),
                        profile=PROFILE, position_response_evidence=path,
                        position_response_evidence_sha256=pin, clock=t.clock, wait=t.clock.wait)
            self.assertFalse(t.calls or t.frames or t.stop_calls)

    def test_runner_has_no_reviewed_motor_ids_override(self):
        for runner in (trial.run_leg_trial, trial.run_bounded_pose_trial):
            self.assertNotIn('reviewed_motor_ids', inspect.signature(runner).parameters)

    def test_mutating_loader_report_during_io_cannot_broaden_captured_review_ids(self):
        t = PoseTransport('FL')
        validated = {}
        original_load = trial.load_current_hold_review
        original_parameter = t.parameter

        def capture_review(*args, **kwargs):
            validated['review'] = original_load(*args, **kwargs)
            return validated['review']

        def mutate_report(*args, **kwargs):
            validated['review']['reviewed_motor_ids'][:] = [5, 6]
            return original_parameter(*args, **kwargs)

        def change(transport, found):
            fb, timestamp = found[5]
            velocity = .06 if transport.window_count % 2 else -.06
            found[5] = replace(fb, velocity_rad_s=velocity), timestamp
            return found

        t.parameter, t.change = mutate_report, change
        with review_file() as (path, pin), patch.object(trial, 'load_current_hold_review',
                                                       side_effect=capture_review):
            result = trial.run_bounded_pose_trial(t,
                {i: f'{i:016x}' for i in t.ids}, lambda: None, lambda _: None,
                absolute_targets=dict(t.centers), matched_start_positions=dict(t.centers),
                profile=PROFILE, position_response_evidence=path,
                position_response_evidence_sha256=pin, clock=t.clock, wait=t.clock.wait)
        self.assertEqual(validated['review']['reviewed_motor_ids'], [5, 6])
        self.assertEqual(result['settled_window']['reviewed_motor_ids'], [6])
        self.assertFalse(result['motion_completed'])
        self.assertFalse(any(f.kind == 3 for _, f in t.frames))
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(t.stop_calls[-1], (4, 5, 6))


if __name__ == '__main__':
    unittest.main()
