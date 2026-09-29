"""Changed-pose preload calculations must remain file-only and unapproved."""

import hashlib
import json
import math
import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools import plan_supported_preload as planner


class SupportedPreloadPlanTest(unittest.TestCase):
    def test_tilted_up_moves_foot_along_world_down_in_body_frame(self):
        target, up = planner.foot_down_target((.1, .2, -.3), 1., (0., 3., 4.))
        self.assertEqual(up, (0., .6, .8))
        for actual, expected in zip(target, (.1, .1994, -.3008)):
            self.assertAlmostEqual(actual, expected)

    def test_invalid_up_direction_is_rejected(self):
        for vector in ((0., 0., 0.), (1., 2.), (0., float('nan'), 1.),
                       (float('inf'), 0., 1.)):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                planner.foot_down_target((0., 0., 0.), .5, vector)

    def test_changed_pose_cannot_reuse_old_start_or_grant_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile_path = root / 'profile.json'
            capture_path = root / 'capture.json'
            urdf_path = root / 'model.urdf'
            profile_path.write_text('{}')
            urdf_path.write_bytes(b'model')
            profile = {
                'scope': 'fixed_catch_current_hold_only',
                'boot_id': 'boot',
                'motor_power_epoch': 'old-power',
                'profile_sha256': hashlib.sha256(b'{}').hexdigest(),
                'start_pose_bounds': {str(i): [-.1, .1] for i in range(1, 13)},
                'axes': {
                    str(i): {
                        'uid': f'{i:016x}', 'sign': 1,
                        'offset_rad': -.5 if i in (1, 4, 7, 10) else 0.,
                        'kp': 12., 'max_estimated_pd_torque_nm': .5,
                        'physical_lower_rad': -1., 'physical_upper_rad': 1.,
                    }
                    for i in range(1, 13)
                },
            }
            capture = {
                'schema': 'singularitydog.readonly-12-angle-capture.v1',
                'motor_output_allowed': False, 'approved_for_runtime': False,
                'angle_wrap_applied': False,
                'status': 'RECORDED_REVIEW_REQUIRED', 'errors': [],
                'boot_id': 'boot', 'started_at': '2026-09-29T00:00:00+09:00',
                'identities': {str(i): {'mcu_uid_hex': f'{i:016x}'}
                               for i in range(1, 13)},
                'telemetry': {'rows': {
                    str(i): {'run_mode': 0, 'current': 0.,
                             'voltage': 40.,
                             'position_span_deg': 0., 'median_position_rad': 0.,
                             'position_samples': [
                                 {'rad': 0., 'request_monotonic_ns': 10*n+1,
                                  'reply_monotonic_ns': 10*n+2} for n in range(3)]}
                    for i in range(1, 13)}},
            }
            capture_path.write_text(json.dumps(capture))
            feet = {'FR': (0., 0., 0.), 'FL': (0., 1., 0.),
                    'RR': (1., 0., 0.), 'RL': (1., 1., .008)}
            with (patch.object(planner, 'load_profile', return_value=profile),
                  patch.object(planner, 'URDF_SHA256',
                               hashlib.sha256(b'model').hexdigest()),
                  patch.object(planner, 'parse_d17', return_value={
                      leg: None for leg in planner.LEGS}),
                  patch.object(planner, 'foot_centers', return_value=feet),
                  patch.object(planner, '_ik_near',
                               side_effect=lambda start, desired, geometry, **_:
                               tuple(value + .01 for value in start))):
                with self.assertRaisesRegex(ValueError, 'initial encoder branch'):
                    planner.plan(profile_path, capture_path, urdf_path, .5, 3.)
                result = planner.plan(profile_path, capture_path, urdf_path,
                                      .5, 3., rebase_offline=True)
            self.assertFalse(result['motor_output_allowed'])
            self.assertFalse(result['box_removal_allowed'])
            self.assertFalse(result['approved_for_runtime'])
            self.assertFalse(result['source_screen_passed'])
            self.assertTrue(result['historical_profile_used_for_calibration_only'])
            self.assertGreater(result['nominal_four_foot_plane_residual_mm'], 2.)
            self.assertEqual(len(result['readiness_blockers']), 3)
            self.assertEqual(result['selected_branch_turns_by_id'],
                             {str(i): 0 for i in range(1, 13)})
            self.assertEqual(result['source_inputs']['capture']['sha256'],
                             hashlib.sha256(capture_path.read_bytes()).hexdigest())

    def test_raw_capture_summary_is_recomputed(self):
        row = dict(run_mode=0, current=0., voltage=40., median_position_rad=1.,
                   position_span_deg=math.degrees(.0002),
                   position_samples=[dict(rad=v, request_monotonic_ns=10*n+1,
                                          reply_monotonic_ns=10*n+2)
                                     for n, v in enumerate((.9999, 1., 1.0001))])
        self.assertEqual(planner.quiet_position(row, '1'), 1.)
        for key, value in (('median_position_rad', 1.0001), ('position_span_deg', 0.),
                           ('position_span_deg', float('nan')), ('voltage', float('inf')),
                           ('run_mode', False), ('current', False)):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                planner.quiet_position({**row, key: value}, '1')
        malformed = copy.deepcopy(row)
        malformed['position_samples'][1]['request_monotonic_ns'] = 1
        with self.assertRaisesRegex(ValueError, 'chronology'):
            planner.quiet_position(malformed, '1')

    def test_duplicate_keys_and_nonfinite_json_are_rejected(self):
        for source in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                planner.strict_json(source)

    def test_nonfinite_foot_translation_and_boolean_direction_are_rejected(self):
        for foot, rise, up in (((1., 2., float('nan')), .25, (0., 0., 1.)),
                              ((1., 2., 3.), float('nan'), (0., 0., 1.)),
                              ((1., 2., 3.), -.25, (0., 0., 1.)),
                              ((1., 2., 3.), .25, (0., 0., True))):
            with self.subTest(foot=foot, rise=rise, up=up), self.assertRaises(ValueError):
                planner.foot_down_target(foot, rise, up)


if __name__ == '__main__':
    unittest.main()
