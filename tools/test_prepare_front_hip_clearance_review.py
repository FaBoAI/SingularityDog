"""No-hardware checks for preflight-bound physical-review draft generation."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prepare_front_hip_clearance_review as command


class FrontHipClearanceDraftTests(unittest.TestCase):
    def test_populates_twelve_angles_but_never_preauthorizes_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            disabled = base / 'disabled'
            disabled.mkdir()
            (disabled / 'prepared_fullbody.py').write_text('frozen')
            command.role.write_json(disabled / 'step2-review.json', {
                'boot_id': 'boot', 'motor_uids': {str(i): f'uid{i}' for i in range(1, 13)},
                'role_group': 'front-hip', 'amplitude_deg': 10.,
                'direction_profile': 'front-hip-mirrored',
                'raw_direction_by_id': {str(i): (1 if i == 3 else -1 if i == 6 else 0)
                                        for i in range(1, 13)}})
            preflight = base / 'preflight'
            preflight.mkdir()
            command.role.write_json(preflight / 'summary.json', {
                'result': {'workers': {
                    'front': {'centers': {str(i): i / 10 for i in range(1, 7)}},
                    'rear': {'centers': {str(i): i / 10 for i in range(7, 13)}}}}})
            (preflight / 'events.jsonl').write_text('{}\n')
            output = base / 'draft.json'
            with patch.object(command.role.active_base, 'validate_frozen'), patch.object(
                    command.role, 'validate_role_group_preflight'):
                result = command.prepare(disabled, preflight, output)
            draft = command.role.read_json(output)
            self.assertEqual(len(draft['clearance_reference_raw_rad_by_id']), 12)
            self.assertEqual(draft['start_tolerance_clearance_verified_deg'], 3.)
            self.assertEqual(draft['start_tolerance_clearance_note'], '')
            self.assertTrue(all(draft[flag] is False for flag in command.role.FLAGS))
            self.assertFalse(result['active_output_authorized'])
            self.assertFalse(result['physical_approval_complete'])

    def test_continuous_draft_requires_separate_19_second_review(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            disabled = base / 'disabled'
            disabled.mkdir()
            (disabled / 'prepared_fullbody.py').write_text('frozen')
            command.role.write_json(disabled / 'step2-review.json', {
                'boot_id': 'boot', 'motor_uids': {str(i): f'uid{i}' for i in range(1, 13)},
                'role_group': 'front-hip', 'amplitude_deg': 10.,
                'direction_profile': 'front-hip-mirrored',
                'raw_direction_by_id': {str(i): (1 if i == 3 else -1 if i == 6 else 0)
                                        for i in range(1, 13)},
                'continuous_profile': command.role.CONTINUOUS_FRONT_HIP_PROFILE,
                'continuous_waypoints_deg': [5., 10.]})
            preflight = base / 'preflight'
            preflight.mkdir()
            command.role.write_json(preflight / 'summary.json', {
                'result': {'workers': {
                    'front': {'centers': {str(i): i / 10 for i in range(1, 7)}},
                    'rear': {'centers': {str(i): i / 10 for i in range(7, 13)}}}}})
            (preflight / 'events.jsonl').write_text('{}\n')
            output = base / 'draft.json'
            with patch.object(command.role.active_base, 'validate_frozen'), patch.object(
                    command.role, 'validate_role_group_preflight'):
                command.prepare(disabled, preflight, output)
            draft = command.role.read_json(output)
            self.assertEqual(draft['continuous_profile'],
                             command.role.CONTINUOUS_FRONT_HIP_PROFILE)
            self.assertEqual(draft['continuous_waypoints_deg'], [5., 10.])
            self.assertIs(draft['continuous_19s_reviewed'], False)


if __name__ == '__main__':
    unittest.main()
