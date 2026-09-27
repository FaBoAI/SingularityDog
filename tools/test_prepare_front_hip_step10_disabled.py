"""File-only orchestration checks for front-hip disabled preparation."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prepare_front_hip_step10_disabled as command


class PrepareFrontHipStep10DisabledTests(unittest.TestCase):
    def test_one_command_builds_only_fixed_front_hip_candidate_and_disabled_package(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'source'
            source.mkdir()
            hold = base / 'hold.json'
            hold.write_text('{}')
            root = base / 'new-output'
            source, hold, root = source.resolve(), hold.resolve(), root.resolve()
            with patch.object(command.builder, 'prepare', return_value={
                    'boot_id': 'boot', 'candidate_sha256': 'a' * 64}) as candidate, patch.object(
                    command.builder, 'disabled', return_value={
                        'manifest_sha256': 'b' * 64}) as disabled:
                result = command.prepare(source, hold, root, 'front-hip-step10-boot-r1')
            candidate.assert_called_once_with(source, hold, 'front-hip',
                                              root / 'front-hip-step10-boot-r1-prepared',
                                              'front-hip-mirrored', 10.)
            disabled.assert_called_once_with(source, hold,
                                             root / 'front-hip-step10-boot-r1-prepared',
                                             root / 'front-hip-step10-boot-r1-disabled')
            self.assertTrue(result['disabled_only'])

    def test_rejects_existing_or_nested_output_before_building(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'source'
            source.mkdir()
            hold = base / 'hold.json'
            hold.write_text('{}')
            with patch.object(command.builder, 'prepare') as candidate:
                for root in (source / 'nested', base):
                    with self.subTest(root=root), self.assertRaises(ValueError):
                        command.prepare(source, hold, root, 'front-hip-step10-r1')
                candidate.assert_not_called()

    def test_continuous_option_is_explicit_and_keeps_disabled_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'source'
            source.mkdir()
            hold = base / 'hold.json'
            hold.write_text('{}')
            root = base / 'new-output'
            source, hold, root = source.resolve(), hold.resolve(), root.resolve()
            with patch.object(command.builder, 'prepare', return_value={
                    'boot_id': 'boot', 'candidate_sha256': 'a' * 64}) as candidate, patch.object(
                    command.builder, 'disabled', return_value={
                        'manifest_sha256': 'b' * 64}):
                result = command.prepare(source, hold, root, 'continuous-r1',
                                         continuous_5_10=True)
            candidate.assert_called_once_with(
                source, hold, 'front-hip', root / 'continuous-r1-prepared',
                'front-hip-mirrored', 10.,
                continuous_profile=command.builder.CONTINUOUS_FRONT_HIP_PROFILE)
            self.assertTrue(result['disabled_only'])
            self.assertEqual(result['continuous_profile'],
                             command.builder.CONTINUOUS_FRONT_HIP_PROFILE)


if __name__ == '__main__':
    unittest.main()
