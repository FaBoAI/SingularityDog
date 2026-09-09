"""Measurement timing and exact media allowlist regressions; no vendor assets."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from artifact_manifest import digest
from check_publication import check, RECORDING_AUTHORIZATION
from render_motion_history import validate


class MotionPublicationTests(unittest.TestCase):
    def recording_fixture(self, root):
        (root / 'docs/media').mkdir(parents=True)
        (root / 'evidence').mkdir()
        payloads = {'selected.mp4': b'\x00\x00\x00\x18ftyp' + b'synthetic-mp4-fixture',
                    'selected.png': b'\x89PNG\r\n\x1a\n' + b'synthetic-png-fixture',
                    'preview.gif': b'GIF89a' + b'synthetic-gif-fixture'}
        files = {}
        for name, payload in payloads.items():
            path = root / 'docs/media' / name
            path.write_bytes(payload)
            entry = {'sha256': digest(path), 'bytes': len(payload), 'reviewed_for_publication': True,
                     'kind': 'temporal_preview' if name == 'preview.gif' else 'user_selected_original'}
            entry['source_sha256'] = files['docs/media/selected.mp4']['sha256'] if name == 'preview.gif' else entry['sha256']
            if name == 'preview.gif':
                entry['source_path'] = 'docs/media/selected.mp4'
            files['docs/media/' + name] = entry
        registry = {'schema': 'singularitydog.user-selected-recordings.v1',
                    'authorization': RECORDING_AUTHORIZATION, 'files': files}
        path = root / 'evidence/recording-publication.json'
        path.write_text(json.dumps(registry))
        return path, registry

    def test_published_data_retains_all_four_first_intervals(self):
        data = json.loads((ROOT / 'evidence/forward-motion.json').read_text())
        validate(data)
        self.assertEqual([s['id'] for s in data['stages']], ['B0', 'H03', 'H04', 'L00'])
        self.assertIsNone(data['stages'][0]['foot_clearance_m'])
        self.assertAlmostEqual(data['stages'][0]['relative_x_m'][-1], .004807)

    def test_missing_time_and_manufactured_completion_rejected(self):
        original = json.loads((ROOT / 'evidence/forward-motion.json').read_text())
        for field in ('time', 'failure', 'height', 'geometry'):
            data = copy.deepcopy(original)
            if field == 'time':
                data['time_s'][3] = .09
            elif field == 'failure':
                data['stages'][1]['first_failure_step'] = 200
            elif field == 'height':
                data['stages'][1]['foot_clearance_m'].pop()
            else:
                data['vendor_geometry_included'] = True
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate(data)

    def test_media_without_review_or_with_changed_bytes_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'docs/media').mkdir(parents=True)
            (root / 'evidence').mkdir()
            media = root / 'docs/media/example.gif'
            media.write_bytes(b'GIF89a' + b'synthetic-test-header')
            self.assertTrue(check(root, ['docs/media/example.gif']))
            (root / 'data.json').write_text('{}')
            (root / 'generator.py').write_text('# synthetic generator fixture\n')
            review = {'schema': 'singularitydog.reviewed-media.v1', 'vendor_geometry_used': False,
                      'measured_data': {'path': 'data.json', 'sha256': digest(root / 'data.json')},
                      'generator': {'path': 'generator.py', 'sha256': digest(root / 'generator.py')},
                      'files': {'docs/media/example.gif': {'sha256': digest(media), 'bytes': media.stat().st_size,
                                                         'reviewed_as_own_measured_visualization': True}}}
            (root / 'evidence/media-publication.json').write_text(json.dumps(review))
            self.assertEqual(check(root, ['docs/media/example.gif']), [])
            media.write_bytes(b'GIF89a' + b'changed')
            self.assertTrue(check(root, ['docs/media/example.gif']))
            (root / 'data.json').write_text('{"changed":true}')
            with self.assertRaises(ValueError):
                check(root, ['docs/media/example.gif'])

    def test_individually_selected_originals_and_bound_preview_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, registry = self.recording_fixture(root)
            self.assertEqual(check(root, list(registry['files'])), [])
            extra = root / 'docs/media/unselected.mp4'
            extra.write_bytes((root / 'docs/media/selected.mp4').read_bytes())
            self.assertTrue(check(root, ['docs/media/unselected.mp4']))
            # Publishing a preview alone still checks the source MP4's actual bytes.
            (root / 'docs/media/selected.mp4').write_bytes(b'\x00\x00\x00\x18ftypchanged')
            with self.assertRaisesRegex(ValueError, 'changed after approval'):
                check(root, ['docs/media/preview.gif'])

    def test_selected_recording_scope_and_source_tampering_rejected(self):
        for variant in ('authorization', 'approval', 'source_hash', 'original_hash', 'source_kind',
                        'source_outside_registry', 'traversal', 'nested', 'unsupported_type'):
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                path, registry = self.recording_fixture(root)
                files = registry['files']
                if variant == 'authorization':
                    registry['authorization'] = 'not-the-user-request'
                elif variant == 'approval':
                    files['docs/media/selected.mp4']['reviewed_for_publication'] = False
                elif variant == 'source_hash':
                    files['docs/media/preview.gif']['source_sha256'] = '0' * 64
                elif variant == 'original_hash':
                    files['docs/media/selected.png']['source_sha256'] = '0' * 64
                elif variant == 'source_kind':
                    files['docs/media/preview.gif'].update(source_path='docs/media/selected.png',
                                                         source_sha256=files['docs/media/selected.png']['sha256'])
                elif variant == 'source_outside_registry':
                    files['docs/media/preview.gif']['source_path'] = 'other.mp4'
                else:
                    replacement = {'traversal': 'docs/media/../selected.png',
                                   'nested': 'docs/media/nested/selected.png',
                                   'unsupported_type': 'docs/media/selected.step'}[variant]
                    files[replacement] = files.pop('docs/media/selected.png')
                path.write_text(json.dumps(registry))
                with self.assertRaises(ValueError):
                    check(root, ['docs/media/preview.gif'])

    def test_selected_magic_size_and_duplicate_registry_keys_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, registry = self.recording_fixture(root)
            preview = root / 'docs/media/preview.gif'
            preview.write_bytes(b'not-a-gif')
            entry = registry['files']['docs/media/preview.gif']
            entry.update(sha256=digest(preview), bytes=preview.stat().st_size)
            path.write_text(json.dumps(registry))
            self.assertTrue(check(root, ['docs/media/preview.gif']))
            png = registry['files']['docs/media/selected.png']
            png['bytes'] += 1
            path.write_text(json.dumps(registry))
            self.assertTrue(check(root, ['docs/media/selected.png']))
            with self.assertRaisesRegex(ValueError, 'Duplicate publication file'):
                check(root, ['docs/media/selected.png', 'docs/media/selected.png'])
            # json.loads normally drops the first duplicate; the registry must not.
            encoded = json.dumps(registry)
            duplicate = '"files": {"docs/media/selected.png": ' + json.dumps(png) + ', '
            encoded = encoded.replace('"files": {', duplicate, 1)
            path.write_text(encoded)
            with self.assertRaisesRegex(ValueError, 'Duplicate publication registry key'):
                check(root, ['docs/media/selected.png'])


if __name__ == '__main__':
    unittest.main()
