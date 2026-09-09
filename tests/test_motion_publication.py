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
from check_publication import check
from render_motion_history import validate


class MotionPublicationTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
