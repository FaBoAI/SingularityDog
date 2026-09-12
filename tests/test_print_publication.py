"""Selected print assets: reject hidden archive members and changed geometry."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from check_publication import check, PRINT_AUTHORIZATION


class PrintPublicationTests(unittest.TestCase):
    def fixture(self, root):
        directory = root / 'docs/printing/next-10-parts'
        directory.mkdir(parents=True)
        (root / 'evidence').mkdir()
        # Synthetic triangle, not a real printable part.
        stl = directory / 'example.stl'
        stl.write_bytes(bytes(80) + struct.pack('<I12fH', 1,
                        0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0))
        doc = directory / 'README.md'
        doc.write_text('Synthetic print fixture.\n')
        files = {}
        for f in [stl, doc]:
            files[str(f.relative_to(root))] = self.entry(f)
        files[str(stl.relative_to(root))].update(
            units='mm', strict_binary_stl=True, triangles=1,
            bbox_min_mm=[0, 0, 0], bbox_max_mm=[1, 1, 0], dimensions_mm=[1, 1, 0],
            source_sha256=hashlib.sha256(stl.read_bytes()).hexdigest())
        archive = directory / 'selected.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            for f in [stl, doc]: z.write(f, f.name)
        files[str(archive.relative_to(root))] = self.entry(archive)
        files[str(archive.relative_to(root))]['members'] = {
            f.name: {'public_path':str(f.relative_to(root)),
                     'sha256':self.entry(f)['sha256'], 'bytes':f.stat().st_size}
            for f in [stl, doc]}
        registry = {'schema':'singularitydog.user-selected-print-files.v1',
                    'authorization':PRINT_AUTHORIZATION, 'files':files}
        self.save(root, registry)
        return directory, registry

    def entry(self, f):
        return {'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),
                'bytes':f.stat().st_size, 'reviewed_for_publication':True}

    def save(self, root, d):
        (root / 'evidence/print-publication.json').write_text(json.dumps(d))

    def test_only_registered_binary_files_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); directory, d = self.fixture(root)
            self.assertEqual(check(root, list(d['files'])), [])
            extra = directory / 'unselected.stl'
            extra.write_bytes((directory / 'example.stl').read_bytes())
            self.assertTrue(check(root, [str(extra.relative_to(root))]))

    def test_changed_stl_length_nan_and_dimensions_rejected(self):
        for variant in ['changed', 'length', 'nan', 'dimensions', 'authorization']:
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as temp:
                root = Path(temp); directory, d = self.fixture(root)
                stl = directory / 'example.stl'; entry = d['files'][str(stl.relative_to(root))]
                if variant == 'authorization': d['authorization'] = 'unapproved'
                elif variant == 'dimensions': entry['bbox_max_mm'][0] = 2
                else:
                    payload = bytearray(stl.read_bytes())
                    if variant in ['changed', 'length']: payload.extend(b'x')
                    else: struct.pack_into('<f', payload, 96, float('nan'))
                    stl.write_bytes(payload)
                    if variant != 'changed':
                        entry.update(self.entry(stl)); entry['source_sha256'] = entry['sha256']
                self.save(root, d)
                with self.assertRaises(ValueError): check(root, list(d['files']))

    def test_archive_unlisted_duplicate_and_member_changes_rejected(self):
        for variant in ['extra', 'duplicate', 'changed', 'traversal']:
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as temp:
                root = Path(temp); directory, d = self.fixture(root)
                archive = directory / 'selected.zip'
                with zipfile.ZipFile(archive, 'w') as z:
                    z.write(directory / 'example.stl', 'example.stl')
                    z.writestr('README.md', 'changed' if variant == 'changed' else (directory / 'README.md').read_text())
                    if variant != 'changed':
                        z.writestr({'extra':'private.jpg', 'duplicate':'README.md',
                                    'traversal':'../private.jpg'}[variant], b'extra')
                d['files'][str(archive.relative_to(root))].update(self.entry(archive))
                self.save(root,d)
                with self.assertRaises(ValueError): check(root, list(d['files']))

    def test_registered_text_still_receives_privacy_scan(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); directory, d = self.fixture(root)
            archive = directory / 'selected.zip'
            del d['files'][str(archive.relative_to(root))]
            doc = directory / 'README.md'
            doc.write_text('/' + 'home' + '/private-user/private-file\n')
            d['files'][str(doc.relative_to(root))].update(self.entry(doc))
            self.save(root,d)
            with self.assertRaisesRegex(ValueError, 'Private or binary content'):
                check(root, list(d['files']))

    def test_archive_only_cannot_bypass_document_privacy(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); directory, d = self.fixture(root)
            doc = directory / 'README.md'
            doc.write_text('/' + 'home' + '/private-user/private-file\n')
            docname = str(doc.relative_to(root))
            d['files'][docname].update(self.entry(doc))
            archive = directory / 'selected.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                for f in [directory / 'example.stl', doc]: z.write(f, f.name)
            zipentry = d['files'][str(archive.relative_to(root))]
            zipentry.update(self.entry(archive))
            zipentry['members']['README.md'].update(sha256=self.entry(doc)['sha256'],
                                                  bytes=doc.stat().st_size)
            self.save(root, d)
            with self.assertRaisesRegex(ValueError, 'Private or binary content'):
                check(root, [str(archive.relative_to(root))])


if __name__ == '__main__':
    unittest.main()
