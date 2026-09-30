"""Explicit kit members retain one path spelling and exact type/hash evidence."""
import copy
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import artifact_manifest as manifest
import bounded_job


class ArtifactManifestTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / 'kit'
        self.root.mkdir()
        (self.root / 'a').mkdir()
        (self.root / 'a/b.txt').write_bytes(b'SYNTHETIC KIT MEMBER')
        (self.root / 'empty.txt').write_bytes(b'')
        self.names = ['a/b.txt', 'empty.txt']

    def write_json(self, name, value):
        path = Path(self.tmp.name) / name
        path.write_text(json.dumps(value))
        return path

    def test_round_trip_counts_real_members_including_zero_byte_file(self):
        value = manifest.create(self.root, self.names)
        self.assertEqual(manifest.verify(self.root, value), 2)
        self.assertEqual(value['files']['empty.txt']['bytes'], 0)
        self.assertEqual(set(value['files']), set(self.names))

    def test_strict_json_rejects_float_overflow_in_nested_values(self):
        for number in ('1e9999','-1e9999'):
            with self.subTest(number=number),self.assertRaisesRegex(ValueError,'Nonfinite JSON number'):
                manifest.parse_json('{"nested":['+number+']}')
        self.assertEqual(manifest.parse_json('{"finite":1e3}'),{'finite':1000.0})

    def test_direct_create_and_load_names_reject_empty_duplicate_and_non_list(self):
        for names in ([], ['a/b.txt', 'a/b.txt'], 'a/b.txt', {'a/b.txt': 1}, None, [1]):
            with self.subTest(names=names):
                with patch.object(manifest, 'digest') as digest:
                    with self.assertRaises(ValueError):
                        manifest.create(self.root, names)
                    digest.assert_not_called()
                with self.assertRaises(ValueError):
                    manifest.load_names(self.write_json('names.json', names))

    def test_path_aliases_absolute_parent_and_non_posix_names_are_rejected(self):
        for name in ('a//b.txt', 'a/./b.txt', './a/b.txt', 'a/b.txt/',
                     '../outside.txt', 'a/../empty.txt', '/a/b.txt', '', '.',
                     'a\\b.txt', 'a/line\n.txt', 'a/nul\0.txt'):
            with self.subTest(name=repr(name)):
                with self.assertRaises(ValueError):
                    manifest.member(self.root, name)
                with self.assertRaises(ValueError):
                    manifest.create(self.root, ['a/b.txt', name])
                with self.assertRaises(ValueError):
                    manifest.load_names(self.write_json('names.json', [name]))

    def test_name_type_is_validated_before_path_construction(self):
        for name in (None, 1, True, [], {}, Path('a/b.txt')):
            with self.subTest(name=name), patch.object(manifest, 'Path') as path:
                with self.assertRaisesRegex(ValueError, 'canonical'):
                    manifest.member(self.root, name)
                path.assert_not_called()

    def test_symbolic_link_file_and_directory_cannot_include_external_members(self):
        outside = Path(self.tmp.name) / 'outside'
        outside.mkdir()
        (outside / 'payload').write_bytes(b'EXTERNAL FIXTURE')
        (self.root / 'linked-file').symlink_to(outside / 'payload')
        (self.root / 'linked-dir').symlink_to(outside, target_is_directory=True)
        for name in ('linked-file', 'linked-dir/payload'):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'symbolic links'):
                manifest.create(self.root, [name])
        with self.assertRaises(ValueError):
            manifest.create(self.root, ['../outside/payload'])

    def test_git_metadata_missing_file_and_directory_are_not_manifest_members(self):
        for name in ('.git/config', 'a/.git/config', 'missing.txt', 'a'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                manifest.create(self.root, [name])

    def test_hash_mismatch_does_not_pass_when_size_is_unchanged(self):
        expected = manifest.create(self.root, self.names)
        original = (self.root / 'a/b.txt').read_bytes()
        (self.root / 'a/b.txt').write_bytes(b'X' * len(original))
        with self.assertRaisesRegex(ValueError, 'Manifest mismatch'):
            manifest.verify(self.root, expected)

    def test_manifest_shape_and_types_are_checked_before_hashing(self):
        expected = manifest.create(self.root, self.names)
        invalid = [None, [], 1, {}, {'schema': expected['schema'], 'files': []},
                   {'schema': expected['schema'], 'files': {}}, {**expected, 'extra': True}]
        for row in (None, [], {}, {'sha256': 'a' * 64, 'bytes': True},
                    {'sha256': 'a' * 64, 'bytes': -1}, {'sha256': 'a' * 64, 'bytes': 1.0},
                    {'sha256': 'A' * 64, 'bytes': 1}, {'sha256': 'g' * 64, 'bytes': 1},
                    {'sha256': 'a' * 63, 'bytes': 1}, {'sha256': None, 'bytes': 1},
                    {'sha256': 'a' * 64, 'bytes': 1, 'extra': True}):
            value = copy.deepcopy(expected)
            value['files']['a/b.txt'] = row
            invalid.append(value)
        for value in invalid:
            with self.subTest(value=value), patch.object(manifest, 'digest') as digest:
                with self.assertRaises(ValueError):
                    manifest.verify(self.root, value)
                digest.assert_not_called()

    def test_manifest_alias_entry_is_rejected(self):
        value = manifest.create(self.root, ['a/b.txt'])
        value['files']['a//b.txt'] = value['files']['a/b.txt']
        with self.assertRaisesRegex(ValueError, 'canonical'):
            manifest.verify(self.root, value)

    def test_cli_round_trip_and_existing_manifest_protection(self):
        names = self.write_json('names.json', self.names)
        output = Path(self.tmp.name) / 'manifest.json'
        common = ['--root', str(self.root), '--manifest', str(output)]
        manifest.main(['create', *common, '--files', str(names)])
        original = output.read_bytes()
        with self.assertRaises(FileExistsError):
            manifest.main(['create', *common, '--files', str(names)])
        self.assertEqual(output.read_bytes(), original)
        with redirect_stdout(io.StringIO()) as stdout:
            manifest.main(['verify', *common])
        self.assertEqual(json.loads(stdout.getvalue()), {'status': 'VERIFIED', 'files': 2})

    def test_cli_rejects_duplicate_json_keys_including_nested_entries(self):
        output = Path(self.tmp.name) / 'manifest.json'
        for value in (
            '{"schema":"bad","schema":"singularitydog.manifest.v1","files":{}}',
            '{"schema":"singularitydog.manifest.v1","files":{"a/b.txt":{},"a/b.txt":{}}}',
            '{"schema":"singularitydog.manifest.v1","files":{"a/b.txt":{"bytes":0,"bytes":1}}}',
        ):
            output.write_text(value)
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'Duplicate'):
                manifest.main(['verify', '--root', str(self.root), '--manifest', str(output)])

    def test_cli_rejects_manifest_self_inclusion(self):
        output = self.root / 'output.json'
        output.write_text('{}')
        names = self.write_json('names.json', ['output.json'])
        with self.assertRaisesRegex(ValueError, 'contain itself'):
            manifest.main(['create', '--root', str(self.root), '--manifest', str(output),
                           '--files', str(names)])
        self.assertEqual(output.read_text(), '{}')

    def test_bounded_job_rejects_duplicate_manifest_keys_after_matching_file_hash(self):
        # The outer job receipt matches these exact bytes: parsing must still
        # reject duplicates before a verified kit can authorize child execution.
        source = Path(self.tmp.name) / 'manifest.json'
        source.write_text('{"schema":"bad","schema":"singularitydog.manifest.v1","files":{}}')
        request = self.write_json('request.json', {
            'schema': 'singularitydog.job.v1', 'job_id': 'synthetic-test',
            'argv': ['/bin/true'], 'cwd': str(self.root), 'timeout_s': 1,
            'expected_completion': {'status': 'COMPLETE'},
            'completion_file': str(Path(self.tmp.name) / 'not-created.json'),
            'manifest_file': str(source), 'manifest_sha256': manifest.digest(source),
            'artifact_root': str(self.root),
        })
        with patch.object(bounded_job.subprocess, 'Popen') as child:
            with self.assertRaisesRegex(ValueError, 'Duplicate manifest JSON key'):
                bounded_job.validated_request(request)
            child.assert_not_called()


if __name__ == '__main__':
    unittest.main()
