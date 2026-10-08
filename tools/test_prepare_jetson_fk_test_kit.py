"""Source-only kit boundaries tested in a disposable Git repository."""
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

import prepare_jetson_fk_test_kit as kit


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class SourceKitTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name).resolve()
        self.repo = self.base / 'repository'
        self.repo.mkdir()
        subprocess.run(['git', 'init', '-q', str(self.repo)], check=True)
        (self.repo / 'runtime' / '__pycache__').mkdir(parents=True)
        (self.repo / 'tools').mkdir()
        (self.repo / 'runtime' / 'regular.py').write_bytes(b'answer = 42\n')
        (self.repo / 'tools' / 'new.py').write_bytes(b'print("file only")\n')
        (self.repo / 'runtime' / 'tracked.bin').write_bytes(b'\x00explicit tracked fixture')
        (self.repo / 'runtime' / 'untracked.bin').write_bytes(b'\x00exclude me')
        (self.repo / 'runtime' / 'untracked.so').write_bytes(b'ASCII binary suffix fixture')
        (self.repo / 'runtime' / '__pycache__' / 'x.pyc').write_bytes(b'\x00bytecode')
        (self.repo / '.gitignore').write_text('runtime/ignored.py\n')
        (self.repo / 'runtime' / 'ignored.py').write_text('ignored = True\n')
        subprocess.run(['git', 'add', 'runtime/regular.py', 'runtime/tracked.bin',
                        'runtime/__pycache__/x.pyc'], cwd=self.repo, check=True)
        self.output = self.base / 'prepared'

    def tearDown(self):
        self.temp.cleanup()

    def build(self):
        result = kit.prepare(self.repo, self.output, build=True)
        self.manifest_path = Path(result['manifest'])
        self.archive_path = Path(result['archive'])
        self.installer_path = Path(result['installer'])
        self.manifest = json.loads(self.manifest_path.read_bytes())
        self.original_manifest = copy.deepcopy(self.manifest)
        self.result = result
        return result

    def invoke(self, destination=None, *, install=False, manifest_sha=None, archive_sha=None):
        destination = destination or self.base / 'target'
        command = [sys.executable, '-I', '-B', str(self.installer_path),
                   '--manifest', str(self.manifest_path), '--manifest-sha256',
                   manifest_sha or sha(self.manifest_path.read_bytes()),
                   '--archive', str(self.archive_path), '--archive-sha256',
                   archive_sha or sha(self.archive_path.read_bytes()),
                   '--destination', str(destination)]
        if install:
            command.append('--install')
        return subprocess.run(command, capture_output=True, text=True, timeout=10)

    def regular_members(self):
        with tarfile.open(self.archive_path, 'r:gz') as stream:
            return [(copy.copy(row), stream.extractfile(row).read()) for row in stream]

    def rewrite(self, *, members=None, mutate=None):
        data = copy.deepcopy(self.original_manifest)
        if members is not None:
            with tarfile.open(self.archive_path, 'w:gz', format=tarfile.PAX_FORMAT) as stream:
                for row, raw in members:
                    stream.addfile(row, io.BytesIO(raw) if row.isreg() else None)
        raw_archive = self.archive_path.read_bytes()
        data['archive']['bytes'] = len(raw_archive)
        data['archive']['sha256'] = sha(raw_archive)
        if mutate:
            mutate(data)
        self.manifest_path.write_text(json.dumps(data, sort_keys=True) + '\n')

    def assert_rejected(self, result, destination=None):
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse((destination or self.base / 'target').exists())

    def test_default_plan_is_read_only_and_source_only(self):
        result = kit.prepare(self.repo, self.output)
        self.assertEqual(result['status'], 'PLAN_SOURCE_ONLY')
        self.assertEqual(result['file_count'], 3)
        self.assertFalse(self.output.exists())
        for key in ('hardware_opened', 'model_loaded', 'output_allowed', 'approved_for_runtime'):
            self.assertIs(result[key], False)

    def test_exact_source_inventory_hashes_and_binary_rules(self):
        self.build()
        expected = {'runtime/regular.py', 'runtime/tracked.bin', 'tools/new.py'}
        self.assertEqual(set(self.manifest['files']), expected)
        self.assertTrue(self.manifest['files']['runtime/tracked.bin']['tracked'])
        self.assertFalse(self.manifest['files']['tools/new.py']['tracked'])
        self.assertEqual(sha(self.archive_path.read_bytes()), self.result['archive_sha256'])
        self.assertEqual(sha(self.installer_path.read_bytes()), self.result['installer_sha256'])
        self.assertEqual({row.name for row, raw in self.regular_members()}, expected)
        for name, row in self.manifest['files'].items():
            self.assertEqual(sha((self.output / 'kit' / name).read_bytes()), row['sha256'])
        self.assertEqual({row['path'] for row in self.manifest['skipped']},
                         {'runtime/__pycache__/x.pyc', 'runtime/untracked.bin', 'runtime/untracked.so'})

    def test_generated_installer_plan_install_and_no_overwrite(self):
        self.build()
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['status'], 'PLAN_SOURCE_ONLY')
        target = self.base / 'target'
        self.assertFalse(target.exists())
        result = self.invoke(install=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['status'], 'INSTALLED_SOURCE_ONLY')
        actual = {p.relative_to(target).as_posix() for p in target.rglob('*') if p.is_file()}
        self.assertEqual(actual, set(self.manifest['files']))
        for name, row in self.manifest['files'].items():
            self.assertEqual(sha((target / name).read_bytes()), row['sha256'])
        again = self.invoke(install=True)
        self.assertNotEqual(again.returncode, 0)
        self.assertEqual(sha((target / 'runtime/regular.py').read_bytes()),
                         self.manifest['files']['runtime/regular.py']['sha256'])

    def test_symlink_sources_and_unsafe_builder_destinations_are_rejected(self):
        (self.repo / 'tools/link.py').symlink_to(self.repo / 'runtime/regular.py')
        with self.assertRaises(ValueError):
            kit.prepare(self.repo, self.output, build=True)
        self.assertFalse(self.output.exists())
        (self.repo / 'tools/link.py').unlink()
        with self.assertRaises(ValueError):
            kit.prepare(self.repo, self.repo / 'inside')
        (self.base / 'alias').symlink_to(self.base, target_is_directory=True)
        with self.assertRaises(ValueError):
            kit.prepare(self.repo, self.base / 'alias' / 'new')

    def test_source_changes_during_build_remove_only_fresh_output(self):
        real = kit.inventory
        calls = 0
        def changing(repository):
            nonlocal calls
            calls += 1
            if calls == 2:
                (self.repo / 'runtime/regular.py').write_text('answer = 43\n')
            return real(repository)
        with mock.patch.object(kit, 'inventory', side_effect=changing):
            with self.assertRaisesRegex(ValueError, 'changed during packaging'):
                kit.prepare(self.repo, self.output, build=True)
        self.assertFalse(self.output.exists())
        self.assertTrue(self.repo.is_dir())

    def test_outer_pins_and_installer_self_pin_are_enforced(self):
        self.build()
        self.assert_rejected(self.invoke(manifest_sha='0' * 64))
        self.assert_rejected(self.invoke(archive_sha='0' * 64))
        with self.installer_path.open('ab') as stream:
            stream.write(b'\n')
        self.assert_rejected(self.invoke())

    def test_rebound_archive_structure_rejects_extra_missing_duplicate_and_links(self):
        self.build()
        original = self.regular_members()
        extra = tarfile.TarInfo('tools/extra.py'); extra.size = 1; extra.mode = 0o644
        traversal = tarfile.TarInfo('../escape'); traversal.size = 1; traversal.mode = 0o644
        for link_type in (tarfile.SYMTYPE, tarfile.LNKTYPE):
            row = copy.copy(original[0][0]); row.type = link_type; row.linkname = '../escape'; row.size = 0
            with self.subTest(kind=link_type):
                self.rewrite(members=[(row, b''), *original[1:]])
                self.assert_rejected(self.invoke(install=True))
        variants = [original + [(extra, b'x')], original[1:], original + [original[0]],
                    original + [(traversal, b'x')]]
        for index, members in enumerate(variants):
            with self.subTest(index=index):
                self.rewrite(members=members)
                self.assert_rejected(self.invoke(install=True))
        self.assertFalse((self.base / 'escape').exists())

    def test_rebound_metadata_cannot_weaken_source_verification_or_flags(self):
        self.build()
        mutations = [lambda d: d['files']['runtime/regular.py'].__setitem__('bytes', 1),
                     lambda d: d['files']['runtime/regular.py'].__setitem__('sha256', '0' * 64),
                     lambda d: d['files']['runtime/regular.py'].__setitem__('mode', 0o777),
                     lambda d: d.__setitem__('output_allowed', True)]
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                self.rewrite(mutate=mutate)
                self.assert_rejected(self.invoke(install=True))

    def test_installer_rejects_existing_git_or_symlink_destination(self):
        self.build()
        existing = self.base / 'existing'; existing.write_bytes(b'keep')
        self.assertNotEqual(self.invoke(existing, install=True).returncode, 0)
        self.assertEqual(existing.read_bytes(), b'keep')
        inside = self.repo / 'inside'
        self.assert_rejected(self.invoke(inside, install=True), inside)
        (self.base / 'alias').symlink_to(self.base, target_is_directory=True)
        target = self.base / 'alias' / 'new'
        self.assert_rejected(self.invoke(target, install=True), target)

    def test_canonical_path_rejects_controls_absolute_and_traversal(self):
        for name in ('.', 'runtime', '/runtime/file.py', 'runtime/../file.py',
                     'runtime//file.py', 'runtime/a\\b.py', 'runtime/a\n.py', 'runtime/a\x00.py'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                kit.member_name(name)


if __name__ == '__main__':
    unittest.main()
