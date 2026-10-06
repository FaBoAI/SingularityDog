"""Packaging boundaries; no adapter/native/model/hardware execution."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from . import package


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='r50-package-test-', dir='/private/tmp')
        self.root = Path(self.temp.name)
        self.source = self.root / 'source'; self.source.mkdir()
        self.local = self.root / 'external'; self.local.mkdir()
        self.references = {}; self.counterparts = {}; self.pins = {}
        self.target_root = '/home/jetson/r50-package-fixture'
        self.input_target = self.target_root + '/inputs'
        self.maps = {self.target_root: str(self.local)}
        for name in package.SOURCE_NAMES:
            raw = ('# sealed fixture ' + name + '\n').encode()
            (self.source / name).write_bytes(raw)
        self.source_pins = {name: package.sha((self.source / name).read_bytes())
                            for name in package.SOURCE_NAMES}
        kit_dir = self.local / 'kit'; kit_dir.mkdir()
        kit_files = {}
        for number in range(650):
            name = 'source-' + str(number) + '.py'
            raw = ('# kit ' + str(number) + '\n').encode()
            (kit_dir / name).write_bytes(raw); kit_files[name] = package.sha(raw)
        kit_raw = package.json_bytes({'files': kit_files})
        (kit_dir / 'kit-manifest.json').write_bytes(kit_raw)
        self.kit_pin = package.sha(kit_raw)
        self.nested_local = self.local / 'nested.json'
        self.nested_local.write_bytes(b'{}\n')
        nested = {'path': self.target_root + '/nested.json',
                  'sha256': package.sha(self.nested_local.read_bytes())}
        for role in package.REFERENCE_PINS:
            if role in package.INPUT_COPIES:
                relative = 'inputs/' + package.INPUT_COPIES[role]
            elif role.endswith('source_manifest'):
                relative = role + '/manifest.json'
            else:
                relative = role + '.json'
            path = self.local / relative; path.parent.mkdir(parents=True, exist_ok=True)
            document = {}
            if role.endswith('source_manifest'):
                count = {'r48_source_manifest': 21, 'r47_source_manifest': 13,
                         'r49_source_manifest': 23}[role]
                inventory = {}
                for number in range(count):
                    name = 'member-' + str(number) + '.py'; raw = ('# ' + name).encode()
                    (path.parent / name).write_bytes(raw); inventory[name] = package.sha(raw)
                document['source_sha256' if role.startswith('r48') else 'files'] = inventory
                if role != 'r48_source_manifest':
                    document['target_bundle_path'] = self.target_root + '/' + role
                if role == 'r49_source_manifest':
                    document.update(baseline_kit_manifest_sha256=self.kit_pin,
                                    baseline_kit_path=self.target_root + '/kit')
            elif role.startswith('fk_candidate') or role == 'snapshot_artifact':
                document['references'] = {'nested': nested}
            raw = package.json_bytes(document); path.write_bytes(raw)
            expected = package.sha(raw); self.pins[role] = expected
            self.references[role] = {'path': self.target_root + '/' + relative, 'sha256': expected}
            self.counterparts[role] = str(path)
        self.patch_pins = mock.patch.object(package, 'REFERENCE_PINS', self.pins)
        self.patch_kit = mock.patch.object(package, 'KIT_PIN', self.kit_pin)
        self.patch_pins.start(); self.patch_kit.start()
        self.config = {'source_sha256': self.source_pins,
            'references': self.references, 'local_references': self.counterparts,
            'local_path_mappings': self.maps, 'target_bundle_path': self.target_root + '/r50-source',
            'target_output_directory': self.target_root + '/result',
            'target_inputs_directory': self.input_target, 'python': '/usr/bin/python3'}

    def tearDown(self):
        self.patch_kit.stop(); self.patch_pins.stop(); self.temp.cleanup()

    def test_exact_separate_archives_and_truthful_command_specs(self):
        out = self.root / 'prepared'
        with mock.patch('subprocess.run', side_effect=AssertionError('No subprocess permitted')):
            result = package.prepare(self.source, out, self.config)
        self.assertEqual(set(result['source_members']), package.SOURCE_NAMES | {'manifest.json'})
        self.assertEqual(set(result['input_archive_members']), set(package.INPUT_COPIES.values()))
        package.verify_archive((out / 'r50-source.tar.gz').read_bytes(), result['archive_members'])
        package.verify_archive((out / 'r50-inputs.tar.gz').read_bytes(), result['input_archive_members'])
        self.assertEqual({p.name for p in (out / 'source').iterdir()},
                         package.SOURCE_NAMES | {'manifest.json'})
        self.assertFalse(result['nested_target_reference_validation_pending'])
        self.assertEqual(result['candidate_implementation_status'], package.IMPLEMENTATION_STATUS)
        self.assertFalse(result['target_run_permitted'])
        self.assertFalse(result['model_loaded']); self.assertFalse(result['target_executed'])
        for mode, requested in (('plan', False), ('exec', True)):
            spec = json.loads((out / (mode + '-spec.json')).read_bytes())
            self.assertEqual(spec['requested_model_load'], requested)
            self.assertFalse(spec['model_loaded'])
            self.assertFalse(spec['target_run_permitted'])
            self.assertTrue(spec['recursive_historical_target_reference_validation_pending'])
            self.assertEqual(spec['argv'][-1], mode.upper())
            self.assertEqual(spec['argv'][1:3], ['-I', '-B'])
            pin_index = spec['argv'].index('--manifest-sha256')
            self.assertEqual(spec['argv'][pin_index + 1], result['manifest_sha256'])
        self.assertEqual(package.archive_bytes({'file.py': b'abc'}),
                         package.archive_bytes({'file.py': b'abc'}))

    def test_changed_core_or_source_fails_before_output(self):
        for label, path in (('source', self.source / 'adapter.py'),
                            ('input', Path(self.counterparts['records']))):
            original = path.read_bytes(); path.write_bytes(original + b'changed')
            out = self.root / ('changed-' + label)
            with self.assertRaisesRegex(ValueError, 'Pinned file changed'):
                package.prepare(self.source, out, self.config)
            self.assertFalse(out.exists()); path.write_bytes(original)

    def test_extra_pyc_or_symlink_source_fails_before_output(self):
        pyc = self.source / 'unexpected.pyc'; pyc.write_bytes(b'cache')
        with self.assertRaisesRegex(ValueError, 'extra files rejected'):
            package.prepare(self.source, self.root / 'bad-pyc', self.config)
        pyc.unlink()
        original = self.source / 'adapter.py'; raw = original.read_bytes(); original.unlink()
        replacement = self.root / 'replacement.py'; replacement.write_bytes(raw)
        original.symlink_to(replacement)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            package.prepare(self.source, self.root / 'bad-link', self.config)

    def test_nested_missing_is_explicit_pending_but_hash_drift_is_fatal(self):
        self.nested_local.unlink()
        with self.assertRaises(FileNotFoundError):
            package.prepare(self.source, self.root / 'missing-strict', self.config)
        self.config['allow_nested_target_validation_pending'] = True
        result = package.prepare(self.source, self.root / 'missing-pending', self.config)
        self.assertTrue(result['nested_target_reference_validation_pending'])
        self.assertEqual(set(result['pending_nested_target_references']),
                         {self.target_root + '/nested.json'})
        self.nested_local.write_bytes(b'{"changed":true}\n')
        with self.assertRaisesRegex(ValueError, 'Pinned file changed'):
            package.prepare(self.source, self.root / 'hash-bad', self.config)

    def test_input_source_overlap_and_immutable_reuse_rejected(self):
        self.config['target_inputs_directory'] = self.config['target_bundle_path'] + '/inputs'
        with self.assertRaisesRegex(ValueError, 'outside the source inventory'):
            package.prepare(self.source, self.root / 'overlap', self.config)
        self.config['target_inputs_directory'] = self.input_target
        self.config['target_bundle_path'] = self.target_root + '/r49_source_manifest/r50'
        with self.assertRaisesRegex(ValueError, 'outside immutable source roots'):
            package.prepare(self.source, self.root / 'immutable', self.config)
        self.config['target_bundle_path'] = self.target_root + '/kit/r50'
        with self.assertRaisesRegex(ValueError, 'outside original K37'):
            package.prepare(self.source, self.root / 'kit', self.config)
        self.config['target_bundle_path'] = self.target_root + '/r50-source'
        self.config['target_output_directory'] = self.input_target + '/result'
        with self.assertRaisesRegex(ValueError, 'result directories must be distinct'):
            package.prepare(self.source, self.root / 'input-result', self.config)

    def test_directory_fifo_rejection_closes_file_descriptors(self):
        fifo = self.root / 'fifo'; os.mkfifo(fifo)
        before = len(os.listdir('/dev/fd'))
        for path in (self.root, fifo):
            for unused in range(12):
                with self.assertRaisesRegex(ValueError, 'regular file'):
                    package.read_pinned(path, '0' * 64)
        self.assertEqual(len(os.listdir('/dev/fd')), before)

    def test_json_and_archive_member_admission(self):
        for raw in (b'{"x":1,"x":2}', b'{"x":NaN}'):
            with self.assertRaises(ValueError):
                package.parse(raw)
        for name in ('../escape.py', '/escape.py', '__pycache__/../cache.pyc'):
            with self.assertRaises(ValueError):
                package.archive_bytes({name: b'unsafe'})
        raw = package.archive_bytes({'a.py': b'a'})
        with self.assertRaisesRegex(ValueError, 'Exact regular archive'):
            package.verify_archive(raw, {'a.py': package.sha(b'a'), 'b.py': package.sha(b'b')})


if __name__ == '__main__':
    unittest.main()
