"""Pinned metadata/factory contracts; no native build, procfs or devices."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import sourced_boot_guard as guard


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class FakeExtensionLoader:
    def __init__(self, name, path):
        self.name, self.path = name, path

    def create_module(self, spec):
        module = types.ModuleType(self.name)
        module.__file__, module.__spec__ = self.path, spec
        module.fresh_boot_matches = Mock(side_effect=lambda fd, expected:
            guard._reference.os.pread(fd, 80, 0).strip() == expected)
        return module

    def exec_module(self, module):
        pass

    def is_package(self, name):
        return False


class SourcedBootGuardTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.original_binding = guard._NATIVE_BINDING
        self.original_module = sys.modules.pop(guard._NATIVE_NAME, None)
        guard._NATIVE_BINDING = None
        runtime = Path(guard.__file__).resolve().parents[1]
        experiments = runtime/'experiments/native_diagnostic_r1'
        originals = {'cpp': experiments/'native_boot_guard.cpp', 'adapter': experiments/'native_guard.py',
                     'build_script': experiments/'build_native_boot_guard.py',
                     'reference_guard': Path(guard._reference.__file__).resolve()}
        self.paths = {}
        for key, original in originals.items():
            target = self.root/original.name
            target.write_bytes(original.read_bytes())
            self.paths[key] = target
        self.paths['library'] = self.root/(guard._NATIVE_NAME + guard._environment()['EXT_SUFFIX'])
        self.paths['library'].write_bytes(b'not loaded by these metadata tests')
        self.paths['build_record'] = self.root/'build-record.json'
        self.build = {'status': 'BUILT', 'python': sys.version, 'platform': guard._environment()['platform'],
                      'command': ['test-compiler', str(self.paths['cpp']), '-o', str(self.paths['library'])],
                      'compiler_version': 'unit test only', 'source_sha256': sha(self.paths['cpp']),
                      'library': str(self.paths['library']), 'library_sha256': sha(self.paths['library'])}
        self.paths['build_record'].write_text(json.dumps(self.build))
        self.manifest = {'schema': guard.SCHEMA, 'mode': guard.MODE, 'scope': guard.SCOPE,
                         'environment': guard._environment(),
                         'files': {key: {'path': str(path), 'sha256': sha(path)} for key, path in self.paths.items()}}
        self.manifest_path = self.root/'manifest.json'
        self.reference = self.write_manifest()
        self.reference_file = patch.object(guard._reference, '__file__', str(self.paths['reference_guard']))
        self.reference_file.start()

    def tearDown(self):
        self.reference_file.stop()
        sys.modules.pop(guard._NATIVE_NAME, None)
        if self.original_module is not None:
            sys.modules[guard._NATIVE_NAME] = self.original_module
        guard._NATIVE_BINDING = self.original_binding
        self.temporary.cleanup()

    def write_manifest(self):
        self.manifest_path.write_text(json.dumps(self.manifest))
        return {'path': str(self.manifest_path), 'sha256': sha(self.manifest_path)}

    def repin_build(self):
        self.paths['build_record'].write_text(json.dumps(self.build))
        self.manifest['files']['build_record']['sha256'] = sha(self.paths['build_record'])
        self.reference = self.write_manifest()

    def load(self):
        with patch.object(guard.sys, 'platform', 'linux'), \
                patch.object(guard.importlib.machinery, 'ExtensionFileLoader', FakeExtensionLoader):
            return guard.load_sourced_boot_guard_factory(self.reference)

    def test_plan_checks_all_pins_without_native_or_adapter_import(self):
        with patch.object(guard.importlib.util, 'spec_from_file_location', side_effect=AssertionError('no import')):
            plan = guard.plan_sourced_boot_guard(self.reference)
        self.assertFalse(plan['native_library_loaded'])
        self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['active_output_eligible'])
        self.assertFalse(plan['existing_cadence_graph_alone_sufficient'])
        self.assertEqual(plan['files'], self.manifest['files'])
        self.assertEqual(plan['selection_loader']['sha256'], sha(guard.__file__))

    def test_reference_requires_exact_path_and_explicit_digest(self):
        for value in ({'path': str(self.manifest_path)}, {**self.reference, 'extra': True},
                      {**self.reference, 'sha256': 'a'*63}, {**self.reference, 'sha256': 'A'*64}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                guard.plan_sourced_boot_guard(value)

    def test_manifest_original_digest_cannot_be_replaced_silently(self):
        self.manifest_path.write_text('{}')
        with self.assertRaisesRegex(ValueError, 'bytes changed'):
            guard.plan_sourced_boot_guard(self.reference)

    def test_schema_scope_mode_and_unknown_approval_rejected(self):
        original = copy.deepcopy(self.manifest)
        for key, value in (('schema', 'other'), ('mode', 'cached'), ('scope', 'active-output'),
                           ('approved_for_runtime', True)):
            self.manifest = copy.deepcopy(original); self.manifest[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                guard.plan_sourced_boot_guard(self.write_manifest())

    def test_all_six_files_required_and_unknown_file_rejected(self):
        original = copy.deepcopy(self.manifest)
        for key in guard._FILE_KEYS:
            self.manifest = copy.deepcopy(original); del self.manifest['files'][key]
            with self.subTest(key=key), self.assertRaises(ValueError):
                guard.plan_sourced_boot_guard(self.write_manifest())
        self.manifest = copy.deepcopy(original); self.manifest['files']['extra'] = self.reference
        with self.assertRaises(ValueError): guard.plan_sourced_boot_guard(self.write_manifest())

    def test_each_file_mutation_detected_before_import(self):
        for key in guard._FILE_KEYS:
            path = self.paths[key]; original = path.read_bytes(); path.write_bytes(original+b'changed')
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'bytes changed'):
                guard.plan_sourced_boot_guard(self.reference)
            path.write_bytes(original)

    def test_repinned_unreviewed_source_rejected(self):
        self.paths['adapter'].write_text('raise RuntimeError("unreviewed")')
        self.manifest['files']['adapter']['sha256'] = sha(self.paths['adapter'])
        with self.assertRaisesRegex(ValueError, 'Unreviewed guard source'):
            guard.plan_sourced_boot_guard(self.write_manifest())

    def test_symlink_and_relative_file_paths_rejected(self):
        link = self.root/'linked'; link.symlink_to(self.paths['cpp'])
        for name in (str(link), self.paths['cpp'].name):
            self.manifest['files']['cpp']['path'] = name
            with self.subTest(name=name), self.assertRaises(ValueError):
                guard.plan_sourced_boot_guard(self.write_manifest())

    def test_reference_guard_must_be_actual_imported_module_path(self):
        with patch.object(guard._reference, '__file__', str(self.root/'other.py')):
            with self.assertRaisesRegex(ValueError, 'reference module path'):
                guard.plan_sourced_boot_guard(self.reference)

    def test_environment_and_abi_metadata_must_match_current_process(self):
        original = copy.deepcopy(self.manifest)
        for key in guard._ENV_KEYS:
            self.manifest = copy.deepcopy(original); self.manifest['environment'][key] += ' different'
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Python/ABI/platform'):
                guard.plan_sourced_boot_guard(self.write_manifest())

    def test_library_exact_abi_filename_required(self):
        new = self.root/'wrong.so'; new.write_bytes(self.paths['library'].read_bytes())
        self.manifest['files']['library'] = {'path': str(new), 'sha256': sha(new)}
        with self.assertRaisesRegex(ValueError, 'ABI filename'):
            guard.plan_sourced_boot_guard(self.write_manifest())

    def test_build_record_source_binary_path_and_environment_bound(self):
        original = copy.deepcopy(self.build)
        for key in ('python', 'platform', 'source_sha256', 'library', 'library_sha256'):
            self.build = copy.deepcopy(original); self.build[key] += ' changed'; self.repin_build()
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'build source/binary/environment'):
                guard.plan_sourced_boot_guard(self.reference)

    def test_build_record_command_and_unknown_fields_rejected(self):
        original = copy.deepcopy(self.build)
        for key, value in (('status', 'UNBUILT'), ('command', ['wrong']), ('compiler_version', ''),
                           ('extra', 'instruction is data')):
            self.build = copy.deepcopy(original); self.build[key] = value; self.repin_build()
            with self.subTest(key=key), self.assertRaises(ValueError):
                guard.plan_sourced_boot_guard(self.reference)

    def test_duplicate_manifest_and_nonfinite_json_rejected(self):
        for raw in ('{"schema":1,"schema":2}', '{"x":NaN}'):
            self.manifest_path.write_text(raw)
            ref = {'path': str(self.manifest_path), 'sha256': sha(self.manifest_path)}
            with self.subTest(raw=raw), self.assertRaises(ValueError): guard.plan_sourced_boot_guard(ref)

    def test_execute_requires_linux_cpython_before_extension_load(self):
        with patch.object(guard.sys, 'platform', 'darwin'), \
                patch.object(guard.importlib.machinery, 'ExtensionFileLoader', side_effect=AssertionError('no import')):
            with self.assertRaisesRegex(ValueError, 'Linux CPython'):
                guard.load_sourced_boot_guard_factory(self.reference)

    def test_factory_load_opens_no_boot_fd_and_default_class_is_unchanged(self):
        original = guard._reference.BootIdentityGuard
        with patch.object(guard._reference.os, 'open', side_effect=AssertionError('no procfs')):
            factory = self.load()
        self.assertIs(guard._reference.BootIdentityGuard, original)
        self.assertEqual(factory._guard_type.__bases__, (original,))
        self.assertTrue(factory.provenance()['native_library_loaded'])
        self.assertIsNone(factory.provenance()['source_files_unchanged'])
        self.assertTrue(factory.verify()['source_files_unchanged'])

    def test_factory_preserves_owned_fd_check_cadence_lock_and_close(self):
        factory = self.load(); boot = b'388804e8-4730-4348-9ef6-519bb9480672\n'
        with patch.object(guard.sys, 'platform', 'linux'), \
                patch.object(guard._reference.os, 'open', return_value=123) as opened, \
                patch.object(guard._reference.os, 'pread', return_value=boot) as read, \
                patch.object(guard._reference.os, 'close') as closed:
            instance = factory(); instance.check(); instance.check()
            self.assertEqual(read.call_count, 3)
            self.assertTrue(all(call.args == (123, 80, 0) for call in read.call_args_list))
            opened.assert_called_once_with('/proc/sys/kernel/random/boot_id',
                                          guard._reference.os.O_RDONLY | guard._reference.os.O_CLOEXEC)
            instance.close(); instance.close(); closed.assert_called_once_with(123)
            with self.assertRaisesRegex(ValueError, 'closed'): instance.check()
            self.assertEqual(read.call_count, 3)

    def test_changed_boot_read_rejects_without_cached_success(self):
        factory = self.load(); boot = b'388804e8-4730-4348-9ef6-519bb9480672'
        for following in (b'', b'00000000-0000-0000-0000-000000000000', OSError('pread failed')):
            with self.subTest(following=following), patch.object(guard.sys, 'platform', 'linux'), \
                    patch.object(guard._reference.os, 'open', return_value=123), \
                    patch.object(guard._reference.os, 'pread', side_effect=[boot, following]), \
                    patch.object(guard._reference.os, 'close'):
                instance = factory()
                try:
                    with self.assertRaises((ValueError, OSError)): instance.check()
                finally: instance.close()

    def test_preloaded_unmanaged_or_different_native_module_rejected(self):
        for value in (None, types.ModuleType(guard._NATIVE_NAME)):
            sys.modules[guard._NATIVE_NAME] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'Existing native guard'):
                self.load()
            sys.modules.pop(guard._NATIVE_NAME)

    def test_same_verified_extension_can_be_reused(self):
        first = self.load(); second = self.load()
        self.assertIs(first._native, second._native)
        self.assertTrue(second.verify()['source_files_unchanged'])

    def test_native_module_or_symbol_replacement_rejected(self):
        factory = self.load(); original = factory._native.fresh_boot_matches
        factory._native.fresh_boot_matches = Mock()
        with self.assertRaisesRegex(ValueError, 'symbol changed'): factory.verify()
        factory._native.fresh_boot_matches = original
        sys.modules[guard._NATIVE_NAME] = types.ModuleType(guard._NATIVE_NAME)
        with self.assertRaisesRegex(ValueError, 'module/symbol changed'): factory.verify()

    def test_loaded_origin_mismatch_rejected_and_unregistered(self):
        class WrongOrigin(FakeExtensionLoader):
            def create_module(self, spec):
                module = super().create_module(spec); module.__file__ = str(self.root/'wrong.so'); return module
        WrongOrigin.root = self.root
        with patch.object(guard.sys, 'platform', 'linux'), \
                patch.object(guard.importlib.machinery, 'ExtensionFileLoader', WrongOrigin):
            with self.assertRaisesRegex(ValueError, 'origin differs'):
                guard.load_sourced_boot_guard_factory(self.reference)
        self.assertNotIn(guard._NATIVE_NAME, sys.modules)

    def test_extension_load_error_has_no_silent_fallback(self):
        with patch.object(guard.sys, 'platform', 'linux'), \
                patch.object(guard.importlib.util, 'module_from_spec', side_effect=ImportError('wrong ABI')):
            with self.assertRaisesRegex(ImportError, 'wrong ABI'):
                guard.load_sourced_boot_guard_factory(self.reference)
        self.assertNotIn(guard._NATIVE_NAME, sys.modules)

    def test_postload_file_mutation_rejects_verify_and_creation(self):
        factory = self.load()
        self.paths['library'].write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'bytes changed'): factory.verify()
        with patch.object(guard._reference.os, 'open', side_effect=AssertionError('no procfs')):
            with self.assertRaisesRegex(ValueError, 'bytes changed'): factory()

    def test_postload_adapter_binding_mutation_rejected(self):
        factory = self.load(); factory._adapter.fresh_boot_matches = Mock()
        with self.assertRaisesRegex(ValueError, 'ownership changed'): factory.verify()

    def test_adapter_uses_hashed_source_without_unpinned_pyc_cache(self):
        with patch.object(guard.importlib.machinery.SourceFileLoader, 'exec_module',
                          side_effect=AssertionError('must not execute a cache')):
            factory = self.load()
        self.assertTrue(factory.verify()['source_files_unchanged'])

    def test_postload_check_implementation_mutation_rejected(self):
        factory = self.load(); factory._guard_type.check = lambda self: None
        with self.assertRaisesRegex(ValueError, 'implementation changed'): factory.verify()

    def test_provenance_is_owned_and_cannot_mutate_factory_selection(self):
        factory = self.load(); value = factory.provenance()
        value['manifest']['sha256'] = '0'*64; value['files'].clear(); value['output_allowed'] = True
        fresh = factory.provenance()
        self.assertEqual(fresh['manifest'], self.reference)
        self.assertEqual(len(fresh['files']), 6); self.assertFalse(fresh['output_allowed'])


if __name__ == '__main__':
    unittest.main()
