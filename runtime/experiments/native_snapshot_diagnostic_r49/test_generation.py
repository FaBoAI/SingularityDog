"""Source inverse, bytecode bootstrap and unapproved artifact binding tests."""
import ast
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import py_compile
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

from . import snapshot_generate as generate
from . import snapshot_loader as loader
from . import snapshot_build as build
from .test_selection import HERE, ROOT, make_source_bundle


def functions(raw):
    return {n.name: ast.dump(n, include_attributes=False) for n in ast.parse(raw).body
            if isinstance(n, (ast.FunctionDef, ast.ClassDef))}


class GenerationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(); self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name).resolve()
        self.bundle = make_source_bundle(self.folder)
        self.manifest = json.loads((self.bundle / 'manifest.json').read_bytes())

    def test_observer_import_only_inverse_preserves_every_function_and_class(self):
        before = (ROOT / 'runtime/singularitydog_hw/policy_observer.py').read_bytes()
        after, proof = generate.derive_observer(before)
        self.assertEqual(after, (self.bundle / 'copy_observer.py').read_bytes())
        self.assertEqual(functions(before), functions(after))
        self.assertEqual(proof['changed_functions'], [])
        self.assertTrue(proof['inverse_bytes_exact']); self.assertTrue(proof['inverse_ast_exact'])
        for name in ('_snapshot_copy', '_digest', 'StatefulPolicyObserver'):
            self.assertEqual(functions(before)[name], functions(after)[name])

    def test_benchmark_main_only_inverse_retains_collect_and_fk_selection(self):
        before = (self.bundle / 'baseline_fk_benchmark.py').read_bytes()
        after, proof = generate.derive_benchmark(before)
        a, b = functions(before), functions(after)
        self.assertEqual([name for name in a if a[name] != b[name]], ['main'])
        self.assertEqual(a['collect'], b['collect']); self.assertTrue(proof['fk_model_selection_unchanged'])
        for line in before.splitlines():
            if b'load_diagnostic_verified' in line or b"report['fk_cache_model_source']" in line:
                self.assertIn(line, after.splitlines())
        self.assertIn(b'_snapshot_diagnostic.restore(report,snapshot_proof,snapshot_selection)', after)

    def test_unknown_sources_and_duplicate_anchors_rejected(self):
        for method, path in ((generate.derive_observer, self.bundle / 'copy_observer.py'),
                             (generate.derive_benchmark, self.bundle / 'fk_cache_diagnostic_benchmark.py'),
                             (generate.derive_child, self.bundle / 'diagnostic_child.py')):
            with self.subTest(method=method.__name__), self.assertRaises(ValueError):
                method(path.read_bytes())
        with self.assertRaises(ValueError): generate.once('repeat repeat', 'repeat', 'once')
        # The old local fixture has the same thirteen member hashes but a
        # different declared target path. It is not the frozen deployed R47.
        path = self.folder / 'r47/manifest.json'
        data = json.loads(path.read_bytes())
        data['target_bundle_path'] = '/home/jetson/singularitydog-logs/validation-20261006-r1/fk-stop-diagnostic-source-bundle-r47'
        path.write_text(json.dumps(data, indent=2) + '\n')
        self.assertEqual(loader.sha(path.read_bytes()),
                         'd1d445a13418af4c103ae704f06865f17eb10473cd96c70bc4d7c2f1a52cacc8')
        output = self.folder / 'rejected-fixture-source'
        with self.assertRaises(ValueError):
            generate.generate_bundle(path.parent, ROOT / 'runtime/singularitydog_hw/policy_observer.py',
                                     HERE, output, target_bundle=output, baseline_kit=ROOT)
        self.assertFalse(output.exists())

    def test_source_bundle_exact_inventory_and_unchanged_r47_dependencies(self):
        self.assertEqual(set(self.manifest['files']), generate.NAMES); self.assertEqual(len(generate.NAMES), 23)
        for name, pin in self.manifest['files'].items():
            self.assertEqual(loader.sha((self.bundle / name).read_bytes()), pin)
        for name, pin in generate.R47_FILES.items():
            if name not in ('diagnostic_child.py', 'fk_cache_diagnostic_benchmark.py'):
                self.assertEqual(self.manifest['files'][name], pin)
        self.assertNotIn('native_policy_output_rows', '\n'.join(self.manifest['files']))

    def test_bundle_and_builder_plan_make_no_compiler_native_or_output_calls(self):
        target = self.folder / 'never-built'
        with mock.patch.object(build.subprocess, 'run', side_effect=AssertionError('Compiler started')), \
                mock.patch.object(loader.importlib.util, 'spec_from_file_location', side_effect=AssertionError('Native loaded')):
            plan = build.build_plan(self.bundle / 'manifest.json',
                loader.sha((self.bundle / 'manifest.json').read_bytes()),
                ROOT / 'runtime/singularitydog_hw/policy_observer.py', target)
        self.assertFalse(target.exists()); self.assertFalse(plan['compiler_started'])
        self.assertFalse(plan['native_library_loaded']); self.assertFalse(plan['output_allowed'])

    def test_fk_rebinding_changes_only_five_integration_references(self):
        refs = {name: {'path': '/frozen/' + name, 'sha256': generate.R47_FILES[
            'native_policy_overnight/target_tail_fk_cache/' + name if name == 'diagnostic_loader.py' else name]}
            for name in ('diagnostic_loader.py','diagnostic_support.py','diagnostic_generate.py',
                         'diagnostic_child.py','fk_cache_diagnostic_benchmark.py')}
        original = {'integration_sources': refs, 'references': {'model': {'path': '/model', 'sha256': 'a' * 64}},
                    'unchanged': {'all_model_library_evidence': [True, False, 'exact']}}
        result = build.rebind_fk_manifest(original, self.manifest, self.bundle)
        self.assertEqual(original['references'], result['references']); self.assertEqual(original['unchanged'], result['unchanged'])
        self.assertEqual(set(result['integration_sources']), set(refs))
        self.assertNotEqual(result['integration_sources'], refs)
        result['unchanged']['all_model_library_evidence'].clear()
        self.assertEqual(len(original['unchanged']['all_model_library_evidence']), 3)
        original['integration_sources']['diagnostic_child.py']['sha256'] = '0' * 64
        with self.assertRaises(ValueError): build.rebind_fk_manifest(original, self.manifest, self.bundle)

    def test_child_authenticated_bootstrap_ignores_valid_stale_pyc_and_restores_scope(self):
        path = self.bundle / 'diagnostic_child.py'
        module = ModuleType('_snapshot_r49_child_test'); module.__file__ = str(path)
        exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
        for name in ('__init__.py', 'snapshot_loader.py', 'snapshot_generate.py', 'snapshot_support.py', 'snapshot_build.py'):
            source = self.bundle / 'native_snapshot_diagnostic_r49' / name
            raw = source.read_bytes(); metadata = source.stat()
            poisoned = b"raise AssertionError('stale bytecode executed')\n"
            self.assertLessEqual(len(poisoned), len(raw))
            try:
                source.write_bytes(poisoned + b' ' * (len(raw) - len(poisoned)))
                os.utime(source, ns=(metadata.st_atime_ns, metadata.st_mtime_ns))
                py_compile.compile(str(source), doraise=True, invalidation_mode=py_compile.PycInvalidationMode.TIMESTAMP)
            finally:
                source.write_bytes(raw); os.utime(source, ns=(metadata.st_atime_ns, metadata.st_mtime_ns))
        actual_execute = module.execute_module
        seen = []
        def execute(path, name, expected):
            if name == 'singularitydog_hw._explicit_fk_stop_diagnostic':
                def main(arguments):
                    selected = sys.modules['native_snapshot_diagnostic_r49.snapshot_loader']
                    self.assertEqual(selected.sha(b'actual byte source'), loader.sha(b'actual byte source'))
                    self.assertEqual(Path(selected.__file__), self.bundle / 'native_snapshot_diagnostic_r49/snapshot_loader.py')
                    self.assertNotIn(loader.PRIVATE_NATIVE, sys.modules)
                    seen.append(arguments); return 0
                return SimpleNamespace(main=main)
            return actual_execute(path, name, expected)
        before = sys.getswitchinterval(); paths = list(sys.path); modules = set(sys.modules)
        with mock.patch.object(module, 'verify', return_value=(self.manifest, ROOT)), \
                mock.patch.object(module, 'require_fresh_interpreter'), \
                mock.patch.object(module, 'bind_replay', return_value=None), \
                mock.patch.object(module, 'execute_module', side_effect=execute), \
                mock.patch.object(module.sys, 'setswitchinterval', side_effect=AssertionError('PLAN changed scheduling')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(module.main(['--bundle', str(self.bundle), '--bundle-sha256',
                loader.sha((self.bundle / 'manifest.json').read_bytes()), '--interval-us', '100', '--']), 0)
        self.assertEqual(seen, [[]])
        self.assertEqual(before, sys.getswitchinterval()); self.assertEqual(paths, sys.path)
        self.assertFalse(any(name.startswith('native_snapshot_diagnostic_r49') for name in set(sys.modules) - modules))


if __name__ == '__main__':
    unittest.main()
