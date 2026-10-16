"""Offline30Hz package boundaries, real release pins and generated installer."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import prepare_jetson_30hz_kit as kit


REPOSITORY = Path(__file__).resolve().parents[1]
BUNDLE = REPOSITORY.parent / 'RS05_30Hz_Model_20261013'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Robot30KitTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name).resolve()
        self.repo = self.base / 'repo'
        self.repo.mkdir()
        subprocess.run(['git', 'init', '-q', str(self.repo)], check=True)
        package = self.repo / 'runtime/singularitydog_hw'; package.mkdir(parents=True)
        (package / '__init__.py').write_text('')
        for name in ('policy_model_30hz.py', 'policy_clock.py'):
            shutil.copyfile(REPOSITORY / 'runtime/singularitydog_hw' / name, package / name)
        (self.repo / 'tools').mkdir()
        (self.repo / 'tools/untracked.py').write_text('candidate = True\n')
        for relative in kit.REFERENCE_FILES.values():
            target = self.repo / relative; target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(REPOSITORY / relative, target)
        (package / 'legacy.so').write_bytes(b'tracked historical native binary\0')
        subprocess.run(['git', 'add', 'runtime'], cwd=self.repo, check=True)
        self.bundle = self.base / 'model'
        shutil.copytree(BUNDLE, self.bundle)
        self.output = self.base / 'prepared'
        self.target = self.base / 'installed'

    def tearDown(self):
        self.temp.cleanup()

    def build(self):
        self.result = kit.prepare(self.repo, self.bundle, self.output, build=True)
        self.manifest = self.output / 'robot30-manifest.json'
        self.data = json.loads(self.manifest.read_bytes())
        return self.result

    def invoke(self, *options, digest=None, target=None):
        return subprocess.run([sys.executable, '-I', '-B', str(self.output / 'verify-robot30-kit.py'),
            '--manifest', str(self.manifest), '--manifest-sha256', digest or sha(self.manifest),
            '--destination', str(target or self.target), *options],
            capture_output=True, text=True, timeout=40)

    def test_plan_is_file_only_and_has_no_output_grants(self):
        original_import = __import__
        def no_torch(name, *args, **kwargs):
            if name == 'torch' or name.startswith(('torch.', 'serial', 'socket')):
                raise AssertionError('Hardware or Torch import in file-only plan: ' + name)
            return original_import(name, *args, **kwargs)
        with mock.patch('builtins.__import__', side_effect=no_torch):
            result = kit.prepare(self.repo, self.bundle, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(result['status'], 'PLAN_OFFLINE30HZ_KIT')
        self.assertEqual(result['policy_hz'], 30)
        self.assertEqual(result['critical_input_to_all_host_write_budget_ns'], 20_000_000)
        self.assertEqual(result['type1_reply_cycle_budget_ns'], 33_333_334)
        self.assertEqual(result['proxy_exchange_budget_ns'], 33_333_334)
        self.assertEqual(result['proxy_input_to_all_host_write_budget_ns'], 20_000_000)
        self.assertEqual(result['diagnostic_proxy_separate_write_deadline_abi_required'], 1)
        self.assertEqual(result['evaluation']['unresolved_cases'], ['left', 'stop'])
        for name in kit.NO_GRANTS:
            self.assertIs(result[name], False)

    def test_altered_release_and_source_closure_rejected_before_output(self):
        for relative in ('model/policy_controller74_30hz.pt', 'source/timing_contract.py'):
            with self.subTest(relative=relative):
                path = self.bundle / relative; original = path.read_bytes()
                path.write_bytes(original + b'\n')
                with self.assertRaisesRegex(ValueError, 'SHA256 differs'):
                    kit.prepare(self.repo, self.bundle, self.output, build=True)
                self.assertFalse(self.output.exists()); path.write_bytes(original)

    def test_build_verify_install_matches_source_and_whole_model_bundle(self):
        result = self.build()
        self.assertEqual(sha(self.manifest), result['manifest_sha256'])
        self.assertNotIn('kit/runtime/singularitydog_hw/legacy.so', self.data['files'])
        source_manifest = json.loads((self.output / 'manifest.json').read_bytes())
        self.assertTrue(any(r['path'].endswith('legacy.so') for r in source_manifest['skipped']))
        expected_model = {p.relative_to(self.bundle).as_posix() for p in self.bundle.rglob('*') if p.is_file()}
        self.assertEqual({n[len('model-bundle/'):] for n in self.data['files'] if n.startswith('model-bundle/')},
                         expected_model)
        response = self.invoke()
        self.assertEqual(response.returncode, 0, response.stderr)
        self.assertEqual(json.loads(response.stdout)['status'], 'PLAN_VERIFIED_OFFLINE30HZ_KIT')
        self.assertFalse(self.target.exists())
        response = self.invoke('--install')
        self.assertEqual(response.returncode, 0, response.stderr)
        report = json.loads(response.stdout)
        self.assertEqual(report['status'], 'INSTALLED_SOURCE_AND30HZ_CANDIDATE')
        for flag in kit.NO_GRANTS:
            if flag in report:
                self.assertIs(report[flag], False)
        for name, row in self.data['files'].items():
            installed_name = name[4:] if name.startswith('kit/') else name
            if name.startswith(('kit/', 'model-bundle/', 'references/')):
                self.assertEqual(sha(self.target / installed_name), row['sha256'])
        self.assertNotEqual(self.invoke('--install').returncode, 0)

    def test_cpu_check_is_explicit_and_never_installs_or_opens_hardware(self):
        self.build()
        result = self.invoke('--cpu-check')
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report['model_loaded'])
        self.assertIsNotNone(report['cpu_model_check'])
        self.assertFalse(self.target.exists())
        for name in ('hardware_opened', 'output_allowed', 'approved_for_runtime',
                     'current_power_or_pose_confirmed', 'hardware_measurement_completed'):
            self.assertIs(report[name], False)

    def test_tampered_archive_model_source_or_verifier_does_not_create_target(self):
        self.build()
        for name in ('source.tar.gz', 'model-bundle/model/actor74_30hz.pt',
                     'kit/runtime/singularitydog_hw/policy_clock.py', 'verify-robot30-kit.py'):
            with self.subTest(name=name):
                path = self.output / name; original = path.read_bytes()
                path.write_bytes(original + b'\n')
                response = self.invoke('--install')
                self.assertNotEqual(response.returncode, 0)
                self.assertFalse(self.target.exists()); path.write_bytes(original)
        self.assertNotEqual(self.invoke('--install', digest='0' * 64).returncode, 0)

    def test_manifest_rebinding_cannot_grant_output_or_weaken_clock(self):
        self.build(); original = self.manifest.read_bytes()
        changes = (('output_allowed', True), ('previous_50hz_qualification_transferred', True),
                   ('critical_input_to_all_host_write_budget_ns', 33_333_334),
                   ('type1_reply_cycle_budget_ns', 40_000_000),
                   ('proxy_exchange_budget_ns', 20_000_000),
                   ('proxy_input_to_all_host_write_budget_ns', 30_000_000),
                   ('diagnostic_proxy_separate_write_deadline_abi_required', 0),
                   ('diagnostic_proxy_separate_write_deadline_abi_required', True))
        for key, value in changes:
            with self.subTest(key=key):
                data = json.loads(original); data[key] = value
                self.manifest.write_text(json.dumps(data, sort_keys=True))
                self.assertNotEqual(self.invoke('--install').returncode, 0)
                self.assertFalse(self.target.exists())
        self.manifest.write_bytes(original)

    def test_symlink_model_output_and_existing_or_git_install_rejected(self):
        linked = self.base / 'alias'; linked.symlink_to(self.bundle, target_is_directory=True)
        with self.assertRaises(ValueError):
            kit.prepare(self.repo, linked, self.output)
        with self.assertRaises(ValueError):
            kit.prepare(self.repo, self.bundle, self.repo / 'inside')
        self.build()
        self.assertNotEqual(self.invoke('--install', target=self.repo / 'inside').returncode, 0)
        self.target.mkdir(); (self.target / 'keep').write_text('keep')
        self.assertNotEqual(self.invoke('--install').returncode, 0)
        self.assertEqual((self.target / 'keep').read_text(), 'keep')

    def test_model_changes_during_build_remove_only_fresh_output(self):
        real = kit.model_inventory; calls = 0
        def changing(*args):
            nonlocal calls
            calls += 1
            if calls == 2:
                (self.bundle / 'report.md').write_text('changed evidence')
            return real(*args)
        with mock.patch.object(kit, 'model_inventory', side_effect=changing):
            with self.assertRaisesRegex(ValueError, 'changed during packaging'):
                kit.prepare(self.repo, self.bundle, self.output, build=True)
        self.assertFalse(self.output.exists()); self.assertTrue(self.repo.is_dir())

    def test_foreign_output_appearing_after_plan_is_preserved(self):
        real_dependencies = kit.dependencies
        def injected_dependencies():
            source, model, clock = real_dependencies()
            real_prepare = source.prepare
            def appeared(repository, output, *, build=False):
                if build:
                    self.output.mkdir()
                    (self.output / 'foreign-keep').write_text('another process owns this')
                return real_prepare(repository, output, build=build)
            source.prepare = appeared
            return source, model, clock
        with mock.patch.object(kit, 'dependencies', side_effect=injected_dependencies):
            with self.assertRaisesRegex(ValueError, 'Fresh output'):
                kit.prepare(self.repo, self.bundle, self.output, build=True)
        self.assertEqual((self.output / 'foreign-keep').read_text(), 'another process owns this')

    def test_replaced_owned_output_is_not_removed_by_composed_cleanup(self):
        real = kit.model_inventory; calls = 0
        original_output = self.base / 'moved-owned-output'
        def replaced(*args):
            nonlocal calls
            calls += 1
            if calls == 2:
                self.output.rename(original_output)
                self.output.mkdir()
                (self.output / 'foreign-keep').write_text('replacement owner')
                raise RuntimeError('replacement injected')
            return real(*args)
        with mock.patch.object(kit, 'model_inventory', side_effect=replaced):
            with self.assertRaisesRegex(RuntimeError, 'replacement injected'):
                kit.prepare(self.repo, self.bundle, self.output, build=True)
        self.assertEqual((self.output / 'foreign-keep').read_text(), 'replacement owner')
        self.assertTrue((original_output / 'source.tar.gz').is_file())


if __name__ == '__main__':
    unittest.main()
