"""Packaging contracts use synthetic snapshot bytes, no robot or real UID files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from test_prepare_overnight_bundle import synthetic_inputs, write_fixture
from singularitydog_hw import policy_live_profile as live


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name+'.py'))
    tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)
    return tool


tool = load_tool('prepare_policy_output_bundle')
diagnostic = load_tool('dog_tomorrow')


class PolicyOutputBundleTests(unittest.TestCase):
    def build_fixture(self, root, *, with_validation=False):
        repo, home, profile, uids = synthetic_inputs(root)
        for name in (*tool.ACTIVE_SOURCE_PATHS,*tool.GROUND_SOURCE_PATHS):
            write_fixture(repo, name, b'// synthetic source\n' if name.endswith('.cpp') else b'# synthetic source\n')
        write_fixture(repo, tool.DESIGN_PATH, b'Synthetic supported-policy design\n')
        write_fixture(repo, tool.GROUND_RUNBOOK_PATH, b'Synthetic ground review runbook\n')
        if with_validation: write_fixture(repo, tool.VALIDATION_PATH, b'Synthetic offline validation\n')
        for name in tool.PRELOAD_DOC_PATHS:write_fixture(repo,name,b'Synthetic current preparation runbook\n')
        write_fixture(repo, 'runtime/experiments/native_active_transport/libdog_active_transport.so', b'MAC HOST BINARY DO NOT COPY')
        output = root/'private-policy-kit'
        with patch.object(tool, 'ROOT', repo), patch.object(tool.overnight, 'ROOT', repo), \
             patch.object(tool.overnight, 'history_profile', return_value=(profile, {}, {}, {}, uids)):
            result = tool.build(home, output)
        return output, result, home

    def test_unapproved_template_preserves_empty_axes_and_pending_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            output, result, _ = self.build_fixture(Path(folder))
            candidate = json.loads((output/tool.PROFILE_PATH).read_text())
            self.assertFalse(candidate['approved_for_supported_policy_output'])
            self.assertIsNone(candidate['review']); self.assertTrue(candidate['blockers'])
            self.assertEqual(set(candidate['axes']), {str(i) for i in range(1,13)})
            self.assertTrue(all(value is None for axis in candidate['axes'].values() for value in axis.values()))
            self.assertFalse(result['supported_policy_profile_approved'])
            self.assertFalse(result['hardware_accessed'])

    def test_new_config_paths_and_source_pins_are_self_contained(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _, _ = self.build_fixture(Path(folder))
            config = json.loads((output/'kit-config.json').read_text())
            policy = config['supported_policy_output']
            self.assertEqual(policy['default_mode'], 'PLAN_ONLY')
            self.assertTrue(policy['diagnostic_entry_actions_unchanged'])
            self.assertEqual(policy['entry_module'], 'singularitydog_hw.policy_output')
            self.assertEqual(policy['source_sha256'], {name: hashlib.sha256((output/name).read_bytes()).hexdigest()
                                                      for name in tool.ACTIVE_SOURCE_PATHS})
            for key in ('profile_template','active_transport_source','active_transport_build','design'):
                path = Path(policy[key]); self.assertFalse(path.is_absolute())
                self.assertTrue((output/path).is_file())
            self.assertFalse((output/policy['active_transport_library']).exists())
            self.assertFalse(policy['active_transport_binary_included'])
            self.assertTrue(policy['build_active_library_on_target_required'])
            self.assertEqual(policy['profile_schema'], live.SCHEMA_V2)
            preload=config['supported_preload']
            self.assertFalse(preload['template_included']);self.assertIsNone(preload['profile_template'])
            self.assertFalse(preload['approved_for_supported_policy_output'])
            self.assertEqual(preload['default_mode'],'PLAN_ONLY')
            self.assertEqual(preload['runbooks'],list(tool.PRELOAD_DOC_PATHS))
            for name in preload['runbooks']:self.assertTrue((output/name).is_file())
            for key in ('expected_uids','angle_profile','calibration','mount','bundle'):
                self.assertTrue((output/config[key]).exists())

    def test_explicit_v3_kit_pins_copied_cadence_sources_and_stays_unapproved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            repo, home, profile, uids = synthetic_inputs(root)
            for name in (*tool.ACTIVE_SOURCE_PATHS, *tool.GROUND_SOURCE_PATHS,
                         tool.PRELOAD_SOURCE_PATH,
                         *('runtime/'+name for name in live.CADENCE_SOURCE_PATHS)):
                write_fixture(repo, name, b'// synthetic source\n' if name.endswith('.cpp') else b'# synthetic source\n')
            write_fixture(repo, tool.DESIGN_PATH, b'Synthetic policy output design\n')
            write_fixture(repo, tool.GROUND_RUNBOOK_PATH, b'Synthetic ground review runbook\n')
            copied = {name: hashlib.sha256((repo/'runtime'/name).read_bytes()).hexdigest()
                      for name in live.CADENCE_SOURCE_PATHS}
            output = root/'private-v3-kit'
            with patch.object(tool, 'ROOT', repo), patch.object(tool.overnight, 'ROOT', repo), \
                 patch.object(tool.overnight, 'history_profile', return_value=(profile, {}, {}, {}, uids)), \
                 patch.object(live, 'cadence_source_hashes', side_effect=lambda profile=None:
                     {name:hashlib.sha256((repo/'runtime'/name).read_bytes()).hexdigest()
                      for name in live.cadence_source_paths(profile)}):
                result = tool.build(home, output, profile_schema=live.SCHEMA_V3)
            candidate = json.loads((output/tool.PROFILE_PATH).read_text())
            config = json.loads((output/'kit-config.json').read_text())['supported_policy_output']
            self.assertEqual(candidate['schema'], live.SCHEMA_V3)
            self.assertEqual(config['profile_schema'], live.SCHEMA_V3)
            self.assertEqual(result['profile_schema'], live.SCHEMA_V3)
            self.assertEqual(candidate['cadence_source_sha256'], copied)
            self.assertFalse(candidate['approved_for_supported_policy_output'])
            self.assertFalse(config['approved_for_supported_policy_output'])
            self.assertIsNone(candidate['review'])
            self.assertTrue(candidate['blockers'])
            preload_config=json.loads((output/'kit-config.json').read_text())['supported_preload']
            self.assertTrue(preload_config['template_included'])
            self.assertFalse(preload_config['support_removal_authorized'])
            self.assertFalse(preload_config['walking_authorized']);self.assertFalse(preload_config['automatic_retry'])
            preload_raw=(output/preload_config['profile_template']).read_bytes()
            self.assertEqual(hashlib.sha256(preload_raw).hexdigest(),preload_config['profile_template_sha256'])
            preload=json.loads(preload_raw)
            self.assertEqual(preload['diagnostic_timing_acceptance'],live.SUPPORTED_PRELOAD_5S)
            self.assertFalse(preload['approved_for_supported_policy_output']);self.assertIsNone(preload['review'])
            self.assertTrue(preload['blockers'])
            self.assertTrue(all(v is None for row in preload['axes'].values() for v in row.values()))
            self.assertTrue(all(v is None for ref in preload['artifacts'].values() for v in ref.values()))
            self.assertEqual(preload['cadence_source_sha256'],{
                name:hashlib.sha256((output/'runtime'/name).read_bytes()).hexdigest()
                for name in live.cadence_source_paths(preload)})
            diagnostic.verify_kit(output)

    def test_v3_rejects_source_mismatch_before_publication(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            repo, home, profile, uids = synthetic_inputs(root)
            for name in (*tool.ACTIVE_SOURCE_PATHS, *tool.GROUND_SOURCE_PATHS,
                         tool.PRELOAD_SOURCE_PATH,
                         *('runtime/'+name for name in live.CADENCE_SOURCE_PATHS)):
                write_fixture(repo, name, b'// synthetic source\n' if name.endswith('.cpp') else b'# synthetic source\n')
            write_fixture(repo, tool.DESIGN_PATH, b'Synthetic policy output design\n')
            write_fixture(repo, tool.GROUND_RUNBOOK_PATH, b'Synthetic ground review runbook\n')
            output = root/'private-v3-kit'
            with patch.object(tool, 'ROOT', repo), patch.object(tool.overnight, 'ROOT', repo), \
                 patch.object(tool.overnight, 'history_profile', return_value=(profile, {}, {}, {}, uids)), \
                 patch.object(live, 'cadence_source_hashes', return_value={name:'0'*64 for name in live.CADENCE_SOURCE_PATHS}):
                with self.assertRaisesRegex(ValueError, 'Cadence source changed'):
                    tool.build(home, output, profile_schema=live.SCHEMA_V3)
            self.assertFalse(output.exists())

    def test_original_manifest_verifier_accepts_every_final_file_and_detects_change(self):
        with tempfile.TemporaryDirectory() as folder:
            output, result, _ = self.build_fixture(Path(folder))
            manifest = json.loads((output/'kit-manifest.json').read_text())
            self.assertEqual(manifest['schema'], 'private-overnight-kit-v1')
            expected = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in output.rglob('*') if path.is_file() and path.name != 'kit-manifest.json'}
            self.assertEqual(manifest['files'], expected)
            self.assertEqual(result['file_count'], len(expected))
            diagnostic.verify_kit(output)
            (output/tool.PROFILE_PATH).write_text('{}')
            with self.assertRaisesRegex(ValueError, 'changed'):
                diagnostic.verify_kit(output)

    def test_private_modes_and_no_host_binaries(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _, home = self.build_fixture(Path(folder))
            for path in (output, *output.rglob('*')):
                self.assertEqual(path.stat().st_mode & 0o777, 0o700 if path.is_dir() else 0o600)
                if path.is_file(): self.assertNotIn(path.suffix, ('.so','.dylib','.dll','.pyc'))
            self.assertEqual((output/'inputs/policy/model_149.pt').read_bytes(),
                             (home/'singularitydog-policy-shadow/20260921-r1/model_149.pt').read_bytes())

    def test_validation_is_optional_but_included_and_hashed_when_present(self):
        for present in (False, True):
            with self.subTest(present=present), tempfile.TemporaryDirectory() as folder:
                output, result, _ = self.build_fixture(Path(folder), with_validation=present)
                config = json.loads((output/'kit-config.json').read_text())
                self.assertEqual(result['validation_included'], present)
                self.assertEqual((output/tool.VALIDATION_PATH).exists(), present)
                self.assertEqual(config['supported_policy_output']['validation'], tool.VALIDATION_PATH if present else None)
                diagnostic.verify_kit(output)

    def test_all_ground_stage_templates_remain_unapproved_and_without_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            output, result, _ = self.build_fixture(Path(folder))
            config = json.loads((output/'kit-config.json').read_text())['ground_trials']
            self.assertEqual(set(config['stage_templates']), {'supported_stance','partial_load','stand','walk'})
            self.assertFalse(config['approved_for_ground_trial']); self.assertFalse(result['ground_trials_approved'])
            self.assertEqual(config['default_mode'],'PLAN_ONLY')
            self.assertTrue(config['prior_hardware_reviews_required'])
            for stage,ref in config['stage_templates'].items():
                contents=(output/ref['path']).read_bytes(); data=json.loads(contents)
                self.assertEqual(hashlib.sha256(contents).hexdigest(),ref['sha256'])
                self.assertEqual(data['stage'],stage);self.assertFalse(data['approved_for_ground_trial'])
                self.assertIsNone(data['review']);self.assertTrue(data['blockers'])
                self.assertIsNone(data['assembly_id']);self.assertIsNone(data['base_profile_sha256'])
                for reference in data['prior_evaluations'].values():
                    self.assertIsNone(reference['path']);self.assertIsNone(reference['sha256'])
            physical=json.loads((output/config['physical_review_template']['path']).read_text())
            self.assertEqual(physical['decision'],'UNREVIEWED')
            self.assertTrue(all(v is None for v in physical['observations'].values()))
            self.assertEqual(config['source_sha256'], {n:hashlib.sha256((output/n).read_bytes()).hexdigest() for n in tool.GROUND_SOURCE_PATHS})
            self.assertTrue((output/config['runbook']).is_file()); diagnostic.verify_kit(output)

    def test_prior_package_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _, home = self.build_fixture(Path(folder))
            before = (output/'kit-manifest.json').read_bytes()
            with self.assertRaisesRegex(ValueError, 'must be new'):
                tool.build(home, output)
            self.assertEqual((output/'kit-manifest.json').read_bytes(), before)

    def test_git_output_rejected_before_reading_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'.git').mkdir()
            with patch.object(tool.overnight, 'build') as old:
                with self.assertRaisesRegex(ValueError, 'outside Git'):
                    tool.build('/DO_NOT_READ/snapshot', root/'private-kit')
                old.assert_not_called()

    def test_partial_publish_does_not_leave_a_valid_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            def interrupted_copy(source, destination, **kwargs):
                Path(destination).mkdir()
                (Path(destination)/'partial.txt').write_text('incomplete')
                raise OSError('synthetic interrupted copy')
            with patch.object(tool.shutil, 'copytree', side_effect=interrupted_copy):
                with self.assertRaisesRegex(OSError, 'interrupted copy'):
                    self.build_fixture(root)
            self.assertFalse((root/'private-policy-kit/kit-manifest.json').exists())
            self.assertFalse(list(root.glob('.policy-kit-stage-*')))


if __name__ == '__main__':
    unittest.main()
