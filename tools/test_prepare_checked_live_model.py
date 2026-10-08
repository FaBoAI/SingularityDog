"""Pure producer control fixtures; injected loader proof is not robot evidence."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import prepare_checked_live_model as tool


class ProducerTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name).resolve()
        self.base=tool.live.template(schema=tool.live.SCHEMA_V3)
        self.base.update(native_target_fk_cache=True,approved_for_supported_policy_output=True,
            review={'reviewer':'SYNTHETIC original'},blockers=[],period_ms=20,hard_cycle_ms=20,
            request_gap_us=900,request_window=3)
        self.base['artifacts'].update({key:dict(path='/SYNTHETIC/'+key,sha256='1'*64) for key in
            ('target_fk_manifest','scalar_step_manifest','model_manifest')})
        self.dependencies=dict(schema=tool.DEPENDENCIES_SCHEMA,references={key:
            dict(path='/SYNTHETIC/'+key,sha256='2'*64) for key in tool.checked.REFS})
        for key in ('base','dependencies'):(self.root/(key+'.json')).write_bytes(tool.encoded(getattr(self,key)))
        self.args=SimpleNamespace(base_profile=str(self.root/'base.json'),
            base_profile_sha256=hashlib.sha256((self.root/'base.json').read_bytes()).hexdigest(),
            dependencies=str(self.root/'dependencies.json'),
            dependencies_sha256=hashlib.sha256((self.root/'dependencies.json').read_bytes()).hexdigest(),
            output=str(self.root/'fresh'),prepare=False)
        # Numerical/admission behavior is covered separately by the real scope
        # tests. These intentionally injected documents test producer lifetime.
        for target in (tool.live._structure,tool.live._settings,tool.checked.scope):
            name=target.__name__;owner=tool.checked if name=='scope' else tool.live
            helper=patch.object(owner,name);helper.start();self.addCleanup(helper.stop)
        helper=patch.object(tool.checked,'_document_plan',return_value={'SYNTHETIC_ONLY':True})
        helper.start();self.addCleanup(helper.stop)

    def test_default_plan_pins_original_and_retains_all_numerical_current_artifacts(self):
        plan,manifest,draft=tool.inspect(self.args)
        self.assertFalse(Path(self.args.output).exists());self.assertFalse(plan['selected_manifest_published'])
        self.assertTrue(plan['file_only_loader_acceptance_pending'])
        self.assertFalse(draft['approved_for_supported_policy_output']);self.assertIsNone(draft['review'])
        self.assertTrue(draft['blockers']);self.assertIsNone(manifest['physical_future_observations'])
        for key in set(self.base)-{'review','blockers','approved_for_supported_policy_output','artifacts','cadence_source_sha256'}:
            self.assertEqual(self.base[key],draft[key])
        self.assertEqual(self.base['artifacts'],{k:v for k,v in draft['artifacts'].items() if k!='checked_model_manifest'})

    def test_changed_pinned_input_missing_reference_and_existing_destination_reject(self):
        (self.root/'base.json').write_bytes(tool.encoded({**self.base,'hard_cycle_ms':21}))
        with self.assertRaisesRegex(tool.live.ProfileError,'SHA256'):tool.inspect(self.args)
        (self.root/'base.json').write_bytes(tool.encoded(self.base))
        incomplete=copy.deepcopy(self.dependencies);del incomplete['references']['checked_model']
        (self.root/'dependencies.json').write_bytes(tool.encoded(incomplete))
        self.args.dependencies_sha256=hashlib.sha256((self.root/'dependencies.json').read_bytes()).hexdigest()
        with self.assertRaisesRegex(tool.live.ProfileError,'dependencies'):tool.inspect(self.args)
        (self.root/'dependencies.json').write_bytes(tool.encoded(self.dependencies))
        self.args.dependencies_sha256=hashlib.sha256((self.root/'dependencies.json').read_bytes()).hexdigest()
        Path(self.args.output).mkdir()
        with self.assertRaisesRegex(tool.live.ProfileError,'Fresh'):tool.inspect(self.args)

    def test_prepare_rejected_complete_loader_removes_only_new_output(self):
        self.args.prepare=True
        with patch.object(tool.live,'load_profile',side_effect=tool.live.ProfileError('SYNTHETIC loader rejection')):
            with self.assertRaisesRegex(tool.live.ProfileError,'loader rejection'):tool.run(self.args)
        self.assertFalse(Path(self.args.output).exists())
        self.assertEqual(json.loads((self.root/'base.json').read_text()),self.base)

    def test_help_imports_no_torch_or_hardware_and_never_creates_destination(self):
        script=Path(tool.__file__).absolute()
        code='import runpy,sys;sys.argv=[sys.argv[1],"--help"];\ntry:runpy.run_path(sys.argv[0],run_name="__main__")\nexcept SystemExit as e:assert e.code==0\nassert "torch" not in sys.modules\n'
        result=subprocess.run([sys.executable,'-B','-c',code,str(script)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr);self.assertFalse(Path(self.args.output).exists())


if __name__=='__main__':unittest.main()
