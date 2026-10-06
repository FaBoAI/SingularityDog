"""Pure source/PLAN tests; no Torch, compilation, native load or devices."""
import ast
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
import shutil
import subprocess

from . import diagnostic_generate as generate
from . import diagnostic_child as child
from . import diagnostic_support as support
from singularitydog_hw import native_pipeline_benchmark as original_benchmark

BASELINE=Path(original_benchmark.__file__).absolute()


def args():
    return types.SimpleNamespace(fk_cache_manifest='/test/fk.json',fk_cache_manifest_sha256='a'*64,
        mode='stop-proxy',supported_disabled=True,cycles=501,startup_cycle_allowance=1,
        request_gap_us=900,request_window=3,voltage_max_v=42,v3_voltage_proxy=True,v3_voltage_overlap=True,
        v3_voltage_validation_overlap=True,v3_voltage_fast_pipeline=True,prepare_voltage_before_feedback_publication=True,
        record_storage='trace',absolute_epoch_cadence=True,release_spin_us=500,main_thread_cpu=4,
        exclude_policy_cpu_from_workers=True,single_thread_math=True,require_pinned_fast_model=True,
        provenance_mode='supported-policy-probe-2s-rare-jitter-v1',power_epoch='explicit-opaque-epoch',
        scalar_step_manifest='/test/scalar.json',scalar_step_manifest_sha256='b'*64,
        native_policy_manifest='/test/base.json',native_policy_manifest_sha256='c'*64,
        acquisition_only=False,compare_feedback=False,v3_voltage_pipeline=False,view_cache_manifest=None,
        view_cache_manifest_sha256=None,native_boot_guard_artifact=None,native_boot_guard_artifact_sha256=None,
        retain_gil_trace_copy=False)


class Parser:
    def error(self,message):raise ValueError(message)


class GenerationTests(unittest.TestCase):
    def test_exact_inverse_and_collect_ast_identity(self):
        original=BASELINE.read_bytes();derived,proof=generate.derive(original)
        self.assertEqual(proof['changed_functions'],['main'])
        left={n.name:ast.dump(n)for n in ast.parse(original).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
        right={n.name:ast.dump(n)for n in ast.parse(derived).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
        self.assertEqual([name for name in left if left[name]!=right[name]],['main'])
        for name in ('collect','_await_output_ready','_await_voltage_ready','_await_acquisition_ready'):
            self.assertEqual(left[name],right[name])
        self.assertIn(b"report['fk_cache_model_source']=source",derived)
        self.assertNotIn(b"report['scalar_step_model_source']=source",derived)

    def test_wrong_baseline_bytes_and_anchor_reject(self):
        with self.assertRaises(ValueError):generate.derive(BASELINE.read_bytes()+b'\n')
        with self.assertRaises(ValueError):generate.once('twice twice','twice','once')

    def test_bundle_is_fresh_private_source_only(self):
        with tempfile.TemporaryDirectory()as directory:
            out=Path(directory).resolve()/'new'
            manifest=generate.generate_bundle(BASELINE,out,target_bundle=out,baseline_kit=Path('/pinned/K37'))
            self.assertEqual(set(manifest['files']),child.NAMES)
            self.assertEqual(len(manifest['files']),13)
            self.assertFalse(manifest['active_controller_qualification'])
            for name,pin in manifest['files'].items():
                self.assertEqual(hashlib.sha256((out/name).read_bytes()).hexdigest(),pin)
            with self.assertRaises(ValueError):generate.generate_bundle(BASELINE,out,target_bundle=out,baseline_kit=Path('/pinned/K37'))

    def module(self):
        raw,_=generate.derive(BASELINE.read_bytes())
        fake=types.SimpleNamespace(validate_cli=mock.Mock(return_value={'selection':'explicit'}),
                                   baseline_runtime=mock.Mock(),finalize_report=mock.Mock())
        name='singularitydog_hw._fk_generation_test'
        spec=importlib.util.spec_from_loader(name,loader=None)
        module=importlib.util.module_from_spec(spec);module.__file__='/explicit/fk_cache_diagnostic_benchmark.py'
        with mock.patch.dict(sys.modules,{'diagnostic_support':fake}):exec(compile(raw,module.__file__,'exec'),module.__dict__)
        return module,fake

    def argv(self):
        return ['--fk-cache-manifest','/explicit/fk.json','--fk-cache-manifest-sha256','a'*64,
            '--mode','stop-proxy','--supported-disabled','--cycles','5','--startup-cycle-allowance','1',
            '--request-gap-us','900','--request-window','3','--v3-voltage-proxy','--v3-voltage-overlap',
            '--v3-voltage-validation-overlap','--v3-voltage-fast-pipeline','--prepare-voltage-before-feedback-publication',
            '--record-storage','trace','--absolute-epoch-cadence','--release-spin-us','500',
            '--require-pinned-fast-model','--single-thread-math','--main-thread-cpu','4','--exclude-policy-cpu-from-workers',
            '--scalar-step-manifest','/scalar.json','--scalar-step-manifest-sha256','b'*64,
            '--native-policy-manifest','/base.json','--native-policy-manifest-sha256','c'*64,
            '--provenance-mode','supported-policy-probe-2s-rare-jitter-v1','--power-epoch','opaque',
            '--accel-input-hypothesis','/hyp.json','--accel-input-hypothesis-sha256','d'*64]

    def test_plan_calls_no_collector_or_torch_and_labels_candidate(self):
        module,fake=self.module();before=set(sys.modules)
        output=io.StringIO()
        with mock.patch.object(module,'collect',side_effect=AssertionError('collector called')),\
             mock.patch.object(module.math_threads,'configure_single_thread_math',return_value={}),\
             mock.patch.object(module,'_start_source_provenance',return_value={'cadence_source_sha256':{'original':'pin'}}),\
             mock.patch.object(module.os,'sched_getaffinity',return_value={0,1,2,3,4},create=True),\
             mock.patch.object(module.os,'sched_setaffinity',create=True),contextlib.redirect_stdout(output):
            self.assertEqual(module.main(self.argv()),0)
        plan=json.loads(output.getvalue());self.assertEqual(plan['policy_backend_requested'],'pinned_fk_cache_cpp')
        self.assertFalse(plan['timing_admission_eligible']);self.assertFalse(plan['active_controller_qualification'])
        self.assertEqual(plan['requests_per_cycle'],26);self.assertEqual(plan['type1_requests_per_cycle'],0)
        self.assertEqual('torch'in sys.modules,'torch'in before)
        fake.validate_cli.assert_called_once()

    def test_abbreviations_and_missing_selection_reject_before_support(self):
        module,fake=self.module()
        for vector in ([],['--fk-cache-man','/test'],self.argv()+['--exec']):
            with self.subTest(vector=vector),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                module.main(vector)
        fake.validate_cli.assert_not_called()


class SupportTests(unittest.TestCase):
    def integration(self):
        return {name:{'path':'/explicit/'+name,'sha256':'e'*64}for name in support.INTEGRATION}

    def test_scope_rejects_any_nonstop_expansion_before_file_read(self):
        negative={'mode':'type17','supported_disabled':False,'cycles':500,'request_gap_us':880,
                  'release_spin_us':200,'request_window':2,'voltage_max_v':43,'record_storage':'objects',
                  'native_boot_guard_artifact':'x','retain_gil_trace_copy':True,'acquisition_only':True,
                  'compare_feedback':True,'main_thread_cpu':3,'single_thread_math':False}
        with mock.patch.object(support.loader,'plan',side_effect=AssertionError('files read')):
            for name,value in negative.items():
                selected=args();setattr(selected,name,value)
                with self.subTest(name=name),self.assertRaises(ValueError):support.validate_cli(selected,Parser(),'/explicit/fk_cache_diagnostic_benchmark.py')

    def test_explicit5_and501_selection_keeps_separate_sources(self):
        refs=self.integration();refs['diagnostic_support.py']['path']=str(Path(support.__file__).absolute())
        with mock.patch.object(support.loader,'plan',return_value={'torch_or_native_loaded':False}),\
             mock.patch.object(support.loader,'_json',return_value={'integration_sources':refs}),\
             mock.patch.object(support.loader,'read',return_value=b'source'),\
             mock.patch.object(support,'baseline_runtime',return_value=Path('/pinned/runtime')):
            for cycles in (5,501):
                selected=args();selected.cycles=cycles
                proof=support.validate_cli(selected,Parser(),'/explicit/fk_cache_diagnostic_benchmark.py')
                self.assertFalse(proof['model_is_original_scalar']);self.assertFalse(proof['timing_admission_eligible'])

    def test_false_executing_source_path_rejected(self):
        refs=self.integration()
        with mock.patch.object(support.loader,'plan',return_value={}),\
             mock.patch.object(support.loader,'_json',return_value={'integration_sources':refs}),self.assertRaises(ValueError):
            support.validate_cli(args(),Parser(),'/other/copy.py')

    def test_finalization_preserves_failure_and_never_qualifies(self):
        proof={'candidate_manifest':{'path':'/p/manifest','sha256':'a'*64},'baseline_dependency':{'path':'/p/original','sha256':'b'*64},'actual_executing_sources':{}}
        report={'status':'COMPLETE_DIAGNOSTIC','errors':[],'cadence_source_sha256':{'original':'pin'},
                'source_provenance':{'cadence_source_sha256':{'original':'pin'}}}
        with mock.patch.object(support.loader,'read',side_effect=ValueError('changed')):support.finalize_report(report,proof)
        self.assertEqual(report['status'],'ABORTED');self.assertFalse(proof['sources_unchanged_after_run'])
        self.assertNotIn('cadence_source_sha256',report);self.assertEqual(report['baseline_dependency_source_sha256'],{'original':'pin'})
        self.assertFalse(report['actual_output_qualification']);self.assertFalse(report['full_controller_50Hz_verified'])


class ChildTests(unittest.TestCase):
    def test_nonblocking_regular_read_and_fifo_reject(self):
        import os
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory).resolve();path=root/'file';path.write_bytes(b'123')
            self.assertEqual(child.read(path),b'123')
            with self.assertRaises(ValueError):child.read(path,limit=2)
            pipe=root/'fifo';os.mkfifo(pipe)
            with self.assertRaises(ValueError):child.read(pipe)

    def test_default_child_fresh_process_and_bundle_pin_fail_closed(self):
        with self.assertRaises(ValueError):child.require_fresh_interpreter()
        with self.assertRaises(ValueError):child.verify(Path('/not/a/bundle'),'UNFROZEN')
        for vector in (['--bund','/p'],['--bundle','/p','--bundle-sha256','a'*64,'--interval-u','100']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):child.main(vector)

    def run_child(self, execute, failure):
        package=types.SimpleNamespace(__path__=[])
        manifest={'files':{'diagnostic_generate.py':'a'*64,'diagnostic_support.py':'b'*64,
                          'fk_cache_diagnostic_benchmark.py':'c'*64},'derivation':{'proof':True}}
        generator=types.SimpleNamespace(derive=mock.Mock(return_value=(b'source',manifest['derivation'])))
        module=types.SimpleNamespace(main=mock.Mock(side_effect=RuntimeError('failed') if failure else None,return_value=0))
        before=sys.getswitchinterval();paths=list(sys.path)
        argv=['--bundle','/bundle','--bundle-sha256','a'*64,'--interval-us','100','--']+(['--execute']if execute else [])
        with mock.patch.object(child,'verify',return_value=(manifest,Path('/kit')))as verify,\
             mock.patch.object(child,'require_fresh_interpreter'),\
             mock.patch.object(child,'bind_replay',return_value={'path':'/replay','sha256':'d'*64}),\
             mock.patch.object(child,'pinned_read',return_value=b'source'),\
             mock.patch.object(child.importlib,'import_module',return_value=package),\
             mock.patch.object(child,'read',return_value=b'source'),\
             mock.patch.object(child,'execute_module',side_effect=[generator,types.SimpleNamespace(),module]),\
             mock.patch.object(child.sys,'setswitchinterval',wraps=sys.setswitchinterval)as switch,\
             contextlib.redirect_stdout(io.StringIO()):
            if failure:
                with self.assertRaisesRegex(RuntimeError,'failed'):child.main(argv)
            else:self.assertEqual(child.main(argv),0)
            self.assertEqual(verify.call_count,2)
            if execute:self.assertEqual(switch.call_count,2)
            else:switch.assert_not_called()
        self.assertEqual(sys.getswitchinterval(),before);self.assertEqual(sys.path,paths)
        self.assertNotIn('diagnostic_support',sys.modules)

    def test_plan_changes_no_switch_or_paths_and_rechecks(self):self.run_child(False,False)
    def test_execute_exception_restores_switch_paths_and_rechecks(self):self.run_child(True,True)

    def test_dependency_selection_is_unique_and_pinned_before_import(self):
        for vector in ([],['--fk-cache-manifest','/m','--fk-cache-manifest-sha256','a'*64,
                           '--fk-cache-manifest','/other']):
            with self.subTest(vector=vector),self.assertRaises(ValueError),\
                 mock.patch.object(child,'execute_module',side_effect=AssertionError('source executed')):
                child.bind_replay(vector)
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory).resolve();helper=root/'helper.py';helper.write_bytes(b'raise AssertionError("not pinned")')
            report=root/'report.json';report.write_text(json.dumps({'replay_helper_sha256':'a'*64}))
            manifest=root/'manifest.json'
            manifest.write_text(json.dumps({'schema':'singularitydog.fk-cache-stop-diagnostic-artifact.v1',
                'status':'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL',
                **dict.fromkeys(support.loader.FALSE_FLAGS,False),
                'references':{'replay_helper':{'path':str(helper),'sha256':'a'*64},
                    'file_only_report':{'path':str(report),'sha256':hashlib.sha256(report.read_bytes()).hexdigest()}}}))
            vector=['--fk-cache-manifest',str(manifest),'--fk-cache-manifest-sha256',hashlib.sha256(manifest.read_bytes()).hexdigest()]
            with self.assertRaisesRegex(ValueError,'Exact frozen'),\
                 mock.patch.object(child,'execute_module',side_effect=AssertionError('source executed')):
                child.bind_replay(vector)

    def test_sparse_original_package_with_pinned_external_replay_plan_in_fresh_process(self):
        # This fixture intentionally omits the helper/first/FK folders exactly
        # as a sparse frozen kit does; the subprocess cannot search the repo.
        here=Path(generate.__file__).absolute().parent
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory).resolve();sparse=root/'sparse';sparse.mkdir()
            shutil.copytree(here.parent,sparse/'native_policy_overnight',ignore=shutil.ignore_patterns(
                'target_tail_fusion','target_tail_fk_cache','saved_input_profile.py','saved_actor_profile.py',
                'test*.py','__pycache__'))
            bundle=root/'bundle';generate.generate_bundle(BASELINE,bundle,target_bundle=bundle,baseline_kit=root/'kit')
            helper=here.parent/'model_call_fastpath/saved_input_profile.py'
            self.assertEqual(hashlib.sha256(helper.read_bytes()).hexdigest(),child.REPLAY_SHA)
            report=root/'report.json';report.write_text(json.dumps({'replay_helper_sha256':child.REPLAY_SHA}))
            manifest=root/'candidate.json';manifest.write_text(json.dumps({
                'schema':'singularitydog.fk-cache-stop-diagnostic-artifact.v1',
                'status':'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL',
                **dict.fromkeys(support.loader.FALSE_FLAGS,False),'references':{
                    'replay_helper':{'path':str(helper),'sha256':child.REPLAY_SHA},
                    'file_only_report':{'path':str(report),'sha256':hashlib.sha256(report.read_bytes()).hexdigest()}}}))
            code=r'''import importlib,importlib.util,json,sys
from pathlib import Path
sparse,bundle,manifest,pin,fixture=sys.argv[1:]
sys.path.insert(0,sparse)
package=importlib.import_module('native_policy_overnight')
package.__path__.append(str(Path(bundle)/'native_policy_overnight'))
spec=importlib.util.spec_from_file_location('isolated_child',Path(bundle)/'diagnostic_child.py')
child=importlib.util.module_from_spec(spec);spec.loader.exec_module(child)
assert not (Path(sparse)/'native_policy_overnight/model_call_fastpath/saved_input_profile.py').exists()
try:importlib.import_module(child.REPLAY_MODULE)
except ModuleNotFoundError:pass
else:raise AssertionError('sparse fixture unexpectedly has replay helper')
ref=child.bind_replay(['--fk-cache-manifest',manifest,'--fk-cache-manifest-sha256',pin])
loader=importlib.import_module('native_policy_overnight.target_tail_fk_cache.diagnostic_loader')
generator=importlib.import_module('native_policy_overnight.target_tail_fk_cache.generator')
replay=importlib.import_module(child.REPLAY_MODULE)
assert generator.replay is replay and Path(replay.__file__).absolute()==Path(ref['path'])
loader._executing_replay(ref)
spec=importlib.util.spec_from_file_location('fixture_tests',fixture)
tests=importlib.util.module_from_spec(spec);spec.loader.exec_module(tests)
case=tests.DiagnosticLoaderFileTests();case.setUp()
try:
 result=case.run_plan(case.write_fixture())
 assert result['torch_or_native_loaded'] is False and result['active_controller_qualification'] is False
finally:case.doCleanups()
assert 'torch' not in sys.modules
print(json.dumps({'status':'PASS_SPARSE_IMPORT_AND_FIXTURE_PLAN','torch_loaded':False}))
'''
            done=subprocess.run([sys.executable,'-I','-B','-c',code,str(sparse),str(bundle),str(manifest),
                hashlib.sha256(manifest.read_bytes()).hexdigest(),str(here/'test_diagnostic_loader.py')],
                capture_output=True,text=True,timeout=15)
            self.assertEqual(done.returncode,0,done.stderr)
            self.assertEqual(json.loads(done.stdout)['status'],'PASS_SPARSE_IMPORT_AND_FIXTURE_PLAN')


if __name__=='__main__':unittest.main()
