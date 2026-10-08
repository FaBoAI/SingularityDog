"""File-only generator and child admission: temporary files, no devices/libraries."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import singularitydog_hw
DIRECTORY=Path(__file__).resolve().parents[1]/'experiments/native_readiness_causal_trace'
BASELINE_SOURCE=Path(__file__).resolve().parent/'fixtures/native_readiness_causal_trace/runtime/singularitydog_hw/native_pipeline_benchmark.py'
BASELINE_SHA='0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

generator=load('causal_generate',DIRECTORY/'generate.py')
child=load('causal_child',DIRECTORY/'child_runner.py')
support=load('causal_support',DIRECTORY/'collector_support.py')
sha=lambda raw:hashlib.sha256(raw).hexdigest()
BASELINE_BYTES=BASELINE_SOURCE.read_bytes()
if sha(BASELINE_BYTES)!=BASELINE_SHA:
    raise ValueError('Exact historical K37 test fixture required')
BASELINE_SPEC=importlib.util.spec_from_file_location('singularitydog_hw._test_generation_k37_baseline',BASELINE_SOURCE)
baseline=importlib.util.module_from_spec(BASELINE_SPEC)
exec(compile(BASELINE_BYTES,str(BASELINE_SOURCE),'exec'),baseline.__dict__)

class GenerationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name).resolve()
        self.addCleanup(self.tmp.cleanup)

    def bundle(self):
        out=self.root/'bundle'
        receipt=generator.build_bundle(BASELINE_SOURCE,out,kit_path=self.root/'kit')
        return out,receipt

    def test_historical_fixture_is_authenticated_and_both_source_pins_unchanged(self):
        provenance=json.loads((BASELINE_SOURCE.parents[2]/'source-provenance.json').read_bytes())
        self.assertEqual(provenance['source_commit'],'6b1c090659b906b0614e13797a88491882c04c19')
        self.assertEqual(provenance['source_path'],'runtime/singularitydog_hw/native_pipeline_benchmark.py')
        self.assertEqual(provenance['bytes'],len(BASELINE_BYTES))
        self.assertEqual(provenance['sha256'],sha(BASELINE_BYTES))
        self.assertEqual(provenance['sha256'],generator.BASELINE_SHA)
        self.assertIs(provenance['hardware_execution_allowed'],False)

    def test_bundle_complete_members_and_inverse_proof_no_execution(self):
        out,receipt=self.bundle();manifest=json.loads((out/'manifest.json').read_bytes())
        self.assertEqual(len(manifest['files']),4)
        for name,digest in manifest['files'].items():self.assertEqual(sha((out/name).read_bytes()),digest)
        self.assertEqual(sha((out/'manifest.json').read_bytes()),receipt['manifest_sha256'])
        self.assertTrue(receipt['inverse_source_bytes_and_ast_identical'])
        self.assertEqual(receipt['changed_function_nodes'],['collect','main'])
        for key in ('hardware_opened','library_loaded','target_deployed','timing_admission_eligible','active_output_eligible'):
            self.assertIs(receipt[key],False)
        self.assertFalse(receipt['execute_launcher_source_sealed'])
        self.assertEqual(manifest['initial_plan_cycles'],5)
        self.assertEqual(manifest['required_phase_choices'],['voltage','output'])
        with self.assertRaisesRegex(ValueError,'Fresh'):
            generator.build_bundle(BASELINE_SOURCE,out,kit_path=self.root/'kit')

    def test_generator_rejects_source_changes_symlink_fifo_and_oversize(self):
        changed=self.root/'changed.py';changed.write_bytes(BASELINE_BYTES+b'\n')
        with self.assertRaisesRegex(ValueError,'baseline'):generator.derive(changed.read_bytes())
        link=self.root/'link';link.symlink_to(changed)
        with self.assertRaises(ValueError):generator.read_regular(link)
        fifo=self.root/'fifo';os.mkfifo(fifo)
        with self.assertRaises(ValueError):generator.read_regular(fifo)
        changed.write_bytes(b'x'*256001)
        with self.assertRaises(ValueError):generator.read_regular(changed)

    def test_private_destination_and_paired_launcher_arguments(self):
        for path in (Path('relative'),self.root/'..'/'other'):
            with self.assertRaises(ValueError):generator.private_destination(path)
        repo=self.root/'repo';repo.mkdir();(repo/'.git').mkdir()
        with self.assertRaises(ValueError):generator.private_destination(repo/'out')
        with self.assertRaises(ValueError):generator.build_bundle(BASELINE_SOURCE,self.root/'out',kit_path=self.root/'kit',target_bundle=self.root/'target')

    def test_wrong_outer_source_never_emits_launcher(self):
        with self.assertRaisesRegex(ValueError,'scoped R37'):
            generator.derive_launcher(b'print("not a launch source")',self.root/'target','0'*64,'1'*64)

    def test_optimized_python_keeps_structure_checks(self):
        command=[sys.executable,'-B','-O','-c',
            'import importlib.util;from pathlib import Path;'
            's=importlib.util.spec_from_file_location("g",'+repr(str(DIRECTORY/'generate.py'))+');'
            'm=importlib.util.module_from_spec(s);s.loader.exec_module(m);'
            'r=m.derive(Path('+repr(str(BASELINE_SOURCE))+').read_bytes());print(r[1])']
        result=subprocess.run(command,capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout.strip(),"['collect', 'main']")
        tree=ast.parse((DIRECTORY/'generate.py').read_bytes())
        self.assertFalse(any(isinstance(node,ast.Assert)for node in ast.walk(tree)))

    def test_cli_unknown_execute_rejected_before_source_access(self):
        result=subprocess.run([sys.executable,'-B',str(DIRECTORY/'generate.py'),'--execute'],
            capture_output=True,text=True,timeout=5)
        self.assertNotEqual(result.returncode,0)
        self.assertNotIn('Traceback',result.stderr)

    def test_no_rows_and_partial_observation_never_claims_requested_cycles(self):
        with patch.object(singularitydog_hw,'native_pipeline_benchmark',baseline,create=True):
            bank=support.TraceBank(5,DIRECTORY/'candidate.py','voltage')
        proof=bank.export_after_cleanup()
        self.assertFalse(proof['trace_complete']);self.assertFalse(proof['requested_cycles_all_traced'])
        self.assertEqual(proof['selected_phase_invocations'],0)
        self.assertEqual(proof['selected_phase_observations'],0)

    def test_child_rejects_changed_bundle_or_wrong_runner_before_import(self):
        out,receipt=self.bundle()
        with self.assertRaisesRegex(ValueError,'pinned bundle member'):child.verify(out,receipt['manifest_sha256'])
        (out/'candidate.py').write_text('raise RuntimeError("must never execute")')
        with self.assertRaisesRegex(ValueError,'member changed'):child.verify(out,receipt['manifest_sha256'])
        with self.assertRaisesRegex(ValueError,'manifest changed'):child.verify(out,'0'*64)

    def test_support_import_rejects_symlink_before_executing_verified_bytes(self):
        link=self.root/'candidate.py';link.symlink_to(DIRECTORY/'candidate.py')
        with self.assertRaisesRegex(ValueError,'non-symlink'):support.candidate_module(link)

    def test_selected_cli_scope_rejects_wrong_cycles_phase_commands_or_experiments(self):
        import argparse
        values=dict(readiness_cause_trace=True,cycles=5,mode='stop-proxy',supported_disabled=True,
            request_gap_us=900,request_window=3,voltage_max_v=42,v3_voltage_proxy=True,
            v3_voltage_overlap=True,v3_voltage_validation_overlap=True,v3_voltage_fast_pipeline=True,
            prepare_voltage_before_feedback_publication=True,record_storage='trace',absolute_epoch_cadence=True,
            release_spin_us=500,startup_cycle_allowance=1,output_dispatch_trace=True,
            inference_thread_cpu_trace=True,provenance_mode='bounded',power_epoch='label',
            acquisition_only=False,compare_feedback=False,v3_voltage_pipeline=False,native_boot_guard_artifact=None,
            native_boot_guard_artifact_sha256=None,retain_gil_trace_copy=False,
            readiness_cause_candidate=str(DIRECTORY/'candidate.py'),readiness_cause_phase='output')
        class Parser:
            def error(self,message):raise ValueError(message)
        for phase in ('voltage','output'):
            support.validate_cli(argparse.Namespace(**dict(values,readiness_cause_phase=phase)),Parser())
        for key,value in [('cycles',4),('cycles',51),('cycles',True),('mode','enable'),('request_gap_us',850),
                          ('readiness_cause_phase','acquisition'),('voltage_max_v',43),('retain_gil_trace_copy',True),
                          ('native_boot_guard_artifact','any'),('supported_disabled',False)]:
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                support.validate_cli(argparse.Namespace(**dict(values,**{key:value})),Parser())

    def synthetic_child(self, execute=False, error=False):
        kit=self.root/'kit';kit.mkdir()
        inventory={}
        for index in range(650):
            name=str(index)+'.txt';raw=str(index).encode();(kit/name).write_bytes(raw);inventory[name]=sha(raw)
        raw=(json.dumps(dict(files=inventory))+'\n').encode();(kit/'kit-manifest.json').write_bytes(raw)
        bundle=self.root/'synthetic';bundle.mkdir()
        code=('import sys,json\ndef main(args):\n'
              ' print(json.dumps(dict(kind="fake_main",value=sys.getswitchinterval(),args=args)))\n')
        code+=' raise RuntimeError("synthetic failure")\n' if error else ' return 0\n'
        files={'candidate.py':b'# never imported\n',
               'readiness_cause_collector_support_r38.py':b'# fake support only\n',
               'native_readiness_cause_benchmark_r38.py':code.encode(),
               'readiness_cause_child_r38.py':(DIRECTORY/'child_runner.py').read_bytes()}
        for name,value in files.items():(bundle/name).write_bytes(value)
        manifest=dict(schema='private.readiness-cause-bundle.v1',files={name:sha(value) for name,value in files.items()},
            active_output_eligible=False,timing_admission_eligible=False,baseline_source_sha256=generator.BASELINE_SHA,
            baseline_kit_path=str(kit),baseline_kit_manifest_sha256=sha(raw))
        manifest_raw=json.dumps(manifest).encode();(bundle/'manifest.json').write_bytes(manifest_raw)
        command=[sys.executable,'-B',str(bundle/'readiness_cause_child_r38.py'),'--bundle',str(bundle),
            '--bundle-sha256',sha(manifest_raw),'--interval-us','100','--']
        if execute:command+=['--execute']
        result=subprocess.run(command,capture_output=True,text=True,timeout=5)
        return result,[json.loads(line) for line in result.stdout.splitlines()]

    def test_child_plan_does_not_change_python_switch_scope(self):
        result,rows=self.synthetic_child()
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertFalse(rows[0]['selected_execute'])
        self.assertEqual(rows[0]['before_s'],rows[0]['during_s'])
        self.assertEqual(rows[1]['value'],rows[0]['before_s'])
        self.assertTrue(rows[-1]['restored']);self.assertTrue(rows[-1]['source_files_unchanged'])

    def test_child_executes_only_synthetic_module_and_restores_after_exception(self):
        result,rows=self.synthetic_child(execute=True,error=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('synthetic failure',result.stderr)
        self.assertTrue(rows[0]['selected_execute'])
        self.assertAlmostEqual(rows[1]['value'],.0001)
        self.assertTrue(rows[-1]['restored']);self.assertTrue(rows[-1]['source_files_unchanged'])

    def test_overhead_default_is_plan_without_library_or_hardware(self):
        result=subprocess.run([sys.executable,'-B',str(DIRECTORY/'measure_overhead.py')],capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        value=json.loads(result.stdout);self.assertEqual(value['status'],'PLAN_ONLY')
        self.assertFalse(value['hardware_opened']);self.assertFalse(value['library_loaded'])

if __name__=='__main__':unittest.main()
