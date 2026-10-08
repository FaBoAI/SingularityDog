"""Private source/collector checks. Synthetic transport and IMU only."""
import ast
import contextlib
from concurrent.futures import Future
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
import singularitydog_hw

DIRECTORY = Path(__file__).resolve().parents[1] / 'experiments/native_readiness_causal_trace'

def load_source(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

generator = load_source('causal_generator_test', DIRECTORY/'generate.py')
support = load_source('readiness_cause_collector_support_r38', DIRECTORY/'collector_support.py')
BASELINE_SOURCE = Path(__file__).resolve().parent / 'fixtures/native_readiness_causal_trace/runtime/singularitydog_hw/native_pipeline_benchmark.py'
BASELINE_BYTES = BASELINE_SOURCE.read_bytes()
if hashlib.sha256(BASELINE_BYTES).hexdigest() != generator.BASELINE_SHA:
    raise ValueError('Exact historical K37 test fixture required')
BASELINE_SPEC = importlib.util.spec_from_file_location('singularitydog_hw._test_collector_k37_baseline',BASELINE_SOURCE)
original = importlib.util.module_from_spec(BASELINE_SPEC)
exec(compile(BASELINE_BYTES,str(BASELINE_SOURCE),'exec'),original.__dict__)
from test_native_pipeline_benchmark import Device, Observer, Session
CANDIDATE = DIRECTORY/'candidate.py'


class SyntheticSession(Session):
    def exchange(self, wires, *, before_native=None):
        if before_native: before_native()
        rows, stats = super().exchange(wires)
        if len(wires) == 1:
            rows[0].rx[11:15] = struct.pack('<f', 40.)
        return rows, stats


class Tests(unittest.TestCase):
    def setUp(self):
        # The historical support lazily imports this exact module. Keep the
        # replacement local to each test and restore the production attribute.
        self.enterContext(patch.object(singularitydog_hw,'native_pipeline_benchmark',original,create=True))

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.generated = Path(cls.directory.name).resolve() / 'generated.py'
        cls.generated.write_bytes(generator.derive(Path(original.__file__).read_bytes())[0])
        with patch.dict(sys.modules, {'readiness_cause_collector_support_r38':support}):
            cls.module = load_source('singularitydog_hw._causal_test_r38', cls.generated)

    @classmethod
    def tearDownClass(cls): cls.directory.cleanup()

    def collect(self, fail=False, fail_after=None):
        class Policy(Observer):
            def consume(self, snapshot):
                if fail or self.calls == fail_after: raise RuntimeError('synthetic model failure')
                return super().consume(snapshot)
        bank = support.TraceBank(5, CANDIDATE, "output")
        def wait(deadline):
            while time.monotonic_ns() < deadline: time.sleep(.00001)
            return time.monotonic_ns()
        return self.module.collect({s:SyntheticSession() for s in ('front','rear')}, Device(), Policy(),
            mode='stop-proxy', cycles=5, v3_voltage_proxy=True, v3_voltage_overlap=True,
            v3_voltage_validation_overlap=True, v3_voltage_fast_pipeline=True,
            record_storage='trace', deadline_wait=wait, absolute_epoch_cadence=True,
            readiness_cause_recorder=bank)

    def test_exact_generation_and_only_declared_function_changes(self):
        source, changed = generator.derive(Path(original.__file__).read_bytes())
        self.assertEqual(source, self.generated.read_bytes())
        self.assertEqual(changed, ['collect', 'main'])
        for name in ('_await_owned_ready', '_readiness_poll_target', '_await_voltage_ready',
                     '_await_acquisition_ready', '_await_output_ready'):
            old = next(n for n in ast.parse(Path(original.__file__).read_bytes()).body if getattr(n,'name',None)==name)
            new = next(n for n in ast.parse(source).body if getattr(n,'name',None)==name)
            self.assertEqual(ast.dump(old), ast.dump(new))

    def test_fake_complete_preserves_raw_and_adds_only_selected_phase_records(self):
        report, rows = self.collect()
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 5)
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])
        proof = report['readiness_causal_trace']
        self.assertEqual(len(proof['rows']), 5)
        self.assertEqual(proof['errors'], [])
        self.assertTrue(proof['original_and_instrumenter_unchanged'])
        self.assertEqual(proof['selected_phase_invocations'],5)
        self.assertEqual(proof['selected_phase_observations'],5)
        self.assertTrue(proof['requested_cycles_all_traced'])
        self.assertTrue(all(x['trace']['helper_returned'] for x in proof['rows']))
        self.assertFalse(report['timing_admission_eligible'])
        saved = self.module._serialize(rows)
        self.assertEqual(len(saved), 5)
        self.assertTrue(all(sum(len(row[p][s]['records']) for p in ('acquired','voltage','output')
                               for s in ('front','rear')) == 26 for row in saved))

    def test_fake_failure_retains_original_error_raw_and_attempted_phase_trace(self):
        report, rows = self.collect(True)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('synthetic model failure', str(report['errors']))
        self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(len(rows), 1)
        saved = self.module._serialize(rows)
        self.assertEqual(saved[0]['output'], {})
        trace = report['readiness_causal_trace']
        self.assertEqual([(x['cycle'],x['phase']) for x in trace['rows']], [])
        self.assertEqual(trace['errors'], [])
        self.assertFalse(trace['trace_complete'])
        self.assertFalse(trace['selected_phase_observed'])
        self.assertFalse(trace['requested_cycles_all_traced'])
        self.assertEqual(trace['selected_phase_invocations'], 0)

    def test_partial_run_reports_only_observed_cycles(self):
        report, rows = self.collect(fail_after=2)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 2)
        trace = report['readiness_causal_trace']
        self.assertEqual(trace['selected_phase_invocations'], 2)
        self.assertEqual(trace['selected_phase_observations'], 2)
        self.assertTrue(trace['trace_complete'])
        self.assertFalse(trace['requested_cycles_all_traced'])
        self.assertIn('synthetic model failure', str(report['errors']))

    def test_abbreviated_execute_is_rejected_before_validation_or_hardware(self):
        for flag in ('--exec', '--exe', '--ex'):
            with self.subTest(flag=flag), patch.object(support, 'validate_cli') as validate:
                output=io.StringIO()
                with contextlib.redirect_stderr(output), self.assertRaises(SystemExit) as result:
                    self.module.main(['--readiness-cause-phase','output',flag])
                self.assertEqual(result.exception.code,2)
                self.assertIn('unrecognized arguments',output.getvalue())
                validate.assert_not_called()

    def test_source_changed_rejects_generator(self):
        with self.assertRaises(ValueError):
            generator.derive(Path(original.__file__).read_bytes()+b'\n')

    def test_bank_configuration_and_cycle_reuse_reject(self):
        for cycles in (4, 51, True):
            with self.assertRaises(ValueError): support.TraceBank(cycles, CANDIDATE, "output")
        bank = support.TraceBank(5, CANDIDATE, "output")
        with self.assertRaises(ValueError):
            bank.validate(cycles=5, mode='type17', fast=True, overlap=True,
                          validation=True, storage='trace', native_wait=lambda x:None)
        bank.select(0)
        with self.assertRaises(ValueError): bank.select(0)

    def test_unselected_acquisition_keeps_exact_original_imu_future_guard(self):
        for selected in ('voltage','output'):
            bank=support.TraceBank(5,CANDIDATE,selected)
            bank.select(0)
            futures={scope:Future() for scope in ('front','rear')}
            for future in futures.values():future.set_result(None)
            options=dict(deadline_ns=1_000_000,deadline_wait=None,clock=lambda:1,
                         thread_clock=lambda:1,check=lambda:None)
            for invalid in (None,object(),False):
                messages=[]
                for call in (original._await_acquisition_ready,bank.acquisition):
                    with self.assertRaises(ValueError) as error:call(futures,invalid,**options)
                    messages.append(str(error.exception))
                self.assertEqual(messages,['Current acquisition IMU Future required']*2)
            imu=Future();imu.set_result(None)
            self.assertEqual(bank.acquisition(futures,imu,**options),
                             original._await_acquisition_ready(futures,imu,**options))
            self.assertEqual(bank.export_after_cleanup()['selected_phase_invocations'],0)

    def test_provenance_names_actual_copy_and_final_report_cannot_claim_canonical_graph(self):
        proof = support.provenance(self.generated, CANDIDATE, "output")
        self.assertFalse(proof['copied_module_is_original'])
        source = {'cadence_source_sha256': {'original':'sha'}, 'source_files_unchanged':True}
        report = {'status':'COMPLETE_DIAGNOSTIC','cadence_source_sha256':{'original':'sha'},
                  'source_provenance':source,'plan':{'source_provenance':source}}
        support.finalize_report(report, proof)
        self.assertNotIn('cadence_source_sha256', report)
        self.assertNotIn('cadence_source_sha256', source)
        self.assertEqual(report['baseline_dependency_source_sha256'], {'original':'sha'})
        self.assertTrue(proof['sources_unchanged_after_run'])
        self.assertFalse(report['timing_admission_eligible'])
        json.dumps(report, allow_nan=False)

    def test_final_provenance_mutation_aborts_without_dropping_original_errors(self):
        proof = support.provenance(self.generated, CANDIDATE, "output")
        proof['actual_executing_module']['sha256']='0'*64
        report={'status':'ABORTED','errors':['original failure']}
        support.finalize_report(report, proof)
        self.assertEqual(report['errors'][0], 'original failure')
        self.assertFalse(proof['sources_unchanged_after_run'])


if __name__ == '__main__': unittest.main()
