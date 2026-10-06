"""Only file reads, a temporary C++ build and synthetic clocks/calls."""
from pathlib import Path
import contextlib
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

EXP=Path(__file__).resolve().parents[1]/'experiments/native_transport_causal_trace'
spec=importlib.util.spec_from_file_location('causal_trace_plan',EXP/'plan.py')
plan=importlib.util.module_from_spec(spec);spec.loader.exec_module(plan)


class TracePlan(unittest.TestCase):
    def test_default_plan_is_explicitly_unimplemented_no_hardware(self):
        with contextlib.redirect_stdout(io.StringIO()) as stream:plan.main([])
        result=json.loads(stream.getvalue())
        self.assertEqual(result['status'],'PLAN_ONLY')
        for key in ('existing_transport_abi_changed','instrumentation_integrated','hardware_opened',
                    'library_loaded','compiled','output_allowed','timing_admission_eligible'):
            self.assertIs(result[key],False)

    def test_no_execute_or_output_flag_exists(self):
        for arg in ('--execute','--output','--build'):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):plan.main([arg])

    def test_unknown_or_symlink_source_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d).resolve()/'source.cpp';p.write_bytes(plan.DEFAULT_SOURCE.read_bytes()+b'\n')
            with self.assertRaises(ValueError):plan.plan(p)
            p.unlink();p.symlink_to(plan.DEFAULT_SOURCE)
            with self.assertRaises(ValueError):plan.plan(p)

    def test_fifo_source_rejected_without_wait(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d).resolve()/'fifo';os.mkfifo(p)
            result=subprocess.run([sys.executable,'-B',str(EXP/'plan.py'),'--source',str(p)],
                capture_output=True,text=True,timeout=2)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Bounded regular source',result.stderr)


class BoundedTracePrimitive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler=shutil.which('clang++') or shutil.which('g++')
        if compiler is None:raise unittest.SkipTest('C++ compiler needed for fake-call tests')
        cls.temp=tempfile.TemporaryDirectory(prefix='causal-trace-fake-')
        cls.addClassCleanup(cls.temp.cleanup);cls.executable=Path(cls.temp.name)/'fake'
        subprocess.run([compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror',
            str(EXP/'fake_trace_harness.cpp'),'-o',str(cls.executable)],
            check=True,capture_output=True,timeout=30)

    def case(self,mode):
        result=subprocess.run([str(self.executable),str(mode)],check=True,
            capture_output=True,text=True,timeout=2)
        return json.loads(result.stdout)

    def test_return_and_errno_preserved_despite_clock_errno_changes(self):
        r=self.case(0)
        self.assertEqual((r['returned'],r['event_returned']),(17,17))
        self.assertEqual(r['final_errno'],r['expected_errno'])
        self.assertEqual(r['event_errno'],r['expected_errno'])
        self.assertEqual(r['seen_errno'],r['entry_errno'])
        self.assertEqual(r['call_count'],1)
        self.assertTrue(r['clock_valid'])

    def test_fixed_capacity_overflow_does_not_skip_or_retry_call(self):
        r=self.case(1)
        self.assertEqual((r['stored'],r['calls'],r['dropped'],r['call_count']),(2,5,3,5))
        self.assertTrue(r['overflow']);self.assertEqual(r['returned'],17)
        self.assertEqual(r['final_errno'],r['expected_errno'])

    def test_backward_clock_marks_invalid_without_altering_result(self):
        r=self.case(2);self.assertFalse(r['clock_valid']);self.assertTrue(r['invalid_clock'])
        self.assertEqual((r['returned'],r['call_count']),(17,1))
        self.assertEqual(r['final_errno'],r['expected_errno'])

    def test_absent_cpu_clock_is_explicitly_unmeasured(self):
        r=self.case(3)
        self.assertFalse(r['cpu_measured']);self.assertTrue(r['clock_valid'])
        self.assertEqual((r['cpu_calls'],r['cpu_begin_ns'],r['cpu_end_ns']),(0,0,0))

    def test_no_recorder_invokes_original_once_without_clock_calls(self):
        r=self.case(4)
        self.assertEqual((r['call_count'],r['wall_calls'],r['cpu_calls'],r['stored']),(1,0,0,0))
        self.assertEqual((r['returned'],r['final_errno']),(17,r['expected_errno']))

    def test_eintr_retained_without_retry(self):
        r=self.case(5)
        self.assertEqual((r['returned'],r['event_returned'],r['call_count']),(-1,-1,1))
        self.assertEqual(r['final_errno'],r['expected_errno'])

    def test_other_error_retained_without_retry(self):
        r=self.case(6)
        self.assertEqual((r['returned'],r['event_returned'],r['call_count']),(-1,-1,1))
        self.assertEqual(r['event_errno'],r['expected_errno'])

    def test_zero_or_missing_wall_clock_marks_evidence_invalid(self):
        for mode in (8,9):
            r=self.case(mode);self.assertFalse(r['clock_valid']);self.assertTrue(r['invalid_clock'])
            self.assertEqual((r['returned'],r['call_count']),(17,1))

    def test_absolute_deadline_and_requested_wait_are_metadata_only(self):
        r=self.case(0);self.assertTrue(r['context_unchanged'])
        self.assertEqual((r['event_deadline_ns'],r['event_wait_ns']),(1000,400))


if __name__=='__main__':unittest.main()
