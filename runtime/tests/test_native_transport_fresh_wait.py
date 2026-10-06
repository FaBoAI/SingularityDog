"""Offline source derivation and simulated syscall regression; no real devices."""
from pathlib import Path
import importlib.util
import json
import shutil
import subprocess
import tempfile
import unittest

EXP = Path(__file__).resolve().parents[1]/'experiments/native_transport_fresh_wait'
SOURCE = EXP.parent/'native_transport/transport.cpp'
LEGACY = EXP/'fixtures/transport-before-fresh-wait.cpp'
spec = importlib.util.spec_from_file_location('fresh_wait_variant', EXP/'prepare_variant.py')
variant = importlib.util.module_from_spec(spec)
spec.loader.exec_module(variant)


class NativeTransportFreshWaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            raise unittest.SkipTest('A C++ compiler is needed for syscall simulation')
        cls.temporary = tempfile.TemporaryDirectory(prefix='fresh-wait-simulator-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.base = Path(cls.temporary.name)
        cls.executables = {}
        original = LEGACY.read_bytes()
        if variant.sha(original) != variant.SOURCE_SHA256:
            raise AssertionError('Legacy fixture bytes changed')
        if SOURCE.read_bytes() != variant.derive(original):
            raise AssertionError('Production must equal the reviewed one-block fix')
        for label, raw in [('original', original), ('fresh', SOURCE.read_bytes())]:
            source = cls.base/(label+'.cpp'); source.write_bytes(raw)
            executable = cls.base/label
            subprocess.run([compiler, '-std=c++17', '-O2',
                f'-DSOURCE_FILE="{source}"', str(EXP/'fake_transport_harness.cpp'),
                '-o', str(executable)], check=True, capture_output=True, timeout=30)
            cls.executables[label] = executable

    def run_case(self, label, mode):
        result = subprocess.run([str(self.executables[label]), str(mode)],
            check=True, capture_output=True, text=True, timeout=2)
        return json.loads(result.stdout)

    def test_exact_source_delta_only_refreshes_relative_timeout(self):
        original = LEGACY.read_bytes()
        derived = variant.derive(original)
        self.assertEqual(derived.decode().replace(variant.AFTER, variant.BEFORE),
                         original.decode())
        self.assertEqual(original.decode().count(variant.BEFORE), 1)
        self.assertEqual(derived.decode().count(variant.AFTER), 1)
        self.assertEqual(SOURCE.read_bytes(), derived)

    def test_corrected_source_is_idempotent_without_reverse_transform(self):
        source = SOURCE.read_bytes()
        self.assertIs(variant.derive(source), source)
        plan = variant.prepare(SOURCE)
        self.assertEqual(plan['input_source_version'], 'corrected_fresh_relative_wait')
        self.assertEqual(plan['output_source_version'], 'corrected_fresh_relative_wait')
        self.assertEqual(plan['transformation'], 'already_corrected_no_delta')
        self.assertEqual(plan['source_sha256'], plan['variant_sha256'])
        self.assertEqual(plan['diff_sha256'], variant.sha(b''))

    def test_legacy_input_identifies_explicit_forward_transformation(self):
        plan = variant.prepare(LEGACY)
        self.assertEqual(plan['input_source_version'], 'legacy_stale_relative_wait')
        self.assertEqual(plan['transformation'], 'legacy_block_replaced')
        self.assertEqual(plan['source_sha256'], variant.SOURCE_SHA256)
        self.assertEqual(plan['variant_sha256'], variant.CORRECTED_SHA256)

    def test_unknown_or_modified_base_is_rejected(self):
        with self.assertRaises(ValueError): variant.derive(SOURCE.read_bytes()+b'\n')
        with self.assertRaises(ValueError): variant.derive(SOURCE.read_text())
        with self.assertRaises(ValueError): variant.derive(LEGACY.read_bytes()+b'\n')

    def test_plan_has_no_build_load_or_output_directory(self):
        plan = variant.prepare(SOURCE)
        self.assertEqual(plan['status'], 'PLAN_ONLY')
        for key in ('compiled', 'native_library_loaded', 'hardware_opened',
                    'output_allowed', 'approved_for_runtime'):
            self.assertIs(plan[key], False)

    def test_fresh_source_staging_keeps_original_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'variant'
            receipt = variant.prepare(SOURCE, output)
            self.assertEqual((output/'original-transport.cpp').read_bytes(), SOURCE.read_bytes())
            self.assertEqual(variant.sha((output/'transport.cpp').read_bytes()),
                             receipt['variant_sha256'])
            with self.assertRaises(ValueError): variant.prepare(SOURCE, output)
            self.assertFalse(receipt['timing_admission_eligible'])

    def test_fixed_source_staging_has_no_source_diff(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'fixed'
            receipt = variant.prepare(SOURCE, output)
            self.assertEqual((output/'source.diff').read_bytes(), b'')
            self.assertEqual((output/'transport.cpp').read_bytes(), SOURCE.read_bytes())
            self.assertEqual(receipt['transformation'], 'already_corrected_no_delta')

    def test_boot_read_elapsed_is_subtracted_from_same_absolute_gap(self):
        old, new = (self.run_case(label, 0) for label in ('original', 'fresh'))
        self.assertEqual(old['status'], 0); self.assertEqual(new['status'], 0)
        self.assertEqual((old['writes'],new['writes'],old['reads'],new['reads']), (2,2,2,2))
        self.assertEqual(old['first_write_ns'], new['first_write_ns'])
        self.assertEqual(new['second_write_ns']-new['first_write_ns'], 5_000_000)
        self.assertEqual(old['second_write_ns']-old['first_write_ns'], 7_000_000)
        self.assertEqual(old['deadline_ns'], new['deadline_ns'])

    def test_no_reply_deadline_is_not_extended_by_boot_read(self):
        old, new = (self.run_case(label, 1) for label in ('original', 'fresh'))
        self.assertEqual((old['status'], new['status']), (-1,-1))
        self.assertEqual(new['end_ns'], new['deadline_ns'])
        self.assertEqual(old['end_ns']-old['deadline_ns'], 2_000_000)
        self.assertEqual((old['writes'], new['writes']), (1,1))
        self.assertIn('no retry', new['error'])

    def test_eintr_recomputes_remaining_deadline_without_retrying_write(self):
        old, new = (self.run_case(label, 2) for label in ('original', 'fresh'))
        self.assertEqual((old['status'],new['status']), (-1,-1))
        self.assertEqual(new['end_ns'],new['deadline_ns'])
        self.assertEqual(old['deadline_ns'],new['deadline_ns'])
        self.assertEqual((old['writes'],new['writes']), (1,1))
        self.assertGreater(old['end_ns'],old['deadline_ns'])

    def test_cancellation_is_still_checked_before_write(self):
        for label in ('original','fresh'):
            result = self.run_case(label,3)
            self.assertEqual(result['status'],-1)
            self.assertEqual(result['writes'],0)
            self.assertEqual(result['error'],'Cancelled')

    def test_partial_write_never_retransmitted(self):
        for label in ('original','fresh'):
            result = self.run_case(label,4)
            self.assertEqual(result['status'],-1)
            self.assertEqual(result['writes'],1)
            self.assertEqual(result['error'],'Partial/failed write; no retransmission')

    def test_no_boot_delay_preserves_all_observed_results(self):
        self.assertEqual(self.run_case('original',5),self.run_case('fresh',5))

    def test_eintr_limit_retained_and_no_commands_sent(self):
        for label in ('original','fresh'):
            result = self.run_case(label,6)
            self.assertEqual(result['status'],-1)
            self.assertEqual(result['writes'],0)
            self.assertEqual(result['waits'],34)
            self.assertEqual(result['error'],'pselect failed')

    def test_expired_during_boot_read_aborts_before_pselect_or_write(self):
        result = self.run_case('fresh',7)
        self.assertEqual(result['status'],-1)
        self.assertEqual(result['writes'],0)
        self.assertEqual(result['waits'],1)  # The original initial backlog/cancel check only.
        self.assertIn('deadline exceeded before all writes',result['error'])

    def test_type1_remains_rejected_before_any_syscall_wait_or_write(self):
        for label in ('original','fresh'):
            result = self.run_case(label,8)
            self.assertEqual(result['status'],-1)
            self.assertEqual((result['writes'],result['waits']), (0,0))
            self.assertEqual(result['error'],'Disallowed/noncanonical diagnostic command')


if __name__ == '__main__':
    unittest.main()
