"""File-only benchmark guards and parity checks, without compiling C++."""
import gc
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


_PATH = (Path(__file__).resolve().parents[1] / 'experiments' /
         'native_policy_batch_encode' / 'benchmark_verified_wrapper.py')
_SPEC = importlib.util.spec_from_file_location('verified_batch_benchmark_tests', _PATH)
benchmark = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(benchmark)


class ReferenceModule:
    def bind(self, specs):
        return lambda command: benchmark.batch._reference_wires(command, specs)


class VerifiedBatchEncoderBenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.gc_before = gc.isenabled()

    def tearDown(self):
        if self.gc_before:
            gc.enable()
        else:
            gc.disable()

    def run_fake(self, module=None, **kwargs):
        options = {'samples': 2, 'trials': 2, 'warmup': 1, 'parity_cases': 3}
        options.update(kwargs)
        with patch.object(benchmark.batch, 'load_verified_module',
                          return_value=module or ReferenceModule()):
            return benchmark.run('/not/opened/library.so', binary_sha256='a' * 64, **options)

    def test_seeded_parity_and_scope_with_authoritative_reference(self):
        report = self.run_fake()
        self.assertEqual(report['parity_cases'], 3)
        self.assertEqual(len(report['trials']), 2)
        self.assertTrue(report['gc_restored'])
        self.assertEqual(len(report['reference_wires_sha256']), 64)
        for key in ('hardware_opened', 'motor_commands_sent', 'model_loaded',
                    'profile_changed', 'output_approved', 'jetson_measured', 'whole_cycle_measured'):
            self.assertFalse(report[key], key)
        for trial in report['trials']:
            for name in ('python', 'verified_cpp'):
                self.assertEqual(trial[name]['samples'], 2)
                self.assertEqual(len(trial['samples_ns'][name]), 2)

    def test_unbounded_or_boolean_counts_rejected_before_loading_library(self):
        for name, value in (('samples', 0), ('samples', 10001), ('samples', True),
                            ('trials', 0), ('trials', 11), ('warmup', -1),
                            ('warmup', 10001), ('parity_cases', 0), ('parity_cases', 10001)):
            with self.subTest(name=name, value=value), \
                    patch.object(benchmark.batch, 'load_verified_module') as load:
                with self.assertRaisesRegex(ValueError, 'Bounded positive'):
                    benchmark.run('/not/opened/library.so', binary_sha256='a' * 64, **{name: value})
                load.assert_not_called()

    def test_source_seed_is_deterministic_and_reference_limits_hold(self):
        specs = benchmark.batch._test_specs()
        first = benchmark.command_cases(specs, count=5)
        second = benchmark.command_cases(specs, count=5)
        self.assertEqual([vars(command) for command in first], [vars(command) for command in second])
        for command in first:
            wires = benchmark.batch._reference_wires(command, specs)
            benchmark.same_wires(wires, wires)

    def test_equal_bytearrays_are_not_accepted_as_the_required_owned_bytes(self):
        specs = benchmark.batch._test_specs()
        command = benchmark.command_cases(specs, count=1)[0]
        expected = benchmark.batch._reference_wires(command, specs)
        actual = {bus: [bytearray(wire) for wire in wires] for bus, wires in expected.items()}
        self.assertEqual(actual, expected)
        with self.assertRaisesRegex(AssertionError, 'bytes wires'):
            benchmark.same_wires(actual, expected)

    def test_bus_order_or_corrupted_wire_is_rejected_before_timing(self):
        for altered_order in (True, False):
            class AlteredModule:
                def bind(self, specs):
                    def encode(command):
                        wires = benchmark.batch._reference_wires(command, specs)
                        if altered_order:
                            return dict(reversed(tuple(wires.items())))
                        wires['front'][0] = b'XX' + wires['front'][0][2:]
                        return wires
                    return encode
            with self.subTest(altered_order=altered_order):
                with self.assertRaisesRegex(AssertionError, 'bytes wires'):
                    self.run_fake(AlteredModule())

    def test_measurement_exception_restores_enabled_gc(self):
        gc.enable()
        class FailsDuringMeasurement:
            def bind(self, specs):
                calls = 0
                def encode(command):
                    nonlocal calls
                    calls += 1
                    if calls == 2:
                        if gc.isenabled():
                            raise AssertionError('Expected GC disabled while timing')
                        raise RuntimeError('injected encoder error during measurement')
                    return benchmark.batch._reference_wires(command, specs)
                return encode
        with self.assertRaisesRegex(RuntimeError, 'injected encoder error'):
            self.run_fake(FailsDuringMeasurement(), parity_cases=1, warmup=0)
        self.assertTrue(gc.isenabled())

    def test_disabled_gc_is_preserved(self):
        gc.disable()
        report = self.run_fake()
        self.assertFalse(gc.isenabled())
        self.assertTrue(report['gc_restored'])

    def test_existing_output_is_preserved_before_loading_library(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'report.json'
            output.write_text('original\n')
            argv = ['benchmark_verified_wrapper.py', '--library', '/not/opened/library.so',
                    '--binary-sha256', 'a' * 64, '--output', str(output)]
            with patch('sys.argv', argv), patch('sys.stderr'), \
                    patch.object(benchmark.batch, 'load_verified_module') as load:
                with self.assertRaises(SystemExit) as caught:
                    benchmark.main()
                self.assertEqual(caught.exception.code, 2)
                load.assert_not_called()
            self.assertEqual(output.read_text(), 'original\n')


if __name__ == '__main__':
    unittest.main()
