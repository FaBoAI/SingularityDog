"""File-only benchmark bounds, parity and restoration; no compiler or devices."""
import gc
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


_PATH=Path(__file__).resolve().parents[1]/'experiments/native_policy_batch_encode/benchmark_runtime_wires.py'
_SPEC=importlib.util.spec_from_file_location('runtime_motion_wire_benchmark_tests',_PATH)
benchmark=importlib.util.module_from_spec(_SPEC);_SPEC.loader.exec_module(benchmark)


class RuntimeMotionWireBenchmarkTests(unittest.TestCase):
    def setUp(self):self.original_gc=gc.isenabled()
    def tearDown(self):
        if self.original_gc:gc.enable()
        else:gc.disable()

    def test_actual_default_parity_and_honest_component_scope(self):
        report=benchmark.run(samples=3,trials=2,parity_cases=4)
        self.assertEqual(report['parity_cases'],4)
        self.assertIsNone(report['exhaustive_u16'])
        self.assertIsNone(report['binary_sha256'])
        self.assertTrue(report['gc_restored'])
        for name in ('hardware_opened','motor_commands_sent','model_loaded','profile_changed',
                     'output_approved','jetson_measured','whole_cycle_measured'):
            self.assertIs(report[name],False,name)
        for row in report['trials']:
            self.assertEqual(set(row['samples_ns']),{'legacy_parser','default_fixed_frame'})
            self.assertEqual(len(row['samples_ns']['default_fixed_frame']),3)

    def test_invalid_counts_and_incomplete_library_pin_rejected_before_load(self):
        for changes in ({'samples':True},{'samples':0},{'samples':10001},{'trials':11},
                        {'parity_cases':0},{'library':'absent.so'},
                        {'binary_sha256':'a'*64},{'exhaustive_u16':1}):
            with self.subTest(changes=changes),patch.object(benchmark.batch,'load_verified_module') as load:
                with self.assertRaises(ValueError):benchmark.run(**changes)
                load.assert_not_called()

    def test_owned_bytes_and_bus_order_required_even_when_values_compare_equal(self):
        args=benchmark.bindings(benchmark.batch._test_specs())
        command=benchmark.batch._command(args[2],(3.,)*12,(.15,)*12,(0.,)*12)
        expected=benchmark.legacy(command,*args)
        for actual in (dict(reversed(tuple(expected.items()))),
                       {scope:[bytearray(wire) for wire in wires] for scope,wires in expected.items()}):
            with self.assertRaises(AssertionError):benchmark.same(actual,expected)

    def test_measurement_exception_restores_enabled_gc(self):
        gc.enable();original=benchmark.runtime._python_motion_wires
        def candidate(*args,**kwargs):
            if not gc.isenabled():raise RuntimeError('injected measurement failure')
            return original(*args,**kwargs)
        with patch.object(benchmark.runtime,'_python_motion_wires',side_effect=candidate):
            with self.assertRaisesRegex(RuntimeError,'measurement failure'):
                benchmark.run(samples=1,trials=1,parity_cases=1)
        self.assertTrue(gc.isenabled())

    def test_existing_disabled_gc_is_preserved(self):
        gc.disable()
        self.assertTrue(benchmark.run(samples=1,trials=1,parity_cases=1)['gc_restored'])
        self.assertFalse(gc.isenabled())


if __name__=='__main__':unittest.main()
