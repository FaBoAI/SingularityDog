"""Independent file-only checks of the feedback comparison harness.

Uses the authoritative Python codec as a fake compiled decoder. No C++ build,
transport descriptor, model, socket or hardware is used.
"""
import dataclasses
import gc
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


_PATH = (Path(__file__).resolve().parents[1] / 'experiments' /
         'native_active_transport' / 'benchmark_feedback_decode.py')
_SPEC = importlib.util.spec_from_file_location('feedback_decode_benchmark_for_tests', _PATH)
benchmark = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(benchmark)


class ReferenceDecoder:
    available = True

    def __init__(self, library):
        self.library = library

    def decode(self, records, first_id):
        assert first_id in (1, 7)
        return benchmark.runtime.decode_records((records, None))


class FeedbackDecodeBenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.gc_before = gc.isenabled()

    def tearDown(self):
        if self.gc_before:
            gc.enable()
        else:
            gc.disable()

    def run_fake(self, decoder=ReferenceDecoder, **kwargs):
        options = {'samples': 2, 'trials': 2, 'warmup': 1}
        options.update(kwargs)
        with patch.object(benchmark.native, 'NativeFeedbackBatchDecoder', decoder), \
                patch.object(benchmark.native.ActiveSession, '__init__',
                             side_effect=AssertionError('No active session may be opened')):
            return benchmark.run(object(), **options)

    def test_reference_parity_and_report_do_not_claim_hardware_timing(self):
        report = self.run_fake()
        self.assertEqual(report['schema'], 'singularitydog.feedback-batch-decode-benchmark.v1')
        self.assertFalse(report['hardware_opened'])
        self.assertFalse(report['robot_commands_sent'])
        self.assertFalse(report['jetson_measured'])
        self.assertFalse(report['whole_cycle_measured'])
        self.assertTrue(report['source_records_unchanged'])
        self.assertTrue(report['gc_restored'])
        self.assertEqual(len(report['trials']), 2)
        for index, row in enumerate(report['trials']):
            self.assertEqual(row['trial'], index)
            self.assertEqual(row['python']['samples'], 2)
            self.assertEqual(row['cpp']['samples'], 2)
            self.assertEqual(len(row['samples_ns']['python']), 2)
            self.assertEqual(len(row['samples_ns']['cpp']), 2)

    def test_absent_optional_abi_rejected_before_measurement(self):
        class Unavailable(ReferenceDecoder):
            available = False
        with self.assertRaisesRegex(ValueError, 'does not have the feedback batch ABI'):
            self.run_fake(Unavailable)

    def test_key_insertion_order_parity_is_required(self):
        class Reordered(ReferenceDecoder):
            def decode(self, records, first_id):
                return dict(reversed(tuple(super().decode(records, first_id).items())))
        with self.assertRaisesRegex(AssertionError, 'decode differs'):
            self.run_fake(Reordered)

    def test_timestamp_parity_is_required(self):
        class Restamped(ReferenceDecoder):
            def decode(self, records, first_id):
                rows = super().decode(records, first_id)
                key = next(iter(rows))
                value, begin, end = rows[key]
                rows[key] = (value, begin, end + 1)
                return rows
        with self.assertRaisesRegex(AssertionError, 'decode differs'):
            self.run_fake(Restamped)

    def test_fault_and_mode_parity_are_required(self):
        for field in ('fault_bits', 'mode_state'):
            with self.subTest(field=field):
                class Altered(ReferenceDecoder):
                    def decode(self, records, first_id):
                        rows = super().decode(records, first_id)
                        key = next(iter(rows))
                        value, begin, end = rows[key]
                        rows[key] = (dataclasses.replace(value, **{
                            field: getattr(value, field) + 1}), begin, end)
                        return rows
                with self.assertRaisesRegex(AssertionError, 'decode differs'):
                    self.run_fake(Altered)

    def test_signed_zero_bits_are_checked_even_when_float_equality_passes(self):
        original_fixtures = benchmark.fixtures

        def zero_temperature():
            cases = original_fixtures()
            for buses in cases:
                for _, records in buses:
                    for record in records:
                        record.rx[13] = record.rx[14] = 0
            return cases

        class NegativeZero(ReferenceDecoder):
            def decode(self, records, first_id):
                rows = super().decode(records, first_id)
                return {key: (dataclasses.replace(value, temperature_c=-0.0), begin, end)
                        for key, (value, begin, end) in rows.items()}

        with patch.object(benchmark, 'fixtures', zero_temperature):
            with self.assertRaisesRegex(AssertionError, 'Double bits differ'):
                self.run_fake(NegativeZero)

    def test_source_mutation_is_rejected_even_for_non_decoded_record_field(self):
        class Mutating(ReferenceDecoder):
            def decode(self, records, first_id):
                # The current codec intentionally does not consume this field.
                records[0].read_start_ns += 1
                return super().decode(records, first_id)
        with self.assertRaisesRegex(AssertionError, 'modified a source record'):
            self.run_fake(Mutating)

    def test_measurement_exception_restores_enabled_gc(self):
        gc.enable()
        class FailsDuringMeasurement(ReferenceDecoder):
            calls = 0
            def decode(self, records, first_id):
                type(self).calls += 1
                # Six bus calls establish initial parity; the seventh occurs
                # inside the measured loop with cyclic GC disabled.
                if self.calls == 7:
                    self.assert_gc_disabled()
                    raise RuntimeError('injected measured decoder error')
                return super().decode(records, first_id)
            @staticmethod
            def assert_gc_disabled():
                if gc.isenabled():
                    raise AssertionError('Expected deferred GC during measurement')
        with self.assertRaisesRegex(RuntimeError, 'injected measured decoder error'):
            self.run_fake(FailsDuringMeasurement, warmup=0)
        self.assertTrue(gc.isenabled())

    def test_initially_disabled_gc_stays_disabled(self):
        gc.disable()
        report = self.run_fake()
        self.assertFalse(gc.isenabled())
        self.assertTrue(report['gc_restored'])

    def test_cli_rejects_unbounded_counts_before_library_loading(self):
        for option, value in (('--samples', '0'), ('--samples', '10001'),
                              ('--trials', '0'), ('--trials', '11'),
                              ('--warmup', '-1'), ('--warmup', '10001')):
            with self.subTest(option=option, value=value), tempfile.TemporaryDirectory() as directory:
                output = str(Path(directory) / 'report.json')
                arguments = ['benchmark_feedback_decode.py', '--library', '/not/opened/library.so',
                             '--output', output, option, value]
                with patch('sys.argv', arguments), \
                        patch.object(benchmark.native, 'load_library') as load, \
                        patch('sys.stderr'):
                    with self.assertRaises(SystemExit) as caught:
                        benchmark.main()
                    self.assertEqual(caught.exception.code, 2)
                    load.assert_not_called()

    def test_cli_refuses_existing_output_before_library_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'report.json'
            output.write_text('preserved original\n')
            arguments = ['benchmark_feedback_decode.py', '--library', '/not/opened/library.so',
                         '--output', str(output)]
            with patch('sys.argv', arguments), \
                    patch.object(benchmark.native, 'load_library') as load, \
                    patch('sys.stderr'):
                with self.assertRaises(SystemExit) as caught:
                    benchmark.main()
                self.assertEqual(caught.exception.code, 2)
                load.assert_not_called()
            self.assertEqual(output.read_text(), 'preserved original\n')


if __name__ == '__main__':
    unittest.main()
