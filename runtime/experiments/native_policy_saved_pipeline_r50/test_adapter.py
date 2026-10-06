"""Incomplete EXEC must fail closed; pure bit checks are not model validation."""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
from . import adapter, inputs, support
from .test_inputs import fixture


class AdapterTests(unittest.TestCase):
    def test_exec_rejected_before_bootstrap_import_or_output(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            output = Path(directory) / 'result'
            before = set(sys.modules)
            with (mock.patch.object(adapter, 'bootstrap', side_effect=AssertionError('Unexpected bootstrap')),
                    contextlib.redirect_stderr(io.StringIO()) as stream):
                status = adapter.main(['--manifest', '/missing/frozen-manifest.json',
                    '--manifest-sha256', '0' * 64, '--output-dir', str(output), '--mode', 'EXEC'])
            self.assertEqual(status, 2); self.assertFalse(output.exists())
            result = json.loads(stream.getvalue())
            self.assertEqual(result['status'], 'INCOMPLETE_EXEC_REJECTED')
            self.assertTrue(result['requested_model_load']); self.assertFalse(result['model_loaded'])
            self.assertFalse(result['native_library_loaded']); self.assertFalse(result['hardware_opened'])
            self.assertFalse(result['target_run_permitted']); self.assertEqual(set(sys.modules), before)

    def test_bootstrap_rejects_extra_and_hash_drift_before_exec(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            folder = Path(directory)
            for name in adapter.NAMES:
                (folder / name).write_text('# source fixture\n')
            doc = {'target_bundle_path': str(folder), 'source_files':
                {name: inputs.sha((folder / name).read_bytes()) for name in adapter.NAMES}}
            manifest = folder / 'manifest.json'; manifest.write_bytes(inputs.canonical(doc))
            with mock.patch.object(adapter, '__file__', str(folder / 'adapter.py')):
                (folder / 'unmanifested.pyc').write_bytes(b'not allowed')
                with self.assertRaisesRegex(ValueError, 'inventory'):
                    adapter.bootstrap(manifest, inputs.sha(manifest.read_bytes()))
                (folder / 'unmanifested.pyc').unlink()
                (folder / 'inputs.py').write_text('raise AssertionError("untrusted")\n')
                with self.assertRaisesRegex(ValueError, 'pin changed'):
                    adapter.bootstrap(manifest, inputs.sha(manifest.read_bytes()))
            self.assertNotIn(adapter.PRIVATE, sys.modules)

    def test_bit_comparison_permits_only_explicit_metadata(self):
        _, rows, _, _, _ = fixture(); expected = rows[0]['observed']
        actual = copy.deepcopy(expected)
        actual.update(run_number=77, consume_profile={'timing_fixture': 1}, timing_scope='fixture')
        support.compare_observed(actual, expected)
        actual['provenance']['extra_source_flag'] = False
        with self.assertRaisesRegex(ValueError, 'provenance bits'): support.compare_observed(actual, expected)

    def test_output_and_input_signed_zero_rejected(self):
        _, rows, _, _, _ = fixture(); expected = rows[0]['observed']
        for field in ('output', 'input'):
            actual = copy.deepcopy(expected)
            values = actual['observation74'] if field == 'output' else actual['inputs']['q_model_rad']
            values[0] = -0.
            with self.assertRaisesRegex(ValueError, 'float32 bits'): support.compare_observed(actual, expected)

    def test_full_binary64_and_scalar_types_are_not_masked(self):
        self.assertNotEqual(support.tree_bits(0.), support.tree_bits(-0.))
        self.assertNotEqual(support.tree_bits(1), support.tree_bits(True))
        self.assertEqual(support.tree_bits({'b': 0., 'a': 1}), support.tree_bits({'a': 1, 'b': 0.}))
        with self.assertRaises(ValueError): support.tree_bits(float('nan'))

    def test_bounded_abba_only_labels_no_times(self):
        self.assertEqual(support.abba_schedule(1), ('python_copy', 'c_copy', 'c_copy', 'python_copy'))
        self.assertEqual(len(support.abba_schedule()), 12)
        for value in (0, 4, True, 1.0):
            with self.assertRaises(ValueError): support.abba_schedule(value)

    def test_pure_support_exec_explicit_incomplete(self):
        with self.assertRaisesRegex(NotImplementedError, 'INCOMPLETE'): support.reject_incomplete_exec()


if __name__ == '__main__': unittest.main()
