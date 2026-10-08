"""File-only build ABI guards; fake preprocessing never opens a device."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import sysconfig
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / 'experiments' / 'native_policy_batch_encode' / 'build_file_only.py'
SPEC = importlib.util.spec_from_file_location('batch_encoder_build_contract', SOURCE)
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class EncoderBuildContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='sdbe-header-contract-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ('Python.h', 'patchlevel.h', 'pyconfig.h'):
            (self.root / name).write_text('synthetic header ' + name)
        self.macros = {'PY_MAJOR_VERSION': str(sys.version_info.major),
                       'PY_MINOR_VERSION': str(sys.version_info.minor),
                       'PY_MICRO_VERSION': str(sys.version_info.micro),
                       'SIZEOF_VOID_P': str(builder.ctypes.sizeof(builder.ctypes.c_void_p)),
                       '__SIZEOF_POINTER__': str(builder.ctypes.sizeof(builder.ctypes.c_void_p)),
                       '__aarch64__': '1'}
        for name in ('Py_DEBUG', 'Py_GIL_DISABLED', 'Py_TRACE_REFS'):
            if sysconfig.get_config_var(name):
                self.macros[name] = '1'
        self.trace = '\n'.join('. ' + str(self.root / name)
                              for name in ('Python.h', 'patchlevel.h', 'pyconfig.h'))

    def probe_result(self):
        return SimpleNamespace(stdout='\n'.join('#define ' + name + ' ' + value
                                               for name, value in self.macros.items()),
                               stderr=self.trace)

    def probe(self):
        with patch.object(builder.subprocess, 'run', return_value=self.probe_result()) as run:
            with patch.object(builder.platform, 'machine', return_value='aarch64'):
                result = builder._header_contract('synthetic-compiler', (str(self.root),))
        return result, run

    def test_exact_target_contract_and_actual_header_hashes_recorded(self):
        result, run = self.probe()
        self.assertEqual(result['target_python']['version'], list(sys.version_info[:3]))
        self.assertEqual(result['header_probe']['version'], list(sys.version_info[:3]))
        self.assertEqual(result['target_python']['pointer_bytes'], builder.ctypes.sizeof(builder.ctypes.c_void_p))
        self.assertEqual({Path(row['path']).name for row in result['header_probe']['headers']},
                         {'Python.h', 'patchlevel.h', 'pyconfig.h'})
        for row in result['header_probe']['headers']:
            self.assertEqual(row['sha256'], builder.hashlib.sha256(Path(row['path']).read_bytes()).hexdigest())
        run.assert_called_once()
        self.assertIn('-E', run.call_args.args[0])
        self.assertNotIn('-o', run.call_args.args[0])
        self.assertEqual(run.call_args.kwargs['input'], '#include <Python.h>\n')

    def test_major_and_minor_mismatch_rejected_before_build(self):
        for name in ('PY_MAJOR_VERSION', 'PY_MINOR_VERSION'):
            with self.subTest(name=name):
                saved = self.macros[name]
                self.macros[name] = str(int(saved) + 1)
                with self.assertRaisesRegex(ValueError, 'major/minor'):
                    self.probe()
                self.macros[name] = saved

    def test_patch_version_recorded_without_inventing_abi_failure(self):
        self.macros['PY_MICRO_VERSION'] = str(sys.version_info.micro + 1)
        result, _ = self.probe()
        self.assertEqual(result['header_probe']['version'][2], sys.version_info.micro + 1)
        self.assertFalse(result['header_probe']['matching_patch_required'])

    def test_header_or_compiler_pointer_mismatch_rejected(self):
        for name in ('SIZEOF_VOID_P', '__SIZEOF_POINTER__'):
            with self.subTest(name=name):
                saved = self.macros[name]
                self.macros[name] = str(int(saved) // 2)
                with self.assertRaisesRegex(ValueError, 'pointer width'):
                    self.probe()
                self.macros[name] = saved

    def test_debug_free_threaded_or_trace_refs_mismatch_rejected(self):
        for name in ('Py_DEBUG', 'Py_GIL_DISABLED', 'Py_TRACE_REFS'):
            with self.subTest(name=name):
                saved = self.macros.get(name)
                if saved is None:
                    self.macros[name] = '1'
                else:
                    del self.macros[name]
                with self.assertRaisesRegex(ValueError, name):
                    self.probe()
                if saved is None:
                    del self.macros[name]
                else:
                    self.macros[name] = saved

    def test_abi_flag_defined_to_zero_is_still_enabled_by_ifdef(self):
        for name in ('Py_DEBUG', 'Py_GIL_DISABLED', 'Py_TRACE_REFS'):
            if not sysconfig.get_config_var(name):
                with self.subTest(name=name):
                    self.macros[name] = '0'
                    with self.assertRaisesRegex(ValueError, name):
                        self.probe()
                    del self.macros[name]

    def test_compiler_architecture_mismatch_rejected(self):
        del self.macros['__aarch64__']
        self.macros['__x86_64__'] = '1'
        with self.assertRaisesRegex(ValueError, 'architecture'):
            self.probe()

    def test_omitted_or_noninteger_macro_fails_closed(self):
        del self.macros['PY_MAJOR_VERSION']
        with self.assertRaisesRegex(ValueError, 'omitted PY_MAJOR_VERSION'):
            self.probe()
        self.macros['PY_MAJOR_VERSION'] = '__invalid__'
        with self.assertRaisesRegex(ValueError, 'Noninteger'):
            self.probe()

    def test_header_trace_must_cover_all_three_abi_headers(self):
        self.trace = '. ' + str(self.root / 'Python.h')
        with self.assertRaisesRegex(ValueError, 'all Python ABI headers'):
            self.probe()

    def test_missing_headers_has_no_compiler_or_fallback_call(self):
        output = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        with patch.object(builder.subprocess, 'run', side_effect=AssertionError('Compiler invoked')):
            with self.assertRaisesRegex(ValueError, 'headers missing'):
                builder.build(output, includes=(str(self.root / 'absent'),))
        self.assertFalse(output.exists())

    def test_wrong_binary_name_or_existing_output_stops_before_probe(self):
        good = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        good.write_bytes(b'existing candidate')
        with patch.object(builder, '_header_contract', side_effect=AssertionError('Probe invoked')):
            for output in (self.root / 'wrong.so', good):
                with self.assertRaisesRegex(ValueError, 'new sdbe_native'):
                    builder.build(output, includes=(str(self.root),))

    def test_dangling_binary_symlink_is_not_a_fresh_output(self):
        output = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        output.symlink_to(self.root / 'missing-target')
        with patch.object(builder, '_header_contract', side_effect=AssertionError('Probe invoked')):
            with self.assertRaisesRegex(ValueError, 'new sdbe_native'):
                builder.build(output, includes=(str(self.root),))

    def test_probe_timeout_never_builds_output(self):
        output = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        with patch.object(builder.subprocess, 'run', side_effect=subprocess.TimeoutExpired('probe', 30)) as run:
            with self.assertRaises(subprocess.TimeoutExpired):
                builder.build(output, includes=(str(self.root),))
        run.assert_called_once()
        self.assertFalse(output.exists())

    def test_pinned_source_failure_precedes_header_probe(self):
        output = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        with patch.dict(builder.PINNED_SOURCE_SHA256, {'batch_encode.cpp': '0' * 64}):
            with patch.object(builder, '_header_contract', side_effect=AssertionError('Probe invoked')):
                with self.assertRaisesRegex(ValueError, 'Pinned batch encoder source differs'):
                    builder.build(output, includes=(str(self.root),))

    def test_complete_build_receipt_is_unapproved_and_header_drift_rejected(self):
        output = self.root / ('sdbe_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        receipt, _ = self.probe()
        def synthetic_compile(command, **kwargs):
            self.assertIn('-ffp-contract=off', command)
            output.write_bytes(b'SYNTHETIC ONLY; never load this file')
            return SimpleNamespace(returncode=0)
        with patch.object(builder, '_header_contract', return_value=receipt):
            with patch.object(builder.subprocess, 'run', side_effect=synthetic_compile):
                result = builder.build(output, includes=(str(self.root),))
        self.assertEqual(result['status'], 'BUILT_UNAPPROVED_CANDIDATE')
        self.assertFalse(result['hardware_opened'])
        self.assertFalse(result['motor_commands_sent'])
        self.assertFalse(result['output_approved'])
        self.assertTrue(result['source_and_header_pins_unchanged_after_build'])
        output.unlink()
        def drift_compile(command, **kwargs):
            synthetic_compile(command, **kwargs)
            (self.root / 'pyconfig.h').write_text('changed during compile')
            return SimpleNamespace(returncode=0)
        with patch.object(builder, '_header_contract', return_value=receipt):
            with patch.object(builder.subprocess, 'run', side_effect=drift_compile):
                with self.assertRaisesRegex(ValueError, 'header changed during build: pyconfig.h'):
                    builder.build(output, includes=(str(self.root),))


if __name__ == '__main__':
    unittest.main()
