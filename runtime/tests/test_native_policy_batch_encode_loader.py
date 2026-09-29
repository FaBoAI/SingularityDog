"""File-only provenance, byte parity and fail-closed loader checks."""

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import sysconfig
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import native_policy_batch_encode as loader


class NativeBatchLoaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('c++')
        include = Path(sysconfig.get_path('include'))
        suffix = sysconfig.get_config_var('EXT_SUFFIX')
        if not compiler or not (include / 'Python.h').is_file() or not suffix:
            raise unittest.SkipTest('C++ compiler or Python development headers absent')
        cls.directory = tempfile.TemporaryDirectory(prefix='sdbe-loader-test-')
        cls.root = Path(cls.directory.name)
        cls.binary = cls.root / ('sdbe_native' + suffix)
        args = [compiler, '-std=c++17', '-O2', '-ffp-contract=off',
                '-I' + str(include)]
        if sys.platform == 'darwin':
            args += ['-dynamiclib', '-undefined', 'dynamic_lookup']
        else:
            args += ['-fPIC', '-shared']
        args += [str(loader.DEFAULT_SOURCE_DIR / 'batch_encode_py.cpp'),
                 '-o', str(cls.binary)]
        subprocess.run(args, check=True, capture_output=True)
        cls.binary_sha = hashlib.sha256(cls.binary.read_bytes()).hexdigest()

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, 'directory'):
            cls.directory.cleanup()

    def test_reviewed_binary_and_sources_produce_exact_wires(self):
        specs = loader._test_specs()
        module = loader.load_verified_module(
            self.binary, expected_binary_sha256=self.binary_sha,
        )
        encoder = module.bind(specs)
        q = tuple(row[4] for row in specs)
        command = SimpleNamespace(q_model_rad=q, kp=(3.,) * 12,
                                  kd=(.15,) * 12,
                                  estimated_pd_torque_nm=(0.,) * 12)
        self.assertEqual(encoder.binary_sha256, self.binary_sha)
        self.assertEqual(module.binary_sha256, self.binary_sha)
        self.assertEqual(encoder(command), loader._reference_wires(command, specs))
        self.assertEqual([len(w) for wires in encoder(command).values() for w in wires],
                         [17] * 12)

    def test_wrong_binary_hash_rejected_before_extension_import(self):
        with patch.object(loader.importlib.util, 'spec_from_file_location',
                          side_effect=AssertionError('extension import attempted')):
            with self.assertRaisesRegex(ValueError, 'binary'):
                loader.load_verified_encoder(
                    self.binary, expected_binary_sha256='0' * 64,
                    axis_specs=loader._test_specs())

    def test_source_change_rejected_before_extension_import(self):
        source_dir = self.root / 'altered-sources'
        source_dir.mkdir()
        for name in loader.PINNED_SOURCE_SHA256:
            shutil.copyfile(loader.DEFAULT_SOURCE_DIR / name, source_dir / name)
        with (source_dir / 'batch_encode.cpp').open('ab') as handle:
            handle.write(b'\n// altered\n')
        with patch.object(loader.importlib.util, 'spec_from_file_location',
                          side_effect=AssertionError('extension import attempted')):
            with self.assertRaisesRegex(ValueError, 'batch_encode.cpp'):
                loader.load_verified_encoder(
                    self.binary, expected_binary_sha256=self.binary_sha,
                    axis_specs=loader._test_specs(), source_dir=source_dir)

    def test_modified_binary_rejected(self):
        # Keep a distinct path while preserving the valid extension suffix.
        altered = self.root / 'other' / self.binary.name
        altered.parent.mkdir()
        altered.write_bytes(self.binary.read_bytes() + b'altered')
        with self.assertRaisesRegex(ValueError, 'binary'):
            loader.load_verified_encoder(altered,
                expected_binary_sha256=self.binary_sha,
                axis_specs=loader._test_specs())

    def test_byte_canary_detects_wrong_native_result(self):
        encoder = loader.load_verified_encoder(
            self.binary, expected_binary_sha256=self.binary_sha,
            axis_specs=loader._test_specs())

        class WrongBytes:
            make_context = encoder._module.make_context

            @staticmethod
            def encode(context, command):
                wires = encoder._module.encode(context, command)
                wires['front'][0] = b'XX' + wires['front'][0][2:]
                return wires

        with self.assertRaisesRegex(ValueError, 'byte parity'):
            loader._check_candidate_parity(WrongBytes())

    def test_bind_rejects_invalid_profile_spec_before_command(self):
        module = loader.load_verified_module(
            self.binary, expected_binary_sha256=self.binary_sha)
        specs = list(loader._test_specs())
        row = list(specs[3])
        row[1] = 0.
        specs[3] = tuple(row)
        with self.assertRaises(ValueError):
            module.bind(tuple(specs))


if __name__ == '__main__':
    unittest.main()
