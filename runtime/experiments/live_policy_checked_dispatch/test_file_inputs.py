"""File-only negative gates; these tests never import Torch."""
import builtins
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

if __package__:
    from . import build
    from .file_inputs import pinned, source_inventory
else:
    import build
    from file_inputs import pinned, source_inventory


class FileInputTests(unittest.TestCase):
    def test_changed_bytes_and_symlink_do_not_acquire_a_pin(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            path = Path(directory) / 'input'
            path.write_bytes(b'original')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(pinned(path, digest), b'original')
            path.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'SHA differs'):
                pinned(path, digest)
            link = path.with_name('link')
            link.symlink_to(path)
            with self.assertRaisesRegex(ValueError, 'nonsymlink'):
                pinned(link, hashlib.sha256(path.read_bytes()).hexdigest())

    def test_inventory_count_false_flags_and_mode_are_required(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            root = Path(directory)
            runtime = root / 'runtime'
            runtime.mkdir()
            source = runtime / 'source.py'
            source.write_bytes(b'original')
            manifest = {'schema': 'fixture.source-only.v1', 'file_count': 1,
                        'output_allowed': False, 'approved_for_runtime': False,
                        'files': {'runtime/source.py': {'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                            'bytes': source.stat().st_size, 'mode': source.stat().st_mode & 0o777}}}
            path = root / 'manifest.json'
            def inspect(value):
                path.write_text(json.dumps(value))
                return source_inventory(runtime, {'path': str(path),
                    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}, {})
            self.assertEqual(inspect(manifest)['file_count'], 1)
            for key, value in (('file_count', 2), ('output_allowed', True), ('approved_for_runtime', True)):
                changed = {**manifest, key: value}
                with self.assertRaisesRegex(ValueError, 'count and false'):
                    inspect(changed)
            source.chmod((source.stat().st_mode & 0o777) ^ 0o100)
            with self.assertRaisesRegex(ValueError, 'size/mode'):
                inspect(manifest)

    def test_build_plan_never_imports_torch_or_creates_output(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            output = Path(directory) / 'fresh-build'
            cpp = Path(build.__file__).parent / 'checked_dispatch.cpp'
            digest = hashlib.sha256(cpp.read_bytes()).hexdigest()
            actual_import = builtins.__import__
            def guarded_import(name, *args, **kwargs):
                if name == 'torch' or name.startswith('torch.'):
                    raise AssertionError('PLAN imported Torch')
                return actual_import(name, *args, **kwargs)
            with patch('builtins.__import__', side_effect=guarded_import), redirect_stdout(io.StringIO()) as console:
                self.assertEqual(build.main(['--output', str(output), '--source-sha256', digest]), 0)
                with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                    build.main(['--output', str(output), '--source-sha256', '0' * 64])
            self.assertEqual(json.loads(console.getvalue())['status'], 'PLAN_ONLY')
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
