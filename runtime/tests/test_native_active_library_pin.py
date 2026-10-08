"""Expected-library handoff over temporary regular files; never a motor device."""
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native


class LibraryPinTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name).resolve() / 'synthetic.so'
        self.path.write_bytes(b'SYNTHETIC NEVER DLOPEN')
        self.sha = hashlib.sha256(self.path.read_bytes()).hexdigest()

    def test_path_replacement_cannot_replace_verified_descriptor_bytes(self):
        with native._library_binding(self.path, self.sha) as (name, digest):
            replacement = self.path.with_suffix('.new'); replacement.write_bytes(b'DIFFERENT')
            os.replace(replacement, self.path)
            self.assertEqual(Path(name).read_bytes(), b'SYNTHETIC NEVER DLOPEN')
            self.assertEqual(digest, self.sha)
        self.assertFalse(Path(name).exists()); self.assertFalse(Path(name).parent.exists())

    def test_wrong_digest_or_invalid_digest_rejected(self):
        for value in ('f' * 64, 'bad', True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                with native._library_binding(self.path, value): self.fail('Must reject before dlopen')

    def test_in_place_change_rejected_and_descriptor_closed(self):
        with self.assertRaisesRegex(ValueError, 'changed while loading'):
            with native._library_binding(self.path, self.sha) as (name, digest):
                self.path.write_bytes(b'CHANGED IN PLACE')
        self.assertFalse(Path(name).exists())

    def test_repeated_loads_never_reuse_the_dlopen_snapshot_name(self):
        names = []
        for _ in range(3):
            with native._library_binding(self.path, self.sha) as (name, digest):
                names.append(name)
                self.assertEqual(Path(name).read_bytes(), self.path.read_bytes())
        self.assertEqual(len(set(names)), 3)
        self.assertTrue(all(not Path(name).exists() for name in names))

    def test_no_pin_preserves_original_path(self):
        with native._library_binding(self.path, None) as (name, digest):
            self.assertEqual(name, str(self.path)); self.assertIsNone(digest)


if __name__ == '__main__': unittest.main()
