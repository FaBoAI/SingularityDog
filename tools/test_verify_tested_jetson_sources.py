"""Integrity checks must not follow source paths outside the repository."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.verify_tested_jetson_sources import verify, SCHEMA


class TestedJetsonSourceVerification(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.file = self.root / "runtime/pkg/code.py"
        self.file.parent.mkdir(parents=True)
        self.file.write_bytes(b"raise RuntimeError('never import historical source')\n")
        self.manifest = {
            "schema": SCHEMA, "snapshot": "example", "historical_source_only": True,
            "live_output_authorized": False, "source_count": 1, "archived_source_count": 0,
            "sources": [{"source_path": "pkg/code.py", "storage_path": "runtime/pkg/code.py",
                         "size_bytes": self.file.stat().st_size,
                         "sha256": hashlib.sha256(self.file.read_bytes()).hexdigest()}],
        }

    def check(self):
        (self.root / "manifest.json").write_text(json.dumps(self.manifest))
        return verify(self.root, "manifest.json")

    def test_valid_source_is_hashed_without_execution(self):
        self.assertEqual(self.check()["sources"], 1)
        self.assertFalse(self.check()["live_output_authorized"])

    def test_content_tamper_is_detected(self):
        self.file.write_bytes(self.file.read_bytes().replace(b"raise", b"print"))
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            self.check()

    def test_traversal_and_noncanonical_paths_rejected(self):
        for path in ("../outside.py", "/tmp/outside.py", "runtime/../outside.py", "runtime//code.py"):
            with self.subTest(path=path):
                self.manifest["sources"][0]["storage_path"] = path
                with self.assertRaises(ValueError):
                    self.check()

    def test_file_symlink_rejected(self):
        target = self.root / "other.py"
        self.file.rename(target)
        self.file.symlink_to(target)
        with self.assertRaisesRegex(ValueError, "Symlink"):
            self.check()

    def test_directory_symlink_rejected(self):
        directory = self.file.parent
        directory.rename(self.root / "other")
        directory.symlink_to(self.root / "other", target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "Symlink"):
            self.check()

    def test_duplicate_and_authorizing_manifest_rejected(self):
        self.manifest["sources"] *= 2
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.check()
        self.manifest["sources"] = self.manifest["sources"][:1]
        self.manifest["live_output_authorized"] = True
        with self.assertRaisesRegex(ValueError, "non-authorizing"):
            self.check()

    def test_manifest_symlink_rejected(self):
        (self.root / "real.json").write_text(json.dumps(self.manifest))
        (self.root / "manifest.json").symlink_to(self.root / "real.json")
        with self.assertRaisesRegex(ValueError, "Symlink"):
            verify(self.root, "manifest.json")


if __name__ == "__main__":
    unittest.main()
