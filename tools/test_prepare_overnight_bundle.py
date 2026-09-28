"""Private kit packaging tests use synthetic history; no robot or saved data access."""

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    "prepare_overnight_bundle", Path(__file__).with_name("prepare_overnight_bundle.py"))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def write_fixture(root, relative, content):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def synthetic_inputs(root):
    repo = root / "repository"
    home = root / "snapshot-home"
    for relative, content in {
        "runtime/singularitydog_hw/__init__.py": b"# synthetic runtime\n",
        "runtime/experiments/native_transport/transport.cpp": b"// synthetic source\n",
        "runtime/experiments/native_transport/transport.h": b"// synthetic header\n",
        "runtime/experiments/native_transport/README.md": b"Synthetic build notes\n",
        "runtime/experiments/native_transport/libdog_transport.so": b"DO NOT COPY A HOST BINARY",
        "runtime/experiments/native_transport/old-build-record.json": b"{}",
        "runtime/__pycache__/stale.py": b"DO NOT COPY A CACHE",
        "runtime/__pycache__/stale.pyc": b"DO NOT COPY A CACHE",
        "tools/dog_tomorrow.py": b"# synthetic next-day entry point\n",
        "tools/audit_angle_calibration.py": b"# synthetic audit entry point\n",
        "docs/overnight-validation-20260927.md": (
            b"[runner](../tools/dog_tomorrow.py)\n"
            b"[source](../runtime/experiments/native_transport/transport.cpp)\n"),
        "docs/angle-calibration-overnight-20260927.md": (
            b"[audit](../tools/audit_angle_calibration.py)\n"),
    }.items():
        write_fixture(repo, relative, content)
    for relative, content in {
        "singularitydog-logs/RO-policy-candidate-current-boot-20260927-r1.json": b'{"approved_for_runtime":false}\n',
        "singularitydog-policy-shadow/20260921-r16-mount/imu-mount-candidate.json": b'{"mount_candidate":true}\n',
        "singularitydog-logs/dual-policy-once-current-boot-20260927-r4/summary.json": b'{"hardware_run":false}\n',
        "singularitydog-policy-shadow/20260921-r1/model_149.pt": b"SYNTHETIC MODEL BYTES",
        "singularitydog-policy-shadow/20260921-r1/swing_core.py": b"# synthetic policy core\n",
        "singularitydog-policy-shadow/20260921-r1/swing_deployment.py": b"# synthetic deployment\n",
    }.items():
        write_fixture(home, relative, content)
    profile = {"schema": "synthetic-review-only-profile", "approved_for_runtime": False}
    uids = {str(mid): f"{mid:016x}" for mid in range(1, 13)}
    return repo, home, profile, uids


class OvernightBundleTests(unittest.TestCase):
    def build_fixture(self, root):
        repo, home, profile, uids = synthetic_inputs(root)
        output = root / "private-kit"
        with patch.object(tool, "ROOT", repo), patch.object(
                tool, "history_profile", return_value=(profile, {}, {}, {}, uids)) as history:
            result = tool.build(home, output)
            history.assert_called_once_with(home.resolve())
        return output, result, home

    def test_config_paths_and_docs_resolve_within_packaged_structure(self):
        with tempfile.TemporaryDirectory() as folder:
            output, result, _ = self.build_fixture(Path(folder))
            config = json.loads((output / "kit-config.json").read_text())
            for key in ("expected_uids", "angle_profile", "calibration", "mount", "bundle"):
                relative = Path(config[key])
                self.assertFalse(relative.is_absolute())
                self.assertNotIn("..", relative.parts)
                self.assertTrue((output / relative).exists(), key)
            self.assertTrue((output / "docs/overnight-validation-20260927.md").is_file())
            docs = output / "docs"
            for relative in ("../tools/dog_tomorrow.py",
                             "../runtime/experiments/native_transport/transport.cpp",
                             "../tools/audit_angle_calibration.py"):
                self.assertTrue((docs / relative).resolve().is_relative_to(output.resolve()))
                self.assertTrue((docs / relative).is_file())
            self.assertFalse((output / "overnight-validation-20260927.md").exists())
            self.assertFalse(config["calibration_approved_for_runtime"])
            self.assertTrue(config["raw_data_private"])
            self.assertFalse(result["hardware_accessed"])

    def test_every_file_and_directory_is_private(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _, _ = self.build_fixture(Path(folder))
            for path in (output, *output.rglob("*")):
                self.assertEqual(path.stat().st_mode & 0o777,
                                 0o700 if path.is_dir() else 0o600, str(path))

    def test_host_binaries_build_records_and_caches_are_excluded(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _, home = self.build_fixture(Path(folder))
            files = {str(path.relative_to(output)) for path in output.rglob("*") if path.is_file()}
            self.assertFalse(any("__pycache__" in name for name in files))
            self.assertFalse(any(name.endswith((".so", ".dylib", ".pyc")) for name in files))
            self.assertNotIn("runtime/experiments/native_transport/old-build-record.json", files)
            # The explicitly selected model is an intentional private binary input.
            self.assertEqual((output / "inputs/policy/model_149.pt").read_bytes(),
                             (home / "singularitydog-policy-shadow/20260921-r1/model_149.pt").read_bytes())
            self.assertIn("runtime/experiments/native_transport/transport.cpp", files)
            self.assertIn("runtime/experiments/native_transport/transport.h", files)

    def test_manifest_covers_exact_file_bytes_with_relative_identities(self):
        with tempfile.TemporaryDirectory() as folder:
            output, result, _ = self.build_fixture(Path(folder))
            manifest = json.loads((output / "kit-manifest.json").read_text())
            self.assertEqual(manifest["schema"], "private-overnight-kit-v1")
            self.assertFalse(manifest["hardware_accessed"])
            expected = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in output.rglob("*")
                        if path.is_file() and path.name != "kit-manifest.json"}
            self.assertEqual(manifest["files"], expected)
            self.assertEqual(result["file_count"], len(expected))
            for relative in manifest["files"]:
                self.assertFalse(Path(relative).is_absolute())
                self.assertNotIn("..", Path(relative).parts)
            changed = output / "tools/dog_tomorrow.py"
            changed.write_bytes(b"# changed after packaging\n")
            self.assertNotEqual(hashlib.sha256(changed.read_bytes()).hexdigest(),
                                manifest["files"]["tools/dog_tomorrow.py"])

    def test_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            output, _, home = self.build_fixture(root)
            before = (output / "kit-manifest.json").read_bytes()
            with self.assertRaisesRegex(ValueError, "must be new"):
                tool.build(home, output)
            self.assertEqual((output / "kit-manifest.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
