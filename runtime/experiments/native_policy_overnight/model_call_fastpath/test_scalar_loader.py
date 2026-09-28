"""Pure file-only rejection tests for the diagnostic scalar-step loader."""
import builtins
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from native_policy_overnight import contracts
from native_policy_overnight.model_call_fastpath import scalar_loader as loader


class ScalarLoaderPreflightTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.manifest_path = self.root / "scalar-manifest.json"
        self.model_path = self.root / "scalar-model.pt"
        self.library_path = self.root / "scalar-step.so"
        self.model_path.write_bytes(b"file-only model fixture")
        self.library_path.write_bytes(b"file-only library fixture")

        source = Path(__file__).resolve().parent
        self.reports = {
            phase: json.loads((source / f"jetson-scalar-{phase}-20260928.json").read_text())
            for phase in ("parity", "synthetic", "timing")
        }
        self.report_paths = {
            phase: self.root / f"{phase}-report.json" for phase in self.reports
        }
        common = self.reports["parity"]
        self.manifest = {
            "schema": "native-step-scalar-file-only-v1",
            "status": "PASS_FILE_ONLY_COMPARE",
            "hardware_opened": False,
            "output_allowed": False,
            "approved_for_runtime": False,
            "live_50hz_verified": False,
            "environment": copy.deepcopy(common["environment"]),
            "baseline_manifest_sha256": common["baseline_manifest_sha256"],
            "view_manifest_path": str(self.root / "cached-view-manifest.json"),
            "view_manifest_sha256": self.reports["timing"]["view_manifest_sha256"],
            "records_sha256": common["records_sha256"],
            "source_hashes": {
                name: contracts.sha(path.read_bytes())
                for name, path in loader._SOURCE_PATHS.items()
            },
            "model_file": self.model_path.name,
            "model_sha256": contracts.sha(self.model_path.read_bytes()),
            "library_file": self.library_path.name,
            "library_sha256": contracts.sha(self.library_path.read_bytes()),
        }
        for phase, report in self.reports.items():
            report["scalar_step_source_sha256"] = self.manifest["source_hashes"][
                "model_call_fastpath/step_scalar.cpp"]
            report["scalar_replay_source_sha256"] = self.manifest["source_hashes"][
                "model_call_fastpath/scalar_replay.py"]
            report["candidate_model_sha256"] = self.manifest["model_sha256"]
            report["candidate_library_sha256"] = self.manifest["library_sha256"]
            self.manifest[f"{phase}_report_file"] = self.report_paths[phase].name

    def write_fixture(self):
        for phase, path in self.report_paths.items():
            path.write_text(json.dumps(self.reports[phase], allow_nan=False))
            self.manifest[f"{phase}_report_sha256"] = contracts.sha(path.read_bytes())
        self.manifest_path.write_text(json.dumps(self.manifest, allow_nan=False))
        return contracts.sha(self.manifest_path.read_bytes())

    def load_without_torch_or_library(self, digest):
        """An invalid file must stop before Torch, the baseline, or any custom op."""
        original_import = builtins.__import__

        def forbidden_torch(name, *args, **kwargs):
            if name == "torch" or name.startswith("torch."):
                raise AssertionError("Invalid scalar manifest reached Torch import")
            return original_import(name, *args, **kwargs)

        with mock.patch.object(loader, "load_cached",
                               side_effect=AssertionError("Invalid manifest loaded baseline")):
            with mock.patch("builtins.__import__", side_effect=forbidden_torch):
                return loader.load_file_only_verified(
                    self.manifest_path, expected_sha256=digest,
                    baseline_manifest=self.root / "baseline.json",
                    baseline_sha=self.manifest["baseline_manifest_sha256"],
                    bundle=self.root / "bundle")

    def assert_rejected(self, pattern):
        digest = self.write_fixture()
        with self.assertRaisesRegex(ValueError, pattern):
            self.load_without_torch_or_library(digest)

    def test_valid_preflight_checks_all_pinned_files_without_importing_torch(self):
        digest = self.write_fixture()
        path, data, reports = loader._prevalidate(
            self.manifest_path, digest, self.manifest["baseline_manifest_sha256"])
        self.assertEqual(path, self.manifest_path)
        self.assertEqual(data["source_hashes"], self.manifest["source_hashes"])
        self.assertEqual(set(reports), {"parity", "synthetic", "timing"})

    def test_bad_manifest_sha_and_duplicate_keys_reject_before_torch(self):
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            self.load_without_torch_or_library("0" * 64)
        self.manifest_path.write_text('{"schema":"native-step-scalar-file-only-v1",'
                                      '"schema":"other"}')
        with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
            self.load_without_torch_or_library(
                contracts.sha(self.manifest_path.read_bytes()))

    def test_manifest_and_report_scope_flags_require_literal_false(self):
        for flag in loader._FALSE_FLAGS:
            for value in (True, 0, None):
                with self.subTest(kind="manifest", flag=flag, value=value):
                    self.manifest[flag] = value
                    self.assert_rejected("diagnostic only")
                    self.manifest[flag] = False
                with self.subTest(kind="parity", flag=flag, value=value):
                    self.reports["parity"][flag] = value
                    self.assert_rejected("report scope differs")
                    self.reports["parity"][flag] = False

    def test_missing_or_inexact_saved_parity_rejects_before_torch(self):
        for key, value in (("saved_recurrent_calls", 499),
                           ("all_named_state_each_call", False),
                           ("observation_actor_target_exact", False),
                           ("input_mutation", True),
                           ("rejection_count", 25)):
            with self.subTest(key=key):
                original = self.reports["parity"]["validation"][key]
                self.reports["parity"]["validation"][key] = value
                self.assert_rejected("saved-input parity incomplete")
                self.reports["parity"]["validation"][key] = original
        maxima = self.reports["parity"]["validation"]["max_errors"]
        original = maxima.pop("target")
        self.assert_rejected("saved-input parity incomplete")
        maxima["target"] = original
        original = self.reports["parity"]["validation"].pop("rejections")
        self.assert_rejected("rejection parity incomplete")
        self.reports["parity"]["validation"]["rejections"] = original

    def test_wrong_source_bytes_or_declared_hash_reject_before_torch(self):
        name = "model_call_fastpath/step_scalar.cpp"
        self.manifest["source_hashes"][name] = "0" * 64
        self.assert_rejected("SHA mismatch")
        self.manifest["source_hashes"][name] = contracts.sha(
            loader._SOURCE_PATHS[name].read_bytes())
        self.manifest["source_hashes"].pop(name)
        self.assert_rejected("source pins incomplete")

    def test_changed_report_or_artifact_bytes_reject_before_torch(self):
        digest = self.write_fixture()
        self.report_paths["synthetic"].write_bytes(
            self.report_paths["synthetic"].read_bytes() + b" ")
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            self.load_without_torch_or_library(digest)
        digest = self.write_fixture()
        self.library_path.write_bytes(b"changed library fixture")
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            self.load_without_torch_or_library(digest)

    def test_report_provenance_and_synthetic_validation_reject_before_torch(self):
        self.reports["synthetic"]["candidate_model_sha256"] = "0" * 64
        self.assert_rejected("report provenance differs")
        self.reports["synthetic"]["candidate_model_sha256"] = self.manifest["model_sha256"]
        self.reports["synthetic"]["validation"]["synthetic_frames"] = 239
        self.assert_rejected("synthetic parity incomplete")


if __name__ == "__main__":
    unittest.main()
