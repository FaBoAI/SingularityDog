"""File-only loader rejection gates and optional actual-artifact stateful parity."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from native_policy_overnight import contracts as c
from native_policy_overnight.lean_swing_core import SwingCore
from native_policy_overnight.view_cache import generate_core, verify_aliases
from native_policy_overnight.view_cache import loader


class ViewCacheLoaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        cls.torch = torch
        cls.generated_directory = tempfile.TemporaryDirectory()
        cls.generated_path = Path(cls.generated_directory.name) / "cached_view_core.py"
        cls.core, cls.source_proof = generate_core(cls.generated_path)

        class Policy(torch.nn.Module):
            inlined_graph = "sd_projection_fileonly_r1::project"

            def __init__(self, core):
                super().__init__()
                self.controller = core(1, "cpu", **c.OPTIONS)

            def reset(self, ids):
                self.controller.reset(ids)

        cls.policy = Policy

    @classmethod
    def tearDownClass(cls):
        cls.generated_directory.cleanup()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.manifest = self.root / "manifest-and-comparison.json"
        (self.root / "cached_view_core.py").write_bytes(self.generated_path.read_bytes())
        self.model_path = self.root / "cached_view_policy_fileonly.pt"
        self.model_path.write_bytes(b"pinned TorchScript bytes")
        self.baseline = self.policy(SwingCore)
        self.candidate = self.policy(self.core)
        self.baseline_proof = dict(manifest_sha256="a" * 64, model_sha256="b" * 64,
            library_sha256="c" * 64, hardware_opened=False, output_allowed=False,
            live_50hz_verified=False)
        maxima = {"target": 0.}
        for accessor in ("named_buffers", "named_parameters"):
            maxima.update((accessor + ":" + name, 0.) for name, value in getattr(self.baseline, accessor)()
                          if value.dtype.is_floating_point)
        self.report = dict(schema="native-view-cache-file-only-v1", status="PASS_FILE_ONLY_COMPARE",
            hardware_opened=False, output_allowed=False, approved_for_runtime=False,
            live_50hz_verified=False, existing_legacy_loader_schema_compatible=False,
            baseline_provenance=self.baseline_proof, baseline_manifest_path="/recorded/old/location.json",
            saved_records_sha256="d" * 64, source_script_sha256=loader._BUILD_SCRIPT_SHA256,
            variant_source_hashes=dict(loader._HELPER_PINS), errors=[],
            view_source_proof=copy.deepcopy(self.source_proof), model_file=self.model_path.name,
            model_sha256=c.sha(self.model_path.read_bytes()), environment=c.environment(self.torch),
            timed_outputs_and_final_state_exact=True,
            validation=dict(synthetic_frames=240, saved_stateful_calls_per_model=120,
                rejection_cases=[dict(case=case, reason=reason)
                                 for case, reason in loader._rejection_reasons().items()],
                max_errors=maxima, exact_named_state_parameters_targets=True,
                saved_reload_exact=True, input_mutation=False, aliases_after_reload_verified=True))

    def write_report(self, report=None):
        self.manifest.write_text(json.dumps(self.report if report is None else report, allow_nan=False))
        return c.sha(self.manifest.read_bytes())

    def load(self, digest):
        return loader.load_file_only_verified(self.manifest, expected_sha256=digest,
            baseline_manifest=self.root / "explicit-baseline.json", baseline_sha="a" * 64,
            bundle=self.root / "bundle")

    def rejected(self, report, pattern, *, baseline_loaded=False):
        digest = self.write_report(report)
        with mock.patch.object(loader, "load_verified", return_value=(self.baseline, self.baseline_proof)) as base:
            with mock.patch.object(self.torch.jit, "load", return_value=self.candidate) as jit:
                with self.assertRaisesRegex(ValueError, pattern):
                    self.load(digest)
                jit.assert_not_called()
                self.assertEqual(base.call_count, int(baseline_loaded))

    def test_valid_load_keeps_baseline_binding_reset_ownership_and_separate_loader_pin(self):
        self.candidate.controller.phase.fill_(.2)
        digest = self.write_report()
        threads = (self.torch.get_num_threads(), self.torch.get_num_interop_threads())
        with mock.patch.object(loader, "load_verified", return_value=(self.baseline, self.baseline_proof)) as base:
            with mock.patch.object(self.torch.jit, "load", return_value=self.candidate) as jit:
                model, proof = self.load(digest)
                base.assert_called_once_with(self.root / "explicit-baseline.json",
                    expected_manifest_sha256="a" * 64, bundle=self.root / "bundle")
                self.assertEqual(jit.call_args.kwargs, {"map_location": "cpu"})
                self.assertEqual(jit.call_args.args[0].getvalue(), self.model_path.read_bytes())
        self.assertIs(model, self.candidate)
        self.assertEqual(model.controller.phase.item(), 0.)
        self.assertTrue(verify_aliases(model.controller))
        self.assertEqual(proof["loader_source_sha256"], c.sha(Path(loader.__file__).read_bytes()))
        self.assertEqual(proof["schema"], "native-view-cache-file-only-loader-v1")
        self.assertEqual(proof["manifest_sha256"], digest)
        self.assertTrue(proof["validation_embedded_in_pinned_manifest"])
        self.assertEqual(proof["build_helper_source_hashes"], loader._HELPER_PINS)
        for key in ("hardware_opened", "output_allowed", "approved_for_runtime",
                    "live_50hz_verified", "existing_legacy_loader_schema_compatible"):
            self.assertIs(proof[key], False)
        self.assertEqual(threads, (self.torch.get_num_threads(), self.torch.get_num_interop_threads()))

    def test_loader_need_not_be_in_build_time_helper_map(self):
        self.report["variant_source_hashes"]["loader.py"] = "e" * 64
        digest = self.write_report()
        with mock.patch.object(loader, "load_verified", return_value=(self.baseline, self.baseline_proof)):
            with mock.patch.object(self.torch.jit, "load", return_value=self.candidate):
                _, proof = self.load(digest)
        self.assertNotIn("loader.py", proof["build_helper_source_hashes"])
        self.assertNotEqual(proof["loader_source_sha256"], "e" * 64)

    def test_manifest_hash_and_duplicate_json_reject_before_any_model_load(self):
        self.write_report()
        with mock.patch.object(loader, "load_verified") as base, mock.patch.object(self.torch.jit, "load") as jit:
            with self.assertRaisesRegex(ValueError, "SHA mismatch"):
                self.load("0" * 64)
            self.manifest.write_text('{"schema":"native-view-cache-file-only-v1","schema":"other"}')
            with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
                self.load(c.sha(self.manifest.read_bytes()))
            base.assert_not_called()
            jit.assert_not_called()

    def test_unknown_schema_status_errors_and_build_source_reject(self):
        for key, value, pattern in (
            ("schema", "native-policy-overnight-v1", "Unvalidated"),
            ("schema", "native-view-cache-file-only-v2", "Unvalidated"),
            ("status", "FAILED", "Unvalidated"),
            ("errors", ["partial comparison"], "comparison has errors"),
            ("source_script_sha256", "0" * 64, "build provenance")):
            with self.subTest(key=key, value=value):
                report = copy.deepcopy(self.report)
                report[key] = value
                self.rejected(report, pattern)

    def test_scope_flags_require_literal_false(self):
        for key in ("hardware_opened", "output_allowed", "approved_for_runtime",
                    "live_50hz_verified", "existing_legacy_loader_schema_compatible"):
            for value in (True, 0, None):
                with self.subTest(key=key, value=value):
                    report = copy.deepcopy(self.report)
                    report[key] = value
                    self.rejected(report, "unapproved file-only diagnostic")

    def test_helper_seed_transform_and_source_scope_drift_reject(self):
        for key, value in (("seed_core_sha256", "0" * 64),
                           ("generator_source_sha256", "0" * 64),
                           ("original_ast_exact_after_inverse_view_substitution", False),
                           ("output_allowed", 0)):
            with self.subTest(key=key):
                report = copy.deepcopy(self.report)
                report["view_source_proof"][key] = value
                self.rejected(report, "source proof differs")
        report = copy.deepcopy(self.report)
        report["variant_source_hashes"]["generator.py"] = "0" * 64
        self.rejected(report, "helper pins differ")
        for value in (2, True):
            report = copy.deepcopy(self.report)
            report["view_source_proof"]["declared_view_replacements"]["_cached_signs_expanded"] = value
            self.rejected(report, "transformation differs")

    def test_generated_source_cannot_change_even_with_a_resigned_source_proof(self):
        source = self.root / "cached_view_core.py"
        source.write_bytes(source.read_bytes() + b"# unrelated source edit\n")
        report = copy.deepcopy(self.report)
        report["view_source_proof"]["generated_core_sha256"] = c.sha(source.read_bytes())
        self.rejected(report, "transformation differs")
        self.rejected(self.report, "SHA mismatch")

    def test_current_helper_bytes_and_generated_source_symlinks_reject(self):
        helper = self.root / "helper"
        helper.mkdir()
        original = Path(loader.__file__).resolve().parent
        (helper / "__init__.py").write_bytes((original / "__init__.py").read_bytes())
        (helper / "generator.py").write_bytes((original / "generator.py").read_bytes() + b"# drift\n")
        with mock.patch.object(loader, "__file__", str(helper / "loader.py")):
            self.rejected(self.report, "SHA mismatch")
        source = self.root / "cached_view_core.py"
        source.unlink()
        source.symlink_to(self.generated_path)
        self.rejected(self.report, "nonsymlink")

    def test_model_bytes_names_and_symlinks_reject_before_baseline_load(self):
        report = copy.deepcopy(self.report)
        report["model_file"] = "../outside.pt"
        self.rejected(report, "filename within")
        self.model_path.write_bytes(b"changed model")
        self.rejected(self.report, "SHA mismatch")
        self.model_path.unlink()
        self.model_path.symlink_to(self.generated_path)
        self.rejected(self.report, "nonsymlink")

    def test_incomplete_or_nonexact_validation_rejects(self):
        for key, value in (("synthetic_frames", 239), ("saved_stateful_calls_per_model", 119),
                           ("exact_named_state_parameters_targets", False), ("saved_reload_exact", 1),
                           ("aliases_after_reload_verified", False), ("input_mutation", 0)):
            with self.subTest(key=key):
                report = copy.deepcopy(self.report)
                report["validation"][key] = value
                self.rejected(report, "validation missing|frame validation")
        report = copy.deepcopy(self.report)
        report["validation"]["max_errors"]["target"] = 1e-12
        self.rejected(report, "must be exact")
        report = copy.deepcopy(self.report)
        report["validation"]["max_errors"].pop("named_buffers:controller.phase")
        self.rejected(report, "named-state validation incomplete", baseline_loaded=True)
        report = copy.deepcopy(self.report)
        report["validation"]["rejection_cases"][1] = report["validation"]["rejection_cases"][0]
        self.rejected(report, "rejection reasons differ")
        report = copy.deepcopy(self.report)
        report["validation"]["rejection_cases"][0]["reason"] = "different guard"
        self.rejected(report, "rejection reasons differ")

    def test_baseline_provenance_and_target_mismatch_reject_before_candidate_load(self):
        report = copy.deepcopy(self.report)
        report["baseline_provenance"]["manifest_sha256"] = "0" * 64
        self.rejected(report, "baseline manifest pin differs")
        for key, value in (("library_sha256", "0" * 64), ("hardware_opened", 0)):
            report = copy.deepcopy(self.report)
            report["baseline_provenance"][key] = value
            self.rejected(report, "baseline provenance differs", baseline_loaded=True)
        report = copy.deepcopy(self.report)
        report["environment"]["torch_version"] = "different ABI"
        self.rejected(report, "target/PyTorch ABI differs", baseline_loaded=True)

    def test_baseline_loader_fail_closed_is_propagated(self):
        digest = self.write_report()
        with mock.patch.object(loader, "load_verified", side_effect=ValueError("Original source SHA mismatch")):
            with mock.patch.object(self.torch.jit, "load") as jit:
                with self.assertRaisesRegex(ValueError, "Original source SHA mismatch"):
                    self.load(digest)
                jit.assert_not_called()

    def test_candidate_requires_native_operator_and_buffer_aliases(self):
        digest = self.write_report()
        with mock.patch.object(loader, "load_verified", return_value=(self.baseline, self.baseline_proof)):
            with mock.patch.object(self.torch.jit, "load", return_value=self.candidate):
                self.candidate.inlined_graph = "other::operator"
                with self.assertRaisesRegex(ValueError, "native projection missing"):
                    self.load(digest)
                self.candidate.inlined_graph = "sd_projection_fileonly_r1::project"
                self.candidate.controller._cached_phase_column = self.candidate.controller.phase.clone().unsqueeze(1)
                with self.assertRaisesRegex(ValueError, "lost its buffer alias"):
                    self.load(digest)

    def test_reset_model_state_must_equal_verified_baseline(self):
        digest = self.write_report()
        self.candidate.controller.anchor.add_(.01)
        with mock.patch.object(loader, "load_verified", return_value=(self.baseline, self.baseline_proof)):
            with mock.patch.object(self.torch.jit, "load", return_value=self.candidate):
                with self.assertRaisesRegex(ValueError, "Exact tensor mismatch"):
                    self.load(digest)


_FIXTURE = {key: os.environ.get("SD_VIEW_CACHE_" + key) for key in
            ("MANIFEST", "MANIFEST_SHA256", "BUNDLE", "RECORDS", "RECORDS_SHA256",
             "VARIANT_MANIFEST", "VARIANT_MANIFEST_SHA256")}


@unittest.skipUnless(all(_FIXTURE.values()), "Explicit pinned baseline/candidate native artifacts required")
class NativeViewCacheLoaderTests(unittest.TestCase):
    def test_actual_loader_saved_recurrent_inputs_rejections_and_independent_reset(self):
        import torch
        from native_policy_overnight import load_verified
        from native_policy_overnight.verification import compare_state, compare_tensor, frame, rejections, reject_reason

        candidate, proof = loader.load_file_only_verified(_FIXTURE["VARIANT_MANIFEST"],
            expected_sha256=_FIXTURE["VARIANT_MANIFEST_SHA256"],
            baseline_manifest=_FIXTURE["MANIFEST"], baseline_sha=_FIXTURE["MANIFEST_SHA256"],
            bundle=_FIXTURE["BUNDLE"])
        baseline, _ = load_verified(_FIXTURE["MANIFEST"],
            expected_manifest_sha256=_FIXTURE["MANIFEST_SHA256"], bundle=_FIXTURE["BUNDLE"])
        with self.assertRaisesRegex(ValueError, "Unvalidated artifact manifest"):
            load_verified(_FIXTURE["VARIANT_MANIFEST"],
                expected_manifest_sha256=_FIXTURE["VARIANT_MANIFEST_SHA256"], bundle=_FIXTURE["BUNDLE"])
        records = c.strict_json(c.pinned(_FIXTURE["RECORDS"], _FIXTURE["RECORDS_SHA256"]))
        self.assertEqual(len(records), 20)
        maxima = {}
        with torch.inference_mode():
            ids = torch.tensor([0], dtype=torch.long)
            for repeat in range(6):
                for model in (baseline, candidate):
                    model.reset(ids)
                for index, row in enumerate(records):
                    self.assertEqual(row["cycle"], index + 1)
                    inputs = tuple(torch.tensor([row["observed"]["inputs"][key]], dtype=torch.float32)
                                   for key in c.INPUT_KEYS)
                    before = [value.clone() for value in inputs]
                    compare_tensor(torch, baseline(*inputs), candidate(*inputs), "target", maxima, exact=True)
                    compare_state(torch, baseline, candidate, maxima, exact=True)
                    self.assertTrue(all(torch.equal(a, b) for a, b in zip(before, inputs)))
                self.assertTrue(verify_aliases(candidate.controller))
            valid = frame(torch, baseline.controller.nominal.float().reshape(1, 12), 0)
            for name, inputs in rejections(torch, valid).items():
                reasons = []
                for model in (baseline, candidate):
                    model.reset(ids)
                    try:
                        model(*inputs)
                    except (ValueError, RuntimeError, torch.jit.Error) as error:
                        reasons.append(reject_reason(error))
                    else:
                        self.fail("Invalid input accepted: " + name)
                self.assertEqual(reasons[0], reasons[1], name)
                compare_state(torch, baseline, candidate, maxima, exact=True)
            other, _ = loader.load_file_only_verified(_FIXTURE["VARIANT_MANIFEST"],
                expected_sha256=_FIXTURE["VARIANT_MANIFEST_SHA256"],
                baseline_manifest=_FIXTURE["MANIFEST"], baseline_sha=_FIXTURE["MANIFEST_SHA256"],
                bundle=_FIXTURE["BUNDLE"])
            self.assertEqual(other.controller.phase.item(), 0.)
            self.assertNotEqual(candidate.controller.phase.untyped_storage().data_ptr(),
                                other.controller.phase.untyped_storage().data_ptr())
            self.assertTrue(verify_aliases(other.controller))
        self.assertTrue(all(value == 0. for value in maxima.values()))
        self.assertTrue(proof["diagnostic_only"])
        self.assertFalse(proof["approved_for_runtime"])


if __name__ == "__main__":
    unittest.main()
