"""File-only adversarial fixtures for the separately selected FK diagnostic loader.

No Torch module, native library, model, serial port, or robot is loaded here.
The scalar dependency and source transformers are mocked at their own boundaries;
the new manifest, data, reference reader, and rejection logic remain real.
"""
import builtins
import contextlib
import copy
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock

from native_policy_overnight.target_tail_fk_cache import diagnostic_loader as loader
from native_policy_overnight.target_tail_fk_cache import generator as fk_generator
from native_policy_overnight.target_tail_fusion import generator as first_generator


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class DiagnosticLoaderFileTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.paths = {}
        self.contents = {}
        for name in loader.REFERENCES:
            self.paths[name] = self.root / (name + ".json")
        self.paths.update({"manifest": self.root / "manifest.json"})
        self.loader_path = self.root / "diagnostic_loader.py"
        self.loader_path.write_bytes(b"# executing source fixture\n")
        patcher = mock.patch.object(loader, "__file__", str(self.loader_path))
        patcher.start(); self.addCleanup(patcher.stop)
        self.executing_patch = mock.patch.object(loader, "_executing_sources")
        self.executing_patch.start(); self.addCleanup(self.executing_patch.stop)
        self.replay_patch = mock.patch.object(loader, "_executing_replay")
        self.replay_patch.start(); self.addCleanup(self.replay_patch.stop)
        for module, result in ((first_generator, b"target fixture"), (fk_generator, b"fk fixture")):
            patcher = mock.patch.object(module, "transform", return_value=result)
            patcher.start(); self.addCleanup(patcher.stop)
        self.contents["scalar_manifest"] = b"scalar fixture"
        self.contents["baseline_manifest"] = b"baseline fixture"
        self.contents["model"] = b"model fixture, not a Torch archive"
        self.contents["library"] = b"library fixture, not executable"
        if "replay_helper" in loader.REFERENCES:
            self.contents["replay_helper"] = b"replay helper fixture"
        self.scalar_source = {
            "schema": "native-step-scalar-file-only-loader-v1",
            "manifest_sha256": sha(self.contents["scalar_manifest"]),
            "model_sha256": "1" * 64, "library_sha256": "2" * 64,
            "baseline_provenance": {"manifest_sha256": sha(self.contents["baseline_manifest"])},
            "hardware_opened": False, "output_allowed": False,
            "approved_for_runtime": False, "live_50hz_verified": False,
        }
        self.environment = {"system": "Linux", "machine": "aarch64",
                            "torch_version": "fixture", "cxx11_abi": True}
        self.scalar_data = {"model_sha256": "1" * 64, "library_sha256": "2" * 64,
                            "environment": self.environment}
        patcher = mock.patch.object(loader.scalar_loader, "_prevalidate",
            return_value=(self.paths["scalar_manifest"], self.scalar_data, {}))
        self.scalar_prevalidate = patcher.start(); self.addCleanup(patcher.stop)
        self.source_refs = {}
        for index, name in enumerate(loader.SOURCE_NAMES):
            path = self.root / ("source_%02d.py" % index)
            raw = ("# " + name).encode()
            if name == "target_tail_fk_cache/target.cpp":
                raw += b'\n.findSchemaOrThrow("sd_step_fileonly_r1::step", "")\n'
                raw += b'.findSchemaOrThrow("sd_projection_fileonly_r1::project", "")\n'
            path.write_bytes(raw)
            self.source_refs[loader.PREFIX + name] = self.ref(path)
        self.source_manifest = {"files": {k: v["sha256"] for k, v in self.source_refs.items()}}
        self.generated = {}
        for name, raw in {"cached.py": b"cached fixture", "step.py": b"step fixture",
                          "target.py": b"target fixture", "fk_cache.py": b"fk fixture"}.items():
            path = self.root / name; path.write_bytes(raw)
            self.generated[name] = self.ref(path)
        self.report = {
            "schema": "singularitydog.saved-input-target-fk-cache-profile.v1",
            "status": "PASS_FILE_ONLY_SCALAR_TAIL_FK_CACHE_COMPARE", "cycles": 501,
            "environment": self.environment, "model_source": self.scalar_source,
            "candidate_source_sha256": self.source_manifest["files"].copy(),
            "hardware_opened": False, "output_allowed": False, "approved_for_runtime": False,
            "live_50hz_verified": False, "actual_controller_qualification": False,
            "saved_validation": {"saved_recurrent_calls": 501,
                "policies": ["current_scalar", "first_target_tail", "fk_cache_target_tail"],
                "saved_output_actor_observation_bits_exact": True, "all_named_state_bits_each_call": True,
                "input_bits_preserved": True, "same_partial_state_after_rejections": True,
                "rejection_count": 26, "rejections": [{"case": k, "reason": v}
                    for k, v in loader.scalar_loader._rejection_reasons().items()]},
            "synthetic_validation": {name: {"frames": 240, "reset_before": [0, 125],
                "all_named_state_bits_exact": True} for name in ("first_target_tail", "fk_cache_target_tail")},
            "first_target_tail": {"generated_sources": {name: self.generated[name]["sha256"]
                for name in ("cached.py", "step.py", "target.py")}},
            "fk_cache_target_tail": {"model_sha256": sha(self.contents["model"]),
                "library_sha256": sha(self.contents["library"]),
                "library_source_sha256": self.source_refs[loader.PREFIX + "target_tail_fk_cache/target.cpp"]["sha256"],
                "same_scalar_projection_actor_observation": True, "undeclared_methods_exact": True,
                "generated_sources": {"fk_cache.py": self.generated["fk_cache.py"]["sha256"]}},
            "cpp_delta": {
                "first_tail_source_sha256": self.source_refs[loader.PREFIX + "target_tail_fusion/target.cpp"]["sha256"],
                "fk_cache_source_sha256": self.source_refs[loader.PREFIX + "target_tail_fk_cache/target.cpp"]["sha256"],
                "inverse_bytes_exact": True},
        }
        self.saved_report = {"status": "COMPLETE_DIAGNOSTIC", "cycles_completed": 501, "errors": [],
            "scalar_step_model_source": self.scalar_source, "motor_enable_sent": False,
            "learned_targets_sent": False, "approved_for_runtime": False, "full_controller_50Hz_verified": False}
        keys = ("gyro_body_rad_s", "gravity_body_unit", "command", "q_model_rad", "dq_model_rad_s", "h_hypothesis12")
        inputs = {key: [0.0] * n for key, n in zip(keys, (3, 3, 3, 12, 12, 12))}
        inputs["gravity_body_unit"] = [0.0, 0.0, -1.0]
        self.records = [{"cycle": i + 1, "observed": {"status": "TICK_OBSERVED_NO_OUTPUT",
            "tick_index": i, "output_allowed": False, "inputs": copy.deepcopy(inputs),
            "q_target_rad_diagnostic_only": [0.0] * 12, "actor_residual12": [0.0] * 12,
            "observation74": [0.0] * 74}} for i in range(501)]
        self.audit = {"status": "PASS_TARGET_RAW_TIMING_SOURCE_AND_RECEIPT_AUDIT",
            "all30_state_bits_proven_by_harness": True, "reject_cases": 26,
            "synthetic_per_candidate": 240, "hardware_opened": False, "output_allowed": False,
            "artifact_sha256": {"report-fk-cache-artifacts/fk_cache_tail_fileonly.pt": sha(self.contents["model"]),
                "report-fk-cache-artifacts/target_tail_fk_cache.so": sha(self.contents["library"])}}
        self.raw_audit = {"status": "PASS_RAW_EVIDENCE", "cycles_audited": 501}
        self.manifest = {"schema": loader.SCHEMA, "status": loader.STATUS,
            **dict.fromkeys(loader.FALSE_FLAGS, False), "source_references": self.source_refs,
            "integration_sources": {"diagnostic_loader.py": self.ref(self.loader_path)},
            "generated_source_references": self.generated}

    def ref(self, path):
        return {"path": str(path), "sha256": sha(path.read_bytes())}

    def write(self, name, data):
        raw = data if isinstance(data, bytes) else json.dumps(data, allow_nan=False).encode()
        self.paths[name].write_bytes(raw)
        return self.ref(self.paths[name])

    def write_fixture(self, manifest_mutation=None):
        refs = {name: self.write(name, data) for name, data in self.contents.items()}
        refs["saved_report"] = self.write("saved_report", self.saved_report)
        refs["saved_records"] = self.write("saved_records", self.records)
        self.raw_audit["input_file_sha256"] = {refs[k]["path"]: refs[k]["sha256"]
            for k in ("saved_report", "saved_records")}
        refs["saved_raw_audit"] = self.write("saved_raw_audit", self.raw_audit)
        refs["candidate_source_manifest"] = self.write("candidate_source_manifest", self.source_manifest)
        for key, field in (("saved_report", "original_report_sha256"), ("saved_records", "original_records_sha256"),
                          ("saved_raw_audit", "raw_audit_sha256"), ("candidate_source_manifest", "candidate_source_manifest_sha256")):
            self.report[field] = refs[key]["sha256"]
        if "replay_helper" in refs:
            self.report["replay_helper_sha256"] = refs["replay_helper"]["sha256"]
        refs["file_only_report"] = self.write("file_only_report", self.report)
        self.audit["report_sha256"] = refs["file_only_report"]["sha256"]
        refs["file_only_audit"] = self.write("file_only_audit", self.audit)
        self.manifest["references"] = refs
        if manifest_mutation:
            manifest_mutation(self.manifest)
        self.write("manifest", self.manifest)
        return sha(self.paths["manifest"].read_bytes())

    def kwargs(self, digest):
        return dict(expected_sha256=digest, scalar_manifest=self.paths["scalar_manifest"],
            scalar_sha=sha(self.contents["scalar_manifest"]), baseline_manifest=self.paths["baseline_manifest"],
            baseline_sha=sha(self.contents["baseline_manifest"]))

    def run_plan(self, digest):
        original = builtins.__import__
        def checked(name, *args, **kwargs):
            if name == "torch" or name.startswith("torch."):
                raise AssertionError("File preflight imported Torch")
            return original(name, *args, **kwargs)
        with mock.patch("builtins.__import__", side_effect=checked), mock.patch.object(
                loader.scalar_loader, "load_file_only_verified", side_effect=AssertionError("Model load in PLAN")):
            return loader.plan(self.paths["manifest"], **self.kwargs(digest))

    def reject(self, pattern, mutation=None):
        digest = self.write_fixture(mutation)
        with self.assertRaisesRegex(ValueError, pattern):
            self.run_plan(digest)

    def test_valid_plan_checks_all_refs_without_torch(self):
        result = self.run_plan(self.write_fixture())
        self.assertEqual(result["backend"], "pinned_fk_cache_cpp")
        self.assertIs(result["torch_or_native_loaded"], False)
        self.assertEqual(result["validated_saved_calls"], 501)
        self.assertTrue(all(result[k] is False for k in loader.FALSE_FLAGS))
        self.scalar_prevalidate.assert_called_once()

    def test_wrong_manifest_pin_duplicate_key_and_nonfinite_reject(self):
        self.write_fixture()
        with self.assertRaises(ValueError): self.run_plan("0" * 64)
        for raw in (b'{"schema":1,"schema":2}', b'{"bad":NaN}'):
            self.paths["manifest"].write_bytes(raw)
            with self.assertRaises(ValueError): self.run_plan(sha(raw))

    def test_manifest_scope_requires_literal_false(self):
        for flag in loader.FALSE_FLAGS:
            for bad in (True, 0, None):
                with self.subTest(flag=flag, bad=bad):
                    self.reject("Unapproved", lambda m: m.__setitem__(flag, bad))
                    self.manifest[flag] = False

    def test_report_scope_requires_literal_false(self):
        for flag in ("hardware_opened", "output_allowed", "approved_for_runtime", "live_50hz_verified", "actual_controller_qualification"):
            self.report[flag] = True; self.reject("scope"); self.report[flag] = False

    def test_missing_extra_and_wrong_selected_ref_reject(self):
        self.reject("references", lambda m: m["references"].pop("model"))
        self.reject("references", lambda m: m["references"].__setitem__("unrelated", self.ref(self.loader_path)))
        other = self.root / "other_scalar.json"; other.write_bytes(b"other scalar")
        self.reject("selections", lambda m: m["references"].__setitem__("scalar_manifest", self.ref(other)))

    def test_reference_paths_must_be_absolute_nonsymlink_regular(self):
        for path in ("relative", str(self.root / ".." / "outside")):
            with self.subTest(path=path):
                self.reject("reference", lambda m: m["references"]["model"].__setitem__("path", path))
        link = self.root / "link.pt"; link.symlink_to(self.paths["model"])
        self.reject("reference", lambda m: m["references"]["model"].__setitem__("path", str(link)))

    def test_fifo_rejected_nonblocking_without_device(self):
        fifo = self.root / "fifo"; os.mkfifo(fifo)
        with self.assertRaisesRegex(ValueError, "regular"):
            loader.read({"path": str(fifo), "sha256": "0" * 64})

    def test_changed_library_and_source_after_pin_reject(self):
        digest = self.write_fixture(); self.paths["library"].write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "SHA256"): self.run_plan(digest)
        digest = self.write_fixture(); path = Path(next(iter(self.source_refs.values()))["path"])
        path.write_bytes(b"changed source")
        with self.assertRaisesRegex(ValueError, "SHA256"): self.run_plan(digest)

    def test_all24_sources_and_executing_loader_binding_required(self):
        self.reject("24", lambda m: m["source_references"].pop(next(iter(m["source_references"]))))

    def test_executing_loader_is_separate_from_candidate_sources(self):
        path = self.root / "other_loader.py"; path.write_bytes(b"unexecuted")
        self.reject("Executing loader", lambda m: m["integration_sources"].__setitem__("diagnostic_loader.py", self.ref(path)))

    def test_saved501_and_order_rejected(self):
        self.records[-1]["cycle"] = 500; self.reject("order")
        self.records[-1]["cycle"] = 501
        self.records.pop(); self.reject("501")

    def test_saved_vectors_shapes_nonfinite_and_bool_rejected(self):
        row = self.records[0]["observed"]["inputs"]
        key = "gyro_body_rad_s"
        for bad in ([0.0] * 2, [False, 0, 0], [1e100, 0, 0]):
            row[key] = bad; self.reject("width|scalar|float32")
        row[key] = [0, 0, 0]
        self.records[0]["observed"]["output_allowed"] = 0; self.reject("observation")

    def test_saved_validation_missing_bit_or_partial_state_proof_rejects(self):
        v = self.report["saved_validation"]
        for key in ("saved_output_actor_observation_bits_exact", "all_named_state_bits_each_call", "input_bits_preserved", "same_partial_state_after_rejections"):
            v[key] = False; self.reject("proof"); v[key] = True

    def test_rejection_count_reason_and_duplicate_case_reject(self):
        v = self.report["saved_validation"]
        v["rejection_count"] = 25; self.reject("proof"); v["rejection_count"] = 26
        saved = copy.deepcopy(v["rejections"])
        v["rejections"][0]["reason"] = "other"; self.reject("26")
        v["rejections"] = saved
        v["rejections"][0] = v["rejections"][1]; self.reject("26")

    def test_both_synthetic240_and_reset_required(self):
        for name in ("first_target_tail", "fk_cache_target_tail"):
            row = self.report["synthetic_validation"][name]
            row["frames"] = 239; self.reject("Synthetic"); row["frames"] = 240
            row["reset_before"] = [0]; self.reject("Synthetic"); row["reset_before"] = [0, 125]

    def test_candidate_identity_and_undeclared_method_proof_reject(self):
        row = self.report["fk_cache_target_tail"]
        row["model_sha256"] = "0" * 64; self.reject("artifact")
        row["model_sha256"] = sha(self.contents["model"])
        row["undeclared_methods_exact"] = False; self.reject("artifact")

    def test_transitive_cpp_proof_missing_or_wrong_pin_rejects_before_torch(self):
        original = copy.deepcopy(self.report["cpp_delta"])
        for bad in (None, {}, {**original, "first_tail_source_sha256": "0" * 64},
                    {**original, "fk_cache_source_sha256": "0" * 64},
                    {**original, "inverse_bytes_exact": False},
                    {**original, "inverse_bytes_exact": 1}):
            self.report["cpp_delta"] = bad
            self.reject("Transitive C\\+\\+")
        self.report["cpp_delta"] = original

    def test_library_source_pin_is_bound_to_transitive_candidate_cpp(self):
        self.report["fk_cache_target_tail"]["library_source_sha256"] = "0" * 64
        self.reject("Transitive C\\+\\+")

    def test_missing_duplicate_or_wrong_cpp_dispatcher_rejects(self):
        key = loader.PREFIX + "target_tail_fk_cache/target.cpp"
        path = Path(self.source_refs[key]["path"])
        valid = path.read_bytes()
        one = b'.findSchemaOrThrow("sd_step_fileonly_r1::step", "")'
        for bad in (valid.replace(one, b""), valid + one,
                    valid.replace(one, b'.findSchemaOrThrow("changed::step", "")')):
            path.write_bytes(bad)
            self.source_refs[key] = self.ref(path)
            self.report["cpp_delta"]["fk_cache_source_sha256"] = self.source_refs[key]["sha256"]
            self.report["fk_cache_target_tail"]["library_source_sha256"] = self.source_refs[key]["sha256"]
            with self.assertRaisesRegex(ValueError, "dispatcher dependency"):
                loader._cpp_dependencies(self.report, self.source_refs)

    def test_generated_source_inverse_and_missing_member_reject(self):
        with mock.patch.object(fk_generator, "transform", return_value=b"wrong"):
            self.reject("inverse")
        self.reject("four|names", lambda m: m["generated_source_references"].pop("cached.py"))

    def test_independent_audit_status_and_artifact_sha_required(self):
        self.audit["status"] = "INCOMPLETE"; self.reject("audit")
        self.audit["status"] = "PASS_TARGET_RAW_TIMING_SOURCE_AND_RECEIPT_AUDIT"
        self.audit["artifact_sha256"]["report-fk-cache-artifacts/fk_cache_tail_fileonly.pt"] = "0" * 64
        self.reject("artifact pins")

    def test_original_raw_audit_must_cover501(self):
        self.raw_audit["cycles_audited"] = 500; self.reject("raw audit")

    def test_scalar_identity_and_environment_are_revalidated(self):
        self.scalar_data["model_sha256"] = "0" * 64; self.reject("scalar identity")
        self.scalar_data["model_sha256"] = "1" * 64
        self.scalar_data["environment"] = {"system": "different"}; self.reject("scalar identity")

    def test_integration_source_tamper_is_rejected(self):
        digest = self.write_fixture(); self.loader_path.write_bytes(b"mutated code")
        with self.assertRaisesRegex(ValueError, "SHA256"): self.run_plan(digest)

    def test_replay_helper_and_baseline_bytes_are_checked_in_plan(self):
        for key in ("replay_helper", "baseline_manifest"):
            digest = self.write_fixture()
            self.paths[key].write_bytes(b"changed dependency")
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "SHA256"):
                self.run_plan(digest)

    def test_create_manifest_returns_independent_unapproved_tree(self):
        self.write_fixture()
        groups = {key: self.manifest[key] for key in ("references", "source_references",
            "generated_source_references", "integration_sources")}
        result = loader.create_manifest(**groups)
        self.assertTrue(all(result[k] is False for k in loader.FALSE_FLAGS))
        result["references"]["model"]["sha256"] = "0" * 64
        self.assertNotEqual(result["references"]["model"], groups["references"]["model"])

    def test_mutation_after_initial_read_rejects_final_rehash(self):
        digest = self.write_fixture()
        def changed(*args):
            self.paths["model"].write_bytes(b"changed during verification")
            return self.paths["scalar_manifest"], self.scalar_data, {}
        self.scalar_prevalidate.side_effect = changed
        with self.assertRaisesRegex(ValueError, "SHA256"):
            self.run_plan(digest)

    def test_executing_dependencies_are_verified_not_just_retained_copies(self):
        self.executing_patch.stop()
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "SHA256"):
            loader._executing_sources(self.source_refs)

    def test_executing_replay_must_match_selected_reference_bytes(self):
        self.replay_patch.stop()
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "SHA256"):
            loader._executing_replay(self.ref(self.paths["replay_helper"]))

    def run_fake_load(self, *, namespace=False, env=None, mutate_library=False):
        digest = self.write_fixture()
        candidate = types.SimpleNamespace()
        candidate.eval = lambda: candidate
        native_namespace = types.SimpleNamespace()
        if namespace:
            native_namespace.target = object()
        calls = []
        def load_library(path):
            calls.append(path)
            if mutate_library:
                self.paths["library"].write_bytes(b"mutated at native registration")
        fake_torch = types.SimpleNamespace(ops=types.SimpleNamespace(
            sd_target_tail_fk_cache_fileonly_r1=native_namespace, load_library=load_library),
            jit=types.SimpleNamespace(load=mock.Mock(return_value=candidate)))
        with mock.patch.dict(sys.modules, {"torch": fake_torch}), mock.patch.object(
            loader, "environment", return_value=self.environment if env is None else env), mock.patch.object(
            loader.scalar_loader, "load_file_only_verified", return_value=(object(), self.scalar_source)) as baseline, mock.patch.object(
            loader, "_validate_models") as model_validation:
            result = loader.load_diagnostic_verified(self.paths["manifest"],
                **self.kwargs(digest), bundle=self.root / "unused-bundle")
            self.assertEqual(calls, [str(self.paths["library"])])
            self.assertEqual(fake_torch.jit.load.call_args.kwargs, {"map_location": "cpu"})
            self.assertEqual(fake_torch.jit.load.call_args.args[0].getvalue(), self.contents["model"])
            baseline.assert_called_once(); model_validation.assert_called_once()
            return result

    def test_fake_load_returns_candidate_with_separate_unapproved_provenance(self):
        model, proof = self.run_fake_load()
        self.assertEqual(proof["schema"], "singularitydog.fk-cache-stop-diagnostic-loader.v1")
        self.assertEqual(proof["original_scalar_dependency"], self.scalar_source)
        self.assertEqual(proof["model_sha256"], sha(self.contents["model"]))
        self.assertNotEqual(proof["model_sha256"], self.scalar_source["model_sha256"])
        self.assertTrue(all(proof[k] is False for k in loader.FALSE_FLAGS))

    def test_fake_load_environment_or_existing_namespace_rejects(self):
        with self.assertRaisesRegex(ValueError, "ABI"):
            self.run_fake_load(env={"system": "changed"})
        with self.assertRaisesRegex(ValueError, "registered"):
            self.run_fake_load(namespace=True)

    def test_fake_load_library_mutation_rejects_before_model(self):
        with self.assertRaisesRegex(ValueError, "SHA256"):
            self.run_fake_load(mutate_library=True)


class FakeMethod:
    def __init__(self, code):
        self.code = code
        self.calls = []

    def __call__(self, ids):
        self.calls.append(ids)


class FakeModule:
    def __init__(self, methods):
        self._methods = {name: FakeMethod(code) for name, code in methods.items()}
        self._c = types.SimpleNamespace(
            _method_names=lambda: list(self._methods),
            _get_method=lambda name: self._methods[name])
        for name, method in self._methods.items():
            setattr(self, name, method)


def fake_model():
    model = FakeModule({"forward": "policy.forward", "reset": "policy.reset"})
    model.controller = FakeModule({"step_target": "candidate-specific", "reset": "core.reset"})
    model.actor = FakeModule({"forward": "actor.forward"})
    model.inlined_graph = "\n".join(loader.OPERATORS)
    return model


class DiagnosticModelBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.torch = types.SimpleNamespace(tensor=lambda v, **kw: v, long="long",
            inference_mode=contextlib.nullcontext)
        self.scalar, self.candidate = fake_model(), fake_model()

    def validate(self):
        with mock.patch.object(loader, "verify_aliases") as aliases, mock.patch.object(loader, "_state_bits") as states:
            loader._validate_models(self.torch, self.scalar, self.candidate)
            return aliases, states

    def test_independent_reset_aliases_and_state_check_happen(self):
        aliases, states = self.validate()
        self.assertEqual(aliases.call_count, 4)
        self.assertEqual(self.scalar.reset.calls, [[0]])
        self.assertEqual(self.candidate.reset.calls, [[0]])
        states.assert_called_once_with(self.torch, self.scalar, self.candidate)

    def test_python_attribute_collision_uses_registered_methods_for_all_modules(self):
        for scalar, candidate in ((self.scalar, self.candidate),
                                  (self.scalar.controller, self.candidate.controller),
                                  (self.scalar.actor, self.candidate.actor)):
            scalar._methods["__len__"] = FakeMethod("registered length")
            candidate._methods["__len__"] = FakeMethod("registered length")
            # Ordinary Python attributes can be unrelated wrappers without code.
            scalar.__len__ = lambda: 1
            candidate.__len__ = lambda: 2
            self.assertFalse(hasattr(scalar.__len__, "code"))
        self.validate()

    def test_registered_code_difference_rejects_even_when_visible_attributes_match(self):
        for attribute, expected in ((None, "policy executable"),
                                    ("controller", "Undeclared controller"),
                                    ("actor", "actor executable")):
            with self.subTest(module=attribute):
                self.scalar, self.candidate = fake_model(), fake_model()
                scalar = getattr(self.scalar, attribute) if attribute else self.scalar
                candidate = getattr(self.candidate, attribute) if attribute else self.candidate
                scalar._methods["__len__"] = FakeMethod("registered length 1")
                candidate._methods["__len__"] = FakeMethod("registered length 2")
                scalar.__len__ = candidate.__len__ = FakeMethod("identical visible code")
                with self.assertRaisesRegex(ValueError, expected):
                    self.validate()

    def test_missing_registered_method_is_not_replaced_with_python_attribute(self):
        self.candidate.actor._c._get_method = mock.Mock(side_effect=RuntimeError("Missing registered method"))
        self.assertIsInstance(self.candidate.actor.forward.code, str)
        with self.assertRaisesRegex(RuntimeError, "Missing registered method"):
            self.validate()

    def test_same_instance_rejects(self):
        self.candidate = self.scalar
        with self.assertRaisesRegex(ValueError, "Independent"): self.validate()

    def test_changed_actor_or_top_forward_code_rejected(self):
        for target in (self.candidate, self.candidate.actor):
            old = target.forward.code; target.forward.code += " + changed"
            with self.assertRaisesRegex(ValueError, "executable"): self.validate()
            target.forward.code = old

    def test_only_torch_mangle_suffix_can_differ(self):
        self.scalar.forward.code = "__torch__.___torch_mangle_2.Class"
        self.candidate.forward.code = "__torch__.___torch_mangle_719.Class"
        self.validate()
        self.candidate.forward.code += " + operation"
        with self.assertRaisesRegex(ValueError, "executable"): self.validate()

    def test_missing_extra_controller_methods_and_undeclared_code_reject(self):
        self.candidate.controller._methods["other"] = "other"
        with self.assertRaisesRegex(ValueError, "methods"): self.validate()
        self.candidate.controller._methods.pop("other")
        self.candidate.controller.reset.code = "different reset"
        with self.assertRaisesRegex(ValueError, "Undeclared"): self.validate()

    def test_operator_graph_missing_or_alias_state_error_reject(self):
        self.candidate.inlined_graph = "\n".join(loader.OPERATORS[:-1])
        with self.assertRaisesRegex(ValueError, "graph"): self.validate()
        self.candidate.inlined_graph = "\n".join(loader.OPERATORS)
        with mock.patch.object(loader, "verify_aliases", side_effect=ValueError("alias mismatch")):
            with self.assertRaisesRegex(ValueError, "alias"): loader._validate_models(self.torch, self.scalar, self.candidate)
        with mock.patch.object(loader, "verify_aliases"), mock.patch.object(loader, "_state_bits", side_effect=ValueError("bits mismatch")):
            with self.assertRaisesRegex(ValueError, "bits"): loader._validate_models(self.torch, self.scalar, self.candidate)

    def test_opaque_fk_only_graph_accepts_with_transitive_proof_checked_preload(self):
        self.candidate.inlined_graph = loader.OPERATORS[-1]
        aliases, states = self.validate()
        self.assertEqual(aliases.call_count, 4)
        states.assert_called_once()

    def test_old_target_graph_is_rejected_even_with_fk_namespace_present(self):
        self.candidate.inlined_graph += "\nsd_target_tail_fileonly_r1::target"
        with self.assertRaisesRegex(ValueError, "graph"):
            self.validate()


class FakeTensor:
    def __init__(self, value, pointer, *, finite=True):
        self.value, self.pointer, self.finite = value, pointer, finite
        self.shape, self.dtype, self.device = (1,), "float32", "cpu"
    def numel(self): return 1
    def untyped_storage(self): return types.SimpleNamespace(data_ptr=lambda: self.pointer)
    def contiguous(self): return self
    def reshape(self, _): return self
    def view(self, _): return struct.pack("=f", self.value)


class DiagnosticStateBitsTests(unittest.TestCase):
    def setUp(self):
        self.left = FakeTensor(0.0, 101)
        self.right = FakeTensor(0.0, 202)
        self.one = types.SimpleNamespace(named_buffers=lambda: [("state", self.left)], named_parameters=lambda: [])
        self.two = types.SimpleNamespace(named_buffers=lambda: [("state", self.right)], named_parameters=lambda: [])
        self.torch = types.SimpleNamespace(uint8="uint8", equal=lambda a, b: a == b,
            isfinite=lambda v: types.SimpleNamespace(all=lambda: v.finite))

    def test_exact_independent_bits_accept_but_signed_zero_rejects(self):
        loader._state_bits(self.torch, self.one, self.two)
        self.right.value = -0.0
        with self.assertRaisesRegex(ValueError, "bits"):
            loader._state_bits(self.torch, self.one, self.two)

    def test_storage_alias_and_missing_state_reject(self):
        self.right.pointer = self.left.pointer
        with self.assertRaisesRegex(ValueError, "storage"):
            loader._state_bits(self.torch, self.one, self.two)
        self.right.pointer = 202
        self.two.named_buffers = lambda: []
        with self.assertRaisesRegex(ValueError, "fields"):
            loader._state_bits(self.torch, self.one, self.two)

    def test_dtype_device_shape_and_nonfinite_reject(self):
        for field, bad in (("dtype", "float64"), ("device", "cuda"), ("shape", (2,)), ("finite", False)):
            old = getattr(self.right, field); setattr(self.right, field, bad)
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "bits"):
                loader._state_bits(self.torch, self.one, self.two)
            setattr(self.right, field, old)


if __name__ == "__main__":
    unittest.main()
