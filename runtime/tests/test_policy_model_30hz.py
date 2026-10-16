"""Immutable release, pure-file closure and real CPU model tests; no hardware."""
import builtins
import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_model_30hz as model30

try:
    import torch
except ImportError:
    torch = None

BUNDLE = Path(__file__).resolve().parents[3] / "RS05_30Hz_Model_20261013"


@unittest.skipUnless(BUNDLE.exists(), "The pinned30Hz distribution is not present")
class ReleasePlanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for relative in model30.ARTIFACT_FILES.values():
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(BUNDLE / relative, target)
        manifest = json.loads((BUNDLE / "source/source_manifest30.json").read_text())
        for relative in manifest:
            target = self.root / "source" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(BUNDLE / "source" / relative, target)
        self.artifacts = model30.bundle_artifacts(self.root)

    def documents(self):
        return {name: json.loads(Path(self.artifacts[name]["path"]).read_text())
                for name in ("control_contract", "export", "train_completion", "source_manifest")}

    def test_bundle_pins_do_not_read_or_self_trust_files(self):
        with patch.object(Path, "read_bytes", side_effect=AssertionError("No read")):
            result = model30.bundle_artifacts(self.root / "not-yet-installed")
        self.assertEqual(result["controller"]["sha256"], model30.RELEASE_SHA256["controller"])
        self.assertTrue(Path(result["controller"]["path"]).is_absolute())

    def test_real_release_plan_without_torch_import_or_model_execution(self):
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name == "torch" or name.startswith("torch."):
                raise AssertionError("PLAN must not import Torch")
            return original(name, *args, **kwargs)
        with patch("builtins.__import__", side_effect=guarded):
            result = model30.plan(self.artifacts)
        self.assertEqual(result["policy_dt_s"], 1 / 30)
        self.assertEqual(result["source_members_verified"], 13)
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["model_artifact_grants_output"])
        self.assertFalse(result["evaluation_all_intervals_completed"])
        self.assertEqual(model30.validate_plan(result), result)

    def test_each_artifact_changed_bytes_rejected(self):
        for name, artifact in self.artifacts.items():
            path = Path(artifact["path"])
            before = path.read_bytes()
            with self.subTest(name=name):
                path.write_bytes(before + b" ")
                try:
                    with self.assertRaisesRegex(ValueError, "SHA256 differs"):
                        model30.plan(self.artifacts)
                finally:
                    path.write_bytes(before)

    def test_changed_artifact_cannot_be_self_repinned(self):
        path = Path(self.artifacts["controller"]["path"])
        path.write_bytes(path.read_bytes() + b" ")
        self.artifacts["controller"]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "selected30Hz release|selected 30Hz release"):
            model30.plan(self.artifacts)

    def test_old50hz_controller_or_extra_backend_selection_rejected(self):
        old = copy.deepcopy(self.artifacts)
        old["controller"]["sha256"] = "7a4f6ee4d1f4ae165356d91ab160fef5ac67d2958e8d717846ebc10be9d4811a"
        with self.assertRaisesRegex(ValueError, "selected 30Hz release"):
            model30.plan(old)
        for key in ("scalar_step_manifest", "target_fk_manifest", "checked_model_manifest", "model_backend"):
            selected = dict(self.artifacts, **{key: "legacy"})
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "no legacy backend"):
                model30.plan(selected)

    def test_frozen_source_member_changed_rejected(self):
        source = self.root / "source/swing_core.py"
        source.write_bytes(source.read_bytes() + b"\n")
        with self.assertRaisesRegex(ValueError, "SHA256 differs: source/swing_core.py"):
            model30.plan(self.artifacts)

    def test_no_historical_gpu_path_is_opened(self):
        # The distribution resides elsewhere than the checkpoint_path recorded
        # on the GPU.  Real file closure still succeeds with local SHA bindings.
        reports = self.documents()
        self.assertNotEqual(reports["export"]["checkpoint_path"], self.artifacts["checkpoint"]["path"])
        self.assertEqual(model30.plan(self.artifacts)["training_updates"], 500)

    def test_symlink_and_relative_path_rejected(self):
        controller = Path(self.artifacts["controller"]["path"])
        link = self.root / "controller-link.pt"
        link.symlink_to(controller)
        self.artifacts["controller"]["path"] = str(link)
        with self.assertRaisesRegex(ValueError, "Absolute regular file"):
            model30.plan(self.artifacts)
        self.artifacts["controller"]["path"] = "relative-controller.pt"
        with self.assertRaisesRegex(ValueError, "Absolute regular file"):
            model30.plan(self.artifacts)

    def test_clock_contract_changes_and_checkpoint_source_mixing_rejected(self):
        for document, key, value in (
            ("control_contract", "policy_dt_s", .02),
            ("export", "source_manifest_sha256", "a" * 64),
            ("train_completion", "checkpoint_sha256", model30.PARENT_SHA256),
            ("train_completion", "updates", 499),
        ):
            docs = self.documents()
            target = docs[document]["timing_contract"] if document == "control_contract" else docs[document]
            target[key] = value
            with self.subTest(document=document, key=key), self.assertRaises(ValueError):
                model30._validate_documents(docs)

    def test_model_evidence_cannot_grant_motor_output(self):
        for document in ("control_contract", "export", "train_completion"):
            docs = self.documents()
            docs[document]["hardware_output_allowed"] = True
            with self.subTest(document=document), self.assertRaisesRegex(ValueError, "grant hardware"):
                model30._validate_documents(docs)

    def test_inline_plan_validation_does_not_read_and_owns_result(self):
        result = model30.plan(self.artifacts)
        with patch.object(Path, "stat", side_effect=AssertionError("No file IO")):
            copied = model30.validate_plan(result)
        copied["artifacts"]["actor"]["path"] = "/different"
        self.assertNotEqual(copied, result)
        for key, value in (("policy_dt_s", .02), ("output_allowed", True),
                           ("model_artifact_grants_output", True), ("training_updates", 499)):
            wrong = copy.deepcopy(result)
            wrong[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "Inline30Hz plan differs"):
                model30.validate_plan(wrong)
        wrong = dict(result, legacy_scalar=True)
        with self.assertRaisesRegex(ValueError, "Unknown inline30Hz"):
            model30.validate_plan(wrong)

    def test_explicit_cpu_execution_required(self):
        with self.assertRaisesRegex(ValueError, "execute_cpu=True"):
            model30.load(self.artifacts)


@unittest.skipIf(torch is None or not BUNDLE.exists(), "CPU Torch/pinned30Hz bundle is unavailable")
class ActualCpuModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.artifacts = model30.bundle_artifacts(BUNDLE)

    def test_real_release_cpu_load_has_full_reset_clock_and_exact_weights(self):
        policy, provenance = model30.load(self.artifacts, execute_cpu=True)
        self.assertEqual(policy.policy_dt_s, 1 / 30)
        self.assertEqual(policy.controller.dt, 1 / 30)
        self.assertFalse(provenance["output_allowed"])
        self.assertFalse(provenance["hardware_opened"])
        self.assertTrue(provenance["cpu_validation"]["all_named_controller_buffers_reset"])
        self.assertTrue(provenance["cpu_validation"]["actor_checkpoint_weights_exact"])
        self.assertEqual(float(policy.controller.elapsed[0]), 0.)
        self.assertEqual(float(policy.controller.phase[0]), 0.)
        self.assertEqual(sum(v.numel() for v in policy.actor.parameters()), 35148)
        with torch.inference_mode():
            args = (torch.zeros(1, 3), torch.tensor([[0., 0., -1.]]), torch.zeros(1, 3),
                    policy.controller.nominal.float().reshape(1, 12).clone(),
                    torch.zeros(1, 12), torch.zeros(1, 12))
            for _ in range(3):
                target = policy(*args)
                self.assertEqual(tuple(target.shape), (1, 12))
            self.assertAlmostEqual(float(policy.controller.elapsed[0]), .1, places=14)
            policy.reset(torch.tensor([0], dtype=torch.long))
            self.assertEqual(float(policy.controller.elapsed[0]), 0.)

    def test_loaded_clock_guard_rejects_50hz_or_modified_filter(self):
        for clock, decay in ((.02, math_decay := 0.8751733190429475),
                             (1 / 30, math_decay)):
            with self.subTest(clock=clock):
                bad = SimpleNamespace(policy_dt_s=clock,
                    controller=SimpleNamespace(dt=clock, num_envs=1, observation_dim=74,
                        action_dim=12, filter_decay=decay))
                with self.assertRaisesRegex(ValueError, "clock differs|constant differs"):
                    model30._validate_clock(bad)

    def test_changed_loaded_actor_cannot_pass_checkpoint_comparison(self):
        artifacts = model30.bundle_artifacts(BUNDLE)
        policy = torch.jit.load(artifacts["controller"]["path"], map_location="cpu").eval()
        actor = torch.jit.load(artifacts["actor"]["path"], map_location="cpu").eval()
        checkpoint = torch.load(artifacts["checkpoint"]["path"], map_location="cpu", weights_only=True)
        with torch.inference_mode():
            next(policy.actor.parameters()).add_(.001)
            with self.assertRaisesRegex(ValueError, "weights differ"):
                model30._validate_cpu(policy, actor, checkpoint, torch)


if __name__ == "__main__":
    unittest.main()
