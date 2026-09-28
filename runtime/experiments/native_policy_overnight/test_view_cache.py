"""Cached-view ownership/reset tests and optional pinned native reload parity."""
import copy
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from native_policy_overnight import contracts as c
from native_policy_overnight.lean_swing_core import SwingCore
from native_policy_overnight.verification import compare_state, compare_tensor, frame, rejections, reject_reason
from native_policy_overnight.view_cache import generate_core, verify_aliases
from native_policy_overnight.view_cache import generator


class ViewCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        cls.torch = torch
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name) / "cached_core.py"
        cls.core_class, cls.proof = generate_core(cls.path)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def pair(self, environments=1):
        return (SwingCore(environments, "cpu", **c.OPTIONS),
                self.core_class(environments, "cpu", **c.OPTIONS))

    def test_source_proof_keeps_seed_pins_and_all_original_ast(self):
        self.assertEqual(self.proof["seed_core_sha256"], c.REUSED_PINS["lean_swing_core.py"])
        self.assertEqual(self.proof["generated_core_sha256"], c.sha(self.path.read_bytes()))
        self.assertTrue(self.proof["original_ast_exact_after_inverse_view_substitution"])
        self.assertEqual(len(self.proof["declared_view_replacements"]), 16)
        self.assertTrue(all(self.proof["declared_view_replacements"].values()))
        self.assertFalse(self.proof["output_allowed"])
        for name, digest in c.REUSED_PINS.items():
            c.pinned(c.HERE / name, digest)
        # The opt-in subdirectory does not invalidate any existing manifest.
        self.assertNotIn("generator.py", c.source_hashes())

    def test_new_source_only_and_drift_rejected_before_creation(self):
        with self.assertRaisesRegex(ValueError, "must be new"):
            generate_core(self.path)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "lean_swing_core.py").write_text("unreviewed controller")
            output = root / "candidate.py"
            with mock.patch.object(generator, "HERE", root):
                with self.assertRaisesRegex(ValueError, "SHA mismatch"):
                    generate_core(output)
            self.assertFalse(output.exists())

    def test_named_state_unchanged_and_all_views_alias_owners(self):
        for environments in (1, 2):
            original, candidate = self.pair(environments)
            with self.torch.inference_mode():
                compare_state(self.torch, original, candidate, {}, exact=True)
                self.assertTrue(verify_aliases(candidate))
                for name, _, _, owner in generator._VIEWS:
                    self.assertEqual(getattr(candidate, name).untyped_storage().data_ptr(),
                                     getattr(candidate, owner).untyped_storage().data_ptr())

    def test_mutation_and_reset_are_visible_without_rebuilding_views(self):
        _, candidate = self.pair()
        views = {name: getattr(candidate, name) for name, _, _, _ in generator._VIEWS}
        with self.torch.inference_mode():
            candidate.phase.fill_(.3)
            candidate.filters.fill_(.4)
            candidate.yaw_filters.fill_(.5)
            candidate.sensor_up.fill_(.6)
            self.assertEqual(candidate._cached_phase_column.item(), .3)
            self.assertTrue(self.torch.equal(candidate._cached_filter_x, candidate.filters[:, 2, 0].unsqueeze(1)))
            self.assertTrue(self.torch.equal(candidate._cached_yaw_filter, candidate.yaw_filters[:, 2].unsqueeze(1)))
            candidate.reset(self.torch.tensor([0], dtype=self.torch.long))
            self.assertTrue(verify_aliases(candidate))
            self.assertEqual(candidate._cached_phase_column.item(), 0.)
            self.assertTrue(self.torch.equal(candidate._cached_up_column,
                                            self.torch.tensor([[[0., 0., 1.]]], dtype=self.torch.float64)))
            self.assertTrue(all(getattr(candidate, name) is value for name, value in views.items()))

    def test_rebound_or_copied_buffer_views_are_rejected(self):
        _, candidate = self.pair()
        candidate._cached_phase_column = candidate._cached_phase_column.clone()
        with self.assertRaisesRegex(ValueError, "lost its buffer alias"):
            verify_aliases(candidate)
        _, candidate = self.pair()
        candidate.phase = candidate.phase.clone()
        with self.assertRaisesRegex(ValueError, "lost its buffer alias"):
            verify_aliases(candidate)

    def test_same_storage_wrong_offset_or_lazy_sign_is_rejected(self):
        _, candidate = self.pair()
        candidate._cached_anchor_x = candidate.anchor[:, 1].unsqueeze(0)
        with self.assertRaisesRegex(ValueError, "metadata differs"):
            verify_aliases(candidate)
        _, candidate = self.pair()
        candidate._cached_phase_column = candidate._cached_phase_column._neg_view()
        with self.assertRaisesRegex(ValueError, "metadata differs"):
            verify_aliases(candidate)

    def test_240_observation_clock_filter_and_geometry_steps_exact(self):
        original, candidate = self.pair()
        torch = self.torch
        maxima = {}
        with torch.inference_mode():
            nominal = original.nominal.float().reshape(1, 12)
            for index in range(240):
                if index in (0, 125):
                    for core in (original, candidate):
                        core.reset(torch.tensor([0], dtype=torch.long))
                inputs = frame(torch, nominal, index)
                before = [value.clone() for value in inputs]
                observed = [core.observation(*inputs) for core in (original, candidate)]
                compare_tensor(torch, observed[0], observed[1], "observation", maxima, exact=True)
                raw = 1.5 * torch.sin(torch.arange(12).reshape(1, 12) * .2 + index * .03)
                results = [core._step_inputs(raw, inputs[2]) for core in (original, candidate)]
                for key, value in results[0].items():
                    compare_tensor(torch, value, results[1][key], key, maxima, exact=True)
                feet = [core.fk(inputs[3]) for core in (original, candidate)]
                compare_tensor(torch, feet[0], feet[1], "fk", maxima, exact=True)
                joints = [core.ik(value) for core, value in zip((original, candidate), feet)]
                compare_tensor(torch, joints[0], joints[1], "ik", maxima, exact=True)
                compare_state(torch, original, candidate, maxima, exact=True)
                self.assertTrue(verify_aliases(candidate))
                self.assertTrue(all(torch.equal(a, b) for a, b in zip(before, inputs)))
        self.assertTrue(all(value == 0. for value in maxima.values()))


_FIXTURE = {key: os.environ.get("SD_VIEW_CACHE_" + key) for key in
            ("MANIFEST", "MANIFEST_SHA256", "BUNDLE", "RECORDS", "RECORDS_SHA256")}


@unittest.skipUnless(all(_FIXTURE.values()), "Explicit pinned CPU native artifact/records required")
class NativeViewCacheParityTests(unittest.TestCase):
    def test_script_reload_saved_inputs_and_all_rejections_exact(self):
        import torch
        from native_policy_overnight import load_verified
        from native_policy_overnight.lean_swing_deployment import DeployableSwingPolicy

        baseline, _ = load_verified(_FIXTURE["MANIFEST"],
            expected_manifest_sha256=_FIXTURE["MANIFEST_SHA256"], bundle=_FIXTURE["BUNDLE"])
        reference, _ = c.reference_policy(_FIXTURE["BUNDLE"])
        with tempfile.TemporaryDirectory() as directory:
            core, _ = generate_core(Path(directory) / "cached_core.py")
            policy = DeployableSwingPolicy(copy.deepcopy(reference.actor), **c.OPTIONS).eval()
            policy.controller = core(1, "cpu", **c.OPTIONS)
            candidate = torch.jit.script(policy)
            buffer = io.BytesIO()
            torch.jit.save(candidate, buffer)
            reloaded = torch.jit.load(io.BytesIO(buffer.getvalue()), map_location="cpu").eval()
            models = (baseline, candidate, reloaded)
            records = c.strict_json(c.pinned(_FIXTURE["RECORDS"], _FIXTURE["RECORDS_SHA256"]))
            self.assertEqual(len(records), 20)
            maxima = {}
            with torch.inference_mode():
                self.assertTrue(verify_aliases(candidate.controller))
                self.assertTrue(verify_aliases(reloaded.controller))
                ids = torch.tensor([0], dtype=torch.long)
                nominal = baseline.controller.nominal.float().reshape(1, 12)
                synthetic = [frame(torch, nominal, index) for index in range(240)]

                def compare_step(inputs):
                    before = [value.clone() for value in inputs]
                    outputs = [model(*inputs) for model in models]
                    for index in (1, 2):
                        compare_tensor(torch, outputs[0], outputs[index], "target", maxima, exact=True)
                        compare_state(torch, models[0], models[index], maxima, exact=True)
                    self.assertTrue(all(torch.equal(a, b) for a, b in zip(before, inputs)))

                for index, inputs in enumerate(synthetic):
                    if index in (0, 125):
                        for model in models:
                            model.reset(ids)
                    compare_step(inputs)
                for repeat in range(6):
                    for model in models:
                        model.reset(ids)
                    for index, row in enumerate(records):
                        self.assertEqual(row["cycle"], index + 1)
                        inputs = tuple(torch.tensor([row["observed"]["inputs"][key]], dtype=torch.float32)
                                       for key in c.INPUT_KEYS)
                        compare_step(inputs)
                for name, inputs in rejections(torch, synthetic[0]).items():
                    reasons = []
                    for model in models:
                        model.reset(ids)
                        try:
                            model(*inputs)
                        except (ValueError, RuntimeError, torch.jit.Error) as error:
                            reasons.append(reject_reason(error))
                        else:
                            self.fail("Invalid input accepted: " + name)
                    self.assertEqual(len(set(reasons)), 1, name)
                    compare_state(torch, baseline, candidate, maxima, exact=True)
                    compare_state(torch, baseline, reloaded, maxima, exact=True)
                self.assertTrue(verify_aliases(candidate.controller))
                self.assertTrue(verify_aliases(reloaded.controller))
            self.assertTrue(all(value == 0. for value in maxima.values()))


if __name__ == "__main__":
    unittest.main()
