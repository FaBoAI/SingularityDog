"""File/CPU fixtures only; no compiled native library or real model artifacts."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

import torch
from torch import Tensor

from ..lean_swing_deployment import DeployableSwingPolicy
from . import saved_actor_profile as tool
from . import test_saved_input_profile as fixtures


class ToyController(torch.nn.Module):
    """Fixed-shape fake controller for scripted module-copy parity."""
    def __init__(self):
        super().__init__()
        self.register_buffer("clock", torch.zeros(1, dtype=torch.float64))
        self.register_buffer("nominal", torch.zeros(12, dtype=torch.float64))

    @torch.jit.export
    def reset(self, ids: Tensor):
        self.clock[ids] = 0.

    @torch.jit.export
    def observation(self, gyro: Tensor, gravity: Tensor, command: Tensor,
                    q: Tensor, dq: Tensor, h: Tensor) -> Tensor:
        if (gyro.size() != [1, 3] or gravity.size() != [1, 3] or command.size() != [1, 3]
                or q.size() != [1, 12] or dq.size() != [1, 12] or h.size() != [1, 12]):
            raise ValueError("input shape")
        if (not bool(torch.isfinite(gyro).all()) or not bool(torch.isfinite(gravity).all())
                or not bool(torch.isfinite(command).all()) or not bool(torch.isfinite(q).all())
                or not bool(torch.isfinite(dq).all()) or not bool(torch.isfinite(h).all())):
            raise ValueError("nonfinite")
        if (not bool(torch.isclose(torch.linalg.vector_norm(gravity), torch.tensor(1.)))
                or bool(gravity[0, 2] == 0.) or bool((h < 0.).any()) or bool((h > 1.).any())
                or bool(command[0, 0] > .46) or bool(abs(command[0, 2]) > .25)
                or bool((abs(command[0, :2]) > 0.).all())
                or bool(command[0, 0] != 0. and command[0, 2] != 0.)):
            raise ValueError("domain")
        return torch.cat((gyro, gravity, command, q, dq, h, torch.zeros((1, 29))), 1)

    @torch.jit.export
    def step_target(self, raw: Tensor, command: Tensor) -> Tensor:
        self.clock.add_(1.)
        return torch.ops.sd_step_fileonly_r1.step(raw)

    def forward(self, raw: Tensor, command: Tensor) -> Tensor:
        return self.step_target(raw, command)


class CachedControllerFixture(torch.nn.Module):
    """Real scripted view aliases, independent of native arithmetic operators."""
    def __init__(self):
        super().__init__()
        self.num_envs = 1
        for name, shape in (("signs", (12,)), ("origins", (12,)),
                            ("anchor", (4, 3)), ("phase", (1,)),
                            ("filters", (1, 3, 3)), ("yaw_filters", (1, 3)),
                            ("sensor_up", (1,)), ("safe_lower", (12,)),
                            ("safe_upper", (12,))):
            self.register_buffer(name, torch.zeros(shape, dtype=torch.float64))
        self.signs[0] = -0.
        self._cached_signs_expanded = self.signs.unsqueeze(0).expand(self.num_envs, -1)
        self._cached_signs_row = self.signs.unsqueeze(0)
        self._cached_origins_row = self.origins.unsqueeze(0)
        self._cached_anchor_x = self.anchor[:, 0].unsqueeze(0)
        self._cached_anchor_y = self.anchor[:, 1].unsqueeze(0)
        self._cached_anchor_xy = self.anchor[:, :2].unsqueeze(0)
        self._cached_anchor_z = self.anchor[:, 2].unsqueeze(0)
        self._cached_phase_column = self.phase.unsqueeze(1)
        self._cached_filter_x = self.filters[:, 2, 0].unsqueeze(1)
        self._cached_filter_y = self.filters[:, 2, 1].unsqueeze(1)
        self._cached_filter_active = self.filters[:, 2, 2].unsqueeze(1)
        self._cached_yaw_filter = self.yaw_filters[:, 2].unsqueeze(1)
        self._cached_up_column = self.sensor_up.unsqueeze(1)
        self._cached_filters_flat = self.filters.reshape(self.num_envs, 9)
        self._cached_safe_lower = self.safe_lower.reshape(1, 4, 3)
        self._cached_safe_upper = self.safe_upper.reshape(1, 4, 3)

    def forward(self) -> Tensor:
        self.phase.add_(1.)
        return self._cached_phase_column


class ActorPlanTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.SavedInputTests()
        self.fixture.setUp()

    def tearDown(self):
        self.fixture.tearDown()

    def args(self):
        args = self.fixture.args()
        args.actor_library = args.actor_library_sha256 = None
        args.replay_helper_sha256 = hashlib.sha256(Path(tool.replay.__file__).read_bytes()).hexdigest()
        files = {"runtime/experiments/native_policy_overnight/" + name:
                 hashlib.sha256((tool.replay.HERE / name).read_bytes()).hexdigest()
                 for name in (*tool.replay._CANDIDATE_SOURCES, *tool._ACTOR_SOURCES)}
        path, pin = self.fixture.save("kit.json", {"schema": "private-overnight-kit-v1", "files": files})
        args.candidate_source_manifest, args.candidate_source_manifest_sha256 = path, pin
        return args

    def test_plan_no_torch_loading_or_original_changes(self):
        args = self.args()
        originals = {p: p.read_bytes() for p in self.fixture.root.iterdir()}
        with mock.patch.dict("sys.modules", {"torch": None}), mock.patch.object(
                tool, "actor_candidate", side_effect=AssertionError("must not build/load")):
            result = tool.run(args)
        self.assertEqual(result["status"], "FILE_ONLY_PLAN")
        self.assertFalse(result["actual_controller_qualification"])
        for p, raw in originals.items():
            self.assertEqual(p.read_bytes(), raw)

    def test_fresh_module_import_does_not_import_torch(self):
        code = """import builtins
old = builtins.__import__
def guarded(name, *args, **kwargs):
    if name == 'torch' or name.startswith('torch.'):
        raise AssertionError('Torch import in PLAN')
    return old(name, *args, **kwargs)
builtins.__import__ = guarded
from native_policy_overnight.model_call_fastpath import saved_actor_profile
"""
        run = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True,
                             text=True, timeout=10)
        self.assertEqual(run.returncode, 0, run.stderr)

    def test_wrong_helper_pin_rejected_before_model_load(self):
        args = self.args()
        args.replay_helper_sha256 = "0" * 64
        with self.assertRaises(ValueError):
            tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_actor_source_pin_required(self):
        args = self.args()
        manifest = json.loads(Path(args.candidate_source_manifest).read_bytes())
        del manifest["files"]["runtime/experiments/native_policy_overnight/model_call_fastpath/actor.cpp"]
        args.candidate_source_manifest, args.candidate_source_manifest_sha256 = self.fixture.save("kit.json", manifest)
        with self.assertRaises(ValueError):
            tool.run(args)

    def test_actor_library_pin_pair_and_bytes_required_in_plan(self):
        for path, pin in ((str(self.fixture.root / "absent.so"), None),
                          (None, "a" * 64), (str(self.fixture.root / "absent.so"), "a" * 64)):
            args = self.args()
            args.actor_library, args.actor_library_sha256 = path, pin
            with self.assertRaises((ValueError, FileNotFoundError)):
                tool.run(args)

    def test_full_fusion_cannot_confound_actor_only_comparison(self):
        args = self.args()
        args.compare_full_fusion = True
        with self.assertRaisesRegex(ValueError, "Actor-only"):
            tool.run(args)

    def test_unselected_full_fusion_library_argument_not_silently_ignored(self):
        args = self.args()
        args.observation_library_sha256 = "a" * 64
        with self.assertRaisesRegex(ValueError, "Unselected"):
            tool.run(args)

    def test_execute_missing_scalar_pins_before_load(self):
        args = self.args()
        args.execute_file_only = True
        with mock.patch.dict("sys.modules", {"torch": None}):
            with self.assertRaisesRegex(ValueError, "explicit scalar_manifest"):
                tool.run(args)

    def test_source_or_original_mutation_rejected_before_publish(self):
        args = self.args()
        original = tool.replay.load_saved
        def changing(*values):
            result = original(*values)
            Path(args.records).write_text("[]")
            return result
        with mock.patch.object(tool.replay, "load_saved", changing):
            with self.assertRaises(ValueError):
                tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_all_four_order_blocks_balanced_with_correct_labels(self):
        scalar, candidate = object(), object()
        frames = [None] * 3
        def timing(_torch, models, _frames):
            left = {"raw_wall_ns": [10, 20, 30], "raw_thread_cpu_ns": [5, 10, 15]}
            right = {"raw_wall_ns": [11, 21, 31], "raw_thread_cpu_ns": [6, 11, 16]}
            return {"current_scalar": copy.deepcopy(left), "full_fusion_candidate": copy.deepcopy(right)}
        with mock.patch.object(tool.replay, "paired_timing", side_effect=timing) as called:
            result = tool.paired_blocks(None, scalar, candidate, frames)
        self.assertEqual([row.args[1] for row in called.call_args_list],
                         [(scalar, candidate), (candidate, scalar), (candidate, scalar), (scalar, candidate)])
        self.assertEqual(result["aggregate"]["current_scalar"]["raw_wall_ns"],
                         [10, 20, 30, 11, 21, 31, 11, 21, 31, 10, 20, 30])
        for name in ("scalar_first", "candidate_first"):
            self.assertEqual(sum(b["by_call_order"][name]["count"] for b in result["blocks"]), 6)
        self.assertFalse(result["latency_improvement_proven"])


class ActorCopyTests(unittest.TestCase):
    def test_real_scripted_cached_views_survive_copy_and_reload(self):
        import io
        from ..view_cache.generator import verify_aliases
        original = torch.jit.script(CachedControllerFixture())
        before = {name: value.clone() for name, value in original.named_buffers()}
        cloned = tool.copy_controller(original)
        verify_aliases(original)
        verify_aliases(cloned)
        for name, value in cloned.named_buffers():
            tool.replay._compare_bits(torch, before[name], value, name)
            self.assertNotEqual(value.data_ptr(), getattr(original, name).data_ptr())
        with torch.inference_mode():
            self.assertEqual(cloned().item(), 1.)
            self.assertEqual(original.phase.item(), 0.)
        output = io.BytesIO()
        torch.jit.save(cloned, output)
        restored = torch.jit.load(io.BytesIO(output.getvalue()))
        verify_aliases(restored)
        with torch.inference_mode():
            self.assertEqual(restored().item(), 2.)
            self.assertEqual(cloned.phase.item(), 1.)
        self.assertEqual(original.forward.code, restored.forward.code)

    def test_invalid_original_alias_is_rejected_without_repairing_original(self):
        original = torch.jit.script(CachedControllerFixture())
        original._cached_signs_expanded = original._cached_signs_expanded.clone()
        with self.assertRaisesRegex(ValueError, "lost its buffer alias"):
            tool.copy_controller(original)
        self.assertNotEqual(original.signs.data_ptr(), original._cached_signs_expanded.data_ptr())

    def test_partial_cached_view_table_rejected(self):
        original = ToyController()
        original._cached_signs_row = torch.zeros((1, 12))
        with self.assertRaisesRegex(ValueError, "Incomplete cached"):
            tool.copy_controller(original)

    def test_scripted_copy_preserves_controller_and_exact_actor_state(self):
        # Python CPU registrations stand in for existing C++ ops; no .so loaded.
        step = torch.library.Library("sd_step_fileonly_r1", "DEF")
        try:
            step.define("step(Tensor raw) -> Tensor")
            step.impl("step", lambda raw: raw.clone(), "CPU")
            actor_libraries = []
            def fake_load(_path):
                lib = torch.library.Library("sd_actor_fileonly_r1", "DEF")
                lib.define("forward(Tensor observation, Tensor w0, Tensor b0, Tensor w1, Tensor b1, Tensor w2, Tensor b2, Tensor w3, Tensor b3) -> Tensor")
                def forward(obs, w0, b0, w1, b1, w2, b2, w3, b3):
                    value = torch.nn.functional.elu(torch.nn.functional.linear(obs, w0, b0))
                    value = torch.nn.functional.elu(torch.nn.functional.linear(value, w1, b1))
                    value = torch.nn.functional.elu(torch.nn.functional.linear(value, w2, b2))
                    return torch.nn.functional.linear(value, w3, b3)
                lib.impl("forward", forward, "CPU")
                actor_libraries.append(lib)
            try:
                with tempfile.TemporaryDirectory() as temp:
                    directory = Path(temp).resolve()
                    library = directory / "pinned-fixture.so"
                    library.write_bytes(b"file-only-fake-library")
                    args = types.SimpleNamespace(actor_library=str(library),
                        actor_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
                    actor = torch.nn.Sequential(torch.nn.Linear(74, 128), torch.nn.ELU(),
                        torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 64),
                        torch.nn.ELU(), torch.nn.Linear(64, 12))
                    with torch.no_grad():
                        actor[0].weight[0, 0] = -0.
                    baseline = DeployableSwingPolicy(actor)
                    baseline.controller = ToyController()
                    scalar = torch.jit.script(baseline)
                    with mock.patch.object(torch.ops, "load_library", side_effect=fake_load):
                        candidate, proof = tool.actor_candidate(torch, scalar, args, directory / "report.json")
                    self.assertTrue(proof["scalar_controller_code_unchanged"])
                    self.assertIsNot(candidate.controller, scalar.controller)
                    self.assertNotEqual(candidate.controller.clock.data_ptr(), scalar.controller.clock.data_ptr())
                    ids = torch.tensor([0], dtype=torch.long)
                    frames = []
                    with torch.inference_mode():
                        scalar.reset(ids)
                        for index in range(501):
                            inputs = tool.replay.frame(torch, scalar.controller.nominal.float().reshape(1, 12), index)
                            target = scalar(*inputs)
                            saved = tuple(x.clone() for x in tool.replay._outputs(scalar, target))
                            frames.append((inputs, saved))
                    with mock.patch.object(tool.replay, "verify_aliases"):
                        result = tool.replay.compare_saved(torch, (scalar, candidate), frames)
                    self.assertEqual(result["rejection_count"], 26)
                    self.assertEqual(result["saved_recurrent_calls"], 501)
                    synthetic = tool.replay.synthetic_parity(torch, scalar, candidate)
                    self.assertEqual(synthetic["frames"], 240)
                    self.assertEqual(candidate.controller.clock.item(), 115.)
            finally:
                for lib in actor_libraries:
                    lib._destroy()
                for namespace, op in ((torch.ops.sd_actor_fileonly_r1, "forward"),
                                      (torch.ops.sd_step_fileonly_r1, "step")):
                    if op in namespace.__dict__:
                        delattr(namespace, op)
        finally:
            step._destroy()

    def test_already_loaded_actor_rejected_before_artifact_creation(self):
        with tempfile.TemporaryDirectory() as temp, mock.patch.object(
                torch.ops, "sd_actor_fileonly_r1", types.SimpleNamespace(forward=object())):
            with self.assertRaisesRegex(ValueError, "already loaded"):
                tool.actor_candidate(torch, None, None, Path(temp).resolve() / "report.json")
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_changed_actor_layer_settings_rejected(self):
        actor = torch.nn.Sequential(torch.nn.Linear(74, 128), torch.nn.ELU(alpha=2.),
            torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 64),
            torch.nn.ELU(), torch.nn.Linear(64, 12))
        scalar = torch.jit.script(DeployableSwingPolicy(actor).actor)
        with self.assertRaisesRegex(ValueError, "ELU settings"):
            tool.eager_actor(torch, types.SimpleNamespace(actor=scalar))

    def test_different_model_objects_cannot_share_state_storage(self):
        one, two = torch.nn.Module(), torch.nn.Module()
        shared = torch.zeros(1)
        one.register_buffer("clock", shared)
        two.register_buffer("clock", shared)
        with self.assertRaisesRegex(ValueError, "shares scalar state"):
            tool.disjoint_state(one, two)


if __name__ == "__main__":
    unittest.main()
