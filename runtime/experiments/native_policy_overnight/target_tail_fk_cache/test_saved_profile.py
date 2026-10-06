"""Three-way PLAN, isolated kernel parity, balanced order and fail-closed pins."""
import ast
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from . import generator
from . import saved_profile as tool
from ..model_call_fastpath import test_saved_input_profile as fixtures


class FKPlanTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.SavedInputTests(); self.fixture.setUp()

    def tearDown(self):self.fixture.tearDown()

    def args(self):
        args = self.fixture.args()
        args.target_library = args.target_library_sha256 = args.fk_library = args.fk_library_sha256 = None
        args.replay_helper_sha256 = hashlib.sha256(Path(tool.replay.__file__).read_bytes()).hexdigest()
        files = {"runtime/experiments/native_policy_overnight/" + name:
                 hashlib.sha256((tool.replay.HERE / name).read_bytes()).hexdigest()
                 for name in (*tool.replay._CANDIDATE_SOURCES,
                    *tool.first._SOURCES,
                    *("target_tail_fk_cache/" + name for name in tool._SOURCES))}
        args.candidate_source_manifest, args.candidate_source_manifest_sha256 = self.fixture.save(
            "kit.json", {"schema": "private-overnight-kit-v1", "files": files})
        return args

    def test_plan_no_torch_build_or_model_and_originals_unchanged(self):
        args = self.args(); originals = {path: path.read_bytes() for path in self.fixture.root.iterdir()}
        with mock.patch.dict("sys.modules", {"torch": None}), mock.patch.object(
                tool, "fk_candidate", side_effect=AssertionError("load forbidden")):
            result = tool.run(args)
        self.assertEqual(result["status"], "FILE_ONLY_PLAN")
        self.assertEqual(result["cycles"], 501)
        self.assertFalse(result["actual_controller_qualification"])
        self.assertEqual(len(result["candidate_source_sha256"]), 24)
        for path, raw in originals.items():self.assertEqual(path.read_bytes(), raw)

    def test_fresh_import_has_no_torch_import(self):
        code = """import builtins
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if name=='torch' or name.startswith('torch.'): raise AssertionError('no Torch in PLAN')
    return original(name,*args,**kwargs)
builtins.__import__=guarded
from native_policy_overnight.target_tail_fk_cache import saved_profile
"""
        result = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_cpp_delta_restores_frozen_first_candidate_bytes(self):
        proof = tool.cpp_delta()
        self.assertTrue(proof["inverse_bytes_exact"])
        self.assertEqual(proof["repeated_trig_calls_removed_per_forward"], 6)
        self.assertEqual(proof["repeated_adds_removed_per_forward"], 3)
        self.assertFalse(proof["performance_gain_measured"])
        self.assertEqual(proof["first_tail_source_sha256"],
                         "2629bb99b6db4789c8a28101d31f65bab894337242060c71b5b8a9c229702b40")

    def test_both_candidate_cpp_pins_required(self):
        for name in ("target_tail_fusion/target.cpp", "target_tail_fk_cache/target.cpp"):
            args = self.args(); manifest = json.loads(Path(args.candidate_source_manifest).read_bytes())
            del manifest["files"]["runtime/experiments/native_policy_overnight/" + name]
            args.candidate_source_manifest, args.candidate_source_manifest_sha256 = self.fixture.save("kit.json", manifest)
            with self.assertRaises(ValueError):tool.run(args)
            self.assertFalse(Path(args.output).exists())

    def test_helper_pin_is_mandatory(self):
        args = self.args(); args.replay_helper_sha256 = "0" * 64
        with self.assertRaises(ValueError):tool.run(args)

    def test_both_library_argument_pairs_fail_before_load(self):
        for prefix in ("target", "fk"):
            for path, pin in ((str(self.fixture.root / "absent.so"), None),
                              (None, "a" * 64), (str(self.fixture.root / "absent.so"), "a" * 64)):
                args = self.args(); setattr(args, prefix + "_library", path); setattr(args, prefix + "_library_sha256", pin)
                with self.assertRaises((ValueError, FileNotFoundError)):tool.run(args)

    def test_foreign_full_fusion_args_rejected(self):
        args = self.args(); args.compare_full_fusion = True
        with self.assertRaisesRegex(ValueError, "Only scalar"):tool.run(args)
        args = self.args(); args.observation_library_sha256 = "a" * 64
        with self.assertRaisesRegex(ValueError, "Unselected"):tool.run(args)

    def test_raw_audit_requires_object_and_both_original_hashes(self):
        for value in ([], None, {"status": "PASS_RAW_EVIDENCE", "cycles_audited": 501,
                                "input_file_sha256": {"only_report": "a" * 64}}):
            args = self.args(); args.raw_audit, args.raw_audit_sha256 = self.fixture.save("audit.json", value)
            with self.assertRaisesRegex(ValueError, "audit|originals"):tool.run(args)
            self.assertFalse(Path(args.output).exists())

    def test_originals_reread_before_publication(self):
        args = self.args(); original = tool.replay.load_saved
        def mutate(*values):
            result = original(*values); Path(args.records).write_text("[]"); return result
        with mock.patch.object(tool.replay, "load_saved", mutate), self.assertRaises(ValueError):tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_no_existing_or_symlink_output(self):
        args = self.args(); Path(args.output).write_text("preserved")
        with self.assertRaises(ValueError):tool.run(args)
        self.assertEqual(Path(args.output).read_text(), "preserved")
        Path(args.output).unlink(); Path(args.output).symlink_to(self.fixture.root / "absent")
        with self.assertRaises(ValueError):tool.run(args)

    def test_execute_requires_manifest_before_torch(self):
        args = self.args(); args.execute_file_only = True
        with mock.patch.dict("sys.modules", {"torch": None}), self.assertRaisesRegex(ValueError, "explicit scalar_manifest"):
            tool.run(args)

    def test_three_distinct_instances_required_before_state_access(self):
        one, two = object(), object()
        with self.assertRaisesRegex(ValueError, "Three independent"):
            tool.independent_three((one, one, two))
        with self.assertRaisesRegex(ValueError, "Three independent"):
            tool.independent_three((one, two))

    def test_namespace_only_generator_ast_inverse(self):
        raw = """import torch
class FusedTargetTailSwingCore:
    def step_target(self, raw, command):
        return torch.ops.sd_target_tail_fileonly_r1.target(raw, command)
    def reset(self, ids):
        self.elapsed[ids]=0
"""
        transformed = generator.transform(raw)
        inverse = transformed.decode().replace("FusedFKCacheTargetSwingCore", "FusedTargetTailSwingCore").replace(
            "sd_target_tail_fk_cache_fileonly_r1", "sd_target_tail_fileonly_r1")
        self.assertEqual(ast.dump(ast.parse(raw)), ast.dump(ast.parse(inverse)))
        for invalid in (raw.replace("FusedTargetTailSwingCore", "Other"),
                        raw.replace("sd_target_tail_fileonly_r1", "foreign"),
                        raw.replace("return torch.ops", "x=torch.ops.sd_target_tail_fileonly_r1\n        return torch.ops")):
            with self.assertRaises(ValueError):generator.transform(invalid)

    def test_generator_wrong_pin_and_destination_refuse(self):
        seed = self.fixture.root / "seed.py"; seed.write_text("class FusedTargetTailSwingCore: pass\n")
        out = self.fixture.root / "out.py"
        with self.assertRaises(ValueError):generator.generate_core(seed, out, "0" * 64)
        self.assertFalse(out.exists())
        out.write_text("preserved")
        with self.assertRaises(ValueError):generator.generate_core(seed, out, hashlib.sha256(seed.read_bytes()).hexdigest())
        self.assertEqual(out.read_text(), "preserved")

    def test_six_permutations_and_rotation_balance_preserve_order(self):
        import torch
        class Policy:
            def reset(self, ids):pass
            def __call__(self, *inputs):return inputs[0]
        models = tuple(Policy() for _ in range(3)); value = torch.zeros((1, 3)); frames = [((value,), (value,))] * 501
        with mock.patch.object(tool, "independent_three"), mock.patch.object(tool.replay, "_state_bits"), mock.patch.object(
                tool.replay, "_outputs", lambda model, result:(result,)):
            proof = tool.balanced_three_timing(torch, models, frames)
        self.assertEqual([row["leading_order"] for row in proof["blocks"]], list(map(list, itertools.permutations(range(3)))))
        for row in proof["blocks"]:
            for value in row["position_counts"].values():self.assertEqual(value, [167, 167, 167])
            self.assertEqual(len(row["raw_call_orders"]), 501)
        for row in proof["aggregate"].values():self.assertEqual(row["wall"]["count"], 3006)
        self.assertFalse(proof["whole_cycle_performance_measured"])


class CppKernelParityTests(unittest.TestCase):
    def test_actual_cpp_three_way_full_state_rejection_boundaries(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root).resolve() / "new"
            result = subprocess.run([sys.executable, "-B", "-m", "native_policy_overnight.target_tail_fk_cache.local_validate",
                "--output", str(output), "--execute-file-only"], capture_output=True, text=True, timeout=180)
            self.assertEqual(result.returncode, 0, result.stderr[-6500:])
            report = json.loads((output / "local-validation.json").read_bytes())
            self.assertEqual(report["status"], "PASS_LOCAL_CPP_THREE_WAY_PARITY")
            self.assertEqual(report["three_way_saved_fixture"]["saved_recurrent_calls"], 501)
            self.assertEqual(report["three_way_saved_fixture"]["rejection_count"], 26)
            self.assertEqual(report["named_state_count"], 30)
            self.assertEqual(report["direct_boundary_cases"], 44)
            for value in report["synthetic_validation"].values():self.assertEqual(value["frames"], 240)
            self.assertFalse(report["original_target_saved_output_validation"])
            self.assertFalse(report["real_model_or_target_loaded"])
            self.assertFalse(report["latency_improvement_proven"])


if __name__ == "__main__":unittest.main()
