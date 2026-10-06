"""PLAN/failure boundaries plus actual isolated C++ CPU kernel parity."""
import ast
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

from . import generator
from . import saved_profile as tool
from ..model_call_fastpath import test_saved_input_profile as fixtures


class TargetPlanTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.SavedInputTests(); self.fixture.setUp()

    def tearDown(self):
        self.fixture.tearDown()

    def args(self):
        args = self.fixture.args()
        args.target_library = args.target_library_sha256 = None
        args.replay_helper_sha256 = hashlib.sha256(Path(tool.replay.__file__).read_bytes()).hexdigest()
        files = {"runtime/experiments/native_policy_overnight/" + name:
                 hashlib.sha256((tool.replay.HERE / name).read_bytes()).hexdigest()
                 for name in (*tool.replay._CANDIDATE_SOURCES, *tool._SOURCES)}
        path, pin = self.fixture.save("kit.json", {"schema": "private-overnight-kit-v1", "files": files})
        args.candidate_source_manifest, args.candidate_source_manifest_sha256 = path, pin
        return args

    def test_plan_does_not_load_torch_model_or_cpp_and_preserves_originals(self):
        args = self.args()
        originals = {path: path.read_bytes() for path in self.fixture.root.iterdir()}
        with mock.patch.dict("sys.modules", {"torch": None}), mock.patch.object(
                tool, "target_candidate", side_effect=AssertionError("must not load")):
            result = tool.run(args)
        self.assertEqual(result["status"], "FILE_ONLY_PLAN")
        self.assertFalse(result["actual_controller_qualification"])
        self.assertEqual(result["cycles"], 501)
        for path, raw in originals.items():
            self.assertEqual(path.read_bytes(), raw)

    def test_fresh_import_does_not_import_torch(self):
        code = """import builtins
old = builtins.__import__
def guarded(name, *args, **kwargs):
    if name == 'torch' or name.startswith('torch.'):
        raise AssertionError('Torch import in PLAN')
    return old(name, *args, **kwargs)
builtins.__import__ = guarded
from native_policy_overnight.target_tail_fusion import saved_profile
"""
        run = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True,
                             text=True, timeout=10)
        self.assertEqual(run.returncode, 0, run.stderr)

    def test_wrong_helper_pin_rejected_before_model_load(self):
        args = self.args(); args.replay_helper_sha256 = "0" * 64
        with self.assertRaises(ValueError):
            tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_cpp_source_pin_is_required(self):
        args = self.args()
        manifest = json.loads(Path(args.candidate_source_manifest).read_bytes())
        del manifest["files"]["runtime/experiments/native_policy_overnight/target_tail_fusion/target.cpp"]
        args.candidate_source_manifest, args.candidate_source_manifest_sha256 = self.fixture.save("kit.json", manifest)
        with self.assertRaises(ValueError):
            tool.run(args)

    def test_target_library_path_and_pin_must_be_paired_and_readable(self):
        for path, pin in ((str(self.fixture.root / "absent.so"), None),
                          (None, "a" * 64), (str(self.fixture.root / "absent.so"), "a" * 64)):
            args = self.args(); args.target_library, args.target_library_sha256 = path, pin
            with self.assertRaises((ValueError, FileNotFoundError)):
                tool.run(args)

    def test_no_full_fusion_confounding_selection(self):
        args = self.args(); args.compare_full_fusion = True
        with self.assertRaisesRegex(ValueError, "Target-tail-only"):
            tool.run(args)
        args = self.args(); args.observation_library_sha256 = "a" * 64
        with self.assertRaisesRegex(ValueError, "Unselected"):
            tool.run(args)

    def test_execution_requires_scalar_pins_before_import(self):
        args = self.args(); args.execute_file_only = True
        with mock.patch.dict("sys.modules", {"torch": None}), self.assertRaisesRegex(
                ValueError, "explicit scalar_manifest"):
            tool.run(args)

    def test_unregistered_scalar_options_rejected_before_build(self):
        import torch
        args = self.args()
        scalar = types.SimpleNamespace(controller=types.SimpleNamespace(num_envs=2))
        with self.assertRaisesRegex(ValueError, "option differs: num_envs"):
            tool.target_candidate(torch, scalar, args, Path(args.output))
        self.assertFalse(Path(args.output).with_name(Path(args.output).stem + "-target-artifacts").exists())

    def test_raw_audit_must_bind_both_originals(self):
        args = self.args()
        audit = {"status": "PASS_RAW_EVIDENCE", "cycles_audited": 501,
                 "input_file_sha256": {"report": args.report_sha256}}
        args.raw_audit, args.raw_audit_sha256 = self.fixture.save("audit.json", audit)
        with self.assertRaisesRegex(ValueError, "both originals"):
            tool.run(args)

    def test_original_changes_before_publish_reject(self):
        args = self.args(); original = tool.replay.load_saved
        def mutate(*values):
            result = original(*values); Path(args.records).write_text("[]"); return result
        with mock.patch.object(tool.replay, "load_saved", mutate), self.assertRaises(ValueError):
            tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_fresh_output_cannot_overwrite_or_follow_symlink(self):
        args = self.args(); Path(args.output).write_text("preserved")
        with self.assertRaises(ValueError):
            tool.run(args)
        self.assertEqual(Path(args.output).read_text(), "preserved")
        args = self.args(); Path(args.output).unlink()
        Path(args.output).symlink_to(self.fixture.root / "absent")
        with self.assertRaises(ValueError):
            tool.run(args)

    def test_four_balanced_blocks_and_raw_differences(self):
        scalar, candidate = object(), object(); frames = [None] * 3
        def timing(_torch, models, _frames):
            return {"current_scalar": {"raw_wall_ns": [10, 20, 30], "raw_thread_cpu_ns": [5, 10, 15]},
                    "full_fusion_candidate": {"raw_wall_ns": [11, 21, 31], "raw_thread_cpu_ns": [6, 11, 16]}}
        with mock.patch.object(tool.replay, "paired_timing", side_effect=timing) as called:
            result = tool.paired_blocks(None, scalar, candidate, frames)
        self.assertEqual([call.args[1] for call in called.call_args_list],
                         [(scalar, candidate), (candidate, scalar), (candidate, scalar), (scalar, candidate)])
        self.assertEqual(result["source_order"], ["AB", "BA", "BA", "AB"])
        self.assertEqual(result["blocks"][0]["raw_paired_wall_difference_ns"], [-1] * 3)
        self.assertEqual(result["blocks"][1]["raw_paired_wall_difference_ns"], [1] * 3)
        self.assertEqual(len(result["aggregate"]["current_scalar"]["raw_wall_ns"]), 12)
        self.assertFalse(result["latency_improvement_proven"])

    def test_only_declared_step_target_body_and_export_metadata_transform(self):
        source = """import torch
class FusedStepSwingCore:
    def _step_inputs(self, raw, command):
        return {'original': raw}
    def step_target(self, raw, command):
        return self._step_inputs(raw, command)
    def reset(self, ids):
        self.phase[ids] = 0
"""
        transformed = ast.parse(generator.transform(source))
        cls = next(node for node in transformed.body if isinstance(node, ast.ClassDef))
        self.assertEqual(cls.name, "FusedTargetTailSwingCore")
        methods = {node.name: node for node in cls.body if isinstance(node, ast.FunctionDef)}
        original = ast.parse(source).body[1]
        old = {node.name: node for node in original.body if isinstance(node, ast.FunctionDef)}
        self.assertEqual(ast.dump(methods["reset"]), ast.dump(old["reset"]))
        self.assertEqual(ast.dump(ast.Module(body=methods["_step_inputs"].body, type_ignores=[])),
                         ast.dump(ast.Module(body=old["_step_inputs"].body, type_ignores=[])))
        self.assertEqual(ast.unparse(methods["_step_inputs"].decorator_list[0]), "torch.jit.export")
        with self.assertRaises(ValueError):
            generator.transform(source.replace("FusedStepSwingCore", "DifferentCore"))

    def test_generator_wrong_pin_and_existing_destination_are_fail_closed(self):
        seed = self.fixture.root / "seed.py"; seed.write_text("class FusedStepSwingCore: pass\n")
        out = self.fixture.root / "new.py"
        with self.assertRaises(ValueError):
            generator.generate_core(seed, out, "0" * 64)
        self.assertFalse(out.exists())
        out.write_text("preserved")
        with self.assertRaises(ValueError):
            generator.generate_core(seed, out, hashlib.sha256(seed.read_bytes()).hexdigest())
        self.assertEqual(out.read_text(), "preserved")


class CppKernelParityTests(unittest.TestCase):
    def test_actual_cpp_cpu_kernels_full_state_rejection_and_boundaries(self):
        # Fresh process isolates custom-op namespaces from synthetic test modules.
        with tempfile.TemporaryDirectory() as root:
            destination = Path(root).resolve() / "new"
            process = subprocess.run([sys.executable, "-B", "-m",
                "native_policy_overnight.target_tail_fusion.local_validate",
                "--output", str(destination), "--execute-file-only"],
                capture_output=True, text=True, timeout=120)
            self.assertEqual(process.returncode, 0, process.stderr[-6000:])
            report = json.loads((destination / "local-validation.json").read_bytes())
            self.assertEqual(report["status"], "PASS_LOCAL_CPP_PARITY")
            self.assertEqual(report["synthetic_501"]["saved_recurrent_calls"], 501)
            self.assertEqual(report["synthetic_501"]["rejection_count"], 26)
            self.assertEqual(report["synthetic_240"]["frames"], 240)
            self.assertEqual(report["named_state_count"], 30)
            self.assertEqual(report["direct_boundary_cases"], 44)
            self.assertFalse(report["real_model_or_target_loaded"])
            self.assertFalse(report["target_saved_output_parity_claimed"])
            self.assertFalse(report["approved_for_runtime"])


if __name__ == "__main__":
    unittest.main()
