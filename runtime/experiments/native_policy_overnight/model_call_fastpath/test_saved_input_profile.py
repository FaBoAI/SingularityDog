"""File/CPU tests; no target libraries, model artifacts or transport imports."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest import mock

from . import saved_input_profile as tool


def fixture():
    source = {"schema": "native-step-scalar-file-only-loader-v1",
              **dict.fromkeys(tool._FALSE_FLAGS, False),
              "manifest_sha256": "a" * 64, "model_sha256": "b" * 64,
              "library_sha256": "c" * 64,
              "baseline_provenance": {"manifest_sha256": "d" * 64}}
    report = {"status": "COMPLETE_DIAGNOSTIC", "cycles_completed": 501,
              "errors": [], "scalar_step_model_source": source,
              "motor_enable_sent": False, "learned_targets_sent": False,
              "approved_for_runtime": False, "full_controller_50Hz_verified": False}
    inputs = {key: [0.] * width for key, width in zip(tool.INPUT_KEYS, tool._WIDTHS)}
    inputs["gravity_body_unit"] = [0., 0., -1.]
    records = [{"cycle": index + 1, "observed": {
        "status": "TICK_OBSERVED_NO_OUTPUT", "output_allowed": False,
        "tick_index": index, "inputs": copy.deepcopy(inputs),
        **{key: [0.] * width for key, width in tool._SAVED_OUTPUTS}}}
        for index in range(501)]
    return report, records


class SavedInputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.report, self.records = fixture()

    def tearDown(self):
        self.temp.cleanup()

    def save(self, name, value, *, raw=None):
        path = self.root / name
        data = raw if raw is not None else json.dumps(value, allow_nan=False).encode()
        path.write_bytes(data)
        return path, hashlib.sha256(data).hexdigest()

    def load(self):
        one = self.save("report.json", self.report)
        two = self.save("records.json", self.records)
        return tool.load_saved(*one, *two)

    def args(self):
        r, rs = self.save("report.json", self.report)
        p, ps = self.save("records.json", self.records)
        a, az = self.save("audit.json", {"status": "PASS_RAW_EVIDENCE",
            "cycles_audited": 501, "input_file_sha256": {str(r): rs, str(p): ps}})
        return tool.parser().parse_args(["--report", str(r), "--report-sha256", rs,
            "--records", str(p), "--records-sha256", ps, "--raw-audit", str(a),
            "--raw-audit-sha256", az, "--output", str(self.root / "new.json")])

    def test_original_vectors_and_negative_zero_kept(self):
        self.records[0]["observed"]["inputs"]["gyro_body_rad_s"][0] = -0.
        _, values = self.load()
        self.assertEqual(len(values), 501)
        self.assertEqual(tool.struct.pack("=f", values[0][0][0][0]), b"\0\0\0\x80")

    def test_plan_does_not_load_model_or_torch_or_write_originals(self):
        args = self.args()
        old = {p: p.read_bytes() for p in self.root.iterdir()}
        with mock.patch.dict("sys.modules", {"torch": None}), mock.patch(
                "native_policy_overnight.model_call_fastpath.scalar_loader.load_file_only_verified",
                side_effect=AssertionError("must not load")):
            result = tool.run(args)
        self.assertEqual(result["status"], "FILE_ONLY_PLAN")
        self.assertFalse(result["output_allowed"])
        for path, raw in old.items():
            self.assertEqual(path.read_bytes(), raw)

    def test_fresh_import_cannot_import_torch(self):
        code = """import builtins
old = builtins.__import__
def guarded(name, *args, **kwargs):
    if name == 'torch' or name.startswith('torch.'):
        raise AssertionError('Torch import in PLAN module')
    return old(name, *args, **kwargs)
builtins.__import__ = guarded
from native_policy_overnight.model_call_fastpath import saved_input_profile
"""
        result = subprocess.run([sys.executable, "-B", "-c", code],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_count_order_and_boolean_cycle_rejected(self):
        for change in (lambda: self.records.pop(),
                       lambda: self.records[0].update(cycle=True),
                       lambda: self.records.reverse()):
            with self.subTest(change=change):
                self.report, self.records = fixture()
                change()
                with self.assertRaises(ValueError):
                    self.load()

    def test_report_incomplete_errors_and_boolean_count_rejected(self):
        for field, value in (("status", "ABORTED"), ("errors", ["partial"]),
                             ("cycles_completed", True), ("motor_enable_sent", True)):
            with self.subTest(field=field):
                self.report, self.records = fixture()
                self.report[field] = value
                with self.assertRaises(ValueError):
                    self.load()

    def test_bad_input_shape_type_extra_or_overflow_rejected(self):
        for value in ([0., 0.], [True, 0., 0.], [1e100, 0., 0.]):
            with self.subTest(value=value):
                self.report, self.records = fixture()
                self.records[0]["observed"]["inputs"]["command"] = value
                with self.assertRaises(ValueError):
                    self.load()
        self.report, self.records = fixture()
        self.records[0]["observed"]["inputs"]["extra"] = [0.]
        with self.assertRaises(ValueError):
            self.load()

    def test_nonfinite_and_duplicate_json_rejected(self):
        for raw in (b'{"x":NaN}', b'{"x":1,"x":2}'):
            with self.subTest(raw=raw):
                p, digest = self.save("bad.json", {}, raw=raw)
                with self.assertRaises(ValueError):
                    tool.strict_json(tool._read(p, digest))

    def test_sha_or_symlink_or_relative_input_rejected(self):
        path, digest = self.save("file.json", {})
        for p, pin in ((path, "0" * 64), (path, "A" * 64), (Path("relative"), digest)):
            with self.assertRaises(ValueError):
                tool._read(p, pin)
        link = self.root / "link.json"
        link.symlink_to(path)
        with self.assertRaises(ValueError):
            tool._read(link, digest)

    def test_sparse_original_over_size_rejected(self):
        path = self.root / "large.json"
        with path.open("wb") as stream:
            stream.truncate(tool._MAX_ORIGINAL_BYTES + 1)
        with self.assertRaises(ValueError):
            tool._read(path, "0" * 64)

    def test_output_overwrite_git_symlink_rejected(self):
        self.save("old.json", {})
        with self.assertRaises(ValueError):
            tool._output_path(self.root / "old.json")
        (self.root / ".git").mkdir()
        with self.assertRaises(ValueError):
            tool._output_path(self.root / "new.json")
        (self.root / ".git").rmdir()
        link = self.root / "link"
        link.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            tool._output_path(link / "new.json")

    def test_raw_audit_must_bind_both_originals(self):
        args = self.args()
        path, digest = self.save("audit.json", {"status": "PASS_RAW_EVIDENCE",
            "cycles_audited": 501, "input_file_sha256": {"wrong": "0" * 64}})
        args.raw_audit, args.raw_audit_sha256 = path, digest
        with self.assertRaisesRegex(ValueError, "bind both"):
            tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_execute_without_artifact_pins_fails_before_loading(self):
        args = self.args()
        args.execute_file_only = True
        with mock.patch.dict("sys.modules", {"torch": None}):
            with self.assertRaisesRegex(ValueError, "explicit scalar_manifest"):
                tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_source_or_original_mutation_before_publish_rejected(self):
        args = self.args()
        original = tool.load_saved
        def changing(*a):
            value = original(*a)
            Path(args.records).write_text("[]")
            return value
        with mock.patch.object(tool, "load_saved", changing):
            with self.assertRaises(ValueError):
                tool.run(args)
        self.assertFalse(Path(args.output).exists())

    def test_cli_abbreviation_rejected(self):
        with mock.patch("sys.stderr"):
            with self.assertRaises(SystemExit):
                tool.parser().parse_args(["--execute-file"])

    def test_candidate_plan_requires_all_pinned_reused_sources(self):
        args = self.args()
        args.compare_full_fusion = True
        with self.assertRaisesRegex(ValueError, "source inventory"):
            tool.run(args)
        source_map = {"runtime/experiments/native_policy_overnight/" + name: "a" * 64
                      for name in tool._CANDIDATE_SOURCES}
        manifest = {"schema": "private-overnight-kit-v1", "files": source_map}
        def read(path, digest):
            if str(path).endswith("kit.json"):
                return json.dumps(manifest).encode()
            self.assertEqual(digest, "a" * 64)
            return b"source"
        with mock.patch.object(tool, "_read", read):
            pins = tool.candidate_sources(self.root / "kit.json", "a" * 64)
            self.assertEqual(len(pins), len(tool._CANDIDATE_SOURCES))
            del source_map[next(iter(source_map))]
            with self.assertRaises(ValueError):
                tool.candidate_sources(self.root / "kit.json", "a" * 64)


class TensorReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        cls.torch = torch

    def model(self, *, state_step=1., mutate=False, mutate_zero=False):
        torch = self.torch
        class Toy(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.register_buffer("clock", torch.zeros(1, dtype=torch.float64))
                self.register_buffer("last_actor_output", torch.zeros((1, 12)))
                self.register_buffer("last_observation", torch.zeros((1, 74)))
                self.controller = torch.nn.Module()
                self.controller.register_buffer("nominal", torch.zeros(12, dtype=torch.float64))
            def reset(self, ids):
                self.clock.zero_()
                self.last_actor_output.zero_()
                self.last_observation.zero_()
            def forward(self, *values):
                for value, width in zip(values, tool._WIDTHS):
                    if value.shape != (1, width) or not torch.isfinite(value).all():
                        raise ValueError("input shape/finite")
                _, gravity, command, q, _, h = values
                if (not torch.isclose(torch.linalg.vector_norm(gravity), torch.tensor(1.))
                        or gravity[0, 2] == 0. or (h < 0.).any() or (h > 1.).any()
                        or command[0, 0] > .46 or abs(command[0, 2]) > .25
                        or (abs(command[0, :2]) > 0.).all()
                        or (command[0, 0] != 0. and command[0, 2] != 0.)):
                    raise ValueError("domain")
                self.clock.add_(state_step)
                if mutate:
                    q.add_(1.)
                if mutate_zero:
                    values[0][0, 0].neg_()
                return torch.zeros((1, 12))
        return Toy()

    def frames(self):
        _, records = fixture()
        frames = [(tuple(r["observed"]["inputs"][k] for k in tool.INPUT_KEYS),
                   tuple(r["observed"][k] for k, _ in tool._SAVED_OUTPUTS))
                  for r in records]
        return tool.tensor_frames(self.torch, frames)

    def test_all_saved_outputs_named_state_and_26_rejections(self):
        with mock.patch.object(tool, "verify_aliases"):
            result = tool.compare_saved(self.torch, (self.model(), self.model()), self.frames())
        self.assertEqual(result["saved_recurrent_calls"], 501)
        self.assertEqual(result["rejection_count"], 26)
        self.assertTrue(result["floating_bits_exact"])

    def test_equal_targets_do_not_hide_changed_named_state(self):
        with mock.patch.object(tool, "verify_aliases"):
            with self.assertRaises(ValueError):
                tool.compare_saved(self.torch, (self.model(), self.model(state_step=2.)), self.frames())

    def test_mutated_input_rejected(self):
        with mock.patch.object(tool, "verify_aliases"):
            with self.assertRaisesRegex(ValueError, "Input mutated"):
                tool.compare_saved(self.torch, (self.model(mutate=True), self.model()), self.frames())

    def test_signed_zero_bits_cannot_hide_behind_torch_equal(self):
        with self.assertRaisesRegex(ValueError, "bits differ"):
            tool._compare_bits(self.torch, self.torch.tensor([-0.]), self.torch.tensor([0.]), "zero")

    def test_saved_input_signed_zero_mutation_rejected_for_either_model(self):
        for slot in range(2):
            with self.subTest(slot=slot):
                frames = self.frames()
                frames[0][0][0][0, 0] = -0.
                models = [self.model(), self.model()]
                models[slot] = self.model(mutate_zero=True)
                with self.assertRaisesRegex(ValueError, "bits differ: Input mutated"):
                    tool.compare_saved(self.torch, models, frames)

    def test_same_model_object_rejected_before_reset_or_forward(self):
        for kind in ("saved", "paired", "synthetic"):
            with self.subTest(kind=kind):
                model = self.model()
                model.clock.fill_(7.)
                with self.assertRaisesRegex(ValueError, "independent policy instances"):
                    if kind == "saved":
                        tool.compare_saved(self.torch, (model, model), self.frames())
                    elif kind == "paired":
                        tool.paired_timing(self.torch, (model, model), self.frames())
                    else:
                        tool.synthetic_parity(self.torch, model, model)
                self.assertEqual(model.clock.item(), 7.)

    def test_timed_warmup_signed_zero_mutation_rejected(self):
        with self.assertRaisesRegex(ValueError, "bits differ: Warmup input mutated"):
            tool.timed_replay(self.torch, self.model(mutate_zero=True), self.frames())

    def test_paired_warmup_signed_zero_mutation_rejected(self):
        with self.assertRaisesRegex(ValueError, "bits differ: Paired warmup input mutated"):
            tool.paired_timing(self.torch, (self.model(), self.model(mutate_zero=True)), self.frames())

    def test_synthetic_signed_zero_mutation_rejected(self):
        with self.assertRaisesRegex(ValueError, "bits differ: Synthetic input mutated"):
            tool.synthetic_parity(self.torch, self.model(), self.model(mutate_zero=True))

    def test_timed_input_bits_checked_after_forward_outside_timing(self):
        for kind in ("single", "paired"):
            with self.subTest(kind=kind):
                model, calls = self.model(), [0]
                def flip_after_warmup(_model, inputs, _output):
                    calls[0] += 1
                    if calls[0] == 11:
                        inputs[0][0, 0].neg_()
                model.register_forward_hook(flip_after_warmup)
                with self.assertRaisesRegex(ValueError, "bits differ: .*input mutated"):
                    if kind == "single":
                        tool.timed_replay(self.torch, model, self.frames())
                    else:
                        tool.paired_timing(self.torch, (model, self.model()), self.frames())

    def test_timing_replays_full_recurrent_sequence_and_saved_outputs(self):
        result = tool.timed_replay(self.torch, self.model(), self.frames())
        self.assertEqual(result["wall"]["count"], 501)
        self.assertEqual(len(result["raw_thread_cpu_ns"]), 501)
        self.assertTrue(all(v >= 0 for v in result["raw_thread_cpu_ns"]))

    def test_profiler_count_strict_and_separate(self):
        for count in (True, 0, 502):
            with self.assertRaises(ValueError):
                tool.operator_profile(self.torch, self.model(), self.frames(), count)

    def test_paired_timing_recurrence_and_saved_outputs_for_both_models(self):
        one, two = self.model(), self.model()
        result = tool.paired_timing(self.torch, (one, two), self.frames())
        self.assertEqual(set(result), {"current_scalar", "full_fusion_candidate"})
        self.assertEqual(one.clock.item(), 501.)
        self.assertEqual(two.clock.item(), 501.)
        for values in result.values():
            self.assertEqual(values["wall"]["count"], 501)
            self.assertEqual(len(values["raw_thread_cpu_ns"]), 501)

    def test_paired_timing_rejects_same_target_but_changed_state(self):
        with self.assertRaises(ValueError):
            tool.paired_timing(self.torch, (self.model(), self.model(state_step=2.)), self.frames())

    def test_synthetic_parity_exercises_240_and_reset125(self):
        one, two = self.model(), self.model()
        result = tool.synthetic_parity(self.torch, one, two)
        self.assertEqual(result["frames"], 240)
        self.assertEqual(result["reset_before"], [0, 125])
        self.assertEqual(one.clock.item(), 115.)


if __name__ == "__main__":
    unittest.main()
