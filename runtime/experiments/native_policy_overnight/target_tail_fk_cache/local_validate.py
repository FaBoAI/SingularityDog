"""Actual C++ CPU fixture, three independent models; no device/model artifacts.

A deterministic test actor is used. This is local kernel equivalence validation,
not the original Jetson saved-output test performed by saved_profile.
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys
import types


def direct_boundaries(torch, models, replay):
    ids = torch.tensor([0], dtype=torch.long); cases = []
    def check(raw, command):
        outputs, reasons = [], []
        for model in models:
            try:
                outputs.append(model.controller.step_target(raw, command)); reasons.append(None)
            except (ValueError, RuntimeError, torch.jit.Error) as error:
                reasons.append(replay.reject_reason(error))
        replay.require(len(reasons) == 3 and len(set(reasons)) == 1, "Three-way boundary rejection differs")
        if reasons[0] is None:
            for other in outputs[1:]:
                replay._compare_bits(torch, outputs[0], other, "Three-way boundary output")
        for other in models[1:]:
            replay._state_bits(torch, models[0], other)
        return reasons[0]
    with torch.inference_mode():
        for raw_value in (-1.0000001, -1., -.9999999, -0., 0., .9999999, 1., 1.0000001):
            for elapsed in (0., 1.98, 2., 2.02):
                for model in models:
                    model.reset(ids); model.controller.elapsed.fill_(elapsed); model.controller.phase.fill_(.99)
                check(torch.full((1, 12), raw_value, dtype=torch.float32),
                      torch.tensor([[.30, 0., 0.]], dtype=torch.float32)); cases.append("clip_bootstrap_phase")
        for command in ((1e-9, 0., 0.), (1.0001e-9, 0., 0.), (.46, 0., 0.),
                        (.460001, 0., 0.), (-.12, 0., 0.), (-.120001, 0., 0.),
                        (0., 0., .25), (0., 0., .250001), (.1, .1, 0.)):
            for model in models:model.reset(ids)
            check(torch.zeros((1, 12)), torch.tensor([command], dtype=torch.float32))
            cases.append("command_boundary")
        for value in (float("nan"), float("inf")):
            for model in models:model.reset(ids)
            raw = torch.zeros((1, 12)); raw[0, 0] = value
            replay.require(check(raw, torch.zeros((1, 3))) is not None, "Nonfinite raw accepted")
            cases.append("raw_nonfinite")
        for model in models:model.reset(ids); model.controller.safe_lower[0] = .2
        reason = check(torch.zeros((1, 12)), torch.zeros((1, 3)))
        replay.require(reason == "L13 reference itself violates registered 0.02 rad joint margins",
                       "Original reference guard differs")
        replay.require(bool(models[0].controller.elapsed[0] == .02), "Original partial state differs")
        cases.append("reference_margin_partial_state")
    return len(cases)


def run(output):
    from ..target_tail_fusion import local_validate as first_local
    from . import saved_profile as tool
    from ..contracts import OPTIONS
    from ..lean_swing_deployment import DeployableSwingPolicy
    from ..verification import frame
    from ..view_cache.generator import verify_aliases
    replay = tool.replay; output = replay._output_path(output); output.mkdir(mode=0o700)
    # Compile/load the same original scalar/projection and first candidate once.
    base = first_local.run(output / "first-tail")
    import torch
    proof = base["candidate"]
    tail = torch.jit.load(proof["model_path"], map_location="cpu").eval()
    step = output / "first-tail/baseline_step.py"
    name = "fk_validation_scalar_" + replay._sha(step.read_bytes())[:16]
    spec = importlib.util.spec_from_file_location(name, step)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module)
    scalar = DeployableSwingPolicy(copy.deepcopy(tail.actor), **OPTIONS).eval()
    scalar.controller = module.FusedStepSwingCore(1, "cpu", **OPTIONS)
    scalar = torch.jit.script(scalar); verify_aliases(scalar.controller)
    args = types.SimpleNamespace(fk_library=None, fk_library_sha256=None)
    fk, fk_proof = tool.fk_candidate(torch, scalar, proof, args, output / "result.json")
    models = (scalar, tail, fk); tool.independent_three(models)
    ids = torch.tensor([0], dtype=torch.long); nominal = scalar.controller.nominal.float().reshape(1, 12)
    frames = []
    with torch.inference_mode():
        scalar.reset(ids)
        for index in range(501):
            inputs = frame(torch, nominal, index); values = scalar(*inputs)
            frames.append((inputs, tuple(value.clone() for value in replay._outputs(scalar, values))))
    saved = tool.compare_three(torch, models, frames)
    synthetic = {name: replay.synthetic_parity(torch, scalar, candidate)
                 for name, candidate in zip(tool._NAMES[1:], models[1:])}
    timing = tool.balanced_three_timing(torch, models, frames)
    direct = direct_boundaries(torch, models, replay)
    for model in models:verify_aliases(model.controller)
    result = {"schema": "private.target-tail-fk-cache-local-validation.v1", "status": "PASS_LOCAL_CPP_THREE_WAY_PARITY",
        "first_tail": base, "fk_candidate": fk_proof, "three_way_saved_fixture": saved,
        "synthetic_validation": synthetic, "balanced_timing": timing,
        "direct_boundary_cases": direct, "named_state_count": len(list(scalar.named_buffers())) + len(list(scalar.named_parameters())),
        "test_actor_only": True, "original_target_saved_output_validation": False,
        "real_model_or_target_loaded": False, "hardware_opened": False, "output_allowed": False,
        "approved_for_runtime": False, "whole_cycle_performance_measured": False, "latency_improvement_proven": False}
    with (output / "local-validation.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--output", required=True); p.add_argument("--execute-file-only", action="store_true")
    args = p.parse_args(argv)
    if not args.execute_file_only:raise ValueError("Explicit file-only execution required")
    result = run(args.output)
    print(json.dumps({key: result[key] for key in ("status", "direct_boundary_cases", "named_state_count")}))


if __name__ == "__main__":main()
