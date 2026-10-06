"""Compile-only CPU fixture parity for the isolated target-tail operator.

This exercises actual C++ kernels and a deterministic test actor, without real
model artifacts, devices or target access. It does not replace saved-target
output validation by saved_profile under the verified scalar loader.
"""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path
import types


def run(output):
    import torch
    from ..contracts import OPTIONS, HERE, sha, require
    from ..lean_swing_deployment import DeployableSwingPolicy
    from ..verification import frame, rejections, reject_reason
    from ..view_cache.generator import generate_core, verify_aliases
    from ..model_call_fastpath.run import compile_actor
    from ..model_call_fastpath.step_generator import generate_step_core
    from ..model_call_fastpath import saved_input_profile as replay
    from .saved_profile import target_candidate
    output = replay._output_path(output)
    output.mkdir(mode=0o700)
    torch.set_num_threads(1); torch.set_num_interop_threads(1); torch.manual_seed(1729)
    libraries = {}
    for name, source in (("projection", "torch_projection.cpp"),
                         ("scalar", "model_call_fastpath/step_scalar.cpp"),
                         ("target", "target_tail_fusion/target.cpp")):
        path = output / (name + ".so")
        elapsed = compile_actor(torch, path, HERE / source)
        libraries[name] = {"path": str(path), "sha256": sha(path.read_bytes()),
                           "source_sha256": sha((HERE / source).read_bytes()), "compile_ms": elapsed}
        if name != "target":
            torch.ops.load_library(str(path))
    _, cache = generate_core(output / "baseline_cached.py")
    core, _ = generate_step_core(output / "baseline_cached.py", output / "baseline_step.py",
                                cache["generated_core_sha256"])
    actor = torch.nn.Sequential(torch.nn.Linear(74, 128), torch.nn.ELU(),
        torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 64),
        torch.nn.ELU(), torch.nn.Linear(64, 12)).eval()
    baseline = DeployableSwingPolicy(actor, **OPTIONS).eval()
    baseline.controller = core(1, "cpu", **OPTIONS)
    baseline = torch.jit.script(baseline)
    args = types.SimpleNamespace(target_library=libraries["target"]["path"],
                                  target_library_sha256=libraries["target"]["sha256"])
    candidate, proof = target_candidate(torch, baseline, args, output / "result.json")
    ids = torch.tensor([0], dtype=torch.long)
    nominal = baseline.controller.nominal.float().reshape(1, 12)
    local_frames = []
    with torch.inference_mode():
        baseline.reset(ids); candidate.reset(ids)
        for index in range(501):
            inputs = frame(torch, nominal, index)
            before = tuple(value.clone() for value in inputs)
            values = [model(*inputs) for model in (baseline, candidate)]
            for model in (baseline, candidate):
                replay._input_bits(torch, before, inputs, "fixture input mutated")
            for one, two in zip(replay._outputs(baseline, values[0]),
                                replay._outputs(candidate, values[1])):
                replay._compare_bits(torch, one, two, "fixture output")
            replay._state_bits(torch, baseline, candidate)
            local_frames.append((inputs, tuple(value.clone() for value in
                                               replay._outputs(baseline, values[0]))))
    saved_like = replay.compare_saved(torch, (baseline, candidate), local_frames)
    synthetic = replay.synthetic_parity(torch, baseline, candidate)
    direct_cases = []
    with torch.inference_mode():
        # Probe branch/rounding boundaries through the genuine scalar op and
        # original projection: clip thresholds, bootstrap crossing, phase wrap,
        # command thresholds, signed zero and reference guard partial state.
        for raw_value in (-1.0000001, -1., -.9999999, -0., 0., .9999999, 1., 1.0000001):
            for elapsed in (0., 1.98, 2., 2.02):
                for model in (baseline, candidate):
                    model.reset(ids); model.controller.elapsed.fill_(elapsed)
                    model.controller.phase.fill_(.99)
                raw = torch.full((1, 12), raw_value, dtype=torch.float32)
                command = torch.tensor([[.30, 0., 0.]], dtype=torch.float32)
                aa = baseline.controller.step_target(raw, command)
                bb = candidate.controller.step_target(raw, command)
                replay._compare_bits(torch, aa, bb, "clip/bootstrap/phase boundary")
                replay._state_bits(torch, baseline, candidate)
                direct_cases.append("clip_bootstrap_phase")
        for command in ((1e-9, 0., 0.), (1.0001e-9, 0., 0.), (.46, 0., 0.),
                        (.460001, 0., 0.), (-.12, 0., 0.), (-.120001, 0., 0.),
                        (0., 0., .25), (0., 0., .250001), (.1, .1, 0.)):
            reasons = []; values = []
            for model in (baseline, candidate):
                model.reset(ids)
                try:
                    values.append(model.controller.step_target(torch.zeros((1, 12)),
                        torch.tensor([command], dtype=torch.float32)))
                    reasons.append(None)
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
            require(reasons[0] == reasons[1], "Boundary rejection differs")
            if reasons[0] is None:
                replay._compare_bits(torch, *values, "command boundary")
            replay._state_bits(torch, baseline, candidate)
            direct_cases.append("command_boundary")
        for value in (float("nan"), float("inf")):
            reasons = []
            for model in (baseline, candidate):
                model.reset(ids)
                raw = torch.zeros((1, 12)); raw[0, 0] = value
                try:
                    model.controller.step_target(raw, torch.zeros((1, 3)))
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
                else:
                    raise ValueError("Nonfinite raw action accepted")
            require(reasons[0] == reasons[1], "Raw rejection reason differs")
            replay._state_bits(torch, baseline, candidate)
            direct_cases.append("raw_nonfinite")
        for model in (baseline, candidate):
            model.reset(ids); model.controller.safe_lower[0] = .2
        reasons = []
        for model in (baseline, candidate):
            try:
                model.controller.step_target(torch.zeros((1, 12)), torch.zeros((1, 3)))
            except (ValueError, RuntimeError, torch.jit.Error) as error:
                reasons.append(reject_reason(error))
            else:
                raise ValueError("Broken reference margin accepted")
        require(reasons == ["L13 reference itself violates registered 0.02 rad joint margins"] * 2,
                "Reference guard wording/order differs")
        replay._state_bits(torch, baseline, candidate)
        require(bool(baseline.controller.elapsed[0] == .02), "Rejected tail lost original partial state")
        direct_cases.append("reference_margin_partial_state")
        verify_aliases(baseline.controller); verify_aliases(candidate.controller)
    result = {"schema": "private.target-tail-cpp-local-validation.v1", "status": "PASS_LOCAL_CPP_PARITY",
        "environment": replay.environment(torch), "libraries": libraries, "candidate": proof,
        "test_actor_only": True, "real_model_or_target_loaded": False,
        "synthetic_501": saved_like, "synthetic_240": synthetic,
        "direct_boundary_cases": len(direct_cases), "named_state_count":
            len(list(baseline.named_buffers())) + len(list(baseline.named_parameters())),
        "hardware_opened": False, "output_allowed": False, "approved_for_runtime": False,
        "target_saved_output_parity_claimed": False, "latency_improvement_proven": False}
    with (output / "local-validation.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--output", required=True)
    p.add_argument("--execute-file-only", action="store_true")
    args = p.parse_args(argv)
    if not args.execute_file_only:
        raise ValueError("Explicit CPU/file-only execution required; use saved_profile for PLAN")
    result = run(args.output)
    print(json.dumps({key: result[key] for key in ("status", "direct_boundary_cases", "named_state_count")}))


if __name__ == "__main__":
    main()
