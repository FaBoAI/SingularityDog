"""Build and replay a CPU zero-projection-cache candidate on 500 saved inputs."""
import argparse
import copy
import io
import json
import os
from pathlib import Path

from ..contracts import OPTIONS, environment, reference_policy, require, sha
from ..lean_swing_deployment import DeployableSwingPolicy
from ..loader import load_verified
from ..model_call_fastpath.run import compile_actor, compare_500, load_records, timing
from ..model_call_fastpath.run_step import compare_synthetic_240
from ..verification import compare_state, compare_tensor
from ..view_cache import generate_core
from .generator import generate_projection_core

HERE = Path(__file__).resolve().parent


def compare_projection_diagnostics(torch, cached, candidate, frames):
    """Exercise all nine native projection outputs through the full step dict."""
    ids = torch.tensor([0], dtype=torch.long)
    maxima = {}
    zero_requested_legs = 0
    requested_legs = 0
    keys = None
    with torch.inference_mode():
        cached.reset(ids)
        candidate.reset(ids)
        for index, inputs in enumerate(frames):
            values = []
            for model in (cached, candidate):
                obs = model.controller.observation(*inputs)
                raw = model.actor(obs)
                values.append(model.controller.step(raw, inputs[2]))
            old, new = values
            require(old.keys() == new.keys(), "Diagnostic field names differ")
            if keys is None:
                keys = list(old)
            for key in old:
                compare_tensor(torch, old[key], new[key], "diagnostic:" + key,
                               maxima, exact=True)
            compare_state(torch, cached, candidate, maxima, exact=True)
            request = old["l13_projection_requested_m"]
            zero_requested_legs += int((request == 0).sum())
            requested_legs += request.numel()
    require(all(value == 0.0 for value in maxima.values()),
            "Projection diagnostic parity failed")
    return {"saved_recurrent_steps": len(frames), "diagnostic_fields": keys,
            "all_fields_exact": True, "all_named_state_exact": True,
            "zero_requested_legs": zero_requested_legs,
            "total_requested_legs": requested_legs, "max_errors": maxima}


def run(args):
    import torch
    output = Path(args.output).expanduser().absolute()
    require(output.parent.is_dir() and not output.exists(), "Use a new private output directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Generated artifacts must remain outside Git")
    output.mkdir(mode=0o700)
    os.chmod(output, 0o700)
    report = {"schema": "native-zero-projection-cache-fileonly-r1", "status": "INCOMPLETE",
              "hardware_opened": False, "output_allowed": False,
              "approved_for_runtime": False, "live_50hz_verified": False,
              "environment": environment(torch),
              "source_hashes": {name: sha((HERE / name).read_bytes()) for name in
                                ("projection.cpp", "torch_projection.cpp", "generator.py", "run.py")},
              "baseline_manifest_sha256": args.baseline_sha,
              "saved_records_sha256": args.records_sha}
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        baseline, _ = load_verified(args.baseline_manifest,
            expected_manifest_sha256=args.baseline_sha, bundle=args.bundle)
        frames = load_records(torch, args.records, args.records_sha)
        library = output / "projection_zero_cache_fileonly.so"
        report["compile_ms"] = compile_actor(torch, library, HERE / "torch_projection.cpp")
        torch.ops.load_library(str(library))
        report["library_sha256"] = sha(library.read_bytes())
        reference, _ = reference_policy(args.bundle)
        cached_core, cached_proof = generate_core(output / "cached_view_core.py")
        fused_core, fused_proof = generate_projection_core(
            output / "cached_view_core.py", output / "zero_cached_projection_core.py",
            cached_proof["generated_core_sha256"])
        report["cached_view_source_proof"] = cached_proof
        report["zero_cached_projection_source_proof"] = fused_proof

        def make(core):
            policy = DeployableSwingPolicy(copy.deepcopy(reference.actor), **OPTIONS).eval()
            policy.controller = core(1, "cpu", **OPTIONS)
            return torch.jit.script(policy)

        cached = make(cached_core)
        candidate = make(fused_core)
        require("sd_projection_zero_cache_fileonly_r1::project" in str(candidate.inlined_graph),
                "Cached projection missing from TorchScript")
        require("sd_projection_fileonly_r1::project" in str(cached.inlined_graph),
                "Original projection missing from comparator")
        model_file = output / "projection_zero_cached_fileonly.pt"
        torch.jit.save(candidate, str(model_file))
        reloaded = torch.jit.load(io.BytesIO(model_file.read_bytes()), map_location="cpu").eval()
        models = {"native": baseline, "cached": cached,
                  "projection_zero_cached": candidate,
                  "projection_zero_reloaded": reloaded}
        report["synthetic_validation"] = compare_synthetic_240(torch, models)
        report["validation"] = compare_500(torch, models, frames)
        report["projection_diagnostics"] = compare_projection_diagnostics(
            torch, cached, candidate, frames)
        report["model_sha256"] = sha(model_file.read_bytes())
        report["timing"] = timing(torch, {key: models[key] for key in
                                 ("native", "cached", "projection_zero_cached")}, frames)
        report["status"] = "PASS_EXACT_FILE_ONLY"
        report["scope"] = "Saved CPU model forward only; no acquisition, output, full-cycle, or Jetson claim"
    except BaseException as error:
        report["status"] = "FAILED"
        report["error"] = type(error).__name__ + ": " + str(error)
        raise
    finally:
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-manifest", required=True)
    parser.add_argument("--baseline-sha", required=True)
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--records", required=True)
    parser.add_argument("--records-sha", required=True)
    parser.add_argument("--output", required=True)
    report = run(parser.parse_args())
    print(json.dumps({key: report[key] for key in ("status", "validation", "timing")}, indent=2))


if __name__ == "__main__":
    main()
