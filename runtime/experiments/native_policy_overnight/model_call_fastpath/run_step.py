"""Build and measure the fixed-shape C++ recurrent-step candidate, file only."""
import argparse
import copy
import io
import json
import os
from pathlib import Path
import shutil

from ..contracts import OPTIONS, environment, pinned, reference_policy, require, sha
from ..lean_swing_deployment import DeployableSwingPolicy
from ..loader import load_verified
from ..verification import compare_state, compare_tensor, frame
from ..view_cache import generate_core
from .run import compile_actor, compare_500, load_records, timing
from .step_generator import generate_step_core

HERE = Path(__file__).resolve().parent


def compare_synthetic_240(torch, models):
    ids = torch.tensor([0], dtype=torch.long)
    nominal = models["native"].controller.nominal.float().reshape(1,12)
    maxima = {}
    names = tuple(models)
    with torch.inference_mode():
        for index in range(240):
            if index in (0, 125):
                for model in models.values():
                    model.reset(ids)
            inputs = frame(torch, nominal, index)
            before = tuple(x.clone() for x in inputs)
            targets = {name: model(*inputs) for name, model in models.items()}
            for name in names[1:]:
                compare_tensor(torch, targets[names[0]], targets[name],
                               "synthetic_target:" + name, maxima, exact=True)
                compare_state(torch, models[names[0]], models[name], maxima, exact=True)
            require(all(torch.equal(a,b) for a,b in zip(before,inputs)),
                    "Synthetic input mutated")
    require(all(value == 0.0 for value in maxima.values()), "Synthetic exact parity failed")
    return {"frames": 240, "reset_before": [0,125], "all_named_state_per_frame": True,
            "max_errors": maxima}


def run(args):
    import torch
    output = Path(args.output).expanduser().absolute()
    require(output.parent.is_dir() and not output.exists(), "Use a new private output directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Generated artifacts must remain outside Git")
    output.mkdir(mode=0o700)
    os.chmod(output, 0o700)
    report = {"schema": "native-step-fusion-fileonly-r1", "status": "INCOMPLETE",
              "hardware_opened": False, "output_allowed": False,
              "approved_for_runtime": False, "live_50hz_verified": False,
              "environment": environment(torch),
              "source_hashes": {name: sha((HERE / name).read_bytes()) for name in
                                ("step.cpp", "step_generator.py", "run_step.py", "run.py")},
              "baseline_manifest_sha256": args.baseline_sha,
              "saved_records_sha256": args.records_sha}
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        baseline, _ = load_verified(args.baseline_manifest,
            expected_manifest_sha256=args.baseline_sha, bundle=args.bundle)
        frames = load_records(torch, args.records, args.records_sha)
        library = output / "step_fileonly.so"
        if args.precompiled_step_library:
            require(args.precompiled_step_sha, "Precompiled library needs a caller pin")
            pinned(args.precompiled_step_library, args.precompiled_step_sha)
            shutil.copyfile(args.precompiled_step_library, library)
            require(sha(library.read_bytes()) == args.precompiled_step_sha,
                    "Precompiled library copy changed")
            report["compile_ms"] = None
        else:
            report["compile_ms"] = compile_actor(torch, library, HERE / "step.cpp")
        torch.ops.load_library(str(library))
        report["library_sha256"] = sha(library.read_bytes())
        reference, _ = reference_policy(args.bundle)
        cached_core, cached_proof = generate_core(output / "cached_view_core.py")
        fused_core, fused_proof = generate_step_core(output / "cached_view_core.py",
                output / "fused_step_core.py", cached_proof["generated_core_sha256"])
        report["cached_view_source_proof"] = cached_proof
        report["fused_step_source_proof"] = fused_proof

        def make(core):
            policy = DeployableSwingPolicy(copy.deepcopy(reference.actor), **OPTIONS).eval()
            policy.controller = core(1, "cpu", **OPTIONS)
            return torch.jit.script(policy)

        cached = make(cached_core)
        candidate = make(fused_core)
        require("sd_step_fileonly_r1::step" in str(candidate.inlined_graph),
                "Native recurrent step missing from TorchScript")
        require("sd_projection_fileonly_r1::project" in str(candidate.inlined_graph),
                "Native projection missing from TorchScript")
        model_file = output / "step_fused_cached_fileonly.pt"
        torch.jit.save(candidate, str(model_file))
        reloaded = torch.jit.load(io.BytesIO(model_file.read_bytes()), map_location="cpu").eval()
        models = {"native": baseline, "cached": cached,
                  "step_fused_cached": candidate, "step_fused_reloaded": reloaded}
        report["synthetic_validation"] = compare_synthetic_240(torch, models)
        report["validation"] = compare_500(torch, models, frames)
        report["model_sha256"] = sha(model_file.read_bytes())
        report["timing"] = timing(torch, {key: models[key] for key in
                    ("native", "cached", "step_fused_cached")}, frames)
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
    parser.add_argument("--precompiled-step-library")
    parser.add_argument("--precompiled-step-sha")
    result = run(parser.parse_args())
    print(json.dumps({key: result[key] for key in ("status", "validation", "timing")}, indent=2))


if __name__ == "__main__":
    main()
