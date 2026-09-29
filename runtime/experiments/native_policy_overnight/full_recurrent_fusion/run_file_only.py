"""Build and compare a combined CPU recurrent-path candidate, without hardware.

The candidate changes only the three methods named by ``generator.py``.  Its
scalar step is a separately pinned library because the scalar and ATen step
implementations register the same Torch operator name.  A PASS is file-only
evidence, never permission to use the model for motor output.
"""
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
from ..model_call_fastpath.run import compile_actor, compare_500, load_records, timing
from ..model_call_fastpath.run_step import compare_synthetic_240
from ..projection_zero_cache.run import compare_projection_diagnostics
from ..verification import frame
from ..view_cache import generate_core
from .generator import generate_core as generate_full_core


HERE = Path(__file__).resolve().parent
STEP_SOURCE = HERE.parent / "model_call_fastpath" / "step_scalar.cpp"
OBSERVATION_SOURCE = HERE.parent / "observation_fusion" / "observation.cpp"
PROJECTION_SOURCE = HERE.parent / "projection_zero_cache" / "torch_projection.cpp"


def run(args):
    import torch

    output = Path(args.output).expanduser().absolute()
    require(output.parent.is_dir() and not output.exists(), "Use a new private output directory")
    require(not any((parent / ".git").exists() for parent in
                    (output.parent, *output.parents)),
            "Generated artifacts must remain outside Git")
    pinned(STEP_SOURCE, args.scalar_source_sha)
    pinned(args.scalar_library, args.scalar_library_sha)
    require(bool(args.precompiled_observation) == bool(args.precompiled_observation_sha),
            "Precompiled observation path and SHA must be supplied together")
    require(bool(args.precompiled_projection) == bool(args.precompiled_projection_sha),
            "Precompiled projection path and SHA must be supplied together")
    for path, digest in ((args.precompiled_observation, args.precompiled_observation_sha),
                         (args.precompiled_projection, args.precompiled_projection_sha)):
        if path:
            pinned(path, digest)
    output.mkdir(mode=0o700)
    report = {
        "schema": "native-full-recurrent-fusion-fileonly-r1",
        "status": "INCOMPLETE",
        "hardware_opened": False,
        "output_allowed": False,
        "approved_for_runtime": False,
        "live_50hz_verified": False,
        "environment": environment(torch),
        "baseline_manifest_sha256": args.baseline_sha,
        "saved_records_sha256": args.records_sha,
        "scalar_library_sha256": args.scalar_library_sha,
        "source_hashes": {name: sha(path.read_bytes()) for name, path in {
            "generator.py": HERE / "generator.py",
            "run_file_only.py": Path(__file__),
            "step_scalar.cpp": STEP_SOURCE,
            "step_generator.py": HERE.parent / "model_call_fastpath" / "step_generator.py",
            "observation.cpp": OBSERVATION_SOURCE,
            "observation_generator.py": HERE.parent / "observation_fusion" / "generator.py",
            "projection.cpp": HERE.parent / "projection_zero_cache" / "projection.cpp",
            "torch_projection.cpp": PROJECTION_SOURCE,
            "projection_generator.py": HERE.parent / "projection_zero_cache" / "generator.py",
            "view_cache_generator.py": HERE.parent / "view_cache" / "generator.py",
        }.items()},
    }
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        baseline, _ = load_verified(args.baseline_manifest,
            expected_manifest_sha256=args.baseline_sha, bundle=args.bundle)
        frames = load_records(torch, args.records, args.records_sha)
        # The scalar library is loaded from the caller's explicit path and SHA.
        # Do not copy or silently rebuild the pinned target-local binary.
        torch.ops.load_library(str(Path(args.scalar_library).resolve()))
        observation_library = output / "observation_fileonly.so"
        projection_library = output / "projection_zero_cache_fileonly.so"
        if args.precompiled_observation:
            shutil.copyfile(args.precompiled_observation, observation_library)
            report["observation_compile_ms"] = None
        else:
            report["observation_compile_ms"] = compile_actor(
                torch, observation_library, OBSERVATION_SOURCE)
        if args.precompiled_projection:
            shutil.copyfile(args.precompiled_projection, projection_library)
            report["projection_compile_ms"] = None
        else:
            report["projection_compile_ms"] = compile_actor(
                torch, projection_library, PROJECTION_SOURCE)
        if args.precompiled_observation:
            require(sha(observation_library.read_bytes()) == args.precompiled_observation_sha,
                    "Copied observation library SHA mismatch")
        if args.precompiled_projection:
            require(sha(projection_library.read_bytes()) == args.precompiled_projection_sha,
                    "Copied projection library SHA mismatch")
        torch.ops.load_library(str(output / "observation_fileonly.so"))
        torch.ops.load_library(str(output / "projection_zero_cache_fileonly.so"))
        report["observation_library_sha256"] = sha((output / "observation_fileonly.so").read_bytes())
        report["projection_library_sha256"] = sha((output / "projection_zero_cache_fileonly.so").read_bytes())

        reference, _ = reference_policy(args.bundle)
        cached_core, cached_proof = generate_core(output / "cached_view_core.py")
        fused_core, fused_proof = generate_full_core(
            output / "cached_view_core.py", output / "full_fused_core.py",
            cached_proof["generated_core_sha256"])
        report["cached_view_source_proof"] = cached_proof
        report["full_fusion_source_proof"] = fused_proof

        def make(core):
            policy = DeployableSwingPolicy(copy.deepcopy(reference.actor), **OPTIONS).eval()
            policy.controller = core(1, "cpu", **OPTIONS)
            return torch.jit.script(policy)

        cached = make(cached_core)
        candidate = make(fused_core)
        graph = str(candidate.inlined_graph)
        for operator in ("sd_observation_fileonly_r1::observe",
                         "sd_step_fileonly_r1::step",
                         "sd_projection_zero_cache_fileonly_r1::project"):
            require(operator in graph, "Missing fused operator: " + operator)
        model_file = output / "full_recurrent_fused_fileonly.pt"
        torch.jit.save(candidate, str(model_file))
        reloaded = torch.jit.load(io.BytesIO(model_file.read_bytes()), map_location="cpu").eval()
        models = {"native": baseline, "cached": cached,
                  "full_fused": candidate, "full_fused_reloaded": reloaded}
        report["synthetic_validation"] = compare_synthetic_240(torch, models)
        report["validation"] = compare_500(torch, models, frames)
        report["projection_diagnostics"] = compare_projection_diagnostics(
            torch, cached, candidate, frames)
        # Saved STOP cycles request no projection. Exercise the projected
        # walking-command paths separately, including the recurrent reset at
        # synthetic frame 125, before trusting a speed result from those paths.
        nominal = baseline.controller.nominal.float().reshape(1, 12)
        synthetic = [frame(torch, nominal, index) for index in range(240)]
        projected_parts = [compare_projection_diagnostics(torch, cached, candidate, part)
                           for part in (synthetic[:125], synthetic[125:])]
        report["projection_synthetic_diagnostics"] = {
            "steps": sum(part["saved_recurrent_steps"] for part in projected_parts),
            "reset_before": [0, 125],
            "all_fields_exact": all(part["all_fields_exact"] for part in projected_parts),
            "all_named_state_exact": all(part["all_named_state_exact"] for part in projected_parts),
            "zero_requested_legs": sum(part["zero_requested_legs"] for part in projected_parts),
            "total_requested_legs": sum(part["total_requested_legs"] for part in projected_parts),
        }
        report["model_sha256"] = sha(model_file.read_bytes())
        report["timing"] = timing(torch, {key: models[key] for key in
                                  ("native", "cached", "full_fused")}, frames)
        report["synthetic_timing"] = timing(torch, {key: models[key] for key in
                                            ("cached", "full_fused")}, synthetic)
        report["status"] = "PASS_EXACT_FILE_ONLY"
        report["scope"] = ("Saved CPU model forward only; no acquisition, motor output, "
                           "full-cycle or 50 Hz validation")
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
    parser.add_argument("--scalar-source-sha", required=True)
    parser.add_argument("--scalar-library", required=True)
    parser.add_argument("--scalar-library-sha", required=True)
    parser.add_argument("--precompiled-observation")
    parser.add_argument("--precompiled-observation-sha")
    parser.add_argument("--precompiled-projection")
    parser.add_argument("--precompiled-projection-sha")
    parser.add_argument("--output", required=True)
    report = run(parser.parse_args())
    print(json.dumps({key: report[key] for key in
                      ("status", "validation", "projection_diagnostics",
                       "projection_synthetic_diagnostics", "timing", "synthetic_timing")}, indent=2))


if __name__ == "__main__":
    main()
