"""File-only actor-fusion build, exact recurrent comparison, and CPU timing.

No transport, device, output, package install, or persistent CPU setting.
Run in a fresh process with explicit pinned artifacts and a new private output.
"""
import argparse
import copy
import io
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time

from ..contracts import INPUT_KEYS, OPTIONS, environment, pinned, reference_policy, require, sha, strict_json
from ..lean_swing_deployment import DeployableSwingPolicy
from ..loader import load_verified
from ..verification import compare_state, compare_tensor, rejections, reject_reason
from ..view_cache import generate_core, verify_aliases
from .candidate import FusedActorPolicy

HERE = Path(__file__).resolve().parent


def compile_actor(torch, destination, source=None):
    from torch.utils.cpp_extension import include_paths, library_paths
    require(platform.system() in ("Darwin", "Linux"), "CPU-only Darwin/Linux build required")
    command = ["c++", "-std=c++20", "-O3", "-ffp-contract=off", "-shared", "-fPIC",
               "-D_GLIBCXX_USE_CXX11_ABI=" + str(int(torch._C._GLIBCXX_USE_CXX11_ABI)),
               str(source or HERE / "actor.cpp"), "-o", str(destination)]
    for path in include_paths():
        command += ["-I", path]
    for path in library_paths():
        command += ["-L", path, "-Wl,-rpath," + path]
    command += ["-ltorch_cpu", "-lc10"]
    started = time.perf_counter_ns()
    result = subprocess.run(command, capture_output=True, text=True, timeout=180)
    require(result.returncode == 0, "Actor C++ build failed: " + result.stderr[-4000:])
    return (time.perf_counter_ns() - started) / 1e6


def load_records(torch, path, digest):
    raw = pinned(path, digest)
    data = strict_json(raw)
    require(type(data) is list and len(data) == 500, "Require 500 saved records")
    frames = []
    for index, row in enumerate(data):
        require(type(row) is dict and row.get("cycle") == index + 1,
                "Saved cycles must be ordered 1..500")
        inputs = row.get("observed", {}).get("inputs")
        require(type(inputs) is dict, "Missing saved six-vector inputs")
        values = []
        for key, width in zip(INPUT_KEYS, (3, 3, 3, 12, 12, 12)):
            value = inputs.get(key)
            require(type(value) is list and len(value) == width and
                    all(type(x) in (int, float) and math.isfinite(x) for x in value),
                    "Invalid saved input " + key)
            values.append(torch.tensor([value], dtype=torch.float32))
        frames.append(tuple(values))
    return frames


def distribution(values):
    ordered = sorted(values)
    count = len(ordered)
    return {"count": count, "median_ms": statistics.median(ordered) / 1e6,
            "p95_ms": ordered[math.ceil(.95 * count) - 1] / 1e6,
            "p99_ms": ordered[math.ceil(.99 * count) - 1] / 1e6,
            "max_ms": ordered[-1] / 1e6}


def compare_500(torch, models, frames):
    ids = torch.tensor([0], dtype=torch.long)
    maxima = {}
    names = tuple(models)
    with torch.inference_mode():
        for model in models.values():
            model.reset(ids)
        for index, inputs in enumerate(frames):
            before = tuple(x.clone() for x in inputs)
            outputs = {name: model(*inputs) for name, model in models.items()}
            for name in names[1:]:
                compare_tensor(torch, outputs[names[0]], outputs[name], "target:" + name,
                               maxima, exact=True)
                compare_state(torch, models[names[0]], models[name], maxima, exact=True)
            require(all(torch.equal(a, b) for a, b in zip(before, inputs)),
                    "Input mutated at saved cycle " + str(index + 1))
        for model in models.values():
            model.reset(ids)
        rejected = []
        for name, inputs in rejections(torch, frames[0]).items():
            reasons = []
            for model in models.values():
                model.reset(ids)
                try:
                    model(*inputs)
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
                else:
                    raise ValueError("Invalid input accepted: " + name)
            require(len(set(reasons)) == 1, "Rejection reason differs: " + name)
            for variant in names[1:]:
                compare_state(torch, models[names[0]], models[variant], maxima, exact=True)
            rejected.append({"case": name, "reason": reasons[0]})
        for model in models.values():
            model.reset(ids)
        for variant in names[1:]:
            compare_state(torch, models[names[0]], models[variant], maxima, exact=True)
        for name in names[1:]:
            verify_aliases(models[name].controller)
    require(len(rejected) == 26 and all(x == 0.0 for x in maxima.values()),
            "Exact comparison incomplete")
    return {"saved_recurrent_calls": len(frames), "all_named_state_per_call": True,
            "observation_actor_target_exact": True, "all_parameters_exact": True,
            "input_mutation": False, "rejection_count": len(rejected),
            "rejections": rejected, "max_errors": maxima,
            "cached_view_aliases_after_reload": True}


def timing(torch, models, frames):
    ids = torch.tensor([0], dtype=torch.long)
    names = tuple(models)
    wall = {name: [] for name in names}
    cpu = {name: [] for name in names}
    with torch.inference_mode():
        for model in models.values():
            model.reset(ids)
            for inputs in frames[:10]:
                model(*inputs)
            model.reset(ids)
        for index, inputs in enumerate(frames):
            outputs = {}
            for offset in range(len(names)):
                name = names[(index + offset) % len(names)]
                start_wall, start_cpu = time.perf_counter_ns(), time.thread_time_ns()
                outputs[name] = models[name](*inputs)
                elapsed_cpu = time.thread_time_ns() - start_cpu
                elapsed_wall = time.perf_counter_ns() - start_wall
                wall[name].append(elapsed_wall)
                cpu[name].append(elapsed_cpu)
            for name in names[1:]:
                require(torch.equal(outputs[names[0]], outputs[name]),
                        "Timed target differs at saved cycle " + str(index + 1))
                compare_state(torch, models[names[0]], models[name], {}, exact=True)
    return {name: {"wall": distribution(wall[name]), "thread_cpu": distribution(cpu[name])}
            for name in names}


def run(args):
    import torch
    output = Path(args.output).expanduser().absolute()
    require(output.parent.is_dir() and not output.exists(), "Use a new private output directory")
    require(not any((parent / ".git").exists() for parent in (output.parent, *output.parents)),
            "Private generated artifacts must remain outside Git")
    output.mkdir(mode=0o700)
    os.chmod(output, 0o700)
    report = {"schema": "native-actor-fusion-fileonly-r1", "status": "INCOMPLETE",
              "hardware_opened": False, "output_allowed": False,
              "approved_for_runtime": False, "live_50hz_verified": False,
              "environment": environment(torch),
              "source_hashes": {name: sha((HERE / name).read_bytes()) for name in
                                ("actor.cpp", "candidate.py", "run.py")},
              "baseline_manifest_sha256": args.baseline_sha,
              "saved_records_sha256": args.records_sha}
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        baseline, provenance = load_verified(args.baseline_manifest,
            expected_manifest_sha256=args.baseline_sha, bundle=args.bundle)
        frames = load_records(torch, args.records, args.records_sha)
        library = output / "actor_fileonly.so"
        report["compile_ms"] = compile_actor(torch, library)
        torch.ops.load_library(str(library))
        report["library_sha256"] = sha(library.read_bytes())
        reference, _ = reference_policy(args.bundle)
        core, source_proof = generate_core(output / "cached_view_core.py")
        report["cached_view_source_proof"] = source_proof

        def make(cls):
            policy = cls(copy.deepcopy(reference.actor), **OPTIONS).eval()
            policy.controller = core(1, "cpu", **OPTIONS)
            return torch.jit.script(policy)

        cached = make(DeployableSwingPolicy)
        candidate = make(FusedActorPolicy)
        require("sd_actor_fileonly_r1::forward" in str(candidate.inlined_graph),
                "Fused actor op missing from TorchScript")
        require("sd_projection_fileonly_r1::project" in str(candidate.inlined_graph),
                "Native projection op missing from TorchScript")
        model_file = output / "actor_fused_cached_fileonly.pt"
        torch.jit.save(candidate, str(model_file))
        reloaded = torch.jit.load(io.BytesIO(model_file.read_bytes()), map_location="cpu").eval()
        models = {"native": baseline, "cached": cached,
                  "actor_fused_cached": candidate, "actor_fused_reloaded": reloaded}
        report["validation"] = compare_500(torch, models, frames)
        report["model_sha256"] = sha(model_file.read_bytes())
        report["timing"] = timing(torch, {key: models[key] for key in
                    ("native", "cached", "actor_fused_cached")}, frames)
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
    result = run(parser.parse_args())
    print(json.dumps({key: result[key] for key in ("status", "validation", "timing")}, indent=2))


if __name__ == "__main__":
    main()
