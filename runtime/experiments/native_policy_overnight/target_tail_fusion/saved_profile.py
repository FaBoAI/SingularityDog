"""File-only CURRENT scalar versus an isolated ATen step_target port.

PLAN checks bytes and saved evidence only. Explicit execution may build/load a
CPU custom operator; it cannot open transport or qualify a live controller.
The original actor, observation, scalar step and original projection remain.
"""
import copy
import io
import json
import os
from pathlib import Path
import statistics

from ..model_call_fastpath import saved_input_profile as replay

_HERE = Path(__file__).resolve().parent
_SOURCES = ("target_tail_fusion/__init__.py", "target_tail_fusion/target.cpp",
            "target_tail_fusion/generator.py", "target_tail_fusion/saved_profile.py",
            "projection.cpp", "torch_projection.cpp")


def source_path(name):
    # New candidate code may reside alongside a frozen private bootstrap; the
    # verified original loader and its protected sources remain in frozen K13.
    return _HERE / name.split("/", 1)[1] if name.startswith("target_tail_fusion/") else replay.HERE / name


def source_pins(args):
    pins = replay.candidate_sources(args.candidate_source_manifest,
                                    args.candidate_source_manifest_sha256)
    manifest = replay.strict_json(replay._read(args.candidate_source_manifest,
                                              args.candidate_source_manifest_sha256))
    for name in _SOURCES:
        key = "runtime/experiments/native_policy_overnight/" + name
        pin = replay._digest(manifest["files"].get(key))
        replay._read(source_path(name), pin)
        pins[key] = pin
    return pins


def disjoint_state(one, two):
    pointers = []
    for model in (one, two):
        values = list(model.named_buffers()) + list(model.named_parameters())
        pointers.append({value.untyped_storage().data_ptr() for _, value in values
                         if value.numel()})
    replay.require(pointers[0].isdisjoint(pointers[1]), "Candidate shares scalar state storage")


def target_candidate(torch, scalar, args, output):
    from ..contracts import OPTIONS
    from ..lean_swing_deployment import DeployableSwingPolicy
    from ..view_cache.generator import generate_core, verify_aliases
    from ..model_call_fastpath.step_generator import generate_step_core
    from ..model_call_fastpath.run import compile_actor
    from .generator import generate_core as generate_target
    replay.require(not hasattr(torch.ops.sd_target_tail_fileonly_r1, "target"),
                   "Target operator already loaded; fresh process required")
    # These options are embedded in the port. Manifest and original scalar must
    # both describe exactly this fixed single-environment configuration.
    for name, expected in (("num_envs", 1), ("use_plane", True), ("dt", .02),
            ("period", .56), ("duty", .60), ("swing_height", .035),
            ("max_projection", .060), ("stance_widen_m", .02),
            ("residual_alpha", 0.), ("heading_gain", .8),
            ("forward_command_limit", .46), ("hip_residual_scale", .27),
            ("joint_margin", .02)):
        actual = getattr(scalar.controller, name)
        replay.require(type(actual) is type(expected) and actual == expected,
                       "Pinned scalar option differs: " + name)
    verify_aliases(scalar.controller)
    directory = replay._output_path(output.with_name(output.stem + "-target-artifacts"))
    directory.mkdir(mode=0o700)
    library = directory / "target_tail.so"
    if args.target_library:
        raw = replay._read(args.target_library, args.target_library_sha256)
        with library.open("xb") as stream:
            stream.write(raw)
        compile_ms = None
    else:
        compile_ms = compile_actor(torch, library, _HERE / "target.cpp")
    pin = replay._sha(library.read_bytes())
    torch.ops.load_library(str(library))
    replay._read(library, pin)
    _, cache = generate_core(directory / "cached.py")
    _, step = generate_step_core(directory / "cached.py", directory / "step.py",
                                 cache["generated_core_sha256"])
    core, target = generate_target(directory / "step.py", directory / "target.py",
                                  step["generated_source_sha256"])
    policy = DeployableSwingPolicy(copy.deepcopy(scalar.actor), **OPTIONS).eval()
    # Fresh owning buffers avoid TorchScript deepcopy of cached view attributes.
    policy.controller = core(1, "cpu", **OPTIONS)
    candidate = torch.jit.script(policy)
    verify_aliases(candidate.controller)
    model = directory / "target_tail_fileonly.pt"
    torch.jit.save(candidate, str(model))
    raw = model.read_bytes()
    candidate = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    verify_aliases(candidate.controller)
    disjoint_state(scalar, candidate)
    replay._state_bits(torch, scalar, candidate)
    before = set(scalar.controller._c._method_names())
    after = set(candidate.controller._c._method_names())
    replay.require(before == after, "Controller exported method names changed")
    for name in before - {"step_target"}:
        replay.require(getattr(scalar.controller, name).code == getattr(candidate.controller, name).code,
                       "Undeclared controller method changed: " + name)
    graph = str(candidate.inlined_graph)
    replay.require("sd_target_tail_fileonly_r1::target" in graph,
                   "Target-tail operator missing")
    generated = {str(path): replay._sha(path.read_bytes())
                 for path in (directory / "cached.py", directory / "step.py", directory / "target.py")}
    return candidate, {"library_path": str(library), "library_sha256": pin,
        "library_source_sha256": replay._sha((_HERE / "target.cpp").read_bytes()),
        "compile_ms": compile_ms, "explicit_precompiled": bool(args.target_library),
        "model_path": str(model), "model_sha256": replay._sha(raw),
        "generated_sources": generated, "cache_proof": cache, "step_proof": step,
        "target_proof": target, "same_original_scalar_and_projection_operators": True,
        "undeclared_controller_methods_exact": True, "state_storage_disjoint": True,
        "all_named_initial_state_bits_exact": True,
        **dict.fromkeys(replay._FALSE_FLAGS, False)}


def paired_blocks(torch, scalar, candidate, frames):
    blocks = []
    aggregate = {name: {"raw_wall_ns": [], "raw_thread_cpu_ns": []}
                 for name in ("current_scalar", "target_candidate")}
    for index, candidate_first in enumerate((False, True, True, False)):
        models = (candidate, scalar) if candidate_first else (scalar, candidate)
        timing = replay.paired_timing(torch, models, frames)
        left, right = timing["current_scalar"], timing["full_fusion_candidate"]
        scalar_rows, target_rows = (right, left) if candidate_first else (left, right)
        differences = [a - b for a, b in zip(scalar_rows["raw_wall_ns"], target_rows["raw_wall_ns"])]
        orders = ["candidate_first" if bool(i % 2) != candidate_first else "scalar_first"
                  for i in range(len(frames))]
        order_stats = {}
        for name in ("scalar_first", "candidate_first"):
            values = [value for value, order in zip(differences, orders) if order == name]
            order_stats[name] = {"count": len(values), "mean_difference_ms": statistics.mean(values) / 1e6,
                                 "median_difference_ms": statistics.median(values) / 1e6}
        blocks.append({"block": index + 1, "candidate_first_at_index_zero": candidate_first,
            "current_scalar": scalar_rows, "target_candidate": target_rows,
            "raw_paired_wall_difference_ns": differences, "by_call_order": order_stats})
        for name, rows in (("current_scalar", scalar_rows), ("target_candidate", target_rows)):
            for key in aggregate[name]:
                aggregate[name][key].extend(rows[key])
    for rows in aggregate.values():
        rows["wall"] = replay.distribution(rows["raw_wall_ns"])
        rows["thread_cpu"] = replay.distribution(rows["raw_thread_cpu_ns"])
    return {"blocks": blocks, "aggregate": aggregate, "source_order": ["AB", "BA", "BA", "AB"],
        "A": "current_scalar", "B": "target_candidate", "reset_each_block": True,
        "per_cycle_call_order_alternates": True, "checks_outside_forward_timing": True,
        "whole_cycle_performance_measured": False, "latency_improvement_proven": False}


def run(args):
    replay.require(not args.compare_full_fusion, "Target-tail-only comparison required")
    replay.require(not any(getattr(args, key) for key in ("observation_library",
        "observation_library_sha256", "projection_library", "projection_library_sha256")),
        "Unselected full-fusion library arguments forbidden")
    replay.require(bool(args.target_library) == bool(args.target_library_sha256),
                   "Target library and SHA required together")
    output = replay._output_path(args.output)
    own_raw = Path(__file__).read_bytes()
    helper = replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    pins = source_pins(args)
    report, saved = replay.load_saved(args.report, args.report_sha256, args.records, args.records_sha256)
    audit = replay.strict_json(replay._read(args.raw_audit, args.raw_audit_sha256))
    replay.require(type(audit) is dict and audit.get("status") == "PASS_RAW_EVIDENCE"
        and type(audit.get("cycles_audited")) is int and audit["cycles_audited"] == 501,
        "Pinned independent full raw audit required")
    evidence = audit.get("input_file_sha256")
    replay.require(type(evidence) is dict and args.report_sha256 in evidence.values()
        and args.records_sha256 in evidence.values(), "Raw audit does not bind both originals")
    if args.target_library:
        replay._read(args.target_library, args.target_library_sha256)
    source = report["scalar_step_model_source"]
    result = {"schema": "singularitydog.saved-input-target-tail-profile.v1", "status": "FILE_ONLY_PLAN",
        "source_sha256": replay._sha(own_raw), "replay_helper_sha256": replay._sha(helper),
        "original_report_sha256": args.report_sha256, "original_records_sha256": args.records_sha256,
        "raw_audit_sha256": args.raw_audit_sha256, "candidate_source_sha256": pins,
        "candidate_source_manifest_sha256": args.candidate_source_manifest_sha256,
        "model_source": source, "cycles": len(saved), "device_or_model_loading_in_plan": False,
        "actual_controller_qualification": False, "baseline": "current_verified_scalar",
        "change": "step_target tail only; same scalar/projection op and actor/observation",
        **dict.fromkeys(replay._FALSE_FLAGS, False)}
    if args.execute_file_only:
        for key in ("scalar_manifest", "scalar_manifest_sha256", "baseline_manifest",
                    "baseline_manifest_sha256", "bundle"):
            replay.require(getattr(args, key), "Execution requires explicit " + key)
        replay.require(args.scalar_manifest_sha256 == source["manifest_sha256"]
            and args.baseline_manifest_sha256 == source["baseline_provenance"]["manifest_sha256"],
            "Loader manifest pins differ from original acquisition")
        from ..model_call_fastpath.scalar_loader import _prevalidate, load_file_only_verified
        _, manifest, _ = _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256,
                                     args.baseline_manifest_sha256)
        replay.require(manifest["model_sha256"] == source["model_sha256"]
            and manifest["library_sha256"] == source["library_sha256"],
            "Scalar artifact differs from original model")
        import torch
        torch.set_num_threads(1); torch.set_num_interop_threads(1)
        scalar, provenance = load_file_only_verified(args.scalar_manifest,
            expected_sha256=args.scalar_manifest_sha256, baseline_manifest=args.baseline_manifest,
            baseline_sha=args.baseline_manifest_sha256, bundle=args.bundle)
        replay.require(provenance == source, "Verified scalar provenance differs")
        frames = replay.tensor_frames(torch, saved)
        candidate, proof = target_candidate(torch, scalar, args, output)
        result.update(environment=replay.environment(torch), verified_loader_provenance=provenance,
                      target_candidate=proof)
        result["saved_validation"] = replay.compare_saved(torch, (scalar, candidate), frames)
        result["synthetic_validation"] = replay.synthetic_parity(torch, scalar, candidate)
        result["paired_forward_timing"] = paired_blocks(torch, scalar, candidate, frames)
        result["operator_profile"] = replay.operator_profile(torch, scalar, frames, args.profile_cycles)
        result["candidate_operator_profile"] = replay.operator_profile(torch, candidate, frames, args.profile_cycles)
        result["profiler_cycles"] = args.profile_cycles
        result["profiler_timing_is_separate"] = True
        for path, pin in {proof["library_path"]: proof["library_sha256"],
                          proof["model_path"]: proof["model_sha256"], **proof["generated_sources"]}.items():
            replay._read(path, pin)
        _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256, args.baseline_manifest_sha256)
        result["status"] = "PASS_FILE_ONLY_SCALAR_TARGET_TAIL_COMPARE"
    replay.load_saved(args.report, args.report_sha256, args.records, args.records_sha256)
    replay._read(args.raw_audit, args.raw_audit_sha256)
    if args.target_library:
        replay._read(args.target_library, args.target_library_sha256)
    replay.require(source_pins(args) == pins, "Target source pins changed")
    replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    replay.require(Path(__file__).read_bytes() == own_raw, "Target profiler source changed")
    with os.fdopen(os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "w") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    return result


def parser():
    p = replay.parser(); p.description = __doc__
    p.add_argument("--replay-helper-sha256", required=True)
    p.add_argument("--target-library"); p.add_argument("--target-library-sha256")
    return p


def main(argv=None):
    result = run(parser().parse_args(argv))
    print(json.dumps({"status": result["status"], "cycles": result["cycles"], "output_allowed": False}))


if __name__ == "__main__":
    main()
