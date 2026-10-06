"""File-only three-way CURRENT scalar / frozen tail / immutable FK reuse.

No first-candidate files are changed. PLAN does not import Torch or build/load
libraries. Explicit execution verifies saved bits before balanced CPU timing.
"""
import copy
import io
import itertools
import json
import os
from pathlib import Path
import statistics
import time

from ..target_tail_fusion import saved_profile as first

replay = first.replay
_HERE = Path(__file__).resolve().parent
_SOURCES = ("__init__.py", "target.cpp", "generator.py", "saved_profile.py")
_OLD_FK = """  auto x = -.12 * (at::sin(b) + at::sin(b + c));
  auto z = -.12 * (at::cos(b) + at::cos(b + c));
  auto y = .064 * signs_expanded;
  return at::stack({x, at::cos(a) * y - at::sin(a) * z,
                      at::sin(a) * y + at::cos(a) * z}, 2) + origins_row;"""
_NEW_FK = """  // Reuse immutable ATen results only; keep every multiply/add/subtract order.
  auto bc = b + c;
  auto x = -.12 * (at::sin(b) + at::sin(bc));
  auto z = -.12 * (at::cos(b) + at::cos(bc));
  auto y = .064 * signs_expanded;
  auto cos_a = at::cos(a), sin_a = at::sin(a);
  return at::stack({x, cos_a * y - sin_a * z,
                      sin_a * y + cos_a * z}, 2) + origins_row;"""
_NAMES = ("current_scalar", "first_target_tail", "fk_cache_target_tail")


def cpp_delta():
    before = (first._HERE / "target.cpp").read_text()
    after = (_HERE / "target.cpp").read_text()
    replay.require(before.count(_OLD_FK) == after.count(_NEW_FK) == 1,
                   "Exact immutable FK delta required")
    restored = after.replace(_NEW_FK, _OLD_FK).replace(
        "sd_target_tail_fk_cache_fileonly_r1", "sd_target_tail_fileonly_r1")
    replay.require(restored == before, "Undeclared first-tail C++ change")
    return {"first_tail_source_sha256": replay._sha(before.encode()),
        "fk_cache_source_sha256": replay._sha(after.encode()), "inverse_bytes_exact": True,
        "only_fk_immutable_subexpressions_and_namespace_changed": True,
        "repeated_trig_calls_removed_per_forward": 6, "repeated_adds_removed_per_forward": 3,
        "performance_gain_measured": False}


def source_pins(args):
    pins = first.source_pins(args)
    manifest = replay.strict_json(replay._read(args.candidate_source_manifest,
                                              args.candidate_source_manifest_sha256))
    for name in _SOURCES:
        key = "runtime/experiments/native_policy_overnight/target_tail_fk_cache/" + name
        pin = replay._digest(manifest["files"].get(key)); replay._read(_HERE / name, pin)
        pins[key] = pin
    cpp_delta()
    return pins


def fk_candidate(torch, scalar, first_proof, args, output):
    from ..contracts import OPTIONS
    from ..lean_swing_deployment import DeployableSwingPolicy
    from ..view_cache.generator import verify_aliases
    from ..model_call_fastpath.run import compile_actor
    from .generator import generate_core
    replay.require(not hasattr(torch.ops.sd_target_tail_fk_cache_fileonly_r1, "target"),
                   "FK candidate operator already loaded; fresh process required")
    cpp_delta()
    directory = replay._output_path(output.with_name(output.stem + "-fk-cache-artifacts"))
    directory.mkdir(mode=0o700)
    library = directory / "target_tail_fk_cache.so"
    if args.fk_library:
        with library.open("xb") as stream:
            stream.write(replay._read(args.fk_library, args.fk_library_sha256))
        compile_ms = None
    else:
        compile_ms = compile_actor(torch, library, _HERE / "target.cpp")
    pin = replay._sha(library.read_bytes()); torch.ops.load_library(str(library)); replay._read(library, pin)
    sources = [path for path in first_proof["generated_sources"] if Path(path).name == "target.py"]
    replay.require(len(sources) == 1, "One pinned first-tail generated core required")
    core, proof = generate_core(sources[0], directory / "fk_cache.py",
                               first_proof["generated_sources"][sources[0]])
    policy = DeployableSwingPolicy(copy.deepcopy(scalar.actor), **OPTIONS).eval()
    policy.controller = core(1, "cpu", **OPTIONS)
    candidate = torch.jit.script(policy); verify_aliases(candidate.controller)
    model = directory / "fk_cache_tail_fileonly.pt"; torch.jit.save(candidate, str(model))
    raw = model.read_bytes(); candidate = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    verify_aliases(candidate.controller); first.disjoint_state(scalar, candidate)
    replay._state_bits(torch, scalar, candidate)
    names = set(scalar.controller._c._method_names())
    replay.require(names == set(candidate.controller._c._method_names()), "Serialized methods changed")
    for name in names - {"step_target"}:
        replay.require(getattr(scalar.controller, name).code == getattr(candidate.controller, name).code,
                       "Undeclared controller method changed: " + name)
    replay.require("sd_target_tail_fk_cache_fileonly_r1::target" in str(candidate.inlined_graph),
                   "FK cache op missing")
    return candidate, {"library_path": str(library), "library_sha256": pin,
        "library_source_sha256": replay._sha((_HERE / "target.cpp").read_bytes()),
        "model_path": str(model), "model_sha256": replay._sha(raw), "compile_ms": compile_ms,
        "explicit_precompiled": bool(args.fk_library), "cpp_delta": cpp_delta(),
        "generated_sources": {str(directory / "fk_cache.py"): replay._sha((directory / "fk_cache.py").read_bytes())},
        "generated_source_proof": proof, "same_scalar_projection_actor_observation": True,
        "undeclared_methods_exact": True, **dict.fromkeys(replay._FALSE_FLAGS, False)}


def independent_three(models):
    replay.require(len(models) == 3 and len({id(model) for model in models}) == 3,
                   "Three independent policy instances required")
    for one, two in itertools.combinations(models, 2):
        first.disjoint_state(one, two)


def compare_three(torch, models, frames):
    independent_three(models); ids = torch.tensor([0], dtype=torch.long); maxima = {}
    with torch.inference_mode():
        for model in models:
            model.reset(ids)
        for index, (inputs, saved) in enumerate(frames):
            before = tuple(value.clone() for value in inputs)
            for name, model in zip(_NAMES, models):
                result = model(*inputs); replay._input_bits(torch, before, inputs, "three-way input")
                for actual, expected in zip(replay._outputs(model, result), saved):
                    replay._compare_bits(torch, actual, expected, name + ":saved:" + str(index))
            for candidate in models[1:]:
                replay._state_bits(torch, models[0], candidate)
        rejected = []
        for name, inputs in replay.rejections(torch, frames[0][0]).items():
            reasons = []
            for model in models:
                model.reset(ids)
                try:
                    model(*inputs)
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(replay.reject_reason(error))
                else:
                    raise ValueError("Invalid input accepted: " + name)
            replay.require(len(set(reasons)) == 1, "Rejection differs: " + name)
            for candidate in models[1:]:
                replay._state_bits(torch, models[0], candidate)
            rejected.append({"case": name, "reason": reasons[0]})
        replay.require(len(rejected) == 26, "Rejection coverage differs")
        for model in models:
            model.reset(ids); replay.verify_aliases(model.controller)
        for candidate in models[1:]:
            replay._state_bits(torch, models[0], candidate)
    return {"saved_recurrent_calls": len(frames), "policies": list(_NAMES),
        "saved_output_actor_observation_bits_exact": True, "all_named_state_bits_each_call": True,
        "input_bits_preserved": True, "rejection_count": len(rejected), "rejections": rejected,
        "same_partial_state_after_rejections": True}


def balanced_three_timing(torch, models, frames):
    independent_three(models); ids = torch.tensor([0], dtype=torch.long)
    permutations = tuple(itertools.permutations(range(3)))
    aggregate = {name: {"raw_wall_ns": [], "raw_thread_cpu_ns": []} for name in _NAMES}; blocks = []
    with torch.inference_mode():
        for block, base in enumerate(permutations):
            rows = {name: {"raw_wall_ns": [], "raw_thread_cpu_ns": []} for name in _NAMES}; orders = []
            for model in models:
                model.reset(ids)
                for inputs, _ in frames[:10]:
                    before = tuple(value.clone() for value in inputs); model(*inputs)
                    replay._input_bits(torch, before, inputs, "three-way warmup input")
                model.reset(ids)
            for index, (inputs, saved) in enumerate(frames):
                offset = index % 3; order = base[offset:] + base[:offset]; orders.append(list(order))
                before = tuple(value.clone() for value in inputs); outputs = [None] * 3
                for slot in order:
                    wb, cb = time.perf_counter_ns(), time.thread_time_ns()
                    outputs[slot] = models[slot](*inputs)
                    ce, we = time.thread_time_ns(), time.perf_counter_ns()
                    rows[_NAMES[slot]]["raw_wall_ns"].append(we - wb)
                    rows[_NAMES[slot]]["raw_thread_cpu_ns"].append(ce - cb)
                    replay._input_bits(torch, before, inputs, "three-way timed input")
                for model, value in zip(models, outputs):
                    for actual, expected in zip(replay._outputs(model, value), saved):
                        replay._compare_bits(torch, actual, expected, "three-way timed saved output")
                for candidate in models[1:]:
                    replay._state_bits(torch, models[0], candidate)
            for name, row in rows.items():
                for key in row:
                    aggregate[name][key].extend(row[key])
                row["wall"] = replay.distribution(row["raw_wall_ns"])
                row["thread_cpu"] = replay.distribution(row["raw_thread_cpu_ns"])
            positions = {name: [sum(order[pos] == slot for order in orders) for pos in range(3)]
                         for slot, name in enumerate(_NAMES)}
            blocks.append({"block": block + 1, "leading_order": list(base), "raw_call_orders": orders,
                           "position_counts": positions, "timing": rows})
    for row in aggregate.values():
        row["wall"] = replay.distribution(row["raw_wall_ns"])
        row["thread_cpu"] = replay.distribution(row["raw_thread_cpu_ns"])
    return {"blocks": blocks, "aggregate": aggregate, "policies": list(_NAMES),
        "six_permutations_with_per_cycle_rotation": True, "reset_each_block": True,
        "checks_outside_forward_timing": True, "whole_cycle_performance_measured": False,
        "latency_improvement_proven": False}


def run(args):
    replay.require(not args.compare_full_fusion, "Only scalar/first-tail/FK-cache comparison supported")
    replay.require(not any(getattr(args, key) for key in ("observation_library",
        "observation_library_sha256", "projection_library", "projection_library_sha256")),
        "Unselected full-fusion library arguments forbidden")
    for prefix in ("target", "fk"):
        replay.require(bool(getattr(args, prefix + "_library")) == bool(getattr(args, prefix + "_library_sha256")),
                       "Library and SHA required together")
        if getattr(args, prefix + "_library"):
            replay._read(getattr(args, prefix + "_library"), getattr(args, prefix + "_library_sha256"))
    output = replay._output_path(args.output); own_raw = Path(__file__).read_bytes()
    helper = replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    pins = source_pins(args)
    report, saved = replay.load_saved(args.report, args.report_sha256, args.records, args.records_sha256)
    audit = replay.strict_json(replay._read(args.raw_audit, args.raw_audit_sha256))
    replay.require(type(audit) is dict, "Pinned full raw audit object required")
    evidence = audit.get("input_file_sha256")
    replay.require(audit.get("status") == "PASS_RAW_EVIDENCE" and type(audit.get("cycles_audited")) is int
        and audit["cycles_audited"] == 501 and type(evidence) is dict
        and args.report_sha256 in evidence.values() and args.records_sha256 in evidence.values(),
        "Pinned full raw audit binding both originals required")
    source = report["scalar_step_model_source"]
    result = {"schema": "singularitydog.saved-input-target-fk-cache-profile.v1", "status": "FILE_ONLY_PLAN",
        "source_sha256": replay._sha(own_raw), "replay_helper_sha256": replay._sha(helper),
        "original_report_sha256": args.report_sha256, "original_records_sha256": args.records_sha256,
        "raw_audit_sha256": args.raw_audit_sha256, "candidate_source_sha256": pins,
        "candidate_source_manifest_sha256": args.candidate_source_manifest_sha256,
        "model_source": source, "cycles": len(saved), "device_or_model_loading_in_plan": False,
        "cpp_delta": cpp_delta(), "actual_controller_qualification": False,
        **dict.fromkeys(replay._FALSE_FLAGS, False)}
    if args.execute_file_only:
        for key in ("scalar_manifest", "scalar_manifest_sha256", "baseline_manifest",
                    "baseline_manifest_sha256", "bundle"):
            replay.require(getattr(args, key), "Execution requires explicit " + key)
        replay.require(args.scalar_manifest_sha256 == source["manifest_sha256"]
            and args.baseline_manifest_sha256 == source["baseline_provenance"]["manifest_sha256"],
            "Loader manifest pins differ from acquisition")
        from ..model_call_fastpath.scalar_loader import _prevalidate, load_file_only_verified
        _, manifest, _ = _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256, args.baseline_manifest_sha256)
        replay.require(manifest["model_sha256"] == source["model_sha256"]
            and manifest["library_sha256"] == source["library_sha256"], "Scalar artifact differs")
        import torch
        torch.set_num_threads(1); torch.set_num_interop_threads(1)
        scalar, provenance = load_file_only_verified(args.scalar_manifest,
            expected_sha256=args.scalar_manifest_sha256, baseline_manifest=args.baseline_manifest,
            baseline_sha=args.baseline_manifest_sha256, bundle=args.bundle)
        replay.require(provenance == source, "Scalar provenance differs")
        tail, tail_proof = first.target_candidate(torch, scalar, args, output)
        fk, fk_proof = fk_candidate(torch, scalar, tail_proof, args, output); models = (scalar, tail, fk)
        frames = replay.tensor_frames(torch, saved)
        result.update(environment=replay.environment(torch), first_target_tail=tail_proof,
                      fk_cache_target_tail=fk_proof, verified_loader_provenance=provenance)
        result["saved_validation"] = compare_three(torch, models, frames)
        result["synthetic_validation"] = {name: replay.synthetic_parity(torch, scalar, candidate)
            for name, candidate in zip(_NAMES[1:], models[1:])}
        result["paired_forward_timing"] = balanced_three_timing(torch, models, frames)
        result["operator_profiles"] = {name: replay.operator_profile(torch, model, frames, args.profile_cycles)
                                       for name, model in zip(_NAMES, models)}
        result["profiler_cycles"] = args.profile_cycles; result["profiler_timing_is_separate"] = True
        for proof in (tail_proof, fk_proof):
            for path, pin in {proof["library_path"]: proof["library_sha256"],
                    proof["model_path"]: proof["model_sha256"], **proof["generated_sources"]}.items():
                replay._read(path, pin)
        _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256, args.baseline_manifest_sha256)
        result["status"] = "PASS_FILE_ONLY_SCALAR_TAIL_FK_CACHE_COMPARE"
    replay.load_saved(args.report, args.report_sha256, args.records, args.records_sha256)
    replay._read(args.raw_audit, args.raw_audit_sha256)
    replay.require(source_pins(args) == pins, "Candidate source pins changed")
    replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    replay.require(Path(__file__).read_bytes() == own_raw, "FK runner source changed")
    for prefix in ("target", "fk"):
        if getattr(args, prefix + "_library"):
            replay._read(getattr(args, prefix + "_library"), getattr(args, prefix + "_library_sha256"))
    with os.fdopen(os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "w") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    return result


def parser():
    p = first.parser(); p.description = __doc__
    p.add_argument("--fk-library"); p.add_argument("--fk-library-sha256")
    return p


def main(argv=None):
    result = run(parser().parse_args(argv))
    print(json.dumps({"status": result["status"], "cycles": result["cycles"], "output_allowed": False}))


if __name__ == "__main__":
    main()
