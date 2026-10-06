"""Compare actor dispatch fusion against the CURRENT pinned scalar on saved data.

Default PLAN reads evidence and source pins only. --execute-file-only may load
CPU model/library artifacts or build the existing actor.cpp in a fresh private
directory. It never opens transport or grants live controller qualification.
The candidate retains the verified scalar controller and all named parameters.
"""
import copy
import io
import json
import os
from pathlib import Path
import statistics

from . import saved_input_profile as replay

_ACTOR_SOURCES = ("model_call_fastpath/actor.cpp", "model_call_fastpath/candidate.py")


def source_pins(args):
    pins = replay.candidate_sources(args.candidate_source_manifest,
                                    args.candidate_source_manifest_sha256)
    manifest = replay.strict_json(replay._read(args.candidate_source_manifest,
                                              args.candidate_source_manifest_sha256))
    for name in _ACTOR_SOURCES:
        key = "runtime/experiments/native_policy_overnight/" + name
        pin = manifest["files"].get(key)
        replay._read(replay.HERE / name, replay._digest(pin))
        pins[key] = pin
    return pins


def eager_actor(torch, scalar):
    """Re-expose the scripted Sequential's pinned parameters for actor indexing."""
    actor = scalar.actor
    replay.require(actor.original_name == "Sequential" and not list(actor.named_buffers()),
                   "Pinned stateless Sequential actor required")
    children = list(actor.named_children())
    replay.require([name for name, _ in children] == [str(index) for index in range(7)]
                   and [child.original_name for _, child in children]
                   == ["Linear", "ELU", "Linear", "ELU", "Linear", "ELU", "Linear"],
                   "Pinned actor layer order differs")
    for index in (1, 3, 5):
        child = children[index][1]
        replay.require(child.alpha == 1. and child.inplace is False, "Pinned ELU settings differ")
    result = torch.nn.Sequential(torch.nn.Linear(74, 128), torch.nn.ELU(),
        torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 64),
        torch.nn.ELU(), torch.nn.Linear(64, 12)).eval()
    original = dict(actor.named_parameters())
    replay.require(original.keys() == dict(result.named_parameters()).keys(),
                   "Pinned actor parameter names differ")
    result.load_state_dict({name: value.clone() for name, value in original.items()}, strict=True)
    for name, value in result.named_parameters():
        value.requires_grad_(original[name].requires_grad)
        replay._compare_bits(torch, original[name], value, "actor parameter:" + name)
    return result


def disjoint_state(scalar, candidate):
    pointers = []
    for model in (scalar, candidate):
        values = list(model.named_buffers()) + list(model.named_parameters())
        pointers.append({tensor.untyped_storage().data_ptr() for _, tensor in values
                         if tensor.numel()})
    replay.require(pointers[0].isdisjoint(pointers[1]), "Candidate shares scalar state storage")


def copy_controller(controller):
    """Copy state, then recreate declared views on the copied owning buffers.

    TorchScript deepcopy can copy cached Tensor attributes separately from their
    buffers. Rebinding only the pinned view table preserves controller methods
    and arithmetic; the original controller and its state are never changed.
    """
    from ..view_cache.generator import _VIEWS, verify_aliases
    present = [hasattr(controller, name) for name, _, _, _ in _VIEWS]
    replay.require(all(present) or not any(present), "Incomplete cached controller views")
    if all(present):
        verify_aliases(controller)
    cloned = copy.deepcopy(controller)
    if all(present):
        for name, expression, constructor_expression, _ in _VIEWS:
            value = eval(constructor_expression or expression,
                         {"__builtins__": {}}, {"self": cloned})
            setattr(cloned, name, value)
        verify_aliases(cloned)
    return cloned


def actor_candidate(torch, scalar, args, output):
    """Same scalar controller; actor dispatch is the only candidate change."""
    from ..contracts import OPTIONS
    from .candidate import FusedActorPolicy
    from .run import compile_actor
    replay.require(not hasattr(torch.ops.sd_actor_fileonly_r1, "forward"),
                   "Actor operator already loaded; fresh process required")
    actor = eager_actor(torch, scalar)
    directory = replay._output_path(output.with_name(output.stem + "-actor-artifacts"))
    directory.mkdir(mode=0o700)
    library = directory / "actor_fileonly.so"
    if args.actor_library:
        raw = replay._read(args.actor_library, args.actor_library_sha256)
        with library.open("xb") as stream:
            stream.write(raw)
        compile_ms = None
    else:
        compile_ms = compile_actor(torch, library, replay.HERE / _ACTOR_SOURCES[0])
    pin = replay._sha(library.read_bytes())
    torch.ops.load_library(str(library))
    replay._read(library, pin)
    policy = FusedActorPolicy(actor, **OPTIONS).eval()
    # Rebuilding a cached-view or full-fused core would confound actor timing.
    # Scripted controller copy retains the CURRENT verified scalar code/state.
    policy.controller = copy_controller(scalar.controller)
    candidate = torch.jit.script(policy)
    graph = str(candidate.inlined_graph)
    for operator in ("sd_actor_fileonly_r1::forward", "sd_step_fileonly_r1::step"):
        replay.require(operator in graph, "Missing actor candidate operator: " + operator)
    model = directory / "actor_fused_scalar_fileonly.pt"
    torch.jit.save(candidate, str(model))
    raw = model.read_bytes()
    reloaded = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    disjoint_state(scalar, reloaded)
    replay._state_bits(torch, scalar, reloaded)
    before_code = {name: getattr(scalar.controller, name).code
                   for name in scalar.controller._c._method_names()}
    after_code = {name: getattr(reloaded.controller, name).code
                  for name in reloaded.controller._c._method_names()}
    replay.require(before_code == after_code,
                   "Scalar controller code changed")
    return reloaded, {"library_path": str(library), "library_sha256": pin,
                      "library_source_sha256": replay._sha((replay.HERE / _ACTOR_SOURCES[0]).read_bytes()),
                      "compile_ms": compile_ms, "explicit_precompiled": bool(args.actor_library),
                      "model_path": str(model), "model_sha256": replay._sha(raw),
                      "scalar_controller_code_unchanged": True,
                      "scalar_controller_methods": sorted(before_code),
                      "actor_layer_order_and_elu_settings_exact": True,
                      "all_named_initial_state_bits_exact": True,
                      "state_storage_disjoint_from_scalar": True,
                      **dict.fromkeys(replay._FALSE_FLAGS, False)}


def paired_blocks(torch, scalar, candidate, frames):
    """Four independent resets; balanced opposite leading call orders."""
    blocks = []
    aggregate = {name: {"raw_wall_ns": [], "raw_thread_cpu_ns": []}
                 for name in ("current_scalar", "actor_candidate")}
    for block, candidate_first in enumerate((False, True, True, False)):
        models = (candidate, scalar) if candidate_first else (scalar, candidate)
        timing = replay.paired_timing(torch, models, frames)
        left, right = timing["current_scalar"], timing["full_fusion_candidate"]
        scalar_timing, candidate_timing = (right, left) if candidate_first else (left, right)
        differences = [a - b for a, b in zip(scalar_timing["raw_wall_ns"],
                                             candidate_timing["raw_wall_ns"])]
        orders = ["candidate_first" if bool(index % 2) != candidate_first else "scalar_first"
                  for index in range(len(frames))]
        order_stats = {}
        for name in ("scalar_first", "candidate_first"):
            values = [d for d, order in zip(differences, orders) if order == name]
            order_stats[name] = {"count": len(values), "mean_difference_ms": statistics.mean(values) / 1e6,
                                 "median_difference_ms": statistics.median(values) / 1e6}
        blocks.append({"block": block + 1, "candidate_first_at_index_zero": candidate_first,
                       "reset_both_before_warmup_and_saved_replay": True,
                       "current_scalar": scalar_timing, "actor_candidate": candidate_timing,
                       "raw_paired_wall_difference_ns": differences,
                       "by_call_order": order_stats})
        for name, value in (("current_scalar", scalar_timing), ("actor_candidate", candidate_timing)):
            for key in ("raw_wall_ns", "raw_thread_cpu_ns"):
                aggregate[name][key].extend(value[key])
    for value in aggregate.values():
        value["wall"] = replay.distribution(value["raw_wall_ns"])
        value["thread_cpu"] = replay.distribution(value["raw_thread_cpu_ns"])
    return {"blocks": blocks, "aggregate": aggregate,
            "source_order": ["AB", "BA", "BA", "AB"],
            "A": "current_scalar", "B": "actor_candidate",
            "per_cycle_call_order_alternates_in_each_block": True,
            "input_and_state_checks_outside_forward_timing": True,
            "comparison_isolated_from_device_and_whole_cycle": True,
            "latency_improvement_proven": False}


def run(args):
    replay.require(not args.compare_full_fusion, "Actor-only comparison required")
    replay.require(not any(getattr(args, key) for key in ("observation_library",
                   "observation_library_sha256", "projection_library", "projection_library_sha256")),
                   "Unselected full-fusion library arguments forbidden")
    replay.require(bool(args.actor_library) == bool(args.actor_library_sha256),
                   "Actor library and SHA required together")
    output = replay._output_path(args.output)
    raw_source = Path(__file__).read_bytes()
    helper_source = replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    pins = source_pins(args)
    report, saved = replay.load_saved(args.report, args.report_sha256,
                                      args.records, args.records_sha256)
    audit = replay.strict_json(replay._read(args.raw_audit, args.raw_audit_sha256))
    replay.require(type(audit) is dict and audit.get("status") == "PASS_RAW_EVIDENCE"
                   and type(audit.get("cycles_audited")) is int and audit["cycles_audited"] == 501,
                   "Pinned independent full raw audit required")
    evidence = audit.get("input_file_sha256")
    replay.require(type(evidence) is dict and args.report_sha256 in evidence.values()
                   and args.records_sha256 in evidence.values(), "Raw audit does not bind both originals")
    if args.actor_library:
        replay._read(args.actor_library, args.actor_library_sha256)
    source = report["scalar_step_model_source"]
    result = {"schema": "singularitydog.saved-input-actor-profile.v1", "status": "FILE_ONLY_PLAN",
              "source_sha256": replay._sha(raw_source), "replay_helper_sha256": replay._sha(helper_source),
              "original_report_sha256": args.report_sha256, "original_records_sha256": args.records_sha256,
              "raw_audit_sha256": args.raw_audit_sha256, "candidate_source_sha256": pins,
              "candidate_source_manifest_sha256": args.candidate_source_manifest_sha256,
              "model_source": source, "cycles": len(saved),
              "device_or_model_loading_in_plan": False, "actual_controller_qualification": False,
              "baseline": "current_verified_scalar_with_its_original_controller",
              **dict.fromkeys(replay._FALSE_FLAGS, False)}
    if args.execute_file_only:
        for key in ("scalar_manifest", "scalar_manifest_sha256", "baseline_manifest",
                    "baseline_manifest_sha256", "bundle"):
            replay.require(getattr(args, key), "Execution requires explicit " + key)
        replay.require(args.scalar_manifest_sha256 == source["manifest_sha256"]
                       and args.baseline_manifest_sha256 == source["baseline_provenance"]["manifest_sha256"],
                       "Loader manifest pins differ from original acquisition")
        from .scalar_loader import _prevalidate, load_file_only_verified
        path, manifest, _ = _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256,
                                         args.baseline_manifest_sha256)
        replay.require(manifest["model_sha256"] == source["model_sha256"]
                       and manifest["library_sha256"] == source["library_sha256"],
                       "Scalar artifact differs from original model")
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        scalar, provenance = load_file_only_verified(args.scalar_manifest,
            expected_sha256=args.scalar_manifest_sha256, baseline_manifest=args.baseline_manifest,
            baseline_sha=args.baseline_manifest_sha256, bundle=args.bundle)
        replay.require(provenance == source, "Verified scalar provenance differs")
        frames = replay.tensor_frames(torch, saved)
        candidate, proof = actor_candidate(torch, scalar, args, output)
        result["environment"] = replay.environment(torch)
        result["verified_loader_provenance"] = provenance
        result["actor_candidate"] = proof
        result["saved_validation"] = replay.compare_saved(torch, (scalar, candidate), frames)
        result["synthetic_validation"] = replay.synthetic_parity(torch, scalar, candidate)
        # All saved/synthetic/rejection parity precedes every performance sample.
        result["paired_forward_timing"] = paired_blocks(torch, scalar, candidate, frames)
        result["operator_profile"] = replay.operator_profile(torch, scalar, frames, args.profile_cycles)
        result["candidate_operator_profile"] = replay.operator_profile(torch, candidate, frames, args.profile_cycles)
        result["profiler_cycles"] = args.profile_cycles
        result["profiler_timing_is_separate"] = True
        replay._read(proof["library_path"], proof["library_sha256"])
        replay._read(proof["model_path"], proof["model_sha256"])
        _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256, args.baseline_manifest_sha256)
        result["status"] = "PASS_FILE_ONLY_SCALAR_ACTOR_COMPARE"
    replay.load_saved(args.report, args.report_sha256, args.records, args.records_sha256)
    replay._read(args.raw_audit, args.raw_audit_sha256)
    if args.actor_library:
        replay._read(args.actor_library, args.actor_library_sha256)
    replay.require(source_pins(args) == pins, "Actor source pins changed")
    replay._read(Path(replay.__file__).absolute(), args.replay_helper_sha256)
    replay.require(Path(__file__).read_bytes() == raw_source, "Actor profiler source changed")
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def parser():
    p = replay.parser()
    p.description = __doc__
    p.add_argument("--replay-helper-sha256", required=True)
    p.add_argument("--actor-library")
    p.add_argument("--actor-library-sha256")
    return p


def main(argv=None):
    result = run(parser().parse_args(argv))
    print(json.dumps({"status": result["status"], "cycles": result["cycles"], "output_allowed": False}))


if __name__ == "__main__":
    main()
