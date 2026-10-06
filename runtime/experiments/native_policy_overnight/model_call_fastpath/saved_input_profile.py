"""Profile a pinned scalar policy on saved 501-cycle inputs, without device I/O.

PLAN is the default. Execution needs the established verified scalar loader in
a fresh process. No model is built, registered or loaded by PLAN. CPU operator
profiling is a separate replay from timing; it cannot qualify a live controller.
"""
import argparse
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import stat
import statistics
import struct
import time

from ..contracts import HERE, INPUT_KEYS, environment, member, require, strict_json
from ..verification import compare_state, compare_tensor, frame, rejections, reject_reason
from ..view_cache.generator import verify_aliases

_WIDTHS = (3, 3, 3, 12, 12, 12)
_SAVED_OUTPUTS = (("q_target_rad_diagnostic_only", 12),
                  ("actor_residual12", 12), ("observation74", 74))
_FALSE_FLAGS = ("hardware_opened", "output_allowed", "approved_for_runtime",
                "live_50hz_verified")
_MAX_ORIGINAL_BYTES = 64 * 1024 * 1024
_CANDIDATE_SOURCES = (
    "contracts.py", "verification.py", "lean_swing_core.py", "lean_swing_deployment.py",
    "view_cache/generator.py", "model_call_fastpath/step_generator.py",
    "model_call_fastpath/step_scalar.cpp", "model_call_fastpath/run.py",
    "observation_fusion/observation.cpp", "observation_fusion/generator.py",
    "projection_zero_cache/projection.cpp", "projection_zero_cache/torch_projection.cpp",
    "projection_zero_cache/generator.py", "full_recurrent_fusion/generator.py")


def distribution(values):
    ordered = sorted(values)
    require(bool(ordered), "Timing samples required")
    count = len(ordered)
    return {"count": count, "median_ms": statistics.median(ordered) / 1e6,
            "p95_ms": ordered[math.ceil(.95 * count) - 1] / 1e6,
            "p99_ms": ordered[math.ceil(.99 * count) - 1] / 1e6,
            "max_ms": ordered[-1] / 1e6}


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _digest(value):
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value),
            "Explicit lowercase SHA256 required")
    return value


def _read(path, digest):
    _digest(digest)
    path = Path(path)
    require(path.is_absolute(), "Absolute input path required")
    require(not any(p.is_symlink() for p in (path, *path.parents)),
            "Symlink input path forbidden")
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        require(stat.S_ISREG(info.st_mode) and info.st_size <= _MAX_ORIGINAL_BYTES,
                "Bounded regular input required")
        raw = stream.read(_MAX_ORIGINAL_BYTES + 1)
    require(len(raw) <= _MAX_ORIGINAL_BYTES and _sha(raw) == digest,
            "Input size or SHA differs")
    return raw


def _float32_row(value, width, label):
    require(type(value) is list and len(value) == width,
            "Invalid saved vector: " + label)
    for item in value:
        require(type(item) in (int, float) and math.isfinite(item),
                "Nonfinite or nonnumeric saved vector: " + label)
        try:
            narrowed = struct.unpack("=f", struct.pack("=f", item))[0]
        except (OverflowError, struct.error):
            raise ValueError("Saved vector overflows float32: " + label) from None
        require(math.isfinite(narrowed), "Saved vector overflows float32: " + label)
    return value


def load_saved(report_path, report_sha, records_path, records_sha):
    """Keep the original six vectors; never substitute nominal/model inputs."""
    report = strict_json(_read(report_path, report_sha))
    records = strict_json(_read(records_path, records_sha))
    require(type(report) is dict and report.get("status") == "COMPLETE_DIAGNOSTIC",
            "Complete diagnostic report required")
    require(type(report.get("cycles_completed")) is int and
            report["cycles_completed"] == 501 and not report.get("errors"),
            "Require 501 complete cycles without errors")
    for key in ("motor_enable_sent", "learned_targets_sent", "approved_for_runtime",
                "full_controller_50Hz_verified"):
        require(report.get(key) is False, "Saved diagnostic scope differs: " + key)
    source = report.get("scalar_step_model_source")
    require(type(source) is dict and source.get("schema") ==
            "native-step-scalar-file-only-loader-v1" and
            all(source.get(key) is False for key in _FALSE_FLAGS),
            "Pinned scalar file-only provenance required")
    for key in ("manifest_sha256", "model_sha256", "library_sha256"):
        _digest(source.get(key))
    baseline = source.get("baseline_provenance")
    require(type(baseline) is dict, "Scalar baseline provenance required")
    _digest(baseline.get("manifest_sha256"))
    require(type(records) is list and len(records) == 501,
            "Require all original 501 records")
    frames = []
    for index, row in enumerate(records):
        require(type(row) is dict and type(row.get("cycle")) is int and
                row["cycle"] == index + 1, "Saved cycles must be ordered 1..501")
        observed = row.get("observed")
        require(type(observed) is dict and observed.get("status") ==
                "TICK_OBSERVED_NO_OUTPUT" and observed.get("output_allowed") is False,
                "Complete no-output observation required")
        require(type(observed.get("tick_index")) is int and
                observed["tick_index"] == index, "Saved tick order differs")
        inputs = observed.get("inputs")
        require(type(inputs) is dict and set(inputs) == set(INPUT_KEYS),
                "Exactly six original input vectors required")
        values = tuple(_float32_row(inputs[key], width, key)
                       for key, width in zip(INPUT_KEYS, _WIDTHS))
        outputs = tuple(_float32_row(observed[key], width, key)
                        for key, width in _SAVED_OUTPUTS)
        frames.append((values, outputs))
    return report, frames


def tensor_frames(torch, frames):
    return [(tuple(torch.tensor([row], dtype=torch.float32) for row in values),
             tuple(torch.tensor([row], dtype=torch.float32) for row in outputs))
            for values, outputs in frames]


def _outputs(model, target):
    return target, model.last_actor_output, model.last_observation


def _compare_bits(torch, one, two, label):
    compare_tensor(torch, one, two, label, {}, exact=True)
    require(torch.equal(one.contiguous().reshape(-1).view(torch.uint8),
                        two.contiguous().reshape(-1).view(torch.uint8)),
            "Tensor bits differ: " + label)


def _state_bits(torch, one, two):
    for accessor in ("named_buffers", "named_parameters"):
        left, right = dict(getattr(one, accessor)()), dict(getattr(two, accessor)())
        require(left.keys() == right.keys(), "State names differ")
        for name, value in left.items():
            _compare_bits(torch, value, right[name], accessor + ":" + name)


def _independent_models(models):
    require(len(models) == 2 and models[0] is not models[1],
            "Two independent policy instances required")


def _input_bits(torch, before, inputs, label):
    for key, original, actual in zip(INPUT_KEYS, before, inputs):
        _compare_bits(torch, original, actual, label + ":" + key)


def compare_saved(torch, models, frames):
    """Two independent resets; all state and original saved outputs each call."""
    _independent_models(models)
    ids = torch.tensor([0], dtype=torch.long)
    maxima = {}
    with torch.inference_mode():
        for model in models:
            model.reset(ids)
        compare_state(torch, *models, maxima, exact=True)
        _state_bits(torch, *models)
        for index, (inputs, saved_outputs) in enumerate(frames):
            original = tuple(x.clone() for x in inputs)
            targets = []
            for model in models:
                targets.append(model(*inputs))
                _input_bits(torch, original, inputs, "Input mutated at cycle " + str(index + 1))
            for model, target in zip(models, targets):
                for (key, _), actual, saved in zip(_SAVED_OUTPUTS,
                        _outputs(model, target), saved_outputs):
                    compare_tensor(torch, actual, saved, key, maxima, exact=True)
                    _compare_bits(torch, actual, saved, key)
            compare_state(torch, *models, maxima, exact=True)
            _state_bits(torch, *models)
        rejected = []
        for name, inputs in rejections(torch, frames[0][0]).items():
            reasons = []
            for model in models:
                model.reset(ids)
                try:
                    model(*inputs)
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
                else:
                    raise ValueError("Invalid input accepted: " + name)
            require(reasons[0] == reasons[1], "Rejection reason differs: " + name)
            compare_state(torch, *models, maxima, exact=True)
            _state_bits(torch, *models)
            rejected.append({"case": name, "reason": reasons[0]})
        require(len(rejected) == 26, "Rejection coverage differs")
        for model in models:
            model.reset(ids)
            verify_aliases(model.controller)
        compare_state(torch, *models, maxima, exact=True)
    return {"saved_recurrent_calls": len(frames), "all_named_state_each_call": True,
            "saved_observation_actor_target_exact": True, "floating_bits_exact": True,
            "input_mutation": False,
            "max_errors": maxima, "rejection_count": len(rejected),
            "rejections": rejected}


def timed_replay(torch, model, frames):
    """Clock only forward; comparison/reset/profiling stays outside timing."""
    wall, cpu = [], []
    ids = torch.tensor([0], dtype=torch.long)
    with torch.inference_mode():
        model.reset(ids)
        for inputs, _ in frames[:10]:
            original = tuple(x.clone() for x in inputs)
            model(*inputs)
            _input_bits(torch, original, inputs, "Warmup input mutated")
        model.reset(ids)
        for inputs, saved_outputs in frames:
            original = tuple(x.clone() for x in inputs)
            begin_wall, begin_cpu = time.perf_counter_ns(), time.thread_time_ns()
            target = model(*inputs)
            end_cpu, end_wall = time.thread_time_ns(), time.perf_counter_ns()
            cpu.append(end_cpu - begin_cpu)
            wall.append(end_wall - begin_wall)
            _input_bits(torch, original, inputs, "Timed input mutated")
            for actual, saved in zip(_outputs(model, target), saved_outputs):
                compare_tensor(torch, actual, saved, "timed saved output", {}, exact=True)
                _compare_bits(torch, actual, saved, "timed saved output")
    return {"wall": distribution(wall), "thread_cpu": distribution(cpu),
            "raw_wall_ns": wall, "raw_thread_cpu_ns": cpu,
            "excludes": ["tensor construction", "validation", "state comparison",
                         "device acquisition", "motor output", "profiler"]}


def operator_profile(torch, model, frames, count):
    require(type(count) is int and 1 <= count <= len(frames), "Invalid profiler count")
    ids = torch.tensor([0], dtype=torch.long)
    with torch.inference_mode():
        model.reset(ids)
        with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU],
                                    record_shapes=True) as profile:
            for inputs, _ in frames[:count]:
                with torch.profiler.record_function("pinned_scalar_policy_forward"):
                    model(*inputs)
        # Profiler observes the original forward, including its original checks.
        # Its overhead and CPU totals are separate from uninstrumented replay.
    rows = [{"operator": item.key, "calls": item.count,
             "self_cpu_time_us": float(item.self_cpu_time_total),
             "inclusive_cpu_time_us": float(item.cpu_time_total),
             "input_shapes": item.input_shapes}
            for item in profile.key_averages(group_by_input_shape=True)]
    return sorted(rows, key=lambda item: item["self_cpu_time_us"], reverse=True)


def candidate_sources(path, digest):
    """Require every reused candidate source in an explicitly pinned kit map."""
    manifest = strict_json(_read(path, digest))
    require(type(manifest) is dict and manifest.get("schema") == "private-overnight-kit-v1"
            and type(manifest.get("files")) is dict, "Frozen source inventory required")
    root = HERE
    pins = {}
    for name in _CANDIDATE_SOURCES:
        key = "runtime/experiments/native_policy_overnight/" + name
        pin = manifest["files"].get(key)
        _read(root / name, _digest(pin))
        pins[key] = pin
    return pins


def build_full_fusion(torch, scalar, args, output):
    """Reuse audited generators and C++ sources; never replace runtime files."""
    from ..contracts import OPTIONS
    from ..lean_swing_deployment import DeployableSwingPolicy
    from ..view_cache import generate_core
    from ..full_recurrent_fusion.generator import generate_core as generate_full_core
    from .run import compile_actor
    directory = _output_path(output.with_name(output.stem + "-candidate-artifacts"))
    directory.mkdir(mode=0o700)
    root = HERE
    builds = {}
    for name, operator, source in (
            ("observation", "sd_observation_fileonly_r1", "observation_fusion/observation.cpp"),
            ("projection", "sd_projection_zero_cache_fileonly_r1", "projection_zero_cache/torch_projection.cpp")):
        require(not hasattr(getattr(torch.ops, operator), "observe" if name == "observation" else "project"),
                "Candidate operator already loaded; fresh process required")
        supplied, pin = getattr(args, name + "_library"), getattr(args, name + "_library_sha256")
        require(bool(supplied) == bool(pin), "Precompiled library and SHA required together")
        library = directory / (name + "_fileonly.so")
        if supplied:
            raw = _read(supplied, pin)
            with library.open("xb") as stream:
                stream.write(raw)
            elapsed = None
        else:
            elapsed = compile_actor(torch, library, root / source)
        library_sha = _sha(library.read_bytes())
        torch.ops.load_library(str(library))
        _read(library, library_sha)
        builds[name] = {"path": str(library), "sha256": library_sha,
                        "source_sha256": _sha((root / source).read_bytes()),
                        "compile_ms": elapsed, "explicit_precompiled": bool(supplied)}
    _, cache_proof = generate_core(directory / "cached_view_core.py")
    core, fusion_proof = generate_full_core(directory / "cached_view_core.py",
        directory / "full_fused_core.py", cache_proof["generated_core_sha256"])
    candidate = DeployableSwingPolicy(copy.deepcopy(scalar.actor), **OPTIONS).eval()
    candidate.controller = core(1, "cpu", **OPTIONS)
    candidate = torch.jit.script(candidate)
    graph = str(candidate.inlined_graph)
    for operator in ("sd_observation_fileonly_r1::observe", "sd_step_fileonly_r1::step",
                     "sd_projection_zero_cache_fileonly_r1::project"):
        require(operator in graph, "Missing candidate operator")
    model = directory / "full_fused_fileonly.pt"
    torch.jit.save(candidate, str(model))
    raw = model.read_bytes()
    reloaded = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    return reloaded, {"libraries": builds, "model_sha256": _sha(raw),
                      "model_path": str(model), "cache_proof": cache_proof,
                      "fusion_proof": fusion_proof, **dict.fromkeys(_FALSE_FLAGS, False)}


def synthetic_parity(torch, one, two):
    _independent_models((one, two))
    ids = torch.tensor([0], dtype=torch.long)
    nominal = one.controller.nominal.float().reshape(1, 12)
    with torch.inference_mode():
        for index in range(240):
            if index in (0, 125):
                one.reset(ids)
                two.reset(ids)
            inputs = frame(torch, nominal, index)
            before = tuple(x.clone() for x in inputs)
            targets = []
            for model in (one, two):
                targets.append(model(*inputs))
                _input_bits(torch, before, inputs, "Synthetic input mutated")
            for actual, candidate in zip(_outputs(one, targets[0]), _outputs(two, targets[1])):
                _compare_bits(torch, actual, candidate, "synthetic output")
            _state_bits(torch, one, two)
    return {"frames": 240, "reset_before": [0, 125], "all_named_state_bits_exact": True}


def paired_timing(torch, models, frames):
    """Alternate call order while both models follow the same saved history."""
    _independent_models(models)
    ids = torch.tensor([0], dtype=torch.long)
    wall, cpu = [[], []], [[], []]
    with torch.inference_mode():
        for model in models:
            model.reset(ids)
            for inputs, _ in frames[:10]:
                original = tuple(x.clone() for x in inputs)
                model(*inputs)
                _input_bits(torch, original, inputs, "Paired warmup input mutated")
            model.reset(ids)
        for index, (inputs, saved) in enumerate(frames):
            original = tuple(x.clone() for x in inputs)
            outputs = [None, None]
            for offset in range(2):
                slot = (index + offset) % 2
                begin_wall, begin_cpu = time.perf_counter_ns(), time.thread_time_ns()
                outputs[slot] = models[slot](*inputs)
                end_cpu, end_wall = time.thread_time_ns(), time.perf_counter_ns()
                cpu[slot].append(end_cpu - begin_cpu)
                wall[slot].append(end_wall - begin_wall)
                _input_bits(torch, original, inputs, "Paired input mutated")
            for model, target in zip(models, outputs):
                for actual, reference in zip(_outputs(model, target), saved):
                    _compare_bits(torch, actual, reference, "paired saved output")
            _state_bits(torch, *models)
    return {name: {"wall": distribution(wall[index]), "thread_cpu": distribution(cpu[index]),
                   "raw_wall_ns": wall[index], "raw_thread_cpu_ns": cpu[index]}
            for index, name in enumerate(("current_scalar", "full_fusion_candidate"))}


def _output_path(value):
    path = Path(value)
    require(path.is_absolute() and path.parent.is_dir() and not path.exists(),
            "Fresh absolute output required")
    require(not any(p.is_symlink() for p in (path, *path.parents)),
            "Symlink output forbidden")
    require(not any((p / ".git").exists() for p in path.parents),
            "Output must stay outside Git")
    return path


def run(args):
    output = _output_path(args.output)
    source_raw = Path(__file__).read_bytes()
    report, saved = load_saved(args.report, args.report_sha256,
                               args.records, args.records_sha256)
    audit = strict_json(_read(args.raw_audit, args.raw_audit_sha256))
    require(type(audit) is dict and audit.get("status") == "PASS_RAW_EVIDENCE" and
            type(audit.get("cycles_audited")) is int and audit["cycles_audited"] == 501,
            "Pinned independent full raw audit required")
    pins = audit.get("input_file_sha256")
    require(type(pins) is dict and args.report_sha256 in pins.values() and
            args.records_sha256 in pins.values(), "Raw audit does not bind both original files")
    source = report["scalar_step_model_source"]
    if args.compare_full_fusion:
        require(args.candidate_source_manifest and args.candidate_source_manifest_sha256,
                "Candidate requires an explicit frozen source inventory")
        candidate_pins = candidate_sources(args.candidate_source_manifest,
                                           args.candidate_source_manifest_sha256)
    result = {"schema": "singularitydog.saved-input-scalar-profile.v1",
              "status": "FILE_ONLY_PLAN", "source_sha256": _sha(source_raw),
              "original_report_sha256": args.report_sha256,
              "original_records_sha256": args.records_sha256,
              "raw_audit_sha256": args.raw_audit_sha256,
              "model_source": source, "cycles": len(saved),
              **dict.fromkeys(_FALSE_FLAGS, False),
              "device_or_model_loading_in_plan": False,
              "actual_controller_qualification": False}
    if args.compare_full_fusion:
        result["candidate_source_sha256"] = candidate_pins
        result["candidate_source_manifest_sha256"] = args.candidate_source_manifest_sha256
        result["candidate_build_and_execution_in_plan"] = False
    if args.execute_file_only:
        for key in ("scalar_manifest", "scalar_manifest_sha256", "baseline_manifest",
                    "baseline_manifest_sha256", "bundle"):
            require(getattr(args, key), "Execution requires explicit " + key)
        require(args.scalar_manifest_sha256 == source["manifest_sha256"] and
                args.baseline_manifest_sha256 == source["baseline_provenance"]["manifest_sha256"],
                "Loader manifest pins differ from original acquisition")
        from .scalar_loader import _prevalidate, load_file_only_verified
        path, manifest, _ = _prevalidate(args.scalar_manifest,
            args.scalar_manifest_sha256, args.baseline_manifest_sha256)
        require(manifest["model_sha256"] == source["model_sha256"] and
                manifest["library_sha256"] == source["library_sha256"],
                "Scalar artifact differs from original model")
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        model, provenance = load_file_only_verified(args.scalar_manifest,
            expected_sha256=args.scalar_manifest_sha256,
            baseline_manifest=args.baseline_manifest,
            baseline_sha=args.baseline_manifest_sha256, bundle=args.bundle)
        require(provenance == source,
                "Verified loader provenance differs from original diagnostic")
        # The first verified load registers the exact op. A second copy of the
        # SAME pinned model makes state parity independent without registering
        # another implementation under the same global operator name.
        model_raw = _read(member(path.parent, manifest["model_file"]), manifest["model_sha256"])
        twin = torch.jit.load(io.BytesIO(model_raw), map_location="cpu").eval()
        frames = tensor_frames(torch, saved)
        result["environment"] = environment(torch)
        result["verified_loader_provenance"] = provenance
        result["validation"] = compare_saved(torch, (model, twin), frames)
        result["forward_timing"] = timed_replay(torch, model, frames)
        result["operator_profile"] = operator_profile(torch, model, frames, args.profile_cycles)
        result["profiler_cycles"] = args.profile_cycles
        result["profiler_timing_is_separate"] = True
        result["status"] = "PASS_FILE_ONLY_SAVED_INPUT_REPLAY"
        if args.compare_full_fusion:
            candidate, proof = build_full_fusion(torch, model, args, output)
            result["full_fusion_candidate"] = proof
            result["candidate_saved_validation"] = compare_saved(torch, (model, candidate), frames)
            result["candidate_synthetic_validation"] = synthetic_parity(torch, model, candidate)
            result["paired_forward_timing"] = paired_timing(torch, (model, candidate), frames)
            result["candidate_operator_profile"] = operator_profile(torch, candidate, frames,
                                                                       args.profile_cycles)
            candidate_sources(args.candidate_source_manifest, args.candidate_source_manifest_sha256)
            for library in proof["libraries"].values():
                _read(library["path"], library["sha256"])
            _read(proof["model_path"], proof["model_sha256"])
            result["status"] = "PASS_FILE_ONLY_SCALAR_FULL_FUSION_COMPARE"
        _prevalidate(args.scalar_manifest, args.scalar_manifest_sha256,
                     args.baseline_manifest_sha256)
    _read(args.report, args.report_sha256)
    _read(args.records, args.records_sha256)
    _read(args.raw_audit, args.raw_audit_sha256)
    require(Path(__file__).read_bytes() == source_raw, "Profiler source changed during run")
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def parser():
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--report", required=True)
    p.add_argument("--report-sha256", required=True)
    p.add_argument("--records", required=True)
    p.add_argument("--records-sha256", required=True)
    p.add_argument("--raw-audit", required=True)
    p.add_argument("--raw-audit-sha256", required=True)
    p.add_argument("--scalar-manifest")
    p.add_argument("--scalar-manifest-sha256")
    p.add_argument("--baseline-manifest")
    p.add_argument("--baseline-manifest-sha256")
    p.add_argument("--bundle")
    p.add_argument("--output", required=True)
    p.add_argument("--profile-cycles", type=int, default=50, choices=range(1, 502))
    p.add_argument("--compare-full-fusion", action="store_true")
    p.add_argument("--candidate-source-manifest")
    p.add_argument("--candidate-source-manifest-sha256")
    p.add_argument("--observation-library")
    p.add_argument("--observation-library-sha256")
    p.add_argument("--projection-library")
    p.add_argument("--projection-library-sha256")
    p.add_argument("--execute-file-only", action="store_true")
    return p


def main(argv=None):
    result = run(parser().parse_args(argv))
    print(json.dumps({"status": result["status"], "cycles": result["cycles"],
                      "output_allowed": False}))


if __name__ == "__main__":
    main()
