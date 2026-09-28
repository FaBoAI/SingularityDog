"""Pinned scalar-step TorchScript loader for diagnostic-only file replay.

This loader has no device access or output permission. The scalar C++ operator
uses the same registered name as the prior ATen step experiment, so each must
run in a fresh process; loading both libraries together is forbidden.
"""
import io
import json
import math
from pathlib import Path
import re

from ..contracts import environment, member, pinned, require, sha, strict_json
from ..verification import compare_state
from ..view_cache.generator import verify_aliases
from ..view_cache.loader import _rejection_reasons, load_file_only_verified as load_cached


_HERE = Path(__file__).resolve().parent
_SOURCE_PATHS = {
    "model_call_fastpath/step.cpp": _HERE / "step.cpp",
    "model_call_fastpath/step_scalar.cpp": _HERE / "step_scalar.cpp",
    "model_call_fastpath/step_generator.py": _HERE / "step_generator.py",
    "model_call_fastpath/run_step.py": _HERE / "run_step.py",
    "model_call_fastpath/short_replay.py": _HERE / "short_replay.py",
    "model_call_fastpath/scalar_replay.py": _HERE / "scalar_replay.py",
    "model_call_fastpath/compile_scalar.py": _HERE / "compile_scalar.py",
    "model_call_fastpath/run.py": _HERE / "run.py",
    "model_call_fastpath/scalar_loader.py": Path(__file__),
    "view_cache/loader.py": _HERE.parent / "view_cache" / "loader.py",
}
_FALSE_FLAGS = ("hardware_opened", "output_allowed", "approved_for_runtime",
                "live_50hz_verified")
_REPORTS = ("parity", "synthetic", "timing")


def _digest(value):
    return type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _zero_maxima(value):
    return (type(value) is dict and "target" in value and
            all(type(name) is str and type(error) in (int, float) and
                math.isfinite(error) and error == 0.0
                for name, error in value.items()))


def _validate_report(phase, report, manifest):
    require(type(report) is dict and report.get("schema") ==
            "native-step-scalar-short-replay-r1" and
            report.get("status") == "PASS_FILE_ONLY" and report.get("phase") == phase,
            "Scalar report status/phase differs")
    require(all(report.get(flag) is False for flag in _FALSE_FLAGS),
            "Scalar report scope differs")
    common = {
        "environment": manifest["environment"],
        "scalar_step_source_sha256": manifest["source_hashes"]["model_call_fastpath/step_scalar.cpp"],
        "scalar_replay_source_sha256": manifest["source_hashes"]["model_call_fastpath/scalar_replay.py"],
        "candidate_model_sha256": manifest["model_sha256"],
        "candidate_library_sha256": manifest["library_sha256"],
        "baseline_manifest_sha256": manifest["baseline_manifest_sha256"],
        "records_sha256": manifest["records_sha256"],
        "operator": "sd_step_fileonly_r1::step",
        "implementation": "scalar_cpp_file_only",
        "same_model_graph_as_aten_step_candidate": True,
        "fresh_process_required": True,
    }
    require(all(type(report.get(key)) is type(value) and report[key] == value
                for key, value in common.items()), "Scalar report provenance differs")
    if phase == "parity":
        validation = report.get("validation")
        require(type(validation) is dict and
                validation.get("saved_recurrent_calls") == 500 and
                validation.get("all_named_state_each_call") is True and
                validation.get("observation_actor_target_exact") is True and
                validation.get("input_mutation") is False and
                validation.get("rejection_count") == 26 and
                _zero_maxima(validation.get("max_errors")),
                "Scalar saved-input parity incomplete")
        rejected = validation.get("rejections")
        require(type(rejected) is list and len(rejected) == 26 and
                all(type(row) is dict and set(row) == {"case", "reason"} and
                    type(row["case"]) is str and type(row["reason"]) is str
                    for row in rejected) and
                {row["case"]: row["reason"] for row in rejected} == _rejection_reasons(),
                "Scalar rejection parity incomplete")
    elif phase == "synthetic":
        validation = report.get("validation")
        require(type(validation) is dict and
                validation.get("synthetic_frames") == 240 and
                validation.get("reset_before") == [0, 125] and
                validation.get("all_named_state_each_frame") is True and
                _zero_maxima(validation.get("max_errors")),
                "Scalar synthetic parity incomplete")
    else:
        require(report.get("view_manifest_sha256") == manifest["view_manifest_sha256"],
                "Scalar timing cached-view pin differs")
        timing = report.get("timing")
        require(type(timing) is dict and set(timing) == {"cached", "step_fused_cached"},
                "Scalar timing report incomplete")
        for name in ("cached", "step_fused_cached"):
            require(type(timing[name]) is dict and set(timing[name]) ==
                    {"wall", "thread_cpu"}, "Scalar timing clock missing")
            for clock in ("wall", "thread_cpu"):
                row = timing[name][clock]
                require(type(row) is dict and row.get("count") == 500 and
                        all(type(row.get(field)) in (int, float) and
                            math.isfinite(row[field]) and row[field] >= 0
                            for field in ("median_ms", "p95_ms", "p99_ms", "max_ms")),
                        "Scalar timing distribution incomplete")


def _prevalidate(manifest, expected_sha256, baseline_sha):
    require(_digest(expected_sha256) and _digest(baseline_sha),
            "Explicit SHA256 pins required")
    path = Path(manifest)
    data = strict_json(pinned(path, expected_sha256))
    require(type(data) is dict and data.get("schema") ==
            "native-step-scalar-file-only-v1" and
            data.get("status") == "PASS_FILE_ONLY_COMPARE", "Unknown scalar manifest")
    require(all(data.get(flag) is False for flag in _FALSE_FLAGS),
            "Scalar manifest must remain diagnostic only")
    require(data.get("baseline_manifest_sha256") == baseline_sha and
            _digest(data.get("view_manifest_sha256")) and
            _digest(data.get("records_sha256")), "Scalar baseline/view/input pins differ")
    view_path = data.get("view_manifest_path")
    require(type(view_path) is str and Path(view_path).is_absolute(),
            "Scalar cached-view manifest path required")
    source_hashes = data.get("source_hashes")
    require(type(source_hashes) is dict and source_hashes.keys() == _SOURCE_PATHS.keys() and
            all(_digest(value) for value in source_hashes.values()),
            "Scalar source pins incomplete")
    for name, source in _SOURCE_PATHS.items():
        pinned(source, source_hashes[name])
    for kind in ("model", "library"):
        require(_digest(data.get(kind + "_sha256")), "Scalar artifact pin missing")
        pinned(member(path.parent, data.get(kind + "_file")), data[kind + "_sha256"])
    reports = {}
    for phase in _REPORTS:
        require(_digest(data.get(phase + "_report_sha256")),
                "Scalar report pin missing")
        report = strict_json(pinned(member(path.parent, data.get(phase + "_report_file")),
                                    data[phase + "_report_sha256"]))
        _validate_report(phase, report, data)
        reports[phase] = report
    parity_keys = set(reports["parity"]["validation"]["max_errors"])
    require(set(reports["synthetic"]["validation"]["max_errors"]) == parity_keys,
            "Scalar parity state fields differ across reports")
    return path, data, reports


def load_file_only_verified(manifest, *, expected_sha256, baseline_manifest,
                            baseline_sha, bundle):
    """Return a reset scalar-step ScriptModule and diagnostic provenance.

    All caller and artifact pins are checked before scalar library registration.
    This API does not grant runtime or motor-output approval.
    """
    path, data, reports = _prevalidate(manifest, expected_sha256, baseline_sha)
    import torch
    require(data.get("environment") == environment(torch),
            "Scalar target/PyTorch ABI differs")
    cached, cached_proof = load_cached(data["view_manifest_path"],
        expected_sha256=data["view_manifest_sha256"],
        baseline_manifest=baseline_manifest, baseline_sha=baseline_sha,
        bundle=bundle)
    require(not hasattr(torch.ops.sd_step_fileonly_r1, "step"),
            "Step operator already registered; use a fresh process")
    library = member(path.parent, data["library_file"])
    pinned(library, data["library_sha256"])
    torch.ops.load_library(str(library.resolve()))
    pinned(library, data["library_sha256"])
    raw = pinned(member(path.parent, data["model_file"]), data["model_sha256"])
    model = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    graph = str(model.inlined_graph)
    require("sd_step_fileonly_r1::step" in graph and
            "sd_projection_fileonly_r1::project" in graph,
            "Scalar model operators missing")
    with torch.inference_mode():
        verify_aliases(model.controller)
        ids = torch.tensor([0], dtype=torch.long)
        cached.reset(ids)
        model.reset(ids)
        verify_aliases(model.controller)
        compare_state(torch, cached, model, {}, exact=True)
    expected_fields = {"target"}
    for accessor in ("named_buffers", "named_parameters"):
        expected_fields.update(accessor + ":" + name
            for name, value in getattr(model, accessor)()
            if value.dtype.is_floating_point)
    require(set(reports["parity"]["validation"]["max_errors"]) == expected_fields,
            "Scalar named-state parity fields incomplete")
    return model, {
        "schema": "native-step-scalar-file-only-loader-v1",
        "manifest_sha256": expected_sha256,
        "model_sha256": data["model_sha256"],
        "library_sha256": data["library_sha256"],
        "baseline_provenance": cached_proof["baseline_provenance"],
        "view_cache_provenance": cached_proof,
        "parity_report_sha256": data["parity_report_sha256"],
        "synthetic_report_sha256": data["synthetic_report_sha256"],
        "timing_report_sha256": data["timing_report_sha256"],
        "loader_source_sha256": sha(Path(__file__).read_bytes()),
        "diagnostic_only": True,
        "hardware_opened": False,
        "output_allowed": False,
        "approved_for_runtime": False,
        "live_50hz_verified": False,
    }
