"""Explicit pinned FK model loader for a separate STOP diagnostic.

PLAN validates files only. This module does not open devices, compile artifacts,
change the ordinary scalar loader, or grant active-controller qualification.
"""
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import stat
import struct

from ..contracts import require, strict_json, environment
from ..model_call_fastpath import scalar_loader
from ..view_cache.generator import verify_aliases

SCHEMA = "singularitydog.fk-cache-stop-diagnostic-artifact.v1"
STATUS = "PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL"
PREFIX = "runtime/experiments/native_policy_overnight/"
SOURCE_NAMES = (
    "contracts.py", "verification.py", "lean_swing_core.py", "lean_swing_deployment.py",
    "view_cache/generator.py", "model_call_fastpath/step_generator.py",
    "model_call_fastpath/step_scalar.cpp", "model_call_fastpath/run.py",
    "observation_fusion/observation.cpp", "observation_fusion/generator.py",
    "projection_zero_cache/projection.cpp", "projection_zero_cache/torch_projection.cpp",
    "projection_zero_cache/generator.py", "full_recurrent_fusion/generator.py",
    "target_tail_fusion/__init__.py", "target_tail_fusion/target.cpp",
    "target_tail_fusion/generator.py", "target_tail_fusion/saved_profile.py",
    "projection.cpp", "torch_projection.cpp", "target_tail_fk_cache/__init__.py",
    "target_tail_fk_cache/target.cpp", "target_tail_fk_cache/generator.py",
    "target_tail_fk_cache/saved_profile.py")
REFERENCES = {"file_only_report", "file_only_audit", "candidate_source_manifest",
              "scalar_manifest", "baseline_manifest", "saved_report", "saved_records",
              "saved_raw_audit", "replay_helper", "model", "library"}
GENERATED = {"cached.py", "step.py", "target.py", "fk_cache.py"}
FALSE_FLAGS = ("output_allowed", "approved_for_runtime", "active_controller_qualification",
               "timing_admission_eligible", "live_50hz_verified")
OPERATORS = ("sd_projection_fileonly_r1::project", "sd_step_fileonly_r1::step",
             "sd_target_tail_fk_cache_fileonly_r1::target")
MAX_BYTES = 64 * 1024 * 1024


def digest(value):
    require(type(value) is str and re.fullmatch("[0-9a-f]{64}", value),
            "Explicit lowercase SHA256 required")
    return value


def reference(value):
    require(type(value) is dict and set(value) == {"path", "sha256"},
            "Exact path/SHA256 reference required")
    digest(value["sha256"])
    require(type(value["path"]) is str, "Reference path must be a string")
    path = Path(value["path"])
    require(path.is_absolute() and ".." not in path.parts
            and not any(p.is_symlink() for p in (path, *path.parents)),
            "Absolute non-symlink reference required")
    return path


def read(value):
    path = reference(value)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and before.st_size <= MAX_BYTES,
                "Bounded regular reference required")
        with os.fdopen(fd, "rb", closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    require(len(raw) <= MAX_BYTES and len(raw) == before.st_size and
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) and
            hashlib.sha256(raw).hexdigest() == value["sha256"],
            "Reference changed or SHA256 differs")
    return raw


def _json(value):
    result = strict_json(read(value))
    require(type(result) is dict, "JSON object reference required")
    return result


def _float32(values, width):
    require(type(values) is list and len(values) == width, "Saved vector width differs")
    for value in values:
        require(type(value) in (int, float) and math.isfinite(value), "Invalid saved scalar")
        try:
            narrowed = struct.unpack("=f", struct.pack("=f", value))[0]
        except (OverflowError, struct.error):
            raise ValueError("Saved scalar overflows float32") from None
        require(math.isfinite(narrowed), "Saved scalar overflows float32")


def _validate_saved(report, records, scalar_source):
    require(report.get("status") == "COMPLETE_DIAGNOSTIC" and
            type(report.get("cycles_completed")) is int and report["cycles_completed"] == 501
            and report.get("errors") == [] and report.get("scalar_step_model_source") == scalar_source,
            "Original complete scalar501 report required")
    for name in ("motor_enable_sent", "learned_targets_sent", "approved_for_runtime",
                 "full_controller_50Hz_verified"):
        require(report.get(name) is False, "Original no-output scope differs")
    require(type(records) is list and len(records) == 501, "All501 original records required")
    keys = ("gyro_body_rad_s", "gravity_body_unit", "command", "q_model_rad",
            "dq_model_rad_s", "h_hypothesis12")
    for index, row in enumerate(records):
        require(type(row) is dict and type(row.get("cycle")) is int and row["cycle"] == index + 1,
                "Original record order differs")
        observed = row.get("observed")
        require(type(observed) is dict and observed.get("status") == "TICK_OBSERVED_NO_OUTPUT"
                and observed.get("output_allowed") is False and type(observed.get("tick_index")) is int
                and observed["tick_index"] == index, "Original observation incomplete")
        inputs = observed.get("inputs")
        require(type(inputs) is dict and set(inputs) == set(keys), "Original six inputs required")
        for key, width in zip(keys, (3, 3, 3, 12, 12, 12)):
            _float32(inputs[key], width)
        for key, width in (("q_target_rad_diagnostic_only", 12), ("actor_residual12", 12),
                           ("observation74", 74)):
            _float32(observed.get(key), width)


def _validate_report(report):
    require(report.get("schema") == "singularitydog.saved-input-target-fk-cache-profile.v1"
            and report.get("status") == "PASS_FILE_ONLY_SCALAR_TAIL_FK_CACHE_COMPARE"
            and type(report.get("cycles")) is int and report["cycles"] == 501,
            "Verified scalar/first-tail/FK501 report required")
    for name in ("hardware_opened", "output_allowed", "approved_for_runtime",
                 "live_50hz_verified", "actual_controller_qualification"):
        require(report.get(name) is False, "File-only validation scope differs")
    validation = report.get("saved_validation")
    require(type(validation) is dict and type(validation.get("saved_recurrent_calls")) is int
            and validation["saved_recurrent_calls"] == 501 and
            validation.get("policies") == ["current_scalar", "first_target_tail", "fk_cache_target_tail"]
            and all(validation.get(key) is True for key in ("saved_output_actor_observation_bits_exact",
                "all_named_state_bits_each_call", "input_bits_preserved", "same_partial_state_after_rejections"))
            and type(validation.get("rejection_count")) is int and validation["rejection_count"] == 26,
            "Saved bits/state/rejection proof incomplete")
    expected_rejections = scalar_loader._rejection_reasons()
    rows = validation.get("rejections")
    require(type(rows) is list and len(rows) == 26 and all(type(row) is dict
            and set(row) == {"case", "reason"} for row in rows)
            and {row["case"]: row["reason"] for row in rows} == expected_rejections,
            "Exact26 rejection reasons required")
    synthetic = report.get("synthetic_validation")
    require(type(synthetic) is dict and set(synthetic) == {"first_target_tail", "fk_cache_target_tail"},
            "Two synthetic proofs required")
    for proof in synthetic.values():
        require(type(proof) is dict and type(proof.get("frames")) is int
                and proof["frames"] == 240 and proof.get("reset_before") == [0, 125]
                and all(type(value) is int for value in proof["reset_before"])
                and proof.get("all_named_state_bits_exact") is True,
                "Synthetic240 state proof incomplete")


def _generated_sources(report, references):
    expected = {}
    for proof in (report["first_target_tail"], report["fk_cache_target_tail"]):
        for path, pin in proof["generated_sources"].items():
            name = Path(path).name
            require(name in GENERATED and name not in expected, "Generated source names differ")
            expected[name] = pin
    require(set(references) == set(expected) == GENERATED, "All four generated sources required")
    raw = {}
    for name, ref in references.items():
        require(ref["sha256"] == expected[name], "Generated source binding differs")
        raw[name] = read(ref)
    from ..target_tail_fusion.generator import transform as first_transform
    from .generator import transform as fk_transform
    require(first_transform(raw["step.py"]) == raw["target.py"] and
            fk_transform(raw["target.py"]) == raw["fk_cache.py"],
            "Generated target/FK inverse source proof differs")


def _executing_sources(source_refs):
    """Authenticate executing Python dependencies, not just retained copies."""
    from .. import contracts
    from ..view_cache import generator as view_generator
    from ..target_tail_fusion import generator as first_generator
    from . import generator as fk_generator
    for name, module in (("contracts.py", contracts),
                         ("view_cache/generator.py", view_generator),
                         ("target_tail_fusion/generator.py", first_generator),
                         ("target_tail_fk_cache/generator.py", fk_generator)):
        read({"path": str(Path(module.__file__).absolute()),
              "sha256": source_refs[PREFIX + name]["sha256"]})


def _cpp_dependencies(report, source_refs):
    """The opaque FK operator calls dependencies in pinned C++, not JIT IR."""
    first = source_refs[PREFIX + "target_tail_fusion/target.cpp"]
    candidate = source_refs[PREFIX + "target_tail_fk_cache/target.cpp"]
    delta = report.get("cpp_delta")
    require(type(delta) is dict and delta.get("first_tail_source_sha256") == first["sha256"]
            and delta.get("fk_cache_source_sha256") == candidate["sha256"]
            and delta.get("inverse_bytes_exact") is True
            and report["fk_cache_target_tail"].get("library_source_sha256") == candidate["sha256"],
            "Transitive C++ source/library proof differs")
    raw = read(candidate)
    for op in OPERATORS[:2]:
        require(raw.count(('.findSchemaOrThrow("' + op + '", "")').encode()) == 1,
                "Exact transitive C++ dispatcher dependency required: " + op)


def _executing_replay(ref):
    from ..model_call_fastpath import saved_input_profile as replay
    read({"path": str(Path(replay.__file__).absolute()), "sha256": ref["sha256"]})


def _prevalidate(manifest, expected_sha256, *, scalar_manifest, scalar_sha,
                 baseline_manifest, baseline_sha):
    ref = {"path": str(manifest), "sha256": digest(expected_sha256)}
    data = _json(ref)
    require(data.get("schema") == SCHEMA and data.get("status") == STATUS
            and all(data.get(flag) is False for flag in FALSE_FLAGS), "Unapproved diagnostic artifact required")
    refs = data.get("references")
    require(type(refs) is dict and set(refs) == REFERENCES, "Complete diagnostic references required")
    for name, value in refs.items():
        read(value)
    require(refs["scalar_manifest"] == {"path": str(scalar_manifest), "sha256": digest(scalar_sha)}
            and refs["baseline_manifest"] == {"path": str(baseline_manifest), "sha256": digest(baseline_sha)},
            "Explicit scalar/baseline selections differ")
    report = _json(refs["file_only_report"])
    _validate_report(report)
    for key, field in (("saved_report", "original_report_sha256"), ("saved_records", "original_records_sha256"),
                       ("saved_raw_audit", "raw_audit_sha256"),
                       ("replay_helper", "replay_helper_sha256"),
                       ("candidate_source_manifest", "candidate_source_manifest_sha256")):
        require(refs[key]["sha256"] == report[field], "Original validation binding differs: " + key)
    source_manifest = _json(refs["candidate_source_manifest"])
    expected_sources = {PREFIX + name for name in SOURCE_NAMES}
    source_refs = data.get("source_references")
    require(type(source_refs) is dict and set(source_refs) == set(report["candidate_source_sha256"])
            == set(source_manifest["files"]) == expected_sources,
            "Exactly24 candidate source bindings required")
    for name, value in source_refs.items():
        require(value["sha256"] == source_manifest["files"][name]
                == report["candidate_source_sha256"][name], "Candidate source SHA differs")
        read(value)
    _executing_sources(source_refs)
    _executing_replay(refs["replay_helper"])
    _cpp_dependencies(report, source_refs)
    integration = data.get("integration_sources")
    require(type(integration) is dict and "diagnostic_loader.py" in integration,
            "Separate integration source pins required")
    require(reference(integration["diagnostic_loader.py"]) == Path(__file__).absolute(),
            "Executing loader must be pinned separately")
    for value in integration.values():
        read(value)
    proof = report["fk_cache_target_tail"]
    require(refs["model"]["sha256"] == proof["model_sha256"]
            and refs["library"]["sha256"] == proof["library_sha256"]
            and proof.get("same_scalar_projection_actor_observation") is True
            and proof.get("undeclared_methods_exact") is True,
            "Candidate artifact/proof differs")
    read(refs["model"]); read(refs["library"])
    _generated_sources(report, data.get("generated_source_references", {}))
    audit = _json(refs["file_only_audit"])
    require(audit.get("status") == "PASS_TARGET_RAW_TIMING_SOURCE_AND_RECEIPT_AUDIT"
            and audit.get("report_sha256") == refs["file_only_report"]["sha256"]
            and audit.get("all30_state_bits_proven_by_harness") is True
            and type(audit.get("reject_cases")) is int and audit["reject_cases"] == 26
            and type(audit.get("synthetic_per_candidate")) is int
            and audit["synthetic_per_candidate"] == 240 and audit.get("hardware_opened") is False
            and audit.get("output_allowed") is False, "Independent file-only audit binding incomplete")
    artifact_pins = audit.get("artifact_sha256", {})
    require(artifact_pins.get("report-fk-cache-artifacts/fk_cache_tail_fileonly.pt") == refs["model"]["sha256"]
            and artifact_pins.get("report-fk-cache-artifacts/target_tail_fk_cache.so") == refs["library"]["sha256"],
            "Independent candidate artifact pins differ")
    saved = _json(refs["saved_report"])
    records = strict_json(read(refs["saved_records"]))
    _validate_saved(saved, records, report["model_source"])
    raw_audit = _json(refs["saved_raw_audit"])
    inputs = raw_audit.get("input_file_sha256")
    require(raw_audit.get("status") == "PASS_RAW_EVIDENCE"
            and type(raw_audit.get("cycles_audited")) is int and raw_audit["cycles_audited"] == 501
            and type(inputs) is dict and all(refs[key]["sha256"] in inputs.values()
            for key in ("saved_report", "saved_records")), "Original501 raw audit binding differs")
    _, scalar_data, _ = scalar_loader._prevalidate(scalar_manifest, scalar_sha, baseline_sha)
    require(scalar_data["model_sha256"] == report["model_source"]["model_sha256"]
            and scalar_data["library_sha256"] == report["model_source"]["library_sha256"]
            and report["model_source"]["manifest_sha256"] == scalar_sha
            and report["model_source"]["baseline_provenance"]["manifest_sha256"] == baseline_sha
            and scalar_data["environment"] == report["environment"], "Original scalar identity differs")
    for group in (refs, source_refs, integration, data["generated_source_references"]):
        for value in group.values():
            read(value)
    _executing_sources(source_refs)
    read(ref)
    return data, report


def create_manifest(*, references, source_references, generated_source_references,
                    integration_sources):
    """Return an unapproved binding document; do not write or infer references."""
    require(type(references) is dict and set(references) == REFERENCES,
            "Complete diagnostic references required")
    for group in (references, source_references, generated_source_references, integration_sources):
        require(type(group) is dict, "Explicit reference maps required")
        for value in group.values():
            reference(value)
    # JSON copying gives this caller independent ownership of the reference tree.
    return json.loads(json.dumps({"schema": SCHEMA, "status": STATUS,
        "references": references, "source_references": source_references,
        "generated_source_references": generated_source_references,
        "integration_sources": integration_sources, **dict.fromkeys(FALSE_FLAGS, False)},
        allow_nan=False))


def plan(manifest, *, expected_sha256, scalar_manifest, scalar_sha, baseline_manifest, baseline_sha):
    data, report = _prevalidate(manifest, expected_sha256, scalar_manifest=scalar_manifest,
        scalar_sha=scalar_sha, baseline_manifest=baseline_manifest, baseline_sha=baseline_sha)
    return {"schema": "singularitydog.fk-cache-stop-diagnostic-loader-plan.v1",
            "manifest_sha256": expected_sha256, "backend": "pinned_fk_cache_cpp",
            "model_sha256": data["references"]["model"]["sha256"],
            "library_sha256": data["references"]["library"]["sha256"],
            "validated_saved_calls": report["cycles"], "torch_or_native_loaded": False,
            **dict.fromkeys(FALSE_FLAGS, False)}


def _state_bits(torch, one, two):
    require(one is not two, "Independent scalar/candidate instances required")
    stores = []
    for model in (one, two):
        stores.append({v.untyped_storage().data_ptr() for _, v in
                       list(model.named_buffers()) + list(model.named_parameters()) if v.numel()})
    require(stores[0].isdisjoint(stores[1]), "Candidate shares scalar state storage")
    for accessor in ("named_buffers", "named_parameters"):
        left, right = dict(getattr(one, accessor)()), dict(getattr(two, accessor)())
        require(left.keys() == right.keys(), "Named state fields differ")
        for name, value in left.items():
            other = right[name]
            require(value.shape == other.shape and value.dtype == other.dtype and value.device == other.device
                    and bool(torch.isfinite(value).all()) and bool(torch.isfinite(other).all())
                    and torch.equal(value.contiguous().reshape(-1).view(torch.uint8),
                                    other.contiguous().reshape(-1).view(torch.uint8)),
                    "Named state bits differ: " + name)


def _validate_models(torch, scalar, candidate):
    require(scalar is not candidate, "Independent model instances required")
    names = set(scalar.controller._c._method_names())
    require(names == set(candidate.controller._c._method_names()), "Controller methods differ")
    for name in names - {"step_target"}:
        require(scalar.controller._c._get_method(name).code == candidate.controller._c._get_method(name).code,
                "Undeclared controller method differs: " + name)
    # TorchScript changes generated type suffixes when loading the second copy.
    # Strip that serialization-only suffix, not operators or executable code.
    def code(value):
        return re.sub(r"___torch_mangle_[0-9]+\.", "", value)
    for one, two, label in ((scalar, candidate, "policy"),
                            (scalar.actor, candidate.actor, "actor")):
        methods = set(one._c._method_names())
        require(methods == set(two._c._method_names()), label + " methods differ")
        for name in methods:
            require(code(one._c._get_method(name).code) == code(two._c._get_method(name).code),
                    label + " executable method differs: " + name)
    verify_aliases(scalar.controller); verify_aliases(candidate.controller)
    ids = torch.tensor([0], dtype=torch.long)
    with torch.inference_mode():
        scalar.reset(ids); candidate.reset(ids)
        verify_aliases(scalar.controller); verify_aliases(candidate.controller)
        _state_bits(torch, scalar, candidate)
    graph = str(candidate.inlined_graph)
    require(OPERATORS[-1] in graph and "sd_target_tail_fileonly_r1::target" not in graph,
            "Candidate operator graph differs")


def load_diagnostic_verified(manifest, *, expected_sha256, scalar_manifest, scalar_sha,
                             baseline_manifest, baseline_sha, bundle):
    """Load one independent reset candidate; no compilation or transport API."""
    kwargs = dict(scalar_manifest=scalar_manifest, scalar_sha=scalar_sha,
                  baseline_manifest=baseline_manifest, baseline_sha=baseline_sha)
    data, report = _prevalidate(manifest, expected_sha256, **kwargs)
    import torch
    require(environment(torch) == report["environment"], "Candidate Torch/CPU ABI differs")
    require(not hasattr(torch.ops.sd_target_tail_fk_cache_fileonly_r1, "target"),
            "FK operator already registered; fresh process required")
    scalar, source = scalar_loader.load_file_only_verified(scalar_manifest,
        expected_sha256=scalar_sha, baseline_manifest=baseline_manifest,
        baseline_sha=baseline_sha, bundle=bundle)
    require(source == report["model_source"], "Verified scalar dependency provenance differs")
    library = data["references"]["library"]
    read(library); torch.ops.load_library(str(reference(library))); read(library)
    raw = read(data["references"]["model"])
    candidate = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    _validate_models(torch, scalar, candidate)
    _prevalidate(manifest, expected_sha256, **kwargs)
    return candidate, {"schema": "singularitydog.fk-cache-stop-diagnostic-loader.v1",
        "manifest_sha256": expected_sha256, "model_sha256": data["references"]["model"]["sha256"],
        "library_sha256": library["sha256"], "baseline_provenance": source["baseline_provenance"],
        "original_scalar_dependency": source, "file_only_report_sha256": data["references"]["file_only_report"]["sha256"],
        "candidate_policy_backend_provenance": "EXPERIMENTAL_STOP_DIAGNOSTIC_ONLY",
        "loader_source_sha256": hashlib.sha256(read(data["integration_sources"]["diagnostic_loader.py"])).hexdigest(),
        "diagnostic_only": True, "hardware_opened": False, **dict.fromkeys(FALSE_FLAGS, False)}
