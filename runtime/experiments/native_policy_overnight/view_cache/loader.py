"""Explicit v1 cached-view loader for file-only diagnostics; never approves output.

The v1 comparison JSON embeds its validation report and pins generated source
and model bytes. It predates this loader: only its two build-time helper files
are required in variant_source_hashes. The current loader hash/version is
returned separately. This is not a legacy native-policy artifact or loader.
"""
import io
import json
from pathlib import Path
import re

from ..contracts import (HERE, REUSED_PINS, environment, member, pinned, require,
                         sha, strict_json)
from ..loader import load_verified
from ..verification import compare_state
from .generator import _transform, verify_aliases


_HELPER_PINS = {
    "__init__.py": "de8603cddbae0b9fa25ca7800907eb85f3598ced538b717a51ab400a2ffabb6e",
    "generator.py": "b0ae01397ba4270d45101bb5e4370111cc0e6a159c2ddd57b530757d85377534",
}
_BUILD_SCRIPT_SHA256 = "3278da17134cf738ab5f45c3ace1cd2f028d9bebb4479a34443ee3afec4928ae"
_GENERATED_FILE = "cached_view_core.py"
_LOADER_SCHEMA = "native-view-cache-file-only-loader-v1"


def _digest(value):
    return type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _same_fields(actual, expected):
    return (type(actual) is dict and actual.keys() == expected.keys() and
            all(type(actual[key]) is type(value) and actual[key] == value
                for key, value in expected.items()))


def _rejection_reasons():
    out = {}
    for name in ("gyro", "gravity", "command", "q", "dq", "h"):
        for suffix in ("nan", "inf"):
            out[name + "_" + suffix] = "nonfinite observation"
        out[name + "_shape"] = "observation shape"
    out.update({
        "gravity_norm": "IMU gravity vector is not normalized",
        "gravity_heading_singular": "IMU heading rate singular near vertical pitch",
        "negative_h": "exposure outside [0,1]",
        "excess_h": "exposure outside [0,1]",
    })
    for name in ("mixed_command", "translation_and_yaw", "forward_limit", "yaw_limit"):
        out[name] = "Outside registered L11 cardinal, pure-yaw or stop command domain"
    return out


def _validation(data):
    validation = data.get("validation")
    require(type(validation) is dict, "Missing cached-view validation report")
    require(type(validation.get("synthetic_frames")) is int and
            validation["synthetic_frames"] == 240 and
            type(validation.get("saved_stateful_calls_per_model")) is int and
            validation["saved_stateful_calls_per_model"] == 120,
            "Incomplete cached-view frame validation")
    require(all(validation.get(key) is True for key in
                ("exact_named_state_parameters_targets", "saved_reload_exact",
                 "aliases_after_reload_verified")) and
            validation.get("input_mutation") is False and
            data.get("timed_outputs_and_final_state_exact") is True,
            "Cached-view exactness/ownership validation missing")
    rejected = validation.get("rejection_cases")
    require(type(rejected) is list and len(rejected) == 26 and
            all(type(row) is dict and set(row) == {"case", "reason"} and
                type(row["case"]) is str and type(row["reason"]) is str for row in rejected),
            "Incomplete cached-view rejection validation")
    require({row["case"]: row["reason"] for row in rejected} == _rejection_reasons(),
            "Cached-view rejection reasons differ")
    maxima = validation.get("max_errors")
    require(type(maxima) is dict and "target" in maxima and
            all(type(key) is str and type(value) in (int, float) and value == 0.
                for key, value in maxima.items()), "Cached-view validation must be exact")
    return validation


def load_file_only_verified(manifest, *, expected_sha256, baseline_manifest,
                            baseline_sha, bundle):
    """Return (reset CPU ScriptModule, diagnostic provenance), using explicit pins.

    Caller must retain the baseline native manifest/SHA and use inference_mode.
    No threads, clocks, transports, output permissions or policy inputs change.
    Each call creates its own mutable model; sharing it concurrently is unsafe.
    """
    require(_digest(expected_sha256) and _digest(baseline_sha), "Explicit SHA256 pins required")
    path = Path(manifest)
    data = strict_json(pinned(path, expected_sha256))
    require(type(data) is dict and data.get("schema") == "native-view-cache-file-only-v1" and
            data.get("status") == "PASS_FILE_ONLY_COMPARE", "Unvalidated cached-view artifact manifest")
    require(all(data.get(key) is False for key in
                ("hardware_opened", "output_allowed", "approved_for_runtime",
                 "live_50hz_verified", "existing_legacy_loader_schema_compatible")),
            "Cached-view artifact must remain an unapproved file-only diagnostic")
    require(type(data.get("errors")) is list and not data["errors"], "Cached-view comparison has errors")
    require(data.get("source_script_sha256") == _BUILD_SCRIPT_SHA256 and
            _digest(data.get("saved_records_sha256")), "Unknown cached-view v1 build provenance")
    helper_hashes = data.get("variant_source_hashes")
    require(type(helper_hashes) is dict and
            all(type(name) is str and _digest(value) for name, value in helper_hashes.items()) and
            all(helper_hashes.get(name) == value for name, value in _HELPER_PINS.items()),
            "Cached-view build helper pins differ")
    helper_directory = Path(__file__).resolve().parent
    for name, value in _HELPER_PINS.items():
        pinned(helper_directory / name, value)

    source_proof = data.get("view_source_proof")
    require(type(source_proof) is dict and
            source_proof.get("schema") == "native-policy-view-cache-source-v1" and
            source_proof.get("seed_core_sha256") == REUSED_PINS["lean_swing_core.py"] and
            source_proof.get("generator_source_sha256") == _HELPER_PINS["generator.py"] and
            source_proof.get("original_ast_exact_after_inverse_view_substitution") is True and
            all(source_proof.get(key) is False for key in
                ("hardware_opened", "output_allowed", "approved_for_runtime")),
            "Cached-view source proof differs")
    generated, counts = _transform(pinned(HERE / "lean_swing_core.py", REUSED_PINS["lean_swing_core.py"]))
    require(_same_fields(source_proof.get("declared_view_replacements"), counts) and
            source_proof.get("generated_core_sha256") == sha(generated),
            "Cached-view transformation differs")
    require(pinned(member(path.parent, _GENERATED_FILE), sha(generated)) == generated,
            "Generated cached-view source differs")
    validation = _validation(data)
    require(_digest(data.get("model_sha256")), "Cached-view model pin missing")
    raw = pinned(member(path.parent, data.get("model_file")), data["model_sha256"])
    declared_baseline = data.get("baseline_provenance")
    require(type(declared_baseline) is dict and
            declared_baseline.get("manifest_sha256") == baseline_sha,
            "Cached-view baseline manifest pin differs")

    # This remains the authority for all original sources, bundle/options, ABI,
    # baseline model/validation and native library registration/provenance.
    baseline, baseline_proof = load_verified(baseline_manifest,
        expected_manifest_sha256=baseline_sha, bundle=bundle)
    require(_same_fields(declared_baseline, baseline_proof), "Cached-view baseline provenance differs")
    import torch
    require(_same_fields(data.get("environment"), environment(torch)), "Cached-view target/PyTorch ABI differs")
    expected_maxima = {"target"}
    for accessor in ("named_buffers", "named_parameters"):
        expected_maxima.update(accessor + ":" + name for name, value in getattr(baseline, accessor)()
                               if value.dtype.is_floating_point)
    require(set(validation["max_errors"]) == expected_maxima, "Cached-view named-state validation incomplete")
    model = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    require("sd_projection_fileonly_r1::project" in str(model.inlined_graph), "Cached-view native projection missing")
    with torch.inference_mode():
        verify_aliases(model.controller)
        model.reset(torch.tensor([0], dtype=torch.long))
        verify_aliases(model.controller)
        compare_state(torch, baseline, model, {}, exact=True)
    loader_path = Path(__file__)
    loader_hash = sha(loader_path.read_bytes())
    return model, dict(schema=_LOADER_SCHEMA, artifact_schema=data["schema"],
        manifest_sha256=expected_sha256, model_sha256=data["model_sha256"],
        library_sha256=baseline_proof["library_sha256"], baseline_provenance=baseline_proof,
        generated_core_sha256=sha(generated), build_helper_source_hashes=dict(_HELPER_PINS),
        loader_source_sha256=loader_hash,
        validation_sha256=sha(json.dumps(validation, sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()),
        validation_embedded_in_pinned_manifest=True, diagnostic_only=True,
        hardware_opened=False, output_allowed=False, approved_for_runtime=False,
        live_50hz_verified=False, existing_legacy_loader_schema_compatible=False)
