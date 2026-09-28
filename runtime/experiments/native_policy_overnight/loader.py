"""Explicit pinned CPU-only loader. Never opens a transport or installs packages."""
import io
from pathlib import Path
from .contracts import (PINS, OPTIONS, environment, member, pinned, require,
                        sha, source_hashes, strict_json, verify_bundle, HERE)

_LOADED_LIBRARY = None


def load_library(path, expected_sha256):
    """Register the existing operator exactly once; unknown registration fails closed."""
    global _LOADED_LIBRARY
    import torch
    path = Path(path)
    pinned(path, expected_sha256)
    path = path.resolve()
    if _LOADED_LIBRARY is not None:
        require(_LOADED_LIBRARY == (str(path), expected_sha256), "Different native library already loaded")
        return
    require(not hasattr(torch.ops.sd_projection_fileonly_r1, "project"),
            "Projection already registered outside this loader; use a fresh process")
    torch.ops.load_library(str(path))
    pinned(path, expected_sha256)
    _LOADED_LIBRARY = (str(path), expected_sha256)


def load_verified(manifest, *, expected_manifest_sha256, bundle):
    """Return (reset ScriptModule, provenance); caller must use inference_mode.

    Inputs: CPU float32 [1,3] gyro/gravity/command and [1,12] q/dq/h.
    Output: CPU float32 [1,12] calibrated-model target, with no device output.
    Mutable controller state is owned by this instance: do not share concurrently.
    """
    import torch
    path = Path(manifest)
    data = strict_json(pinned(path, expected_manifest_sha256))
    require(data.get("schema") == "native-policy-overnight-v1" and
            data.get("status") == "VALIDATED_FILE_ONLY", "Unvalidated artifact manifest")
    require(data.get("source_hashes") == source_hashes(), "Experiment source differs from validated build")
    require(data.get("bundle_hashes") == PINS and data.get("options") == OPTIONS, "Bundle/options mismatch")
    verify_bundle(bundle)
    require(data.get("environment") == environment(torch), "Target/PyTorch ABI differs; build on this target")
    require(data.get("frozen") is False and data.get("hardware_opened") is False,
            "Unexpected artifact scope")
    require(all(data.get(key) is False for key in ("output_allowed", "approved_for_runtime", "live_50hz_verified")),
            "Artifact must remain an explicitly unapproved experiment")
    reference = HERE.parents[1] / "singularitydog_hw" / "policy_shadow.py"
    require(sha(reference.read_bytes()) == data.get("reference_loader_sha256"), "Current loader changed")
    validation = strict_json(pinned(member(path.parent, data["validation_file"]), data["validation_sha256"]))
    require(validation.get("status") == "PASS" and validation.get("frames") == 240 and
            validation.get("rejection_count") == 26, "Incomplete equivalence report")
    raw = pinned(member(path.parent, data["model_file"]), data["model_sha256"])
    load_library(member(path.parent, data["library_file"]), data["library_sha256"])
    model = torch.jit.load(io.BytesIO(raw), map_location="cpu").eval()
    require("sd_projection_fileonly_r1::project" in str(model.inlined_graph), "Native projection missing")
    with torch.inference_mode():
        model.reset(torch.tensor([0], dtype=torch.long))
    return model, dict(manifest_sha256=expected_manifest_sha256, model_sha256=data["model_sha256"],
                      library_sha256=data["library_sha256"], hardware_opened=False,
                      output_allowed=False, live_50hz_verified=False)
