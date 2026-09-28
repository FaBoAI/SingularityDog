"""Explicit target-local build and equivalence gate. No automatic installation."""
import copy
import io
import json
import os
from pathlib import Path
import platform
import subprocess
import time
from .contracts import (HERE, OPTIONS, PINS, environment, reference_policy, require,
                        sha, source_hashes, source_scope_check)
from .loader import load_library
from .verification import validate


def write_json(path, data):
    with Path(path).open("x", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, allow_nan=False)
        handle.write("\n")


def compile_library(torch, output, compiler):
    from torch.utils.cpp_extension import include_paths, library_paths
    require(platform.system() in ("Darwin", "Linux"), "Only explicit Darwin/Linux CPU builds supported")
    require(type(compiler) is str and compiler and not compiler.startswith("-"), "Invalid compiler executable")
    libraries = library_paths()
    require(libraries, "Installed PyTorch libraries missing")
    command = [compiler, "-std=c++20", "-O3", "-ffp-contract=off", "-shared", "-fPIC",
               "-D_GLIBCXX_USE_CXX11_ABI="+str(int(torch._C._GLIBCXX_USE_CXX11_ABI)),
               str(HERE / "torch_projection.cpp"), "-o", str(output)]
    for path in include_paths():
        command += ["-I", path]
    for path in libraries:
        command += ["-L", path, "-Wl,-rpath,"+path]
    command += ["-ltorch_cpu", "-lc10"]
    started = time.perf_counter_ns()
    result = subprocess.run(command, capture_output=True, text=True, timeout=180, check=False)
    require(result.returncode == 0, "C++ build failed: " + result.stderr[-6000:])
    return dict(command=command, elapsed_ms=(time.perf_counter_ns()-started)/1e6,
                stderr=result.stderr[-6000:], source_hashes=source_hashes())


def build(bundle, output, *, compiler="c++"):
    """Build new artifacts; publish a loadable manifest only after parity passes."""
    import torch
    output = Path(output).absolute()
    require(not output.exists(), "Output directory must be new")
    require(output.parent.is_dir(), "Output parent must already exist")
    scope = source_scope_check(bundle)
    initial_source_hashes = source_hashes()
    output.mkdir(mode=0o700)
    os.chmod(output, 0o700)
    report = dict(status="INCOMPLETE", hardware_opened=False, output_allowed=False,
                  source_scope=scope, environment=environment(torch))
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        reference, reference_sha = reference_policy(bundle)
        library = output / "torch_projection_fileonly.so"
        report["compile"] = compile_library(torch, library, compiler)
        library_sha = sha(library.read_bytes())
        load_library(library, library_sha)
        from .lean_swing_deployment import DeployableSwingPolicy
        candidate = torch.jit.script(DeployableSwingPolicy(copy.deepcopy(reference.actor), **OPTIONS).eval())
        require("sd_projection_fileonly_r1::project" in str(candidate.inlined_graph), "Native op missing")
        model = output / "lean_policy_fileonly.pt"
        torch.jit.save(candidate, str(model))
        model_raw = model.read_bytes()
        reloaded = torch.jit.load(io.BytesIO(model_raw), map_location="cpu").eval()
        validation = validate(torch, reference, candidate, reloaded)
        require(source_hashes() == initial_source_hashes, "Experiment sources changed during build/validation")
        write_json(output / "validation.json", validation)
        manifest = dict(schema="native-policy-overnight-v1", status="VALIDATED_FILE_ONLY",
            hardware_opened=False, output_allowed=False, approved_for_runtime=False,
            live_50hz_verified=False, frozen=False, source_hashes=source_hashes(),
            bundle_hashes=PINS, options=OPTIONS, reference_loader_sha256=reference_sha,
            environment=environment(torch), model_file=model.name, model_sha256=sha(model_raw),
            library_file=library.name, library_sha256=library_sha, validation_file="validation.json",
            validation_sha256=sha((output / "validation.json").read_bytes()))
        write_json(output / "manifest.json", manifest)
        report.update(status="PASS", manifest_sha256=sha((output / "manifest.json").read_bytes()))
        return report
    except BaseException as error:
        report.update(status="FAILED", error=type(error).__name__+": "+str(error))
        raise
    finally:
        write_json(output / "build-report.json", report)
