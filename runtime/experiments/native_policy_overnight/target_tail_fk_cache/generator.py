"""Switch only first-tail namespace/class in a fresh private core source."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..model_call_fastpath import saved_input_profile as replay


def transform(raw):
    original = ast.parse(raw); changed = copy.deepcopy(original)
    classes = [node for node in changed.body if isinstance(node, ast.ClassDef)]
    replay.require(len(classes) == 1 and classes[0].name == "FusedTargetTailSwingCore",
                   "Pinned first-target core required")
    cls = classes[0]; cls.name = "FusedFKCacheTargetSwingCore"
    calls = [node for node in ast.walk(cls) if isinstance(node, ast.Attribute)
             and node.attr == "sd_target_tail_fileonly_r1"]
    replay.require(len(calls) == 1, "One first-target operator reference required")
    calls[0].attr = "sd_target_tail_fk_cache_fileonly_r1"
    inverse = copy.deepcopy(changed); inverse_cls = next(n for n in inverse.body if isinstance(n, ast.ClassDef))
    inverse_cls.name = "FusedTargetTailSwingCore"
    for node in ast.walk(inverse_cls):
        if isinstance(node, ast.Attribute) and node.attr == "sd_target_tail_fk_cache_fileonly_r1":
            node.attr = "sd_target_tail_fileonly_r1"
    replay.require(ast.dump(inverse, include_attributes=False) == ast.dump(original, include_attributes=False),
                   "FK candidate changed undeclared first-tail code")
    ast.fix_missing_locations(changed)
    return (ast.unparse(changed) + "\n").encode()


def generate_core(first_path, destination, expected_sha):
    raw = replay._read(first_path, expected_sha); destination = replay._output_path(destination)
    replay.require(destination.suffix == ".py", "New private source required")
    result = transform(raw)
    with os.fdopen(os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "wb") as stream:
        stream.write(result)
    name = "native_policy_fk_cache_" + replay._sha(result)[:16]
    replay.require(name not in sys.modules, "Fresh generated module required")
    spec = importlib.util.spec_from_file_location(name, destination)
    replay.require(spec is not None and spec.loader is not None, "Source import unavailable")
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module)
    return module.FusedFKCacheTargetSwingCore, {"seed_first_tail_source_sha256": expected_sha,
        "generated_source_sha256": replay._sha(result), "first_tail_ast_exact_after_namespace_inverse": True,
        "hardware_opened": False, "output_allowed": False, "approved_for_runtime": False}
