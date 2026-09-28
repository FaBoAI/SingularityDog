"""Select the cached projection op in both paths of a private core copy."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import require, sha

OLD = "sd_projection_fileonly_r1"
NEW = "sd_projection_zero_cache_fileonly_r1"


def generate_projection_core(cached_path, destination, expected_cached_sha):
    cached_path, destination = Path(cached_path), Path(destination)
    require(destination.suffix == ".py" and destination.parent.is_dir() and not destination.exists(),
            "A new private .py destination is required")
    require(not any((parent / ".git").exists() for parent in
                    (destination.parent, *destination.parents)),
            "Generated source must stay outside Git")
    raw = cached_path.read_bytes()
    require(sha(raw) == expected_cached_sha, "Cached source changed")
    original = ast.parse(raw)
    transformed = copy.deepcopy(original)
    cls = next(node for node in transformed.body if isinstance(node, ast.ClassDef))
    require(cls.name == "CachedViewSwingCore", "Unexpected cached core class")
    for method_name in ("step", "step_target"):
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                      and node.name == method_name)
        matches = [node for node in ast.walk(method) if isinstance(node, ast.Attribute)
                   and node.attr == OLD]
        require(len(matches) == 1, "Expected one projection op in " + method_name)
        matches[0].attr = NEW
    cls.name = "ZeroCachedProjectionSwingCore"
    inverse = copy.deepcopy(transformed)
    inverse_cls = next(node for node in inverse.body if isinstance(node, ast.ClassDef))
    inverse_cls.name = "CachedViewSwingCore"
    for method_name in ("step", "step_target"):
        inverse_method = next(node for node in inverse_cls.body if isinstance(node, ast.FunctionDef)
                              and node.name == method_name)
        inverse_matches = [node for node in ast.walk(inverse_method) if isinstance(node, ast.Attribute)
                           and node.attr == NEW]
        require(len(inverse_matches) == 1, "Projection inverse unavailable")
        inverse_matches[0].attr = OLD
    require(ast.dump(inverse, include_attributes=False) ==
            ast.dump(original, include_attributes=False),
            "Projection selection changed undeclared controller code")
    ast.fix_missing_locations(transformed)
    generated = (ast.unparse(transformed) + "\n").encode()
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    name = "native_policy_zero_projection_" + sha(generated)[:16]
    spec = importlib.util.spec_from_file_location(name, destination)
    require(spec is not None and spec.loader is not None, "Generated import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.ZeroCachedProjectionSwingCore, {
        "seed_cached_source_sha256": expected_cached_sha,
        "generated_source_sha256": sha(generated),
        "all_undeclared_ast_exact": True,
        "replaced_step_and_step_target_operator": OLD + "::project -> " + NEW + "::project",
        "hardware_opened": False,
        "output_allowed": False,
    }
