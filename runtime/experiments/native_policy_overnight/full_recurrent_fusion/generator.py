"""Generate a private core changing only observation, step inputs, and projection op."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import require, sha
from ..model_call_fastpath.step_generator import _BODY as STEP_BODY
from ..observation_fusion.generator import _BODY as OBSERVATION_BODY
from ..projection_zero_cache.generator import OLD, NEW


def _method(cls, name):
    return next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name == name)


def _body(text):
    return ast.parse("def replacement():\n" +
                     "".join("    " + line + "\n" for line in text.strip().splitlines())).body[0].body


def generate_core(cached_path, destination, expected_cached_sha):
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
    _method(cls, "observation").body = _body(OBSERVATION_BODY)
    _method(cls, "_step_inputs").body = _body(STEP_BODY)
    for name in ("step", "step_target"):
        matches = [node for node in ast.walk(_method(cls, name))
                   if isinstance(node, ast.Attribute) and node.attr == OLD]
        require(len(matches) == 1, "Expected one projection op in " + name)
        matches[0].attr = NEW
    cls.name = "FusedRecurrentSwingCore"

    inverse = copy.deepcopy(transformed)
    inverse_cls = next(node for node in inverse.body if isinstance(node, ast.ClassDef))
    original_cls = next(node for node in original.body if isinstance(node, ast.ClassDef))
    inverse_cls.name = "CachedViewSwingCore"
    for name in ("observation", "_step_inputs"):
        _method(inverse_cls, name).body = copy.deepcopy(_method(original_cls, name).body)
    for name in ("step", "step_target"):
        matches = [node for node in ast.walk(_method(inverse_cls, name))
                   if isinstance(node, ast.Attribute) and node.attr == NEW]
        require(len(matches) == 1, "Projection inverse unavailable")
        matches[0].attr = OLD
    require(ast.dump(inverse, include_attributes=False) ==
            ast.dump(original, include_attributes=False),
            "Fusion changed undeclared controller code")
    ast.fix_missing_locations(transformed)
    generated = (ast.unparse(transformed) + "\n").encode()
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    name = "native_policy_full_fusion_" + sha(generated)[:16]
    spec = importlib.util.spec_from_file_location(name, destination)
    require(spec is not None and spec.loader is not None, "Generated import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.FusedRecurrentSwingCore, {
        "seed_cached_source_sha256": expected_cached_sha,
        "generated_source_sha256": sha(generated),
        "all_undeclared_ast_exact": True,
        "replaced_methods": ["observation", "_step_inputs"],
        "selected_projection": NEW + "::project",
        "hardware_opened": False, "output_allowed": False,
    }
