"""Generate an explicit private cached-view core with only _step_inputs swapped."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import require, sha

_BODY = """
native = torch.ops.sd_step_fileonly_r1.step(
    raw, command, self.phase, self.elapsed, self.filters, self.yaw_filters,
    self.heading_error_rad, self.sensor_yaw_rate, self.previous_clipped,
    self.offsets, self.signs, self.anchor, self.origins, self.lower, self.upper)
return {'clipped_action': native[0], 'feet_relative_to_base_m': native[1],
        'planned_center_clearance_m': native[2], 'bootstrap_blend_after': native[3]}
"""


def generate_step_core(cached_path, destination, expected_cached_sha):
    cached_path = Path(cached_path)
    destination = Path(destination)
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
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                  and node.name == "_step_inputs")
    method.body = ast.parse("def _replacement():\n" +
                            "".join("    " + line + "\n" for line in _BODY.strip().splitlines())).body[0].body
    cls.name = "FusedStepSwingCore"
    verify = copy.deepcopy(transformed)
    verify_cls = next(node for node in verify.body if isinstance(node, ast.ClassDef))
    verify_cls.name = "CachedViewSwingCore"
    verify_method = next(node for node in verify_cls.body if isinstance(node, ast.FunctionDef)
                         and node.name == "_step_inputs")
    original_method = next(node for node in
                           next(n for n in original.body if isinstance(n, ast.ClassDef)).body
                           if isinstance(node, ast.FunctionDef) and node.name == "_step_inputs")
    verify_method.body = copy.deepcopy(original_method.body)
    require(ast.dump(verify, include_attributes=False) ==
            ast.dump(original, include_attributes=False),
            "Step fusion changed undeclared controller code")
    ast.fix_missing_locations(transformed)
    generated = (ast.unparse(transformed) + "\n").encode()
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    name = "native_policy_fused_step_" + sha(generated)[:16]
    spec = importlib.util.spec_from_file_location(name, destination)
    require(spec is not None and spec.loader is not None, "Generated import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.FusedStepSwingCore, {
        "seed_cached_source_sha256": expected_cached_sha,
        "generated_source_sha256": sha(generated),
        "all_undeclared_ast_exact": True,
        "replaced_method": "_step_inputs",
        "hardware_opened": False,
        "output_allowed": False,
    }
