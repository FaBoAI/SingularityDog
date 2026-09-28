"""Replace only observation in a generated private cached-view controller."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import require, sha

_BODY = """
return torch.ops.sd_observation_fileonly_r1.observe(
    gyro, gravity, command, q, dq, exposure_h, self.nominal,
    self.previous_clipped, self.filters, self.phase, self.elapsed,
    self.yaw_filters, self.heading_error_rad, self.sensor_q,
    self.sensor_up, self.sensor_yaw_rate)
"""


def generate_observation_core(cached_path, destination, expected_cached_sha):
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
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                  and node.name == "observation")
    original_method = copy.deepcopy(method)
    method.body = ast.parse("def replacement():\n" +
                            "".join("    " + line + "\n" for line in _BODY.strip().splitlines())).body[0].body
    cls.name = "FusedObservationSwingCore"
    inverse = copy.deepcopy(transformed)
    inverse_cls = next(node for node in inverse.body if isinstance(node, ast.ClassDef))
    inverse_cls.name = "CachedViewSwingCore"
    inverse_method = next(node for node in inverse_cls.body if isinstance(node, ast.FunctionDef)
                          and node.name == "observation")
    inverse_method.body = original_method.body
    require(ast.dump(inverse, include_attributes=False) ==
            ast.dump(original, include_attributes=False),
            "Observation fusion changed undeclared controller code")
    ast.fix_missing_locations(transformed)
    generated = (ast.unparse(transformed) + "\n").encode()
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    name = "native_policy_fused_observation_" + sha(generated)[:16]
    spec = importlib.util.spec_from_file_location(name, destination)
    require(spec is not None and spec.loader is not None, "Generated import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.FusedObservationSwingCore, {
        "seed_cached_source_sha256": expected_cached_sha,
        "generated_source_sha256": sha(generated),
        "all_undeclared_ast_exact": True,
        "replaced_method": "observation",
        "hardware_opened": False,
        "output_allowed": False,
    }
