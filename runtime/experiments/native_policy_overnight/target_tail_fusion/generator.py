"""Private source-only replacement of step_target; inverse AST must be exact."""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import require, sha

_BODY = """
return torch.ops.sd_target_tail_fileonly_r1.target(
    raw, command, self.phase, self.elapsed, self.filters, self.yaw_filters,
    self.heading_error_rad, self.sensor_yaw_rate, self.previous_clipped,
    self.offsets, self.signs, self.anchor, self.origins, self.lower, self.upper,
    self.scale, self.safe_lower, self.safe_upper, self.sensor_q, self.sensor_up,
    self.output_anchor, self._cached_signs_row, self._cached_signs_expanded,
    self._cached_origins_row, self._cached_up_column)
"""


def transform(raw):
    original = ast.parse(raw)
    transformed = copy.deepcopy(original)
    classes = [node for node in transformed.body if isinstance(node, ast.ClassDef)]
    require(len(classes) == 1, "One pinned controller class required")
    cls = classes[0]
    require(cls.name == "FusedStepSwingCore", "Current scalar-step core required")
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name == "step_target"]
    steps = [node for node in cls.body if isinstance(node, ast.FunctionDef)
             and node.name == "_step_inputs"]
    require(len(methods) == len(steps) == 1, "Exact original target/step methods required")
    method = methods[0]
    original_body = copy.deepcopy(method.body)
    step = steps[0]
    original_decorators = copy.deepcopy(step.decorator_list)
    # The port no longer calls this method from forward. Preserve its original
    # serialized availability instead of letting scripting drop it as unused.
    step.decorator_list.append(ast.parse("torch.jit.export", mode="eval").body)
    method.body = ast.parse("def replacement():\n" +
        "".join("    " + line + "\n" for line in _BODY.strip().splitlines())).body[0].body
    cls.name = "FusedTargetTailSwingCore"
    inverse = copy.deepcopy(transformed)
    inverse_cls = next(node for node in inverse.body if isinstance(node, ast.ClassDef))
    inverse_cls.name = "FusedStepSwingCore"
    inverse_method = next(node for node in inverse_cls.body if isinstance(node, ast.FunctionDef)
                          and node.name == "step_target")
    inverse_method.body = original_body
    inverse_step = next(node for node in inverse_cls.body if isinstance(node, ast.FunctionDef)
                        and node.name == "_step_inputs")
    inverse_step.decorator_list = original_decorators
    require(ast.dump(inverse, include_attributes=False) == ast.dump(original, include_attributes=False),
            "Target-tail transformation changed undeclared controller code")
    ast.fix_missing_locations(transformed)
    return (ast.unparse(transformed) + "\n").encode()


def generate_core(step_path, destination, expected_sha):
    from ..model_call_fastpath.saved_input_profile import _output_path, _read
    step_path, destination = Path(step_path), _output_path(destination)
    require(destination.suffix == ".py", "New private Python source required")
    raw = _read(step_path, expected_sha)
    generated = transform(raw)
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    name = "native_policy_target_tail_" + sha(generated)[:16]
    require(name not in sys.modules, "Fresh generated source module required")
    spec = importlib.util.spec_from_file_location(name, destination)
    require(spec is not None and spec.loader is not None, "Generated source import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.FusedTargetTailSwingCore, {
        "seed_step_source_sha256": expected_sha, "generated_source_sha256": sha(generated),
        "all_undeclared_ast_exact": True, "replaced_method": "step_target",
        "original_step_inputs_serialized_method_preserved": True,
        "hardware_opened": False, "output_allowed": False, "approved_for_runtime": False}
