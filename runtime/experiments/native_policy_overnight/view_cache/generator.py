"""Generate an explicitly selected controller with reusable aliasing views.

The seed remains the original pinned controller. Only declared Tensor view
expressions change; reversing them must recover its entire AST. No arithmetic,
guard, dtype conversion or state write changes. Cached views are attributes,
not extra named buffers. The original buffer objects must retain their identity;
call verify_aliases after scripting/reload or any explicit module transfer.

This helper creates source only. It does not publish a loadable model manifest,
load libraries, set Torch threads, open devices or approve runtime output.
"""
import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys

from ..contracts import HERE, REUSED_PINS, pinned, require, sha


# (cache attribute, seed expression, constructor expression, owning buffer).
# None means the constructor uses the same expression as the seed method.
_VIEWS = (
    ("_cached_signs_expanded", "self.signs.unsqueeze(0).expand(self.num_envs,-1)", None, "signs"),
    ("_cached_signs_row", "self.signs.unsqueeze(0)", None, "signs"),
    ("_cached_origins_row", "self.origins.unsqueeze(0)", None, "origins"),
    ("_cached_anchor_x", "self.anchor[:,0].unsqueeze(0)", None, "anchor"),
    ("_cached_anchor_y", "self.anchor[:,1].unsqueeze(0)", None, "anchor"),
    ("_cached_anchor_xy", "self.anchor[:,:2].unsqueeze(0)", None, "anchor"),
    ("_cached_anchor_z", "self.anchor[:,2].unsqueeze(0)", None, "anchor"),
    ("_cached_phase_column", "self.phase.unsqueeze(1)", None, "phase"),
    ("_cached_filter_x", "self.filters[:,2,0].unsqueeze(1)", None, "filters"),
    ("_cached_filter_y", "self.filters[:,2,1].unsqueeze(1)", None, "filters"),
    ("_cached_filter_active", "self.filters[:,2,2].unsqueeze(1)", None, "filters"),
    ("_cached_yaw_filter", "self.yaw_filters[:,2].unsqueeze(1)", None, "yaw_filters"),
    ("_cached_up_column", "self.sensor_up.unsqueeze(1)", None, "sensor_up"),
    ("_cached_filters_flat", "self.filters.reshape(n,9)", "self.filters.reshape(self.num_envs,9)", "filters"),
    ("_cached_safe_lower", "self.safe_lower.reshape(1,4,3)", None, "safe_lower"),
    ("_cached_safe_upper", "self.safe_upper.reshape(1,4,3)", None, "safe_upper"),
)


def _dump(node):
    return ast.dump(node, include_attributes=False)


def _transform(raw):
    original = ast.parse(raw)
    transformed = copy.deepcopy(original)
    expressions = {_dump(ast.parse(expression, mode="eval").body): name
                   for name, expression, _, _ in _VIEWS}
    inverse_expressions = {name: ast.parse(expression, mode="eval").body
                           for name, expression, _, _ in _VIEWS}
    counts = {name: 0 for name, _, _, _ in _VIEWS}

    class Replace(ast.NodeTransformer):
        def visit_FunctionDef(self, node):
            # The new aliases are assigned only after all original buffers exist.
            return node if node.name == "__init__" else self.generic_visit(node)

        def visit(self, node):
            if isinstance(node, ast.expr) and _dump(node) in expressions:
                name = expressions[_dump(node)]
                counts[name] += 1
                return ast.copy_location(ast.Attribute(
                    value=ast.Name(id="self", ctx=ast.Load()), attr=name, ctx=ast.Load()), node)
            return super().visit(node)

    transformed = Replace().visit(transformed)
    cls = next(node for node in transformed.body if isinstance(node, ast.ClassDef))
    constructor = next(node for node in cls.body
                       if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    for name, expression, constructor_expression, _ in _VIEWS:
        require(counts[name] > 0, "Declared view is absent from the pinned controller: " + name)
        constructor.body.append(ast.parse(
            "self." + name + " = " + (constructor_expression or expression)).body[0])

    inverse = copy.deepcopy(transformed)
    inverse_cls = next(node for node in inverse.body if isinstance(node, ast.ClassDef))
    inverse_constructor = next(node for node in inverse_cls.body
                               if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    inverse_constructor.body = inverse_constructor.body[:-len(_VIEWS)]

    class Restore(ast.NodeTransformer):
        def visit_Attribute(self, node):
            if (isinstance(node.value, ast.Name) and node.value.id == "self"
                    and node.attr in inverse_expressions):
                return copy.deepcopy(inverse_expressions[node.attr])
            return self.generic_visit(node)

    require(_dump(Restore().visit(inverse)) == _dump(original),
            "View transformation changed the original AST")
    cls.name = "CachedViewSwingCore"
    ast.fix_missing_locations(transformed)
    return (ast.unparse(transformed) + "\n").encode(), counts


def generate_core(destination):
    """Return (controller class, source proof), writing new private source.

    The caller explicitly constructs the candidate with the same actor/options,
    then performs parity and saved/reload validation. The legacy loader must not
    be used to label this candidate as a legacy validated artifact.
    """
    path = Path(destination).expanduser().absolute()
    require(path.suffix == ".py" and path.parent.is_dir(),
            "New .py source needs an existing private parent directory")
    require(not path.exists(), "Generated controller source must be new")
    require(not any((parent / ".git").exists() for parent in (path.parent, *path.parents)),
            "Generated controller source must stay outside Git")
    seed_sha = REUSED_PINS["lean_swing_core.py"]
    raw = pinned(HERE / "lean_swing_core.py", seed_sha)
    generated, counts = _transform(raw)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(generated)
    generated_sha = sha(generated)
    module_name = "native_policy_cached_views_" + generated_sha[:16] + "_" + sha(str(path).encode())[:8]
    spec = importlib.util.spec_from_file_location(module_name, path)
    require(spec is not None and spec.loader is not None, "Generated controller import unavailable")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module.CachedViewSwingCore, {
        "schema": "native-policy-view-cache-source-v1",
        "seed_core_sha256": seed_sha, "generated_core_sha256": generated_sha,
        "generator_source_sha256": sha(Path(__file__).read_bytes()),
        "original_ast_exact_after_inverse_view_substitution": True,
        "declared_view_replacements": counts, "hardware_opened": False,
        "output_allowed": False, "approved_for_runtime": False,
    }


def verify_aliases(core):
    """Reject stale, copied or differently shaped cached views, including reload.

    Expected expressions come only from the fixed table above. Evaluating those
    Tensor view operations here checks storage, dtype, shape, strides and offset;
    it neither advances controller time nor changes a buffer.
    """
    for name, expression, constructor_expression, buffer_name in _VIEWS:
        owner = getattr(core, buffer_name)
        cached = getattr(core, name)
        expected = eval(constructor_expression or expression, {"__builtins__": {}}, {"self": core})
        require(cached.untyped_storage().data_ptr() == owner.untyped_storage().data_ptr(),
                "Cached view lost its buffer alias: " + name)
        require(cached.dtype == expected.dtype and cached.device == expected.device
                and cached.shape == expected.shape and cached.stride() == expected.stride()
                and cached.storage_offset() == expected.storage_offset()
                and cached.is_neg() == expected.is_neg() and cached.is_conj() == expected.is_conj()
                and cached.requires_grad == expected.requires_grad,
                "Cached view metadata differs: " + name)
    return True
