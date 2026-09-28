"""Pinned source/data contracts. Importing this module does not import torch."""
import ast
import copy
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PINS = {
    "model_149.pt": "7b06b035894c2696381a028d150609c67fef3a9a421cb001799152777e179434",
    "swing_core.py": "7a4f6ee4d1f4ae165356d91ab160fef5ac67d2958e8d717846ebc10be9d4811a",
    "swing_deployment.py": "8ea25946b3e2799d509afc32490a4efafd752d04a40b6b94a9839a576da59bfa",
}
REUSED_PINS = {
    "projection.cpp": "4d73a30cc08ad05fd9ad496079469cde8cd36636748a1544521d8f7c98ca94d1",
    "torch_projection.cpp": "16438adb456d1babecd70c81bf72febe3c496befd0789ade699836660cd94ebc",
    "lean_swing_core.py": "76f9af18a2d031dc690021a94ba5418ba308056c471e27a854b724805722018e",
    "lean_swing_deployment.py": "6f01774d5a1642502d7eacab7b43e18fa1827f4de0f1b56097be0d83635c5aba",
}
OPTIONS = dict(use_plane=True, period=.56, duty=.60, swing_height=.035,
               residual_tau=0., stance_widen_m=.02, heading_gain=.8,
               forward_command_limit=.46, hip_residual_scale=.27)
INPUT_KEYS = ("gyro_body_rad_s", "gravity_body_unit", "command", "q_model_rad",
              "dq_model_rad_s", "h_hypothesis12")
NATIVE_BLOCK = '''        # File-only experiment: native fixed-four-leg scan/refine, preserving source postchecks below.
        native=torch.ops.sd_projection_fileonly_r1.project(pre.contiguous(),start_feet.contiguous(),self.sensor_up.contiguous(),requested.contiguous(),alpha.contiguous(),uncapped.contiguous())
        solved=native[0]
        endpoint_q=native[1]
        lo=native[2]
        first_invalid=native[3]
        hi=native[4]
        applied=native[5]
        endpoint_domain=native[6]
        endpoint_ok=native[7]
        active=~native[8]
'''


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def pinned(path, expected):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "Regular nonsymlink file required: " + path.name)
    raw = path.read_bytes()
    require(sha(raw) == expected, "SHA mismatch: " + path.name)
    return raw


def strict_json(raw):
    def unique(pairs):
        out = {}
        for key, value in pairs:
            require(key not in out, "Duplicate JSON key")
            out[key] = value
        return out
    def reject(_):
        raise ValueError("Nonfinite JSON")
    return json.loads(raw, object_pairs_hook=unique, parse_constant=reject)


def verify_bundle(bundle):
    bundle = Path(bundle)
    return {name: sha(pinned(bundle / name, digest)) for name, digest in PINS.items()}


def source_hashes():
    files = [p for p in HERE.iterdir() if p.suffix in (".py", ".cpp") and not p.name.startswith("test_")]
    return {p.name: sha(p.read_bytes()) for p in sorted(files)}


def source_scope_check(bundle):
    """Prove native replacement boundaries and diagnostic-only lean changes."""
    verify_bundle(bundle)
    for name, digest in REUSED_PINS.items():
        pinned(HERE / name, digest)
    source = pinned(Path(bundle) / "swing_core.py", PINS["swing_core.py"]).decode()
    begin = source.index("        endpoint=start_feet+requested.unsqueeze(2)")
    end = source.index("        # This physical clip is an invariant backstop only.", begin)
    native = source[:begin] + NATIVE_BLOCK + source[end:]
    require(sha(native.encode()) == "ee46d6d6ba931a27814f015f81bea1cc3dd0054c0cd15b0006928c3d1692d4c2",
            "Native source reconstruction differs")
    def methods(text):
        cls = next(n for n in ast.parse(text).body if isinstance(n, ast.ClassDef))
        return {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    old, new = methods(native), methods((HERE / "lean_swing_core.py").read_text())
    dump = lambda n: ast.dump(n, include_attributes=False)
    require(all(dump(method) == dump(new[name]) for name, method in old.items()), "Old core method changed")
    require([dump(n) for n in old["_step_impl"].body[:-1]] ==
            [dump(n) for n in new["_step_inputs"].body[:-1]], "Step checks/state prefix changed")
    old_ret, new_ret = old["_step_impl"].body[-1].value, new["_step_inputs"].body[-1].value
    old_fields = {k.value: dump(v) for k, v in zip(old_ret.keys, old_ret.values)}
    fields = {k.value: dump(v) for k, v in zip(new_ret.keys, new_ret.values)}
    require(set(fields) == {"clipped_action", "feet_relative_to_base_m",
            "planned_center_clearance_m", "bootstrap_blend_after"} and
            all(value == old_fields[key] for key, value in fields.items()), "Intermediate rounding changed")
    prefix = copy.deepcopy(old["step"].body[:len(new["step_target"].body)-1])
    prefix[0].value.func.attr = "_step_inputs"
    require([dump(n) for n in prefix] == [dump(n) for n in new["step_target"].body[:-1]],
            "Projection/state/postcheck prefix changed")
    return dict(native_prefix_suffix_preserved=True, original_methods_preserved=True,
                lean_omits_diagnostics_only=True, state_writes_checks_rounding_preserved=True)


def reference_policy(bundle):
    """Use the same current loader as dual_policy_once, not a rewritten actor."""
    from singularitydog_hw import policy_shadow
    expected = HERE.parents[1] / "singularitydog_hw" / "policy_shadow.py"
    require(Path(policy_shadow.__file__).resolve() == expected, "Unexpected reference loader")
    verify_bundle(bundle)
    require(policy_shadow.SOURCE_HASHES == PINS and policy_shadow.OPTIONS == OPTIONS,
            "Current loader pins/options changed; re-audit this candidate")
    model, _ = policy_shadow.load_policy(bundle)
    return model, sha(expected.read_bytes())


def environment(torch):
    import platform
    return dict(system=platform.system(), machine=platform.machine(), torch_version=str(torch.__version__),
                cxx11_abi=bool(torch._C._GLIBCXX_USE_CXX11_ABI))


def member(directory, name):
    require(type(name) is str and Path(name).name == name and name not in ("", ".", ".."),
            "Artifact must be a filename within its manifest directory")
    return Path(directory) / name
