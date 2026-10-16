"""Source-bound 30 Hz candidate loader; file inspection and CPU inference only.

The seven artifacts describe the 2026-10-13 release.  Their hashes are trusted
release pins, not hashes learned from whichever files happen to be supplied.
Paths may change when the bundle is copied to Jetson.  The historical GPU path
inside the training report is evidence text; it is never opened or executed.

Neither a successful plan nor a CPU load authorizes motor output.  A caller
must separately bind current calibration, boot/power, transport and physical
conditions to a fresh output profile.  Native/scalar/FK/checked 50 Hz kernels
have no entry point in this loader.  h remains an explicit, unmeasured input.
"""
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import re
import stat

SCHEMA = "singularitydog.policy-model-30hz.v1"
POLICY_HZ = 30
POLICY_DT_S = 1. / 30.
PHYSICS_DT_S = 1. / 480.
DECIMATION = 16
CAN_ORDER = (6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7)
PARENT_SHA256 = "7b06b035894c2696381a028d150609c67fef3a9a421cb001799152777e179434"
ARTIFACT_FILES = {
    "controller": "model/policy_controller74_30hz.pt",
    "actor": "model/actor74_30hz.pt",
    "control_contract": "model/control_contract30.json",
    "export": "model/export.json",
    "train_completion": "checks/train-completion.json",
    "source_manifest": "source/source_manifest30.json",
    "checkpoint": "checkpoint/verified_model_499.pt",
}
RELEASE_SHA256 = {
    "controller": "37161d7ff58cfca626b4a3a35ff5f84c294d2478584309b436726f3a0768f682",
    "actor": "9dc069245d3fe28928f2b22a2240422eda4a6fc03aae66ba4306b050699dad6c",
    "control_contract": "c0270739e6093b4b5bc2e5edd4d3adf29a32b37c25606d9afa545a547304bd31",
    "export": "f03e405c734ec661c0594f15802e407ef925841358b56fdd42c70f094d61e6b6",
    "train_completion": "7ba5715cee4e8a5d1293baa857727a5be2746d40100345a2ea9188865145a443",
    "source_manifest": "72fe382545ec8d04e1c25baf07162f08f1a22a3af6a094253ac8ca8506d9be6c",
    "checkpoint": "771a77e950f3fe7ece76c2c0e2a8967161fe6285468455468cba7c6f433891bb",
}
_SOURCE_NAMES = frozenset(("README.md", "environment30.py", "export30.py",
    "runtime_common30.py", "seal30.py", "smoke30.py", "stage_driver30.py",
    "swing_core.py", "tests/test_stage_driver30.py", "tests/test_timing30.py",
    "tests/test_training30.py", "timing_contract.py", "train30.py"))
_ACTOR_SHAPES = {"0.weight": (128, 74), "0.bias": (128,),
    "2.weight": (128, 128), "2.bias": (128,), "4.weight": (64, 128),
    "4.bias": (64,), "6.weight": (12, 64), "6.bias": (12,)}
_MAX_FILE_BYTES = 2 * 1024 * 1024


def _need(value, message):
    if not value:
        raise ValueError(message)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        _need(key not in result, "Duplicate JSON key")
        result[key] = value
    return result


def _json(raw):
    def bad(value):
        raise ValueError("Nonfinite JSON number: " + value)
    try:
        result = json.loads(raw, object_pairs_hook=_pairs, parse_constant=bad)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("Invalid UTF-8 JSON artifact") from error
    _need(type(result) is dict, "JSON artifact must be an object")
    return result


def _read(path, expected, label):
    _need(type(expected) is str and re.fullmatch(r"[0-9a-f]{64}", expected),
          "Explicit SHA256 required: " + label)
    _need(type(path) in (str, Path) or isinstance(path, Path),
          "File path required: " + label)
    path = Path(path)
    _need(path.is_absolute() and not path.is_symlink(),
          "Absolute regular file required: " + label)
    before = path.stat()
    _need(stat.S_ISREG(before.st_mode) and 0 < before.st_size <= _MAX_FILE_BYTES,
          "Artifact file size/type invalid: " + label)
    with path.open("rb") as stream:
        raw = stream.read(_MAX_FILE_BYTES + 1)
    after = path.stat()
    signature = lambda item: (item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns)
    _need(signature(before) == signature(after) and len(raw) == before.st_size,
          "Artifact changed while reading: " + label)
    _need(_sha(raw) == expected, "Artifact SHA256 differs: " + label)
    return raw


def bundle_artifacts(bundle):
    """Make explicit release pins for a relocated distribution, without reads."""
    root = Path(bundle).expanduser().resolve()
    return {name: {"path": str(root / relative), "sha256": RELEASE_SHA256[name]}
            for name, relative in ARTIFACT_FILES.items()}


def _snapshot(artifacts):
    _need(type(artifacts) is dict and artifacts.keys() == ARTIFACT_FILES.keys(),
          "Require exactly seven 30Hz artifacts; no legacy backend selection")
    raw, records = {}, {}
    for name in ARTIFACT_FILES:
        record = artifacts[name]
        _need(type(record) is dict and record.keys() == {"path", "sha256"},
              "Artifact requires only path and SHA256: " + name)
        _need(record["sha256"] == RELEASE_SHA256[name],
              "Artifact is not the selected 30Hz release: " + name)
        raw[name] = _read(record["path"], record["sha256"], name)
        records[name] = {"path": str(Path(record["path"])),
                         "sha256": record["sha256"]}
    docs = {name: _json(raw[name]) for name in
            ("control_contract", "export", "train_completion", "source_manifest")}
    _validate_documents(docs)
    manifest = docs["source_manifest"]
    _need(manifest.keys() == _SOURCE_NAMES, "30Hz source manifest is incomplete")
    source_root = Path(records["source_manifest"]["path"]).parent.resolve()
    for name, digest in manifest.items():
        relative = PurePosixPath(name)
        _need(not relative.is_absolute() and ".." not in relative.parts,
              "Source member escapes bundle")
        source = source_root / name
        _need(source.resolve().is_relative_to(source_root), "Source member escapes bundle")
        _read(source, digest, "source/" + name)
    return raw, records, docs


def _validate_documents(docs):
    """Check report relationships as well as the external immutable file pins."""
    contract, export, train = (docs[name] for name in
                              ("control_contract", "export", "train_completion"))
    training = contract.get("training_contract", {})
    _need(training == export.get("training_contract"), "Training contracts differ")
    for key, value in {"seed": 42, "num_envs": 256, "rollout_steps": 32,
        "updates": 500, "actor_dim": 74, "critic_dim": 89, "action_dim": 12,
        "policy_dt_s": POLICY_DT_S, "physics_dt_s": PHYSICS_DT_S,
        "decimation": DECIMATION, "parent_iteration": 149,
        "parent_checkpoint_sha256": PARENT_SHA256,
        "initialization": "PARENT_ACTOR_MEAN_AND_LEARNED_STD_FRESH_CRITIC_AND_OPTIMIZER",
        "strict_resume": False, "automatic_retry": False,
        "hardware_output_allowed": False}.items():
        _need(type(training.get(key)) is type(value) and training[key] == value,
              "30Hz training contract differs: " + key)
    timing = contract.get("timing_contract", {})
    for key, value in {"schema": "fabo.walking30.timing.v1", "policy_hz": 30,
        "policy_dt_s": POLICY_DT_S, "physics_hz": 480, "physics_dt_s": PHYSICS_DT_S,
        "decimation": 16, "filter_decay": math.exp(-POLICY_DT_S / .15),
        "filter_tau_s": .15, "bootstrap_s": 2., "gait_period_s": .56,
        "duty": .6, "previous_physics_qualification_transferred": False,
        "geometry_mass_inertia_and_drive_limits_changed": False,
        "hardware_output_allowed": False}.items():
        _need(type(timing.get(key)) is type(value) and timing[key] == value,
              "30Hz timing contract differs: " + key)
    _need(contract.get("controller_options") == dict(use_plane=True, period=.56,
        duty=.6, swing_height=.035, residual_tau=0., stance_widen_m=.02,
        heading_gain=.8, forward_command_limit=.46, hip_residual_scale=.27),
        "Controller options differ")
    _need(export.get("schema") == "fabo.l23.30hz-export.v1" and
          export.get("status") == "EXPORTED_NEW30HZ_TRAINED_POLICY",
          "Completed new30Hz export required")
    for document in (contract, export, train):
        _need(document.get("hardware_output_allowed") is False,
              "Model evidence must not grant hardware output")
        _need(document.get("parent_checkpoint_sha256") == PARENT_SHA256 and
              document.get("checkpoint_sha256") == RELEASE_SHA256["checkpoint"],
              "Checkpoint lineage differs")
    _need(export.get("source_manifest_sha256") == train.get("source_manifest_sha256") ==
          RELEASE_SHA256["source_manifest"], "Source closure differs")
    _need(export.get("training_completion_sha256") == RELEASE_SHA256["train_completion"],
          "Training completion closure differs")
    _need(export.get("files") == {ARTIFACT_FILES[name].split("/")[-1]: RELEASE_SHA256[name]
          for name in ("controller", "actor", "control_contract")}, "Export file closure differs")
    _need(export.get("checkpoint_updates") == 500 and export.get("checkpoint_iteration") == 499
          and export.get("parent_weights_only_transfer") is True and
          export.get("critic_or_optimizer_reused") is False and
          export.get("previous_50hz_model_or_hardware_qualification_transferred") is False,
          "Old model/optimizer qualification cannot transfer")
    parity = export.get("parity", {})
    _need(parity.get("status") == "CPU_EAGER_TORCHSCRIPT_COMPLETE_STATE_PARITY_PASS"
          and parity.get("steps") == 420 and parity.get("state_buffer_comparisons") == 8400
          and parity.get("output_is_canonical_joint_target") is True
          and all(parity.get(key) == 0. for key in ("max_target_error_rad",
                  "max_observation_error", "max_actor_output_error")), "Export parity incomplete")
    _need(train.get("status") == "TRAIN30_COMPLETED" and train.get("stage") == "train",
          "Actual completed training required")
    for key, value in {"updates": 500, "planned_updates": 500, "policy_steps": 16000,
        "physics_updates": 256000, "transitions": 4096000, "optimizer_states": 17,
        "optimizer_steps_per_parameter": 10000, "final_iteration": 499,
        "checkpoint_status": "VERIFIED_NEW30HZ_CHECKPOINT",
        "runtime_physics_validated": True, "audit_hooks_restored": True,
        "policy_dt_s": POLICY_DT_S, "physics_dt_s": PHYSICS_DT_S,
        "decimation": DECIMATION}.items():
        _need(type(train.get(key)) is type(value) and train[key] == value,
              "Actual training completion differs: " + key)
    audit = train.get("audit", {})
    _need(audit.get("all_real_compute_calls_checked") is True and
          audit.get("physical_dt_s") == PHYSICS_DT_S and
          audit.get("policy_dt_s") == POLICY_DT_S and
          audit.get("physics_updates") == 256000 and
          audit.get("completed_policy_steps") == 16000 and
          audit.get("compute_calls") == 256001, "Actual480Hz audit incomplete")
    cleanup = train.get("cleanup", {})
    _need(cleanup.get("audit_hooks_restored") is True and
          cleanup.get("environment_close_returned") is True and
          cleanup.get("application_close_attempted") is True, "Training cleanup incomplete")


def _plan(records, docs):
    return dict(schema=SCHEMA, status="VERIFIED_FILE_PLAN_NO_MODEL_EXECUTION",
        artifacts=records, policy_hz=POLICY_HZ, policy_dt_s=POLICY_DT_S,
        physics_dt_s=PHYSICS_DT_S, decimation=DECIMATION,
        model_can_order=list(CAN_ORDER), input_shapes=[[1, n] for n in (3, 3, 3, 12, 12, 12)],
        input_dtype="cpu_float32", output_shape=[1, 12], output_semantics="absolute_model_joint_target_rad",
        actor_input_dim=74, actor_mean_parameters=35148, learned_std_parameters=12,
        source_members_verified=len(docs["source_manifest"]), training_updates=500,
        h_semantics="caller_supplied_unmeasured_load_history_or_explicit_diagnostic_hypothesis",
        cpu_model_executed=False, model_artifact_grants_output=False,
        output_allowed=False, approved_for_runtime=False, hardware_opened=False,
        previous_50hz_qualification_transferred=False,
        evaluation_all_intervals_completed=docs["export"].get("evaluation_all_intervals_completed"),
        walking_acceptance=False)


def plan(artifacts):
    """Verify file/hash/lineage closure without importing Torch or executing code."""
    _, records, docs = _snapshot(artifacts)
    return _plan(records, docs)


def validate_plan(value):
    """Validate an inline plan without file/Torch access; does not reprove files.

    Contract builders may freeze this returned JSON tree.  The foreground
    runner must still call load() to verify that the named files exist and
    retain these hashes on the current machine.
    """
    _need(type(value) is dict and value.get("schema") == SCHEMA and
          value.get("status") == "VERIFIED_FILE_PLAN_NO_MODEL_EXECUTION",
          "Expected unexecuted 30Hz file plan")
    artifacts = value.get("artifacts")
    _need(type(artifacts) is dict and artifacts.keys() == ARTIFACT_FILES.keys(),
          "Inline30Hz artifact set differs")
    for name, record in artifacts.items():
        _need(type(record) is dict and record.keys() == {"path", "sha256"} and
              type(record["path"]) is str and Path(record["path"]).is_absolute() and
              record["sha256"] == RELEASE_SHA256[name], "Inline30Hz artifact pin differs: " + name)
    for key, expected in {"policy_hz": POLICY_HZ, "policy_dt_s": POLICY_DT_S,
        "physics_dt_s": PHYSICS_DT_S, "decimation": DECIMATION,
        "model_can_order": list(CAN_ORDER), "source_members_verified": 13,
        "training_updates": 500, "actor_input_dim": 74, "actor_mean_parameters": 35148,
        "learned_std_parameters": 12, "input_shapes": [[1, n] for n in (3, 3, 3, 12, 12, 12)],
        "input_dtype": "cpu_float32", "output_shape": [1, 12],
        "output_semantics": "absolute_model_joint_target_rad",
        "h_semantics": "caller_supplied_unmeasured_load_history_or_explicit_diagnostic_hypothesis",
        "cpu_model_executed": False, "model_artifact_grants_output": False,
        "output_allowed": False, "approved_for_runtime": False, "hardware_opened": False,
        "previous_50hz_qualification_transferred": False, "walking_acceptance": False}.items():
        _need(type(value.get(key)) is type(expected) and value[key] == expected,
              "Inline30Hz plan differs: " + key)
    _need(type(value.get("evaluation_all_intervals_completed")) is bool,
          "Inline30Hz evaluation status missing")
    # The plan has a fixed shallow JSON schema.  Copy it to avoid retaining
    # mutable dictionaries owned by a contract-builder caller.
    expected_keys = set(_plan(artifacts, {"source_manifest": dict.fromkeys(_SOURCE_NAMES),
                                         "export": {"evaluation_all_intervals_completed": False}}))
    _need(value.keys() == expected_keys, "Unknown inline30Hz plan fields")
    return _json(json.dumps(value, allow_nan=False, sort_keys=True).encode())


def _validate_clock(policy):
    _need(policy.policy_dt_s == POLICY_DT_S and policy.controller.dt == POLICY_DT_S,
          "Loaded controller clock differs from 30Hz")
    for name, value in {"num_envs": 1, "observation_dim": 74, "action_dim": 12,
        "filter_decay": math.exp(-POLICY_DT_S / .15), "filter_tau": .15,
        "bootstrap_s": 2., "period": .56, "duty": .6, "heading_gain": .8,
        "forward_command_limit": .46, "hip_residual_scale": .27}.items():
        _need(getattr(policy.controller, name, None) == value,
              "Loaded controller constant differs: " + name)


def _validate_cpu(policy, actor, checkpoint, torch):
    _validate_clock(policy)
    model_params, actor_params = dict(policy.actor.named_parameters()), dict(actor.named_parameters())
    _need(model_params.keys() == actor_params.keys() == _ACTOR_SHAPES.keys(),
          "Loaded actor74 shape schema differs")
    state = checkpoint.get("actor_state_dict", {})
    _need(state.keys() == {"distribution.std_param", *("mlp." + k for k in _ACTOR_SHAPES)},
          "Checkpoint actor74 schema differs")
    for name, shape in _ACTOR_SHAPES.items():
        for value in (model_params[name], actor_params[name], state["mlp." + name]):
            _need(value.device.type == "cpu" and value.dtype == torch.float32 and
                  tuple(value.shape) == shape and bool(torch.isfinite(value).all()),
                  "Actor weights are not finite CPUfloat32: " + name)
        _need(torch.equal(model_params[name], actor_params[name]) and
              torch.equal(model_params[name], state["mlp." + name]), "Actor/checkpoint weights differ")
    std = state["distribution.std_param"]
    _need(tuple(std.shape) == (12,) and bool(torch.isfinite(std).all()) and bool((std > 0).all()),
          "Learned standard deviation is invalid")
    infos = checkpoint.get("infos", {})
    _need(checkpoint.get("iter") == 499 and infos.get("completed_new_updates") == 500
          and infos.get("parent_checkpoint_sha256") == PARENT_SHA256 and
          infos.get("training_contract", {}).get("policy_dt_s") == POLICY_DT_S,
          "Checkpoint actual500/30Hz metadata differs")
    buffers = dict(policy.controller.named_buffers())
    _need(all(value.device.type == "cpu" and bool(torch.isfinite(value).all())
              for value in buffers.values()), "Controller buffers must be finite CPU tensors")
    lower, upper = torch.tensor([-.5, -.9, -2.2] * 4, dtype=torch.float64), \
                   torch.tensor([.5, 1.2, -.08] * 4, dtype=torch.float64)
    _need(torch.equal(policy.controller.lower, lower) and torch.equal(policy.controller.upper, upper),
          "Loaded model joint limits differ")
    ids = torch.tensor([0], dtype=torch.long)
    policy.reset(ids)
    initial = {name: value.clone() for name, value in policy.controller.named_buffers()}
    _need(all(bool((initial[name] == 0).all()) for name in ("phase", "elapsed", "filters",
          "yaw_filters", "heading_error_rad", "sensor_yaw_rate", "previous_clipped")),
          "Controller reset retained old recurrent state")
    _need(torch.equal(initial["sensor_q"], policy.controller.nominal.reshape(1, 12)) and
          torch.equal(initial["sensor_up"], torch.tensor([[0., 0., 1.]], dtype=torch.float64)),
          "Controller reset retained sensor state")
    _need(bool((policy.last_observation == 0).all()) and bool((policy.last_actor_output == 0).all()),
          "Controller reset retained last output")
    commands = ((.05, 0., 0.), (0., 0., .05), (0., 0., 0.))
    for tick in range(6):
        args = (torch.tensor([[.01, .02, .03]], dtype=torch.float32),
                torch.tensor([[0., 0., -1.]], dtype=torch.float32),
                torch.tensor([commands[tick // 2]], dtype=torch.float32),
                policy.controller.nominal.float().reshape(1, 12).clone(),
                torch.full((1, 12), .01, dtype=torch.float32),
                torch.full((1, 12), .02, dtype=torch.float32))
        before = tuple(value.clone() for value in args)
        target = policy(*args)
        for value, count in ((target, 12), (policy.last_actor_output, 12), (policy.last_observation, 74)):
            _need(value.device.type == "cpu" and value.dtype == torch.float32 and
                  tuple(value.shape) == (1, count) and bool(torch.isfinite(value).all()),
                  "Loaded model six-input ABI differs")
        _need(bool((target >= lower.float()).all() & (target <= upper.float()).all()),
              "Loaded target outside model range")
        _need(torch.equal(actor(policy.last_observation), policy.last_actor_output),
              "Actor and controller observation/output differ")
        _need(all(torch.equal(a, b) for a, b in zip(args, before)), "Model mutated caller inputs")
        _need(abs(float(policy.controller.elapsed[0]) - (tick + 1) * POLICY_DT_S) < 1e-14 and
              abs(float(policy.controller.phase[0]) - ((tick + 1) * POLICY_DT_S / .56) % 1.) < 1e-14,
              "Controller recurrent clock differs")
    policy.reset(ids)
    current = dict(policy.controller.named_buffers())
    _need(current.keys() == initial.keys() and all(torch.equal(value, current[name])
          for name, value in initial.items()), "Full controller state did not reset")
    _need(bool((policy.last_observation == 0).all()) and bool((policy.last_actor_output == 0).all()),
          "Telemetry state did not reset")
    return dict(status="CPU_ARTIFACT_ABI_STATE_CLOCK_PASS", calls=6,
                all_named_controller_buffers_reset=True, actor_checkpoint_weights_exact=True,
                actor_matches_last_observation=True, input_mutation=False, hardware_acceptance=False)


def load(artifacts, *, execute_cpu=False):
    """Explicit CPU-only load returning (fresh ScriptModule, unapproved proof).

    The caller owns one model instance per run and supplies finite CPUfloat32
    rows in the declared six-input order.  Warmup must be followed by reset;
    normal ticks/command changes must retain state.  No external model injection
    or fallback to another clock/backend is provided.
    """
    _need(execute_cpu is True, "Use plan() or explicitly request execute_cpu=True")
    raw, records, docs = _snapshot(artifacts)
    import torch
    policy = torch.jit.load(io.BytesIO(raw["controller"]), map_location="cpu").eval()
    actor = torch.jit.load(io.BytesIO(raw["actor"]), map_location="cpu").eval()
    checkpoint = torch.load(io.BytesIO(raw["checkpoint"]), map_location="cpu", weights_only=True)
    with torch.inference_mode():
        proof = _validate_cpu(policy, actor, checkpoint, torch)
    # Bytes used above belong to the verified snapshot.  Recheck the files to
    # reject a package swap during setup rather than reporting current paths
    # for an earlier, already replaced snapshot.
    _snapshot(artifacts)
    provenance = _plan(records, docs)
    provenance.update(status="VERIFIED_CPU_LOADED_30HZ_CANDIDATE", cpu_model_executed=True,
                      cpu_validation=proof, torch_version=str(torch.__version__))
    return policy, provenance
