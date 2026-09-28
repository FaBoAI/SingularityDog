# File-only model-call fusion experiments

These are opt-in CPU experiments against the pinned Trial28A/model_149 native
policy. The default runtime-output loader does not import this directory. The
disabled STOP-proxy benchmark can explicitly select the verified scalar
candidate through the pinned loader described below. Both candidates use the
original actor weights, observation, controller state names, native projection,
and six saved input vectors. They do not open CAN, QDD, IMU, or motor output.
The runner changes PyTorch's intra/inter-op thread count to one only within its
own process. It does not change CPU governor, affinity, frequency, or persistent
settings.

`actor.cpp` fuses the four ATen linear calls and three ELU calls into one custom
operator. The original `actor` module remains registered, including every
named parameter. Mac replay proved exact parity but no useful speed gain, so
this candidate is rejected.

`step.cpp` fuses the fixed one-environment `_step_inputs` path into one C++/ATen
operator. It updates the same registered recurrent buffers in place and returns
the same four intermediates consumed by the unchanged `step_target`. The
`step_generator.py` helper starts with the pinned cached-view source, changes
only `_step_inputs`, and proves all other AST nodes unchanged. The C++ path
uses pinned option constants; it is specific to `[1,74]` observation and
`[1,12]` action shapes. The saved-input and 240 synthetic checks establish
finite exact parity, not equivalence at every numerical branch boundary.

The runners require an existing CPU PyTorch installation, C++20 compiler,
pinned native manifest/bundle and 500 ordered saved records. Create a fresh
private output directory outside Git. For example, from the repository root:

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m \
  native_policy_overnight.model_call_fastpath.run_step \
  --baseline-manifest /private/native/manifest.json \
  --baseline-sha PINNED_MANIFEST_SHA256 \
  --bundle /private/policy-bundle \
  --records /private/500-cycle/records.json \
  --records-sha PINNED_RECORDS_SHA256 \
  --output /private/new-step-candidate
```

`run` selects the actor-only candidate with the same arguments. `run_step`
builds the C++ library for the current host, generates private source, scripts
and saves the candidate, reloads it, and compares native/cached/fused/reloaded
models. Every one of 500 sequential saved calls compares target, observation,
actor output, all named buffers and parameters exactly, and checks input
ownership. It then checks all 26 established rejection reasons and 240
synthetic frames spanning the command directions. Timings are interleaved,
single-thread CPU forward calls on the saved inputs, with both host wall and
thread CPU distributions. A PASS report does not make the model eligible for
runtime output. The private model and native library are host-specific.

The 2026-09-28 Mac result in `mac-validation-20260928.json` shows exact parity.
Against the already faster cached-view path, recurrent-step fusion changed
median model-call time from 0.442 to 0.405 ms in the latest paired 500-call
run. Wall p99 moved from 0.532 to 0.519 ms. Maximum durations varied across
runs because of host scheduling. A target-local Jetson C++ build completed in
22.4 seconds. The first four-model Jetson process reached its strict 30-second
timeout; an earlier short parity attempt lost SSH before its result could be
verified (`jetson-parity-attempt-20260928.json`). Those attempts are historical
and do not supply the Jetson result below.

`short_replay.py` splits target validation into independent `parity`,
`synthetic`, and `timing` phases. Each requires a caller-pinned native baseline,
500-record file, candidate model and C++ library; timing also requires the
cached-view manifest. Run each phase under `timeout 30s` on Jetson and write
separate private reports. The `parity` phase checks 500 saved recurrent calls,
all named state, observation, actor output, target and 26 rejection reasons;
`synthetic` checks 240 directional frames; `timing` alternates cached and fused
forward calls for 500 samples. The short parity and synthetic phases passed on
Mac against the saved candidate.

The recovered Jetson validation used the target Python 3.12 venv with PyTorch
2.14.0+cpu. The candidate is the target-local
`step_fused_cached_fileonly.pt` (SHA256
`7401bf982aff06c9394f36c26acf8c295fe1dd15826e6732e789e979e159113b`)
and `step_fileonly.so` (SHA256
`e05aecc5cd0168e67a4dd26f084eb9ae86ce55d3ce73bac1daa8564b2ee6ebe0`),
built from [`step.cpp`](step.cpp) (SHA256
`69b34790df67f00314c638199efba99abc0469969cfda52e31f4a48ad8a6332d`)
and [`step_generator.py`](step_generator.py). The short replay source SHA256 was
`8bcd64594bcc13215cb59a28e591deb8fcb3d6d5016965f8602b954b2e4cfe3a`.
The input was one pinned ordered 500-record file (SHA256
`617d85a521338d9108f10218336b1ce9e33ea21aa2aa711e76cd3a995dbc910d`).

The copied target reports are `jetson-parity-20260928.json`,
`jetson-synthetic-20260928.json`, and `jetson-timing-20260928.json`. Each phase
ran in a separate process with `timeout 30s`; no phase opened hardware or
enabled output. Parity passed 500 sequential saved calls with exact target,
observation, actor output, all named state and parameters, unchanged inputs,
and matching reasons for 26 rejected cases. Synthetic parity passed 240 frames
with recurrent resets at frames 0 and 125. All reported maximum errors are zero.

The alternating 500-call Jetson timing compared the pinned cached-view model
with the fused-step candidate, after ten untimed warmup calls per model:

| Model call (ms) | Cached view | Fused step | Difference |
| --- | ---: | ---: | ---: |
| Wall median | 2.224958 | 2.124489 | -0.100469 (-4.52%) |
| Wall p99 | 2.505422 | 2.411625 | -0.093797 (-3.74%) |
| Wall maximum | 5.335118 | 3.951808 | -1.383310 (one run) |
| Thread CPU median | 2.209568 | 2.110912 | -0.098656 (-4.47%) |
| Thread CPU maximum | 4.167584 | 3.829600 | -0.337984 (one run) |

The maximum values are scheduling-sensitive and are not an established tail
reduction. This isolated model-call result does not establish a full 20 ms cycle
gain, nor explain the approximately 4.1 ms live model-call spikes. The candidate
remains diagnostic and is not approved for runtime output or live 50 Hz.

To reproduce the three file-only phases on the same Jetson, use the exact paths,
hashes and command in the private diagnostic copy at
`/Users/akira/.codex/private-robotdog-kits/diagnostics-20260928/native-step-fusion-jetson-fileonly-20260928-r2/README.md`.
Each phase needs a fresh output filename and the target-local C++ library. The
required baseline and cached-view manifests are SHA-pinned by that command.

## Separate scalar C++ step candidate

[`step_scalar.cpp`](step_scalar.cpp) is a second, file-only implementation of
the same fixed-shape recurrent-step operator. It writes the registered state
buffers in place and allocates only four returned tensors. The heading-angle
update retains a small ATen chain to preserve exact numerical results; the
remaining filter, reference-foot and IK arithmetic uses scalar CPU loops.
`step_scalar.cpp` registers the **same** `sd_step_fileonly_r1::step` name as
`step.cpp`, so the two libraries must be tested in separate fresh processes.
This experiment reuses the SHA-pinned TorchScript graph above and substitutes
only the target-local scalar library. Neither library is part of the default
native-policy loader.

The first pure-scalar build failed exact parity at
`controller.heading_error_rad` and remains rejected; its Jetson report is in
the private diagnostic directory. After two targeted arithmetic repairs, the
final source SHA256 is
`0e47c261541aaf837a4b43484e5bbc696be695cc130e01a33fc6546c411b3ddc`
and Jetson `.so` SHA256 is
`3dfe86772586c4322de751f61fe4684fbe0c3ee1603181b0afafca4e66270457`.
The final `jetson-scalar-parity-20260928.json` and
`jetson-scalar-synthetic-20260928.json` passed the same strict 500 saved
sequential calls, all named state and parameters, 26 rejection reasons, 240
synthetic frames, and two resets. Every reported maximum error is zero.

The alternating 500-call Jetson timing in
`jetson-scalar-timing-20260928.json`, after ten untimed warmup calls per model,
was:

| Model call (ms) | Cached view | Scalar step | Difference |
| --- | ---: | ---: | ---: |
| Wall median | 2.270622 | 1.735172 | -0.535450 (-23.6%) |
| Wall p99 | 4.346675 | 3.356323 | -0.990352 (one run) |
| Wall maximum | 4.671874 | 4.206156 | -0.465718 (one run) |
| Thread CPU median | 2.251824 | 1.721104 | -0.530720 (-23.6%) |
| Thread CPU maximum | 3.846272 | 3.293728 | -0.552544 (one run) |

The median improvement exceeds 0.4 ms in this isolated file replay. The p99
and maxima are scheduling-sensitive and do not establish that the live
approximately 4.1 ms model-call spikes or the full 20 ms cycle are fixed.
That file-only comparison used no CAN, QDD, IMU, motor output, CPU setting,
or live benchmark. Subsequent live diagnostic integration is described below.

[`scalar_loader.py`](scalar_loader.py) provides a dedicated diagnostic-only
`load_file_only_verified(manifest, *, expected_sha256, baseline_manifest,
baseline_sha, bundle)` API. The target-local manifest at
`/home/jetson/singularitydog-logs/native-step-scalar-fileonly-20260928-r2/manifest.json`
has SHA256
`7ad6670368feb10d36bd62f594c7a53e89f28a5462fd1c3cf2292e2c1987f666`.
It pins the scalar source, unchanged shared sources, target model/library,
baseline and cached-view manifests, and all three reports. The loader checks
those sources, hashes, exact-parity report fields and false approval/output
flags before loading the native library. It then verifies target ABI, original
baseline provenance, graph operators, controller aliases and reset state.
A fresh Jetson process loaded this manifest successfully; seven file-only
loader tests cover rejected pins and incomplete evidence. The loader's
provenance retains `baseline_provenance`, `view_cache_provenance`, report hashes,
and `diagnostic_only: true`. It grants no runtime output approval.

The native pipeline benchmark accepts `--scalar-step-manifest` and
`--scalar-step-manifest-sha256` only with the native baseline, inference,
STOP-proxy mode and a bounded 500-cycle comparison (501 when the first cycle
is separately allowed). Verification runs before opening device sessions. The
scalar selection is mutually exclusive with a separate cached-view selection;
the manifest already pins that dependency. Four CLI tests cover invalid
combinations, validation failure before hardware access, selected provenance
and the first-plus-500-cycle plan. Live acquisition and STOP-proxy timing,
including unsuccessful repeat runs, is recorded in
[the angle and steady-cycle report](../../../../docs/angle-steady-cycle-fix-20260928.md).
Those runs neither enabled motors nor sent learned target angles.

The byte-identical reports, target artifacts, manifest and short reproduction
notes are saved outside Git at
`/Users/akira/.codex/private-robotdog-kits/diagnostics-20260928/native-step-scalar-fileonly-20260928/`.
