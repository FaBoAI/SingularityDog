# Cached Tensor view experiment

This explicit candidate reuses sixteen fixed-shape views of the existing
controller buffers. Arithmetic, float32/float64 conversions, input rejection,
clock updates and state writes remain the original pinned AST. The views alias
their owning buffers; they are not extra named buffers or copied state.

The original four reused source files, native operator, build, loader and
manifest source hashes remain unchanged. Nothing selects this candidate by
default. The local CPU comparison reduced the median forward CPU duration by
about 5.65%; it did not establish a maximum-duration improvement or a Jetson gain.

`generate_core(new_private_py_path)` writes new source outside Git and returns
`(controller_class, source_proof)`. It rejects a changed seed, an existing output
or an AST change beyond the declared view substitutions and constructor aliases.
The generated source must remain available while TorchScript inspects it.

An explicit file-only construction is:

```python
import copy
import torch
from native_policy_overnight import load_verified
from native_policy_overnight.contracts import OPTIONS, reference_policy
from native_policy_overnight.lean_swing_deployment import DeployableSwingPolicy
from native_policy_overnight.view_cache import generate_core, verify_aliases

baseline, baseline_proof = load_verified(
    manifest, expected_manifest_sha256=expected_manifest_sha256, bundle=bundle)
reference, _ = reference_policy(bundle)
core, view_proof = generate_core(new_private_source_path)
policy = DeployableSwingPolicy(copy.deepcopy(reference.actor), **OPTIONS).eval()
policy.controller = core(1, "cpu", **OPTIONS)
candidate = torch.jit.script(policy)
verify_aliases(candidate.controller)
```

Use the same inference mode, warmup, reset and six-vector inputs as the baseline.
Before treating a saved artifact as validated, compare targets, actor/observation,
all parameters/buffers, input ownership and rejection reasons, then save/reload
and repeat. `verify_aliases(reloaded.controller)` checks storage, dtype, device,
shape, stride and offset after reload.

The existing indexed resets and in-place buffer writes preserve aliases.
Replacing a buffer object or transferring its dtype/device may leave an
unregistered view stale. This candidate uses the existing fixed CPU model
precision; explicitly verify aliases after reload or any module transfer. It
does not silently rebuild caches or change model precision.

The legacy manifest/loader does not attest this generator or these extra Tensor
attributes. Do not label a cached-view model as a legacy artifact merely by
substituting its model hash. A file-only build/compare retains the existing
native library and creates only new generated source, model and comparison
evidence in a new private directory.

The explicit diagnostic loader supports the pinned build/compare v1 schema:

```python
from native_policy_overnight.view_cache.loader import load_file_only_verified

candidate, provenance = load_file_only_verified(
    variant_manifest, expected_sha256=variant_manifest_sha256,
    baseline_manifest=baseline_manifest, baseline_sha=baseline_manifest_sha256,
    bundle=bundle)
```

It requires `PASS_FILE_ONLY_COMPARE` and literal false hardware/output/approval/
live-verification/legacy-compatibility flags. The comparison JSON embeds its
validation report; its caller-supplied SHA pins the whole report, including 240
synthetic frames, 120 saved stateful calls, 26 known rejection reasons, exact
named state/parameters/targets, input ownership and saved/reload aliases.
The loader reproduces the generated source from the pinned seed, checks model
bytes, verifies all original source/bundle/ABI/library provenance through the
unchanged baseline loader, then checks native projection, aliases and reset
state after candidate reload. The recorded baseline path is informational;
loading uses the explicit baseline path/SHA and remains portable across copies.

The v1 build predates the diagnostic loader. It requires only the known
`__init__.py` and `generator.py` build-time helper pins. Additional entries in
that map are informational; the current loader source SHA and version
`native-view-cache-file-only-loader-v1` are returned separately. Import the
loader directly: the pinned helper `__init__.py` remains unchanged. Unknown
artifact versions or changed generator/build pins require an explicit new
audit. This loader changes no threads, clocks or output permissions, and the
candidate remains a diagnostic model without runtime/output approval.

Run the ordinary ownership/source tests without private fixtures. To include
the actual native save/reload test, explicitly provide these environment values:

```sh
SD_VIEW_CACHE_MANIFEST=/path/to/current/native/manifest.json
SD_VIEW_CACHE_MANIFEST_SHA256=PINNED_MANIFEST_SHA256
SD_VIEW_CACHE_BUNDLE=/path/to/pinned/policy/bundle
SD_VIEW_CACHE_RECORDS=/path/to/r6/gap800/records.json
SD_VIEW_CACHE_RECORDS_SHA256=PINNED_RECORDS_SHA256
```

Export those values and run the tests in a fresh process:

```sh
PYTHONPATH=runtime:runtime/experiments python3 -B -m unittest \
  native_policy_overnight.test_contracts native_policy_overnight.test_view_cache
```

The optional test checks 240 synthetic frames, 26 rejection cases and the twenty
saved frames replayed six times, with exact target/state/parameter parity and
input nonmutation across the baseline, candidate and saved/reloaded candidate.

To test the explicit loader against an actual saved candidate, additionally
export `SD_VIEW_CACHE_VARIANT_MANIFEST` and
`SD_VIEW_CACHE_VARIANT_MANIFEST_SHA256`, then include
`native_policy_overnight.test_view_cache_loader` in the test command. Its ordinary
tests require no private artifacts; the optional run verifies 120 recurrent
saved calls, rejection parity and independent model/reset ownership. The legacy
loader must reject the variant manifest.
