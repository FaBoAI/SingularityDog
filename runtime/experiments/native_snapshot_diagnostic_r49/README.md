R49 is an isolated source candidate for the separately selected, supported and
already disabled STOP diagnostic. It retains the frozen R47 FK model selection
and adds the R48 native snapshot copier. It grants no runtime, output or timing
admission approval. It does not contain output98 selection.

The required R47 manifest is the actual frozen `fk-stop-source-bundle-r47`
manifest, SHA
`1e08148e7426172f6123c699e8ce52f10663adb2ab6adaf08a6623048aca9a17`.
Test fixtures reproduce those exact immutable manifest bytes. A manifest with
the same thirteen member hashes but a different declared destination is rejected.

The ordinary observer SHA is
`21a18c24b19fc9eb38f2d9f172556f4928dec07a36eedc2d47a41b9cd990b0ad`.
Its copied source replaces only `from .event_snapshot import snapshot_event`
with the private native import. Every observer function and class has identical
AST; reversing that import recovers every original byte. This preserves
`_inputs`, snapshot digest, validation, tensor/model work, getter and failure
order, input ownership and output ownership. The copy shares the original
`ObserverError` and `_OWNERS` registry, including policies whose observer remains
alive after selection restoration.

The native source remains byte-identical to
`cafd55c92e4c61afabaf96ba7c02d6af70f02cb8581636658c7edb44c513b27a`;
the original builder is
`7299ae7294bcebbffa000cc80bcbdfeb9413342c61809b544f8f133f34790df3`.
Neither is compiled or loaded by default PLAN. No ordinary module is patched.
Only the separately generated benchmark's `observer` alias changes in the
execution scope. Its existing execution `try/finally` restores selection before
ordinary diagnostic cleanup. The selected bindings are checked before cleanup;
cleanup still restores the alias, private namespace and `_ACTIVE` after a guard
error. A primary error stays primary; cleanup verification is added as a note
for context-manager callers or a separate diagnostic report error.

The derived benchmark changes `main` and one import. All collection, motor,
freshness, voltage, fault and STOP functions remain byte/AST inverse verified;
the R47 FK loading branch is unchanged. Its fixed scope still comes from the
unmodified R47 support and loader. The copied child verifies the original K37
650-member inventory, the original K37-to-R47 derivation, and the new
R47-to-R49 and observer derivations. It executes the new snapshot package and
modules from authenticated source bytes directly, so a valid stale `.pyc`
cannot replace those sources. All R47 model/support dependencies remain pinned.

The source bundle has 23 members plus `manifest.json`. Native compilation uses
a fresh private directory outside the source bundle. The build receipt binds
source, builder, library, compiler result, CPython version and extension suffix.
Selection requires the current CPython ABI. The artifact manifest separately
binds the original observer, copied observer, native source, library and all
executing integration files. Unknown source bytes, mismatched source paths,
prebound private namespaces and nested selections are rejected.

CPython may keep a single-phase extension image cached after module entries are
removed. Repeated selections therefore check the builtin's owning module name,
pinned library path and builtin identity; they allow CPython's new wrapper for
that same owner. Restoration removes the private bindings and does not claim
that the shared-library image was unloaded. The intended diagnostic child runs
in a fresh interpreter.

The report retains the original cadence map as a dependency graph and identifies
the derived observer and benchmark under `executing_source_sha256`. It also
records the actual native/library/build references, shared error/owner checks,
selected-binding integrity, restoration and post-run source checks. PLAN labels
those identities as expected selection and records that selection/native load
have not happened. Qualification flags remain false on success and failure.

Generate a fresh source bundle with `PYTHONPATH=runtime/experiments`:

```sh
python3 -B -m native_snapshot_diagnostic_r49.snapshot_generate \
  --r47 /absolute/frozen-r47-source \
  --original-observer /absolute/K37/runtime/singularitydog_hw/policy_observer.py \
  --native-folder /absolute/native_snapshot_diagnostic_r49 \
  --output /absolute/fresh-source-bundle \
  --target-bundle /absolute/declared-target-source-bundle \
  --baseline-kit /absolute/declared-K37-kit
```

The generated manifest contains declared destination paths; for local checks,
`--target-bundle` must equal `--output`. Target packaging uses its target path.
Read and pin the manifest bytes after generation. Default native-build PLAN:

```sh
python3 -B -m native_snapshot_diagnostic_r49.snapshot_build \
  --source-bundle /absolute/source-bundle/manifest.json \
  --source-bundle-sha256 EXACT_SHA256 \
  --original-observer /absolute/K37/runtime/singularitydog_hw/policy_observer.py \
  --output /absolute/fresh-native-build
```

An explicitly requested file-only build adds `--execute` and
`--artifact-output /absolute/fresh-snapshot-artifact.json`. It opens no devices
and loads no extension. The separate diagnostic child also requires
`--snapshot-manifest` and `--snapshot-manifest-sha256`, alongside all existing
R47 CLI selections. `snapshot_build.rebind_fk_manifest` changes only the five
R47 integration source references; all model, library and evidence references
are copied unchanged. It returns data for a new manifest and never overwrites
the frozen R47 artifact. Actual target compilation, artifact validation and
hardware collection remain separate work.

Focused local tests:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/experiments python3 -B -m unittest \
  experiments.native_snapshot_diagnostic_r49.test_generation \
  experiments.native_snapshot_diagnostic_r49.test_selection -q
```

The tests explicitly build the pinned C source once in a temporary directory.
They use a fake policy and tensors: eight full observer ticks, 28 rejection and
freshness cases, mixed ownership in both orders, live ownership after restore,
the profiled measured/reused-buffer path, signed-zero and complete record/input
state equality, returned ownership, and caller immutability. Other cases cover
source inverse checks, native/alias import failures, deletion and replacement,
primary-plus-cleanup errors, unknown sources, descriptor closure and stale pyc.
The full-main PLAN test uses real snapshot artifact validation and stubs only
the existing FK artifact/provenance boundary. The child bootstrap test stubs the
baseline inventory/replay/collection boundary; it validates exact source
bootstrap and scheduler/path/module restoration. These tests do not establish
target ABI, target latency, full 20 ms timing, or controller qualification.
