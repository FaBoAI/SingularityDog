# CPU checked model dispatch experiment

This package moves the original post-forward checks into a small C++ operator
called by a TorchScript wrapper. The original actor, controller, observation74,
actor12, target12, named state and reset methods remain in the inner model. The
operator preserves target → actor → observation → target-range error order,
finite checks, float32 values, signed zero and the original CAN permutation.

It supplies no active runtime selector, model-source approval, sensor/current
validation or Type1 qualification. The outer call fixture preserves the original
freshness, command and successful-call commit behavior; its sensor validation is
injected test data. The supported wrapper input is the pinned serialized model,
whose output attributes are ordinary state reads. Generic Python properties or
Tensor subclasses do not acquire an equivalence claim.

All command entries default to PLAN. Paths and SHA256 pins must be supplied
explicitly. PLAN reads files without importing Torch. Use a fresh build/output
directory; failed compiler output and replay failures are retained.

```sh
python3 -B build.py --output /absolute/fresh-build --source-sha256 CPP_SHA
python3 -B build.py --output /absolute/fresh-build --source-sha256 CPP_SHA --execute

python3 -B unit.py --runtime-root /absolute/source/kit/runtime \
  --original-source-sha256 ORIGINAL_LIVE_SOURCE_SHA \
  --build-record /absolute/fresh-build/build-record.json \
  --build-record-sha256 BUILD_RECORD_SHA --execute

python3 -B compare.py --inputs /absolute/inputs.json --inputs-sha256 INPUTS_SHA \
  --build-record /absolute/fresh-build/build-record.json \
  --build-record-sha256 BUILD_RECORD_SHA --output /absolute/fresh-replay
```

The builder uses C++20 for current LibTorch headers, with fast math disabled and
floating point contraction disabled. The replay configuration has this shape;
each reference contains an absolute path and its exact SHA256:

```json
{
  "schema": "experimental.live-checked-dispatch-file-inputs.v1",
  "runtime_root": "/absolute/source/kit/runtime",
  "source_manifest": {"path": "/absolute/source/manifest.json", "sha256": "SHA"},
  "original_live_source": {"path": "/absolute/source/kit/runtime/singularitydog_hw/policy_output_model.py", "sha256": "SHA"},
  "original_model": {"path": "/absolute/original.pt", "sha256": "SHA"},
  "candidate_model": null,
  "libraries": [{"path": "/absolute/original-operator.so", "sha256": "SHA"}],
  "records": {"path": "/absolute/original501/records.json", "sha256": "SHA"}
}
```

The source manifest's `files` map has SHA256, byte count and mode for each
relative source path, an exact `file_count`, and false `output_allowed` and
`approved_for_runtime` flags. Every source is checked before and after replay.
The manifest's schema is reported rather than relabelled as an active proof.

With `candidate_model` null, replay compares original and checked original.
Supplying the separately pinned R11 serialized model adds R11 and checked R11.
That candidate must have identical executable methods except the one explicit
target operator namespace, and all parameters/buffers must start independently
with identical bits. Its prior diagnostic qualification remains separate.

Add `--execute` to replay 501 saved inputs, compare every target/actor/observation
and named state bit, and measure four balanced blocks of 2004 calls per variant.
`--require-saved-reference-bit-parity` also rejects a selected reference whose
outputs differ from the saved machine's original outputs. Without that option,
the report retains the actual saved-match counts. No failed call is removed from
a passing report; a failed replay produces a failure artifact.

The frozen ARM CPU component on 2026-10-08 passed 15 guard/state tests and all
501 inputs for four variants. Original median was 1700.593 µs; checked R11 was
1486.5705 µs, a 214.0225 µs reduction (12.585%). These are CPU component times,
excluding sensor validation, CAN and the whole 20 ms cycle. Report SHA256:
`00e97e77bc414bece7a289c38c2d648f3124dcd0a0de9da1818f39cedf0234e6`.
The portable files here require their own source-specific build and replay;
those prior results do not qualify this source for live output.
