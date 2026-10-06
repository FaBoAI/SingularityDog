# File-only ATen target-tail candidate

This candidate replaces only the current scalar controller's `step_target`
method body. The C++ op calls the existing scalar op and original projection
op, with ATen FK/IK, residual contraction, plane/clearance calculations and
the original postchecks in the original order. The scalar's float32
intermediate rounding remains. Actor, observation, reset, registered buffers,
cached views and the other controller method bodies remain unchanged.
`_step_inputs` is explicitly exported so scripting retains its original
serialized availability when forward no longer calls it directly.

This is an experiment. It has no transport or motor API, no runtime selector,
and no live controller qualification. Passing CPU model equivalence does not
prove that an acquisition/output cycle meets 20ms.

`saved_profile` defaults to PLAN, reading pinned source/data bytes without
importing Torch, loading models/libraries, or compiling. Execution requires
`--execute-file-only`, the existing verified scalar/baseline manifests, their
SHA256 values, the original policy bundle, and a fresh output path outside
Git. The original 501-cycle report/records and independent raw audit must
also be pinned. Source inventory includes the protected original sources
and the new candidate sources. An optional precompiled target library needs
an explicit SHA; by default it is compiled privately from pinned `target.cpp`
with `-ffp-contract=off` and no fast math.

Before timing, both independently owned models must reproduce every original
saved target/actor/observation float32 bit and every named state bit, preserve
input bits including signed zero, match 26 rejection reasons and rejection
partial states, and pass 240 zero/nonzero command cases with resets. New
controller buffers are constructed fresh; cached-view storage/stride/offset
is checked before and after serialization. Four timing blocks use leading
orders AB, BA, BA, AB with per-cycle alternation, raw wall/thread-CPU arrays
and call-order summaries. Profiling is separate from timing.

Local kernel tests compile the actual original scalar/projection and new
target kernels in a fresh process, using a deterministic test actor. They
check 501 recurrent cases, the 26 rejection cases, 240 synthetic cases and
44 clip/bootstrap/phase/command/nonfinite/reference-margin probes. This
local test is distinct from exact saved Jetson-output parity. The existing
scalar's saved output can differ between platforms or Torch/BLAS versions;
the target runner rejects such a difference rather than substituting output.

Run focused tests from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime/experiments:runtime python3 -B -m unittest native_policy_overnight.target_tail_fusion.test_saved_profile -q
```

No performance gain is asserted until a fresh, pinned target file-only run
passes all parity checks and its timing/raw restoration evidence is reviewed.
