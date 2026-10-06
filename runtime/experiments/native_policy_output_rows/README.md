# Saved output-row conversion experiment

The current observer converts **98 values** (target12 + actor12 + observation74).
R37's 33 completed consume profiles measured a median 180.517 µs in this combined
conversion/validation section; that is host timing with instrumentation, not a
prediction of the speed available from a native implementation.

This isolated candidate converts and checks one row per C++ call. Three calls and
lazy actor/observation getters preserve the original order: target batch/length/
finite guard, then actor, then observation, then the existing Python target-range
guard. Combining all outputs eagerly would move model attribute access before a
target failure, so this candidate deliberately keeps the original ordering.

Only exact ordinary CPU float32 contiguous strided Tensor rows of shape (1,12) or
(1,74) use the native path. Negative/conjugate views, Tensor subclasses, custom
instance/class methods, function/dispatch modes, other dtypes/layouts/shapes and
duck objects use the exact pinned original `_tensor_row`/`_vector`/`finite` source.
Native code rechecks metadata before reading, widens float32 to float64 without
arithmetic, checks finite values and returns independently owned Python lists.
Signed zero, input bits, output list ownership, ValueError reasons and error/getter
ordering are covered by genuine compiled local tests. It preserves the original
range endpoints; it does not widen a physical limit or provide motor permission.

Default saved-profile CLI reads pinned report/records/raw-audit and source metadata,
but imports no Torch or library. Explicit `--execute-file-only` compiles one private
library and compares 501 saved rows using balanced ABBA order, recording every raw
wall/thread-CPU time. Comparison and mutation checks occur outside the timing.
This consumes already saved output values, loads no model, opens no devices and
does not measure inference, acquisition, Future waits or the live 20 ms loop.

Normal observer code and runtime selection are unchanged. Global namespace,
source/build/library ownership and observer error-class identity need separate
review before any production integration. Local timing is not target timing; no
performance gain or cause is inferred until the saved-only comparison is measured.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime python3 -B -m unittest \
  experiments.native_policy_output_rows.test_conversion -q

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime python3 -B -m \
  experiments.native_policy_output_rows.saved_profile \
  --report /absolute/original/report.json --report-sha256 EXPLICIT_SHA \
  --records /absolute/original/records.json --records-sha256 EXPLICIT_SHA \
  --raw-audit /absolute/audit.json --raw-audit-sha256 EXPLICIT_SHA \
  --output /absolute/fresh/private/conversion.json
```
