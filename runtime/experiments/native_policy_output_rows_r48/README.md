# Output-row CPython bridge, isolated r48 candidate

This candidate follows the rejected saved-output98 experiment in
`../native_policy_output_rows/`. That earlier candidate passed its Jetson tests
but measured 68.963 → 88.771 µs for the original → candidate wall medians on ARM.
Those frozen sources and ordinary observer/runtime selection are unchanged.

Local attribution was performed before this implementation. On this Mac, a
74-value row spent approximately 0.555 µs in its eleven class-attribute guards,
1.174 µs in overall eligibility, and 1.434 µs in dispatcher plus kernel. These
overlapping component timings are local evidence, not ARM cause attribution.

r48 moves the existing per-row class, exact Tensor type, empty instance dict,
mode, and metadata checks into a CPython extension. It returns an owned Python
float list directly, with no Torch dispatcher call. It rechecks every row, so
lazy actor/observation getters can change methods, instance attributes, or modes
between rows. Target conversion/finite errors precede the actor getter, actor
errors precede the observation getter, and the pinned Python joint-range check
remains last. It never combines outputs eagerly or broadens joint limits.

Identity guards read exact module dictionaries and raw class MRO dictionaries.
Missing module attributes select the original fallback, and class descriptor
checks never execute a custom `__get__`. The instance dict comes from its checked
canonical getset C getter. Five independent-review regressions cover removed
Torch Tensor/dtype/layout attributes and custom device/shape descriptors.
Type dictionaries and returned descriptors use scoped owned references on every
branch. A reference-count regression covers repeated success, early fallback,
and context construction/destruction calls.

Only the exact Tensor binding exported by the loaded Torch build can be unpacked.
Subclasses, Parameter, ducks, patched descriptors, nonempty or custom instance
dicts, active function/dispatch modes, wrong dtype/layout/shape, negative or
conjugate views, and noncontiguous tensors use the pinned original row function.
Canonical libtorch dtype/layout identities reject constants already patched
before construction. A genuine Tensor dict descriptor is checked before reading
an exact instance dict, so custom dict properties and boolean callbacks do not
run in the selector. Native mode-stack lengths use verified Torch C builtins;
the stack must be empty, and changed/noncanonical builtin getters fall back
without being called. Python mode-helper callbacks are not used. Metadata checks
finish before the storage pointer is read, under the GIL. Float32 widening uses
no arithmetic, preserving signed zero and input bits.

The bridge uses libtorch_python's private Tensor ABI and the current CPython
headers. The build record pins source, library, Python/Torch versions, and the
complete compiler command/output. The target must build it separately and pass
the same tests; Mac compilation and timing prove no target ABI compatibility or
20 ms loop improvement. Namespace ownership, source/binary verification, lazy
error order, independent lists, requires-grad preservation, original fallbacks,
and mutations before/between rows are tested with a genuine compiled extension.

The hardened local comparison used 501 pinned saved frames and 2,004 ABBA-timed
calls per method with zero fallback calls. Original/r48 wall medians were
11.250/4.125 µs; p95 values were 11.583/4.333 µs. A separate paired run of the
frozen candidate in that process measured original/frozen medians of
11.500/9.625 µs. Comparisons checked every
output's float64 bits and all inputs' float32 bits outside timing. These are
Mac/Torch 2.10.0 results and do not supersede the failed ARM result or establish
any target speedup. The private raw receipt SHA256 is
`787d9f27b087fae710acaddc52b58616e693a02846fe61b8617e3eeea9fb16b6`.

The saved-profile CLI retains the original strict pinned 501-row input audit and
ABBA comparison with all raw wall/thread-CPU samples and bit/mutation checks
outside timing. Its default plan imports no Torch, compiles no library, and opens
no hardware. Explicit execution reads saved values only; it loads no model or
controller. No production/runtime selection is added.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime python3 -B -m unittest \
  experiments.native_policy_output_rows_r48.test_conversion \
  experiments.native_policy_output_rows_r48.test_saved_profile -q

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime python3 -B -m \
  experiments.native_policy_output_rows_r48.saved_profile \
  --report /absolute/original/report.json --report-sha256 EXPLICIT_SHA \
  --records /absolute/original/records.json --records-sha256 EXPLICIT_SHA \
  --raw-audit /absolute/audit.json --raw-audit-sha256 EXPLICIT_SHA \
  --output /absolute/fresh/private/conversion.json
```

Add `--execute-file-only` only for the explicit saved-file comparison. Output
paths must be fresh, plain, and outside Git.
