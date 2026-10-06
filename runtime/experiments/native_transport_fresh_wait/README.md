# Fresh relative wait regression

The diagnostic transport source corrects one timing issue. In the original loop, `t` is captured before the boot-identity `pread`, then reused to compute a relative `pselect` timeout. Time spent in that check can extend the intended absolute wake. The corrected source reads the monotonic clock immediately before computing the timeout and rejects an already-expired original deadline.

`prepare_variant.py` accepts two exact reviewed source SHAs. Legacy input receives the single-block forward correction; already-corrected input is copied unchanged with an empty diff. The manifest names both source versions and the transformation. Unknown bytes are rejected. It keeps the input and output complete source plus a diff and manifest in a fresh directory. It never reverses the fix, compiles, loads a library, accesses hardware, or grants timing/actuation approval. Existing deployed binaries and frozen kits remain unchanged until a separate build and validation; callers are unchanged.

`fixtures/transport-before-fresh-wait.cpp` preserves the exact pre-fix source for deterministic comparisons. Tests verify that current production bytes equal the one-block forward transformation before compiling either source. It is a test fixture, not a runtime fallback.

```sh
python3 runtime/experiments/native_transport_fresh_wait/prepare_variant.py \
  --source runtime/experiments/native_transport/transport.cpp
```

Omit `--output` for PLAN. A fresh absolute `--output` directory writes the separate source artifact. A complete source file, rather than a hidden include dependency, lets a later separate build bind every compiled source byte. No build or live runner is supplied here.

The simulator intercepts every transport syscall before including either the legacy fixture or current production source. Tests do not open real device FDs. With a synthetic 2ms boot read, the same 5ms request gap becomes 7ms in the original and remains 5ms in the corrected source. A no-response timeout similarly reaches the original absolute deadline instead of 2ms afterward. These controlled examples demonstrate the defect; they do not establish the cause of any real robot overrun.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/tests \
  python3 -B -m unittest test_native_transport_fresh_wait -v
```

Original cancellation, EINTR limit, absolute deadlines, no retry, allowed request kinds, matching and malformed-response behavior are preserved. One additional monotonic clock read per loop has a measurement cost. No reduction in USB/CAN response latency is promised.

## Remaining causal measurement

Current `SDRecord` timestamps identify host write and read brackets. Several frames can share the same `read_start_ns` and `received_ns` because one read returned a batch. `SDStats` records only aggregate wait/read counts. It does not record when an adapter received a CAN frame or when the kernel first made a serial FD readable.

A future isolated diagnostic can preallocate fixed event slots for `pread`, `pselect`, `read`, and `write`: entry/return wall and thread-CPU clocks, requested absolute wake/timeout, return value, saved errno, serial/cancel readiness bits, and returned byte count. Record incomplete/overflow explicitly and preserve the original errno. Never allocate, print, retry, or change deadlines inside those hooks. The event clocks add overhead and do not themselves distinguish kernel sleep from runnable scheduling delay; a bounded scheduler trace is needed for that distinction. USB arrival and CAN-wire timing need separately timestamped kernel/adapter evidence. This instrumentation is not implemented or enabled by this variant.
