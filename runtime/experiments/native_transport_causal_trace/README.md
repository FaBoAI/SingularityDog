# Separate diagnostic syscall trace proposal

This directory is a proposal and an isolated recorder regression. It is **not integrated** with any transport, collector, controller or existing ABI. Default `plan.py` only verifies the exact corrected diagnostic source and prints `PLAN_ONLY`. No execute/build option, native loader, device path or output permission exists.

```sh
python3 -B runtime/experiments/native_transport_causal_trace/plan.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/tests \
  python3 -B -m unittest test_native_transport_causal_trace_plan -v
```

`bounded_trace.hpp` is a fixed-capacity POD recorder with a synthetic/syscall-call wrapper. It invokes its noexcept callback once, preserves both the callback's incoming and outgoing errno across optional clock reads, and records a return value without retrying. A null recorder bypasses all clock work. Overflow and invalid/missing clocks are explicit; neither changes the original callback result. The supplied harness uses only fake callbacks/clocks, never a real syscall or transport. It is not an exported ABI or a ready live trace adapter.

## Proposed integration, still unimplemented

Build a separate, explicitly pinned STOP-only diagnostic library with an additional opt-in traced entry point or getter. Preserve the original `sd_exchange` signature and `SDRecord`/`SDStats` layouts; never overload their fields. Keep three preallocated trace slots per bus owner, one for acquisition, voltage and output, each with 256 event slots initially. Bind slots to the owner thread, bus, native begin timestamp, request sequence and original absolute deadline. No other thread may read a slot until its owner has finished. Overflow must prevent a complete-causality claim.

Bracket boot `pread` and `pselect`. Record requested absolute wake/relative timeout, result, saved errno, and serial/cancel readiness. Reuse existing read/write timestamps and preserve returned chunk sizes and all original bytes. Ready-bit capture, owner binding, separate-library selection and serialization are not implemented here. Capture errno immediately after each real syscall and restore it after tracing operations. Optional worker thread-CPU clocks are a separate, more intrusive mode.

Retain the failed coordinator cycle **after all owners settle**, even when a native exchange succeeds after the coordinator deadline. Keep a few bounded prior slots or per-call maxima for comparison. Failure-only persistence still incurs instrumentation during every measured call: compare traced and untraced runs separately and measure overhead. Do not allocate, print, write files, resend, alter timeout arithmetic, or substitute recorded source timestamps in the hot loop.

## What the evidence can establish

A long boot-read bracket locates elapsed time around that syscall; it does not alone distinguish kernel execution from preemption. A `pselect` timeout return later than the requested wait locates sleep/wakeup/return delay. A ready return cannot distinguish late USB data from delayed scheduling without kernel evidence. Several frames returned by one read prove host batching only. Wall time minus worker CPU time is not automatically GIL, USB or kernel waiting.

For the motivating failure, the final host `read` bracket was only a few microseconds, while the preceding gap was milliseconds. Existing records do not identify which part of that gap was boot checking, readiness waiting, OS scheduling or adapter delivery. The corrected relative-wait arithmetic is independently justified, but it has not established the cause of that specific delay. This proposal grants no active timing or physical-output qualification.
