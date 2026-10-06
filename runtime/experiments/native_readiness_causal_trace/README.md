# Isolated readiness timing observations

This source-only prototype investigates the saved R37 diagnostic failure. Its
last native reply preceded the 20 ms deadline, but Python observed the final
wait callback return after the deadline. Earlier in that same cycle, the saved
voltage validation end preceded the main voltage join by about 867 microseconds.
These observations do not identify GIL, scheduling or device latency as a cause.

The command line prints a file-only plan. It has no execute/build/device option.
No production benchmark, controller, transport, profile or caller is changed.
`build_traced_helper()` verifies the fixed K37 source SHA and imported function
bytecode, then constructs a separate helper. With `trace=None`, it delegates
directly to the original function without extra trace clocks or callbacks.

An explicitly supplied `FixedReadinessTrace` records fixed-size numeric rows:

- readiness/error checking and boot/cancellation guard wall and thread CPU times;
- planned native wake, the native callback's returned `woke_ns`, and the existing
  Python return clock;
- the unchanged decision and completion clocks;
- one registration and completion-callback observation per owner Future,
  including the separate voltage-validation Future when present.

The completion callback runs **after** internal Future completion. It is an
upper-bound observation, not the exact publication time, native exchange end,
or on-wire arrival. A callback registered on an already-done Future is explicitly
marked. The returned native time also requires the original verified native
waiter binding; the experiment alone does not authenticate a callback's claim.
Callback thread CPU time belongs to the thread executing that callback: the
registering thread for an already-done Future, or its completing worker. It is
not authenticated native-owner CPU time and must not be subtracted from a
different thread's clock.

Rows and callback closures are allocated before polling. No per-poll list,
dictionary or I/O is added. Python clocks and numeric bookkeeping still have a
cost, so this is not claimed to be allocation-free or free of measurement
overhead. Capacity is bounded; overflow and measurement errors invalidate trace
completeness without extending deadlines, hiding original errors or replacing
results. Source sample times and native dispatch deadlines are untouched.

Export only after the helper has finished and every original owner has settled;
normal owner cleanup remains the caller's responsibility. Returned JSON is a
copy and grants neither timing qualification nor motor-output authority.

Before a separately approved disabled diagnostic integrates this prototype:
bind source/library/ABI and cycle/phase/owner identities, measure added trace
overhead against an uninstrumented run, preserve original result/error/restore
artifacts, and separately identify worker-return versus Future callback times.
No such integration or target measurement has been performed here.

Tests use synthetic clocks, Futures and subprocesses that can only print PLAN
or reject unsupported CLI arguments. They reproduce timely completion, observed
return overshoot, guard delays, error/cancellation precedence, overflow, source
mutation, and default delegation without CAN/IMU/native library/model access.

## Separate collector and source bundle

`generate.py`, `collector_support.py`, and `child_runner.py` provide a reusable
integration **outside** the frozen benchmark. Generation is file-only; it does
not import the generated benchmark or launch anything. It requires the exact K37
benchmark bytes. Only `collect` and `main` change in the generated copy. Every
replacement is inverted and the complete original bytes and AST must match.
Structural checks use exceptions and remain active under `python -O`.

Supply an absolute fresh private output directory and the intended target kit
path. The latter is a declaration about the target host, not a local filesystem
check; the child verifies all 650 target kit members before importing the copy.
For example, after setting `BASELINE`, `KIT`, and `OUT` to the intended paths:

```sh
python3 -B runtime/experiments/native_readiness_causal_trace/generate.py \
  --baseline "$BASELINE" --kit-path "$KIT" --output "$OUT"
```

This writes four pinned source members plus `manifest.json` and a derivation
receipt. A source-sealed scoped launcher can also be derived with
`--launcher-template "$ORIGINAL_R37_LAUNCHER" --target-bundle "$TARGET_BUNDLE"`.
The exact original outer-launcher SHA is required. Its boot, power, calibration,
UID, native build/ABI, model, source-test and reversible performance-scope checks
remain. Inverse byte/AST validation proves that all declared launcher changes
can be removed to recover that exact original. Target source checks and current
physical conditions must still be reviewed before any target execution.

The derived scoped launcher's default is PLAN, with required `--phase voltage`
or `--phase output` and default `--cycles 5`. It permits at most 50 cycles. It
retains STOP-only, gap 900 microseconds, window 3, 20 ms deadlines, and the
existing source-bound limits. `--execute` is an explicit future diagnostic
choice; **no generated bundle or launcher has been deployed or executed on a
robot in this source task**. PLAN performs file checks and imports the copied
module but does not open hardware or change the Python switch interval. The
child executes verified source bytes rather than a stale bytecode cache.
All copied and derived command-line parsers reject abbreviated options, so
`--exec` cannot bypass the child's exact `--execute` scope selection.

Only the selected phase is instrumented. Acquisition and the other join call
the untouched helper. Allocation happens before the measured loop; numeric
trace storage is exported after the original worker cleanup. The bank reports
invocation count, observed rows, and `requested_cycles_all_traced` separately.
Zero observations are never complete. An aborted partial run can have complete
observations for its attempted phases without claiming the requested duration.
Original errors, raw records and cleanup results remain in the report.

Executing-copy, support, instrumenter and original-dependency hashes are
separate. Finalization labels the old cadence map as a **dependency** map,
rather than a claim that the original benchmark was executed. Both timing
admission and active-output eligibility remain false. Future callbacks are
qualified as after-completion observations, including already-done registration.
Missing or noncausal native wake values invalidate the trace; they do not change
or suppress the original control result.

## Local observer-cost measurement

`measure_overhead.py` defaults to PLAN. The explicit
`--measure-local-pipe --output /absolute/fresh/private/directory` option compiles
the pinned diagnostic C++ source locally and uses only its wait ABI with a
cancellation pipe. It never calls an exchange or opens a serial port, IMU or
model. This is a separate microbenchmark, not a robot diagnostic. It retains all
800 measured rows in four ABBA blocks per scenario, including outliers.

A local Mac measurement of the r2 instrumenter (`ab299e13…`) gave:

| Scenario | Baseline median | Observed median | Added median |
| --- | ---: | ---: | ---: |
| Synthetic owners already ready | 6.542 µs | 18.313 µs | 11.771 µs |
| Native cancellation-pipe wait, one poll | 206.605 µs | 214.166 µs | 7.562 µs |

The retained raw report SHA is
`1308c77170342ef7593ddb3501a8c004438edd183c81ad1b1032890f16775743`.
Preallocation and export are outside the timed section; binding/callbacks,
clocks and recording are inside. Native-case completions are synthetic on the
coordinator, not real bus workers. This does not reproduce Jetson scheduling,
GIL contention or a real boot check. Added observation cost can itself cause a
20 ms failure. No whole-loop speed or causal conclusion follows from it.

Run the portable file-only checks from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/tests python3 -B -m unittest \
  test_native_readiness_causal_trace test_native_readiness_causal_collector \
  test_native_readiness_causal_generation test_native_adaptive_readiness_wait -q
```
