# Bounded native active RS05 transport

This is a separate opt-in experiment. It does not change the diagnostic
`native_transport` library. It can send enable and position commands and must
only be called by the supervised active runner. The module never opens a device,
changes its baud rate, starts a worker, arms a motor, or loads a library on import.

Build locally for the target architecture, without installing dependencies:

```sh
python3 runtime/experiments/native_active_transport/build.py
PYTHONPATH=runtime:runtime/tests python3 -m unittest test_native_active_transport -v
```

The output is `libdog_active_transport.so` with a source/binary SHA-256 build
record. These generated files are ignored by Git. The build uses C++17 and the
standard library only. Tests use local socket pairs and PTYs; none open a robot,
serial device, network connection, or SSH session.

## API and ownership

`singularitydog_hw.native_active_transport.load_library(path)` verifies the
adjacent build record and monotonic-clock agreement before returning the library.

```python
session = ActiveSession(
    lib, caller_owned_nonblocking_fd,
    first_id=1,                 # exactly 1..6 or first_id=7 for 7..12
    cancel_fd=cancel_pipe_read_fd,
    boot_fd=caller_owned_boot_file_fd, boot_id=expected_boot_id,
    raw_lower_by_id=lower, raw_upper_by_id=upper,
    kp_max_by_id=kp_caps, kd_max_by_id=kd_caps,
    gap_ns=600_000, window=3,
)
records, stats = session.exchange(wires, timeout_ns=100_000_000)
```

An explicitly selected `ActivePhasePair(front_session, rear_session)` borrows the
same two serial sessions and creates two persistent C++ owners. Its
`submit({'front': front_frames, 'rear': rear_frames}, deadline_ns=absolute)`
returns two ordinary Futures after one coordinator submission. C++ validates
both complete batches before releasing a single generation to both owners,
then calls the same individual-frame exchange parser. It does not combine
17-byte USB writes, change the selected request interval, relax input freshness or omit an
acknowledgement. A native failure wakes the sibling owner's `pselect` through a
private cancellation pipe; each Future retains that bus's raw records/stats.
Normal owner completion wakes the condition-variable coordinator only after
both owners finish. Error, cancellation, settings and closing wakeups remain;
the raw result slots are still copied only after both writers join.

The optional completion-notification ABI uses a private nonblocking pipe. C++
signals only after both owners have joined and copied their raw result slots;
Python signals again after publishing the current two Futures. A notification
is a wake hint, not an acknowledgement or a Future readiness proof. The bounded
C++ waiter checks cancellation and the unchanged absolute host deadline. The
supervisor still checks both current Futures, reply timestamps, decode errors,
and the result-takeout deadline. Older binaries without this optional ABI keep
the existing wait path; a partial or malformed notification ABI is rejected.

`submit(..., result_transform=callback)` can transform the stable raw bus mapping
on the same coordinator before publishing those original Futures. The raw
mapping stays in `last_completed_bus_results`. The selected Type1 runner uses
this to journal/decode without a second queued task or replacement Futures.
The transform must not synchronously invoke STOP or join its own coordinator;
the runner dispatches failure recovery separately after native writers join.
No succeeding generation can overlap result transformation/publication, and
notification descriptors are retained until all borrowers have finished.

Native join, `Future.done()`, and completed publication are separate states.
`wait_idle()` confirms that both native writers have joined and released their
session locks. The generation remains owned until transformation, publication
of both original Futures, and the final notification attempt finish. Periodic
collection checks `publication_complete(futures)` as well as those exact current
Futures; non-periodic collection uses the bounded `wait_published(futures)`.
Callback or notification failures remain visible through this publication
fence. The next generation cannot reuse a result slot prematurely. Notification
waits retain the existing 200µs readiness ticks and 50µs ticks in the last 1ms,
without extending either the native I/O or coordinator deadline.

The pair can coexist with the original per-bus Python owners while idle, so the
prepared feedback-publication/voltage path remains on its existing owners.
The supervisor must cancel and `wait_idle()` before scheduling STOP, and must
`close()` the pair before closing either borrowed session. Pair construction,
close and destruction do not send motor commands. Pair selection is opt-in;
an older native library without the optional pair ABI is rejected only when
that feature is requested. Original defaults are retained.

## Original-Future readiness hints

The separate optional `sda_future_readiness_abi` / `sda_wait_future_ready` ABI
waits on a coordinator-owned cancellation pipe and a private, nonblocking hint
pipe. It receives no motor/serial descriptor and consumes no hint byte. A queued
hint returns an actual monotonic wake time; EOF, changed descriptor bindings,
wrong endpoint modes and cancellation fail closed. Cancellation takes priority
over a hint or an expired deadline.

The exact pinned `_OwnedActiveWaiter` can register one group of original,
distinct `concurrent.futures.Future` objects. Each callback writes one bounded
hint. The coordinator drains the pipe and rechecks those same Futures, their
errors, the original absolute deadline and the result-takeout deadline. A hint
never certifies that all inputs are ready. Group cleanup is also inside that
original deadline. The active runtime uses this for CAN/IMU acquisition and
overlapped voltage collection; an older binary or a generic injected waiter
keeps the existing polling path.
The new hint path waits interruptibly to the original deadline on cancellation
and Future events. It does not claim a periodic 200us monitoring tick; the
legacy polling path and the separate output-pair notification loop retain
their existing bounded ticks.

Each group owns a fresh pipe. Deactivation and close are fenced against
publishing callbacks, old callbacks cannot write to a reused descriptor, and
weak references avoid accumulating closed groups while cyclic GC is deferred.
No additional bus owner, motor request, profile approval or deadline allowance
is introduced.

`benchmark_voltage_readiness.py` compares both routes using anonymous pipes
and synthetic original Futures, including registration, publication, takeout
and cleanup. Its 20ms synthetic phase budget is not a CAN/model cycle timing
qualification. Run `test_native_future_readiness`,
`test_voltage_future_notification_join` and
`test_acquisition_future_notification_join` for the cancellation, lifetime and
original-proof contracts. Build and benchmark for the actual Jetson architecture
before claiming any live timing improvement.

On Linux, explicitly configure `pair.configure_owners((0, 1, 2, 3),
timer_slack_ns=1000)`. The API reads/applies/verifies CPU masks and timer slack
on each same native owner TID, retains original values, and returns evidence.
`restore_owners()` restores/readbacks on those same TIDs; `close()` also attempts
restoration before destroying them. Unsupported or failed placement is an
error, never a claimed performance setting. macOS socket tests exercise the
transport ownership but explicitly reject these Linux-only placement controls.
`last_phase` retains generation, publication and both owner start/finish times.

`tools/offline_native_pair_comparison.py` defaults to PLAN and can explicitly
compare ordinary 26-request phases on local socket fake QDDs. Those fast replies
measure host implementation overhead only. They do not establish real USB/CAN
latency, Jetson performance, Type1 output, or 20ms hardware qualification.

Each limit mapping must contain exactly the bus's six integer IDs. Limits are
copied at construction. Both Python and C++ enforce raw position within
−12.57..12.57 rad, per-axis Kp at most 36, and Kd at most 1. These are software
experiment caps, **not manufacturer assurances of safe torque or safe support**.
The runner must supply tighter reviewed per-axis limits where appropriate.

Only one native session can own the same underlying FD (including duplicated
descriptors). A native mutex and Python lock reject overlapping operations.
The caller must also hold the project's cross-process port/common locks. The
caller retains the FD lifetime. `close()` releases the native handle, not the FD,
and does not send commands. The runner must complete STOP handling before close.

## Exact command and reply boundaries

| Request | Accepted payload | Reply |
|---|---|---|
| Type0 | all zero | exact source, host `FE`, Type0 UID |
| Type17 | indexes `7019`, `701B`, `701C`, `7028`, `7005`; reserved bytes zero | exact index/source/host `FD`, zero status/reserved; finite floats, uint32 watchdog or uint8 run-mode |
| Type3 | all zero | Type2, mode 0 or 2, fault 0 |
| Type1 | canonical zero FF and zero velocity; encoded position and gains inside configured per-axis limits | Type2, mode 2, fault 0 |
| Type4 ordinary STOP | all zero; never clear faults | ordinary Type2, mode 0; fault bits retained |
| Type4 version query, used before enable | exactly `00c4000000000000` | Type2 mode0/fault0 with `00c456` prefix, 4 raw version bytes, and separately retained byte7 |
| Type18, acknowledged exchange | index `7028`, reserved zero, exactly 4000 ticks | Type2 mode0/fault0 acknowledgement, followed by a separate Type17 readback and validation |

`send_only()` is retained for ABI compatibility but rejects calls. Watchdog writes
must consume their acknowledgement before the readback is sent.

`encode_motion(mid, q, kp, kd)` uses the same floor-to-u16 profile as the existing
RS05 trial codec. Zero FF and velocity encode as 32767. The C++ bound check uses
the **decoded quantized** position; a target exactly on a lower limit may round
below that limit and be rejected. There is no saturation, wrap repair or implicit
gain selection. Type1's middle 16 CAN-ID bits contain torque, not host `FD`.

All requested frames are validated before any write. Every OS write is exactly
one 17-byte AT frame. The minimum gap is measured from the preceding write's
completion, including preceding exchanges. At most three distinct request keys
are outstanding. Requests sharing an actuator and Type2 reply kind cannot be
mixed in one exchange. Cancellation, boot binding, FD binding and deadline are
checked before every write. The overall exchange budget is 1..250 ms.

`Record` matches the diagnostic layout. `Stats` preserves its existing fields and
adds `rejected_total`. Diagnostic `exchange_evidence(records, stats)` is compatible;
also retain `stats.rejected_total` to detect bounded raw-log truncation.
`decode_record()` reports raw values, mode and faults without declaring them a
valid calibrated robot state. Timing is host-side, not CAN-wire timing.

## Optional pure six-record feedback decoder

The existing library now exports a separate optional codec ABI:
`sda_feedback_decode_abi()` and `sda_feedback_decode_batch(...)`.
`NativeFeedbackBatchDecoder` accepts one exact, root-owned ctypes `Record * 6`
array after its native writers have joined, with `first_id=1` or `first_id=7`.
It decodes six canonical Type2 replies to Type1, Type3, ordinary Type4, or Type18
requests. The opted-in native-pair Type1 runner selects it only for genuine
`ActiveSession` libraries. Generic sessions and the default transport route
retain their existing decoder.

ABI 1 is reported only when the C++ `sizeof` and every field offset match the
declared layouts: `SDRecord` is 88 bytes, with `tx` at 40, `rx` at 57, `written`
at 76 and `received` at 80; `SDFeedbackDecoded` is 64 bytes, with its four double
fields at 16, 24, 32 and 40 and timestamps at 48 and 56. Unexpected compiler
packing is rejected. Python verifies the exact GIL-releasing CDECL function
identities, argument types and return types before using its reusable scratch.
Both optional symbols absent means the legacy decoder remains available;
a partial, changed or incompatible ABI fails closed.

This codec has no session, descriptor, clock call, worker, or transport I/O. It
reads stable records and publishes decoded scratch only after the whole batch
passes. It preserves the original Record buffers, request/receive timestamps,
record and dictionary insertion order, Type2Feedback type, mode and fault bits.
It does not unwrap positions or decide whether a reported mode/fault is safe
for motion. Native transport validation and the supervisor's existing admission
checks still make those decisions.

The position, velocity and torque calculations retain Python's double rounding
after each multiply and divide; temperature uses the same division. Exhaustive
uint16 tests compare IEEE-754 bits for all four numeric fields. Unsupported
counts, mixed identity/parameter/version transactions, invalid frames or
timestamps, and scratch contention return to the authoritative Python codec,
preserving its accepted values and exact rejection messages. They do not skip
replies, reuse an earlier result or relax a deadline.

Run the component comparison after building this library. The output file must
be new:

```sh
PYTHONPATH=runtime python3 \
  runtime/experiments/native_active_transport/benchmark_feedback_decode.py \
  --library runtime/experiments/native_active_transport/libdog_active_transport.so \
  --output /tmp/feedback-decode-component.json \
  --samples 2000 --trials 4 --warmup 200
```

It alternates Python/C++ measurement order and includes the ctypes boundary,
Type2Feedback objects and dictionaries for twelve synthetic Type1 feedback
records. Before timing it checks values, order, timestamps and float bits; it
also checks source-buffer preservation and GC restoration. No session or model
is created, and the report does not claim Jetson or whole-cycle measurement.

Compare against an existing pinned raw-record file separately:

```sh
PYTHONPATH=runtime python3 tools/verify_native_feedback_decode.py \
  --records "$SAVED_RECORDS_JSON" --records-sha256 "$SAVED_RECORDS_SHA256" \
  --library runtime/experiments/native_active_transport/libdog_active_transport.so \
  --output /tmp/feedback-decode-saved-parity.json
```

The saved JSON is a cycle list with `acquired`, `voltage` and `output` phases,
each containing `front` and `rear` objects with original `records` arrays.
Each record supplies the original seven integer fields (`start_ns`, `finish_ns`,
`read_start_ns`, `received_ns`, `deadline_ns`, `written`, `received`) and exact
lowercase `tx_hex`/`rx_hex` for its two 17-byte frames. The tool verifies the file
SHA before and after processing, reconstructs owned buffers, compares original
timestamps/order/float bits, and reports native and legacy batch counts. It
accepts at most 10,000 cycles and a 64MiB input file. This is saved-data parity,
not a timing measurement or hardware qualification.

Neither codec nor component benchmark changes the selected request interval,
window3, the success-setting baseline, arming conditions or the 20ms deadline.
Actual Type1 output and full-cycle latency still require separate Jetson tests.

Version replies cannot acknowledge ordinary STOP/enable/Type1 commands, and
ordinary feedback cannot acknowledge a version query. The runner reads all
twelve raw fingerprints on the same bus owners before enabling, then compares
them to the tested watchdog evidence. A semantic version label does not replace
those bytes. This adds startup work only, not per-cycle communication. Emergency
STOP remains restricted to the all-zero payload.

## Fault and emergency STOP behavior

Partial writes, missing/late/unmatched/duplicate replies, unexpected RX, malformed
frames, partial tails, cancellation, FD/boot changes and invalid requests poison
the session. No active command is retried; C++ retains the poison even if the
Python flag is changed. A STOP that reports fault bits returns those bits and
also prevents later active commands. The supervisor must perform STOP handling
after any failure and preserve both the failing exchange and STOP evidence.

After cancellation, join the owner call before invoking:

```python
result = session.emergency_stop()  # 250 ms total for this six-axis bus
```

This method ignores cancellation, a changed boot ID, and poison **only to send
all-zero Type4 STOP**. It retains FD identity/nonblocking checks so that a reused
descriptor cannot receive actuator traffic. Its six independent time slices each
permit one individual STOP write; a missing reply from one motor does not prevent
the remaining attempts. No axis is retransmitted. STOP never clears faults.
The default 250 ms budget includes all six attempts; OS scheduling, disconnected
FDs, or an insufficient caller-selected budget may prevent an attempt, and the
returned `attempted_ids` makes that explicit.

The prior 150 ms default allocated about 25 ms to the first axis. In the first
supported output trial, matching front ID1/2 disabled replies missed those slices
and were retained as rejected bytes; confirmed replies on other axes took about
22–26 ms. The new default gives each axis about 41.7 ms. Replies are still accepted
only before their original, recorded axis deadline. The previous trial remains
STOP-unconfirmed; this change does not reinterpret its rejected replies.

This is a cleanup acknowledgement budget, not a change to the 20 ms active cycle
or the 200 ms device watchdog. When every reply is missing, the sixth STOP write
is attempted at about 208 ms rather than 125 ms; a single default call still budgets 250 ms per bus, with both bus owners
running independently. The repeated wrapper has a separate total budget below. OS
descheduling can delay execution beyond a configured deadline, so this is not a
hard real-time guarantee. No later motion is permitted while STOP collection is
running. An uncertain STOP still requires physical cutoff. Synthetic socket
regressions cover 28 ms replies, expired replies, and all-six missing replies;
the revised default has not yet been validated on the robot.

The JSON-ready result contains `complete`, `timeout_ns`, `deadline_monotonic_ns`,
`attempted_ids`, `confirmed_ids`,
`unconfirmed_ids`, `ambiguous_ids`, `fault_by_id`, decoded `replies`, and raw
`evidence`. Confirmation means a matching mode-zero reply was observed after
the write; it is not proof of physical cutoff. Pending motion/enable/STOP replies
from a failed earlier exchange make that motor's confirmation explicitly ambiguous.
A pending Type1 may respond in mode zero after watchdog/fault disable; it cannot
be distinguished from a new STOP acknowledgement.
An unanswered STOP from emergency cleanup is also remembered by the session.
If a caller attempts emergency STOP again on that session, a delayed reply to
the earlier attempt remains ambiguous even when it arrives after the new write.
The runtime calls `emergency_stop_repeated` on each bus owner. A successful first
round ends cleanup; otherwise at most three STOP-only rounds share one absolute
one-second budget. The first round retains a 250 ms cap; later rounds may use
500 ms or the remaining total budget. The collector uses a shared 1.25-second
wait including dispatch overhead. No motion is retried, fault evidence is
retained, and additional mode-zero replies never erase prior ambiguity.
There is no sequence number in Type2: very late duplicate replies cannot be
proven fresh by this transport. The runner must use freshness and state limits,
and an unconfirmed or ambiguous STOP requires the physical cutoff procedure.

Recovery preserves pre-write backlog and every skipped/malformed byte up to a
4096-byte evidence cap, plus total byte count and a truncation flag. This bounded
STOP-only parser may scan past malformed backlog; the active parser never
resynchronizes or discards bytes. Previously consumed failure bytes remain in
the original `ExchangeError.stats`. Emergency STOP leaves the session poisoned;
it cannot restart a trial.

This offline implementation and its socket/PTY tests are not a hardware trial,
watchdog validation, motor-scale confirmation, real-time guarantee, or approval
to deploy a policy. Mechanical support, cutoff readiness, identities, current
boot calibration, mode, watchdog readback/behavior, and fresh inputs belong to the
supervised runner's arming checks.
