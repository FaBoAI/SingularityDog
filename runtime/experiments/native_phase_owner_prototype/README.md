# Fake native phase owners

This complete, isolated **file/pipe-only prototype** tests a different handoff
structure: two persistent pthread owners publish fixed native phase slots, and a
coordinator collects both buses through a bounded CDLL call. It does not open or
configure any serial/CAN/IMU device, link `sd_exchange`, load a model, select a
production runtime, or implement a motor frame. The 32-byte `SDFAKEQ1` /
`SDFKREP1` packets are an explicitly separate fake protocol.

Each generation has one absolute native `CLOCK_MONOTONIC` deadline of at most
20 ms and three fixed slots per bus. Each owner publishes feedback, notifies the
native condition, then progresses to voltage without a Python Future. The
coordinator must collect and acknowledge both same-generation pairs, and
explicitly submit the fake `STOP_ONLY` phase. Slots cannot be overwritten until
all six are acknowledged. Generations strictly increase; each owner retains its
last finish time and applies a 900 µs fake inter-stage gap. Failed packets or
generations are not retried. A bounded fake exchange may continue partial
reads/writes and repeat EINTR/EAGAIN operations within that same packet; this is
not the real transport's transient-count or partial-write contract.
The real 26-request schedule, window-three behavior, motor feedback semantics,
boot identity checks and STOP confirmation are **not** reproduced or qualified.

`collect` requests at most 200 µs per call, records the actual monotonic return
observation, and checks native errors before readiness/timeouts. Linux uses
`pthread_condattr_setclock(CLOCK_MONOTONIC)` and `ppoll`; macOS uses relative
`pthread_cond_timedwait_relative_np` and `select` derived from fresh monotonic
remaining time. No realtime-absolute deadline is substituted. An OS scheduling
stall can still make an observed return late. This is not a proof that runtime
boot-guard checkpoints, main-thread GIL reacquisition, IMU work or Python
semantic validation have become bounded or unnecessary.

Native errors poison the entire session. Cancellation is sticky and wakes
native waiters; a later timeout/cancellation does not overwrite the first native
error. A copied snapshot must match its native slot byte for byte before an
acknowledgement. Partial or absent replies retain their actual byte counts and
cannot become completed rows. Ready/consumed masks distinguish missing slots
from saved rows; unready all-zero scratch is not evidence of success.

The native API accepts distinct nonblocking FIFO pipe descriptors. The supplied
executable example and tests create private **anonymous pipes**, but `S_ISFIFO`
alone also accepts named FIFOs. No device descriptors are accepted. The API
captures dev/inode/access/nonblocking identity and duplicates descriptors with
close-on-exec. It never sets caller descriptor flags or closes caller-owned FDs.
macOS may add its kernel `FWRITTEN` observation after a write; caller-settable
access/nonblocking/append flags remain unchanged. Duplicating a pipe does not
prove the existing real-transport `Session` FD identity/boot guard contract.

The coordinator is a single creating thread with nonreentrant entry checks.
`Owners(bindings)` allocates no native resources. The caller retains that object
before calling `start(fds)`. Native creation publishes its token into the object's
preallocated caller-owned cell before returning. An interruption after creation
therefore leaves a reachable token for the caller's `finally` cleanup, including
retry if cleanup fails; there is no pointer-return creation factory. Callers that
discard the object or fail to attempt cleanup are outside this managed protocol.
Cancellation can come from another thread. Native handles are nonreused opaque
registry tokens, not addresses: the registry serializes lookup/deletion, and an
old token is rejected even if Python is interrupted after native close but
before clearing its wrapper field. Such a cleanup retry reports **unknown
original cleanup evidence**, rather than inventing a restored proof. The Python
lifetime lock additionally serializes cancellation versus close. The registry
also serializes prototype C entries; this is an explicit experimental design
tradeoff, not a performance claim.

Close marks cancellation before waiting for in-flight owners, observes their
termination, joins them, then closes owned duplicates and releases state. Linux
uses bounded try-join; macOS has no timed pthread join, so it joins only after
owner exit is observed and reports any late completion instead of bounded
success. Cleanup failure retains the session for a safe retry. No worker is
detached, and state is not freed while an owner can use it. Fake responder threads
are independently joined even when native cleanup or a Python interrupt fails.
The tested interruptions are the native-create/native-close return boundaries;
this is not a blanket asynchronous-exception guarantee during Python fixture
construction or file-descriptor creation.

Default CLI is PLAN: it reads source metadata and writes a fresh result outside
Git, without compiling/loading a library or opening pipes. Explicit file-only
execution builds one private library, verifies its source/binary/build record
and exact structure ABI, and runs two fake generations. Outputs cannot overwrite
or follow symlinks. All live qualification and performance claims remain false.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime/experiments \
  python3 -B -m unittest native_phase_owner_prototype.test_prototype -q

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime/experiments \
  python3 -B -m native_phase_owner_prototype.prototype \
  --execute-file-only --output /absolute/fresh/private/prototype.json
```

Future real integration needs a separate source/ABI/artifact review of unchanged
`sd_exchange`, existing session/boot guards, exact request pacing, voltage
validation, snapshots, native error handling, stop ambiguity and cleanup. This
prototype does not establish core parity, explain an 866.646 µs delay, or prove
that removing Future publication improves the real loop.
