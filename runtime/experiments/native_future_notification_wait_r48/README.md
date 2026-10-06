# Dedicated readiness notification experiment

This is a complete **file-only ordinary-Future/pipe prototype**. It has no CAN,
IMU, serial, model, Type1, runtime selector, benchmark patch or actuation API.
K37 and the repository runtime remain unchanged. It has not run on Jetson or
inside a hardware diagnostic. All output/qualification flags are false.

The original coordinator checks Future readiness/error, fresh boot, and actual
clock at most 200µs requested ticks, reducing to 50µs/half remaining budget in
the final 400µs. The original native wait receives no completion notification;
the 500µs spin setting therefore busy-spins most of these short ticks. In five
instrumented output joins the latest callback-to-helper-end delay was median
168.966µs, maximum192.135µs, while 174 native wakes overshot their planned target
by median0.776µs/max1.8µs. Callback time is after Future internal completion and
may belong to the registration thread. It is not exact publication or native
owner time. Added trace-clock overhead is unmeasured.

R37's866.646µs validation-finished→join-end delay did not recur in five voltage
joins. Validation-finished is not Future publication. At least564.534µs of main
non-CPU time lies after validation-finished; GIL, scheduling, lock and syscall
causes remain unseparated. Its20.016074ms field is **Python native-wait return**,
not a recorded Future callback. Native rear receive19.953641ms/native end
19.953897ms precede the deadline, but readiness decision20.031851ms correctly
fails. None of these observations prove the new candidate fixes that failure.

`candidate.py` registers one callback per two/three ordinary Futures. The
callback writes one byte to a private nonblocking pipe. `notify_wait.cpp` waits
on that pipe and a **separate duplicated cancellation read FD**, clipped to the
original tick and overall hard deadline. TICK and NOTIFIED are different return
kinds; an early notification is never disguised as the original deadline wait.
Every return loops through the ready/error/boot/actual-clock checks. Taking
results, validating raw frames/faults and checking source freshness remain with
the caller; this prototype does not perform or bypass those operations.

The callback/write and close share a nonblocking lock. A contended callback may
skip the write and rely on the original tick fallback. A saturated pipe is
already readable. Draining is nonblocking and finite. A delayed callback after
close skips the write, so it cannot write through an FD number reused elsewhere.
`CloseBusy` leaves ownership explicit; no blocked callback is falsely declared
closed. Unknown close errors are sticky. `join_once` preserves a primary owner
failure and attaches incomplete cleanup/scope when needed. OS INT/TERM/HUP are
deferred only across FD allocation/close bookkeeping. Arbitrary asynchronous
exceptions at every interpreter bytecode are not certified. No failed join is
automatically retried.

Requested tick/guard cadence and all absolute limits remain unchanged. Actual
OS wake time can overshoot; strict clock checks reject that overshoot. New
`pselect` sleeping may have worse scheduler tails than the original busy-spin.
Notification callback/write, new fd validation, draining and GIL reacquisition
also cost time. This source/test result **does not claim a latency gain or an
actual≤200µs guard interval**. Linux file-only comparison under the same pinned
CPU/performance conditions is needed before a separate STOP diagnostic module
could be considered. Real native phase-owner integration remains a separate,
larger design and is not present here.

Run the fixed file-only tests (local C++ compiler required):

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest test_notification_wait -q
```

The tests compile only this small scheduling library in a fresh temporary
subdirectory and use synthetic Futures and anonymous pipes, never devices.
They bind source/binary in a build record before `ctypes.CDLL`, check ABI and
Python/native monotonic clock agreement, and remove the temporary build. The
canonical two K37 functions in `baseline_waiter.py` are exact AST fixtures for
valid ready/guard/deadline/error-order parity. There is no default execution CLI;
importing the candidate opens no FD and loads no library.
# Test entry points

The tests support both a package module invocation from the repository root and
standalone discovery from this directory. The imports select the local candidate
and baseline explicitly; they do not select a runtime backend.

```sh
PYTHONPATH=runtime python3 -B -m unittest \
  experiments.native_future_notification_wait_r48.test_notification_wait
```
