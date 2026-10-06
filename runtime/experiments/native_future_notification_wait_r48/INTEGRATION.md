# Phase and limit boundary

These five source files and `source-manifest.json` preserve the frozen private
candidate bytes. The experiment can observe ordinary Future readiness for
acquisition (two bus owners plus IMU Future), voltage (two bus owners plus
validation Future), or STOP proxy output (two bus owners). It is not connected
to any of these production phases. It accepts only caller-created FIFO read
descriptors for notification and cancellation, never serial/CAN device FDs.

The coordinator still checks owner errors before waiting or accepting results,
checks fresh boot at each readiness decision, and rejects at the actual absolute
deadline. Notification timestamps are not source acquisition times or exact
Future publication times. Result takeout, wire checks, fault checks and input
freshness are unchanged caller responsibilities. The existing 200µs requested
fallback and final400µs/50µs/half-budget rule remain; this is not an actual
200µs scheduler bound. No native20ms/pre-send20ms or any numerical/physical
limit is widened. The prototype never applies an active post-reply allowance.

Importing this package neither loads the library nor constructs a pipe. No
production selector, live-profile option or active qualification is provided.
Separate pinned-source integration, explicit passive STOP diagnostics and
review would be necessary before hardware comparison. The native persistent
phase-owner fake prototype is separate and has not been connected here.

For Linux file-only validation, run from this experiment directory:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest test_notification_wait -q
```

The 43 tests compile the isolated scheduling library using the host C++ compiler
and exercise synthetic ordinary Futures/pipes, cancellation, signals, deadline
and lifetime contracts. They do not import the robot runtime, Torch or a model,
and do not open any hardware or network. Temporary compiler output is removed.
Mac43/43 is source/contract evidence; Linux timing benefit is unmeasured. The
new `pselect` wake may have worse scheduler tails than the existing busy-spin,
so passing these tests does not prove a speedup or stable20ms execution.

The r2 static source reader also closes its owned FD when the regular-file
gate or stream-wrapper construction rejects an input. The original r48
archive is retained privately; wait logic, native C++ and baseline fixture
are byte-identical to that original revision.

The r3 one-shot join checks callback errors again after close. A late callback
I/O error cannot turn into a successful scope result, and an earlier owner
error remains primary. The receipt distinguishes a closed-FD cleanup from a
callback error; neither is a performance or timing qualification. The r1 and
r2 private source snapshots and archives remain preserved.

Ordinary `Future.done()` may become visible before its callback returns. If
that callback owns the FD lock when the one-shot join closes the scope,
`CloseBusy` correctly aborts the join with cleanup incomplete. This race can
also occur outside a test; the preserved failed local fixture is evidence of
this boundary. Synchronizing that fixture only makes its intended successful
close test deterministic and does not remove the candidate limitation. Before
any actual passive comparison, separately review whether a retained scope
lifetime is necessary. This candidate does not wait or relax a deadline to
finish callback cleanup.
