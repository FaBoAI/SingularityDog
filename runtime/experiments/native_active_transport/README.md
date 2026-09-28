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
is attempted at about 208 ms rather than 125 ms; the entire configured wait is
still at most 250 ms per bus, with both bus owners running independently. OS
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
The runtime makes one cleanup attempt; this API safeguard does not add retries.
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
