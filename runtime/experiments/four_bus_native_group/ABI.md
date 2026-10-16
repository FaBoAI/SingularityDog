# Experimental native group ABI 1

This optional composite DSO retains all ordinary/subset-active/subset-STOP
symbols and compiles their original source copies unchanged. Every Session
passed to it must originate from this same loaded DSO. Sessions created in a
different DSO must never be cast across libraries. A build receipt is not an
output approval; existing foreground admission remains required.
The composite exports ABI-compatible Session lifecycle wrappers around the
unchanged included functions. A DSO-local pointer registry rejects foreign or
unknown Session pointers before native group creation dereferences them.

No device opens, automatic retries, deadline extensions, wire coalescing,
feedback discards, encoder conversion or policy inference are introduced.
The gap is exactly 900 µs and window is 3. Session limits, request matching,
boot/FD/cancel validation, raw rejection bytes and sticky Type2 ambiguity
remain owned by the unchanged `exchange_owned` kernel.

## C layouts

`SDRecord`, `SDStats` and `SDPairOwnerSettings` retain their existing native
active ABI. `SDRecord` is 88 bytes; field names/order are unchanged.

```c
struct SDGPort {
    uint64_t owner_started_ns, owner_finished_ns;
    int32_t status;
    uint32_t count;
    SDRecord records[3];
    SDStats stats;
    char error[256];
};
struct SDGResult {
    uint64_t ticket, submitted_ns, validated_ns, released_ns;
    uint64_t collected_ns, cancel_requested_ns;
    SDGPort ports[4];
};
```

Use `sdg_result_size()` and `sdg_port_size()` to verify the platform's normal
C alignment against ctypes before creating a group. The receipt records
these query values. Port results remain in the immutable caller-provided
order; native CPU owner IDs do not imply a leg or physical serial port.

## Signatures and lifecycle

```c
uint32_t sdg_abi(void);                   // exactly 1
uint32_t sdg_result_size(void);
uint32_t sdg_port_size(void);
void *sdg_create(void *handles[4], const uint32_t masks[4],
                const uint64_t cpu_masks[4], uint64_t timer_slack_ns,
                SDPairOwnerSettings settings[4], char *error, uint32_t size);
int sdg_start_hold(void *group, const unsigned char triples[4*51],
                   const unsigned char voltages[4*17], uint64_t deadline_ns,
                   uint64_t *ticket, char *error, uint32_t size);
int sdg_initial_hold(void *group, const unsigned char zero_triples[4*51],
                     uint64_t deadline_ns, SDGResult *result,
                     char *error, uint32_t size);
int sdg_wait_hold(void *group, uint64_t ticket, SDGResult *result,
                  char *error, uint32_t size);
int sdg_wait_voltage(void *group, uint64_t ticket, SDGResult *result,
                     char *error, uint32_t size);
int sdg_output(void *group, uint64_t ticket, const unsigned char triples[4*51],
               uint64_t reply_deadline_ns, uint64_t write_deadline_ns,
               int validated_inputs, SDGResult *result, char *error, uint32_t size);
void sdg_cancel(void *group);
int sdg_cancel_snapshot(void *group, SDGResult *hold, SDGResult *voltage,
                        SDGResult *output, uint32_t *phase_bits,
                        char *error, uint32_t size);
int sdg_close(void *group, SDPairOwnerSettings restored_settings[4],
              char *error, uint32_t size);
```

1. The caller performs identity, watchdog, STOP and enable preflight through
   the ordinary/subset APIs **before** transferring Sessions. Four distinct
   `(first_id,mask)` combinations must cover IDs 1–12 exactly once, in any
   physical enumeration order. Each borrowed Session must have gap 900 µs,
   window 3, identical boot and valid nonblocking FD. All four CPU masks and
   positive timer slack are explicit. Linux owners verify affinity and timer
   slack by readback. Placement failure restores and joins without any wire
   write. All-zero placement/slack is accepted only for socket-only CPU test
   fixtures; this mode cannot accept serial FDs.
2. Optional `initial_hold` returns a fresh raw zero-gain sample once after
   owner configuration/READY and before any regular ticket. All twelve Type1
   commands must have Kp/Kd exactly zero. The result has ticket 0; no voltage
   is requested. This replaces startup's **actual sample**, never its clock
   alone, so initialization time cannot become a fictitious fresh input.
   Application checks remain unchanged. `start_hold` privately copies and
   validates **all eight batches**, including
   masked ascending Type1 triples and exactly one Type17 voltage request per
   port, before releasing any owner. It returns a strictly increasing ticket.
   Persistent native workers independently perform hold then voltage, with
   the original absolute deadline; they do not return to Python between them.
3. `wait_hold` returns once for all four complete hold batches. The caller can
   validate/construct observations and run IMU/inference while native voltage
   requests continue. `wait_voltage` returns once after `wait_hold`. Wrong,
   duplicate, stale or unordered waits are rejected without new I/O.
4. The caller validates raw per-port replies and all existing application
   guards, then calls `output` with `validated_inputs=1`. Both input phases
   must have completed and been consumed successfully under the exact ticket.
   All output triples are copied/validated before publication. Each original
   write retains the earlier write deadline and replies retain the later
   deadline. The returned raw/stamped results do not grant output admission.
   A new cycle is permitted only after successful output completion.
   A rejection **before publication** returns current-ticket API attempt
   metadata with real submission/failure-collection/cancellation stamps;
   `released_ns` and `validated_ns` remain zero. Each port has status -1 and
   the rejection reason but zero request count, owner stamps, records and
   stats. This is a **not-issued call**, never an observed native exchange or
   ACK. The caller journals it separately and preserves the primary error;
   it must not replace/bootstrap or create a raw output row. The actual group
   output buffer and publication bits remain untouched. The subsequent
   cancel snapshot returns only the phases that really were published.
5. Any native exchange failure sets sticky shared cancellation, interrupts
   siblings and prohibits later output. The failure's ordinary Session poison
   and outstanding-Type2 ambiguity persist. Results retain the native error,
   stats, partial writes, rejection bytes and stamps. No active retry occurs.
6. Before destroying a group, use `cancel_snapshot` to preserve native work
   that was never collected by Python. It sets sticky cancellation and waits
   for every issued owner to settle before copying buffers. Its return 0 means
   only that snapshot collection succeeded, never that any native phase
   succeeded. Owner return codes/partial writes/rejections remain unchanged.
   `phase_bits` identifies actual publication: 1 hold, 2 voltage, 4 regular
   output, 8 initial zero hold (initial raw uses the output buffer/ticket 0).
   Absent buffers are zero with their bits clear and are **not observations**.
   Previously consumed raw images are unchanged; original collection time is
   retained. Cancellation metadata can change. Journal each raw port once by
   `(ticket,phase,port)`; compare raw records/stats/owner stamps when deduplicating,
   not subsequent collection/cancellation metadata. No previous-cycle output
   is presented as a new cycle's output. Workers stay reserved until close.
   On success, failure or external cancellation, call `close` before existing
   bounded `sda_emergency_stop_subset`. Close cancels in-flight work, joins all
   owners, restores placement/slack and clears the borrowing reservation.
   The four native Sessions are terminally poisoned before ownership is
   released: raw ordinary active/read exchanges cannot reactivate them after
   close, including close of an otherwise idle group. Only the existing
   emergency STOP cleanup follows this handback.
   Even restoration failure destroys the group and releases Session ownership;
   its return is `-1` and readback rows identify failure. Repeated terminal STOP
   remains the unchanged subset API and cannot clear sticky ambiguity. The
   caller must keep Sessions and all descriptors alive until close completes.
   Existing Session destruction refuses a borrowed Session. Group handles are
   invalid immediately after close. Concurrent group operations are rejected;
   cancel is the only asynchronous operation.

All deadlines are absolute native monotonic nanoseconds, at most 250 ms into
the future for a phase. This is the unchanged kernel's argument bound, not
the real controller's 20/33.33 ms admission budget. The caller must retain
the stricter application deadlines. Borrowed Sessions cannot use ordinary,
subset or pair exchange APIs until group close. No fallback is selected.
