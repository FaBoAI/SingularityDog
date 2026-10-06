# Active relative-wait regression

The active exchange previously sampled the loop clock before a blocking boot-identity read and reused that clock to calculate a relative `pselect` timeout. The correction resamples immediately before the wait and fails through the existing poison/ambiguity path if the original absolute deadline has expired. Absolute deadlines, command validation, request gaps, session ownership, no-retry behavior, and emergency STOP handling are unchanged.

The named legacy fixture preserves the exact pre-fix active source. The test requires current production to differ by only the reviewed timeout block, then compiles both with every transport syscall replaced by deterministic fakes. No actual file descriptor, motor, CAN/USB device, shared library, or controller is opened. The harness exercises real session creation, command validation, failure poisoning, and emergency STOP logic with fake system calls.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/tests \
  python3 -B -m unittest test_native_active_transport_fresh_wait -v
```

A simulated 2ms boot read shortens the observed request gap from 9ms to 7ms for a configured minimum of 5ms; the separate required pre-write boot check still consumes 2ms. The fix does not remove that check. No-response deadlines stop earlier without retransmission. A partial Type1 write remains ambiguous even after new emergency STOP replies, so it cannot become a falsely confirmed stop.

These are source and deterministic regression results. They do not identify a particular robot delay, prove a hardware performance improvement, or qualify an existing controller profile. Historical frozen kits and binaries retain their original bytes; a new build needs its own validation.
