# Receive-gap experiment (offline only)

`contract.py` has no I/O and is not imported by the live controller. It models
an already-written transaction through a 20ms soft boundary and one additional
slot. It never authorizes a new motor write or policy inference, even after
the original replies arrive. It returns `RESAMPLE_REQUIRED` instead.

Run deterministic tests with:

```sh
PYTHONPATH=runtime:runtime/tests python3 -m unittest test_receive_gap_contract
```

Three additional socket tests in `test_native_active_transport.py` exercise
the real C++ parser with a single delayed reply, delayed CRLF, and a full loss.
Their 40ms exchange budget is confined to the offline test. The production
runtime still uses its reviewed 20ms absolute deadline and fail-closed state.

The prototype cannot resolve a genuinely lost unsequenced Type2 response by
discarding it and sending a new Type1. Startup, disconnect, malformed frames,
faults, wrong mode, partial writes and exhausted skip frequency must stop.
Before any live integration, account for host/device watchdogs, input age,
model history, resampling and remaining output time; do not clear poison or
backdate timestamps. See the [investigation](../../../docs/can-usb-loss-investigation-20260929.md).
