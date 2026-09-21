# Offline safety guard

`singularitydog_hw/safety.py` is a software permission state machine. It has no
serial, CAN, I2C, motor encoder, hardware writer, or physical emergency-stop
implementation. A `stop_intent` decision/event requests handling by a future
control owner; it never means a stop command was sent or that hardware stopped.
There is no background watchdog. If the caller stalls or stops evaluating inputs,
this guard cannot act. Hardware stop and watchdog verification remain separate
commissioning requirements.

## API and required evidence

Construct `GuardConfig(sensor_max_age_s={...}, deadman_max_age_s=...)` with an
explicit, finite, positive freshness threshold for **every required source**.
There are no default robot timings or inferred mechanical/current/temperature
limits. Invalid configuration raises `ValueError` before creating a usable guard.
The required-source map is copied and made immutable.

Use a separate required source for every motor whose telemetry is needed, plus
the IMU and any other required sensors. One recent aggregate motor timestamp
must not hide an old or missing member. The caller is responsible for validating
device identity, expected vector dimensions, scaling, frames and physical ranges;
the guard checks presence, health assertions, finite values and freshness.

`Prerequisites` defaults all four assertions to false:

- `calibration_verified`: physical joint order, direction, offsets and applicable
  limits have evidence, not merely a user-reported ID list.
- `equipment_verified`: required devices and their associations have been checked.
- `motor_models_verified`: actual models/protocol variants and their units are known.
- `physical_stop_verified`: the physical stop path has been checked separately.

These flags describe evidence supplied by the caller. Setting them true does not
perform verification. In the current commissioning state, missing model,
calibration or physical-stop evidence must keep the respective flag false.

Each `SensorSample(timestamp, values, healthy=False)` needs a nonempty finite
numeric vector and a literal `healthy=True`. `DeadmanStatus(timestamp,
pressed=False)` needs literal `pressed=True`. Timestamps use the **same monotonic
clock domain** as the injected guard clock. Do not use wall-clock timestamps or
refresh timestamps merely because cached data was reread. If the source can
silently replay data, its acquisition layer must detect this and mark it unhealthy.

Call `arm(SafetyInputs(...))` explicitly to request permission. Call
`evaluate(inputs)` before each prospective control operation with current inputs.
Only its `permitted` field is the evaluated permission snapshot; `state` is
diagnostic metadata. A decision is not a reusable authorization token.
`valid_until` is the earliest sensor/deadman expiry; permission expires at that
instant (`now >= valid_until`). Re-evaluation may revoke it earlier, for example
when the deadman is released. No method in this module executes motor control.

## State transitions

| Current state | Input/action | Result |
| --- | --- | --- |
| Startup | Construction | DISARMED; permission denied |
| DISARMED | Fresh valid inputs to `evaluate` | Still DISARMED |
| DISARMED, no fault latch | Explicit `arm` with all gates satisfied | ARMED |
| DISARMED | Invalid arm inputs | Denied; reasons emitted |
| ARMED | Missing/unhealthy/nonfinite/stale/future data, released/stale/missing deadman, revoked prerequisites, failed/backward clock | FAULT; permission revoked and stop intent emitted |
| FAULT | Fresh data or `arm` | Still denied; original fault stays latched |
| FAULT | `clear_fault` | Denied; `disarm` required first |
| Any open state | `disarm` | DISARMED; permission denied; existing fault remains latched |
| DISARMED | `clear_fault` with a valid clock | Fault latch cleared; still DISARMED |
| ARMED | `clear_fault` without disarming | FAULT; fail closed |
| Any state | `close` | CLOSED; permission denied permanently |

Recovery requires **disarm → clear_fault → explicit arm with fresh valid inputs**.
Clearing explicitly resets the guard's monotonic-clock comparison baseline so an
acknowledged clock rollback can recover; the next arm still checks all timestamps
and prerequisites. Nonfinite/unavailable clocks prevent clearing. A new process
always starts DISARMED and must not restore an earlier armed state from a log.

`drain_events()` returns immutable transition records. Fault stop intent remains
visible in decisions until cleared; the transition event is emitted once.
Disarming or closing an armed guard also emits stop intent. No event claims
physical completion. Use this object from one control owner; it is not designed
for concurrent mutation from multiple threads.

## Offline verification

From `runtime/`:

```sh
PYTHONPATH=. python3 -m unittest discover -s tests -p test_safety.py -v
```

The suite uses an injected clock and synthetic samples. It checks default denial,
each prerequisite, stale and missing sources, future timestamps, nonfinite data,
deadman release, backward/invalid clocks, latched recovery, terminal close and
stop-intent events. Test freshness values are test fixtures, not approved robot
operating thresholds. No hardware access is involved.
