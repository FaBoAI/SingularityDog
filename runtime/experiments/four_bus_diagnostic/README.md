# Four physical USB2CAN buses: STOP-only diagnostic candidate

This independent experiment does not enable motors, encode Type1 targets,
grant output approval, or reuse the two-bus timing/profile qualification.
The command-line entry is PLAN only. Execution requires an explicit supervisor
calling `pipeline.run(..., execute=True)` with current, source-bound factories,
model/snapshot guards, operating scopes and caller-owned descriptors.

Each physical port must contain exactly one of `{1,2,3}`, `{4,5,6}`, `{7,8,9}`,
or `{10,11,12}`. Port order comes from current read-only UID discovery. The
whitelist compares complete canonical 17-byte wires, including flags/payload.
Cross-group frames, enable/configuration/Type1 and arbitrary parameter requests
are rejected before the native exchange.

| Timed phase | Requests per physical port | Owners and readiness |
| --- | ---: | --- |
| Feedback | 3 canonical Type4 STOPs | Four persistent dedicated CAN workers |
| Rotating voltage | 1 Type17 voltage | Same physical worker and native session |
| Inference | None | Main thread, ordinary observer and actual selected model |
| Output proxy | 3 canonical Type4 STOPs | Four original output Futures |

The four ports send **28 requests per cycle**. The original 900µs gap,
window3 and absolute 20ms/input-age deadlines remain. Three feedback replies
publish an owned prefix, while the original full Future retains the physical
owner through voltage. Main joins all four full Futures and revalidates all12
feedback values, raw images, four current voltage replies, cached12 voltage
ages (original126ms), IMU and snapshot before output. Inference targets are
recorded but never transmitted. Three startup voltage reads per port and the
independent final STOP are separately recorded, outside the per-cycle28.

`model_bridge.py` prepares a new four-bus measured-input profile and snapshot
from actual current UID/mode/current/voltage/three-position raw. Historical
model/calibration files are dependencies; they do not establish a current boot,
power interval, pose, physical clearance or timing result. `topology.py` grants
no STOP confirmation or output qualification. The original pure C++ feedback
decoder is fixed-six, so the three-record batches truthfully use the original
Python parser. No dummy records or fabricated six-exchange Stats are inserted.

The native session still uses the original six-axis half-envelope with zero
Kp/Kd caps. `subset_stop.cpp` includes the unchanged ordinary transport and
adds only optional ABI1 exact-three recovery. Its masks are7 and56; its output
storage remains six record/fault slots with unselected slots untouched/zero.
It never calls the original fixed-six emergency routine. Cancel/boot do not
prevent STOP, while native FD/mutex/paired ownership checks remain. Pending or
partial STOP replies add selected sticky ambiguity; later replies cannot erase
it. The separate20–500ms cleanup budget does not extend any active20ms deadline.

On failure, genuine original owners settle or are explicitly observed pending;
late success is cleanup evidence only. Recovery queues behind each original
physical owner, all workers join, same-thread scopes restore, and sessions close
before the supervisor can reuse descriptors. Worker, session or main restoration
errors retain the primary failure and make the overall result ABORTED.

The real supervisor must verify nice−10, mainCPU4, four CAN owners CPU0/1/2/3,
IMU/validation mask0..3, timer slack1000ns, switch interval100µs, single-thread
math, actual CPU/C7/EMC settings and GC behavior before device factories.
`LinuxWorkerScope` implements only worker affinity/slack and exact restoration;
it does not change power settings. The supplied main scope remains alive through
owner settlement, independent STOP, worker restoration and session close.
The supplied model setup retains pre10/post10 selected-wrapper warmup, original
reset/state checks, and records any temporary pre-pin CPU mask explicitly.

File-only build and tests from the repository root:

```sh
PYTHONPATH=runtime python3 -B -m experiments.four_bus_diagnostic.build --output "$FRESH_BUILD"
PYTHONPATH=runtime python3 -B -m experiments.four_bus_diagnostic.build --output "$FRESH_BUILD" --build
PYTHONPATH=runtime python3 -B -m unittest experiments.four_bus_diagnostic.test_pipeline experiments.four_bus_diagnostic.test_subset_stop
```

PLAN builds nothing and opens no model/library/device. `--build` writes only a
fresh output directory. Its build receipt pins both the included ordinary source
and extension source, compiler command and actual binary. For target socket tests,
set `FOUR_BUS_TEST_LIBRARY` to that exact newly built library; the test harness
then performs no build. Root must independently pin the source inventory and
build receipt before treating that test selection as a source-bound result.
All socket/PTY results, mock model/OS readbacks and real-model/CAN timing remain
separate. No finite test implies a hard real-time guarantee or standing/walking.
