# Dedicated diagnostic owners: source-only candidate

This default-off experiment proposes **three total persistent workers**: one
front bus owner, one rear bus owner, and one IMU/voltage-validation owner. Each
has its own `ThreadPoolExecutor(max_workers=1)` queue. The main caller starts all
three before a measured schedule and receives the executor's original Future
for each task. There is no extra owner, reader, Future wrapper, native task or
model worker in this module. It imports only the Python standard library.

The default command is a file-only PLAN, which creates no workers:

```sh
python3 runtime/experiments/diagnostic_dedicated_owners/owners.py
python3 runtime/experiments/diagnostic_dedicated_owners/owners.py --dedicated-owners
PYTHONPATH=runtime python3 -m unittest discover -v \
  -s runtime/experiments/diagnostic_dedicated_owners -p 'test_owners.py'
```

Both PLAN commands create no worker, model or device. The second names the
proposed selection. There is no live selector, profile grant or benchmark hook;
existing production files and frozen source kits are untouched. The deterministic
tests exercise queues and lifecycle only. They show that a blocked IMU task and
its queued validation task do not occupy either bus queue. They do not establish
an OS/GIL cause, a speed improvement, CAN performance or whole-cycle qualification.

## Integration plan for a separate diagnostic source

Keep the original 900µs/window3, 26 requests, original 20ms absolute deadline,
real actor and normal observer, unchanged voltage/age/mode/fault/STOP checks,
10+10 selected warmup/reset, CPU/slack/switch/GC controls, and all raw evidence.
The scheduler changes submission ownership only. `task_deadline_ns` is evidence
metadata; the original deadline must still be passed to/enforced by the unchanged
task and collector. No clock, takeout or native reply becomes timely by being
queued on a different owner.

| Handoff | Dedicated queue | Preserve |
|---|---|---|
| Administrative all12 STOP prime | front / rear, one task each | Original own deadline, genuine raw replies, outside measured cycles |
| Six feedback then separate rotating Type17 voltage | matching front / rear | One original `_feedback_then_voltage` task per bus; six-record bridge Future remains distinct from genuine full owner Future |
| Gated voltage variant, if explicitly selected | matching front / rear | Original gate/cancel events and failure settlement; no early owner reuse |
| IMU read | imu_validation | Original read timestamps, frame/freshness validation and Future |
| Voltage validation during main inference | imu_validation, after the IMU task completes | Wait exact bus-owner voltage Futures; retain immutable raw records and original final freshness gate |
| Main inference / observer | main caller | No inference task added; full original model guards and clocks |
| STOP-proxy decoded output | matching front / rear | Original per-bus session, Future seal, native/decoder errors, readiness/cancel priority and absolute deadline |
| Emergency STOP after a failure | matching front / rear | Signal original cancellation/release voltage gates, settle prior owner/raw evidence before reuse; never cancel a required STOP as generic queued work |
| Restore and shutdown | same worker per owner, then main joins all three | Worker finalizer runs on its original thread after queued tasks settle; close sessions/FDs only after owner joins |

The current `_BorrowedDiagnosticOutput` requires one exact prestarted shared
three-worker executor. **Do not present this scheduler as that executor or claim
`borrowed_executor_owner=original_collector` for a new topology.** A new explicitly
selected diagnostic adapter must route its existing `_exchange_decoded` tasks to
the matching owner and preserve current original-Future seals, raw journal,
notification/poll source binding, and emergency teardown. Its `submit_decoded`
must still accept exact all12 STOP batches only. Existing pair/split7/live routes
are outside this experiment.

Any later integration needs its own source inventory/profile/cadence binding and
current source-bound tests/diagnostic evidence. Truthful topology markers must
name `three_separate_single_worker_queues`, actual owner-to-native-thread mapping,
actual worker count **3**, original executor Future identities, initializer and
same-thread restore readbacks, queued/running task settlement, and per-owner
feedback/voltage/validation/output dispatch clocks. Retain original bridge/full
Future distinctions and honest `actual_release → cycle_end` timing. A Future's
readiness remains separate from its result, validation and admission. Mark
output/Type1/live qualification, speedup and OS/GIL cause as unproved.

`cancel_pending()` cancels only not-yet-running executor Futures; it does not
signal native cancellation, settle raw evidence, send STOP, release a voltage
gate, kill a running owner or close a device. The integrating caller owns those
existing contracts before `close()`. Initializers must roll back their own
partial setup if they raise, as a failed worker cannot run its finalizer.

After the caller finishes its original current-generation joins, callback
fences, validation and raw journaling, `release_settled(futures)` retires the
scheduler's identity records without modifying the genuine Futures/results.
Keep current identity seals until that point; do not accumulate complete native
results in the scheduler for every historical cycle. Release work performed
during a cycle remains inside its honest clock. A `.done()` check here is only
a prerequisite for retiring references, never an output or owner-reuse proof.

Keep future evidence separate from existing shared-pool measurements. Compare
first cycle and steady cycles, all26 raw requests/replies, input→all12 host
writes/replies, STOP-proxy versus terminal STOP, deadline/skips/start intervals,
model/observer work, source pins and complete restoration. This experiment does
not change current hardware authorization or add a user permission gate.
