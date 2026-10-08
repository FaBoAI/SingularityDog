# Four-bus live Type1 (boxed) — implementation design

Status: design for implementation. Nothing here grants output permission. A
library, profile, test pass or timing result is never an approval; every real
run additionally needs the current direct-human condition record (section 6)
written only from the user's own chat statements in the current boot and
power epoch.

## 0. Ground rules

- **Additive only.** Do not edit any file pinned by
  `singularitydog_hw/policy_live_profile.py` `CADENCE_SOURCE_PATHS`
  (policy_output_runtime.py, policy_output.py, native_active_transport.py,
  policy_live_profile.py, policy_post_reply_timing.py, rs05_trial_protocol.py,
  can_readonly.py, experiments/native_active_transport/transport.cpp, encoder
  sources) and do not change behaviour of
  `experiments/four_bus_diagnostic/*`. Import and reuse them unchanged.
  New code lives in `runtime/experiments/four_bus_type1/`.
- The four-bus STOP-only diagnostic stays the default and all of its tests
  must keep passing unchanged.
- Hardware: four USB2CAN adapters, three RS05 motors each. Port→IDs come
  only from the current read-only topology capture `ids_by_port`
  (today: port0 7-9, port1 10-12, port2 4-6, port3 1-3), never from index.
  Each port's native session is the ordinary 6-slot half session
  (`first_id` 1 or 7) restricted by group mask 0x07 (slots 0-2) or 0x38
  (slots 3-5).
- Pacing is the R8-qualified four-bus configuration: request gap 900 µs,
  window 3, release spin 500 µs, main CPU4, CAN owners CPU0..3, IMU mask
  0..3, timer slack 1000 ns, switch interval 100 µs, single-thread math,
  separate acquisition (3 hold + 1 voltage per port, voltage overlapped
  with inference), boundary current checks, final-gate input identity.
  These are recorded explicitly in PLAN and report.
- Reports are truthful: `learned_targets_attempted`, `motor_enable_sent`,
  `type1_sent`, `positive_gain_sent` are set **before** the corresponding
  first write and never reset; physical observations stay `None`.

Reference reports: a private porting map of the two-bus native-pair path
(OR = policy_output_runtime.py, PO = policy_output.py, NAT = native_active_transport.py,
LP = policy_live_profile.py, ME = policy_motion_envelope.py, T = native transport.cpp,
PL/TA/SS/FG/MB = four_bus_diagnostic pipeline/transport_adapter/subset_stop/foreground/model_bridge).

## 1. Native extension — `subset_active.cpp`, `build.py`, `test_subset_active.py`

`subset_active.cpp` `#include`s `../four_bus_diagnostic/subset_stop.cpp`
(honouring the same `FOUR_BUS_ORDINARY_SOURCE` override), which includes the
unchanged ordinary `transport.cpp`. New exported symbols only; no struct
changes:

```c
uint32_t sda_subset_active_abi(void);   // 1 iff sizeof(SDRecord)==88 && sizeof(SDStopResult)==36
int sda_subset_validate(void *handle, uint32_t group_mask,
        const unsigned char *wires, uint32_t count, char *error, uint32_t size);
int sda_subset_exchange(void *handle, uint32_t group_mask,
        const unsigned char *wires, uint32_t count, int send_only, uint64_t deadline,
        SDRecord *records, SDStats *stats, char *error, uint32_t size);
        // exact same trailing parameters/semantics as sda_exchange
```

Validation (both functions; exchange validates first, writes nothing on
failure, sets `s->poisoned=true` under the session mutex on violation):
- `group_mask ∈ {0x07, 0x38}`; `1 ≤ count ≤ 6`; framing valid.
- Every wire's motor id `mid` satisfies `mid - s->first ∈ [0,6)` and bit
  `(mid - s->first)` is set in `group_mask`.
- Allowed kinds: 0 (identity), 1 (Type1), 3 (enable), 4 (STOP / version
  probe), 17 (read), 18 (write). No other kind.
- A batch containing any Type1 contains only Type1, and is either exactly
  one wire (single-axis zero-gain/handshake) or exactly three wires covering
  the mask's three IDs in ascending order.
- Every Type1 also passes the existing static `valid_request` against the
  session per-slot limits (q window, kp/kd caps, torque/vref 32767).
- Then `sda_subset_exchange` releases its validation lock and calls the
  existing `exchange_owned(...)` exactly as `sda_exchange` does (it re-takes
  the mutex and re-checks poison/cancel/boot/FD/caps).

`build.py`: fresh output directory only; `-std=c++17 -O2 -Wall -Wextra
-Werror -pthread -fPIC -shared`; pins and re-checks after compiling the
ordinary `transport.cpp`, `subset_stop.cpp`, `subset_active.cpp`; writes
`build-record.json` with scope `four_bus_subset_active.v1`,
`allowed_masks [7,56]`, `type1_exchange_abi 1`, `output_allowed false`,
binary sha256, compiler command, sources. Same style as
`four_bus_diagnostic/build.py`. The STOP-only loader must keep rejecting
this receipt and vice versa (different scope strings).

`test_subset_active.py`: socketpair fixture modelled on
`four_bus_diagnostic/test_subset_stop.py` (`FOUR_BUS_TYPE1_TEST_LIBRARY`
selects a prebuilt library; otherwise build into a temp dir). Peer replies
Type2 with mode 2 for Type1, mode 0 for STOP. Cover: foreign-ID
Type1/Type3/STOP rejected with no bytes written; mixed Type1+other
rejected; 2-wire Type1 rejected; unordered Type1 triple rejected; kp/kd
above cap and q outside window rejected; mode-0 reply to Type1 rejected;
valid triple accepted with mode-2 replies; subset STOP after a pending
Type1 reports it ambiguous; existing STOP-only exports still work.

## 2. Transport — `type1_transport.py`, `test_type1_transport.py`

```python
class Type1Transport:
    group: four_bus_diagnostic.transport_adapter.Group   # port, ids (3), first_id, mask
    journal: list            # (label, raw records/stats) for every exchange incl. failures
    @classmethod
    def create(cls, library, fd, *, group, cancel_fd, boot_fd, boot_id,
               axis_raw_bounds: dict[int, tuple[float, float]],   # member IDs only, raw rad
               kp_cap_by_id: dict[int, float], kd_cap_by_id: dict[int, float],
               cancel_all: callable) -> 'Type1Transport'
    # Preflight / handshake (all window semantics come from native):
    def identify(self, *, deadline_ns) -> dict[int, bytes]          # Type0 UIDs, members
    def stop(self, *, deadline_ns) -> dict[int, Feedback]          # 3x Type4, mode 0 fault 0
    def version_probe(self, *, deadline_ns) -> dict[int, bytes]
    def read_params(self, names, *, deadline_ns) -> dict[(int,str), value]   # Type17
    def write_watchdog(self, ticks, *, deadline_ns) -> dict[int, Feedback]   # Type18 0x7028
    def enable(self, mid, *, deadline_ns) -> Feedback              # Type3, mode∈{0,2}, fault 0
    def zero_gain(self, mid, q_raw, *, deadline_ns) -> Feedback    # single Type1 kp=kd=0, mode 2
    # Cycle:
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check)
        # wires must be byte-identical to this port's last validated output batch.
        # Publishes the 3-row hold Batch into prefix_future from before_native, then
        # one Type17 voltage for voltage_id; returns (hold_batch, voltage_batch).
    def output(self, wires, *, deadline_ns, check) -> Batch        # exactly 3 Type1, ids ascending
    # Termination:
    def stop_repeated(self, *, total_budget_ns=1_000_000_000, rounds=3) -> dict
        # repeated sda_emergency_stop_subset(mask); ambiguity union is sticky;
        # result {'complete', 'confirmed_ids', 'unconfirmed_ids', 'ambiguous_ids',
        #         'faults', 'rounds', 'physical_cutoff_required'}
    def close(self)
```

- Session: `NAT.ActiveSession(library, fd, first_id=group.first_id, …,
  gap_ns=900_000, window=3)`. Member slots get `axis_raw_bounds` and the
  profile kp/kd caps (≤3 / ≤0.15). The three non-member slots get kp=kd=0
  and an unreachable raw window strictly between two adjacent u16 codes
  (`lo = q(c)+LSB/3`, `hi = q(c)+2·LSB/3`, LSB=25.14/65535) so no Type1 can
  ever decode inside it.
- Every exchange goes through `sda_subset_exchange` with the group mask
  **and** a Python whitelist (exact kinds/ids per method). Required reply
  mode per method: STOP 0; enable {0,2}; Type1 (zero_gain/hold/output) 2;
  all fault 0. Decode with the Python `OR.decode_records` path (not the
  native fixed-six decoder).
- Ownership: one owner thread per transport (like TA `_owner_check`); the
  library/receipt is verified and symbols resolved with explicit argtypes
  and sealed (pattern of TA `verify_library`).
- On any exception inside an exchange the transport poisons itself and
  calls `cancel_all()` (writes the shared cancel byte) before re-raising,
  so sibling ports stop writing immediately.
- Tests: MockSession-based unit tests for whitelist/mode/ownership/
  cancel_all; plus native integration over socketpair when
  `FOUR_BUS_TYPE1_TEST_LIBRARY` (or a temp build) is available.

## 3. Run profile and admission — `type1_profile.py`, `test_type1_profile.py`

A four-bus Type1 run profile JSON (schema
`singularitydog.four-bus-type1-boxed-profile.v1`) built by a file-only
`prepare` CLI from pinned inputs (path+sha256 each):
- current four-bus read-only topology capture + events (boot id, power
  epoch, ids_by_port, UIDs) — must be `COMPLETE_READONLY_TOPOLOGY_CANDIDATE`;
- the existing reviewed boxed axis geometry: per-axis `offset`, `sign`,
  `physical_lower_rad/upper_rad` (≤ ±3° window), `start_pose_bounds`
  (≤1° wide), from the pinned reviewed profile used by today's two-bus boxed
  2 s/10 s runs (re-validate numerically; UIDs must match the topology);
- the four-bus model plan (`four_bus_diagnostic.model_bridge`) and checked
  model references;
- fixed caps identical to `LP._preauthorized_boxed_axis_caps` (call it on
  the profile dict): kp 3, kd 0.15, displacement ≤1°, PD est ≤0.1 Nm,
  measured τ ≤1 Nm, vmax 1°/s, amax 5°/s², tracking ≤2°, temp ≤45 °C,
  measured v ≤0.35 rad/s; `policy_weight .005`, `h_hypothesis 0`,
  `command [0,0,0]`, startup damping 0.08 s, post-reply policy
  `bounded_post_reply_v1` (1 ms / 1 consecutive / 1 per 100), voltage 35–42 V,
  voltage max age 126 ms;
- `mode ∈ {'zero_gain_timing', 'learned_boxed'}` and `duration_s ∈ {2,10,20}`;
- predecessor chain (learned_boxed only): 2 s requires a COMPLETE
  zero-gain-timing report of the same boot/epoch/source with all-12 STOP
  confirmed; 10 s requires COMPLETE learned 2 s; 20 s requires COMPLETE
  learned 10 s **and** a new four-bus STOP-proxy 501-cycle diagnostic
  COMPLETE after the 10 s run. Every step also requires a fresh read-only
  topology capture taken after the predecessor's terminal STOP.
  zero_gain_timing requires a current-boot four-bus STOP-proxy diagnostic
  COMPLETE (501 cycles) bound to the same capture lineage.
- `contract_sha256` over everything except duration and evidence.

Admission (`admit(profile, conditions)`) additionally requires the direct
human current-condition record (section 6) for the same boot/epoch and
contract, and returns an immutable admitted object used by the runner
and foreground. Validation helpers for reports of the predecessors live
here too. Tests use synthetic files.

## 4. Runner — `type1_runner.py`, `test_type1_runner.py`

`run(admitted, *, factory, imu_read, observer, check_current,
check_cancelled, cancel_io, model_setup, worker_scope, main_scope,
release_wait, announce, encoder, clock=time.monotonic_ns, execute=False)`
— PLAN when `execute` is not True (opens nothing). All capabilities are
injected so the whole run is testable with mocks (pattern of PL.run).

Sequence (port-keyed equivalent of OR phases A–J; cite and mirror the OR
lines):
1. Gates; 4 per-port single-thread pools + IMU pool; worker placement
   readback (CPU0..3, IMU 0..3, slack 1000); main scope (CPU4).
2. Preflight per port (concurrently across ports, each port serial):
   Type0 UID == topology; STOP mode 0 fault 0; version probe fingerprint;
   Type17 run_mode == 0 (no mode write) and voltage 35–42; Type18 watchdog
   4000 then readback == 4000; Type17 position/velocity then STOP → unique
   branch, offsets, `starts` (raw Type2 position).
3. Announce (injected), then all-axis `can_timeout` readback again; model
   warmup 10 + prime 10 (`model_setup`); pose recapture with STOP within
   0.02 rad of starts; `trial_origin`; IMU + `validate_inputs`; duration
   reserve check; all-axis voltage refresh.
4. Enable: 12 serial single-axis steps interleaved across ports
   (port3,port2,port0,port1 order by id: 1,4,7,10,2,5,8,11,3,6,9,12),
   each: voltage check, Type3 (30 ms), displacement check, watchdog kick,
   zero-gain Type1 at `starts[mid]` (20 ms, mode 2). Total ≤ 120 ms.
   All-axis voltage refresh after.
5. Initial hold: all-port zero-gain triple at `starts` → `q0` (mode 2);
   bounds ∩ origin±disp; `PolicyMotionEnvelope` (ME, unchanged);
   encoder bound; GC disabled; host `OutputWatchdog`-equivalent (40 ms,
   2 ms poll → emergency).
6. Cycles at absolute 20 ms epoch slots (`OR._absolute_epoch_slot`; skipped
   slot → abort/STOP). Per cycle: hold gates (cmd/sample gap ≤21 ms,
   previous replies ≤20 ms old, voltage before Type1) → submit per port
   `hold_then_voltage(last_wires[port], ids[cycle%3])` + IMU →
   boundary current check → feedback sample (mode 2,
   previous = last output replies), validate_measured(initial=trial_origin),
   voltage cache → `observer.consume(snapshot)` (Type1-hold snapshot:
   tx must be the exact previous wire, rx mode 2) → CAN_ORDER→ID remap,
   model-range check → blend `q0 + w(q−q0)` with w = .005·smoothstep5 ramp
   → local range check → voltage join + cache → final gate (input
   identity) + boundary current check → `envelope.step` (reject, never
   clip) → encode all 12 then quantized re-checks → split per port →
   voltage-before-Type1 + clock → `output` on all 4 ports → join by
   `hard_end` (or ≤+1 ms under the bounded post-reply policy, first-cycle
   rule) → validate output replies (range, |τ|≤1, |v|≤0.35, temp≤45,
   tracking≤2°, disp≤1°, PD≤0.1) → miss accounting → `previous =
   output replies`, `last_wires = outgoing`, kick watchdog.
   `hard_end = min(release+20 ms, first_input+20 ms)` is the single
   absolute native deadline for every write/reply in the cycle.
   `zero_gain_timing` mode: same schedule, but every output is the zero-gain
   Type1 at `q0` (kp=kd=0); the model still runs and its targets are
   recorded only. `learned_boxed`: normal envelope gains.
7. Ramp-down at `stop_at_s = duration − max_stop_s − 0.04` or on SIGUSR1:
   `envelope.request_stop()`, send until `stopped` (`graceful_stop`).
   Budget exhaustion without `stopped` is an error.
8. Emergency (latched once): set aborted → `cancel_io()` → join every
   in-flight owner Future → queue `stop_repeated` on **each port's own
   pool concurrently** → collect with a shared 1.25 s deadline. Normal
   completion also runs the terminal `stop_repeated` on all ports.
   `stop_confirmed` requires all 12 confirmed, no ambiguity; otherwise
   status `STOP_UNCONFIRMED_POWER_OFF_REQUIRED` and
   `physical_cutoff_required`.
9. Restore: watchdog, GC, affinity, slack, sessions; report with raw
   journals, per-stage timestamps per cycle (release, first write, last
   hold reply, gather, infer end, voltage join, envelope/encode end,
   output first write, last output reply, cycle end), cycle counts,
   flags. Final status `COMPLETE_FOUR_BUS_TYPE1_<MODE>` only if every
   cycle passed and STOP confirmed.

Reuse unchanged: `NAT.encode_motion`, `OR._python_motion_wires` or
`native_policy_batch_encode.VerifiedBatchEncoder` (split front/rear dicts
into ports: port with ids 1-3 = front[0:3], 4-6 = front[3:6], 7-9 =
rear[0:3], 10-12 = rear[3:6] — map by id, not position),
`PolicyMotionEnvelope`, `OR.feedback_sample`, `OR.validate_measured`,
`OR.checked_voltage_rows/cache`, `OR._absolute_epoch_slot`,
`PostReplyDeadlineBudget`, `rs05_trial_protocol` frame builders.

Tests: mock transports (mode-2 Type1 replies) driving full runs in both
modes; failure injection: late hold reply, mode-0 reply to Type1, fault
bit, out-of-range voltage, envelope rejection, model out of range,
slot skip, owner exception on one port (siblings cancelled, concurrent
STOP on all 4), cancel/SIGUSR1 ramp, STOP unconfirmed path, budget
exhaustion, exact request counts (28 per cycle), truthful flags.

## 5. Foreground — `type1_foreground.py`, `test_type1_foreground.py`

CLI mirroring `four_bus_diagnostic/foreground.py`: PLAN by default
(opens no device, loads no library/model); `--execute` runs. Inputs (all
path + sha256): source root/manifest/binding, admitted profile, conditions
record, topology capture/events, subset-active library + build record,
current-guard library + build record, optional C++ encoder binary,
announcement audio, `--mode`, `--duration`, `--output`. Reuses from the
diagnostic foreground: pins, startup readback, holder check, port locks,
cancel binding, signal handlers (SIGUSR1 → graceful ramp), MainScope,
`read_imu`, `make_release_wait` (spin 500), current guard, settings
restoration, report writing (O_EXCL). Intended to be wrapped by
`tools/jetson_latency_power_scope.py --supported-characterization`.

## 6. Direct-human condition record

JSON (schema `singularitydog.four-bus-type1-current-conditions.v1`) with
`source: direct_current_user_reply`, `direct_human: true`,
`synthetic_interaction: false`, the verbatim `user_statement`, boot id,
power epoch, contract sha256, authorized mode/durations, and booleans all
true: `motor_40v_on`, `box_supports_body`, `four_feet_touch_floor`,
`all12_local_plus_minus3deg_clear`, `hands_off`, `immediate_40v_cutoff`,
`other_drive_tools_stopped`, `box_will_remain`; and all false:
`load_transfer_allowed`, `standing_allowed`, `walking_allowed`. A helper
writes it only from explicit arguments; nothing infers it.
