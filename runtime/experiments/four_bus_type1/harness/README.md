# Four-bus Type1 motor-free timing harness

This is a measurement harness only.

- It never opens `/dev/ttyUSB*`, `/dev/serial/*`, I2C, a motor or an IMU.
- It never writes `/sys`, needs no root and makes no network connection.
- Its only I/O partners are four anonymous socketpairs to its own synthetic peer process, and an optional injection control pipe to that process.
- Nothing it reports is a timing qualification or an output approval.

It runs the real `type1_runner.run` and the real `Type1Transport.create` over the genuine subset-active native library, built by `--build-library` or passed with `--library`. That includes `sda_subset_exchange` and, for F3, the native `sda_subset_exchange_at`. Pre-armed holds are never emulated in Python: F3 refuses a library whose `build-record.json` lacks `exchange_at_abi: 1`.

What is synthetic:
- the motors (`peer.py`, with reply delays drawn from recorded hardware in `latency_model.json`);
- the IMU device;
- the current guard;
- the observer (`synthetic`, or `torch-policy`, which loads the real checked policy module read-only).

| File | Role |
|---|---|
| `run_harness.py` | Entry point. Forwards the opt-in runner options. Records per-cycle rows, `harness-report.json`, `cycles.csv` and `peer-log.json`. |
| `peer.py` | Synthetic RS05 ×3 peers on four socketpairs, in a separate process (CPU5 on Linux). |
| `injection.py` | Deterministic fault injections (hold tail, voltage tail, host stall). |
| `analyze.py` | Offline summary per run and per configuration, with cause attribution and acceptance criteria (emulation only). |
| `latency_model.py`, `latency_model.json` | Recorded USB2CAN/RS05 reply delays. |
| `test_harness.py` | Quick tests. The end-to-end part builds the library once and runs two 2 s harness runs. |

## Options (all opt-in; without them the run is the reviewed default)

The runner options are forwarded unchanged into the admitted `pacing`, exactly as `type1_profile.cli_options` builds it:

| Flag | Fix |
|---|---|
| `--command-phase-offset-us K` | F1 (9000..11540; with F3 at most 10710 − max(L, 1250), e.g. 9460 at L = 1000, 9210 at L = 1500) |
| `--decode-once` | F2b |
| `--prearmed-hold-lead-us L` | F3 (300..2000) |
| `--gc-freeze` | F4 |
| `--timing-evidence` | F0 (diagnostic only, not a contract input) |

**Envelope gap modes** (unchanged):
- `--envelope-gap-mode strict` (default): the real behaviour. The first command or sample interval over 21 ms aborts the run.
- `--envelope-gap-mode measure`: the interval is recorded as a would-be violation, and only that one gap comparison is skipped. Every other check and the 21 ms value are unchanged. A measure-mode run is not a valid run.

**Deterministic injections.** None by default; with none selected, nothing is wrapped and no control pipe exists.

- `--inject-hold-tail PORT:CYCLE:+MS`: the peer on PORT answers requests 2 and 3 of cycle CYCLE's hold burst MS later than the model draws. This is the r2 cycle 70 shape (`port3:70:+1.8`).
- `--inject-voltage-tail PORT:CYCLE:+MS`: the same for cycle CYCLE's rotating Type17 voltage reply on PORT.
- `--inject-host-stall CYCLE:MS[:gil|sleep]`: the main thread stalls for MS, starting at release + 0.16 ms. This is the r1 cycle 26 location: after Boundary 1 and the hold gates, before any hold write.
  - Default path: the stall happens in the first hold `submit` of that cycle.
  - F3: it happens right after the actual release wait returns, before the IMU submit.
  - `gil` (default): the GIL is held while the thread is off-CPU, like a preempted GIL holder.
  - `sleep`: the GIL is released, like a descheduled main thread.

How injections behave:
- Tails are armed through the control pipe before the cycle's release wait. The latency-model draw sequence is unchanged, so a seed replays the same way with and without an injection.
- The report records what was requested, sent, applied by the peer and performed (actual stall length). Rows carry `inject_*` fields.
- `analyze.py` labels causes `injected_*`.
- Limits: tails up to 10 ms, stalls up to 15 ms.

**Rows report these separately:**
- `natural_gate_after_release_ms`: the final-gate end, always the natural gate;
- `command_after_release_ms`: the time actually given to `envelope.step`. With F1 this is the paced time; without it, it equals the natural gate;
- `hold_first_write_after_release_ms`;
- `output_last_reply_after_begin_ms`.

## macOS quick check (from `runtime/`)

```
PYTHONPATH=. python3 -B -m unittest experiments.four_bus_type1.harness.test_harness
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.harness.run_harness --output /fresh/dir \
    --build-library --duration 2 --envelope-gap-mode measure
python3 -B experiments/four_bus_type1/harness/analyze.py /fresh/dir
```

Caveats on macOS:
- It uses a Python release/command wait, labelled `PORTABLE_MACOS_VALIDATION_ONLY`. The native one oversleeps there.
- The native `exchange_at` wait also oversleeps there: release to first write was 0.0–0.39 ms with F3.
- Stalls oversleep by about 1–1.5 ms; the actual length is recorded.
- None of this is Jetson evidence.

## Jetson (motor-free; Motor 40 V off; no device opened; no root)

Run these on the Jetson in the account's own shell. The harness pins itself to CPU 0–4 and the peer to CPU5.

`--cpu-keepalive 0,1,2,3,4` with `--peer-busy-spin` only **emulates** the canonical power scope. If the canonical root power scope (`tools/jetson_latency_power_scope.py`, sudo) is established by the operator, omit `--cpu-keepalive`.

### 0. Variables and kit

```
D=$LOGS/harness-fix-r1          # LOGS: your private Jetson log directory
V=$VENV/bin/python3              # VENV: the Jetson policy venv
M=$MODEL_PROFILE                 # pinned checked model profile.json
mkdir -p $D/runs
```

Copy the current `runtime/` tree, including this `harness/` directory, to `$D/kit-runtime`, for example from the Mac:

```
scp -r runtime "$JETSON:$D/kit-runtime"   # JETSON: user@host of the robot
```

### 1. Tests and one library build (every run pins the same SHA256)

```
cd $D/kit-runtime
PYTHONPATH=. $V -B -m unittest experiments.four_bus_type1.harness.test_harness \
  experiments.four_bus_type1.test_subset_active experiments.four_bus_type1.test_type1_transport \
  experiments.four_bus_type1.test_type1_profile experiments.four_bus_type1.test_type1_runner \
  experiments.four_bus_type1.test_type1_foreground experiments.four_bus_type1.test_type1_options
PYTHONPATH=. $V -B -m experiments.four_bus_type1.build --output $D/type1-build --build
L=$D/type1-build/libdog_four_bus_type1_transport.so
LSHA=$(sha256sum $L | cut -d' ' -f1)
grep -q '"exchange_at_abi": 1' $D/type1-build/build-record.json && echo exchange_at-ok   # F3 needs it
```

### 2. Run matrix (seeds 101–103, 20 s = 989 cycles each)

```
cd $D/kit-runtime
# F0 (--timing-evidence) is diagnostic only and is kept in every run so configurations compare alike.
COMMON="--library $L --library-sha256 $LSHA --duration 20 --observer torch-policy --model-profile $M \
  --peer-busy-spin --cpu-keepalive 0,1,2,3,4 --timing-evidence"
run() {  # run NAME SEED MODE LATENCY [OPTIONS...]
  name=$1 seed=$2 mode=$3 latency=$4; shift 4
  PYTHONPATH=. $V -B -m experiments.four_bus_type1.harness.run_harness --output $D/runs/$name-s$seed \
    $COMMON --seed $seed --envelope-gap-mode $mode --latency $latency "$@"
}
F1="--command-phase-offset-us 11250"
F2="--decode-once"
F3="--prearmed-hold-lead-us 1000"   # measure 1000 and 1500 before choosing
F1C="--command-phase-offset-us 9400"  # F1 with F3: the lead-aware static check allows K <= 9460 at L = 1000
for s in 101 102 103; do
  run base        $s measure empirical
  run f1          $s measure empirical $F1
  run f1f2        $s measure empirical $F1 $F2
  run f1f2f3      $s measure empirical $F1C $F2 $F3
  run f1f2f3-l1500 $s measure empirical --command-phase-offset-us 9200 $F2 --prearmed-hold-lead-us 1500
  run f1f2f3-med  $s measure median    $F1C $F2 $F3
  run f1f2f3f4    $s measure empirical $F1C $F2 $F3 --gc-freeze
done
run f1f2f3-strict 101 strict empirical $F1C $F2 $F3
```

### 3. Injection replays

- Cycle 500 is an inference cycle.
- With 20 s runs, gain-down starts at `runner.stop_at_s` = 19.26 s, which is cycle ≈ 963. Cycle 970 is a gain-down cycle.

```
for c in 500 970; do
  run inj-r2hold-base-c$c 101 measure empirical          --inject-hold-tail port3:$c:+2.0
  run inj-r2hold-f1-c$c   101 measure empirical $F1 $F2  --inject-hold-tail port3:$c:+2.0
  run inj-volt-f1-c$c     101 measure empirical $F1 $F2  --inject-voltage-tail port3:$c:+1.5
  run inj-both-f1f3-c$c   101 measure empirical $F1C $F2 $F3 --inject-hold-tail port3:$c:+1.8 --inject-voltage-tail port3:$c:+1.5
  for m in gil sleep; do
    run inj-r1stall-$m-f1-c$c   101 measure empirical $F1 $F2     --inject-host-stall $c:2.6:$m   # fault expected (recorded)
    run inj-r1stall-$m-f1f3-c$c 101 measure empirical $F1C $F2 $F3 --inject-host-stall $c:2.6:$m   # must show no fault
  done
done
```

### 4. Analysis

```
$V -B experiments/four_bus_type1/harness/analyze.py $D/runs/* --json $D/analysis.json
```

**Per run**, the analysis reports:
- the command and sample interval distributions (median, p99, p99.9, max, std) and the counts over 20.8 ms and over 21 ms;
- release → first hold write;
- release → natural gate and release → command;
- output last reply − begin;
- cycle end (post-reply admission) → next release;
- status and abort error;
- cause attribution for every interval over 21 ms. An owner counts as starting late when its native hold begins more than 1.6 ms after the Boundary-1 check ends; with F3 (owners begin natively at the release, Boundary 1 runs before the wake lead) it is more than 0.5 ms after the release.

Per-cycle `current_check1/2/3` are Boundary 1, 2 and 3 of that cycle. Guard calls are assigned from the cycle's pre-arm wake (F3) or begin to the next cycle's; an F3 cycle that aborts before its release keeps `begin_ns` null and is windowed from its wake.

**Per configuration**, seeds are pooled and the same values are reported, plus aborts and the synthesis acceptance criteria. These are evaluated as harness emulation only:

| Measure | Required |
|---|---|
| Runs | All COMPLETE |
| Command and sample intervals > 20.8 ms | 0 |
| Command interval max | ≤ 20.5 ms |
| Sample interval max | ≤ 20.5 ms (F3: ≤ 20.2 ms and std ≤ 0.05 ms) |
| F1 command intervals | ≥ 99.5 % within 20.00 ± 0.01 ms |
| Natural gate | p99.9 ≤ K − 0.25 ms, max ≤ K + 0.5 ms (without F1: p99.9 ≤ 11.0 ms) |
| Release → first hold write | ≤ 1.3 ms (F3: ≤ 0.2 ms) |
| Output last reply − begin | ≤ 19.0 ms |
| Post-reply late cycles | 0 |
| Iteration | ≤ 18.5 ms |
| F3: cycle end → next release | min ≥ max(L, 1.25 ms): the next cycle's wake at release − L, Boundary 1, the gates and owner prep fit |
| Steady cycles | ≥ 3 × 988 = 2964 (three complete 20 s runs of 989 cycles; cycle 0 has no interval) |

An injected configuration is judged only on "no fault": COMPLETE, and no interval over 21 ms.

Do not use `--no-pin` or `--uclamp-min` for acceptance runs.
