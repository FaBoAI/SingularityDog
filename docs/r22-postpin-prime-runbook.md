# R22 STOP-proxy startup-prime comparison

R22 is a new private copy of the R18 kit. Its model, calibration, UID file and
saved report are byte-identical to R18. The source overlay adds an opt-in
`--post-pin-policy-prime-calls 1..100` to the native diagnostic. It requires
`--pre-cycle-policy-warmup-calls`, `--main-thread-cpu`, STOP-proxy policy
inference and at most 500 cycles. The existing 10-call pre-pin warmup runs
first. The extra calls use the observer's six persistent CPU input buffers
after the affinity check. A single observer reset follows those calls, then
cycle 1 starts with newly acquired CAN and IMU data. No extra sensor or STOP
cycles are hidden before the measured 500.

Before any live CAN run, verify the copied kit manifest and run the file-only
proof from the new kit root with the existing CPU PyTorch Python:

```sh
python3 -B tools/prove_postpin_prime_file_only.py --prime-calls 10
```

The proof checks the CLI's 0-call baseline (flag omitted) and 10-call plan,
replays the R18 saved sensor snapshot through separate reset policy instances,
and requires exact input, action, observation and model-state parity. It
installs a serial-port-open trap; any attempted open fails the proof. It does
not read live CAN/IMU or load the native transport library. If the private
R21 500-cycle `records.json` is available on the same machine, add
`--saved-records PATH --saved-records-sha256 SHA256`. This compares every saved
model input after the final reset, including the first three calls, and prints
local model-call first-three, p99 and maximum durations. That optional
sequence uses the R18 bundled policy because the R21 cached-view binary is
not in this source-only kit; those local times cannot predict Jetson cycles.

For the live disabled, supported STOP-proxy A/B comparison, hold the R21
settings fixed: MAXN_SUPER, CPU4, pre-pin warmup 10, automatic GC deferred
during 500 trace cycles, output-dispatch trace, and the same CAN window/gap.
The baseline omits `--post-pin-policy-prime-calls`; the candidate adds
`--post-pin-policy-prime-calls 10`. Use separate new output directories and
recheck the independently supported disabled state before each trial. A
candidate report must show `plan.post_pin_policy_prime_calls=10` and
`setup_policy_prime.complete=true`, `iterations=10`,
`observer_reset_after=true`, `sensor_cycles=0`, `stop_writes=0`, and
`setup_policy_prime.end_ns <= measurements[0].release_ns`. Reject an aborted
run or incomplete/old sensor inputs. Compare cycles 1–3 separately, then all
500: model-call section, last host write, last STOP reply, whole iteration,
release interval, p99, maximum and every >20 ms count. Preserve all misses;
497 steady cycles under 20 ms in R21 did not prove a complete 500-cycle pass.

This is a no-learned-output diagnostic. Priming performance on Jetson is
unmeasured until the new live comparison completes. Neither this proof nor a
STOP-proxy result approves active learned-target output or full-controller
50 Hz operation.
