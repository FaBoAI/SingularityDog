# 20 ms model hot-path investigation, 2026-09-29

This is a file-only CPU experiment. It neither opens CAN/IMU devices nor
enables motors. It cannot approve a learned-output or walking run.

Earlier 501-cycle STOP-proxy diagnostics had only a small margin below
20 ms in steady state, with a few model-call outliers. A separate 500-input
replay on Jetson CPU5 isolated those outliers from CAN and IMU: the pinned
scalar model returned all 500 saved targets exactly, but model-call wall time
had a 1.966 ms median, 3.755 ms p99, and 4.094 ms maximum. The corresponding
thread-CPU maximum was 3.423 ms. For the three wall calls above 4 ms, wall
minus thread CPU was 0.67–1.14 ms. This establishes that both model CPU work
and descheduling can consume the cycle margin.

The existing experimental building blocks are the scalar recurrent step,
fused observation, and zero-request projection cache. The new
[`run_file_only.py`](../runtime/experiments/native_policy_overnight/full_recurrent_fusion/run_file_only.py)
combines those three *only in a private TorchScript candidate*. The runner
requires an explicitly SHA-pinned scalar library, a SHA-pinned baseline model
and saved inputs, produces all generated code/artifacts outside Git, and keeps
the output-approval flags false. It checks the unchanged actor and controller
state after each recurrent call, rejects the same invalid inputs, then times
interleaved candidate and comparator calls.

The final Jetson file-only run (report SHA256
`98ba1633ae24b3c3a41e34f77b1dc4c1f05f75f2533ce94537a40bd6b12f2209`)
is saved privately at
`/Users/akira/.codex/private-robotdog-backups/20260929-file-only-hotpath-r1/full-recurrent-fileonly-20260929-r3/report.json`.
Its copied files and hashes are recorded alongside it in
`artifact-sha256.json` (SHA256
`0b78d901577058fbcf3488f92f5161a3f8ea262ea82b795ffa6dee82009383e0`).
The run passed 500
sequential saved calls with exact target, observation, actor, every named
state/parameter, all 26 rejection reasons, and no input mutation. It also
passed 240 synthetic frames with recurrent resets and exact comparison of all
full-step diagnostic fields. Of 960 synthetic leg projection requests, 168
were nonzero. All 2,000 saved STOP-leg projection requests were zero.

| Jetson file-only model call, ms | Cached view | Combined candidate |
| --- | ---: | ---: |
| Wall median | 2.222 | 1.663 |
| Wall p99 | 2.444 | 1.880 |
| Wall maximum | 2.567 | 2.838 |
| Thread CPU median | 2.200 | 1.646 |

For the 240 synthetic frames, including nonzero translation and yaw commands,
paired wall medians were 2.254 ms (cached view) and 1.695 ms (combined). The
p99s were 2.420 and 1.814 ms, respectively. This remains a model-only replay.
The candidate's 2.838 ms saved-call maximum exceeded its 1.920 ms thread-CPU
maximum, showing that OS descheduling is still possible even after C++ fusion.

Separately, setting `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, and
`MKL_NUM_THREADS=1` in a fresh process improved the existing model without
changing its outputs. Three integrated, no-drive 501-cycle runs using that
environment each had 0/500 steady 20 ms deadline misses (maximum 19.392,
19.303, and 19.910 ms), with all 501 final STOP replies. The third run
selected the same settings through `--single-thread-math`. These runs did **not** use the
combined candidate.

These are paired, interleaved **model calls**, not a 20 ms acquisition →
inference → STOP reply result. The combined candidate has not been selected by
the live diagnostic or active-output loader. A diagnostic-only 501-cycle trial
would require a new manifest that SHA-pins the three C++ libraries, generated
model and core, exact-parity report, source hashes, baseline/scalar manifests,
records, target PyTorch ABI, and false approval/output flags. A separate loader
must verify all of those before registering the three operators, check the
graph and reset-state aliases, then be an explicit mutually exclusive option
in the STOP-proxy benchmark. The live output loader should remain unchanged.
That benchmark must use a new output directory and report both 500 steady
cycle deadlines and final STOP replies. Measured output deadlines, UID checks,
angle/IMU gates, command-loss STOP, and physical support remain unchanged.
