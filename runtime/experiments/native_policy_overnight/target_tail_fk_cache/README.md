# Isolated immutable FK reuse candidate

This file-only experiment compares the current verified scalar policy, the frozen
`target_tail_fusion` candidate, and a second C++ target-tail candidate. It adds no
runtime selection, motor commands, timing admission, or physical approval.

Only FK's repeated `b + c`, `sin(a)`, and `cos(a)` values are reused. The original
ATen multiplication, subtraction, addition, stacking, dtype and rounding order
remain unchanged. An inverse byte check rejects any other change to the first
candidate's C++ source; the new core has a separate custom-op namespace. The
scalar, projection, actor, observation, recurrent state and guards are retained.
There is no performance claim before target measurement.

Default `saved_profile` execution is PLAN: it verifies the original 501-call
report/records, their independent raw audit, the helper and 24 source hashes. It
does not import Torch or compile/load model code. Explicit `--execute-file-only`
uses the same verified current scalar loader and original artifacts. It may reuse
the first target-tail's compiled library through an explicit path/SHA pair; the
FK library is compiled separately, or likewise supplied with a SHA.

Three independent models are compared against all original saved output,
actor and observation bits, and all 30 named state tensors are compared after
every call. The 26 rejection cases retain error text and partial state; each
candidate additionally receives the same 240 synthetic cases. Six timing blocks
cover every model permutation. Within each block the three-way call order
rotates per cycle; for 501 calls each model appears 167 times at every position.
Input bit, output bit and state checks stay outside the forward clocks. Warmup,
reset, raw wall/thread-CPU timings, per-cycle order and position counts are saved.
The separate operator profiler is not included in those timings.

`local_validate` compiles the genuine original projection/scalar and both target
operators on the local CPU with a deterministic test actor. It checks 501
recurrent calls, both 240-case suites, 26 rejections, 44 direct boundary cases,
all states and cached-view aliases. It does not claim equivalence with original
Jetson output arrays; that check belongs to the target saved-input runner.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime/experiments:runtime \
  python3 -B -m unittest \
  native_policy_overnight.target_tail_fk_cache.test_saved_profile -q
```

The old target-tail sources and their target results stay unchanged. No hardware
or live 20 ms loop is exercised by this experiment, and a CPU gain cannot by
itself qualify active control.
