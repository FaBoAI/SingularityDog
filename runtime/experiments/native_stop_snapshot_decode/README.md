# Fixed STOP snapshot decode experiment

This file-only experiment reduces Python packet conversion work for canonical
STOP-proxy acquisition records. It is separate from the production transport,
collector, model and active controller. No runtime flag selects it, and it does
not open CAN/IMU, load a native library/model, change timestamps or approve output.

The original assembler creates one `Frame` for the request and one for the reply
of each of the twelve feedback records. The candidate matches canonical STOP
request bytes and the exact disabled, fault-free Type2 header directly, then uses
the same unsigned-16 decoding expressions. This removes those 24 `Frame`
allocations while retaining the full snapshot construction. Other allocations
remain. Type17 input is explicitly unsupported by the candidate.

`candidate.py` accepts only its four pinned Python source files. It extracts pure
codec/record/snapshot definitions with AST selection; it does not import the
production runtime. The candidate replaces only the packet conversion branch.
In the supported STOP domain, all remaining snapshot logic is the original code:
causal/full receipt checks, bus membership, duplicate/missing values, raw SI
conversion, IMU finite/causal checks, timing summaries and source flags. Malformed
STOP records retain the original error reason and check ordering. Outputs own
new list/dict descendants, leaving the original record bytes and IMU input intact.

The native exchange, completed-buffer seals, observer validation/model ranges,
post-inference freshness and final pre-send/native deadlines are still necessary.
This experiment does not replace them. The assembler's internal snapshot limit
is the original 100ms diagnostic limit; the real pipeline's stricter 20ms gates
are outside this experiment and remain unchanged. A passing experiment cannot
qualify a live controller or certify any physical measurement.

Default PLAN reads and verifies source hashes without compiling the isolated
engine or loading a model. Its default root is the included
`fixtures/frozen-source/`, containing the four exact original source files under
`runtime/singularitydog_hw/`. Later runtime changes do not replace this fixture
or its frozen hashes. These are source-only historical references; they are
never imported as a runtime package.

`--source-root` explicitly selects a separate repository or frozen kit root.
All four source hashes must still match; a newer repository source is rejected.
The hash checks and all timing/error conditions remain unchanged. No private
host paths or hardware identifiers are embedded in this experiment.

```sh
python3 -B runtime/experiments/native_stop_snapshot_decode/candidate.py
python3 -B runtime/experiments/native_stop_snapshot_decode/candidate.py \
  --source-root /absolute/path/to/frozen-kit
```

An optional bounded synthetic microbenchmark reports every wall/thread-CPU
sample in alternating baseline/candidate order. It measures only snapshot
assembly. Clock/list overhead is included, GC state/statistics are recorded but
unchanged, and actual native I/O, concurrent GIL scheduling, model inference and
final freshness gates are excluded. It provides no target latency improvement
claim. Saved actual records can later be passed to the isolated engine for an
independent CPU/file-only parity experiment; a saved-record CLI is not supplied.

```sh
python3 -B runtime/experiments/native_stop_snapshot_decode/candidate.py \
  --profile-synthetic --iterations 1000
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime:runtime/tests \
  python3 -B -m unittest test_native_stop_snapshot_decode -v
```

The tests include 100 deterministic synthetic payload sets, exact whole-snapshot
and timestamp equality, baseline/candidate Frame counts, malformed/fault/mode/
wrong-bus/sentinel replies, incomplete receipts, duplicate/missing axes,
noncausal/stale times, nonfinite IMU, output mutation isolation, pinned-source
changes/symlinks/size rejection, explicit Type17 refusal and PLAN separation.
These are source/CPU tests, not device or ABI measurements.
