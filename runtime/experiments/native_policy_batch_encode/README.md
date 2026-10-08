# File-only twelve-axis Type1 encoder candidate

`batch_encode.cpp` and `batch_encode_py.cpp` contain an isolated CPython C++
candidate that predates the completion-notification optimization. The current
work adds a balanced verified-wrapper benchmark; it does not introduce or
replace the encoder implementation. These sources neither open a device nor
send a command. The active runner's
default remains its Python encoder. The opt-in loader pins both source hashes
and requires an independently reviewed binary SHA-256. Its startup canaries
compare exact Type1 bytes and rejection behavior before it returns an encoder.

Build for the target machine's Python into a fresh temporary directory. The
helper prints the binary SHA-256 and an explicit **unapproved**
status; it does not create or edit an output profile:

Run from the repository or source-kit root:

```sh
TASK_ENCODER_DIR=$(mktemp -d "${TMPDIR:-/tmp}/dog-batch-encoder.XXXXXX")
TASK_ENCODER_SUFFIX=$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')
TASK_ENCODER_BINARY="$TASK_ENCODER_DIR/sdbe_native$TASK_ENCODER_SUFFIX"
python3 runtime/experiments/native_policy_batch_encode/build_file_only.py \
  --compiler c++ --output "$TASK_ENCODER_BINARY"
```

The default include directory comes from the running Python's `sysconfig`.
If Jetson's development headers are stored separately, provide their matching
Python 3.12 include directory with `--include "$TASK_PYTHON_INCLUDE_DIR"`;
repeat `--include` for an additional architecture header directory when needed.
The builder verifies both pinned sources and requires a new extension filename
matching that interpreter's suffix. It uses `-ffp-contract=off` and does not
install a package or change the existing profile.

From the repository root, run the file-only loader and runtime wiring tests:

```sh
PYTHONPATH=runtime python3 -m unittest \
  runtime.tests.test_native_policy_batch_encode_loader \
  runtime.tests.test_policy_output_native_batch_integration
```

Live selection requires a separately approved V3 profile with the reviewed
`native_batch_encoder` path and SHA-256. The runner loads the extension before
enable and binds its displacement limits to the last pre-enable sample. The
fresh post-enable sample supplies the smooth starting target without resetting
the trial's displacement allowance. Any mismatch or encoding
error fails closed; it does not fall back during an active cycle. The existing
motion envelope, voltage/freshness gates, motor reply checks and STOP handling
remain in the runner. This is not a motor-output authorization.

## Balanced verified-wrapper component benchmark

`benchmark_verified_wrapper.py` loads through the existing verified loader,
binds immutable axis specs, and measures the actual wrapper returned to the
runner. Its Python reference performs the same twelve encodes, frame parsing,
quantized physical-range/displacement checks, and estimated-PD checks.
Both methods return twelve independent 17-byte `bytes` objects in front/rear
bus order. The loader runs 96 seeded byte canaries and 72 rejection canaries;
the benchmark adds 2,000 seeded parity cases by default and compares each
measured result outside the timed intervals.

After the file-only build above, run:

```sh
TASK_ENCODER_SHA256=$(python3 -c 'import hashlib, sys; print(hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest())' "$TASK_ENCODER_BINARY")
PYTHONPATH=runtime python3 \
  runtime/experiments/native_policy_batch_encode/benchmark_verified_wrapper.py \
  --library "$TASK_ENCODER_BINARY" --binary-sha256 "$TASK_ENCODER_SHA256" \
  --output "$TASK_ENCODER_DIR/verified-wrapper-benchmark.json" \
  --samples 2000 --trials 4 --warmup 200 --parity-cases 2000
```

Measurement order alternates every sample and reverses the first method each
trial. Cyclic GC is deferred only during measurement and restored on success
or failure. The report retains per-sample timings, source/binary hashes and the
reference wire digest. An existing output file is never overwritten.

On Mac ARM64 with Python 3.13, four rounds of 2,000 samples measured medians of
48.417–49.041µs for Python and 1.625–1.667µs for the verified wrapper, saving
46.792–47.374µs. These are component measurements using synthetic commands.
They do not measure USB/CAN, inference, the Jetson scheduler or a whole 20ms
cycle, and must not be added to a measured hardware result as proven savings.

The Mac extension cannot be used on Jetson. Rebuild the same pinned sources for
Jetson's Python 3.12/aarch64, repeat byte/rejection parity and this component
benchmark, then measure the explicitly selected real Type1 route. This work
does not change the success-setting baseline, request interval, window3,
20ms deadline, profile defaults or output approval.
