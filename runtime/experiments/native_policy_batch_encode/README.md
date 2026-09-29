# File-only twelve-axis Type1 encoder candidate

`batch_encode.cpp` and `batch_encode_py.cpp` contain an isolated CPython C++
candidate. They neither open a device nor send a command. The active runner's
default remains its Python encoder. The opt-in loader pins both source hashes
and requires an independently reviewed binary SHA-256. Its startup canaries
compare exact Type1 bytes and rejection behavior before it returns an encoder.

Build on Jetson into a fresh temporary path using the cached Python 3.12
headers. The helper prints the binary SHA-256 and an explicit **unapproved**
status; it does not create or edit an output profile:

Run from the repository or source-kit root:

```sh
python3 runtime/experiments/native_policy_batch_encode/build_file_only.py \
  --compiler g++ \
  --include /home/jetson/singularitydog-tests/python-dev-headers-20260925/extracted/usr/include/python3.12 \
  --include /home/jetson/singularitydog-tests/python-dev-headers-20260925/extracted/usr/include \
  --output /tmp/sdbe_native.cpython-312-aarch64-linux-gnu.so
```

From the repository root, run the file-only loader and runtime wiring tests:

```sh
PYTHONPATH=runtime python3 -m unittest \
  runtime.tests.test_native_policy_batch_encode_loader \
  runtime.tests.test_policy_output_native_batch_integration
```

Live selection requires a separately approved V3 profile with the reviewed
`native_batch_encoder` path and SHA-256. The runner loads the extension before
enable and binds it to the fresh post-enable sample. Any mismatch or encoding
error fails closed; it does not fall back during an active cycle. The existing
motion envelope, voltage/freshness gates, motor reply checks and STOP handling
remain in the runner. This is not a motor-output authorization.
