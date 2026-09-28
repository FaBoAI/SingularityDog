"""File-only exact comparison of a refinement-loop candidate; no robot I/O.

The live records contain model inputs, not projection arguments. The pinned
eager model reconstructs those arguments from each saved input. Native/eager
rounding may differ from the Jetson's saved target; the comparison here is
strictly between the two C++ projection implementations on identical inputs.
"""

import ctypes
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile
import time
import unittest

HERE = Path(__file__).resolve().parent
BASELINE = HERE.parent / "projection.cpp"
CANDIDATE = HERE / "projection.cpp"


class Result(ctypes.Structure):
    _fields_ = [
        ("solved", ctypes.c_double * 12),
        ("endpoint_q", ctypes.c_double * 12),
        ("fraction", ctypes.c_double * 4),
        ("first_invalid", ctypes.c_double * 4),
        ("hi", ctypes.c_double * 4),
        ("applied", ctypes.c_double * 4),
        ("roundtrip", ctypes.c_double * 12),
        ("joint_margin", ctypes.c_double * 12),
        ("reason", ctypes.c_int32 * 4),
        ("endpoint_domain", ctypes.c_int32 * 4),
        ("endpoint_ok", ctypes.c_int32 * 4),
        ("path_rejected", ctypes.c_int32 * 4),
    ]


def build(source, destination):
    subprocess.run(["c++", "-std=c++20", "-O3", "-ffp-contract=off", "-shared",
                    "-fPIC", str(source), "-o", str(destination)], check=True,
                   capture_output=True, text=True, timeout=90)
    library = ctypes.CDLL(str(destination))
    call = library.projection_file_only
    call.argtypes = [ctypes.POINTER(ctypes.c_double)] * 6 + [ctypes.POINTER(Result)]
    call.restype = ctypes.c_int
    return call


def invoke(call, values):
    arrays = [(ctypes.c_double * len(row))(*row) for row in values]
    result = Result()
    status = call(*arrays, ctypes.byref(result))
    return status, bytes(result), result, arrays


def fk(leg, q):
    a, b, c = q
    sign = (1., -1., 1., -1.)[leg]
    ox = (.155, .155, -.155, -.155)[leg]
    oy = (.110, -.110, .110, -.110)[leg]
    x = -.12 * (math.sin(b) + math.sin(b + c))
    z = -.12 * (math.cos(b) + math.cos(b + c))
    y = .064 * sign
    return (x + ox, math.cos(a) * y - math.sin(a) * z + oy,
            math.sin(a) * y + math.cos(a) * z)


def make_synthetic(leg, q, requested):
    pre = list(q) * 4
    start = [v for index in range(4) for v in fk(index, q)]
    request = [0.] * 4
    request[leg] = requested
    return pre, start, [0., 0., 1.], request, [1.] * 4, request[:]


def mixed_case(base):
    # Find one valid input where exactly one leg rejects its scanned path.
    # This exercises the skip on the other three accepted legs.
    for thigh in (-.85, -.65, -.4, 0., .3, .7, 1.05):
        for calf in (-2.15, -1.8, -1.3, -.8, -.3, -.12):
            for leg in range(4):
                values = make_synthetic(leg, (.0, thigh, calf), .06)
                status, _, result, _ = invoke(base, values)
                if status == 0 and sum(result.path_rejected) == 1:
                    return values
    raise AssertionError("No mixed accepted/rejected projection case found")


def saved_inputs(records_path, bundle):
    import torch
    from runtime.experiments.native_policy_overnight.contracts import INPUT_KEYS, reference_policy

    rows = json.loads(Path(records_path).read_text())
    assert len(rows) == 500
    torch.set_num_threads(1)
    model, _ = reference_policy(bundle)
    model.reset(torch.tensor([0], dtype=torch.long))
    with torch.inference_mode():
        for index, row in enumerate(rows, 1):
            assert row["cycle"] == index
            inputs = [torch.tensor([row["observed"]["inputs"][key]], dtype=torch.float32)
                      for key in INPUT_KEYS]
            observation = model.controller.observation(*inputs)
            raw = model.actor(observation)
            data = model.controller.step(raw, inputs[2])
            pre = data["l07_preprojection_target_rad"].reshape(-1).tolist()
            start = model.controller.fk(data["l07_preprojection_target_rad"]).reshape(-1).tolist()
            up = model.controller.sensor_up.reshape(-1).tolist()
            requested = data["l13_projection_requested_m"].reshape(-1).tolist()
            alpha = data["l13_residual_scale_applied"].reshape(-1).tolist()
            uncapped = data["l13_projection_request_before_cap_m"].reshape(-1).tolist()
            yield pre, start, up, requested, alpha, uncapped


def microbenchmark(call, values, repeats=20000):
    arrays = [(ctypes.c_double * len(row))(*row) for row in values]
    result = Result()
    for _ in range(1000):
        assert call(*arrays, ctypes.byref(result)) == 0
    wall = time.perf_counter_ns()
    cpu = time.thread_time_ns()
    for _ in range(repeats):
        assert call(*arrays, ctypes.byref(result)) == 0
    return {"calls": repeats, "wall_us_per_call": (time.perf_counter_ns()-wall)/repeats/1000,
            "thread_cpu_us_per_call": (time.thread_time_ns()-cpu)/repeats/1000}


def run(record_paths, bundle):
    with tempfile.TemporaryDirectory(prefix="dog-projection-accept-skip-") as folder:
        folder = Path(folder)
        original = build(BASELINE, folder / "baseline.so")
        candidate = build(CANDIDATE, folder / "candidate.so")
        report = {"scope": "file_only_projection_no_hardware", "output_allowed": False,
                  "source_sha256": {"baseline": hashlib.sha256(BASELINE.read_bytes()).hexdigest(),
                                    "candidate": hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()},
                  "records": {}, "synthetic": {}, "benchmarks": {}}
        for path in record_paths:
            count = zeros = mixed = 0
            for values in saved_inputs(path, bundle):
                left = invoke(original, values)
                right = invoke(candidate, values)
                assert left[0] == right[0] and left[1] == right[1]
                assert left[0] == 0
                count += 1
                zeros += sum(v == 0. for v in values[3])
                mixed += bool(any(left[2].path_rejected) and not all(left[2].path_rejected))
            report["records"][Path(path).name] = {
                "cycles": count, "leg_requests_zero": zeros,
                "mixed_rejection_cycles": mixed, "all_projection_bytes_exact": True,
                "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}
        cases = {
            "all_zero": make_synthetic(0, (.0, .2, -1.3), 0.),
            "mixed_rejected": mixed_case(original),
        }
        for name, values in cases.items():
            left = invoke(original, values)
            right = invoke(candidate, values)
            assert left[0] == right[0] == 0 and left[1] == right[1]
            report["synthetic"][name] = {
                "path_rejected": list(left[2].path_rejected),
                "all_projection_bytes_exact": True,
                "requested": values[3]}
            report["benchmarks"][name] = {
                "baseline": microbenchmark(original, values),
                "candidate": microbenchmark(candidate, values)}
        return report


class CandidateSourceTest(unittest.TestCase):
    def test_only_accepted_refinement_skip_differs(self):
        old, new = BASELINE.read_text(), CANDIDATE.read_text()
        old = old.replace(
            "// File-only scalar projection experiment. No devices, networking or actuation.\n"
            "// Derived from hash-pinned SwingCore.step projection block; not a controller.\n",
            "// Isolated accepted-leg refinement experiment. No devices or actuation.\n"
            "// This is deliberately separate from the hash-pinned production projection.\n")
        marker = "    for(int k=0;k<14;++k) for(int leg=0;leg<4;++leg) {\n"
        added = ("      // A leg accepted at all 16 scan fractions is already at fraction=hi=1.\n"
                 "      // The baseline repeats evaluate(leg, 1) fourteen times only because a\n"
                 "      // different leg was rejected. Skipping that work preserves all outputs.\n"
                 "      if(active[leg]) continue;\n")
        self.assertEqual(new, old.replace(marker, marker + added, 1))


if __name__ == "__main__":
    unittest.main()
