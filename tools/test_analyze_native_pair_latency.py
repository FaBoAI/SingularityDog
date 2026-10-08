"""Synthetic, file-only native-pair timing analysis contracts."""

from contextlib import redirect_stderr, redirect_stdout
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import analyze_native_pair_latency as tool


REPORT_SHA = "a" * 64
RECORDS_SHA = "b" * 64
PRIVATE_MARKERS = (
    "private-uid-sentinel", "private-boot-sentinel",
    "/private/raw-log-sentinel", "private-error-sentinel",
)


def saved_pair_inputs():
    """One complete cycle, one failed join, and one unattempted request."""
    completed_base = 1_000_000_000
    failed_base = 2_000_000_000
    measurement = {
        "release_ns": completed_base + 200_000,
        "oldest_input_start_ns": completed_base,
        "prepare_end_ns": completed_base + 4_700_000,
        "infer_end_ns": completed_base + 8_900_000,
        "final_host_write_ns": completed_base + 14_000_000,
        "last_proxy_reply_ns": completed_base + 15_000_000,
        "cycle_end_ns": completed_base + 16_000_000,
        # A reported aggregate must not replace the exact monotonic interval.
        "inference_ms": 999.0,
        "oldest_input_to_final_host_write_ms": 999.0,
        "oldest_input_to_last_reply_ms": 999.0,
    }
    report = {
        "status": "ABORTED", "mode": "stop-proxy",
        "motor_enable_sent": False, "learned_targets_sent": False,
        "native_phase_pair": True,
        "cycles_requested": 3, "cycles_completed": 1,
        "measurements": [measurement],
        "boot_id": PRIVATE_MARKERS[1],
        "source_path": PRIVATE_MARKERS[2],
        "errors": ["TimeoutError: " + PRIVATE_MARKERS[3]],
        "inference_thread_cpu_trace": {
            "fields": ["cycle", "thread_cpu_begin_ns", "thread_cpu_end_ns",
                       "thread_cpu_ns", "inference_wall_ns",
                       "wall_minus_thread_cpu_ns"],
            "rows": [[1, 100_000_000, 104_100_000,
                      4_100_000, 4_200_000, 100_000]],
            "main_native_tid": 1234567,
        },
        "incomplete_cycle_traces": [{
            "cycle": 2, "complete_measurement": False,
            "output_allowed": False,
            "reported_errors": [PRIVATE_MARKERS[3]],
            "output_dispatch_trace": {
                "fields": ["infer_end_ns"],
                "row": [failed_base + 8_900_000],
            },
            "inference_thread_cpu_trace": {
                "fields": ["thread_cpu_begin_ns", "thread_cpu_end_ns",
                           "inference_wall_ns"],
                "row": [200_000_000, 201_400_000, 0],
            },
        }],
    }
    records = []
    for cycle, base in ((1, completed_base), (2, failed_base)):
        row = {
            "cycle": cycle,
            "acquired": {
                "front": {"records": [{"start_ns": base + 150_000}]},
                "rear": {"records": [{"start_ns": base + 100_000}]},
            },
            "imu": {"read_started_monotonic_ns": base},
            "output": {},
            "native_phase_pair_phase": {
                "generation": cycle,
                "owner_finished_ns": [base + 15_100_000,
                                      base + 15_200_000] if cycle == 1 else
                                     [base + 19_100_000, base + 19_200_000],
                "native_tid": 1234567,
            },
            "voltage_fast_pipeline": {
                "output_join_begin_ns": base + (15_800_000 if cycle == 1 else 20_400_000),
                "output_join_deadline_ns": base + 20_000_000,
                "output_join_settled_ns": base + (16_000_000 if cycle == 1 else 22_000_000),
            },
            "uids_by_id": {"1": PRIVATE_MARKERS[0]},
            "boot_id": PRIVATE_MARKERS[1],
            "private_path": PRIVATE_MARKERS[2],
        }
        for scope, later in (("front", False), ("rear", True)):
            # The second slot establishes a real maximum, independent of order.
            write = base + (14_000_000 if cycle == 1 else 18_000_000)
            reply = base + (15_000_000 if cycle == 1 else 19_000_000)
            row["output"][scope] = {"records": [
                {"start_ns": base + 10_000_000,
                 "finish_ns": write - (0 if later else 200_000),
                 "received_ns": reply - (0 if later else 200_000),
                 "written": 17, "received": 17},
                {"start_ns": base + 9_000_000,
                 "finish_ns": write - 500_000,
                 "received_ns": reply - 500_000,
                 "written": 17, "received": 17},
            ]}
        if cycle == 2:
            row["voltage_fast_pipeline"]["output_join_failure_proof"] = {
                "stage": "readiness_join",
                "reason": "TimeoutError: " + PRIVATE_MARKERS[3],
                "captured_ns": base + 22_500_000,
                "capture_is_deadline_decision_time": False,
                "native_reply_time_inferred": False,
            }
        records.append(row)
    return report, records


def analyze(report, records):
    return tool.analyze(report, records, report_sha256=REPORT_SHA,
                        records_sha256=RECORDS_SHA)


class NativePairLatencyAnalysisTests(unittest.TestCase):
    def test_retains_completed_and_deadline_failed_cycles_without_admission(self):
        report, records = saved_pair_inputs()
        result = analyze(report, records)
        self.assertEqual(result["schema"], "singularitydog.native-pair-latency-analysis.v1")
        self.assertIs(result["output_allowed"], False)
        self.assertIs(result["active_controller_qualification"], False)
        self.assertEqual(result["source_sha256"],
                         {"report": REPORT_SHA, "records": RECORDS_SHA})
        self.assertEqual(result["counts"], {
            "requested_cycles": 3, "completed_measurements": 1,
            "raw_rows": 2, "failed_raw_rows": 1,
        })
        self.assertEqual([(row["cycle"], row["status"]) for row in result["cycles"]],
                         [(1, "completed"), (2, "failed")])
        completed, failed = result["cycles"]
        self.assertIsNone(completed["error_stage"])
        self.assertEqual(failed["error_stage"], "readiness_join")
        for row in result["cycles"]:
            if "output_allowed" in row:
                self.assertIs(row["output_allowed"], False)

    def test_original_owner_to_python_join_offset_and_input_age_are_exact(self):
        report, records = saved_pair_inputs()
        completed, failed = analyze(report, records)["cycles"]
        self.assertEqual(completed["native_owner_finished_to_output_join_begin_ns_by_bus"],
                         {"front": 700_000, "rear": 600_000})
        self.assertEqual(completed["native_owner_finished_to_output_join_begin_ms_by_bus"],
                         {"front": .7, "rear": .6})
        self.assertEqual(failed["native_owner_finished_to_output_join_begin_ns_by_bus"],
                         {"front": 1_300_000, "rear": 1_200_000})
        self.assertEqual(failed["native_owner_finished_to_output_join_begin_ms_by_bus"],
                         {"front": 1.3, "rear": 1.2})
        for row, write, reply in ((completed, 14_000_000, 15_000_000),
                                  (failed, 18_000_000, 19_000_000)):
            self.assertEqual(row["oldest_input_to_final_host_write_ns"], write)
            self.assertEqual(row["oldest_input_to_final_host_write_ms"], write / 1e6)
            self.assertEqual(row["oldest_input_to_last_reply_ns"], reply)
            self.assertEqual(row["oldest_input_to_last_reply_ms"], reply / 1e6)

    def test_signed_owner_to_join_offset_is_preserved(self):
        report, records = saved_pair_inputs()
        records[1]["native_phase_pair_phase"]["owner_finished_ns"][1] = 2_021_000_000
        failed = analyze(report, records)["cycles"][1]
        self.assertEqual(failed["native_owner_finished_to_output_join_begin_ns_by_bus"]["rear"],
                         -600_000)
        self.assertEqual(failed["native_owner_finished_to_output_join_begin_ms_by_bus"]["rear"], -.6)

    def test_inference_wall_uses_exact_prepare_stamp_and_missing_start_stays_null(self):
        report, records = saved_pair_inputs()
        completed, failed = analyze(report, records)["cycles"]
        self.assertEqual(completed["inference_wall_ns"], 4_200_000)
        self.assertEqual(completed["inference_wall_ms"], 4.2)
        self.assertEqual(completed["inference_thread_cpu_ns"], 4_100_000)
        self.assertEqual(completed["inference_thread_cpu_ms"], 4.1)
        self.assertIsNone(failed["inference_wall_ns"])
        self.assertIsNone(failed["inference_wall_ms"])
        self.assertEqual(failed["inference_thread_cpu_ns"], 1_400_000)
        self.assertEqual(failed["inference_thread_cpu_ms"], 1.4)
        del report["measurements"][0]["prepare_end_ns"]
        self.assertIsNone(analyze(report, records)["cycles"][0]["inference_wall_ns"])

    def test_missing_cpu_end_stays_null(self):
        report, records = saved_pair_inputs()
        report["incomplete_cycle_traces"][0]["inference_thread_cpu_trace"]["row"][1] = None
        failed = analyze(report, records)["cycles"][1]
        self.assertIsNone(failed["inference_thread_cpu_ns"])
        self.assertIsNone(failed["inference_thread_cpu_ms"])

    def test_saved_cpu_duration_must_match_original_thread_cpu_stamps(self):
        report, records = saved_pair_inputs()
        report["inference_thread_cpu_trace"]["rows"][0][3] += 1
        with self.assertRaisesRegex(ValueError, "CPU duration"):
            analyze(report, records)
        report, records = saved_pair_inputs()
        partial = report["incomplete_cycle_traces"][0]["inference_thread_cpu_trace"]
        partial["fields"].append("thread_cpu_ns")
        partial["row"].append(1_400_001)
        with self.assertRaisesRegex(ValueError, "CPU duration"):
            analyze(report, records)

    def test_completed_and_failed_distributions_have_independent_populations(self):
        report, records = saved_pair_inputs()
        summary = analyze(report, records)["distributions"]
        self.assertEqual(set(summary), {"completed", "failed"})
        for status, write, reply, cpu in (("completed", 14_000_000, 15_000_000, 4_100_000),
                                        ("failed", 18_000_000, 19_000_000, 1_400_000)):
            for name, duration in (("oldest_input_to_final_host_write_ns", write),
                                   ("oldest_input_to_last_reply_ns", reply),
                                   ("inference_thread_cpu_ns", cpu)):
                with self.subTest(status=status, metric=name):
                    metric = summary[status][name]
                    self.assertEqual(metric["count"], 1)
                    self.assertEqual([metric[key] for key in ("min_ns", "median_ns", "max_ns")],
                                     [duration] * 3)
                    self.assertEqual([metric[key] for key in ("min_ms", "median_ms", "max_ms")],
                                     [duration / 1e6] * 3)
        self.assertEqual(summary["completed"]["inference_wall_ns"]["median_ns"], 4_200_000)
        self.assertEqual(summary["failed"]["inference_wall_ns"], {
            "count": 0, "min_ns": None, "median_ns": None, "max_ns": None,
            "min_ms": None, "median_ms": None, "max_ms": None,
        })
        self.assertEqual(summary["completed"]["both_native_owners_finished_to_output_join_begin_ns"]
                         ["median_ns"], 600_000)
        self.assertEqual(summary["failed"]["both_native_owners_finished_to_output_join_begin_ns"]
                         ["median_ns"], 1_200_000)
        self.assertEqual(summary["completed"]["output_join_begin_after_deadline_ns"]
                         ["median_ns"], -4_200_000)
        self.assertEqual(summary["failed"]["output_join_begin_after_deadline_ns"]
                         ["median_ns"], 400_000)

    def test_failed_oldest_input_can_come_from_acquired_request(self):
        report, records = saved_pair_inputs()
        records[1]["acquired"]["front"]["records"][0]["start_ns"] = 1_999_500_000
        failed = analyze(report, records)["cycles"][1]
        self.assertEqual(failed["oldest_input_to_final_host_write_ns"], 18_500_000)
        self.assertEqual(failed["oldest_input_to_last_reply_ns"], 19_500_000)

    def test_saved_error_stage_classification_does_not_echo_arbitrary_text(self):
        for key, expected in (("final_gate_error", "final_gate"),
                              ("proxy_submit_error", "proxy_submit")):
            with self.subTest(key=key):
                report, records = saved_pair_inputs()
                gate = records[1]["voltage_fast_pipeline"]
                gate.pop("output_join_failure_proof")
                gate[key] = "TimeoutError: " + PRIVATE_MARKERS[3]
                result = analyze(report, records)
                self.assertEqual(result["cycles"][1]["error_stage"], expected)
                self.assertNotIn(PRIVATE_MARKERS[3], json.dumps(result))
        report, records = saved_pair_inputs()
        records[1]["voltage_fast_pipeline"]["output_join_failure_proof"]["stage"] = "result_takeout"
        self.assertEqual(analyze(report, records)["cycles"][1]["error_stage"], "result_takeout")

    def test_unknown_stage_is_sanitized_without_guessing_from_error_prose(self):
        report, records = saved_pair_inputs()
        proof = records[1]["voltage_fast_pipeline"]["output_join_failure_proof"]
        proof["stage"] = PRIVATE_MARKERS[3]
        proof["reason"] = "TimeoutError: " + PRIVATE_MARKERS[2] + " result_takeout"
        result = analyze(report, records)
        self.assertEqual(result["cycles"][1]["error_stage"], "unknown")
        self.assertNotIn(PRIVATE_MARKERS[3], json.dumps(result))
        self.assertNotIn(PRIVATE_MARKERS[2], json.dumps(result))

    def test_output_join_proof_only_classifies_actual_join_stages(self):
        report, records = saved_pair_inputs()
        records[1]["voltage_fast_pipeline"]["output_join_failure_proof"]["stage"] = "final_gate"
        self.assertEqual(analyze(report, records)["cycles"][1]["error_stage"], "unknown")

    def test_source_qualification_claims_cannot_grant_analyzer_admission(self):
        report, records = saved_pair_inputs()
        report.update(output_allowed=True, active_controller_qualification=True,
                      approved_for_runtime=True)
        records[1].update(output_allowed=True, active_controller_qualification=True)
        result = analyze(report, records)
        for flag in ("output_allowed", "active_controller_qualification",
                     "timing_admission_evaluated", "strict_20ms_qualification_claimed"):
            self.assertIs(result[flag], False)

    def test_input_objects_unchanged_and_output_sanitized(self):
        report, records = saved_pair_inputs()
        before = copy.deepcopy((report, records))
        result = analyze(report, records)
        self.assertEqual((report, records), before)
        encoded = json.dumps(result, sort_keys=True)
        for marker in PRIVATE_MARKERS:
            self.assertNotIn(marker, encoded)
        for sensitive_key in ("boot_id", "uids_by_id", "native_tid", "main_native_tid", "source_path"):
            self.assertNotIn('"' + sensitive_key + '"', encoded)


class PinnedInputTests(unittest.TestCase):
    def load_bytes(self, payload, *, pin=None, max_bytes=32 * 1024 * 1024):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "saved.json"
            path.write_bytes(payload)
            result = tool.load_pinned(path, pin or hashlib.sha256(payload).hexdigest(),
                                      max_bytes=max_bytes)
            self.assertEqual(path.read_bytes(), payload)
            return result

    def test_valid_input_at_exact_size_boundary(self):
        raw = b'{"rows": [1, 2], "unicode": "\\u4e00"}'
        self.assertEqual(self.load_bytes(raw, max_bytes=len(raw)),
                         {"rows": [1, 2], "unicode": "一"})

    def test_sha_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            self.load_bytes(b'{"rows": []}', pin="0" * 64)

    def test_oversize_input_rejected_even_with_matching_sha(self):
        raw = b'{"rows": []}'
        with self.assertRaises(ValueError):
            self.load_bytes(raw, max_bytes=len(raw) - 1)

    def test_duplicate_keys_rejected_at_root_and_nested_levels(self):
        for raw in (b'{"rows": [], "rows": [1]}',
                    b'{"outer": {"stage": "a", "stage": "b"}}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                self.load_bytes(raw)

    def test_exponent_overflow_and_explicit_nonfinite_numbers_rejected(self):
        for number in (b"1e999", b"-1e999", b"NaN", b"Infinity", b"-Infinity"):
            with self.subTest(number=number), self.assertRaises(ValueError):
                self.load_bytes(b'{"value": ' + number + b'}')


class NativePairLatencyCliTests(unittest.TestCase):
    def saved_files(self, root):
        report, records = saved_pair_inputs()
        report_path = root / "report.json"
        records_path = root / "records.json"
        report_bytes = json.dumps(report).encode()
        records_bytes = json.dumps(records).encode()
        report_path.write_bytes(report_bytes)
        records_path.write_bytes(records_bytes)
        args = ["--report", str(report_path), "--report-sha256",
                hashlib.sha256(report_bytes).hexdigest(),
                "--records", str(records_path), "--records-sha256",
                hashlib.sha256(records_bytes).hexdigest()]
        return args, {report_path: report_bytes, records_path: records_bytes}

    def test_stdout_and_optional_output_are_sanitized_and_inputs_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            args, originals = self.saved_files(root)
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(tool.main(args), 0)
            stdout_result = json.loads(stream.getvalue())
            output = root / "analysis.json"
            with redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(args + ["--output", str(output)]), 0)
            self.assertEqual(json.loads(output.read_text()), stdout_result)
            for path, raw in originals.items():
                self.assertEqual(path.read_bytes(), raw)
            for marker in (*PRIVATE_MARKERS, str(root)):
                self.assertNotIn(marker, output.read_text())
            self.assertIs(stdout_result["output_allowed"], False)
            self.assertIs(stdout_result["active_controller_qualification"], False)

    def test_wrong_pin_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            args, originals = self.saved_files(root)
            args[args.index("--records-sha256") + 1] = "0" * 64
            output = root / "analysis.json"
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                try:
                    status = tool.main(args + ["--output", str(output)])
                except SystemExit as error:
                    status = error.code
            self.assertNotEqual(status, 0)
            self.assertFalse(output.exists())
            for path, raw in originals.items():
                self.assertEqual(path.read_bytes(), raw)

    def test_deeply_nested_saved_json_returns_generic_private_safe_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            args, originals = self.saved_files(root)
            records_path = Path(args[args.index("--records") + 1])
            nested = b"[" * 2_000 + b"0" + b"]" * 2_000
            records_path.write_bytes(nested)
            originals[records_path] = nested
            args[args.index("--records-sha256") + 1] = hashlib.sha256(nested).hexdigest()
            output = root / "analysis.json"
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(tool.main(args + ["--output", str(output)]), 2)
            blocked = json.loads(stdout.getvalue())
            self.assertEqual(blocked["status"], "BLOCKED_SAVED_FILE_ANALYSIS")
            self.assertIs(blocked["output_allowed"], False)
            self.assertIs(blocked["active_controller_qualification"], False)
            self.assertEqual(stderr.getvalue(), "")
            self.assertFalse(output.exists())
            for marker in (*PRIVATE_MARKERS, str(root), "Traceback"):
                self.assertNotIn(marker, stdout.getvalue() + stderr.getvalue())
            for path, raw in originals.items():
                self.assertEqual(path.read_bytes(), raw)


if __name__ == "__main__":
    unittest.main()
