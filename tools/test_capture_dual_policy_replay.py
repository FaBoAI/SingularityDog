"""Synthetic serial/IMU only: saved raw contract, ownership and failure cleanup."""
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import capture_dual_policy_replay as tool


EXPECTED = {i: f"{i:016x}" for i in range(1, 13)}
BINDINGS = {b: {"path": "/dev/serial/by-path/" + b, "resolved": "/dev/" + b,
                "st_rdev": n} for b, n in (("front", 1), ("rear", 2))}


class Hardware:
    def __init__(self):
        self.ports, self.writes, self.order, self.samples = [], [], [], []
        self.uid_mismatch = self.timeout = self.negative_reply = self.corrupt = None
        self.initial_current = self.initial_mode = 0
        self.start_failure = self.read_failure = self.restore_failure = self.audit_change = False
        self.no_imu = False
        self.cancel = None
        self.binding_failure_after = None
        self.imu_device = None
        self.on_close = None

    @contextlib.contextmanager
    def lease(self, name):
        self.order.append("lock:" + name)
        try:
            yield
        finally:
            self.order.append("unlock:" + name)

    def can(self, **kwargs):
        port = Serial(self, kwargs["port"])
        self.ports.append(port)
        return tool.codec.ReadOnlyCAN(**kwargs, serial_port=port)

    def imu(self):
        self.imu_device = IMU(self)
        return self.imu_device

    def binding_check(self, can, binding):
        if self.binding_failure_after is not None and len(self.writes) >= self.binding_failure_after:
            raise ValueError("Opened USB2CAN binding changed")
        assert can.serial.port.endswith(binding["path"].split("/")[-1])

    def kwargs(self):
        return dict(can_factory=self.can, imu_factory=self.imu,
                    common_lock=lambda: self.lease("common"),
                    port_lock=lambda path: self.lease(path),
                    imu_lock=lambda: self.lease("imu"), binding_check=self.binding_check)


class Serial:
    def __init__(self, hardware, port):
        self.h, self.port, self.pending = hardware, port, b""
        self.is_open = True
        self.open_thread = threading.get_ident()

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        frame, = tool.codec.ATParser().feed(data)
        mid = frame.destination
        name = None if frame.kind == 0 else next(
            n for n, spec in tool.codec.PARAMETERS.items() if spec[0] == int.from_bytes(frame.data[:2], "little"))
        self.h.writes.append((self.port, mid, name, frame.kind, bytes(data), threading.get_ident()))
        if self.h.timeout == (mid, name):
            return len(data)
        cid = (frame.kind << 24) | (mid << 8) | (0xFE if name is None else 0xFD)
        if name is None:
            payload = bytes.fromhex("f" * 16 if self.h.uid_mismatch == mid else EXPECTED[mid])
        else:
            values = {"run_mode": self.h.initial_mode, "current": self.h.initial_current,
                      "voltage": 40., "position": mid * .1, "velocity": mid * .001}
            payload = struct.pack("<H", tool.codec.PARAMETERS[name][0]) + bytes(2)
            payload += struct.pack("<" + tool.codec.PARAMETERS[name][1], values[name])
            payload = payload.ljust(8, b"\0")
        if self.h.negative_reply == (mid, name):
            cid |= 1 << 16
        self.pending = b"AT" + ((cid << 3) | 4).to_bytes(4, "big") + b"\x08" + payload + b"\r\n"
        if self.h.corrupt == (mid, name):
            self.pending = b"!" + self.pending
        return len(data)

    def read(self, count):
        time.sleep(.0004)
        value, self.pending = self.pending[:count], self.pending[count:]
        return value

    def close(self):
        self.is_open = False
        self.h.order.append("close:" + self.port)


class IMU:
    def __init__(self, hardware):
        self.h = hardware
        self.original_registers = {"REG_BANK_SEL": 0, "bank0:0x06": 0x41}
        self.restore_status = "not_needed"
        self.closed = False
        self.audit_reads = 0

    def start(self):
        self.restore_status = "pending"
        if self.h.start_failure:
            raise OSError("Synthetic setup failure")
        return {"frame": "sensor", "accel_m_s2_per_lsb": 9.80665 / 16384,
                "gyro_rad_s_per_lsb": math.pi / 180 / 131, "registers": {"bank0:0x06": 1}}

    def diagnostic_registers(self):
        self.audit_reads += 1
        return {"raw_registers": {"trim": 9 if self.h.audit_change and self.audit_reads > 1 else 8}}

    def read_sample(self):
        if self.h.read_failure:
            raise OSError("Synthetic I2C read failure")
        if self.h.no_imu:
            return None
        now = time.monotonic_ns()
        sample = {"sequence": len(self.h.samples) + 1, "frame": "sensor", "monotonic_ns": now,
                  "read_started_monotonic_ns": now - 10, "read_finished_monotonic_ns": now + 10,
                  "wall_time_ns": time.time_ns(), "data_ready_during_read": False,
                  "raw_accel": [0, 0, -16384], "raw_gyro": [0, 0, 0], "raw_temperature": 0,
                  "temperature_c": 21., "accel_m_s2": [0., 0., -9.80665], "gyro_rad_s": [0., 0., 0.]}
        self.h.samples.append(sample)
        if self.h.cancel is not None and len(self.h.samples) == 3:
            self.h.cancel.set()
        return sample

    def close(self):
        if self.h.on_close is not None:
            self.h.on_close()
        self.closed = True
        self.h.order.append("close:imu")
        self.restore_status = "failed" if self.h.restore_failure else "restored"
        if self.h.restore_failure:
            raise OSError("Synthetic restore failure")


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()

    def tearDown(self):
        self.temp.cleanup()

    def run_capture(self, hardware=None, stop=None):
        h = hardware or Hardware()
        path = self.root / ("capture-" + str(len(list(self.root.iterdir()))))
        path.mkdir()
        recorder = tool.Recorder(path)
        try:
            timeout = .01 if h.timeout is not None else .25
            report = tool.capture(tool.make_plan(1., timeout), BINDINGS, EXPECTED, recorder,
                                  stop=stop, **h.kwargs())
        finally:
            recorder.close()
        (path / "summary.json").write_text(json.dumps(report))
        return h, path, report, recorder.records

    def assert_cleanup(self, h, report):
        self.assertTrue(h.imu_device.closed)
        self.assertTrue(all(not p.is_open for p in h.ports))
        self.assertTrue(report["acquisition"]["can_worker_joined"] if h.ports else True)
        for i, item in enumerate(h.order):
            if item.startswith("unlock:"):
                self.assertLess(h.order.index("close:imu"), i)
                for port in h.ports:
                    self.assertLess(h.order.index("close:" + port.port), i)

    def test_default_plan_opens_no_devices_locks_files_or_reads_required_inputs(self):
        with patch.object(tool.dual, "validate_ports", side_effect=AssertionError), \
             patch.object(tool, "private_directory", side_effect=AssertionError), \
             patch.object(tool, "capture", side_effect=AssertionError), \
             contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main([]), 0)
        plan = json.loads(out.getvalue())
        self.assertEqual(plan["status"], "PLAN_ONLY")
        self.assertEqual(plan["allowed_can_types"], [0, 17])
        for name, value in tool.FLAGS.items():
            self.assertIs(plan[name], value)

    def test_bounded_plan_rejects_boolean_nonfinite_and_outside_duration_timeout(self):
        for seconds in (True, math.nan, math.inf, 0, 10.1):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError): tool.make_plan(seconds)
        for timeout in (True, math.nan, 0, .26):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError): tool.make_plan(timeout_s=timeout)

    def test_real_readonly_codec_events_load_in_shadow_and_build_all12_samples(self):
        h, path, report, records = self.run_capture()
        self.assertIn(report["status"], ("COMPLETE", "COMPLETE_WITH_WARNINGS"))
        self.assertEqual(report["errors"], [])
        self.assertTrue(report["required_coverage_complete"])
        loaded, audit = tool.shadow.load_capture(path)
        self.assertEqual(loaded, records)
        self.assertEqual(audit["raw_redecoded_parameter_count"], len(h.writes))
        candidate = {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
                     "model_can_order_candidate": list(tool.shadow.CAN_ORDER),
                     "identities": {str(i): x for i, x in EXPECTED.items()},
                     "formula": "q_model = sign * raw + offset; rad; no wrapping",
                     "approved_for_runtime": False,
                     "candidates": [{"motor_id": i, "sign_candidate": 1,
                                     "offset_candidate_rad": 0.} for i in range(1, 13)]}
        samples = tool.shadow.build_samples(records, candidate, assume_sensor_aligned=True)
        self.assertTrue(samples)
        self.assertTrue(all(len(row["q_model_rad"]) == 12 for row in samples))
        self.assertTrue(all(not row["motor_output_available"] for row in samples))
        self.assert_cleanup(h, report)

    def test_uid_and_initial_quiet_precede_telemetry_on_one_worker_across_two_buses(self):
        h, _, report, records = self.run_capture()
        self.assertEqual([(mid, name) for _, mid, name, *_ in h.writes[:12]], [(i, None) for i in range(1, 13)])
        self.assertEqual([(mid, name) for _, mid, name, *_ in h.writes[12:48]],
                         [(i, name) for i in range(1, 13) for name in ("run_mode", "current", "voltage")])
        self.assertEqual(len({row[-1] for row in h.writes}), 1)
        self.assertNotEqual(h.writes[0][-1], threading.get_ident())
        self.assertEqual({row[3] for row in h.writes}, {0, 17})
        for port, mid, *_ in h.writes:
            self.assertTrue(port.endswith("front" if mid <= 6 else "rear"))
        samples = [r for r in records if r.get("kind") == "imu"]
        self.assertEqual([{k: r[k] for k in sample} for r, sample in zip(samples, h.samples)], h.samples)
        self.assertFalse(report["plan"]["pure_hardware_readonly"])
        self.assert_cleanup(h, report)

    def test_uid_mismatch_stops_before_parameter_reads_and_saves_partial_raw_evidence(self):
        h = Hardware(); h.uid_mismatch = 1
        _, path, report, _ = self.run_capture(h)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertTrue(any("UID mismatch" in row["error"] for row in report["errors"]))
        self.assertEqual(len(h.writes), 1)
        self.assertGreater((path / "events.jsonl").stat().st_size, 0)
        self.assert_cleanup(h, report)

    def test_nonzero_run_mode_or_current_stops_without_enable_stop_or_telemetry(self):
        for name in ("initial_mode", "initial_current"):
            h = Hardware(); setattr(h, name, 1)
            _, _, report, _ = self.run_capture(h)
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assertTrue(any("is not zero" in row["error"] for row in report["errors"]))
            self.assertTrue(all(row[3] in (0, 17) for row in h.writes))
            self.assertFalse(any(row[2] in ("position", "velocity") for row in h.writes))
            self.assert_cleanup(h, report)

    def test_timeout_is_not_retried_and_shadow_rejects_incomplete_saved_capture(self):
        h = Hardware(); h.timeout = (2, None)
        _, path, report, records = self.run_capture(h)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(len(h.writes), 2)
        self.assertEqual(sum(r.get("kind") == "can_timeout" for r in records), 1)
        with self.assertRaises(ValueError): tool.shadow.load_capture(path)
        self.assert_cleanup(h, report)

    def test_negative_parameter_reply_and_parser_discard_both_stop(self):
        for failure, target in (("negative_reply", (1, "voltage")), ("corrupt", (1, None))):
            h = Hardware(); setattr(h, failure, target)
            _, _, report, _ = self.run_capture(h)
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assert_cleanup(h, report)

    def test_port_binding_change_stops_and_closes_both_ports_before_leases(self):
        h = Hardware(); h.binding_failure_after = 3
        _, _, report, _ = self.run_capture(h)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(len(h.writes), 3)
        self.assert_cleanup(h, report)

    def test_setup_and_i2c_read_errors_always_restore_and_keep_incomplete_summary(self):
        for failure in ("start_failure", "read_failure"):
            h = Hardware(); setattr(h, failure, True)
            _, _, report, _ = self.run_capture(h)
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assertEqual(report["imu_restore_status"], "restored")
            self.assert_cleanup(h, report)

    def test_restore_failure_and_trim_change_prevent_complete_promotion(self):
        for failure in ("restore_failure", "audit_change"):
            h = Hardware(); setattr(h, failure, True)
            _, _, report, _ = self.run_capture(h)
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assertTrue(any(row["component"].startswith("imu_") for row in report["errors"]))
            self.assert_cleanup(h, report)

    def test_no_imu_samples_or_cooperative_cancel_stops_and_joins(self):
        for failure in ("no_imu", "cancel"):
            h = Hardware(); stop = threading.Event()
            if failure == "no_imu": h.no_imu = True
            else: h.cancel = stop
            _, _, report, _ = self.run_capture(h, stop)
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assert_cleanup(h, report)

    def test_request_guard_rejects_write_wrong_bus_and_bad_wire(self):
        for event in ({"kind": "can_tx", "motor_id": 1, "parameter": "enable", "hex": ""},
                      {"kind": "can_tx", "motor_id": 7, "parameter": "identity", "hex": ""},
                      {"kind": "can_tx", "motor_id": 1, "parameter": "identity", "hex": "00"},
                      {"kind": "can_rx_frame", "source_id": 7},
                      {"kind": "motor_feedback", "type": 2, "mode_state": 2}):
            with self.subTest(event=event), self.assertRaises(ValueError): tool.guard_can_event("front", event)

    def test_private_destination_existing_git_and_symlink_are_rejected(self):
        with self.assertRaises(ValueError): tool.private_directory(self.root)
        checkout = self.root / "checkout"; checkout.mkdir(); (checkout / ".git").mkdir()
        with self.assertRaises(ValueError): tool.private_directory(checkout / "new")
        symlink = self.root / "link"; symlink.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError): tool.private_directory(symlink / "new")

    def test_explicit_execution_requires_all_inputs_before_any_hardware(self):
        with patch.object(tool.dual, "validate_ports", side_effect=AssertionError), \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            tool.main(["--execute-readonly"])

    def test_event_budget_failure_saves_partial_trace_and_prevents_complete(self):
        with patch.object(tool, "MAX_EVENTS", 3):
            h, path, report, records = self.run_capture()
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertLessEqual(len(records), 3)
        self.assertGreater((path / "events.jsonl").stat().st_size, 0)
        self.assert_cleanup(h, report)

    def test_required_coverage_does_not_claim_optional_parameters_or_missing_axes(self):
        summary = {"imu": {"samples": 2}, "motors": {
            str(i): {"identities": [EXPECTED[i]], "parameters": {
                name: {"last": 0 if name in tool.READS else None} for name in tool.codec.PARAMETERS}}
            for i in range(1, 13)}}
        state = {"initial_quiet_verified": True, "complete_sweeps": 1}
        self.assertEqual(tool.coverage_errors(summary, EXPECTED, state), [])
        summary["motors"]["12"]["parameters"]["velocity"]["last"] = None
        self.assertTrue(tool.coverage_errors(summary, EXPECTED, state))
        summary["motors"]["12"]["parameters"]["velocity"]["last"] = 0
        state["complete_sweeps"] = 0
        self.assertTrue(tool.coverage_errors(summary, EXPECTED, state))

    def test_source_or_uid_change_after_capture_blocks_summary_success(self):
        original = tool.capture
        for kind in ("source", "uid"):
            h = Hardware()
            expected = self.root / (kind + "-uids.json"); expected.write_text(json.dumps(EXPECTED))
            destination = self.root / (kind + "-capture")
            def collect(plan, bindings, values, recorder, **kwargs):
                result = original(plan, bindings, values, recorder, **kwargs, **h.kwargs())
                if kind == "uid": expected.write_text(json.dumps({str(i): "f" * 16 for i in EXPECTED}))
                return result
            pins = [{"synthetic_source": "a" * 64}, {"synthetic_source": "b" * 64}] if kind == "source" else [{}, {}]
            with patch.object(tool.dual, "validate_ports", return_value=BINDINGS), \
                 patch.object(tool, "_boot_id", return_value="synthetic-boot"), \
                 patch.object(tool, "source_pins", side_effect=pins), \
                 patch.object(tool, "capture", side_effect=collect), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(["--execute-readonly", "--front-port", "front", "--rear-port", "rear",
                    "--expected-uids", str(expected), "--output", str(destination), "--seconds", "1"]), 1)
            result = json.loads((destination / "summary.json").read_text())
            self.assertEqual(result["status"], "INCOMPLETE")
            self.assertTrue(any(row["component"] == "provenance" for row in result["errors"]))

    def test_repeated_interrupt_during_restore_is_recorded_and_original_handlers_restored(self):
        import signal
        h = Hardware()
        previous = signal.getsignal(signal.SIGINT)
        h.on_close = lambda: [signal.getsignal(signal.SIGINT)(signal.SIGINT, None) for _ in range(2)]
        expected = self.root / "signal-uids.json"; expected.write_text(json.dumps(EXPECTED))
        destination = self.root / "signal-capture"
        original = tool.capture
        def collect(plan, bindings, values, recorder, **kwargs):
            return original(plan, bindings, values, recorder, **kwargs, **h.kwargs())
        with patch.object(tool.dual, "validate_ports", return_value=BINDINGS), \
             patch.object(tool, "_boot_id", return_value="synthetic-boot"), \
             patch.object(tool, "capture", side_effect=collect), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tool.main(["--execute-readonly", "--front-port", "front", "--rear-port", "rear",
                "--expected-uids", str(expected), "--output", str(destination), "--seconds", "1"]), 1)
        report = json.loads((destination / "summary.json").read_text())
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(report["imu_restore_status"], "restored")
        self.assertTrue(any(row["component"] == "process" for row in report["errors"]))
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.assert_cleanup(h, report)

    def test_main_publishes_private_summary_with_source_uid_trace_hashes(self):
        h = Hardware()
        expected = self.root / "uids.json"; expected.write_text(json.dumps(EXPECTED))
        destination = self.root / "main-capture"
        original = tool.capture
        def collect(plan, bindings, values, recorder, **kwargs):
            return original(plan, bindings, values, recorder, **kwargs, **h.kwargs())
        with patch.object(tool.dual, "validate_ports", return_value=BINDINGS), \
             patch.object(tool, "_boot_id", return_value="synthetic-boot"), \
             patch.object(tool, "capture", side_effect=collect), \
             contextlib.redirect_stdout(io.StringIO()):
            code = tool.main(["--execute-readonly", "--front-port", BINDINGS['front']['path'],
                              "--rear-port", BINDINGS['rear']['path'], "--expected-uids", str(expected),
                              "--output", str(destination), "--seconds", "1", "--timeout", ".25"])
        self.assertEqual(code, 0)
        summary = json.loads((destination / "summary.json").read_text())
        self.assertEqual(summary["expected_uids_sha256"], hashlib.sha256(expected.read_bytes()).hexdigest())
        self.assertEqual(summary["events_sha256"], hashlib.sha256((destination / "events.jsonl").read_bytes()).hexdigest())
        self.assertEqual((destination / "summary.json").stat().st_mode & 0o777, 0o600)
        self.assertEqual((destination / "events.jsonl").stat().st_mode & 0o777, 0o600)
        self.assertEqual(destination.stat().st_mode & 0o777, 0o700)
        tool.shadow.load_capture(destination)


if __name__ == "__main__":
    unittest.main()
