"""No-hardware tests for the live, read-only box-pose alignment display."""

import contextlib
import hashlib
import io
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import box_pose_alignment as alignment
from singularitydog_hw.can_readonly import read_request


UIDS = {mid: f"{mid:016x}" for mid in range(1, 13)}  # Synthetic identities.


def synthetic_capture():
    return {
        "schema": alignment.CAPTURE_SCHEMA,
        "status": "RECORDED_REVIEW_REQUIRED", "errors": [], "boot_id": "synthetic-boot",
        "plan": {"ports": {"front": "/dev/serial/by-path/front-test",
                           "rear": "/dev/serial/by-path/rear-test"},
                 "ids_by_bus": {bus: list(ids) for bus, ids in alignment.IDS_BY_BUS.items()}},
        "identities": {str(mid): {"mcu_uid_hex": UIDS[mid]} for mid in UIDS},
        "pose": {"raw_rad_by_id": {str(mid): mid / 10 for mid in UIDS},
                 "sampling_issues": [], "sampling_stability_heuristic_passed": True},
    }


def synthetic_digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FakeCAN:
    def __init__(self, bus, *, sink=alignment.guard_event):
        self.bus, self.sink = bus, sink
        self.parser = type("Parser", (), {"buffer": b"", "discarded_bytes": 0})()
        self.calls = []

    def query(self, mid, parameter=None):
        self.calls.append((mid, parameter))
        assert mid in alignment.IDS_BY_BUS[self.bus]
        assert parameter in (None, "position")
        self.sink(self.bus, {"kind": "can_tx", "motor_id": mid,
                             "parameter": "identity" if parameter is None else parameter,
                             "hex": read_request(mid, parameter).hex()})
        requested = time.monotonic_ns()
        row = {"ok": True, "request_monotonic_ns": requested,
               "monotonic_ns": requested + 1}
        if parameter is None:
            row["mcu_uid_hex"] = UIDS[mid]
        else:
            row.update(value=mid / 10 + math.radians(1.5),
                       unit="rad_output_shaft", index=0x7019)
        return row


class SessionCAN(FakeCAN):
    sessions = []

    def __init__(self, *, port, event_sink):
        bus = "front" if "front" in port else "rear"
        super().__init__(bus, sink=lambda _bus, event: event_sink(event))
        self.serial = None
        self._fd = None
        self.sessions.append(self)

    def __enter__(self):
        self._fd = os.open(os.devnull, os.O_RDONLY)
        self.serial = type("Serial", (), {"fileno": lambda _self: self._fd})()
        return self

    def __exit__(self, *_args):
        os.close(self._fd)


class AlignmentTests(unittest.TestCase):
    def test_loads_complete_private_baseline_without_printing_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "box.json"
            capture = synthetic_capture()
            path.write_text(json.dumps(capture))
            baseline = alignment.load_baseline(path, expected_sha256=synthetic_digest(path))
            self.assertEqual(set(baseline["raw_rad_by_id"]), set(range(1, 13)))
            with patch.object(alignment, "EXPECTED_BASELINE_SHA256", synthetic_digest(path)), \
                 patch.object(alignment, "ReadOnlyCAN") as can_open, \
                 contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(alignment.main(["--baseline", str(path)]), 0)
            can_open.assert_not_called()
            display = output.getvalue()
            self.assertNotIn("raw_rad_by_id", display)
            self.assertNotIn("mcu_uid_hex", display)
            self.assertIn('"motor_output_available": false', display)

    def test_rejects_incomplete_unstable_or_nonfinite_baseline(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "box.json"
            for change in (lambda d: d.update(status="INCOMPLETE"),
                           lambda d: d["pose"].update(sampling_issues=[{"motor_id": 1}]),
                           lambda d: d["pose"]["raw_rad_by_id"].update({"12": float("nan")}),
                           lambda d: d["identities"].pop("12")):
                data = synthetic_capture()
                change(data)
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    alignment.load_baseline(path, expected_sha256=synthetic_digest(path))

    def test_rejects_wrong_baseline_digest_before_live_io(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "box.json"
            path.write_text(json.dumps(synthetic_capture()))
            with self.assertRaisesRegex(ValueError, "SHA-256 differs"):
                alignment.load_baseline(path)

    def test_summary_is_fresh_private_delta_only_and_mode_0600(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "alignment.json"
            self.assertEqual(alignment.private_summary_path(path), path.resolve())
            deltas = {mid: float(mid) / 10 for mid in range(1, 13)}
            alignment.write_private_summary(path, "OPERATOR_EXIT", 1, deltas, deltas)
            data = json.loads(path.read_text())
            self.assertEqual(set(data), {"status", "complete_sweeps", "last_delta_deg_by_id",
                                         "best_sweep_delta_deg_by_id", "last_max_abs_delta_deg",
                                         "best_max_abs_delta_deg"})
            self.assertEqual(data["last_delta_deg_by_id"]["12"], 1.2)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            with self.assertRaisesRegex(ValueError, "already exists"):
                alignment.private_summary_path(path)
            (Path(tmp) / ".git").mkdir()
            with self.assertRaisesRegex(ValueError, "outside Git"):
                alignment.private_summary_path(Path(tmp) / "new.json")

    def test_one_identity_and_position_read_per_axis_and_signed_deltas(self):
        cans = {bus: FakeCAN(bus) for bus in alignment.IDS_BY_BUS}
        identity = alignment.capture_identities(cans, UIDS, lambda: None)
        self.assertEqual(len(identity), 12)
        positions = alignment.collect_positions(cans, lambda: None)
        self.assertEqual(set(positions), set(range(1, 13)))
        self.assertEqual(cans["front"].calls,
                         [(mid, None) for mid in range(1, 7)] +
                         [(mid, "position") for mid in range(1, 7)])
        self.assertEqual(cans["rear"].calls,
                         [(mid, None) for mid in range(7, 13)] +
                         [(mid, "position") for mid in range(7, 13)])
        reference = {mid: mid / 10 for mid in range(1, 13)}
        deltas = alignment.signed_deltas_deg(positions, reference)
        self.assertTrue(all(abs(value - 1.5) < 1e-10 for value in deltas.values()))
        display = alignment.format_screen(deltas, 1.2, 3, 1.5, 2.5)
        self.assertIn("ID01  +1.50°", display)
        self.assertIn("ID12  +1.50°", display)
        self.assertIn("逐次読取り全軸±3°: はい", display)
        self.assertNotIn("0.1", display)  # Absolute raw positions stay private.

    def test_refuses_noncanonical_or_active_transmission(self):
        valid = {"kind": "can_tx", "motor_id": 1, "parameter": "position",
                 "hex": read_request(1, "position").hex()}
        alignment.guard_event("front", valid)
        for row in ({**valid, "hex": read_request(1, "current").hex()},
                    {**valid, "parameter": "identity"},
                    {**valid, "motor_id": 7},
                    {**valid, "parameter": "enable"}):
            with self.assertRaises(RuntimeError):
                alignment.guard_event("front", row)
        with self.assertRaises(RuntimeError):
            alignment.guard_event("front", {"kind": "motor_feedback", "type": 2,
                                            "mode_state": 2})

    def test_wrong_scope_or_missing_read_fails_closed(self):
        front = FakeCAN("front")
        with self.assertRaises(ValueError):
            alignment.collect_positions({"front": front, "rear": front}, lambda: None)
        rear = FakeCAN("rear")
        def fails(_mid, _parameter=None):
            raise TimeoutError("no reply")
        rear.query = fails
        with self.assertRaises(TimeoutError):
            alignment.collect_positions({"front": front, "rear": rear}, lambda: None)

    def test_interactive_session_quits_after_one_complete_read_only_sweep(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "box.json"
            summary = Path(tmp) / "delta-summary.json"
            path.write_text(json.dumps(synthetic_capture()))
            device = os.stat(os.devnull).st_rdev
            bindings = {bus: {"path": f"/dev/serial/by-path/{bus}-test",
                              "resolved": f"/dev/{bus}-test", "st_rdev": device}
                        for bus in alignment.IDS_BY_BUS}
            SessionCAN.sessions.clear()
            with patch.object(alignment.dual, "validate_ports", return_value=bindings), \
                 patch.object(alignment.dual, "binding_matches", return_value=True), \
                 patch.object(alignment.dual, "port_lock", side_effect=lambda _p: contextlib.nullcontext()), \
                 patch.object(alignment, "ownership_locks", return_value=contextlib.nullcontext()), \
                 patch.object(alignment, "ReadOnlyCAN", SessionCAN), \
                 patch.object(alignment, "EXPECTED_BASELINE_SHA256", synthetic_digest(path)), \
                 patch.object(alignment, "_boot_id", return_value="synthetic-boot"), \
                 patch.object(alignment, "_single_key_terminal", return_value=contextlib.nullcontext(0)), \
                 patch.object(alignment, "_quit_key", side_effect=[False, True]), \
                 patch("sys.stdin.isatty", return_value=True), \
                 contextlib.redirect_stdout(io.StringIO()) as output, \
                 patch("sys.stdout.isatty", return_value=True):
                self.assertEqual(alignment.main(["--baseline", str(path),
                    "--seconds", "1", "--summary", str(summary), "--execute-readonly"]), 0)
            self.assertEqual(len(SessionCAN.sessions), 2)
            for can in SessionCAN.sessions:
                self.assertEqual(len(can.calls), 12)
                self.assertTrue(all(parameter in (None, "position")
                                    for _mid, parameter in can.calls))
            self.assertIn("終了: OPERATOR_EXIT", output.getvalue())
            self.assertIn("最終 最大|Δ|=1.50°", output.getvalue())
            saved = json.loads(summary.read_text())
            self.assertEqual(saved["status"], "OPERATOR_EXIT")
            self.assertEqual(saved["complete_sweeps"], 1)
            self.assertEqual(len(saved["last_delta_deg_by_id"]), 12)
            self.assertAlmostEqual(saved["best_max_abs_delta_deg"], 1.5)


if __name__ == "__main__":
    unittest.main()
