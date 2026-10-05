"""No-hardware checks for the post-charge, read-only box-pose comparison."""

import contextlib
import io
import json
import math
import os
from pathlib import Path
import stat
import tempfile
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import post_charge_box_pose_check as check
from singularitydog_hw.can_readonly import PARAMETERS, read_request


BOOT = "test-boot"
UIDS = {str(i): f"{i:016x}" for i in range(1, 13)}


def baseline():
    return {"status": check.BASELINE_STATUS, "boot_id": BOOT,
            "created_ns": 100, "motor_output_allowed": False,
            "rows": {str(i): {"uid": UIDS[str(i)], "run_mode": 0,
                              "current_A": 0.0, "median_position_rad": i / 10,
                              "span_deg": .01} for i in range(1, 13)}}


class FakeCAN:
    opened = []
    wrong_uid = False
    wrong_parameter = False

    def __init__(self, port, event_sink):
        self.bus = "front" if port.endswith("front") else "rear"
        self.sink = event_sink
        self.parser = type("Parser", (), {"buffer": b"", "discarded_bytes": 0})()
        self.serial = self
        self.fd = None
        self.calls = []
        self.position_counts = {}
        FakeCAN.opened.append(self)

    def __enter__(self):
        self.fd = os.open("/dev/null", os.O_RDONLY)
        return self

    def __exit__(self, *_):
        os.close(self.fd)

    def fileno(self):
        return self.fd

    def query(self, mid, parameter=None):
        self.calls.append((mid, parameter))
        self.sink({"kind": "can_tx", "motor_id": mid,
                   "parameter": parameter or "identity",
                   "hex": read_request(mid, parameter).hex()})
        before = time.monotonic_ns()
        row = {"ok": True, "request_monotonic_ns": before,
               "monotonic_ns": before + 1}
        if parameter is None:
            row["mcu_uid_hex"] = ("f" * 16 if self.wrong_uid and mid == 7
                                  else UIDS[str(mid)])
        else:
            index, _, unit = PARAMETERS[parameter]
            row.update(index=index + (1 if self.wrong_parameter and mid == 7 else 0),
                       unit=unit)
            if parameter == "position":
                count = self.position_counts.get(mid, 0)
                self.position_counts[mid] = count + 1
                row["value"] = mid / 10 + (2 * math.pi if mid == 3 else 0) + count * .001
            else:
                row["value"] = {"run_mode": 0, "current": 0., "voltage": 40.}[parameter]
        return row


class PostChargeTests(unittest.TestCase):
    def setUp(self):
        FakeCAN.opened = []
        FakeCAN.wrong_uid = False
        FakeCAN.wrong_parameter = False

    def _run(self, directory, *, execute=True):
        root = Path(directory)
        source = root / "baseline.json"
        source.write_text(json.dumps(baseline()))
        output = root / "private-result.json"
        binding = {"path": "/dev/front", "resolved": "/dev/null",
                   "st_rdev": os.stat("/dev/null").st_rdev}
        bindings = {"front": dict(binding),
                    "rear": {**binding, "path": "/dev/rear"}}
        argv = ["--baseline", str(source), "--front-port", "/dev/front",
                "--rear-port", "/dev/rear", "--output", str(output)]
        if execute:
            argv.append("--execute-readonly")
        with patch.object(check, "_boot_id", return_value=BOOT), \
             patch.object(check.dual, "validate_ports", return_value=bindings), \
             patch.object(check.dual, "binding_matches", return_value=True), \
             patch.object(check.dual, "port_lock", return_value=contextlib.nullcontext()), \
             patch.object(check, "ownership_locks", return_value=contextlib.nullcontext()), \
             patch.object(check, "ReadOnlyCAN", FakeCAN), \
             contextlib.redirect_stdout(io.StringIO()):
            code = check.main(argv)
        return code, output

    def test_complete_read_keeps_direct_360_degree_branch_and_private_file(self):
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 0)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "RECORDED_REVIEW_REQUIRED")
        self.assertFalse(result["angle_wrap_applied"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertEqual(set(result["identities"]), check.ALL_IDS)
        self.assertEqual(set(result["telemetry"]["rows"]), check.ALL_IDS)
        self.assertAlmostEqual(result["direct_delta_by_id"]["3"]["direct_delta_deg"],
                               360 + math.degrees(.001), places=6)
        self.assertEqual(len(result["telemetry"]["rows"]["3"]["position_samples"]), 3)
        for can in FakeCAN.opened:
            allowed = check.IDS_BY_BUS[can.bus]
            self.assertEqual(len(can.calls), 6 + 6 * (3 + 3))
            self.assertTrue(all(mid in allowed and parameter in (None, *check.READS)
                                for mid, parameter in can.calls))
            self.assertTrue(all(parameter is None for _, parameter in can.calls[:6]))

    def test_uid_mismatch_stops_before_type17(self):
        FakeCAN.wrong_uid = True
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 1)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIsNone(result["telemetry"])
        self.assertIn("UID mismatch", result["errors"][0])
        self.assertTrue(all(parameter is None for can in FakeCAN.opened
                            for _, parameter in can.calls))

    def test_parameter_mismatch_fails_closed(self):
        FakeCAN.wrong_parameter = True
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 1)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("parameter mismatch", result["errors"][0])
        self.assertIsNone(result["direct_delta_by_id"])

    def test_plan_does_not_open_can_or_write(self):
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory, execute=False)
            self.assertEqual(code, 0)
            self.assertFalse(output.exists())
            self.assertEqual(FakeCAN.opened, [])

    def test_publish_durability_failure_has_no_cli_success_even_with_complete_final_json(self):
        fsync = os.fsync
        def fail(fd):
            if stat.S_ISDIR(os.fstat(fd).st_mode): raise OSError("directory fsync")
            fsync(fd)
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(check.os, "fsync", side_effect=fail), contextlib.redirect_stderr(io.StringIO()) as errors:
            code, output = self._run(directory)
            self.assertEqual(code, 1)
            self.assertEqual(json.loads(output.read_text())["status"], "RECORDED_REVIEW_REQUIRED")
            self.assertIn("Private output could not be saved", errors.getvalue())

    def test_baseline_and_tx_guard_reject_invalid_data(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "baseline.json"
            data = baseline()
            data["rows"]["9"]["span_deg"] = .2
            source.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "invalid raw position"):
                check.load_baseline(source)
            source.write_text('{"status":1,"status":2}')
            with self.assertRaisesRegex(ValueError, "Duplicate key"):
                check.load_baseline(source)
        with self.assertRaisesRegex(RuntimeError, "Unexpected CAN transmission"):
            check.guard_event("front", {"kind": "can_tx", "motor_id": 1,
                                        "parameter": "enable", "hex": ""})
        with self.assertRaisesRegex(RuntimeError, "Noncanonical CAN transmission"):
            check.guard_event("front", {"kind": "can_tx", "motor_id": 1,
                                        "parameter": "position", "hex": "bad"})


class PrivateJSONTests(unittest.TestCase):
    """Only temporary files; no serial, CAN, SSH or recovered artifacts."""
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name).resolve()
        self.output = self.root / "capture.json"
        self.payload = {"status": "RECORDED_REVIEW_REQUIRED", "note": "静止", "output_allowed": False}

    def pending(self):
        return list(self.root.glob(".capture.json.pending-*"))

    def stream(self, events=None, *, write=None, close=None):
        original = os.fdopen
        def factory(fd, *args, **kwargs):
            underlying = original(fd, *args, **kwargs)
            class Wrapped:
                def __enter__(self): return self
                def __exit__(self, *_): self.close()
                def fileno(self): return underlying.fileno()
                def write(self, data):
                    if events is not None: events.append("write")
                    return write(underlying, data) if write else underlying.write(data)
                def flush(self):
                    if events is not None: events.append("flush")
                    underlying.flush()
                def close(self):
                    underlying.close()
                    if events is not None: events.append("close")
                    if close: close()
            return Wrapped()
        return factory

    def test_complete_bytes_close_before_publish_directory_barrier_and_retained_reservation(self):
        events = []
        fsync, link = os.fsync, os.link
        def sync(fd):
            events.append("dirsync" if stat.S_ISDIR(os.fstat(fd).st_mode) else "filesync")
            fsync(fd)
        def publish(*args, **kwargs):
            self.assertFalse(self.output.exists())
            self.assertEqual(len(self.pending()), 1)
            self.assertEqual(json.loads(self.pending()[0].read_text()), self.payload)
            self.assertEqual(stat.S_IMODE(self.pending()[0].stat().st_mode), 0o600)
            events.append("publish"); link(*args, **kwargs)
        with patch.object(check.os, "fdopen", side_effect=self.stream(events)), \
             patch.object(check.os, "fsync", side_effect=sync), \
             patch.object(check.os, "link", side_effect=publish), \
             patch.object(check.os, "unlink", side_effect=AssertionError("No inode-conditional unlink")) as unlink:
            self.assertIsNone(check._write_private(self.output, self.payload))
        unlink.assert_not_called()
        self.assertEqual(events, ["write", "flush", "filesync", "close", "publish", "dirsync"])
        self.assertEqual(self.output.read_bytes(), (json.dumps(self.payload, ensure_ascii=False, indent=2, allow_nan=False)+"\n").encode())
        self.assertEqual(stat.S_IMODE(self.output.stat().st_mode), 0o600)
        self.assertEqual(len(self.pending()), 1)
        self.assertEqual(self.pending()[0].stat().st_ino, self.output.stat().st_ino)
        self.assertEqual(self.pending()[0].read_bytes(), self.output.read_bytes())
        self.assertNotEqual(self.pending()[0].suffix, ".json")

    def test_serialization_failure_creates_no_file(self):
        for payload in ({"value": math.nan}, {"value": math.inf}, {"value": object()}):
            with self.assertRaises((ValueError, TypeError)):
                check._write_private(self.output, payload)
            self.assertEqual(list(self.root.iterdir()), [])

    def test_existing_or_competing_target_remains_unchanged(self):
        self.output.write_bytes(b"foreign-original")
        with self.assertRaises(FileExistsError): check._write_private(self.output, self.payload)
        self.assertEqual(self.output.read_bytes(), b"foreign-original")
        self.assertEqual(self.pending(), [])
        self.output.unlink()
        original_link = os.link
        def compete(*args, **kwargs):
            self.output.write_bytes(b"foreign-racer")
            return original_link(*args, **kwargs)
        with patch.object(check.os, "link", side_effect=compete), self.assertRaises(FileExistsError):
            check._write_private(self.output, self.payload)
        self.assertEqual(self.output.read_bytes(), b"foreign-racer")
        self.assertEqual(len(self.pending()), 1)

    def test_two_actual_publishers_have_one_winner_and_keep_both_reservations(self):
        from concurrent.futures import ThreadPoolExecutor
        import threading
        barrier = threading.Barrier(2); link = os.link
        def compete(*args, **kwargs):
            barrier.wait(timeout=5)
            return link(*args, **kwargs)
        def save(value):
            try:
                check._write_private(self.output, value)
                return "saved", value
            except FileExistsError:
                return "competed", value
        a, b = {"writer": "a"}, {"writer": "b"}
        with patch.object(check.os, "link", side_effect=compete), ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(save, (a, b)))
        saved = [v for status, v in results if status == "saved"]
        competed = [v for status, v in results if status == "competed"]
        self.assertEqual(len(saved), 1); self.assertEqual(len(competed), 1)
        self.assertEqual(json.loads(self.output.read_text()), saved[0])
        self.assertEqual(len(self.pending()), 2)
        self.assertEqual(sorted(json.loads(p.read_text())["writer"] for p in self.pending()), ["a", "b"])

    def test_competing_symlink_target_and_its_destination_are_never_deleted(self):
        foreign = self.root / "foreign"; foreign.write_bytes(b"keep")
        original_link = os.link
        def compete(*args, **kwargs):
            self.output.symlink_to(foreign)
            return original_link(*args, **kwargs)
        with patch.object(check.os, "link", side_effect=compete), self.assertRaises(FileExistsError):
            check._write_private(self.output, self.payload)
        self.assertTrue(self.output.is_symlink()); self.assertEqual(foreign.read_bytes(), b"keep")

    def test_partial_writes_complete_but_zero_progress_or_error_never_publish(self):
        with patch.object(check.os, "fdopen", side_effect=self.stream(write=lambda f, data: f.write(data[:7]))):
            check._write_private(self.output, self.payload)
        self.assertEqual(json.loads(self.output.read_text()), self.payload)
        self.output.unlink()
        for kind in ("zero", "partial"):
            def fail(stream, data):
                if kind == "zero": return 0
                stream.write(data[:7]); raise OSError("synthetic partial write")
            with patch.object(check.os, "fdopen", side_effect=self.stream(write=fail)), self.assertRaises(OSError):
                check._write_private(self.output, self.payload)
            self.assertFalse(self.output.exists())
        full_size = len((json.dumps(self.payload, ensure_ascii=False, indent=2)+"\n").encode())
        self.assertEqual(sorted(p.stat().st_size for p in self.pending()), [0, 7, full_size])

    def test_file_fsync_close_and_unsupported_publish_fail_before_final_name(self):
        failures = (("fsync", patch.object(check.os, "fsync", side_effect=OSError("file fsync"))),
                    ("close", patch.object(check.os, "fdopen", side_effect=self.stream(close=lambda: (_ for _ in ()).throw(OSError("close"))))),
                    ("link", patch.object(check.os, "link", side_effect=OSError("hardlink unavailable"))))
        for kind, failure in failures:
            with self.subTest(kind=kind), failure, self.assertRaises(OSError):
                check._write_private(self.output, self.payload)
            self.assertFalse(self.output.exists())
        self.assertEqual(len(self.pending()), 3)

    def test_directory_fsync_failure_reports_error_even_if_complete_final_exists(self):
        fsync = os.fsync
        def sync(fd):
            if stat.S_ISDIR(os.fstat(fd).st_mode): raise OSError("directory barrier")
            fsync(fd)
        with patch.object(check.os, "fsync", side_effect=sync), self.assertRaises(OSError):
            check._write_private(self.output, self.payload)
        self.assertEqual(json.loads(self.output.read_text()), self.payload)
        self.assertEqual(len(self.pending()), 1)

    def test_directory_close_failure_is_not_success_and_never_deletes_paths(self):
        close = os.close
        def fail(fd):
            directory = stat.S_ISDIR(os.fstat(fd).st_mode)
            close(fd)
            if directory: raise OSError("directory close")
        with patch.object(check.os, "close", side_effect=fail), \
             patch.object(check.os, "unlink", side_effect=AssertionError("No unlink")) as unlink, self.assertRaises(OSError):
            check._write_private(self.output, self.payload)
        unlink.assert_not_called()
        self.assertEqual(json.loads(self.output.read_text()), self.payload)
        self.assertEqual(len(self.pending()), 1)

    def test_git_symlink_parent_and_symlink_leaf_rejected_without_overwrite(self):
        git = self.root / "repo"; git.mkdir(); (git / ".git").mkdir()
        alias = self.root / "alias"; alias.symlink_to(self.root, target_is_directory=True)
        leaf = self.root / "leaf"; leaf.symlink_to(self.root / "absent")
        for target in (git / "capture.json", alias / "capture.json", leaf):
            with self.subTest(path=str(target)), self.assertRaises(ValueError):
                check._write_private(target, self.payload)
        self.assertTrue(leaf.is_symlink()); self.assertFalse(self.output.exists())
        self.assertEqual(list(git.glob("*.pending-*")), [])

    def test_replaced_parent_before_publish_does_not_write_foreign_directory(self):
        parent = self.root / "parent"; parent.mkdir(); output = parent / "capture.json"
        moved = self.root / "original-parent"
        def replace():
            parent.rename(moved); parent.mkdir(); (parent / "capture.json").write_bytes(b"foreign")
        with patch.object(check.os, "fdopen", side_effect=self.stream(close=replace)), self.assertRaisesRegex(ValueError, "Output parent differs"):
            check._write_private(output, self.payload)
        self.assertEqual(output.read_bytes(), b"foreign")
        self.assertFalse((moved / "capture.json").exists())
        self.assertEqual(len(list(moved.glob(".capture.json.pending-*"))), 1)

    def test_raw_requested_parent_redirected_before_resolve_is_rejected(self):
        parent = self.root / "parent"; parent.mkdir()
        moved = self.root / "original-parent"
        redirected = self.root / "redirected"; redirected.mkdir()
        original = check.private_output_path
        def redirect(path):
            parent.rename(moved); parent.symlink_to(redirected, target_is_directory=True)
            return original(path)
        with patch.object(check, "private_output_path", side_effect=redirect), self.assertRaisesRegex(ValueError, "Requested output path changed"):
            check._write_private(parent / "capture.json", self.payload)
        self.assertEqual(list(redirected.iterdir()), [])
        self.assertEqual(list(moved.iterdir()), [])
        self.assertTrue(parent.is_symlink())

    def test_foreign_staging_replacement_is_never_published_or_cleaned_up(self):
        saved = self.root / "original-pending"
        def replace():
            pending = self.pending()[0]; pending.rename(saved); pending.write_bytes(b"foreign")
        with patch.object(check.os, "fdopen", side_effect=self.stream(close=replace)), self.assertRaisesRegex(ValueError, "pathname differs"):
            check._write_private(self.output, self.payload)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.pending()[0].read_bytes(), b"foreign")
        self.assertEqual(json.loads(saved.read_text()), self.payload)

    def test_foreign_staging_replaced_after_publish_is_not_unlinked(self):
        fsync = os.fsync; replaced = False
        def sync(fd):
            nonlocal replaced
            fsync(fd)
            if stat.S_ISDIR(os.fstat(fd).st_mode) and not replaced:
                replaced = True
                pending = self.pending()[0]; pending.rename(self.root / "original-pending")
                pending.write_bytes(b"foreign")
        with patch.object(check.os, "fsync", side_effect=sync), self.assertRaisesRegex(ValueError, "pathname differs"):
            check._write_private(self.output, self.payload)
        self.assertEqual(self.pending()[0].read_bytes(), b"foreign")
        self.assertEqual(json.loads(self.output.read_text()), self.payload)

    def test_foreign_final_replacement_after_publish_is_preserved_on_failure(self):
        fsync = os.fsync; replaced = False
        def sync(fd):
            nonlocal replaced
            fsync(fd)
            if stat.S_ISDIR(os.fstat(fd).st_mode) and not replaced:
                replaced = True
                self.output.rename(self.root / "original-final")
                self.output.write_bytes(b"foreign-final")
        with patch.object(check.os, "fsync", side_effect=sync), \
             patch.object(check.os, "unlink", side_effect=AssertionError("Never delete foreign entries")) as unlink, \
             self.assertRaisesRegex(ValueError, "pathname differs"):
            check._write_private(self.output, self.payload)
        unlink.assert_not_called()
        self.assertEqual(self.output.read_bytes(), b"foreign-final")
        self.assertEqual(json.loads(self.pending()[0].read_text()), self.payload)


if __name__ == "__main__":
    unittest.main()
