import contextlib
import io
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from singularitydog_hw.joint_snapshot import collect, main


class FakeCAN:
    def __init__(self, bad=None):
        self.calls = []
        self.bad = bad

    def query(self, mid, parameter=None):
        self.calls.append((mid, parameter))
        result = ({"ok": True, "mcu_uid_hex": f"{mid:016x}"} if parameter is None else
                  {"ok": True, "value": mid / 10})
        return self.bad(mid, parameter, result) if self.bad else result


class SnapshotTests(unittest.TestCase):
    def test_covers_every_id_only_identity_and_core_reads(self):
        can = FakeCAN()
        result = collect(can, lambda _: None, sweeps=3, wait=lambda _: None)
        self.assertEqual(len(can.calls), 12 + 3 * 48)
        self.assertFalse(result["joint_calibration_verified"])
        self.assertFalse(result["automatic_wrap_applied"])
        self.assertEqual(result["positions"]["1"]["samples"], 3)
        self.assertEqual({p for _, p in can.calls}, {None, "position", "current", "velocity", "voltage"})

    def test_failed_or_nonfinite_read_aborts(self):
        for bad in ({"ok": False, "value": 0}, {"ok": True, "value": math.nan},
                    {"ok": True, "value": True}):
            can = FakeCAN(lambda i, p, r: bad if i == 2 and p == "position" else r)
            with self.assertRaises(RuntimeError):
                collect(can, lambda _: None, sweeps=3, wait=lambda _: None)
            self.assertEqual(can.calls[-1], (2, "position"))

    def test_duplicate_uid_aborts_before_position_reads(self):
        can = FakeCAN(lambda i, p, r: {"ok": True, "mcu_uid_hex": "a" * 16})
        with self.assertRaises(RuntimeError):
            collect(can, lambda _: None, sweeps=3, wait=lambda _: None)
        self.assertEqual(can.calls, [(1, None), (2, None)])

    def test_interruption_stops_before_next_request(self):
        can = FakeCAN()
        def interrupt():
            if len(can.calls) == 14:
                raise InterruptedError()
        with self.assertRaises(InterruptedError):
            collect(can, lambda _: None, sweeps=3, check_interrupt=interrupt, wait=lambda _: None)
        self.assertEqual(len(can.calls), 14)

    def test_dry_run_never_opens_hardware_or_creates_output(self):
        with tempfile.TemporaryDirectory() as tmp, patch("singularitydog_hw.joint_snapshot.ReadOnlyCAN") as can:
            path = Path(tmp) / "capture"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--output", str(path)]), 0)
            can.assert_not_called()
            self.assertFalse(path.exists())

    def test_reject_git_output_and_existing_directory(self):
        with tempfile.TemporaryDirectory() as tmp, patch("singularitydog_hw.joint_snapshot.ReadOnlyCAN") as can:
            root = Path(tmp)
            (root / ".git").mkdir()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(["--execute", "--output", str(root / "capture")])
            can.assert_not_called()
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(FileExistsError):
            main(["--execute", "--output", tmp])


if __name__ == "__main__":
    unittest.main()
