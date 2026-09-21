"""Offline pose library checks with synthetic motor identities and telemetry."""
import contextlib
import io
import json
import math
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import PARAMETERS, read_request
from singularitydog_hw.pose_record import build_record, save_record, main, ObservationError


def snapshot(root, name, start=1_000_000_000, change=0.0):
    directory = Path(root) / name
    directory.mkdir()
    records, positions = [], {}
    def emit(event):
        moment = start + len(records) * 1_000_000
        records.append({"monotonic_ns": moment, "wall_time_ns": moment + 1_790_000_000_000_000_000, **event})
    plan = {"ids": list(range(1, 13)), "sweeps": 3, "allowed_can_types": [0, 17],
            "motor_output_available": False, "parameters": ["identity", "position", "current", "velocity", "voltage"]}
    emit({"kind": "capture_metadata", **plan})
    queries = [(i, "identity") for i in range(1, 13)] + [
        (i, p) for _ in range(3) for i in range(1, 13) for p in ("position", "current", "velocity", "voltage")]
    for seq, (mid, parameter) in enumerate(queries, 1):
        emit({"kind": "can_tx", "sequence": seq, "motor_id": mid, "parameter": parameter,
              "hex": read_request(mid, None if parameter == "identity" else parameter).hex()})
        reply = {"kind": "motor_parameter", "sequence": seq, "motor_id": mid, "parameter": parameter,
                 "ok": True, "request_monotonic_ns": records[-1]["monotonic_ns"]}
        if parameter == "identity":
            reply["mcu_uid_hex"] = f"{mid:016x}"
        else:
            value = {"position": mid * 0.1 + (change if mid == 3 else 0), "current": 0.0, "velocity": 0.0, "voltage": 39.0}[parameter]
            reply.update(index=PARAMETERS[parameter][0], unit=PARAMETERS[parameter][2], status=0, value=value)
            if parameter == "position":
                positions[str(mid)] = {"samples": 3, "mean": value, "min": value, "max": value, "last": value}
        emit(reply)
        if seq > 12 and (seq - 12) % 48 == 0:
            emit({"kind": "joint_snapshot_sweep", "sweep": (seq - 12) // 48})
    meta = {"status": "RECORDED_NOT_CALIBRATED", "errors": [], "discarded_rx_bytes": 0,
            "started_at": "2026-09-20T17:00:00+09:00", "completed_at": "2026-09-20T17:00:02+09:00",
            "plan": plan,
            "tx_count": len(queries), "summary": {"identities": {str(i): f"{i:016x}" for i in range(1, 13)},
                "position_unit": "rad_output_shaft", "automatic_wrap_applied": False, "positions": positions}}
    write(directory, records, meta)
    return directory


def write(directory, records, meta):
    (directory / "events.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
    (directory / "summary.json").write_text(json.dumps(meta))


def edit(directory, change):
    records = [json.loads(r) for r in (directory / "events.jsonl").read_text().splitlines()]
    meta = json.loads((directory / "summary.json").read_text())
    change(records, meta)
    write(directory, records, meta)


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.before = snapshot(self.root, "before")
        self.after = snapshot(self.root, "after", start=5_000_000_000, change=0.2)

    def test_named_baseline_is_immutable_private_and_not_calibration(self):
        output = self.root / "poses" / "p00"
        hashes_before = [(p.name, p.read_bytes()) for p in self.before.iterdir()]
        record = save_record(self.before, "p00", "Supported current pose", output)
        self.assertEqual(record["sampling_quality"], "stationary_candidate")
        self.assertEqual(len(record["motors"]), 12)
        for key in ("approved_for_runtime", "zero_inferred", "sign_inferred", "angle_wrapping_applied",
                    "motor_enable_state_verified", "motor_power_cycle_continuity_verified", "pose_replay_available"):
            self.assertIs(record[key], False)
        self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o700)
        for name in ("pose.json", "POSE.md"):
            self.assertEqual(stat.S_IMODE((output / name).stat().st_mode), 0o600)
        self.assertEqual(hashes_before, [(p.name, p.read_bytes()) for p in self.before.iterdir()])
        with self.assertRaises(FileExistsError):
            save_record(self.before, "p00", "again", output)

    def test_reference_reports_all_ids_and_preserves_direct_delta(self):
        record = build_record(self.after, "p01", "Known hand-positioned pose", self.before)
        diff = record["reference_comparison"]
        self.assertAlmostEqual(diff["changes"]["3"]["median_delta_rad"], 0.2)
        self.assertEqual(diff["changes"]["4"]["median_delta_rad"], 0)
        self.assertFalse(diff["power_cycle_continuity_verified"])
        large = snapshot(self.root, "large", start=9_000_000_000, change=-6.1)
        record = build_record(large, "p02", "Potential reset or turn change", self.before)
        self.assertAlmostEqual(record["reference_comparison"]["changes"]["3"]["median_delta_rad"], -6.1)
        self.assertEqual(record["reference_comparison"]["large_raw_change_or_discontinuity_ids"], [3])

    def test_incomplete_or_error_summary_rejected_without_output(self):
        for key, value in (("status", "INCOMPLETE"), ("errors", ["interrupted"]), ("discarded_rx_bytes", 1)):
            source = snapshot(self.root, "bad-" + key)
            edit(source, lambda r, m: m.update({key: value}))
            with self.assertRaises(ObservationError):
                save_record(source, "bad", "bad source", self.root / "invalid")
            self.assertFalse((self.root / "invalid").exists())

    def test_motion_command_hidden_in_tx_rejected(self):
        edit(self.after, lambda r, m: r[1].update(hex="41542007e81c0800000000000000000d0a"))
        with self.assertRaisesRegex(ObservationError, "request bytes"):
            build_record(self.after, "bad", "motion capture")

    def test_missing_telemetry_or_wrong_sequence_rejected(self):
        edit(self.after, lambda r, m: r.pop())
        with self.assertRaises(ObservationError):
            build_record(self.after, "bad", "truncated capture")
        edit(self.before, lambda r, m: r[2].update(sequence=2))
        with self.assertRaises(ObservationError):
            build_record(self.before, "bad", "misassociated capture")

    def test_uid_changes_reject_comparison(self):
        def change(records, meta):
            records[2]["mcu_uid_hex"] = "ffffffffffffffff"
            meta["summary"]["identities"]["1"] = "ffffffffffffffff"
        edit(self.after, change)
        with self.assertRaisesRegex(ObservationError, "UID mapping"):
            build_record(self.after, "p01", "wrong motor", self.before)

    def test_same_or_reversed_capture_rejected(self):
        for reference, current in ((self.before, self.before), (self.after, self.before)):
            with self.assertRaises(ObservationError):
                build_record(current, "bad", "bad ordering", reference)

    def test_current_and_velocity_outliers_are_not_averaged_away(self):
        def change(records, meta):
            next(r for r in records if r["kind"] == "motor_parameter" and r["parameter"] == "current")["value"] = 0.4
            next(r for r in records if r["kind"] == "motor_parameter" and r["parameter"] == "velocity")["value"] = 0.3
        edit(self.after, change)
        report = build_record(self.after, "p01", "moving or loaded")
        self.assertEqual(report["sampling_quality"], "review_required")
        self.assertEqual(report["motors"]["1"]["sampling_issues"], ["velocity", "current"])

    def test_summary_cannot_hide_position_spread(self):
        def change(records, meta):
            next(r for r in records if r["kind"] == "motor_parameter" and r["parameter"] == "position")["value"] += 0.3
        edit(self.after, change)
        with self.assertRaisesRegex(ObservationError, "statistics"):
            build_record(self.after, "bad", "contradictory summary")

    def test_bad_core_units_and_nonfinite_rejected(self):
        def change(records, meta):
            next(r for r in records if r["kind"] == "motor_parameter" and r["parameter"] == "current")["unit"] = "mA"
        edit(self.after, change)
        with self.assertRaises(ObservationError):
            build_record(self.after, "bad", "wrong unit")
        edit(self.before, lambda r, m: r[-1].update(value=math.nan))
        with self.assertRaises(ObservationError):
            build_record(self.before, "bad", "nan")

    def test_git_output_symlink_and_path_names_rejected(self):
        git = self.root / "repo"
        git.mkdir()
        (git / ".git").write_text("gitdir: elsewhere")
        with self.assertRaises(ObservationError):
            save_record(self.before, "p00", "private", git / "pose")
        link = self.root / "link"
        link.symlink_to(git, target_is_directory=True)
        with self.assertRaises(ObservationError):
            save_record(self.before, "p00", "private", link / "pose")
        for name in ("../escape", "P00", "", "x" * 65):
            with self.assertRaises(ObservationError):
                build_record(self.before, name, "private")

    def test_cli_opens_no_hardware(self):
        with patch("singularitydog_hw.can_readonly.ReadOnlyCAN") as can, contextlib.redirect_stdout(io.StringIO()):
            rc = main(["--capture", str(self.before), "--name", "p00", "--note", "supported",
                       "--output", str(self.root / "out")])
        self.assertEqual(rc, 0)
        can.assert_not_called()

    def test_impossible_reply_order_and_wrong_request_clock_rejected(self):
        def reorder(records, meta):
            records[:] = [records[0]] + [r for r in records if r["kind"] == "motor_parameter"] + [r for r in records if r["kind"] not in ("capture_metadata", "motor_parameter")]
        edit(self.after, reorder)
        with self.assertRaises(ObservationError):
            build_record(self.after, "bad", "reordered")
        edit(self.before, lambda r, m: r[2].update(request_monotonic_ns=123))
        with self.assertRaises(ObservationError):
            build_record(self.before, "bad", "wrong request clock")

    def test_missing_metadata_and_sweep_rejected(self):
        edit(self.after, lambda r, m: r.pop(0))
        with self.assertRaises(ObservationError):
            build_record(self.after, "bad", "missing metadata")
        edit(self.before, lambda r, m: r.pop())
        with self.assertRaises(ObservationError):
            build_record(self.before, "bad", "missing final sweep")


if __name__ == "__main__":
    unittest.main()
