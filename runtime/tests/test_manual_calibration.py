"""Offline observations and terminal-flow checks; no physical calibration claims."""
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import manual_calibration as manual


def synthetic_pose(changes=None):
    """Twelve stable, uniquely identified motors, with direct unwrapped changes."""
    changes = changes or {}
    motors = {}
    for mid in range(1, 13):
        radians = mid / 10 + math.radians(changes.get(mid, 0))
        motors[str(mid)] = {
            "mcu_uid_hex": f"{mid:016x}",
            "median_deg_raw_shaft": math.degrees(radians),
            "position": {"median_rad": radians, "peak_to_peak_rad": .001},
            "max_abs_current_A": 0., "max_abs_velocity_rad_s": 0.,
        }
    return {"motors": motors, "completed_at": "2026-09-21T10:00:00+09:00"}


def observation(stage, pose=None, baseline=None):
    pose = pose or synthetic_pose()
    quality = manual.analyse(stage, pose, baseline)
    return {"stage_id": stage["id"], "stage": stage, "motors": pose["motors"],
            "path": f"/synthetic/observations/{stage['id']}",
            "quality": quality, "accepted": not quality["blockers"], "status": "RECORDED"}


def complete_records(stages, sign=1):
    baseline = synthetic_pose()
    result = []
    for stage in stages:
        changes = {}
        if stage["moving_id"]:
            kind = stage["kind"]
            base_nominal = -90 if kind == "calf" else 0
            nominal_delta = stage["target_nominal_deg"][kind] - base_nominal
            # Hand placement is approximate: deliberately observe 8°, not the nominal 10°.
            changes[stage["moving_id"]] = math.copysign(8, nominal_delta) * sign
        result.append(observation(stage, synthetic_pose(changes),
                                  baseline if stage["kind"] != "l" else None))
    return result


class StageTests(unittest.TestCase):
    def test_twenty_poses_use_robot_frame_hip_signs_and_model_limits(self):
        stages = manual.make_stages()
        self.assertEqual(len(stages), 20)
        self.assertEqual(len({stage["id"] for stage in stages}), 20)
        # fabo_robotdog_d17_hardware_r1.urdf joint limits, in radians.
        limits = {"hip": (-.5, .5), "thigh": (-.9, 1.2), "calf": (-2.2, -.08)}
        for leg, ids in {"FR": [1, 2, 3], "FL": [4, 5, 6],
                         "RR": [7, 8, 9], "RL": [10, 11, 12]}.items():
            with self.subTest(leg=leg):
                selected = [stage for stage in stages if stage["leg"] == leg]
                self.assertEqual([stage["kind"] for stage in selected],
                                 ["l", "calf", "thigh", "hip", "return"])
                self.assertEqual([stage["moving_id"] for stage in selected],
                                 [None, ids[0], ids[1], ids[2], None])
                self.assertEqual(selected[3]["target_nominal_deg"]["hip"],
                                 -10 if leg in ("FR", "RR") else 10)
                self.assertEqual(selected[0]["target_nominal_deg"],
                                 {"hip": 0, "thigh": 0, "calf": -90})
                self.assertEqual(selected[0]["target_nominal_deg"], selected[-1]["target_nominal_deg"])
                for stage in selected:
                    self.assertEqual(stage["ids"], ids)
                    self.assertIs(stage["target_measured"], False)
                    for joint, nominal in stage["target_nominal_deg"].items():
                        low, high = limits[joint]
                        self.assertGreaterEqual(math.radians(nominal), low)
                        self.assertLessEqual(math.radians(nominal), high)

    def test_one_leg_selection_keeps_five_manual_steps(self):
        stages = manual.make_stages(["RL"])
        self.assertEqual(len(stages), 5)
        self.assertEqual({stage["leg"] for stage in stages}, {"RL"})
        self.assertIn("下脚も一緒に回ります", manual.guidance(stages[2]))
        self.assertIn("測定済み角度や駆動目標にはしません", manual.guidance(stages[3]))

    def test_two_calf_recheck_retains_reference_and_return_without_other_candidates(self):
        stages = manual.make_stages(["FL", "RR"], "calf")
        self.assertEqual([s["id"] for s in stages],
                         ["fl-l", "fl-calf", "fl-return", "rr-l", "rr-calf", "rr-return"])
        self.assertEqual([s["ids"] for s in stages], [[4, 5, 6]] * 3 + [[7, 8, 9]] * 3)
        for sign in (-1, 1):
            signed = manual.build_candidates(stages, complete_records(stages, sign))
            self.assertEqual([c["sign_candidate"] for c in signed], [sign, sign])
        records = complete_records(stages)
        candidates = manual.build_candidates(stages, records)
        self.assertEqual([c["motor_id"] for c in candidates], [4, 7])
        self.assertTrue(all(c["approved_for_runtime"] is False for c in candidates))
        without_rr_return = manual.build_candidates(stages, records[:-1])
        self.assertEqual(without_rr_return[0]["sign_candidate"], 1)
        self.assertIsNone(without_rr_return[1]["sign_candidate"])

    def test_subset_still_rejects_other_axis_motion_and_return_drift(self):
        stages = manual.make_stages(["FL"], "calf")
        records = complete_records(stages)
        for index, changes in ((1, {4: 8, 5: 4}), (2, {6: 4})):
            with self.subTest(index=index):
                bad = list(records)
                bad[index] = observation(stages[index], synthetic_pose(changes), synthetic_pose())
                self.assertIsNone(manual.build_candidates(stages, bad)[0]["sign_candidate"])

    def test_selection_errors_happen_without_capture(self):
        for legs, joint in (([], None), (["FL", "FL"], "calf"), (["XX"], None), (["FL"], "bad")):
            with self.subTest(legs=legs, joint=joint), self.assertRaises(ValueError):
                manual.make_stages(legs, joint)

    def test_subset_cli_guide_does_not_open_transport(self):
        with patch.object(manual.joint_snapshot, "main") as capture, contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(manual.main(["--legs", "FL", "RR", "--joint", "calf"]), 0)
        capture.assert_not_called()
        self.assertIn("[6/6]", out.getvalue())
        self.assertNotIn("上脚を少し前へ", out.getvalue())

    def test_cli_rejects_ambiguous_or_duplicate_legs(self):
        for args in (["--leg", "FL", "--legs", "RR"], ["--legs", "FL", "FL"]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                manual.main(args)
            self.assertEqual(error.exception.code, 2)


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        self.stages = manual.make_stages(["FR"])
        self.baseline = synthetic_pose()

    def assert_rejected(self, stage, pose, baseline=None):
        report = manual.analyse(stage, pose, baseline)
        self.assertEqual(report["status"], "RETRY_RECOMMENDED")
        self.assertTrue(report["blockers"])
        self.assertIs(report["approved_for_runtime"], False)
        return report

    def test_stable_baseline_and_isolated_manual_change_are_observations_only(self):
        base = manual.analyse(self.stages[0], self.baseline)
        moved = manual.analyse(self.stages[1], synthetic_pose({1: 8}), self.baseline)
        for report in (base, moved):
            self.assertEqual(report["status"], "OBSERVATION_CANDIDATE")
            self.assertEqual(report["blockers"], [])
            self.assertIs(report["approved_for_runtime"], False)
        self.assertEqual(len(moved["delta_deg"]), 12)
        self.assertAlmostEqual(moved["delta_deg"]["1"], 8)
        self.assertEqual(moved["delta_deg"]["2"], 0)

    def test_position_spread_and_current_reject_even_the_baseline(self):
        for field, value in (("spread", .03), ("current", .051)):
            with self.subTest(field=field):
                pose = synthetic_pose()
                if field == "spread":
                    pose["motors"]["2"]["position"]["peak_to_peak_rad"] = value
                else:
                    pose["motors"]["2"]["max_abs_current_A"] = value
                self.assert_rejected(self.stages[0], pose)

    def test_two_changed_axes_require_retry(self):
        self.assert_rejected(self.stages[1], synthetic_pose({1: 8, 2: 3.1}), self.baseline)

    def test_large_jump_is_preserved_without_wrapping_and_refuses_candidate(self):
        for jump in (-350, 46, 360):
            with self.subTest(jump=jump):
                report = self.assert_rejected(self.stages[1], synthetic_pose({1: jump}), self.baseline)
                self.assertAlmostEqual(report["delta_deg"]["1"], jump)
                records = complete_records(self.stages)
                records[1] = observation(self.stages[1], synthetic_pose({1: jump}), self.baseline)
                candidate = manual.build_candidates(self.stages, records)[0]
                self.assertIsNone(candidate["sign_candidate"])
                self.assertIsNone(candidate["offset_candidate_rad"])

    def test_too_small_change_and_missing_baseline_do_not_establish_direction(self):
        self.assert_rejected(self.stages[1], synthetic_pose({1: 2.9}), self.baseline)
        for stage in self.stages[1:]:
            with self.subTest(kind=stage["kind"]):
                self.assert_rejected(stage, synthetic_pose({1: 8}))

    def test_return_pose_drift_is_rejected(self):
        self.assert_rejected(self.stages[-1], synthetic_pose({3: 3.1}), self.baseline)

    def test_identity_change_on_any_motor_invalidates_comparison(self):
        pose = synthetic_pose({1: 8})
        pose["motors"]["12"]["mcu_uid_hex"] = "f" * 16
        with self.assertRaisesRegex(ValueError, "ID 12"):
            manual.analyse(self.stages[1], pose, self.baseline)


class CandidateTests(unittest.TestCase):
    def test_both_signs_and_offsets_follow_documented_model_convention(self):
        stages = manual.make_stages()
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                records = complete_records(stages, sign)
                candidates = manual.build_candidates(stages, records)
                self.assertEqual(len(candidates), 12)
                for candidate in candidates:
                    for flag in ("approved_for_runtime", "zero_verified", "sign_verified",
                                 "measured_model_angles"):
                        self.assertIs(candidate[flag], False)
                    mid = candidate["motor_id"]
                    if mid == 11:
                        self.assertIsNone(candidate["sign_candidate"])
                        self.assertIsNone(candidate["offset_candidate_rad"])
                        self.assertEqual(candidate["status"], "KNOWN_POSITION_JUMP_REQUIRES_REVIEW")
                        continue
                    self.assertEqual(candidate["sign_candidate"], sign)
                    self.assertEqual(candidate["status"], "MANUAL_NOMINAL_CANDIDATE_REVIEW_REQUIRED")
                    baseline_angle = math.radians(-90 if candidate["joint"] == "calf" else 0)
                    # Existing convention: q_model = sign * raw + offset, not sign * (raw - offset).
                    recovered = sign * (mid / 10) + candidate["offset_candidate_rad"]
                    self.assertAlmostEqual(recovered, baseline_angle)
                    self.assertAlmostEqual(abs(candidate["observed_delta_deg"]), 8)
                    prefix = f"/synthetic/observations/{candidate['leg'].lower()}-"
                    self.assertEqual(candidate["evidence_paths"],
                                     {"baseline": prefix + "l", "direction": prefix + candidate["joint"],
                                      "return": prefix + "return"})

    def test_return_record_is_required_and_failed_repeatability_blocks_every_axis(self):
        stages = manual.make_stages(["FR"])
        complete = complete_records(stages)
        bad_return = observation(stages[-1], synthetic_pose({2: 5}), synthetic_pose())
        for records in (complete[:-1], complete[:-1] + [bad_return]):
            with self.subTest(return_present=len(records) == 5):
                for candidate in manual.build_candidates(stages, records):
                    self.assertIsNone(candidate["sign_candidate"])
                    self.assertIsNone(candidate["offset_candidate_rad"])
                    self.assertEqual(candidate["status"], "INSUFFICIENT_OBSERVATIONS")

    def test_quality_rejection_blocks_candidate_even_if_record_marked_accepted(self):
        stages = manual.make_stages(["FR"])
        for rejected_index in (0, 1, 4):
            with self.subTest(rejected_index=rejected_index):
                records = complete_records(stages)
                records[rejected_index]["quality"]["status"] = "RETRY_RECOMMENDED"
                calf = manual.build_candidates(stages, records)[0]
                self.assertIsNone(calf["sign_candidate"])


class FakeSession:
    def __init__(self, stages, captures=None):
        self.stages = stages
        self.path = Path("/synthetic/manual-observations")
        self.data = {"records": []}
        self.finished = []
        self.captured = []
        self.captures = iter(captures) if captures is not None else None

    def capture(self, stage):
        self.captured.append(stage["id"])
        record = next(self.captures) if self.captures is not None else complete_records([stage])[0]
        if isinstance(record, BaseException):
            raise record
        self.data["records"].append(record)
        return record

    def finish(self, status):
        self.finished.append(status)


class InteractiveTests(unittest.TestCase):
    def test_one_fresh_empty_enter_per_pose_captures_then_advances(self):
        stages = manual.make_stages(["FR"])
        session = FakeSession(stages)
        commands = Mock(side_effect=[""] * 5)
        output = []
        self.assertEqual(manual.run_interactive(session, read=commands, write=output.append), 0)
        self.assertEqual(session.captured, [stage["id"] for stage in stages])
        self.assertEqual(commands.call_count, 5)
        self.assertEqual(session.finished, ["OBSERVATIONS_COMPLETE"])
        self.assertIn("\n観測の記録が揃いました（校正候補・実機駆動は未承認）", output)
        self.assertFalse(any("校正は未完了:" in line for line in output))
        self.assertFalse(any("原点・方向の候補は保存済み" in line for line in output))

    def test_skips_are_partial_and_do_not_capture(self):
        stages = manual.make_stages(["FR"])
        session = FakeSession(stages)
        output = []
        self.assertEqual(manual.run_interactive(session, read=Mock(side_effect=["s"] * 5),
                                               write=output.append), 0)
        self.assertEqual(session.captured, [])
        self.assertEqual(session.finished, ["OBSERVATIONS_PARTIAL"])
        self.assertIn("\n校正は未完了: 0/5姿勢の有効な観測を記録しました。", output)
        missing = [line for line in output if line.startswith("  未完了:")]
        self.assertEqual(len(missing), 5)
        self.assertTrue(all("右前脚" in line and "ID " in line for line in missing))
        self.assertFalse(any("観測の記録が揃いました" in line for line in output))

    def test_partial_rl_summary_identifies_missing_thigh_and_return_observations(self):
        stages = manual.make_stages(["RL"])
        session = FakeSession(stages)
        output = []
        self.assertEqual(manual.run_interactive(session, read=Mock(side_effect=["", "", "s", "", "s"]),
                                               write=output.append), 0)
        self.assertEqual(session.finished, ["OBSERVATIONS_PARTIAL"])
        self.assertIn("\n校正は未完了: 3/5姿勢の有効な観測を記録しました。", output)
        self.assertEqual([line for line in output if line.startswith("  未完了:")], [
            "  未完了: 左後脚 / 上脚を少し前へ / ID 11",
            "  未完了: 左後脚 / L字に戻して照合 / ID 10, 11, 12",
        ])
        self.assertIn("原点・方向は未確定、実機駆動は未承認です。保存済みの観測を確認してください。", output)
        self.assertFalse(any("観測の記録が揃いました" in line or "原点・方向の候補は保存済み" in line
                             for line in output))

    def test_quit_eof_and_keyboard_interrupt_preserve_finished_status_without_capture(self):
        for command, code in (("q", 0), (EOFError(), 130), (KeyboardInterrupt(), 130)):
            with self.subTest(command=repr(command)):
                session = FakeSession(manual.make_stages(["FR"]))
                self.assertEqual(manual.run_interactive(session, read=Mock(side_effect=[command]),
                                                       write=lambda _: None), code)
                self.assertEqual(session.captured, [])
                self.assertEqual(session.finished, ["INTERRUPTED"])

    def test_unrecognized_input_does_not_capture_or_advance(self):
        session = FakeSession(manual.make_stages(["FR"]))
        commands = Mock(side_effect=["yes", "next", "", "q"])
        self.assertEqual(manual.run_interactive(session, read=commands, write=lambda _: None), 0)
        self.assertEqual(session.captured, ["fr-l"])
        self.assertEqual(commands.call_count, 4)

    def test_rejected_capture_retries_same_stage_and_keeps_both_observations(self):
        stage = manual.make_stages(["FR"])[0]
        unstable = synthetic_pose()
        unstable["motors"]["1"]["position"]["peak_to_peak_rad"] = .04
        session = FakeSession([stage], [observation(stage, unstable), observation(stage)])
        self.assertEqual(manual.run_interactive(session, read=Mock(side_effect=["", ""]),
                                               write=lambda _: None), 0)
        self.assertEqual(session.captured, ["fr-l", "fr-l"])
        self.assertEqual([r["accepted"] for r in session.data["records"]], [False, True])
        self.assertEqual(session.finished, ["OBSERVATIONS_COMPLETE"])

    def test_capture_exception_terminates_before_next_pose(self):
        session = FakeSession(manual.make_stages(["FR"]), [RuntimeError("read failed")])
        read = Mock(return_value="")
        output = []
        self.assertEqual(manual.run_interactive(session, read=read, write=output.append), 1)
        self.assertEqual(session.captured, ["fr-l"])
        self.assertEqual(read.call_count, 1)
        self.assertEqual(session.finished, ["FAILED"])
        self.assertTrue(any("read failed" in line for line in output))

    def test_terminal_discards_buffered_input_before_prompting_for_fresh_enter(self):
        events = []
        with patch.object(manual.sys.stdin, "fileno", return_value=17), \
                patch.object(manual.termios, "tcflush", side_effect=lambda *args: events.append(("flush", args))), \
                patch("builtins.input", side_effect=lambda prompt: events.append(("prompt", prompt)) or " S "):
            self.assertEqual(manual.tty_input("ready? "), "s")
        self.assertEqual(events, [("flush", (17, manual.termios.TCIFLUSH)), ("prompt", "ready? ")])


class SessionAndCliTests(unittest.TestCase):
    def test_dry_run_opens_no_hardware_and_creates_no_session_or_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "not-created"
            with patch.object(manual, "Session") as session, \
                    patch.object(manual.joint_snapshot, "main") as snapshot, \
                    patch("singularitydog_hw.joint_snapshot.ReadOnlyCAN") as can, \
                    contextlib.redirect_stdout(io.StringIO()) as printed:
                self.assertEqual(manual.main(["--output", str(output)]), 0)
            session.assert_not_called()
            snapshot.assert_not_called()
            can.assert_not_called()
            self.assertFalse(output.exists())
            self.assertIn("[20/20]", printed.getvalue())

    def test_failed_read_is_saved_and_never_processed_as_a_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = manual.Session(Path(tmp) / "session", manual.make_stages(["FR"]),
                                     get_boot=lambda: "synthetic-boot")
            with patch.object(manual.joint_snapshot, "main", return_value=1) as snapshot, \
                    patch.object(manual, "save_record") as save_pose:
                with self.assertRaises(RuntimeError):
                    session.capture(session.stages[0])
            snapshot.assert_called_once()
            save_pose.assert_not_called()
            data = json.loads((session.path / "session.json").read_text())
            self.assertEqual(data["records"][0]["status"], "FAILED")
            self.assertFalse(data["records"][0]["accepted"])
            self.assertFalse(session.baselines)
            self.assertTrue(all(candidate["sign_candidate"] is None for candidate in data["candidates"]))

    def test_reboot_prevents_any_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            boot = Mock(side_effect=["before", "after"])
            session = manual.Session(Path(tmp) / "session", manual.make_stages(["FR"]), get_boot=boot)
            with patch.object(manual.joint_snapshot, "main") as snapshot:
                with self.assertRaisesRegex(RuntimeError, "再起動"):
                    session.capture(session.stages[0])
            snapshot.assert_not_called()
            self.assertEqual(session.data["records"], [])

    def test_changed_identity_between_leg_baselines_aborts_and_retains_first_record(self):
        stages = manual.make_stages(["FR", "FL"])
        first, changed = synthetic_pose(), synthetic_pose()
        changed["motors"]["12"]["mcu_uid_hex"] = "f" * 16
        with tempfile.TemporaryDirectory() as tmp:
            session = manual.Session(Path(tmp) / "session", stages, get_boot=lambda: "synthetic-boot")
            with patch.object(manual.joint_snapshot, "main", return_value=0) as snapshot, \
                    patch.object(manual, "save_record", side_effect=[first, changed]):
                self.assertTrue(session.capture(stages[0])["accepted"])
                with self.assertRaisesRegex(RuntimeError, "個体"):
                    session.capture(stages[5])
            self.assertEqual(snapshot.call_count, 2)
            data = json.loads((session.path / "session.json").read_text())
            self.assertEqual([record["status"] for record in data["records"]], ["RECORDED", "FAILED"])
            self.assertEqual(data["identities"]["12"], first["motors"]["12"]["mcu_uid_hex"])
            self.assertEqual(set(session.baselines), {"FR"})


if __name__ == "__main__":
    unittest.main()
