"""Causal offline replay with synthetic captures and independent fake policies."""
import contextlib
import argparse
from array import array
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer_replay as replay
from singularitydog_hw import policy_shadow as shadow
from test_policy_observer import Policy, FakeTorch, calibration, mount, snapshot, bias_candidate
from test_policy_shadow import capture_fixture, write_capture


def records(ticks=4):
    c = calibration()
    result = [{"kind": "motor_parameter", "motor_id": i, "parameter": "identity", "ok": True,
               "mcu_uid_hex": c["identities"][str(i)], "monotonic_ns": 900_000_000+i}
              for i in range(1, 13)]
    for n in range(ticks):
        s = snapshot(1_000_000_000+n*20_000_000)
        for row in s["motors"]:
            result.append({"kind": "motor_parameter", "motor_id": row["motor_id"],
                "parameter": row["parameter"], "value": row["value"], "ok": True, "status": 0,
                "unit": shadow.PARAMETERS[row["parameter"]][2],
                "request_monotonic_ns": row["request_ns"], "monotonic_ns": row["received_ns"]})
        im = s["imu"]
        result.append({"kind": "imu", "frame": "sensor", "accel_m_s2": im["accel_m_s2"],
                       "gyro_rad_s": im["gyro_rad_s"], "read_started_monotonic_ns": im["read_started_ns"],
                       "read_finished_monotonic_ns": im["read_finished_ns"],
                       "monotonic_ns": (im["read_started_ns"]+im["read_finished_ns"])//2})
    return result


class ReplayTests(unittest.TestCase):
    def test_optional_warmup_primes_the_same_reused_input_storage(self):
        import torch
        seen = []
        class Model:
            def __call__(self, *inputs):
                seen.append(tuple(t.data_ptr() for t in inputs))
                self.last_actor_output = torch.zeros(1, 12)
                self.last_observation = torch.zeros(1, 74)
                return torch.zeros(1, 12)
        buffers = tuple(array('f', [42.]*n) for n in (3, 3, 3, 12, 12, 12))
        inputs = tuple(torch.frombuffer(buf, dtype=torch.float32).reshape(1, len(buf))
                       for buf in buffers)
        model = Model()
        replay.warmup_policy(model, torch, 1, 3, input_tensors=inputs)
        self.assertEqual(len(seen), 3)
        self.assertTrue(all(ptrs == seen[0] for ptrs in seen))
        self.assertEqual(seen[0], tuple(t.data_ptr() for t in inputs))
        for actual, expected in zip(buffers[3], [0., .4, -.8]*4):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(list(buffers[5]), [1.]*12)
        with self.assertRaisesRegex(ValueError, 'six float32'):
            replay.warmup_policy(model, torch, 1, 1, input_tensors=inputs[:-1])

    def test_reused_input_prime_cannot_replace_first_fresh_observation(self):
        import torch
        model=Policy()
        run=replay.observer.StatefulPolicyObserver(
            model,calibration(),imu_mount_candidate=mount(),h_hypothesis=0,
            command=[0.,0.,0.],max_ticks=1,max_age_ns=10_000_000,
            max_spread_ns=5_000_000,torch_module=torch,reuse_input_buffers=True)
        replay.warmup_policy(model,torch,0,3,input_tensors=run._input_tensors)
        run.prepare_run(warmup_completed=True)
        run.arm_run(1_000_000_000)
        observed=run.consume(snapshot())
        self.assertEqual(run.reset_count,1)
        self.assertEqual(run.ticks_completed,1)
        self.assertEqual(len(model.calls),4)
        for actual,expected in zip(model.calls[-1][0],[.22,.11,-.33]):
            self.assertAlmostEqual(actual,expected)
        for actual,expected in zip(model.calls[-1][4],[.02*(i+1) for i in range(12)]):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(observed['tick_ns'],1_000_000_000)
        self.assertGreater(observed['provenance']['oldest_observation_age_ns'],0)

    def run_replay(self, data=None, policies=None, **kw):
        emitted = []
        p = policies or [Policy(), Policy()]
        options = dict(imu_mount_candidate=mount(), max_ticks=3, max_age_ns=10_000_000,
                       max_spread_ns=5_000_000, torch_module=FakeTorch, warmup_ticks=2, emit=emitted.append)
        options.update(kw)
        result = replay.replay_records(records() if data is None else data, calibration(), p, **options)
        return result, emitted, p

    def test_preserves_timestamps_and_warmup_then_exactly_one_reset_each(self):
        data = records()
        before = copy.deepcopy(data)
        result, emitted, policies = self.run_replay(data)
        self.assertEqual(data, before)
        self.assertEqual(result["status"], "COMPLETE_NO_OUTPUT_DIAGNOSTIC")
        self.assertEqual(result["timeline"]["first_tick_ns"], 999_000_000)
        for h, p in enumerate(policies):
            self.assertEqual(p.resets, 1)
            self.assertEqual(len(p.calls), 5)
            rows = [r for r in emitted if r["h_hypothesis"] == h]
            self.assertEqual([r["tick_ns"] for r in rows], [999_000_000, 1_019_000_000, 1_039_000_000])
            self.assertEqual(rows[0]["observation74"][33:45], [0.]*12)
            self.assertEqual(rows[1]["observation74"][33:45], rows[0]["actor_residual12"])
            self.assertEqual(rows[0]["inputs"]["h_hypothesis12"], [float(h)]*12)
            self.assertEqual(rows[0]["provenance"]["oldest_observation_age_ns"], 2_000_000)
        self.assertFalse(result["live_50hz_verified"])

    def test_future_imu_is_unavailable_until_read_finishes(self):
        data = records()
        imu_rows = [r for r in data if r["kind"] == "imu"]
        imu_rows[1].update(monotonic_ns=1_018_500_000, read_finished_monotonic_ns=1_020_000_000,
                           gyro_rad_s=[9., 9., 9.])
        result, emitted, _ = self.run_replay(data, max_age_ns=30_000_000, max_spread_ns=30_000_000)
        h0 = [r for r in emitted if r["h_hypothesis"] == 0]
        self.assertEqual(h0[1]["provenance"]["raw_gyro_rad_s"], [.11, .22, .33])
        self.assertEqual(result["status"], "COMPLETE_NO_OUTPUT_DIAGNOSTIC")

    def test_missing_stale_and_spread_failures_report_first_tick_no_fill(self):
        data = records()
        missing = [r for r in data if not (r.get("motor_id") == 7 and r.get("parameter") == "velocity")]
        result, emitted, policies = self.run_replay(missing)
        self.assertIn("missing:ID7.velocity", result["hypotheses"][0]["failure"])
        self.assertIsNone(next(r for r in emitted[0]["snapshot"]["motors"]
                               if r["motor_id"] == 7 and r["parameter"] == "velocity")["value"])
        self.assertEqual(result["hypotheses"][0]["ticks_completed"], 0)
        self.assertEqual(len(policies[0].calls), 2)  # Synthetic warmup only.
        late_refresh = [r for r in data if not (r.get("motor_id") == 7 and r.get("parameter") == "velocity"
                                               and r["monotonic_ns"] == 1_019_000_000)]
        result, _, _ = self.run_replay(late_refresh)
        failure = result["hypotheses"][0]["first_blocked_tick"]
        self.assertEqual(failure["tick_index"], 1)
        self.assertIn("stale:ID7.velocity", failure["reason"])
        self.assertIn("acquisition_spread_exceeded", failure["reason"])

    def test_model_range_violation_is_preserved_not_clipped(self):
        data = records()
        next(r for r in data if r.get("motor_id") == 6 and r.get("parameter") == "position")["value"] = 20.
        result, emitted, policies = self.run_replay(data)
        self.assertIn("outside registered joint range", result["hypotheses"][0]["failure"])
        self.assertEqual(result["hypotheses"][0]["ticks_completed"], 0)
        self.assertEqual(len(policies[0].calls), 2)
        self.assertEqual(next(r for r in emitted[0]["snapshot"]["motors"]
                              if r["motor_id"] == 6 and r["parameter"] == "position")["value"], 20.)

    def test_missing_wrong_duplicate_and_future_identities_are_rejected(self):
        for change in ("missing", "wrong", "duplicate", "future"):
            data = records()
            if change == "missing": del data[0]
            elif change == "wrong": data[0]["mcu_uid_hex"] = "f"*16
            elif change == "duplicate": data.insert(0, copy.deepcopy(data[0]))
            else: data[0]["monotonic_ns"] = 2_000_000_000
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "identity|identities"):
                self.run_replay(data)

    def test_timestamp_fields_are_required_and_sorting_cannot_hide_reversal(self):
        for change in ("missing_start", "reordered", "nonfinite", "corrected"):
            data = records()
            first = next(r for r in data if r.get("parameter") == "position")
            if change == "missing_start": del first["request_monotonic_ns"]
            elif change == "nonfinite": first["value"] = float("nan")
            elif change == "corrected": next(r for r in data if r["kind"] == "imu")["gyro_bias_subtracted"] = True
            else:
                last = next(r for r in data if r.get("parameter") == "position" and r["monotonic_ns"] > 999_000_000)
                last["request_monotonic_ns"] = first["request_monotonic_ns"]
            with self.subTest(change=change), self.assertRaises(ValueError): self.run_replay(data)

    def test_capture_end_and_shared_policy_cannot_produce_extra_ticks(self):
        result, _, _ = self.run_replay(records(1))
        self.assertEqual(result["hypotheses"][0]["ticks_completed"], 1)
        self.assertEqual(result["hypotheses"][0]["first_blocked_tick"]["reason"], "capture_exhausted")
        p = Policy()
        with self.assertRaisesRegex(ValueError, "independent"):
            self.run_replay(policies=[p, p])

    def test_optional_bias_is_only_a_hypothesis_and_invalid_warmup_is_reported(self):
        result, emitted, _ = self.run_replay(gyro_bias_candidate=bias_candidate())
        self.assertEqual(emitted[0]["inputs"]["gyro_body_rad_s"], [.2, .1, -.30000000000000004])
        self.assertFalse(result["approved_for_runtime"])
        result, _, _ = self.run_replay(policies=[Policy("actor_nan"), Policy()])
        self.assertEqual(result["hypotheses"][0]["first_blocked_tick"]["phase"], "warmup_or_reset")
        self.assertEqual(result["hypotheses"][0]["reset_count"], 0)
        self.assertEqual(result["hypotheses"][1]["ticks_completed"], 3)

    def test_cli_uses_complete_capture_only_and_new_private_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            capture = root/"capture"
            capture.mkdir()
            summary, events = capture_fixture(capture)
            c, m = root/"calibration.json", root/"mount.json"
            c.write_text(json.dumps(calibration()))
            m.write_text(json.dumps(mount()))
            output = root/"output"
            argv = ["--capture", str(capture), "--calibration", str(c), "--imu-mount-candidate", str(m),
                    "--bundle", str(root), "--output", str(output), "--max-ticks", "3",
                    "--max-age-ms", "100", "--max-spread-ms", "100"]
            with patch.object(shadow, "load_policy", side_effect=lambda p: (Policy(), {"test": True})), \
                 patch.dict("sys.modules", {"torch": FakeTorch}), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(replay.main(argv), 2)  # Real fixture violates a model joint range.
            self.assertEqual(output.stat().st_mode & 0o777, 0o700)
            self.assertEqual({p.name for p in output.iterdir()}, {"events.jsonl", "summary.json"})
            report = json.loads((output/"summary.json").read_text())
            self.assertEqual(report["provenance"]["capture"]["source_status"], "COMPLETE")
            self.assertFalse(report["incomplete_capture_override_available"])
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): replay.main(argv)
            argv[argv.index(str(output))] = str(root/"incomplete-output")
            summary.update(status="INCOMPLETE", errors=[{"component": "can", "error": "TimeoutError('No fresh response: x')"}])
            write_capture(capture, summary, events)
            with patch.object(shadow, "load_policy") as loader, contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                replay.main(argv)
            loader.assert_not_called()
            rejected = json.loads((root/"incomplete-output"/"summary.json").read_text())
            self.assertEqual(rejected["status"], "INCOMPLETE")
            self.assertEqual(rejected["failure_phase"], "capture_validation")
            self.assertFalse(rejected["capture_validation_passed"])
            self.assertEqual((root/"incomplete-output"/"events.jsonl").read_bytes(), b"")

    def test_git_output_and_invalid_limit_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/".git").write_text("gitdir: elsewhere")
            with self.assertRaisesRegex(ValueError, "outside Git"): replay._output_path(root/"report")
        for text in ("nan", "inf", "-1", "0.0000001"):
            with self.subTest(text=text), self.assertRaises(argparse.ArgumentTypeError): replay._milliseconds(text)


if __name__ == "__main__":
    unittest.main()
