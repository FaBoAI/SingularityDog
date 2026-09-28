"""Optional 183-frame saved CPU-model parity; no devices, network or output.

Set SD_BUFFER_PARITY_REPORT, SD_BUFFER_PARITY_REPORT_SHA256,
SD_BUFFER_PARITY_MANIFEST, SD_BUFFER_PARITY_MANIFEST_SHA256 and
SD_BUFFER_PARITY_BUNDLE to pinned local artifacts. The report must contain a
fixed zero-command/h=0 sequence of saved six-vector model inputs. Its source
request/read times are absent: the harness labels synthetic source intervals
while retaining every original model-call timestamp and model input value.
Optionally pin SD_BUFFER_PARITY_OBSERVER_SOURCE and its _SHA256 to compare
the complete pre-copy-optimization observer source against both current paths.
"""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import unittest

from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.policy_observer_replay import warmup_policy
from singularitydog_hw.telemetry_snapshot import TelemetrySnapshotBuffer
from test_policy_observer import calibration, mount


ARTIFACTS = {key: os.environ.get("SD_BUFFER_PARITY_"+key) for key in
             ("REPORT", "REPORT_SHA256", "MANIFEST", "MANIFEST_SHA256", "BUNDLE")}


@unittest.skipUnless(all(ARTIFACTS.values()), "pinned saved CPU parity artifacts not selected")
class SavedObserverBufferParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        from native_policy_overnight import load_verified
        from native_policy_overnight.verification import compare_state, compare_tensor
        cls.torch, cls.load_verified = torch, staticmethod(load_verified)
        cls.compare_state, cls.compare_tensor = staticmethod(compare_state), staticmethod(compare_tensor)
        report_bytes = Path(ARTIFACTS["REPORT"]).read_bytes()
        if hashlib.sha256(report_bytes).hexdigest() != ARTIFACTS["REPORT_SHA256"]:
            raise ValueError("Saved model input report SHA256 differs")
        cls.rows = json.loads(report_bytes)["policy_calls"]
        if len(cls.rows) != 183 or any(row["inputs"][2] != [0., 0., 0.]
                                      or row["inputs"][5] != [0.]*12 for row in cls.rows):
            raise ValueError("Require the complete 183-frame fixed-command h=0 sequence")
        cls.baseline_observer = observer.StatefulPolicyObserver
        source = os.environ.get("SD_BUFFER_PARITY_OBSERVER_SOURCE")
        source_sha = os.environ.get("SD_BUFFER_PARITY_OBSERVER_SOURCE_SHA256")
        if source or source_sha:
            if not source or not source_sha or hashlib.sha256(Path(source).read_bytes()).hexdigest() != source_sha:
                raise ValueError("Pre-copy observer source SHA256 differs or is missing")
            spec = importlib.util.spec_from_file_location("singularitydog_hw._saved_pre_copy_observer", source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls.baseline_observer = module.StatefulPolicyObserver

    def model(self, h):
        model, _ = self.load_verified(
            ARTIFACTS["MANIFEST"], expected_manifest_sha256=ARTIFACTS["MANIFEST_SHA256"],
            bundle=ARTIFACTS["BUNDLE"])
        warmup_policy(model, self.torch, h, 10)
        return model

    def runs(self, h, *, max_ticks):
        cal = calibration()
        cal["identities"] = {str(mid): (bytes([mid])*8).hex() for mid in range(1, 13)}
        for row in cal["candidates"]:
            row.update(sign_candidate=1, offset_candidate_rad=0.)
        imu_mount = mount([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        paths = [(self.baseline_observer, False), (observer.StatefulPolicyObserver, True)]
        if self.baseline_observer is not observer.StatefulPolicyObserver:
            paths.insert(1, (observer.StatefulPolicyObserver, False))
        models = [self.model(h) for _ in paths]
        runs = [observer_class(
            model, cal, imu_mount_candidate=imu_mount, h_hypothesis=h,
            command=[0., 0., 0.], max_ticks=max_ticks, max_age_ns=10_000_000,
            max_spread_ns=5_000_000, torch_module=self.torch,
            measured_diagnostic_ticks=True, reuse_input_buffers=reuse,
            profile_consume=True, monotonic_ns=lambda clock=iter(range(100, 100_000, 10)): next(clock))
            for model, (observer_class, reuse) in zip(models, paths)]
        return models, runs

    def source(self, index):
        saved = self.rows[index]
        gyro, gravity, command, q, dq, _ = saved["inputs"]
        tick = saved["call_monotonic_ns"]
        buffer = TelemetrySnapshotBuffer(max_age_ns=10_000_000, max_spread_ns=5_000_000)
        for mid, position, velocity in zip(shadow.CAN_ORDER, q, dq):
            for parameter, value, unit in (("position", position, "rad"),
                                            ("velocity", velocity, "rad_s")):
                buffer.ingest_motor(can_type=17, motor_id=mid, parameter=parameter,
                                    value=value, unit=unit, request_ns=tick-3_000_000,
                                    received_ns=tick-1_000_000)
        buffer.ingest_imu(accel_m_s2=[-value*9.80665 for value in gravity], gyro_rad_s=gyro,
                          read_started_ns=tick-2_000_000, read_finished_ns=tick-1_500_000)
        source = buffer.snapshot(tick).as_dict()
        source["source_flags"] = {"file_only_buffer_parity": True,
            "saved_model_input_report_sha256": ARTIFACTS["REPORT_SHA256"],
            "harness_generated_source_intervals": True, "saved_frame_index": index,
            "original_model_call_monotonic_ns": tick, "hardware_opened": False}
        return source

    def run_parity(self, h):
        models, runs = self.runs(h, max_ticks=183)
        for run in runs:
            run.reset_run(self.rows[0]["call_monotonic_ns"], warmup_completed=True)
        records = [[] for _ in runs]
        maxima = {}
        for index, saved in enumerate(self.rows):
            gyro, gravity, command, q, dq, _ = saved["inputs"]
            source = self.source(index)
            original = copy.deepcopy(source)
            for records_for_run, run in zip(records, runs):
                records_for_run.append(run.consume(source))
            for candidate in records[1:]:
                self.assertEqual(records[0][-1], candidate[-1], "full record at saved frame "+str(index))
            self.assertEqual(source, original)
            actual = records[-1][-1]
            for name, values in zip(("gyro_body_rad_s", "gravity_body_unit", "command",
                                     "q_model_rad", "dq_model_rad_s", "h_hypothesis12"),
                                    (gyro, gravity, command, q, dq, [float(h)]*12)):
                self.assertEqual(actual["inputs"][name], values)
            with self.torch.inference_mode():
                for model in models[1:]:
                    self.compare_state(self.torch, models[0], model, maxima, exact=True)
                if h == 0:
                    target = self.torch.tensor([actual["q_target_rad_diagnostic_only"]], dtype=self.torch.float32)
                    expected = self.torch.tensor(
                        [[saved["target_can_order"][mid-1] for mid in shadow.CAN_ORDER]],
                        dtype=self.torch.float32)
                    self.compare_tensor(self.torch, target, expected, "saved_target", maxima, exact=True)
            for run in runs[1:]:
                self.assertEqual(runs[0]._last_sources, run._last_sources)
        # Compare all retained records after the final buffer reuse as well.
        summary = runs[0].finish()
        for record, run in zip(records[1:], runs[1:]):
            self.assertEqual(records[0], record)
            self.assertEqual(summary, run.finish())
            self.assertEqual(run.reset_count, 1)
            self.assertEqual(run.status, "COMPLETE_NO_OUTPUT_DIAGNOSTIC")
            self.assertEqual(run.ticks_completed, 183)

    def test_saved_183_h0_matches_records_targets_and_every_model_state(self):
        self.run_parity(0)

    def test_saved_183_h1_matches_records_and_every_model_state(self):
        self.run_parity(1)

    def test_saved_snapshot_rejections_keep_full_failure_profile_and_model_state(self):
        changes = (
            lambda source: source.update(output_allowed=True),
            lambda source: source.update(source_flags=[]),
            lambda source: source["motors"].pop(),
            lambda source: source["motors"][0].update(unit="degrees"),
            lambda source: source["motors"][0].update(value=999.),
            lambda source: source["motors"][0].update(age_upper_bound_ns=0),
            lambda source: source["imu"]["gyro_rad_s"].__setitem__(0, float("nan")),
            lambda source: source.update(tick_ns=self.rows[0]["call_monotonic_ns"]),
            lambda source: source["motors"][0].update(
                request_ns=self.rows[0]["call_monotonic_ns"]-3_000_000),
            lambda source: source["source_flags"].update(oversized="x"*90_000),
        )
        for h in (0, 1):
            models, runs = self.runs(h, max_ticks=183)
            for index, change in enumerate(changes):
                with self.subTest(h=h, change=index):
                    source = self.source(1)
                    change(source)
                    original = copy.deepcopy(source)
                    failures, summaries = [], []
                    for run in runs:
                        run.reset_run(self.rows[0]["call_monotonic_ns"], warmup_completed=True)
                        run.consume(self.source(0))
                        with self.assertRaises(ValueError) as caught:
                            run.consume(source)
                        failures.append((type(caught.exception).__name__, str(caught.exception)))
                        summaries.append(run.summary())
                        self.assertEqual(run.ticks_completed, 1)
                        self.assertEqual(run.status, "INCOMPLETE")
                    for failure, summary in zip(failures[1:], summaries[1:]):
                        self.assertEqual(failures[0], failure)
                        self.assertEqual(summaries[0], summary)
                    # NaN is intentionally rejected, so compare its serialized
                    # fixture rather than equality's nonreflexive float behavior.
                    self.assertEqual(json.dumps(source, sort_keys=True), json.dumps(original, sort_keys=True))
                    with self.torch.inference_mode():
                        for model in models[1:]:
                            self.compare_state(self.torch, models[0], model, {}, exact=True)


if __name__ == "__main__":
    unittest.main()
