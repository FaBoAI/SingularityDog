"""Real CPU tensor reuse preserves observer guards, state and owned evidence."""
import copy
import importlib.util
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer as observer
from test_policy_observer import calibration, mount, snapshot, make

if importlib.util.find_spec("torch") is not None:
    import torch
else:
    torch = None


class CpuPolicy:
    """Stateful CPU peer whose state and telemetry own their tensor storage."""
    def __init__(self, bad=None):
        self.bad = bad
        self.calls = []
        self.resets = 0
        self.previous = torch.full((1, 12), 99., dtype=torch.float32)
        self.tail = torch.tensor([[0.]*9+[0., 1., 0.]+[0., 0., 0., 0., 1.]], dtype=torch.float32)

    def reset(self, ids):
        assert ids.tolist() == [0]
        self.previous.zero_()
        self.resets += 1

    def __call__(self, *tensors):
        self.calls.append(tuple(value.clone() for value in tensors))
        gyro, gravity, command, q, dq, h = tensors
        self.last_observation = torch.cat(
            (gyro*.25, gravity, command, q, dq*.05, self.previous, h, self.tail), dim=1)
        self.last_actor_output = (self.previous*.5+q*.01+dq*.02+h*.03
                                  +(gyro.sum()+gravity.sum()+command.sum())*.001)
        self.previous = self.last_actor_output.clone()
        target = q+.002*torch.tanh(self.last_actor_output)
        if self.bad == "actor_nan":
            self.last_actor_output[0, 0] = float("nan")
        elif self.bad == "observation_inf":
            self.last_observation[0, 73] = float("inf")
        elif self.bad == "target_nan":
            target[0, 11] = float("nan")
        elif self.bad == "target_shape":
            target = target[:, :11]
        elif self.bad == "target_bounds":
            target[0, 0] = 2.
        return target


def cpu_observer(reuse, *, h=0, policy=None, profile=False, clock=None):
    return observer.StatefulPolicyObserver(
        policy or CpuPolicy(), calibration(), imu_mount_candidate=mount(),
        h_hypothesis=h, command=[.12, 0., 0.], max_ticks=3,
        max_age_ns=10_000_000, max_spread_ns=5_000_000,
        torch_module=torch, reuse_input_buffers=reuse,
        profile_consume=profile, monotonic_ns=clock)


class BufferOptionTests(unittest.TestCase):
    def test_reuse_is_explicit_and_requires_cpu_buffer_support(self):
        for value in (1, 0, None, "yes"):
            with self.subTest(value=value), self.assertRaisesRegex(observer.ObserverError, "must be boolean"):
                make(reuse_input_buffers=value)
        with self.assertRaisesRegex(observer.ObserverError, "requires torch.frombuffer"):
            make(reuse_input_buffers=True)
        self.assertIsNone(make()._input_buffers)


@unittest.skipIf(torch is None, "CPU PyTorch unavailable")
class ObserverBufferTests(unittest.TestCase):
    def assert_policy_equal(self, baseline, candidate):
        self.assertEqual(baseline.resets, candidate.resets)
        self.assertEqual(len(baseline.calls), len(candidate.calls))
        for before, after in zip(baseline.calls, candidate.calls):
            for expected, actual in zip(before, after):
                self.assertTrue(torch.equal(expected, actual))
        self.assertTrue(torch.equal(baseline.previous, candidate.previous))

    def test_full_records_float32_inputs_and_recurrent_state_match_both_hypotheses(self):
        for h in (0, 1):
            with self.subTest(h=h):
                baseline, candidate = cpu_observer(False, h=h), cpu_observer(True, h=h)
                for run in (baseline, candidate):
                    run.reset_run(1_000_000_000, warmup_completed=True)
                pointers = tuple(value.data_ptr() for value in candidate._input_tensors)
                buffers = tuple(id(value) for value in candidate._input_buffers)
                expected_records, actual_records = [], []
                for index in range(3):
                    source = snapshot(1_000_000_000+index*observer.DT_NS)
                    source["source_flags"] = {"sequence": index, "nested": [False, {"owned": True}]}
                    source["imu"]["gyro_rad_s"][0] += index*.017
                    next(row for row in source["motors"] if row["motor_id"] == 2
                         and row["parameter"] == "position")["value"] += index*.007
                    original = copy.deepcopy(source)
                    expected_records.append(baseline.consume(source))
                    actual_records.append(candidate.consume(source))
                    self.assertEqual(actual_records, expected_records)
                    self.assertEqual(source, original)
                    self.assertEqual(pointers, tuple(value.data_ptr() for value in candidate._input_tensors))
                    self.assertEqual(buffers, tuple(id(value) for value in candidate._input_buffers))
                    self.assert_policy_equal(baseline._policy, candidate._policy)
                    self.assertEqual(baseline._last_sources, candidate._last_sources)
                self.assertEqual(baseline.finish(), candidate.finish())

    def test_tick_allocates_no_new_input_tensors_and_profile_includes_buffer_fill(self):
        clocks = [iter(range(0, 1000, 10)) for _ in range(2)]
        baseline = cpu_observer(False, profile=True, clock=lambda: next(clocks[0]))
        candidate = cpu_observer(True, profile=True, clock=lambda: next(clocks[1]))
        for run in (baseline, candidate):
            run.reset_run(1_000_000_000, warmup_completed=True)
        expected = baseline.consume(snapshot())
        with patch.object(torch, "tensor", side_effect=AssertionError("new input tensor")), \
                patch.object(torch, "frombuffer", side_effect=AssertionError("new buffer tensor")):
            actual = candidate.consume(snapshot())
        self.assertEqual(actual, expected)
        timings = actual["consume_profile"]
        self.assertEqual(timings["durations_ns"]["tensor_conversion"], 10)
        self.assertEqual(timings["measured_total_ns"], sum(timings["durations_ns"].values()))
        self.assertEqual(timings["measured_total_ns"], 80)

    def test_observers_own_distinct_buffers_and_reset_reuses_storage(self):
        first, second = cpu_observer(True, h=0), cpu_observer(True, h=1)
        first_pointers = tuple(value.data_ptr() for value in first._input_tensors)
        self.assertFalse(set(first_pointers) & {value.data_ptr() for value in second._input_tensors})
        for run in (first, second):
            run.reset_run(1_000_000_000, warmup_completed=True)
        first.consume(snapshot())
        first_values = tuple(value.clone() for value in first._input_tensors)
        second.consume(snapshot())
        self.assertTrue(all(torch.equal(expected, actual)
                            for expected, actual in zip(first_values, first._input_tensors)))
        first.invalidate("explicit file-only reset")
        first.reset_run(2_000_000_000, warmup_completed=True)
        self.assertEqual(first_pointers, tuple(value.data_ptr() for value in first._input_tensors))
        self.assertEqual(first.reset_count, 2)
        self.assertTrue(torch.equal(first._policy.previous, torch.zeros((1, 12))))

    def test_records_and_source_do_not_alias_reused_model_storage(self):
        baseline, candidate = cpu_observer(False), cpu_observer(True)
        for run in (baseline, candidate):
            run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot()
        source["source_flags"] = {"nested": ["owned"]}
        first = candidate.consume(source)
        expected = baseline.consume(source)
        self.assertEqual(first, expected)
        frozen_first = copy.deepcopy(first)
        source["imu"]["gyro_rad_s"][0] = 999.
        source["source_flags"]["nested"].append("changed")
        second_source = snapshot(1_020_000_000)
        self.assertEqual(candidate.consume(second_source), baseline.consume(second_source))
        self.assertEqual(first, frozen_first)
        first["inputs"]["command"][0] = 999.
        first["inputs"]["dq_model_rad_s"][0] = 999.
        first["observation74"].clear()
        first["actor_residual12"].clear()
        first["q_target_rad_diagnostic_only"].clear()
        first["provenance"]["imu_mount_candidate"]["R_body_from_sensor"][0][0] = 9
        self.assertEqual(candidate.consume(snapshot(1_040_000_000)), baseline.consume(snapshot(1_040_000_000)))
        self.assert_policy_equal(baseline._policy, candidate._policy)

    def test_invalid_sources_and_order_fail_before_forward_in_both_paths(self):
        def duplicate(source):
            source["motors"][1] = copy.deepcopy(source["motors"][0])
        changes = (
            lambda s: s.update(output_allowed=True),
            lambda s: s["imu"]["gyro_rad_s"].__setitem__(0, float("nan")),
            lambda s: s["motors"][0].update(value=999.),
            lambda s: s["motors"][0].update(age_upper_bound_ns=0),
            lambda s: s["motors"][0].update(received_ns=s["tick_ns"]+1),
            lambda s: s["motors"].pop(), duplicate,
            lambda s: s.update(tick_ns=s["tick_ns"]+1),
        )
        for index, change in enumerate(changes):
            with self.subTest(change=index):
                messages = []
                for reuse in (False, True):
                    run = cpu_observer(reuse)
                    run.reset_run(1_000_000_000, warmup_completed=True)
                    source = snapshot()
                    change(source)
                    with self.assertRaises(observer.ObserverError) as caught:
                        run.consume(source)
                    messages.append(str(caught.exception))
                    self.assertEqual(run._policy.calls, [])
                    self.assertEqual(run.ticks_completed, 0)
                    self.assertEqual(run.status, "INCOMPLETE")
                    with self.assertRaisesRegex(observer.ObserverError, "inactive or invalid"):
                        run.consume(snapshot())
                self.assertEqual(*messages)

    def test_nonfinite_invalid_model_results_keep_identical_failure_and_state(self):
        for bad in ("actor_nan", "observation_inf", "target_nan", "target_shape", "target_bounds"):
            with self.subTest(bad=bad):
                runs = [cpu_observer(reuse, policy=CpuPolicy(bad)) for reuse in (False, True)]
                messages = []
                for run in runs:
                    run.reset_run(1_000_000_000, warmup_completed=True)
                    with self.assertRaises(observer.ObserverError) as caught:
                        run.consume(snapshot())
                    messages.append(str(caught.exception))
                    self.assertEqual(run.ticks_completed, 0)
                    self.assertEqual(run.status, "INCOMPLETE")
                self.assertEqual(*messages)
                self.assert_policy_equal(runs[0]._policy, runs[1]._policy)


if __name__ == "__main__":
    unittest.main()
