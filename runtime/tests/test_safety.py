"""Offline guard tests; no serial, CAN, I2C or motor access."""

from dataclasses import replace
import math
import unittest

from singularitydog_hw.safety import (
    DeadmanStatus, GuardConfig, GuardState, Prerequisites, SafetyGuard,
    SafetyInputs, SensorSample,
)


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class SafetyGuardTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        # Test timings only; these are not approved robot operating thresholds.
        self.config = GuardConfig({"imu": 0.2, "motors": 0.3}, 0.1)
        self.guard = SafetyGuard(self.config, clock=self.clock)

    def ready(self, now=None):
        now = self.clock.now if now is None else now
        return SafetyInputs(
            Prerequisites(True, True, True, True),
            {"imu": SensorSample(now, (0.0, 0.0, 9.8), True),
             "motors": SensorSample(now, (0.0, 0.0), True)},
            DeadmanStatus(now, True),
        )

    def test_startup_default_deny_and_no_automatic_arm(self):
        self.assertEqual(self.guard.state, GuardState.DISARMED)
        self.assertFalse(self.guard.evaluate(SafetyInputs()).permitted)
        self.assertFalse(self.guard.arm(SafetyInputs()).permitted)
        self.assertFalse(self.guard.evaluate(self.ready()).permitted)
        self.assertTrue(self.guard.arm(self.ready()).permitted)

    def test_each_prerequisite_requires_literal_verified_true(self):
        for name in Prerequisites.__dataclass_fields__:
            for value in (False, None, 1, "true"):
                with self.subTest(name=name, value=value):
                    inputs = self.ready()
                    inputs = replace(inputs, prerequisites=replace(inputs.prerequisites, **{name: value}))
                    result = self.guard.arm(inputs)
                    self.assertFalse(result.permitted)
                    self.assertIn("unverified:" + name, result.reasons)

    def test_permission_has_earliest_input_expiry(self):
        result = self.guard.arm(self.ready())
        self.assertEqual(result.checked_at, 10.0)
        self.assertAlmostEqual(result.valid_until, 10.1)
        self.clock.now = 10.05
        self.assertTrue(self.guard.evaluate(self.ready(now=10.0)).permitted)

    def test_dropout_fault_stays_latched_until_disarm_clear_explicit_arm(self):
        self.guard.arm(self.ready())
        missing = replace(self.ready(), sensors={"imu": self.ready().sensors["imu"]})
        result = self.guard.evaluate(missing)
        self.assertEqual(result.state, GuardState.FAULT)
        self.assertTrue(result.stop_intent)
        fault = self.guard.fault_reasons
        self.assertFalse(self.guard.evaluate(self.ready()).permitted)
        self.assertFalse(self.guard.arm(self.ready()).permitted)
        self.assertIn("disarm_before_clear_fault", self.guard.clear_fault().reasons)
        self.guard.disarm()
        self.assertEqual(self.guard.state, GuardState.DISARMED)
        self.assertEqual(self.guard.fault_reasons, fault)
        self.assertFalse(self.guard.arm(self.ready()).permitted)
        self.assertFalse(self.guard.clear_fault().permitted)
        self.assertFalse(self.guard.evaluate(self.ready()).permitted)
        self.assertTrue(self.guard.arm(self.ready()).permitted)

    def test_stale_sensor_and_deadman_are_independently_detected(self):
        for source in ("imu", "motors", "deadman"):
            with self.subTest(source=source):
                guard = SafetyGuard(self.config, clock=self.clock)
                guard.arm(self.ready())
                inputs = self.ready()
                if source == "deadman":
                    inputs = replace(inputs, deadman=DeadmanStatus(9.0, True))
                else:
                    sensors = dict(inputs.sensors)
                    sensors[source] = replace(sensors[source], timestamp=9.0)
                    inputs = replace(inputs, sensors=sensors)
                result = guard.evaluate(inputs)
                self.assertEqual(result.state, GuardState.FAULT)
                self.assertIn(("deadman" if source == "deadman" else "sensor:" + source) + ":stale", result.reasons)

    def test_exact_freshness_boundary_is_expired(self):
        guard = SafetyGuard(GuardConfig({"imu": 1.0}, 1.0), clock=self.clock)
        inputs = replace(self.ready(), sensors={"imu": SensorSample(9.0, (1.0,), True)},
                         deadman=DeadmanStatus(9.0, True))
        self.assertFalse(guard.arm(inputs).permitted)

    def test_fractional_expiry_matches_returned_permission_deadline(self):
        inputs = self.ready()
        decision = self.guard.arm(inputs)
        self.clock.now = decision.valid_until
        result = self.guard.evaluate(inputs)
        self.assertEqual(result.state, GuardState.FAULT)
        self.assertIn("deadman:stale", result.reasons)

    def test_deadman_release_or_missing_latches(self):
        for deadman in (None, DeadmanStatus(10.0, False), DeadmanStatus(10.0, 1)):
            with self.subTest(deadman=deadman):
                guard = SafetyGuard(self.config, clock=self.clock)
                guard.arm(self.ready())
                self.assertEqual(guard.evaluate(replace(self.ready(), deadman=deadman)).state, GuardState.FAULT)

    def test_future_sensor_and_deadman_timestamps_latch(self):
        for source in ("imu", "deadman"):
            with self.subTest(source=source):
                guard = SafetyGuard(self.config, clock=self.clock)
                guard.arm(self.ready())
                inputs = self.ready()
                if source == "deadman":
                    inputs = replace(inputs, deadman=DeadmanStatus(10.01, True))
                else:
                    sensors = dict(inputs.sensors)
                    sensors[source] = replace(sensors[source], timestamp=10.01)
                    inputs = replace(inputs, sensors=sensors)
                result = guard.evaluate(inputs)
                self.assertEqual(result.state, GuardState.FAULT)
                self.assertTrue(any("future_timestamp" in reason for reason in result.reasons))

    def test_nonfinite_or_invalid_required_samples_fail_closed(self):
        cases = [SensorSample(10.0, values, True)
                 for values in ((), (math.nan,), (math.inf,), (-math.inf,), (True,), ("1",), None)]
        cases += [SensorSample(t, (1.0,), True) for t in (math.nan, math.inf, None, True)]
        cases += [SensorSample(10.0, (1.0,), flag) for flag in (False, 1, "true")]
        for sample in cases:
            with self.subTest(sample=sample):
                guard = SafetyGuard(self.config, clock=self.clock)
                guard.arm(self.ready())
                inputs = self.ready()
                result = guard.evaluate(replace(inputs, sensors={**inputs.sensors, "imu": sample}))
                self.assertFalse(result.permitted)
                self.assertEqual(result.state, GuardState.FAULT)

    def test_invalid_inputs_and_revoked_prerequisites_latch(self):
        for inputs in (None, SafetyInputs(sensors=None), replace(self.ready(), prerequisites=None),
                       replace(self.ready(), prerequisites=Prerequisites())):
            with self.subTest(inputs=inputs):
                guard = SafetyGuard(self.config, clock=self.clock)
                guard.arm(self.ready())
                self.assertEqual(guard.evaluate(inputs).state, GuardState.FAULT)

    def test_clock_backwards_fault_and_explicit_clock_epoch_recovery(self):
        self.guard.arm(self.ready())
        self.clock.now = 9.0
        result = self.guard.evaluate(self.ready())
        self.assertIn("clock_moved_backwards", result.reasons)
        self.assertFalse(self.guard.arm(self.ready()).permitted)
        self.guard.disarm()
        self.guard.clear_fault()
        self.assertFalse(self.guard.evaluate(self.ready()).permitted)
        self.assertTrue(self.guard.arm(self.ready()).permitted)

    def test_invalid_clock_fails_closed_and_does_not_block_disarm_or_close(self):
        for bad in (math.nan, math.inf, True, None, "10"):
            with self.subTest(clock=bad):
                guard = SafetyGuard(self.config, clock=self.clock)
                self.clock.now = 10.0
                guard.arm(self.ready())
                self.clock.now = bad
                self.assertEqual(guard.evaluate(self.ready(now=10.0)).state, GuardState.FAULT)
                self.assertFalse(guard.disarm().permitted)
                self.assertFalse(guard.clear_fault().permitted)
                self.assertTrue(guard.fault_reasons)
                self.assertEqual(guard.close().state, GuardState.CLOSED)
        self.clock.now = 10.0

    def test_clock_exception_latches(self):
        failed = False
        def failing_clock():
            if failed:
                raise OSError("clock unavailable")
            return self.clock.now
        guard = SafetyGuard(self.config, clock=failing_clock)
        self.assertTrue(guard.arm(self.ready()).permitted)
        failed = True
        result = guard.evaluate(self.ready())
        self.assertEqual(result.state, GuardState.FAULT)
        self.assertIn("clock_unavailable", result.reasons)
        failed = False
        self.assertFalse(guard.arm(self.ready()).permitted)
        self.assertFalse(guard.close().permitted)

    def test_disarm_and_terminal_close_revoke_permission(self):
        self.guard.arm(self.ready())
        result = self.guard.disarm()
        self.assertFalse(result.permitted)
        self.assertTrue(result.stop_intent)
        self.assertFalse(self.guard.evaluate(self.ready()).permitted)
        self.guard.arm(self.ready())
        result = self.guard.close()
        self.assertEqual(result.state, GuardState.CLOSED)
        self.assertTrue(result.stop_intent)
        for result in (self.guard.arm(self.ready()), self.guard.evaluate(self.ready()),
                       self.guard.clear_fault(), self.guard.disarm(), self.guard.close()):
            self.assertEqual(result.state, GuardState.CLOSED)
            self.assertFalse(result.permitted)

    def test_illegal_clear_while_armed_fails_closed(self):
        self.guard.arm(self.ready())
        result = self.guard.clear_fault()
        self.assertFalse(result.permitted)
        self.assertEqual(result.state, GuardState.FAULT)

    def test_stop_intent_is_an_event_not_a_claim_that_hardware_stopped(self):
        self.guard.arm(self.ready())
        self.guard.evaluate(SafetyInputs())
        self.guard.evaluate(self.ready())
        events = self.guard.drain_events()
        self.assertEqual([event.kind for event in events], ["armed", "fault_latched"])
        self.assertTrue(events[-1].stop_intent)
        self.assertEqual(self.guard.drain_events(), ())

    def test_invalid_configuration_is_rejected_and_source_mapping_copied(self):
        for ages in ({}, None, {"": 1}, {" imu": 1}, {1: 1}, {"imu": 0},
                     {"imu": -1}, {"imu": math.nan}, {"imu": math.inf}, {"imu": True}):
            with self.subTest(ages=ages), self.assertRaises(ValueError):
                GuardConfig(ages, 1.0)
        for age in (0, -1, math.nan, math.inf, True, None, "1"):
            with self.subTest(age=age), self.assertRaises(ValueError):
                GuardConfig({"imu": 1.0}, age)
        ages = {"imu": 0.2}
        config = GuardConfig(ages, 0.1)
        ages["imu"] = 999
        self.assertEqual(config.sensor_max_age_s["imu"], 0.2)
        with self.assertRaises(TypeError):
            config.sensor_max_age_s["imu"] = 999
        with self.assertRaises(ValueError):
            SafetyGuard(None)
        with self.assertRaises(ValueError):
            SafetyGuard(config, clock=None)


if __name__ == "__main__":
    unittest.main()
