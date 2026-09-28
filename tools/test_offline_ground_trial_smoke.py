"""Pure contracts of the local ground smoke. No model files or hardware needed."""
import unittest
from unittest.mock import patch

import offline_ground_trial_smoke as smoke


class OfflineGroundSmokeTests(unittest.TestCase):
    def test_fixtures_are_in_memory_bypass_not_physical_review(self):
        for stage in ("supported_stance", "walk"):
            profile, plan = smoke.synthetic_fixtures(stage)
            self.assertTrue(profile["simulation_only"])
            self.assertTrue(plan["physical_validation_bypassed_for_local_sockets"])
            self.assertFalse(plan["physical_review_generated"])
            self.assertNotIn("review", plan)
            self.assertNotIn("approved_for_ground_trial", plan)
            timeline = smoke.trajectory.GroundTimeline(**plan["trajectory"])
            self.assertEqual(timeline.timing["duration_s"], 4.)
            self.assertEqual(profile["hard_cycle_ms"], 100.)  # Not a hardware walk profile.

    def test_synthetic_operator_event_is_later_than_display_and_labeled(self):
        _, plan = smoke.synthetic_fixtures("walk")
        execution = smoke.SimulatedGroundExecution(plan, acknowledge=True)
        start = 1_000_000_000
        execution.on_start(start)
        with patch.object(smoke.time, "monotonic_ns", return_value=start + 2_110_000_000):
            execution.before_cycle(start + 2_100_000_000)
        self.assertIsNone(execution.ack_ns)
        execution.before_cycle(start + 2_120_000_000)
        self.assertIsNone(execution.ack_ns)
        execution.before_cycle(start + 2_140_000_000)
        self.assertEqual(execution.ack_ns, start + 2_140_000_000)
        injected = next(e for e in execution.events if e["key"] == "SIMULATED_OPERATOR_ACK_INJECTED")
        self.assertTrue(injected["simulation_only"])
        self.assertFalse(injected["physical_readiness"])

    def test_missing_ack_is_not_fabricated_and_emergency_does_not_wait(self):
        _, plan = smoke.synthetic_fixtures("walk")
        execution = smoke.SimulatedGroundExecution(plan, acknowledge=False)
        cancelled = []
        execution.connect_cancel(lambda: cancelled.append(True))
        execution.on_start(1_000_000_000)
        execution.before_cycle(3_100_000_000)
        self.assertIsNone(execution.ack_ns)
        with self.assertRaisesRegex(RuntimeError, "resupport_not_confirmed"):
            execution.before_cycle(4_660_000_000)
        self.assertEqual(cancelled, [True])
        self.assertIsNone(execution.timeline.resupport_ack_s)


if __name__ == "__main__": unittest.main()
