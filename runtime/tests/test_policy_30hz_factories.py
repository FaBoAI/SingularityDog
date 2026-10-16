"""Real release through production observer factories; synthetic CAN only."""
from pathlib import Path
import math
import unittest

from experiments.four_bus_diagnostic import model_bridge as bridge
from experiments.four_bus_diagnostic import test_model_bridge as fixtures
from experiments.four_bus_type1 import type1_foreground as foreground
from singularitydog_hw import policy_model_30hz as models
from singularitydog_hw.policy_clock import control_clock_payload
from singularitydog_hw.policy_observer import StatefulPolicyObserver

try:
    import torch
except ImportError:
    torch = None

BUNDLE = Path(__file__).resolve().parents[3] / 'RS05_30Hz_Model_20261013'


@unittest.skipUnless(torch is not None and BUNDLE.exists(), 'Pinned CPU release required')
class ActualFactory30Tests(unittest.TestCase):
    def setUp(self):
        self.artifacts = models.bundle_artifacts(BUNDLE)
        self.plan = bridge.reclock_model_plan(fixtures.Fixture().plan(),
            control_clock_payload(30), models.plan(self.artifacts))
        self.policy, self.proof = models.load(self.artifacts, execute_cpu=True)
        # Warm the selected controller before the observer's one ordinary reset.
        with torch.inference_mode():
            args = (torch.zeros(1, 3), torch.tensor([[0., 0., -1.]]), torch.zeros(1, 3),
                    self.policy.controller.nominal.float().reshape(1, 12).clone(),
                    torch.zeros(1, 12), torch.zeros(1, 12))
            for _ in range(20):
                self.policy(*args)

    def exercise(self, run):
        records, sample, tick = fixtures.records(self.plan)
        snapshot = bridge.snapshot_from_four_records(self.plan, records, sample, tick)
        self.assertEqual(snapshot['max_age_ns'], 20_000_000)
        self.assertEqual(snapshot['max_spread_ns'], 20_000_000)
        run.prepare_run(warmup_completed=True)
        self.assertEqual(float(self.policy.controller.elapsed[0]), 0.)
        run.arm_run(tick)
        result = run.consume(snapshot)
        self.assertEqual(result['dt_s'], 1/30)
        self.assertEqual(result['policy_hz'], 30)
        self.assertFalse(result['output_allowed'])
        self.assertEqual(run.ticks_completed, 1)
        self.assertAlmostEqual(float(self.policy.controller.elapsed[0]), 1/30, places=14)
        self.assertTrue(torch.isfinite(self.policy.last_actor_output).all().item())
        self.assertEqual(len(result['q_target_rad_diagnostic_only']), 12)
        self.assertTrue(all(math.isfinite(value)
                            for value in result['q_target_rad_diagnostic_only']))
        self.assertFalse(self.proof['hardware_opened'])

    def test_diagnostic_production_factory_uses_real_model_without_legacy_wrapper(self):
        run = bridge.create_guarded_observer(self.plan, StatefulPolicyObserver,
            policy=self.policy, max_ticks=5, torch_module=torch, checked_dispatch_wrapper=None)
        self.assertEqual(run.max_ticks, 5)
        self.exercise(run)

    def test_output_production_factory_uses_real_model_without_legacy_wrapper(self):
        run = foreground.create_type1_observer(self.plan, StatefulPolicyObserver,
            policy=self.policy, duration_s=2, torch_module=torch, checked_dispatch_wrapper=None)
        self.assertEqual(run.max_ticks, 62)
        self.exercise(run)

    def test_legacy_diagnostic_factory_still_requires_selected_wrapper(self):
        with self.assertRaisesRegex(ValueError, 'Selected checked wrapper'):
            bridge.create_guarded_observer(fixtures.Fixture().plan(), StatefulPolicyObserver,
                policy=self.policy, max_ticks=5, torch_module=torch, checked_dispatch_wrapper=None)


if __name__ == '__main__':
    unittest.main()
