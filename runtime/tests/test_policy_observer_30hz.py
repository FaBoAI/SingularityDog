"""30Hz state/sensor integration; virtual telemetry, no device operations."""
import types
import unittest

from singularitydog_hw import policy_observer as observer
from singularitydog_hw.policy_clock import PolicyClock, control_clock_payload
from test_policy_observer import Policy, make, snapshot


class Policy30(Policy):
    def __init__(self):
        super().__init__()
        self.policy_dt_s = 1/30
        self.controller = types.SimpleNamespace(dt=1/30)
        self.elapsed = 0.

    def reset(self, ids):
        super().reset(ids)
        self.elapsed = 0.

    def __call__(self, *args):
        self.elapsed += self.policy_dt_s
        return super().__call__(*args)


class Observer30Tests(unittest.TestCase):
    def test_rational_grid_and_recurrent_state_are_not_reset_per_tick(self):
        policy = Policy30()
        clock = PolicyClock(30)
        run = make(policy, max_ticks=300, control_clock=control_clock_payload(30))
        epoch = 1_000_000_000
        run.reset_run(epoch, warmup_completed=True)
        results = [run.consume(snapshot(clock.release_ns(epoch, i))) for i in range(300)]
        self.assertEqual(policy.resets, 1)
        self.assertAlmostEqual(policy.elapsed, 10., places=11)
        self.assertEqual(results[-1]['tick_ns'], epoch+9_966_666_666)
        self.assertEqual(run._next_tick_ns, epoch+10_000_000_000)
        self.assertEqual(results[-1]['dt_s'], 1/30)
        self.assertFalse(results[-1]['output_allowed'])
        self.assertEqual(run.finish()['status'], 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')

    def test_rounded_up_or_duplicate_slots_are_rejected(self):
        for tick in (1_033_333_334, 1_000_000_000):
            run = make(Policy30(), control_clock=control_clock_payload(30))
            run.reset_run(1_000_000_000, warmup_completed=True)
            run.consume(snapshot())
            with self.assertRaisesRegex(observer.ObserverError, '30Hz rational tick'):
                run.consume(snapshot(tick))
            self.assertEqual(run.status, 'INCOMPLETE')

    def test_legacy_model_or_controller_clock_mismatch_is_rejected(self):
        for target in ('policy', 'controller'):
            policy = Policy30()
            if target == 'policy':policy.policy_dt_s = .02
            else:policy.controller.dt = .02
            with self.assertRaisesRegex(observer.ObserverError, 'clocks'):
                make(policy, control_clock=control_clock_payload(30))

    def test_legacy_native_checked_dispatch_is_not_available(self):
        with self.assertRaisesRegex(observer.ObserverError, 'Old checked/native'):
            make(Policy30(), control_clock=control_clock_payload(30),
                 checked_dispatch_wrapper=object())

    def test_new_model_cannot_silently_use_default_50hz_clock(self):
        with self.assertRaisesRegex(observer.ObserverError, 'requires an explicit30Hz'):
            make(Policy30())

    def test_measured_acquisition_timestamp_does_not_change_model_dt(self):
        run = make(Policy30(), measured_diagnostic_ticks=True,
                   control_clock=control_clock_payload(30))
        run.reset_run(1_000_000_000, warmup_completed=True)
        result = run.consume(snapshot(1_010_000_000))
        self.assertEqual(result['dt_s'], 1/30)
        self.assertIn('model dt remains1/30s', result['timing_scope'])
        self.assertEqual(run._next_tick_ns, 1_010_000_001)

    def test_bad_freshness_or_missing_input_still_rejects(self):
        for damage in ('missing', 'changed_age'):
            run = make(Policy30(), control_clock=control_clock_payload(30))
            run.reset_run(1_000_000_000, warmup_completed=True)
            value = snapshot()
            if damage == 'missing':value['motors'].pop()
            else:value['motors'][0]['age_upper_bound_ns'] += 1
            with self.assertRaises(observer.ObserverError):run.consume(value)
            self.assertEqual(run.ticks_completed, 0)

    def test_old_default_remains_20ms_without_extra_clock_fields(self):
        run = make()
        run.reset_run(1_000_000_000, warmup_completed=True)
        first = run.consume(snapshot())
        self.assertEqual(first['dt_s'], .02)
        self.assertNotIn('control_clock', first)
        run.consume(snapshot(1_020_000_000))
        self.assertEqual(run._next_tick_ns, 1_040_000_000)


if __name__ == '__main__':unittest.main()
