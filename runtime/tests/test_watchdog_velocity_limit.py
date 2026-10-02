"""Explicit zero-gain speed choices; fake clocks/buses only, no live approval."""
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import watchdog_commissioning as watchdog
from test_watchdog_commissioning import Clock, FakeChannel


class VelocityLimitTests(unittest.TestCase):
    def run_case(self, *, stage='silent', values=None, limit=.5, bad=None,
                 stop_missing=False, disable_bound_ns=None):
        clock = Clock()
        class Injected(FakeChannel):
            def exchange(self, mid, step, center=0.):
                reply = super().exchange(mid, step, center)
                if mid == 4 and step == 'zero':
                    if self.zero_counts[mid] == 2:
                        self.last_zero_start = reply['request_start_ns']
                    if self.zero_counts[mid] == (2 if stage == 'active' else 3):
                        reply.update(values or {})
                    if self.zero_counts[mid] == 3 and disable_bound_ns is not None:
                        reply['received_ns'] = self.last_zero_start + disable_bound_ns
                return reply
            def stop_all(self):
                result = super().stop_all()
                if stop_missing and 4 in self.ids:
                    result.update(complete=False, confirmed_ids=[],
                                  unconfirmed_ids=list(self.ids), errors=['Synthetic missing STOP'])
                return result
        channels = {name: Injected(ids, clock, bad=bad if name == 'front' else None)
                    for name, ids in watchdog.BUSES.items()}
        expected = {mid: (bytes([mid])*8).hex() for mid in watchdog.IDS}
        report = watchdog.run(channels, expected, clock=clock, wait=clock.wait,
                              position_window_deg=6, velocity_limit_rad_s=limit)
        return report, channels

    def assert_stopped_without_output(self, report, channels):
        self.assertTrue(all(channel.stop_calls == 1 for channel in channels.values()))
        self.assertFalse(any(channel.enabled for channel in channels.values()))
        self.assertFalse(report['positive_gain_sent'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertFalse(report['approved_for_runtime'])

    def test_default_point_five_and_legacy_observation_key_are_unchanged(self):
        self.assertEqual(watchdog.plan()['zero_gain_velocity_limit_rad_s'], .5)
        for stage in ('active', 'silent'):
            for value in (-.5, .5):
                report, channels = self.run_case(stage=stage, values={'velocity_rad_s': value})
                self.assertEqual(report['status'], 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC')
                obs = report['axes']['4']['zero_gain_observations'][0 if stage == 'active' else -1]
                self.assertTrue(obs['checks']['velocity_abs_within_0_5rad_s'])
                self.assertNotIn('velocity_abs_within_1_0rad_s', obs['checks'])
                self.assertEqual(obs['limits']['velocity_abs_max_rad_s'], .5)
                self.assert_stopped_without_output(report, channels)
            report, channels = self.run_case(stage=stage, values={'velocity_rad_s': .691996643})
            self.assertEqual(report['status'], 'ABORTED')
            self.assertEqual(report['axes']['4']['zero_gain_observations'][-1]['failed_checks'],
                             ['velocity_abs_within_0_5rad_s'])
            self.assert_stopped_without_output(report, channels)

    def test_explicit_one_inclusive_both_signs_and_phases_records_actual_limit(self):
        for stage in ('active', 'silent'):
            for value in (-1., 1., .691996643):
                report, channels = self.run_case(stage=stage, values={'velocity_rad_s': value}, limit=1.)
                self.assertEqual(report['status'], 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC', report['errors'])
                self.assertEqual(report['zero_gain_velocity_limit_rad_s'], 1.)
                obs = report['axes']['4']['zero_gain_observations'][0 if stage == 'active' else -1]
                self.assertTrue(obs['checks']['velocity_abs_within_1_0rad_s'])
                self.assertNotIn('velocity_abs_within_0_5rad_s', obs['checks'])
                self.assertEqual(obs['limits']['velocity_abs_max_rad_s'], 1.)
                self.assertEqual(obs['reply']['velocity_rad_s'], value)
                self.assert_stopped_without_output(report, channels)

    def test_one_nextafter_and_nonfinite_feedback_abort_before_next_enable(self):
        for stage in ('active', 'silent'):
            for value in (math.nextafter(1., math.inf), math.nextafter(-1., -math.inf),
                          float('nan'), float('inf'), -float('inf')):
                report, channels = self.run_case(stage=stage, values={'velocity_rad_s': value}, limit=1.)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertEqual(report['axes']['4']['zero_gain_observations'][-1]['failed_checks'],
                                 ['velocity_abs_within_1_0rad_s'])
                self.assertEqual([i for c in channels.values() for _, i, step in c.calls if step == 'enable'],
                                 [1, 2, 3, 4])
                self.assert_stopped_without_output(report, channels)

    def test_only_two_finite_numeric_choices_reject_before_any_channel_call(self):
        for value in (.5, 1, 1.):
            self.assertEqual(watchdog.plan(velocity_limit_rad_s=value)['zero_gain_velocity_limit_rad_s'], value)
        for value in (True, False, 0, .4999, .75, 1.0001, 2, '1', None,
                      float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'velocity limit'):
                watchdog.plan(velocity_limit_rad_s=value)
            clock = Clock()
            channels = {name: FakeChannel(ids, clock) for name, ids in watchdog.BUSES.items()}
            expected = {mid: (bytes([mid])*8).hex() for mid in watchdog.IDS}
            with self.assertRaises(ValueError):
                watchdog.run(channels, expected, clock=clock, wait=clock.wait, velocity_limit_rad_s=value)
            self.assertTrue(all(not c.calls and c.stop_calls == 0 for c in channels.values()))

    def test_explicit_one_preserves_position_temperature_mode_fault_guards(self):
        for stage in ('active', 'silent'):
            for values in ({'protocol_position_rad': math.nextafter(math.radians(6), math.inf)},
                           {'temperature_c': 60.}, {'temperature_c': -10.01},
                           {'mode_state': 0 if stage == 'active' else 2}, {'fault_bits': 1}):
                report, channels = self.run_case(stage=stage, values=values, limit=1.)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertFalse(report['axes']['4']['command_loss_tested'])
                self.assertEqual([i for c in channels.values() for _, i, step in c.calls if step == 'enable'],
                                 [1, 2, 3, 4])
                self.assert_stopped_without_output(report, channels)

    def test_explicit_one_preserves_exact_disable_bound_and_no_silence_polls(self):
        for bound, expected in ((250_000_000, 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC'),
                                (250_000_001, 'ABORTED')):
            report, channels = self.run_case(limit=1., disable_bound_ns=bound)
            self.assertEqual(report['status'], expected, report['errors'])
            self.assertEqual(report['configured_timeout_ms'], 200)
            self.assertEqual(report['silent_interval_ms'], 210)
            self.assertEqual(report['accepted_disable_upper_bound_ms'], 250)
            self.assertEqual(report['axes']['4']['disable_reply_upper_bound_ms'], bound / 1e6)
            calls = sorted(call for c in channels.values() for call in c.calls)
            zeros = [t for t, mid, step in calls if mid == 4 and step == 'zero']
            self.assertFalse(any(zeros[1] < t < zeros[2] for t, _, _ in calls))
            self.assert_stopped_without_output(report, channels)
        for bad in ('enable_timeout', 'no_disable', 'late', 'late_write_completion'):
            report, channels = self.run_case(limit=1., bad=bad)
            self.assertEqual(report['status'], 'ABORTED')
            self.assert_stopped_without_output(report, channels)

    def test_speed_failure_and_missing_stop_never_claim_complete(self):
        report, channels = self.run_case(limit=1., values={'velocity_rad_s': 1.001}, stop_missing=True)
        self.assertEqual(report['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertIn('Physically switch motor power Off', report['errors'][-1])
        self.assert_stopped_without_output(report, channels)

    def test_cli_default_and_explicit_one_plan_only_and_invalid_choices(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'uids.json'
            path.write_text(json.dumps({str(mid): (bytes([mid])*8).hex() for mid in watchdog.IDS}))
            for flags, limit in (([], .5), (['--velocity-limit-rad-s', '1.0'], 1.)):
                output = io.StringIO()
                with redirect_stdout(output), patch.object(watchdog, 'Channel') as channel:
                    self.assertEqual(watchdog.main(['--expected-uids', str(path), *flags]), 0)
                    channel.assert_not_called()
                result = json.loads(output.getvalue())
                self.assertEqual(result['zero_gain_velocity_limit_rad_s'], limit)
                self.assertFalse(result['hardware_opened'])
                self.assertFalse(result['positive_gains_available'])
                self.assertFalse(result['learned_targets_available'])
            for value in ('0.75', 'nan', 'inf', 'true', '2'):
                with patch('sys.stderr', io.StringIO()), self.assertRaises(SystemExit), \
                        patch.object(watchdog, 'Channel') as channel:
                    watchdog.main(['--expected-uids', str(path), '--velocity-limit-rad-s', value])
                channel.assert_not_called()

    def test_channel_zero_wire_keeps_zero_gains_and_existing_fixed_deadlines(self):
        channel = watchdog.Channel.__new__(watchdog.Channel)
        channel.ids = watchdog.BUSES['front']
        parser = codec.ATParser()
        frame, = parser.feed(channel._wire(4, 'zero', center=.1))
        self.assertEqual(frame.kind, 1)
        self.assertEqual(struct.unpack('>4H', frame.data)[2:], (0, 0))
        self.assertEqual(watchdog.REQUEST_NS, 250_000_000)
        self.assertEqual(watchdog.SILENCE_NS, 210_000_000)
        self.assertEqual(watchdog.MAX_DISABLE_UPPER_BOUND_NS, 250_000_000)
        for step in ('learned_target', 'positive_gain', 'arbitrary_parameter_write'):
            with self.assertRaises(ValueError):
                channel._wire(4, step)


if __name__ == '__main__':
    unittest.main()
