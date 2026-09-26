"""Offline prospective position-first guards; no serial or actuator access."""
import math
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch
from test_rs05_leg_settled import samples, CENTERS, WindowTransport
from test_rs05_leg_trial import UIDS, DIRECTIONS
from test_position_response_evidence import evidence_data, evidence_file
from singularitydog_hw.rs05_leg_trial import evaluate_settled_window, run_leg_trial, main
from singularitydog_hw.position_response_evidence import load_position_response_evidence


class SelectedWindowTransport(WindowTransport):
    def __init__(self, leg, change=None):
        super().__init__(change)
        self.ids = {'FR': (1, 2, 3), 'FL': (4, 5, 6), 'RR': (7, 8, 9), 'RL': (10, 11, 12)}[leg]
        self.centers = dict(zip(self.ids, (5.5, 4.4, 5.4)))
        self.watchdogs = {mid: 0 for mid in self.ids}

    def parameter(self, mid, name=None):
        if name is None:
            self.calls.append(('parameter', mid, name, self.clock()))
            self.clock.wait(.004)
            return {'mcu_uid_hex': f'{mid:016x}'}
        return super().parameter(mid, name)


def evaluate_fl(rows):
    return evaluate_settled_window({i+3: v for i, v in rows.items()},
        {i+3: v for i, v in CENTERS.items()}, profile='position-v2')

class PositionGateTests(unittest.TestCase):
    def evaluate(self, rows):
        return evaluate_settled_window(rows, CENTERS, profile="position-v2")

    def test_alternating_velocity_noise_retained_and_warned_only_in_v2(self):
        rows = samples(velocity=lambda t: .06 if round(t*10)%2 else -.06)
        self.assertFalse(evaluate_settled_window(rows, CENTERS)["passed"])
        report = self.evaluate(rows)
        self.assertTrue(report["passed"])
        self.assertTrue(report["warnings"])
        self.assertFalse(report["absolute_rest_proven"])
        self.assertFalse(report["joint_calibration_verified"])
        self.assertAlmostEqual(report["motors"][1]["velocity_RMS_rad_s"], .06)
        self.assertEqual(report["motors"][1]["samples"], rows[1])

    def test_frozen_position_does_not_mask_reported_constant_velocity(self):
        self.assertFalse(self.evaluate(samples(velocity=lambda t: .06))["passed"])

    def test_small_signed_velocity_bias_with_stationary_position_passes(self):
        # A physical ID1 window had only 0.000384 rad of position variation,
        # despite a -0.02736 rad/s signed velocity estimate. Position drift,
        # the six-sample tail, and per-sample feedback guards still apply.
        rows = samples(position=lambda t: .000384 if round(t*10) % 2 else 0.,
                       velocity=lambda t: -.02736)
        report = self.evaluate(rows)
        self.assertTrue(report['passed'])
        self.assertAlmostEqual(report['limits']['abs_velocity_mean_rad_s'], .05)
        self.assertLessEqual(report['motors'][1]['position_range_rad'], .0003841)

    def test_signed_bias_over_new_limit_and_real_creep_still_fail(self):
        high_bias = self.evaluate(samples(velocity=lambda t: .051))
        self.assertFalse(high_bias['passed'])
        self.assertTrue(any('abs_velocity_mean_rad_s' in error
                            for error in high_bias['errors']))
        # The same biased velocity as the physical window must not hide actual
        # displacement across the 2-second observation.
        creep = self.evaluate(samples(position=lambda t: .0006*t,
                                           velocity=lambda t: -.02736))
        self.assertFalse(creep['passed'])
        self.assertTrue(any('position_range_rad' in error or 'abs_OLS_slope_rad_s' in error
                            for error in creep['errors']))

    def test_creep_returning_oscillation_and_late_motion_rejected(self):
        for position in (lambda t: .0006*t,
                         lambda t: .0006*math.sin(math.pi*t),
                         lambda t: max(0, t-1.5)*.0015):
            with self.subTest(position=position):
                self.assertFalse(self.evaluate(samples(position=position))["passed"])

    def test_missing_stale_fault_and_instantaneous_guards_preserved(self):
        for failure in ("missing", "stale", "fault", "instant", "mode"):
            rows = samples()
            if failure == "missing": rows[1].pop(10)
            if failure == "stale": rows[1][10]["checked_monotonic_s"] += .11
            if failure == "fault": rows[1][10]["feedback"]["fault_bits"] = 1
            if failure == "instant": rows[1][10]["feedback"]["velocity_rad_s"] = .51
            if failure == "mode": rows[1][10]["feedback"]["mode_state"] = 2
            with self.subTest(failure=failure):
                self.assertFalse(self.evaluate(rows)["passed"])

    def test_failed_gate_collects_exact_window_and_never_enables_or_retries(self):
        def change(t, found):
            fb, ts = found[2]
            found[2] = (replace(fb, velocity_rad_s=.06), ts)
            return found
        t = WindowTransport(change)
        with evidence_file() as path:
            result = run_leg_trial(t, UIDS, lambda: None, lambda _: None,
                directions=DIRECTIONS, clock=t.clock, wait=t.clock.wait, profile="position-v2",
                position_response_evidence=path)
        self.assertEqual(t.window_count, 21)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(any(f.kind == 3 for _, f in t.frames))
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["joint_calibration_verified"])

    def test_pure_calculation_keeps_same_limits_for_each_leg_and_rejects_mixed_ids(self):
        reference = self.evaluate(samples())
        for shift in (0, 3, 6, 9):
            result = evaluate_settled_window({i+shift: v for i, v in samples().items()},
                {i+shift: v for i, v in CENTERS.items()}, profile='position-v2')
            self.assertTrue(result['passed'])
            self.assertEqual(result['limits'], reference['limits'])
        with self.assertRaises(ValueError):
            evaluate_settled_window({1:[], 5:[], 9:[]}, {1:0., 5:0., 9:0.}, profile='position-v2')

    def test_fl_uses_same_strict_limits_and_rejects_bias_creep_stale(self):
        fr = self.evaluate(samples())
        fl = evaluate_fl(samples())
        self.assertEqual(fr['limits'], fl['limits'])
        noise = samples(velocity=lambda t: .06 if round(t*10)%2 else -.06)
        self.assertTrue(evaluate_fl(noise)['passed'])
        self.assertTrue(evaluate_fl(noise)['warnings'])
        for rows in (samples(velocity=lambda t: .06), samples(position=lambda t: .0006*t)):
            self.assertFalse(evaluate_fl(rows)['passed'])
        stale = samples(); stale[3][10]['checked_monotonic_s'] += .11
        self.assertFalse(evaluate_fl(stale)['passed'])

    def test_run_api_rejects_missing_wrong_leg_uid_or_changed_file_before_transport(self):
        for leg in ('FR', 'FL', 'RR', 'RL'):
            t = SelectedWindowTransport(leg)
            expected = {i: f'{i:016x}' for i in t.ids}
            with self.subTest(leg=leg), self.assertRaises(ValueError):
                run_leg_trial(t, expected, lambda: None, lambda _: None,
                              directions=DIRECTIONS, profile='position-v2')
            with evidence_file('FR' if leg == 'FL' else 'FL') as wrong:
                with self.assertRaises(ValueError):
                    run_leg_trial(t, expected, lambda: None, lambda _: None,
                        directions=DIRECTIONS, profile='position-v2', position_response_evidence=wrong)
            bad = evidence_data(leg); bad['motors'][str(t.ids[-1])]['mcu_uid_hex'] = 'f'*16
            with evidence_file(leg, bad) as wrong, self.assertRaises(ValueError):
                run_leg_trial(t, expected, lambda: None, lambda _: None,
                    directions=DIRECTIONS, profile='position-v2', position_response_evidence=wrong)
            with evidence_file(leg) as path:
                pin = load_position_response_evidence(path, expected, leg=leg)['sha256']
                path.write_text(path.read_text() + '\n')
                with self.assertRaisesRegex(ValueError, 'changed'):
                    run_leg_trial(t, expected, lambda: None, lambda _: None, directions=DIRECTIONS,
                        profile='position-v2', position_response_evidence=path,
                        position_response_evidence_sha256=pin)
            self.assertEqual(t.calls, []); self.assertEqual(t.frames, []); self.assertEqual(t.stop_calls, [])

    def test_rear_legs_without_their_own_evidence_rejected_before_transport(self):
        for ids in ((7, 8, 9), (10, 11, 12)):
            transport = Mock(ids=ids)
            with evidence_file('FL') as path, self.assertRaises(ValueError):
                run_leg_trial(transport, {i: f'{i:016x}' for i in ids}, lambda: None, lambda _: None,
                    directions=DIRECTIONS, profile='position-v2', position_response_evidence=path)
            self.assertEqual(transport.mock_calls, [])

    def test_valid_each_leg_file_preserves_trial_and_failed_fresh_window_all_stop(self):
        for leg in ('FR', 'FL', 'RR', 'RL'):
            for bad_window in (False, True):
                def change(t, found):
                    if bad_window:
                        mid = t.ids[-1]; fb, ts = found[mid]
                        found[mid] = replace(fb, velocity_rad_s=.06), ts
                    return found
                t = SelectedWindowTransport(leg, change)
                with self.subTest(leg=leg, bad_window=bad_window), evidence_file(leg) as path:
                    result = run_leg_trial(t, {i: f'{i:016x}' for i in t.ids}, lambda: None, lambda _: None,
                        directions=DIRECTIONS, clock=t.clock, wait=t.clock.wait, profile='position-v2',
                        position_response_evidence=path)
                self.assertEqual(t.window_count, 21)
                self.assertEqual(result['position_response_evidence']['leg'], leg)
                self.assertEqual(result['motion_completed'], not bad_window)
                self.assertEqual(any(f.kind == 3 for _, f in t.frames), not bad_window)
                self.assertTrue(result['stop_confirmed'])
                self.assertEqual(t.stop_calls[-1], t.ids)
                self.assertFalse(result['joint_calibration_verified'])

    def test_cli_fl_plan_is_scoped_and_does_not_open_transport(self):
        with tempfile.TemporaryDirectory() as directory, evidence_file('FL') as path:
            root = Path(directory); uid_file = root/'uids.json'
            uid_file.write_text(json.dumps({i: f'{i:016x}' for i in range(1, 13)}))
            argv = ['--leg', 'FL', '--directions', '-1', '-1', '-1', '--expected-uids', str(uid_file),
                    '--output', str(root/'unused'), '--stationarity-profile', 'position-v2',
                    '--position-response-evidence', str(path)]
            with patch('singularitydog_hw.rs05_leg_trial.LegTrialTransport') as transport, \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main(argv), 0)
            transport.assert_not_called(); self.assertFalse((root/'unused').exists())
            plan = json.loads(output.getvalue())
            self.assertEqual(plan['position_response_evidence']['motor_ids'], [4, 5, 6])
            self.assertEqual(plan['trajectory_duration_s'], 5)
            self.assertEqual(plan['Kp'], 3.)
            self.assertEqual(plan['Kd'], .15)

if __name__ == "__main__":
    unittest.main()
