"""Offline, dual-bus tests for the finite load-transfer hold candidate."""
from dataclasses import replace
import math
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_load_transfer_hold as trial
from singularitydog_hw.can_readonly import ATParser
from test_rs05_fullbody_hold import (BUS_IDS, UIDS, FakeBus, WorkerClock, REAL_BARRIER,
                                     boot_file, fullbody_review)


class DelayedBurstBus(FakeBus):
    """Tick-one six-frame batch spends 8 ms per write before any publish."""

    def send(self, command):
        frame = ATParser().feed(command)[0]
        if frame.kind == 1 and frame.data[4:8] != bytes(4) and self.active_batches >= 6:
            self.clock.wait(.008)
        return super().send(command)


def reviewed(duration_s=5., *, active=False, gain_profile='kp3'):
    row = fullbody_review(active=False)
    row.update(schema=trial.REVIEW_SCHEMA, scope=trial.REVIEW_SCOPE,
               duration_s=duration_s, gain_profile=gain_profile,
               load_transfer_hold_authorized=active,
               preflight_review_complete=True,
               load_specific_limits_reviewed=True,
               supported_stance_passed=True, physical_catch_reviewed=True,
               power_cutoff_operator_reviewed=True,
               off_power_transfer_rehearsal_reviewed=True,
               wrap_equivalence_motor_ids=[3, 9], physical_full_turn_excluded=True,
               supported_floor_start_raw_rad_by_id={str(mid): .5 + .1 * mid
                                                     for mid in range(1, 13)},
               supported_floor_start_tolerance_rad=trial.SUPPORTED_FLOOR_START_TOLERANCE_RAD,
               feedback_torque_abort_nm_by_motor={str(mid): 1.5 for mid in range(1, 13)})
    row.pop('supported_hold_authorized')
    return row


class LoadTransferHoldTests(unittest.TestCase):
    def buses(self, *, failure=None, failed_bus='front'):
        clock = WorkerClock()
        transports = {name: FakeBus(name, clock,
                      failure=failure if name == failed_bus else None)
                      for name in BUS_IDS}
        for bus in transports.values():
            bus.interleave_feedback = True
        return clock, transports

    def run_trial(self, clock, transports, *, duration_s=5., preflight_only=True,
                  live_output=False, review=None, gain_profile='kp3',
                  active_start_signal=None, emit=None, barrier_factory=None):
        if review is None:
            review = reviewed(duration_s, active=live_output,
                              gain_profile=gain_profile)
        with boot_file(), patch.object(trial.threading, 'Barrier',
                                       side_effect=barrier_factory or clock.barrier), \
                patch.object(trial, 'LIVE_OUTPUT_ENABLED', True):
            return trial.run_load_transfer_hold(
                transports, UIDS, lambda: None, emit or (lambda _: None),
                validated_review=review, duration_s=duration_s,
                preflight_only=preflight_only, live_output=live_output,
                gain_profile=gain_profile, clock=clock, wait=clock.wait,
                active_start_signal=active_start_signal)

    def assert_stopped_all_twelve(self, buses):
        for transport in buses.values():
            self.assertEqual(transport.stop_calls[-1][0], transport.ids)
            self.assertFalse(transport.enabled)

    def test_disabled_preflight_is_default_and_has_no_active_frame(self):
        clock, buses = self.buses()
        benchmark_batches = {name: [] for name in buses}
        for name, bus in buses.items():
            old_feedback = bus.feedback_many

            def capture(commands, expected_ids, *, bus=bus, name=name, old=old_feedback):
                if bus.active_deadline is not None:
                    benchmark_batches[name].append(tuple(expected_ids))
                return old(commands, expected_ids)

            bus.feedback_many = capture
        started = threading.Event()
        result = self.run_trial(clock, buses, active_start_signal=started)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertFalse(result['live_output'])
        self.assertFalse(started.is_set())
        for name, transport in buses.items():
            self.assertEqual(transport.active_batches, 0)
            self.assertEqual(len(result['workers'][name]['electrical_samples']), 25)
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in transport.frames))
            self.assertEqual(len(transport.stop_calls), 2)
            self.assertEqual(benchmark_batches[name][:7],
                             [(mid,) for mid in transport.ids] + [transport.ids])
        self.assert_stopped_all_twelve(buses)

    def test_live_output_and_duration_bound_review_required_before_any_io(self):
        clock, buses = self.buses()
        with boot_file(), self.assertRaisesRegex(ValueError, 'source gate is disabled'):
            trial.run_load_transfer_hold(buses, UIDS, lambda: None, lambda _: None,
                validated_review=reviewed(active=True), preflight_only=False,
                live_output=True, gain_profile='kp3', clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))
        for duration in (True, 1.95, 15.05, math.nan, math.inf):
            clock, buses = self.buses()
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.run_trial(clock, buses, duration_s=duration,
                               preflight_only=False, live_output=True,
                               review=reviewed(5., active=True))
            self.assertTrue(all(not bus.calls for bus in buses.values()))
        for live, review in ((False, reviewed(5., active=True)),
                             (True, reviewed(5., active=False)),
                             (True, reviewed(2., active=True))):
            clock, buses = self.buses()
            with self.subTest(live=live, review_duration=review['duration_s']), self.assertRaises(ValueError):
                self.run_trial(clock, buses, preflight_only=False,
                               live_output=live, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))
        clock, buses = self.buses()
        buses['rear'].interleave_feedback = False
        with self.assertRaisesRegex(ValueError, 'interleaved'):
            self.run_trial(clock, buses, preflight_only=False, live_output=True)
        self.assertTrue(all(not bus.calls for bus in buses.values()))
        clock, buses = self.buses()
        review = reviewed(active=True)
        review['feedback_torque_abort_nm_by_motor']['9'] = 1.6
        with self.assertRaises(ValueError):
            self.run_trial(clock, buses, preflight_only=False,
                           live_output=True, review=review)
        self.assertTrue(all(not bus.calls for bus in buses.values()))
        for field in ('supported_stance_passed', 'physical_catch_reviewed',
                      'power_cutoff_operator_reviewed', 'off_power_transfer_rehearsal_reviewed'):
            clock, buses = self.buses()
            review = reviewed(active=True)
            review[field] = False
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.run_trial(clock, buses, preflight_only=False,
                               live_output=True, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))
        clock, buses = self.buses()
        buses['front'].centers[1] += math.radians(3.01)
        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('outside supported floor envelope' in error for error in result['errors']))
        self.assertTrue(all(not any(frame.kind == 3 for _, frame, _ in bus.frames)
                            for bus in buses.values()))
        self.assert_stopped_all_twelve(buses)

    def test_two_and_fifteen_second_limits_complete_with_fixed_targets_and_all_stop(self):
        for duration_s, ticks in ((2., 25), (15., 187)):
            clock, buses = self.buses()
            started = threading.Event()
            result = self.run_trial(clock, buses, duration_s=duration_s,
                                    preflight_only=False, live_output=True,
                                    active_start_signal=started)
            with self.subTest(duration_s=duration_s):
                self.assertEqual(result['status'],
                    'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(started.is_set())
                self.assertEqual(result['requested_ticks'], ticks)
                for name, bus in buses.items():
                    self.assertEqual(result['workers'][name]['cycle_count'], ticks)
                    self.assertEqual(len(result['workers'][name]['electrical_samples']), ticks)
                    self.assertEqual({row['motor_id'] for row in result['workers'][name]['electrical_samples']},
                                     set(bus.ids))
                    # Tick zero is six serialized one-axis exchanges.
                    self.assertEqual(bus.active_batches, ticks + 5)
                    self.assertEqual(len(bus.commanded), 6)
                    for mid in bus.ids:
                        self.assertAlmostEqual(bus.commanded[mid], bus.centers[mid], places=3)
                self.assert_stopped_all_twelve(buses)

    def test_feedback_fault_and_active_electrical_failure_abort_and_stop(self):
        for failure in ('fault', 'stale', 'missing', 'write'):
            clock, buses = self.buses(failure=failure)
            with self.subTest(failure=failure):
                result = self.run_trial(clock, buses, duration_s=2.,
                                        preflight_only=False, live_output=True)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertFalse(result['motion_completed'])
                self.assertTrue(result['stop_confirmed'])
                self.assert_stopped_all_twelve(buses)
        for name, altered in (('voltage', 34.9), ('can_timeout', 0)):
            clock, buses = self.buses()
            old_parameter = buses['front'].parameter

            def parameter(mid, key=None, *, old=old_parameter):
                result = old(mid, key)
                if key == name and buses['front'].active_batches >= 3:
                    return {'value': altered, 'ok': True}
                return result

            buses['front'].parameter = parameter
            with self.subTest(parameter=name):
                result = self.run_trial(clock, buses, duration_s=2.,
                                        preflight_only=False, live_output=True)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertTrue(result['stop_confirmed'])
                label = 'voltage' if name == 'voltage' else 'watchdog'
                self.assertTrue(any(label in error for error in result['errors']))
                self.assert_stopped_all_twelve(buses)
        clock, buses = self.buses()
        old_feedback = buses['rear'].feedback_many

        def high_torque(commands, expected_ids):
            found = old_feedback(commands, expected_ids)
            if buses['rear'].active_batches >= 3 and 9 in found:
                value, received = found[9]
                found[9] = replace(value, torque_nm=1.51), received
            return found

        buses['rear'].feedback_many = high_torque
        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('torque feedback monitor' in error for error in result['errors']))
        self.assertTrue(result['stop_confirmed'])
        self.assert_stopped_all_twelve(buses)

    def test_first_enable_mode2_torque_overlimit_aborts_before_any_gain(self):
        clock, buses = self.buses()
        old_value = buses['front'].value

        def high_torque_on_first_enable(mid):
            value = old_value(mid)
            if mid == 1 and mid in buses['front'].enabled:
                return replace(value, torque_nm=1.51)
            return value

        buses['front'].value = high_torque_on_first_enable
        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(any('torque feedback monitor' in error for error in result['errors']))
        for bus in buses.values():
            self.assertLessEqual(sum(frame.kind == 3 for _, frame, _ in bus.frames), 1)
            self.assertFalse(any(frame.kind == 1 and frame.data[4:8] != bytes(4)
                                 for _, frame, _ in bus.frames))
        self.assert_stopped_all_twelve(buses)

    def test_slow_cycle_logging_aborts_without_a_second_active_tick(self):
        clock, buses = self.buses()

        def slow_log(event):
            if event.get('kind') == 'load_transfer_cycle' and event.get('tick') == 0:
                clock.wait(.081)

        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True,
                                emit=slow_log)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(any('missed80ms' in error for error in result['errors']))
        for bus in buses.values():
            active = [frame for _, frame, _ in bus.frames
                      if frame.kind == 1 and frame.data[4:8] != bytes(4)]
            self.assertLessEqual(len(active), 6)
        self.assert_stopped_all_twelve(buses)

    def test_late_tick_zero_barrier_does_not_cue_operator(self):
        clock, buses = self.buses()
        started = threading.Event()
        logged = set()
        delayed = [False]

        def emit(event):
            if event.get('kind') == 'load_transfer_cycle' and event.get('tick') == 0:
                logged.add(event.get('bus'))

        def barrier_factory(parties, action):
            def wrapped_action():
                clock.synchronize()
                action()
                if logged == set(BUS_IDS) and not delayed[0]:
                    delayed[0] = True
                    clock.wait(.081)

            return REAL_BARRIER(parties, action=wrapped_action)

        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True,
                                active_start_signal=started, emit=emit,
                                barrier_factory=barrier_factory)
        self.assertTrue(delayed[0])
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertFalse(started.is_set())
        self.assert_stopped_all_twelve(buses)

    def test_only_reviewed_id3_id9_single_turn_branch_is_accepted(self):
        review = reviewed(2.)
        self.assertTrue(trial.floor_start_equivalent(-2 * math.pi + .004, 0., 3, review))
        self.assertTrue(trial.floor_start_equivalent(2 * math.pi - .004, 0., 9, review))
        self.assertFalse(trial.floor_start_equivalent(2 * math.pi + math.radians(10), 0., 3, review))
        self.assertFalse(trial.floor_start_equivalent(2 * math.pi, 0., 1, review))
        self.assertFalse(trial.floor_start_equivalent(4 * math.pi, 0., 3, review))
        self.assertFalse(trial.floor_start_equivalent(2 * math.pi, 0., 3,
                           {**review, 'physical_full_turn_excluded': False}))
        clock, buses = self.buses()
        buses['front'].centers[3] -= 2 * math.pi
        buses['rear'].centers[9] += 2 * math.pi + .004
        result = self.run_trial(clock, buses, duration_s=2., review=review)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['workers']['front']['wrap_equivalence_used'], {3: -1})
        self.assertEqual(result['workers']['rear']['wrap_equivalence_used'], {9: 1})
        self.assert_stopped_all_twelve(buses)
        clock, buses = self.buses()
        buses['front'].centers[3] -= 2 * math.pi + math.radians(10)
        result = self.run_trial(clock, buses, duration_s=2., review=review)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('outside supported floor envelope' in e for e in result['errors']))
        self.assert_stopped_all_twelve(buses)

    def test_delayed_tick_one_batch_uses_cycle_entry_snapshot(self):
        clock, buses = self.buses()
        buses['rear'] = DelayedBurstBus('rear', clock)
        buses['rear'].interleave_feedback = True
        original_check = trial.check_feedback

        def virtual_clock_check(value, center, received, now, **kwargs):
            # WorkerClock advances one fake thread at a time, whereas the
            # hardware monotonic clock advances for both workers. Clamp only
            # impossible future timestamps introduced by that test clock.
            return original_check(value, center, received, max(now, received), **kwargs)

        with patch.object(trial, 'check_feedback', side_effect=virtual_clock_check):
            result = self.run_trial(clock, buses, duration_s=2.,
                                    preflight_only=False, live_output=True)
        self.assertEqual(result['status'],
                         'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertEqual([result['workers'][name]['cycle_count'] for name in BUS_IDS], [25, 25])
        self.assertFalse(any(phase == 'motion' for phase, _ in buses['rear'].guard_checks))
        self.assert_stopped_all_twelve(buses)

    def test_peer_loses_tick_one_replies_limits_other_bus_to_one_batch(self):
        clock, buses = self.buses()
        rear = buses['rear']
        original = rear.feedback_many

        def missing_after_cycle_entry(commands, expected_ids):
            commands = tuple(commands)
            if rear.active_batches == 6 and len(commands) == 6:
                rear.own('feedback_many', tuple(expected_ids))
                for command in commands:
                    rear.send(command)
                rear.active_batches += 1
                return {}
            return original(commands, expected_ids)

        rear.feedback_many = missing_after_cycle_entry
        result = self.run_trial(clock, buses, duration_s=2.,
                                preflight_only=False, live_output=True)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertFalse(result['motion_completed'])
        self.assertLessEqual(buses['front'].active_batches, 7)
        self.assertLessEqual(buses['rear'].active_batches, 7)
        self.assert_stopped_all_twelve(buses)


if __name__ == '__main__':
    unittest.main()
