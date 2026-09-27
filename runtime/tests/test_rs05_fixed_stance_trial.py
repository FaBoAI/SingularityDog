"""Virtual two-bus fault injection for the disabled fixed-stance runner.

The production gate is never changed on disk. Active-path tests patch it only
inside this process and use FakeBus; they do not open a serial device.
"""

import math
from dataclasses import replace
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_fixed_stance_trial as stance
from singularitydog_hw.fixed_stance_candidate import REQUIRED_REVIEWS, SCHEMA
from test_current_hold_review import BOOT_ID
from test_rs05_fullbody_hold import BUS_IDS, FakeBus, UIDS, WorkerClock, boot_file


def reviewed():
    starts = {str(i): .5 + .1*i for i in range(1, 13)}
    target = {str(i): starts[str(i)] + math.radians(5.) for i in range(1, 13)}
    identities = {str(i): UIDS[i] for i in range(1, 13)}
    return {
        'schema': SCHEMA, 'boot_id': BOOT_ID, 'motor_uids': identities,
        'stance_capture_boot_id': BOOT_ID,
        'stance_capture_motor_uids': identities.copy(),
        'stance_capture_raw_rad_by_id': target.copy(),
        'hold_summary_sha256': 'a'*64, 'stance_capture_sha256': 'b'*64,
        'physical_route_review_sha256': 'c'*64,
        'segment_clearance_sha256': ['d'*64],
        'clearance_reviewed_start_envelope_deg': 3.,
        'reviewed_raw_corridor_by_id': {str(i): {
            'min_rad': starts[str(i)] - math.radians(4.),
            'max_rad': starts[str(i)] + math.radians(20.),
        } for i in range(1, 13)},
        'raw_corridor_physical_source_note': 'Virtual corridor only.',
        'hold_status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
        'hold_stop_confirmed': True,
        'old_d17_target_reused': False,
        'learned_policy_allowed': False,
        'automatic_retry_allowed': False,
        'start_raw_rad_by_id': starts,
        'waypoints_raw_rad_by_id': [target],
        'fixed_stance_raw_rad_by_id': target.copy(),
        'source_files_verified': True,
        'frozen_package_sha256': 'e'*64,
        'joint_limits_all_samples_verified': True,
        'physical_route_all_samples_verified': True,
        'supported_transition_authorized': True,
        **{flag: True for flag in REQUIRED_REVIEWS},
    }


class FixedStanceTrialTests(unittest.TestCase):
    def run_trial(self, *, failure=None, failure_bus='front', active=False, review=None):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock, failure=failure if name == failure_bus else None)
                 for name in BUS_IDS}
        with (boot_file(), patch.object(stance.threading, 'Barrier', side_effect=clock.barrier),
              patch.object(stance, 'LIVE_OUTPUT_ENABLED', active),
              patch.object(stance, 'verify_package_files', return_value={'source_sha256': 'virtual'})):
            result = stance.run_fixed_stance_trial(
                buses, UIDS, lambda: None, lambda _: None,
                validated_review=reviewed() if review is None else review,
                preflight_only=not active,
                package_dir='virtual-test-package' if active else None,
                clock=clock, wait=clock.wait)
        return result, buses

    def assert_all_stopped(self, buses):
        for name, bus in buses.items():
            self.assertTrue(bus.stop_calls, name)
            self.assertEqual(bus.stop_calls[-1][0], BUS_IDS[name])
            self.assertFalse(bus.enabled)
            self.assertEqual(len(bus.owner_threads), 1)

    def test_default_active_output_refused_before_touching_bus(self):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock) for name in BUS_IDS}
        with boot_file(), self.assertRaisesRegex(ValueError, 'Live fixed-stance output is disabled'):
            stance.run_fixed_stance_trial(buses, UIDS, lambda: None, lambda _: None,
                                           validated_review=reviewed(), preflight_only=False,
                                           clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_active_requires_frozen_package_before_any_bus_io(self):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock) for name in BUS_IDS}
        with (boot_file(), patch.object(stance, 'LIVE_OUTPUT_ENABLED', True),
              self.assertRaisesRegex(ValueError, 'frozen stance package')):
            stance.run_fixed_stance_trial(buses, UIDS, lambda: None, lambda _: None,
                                           validated_review=reviewed(), preflight_only=False,
                                           clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_disabled_preflight_never_enables(self):
        review = reviewed()
        review['joint_limits_all_samples_verified'] = False
        review['physical_route_all_samples_verified'] = False
        result, buses = self.run_trial(review=review)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        for bus in buses.values():
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in bus.frames))
            self.assertFalse(any(frame.kind == 1 and frame.data[4:8] != bytes(4)
                                 for _, frame, _ in bus.frames))
        self.assert_all_stopped(buses)

    def test_virtual_five_degree_path_and_all_stop(self):
        result, buses = self.run_trial(active=True)
        self.assertEqual(result['status'], 'SUPPORTED_FIXED_STANCE_CANDIDATE_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['active_ticks'], 180)
        self.assertFalse(result['self_supported_standing_verified'])
        for bus in buses.values():
            self.assertEqual(result['workers'][bus.bus_name]['cycle_count'], 180)
            self.assertEqual(sum(frame.kind == 3 for _, frame, _ in bus.frames), 6)
        self.assert_all_stopped(buses)

    def test_two_virtual_segments_each_have_confirmed_endpoint_hold(self):
        review = reviewed()
        first = review['waypoints_raw_rad_by_id'][0]
        second = {str(i): first[str(i)] + math.radians(5.) for i in range(1, 13)}
        review['waypoints_raw_rad_by_id'] = [first, second]
        review['stance_capture_raw_rad_by_id'] = second
        review['fixed_stance_raw_rad_by_id'] = second
        review['segment_clearance_sha256'] = ['d'*64, 'f'*64]
        result, buses = self.run_trial(active=True, review=review)
        self.assertEqual(result['status'], 'SUPPORTED_FIXED_STANCE_CANDIDATE_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['active_ticks'], 360)
        self.assertEqual(result['reviewed_segment_count'], 2)
        for bus in buses.values():
            self.assertEqual(len(result['workers'][bus.bus_name]['continuous_hold_checks']), 2)
            self.assertTrue(all(row['confirmed'] for row in
                                result['workers'][bus.bus_name]['continuous_hold_checks']))
        self.assert_all_stopped(buses)

    def test_missing_reply_aborts_and_attempts_all_twelve_stop(self):
        result, buses = self.run_trial(active=True, failure='missing')
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['motion_completed'])
        self.assert_all_stopped(buses)

    def test_fault_aborts_and_attempts_all_twelve_stop(self):
        result, buses = self.run_trial(active=True, failure='fault')
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['motion_completed'])
        self.assert_all_stopped(buses)

    def test_measured_feedback_outside_physical_corridor_aborts_below_tracking_limit(self):
        clock = WorkerClock()

        class CorridorViolationBus(FakeBus):
            def value(self, mid):
                value = super().value(mid)
                if mid == 1 and self.active_batches >= 3 and self.enabled:
                    # Nine degrees is outside the reviewed eight-degree
                    # corridor, yet remains below the twelve-degree tracking
                    # error monitor used by the diagnostic worker.
                    return replace(value, protocol_position_rad=(
                        self.centers[mid] + math.radians(9.)))
                return value

        buses = {'front': CorridorViolationBus('front', clock),
                 'rear': FakeBus('rear', clock)}
        review = reviewed()
        review['reviewed_raw_corridor_by_id']['1']['max_rad'] = (
            review['start_raw_rad_by_id']['1'] + math.radians(8.))
        with (boot_file(), patch.object(stance.threading, 'Barrier', side_effect=clock.barrier),
              patch.object(stance, 'LIVE_OUTPUT_ENABLED', True),
              patch.object(stance, 'verify_package_files', return_value={'source_sha256': 'virtual'})):
            result = stance.run_fixed_stance_trial(
                buses, UIDS, lambda: None, lambda _: None,
                validated_review=review, preflight_only=False,
                package_dir='virtual-test-package', clock=clock, wait=clock.wait)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('measured position left reviewed raw corridor' in error
                            for error in result['errors']))
        self.assert_all_stopped(buses)

    def test_deadline_or_write_failure_aborts_and_attempts_all_stop(self):
        for failure in ('deadline', 'write'):
            with self.subTest(failure=failure):
                result, buses = self.run_trial(active=True, failure=failure)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertFalse(result['motion_completed'])
                self.assert_all_stopped(buses)

    def test_stop_exception_fallback_attempts_each_id(self):
        clock = WorkerClock()

        class StopExceptionBus(FakeBus):
            def stop_all(self, ids=None):
                if self.bus_name == 'front' and self.stop_calls:
                    self.own('injected_stop_exception')
                    raise IOError('Injected final group STOP exception')
                return super().stop_all(ids)

        buses = {name: StopExceptionBus(name, clock) for name in BUS_IDS}
        with (boot_file(), patch.object(stance.threading, 'Barrier', side_effect=clock.barrier),
              patch.object(stance, 'LIVE_OUTPUT_ENABLED', True),
              patch.object(stance, 'verify_package_files', return_value={'source_sha256': 'virtual'})):
            result = stance.run_fixed_stance_trial(
                buses, UIDS, lambda: None, lambda _: None,
                validated_review=reviewed(), preflight_only=False,
                package_dir='virtual-test-package',
                clock=clock, wait=clock.wait)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['stop_confirmed'])
        front_stops = [frame.destination for _, frame, _ in buses['front'].frames if frame.kind == 4]
        for mid in BUS_IDS['front']:
            self.assertGreaterEqual(front_stops.count(mid), 2)
        self.assertFalse(buses['front'].enabled)
        self.assertFalse(buses['rear'].enabled)

    def test_final_stop_failure_cannot_report_success(self):
        result, buses = self.run_trial(active=True, failure='final_stop')
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['stop_confirmed'])
        self.assert_all_stopped(buses)

    def test_old_boot_or_clearance_failure_rejected_without_io(self):
        for field, changed in [('boot_id', 'prior-boot'),
                               ('front_upper_leg_carbon_clamp_clearance_verified', False),
                               ('joint_limits_all_samples_verified', False),
                               ('physical_route_all_samples_verified', False)]:
            review = reviewed()
            review[field] = changed
            clock = WorkerClock()
            buses = {name: FakeBus(name, clock) for name in BUS_IDS}
            with boot_file(), self.assertRaises(ValueError):
                stance.run_fixed_stance_trial(buses, UIDS, lambda: None, lambda _: None,
                                               validated_review=review, preflight_only=False,
                                               clock=clock, wait=clock.wait)
            self.assertTrue(all(not bus.calls for bus in buses.values()), field)


if __name__ == '__main__':
    unittest.main()
