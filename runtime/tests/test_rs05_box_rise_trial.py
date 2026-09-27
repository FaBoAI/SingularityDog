"""Virtual two-bus fault injection for the gated 2 mm box-rise runner."""

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_box_rise_trial as rise
from singularitydog_hw.box_rise_candidate import (
    SCHEMA, CANDIDATE_SCHEMA, REQUIRED_PHYSICAL_REVIEWS, PERIOD_S, TICKS,
)
from singularitydog_hw import box_rise_candidate as validator
from test_current_hold_review import BOOT_ID
from test_rs05_fullbody_hold import BUS_IDS, FakeBus, UIDS, WorkerClock, boot_file


MOVING = (1, 2, 4, 5, 7, 8, 10, 11)


def reviewed():
    starts = {str(i): .5 + .1*i for i in range(1, 13)}
    identities = {str(i): UIDS[i] for i in range(1, 13)}
    samples = []
    for tick in range(TICKS):
        half = (TICKS-1)//2
        progress = (tick if tick <= half else TICKS-1-tick)/half
        fraction = progress**3*(10 + progress*(-15 + 6*progress))
        samples.append({'tick': tick, 'elapsed_s': tick*PERIOD_S,
                        'body_rise_mm': 2.*fraction,
                        'raw_rad_by_id': {str(i): starts[str(i)]
                                          + (.02*fraction if i in MOVING else 0.)
                                          for i in range(1, 13)}})
    candidate = {
        'schema': CANDIDATE_SCHEMA, 'status': 'OFFLINE_GEOMETRY_CANDIDATE_ONLY',
        'boot_id': BOOT_ID, 'motor_uids': identities.copy(),
        'floor_snapshot_sha256': 'a'*64,
        'camera_l_sha256_by_leg': {leg: 'a'*64 for leg in ('FR', 'FL', 'RR', 'RL')},
        'requested_body_rise_mm': 2., 'sample_period_s': PERIOD_S,
        'sample_count': TICKS, 'samples': samples,
        'fixed_hip': True, 'hip_raw_targets_equal_fresh_start': True,
        'hip_lateral_paw_drift_bound_mm': .61,
        'motor_output_allowed': False, 'live_runner_available': False,
        'load_transfer_verified': False, 'angle_wrapping_applied': False,
    }
    return {
        'schema': SCHEMA, 'boot_id': BOOT_ID, 'motor_uids': identities,
        'candidate_sha256': 'b'*64, 'floor_snapshot_sha256': 'a'*64,
        'd17_urdf_sha256': 'c'*64, 'angle_review_sha256': 'd'*64,
        'physical_review_sha256': 'e'*64, 'runner_sha256': 'f'*64,
        'validator_sha256': '1'*64,
        'source_files_verified': True, 'box_rise_authorized': True,
        'automatic_retry_allowed': False, 'learned_policy_allowed': False,
        'box_removal_allowed': False,
        'start_raw_rad_by_id': starts,
        'reviewed_raw_corridor_by_id': {
            str(i): {'min_rad': starts[str(i)]-math.radians(1.),
                     'max_rad': starts[str(i)]+math.radians(3.)}
            for i in range(1, 13)},
        'candidate': candidate,
        **{flag: True for flag in REQUIRED_PHYSICAL_REVIEWS},
    }


class BoxRiseTrialTests(unittest.TestCase):
    def run_trial(self, *, active=False, failure=None, failure_bus='front', review=None,
                  bus_factory=FakeBus):
        clock = WorkerClock()
        buses = {name: bus_factory(name, clock, failure=failure if name == failure_bus else None)
                 for name in BUS_IDS}
        with (boot_file(), patch.object(rise.threading, 'Barrier', side_effect=clock.barrier),
              patch.object(rise, 'LIVE_OUTPUT_ENABLED', active),
              patch.object(rise, 'verify_package_files', return_value={'virtual': True})):
            result = rise.run_box_rise_trial(
                buses, UIDS, lambda: None, lambda _: None,
                validated_review=reviewed() if review is None else review,
                preflight_only=not active,
                package_dir='virtual-box-rise' if active else None,
                clock=clock, wait=clock.wait)
        return result, buses

    def assert_all_stopped(self, buses):
        for name, bus in buses.items():
            self.assertTrue(bus.stop_calls, name)
            self.assertEqual(bus.stop_calls[-1][0], BUS_IDS[name])
            self.assertFalse(bus.enabled)
            self.assertEqual(len(bus.owner_threads), 1)

    def test_closed_source_gate_refuses_active_before_bus_io(self):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock) for name in BUS_IDS}
        with boot_file(), self.assertRaisesRegex(ValueError, 'Live box-rise output is disabled'):
            rise.run_box_rise_trial(buses, UIDS, lambda: None, lambda _: None,
                                    validated_review=reviewed(), preflight_only=False,
                                    package_dir='virtual-box-rise', clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_disabled_preflight_never_enables_and_all_stops(self):
        result, buses = self.run_trial()
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        for bus in buses.values():
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in bus.frames))
        self.assert_all_stopped(buses)

    def test_virtual_two_mm_rise_return_and_all_stop(self):
        result, buses = self.run_trial(active=True)
        self.assertEqual(result['status'],
                         'BOX_SUPPORTED_2MM_RISE_RETURN_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['active_ticks'], TICKS + rise.END_HOLD_TICKS)
        self.assertTrue(result['stop_confirmed'])
        for bus in buses.values():
            self.assertEqual(result['workers'][bus.bus_name]['cycle_count'],
                             TICKS + rise.END_HOLD_TICKS)
            self.assertEqual(len(result['workers'][bus.bus_name]['electrical_samples']),
                             TICKS + rise.END_HOLD_TICKS)
        self.assert_all_stopped(buses)

    def test_missing_fault_and_write_failure_abort_with_all_stop(self):
        for failure in ('missing', 'fault', 'write'):
            with self.subTest(failure=failure):
                result, buses = self.run_trial(active=True, failure=failure)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertFalse(result['motion_completed'])
                self.assert_all_stopped(buses)

    def test_late_cycle_aborts_and_all_stops(self):
        class LateBus(FakeBus):
            def feedback_many(self, wires, expected_ids):
                found = super().feedback_many(wires, expected_ids)
                if self.bus_name == 'front' and self.active_batches == 3:
                    self.clock.wait(.09)
                return found

        result, buses = self.run_trial(active=True, bus_factory=LateBus)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('missed80ms' in error for error in result['errors']))
        self.assert_all_stopped(buses)

    def test_stale_feedback_aborts_and_all_stops(self):
        class StaleBus(FakeBus):
            def feedback_many(self, wires, expected_ids):
                found = super().feedback_many(wires, expected_ids)
                if self.bus_name == 'front' and self.active_batches >= 3 and self.enabled:
                    return {mid: (value, received-.2) for mid, (value, received) in found.items()}
                return found

        result, buses = self.run_trial(active=True, bus_factory=StaleBus)
        self.assertEqual(result['status'], 'ABORTED')
        self.assert_all_stopped(buses)

    def test_target_and_wire_mismatch_fail_without_output(self):
        review = reviewed()
        review['candidate']['samples'][20]['raw_rad_by_id']['1'] += math.radians(2.)
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock) for name in BUS_IDS}
        with (boot_file(), patch.object(rise, 'LIVE_OUTPUT_ENABLED', True),
              patch.object(rise, 'verify_package_files', return_value={'virtual': True}),
              self.assertRaisesRegex(ValueError, 'sample step')):
            rise.run_box_rise_trial(buses, UIDS, lambda: None, lambda _: None,
                                    validated_review=review, preflight_only=False,
                                    package_dir='virtual-box-rise', clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_unexpected_active_wire_aborts_and_all_stops(self):
        class AlterWireBus(FakeBus):
            def feedback_many(self, commands, expected_ids):
                if self.bus_name == 'front' and self.active_batches >= 2 and self.enabled:
                    commands = list(commands)
                    commands[0] = commands[1]
                return super().feedback_many(commands, expected_ids)

        result, buses = self.run_trial(active=True, bus_factory=AlterWireBus)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('Unexpected raw step Type1 wire' in error
                            for error in result['errors']))
        self.assert_all_stopped(buses)

    def test_tracking_error_aborts_and_all_stops(self):
        class PositionErrorBus(FakeBus):
            def value(self, mid):
                value = super().value(mid)
                if self.bus_name == 'front' and mid == 1 and self.active_batches >= 3:
                    return replace(value, protocol_position_rad=(
                        self.commanded[mid] + math.radians(3.)))
                return value

        result, buses = self.run_trial(active=True, bus_factory=PositionErrorBus)
        self.assertEqual(result['status'], 'ABORTED')
        self.assert_all_stopped(buses)

    def test_active_voltage_sag_or_watchdog_change_aborts_and_stops(self):
        class ElectricalBus(FakeBus):
            injected_parameter = 'voltage'

            def parameter(self, mid, name=None):
                answer = super().parameter(mid, name)
                if (self.bus_name == 'front' and self.active_batches >= 3
                        and self.enabled and name == self.injected_parameter):
                    return {'value': 34.9 if name == 'voltage' else 0, 'ok': True}
                return answer

        for parameter, phrase in (('voltage', 'voltage envelope'),
                                  ('can_timeout', 'watchdog readback')):
            with self.subTest(parameter=parameter):
                class InjectedBus(ElectricalBus):
                    injected_parameter = parameter

                result, buses = self.run_trial(active=True, bus_factory=InjectedBus)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertTrue(any(phrase in error for error in result['errors']))
                self.assert_all_stopped(buses)

    def test_final_stop_not_confirmed_cannot_report_success(self):
        result, buses = self.run_trial(active=True, failure='final_stop')
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['motion_completed'])
        self.assertFalse(result['stop_confirmed'])
        self.assert_all_stopped(buses)

    def test_hip_target_change_rejected_before_bus_io(self):
        review = reviewed()
        review['candidate']['samples'][50]['raw_rad_by_id']['3'] += math.radians(.1)
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock) for name in BUS_IDS}
        with (boot_file(), patch.object(rise, 'LIVE_OUTPUT_ENABLED', True),
              patch.object(rise, 'verify_package_files', return_value={'virtual': True}),
              self.assertRaisesRegex(ValueError, 'hip target must stay')):
            rise.run_box_rise_trial(buses, UIDS, lambda: None, lambda _: None,
                                    validated_review=review, preflight_only=False,
                                    package_dir='virtual-box-rise', clock=clock, wait=clock.wait)
        self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_frozen_package_binds_runner_validator_and_source_bytes(self):
        review = reviewed()
        review['runner_sha256'] = hashlib.sha256(Path(rise.__file__).read_bytes()).hexdigest()
        review['validator_sha256'] = hashlib.sha256(Path(validator.__file__).read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)

            def write(name, content):
                (directory / name).write_bytes(content)
                return hashlib.sha256(content).hexdigest()

            review['candidate_sha256'] = write(
                'candidate.json', json.dumps(review['candidate']).encode())
            for filename, field in (
                ('floor-snapshot.json', 'floor_snapshot_sha256'),
                ('angle-review.json', 'angle_review_sha256'),
                ('physical-review.json', 'physical_review_sha256'),
                ('d17.urdf', 'd17_urdf_sha256'),
            ):
                review[field] = write(filename, field.encode())
            review['candidate']['floor_snapshot_sha256'] = review['floor_snapshot_sha256']
            for leg in ('FR', 'FL', 'RR', 'RL'):
                review['candidate']['camera_l_sha256_by_leg'][leg] = write(
                    f'{leg.lower()}-camera-l.json', leg.encode())
            review['candidate_sha256'] = write(
                'candidate.json', json.dumps(review['candidate']).encode())
            write('review.json', json.dumps(review).encode())
            self.assertEqual(validator.verify_package_files(directory, review)
                             ['candidate_sha256'], review['candidate_sha256'])
            (directory / 'physical-review.json').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'physical-review.json differs'):
                validator.verify_package_files(directory, review)


if __name__ == '__main__':
    unittest.main()
