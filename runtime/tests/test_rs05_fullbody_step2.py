"""Virtual-bus tests for the separate, supported raw two-degree trial."""
from dataclasses import replace
import math
import threading
from unittest.mock import patch
import unittest

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from singularitydog_hw import rs05_fullbody_step2 as step2
from test_current_hold_review import BOOT_ID
from test_rs05_fullbody_hold import BUS_IDS, FakeBus, UIDS, WorkerClock, boot_file


def reviewed_path(*, authorized=True, amplitude=2.):
    return {
        'schema': step2.REVIEW_SCHEMA, 'scope': step2.REVIEW_SCOPE,
        'motor_ids': list(step2.ALL_IDS),
        'motor_uids': {str(mid): uid for mid, uid in UIDS.items()},
        'firmware': '0.5.0.13', 'sha256': 'a'*64,
        'review_complete': True, 'source_files_verified': True,
        'gain_profile': step2.GAIN_PROFILE,
        'old_raw_target_reused': False,
        'calibration_verified': False, 'model_mapping_verified': False,
        'learned_policy_allowed': False, 'standing_allowed': False,
        'automatic_retry_allowed': False,
        'raw_direction_reviewed_for_diagnostic': True,
        'swept_clearance_verified': True, 'support_stand_verified': True,
        'feet_clear_verified': True, 'hands_clear_verified': True,
        'physical_cutoff_ready': True,
        'current_hold_status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
        'current_hold_stop_confirmed': True,
        'current_hold_summary_sha256': 'b'*64,
        'current_hold_gain_profile': step2.GAIN_PROFILE,
        'supported_step_authorized': authorized,
        'boot_id': BOOT_ID, 'current_hold_boot_id': BOOT_ID,
        'start_raw_rad_by_id': {str(i): .5+.1*i for i in step2.ALL_IDS},
        'raw_direction_by_id': {str(i): (1 if i % 2 else -1) for i in step2.ALL_IDS},
        'amplitude_deg': amplitude,
    }


def role_group_path(group='thigh', *, authorized=True):
    review = reviewed_path(authorized=authorized,
                           amplitude=10. if group in ('thigh', 'front-thigh') else
                                     5. if group == 'front-hip' else 1.)
    review.update(schema=step2.ROLE_GROUP_REVIEW_SCHEMA,
                  scope=(step2.FRONT_THIGH_REVIEW_SCOPE if group == 'front-thigh'
                         else step2.FRONT_HIP_REVIEW_SCOPE if group == 'front-hip'
                         else step2.ROLE_GROUP_REVIEW_SCOPE), role_group=group,
                  direction_profile='front-hip-mirrored' if group == 'front-hip' else 'raw-plus',
                  gain_profile=(step2.ROLE_FRONT_THIGH_GAIN_PROFILE if group == 'front-thigh'
                                else step2.ROLE_THIGH_GAIN_PROFILE if group == 'thigh'
                                else step2.ROLE_FRONT_HIP_GAIN_PROFILE if group == 'front-hip'
                                else step2.GAIN_PROFILE),
                  raw_direction_by_id=({str(i): (-1 if i == 2 else 1 if i == 5 else 0)
                                        for i in step2.ALL_IDS}
                                       if group == 'front-thigh' else
                                       {str(i): (1 if i == 3 else -1 if i == 6 else 0)
                                        for i in step2.ALL_IDS}
                                       if group == 'front-hip' else
                                       {str(i): (1 if i in step2.ROLE_GROUP_IDS[group] else 0)
                                        for i in step2.ALL_IDS}))
    return review


class RawStep2Tests(unittest.TestCase):
    def fixture(self, *, failure=None, failure_bus='front'):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock, failure=failure if name == failure_bus else None)
                 for name in BUS_IDS}
        return clock, buses

    def run_trial(self, clock, buses, *, review=None, preflight_only=False, events=None):
        review = reviewed_path() if review is None else review
        with boot_file(), patch.object(step2.threading, 'Barrier', side_effect=clock.barrier):
            return step2.run_fullbody_step2(buses, UIDS, lambda: None,
                                            events.append if events is not None else lambda _: None,
                                            validated_review=review, preflight_only=preflight_only,
                                            clock=clock, wait=clock.wait)

    def assert_stopped(self, buses):
        for name, bus in buses.items():
            self.assertTrue(bus.stop_calls, name)
            self.assertEqual(bus.stop_calls[-1][0], BUS_IDS[name])
            self.assertFalse(bus.enabled)
            self.assertEqual(len(bus.owner_threads), 1)

    def test_disabled_preflight_never_enables_or_sends_gain(self):
        clock, buses = self.fixture()
        review = reviewed_path(authorized=False)
        for flag in ('raw_direction_reviewed_for_diagnostic',
                     'swept_clearance_verified', 'support_stand_verified',
                     'feet_clear_verified', 'hands_clear_verified', 'physical_cutoff_ready'):
            review[flag] = False
        result = self.run_trial(clock, buses, review=review, preflight_only=True)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        for bus in buses.values():
            self.assertEqual(result['workers'][bus.bus_name]['cycle_count'], 20)
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in bus.frames))
            self.assertTrue(all(frame.data[4:8] == bytes(4) for _, frame, _ in bus.frames
                                if frame.kind == 1))
        self.assert_stopped(buses)

    def test_fixed_two_degree_quintic_sends_180_exact_frames_per_axis(self):
        clock, buses = self.fixture()
        review = reviewed_path()
        events = []
        result = self.run_trial(clock, buses, review=review, events=events)
        self.assertEqual(result['status'], 'RAW_STEP2_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['standing_allowed'])
        self.assertFalse(result['model_mapping_verified'])
        self.assertEqual(result['Kp_by_motor_id'], {i: 4. if i in (4, 10) else 3.
                                                     for i in step2.ALL_IDS})
        plan = step2._plan({i: .5+.1*i for i in step2.ALL_IDS}, review)
        self.assertEqual(len(plan), 180)
        self.assertEqual(plan[0], {i: .5+.1*i for i in step2.ALL_IDS})
        self.assertEqual(plan[-1], plan[160])
        self.assertTrue(all(abs(plan[-1][i]-plan[0][i]) <= math.radians(2.)+1e-12
                            for i in step2.ALL_IDS))
        for name, bus in buses.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 180)
            self.assertEqual(sum(frame.kind == 3 for _, frame, _ in bus.frames), 6)
            for index, (_, frame, _) in enumerate(bus.frames):
                if frame.kind == 3:
                    following = bus.frames[index + 1][1]
                    self.assertEqual(following.wire,
                                     motion_request(phase=TrialPhase.ZERO_GAIN,
                                                    center_rad=bus.centers[frame.destination],
                                                    motor_id=frame.destination))
            for mid in bus.ids:
                active = [frame.wire for _, frame, _ in bus.frames
                          if frame.kind == 1 and frame.destination == mid
                          and frame.data[4:8] != bytes(4)]
                phase = TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10) else TrialPhase.POSITION_STEP5
                expected = [motion_request(phase=phase, center_rad=bus.centers[mid],
                            offset_rad=plan[tick][mid]-bus.centers[mid], motor_id=mid)
                            for tick in range(180)]
                self.assertEqual(active, expected)
            self.assertAlmostEqual(bus.stop_calls[-1][1]
                                   - result['workers'][name]['start_monotonic_s'], 9.)
        self.assert_stopped(buses)

    def test_missing_enable_reply_can_be_confirmed_by_zero_gain_reply(self):
        clock = WorkerClock()

        class LostEnableReplyBus(FakeBus):
            def feedback_many(self, commands, expected_ids):
                commands = tuple(commands)
                first = ATParser().feed(commands[0])[0]
                if self.bus_name == 'front' and first.kind == 3 and first.destination == 2:
                    # The adapter loses Type3's reply; Type1 still elicits a
                    # fresh mode2 reply from the same enabled actuator.
                    if len(commands) != 2 or ATParser().feed(commands[1])[0].kind != 1:
                        raise TimeoutError('Injected missing Enable reply')
                return super().feedback_many(commands, expected_ids)

        buses = {name: LostEnableReplyBus(name, clock) for name in BUS_IDS}
        result = self.run_trial(clock, buses, review=role_group_path('front-hip'))
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['workers']['front']['cycle_count'], 180)
        self.assert_stopped(buses)

    def test_tick_zero_uses_validated_replies_before_peer_batch_finishes(self):
        clock = WorkerClock()
        first_front_reply = threading.Event()
        rear_last_write = threading.Event()
        observed = {}

        class StartupTimingBus(FakeBus):
            def send(self, command):
                frame = ATParser().feed(command)[0]
                active = frame.kind == 1 and frame.data[4:8] != bytes(4)
                if (self.bus_name == 'rear' and active and self.active_batches == 0
                        and frame.destination == 12):
                    if not first_front_reply.wait(timeout=2):
                        raise AssertionError('Front reply did not arrive before rear ID12')
                    # As in the physical trace, the earliest Enable reply has
                    # aged out while a fresh front Type2 is still inside its
                    # unfinished six-frame receive batch.
                    clock.wait(max(0., observed['first_enable_at'] + .101 - clock()))
                    observed['old_reply_age'] = clock() - observed['first_enable_at']
                    self.feedback_guard(self.value(7), clock(), 7)
                    try:
                        super().send(command)
                    finally:
                        rear_last_write.set()
                else:
                    super().send(command)
                if frame.kind == 3:
                    clock.wait(.012)
                elif active and self.active_batches == 0:
                    clock.wait(.005)

            def feedback_many(self, commands, expected_ids):
                commands = tuple(commands)
                active = any(ATParser().feed(command)[0].kind == 1
                             and ATParser().feed(command)[0].data[4:8] != bytes(4)
                             for command in commands)
                if self.bus_name == 'front' and active and self.active_batches == 0:
                    original_guard = self.feedback_guard

                    def hold_after_first_reply(value, received, mid):
                        original_guard(value, received, mid)
                        if mid == 1:
                            first_front_reply.set()
                            if not rear_last_write.wait(timeout=2):
                                raise AssertionError('Rear ID12 was not attempted')

                    self.feedback_guard = hold_after_first_reply
                    try:
                        return super().feedback_many(commands, expected_ids)
                    finally:
                        self.feedback_guard = original_guard
                found = super().feedback_many(commands, expected_ids)
                if (self.bus_name == 'front' and expected_ids == (1,)
                        and ATParser().feed(commands[0])[0].kind == 3):
                    observed['first_enable_at'] = found[1][1]
                return found

        buses = {name: StartupTimingBus(name, clock) for name in BUS_IDS}
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'RAW_STEP2_COMPLETED_RESET_CONFIRMED',
                         (result['errors'], {k:(v['stage'],v['errors']) for k,v in result['workers'].items()}))
        self.assertGreater(observed['old_reply_age'], .1)
        self.assertEqual(result['workers']['rear']['cycle_count'], 180)
        self.assert_stopped(buses)

    def test_first_active_batch_allows_120ms_but_rejects_over_125ms(self):
        for send_interval, expected_status in ((.006, 'RAW_STEP2_COMPLETED_RESET_CONFIRMED'),
                                               (.008, 'ABORTED')):
            clock = WorkerClock()
            first_active_barrier = threading.Barrier(2, action=clock.synchronize)

            class PacedStartupBus(FakeBus):
                def __init__(self, name):
                    super().__init__(name, clock)
                    self.startup_snapshot_ages = []

                def send(self, command):
                    frame = ATParser().feed(command)[0]
                    first_active = (frame.kind == 1 and frame.data[4:8] != bytes(4)
                                    and self.active_batches == 0)
                    if first_active:
                        self.startup_snapshot_ages.append(
                            clock() - min(received for _, received in self.latest.values()))
                    super().send(command)
                    if frame.kind == 3:
                        clock.wait(.018)
                    elif first_active:
                        clock.wait(send_interval)
                        first_active_barrier.wait(timeout=2)

            buses = {name: PacedStartupBus(name) for name in BUS_IDS}
            with self.subTest(send_interval=send_interval):
                result = self.run_trial(clock, buses)
                self.assertEqual(result['status'], expected_status,
                                 (result['errors'], {name: report['stage'] for name, report
                                                     in result['workers'].items()},
                                  {name: bus.startup_snapshot_ages for name, bus in buses.items()}))
                oldest = max(age for bus in buses.values()
                             for age in bus.startup_snapshot_ages)
                if expected_status == 'ABORTED':
                    self.assertGreater(oldest, .125)
                    self.assertTrue(any('Stale feedback' in error for error in result['errors']))
                else:
                    self.assertAlmostEqual(oldest, .12, places=6)
                self.assert_stopped(buses)

    def test_four_thigh_axes_move_together_while_other_eight_hold(self):
        clock, buses = self.fixture()
        review = role_group_path('thigh')
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['moving_motor_ids'], [2, 5, 8, 11])
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual({result['Kp_by_motor_id'][i] for i in (2, 5, 8, 11)}, {12.})
        self.assertEqual({result['Kp_by_motor_id'][i] for i in (3, 6, 9, 12)}, {12.})
        plan = step2._plan({i: .5+.1*i for i in step2.ALL_IDS}, review)
        for mid in step2.ALL_IDS:
            delta = plan[-1][mid] - plan[0][mid]
            self.assertAlmostEqual(delta, math.radians(10.) if mid in (2, 5, 8, 11) else 0.)
        self.assert_stopped(buses)

    def test_front_thigh_two_axes_move_toward_face_while_other_ten_hold(self):
        clock, buses = self.fixture()
        review = role_group_path('front-thigh')
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['moving_motor_ids'], [2, 5])
        self.assertEqual(result['gain_profile'], step2.ROLE_FRONT_THIGH_GAIN_PROFILE)
        self.assertEqual(result['Kp_by_motor_id'], {
            mid: (12. if mid in (2, 3, 5, 6, 9, 12) else
                  4. if mid in (4, 10) else 3.) for mid in step2.ALL_IDS})
        self.assertEqual(step2._torque_limit(review, 2), 1.8)
        self.assertEqual(step2._torque_limit(review, 5), 1.8)
        self.assertEqual(step2._torque_limit(review, 8), .8)
        self.assertEqual(step2._torque_limit(review, 11), .8)
        self.assertEqual(step2._limits(review, 2),
                         (step2.ROLE_THIGH_TRACKING_ERROR_RAD,
                          step2.ROLE_THIGH_EXCURSION_RAD,
                          step2.ROLE_THIGH_FINAL_ERROR_RAD))
        self.assertEqual(step2._limits(review, 8)[0], step2.MAX_TRACKING_ERROR_RAD)
        self.assertEqual(step2._drift_limit(review, 11), step2.MAX_DRIFT_RAD)
        centers = {i: .5+.1*i for i in step2.ALL_IDS}
        plan = step2._plan(centers, review)
        self.assertEqual(len(plan), 180)
        for mid in step2.ALL_IDS:
            self.assertAlmostEqual(plan[-1][mid] - plan[0][mid],
                                   math.radians(-10. if mid == 2 else 10. if mid == 5 else 0.))
        for name, bus in buses.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 180)
            for mid in bus.ids:
                active = [frame.wire for _, frame, _ in bus.frames
                          if frame.kind == 1 and frame.destination == mid
                          and frame.data[4:8] != bytes(4)]
                expected = [motion_request(phase=step2._motion_phase(review, mid),
                            center_rad=bus.centers[mid],
                            offset_rad=plan[tick][mid]-bus.centers[mid], motor_id=mid)
                            for tick in range(180)]
                self.assertEqual(active, expected)
        self.assert_stopped(buses)

    def test_front_thigh_scope_rejects_wrong_map_and_scope_before_io(self):
        for change in ('extra_rear', 'missing_front', 'reverse_front', 'bool_sign',
                       'old_scope', 'over_ten'):
            clock, buses = self.fixture()
            review = role_group_path('front-thigh')
            if change == 'extra_rear':
                review['raw_direction_by_id']['8'] = 1
            elif change == 'missing_front':
                review['raw_direction_by_id']['5'] = 0
            elif change == 'reverse_front':
                review['raw_direction_by_id']['2'] = 1
            elif change == 'bool_sign':
                review['raw_direction_by_id']['5'] = True
            elif change == 'old_scope':
                review['scope'] = step2.ROLE_GROUP_REVIEW_SCOPE
            elif change == 'over_ten':
                review['amplitude_deg'] = 10.01
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))

        clock, buses = self.fixture()
        review = role_group_path('front-thigh')
        review['start_raw_rad_by_id']['2'] += 2*math.pi
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(any(frame.kind == 3 for bus in buses.values()
                             for _, frame, _ in bus.frames))
        self.assert_stopped(buses)

    def test_front_hip_pair_five_degrees_other_ten_hold_and_all_stop(self):
        clock, buses = self.fixture()
        review = role_group_path('front-hip')
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['moving_motor_ids'], [3, 6])
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['standing_allowed'])
        self.assertFalse(result['learned_policy_allowed'])
        self.assertEqual(result['gain_profile'], step2.ROLE_FRONT_HIP_GAIN_PROFILE)
        self.assertEqual(result['Kp_by_motor_id'], {
            mid: (12. if mid in (3, 6) else 4. if mid in (4, 10) else 3.)
            for mid in step2.ALL_IDS})
        self.assertEqual({step2._motion_phase(review, mid) for mid in (3, 6)},
                         {TrialPhase.POSITION_ROLE_FRONT_HIP_KP12})
        self.assertEqual(step2._torque_limit(review, 3), .8)
        self.assertEqual(step2._speed_limit(review), .25)
        self.assertEqual(step2._limits(review, 3)[0], math.radians(2.))
        self.assertEqual(step2._limits(review, 3),
                         (step2.MAX_TRACKING_ERROR_RAD, step2.FRONT_HIP_MAX_EXCURSION_RAD,
                          step2.FRONT_HIP_FINAL_ERROR_RAD))
        centers = {i: .5+.1*i for i in step2.ALL_IDS}
        plan = step2._plan(centers, review)
        self.assertEqual(len(plan), 180)
        for mid in step2.ALL_IDS:
            self.assertAlmostEqual(plan[-1][mid] - plan[0][mid],
                                   math.radians(5. if mid == 3 else -5. if mid == 6 else 0.))
        for name, bus in buses.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 180)
            for mid in bus.ids:
                active = [frame.wire for _, frame, _ in bus.frames
                          if frame.kind == 1 and frame.destination == mid
                          and frame.data[4:8] != bytes(4)]
                expected = [motion_request(phase=step2._motion_phase(review, mid),
                            center_rad=bus.centers[mid],
                            offset_rad=plan[tick][mid]-bus.centers[mid], motor_id=mid)
                            for tick in range(180)]
                self.assertEqual(active, expected)
        self.assert_stopped(buses)

    def test_front_hip_ten_degree_single_trajectory(self):
        review = role_group_path('front-hip')
        review['amplitude_deg'] = 10.
        centers = {i: .5 + .1*i for i in step2.ALL_IDS}
        plan = step2._plan(centers, review)
        self.assertEqual(len(plan), 180)
        self.assertEqual(step2._motion_phase(review, 3),
                         TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10)
        self.assertEqual(step2._limits(review, 3)[1],
                         step2.FRONT_HIP_STEP10_MAX_EXCURSION_RAD)
        self.assertEqual(step2._drift_limit(review, 3),
                         step2.FRONT_HIP_STEP10_MAX_EXCURSION_RAD)
        self.assertEqual(step2._drift_limit(review, 6),
                         step2.FRONT_HIP_STEP10_MAX_EXCURSION_RAD)
        self.assertEqual(step2._drift_limit(review, 2), step2.MAX_DRIFT_RAD)
        self.assertAlmostEqual(plan[-1][3]-centers[3], math.radians(10.), places=8)
        self.assertAlmostEqual(plan[-1][6]-centers[6], -math.radians(10.), places=8)
        for mid in (3, 6):
            motion_request(phase=step2._motion_phase(review, mid),
                           center_rad=centers[mid], offset_rad=plan[-1][mid]-centers[mid],
                           motor_id=mid)

    def test_front_hip_held_id9_stable_bias_passes_endpoint_only(self):
        review = role_group_path('front-hip')
        self.assertEqual(step2._limits(review, 9),
                         (step2.MAX_TRACKING_ERROR_RAD, step2.MAX_EXCURSION_RAD,
                          math.radians(1.5)))
        self.assertEqual(step2._torque_limit(review, 9), step2.MAX_FEEDBACK_TORQUE_NM)
        self.assertEqual(step2._drift_limit(review, 9), step2.MAX_DRIFT_RAD)
        self.assertEqual(step2._limits(role_group_path('toe'), 9)[2],
                         step2.ARRIVAL_ERROR_CANDIDATE_RAD)

        clock, buses = self.fixture(failure='id9_stable_bias', failure_bus='rear')
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['workers']['rear']['cycle_count'], 180)
        samples = result['workers']['rear']['final_hold_samples'][9]
        self.assertEqual(len(samples), 20)
        for _, position in samples:
            self.assertAlmostEqual(math.degrees(position - buses['rear'].centers[9]), -1.04)
        self.assertTrue(result['stop_confirmed'])
        self.assert_stopped(buses)

    def test_front_hip_held_id9_large_bias_and_id10_old_gate_still_abort(self):
        for failure, mid in (('id9_large_bias', 9), ('final_error', 10)):
            clock, buses = self.fixture(failure=failure, failure_bus='rear')
            with self.subTest(failure=failure):
                result = self.run_trial(clock, buses, review=role_group_path('front-hip'))
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertTrue(any(f'ID{mid} final raw step error exceeds diagnostic limit' in err
                                    for err in result['errors']))
                self.assertEqual(result['workers']['rear']['cycle_count'], 180)
                self.assertTrue(result['stop_confirmed'])
                self.assert_stopped(buses)

    def test_front_hip_preflight_and_bad_profile_rejected_before_io(self):
        clock, buses = self.fixture()
        review = role_group_path('front-hip', authorized=False)
        for flag in ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
                     'support_stand_verified', 'feet_clear_verified',
                     'hands_clear_verified', 'physical_cutoff_ready'):
            review[flag] = False
        result = self.run_trial(clock, buses, review=review, preflight_only=True)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertFalse(any(frame.kind == 3 for bus in buses.values()
                             for _, frame, _ in bus.frames))
        self.assert_stopped(buses)
        for change in ('reverse_id3', 'reverse_id6', 'extra_id9', 'wrong_scope',
                       'wrong_profile', 'over_five', 'old_gain_profile', 'standing_flag'):
            clock, buses = self.fixture()
            review = role_group_path('front-hip')
            if change == 'reverse_id3':
                review['raw_direction_by_id']['3'] = -1
            elif change == 'reverse_id6':
                review['raw_direction_by_id']['6'] = 1
            elif change == 'extra_id9':
                review['raw_direction_by_id']['9'] = 1
            elif change == 'wrong_scope':
                review['scope'] = step2.ROLE_GROUP_REVIEW_SCOPE
            elif change == 'wrong_profile':
                review['direction_profile'] = 'raw-plus'
            elif change == 'over_five':
                review['amplitude_deg'] = 5.01
            elif change == 'old_gain_profile':
                review['gain_profile'] = step2.GAIN_PROFILE
            else:
                review['standing_allowed'] = True
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_role_thigh_id4_held_endpoint_matches_reviewed_dual_kp4_hold(self):
        review = role_group_path('thigh')
        self.assertEqual(step2._limits(review, 4),
                         (step2.MAX_TRACKING_ERROR_RAD, step2.MAX_EXCURSION_RAD,
                          math.radians(1.5)))
        id7_review = reviewed_path()
        id7_review.update(schema=step2.ID7_REVIEW_SCHEMA, scope=step2.ID7_REVIEW_SCOPE)
        for other in (reviewed_path(), id7_review, role_group_path('hip')):
            self.assertEqual(step2._limits(other, 4)[2], step2.ARRIVAL_ERROR_CANDIDATE_RAD)
        self.assertEqual(step2._limits(role_group_path('toe'), 4)[2],
                         step2.ID7_FINAL_ERROR_RAD)
        wrong_profile = role_group_path('thigh')
        wrong_profile['gain_profile'] = step2.GAIN_PROFILE
        self.assertEqual(step2._limits(wrong_profile, 4)[2],
                         step2.ARRIVAL_ERROR_CANDIDATE_RAD)
        self.assertEqual(step2._limits(review, 10)[2], step2.ARRIVAL_ERROR_CANDIDATE_RAD)

        for failure, expected_status in (
                ('id4_stable_bias', 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED'),
                ('id4_large_bias', 'ABORTED')):
            clock, buses = self.fixture(failure=failure)
            with self.subTest(failure=failure):
                result = self.run_trial(clock, buses, review=review)
                self.assertEqual(result['status'], expected_status, result['errors'])
                self.assertTrue(result['stop_confirmed'])
                if failure == 'id4_large_bias':
                    self.assertTrue(any('ID4 final raw step error exceeds diagnostic limit' in err
                                        for err in result['errors']))
                self.assert_stopped(buses)

    def test_mirrored_thigh_directions_reach_opposite_raw_targets(self):
        clock, buses = self.fixture()
        review = role_group_path('thigh')
        for mid in (5, 11):
            review['raw_direction_by_id'][str(mid)] = -1
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertTrue(result['stop_confirmed'])
        plan = step2._plan({i: .5+.1*i for i in step2.ALL_IDS}, review)
        self.assertEqual([round(math.degrees(plan[-1][i] - plan[0][i]))
                          for i in (2, 5, 8, 11)], [10, -10, 10, -10])
        self.assertTrue(all(plan[-1][i] == plan[0][i]
                            for i in step2.ALL_IDS if i not in (2, 5, 8, 11)))
        self.assert_stopped(buses)

    def test_four_axis_scope_rejects_fifth_axis_or_more_than_ten_degrees(self):
        for change in ('fifth_axis', 'missing_axis', 'unknown_group', 'too_large'):
            clock, buses = self.fixture()
            review = role_group_path('thigh')
            if change == 'fifth_axis':
                review['raw_direction_by_id']['7'] = 1
            elif change == 'missing_axis':
                review['raw_direction_by_id']['8'] = 0
            elif change == 'unknown_group':
                review['role_group'] = 'all'
            else:
                review['amplitude_deg'] = 10.01
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertFalse(any(bus.calls for bus in buses.values()))

    def test_four_axis_plan_rebases_after_sag_but_rejects_over_sixty_degrees(self):
        review = role_group_path('thigh')
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in step2.ALL_IDS}
        centers[8] -= math.radians(49.)
        plan = step2._plan(centers, review)
        self.assertAlmostEqual(plan[0][8], centers[8])
        self.assertAlmostEqual(plan[-1][8], centers[8] + math.radians(10.))
        centers[8] = review['start_raw_rad_by_id']['8'] - math.radians(60.01)
        with self.assertRaises(ValueError):
            step2._plan(centers, review)

    def test_review_denies_unproved_direction_clearance_and_authorization_before_io(self):
        for field in ('raw_direction_reviewed_for_diagnostic',
                      'swept_clearance_verified', 'support_stand_verified',
                      'feet_clear_verified', 'hands_clear_verified', 'physical_cutoff_ready',
                      'current_hold_stop_confirmed', 'supported_step_authorized'):
            clock, buses = self.fixture()
            review = reviewed_path(); review[field] = False
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))
        for field in ('calibration_verified', 'model_mapping_verified',
                      'learned_policy_allowed', 'standing_allowed'):
            clock, buses = self.fixture()
            review = reviewed_path(); review[field] = True
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))
        clock, buses = self.fixture()
        review = reviewed_path(); review['old_raw_target_reused'] = True
        with self.assertRaises(ValueError):
            self.run_trial(clock, buses, review=review)
        self.assertTrue(all(not bus.calls for bus in buses.values()))
        for bad_boot in ('old-boot', ''):
            clock, buses = self.fixture()
            review = reviewed_path(); review['current_hold_boot_id'] = bad_boot
            with self.subTest(bad_boot=bad_boot), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_malformed_stale_or_over_two_degree_path_rejected_before_enable(self):
        for change in ('missing_direction', 'wrapped_start', 'stale_fresh_start',
                       'over_two', 'bool_sign', 'headroom'):
            clock, buses = self.fixture()
            review = reviewed_path()
            if change == 'missing_direction':
                del review['raw_direction_by_id']['9']
            elif change == 'wrapped_start':
                review['start_raw_rad_by_id']['9'] += 2*math.pi
            elif change == 'stale_fresh_start':
                buses['rear'].centers[9] += math.radians(1.)
            elif change == 'over_two':
                review['amplitude_deg'] = 2.01
            elif change == 'bool_sign':
                review['raw_direction_by_id']['9'] = True
            elif change == 'headroom':
                review['start_raw_rad_by_id']['9'] = 12.55
                buses['rear'].centers[9] = 12.55
            with self.subTest(change=change):
                if change in ('stale_fresh_start', 'wrapped_start'):
                    result = self.run_trial(clock, buses, review=review)
                    self.assertEqual(result['status'], 'ABORTED', result['errors'])
                    self.assertFalse(any(frame.kind == 3 for bus in buses.values()
                                         for _, frame, _ in bus.frames))
                    self.assert_stopped(buses)
                else:
                    with self.assertRaises(ValueError):
                        self.run_trial(clock, buses, review=review)
                    self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_fault_missing_write_stale_deadline_and_stop_failures_abort_both_buses(self):
        for name in BUS_IDS:
            for failure in ('fault', 'missing', 'write', 'deadline', 'stale', 'final_stop'):
                clock, buses = self.fixture(failure=failure, failure_bus=name)
                with self.subTest(bus=name, failure=failure):
                    result = self.run_trial(clock, buses)
                    self.assertEqual(result['status'], 'ABORTED', result['errors'])
                    self.assertFalse(result['motion_completed'])
                    self.assertEqual(result['stop_confirmed'], failure != 'final_stop')
                    if failure != 'final_stop':
                        self.assert_stopped(buses)
                    else:
                        self.assertTrue(all(bus.stop_calls for bus in buses.values()))

    def test_initial_identity_or_voltage_failure_aborts_before_enable(self):
        for failure in ('uid', 'initial_stop', 'run_mode', 'voltage', 'watchdog_readback'):
            clock, buses = self.fixture(failure=failure)
            with self.subTest(failure=failure):
                result = self.run_trial(clock, buses)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertFalse(any(frame.kind == 3 for bus in buses.values()
                                     for _, frame, _ in bus.frames))
                self.assert_stopped(buses)

    def test_changed_active_wire_is_blocked_before_uart_write(self):
        clock, buses = self.fixture()
        bus = buses['front']
        original = bus.feedback_many
        changed = []

        def tamper(commands, ids):
            commands = list(commands)
            for index, command in enumerate(commands):
                frame = ATParser().feed(command)[0]
                if (not changed and frame.kind == 1 and frame.data[4:8] != bytes(4)):
                    mid = frame.destination
                    changed.append(mid)
                    commands[index] = motion_request(phase=TrialPhase.POSITION_STEP5,
                                                     center_rad=bus.centers[mid],
                                                     offset_rad=math.radians(1.), motor_id=mid)
                    break
            return original(commands, ids)

        bus.feedback_many = tamper
        result = self.run_trial(clock, buses)
        self.assertEqual(changed, [1])
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(any('Unexpected raw step Type1 wire' in err for err in result['errors']))
        self.assertFalse(any(frame.kind == 1 and frame.data[4:8] != bytes(4)
                             for _, frame, _ in bus.frames))
        self.assert_stopped(buses)

    def test_final_tracking_error_aborts_and_stops(self):
        clock, buses = self.fixture(failure='final_error', failure_bus='rear')
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(any('raw step tracking error' in err or
                            'final raw step error exceeds1-degree' in err for err in result['errors']))
        self.assert_stopped(buses)


if __name__ == '__main__':
    unittest.main()
