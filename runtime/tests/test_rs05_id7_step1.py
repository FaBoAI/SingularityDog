"""Virtual, no-device checks for the ID7-only one-degree diagnostic."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_fullbody_step2 as core
from singularitydog_hw.rs05_id7_step1 import run_id7_step1
from singularitydog_hw.rs05_step2_packet_gate import (ID7Step1PacketState,
                                                      Step2PacketPort)
from singularitydog_hw.rs05_trial_protocol import (TrialPhase, enable_request,
                                                  motion_request, stop_request)
from test_rs05_fullbody_hold import BUS_IDS, FakeBus, UIDS, WorkerClock, boot_file
from test_rs05_fullbody_step2 import reviewed_path
from test_rs05_step2_packet_gate import FakeSerial


def id7_review(*, authorized=True, amplitude=1.):
    review = reviewed_path(authorized=authorized, amplitude=amplitude)
    review.update(schema=core.ID7_REVIEW_SCHEMA, scope=core.ID7_REVIEW_SCOPE,
                  raw_direction_by_id={str(i): (1 if i == 7 else 0)
                                       for i in core.ALL_IDS},
                  source_active_step2_abort_sha256='c'*64,
                  id7_joint_inspection_verified=True)
    return review


class ID7Step1Tests(unittest.TestCase):
    def fixture(self, *, failure=None, failure_bus='rear'):
        clock = WorkerClock()
        buses = {name: FakeBus(name, clock, failure=failure if name == failure_bus else None)
                 for name in BUS_IDS}
        return clock, buses

    def run_trial(self, clock, buses, review=None):
        review = id7_review() if review is None else review
        with boot_file(), patch.object(core.threading, 'Barrier', side_effect=clock.barrier):
            return run_id7_step1(buses, UIDS, lambda: None, lambda _: None,
                                 validated_review=review, preflight_only=False,
                                 clock=clock, wait=clock.wait)

    def assert_stopped(self, buses):
        for name, bus in buses.items():
            self.assertEqual(bus.stop_calls[-1][0], BUS_IDS[name])
            self.assertFalse(bus.enabled)

    def test_only_id7_moves_and_all_other_wires_hold_fresh_centers(self):
        centers = {i: .5+.1*i for i in core.ALL_IDS}
        review = id7_review()
        plan = core._plan(centers, review)
        self.assertEqual(len(plan), 180)
        self.assertEqual(plan[0], centers)
        self.assertAlmostEqual(plan[-1][7]-centers[7], math.radians(1.))
        for tick in range(180):
            for mid in core.ALL_IDS:
                if mid != 7:
                    self.assertEqual(plan[tick][mid], centers[mid])
        clock, buses = self.fixture()
        result = self.run_trial(clock, buses, review)
        self.assertEqual(result['status'], 'RAW_ID7_STEP1_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['moving_motor_ids'], [7])
        for name, bus in buses.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 180)
            for mid in bus.ids:
                active = [frame.wire for _, frame, _ in bus.frames if frame.kind == 1
                          and frame.destination == mid and frame.data[4:8] != bytes(4)]
                phase = TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10) else TrialPhase.POSITION_STEP5
                expected = [motion_request(phase=phase, center_rad=centers[mid],
                            offset_rad=plan[tick][mid]-centers[mid], motor_id=mid)
                            for tick in range(180)]
                self.assertEqual(active, expected)
        self.assert_stopped(buses)

    def test_id7_can_arrive_during_the_full_endpoint_hold(self):
        clock, buses = self.fixture()
        rear = buses['rear']
        original = rear.value

        def delayed_position(mid):
            feedback = original(mid)
            if mid == 7 and mid in rear.enabled:
                hold_samples = max(0, rear.active_batches - core.RAMP_TICKS)
                return replace(feedback, protocol_position_rad=
                               rear.centers[mid] + math.radians(hold_samples / 20.))
            return feedback

        rear.value = delayed_position
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'RAW_ID7_STEP1_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['workers']['rear']['cycle_count'], core.ACTIVE_TICKS)
        samples = result['workers']['rear']['final_hold_samples'][7]
        self.assertEqual(len(samples), core.END_HOLD_TICKS)
        self.assertGreater(math.radians(1.) - (samples[0][1] - rear.centers[7]),
                           core.ID7_FINAL_ERROR_RAD)
        self.assertAlmostEqual(samples[-1][1] - rear.centers[7], math.radians(1.))
        self.assert_stopped(buses)

    def test_id7_still_fails_final_arrival_after_all_hold_replies(self):
        clock, buses = self.fixture()
        rear = buses['rear']
        original = rear.value

        def stationary_position(mid):
            feedback = original(mid)
            if mid == 7 and mid in rear.enabled:
                return replace(feedback, protocol_position_rad=rear.centers[mid])
            return feedback

        rear.value = stationary_position
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID7 final raw step error exceeds diagnostic limit' in err
                            for err in result['errors']))
        self.assertEqual(result['workers']['rear']['cycle_count'], core.ACTIVE_TICKS)
        self.assertEqual(len(result['workers']['rear']['final_hold_samples'][7]),
                         core.END_HOLD_TICKS)
        self.assertTrue(result['stop_confirmed'])
        self.assert_stopped(buses)

    def test_id7_speed_guard_remains_active_during_endpoint_hold(self):
        clock, buses = self.fixture()
        rear = buses['rear']
        original = rear.value

        def fast_during_hold(mid):
            feedback = original(mid)
            if mid == 7 and rear.active_batches >= core.RAMP_TICKS + 3:
                return replace(feedback, velocity_rad_s=.201)
            return feedback

        rear.value = fast_during_hold
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID7 raw step feedback velocity' in err
                            for err in result['errors']))
        self.assertLess(result['workers']['rear']['cycle_count'], core.ACTIVE_TICKS)
        self.assertTrue(result['stop_confirmed'])
        self.assert_stopped(buses)

    def test_unapproved_disabled_preflight_cannot_enable_or_send_nonzero_gain(self):
        clock, buses = self.fixture()
        review = id7_review(authorized=False)
        review['id7_joint_inspection_verified'] = False
        for flag in ('raw_direction_reviewed_for_diagnostic',
                     'swept_clearance_verified', 'support_stand_verified',
                     'feet_clear_verified', 'hands_clear_verified', 'physical_cutoff_ready'):
            review[flag] = False
        with boot_file(), patch.object(core.threading, 'Barrier', side_effect=clock.barrier):
            result = run_id7_step1(buses, UIDS, lambda: None, lambda _: None,
                                   validated_review=review, preflight_only=True,
                                   clock=clock, wait=clock.wait)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        for bus in buses.values():
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in bus.frames))
            self.assertTrue(all(frame.data[4:8] == bytes(4) for _, frame, _ in bus.frames
                                if frame.kind == 1))
        self.assert_stopped(buses)

    def test_new_scope_requires_same_boot_fresh_centers_inspection_and_no_old_target(self):
        for changed in ('old_scope', 'uninspected', 'old_target', 'other_axis_direction',
                        'over_one_degree', 'old_boot', 'stale_start'):
            clock, buses = self.fixture()
            review = id7_review()
            if changed == 'old_scope':
                review['scope'] = core.REVIEW_SCOPE
            elif changed == 'uninspected':
                review['id7_joint_inspection_verified'] = False
            elif changed == 'old_target':
                review['old_raw_target_reused'] = True
            elif changed == 'other_axis_direction':
                review['raw_direction_by_id']['8'] = 1
            elif changed == 'over_one_degree':
                review['amplitude_deg'] = 1.001
            elif changed == 'old_boot':
                review['boot_id'] = 'another-boot'
            else:
                buses['rear'].centers[7] += math.radians(.6)
            with self.subTest(changed=changed):
                if changed == 'stale_start':
                    result = self.run_trial(clock, buses, review)
                    self.assertEqual(result['status'], 'ABORTED')
                    self.assertFalse(any(frame.kind == 3 for bus in buses.values()
                                         for _, frame, _ in bus.frames))
                    self.assert_stopped(buses)
                else:
                    with self.assertRaises(ValueError):
                        self.run_trial(clock, buses, review)
                    self.assertTrue(all(not bus.calls for bus in buses.values()))

    def test_id7_fast_feedback_aborts_both_buses_and_stops_all_twelve(self):
        for abnormal in ('reported_velocity', 'position_rate', 'excursion', 'held_drift'):
            clock, buses = self.fixture()
            bus = buses['rear']
            original = bus.value

            def value(mid):
                feedback = original(mid)
                if mid == 8 and abnormal == 'held_drift' and bus.active_batches >= 3:
                    return replace(feedback, protocol_position_rad=
                                   bus.centers[mid]+math.radians(2.51))
                if mid == 7 and bus.active_batches >= 3:
                    if abnormal == 'reported_velocity':
                        return replace(feedback, velocity_rad_s=.201)
                    if abnormal == 'position_rate':
                        return replace(feedback, protocol_position_rad=
                                       feedback.protocol_position_rad + math.radians(1.))
                    if abnormal == 'excursion':
                        return replace(feedback, protocol_position_rad=
                                       bus.centers[mid]+math.radians(1.51))
                return feedback

            bus.value = value
            with self.subTest(abnormal=abnormal):
                result = self.run_trial(clock, buses)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertTrue(result['stop_confirmed'])
                expected_id = 'ID8' if abnormal == 'held_drift' else 'ID7'
                self.assertTrue(any(expected_id in err for err in result['errors']),
                                result['errors'])
                self.assert_stopped(buses)

    def test_missing_or_stale_reply_and_partial_write_abort_with_all_stop_attempts(self):
        for failure in ('missing', 'stale', 'write'):
            clock, buses = self.fixture(failure=failure)
            result = self.run_trial(clock, buses)
            self.assertEqual(result['status'], 'ABORTED', result['errors'])
            self.assertTrue(result['stop_confirmed'])
            self.assert_stopped(buses)

    def test_id7_speed_after_enable_blocks_first_motion_packet(self):
        clock, buses = self.fixture()
        rear = buses['rear']
        original = rear.value

        def fast_enabled(mid):
            feedback = original(mid)
            return (replace(feedback, velocity_rad_s=.201)
                    if mid == 7 and mid in rear.enabled else feedback)

        rear.value = fast_enabled
        result = self.run_trial(clock, buses)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID7 raw step feedback velocity' in e for e in result['errors']))
        self.assertFalse(any(frame.kind == 1 and frame.data[4:8] != bytes(4)
                             for bus in buses.values() for _, frame, _ in bus.frames))
        self.assert_stopped(buses)

    def test_packet_gate_rejects_changed_id7_or_held_axis_before_uart_write(self):
        for changed_mid in (7, 8):
            review = id7_review()
            with boot_file():
                state = ID7Step1PacketState(review, clock=lambda: 10.)
            serials = {name: FakeSerial() for name in BUS_IDS}
            ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
            centers = {i: review['start_raw_rad_by_id'][str(i)] for i in core.ALL_IDS}
            for bus, ids in BUS_IDS.items():
                state.bind_centers(bus, {i: centers[i] for i in ids})
            for bus, ids in BUS_IDS.items():
                for mid in ids:
                    ports[bus].write(enable_request(phase=TrialPhase.ENABLE, motor_id=mid))
                    ports[bus].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                    center_rad=centers[mid], motor_id=mid))
            plan = core._plan(centers, review)
            for mid in BUS_IDS['front']:
                phase = TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10) else TrialPhase.POSITION_STEP5
                ports['front'].write(motion_request(phase=phase, center_rad=centers[mid],
                                     offset_rad=plan[0][mid]-centers[mid], motor_id=mid))
            for mid in BUS_IDS['rear']:
                phase = TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10) else TrialPhase.POSITION_STEP5
                offset = math.radians(.5) if mid == changed_mid else plan[0][mid]-centers[mid]
                wire = motion_request(phase=phase, center_rad=centers[mid],
                                      offset_rad=offset, motor_id=mid)
                if mid == changed_mid:
                    before = len(serials['rear'].writes)
                    with self.assertRaises(RuntimeError):
                        ports['rear'].write(wire)
                    self.assertEqual(len(serials['rear'].writes), before)
                    self.assertTrue(state.audit()['terminal'])
                    ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))
                    break
                ports['rear'].write(wire)

    def test_id7_gate_has_same_finite_180_tick_budget(self):
        review = id7_review()
        with boot_file():
            state = ID7Step1PacketState(review, clock=lambda: 10.)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in core.ALL_IDS}
        plan = core._plan(centers, review)
        for bus, ids in BUS_IDS.items():
            state.bind_centers(bus, {i: centers[i] for i in ids})
        for bus, ids in BUS_IDS.items():
            for mid in ids:
                ports[bus].write(enable_request(phase=TrialPhase.ENABLE, motor_id=mid))
                ports[bus].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                center_rad=centers[mid], motor_id=mid))
        for tick in range(core.ACTIVE_TICKS):
            for bus, ids in BUS_IDS.items():
                for mid in ids:
                    phase = TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10) else TrialPhase.POSITION_STEP5
                    ports[bus].write(motion_request(phase=phase, center_rad=centers[mid],
                                      offset_rad=plan[tick][mid]-centers[mid], motor_id=mid))
        self.assertEqual(state.audit()['next_tick_by_bus'], {'front': 180, 'rear': 180})
        before = len(serials['rear'].writes)
        with self.assertRaises(RuntimeError):
            ports['rear'].write(motion_request(phase=TrialPhase.POSITION_STEP5,
                                center_rad=centers[7], offset_rad=math.radians(1.), motor_id=7))
        self.assertEqual(len(serials['rear'].writes), before)
        ports['rear'].write(stop_request(phase=TrialPhase.STOP, motor_id=7))

    def test_id7_gate_partial_write_blocks_peer_motion_and_allows_stop(self):
        review = id7_review()
        with boot_file():
            state = ID7Step1PacketState(review, clock=lambda: 10.)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in core.ALL_IDS}
        for bus, ids in BUS_IDS.items():
            state.bind_centers(bus, {i: centers[i] for i in ids})
        for bus, ids in BUS_IDS.items():
            for mid in ids:
                ports[bus].write(enable_request(phase=TrialPhase.ENABLE, motor_id=mid))
                ports[bus].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                center_rad=centers[mid], motor_id=mid))
        serials['rear'].partial = True
        wire = motion_request(phase=TrialPhase.POSITION_STEP5,
                              center_rad=centers[7], motor_id=7)
        with self.assertRaises(RuntimeError):
            ports['rear'].write(wire)
        self.assertTrue(state.audit()['terminal'])
        before = len(serials['front'].writes)
        with self.assertRaises(RuntimeError):
            ports['front'].write(motion_request(phase=TrialPhase.POSITION_STEP5,
                                 center_rad=centers[1], motor_id=1))
        self.assertEqual(len(serials['front'].writes), before)
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))


if __name__ == '__main__':
    unittest.main()
