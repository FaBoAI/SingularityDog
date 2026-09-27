"""No-device tests for the opt-in 19-second runner and exact UART gate."""
from dataclasses import replace
import math
import unittest

from singularitydog_hw import rs05_fullbody_step2 as runner
from singularitydog_hw.rs05_step2_packet_gate import Step2PacketPort, Step2PacketState
from singularitydog_hw.rs05_trial_protocol import TrialPhase, enable_request, motion_request, stop_request
from test_rs05_fullbody_hold import BUS_IDS, FakeBus, WorkerClock, boot_file
import test_rs05_fullbody_step2 as step_tests
import test_rs05_step2_packet_gate as gate_tests
from test_rs05_fullbody_step2 import role_group_path
from test_rs05_step2_packet_gate import FakeSerial


def continuous_review(*, authorized=True):
    review = role_group_path('front-hip', authorized=authorized, amplitude=10.)
    review.update(continuous_profile=runner.CONTINUOUS_PROFILE,
                  continuous_waypoints_deg=[5., 10.], continuous_19s_reviewed=authorized)
    return review


class ContinuousRuntimeTests(unittest.TestCase):
    fixture = step_tests.RawStep2Tests.fixture
    run_trial = step_tests.RawStep2Tests.run_trial
    assert_stopped = step_tests.RawStep2Tests.assert_stopped

    def test_one_enable_interval_all_380_exact_ticks_and_three_checked_holds(self):
        clock, buses = self.fixture()
        review = continuous_review()
        result = self.run_trial(clock, buses, review=review)
        self.assertEqual(result['status'], 'RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual((result['active_ticks'], result['active_duration_s'], result['active_budget_s']),
                         (380, 19., 20.5))
        self.assertFalse(result['automatic_retry'])
        centers = {mid: .5+.1*mid for mid in runner.ALL_IDS}
        plan = runner._plan(centers, review)
        self.assertIs(type(plan), tuple)
        self.assertEqual(len(plan), 380)
        for name, bus in buses.items():
            frames = [frame for _, frame, _ in bus.frames]
            enables = [frame for frame in frames if frame.kind == 3]
            self.assertEqual([frame.destination for frame in enables], list(bus.ids))
            self.assertEqual(len(bus.stop_calls), 2)  # Initial disabled STOP and one final STOP.
            first_enable = next(index for index, frame in enumerate(frames) if frame.kind == 3)
            last_active = max(index for index, frame in enumerate(frames)
                              if frame.kind == 1 and frame.data[4:8] != bytes(4))
            self.assertFalse(any(frame.kind == 4 for frame in frames[first_enable:last_active+1]))
            for mid in bus.ids:
                actual = [frame.wire for frame in frames if frame.kind == 1
                          and frame.destination == mid and frame.data[4:8] != bytes(4)]
                expected = [motion_request(phase=runner._motion_phase(review, mid),
                    center_rad=centers[mid], offset_rad=sample[mid]-centers[mid], motor_id=mid)
                    for sample in plan]
                self.assertEqual(actual, expected)
            worker = result['workers'][name]
            self.assertEqual(worker['cycle_count'], 380)
            self.assertEqual([row['end_tick'] for row in worker['continuous_hold_checks']], [19, 199, 379])
            self.assertTrue(all(len(samples) == 20 for row in worker['continuous_hold_checks']
                                for samples in row['samples'].values()))
            self.assertAlmostEqual(bus.stop_calls[-1][1]-worker['start_monotonic_s'], 19.)
        self.assert_stopped(buses)

    def test_first_waypoint_failure_on_either_bus_blocks_second_segment(self):
        for bad_bus, bad_mid in (('front', 3), ('rear', 9)):
            class EndpointBiasBus(FakeBus):
                def value(self, mid):
                    value = super().value(mid)
                    if mid == bad_mid and self.active_batches > 20:
                        # Stay inside the 2-degree live tracking limit, with
                        # a gradual bias, but fail the 1.5-degree endpoint.
                        bias = math.radians(1.6)*min((self.active_batches-20)/20, 1.)
                        value = replace(value, protocol_position_rad=value.protocol_position_rad+bias)
                    return value
            clock = WorkerClock()
            buses = {name: (EndpointBiasBus if name == bad_bus else FakeBus)(name, clock)
                     for name in BUS_IDS}
            with self.subTest(bus=bad_bus):
                result = self.run_trial(clock, buses, review=continuous_review())
                self.assertEqual(result['status'], 'ABORTED')
                self.assertTrue(any(f'ID{bad_mid} continuous tick199 raw step error' in error
                                    for error in result['errors']), result['errors'])
                self.assertTrue(all(bus.active_batches <= 200 for bus in buses.values()))
                self.assertTrue(result['stop_confirmed'])
                self.assert_stopped(buses)

    def test_second_segment_deadline_failure_still_stops_both_buses(self):
        class LateSecondSegmentBus(FakeBus):
            def feedback_many(self, commands, expected_ids):
                found = super().feedback_many(commands, expected_ids)
                if self.active_batches == 201:
                    self.clock.wait(.051)
                return found
        clock = WorkerClock()
        buses = {'front': LateSecondSegmentBus('front', clock), 'rear': FakeBus('rear', clock)}
        result = self.run_trial(clock, buses, review=continuous_review())
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('missed50ms' in error for error in result['errors']), result['errors'])
        self.assertTrue(all(bus.active_batches <= 201 for bus in buses.values()))
        self.assertTrue(result['stop_confirmed'])
        self.assert_stopped(buses)

    def test_profile_and_physical_review_are_explicit_before_io(self):
        for change in ('profile_missing', 'waypoints_missing', 'reversed', 'extra', 'bool',
                       'not_reviewed', 'truthy_review', 'wrong_scope'):
            review = continuous_review()
            if change == 'profile_missing': del review['continuous_profile']
            elif change == 'waypoints_missing': del review['continuous_waypoints_deg']
            elif change == 'reversed': review['continuous_waypoints_deg'] = [10., 5.]
            elif change == 'extra': review['continuous_waypoints_deg'] = [5., 10., 0.]
            elif change == 'bool': review['continuous_waypoints_deg'] = [True, 10.]
            elif change == 'not_reviewed': review['continuous_19s_reviewed'] = False
            elif change == 'truthy_review': review['continuous_19s_reviewed'] = 1
            else: review['amplitude_deg'] = 5.
            clock, buses = self.fixture()
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.run_trial(clock, buses, review=review)
            self.assertFalse(any(bus.calls for bus in buses.values()))

    def test_continuous_disabled_preflight_never_enables(self):
        clock, buses = self.fixture()
        result = self.run_trial(clock, buses, review=continuous_review(authorized=False), preflight_only=True)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(all(row['cycle_count'] == 20 for row in result['workers'].values()))
        self.assertFalse(any(frame.kind == 3 or (frame.kind == 1 and frame.data[4:8] != bytes(4))
                             for bus in buses.values() for _, frame, _ in bus.frames))
        self.assert_stopped(buses)


class ContinuousPacketTests(unittest.TestCase):
    bind = gate_tests.PacketGateTests.bind
    enable = gate_tests.PacketGateTests.enable

    def fixture(self, *, review=None, clock=lambda: 10.):
        review = continuous_review() if review is None else review
        with boot_file():
            state = Step2PacketState(review, clock=clock)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        centers = {mid: review['start_raw_rad_by_id'][str(mid)] for mid in runner.ALL_IDS}
        return state, ports, serials, centers, review

    def command(self, review, centers, plan, tick, mid):
        return motion_request(phase=runner._motion_phase(review, mid), center_rad=centers[mid],
                              offset_rad=plan[tick][mid]-centers[mid], motor_id=mid)

    def test_exact_380_tick_budget_and_no_extra_active_frame(self):
        state, ports, serials, centers, review = self.fixture()
        self.bind(state, centers)
        self.enable(ports, centers)
        plan = runner._plan(centers, review)
        for tick in range(380):
            for bus, ids in BUS_IDS.items():
                for mid in ids:
                    ports[bus].write(self.command(review, centers, plan, tick, mid))
        self.assertEqual(state.audit()['next_tick_by_bus'], {'front': 380, 'rear': 380})
        self.assertEqual(state.audit()['active_budget_s'], 20.5)
        before = len(serials['front'].writes)
        with self.assertRaisesRegex(RuntimeError, 'budget exhausted'):
            ports['front'].write(self.command(review, centers, plan, 379, 1))
        self.assertEqual(len(serials['front'].writes), before)
        for bus, ids in BUS_IDS.items():
            for mid in ids:
                ports[bus].write(stop_request(phase=TrialPhase.STOP, motor_id=mid))

    def test_packet_gate_checks_clearance_independently_for_single_and_continuous(self):
        for review in (role_group_path('front-hip', amplitude=10.), continuous_review()):
            review['clearance_reference_raw_rad_by_id']['9'] += math.radians(3.1)
            state, ports, serials, centers, _ = self.fixture(review=review)
            state.bind_centers('front', {mid: centers[mid] for mid in BUS_IDS['front']})
            with self.assertRaisesRegex(ValueError, 'clearance envelope'):
                state.bind_centers('rear', {mid: centers[mid] for mid in BUS_IDS['rear']})
            with self.assertRaisesRegex(RuntimeError, 'already stopped'):
                ports['front'].write(enable_request(phase=TrialPhase.ENABLE, motor_id=1))
            self.assertFalse(any(port.writes for port in serials.values()))

    def test_gate_owns_review_copy_and_does_not_extend_20_5_second_budget(self):
        now = [10.]
        state, ports, serials, centers, review = self.fixture(clock=lambda: now[0])
        self.bind(state, centers)
        self.enable(ports, centers)
        plan = runner._plan(centers, review)
        review['continuous_waypoints_deg'][0] = 10.
        review['clearance_reference_raw_rad_by_id']['9'] = 0.
        self.assertEqual(state.review['continuous_waypoints_deg'], [5., 10.])
        command = self.command(state.review, centers, plan, 0, 1)
        ports['front'].write(command)
        now[0] += 20.5
        before = len(serials['front'].writes)
        with self.assertRaisesRegex(RuntimeError, 'budget expired'):
            ports['front'].write(self.command(state.review, centers, plan, 0, 2))
        self.assertEqual(len(serials['front'].writes), before)
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))


if __name__ == '__main__':
    unittest.main()
