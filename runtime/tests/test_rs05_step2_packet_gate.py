"""Virtual physical-UART boundary tests for the finite raw-step trial."""
import math
import struct
import unittest

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_step2_packet_gate import Step2PacketPort, Step2PacketState
from singularitydog_hw.rs05_fullbody_step2 import _motion_phase, _plan, ACTIVE_TICKS, KP4_IDS
from singularitydog_hw.rs05_trial_protocol import (TrialPhase, enable_request,
                                                   motion_request, stop_request)
from singularitydog_hw.rs05_bus_transport import BUS_IDS
from test_rs05_fullbody_hold import boot_file
from test_rs05_fullbody_step2 import reviewed_path, role_group_path


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.partial = False

    def write(self, wire):
        self.writes.append(wire)
        return len(wire)-1 if self.partial else len(wire)

    def read(self, count):
        return b''

    @property
    def in_waiting(self):
        return 0


class PacketGateTests(unittest.TestCase):
    def fixture(self):
        review = reviewed_path()
        with boot_file():
            state = Step2PacketState(review, clock=lambda: 10.)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in range(1, 13)}
        return state, ports, serials, centers, review

    def bind(self, state, centers):
        for bus, ids in BUS_IDS.items():
            state.bind_centers(bus, {i: centers[i] for i in ids})

    def enable(self, ports, centers):
        for bus, ids in BUS_IDS.items():
            for mid in ids:
                ports[bus].write(enable_request(phase=TrialPhase.ENABLE, motor_id=mid))
                ports[bus].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                center_rad=centers[mid], motor_id=mid))

    def frame(self, centers, plan, tick, mid):
        phase = TrialPhase.POSITION_STEP5_KP4 if mid in KP4_IDS else TrialPhase.POSITION_STEP5
        return motion_request(phase=phase, center_rad=centers[mid],
                              offset_rad=plan[tick][mid]-centers[mid], motor_id=mid)

    def test_no_gain_before_both_buses_bind_and_enable(self):
        state, ports, serials, centers, review = self.fixture()
        plan = _plan(centers, review)
        with self.assertRaises(RuntimeError):
            ports['front'].write(self.frame(centers, plan, 0, 1))
        self.assertEqual(serials['front'].writes, [])
        self.assertTrue(state.audit()['terminal'])

    def test_front_hip_pair_kp12_is_exactly_pinned_at_uart_boundary(self):
        review = role_group_path('front-hip')
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in range(1, 13)}
        with boot_file():
            state = Step2PacketState(review, clock=lambda: 10.)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        self.bind(state, centers)
        self.enable(ports, centers)
        plan = _plan(centers, review)
        for mid in (1, 2):
            ports['front'].write(motion_request(
                phase=_motion_phase(review, mid), center_rad=centers[mid],
                offset_rad=plan[0][mid]-centers[mid], motor_id=mid))
        wrong = motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=centers[3],
                               offset_rad=plan[0][3]-centers[3], motor_id=3)
        before = len(serials['front'].writes)
        with self.assertRaisesRegex(RuntimeError, 'differs from exact frozen trajectory'):
            ports['front'].write(wrong)
        self.assertEqual(len(serials['front'].writes), before)
        self.assertTrue(state.audit()['terminal'])
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=3))

        pair = [motion_request(phase=TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                               center_rad=centers[mid], offset_rad=math.radians(4.),
                               motor_id=mid) for mid in (3, 6)]
        for wire in pair:
            frame = ATParser().feed(wire)[0]
            self.assertEqual(struct.unpack('>4H', frame.data)[2], int(12*65535/500))
        with self.assertRaisesRegex(ValueError, 'ID3 or ID6'):
            motion_request(phase=TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                           center_rad=centers[9], motor_id=9)
        with self.assertRaisesRegex(ValueError, 'exceeds'):
            motion_request(phase=TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                           center_rad=centers[3], offset_rad=math.radians(5.01), motor_id=3)

    def test_exact_sequence_and_finite_180_tick_budget(self):
        state, ports, serials, centers, review = self.fixture()
        self.bind(state, centers)
        self.enable(ports, centers)
        self.assertEqual(state.audit()['post_enable_zero_gain_ids'], list(range(1, 13)))
        plan = _plan(centers, review)
        for tick in range(ACTIVE_TICKS):
            for bus, ids in BUS_IDS.items():
                for mid in ids:
                    ports[bus].write(self.frame(centers, plan, tick, mid))
        self.assertEqual(state.audit()['next_tick_by_bus'], {'front': 180, 'rear': 180})
        self.assertEqual(len(serials['front'].writes), 12 + 180*6)
        self.assertEqual(len(serials['rear'].writes), 12 + 180*6)
        with self.assertRaises(RuntimeError):
            ports['front'].write(self.frame(centers, plan, 179, 1))
        self.assertTrue(state.audit()['terminal'])

    def test_pre_enable_neutral_behavior_and_post_enable_center_binding(self):
        state, ports, serials, centers, review = self.fixture()
        ports['front'].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                            center_rad=0., motor_id=1))
        self.bind(state, centers)
        neutral = motion_request(phase=TrialPhase.ZERO_GAIN,
                                 center_rad=centers[1], motor_id=1)
        ports['front'].write(neutral)
        ports['front'].write(neutral)
        self.assertEqual(state.audit()['post_enable_zero_gain_ids'], [])
        ports['front'].write(enable_request(phase=TrialPhase.ENABLE, motor_id=1))
        ports['front'].write(neutral)
        self.assertEqual(state.audit()['post_enable_zero_gain_ids'], [1])
        self.assertEqual(len(serials['front'].writes), 5)

    def test_post_enable_zero_gain_is_exactly_once_and_stop_still_passes(self):
        for change in ('duplicate', 'changed_center'):
            with self.subTest(change=change):
                state, ports, serials, centers, review = self.fixture()
                self.bind(state, centers)
                ports['front'].write(enable_request(phase=TrialPhase.ENABLE, motor_id=1))
                if change == 'duplicate':
                    ports['front'].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                        center_rad=centers[1], motor_id=1))
                changed_center = (centers[1] if change == 'duplicate'
                                  else centers[1] + math.radians(1.))
                wire = motion_request(phase=TrialPhase.ZERO_GAIN,
                                      center_rad=changed_center, motor_id=1)
                before = len(serials['front'].writes)
                with self.assertRaisesRegex(RuntimeError, 'Unexpected zero-gain command'):
                    ports['front'].write(wire)
                self.assertEqual(len(serials['front'].writes), before)
                self.assertTrue(state.audit()['terminal'])
                ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))

    def test_first_active_tick_waits_for_every_post_enable_zero_gain(self):
        state, ports, serials, centers, review = self.fixture()
        self.bind(state, centers)
        for bus, ids in BUS_IDS.items():
            for mid in ids:
                ports[bus].write(enable_request(phase=TrialPhase.ENABLE, motor_id=mid))
                if mid != 12:
                    ports[bus].write(motion_request(phase=TrialPhase.ZERO_GAIN,
                                                    center_rad=centers[mid], motor_id=mid))
        self.assertEqual(state.audit()['post_enable_zero_gain_ids'], list(range(1, 12)))
        plan = _plan(centers, review)
        before = len(serials['front'].writes)
        with self.assertRaisesRegex(RuntimeError, 'all twelve Enable and zero-gain replies'):
            ports['front'].write(self.frame(centers, plan, 0, 1))
        self.assertEqual(len(serials['front'].writes), before)
        self.assertTrue(state.audit()['terminal'])

    def test_wrong_order_or_changed_wire_never_reaches_uart(self):
        for change in ('order', 'wire', 'gain'):
            with self.subTest(change=change):
                state, ports, serials, centers, review = self.fixture()
                self.bind(state, centers)
                self.enable(ports, centers)
                plan = _plan(centers, review)
                if change == 'order':
                    wire = self.frame(centers, plan, 0, 2)
                elif change == 'wire':
                    wire = motion_request(phase=TrialPhase.POSITION_STEP5,
                                          center_rad=centers[1],
                                          offset_rad=math.radians(1.), motor_id=1)
                else:
                    wire = motion_request(phase=TrialPhase.POSITION_STEP5_KP4,
                                          center_rad=centers[1], motor_id=1)
                before = len(serials['front'].writes)
                with self.assertRaises(RuntimeError):
                    ports['front'].write(wire)
                self.assertEqual(len(serials['front'].writes), before)
                self.assertTrue(state.audit()['terminal'])

    def test_stop_can_always_pass_and_ends_active_transaction(self):
        state, ports, serials, centers, review = self.fixture()
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))
        self.bind(state, centers)
        self.enable(ports, centers)
        ports['rear'].write(stop_request(phase=TrialPhase.STOP, motor_id=7))
        self.assertTrue(state.audit()['terminal'])
        before = len(serials['front'].writes)
        with self.assertRaises(RuntimeError):
            ports['front'].write(self.frame(centers, _plan(centers, review), 0, 1))
        self.assertEqual(len(serials['front'].writes), before)
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=2))

    def test_partial_uart_write_latches_abort_but_allows_stop(self):
        state, ports, serials, centers, review = self.fixture()
        serials['front'].partial = True
        with self.assertRaises(RuntimeError):
            ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=1))
        self.assertTrue(state.audit()['terminal'])
        serials['front'].partial = False
        ports['front'].write(stop_request(phase=TrialPhase.STOP, motor_id=2))

    def test_partial_active_write_blocks_peer_active_but_peer_stop_passes(self):
        state, ports, serials, centers, review = self.fixture()
        self.bind(state, centers)
        self.enable(ports, centers)
        plan = _plan(centers, review)
        serials['front'].partial = True
        with self.assertRaises(RuntimeError):
            ports['front'].write(self.frame(centers, plan, 0, 1))
        before = len(serials['rear'].writes)
        with self.assertRaises(RuntimeError):
            ports['rear'].write(self.frame(centers, plan, 0, 7))
        self.assertEqual(len(serials['rear'].writes), before)
        ports['rear'].write(stop_request(phase=TrialPhase.STOP, motor_id=7))

    def test_stale_fresh_center_refuses_enable_before_uart_write(self):
        state, ports, serials, centers, review = self.fixture()
        state.bind_centers('front', {i: centers[i] for i in BUS_IDS['front']})
        wrong_rear = {i: centers[i] for i in BUS_IDS['rear']}
        wrong_rear[9] += math.radians(1.)
        with self.assertRaises(ValueError):
            state.bind_centers('rear', wrong_rear)
        self.assertTrue(state.audit()['terminal'])
        with self.assertRaises(RuntimeError):
            ports['front'].write(enable_request(phase=TrialPhase.ENABLE, motor_id=1))
        self.assertEqual(serials['front'].writes, [])

    def test_stale_review_or_old_raw_target_is_rejected_at_construction(self):
        for change in ('old_target', 'not_authorized', 'unreviewed_direction'):
            review = reviewed_path()
            if change == 'old_target':
                review['old_raw_target_reused'] = True
            elif change == 'not_authorized':
                review['supported_step_authorized'] = False
            else:
                review['raw_direction_reviewed_for_diagnostic'] = False
            with self.subTest(change=change), boot_file(), self.assertRaises((ValueError, RuntimeError)):
                Step2PacketState(review)

    def test_four_axis_plan_blocks_an_unselected_joint_before_uart(self):
        review = role_group_path('thigh')
        with boot_file():
            state = Step2PacketState(review, clock=lambda: 10.)
        serial = FakeSerial()
        port = Step2PacketPort(serial, 'front', state)
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in range(1, 13)}
        self.bind(state, centers)
        ports = {name: Step2PacketPort(FakeSerial(), name, state) for name in BUS_IDS}
        self.enable(ports, centers)
        wrong = motion_request(phase=TrialPhase.POSITION_STEP5,
                               center_rad=centers[1], offset_rad=math.radians(1.), motor_id=1)
        with self.assertRaises(RuntimeError):
            port.write(wrong)
        self.assertEqual(serial.writes, [])
        self.assertTrue(state.audit()['terminal'])

    def test_four_thigh_kp12_wire_is_accepted_and_kp3_is_blocked(self):
        review = role_group_path('thigh')
        with boot_file():
            state = Step2PacketState(review, clock=lambda: 10.)
        serials = {name: FakeSerial() for name in BUS_IDS}
        ports = {name: Step2PacketPort(serials[name], name, state) for name in BUS_IDS}
        centers = {i: review['start_raw_rad_by_id'][str(i)] for i in range(1, 13)}
        self.bind(state, centers)
        self.enable(ports, centers)
        plan = _plan(centers, review)
        ports['front'].write(motion_request(phase=_motion_phase(review, 1),
                                            center_rad=centers[1], motor_id=1))
        wrong = motion_request(phase=TrialPhase.POSITION_STEP5,
                               center_rad=centers[2], motor_id=2)
        with self.assertRaises(RuntimeError):
            ports['front'].write(wrong)
        self.assertEqual(len(serials['front'].writes), 13)
        self.assertTrue(state.audit()['terminal'])

    def test_front_thigh_gate_rejects_opposite_sign_and_rear_motion(self):
        review = role_group_path('front-thigh')
        for change in ('opposite_front_sign', 'rear_thigh_motion'):
            with self.subTest(change=change), boot_file():
                state = Step2PacketState(review, clock=lambda: 10.)
                serials = {name: FakeSerial() for name in BUS_IDS}
                ports = {name: Step2PacketPort(serials[name], name, state)
                         for name in BUS_IDS}
                centers = {i: review['start_raw_rad_by_id'][str(i)] for i in range(1, 13)}
                self.bind(state, centers)
                self.enable(ports, centers)
                if change == 'opposite_front_sign':
                    ports['front'].write(motion_request(phase=_motion_phase(review, 1),
                                                        center_rad=centers[1], motor_id=1))
                    bus, mid = 'front', 2
                    wire = motion_request(phase=_motion_phase(review, mid),
                                          center_rad=centers[mid],
                                          offset_rad=math.radians(1.), motor_id=mid)
                else:
                    ports['rear'].write(motion_request(phase=_motion_phase(review, 7),
                                                       center_rad=centers[7], motor_id=7))
                    bus, mid = 'rear', 8
                    wire = motion_request(phase=_motion_phase(review, mid),
                                          center_rad=centers[mid],
                                          offset_rad=math.radians(1.), motor_id=mid)
                before = len(serials[bus].writes)
                with self.assertRaises(RuntimeError):
                    ports[bus].write(wire)
                self.assertEqual(len(serials[bus].writes), before)
                self.assertTrue(state.audit()['terminal'])


if __name__ == '__main__':
    unittest.main()
