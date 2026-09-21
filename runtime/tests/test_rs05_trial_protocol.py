import math
import struct
import unittest
from dataclasses import replace

from singularitydog_hw.can_readonly import ATParser, Frame
from singularitydog_hw.rs05_trial_protocol import (
    MAX_OFFSET_RAD, VISIBLE_MAX_OFFSET_RAD, POSITION_MIN, POSITION_MAX, TrialPhase,
    decode_type2, decode_type17_position, enable_request, stop_request,
    watchdog_setup_request, watchdog_readback_request, motion_request,
)


def feedback(*, kind=2, source=1, destination=0xFD, mode=0, faults=0,
             status=0, flags=4, data=None):
    can_id = ((kind << 24) | (mode << 22) | (faults << 16)
              | (status << 16) | (source << 8) | destination)
    if data is None:
        data = bytes.fromhex("7fff7fff7fff012c")
    wire = (b"AT" + ((can_id << 3) | flags).to_bytes(4, "big")
            + bytes([len(data)]) + data + b"\r\n")
    return Frame(can_id, flags, data, wire)


class RS05TrialCodecTests(unittest.TestCase):
    def test_enable_stop_and_watchdog_golden_wire(self):
        cases = [
            (enable_request, TrialPhase.ENABLE,
             "41541807e80c0800000000000000000d0a"),
            (stop_request, TrialPhase.STOP,
             "41542007e80c0800000000000000000d0a"),
            (watchdog_setup_request, TrialPhase.WATCHDOG_SETUP,
             "41549007e80c0828700000a00f00000d0a"),
            (watchdog_readback_request, TrialPhase.WATCHDOG_READBACK,
             "41548807e80c0828700000000000000d0a"),
        ]
        for encoder, phase, expected in cases:
            with self.subTest(phase=phase):
                wire = encoder(phase=phase)
                self.assertEqual(wire.hex(), expected)
                frame = ATParser().feed(wire)[0]
                self.assertEqual((frame.flags, frame.destination), (4, 1))
                self.assertEqual((frame.can_id >> 8) & 0xFFFF, 0xFD)

    def test_explicit_exact_phases(self):
        for encoder, good in (
            (enable_request, TrialPhase.ENABLE), (stop_request, TrialPhase.STOP),
            (watchdog_setup_request, TrialPhase.WATCHDOG_SETUP),
            (watchdog_readback_request, TrialPhase.WATCHDOG_READBACK),
        ):
            for wrong in (None, True, 1, good.value,
                          *[p for p in TrialPhase if p is not good]):
                with self.subTest(encoder=encoder.__name__, phase=wrong):
                    with self.assertRaises(ValueError): encoder(phase=wrong)
            with self.assertRaises(TypeError): encoder()

    def test_broadcast_out_of_range_and_noninteger_ids_rejected(self):
        calls = (
            lambda mid: enable_request(phase=TrialPhase.ENABLE, motor_id=mid),
            lambda mid: stop_request(phase=TrialPhase.STOP, motor_id=mid),
            lambda mid: watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid),
            lambda mid: watchdog_readback_request(phase=TrialPhase.WATCHDOG_READBACK, motor_id=mid),
            lambda mid: motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=0, motor_id=mid),
            lambda mid: motion_request(phase=TrialPhase.POSITION, center_rad=0, motor_id=mid),
            lambda mid: motion_request(phase=TrialPhase.POSITION_STEP2, center_rad=0, motor_id=mid),
            lambda mid: motion_request(phase=TrialPhase.POSITION_VISIBLE, center_rad=0, motor_id=mid),
        )
        for mid in (0, 13, 255, -1, 2**64, True, False, 1.0, "1", None):
            for call in calls:
                with self.subTest(mid=mid):
                    with self.assertRaises(ValueError): call(mid)

    def test_all_twelve_ids_encode_one_selected_destination_only(self):
        cases = (
            (enable_request, {"phase": TrialPhase.ENABLE}, 3),
            (stop_request, {"phase": TrialPhase.STOP}, 4),
            (watchdog_setup_request, {"phase": TrialPhase.WATCHDOG_SETUP}, 18),
            (watchdog_readback_request, {"phase": TrialPhase.WATCHDOG_READBACK}, 17),
            *[(motion_request, {"phase": phase, "center_rad": 0.0}, 1)
              for phase in (TrialPhase.ZERO_GAIN, TrialPhase.POSITION,
                            TrialPhase.POSITION_STEP2, TrialPhase.POSITION_VISIBLE)],
        )
        for motor_id in range(1, 13):
            for encoder, kwargs, kind in cases:
                with self.subTest(motor_id=motor_id, phase=kwargs["phase"]):
                    default = ATParser().feed(encoder(**kwargs))[0]
                    frames = ATParser().feed(encoder(**kwargs, motor_id=motor_id))
                    self.assertEqual(len(frames), 1)
                    frame = frames[0]
                    self.assertEqual((frame.kind, frame.destination, frame.flags),
                                     (kind, motor_id, 4))
                    self.assertEqual(frame.can_id, (default.can_id & ~255) | motor_id)
                    self.assertEqual(frame.data, default.data)

    def test_all_twelve_reply_ids_require_explicit_matching_selection(self):
        payload = struct.pack("<H2xf", 0x7019, 5.25)
        for motor_id in range(1, 13):
            f2 = feedback(source=motor_id, mode=2)
            f17 = feedback(kind=17, source=motor_id, data=payload)
            self.assertEqual(decode_type2(f2, motor_id=motor_id).mode_state, 2)
            self.assertEqual(decode_type17_position(f17, motor_id=motor_id).direct_position_rad, 5.25)
            for other_id in range(1, 13):
                if other_id != motor_id:
                    with self.assertRaises(ValueError): decode_type2(f2, motor_id=other_id)
                    with self.assertRaises(ValueError): decode_type17_position(f17, motor_id=other_id)
            if motor_id != 1:
                with self.assertRaises(ValueError): decode_type2(f2)
                with self.assertRaises(ValueError): decode_type17_position(f17)

    def test_reply_selection_rejects_invalid_ids_and_wrong_host(self):
        payload = struct.pack("<H2xf", 0x7019, 5.25)
        for motor_id in (0, 13, 255, -1, 2**64, True, False, 1.0, "1", None):
            with self.assertRaises(ValueError): decode_type2(feedback(), motor_id=motor_id)
            with self.assertRaises(ValueError):
                decode_type17_position(feedback(kind=17, data=payload), motor_id=motor_id)
        for motor_id in range(1, 13):
            with self.assertRaises(ValueError):
                decode_type2(feedback(source=motor_id, destination=0xFE), motor_id=motor_id)
            with self.assertRaises(ValueError):
                decode_type17_position(feedback(kind=17, source=motor_id,
                    destination=1, data=payload), motor_id=motor_id)

    def test_zero_gain_golden_and_quantization_bias(self):
        wire = motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=0)
        self.assertEqual(wire.hex(), "41540bfff80c087fff7fff000000000d0a")
        frame = ATParser().feed(wire)[0]
        self.assertEqual((frame.can_id >> 8) & 65535, 32767)
        self.assertNotEqual((frame.can_id >> 8) & 65535, 0xFD)
        neutral = decode_type2(feedback())
        self.assertAlmostEqual(neutral.protocol_position_rad, -12.57 / 65535)
        self.assertAlmostEqual(neutral.velocity_rad_s, -50.0 / 65535)
        self.assertAlmostEqual(neutral.torque_nm, -5.5 / 65535)
        self.assertNotEqual(neutral.torque_nm, 0.0)

    def test_plus_and_minus_one_degree_golden(self):
        for sign, payload in [(1, "802c7fff00410106"), (-1, "7fd27fff00410106")]:
            wire = motion_request(phase=TrialPhase.POSITION, center_rad=0,
                                  offset_rad=sign * MAX_OFFSET_RAD)
            self.assertEqual(wire.hex(), "41540bfff80c08" + payload + "0d0a")

    def test_step2_fixed_gains_and_one_degree_golden(self):
        for offset, payload in [(MAX_OFFSET_RAD, "802c7fff028f028f"),
                                (-MAX_OFFSET_RAD, "7fd27fff028f028f"),
                                (0.0, "7fff7fff028f028f")]:
            wire = motion_request(phase=TrialPhase.POSITION_STEP2, center_rad=0,
                                  offset_rad=offset)
            self.assertEqual(wire.hex(), "41540bfff80c08" + payload + "0d0a")
        # The initial profile remains unchanged; phase selection is explicit.
        initial = motion_request(phase=TrialPhase.POSITION, center_rad=0)
        self.assertEqual(initial.hex(), "41540bfff80c087fff7fff004101060d0a")
        with self.assertRaises(ValueError):
            motion_request(phase="position_step2", center_rad=0)

    def test_step2_same_input_and_headroom_boundaries(self):
        for center in (POSITION_MIN + MAX_OFFSET_RAD, POSITION_MAX - MAX_OFFSET_RAD):
            for offset in (-MAX_OFFSET_RAD, MAX_OFFSET_RAD):
                motion_request(phase=TrialPhase.POSITION_STEP2,
                               center_rad=center, offset_rad=offset)
        for center in (POSITION_MIN, POSITION_MAX,
                       math.nextafter(POSITION_MIN + MAX_OFFSET_RAD, -math.inf),
                       math.nextafter(POSITION_MAX - MAX_OFFSET_RAD, math.inf)):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_STEP2, center_rad=center)
        invalid = (math.nan, math.inf, -math.inf, True, False, None,
                   "0.0", complex(0), 10**500)
        for name in ("center_rad", "offset_rad"):
            for value in invalid:
                with self.assertRaises(ValueError):
                    motion_request(**{"phase": TrialPhase.POSITION_STEP2,
                                      "center_rad": 0.0, name: value})
        for offset in (math.nextafter(MAX_OFFSET_RAD, math.inf),
                       -math.nextafter(MAX_OFFSET_RAD, math.inf), 2 * math.pi):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_STEP2,
                               center_rad=0, offset_rad=offset)

    def test_targets_are_relative_to_center_not_joint_zero(self):
        center = 5.25
        for offset in (-MAX_OFFSET_RAD, 0, MAX_OFFSET_RAD):
            wire = motion_request(phase=TrialPhase.POSITION, center_rad=center,
                                  offset_rad=offset)
            p = int.from_bytes(ATParser().feed(wire)[0].data[:2], "big")
            decoded = p * 25.14 / 65535 - 12.57
            self.assertLessEqual(abs(decoded - (center + offset)), 25.14 / 65535)

    def test_visible_three_degree_fixed_gains_golden(self):
        for offset, payload in [(VISIBLE_MAX_OFFSET_RAD, "80877fff018907ae"),
                                (-VISIBLE_MAX_OFFSET_RAD, "7f777fff018907ae"),
                                (0.0, "7fff7fff018907ae")]:
            wire = motion_request(phase=TrialPhase.POSITION_VISIBLE, center_rad=0,
                                  offset_rad=offset)
            self.assertEqual(wire.hex(), "41540bfff80c08" + payload + "0d0a")
            frame = ATParser().feed(wire)[0]
            self.assertEqual(struct.unpack(">4H", frame.data)[2:], (393, 1966))
        with self.assertRaises(ValueError):
            motion_request(phase="position_visible", center_rad=0)

    def test_visible_three_degree_boundaries_and_input_rejections(self):
        for center in (POSITION_MIN + VISIBLE_MAX_OFFSET_RAD,
                       POSITION_MAX - VISIBLE_MAX_OFFSET_RAD):
            for offset in (-VISIBLE_MAX_OFFSET_RAD, VISIBLE_MAX_OFFSET_RAD):
                motion_request(phase=TrialPhase.POSITION_VISIBLE,
                               center_rad=center, offset_rad=offset)
        for center in (POSITION_MIN, POSITION_MAX,
                       math.nextafter(POSITION_MIN + VISIBLE_MAX_OFFSET_RAD, -math.inf),
                       math.nextafter(POSITION_MAX - VISIBLE_MAX_OFFSET_RAD, math.inf)):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_VISIBLE, center_rad=center)
        for offset in (math.nextafter(VISIBLE_MAX_OFFSET_RAD, math.inf),
                       -math.nextafter(VISIBLE_MAX_OFFSET_RAD, math.inf), 2 * math.pi):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION_VISIBLE,
                               center_rad=0, offset_rad=offset)
        for name in ("center_rad", "offset_rad"):
            for value in (math.nan, math.inf, -math.inf, True, False, None,
                          "0.0", complex(0), 10**500):
                with self.assertRaises(ValueError):
                    motion_request(**{"phase": TrialPhase.POSITION_VISIBLE,
                                      "center_rad": 0.0, name: value})

    def test_three_degrees_never_relaxes_prior_profile_bounds(self):
        for phase in (TrialPhase.POSITION, TrialPhase.POSITION_STEP2):
            for offset in (-VISIBLE_MAX_OFFSET_RAD, VISIBLE_MAX_OFFSET_RAD,
                           math.nextafter(MAX_OFFSET_RAD, math.inf),
                           -math.nextafter(MAX_OFFSET_RAD, math.inf)):
                with self.assertRaises(ValueError):
                    motion_request(phase=phase, center_rad=0, offset_rad=offset)
            # The old profiles still accept centers with exactly one degree headroom.
            for center in (POSITION_MIN + MAX_OFFSET_RAD, POSITION_MAX - MAX_OFFSET_RAD):
                motion_request(phase=phase, center_rad=center)
                with self.assertRaises(ValueError):
                    motion_request(phase=TrialPhase.POSITION_VISIBLE, center_rad=center)

    def test_nonfinite_bool_and_oversized_inputs_rejected(self):
        for value in (math.nan, math.inf, -math.inf, True, False, None,
                      "0.0", complex(0), 10**500):
            for name in ("center_rad", "offset_rad"):
                kwargs = {"phase": TrialPhase.POSITION, "center_rad": 0.0, name: value}
                with self.subTest(name=name, value=str(value)[:30]):
                    with self.assertRaises(ValueError): motion_request(**kwargs)

    def test_no_clamping_or_wrapping(self):
        for offset in (math.nextafter(MAX_OFFSET_RAD, math.inf), 0.1,
                       -math.nextafter(MAX_OFFSET_RAD, math.inf), 2 * math.pi):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION, center_rad=0, offset_rad=offset)
        for center in (-100, 100, math.nextafter(POSITION_MAX, math.inf),
                       math.nextafter(POSITION_MIN, -math.inf)):
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=center)

    def test_full_trial_headroom_required_and_boundaries_accepted(self):
        for center in (POSITION_MIN, POSITION_MAX):
            motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=center)
            with self.assertRaises(ValueError):
                motion_request(phase=TrialPhase.POSITION, center_rad=center)
        for center in (POSITION_MIN + MAX_OFFSET_RAD, POSITION_MAX - MAX_OFFSET_RAD):
            for offset in (-MAX_OFFSET_RAD, MAX_OFFSET_RAD):
                motion_request(phase=TrialPhase.POSITION, center_rad=center, offset_rad=offset)

    def test_zero_gain_cannot_move_offset_and_motion_phase_is_explicit(self):
        with self.assertRaises(ValueError):
            motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=0, offset_rad=0.000001)
        for phase in (None, True, "position", TrialPhase.ENABLE, TrialPhase.STOP):
            with self.assertRaises(ValueError): motion_request(phase=phase, center_rad=0)
        with self.assertRaises(TypeError): motion_request(center_rad=0)

    def test_type2_mode_fault_temperature_and_profile_endpoints(self):
        f = feedback(mode=2, faults=0b101011,
                     data=struct.pack(">4H", 65535, 0, 65535, 372))
        result = decode_type2(f)
        self.assertEqual((result.mode_state, result.fault_bits), (2, 0b101011))
        self.assertEqual(result.position_u16, 65535)
        self.assertAlmostEqual(result.protocol_position_rad, 12.57)
        self.assertEqual(result.velocity_rad_s, -50)
        self.assertEqual(result.torque_nm, 5.5)
        self.assertEqual(result.temperature_c, 37.2)
        self.assertAlmostEqual(decode_type2(feedback(data=bytes(8))).protocol_position_rad, -12.57)
        with self.assertRaises(ValueError): decode_type2(feedback(mode=3))

    def test_type2_rejects_wrong_address_kind_flags_and_dlc(self):
        invalid = [feedback(source=2), feedback(source=12), feedback(destination=1),
                   feedback(destination=0xFE), feedback(kind=17), feedback(flags=0),
                   feedback(flags=5), feedback(data=bytes(7)), feedback(data=bytes(9)),
                   feedback(data=b"\x00\xc4\x56" + bytes(5))]
        for f in invalid:
            with self.assertRaises(ValueError): decode_type2(f)

    def test_noncanonical_or_oversized_frame_rejected(self):
        f = feedback()
        for bad in (None, b"AT", replace(f, can_id=True),
                    replace(f, can_id=-1), replace(f, can_id=1 << 29),
                    replace(f, flags=True), replace(f, data=bytearray(f.data)),
                    replace(f, wire=f.wire + b"extra"), replace(f, wire=b""),
                    replace(f, wire=f.wire[:6] + b"\x07" + f.wire[7:])):
            with self.assertRaises(ValueError): decode_type2(bad)

    def test_type17_position_is_direct_and_not_type2_wrapped(self):
        for value in (0.0, 5.25, -33.5, 40.0):
            f = feedback(kind=17, data=struct.pack("<H2xf", 0x7019, value))
            result = decode_type17_position(f)
            self.assertEqual(result.direct_position_rad, value)
            self.assertFalse(hasattr(result, "protocol_position_rad"))
            with self.assertRaises(ValueError): decode_type2(f)
        with self.assertRaises(ValueError): decode_type17_position(feedback())

    def test_type17_rejects_failed_nonfinite_wrong_index_and_reserved(self):
        good = struct.pack("<H2xf", 0x7019, 0.0)
        invalid = [feedback(kind=17, status=1, data=good),
                   feedback(kind=17, status=255, data=good),
                   feedback(kind=17, source=2, data=good),
                   feedback(kind=17, destination=1, data=good),
                   feedback(kind=17, data=struct.pack("<H2xf", 0x701C, 0.0)),
                   feedback(kind=17, data=b"\x19\x70\x01\x00" + bytes(4))]
        invalid += [feedback(kind=17, data=struct.pack("<H2xf", 0x7019, v))
                    for v in (math.nan, math.inf, -math.inf)]
        for f in invalid:
            with self.assertRaises(ValueError): decode_type17_position(f)


if __name__ == "__main__":
    unittest.main()
