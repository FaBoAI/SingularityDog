"""Fixed-record decoding must retain fault evidence and reject bad envelopes."""
import math
import struct
import unittest

from singularitydog_hw import native_active_transport as native
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.motor_version_probe import version_request
from singularitydog_hw import policy_output_runtime as runtime


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def record(tx, rx):
    value = native.Record()
    value.tx[:] = tx
    value.rx[:] = rx
    value.written = value.received = 17
    value.start_ns, value.finish_ns, value.received_ns, value.deadline_ns = 100, 110, 200, 300
    return value


class FixedRecordDecodeTests(unittest.TestCase):
    def test_all_ids_modes_and_faults_preserve_exact_wire_and_extreme_values(self):
        for mid in range(1, 13):
            for mode in range(4):
                for fault in range(64):
                    rx = frame((2 << 24)|(mode << 22)|(fault << 16)|(mid << 8)|0xfd,
                               struct.pack('>4H', 65535, 0, 65535, 301))
                    d = native.decode_record(record(native.encode_motion(mid, 0, 0, 0), rx))
                    self.assertEqual((d['motor_id'], d['mode_state'], d['fault_bits']), (mid, mode, fault))
                    self.assertEqual((d['position_rad_candidate'], d['velocity_rad_s_candidate'],
                                      d['torque_nm_candidate'], d['temperature_c']), (12.57, -50., 5.5, 30.1))
                    self.assertEqual(d['reply_wire_hex'], rx.hex())
                    self.assertEqual(d['received_monotonic_ns'], 200)

    def test_readback_types_do_not_change_units_or_hide_nonfinite_values(self):
        for index, payload, expected in ((0x7028, struct.pack('<I', 4000), 4000),
                                         (0x7005, bytes([2, 0, 0, 0]), 2),
                                         (0x7019, struct.pack('<f', -6.25), -6.25),
                                         (0x701c, struct.pack('<f', float('inf')), float('inf'))):
            rx = frame((17 << 24)|(12 << 8)|0xfd, struct.pack('<H', index)+bytes(2)+payload)
            d = native.decode_record(record(read_request(12, 'position'), rx))
            self.assertEqual(d['index'], index)
            self.assertEqual(d['value'], expected)
        rx = frame((17 << 24)|(1 << 8)|0xfd, b'\x19\x70\x00\x00'+struct.pack('<f', float('nan')))
        self.assertTrue(math.isnan(native.decode_record(record(read_request(1, 'position'), rx))['value']))

    def test_uid_and_version_are_distinct_from_motion_feedback(self):
        rx = frame((1 << 8)|0xfe, bytes.fromhex('0123456789abcdef'))
        self.assertEqual(native.decode_record(record(read_request(1), rx))['uid_hex'], '0123456789abcdef')
        rx = frame((2 << 24)|(1 << 8)|0xfd, bytes.fromhex('00c45605001300a5'))
        d = native.decode_record(record(version_request(1), rx))
        self.assertEqual(d['version_bytes_hex'], '05001300')
        self.assertEqual(d['request_started_monotonic_ns'], 100)
        with self.assertRaisesRegex(ValueError, 'version reply'):
            native.decode_record(record(native.encode_motion(1, 0, 0, 0), rx))
        for flags in ((1 << 16), (2 << 22)):
            bad = frame((2 << 24)|flags|(1 << 8)|0xfd, bytes.fromhex('00c45605001300a5'))
            with self.assertRaises(ValueError):
                native.decode_record(record(version_request(1), bad))

    def test_incomplete_or_noncanonical_records_are_rejected(self):
        tx = native.encode_motion(1, 0, 0, 0)
        rx = frame((2 << 24)|(1 << 8)|0xfd, bytes(8))
        for field in ('written', 'received'):
            r = record(tx, rx)
            setattr(r, field, 16)
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                native.decode_record(r)
        for side in ('tx', 'rx'):
            for index, value in ((0, 0), (1, 0), (5, 0), (6, 7), (15, 0), (16, 0)):
                r = record(tx, rx)
                getattr(r, side)[index] = value
                with self.assertRaisesRegex(ValueError, 'Noncanonical'):
                    native.decode_record(r)


class RuntimeFixedRecordDecodeTests(unittest.TestCase):
    def feedback(self, mid=1, *, mode=2, fault=0, host=0xfd):
        return frame((2 << 24)|(mode << 22)|(fault << 16)|(mid << 8)|host,
                     struct.pack('>4H', 32768, 32767, 32767, 300))

    def decode(self, rx, tx=None):
        return runtime.decode_records(([record(tx or native.encode_motion(1, 0, 0, 0), rx)], None))

    def test_strict_reply_identity_and_reserved_mode_validation_remain(self):
        for rx in (self.feedback(mid=2), self.feedback(host=0xfe), self.feedback(mode=3)):
            with self.assertRaises(ValueError):
                self.decode(rx)
        for mode in range(3):
            for fault in range(64):
                feedback, start, end = self.decode(self.feedback(mode=mode, fault=fault))[1, 'feedback']
                self.assertEqual((feedback.mode_state, feedback.fault_bits, start, end), (mode, fault, 100, 200))

    def test_timestamp_duplicate_and_envelope_checks_remain(self):
        r = record(native.encode_motion(1, 0, 0, 0), self.feedback())
        with self.assertRaisesRegex(RuntimeError, 'Duplicate'):
            runtime.decode_records(([r, r], None))
        for field, value in (('written', 0), ('start_ns', 0), ('finish_ns', 201), ('deadline_ns', 200)):
            bad = record(bytes(r.tx), bytes(r.rx));setattr(bad, field, value)
            with self.assertRaisesRegex(RuntimeError, 'noncausal'):
                runtime.decode_records(([bad], None))
        for side in ('tx', 'rx'):
            for index, value in ((0, 0), (1, 0), (5, 0), (6, 7), (15, 0), (16, 0)):
                bad = record(bytes(r.tx), bytes(r.rx));getattr(bad, side)[index] = value
                with self.assertRaisesRegex(RuntimeError, 'Invalid native frame'):
                    runtime.decode_records(([bad], None))

    def test_voltage_read_errors_and_version_confusion_are_rejected(self):
        for payload in (b'\x1c\x70\x01\x00'+struct.pack('<f', 40.),
                        b'\x1c\x70\x00\x00'+struct.pack('<f', float('nan'))):
            with self.assertRaisesRegex(RuntimeError, 'Parameter or identity rejected'):
                self.decode(frame((17 << 24)|(1 << 8)|0xfd, payload), read_request(1, 'voltage'))
        version = frame((2 << 24)|(1 << 8)|0xfd, bytes.fromhex('00c45605001300a5'))
        with self.assertRaisesRegex(ValueError, 'Version-shaped'):
            self.decode(version)
        self.assertEqual(self.decode(version, version_request(1))[1, 'version'][0]['version_bytes_hex'], '05001300')


if __name__ == '__main__':
    unittest.main()
