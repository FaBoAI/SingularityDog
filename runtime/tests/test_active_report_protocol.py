import unittest
from dataclasses import replace

from singularitydog_hw.active_report_protocol import (
    ALLOWED_REQUEST_KINDS, decode_period_reply, period_read_request,
    period_write_request, reporting_request,
)
from singularitydog_hw.can_readonly import ATParser, Frame


def reply(*, kind=17, source=1, destination=0xFD, status=0, flags=4,
          payload=bytes.fromhex("2670000001000000")):
    can_id = (kind << 24) | (status << 16) | (source << 8) | destination
    wire = (b"AT" + ((can_id << 3) | flags).to_bytes(4, "big")
            + bytes((len(payload),)) + payload + b"\r\n")
    return Frame(can_id, flags, payload, wire)


class ActiveReportProtocolTests(unittest.TestCase):
    def test_official_golden_wires_use_eight_byte_can_data(self):
        cases = (
            (reporting_request(1, True),
             "4154c007e80c0801020304050601080d0a"),
            (reporting_request(1, False),
             "4154c007e80c0801020304050600080d0a"),
            (period_read_request(1),
             "41548807e80c0826700000000000000d0a"),
            (period_write_request(1, 1),
             "41549007e80c0826700000010000000d0a"),
            (period_write_request(1, 1025),
             "41549007e80c0826700000010400000d0a"),
            (period_write_request(1, 65535),
             "41549007e80c0826700000ffff00000d0a"),
        )
        for actual, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(actual.hex(), expected)
                parser = ATParser()
                frames = parser.feed(actual)
                self.assertEqual(len(frames), 1)
                self.assertEqual(len(frames[0].data), 8)
                self.assertEqual(parser.buffer, b"")
                self.assertEqual(parser.discarded_bytes, 0)

    def test_requests_have_only_selected_id_host_and_allowed_kinds(self):
        self.assertEqual(ALLOWED_REQUEST_KINDS, frozenset((17, 18, 24)))
        for mid in range(1, 13):
            cases = ((reporting_request(mid, True), 24),
                     (reporting_request(mid, False), 24),
                     (period_read_request(mid), 17),
                     (period_write_request(mid, 1), 18))
            for wire, kind in cases:
                frame, = ATParser().feed(wire)
                self.assertEqual(frame.can_id, (kind << 24) | (0xFD << 8) | mid)
                self.assertEqual(frame.flags, 4)
                if kind in (17, 18):
                    self.assertEqual(frame.data[:4], bytes.fromhex("26700000"))

    def test_bool_ids_broadcast_and_noninteger_ids_are_rejected(self):
        for mid in (True, False, 0, 13, 255, -1, 2**64, 1.0, "1", None):
            for function in (lambda: reporting_request(mid, True),
                             lambda: period_read_request(mid),
                             lambda: period_write_request(mid, 1),
                             lambda: decode_period_reply(reply(), mid)):
                with self.subTest(mid=mid):
                    with self.assertRaises(ValueError):
                        function()

    def test_enabled_requires_exact_boolean(self):
        for enabled in (0, 1, -1, 1.0, "true", b"\x01", None):
            with self.subTest(enabled=enabled), self.assertRaises(ValueError):
                reporting_request(1, enabled)

    def test_period_requires_restorable_uint16_positive_integer(self):
        for ticks in (0, -1, 65536, 2**64, True, False, 1.0, "1", None):
            with self.subTest(ticks=ticks), self.assertRaises(ValueError):
                period_write_request(1, ticks)

    def test_reply_golden_wire_and_unsigned_little_endian_value(self):
        frame, = ATParser().feed(bytes.fromhex("415488000fec0826700000010000000d0a"))
        self.assertEqual(decode_period_reply(frame, 1), 1)
        for ticks in (1, 3, 1025, 32768, 65535):
            data = b"\x26\x70\x00\x00" + ticks.to_bytes(2, "little") + b"\x00\x00"
            for mid in range(1, 13):
                with self.subTest(ticks=ticks, mid=mid):
                    self.assertEqual(decode_period_reply(reply(source=mid, payload=data), mid), ticks)

    def test_read_reply_unspecified_trailing_bytes_are_not_assumed_zero(self):
        frame = reply(payload=bytes.fromhex("267000000304a55a"))
        self.assertEqual(decode_period_reply(frame, 1), 1027)
        self.assertEqual(frame.data[6:], bytes.fromhex("a55a"))

    def test_observed_zero_requires_explicit_unqualified_read_option(self):
        # Actual ID1 read supplied to the review, not a newly sent command.
        frame, = ATParser().feed(bytes.fromhex("415488000fec0826700000000000000d0a"))
        for kwargs in ({}, {"allow_unqualified_zero": False}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                decode_period_reply(frame, 1, **kwargs)
        self.assertEqual(decode_period_reply(frame, 1, allow_unqualified_zero=True), 0)
        with self.assertRaises(ValueError):
            period_write_request(1, 0)
        with self.assertRaises(TypeError):
            decode_period_reply(frame, 1, True)

    def test_unqualified_zero_option_must_be_exact_boolean(self):
        for flag in (0, 1, None, "true", 1.0, b"\x01", [], {}):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                decode_period_reply(reply(), 1, allow_unqualified_zero=flag)
        for flag in (False, True):
            self.assertEqual(decode_period_reply(reply(), 1, allow_unqualified_zero=flag), 1)

    def test_unqualified_zero_retains_all_reply_validation(self):
        zero = bytes.fromhex("2670000000000000")
        cases = (reply(kind=24, payload=zero), reply(source=2, payload=zero),
                 reply(destination=1, payload=zero), reply(status=1, payload=zero),
                 reply(flags=0, payload=zero), reply(payload=zero[:-1]),
                 reply(payload=bytes.fromhex("2570000000000000")),
                 reply(payload=bytes.fromhex("2670010000000000")),
                 replace(reply(payload=zero), wire=b""))
        for frame in cases:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                decode_period_reply(frame, 1, allow_unqualified_zero=True)

    def test_wrong_reply_type_source_destination_flags_and_status_rejected(self):
        cases = ({"kind": 2}, {"kind": 18}, {"kind": 24},
                 {"source": 2}, {"source": 0}, {"source": 255},
                 {"destination": 1}, {"destination": 0xFE},
                 {"flags": 0}, {"flags": 6}, {"status": 1}, {"status": 255})
        for kwargs in cases:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                decode_period_reply(reply(**kwargs), 1)

    def test_wrong_index_reserved_bytes_zero_ticks_and_dlc_rejected(self):
        for data in (bytes.fromhex("2570000001000000"),
                     bytes.fromhex("7026000001000000"),
                     bytes.fromhex("2670010001000000"),
                     bytes.fromhex("2670000101000000"),
                     bytes.fromhex("2670000000000000"), b"", bytes(7), bytes(9)):
            with self.subTest(data=data.hex()), self.assertRaises(ValueError):
                decode_period_reply(reply(payload=data), 1)

    def test_noncanonical_frames_and_invalid_field_types_rejected(self):
        frame = reply()
        cases = (None, b"AT", {},
                 replace(frame, can_id=True), replace(frame, can_id=-1),
                 replace(frame, can_id=0x20000000), replace(frame, can_id=1.0),
                 replace(frame, flags=True), replace(frame, flags=4.0),
                 replace(frame, data=bytearray(frame.data)),
                 replace(frame, data=frame.data.hex()),
                 replace(frame, wire=bytearray(frame.wire)),
                 replace(frame, wire=frame.wire[:-1]),
                 replace(frame, wire=b"XX" + frame.wire[2:]),
                 replace(frame, wire=frame.wire[:6] + b"\x07" + frame.wire[7:]),
                 replace(frame, wire=frame.wire[:-2] + b"\x00\x00"),
                 replace(frame, data=bytes.fromhex("2670000002000000")),
                 replace(frame, can_id=frame.can_id + 0x100))
        for malformed in cases:
            with self.subTest(malformed=malformed), self.assertRaises(ValueError):
                decode_period_reply(malformed, 1)


if __name__ == "__main__":
    unittest.main()
