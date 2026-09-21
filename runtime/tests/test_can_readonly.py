import math
import struct
import unittest
from singularitydog_hw.can_readonly import (ATParser, Frame, ReadOnlyCAN, PARAMETERS,
                                          read_request, decode_reply, matches)


def response(mid=1, index=0x701C, value=40.5, status=0, payload=None, dest=0xFD):
    data = payload if payload is not None else struct.pack("<H2xf", index, value)
    can_id = (17 << 24) | (status << 16) | (mid << 8) | dest
    wire = b"AT" + ((can_id << 3)|4).to_bytes(4,"big") + bytes([8]) + data + b"\r\n"
    return ATParser().feed(wire)[0]


class CodecTests(unittest.TestCase):
    def test_golden_identity_and_read_only_surface(self):
        self.assertEqual(read_request(1).hex(), '41540007e80c0800000000000000000d0a')
        for mid in range(1,13):
            for name in [None,*PARAMETERS]:
                wire = read_request(mid,name)
                f=ATParser().feed(wire)[0]
                self.assertIn(f.kind,(0,17))
                self.assertEqual(f.flags,4)
                self.assertEqual(f.destination,mid)
                self.assertEqual(f.data[2:],bytes(6))
        for bad in (0,13,253,True,'1'):
            with self.assertRaises(ValueError): read_request(bad)
        with self.assertRaises(ValueError): read_request(1,'enable')

    def test_binary_payload_fragmentation_and_resync(self):
        wire=response(payload=b'AT\r\n\x00\xff\x00\xff').wire
        for cut in range(len(wire)+1):
            p=ATParser();got=p.feed(b'noise'+wire[:cut])+p.feed(wire[cut:]+wire)
            self.assertEqual([f.wire for f in got],[wire,wire])
            self.assertEqual(p.discarded_bytes,5)
        p=ATParser();self.assertEqual(p.feed(b'AT\x00\x00\x00\x04\xff'+wire)[0].wire,wire)

    def test_matching_rejects_echo_wrong_id_index_and_destination(self):
        self.assertFalse(matches(ATParser().feed(read_request(1,'voltage'))[0],1,'voltage'))
        for f in (response(mid=2), response(index=0x7019), response(dest=0xFE)):
            with self.assertRaises(ValueError): decode_reply(f,1,'voltage')

    def test_float_and_error_status(self):
        self.assertEqual(decode_reply(response(),1,'voltage')['value'],40.5)
        r=decode_reply(response(status=1),1,'voltage')
        self.assertFalse(r['ok']);self.assertIsNone(r['value'])
        for value in (math.nan,math.inf,-math.inf):
            r=decode_reply(response(value=value),1,'voltage')
            self.assertFalse(r['ok']);self.assertIsNone(r['value'])

    def test_uint8_and_timeout_not_float(self):
        f=response(index=0x7028,payload=struct.pack('<H2xI',0x7028,20000))
        self.assertEqual(decode_reply(f,1,'can_timeout')['timeout_seconds'],1)
        f=response(index=0x7005,payload=struct.pack('<H2x4B',0x7005,5,0,0,0))
        self.assertEqual(decode_reply(f,1,'run_mode')['value'],5)

    def test_timeout_poison_no_retry(self):
        class Silent:
            in_waiting=0
            def write(self,data): return len(data)
            def read(self,n): return b''
        ticks=iter(range(0,100_000_000,2_000_000))
        can=ReadOnlyCAN(serial_port=Silent(),timeout_s=.01,clock=lambda:next(ticks))
        with self.assertRaises(TimeoutError): can.query(1,'voltage')
        with self.assertRaises(RuntimeError): can.query(1,'voltage')
        self.assertEqual(can.tx_count,1)

    def test_read_failure_poison_no_late_reply_reuse(self):
        class BrokenRead:
            in_waiting=0
            def write(self,data): return len(data)
            def read(self,n): raise OSError('disconnected after TX')
        can=ReadOnlyCAN(serial_port=BrokenRead())
        with self.assertRaises(OSError): can.query(1,'voltage')
        self.assertTrue(can.poisoned)
        with self.assertRaises(RuntimeError): can.query(1,'voltage')
        self.assertEqual(can.tx_count,1)


if __name__=='__main__': unittest.main()
