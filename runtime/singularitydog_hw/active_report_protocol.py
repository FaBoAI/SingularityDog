"""Pure RS05 active-report codecs; no transport or hardware access.

The only requests produced are Type 24 reporting on/off, Type 17 reading
EPScan_time (0x7026), and Type 18 writing that same uint16 parameter. They
target exactly one configured ID in 1..12 and host 0xFD. This module does not
expand the existing read-only transport allowlist or establish that a motor
is stopped, that its firmware supports reporting, or that a write succeeded.

Verified sources:
* Official STM32 example, RobStride_Motor_ProactiveEscalationSet, lines 734-757:
  https://github.com/RobStride/SampleProgram/blob/5f598686b05fcc527ee0fc0ea954f0afd652b234/RS/Robstride01.cpp#L734-L757
  Extended CAN data frame, DLC 8, data 01 02 03 04 05 06 F_CMD 08.
  F_CMD is 00 off / 01 on. The comment permits any final byte; we retain 08.
  This shared example does not establish a minimum supported RS05 firmware.
* RS05 User Manual 260713, sections 4.1.7, 4.1.8, 4.1.12, and 4.1.14:
  https://github.com/RobStride/Product_Information/blob/0f4ad74fdb67023e75bbcbeebecd6f1a003ce000/Product%20Literature/RS05/RS05User%20Manual260713.pdf
  EPScan_time is uint16: 1 means 10 ms, each further tick adds 5 ms.
  Reporting reply type remains a firmware qualification concern: its table
  says Type 24 while section 4.2.1 says Type 2. Neither is decoded here.
* Official Python SDK, uint16 parameter packing and EPSCAN_TIME definition:
  https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/bus.py#L149-L206
  https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/protocol.py#L91
  Parameter index and value are little-endian. Writes zero-pad bytes 6..7;
  zero padding of a uint16 READ reply is not specified, so we ignore those
  two bytes. Bytes 2..3 are explicitly reserved zero in the RS05 manual.
  The SDK's uint16 read branch uses signed '<h'; this decoder uses '<H'.
* Official USB adapter encoding uses (CAN_ID << 3) | 4, big-endian on UART:
  https://github.com/RobStride/CAN-USB-data-conversion/blob/72524311fbf4980d74ad53387c075cc50d8dd417/switch/mainwindow.cpp

Successful Type 18 replies are status feedback, not parameter readback. The
caller must separately read 0x7026 and verify its value, with bounded I/O and
appropriate handling of asynchronous reports. No reporting/STOP ACK
correlation or original reporting-state recovery is implied by these codecs.
"""

import struct

from .can_readonly import Frame, HOST_ID


PERIOD_INDEX = 0x7026
ALLOWED_REQUEST_KINDS = frozenset((17, 18, 24))


def _motor_id(mid):
    if type(mid) is not int or not 1 <= mid <= 12:
        raise ValueError("Select exactly one integer motor ID in 1..12")


def _period_ticks(ticks):
    if type(ticks) is not int or not 1 <= ticks <= 65535:
        raise ValueError("EPScan_time must be an integer in 1..65535")


def _wire(can_id, payload):
    return (b"AT" + ((can_id << 3) | 4).to_bytes(4, "big")
            + b"\x08" + payload + b"\r\n")


def _request(mid, kind, payload):
    _motor_id(mid)
    return _wire((kind << 24) | (HOST_ID << 8) | mid, payload)


def reporting_request(mid, enabled: bool) -> bytes:
    """Encode the official Type 24 frame, without enabling motor torque."""
    if type(enabled) is not bool:
        raise ValueError("Reporting enabled must be exactly bool")
    return _request(mid, 24, b"\x01\x02\x03\x04\x05\x06"
                    + bytes((int(enabled), 0x08)))


def period_read_request(mid) -> bytes:
    """Read only the uint16 EPScan_time parameter; all other bytes are zero."""
    return _request(mid, 17, struct.pack("<H", PERIOD_INDEX) + bytes(6))


def period_write_request(mid, ticks) -> bytes:
    """Set/restore EPScan_time in 1..65535; successful sending is not readback."""
    _period_ticks(ticks)
    return _request(mid, 18, struct.pack("<HHHH", PERIOD_INDEX, 0, ticks, 0))


def decode_period_reply(frame, mid, *, allow_unqualified_zero=False) -> int:
    """Validate a canonical successful Type 17 reply and return uint16 ticks.

    The unspecified bytes 6..7 are retained in the caller's original Frame
    but ignored here. Zero is outside the qualified period/restore range.
    An explicit allow_unqualified_zero=True permits recording raw zero for
    an observation without period writes. It establishes no period duration,
    firmware support, or permission to write/restore zero. The setter remains
    restricted to 1..65535; all reply validation below remains mandatory.
    This validates content and addressing, not freshness or request matching
    after a timeout: the protocol has no transaction sequence number.
    """
    if type(allow_unqualified_zero) is not bool:
        raise ValueError("allow_unqualified_zero must be exactly bool")
    _motor_id(mid)
    if not isinstance(frame, Frame):
        raise ValueError("A parsed AT Frame is required")
    if type(frame.can_id) is not int or not 0 <= frame.can_id <= 0x1FFFFFFF:
        raise ValueError("CAN ID must be an unsigned 29-bit integer")
    if type(frame.flags) is not int or frame.flags != 4:
        raise ValueError("Only extended CAN data frames are accepted")
    if type(frame.data) is not bytes or len(frame.data) != 8:
        raise ValueError("Period reply DLC must be exactly 8")
    if type(frame.wire) is not bytes or frame.wire != _wire(frame.can_id, frame.data):
        raise ValueError("Reply must have a consistent canonical AT wire frame")
    if frame.kind != 17 or frame.source != mid or frame.destination != HOST_ID:
        raise ValueError("Period reply type/source/destination does not match")
    if (frame.can_id >> 16) & 0xFF:
        raise ValueError("EPScan_time read status is nonzero")
    if frame.data[:2] != struct.pack("<H", PERIOD_INDEX):
        raise ValueError("Expected parameter index 0x7026")
    if frame.data[2:4] != bytes(2):
        raise ValueError("Period reply reserved bytes must be zero")
    ticks = struct.unpack_from("<H", frame.data, 4)[0]
    if ticks != 0 or not allow_unqualified_zero:
        _period_ticks(ticks)
    return ticks
