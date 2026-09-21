"""RobStride AT transport restricted to identification and parameter reads.

Protocol facts: RobStride RS05User Manual260713, sections3.4/3.5;
RobStride/CAN-USB-data-conversion switch/mainwindow.cpp.
No configuration, enable, disable, zeroing or motion frame is generated here.
"""
from dataclasses import dataclass
import math
import struct
import time

HOST_ID = 0xFD
# Requests are limited to these documented parameters, with no arbitrary-index CLI.
PARAMETERS = {
    "run_mode": (0x7005, "B", "enum"),
    "position": (0x7019, "f", "rad_output_shaft"),
    "current": (0x701A, "f", "A"),
    "velocity": (0x701B, "f", "rad_s_output_shaft"),
    "voltage": (0x701C, "f", "V"),
    "can_timeout": (0x7028, "I", "50_us_ticks"),
    "zero_state": (0x7029, "B", "enum"),
}


@dataclass(frozen=True)
class Frame:
    can_id: int
    flags: int
    data: bytes
    wire: bytes

    @property
    def kind(self):
        return (self.can_id >> 24) & 31

    @property
    def source(self):
        return (self.can_id >> 8) & 255

    @property
    def destination(self):
        return self.can_id & 255

    def record(self):
        return {"can_id": self.can_id, "type": self.kind,
                "source_id": self.source, "destination_id": self.destination,
                "flags": self.flags, "data_hex": self.data.hex(), "wire_hex": self.wire.hex()}


class ATParser:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"AT":
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            if len(self.buffer) < 7:
                break
            dlc = self.buffer[6]
            if dlc > 8:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            size = 9 + dlc
            if len(self.buffer) < size:
                break
            if self.buffer[size-2:size] != b"\r\n":
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            wire = bytes(self.buffer[:size])
            del self.buffer[:size]
            encoded_id = int.from_bytes(wire[2:6], "big")
            frames.append(Frame(encoded_id >> 3, encoded_id & 7, wire[7:7+dlc], wire))
        return frames


def read_request(motor_id, parameter=None):
    if type(motor_id) is not int or not 1 <= motor_id <= 12:
        raise ValueError("Configured robot IDs are exactly 1..12")
    kind, payload = 0, bytes(8)
    if parameter is not None:
        if parameter not in PARAMETERS:
            raise ValueError("Parameter is not in the read-only allowlist")
        kind = 17
        payload = struct.pack("<H", PARAMETERS[parameter][0]) + bytes(6)
    can_id = (kind << 24) | (HOST_ID << 8) | motor_id
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + bytes([8]) + payload + b"\r\n"


def matches(frame, motor_id, parameter):
    if frame.flags != 4 or len(frame.data) != 8 or frame.source != motor_id:
        return False
    if parameter is None:
        return frame.kind == 0 and frame.destination == 0xFE
    return (frame.kind == 17 and frame.destination == HOST_ID and
            int.from_bytes(frame.data[:2], "little") == PARAMETERS[parameter][0])


def decode_reply(frame, motor_id, parameter):
    if not matches(frame, motor_id, parameter):
        raise ValueError("Reply does not match request")
    if parameter is None:
        return {"motor_id": motor_id, "parameter": "identity", "ok": True,
                "mcu_uid_hex": frame.data.hex()}
    index, fmt, unit = PARAMETERS[parameter]
    status = (frame.can_id >> 16) & 255
    result = {"motor_id": motor_id, "parameter": parameter, "index": index,
              "status": status, "raw_value_hex": frame.data[4:8].hex(),
              "unit": unit, "value": None, "ok": False}
    if frame.data[2:4] != bytes(2):
        result["error"] = "reserved_bytes_nonzero"
        return result
    if status:
        result["error"] = "parameter_status_nonzero"
        return result
    value = struct.unpack_from("<" + fmt, frame.data, 4)[0]
    if not math.isfinite(value):
        result["error"] = "nonfinite_parameter"
        return result
    result.update(value=value, ok=True)
    if parameter == "can_timeout":
        result["timeout_seconds"] = value / 20000.0
        result["timeout_enabled"] = value != 0
    return result


def feedback_metadata(frame):
    """Preserve raw positions/torques; do not conflate firmware-dependent wraps."""
    if frame.flags != 4 or len(frame.data) != 8 or frame.kind not in (2, 21):
        return None
    result = {"motor_id": frame.source, "type": frame.kind, "raw_frame": frame.record()}
    if frame.kind == 2:
        p, v, t, temp = struct.unpack(">4H", frame.data)
        result.update(position_u16=p, velocity_u16=v, torque_u16=t,
                      temperature_c=temp/10.0, mode_state=(frame.can_id >> 22) & 3,
                      fault_bits=(frame.can_id >> 16) & 63)
    else:
        result["fault_detail_u32"] = int.from_bytes(frame.data[:4], "little")
    return result


class ReadOnlyCAN:
    """One owner/thread, one outstanding request. A timeout poisons the session.

    The private protocol has no transaction sequence number. After a timeout,
    this object will not retry and risk treating a late reply as fresh data.
    """
    def __init__(self, port="/dev/robstride-usb2can", *, timeout_s=0.25,
                 event_sink=None, serial_port=None, clock=time.monotonic_ns):
        if not math.isfinite(timeout_s) or not 0.01 <= timeout_s <= 2.0:
            raise ValueError("timeout_s must be finite and in0.01..2")
        self.port_name = port
        self.timeout_s = timeout_s
        self.sink = event_sink or (lambda event: None)
        self.serial = serial_port
        self.clock = clock
        self.parser = ATParser()
        self.poisoned = False
        self.sequence = 0
        self.tx_count = 0
        self.rx_bytes = 0

    def __enter__(self):
        if self.serial is None:
            import serial
            s = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
                              timeout=0.003, write_timeout=0.1, xonxoff=False,
                              rtscts=False, dsrdtr=False, exclusive=True)
            s.dtr = s.rts = False
            s.port = self.port_name
            try:
                s.open()
            except BaseException:
                s.close()
                raise
            self.serial = s
        return self

    def __exit__(self, *_):
        if self.serial is not None:
            self.serial.close()
        self.poisoned = True

    def _receive(self):
        chunk = self.serial.read(min(max(self.serial.in_waiting, 1), 2048))
        if not chunk:
            return []
        now = self.clock()
        self.rx_bytes += len(chunk)
        self.sink({"kind": "can_rx_bytes", "monotonic_ns": now, "hex": chunk.hex()})
        frames = self.parser.feed(chunk)
        for frame in frames:
            self.sink({"kind": "can_rx_frame", "monotonic_ns": now, **frame.record()})
            fb = feedback_metadata(frame)
            if fb is not None:
                self.sink({"kind": "motor_feedback", "monotonic_ns": now, **fb})
        return frames

    def query(self, motor_id, parameter=None):
        try:
            return self._query(motor_id, parameter)
        except BaseException:
            # Any interrupted exchange can leave an uncorrelatable delayed reply.
            self.poisoned = True
            raise

    def _query(self, motor_id, parameter=None):
        if self.serial is None or self.poisoned:
            raise RuntimeError("Read-only CAN session is closed or timed out")
        wire = read_request(motor_id, parameter)
        # Log, but never reuse frames that were already buffered before this request.
        drain_deadline = self.clock() + 50_000_000
        while self.serial.in_waiting:
            self._receive()
            if self.clock() >= drain_deadline:
                self.poisoned = True
                raise RuntimeError("CAN input backlog; cannot establish fresh request boundary")
        if self.parser.buffer:
            self.poisoned = True
            raise RuntimeError("Incomplete frame before request; cannot correlate fresh reply")
        started = self.clock()
        self.sequence += 1
        self.sink({"kind": "can_tx", "monotonic_ns": started,
                   "sequence": self.sequence, "motor_id": motor_id,
                   "parameter": parameter or "identity", "hex": wire.hex()})
        self.tx_count += 1
        try:
            if self.serial.write(wire) != len(wire):
                raise IOError("Partial serial write")
        except BaseException:
            self.poisoned = True
            raise
        deadline = started + int(self.timeout_s * 1e9)
        while self.clock() < deadline:
            for frame in self._receive():
                if matches(frame, motor_id, parameter):
                    finished = self.clock()
                    if finished >= deadline:
                        break
                    result = decode_reply(frame, motor_id, parameter)
                    result.update(kind="motor_parameter", sequence=self.sequence,
                                  request_monotonic_ns=started, monotonic_ns=finished,
                                  round_trip_ms=(finished-started)/1e6)
                    self.sink(result)
                    return result
        self.poisoned = True
        self.sink({"kind": "can_timeout", "monotonic_ns": self.clock(),
                   "motor_id": motor_id, "parameter": parameter or "identity"})
        raise TimeoutError(f"No fresh response: ID{motor_id} {parameter or 'identity'}")
