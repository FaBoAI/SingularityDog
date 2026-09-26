"""Exact RS05 0.5.0.13 transfer over an already-owned CH340 AT port.

Success path follows RobStride/Product_Information OTA/OTA.py at
6ad12f50006273b7ea4eea88980f927d97c22f0d. Explicit start_attempts=3 allows
only zero-receive-byte START timeouts to retry, with the official 50ms gap.
No DATA retries, resume, alternate ACK, enable, reset, parameter write, or
cleanup transmission is available here. Default remains one START attempt.
Identity, stopped-state, backup, binding and ownership are caller prerequisites.
"""
import hashlib
import os
import select
import struct
import time

from .can_readonly import ATParser, HOST_ID

BIN_SIZE = 93584
BIN_SHA256 = '3a4dfc1e9ad0ff2116b7b876c1fced3b8d23cf705e5d6366e5a286bac3989047'
PACKETS = BIN_SIZE // 8
ACK_TIMEOUT_NS = 2_000_000_000
MAX_TRANSFER_NS = 180_000_000_000
GAP_NS = 1_000_000
START_RETRY_GAP_NS = 50_000_000


class MissingAcknowledgement(TimeoutError):
    """Only a complete write followed by an ACK-free deadline reaches this."""


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def verify_image(image):
    require(type(image) is bytes and len(image) == BIN_SIZE, 'Wrong RS05 image length')
    require(hashlib.sha256(image).hexdigest() == BIN_SHA256, 'Wrong RS05 image SHA256')
    return image


def frame_for(image, motor_id, uid, ordinal):
    """Only this exact image and fixed START/INFO/DATA/END schedule can be sent."""
    require(type(motor_id) is int and 1 <= motor_id <= 12, 'Invalid robot motor ID')
    require(type(uid) is bytes and len(uid) == 8, 'UID must be eight bytes')
    require(type(image) is bytes and len(image) == BIN_SIZE, 'Wrong image length')
    require(type(ordinal) is int and 0 <= ordinal <= PACKETS + 2, 'Invalid sequence')
    if ordinal == 0:
        kind, data, payload = 11, HOST_ID, uid
    elif ordinal == 1:
        kind, data, payload = 12, HOST_ID, struct.pack('<II', BIN_SIZE, PACKETS)
    elif ordinal <= PACKETS + 1:
        index = ordinal - 2
        kind, data, payload = 13, index, image[index * 8:index * 8 + 8]
    else:
        kind, data, payload = 14, 0, struct.pack('<II', PACKETS, 0)
    can_id = (kind << 24) | (data << 8) | motor_id
    return kind, b'AT' + ((can_id << 3) | 4).to_bytes(4, 'big') + b'\x08' + payload + b'\r\n'


def validate_ack(frame, kind, motor_id):
    require(frame.flags == 4 and len(frame.data) == 8 and frame.can_id < (1 << 29),
            'Noncanonical OTA acknowledgement')
    require(frame.kind == kind, 'Unexpected OTA acknowledgement phase')
    require(frame.destination == HOST_ID and frame.source == motor_id,
            'Unexpected OTA acknowledgement identity')
    # Official documents disagree about the failure value. All nonzero values
    # fail, so neither 0x0f nor 0xf0 can accidentally advance the transfer.
    require(((frame.can_id >> 16) & 0xff) == 0, 'OTA device reported failure')


class FirmwareTransfer:
    def __init__(self, raw, image, motor_id, uid, *, check=lambda: None,
                 progress=lambda *_: None, clock=time.monotonic_ns,
                 read_ready=None, read_bytes=None, wait=None, start_attempts=1,
                 start_ack_timeout_s=2):
        require(type(start_attempts) is int and start_attempts in (1, 3),
                'START attempts must be explicitly 1 or 3')
        require(type(start_ack_timeout_s) is int and start_ack_timeout_s in (2, 3),
                'START ACK timeout must be explicitly 2 or 3 seconds')
        self.image = verify_image(image)
        frame_for(image, motor_id, uid, 0)
        self.raw, self.motor_id, self.uid = raw, motor_id, uid
        self.start_attempts = start_attempts
        self.start_ack_timeout_s = start_ack_timeout_s
        self.check, self.progress, self.clock = check, progress, clock
        self.read_ready = read_ready or (lambda timeout: bool(select.select([raw.fileno()], [], [], timeout)[0]))
        self.read_bytes = read_bytes or (lambda: os.read(raw.fileno(), 4096))
        self.wait = wait or time.sleep
        self.parser = ATParser()
        self.raw_log, self.tx_log = [], []
        self.started = self.deadline = None
        self._ran = False
        self.last_clock = None
        self.raw_bytes = 0
        self.report = {'status': 'NOT_STARTED', 'motor_id': motor_id,
            'target_version': '0.5.0.13', 'image_sha256': BIN_SHA256,
            'image_bytes': BIN_SIZE, 'data_packets': PACKETS,
            'bootloader_entry_attempted': False, 'acknowledged_data_packets': 0,
            'end_acknowledged': False, 'version_verified': False,
            'motor_enabling_available': False, 'motion_command_available': False,
            'automatic_retry': start_attempts > 1,
            'start_attempt_limit': start_attempts, 'start_transmit_attempts': 0,
            'start_ack_timeout_seconds': start_ack_timeout_s,
            'other_ack_timeout_seconds': 2,
            'data_retry_available': False, 'resume_available': False,
            'cleanup_transmission_available': False}

    def guard(self):
        self.check()
        now = self.clock()
        require(type(now) is int and (self.last_clock is None or now >= self.last_clock),
                'Invalid monotonic clock')
        self.last_clock = now
        if self.deadline is not None and now >= self.deadline:
            raise TimeoutError('OTA total deadline exhausted')
        return now

    def read(self, deadline):
        started = self.guard()
        ready = self.read_ready(max(0., min(.01, (deadline - started) / 1e9)))
        if not ready:
            self.guard()
            return []
        # Keep received evidence before any cancellation/boot/deadline check.
        chunk = self.read_bytes()
        stamp = self.clock()
        require(type(chunk) is bytes, 'Invalid serial read')
        self.raw_bytes += len(chunk)
        require(self.raw_bytes <= 2_000_000 and len(self.raw_log) < 100_000,
                'OTA receive evidence bound exceeded')
        self.raw_log.append({'read_started_ns': started, 'received_ns': stamp, 'hex': chunk.hex()})
        self.guard()
        require(stamp < deadline, 'Late OTA bytes')
        frames = self.parser.feed(chunk)
        require(not self.parser.discarded_bytes, 'Malformed OTA stream')
        return [(frame, stamp) for frame in frames]

    def quiet(self, duration_ns):
        require(not self.parser.buffer, 'Partial frame before quiet boundary')
        end = self.guard() + duration_ns
        while self.guard() < end:
            # No data is expected. Partial and complete stale frames both fail.
            old_bytes = self.raw_bytes
            self.read(min(end + 1_000_000, self.deadline))
            require(self.raw_bytes == old_bytes, 'Unexpected bytes at OTA boundary')
        require(not self.parser.buffer, 'Partial OTA boundary')

    def exchange(self, ordinal, kind, wire, attempt):
        require((kind, wire) == frame_for(self.image, self.motor_id, self.uid, ordinal),
                'Exchange must match the fixed image schedule')
        require(type(attempt) is int and 1 <= attempt <= (self.start_attempts if kind == 11 else 1),
                'Exchange attempt exceeds the declared scope')
        row = {'ordinal': ordinal, 'kind': kind, 'attempt': attempt,
               'wire_hex': wire.hex(), 'write_started_ns': None,
               'write_finished_ns': None, 'returned_bytes': None,
               'ack_received_ns': None}
        self.guard()
        self.tx_log.append(row)
        if kind == 11:
            self.report['bootloader_entry_attempted'] = True
            self.report['start_transmit_attempts'] += 1
        row['write_started_ns'] = self.clock()
        ack_timeout_ns = self.start_ack_timeout_s * 1_000_000_000 if kind == 11 else ACK_TIMEOUT_NS
        limit = min(row['write_started_ns'] + ack_timeout_ns, self.deadline)
        row['ack_timeout_ns'] = ack_timeout_ns
        self.raw.write_timeout = min(.1, (limit - row['write_started_ns']) / 1e9)
        try:
            row['returned_bytes'] = self.raw.write(wire)
        finally:
            row['write_finished_ns'] = self.clock()
        require(type(row['returned_bytes']) is int and row['returned_bytes'] == 17,
                'Partial OTA write; no further transmission permitted')
        self.guard()
        require(row['write_finished_ns'] < limit,
                'OTA write exceeded ACK deadline; transmission outcome is uncertain')
        while self.guard() < limit:
            frames = self.read(limit)
            require(len(frames) <= 1, 'Duplicate/extra OTA acknowledgements')
            if frames:
                ack, stamp = frames[0]
                validate_ack(ack, kind, self.motor_id)
                require(not self.parser.buffer, 'Extra partial OTA frame')
                row['ack_received_ns'] = stamp
                row['ack_wire_hex'] = ack.wire.hex()
                return
        row['ack_timeout'] = True
        raise MissingAcknowledgement('OTA acknowledgement missing; attempt exhausted')

    def run(self):
        require(not self._ran, 'Transfer is single use')
        self._ran = True
        self.started = self.clock()
        self.deadline = self.started + MAX_TRANSFER_NS
        self.report['status'] = 'INCOMPLETE'
        try:
            self.quiet(100_000_000)
            for ordinal in range(PACKETS + 3):
                if ordinal:
                    self.quiet(START_RETRY_GAP_NS if ordinal == 1 and self.start_attempts > 1
                               else GAP_NS)
                kind, wire = frame_for(self.image, self.motor_id, self.uid, ordinal)
                attempts = self.start_attempts if kind == 11 else 1
                for attempt in range(1, attempts + 1):
                    received_before = self.raw_bytes
                    writes_before = len(self.tx_log)
                    try:
                        self.exchange(ordinal, kind, wire, attempt)
                        break
                    except MissingAcknowledgement:
                        # Partial/late/invalid traffic is never discarded to permit
                        # a retry. Only a completely silent, fully written START.
                        if (kind != 11 or attempt == attempts or
                                self.raw_bytes != received_before or self.parser.buffer):
                            raise
                        self.tx_log[-1]['retry_reason'] = 'complete write; zero RX; START ACK timeout'
                        self.quiet(START_RETRY_GAP_NS)
                    finally:
                        if len(self.tx_log) > writes_before:
                            self.tx_log[-1]['received_bytes_during_attempt_and_gap'] = self.raw_bytes - received_before
                if kind == 13:
                    self.report['acknowledged_data_packets'] += 1
                if kind == 14:
                    self.report['end_acknowledged'] = True
                if ordinal < 2 or (kind == 13 and (ordinal - 1) % 1000 == 0) or kind == 14:
                    self.progress(dict(self.report))
            self.report['status'] = 'TRANSFER_ACK_COMPLETE_PENDING_VERSION'
        except BaseException as exc:
            self.report['failure'] = repr(exc)
        self.report['elapsed_s'] = (self.clock() - self.started) / 1e9
        self.report['transmit_attempts'] = len(self.tx_log)
        self.report['raw_bytes'] = self.raw_bytes
        self.report['residual_hex'] = self.parser.buffer.hex()
        self.report['discarded_bytes'] = self.parser.discarded_bytes
        return self.report
