"""Offline-only one-slot receive-gap experiment. NOT imported by live runtime.

No FDs, device access, retransmission, stale-state inference or re-arm operation.
All original requests must already be fully written. Late Type2 bytes remain
attached to the original pending set; a new transaction cannot start here.
Even after resolution this returns RESAMPLE_REQUIRED, never permission to move.
20/40 ms are hypothesis bounds, not a change to the reviewed live watchdogs.
"""
from enum import Enum


class State(str, Enum):
    WAIT = 'WAIT_ORIGINAL_REPLIES'
    SKIP = 'SKIP_UPDATE_RECEIVE_ONLY'
    COMPLETE = 'COMPLETE_ON_TIME'
    RESAMPLE = 'RESAMPLE_REQUIRED'
    STOP = 'STOP_REQUIRED'


class ReceiveGap:
    def __init__(self, *, cycle_start_ns, requests, previous_slot_skipped=False,
                 skips_in_previous_99_slots=0, phase='steady'):
        # Requests are (bus, ID, original full-write completion timestamp).
        if type(cycle_start_ns) is not int or cycle_start_ns <= 0:
            raise ValueError('Positive monotonic cycle start required')
        if (type(previous_slot_skipped) is not bool or
                type(skips_in_previous_99_slots) is not int or
                not 0 <= skips_in_previous_99_slots <= 99 or phase not in ('steady', 'startup')):
            raise ValueError('Explicit phase and valid skip history required')
        self.soft = cycle_start_ns + 20_000_000
        self.hard = self.soft + 20_000_000
        self.last_ns = cycle_start_ns
        self.pending = {}
        for bus, mid, sent in requests:
            valid_ids = range(1, 7) if bus == 'front' else range(7, 13) if bus == 'rear' else ()
            if (type(mid) is not int or mid not in valid_ids or (bus, mid) in self.pending or
                    type(sent) is not int or not cycle_start_ns <= sent < self.soft):
                raise ValueError('Unique fully written requests before soft deadline required')
            self.pending[(bus, mid)] = sent
        if not self.pending:
            raise ValueError('Original pending request set required')
        self.buffers = {'front': bytearray(), 'rear': bytearray()}
        self.allow_gap = (phase == 'steady' and not previous_slot_skipped and
                          skips_in_previous_99_slots == 0)
        self.state, self.reason, self.skipped = State.WAIT, '', False

    def stop(self, reason):
        self.state, self.reason = State.STOP, reason
        return self.state

    def tick(self, now_ns):
        if type(now_ns) is not int or now_ns < self.last_ns:
            return self.stop('invalid_or_backwards_clock')
        self.last_ns = now_ns
        if self.state in (State.STOP, State.COMPLETE, State.RESAMPLE):
            return self.state
        if now_ns >= self.hard:
            return self.stop('original_reply_still_missing_after_one_slot')
        if now_ns >= self.soft:
            if not self.allow_gap:
                return self.stop('startup_or_skip_frequency_limit')
            self.skipped, self.state = True, State.SKIP
        return self.state

    def feed(self, bus, raw, now_ns):
        self.tick(now_ns)
        if self.state == State.STOP:
            return self.state
        if self.state in (State.COMPLETE, State.RESAMPLE):
            return self.stop('unexpected_bytes_after_transaction')
        if bus not in self.buffers or not isinstance(raw, bytes) or not raw:
            return self.stop('invalid_receive_event')
        buffer = self.buffers[bus]
        if len(buffer) + len(raw) > 4096:
            return self.stop('receive_overflow')
        buffer.extend(raw)
        while len(buffer) >= 17:
            wire = bytes(buffer[:17])
            if (wire[:2] != b'AT' or wire[5] & 7 != 4 or wire[6] != 8 or wire[15:] != b'\r\n'):
                return self.stop('malformed_frame')
            cid = int.from_bytes(wire[2:6], 'big') >> 3
            key = (bus, (cid >> 8) & 255)
            if (cid >> 24 != 2 or cid & 255 != 0xfd or (cid >> 22) & 3 != 2 or
                    (cid >> 16) & 63 or wire[7:10] == b'\x00\xc4\x56'):
                return self.stop('fault_or_wrong_reply_type_mode_host')
            if key not in self.pending or now_ns < self.pending[key]:
                return self.stop('duplicate_foreign_or_noncausal_reply')
            del self.pending[key]
            del buffer[:17]
        if buffer and not any(key[0] == bus for key in self.pending):
            return self.stop('unattributed_partial_tail')
        if not self.pending:
            self.state = State.RESAMPLE if self.skipped else State.COMPLETE
        return self.state

    def attempt_new_command(self):
        return self.stop('new_command_prohibited_in_receive_gap_experiment')

    def transport_error(self, reason):
        return self.stop('transport_error:' + str(reason))

    def result(self):
        return {'state': self.state.value, 'reason': self.reason,
                'skipped_slots': int(self.skipped), 'unresolved': sorted(self.pending),
                'new_motor_writes_allowed': False, 'policy_inference_allowed': False,
                'live_resume_approved': False, 'hardware_opened': False}
