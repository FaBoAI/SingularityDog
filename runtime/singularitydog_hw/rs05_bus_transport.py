"""One UART/parser for exactly the front or rear six-axis RS05 bus.

This transport does not open ports, combine UARTs, choose targets, or authorize
motion. Its caller owns one serial port per instance and coordinates both buses.
Like LegTrialTransport, replies have an 80ms deadline after the last batch
write, with at least 5ms between completed writes. That is not a 50ms cycle
guarantee: the coordinator must enforce its own complete-cycle deadline.
Feedback age remains at most 100ms; motion/enable guard hooks let the caller
recheck every selected joint immediately before each active write.
"""
from dataclasses import asdict
import time

from .can_readonly import ATParser, PARAMETERS, decode_reply, matches, read_request
from .rs05_joint_trial import MAX_FEEDBACK_AGE_S
from .rs05_trial_protocol import (TrialPhase, decode_type2, enable_request,
                                 motion_request, stop_request, watchdog_setup_request)


BUS_IDS = {'front': (1, 2, 3, 4, 5, 6), 'rear': (7, 8, 9, 10, 11, 12)}
MIN_TX_INTERVAL_S = .005
EXCHANGE_TIMEOUT_S = .08
REPLY_QUIET_S = .004
MOTION_PHASES = (TrialPhase.ZERO_GAIN, TrialPhase.POSITION, TrialPhase.POSITION_STEP2,
                 TrialPhase.POSITION_VISIBLE, TrialPhase.POSITION_STEP5,
                 TrialPhase.POSITION_STEP5_KP4, TrialPhase.POSITION_ROLE_THIGH_KP6,
                 TrialPhase.POSITION_ROLE_THIGH_KP12,
                 TrialPhase.POSITION_ROLE_HIP_HOLD_KP12,
                 TrialPhase.POSITION_STEP5_RR_HIP_KP6,
                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP6,
                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                 TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10)


def selected_bus_ids(ids):
    try:
        ids = tuple(ids)
    except TypeError as error:
        raise ValueError('Select exactly front IDs1..6 or rear IDs7..12') from error
    if any(type(i) is not int for i in ids) or ids not in BUS_IDS.values():
        raise ValueError('Select exactly ordered front IDs1..6 or rear IDs7..12')
    return ids


class BusTrialTransport:
    """Independent six-axis transport; the three-axis API remains unchanged."""
    def __init__(self, serial_port, emit, check_interrupt=lambda: None, *, ids, wait=None):
        self.ids = selected_bus_ids(ids)
        self.bus_name = next(name for name, members in BUS_IDS.items() if members == self.ids)
        if (not callable(getattr(serial_port, 'read', None))
                or not callable(getattr(serial_port, 'write', None))):
            raise ValueError('One serial port object is required per bus; UART streams cannot be combined')
        self.serial, self.emit, self.check_interrupt = serial_port, emit, check_interrupt
        self.parser, self.latest = ATParser(), {}
        self.relaxed_log, self.active_deadline, self.feedback_guard = False, None, None
        self.pre_send_guard = self.pre_enable_guard = None
        self.fault_latched = None
        self.wait = (lambda seconds: time.sleep(seconds)) if wait is None else wait
        self.last_write_finished_s = None

    def _latch(self, error):
        self.fault_latched = self.fault_latched or repr(error)

    def _subset(self, ids):
        try:
            ids = tuple(ids)
        except TypeError as error:
            raise ValueError('IDs must be a nonempty unique selected-bus subset') from error
        if (not ids or any(type(i) is not int for i in ids)
                or len(set(ids)) != len(ids) or not set(ids) <= set(self.ids)):
            raise ValueError('IDs must be a nonempty unique selected-bus subset')
        return ids

    def _validated_wire(self, wire):
        if not isinstance(wire, bytes):
            raise ValueError('Canonical trial wire must be bytes')
        parser = ATParser()
        frames = parser.feed(wire)
        if (len(frames) != 1 or parser.buffer or parser.discarded_bytes
                or frames[0].flags != 4 or len(frames[0].data) != 8
                or frames[0].destination not in self.ids
                or frames[0].kind not in (0, 1, 3, 4, 17, 18)):
            raise ValueError('Only selected-bus canonical trial frames are allowed')
        frame = frames[0]
        mid = frame.destination
        if frame.kind == 1:
            # Position is caller-planned raw uint16. All other bits must match
            # the existing codec's zero-FF, zero-velocity, fixed-gain phases.
            candidates = [motion_request(phase=phase, center_rad=0., motor_id=mid)
                          for phase in MOTION_PHASES
                          if (phase != TrialPhase.POSITION_STEP5_RR_HIP_KP6 or mid == 9)
                          and (phase != TrialPhase.POSITION_ROLE_FRONT_HIP_KP6
                               or mid in (3, 6))
                          and (phase != TrialPhase.POSITION_ROLE_FRONT_HIP_KP12
                               or mid in (3, 6))
                          and (phase != TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10
                               or mid in (3, 6))
                          and (phase != TrialPhase.POSITION_ROLE_THIGH_KP6
                               or mid in (2, 5, 8, 11))
                          and (phase != TrialPhase.POSITION_ROLE_THIGH_KP12
                               or mid in (2, 5, 8, 11))
                          and (phase != TrialPhase.POSITION_ROLE_HIP_HOLD_KP12
                               or mid in (3, 6, 9, 12))]
            valid = any(wire[:7] == candidate[:7] and wire[9:] == candidate[9:]
                        for candidate in candidates)
        elif frame.kind == 17:
            valid = any(wire == read_request(mid, name) for name in PARAMETERS)
        else:
            canonical = {0: lambda: read_request(mid),
                3: lambda: enable_request(phase=TrialPhase.ENABLE, motor_id=mid),
                4: lambda: stop_request(phase=TrialPhase.STOP, motor_id=mid),
                18: lambda: watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid)}
            valid = wire == canonical[frame.kind]()
        if not valid:
            raise ValueError('Noncanonical selected-bus trial command')
        return frame

    def pace_transmit(self):
        """Space from write completion, even when the preceding write failed."""
        if self.last_write_finished_s is None:
            return
        next_write = self.last_write_finished_s + MIN_TX_INTERVAL_S
        while True:
            remaining = next_write - time.monotonic()
            if remaining <= 0:
                return
            self.wait(remaining)

    def log(self, event):
        try:
            self.emit({'monotonic_ns': time.monotonic_ns(), 'bus': self.bus_name, **event})
        except BaseException as error:
            if not self.relaxed_log:
                self._latch(error)
                raise

    def receive(self):
        try:
            chunk = self.serial.read(min(max(self.serial.in_waiting, 1), 2048))
        except BaseException as error:
            self._latch(error)
            raise
        when = time.monotonic()
        if not chunk:
            return []
        self.log({'kind': 'can_rx_bytes', 'hex': chunk.hex()})
        frames = self.parser.feed(chunk)
        for frame in frames:
            self.log({'kind': 'can_rx_frame', **frame.record()})
            if (frame.kind in (0, 2, 17, 21) and 1 <= frame.source <= 12
                    and frame.source not in self.ids):
                error = RuntimeError(f'ID{frame.source} reply belongs to another bus')
                self._latch(error)
                raise error
            if frame.source not in self.ids or frame.kind not in (2, 21):
                continue
            try:
                if frame.kind == 21:
                    raise RuntimeError(f'ID{frame.source} reported Type21 fault')
                value = decode_type2(frame, motor_id=frame.source)
                self.latest[frame.source] = (value, when)
                if value.fault_bits:
                    raise RuntimeError(f'ID{frame.source} feedback fault')
                if self.feedback_guard is not None and not self.relaxed_log:
                    self.feedback_guard(value, when, frame.source)
            except BaseException as error:
                self._latch(error)
                if not self.relaxed_log:
                    raise
        if self.parser.discarded_bytes and not self.relaxed_log:
            error = RuntimeError('Parser discarded bytes')
            self._latch(error)
            raise error
        return [(frame, when) for frame in frames]

    def fresh_boundary(self):
        if self.fault_latched and not self.relaxed_log:
            raise RuntimeError(self.fault_latched)
        end = time.monotonic() + .03
        while self.serial.in_waiting:
            self.receive()
            if time.monotonic() > end:
                error = RuntimeError('Input backlog')
                self._latch(error)
                raise error
        if self.parser.buffer or self.parser.discarded_bytes:
            error = RuntimeError('Partial or discarded serial frame before command')
            self._latch(error)
            raise error

    def send(self, wire):
        frame = self._validated_wire(wire)
        if frame.kind != 4:
            self.check_interrupt()
            if self.fault_latched:
                raise RuntimeError(self.fault_latched)
        self.pace_transmit()
        if frame.kind != 4:
            self.check_interrupt()
            if self.fault_latched:
                raise RuntimeError(self.fault_latched)
        if frame.kind in (1, 3) and self.active_deadline is not None and time.monotonic() >= self.active_deadline:
            raise RuntimeError('Active trial deadline reached')
        if frame.kind == 1 and frame.data[4:8] != bytes(4) and self.pre_send_guard is not None:
            self.pre_send_guard()
        if frame.kind == 3 and self.pre_enable_guard is not None:
            self.pre_enable_guard()
        if frame.kind in (1, 3) and self.active_deadline is not None and time.monotonic() >= self.active_deadline:
            raise RuntimeError('Active trial deadline reached after pre-send guards')
        try:
            written = self.serial.write(wire)
        except BaseException as error:
            self._latch(error)
            raise
        finally:
            self.last_write_finished_s = time.monotonic()
        if written != len(wire):
            error = IOError('Partial serial write')
            self._latch(error)
            raise error
        self.log({'kind': 'can_tx', 'hex': wire.hex(), 'type': frame.kind, 'motor_id': frame.destination})

    def exchange_many(self, wires, expected_ids, accept):
        expected_ids = self._subset(expected_ids)
        wires = tuple(wires)
        if not wires:
            raise ValueError('A nonempty canonical selected-bus command batch is required')
        for wire in wires:
            self._validated_wire(wire)
        try:
            self.fresh_boundary()
            for wire in wires:
                self.send(wire)
            batch_completed = time.monotonic()
            reply_deadline = batch_completed + EXCHANGE_TIMEOUT_S
            deadline = reply_deadline
            if self.active_deadline is not None:
                deadline = min(deadline, self.active_deadline)
            found = {}
            while time.monotonic() < deadline:
                self.check_interrupt()
                for frame, when in self.receive():
                    if frame.source in expected_ids:
                        value = accept(frame)
                        if value is not None:
                            found[frame.source] = (value, when)
                now = time.monotonic()
                if now >= deadline:
                    break
                if set(found) == set(expected_ids) and now - max(t for _, t in found.values()) >= REPLY_QUIET_S:
                    if any(not 0 <= now - when <= MAX_FEEDBACK_AGE_S for _, when in found.values()):
                        raise RuntimeError('Stale selected-bus reply')
                    return found
            timed_out = time.monotonic()
            quiet_elapsed = None if not found else timed_out - max(t for _, t in found.values())
            diagnostics = {
                'bus': self.bus_name,
                'deadline_source': 'active_deadline' if deadline < reply_deadline else 'reply_timeout',
                'effective_deadline_monotonic_s': deadline,
                'batch_completed_monotonic_s': batch_completed,
                'effective_reply_budget_s': deadline - batch_completed,
                'timed_out_monotonic_s': timed_out,
                'received_ids': sorted(found),
                'missing_ids': sorted(set(expected_ids) - set(found)),
                'quiet_elapsed_s': quiet_elapsed,
                'quiet_required_s': REPLY_QUIET_S,
            }
            quiet_text = 'none' if quiet_elapsed is None else f'{quiet_elapsed:.6f}'
            error = TimeoutError(
                f"Selected bus reply timeout ({self.bus_name}): "
                f"deadline_source={diagnostics['deadline_source']}, "
                f"effective_deadline_monotonic_s={deadline:.9f}, "
                f"effective_reply_budget_s={deadline - batch_completed:.6f}, "
                f"received_ids={diagnostics['received_ids']}, missing_ids={diagnostics['missing_ids']}, "
                f"quiet_elapsed_s={quiet_text}, quiet_required_s={REPLY_QUIET_S:.6f}")
            error.diagnostics = diagnostics
            raise error
        except BaseException as error:
            self._latch(error)
            raise

    def parameter(self, motor_id, name=None):
        self._subset((motor_id,))

        def accept(frame):
            if matches(frame, motor_id, name):
                value = decode_reply(frame, motor_id, name)
                if not value['ok']:
                    raise RuntimeError(f'ID{motor_id} parameter rejected: {name}')
                return value

        return self.exchange_many([read_request(motor_id, name)], (motor_id,), accept)[motor_id][0]

    def feedback_many(self, wires, expected_ids):
        def accept(frame):
            if frame.kind == 2:
                return decode_type2(frame, motor_id=frame.source)

        found = self.exchange_many(wires, expected_ids, accept)
        for motor_id, (value, when) in found.items():
            self.log({'kind': 'bus_trial_feedback', 'motor_id': motor_id,
                      'received_monotonic_s': when, **asdict(value)})
        now = time.monotonic()
        if any(not 0 <= now - when <= MAX_FEEDBACK_AGE_S for _, when in found.values()):
            error = RuntimeError('Stale selected-bus feedback')
            self._latch(error)
            raise error
        return found

    def stop_all(self, ids=None):
        """Attempt every paced STOP before reading; isolate all write failures."""
        ids = self.ids if ids is None else self._subset(ids)
        self.relaxed_log, self.feedback_guard, self.pre_send_guard = True, None, None
        self.pre_enable_guard = None
        reports = {i: {'confirmed': False, 'feedback': None, 'error': None} for i in ids}
        boundary_valid = True
        try:
            self.fresh_boundary()
        except BaseException:
            boundary_valid, self.parser = False, ATParser()
        sent = {}
        for motor_id in ids:
            try:
                self.send(stop_request(phase=TrialPhase.STOP, motor_id=motor_id))
                sent[motor_id] = time.monotonic()
            except BaseException as error:
                reports[motor_id]['error'] = 'Stop write failed: ' + repr(error)
        deadline = time.monotonic() + EXCHANGE_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                received = self.receive()
            except BaseException as error:
                for entry in reports.values():
                    entry['confirmed'] = False
                    entry['error'] = entry['error'] or 'Stop read failed: ' + repr(error)
                break
            for frame, when in received:
                i = frame.source
                if i not in reports or frame.kind not in (2, 21):
                    continue
                try:
                    if frame.kind == 21:
                        raise RuntimeError('Type21 fault during stop')
                    value = decode_type2(frame, motor_id=i)
                    reports[i]['feedback'] = asdict(value)
                    if value.fault_bits:
                        raise RuntimeError('Fault bits during stop')
                    if when >= deadline or not 0 <= time.monotonic() - when <= MAX_FEEDBACK_AGE_S:
                        raise RuntimeError('Stale or late stop feedback')
                    reports[i]['confirmed'] = bool(boundary_valid and i in sent and when >= sent[i]
                        and value.mode_state == 0 and reports[i]['error'] is None)
                except BaseException as error:
                    reports[i]['confirmed'] = False
                    reports[i]['error'] = repr(error)
        for entry in reports.values():
            if not boundary_valid or self.parser.discarded_bytes or self.parser.buffer:
                entry['confirmed'] = False
                entry['error'] = entry['error'] or 'Stop sent; fresh clean reply boundary unavailable'
            if not entry['confirmed']:
                entry['error'] = entry['error'] or 'No fresh reset0/fault0 confirmation'
        self.relaxed_log = False
        return reports
