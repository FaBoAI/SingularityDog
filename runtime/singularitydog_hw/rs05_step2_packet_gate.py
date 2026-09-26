"""Last software boundary before a supported raw two-degree UART write.

The caller must pin this source, its reviewed plan, the same-boot disabled
preflight and the two-bus binding. This gate accepts only the exact finite
wire sequence formed from fresh raw centers; it is not an emergency stop.
"""
import math
import struct
import threading
import time

from .can_readonly import ATParser, PARAMETERS, read_request
from .rs05_bus_transport import BUS_IDS
from .rs05_fullbody_step2 import (ACTIVE_TICKS, ALL_IDS, _diagnostic,
                                  _motion_phase, _plan, _review)
from .rs05_trial_protocol import (TrialPhase, enable_request, motion_request,
                                  stop_request, watchdog_setup_request)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class Step2PacketState:
    """Shared exact-wire gate; one six-axis batch per bus per 50 ms tick."""

    def __init__(self, review, *, clock=time.monotonic):
        require(type(review) is dict and review.get('supported_step_authorized') is True,
                'Explicit active raw-diagnostic review required')
        # The same validation used by the trajectory runner must pass before
        # any port is opened. In particular, old raw targets are forbidden.
        _review(review.get('motor_uids'), review, False)
        self.diagnostic = _diagnostic(review)
        self.review, self.clock = review, clock
        self.lock = threading.RLock()
        self._centers = {}
        self._plan = None
        self._bound_buses = set()
        self._watchdogs, self._enabled, self._enable_neutrals = set(), set(), set()
        self._next_tick = {bus: 0 for bus in BUS_IDS}
        self._next_motor_index = {bus: 0 for bus in BUS_IDS}
        self._first_enable = self._first_active = None
        self._terminal = False

    def abort(self):
        with self.lock:
            self._terminal = True

    def bind_centers(self, bus, centers):
        """Bind actual static-window centers from ``fullbody_bus_ready``."""
        require(bus in BUS_IDS and type(centers) is dict
                and set(centers) == set(BUS_IDS[bus])
                and all(type(i) is int and type(v) in (int, float) and math.isfinite(v)
                        for i, v in centers.items()),
                'Bind exactly six finite fresh raw centers on the named bus')
        with self.lock:
            require(not self._terminal and not self._enabled and bus not in self._bound_buses,
                    'Bind each bus exactly once before Enable')
            proposed = {**self._centers, **centers}
            proposed_buses = self._bound_buses | {bus}
            try:
                plan = _plan(proposed, self.review) if proposed_buses == set(BUS_IDS) else None
            except BaseException:
                self._terminal = True
                raise
            self._centers = proposed
            self._bound_buses = proposed_buses
            if plan is not None:
                self._plan = plan

    def audit(self):
        with self.lock:
            return {'bound_buses': sorted(self._bound_buses),
                    'enabled_ids': sorted(self._enabled),
                    'post_enable_zero_gain_ids': sorted(self._enable_neutrals),
                    'watchdog_ids': sorted(self._watchdogs),
                    'next_tick_by_bus': dict(self._next_tick),
                    'next_motor_index_by_bus': dict(self._next_motor_index),
                    'first_enable_monotonic_s': self._first_enable,
                    'first_active_monotonic_s': self._first_active,
                    'terminal': self._terminal}

    def allow(self, wire, bus):
        require(bus in BUS_IDS and type(wire) is bytes,
                'One bytes UART frame and an exact bus are required')
        parser = ATParser()
        frames = parser.feed(wire)
        require(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes,
                'Exactly one complete UART frame required')
        frame = frames[0]
        mid = frame.destination
        require(mid in BUS_IDS[bus] and frame.flags == 4 and len(frame.data) == 8,
                'Wrong bus, actuator or frame flags')
        with self.lock:
            if frame.kind == 4:
                require(wire == stop_request(phase=TrialPhase.STOP, motor_id=mid),
                        'Noncanonical STOP')
                if self._bound_buses or self._enabled:
                    self._terminal = True
                return
            if frame.kind == 0:
                require(wire == read_request(mid), 'Noncanonical identity read')
                return
            if frame.kind == 17:
                require(any(wire == read_request(mid, name) for name in PARAMETERS),
                        'Noncanonical parameter read')
                return
            require(not self._terminal, 'Raw-step transaction already stopped')
            now = self.clock()
            require(type(now) in (int, float) and math.isfinite(now),
                    'Invalid packet clock')
            if frame.kind == 18:
                require(not self._enabled and mid not in self._watchdogs
                        and wire == watchdog_setup_request(
                            phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid),
                        'Unexpected or repeated watchdog write')
                self._watchdogs.add(mid)
                return
            if frame.kind == 3:
                require(self._plan is not None and mid not in self._enabled
                        and self._next_tick == {bus: 0 for bus in BUS_IDS}
                        and wire == enable_request(phase=TrialPhase.ENABLE, motor_id=mid),
                        'Enable requires both bound buses and exact once per ID')
                if self._first_enable is None:
                    self._first_enable = now
                require(0 <= now - self._first_enable < 6., 'Enable stage timed out')
                self._enabled.add(mid)
                return
            require(frame.kind == 1, 'Packet kind is outside the raw-step profile')
            _, velocity, kp, kd = struct.unpack('>4H', frame.data)
            neutral = frame.can_id == ((1 << 24) | (32767 << 8) | mid) \
                and velocity == 32767 and kp == 0 and kd == 0
            if neutral:
                canonical = (mid in self._centers and wire == motion_request(
                    phase=TrialPhase.ZERO_GAIN, center_rad=self._centers[mid], motor_id=mid))
                if not self._enabled:
                    require(mid not in self._centers or canonical,
                            'Unexpected zero-gain command')
                else:
                    require(self._first_active is None and mid in self._enabled
                            and mid not in self._enable_neutrals and canonical,
                            'Unexpected zero-gain command')
                    self._enable_neutrals.add(mid)
                return
            require(self._enabled == set(ALL_IDS)
                    and self._enable_neutrals == set(ALL_IDS) and self._plan is not None,
                    'No active command before all twelve Enable and zero-gain replies')
            tick = self._next_tick[bus]
            index = self._next_motor_index[bus]
            require(tick < ACTIVE_TICKS and index < len(BUS_IDS[bus]),
                    'Finite raw-step packet budget exhausted')
            required_mid = BUS_IDS[bus][index]
            require(mid == required_mid, 'Raw-step six-axis order changed')
            phase = _motion_phase(self.review, mid)
            target = self._plan[tick][mid]
            expected = motion_request(phase=phase, center_rad=self._centers[mid],
                                      offset_rad=target-self._centers[mid], motor_id=mid)
            require(wire == expected, 'Raw-step packet differs from exact frozen trajectory')
            if self._first_active is None:
                self._first_active = now
            require(0 <= now - self._first_active < 10.5,
                    'Finite raw-step transmission budget expired')
            if index + 1 == len(BUS_IDS[bus]):
                self._next_tick[bus] = tick + 1
                self._next_motor_index[bus] = 0
            else:
                self._next_motor_index[bus] = index + 1


class Step2PacketPort:
    """Enforce the shared gate before physical ``serial.write``."""

    def __init__(self, serial_port, bus, state):
        require(bus in BUS_IDS and isinstance(state, Step2PacketState),
                'Exact bus and shared gate are required')
        self.port, self.bus, self.state = serial_port, bus, state

    def read(self, count):
        try:
            return self.port.read(count)
        except BaseException:
            self.state.abort()
            raise

    @property
    def in_waiting(self):
        try:
            return self.port.in_waiting
        except BaseException:
            self.state.abort()
            raise

    def write(self, wire):
        try:
            self.state.allow(wire, self.bus)
            written = self.port.write(wire)
            require(written == len(wire), 'Partial UART write; raw-step transaction ended')
            return written
        except BaseException:
            self.state.abort()
            raise


class ID7Step1PacketState(Step2PacketState):
    """The same exact-wire state machine, pinned to the ID7-only scope."""

    def __init__(self, review, *, clock=time.monotonic):
        require(_diagnostic(review) == 'id7-step1',
                'ID7-only packet state requires its distinct review scope')
        super().__init__(review, clock=clock)
