"""Offline permission guard. It neither sends motor commands nor stops hardware.

Call evaluate with current inputs before every prospective control operation.
There is no background watchdog: a stalled caller cannot be stopped by this class.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
import math
from numbers import Real
import time
from types import MappingProxyType


class GuardState(str, Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    FAULT = "FAULT"
    CLOSED = "CLOSED"


def _finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class GuardConfig:
    """Required sources and freshness bounds, supplied explicitly by the caller."""

    sensor_max_age_s: Mapping[str, float]
    deadman_max_age_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.sensor_max_age_s, Mapping) or not self.sensor_max_age_s:
            raise ValueError("At least one required sensor and freshness bound is required")
        ages = dict(self.sensor_max_age_s)
        for name, age in ages.items():
            if not isinstance(name, str) or not name.strip() or name != name.strip():
                raise ValueError("Sensor names must be nonempty strings without edge whitespace")
            if not _finite_number(age) or age <= 0:
                raise ValueError("Sensor freshness bounds must be finite and positive")
        if not _finite_number(self.deadman_max_age_s) or self.deadman_max_age_s <= 0:
            raise ValueError("Deadman freshness bound must be finite and positive")
        object.__setattr__(self, "sensor_max_age_s", MappingProxyType(ages))


@dataclass(frozen=True)
class Prerequisites:
    """Caller assertions about verified evidence; the guard cannot verify it itself."""

    calibration_verified: bool = False
    equipment_verified: bool = False
    motor_models_verified: bool = False
    physical_stop_verified: bool = False


@dataclass(frozen=True)
class SensorSample:
    """Timestamp must use the guard's monotonic clock domain, not wall time."""

    timestamp: float
    values: tuple[float, ...]
    healthy: bool = False


@dataclass(frozen=True)
class DeadmanStatus:
    timestamp: float
    pressed: bool = False


@dataclass(frozen=True)
class SafetyInputs:
    prerequisites: Prerequisites = field(default_factory=Prerequisites)
    sensors: Mapping[str, SensorSample] = field(default_factory=dict)
    deadman: DeadmanStatus | None = None


@dataclass(frozen=True)
class GuardDecision:
    state: GuardState
    permitted: bool
    reasons: tuple[str, ...]
    checked_at: float | None
    valid_until: float | None = None
    stop_intent: bool = False


@dataclass(frozen=True)
class GuardEvent:
    kind: str
    reasons: tuple[str, ...]
    timestamp: float | None
    previous_state: GuardState
    state: GuardState
    stop_intent: bool = False


class SafetyGuard:
    """Single-caller state machine; returned permission is only a checked snapshot.

    A fault must be acknowledged in order: disarm(), clear_fault(), arm(inputs).
    Neither fresh data nor clearing a fault grants permission. close() is terminal.
    """

    def __init__(self, config: GuardConfig, *, clock: Callable[[], float] = time.monotonic):
        if not isinstance(config, GuardConfig) or not callable(clock):
            raise ValueError("A validated GuardConfig and callable clock are required")
        self._config = config
        self._clock = clock
        self._state = GuardState.DISARMED
        self._fault_reasons: tuple[str, ...] = ()
        self._last_now: float | None = None
        self._events: list[GuardEvent] = []

    @property
    def state(self) -> GuardState:
        return self._state

    @property
    def fault_reasons(self) -> tuple[str, ...]:
        return self._fault_reasons

    def drain_events(self) -> tuple[GuardEvent, ...]:
        events = tuple(self._events)
        self._events.clear()
        return events

    def _now(self, *, reset: bool = False) -> tuple[float | None, tuple[str, ...]]:
        try:
            now = self._clock()
        except Exception:
            return None, ("clock_unavailable",)
        if not _finite_number(now):
            return None, ("clock_nonfinite_or_invalid",)
        now = float(now)
        if not reset and self._last_now is not None and now < self._last_now:
            return now, ("clock_moved_backwards",)
        self._last_now = now
        return now, ()

    def _transition(self, state: GuardState, kind: str, reasons: tuple[str, ...],
                    now: float | None, *, stop: bool = False) -> None:
        previous = self._state
        self._state = state
        self._events.append(GuardEvent(kind, reasons, now, previous, state, stop))

    def _deny(self, reasons: tuple[str, ...], now: float | None,
              *, stop: bool = False) -> GuardDecision:
        return GuardDecision(self._state, False, reasons, now,
                             stop_intent=stop or bool(self._fault_reasons))

    def _fault(self, reasons: tuple[str, ...], now: float | None) -> GuardDecision:
        self._fault_reasons = reasons
        self._transition(GuardState.FAULT, "fault_latched", reasons, now, stop=True)
        return self._deny(reasons, now, stop=True)

    def _check(self, inputs: SafetyInputs, now: float | None) -> tuple[tuple[str, ...], float | None]:
        reasons: list[str] = []
        deadlines: list[float] = []
        if not isinstance(inputs, SafetyInputs):
            return ("invalid_safety_inputs",), None
        prerequisites = inputs.prerequisites
        if not isinstance(prerequisites, Prerequisites):
            reasons.append("invalid_prerequisites")
        else:
            for name in Prerequisites.__dataclass_fields__:
                if getattr(prerequisites, name) is not True:
                    reasons.append("unverified:" + name)

        def timestamp_check(timestamp: object, max_age: float, label: str) -> None:
            if not _finite_number(timestamp):
                reasons.append(label + ":invalid_timestamp")
            elif now is not None:
                if timestamp > now:
                    reasons.append(label + ":future_timestamp")
                else:
                    deadline = timestamp + max_age
                    if not _finite_number(deadline):
                        reasons.append(label + ":invalid_deadline")
                    elif now >= deadline:
                        reasons.append(label + ":stale")
                    else:
                        deadlines.append(float(deadline))

        sensors = inputs.sensors
        if not isinstance(sensors, Mapping):
            reasons.append("invalid_sensor_mapping")
            sensors = {}
        for name, max_age in self._config.sensor_max_age_s.items():
            sample = sensors.get(name)
            label = "sensor:" + name
            if not isinstance(sample, SensorSample):
                reasons.append(label + ":missing_or_invalid")
                continue
            if sample.healthy is not True:
                reasons.append(label + ":unhealthy")
            if (not isinstance(sample.values, (tuple, list)) or not sample.values or
                    not all(_finite_number(value) for value in sample.values)):
                reasons.append(label + ":missing_or_nonfinite_values")
            timestamp_check(sample.timestamp, max_age, label)
        deadman = inputs.deadman
        if not isinstance(deadman, DeadmanStatus):
            reasons.append("deadman:missing_or_invalid")
        else:
            if deadman.pressed is not True:
                reasons.append("deadman:released_or_invalid")
            timestamp_check(deadman.timestamp, self._config.deadman_max_age_s, "deadman")
        return tuple(reasons), min(deadlines) if deadlines else None

    def evaluate(self, inputs: SafetyInputs) -> GuardDecision:
        if self._state == GuardState.CLOSED:
            return self._deny(("closed",), None)
        now, clock_reasons = self._now()
        if self._fault_reasons:
            return self._deny(self._fault_reasons + ("fault_latched",), now)
        reasons, deadline = self._check(inputs, now)
        reasons = clock_reasons + reasons
        if self._state == GuardState.ARMED:
            if reasons:
                return self._fault(reasons, now)
            return GuardDecision(self._state, True, (), now, deadline)
        return self._deny(reasons + ("explicit_arm_required",), now)

    def arm(self, inputs: SafetyInputs) -> GuardDecision:
        if self._state in (GuardState.ARMED, GuardState.CLOSED) or self._fault_reasons:
            return self.evaluate(inputs)
        now, clock_reasons = self._now()
        reasons, deadline = self._check(inputs, now)
        reasons = clock_reasons + reasons
        if reasons:
            self._transition(GuardState.DISARMED, "arm_denied", reasons, now)
            return self._deny(reasons, now)
        self._transition(GuardState.ARMED, "armed", ("explicit_arm",), now)
        return GuardDecision(self._state, True, (), now, deadline)

    def disarm(self, reason: str = "explicit_disarm") -> GuardDecision:
        if self._state == GuardState.CLOSED:
            return self._deny(("closed",), None)
        now, clock_reasons = self._now()
        stop = self._state == GuardState.ARMED or bool(self._fault_reasons)
        if not isinstance(reason, str) or not reason.strip():
            reason = "invalid_disarm_reason"
        reasons = (reason,) + clock_reasons
        self._transition(GuardState.DISARMED, "disarmed", reasons, now, stop=stop)
        return self._deny(reasons + self._fault_reasons, now, stop=stop)

    def clear_fault(self) -> GuardDecision:
        if self._state == GuardState.CLOSED:
            return self._deny(("closed",), None)
        if self._state != GuardState.DISARMED:
            now, _ = self._now()
            if self._state == GuardState.ARMED:
                return self._fault(("clear_fault_requires_disarmed",), now)
            return self._deny(self._fault_reasons + ("disarm_before_clear_fault",), now)
        # Explicit fault acknowledgement permits a new monotonic-clock epoch.
        now, reasons = self._now(reset=True)
        if reasons:
            return self._deny(reasons + self._fault_reasons, now)
        self._fault_reasons = ()
        self._transition(GuardState.DISARMED, "fault_cleared", ("explicit_arm_required",), now)
        return self._deny(("explicit_arm_required",), now)

    def close(self) -> GuardDecision:
        if self._state == GuardState.CLOSED:
            return self._deny(("closed",), None)
        now, clock_reasons = self._now()
        stop = self._state == GuardState.ARMED or bool(self._fault_reasons)
        self._transition(GuardState.CLOSED, "closed", ("closed",) + clock_reasons, now, stop=stop)
        return self._deny(("closed",) + clock_reasons, now, stop=stop)
