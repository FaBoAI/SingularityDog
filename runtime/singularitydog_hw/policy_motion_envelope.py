"""Pure twelve-axis shaping and monitoring for reviewed supported trials.

All tuples use CAN ID 1..12 order, in calibrated model coordinates.  This module
does not encode frames, enable motors, select physical limits, or permit an
unsupported trial.  The caller must provide reviewed per-axis limits and map
model positions back to the current, UID-bound raw-angle branch.

Position references follow continuous, bounded-velocity/acceleration trajectories.
MIT velocity references and feedforward torque remain zero: the internal reference
velocity is bookkeeping, not a second damping target.  PD torque is an estimate
from the new command and measured state, NOT a physical motor torque cap.

request_stop() stops accepting policy targets.  Starting at the last command it
brakes each reference continuously, then decreases gains with a quintic ramp.
Completion is no later than max(v_limit/a_limit) + stop_duration_s from the last
command, plus at most one sample interval to observe it.  The caller must keep
support in place, continue fresh samples, and issue the hardware STOP at completion.
A MotionFault is latched and requires an immediate caller-managed hardware STOP;
no graceful ramp is attempted after a fault.  Recreating this object is not fault
recovery authorization.
"""

from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional, Sequence, Tuple


AXIS_COUNT = 12


class MotionFault(ValueError):
    """A hard monitoring or input fault; the owner must stop hardware output."""


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise MotionFault(f"{name}: finite real number required")
    try:
        converted = float(value)
    except (ValueError, OverflowError) as exc:
        raise MotionFault(f"{name}: finite real number required") from exc
    if not math.isfinite(converted):
        raise MotionFault(f"{name}: finite real number required")
    return converted


def _positive(value, name):
    value = _number(value, name)
    if value <= 0:
        raise MotionFault(f"{name}: positive value required")
    return value


def _vector(values, name):
    # The live decoder supplies exact float tuples. Validate those in C-level
    # loops while retaining the generic conversion/rejection path for every
    # other input type (including bool, int, and float subclasses).
    if (type(values) is tuple and len(values) == AXIS_COUNT and
            all(type(value) is float for value in values) and
            all(map(math.isfinite, values))):
        return values
    if isinstance(values, (str, bytes)):
        raise MotionFault(f"{name}: twelve numeric values required")
    try:
        result = tuple(_number(value, name) for value in values)
    except TypeError as exc:
        raise MotionFault(f"{name}: twelve numeric values required") from exc
    if len(result) != AXIS_COUNT:
        raise MotionFault(f"{name}: exactly twelve axes required")
    return result


@dataclass(frozen=True)
class AxisLimits:
    lower_rad: float
    upper_rad: float
    kp: float
    kd: float
    max_command_velocity_rad_s: float
    max_command_acceleration_rad_s2: float
    max_tracking_error_rad: float
    max_measured_velocity_rad_s: float
    max_measured_torque_nm: float
    max_temperature_c: float
    max_estimated_pd_torque_nm: float
    max_displacement_from_start_rad: float

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = _number(getattr(self, name), name)
            object.__setattr__(self, name, value)
        if self.lower_rad >= self.upper_rad:
            raise MotionFault("AxisLimits: lower_rad must be below upper_rad")
        if self.kp < 0 or self.kd < 0:
            raise MotionFault("AxisLimits: gains must be nonnegative")
        for name in self.__dataclass_fields__:
            if name not in ("lower_rad", "upper_rad", "kp", "kd"):
                _positive(getattr(self, name), name)


@dataclass(frozen=True)
class MotionSample:
    q_model_rad: Tuple[float, ...]
    velocity_rad_s: Tuple[float, ...]
    torque_nm: Tuple[float, ...]
    temperature_c: Tuple[float, ...]
    monotonic_s: float

    def __post_init__(self):
        for name in ("q_model_rad", "velocity_rad_s", "torque_nm", "temperature_c"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        stamp = _number(self.monotonic_s, "sample monotonic_s")
        if stamp < 0:
            raise MotionFault("sample monotonic_s must be nonnegative")
        object.__setattr__(self, "monotonic_s", stamp)


@dataclass(frozen=True)
class MotionCommand:
    q_model_rad: Tuple[float, ...]
    kp: Tuple[float, ...]
    kd: Tuple[float, ...]
    velocity_reference_rad_s: Tuple[float, ...]
    feedforward_torque_nm: Tuple[float, ...]
    command_velocity_rad_s: Tuple[float, ...]
    tracking_error_rad: Tuple[float, ...]
    estimated_pd_torque_nm: Tuple[float, ...]
    gain_scale: float
    phase: str
    stop_stage: Optional[str]
    monotonic_s: float


def _quintic_fraction(value):
    # Bounding elapsed fractions is not clamping an invalid policy target.
    if value <= 0:
        return 0.
    if value >= 1:
        return 1.
    # Near one, floating-point cancellation can produce 1 + a few ulps and
    # make a stopping gain negative. Bound this dimensionless ramp only;
    # policy targets and measured limits continue to reject invalid values.
    return min(1., max(0., value ** 3 * (10. + value * (-15. + 6. * value))))


def _advance_reference(q, velocity, target, dt, vmax, acceleration):
    """Integrate a time-optimal bounded-acceleration path toward a fixed target.

    Replanning at every call preserves q and velocity, including reversals.  If a
    new target is inside the current braking distance, the path brakes past it
    before returning.  Its excursion is bounded by the previous stopping point
    and the valid new target; both are inside the supported position interval.
    """
    if q == target and velocity == 0:
        return target, 0.
    stopping_point = q + velocity * abs(velocity) / (2. * acceleration)
    difference = target - stopping_point
    if abs(difference) <= 1e-14:
        if velocity == 0:
            return target, 0.
        elapsed = min(dt, abs(velocity) / acceleration)
        signed_a = -math.copysign(acceleration, velocity)
        if dt >= abs(velocity) / acceleration:
            return target, 0.
        return q + velocity * elapsed + .5 * signed_a * elapsed ** 2, velocity + signed_a * elapsed
    direction = math.copysign(1., difference)
    forward_velocity = direction * velocity
    distance = direction * (target - q)
    peak = math.sqrt(max(0., acceleration * distance + .5 * velocity ** 2))
    top = min(peak, vmax)
    accelerate_time = max(0., (top - forward_velocity) / acceleration)
    brake_time = top / acceleration
    cruise_time = 0.
    if peak > vmax:
        accelerate_distance = (vmax ** 2 - forward_velocity ** 2) / (2. * acceleration)
        brake_distance = vmax ** 2 / (2. * acceleration)
        cruise_time = max(0., (distance - accelerate_distance - brake_distance) / vmax)
    total = accelerate_time + cruise_time + brake_time
    if dt >= total:
        return target, 0.
    for duration, signed_a in ((accelerate_time, direction * acceleration),
                               (cruise_time, 0.), (brake_time, -direction * acceleration)):
        elapsed = min(dt, duration)
        q += velocity * elapsed + .5 * signed_a * elapsed ** 2
        velocity += signed_a * elapsed
        dt -= elapsed
        if dt <= 0:
            break
    return q, velocity


class PolicyMotionEnvelope:
    def __init__(self, limits: Sequence[AxisLimits], initial_sample: MotionSample, *,
                 now_s, startup_duration_s, stop_duration_s,
                 max_sample_age_s, max_sample_gap_s, startup_damping_duration_s=None):
        self._fault_reason = None
        self._stop_requested = False
        self._stop_anchor = None
        try:
            self.limits = tuple(limits)
        except TypeError as exc:
            raise MotionFault("Twelve AxisLimits required") from exc
        if len(self.limits) != AXIS_COUNT or any(type(row) is not AxisLimits for row in self.limits):
            raise MotionFault("Twelve AxisLimits required")
        self.startup_duration_s = _positive(startup_duration_s, "startup_duration_s")
        self.startup_damping_duration_s = (self.startup_duration_s if startup_damping_duration_s is None
            else _positive(startup_damping_duration_s, "startup_damping_duration_s"))
        if self.startup_damping_duration_s > self.startup_duration_s:
            raise MotionFault("Damping ramp must not be slower than position gain ramp")
        self.stop_duration_s = _positive(stop_duration_s, "stop_duration_s")
        self.max_sample_age_s = _positive(max_sample_age_s, "max_sample_age_s")
        self.max_sample_gap_s = _positive(max_sample_gap_s, "max_sample_gap_s")
        now = _number(now_s, "now_s")
        self._validate_sample(initial_sample, now)
        self._initial_q = initial_sample.q_model_rad
        self._q = self._initial_q
        self._velocity = (0.,) * AXIS_COUNT
        self._bounds = tuple((max(row.lower_rad, q - row.max_displacement_from_start_rad),
                              min(row.upper_rad, q + row.max_displacement_from_start_rad))
                             for row, q in zip(self.limits, self._initial_q))
        self._started_at = self._last_now = now
        self._last_sample_at = initial_sample.monotonic_s
        self._gain = 0.
        self._damping_gain = 0.
        self._phase = "starting"

    @property
    def fault_reason(self):
        return self._fault_reason

    @property
    def phase(self):
        return "faulted" if self._fault_reason is not None else self._phase

    @property
    def maximum_graceful_stop_s(self):
        return max(row.max_command_velocity_rad_s / row.max_command_acceleration_rad_s2
                   for row in self.limits) + self.stop_duration_s

    def emergency_fault(self, reason):
        if self._fault_reason is None:
            self._fault_reason = str(reason) if reason else "Unspecified emergency"
        raise MotionFault(self._fault_reason)

    def request_stop(self):
        if self._fault_reason is not None:
            raise MotionFault(self._fault_reason)
        self._stop_requested = True

    def _validate_sample(self, sample, now):
        if type(sample) is not MotionSample:
            raise MotionFault("MotionSample required")
        if now < 0 or sample.monotonic_s > now:
            raise MotionFault("Sample/command time is negative or in the future")
        if now - sample.monotonic_s > self.max_sample_age_s:
            raise MotionFault("Stale sample")
        for mid, (row, q, v, torque, temperature) in enumerate(zip(
                self.limits, sample.q_model_rad, sample.velocity_rad_s,
                sample.torque_nm, sample.temperature_c), 1):
            if not row.lower_rad <= q <= row.upper_rad:
                raise MotionFault(f"ID{mid}: measured joint range exceeded")
            if abs(v) > row.max_measured_velocity_rad_s:
                raise MotionFault(f"ID{mid}: measured velocity exceeded")
            if abs(torque) > row.max_measured_torque_nm:
                raise MotionFault(f"ID{mid}: measured torque exceeded")
            if temperature > row.max_temperature_c:
                raise MotionFault(f"ID{mid}: temperature exceeded")

    def _stop_command(self, now):
        if self._stop_anchor is None:
            self._stop_anchor = (self._last_now, self._q, self._velocity, self._gain, self._damping_gain)
        started, start_q, start_v, start_gain, start_damping_gain = self._stop_anchor
        elapsed = now - started
        brake_duration = max(abs(v) / row.max_command_acceleration_rad_s2
                             for v, row in zip(start_v, self.limits))
        q, velocity = [], []
        for pos, v, row in zip(start_q, start_v, self.limits):
            acceleration = row.max_command_acceleration_rad_s2
            duration = abs(v) / acceleration
            t = min(elapsed, duration)
            signed_a = -math.copysign(acceleration, v)
            q.append(pos + v * t + .5 * signed_a * t ** 2)
            velocity.append(0. if elapsed >= duration else v + signed_a * t)
        ramp_elapsed = elapsed - brake_duration
        gain = start_gain * (1. - _quintic_fraction(ramp_elapsed / self.stop_duration_s))
        damping_gain = start_damping_gain * (1. - _quintic_fraction(ramp_elapsed / self.stop_duration_s))
        completed = ramp_elapsed >= self.stop_duration_s
        return (tuple(q), tuple(velocity), gain, damping_gain, "stopped" if completed else "stopping",
                "complete" if completed else "braking" if ramp_elapsed < 0 else "gain_ramp")

    def step(self, target_model_rad, sample: MotionSample, *, now_s):
        """Return one bounded command, or latch a MotionFault without a command.

        All checks are atomic across twelve axes.  Invalid targets are rejected,
        never clipped.  After request_stop(), target_model_rad may be None; policy
        input is ignored because the stopping path is frozen from the last command.
        """
        if self._fault_reason is not None:
            raise MotionFault(self._fault_reason)
        try:
            now = _number(now_s, "now_s")
            self._validate_sample(sample, now)
            for mid, (measured, start, row) in enumerate(zip(
                    sample.q_model_rad, self._initial_q, self.limits), 1):
                if abs(measured - start) > row.max_displacement_from_start_rad:
                    raise MotionFault(f"ID{mid}: measured displacement from start exceeded")
            dt = now - self._last_now
            sample_dt = sample.monotonic_s - self._last_sample_at
            if dt <= 0 or sample_dt <= 0:
                raise MotionFault("Non-increasing command/sample time")
            # Command computation and input acquisition have distinct timestamps
            # on the same monotonic clock. Either reviewed gap can
            # fault while the other remains valid; retain both measurements
            # in the latched error instead of losing the distinction.
            if dt > self.max_sample_gap_s:
                raise MotionFault(f"Command gap exceeded: command_interval_ms={dt*1000:.6f}, "
                                  f"sample_interval_ms={sample_dt*1000:.6f}, "
                                  f"limit_ms={self.max_sample_gap_s*1000:.6f}")
            if sample_dt > self.max_sample_gap_s:
                raise MotionFault(f"Sample gap exceeded: command_interval_ms={dt*1000:.6f}, "
                                  f"sample_interval_ms={sample_dt*1000:.6f}, "
                                  f"limit_ms={self.max_sample_gap_s*1000:.6f}")
            if self._stop_requested:
                q, velocity, gain, damping_gain, phase, stop_stage = self._stop_command(now)
            else:
                target = _vector(target_model_rad, "target_model_rad")
                for mid, (wanted, (low, high)) in enumerate(zip(target, self._bounds), 1):
                    if not low <= wanted <= high:
                        raise MotionFault(f"ID{mid}: target outside joint/supported displacement envelope")
                advanced = [_advance_reference(pos, v, wanted, dt,
                            row.max_command_velocity_rad_s, row.max_command_acceleration_rad_s2)
                            for pos, v, wanted, row in zip(self._q, self._velocity, target, self.limits)]
                q, velocity = tuple(item[0] for item in advanced), tuple(item[1] for item in advanced)
                gain = _quintic_fraction((now - self._started_at) / self.startup_duration_s)
                damping_gain = _quintic_fraction((now - self._started_at) / self.startup_damping_duration_s)
                phase = "active" if gain >= 1. else "starting"
                stop_stage = None
            errors, torques, kp, kd, normalized_q = [], [], [], [], []
            for mid, (row, pos, v, previous_v, measured, measured_v, (low, high)) in enumerate(zip(
                    self.limits, q, velocity, self._velocity, sample.q_model_rad,
                    sample.velocity_rad_s, self._bounds), 1):
                # Tolerance is only for floating-point integration, not external targets.
                if not low - 1e-12 <= pos <= high + 1e-12:
                    raise MotionFault(f"ID{mid}: internal reference escaped supported range")
                # Snap only integration roundoff at an exact permitted endpoint.
                # External policy targets and measured angles are never clipped.
                pos = min(high, max(low, pos))
                if abs(v) > row.max_command_velocity_rad_s + 1e-12:
                    raise MotionFault(f"ID{mid}: internal reference velocity exceeded")
                if abs(v - previous_v) > row.max_command_acceleration_rad_s2 * dt + 1e-12:
                    raise MotionFault(f"ID{mid}: internal reference acceleration exceeded")
                error = pos - measured
                if abs(error) > row.max_tracking_error_rad:
                    raise MotionFault(f"ID{mid}: tracking error exceeded")
                p_gain, d_gain = gain * row.kp, damping_gain * row.kd
                torque = p_gain * error - d_gain * measured_v
                if abs(torque) > row.max_estimated_pd_torque_nm:
                    raise MotionFault(f"ID{mid}: estimated PD torque budget exceeded")
                errors.append(error); torques.append(torque); kp.append(p_gain); kd.append(d_gain)
                normalized_q.append(pos)
            q = tuple(normalized_q)
            command = MotionCommand(q, tuple(kp), tuple(kd), (0.,) * AXIS_COUNT,
                (0.,) * AXIS_COUNT, velocity, tuple(errors), tuple(torques), gain,
                phase, stop_stage, now)
            self._q, self._velocity, self._gain, self._phase = q, velocity, gain, phase
            self._damping_gain = damping_gain
            self._last_now, self._last_sample_at = now, sample.monotonic_s
            return command
        except (ValueError, TypeError, OverflowError) as exc:
            self.emergency_fault(str(exc))
