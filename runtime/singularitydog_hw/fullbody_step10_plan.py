"""Offline-only synchronized twelve-axis diagnostic plan and feedback gate.

This module neither opens CAN ports nor authorizes motor output. A live runner
must separately prove the current boot's twelve-axis current-position hold,
physical clearance, actuator identities, and a confirmed twelve-axis STOP path.
Raw motor-coordinate directions are explicit; ``+1`` is not a joint sign.
"""

from dataclasses import dataclass
import math

from .rs05_trial_protocol import POSITION_MAX, POSITION_MIN


IDS = tuple(range(1, 13))
PERIOD_S = 0.05
RAMP_S = 8.0
HOLD_S = 1.0
MAX_STEP_DEG = 10.0
MAX_TRACKING_ERROR_DEG = 2.0
MAX_TORQUE_FEEDBACK_NM = 0.8
MAX_ANGLE_EXCURSION_DEG = 10.5
MAX_FEEDBACK_AGE_S = 0.1
MAX_ABS_VELOCITY_RAD_S = 0.6


def _real(value, label):
    if type(value) not in (int, float):
        raise ValueError(f'{label} must be a finite number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _twelve(values, label):
    if type(values) is not dict or set(values) != set(IDS) or any(type(i) is not int for i in values):
        raise ValueError(f'{label} must map exactly integer IDs 1..12')
    return values


def quintic_fraction(t):
    """Zero velocity and acceleration at both endpoints, with 0<=t<=1."""
    t = _real(t, 'normalized time')
    if not 0.0 <= t <= 1.0:
        raise ValueError('normalized time outside 0..1')
    return t*t*t*(10.0 + t*(-15.0 + 6.0*t))


@dataclass(frozen=True)
class StepPlan:
    start_rad_by_id: dict
    delta_rad_by_id: dict
    samples: tuple
    ramp_s: float
    hold_s: float
    period_s: float


def build_plan(start_rad_by_id, raw_direction_by_id, *, amplitude_deg=MAX_STEP_DEG):
    """Build one finite, common-clock raw-coordinate ramp then a one-second hold.

    An individual direction is required for every motor. The caller must prove
    clearance across the whole trajectory; the protocol range alone is not a
    mechanical limit. Up to ten degrees is accepted, without automatic retry.
    """
    starts = {i: _real(v, f'ID{i} start') for i, v in _twelve(start_rad_by_id, 'starts').items()}
    directions = _twelve(raw_direction_by_id, 'directions')
    if any(type(directions[i]) is not int or directions[i] not in (-1, 1) for i in IDS):
        raise ValueError('Each raw direction must be explicit integer -1 or +1')
    amplitude_deg = _real(amplitude_deg, 'amplitude_deg')
    if not 0.0 < amplitude_deg <= MAX_STEP_DEG:
        raise ValueError('amplitude must be greater than zero and at most ten degrees')
    magnitude = math.radians(amplitude_deg)
    delta = {i: directions[i] * magnitude for i in IDS}
    for i in IDS:
        if not (POSITION_MIN <= starts[i] <= POSITION_MAX
                and POSITION_MIN <= starts[i] + delta[i] <= POSITION_MAX):
            raise ValueError(f'ID{i} start/end outside unwrapped RS05 protocol range')
    ramp_ticks = round(RAMP_S / PERIOD_S)
    hold_ticks = round(HOLD_S / PERIOD_S)
    samples = []
    for tick in range(ramp_ticks + hold_ticks):
        elapsed_s = tick * PERIOD_S
        fraction = quintic_fraction(min(tick / ramp_ticks, 1.0))
        targets = {i: starts[i] + delta[i] * fraction for i in IDS}
        samples.append({'tick': tick, 'elapsed_s': elapsed_s,
                        'fraction': fraction, 'target_rad_by_id': targets})
    if len(samples) != 180 or samples[0]['fraction'] != 0.0 or samples[-1]['fraction'] != 1.0:
        raise AssertionError('Fixed 8-second ramp plus 1-second endpoint hold changed')
    return StepPlan(starts, delta, tuple(samples), RAMP_S, HOLD_S, PERIOD_S)


def check_feedback(plan, tick, feedback_by_id, *, now_s):
    """Fail closed on stale, missing, faulted, high-load or poorly tracked axes.

    ``feedback_by_id`` maps ID to ``(Type2Feedback, received_monotonic_s)``.
    The diagnostic torque threshold is a software STOP monitor, not a motor
    torque limit. A live runner must apply this to every fresh reply and abort
    both buses at the first violation, including read/write/timeout failures.
    """
    if type(tick) is not int or not 0 <= tick < len(plan.samples):
        raise ValueError('Invalid plan tick')
    _twelve(feedback_by_id, 'feedback')
    now_s = _real(now_s, 'now_s')
    target = plan.samples[tick]['target_rad_by_id']
    observations = {}
    for i in IDS:
        value, received_s = feedback_by_id[i]
        age = now_s - _real(received_s, f'ID{i} received time')
        if not 0.0 <= age <= MAX_FEEDBACK_AGE_S:
            raise RuntimeError(f'ID{i} stale or future Type2 feedback')
        if value.mode_state != 2 or value.fault_bits != 0:
            raise RuntimeError(f'ID{i} wrong mode or fault')
        position = _real(value.protocol_position_rad, f'ID{i} position')
        velocity = _real(value.velocity_rad_s, f'ID{i} velocity')
        torque = _real(value.torque_nm, f'ID{i} torque')
        temperature = _real(value.temperature_c, f'ID{i} temperature')
        if not POSITION_MIN <= position <= POSITION_MAX or not -20 <= temperature <= 80:
            raise RuntimeError(f'ID{i} position or temperature outside diagnostic envelope')
        if abs(velocity) > MAX_ABS_VELOCITY_RAD_S:
            raise RuntimeError(f'ID{i} excessive velocity')
        if abs(torque) > MAX_TORQUE_FEEDBACK_NM:
            raise RuntimeError(f'ID{i} torque monitor tripped')
        excursion = position - plan.start_rad_by_id[i]
        if abs(excursion) > math.radians(MAX_ANGLE_EXCURSION_DEG):
            raise RuntimeError(f'ID{i} excursion exceeds ten-degree plan allowance')
        error = position - target[i]
        if abs(error) > math.radians(MAX_TRACKING_ERROR_DEG):
            raise RuntimeError(f'ID{i} target tracking error exceeds two degrees')
        observations[i] = {'target_rad': target[i], 'position_rad': position,
                           'tracking_error_rad': error, 'excursion_rad': excursion,
                           'velocity_rad_s': velocity, 'torque_nm': torque,
                           'temperature_c': temperature, 'age_s': age}
    return observations


def require_current_hold_gate(*, hold_status, hold_boot_id, current_boot_id,
                              hold_stop_confirmed, id4_mechanical_check_passed):
    """Reject a ten-degree trial until the specific r13 failure is resolved."""
    if (hold_status != 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            or type(hold_boot_id) is not str or not hold_boot_id
            or hold_boot_id != current_boot_id
            or hold_stop_confirmed is not True
            or id4_mechanical_check_passed is not True):
        raise RuntimeError('ID4 remediation and a passing same-boot twelve-axis hold are required')
    return True
