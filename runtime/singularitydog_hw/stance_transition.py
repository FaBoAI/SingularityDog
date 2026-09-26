"""Pure model-angle transition planning for saved-input diagnostics only.

The quintic path starts at the supplied current-angle candidate, rather than
jumping directly to the policy's first target. Caller-supplied velocity and
acceleration limits are mathematical planning inputs, not verified motor limits.
There is no transport, raw-angle conversion, policy handoff, or live runner here.
This module does not change or subdivide the existing five-degree trials.
"""
import math

from .policy_shadow import CAN_ORDER, LOWER, UPPER

DT_NS = 20_000_000
DT_S = DT_NS / 1_000_000_000
MAX_SAMPLES = 10_001
QUINTIC_PEAK_SPEED = 15. / 8.
QUINTIC_PEAK_ACCELERATION = 10. * math.sqrt(3.) / 3.


def _real(value, label):
    if type(value) not in (int, float):
        raise ValueError(label + ' must be a finite real number, not a boolean')
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(label + ' must be finite') from None
    if not math.isfinite(result):
        raise ValueError(label + ' must be finite')
    return result


def _angles(values, label):
    if type(values) not in (list, tuple) or len(values) != 12:
        raise ValueError(label + ' must contain exactly twelve model angles')
    result = tuple(_real(value, label) for value in values)
    for index, (value, low, high) in enumerate(zip(result, LOWER, UPPER)):
        if not low <= value <= high:
            raise ValueError(label + f' joint {index} outside policy model range')
    return result


def _smoothstep(u):
    # Evaluate symmetrically to avoid cancellation near the final endpoint.
    x = u if u <= .5 else 1. - u
    small = x*x*x*(10. + x*(-15. + 6.*x))
    return small if u <= .5 else 1. - small


def build_plan(current_model_rad, target_model_rad, *, max_velocity_rad_s,
               max_acceleration_rad_s2, max_samples=MAX_SAMPLES):
    """Return a finite, JSON-compatible 20 ms diagnostic trajectory.

    Both limits are explicit positive scalars applied independently to each of
    the twelve joints. Duration is rounded upward to whole 20 ms intervals from
    the analytic quintic derivative maxima. A stationary path still has two
    endpoint samples over one interval. Endpoint derivatives are assumed zero;
    this is not evidence that measured joint velocities are zero.
    """
    current = _angles(current_model_rad, 'current_model_rad')
    target = _angles(target_model_rad, 'target_model_rad')
    velocity = _real(max_velocity_rad_s, 'max_velocity_rad_s')
    acceleration = _real(max_acceleration_rad_s2, 'max_acceleration_rad_s2')
    if velocity <= 0. or acceleration <= 0.:
        raise ValueError('Explicit velocity and acceleration limits must be positive')
    if type(max_samples) is not int or not 2 <= max_samples <= MAX_SAMPLES:
        raise ValueError(f'max_samples must be an integer from 2 to {MAX_SAMPLES}')
    delta = tuple(end-start for start, end in zip(current, target))
    largest_delta = max(abs(value) for value in delta)
    speed_duration = QUINTIC_PEAK_SPEED * largest_delta / velocity
    acceleration_duration = math.sqrt(QUINTIC_PEAK_ACCELERATION * largest_delta / acceleration)
    required_duration = max(speed_duration, acceleration_duration)
    maximum_duration = (max_samples-1)*DT_S
    if not math.isfinite(required_duration) or required_duration > maximum_duration:
        raise ValueError('Required duration exceeds bounded sample count')
    intervals = max(1, math.ceil(required_duration / DT_S))
    # Account for floating-point rounding in the duration calculation without
    # relaxing either caller limit. This can add only a sampling interval.
    while True:
        duration_s = intervals*DT_S
        peak_velocity = [QUINTIC_PEAK_SPEED*abs(value)/duration_s for value in delta]
        peak_acceleration = [QUINTIC_PEAK_ACCELERATION*abs(value)/duration_s**2 for value in delta]
        if max(peak_velocity) <= velocity and max(peak_acceleration) <= acceleration:
            break
        intervals += 1
    if intervals+1 > max_samples:
        raise ValueError('Required duration exceeds bounded sample count')
    samples = []
    for tick in range(intervals+1):
        u = tick/intervals
        blend = _smoothstep(u)
        if tick == 0:
            q = list(current)
        elif tick == intervals:
            q = list(target)
        elif blend <= .5:
            q = [start+d*blend for start, d in zip(current, delta)]
        else:
            q = [end-d*(1.-blend) for end, d in zip(target, delta)]
        speed_fraction = 30.*u*u*(1.-u)**2 / duration_s
        acceleration_fraction = 60.*u*(1.-u)*(1.-2.*u) / duration_s**2
        samples.append({'sample_index': tick, 'elapsed_ns': tick*DT_NS,
                        'q_model_rad': q,
                        'dq_model_rad_s': [d*speed_fraction for d in delta],
                        'ddq_model_rad_s2': [d*acceleration_fraction for d in delta]})
    return {'schema': 'singularitydog.stance-transition-candidate.v1',
            'status': 'OFFLINE_MODEL_ANGLE_TRANSITION_ONLY',
            'output_allowed': False, 'live_runner_available': False,
            'calibration_verified': False, 'physical_limits_verified': False,
            'physical_standing_verified': False, 'policy_handoff_verified': False,
            'raw_angle_conversion_available': False,
            'automatic_segmentation': False, 'existing_trial_limits_changed': False,
            'coordinate_frame': 'candidate calibrated model joint angles in radians; no wrapping',
            'model_can_order_candidate': list(CAN_ORDER),
            'current_model_rad': list(current), 'target_model_rad': list(target),
            'delta_model_rad': list(delta), 'sample_period_ns': DT_NS,
            'duration_ns': intervals*DT_NS, 'duration_s': duration_s,
            'sample_count': intervals+1, 'sample_count_limit': max_samples,
            'max_velocity_rad_s': velocity, 'max_acceleration_rad_s2': acceleration,
            'limit_scope': 'explicit diagnostic mathematical bounds; not validated physical limits',
            'endpoint_velocity_rad_s': 0., 'endpoint_acceleration_rad_s2': 0.,
            'boundary_derivatives_scope': 'assumed at both endpoints; not measured velocity or acceleration',
            'analytic_peak_velocity_rad_s': peak_velocity,
            'analytic_peak_acceleration_rad_s2': peak_acceleration,
            'max_analytic_velocity_rad_s': max(peak_velocity),
            'max_analytic_acceleration_rad_s2': max(peak_acceleration),
            'samples': samples}
