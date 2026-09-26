"""Pure 50 ms raw/model stance candidates, never commands or a live runner.

The caller supplies all twelve calibration signs/offsets and mathematical speed
and acceleration limits. Neither those inputs nor successful interpolation
verify identities, calibration, physical limits, collisions, support, or stance.
Out-of-model-range starts require an explicit unsupported-recovery opt-in; this
does not change the existing model planner or any runtime guard.
"""
import math

from .policy_shadow import CAN_ORDER, LOWER, UPPER
from .rs05_trial_protocol import POSITION_MIN, POSITION_MAX
from .stance_transition import QUINTIC_PEAK_SPEED, QUINTIC_PEAK_ACCELERATION

MOTOR_IDS = tuple(range(1, 13))
DT_NS = 50_000_000
DT_S = DT_NS / 1_000_000_000
MAX_SAMPLES = 10_001
MODEL_RANGES = dict(zip(CAN_ORDER, zip(LOWER, UPPER)))


def _real(value, label):
    if type(value) not in (int, float):
        raise ValueError(label + ' must be a finite real number, not a boolean')
    try:
        value = float(value)
    except (ValueError, OverflowError):
        raise ValueError(label + ' must be finite') from None
    if not math.isfinite(value):
        raise ValueError(label + ' must be finite')
    return value


def _map(values, label):
    if (type(values) is not dict or any(type(i) is not int for i in values)
            or set(values) != set(MOTOR_IDS)):
        raise ValueError(label + ' must map exactly integer motor IDs 1..12')
    return {i: _real(values[i], f'{label}[{i}]') for i in MOTOR_IDS}


def _calibration(sign_by_id, offset_rad_by_id):
    _map(sign_by_id, 'sign_by_id')
    if any(type(sign_by_id[i]) is not int or sign_by_id[i] not in (-1, 1) for i in MOTOR_IDS):
        raise ValueError('Every calibration sign must be integer -1 or +1')
    return dict(sign_by_id), _map(offset_rad_by_id, 'offset_rad_by_id')


def _raw_range(values):
    for i, value in values.items():
        if not POSITION_MIN <= value <= POSITION_MAX:
            raise ValueError(f'ID{i} raw angle outside declared protocol range; no wrapping')
    return values


def raw_to_model(raw_rad_by_id, *, sign_by_id, offset_rad_by_id):
    """Apply q_model = sign * raw + offset without enforcing model limits."""
    raw = _raw_range(_map(raw_rad_by_id, 'raw_rad_by_id'))
    signs, offsets = _calibration(sign_by_id, offset_rad_by_id)
    return {i: _real(signs[i]*raw[i] + offsets[i], f'ID{i} converted model angle') for i in MOTOR_IDS}


def model_to_raw(model_rad_by_id, *, sign_by_id, offset_rad_by_id):
    """Invert the explicit affine map, preserving turns without wrapping."""
    model = _map(model_rad_by_id, 'model_rad_by_id')
    signs, offsets = _calibration(sign_by_id, offset_rad_by_id)
    return _raw_range({i: _real(signs[i]*(model[i]-offsets[i]), f'ID{i} converted raw angle')
                       for i in MOTOR_IDS})


def _distance(value, bounds):
    low, high = bounds
    return max(low-value, 0., value-high)


def _blend(u):
    # Same symmetric quintic as stance_transition, avoiding endpoint cancellation.
    x = u if u <= .5 else 1.-u
    small = x*x*x*(10. + x*(-15. + 6.*x))
    return small if u <= .5 else 1.-small


def _interpolate(start, end, delta, blend):
    if blend <= .5:
        return {i: start[i] + delta[i]*blend for i in MOTOR_IDS}
    return {i: end[i] - delta[i]*(1.-blend) for i in MOTOR_IDS}


def build_plan(current_raw_rad_by_id, target_model_rad_by_id, *, sign_by_id,
               offset_rad_by_id, max_velocity_rad_s, max_acceleration_rad_s2,
               allow_unsupported_recovery=False, max_samples=MAX_SAMPLES):
    """Build one finite offline quintic path with fixed 50 ms sample spacing.

    Limits are required positive mathematical inputs, with no physical defaults.
    For example, a caller may explicitly choose radians(10) and radians(20).
    Duration is the analytic minimum rounded upward to whole intervals; it is
    fixed before sampling. There is no automatic retry, segmentation, or handoff.
    All targets must lie in existing policy model ranges and the declared raw
    protocol range. An outside start is rejected unless explicitly requested as
    an unsupported recovery candidate, whose distance to each model range must
    never increase. Every result, including an in-range one, forbids output.
    """
    if type(allow_unsupported_recovery) is not bool:
        raise ValueError('allow_unsupported_recovery must be an explicit boolean')
    if type(max_samples) is not int or not 2 <= max_samples <= MAX_SAMPLES:
        raise ValueError(f'max_samples must be an integer from 2 to {MAX_SAMPLES}')
    current_raw = _raw_range(_map(current_raw_rad_by_id, 'current_raw_rad_by_id'))
    target_model = _map(target_model_rad_by_id, 'target_model_rad_by_id')
    signs, offsets = _calibration(sign_by_id, offset_rad_by_id)
    for i, value in target_model.items():
        if not MODEL_RANGES[i][0] <= value <= MODEL_RANGES[i][1]:
            raise ValueError(f'ID{i} target outside policy model range')
    current_model = raw_to_model(current_raw, sign_by_id=signs, offset_rad_by_id=offsets)
    target_raw = model_to_raw(target_model, sign_by_id=signs, offset_rad_by_id=offsets)
    initial_distance = {i: _distance(current_model[i], MODEL_RANGES[i]) for i in MOTOR_IDS}
    outside_ids = [i for i in MOTOR_IDS if initial_distance[i] > 0.]
    if outside_ids and not allow_unsupported_recovery:
        raise ValueError('Start outside policy model range requires explicit unsupported recovery candidate')
    velocity = _real(max_velocity_rad_s, 'max_velocity_rad_s')
    acceleration = _real(max_acceleration_rad_s2, 'max_acceleration_rad_s2')
    if velocity <= 0. or acceleration <= 0.:
        raise ValueError('Explicit velocity and acceleration limits must be positive')
    raw_delta = {i: target_raw[i]-current_raw[i] for i in MOTOR_IDS}
    model_delta = {i: target_model[i]-current_model[i] for i in MOTOR_IDS}
    # Both coordinate calculations are checked to cover floating-point roundoff.
    largest_delta = max(abs(d) for delta in (raw_delta, model_delta) for d in delta.values())
    required_s = max(QUINTIC_PEAK_SPEED*largest_delta/velocity,
                     math.sqrt(QUINTIC_PEAK_ACCELERATION*largest_delta/acceleration))
    if not math.isfinite(required_s) or required_s > (max_samples-1)*DT_S:
        raise ValueError('Required duration exceeds bounded sample count')
    intervals = max(1, math.ceil(required_s/DT_S))
    while True:
        duration_s = intervals*DT_S
        peaks = {frame: {'velocity_rad_s': {i: QUINTIC_PEAK_SPEED*abs(delta[i])/duration_s for i in MOTOR_IDS},
                         'acceleration_rad_s2': {i: QUINTIC_PEAK_ACCELERATION*abs(delta[i])/duration_s**2 for i in MOTOR_IDS}}
                 for frame, delta in (('raw', raw_delta), ('model', model_delta))}
        if all(max(p['velocity_rad_s'].values()) <= velocity and
               max(p['acceleration_rad_s2'].values()) <= acceleration for p in peaks.values()):
            break
        intervals += 1
    if intervals+1 > max_samples:
        raise ValueError('Required duration exceeds bounded sample count')
    samples, previous_distance = [], initial_distance
    for tick in range(intervals+1):
        u = tick/intervals
        blend = _blend(u)
        raw = dict(current_raw) if tick == 0 else dict(target_raw) if tick == intervals else _interpolate(current_raw, target_raw, raw_delta, blend)
        model = dict(current_model) if tick == 0 else dict(target_model) if tick == intervals else _interpolate(current_model, target_model, model_delta, blend)
        distance = {i: _distance(model[i], MODEL_RANGES[i]) for i in MOTOR_IDS}
        if any(distance[i] > previous_distance[i] for i in MOTOR_IDS):
            raise ValueError('Candidate increases distance to model range')
        previous_distance = distance
        speed_fraction = 30.*u*u*(1.-u)**2/duration_s
        acceleration_fraction = 60.*u*(1.-u)*(1.-2.*u)/duration_s**2
        samples.append({'sample_index': tick, 'elapsed_ns': tick*DT_NS,
                        'q_raw_rad_by_id': raw, 'q_model_rad_by_id': model,
                        'dq_raw_rad_s_by_id': {i: raw_delta[i]*speed_fraction for i in MOTOR_IDS},
                        'dq_model_rad_s_by_id': {i: model_delta[i]*speed_fraction for i in MOTOR_IDS},
                        'ddq_raw_rad_s2_by_id': {i: raw_delta[i]*acceleration_fraction for i in MOTOR_IDS},
                        'ddq_model_rad_s2_by_id': {i: model_delta[i]*acceleration_fraction for i in MOTOR_IDS},
                        'distance_to_model_range_rad_by_id': distance})
    return {'schema': 'singularitydog.raw-stance-transition-candidate.v1',
            'status': 'UNSUPPORTED_RECOVERY_CANDIDATE_ONLY' if outside_ids else 'OFFLINE_RAW_STANCE_CANDIDATE_ONLY',
            'output_allowed': False, 'live_runner_available': False, 'policy_handoff_available': False,
            'calibration_verified': False, 'identity_verified': False, 'physical_limits_verified': False,
            'physical_standing_verified': False, 'collision_clearance_verified': False,
            'automatic_retries': False, 'automatic_segmentation': False, 'existing_runtime_guards_changed': False,
            'angle_wrapping_applied': False, 'motor_ids': list(MOTOR_IDS), 'model_can_order_candidate': list(CAN_ORDER),
            'calibration_formula': 'q_model = sign * raw + offset; rad; no wrapping',
            'sign_by_id': signs, 'offset_rad_by_id': offsets,
            'current_raw_rad_by_id': current_raw, 'current_model_rad_by_id': current_model,
            'target_raw_rad_by_id': target_raw, 'target_model_rad_by_id': target_model,
            'delta_raw_rad_by_id': raw_delta, 'delta_model_rad_by_id': model_delta,
            'model_ranges_rad_by_id': {i: list(MODEL_RANGES[i]) for i in MOTOR_IDS},
            'raw_protocol_range_rad': [POSITION_MIN, POSITION_MAX],
            'allow_unsupported_recovery': allow_unsupported_recovery,
            'initial_out_of_model_range_ids': outside_ids,
            'initial_distance_to_model_range_rad_by_id': initial_distance,
            'monotonic_distance_to_model_range_required': True,
            'monotonic_distance_to_model_range_satisfied': True,
            'recovery_scope': 'unsupported offline candidate; no existing model/runtime guard bypass',
            'duration_ns': intervals*DT_NS, 'duration_s': duration_s,
            'sample_period_ns': DT_NS, 'sample_count': len(samples), 'sample_count_limit': max_samples,
            'max_velocity_rad_s': velocity, 'max_acceleration_rad_s2': acceleration,
            'limit_scope': 'caller mathematical bounds only; not certified physical limits',
            'analytic_peaks_by_frame': peaks,
            'endpoint_velocity_rad_s': 0., 'endpoint_acceleration_rad_s2': 0.,
            'endpoint_derivative_scope': 'assumed, not measured', 'samples': samples}
