"""Screen a pinned policy against saved, nonsimultaneous robot observations.

No CAN, serial, I2C, output runner or actuator API is imported.  The historical
profile supplies calibration and model provenance, never trial authorization.
Keeping the recorded joints and IMU fixed across virtual ticks is a diagnostic
hypothesis, not a prediction of closed-loop motion or support force.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

from singularitydog_hw import policy_shadow
from singularitydog_hw.policy_live_profile import load_profile
from singularitydog_hw.policy_output_model import LivePolicyModel
from singularitydog_hw.policy_observer import _TARGET_LOWER, _TARGET_UPPER


def _read(path):
    raw = Path(path).read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def _model_pose(profile, capture):
    if (capture.get('status') != 'RECORDED_REVIEW_REQUIRED' or capture.get('errors') or
            capture.get('boot_id') != profile['boot_id']):
        raise ValueError('Same-boot quiet twelve-axis capture required')
    rows = capture['telemetry']['rows']
    identities = capture['identities']
    if set(rows) != set(profile['axes']) or set(identities) != set(rows):
        raise ValueError('Expected twelve reviewed motor identities')
    pose = [0.] * 12
    turns = {}
    for key, axis in profile['axes'].items():
        row = rows[key]
        if (identities[key]['mcu_uid_hex'] != axis['uid'] or
                row['run_mode'] != 0 or row['current'] != 0 or
                row['position_span_deg'] > .1):
            raise ValueError(f'ID{key} identity or static angle invalid')
        raw = row['median_position_rad']
        index = policy_shadow.CAN_ORDER.index(int(key))
        lower, upper = policy_shadow.LOWER[index], policy_shadow.UPPER[index]
        candidates = [(turn, axis['sign'] * (raw - turn * 2 * math.pi) + axis['offset_rad'])
                      for turn in range(-3, 4)]
        candidates = [(turn, value) for turn, value in candidates
                      if math.isfinite(value) and lower <= value <= upper]
        if len(candidates) != 1:
            raise ValueError(f'ID{key} calibrated branch not unique')
        turns[key], pose[int(key)-1] = candidates[0]
    return pose, turns


def _static_imu(profile, imu_capture, model):
    if (imu_capture.get('status') != 'RECORDED_NOT_CALIBRATED' or
            imu_capture.get('errors') or
            imu_capture['plan']['can_opened'] is not False or
            imu_capture['summary']['samples'] < 100):
        raise ValueError('Static sensor-frame IMU record required')
    summary = imu_capture['summary']
    accel = summary['accel_mean_m_s2']
    gyro_sensor = summary['gyro_mean_rad_s']
    if any(not math.isfinite(value) for value in (*accel, *gyro_sensor)):
        raise ValueError('Nonfinite saved IMU')
    norm = math.hypot(*accel)
    if not profile['imu_accel_norm_min_m_s2'] <= norm <= profile['imu_accel_norm_max_m_s2']:
        raise ValueError('Saved IMU gravity norm outside reviewed range')
    body = [sum(row[j]*accel[j] for j in range(3)) for row in model.rotation]
    gyro = [sum(row[j]*(gyro_sensor[j]-model.bias[j]) for j in range(3))
            for row in model.rotation]
    gravity = [-value/norm for value in body]
    tilt = math.acos(max(-1., min(1., -gravity[2])))
    if tilt > profile['imu_tilt_limit_rad'] or math.hypot(*gyro) > profile['imu_gyro_limit_rad_s']:
        raise ValueError('Saved static IMU body tilt/rate outside profile limits')
    return gyro, gravity, {'accel_norm_m_s2': norm, 'body_tilt_deg': math.degrees(tilt),
                           'corrected_body_gyro_rad_s': gyro}


def screen(profile_path, capture_path, imu_path, ticks):
    if not 1 <= ticks <= 500:
        raise ValueError('Virtual ticks must be in 1..500')
    profile = load_profile(profile_path, require_approved=True)
    capture, capture_sha = _read(capture_path)
    imu_capture, imu_sha = _read(imu_path)
    pose, turns = _model_pose(profile, capture)
    model = LivePolicyModel(profile)
    gyro, gravity, imu_assessment = _static_imu(profile, imu_capture, model)
    q = [pose[i-1] for i in policy_shadow.CAN_ORDER]
    values = (gyro, gravity, profile['command'], q, [0.]*12,
              [float(profile['h_hypothesis'])]*12)
    selected = {}
    max_abs_deg = [0.]*12
    sign_sets = [set() for _ in range(12)]
    previous = None
    max_step_deg = 0.
    for tick in range(ticks):
        with model.torch.inference_mode():
            for buffer, row in zip(model.input_buffers, values):
                for j, value in enumerate(row):
                    buffer[j] = value
            output = model.policy(*model.tensors)
            if (not isinstance(output, model.torch.Tensor) or
                    output.device.type != 'cpu' or output.dtype != model.torch.float32 or
                    tuple(output.shape) != (1, 12)):
                raise ValueError('Pinned policy returned wrong target ABI')
            raw = output[0].tolist()
        if not all(math.isfinite(value) and lower <= value <= upper
                   for value, lower, upper in zip(raw, _TARGET_LOWER, _TARGET_UPPER)):
            raise ValueError(f'Policy target outside model range on tick {tick}')
        target = [0.]*12
        for motor_id, value in zip(policy_shadow.CAN_ORDER, raw):
            target[motor_id-1] = value
        delta = [math.degrees(target[j]-pose[j]) for j in range(12)]
        for j, value in enumerate(delta):
            max_abs_deg[j] = max(max_abs_deg[j], abs(value))
            sign_sets[j].add('positive' if value > .01 else 'negative' if value < -.01 else 'near_zero')
        if previous is not None:
            max_step_deg = max(max_step_deg, *(abs(math.degrees(target[j]-previous[j]))
                                                for j in range(12)))
        previous = target
        if tick in {0, 1, 4, 9, 49, 99, 249, ticks-1}:
            selected[str(tick)] = {'target_model_rad_by_id': target,
                                   'delta_deg_by_id': delta}
    return {
        'schema': 'singularitydog.saved-policy-direction-screen.v1',
        'status': 'FILE_ONLY_HYPOTHESIS_NOT_OUTPUT_APPROVAL',
        'profile_sha256': profile['profile_sha256'],
        'capture_sha256': capture_sha, 'imu_sha256': imu_sha,
        'boot_id': capture['boot_id'], 'historical_profile_power_epoch': profile['motor_power_epoch'],
        'capture_power_epoch_not_proven_equal': True,
        'saved_q_and_imu_not_simultaneous': True,
        'fixed_saved_input_not_closed_loop_motion': True,
        'model_backend': model.execution['model_backend'],
        'model_provenance': model.provenance,
        'command': profile['command'], 'h_hypothesis': profile['h_hypothesis'],
        'virtual_ticks': ticks, 'static_imu_assessment': imu_assessment,
        'start_model_rad_by_id': pose, 'calibrated_encoder_turn_by_id': turns,
        'selected_ticks': selected,
        'max_absolute_target_delta_deg_by_id': max_abs_deg,
        'target_delta_signs_by_id': [sorted(signs) for signs in sign_sets],
        'max_policy_target_step_deg_per_virtual_tick': max_step_deg,
        'motor_output_allowed': False, 'box_removal_allowed': False,
        'load_bearing_verified': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--imu', type=Path, required=True)
    parser.add_argument('--ticks', type=int, default=100)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output must be a fresh path')
    result = screen(args.profile, args.capture, args.imu, args.ticks)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in (
        'status', 'virtual_ticks', 'max_absolute_target_delta_deg_by_id',
        'max_policy_target_step_deg_per_virtual_tick', 'motor_output_allowed')},
        ensure_ascii=False))


if __name__ == '__main__':
    main()
