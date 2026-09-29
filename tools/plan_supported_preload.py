"""File-only screen for a small four-leg preload with the torso box retained.

This computes a geometric target. It has no CAN imports or actuator path and
does not authorize a trial, support transfer, box removal, or standing.
"""

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

from singularitydog_hw.policy_live_profile import load_profile
from singularitydog_hw import policy_shadow, policy_live_profile
from tools.screen_box_lift_offline import (URDF_SHA256, parse_d17, foot_centers,
                                           rl_plane_residual_mm)
from tools.offline_box_rise_candidate import _ik_near

IDS = {str(i) for i in range(1, 13)}


def finite_number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


def strict_json(source):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'Duplicate JSON key: {key}')
            result[key] = value
        return result

    def bad_constant(value):
        raise ValueError(f'Nonfinite JSON constant: {value}')

    return json.loads(source, object_pairs_hook=pairs, parse_constant=bad_constant)


def source_fingerprints():
    paths = [Path(__file__), Path(__file__).with_name('screen_box_lift_offline.py'),
             Path(__file__).with_name('offline_box_rise_candidate.py'),
             Path(policy_live_profile.__file__),
             Path(policy_shadow.__file__)]
    return {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def quiet_position(row, key):
    value = finite_number(row['median_position_rad'], f'ID{key} position')
    span = finite_number(row['position_span_deg'], f'ID{key} span')
    current = finite_number(row['current'], f'ID{key} current')
    voltage = finite_number(row['voltage'], f'ID{key} voltage')
    if type(row['run_mode']) is not int or row['run_mode'] != 0 or current != 0:
        raise ValueError(f'ID{key} quiet posture differs')
    if not 0 <= span <= .1 or voltage <= 0:
        raise ValueError(f'ID{key} quiet span or voltage differs')
    samples = row['position_samples']
    if type(samples) is not list or len(samples) != 3:
        raise ValueError(f'ID{key} needs three raw position samples')
    values = []
    prior_reply = 0
    for sample in samples:
        values.append(finite_number(sample['rad'], f'ID{key} raw sample'))
        request, reply = sample['request_monotonic_ns'], sample['reply_monotonic_ns']
        if (type(request) is not int or type(reply) is not int or
                request <= prior_reply or reply < request):
            raise ValueError(f'ID{key} sample chronology differs')
        prior_reply = reply
    computed_span = math.degrees(max(values)-min(values))
    if (not math.isclose(value, statistics.median(values), rel_tol=0., abs_tol=1e-12) or
            not math.isclose(span, computed_span, rel_tol=0., abs_tol=1e-9)):
        raise ValueError(f'ID{key} raw samples disagree with capture summary')
    return value


LEGS = {'FR': (1, 2, 3), 'FL': (4, 5, 6),
        'RR': (7, 8, 9), 'RL': (10, 11, 12)}


def foot_down_target(foot, rise_mm, body_up_vector):
    """Translate along an explicit up direction expressed in body coordinates."""
    rise_mm = finite_number(rise_mm, 'Foot translation')
    if len(foot) != 3:
        raise ValueError('A finite three-component foot position is required')
    foot = tuple(finite_number(v, 'Foot coordinate') for v in foot)
    if rise_mm < 0:
        raise ValueError('Foot translation must be nonnegative')
    if len(body_up_vector) != 3 or any(type(v) not in (int, float) for v in body_up_vector) or not all(math.isfinite(v) for v in body_up_vector):
        raise ValueError('A finite three-component body up vector is required')
    norm = math.sqrt(sum(v * v for v in body_up_vector))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Body up vector must be nonzero')
    up = tuple(v / norm for v in body_up_vector)
    return tuple(v - rise_mm / 1000. * u for v, u in zip(foot, up)), up


def plan(profile_path, capture_path, urdf_path, rise_mm, clearance_deg,
         *, rebase_offline=False, body_up_vector=(0., 0., 1.)):
    rise_mm = finite_number(rise_mm, 'Rise')
    clearance_deg = finite_number(clearance_deg, 'Clearance')
    if type(rebase_offline) is not bool:
        raise ValueError('Offline rebase must be an explicit boolean')
    if not (0 < rise_mm <= 2 and 0 < clearance_deg <= 10):
        raise ValueError('Small finite rise and measured clearance required')
    _, up = foot_down_target((0., 0., 0.), rise_mm, body_up_vector)
    fingerprints = source_fingerprints()
    inputs = {name: (Path(path), Path(path).read_bytes()) for name, path in
              (('profile', profile_path), ('capture', capture_path), ('urdf', urdf_path))}
    profile = load_profile(profile_path, require_approved=True)
    if hashlib.sha256(inputs['profile'][1]).hexdigest() != profile['profile_sha256']:
        raise ValueError('Profile changed while loading')
    if set(profile['axes']) != IDS or set(profile['start_pose_bounds']) != IDS:
        raise ValueError('Exactly twelve reviewed axes required')
    if profile['scope'] not in ('fixed_catch_current_hold_only',
                                'supported_characterization_only'):
        raise ValueError('Expected a reviewed box-supported source profile')
    capture = strict_json(inputs['capture'][1])
    if (capture.get('schema') != 'singularitydog.readonly-12-angle-capture.v1' or
            capture.get('motor_output_allowed') is not False or
            capture.get('approved_for_runtime') is not False or
            capture.get('angle_wrap_applied') is not False or
            capture.get('status') != 'RECORDED_REVIEW_REQUIRED' or capture.get('errors') or
            capture.get('boot_id') != profile['boot_id']):
        raise ValueError('Unmodified quiet twelve-axis capture from the same boot required')
    raw_rows = capture['telemetry']['rows']
    identities = capture['identities']
    if set(raw_rows) != set(profile['axes']) or set(identities) != set(profile['axes']):
        raise ValueError('Exactly the reviewed twelve axes required')
    urdf_bytes = inputs['urdf'][1]
    if hashlib.sha256(urdf_bytes).hexdigest() != URDF_SHA256:
        raise ValueError('D17 URDF hash differs')
    geometry = parse_d17(urdf_bytes)
    q = {}
    raw = {}
    turns = {}
    for key, axis in profile['axes'].items():
        row = raw_rows[key]
        if identities[key]['mcu_uid_hex'] != axis['uid']:
            raise ValueError(f'ID{key} identity differs')
        if type(axis['sign']) not in (int, float) or axis['sign'] not in (-1, 1):
            raise ValueError(f'ID{key} sign invalid')
        for field in ('offset_rad', 'kp', 'max_estimated_pd_torque_nm',
                      'physical_lower_rad', 'physical_upper_rad'):
            finite_number(axis[field], f'ID{key} {field}')
        if axis['kp'] < 0 or axis['max_estimated_pd_torque_nm'] <= 0:
            raise ValueError(f'ID{key} PD parameters invalid')
        value = quiet_position(row, key)
        choices = [(turn, axis['sign'] * (value - turn * 2 * math.pi) + axis['offset_rad'])
                   for turn in range(-3, 4)]
        if rebase_offline:
            index = policy_shadow.CAN_ORDER.index(int(key))
            lower, upper = policy_shadow.LOWER[index], policy_shadow.UPPER[index]
        else:
            lower, upper = profile['start_pose_bounds'][key]
        choices = [(turn, angle) for turn, angle in choices if lower <= angle <= upper]
        if len(choices) != 1:
            raise ValueError(f'ID{key} initial encoder branch differs')
        raw[int(key)] = value
        q[int(key)] = choices[0][1]
        turns[key] = choices[0][0]
    feet = foot_centers(q, geometry)
    target = {}
    max_delta_deg = 0.
    max_stall_pd_nm = 0.
    failures = []
    blockers = []
    if rebase_offline:
        blockers.extend(('New pose has no reviewed start bounds or power-epoch binding',
                         'New pose has no reviewed physical corridor or swept-clearance proof'))
    leg_rows = {}
    for name, ids in LEGS.items():
        start = tuple(q[i] for i in ids)
        foot = feet[name]
        desired, _ = foot_down_target(foot, rise_mm, up)
        final = _ik_near(start, desired, geometry[name], tolerance_m=1e-9)
        if len(final) != 3:
            raise ValueError('Inverse kinematics must return all three leg axes')
        final = tuple(finite_number(v, 'Inverse kinematics target') for v in final)
        deltas = {}
        for i, angle in zip(ids, final):
            axis = profile['axes'][str(i)]
            ddeg = math.degrees(angle - q[i])
            stall_pd = axis['kp'] * abs(angle - q[i])
            target[i] = angle
            deltas[str(i)] = round(ddeg, 4)
            max_delta_deg = max(max_delta_deg, abs(ddeg))
            max_stall_pd_nm = max(max_stall_pd_nm, stall_pd)
            if abs(ddeg) > clearance_deg:
                label = ('assumed' if rebase_offline else 'confirmed')
                failures.append(f'ID{i} exceeds {label} angular clearance')
            if not rebase_offline and not axis['physical_lower_rad'] <= angle <= axis['physical_upper_rad']:
                failures.append(f'ID{i} exceeds reviewed local envelope')
            if stall_pd > axis['max_estimated_pd_torque_nm']:
                failures.append(f'ID{i} would exceed monitored PD torque if initially stationary')
            k = policy_shadow.CAN_ORDER.index(i)
            if not policy_shadow.LOWER[k] <= angle <= policy_shadow.UPPER[k]:
                failures.append(f'ID{i} exceeds learned-model joint range')
        leg_rows[name] = {'nominal_joint_delta_deg_by_id': deltas,
                          'nominal_foot_down_mm': rise_mm,
                          'nominal_foot_target_body_m': desired}
    foot_z = [feet[name][2] for name in LEGS]
    plane_residual_mm = finite_number(rl_plane_residual_mm(feet), 'RL plane residual')
    if plane_residual_mm > 2.:
        blockers.append('Four nominal feet differ from a common plane by more than 2 mm')
    if fingerprints != source_fingerprints() or any(path.read_bytes() != raw for path, raw in inputs.values()):
        raise ValueError('Source input or implementation changed during planning')
    return {
        'schema': 'singularitydog.supported-preload-file-screen.v1',
        'status': 'OFFLINE_GEOMETRY_SCREEN_ONLY',
        'source_fingerprints': fingerprints,
        'source_inputs': {name: {'path': str(path.resolve()), 'sha256': hashlib.sha256(raw).hexdigest()}
                          for name, (path, raw) in inputs.items()},
        'capture_boot_id': capture['boot_id'],
        'capture_power_epoch': capture.get('motor_power_epoch'),
        'source_screen_passed': not failures and not blockers,
        'approved_for_runtime': False,
        'profile_sha256': profile['profile_sha256'],
        'profile_power_epoch': profile['motor_power_epoch'],
        'capture_sha256': hashlib.sha256(inputs['capture'][1]).hexdigest(),
        'capture_started_at': capture['started_at'],
        'urdf_sha256': URDF_SHA256,
        'rise_mm': rise_mm,
        'up_direction_body_unit_vector': up,
        'up_direction_independently_verified': False,
        'assumed_clearance_deg': clearance_deg,
        'historical_profile_used_for_calibration_only': rebase_offline,
        'leg_targets': leg_rows,
        'initial_model_rad_by_id': {str(i): q[i] for i in sorted(q)},
        'selected_branch_turns_by_id': turns,
        'initial_raw_rad_by_id': {str(i): raw[i] for i in sorted(raw)},
        'target_model_rad_by_id': {str(i): target[i] for i in sorted(target)},
        'maximum_nominal_joint_delta_deg': max_delta_deg,
        'maximum_stationary_pd_estimate_nm': max_stall_pd_nm,
        'nominal_four_foot_height_spread_mm': 1000 * (max(foot_z) - min(foot_z)),
        'nominal_four_foot_plane_residual_mm': plane_residual_mm,
        'screen_failures': failures,
        'readiness_blockers': blockers,
        'motor_output_allowed': False,
        'load_transfer_verified': False,
        'actual_body_rise_verified': False,
        'box_removal_allowed': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--rise-mm', type=float, required=True)
    parser.add_argument('--clearance-deg', type=float, required=True)
    parser.add_argument('--body-up-vector', nargs=3, type=float,
                        default=(0., 0., 1.), metavar=('X', 'Y', 'Z'),
                        help='Offline up direction in body coordinates; default is body +Z, not measured gravity')
    parser.add_argument('--rebase-offline', action='store_true',
                        help='Use only historic sign/zero with a changed pose; never grants output')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output must be a fresh path')
    result = plan(args.profile, args.capture, args.urdf,
                  args.rise_mm, args.clearance_deg,
                  rebase_offline=args.rebase_offline,
                  body_up_vector=args.body_up_vector)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in (
        'status', 'rise_mm', 'maximum_nominal_joint_delta_deg',
        'maximum_stationary_pd_estimate_nm',
        'nominal_four_foot_plane_residual_mm',
        'screen_failures', 'readiness_blockers',
        'motor_output_allowed')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
