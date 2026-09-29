"""Build and inspect a five-second preload/return path without hardware access.

This file-only candidate does not arm motors or grant runtime approval.
The old profile supplies calibration and limits, not a new power epoch.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from singularitydog_hw.policy_live_profile import load_profile
from singularitydog_hw import policy_shadow
from tools.plan_supported_preload import plan, LEGS, foot_down_target, finite_number, IDS, source_fingerprints
from tools.screen_box_lift_offline import parse_d17, foot_centers
from tools.offline_box_rise_candidate import _ik_near

PERIOD = .02
TICKS = 251
MAX_DISPLACEMENT = math.radians(1.)
MAX_SPEED = math.radians(1.)
MAX_ACCELERATION = math.radians(5.)


def rise_fraction(t):
    t = finite_number(t, 'Path time')
    if not 0 <= t <= 5:
        raise ValueError('Time must be within the finite five-second path')
    if t <= 1 or t >= 4:
        return 0.
    if t < 2.25:
        s = (t-1)/1.25
    elif t <= 2.75:
        return 1.
    else:
        s = (4-t)/1.25
    return s*s*s*(10+s*(-15+6*s))


def audit_samples(samples, initial, signs, raw_start):
    identities = IDS
    if any(set(values) != identities for values in (initial, signs, raw_start)):
        raise ValueError('Exactly twelve initial joint identities are required')
    if any(type(signs[i]) not in (int, float) or signs[i] not in (-1, 1) for i in identities):
        raise ValueError('Invalid initial joint calibration')
    for i in identities:
        finite_number(initial[i], 'Initial model angle')
        finite_number(raw_start[i], 'Initial raw angle')
    if type(samples) is not list or len(samples) != TICKS:
        raise ValueError('All 251 samples are required')
    maxima = dict(displacement_rad=0., speed_rad_s=0., acceleration_rad_s2=0.)
    prior_q = None
    prior_v = None
    for n, row in enumerate(samples):
        if set(row) != {'time_s', 'rise_fraction', 'q_model_rad_by_id', 'q_raw_rad_by_id'}:
            raise ValueError('Path sample fields differ')
        t = finite_number(row['time_s'], 'Path time')
        fraction = finite_number(row['rise_fraction'], 'Rise fraction')
        if not math.isclose(fraction, rise_fraction(t), rel_tol=0., abs_tol=1e-12):
            raise ValueError('Rise schedule differs from the five-second trial')
        if not math.isclose(t, n*PERIOD, rel_tol=0., abs_tol=1e-12):
            raise ValueError('Path cadence differs from 20ms')
        if set(row['q_model_rad_by_id']) != set(initial) or set(row['q_raw_rad_by_id']) != set(initial):
            raise ValueError('Path must retain all twelve joint identities')
        velocity = {}
        for mid, q0 in initial.items():
            q = finite_number(row['q_model_rad_by_id'][mid], 'Model target')
            raw = finite_number(row['q_raw_rad_by_id'][mid], 'Raw target')
            if (t <= 1. or t >= 4.) and abs(q-q0) > 1e-10:
                raise ValueError('Path must retain initial/final one-second captured-pose hold')
            if abs(raw-(raw_start[mid]+signs[mid]*(q-q0))) > 1e-10:
                raise ValueError('Path changed the encoder branch')
            if not -12.57+math.radians(5) <= raw <= 12.57-math.radians(5):
                raise ValueError('Path leaves the Type1 raw position range')
            index = policy_shadow.CAN_ORDER.index(int(mid))
            if not policy_shadow.LOWER[index] <= q <= policy_shadow.UPPER[index]:
                raise ValueError('Path leaves the model joint range')
            delta = abs(q-q0)
            maxima['displacement_rad'] = max(maxima['displacement_rad'], delta)
            if delta > MAX_DISPLACEMENT+1e-10:
                raise ValueError('Path exceeds the one-degree displacement cap')
            if prior_q is not None:
                v = (q-prior_q[mid])/PERIOD
                velocity[mid] = v
                maxima['speed_rad_s'] = max(maxima['speed_rad_s'], abs(v))
                if abs(v) > MAX_SPEED+1e-10:
                    raise ValueError('Path exceeds the one-degree-per-second speed cap')
                if prior_v is not None:
                    acceleration = abs(v-prior_v[mid])/PERIOD
                    maxima['acceleration_rad_s2'] = max(maxima['acceleration_rad_s2'], acceleration)
                    if acceleration > MAX_ACCELERATION+1e-10:
                        raise ValueError('Path exceeds the acceleration cap')
        prior_q = row['q_model_rad_by_id']
        if velocity:
            prior_v = velocity
    for row in (samples[0], samples[-1]):
        if any(abs(row['q_model_rad_by_id'][i]-q) > 1e-10 for i, q in initial.items()):
            raise ValueError('Path must start and return to the captured pose')
    return maxima


def audit_stationary_pd(samples, initial, axes):
    """Worst position error if the mechanism remains at the captured pose.

    This is a stationary, zero-velocity estimate, not a load or torque guarantee.
    Measured velocity and torque still require monitoring during execution.
    """
    if any(set(values) != IDS for values in (initial, axes)) or not samples:
        raise ValueError('Exactly twelve PD estimate axes and nonempty samples required')
    maximum = 0.
    for mid, q0 in initial.items():
        q0 = finite_number(q0, 'Initial model angle')
        kp = finite_number(axes[mid]['kp'], 'PD gain')
        limit = finite_number(axes[mid]['max_estimated_pd_torque_nm'], 'PD limit')
        if not math.isfinite(kp) or kp < 0 or not math.isfinite(limit) or limit <= 0:
            raise ValueError('Invalid PD estimate parameters')
        for row in samples:
            if set(row['q_model_rad_by_id']) != IDS:
                raise ValueError('Exactly twelve PD estimate sample axes required')
            value = finite_number(row['q_model_rad_by_id'][mid], 'PD model target')
            torque = kp*abs(value-q0)
            if not math.isfinite(torque) or torque > limit:
                raise ValueError(f'ID{mid} path exceeds stationary PD estimate limit')
            maximum = max(maximum, torque)
    return maximum


def build(profile_path, capture_path, urdf_path, *, rise_mm=.25, body_up_vector=(0., 0., 1.)):
    rise_mm = finite_number(rise_mm, 'Rise')
    if not 0 < rise_mm <= .25:
        raise ValueError('This candidate is limited to 0.25mm')
    sources = {**source_fingerprints(), str(Path(__file__).resolve()): hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    inputs = {str(Path(path).resolve()): hashlib.sha256(Path(path).read_bytes()).hexdigest()
              for path in (profile_path, capture_path, urdf_path)}
    base = plan(profile_path, capture_path, urdf_path, rise_mm, 3.,
                rebase_offline=True, body_up_vector=body_up_vector)
    profile = load_profile(profile_path, require_approved=True)
    if profile['profile_sha256'] != base['profile_sha256']:
        raise ValueError('Profile changed between endpoint and path planning')
    geometry = parse_d17(Path(urdf_path).read_bytes())
    initial = base['initial_model_rad_by_id']
    raw_start = base['initial_raw_rad_by_id']
    signs = {i: profile['axes'][i]['sign'] for i in initial}
    initial_int = {int(i): q for i, q in initial.items()}
    feet = foot_centers(initial_int, geometry)
    previous = initial_int.copy()
    samples = []
    max_fk_error_mm = 0.
    for tick in range(TICKS):
        t = tick*PERIOD
        fraction = rise_fraction(t)
        q = initial_int.copy()
        wanted = {}
        for leg, ids in LEGS.items():
            wanted[leg], _ = foot_down_target(feet[leg], rise_mm*fraction, body_up_vector)
            if fraction:
                solved = _ik_near(tuple(previous[i] for i in ids), wanted[leg], geometry[leg],
                                  tolerance_m=1e-9)
                q.update(zip(ids, solved))
        actual = foot_centers(q, geometry)
        for leg in LEGS:
            error_mm = math.dist(actual[leg], wanted[leg])*1000
            max_fk_error_mm = max(max_fk_error_mm, error_mm)
            if error_mm > .001:
                raise ValueError('Inverse/forward kinematics disagree by more than 0.001mm')
        model = {str(i): q[i] for i in sorted(q)}
        raw = {i: raw_start[i]+signs[i]*(v-initial[i]) for i, v in model.items()}
        samples.append(dict(time_s=t, rise_fraction=fraction,
                            q_model_rad_by_id=model, q_raw_rad_by_id=raw))
        previous = q
    maxima = audit_samples(samples, initial, signs, raw_start)
    maxima['stationary_pd_estimate_nm'] = audit_stationary_pd(samples, initial, profile['axes'])
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest
           for p, digest in {**sources, **inputs}.items()):
        raise ValueError('Source input or implementation changed during path planning')
    status = ('FILE_ONLY_PATH_SCREEN_FAILED' if base['screen_failures'] else
              'FILE_ONLY_PATH_REVIEW_REQUIRED' if base['readiness_blockers'] else
              'VALIDATED_FILE_ONLY_PATH')
    return dict(schema='singularitydog.supported-preload-path-file-only.v1',
                status=status, duration_s=5., period_s=PERIOD,
                kinematic_path_checks_passed=True,
                source_screen_passed=not base['screen_failures'] and not base['readiness_blockers'],
                source_fingerprints=sources,
                path_caps=dict(displacement_rad=MAX_DISPLACEMENT, speed_rad_s=MAX_SPEED,
                               acceleration_rad_s2=MAX_ACCELERATION, max_fk_error_mm=.001),
                source_screen=base, samples=samples, sampled_maxima=maxima,
                max_fk_error_mm=max_fk_error_mm,
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                motor_output_allowed=False, approved_for_runtime=False,
                learned_model_output=False, box_removal_allowed=False,
                blockers=base['readiness_blockers']+base['screen_failures']+[
                    'Dedicated runtime and fault/STOP tests are not bound to this candidate',
                    'Current power epoch and physical path need execution-time binding'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('profile', 'capture', 'urdf', 'output'):
        ap.add_argument('--'+name, type=Path, required=True)
    ap.add_argument('--body-up-vector', nargs=3, type=float, default=(0., 0., 1.))
    a = ap.parse_args()
    if a.output.exists():
        ap.error('Output must be new')
    result = build(a.profile, a.capture, a.urdf, body_up_vector=a.body_up_vector)
    with a.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('status', 'sampled_maxima',
                     'max_fk_error_mm', 'motor_output_allowed', 'blockers')}))


if __name__ == '__main__':
    main()
