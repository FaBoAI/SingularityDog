"""File-only plan for a slow position transition toward the 30 Hz model's nominal stance.

Stage A of the load-support ladder: from the current box-supported pose,
interpolate all twelve joints together on one minimum-jerk (quintic)
profile toward the controller's nominal stance, optionally stopping at a
fraction of the way. With the feet on the floor this raises the body and
transfers load, so live use requires a full-weight support, a cutoff
operator and a separately admitted contract. This module opens no device,
loads no model and grants no motion.
"""
import argparse
import json
import math
from pathlib import Path

SCHEMA = 'singularitydog.stance-transition-plan.v1'
CYCLE_HZ = 30
# Conservative defaults; a live contract must review and pin its own values.
DEFAULT_MAX_VELOCITY_RAD_S = math.radians(10.)
DEFAULT_MAX_ACCELERATION_RAD_S2 = math.radians(20.)
LIMIT_MARGIN_RAD = math.radians(2.)
QUINTIC_PEAK_VELOCITY = 1.875           # max d/dt of 10s^3-15s^4+6s^5 is 15/8.
QUINTIC_PEAK_ACCELERATION = 10/math.sqrt(3)  # max second derivative is 10/sqrt(3).
# Frozen controller nominal stance (float64, radians) in CAN_ORDER
# (singularitydog_hw.policy_shadow.CAN_ORDER): about -16.04/21.99/-43.97 deg.
_HIP, _THIGH, _CALF = 0.2800237445733691, 0.3837207788252659, -0.7674415576505315
NOMINAL_STANCE_RAD = (-_HIP, _THIGH, _CALF, _HIP, _THIGH, _CALF, -_HIP, _THIGH, _CALF, _HIP, _THIGH, _CALF)
CAN_ORDER = (6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7)
# Leg geometry of the frozen controller fk(): thigh/calf 0.12 m, hip lateral 0.064 m.
THIGH_M = CALF_M = .12
HIP_LATERAL_M = .064
LEG_SIGNS = (1., -1., 1., -1.)  # Frozen controller signs for the four CAN_ORDER legs.
LEG_ORIGINS = ((.155, .11, 0.), (.155, -.11, 0.), (-.155, .11, 0.), (-.155, -.11, 0.))
MAX_LIFT_M = .02


def need(condition, message):
    if not condition:
        raise ValueError(message)


def quintic(s):
    return s*s*s*(10.+s*(-15.+6.*s))


def leg_height_m(hip, thigh, calf, sign):
    """Vertical foot drop below the hip joint, from the frozen controller fk()."""
    z = -THIGH_M*(math.cos(thigh)+math.cos(thigh+calf))
    return -(math.sin(hip)*HIP_LATERAL_M*sign+math.cos(hip)*z)


def foot_m(hip, thigh, calf, leg):
    """Foot position in the body frame, exactly the frozen controller fk()."""
    x = -THIGH_M*(math.sin(thigh)+math.sin(thigh+calf))
    z = -THIGH_M*(math.cos(thigh)+math.cos(thigh+calf))
    y = HIP_LATERAL_M*LEG_SIGNS[leg]
    ox, oy, oz = LEG_ORIGINS[leg]
    return (x+ox, math.cos(hip)*y-math.sin(hip)*z+oy, math.sin(hip)*y+math.cos(hip)*z+oz)


def _wrap(angle):
    return math.remainder(angle, 2*math.pi)


def leg_ik(foot, leg):
    """Joint angles reaching a body-frame foot position, as the frozen controller ik()."""
    ox, oy, oz = LEG_ORIGINS[leg]
    x, y, z = foot[0]-ox, foot[1]-oy, foot[2]-oz
    rad = y*y+z*z-HIP_LATERAL_M**2
    cosine = (x*x+rad-2*THIGH_M**2)/(2*THIGH_M**2)
    need(rad > 0. and -1. <= cosine <= 1., 'Foot target outside the leg IK domain')
    zs = -math.sqrt(rad)
    calf = -math.acos(cosine)
    hip = _wrap(math.atan2(z, y)-math.atan2(zs, HIP_LATERAL_M*LEG_SIGNS[leg]))
    thigh = _wrap(math.atan2(-x, -zs)-math.atan2(THIGH_M*math.sin(calf), THIGH_M+THIGH_M*math.cos(calf)))
    return hip, thigh, calf


def _legs(q_by_id):
    canonical = [q_by_id[mid] for mid in CAN_ORDER]
    return [canonical[i:i+3] for i in range(0, 12, 3)]


def body_height_m(q_by_id):
    """Mean hip height above the feet for one pose (model radians by motor ID)."""
    return sum(leg_height_m(*leg, sign) for leg, sign in zip(_legs(q_by_id), LEG_SIGNS))/4.


def target_by_id():
    return dict(zip(CAN_ORDER, NOMINAL_STANCE_RAD))


def make_plan(start_by_id, limits_by_id, *, fraction=1., max_velocity=DEFAULT_MAX_VELOCITY_RAD_S,
              max_acceleration=DEFAULT_MAX_ACCELERATION_RAD_S2, hold_s=1.):
    need(set(start_by_id) == set(range(1, 13)) and set(limits_by_id) == set(range(1, 13)),
         'Exact twelve motor IDs required')
    need(type(fraction) in (int, float) and 0. < fraction <= 1., 'Fraction must be in (0, 1]')
    need(0. < max_velocity <= math.radians(30.) and 0. < max_acceleration <= math.radians(60.),
         'Velocity/acceleration caps outside the reviewed planning range')
    need(0. <= hold_s <= 5., 'Hold must be 0..5 s')
    goal = {mid: start_by_id[mid]+fraction*(q-start_by_id[mid]) for mid, q in target_by_id().items()}
    result = _plan(start_by_id, goal, limits_by_id, max_velocity, max_acceleration, hold_s)
    result['fraction'] = fraction
    return result


def make_lift_plan(start_by_id, limits_by_id, *, lift_m, max_velocity=DEFAULT_MAX_VELOCITY_RAD_S,
                   max_acceleration=DEFAULT_MAX_ACCELERATION_RAD_S2, hold_s=1.):
    """Raise the body straight up by lift_m with every foot fixed in place.

    The linear joint interpolation toward the nominal stance mostly moves the
    hips sideways and drags planted feet across the floor. Here each foot keeps
    its body-frame x and y and moves lift_m down relative to the body, solved
    with the frozen controller IK, so the legs extend vertically.
    """
    need(set(start_by_id) == set(range(1, 13)) and set(limits_by_id) == set(range(1, 13)),
         'Exact twelve motor IDs required')
    need(type(lift_m) in (int, float) and 0. < lift_m <= MAX_LIFT_M, 'Lift must be in (0, %g] m' % MAX_LIFT_M)
    need(0. < max_velocity <= math.radians(30.) and 0. < max_acceleration <= math.radians(60.),
         'Velocity/acceleration caps outside the reviewed planning range')
    need(0. <= hold_s <= 5., 'Hold must be 0..5 s')
    goal = {}
    slide = 0.
    for leg, angles in enumerate(_legs(start_by_id)):
        foot = foot_m(*angles, leg)
        target = (foot[0], foot[1], foot[2]-lift_m)
        solved = leg_ik(target, leg)
        reached = foot_m(*solved, leg)
        need(max(abs(a-b) for a, b in zip(reached, target)) < 1e-9, 'Leg IK/FK closure failed')
        slide = max(slide, math.hypot(reached[0]-foot[0], reached[1]-foot[1]))
        for mid, value in zip(CAN_ORDER[3*leg:3*leg+3], solved):
            goal[mid] = value
    result = _plan(start_by_id, goal, limits_by_id, max_velocity, max_acceleration, hold_s)
    result.update(kind='vertical_lift_feet_fixed', lift_m=lift_m, max_foot_slide_m=slide)
    return result


def _plan(start_by_id, goal, limits_by_id, max_velocity, max_acceleration, hold_s):
    for mid in range(1, 13):
        lower, upper = limits_by_id[mid]
        for value, name in ((start_by_id[mid], 'start'), (goal[mid], 'goal')):
            need(lower+LIMIT_MARGIN_RAD <= value <= upper-LIMIT_MARGIN_RAD,
                 'ID%d %s outside model range with margin' % (mid, name))
    delta = max(abs(goal[mid]-start_by_id[mid]) for mid in range(1, 13))
    duration = max(QUINTIC_PEAK_VELOCITY*delta/max_velocity,
                   math.sqrt(QUINTIC_PEAK_ACCELERATION*delta/max_acceleration), 1./CYCLE_HZ)
    moving = math.ceil(duration*CYCLE_HZ)
    holding = math.ceil(hold_s*CYCLE_HZ)
    waypoints = []
    for index in range(moving+holding+1):
        s = quintic(min(1., index/moving))
        waypoints.append({mid: start_by_id[mid]+s*(goal[mid]-start_by_id[mid]) for mid in range(1, 13)})
    step_cap = max_velocity/CYCLE_HZ
    worst = 0.
    for before, after in zip(waypoints, waypoints[1:]):
        for mid in range(1, 13):
            worst = max(worst, abs(after[mid]-before[mid]))
            lower, upper = limits_by_id[mid]
            need(lower+LIMIT_MARGIN_RAD <= after[mid] <= upper-LIMIT_MARGIN_RAD,
                 'ID%d waypoint outside model range' % mid)
    need(worst <= step_cap*(1.+1e-9), 'Per-cycle step exceeds the velocity cap')
    need(all(abs(waypoints[-1][mid]-goal[mid]) < 1e-12 for mid in range(1, 13)), 'Goal not reached exactly')
    rise = body_height_m(goal)-body_height_m(start_by_id)
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY_NO_DEVICE', 'output_allowed': False,
            'requires': ['full_weight_support', 'cutoff_operator', 'fixed_video', 'separate_admitted_contract'],
            'cycle_hz': CYCLE_HZ, 'moving_cycles': moving, 'holding_cycles': holding,
            'duration_s': moving/CYCLE_HZ, 'max_velocity_rad_s': max_velocity,
            'max_acceleration_rad_s2': max_acceleration, 'max_joint_delta_deg': math.degrees(delta),
            'max_step_per_cycle_deg': math.degrees(worst),
            'start_body_height_m': body_height_m(start_by_id), 'goal_body_height_m': body_height_m(goal),
            'expected_body_rise_m': rise,
            'goal_by_id_deg': {str(mid): math.degrees(goal[mid]) for mid in range(1, 13)},
            'delta_by_id_deg': {str(mid): math.degrees(goal[mid]-start_by_id[mid]) for mid in range(1, 13)},
            'waypoints_by_id_rad': [[w[mid] for mid in range(1, 13)] for w in waypoints]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner-report', type=Path, required=True,
                        help='A completed Type1 report whose q0_model_rad_by_id is the start pose')
    parser.add_argument('--fraction', type=float, default=1.)
    parser.add_argument('--max-velocity-deg-s', type=float, default=math.degrees(DEFAULT_MAX_VELOCITY_RAD_S))
    parser.add_argument('--max-acceleration-deg-s2', type=float, default=math.degrees(DEFAULT_MAX_ACCELERATION_RAD_S2))
    parser.add_argument('--waypoints', action='store_true')
    args = parser.parse_args(argv)
    from singularitydog_hw.policy_output_runtime import MODEL_TARGET_LIMITS_BY_ID
    report = json.loads(args.runner_report.read_bytes())
    start = {int(k): float(v) for k, v in report['measurement']['q0_model_rad_by_id'].items()}
    result = make_plan(start, MODEL_TARGET_LIMITS_BY_ID, fraction=args.fraction,
                       max_velocity=math.radians(args.max_velocity_deg_s),
                       max_acceleration=math.radians(args.max_acceleration_deg_s2))
    if not args.waypoints:
        result.pop('waypoints_by_id_rad')
    print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
