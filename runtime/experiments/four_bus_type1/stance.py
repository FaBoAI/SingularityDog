"""Explicit box-kept stance-transition caps for the 30 Hz four-bus Type1 runner.

Selected only by the contract pacing option ``stance_transition`` (user
decision 2026-10-17: staged gains starting at Kp12). The contract keeps the
reviewed boxed caps for every existing check; these fixed constants replace
them only inside a learned_boxed 2 s run that selected the option. The box
stays under the body: the target raises it straight up with the feet fixed,
limited to STANCE_MAX_DELTA_RAD so it stays inside the reviewed +-3 deg window. The
learned model is still evaluated and recorded but its target is not sent.
"""
import copy
import math

from experiments.stance_transition import plan as stance_plan

OPTION = 'stance_transition'
KP, KD = 24., .3  # Staged 2026-10-17: Kp12 -> Kp18 (joints ~32% -> ~41% of command, no lift seen) -> Kp24 by user selection.
STANCE_CAPS = {'kp': KP, 'kd': KD,
               'max_displacement_from_start_rad': math.radians(3),
               'max_estimated_pd_torque_nm': 2., 'max_measured_torque_nm': 3.,
               'max_command_velocity_rad_s': math.radians(10),
               'max_command_acceleration_rad_s2': math.radians(20),
               'max_tracking_error_rad': math.radians(3)}
STANCE_MAX_DELTA_RAD = math.radians(2.5)
STANCE_DURATION_S = 2
PROBE_LIFT_M = .003


def need(condition, message):
    if not condition:
        raise ValueError(message)


def selected(options):
    return options.get(OPTION) is True


def profile_with_caps(profile):
    """Owned copy of the admitted profile with the fixed stance caps on every axis."""
    result = copy.deepcopy(profile)
    for axis in result['axes'].values():
        axis.update(STANCE_CAPS)
        need(axis['max_estimated_pd_torque_nm'] <= axis['max_measured_torque_nm'] and
             axis['max_command_velocity_rad_s'] <= axis['max_measured_velocity_rad_s'],
             'Stance PD/velocity budget exceeds the measured monitor')
    return result


def targets(q0_by_id, axis_rows, model_limits_by_id):
    """Per-cycle stance targets (tuples in ID order) and a plan summary.

    Vertical lift with every foot fixed (the Kp12/18/24 runs showed the joint
    interpolation toward the nominal stance dragged planted feet sideways at the
    hips). The lift is sized so every joint stays within STANCE_MAX_DELTA_RAD,
    and every waypoint must also stay inside each axis's effective local window.
    """
    start = {mid: float(q0_by_id[mid]) for mid in range(1, 13)}
    caps = dict(max_velocity=STANCE_CAPS['max_command_velocity_rad_s'],
                max_acceleration=STANCE_CAPS['max_command_acceleration_rad_s2'], hold_s=0.)
    probe = stance_plan.make_lift_plan(start, model_limits_by_id, lift_m=PROBE_LIFT_M, **caps)
    lift = PROBE_LIFT_M*min(1., math.degrees(STANCE_MAX_DELTA_RAD)/probe['max_joint_delta_deg'])*.98
    result = stance_plan.make_lift_plan(start, model_limits_by_id, lift_m=lift, **caps)
    need(result['max_joint_delta_deg'] <= math.degrees(STANCE_MAX_DELTA_RAD), 'Lift exceeds the stance delta cap')
    waypoints = tuple(tuple(row) for row in result.pop('waypoints_by_id_rad'))
    for row in waypoints:
        for mid, (q, axis) in enumerate(zip(row, axis_rows), 1):
            need(axis['lower_rad'] <= q <= axis['upper_rad'], 'ID%d stance waypoint outside local window' % mid)
            need(abs(q-start[mid]) <= STANCE_MAX_DELTA_RAD+1e-12, 'ID%d stance waypoint beyond delta cap' % mid)
    result['waypoint_count'] = len(waypoints)
    return waypoints, result


def target_at(waypoints, elapsed_s, startup_s, cycle_hz=stance_plan.CYCLE_HZ):
    """Hold the start during startup, then follow the waypoints and hold the last."""
    index = 0 if elapsed_s <= startup_s else int(math.floor((elapsed_s-startup_s)*cycle_hz))
    return waypoints[min(index, len(waypoints)-1)]
