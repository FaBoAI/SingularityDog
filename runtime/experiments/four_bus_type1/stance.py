"""Explicit box-kept stance-transition caps for the 30 Hz four-bus Type1 runner.

Selected only by the contract pacing option ``stance_transition`` (user
decisions 2026-10-17). The contract keeps the reviewed boxed caps for every
existing check; these fixed constants replace them only inside a selected
learned_boxed run. The box stays under the body and the feet stay fixed: the
target raises the body straight up. The learned model is still evaluated and
recorded but its target is not sent.

- 2 s: a small lift sized to STANCE_MAX_DELTA_RAD inside the reviewed +-3 deg window.
- 10 s with ``stance_lift_percent`` P: lift P % of the way to the controller
  nominal stance height, hold, lower back to the start, then the normal stop.
  Each axis window widens to the planned change plus WINDOW_MARGIN_RAD (user
  confirmed clearance for up to 2 cm / ~20 deg calf, then selected 50 % and
  65 % (~2.3 cm / ~30 deg calf) on 2026-10-17).
"""
import copy
import math

from experiments.stance_transition import plan as stance_plan

OPTION = 'stance_transition'
PERCENT_OPTION = 'stance_lift_percent'
PERCENT_RANGE = (1, 65)  # User 2026-10-17: 35 -> 50 -> 65 %; feet-fixed reach ends near 71 %.
KP, KD = 36., .3  # Staged 2026-10-17: Kp12/18/24 (joint step dragged feet), vertical lift Kp24 (~1 mm FK), Kp36 by user selection; equals the native hard cap.
STANCE_CAPS = {'kp': KP, 'kd': KD,
               'max_displacement_from_start_rad': math.radians(3),
               'max_estimated_pd_torque_nm': 2., 'max_measured_torque_nm': 3.,
               'max_command_velocity_rad_s': math.radians(15),  # User-loosened 10 -> 15 deg/s for 50-65 % in 10 s.
               'max_command_acceleration_rad_s2': math.radians(20),
               'max_tracking_error_rad': math.radians(3)}
STANCE_MAX_DELTA_RAD = math.radians(2.5)
STANCE_DURATION_S = 2
PERCENT_DURATIONS_S = (10, 20)
PROBE_LIFT_M = .003
WINDOW_MARGIN_RAD = math.radians(2)
MIN_HOLD_S = .5
# User-selected 2026-10-17 after the 35 % lift tripped 10.97 m/s^2 as the body left the box.
# Percent runs only; within the runner's IMU_LIMIT_RANGES. Tilt and gyro limits are unchanged.
PERCENT_IMU_ACCEL_NORM_M_S2 = (10.0, 11.2)
LOCAL_MARGIN_RAD = 2*25.14/65535  # Same as policy_live_profile.LOCAL_NUMERICAL_MARGIN_RAD (contract effective bounds).


def need(condition, message):
    if not condition:
        raise ValueError(message)


def selected(options):
    return options.get(OPTION) is True


def lift_percent(options):
    """The selected percent, or None. Only valid together with stance_transition."""
    value = options.get(PERCENT_OPTION)
    need(value is None or (type(value) is int and PERCENT_RANGE[0] <= value <= PERCENT_RANGE[1] and selected(options)),
         'stance_lift_percent must be an integer %d..%d with stance_transition' % PERCENT_RANGE)
    return value


def run_percent(options, mode, duration):
    """Percent applied to this run (10 s learned only); validates the duration ladder."""
    if not (selected(options) and mode == 'learned_boxed'):
        return None
    percent = lift_percent(options)
    need(duration == STANCE_DURATION_S or (percent is not None and duration in PERCENT_DURATIONS_S),
         'Stance transition learned_boxed is 2 s, or 10/20 s with stance_lift_percent')
    return percent if duration in PERCENT_DURATIONS_S else None


def _caps():
    return dict(max_velocity=STANCE_CAPS['max_command_velocity_rad_s'],
                max_acceleration=STANCE_CAPS['max_command_acceleration_rad_s2'])


def lift_for_percent(start_by_id, percent):
    full = stance_plan.body_height_m(stance_plan.target_by_id())-stance_plan.body_height_m(start_by_id)
    need(full > 0., 'Start is already at or above the nominal stance height')
    lift = full*percent/100.
    need(0. < lift <= stance_plan.MAX_LIFT_M, 'Percent lift outside the planner range')
    return lift


def percent_goal(start_by_id, percent, model_limits_by_id):
    return stance_plan.make_lift_plan(start_by_id, model_limits_by_id,
                                      lift_m=lift_for_percent(start_by_id, percent), hold_s=0., **_caps())


def widened_axes(axes, lineage_by_id, percent, model_limits_by_id, offsets_by_id=None):
    """Owned axes whose windows also cover the lineage-pose plan change plus margin."""
    plan = percent_goal(lineage_by_id, percent, model_limits_by_id)
    goal = {int(k): math.radians(v)+lineage_by_id[int(k)] for k, v in plan['delta_by_id_deg'].items()}
    result = copy.deepcopy(axes)
    for key, axis in result.items():
        mid = int(key)
        start = lineage_by_id[mid]
        # Union of the reviewed window and the planned range plus margin, inside the model range.
        lower = max(min(axis['physical_lower_rad'], min(start, goal[mid])-WINDOW_MARGIN_RAD), model_limits_by_id[mid][0])
        upper = min(max(axis['physical_upper_rad'], max(start, goal[mid])+WINDOW_MARGIN_RAD), model_limits_by_id[mid][1])
        need(lower < start < upper, 'ID%d widened window must contain the start' % mid)
        sign = axis['sign']
        # Fixed linear map q = offset + sign*raw: contract axes carry it, the runner passes its admitted offsets.
        offset = float(offsets_by_id[mid]) if offsets_by_id is not None else axis['fixed_offset_rad']
        need('fixed_offset_rad' not in axis or abs(offset-axis['fixed_offset_rad']) < 1e-9,
             'ID%d admitted offset and contract fixed offset differ' % mid)
        axis['physical_lower_rad'], axis['physical_upper_rad'] = lower, upper
        axis['lower_rad'], axis['upper_rad'] = lower+LOCAL_MARGIN_RAD, upper-LOCAL_MARGIN_RAD
        raw = sorted(((axis['lower_rad']-offset)/sign, (axis['upper_rad']-offset)/sign))
        need(-12.57 <= raw[0] < raw[1] <= 12.57, 'ID%d widened raw window exceeds the Type1 field' % mid)
        axis['raw_lower_rad'], axis['raw_upper_rad'] = raw
        axis['max_displacement_from_start_rad'] = abs(goal[mid]-start)+WINDOW_MARGIN_RAD
    return result


def lineage_pose(start_pose_bounds):
    """The contract lineage pose: the centre of each reviewed +-0.5 deg start interval."""
    return {int(key): (bounds[0]+bounds[1])/2. for key, bounds in start_pose_bounds.items()}


def profile_with_caps(profile, *, lineage_by_id=None, percent=None, model_limits_by_id=None, offsets_by_id=None):
    """Owned copy of the admitted profile with the fixed stance caps (and widened windows for a percent run)."""
    result = copy.deepcopy(profile)
    for axis in result['axes'].values():
        axis.update(STANCE_CAPS)
    if percent is not None:
        result['axes'] = widened_axes(result['axes'], lineage_by_id, percent, model_limits_by_id, offsets_by_id)
    for axis in result['axes'].values():
        need(axis['max_estimated_pd_torque_nm'] <= axis['max_measured_torque_nm'] and
             axis['max_command_velocity_rad_s'] <= axis['max_measured_velocity_rad_s'],
             'Stance PD/velocity budget exceeds the measured monitor')
    return result


def _check(waypoints, start, axis_rows, cap_by_id):
    for row in waypoints:
        for mid, (q, axis) in enumerate(zip(row, axis_rows), 1):
            need(axis['lower_rad'] <= q <= axis['upper_rad'], 'ID%d stance waypoint outside local window' % mid)
            need(abs(q-start[mid]) <= cap_by_id[mid]+1e-12, 'ID%d stance waypoint beyond delta cap' % mid)


def targets(q0_by_id, axis_rows, model_limits_by_id, *, percent=None, available_s=None):
    """Per-cycle stance targets (tuples in ID order) and a plan summary.

    Vertical lift with every foot fixed (the Kp12/18/24 runs showed the joint
    interpolation toward the nominal stance dragged planted feet sideways at the
    hips). Without a percent the lift is sized to STANCE_MAX_DELTA_RAD; with one,
    the body rises, holds and returns to the start within available_s.
    """
    start = {mid: float(q0_by_id[mid]) for mid in range(1, 13)}
    if percent is None:
        probe = stance_plan.make_lift_plan(start, model_limits_by_id, lift_m=PROBE_LIFT_M, hold_s=0., **_caps())
        lift = PROBE_LIFT_M*min(1., math.degrees(STANCE_MAX_DELTA_RAD)/probe['max_joint_delta_deg'])*.98
        result = stance_plan.make_lift_plan(start, model_limits_by_id, lift_m=lift, hold_s=0., **_caps())
        need(result['max_joint_delta_deg'] <= math.degrees(STANCE_MAX_DELTA_RAD), 'Lift exceeds the stance delta cap')
        waypoints = tuple(tuple(row) for row in result.pop('waypoints_by_id_rad'))
        _check(waypoints, start, axis_rows, {mid: STANCE_MAX_DELTA_RAD for mid in range(1, 13)})
    else:
        need(type(available_s) in (int, float) and available_s > 0, 'Available motion time required')
        # A shorter run of the same contract lifts the largest whole percent that fits (recorded below).
        for applied in range(percent, 0, -1):
            result = percent_goal(start, applied, model_limits_by_id)
            up = tuple(tuple(row) for row in result.pop('waypoints_by_id_rad'))
            hold = int(math.floor(available_s*stance_plan.CYCLE_HZ))-2*(len(up)-1)
            if hold >= MIN_HOLD_S*stance_plan.CYCLE_HZ:
                break
        need(hold >= MIN_HOLD_S*stance_plan.CYCLE_HZ, 'Lift and return do not fit the run with a %.1f s hold' % MIN_HOLD_S)
        waypoints = up+(up[-1],)*hold+tuple(reversed(up))
        need(waypoints[-1] == tuple(start[mid] for mid in range(1, 13)), 'Return must end exactly at the start')
        _check(waypoints, start, axis_rows, {mid: axis['max_displacement_from_start_rad']
                                              for mid, axis in enumerate(axis_rows, 1)})
        result.update(percent=applied, percent_requested=percent, hold_cycles=hold, return_to_start=True)
    result['waypoint_count'] = len(waypoints)
    return waypoints, result


def imu_limits_for_run(limits, percent):
    """IMU limits of this run: a percent lift widens only the accelerometer-norm interval."""
    result = dict(limits)
    if percent is not None:
        lower, upper = PERCENT_IMU_ACCEL_NORM_M_S2
        # Only ever widens: the union of the reviewed interval and the percent interval.
        result.update(imu_accel_norm_min_m_s2=min(lower, result['imu_accel_norm_min_m_s2']),
                      imu_accel_norm_max_m_s2=max(upper, result['imu_accel_norm_max_m_s2']))
    return result


def target_at(waypoints, elapsed_s, startup_s, cycle_hz=stance_plan.CYCLE_HZ):
    """Hold the start during startup, then follow the waypoints and hold the last."""
    index = 0 if elapsed_s <= startup_s else int(math.floor((elapsed_s-startup_s)*cycle_hz))
    return waypoints[min(index, len(waypoints)-1)]
