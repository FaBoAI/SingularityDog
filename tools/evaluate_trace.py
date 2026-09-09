#!/usr/bin/env python3
"""First-episode locomotion metrics for an explicit, regularly sampled JSON trace."""
import argparse
import json
import math
from pathlib import Path

FEET = ('FL', 'FR', 'RL', 'RR')


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Finite number required')
    return float(value)


def vector(value, count):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError('Wrong vector length')
    return [number(x) for x in value]


def evaluate(trace):
    if trace.get('schema') != 'singularitydog.trace.v1':
        raise ValueError('Unsupported trace schema')
    dt, duration = number(trace['sample_dt_s']), number(trace['requested_duration_s'])
    if dt <= 0 or duration < dt or abs(duration / dt - round(duration / dt)) > 1e-8:
        raise ValueError('Positive regular sampling and integral duration required')
    direction = vector(trace['direction_world_xy'], 2)
    norm = math.hypot(*direction)
    if abs(norm - 1) > 1e-8:
        raise ValueError('Fixed world direction must be normalized')
    initial = vector(trace['initial_position_world_m'], 3)
    heading = number(trace['initial_heading_rad'])
    initial_heading = heading
    enter, leave = 2.0, 0.5
    feet = {name: {'state': None, 'pending': None, 'count': 0, 'peak': None, 'completed': 0, 'lift15': 0,
                   'slip_sq': 0.0, 'slip_distance': 0.0, 'slip_count': 0} for name in FEET}
    rows = trace['samples']
    if not isinstance(rows, list) or not rows:
        raise ValueError('No actual samples')
    used, velocity_sum, failed = 0, 0.0, False
    pos = initial
    target = number(trace.get('target_speed_m_s', 0.2))
    if target <= 0:
        raise ValueError('Positive target speed required')
    for row in rows[:round(duration / dt)]:
        t = number(row['time_s'])
        if abs(t - (used + 1) * dt) > 1e-8:
            raise ValueError('Missing, shifted, duplicate or irregular samples')
        pos = vector(row['position_world_m'], 3)
        vel = vector(row['velocity_world_m_s'], 3)
        next_heading = number(row['heading_rad'])
        heading += math.atan2(math.sin(next_heading - heading), math.cos(next_heading - heading))
        velocity_sum += sum(vel[i] * direction[i] for i in range(2))
        if set(row['feet']) != set(FEET):
            raise ValueError('Exactly four named feet required')
        for name in FEET:
            sample, state = row['feet'][name], feet[name]
            clearance = number(sample['clearance_m'])
            force = number(sample['normal_force_N'])
            slip = vector(sample['material_point_velocity_world_xy_m_s'], 2)
            if force < 0:
                raise ValueError('Normal-force magnitude cannot be negative')
            candidate = True if force >= enter else False if force <= leave else state['state']
            if candidate is not None and candidate != state['state']:
                state['count'] = state['count'] + 1 if state['pending'] == candidate else 1
                state['pending'] = candidate
                if state['count'] >= 2:
                    previous = state['state']
                    state['state'], state['pending'], state['count'] = candidate, None, 0
                    if previous is True and candidate is False:
                        state['peak'] = clearance
                    elif previous is False and candidate is True and state['peak'] is not None:
                        state['completed'] += 1
                        state['lift15'] += int(state['peak'] >= 0.015)
                        state['peak'] = None
            else:
                state['pending'], state['count'] = None, 0
            if state['state'] is False and state['peak'] is not None:
                state['peak'] = max(state['peak'], clearance)
            if state['state'] is True and force >= leave:
                speed = math.hypot(*slip)
                state['slip_distance'] += speed * dt
                state['slip_sq'] += speed * speed
                state['slip_count'] += 1
        if any(not isinstance(row[k], bool) for k in ('terminal', 'nonfoot_contact')):
            raise ValueError('Explicit boolean terminal flags required')
        used += 1
        if row['terminal'] or row['nonfoot_contact']:
            failed = True
            break
    elapsed = used * dt
    complete = used == round(duration / dt) and not failed
    velocity = velocity_sum / used
    return {'schema': 'singularitydog.trace-metrics.v1', 'observed_seconds': elapsed, 'requested_seconds': duration,
            'completed_samples': used, 'discarded_after_first_end': len(rows) - used,
            'completed_requested_interval': complete, 'first_failure_step': used if failed else None,
            'mean_velocity_along_fixed_world_direction_m_s': velocity,
            'net_translation_along_fixed_world_direction_m': sum((pos[i] - initial[i]) * direction[i] for i in range(2)),
            'net_translation_over_observed_time_m_s': sum((pos[i] - initial[i]) * direction[i] for i in range(2)) / elapsed,
            'heading_change_deg': math.degrees(heading - initial_heading),
            'speed_target_met_in_complete_interval': complete and velocity >= target,
            'walking_acceptance': False,
            'feet': {name: {'completed_swings': s['completed'], 'completed_swings_peak_ge_15mm': s['lift15'],
                            'end_censored_swing': s['peak'] is not None,
                            'material_point_slip_distance_m': s['slip_distance'],
                            'material_point_slip_rms_m_s': math.sqrt(s['slip_sq'] / s['slip_count']) if s['slip_count'] else None}
                     for name, s in feet.items()},
            'limitations': ['Force hysteresis delays timing; initial airborne and end-censored swings are excluded.',
                            'Material-point slip is not measured contact-patch slip.',
                            'Sampled development metrics do not establish hardware readiness or high-rate torque feasibility.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate(json.loads(args.trace.read_text())), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
