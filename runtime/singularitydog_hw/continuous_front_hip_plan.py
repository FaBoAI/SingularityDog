"""Offline plan for one finite, uninterrupted front-hip diagnostic.

This module does not open a bus, enable a motor, or authorize a physical path.
The caller must independently review the *entire* outward 0→5→10° sweep
    and the clearance after the final STOP releases the joints,
    capture all twelve fresh starts on the current boot, check every reply at each
    50 ms tick and at each waypoint, and issue twelve STOPs on every exit. In
particular, separate calls to the existing step runner are not continuous:
that runner always STOPs and releases all motors at the end of each call.
"""

from dataclasses import dataclass
import math

from .fullbody_step10_plan import quintic_fraction
from .rs05_trial_protocol import POSITION_MAX, POSITION_MIN, TrialPhase, motion_request


ALL_IDS = tuple(range(1, 13))
MOVING_IDS = (3, 6)
RAW_DIRECTIONS = {3: 1, 6: -1}
CYCLE_S = .05
INITIAL_HOLD_TICKS = 20
RAMP_TICKS = 160
HOLD_TICKS = 20
TICKS_PER_SEGMENT = RAMP_TICKS + HOLD_TICKS
WAYPOINTS_DEG = (5., 10.)
MAX_EXCURSION_DEG = 10.


@dataclass(frozen=True)
class ContinuousFrontHipPlan:
    start_raw_rad_by_id: dict
    waypoints_deg: tuple
    waypoint_targets: tuple
    initial_hold_end_tick: int
    segment_end_ticks: tuple
    samples: tuple

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, tick):
        return self.samples[tick]


def _real(value, label):
    if type(value) not in (float, int):
        raise ValueError(f'{label} must be a finite real number')
    try:
        number = float(value)
    except OverflowError:
        raise ValueError(f'{label} must be finite') from None
    if not math.isfinite(number):
        raise ValueError(f'{label} must be finite')
    return number


def _waypoints(value):
    if type(value) is not list or len(value) != 2:
        raise ValueError('Exactly the two explicit front-hip waypoints are required')
    points = tuple(_real(v, f'waypoint {index}') for index, v in enumerate(value))
    if points != WAYPOINTS_DEG:
        raise ValueError('Only the reviewed 5-degree then 10-degree path is supported')
    return points


def build_front_hip_continuous_plan(centers, waypoints_deg):
    """Plan a one-second hold then 5° and 10° under one Enable/final STOP.

    ``waypoints_deg`` are nonnegative displacements relative to the same fresh
    twelve-axis start. ID3 increases, ID6 decreases, and the other ten axes
    remain at their fresh start. Exactly ``[5.0, 10.0]`` is supported in this
    first version. A 20-tick initial hold, followed by a 160-tick quintic ramp
    and 20-tick endpoint hold for *each* segment, gives 380 ticks/19 seconds.
    The duplicate endpoint at a segment boundary deliberately preserves
    position continuity and zero desired velocity. This plan does not reduce
    the current safety monitors or replace the separately confirmed five-second
    current-position hold prerequisite.
    """
    if (type(centers) is not dict or set(centers) != set(ALL_IDS)
            or any(type(mid) is not int for mid in centers)):
        raise ValueError('Fresh centers must contain exactly integer IDs 1..12')
    starts = {mid: _real(centers[mid], f'ID{mid} fresh center') for mid in ALL_IDS}
    if any(not POSITION_MIN <= value <= POSITION_MAX for value in starts.values()):
        raise ValueError('A fresh center exceeds the RS05 protocol position range')
    waypoints = _waypoints(waypoints_deg)
    targets = []
    for degrees in waypoints:
        # Leave the same sub-nanoradian arithmetic margin used by the existing
        # ten-degree wire plan, so offset subtraction cannot round above 10°.
        offset = math.radians(degrees) - (1e-12 if degrees > 0. else 0.)
        target = {mid: starts[mid] + RAW_DIRECTIONS.get(mid, 0) * offset
                  for mid in ALL_IDS}
        if any(not POSITION_MIN <= value <= POSITION_MAX for value in target.values()):
            raise ValueError('Waypoint exceeds the unwrapped RS05 protocol position range')
        for mid in MOVING_IDS:
            motion_request(phase=TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10,
                           center_rad=starts[mid], offset_rad=target[mid]-starts[mid],
                           motor_id=mid)
        targets.append(target)

    samples = [dict(starts) for _ in range(INITIAL_HOLD_TICKS)]
    previous = starts
    for target in targets:
        for local_tick in range(TICKS_PER_SEGMENT):
            fraction = quintic_fraction(min(local_tick / RAMP_TICKS, 1.))
            sample = {mid: previous[mid] + (target[mid]-previous[mid]) * fraction
                      for mid in ALL_IDS}
            samples.append(sample)
        previous = target
    ends = tuple(INITIAL_HOLD_TICKS + (index + 1) * TICKS_PER_SEGMENT - 1
                 for index in range(len(targets)))
    if (samples[0] != starts or samples[INITIAL_HOLD_TICKS - 1] != starts
            or any(samples[end] != target for end, target in zip(ends, targets))
            or len(samples) != 380):
        raise AssertionError('Finite continuous trajectory endpoints changed')
    return ContinuousFrontHipPlan(starts, waypoints, tuple(targets),
                                  INITIAL_HOLD_TICKS - 1, ends, tuple(samples))
