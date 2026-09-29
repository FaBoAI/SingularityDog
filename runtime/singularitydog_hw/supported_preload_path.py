"""Finite geometric path validation and sampling; no device access or approval.

The profile loader supplies a separate reviewed admission. This module checks
the original file-only artifact again and binds only the explicitly reviewed
small capture/feedback quantization difference. Faults never request a return
trajectory: the caller must send STOP instead.
"""
from dataclasses import dataclass, field
import math

from . import policy_shadow

IDS = tuple(str(i) for i in range(1, 13))
PERIOD_S = .02
SAMPLE_COUNT = 251
RETURN_COMPLETE_S = 4.
MAX_ANCHOR_DIFFERENCE_RAD = math.radians(.05)
COMMAND_RETURN_TOLERANCE_RAD = 2*25.14/65535
MEASURED_RETURN_TOLERANCE_RAD = math.radians(.15)
_MODEL_LIMITS = {str(mid): (lo, hi) for mid, lo, hi in
                 zip(policy_shadow.CAN_ORDER, policy_shadow.LOWER, policy_shadow.UPPER)}


def _need(value, message):
    if not value:
        raise ValueError('Preload: '+message)


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _vector(values, label):
    _need(isinstance(values, (tuple, list)) and len(values) == 12 and
          all(_finite(v) for v in values), label+' needs twelve finite values')
    return tuple(values)


def _by_id(values, label):
    _need(type(values) is dict and set(values) == set(IDS), label+' needs exact IDs 1..12')
    return _vector(tuple(values[mid] for mid in IDS), label)


def _fraction(t):
    if t <= 1 or t >= 4:
        return 0.
    if t <= 2.25:
        x = (t-1)/1.25
    elif t <= 2.75:
        return 1.
    else:
        x = (4-t)/1.25
    return x*x*x*(10+x*(-15+6*x))


@dataclass(frozen=True)
class PreloadPath:
    initial_model: tuple
    initial_raw: tuple
    deltas: tuple
    signs: tuple
    lower: tuple
    upper: tuple
    kp: tuple
    pd_limits: tuple

    def check_origin(self, model, raw):
        model = _vector(model, 'fresh model origin')
        raw = _vector(raw, 'fresh raw origin')
        for n, (q, r) in enumerate(zip(model, raw)):
            _need(abs(q-self.initial_model[n]) <= MAX_ANCHOR_DIFFERENCE_RAD and
                  abs(r-self.initial_raw[n]) <= MAX_ANCHOR_DIFFERENCE_RAD,
                  f'ID{n+1} capture/origin mismatch or encoder branch changed')
            _need(abs((r-self.initial_raw[n])-self.signs[n]*(q-self.initial_model[n])) < 1e-8,
                  f'ID{n+1} raw/model origin disagree')
        # Check every shifted target, not just the original mathematical path.
        for delta in self.deltas:
            for n, d in enumerate(delta):
                target = model[n]+d
                lo, hi = _MODEL_LIMITS[str(n+1)]
                _need(max(lo, self.lower[n]) <= target <= min(hi, self.upper[n]),
                      f'ID{n+1} fresh-anchored target outside reviewed bounds')
                _need(-12.57+math.radians(5) <= raw[n]+self.signs[n]*d <= 12.57-math.radians(5),
                      f'ID{n+1} fresh-anchored raw target outside range')
                _need(self.kp[n]*abs(d) <= self.pd_limits[n],
                      f'ID{n+1} stationary PD estimate exceeds reviewed limit')
        return tuple(q-q0 for q, q0 in zip(model, self.initial_model))

    def bind(self, model, raw):
        differences = self.check_origin(model, raw)
        return BoundPreloadPath(self, tuple(model), tuple(raw), differences)


@dataclass(frozen=True)
class BoundPreloadPath:
    """Strictly ordered slot cursor; the caller must also enforce real deadlines.

    The return flag describes generated targets, not measured return or STOP.
    The bound origin and cursor cannot be reassigned to replay a used path.
    """
    path: PreloadPath
    initial_model: tuple
    initial_raw: tuple
    anchor_difference_rad: tuple
    _next_slot: int = field(default=0, init=False, repr=False)
    _return_complete: bool = field(default=False, init=False, repr=False)

    @property
    def next_slot(self):
        return self._next_slot

    @property
    def return_complete(self):
        return self._return_complete

    def target_for_slot(self, slot):
        _need(type(slot) is int and slot == self.next_slot and 0 <= slot < SAMPLE_COUNT,
              'path slot skipped, duplicated, reversed or exhausted')
        delta = self.path.deltas[slot]
        object.__setattr__(self, '_next_slot', self.next_slot+1)
        object.__setattr__(self, '_return_complete', slot >= round(RETURN_COMPLETE_S/PERIOD_S))
        return tuple(q+d for q, d in zip(self.initial_model, delta))


def validate_path(data, profile):
    """Return immutable numerical path, never a hardware authorization token."""
    _need(type(data) is dict and data.get('schema') ==
          'singularitydog.supported-preload-path-file-only.v1', 'unsupported path schema')
    _need(all(data.get(k) is False for k in ('motor_output_allowed', 'approved_for_runtime',
          'learned_model_output', 'box_removal_allowed')), 'candidate provenance must remain file-only')
    _need(_finite(data.get('duration_s')) and _finite(data.get('period_s')) and
          data['duration_s'] == 5. and data['period_s'] == PERIOD_S,
          'five-second path at 20ms required')
    screen = data.get('source_screen')
    _need(type(screen) is dict and screen.get('screen_failures') == [] and
          screen.get('motor_output_allowed') is False, 'source numerical screen failed')
    _need(_finite(screen.get('rise_mm')) and 0 < screen['rise_mm'] <= .25,
          'rise exceeds 0.25mm scope')
    initial = _by_id(screen.get('initial_model_rad_by_id'), 'capture model')
    raw_start = _by_id(screen.get('initial_raw_rad_by_id'), 'capture raw')
    _need(type(profile) is dict, 'profile must be a dictionary')
    axes = profile.get('axes')
    _need(type(axes) is dict and set(axes) == set(IDS), 'profile axes differ')
    for mid in IDS:
        axis = axes[mid]
        _need(type(axis) is dict, f'ID{mid} profile axis must be a dictionary')
        fields = ('offset_rad', 'lower_rad', 'upper_rad', 'kp',
                  'max_estimated_pd_torque_nm', 'max_displacement_from_start_rad',
                  'max_command_velocity_rad_s', 'max_command_acceleration_rad_s2')
        _need(all(_finite(axis.get(name)) for name in fields),
              f'ID{mid} profile needs finite numerical limits')
        _need(axis['lower_rad'] < axis['upper_rad'], f'ID{mid} invalid model bounds')
        _need(0 <= axis['kp'] <= 6 and 0 < axis['max_estimated_pd_torque_nm'] <= .2,
              f'ID{mid} invalid preload limits')
        for name, maximum in (('max_displacement_from_start_rad', math.radians(1)),
                              ('max_command_velocity_rad_s', math.radians(1)),
                              ('max_command_acceleration_rad_s2', math.radians(5))):
            _need(0 < axis[name] <= maximum, f'ID{mid} invalid {name}')
    signs = tuple(axes[mid].get('sign') for mid in IDS)
    _need(all(type(s) is int and s in (-1, 1) for s in signs), 'invalid signs')
    for n, mid in enumerate(IDS):
        offset = axes[mid]['offset_rad']
        _need(_finite(offset), 'invalid calibration offset')
        turns = (raw_start[n]-signs[n]*(initial[n]-offset))/(2*math.pi)
        _need(_finite(turns) and abs(turns-round(turns)) <= 1e-8,
              f'ID{mid} calibration/path mismatch')
    samples = data.get('samples')
    _need(type(samples) is list and len(samples) == SAMPLE_COUNT, '251 complete samples required')
    deltas = []
    previous = previous_speed = None
    for slot, row in enumerate(samples):
        t = slot*PERIOD_S
        _need(type(row) is dict and _finite(row.get('time_s')) and
              abs(row['time_s']-t) < 1e-10, 'invalid sample timestamp')
        _need(_finite(row.get('rise_fraction')) and abs(row['rise_fraction']-_fraction(t)) < 1e-10,
              'invalid finite extension/return schedule')
        model = _by_id(row.get('q_model_rad_by_id'), 'target model')
        raw = _by_id(row.get('q_raw_rad_by_id'), 'target raw')
        delta = tuple(q-q0 for q, q0 in zip(model, initial))
        speed = None if previous is None else tuple((q-p)/PERIOD_S for q, p in zip(model, previous))
        for n, mid in enumerate(IDS):
            a = axes[mid]
            _need(abs(raw[n]-(raw_start[n]+signs[n]*delta[n])) < 1e-10,
                  f'ID{mid} path changed encoder branch')
            _need(abs(delta[n]) <= min(math.radians(1), a['max_displacement_from_start_rad'])+1e-10,
                  f'ID{mid} displacement limit')
            if _fraction(t) == 0:
                _need(abs(delta[n]) < 1e-10, 'start/end holds must remain at origin')
            if speed is not None:
                _need(abs(speed[n]) <= min(math.radians(1), a['max_command_velocity_rad_s'])+1e-10,
                      f'ID{mid} speed limit')
                if previous_speed is not None:
                    _need(abs(speed[n]-previous_speed[n])/PERIOD_S <=
                          min(math.radians(5), a['max_command_acceleration_rad_s2'])+1e-10,
                          f'ID{mid} acceleration limit')
        deltas.append(delta)
        previous, previous_speed = model, speed
    result = PreloadPath(initial, raw_start, tuple(deltas), signs,
                         tuple(axes[i]['lower_rad'] for i in IDS),
                         tuple(axes[i]['upper_rad'] for i in IDS),
                         tuple(axes[i]['kp'] for i in IDS),
                         tuple(axes[i]['max_estimated_pd_torque_nm'] for i in IDS))
    _need(all(_finite(v) for values in (result.lower, result.upper, result.kp, result.pd_limits)
              for v in values) and all(0 <= v <= 6 for v in result.kp) and
          all(0 < v <= .2 for v in result.pd_limits), 'invalid preload limits')
    result.check_origin(initial, raw_start)
    return result
