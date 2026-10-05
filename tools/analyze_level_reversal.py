"""File-only spirit-level reversal analysis; never a robot calibration approval.

Mark one end of the level. Every signed bubble reading is positive TOWARD THAT
MARKED END, including after an end-for-end 180-degree turn on the same measuring
face and footprint. Do not turn the level upside down. Under a locally linear
model f=b+t, r=b-t: (f+r)/2 indicates zero offset and (f-r)/2 indicates surface
tilt, in reading units. These contrasts do not separate unknown seating errors.
The return f2-f describes observed return variation, not a future error bound.
With the world axis fixed toward the initially marked end, readings are F=f,
R=-r: world surface indication is (F+R)/2; zero indication is (F-R)/2.
Camera-left or observer-left is not either of these conventions.

Use --template for an unmeasured A/B/A record. Null range, angular sensitivity,
seating bounds or geometry remain unknown. A repeatable bubble is not evidence
of a bounded joint datum, squareness, or absolute robot origin. Known geometry
requires measured horizontal run/rise intervals and an additional alignment
bound; nominal dimensions and a photograph alone do not supply those bounds.

Manufacturer procedures verified 2026-10-05 (no model specification assumed):
https://www.stabila.com/files/default/pdf/en/FAQ_Level-check_EN.pdf
https://www.wylerag.com/en/support/faq/
https://www.starrett.com/docs/default-source/faq%27s/faq-98-series-levels.pdf?sfvrsn=f8951b8f_10
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import stat

SCHEMA = 'singularitydog.level-reversal-records.v1'
RESULT_SCHEMA = 'singularitydog.level-reversal-analysis.v1'
MAX_BYTES, MAX_CYCLES = 1024 * 1024, 64
FLAGS = dict.fromkeys(('hardware_opened', 'motor_output_allowed', 'approved_for_runtime',
    'calibration_approved', 'profile_changed', 'existing_thresholds_changed',
    'physical_stationarity_proven', 'joint_datum_verified', 'uncertainty_inferred_from_repeatability'), False)
CONDITIONS = ('same_footprint', 'same_measuring_face', 'surface_unchanged',
              'settled_each_reading', 'acclimatized')
REFERENCES = [
    'https://www.stabila.com/files/default/pdf/en/FAQ_Level-check_EN.pdf',
    'https://www.wylerag.com/en/support/faq/',
    "https://www.starrett.com/docs/default-source/faq%27s/faq-98-series-levels.pdf?sfvrsn=f8951b8f_10",
]


def need(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names):
    need(type(value) is dict and set(value) == set(names), 'Unknown/missing fields')


def label(value):
    need(type(value) is str and value.strip() == value and 0 < len(value) <= 256,
         'A nonempty bounded label is required')
    return value


def number(value):
    need(type(value) in (int, float) and math.isfinite(value) and abs(value) < 1e9,
         'Expected bounded finite number, never boolean')
    return value


def interval(value):
    if value is None:
        return None
    need(type(value) is list and len(value) == 2, 'Interval must be [lower, upper] or null')
    lo, hi = map(number, value)
    need(lo <= hi, 'Reversed interval')
    return [lo, hi]


def reference(value, *, positive=False):
    if value is None:
        return None
    fields(value, ('interval', 'evidence'))
    result = interval(value['interval']); need(result is not None, 'Use null for unknown reference')
    label(value['evidence'])
    need(result[0] < result[1], 'Measured reference must have a nonzero stated uncertainty range')
    if positive:
        need(result[0] > 0, 'Angular sensitivity must be positive throughout its interval')
    return result


def error_bound(value):
    if value is None:
        return None
    fields(value, ('upper', 'evidence')); upper = number(value['upper']); label(value['evidence'])
    need(upper > 0, 'Unknown error must be null; a measured bound must be positive')
    return upper


def combine(a, b, sign=1, factor=1):
    if a is None or b is None:
        return None
    out = [(a[0]+b[0])*factor, (a[1]+b[1])*factor] if sign == 1 else [
        (a[0]-b[1])*factor, (a[1]-b[0])*factor]
    for v in out: number(v)
    return out


def widen(value, bound):
    return None if value is None or bound is None else [value[0]-bound, value[1]+bound]


def multiply(a, b):
    if a is None or b is None:
        return None
    corners = [x*y for x in a for y in b]
    for v in corners: number(v)
    return [min(corners), max(corners)]


def geometry(value):
    fields(value, ('id', 'rise_mm', 'horizontal_run_mm', 'alignment_error_bound_rad'))
    label(value['id']); rise = reference(value['rise_mm']); run = reference(value['horizontal_run_mm'])
    extra = error_bound(value['alignment_error_bound_rad'])
    if run is not None: need(run[0] > 0, 'Horizontal run must be strictly positive')
    nominal = None
    if rise is not None and run is not None:
        angles = [math.atan2(h, length) for h in rise for length in run]
        nominal = [min(angles), max(angles)]
    return {'id': value['id'], 'angle_from_length_intervals_rad': nominal,
        'angle_with_supplied_alignment_bound_rad': widen(nominal, extra),
        'alignment_bound_unknown': extra is None,
        'scope': 'Conditional measured geometry only; no sensitivity fit or joint-datum transfer.'}


def analyze(value):
    fields(value, ('schema', 'record_kind', 'instrument_id', 'reading_unit', 'coordinate_convention',
        'linear_reading_range', 'angular_sensitivity_rad_per_unit', 'per_reading_seating_bound_units',
        'cycles', 'geometry_references'))
    need(value['schema'] == SCHEMA, 'Invalid input schema'); label(value['instrument_id'])
    need(value['record_kind'] in ('PLANNED_UNMEASURED', 'SYNTHETIC_FIXTURE', 'RECORDED_READINGS_UNREVIEWED'),
         'Declare planned, synthetic or unreviewed recorded readings')
    need(value['reading_unit'] in ('vial_divisions', 'marked_arbitrary_units'), 'Unsupported bubble units')
    need(value['coordinate_convention'] == 'toward_marked_level_end_positive', 'Ambiguous reversal sign convention')
    linear_range = reference(value['linear_reading_range'])
    sensitivity = reference(value['angular_sensitivity_rad_per_unit'], positive=True)
    seating = error_bound(value['per_reading_seating_bound_units'])
    planned = value['record_kind'] == 'PLANNED_UNMEASURED'
    if planned:
        need(linear_range is None and sensitivity is None and seating is None,
             'Unmeasured plan must retain unknown references')
    need(type(value['cycles']) is list and 1 <= len(value['cycles']) <= MAX_CYCLES, 'Bounded A/B/A cycles required')
    outputs, ids = [], set()
    for cycle in value['cycles']:
        fields(cycle, ('id', 'datum_id', 'contact_patch_id', 'measuring_face_id', 'conditions', 'readings'))
        for key in ('id', 'datum_id', 'contact_patch_id', 'measuring_face_id'): label(cycle[key])
        need(cycle['id'] not in ids, 'Duplicate cycle'); ids.add(cycle['id'])
        fields(cycle['conditions'], CONDITIONS)
        need(all(v is None or type(v) is bool for v in cycle['conditions'].values()), 'Conditions must be true, false or null')
        reads = cycle['readings']; need(type(reads) is list and len(reads) == 3, 'Require forward/reverse/forward A/B/A')
        readings = []
        for row, orientation in zip(reads, ('forward', 'reverse', 'forward')):
            fields(row, ('orientation', 'displacement_interval'))
            need(row['orientation'] == orientation, 'Wrong A/B/A orientation order')
            readings.append(interval(row['displacement_interval']))
        if planned:
            need(all(r is None for r in readings) and all(v is None for v in cycle['conditions'].values()),
                 'Unmeasured plan must not contain readings or physical confirmations')
        in_range = linear_range is not None and all(r is not None and linear_range[0] <= r[0] <= r[1] <= linear_range[1] for r in readings)
        conditions_known = all(cycle['conditions'][k] is True for k in CONDITIONS)
        f, r, f2 = readings
        bias = combine(f, r, factor=.5); tilt = combine(f, r, sign=-1, factor=.5)
        returned_bias = combine(f2, r, factor=.5); returned_tilt = combine(f2, r, sign=-1, factor=.5)
        bounded = in_range and conditions_known and seating is not None
        zero = widen(bias, seating) if bounded else None
        surface = widen(tilt, seating) if bounded else None
        outputs.append({'id': cycle['id'], 'datum_id': cycle['datum_id'], 'contact_patch_id': cycle['contact_patch_id'],
            'measuring_face_id': cycle['measuring_face_id'], 'conditions': cycle['conditions'],
            'reading_intervals': readings,
            'conditional_linear_model_contrasts_units': {'zero_indication': bias, 'surface_indication': tilt,
                'return_zero_indication': returned_bias, 'return_surface_indication': returned_tilt},
            'observed_forward_return_difference_units': combine(f2, f, sign=-1),
            'readings_within_supplied_linear_range': in_range,
            'conditions_attested': conditions_known, 'seating_bound_unknown': seating is None,
            'zero_bias_with_supplied_seating_bound_units': zero,
            'surface_tilt_with_supplied_seating_bound_units': surface,
            'zero_bias_with_supplied_bounds_rad': multiply(zero, sensitivity),
            'surface_tilt_with_supplied_bounds_rad': multiply(surface, sensitivity),
            'angular_sensitivity_unknown': sensitivity is None,
            'scope': 'Input-conditioned intervals, not independently verified accuracy. Reversal bias remains confounded with unbounded contact errors.'})
    refs = value['geometry_references']; need(type(refs) is list and len(refs) <= MAX_CYCLES, 'Geometry list exceeds bound')
    if planned: need(not refs, 'Unmeasured plan has no measured geometry')
    geometries = [geometry(g) for g in refs]
    need(len({g['id'] for g in geometries}) == len(geometries), 'Duplicate geometry reference')
    return {'schema': RESULT_SCHEMA, 'status': 'DESCRIPTIVE_LEVEL_REVERSAL_REVIEW_REQUIRED', **FLAGS,
        'record_kind': value['record_kind'], 'synthetic_fixture_result': value['record_kind'] == 'SYNTHETIC_FIXTURE',
        'instrument_id': value['instrument_id'], 'reading_unit': value['reading_unit'],
        'coordinate_convention': value['coordinate_convention'], 'cycles': outputs,
        'supplied_reference_bounds': {
            'linear_reading_range': value['linear_reading_range'],
            'angular_sensitivity_rad_per_unit': value['angular_sensitivity_rad_per_unit'],
            'per_reading_seating_bound_units': value['per_reading_seating_bound_units']},
        'geometry_references': geometries, 'absolute_origin_error_rad': None,
        'absolute_origin_uncertainty_rad': None, 'joint_reference_angle_rad': None,
        'fit_observations_generated': False,
        'manufacturer_procedure_references': REFERENCES,
        'model_assumption': 'Instrument-axis readings f=b+t, r=b-t, f2=b+t; unknown seating/range/drift may invalidate separation.',
        'world_axis_conversion': 'World positive is toward the marked end before reversal: F=f, R=-r. Surface=(F+R)/2; zero=(F-R)/2.',
        'limitations': [
            'Visual centering or repeatability alone supplies no angular sensitivity or justified total error bound.',
            'Observed A/B/A difference is a sample variation, not an upper bound on future repeatability or thermal drift.',
            'A stable common datum/contact bias and link-axis alignment cannot be separated by reversing only the level.',
            'Horizontal and vertical vials or different measuring faces need separate characterization; horizontal reversal does not establish squareness.',
            'Geometry intervals use supplied measured horizontal run and rise, not nominal dimensions, slope length, or invented ruler precision.',
            'Sensitivity conversion requires its own range/nonlinearity evidence. Geometry references do not automatically calibrate the vial.',
            'No prior robot pose receives a retroactive external measurement; this tool creates no fit-compatible reference or profile.']}


def template():
    return {'schema': SCHEMA, 'record_kind': 'PLANNED_UNMEASURED', 'instrument_id': 'UNKNOWN', 'reading_unit': 'vial_divisions',
        'coordinate_convention': 'toward_marked_level_end_positive', 'linear_reading_range': None,
        'angular_sensitivity_rad_per_unit': None, 'per_reading_seating_bound_units': None,
        'cycles': [{'id': 'unmeasured-cycle', 'datum_id': 'UNKNOWN', 'contact_patch_id': 'UNKNOWN',
            'measuring_face_id': 'UNKNOWN', 'conditions': dict.fromkeys(CONDITIONS),
            'readings': [{'orientation': o, 'displacement_interval': None} for o in ('forward', 'reverse', 'forward')]}],
        'geometry_references': []}


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items: need(key not in result, 'Duplicate JSON key'); result[key] = value
        return result
    def finite(value):
        n = float(value); need(math.isfinite(n), 'Nonfinite JSON value'); return n
    return json.loads(raw, object_pairs_hook=pairs, parse_float=finite,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON value')))


def read_file(path):
    path = Path(path).expanduser().absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Symlink input refused')
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= MAX_BYTES, 'Expected bounded regular JSON input')
        raw = handle.read(MAX_BYTES + 1); named = path.lstat()
        need((info.st_dev, info.st_ino, info.st_size) == (named.st_dev, named.st_ino, named.st_size)
             and len(raw) == info.st_size, 'Input binding changed')
    return raw, {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'byte_count': len(raw)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--template', action='store_true'); mode.add_argument('--input')
    args = parser.parse_args(argv)
    if args.template:
        print(json.dumps(template(), indent=2, allow_nan=False)); return 0
    try:
        raw, binding = read_file(args.input); result = analyze(strict_json(raw))
        result['input_binding'] = binding
        result['analysis_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except (OSError, ValueError, TypeError, OverflowError) as error:
        print(json.dumps({'schema': RESULT_SCHEMA, 'status': 'INVALID_LEVEL_RECORDS', **FLAGS,
            'absolute_origin_error_rad': None, 'absolute_origin_uncertainty_rad': None,
            'error': str(error)}, allow_nan=False)); return 1
    print(json.dumps(result, indent=2, allow_nan=False)); return 0


if __name__ == '__main__':
    raise SystemExit(main())
