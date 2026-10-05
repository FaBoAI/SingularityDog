"""File-only review of manually identified optical joint markers; no camera/CAN.

Use --template to describe a planned acquisition, then record every requested
frame, including MISSING frames. Point order is fixed by marker IDs. Image axes
are x right, y down; angles are counterclockwise positive after y inversion.
For each CAPTURED frame, stator_points and output_points are ordered lists:
[{"xy_px": [x0,y0], "max_error_px": null},
 {"xy_px": [x1,y1], "max_error_px": {"upper": radius, "evidence": description}}].
An occluded pair is null with a missing_reason. An unknown timestamp is null
with a null timestamp_max_error_s. Frame/image IDs bind observations but do not
make visual identity, acquisition times or supplied bounds independently true.
Both stator and output directions are measured in every image. Their difference
cancels a common in-plane camera rotation, but perspective, axis misalignment,
distortion, rolling shutter, marker flex and exposure motion are not cancelled.

Each point's max_error_px is a supplied maximum Euclidean localization error,
with evidence, or null. It must include all localization errors, not a standard
deviation or repeated-click scatter. For measured vector length L and endpoint
error radii r0,r1, E=r0+r1<L gives direction error <=asin(E/L). Relative error
is the sum of the two direction bounds. These are conditional projected-image
bounds, not verified physical accuracy. Short/uncertain vectors retain unknown
bounds. No OpenCV result, camera specification or nominal marker dimension is
assumed to be a ground truth.

Only if all supplied geometry/rigidity conditions are true, an image-to-joint
sign with evidence and a separate per-frame projection error bound are supplied
do changes receive a conditional
joint-angle circular interval. Unknown marker mounting offset still prevents
absolute origin measurement. Circular differences never establish turn counts.
The projection bound is for each frame's entire signed relative angle after a
constant unknown mounting offset, excluding the separately bounded pixel errors.
It is not a per-marker bound. A two-frame change adds twice this supplied bound.
Rates assume no extra turns and remain descriptions, not velocity corrections.
Frame spacing/nominal Nyquist values cannot establish motion bandwidth or
exclude aliasing. Input image hashes and physical attestations are recorded,
never verified by this tool. Stdout JSON only; no original files are changed.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import stat

SCHEMA = 'singularitydog.optical-joint-motion-records.v1'
RESULT_SCHEMA = 'singularitydog.optical-joint-motion-analysis.v1'
MAX_BYTES, MAX_FRAMES = 8 * 1024 * 1024, 5000
MIN_VECTOR_LENGTH_PX = 1.0  # Degenerate image geometry, not a robot safety gate.
CONDITIONS = ('same_camera_viewpoint', 'joint_axis_parallel_optical_axis',
    'marker_planes_normal_to_joint_axis', 'isotropic_image_metric',
    'lens_distortion_bounded', 'rolling_shutter_effect_bounded')
FLAGS = dict.fromkeys(('hardware_opened', 'camera_opened', 'motor_output_allowed',
    'approved_for_runtime', 'calibration_approved', 'physical_groundtruth_established',
    'physical_stationarity_proven', 'velocity_cause_confirmed', 'alias_excluded',
    'existing_thresholds_changed', 'velocity_correction_applied',
    'mechanical_zero_installation_approved', 'image_hashes_verified',
    'marker_identity_verified', 'projection_model_verified',
    'uncertainty_bounds_verified', 'external_clock_alignment_verified'), False)


def need(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names):
    need(type(value) is dict and set(value) == set(names), 'Unknown/missing fields')


def label(value):
    need(type(value) is str and value.strip() == value and 0 < len(value) <= 256,
         'Expected bounded nonempty label')
    return value


def number(value, *, positive=False):
    need(type(value) in (int, float) and math.isfinite(value) and abs(value) < 1e12,
         'Expected bounded finite number, never boolean')
    need(not positive or value > 0, 'Expected positive number')
    return value


def integer(value, lower, upper):
    need(type(value) is int and lower <= value <= upper, 'Invalid integer')
    return value


def condition(value):
    need(value is None or type(value) is bool, 'Condition must be true, false or null')
    return value


def bound(value):
    if value is None:
        return None
    fields(value, ('upper', 'evidence'))
    label(value['evidence'])
    return number(value['upper'], positive=True)


def measured_interval(value, *, positive=False):
    if value is None:
        return None
    fields(value, ('interval', 'evidence')); label(value['evidence'])
    a = value['interval']
    need(type(a) is list and len(a) == 2, 'Expected [lower, upper]')
    lo, hi = (number(v) for v in a)
    need(lo < hi and (not positive or lo > 0), 'Invalid measured interval')
    return [lo, hi]


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def circular_interval(center, halfwidth):
    if halfwidth is None:
        return None
    center = wrap(number(center)); number(halfwidth)
    need(halfwidth >= 0, 'Negative angular uncertainty')
    if halfwidth >= math.pi:
        return {'center_rad': center, 'halfwidth_rad': math.pi, 'full_circle': True,
                'arcs_rad': [[-math.pi, math.pi]]}
    lo, hi = center - halfwidth, center + halfwidth
    arcs = [[lo, hi]]
    if lo < -math.pi:
        arcs = [[-math.pi, hi], [lo + 2 * math.pi, math.pi]]
    elif hi > math.pi:
        arcs = [[-math.pi, hi - 2 * math.pi], [lo, math.pi]]
    return {'center_rad': center, 'halfwidth_rad': halfwidth, 'full_circle': False,
            'arcs_rad': arcs}


def points(value, dimensions):
    if value is None:
        return {'angle_rad': None, 'length_px': None, 'error_bound_rad': None,
                'state': 'MARKER_NOT_OBSERVED'}
    need(type(value) is list and len(value) == 2, 'Each rigid body needs two ordered points')
    coords, errors = [], []
    for point in value:
        fields(point, ('xy_px', 'max_error_px'))
        xy = point['xy_px']; need(type(xy) is list and len(xy) == 2, 'Expected pixel [x,y]')
        x, y = (number(v) for v in xy)
        need(0 <= x < dimensions[0] and 0 <= y < dimensions[1], 'Point outside supplied frame dimensions')
        coords.append((x, y)); errors.append(bound(point['max_error_px']))
    dx, dy = coords[1][0] - coords[0][0], coords[1][1] - coords[0][1]
    length = number(math.hypot(dx, dy))
    if length <= MIN_VECTOR_LENGTH_PX:
        return {'angle_rad': None, 'length_px': length, 'error_bound_rad': None,
                'state': 'DEGENERATE_MARKER_VECTOR'}
    error = None
    if all(e is not None for e in errors):
        total = number(sum(errors))
        if total < length:
            error = math.asin(total / length)
    return {'angle_rad': math.atan2(-dy, dx), 'length_px': length,
            'error_bound_rad': error,
            'state': 'BOUNDED_PROJECTED_DIRECTION' if error is not None else 'PROJECTED_DIRECTION_BOUND_UNKNOWN'}


def metadata(value, planned):
    fields(value['camera'], ('id', 'view_id', 'lens_id', 'frame_dimensions_px',
        'timestamp_clock', 'timestamp_definition', 'nominal_frame_interval_s',
        'measured_acquisition_bandwidth_hz'))
    camera = value['camera']
    for k in ('id', 'view_id', 'lens_id', 'timestamp_clock'): label(camera[k])
    need(camera['timestamp_definition'] in ('exposure_midpoint', 'frame_start',
        'frame_end', 'host_receipt', 'unknown'), 'Unknown timestamp definition')
    dimensions = camera['frame_dimensions_px']
    if dimensions is not None:
        need(type(dimensions) is list and len(dimensions) == 2, 'Expected frame dimensions')
        for n in dimensions: integer(n, 2, 65536)
    if camera['nominal_frame_interval_s'] is not None:
        number(camera['nominal_frame_interval_s'], positive=True)
    measured_interval(camera['measured_acquisition_bandwidth_hz'], positive=True)
    fields(value['markers'], ('stator', 'output'))
    all_ids = []
    for marker in value['markers'].values():
        fields(marker, ('rigid_body_id', 'point_ids', 'rigid_attachment_confirmed'))
        label(marker['rigid_body_id']); condition(marker['rigid_attachment_confirmed'])
        ids = marker['point_ids']; need(type(ids) is list and len(ids) == 2, 'Two ordered marker IDs required')
        all_ids.extend(label(i) for i in ids)
    need(len(set(all_ids)) == 4, 'All four marker point IDs must be distinct')
    need(value['markers']['stator']['rigid_body_id'] != value['markers']['output']['rigid_body_id'],
         'Stator and output are distinct rigid bodies')
    fields(value['geometry_conditions'], CONDITIONS)
    for v in value['geometry_conditions'].values(): condition(v)
    projection = bound(value['per_frame_projection_error_bound_rad'])
    sign = value['image_to_joint_sign']
    if sign is not None:
        fields(sign, ('value', 'evidence')); label(sign['evidence'])
        need(type(sign['value']) is int and sign['value'] in (-1, 1), 'Image-to-joint sign must be +1 or -1')
    alignment = value['external_clock_alignment']
    if alignment is not None:
        fields(alignment, ('reference_clock', 'offset_interval_s', 'evidence'))
        label(alignment['reference_clock']); label(alignment['evidence'])
        interval = alignment['offset_interval_s']
        need(type(interval) is list and len(interval) == 2, 'Clock offset interval required')
        lo, hi = (number(v) for v in interval); need(lo < hi, 'Nonzero clock alignment uncertainty required')
    f = value['highest_motion_frequency_of_interest_hz']
    if f is not None: number(f, positive=True)
    if planned:
        need(dimensions is None and camera['timestamp_definition'] == 'unknown' and
             camera['nominal_frame_interval_s'] is None and camera['measured_acquisition_bandwidth_hz'] is None and
             projection is None and sign is None and alignment is None and f is None and
             all(v is None for v in value['geometry_conditions'].values()) and
             all(m['rigid_attachment_confirmed'] is None for m in value['markers'].values()),
             'Unmeasured plan must retain unknown measurements and attestations')
    return dimensions, projection


def analyze(value):
    fields(value, ('schema', 'record_kind', 'joint_id', 'image_axes', 'camera', 'markers',
        'geometry_conditions', 'per_frame_projection_error_bound_rad', 'image_to_joint_sign', 'external_clock_alignment',
        'highest_motion_frequency_of_interest_hz', 'requested_frame_count', 'frames'))
    need(value['schema'] == SCHEMA, 'Invalid input schema')
    need(value['record_kind'] in ('PLANNED_UNMEASURED', 'SYNTHETIC_FIXTURE', 'RECORDED_OBSERVATIONS_UNREVIEWED'),
         'Declare planned, synthetic or unreviewed recorded observations')
    integer(value['joint_id'], 1, 127)
    need(value['image_axes'] == 'x_right_y_down_ccw_angle_positive', 'Ambiguous image angle convention')
    planned = value['record_kind'] == 'PLANNED_UNMEASURED'
    dimensions, projection = metadata(value, planned)
    count = integer(value['requested_frame_count'], 1, MAX_FRAMES)
    frames = value['frames']
    need(type(frames) is list and len(frames) == count, 'Include every requested frame, including missing frames')
    projected, outputs, seen_ids, seen_hashes = [], [], set(), set()
    repeated_image_hashes = 0
    captured = missing = unmeasured = 0
    last_time = None
    physical_conditions = (all(v is True for v in value['geometry_conditions'].values()) and
        all(m['rigid_attachment_confirmed'] is True for m in value['markers'].values()) and
        projection is not None and value['image_to_joint_sign'] is not None)
    for index, row in enumerate(frames):
        fields(row, ('index', 'state', 'frame_id', 'image_sha256', 'timestamp_s',
            'timestamp_max_error_s', 'exposure_s', 'missing_reason', 'stator_points', 'output_points'))
        integer(row['index'], 0, count - 1); need(row['index'] == index, 'Frame order/index mismatch')
        need(row['state'] in ('CAPTURED', 'MISSING', 'UNMEASURED'), 'Invalid frame state')
        out = {'index': index, 'state': row['state'], 'frame_id': row['frame_id'],
               'image_sha256': row['image_sha256'], 'timestamp_s': row['timestamp_s'],
               'timestamp_max_error_s': row['timestamp_max_error_s'], 'exposure_s': row['exposure_s'],
               'missing_reason': row['missing_reason'], 'projected_relative_angle_rad': None,
               'projected_relative_angle_interval_rad': None}
        if row['state'] != 'CAPTURED':
            need(all(row[k] is None for k in ('frame_id', 'image_sha256', 'timestamp_s',
                'timestamp_max_error_s', 'exposure_s', 'stator_points', 'output_points')),
                'Missing/unmeasured frames cannot contain acquired data')
            if row['state'] == 'MISSING':
                need(not planned, 'Plan must use UNMEASURED'); label(row['missing_reason']); missing += 1
            else:
                need(planned and row['missing_reason'] is None, 'UNMEASURED is only for plans'); unmeasured += 1
            outputs.append(out); continue
        need(not planned and dimensions is not None, 'Captured frames need dimensions and cannot be planned')
        marker_missing = row['stator_points'] is None or row['output_points'] is None
        if marker_missing:
            label(row['missing_reason'])
        else:
            need(row['missing_reason'] is None, 'Observed markers cannot have a missing reason')
        label(row['frame_id']); need(row['frame_id'] not in seen_ids, 'Duplicate frame ID'); seen_ids.add(row['frame_id'])
        sha = row['image_sha256']
        need(type(sha) is str and len(sha) == 64 and all(c in '0123456789abcdef' for c in sha), 'Image SHA256 required')
        # Identical bytes may be a held/reused image or truly identical static
        # exposures. Record this ambiguity; do not certify either explanation.
        repeated_image_hashes += sha in seen_hashes; seen_hashes.add(sha)
        t = row['timestamp_s']
        if t is not None:
            number(t); need(t >= 0 and (last_time is None or t > last_time), 'Known captured timestamps must increase')
            last_time = t
        else:
            need(row['timestamp_max_error_s'] is None, 'Unknown timestamp cannot have an error bound')
        time_error = bound(row['timestamp_max_error_s'])
        if row['exposure_s'] is not None: number(row['exposure_s'], positive=True)
        s, o = points(row['stator_points'], dimensions), points(row['output_points'], dimensions)
        captured += 1; out['stator'] = s; out['output'] = o
        if s['angle_rad'] is not None and o['angle_rad'] is not None:
            angle = wrap(o['angle_rad'] - s['angle_rad'])
            error = None if s['error_bound_rad'] is None or o['error_bound_rad'] is None else s['error_bound_rad'] + o['error_bound_rad']
            out['projected_relative_angle_rad'] = angle
            out['projected_relative_angle_interval_rad'] = circular_interval(angle, error)
            projected.append({'index': index, 'time': t, 'time_error': time_error, 'angle': angle, 'error': error})
        outputs.append(out)
    changes, gaps, captured_gaps = [], [], []
    acquired = [r for r in outputs if r['state'] == 'CAPTURED']
    for a, b in zip(acquired, acquired[1:]):
        if a['timestamp_s'] is not None and b['timestamp_s'] is not None:
            captured_gaps.append(number(b['timestamp_s'] - a['timestamp_s'], positive=True))
    for a, b in zip(projected, projected[1:]):
        dt = None
        if a['time'] is not None and b['time'] is not None:
            dt = number(b['time'] - a['time'], positive=True); gaps.append(dt)
        delta = wrap(b['angle'] - a['angle'])
        e = None if a['error'] is None or b['error'] is None else a['error'] + b['error']
        dt_interval = None
        if dt is not None and a['time_error'] is not None and b['time_error'] is not None:
            te = a['time_error'] + b['time_error']; dt_interval = [dt - te, dt + te]
        rate_interval = None
        # A cut-crossing displacement is ambiguous as a signed principal rate.
        if e is not None and abs(delta) + e < math.pi and dt_interval is not None and dt_interval[0] > 0:
            corners = [(delta + sign * e) / duration for sign in (-1, 1) for duration in dt_interval]
            rate_interval = [number(min(corners)), number(max(corners))]
        physical_error = None if e is None or not physical_conditions else e + 2 * projection
        physical_delta = delta if not physical_conditions else delta * value['image_to_joint_sign']['value']
        changes.append({'from_index': a['index'], 'to_index': b['index'],
            'unusable_requested_frames_between': b['index'] - a['index'] - 1,
            'elapsed_timestamp_s': dt, 'elapsed_timestamp_interval_s': dt_interval,
            'projected_principal_displacement_rad': delta,
            'projected_displacement_circular_interval_rad': circular_interval(delta, e),
            'conditional_joint_displacement_circular_interval_rad': circular_interval(physical_delta, physical_error),
            'conditional_no_extra_turn_projected_rate_rad_per_s': None if dt is None else number(delta / dt),
            'conditional_no_extra_turn_projected_rate_interval_rad_per_s': rate_interval,
            'extra_turns_excluded': False, 'missing_frames_interpolated': False})
    bounded = sum(r['projected_relative_angle_interval_rad'] is not None for r in outputs)
    all_times_known = bool(acquired) and all(r['timestamp_s'] is not None for r in acquired)
    span = None if len(acquired) < 2 or not all_times_known else acquired[-1]['timestamp_s'] - acquired[0]['timestamp_s']
    nominal = value['camera']['nominal_frame_interval_s']
    interest = value['highest_motion_frequency_of_interest_hz']
    return {'schema': RESULT_SCHEMA, 'status': 'DESCRIPTIVE_OPTICAL_REVIEW_REQUIRED', **FLAGS,
        'record_kind': value['record_kind'], 'joint_id': value['joint_id'],
        'synthetic_fixture_result': value['record_kind'] == 'SYNTHETIC_FIXTURE',
        'supplied_metadata': {k: value[k] for k in ('camera', 'markers', 'image_axes', 'geometry_conditions',
            'per_frame_projection_error_bound_rad', 'image_to_joint_sign', 'external_clock_alignment', 'highest_motion_frequency_of_interest_hz')},
        'coverage': {'requested_frames': count, 'captured_frames': captured, 'missing_frames': missing,
            'unmeasured_frames': unmeasured, 'usable_projected_angles': len(projected),
            'unusable_captured_frames': captured - len(projected),
            'degenerate_captured_frames': sum(any(r.get(k, {}).get('state') == 'DEGENERATE_MARKER_VECTOR' for k in ('stator', 'output')) for r in outputs),
            'unobserved_marker_frames': sum(any(r.get(k, {}).get('state') == 'MARKER_NOT_OBSERVED' for k in ('stator', 'output')) for r in outputs),
            'bounded_projected_angles': bounded,
            'repeated_image_hash_frames': repeated_image_hashes,
            'data_coverage_complete': captured == count and len(projected) == count,
            'all_projected_error_bounds_supplied': bounded == count,
            'conditional_joint_changes': sum(c['conditional_joint_displacement_circular_interval_rad'] is not None for c in changes)},
        'timing': {'captured_span_s': span, 'captured_frame_gaps_s': captured_gaps,
            'usable_angle_gaps_s': gaps, 'max_captured_frame_gap_s': max(captured_gaps, default=None),
            'observed_average_frame_rate_hz': None if span is None else number((len(acquired) - 1) / span),
            'nominal_spacing_nyquist_hz_only': None if nominal is None else number(1 / (2 * nominal)),
            'worst_gap_half_rate_hz_only': None if not captured_gaps else number(1 / (2 * max(captured_gaps))),
            'spacing_fails_nyquist_for_declared_interest': None if interest is None or not captured_gaps else (
                True if max(captured_gaps) >= 1 / (2 * interest) else False if all_times_known else None),
            'timestamp_error_unknown_frames': sum(r['timestamp_max_error_s'] is None for r in acquired),
            'timestamp_unknown_frames': sum(r['timestamp_s'] is None for r in acquired),
            'all_captured_timestamps_known': all_times_known,
            'exposure_unknown_frames': sum(r['exposure_s'] is None for r in acquired),
            'clock_alignment_unknown': value['external_clock_alignment'] is None,
            'measurement_bandwidth_unknown': value['camera']['measured_acquisition_bandwidth_hz'] is None,
            'sensor_acquisition_time_verified': False, 'motion_bandlimited': False},
        'frames': outputs, 'changes': changes,
        'absolute_joint_angle_rad': None, 'absolute_origin_error_rad': None,
        'absolute_origin_uncertainty_rad': None, 'physical_velocity_rad_per_s': None,
        'limitations': [
            'Supplied point/projection/time bounds and physical conditions are unverified input evidence, not certified accuracy.',
            'Relative image angle cancels a common in-plane rotation; perspective, out-of-plane camera motion, distortion, rolling shutter and marker flex remain conditional.',
            'Projection geometry, rigid attachments, an evidenced image-to-joint sign and a separate projection bound are required for conditional joint-angle change intervals; mounting offset and absolute mechanical zero remain unknown.',
            'All changes are circular principal differences. Unknown extra turns and within-frame motion prevent unambiguous angular velocity or continuous unwrap.',
            'Missing frames and degenerate marker vectors remain in requested coverage; no missing image, point, timestamp or angle is interpolated.',
            'Frame spacing half-rate is only a sampling descriptor. Exposure averaging, unknown measurement bandwidth and sensor timestamps leave aliasing and high-frequency/subpixel motion unresolved.',
            'External clock alignment is supplied metadata only. This tool does not retrospectively synchronize CAN observations or establish an internal velocity-estimator fault.']}


def template(joint_id=5, frame_count=3):
    integer(joint_id, 1, 127); integer(frame_count, 1, MAX_FRAMES)
    return {'schema': SCHEMA, 'record_kind': 'PLANNED_UNMEASURED', 'joint_id': joint_id,
        'image_axes': 'x_right_y_down_ccw_angle_positive',
        'camera': {'id': 'UNKNOWN', 'view_id': 'UNKNOWN', 'lens_id': 'UNKNOWN', 'frame_dimensions_px': None,
            'timestamp_clock': 'UNKNOWN', 'timestamp_definition': 'unknown', 'nominal_frame_interval_s': None,
            'measured_acquisition_bandwidth_hz': None},
        'markers': {k: {'rigid_body_id': k, 'point_ids': [k + '-point-0', k + '-point-1'],
            'rigid_attachment_confirmed': None} for k in ('stator', 'output')},
        'geometry_conditions': dict.fromkeys(CONDITIONS), 'per_frame_projection_error_bound_rad': None,
        'image_to_joint_sign': None,
        'external_clock_alignment': None, 'highest_motion_frequency_of_interest_hz': None,
        'requested_frame_count': frame_count,
        'frames': [dict(index=i, state='UNMEASURED', frame_id=None, image_sha256=None,
            timestamp_s=None, timestamp_max_error_s=None, exposure_s=None, missing_reason=None,
            stator_points=None, output_points=None) for i in range(frame_count)]}


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items: need(key not in result, 'Duplicate JSON key'); result[key] = value
        return result
    def finite_float(raw_number):
        value = float(raw_number); need(math.isfinite(value), 'Nonfinite JSON value'); return value
    return json.loads(raw, object_pairs_hook=pairs, parse_float=finite_float,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON value')))


def read_file(path):
    path = Path(path).expanduser().absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Symlink input refused')
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as source:
        info = os.fstat(source.fileno())
        need(stat.S_ISREG(info.st_mode) and info.st_size <= MAX_BYTES, 'Bounded regular input required')
        raw = source.read(MAX_BYTES + 1); need(len(raw) <= MAX_BYTES, 'Input exceeds byte budget')
    return strict_json(raw), {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input'); source.add_argument('--template', action='store_true')
    parser.add_argument('--joint-id', type=int, default=5)
    parser.add_argument('--frame-count', type=int, default=3)
    args = parser.parse_args(argv)
    try:
        if args.template:
            result = template(args.joint_id, args.frame_count)
        else:
            need(args.joint_id == 5 and args.frame_count == 3,
                 'Set joint/frame metadata in input; CLI template options do not override observations')
            value, binding = read_file(args.input); result = analyze(value); result['input_artifact'] = binding
        print(json.dumps(result, allow_nan=False, sort_keys=True, indent=2))
        return 0
    except (OSError, ValueError, TypeError, OverflowError, RecursionError) as error:
        print(json.dumps({'schema': RESULT_SCHEMA, 'status': 'INVALID_OPTICAL_ARTIFACT',
            **FLAGS, 'error': str(error)}, allow_nan=False, sort_keys=True))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
