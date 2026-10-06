#!/usr/bin/env python3
"""File-only norm-based diagonal IMU candidate; approximate faces stay approximate.

Fit six acceleration means from each fit capture's chronological first75%.
Norm constraints do not assert exact axis alignment or antipodal poses. Six
equations determine six parameters with zero fit residual degrees of freedom.
The untouched last25% and six separate captures are evaluation inputs only.
This diagnostic schema is not a runtime calibration file or an approval.
"""
import argparse
import hashlib
import math
from pathlib import Path

from singularitydog_hw import imu_calibration as calibration
from analyze_stationary_velocity_probe import exact, need, read_file, strict_json

INPUT_SCHEMA = 'singularitydog.imu-pose-tilt-input.v1'
SCHEMA = 'singularitydog.imu-pose-tilt-diagnostic.v1'
# Numerical solve guards, not physical acceptance or calibration thresholds.
PIVOT_MIN = 1e-12
CONDITION_MAX = 1e8


def fit_norm_ellipse(means):
    """Solve [u²,u]p=1 for u=measured/g; no supplied pose angles are used."""
    need(type(means) is dict and set(means) == set(calibration.FACES), 'Six face means required')
    u = []
    for label in calibration.FACES:
        vector = means[label]
        need(type(vector) in (list, tuple) and len(vector) == 3, 'Three mean values required')
        need(all(type(x) in (int, float) and math.isfinite(x)
                 and abs(x) <= 4*calibration.GRAVITY for x in vector), 'Finite bounded mean required')
        u.append([x/calibration.GRAVITY for x in vector])
    matrix = [[x*x for x in row]+row for row in u]
    columns = [max(abs(row[i]) for row in matrix) for i in range(6)]
    need(all(x > 0 for x in columns), 'Rank-deficient norm design')
    normalized = [[x/columns[i] for i, x in enumerate(row)] for row in matrix]
    augmented = [row[:] + [float(i == j) for j in range(6)] for i, row in enumerate(normalized)]
    for column in range(6):
        pivot = max(range(column, 6), key=lambda i: abs(augmented[i][column]))
        need(abs(augmented[pivot][column]) > PIVOT_MIN, 'Rank-deficient or numerically singular norm design')
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        value = augmented[column][column]
        augmented[column] = [x/value for x in augmented[column]]
        for i in range(6):
            if i != column:
                value = augmented[i][column]
                augmented[i] = [a-value*b for a, b in zip(augmented[i], augmented[column])]
    inverse = [row[6:] for row in augmented]
    condition = max(math.fsum(map(abs, row)) for row in normalized)*max(math.fsum(map(abs, row)) for row in inverse)
    original_inverse = [[x/columns[i] for x in row] for i, row in enumerate(inverse)]
    original_condition = max(math.fsum(map(abs, row)) for row in matrix)*max(math.fsum(map(abs, row)) for row in original_inverse)
    need(all(math.isfinite(x) and x <= CONDITION_MAX for x in (condition, original_condition)), 'Ill-conditioned norm design')
    coefficients = [math.fsum(row)/columns[i] for i, row in enumerate(inverse)]
    q, linear = coefficients[:3], coefficients[3:]
    need(all(math.isfinite(x) for x in coefficients) and all(x > 0 for x in q), 'Positive finite ellipse coefficients required')
    center = [-l/(2*v) for l, v in zip(linear, q)]
    k = 1+math.fsum(v*b*b for v, b in zip(q, center))
    need(math.isfinite(k) and k > 0, 'Positive finite ellipse normalizer required')
    bias = [calibration.GRAVITY*x for x in center]
    scale = [math.sqrt(v/k) for v in q]
    need(all(math.isfinite(x) for x in bias+scale), 'Finite bias/scale required')
    residual = [math.fsum(a*b for a, b in zip(row, coefficients))-1 for row in matrix]
    return {'bias_m_s2': bias, 'scale': scale,
        'formula': 'corrected[i] = (measured[i] - bias_m_s2[i]) * scale[i]',
        'normalized_quadratic_coefficients': q, 'normalized_linear_coefficients': linear,
        'normalized_center': center, 'ellipse_normalizer': k,
        'normalized_column_scales': columns, 'column_normalized_condition_inf': condition,
        'dimensionless_design_condition_inf': original_condition,
        'rank': 6, 'equations': 6, 'parameters': 6, 'fit_residual_degrees_of_freedom': 0,
        'maximum_mean_equation_residual': max(map(abs, residual)),
        'numerical_condition_rejection_limit': CONDITION_MAX,
        'numerical_condition_is_physical_uncertainty_bound': False}


def evaluate(rows, candidate, label):
    bias, scale = candidate['bias_m_s2'], candidate['scale']
    vectors = [[(a-b)*s for a, b, s in zip(row['accel'], bias, scale)] for row in rows]
    norms = [math.hypot(*v) for v in vectors]
    errors = [n-calibration.GRAVITY for n in norms]
    mean = calibration._mean(vectors)
    axis, sign = 'xyz'.index(label[0]), (1 if label[1] == '+' else -1)
    angle = math.degrees(math.acos(max(-1., min(1., sign*mean[axis]/math.hypot(*mean)))))
    return {'samples': len(rows), 'corrected_mean_m_s2': mean,
        'corrected_norm_mean_m_s2': math.fsum(norms)/len(norms),
        'norm_mean_error_m_s2': math.fsum(errors)/len(errors),
        'norm_RMS_error_m_s2': math.sqrt(math.fsum(x*x for x in errors)/len(errors)),
        'norm_max_absolute_error_m_s2': max(map(abs, errors)),
        'corrected_mean_angle_from_labeled_axis_deg_descriptive': angle,
        'exact_labeled_axis_alignment_verified': False, 'physical_pose_uncertainty_rad': None}


def analyze_datasets(fit, independent):
    limits = calibration.CalibrationLimits()
    need(type(fit) is dict and type(independent) is dict
         and set(fit) == set(independent) == set(calibration.FACES), 'Six fit and six independent datasets required')
    train, held, validation, stats, intervals, fingerprints = {}, {}, {}, {}, {}, set()
    for partition, datasets in (('fit', fit), ('independent', independent)):
        for label in calibration.FACES:
            rows, first, last, stat = calibration._validate_face(label, datasets[label], limits)
            digest = stat['measurement_sequence_sha256']
            need(digest not in fingerprints, 'Reused fit/holdout measurement sequence')
            fingerprints.add(digest); intervals[partition+' '+label] = rows
            if partition == 'fit':
                train[label], held[label], stats[label] = first, last, stat
            else:
                validation[label] = rows
    clock = calibration._validate_interval_provenance(intervals)
    means = {label: calibration._mean([r['accel'] for r in rows]) for label, rows in train.items()}
    candidate = fit_norm_ellipse(means)
    pairs = {}
    for axis in 'xyz':
        positive, negative = means[axis+'+'], means[axis+'-']
        midpoint = [(a+b)/2 for a, b in zip(positive, negative)]
        half = [(a-b)/2 for a, b in zip(positive, negative)]
        corrected_mid = [(a-b)*s for a, b, s in zip(midpoint, candidate['bias_m_s2'], candidate['scale'])]
        plus = [(a-b)*s for a, b, s in zip(positive, candidate['bias_m_s2'], candidate['scale'])]
        minus = [(a-b)*s for a, b, s in zip(negative, candidate['bias_m_s2'], candidate['scale'])]
        cosine = -math.fsum(a*b for a, b in zip(plus, minus))/(math.hypot(*plus)*math.hypot(*minus))
        pairs[axis] = {'raw_pair_midpoint_m_s2': midpoint, 'raw_half_difference_m_s2': half,
            'corrected_pair_midpoint_m_s2': corrected_mid,
            'corrected_pair_midpoint_norm_m_s2': math.hypot(*corrected_mid),
            'opposition_mismatch_angle_deg_descriptive': math.degrees(math.acos(max(-1., min(1., cosine)))),
            'pair_midpoint_is_common_sensor_bias': False, 'antipodal_pose_verified': False}
    return {'schema': SCHEMA, 'status': 'UNAPPROVED_NORM_ELLIPSE_DIAGNOSTIC',
        'method': 'norm-based diagonal offline candidate', 'frame': 'sensor',
        'gravity_reference_m_s2': calibration.GRAVITY, 'accel_diagnostic_candidate': candidate,
        'fit_partition': 'chronological first75% of each of six fit captures',
        'fit_training_means_m_s2': means, 'fit_capture_stats': stats, 'pair_descriptors': pairs,
        'evaluation': {'fit_training': {l: evaluate(train[l], candidate, l) for l in calibration.FACES},
            'chronological_last25_percent': {l: evaluate(held[l], candidate, l) for l in calibration.FACES},
            'independent_captures': {l: evaluate(validation[l], candidate, l) for l in calibration.FACES}},
        'acquisition_interval_clock': clock, 'heldout_used_for_fit': False,
        'independent_captures_used_for_fit': False, 'gyro_bias_or_scale_estimated': False,
        'physical_pose_uncertainty_rad': None, 'absolute_scale_uncertainty': None,
        'approved_for_runtime': False, 'automatically_applied': False, 'hardware_opened': False,
        'motor_output_allowed': False, 'profile_changed': False, 'thresholds_changed': False,
        'limitations': ['Six means and six parameters give zero fit residual degrees of freedom; exact fit does not prove accuracy.',
            'The norm model assumes static gravity with fixed bias and positive diagonal scale; non-gravity acceleration, cross-axis error and temperature variation remain unmodeled.',
            'Unknown local gravity is confounded with common scale. Pose/reference/mount uncertainty and absolute orientation remain unquantified.',
            'A full cross-axis ellipsoid needs more independently varied orientations; six means cannot identify its nine parameters.',
            'Separate holdout captures share sensor and operator errors. Norm residuals and descriptive axis angles are not physical calibration approval.']}


def analyze_file(path):
    content, manifest_pin = read_file(path, 16384)
    manifest = strict_json(content)
    need(type(manifest) is dict and set(manifest) == {'schema', 'operator_confirmed_stationary', 'fit', 'independent'}, 'Exact input manifest fields required')
    exact(manifest['schema'], INPUT_SCHEMA, 'Input schema')
    exact(manifest['operator_confirmed_stationary'], True, 'Operator stationarity assertion required')
    datasets, pins, reference = {'fit': {}, 'independent': {}}, [manifest_pin], None
    seen = set()
    for partition in datasets:
        paths = manifest[partition]
        need(type(paths) is dict and set(paths) == set(calibration.FACES), 'Six pinned capture directories required')
        for label in calibration.FACES:
            pin = paths[label]
            need(type(pin) is dict and set(pin) == {'directory', 'summary_sha256', 'events_sha256'}, 'Capture pin fields required')
            need(type(pin['directory']) is str and bool(pin['directory']), 'Capture directory required')
            directory = Path(pin['directory']).expanduser()
            if not directory.is_absolute(): directory = Path(path).resolve().parent/directory
            observed = {}
            for name, key in (('summary.json', 'summary_sha256'), ('events.jsonl', 'events_sha256')):
                _, binding = read_file(directory/name)
                exact(binding['sha256'], pin[key], 'Capture input SHA')
                need(binding['path'] not in seen, 'Reused capture path')
                seen.add(binding['path']); pins.append(binding); observed[key] = binding['sha256']
            meta, rows, stats, origin = calibration.baseline._load_capture(directory, expected_face=label)
            for key in observed: exact(origin[key], observed[key], 'Audited capture input SHA')
            if reference is None: reference = meta
            for key in ('source_sha256', 'configuration', 'register_audit_before'):
                exact(meta[key], reference[key], 'Capture setup')
            for key in ('bus', 'address'): exact(meta['plan'][key], reference['plan'][key], 'Capture device')
            temp = stats['temperature_c']; limits = calibration.baseline.LIMITS
            need(temp['max']-temp['min'] <= limits['temperature_span_max_c']
                 and abs(stats['first_to_last_quarter']['temperature_mean_change_c']) <= limits['temperature_drift_max_c']
                 and abs(temp['mean']-reference['summary']['temperature_c']['mean']) <= limits['temperature_mean_change_max_c'], 'Capture temperature gates')
            datasets[partition][label] = rows
    sources = [Path(__file__), Path(calibration.__file__), Path(calibration.baseline.__file__)]
    # Include the shared strict bounded-file reader in the analysis provenance.
    import analyze_stationary_velocity_probe as reader
    sources.append(Path(reader.__file__))
    source_pins = [read_file(p)[1] for p in sources]
    report = analyze_datasets(datasets['fit'], datasets['independent'])
    report.update(input_bindings=pins, source_bindings=source_pins, capture_audit_verified=True,
        operator_stationarity_assertion_recorded=True, physical_stationarity_proven=False)
    for pin in pins+source_pins: exact(read_file(pin['path'])[1], pin, 'Input/source changed during analysis')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        need(not args.output.exists() and not args.output.is_symlink(), 'Fresh private output required')
        report = analyze_file(args.input)
        calibration._write_candidate(report, args.output)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        parser.error(str(error))
    print('UNAPPROVED_NORM_ELLIPSE_DIAGNOSTIC '+hashlib.sha256(args.output.read_bytes()).hexdigest())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
