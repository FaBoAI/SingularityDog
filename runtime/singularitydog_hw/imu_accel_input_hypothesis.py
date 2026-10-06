"""Pinned norm-only acceleration hypothesis for a separately authorized small trial.

No device API, dynamic source imports, motor authorization or calibration approval.
The report is independently reconstructed from twelve raw captures. The duplicated
pure reference calculation intentionally matches a fixed diagnostic revision;
changing it requires new source bindings and a new hypothesis artifact. Unknown
mount/reference/cross-axis/temperature errors remain unknown after this load.
"""
from dataclasses import dataclass, field
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat

from . import imu_calibration as calibration

SCHEMA = 'singularitydog.supported-accel-input-hypothesis.v1'
SCOPE = 'boxed_small_mix_only'
DECISION = 'ACCEPT_BOXED_SMALL_MIX_INPUT_HYPOTHESIS'
INPUT_SCHEMA = 'singularitydog.imu-pose-tilt-input.v1'
REPORT_SCHEMA = 'singularitydog.imu-pose-tilt-diagnostic.v1'
PIVOT_MIN = 1e-12
CONDITION_MAX = 1e8
MAX_BYTES = 4*1024*1024
FROZEN_ANALYSIS_SOURCES = {
    'analyze_imu_pose_tilt.py': 'efe6da19beebc2b136ca528a9a5b4a7305104f49682226b6d1d9e3edf249474f',
    'analyze_stationary_velocity_probe.py': '2d3fd9d1619aa8b79fd4fb3d9cfbd91b95247cf9e94e577933f6e6d057980cbd'}
SOURCE_PATHS = ('imu_accel_input_hypothesis.py', 'imu_calibration.py',
                'imu_fixed_mount_baseline.py', 'imu.py', 'imu_capture.py')
ASSUMPTIONS = ('stationary_gravity_diagonal_model', 'cross_axis_and_temperature_not_identified',
               'absolute_orientation_not_certified')
_VALIDATED = object()


def need(condition, message):
    if not condition:
        raise ValueError(message)


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def number(value):
        result = float(value)
        need(math.isfinite(result), 'Nonfinite JSON number')
        return result
    def bad(_):
        raise ValueError('Nonfinite JSON constant')
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number, parse_constant=bad)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def exact(actual, expected, label):
    need(_canonical(actual) == _canonical(expected), label+' mismatch')


def read_file(path, maximum=MAX_BYTES):
    path = Path(path).expanduser().absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Input symlink refused')
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= maximum,
             'Expected bounded regular nonempty file')
        raw = handle.read(maximum+1)
        named = path.lstat()
        need((info.st_dev, info.st_ino, info.st_size) == (named.st_dev, named.st_ino, named.st_size)
             and len(raw) == info.st_size, 'Input binding/size changed')
    return raw, {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'byte_count': len(raw)}


def _reference(reference, label):
    need(type(reference) is dict and set(reference) == {'path', 'sha256'}, label+' pinned reference required')
    need(type(reference['path']) is str and Path(reference['path']).is_absolute(), label+' absolute path required')
    need(type(reference['sha256']) is str and re.fullmatch('[0-9a-f]{64}', reference['sha256']), label+' SHA required')
    raw, pin = read_file(reference['path'])
    exact(pin['sha256'], reference['sha256'], label+' SHA')
    return strict_json(raw), pin


def source_hashes():
    root = Path(__file__).resolve().parent
    return {'singularitydog_hw/'+name: read_file(root/name)[1]['sha256'] for name in SOURCE_PATHS}


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
    return {'schema': REPORT_SCHEMA, 'status': 'UNAPPROVED_NORM_ELLIPSE_DIAGNOSTIC',
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


def _audit_report(manifest_ref, candidate_ref):
    manifest, manifest_pin = _reference(manifest_ref, 'Manifest')
    candidate, candidate_pin = _reference(candidate_ref, 'Candidate')
    need(type(candidate) is dict, 'Candidate object required')
    expected_sources = {**FROZEN_ANALYSIS_SOURCES,
        'imu_calibration.py': read_file(Path(calibration.__file__))[1]['sha256'],
        'imu_fixed_mount_baseline.py': read_file(Path(calibration.baseline.__file__))[1]['sha256']}
    source_pins = candidate.get('source_bindings')
    need(type(source_pins) is list and len(source_pins) == 4, 'Four diagnostic source bindings required')
    names = set()
    for pin in source_pins:
        need(type(pin) is dict and set(pin) == {'path', 'sha256', 'byte_count'} and
             type(pin['path']) is str and Path(pin['path']).is_absolute(), 'Diagnostic source pin required')
        name = Path(pin['path']).name
        need(name in expected_sources and name not in names, 'Unknown/duplicate diagnostic source')
        names.add(name)
        exact(pin['sha256'], expected_sources[name], 'Diagnostic implementation source')
        exact(read_file(pin['path'])[1], pin, 'Diagnostic source bytes')
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
            if not directory.is_absolute(): directory = Path(manifest_pin['path']).parent/directory
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
    recomputed = analyze_datasets(datasets['fit'], datasets['independent'])
    recomputed.update(input_bindings=pins, source_bindings=source_pins, capture_audit_verified=True,
        operator_stationarity_assertion_recorded=True, physical_stationarity_proven=False)
    comparable = {key: value for key, value in candidate.items() if key != 'generated_at_utc'}
    exact(comparable, recomputed, 'Norm diagnostic recomputation')
    all_pins = pins+source_pins+[candidate_pin]
    for pin in all_pins:
        exact(read_file(pin['path'])[1], pin, 'Input/source changed during hypothesis audit')
    return candidate, datasets, all_pins


def _rotation(value):
    need(type(value) in (list, tuple) and len(value) == 3 and
         all(type(row) in (list, tuple) and len(row) == 3 and
             all(type(x) in (int, float) and math.isfinite(x) for x in row) for row in value),
         'Finite real 3x3 mount rotation required')
    matrix = [[float(x) for x in row] for row in value]
    need(all(abs(math.fsum(a*b for a,b in zip(matrix[i],matrix[j]))-(i==j)) <= 1e-6
             for i in range(3) for j in range(3)), 'Mount rotation must be orthonormal')
    det = math.fsum(matrix[0][j]*(matrix[1][(j+1)%3]*matrix[2][(j+2)%3]
                                 -matrix[1][(j+2)%3]*matrix[2][(j+1)%3]) for j in range(3))
    need(abs(det-1) <= 1e-6, 'Mount rotation must be right handed')
    return matrix


def _bounds(value, label, *, centered=False):
    need(type(value) is list and len(value) == 2 and all(type(x) in (int,float) and math.isfinite(x) for x in value), label+' bounds required')
    low, high = value
    need(8.8 <= low < high <= 11.2 and high-low <= 1.5, label+' bounds outside existing envelope')
    if centered: need(low < calibration.GRAVITY < high, label+' bounds must contain gravity reference')
    return float(low), float(high)


@dataclass(frozen=True)
class AccelInputHypothesis:
    bias_m_s2: tuple
    scale: tuple
    raw_norm_min_m_s2: float
    raw_norm_max_m_s2: float
    corrected_norm_min_m_s2: float
    corrected_norm_max_m_s2: float
    reference_sha256: str
    candidate_sha256: str
    manifest_sha256: str
    _provenance_json: str = field(repr=False)
    _proof: object = field(repr=False)

    def correct(self, acceleration):
        need(self._proof is _VALIDATED, 'Hypothesis loader proof required')
        need(type(acceleration) in (list,tuple) and len(acceleration) == 3 and
             all(type(x) in (int,float) and math.isfinite(x) for x in acceleration),
             'Finite uncorrected acceleration triplet required')
        need(self.raw_norm_min_m_s2 <= math.hypot(*acceleration) <= self.raw_norm_max_m_s2,
             'Raw acceleration norm outside hypothesis range')
        corrected = [(a-b)*s for a,b,s in zip(acceleration,self.bias_m_s2,self.scale)]
        norm = math.hypot(*corrected)
        need(all(math.isfinite(x) for x in corrected) and
             self.corrected_norm_min_m_s2 <= norm <= self.corrected_norm_max_m_s2,
             'Corrected acceleration norm outside hypothesis range')
        return corrected, norm

    def provenance(self):
        need(self._proof is _VALIDATED, 'Hypothesis loader proof required')
        return strict_json(self._provenance_json)


def load_accel_input_hypothesis(reference, mount_rotation):
    """Audit saved bytes only. Profile/live callers must separately authorize scope."""
    document, wrapper_pin = _reference(reference, 'Hypothesis')
    keys = {'schema','scope','manifest','candidate','R_body_from_sensor','raw_norm_bounds_m_s2',
        'corrected_norm_bounds_m_s2','source_sha256','hypothesis_review','assumptions_acknowledged',
        'formal_calibration_approved','absolute_orientation_error_bound_rad','grants_motor_output'}
    need(type(document) is dict and set(document) == keys and document['schema'] == SCHEMA
         and document['scope'] == SCOPE, 'Exact supported hypothesis schema/scope required')
    need(document['formal_calibration_approved'] is False and document['grants_motor_output'] is False
         and document['absolute_orientation_error_bound_rad'] is None, 'Hypothesis cannot certify calibration or motor output')
    exact(document['source_sha256'], source_hashes(), 'Hypothesis runtime sources')
    assumptions = document['assumptions_acknowledged']
    need(type(assumptions) is dict and set(assumptions) == set(ASSUMPTIONS) and
         all(x is True for x in assumptions.values()), 'Unidentified errors must be explicitly acknowledged')
    review = document['hypothesis_review']
    need(type(review) is dict and set(review) == {'reviewer','reviewed_at','decision','rationale'}
         and review['decision'] == DECISION, 'Explicit limited hypothesis review required')
    for key in ('reviewer','rationale'):
        need(type(review[key]) is str and bool(review[key].strip()) and len(review[key]) <= 2048,
             'Named limited hypothesis review required')
    try:
        stamp = datetime.fromisoformat(review['reviewed_at'].replace('Z','+00:00'))
        need(stamp.utcoffset() is not None, 'Hypothesis review requires timezone')
    except (ValueError,AttributeError,TypeError) as error:
        raise ValueError('Invalid hypothesis review timestamp') from error
    rotation = _rotation(document['R_body_from_sensor'])
    exact(rotation, _rotation(mount_rotation), 'Selected hypothesis mount')
    raw_low, raw_high = _bounds(document['raw_norm_bounds_m_s2'], 'Raw acceleration')
    low, high = _bounds(document['corrected_norm_bounds_m_s2'], 'Corrected acceleration', centered=True)
    candidate, datasets, pins = _audit_report(document['manifest'],document['candidate'])
    fitted = candidate['accel_diagnostic_candidate']
    bias, scale = tuple(fitted['bias_m_s2']), tuple(fitted['scale'])
    need(len(bias) == len(scale) == 3 and all(type(x) in (int,float) and math.isfinite(x) for x in bias+scale)
         and all(x > 0 for x in scale), 'Positive finite diagonal candidate required')
    # Every chronological holdout and independent sample is checked with the
    # fixed parameters; these observations never change the fit or the bounds.
    limits = calibration.CalibrationLimits()
    for partition, captures in datasets.items():
        for label, capture in captures.items():
            rows, _, held, _ = calibration._validate_face(label,capture,limits)
            for row in (held if partition == 'fit' else rows):
                norm = math.hypot(*[(a-b)*s for a,b,s in zip(row['accel'],bias,scale)])
                need(low <= norm <= high, 'Heldout corrected sample outside hypothesis bounds')
    for pin in [wrapper_pin,*pins]:
        exact(read_file(pin['path'])[1],pin,'Hypothesis input/source changed before return')
    exact(document['source_sha256'],source_hashes(),'Hypothesis runtime sources changed before return')
    provenance = {'kind':SCHEMA,'scope':SCOPE,'hypothesis_sha256':wrapper_pin['sha256'],
        'candidate_sha256':document['candidate']['sha256'],'manifest_sha256':document['manifest']['sha256'],
        'bias_sensor_m_s2':list(bias),'scale_sensor':list(scale),'R_body_from_sensor':rotation,
        'raw_norm_bounds_m_s2':[raw_low,raw_high],'corrected_norm_bounds_m_s2':[low,high],
        'source_sha256':document['source_sha256'],'fit_and_independent_captures_reaudited':True,
        'formal_calibration_approved':False,'absolute_orientation_error_bound_rad':None,
        'grants_motor_output':False}
    return AccelInputHypothesis(bias,scale,raw_low,raw_high,low,high,wrapper_pin['sha256'],
        document['candidate']['sha256'],document['manifest']['sha256'],_canonical(provenance),_VALIDATED)


def create_template(manifest_reference, candidate_reference, mount_rotation):
    """Return an unreviewed in-memory template; no file writes or approvals."""
    _audit_report(manifest_reference,candidate_reference)
    return {'schema':SCHEMA,'scope':SCOPE,'manifest':dict(manifest_reference),'candidate':dict(candidate_reference),
        'R_body_from_sensor':_rotation(mount_rotation),'raw_norm_bounds_m_s2':None,'corrected_norm_bounds_m_s2':None,
        'source_sha256':source_hashes(),'hypothesis_review':{'reviewer':None,'reviewed_at':None,
            'decision':'UNREVIEWED','rationale':None},'assumptions_acknowledged':dict.fromkeys(ASSUMPTIONS,False),
        'formal_calibration_approved':False,'absolute_orientation_error_bound_rad':None,'grants_motor_output':False}
