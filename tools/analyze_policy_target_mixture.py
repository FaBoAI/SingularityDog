"""Compare saved raw policy targets with a measured initial pose, without output.

This is an algebraic replay of targets generated from recorded observations.
It does not predict feedback, loading, balance, or future closed-loop motion.
No devices, model, approvals, or output files are accessed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import stat

ID_ORDER = list(range(1, 13))
WEIGHTS = (0.1, 0.3, 0.5, 1.0)
TARGET_SCHEMA = 'singularitydog.saved-policy-target-sequence.v1'


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError('Finite numeric value required: ' + label)
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError('Finite numeric value required: ' + label) from error
    if not math.isfinite(number):
        raise ValueError('Finite numeric value required: ' + label)
    return number


def _vector(value, label):
    if type(value) is not list or len(value) != 12:
        raise ValueError('Twelve values in ID1..12 order required: ' + label)
    return [_number(x, label) for x in value]


def _caps(value, label):
    values = _vector(value, label) if type(value) is list else [_number(value, label)] * 12
    if any(x <= 0 for x in values):
        raise ValueError('Positive cap required: ' + label)
    return values


def _digest(value, label):
    if type(value) is not str or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('Canonical SHA256 required: ' + label)
    return value


def analyze(report, targets, *, report_sha256, targets_sha256=None,
            physical_clearance_deg, max_displacement_deg):
    """Compute fixed-origin mixtures; the supplied clearance is not certified."""
    _digest(report_sha256, 'report')
    if targets_sha256 is not None:
        _digest(targets_sha256, 'targets')
    if type(report) is not dict or type(targets) is not dict:
        raise ValueError('Report and target JSON objects required')
    order = targets.get('id_order')
    if (targets.get('schema') != TARGET_SCHEMA or type(order) is not list or
            any(type(i) is not int for i in order) or order != ID_ORDER):
        raise ValueError('Saved target schema and exact ID order required')
    if targets.get('report_sha256') != report_sha256:
        raise ValueError('Raw targets are bound to a different report')
    for key in ('output_allowed', 'hardware_accessed', 'closed_loop_prediction'):
        if targets.get(key, False) is not False:
            raise ValueError('Target sequence must not claim ' + key)
    cycles = report.get('cycles')
    if type(cycles) is not list or not cycles:
        raise ValueError('Nonempty saved cycle sequence required')
    feedback = {}
    last_index = -1
    for cycle in cycles:
        if type(cycle) is not dict:
            raise ValueError('Invalid saved cycle')
        index = cycle.get('index')
        if type(index) is not int or index <= last_index:
            raise ValueError('Unique ascending nonnegative report cycle indices required')
        last_index = index
        sample = cycle.get('feedback')
        if type(sample) is not dict:
            raise ValueError('Saved cycle feedback required')
        feedback[index] = _vector(sample.get('q_model_rad'), 'cycle feedback')
    origin = report.get('trial_origin_model_rad_by_id')
    if origin is None:
        initial = feedback[cycles[0]['index']]
        origin_source = 'first_saved_cycle_feedback'
    else:
        if type(origin) is not dict or set(origin) != {str(i) for i in ID_ORDER}:
            raise ValueError('Initial pose must contain exact IDs1..12')
        initial = [_number(origin[str(i)], 'initial pose') for i in ID_ORDER]
        origin_source = 'report_trial_origin_model_rad_by_id'
    if 'initial_q_model_rad_by_id' in targets:
        if _vector(targets['initial_q_model_rad_by_id'], 'replay initial pose') != initial:
            raise ValueError('Replay initial pose differs from report')
    rows = targets.get('rows')
    if type(rows) is not list or not rows:
        raise ValueError('At least one raw target row required')
    checked = []
    last_index = -1
    for row in rows:
        if type(row) is not dict:
            raise ValueError('Invalid raw target row')
        index = row.get('cycle_index')
        if type(index) is not int or index <= last_index or index not in feedback:
            raise ValueError('Raw target cycle indices must be unique, ascending and present in report')
        last_index = index
        raw = _vector(row.get('raw_target_model_rad'), 'raw target')
        if 'feedback_q_model_rad_by_id' in row:
            if _vector(row['feedback_q_model_rad_by_id'], 'replay feedback') != feedback[index]:
                raise ValueError('Replay feedback differs from saved cycle')
        checked.append((index, raw))
    physical = _caps(physical_clearance_deg, 'physical clearance')
    displacement = _caps(max_displacement_deg, 'maximum displacement')
    counts_match = type(report.get('actual_model_calls')) is int and len(checked) == report['actual_model_calls']
    extent = ('single_target_snapshot' if len(checked) == 1 else
              'logged_model_call_count_matches' if counts_match else 'partial_saved_sequence')
    results = []
    for weight in WEIGHTS:
        axes = []
        for offset, axis_id in enumerate(ID_ORDER):
            deltas = [weight * (raw[offset] - initial[offset]) for _, raw in checked]
            if not all(math.isfinite(x) for x in deltas):
                raise ValueError('Nonfinite mixture result')
            peak_index = max(range(len(deltas)), key=lambda n: abs(deltas[n]))
            peak_deg = math.degrees(deltas[peak_index])
            endpoint = initial[offset] + deltas[-1]
            if not math.isfinite(peak_deg) or not math.isfinite(endpoint):
                raise ValueError('Nonfinite mixture endpoint or degrees')
            axes.append(dict(id=axis_id, initial_q_model_rad=initial[offset],
                first_raw_target_model_rad=checked[0][1][offset],
                first_delta_deg=math.degrees(deltas[0]),
                first_direction='positive' if deltas[0] > 0 else 'negative' if deltas[0] < 0 else 'unchanged',
                final_raw_target_model_rad=checked[-1][1][offset],
                mixed_target_endpoint_model_rad=endpoint, final_delta_deg=math.degrees(deltas[-1]),
                peak_absolute_delta_deg=abs(peak_deg), peak_signed_delta_deg=peak_deg,
                peak_cycle_index=checked[peak_index][0],
                maximum_positive_delta_deg=max(0.0, *map(math.degrees, deltas)),
                maximum_negative_delta_deg=min(0.0, *map(math.degrees, deltas)),
                physical_clearance_deg=physical[offset], maximum_displacement_deg=displacement[offset],
                exceeds_supplied_physical_clearance=abs(peak_deg) > physical[offset],
                exceeds_supplied_maximum_displacement=abs(peak_deg) > displacement[offset]))
        physical_exceeded = [a['id'] for a in axes if a['exceeds_supplied_physical_clearance']]
        displacement_exceeded = [a['id'] for a in axes if a['exceeds_supplied_maximum_displacement']]
        recommendation = ('SUPPLIED_PHYSICAL_CLEARANCE_EXCEEDED_KEEP_CURRENT_TRIAL_CAPS' if physical_exceeded else
                          'CURRENT_MAXIMUM_DISPLACEMENT_EXCEEDED_NEW_REVIEW_REQUIRED' if displacement_exceeded else
                          'NUMERIC_CAPS_ONLY_MATCH_NO_EXECUTION_APPROVAL')
        results.append(dict(weight=weight, per_axis=axes, physical_clearance_exceeded_ids=physical_exceeded,
                            maximum_displacement_exceeded_ids=displacement_exceeded,
                            maximum_absolute_delta_deg=max(a['peak_absolute_delta_deg'] for a in axes),
                            recommendation=recommendation, output_allowed=False))
    return dict(schema='singularitydog.saved-policy-target-mixture-analysis.v1',
                input_sha256=dict(report=report_sha256, targets=targets_sha256), id_order=ID_ORDER,
                weights=list(WEIGHTS), initial_pose_source=origin_source,
                initial_q_model_rad_by_id=initial, target_rows=len(checked), sequence_extent=extent,
                first_cycle_index=checked[0][0], last_cycle_index=checked[-1][0],
                cap_deg=dict(physical_clearance=physical, maximum_displacement=displacement),
                mixtures=results, formula='initial_q + weight * (raw_target - initial_q)',
                physical_clearance_independently_verified=False, model_replay_verified_by_this_tool=False,
                closed_loop_prediction=False, standing_prediction=False, timestamps_predict_future_motion=False,
                hardware_accessed=False, model_executed=False, approvals_created=False, output_allowed=False)


def _pairs(items):
    value = {}
    for key, item in items:
        if key in value:
            raise ValueError('Duplicate JSON key: ' + key)
        value[key] = item
    return value


def read_json(path, expected_sha256=None):
    path = Path(path)
    if any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError('Symlink input forbidden')
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_size > 128 * 1024 * 1024:
        raise ValueError('Regular input at most128MiB required')
    raw = path.read_bytes()
    if len(raw) > 128 * 1024 * 1024:
        raise ValueError('Input grew beyond128MiB')
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and digest != _digest(expected_sha256, 'expected input'):
        raise ValueError('Input SHA256 differs')
    return json.loads(raw, object_pairs_hook=_pairs,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON'))), digest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True)
    parser.add_argument('--targets', required=True)
    parser.add_argument('--expected-report-sha256')
    parser.add_argument('--expected-targets-sha256')
    parser.add_argument('--physical-clearance-deg', type=float, required=True)
    parser.add_argument('--max-displacement-deg', type=float, required=True)
    args = parser.parse_args(argv)
    try:
        report, report_sha = read_json(args.report, args.expected_report_sha256)
        targets, targets_sha = read_json(args.targets, args.expected_targets_sha256)
        result = analyze(report, targets, report_sha256=report_sha, targets_sha256=targets_sha,
                         physical_clearance_deg=args.physical_clearance_deg,
                         max_displacement_deg=args.max_displacement_deg)
    except (ValueError, OSError) as error:
        print(json.dumps(dict(status='INVALID_SAVED_TARGET_ANALYSIS', error=str(error), output_allowed=False)))
        return 2
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
