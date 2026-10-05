"""File-only velocity integral versus reported position consistency diagnostics.

Revalidate the six saved artifacts with the frozen PVP comparator. Positions
interpolated at host velocity timestamps are explicitly proxies. Numerical
integration, fits and timestamp sensitivity are not physical error bounds:
unobserved motion, aliasing and unknown sensor acquisition time remain possible.
No velocity is changed, filtered or replaced. No calibration/gate is approved.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import analyze_stationary_velocity_probe as audit

SCHEMA = 'singularitydog.velocity-position-consistency.v1'
COMPARATOR_SHA256 = '2d3fd9d1619aa8b79fd4fb3d9cfbd91b95247cf9e94e577933f6e6d057980cbd'
TIME_REFERENCES = ('raw_frame', 'decoded_receipt', 'request', 'request_raw_midpoint')
# Unit/reduction hypotheses only; the declared protocol remains rad and rad/s.
CANDIDATE_FACTORS = {'declared_rad_s': 1., 'reversed_sign': -1., 'divide_by_7_75': 1/7.75,
    'multiply_by_7_75': 7.75, 'degrees_to_radians': math.pi/180,
    'rpm_to_rad_s': 2*math.pi/60, 'radians_to_degrees': 180/math.pi}
FALSE_FLAGS = {**audit.FLAGS, 'hardware_opened': False, 'velocity_ground_truth': False,
    'software_bug_confirmed': False, 'scale_fit_applied': False, 'sign_fit_applied': False,
    'automatic_approval': False, 'integration_error_bound_verified': False,
    'absolute_origin_uncertainty_quantified': False, 'sensor_time_alignment_verified': False}


def correlation(x, y):
    audit.need(len(x) == len(y), 'Paired correlation length')
    if len(x) < 2:
        return None
    mx, my = math.fsum(x)/len(x), math.fsum(y)/len(y)
    numerator = math.fsum((a-mx)*(b-my) for a, b in zip(x, y))
    denominator = math.sqrt(math.fsum((a-mx)**2 for a in x)*math.fsum((b-my)**2 for b in y))
    return numerator/denominator if denominator else None


def fit_scale(x, y):
    """Zero-intercept diagnostic fit; static quantized data cannot approve scale."""
    audit.need(len(x) == len(y), 'Paired scale length')
    denominator = math.fsum(v*v for v in x)
    factor = math.fsum(a*b for a, b in zip(x, y))/denominator if denominator else None
    return {'pairs': len(x), 'factor': factor, 'Pearson_r': correlation(x, y),
            'residual_rad': audit.statistics([factor*a-b for a, b in zip(x, y)]) if factor is not None else None,
            'identification_status': 'UNVERIFIED_HOST_PROXY_FIT', 'applied': False, 'approved': False}


def receipt_time(timing, reference):
    if reference == 'raw_frame': return timing['raw_frame_received_monotonic_ns']
    if reference == 'decoded_receipt': return timing['receipt_monotonic_ns']
    if reference == 'request': return timing['request_monotonic_ns']
    audit.need(reference == 'request_raw_midpoint', 'Unknown time reference')
    return (timing['request_monotonic_ns']+timing['raw_frame_received_monotonic_ns'])//2


def series_from_report(report, run, reference):
    """Use only complete PVPs; every discarded slot remains in caller coverage."""
    by_sequence = {(r['bus'], r['sequence']): r for r in run['all_reply_timing_rows']}
    result = []
    for row in report['samples']:
        if not row['complete']: continue
        before, velocity, after = [row[k] for k in audit.KEYS]
        timings = [by_sequence[(report['plan']['selected_bus'], r['sequence'])] for r in (before, velocity, after)]
        tb, tv, ta = [receipt_time(r, reference) for r in timings]
        audit.need(tb < tv < ta, 'Host PVP timing does not bracket velocity')
        fraction = (tv-tb)/(ta-tb)
        proxy = before['value']+(after['value']-before['value'])*fraction
        result.append({'slot_index': row['slot_index'], 'time_ns': tv,
            'velocity_rad_s': velocity['value'], 'position_proxy_rad': proxy,
            'position_before_rad': before['value'], 'position_after_rad': after['value'],
            'host_position_bracket_ns': ta-tb, 'host_interpolation_fraction': fraction,
            'sensor_sample_time_verified': False, 'velocity_ground_truth': False})
    return result


def analyze_series(series):
    audit.need(type(series) is list and len(series) <= 500, 'Bounded series required')
    for row in series:
        audit.fields(row, {'slot_index', 'time_ns', 'velocity_rad_s', 'position_proxy_rad',
                          'position_before_rad', 'position_after_rad', 'host_position_bracket_ns',
                          'host_interpolation_fraction', 'sensor_sample_time_verified', 'velocity_ground_truth'})
        audit.timestamp(row['time_ns'])
        audit.need(type(row['slot_index']) is int and 0 <= row['slot_index'] < 500, 'Invalid slot index')
        for key in ('velocity_rad_s', 'position_proxy_rad', 'position_before_rad', 'position_after_rad', 'host_interpolation_fraction'):
            audit.need(type(row[key]) in (int, float) and math.isfinite(row[key]), 'Nonfinite series value')
        audit.need(type(row['host_position_bracket_ns']) is int and row['host_position_bracket_ns'] > 0
                   and 0 < row['host_interpolation_fraction'] < 1, 'Invalid host bracket')
        audit.exact(row['sensor_sample_time_verified'], False, 'Sensor time scope')
        audit.exact(row['velocity_ground_truth'], False, 'Ground truth scope')
    intervals = []
    cumulative = {'left': [], 'right': [], 'trapezoid': []}; totals = {k: 0. for k in cumulative}
    for before, after in zip(series, series[1:]):
        audit.need(after['time_ns'] > before['time_ns'] and after['slot_index'] > before['slot_index'], 'Reordered/repeated series')
        dt = (after['time_ns']-before['time_ns'])/1e9
        estimates = {'left': before['velocity_rad_s']*dt, 'right': after['velocity_rad_s']*dt,
                     'trapezoid': (before['velocity_rad_s']+after['velocity_rad_s'])*.5*dt}
        delta = after['position_proxy_rad']-before['position_proxy_rad']
        item = {'from_slot': before['slot_index'], 'to_slot': after['slot_index'], 'host_interval_s': dt,
                'missing_requested_slots_between': after['slot_index']-before['slot_index']-1,
                'host_proxy_position_change_rad': delta, 'sampled_velocity_integral_rad': estimates,
                'trapezoid_minus_host_proxy_change_rad': estimates['trapezoid']-delta}
        intervals.append(item)
        for key, value in estimates.items():
            totals[key] = math.fsum((totals[key], value)); cumulative[key].append(totals[key])
    q0 = series[0]['position_proxy_rad'] if series else None
    q_changes = [r['position_proxy_rad']-q0 for r in series[1:]]
    x = [r['sampled_velocity_integral_rad']['trapezoid'] for r in intervals]
    y = [r['host_proxy_position_change_rad'] for r in intervals]
    candidates = {}
    for name, factor in CANDIDATE_FACTORS.items():
        candidates[name] = {'factor': factor,
            'interval_residual_rad': audit.statistics([factor*a-b for a, b in zip(x, y)]),
            'cumulative_residual_rad': audit.statistics([factor*a-b for a, b in zip(cumulative['trapezoid'], q_changes)]),
            'final_residual_rad': factor*totals['trapezoid']-(q_changes[-1] if q_changes else 0.) if intervals else None,
            'candidate_only': True, 'applied': False, 'approved': False}
    span = (series[-1]['time_ns']-series[0]['time_ns'])/1e9 if len(series) >= 2 else None
    endpoint_range = None
    if len(series) >= 2:
        first, last = series[0], series[-1]
        endpoint_range = [min(last['position_before_rad'], last['position_after_rad'])-max(first['position_before_rad'], first['position_after_rad']),
                          max(last['position_before_rad'], last['position_after_rad'])-min(first['position_before_rad'], first['position_after_rad'])]
    return {'complete_sample_count': len(series), 'interval_count': len(intervals),
        'host_span_s': span, 'net_sampled_velocity_integral_rad': totals if intervals else None,
        'net_host_proxy_position_change_rad': q_changes[-1] if q_changes else None,
        'observed_endpoint_position_difference_range_rad': endpoint_range,
        'host_proxy_position_change_rad': audit.statistics(y),
        'trapezoid_velocity_interval_integral_rad': audit.statistics(x),
        'interval_velocity_integral_vs_host_proxy_position_Pearson_r': correlation(x, y),
        'interval_scale_fit': fit_scale(x, y), 'cumulative_scale_fit': fit_scale(cumulative['trapezoid'], q_changes),
        'unmeasured_velocity_bias_that_would_cancel_final_residual_rad_s':
            (totals['trapezoid']-q_changes[-1])/span if span else None,
        'bias_fit_applied': False, 'bias_identification_approved': False,
        'intervals_crossing_missing_slots': sum(r['missing_requested_slots_between'] > 0 for r in intervals),
        'intervals': intervals, 'scale_and_sign_candidates': candidates,
        'series': series, 'sensor_sample_time_verified': False, 'velocity_ground_truth': False,
        'integration_error_bound_verified': False,
        'integration_scope': 'Quadrature of sampled reported velocities at host timestamps. Unknown values between reads and missing slots are not reconstructed.'}


def analyze(comparison_path, *, expected_boot_id, motor_id):
    raw, binding = audit.read_file(comparison_path)
    original = audit.strict_json(raw)
    audit.exact(original['schema'], audit.ANALYSIS_SCHEMA, 'Comparison schema')
    comparator_hash = hashlib.sha256(Path(audit.__file__).read_bytes()).hexdigest()
    audit.exact(comparator_hash, COMPARATOR_SHA256, 'Frozen comparator source')
    audit.exact(original['analysis_source_sha256'], COMPARATOR_SHA256, 'Comparison provenance')
    audit.exact(original['motor_id'], motor_id, 'Selected motor')
    paths, reports = [], {}
    for period in (20, 200):
        bindings = original['runs'][str(period)]['artifact_bindings']
        selected = {}
        for key in ('report', 'receipt', 'trace'):
            contents, observed = audit.read_file(bindings[key]['path'], allow_empty=key == 'trace')
            audit.exact(observed, bindings[key], 'Comparison artifact binding')
            selected[key] = observed['path']
            if key == 'report': reports[str(period)] = audit.strict_json(contents)
        paths.append(selected)
    for key in ('expected_uid_file_binding', 'expected_source_manifest_binding'):
        _, observed = audit.read_file(original[key]['path'], 16384)
        audit.exact(observed, original[key], 'Expected input binding')
    recomputed = audit.compare(paths, expected_uids=original['expected_uid_file_binding']['path'],
        expected_sources=original['expected_source_manifest_binding']['path'], expected_boot_id=expected_boot_id, motor_id=motor_id)
    audit.exact(recomputed, original, 'Full frozen comparison replay')
    runs = {}
    for period in ('20', '200'):
        run, report = original['runs'][period], reports[period]
        analyses = {reference: analyze_series(series_from_report(report, run, reference)) for reference in TIME_REFERENCES}
        nets = [r['net_sampled_velocity_integral_rad']['trapezoid'] for r in analyses.values()
                if r['net_sampled_velocity_integral_rad'] is not None]
        runs[period] = {'requested_slots': run['requested_slot_count'], 'complete_triplets': run['complete_triplets'],
            'incomplete_triplets': run['incomplete_triplets'], 'dropped_slots': run['dropped_slots'],
            'unacquired_slots': run['unacquired_slots'], 'data_coverage': run['data_coverage'],
            'recording_complete': run['recording_complete'], 'final_report_binding_verified': run['final_report_binding_verified'],
            'time_references': analyses, 'trapezoid_net_integral_host_time_reference_sensitivity_rad': audit.statistics(nets),
            'host_time_sensitivity_is_not_sensor_latency_bound': True}
    return {'schema': SCHEMA, 'status': 'HOST_CONSISTENCY_DESCRIBED_REVIEW_REQUIRED'
            if original['status'] == 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED' else 'INCOMPLETE_CONSISTENCY_RECORDS',
        **FALSE_FLAGS, 'comparison_binding': binding, 'comparator_sha256': comparator_hash,
        'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'motor_id': motor_id,
        'runs': runs, 'software_cause_status': 'NO_CAUSE_CERTIFIED_BY_HOST_CONSISTENCY_DIAGNOSTICS',
        'limitations': ['Host position interpolation assumes a linear reported trajectory between distinct reads; it is not measured sensor-time position.',
            'Trapezoid/left/right integration assumes values between sparse sampled velocities. Alias and unobserved motion can cause disagreement without a codec bug.',
            'Missing slots stay missing; intervals crossing them are marked. No held value, zero fill, smoothing or threshold change is applied.',
            'Fits and unit/reduction/sign factors are candidates only. Static quantized position data cannot establish physical velocity scale or sign.',
            'A constant position offset cancels from position changes; changing the model sign reverses both q and velocity and cannot remove absolute residual or RMS.',
            'Four host timestamp choices test recorded latency sensitivity only. None constrains internal sensor acquisition time.',
            'Any apparent drift/bias cancellation is a fitted description, not authorization to subtract a velocity bias.',
            'No STOP, physical static state, accuracy, calibration, motor power epoch or output approval follows.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', required=True)
    parser.add_argument('--expected-boot-id', required=True)
    parser.add_argument('--id', required=True, type=int, dest='motor_id')
    args = parser.parse_args(argv)
    try:
        result = analyze(args.comparison, expected_boot_id=args.expected_boot_id, motor_id=args.motor_id)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(json.dumps({'schema': SCHEMA, 'status': 'INVALID_ARTIFACTS', 'error': str(error), **FALSE_FLAGS}, allow_nan=False))
        return 1
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['status'] == 'HOST_CONSISTENCY_DESCRIBED_REVIEW_REQUIRED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
