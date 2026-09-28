#!/usr/bin/env python3
"""File-only comparison of a fixed-mount recording with an external level.

Keeps measured direction error separate from unspecified level accuracy. Never
fits accelerometer scale/bias or writes a runtime approval.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime'))
from singularitydog_hw.imu_fixed_mount_baseline import _load_capture
from singularitydog_hw.imu_commissioning_audit import rotate
from singularitydog_hw.policy_shadow import validate_imu_mount_candidate


def direction_error_deg(accel, rotation):
    if len(accel) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in accel):
        raise ValueError('Finite acceleration vector required')
    body = rotate(rotation, accel)
    norm = math.hypot(*body)
    if norm < 1e-9:
        raise ValueError('Zero acceleration has no direction')
    return math.degrees(math.acos(max(-1., min(1., body[2] / norm))))


def review(capture, mount, bias, *, method, operator_confirmed, reference_uncertainty_deg=None):
    if not isinstance(method, str) or not method.strip() or type(operator_confirmed) is not bool:
        raise ValueError('Explicit reference method and operator statement required')
    if reference_uncertainty_deg is not None and (type(reference_uncertainty_deg) not in (int, float)
            or not math.isfinite(reference_uncertainty_deg) or not 0 <= reference_uncertainty_deg <= 180):
        raise ValueError('Invalid external reference uncertainty')
    metadata, rows, stats, provenance = _load_capture(capture)
    paths = {'mount': Path(mount), 'bias': Path(bias)}
    raw = {k: p.read_bytes() for k, p in paths.items()}
    rotation = validate_imu_mount_candidate(json.loads(raw['mount']))['R_body_from_sensor']
    bias_value = json.loads(raw['bias'])['gyro_bias_candidate_rad_s']
    if len(bias_value) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in bias_value):
        raise ValueError('Invalid saved gyro bias')
    angles = [direction_error_deg(r['accel_m_s2'], rotation) for r in rows]
    gyro = [math.hypot(*rotate(rotation, [v-b for v, b in zip(r['gyro_rad_s'], bias_value)])) for r in rows]
    worst = max(angles)
    combined = None if reference_uncertainty_deg is None else worst + reference_uncertainty_deg
    return {
        'schema': 'singularitydog.imu-external-level-review.v1',
        'status': 'EXTERNAL_LEVEL_COMPARISON_RECORDED',
        'hardware_opened': False, 'approved_for_runtime': False,
        'external_reference': {'method': method, 'operator_confirmed_level_and_stationary': operator_confirmed,
            'reference_body_gravity_unit': [0, 0, -1],
            'reference_uncertainty_deg': reference_uncertainty_deg,
            'capture_interval_ns': provenance['monotonic_interval_ns']},
        'capture': provenance,
        'sources': {k: {'path': str(paths[k].resolve()), 'sha256': hashlib.sha256(v).hexdigest()} for k, v in raw.items()},
        'R_body_from_sensor': rotation, 'gyro_bias_sensor_rad_s': bias_value,
        'samples': len(rows), 'summary': stats,
        'direction_mean_vector_error_deg': direction_error_deg(stats['accel_mean_m_s2'], rotation),
        'direction_sample_error_max_deg': worst,
        'direction_sample_error_p95_deg': sorted(angles)[math.ceil(.95*len(angles))-1],
        'combined_error_bound_deg': combined,
        'combined_bound_within_3deg': operator_confirmed and combined is not None and combined <= 3,
        'corrected_gyro_norm_max_rad_s': max(gyro),
        'corrected_gyro_norm_mean_rad_s': statistics.fmean(gyro),
        'restore_status': metadata['restore_status'],
        'limitations': ['Level accuracy is not inferred from a centered bubble.',
            'One level pose does not identify accelerometer bias, scale or yaw.',
            'No normalization scale is fitted; raw acceleration and its norm remain unchanged.',
            'Measured errors are relative to the operator-established level, not certified absolute accuracy.'],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('capture', 'mount', 'bias', 'method', 'output'):
        p.add_argument('--'+key, required=True)
    p.add_argument('--operator-level-confirmed', action='store_true')
    p.add_argument('--reference-uncertainty-deg', type=float)
    a = p.parse_args()
    out = Path(a.output).expanduser().resolve()
    if any((parent/'.git').exists() for parent in (out, *out.parents)):
        p.error('Keep raw capture review outside Git')
    result = review(a.capture, a.mount, a.bias, method=a.method,
        operator_confirmed=a.operator_level_confirmed, reference_uncertainty_deg=a.reference_uncertainty_deg)
    with out.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('status', 'samples', 'direction_mean_vector_error_deg',
        'direction_sample_error_max_deg', 'combined_error_bound_deg', 'corrected_gyro_norm_max_rad_s')}, indent=2))


if __name__ == '__main__':
    main()
