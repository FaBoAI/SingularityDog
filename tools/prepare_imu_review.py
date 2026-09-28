#!/usr/bin/env python3
"""Recompute saved IMU evidence into a private, UNREVIEWED review fragment.

File-only: no capture process, I2C, CAN, SSH, calibration application or approval.
Use --manifest, --audit, --mount, --bias and --output NEW_PRIVATE_JSON. All raw
capture files must remain available at the paths used by the saved manifest and
audit. An old schema-1 audit without the newly added held-out gyro maximum is
accepted; that maximum is always recomputed from the original samples.

The output's `imu` has the hardware_review.imu keys. A reviewer must inspect the
pinned originals, supply independently observed directions and an external
gravity reference with synchronization/uncertainty, and justify the raw norm
range. After that explicit review, merge `imu` and `source_captures` into the
complete hardware review, bind its other artifacts/settings, and use the normal
profile validator. This fragment itself cannot authorize a profile or motor I/O.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'runtime'))
from singularitydog_hw import imu_commissioning_audit as audit
from singularitydog_hw import imu_fixed_mount_baseline as baseline

SCHEMA = 'singularitydog.imu-review-preparation.v1'
MAX_FIELD = 'gyro_body_corrected_norm_max_rad_s'
PHYSICAL_KEYS = ('right_handed_mount_physically_verified', 'nose_up_verified',
                 'left_up_verified', 'yaw_left_verified',
                 'gyro_bias_independent_stationary_validation', 'gravity_direction_verified')


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _same(a, b, label):
    # JSON true/false must not compare equal to numeric 1/0.
    if _canonical(a) != _canonical(b):
        raise ValueError(label + ' does not match recomputed capture evidence')


def _read_file(path):
    path = Path(path).expanduser().absolute()
    if path.is_symlink() or not path.is_file():
        raise ValueError('Regular source file required: ' + str(path))
    path = path.resolve()
    return path, path.read_bytes()


def _capture_directories(manifest, parent):
    stationary = manifest.get('stationary', {})
    movements = manifest.get('movements', [])
    if type(stationary) is not dict or type(movements) is not list:
        raise ValueError('Invalid capture manifest')
    entries = [('static_a', stationary.get('a')), ('static_b', stationary.get('b'))]
    for movement in movements:
        if type(movement) is not dict or movement.get('movement') not in audit.MOVEMENTS:
            raise ValueError('Invalid motion capture manifest')
        entries.append((movement['movement'], movement.get('capture')))
    result = {}
    for label, value in entries:
        if label in result or not isinstance(value, str) or not value.strip():
            raise ValueError('Missing/duplicate capture path')
        path = Path(value).expanduser()
        result[label] = (path if path.is_absolute() else parent/path).resolve()
    return result


def prepare(manifest_path, audit_path, mount_path, bias_path, output):
    """Strictly bind saved evidence; return data only after exclusive private save."""
    out = Path(output).expanduser().absolute()
    if out.exists() or out.is_symlink():
        raise FileExistsError('Use a new private output path')
    out = out.resolve()
    if any((p/'.git').exists() for p in (out, *out.parents)):
        raise ValueError('Keep IMU review evidence outside Git')

    pinned = {}
    def pin(name, path, *, parse=True):
        source, raw = _read_file(path)
        pinned[name] = (source, raw)
        return baseline._json(raw) if parse else raw
    manifest = pin('manifest', manifest_path)
    saved = pin('audit', audit_path)
    mount = pin('mount', mount_path)
    bias = pin('bias', bias_path)
    directories = _capture_directories(manifest, pinned['manifest'][0].parent)
    for label, directory in directories.items():
        # Even a malformed, explicitly unresolved motion is pinned as bytes.
        pin(label + '.summary', directory/'summary.json', parse=False)
        pin(label + '.events', directory/'events.jsonl', parse=False)
    expected_hash = hashlib.sha256(pinned['manifest'][1]).hexdigest()
    if saved.get('manifest_sha256') != expected_hash:
        raise ValueError('Saved audit manifest hash mismatch')
    recomputed = audit.audit_manifest(manifest, root=pinned['manifest'][0].parent)
    comparable = copy.deepcopy(saved)
    comparable.pop('manifest_sha256', None)
    comparable.pop('generated_at_utc', None)
    expected = copy.deepcopy(recomputed)
    # Compatible with immutable older schema-1 reports. No other field may be
    # omitted or replaced, including all source hashes and movement failures.
    if (type(comparable.get('heldout_body_diagnostic')) is dict and
            MAX_FIELD not in comparable['heldout_body_diagnostic']):
        expected['heldout_body_diagnostic'].pop(MAX_FIELD, None)
    _same(comparable, expected, 'Saved audit')
    _same(audit.validate_imu_mount_candidate(mount), recomputed['mount_candidate'], 'Mount candidate')
    _same(bias, recomputed['stationary'], 'Bias candidate')
    if recomputed['checks']['gyro_bias_candidate'] is not True:
        raise ValueError('An eligible, unapproved static A/B gyro candidate is required')
    # Recheck exact bytes after all rereads by baseline/audit. This prevents a
    # concurrent capture update from being packaged as the initial snapshot.
    for source, raw in pinned.values():
        check_source, check_raw = _read_file(source)
        if check_source != source or check_raw != raw:
            raise ValueError('Source changed during preparation: ' + str(source))

    captures = recomputed['stationary']['captures']
    norm_min = min(captures[k]['accel_norm_min_m_s2'] for k in ('a', 'b'))
    norm_max = max(captures[k]['accel_norm_max_m_s2'] for k in ('a', 'b'))
    gyro_max = recomputed['heldout_body_diagnostic'][MAX_FIELD]
    references = {name: {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}
                  for name, (path, raw) in pinned.items()}
    # The profile's source_captures accepts JSON objects, not raw JSONL streams.
    # All exact raw streams (and malformed unresolved captures) remain pinned
    # in references. Include this preparation document's printed SHA alongside
    # the JSON sources when merging, so those raw references remain attached.
    json_source_names = ['manifest', 'audit', 'mount', 'bias']
    for name, (_, raw) in pinned.items():
        if name.endswith('.summary'):
            try:
                baseline._json(raw)
            except ValueError:
                continue
            json_source_names.append(name)
    imu = {key: None for key in PHYSICAL_KEYS}
    imu.update(gravity_direction_max_error_rad=None,
               corrected_static_gyro_max_rad_s=gyro_max,
               raw_gravity_norm_min_m_s2=norm_min, raw_gravity_norm_max_m_s2=norm_max,
               norm_deviation_rationale='')
    result = {
        'schema': SCHEMA, 'status': 'UNREVIEWED', 'approved_for_runtime': False,
        'dependency_eligible': False, 'hardware_opened': False, 'automatically_applied': False,
        'references': references, 'source_captures': [references[name] for name in json_source_names],
        'review': {'decision': 'UNREVIEWED', 'reviewed_by': None, 'reviewed_at': None},
        'imu': imu,
        'external_gravity_reference': {
            'reference_capture': {'path': None, 'sha256': None},
            'method': None, 'body_gravity_unit_vector': None,
            'reference_direction_uncertainty_rad': None,
            'synchronization': {'capture_label': None, 'start_monotonic_ns': None,
                                'end_monotonic_ns': None, 'uncertainty_ms': None},
            'measured_direction_error_rad': None, 'combined_error_bound_rad': None,
        },
        'recomputed_evidence': {
            'checks': recomputed['checks'], 'unresolved': recomputed['unresolved'],
            'movements': recomputed['movements'],
            'heldout_body_diagnostic': recomputed['heldout_body_diagnostic'],
            'stationary_diagnostic_gates': recomputed['stationary']['diagnostic_gates'],
            'stationary_provenance': recomputed['stationary']['provenance'],
            'static_norm_deviation_percent': {k: captures[k]['gravity_norm_deviation_percent'] for k in ('a', 'b')},
            'temperature_c': {k: captures[k]['temperature_c'] for k in ('a', 'b')},
            'numeric_review_contract': {'heldout_gyro_max_at_most_0_02_rad_s': gyro_max <= .02,
                                        'raw_norm_within_8_8_to_11_2_m_s2': 8.8 <= norm_min <= norm_max <= 11.2},
            'gyro_max_definition': 'Maximum over individual B samples of norm(R * (gyro_sensor - bias_A)); not norm of the mean.',
            'raw_norm_definition': 'Minimum/maximum of original, unscaled acceleration norms over both static A and B.',
        },
        'merge_instructions': [
            'Review pinned original records; diagnostic direction flags are not physical approvals.',
            'Complete independent gravity reference, time association and uncertainty before entering the <=3 degree error bound.',
            'Supply each physical confirmation and raw-norm rationale only from actual reviewed evidence; do not fit acceleration bias/scale from one pose.',
            'After explicit review, merge imu plus source_captures into the complete supported-policy hardware review; bind all profile artifacts, assembly, UIDs and exact settings.',
            'Also add this preparation JSON as a source_captures path/SHA reference using the printed output_sha256; raw JSONL is pinned inside references, not passed as a JSON-object artifact.',
            'Run the normal profile validator. This fragment alone is never an approved hardware review; other angle, Type2, watchdog and timing evidence remains required.',
        ],
        'limitations': [
            'Recorded operator statements and file hashes cannot prove physical stationarity, sensor identity or mounting alignment.',
            'No absolute heading or accelerometer bias/scale is identified; a static gravity candidate is not valid during arbitrary linear acceleration.',
            'No runtime norm limit is changed. The exact selected runtime bounds and norm deviation disposition require review.',
        ],
    }
    out.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd = os.open(out, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('manifest', 'audit', 'mount', 'bias', 'output'):
        parser.add_argument('--' + key, required=True)
    args = parser.parse_args(argv)
    try:
        result = prepare(args.manifest, args.audit, args.mount, args.bias, args.output)
    except (OSError, ValueError) as error:
        parser.exit(2, 'IMU review preparation rejected: %s\n' % error)
    print(json.dumps({'status': result['status'], 'output': str(Path(args.output).expanduser().resolve()),
                      'output_sha256': hashlib.sha256(Path(args.output).expanduser().read_bytes()).hexdigest(),
                      'approved_for_runtime': False, 'hardware_opened': False, 'dependency_eligible': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
