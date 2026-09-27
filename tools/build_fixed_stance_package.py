"""Freeze a disabled-only supported fixed-stance candidate and its evidence.

Inputs are one current-boot 12-axis hold, one physically placed supported
stance capture and a separately reviewed swept path. The package never
enables motors: its runner source retains LIVE_OUTPUT_ENABLED=False and the
review retains supported_transition_authorized=False. Hashing records exact
input/source bytes; a human physical clearance assertion is not a sensor proof.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from build_fullbody_step2_disabled import validate_hold

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'runtime' / 'singularitydog_hw'
IDS = tuple(range(1, 13))
CAPTURE_SCHEMA = 'singularitydog.supported-fixed-stance-capture.v2'
FLOOR_STANCE_CLASS = 'four_foot_floor_supported_stance'
ROUTE_SCHEMA = 'singularitydog.fixed-stance-physical-route-review.v1'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pairs(rows):
    value = {}
    for key, item in rows:
        if key in value:
            raise ValueError('Duplicate JSON key: ' + key)
        value[key] = item
    return value


def read_json(path: Path):
    if not path.is_file() or path.is_symlink():
        raise ValueError('Required evidence file missing or symlink: ' + str(path))
    return json.loads(path.read_text(), object_pairs_hook=_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError('Nonfinite JSON value: ' + value)))


def write_json(path: Path, value):
    data = (json.dumps(value, indent=2, allow_nan=False) + '\n').encode()
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'wb') as stream:
        stream.write(data)


def require(condition, text):
    if not condition:
        raise ValueError(text)


def _ids(value, label):
    require(type(value) is dict and set(value) == {str(i) for i in IDS},
            label + ' requires exactly twelve string motor IDs')
    return value


def _is_digest(value):
    return (type(value) is str and len(value) == 64
            and all(char in '0123456789abcdef' for char in value))


def private_output_path(value: Path) -> Path:
    """Keep private identities and raw poses out of every Git checkout."""
    raw = Path(value).expanduser()
    require(not raw.is_symlink(), 'Package output must not be a symlink')
    output = raw.resolve()
    require(not output.exists() and output.parent.is_dir(), 'Use a fresh package path')
    require(not any(parent.name == '.git' or (parent / '.git').exists()
                    for parent in output.parents),
            'Private UID and pose package must remain outside Git')
    return output


def build(hold_path: Path, capture_path: Path, route_path: Path,
          output: Path, *, expected_boot_id: str) -> dict:
    output = private_output_path(output)
    require(type(expected_boot_id) is str and bool(expected_boot_id.strip()),
            'Expected Jetson boot ID required')
    hold, capture, route = (read_json(path) for path in
                            (hold_path, capture_path, route_path))
    result = hold.get('result', {})
    uids = _ids(result.get('review', {}).get('motor_uids'), 'hold identities')
    starts = validate_hold(hold, sha(hold_path), expected_boot_id, uids)
    require(capture.get('schema') == CAPTURE_SCHEMA
            and capture.get('boot_id') == expected_boot_id
            and capture.get('motor_uids') == uids
            and capture.get('read_only') is True
            and capture.get('motor_enable_sent') is False
            and capture.get('supported_pose_placed_by_operator') is True
            and capture.get('simultaneous_physical_stance_verified') is True
            and capture.get('stand_removed') is False
            and capture.get('pose_class') == FLOOR_STANCE_CLASS
            and capture.get('foot_support_kind') == 'floor'
            and type(capture.get('foot_support_height_cm')) in (int, float)
            and capture['foot_support_height_cm'] == 0
            and _is_digest(capture.get('physical_pose_review_sha256'))
            and capture.get('sampling_stability_heuristic_passed') is True
            and capture.get('output_allowed') is False
            and capture.get('approved_for_runtime') is False
            and type(capture.get('operator_note')) is str
            and bool(capture['operator_note'].strip())
            and type(capture.get('evidence_reference')) is str
            and bool(capture['evidence_reference'].strip()),
            'Supported stance capture is not a current-boot, read-only physical pose')
    target = _ids(capture.get('raw_rad_by_id'), 'supported stance capture')
    require(route.get('schema') == ROUTE_SCHEMA
            and route.get('boot_id') == expected_boot_id
            and route.get('motor_uids') == uids
            and route.get('hold_summary_sha256') == sha(hold_path)
            and route.get('stance_capture_sha256') == sha(capture_path)
            and route.get('start_raw_rad_by_id') == starts
            and route.get('fixed_stance_raw_rad_by_id') == target
            and route.get('old_d17_target_reused') is False
            and route.get('clearance_reviewed_start_envelope_deg') == 3.
            and type(route.get('reviewed_raw_corridor_by_id')) is dict
            and set(route['reviewed_raw_corridor_by_id']) == {str(i) for i in IDS}
            and type(route.get('raw_corridor_physical_source_note')) is str
            and bool(route['raw_corridor_physical_source_note'].strip())
            and route.get('whole_route_physically_reviewed') is True
            and route.get('support_and_foot_clearance_verified') is True
            and route.get('attended_40v_cutoff_ready') is True,
            'Physical route is not pinned to the same-boot hold and stance capture')
    waypoints = route.get('waypoints_raw_rad_by_id')
    segments = route.get('segments')
    require(type(waypoints) is list and type(segments) is list
            and len(waypoints) == len(segments) and 1 <= len(waypoints) <= 12,
            'Require one to twelve separately reviewed segments')
    previous = starts
    for index, (waypoint, segment) in enumerate(zip(waypoints, segments)):
        _ids(waypoint, f'waypoint {index}')
        require(type(segment) is dict
                and segment.get('index') == index
                and segment.get('from_raw_rad_by_id') == previous
                and segment.get('to_raw_rad_by_id') == waypoint
                and segment.get('swept_clearance_verified') is True
                and segment.get('all_joint_limits_verified') is True
                and segment.get('front_upper_leg_carbon_clamp_clear') is True
                and type(segment.get('operator_note')) is str
                and bool(segment['operator_note'].strip()),
                f'Segment {index} lacks physical/joint clearance review')
        previous = waypoint
    require(waypoints[-1] == target, 'Final waypoint is not the physical stance capture')

    # Calculate and validate the entire plan before creating an output folder.
    source_files = sorted(SOURCE.glob('*.py'))
    require(source_files and all(path.is_file() and not path.is_symlink()
                                 for path in source_files), 'Source files are incomplete or symlinked')
    source_hashes = {path.name: sha(path) for path in source_files}
    source_digest = hashlib.sha256(json.dumps(source_hashes, sort_keys=True).encode()).hexdigest()
    review = {
        'schema': 'singularitydog.fixed-stance-candidate.v1',
        'boot_id': expected_boot_id, 'motor_uids': uids,
        'stance_capture_boot_id': expected_boot_id,
        'stance_capture_motor_uids': uids,
        'stance_capture_raw_rad_by_id': target,
        'hold_summary_sha256': sha(hold_path),
        'stance_capture_sha256': sha(capture_path),
        'physical_route_review_sha256': sha(route_path),
        'segment_clearance_sha256': [hashlib.sha256(json.dumps(segment, sort_keys=True,
            allow_nan=False).encode()).hexdigest() for segment in segments],
        'clearance_reviewed_start_envelope_deg': 3.,
        'reviewed_raw_corridor_by_id': route['reviewed_raw_corridor_by_id'],
        'raw_corridor_physical_source_note': route['raw_corridor_physical_source_note'],
        'hold_status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
        'hold_stop_confirmed': True,
        'old_d17_target_reused': False,
        'learned_policy_allowed': False,
        'automatic_retry_allowed': False,
        'start_raw_rad_by_id': starts,
        'waypoints_raw_rad_by_id': waypoints,
        'fixed_stance_raw_rad_by_id': target,
        'source_files_verified': True,
        'frozen_package_sha256': source_digest,
        # The numeric raw corridor and segment records are checked for shape
        # and internal consistency. They are still operator assertions: no
        # measured per-sample mechanical limits or swept 3-D clearance proof
        # is available from these files alone. Keep the live runtime gates
        # closed until independent evidence is reviewed and frozen.
        'joint_limits_all_samples_verified': False,
        'physical_route_all_samples_verified': False,
        'operator_asserted_joint_limits_on_segments': True,
        'operator_asserted_swept_clearance_on_segments': True,
        'supported_transition_authorized': False,
        'same_boot_12_axis_hold_passed': True,
        'same_boot_physical_stance_capture_verified': True,
        'all_segment_sweeps_physically_reviewed': True,
        'front_upper_leg_carbon_clamp_clearance_verified': True,
        'support_and_foot_clearance_verified': True,
        'attended_40v_cutoff_ready': True,
    }
    from sys import path as sys_path
    sys_path.insert(0, str(ROOT / 'runtime'))
    from singularitydog_hw.fixed_stance_candidate import prepare_candidate
    candidate = prepare_candidate(review, current_boot_id=expected_boot_id,
                                  current_motor_uids=uids,
                                  fresh_raw_rad_by_id=starts)
    # File-only package: include exact source files and input evidence bytes.
    output.mkdir(mode=0o700)
    evidence_out, source_out = output / 'evidence', output / 'source'
    evidence_out.mkdir(mode=0o700)
    source_out.mkdir(mode=0o700)
    for name, path in (('hold-summary.json', hold_path),
                       ('stance-capture.json', capture_path),
                       ('physical-route.json', route_path)):
        shutil.copy2(path, evidence_out / name)
        (evidence_out / name).chmod(0o600)
    for path in source_files:
        shutil.copy2(path, source_out / path.name)
    require({path.name: sha(path) for path in source_out.glob('*.py')} == source_hashes,
            'Copied source files changed during freeze')
    write_json(output / 'review.json', review)
    write_json(output / 'candidate.json', candidate)
    manifest = {'evidence/' + path.name: sha(path) for path in evidence_out.iterdir()}
    manifest.update({'source/' + path.name: sha(path) for path in source_out.iterdir()})
    manifest['review.json'] = sha(output / 'review.json')
    manifest['candidate.json'] = sha(output / 'candidate.json')
    write_json(output / 'manifest.json', manifest)
    require(all(sha(output / name) == digest for name, digest in manifest.items()),
            'Frozen package verification failed')
    return {'status': 'DISABLED_ONLY_FILE_PACKAGE', 'package': str(output),
            'boot_id': expected_boot_id, 'segments': len(segments),
            'source_files': len(source_hashes), 'output_allowed': False,
            'live_runner_enabled': False, 'self_supported_standing_verified': False,
            'manifest_sha256': sha(output / 'manifest.json')}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hold-summary', type=Path, required=True)
    parser.add_argument('--stance-capture', type=Path, required=True)
    parser.add_argument('--physical-route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-boot-id', required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.hold_summary, args.stance_capture,
                           args.physical_route, args.output,
                           expected_boot_id=args.expected_boot_id), indent=2))


if __name__ == '__main__':
    main()
