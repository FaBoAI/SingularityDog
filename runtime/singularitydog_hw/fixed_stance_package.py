"""Check frozen fixed-stance files before a future reviewed active trial.

Hashes detect changed files and bind the selected hold, capture, route and
runtime source to the review. They do not establish that an operator's physical
clearance statements are true. The output gate remains disabled separately.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


EVIDENCE_NAMES = ('hold-summary.json', 'stance-capture.json', 'physical-route.json')
HEX = frozenset('0123456789abcdef')


def _digest(value):
    return type(value) is str and len(value) == 64 and set(value) <= HEX


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pairs(rows):
    result = {}
    for key, value in rows:
        if key in result:
            raise ValueError('Duplicate key in frozen stance JSON: ' + key)
        result[key] = value
    return result


def _read(path):
    return json.loads(path.read_text(), object_pairs_hook=_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError('Nonfinite frozen stance JSON: ' + value)))


def verify_package_files(package_dir, review):
    """Reject changed/mismatched package files without touching a motor bus.

    The active launcher must separately verify live boot, UIDs, packet limits,
    support, route clearance and the attended cutoff. This only pins bytes.
    """
    package = Path(package_dir)
    if not package.is_dir() or package.is_symlink():
        raise ValueError('Frozen stance package directory missing or symlinked')
    manifest_path = package / 'manifest.json'
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise ValueError('Frozen stance manifest missing or symlinked')
    manifest = _read(manifest_path)
    source_dir = Path(__file__).resolve().parent
    source_paths = list(source_dir.glob('*.py'))
    if any(not path.is_file() or path.is_symlink() for path in source_paths):
        raise ValueError('Loaded stance runtime source contains a symlink or missing file')
    source_names = {path.name for path in source_paths}
    expected = ({'review.json', 'candidate.json'}
                | {f'evidence/{name}' for name in EVIDENCE_NAMES}
                | {f'source/{name}' for name in source_names})
    if type(manifest) is not dict or set(manifest) != expected:
        raise ValueError('Frozen stance manifest does not list exact runtime and evidence files')
    for dirname in ('source', 'evidence'):
        directory = package / dirname
        if not directory.is_dir() or directory.is_symlink():
            raise ValueError('Frozen stance package directory missing or symlinked: ' + dirname)
    actual_files = {str(path.relative_to(package)) for path in package.rglob('*') if path.is_file()}
    if actual_files != expected | {'manifest.json'}:
        raise ValueError('Frozen stance package has missing or extra files')
    for name, digest in manifest.items():
        path = package / name
        if not _digest(digest) or not path.is_file() or path.is_symlink() or _sha(path) != digest:
            raise ValueError('Frozen stance file changed: ' + name)
    for name in source_names:
        if _sha(source_dir / name) != manifest[f'source/{name}']:
            raise ValueError('Loaded stance runtime differs from frozen source: ' + name)
    if _read(package / 'review.json') != review:
        raise ValueError('Selected stance review differs from frozen review')
    from .fixed_stance_candidate import prepare_candidate
    expected_candidate = prepare_candidate(
        review, current_boot_id=review['boot_id'],
        current_motor_uids=review['motor_uids'],
        fresh_raw_rad_by_id=review['start_raw_rad_by_id'])
    if _read(package / 'candidate.json') != expected_candidate:
        raise ValueError('Frozen stance candidate differs from regenerated finite plan')
    hold = _read(package / 'evidence' / 'hold-summary.json')
    capture = _read(package / 'evidence' / 'stance-capture.json')
    route = _read(package / 'evidence' / 'physical-route.json')
    for field, name in (('hold_summary_sha256', 'hold-summary.json'),
                        ('stance_capture_sha256', 'stance-capture.json'),
                        ('physical_route_review_sha256', 'physical-route.json')):
        if review.get(field) != manifest[f'evidence/{name}']:
            raise ValueError('Stance review evidence digest mismatch: ' + field)
    if (hold.get('boot_id') != review.get('boot_id')
            or hold.get('result', {}).get('review', {}).get('motor_uids') != review.get('motor_uids')
            or capture.get('boot_id') != review.get('boot_id')
            or capture.get('motor_uids') != review.get('motor_uids')
            or capture.get('raw_rad_by_id') != review.get('fixed_stance_raw_rad_by_id')
            or route.get('boot_id') != review.get('boot_id')
            or route.get('motor_uids') != review.get('motor_uids')
            or route.get('start_raw_rad_by_id') != review.get('start_raw_rad_by_id')
            or route.get('waypoints_raw_rad_by_id') != review.get('waypoints_raw_rad_by_id')
            or route.get('reviewed_raw_corridor_by_id') != review.get('reviewed_raw_corridor_by_id')
            or route.get('raw_corridor_physical_source_note')
            != review.get('raw_corridor_physical_source_note')):
        raise ValueError('Frozen stance evidence contents differ from selected review')
    segments = route.get('segments')
    segment_digests = review.get('segment_clearance_sha256')
    if (type(segments) is not list or type(segment_digests) is not list
            or len(segments) != len(segment_digests)):
        raise ValueError('Frozen stance segment evidence missing')
    for index, (segment, digest) in enumerate(zip(segments, segment_digests)):
        actual = hashlib.sha256(json.dumps(segment, sort_keys=True,
                              allow_nan=False).encode()).hexdigest()
        if digest != actual:
            raise ValueError(f'Frozen stance segment {index} changed')
    source_hashes = {name: manifest[f'source/{name}'] for name in source_names}
    source_digest = hashlib.sha256(json.dumps(source_hashes, sort_keys=True).encode()).hexdigest()
    if review.get('frozen_package_sha256') != source_digest:
        raise ValueError('Frozen stance source set digest mismatch')
    return {'manifest_sha256': _sha(manifest_path), 'source_sha256': source_digest}
