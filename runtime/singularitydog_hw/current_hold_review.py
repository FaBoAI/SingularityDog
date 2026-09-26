"""Pinned per-axis reviewer assertions for scoped current-reference holds.

This is not a generic stationary-sensor approval or a calibration. The frozen
deployment package verifies source logs separately. Fresh Type2 guards are
still required by the runner after this file is checked, before any enable.
"""
import hashlib
import json
import math
from pathlib import Path

from .position_response_evidence import _pairs, _invalid_constant, _digest, _expected_identities

PROFILE = 'current-hold-reviewed-v1'
REVIEWED_IDS = {(4, 5, 6): 6, (7, 8, 9): 8}
ALLOWED_REVIEW_SETS = {(4, 5, 6): ((6,), (5, 6)), (7, 8, 9): ((8,),)}
REVIEW_SCHEMA = 'rs05-current-hold-review-v1'
REVIEW_SET_SCHEMA = 'rs05-current-hold-review-set-v1'
SCOPE = 'supported-current-reference-hold-5s'


def validated_reviewed_ids(ids, reviewed_motor_ids=None):
    """Permit only the original singleton or the explicit two-review FL set."""
    if (type(ids) is not tuple or any(type(i) is not int for i in ids)
            or ids not in REVIEWED_IDS):
        raise ValueError('Current hold review permits only exact FL or RR IDs')
    if reviewed_motor_ids is None:
        return (REVIEWED_IDS[ids],)
    if (type(reviewed_motor_ids) not in (tuple, list)
            or any(type(i) is not int for i in reviewed_motor_ids)
            or tuple(reviewed_motor_ids) not in ALLOWED_REVIEW_SETS[ids]):
        raise ValueError('Current hold requires an exact allowed reviewed motor set')
    return tuple(reviewed_motor_ids)


def _validate_motor_review(value, expected, mid, boot):
    """Validate every assertion independently, including both nested FL reviews."""
    if (not isinstance(value, dict)
            or value.get('schema') != REVIEW_SCHEMA
            or value.get('scope') != SCOPE
            or type(value.get('reviewed_motor_id')) is not int
            or value['reviewed_motor_id'] != mid
            or value.get('motor_uid_hex') != expected[str(mid)]
            or value.get('firmware') != '0.5.0.13'
            or value.get('review_complete') is not True
            or value.get('physical_large_motion_confirmed') is not True
            or value.get('calibration_verified') is not False
            or value.get('learned_policy_allowed') is not False):
        raise ValueError(f'ID{mid} wrong identity, firmware or current-hold review scope')
    if not boot or value.get('boot_id') != boot:
        raise ValueError(f'ID{mid} current hold review boot mismatch')
    m = value.get('metrics', {})
    if not isinstance(m, dict):
        raise ValueError(f'ID{mid} missing current-hold response metrics')
    span, corr, unique = (m.get(k) for k in
                         ('position_span_deg', 'position_velocity_correlation', 'position_unique_values'))
    if (type(span) not in (int, float) or not math.isfinite(span) or span < 1
            or type(corr) not in (int, float) or not math.isfinite(corr) or not .9 <= corr <= 1
            or type(unique) is not int or unique < 10):
        raise ValueError(f'ID{mid} insufficient current-hold manual response evidence')
    sources = value.get('source_sha256')
    if (type(sources) is not dict or set(sources) != {'events.jsonl', 'summary.json', 'audit.json'}
            or not all(_digest(h) for h in sources.values())):
        raise ValueError(f'ID{mid} current hold source hashes are incomplete')
    return {'scope': value['scope'], 'reviewed_motor_id': mid,
            'boot_id': boot, 'firmware': value['firmware'], 'source_sha256': sources,
            'manual_response_reviewed': True, 'calibration_verified': False,
            'learned_policy_allowed': False, 'physical_angle_accuracy_verified': False}


def load_current_hold_review(path, expected_uids, *, ids, absolute_targets,
        matched_start_positions, matched_start_tolerance_rad, gain_profile,
        observation_profile, expected_sha256):
    """Reject wider scope before transport I/O; never fall back to old evidence."""
    original_reviewed = validated_reviewed_ids(ids)
    if (type(absolute_targets) is not dict or type(matched_start_positions) is not dict
            or set(absolute_targets) != set(ids) or set(matched_start_positions) != set(ids)
            or any(type(k) is not int for k in (*absolute_targets, *matched_start_positions))
            or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 12.57
                   for v in (*absolute_targets.values(), *matched_start_positions.values()))
            or absolute_targets != matched_start_positions):
        raise ValueError('Current hold requires identical finite fixed targets and matched references')
    if (type(matched_start_tolerance_rad) not in (int, float)
            or not math.isfinite(matched_start_tolerance_rad)
            or not 0 < matched_start_tolerance_rad <= math.radians(.5)
            or gain_profile != 'kp3' or observation_profile is not None):
        raise ValueError('Current hold requires kp3, no observation profile, and <=0.5degree match')
    if path is None or not _digest(expected_sha256):
        raise ValueError('Current hold requires a SHA256-pinned review file')
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError('Current hold review hash mismatch')
    value = json.loads(raw, object_pairs_hook=_pairs, parse_constant=_invalid_constant)
    expected = _expected_identities(expected_uids, ids)
    if not isinstance(value, dict):
        raise ValueError('Current hold review must be an object')
    if value.get('schema') == REVIEW_SET_SCHEMA:
        reviews = value.get('reviews')
        if (set(value) != {'schema', 'scope', 'reviews'} or value.get('scope') != SCOPE
                or ids != (4, 5, 6) or type(reviews) is not dict
                or set(reviews) != {'5', '6'}):
            raise ValueError('Current hold review set requires exactly FL ID5 and ID6 reviews')
        reviewed = validated_reviewed_ids(ids, (5, 6))
    elif value.get('schema') == REVIEW_SCHEMA:
        reviewed = original_reviewed
        reviews = {str(reviewed[0]): value}
    else:
        raise ValueError('Unknown current hold review schema')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    checked = {str(mid): _validate_motor_review(reviews[str(mid)], expected, mid, boot)
               for mid in reviewed}
    if len(reviewed) == 1:
        return {**checked[str(reviewed[0])], 'sha256': expected_sha256,
                'reviewed_motor_ids': list(reviewed)}
    return {'sha256': expected_sha256, 'scope': SCOPE, 'reviewed_motor_ids': list(reviewed),
            'motor_reviews': checked, 'boot_id': boot, 'firmware': '0.5.0.13',
            'manual_response_reviewed': True, 'calibration_verified': False,
            'learned_policy_allowed': False, 'physical_angle_accuracy_verified': False}
