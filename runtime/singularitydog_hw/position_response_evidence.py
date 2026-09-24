"""Validate a reviewed, UID-bound manual response record; never approve a pose.

The supplied record is an operator/reviewer assertion backed by private logs,
not an independent sensor. Its only scope is one supported, explicitly selected leg relative jog.
Fresh position/feedback checks remain the trial runner's responsibility.
"""
import hashlib
import json
import math
from pathlib import Path


LEG_IDS = {'FR': (1, 2, 3), 'FL': (4, 5, 6), 'RR': (7, 8, 9), 'RL': (10, 11, 12)}


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('Duplicate evidence JSON key')
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError('Nonfinite evidence JSON constant: ' + value)


def _digest(value):
    return (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


def _expected_identities(values, ids):
    if not isinstance(values, dict) or any(type(k) not in (str, int) for k in values):
        raise ValueError('Expected identities must be a motor-ID object')
    if any(str(k) not in {str(i) for i in range(1, 13)} for k in values):
        raise ValueError('Invalid expected motor ID')
    normalized = {int(k): v for k, v in values.items()}
    if len(normalized) != len(values) or set(normalized) not in (set(ids), set(range(1, 13))):
        raise ValueError('Expected exactly the selected leg or all twelve identities')
    if (any(not isinstance(v, str) or len(v) != 16
            or any(c not in '0123456789abcdef' for c in v) for v in normalized.values())
            or len(set(normalized.values())) != len(normalized)):
        raise ValueError('Expected unique lowercase hexadecimal motor identities')
    return {str(i): normalized[i] for i in ids}


def load_position_response_evidence(path, expected_uids, *, leg='FR', expected_sha256=None):
    """Validate the file itself for one selected leg, optionally pinned by an earlier read.

    This does not reopen the source logs or replace the trial's fresh checks.
    The historical source hashes are assertions checked by the offline reviewer.
    """
    if not isinstance(leg, str) or leg not in LEG_IDS:
        raise ValueError('Position-response evidence requires one named leg')
    if expected_sha256 is not None and not _digest(expected_sha256):
        raise ValueError('Invalid pinned position-response evidence SHA256')
    ids = LEG_IDS[leg]
    expected = _expected_identities(expected_uids, ids)
    raw = Path(path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError('Position-response evidence file changed after planning')
    value = json.loads(raw, object_pairs_hook=_pairs, parse_constant=_invalid_constant)
    if (not isinstance(value, dict)
            or value.get('schema') != 'rs05-manual-response-evidence-v1'
            or value.get('scope') != f'supported-{leg}-relative-5deg-5s'
            or value.get('review_complete') is not True
            or value.get('calibration_verified') is not False
            or value.get('absolute_pose_replay') is not False):
        raise ValueError('Wrong or unreviewed position-response evidence scope')
    motors = value.get('motors')
    if not isinstance(motors, dict) or set(motors) != set(expected):
        raise ValueError('Manual response evidence must cover exactly the selected leg')
    for key, entry in motors.items():
        if not isinstance(entry, dict):
            raise ValueError('Malformed manual response evidence')
        if entry.get('mcu_uid_hex') != expected[key]:
            raise ValueError(f'ID{key} manual evidence identity mismatch')
        if (entry.get('physical_movement_confirmed') is not True
                or entry.get('confirmation_source') not in ('manual_cli_y', 'user_followup')):
            raise ValueError(f'ID{key} missing physical movement confirmation')
        span, corr = entry.get('position_span_deg'), entry.get('position_velocity_correlation')
        unique = entry.get('position_unique_values')
        if (type(span) not in (int, float) or not math.isfinite(span) or span < 1
                or type(corr) not in (int, float) or not math.isfinite(corr) or not .9 <= corr <= 1
                or type(unique) is not int or unique < 10):
            raise ValueError(f'ID{key} insufficient recorded position response')
        sources = entry.get('source_sha256')
        if not isinstance(sources, dict) or not {'events.jsonl', 'summary.json'} <= set(sources):
            raise ValueError(f'ID{key} missing source hashes')
        for digest in sources.values():
            if not _digest(digest):
                raise ValueError(f'ID{key} invalid source hash')
    return {'sha256': hashlib.sha256(raw).hexdigest(), 'scope': value['scope'], 'leg': leg,
            'motor_ids': list(ids), 'prior_manual_response_reviewed': True,
            'current_boot_response_retested': False, 'calibration_verified': False,
            'absolute_pose_replay': False}
