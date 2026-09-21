"""Validate a reviewed, UID-bound manual response record; never approve a pose.

The supplied record is an operator/reviewer assertion backed by private logs,
not an independent sensor. Its only scope is one supported FR relative jog.
Fresh position/feedback checks remain the trial runner's responsibility.
"""
import hashlib
import json
import math
from pathlib import Path


def load_position_response_evidence(path, expected_uids):
    raw = Path(path).read_bytes()
    value = json.loads(raw)
    if (not isinstance(value, dict)
            or value.get('schema') != 'rs05-manual-response-evidence-v1'
            or value.get('scope') != 'supported-FR-relative-5deg-5s'
            or value.get('review_complete') is not True
            or value.get('calibration_verified') is not False
            or value.get('absolute_pose_replay') is not False):
        raise ValueError('Wrong or unreviewed position-response evidence scope')
    motors = value.get('motors')
    if not isinstance(motors, dict) or set(motors) != {'1', '2', '3'}:
        raise ValueError('Manual response evidence must cover exactly FR IDs1/2/3')
    for key, entry in motors.items():
        if not isinstance(entry, dict):
            raise ValueError('Malformed manual response evidence')
        expected = expected_uids.get(int(key), expected_uids.get(key))
        if not expected or entry.get('mcu_uid_hex') != expected:
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
            if (not isinstance(digest, str) or len(digest) != 64
                    or any(c not in '0123456789abcdef' for c in digest)):
                raise ValueError(f'ID{key} invalid source hash')
    return {'sha256': hashlib.sha256(raw).hexdigest(), 'scope': value['scope'],
            'motor_ids': [1, 2, 3], 'prior_manual_response_reviewed': True,
            'current_boot_response_retested': False, 'calibration_verified': False,
            'absolute_pose_replay': False}
