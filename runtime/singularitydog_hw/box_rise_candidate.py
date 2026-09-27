"""Pure validation of a frozen 2 mm box-supported rise-and-return path.

This module has no hardware imports.  It checks the *recorded* raw path; joint
mapping and physical swept clearance must also be independently reviewed.
"""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

SCHEMA = 'singularitydog.box-rise-trial-review.v1'
CANDIDATE_SCHEMA = 'singularitydog.offline-box-rise-candidate.v1'
IDS = tuple(range(1, 13))
HIP_IDS = (3, 6, 9, 12)
TICKS = 101
PERIOD_S = .08
RISE_MM = 2.
START_TOLERANCE_RAD = math.radians(.5)
MAX_STEP_RAD = math.radians(.5)
MAX_EXCURSION_RAD = math.radians(12.)
RAW_MIN, RAW_MAX = -12.57 + math.radians(5.), 12.57 - math.radians(5.)
REQUIRED_PHYSICAL_REVIEWS = (
    'all_four_paws_on_floor_confirmed', 'box_directly_under_torso_confirmed',
    'torso_supported_by_box_confirmed', 'full_nonfoot_sweep_measured',
    'all_sample_joint_limits_verified', 'all_sample_swept_clearance_verified',
    'all_stop_reviewed', 'attended_40v_cutoff_ready',
    'loaded_voltage_and_torque_limits_reviewed', 'angle_sign_and_zero_revalidated',
)


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _digest(value):
    return (type(value) is str and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


def _raw_map(value, label):
    _require(type(value) is dict and set(value) == {str(i) for i in IDS},
             f'{label} requires exactly IDs 1..12')
    result = {}
    for mid in IDS:
        number = value[str(mid)]
        _require(type(number) in (int, float) and math.isfinite(number)
                 and RAW_MIN <= number <= RAW_MAX,
                 f'{label} ID{mid} outside finite Type1 range')
        result[mid] = float(number)
    return result


def _identities(value):
    _require(type(value) is dict and set(value) == {str(i) for i in IDS},
             'Exactly twelve identities required')
    _require(all(type(uid) is str and len(uid) == 16
                 and all(c in '0123456789abcdefABCDEF' for c in uid)
                 for uid in value.values()), 'Invalid motor identity')
    normalized = {i: value[str(i)].lower() for i in IDS}
    _require(len(set(normalized.values())) == 12, 'Motor identities are not unique')
    return normalized


def _rise_fraction(tick):
    half = (TICKS-1)//2
    progress = (tick if tick <= half else TICKS-1-tick)/half
    return progress**3*(10. + progress*(-15. + 6.*progress))


def prepare_candidate(review, *, current_boot_id, current_motor_uids,
                      fresh_raw_rad_by_id):
    """Validate all 101 recorded poses before either bus may Enable."""
    _require(type(review) is dict and review.get('schema') == SCHEMA,
             'Dedicated box-rise trial review required')
    _require(type(current_boot_id) is str and current_boot_id
             and review.get('boot_id') == current_boot_id,
             'Box-rise review boot mismatch')
    uids = _identities(current_motor_uids)
    _require(_identities(review.get('motor_uids')) == uids,
             'Box-rise review identities differ')
    for name in ('candidate_sha256', 'floor_snapshot_sha256', 'd17_urdf_sha256',
                 'angle_review_sha256', 'physical_review_sha256', 'runner_sha256',
                 'validator_sha256'):
        _require(_digest(review.get(name)), f'{name} missing')
    _require(review.get('automatic_retry_allowed') is False
             and review.get('learned_policy_allowed') is False
             and review.get('box_removal_allowed') is False,
             'Box-rise review cannot allow policy, retry or box removal')
    for name in REQUIRED_PHYSICAL_REVIEWS:
        _require(review.get(name) is True, f'Box-rise review missing {name}')
    start = _raw_map(review.get('start_raw_rad_by_id'), 'reviewed start')
    fresh = _raw_map(fresh_raw_rad_by_id, 'fresh start')
    for mid in IDS:
        _require(abs(fresh[mid]-start[mid]) <= START_TOLERANCE_RAD,
                 f'ID{mid} outside supported start envelope; no wrapping')
    corridors = review.get('reviewed_raw_corridor_by_id')
    _require(type(corridors) is dict and set(corridors) == {str(i) for i in IDS},
             'Twelve measured raw corridors required')
    bounds = {}
    for mid in IDS:
        row = corridors[str(mid)]
        _require(type(row) is dict and set(row) == {'min_rad', 'max_rad'},
                 f'ID{mid} corridor malformed')
        low, high = row['min_rad'], row['max_rad']
        _require(type(low) in (int, float) and type(high) in (int, float)
                 and math.isfinite(low) and math.isfinite(high)
                 and RAW_MIN <= low < high <= RAW_MAX,
                 f'ID{mid} corridor exceeds Type1 range')
        bounds[mid] = (low, high)
        _require(low <= fresh[mid] <= high, f'ID{mid} start outside corridor')
    candidate = review.get('candidate')
    camera_hashes = candidate.get('camera_l_sha256_by_leg') if type(candidate) is dict else None
    _require(type(camera_hashes) is dict and set(camera_hashes) == {'FR', 'FL', 'RR', 'RL'}
             and all(_digest(digest) for digest in camera_hashes.values()),
             'Four exact camera-L source hashes required')
    _require(type(candidate) is dict
             and candidate.get('schema') == CANDIDATE_SCHEMA
             and candidate.get('status') == 'OFFLINE_GEOMETRY_CANDIDATE_ONLY'
             and candidate.get('boot_id') == current_boot_id
             and _identities(candidate.get('motor_uids')) == uids
             and candidate.get('floor_snapshot_sha256') == review['floor_snapshot_sha256']
             and candidate.get('requested_body_rise_mm') == RISE_MM
             and candidate.get('sample_period_s') == PERIOD_S
             and candidate.get('sample_count') == TICKS
             and candidate.get('fixed_hip') is True
             and candidate.get('hip_raw_targets_equal_fresh_start') is True
             and type(candidate.get('hip_lateral_paw_drift_bound_mm')) in (int, float)
             and math.isfinite(candidate['hip_lateral_paw_drift_bound_mm'])
             and 0 <= candidate['hip_lateral_paw_drift_bound_mm'] <= .75
             and candidate.get('motor_output_allowed') is False
             and candidate.get('live_runner_available') is False
             and candidate.get('load_transfer_verified') is False
             and candidate.get('angle_wrapping_applied') is False,
             'Frozen candidate does not describe the exact 2 mm offline path')
    samples = candidate.get('samples')
    _require(type(samples) is list and len(samples) == TICKS,
             'Exact 101-sample box-rise path required')
    previous = None
    for tick, row in enumerate(samples):
        _require(type(row) is dict and row.get('tick') == tick
                 and type(row.get('elapsed_s')) in (int, float)
                 and math.isfinite(row['elapsed_s'])
                 and abs(row['elapsed_s']-tick*PERIOD_S) < 1e-9
                 and type(row.get('body_rise_mm')) in (int, float)
                 and math.isfinite(row['body_rise_mm'])
                 and abs(row['body_rise_mm']-RISE_MM*_rise_fraction(tick)) < 1e-7,
                 f'Box-rise sample {tick} timing or height differs')
        raw = _raw_map(row.get('raw_rad_by_id'), f'sample {tick}')
        for mid in IDS:
            low, high = bounds[mid]
            _require(low <= raw[mid] <= high, f'ID{mid} sample leaves corridor')
            _require(abs(raw[mid]-start[mid]) <= MAX_EXCURSION_RAD,
                     f'ID{mid} sample exceeds finite excursion')
            if previous is not None:
                _require(abs(raw[mid]-previous[mid]) <= MAX_STEP_RAD + 1e-12,
                         f'ID{mid} sample step exceeds half a degree')
            if mid in HIP_IDS:
                _require(abs(raw[mid]-start[mid]) < 1e-8,
                         f'ID{mid} hip target must stay at fresh raw start')
        if tick in (0, TICKS-1):
            _require(all(abs(raw[mid]-start[mid]) < 1e-8 for mid in IDS),
                     'Box-rise path must start and finish at the box pose')
        previous = raw
    return deepcopy(candidate)


def verify_package_files(package_dir, review):
    """Bind the in-memory review, candidate, and runner to frozen bytes."""
    directory = Path(package_dir)
    _require(directory.is_dir() and not directory.is_symlink(),
             'Exact frozen box-rise package directory required')
    source = Path(__file__).with_name('rs05_box_rise_trial.py').read_bytes()
    _require(hashlib.sha256(source).hexdigest() == review['runner_sha256'],
             'Box-rise runner source hash differs')
    _require(hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
             == review['validator_sha256'],
             'Box-rise validator source hash differs')
    source_files = {
        'floor-snapshot.json': review['floor_snapshot_sha256'],
        'angle-review.json': review['angle_review_sha256'],
        'physical-review.json': review['physical_review_sha256'],
        'd17.urdf': review['d17_urdf_sha256'],
    }
    for leg, digest in review['candidate']['camera_l_sha256_by_leg'].items():
        source_files[f'{leg.lower()}-camera-l.json'] = digest
    for name, digest in source_files.items():
        _require(hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest,
                 f'Frozen box-rise {name} differs')
    candidate_bytes = (directory / 'candidate.json').read_bytes()
    _require(hashlib.sha256(candidate_bytes).hexdigest() == review['candidate_sha256']
             and json.loads(candidate_bytes) == review['candidate'],
             'Box-rise candidate bytes differ')
    review_bytes = (directory / 'review.json').read_bytes()
    _require(json.loads(review_bytes) == review, 'Box-rise package review differs')
    return {'candidate_sha256': review['candidate_sha256'],
            'runner_sha256': review['runner_sha256']}
