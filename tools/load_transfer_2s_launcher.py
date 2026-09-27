"""Local audit entry point for a frozen, output-disabled 2 s package.

This file deliberately imports no serial or motor transport. `--active` is
always rejected. A future hardware wrapper must separately provide physical
support, an audio cue before actuation, dual-port ownership and STOP handling.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import threading

BOOT = '5662ee00-b2f5-4913-9bfd-33ae39642427'
SCHEMA = 'rs05-load-transfer-two-second-local-package-v1'
SOURCES = ('__init__.py', 'bounded_pose_plan.py', 'can_readonly.py',
           'current_hold_review.py', 'position_response_evidence.py',
           'rs05_bus_transport.py', 'rs05_joint_trial.py', 'rs05_leg_trial.py',
           'rs05_load_transfer_hold.py', 'rs05_trial_protocol.py')
EVIDENCE = ('floor-summary.json', 'floor-events.jsonl', 'readonly-summary.json',
            'operator-rehearsal.json')
FILE_NAMES = ({'review.json', 'launcher.py'}
              | {f'source/{name}' for name in SOURCES}
              | {f'evidence/{name}' for name in EVIDENCE})


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pairs(rows):
    result = {}
    for key, value in rows:
        if key in result:
            raise ValueError('Duplicate JSON key: ' + key)
        result[key] = value
    return result


def _read(path):
    path = Path(path)
    if not path.is_file() or path.is_symlink():
        raise ValueError('Missing or symlinked package file: ' + str(path))
    return json.loads(path.read_text(), object_pairs_hook=_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError('Nonfinite JSON value: ' + value)))


@contextmanager
def audit_lock(path):
    """Only a local audit lock; a future live wrapper needs motor/port locks."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('Frozen package audit lock already held') from exc
        yield
    finally:
        os.close(fd)


def active_start_reporter(started: threading.Event, finished: threading.Event,
                          *, stream=None):
    """Future wrapper helper: print from a separate thread after both buses start.

    The runner sets `started` at its active barrier. The reporter never runs
    in a CAN worker and does not move the 50 ms cycle deadline. The external
    I2S announcement must finish before the runner is called.
    """
    if stream is None:
        stream = sys.stdout
    while not finished.is_set():
        if started.wait(.01):
            print('ACTIVE_HOLD_STARTED', file=stream, flush=True)
            return True
    return False


def verify_package(package, expected_manifest_sha256, observed_boot_id):
    """Check exact local bytes and rejected-live review without device access."""
    package = Path(package)
    if not package.is_dir() or package.is_symlink():
        raise ValueError('Frozen package directory missing or symlinked')
    manifest_path = package / 'manifest.json'
    if _sha(manifest_path) != expected_manifest_sha256:
        raise ValueError('Trusted manifest SHA mismatch')
    manifest = _read(manifest_path)
    if (type(manifest) is not dict or set(manifest) != {'schema', 'boot_id', 'duration_s', 'file_sha256'}
            or manifest['schema'] != SCHEMA or manifest['boot_id'] != BOOT
            or type(manifest['duration_s']) not in (int, float) or manifest['duration_s'] != 2.
            or type(manifest['file_sha256']) is not dict
            or set(manifest['file_sha256']) != FILE_NAMES):
        raise ValueError('Wrong frozen two-second manifest')
    if observed_boot_id != BOOT:
        raise ValueError('Current boot differs from frozen trial')
    actual = {str(path.relative_to(package)) for path in package.rglob('*') if path.is_file()}
    if actual != FILE_NAMES | {'manifest.json'}:
        raise ValueError('Frozen package contains a missing or extra file')
    for dirname in ('source', 'evidence'):
        path = package / dirname
        if not path.is_dir() or path.is_symlink():
            raise ValueError('Frozen package directory missing or symlinked')
    for name, digest in manifest['file_sha256'].items():
        path = package / name
        if (type(digest) is not str or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)
                or not path.is_file() or path.is_symlink() or _sha(path) != digest):
            raise ValueError('Frozen file SHA mismatch: ' + name)
    source = (package / 'source' / 'rs05_load_transfer_hold.py').read_text()
    if 'LIVE_OUTPUT_ENABLED = False' not in source:
        raise ValueError('Frozen runtime live output gate is not closed')
    review = _read(package / 'review.json')
    floor = _read(package / 'evidence' / 'floor-summary.json')
    readonly = _read(package / 'evidence' / 'readonly-summary.json')
    operator = _read(package / 'evidence' / 'operator-rehearsal.json')
    if (review.get('schema') != 'rs05-load-transfer-current-hold-review-v1'
            or review.get('scope') != 'supervised-load-transfer-current-position-2-15s'
            or review.get('duration_s') != 2.
            or review.get('boot_id') != BOOT
            or review.get('load_transfer_hold_authorized') is not False
            or review.get('review_complete') is not False
            or review.get('learned_policy_allowed') is not False
            or review.get('standing_allowed') is not False
            or review.get('l_target_replay_allowed') is not False
            or review.get('automatic_retry_allowed') is not False
            or review.get('floor_hold_summary_sha256') != manifest['file_sha256']['evidence/floor-summary.json']
            or review.get('floor_hold_events_sha256') != manifest['file_sha256']['evidence/floor-events.jsonl']
            or review.get('operator_rehearsal_sha256') != manifest['file_sha256']['evidence/operator-rehearsal.json']
            or floor.get('boot_id') != BOOT
            or floor.get('result', {}).get('review', {}).get('motor_uids') != review.get('motor_uids')
            or floor.get('result', {}).get('status') != 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            or floor.get('result', {}).get('stop_confirmed') is not True
            or floor.get('events_sha256') != manifest['file_sha256']['evidence/floor-events.jsonl']
            or readonly.get('boot_id') != BOOT
            or operator.get('boot_id') != BOOT
            or operator.get('off_power_transfer_rehearsal_reported') is not True
            or operator.get('stand_restored') is not True
            or operator.get('two_operators_present') is not True):
        raise ValueError('Frozen review/evidence or physical report mismatch')
    return {'status': 'LOCAL_AUDIT_PASSED_LIVE_OUTPUT_DISABLED',
            'boot_id': BOOT, 'duration_s': 2., 'manifest_sha256': expected_manifest_sha256,
            'live_output_allowed': False, 'motor_io_attempted': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--observed-boot-id', required=True)
    parser.add_argument('--active', action='store_true')
    args = parser.parse_args(argv)
    if args.active:
        parser.error('LIVE_OUTPUT_DISABLED: this local launcher has no active path')
    lock_path = args.package.parent / (args.package.name + '.audit.lock')
    with audit_lock(lock_path):
        result = verify_package(args.package, args.manifest_sha256, args.observed_boot_id)
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
