"""File-only ledger for one same-boot front-hip 10-degree test session.

The commands freeze packages and import logs. They never connect to the robot,
open a serial port, run a frozen launcher, or approve a physical review. Each
stage has an immutable receipt so a failed command can be retried without
repeating an already completed stage or silently replacing evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile

import build_role_group_step as role
import prepare_front_hip_clearance_review as clearance


SCHEMA = 'singularitydog.front-hip-step10-session.v1'
PREPARED = 'front-hip-step10-prepared'
DISABLED = 'front-hip-step10-disabled'
ACTIVE = 'front-hip-step10-active'
ATTEMPT = re.compile(r'[a-z0-9][a-z0-9-]{0,47}\Z')


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    value = json.loads(path.read_text())
    require(type(value) is dict, f'Expected JSON object: {path}')
    return value


def write_once(path: Path, value: dict) -> None:
    """Publish a receipt atomically; an existing different receipt is an error."""
    if path.exists() or path.is_symlink():
        require(path.is_file() and not path.is_symlink() and read(path) == value,
                f'Existing receipt differs: {path}')
        return
    require(path.parent.is_dir() and not path.parent.is_symlink(),
            f'Receipt parent is missing or linked: {path.parent}')
    fd, tmp_name = tempfile.mkstemp(prefix='.receipt-', suffix='.json', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        # Refuse to overwrite a receipt that appeared while the temp was made.
        require(not path.exists() and not path.is_symlink(),
                f'Existing receipt appeared: {path}')
        os.link(tmp_name, path)
    finally:
        Path(tmp_name).unlink(missing_ok=True)


def check_session(root: Path) -> tuple[dict, Path, Path]:
    require(root.is_dir() and not root.is_symlink(), 'Session directory is missing or linked')
    ledger = read(root / 'session.json')
    require(ledger.get('schema') == SCHEMA, 'Wrong session schema')
    inputs = read(root / 'inputs.json')
    require(inputs.get('schema') == SCHEMA
            and inputs.get('boot_id') == ledger.get('boot_id')
            and inputs.get('continuous_profile') == ledger.get('continuous_profile')
            and inputs.get('source_disabled') == ledger.get('source_disabled')
            and inputs.get('source_disabled_manifest_sha256')
                == ledger.get('source_disabled_manifest_sha256')
            and inputs.get('hold_summary_sha256') == ledger.get('hold_summary_sha256'),
            'Session input receipt changed')
    disabled = root / DISABLED
    role.active_base.validate_frozen(disabled, 'prepared_fullbody.py', 'manifest.json')
    review = read(disabled / 'step2-review.json')
    require(review.get('boot_id') == ledger.get('boot_id')
            and review.get('role_group') == 'front-hip'
            and review.get('direction_profile') == 'front-hip-mirrored'
            and review.get('amplitude_deg') == 10.
            and review.get('continuous_profile') == ledger.get('continuous_profile')
            and (review.get('continuous_waypoints_deg')
                 == role.CONTINUOUS_FRONT_HIP_WAYPOINTS
                 if ledger.get('continuous_profile') else
                 'continuous_waypoints_deg' not in review)
            and digest(disabled / 'manifest.json') == ledger.get('disabled_manifest_sha256')
            and digest(disabled / 'current-hold-summary.json') == ledger.get('hold_summary_sha256'),
            'Session package, boot, or hold changed')
    return ledger, disabled, root / 'preflight'


def create(root: Path, source: Path, hold: Path, *, continuous_5_10: bool = False) -> dict:
    """Freeze the 10-degree candidate and disabled package exactly once."""
    require(not root.is_symlink(), 'Session path must not be a symlink')
    source, hold = source.resolve(strict=True), hold.resolve(strict=True)
    require(source.is_dir() and hold.is_file(), 'Frozen source and hold summary are required')
    require(not root.resolve().is_relative_to(source),
            'Session cannot be inside the frozen source')
    role.disabled_base.validate_source(source)
    source_review = read(source / 'fullbody-review.json')
    boot = source_review['boot_id']
    role.disabled_base.validate_hold(read(hold), digest(hold), boot,
                                     source_review['motor_uids'])
    source_sha, hold_sha = digest(source / 'manifest.json'), digest(hold)
    continuous_profile = (role.CONTINUOUS_FRONT_HIP_PROFILE if continuous_5_10 else None)
    inputs = {'schema': SCHEMA, 'boot_id': boot, 'source_disabled': str(source),
              'source_disabled_manifest_sha256': source_sha,
              'hold_summary_sha256': hold_sha,
              'continuous_profile': continuous_profile}
    if root.exists():
        require(root.is_dir(), 'Session path is not a directory')
        receipt = root / 'session.json'
        if receipt.exists():
            ledger, _, _ = check_session(root)
            require(ledger['source_disabled_manifest_sha256'] == source_sha
                    and ledger['source_disabled'] == str(source)
                    and ledger.get('continuous_profile') == continuous_profile
                    and ledger['hold_summary_sha256'] == hold_sha,
                    'Completed session has different source or hold')
            return ledger
        require({p.name for p in root.iterdir()} <= {'inputs.json', PREPARED, DISABLED},
                'Unreceipted session contains unexpected files')
        require((root / 'inputs.json').is_file()
                or not any(root.iterdir()),
                'Partial session lacks its original input receipt')
    else:
        require(root.parent.is_dir() and not root.parent.is_symlink(),
                'Create the session parent directory first')
        root.mkdir()
    write_once(root / 'inputs.json', inputs)
    prepared, disabled = root / PREPARED, root / DISABLED
    if prepared.exists():
        candidate = read(prepared / 'offline-raw-step2-candidate.json')
        evidence = prepared / 'fullbody-hold-evidence.json'
        require(candidate.get('boot_id') == boot
                and candidate.get('role_group') == 'front-hip'
                and candidate.get('amplitude_deg') == 10.
                and candidate.get('direction_profile') == 'front-hip-mirrored'
                and candidate.get('continuous_profile') == continuous_profile
                and (candidate.get('continuous_waypoints_deg')
                     == role.CONTINUOUS_FRONT_HIP_WAYPOINTS
                     if continuous_5_10 else
                     'continuous_waypoints_deg' not in candidate)
                and candidate.get('current_hold_source_sha256') == digest(evidence)
                and read(evidence)['revised_active_hold']['summary_sha256'] == hold_sha,
                'Existing prepared candidate is incomplete or belongs to another hold')
    else:
        options = ({'continuous_profile': continuous_profile} if continuous_5_10 else {})
        role.prepare(source, hold, 'front-hip', prepared, 'front-hip-mirrored', 10., **options)
    if disabled.exists():
        role.active_base.validate_frozen(disabled, 'prepared_fullbody.py', 'manifest.json')
        review = read(disabled / 'step2-review.json')
        require(review.get('boot_id') == boot and review.get('role_group') == 'front-hip'
                and review.get('amplitude_deg') == 10.
                and review.get('continuous_profile') == continuous_profile
                and digest(disabled / 'current-hold-summary.json') == hold_sha,
                'Existing disabled package is incomplete or belongs to another hold')
    else:
        role.disabled(source, hold, prepared, disabled)
    ledger = {'schema': SCHEMA, 'boot_id': boot,
              'continuous_profile': continuous_profile,
              'source_disabled': str(source),
              'source_disabled_manifest_sha256': source_sha,
              'hold_summary_sha256': hold_sha,
              'candidate_sha256': digest(prepared / 'offline-raw-step2-candidate.json'),
              'disabled_manifest_sha256': digest(disabled / 'manifest.json'),
              'disabled_only': True, 'motor_output_performed': False}
    write_once(root / 'session.json', ledger)
    return ledger


def copy_once(source: Path, destination: Path) -> str:
    require(source.is_file() and not source.is_symlink(), f'Missing regular source: {source}')
    value = digest(source)
    if destination.exists() or destination.is_symlink():
        require(destination.is_file() and not destination.is_symlink()
                and digest(destination) == value,
                f'Existing evidence differs: {destination}')
        return value
    with tempfile.NamedTemporaryFile(prefix='.evidence-', dir=destination.parent,
                                     delete=False) as stream:
        temp = Path(stream.name)
        try:
            with source.open('rb') as incoming:
                shutil.copyfileobj(incoming, stream)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temp.unlink(missing_ok=True)
            raise
    try:
        require(digest(temp) == value, 'Source evidence changed while copying')
        require(not destination.exists() and not destination.is_symlink(),
                f'Existing evidence appeared: {destination}')
        os.link(temp, destination)
    finally:
        temp.unlink(missing_ok=True)
    return value


def ingest_preflight(root: Path, log: Path) -> dict:
    ledger, disabled, preflight = check_session(root)
    summary, events = log / 'summary.json', log / 'events.jsonl'
    role.validate_role_group_preflight(disabled, summary, events, ledger['boot_id'])
    preflight.mkdir(exist_ok=True)
    summary_sha = copy_once(summary, preflight / 'summary.json')
    events_sha = copy_once(events, preflight / 'events.jsonl')
    role.validate_role_group_preflight(disabled, preflight / 'summary.json',
                                       preflight / 'events.jsonl', ledger['boot_id'])
    draft = root / 'physical-review-draft.json'
    if not draft.exists():
        clearance.prepare(disabled, preflight, draft)
    physical = read(draft)
    require(physical.get('boot_id') == ledger['boot_id']
            and physical.get('continuous_profile') == ledger.get('continuous_profile')
            and (physical.get('continuous_waypoints_deg')
                 == role.CONTINUOUS_FRONT_HIP_WAYPOINTS
                 if ledger.get('continuous_profile') else
                 'continuous_waypoints_deg' not in physical)
            and (physical.get('continuous_19s_reviewed') is False
                 if ledger.get('continuous_profile') else
                 'continuous_19s_reviewed' not in physical)
            and physical.get('clearance_reference_preflight_summary_sha256') == summary_sha
            and physical.get('source_disabled_review_sha256')
                == digest(disabled / 'step2-review.json')
            and all(physical.get(flag) is False for flag in role.FLAGS),
            'Physical-review draft was changed or belongs to another preflight')
    receipt = {'schema': SCHEMA, 'boot_id': ledger['boot_id'],
               'summary_sha256': summary_sha, 'events_sha256': events_sha,
               'draft_sha256': digest(draft), 'preflight_passed': True,
               'physical_approval_complete': False, 'motor_output_performed': False}
    write_once(root / 'preflight-receipt.json', receipt)
    return receipt


def check_preflight(root: Path) -> tuple[dict, dict, Path, Path]:
    ledger, disabled, preflight = check_session(root)
    receipt = read(root / 'preflight-receipt.json')
    require(receipt.get('schema') == SCHEMA and receipt.get('boot_id') == ledger['boot_id']
            and receipt.get('summary_sha256') == digest(preflight / 'summary.json')
            and receipt.get('events_sha256') == digest(preflight / 'events.jsonl')
            and receipt.get('draft_sha256') == digest(root / 'physical-review-draft.json'),
            'Preflight evidence or physical draft changed')
    role.validate_role_group_preflight(disabled, preflight / 'summary.json',
                                       preflight / 'events.jsonl', ledger['boot_id'])
    return ledger, receipt, disabled, preflight


def build_active(root: Path, source: Path, physical: Path) -> dict:
    ledger, preflight_receipt, disabled, preflight = check_preflight(root)
    source, physical = source.resolve(strict=True), physical.resolve(strict=True)
    physical_data = read(physical)
    continuous = ledger.get('continuous_profile')
    require(physical_data.get('continuous_profile') == continuous
            and (physical_data.get('continuous_waypoints_deg')
                 == role.CONTINUOUS_FRONT_HIP_WAYPOINTS
                 and physical_data.get('continuous_19s_reviewed') is True
                 if continuous else
                 'continuous_waypoints_deg' not in physical_data
                 and 'continuous_19s_reviewed' not in physical_data),
            'Continuous 1 s hold / 5° / 10° path needs its explicit 19 s physical review')
    source_sha = digest(source / 'active-manifest.json')
    physical_sha = digest(physical)
    active_dir = root / ACTIVE
    receipt_path = root / 'active-receipt.json'
    if receipt_path.exists():
        receipt = read(receipt_path)
        require(receipt.get('boot_id') == ledger['boot_id']
                and receipt.get('source_active_manifest_sha256') == source_sha
                and receipt.get('physical_review_sha256') == physical_sha
                and receipt.get('preflight_summary_sha256') == preflight_receipt['summary_sha256']
                and receipt.get('active_manifest_sha256') == digest(active_dir / 'active-manifest.json'),
                'Existing active receipt differs from supplied evidence')
        role.active_base.validate_frozen(active_dir, 'prepared_current_hold.py',
                                         'active-manifest.json')
        return receipt
    if active_dir.exists():
        # A builder interrupted after writing a complete package can be adopted;
        # a partial one is retained for diagnosis and never silently replaced.
        role.active_base.validate_frozen(active_dir, 'prepared_current_hold.py',
                                         'active-manifest.json')
        require(digest(active_dir / 'physical-review.json') == physical_sha
                and digest(active_dir / 'step2-preflight' / 'summary.json')
                    == preflight_receipt['summary_sha256'],
                'Existing active package differs from supplied physical/preflight evidence')
    else:
        role.active(source, disabled, preflight, physical, active_dir)
    role.active_base.validate_frozen(active_dir, 'prepared_current_hold.py',
                                     'active-manifest.json')
    require(digest(active_dir / 'physical-review.json') == physical_sha,
            'Physical review changed while building active package')
    receipt = {'schema': SCHEMA, 'boot_id': ledger['boot_id'],
               'continuous_profile': continuous,
               'source_active_manifest_sha256': source_sha,
               'physical_review_sha256': physical_sha,
               'preflight_summary_sha256': preflight_receipt['summary_sha256'],
               'active_manifest_sha256': digest(active_dir / 'active-manifest.json'),
               'package_ready': True, 'motor_output_performed': False,
               'fresh_pre_run_physical_confirmation_required': True}
    write_once(receipt_path, receipt)
    return receipt


def ingest_result(root: Path, log: Path, attempt: str) -> dict:
    """Import one finished trial, including failures, without retrying motion."""
    require(ATTEMPT.fullmatch(attempt) is not None, 'Attempt must be 1-48 lowercase letters/digits/hyphens')
    ledger, _, _, _ = check_preflight(root)
    active_receipt = read(root / 'active-receipt.json')
    active_dir = root / ACTIVE
    role.active_base.validate_frozen(active_dir, 'prepared_current_hold.py',
                                     'active-manifest.json')
    require(active_receipt.get('boot_id') == ledger['boot_id']
            and active_receipt.get('active_manifest_sha256')
                == digest(active_dir / 'active-manifest.json'),
            'Active package changed after preparation')
    summary, events = log / 'summary.json', log / 'events.jsonl'
    report = read(summary)
    result = report.get('result', {})
    active_review = read(active_dir / 'step2-active-review.json')
    require(report.get('boot_id') == ledger['boot_id']
            and report.get('wrapper_sha256') == digest(active_dir / 'prepared_current_hold.py')
            and report.get('events_sha256') == digest(events)
            and type(result) is dict
            and (result.get('review') == active_review
                 or (report.get('status') == 'INCOMPLETE' and 'review' not in result)),
            'Trial result does not belong to the same boot and frozen active package')
    target = root / 'attempts' / attempt
    target.mkdir(parents=True, exist_ok=True)
    summary_sha = copy_once(summary, target / 'summary.json')
    events_sha = copy_once(events, target / 'events.jsonl')
    stop = result.get('stop_confirmed') is True
    workers = result.get('workers', {})
    all_stops = (type(workers) is dict and set(workers) == {'front', 'rear'}
                 and all(type(workers[bus]) is dict
                         and type(workers[bus].get('stop_reports')) is dict
                         and set(workers[bus]['stop_reports'])
                         == {str(mid) for mid in ids}
                         and all(workers[bus]['stop_reports'][str(mid)].get('confirmed') is True
                                 for mid in ids)
                         for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13)))))
    receipt = {'schema': SCHEMA, 'boot_id': ledger['boot_id'], 'attempt': attempt,
               'continuous_profile': ledger.get('continuous_profile'),
               'summary_sha256': summary_sha, 'events_sha256': events_sha,
               'status': report.get('status'), 'motion_completed': result.get('motion_completed') is True,
               'all_twelve_stop_confirmed': stop and all_stops,
               'errors': report.get('errors', []),
               'manual_review_required_before_next_motion': True,
               'automatic_retry_performed': False}
    write_once(target / 'receipt.json', receipt)
    return receipt


def status(root: Path) -> dict:
    ledger, _, _ = check_session(root)
    result = {'schema': SCHEMA, 'boot_id': ledger['boot_id'],
              'continuous_profile': ledger.get('continuous_profile'),
              'disabled_package': str(root / DISABLED),
              'preflight_imported': False, 'active_package_ready': False,
              'attempts': [], 'next': 'Run disabled preflight on Jetson, then import its logs'}
    if (root / 'preflight-receipt.json').is_file():
        check_preflight(root)
        result['preflight_imported'] = True
        result['physical_review_draft'] = str(root / 'physical-review-draft.json')
        result['next'] = 'Inspect the current physical path and provide a separate signed-off review'
    if (root / 'active-receipt.json').is_file():
        active = read(root / 'active-receipt.json')
        role.active_base.validate_frozen(root / ACTIVE, 'prepared_current_hold.py',
                                         'active-manifest.json')
        require(active.get('active_manifest_sha256')
                == digest(root / ACTIVE / 'active-manifest.json')
                and active.get('physical_review_sha256')
                == digest(root / ACTIVE / 'physical-review.json')
                and active.get('preflight_summary_sha256')
                == digest(root / ACTIVE / 'step2-preflight' / 'summary.json'),
                'Active manifest or physical/preflight evidence changed')
        result['active_package_ready'] = True
        result['next'] = 'Fresh physical confirmation, one manual trial, then import its logs'
    attempts = root / 'attempts'
    if attempts.is_dir():
        for item in sorted(attempts.iterdir()):
            receipt = read(item / 'receipt.json')
            require(receipt['summary_sha256'] == digest(item / 'summary.json')
                    and receipt['events_sha256'] == digest(item / 'events.jsonl'),
                    f'Attempt evidence changed: {item.name}')
            result['attempts'].append({'attempt': item.name, 'status': receipt['status'],
                                       'motion_completed': receipt['motion_completed'],
                                       'all_twelve_stop_confirmed': receipt['all_twelve_stop_confirmed']})
        if any(not item['all_twelve_stop_confirmed'] for item in result['attempts']):
            result['next'] = 'STOP is unverified; inspect the robot and logs before any motion'
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='stage', required=True)
    p = sub.add_parser('create', help='Freeze a same-boot disabled package')
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--source-disabled', type=Path, required=True)
    p.add_argument('--hold-summary', type=Path, required=True)
    p.add_argument('--continuous-5-10', action='store_true',
                   help='Opt in to initial hold, then 5° and 10° in one active run')
    p = sub.add_parser('ingest-preflight', help='Verify disabled log and create an unapproved review draft')
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--log-dir', type=Path, required=True)
    p = sub.add_parser('build-active', help='Freeze reviewed active package; do not run it')
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--source-active', type=Path, required=True)
    p.add_argument('--physical-review', type=Path, required=True)
    p = sub.add_parser('ingest-result', help='Preserve one active attempt without retrying it')
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--log-dir', type=Path, required=True)
    p.add_argument('--attempt', required=True)
    p = sub.add_parser('status', help='Verify saved receipts and show the next manual stage')
    p.add_argument('--session', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.stage == 'create':
        result = create(args.session, args.source_disabled, args.hold_summary,
                        continuous_5_10=args.continuous_5_10)
    elif args.stage == 'ingest-preflight':
        result = ingest_preflight(args.session, args.log_dir)
    elif args.stage == 'build-active':
        result = build_active(args.session, args.source_active, args.physical_review)
    elif args.stage == 'ingest-result':
        result = ingest_result(args.session, args.log_dir, args.attempt)
    else:
        result = status(args.session)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
