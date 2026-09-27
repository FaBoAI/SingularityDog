"""Bind one supported-box pose and successful active hold to the partial trial.

This is file-only. It never opens a USB port or sends a motor command. The
frozen r4 runtime, gains, limits and ±3-degree start envelope are unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil

from load_transfer_2s_rebased_wrapper import BOOT, EVIDENCE, SOURCES, sha, verify_files
from load_transfer_2s_partial_wrapper import verify_files as verify_r4


R4_MANIFEST_SHA = '927425f68f663e2d630a34a6c93cafa8c135ede957fd9f53d9704e1d95c2a5ad'
BOX_SUPPORTED_MANIFEST_SHA = 'cc3e47176f9d8d856b0ef9aeac74ea791f259a85fe9af55c37fd167ba302e064'
R4_SOURCE = Path(__file__).resolve().parents[1] / 'runtime/singularitydog_hw/fixed_stance_readonly_capture.py'
WRAPPER = Path(__file__).resolve().with_name('load_transfer_2s_rebased_wrapper.py')
IDS = {str(i) for i in range(1, 13)}
PHYSICAL_FLAGS = ('box_supporting_torso', 'four_paws_floor',
                  'no_slip_or_clamp_contact')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), 'Missing or symlinked evidence: ' + str(path))
    return json.loads(path.read_text())


def validate_capture(summary_path, events_path, draft_path, source_path,
                     operator_path, review, after_monotonic_ns):
    capture = read(summary_path)
    draft = read(draft_path)
    operator = read(operator_path)
    source_path = Path(source_path)
    require(capture.get('schema') == 'singularitydog.fixed-stance-readonly-capture.v1'
            and capture.get('status') == 'RECORDED_REVIEW_REQUIRED'
            and capture.get('boot_id') == BOOT and capture.get('errors') == []
            and capture.get('output_allowed') is False
            and capture.get('approved_for_runtime') is False
            and capture.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL'
            and capture.get('plan', {}).get('allowed_can_types') == [0, 17]
            and capture.get('plan', {}).get('motor_output_available') is False
            and capture.get('plan', {}).get('stop_command_available') is False
            and capture.get('plan', {}).get('sweeps') == 3
            and capture.get('source_sha256', {}).get('fixed_stance_readonly_capture.py') == sha(source_path)
            and capture.get('source_sha256', {}).get('can_readonly.py') ==
                sha(Path(__file__).resolve().parents[1] / 'runtime/singularitydog_hw/can_readonly.py'),
            'Capture is not the current-boot, motor-disabled 12-axis tool')
    require(sha(source_path) == sha(R4_SOURCE), 'Capture collector source differs from reviewed local source')
    identities = capture.get('identities') or {}
    require(set(identities) == IDS
            and {mid: row.get('mcu_uid_hex') for mid, row in identities.items()} == review['motor_uids'],
            'Fresh read-only motor identities differ')
    pose = capture.get('pose') or {}
    raw = pose.get('raw_rad_by_id') or {}
    samples = pose.get('samples') or {}
    require(set(raw) == IDS and set(samples) == IDS
            and pose.get('sampling_issues') == []
            and pose.get('sampling_stability_heuristic_passed') is True
            and pose.get('stationarity_verified') is False
            and type(pose.get('started_monotonic_ns')) is int
            and pose['started_monotonic_ns'] > after_monotonic_ns
            and type(pose.get('ended_monotonic_ns')) is int
            and 0 < pose['ended_monotonic_ns'] - pose['started_monotonic_ns'] <= 15_000_000_000,
            'Capture is stale, incomplete or unstable')
    for mid in IDS:
        center = raw[mid]
        rows = samples[mid]
        require(type(center) in (int, float) and math.isfinite(center)
                and -12.57 <= center <= 12.57
                and type(rows) is list and len(rows) == 3,
                f'ID{mid} capture center/sweeps invalid')
        positions = []
        for sample in rows:
            require(set(sample) == {'position', 'velocity', 'current', 'voltage', 'run_mode'},
                    f'ID{mid} missing read-only parameter')
            values = {key: sample[key]['value'] for key in sample}
            require(all(type(v) in (int, float) and math.isfinite(v) for v in values.values())
                    and values['run_mode'] == 0 and abs(values['current']) <= .05
                    and abs(values['velocity']) <= .1
                    and 35. <= values['voltage'] <= 43.,
                    f'ID{mid} read-only electrical/motion envelope failed')
            positions.append(values['position'])
        require(max(positions) - min(positions) <= .02
                and min(positions) <= center <= max(positions),
                f'ID{mid} human-held pose moved or median is invalid')
    require(draft.get('schema') == 'singularitydog.fixed-stance-capture-draft.v1'
            and draft.get('boot_id') == BOOT
            and draft.get('motor_uids') == review['motor_uids']
            and draft.get('raw_rad_by_id') == raw
            and draft.get('stance_capture_sha256') == sha(summary_path)
            and draft.get('output_allowed') is False,
            'Read-only draft differs from exact capture')
    require(operator.get('boot_id') == BOOT
            and operator.get('baseline_summary_sha256') == sha(summary_path)
            and all(operator.get(flag) is True for flag in PHYSICAL_FLAGS)
            and operator.get('stand_removed_during_capture') is False
            and type(operator.get('operator_note')) is str
            and bool(operator['operator_note'].strip()),
            'Supported-box four-paw physical report is missing')
    require(Path(events_path).is_file() and Path(events_path).stat().st_size > 0,
            'Capture events are missing')
    return raw


def build(r4_package, r4_summary, r4_events, box_summary, box_events,
          box_draft, box_source, operator_report, supported_package,
          supported_summary, supported_events, output):
    (r4_package, r4_summary, r4_events, box_summary, box_events, box_draft,
     box_source, operator_report, supported_package, supported_summary,
     supported_events, output) = map(Path, (
        r4_package, r4_summary, r4_events, box_summary, box_events,
        box_draft, box_source, operator_report, supported_package,
        supported_summary, supported_events, output))
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir(),
            'Use a fresh private package directory')
    require(not any((p / '.git').exists() for p in output.parents),
            'Raw pose and motor UID evidence must stay outside Git')
    require(sha(r4_package / 'manifest.json') == R4_MANIFEST_SHA,
            'Prior partial r4 package is not the reviewed exact-source release')
    prior_review = verify_r4(r4_package, R4_MANIFEST_SHA)
    abort = read(r4_summary)
    prior_manifest = read(r4_package / 'manifest.json')
    require(abort.get('boot_id') == BOOT and abort.get('status') == 'ABORTED'
            and abort.get('motor_enable_sent') is False
            and abort.get('motion_gain_sent') is False
            and abort.get('trial_device_closed') is True
            and abort.get('locks_released') is True
            and abort.get('wrapper_sha256') == prior_manifest['prepared_load_transfer.py']
            and abort.get('events_sha256') == sha(r4_events)
            and abort.get('result', {}).get('stop_confirmed') is True
            and any('Exact-wire gate rejected' in error for error in
                    abort['result'].get('errors', [])),
            'Prior r4 pose mismatch did not end with all12 STOP before Enable')
    last_event_ns = max(json.loads(line)['monotonic_ns'] for line in
                        r4_events.read_text().splitlines() if line.strip())
    raw = validate_capture(box_summary, box_events, box_draft, box_source,
                           operator_report, prior_review, last_event_ns)
    before = {str(mid): abort['result']['workers'][
        'front' if mid <= 6 else 'rear']['motors'][str(mid)]['position']
        for mid in range(1, 13)}
    require(all(abs(raw[mid] - before[mid]) <= math.radians(.1) for mid in IDS),
            'Box capture differs from prior human-held r4 pose')
    require(len(BOX_SUPPORTED_MANIFEST_SHA) == 64
            and sha(supported_package / 'manifest.json') == BOX_SUPPORTED_MANIFEST_SHA,
            'Successful box-supported package hash has not been frozen')
    box_manifest = read(supported_package / 'manifest.json')
    require(all(sha(supported_package / name) == digest
                for name, digest in box_manifest.items()),
            'Box-supported package changed')
    box_result = read(supported_summary)
    require(box_result.get('boot_id') == BOOT
            and box_result.get('status') == 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED'
            and box_result.get('errors') == [] and box_result.get('signals') == []
            and box_result.get('wrapper_sha256') == box_manifest['prepared_load_transfer.py']
            and box_result.get('events_sha256') == sha(supported_events)
            and box_result.get('motor_enable_sent') is True
            and box_result.get('motion_gain_sent') is True
            and box_result.get('trial_device_closed') is True
            and box_result.get('locks_released') is True
            and box_result.get('result', {}).get('status') ==
                'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
            and box_result['result'].get('stop_confirmed') is True
            and all(box_result['result']['workers'][bus]['cycle_count'] == 25
                    and len(box_result['result']['workers'][bus]['electrical_samples']) == 25
                    and len(box_result['result']['workers'][bus]['stop_reports']) == 6
                    and all(row['confirmed'] is True for row in
                            box_result['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear')),
            'New box-pose supported active hold must succeed before partial load')
    require(box_result['result']['review']['motor_uids'] == prior_review['motor_uids']
            and box_result['result']['review']['supported_floor_start_raw_rad_by_id'] == raw,
            'Supported box hold did not use captured raw start envelope')
    supported_source = (supported_package / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
    from load_transfer_2s_rebased_wrapper import unpatch_partial_source
    require(unpatch_partial_source((r4_package / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()) ==
            supported_source
            and all(sha(r4_package / 'singularitydog_hw' / name) ==
                    box_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Partial runtime differs from successful supported box source beyond ID3 pre-Enable check')
    review = dict(prior_review)
    review.update(
        sha256=sha(box_summary),
        supported_floor_start_raw_rad_by_id=raw,
        box_baseline_correlated_with_human_held_r4=True,
        human_held_pre_enable_requalification_required=True,
        box_baseline_summary_sha256=sha(box_summary),
        box_baseline_events_sha256=sha(box_events),
        box_baseline_draft_sha256=sha(box_draft),
        box_baseline_operator_report_sha256=sha(operator_report),
        box_supported_summary_sha256=sha(supported_summary),
        box_supported_events_sha256=sha(supported_events),
        box_supported_manifest_sha256=BOX_SUPPORTED_MANIFEST_SHA,
        partial_r4_summary_sha256=sha(r4_summary),
        partial_r4_events_sha256=sha(r4_events),
        partial_r4_manifest_sha256=R4_MANIFEST_SHA,
        review_note=('Box-supported four-paw read-only baseline, captured after '
                     'the r4 pre-Enable pose-mismatch STOP, matches all twelve r4 '
                     'human-held raw positions within 0.1 degree and has a new '
                     'successful stand-supported 2-second active hold. The raw ±3-degree comparison, '
                     'fresh current-position targets, ID3-only pre-Enable 0.003-rad '
                     'full-window range, gains, all active guards, 2-second duration, '
                     'STOP and human catch remain unchanged. The actual human-held '
                     'trial must pass a fresh 12-axis raw start and 21-sample '
                     'pre-Enable stationarity check. No autonomous '
                     'rise or self-standing is claimed.'))
    os.umask(0o077)
    output.mkdir(mode=0o700)
    (output / 'singularitydog_hw').mkdir(mode=0o700)
    (output / 'evidence').mkdir(mode=0o700)
    for name in SOURCES:
        shutil.copyfile(r4_package / 'singularitydog_hw' / name,
                        output / 'singularitydog_hw' / name)
    new_evidence = {
        'partial-r4-summary.json': r4_summary,
        'partial-r4-events.jsonl': r4_events,
        'partial-r4-manifest.json': r4_package / 'manifest.json',
        'box-baseline-summary.json': box_summary,
        'box-baseline-events.jsonl': box_events,
        'box-baseline-draft.json': box_draft,
        'box-baseline-source.py': box_source,
        'box-baseline-operator-report.json': operator_report,
        'box-supported-summary.json': supported_summary,
        'box-supported-events.jsonl': supported_events,
        'box-supported-manifest.json': supported_package / 'manifest.json',
    }
    for name in EVIDENCE:
        shutil.copyfile(new_evidence.get(name, r4_package / 'evidence' / name),
                        output / 'evidence' / name)
    shutil.copyfile(WRAPPER, output / 'prepared_load_transfer.py')
    (output / 'review.json').write_text(json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for path in output.rglob('*'):
        path.chmod(0o700 if path.is_dir() else 0o600)
    digest = sha(output / 'manifest.json')
    verify_files(output, digest)
    return {'status': 'BOX_BASELINE_PARTIAL_PACKAGE_FROZEN_NOT_RUN',
            'package': str(output), 'manifest_sha256': digest,
            'boot_id': BOOT, 'duration_s': 2., 'motor_output_sent': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('r4-package', 'r4-summary', 'r4-events', 'box-summary',
                'box-events', 'box-draft', 'box-source', 'operator-report',
                'supported-package', 'supported-summary', 'supported-events',
                'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.r4_package, args.r4_summary, args.r4_events,
                           args.box_summary, args.box_events, args.box_draft,
                           args.box_source, args.operator_report,
                           args.supported_package, args.supported_summary,
                           args.supported_events, args.output),
                     indent=2), flush=True)


if __name__ == '__main__':
    main()
