"""Freeze a two-second, human-supported partial-load diagnostic package.

Reads only local files. It makes no serial connection and sends no motor command.
The complete stand-supported r2 run is a mandatory exact-source prerequisite.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from load_transfer_2s_partial_wrapper import (
    BOOT, EVIDENCE, SOURCES, patch_partial_source, sha, verify_files)

SUPPORTED_R2_MANIFEST_SHA = 'd07e596a6782f8c2d7a360c1e4744734fa92fb76eb523e43550ebe2c6b971ffc'
PRIOR_PARTIAL_R3_MANIFEST_SHA = '86cbfff6e6d4930e0f1636b64bdcbe4e8828f0deca559a876d9a8642e83f2c81'
ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / 'tools/load_transfer_2s_partial_wrapper.py'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def build(supported_package: Path, supported_summary: Path,
          supported_events: Path, prior_partial_package: Path,
          prior_partial_summary: Path, prior_partial_events: Path, output: Path):
    (supported_package, supported_summary, supported_events, prior_partial_package,
     prior_partial_summary, prior_partial_events, output) = map(Path, (
        supported_package, supported_summary, supported_events, prior_partial_package,
        prior_partial_summary, prior_partial_events, output))
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir(),
            'Use a fresh private package directory')
    require(not any((p / '.git').exists() for p in output.parents),
            'Motor identities and physical reports must stay outside Git')
    require(sha(supported_package / 'manifest.json') == SUPPORTED_R2_MANIFEST_SHA,
            'Supported r2 package differs from approved exact-source release')
    prior_manifest = json.loads((supported_package / 'manifest.json').read_text())
    require(all(sha(supported_package / name) == digest
                for name, digest in prior_manifest.items()),
            'Supported r2 package has changed')
    require(sha(prior_partial_package / 'manifest.json') == PRIOR_PARTIAL_R3_MANIFEST_SHA,
            'Prior partial r3 package differs')
    prior_partial_manifest = json.loads((prior_partial_package / 'manifest.json').read_text())
    require(all(sha(prior_partial_package / name) == digest
                for name, digest in prior_partial_manifest.items()),
            'Prior partial r3 package has changed')
    failed = json.loads(prior_partial_summary.read_text())
    front = failed.get('result', {}).get('workers', {}).get('front', {})
    rejected = front.get('settled_windows', {}).get('FR', {})
    id3 = rejected.get('motors', {}).get('3', {})
    require(failed.get('boot_id') == BOOT and failed.get('status') == 'ABORTED'
            and failed.get('motor_enable_sent') is False
            and failed.get('motion_gain_sent') is False
            and failed.get('trial_device_closed') is True
            and failed.get('locks_released') is True
            and failed.get('wrapper_sha256') == prior_partial_manifest['prepared_load_transfer.py']
            and failed.get('events_sha256') == sha(prior_partial_events)
            and failed.get('result', {}).get('stop_confirmed') is True
            and rejected.get('errors') == ['ID3: position_range_rad exceeds 0.001']
            and id3.get('errors') == ['position_range_rad exceeds 0.001']
            and .001 < id3.get('position_range_rad', 0) < .003,
            'Prior partial r3 did not isolate a pre-Enable ID3 range reject')
    prior_review = json.loads((supported_package / 'review.json').read_text())
    require(prior_review.get('boot_id') == BOOT
            and prior_review.get('duration_s') == 2.
            and prior_review.get('gain_profile') == 'id4-id10-kp4'
            and prior_review.get('stand_fully_supporting_required') is True
            and prior_review.get('partial_load_allowed') is False,
            'Prior stand-supported scope differs')
    supported = json.loads(supported_summary.read_text())
    require(supported.get('boot_id') == BOOT
            and supported.get('status') == 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED'
            and supported.get('errors') == [] and supported.get('signals') == []
            and supported.get('wrapper_sha256') == prior_manifest['prepared_load_transfer.py']
            and supported.get('events_sha256') == sha(supported_events)
            and supported.get('motor_enable_sent') is True
            and supported.get('motion_gain_sent') is True
            and supported.get('first_hold_cycle_confirmed') is True
            and supported.get('trial_device_closed') is True
            and supported.get('locks_released') is True
            and supported.get('result', {}).get('status') == 'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
            and supported['result'].get('stop_confirmed') is True
            and supported['result'].get('errors') == []
            and all(supported['result']['workers'][bus]['cycle_count'] == 25
                    and len(supported['result']['workers'][bus]['electrical_samples']) == 25
                    and len(supported['result']['workers'][bus]['stop_reports']) == 6
                    and all(row['confirmed'] is True for row in
                            supported['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear')),
            'Successful same-boot stand-supported r2 evidence is missing')
    require(supported['result']['review']['motor_uids'] == prior_review['motor_uids']
            and supported['result']['review']['gain_profile'] == 'id4-id10-kp4'
            and supported['result']['review']['duration_s'] == 2.,
            'Supported r2 motor binding or gain review differs')
    review = dict(prior_review)
    review.update(
        stand_fully_supporting_required=False, partial_load_allowed=True,
        stand_removed_only_with_40v_off=True, full_support_through_tick0=True,
        slight_ease_max_duration_s=.5, full_resupport_before_stop=True,
        partial_pre_enable_id3_full_window_range_rad=.003,
        side_view_video_available=True,
        self_supported_stance_proven=False, timed_stop_catch_demonstrated=False,
        supported_r2_summary_sha256=sha(supported_summary),
        supported_r2_events_sha256=sha(supported_events),
        supported_r2_manifest_sha256=SUPPORTED_R2_MANIFEST_SHA,
        partial_r3_summary_sha256=sha(prior_partial_summary),
        partial_r3_events_sha256=sha(prior_partial_events),
        partial_r3_manifest_sha256=PRIOR_PARTIAL_R3_MANIFEST_SHA,
        review_note=('Only a two-second diagnostic with continuous two-person catch. '
                     'Remove fixed stand with 40 V Off and fully support torso throughout '
                     'preflight, Enable and confirmed tick0. After '
                     'FIRST_HOLD_CYCLE_CONFIRMED, slightly ease support for at most '
                     '0.5 s and fully resupport at RE_SUPPORT_NOW, well before STOP. '
                     'Keep cutoff operator ready; catch through STOP. No hands-off '
                     'self-standing, learned policy, or automatic retry. 1.5 Nm Type2 '
                     'estimate is an abort monitor, not a physical torque cap. '
                     'Only FR ID3 pre-Enable full-window position range may reach '
                     '0.003 rad under two-person support; all other 11 axes '
                     'retain 0.001 rad, and slope/tail/speed/fault/freshness, '
                     'active drift/torque and final hold gates are unchanged.'))
    os.umask(0o077)
    output.mkdir(mode=0o700)
    (output / 'singularitydog_hw').mkdir(mode=0o700)
    (output / 'evidence').mkdir(mode=0o700)
    for name in SOURCES:
        source = supported_package / 'singularitydog_hw' / name
        target = output / 'singularitydog_hw' / name
        if name == 'rs05_load_transfer_hold.py':
            target.write_text(patch_partial_source(source.read_text()))
        else:
            shutil.copyfile(source, target)
    for name in EVIDENCE:
        src = {'supported-r2-summary.json': supported_summary,
               'supported-r2-events.jsonl': supported_events,
               'supported-r2-manifest.json': supported_package / 'manifest.json',
               'supported-r2-runtime.py': supported_package / 'singularitydog_hw/rs05_load_transfer_hold.py',
               'partial-r3-summary.json': prior_partial_summary,
               'partial-r3-events.jsonl': prior_partial_events,
               'partial-r3-manifest.json': prior_partial_package / 'manifest.json'}.get(name)
        shutil.copyfile(src or supported_package / 'evidence' / name,
                        output / 'evidence' / name)
    shutil.copyfile(WRAPPER, output / 'prepared_load_transfer.py')
    (output / 'review.json').write_text(json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for p in output.rglob('*'):
        p.chmod(0o700 if p.is_dir() else 0o600)
    digest = sha(output / 'manifest.json')
    verify_files(output, digest)
    return {'status': 'PARTIAL_LOAD_PACKAGE_FROZEN_NOT_RUN', 'package': str(output),
            'manifest_sha256': digest, 'boot_id': BOOT, 'duration_s': 2.,
            'requires_fresh_operator_authorization': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--supported-package', type=Path, required=True)
    parser.add_argument('--supported-summary', type=Path, required=True)
    parser.add_argument('--supported-events', type=Path, required=True)
    parser.add_argument('--prior-partial-package', type=Path, required=True)
    parser.add_argument('--prior-partial-summary', type=Path, required=True)
    parser.add_argument('--prior-partial-events', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.supported_package, args.supported_summary,
                           args.supported_events, args.prior_partial_package,
                           args.prior_partial_summary, args.prior_partial_events,
                           args.output), indent=2), flush=True)


if __name__ == '__main__':
    main()
