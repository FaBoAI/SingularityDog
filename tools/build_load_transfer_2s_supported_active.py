"""Freeze a two-second, stand-supported active trial only after an r8 disabled pass.

The resulting package still needs a fresh, explicit operator authorization at
execution. This builder makes no serial connection and sends no motor command.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from load_transfer_2s_active_wrapper import (BOOT, EVIDENCE, SOURCES, sha,
                                              verify_files)

DISABLED_MANIFEST_SHA = '2cf7b37ac82fc4f28e63d75a10eef4af9ecb76d4defc7ba138caef51dcd59167'
PRIOR_ACTIVE_MANIFEST_SHA = '94244c00ba9711a50bc48d2456b64a64a63293e704be2358f5b9be30d3ad37ad'
ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / 'tools/load_transfer_2s_active_wrapper.py'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def build(disabled_package: Path, disabled_summary: Path,
          disabled_events: Path, prior_active_package: Path,
          prior_active_summary: Path, prior_active_events: Path, output: Path):
    disabled_package, prior_active_package = map(Path, (disabled_package, prior_active_package))
    disabled_summary, disabled_events, prior_active_summary, prior_active_events, output = map(
        Path, (disabled_summary, disabled_events, prior_active_summary, prior_active_events, output))
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir(),
            'Use a fresh private package directory')
    require(not any((p / '.git').exists() for p in output.parents),
            'Motor identities and physical reports must stay outside Git')
    require(sha(disabled_package / 'manifest.json') == DISABLED_MANIFEST_SHA,
            'Disabled source package differs from cue-fixed r8')
    disabled_manifest = json.loads((disabled_package / 'manifest.json').read_text())
    require(all(sha(disabled_package / name) == digest
                for name, digest in disabled_manifest.items()),
            'Disabled source package has changed')
    require(sha(prior_active_package / 'manifest.json') == PRIOR_ACTIVE_MANIFEST_SHA,
            'Prior supported-active package differs from reviewed r1')
    prior_manifest = json.loads((prior_active_package / 'manifest.json').read_text())
    require(all(sha(prior_active_package / name) == digest
                for name, digest in prior_manifest.items()),
            'Prior supported-active package has changed')
    prior = json.loads(prior_active_summary.read_text())
    require(prior.get('boot_id') == BOOT and prior.get('status') == 'ABORTED'
            and prior.get('events_sha256') == sha(prior_active_events)
            and prior.get('wrapper_sha256') == prior_manifest['prepared_load_transfer.py']
            and prior.get('motor_enable_sent') is True
            and prior.get('motion_gain_sent') is True
            and prior.get('trial_device_closed') is True
            and prior.get('locks_released') is True
            and prior.get('result', {}).get('stop_confirmed') is True
            and any('Stale feedback' in error for error in prior['result']['errors'])
            and all(prior['result']['workers'][bus]['cycle_count'] == 1
                    for bus in ('front', 'rear')),
            'Prior one-cycle stale-feedback abort/STOP evidence is incomplete')
    summary = json.loads(disabled_summary.read_text())
    require(summary.get('boot_id') == BOOT
            and summary.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary.get('errors') == [] and summary.get('signals') == []
            and summary.get('motor_enable_sent') is False
            and summary.get('motion_gain_sent') is False
            and summary.get('trial_device_closed') is True
            and summary.get('locks_released') is True
            and summary.get('events_sha256') == sha(disabled_events)
            and summary.get('wrapper_sha256') == disabled_manifest['prepared_load_transfer.py']
            and summary.get('result', {}).get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary['result'].get('stop_confirmed') is True
            and all(summary['result']['workers'][bus]['cycle_count'] == 25
                    and len(summary['result']['workers'][bus]['electrical_samples']) == 25
                    and len(summary['result']['workers'][bus]['stop_reports']) == 6
                    and all(row['confirmed'] is True for row in
                            summary['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear')),
            'Same-boot exact-source disabled r8 must pass every safety check')
    review = json.loads((disabled_package / 'preflight-review.json').read_text())
    require(review.get('boot_id') == BOOT and review.get('duration_s') == 2.
            and review.get('wrap_equivalence_motor_ids') == [3, 9]
            and review.get('physical_full_turn_excluded') is True
            and review.get('motor_uids') ==
                json.loads((disabled_package / 'evidence/floor-summary.json').read_text())
                ['result']['review']['motor_uids'],
            'Frozen review does not match supported floor evidence')
    review.update(review_complete=True, load_transfer_hold_authorized=True,
        supported_stance_passed=True, physical_catch_reviewed=True,
        power_cutoff_operator_reviewed=True, off_power_transfer_rehearsal_reviewed=True,
        load_specific_limits_reviewed=True, serial_write_timeout_verified=True,
        two_usb_80ms_workload_verified=True,
        stand_fully_supporting_required=True, partial_load_allowed=False,
        continuous_human_support_required=True, self_supported_stance_proven=False,
        timed_stop_catch_demonstrated=False,
        disabled_r8_summary_sha256=sha(disabled_summary),
        disabled_r8_events_sha256=sha(disabled_events),
        disabled_r8_manifest_sha256=DISABLED_MANIFEST_SHA,
        supported_r1_summary_sha256=sha(prior_active_summary),
        supported_r1_events_sha256=sha(prior_active_events),
        supported_r1_manifest_sha256=PRIOR_ACTIVE_MANIFEST_SHA,
        review_note=('Supported two-second electrical/mechanical diagnostic only. '
                     'Two operators maintain catch/cutoff; fixed stand carries torso. '
                     'No partial load, stand removal, self-standing or retry is authorized. '
                     'Prior r1 stopped after one cycle when a repeated per-write all-axis '
                     'freshness check read uncommitted interleaved replies; r2 checks '
                     'the complete snapshot once per cycle. 1.5 Nm Type2 estimate is '
                     'an abort monitor, not a physical torque cap.'))
    os.umask(0o077)
    output.mkdir(mode=0o700)
    (output / 'singularitydog_hw').mkdir(mode=0o700)
    (output / 'evidence').mkdir(mode=0o700)
    for name in SOURCES:
        src = disabled_package / 'singularitydog_hw' / name
        dst = output / 'singularitydog_hw' / name
        if name == 'rs05_load_transfer_hold.py':
            source = src.read_text()
            require(source.count('LIVE_OUTPUT_ENABLED = False') == 1,
                    'Disabled source gate is not unique')
            dst.write_text(source.replace('LIVE_OUTPUT_ENABLED = False',
                                          'LIVE_OUTPUT_ENABLED = True'))
        else:
            shutil.copyfile(src, dst)
    for name in EVIDENCE:
        if name.startswith('disabled-r8-'):
            src = {'disabled-r8-summary.json': disabled_summary,
                   'disabled-r8-events.jsonl': disabled_events,
                   'disabled-r8-manifest.json': disabled_package / 'manifest.json'}[name]
        elif name.startswith('supported-r1-'):
            src = {'supported-r1-summary.json': prior_active_summary,
                   'supported-r1-events.jsonl': prior_active_events,
                   'supported-r1-manifest.json': prior_active_package / 'manifest.json'}[name]
        else:
            src = disabled_package / 'evidence' / name
        shutil.copyfile(src, output / 'evidence' / name)
    shutil.copyfile(WRAPPER, output / 'prepared_load_transfer.py')
    (output / 'review.json').write_text(json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for p in output.rglob('*'):
        p.chmod(0o700 if p.is_dir() else 0o600)
    digest = sha(output / 'manifest.json')
    verify_files(output, digest)
    return {'status': 'SUPPORTED_ACTIVE_PACKAGE_FROZEN_NOT_RUN', 'package': str(output),
            'manifest_sha256': digest, 'boot_id': BOOT, 'duration_s': 2.,
            'requires_fresh_operator_authorization': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disabled-package', type=Path, required=True)
    parser.add_argument('--disabled-summary', type=Path, required=True)
    parser.add_argument('--disabled-events', type=Path, required=True)
    parser.add_argument('--prior-active-package', type=Path, required=True)
    parser.add_argument('--prior-active-summary', type=Path, required=True)
    parser.add_argument('--prior-active-events', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.disabled_package, args.disabled_summary,
                           args.disabled_events, args.prior_active_package,
                           args.prior_active_summary, args.prior_active_events,
                           args.output), indent=2), flush=True)


if __name__ == '__main__':
    main()
