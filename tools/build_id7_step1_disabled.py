"""Freeze a disabled-only ID7 one-degree diagnostic preflight package.

The read-only snapshot is a start *hint*, not an authorization or a settled
window. This builder never opens a device. Its output is local; transfer and
hardware execution are separate operations.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import shutil


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
SOURCE_NAME = 'fullbody-step2-disabled-20260926-r2'
OUTPUT_NAME = 'id7-step1-disabled-20260926-r3'
BOOT_RE = re.compile(r"^BOOT = '[^']+'$", re.MULTILINE)
BASE_RE = re.compile(r"^BASE = Path\('[^']+'\)$", re.MULTILINE)
FLAGS = ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
         'support_stand_verified', 'feet_clear_verified', 'hands_clear_verified',
         'physical_cutoff_ready')
IDS = {str(mid) for mid in range(1, 13)}


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def validate_source(source: Path) -> tuple[dict, dict]:
    require(source.is_dir() and not source.is_symlink() and source.name == SOURCE_NAME,
            'A frozen r2 disabled-only source package is required')
    wrapper = source / 'prepared_fullbody.py'
    require(wrapper.is_file() and not wrapper.is_symlink(), 'Disabled wrapper missing')
    body = wrapper.read_text()
    require('class DisabledPort:' in body
            and "if not args.preflight_only:" in body
            and 'Disabled diagnostic rejects enable, nonzero gains' in body
            and 'preflight_only=True)' in body,
            'Source is not the write-gated disabled wrapper')
    match = re.search(r'^PINS = (.*)$', body, re.MULTILINE)
    require(match is not None, 'No source pins')
    pins = ast.literal_eval(match.group(1))
    manifest = read_json(source / 'manifest.json')
    require(type(pins) is dict and len(pins) >= 100
            and manifest == {**pins, wrapper.name: sha(wrapper)},
            'Source manifest differs from wrapper pins')
    for name, digest in manifest.items():
        path = source / name
        require(path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Source pin mismatch: ' + name)
    require(not any(path.is_symlink() for path in source.rglob('*')),
            'Source package contains a symlink')
    prior = read_json(source / 'step2-review.json')
    hold = read_json(source / 'current-hold-summary.json')
    require(prior.get('schema') == 'rs05-fullbody-step2-review-v1'
            and prior.get('supported_step_authorized') is False
            and prior.get('current_hold_summary_sha256')
                == sha(source / 'current-hold-summary.json')
            and hold.get('boot_id') == prior.get('boot_id')
            and hold.get('result', {}).get('status')
                == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            and hold.get('result', {}).get('stop_confirmed') is True,
            'The frozen r2 hold provenance is incomplete')
    return prior, hold


def make_review(source: Path, readonly_path: Path, abort_path: Path) -> dict:
    prior, hold = validate_source(source)
    readonly = read_json(readonly_path)
    aborted = read_json(abort_path)
    boot = prior['boot_id']
    uids = prior['motor_uids']
    require(type(uids) is dict and set(uids) == IDS,
            'Frozen source lacks twelve motor UIDs')
    require(readonly.get('boot_id') == boot and readonly.get('uids') == uids
            and readonly.get('method')
                == 'ReadOnlyCAN Type0 identity and Type17 position only, eight sequential sweeps per bus'
            and readonly.get('source_sha256')
                == sha(source / 'singularitydog_hw' / 'can_readonly.py')
            and type(readonly.get('started_wall_ns')) is int
            and type(readonly.get('completed_wall_ns')) is int
            and readonly['started_wall_ns'] < readonly['completed_wall_ns'],
            'Latest read-only sample is not same-boot, same-UID, or source-pinned')
    require(aborted.get('boot_id') == boot and aborted.get('status') == 'INCOMPLETE'
            and aborted.get('motor_enable_sent') is True
            and aborted.get('motion_gain_sent') is True
            and aborted.get('trial_device_closed') is True
            and aborted.get('locks_released') is True
            and aborted.get('result', {}).get('status') == 'ABORTED'
            and aborted.get('result', {}).get('stop_confirmed') is True
            and any('ID7 raw step excursion exceeded2.5deg' in e
                    for e in aborted['result'].get('errors', []))
            and type(aborted.get('completed_wall_time_ns')) is int
            and aborted['completed_wall_time_ns'] < readonly['started_wall_ns'],
            'ID7 abort evidence must precede the read-only snapshot, with all12 STOP')
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        stops = aborted['result'].get('workers', {}).get(bus, {}).get('stop_reports', {})
        require(set(stops) == {str(mid) for mid in ids}
                and all(stops[str(mid)].get('confirmed') is True for mid in ids),
                f'Incomplete {bus} STOP confirmation in ID7 abort evidence')
    positions = readonly.get('positions')
    require(type(positions) is dict and set(positions) == IDS,
            'Read-only snapshot must contain all twelve positions')
    starts = {}
    for mid in sorted(IDS, key=int):
        row = positions[mid]
        require(type(row) is dict and set(row) == {'min_rad', 'max_rad', 'mean_rad', 'last_rad'},
                f'ID{mid} read-only fields incomplete')
        values = list(row.values())
        require(all(type(x) in (int, float) and math.isfinite(x) for x in values)
                and 0. <= row['min_rad'] <= row['last_rad'] <= row['max_rad'] <= 2*math.pi
                and row['min_rad'] <= row['mean_rad'] <= row['max_rad']
                and row['max_rad']-row['min_rad'] < math.radians(.5),
                f'ID{mid} read-only position invalid or too variable')
        starts[mid] = row['last_rad']
    return {
        'schema': 'rs05-id7-step1-review-v1',
        'scope': 'supported-id7-raw-1deg-other11-hold-50ms-diagnostic',
        'motor_ids': list(range(1, 13)), 'motor_uids': uids,
        'firmware': '0.5.0.13', 'sha256': sha(readonly_path),
        'source_active_step2_abort_sha256': sha(abort_path),
        'id7_joint_inspection_verified': False,
        'review_complete': True, 'source_files_verified': True,
        'gain_profile': 'id4-id10-kp4', 'old_raw_target_reused': False,
        'calibration_verified': False, 'model_mapping_verified': False,
        'learned_policy_allowed': False, 'standing_allowed': False,
        'automatic_retry_allowed': False,
        **{flag: False for flag in FLAGS},
        'current_hold_status': hold['result']['status'],
        'current_hold_stop_confirmed': True,
        'current_hold_summary_sha256': sha(source / 'current-hold-summary.json'),
        'current_hold_gain_profile': 'id4-id10-kp4',
        'supported_step_authorized': False,
        'boot_id': boot, 'current_hold_boot_id': boot,
        'start_raw_rad_by_id': starts,
        'raw_direction_by_id': {str(mid): 1 if mid == 7 else 0
                                for mid in range(1, 13)},
        'amplitude_deg': 1.,
        'read_only_eight_sweeps_are_not_a_settled_window': True,
        'read_only_last_position_is_only_a_start_hint': True,
    }


def adapt_wrapper(body: str, name: str, boot: str) -> str:
    base = BASE_RE.search(body)
    require(base is not None, 'No source BASE')
    body = body[:base.start()] + (
        f"BASE = Path('/home/jetson/singularitydog-tests/{name}')") + body[base.end():]
    body, count = BOOT_RE.subn(f"BOOT = '{boot}'", body, count=1)
    require(count == 1, 'No source BOOT')
    old_import = 'from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2'
    require(body.count(old_import) == 1, 'No singular step2 runner import')
    body = body.replace(old_import,
                        'from singularitydog_hw.rs05_id7_step1 import run_id7_step1')
    old_modules = "'fullbody_step10_plan', 'rs05_fullbody_step2')"
    require(body.count(old_modules) == 1, 'Unexpected module verification list')
    body = body.replace(old_modules,
                        "'fullbody_step10_plan', 'rs05_fullbody_step2', 'rs05_id7_step1')")
    start = "        candidate = json.loads((BASE / 'offline-raw-step2-candidate.json').read_text())"
    end = "                'No same-boot verified hold or disabled-only step review')"
    require(body.count(start) == body.count(end) == 1,
            'No singular source review assertion block')
    a = body.index(start)
    b = body.index(end, a) + len(end)
    replacement = """        id7_review = json.loads((BASE / 'id7-review.json').read_text())
        readonly = json.loads((BASE / 'id7-readonly-summary.json').read_text())
        abort = json.loads((BASE / 'id7-active-step2-abort-summary.json').read_text())
        hold = json.loads((BASE / 'current-hold-summary.json').read_text())
        require(id7_review['schema'] == 'rs05-id7-step1-review-v1'
                and id7_review['scope'] == 'supported-id7-raw-1deg-other11-hold-50ms-diagnostic'
                and id7_review['supported_step_authorized'] is False
                and id7_review['id7_joint_inspection_verified'] is False
                and all(id7_review[k] is False for k in
                        ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
                         'support_stand_verified', 'feet_clear_verified', 'hands_clear_verified',
                         'physical_cutoff_ready'))
                and id7_review['sha256'] == sha(BASE / 'id7-readonly-summary.json')
                and id7_review['source_active_step2_abort_sha256']
                    == sha(BASE / 'id7-active-step2-abort-summary.json')
                and id7_review['current_hold_summary_sha256'] == sha(BASE / 'current-hold-summary.json')
                and readonly['boot_id'] == BOOT and readonly['uids'] == expected
                and readonly['source_sha256'] == sha(BASE / 'singularitydog_hw/can_readonly.py')
                and abort['boot_id'] == BOOT and abort['result']['status'] == 'ABORTED'
                and abort['result']['stop_confirmed'] is True
                and hold['boot_id'] == BOOT
                and hold['status'] == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
                and hold['result']['status'] == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
                and hold['result']['stop_confirmed'] is True
                and hold['result']['errors'] == []
                and hold['result']['review']['motor_uids'] == expected
                and id7_review['start_raw_rad_by_id'] ==
                    {k: v['last_rad'] for k, v in readonly['positions'].items()}
                and id7_review['raw_direction_by_id'] ==
                    {str(i): (1 if i == 7 else 0) for i in range(1, 13)}
                and id7_review['amplitude_deg'] == 1.,
                'No same-boot ID7 disabled-only source and STOP evidence')"""
    body = body[:a] + replacement + body[b:]
    old_call = '''report['result'] = run_fullbody_step2(transports, expected, check, emit,
                    validated_review=step2_review, preflight_only=True)'''
    new_call = '''report['result'] = run_id7_step1(transports, expected, check, emit,
                    validated_review=id7_review, preflight_only=True)'''
    require(body.count(old_call) == 1, 'No singular disabled step2 call')
    body = body.replace(old_call, new_call)
    body = body.replace('disabled raw-step preflight; no active mode exists',
                        'disabled ID7 one-degree preflight; no active mode exists')
    require('run_fullbody_step2(transports' not in body
            and 'validated_review=step2_review' not in body
            and body.count('class DisabledPort:') == 1
            and body.count('if not args.preflight_only:') == 1,
            'Adapted wrapper lost disabled-only gate')
    ast.parse(body)
    return body


def build(source: Path, readonly_path: Path, abort_path: Path, output: Path) -> dict:
    review = make_review(source, readonly_path, abort_path)
    require(not output.exists() and output.name == OUTPUT_NAME,
            'Use a fresh designated ID7 disabled-only output directory')
    shutil.copytree(source, output, symlinks=False)
    shutil.copy2(readonly_path, output / 'id7-readonly-summary.json')
    shutil.copy2(abort_path, output / 'id7-active-step2-abort-summary.json')
    for module in ('rs05_leg_trial.py', 'rs05_fullbody_step2.py', 'rs05_id7_step1.py'):
        shutil.copy2(RUNTIME / module, output / 'singularitydog_hw' / module)
    shutil.copy2(ROOT / 'runtime' / 'tests' / 'test_rs05_fullbody_hold.py',
                 output / 'tests' / 'test_rs05_fullbody_hold.py')
    write_json(output / 'id7-review.json', review)
    wrapper = output / 'prepared_fullbody.py'
    body = adapt_wrapper(wrapper.read_text(), output.name, review['boot_id'])
    wrapper.write_text(body)
    require(not any(path.is_symlink() for path in output.rglob('*')),
            'Frozen output contains a symlink')
    pins = {str(path.relative_to(output)): sha(path)
            for path in sorted(output.rglob('*'))
            if path.is_file() and path not in (output / 'manifest.json', wrapper)}
    body, count = re.subn(r'^PINS = .*$', 'PINS = ' + repr(pins), body,
                          count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular output PINS')
    ast.parse(body)
    wrapper.write_text(body)
    manifest = {**pins, wrapper.name: sha(wrapper)}
    write_json(output / 'manifest.json', manifest)
    require(read_json(output / 'manifest.json') == manifest
            and all(sha(output / name) == digest for name, digest in manifest.items())
            and ast.literal_eval(re.search(r'^PINS = (.*)$', body, re.MULTILINE).group(1)) == pins,
            'Final frozen manifest mismatch')
    return {'package': str(output), 'boot_id': review['boot_id'],
            'disabled_only': True, 'active_output_authorized': False,
            'id7_joint_inspection_verified': False,
            'read_only_snapshot_sha256': sha(output / 'id7-readonly-summary.json'),
            'active_abort_sha256': sha(output / 'id7-active-step2-abort-summary.json'),
            'pinned_files': len(manifest), 'manifest_sha256': sha(output / 'manifest.json'),
            'wrapper_sha256': sha(wrapper), 'review_sha256': sha(output / 'id7-review.json')}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-disabled', type=Path, required=True)
    parser.add_argument('--readonly-summary', type=Path, required=True)
    parser.add_argument('--active-step2-abort', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.source_disabled, args.readonly_summary,
                           args.active_step2_abort, args.output), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
