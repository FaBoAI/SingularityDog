"""Build a local, disabled-only frozen RS05 raw-step preflight package.

This copies an already reviewed r17 disabled package, replaces its entry point
with a disabled-only two-degree preflight, and pins every source/evidence file.
It never opens a device or creates an active-motion authorization. The output
directory is local; transfer to the robot is a separate reviewed operation.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import shutil


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
REQUIRED_FLAGS = ('raw_direction_reviewed_for_diagnostic',
                  'swept_clearance_verified', 'support_stand_verified',
                  'feet_clear_verified', 'hands_clear_verified',
                  'physical_cutoff_ready')


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path):
    return json.loads(path.read_text())


def write_json(path: Path, data) -> None:
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def validate_source(source: Path) -> None:
    wrapper = source / 'prepared_fullbody.py'
    require(wrapper.is_file() and not wrapper.is_symlink(), 'Missing r17 disabled wrapper')
    text = wrapper.read_text()
    require('No enable or nonzero-gain writes' in text
            and 'class DisabledPort' in text
            and "if not args.preflight_only:" in text,
            'Source wrapper is not the disabled-only r17 template')
    match = re.search(r'^PINS = (.*)$', text, re.MULTILINE)
    require(match is not None, 'No source pins')
    pins = ast.literal_eval(match.group(1))
    manifest = read_json(source / 'manifest.json')
    require(type(pins) is dict and len(pins) >= 100 and type(manifest) is dict,
            'Source manifest is incomplete')
    require(manifest == {**pins, 'prepared_fullbody.py': sha(wrapper)},
            'Source manifest differs from r17 pins')
    for name, digest in pins.items():
        path = source / name
        require(path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Source pin mismatch: ' + name)


def validate_hold(hold: dict, digest: str, boot: str, uids: dict) -> dict:
    result = hold.get('result', {})
    require(hold.get('boot_id') == boot
            and hold.get('status') == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            and hold.get('errors') == []
            and hold.get('motor_enable_sent') is True
            and hold.get('motion_gain_sent') is True
            and hold.get('trial_device_closed') is True
            and hold.get('locks_released') is True
            and result.get('status') == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            and result.get('stop_confirmed') is True
            and result.get('errors') == []
            and result.get('gain_profile') == 'id4-id10-kp4'
            and result.get('review', {}).get('motor_uids') == uids,
            'Current hold is not a complete same-boot all-twelve success')
    centers = {}
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        worker = result.get('workers', {}).get(bus, {})
        expected = {str(mid) for mid in ids}
        stops = worker.get('stop_reports', {})
        require(worker.get('completed') is True
                and worker.get('cycle_count') == 100
                and set(worker.get('centers', {})) == expected
                and set(stops) == expected
                and all(stops[mid].get('confirmed') is True for mid in expected),
                'Current hold six-axis record incomplete: ' + bus)
        centers.update(worker['centers'])
    require(len(digest) == 64 and set(centers) == {str(i) for i in range(1, 13)},
            'Hold digest or centers missing')
    return centers


def make_review(source: Path, hold_path: Path, hold_evidence_path: Path,
                candidate_path: Path) -> dict:
    source_review = read_json(source / 'fullbody-review.json')
    hold = read_json(hold_path)
    hold_evidence = read_json(hold_evidence_path)
    candidate = read_json(candidate_path)
    boot = source_review['boot_id']
    uids = source_review['motor_uids']
    require(candidate.get('boot_id') == boot
            and candidate.get('output_allowed') is False
            and candidate.get('executed') is False
            and candidate.get('status') == 'OFFLINE_CANDIDATE_ONLY'
            and candidate.get('current_hold_source_sha256') == sha(hold_evidence_path)
            and hold_evidence.get('boot_id') == boot
            and hold_evidence.get('revised_active_hold', {}).get('summary_sha256') == sha(hold_path)
            and hold_evidence.get('revised_active_hold', {}).get('status')
                == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
            'Offline candidate/hold-evidence/actual-summary SHA chain differs')
    centers = validate_hold(hold, sha(hold_path), boot, uids)
    rows = candidate.get('rows')
    require(type(rows) is list and len(rows) == 12
            and {row.get('id') for row in rows} == set(range(1, 13)),
            'Candidate must specify twelve distinct raw directions')
    directions = {}
    for row in rows:
        mid = row['id']
        delta = row['candidate_end_raw_rad'] - row['start_raw_rad']
        require(abs(row['candidate_raw_step_deg']) == 2
                and (row['candidate_raw_step_deg'] > 0) == (delta > 0)
                and abs(abs(delta) - 2 * 3.141592653589793 / 180) < 1e-5
                and abs(row['start_raw_rad'] - centers[str(mid)]) < 1e-5,
                f'ID{mid} offline raw candidate differs from same-boot hold')
        directions[str(mid)] = 1 if delta > 0 else -1
    return {
        'schema': 'rs05-fullbody-step2-review-v1',
        'scope': 'supported-fullbody-raw-2deg-50ms-diagnostic',
        'motor_ids': list(range(1, 13)), 'motor_uids': uids,
        'firmware': '0.5.0.13', 'sha256': sha(candidate_path),
        'review_complete': True, 'source_files_verified': True,
        'gain_profile': 'id4-id10-kp4', 'old_raw_target_reused': False,
        'calibration_verified': False, 'model_mapping_verified': False,
        'learned_policy_allowed': False, 'standing_allowed': False,
        'automatic_retry_allowed': False,
        **{flag: False for flag in REQUIRED_FLAGS},
        'current_hold_status': result_status(hold),
        'current_hold_stop_confirmed': True,
        'current_hold_summary_sha256': sha(hold_path),
        'current_hold_gain_profile': 'id4-id10-kp4',
        'supported_step_authorized': False,
        'boot_id': boot, 'current_hold_boot_id': boot,
        'start_raw_rad_by_id': centers, 'raw_direction_by_id': directions,
        'amplitude_deg': 2.,
        # The offline candidate is a direction hint, not a trusted source of
        # live start angles or an output authorization. Keep provenance visible.
        'offline_candidate_hold_provenance_verified': True,
        'offline_hold_evidence_sha256': sha(hold_evidence_path),
    }


def result_status(hold: dict) -> str:
    return hold['result']['status']


def adapt_wrapper(text: str, name: str, boot: str) -> str:
    old_base = re.search(r"^BASE = Path\('[^']+'\)$", text, re.MULTILINE)
    require(old_base is not None, 'No singular template BASE')
    text = text[:old_base.start()] + (
        f"BASE = Path('/home/jetson/singularitydog-tests/{name}')") + text[old_base.end():]
    text, count = re.subn(r"^BOOT = '[^']+'$", f"BOOT = '{boot}'", text, count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular template BOOT')
    old_import = 'from singularitydog_hw.rs05_fullbody_hold import run_fullbody_hold'
    require(text.count(old_import) == 1, 'No singular hold import')
    text = text.replace(old_import,
                        'from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2')
    old_modules = "'rs05_leg_trial', 'current_hold_review', 'rs05_bus_transport', 'rs05_fullbody_hold'"
    new_modules = "'rs05_leg_trial', 'current_hold_review', 'rs05_bus_transport', 'fullbody_step10_plan', 'rs05_fullbody_step2'"
    require(text.count(old_modules) == 1, 'Unexpected module verification list')
    text = text.replace(old_modules, new_modules)
    old_review = 'review = reviewed_assertions(BASE, expected, legacy)'
    require(text.count(old_review) == 1, 'No singular old review check')
    text = text.replace(old_review, '''reviewed_assertions(BASE, expected, legacy)
        candidate = json.loads((BASE / 'offline-raw-step2-candidate.json').read_text())
        hold_evidence = json.loads((BASE / 'fullbody-hold-evidence.json').read_text())
        step2_review = json.loads((BASE / 'step2-review.json').read_text())
        hold = json.loads((BASE / 'current-hold-summary.json').read_text())
        require(candidate['output_allowed'] is False and candidate['executed'] is False
                and step2_review['supported_step_authorized'] is False
                and all(step2_review[k] is False for k in
                        ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
                         'support_stand_verified', 'feet_clear_verified', 'hands_clear_verified',
                         'physical_cutoff_ready'))
                and step2_review['sha256'] == sha(BASE / 'offline-raw-step2-candidate.json')
                and step2_review['offline_candidate_hold_provenance_verified'] is True
                and candidate['current_hold_source_sha256'] == sha(BASE / 'fullbody-hold-evidence.json')
                and step2_review['current_hold_summary_sha256'] == sha(BASE / 'current-hold-summary.json')
                and hold_evidence['revised_active_hold']['summary_sha256'] == sha(BASE / 'current-hold-summary.json')
                and hold['boot_id'] == BOOT
                and hold['status'] == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
                and hold['result']['status'] == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
                and hold['result']['stop_confirmed'] is True
                and hold['result']['errors'] == []
                and hold['result']['review']['motor_uids'] == expected,
                'No same-boot verified hold or disabled-only step review')''')
    old_call = '''report['result'] = run_fullbody_hold(transports, expected, check, emit,
                    validated_review=review, reviewed_motor_ids=(5, 6, 8), preflight_only=True)'''
    new_call = '''report['result'] = run_fullbody_step2(transports, expected, check, emit,
                    validated_review=step2_review, preflight_only=True)'''
    require(text.count(old_call) == 1, 'No singular hold call')
    text = text.replace(old_call, new_call)
    text = text.replace('disabled12-axis preflight; no active mode exists',
                        'disabled raw-step preflight; no active mode exists')
    text = text.replace('This executable cannot run the active hold path.',
                        'This executable cannot run the active raw-step path.')
    require(text.count('class DisabledPort') == 1
            and text.count('if not args.preflight_only:') == 1
            and 'run_fullbody_hold' not in text,
            'Adapted wrapper lost its disabled-only gate')
    ast.parse(text)
    return text


def build(source: Path, hold_path: Path, hold_evidence_path: Path,
          candidate_path: Path, output: Path) -> dict:
    validate_source(source)
    require(not output.exists() and output.name == 'fullbody-step2-disabled-20260926-r2',
            'Use a fresh designated disabled-only output directory')
    review = make_review(source, hold_path, hold_evidence_path, candidate_path)
    shutil.copytree(source, output, symlinks=False)
    shutil.copy2(hold_path, output / 'current-hold-summary.json')
    shutil.copy2(hold_evidence_path, output / 'fullbody-hold-evidence.json')
    shutil.copy2(candidate_path, output / 'offline-raw-step2-candidate.json')
    for module in ('fullbody_step10_plan.py', 'rs05_fullbody_step2.py'):
        shutil.copy2(RUNTIME / module, output / 'singularitydog_hw' / module)
    write_json(output / 'step2-review.json', review)
    wrapper = output / 'prepared_fullbody.py'
    adapted = adapt_wrapper(wrapper.read_text(), output.name, review['boot_id'])
    wrapper.write_text(adapted)
    pinned = {str(path.relative_to(output)): sha(path)
              for path in sorted(output.rglob('*'))
              if path.is_file() and path not in (output / 'manifest.json', wrapper)}
    require(all(not path.is_symlink() for path in output.rglob('*')),
            'Frozen package contains a symlink')
    adapted, count = re.subn(r'^PINS = .*$', 'PINS = ' + repr(pinned),
                             adapted, count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular wrapper PINS')
    wrapper.write_text(adapted)
    ast.parse(adapted)
    manifest = {**pinned, wrapper.name: sha(wrapper)}
    write_json(output / 'manifest.json', manifest)
    # A second pass proves that the final frozen sources, including the
    # wrapper itself, have precisely the bytes recorded by the manifest.
    require(read_json(output / 'manifest.json') == manifest
            and all(sha(output / name) == digest for name, digest in manifest.items())
            and ast.literal_eval(re.search(r'^PINS = (.*)$', adapted, re.MULTILINE).group(1)) == pinned,
            'Final frozen manifest mismatch')
    return {'package': str(output), 'boot_id': review['boot_id'],
            'disabled_only': True, 'active_output_authorized': False,
            'offline_candidate_hold_provenance_verified': review['offline_candidate_hold_provenance_verified'],
            'pinned_files': len(manifest), 'manifest_sha256': sha(output / 'manifest.json'),
            'wrapper_sha256': sha(wrapper), 'review_sha256': sha(output / 'step2-review.json')}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-disabled', type=Path, required=True)
    parser.add_argument('--current-hold-summary', type=Path, required=True)
    parser.add_argument('--hold-evidence', type=Path, required=True)
    parser.add_argument('--offline-candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    report = build(args.source_disabled, args.current_hold_summary, args.hold_evidence,
                   args.offline_candidate, args.output)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
