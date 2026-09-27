"""File-only builder for a frozen, bounded raw 2-degree diagnostic package.

No device is opened. The frozen launcher retains the old same-boot evidence
checks, adds the successful new disabled preflight and puts a new exact-wire
packet gate immediately before every physical UART write.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import shutil

from build_fullbody_step2_disabled import (ROOT, REQUIRED_FLAGS, require, sha,
                                           read_json, write_json)


RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
TESTS = ROOT / 'runtime' / 'tests'


def validate_frozen(source: Path, wrapper_name: str, manifest_name: str) -> None:
    wrapper = source / wrapper_name
    require(wrapper.is_file() and not wrapper.is_symlink(), 'Missing frozen source wrapper')
    text = wrapper.read_text()
    match = re.search(r'^PINS = (.*)$', text, re.MULTILINE)
    require(match is not None, 'Frozen source has no pins')
    pins = ast.literal_eval(match.group(1))
    manifest = read_json(source / manifest_name)
    require(type(pins) is dict and len(pins) >= 100
            and manifest == {**pins, wrapper_name: sha(wrapper)},
            'Frozen source manifest and wrapper differ')
    for name, digest in pins.items():
        path = source / name
        require(path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Frozen source pin mismatch: ' + name)


def validate_step2_preflight(summary_path: Path, events_path: Path, boot: str) -> None:
    summary = read_json(summary_path)
    result = summary.get('result', {})
    require(summary.get('boot_id') == boot
            and summary.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary.get('preflight_only') is True
            and summary.get('motor_enable_sent') is False
            and summary.get('motion_gain_sent') is False
            and summary.get('trial_device_closed') is True
            and summary.get('locks_released') is True
            and summary.get('port_closes') == {'front': True, 'rear': True}
            and summary.get('errors') == [] and summary.get('signals') == []
            and summary.get('events_sha256') == sha(events_path)
            and result.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and result.get('preflight_only') is True
            and result.get('preflight_completed') is True
            and result.get('motion_completed') is False
            and result.get('stop_confirmed') is True
            and result.get('errors') == [],
            'New same-boot disabled step2 preflight did not pass')
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        worker = result.get('workers', {}).get(bus, {})
        stops = worker.get('stop_reports', {})
        require(worker.get('completed') is True
                and worker.get('cycle_count') == 20
                and set(stops) == {str(mid) for mid in ids}
                and all(stops[str(mid)].get('confirmed') is True for mid in ids),
                'New disabled six-axis preflight incomplete: ' + bus)
    require(0 < events_path.stat().st_size < 20_000_000,
            'New disabled preflight event log empty or too large')


def make_review(disabled_package: Path, summary_path: Path, events_path: Path,
                physical_path: Path) -> dict:
    disabled = read_json(disabled_package / 'step2-review.json')
    physical = read_json(physical_path)
    boot = disabled['boot_id']
    validate_step2_preflight(summary_path, events_path, boot)
    require(disabled['supported_step_authorized'] is False
            and disabled['old_raw_target_reused'] is False
            and disabled['offline_candidate_hold_provenance_verified'] is True
            and all(disabled[flag] is False for flag in REQUIRED_FLAGS),
            'Input package is not a disabled, reviewed raw-step source')
    require(physical.get('boot_id') == boot
            and physical.get('scope') == 'supported-all12-current-raw-plus-minus-2deg-diagnostic-only'
            and physical.get('raw_direction_by_id') == disabled['raw_direction_by_id']
            and physical.get('old_raw_target_reused') is False
            and physical.get('calibration_verified') is False
            and physical.get('model_mapping_verified') is False
            and physical.get('standing_allowed') is False
            and physical.get('learned_policy_allowed') is False
            and physical.get('automatic_retry_allowed') is False
            and physical.get('requires_fresh_pre_run_confirmation') is True
            and all(physical.get(flag) is True for flag in REQUIRED_FLAGS),
            'Physical review is not limited to the frozen two-degree raw diagnostic')
    return {**disabled, **{flag: True for flag in REQUIRED_FLAGS},
            'supported_step_authorized': True,
            'source_disabled_step2_review_sha256': sha(disabled_package / 'step2-review.json'),
            'source_step2_preflight_summary_sha256': sha(summary_path),
            'source_step2_preflight_events_sha256': sha(events_path),
            'source_physical_review_sha256': sha(physical_path)}


STEP2_SOURCE_AUDIT = '''
def verify_step2_sources(base, expected):
    """Pin disabled pass, actual hold, offline direction hint and physical scope."""
    required = {'step2-disabled-review.json', 'step2-active-review.json',
                'step2-preflight/summary.json', 'step2-preflight/events.jsonl',
                'physical-review.json', 'current-hold-summary.json',
                'fullbody-hold-evidence.json', 'offline-raw-step2-candidate.json'}
    require(required <= set(PINS), 'Raw-step evidence is not pinned')
    disabled = json.loads((base / 'step2-disabled-review.json').read_text())
    active = json.loads((base / 'step2-active-review.json').read_text())
    physical = json.loads((base / 'physical-review.json').read_text())
    candidate = json.loads((base / 'offline-raw-step2-candidate.json').read_text())
    hold_evidence = json.loads((base / 'fullbody-hold-evidence.json').read_text())
    hold = json.loads((base / 'current-hold-summary.json').read_text())
    summary = json.loads((base / 'step2-preflight/summary.json').read_text())
    events_path = base / 'step2-preflight/events.jsonl'
    flags = ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
             'support_stand_verified', 'feet_clear_verified', 'hands_clear_verified',
             'physical_cutoff_ready')
    clearance_extra = {}
    if disabled.get('role_group') == 'front-hip' and disabled.get('amplitude_deg') == 10.:
        workers = summary.get('result', {}).get('workers', {})
        require(type(workers) is dict and set(workers) == {'front', 'rear'},
                'Front-hip 10-degree preflight lacks measured buses')
        reference = {}
        for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
            centers = workers[bus].get('centers', {})
            require(type(centers) is dict and set(centers) == {str(i) for i in ids},
                    'Front-hip 10-degree preflight lacks measured centers')
            reference.update(centers)
        note = physical.get('start_tolerance_clearance_note')
        require(physical.get('clearance_reference_raw_rad_by_id') == reference
                and physical.get('clearance_reference_preflight_summary_sha256')
                    == sha(base / 'step2-preflight/summary.json')
                and type(physical.get('start_tolerance_clearance_verified_deg')) in (int, float)
                and physical['start_tolerance_clearance_verified_deg'] == 3.
                and type(note) is str and bool(note.strip()),
                'Front-hip 10-degree path is not tied to the physical +/-3-degree pose')
        clearance_extra = {
            'clearance_reference_raw_rad_by_id': reference,
            'clearance_reference_preflight_summary_sha256': sha(base / 'step2-preflight/summary.json'),
            'start_tolerance_clearance_verified_deg': 3.,
            'start_tolerance_clearance_note': note}
    require(disabled['boot_id'] == active['boot_id'] == physical['boot_id'] == BOOT
            and disabled['motor_uids'] == active['motor_uids'] == expected
            and disabled['supported_step_authorized'] is False
            and all(disabled[k] is False and physical[k] is True for k in flags)
            and physical['raw_direction_by_id'] == disabled['raw_direction_by_id']
            and active == {**disabled, **{k: True for k in flags}, **clearance_extra,
                           'supported_step_authorized': True,
                           'source_disabled_step2_review_sha256': sha(base / 'step2-disabled-review.json'),
                           'source_step2_preflight_summary_sha256': sha(base / 'step2-preflight/summary.json'),
                           'source_step2_preflight_events_sha256': sha(events_path),
                           'source_physical_review_sha256': sha(base / 'physical-review.json')},
            'Active raw-step review differs from the pinned disabled derivative')
    require(candidate['output_allowed'] is False and candidate['executed'] is False
            and candidate['current_hold_source_sha256'] == sha(base / 'fullbody-hold-evidence.json')
            and hold_evidence['revised_active_hold']['summary_sha256'] == sha(base / 'current-hold-summary.json')
            and hold['boot_id'] == BOOT and hold['status'] == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            and hold['result']['stop_confirmed'] is True
            and active['current_hold_summary_sha256'] == sha(base / 'current-hold-summary.json'),
            'Raw-step hold and offline evidence SHA chain mismatch')
    result = summary.get('result', {})
    require(summary.get('boot_id') == BOOT
            and summary.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary.get('preflight_only') is True
            and summary.get('motor_enable_sent') is False
            and summary.get('motion_gain_sent') is False
            and summary.get('trial_device_closed') is True
            and summary.get('locks_released') is True
            and summary.get('port_closes') == {'front': True, 'rear': True}
            and summary.get('errors') == [] and summary.get('signals') == []
            and summary.get('events_sha256') == sha(events_path)
            and result.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and result.get('preflight_completed') is True
            and result.get('motion_completed') is False
            and result.get('stop_confirmed') is True
            and result.get('errors') == [],
            'New disabled preflight summary did not pass')
    for bus, ids in BUS_IDS.items():
        worker = result.get('workers', {}).get(bus, {})
        stops = worker.get('stop_reports', {})
        require(worker.get('completed') is True and worker.get('cycle_count') == 20
                and set(stops) == {str(i) for i in ids}
                and all(stops[str(i)]['confirmed'] is True for i in ids),
                'New disabled preflight lacks a six-axis STOP: ' + bus)
    spec = importlib.util.spec_from_file_location('step2_disabled_wire_proof', base / 'prepared_fullbody.py')
    proof = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(proof)
    class AuditSink:
        def write(self, wire):
            return len(wire)
    gates = {bus: proof.DisabledPort(AuditSink(), ids) for bus, ids in BUS_IDS.items()}
    rows = events_path.read_text().splitlines()
    require(0 < len(rows) <= 50000, 'New disabled preflight events absent/oversized')
    tx_count = 0
    for line in rows:
        event = json.loads(line)
        if event.get('kind') == 'can_tx':
            require(event.get('bus') in gates, 'New preflight TX has no exact bus')
            gates[event['bus']].write(bytes.fromhex(event['hex']))
            tx_count += 1
    require(tx_count > 0, 'No new preflight UART writes to audit')
    return {'summary_sha256': sha(base / 'step2-preflight/summary.json'),
            'events_sha256': sha(events_path), 'boot_id': BOOT,
            'tx_count': tx_count, 'enable_sent': False, 'motion_gain_sent': False}


'''


def adapt_wrapper(text: str, name: str, boot: str) -> str:
    require(text.count('class ActiveHoldState:') == 1
            and text.count('class ActivePort:') == 1
            and text.count('def main(argv=None):') == 1,
            'Unexpected r17 active wrapper template')
    text = text.replace('"""Unfrozen ID4+ID10 Kp4 candidate for one supported current-position5s hold.\n\nNo L target, policy target, stand transition, gain escalation or retry exists.\nFreezing and execution are separate parent-reviewed actions after a passing\nsame-boot disabled preflight. An empty manifest/UNFROZEN boot cannot execute.\n"""',
        '"""Frozen supported all-twelve raw 2-degree diagnostic.\n\nNo L target, learned policy, standing claim, gain escalation or retry exists.\nThe old hold gate remains only to audit its historical disabled preflight.\nA separate exact-wire packet gate guards this finite raw trajectory.\n"""')
    text, n = re.subn(r"^BASE = Path\('[^']+'\)$",
                      f"BASE = Path('/home/jetson/singularitydog-tests/{name}')",
                      text, count=1, flags=re.MULTILINE)
    require(n == 1, 'No singular BASE')
    text, n = re.subn(r"^BOOT = '[^']+'$", f"BOOT = '{boot}'", text,
                      count=1, flags=re.MULTILINE)
    require(n == 1, 'No singular BOOT')
    old_complete = '''def completed_hold(result):
    return (result.get('status') == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            and result.get('motion_completed') is True
            and result.get('preflight_only') is False
            and result.get('stop_confirmed') is True
            and result.get('errors') == [])'''
    new_complete = '''def completed_step2(result):
    return (result.get('status') == 'RAW_STEP2_COMPLETED_RESET_CONFIRMED'
            and result.get('motion_completed') is True
            and result.get('preflight_only') is False
            and result.get('stop_confirmed') is True
            and result.get('raw_diagnostic_only') is True
            and result.get('standing_allowed') is False
            and result.get('learned_policy_allowed') is False
            and result.get('errors') == [])'''
    require(text.count(old_complete) == 1, 'No singular completion check')
    text = text.replace(old_complete, new_complete)
    text = text.replace('\ndef main(argv=None):', STEP2_SOURCE_AUDIT + 'def main(argv=None):')
    text = text.replace("ap.add_argument('--execute-current-hold', action='store_true')",
        "ap.add_argument('--execute-raw-step2', action='store_true')\n"
        "    ap.add_argument('--clearance-confirmed', action='store_true')\n"
        "    ap.add_argument('--cutoff-ready', action='store_true')")
    text = text.replace('if not args.execute_current_hold:', 'if not args.execute_raw_step2:')
    text = text.replace("print('PLAN_ONLY: supported fixed-current12-axis5s hold; unfrozen template has no execution authority')",
                        "print('PLAN_ONLY: supported raw2-degree diagnostic; no motion without explicit flags')")
    text = text.replace('if not args.supported or not args.hands_off:',
                        'if not args.supported or not args.hands_off or not args.clearance_confirmed or not args.cutoff_ready:')
    text = text.replace("ap.error('Physical support and immediate hands-off confirmation are required')",
                        "ap.error('Current support, hands-off, swept clearance and immediate40V cutoff are required')")
    text = text.replace('supported_current_hold_only=True, learned_policy_allowed=False, standing_allowed=False,',
                        'raw_diagnostic_only=True, learned_policy_allowed=False, standing_allowed=False,')
    old_review = "review = reviewed_assertions(BASE, expected, legacy)\n        packet_state = ActiveHoldState(authorized=review['supported_hold_authorized'])"
    new_review = '''reviewed_assertions(BASE, expected, legacy)
        report['step2_preflight_source_audit'] = verify_step2_sources(BASE, expected)
        step2_review = json.loads((BASE / 'step2-active-review.json').read_text())
        from singularitydog_hw.rs05_step2_packet_gate import Step2PacketState, Step2PacketPort
        packet_state = Step2PacketState(step2_review)'''
    require(text.count(old_review) == 1, 'No singular old review setup')
    text = text.replace(old_review, new_review)
    text = text.replace('from singularitydog_hw.rs05_fullbody_hold import run_fullbody_hold',
                        'from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2')
    old_proof = '    proof.verify_files(base)\n'
    require(text.count(old_proof) == 1, 'No singular inherited source proof')
    text = text.replace(old_proof,
        "    proof.PINS = {**proof.PINS, 'singularitydog_hw/rs05_joint_trial.py': "
        "PINS['singularitydog_hw/rs05_joint_trial.py']}\n" + old_proof, 1)
    old_modules = "'rs05_leg_trial', 'current_hold_review', 'rs05_bus_transport', 'rs05_fullbody_hold'"
    new_modules = "'rs05_leg_trial', 'current_hold_review', 'rs05_bus_transport', 'rs05_fullbody_step2', 'fullbody_step10_plan', 'rs05_step2_packet_gate'"
    require(text.count(old_modules) == 1, 'No singular module audit')
    text = text.replace(old_modules, new_modules)
    text = text.replace('transports[bus] = BusTrialTransport(ActivePort(p, legacy.BUS_IDS[bus], packet_state),',
                        'transports[bus] = BusTrialTransport(Step2PacketPort(p, bus, packet_state),')
    old_call = '''report['result'] = run_fullbody_hold(transports, expected, check, emit,
                    validated_review=review, reviewed_motor_ids=(5, 6, 8), preflight_only=False,
                    gain_profile=GAIN_PROFILE)'''
    new_call = '''report['result'] = run_fullbody_step2(transports, expected, check, emit,
                    validated_review=step2_review, preflight_only=False)'''
    require(text.count(old_call) == 1, 'No singular hold invocation')
    text = text.replace(old_call, new_call)
    old_audit = '''require(report['packet_gate']['enabled_ids'] == list(range(1, 13))
                and set(report['packet_gate']['active_counts']) == set(range(1, 13))
                and all(n == 100 for n in report['packet_gate']['active_counts'].values()),
                'The single fixed100-tick hold did not complete on all12 axes')'''
    new_audit = '''require(report['packet_gate']['enabled_ids'] == list(range(1, 13))
                and report['packet_gate']['next_tick_by_bus'] == {'front': 180, 'rear': 180}
                and report['packet_gate']['next_motor_index_by_bus'] == {'front': 0, 'rear': 0},
                'The single finite180-tick raw step did not complete on both buses')'''
    require(text.count(old_audit) == 1, 'No singular active count audit')
    text = text.replace(old_audit, new_audit)
    text = text.replace('require(completed_hold(report[\'result\']),',
                        'require(completed_step2(report[\'result\']),')
    text = text.replace("'Supported current-position hold did not pass; see per-bus results'",
                        "'Supported raw2-degree diagnostic did not pass; see per-bus results'")
    text = text.replace("completed_hold(report.get('result', {}))", "completed_step2(report.get('result', {}))")
    require('run_fullbody_hold(transports' not in text
            and 'ActivePort(p, legacy.BUS_IDS' not in text
            and 'Step2PacketPort(p, bus, packet_state)' in text,
            'Adapted active wrapper retained an old motion path')
    ast.parse(text)
    return text


def build(source: Path, disabled_package: Path, preflight_log: Path,
          physical_path: Path, output: Path) -> dict:
    validate_frozen(source, 'prepared_current_hold.py', 'active-manifest.json')
    validate_frozen(disabled_package, 'prepared_fullbody.py', 'manifest.json')
    require(not output.exists() and output.name == 'fullbody-step2-active-20260926-r2',
            'Use the fresh designated active package directory')
    summary_path, events_path = preflight_log / 'summary.json', preflight_log / 'events.jsonl'
    active_review = make_review(disabled_package, summary_path, events_path, physical_path)
    shutil.copytree(source, output, symlinks=False)
    for module in ('rs05_fullbody_step2.py', 'rs05_joint_trial.py',
                   'fullbody_step10_plan.py',
                   'rs05_step2_packet_gate.py'):
        shutil.copy2(RUNTIME / module, output / 'singularitydog_hw' / module)
    for test in ('test_rs05_fullbody_step2.py', 'test_rs05_step2_packet_gate.py'):
        shutil.copy2(TESTS / test, output / 'tests' / test)
    for src, dest in ((disabled_package / 'step2-review.json', output / 'step2-disabled-review.json'),
                      (disabled_package / 'offline-raw-step2-candidate.json', output / 'offline-raw-step2-candidate.json'),
                      (disabled_package / 'current-hold-summary.json', output / 'current-hold-summary.json'),
                      (disabled_package / 'fullbody-hold-evidence.json', output / 'fullbody-hold-evidence.json'),
                      (physical_path, output / 'physical-review.json')):
        shutil.copy2(src, dest)
    step2_preflight = output / 'step2-preflight'
    step2_preflight.mkdir()
    shutil.copy2(summary_path, step2_preflight / 'summary.json')
    shutil.copy2(events_path, step2_preflight / 'events.jsonl')
    write_json(output / 'step2-active-review.json', active_review)
    wrapper = output / 'prepared_current_hold.py'
    adapted = adapt_wrapper(wrapper.read_text(), output.name, active_review['boot_id'])
    wrapper.write_text(adapted)
    pinned = {str(path.relative_to(output)): sha(path)
              for path in sorted(output.rglob('*'))
              if path.is_file() and path not in (output / 'active-manifest.json', wrapper)}
    require(all(not path.is_symlink() for path in output.rglob('*')),
            'Active package contains a symlink')
    adapted, count = re.subn(r'^PINS = .*$', 'PINS = ' + repr(pinned),
                             adapted, count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular active PINS')
    wrapper.write_text(adapted)
    ast.parse(adapted)
    manifest = {**pinned, wrapper.name: sha(wrapper)}
    write_json(output / 'active-manifest.json', manifest)
    require(read_json(output / 'active-manifest.json') == manifest
            and all(sha(output / name) == digest for name, digest in manifest.items())
            and ast.literal_eval(re.search(r'^PINS = (.*)$', adapted, re.MULTILINE).group(1)) == pinned,
            'Final active source manifest mismatch')
    return {'package': str(output), 'boot_id': active_review['boot_id'],
            'raw_diagnostic_only': True, 'standing_allowed': False,
            'pinned_files': len(manifest), 'manifest_sha256': sha(output / 'active-manifest.json'),
            'wrapper_sha256': sha(wrapper), 'review_sha256': sha(output / 'step2-active-review.json'),
            'preflight_summary_sha256': sha(summary_path),
            'preflight_events_sha256': sha(events_path)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-active', type=Path, required=True)
    parser.add_argument('--disabled-package', type=Path, required=True)
    parser.add_argument('--preflight-log', type=Path, required=True)
    parser.add_argument('--physical-review', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.source_active, args.disabled_package,
                           args.preflight_log, args.physical_review, args.output), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
