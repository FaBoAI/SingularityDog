"""Build a frozen ID7-only raw one-degree package after disabled proof.

This builder performs file-only work. It refuses to freeze any active package
until a new same-boot ID7 disabled preflight, its byte log, and an independent
current physical review all pass. It never opens a device or runs a motor.
"""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import re
import shutil

from build_fullbody_step2_active import validate_frozen
from build_id7_step1_disabled import (FLAGS, OUTPUT_NAME as DISABLED_NAME, ROOT,
                                      require, sha, read_json, write_json)


RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
TESTS = ROOT / 'runtime' / 'tests'
ACTIVE_NAME = 'id7-step1-active-20260926-r9'
SOURCE_ACTIVE_NAME = 'fullbody-step2-active-20260926-r2'
PHYSICAL_SCHEMA = 'singularitydog.id7-step1-physical-review.v1'
PHYSICAL_SCOPE = 'supported-id7-current-raw-plus-1deg-other11-hold-diagnostic-only'


def validate_id7_preflight(disabled: Path, summary_path: Path,
                           events_path: Path, boot: str) -> dict:
    summary = read_json(summary_path)
    result = summary.get('result', {})
    review = read_json(disabled / 'id7-review.json')
    require(summary.get('boot_id') == boot
            and summary.get('wrapper_sha256') == sha(disabled / 'prepared_fullbody.py')
            and summary.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary.get('preflight_only') is True
            and summary.get('output_allowed') is False
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
            and result.get('errors') == []
            and result.get('review') == review
            and result.get('gain_profile') == 'id4-id10-kp4'
            and result.get('raw_diagnostic_only') is True,
            'New same-boot ID7 disabled preflight did not pass')
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        worker = result.get('workers', {}).get(bus, {})
        stops = worker.get('stop_reports', {})
        windows = worker.get('settled_windows', {})
        require(worker.get('completed') is True and worker.get('errors') == []
                and worker.get('cycle_count') == 20
                and set(stops) == {str(mid) for mid in ids}
                and all(stops[str(mid)].get('confirmed') is True for mid in ids)
                and set(windows) == ({'FR', 'FL'} if bus == 'front' else {'RR', 'RL'})
                and all(row.get('passed') is True for row in windows.values()),
                f'ID7 disabled preflight lacks settled window or STOP on {bus}')
    require(0 < events_path.stat().st_size < 20_000_000,
            'ID7 disabled event log empty or too large')
    return summary


def make_review(disabled: Path, summary_path: Path, events_path: Path,
                physical_path: Path) -> dict:
    review = read_json(disabled / 'id7-review.json')
    physical = read_json(physical_path)
    boot = review['boot_id']
    validate_id7_preflight(disabled, summary_path, events_path, boot)
    require(review['schema'] == 'rs05-id7-step1-review-v1'
            and review['scope'] == 'supported-id7-raw-1deg-other11-hold-50ms-diagnostic'
            and review['amplitude_deg'] == 1.
            and review['raw_direction_by_id']
                == {str(mid): (1 if mid == 7 else 0) for mid in range(1, 13)}
            and review['supported_step_authorized'] is False
            and review['id7_joint_inspection_verified'] is False
            and review['old_raw_target_reused'] is False
            and all(review[flag] is False for flag in FLAGS),
            'Input package is not the disabled ID7-only source')
    require(physical.get('schema') == PHYSICAL_SCHEMA
            and physical.get('scope') == PHYSICAL_SCOPE
            and physical.get('boot_id') == boot
            and physical.get('motor_uids') == review['motor_uids']
            and physical.get('raw_direction_by_id') == review['raw_direction_by_id']
            and physical.get('source_disabled_review_sha256') == sha(disabled / 'id7-review.json')
            and physical.get('source_disabled_preflight_summary_sha256') == sha(summary_path)
            and physical.get('source_active_step2_abort_sha256')
                == review['source_active_step2_abort_sha256']
            and physical.get('id7_joint_inspection_verified') is True
            and physical.get('swept_clearance_verified') is True
            and physical.get('support_stand_verified') is True
            and physical.get('feet_clear_verified') is True
            and physical.get('hands_clear_verified') is True
            and physical.get('physical_cutoff_ready') is True
            and physical.get('raw_direction_reviewed_for_diagnostic') is True
            and physical.get('requires_fresh_pre_run_confirmation') is True
            and physical.get('old_raw_target_reused') is False
            and physical.get('calibration_verified') is False
            and physical.get('model_mapping_verified') is False
            and physical.get('standing_allowed') is False
            and physical.get('learned_policy_allowed') is False
            and physical.get('automatic_retry_allowed') is False,
            'ID7 physical review is missing, stale, or broader than one-degree diagnostic')
    return {**review, **{flag: True for flag in FLAGS},
            'id7_joint_inspection_verified': True,
            'supported_step_authorized': True,
            'source_disabled_review_sha256': sha(disabled / 'id7-review.json'),
            'source_disabled_preflight_summary_sha256': sha(summary_path),
            'source_disabled_preflight_events_sha256': sha(events_path),
            'source_physical_review_sha256': sha(physical_path)}


ID7_SOURCE_AUDIT = '''
def verify_id7_sources(base, expected):
    """Prove the new disabled run and physical review before opening a UART."""
    required = {'id7-disabled/prepared_fullbody.py', 'id7-disabled/manifest.json',
                'id7-disabled/id7-review.json', 'id7-disabled/id7-readonly-summary.json',
                'id7-disabled/id7-active-step2-abort-summary.json',
                'id7-preflight/summary.json', 'id7-preflight/events.jsonl',
                'id7-physical-review.json', 'id7-active-review.json'}
    require(required <= set(PINS), 'ID7 active evidence is not pinned')
    disabled_base = base / 'id7-disabled'
    # The older full-body proof must retain its exact source files. The ID7
    # implementation runs through separately pinned module copies instead of
    # replacing files that the older proof validates in this directory.
    leg_source = (disabled_base / 'singularitydog_hw/rs05_leg_trial.py').read_text()
    step_source = (disabled_base / 'singularitydog_hw/rs05_fullbody_step2.py').read_text()
    id7_source = (disabled_base / 'singularitydog_hw/rs05_id7_step1.py').read_text()
    require((base / 'singularitydog_hw/rs05_leg_trial_id7.py').read_text() == leg_source
            and (base / 'singularitydog_hw/rs05_fullbody_step2_id7.py').read_text()
                == step_source.replace('from .rs05_leg_trial import',
                                       'from .rs05_leg_trial_id7 import')
            and (base / 'singularitydog_hw/rs05_id7_step1.py').read_text()
                == id7_source.replace('from .rs05_fullbody_step2 import',
                                      'from .rs05_fullbody_step2_id7 import'),
            'ID7-specific modules differ from the passing disabled source')
    spec = importlib.util.spec_from_file_location('id7_disabled_wire_proof',
                                                  disabled_base / 'prepared_fullbody.py')
    proof = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(proof)
    require(proof.BOOT == BOOT, 'ID7 disabled proof belongs to another boot')
    proof.verify_files(disabled_base)
    disabled_manifest = json.loads((disabled_base / 'manifest.json').read_text())
    require(disabled_manifest == {**proof.PINS,
             'prepared_fullbody.py': sha(disabled_base / 'prepared_fullbody.py')},
            'ID7 disabled proof manifest changed')
    disabled = json.loads((disabled_base / 'id7-review.json').read_text())
    active = json.loads((base / 'id7-active-review.json').read_text())
    physical = json.loads((base / 'id7-physical-review.json').read_text())
    readonly = json.loads((disabled_base / 'id7-readonly-summary.json').read_text())
    abort = json.loads((disabled_base / 'id7-active-step2-abort-summary.json').read_text())
    summary = json.loads((base / 'id7-preflight/summary.json').read_text())
    events_path = base / 'id7-preflight/events.jsonl'
    flags = ('raw_direction_reviewed_for_diagnostic', 'swept_clearance_verified',
             'support_stand_verified', 'feet_clear_verified', 'hands_clear_verified',
             'physical_cutoff_ready')
    require(disabled['boot_id'] == active['boot_id'] == physical['boot_id'] == BOOT
            and readonly['boot_id'] == abort['boot_id'] == BOOT
            and disabled['motor_uids'] == active['motor_uids'] == physical['motor_uids'] == expected
            and readonly['uids'] == expected
            and disabled['schema'] == active['schema'] == 'rs05-id7-step1-review-v1'
            and disabled['scope'] == active['scope']
                == 'supported-id7-raw-1deg-other11-hold-50ms-diagnostic'
            and disabled['supported_step_authorized'] is False
            and disabled['id7_joint_inspection_verified'] is False
            and all(disabled[k] is False and physical[k] is True for k in flags)
            and physical['id7_joint_inspection_verified'] is True
            and physical['raw_direction_by_id'] == disabled['raw_direction_by_id']
            and disabled['raw_direction_by_id'] ==
                {str(i): (1 if i == 7 else 0) for i in range(1, 13)}
            and disabled['start_raw_rad_by_id'] ==
                {k: v['last_rad'] for k, v in readonly['positions'].items()}
            and disabled['sha256'] == sha(disabled_base / 'id7-readonly-summary.json')
            and disabled['source_active_step2_abort_sha256']
                == sha(disabled_base / 'id7-active-step2-abort-summary.json')
            and active == {**disabled, **{k: True for k in flags},
                           'id7_joint_inspection_verified': True,
                           'supported_step_authorized': True,
                           'source_disabled_review_sha256': sha(disabled_base / 'id7-review.json'),
                           'source_disabled_preflight_summary_sha256': sha(base / 'id7-preflight/summary.json'),
                           'source_disabled_preflight_events_sha256': sha(events_path),
                           'source_physical_review_sha256': sha(base / 'id7-physical-review.json')},
            'ID7 active review differs from the pinned disabled derivative')
    require(physical['source_disabled_review_sha256'] == sha(disabled_base / 'id7-review.json')
            and physical['source_disabled_preflight_summary_sha256']
                == sha(base / 'id7-preflight/summary.json')
            and physical['source_active_step2_abort_sha256']
                == sha(disabled_base / 'id7-active-step2-abort-summary.json')
            and physical['schema'] == 'singularitydog.id7-step1-physical-review.v1'
            and physical['scope']
                == 'supported-id7-current-raw-plus-1deg-other11-hold-diagnostic-only'
            and physical['requires_fresh_pre_run_confirmation'] is True
            and all(physical[k] is False for k in
                    ('old_raw_target_reused', 'calibration_verified', 'model_mapping_verified',
                     'standing_allowed', 'learned_policy_allowed', 'automatic_retry_allowed')),
            'ID7 physical scope or evidence source changed')
    require(abort['result']['status'] == 'ABORTED'
            and abort['result']['stop_confirmed'] is True
            and any('ID7 raw step excursion exceeded2.5deg' in e
                    for e in abort['result']['errors']),
            'Prior ID7 overshoot and STOP source changed')
    result = summary.get('result', {})
    require(summary['boot_id'] == BOOT
            and summary['wrapper_sha256'] == sha(disabled_base / 'prepared_fullbody.py')
            and summary['status'] == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary['preflight_only'] is True
            and summary['output_allowed'] is False
            and summary['motor_enable_sent'] is False
            and summary['motion_gain_sent'] is False
            and summary['trial_device_closed'] is True
            and summary['locks_released'] is True
            and summary['port_closes'] == {'front': True, 'rear': True}
            and summary['errors'] == [] and summary['signals'] == []
            and summary['events_sha256'] == sha(events_path)
            and result['status'] == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and result['preflight_only'] is True
            and result['preflight_completed'] is True
            and result['motion_completed'] is False
            and result['stop_confirmed'] is True
            and result['errors'] == [] and result['review'] == disabled,
            'ID7 disabled run did not pass')
    for bus, ids in BUS_IDS.items():
        worker = result['workers'][bus]
        stops = worker['stop_reports']
        windows = worker['settled_windows']
        require(worker['completed'] is True and worker['errors'] == []
                and worker['cycle_count'] == 20
                and set(stops) == {str(i) for i in ids}
                and all(stops[str(i)]['confirmed'] is True for i in ids)
                and set(windows) == ({'FR', 'FL'} if bus == 'front' else {'RR', 'RL'})
                and all(row['passed'] is True for row in windows.values()),
                'ID7 disabled run incomplete on ' + bus)
    class AuditSink:
        def write(self, wire):
            return len(wire)
    gates = {bus: proof.DisabledPort(AuditSink(), ids) for bus, ids in BUS_IDS.items()}
    rows = events_path.read_text().splitlines()
    require(0 < len(rows) <= 50000, 'ID7 disabled events absent or oversized')
    tx_count = 0
    for line in rows:
        event = json.loads(line)
        if event.get('kind') == 'can_tx':
            require(event.get('bus') in gates, 'ID7 disabled TX has no exact bus')
            gates[event['bus']].write(bytes.fromhex(event['hex']))
            tx_count += 1
    require(tx_count > 0, 'No ID7 disabled UART writes to audit')
    return {'summary_sha256': sha(base / 'id7-preflight/summary.json'),
            'events_sha256': sha(events_path), 'boot_id': BOOT,
            'tx_count': tx_count, 'enable_sent': False, 'motion_gain_sent': False}


'''


def adapt_wrapper(body: str, name: str, boot: str) -> str:
    require(body.count('def completed_step2(result):') == 1
            and body.count('def verify_step2_sources(base, expected):') == 1
            and body.count('def main(argv=None):') == 1,
            'Unexpected frozen r2 active wrapper template')
    body, count = re.subn(r"^BASE = Path\('[^']+'\)$",
                          f"BASE = Path('/home/jetson/singularitydog-tests/{name}')",
                          body, count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular active BASE')
    body, count = re.subn(r"^BOOT = '[^']+'$", f"BOOT = '{boot}'", body,
                          count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular active BOOT')
    body = body.replace('Frozen supported all-twelve raw 2-degree diagnostic.',
                        'Frozen supported ID7-only raw one-degree diagnostic.')
    old_complete = '''def completed_step2(result):
    return (result.get('status') == 'RAW_STEP2_COMPLETED_RESET_CONFIRMED'
            and result.get('motion_completed') is True
            and result.get('preflight_only') is False
            and result.get('stop_confirmed') is True
            and result.get('raw_diagnostic_only') is True
            and result.get('standing_allowed') is False
            and result.get('learned_policy_allowed') is False
            and result.get('errors') == [])'''
    new_complete = '''def completed_id7_step1(result):
    return (result.get('status') == 'RAW_ID7_STEP1_COMPLETED_RESET_CONFIRMED'
            and result.get('motion_completed') is True
            and result.get('preflight_only') is False
            and result.get('stop_confirmed') is True
            and result.get('raw_diagnostic_only') is True
            and result.get('moving_motor_ids') == [7]
            and result.get('amplitude_deg') == 1.
            and result.get('standing_allowed') is False
            and result.get('learned_policy_allowed') is False
            and result.get('errors') == [])'''
    require(body.count(old_complete) == 1, 'No singular old completion predicate')
    body = body.replace(old_complete, new_complete)
    body = body.replace('\ndef main(argv=None):', ID7_SOURCE_AUDIT + 'def main(argv=None):')
    body = body.replace("ap.add_argument('--execute-raw-step2', action='store_true')",
                        "ap.add_argument('--execute-id7-step1', action='store_true')")
    body = body.replace('if not args.execute_raw_step2:', 'if not args.execute_id7_step1:')
    body = body.replace('PLAN_ONLY: supported raw2-degree diagnostic; no motion without explicit flags',
                        'PLAN_ONLY: ID7-only raw1-degree diagnostic; no motion without explicit flags')
    old_review = '''        report['step2_preflight_source_audit'] = verify_step2_sources(BASE, expected)
        step2_review = json.loads((BASE / 'step2-active-review.json').read_text())
        from singularitydog_hw.rs05_step2_packet_gate import Step2PacketState, Step2PacketPort
        packet_state = Step2PacketState(step2_review)'''
    new_review = '''        report['step2_preflight_source_audit'] = verify_step2_sources(BASE, expected)
        report['id7_preflight_source_audit'] = verify_id7_sources(BASE, expected)
        id7_review = json.loads((BASE / 'id7-active-review.json').read_text())
        from singularitydog_hw.rs05_step2_packet_gate import ID7Step1PacketState, Step2PacketPort
        packet_state = ID7Step1PacketState(id7_review)'''
    require(body.count(old_review) == 1, 'No singular source review setup')
    body = body.replace(old_review, new_review)
    body = body.replace('from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2',
                        'from singularitydog_hw.rs05_id7_step1 import run_id7_step1')
    old_modules = "'rs05_fullbody_step2', 'fullbody_step10_plan', 'rs05_step2_packet_gate')"
    new_modules = "'rs05_fullbody_step2', 'rs05_leg_trial_id7', 'rs05_fullbody_step2_id7', 'rs05_id7_step1', 'fullbody_step10_plan', 'rs05_step2_packet_gate')"
    require(body.count(old_modules) == 1, 'No singular module audit list')
    body = body.replace(old_modules, new_modules)
    old_module_check = """            require(Path(sys.modules['singularitydog_hw.' + name].__file__).resolve()
                    == BASE / 'singularitydog_hw' / (name + '.py'), 'Unexpected imported module')"""
    new_module_check = """            module = importlib.import_module('singularitydog_hw.' + name)
            require(Path(module.__file__).resolve()
                    == BASE / 'singularitydog_hw' / (name + '.py'), 'Unexpected imported module')"""
    require(body.count(old_module_check) == 1, 'Unexpected module-origin check')
    body = body.replace(old_module_check, new_module_check)
    old_call = '''report['result'] = run_fullbody_step2(transports, expected, check, emit,
                    validated_review=step2_review, preflight_only=False)'''
    new_call = '''report['result'] = run_id7_step1(transports, expected, check, emit,
                    validated_review=id7_review, preflight_only=False)'''
    require(body.count(old_call) == 1, 'No singular raw-step call')
    body = body.replace(old_call, new_call)
    body = body.replace('The single finite180-tick raw step did not complete on both buses',
                        'The single finite180-tick ID7 step did not complete on both buses')
    body = body.replace("require(completed_step2(report['result']),",
                        "require(completed_id7_step1(report['result']),")
    body = body.replace("'Supported raw2-degree diagnostic did not pass; see per-bus results'",
                        "'Supported ID7 one-degree diagnostic did not pass; see per-bus results'")
    body = body.replace("completed_step2(report.get('result', {}))",
                        "completed_id7_step1(report.get('result', {}))")
    require('run_fullbody_step2(transports' not in body
            and 'validated_review=step2_review' not in body
            and 'ID7Step1PacketState(id7_review)' in body
            and 'Step2PacketPort(p, bus, packet_state)' in body
            and 'verify_id7_sources(BASE, expected)' in body,
            'ID7 active wrapper retained an old motion path')
    ast.parse(body)
    return body


def build(source_active: Path, disabled: Path, preflight_log: Path,
          physical_path: Path, output: Path) -> dict:
    require(source_active.name == SOURCE_ACTIVE_NAME and disabled.name == DISABLED_NAME,
            'Use the exact r2 active and r3 ID7 disabled packages')
    validate_frozen(source_active, 'prepared_current_hold.py', 'active-manifest.json')
    validate_frozen(disabled, 'prepared_fullbody.py', 'manifest.json')
    require(not any(path.is_symlink() for package in (source_active, disabled)
                    for path in package.rglob('*')),
            'Frozen source package contains a symlink')
    for module in ('rs05_leg_trial.py', 'rs05_fullbody_step2.py', 'rs05_id7_step1.py'):
        require(sha(RUNTIME / module) == sha(disabled / 'singularitydog_hw' / module),
                'Active runner differs from the passing disabled source: ' + module)
    summary_path, events_path = preflight_log / 'summary.json', preflight_log / 'events.jsonl'
    active_review = make_review(disabled, summary_path, events_path, physical_path)
    require(not output.exists() and output.name == ACTIVE_NAME,
            'Use a fresh designated active ID7 output directory')
    shutil.copytree(source_active, output, symlinks=False)
    shutil.copytree(disabled, output / 'id7-disabled', symlinks=False)
    (output / 'id7-preflight').mkdir()
    shutil.copy2(summary_path, output / 'id7-preflight' / 'summary.json')
    shutil.copy2(events_path, output / 'id7-preflight' / 'events.jsonl')
    shutil.copy2(physical_path, output / 'id7-physical-review.json')
    # Keep the source-active's original rs05_leg_trial/fullbody_step2 files for
    # its pinned proof. Give ID7 byte-derived private copies of the passing
    # disabled implementation; only their internal imports are redirected.
    package_modules = output / 'singularitydog_hw'
    shutil.copy2(disabled / 'singularitydog_hw' / 'rs05_leg_trial.py',
                 package_modules / 'rs05_leg_trial_id7.py')
    step_source = (disabled / 'singularitydog_hw' / 'rs05_fullbody_step2.py').read_text()
    require(step_source.count('from .rs05_leg_trial import') == 1,
            'Unexpected ID7 settled-window import')
    (package_modules / 'rs05_fullbody_step2_id7.py').write_text(
        step_source.replace('from .rs05_leg_trial import', 'from .rs05_leg_trial_id7 import'))
    id7_source = (disabled / 'singularitydog_hw' / 'rs05_id7_step1.py').read_text()
    require(id7_source.count('from .rs05_fullbody_step2 import') == 1,
            'Unexpected ID7 step runner import')
    (package_modules / 'rs05_id7_step1.py').write_text(
        id7_source.replace('from .rs05_fullbody_step2 import',
                           'from .rs05_fullbody_step2_id7 import'))
    gate_source = (RUNTIME / 'rs05_step2_packet_gate.py').read_text()
    require(gate_source.count('from .rs05_fullbody_step2 import') == 1,
            'Unexpected ID7 packet-gate runner import')
    (package_modules / 'rs05_step2_packet_gate.py').write_text(
        gate_source.replace('from .rs05_fullbody_step2 import',
                            'from .rs05_fullbody_step2_id7 import'))
    for name in ('test_rs05_fullbody_step2.py', 'test_rs05_step2_packet_gate.py'):
        shutil.copy2(TESTS / name, output / 'tests' / name)
    id7_test_source = (TESTS / 'test_rs05_id7_step1.py').read_text()
    require(id7_test_source.count('from singularitydog_hw import rs05_fullbody_step2 as core') == 1,
            'Unexpected ID7 test runner import')
    (output / 'tests' / 'test_rs05_id7_step1.py').write_text(
        id7_test_source.replace('from singularitydog_hw import rs05_fullbody_step2 as core',
                                'from singularitydog_hw import rs05_fullbody_step2_id7 as core'))
    write_json(output / 'id7-active-review.json', active_review)
    wrapper = output / 'prepared_current_hold.py'
    body = adapt_wrapper(wrapper.read_text(), output.name, active_review['boot_id'])
    wrapper.write_text(body)
    require(not any(path.is_symlink() for path in output.rglob('*')),
            'Active ID7 package contains a symlink')
    pins = {str(path.relative_to(output)): sha(path)
            for path in sorted(output.rglob('*'))
            if path.is_file() and path not in (output / 'active-manifest.json', wrapper)}
    body, count = re.subn(r'^PINS = .*$', 'PINS = ' + repr(pins), body,
                          count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular active PINS')
    ast.parse(body)
    wrapper.write_text(body)
    manifest = {**pins, wrapper.name: sha(wrapper)}
    write_json(output / 'active-manifest.json', manifest)
    require(read_json(output / 'active-manifest.json') == manifest
            and all(sha(output / name) == digest for name, digest in manifest.items())
            and ast.literal_eval(re.search(r'^PINS = (.*)$', body, re.MULTILINE).group(1)) == pins,
            'Final active ID7 source manifest mismatch')
    return {'package': str(output), 'boot_id': active_review['boot_id'],
            'raw_diagnostic_only': True, 'moving_motor_ids': [7],
            'standing_allowed': False, 'learned_policy_allowed': False,
            'pinned_files': len(manifest), 'manifest_sha256': sha(output / 'active-manifest.json'),
            'wrapper_sha256': sha(wrapper), 'review_sha256': sha(output / 'id7-active-review.json'),
            'disabled_preflight_summary_sha256': sha(summary_path),
            'disabled_preflight_events_sha256': sha(events_path)}


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
