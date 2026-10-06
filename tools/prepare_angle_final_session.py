#!/usr/bin/env python3
"""Prepare/review private angle observations from files; never execute a capture.

Bubble-level L reproducibility and relative direction are candidates only.
Existing offsets remain unchanged; no absolute error, scale, or output approval
is inferred. Capture argv is for the existing dual-port Type0/17 reader.
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import asdict
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from artifact_manifest import member, parse_json
from audit_angle_calibration import load_profile, read, write_private
from dog_tomorrow import verify_kit
from manual_angle_prompt import PHYSICAL_NOTE_CANCEL, PHYSICAL_NOTE_CONFIRMATION_TOKENS
from singularitydog_hw.angle_calibration_audit import IDS, UNKNOWN_EPOCHS, AxisCalibration, audit_twelve_axes

PLAN_SCHEMA = 'singularitydog.angle-final-session-plan.v1'
OBSERVATION_SCHEMA = 'singularitydog.angle-final-observation.v1'
CAPTURE_SCHEMA = 'singularitydog.readonly-12-angle-capture.v1'
CAPTURE_MODULE = 'singularitydog_hw.motor_epoch_readonly_capture'
CAPTURE_SOURCE = 'runtime/singularitydog_hw/motor_epoch_readonly_capture.py'
TRACE_MAX_EVENTS, TRACE_MAX_BYTES = 4096, 2 * 1024 * 1024
LEGS = {'FR': ('右前脚', (1, 2, 3)), 'FL': ('左前脚', (4, 5, 6)),
        'RR': ('右後脚', (7, 8, 9)), 'RL': ('左後脚', (10, 11, 12))}
# Existing manual candidate screens; these are not physical angle error bounds.
MOVE_MIN_DEG, MOVE_MAX_DEG, OTHER_AXIS_LIMIT_DEG, RETURN_LIMIT_DEG = 3., 20., 3., 3.
L_INSTRUCTIONS = ['付け根の横リンクを左右へ水平にする。', '上脚を鉛直下へ向ける。',
                  '膝から足先へ向かう下脚を顔側へ水平にする。',
                  '気泡式水準器の精度は未知。名目角は診断の目安で、測定済み原点ではない。']
FLAGS = {'hardware_accessed': False, 'automatic_execution': False, 'motor_targets_generated': False,
         'profile_changed': False, 'zero_verified': False, 'direction_verified_for_runtime': False,
         'dynamic_scale_verified': False, 'calibration_approved_for_runtime': False,
         'approved_for_runtime': False, 'output_allowed': False, 'physical_uncertainty_rad': None}


def need(condition, message):
    if not condition: raise ValueError(message)


def finite(value, name):
    need(type(value) in (int, float), 'Finite numeric '+name+' required')
    try: number = float(value)
    except OverflowError as error: raise ValueError('Finite numeric '+name+' required') from error
    need(math.isfinite(number), 'Finite numeric '+name+' required')
    return number


def text(value, name):
    need(type(value) is str and value.strip() == value and bool(value), 'Explicit '+name+' required')
    return value


def sha(value):
    need(type(value) is str and re.fullmatch('[0-9a-f]{64}', value) is not None, 'Explicit SHA256 required')
    return value


def stamp(value):
    result = datetime.datetime.fromisoformat(text(value, 'timestamp'))
    need(result.tzinfo is not None, 'Timestamp timezone required')
    return result


def selected_ids(values=None):
    values = list(IDS) if values is None else list(values)
    need(values and all(type(mid) is int and mid in IDS for mid in values)
         and len(set(values)) == len(values), 'Unique selected IDs1..12 required')
    return sorted(values)


def axis_direction(mid):
    need(type(mid) is int and mid in IDS, 'Motor ID1..12 required')
    leg = next(name for name, (_, ids) in LEGS.items() if mid in ids)
    kind = ('calf', 'thigh', 'hip')[(mid-1) % 3]
    direction = {'calf': 'down', 'thigh': 'face', 'hip': 'outward'}[kind]
    sign = 1 if kind == 'calf' or kind == 'hip' and leg in ('FL', 'RL') else -1
    instructions = {'calf': '上脚と付け根を保ち、膝から先だけを回して足先を少し下げる。',
                    'thigh': '膝の相対角と付け根を保ち、上脚を顔側へ少し傾ける。下脚は一緒に回る。',
                    'hip': '上脚と膝の相対角を保ち、付け根で脚全体を胴体の外側へ少し開く。'}
    return {'leg': leg, 'leg_label': LEGS[leg][0], 'motor_id': mid, 'joint': kind,
            'physical_direction': direction, 'model_delta_sign': sign, 'instructions': [instructions[kind]],
            'measured_model_angle_rad': None, 'physical_uncertainty_rad': None}


def absolute_path(value):
    path = Path(value)
    need(path.is_absolute() and '..' not in path.parts and str(path) == str(value), 'Canonical absolute path required')
    return path


def capture_values(capture, axes, expected_uid_sha, ports):
    need(type(capture) is dict and capture.get('schema') == CAPTURE_SCHEMA
         and capture.get('status') == 'RECORDED_REVIEW_REQUIRED' and capture.get('errors') == []
         and capture.get('motor_output_allowed') is False and capture.get('approved_for_runtime') is False
         and capture.get('angle_wrap_applied') is False, 'Complete unwrapped no-output readonly capture required')
    plan = capture.get('plan', {})
    need(plan.get('allowed_can_types') == [0, 17] and all(type(v) is int for v in plan['allowed_can_types'])
         and json.dumps(plan.get('ids_by_bus'), sort_keys=True) == json.dumps(
             {'front': list(range(1, 7)), 'rear': list(range(7, 13))}, sort_keys=True)
         and plan.get('automatic_retry') is False
         and plan.get('motor_output_available') is False and plan.get('ports') == ports,
         'Canonical two-port Type0/17 capture context required')
    need(capture.get('expected_uids_sha256') == expected_uid_sha, 'Capture expected UID source differs')
    text(capture.get('boot_id'), 'capture boot ID')
    start, end = stamp(capture.get('started_at')), stamp(capture.get('completed_at'))
    need(start <= end, 'Capture time order invalid')
    identities, rows = capture.get('identities'), capture.get('telemetry', {}).get('rows')
    keys = {str(mid) for mid in IDS}
    need(type(identities) is dict and type(rows) is dict and set(identities) == set(rows) == keys,
         'All12 capture identities and rows required')
    raw = {}
    for mid in IDS:
        key, row = str(mid), rows[str(mid)]
        need(type(row) is dict and type(identities[key]) is dict
             and identities[key].get('mcu_uid_hex') == axes[key]['uid'], 'Capture UID changed: ID'+key)
        need(type(row.get('run_mode')) is int and row['run_mode'] == 0
             and finite(row.get('current'), 'current') == 0, 'Capture is not observed quiet: ID'+key)
        need(0 <= finite(row.get('position_span_deg'), 'position span') <= .1,
             'Capture is not static: ID'+key)
        raw[key] = finite(row.get('median_position_rad'), 'raw position')
    return raw


def source(path, expected):
    path = Path(path).expanduser()
    need(not path.is_symlink(), 'Regular source file required')
    path = path.resolve(strict=True)
    need(path.is_file() and not path.is_symlink(), 'Regular source file required')
    data, digest = read(path, sha(expected))
    return data, {'path': str(path), 'sha256': digest}


def trace_capture_source(kit, manifest):
    """Check optional CLI support in the pinned collector without importing it."""
    need(CAPTURE_SOURCE in manifest['files'], 'Frozen collector with --trace-events required')
    path = member(kit, CAPTURE_SOURCE)
    need(not path.is_symlink(), 'Regular frozen collector source required')
    raw = path.read_bytes()
    need(len(raw) <= TRACE_MAX_BYTES and hashlib.sha256(raw).hexdigest() == manifest['files'][CAPTURE_SOURCE],
         'Frozen collector source changed')
    try: tree = ast.parse(raw)
    except (SyntaxError, ValueError) as error: raise ValueError('Invalid frozen collector source') from error
    mains = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main']
    need(len(mains) == 1 and any(isinstance(statement, ast.Expr)
             and isinstance(statement.value, ast.Call) and isinstance(statement.value.func, ast.Attribute)
             and statement.value.func.attr == 'add_argument' and any(isinstance(arg, ast.Constant)
             and arg.value == '--trace-events' for arg in statement.value.args)
             for statement in mains[0].body),
         'Frozen collector does not support --trace-events; prepare a new kit')
    return {'path': str(path), 'sha256': manifest['files'][CAPTURE_SOURCE]}


def read_trace_bytes(path):
    path = absolute_path(path)
    need(not any(parent.is_symlink() for parent in (path, *path.parents)) and path.is_file(),
         'Regular trace path without symlinks required')
    with path.open('rb') as stream: raw = stream.read(TRACE_MAX_BYTES + 1)
    need(0 < len(raw) <= TRACE_MAX_BYTES and raw.endswith(b'\n'), 'Complete bounded JSONL trace required')
    return raw


def review_event_trace(plan, name, capture):
    """Bind completed optional raw-event bytes to this exact capture stage.

    This is file integrity, not a proof of CAN delivery or physical motion.
    Failed/partial traces remain readable, but cannot produce a review.
    """
    if 'readonly_event_trace' not in plan: return None
    stage = next((value for value in plan['stages'] if value['name'] == name), None)
    need(stage is not None, 'Planned trace capture stage required')
    expected_path = stage['capture_trace_output']
    trace = capture.get('trace_events')
    need(type(trace) is dict and trace.get('path') == expected_path
         and trace.get('complete') is True and trace.get('status') == 'COMPLETE_EVENT_TRACE'
         and trace.get('errors') == [], 'Complete bound raw-event trace required: '+name)
    declaration = capture.get('plan', {}).get('trace_events')
    need(declaration == {'path': expected_path, 'max_events': TRACE_MAX_EVENTS, 'max_bytes': TRACE_MAX_BYTES},
         'Capture trace declaration differs from plan')
    raw = read_trace_bytes(expected_path)
    lines = raw.splitlines()
    need(0 < len(lines) <= TRACE_MAX_EVENTS and type(trace.get('event_count')) is int
         and trace['event_count'] == len(lines) and type(trace.get('byte_count')) is int
         and trace['byte_count'] == len(raw) and type(trace.get('attempted_event_count')) is int
         and trace['attempted_event_count'] == len(lines), 'Trace byte/event counts differ')
    need(hashlib.sha256(raw).hexdigest() == sha(trace.get('sha256')), 'Raw-event trace SHA256 changed')
    for line in lines:
        try: event = parse_json(line)
        except (ValueError, UnicodeDecodeError) as error: raise ValueError('Invalid trace JSONL') from error
        need(type(event) is dict and event.get('bus') in ('front', 'rear'), 'Bound CAN bus event required')
    return {'path': expected_path, 'sha256': trace['sha256'],
            'event_count': len(lines), 'byte_count': len(raw)}


def make_plan(*, kit_root, kit_manifest_sha256, angle_profile, angle_profile_sha256,
              capture, capture_sha256, session_dir, target_kit_root, target_python, ids=None, l_legs=None,
              trace_events=False):
    """Pinned file preparation only; caller owns any later human/capture workflow."""
    kit = Path(kit_root).expanduser().resolve(strict=True)
    verify_kit(kit)
    manifest, manifest_pin = source(member(kit, 'kit-manifest.json'), kit_manifest_sha256)
    need(type(trace_events) is bool, 'Boolean trace-events selection required')
    trace_pin = trace_capture_source(kit, manifest) if trace_events else None
    config_path = member(kit, 'kit-config.json')
    config, config_pin = source(config_path, manifest['files']['kit-config.json'])
    need(config.get('calibration_approved_for_runtime') is False, 'Unapproved calibration kit required')
    ports = {bus: config[bus+'_port'] for bus in ('front', 'rear')}
    need(all(type(value) is str and re.fullmatch('/dev/serial/by-path/[A-Za-z0-9_.:-]+', value)
             for value in ports.values()) and ports['front'] != ports['rear'], 'Distinct canonical by-path ports required')
    uid_path = member(kit, config['expected_uids'])
    uids, uid_pin = source(uid_path, manifest['files'][config['expected_uids']])
    need(set(uids) == {str(mid) for mid in IDS} and len(set(uids.values())) == 12,
         'Twelve unique expected UIDs required')
    profile, profile_pin = source(angle_profile, angle_profile_sha256)
    need(profile_pin['sha256'] == manifest['files'][config['angle_profile']],
         'Explicit angle profile differs from frozen kit selection')
    loaded, contracts = load_profile(profile_pin['path'])
    need(loaded == profile and all(c.calibration_sha256 == profile_pin['sha256'] for c in contracts.values()),
         'Angle profile changed during validation')
    axes = {str(mid): {key: value for key, value in asdict(contracts[mid]).items() if key != 'calibration_sha256'}
            for mid in IDS}
    need(all(axes[str(mid)]['uid'] == uids[str(mid)] for mid in IDS), 'Profile/frozen kit UID mismatch')
    seed, seed_pin = source(capture, capture_sha256)
    seed_raw = capture_values(seed, axes, uid_pin['sha256'], ports)
    ids = selected_ids(ids)
    l_legs = [leg for leg, (_, leg_ids) in LEGS.items() if any(mid in ids for mid in leg_ids)] if l_legs is None else list(l_legs)
    need(len(set(l_legs)) == len(l_legs) and all(leg in LEGS for leg in l_legs), 'Unique L review legs required')
    l_legs = [leg for leg in LEGS if leg in l_legs]
    l_ids = {leg: [mid for mid in LEGS[leg][1] if mid in ids] or list(LEGS[leg][1]) for leg in l_legs}
    destination, target, python = absolute_path(session_dir), absolute_path(target_kit_root), absolute_path(target_python)
    need(target.name == kit.name, 'Target frozen kit name differs')
    need(not any((parent/'.git').exists() for parent in (destination, *destination.parents)),
         'Session capture paths must be private outside Git')
    environment = {'PYTHONPATH': str(target/'runtime')}
    stages = []
    def stage(name, kind, leg, instructions, extra):
        output = destination/(name+'.json')
        argv = [str(python), '-B', '-m', CAPTURE_MODULE, '--front-port', ports['front'],
                '--rear-port', ports['rear'], '--expected-uids', str(target/config['expected_uids']),
                '--execute-readonly', '--output', str(output)]
        trace_fields = {}
        if trace_events:
            trace_output = destination/(name+'.events.jsonl')
            argv += ['--trace-events', str(trace_output)]
            trace_fields['capture_trace_output'] = str(trace_output)
        stages.append({'name': name, 'kind': kind, 'leg': leg, 'leg_label': LEGS[leg][0],
                       'instructions': instructions, 'capture_argv': argv, 'capture_env': dict(environment),
                       'capture_output': str(output), 'all12_UIDs_required': True,
                       'capture_requires_explicit_caller_execution': True, **trace_fields, **extra})
    for leg, chosen in l_ids.items():
        stage(leg.lower()+'-l', 'l_reproducibility', leg, list(L_INSTRUCTIONS),
              {'selected_ids': chosen, 'nominal_model_deg_by_id': {
                  str(mid): -90 if (mid-1) % 3 == 0 else 0 for mid in chosen}, 'nominal_angles_measured': False})
    for mid in ids:
        direction = axis_direction(mid)
        for phase in ('baseline', 'moved', 'return'):
            instructions = (list(L_INSTRUCTIONS) if phase == 'baseline' else direction['instructions']
                            if phase == 'moved' else ['元のL字へ戻し、他の関節も最初の姿勢を保つ。'])
            stage('id'+str(mid)+'-'+phase, 'direction', direction['leg'], instructions,
                  {'phase': phase, **direction})
    result = {'schema': PLAN_SCHEMA, 'status': 'FILE_ONLY_WORK_PLAN', **FLAGS,
              'kit_root': str(kit), 'target_kit_root': str(target), 'target_python': str(python),
              'session_dir': str(destination), 'selected_ids': ids, 'axes': axes, 'stages': stages,
              'l_legs': l_legs, 'l_ids_by_leg': l_ids,
              'boot_id': seed['boot_id'], 'seed_motor_power_epoch_recorded': seed.get('motor_power_epoch'),
              'seed_completed_at': seed['completed_at'],
              'seed_raw_rad_by_id': seed_raw,
              'ports': ports, 'source_files': {'kit_manifest': manifest_pin, 'kit_config': config_pin,
                  'expected_uids': uid_pin, 'angle_profile': profile_pin, 'seed_capture': seed_pin},
              'direction_screens_deg': {'selected_raw_min': MOVE_MIN_DEG, 'selected_raw_max': MOVE_MAX_DEG,
                  'other_axis_abs_max': OTHER_AXIS_LIMIT_DEG, 'return_abs_max': RETURN_LIMIT_DEG,
                  'discontinuity_abs_max': 180.},
              'reference_method': 'bubble_spirit_level_unknown_precision',
              'absolute_zero_error_rad': None, 'old_zero_and_sign_retained': True,
              'power_epoch_inferred_from_boot': False,
              'observation_template': {'schema': OBSERVATION_SCHEMA, 'operator_id': None, 'observed_at': None,
                  'power_event': {'label': None, 'unchanged_since_seed': None, 'inferred_from_boot': None,
                                  'boot_id': seed['boot_id'], 'capture_sha256': {}},
                  'l_pose_observed': None, 'relative_output_shaft_observed': None,
                  'only_selected_relative_joint_moved': None, 'physical_direction_observed': None,
                  'model_delta_sign': None, 'returned_to_baseline': None, 'physical_observation_note': None}}
    if trace_events:
        result['readonly_event_trace'] = {'required': True, 'collector_source': trace_pin,
            'format': 'jsonl', 'max_events': TRACE_MAX_EVENTS, 'max_bytes': TRACE_MAX_BYTES,
            'file_integrity_only': True}
    verify_kit(kit)
    for pin in result['source_files'].values(): source(pin['path'], pin['sha256'])
    if trace_events: need(trace_capture_source(kit, manifest) == trace_pin, 'Frozen collector changed during preparation')
    return result


def validate_plan(plan):
    """Recompute the file plan, including retained typed axis values and argv."""
    need(type(plan) is dict and plan.get('schema') == PLAN_SCHEMA, 'Final session plan required')
    files = plan['source_files']
    expected = make_plan(kit_root=plan['kit_root'], kit_manifest_sha256=files['kit_manifest']['sha256'],
        angle_profile=files['angle_profile']['path'], angle_profile_sha256=files['angle_profile']['sha256'],
        capture=files['seed_capture']['path'], capture_sha256=files['seed_capture']['sha256'],
        session_dir=plan['session_dir'], target_kit_root=plan['target_kit_root'], target_python=plan['target_python'],
        ids=plan['selected_ids'], l_legs=plan['l_legs'], trace_events='readonly_event_trace' in plan)
    need(json.dumps(plan, sort_keys=True, allow_nan=False) == json.dumps(expected, sort_keys=True, allow_nan=False),
         'Session plan differs from pinned inputs')
    return expected


def attest_power(plan, observation, capture_pins):
    need(type(observation) is dict and observation.get('schema') == OBSERVATION_SCHEMA, 'Observation schema required')
    text(observation.get('operator_id'), 'operator ID')
    stamp(observation.get('observed_at'))
    note = text(observation.get('physical_observation_note'), 'physical relative observation note')
    need(note.lower() not in PHYSICAL_NOTE_CONFIRMATION_TOKENS | PHYSICAL_NOTE_CANCEL,
         'Physical relative observation note cannot be a standalone confirmation or cancellation token')
    event = observation.get('power_event')
    need(type(event) is dict, 'Explicit operator power event required')
    label = text(event.get('label'), 'operator power event label')
    need(label.upper() not in {x.upper() for x in UNKNOWN_EPOCHS} | {'NONE', 'NULL'}
         and not label.upper().startswith(('UNKNOWN', 'UNSPECIFIED', 'NOT_INFERRED'))
         and label != plan['boot_id'], 'Unknown motor power epoch blocks review')
    need(event.get('unchanged_since_seed') is True and event.get('inferred_from_boot') is False
         and event.get('boot_id') == plan['boot_id'], 'Operator-attested unchanged motor power required')
    need(event.get('capture_sha256') == {'seed': plan['source_files']['seed_capture']['sha256'], **capture_pins},
         'Power event must bind the exact seed and observation captures')
    return label


def review_context(plan, captures, observation):
    raw, digests, previous = {}, {}, stamp(plan['seed_completed_at'])
    previous_raw = plan['seed_raw_rad_by_id']
    for phase, (capture, digest) in captures.items():
        sha(digest); need(digest not in digests.values(), 'Distinct observation captures required')
        need(capture.get('boot_id') == plan['boot_id'], 'Capture boot changed')
        raw[phase] = capture_values(capture, plan['axes'], plan['source_files']['expected_uids']['sha256'], plan['ports'])
        need(all(abs(math.degrees(raw[phase][str(mid)]-previous_raw[str(mid)])) <= 180. for mid in IDS),
             'Unwrapped discontinuity over180deg between captures')
        start, end = stamp(capture['started_at']), stamp(capture['completed_at'])
        if previous is not None: need(previous <= start, 'Observation capture order invalid')
        previous = end; previous_raw = raw[phase]; digests[phase] = digest
    label = attest_power(plan, observation, digests)
    need(stamp(observation['observed_at']) >= previous,
         'Bound observation must be recorded after the final capture completes')
    recorded_epochs = [plan['seed_motor_power_epoch_recorded']] + [c.get('motor_power_epoch') for c, _ in captures.values()]
    for recorded in recorded_epochs:
        need(recorded in UNKNOWN_EPOCHS or recorded == label, 'Recorded motor power epoch conflicts with attestation')
    return raw, label


def review_direction(plan, mid, baseline, moved, returned, observation):
    """Pure direction-only review; inputs are (capture object, pinned digest)."""
    need(type(mid) is int and mid in plan['selected_ids'], 'Selected motor ID required')
    direction = axis_direction(mid)
    raw, label = review_context(plan, {'baseline': baseline, 'moved': moved, 'return': returned}, observation)
    trace_pins = {phase: review_event_trace(plan, 'id'+str(mid)+'-'+phase, item[0])
                  for phase, item in {'baseline': baseline, 'moved': moved, 'return': returned}.items()}
    for key in ('l_pose_observed', 'relative_output_shaft_observed', 'only_selected_relative_joint_moved', 'returned_to_baseline'):
        need(observation.get(key) is True, 'Complete explicit physical observation required: '+key)
    need(observation.get('physical_direction_observed') == direction['physical_direction'], 'Physical direction differs from plan')
    sign = observation.get('model_delta_sign')
    need(type(sign) is int and sign == direction['model_delta_sign'], 'Explicit model delta sign differs from physical direction')
    deltas, returns = {}, {}
    for axis in IDS:
        key = str(axis)
        delta = math.degrees(raw['moved'][key]-raw['baseline'][key])
        back = math.degrees(raw['return'][key]-raw['baseline'][key])
        need(max(abs(delta), abs(back), abs(math.degrees(raw['return'][key]-raw['moved'][key]))) <= 180.,
             'Unwrapped discontinuity over180deg: ID'+key)
        need(abs(back) <= RETURN_LIMIT_DEG, 'Failure to return: ID'+key)
        if axis != mid: need(abs(delta) <= OTHER_AXIS_LIMIT_DEG, 'Another relative joint moved: ID'+key)
        deltas[key], returns[key] = delta, back
    need(MOVE_MIN_DEG <= abs(deltas[str(mid)]) <= MOVE_MAX_DEG, 'Selected raw move outside existing3..20deg candidate screen')
    candidate = 1 if deltas[str(mid)]*sign > 0 else -1
    axis = plan['axes'][str(mid)]
    return {'schema': 'singularitydog.angle-direction-review.v1', **FLAGS,
        **({'event_trace_sources': trace_pins} if 'readonly_event_trace' in plan else {}),
        'status': 'DIRECTION_ONLY_CANDIDATE' if candidate == axis['sign'] else 'SIGN_CONFLICT_REVIEW_REQUIRED',
        'motor_id': mid, 'direction': direction, 'sign_candidate': candidate, 'profile_sign_matches': candidate == axis['sign'],
        'retained_axis': dict(axis), 'raw_delta_deg_by_id': deltas, 'return_delta_deg_by_id': returns,
        'operator_attested_motor_power_epoch': label, 'power_epoch_inferred_from_boot': False,
        'capture_sha256': {phase: value[1] for phase, value in {'baseline':baseline,'moved':moved,'return':returned}.items()},
        'absolute_zero_error_rad': None, 'measured_model_angle_rad': None, 'reference_method': 'bubble_spirit_level_unknown_precision'}


def review_l(plan, leg, captured, observation):
    """Report nominal L differences through existing unique numeric branches only."""
    need(leg in plan['l_legs'], 'Planned L review leg required')
    raw, label = review_context(plan, {'l': captured}, observation)
    trace_pin = review_event_trace(plan, leg.lower()+'-l', captured[0])
    need(observation.get('l_pose_observed') is True, 'Explicit operator L pose observation required')
    contracts = {mid: AxisCalibration(**plan['axes'][str(mid)],
                 calibration_sha256=plan['source_files']['angle_profile']['sha256']) for mid in IDS}
    audit = audit_twelve_axes(contracts, raw['l'], {str(mid): plan['axes'][str(mid)]['uid'] for mid in IDS})
    rows = {}
    for mid in plan['l_ids_by_leg'][leg]:
        options = audit['rows_by_id'][str(mid)]['periodic_branch_candidates']
        need(len(options) == 1 and options[0]['whole_uncertainty_inside_limits'], 'Unique existing numeric branch required: ID'+str(mid))
        nominal = -90. if (mid-1) % 3 == 0 else 0.
        rows[str(mid)] = {'retained_axis': dict(plan['axes'][str(mid)]), 'numeric_branch_turns': options[0]['turns'],
            'model_rad_using_old_zero': options[0]['model_rad'], 'nominal_model_deg_diagnostic_only': nominal,
            'difference_from_nominal_deg': math.degrees(options[0]['model_rad'])-nominal,
            'absolute_zero_error_rad': None, 'zero_verified': False, 'nominal_angle_measured': False}
    return {'schema':'singularitydog.angle-l-reproducibility-review.v1', **FLAGS, 'status':'L_REPRODUCIBILITY_CANDIDATE',
            **({'event_trace_sources': {'l': trace_pin}} if trace_pin is not None else {}),
            'leg':leg, 'rows_by_id':rows, 'operator_attested_motor_power_epoch':label,
            'capture_sha256':captured[1], 'absolute_zero_error_rad':None,
            'reference_method':'bubble_spirit_level_unknown_precision'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    prepare = modes.add_parser('plan')
    for name in ('kit-root','angle-profile','capture','session-dir','target-kit-root','target-python'):
        prepare.add_argument('--'+name, required=True)
    for name in ('kit-manifest','angle-profile','capture'):
        prepare.add_argument('--'+name+'-sha256', required=True)
    prepare.add_argument('--id', type=int, choices=IDS, action='append')
    prepare.add_argument('--l-leg', choices=LEGS, action='append')
    prepare.add_argument('--trace-events', action='store_true', help='Plan bounded raw-event recording with a new trace-capable frozen kit')
    for name in ('review-direction','review-l'):
        review = modes.add_parser(name)
        for item in ('plan','observation'):
            review.add_argument('--'+item, required=True); review.add_argument('--'+item+'-sha256', required=True)
        fields = ('baseline','moved','return-capture') if name == 'review-direction' else ('capture',)
        for item in fields:
            review.add_argument('--'+item, required=True); review.add_argument('--'+item+'-sha256', required=True)
        review.add_argument('--id', type=int, choices=IDS, required=True) if name == 'review-direction' else review.add_argument('--leg', choices=LEGS, required=True)
    for mode in (prepare, *[modes.choices[name] for name in ('review-direction','review-l')]): mode.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    try:
        if args.mode == 'plan':
            result = make_plan(kit_root=args.kit_root, kit_manifest_sha256=args.kit_manifest_sha256,
                angle_profile=args.angle_profile, angle_profile_sha256=args.angle_profile_sha256,
                capture=args.capture, capture_sha256=args.capture_sha256, session_dir=args.session_dir,
                target_kit_root=args.target_kit_root, target_python=args.target_python, ids=args.id, l_legs=args.l_leg,
                trace_events=args.trace_events)
        else:
            plan, plan_pin = source(args.plan, args.plan_sha256); validate_plan(plan)
            observation, observation_pin = source(args.observation, args.observation_sha256)
            files = {'plan':plan_pin, 'observation':observation_pin}
            def captured(name):
                value, pin = source(getattr(args, name), getattr(args, name+'_sha256'))
                files[name] = pin; return value, pin['sha256']
            if args.mode == 'review-direction':
                result = review_direction(plan, args.id, captured('baseline'), captured('moved'), captured('return_capture'), observation)
            else: result = review_l(plan, args.leg, captured('capture'), observation)
            validate_plan(plan)
            for pin in files.values(): source(pin['path'], pin['sha256'])
            for pin in result.get('event_trace_sources', {}).values():
                raw = read_trace_bytes(pin['path'])
                need(hashlib.sha256(raw).hexdigest() == pin['sha256'], 'Raw-event trace changed after review')
            result['source_files'] = files
        write_private(args.output, result)
    except (OSError, ValueError, KeyError, TypeError) as error: parser.error(str(error))
    print(json.dumps({'status':result['status'], 'approved_for_runtime':False, 'output_allowed':False}))
    return 0


if __name__ == '__main__': raise SystemExit(main())
