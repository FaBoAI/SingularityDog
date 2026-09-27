"""Freeze a same-boot role-group raw step diagnostic from a completed hold.

This is a file-only builder. The disabled package must run and pass on the
robot before the active package can be built. Neither package is a standing or
learned-policy controller.
"""
from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

import build_fullbody_step2_disabled as disabled_base
import build_fullbody_step2_active as active_base


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
TESTS = ROOT / 'runtime' / 'tests'
DEFAULT_ANNOUNCEMENT_WAV = ROOT / 'runtime' / 'assets' / 'test-start-ja.wav'
GROUPS = {'toe': (1, 4, 7, 10), 'thigh': (2, 5, 8, 11),
          'front-thigh': (2, 5), 'front-hip': (3, 6), 'hip': (3, 6, 9, 12)}
DIRECTION_PROFILES = ('raw-plus', 'mirrored-thigh', 'front-thigh-toward-face',
                      'front-hip-mirrored')
THIGH_GROUPS = frozenset(('thigh', 'front-thigh'))
FLAGS = disabled_base.REQUIRED_FLAGS
CLEARANCE_START_TOLERANCE_DEG = 3.
CONTINUOUS_FRONT_HIP_PROFILE = 'front-hip-hold1s-step5-step10-v1'
CONTINUOUS_FRONT_HIP_WAYPOINTS = [5.0, 10.0]
CONTINUOUS_FRONT_HIP_TICKS = 380
require, sha, read_json, write_json = (disabled_base.require, disabled_base.sha,
                                      disabled_base.read_json, disabled_base.write_json)


def pin_package(output: Path, wrapper_name: str, manifest_name: str, text: str) -> dict:
    # Python bytecode is mutable across hosts/interpreters and must never be
    # part of the frozen source manifest.
    for cache in sorted(output.rglob('__pycache__'), reverse=True):
        shutil.rmtree(cache)
    for bytecode in output.rglob('*.pyc'):
        bytecode.unlink()
    wrapper = output / wrapper_name
    pinned = {str(path.relative_to(output)): sha(path) for path in sorted(output.rglob('*'))
              if path.is_file() and path not in (output / manifest_name, wrapper)}
    require(all(not path.is_symlink() for path in output.rglob('*')),
            'Frozen role-group package may not contain symlinks')
    text, count = re.subn(r'^PINS = .*$', 'PINS = ' + repr(pinned), text,
                          count=1, flags=re.MULTILINE)
    require(count == 1, 'No singular PINS assignment')
    wrapper.write_text(text)
    ast.parse(text)
    manifest = {**pinned, wrapper_name: sha(wrapper)}
    write_json(output / manifest_name, manifest)
    require(read_json(output / manifest_name) == manifest
            and all(sha(output / name) == digest for name, digest in manifest.items()),
            'Final source manifest mismatch')
    return manifest


def validate_frozen_front_hip_transport(output: Path) -> None:
    """Exercise both Kp12 front-hip wires through the frozen transport, without I/O."""
    manifest = read_json(output / 'active-manifest.json')
    for name in ('singularitydog_hw/rs05_bus_transport.py',
                 'singularitydog_hw/rs05_fullbody_step2.py',
                 'singularitydog_hw/rs05_trial_protocol.py'):
        require(manifest.get(name) == sha(output / name),
                f'Frozen transport source is not pinned: {name}')
    program = """
import json, math, pathlib, sys
sys.path.insert(0, sys.argv[1])
from singularitydog_hw.rs05_bus_transport import BusTrialTransport, MOTION_PHASES
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from singularitydog_hw.rs05_fullbody_step2 import (
    _interleaved_feedback_enabled, ROLE_GROUP_REVIEW_SCHEMA,
    FRONT_HIP_REVIEW_SCOPE, ROLE_FRONT_HIP_GAIN_PROFILE)
review_path = pathlib.Path(sys.argv[1]) / 'step2-active-review.json'
amplitude = json.loads(review_path.read_text())['amplitude_deg'] if review_path.exists() else 5.
review = {'schema': ROLE_GROUP_REVIEW_SCHEMA, 'scope': FRONT_HIP_REVIEW_SCOPE,
          'role_group': 'front-hip', 'amplitude_deg': amplitude,
          'gain_profile': ROLE_FRONT_HIP_GAIN_PROFILE}
interleave = _interleaved_feedback_enabled(review, False)
assert interleave is (amplitude == 10.)
assert _interleaved_feedback_enabled(review, True) is False
phase = (TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10 if amplitude == 10.
         else TrialPhase.POSITION_ROLE_FRONT_HIP_KP12)
assert phase in MOTION_PHASES
class NoHardware:
    def read(self, count): raise AssertionError('No serial read is permitted')
    def write(self, data): raise AssertionError('No serial write is permitted')
for mid, direction in ((3, 1), (6, -1)):
    transport = BusTrialTransport(NoHardware(), lambda event: None,
                                  ids=(1, 2, 3, 4, 5, 6), interleave_feedback=interleave)
    assert transport.interleave_feedback is interleave
    command = motion_request(phase=phase,
                             center_rad=0., offset_rad=direction*math.radians(amplitude-1e-6),
                             motor_id=mid)
    frame = transport._validated_wire(command)
    assert frame.kind == 1 and frame.destination == mid
"""
    result = subprocess.run([sys.executable, '-B', '-c', program, str(output)],
                            capture_output=True, text=True, timeout=5)
    require(result.returncode == 0,
            'Frozen front-hip Kp12 command rejected before UART: ' + result.stderr[-500:])


def validate_frozen_feedback_age(output: Path, manifest_name: str) -> None:
    """Check the pinned runner and age guard together without opening a device."""
    manifest = read_json(output / manifest_name)
    for name in ('singularitydog_hw/rs05_fullbody_step2.py',
                 'singularitydog_hw/rs05_joint_trial.py'):
        require(manifest.get(name) == sha(output / name),
                f'Frozen feedback guard source is not pinned: {name}')
    program = """
import sys
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
from singularitydog_hw.rs05_joint_trial import check_feedback
from singularitydog_hw.rs05_fullbody_step2 import check_feedback as runner_check_feedback
assert runner_check_feedback is check_feedback
feedback = SimpleNamespace(protocol_position_rad=0., velocity_rad_s=0.,
                           temperature_c=30., mode_state=2, fault_bits=0)
def stale(age, **kwargs):
    try:
        check_feedback(feedback, 0., 0., age, **kwargs)
    except RuntimeError as exc:
        assert str(exc) == 'Stale feedback'
    else:
        raise AssertionError('Stale feedback was accepted')
stale(.12)
check_feedback(feedback, 0., 0., .12, max_age_s=.125)
stale(.126, max_age_s=.125)
"""
    result = subprocess.run([sys.executable, '-B', '-c', program, str(output)],
                            capture_output=True, text=True, timeout=5)
    require(result.returncode == 0,
            'Frozen feedback age guard rejected: ' + result.stderr[-500:])


def directions_for(group: str, profile: str) -> dict[str, int]:
    require(group in GROUPS and profile in DIRECTION_PROFILES,
            'Use a known role group and direction profile')
    require(profile != 'mirrored-thigh' or group == 'thigh',
            'Mirrored direction is defined only for the four upper-leg joints')
    require((group == 'front-thigh') == (profile == 'front-thigh-toward-face'),
            'Front-thigh direction is defined only by the toward-face profile')
    require((group == 'front-hip') == (profile == 'front-hip-mirrored'),
            'Front-hip direction is defined only by the mirrored profile')
    moving = set(GROUPS[group])
    return {str(mid): (-1 if profile == 'front-thigh-toward-face' and mid == 2
                       else -1 if profile == 'front-hip-mirrored' and mid == 6
                       else -1 if profile == 'mirrored-thigh' and mid in (5, 11)
                       else 1 if mid in moving else 0) for mid in range(1, 13)}


def amplitude_for(group: str) -> float:
    return 10. if group in THIGH_GROUPS else 5. if group == 'front-hip' else 1.


def validate_candidate_directions(candidate: dict, group: str, centers: dict) -> tuple[str, dict]:
    """Bind every signed raw step to the exact reviewed direction profile."""
    profile = candidate.get('direction_profile', 'raw-plus')
    directions = directions_for(group, profile)
    # Historical raw-plus candidates have no explicit direction map. A new
    # mirrored candidate must carry its complete signed map as evidence.
    if profile != 'raw-plus' or 'raw_direction_by_id' in candidate:
        require(candidate.get('raw_direction_by_id') == directions,
                'Offline candidate raw directions differ from its profile')
    rows = candidate.get('rows')
    require(type(rows) is list and len(rows) == 12
            and {row.get('id') for row in rows} == set(range(1, 13)),
            'Candidate must name all twelve axes')
    amplitude = candidate.get('amplitude_deg', amplitude_for(group))
    require(type(amplitude) in (int, float)
            and (amplitude in (5., 10.) if group == 'front-hip'
                 else amplitude == amplitude_for(group)),
            'Unsupported role-group amplitude')
    if ('continuous_profile' in candidate or 'continuous_waypoints_deg' in candidate):
        require(group == 'front-hip' and profile == 'front-hip-mirrored'
                and amplitude == 10.
                and candidate.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE
                and candidate.get('continuous_waypoints_deg') == CONTINUOUS_FRONT_HIP_WAYPOINTS,
                'Unsupported continuous front-hip candidate')
    for row in rows:
        mid = row['id']
        expected = amplitude * directions[str(mid)]
        require(row.get('candidate_raw_step_deg') == expected
                and abs(row.get('start_raw_rad') - centers[str(mid)]) < 1e-9
                and abs(row.get('candidate_end_raw_rad') - centers[str(mid)]
                        - math.radians(expected)) < 1e-9,
                f'ID{mid} candidate differs from the selected signed group plan')
    return profile, directions


def prepare(source: Path, hold_path: Path, group: str, directory: Path,
            direction_profile: str = 'raw-plus', amplitude_deg: float | None = None,
            continuous_profile: str | None = None) -> dict:
    disabled_base.validate_source(source)
    require(group in GROUPS and not directory.exists(), 'Use one fresh known role group directory')
    directions = directions_for(group, direction_profile)
    amplitude = amplitude_for(group) if amplitude_deg is None else amplitude_deg
    require(amplitude in ((5., 10.) if group == 'front-hip' else
                          (amplitude_for(group),)), 'Unsupported role-group amplitude')
    require(continuous_profile is None or
            (continuous_profile == CONTINUOUS_FRONT_HIP_PROFILE
             and group == 'front-hip' and direction_profile == 'front-hip-mirrored'
             and amplitude == 10.), 'Unsupported continuous front-hip profile')
    hold = read_json(hold_path)
    boot = read_json(source / 'fullbody-review.json')['boot_id']
    uids = read_json(source / 'fullbody-review.json')['motor_uids']
    centers = disabled_base.validate_hold(hold, sha(hold_path), boot, uids)
    directory.mkdir(parents=True)
    evidence = {'boot_id': boot,
                'revised_active_hold': {'summary_sha256': sha(hold_path),
                                        'status': hold['status']},
                'source': 'same-boot all-twelve current-position hold; no pose calibration'}
    write_json(directory / 'fullbody-hold-evidence.json', evidence)
    rows = []
    for mid in range(1, 13):
        start = centers[str(mid)]
        step = amplitude * directions[str(mid)]
        rows.append({'id': mid, 'start_raw_rad': start, 'candidate_raw_step_deg': step,
                     'candidate_end_raw_rad': start + math.radians(step)})
    candidate = {'boot_id': boot, 'status': 'OFFLINE_CANDIDATE_ONLY',
                 'output_allowed': False, 'executed': False,
                 'role_group': group, 'moving_motor_ids': list(GROUPS[group]),
                 'direction_profile': direction_profile,
                 'amplitude_deg': amplitude,
                 'raw_direction_by_id': directions,
                 'current_hold_source_sha256': sha(directory / 'fullbody-hold-evidence.json'),
                 'rows': rows}
    if continuous_profile is not None:
        candidate['continuous_profile'] = continuous_profile
        candidate['continuous_waypoints_deg'] = list(CONTINUOUS_FRONT_HIP_WAYPOINTS)
    write_json(directory / 'offline-raw-step2-candidate.json', candidate)
    return {'boot_id': boot, 'group': group, 'direction_profile': direction_profile,
            'moving_motor_ids': list(GROUPS[group]),
            'hold_summary_sha256': sha(hold_path), 'candidate_sha256': sha(directory / 'offline-raw-step2-candidate.json')}


def disabled(source: Path, hold_path: Path, prepared: Path, output: Path) -> dict:
    disabled_base.validate_source(source)
    candidate_path = prepared / 'offline-raw-step2-candidate.json'
    evidence_path = prepared / 'fullbody-hold-evidence.json'
    candidate, hold, evidence = (read_json(candidate_path), read_json(hold_path),
                                 read_json(evidence_path))
    group = candidate.get('role_group')
    require(group in GROUPS and not output.exists(), 'Fresh known role-group package required')
    boot = read_json(source / 'fullbody-review.json')['boot_id']
    uids = read_json(source / 'fullbody-review.json')['motor_uids']
    centers = disabled_base.validate_hold(hold, sha(hold_path), boot, uids)
    require(candidate['boot_id'] == evidence['boot_id'] == boot
            and candidate['status'] == 'OFFLINE_CANDIDATE_ONLY'
            and candidate['output_allowed'] is False and candidate['executed'] is False
            and candidate['moving_motor_ids'] == list(GROUPS[group])
            and candidate['current_hold_source_sha256'] == sha(evidence_path)
            and evidence['revised_active_hold']['summary_sha256'] == sha(hold_path)
            and evidence['revised_active_hold']['status'] == hold['status'],
            'Hold evidence/candidate SHA chain mismatch')
    profile, directions = validate_candidate_directions(candidate, group, centers)
    review = {
        'schema': 'rs05-role-group-step1-review-v1',
        'scope': ('supported-front-thigh-two-axis-raw-bounded-other10-hold-50ms-diagnostic'
                  if group == 'front-thigh' else
                  'supported-front-hip-two-axis-raw-bounded-other10-hold-50ms-diagnostic'
                  if group == 'front-hip' else
                  'supported-four-axis-raw-bounded-other8-hold-50ms-diagnostic'),
        'role_group': group, 'direction_profile': profile,
        'motor_ids': list(range(1, 13)), 'motor_uids': uids,
        'firmware': '0.5.0.13', 'sha256': sha(candidate_path),
        'review_complete': True, 'source_files_verified': True,
        'gain_profile': ('role-front-thigh-kp12-front2-hip-kp12-all4-id4-id10-kp4'
                         if group == 'front-thigh' else
                         'role-thigh-kp12-all4-hip-kp12-all4-id4-id10-kp4'
                         if group == 'thigh' else
                         'role-front-hip-kp12-front2-id4-id10-kp4'
                         if group == 'front-hip' else 'id4-id10-kp4'),
        'old_raw_target_reused': False,
        'calibration_verified': False, 'model_mapping_verified': False,
        'learned_policy_allowed': False, 'standing_allowed': False,
        'automatic_retry_allowed': False, **{flag: False for flag in FLAGS},
        'current_hold_status': hold['result']['status'],
        'current_hold_stop_confirmed': True,
        'current_hold_summary_sha256': sha(hold_path),
        'current_hold_gain_profile': 'id4-id10-kp4',
        'supported_step_authorized': False,
        'boot_id': boot, 'current_hold_boot_id': boot,
        'start_raw_rad_by_id': centers,
        'raw_direction_by_id': directions,
        'amplitude_deg': candidate.get('amplitude_deg', amplitude_for(group)),
        'offline_candidate_hold_provenance_verified': True,
        'offline_hold_evidence_sha256': sha(evidence_path),
    }
    if candidate.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE:
        review['continuous_profile'] = CONTINUOUS_FRONT_HIP_PROFILE
        review['continuous_waypoints_deg'] = list(CONTINUOUS_FRONT_HIP_WAYPOINTS)
    shutil.copytree(source, output, symlinks=False)
    for src, dest in ((hold_path, 'current-hold-summary.json'),
                      (evidence_path, 'fullbody-hold-evidence.json'),
                      (candidate_path, 'offline-raw-step2-candidate.json')):
        shutil.copy2(src, output / dest)
    modules = ['fullbody_step10_plan.py', 'rs05_fullbody_step2.py',
               'rs05_joint_trial.py', 'rs05_trial_protocol.py']
    if candidate.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE:
        modules.append('continuous_front_hip_plan.py')
    for name in modules:
        shutil.copy2(RUNTIME / name, output / 'singularitydog_hw' / name)
    write_json(output / 'step2-review.json', review)
    wrapper = output / 'prepared_fullbody.py'
    text = disabled_base.adapt_wrapper(wrapper.read_text(), output.name, boot)
    manifest = pin_package(output, wrapper.name, 'manifest.json', text)
    validate_frozen_feedback_age(output, 'manifest.json')
    return {'package': str(output), 'boot_id': boot, 'group': group,
            'direction_profile': profile,
            'disabled_only': True, 'pinned_files': len(manifest),
            'manifest_sha256': sha(output / 'manifest.json')}


def attach_announcement(text: str) -> str:
    """Place the pinned speaker gate immediately before the active runner."""
    call = "report['result'] = run_fullbody_step2(transports, expected, check, emit,"
    motor_import = 'from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2'
    require(text.count(call) == 1, 'No singular active raw-step invocation')
    require(text.count(motor_import) == 1, 'No singular active raw-step import')
    line = next(line for line in text.splitlines() if call in line)
    indent = line[:line.index(call)]
    import_line = next(line for line in text.splitlines() if motor_import in line)
    import_indent = import_line[:import_line.index(motor_import)]
    require(indent.isspace(), 'Active runner must start on an indented line')
    require(import_indent.isspace(), 'Active runner import must be indented')
    text = text.replace(call,
        "play_test_start(BASE / 'test-start-ja.wav')\n"
        + indent + call, 1)
    text = text.replace(motor_import,
        motor_import + '\n'
        + import_indent + 'from singularitydog_hw.i2s_announcement import play_test_start', 1)
    modules = "'rs05_step2_packet_gate'"
    require(text.count(modules) == 1, 'No singular imported-module audit')
    text = text.replace(modules, "'rs05_step2_packet_gate', 'i2s_announcement'", 1)
    ast.parse(text)
    return text


def validate_active_directions(review: dict, candidate: dict, physical: dict,
                               centers: dict) -> str:
    group = review['role_group']
    profile, directions = validate_candidate_directions(candidate, group, centers)
    scope = ('supported-front-thigh-current-raw-toward-face-10deg-diagnostic-only'
             if group == 'front-thigh' else
             f'supported-front-hip-current-raw-mirrored-{review.get("amplitude_deg", 5.):g}deg-diagnostic-only'
             if group == 'front-hip' else
             'supported-four-thigh-current-raw-mirrored-10deg-diagnostic-only'
             if profile == 'mirrored-thigh' else
             'supported-four-thigh-current-raw-plus-10deg-diagnostic-only'
             if group == 'thigh' else
             'supported-four-axis-current-raw-plus-1deg-diagnostic-only')
    require(review.get('direction_profile', 'raw-plus') == profile
            and review['raw_direction_by_id'] == directions
            and physical.get('direction_profile', 'raw-plus') == profile
            and physical.get('raw_direction_by_id') == directions
            and physical.get('scope') == scope,
            'Candidate, disabled review and physical review must agree on exact raw directions')
    continuous = review.get('continuous_profile')
    if (continuous is not None or candidate.get('continuous_profile') is not None
            or physical.get('continuous_profile') is not None):
        require(continuous == candidate.get('continuous_profile')
                == physical.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE
                and review.get('continuous_waypoints_deg')
                    == candidate.get('continuous_waypoints_deg')
                    == physical.get('continuous_waypoints_deg')
                    == CONTINUOUS_FRONT_HIP_WAYPOINTS,
                'Continuous front-hip profile differs across candidate and reviews')
    return profile


def validate_role_group_preflight(disabled_package: Path, summary_path: Path,
                                  events_path: Path, boot: str) -> None:
    active_base.validate_step2_preflight(summary_path, events_path, boot)
    review = read_json(disabled_package / 'step2-review.json')
    summary = read_json(summary_path)
    result = summary.get('result', {})
    require(summary.get('wrapper_sha256') == sha(disabled_package / 'prepared_fullbody.py')
            and result.get('review') == review
            and result.get('moving_motor_ids') == list(GROUPS[review['role_group']])
            and result.get('gain_profile') == review['gain_profile']
            and result.get('raw_diagnostic_only') is True,
            'Disabled preflight does not belong to this exact role-group package')


def front_hip_step10_preflight_centers(summary_path: Path) -> dict[str, float]:
    """Read all twelve starts from one completed disabled preflight summary."""
    summary = read_json(summary_path)
    workers = summary.get('result', {}).get('workers', {})
    require(type(workers) is dict and set(workers) == {'front', 'rear'},
            'Front-hip 10-degree preflight lacks both measured buses')
    reference = {}
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        centers = workers[bus].get('centers', {})
        require(type(centers) is dict and set(centers) == {str(mid) for mid in ids},
                f'Front-hip 10-degree preflight lacks six {bus} centers')
        for mid in ids:
            angle = centers[str(mid)]
            require(type(angle) in (int, float) and math.isfinite(angle),
                    f'ID{mid} preflight clearance center is invalid')
            reference[str(mid)] = angle
    return reference


def front_hip_step10_clearance_extras(review: dict, physical: dict,
                                      summary_path: Path) -> dict:
    """Bind physical 10-degree clearance to all twelve measured preflight starts."""
    if review.get('role_group') != 'front-hip' or review.get('amplitude_deg') != 10.:
        return {}
    reference = front_hip_step10_preflight_centers(summary_path)
    note = physical.get('start_tolerance_clearance_note')
    require(physical.get('clearance_reference_raw_rad_by_id') == reference
            and physical.get('clearance_reference_preflight_summary_sha256') == sha(summary_path)
            and type(physical.get('start_tolerance_clearance_verified_deg')) in (int, float)
            and physical['start_tolerance_clearance_verified_deg'] == CLEARANCE_START_TOLERANCE_DEG
            and type(note) is str and bool(note.strip()),
            'Front-hip 10-degree physical review must cover the preflight pose +/-3 degrees')
    extras = {'clearance_reference_raw_rad_by_id': reference,
            'clearance_reference_preflight_summary_sha256': sha(summary_path),
            'start_tolerance_clearance_verified_deg': CLEARANCE_START_TOLERANCE_DEG,
            'start_tolerance_clearance_note': note}
    if review.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE:
        require(physical.get('continuous_19s_reviewed') is True,
                'Continuous 19-second test must be explicitly reviewed')
        extras['continuous_19s_reviewed'] = True
    return extras


def active(source: Path, disabled_package: Path, preflight_log: Path,
           physical_path: Path, output: Path,
           announcement_wav: Path = DEFAULT_ANNOUNCEMENT_WAV) -> dict:
    active_base.validate_frozen(source, 'prepared_current_hold.py', 'active-manifest.json')
    active_base.validate_frozen(disabled_package, 'prepared_fullbody.py', 'manifest.json')
    require(not output.exists(), 'Use a fresh active package directory')
    audio_spec = importlib.util.spec_from_file_location('build_i2s_announcement',
                                                        RUNTIME / 'i2s_announcement.py')
    require(audio_spec is not None and audio_spec.loader is not None,
            'Cannot load I2S announcement validator')
    audio = importlib.util.module_from_spec(audio_spec)
    audio_spec.loader.exec_module(audio)
    audio.validate_announcement(announcement_wav)
    review = read_json(disabled_package / 'step2-review.json')
    physical = read_json(physical_path)
    boot, group = review['boot_id'], review['role_group']
    candidate_path = disabled_package / 'offline-raw-step2-candidate.json'
    hold_path = disabled_package / 'current-hold-summary.json'
    candidate = read_json(candidate_path)
    hold = read_json(hold_path)
    centers = disabled_base.validate_hold(hold, sha(hold_path), boot, review['motor_uids'])
    require(candidate.get('boot_id') == boot and review['sha256'] == sha(candidate_path),
            'Active directions need the exact same-boot signed candidate')
    profile = validate_active_directions(review, candidate, physical, centers)
    summary_path, events_path = preflight_log / 'summary.json', preflight_log / 'events.jsonl'
    validate_role_group_preflight(disabled_package, summary_path, events_path, boot)
    clearance_extras = front_hip_step10_clearance_extras(review, physical, summary_path)
    require(review['supported_step_authorized'] is False
            and review['offline_candidate_hold_provenance_verified'] is True
            and all(review[flag] is False for flag in FLAGS)
            and physical.get('boot_id') == boot
            and physical.get('role_group') == group
            and physical.get('source_disabled_review_sha256')
                == sha(disabled_package / 'step2-review.json')
            and physical.get('old_raw_target_reused') is False
            and all(physical.get(flag) is True for flag in FLAGS)
            and physical.get('calibration_verified') is False
            and physical.get('model_mapping_verified') is False
            and physical.get('standing_allowed') is False
            and physical.get('learned_policy_allowed') is False
            and physical.get('automatic_retry_allowed') is False
            and physical.get('requires_fresh_pre_run_confirmation') is True,
            'Physical review must match this exact role-group raw scope')
    approved = {**review, **{flag: True for flag in FLAGS}, **clearance_extras,
                'supported_step_authorized': True,
                'source_disabled_step2_review_sha256': sha(disabled_package / 'step2-review.json'),
                'source_step2_preflight_summary_sha256': sha(summary_path),
                'source_step2_preflight_events_sha256': sha(events_path),
                'source_physical_review_sha256': sha(physical_path)}
    shutil.copytree(source, output, symlinks=False)
    modules = ['rs05_fullbody_step2.py', 'rs05_joint_trial.py',
               'fullbody_step10_plan.py', 'rs05_step2_packet_gate.py',
               'rs05_trial_protocol.py', 'rs05_bus_transport.py',
               'i2s_announcement.py']
    continuous = review.get('continuous_profile') == CONTINUOUS_FRONT_HIP_PROFILE
    if continuous:
        modules.append('continuous_front_hip_plan.py')
    for name in modules:
        shutil.copy2(RUNTIME / name, output / 'singularitydog_hw' / name)
    shutil.copy2(announcement_wav, output / 'test-start-ja.wav')
    tests = ['test_rs05_fullbody_step2.py', 'test_rs05_step2_packet_gate.py',
                 'test_rs05_bus_transport.py', 'test_rs05_bus_interleave.py',
                 'test_rs05_joint_trial.py', 'test_rs05_leg_pacing.py',
                 'test_rs05_leg_transport.py']
    if continuous:
        tests.append('test_continuous_front_hip_plan.py')
        tests.append('test_rs05_front_hip_continuous_runtime.py')
    for name in tests:
        shutil.copy2(TESTS / name, output / 'tests' / name)
    for src, dest in ((disabled_package / 'step2-review.json', 'step2-disabled-review.json'),
                      (disabled_package / 'offline-raw-step2-candidate.json', 'offline-raw-step2-candidate.json'),
                      (disabled_package / 'current-hold-summary.json', 'current-hold-summary.json'),
                      (disabled_package / 'fullbody-hold-evidence.json', 'fullbody-hold-evidence.json'),
                      (physical_path, 'physical-review.json')):
        shutil.copy2(src, output / dest)
    (output / 'step2-preflight').mkdir()
    shutil.copy2(summary_path, output / 'step2-preflight' / 'summary.json')
    shutil.copy2(events_path, output / 'step2-preflight' / 'events.jsonl')
    write_json(output / 'step2-active-review.json', approved)
    wrapper = output / 'prepared_current_hold.py'
    text = active_base.adapt_wrapper(wrapper.read_text(), output.name, boot)
    # The inherited hold proof pins the pre-role protocol codec. This finite
    # role package requires a new, separately pinned role-specific codec at
    # the same import path. Let the historical proof verify the exact new
    # codec hash from this package's outer manifest; all other historical
    # source hashes remain unchanged.
    old = '    proof.verify_files(base)\n'
    require(text.count(old) == 1, 'Unexpected inherited hold proof call')
    text = text.replace(old,
        "    proof.PINS = {**proof.PINS, **{name: PINS[name] for name in "
        "('singularitydog_hw/rs05_trial_protocol.py', "
        "'singularitydog_hw/rs05_bus_transport.py')}}\n"
        + old, 1)
    label = ('RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED'
             if continuous else 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED')
    text = text.replace('RAW_STEP2_COMPLETED_RESET_CONFIRMED', label)
    axis_count = len(GROUPS[group])
    text = text.replace('all-twelve raw 2-degree diagnostic',
                        f'{axis_count}-axis raw {approved["amplitude_deg"]:g}-degree diagnostic')
    text = text.replace('raw2-degree diagnostic',
                        f'{axis_count}-axis raw{approved["amplitude_deg"]:g}-degree diagnostic')
    text = text.replace('--execute-raw-step2', '--execute-role-group-step1')
    text = text.replace('execute_raw_step2', 'execute_role_group_step1')
    text = attach_announcement(text)
    if continuous:
        import_line = '        from singularitydog_hw.rs05_fullbody_step2 import run_fullbody_step2\n'
        require(text.count(import_line) == 1, 'Continuous wrapper lost runner import')
        text = text.replace(import_line,
                            import_line + '        from singularitydog_hw import continuous_front_hip_plan\n',
                            1)
        old_ticks = "{'front': 180, 'rear': 180}"
        require(text.count(old_ticks) == 1, 'Continuous wrapper lost exact 180-tick audit')
        text = text.replace(old_ticks,
                            f"{{'front': {CONTINUOUS_FRONT_HIP_TICKS}, 'rear': {CONTINUOUS_FRONT_HIP_TICKS}}}",
                            1)
        module_audit = "'rs05_step2_packet_gate', 'i2s_announcement'"
        require(text.count(module_audit) == 1, 'Continuous module audit is missing')
        text = text.replace(module_audit,
                            module_audit + ", 'continuous_front_hip_plan'", 1)
    manifest = pin_package(output, wrapper.name, 'active-manifest.json', text)
    validate_frozen_feedback_age(output, 'active-manifest.json')
    if group == 'front-hip':
        validate_frozen_front_hip_transport(output)
    return {'package': str(output), 'boot_id': boot, 'group': group,
            'direction_profile': profile,
            'raw_diagnostic_only': True, 'standing_allowed': False,
            'pinned_files': len(manifest), 'manifest_sha256': sha(output / 'active-manifest.json')}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='stage', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--source-disabled', type=Path, required=True)
    p.add_argument('--current-hold-summary', type=Path, required=True)
    p.add_argument('--group', choices=GROUPS, required=True)
    p.add_argument('--direction-profile', choices=DIRECTION_PROFILES,
                   default='raw-plus')
    p.add_argument('--amplitude-deg', type=float)
    p.add_argument('--continuous-front-hip-5-10', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('disabled')
    p.add_argument('--source-disabled', type=Path, required=True)
    p.add_argument('--current-hold-summary', type=Path, required=True)
    p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('active')
    p.add_argument('--source-active', type=Path, required=True)
    p.add_argument('--disabled-package', type=Path, required=True)
    p.add_argument('--preflight-log', type=Path, required=True)
    p.add_argument('--physical-review', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--announcement-wav', type=Path, default=DEFAULT_ANNOUNCEMENT_WAV)
    args = ap.parse_args(argv)
    if args.stage == 'prepare':
        result = prepare(args.source_disabled, args.current_hold_summary,
                         args.group, args.output, args.direction_profile,
                         args.amplitude_deg,
                         CONTINUOUS_FRONT_HIP_PROFILE
                         if args.continuous_front_hip_5_10 else None)
    elif args.stage == 'disabled':
        result = disabled(args.source_disabled, args.current_hold_summary,
                          args.prepared, args.output)
    else:
        result = active(args.source_active, args.disabled_package,
                        args.preflight_log, args.physical_review, args.output,
                        args.announcement_wav)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
