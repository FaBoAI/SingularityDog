#!/usr/bin/env python3
"""Build a new private kit with the unapproved supported-policy output tooling.

File copying/hashing only. Reuses the diagnostic package builder; does not
compile, SSH, open hardware, approve a profile or change dog_tomorrow actions.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'runtime'))
import prepare_overnight_bundle as overnight
import audit_angle_calibration as angle_audit
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw.ground_trial_plan import template_ground_plan, STAGES
from singularitydog_hw.ground_trial_review import physical_review_template

PROFILE_PATH = 'inputs/policy-profile-template.json'
DESIGN_PATH = 'docs/policy-output-design-20260928.md'
VALIDATION_PATH = 'docs/policy-output-validation-20260928.md'
GROUND_RUNBOOK_PATH = 'docs/ground-trials-20260928.md'
READINESS_PATH = 'docs/commissioning-readiness-20260928.md'
LATENCY_PATH = 'docs/active-loop-latency-budget-20260928.md'
TODAY_PATH = 'docs/today-test-plan-20260928.md'
TODAY_EVIDENCE_PATH = 'evidence/today-preparation-20260928.json'
PUBLIC_EVIDENCE_PATH = 'evidence/policy-output-software-validation-20260928.json'
PREPARATION_EVIDENCE_PATH = 'evidence/commissioning-preparation-20260928.json'
PRELOAD_PROFILE_PATH = 'inputs/supported-preload-profile-template.json'
PRELOAD_SOURCE_PATH = 'runtime/singularitydog_hw/supported_preload_path.py'
PRELOAD_DOC_PATHS = ('docs/supported-preload-runtime-20260930.md',
                     'docs/walking-plan-20260930.md', 'docs/system-readiness-20260930.md')
CURRENT_VALIDATION_DOC_PATHS = ('docs/today-validation-20261005.md',
                              'docs/walking-week-20261005.md',
                              'docs/angle-final-review-20261005.md',
                              'docs/imu-fixed-mount-20260922.md')
GROUND_SOURCE_PATHS = (
    'runtime/singularitydog_hw/ground_trial_plan.py',
    'runtime/singularitydog_hw/ground_trial_trajectory.py',
    'runtime/singularitydog_hw/ground_trial_output.py',
    'runtime/singularitydog_hw/ground_trial_review.py',
)
ACTIVE_SOURCE_PATHS = (
    'runtime/experiments/native_active_transport/transport.cpp',
    'runtime/experiments/native_active_transport/build.py',
    'runtime/singularitydog_hw/native_active_transport.py',
    'runtime/singularitydog_hw/policy_live_profile.py',
    'runtime/singularitydog_hw/policy_output.py',
    'runtime/singularitydog_hw/math_thread_startup.py',
    'runtime/singularitydog_hw/policy_output_runtime.py',
    'runtime/singularitydog_hw/active_output_timer_slack.py',
    'runtime/singularitydog_hw/thread_timer_slack.py',
    'runtime/singularitydog_hw/policy_post_reply_timing.py',
    'runtime/singularitydog_hw/policy_output_model.py',
    'runtime/singularitydog_hw/policy_motion_envelope.py',
)


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _save(path, data):
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('wb') as stream:
        os.chmod(path, 0o600)
        stream.write(raw)


def _new_private_output(output):
    out = Path(output).expanduser().absolute()
    if out.exists() or out.is_symlink():
        raise ValueError('Kit output must be new; an existing package is never overwritten')
    resolved = out.resolve()
    if any((ancestor/'.git').exists() for ancestor in (resolved, *resolved.parents)):
        raise ValueError('Private kit with motor UIDs/model must be outside Git')
    return resolved


def _select_angle_profile(stage, path, pins):
    """Keep an explicitly selected review profile instead of a historical zero.

    Selection copies provenance, not a current-power capture or runtime permit.
    Referenced evidence is copied and rebound so the kit remains self-contained.
    """
    source=overnight._regular_source(Path(path).expanduser())
    before=_sha(source)
    profile,contracts=angle_audit.load_profile(source)
    if any(row.calibration_sha256!=before for row in contracts.values()):
        raise ValueError('Selected angle profile changed during validation')
    expected=json.loads((stage/'inputs/expected-uids.json').read_text())
    if {str(mid):row.uid for mid,row in contracts.items()}!=expected:
        raise ValueError('Selected angle profile differs from snapshot motor UIDs')
    overnight.copy_source(source,stage/'inputs/selected-angle-profile-original.json',pins)
    if pins[source]!=before:raise ValueError('Selected angle profile changed during validation')
    evidence_map={}
    for digest,name in profile.get('evidence_files',{}).items():
        if (type(digest) is not str or len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest)
                or type(name) is not str or not name):raise ValueError('Invalid selected angle evidence')
        evidence=Path(name).expanduser()
        if not evidence.is_absolute():evidence=source.parent/evidence
        evidence=overnight._regular_source(evidence)
        target='angle-evidence/'+digest+evidence.suffix
        overnight.copy_source(evidence,stage/'inputs'/target,pins)
        if pins[evidence]!=digest:raise ValueError('Selected angle evidence changed or hash differs')
        evidence_map[digest]=target
    profile=dict(profile,evidence_files=evidence_map)
    _save(stage/'inputs/angle-profile.json',profile)
    angle_audit.load_profile(stage/'inputs/angle-profile.json')
    return {'selection':'explicit_review_profile','original_sha256':before,
            'packaged_sha256':_sha(stage/'inputs/angle-profile.json'),
            'motor_uid_match_verified':True,'current_boot_or_power_verified':False,
            'approved_for_runtime':False}


def build(snapshot_home, output, *, profile_schema=live.SCHEMA_V2, angle_profile=None):
    """Publish a new kit; final manifest exists only after all files are copied."""
    if profile_schema not in (live.SCHEMA_V2, live.SCHEMA_V3):
        raise ValueError('An explicit supported V2 or V3 profile schema is required')
    out = _new_private_output(output)
    # Fail before making an output if this checkout lacks part of the new path.
    required = (*ACTIVE_SOURCE_PATHS, *GROUND_SOURCE_PATHS, DESIGN_PATH, GROUND_RUNBOOK_PATH,
                *CURRENT_VALIDATION_DOC_PATHS)
    if profile_schema == live.SCHEMA_V3:required += (PRELOAD_SOURCE_PATH,)
    for name in required:
        source = ROOT/name
        if not source.is_file() or source.is_symlink():
            raise ValueError('Required policy-output source missing or symlinked: '+name)
    out.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.policy-kit-stage-', dir=out.parent) as stage_dir:
        stage = Path(stage_dir)/'kit'
        overnight.build(snapshot_home, stage)
        source_pins = {}
        angle_selection=(_select_angle_profile(stage,angle_profile,source_pins) if angle_profile is not None else
                         {'selection':'historical_snapshot_diagnostic_only','current_boot_or_power_verified':False,
                          'approved_for_runtime':False})
        candidate = live.template(schema=profile_schema)
        if profile_schema == live.SCHEMA_V3:
            # Pin the bytes actually copied into the kit. A concurrent source
            # edit during packaging must not make the candidate look valid.
            candidate['cadence_source_sha256'] = {
                name: _sha(stage/'runtime'/name) for name in live.CADENCE_SOURCE_PATHS
            }
            if candidate['cadence_source_sha256'] != live.cadence_source_hashes():
                raise ValueError('Cadence source changed while packaging V3')
        if (candidate.get('approved_for_supported_policy_output') is not False or
                candidate.get('review') is not None or not candidate.get('blockers') or
                any(value is not None for axis in candidate.get('axes', {}).values() for value in axis.values())):
            raise ValueError('Packaging must never create an approved output profile')
        _save(stage/PROFILE_PATH, candidate)
        preload = None
        if profile_schema == live.SCHEMA_V3:
            preload = live.supported_preload_template()
            preload['cadence_source_sha256'] = {
                name: _sha(stage/'runtime'/name) for name in live.cadence_source_paths(preload)}
            if preload['cadence_source_sha256'] != live.cadence_source_hashes(preload):
                raise ValueError('Preload source changed while packaging V3')
            if (preload.get('approved_for_supported_policy_output') is not False
                    or preload.get('review') is not None or not preload.get('blockers')
                    or any(value is not None for axis in preload['axes'].values() for value in axis.values())
                    or any(value is not None for ref in preload['artifacts'].values() for value in ref.values())):
                raise ValueError('Packaging must never approve a preload profile or invent its evidence')
            _save(stage/PRELOAD_PROFILE_PATH, preload)
        copied_docs = []
        for name in (DESIGN_PATH, VALIDATION_PATH, GROUND_RUNBOOK_PATH, READINESS_PATH, LATENCY_PATH, TODAY_PATH,
                     'docs/hardware-native-host-processing-20260925.md', *PRELOAD_DOC_PATHS,
                     *CURRENT_VALIDATION_DOC_PATHS):
            source = ROOT.resolve()/name
            if source.exists():
                if not source.is_file() or source.is_symlink():
                    raise ValueError('Nonregular documentation source: '+name)
                target = stage/name; target.parent.mkdir(parents=True, exist_ok=True)
                overnight.copy_source(source, target, source_pins); copied_docs.append(name)
        # Compact public software results only; raw robot logs remain private
        # inputs selected by the base builder, never added by this document link.
        for name in (PUBLIC_EVIDENCE_PATH, PREPARATION_EVIDENCE_PATH, TODAY_EVIDENCE_PATH):
            evidence = ROOT.resolve()/name
            if evidence.exists():
                if not evidence.is_file() or evidence.is_symlink():
                    raise ValueError('Nonregular public validation evidence')
                target=stage/name;target.parent.mkdir(parents=True,exist_ok=True)
                overnight.copy_source(evidence,target,source_pins)
        # Every active source is pinned independently of its eventual target-ABI
        # binary. The global kit manifest also covers every copied source byte.
        sources = {name: _sha(stage/name) for name in ACTIVE_SOURCE_PATHS}
        config_path = stage/'kit-config.json'
        config = json.loads(config_path.read_text())
        config['validation_preparation'] = {
            'runbooks': [name for name in CURRENT_VALIDATION_DOC_PATHS if name in copied_docs],
            'hardware_results_included': False,
            'calibration_approved_for_runtime': False,
            'output_allowed': False,
            'source_of_historical_inputs': 'saved snapshot; current boot/power must be acquired on target',
            'angle_profile_selection':angle_selection,
        }
        config['supported_policy_output'] = {
            'profile_template': PROFILE_PATH,
            'profile_template_sha256': _sha(stage/PROFILE_PATH),
            'approved_for_supported_policy_output': False,
            'scope': 'supported_characterization_only',
            'profile_schema': profile_schema,
            'default_mode': 'PLAN_ONLY',
            'entry_module': 'singularitydog_hw.policy_output',
            'active_transport_source': ACTIVE_SOURCE_PATHS[0],
            'active_transport_build': ACTIVE_SOURCE_PATHS[1],
            'active_transport_library': 'runtime/experiments/native_active_transport/libdog_active_transport.so',
            'active_transport_binary_included': False,
            'build_active_library_on_target_required': True,
            'source_sha256': sources,
            'design': DESIGN_PATH,
            'validation': VALIDATION_PATH if VALIDATION_PATH in copied_docs else None,
            'readiness': READINESS_PATH if READINESS_PATH in copied_docs else None,
            'diagnostic_entry_actions_unchanged': True,
        }
        ground_templates = {}
        for stage_name in STAGES:
            pending = template_ground_plan(stage_name, candidate)
            if pending['approved_for_ground_trial'] is not False or pending['review'] is not None or not pending['blockers']:
                raise ValueError('Packaging must never approve a ground stage')
            name = f'inputs/ground-{stage_name}-template.json'
            _save(stage/name, pending)
            ground_templates[stage_name] = {'path': name, 'sha256': _sha(stage/name)}
        physical_review_path = 'inputs/ground-physical-review-template.json'
        _save(stage/physical_review_path, physical_review_template())
        config['ground_trials'] = {
            'scope': 'bounded_ground_characterization', 'default_mode': 'PLAN_ONLY',
            'approved_for_ground_trial': False, 'entry_module': 'singularitydog_hw.ground_trial_output',
            'review_module': 'singularitydog_hw.ground_trial_review',
            'stage_templates': ground_templates,
            'physical_review_template': {'path':physical_review_path,'sha256':_sha(stage/physical_review_path)},
            'source_sha256': {name:_sha(stage/name) for name in GROUND_SOURCE_PATHS},
            'runbook': GROUND_RUNBOOK_PATH, 'prior_hardware_reviews_required': True,
            'diagnostic_entry_actions_unchanged': True,
        }
        config['today_preparation'] = {
            'runbook': TODAY_PATH if TODAY_PATH in copied_docs else None,
            'default_mode': 'PLAN_ONLY', 'diagnostic_action': 'diagnostics',
            'profile_assembly_tool': 'tools/prepare_supported_profile.py',
            'watchdog_entry_module': 'singularitydog_hw.watchdog_commissioning',
            'watchdog_scope': 'supported_zero_gain_command_loss_only',
            'usb_disconnect_evidence_generated': False,
            'supported_policy_profile_approved': False,
        }
        config['supported_preload'] = {
            'default_mode': 'PLAN_ONLY', 'approved_for_supported_policy_output': False,
            'profile_schema': live.SCHEMA_V3, 'diagnostic_timing_acceptance': live.SUPPORTED_PRELOAD_5S,
            'template_included': preload is not None,
            'profile_template': PRELOAD_PROFILE_PATH if preload is not None else None,
            'profile_template_sha256': _sha(stage/PRELOAD_PROFILE_PATH) if preload is not None else None,
            'entry_module': 'singularitydog_hw.policy_output',
            'execution_flag': '--execute-supported-preload',
            'absolute_epoch_cadence_required': True, 'support_must_remain': True,
            'support_removal_authorized': False, 'walking_authorized': False,
            'physical_evidence_required': True, 'current_power_diagnostic_required': True,
            'automatic_retry': False,
            'runbooks': [name for name in PRELOAD_DOC_PATHS if name in copied_docs],
            'preparation_note': ('Fill current, reviewed evidence; this template never authorizes output.'
                if preload is not None else 'Rebuild with --profile-schema singularitydog.supported-policy-profile.v3 for the opt-in preload template.'),
        }
        _save(config_path, config)
        files = {}
        for path in sorted(stage.rglob('*')):
            if path.is_symlink():
                raise ValueError('Symlink in staged kit: '+str(path.relative_to(stage)))
            if path.is_file() and path != stage/'kit-manifest.json':
                # The one intentional binary is the pinned private model.
                if path.suffix in ('.so', '.dylib', '.dll', '.pyc'):
                    raise ValueError('Host binary/cache cannot be included')
                files[str(path.relative_to(stage))] = _sha(path)
        manifest = {
            'schema': 'private-overnight-kit-v1', 'hardware_accessed': False,
            'files': files, 'supported_policy_profile_approved': False,
            'policy_profile_template': PROFILE_PATH, 'ground_trials_approved': False,
            'ground_stage_templates': ground_templates,
            'note': 'Private motor UIDs, model and saved diagnostics. Never commit/publish this kit. '
                    'Supported policy output remains unapproved; device tests/review are required.',
        }
        _save(stage/'kit-manifest.json', manifest)
        for path in (stage, *stage.rglob('*')):
            os.chmod(path, 0o700 if path.is_dir() else 0o600)
        overnight.recheck_sources(source_pins)
        # copytree refuses an existing destination even if another process
        # created it since the first check. Never overwrite a prior package.
        # Exclude the manifest until the remaining files have copied fully.
        overnight.publish(stage, out, manifest)
    return {'output': str(out), 'file_count': len(files), 'model_and_logs_private': True,
            'hardware_accessed': False, 'supported_policy_profile_approved': False,
            'profile_schema': profile_schema,
            'active_transport_binary_included': False, 'profile_template': PROFILE_PATH,
            'validation_included': VALIDATION_PATH in copied_docs, 'ground_trials_approved': False,
            'preload_template_included': preload is not None, 'preload_profile_approved': False,
            'ground_stage_templates': ground_templates}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot-home', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--angle-profile',help='Explicit latest unapproved review profile; copy/pin evidence and verify snapshot UIDs')
    parser.add_argument('--profile-schema', choices=(live.SCHEMA_V2, live.SCHEMA_V3),
                        default=live.SCHEMA_V2,
                        help='V3 must be selected explicitly; neither schema approves output')
    args = parser.parse_args(argv)
    print(json.dumps(build(args.snapshot_home, args.output, profile_schema=args.profile_schema,
                           angle_profile=args.angle_profile), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
