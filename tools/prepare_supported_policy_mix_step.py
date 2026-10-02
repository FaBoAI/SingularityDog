"""File-only boxed 10%/5s candidate; explicit named review never substitutes for evidence.

The default writes an unapproved draft. --finalize requires reviewer, time,
rationale and the installed loader's complete admission check. No model,
communication port, audio, or robot command is opened by this tool.
"""
import argparse
import copy
import datetime
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import re
import sys
import uuid

MODE = 'supported-policy-mix-step-10pct-5s-v1'
SCOPE = 'supported_characterization_only'
IDS = tuple(str(i) for i in range(1, 13))
CHANGED_SOURCES = frozenset(('singularitydog_hw/policy_live_profile.py',
                             'singularitydog_hw/native_pipeline_benchmark.py'))
OMIT = ('startup_damping_duration_s', 'fixed_catch', 'human_supported_hold')
DECISIONS = {'voltage_pipeline_acceptance': 'ACCEPT_FEEDBACK_THEN_VOLTAGE',
             'native_batch_encoder_acceptance': 'ACCEPT_NATIVE_BATCH_ENCODER',
             'post_reply_deadline_acceptance': 'ACCEPT_BOUNDED_POST_REPLY_DEADLINE',
             'startup_cycle_acceptance': 'ACCEPT_FIRST_CYCLE_POST_REPLY',
             'supported_mix_step_acceptance': 'ACCEPT_5S_SUPPORTED_LEARNED_MIX_STEP_10PCT'}
REF_NAMES = ('prior_supported_profile', 'prior_supported_report', 'prior_supported_observation',
             'saved_policy_target_sequence', 'policy_mixture_analysis', 'pipeline_diagnostic')


def need(value, message):
    if not value:
        raise ValueError(message)


def digest(value):
    need(type(value) is str and re.fullmatch('[0-9a-f]{64}', value), 'Lowercase SHA256 required')
    return value


def clear_reviews(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key == 'review': value[key] = None
            else: clear_reviews(item)
    elif isinstance(value, list):
        for item in value: clear_reviews(item)


def review_factory(reviewer, reviewed_at, rationale):
    need(type(reviewer) is str and reviewer.strip() and type(rationale) is str and
         rationale.strip() and len(rationale) <= 1024, 'Explicit named reviewer/rationale required')
    need(type(reviewed_at) is str, 'Explicit review timestamp required')
    stamp = datetime.datetime.fromisoformat(reviewed_at.replace('Z', '+00:00'))
    need(stamp.utcoffset() is not None, 'Review timestamp requires timezone')
    return lambda decision: dict(decision=decision, reviewer=reviewer,
                                reviewed_at=reviewed_at, rationale=rationale)


def validate_operator(receipt, prior):
    need(receipt.get('schema') == 'singularitydog.supported-mix-step-operator-receipt.v1' and
         receipt.get('boot_id') == prior['boot_id'] and
         receipt.get('motor_power_epoch') == prior['motor_power_epoch'], 'Current operator boot/power differs')
    for key in ('source_id', 'question', 'answer'):
        need(type(receipt.get(key)) is str and receipt[key].strip(), 'Original operator '+key+' required')
    need(type(receipt.get('clearance_deg')) in (int, float) and receipt['clearance_deg'] == 7 and
         all(receipt.get(key) is True for key in ('all_twelve_current_clearance_confirmed',
             'box_supports_body', 'four_paws_floor', 'hands_clear', 'cutoff_ready',
             'power_and_pose_unchanged_since_prior_10s')) and
         all(receipt.get(key) is False for key in ('box_removal_authorized',
             'load_bearing_verified', 'standing_verified')), 'Explicit current seven-degree boxed facts required')


def build_documents(live, documents, refs, new_sources, *, assembly_id,
                    finalize=False, reviewer=None, reviewed_at=None, rationale=None):
    """Pure graph transformation. Facts and source inputs are independently supplied."""
    prior, report = documents['prior_supported_profile'], documents['prior_supported_report']
    observed, operator = documents['prior_supported_observation'], documents['operator_receipt']
    targets, analysis = documents['saved_policy_target_sequence'], documents['policy_mixture_analysis']
    need(prior.get('approved_for_supported_policy_output') is True and prior.get('blockers') == [] and
         prior.get('diagnostic_timing_acceptance') == live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S and
         prior.get('scope') == SCOPE and prior.get('duration_s') == 10. and
         prior.get('policy_weight') == .005, 'Approved same-session learned ten-second predecessor required')
    need(report.get('profile_sha256') == refs['prior_supported_profile']['sha256'] and
         report.get('boot_id') == prior['boot_id'] and report.get('motor_power_epoch') == prior['motor_power_epoch'] and
         report.get('cadence_source_sha256') == prior['cadence_source_sha256'] and
         report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
         all(report.get(key) is True for key in ('normal_ramp_completed', 'learned_targets_sent', 'stop_confirmed')) and
         report.get('deadline20ms_misses') == 0 and report.get('post_reply_deadline_allowance_uses') == 0,
         'Completed current ten-second output with no timing allowance required')
    need(observed.get('report_sha256') == refs['prior_supported_report']['sha256'] and
         observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
         observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
         observed.get('box_support_maintained') is True and
         observed.get('autonomous_standing_or_walking_observed') is False and
         all(type(observed.get(key)) is str and observed[key].strip()
             for key in ('user_statement', 'source_question_item_id', 'statement_source')),
         'Original matching ten-second physical observation required')
    validate_operator(operator, prior)
    old_sources = prior['cadence_source_sha256']
    need(set(old_sources) == set(new_sources), 'Execution source set changed')
    changed = {key: dict(before=old_sources[key], after=new_sources[key])
               for key in old_sources if old_sources[key] != new_sources[key]}
    need(changed and set(changed) <= CHANGED_SOURCES and
         'singularitydog_hw/policy_live_profile.py' in changed, 'Unreviewable executable source delta')
    need(targets.get('profile_sha256') == refs['prior_supported_profile']['sha256'] and
         targets.get('report_sha256') == refs['prior_supported_report']['sha256'] and
         targets.get('kit_manifest_sha256') == refs['replay_kit_manifest']['sha256'] and
         targets.get('model_backend') == prior['model_backend'] and
         targets.get('input_sequence') == 'saved_non_stopping_feedback_and_uncorrected_sensor_IMU' and
         targets.get('feedback_not_generated_by_new_mixture') is True and
         all(targets.get(key) is False for key in ('output_allowed', 'hardware_accessed', 'closed_loop_prediction')),
         'Pinned original-input model replay provenance required')
    model = targets.get('model_provenance', {})
    need(model.get('manifest_sha256') == prior['artifacts']['scalar_step_manifest']['sha256'],
         'Replay scalar model manifest differs')
    recomputed = documents['recomputed_analysis']
    need(analysis == recomputed and analysis.get('sequence_extent') == 'logged_model_call_count_matches' and
         analysis.get('target_rows') == report.get('actual_model_calls'), 'Full exact saved-mixture analysis required')
    low = next((row for row in analysis['mixtures'] if row['weight'] == .1), None)
    need(low is not None and low['physical_clearance_exceeded_ids'] == [] and
         low['maximum_displacement_exceeded_ids'] == [], 'Ten-percent targets exceed seven/six-degree numerical limits')
    p, h, o = copy.deepcopy((prior, documents['hardware_review'], documents['operator_acceptance']))
    need(h.get('reviewed_settings_sha256') == o.get('reviewed_settings_sha256') ==
         live.reviewed_settings_sha256(prior), 'Prior review settings differ')
    clear_reviews(p); clear_reviews(h); clear_reviews(o)
    for key in OMIT: p.pop(key, None)
    p.update(diagnostic_timing_acceptance=MODE, duration_s=5., startup_duration_s=1.,
             policy_ramp_s=2., stop_duration_s=.4, policy_weight=.1, assembly_id=assembly_id,
             cadence_source_sha256=copy.deepcopy(new_sources), approved_for_supported_policy_output=False,
             review=None, blockers=['Final named root review of new sources, strict current501, replay and current7deg boxed clearance required'])
    p['artifacts'].update({key: copy.deepcopy(refs[key]) for key in REF_NAMES})
    for name in ('mix_step_clearance','mix_step_source_review'):
        p['artifacts'][name] = dict(path=name+'.json',sha256='0'*64)
    capture = documents['local_reference_capture']
    local = h['local_characterization']; turns = local['reference_turns_by_id']
    need(capture.get('boot_id') == p['boot_id'] and capture.get('motor_output_allowed') is False,
         'Same current non-driving reference capture required')
    for mid in IDS:
        a, angle = p['axes'][mid], h['angles'][mid]
        need(all(a[key] == value for key,value in dict(kp=3,kd=.15,
             max_command_velocity_rad_s=math.radians(1),max_command_acceleration_rad_s2=math.radians(5),
             max_tracking_error_rad=math.radians(2),max_displacement_from_start_rad=math.radians(1),
             max_estimated_pd_torque_nm=.1,max_measured_velocity_rad_s=.35,
             max_measured_torque_nm=1.,max_temperature_c=45.).items()) and a['uncertainty_rad'] is None and
             h['type2_dynamic'][mid]['velocity_scale_and_sign_verified'] is False and
             h['type2_dynamic'][mid]['torque_interpretation_verified'] is False, 'Calibration/dynamic uncertainty must remain unknown')
        need(type(turns[mid]) is int and capture['identities'][mid]['mcu_uid_hex'] == a['uid'],
             'Reference branch/UID differs')
        q = a['sign'] * (capture['telemetry']['rows'][mid]['median_position_rad'] - turns[mid]*2*math.pi) + a['offset_rad']
        index = live.shadow.CAN_ORDER.index(int(mid))
        lo, hi = max(live.shadow.LOWER[index], q-math.radians(7)), min(live.shadow.UPPER[index], q+math.radians(7))
        need(lo+live.LOCAL_NUMERICAL_MARGIN_RAD < q < hi-live.LOCAL_NUMERICAL_MARGIN_RAD,
             'Numerical margin consumes current model range: ID'+mid)
        a.update(max_command_velocity_rad_s=.12, max_command_acceleration_rad_s2=.5,
                 max_tracking_error_rad=math.radians(3), max_displacement_from_start_rad=math.radians(6),
                 max_estimated_pd_torque_nm=.25, physical_lower_rad=lo, physical_upper_rad=hi)
        for key in ('sign','offset_rad','physical_lower_rad','physical_upper_rad','uncertainty_rad'):
            angle[key] = a[key]
    local['local_clearance_rad'] = math.radians(7)
    clearance = dict(schema='singularitydog.supported-mix-step-clearance.v1', mode=MODE, scope=SCOPE,
        boot_id=p['boot_id'], motor_power_epoch=p['motor_power_epoch'], capture_sha256=p['artifacts']['local_reference_capture']['sha256'],
        uids_by_id={mid:p['axes'][mid]['uid'] for mid in IDS}, reference_turns_by_id=copy.deepcopy(turns),
        local_clearance_rad=math.radians(7), support_must_remain=True, four_paws_floor=True,
        hands_off=True, cutoff_ready=True, current_pose_unchanged=True, current_power_unchanged=True,
        box_removal_allowed=False, standing_allowed=False, walking_allowed=False, load_bearing_verified=False,
        user_statement=operator['answer'], source_receipt=copy.deepcopy(refs['operator_receipt']), review=None)
    source = dict(schema='singularitydog.supported-mix-step-source-review.v1', prior_source_sha256=copy.deepcopy(old_sources),
        new_source_sha256=copy.deepcopy(new_sources), changed_sources=changed,
        target_sequence_sha256=refs['saved_policy_target_sequence']['sha256'],
        replay_input_report_sha256=refs['prior_supported_report']['sha256'],
        replay_source=copy.deepcopy(refs['replay_source']), replay_kit_manifest=copy.deepcopy(refs['replay_kit_manifest']),
        model_values_unchanged=True, replay_uses_original_inputs=True, recorded_feedback_not_new_mix_feedback=True,
        closed_loop_prediction=False, standing_prediction=False, output_allowed=False, review=None)
    h['assembly_id'] = assembly_id
    for key in ('rare_jitter_diagnostic_acceptance','supported_extension_acceptance'):
        h.pop(key, None)
    h['voltage_pipeline_acceptance']['diagnostic_sha256'] = refs['pipeline_diagnostic']['sha256']
    h['supported_mix_step_acceptance'] = dict(mode=MODE, scope=SCOPE,
        prior_profile_sha256=refs['prior_supported_profile']['sha256'], prior_report_sha256=refs['prior_supported_report']['sha256'],
        prior_observation_sha256=refs['prior_supported_observation']['sha256'],
        target_sequence_sha256=refs['saved_policy_target_sequence']['sha256'], mixture_analysis_sha256=refs['policy_mixture_analysis']['sha256'],
        clearance_sha256=None, source_review_sha256=None,
        capture_sha256=p['artifacts']['local_reference_capture']['sha256'],
        diagnostic_sha256=refs['pipeline_diagnostic']['sha256'], support_must_remain=True, load_bearing_not_established=True,
        box_removal_allowed=False, standing_allowed=False, walking_allowed=False, review=None)
    note = ('UNAPPROVED boxed10%/5s Kp3/Kd.15; vcmd.12/acmd.5/tracking3deg/displacement6deg/estimatedPD.25. '
            'Original-input replay is not a closed-loop or standing prediction. Absolute/dynamic calibration remains unknown; '
            'box removal, load transfer, standing and walking remain forbidden. Final actual-source/501/named review pending.')
    h['engineering_monitor_note'] = h['timing_budget_rationale'] = note
    o['operator_confirmation_source'] = copy.deepcopy(refs['operator_receipt'])
    o['physical_confirmation_statement'] = operator['answer']
    settings = live.reviewed_settings_sha256(p)
    h['reviewed_settings_sha256'] = o['reviewed_settings_sha256'] = settings
    if finalize:
        review = review_factory(reviewer, reviewed_at, rationale)
        p['review'] = h['review'] = review('APPROVED_SUPPORTED_CHARACTERIZATION')
        o['review'] = review('ACCEPT_COMMAND_LOSS_ONLY_SUPPORTED_TRIAL')
        clearance['review'] = review('ACCEPT_CURRENT_7DEG_SUPPORTED_MIX_STEP_CLEARANCE')
        source['review'] = review('ACCEPT_SUPPORTED_MIX_STEP_SOURCE_DELTA')
        for key, decision in DECISIONS.items():
            if key in h: h[key]['review'] = review(decision)
        h['engineering_monitor_note'] = h['timing_budget_rationale'] = rationale
        p['approved_for_supported_policy_output'] = True; p['blockers'] = []
    else:
        need(all(value is None for value in (reviewer, reviewed_at, rationale)), 'Review arguments require explicit --finalize')
    need(p['start_pose_bounds'] == prior['start_pose_bounds'], 'Start-pose bounds changed')
    live._structure(p); live._settings(p)
    return p, h, o, clearance, source, settings


class Evidence:
    def __init__(self): self.pins = {}
    def bytes(self, path, expected=None):
        p = Path(path)
        need(p.is_absolute() and all(not parent.is_symlink() for parent in (p,*p.parents)) and
             p.is_file() and p.stat().st_size <= 128*1024*1024, 'Absolute bounded regular evidence required')
        raw = p.read_bytes(); actual = hashlib.sha256(raw).hexdigest()
        need(len(raw) <= 128*1024*1024 and (expected is None or actual == digest(expected)), 'Evidence SHA/size differs: '+p.name)
        key = str(p.resolve()); need(key not in self.pins or self.pins[key] == actual, 'Evidence changed during preparation')
        self.pins[key] = actual
        return raw, dict(path=key, sha256=actual)
    def read(self, path, expected=None):
        raw, ref = self.bytes(path, expected)
        def pairs(items):
            value = {}
            for key,item in items:
                need(key not in value, 'Duplicate JSON key'); value[key] = item
            return value
        doc = json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda _: need(False,'Nonfinite JSON'))
        return doc, ref
    def artifact(self, ref, base):
        p = Path(ref['path']); p = p if p.is_absolute() else base/p
        return self.read(p, ref['sha256'])
    def verify(self):
        for path, expected in self.pins.items():
            need(hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, 'Evidence changed before/after sealing')


def write(path, doc):
    raw = (json.dumps(doc, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)+'\n').encode()
    with os.fdopen(os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os,'O_NOFOLLOW',0),0o600),'wb') as stream:
        stream.write(raw)
    return dict(path=str(Path(path).resolve()), sha256=hashlib.sha256(raw).hexdigest())


def verify_kit(evidence, kit, expected, count, boot):
    need(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == boot, 'Current boot differs')
    manifest, ref = evidence.read(kit/'kit-manifest.json', expected); files = manifest.get('files',{})
    need(type(count) is int and type(files) is dict and len(files) == count, 'Exact installed file count required')
    actual = set()
    for path in kit.rglob('*'):
        if '__pycache__' in path.parts or path.suffix in ('.pyc','.pyo'): continue
        need(not path.is_symlink(), 'Kit symlink forbidden')
        if path.is_file(): actual.add(str(path.relative_to(kit)))
    need(actual == set(files)|{'kit-manifest.json'}, 'Missing/unregistered kit member')
    for name, expected_sha in files.items():
        rel = Path(name); need(not rel.is_absolute() and '..' not in rel.parts and str(rel)==name, 'Canonical kit member required')
        evidence.bytes(kit/rel, expected_sha)
    return manifest, ref


def load_live(kit):
    sys.dont_write_bytecode = True; sys.path.insert(0,str(kit)); sys.path.insert(0,str(kit/'runtime'))
    live = importlib.import_module('singularitydog_hw.policy_live_profile')
    need(Path(live.__file__).resolve() == (kit/'runtime/singularitydog_hw/policy_live_profile.py').resolve(), 'Unexpected loader')
    for name,module in list(sys.modules.items()):
        path = getattr(module,'__file__',None)
        if (name=='singularitydog_hw' or name.startswith('singularitydog_hw.')) and path:
            need(Path(path).resolve().is_relative_to((kit/'runtime').resolve()), 'Mixed runtime imports')
    need(live.SUPPORTED_POLICY_MIX_STEP_10PCT == MODE, 'Installed loader lacks exact mix-step contract')
    return live


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('kit','kit-manifest-sha256','expected-boot','power-epoch','output'):
        parser.add_argument('--'+key, required=True)
    parser.add_argument('--kit-file-count', required=True, type=int)
    for name in (*REF_NAMES,'operator-receipt','replay-source','replay-kit-manifest'):
        name = name.replace('_','-'); parser.add_argument('--'+name,required=True); parser.add_argument('--'+name+'-sha256',required=True)
    parser.add_argument('--finalize',action='store_true')
    for key in ('reviewer','reviewed-at','review-rationale'): parser.add_argument('--'+key)
    args = parser.parse_args(argv); evidence = Evidence(); kit = Path(args.kit); out = Path(args.output)
    need(kit.is_absolute() and out.is_absolute() and not out.exists() and str(out)==str(out.resolve(strict=False)) and
         out.parent.is_dir() and out.parent.stat().st_uid==os.getuid() and
         all(not parent.is_symlink() and not (parent/'.git').exists() for parent in (out,*out.parents)),
         'Fresh private canonical output outside Git required')
    need(str(uuid.UUID(args.expected_boot)) == args.expected_boot and args.power_epoch.strip(), 'Exact canonical boot/current power label required')
    manifest, _ = verify_kit(evidence,kit,args.kit_manifest_sha256,args.kit_file_count,args.expected_boot)
    live = load_live(kit); documents,refs = {},{}
    for name in (*REF_NAMES,'operator_receipt','replay_source','replay_kit_manifest'):
        path, expected = getattr(args,name),getattr(args,name+'_sha256')
        if name == 'replay_source': _,refs[name] = evidence.bytes(path,expected)
        else: documents[name],refs[name] = evidence.read(path,expected)
    prior = documents['prior_supported_profile']; parent = Path(refs['prior_supported_profile']['path']).parent
    need(prior['boot_id']==args.expected_boot and prior['motor_power_epoch']==args.power_epoch, 'Predecessor current session differs')
    for name,ref in prior['artifacts'].items():
        if name not in ('prior_supported_profile','prior_supported_report','prior_supported_observation'):
            documents[name], resolved = evidence.artifact(ref,parent)
            if name != 'pipeline_diagnostic': refs[name] = resolved
    # Current501 is supplied independently, never replaced by the historical diagnostic.
    documents['pipeline_diagnostic'],refs['pipeline_diagnostic'] = evidence.read(args.pipeline_diagnostic,args.pipeline_diagnostic_sha256)
    for source in documents['hardware_review'].get('source_captures',[]): evidence.artifact(source,parent)
    from tools.analyze_policy_target_mixture import analyze
    documents['recomputed_analysis'] = analyze(documents['prior_supported_report'],documents['saved_policy_target_sequence'],
        report_sha256=refs['prior_supported_report']['sha256'],targets_sha256=refs['saved_policy_target_sequence']['sha256'],
        physical_clearance_deg=7,max_displacement_deg=6)
    new_sources = live.cadence_source_hashes()
    p,h,o,clearance,source,settings = build_documents(live,documents,refs,new_sources,assembly_id=out.name,
        finalize=args.finalize,reviewer=args.reviewer,reviewed_at=args.reviewed_at,rationale=args.review_rationale)
    for name in p['artifacts']:
        if name in refs and name not in ('hardware_review','operator_acceptance'):
            p['artifacts'][name] = copy.deepcopy(refs[name])
    # Extra references are needed before the loader can validate artifact structure.
    for name in ('mix_step_clearance','mix_step_source_review'):
        p['artifacts'][name] = dict(path=str(out/(name+'.json')),sha256='0'*64)
    settings = live.reviewed_settings_sha256(p); h['reviewed_settings_sha256']=o['reviewed_settings_sha256']=settings
    live._structure(p); live._settings(p); live._axes(copy.deepcopy(p),documents['calibration'])
    timing = live._timing(documents['pipeline_diagnostic'],p)
    verify_kit(evidence,kit,args.kit_manifest_sha256,args.kit_file_count,args.expected_boot); evidence.verify()
    out.mkdir(mode=0o700)
    p['artifacts']['mix_step_clearance'] = write(out/'mix_step_clearance.json',clearance)
    p['artifacts']['mix_step_source_review'] = write(out/'mix_step_source_review.json',source)
    accept = h['supported_mix_step_acceptance']
    accept['clearance_sha256'] = p['artifacts']['mix_step_clearance']['sha256']
    accept['source_review_sha256'] = p['artifacts']['mix_step_source_review']['sha256']
    h['source_captures'].extend(copy.deepcopy([refs[key] for key in REF_NAMES]+[refs['operator_receipt']]))
    o['artifact_sha256'] = {key:p['artifacts'][key]['sha256'] for key in live.artifact_names(p) if key not in ('hardware_review','operator_acceptance')}
    p['artifacts']['operator_acceptance'] = write(out/'operator-acceptance.json',o)
    h['artifact_sha256'] = {key:p['artifacts'][key]['sha256'] for key in live.artifact_names(p) if key!='hardware_review'}
    p['artifacts']['hardware_review'] = write(out/'hardware-review.json',h)
    profile_path = out/'profile.json'
    try:
        profile = write(profile_path,p); loaded = live.load_profile(profile_path,require_approved=args.finalize)
        need(loaded['output_allowed'] is args.finalize, 'Final loader approval state differs')
        verify_kit(evidence,kit,args.kit_manifest_sha256,args.kit_file_count,args.expected_boot); evidence.verify()
        result = dict(schema='singularitydog.supported-mix-step-preparation.v1',
            status='APPROVED_ROOT_FILE_REVIEW' if args.finalize else 'UNAPPROVED_DRAFT',profile=profile,
            reviewed_settings_sha256=settings,kit_manifest_sha256=args.kit_manifest_sha256,
            source_input_sha256={key:ref['sha256'] for key,ref in refs.items()},timing=timing,
            named_reviews_created=args.finalize,approved_load_passed=args.finalize,
            physical_statements_generated=False,hardware_accessed=False,model_executed=False,
            box_removal_allowed=False,standing_allowed=False,walking_allowed=False,output_allowed=loaded['output_allowed'])
        write(out/'preparation.json',result)
    except BaseException:
        if profile_path.is_file(): profile_path.unlink()
        raise
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == '__main__':
    try: raise SystemExit(main())
    except Exception as error:
        print(json.dumps(dict(status='FILE_ONLY_MIX_STEP_PREPARATION_ABORTED',error=type(error).__name__+': '+str(error),
                              hardware_accessed=False,output_allowed=False),ensure_ascii=False)); raise SystemExit(2)
