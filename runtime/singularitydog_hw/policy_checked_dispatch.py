"""Explicit checked model bindings; CPU evidence never grants motor output."""
import copy
import hashlib
import io
import importlib
import json
from pathlib import Path

SCHEMA = 'singularitydog.checked-live-model-file-only.v1'
SOURCE_SCHEMA = 'singularitydog.checked-live-model-source.v1'
TOKEN = object()
FALSE_FLAGS = ('hardware_opened', 'output_allowed', 'approved_for_runtime',
               'active_controller_qualification', 'timing_admission_eligible', 'live_type1_qualified')
REFS = ('checked_model', 'original_model', 'checked_library', 'checked_build',
        'r11_library', 'r11_build', 'component_report', 'component_receipt')
_OWNED_CALL = None


def source_paths():
    return ('singularitydog_hw/policy_checked_dispatch.py',
            'singularitydog_hw/policy_output_model.py', 'singularitydog_hw/policy_observer.py',
            'singularitydog_hw/policy_observer_replay.py',
            'experiments/private_checked_policy_dispatch/adapter.py',
            'experiments/private_checked_policy_dispatch/cpu_kernel.py',
            'experiments/private_checked_policy_dispatch/checked_dispatch.cpp')


def _profiles():
    from . import policy_live_profile
    return policy_live_profile


def selected(profile):
    value = profile.get('native_checked_policy_dispatch', False)
    p = _profiles()
    p._need(type(value) is bool and (not value or profile.get('schema') == p.SCHEMA_V3),
            'Checked model selection requires an explicit V3 boolean')
    return value


def scope(profile):
    if not selected(profile):
        return
    p = _profiles()
    p._need(profile.get('scope') == 'supported_characterization_only' and
        profile.get('native_target_fk_cache') is True and profile.get('model_backend') == p.SCALAR_BACKEND and
        profile.get('native_phase_pair', False) is False and
        profile.get('request_gap_us') == 900 and profile.get('request_window') == 3 and
        profile.get('period_ms') == 20 and profile.get('hard_cycle_ms') == 20 and
        profile.get('duration_s') in (2, 10) and
        profile.get('diagnostic_timing_acceptance') ==
            (p.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER if profile['duration_s'] == 2
             else p.SUPPORTED_POLICY_PROBE_10S_AFTER_2S),
        'Checked model selection is ordinary900/window3/original20ms boxed2/10 only')
    p._need(profile.get('preauthorized_boxed_sequence',False) is False and
            profile.get('preauthorized_native_boxed_sequence',False) is False,
            'Checked model cannot select a legacy/native preauthorization scope')
    # This copied boolean invokes only the existing pure numerical scope check.
    # It supplies no authorization token, review or future physical observation.
    p._preauthorized_ordinary_boxed_sequence_scope(
        dict(profile,preauthorized_ordinary_boxed_sequence=True))


def _read(ref, pins, *, binary=False):
    p = _profiles()
    p._need(type(ref) is dict and set(ref) == {'path', 'sha256'}, 'Exact checked model reference required')
    path = Path(p._text(ref['path'], 'checked model reference path'))
    p._need(path.is_absolute() and path.is_file() and
            not any(item.is_symlink() for item in (path, *path.parents)),
            'Absolute regular nonsymlink checked model input required')
    digest = p._hash(ref['sha256'], 'checked model input')
    raw = path.read_bytes()
    p._need(len(raw) <= 32*1024*1024 and hashlib.sha256(raw).hexdigest() == digest,
            'Checked model input SHA256/extent mismatch')
    p._need(str(path) not in pins or pins[str(path)] == digest, 'Conflicting checked model source pin')
    pins[str(path)] = digest
    return raw if binary else p.shadow._json(raw.decode('utf-8'))


def verify_files(proof):
    for path, digest in proof['input_sha256'].items():
        _read({'path': path, 'sha256': digest}, {}, binary=True)


def _component_functions():
    """Bind actual imported functions to this exact kit before model setup."""
    p=_profiles();root=Path(__file__).absolute().parents[1];result={}
    for leaf,names in (('adapter',('checked_call',)),
                       ('cpu_kernel',('compare_named_state','verify_methods'))):
        name='experiments.private_checked_policy_dispatch.'+leaf
        module=importlib.import_module(name)
        path=root/('experiments/private_checked_policy_dispatch/'+leaf+'.py')
        p._need(type(getattr(module,'__file__',None)) is str and
            Path(module.__file__).absolute()==path and path.is_file() and
            not any(item.is_symlink() for item in (path,*path.parents)),
            'Actual checked component import origin differs: '+name)
        for symbol in names:
            value=getattr(module,symbol,None)
            p._need(callable(value) and getattr(value,'__module__',None)==name and
                getattr(value,'__globals__',None) is module.__dict__ and
                Path(value.__code__.co_filename).absolute()==path,
                'Actual checked component function origin differs: '+symbol)
            result[symbol]=value
    return result


def plan(profile, manifest=None):
    """Read exact CPU artifacts and source; no Torch/native/device load."""
    p = _profiles()
    scope(profile)
    p._need(selected(profile), 'Explicit checked model selection required')
    pins = {}
    reference = profile['artifacts']['checked_model_manifest']
    actual = _read(reference, pins)
    p._need(manifest is None or manifest == actual, 'Checked model manifest document differs')
    return _document_plan(profile,actual,reference,pins)


def _document_plan(profile, manifest, reference, pins):
    """Validate proposed dependency bytes; caller must label unpublished plans."""
    p = _profiles()
    scope(profile)
    p._need(type(manifest) is dict and set(manifest) ==
        {'schema', 'status', 'variant', 'references', 'original_fk_artifacts', 'source_sha256',
         'physical_future_observations', *FALSE_FLAGS} and manifest['schema'] == SCHEMA and
        manifest['status'] == 'FILE_ONLY_MODEL_BINDINGS_NO_OUTPUT_APPROVAL' and
        manifest['variant'] == 'checked_r11' and all(manifest[key] is False for key in FALSE_FLAGS) and
        manifest['physical_future_observations'] is None,
        'New checked model file-only schema and false qualification flags required')
    refs = manifest['references']
    p._need(type(refs) is dict and set(refs) == set(REFS), 'Complete checked model dependency references required')
    documents = {key: _read(refs[key], pins, binary=key in
        ('checked_model', 'original_model', 'checked_library', 'r11_library')) for key in REFS}
    sources = manifest['source_sha256']
    root = Path(__file__).absolute().parents[1]
    p._need(type(sources) is dict and set(sources) == set(source_paths()), 'Complete checked model source pins required')
    for name, digest in sources.items():
        p._need(profile['cadence_source_sha256'].get(name) == digest, 'Checked model cadence/source binding differs')
        _read({'path': str(root/name), 'sha256': digest}, pins, binary=True)
    expected_fk = {key: profile['artifacts'][key] for key in
                   ('target_fk_manifest', 'scalar_step_manifest', 'model_manifest')}
    p._need(manifest['original_fk_artifacts'] == expected_fk, 'Checked model original FK artifact binding differs')
    report, build, r11build, receipt = (documents[key] for key in
        ('component_report', 'checked_build', 'r11_build', 'component_receipt'))
    p._need(report.get('schema') == 'PRIVATE.live-checked-dispatch-four-variant-plan.v1' and
        report.get('status') == 'PASS_LOCAL_REAL_ACTOR_FOUR_VARIANT_501' and
        report.get('all_501_actor12_observation74_can_target_and_named_state_bits_exact') is True and
        report.get('current_target_saved_bit_match_counts') == {'actor':501,'observation':501,'target':501} and
        report.get('source_and_input_pins_unchanged') is True and report.get('input_snapshot_mutation') is False and
        report.get('warmup_original_ten_ten_reset') is True and
        report.get('original_methods_unchanged_except_r11_explicit_target_operator') is True and
        report.get('local_reference_model_sha256') == refs['original_model']['sha256'] and
        type(report.get('checked_serialized_sha256')) is list and len(report['checked_serialized_sha256']) == 2 and
        report['checked_serialized_sha256'][1] == refs['checked_model']['sha256'] and
        all(report.get(key) is False for key in ('hardware_opened','output_allowed','approved_for_runtime',
             'active_controller_qualification','timing_admission_eligible','live_type1_qualified')),
        'Actual original-reference501 checked model parity proof required')
    p._need(report.get('balanced_four_variant_blocks') == [dict(block=index+1, reverse=reverse,
        calls_per_variant=501) for index, reverse in enumerate((False, True, True, False))] and
        set(report.get('raw_times', {})) == {'original','checked_original','r11','checked_r11'} and
        all(set(value) == {'wall_ns','thread_cpu_ns'} and all(type(rows) is list and len(rows) == 2004 and
             all(type(n) is int and n >= 0 for n in rows) for rows in value.values())
            for value in report['raw_times'].values()), 'All four original balanced raw timing arrays required')
    p._need(build.get('schema') == 'PRIVATE.live-checked-dispatch-build.v1' and
        build.get('status') == 'CPU_LIBRARY_BUILT_NO_RUNTIME_QUALIFICATION' and
        build.get('source_sha256') == sources['experiments/private_checked_policy_dispatch/checked_dispatch.cpp'] and
        build.get('library_sha256') == refs['checked_library']['sha256'] and
        build.get('compiler_returncode') == 0 and build.get('machine') == 'aarch64' and
        type(build.get('cxx11_abi')) is bool and
        build.get('flags') == ['-O3','-std=c++20','-ffp-contract=off','-fno-fast-math'] and
        build.get('torch_version') == report.get('torch_version') and
        all(build.get(key) is False for key in ('hardware_opened','output_allowed','approved_for_runtime')),
        'Actual ARM checked C++ source/build/library proof required')
    p._need(r11build.get('library_sha256') == refs['r11_library']['sha256'], 'R11 library/build pin differs')
    p._need(receipt.get('status') == 'ARM_FILE_ONLY_MODEL_COMPONENT_BUILT_UNIT15_REPLAY501_PASS' and
        receipt.get('actual_arm_original_model_sha256') == refs['original_model']['sha256'] and
        receipt.get('source804_manifest_sha256') == report.get('source_manifest_sha256') and
        receipt.get('report_sha256') == refs['component_report']['sha256'] and
        receipt.get('new_build_record_sha256') == refs['checked_build']['sha256'] and
        receipt.get('source_and_input_pins_unchanged') is True and
        receipt.get('actual_guard_and_cpu_c7_emc_restored') is True and
        receipt.get('whole_cycle_or_type1_qualification') is False and
        receipt.get('physical_future_observations') is None and
        all(receipt.get(key) is False for key in
            ('hardware_opened','output_allowed','approved_for_runtime','live_type1_qualified')),
        'Original ARM component receipt/reference binding required')
    steps=receipt.get('steps')
    p._need(type(steps) is list and len(steps)==4 and
        all(step.get('exit_code')==0 for step in steps), 'All original ARM component steps required')
    for step in steps[2:]:
        scopes=step.get('verified_scopes',{});guard=scopes.get('guard',{})
        before,during=guard.get('before',{}),guard.get('during',{})
        inner,outer=scopes.get('inner',{}),scopes.get('outer',{})
        p._need(guard.get('status')=='VERIFIED' and guard.get('hardware_opened') is False and
            before.get('uid')==before.get('euid') and type(before.get('uid')) is int and before['uid']>0 and
            before.get('gid')==before.get('egid') and type(before.get('gid')) is int and before['gid']>0 and
            before.get('nice')==-10 and before.get('cpus')==[0,1,2,3,4] and
            during.get('cpus')==[4] and during.get('timer_slack_ns')==1000 and
            type(during.get('switch_s')) in (int,float) and abs(during['switch_s']-.0001)<=1e-15 and
            inner.get('restored') is True and inner.get('failures')==[] and
            inner.get('actual_after')=={key:before[key] for key in ('switch_s','timer_slack_ns','cpus')} and
            outer.get('status')=='RESTORED' and outer.get('restored') is True and
            outer.get('cpu_performance_restored') is True and outer.get('restore_errors')==[] and
            outer.get('caught_signal') is None and outer.get('child_exit_code')==0 and
            outer.get('output_permission_granted_by_scope') is False,
            'Actual original ARM component control/readback/restoration proof required')
    from . import policy_active_fk
    original_plan = policy_active_fk.plan(profile)
    p._need(original_plan['model_sha256'] == refs['original_model']['sha256'],
            'Original verified FK model differs from checked component reference')
    return {'schema': 'singularitydog.checked-live-model-file-plan.v1',
        'manifest': copy.deepcopy(reference), 'variant': 'checked_r11',
        'references': copy.deepcopy(refs), 'source_sha256': copy.deepcopy(sources),
        'original_fk_plan': original_plan, 'input_sha256': pins,
        'physical_future_observations': None, **dict.fromkeys(FALSE_FLAGS, False)}


def binding(profile):
    p = _profiles()
    value = {'settings': p.reviewed_settings_sha256(profile), 'axes': profile['axes'],
             'artifacts': profile['artifacts'],
             'sources': profile['cadence_source_sha256'], 'proof': profile.get('_checked_model_plan'),
             'boot_id':profile['boot_id'],'motor_power_epoch':profile['motor_power_epoch'],
             'review':profile['review'],'blockers':profile['blockers'],
             'approved':profile['approved_for_supported_policy_output']}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def active_settings(profile):
    p = _profiles()
    if not selected(profile):
        p._need('_checked_model_token' not in profile,'Cannot deselect an admitted checked model')
        return None
    p._need(profile.get('output_allowed') is True and profile.get('_checked_model_token') is TOKEN and
            profile.get('_checked_model_binding') == binding(profile),
            'Checked model requires immutable complete current profile admission')
    return copy.deepcopy(profile['artifacts']['checked_model_manifest'])


def provenance(original, proof):
    return {'schema': SOURCE_SCHEMA, 'manifest_sha256': proof['manifest']['sha256'],
        'variant': proof['variant'], 'checked_model_file_plan': copy.deepcopy(proof),
        'original_fk_provenance': copy.deepcopy(original),
        'observer_route': 'normal_python_owned_telemetry_with_checked_can_target.v1',
        'model_artifact_grants_output': False, 'physical_future_observations': None,
        **dict.fromkeys(FALSE_FLAGS, False)}


def load(profile, original_policy, original_provenance, *, active):
    """Load a separately identified ScriptModule only after all file checks."""
    p = _profiles()
    p._need(type(active) is bool, 'Explicit active/diagnostic selection required')
    if active:
        active_settings(profile)
    proof = plan(profile)
    p._need(profile.get('_checked_model_plan') == proof, 'Checked model proof changed since complete loader')
    import torch
    functions=_component_functions()
    refs = proof['references']
    build = _read(refs['checked_build'], {})
    p._need(torch.__version__ == build['torch_version'] and
            bool(torch._C._GLIBCXX_USE_CXX11_ABI) == build['cxx11_abi'], 'Checked C++ build/load Torch ABI differs')
    for key in ('r11_library', 'checked_library'):
        torch.ops.load_library(refs[key]['path'])
    wrapper = torch.jit.load(io.BytesIO(_read(refs['checked_model'], {}, binary=True)), map_location='cpu').eval()
    functions['verify_methods'](original_policy, wrapper.inner)
    functions['compare_named_state'](torch, original_policy, wrapper.inner, independent=True)
    p._need(plan(profile) == proof, 'Checked model inputs/source changed while loading')
    global _OWNED_CALL
    _OWNED_CALL=functions['checked_call']
    return wrapper.inner, wrapper, provenance(original_provenance, proof)


def checked_call(wrapper, tensors):
    global _OWNED_CALL
    if _OWNED_CALL is None:
        _OWNED_CALL=_component_functions()['checked_call']
    return _OWNED_CALL(wrapper, tensors)
