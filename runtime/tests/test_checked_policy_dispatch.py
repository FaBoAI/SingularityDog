"""Synthetic admission and actual CPU kernel fixtures; never robot evidence."""
from array import array
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_checked_dispatch as checked
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_model as original
from singularitydog_hw import policy_observer as observer
from singularitydog_hw.policy_shadow import CAN_ORDER
from test_policy_observer import calibration, mount, snapshot


def scope_fixture():
    data = live.template(schema=live.SCHEMA_V3)
    data.update(native_checked_policy_dispatch=True, native_target_fk_cache=True,
        model_backend=live.SCALAR_BACKEND, native_phase_pair=False, request_gap_us=900,
        request_window=3, duration_s=2, diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
        local_characterization=live.LOCAL_RELATIVE_SUPPORTED, watchdog_review_policy=live.COMMAND_LOSS_ONLY_SUPPORTED,
        voltage_overlap=True, voltage_pipeline=True, prepare_voltage_before_feedback_publication=True,
        policy_weight=.005, h_hypothesis=0, command=[0.,0.,0.], period_ms=20, hard_cycle_ms=20,
        max_sample_age_ms=20,max_sample_gap_ms=21,max_consecutive_20ms_misses=0,
        voltage_min_v=35,voltage_max_v=42,startup_damping_duration_s=.08,
        post_reply_deadline_policy=dict(mode='bounded_post_reply_v1',max_lateness_ms=1.,
            max_consecutive_misses=1,rolling_window_cycles=100,max_misses_per_window=1))
    caps=dict(kp=3.,kd=.15,max_displacement_from_start_rad=math.radians(1),max_estimated_pd_torque_nm=.1,
        max_measured_torque_nm=1.,max_command_velocity_rad_s=math.radians(1),
        max_command_acceleration_rad_s2=math.radians(5),max_tracking_error_rad=math.radians(2),
        max_temperature_c=45.,max_measured_velocity_rad_s=.35)
    for axis in data['axes'].values():
        axis.update(caps,physical_lower_rad=-.01,physical_upper_rad=.01)
    data['start_pose_bounds']={mid:[-.001,.001] for mid in live.IDS}
    data['artifacts']={name:{'path':'/SYNTHETIC/'+name,'sha256':'1'*64} for name in live.artifact_names(data)}
    data['cadence_source_sha256']=live.cadence_source_hashes(data)
    return data


class AdmissionBoundaryTests(unittest.TestCase):
    def test_default_false_retains_baseline_and_invalid_flags_reject(self):
        for profile in ({}, {'native_checked_policy_dispatch':False}):
            self.assertFalse(checked.selected(profile))
        for value in (1,0,None,'true'):
            with self.assertRaises(live.ProfileError):checked.selected({'native_checked_policy_dispatch':value})
        with self.assertRaises(live.ProfileError):checked.selected({'schema':live.SCHEMA_V2,'native_checked_policy_dispatch':True})

    def test_exact_ordinary_two_ten_scope_rejects_other_modes_gaps_and_caps(self):
        data=scope_fixture();checked.scope(data)
        after=copy.deepcopy(data);after.update(duration_s=10,diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S)
        checked.scope(after)
        for key,value in (('duration_s',20),('duration_s',30),('request_gap_us',890),('request_window',2),
            ('native_phase_pair',True),('hard_cycle_ms',21),('policy_weight',.006),
            ('preauthorized_boxed_sequence',True),('post_reply_deadline_policy',{})):
            with self.subTest(key=key),self.assertRaises(live.ProfileError):checked.scope({**data,key:value})
        changed=copy.deepcopy(data);changed['axes']['6']['max_measured_velocity_rad_s']=.351
        with self.assertRaises(live.ProfileError):checked.scope(changed)

    def test_missing_full_loader_and_both_flag_mutation_directions_reject(self):
        data=scope_fixture();data['output_allowed']=True
        with self.assertRaisesRegex(live.ProfileError,'immutable'):checked.active_settings(data)
        data['_checked_model_plan']={'schema':'SYNTHETIC only'}
        data['_checked_model_token']=checked.TOKEN;data['_checked_model_binding']=checked.binding(data)
        self.assertEqual(checked.active_settings(data),data['artifacts']['checked_model_manifest'])
        data['native_checked_policy_dispatch']=False
        with self.assertRaisesRegex(live.ProfileError,'deselect'):checked.active_settings(data)
        fresh=scope_fixture();fresh['native_checked_policy_dispatch']=False
        fresh['output_allowed']=True;fresh['native_checked_policy_dispatch']=True
        with self.assertRaisesRegex(live.ProfileError,'immutable'):checked.active_settings(fresh)

    def test_manifest_current_artifact_and_boot_power_mutations_reject(self):
        data=scope_fixture();data['output_allowed']=True;data['_checked_model_plan']={'schema':'SYNTHETIC only'}
        data['_checked_model_token']=checked.TOKEN;data['_checked_model_binding']=checked.binding(data)
        for key in ('boot_id','motor_power_epoch'):
            with self.subTest(key=key),self.assertRaises(live.ProfileError):checked.active_settings({**data,key:'changed'})
        changed=copy.deepcopy(data);changed['_checked_model_token']=checked.TOKEN
        changed['artifacts']['local_reference_capture']['sha256']='2'*64
        with self.assertRaises(live.ProfileError):checked.active_settings(changed)

    def test_new_source_schema_never_aliases_old_fk_or_grants(self):
        data=scope_fixture();proof={'manifest':data['artifacts']['checked_model_manifest'],'variant':'checked_r11'}
        data['_checked_model_plan']=proof
        old={'schema':'SYNTHETIC original FK'};new=checked.provenance(old,proof)
        with patch.object(live,'_model_fk_provenance_matches',return_value=True) as validator:
            self.assertTrue(live._model_provenance_matches(new,data));validator.assert_called_once()
            self.assertFalse(live._model_provenance_matches(old,data))
            changed=copy.deepcopy(new);changed['output_allowed']=True
            self.assertFalse(live._model_provenance_matches(changed,data))
            changed=copy.deepcopy(new);changed['checked_model_file_plan']['variant']='wrong'
            self.assertFalse(live._model_provenance_matches(changed,data))

    def test_regular_source_and_declared_pin_reject_changes_and_symlinks(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder).resolve()/'raw';path.write_bytes(b'original');digest=hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(checked._read({'path':str(path),'sha256':digest},{},binary=True),b'original')
            path.write_bytes(b'changed')
            with self.assertRaises(live.ProfileError):checked._read({'path':str(path),'sha256':digest},{},binary=True)
            link=path.with_name('link');link.symlink_to(path)
            with self.assertRaises(live.ProfileError):checked._read({'path':str(link),'sha256':digest},{},binary=True)


# The actual fixture library is externally pinned and recorded, never rebuilt
# by these tests. Mac and ARM use their own actual build-record/library bytes.
import torch
record_path=Path(os.environ['R26_CHECKED_BUILD_RECORD'])
record_raw=record_path.read_bytes()
if hashlib.sha256(record_raw).hexdigest()!=os.environ['R26_CHECKED_BUILD_SHA256']:
    raise ValueError('Explicit actual tested C++ build-record SHA required')
record=json.loads(record_raw);library=record_path.parent/'live_checked_dispatch.so'
if (hashlib.sha256(library.read_bytes()).hexdigest()!=record['library_sha256'] or
        torch.__version__!=record['torch_version'] or
        bool(torch._C._GLIBCXX_USE_CXX11_ABI)!=record['cxx11_abi'] or
        hashlib.sha256((Path(checked.__file__).absolute().parents[1]/
            'experiments/private_checked_policy_dispatch/checked_dispatch.cpp').read_bytes()).hexdigest()!=record['source_sha256']):
    raise ValueError('Actual fixture C++/Torch build pin differs')
torch.ops.load_library(str(library))
from experiments.private_checked_policy_dispatch.adapter import wrap_policy
from experiments.private_checked_policy_dispatch.cpu_kernel import compare_named_state


class StatefulFixture(torch.nn.Module):
    def __init__(self):
        super().__init__();self.mode=0
        self.register_buffer('last_actor_output',torch.zeros(1,12))
        self.register_buffer('last_observation',torch.zeros(1,74));self.register_buffer('count',torch.zeros(1))
    @torch.jit.export
    def reset(self,ids:torch.Tensor):
        self.count.zero_();self.last_actor_output.zero_();self.last_observation.zero_()
    def forward(self,gyro:torch.Tensor,gravity:torch.Tensor,command:torch.Tensor,q:torch.Tensor,
                dq:torch.Tensor,h:torch.Tensor)->torch.Tensor:
        self.count.add_(1.);self.last_actor_output.zero_();self.last_observation.zero_()
        target=torch.tensor([[-0.,.4,-.8]*4],dtype=torch.float32)
        if self.mode==1:target[0,0]=float('nan')
        if self.mode==2:self.last_actor_output[0,11]=float('inf')
        if self.mode==3:self.last_observation[0,73]=float('nan')
        if self.mode==4:target[0,2]=0.
        if self.mode==5:raise RuntimeError('ORIGINAL_POLICY_FAILURE')
        return target


class RuntimeParityTests(unittest.TestCase):
    def model(self,policy,wrapper):
        model=object.__new__(original.LivePolicyModel);model.policy=policy;model.torch=torch
        model._startup_stage='ready';model.last_tick_ns=0;model.last_imu_ns=0;model.calls=0
        model.accel_input_hypothesis=None;model._checked_dispatch_wrapper=wrapper
        model.input_buffers=tuple(array('f',[0.]*n) for n in (3,3,3,12,12,12))
        model.tensors=tuple(torch.frombuffer(b,dtype=torch.float32).reshape(1,len(b)) for b in model.input_buffers)
        model.validate_inputs=lambda sample,imu,now:([0.]*3,[0.,0.,-1.],[0.]*3,[-0.,.4,-.8]*4,[0.]*12,[0.]*12)
        return model

    def test_actual_live_branch_preserves_bits_failure_states_and_successful_commit(self):
        one,two=(torch.jit.script(StatefulFixture()) for _ in range(2));wrapper=wrap_policy(two)
        baseline,selected=self.model(one,None),self.model(wrapper.inner,wrapper)
        for index in range(11):
            stamp=2+index*20
            a=baseline({}, {'read_started_monotonic_ns':stamp-1},stamp)
            b=selected({}, {'read_started_monotonic_ns':stamp-1},stamp)
            self.assertEqual(struct.pack('>12d',*a),struct.pack('>12d',*b));compare_named_state(torch,one,two)
        for mode in (1,2,3,4,5):
            one.mode=mode;two.mode=mode;errors=[]
            for model in (baseline,selected):
                try:model({}, {'read_started_monotonic_ns':999},1000)
                except Exception as error:errors.append((type(error),str(error)))
            self.assertEqual(len(errors),2)
            if mode!=5:self.assertEqual(*errors)
            else:self.assertTrue(all('ORIGINAL_POLICY_FAILURE' in value for _,value in errors))
            self.assertEqual((baseline.last_imu_ns,baseline.last_tick_ns,baseline.calls),
                             (selected.last_imu_ns,selected.last_tick_ns,selected.calls))
            compare_named_state(torch,one,two)

    def test_original_duplicate_clock_command_and_validation_error_precedence(self):
        policy=torch.jit.script(StatefulFixture());wrapper=wrap_policy(policy)
        model=self.model(wrapper.inner,wrapper)
        model({}, {'read_started_monotonic_ns':1},2)
        with self.assertRaisesRegex(ValueError,'reused'):model({}, {'read_started_monotonic_ns':3},2)
        with self.assertRaisesRegex(ValueError,'bounded forward'):model({}, {'read_started_monotonic_ns':3},4,command_override=(.051,0.,0.))
        model.accel_input_hypothesis=object()
        with self.assertRaisesRegex(ValueError,'boxed-only'):model({}, {'read_started_monotonic_ns':3},4,command_override=(0.,0.,0.))
        model.validate_inputs=lambda *a:(_ for _ in ()).throw(ValueError('ORIGINAL_INPUT_GUARD'))
        with self.assertRaisesRegex(ValueError,'ORIGINAL_INPUT_GUARD'):model({}, {'read_started_monotonic_ns':3},4)
        self.assertEqual((model.last_imu_ns,model.last_tick_ns,model.calls),(1,2,1))

    def test_normal_observer_keeps_owned_full_records_inputs_reset_and_state(self):
        policies=[torch.jit.script(StatefulFixture()) for _ in range(2)];wrapper=wrap_policy(policies[1])
        runs=[]
        for policy,dispatch in ((policies[0],None),(wrapper.inner,wrapper)):
            run=observer.StatefulPolicyObserver(policy,calibration(),imu_mount_candidate=mount(),
                h_hypothesis=0,command=[0.,0.,0.],max_ticks=3,max_age_ns=10_000_000,
                max_spread_ns=5_000_000,torch_module=torch,reuse_input_buffers=True,
                checked_dispatch_wrapper=dispatch)
            run.reset_run(1_000_000_000,warmup_completed=True);runs.append(run)
        old=[]
        for index in range(3):
            source=snapshot(1_000_000_000+index*observer.DT_NS);before=copy.deepcopy(source)
            results=[run.consume(source) for run in runs]
            self.assertEqual(*results);self.assertEqual(source,before);old.append(copy.deepcopy(results[1]))
            compare_named_state(torch,policies[0],policies[1])
        policies[1].last_actor_output.fill_(99.)
        self.assertEqual(old[0]['actor_residual12'],[0.]*12)
        self.assertEqual(runs[0].finish(),runs[1].finish())

    def test_cached_foreign_component_module_and_foreign_function_reject(self):
        name='experiments.private_checked_policy_dispatch.adapter'
        foreign=types.ModuleType(name);foreign.__file__='/SYNTHETIC/foreign-adapter.py'
        with patch.dict(sys.modules,{name:foreign}),self.assertRaisesRegex(live.ProfileError,'import origin'):
            checked._component_functions()
        from experiments.private_checked_policy_dispatch import adapter
        with patch.object(adapter,'checked_call',lambda *a:()),self.assertRaisesRegex(live.ProfileError,'function origin'):
            checked._component_functions()


class ManifestBoundaryTests(unittest.TestCase):
    """Explicit mocked file documents are guard tests, never ARM qualification."""
    def fixture(self):
        profile=scope_fixture();root=Path(checked.__file__).absolute().parents[1]
        refs={name:{'path':'/SYNTHETIC/'+name,'sha256':'1'*64} for name in checked.REFS}
        sources={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in checked.source_paths()}
        manifest=dict(schema=checked.SCHEMA,status='FILE_ONLY_MODEL_BINDINGS_NO_OUTPUT_APPROVAL',
            variant='checked_r11',references=refs,source_sha256=sources,
            original_fk_artifacts={key:profile['artifacts'][key] for key in
                ('target_fk_manifest','scalar_step_manifest','model_manifest')},
            physical_future_observations=None,**dict.fromkeys(checked.FALSE_FLAGS,False))
        report=dict(schema='PRIVATE.live-checked-dispatch-four-variant-plan.v1',
            status='PASS_LOCAL_REAL_ACTOR_FOUR_VARIANT_501',torch_version='SYNTHETIC',
            all_501_actor12_observation74_can_target_and_named_state_bits_exact=True,
            current_target_saved_bit_match_counts={'actor':501,'observation':501,'target':501},
            source_and_input_pins_unchanged=True,input_snapshot_mutation=False,warmup_original_ten_ten_reset=True,
            original_methods_unchanged_except_r11_explicit_target_operator=True,
            local_reference_model_sha256='1'*64,checked_serialized_sha256=['2'*64,'1'*64],
            source_manifest_sha256='3'*64,
            balanced_four_variant_blocks=[dict(block=i+1,reverse=r,calls_per_variant=501)
                for i,r in enumerate((False,True,True,False))],
            raw_times={key:{'wall_ns':[1]*2004,'thread_cpu_ns':[1]*2004} for key in
                ('original','checked_original','r11','checked_r11')},**dict.fromkeys(checked.FALSE_FLAGS,False))
        build=dict(schema='PRIVATE.live-checked-dispatch-build.v1',
            status='CPU_LIBRARY_BUILT_NO_RUNTIME_QUALIFICATION',source_sha256=sources[
                'experiments/private_checked_policy_dispatch/checked_dispatch.cpp'],library_sha256='1'*64,
            compiler_returncode=0,machine='aarch64',cxx11_abi=True,torch_version='SYNTHETIC',
            flags=['-O3','-std=c++20','-ffp-contract=off','-fno-fast-math'],
            hardware_opened=False,output_allowed=False,approved_for_runtime=False)
        before=dict(uid=1,euid=1,gid=1,egid=1,nice=-10,cpus=[0,1,2,3,4],switch_s=.005,timer_slack_ns=50000)
        scopes=dict(guard=dict(status='VERIFIED',hardware_opened=False,before=before,
            during=dict(cpus=[4],timer_slack_ns=1000,switch_s=.0001)),
            inner=dict(restored=True,failures=[],actual_after={key:before[key] for key in
                ('switch_s','timer_slack_ns','cpus')}),
            outer=dict(status='RESTORED',restored=True,cpu_performance_restored=True,restore_errors=[],
                caught_signal=None,child_exit_code=0,output_permission_granted_by_scope=False))
        receipt=dict(status='ARM_FILE_ONLY_MODEL_COMPONENT_BUILT_UNIT15_REPLAY501_PASS',
            actual_arm_original_model_sha256='1'*64,source804_manifest_sha256='3'*64,
            report_sha256='1'*64,new_build_record_sha256='1'*64,
            source_and_input_pins_unchanged=True,actual_guard_and_cpu_c7_emc_restored=True,
            whole_cycle_or_type1_qualification=False,physical_future_observations=None,
            steps=[dict(exit_code=0),dict(exit_code=0),dict(exit_code=0,verified_scopes=scopes),
                dict(exit_code=0,verified_scopes=copy.deepcopy(scopes))],
            hardware_opened=False,output_allowed=False,approved_for_runtime=False,live_type1_qualified=False)
        docs=dict(checked_model=b'SYNTHETIC',original_model=b'SYNTHETIC',checked_library=b'SYNTHETIC',
                  r11_library=b'SYNTHETIC',component_report=report,checked_build=build,
                  r11_build=dict(library_sha256='1'*64),component_receipt=receipt)
        return profile,manifest,docs

    def verify(self,profile,manifest,docs,model_sha='1'*64):
        actual_read=checked._read
        def read(ref,pins,**kwargs):
            if ref['path'].startswith('/SYNTHETIC/'):
                return docs[Path(ref['path']).name]
            return actual_read(ref,pins,**kwargs)
        with patch.object(checked,'_read',side_effect=read),patch(
                'singularitydog_hw.policy_active_fk.plan',return_value=dict(model_sha256=model_sha)):
            return checked._document_plan(profile,manifest,profile['artifacts']['checked_model_manifest'],{})

    def test_schema_false_grants_and_source_reference_substitution_reject(self):
        profile,manifest,docs=self.fixture();proof=self.verify(profile,manifest,docs)
        self.assertTrue(all(proof[key] is False for key in checked.FALSE_FLAGS))
        for key,value in (('schema','old-schema'),('output_allowed',True),('physical_future_observations',True),
                          ('variant','original')):
            with self.subTest(key=key),self.assertRaises(live.ProfileError):
                self.verify(profile,{**manifest,key:value},docs)
        changed=copy.deepcopy(manifest);changed['source_sha256'][checked.source_paths()[0]]='2'*64
        with self.assertRaisesRegex(live.ProfileError,'cadence/source'):self.verify(profile,changed,docs)
        changed=copy.deepcopy(manifest);changed['original_fk_artifacts']['model_manifest']['sha256']='2'*64
        with self.assertRaisesRegex(live.ProfileError,'FK artifact'):self.verify(profile,changed,docs)
        with self.assertRaisesRegex(live.ProfileError,'verified FK model'):self.verify(profile,manifest,docs,'2'*64)

    def test_incomplete_changed_reference_parity_raw_or_warmup_reject(self):
        profile,manifest,docs=self.fixture()
        for key,value in (('current_target_saved_bit_match_counts',{'actor':0,'observation':1,'target':0}),
            ('source_and_input_pins_unchanged',False),('warmup_original_ten_ten_reset',False),
            ('checked_serialized_sha256',['1'*64,'2'*64]),('balanced_four_variant_blocks',[])):
            changed=copy.deepcopy(docs);changed['component_report'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.verify(profile,manifest,changed)
        changed=copy.deepcopy(docs);changed['component_report']['raw_times']['original']['wall_ns'].pop()
        with self.assertRaisesRegex(live.ProfileError,'raw timing'):self.verify(profile,manifest,changed)

    def test_build_library_receipt_binding_and_real_restoration_reject(self):
        profile,manifest,docs=self.fixture()
        for key,value in (('machine','arm64'),('flags',['-O3','-std=c++17']),('library_sha256','2'*64),
                          ('cxx11_abi',None)):
            changed=copy.deepcopy(docs);changed['checked_build'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.verify(profile,manifest,changed)
        for key,value in (('report_sha256','2'*64),('actual_guard_and_cpu_c7_emc_restored',False),
                          ('new_build_record_sha256','2'*64),('whole_cycle_or_type1_qualification',True)):
            changed=copy.deepcopy(docs);changed['component_receipt'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):self.verify(profile,manifest,changed)
        changed=copy.deepcopy(docs);changed['component_receipt']['steps'][2]['verified_scopes']['inner']['restored']=False
        with self.assertRaisesRegex(live.ProfileError,'readback/restoration'):self.verify(profile,manifest,changed)


if __name__=='__main__':unittest.main()
