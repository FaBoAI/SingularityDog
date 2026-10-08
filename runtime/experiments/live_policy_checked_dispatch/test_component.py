"""Pure CPU guard/state fixtures; no source/profile/hardware qualification."""
from array import array
from pathlib import Path
import io
import struct
import sys
import unittest
import hashlib
import json
import os

RUNTIME=Path(os.environ['LIVE_COMPONENT_RUNTIME'])
SOURCE_SHA=os.environ['LIVE_COMPONENT_ORIGINAL_SOURCE_SHA256']
sys.path.insert(0,str(RUNTIME))
import torch
HERE=Path(__file__).absolute().parent
BUILD=Path(os.environ['LIVE_COMPONENT_BUILD'])
build_raw=(BUILD/'build-record.json').read_bytes()
if 'LIVE_COMPONENT_BUILD' in os.environ:
    if hashlib.sha256(build_raw).hexdigest()!=os.environ.get('LIVE_COMPONENT_BUILD_SHA256'):
        raise ValueError('External portable fixture build-record SHA required')
record=json.loads(build_raw)
if torch.__version__!=record['torch_version'] or bool(torch._C._GLIBCXX_USE_CXX11_ABI)!=record['cxx11_abi']:
    raise ValueError('Actual tested library Torch build/load ABI mismatch')
lib=BUILD/'live_checked_dispatch.so'
if hashlib.sha256(lib.read_bytes()).hexdigest()!=record['library_sha256']:
    raise ValueError('Actual CPU fixture library SHA mismatch')
torch.ops.load_library(str(lib))
from adapter import (CheckedDispatchCpuKernel,CheckedLiveCallComponent,
    wrap_policy,checked_call,MARKERS)
from cpu_kernel import GuardedCpuPolicyKernel,compare_named_state
from singularitydog_hw import policy_output_model as original
from singularitydog_hw.policy_shadow import CAN_ORDER


class StatefulFixture(torch.nn.Module):
    def __init__(self):
        super().__init__();self.register_buffer('last_actor_output',torch.zeros(1,12))
        self.register_buffer('last_observation',torch.zeros(1,74));self.register_buffer('count',torch.zeros(1))
        self.register_buffer('resets',torch.zeros(1));self.mode=0
    @torch.jit.export
    def reset(self,ids:torch.Tensor):
        self.count.zero_();self.resets.add_(1.);self.last_actor_output.zero_();self.last_observation.zero_()
    def forward(self,gyro:torch.Tensor,gravity:torch.Tensor,command:torch.Tensor,
                q:torch.Tensor,dq:torch.Tensor,h:torch.Tensor)->torch.Tensor:
        self.count.add_(1.)
        self.last_actor_output.zero_();self.last_observation.zero_()
        target=torch.tensor([[-0.,.4,-.8]*4],dtype=torch.float32)
        if self.mode==1:target[0,0]=float('nan')
        if self.mode==2:self.last_actor_output[0,0]=float('inf')
        if self.mode==3:self.last_observation[0,73]=float('nan')
        if self.mode==4:target[0,2]=0.
        if self.mode==5:raise RuntimeError('ORIGINAL_MODEL_FAILURE_UNCHANGED')
        return target


def original_checks(target,actor,obs):
    values=original._cpu_float32_target_row(target,12,'live policy target',torch)
    original._finite_cpu_float32_row(actor,12,'actor output',torch)
    original._finite_cpu_float32_row(obs,74,'observation',torch)
    if not all(lo<=q<=hi for q,lo,hi in zip(values,original._TARGET_LOWER,original._TARGET_UPPER)):
        raise ValueError('Learned target outside model range')
    out=[0.]*12
    for can,value in zip(CAN_ORDER,values):out[can-1]=value
    return tuple(out)


def native_checks(target,actor,obs):
    try:return tuple(torch.ops.sd_live_checked_dispatch_private_r1.checked_targets(target,actor,obs))
    except RuntimeError as exc:
        for key,(kind,message) in MARKERS.items():
            if 'PRIVATE_LIVE_CHECKED_'+key+'_R1' in str(exc):raise kind(message) from exc
        raise


class CheckedOutputGuardTests(unittest.TestCase):
    def setUp(self):
        self.target=torch.tensor([[-0.,.4,-.8]*4],dtype=torch.float32)
        self.actor=torch.zeros((1,12));self.obs=torch.zeros((1,74))
    def same_error(self,target,actor,obs):
        caught=[]
        for fn in (original_checks,native_checks):
            try:fn(target,actor,obs)
            except Exception as exc:caught.append((type(exc),str(exc)))
        self.assertEqual(len(caught),2);self.assertEqual(*caught)
    def test_exact_can_order_signed_zero_and_owned_return(self):
        got=native_checks(self.target,self.actor,self.obs);want=original_checks(self.target,self.actor,self.obs)
        self.assertEqual(struct.pack('>12d',*got),struct.pack('>12d',*want))
        self.target.fill_(0.);self.assertEqual(struct.pack('>12d',*got),struct.pack('>12d',*want))
    def test_shape_dtype_layout_and_non_cpu_reject_before_access(self):
        invalid=(torch.zeros(12),torch.zeros(1,11),self.target.double(),self.target.to_sparse(),
                 torch.empty((1,12),device='meta'))
        for value in invalid:
            with self.subTest(target=value.layout,dtype=value.dtype,device=value.device.type):
                self.same_error(value,self.actor,self.obs)
        for actor in (torch.zeros(1,11),self.actor.double(),self.actor.to_sparse(),torch.empty(1,12,device='meta')):
            self.same_error(self.target,actor,self.obs)
        for obs in (torch.zeros(1,73),self.obs.double(),self.obs.to_sparse(),torch.empty(1,74,device='meta')):
            self.same_error(self.target,self.actor,obs)
    def test_noncontiguous_and_negative_views_match_without_mutation(self):
        for negative in (False,True):
            for which in (0,1,2):
                rows=[self.target.clone(),self.actor.clone(),self.obs.clone()];value=rows[which]
                source=torch.zeros((1,value.shape[1]*2));source[:,::2]=value
                value=source[:,::2]
                if negative:value=torch._neg_view(-value)
                rows[which]=value;before=[v.clone() for v in rows]
                self.assertEqual(struct.pack('>12d',*native_checks(*rows)),struct.pack('>12d',*original_checks(*rows)))
                for one,two in zip(rows,before):self.assertTrue(torch.equal(one,two))
    def test_nan_inf_last_element_and_range_error_exact(self):
        for slot,index in ((0,11),(1,11),(2,73)):
            for value in (float('nan'),float('inf'),float('-inf')):
                rows=[self.target.clone(),self.actor.clone(),self.obs.clone()];rows[slot][0,index]=value
                self.same_error(*rows)
        for slot,value in ((0,-.5001),(1,1.2001),(2,-.079)):
            target=self.target.clone();target[0,slot]=value;self.same_error(target,self.actor,self.obs)
    def test_original_error_precedence_all_invalid_and_actor_before_range(self):
        target=self.target.clone();target[0,11]=float('nan');actor=torch.zeros(1,11);obs=torch.zeros(1,73)
        self.same_error(target,actor,obs)
        target=self.target.clone();target[0,2]=0.;self.same_error(target,actor,obs)
        self.same_error(target,self.actor,obs)
    def test_exact_f32_range_boundaries_and_nextafter_outside(self):
        for lower in (True,False):
            bounds=original._TARGET_LOWER if lower else original._TARGET_UPPER
            target=torch.tensor([bounds],dtype=torch.float32)
            self.assertEqual(struct.pack('>12d',*native_checks(target,self.actor,self.obs)),
                             struct.pack('>12d',*original_checks(target,self.actor,self.obs)))
            for index in range(12):
                changed=target.clone();direction=torch.tensor(float('-inf') if lower else float('inf'))
                changed[0,index]=torch.nextafter(changed[0,index],direction);self.same_error(changed,self.actor,self.obs)


class DispatchStateTests(unittest.TestCase):
    def setUp(self):
        self.values=([0.,0.,0.],[0.,0.,-1.],[0.,0.,0.],[-.28,.4,-.8]*4,[0.]*12,[0.]*12)
        self.policy=torch.jit.script(StatefulFixture().eval());self.wrapper=wrap_policy(self.policy)
        self.kernel=CheckedDispatchCpuKernel(self.wrapper,torch,original_source_sha256=SOURCE_SHA)
    def ready(self):
        self.kernel.pre_pin_warmup();self.kernel.post_pin_prime();self.kernel.finish_startup()
    def test_warm_reset_retains_original_state_and_20_calls(self):
        self.kernel.pre_pin_warmup();self.assertEqual(self.policy.count.item(),10)
        ptrs=[t.data_ptr() for t in self.kernel.tensors];self.kernel.post_pin_prime()
        self.assertEqual(self.policy.count.item(),20);self.kernel.finish_startup()
        self.assertEqual(self.policy.count.item(),0);self.assertEqual(self.policy.resets.item(),1)
        self.kernel(self.values);self.assertEqual(self.policy.count.item(),1);self.assertEqual(self.kernel.calls,1)
        self.kernel.reset();self.assertEqual(self.policy.count.item(),0);self.assertEqual(self.kernel.calls,0)
        self.assertEqual(ptrs,[t.data_ptr() for t in self.kernel.tensors])
    def test_serialization_reset_and_independent_named_state(self):
        data=io.BytesIO();torch.jit.save(self.wrapper,data)
        cloned=torch.jit.load(io.BytesIO(data.getvalue())).eval()
        compare_named_state(torch,self.policy,cloned.inner,independent=True)
        other=CheckedDispatchCpuKernel(cloned,torch,original_source_sha256=SOURCE_SHA)
        for k in (self.kernel,other):k.pre_pin_warmup();k.post_pin_prime();k.finish_startup()
        for cycle in range(17):
            self.assertEqual(self.kernel(self.values),other(self.values));compare_named_state(torch,self.policy,cloned.inner)
        for k in (self.kernel,other):k.reset()
        compare_named_state(torch,self.policy,cloned.inner)
    def test_native_guard_failure_does_not_commit_calls_original_model_failure_primary(self):
        self.ready()
        baseline=torch.jit.script(StatefulFixture().eval());old=GuardedCpuPolicyKernel(baseline,torch,original_source_sha256=SOURCE_SHA)
        old.pre_pin_warmup();old.post_pin_prime();old.finish_startup()
        for mode in (1,2,3,4):
            self.policy.mode=mode;baseline.mode=mode
            with self.assertRaises((ValueError,original.ObserverError)):old(self.values)
            with self.assertRaises((ValueError,original.ObserverError)):self.kernel(self.values)
            self.assertEqual(self.kernel.calls,0)
            compare_named_state(torch,baseline,self.policy)
        self.policy.mode=5
        with self.assertRaises(Exception) as before:self.policy(*self.kernel.tensors)
        with self.assertRaises(type(before.exception)) as after:self.kernel(self.values)
        self.assertIn('ORIGINAL_MODEL_FAILURE_UNCHANGED',str(after.exception))
        self.assertEqual(self.kernel.calls,0)
    def test_outer_original_freshness_command_and_failure_commit_preserved(self):
        self.ready();model=object.__new__(original.LivePolicyModel)
        model.policy=self.wrapper.inner;model.torch=torch;model._startup_stage='ready'
        model.last_tick_ns=0;model.last_imu_ns=0;model.calls=0;model.accel_input_hypothesis=None
        model.input_buffers=self.kernel.input_buffers;model.tensors=self.kernel.tensors
        # Injected fixture is labelled, and supplies no current sensor/physical grant.
        model.validate_inputs=lambda sample,imu,now:self.values
        call=CheckedLiveCallComponent(model,self.wrapper,original_source_sha256=SOURCE_SHA)
        target=call({}, {'read_started_monotonic_ns':1},2)
        self.assertEqual(model.calls,1);self.assertEqual(model.last_target,target)
        with self.assertRaisesRegex(ValueError,'reused'):call({}, {'read_started_monotonic_ns':2},2)
        with self.assertRaisesRegex(ValueError,'bounded forward'):call({}, {'read_started_monotonic_ns':3},4,command_override=(.051,0.,0.))
        self.assertEqual(model.calls,1);self.policy.mode=1
        with self.assertRaises(original.ObserverError):call({}, {'read_started_monotonic_ns':3},4)
        self.assertEqual((model.last_imu_ns,model.last_tick_ns,model.calls),(1,2,1))
        model.accel_input_hypothesis=object()
        with self.assertRaisesRegex(ValueError,'boxed-only'):call({}, {'read_started_monotonic_ns':3},4,command_override=(0.,0.,0.))
        self.assertTrue(all(call.provenance[k] is False for k in ('output_allowed','approved_for_runtime','hardware_opened')))


if __name__=='__main__':unittest.main()
