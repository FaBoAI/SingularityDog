"""CPU-only live inference-kernel comparison; no runtime/output admission.

The six buffers, 12-target conversion, actor12/observation74 validation, learned
target range, CAN permutation and 10/10/reset protocol reuse the frozen live
model's exact helpers. Sensor/current/physical/clock admission is not supplied by
this component. Existing LivePolicyModel and every runtime selector are untouched.
"""
from array import array
import hashlib
import math
from pathlib import Path
import re

from singularitydog_hw import policy_output_model as original
from singularitydog_hw import policy_shadow as shadow

OLD_NAMESPACE = 'sd_target_tail_fk_cache_fileonly_r1'
NEW_NAMESPACE = 'sd_target_envelope_scalar_fileonly_r11'
FALSE_FLAGS = ('output_allowed', 'approved_for_runtime', 'active_controller_qualification',
               'timing_admission_eligible', 'live_50hz_verified', 'hardware_opened')
SIZES = (3,3,3,12,12,12)


def need(condition, message):
    if not condition:
        raise ValueError(message)


def verify_original_source(expected_sha256):
    need(type(expected_sha256) is str and re.fullmatch(r"[0-9a-f]{64}",expected_sha256) is not None,
         "Explicit original live adapter source SHA256 required")
    path=Path(original.__file__)
    need(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)) and
         hashlib.sha256(path.read_bytes()).hexdigest()==expected_sha256,
         'Exact unchanged original live adapter source required')
    return {'path':str(path),'sha256':expected_sha256}


def compare_named_state(torch, baseline, candidate, *, independent=False):
    for accessor in ('named_buffers','named_parameters'):
        one,two=dict(getattr(baseline,accessor)()),dict(getattr(candidate,accessor)())
        need(tuple(one)==tuple(two),'Named state key/order differs')
        for name,value in one.items():
            other=two[name]
            need(value.dtype==other.dtype and value.shape==other.shape and
                 value.device.type==other.device.type=='cpu' and
                 torch.equal(value.contiguous().reshape(-1).view(torch.uint8),
                             other.contiguous().reshape(-1).view(torch.uint8)),
                 'Named state bits differ: '+name)
            if independent and value.numel():
                need(value.untyped_storage().data_ptr()!=other.untyped_storage().data_ptr(),
                     'Candidate borrows original state: '+name)


def verify_methods(baseline,candidate):
    normalize=lambda code:re.sub(r'___torch_mangle_[0-9]+\.', '', code)
    for one,two,label in ((baseline,candidate,'policy'),(baseline.actor,candidate.actor,'actor'),
                          (baseline.controller,candidate.controller,'controller')):
        names=set(one._c._method_names())
        need(names==set(two._c._method_names()),label+' method names differ')
        for name in names:
            old,new=normalize(one._c._get_method(name).code),normalize(two._c._get_method(name).code)
            if label=='controller' and name=='step_target':
                need(old.count(OLD_NAMESPACE)==new.count(NEW_NAMESPACE)==1,'One target operator replacement required')
                new=new.replace(NEW_NAMESPACE,OLD_NAMESPACE)
            need(old==new,'Executable method differs: '+label+'.'+name)


class GuardedCpuPolicyKernel:
    """The live model's CPU kernel, explicitly unusable as a profile grant."""
    def __init__(self,policy,torch_module,*,original_source_sha256):
        self.source=verify_original_source(original_source_sha256)
        self.policy,self.torch=policy,torch_module
        self.input_buffers=tuple(array('f',[0.]*n) for n in SIZES)
        self.tensors=tuple(self.torch.frombuffer(buf,dtype=self.torch.float32).reshape(1,len(buf))
                          for buf in self.input_buffers)
        self.stage='pending_warmup';self.calls=0
        self.provenance={'schema':'experimental.live-checked-reference-cpu-component.v1',
            'original_live_source':self.source,'physical_observation':None,
            'sensor_clock_current_and_output_admission_supplied':False,
            **dict.fromkeys(FALSE_FLAGS,False)}

    def pre_pin_warmup(self,*,h_hypothesis=0.):
        need(self.stage=='pending_warmup','Pre-pin warmup is not pending')
        original.warmup_policy(self.policy,self.torch,h_hypothesis,10)
        self.stage='warmed'

    def post_pin_prime(self,*,h_hypothesis=0.):
        need(self.stage=='warmed','Post-pin prime requires pre-pin warmup')
        need(len(self.input_buffers)==len(self.tensors)==6 and
             all(type(buf) is array and len(buf)==n and tensor.data_ptr()==buf.buffer_info()[0] and
                 tensor.device.type=='cpu' and tensor.dtype==self.torch.float32 and tuple(tensor.shape)==(1,n)
                 for buf,tensor,n in zip(self.input_buffers,self.tensors,SIZES)),
             'Post-pin prime requires six owned reused CPU float input buffers')
        original.warmup_policy(self.policy,self.torch,h_hypothesis,10,input_tensors=self.tensors)
        self.stage='primed'

    def finish_startup(self):
        need(self.stage in ('warmed','primed'),'Startup reset requires completed warmup')
        with self.torch.inference_mode():
            self.policy.reset(self.torch.tensor([0],dtype=self.torch.long))
        self.stage='ready';self.calls=0

    def reset(self):
        need(self.stage=='ready','Component kernel is not ready')
        with self.torch.inference_mode():
            self.policy.reset(self.torch.tensor([0],dtype=self.torch.long))
        self.calls=0

    def __call__(self,values):
        need(self.stage=='ready','Startup reset is incomplete')
        need(type(values) in (tuple,list) and len(values)==6 and
             all(type(row) in (tuple,list) and len(row)==n for row,n in zip(values,SIZES)),
             'Six exact component input rows required')
        with self.torch.inference_mode():
            for buf,row in zip(self.input_buffers,values):
                for j,value in enumerate(row):buf[j]=value
            raw_output=self.policy(*self.tensors)
            output=original._cpu_float32_target_row(raw_output,12,'live policy target',self.torch)
            original._finite_cpu_float32_row(self.policy.last_actor_output,12,'actor output',self.torch)
            original._finite_cpu_float32_row(self.policy.last_observation,74,'observation',self.torch)
        if not all(lo<=q<=hi for q,lo,hi in zip(output,original._TARGET_LOWER,original._TARGET_UPPER)):
            raise ValueError('Learned target outside model range')
        target=[0.]*12
        for i,q in zip(shadow.CAN_ORDER,output):target[i-1]=q
        self.calls+=1
        return tuple(target)
