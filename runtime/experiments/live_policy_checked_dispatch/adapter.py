"""Default-unselected CPU component, with no runtime/output admission.

TorchScript retains the exact original actor/controller and named state. Only
the original post-forward output validation and CAN conversion are dispatched
inside the same TorchScript call. Sensor validation, freshness/command checks,
and successful-call commit remain the original LivePolicyModel outer contract.
The R11 diagnostic loader's STOP-only qualification is never a live grant.
"""
from typing import List
import re
import torch
if __package__:
    from .cpu_kernel import GuardedCpuPolicyKernel, need, FALSE_FLAGS, verify_original_source
else:
    from cpu_kernel import GuardedCpuPolicyKernel, need, FALSE_FLAGS, verify_original_source
from singularitydog_hw import policy_output_model as original

MARKERS={
    'TARGET_ABI':(ValueError,'Invalid live policy target; expected CPU float32 [1,12]'),
    'TARGET_FINITE':(original.ObserverError,'Invalid live policy target'),
    'ACTOR_ABI':(ValueError,'Invalid actor output; expected CPU float32 [1,12]'),
    'ACTOR_FINITE':(ValueError,'Invalid actor output; nonfinite tensor'),
    'OBSERVATION_ABI':(ValueError,'Invalid observation; expected CPU float32 [1,74]'),
    'OBSERVATION_FINITE':(ValueError,'Invalid observation; nonfinite tensor'),
    'TARGET_RANGE':(ValueError,'Learned target outside model range'),
}

class CheckedPolicyDispatch(torch.nn.Module):
    def __init__(self,inner):
        super().__init__();self.inner=inner
    def forward(self,gyro:torch.Tensor,gravity:torch.Tensor,command:torch.Tensor,
                q:torch.Tensor,dq:torch.Tensor,h:torch.Tensor)->List[float]:
        target=self.inner(gyro,gravity,command,q,dq,h)
        return torch.ops.sd_live_checked_dispatch_private_r1.checked_targets(
            target,self.inner.last_actor_output,self.inner.last_observation)


def wrap_policy(policy):
    need(isinstance(policy,torch.jit.ScriptModule),'Genuine serialized TorchScript policy required')
    for method in policy._c._method_names():
        need('PRIVATE_LIVE_CHECKED_' not in policy._c._get_method(method).code,
             'Original model must not spoof new validator markers')
    wrapped=torch.jit.script(CheckedPolicyDispatch(policy).eval())
    # ScriptModule wrapping must share the intended original IValue state, not
    # leave warm/reset acting on a detached mutable-state copy.
    for accessor in ('named_buffers','named_parameters'):
        one,two=dict(getattr(policy,accessor)()),dict(getattr(wrapped.inner,accessor)())
        need(tuple(one)==tuple(two),'Wrapper inner named-state key/order differs')
        for name,value in one.items():
            need(value.untyped_storage().data_ptr()==two[name].untyped_storage().data_ptr(),
                 'Wrapper detached original state: '+name)
    return wrapped


def checked_call(wrapper,tensors):
    try:
        return tuple(wrapper(*tensors))
    except (RuntimeError,torch.jit.Error) as exc:
        # Pinned original code cannot emit these markers. Translate only the
        # exact new operator errors; preserve original policy failures as-is.
        matches=re.findall(r'PRIVATE_LIVE_CHECKED_([A-Z_]+)_R1',str(exc))
        known=set(matches)&set(MARKERS)
        if len(known)==1:
            kind,message=MARKERS[known.pop()]
            raise kind(message) from exc
        raise


class CheckedDispatchCpuKernel(GuardedCpuPolicyKernel):
    """CPU-only warm/reset/state boundary using the genuine original inner."""
    def __init__(self,wrapper,torch_module,*,original_source_sha256):
        super().__init__(wrapper.inner,torch_module,original_source_sha256=original_source_sha256);self.wrapper=wrapper
        self.provenance['schema']='experimental.live-checked-dispatch-cpu-component.v1'
        self.provenance['original_policy_inner_retained']=True
    def __call__(self,values):
        need(self.stage=='ready','Startup reset is incomplete')
        need(type(values) in (tuple,list) and len(values)==6 and
             all(type(row) in (tuple,list) and len(row)==n for row,n in zip(values,(3,3,3,12,12,12))),
             'Six exact component input rows required')
        with self.torch.inference_mode():
            for buf,row in zip(self.input_buffers,values):
                for j,value in enumerate(row):buf[j]=value
            target=checked_call(self.wrapper,self.tensors)
        self.calls+=1
        return target


class CheckedLiveCallComponent:
    """Offline outer-call parity fixture; never a registered live selector.

    The supplied original LivePolicyModel already owns sensor/profile/clock
    guards. This wrapper supplies no source or output approval and exposes only
    a CPU call component. A production caller would require a new reviewed
    source/selector/provenance and source-specific full-loop qualification.
    """
    def __init__(self,model,wrapper,*,original_source_sha256):
        verify_original_source(original_source_sha256)
        need(type(model) is original.LivePolicyModel,'Exact original live model fixture required')
        need(model.policy is wrapper.inner or model.policy._c is wrapper.inner._c,
             'Wrapper must retain the exact original model state')
        self.model,self.wrapper=model,wrapper
        self.provenance=dict(schema='experimental.live-checked-dispatch-outer-component.v1',
            physical_future_observations=None,**dict.fromkeys(FALSE_FLAGS,False))
    def __call__(self,sample,imu,now_ns,*,command_override=None):
        model=self.model
        if model._startup_stage!='ready':raise RuntimeError('Policy startup reset is incomplete')
        if now_ns<=model.last_tick_ns or imu['read_started_monotonic_ns']<=model.last_imu_ns:
            raise ValueError('Model tick or IMU reused')
        values=model.validate_inputs(sample,imu,now_ns)
        if command_override is not None:
            if model.accel_input_hypothesis is not None:
                raise ValueError('Acceleration input hypothesis is boxed-only; no locomotion override')
            command=tuple(command_override)
            import math
            if (len(command)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in command)
                    or not 0<=command[0]<=.05 or command[1:]!=(0.,0.)):
                raise ValueError('Ground command must be bounded forward-only <=0.05 m/s')
            values=(*values[:2],list(command),*values[3:])
        with model.torch.inference_mode():
            for buf,row in zip(model.input_buffers,values):
                for j,value in enumerate(row):buf[j]=value
            target=checked_call(self.wrapper,model.tensors)
        model.last_imu_ns=imu['read_started_monotonic_ns'];model.last_tick_ns=now_ns
        model.last_target=target;model.calls+=1
        return model.last_target
