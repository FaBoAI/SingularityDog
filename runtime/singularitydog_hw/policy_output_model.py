"""Pinned, stateful CPU policy adapter for a separately reviewed output runner.

The existing no-output observer is unchanged. This adapter has no motor API;
calibrated CAN-order inputs and targets are checked independently by the runner.
"""
from array import array
import hashlib
import json
import math
from pathlib import Path
import sys

from . import policy_shadow as shadow
from .policy_observer import _mount, _bias, _tensor_row, _TARGET_LOWER, _TARGET_UPPER, RAW_IMU_CORRECTION_FLAGS
from .policy_observer_replay import warmup_policy
from .policy_live_profile import (execution_settings, SCALAR_BACKEND,
    acceleration_calibration_selected, accel_input_hypothesis_settings)
from .imu_calibration_review import reviewed_acceleration


def _copy_json_tree(value):
    """Copy containers of an already strictly parsed, private JSON snapshot.

    JSON scalar leaves are immutable. Rebuilding every dict/list keeps emitted
    telemetry independent without repeating the frozen hypothesis JSON parse.
    This is not a general-purpose copy or a validation path for new input.
    """
    if type(value) is dict:
        return {key:_copy_json_tree(item) for key,item in value.items()}
    if type(value) is list:
        return [_copy_json_tree(item) for item in value]
    return value


def _cpu_float32_row(value,count,label,torch_module):
    # This is the pinned model's declared ABI. Check it before any reduction or
    # conversion; integers, complex tensors, sparse layouts and devices are not
    # alternate model-output formats.
    if (not isinstance(value,torch_module.Tensor) or value.device.type!='cpu'
            or value.dtype!=torch_module.float32 or value.layout!=torch_module.strided
            or value.shape!=(1,count)):
        raise ValueError('Invalid '+label+'; expected CPU float32 [1,'+str(count)+']')


def _finite_cpu_float32_row(value,count,label,torch_module):
    """Validate an unused telemetry tensor without copying all elements to Python.

    Torch's CPU aminmax scans the values in C++ and propagates NaN. Testing both
    scalar extrema rejects either sign of infinity as well. This neither mutates
    state nor changes actor/observation values; only two scalar values cross into
    Python. The target itself still needs a Python row for the motion envelope.
    """
    _cpu_float32_row(value,count,label,torch_module)
    lower,upper=torch_module.aminmax(value)
    if not math.isfinite(lower.item()) or not math.isfinite(upper.item()):
        raise ValueError('Invalid '+label+'; nonfinite tensor')


class LivePolicyModel:
    def __init__(self,profile,*,policy=None,torch_module=None,defer_warmup=False):
        if not profile.get('output_allowed'):raise ValueError('Reviewed output profile required')
        if type(defer_warmup) is not bool:raise ValueError('defer_warmup must be a bool')
        if torch_module is None:import torch as torch_module
        self.torch=torch_module;self.profile=profile
        execution=execution_settings(profile)
        documents={}
        for name in ('mount','bias'):
            artifact=profile['artifacts'][name]
            raw=Path(artifact['path']).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=artifact['sha256']:
                raise ValueError(f'{name} changed since profile review')
            documents[name]=json.loads(raw)
        self.rotation=_mount(documents['mount'])['R_body_from_sensor']
        self.bias=_bias(documents['bias'])['bias_sensor_rad_s']
        self.accel_calibration=reviewed_acceleration(documents['bias'],self.rotation,
            enabled=acceleration_calibration_selected(profile))
        self.accel_input_hypothesis = None
        self._accel_provenance_source = None
        self._accel_provenance_snapshot = None
        hypothesis = accel_input_hypothesis_settings(profile)
        if hypothesis is not None:
            from .imu_accel_input_hypothesis import (AccelInputHypothesis,
                                                    load_accel_input_hypothesis)
            self.accel_input_hypothesis = load_accel_input_hypothesis(hypothesis, self.rotation)
            self.accel_calibration = self.accel_input_hypothesis
            if type(self.accel_input_hypothesis) is AccelInputHypothesis:
                # Only this exact loader-owned frozen type has immutable source
                # JSON. Custom/subclass corrections retain per-call provenance.
                self._accel_provenance_snapshot = self.accel_input_hypothesis.provenance()
                self._accel_provenance_source = self.accel_input_hypothesis
        if policy is None:
            sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
            from native_policy_overnight import load_verified
            artifact=profile['artifacts']['model_manifest']
            if execution['model_backend']==SCALAR_BACKEND:
                # The scalar loader retains all exact output/observation/state,
                # rejection, source and ABI checks. Its file-only provenance is
                # unchanged; only the separately reviewed profile grants output.
                from native_policy_overnight.model_call_fastpath.scalar_loader import load_file_only_verified
                scalar=profile['artifacts']['scalar_step_manifest']
                policy,self.provenance=load_file_only_verified(scalar['path'],
                    expected_sha256=scalar['sha256'],baseline_manifest=artifact['path'],
                    baseline_sha=artifact['sha256'],bundle=profile['bundle_path'])
            else:
                policy,self.provenance=load_verified(artifact['path'],expected_manifest_sha256=artifact['sha256'],bundle=profile['bundle_path'])
        else:self.provenance={'injected_model':True,'hardware_verified':False}
        self.execution={'model_backend':execution['model_backend'],
            'adapter_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'approval_source':'separately_reviewed_supported_profile',
            'model_artifact_grants_output':False}
        self.policy=policy;self.last_imu_ns=0;self.last_tick_ns=0;self.calls=0;self.last_target=None
        self.last_validation=None
        self._startup_stage='pending_warmup' if defer_warmup else 'ready'
        if not defer_warmup:
            warmup_policy(policy,self.torch,profile['h_hypothesis'],10)
            with self.torch.inference_mode():policy.reset(self.torch.tensor([0],dtype=self.torch.long))
        # CPU tensors share writable, fixed-size float buffers. Filling these
        # avoids six temporary tensors and six tensor-to-tensor copies per tick.
        self.input_buffers=tuple(array('f',[0.]*n) for n in (3,3,3,12,12,12))
        self.tensors=tuple(self.torch.frombuffer(buf,dtype=self.torch.float32).reshape(1,len(buf))
                           for buf in self.input_buffers)

    def pre_pin_warmup(self):
        if self._startup_stage!='pending_warmup':raise RuntimeError('Pre-pin warmup is not pending')
        warmup_policy(self.policy,self.torch,self.profile['h_hypothesis'],10)
        self._startup_stage='warmed'

    def post_pin_prime(self):
        if self._startup_stage!='warmed':raise RuntimeError('Post-pin prime requires pre-pin warmup')
        sizes=(3,3,3,12,12,12)
        if (len(self.input_buffers)!=6 or len(self.tensors)!=6 or
                any(type(buf) is not array or len(buf)!=size or
                    tensor.data_ptr()!=buf.buffer_info()[0] or
                    tensor.device.type!='cpu' or tensor.dtype!=self.torch.float32 or
                    tuple(tensor.shape)!=(1,size)
                    for buf,tensor,size in zip(self.input_buffers,self.tensors,sizes))):
            raise RuntimeError('Post-pin prime requires six owned reused CPU float input buffers')
        warmup_policy(self.policy,self.torch,self.profile['h_hypothesis'],10,input_tensors=self.tensors)
        self._startup_stage='primed'

    def finish_startup(self):
        if self._startup_stage not in ('warmed','primed'):
            raise RuntimeError('Startup reset requires completed warmup')
        with self.torch.inference_mode():
            self.policy.reset(self.torch.tensor([0],dtype=self.torch.long))
        self._startup_stage='ready'

    def validate_inputs(self,sample,imu,now_ns):
        if (imu.get('frame')!='sensor' or any(imu.get(key,False) is not False for key in
                RAW_IMU_CORRECTION_FLAGS)):
            raise ValueError('Uncorrected sensor-frame IMU required')
        begin=imu.get('read_started_monotonic_ns');end=imu.get('read_finished_monotonic_ns')
        if not (type(begin) is int and type(end) is int and 0<begin<=end<=now_ns and
                now_ns-begin<=self.profile['max_sample_age_ms']*1e6):raise ValueError('Stale/noncausal IMU')
        for name in ('accel_m_s2','gyro_rad_s'):
            values=imu.get(name)
            if not isinstance(values,(list,tuple)) or len(values)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in values):
                raise ValueError('Invalid IMU vector')
        norm=math.hypot(*imu['accel_m_s2'])
        if not self.profile['imu_accel_norm_min_m_s2']<=norm<=self.profile['imu_accel_norm_max_m_s2']:
            raise ValueError('IMU gravity-proxy norm outside reviewed range')
        corrected_accel,corrected_norm=(imu['accel_m_s2'],norm) if self.accel_calibration is None else self.accel_calibration.correct(imu['accel_m_s2'])
        body=[sum(row[j]*corrected_accel[j] for j in range(3)) for row in self.rotation]
        gyro=[sum(row[j]*(imu['gyro_rad_s'][j]-self.bias[j]) for j in range(3)) for row in self.rotation]
        gravity=[-v/corrected_norm for v in body]
        tilt=math.acos(max(-1.,min(1.,-gravity[2])))
        # A candidate may alter the policy's gravity input, but must never hide
        # a tilt that the original raw-input monitor would have rejected.
        raw_tilt = None
        if self.accel_input_hypothesis is not None:
            raw_body_z = sum(self.rotation[2][j]*imu['accel_m_s2'][j] for j in range(3))
            raw_tilt = math.acos(max(-1., min(1., raw_body_z/norm)))
            if raw_tilt > self.profile['imu_tilt_limit_rad']:
                raise ValueError('Raw body tilt exceeded with acceleration hypothesis')
        if tilt>self.profile['imu_tilt_limit_rad'] or math.hypot(*gyro)>self.profile['imu_gyro_limit_rad_s']:
            raise ValueError('Body tilt/angular velocity exceeded')
        q=[sample.q_model_rad[i-1] for i in shadow.CAN_ORDER]
        dq=[sample.velocity_rad_s[i-1] for i in shadow.CAN_ORDER]
        if not all(math.isfinite(x) and lo<=x<=hi for x,lo,hi in zip(q,shadow.LOWER,shadow.UPPER)):
            raise ValueError('Measured joints outside learned-model range')
        self.last_validation={'accel_m_s2':body,'gyro_rad_s':gyro,'tilt_rad':tilt,
            'accel_norm_m_s2':norm,'source_monotonic_ns':begin,'frame':'body',
            'mount_rotation_applied':True,'gyro_bias_subtracted':True}
        if self.accel_calibration is not None:
            self.last_validation.update(raw_accel_sensor_m_s2=list(imu['accel_m_s2']),
                raw_accel_norm_m_s2=norm,corrected_accel_norm_m_s2=corrected_norm,
                accel_bias_subtracted=True,accel_scale_corrected=True)
            key = ('accel_input_hypothesis' if self.accel_input_hypothesis is not None
                   else 'reviewed_accel_calibration')
            self.last_validation[key]=(
                _copy_json_tree(self._accel_provenance_snapshot)
                if self.accel_calibration is self._accel_provenance_source
                else self.accel_calibration.provenance())
            if raw_tilt is not None:
                self.last_validation['raw_tilt_rad'] = raw_tilt
        return gyro,gravity,list(self.profile['command']),q,dq,[float(self.profile['h_hypothesis'])]*12

    def __call__(self,sample,imu,now_ns,*,command_override=None):
        if self._startup_stage!='ready':raise RuntimeError('Policy startup reset is incomplete')
        if now_ns<=self.last_tick_ns or imu['read_started_monotonic_ns']<=self.last_imu_ns:
            raise ValueError('Model tick or IMU reused')
        values=self.validate_inputs(sample,imu,now_ns)
        if command_override is not None:
            if self.accel_input_hypothesis is not None:
                raise ValueError('Acceleration input hypothesis is boxed-only; no locomotion override')
            # Only the separate reviewed ground runner supplies an override.
            # Supported output keeps its profile's zero locomotion command.
            command=tuple(command_override)
            if (len(command)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in command)
                    or not 0<=command[0]<=.05 or command[1:]!=(0.,0.)):
                raise ValueError('Ground command must be bounded forward-only <=0.05 m/s')
            values=(*values[:2],list(command),*values[3:])
        with self.torch.inference_mode():
            for buf,row in zip(self.input_buffers,values):
                for j,value in enumerate(row):buf[j]=value
            raw_output=self.policy(*self.tensors)
            _cpu_float32_row(raw_output,12,'live policy target',self.torch)
            output=_tensor_row(raw_output,12,'live policy target')
            _finite_cpu_float32_row(self.policy.last_actor_output,12,'actor output',self.torch)
            _finite_cpu_float32_row(self.policy.last_observation,74,'observation',self.torch)
        if not all(lo<=q<=hi for q,lo,hi in zip(output,_TARGET_LOWER,_TARGET_UPPER)):
            raise ValueError('Learned target outside model range')
        target=[0.]*12
        for i,q in zip(shadow.CAN_ORDER,output):target[i-1]=q
        self.last_imu_ns=imu['read_started_monotonic_ns'];self.last_tick_ns=now_ns
        self.last_target=tuple(target);self.calls+=1
        return self.last_target
