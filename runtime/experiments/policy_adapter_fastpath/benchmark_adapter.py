"""File-only adapter A/B on saved501 values and an injected CPU tensor policy.

No CAN descriptor, network, or motor API is opened. A private baseline source is
loaded only when its explicit SHA matches. Both adapters use the same pinned
mount/bias files and saved correction coefficients. The injected correction
fixture is not a fresh calibration re-audit or output authorization. By default
this measures adapter overhead around saved tensor fixtures. --real-bundle also
verifies and scripts the original actor/controller on this CPU, independently
of the active FK custom-operator artifact. Neither mode measures a Jetson cycle.
"""
import argparse
from array import array
import gc
import hashlib
import importlib.util
import io
import json
import math
import marshal
from pathlib import Path
import platform
import statistics
import struct
import sys
import time
from unittest.mock import patch

from singularitydog_hw import imu_accel_input_hypothesis as hypotheses
from singularitydog_hw import policy_output_model as current
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.policy_motion_envelope import MotionSample

KEYS=('gyro_body_rad_s','gravity_body_unit','command','q_model_rad',
      'dq_model_rad_s','h_hypothesis12')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summary(samples):
    ordered=sorted(samples)
    return {'samples':len(samples),'median_us':statistics.median(samples)/1000,
            'p99_us':ordered[int((len(ordered)-1)*.99)]/1000,
            'max_us':max(samples)/1000}


def bits(values):return b''.join(struct.pack('>d',value) for value in values)


def same_tree(a,b):
    if type(a) is not type(b):raise AssertionError('Provenance type differs')
    if type(a) is dict:
        if a is b:raise AssertionError('Mutable provenance dictionary alias')
        if list(a)!=list(b):raise AssertionError('Provenance key order differs')
        for key in a:same_tree(a[key],b[key])
    elif type(a) in (list,tuple):
        if type(a) is list and a is b:raise AssertionError('Mutable provenance list alias')
        if len(a)!=len(b):raise AssertionError('Provenance length differs')
        for x,y in zip(a,b):same_tree(x,y)
    elif type(a) is float:
        if bits([a])!=bits([b]):raise AssertionError('Provenance float bits differ')
    elif a!=b:raise AssertionError('Provenance scalar differs')


class SavedTensorPolicy:
    """Output fixtures only; no assertion about true recurrent model behavior."""
    def __init__(self,records,torch):
        self.torch=torch;self.state=0;self.resets=0
        self.rows=tuple(tuple(torch.tensor([r['observed'][key]],dtype=torch.float32)
            for key in ('q_target_rad_diagnostic_only','actor_residual12','observation74'))
            for r in records)

    def reset(self,ids):
        if ids.tolist()!=[0]:raise AssertionError('Reset differs')
        self.state=0;self.resets+=1

    def __call__(self,*values):
        target,self.last_actor_output,self.last_observation=self.rows[self.state%len(self.rows)]
        self.state+=1
        return target


def inputs(records):
    values=[]
    for row in records:
        observed=row['observed'];saved=observed['inputs'];q=[0.]*12;dq=[0.]*12
        for i,mid in enumerate(shadow.CAN_ORDER):
            q[mid-1]=saved['q_model_rad'][i];dq[mid-1]=saved['dq_model_rad_s'][i]
        sample=MotionSample(tuple(q),tuple(dq),(0.,)*12,(25.,)*12,observed['tick_ns']/1e9)
        values.append((sample,row['imu'],observed['tick_ns'],saved))
    return values


def correction(records):
    p=records[0]['observed']['provenance']['accel_input_hypothesis']
    # Explicit synthetic loader-proof fixture from original captured provenance;
    # this never enters an actuation runner, rewrites a profile, or grants output.
    return hypotheses.AccelInputHypothesis(tuple(p['bias_sensor_m_s2']),tuple(p['scale_sensor']),
        *p['raw_norm_bounds_m_s2'],*p['corrected_norm_bounds_m_s2'],p['hypothesis_sha256'],
        p['candidate_sha256'],p['manifest_sha256'],
        json.dumps(p,ensure_ascii=False,allow_nan=False),hypotheses._VALIDATED)


def adapter(module,records,pins,torch,policy_factory=None):
    c=correction(records)
    profile={'schema':'singularitydog.supported-policy-profile.v1',
        'output_allowed':True,'artifacts':{'mount':pins['mount'],'bias':pins['gyro_bias']},
        'h_hypothesis':0.,'command':[0.,0.,0.],
        'max_sample_age_ms':20.,'imu_accel_norm_min_m_s2':c.raw_norm_min_m_s2,
        'imu_accel_norm_max_m_s2':c.raw_norm_max_m_s2,
        'imu_tilt_limit_rad':math.radians(10),'imu_gyro_limit_rad_s':.5}
    policy=SavedTensorPolicy(records,torch) if policy_factory is None else policy_factory()
    # This internal injected-model fixture admits only Python computation. There
    # is no serialized review/profile, hardware state, transport or output runner.
    with patch.object(module,'accel_input_hypothesis_settings',return_value={'fixture':True}), \
         patch.object(hypotheses,'load_accel_input_hypothesis',return_value=c):
        result=module.LivePolicyModel(profile,policy=policy,torch_module=torch)
    return result


def load_baseline(path,expected):
    if sha(path)!=expected:raise ValueError('Explicit baseline source SHA differs')
    name='singularitydog_hw._file_only_adapter_baseline_'+expected[:16]
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def component_probe(baseline,records,torch,*,samples,trials):
    record=records[0]['observed'];rows=tuple(record['inputs'][key] for key in KEYS)
    buffers=tuple(array('f',[0.]*len(row)) for row in rows)
    target=torch.tensor([record['q_target_rad_diagnostic_only']],dtype=torch.float32)
    provenance=record['provenance']['accel_input_hypothesis'];blob=marshal.dumps(provenance)
    factory=current._frozen_json_literal_factory(provenance)
    def fill_loop():
        for buf,row in zip(buffers,rows):
            for j,value in enumerate(row):buf[j]=value
    def fill_bulk():
        for buf,row in zip(buffers,rows):buf[:]=array('f',row)
    def fill_guarded_bulk():
        for buf,row in zip(buffers,rows):
            if all(type(value) in (int,float) for value in row):buf[:]=array('f',row)
            else:
                for j,value in enumerate(row):buf[j]=value
    def old_target():
        baseline._cpu_float32_row(target,12,'live policy target',torch)
        return baseline._tensor_row(target,12,'live policy target')
    funcs={'target_baseline':old_target,
        'target_candidate':lambda:current._cpu_float32_target_row(target,12,'live policy target',torch),
        'provenance_baseline':lambda:baseline._copy_json_tree(provenance),
        'provenance_marshal_only':lambda:marshal.loads(blob),
        'input_fill_loop_unchanged':fill_loop,'input_fill_bulk_unselected':fill_bulk,
        'input_fill_guarded_bulk_unselected':fill_guarded_bulk}
    if factory is not None:funcs['provenance_literal_only']=factory
    for func in funcs.values():
        for _ in range(100):func()
    raw={key:[] for key in funcs}
    for trial in range(trials):
        order=list(funcs) if trial%2==0 else list(reversed(funcs))
        for key in order:
            start=time.perf_counter_ns()
            for _ in range(samples):funcs[key]()
            raw[key].append((time.perf_counter_ns()-start)/samples)
    return {'scope':'component mean per call; no adapter guards on marshal-only',
            'samples_per_mean':samples,'raw_mean_ns':raw,
            'median_us':{key:statistics.median(value)/1000 for key,value in raw.items()}}


def run(baseline_path,baseline_sha,records_path,pins_path,*,trials,component_samples,
        real_bundle=None):
    import torch
    source_paths={'baseline':baseline_path,'candidate':current.__file__,'benchmark':__file__,
        'observer_helpers':Path(current.__file__).with_name('policy_observer.py'),
        'hypothesis':hypotheses.__file__,'original_bundle_loader':shadow.__file__}
    source_pins={key:sha(path) for key,path in source_paths.items()}
    baseline=load_baseline(baseline_path,baseline_sha)
    records=json.loads(records_path.read_bytes());pins=json.loads(pins_path.read_bytes())
    if len(records)!=501:raise ValueError('Original501 records required')
    for key in ('mount','gyro_bias'):
        if sha(pins[key]['path'])!=pins[key]['sha256']:raise ValueError(key+' SHA differs')
    policy_factory=None;model_source=None
    if real_bundle is not None:
        # Original checkpoint/controller pins are verified by this existing
        # file-only loader. A pure TorchScript copy runs on this CPU; the live
        # FK custom-operator artifact is not loaded, built, or qualified here.
        policy,model_source=shadow.load_policy(real_bundle)
        scripted=torch.jit.script(policy).eval();stream=io.BytesIO()
        torch.jit.save(scripted,stream);model_raw=stream.getvalue()
        model_source.update(export='pure TorchScript of pinned original bundle',
                            local_script_sha256=hashlib.sha256(model_raw).hexdigest())
        policy_factory=lambda:torch.jit.load(io.BytesIO(model_raw),map_location='cpu').eval()
    cases=inputs(records)
    old=adapter(baseline,records,pins,torch,policy_factory);new=adapter(current,records,pins,torch,policy_factory)
    previous_new_validation=None
    for index,(sample,imu,now,saved) in enumerate(cases):
        a=old(sample,imu,now);b=new(sample,imu,now)
        if type(a) is not tuple or type(b) is not tuple or bits(a)!=bits(b):
            raise AssertionError('Target bits/immutability differ: '+str(index))
        same_tree(old.last_validation,new.last_validation)
        if previous_new_validation is not None:
            if (previous_new_validation is new.last_validation or
                    previous_new_validation['accel_input_hypothesis'] is
                    new.last_validation['accel_input_hypothesis']):
                raise AssertionError('A mutable validation tree was reused across ticks')
        previous_new_validation=new.last_validation
        if (old.calls,old.last_tick_ns,old.last_imu_ns)!=(new.calls,new.last_tick_ns,new.last_imu_ns):
            raise AssertionError('Commit timestamps differ')
        if policy_factory is None:
            if (old.policy.state,old.policy.resets)!=(new.policy.state,new.policy.resets):
                raise AssertionError('Synthetic state/reset differs')
        else:
            state_a=dict(old.policy.named_buffers());state_b=dict(new.policy.named_buffers())
            if state_a.keys()!=state_b.keys():raise AssertionError('Named state differs')
            for name,value in state_a.items():
                other=state_b[name]
                if value.dtype!=other.dtype or value.shape!=other.shape or \
                   value.detach().contiguous().view(torch.uint8).numpy().tobytes()!= \
                   other.detach().contiguous().view(torch.uint8).numpy().tobytes():
                    raise AssertionError('Named buffer bits differ: '+name)
        for tensor_a,tensor_b,key in zip(old.tensors,new.tensors,KEYS):
            torch.testing.assert_close(tensor_a,tensor_b,rtol=0,atol=0)
            torch.testing.assert_close(tensor_a,torch.tensor([saved[key]],dtype=torch.float32),rtol=0,atol=0)
    enabled=gc.isenabled();gc.collect();gc.disable();raw=[]
    try:
        components=component_probe(baseline,records,torch,samples=component_samples,trials=trials)
        for trial in range(trials):
            old=adapter(baseline,records,pins,torch,policy_factory);new=adapter(current,records,pins,torch,policy_factory)
            durations={'baseline':[],'candidate':[]}
            for index,(sample,imu,now,_) in enumerate(cases):
                names=('baseline','candidate') if (trial+index)%2==0 else ('candidate','baseline')
                for name in names:
                    model=old if name=='baseline' else new
                    start=time.perf_counter_ns();target=model(sample,imu,now)
                    durations[name].append(time.perf_counter_ns()-start)
                    if type(target) is not tuple or len(target)!=12:raise AssertionError('Target shape differs')
            raw.append({'trial':trial,'samples_ns':durations,
                **{name:summary(value) for name,value in durations.items()},
                'median_saved_us':(statistics.median(durations['baseline'])-
                                  statistics.median(durations['candidate']))/1000})
    finally:
        if enabled:gc.enable()
    if source_pins!={key:sha(path) for key,path in source_paths.items()}:
        raise ValueError('Benchmark/adapter/helper source changed during comparison')
    return {'schema':'singularitydog.file-only-policy-adapter-benchmark.v1',
        'scope':('Mac CPU adapter + pure TorchScript pinned original checkpoint/controller; not active FK artifact'
            if policy_factory is not None else
            'Mac CPU adapter; injected saved target/actor/observation tensors; no true policy forward'),
        'hardware_opened':False,'robot_commands_sent':False,'approved_for_runtime':False,
        'output_allowed':False,'real_model_artifact_loaded':policy_factory is not None,
        'live_fk_artifact_loaded':False,'real_model_source':model_source,'whole_cycle_measured':False,
        'jetson_measured':False,'gc_restored':gc.isenabled()==enabled,
        'platform':platform.platform(),'machine':platform.machine(),'python':sys.version,'torch':torch.__version__,
        'torch_num_threads':torch.get_num_threads(),
        'torch_num_interop_threads':torch.get_num_interop_threads(),
        'source_sha256':source_pins,'source_bytes_unchanged_during_comparison':True,
        'records_sha256':sha(records_path),'matched_inputs_sha256':sha(pins_path),
        'mount_bias_sha256':{key:pins[key]['sha256'] for key in ('mount','gyro_bias')},
        'saved501_parity':{'targets_double_bits':True,'input_tensors_f32_bits':True,
            'provenance_types_order_double_bits':True,'source_timestamps_unchanged':True,
            'synthetic_state_reset_and_commit':policy_factory is None,
            'real_named_buffer_bits_each_tick':policy_factory is not None,
            'mutable_validation_independent':True},
        'components':components,'trials':raw}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-source',type=Path,required=True)
    p.add_argument('--baseline-sha256',required=True)
    p.add_argument('--records',type=Path,required=True)
    p.add_argument('--matched-inputs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--trials',type=int,default=9)
    p.add_argument('--component-samples',type=int,default=5000)
    p.add_argument('--real-bundle',type=Path)
    args=p.parse_args()
    if not 1<=args.trials<=25 or not 1<=args.component_samples<=100000:
        p.error('trials1..25 and component-samples1..100000 required')
    if args.output.exists():p.error('Fresh private output required')
    result=run(args.baseline_source,args.baseline_sha256,args.records,args.matched_inputs,
               trials=args.trials,component_samples=args.component_samples,real_bundle=args.real_bundle)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output),'parity':result['saved501_parity'],
                     'median_saved_us':[t['median_saved_us'] for t in result['trials']]}))


if __name__=='__main__':main()
