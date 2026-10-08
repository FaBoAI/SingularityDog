"""File-only parity/timing of old parser, current default and opt-in C++ wires."""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import platform
import random
import statistics
import time

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import native_active_transport as native
from singularitydog_hw import native_policy_batch_encode as batch
from singularitydog_hw import policy_output_runtime as runtime


def bindings(specs):
    return ({i:row[0] for i,row in enumerate(specs,1)},
            {str(i):dict(sign=row[1],lower_rad=row[2],upper_rad=row[3],
                        max_displacement_from_start_rad=row[5],
                        max_estimated_pd_torque_nm=row[6]) for i,row in enumerate(specs,1)},
            tuple(row[4] for row in specs))


def legacy(command,offsets,axes,initial,*,encode_motion=native.encode_motion):
    """Original default algorithm, including all twelve parser allocations."""
    raws={i:(command.q_model_rad[i-1]-offsets[i])/axes[str(i)]['sign'] for i in runtime.IDS}
    result={s:[encode_motion(i,raws[i],command.kp[i-1],command.kd[i-1])
               for i in ids] for s,ids in runtime.BUSES.items()}
    for wires in result.values():
        for wire in wires:
            frame=codec.ATParser().feed(wire)[0];i=frame.destination;a=axes[str(i)]
            raw=int.from_bytes(frame.data[:2],'big')*25.14/65535-12.57
            q=a['sign']*raw+offsets[i]
            runtime.need(a['lower_rad']<=q<=a['upper_rad'],f'ID{i} quantized target outside physical range')
            runtime.need(abs(q-initial[i-1])<=a['max_displacement_from_start_rad'],
                         f'ID{i} quantized target outside trial displacement')
            estimated=command.kp[i-1]*(q-command.q_model_rad[i-1])+command.estimated_pd_torque_nm[i-1]
            runtime.need(abs(estimated)<=a['max_estimated_pd_torque_nm'],f'ID{i} quantized estimated PD torque')
    return result


def same(actual,expected):
    if actual!=expected or list(actual)!=['front','rear'] or any(
            type(ws) is not list or len(ws)!=6 or
            any(type(w) is not bytes or len(w)!=17 for w in ws) for ws in actual.values()):
        raise AssertionError('Twelve quantized motion wires or bus order differ')


def exhaustive():
    """Inject every u16 into twelve genuine fixed frames; retain exact guards."""
    args=bindings(((0.,1.,-12.57,12.57,0.,26.,1.),)*12)
    command=batch._command((0.,)*12,(0.,)*12,(0.,)*12,(0.,)*12)
    digest=hashlib.sha256()
    for position in range(65536):
        wires=tuple(b'AT'+((((1<<24)|(32767<<8)|mid)<<3)|4).to_bytes(4,'big')+
                    b'\x08'+position.to_bytes(2,'big')+b'\x7f\xff'+bytes(4)+b'\r\n'
                    for mid in range(1,13))
        encoder=lambda mid,*unused:wires[mid-1]
        expected=legacy(command,*args,encode_motion=encoder)
        actual=runtime._python_motion_wires(command,*args,encode_motion=encoder)
        same(actual,expected)
        for wire in wires:digest.update(wire)
    return dict(position_codes=65536,axes=12,checked_wire_positions=65536*12,
                reference_wires_sha256=digest.hexdigest())


def summary(values):
    ordered=sorted(values)
    return dict(samples=len(values),median_us=statistics.median(values)/1000,
                p99_us=ordered[int((len(ordered)-1)*.99)]/1000,max_us=max(values)/1000)


def run(*,library=None,binary_sha256=None,samples=2000,trials=4,parity_cases=2000,
        exhaustive_u16=False):
    for value,maximum in ((samples,10000),(trials,10),(parity_cases,10000)):
        if type(value) is not int or not 1<=value<=maximum:
            raise ValueError('Bounded positive benchmark counts required')
    if (library is None)!=(binary_sha256 is None) or type(exhaustive_u16) is not bool:
        raise ValueError('Library and reviewed SHA must be selected together')
    specs=batch._test_specs();args=bindings(specs)
    methods={'legacy_parser':lambda c:legacy(c,*args),
             'default_fixed_frame':lambda c:runtime._python_motion_wires(c,*args,encode_motion=native.encode_motion)}
    if library is not None:
        methods['verified_cpp']=batch.load_verified_module(library,
            expected_binary_sha256=binary_sha256).bind(specs)
    rng=random.Random(890);initial=args[2]
    cases=tuple(batch._command((q+rng.uniform(-.015,.015) for q in initial),
                (rng.uniform(0.,10.) for _ in initial),(rng.uniform(0.,.5) for _ in initial),
                (rng.uniform(-.01,.01) for _ in initial)) for _ in range(parity_cases))
    digest=hashlib.sha256()
    for command in cases:
        expected=methods['legacy_parser'](command)
        for method in methods.values():same(method(command),expected)
        for wires in expected.values():
            for wire in wires:digest.update(wire)
    exhaustive_result=exhaustive() if exhaustive_u16 else None
    for index in range(200):
        for method in methods.values():method(cases[index%len(cases)])
    original_gc=gc.isenabled();rows=[];names=tuple(methods)
    gc.collect();gc.disable()
    try:
        for trial in range(trials):
            measured={name:[] for name in names}
            for index in range(samples):
                command=cases[index%len(cases)];results={};offset=(trial+index)%len(names)
                order=names[offset:]+names[:offset]
                if trial%2:order=tuple(reversed(order))
                for name in order:
                    begin=time.perf_counter_ns();result=methods[name](command)
                    measured[name].append(time.perf_counter_ns()-begin);results[name]=result
                for result in results.values():same(result,results['legacy_parser'])
            rows.append(dict(trial=trial,**{name:summary(values) for name,values in measured.items()},
                             samples_ns=measured))
    finally:
        if original_gc:gc.enable()
    return dict(schema='singularitydog.runtime-motion-wire-benchmark.v1',
        platform=platform.platform(),machine=platform.machine(),
        scope='twelve Type1 bytes plus original quantized limit checks; no I/O or inference',
        measurement_order='rotated per sample; reversed on alternate trials',
        source_sha256={str(Path(source).name):hashlib.sha256(Path(source).read_bytes()).hexdigest()
                       for source in (__file__,runtime.__file__,native.__file__,batch.__file__)},
        binary_sha256=binary_sha256,parity_cases=parity_cases,exhaustive_u16=exhaustive_result,
        reference_wires_sha256=digest.hexdigest(),gc_restored=gc.isenabled()==original_gc,
        hardware_opened=False,motor_commands_sent=False,model_loaded=False,profile_changed=False,
        output_approved=False,jetson_measured=False,whole_cycle_measured=False,trials=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--library',type=Path)
    parser.add_argument('--binary-sha256')
    parser.add_argument('--samples',type=int,default=2000)
    parser.add_argument('--trials',type=int,default=4)
    parser.add_argument('--parity-cases',type=int,default=2000)
    parser.add_argument('--exhaustive-u16',action='store_true')
    args=parser.parse_args()
    if args.output.exists():parser.error('Fresh output required')
    report=run(library=args.library,binary_sha256=args.binary_sha256,samples=args.samples,
               trials=args.trials,parity_cases=args.parity_cases,exhaustive_u16=args.exhaustive_u16)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps({key:report[key] for key in ('parity_cases','exhaustive_u16','gc_restored')}))
    print(json.dumps([{key:value for key,value in row.items() if key!='samples_ns'} for row in report['trials']]))


if __name__=='__main__':main()
