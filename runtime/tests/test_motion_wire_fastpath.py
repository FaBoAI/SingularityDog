"""Default Type1 byte/quantized-check parity; never open a motor descriptor."""
import math
import random
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import native_policy_batch_encode as batch
from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime


def bindings(specs):
    offsets={i:row[0] for i,row in enumerate(specs,1)}
    axes={str(i):dict(sign=row[1],lower_rad=row[2],upper_rad=row[3],
                      max_displacement_from_start_rad=row[5],
                      max_estimated_pd_torque_nm=row[6]) for i,row in enumerate(specs,1)}
    return offsets,axes,tuple(row[4] for row in specs)


def legacy(command,offsets,axes,initial):
    raws={i:(command.q_model_rad[i-1]-offsets[i])/axes[str(i)]['sign'] for i in runtime.IDS}
    result={s:[native.encode_motion(i,raws[i],command.kp[i-1],command.kd[i-1])
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


def candidate(command,*args):
    return runtime._python_motion_wires(command,*args,encode_motion=native.encode_motion)


def outcome(method,*args):
    try:return ('returned',method(*args))
    except Exception as error:return ('raised',type(error),str(error))


class MotionWireFastpathTests(unittest.TestCase):
    def test_seeded_bytes_bus_order_and_all_quantized_checks_match(self):
        rng=random.Random(890);specs=batch._test_specs();args=bindings(specs)
        initial=args[2]
        for _ in range(2000):
            command=batch._command((q+rng.uniform(-.015,.015) for q in initial),
                (rng.uniform(0.,10.) for _ in initial),(rng.uniform(0.,.5) for _ in initial),
                (rng.uniform(-.01,.01) for _ in initial))
            expected=legacy(command,*args);actual=candidate(command,*args)
            self.assertEqual(actual,expected)
            self.assertEqual(list(actual),['front','rear'])
            self.assertTrue(all(type(w) is bytes and len(w)==17 for ws in actual.values() for w in ws))

    def test_each_axis_numeric_and_quantized_failure_has_original_rejection(self):
        specs=batch._test_specs();initial=tuple(row[4] for row in specs)
        fields=('q_model_rad','kp','kd','estimated_pd_torque_nm')
        for mid in range(1,13):
            for field,value in (('q_model_rad',math.nan),('q_model_rad',13.),
                ('kp',37.),('kd',1.1),('kp',True),('kd',math.inf),
                ('estimated_pd_torque_nm',math.nan),('estimated_pd_torque_nm',1.)):
                original=batch._command(initial,(3.,)*12,(.15,)*12,(0.,)*12)
                values={name:list(getattr(original,name)) for name in fields}
                values[field][mid-1]=value
                command=batch._command(*(values[name] for name in fields))
                with self.subTest(mid=mid,field=field,value=value):
                    args=bindings(specs)
                    self.assertEqual(outcome(candidate,command,*args),
                                     outcome(legacy,command,*args))
            for index,value in ((2,initial[mid-1]+.001),(5,.000001),(6,.000001)):
                changed=[list(row) for row in specs];changed[mid-1][index]=value
                args=bindings(changed);command=batch._command(initial,(3.,)*12,(.15,)*12,(0.,)*12)
                with self.subTest(mid=mid,limit_index=index):
                    self.assertEqual(outcome(candidate,command,*args),
                                     outcome(legacy,command,*args))

    def test_noncanonical_frames_keep_original_parser_and_error_behavior(self):
        specs=batch._test_specs();args=bindings(specs)
        command=batch._command(args[2],(3.,)*12,(.15,)*12,(0.,)*12)
        encoder=native.encode_motion
        transforms=(lambda w:w,lambda w:b'noise'+w,lambda w:w+b'tail',
                    lambda w:w+w,lambda w:bytearray(w),lambda w:memoryview(w),
                    lambda w:w[:-1],lambda w:b'XX'+w[2:],
                    lambda w:w[:6]+b'\x09'+w[7:],lambda w:w[:-2]+b'XX',
                    lambda w:w[:5]+bytes((w[5]^1,))+w[6:])
        for transform in transforms:
            with self.subTest(transform=transform),patch.object(native,'encode_motion',
                    side_effect=lambda *values:transform(encoder(*values))):
                self.assertEqual(outcome(candidate,command,*args),
                                 outcome(legacy,command,*args))

    def test_later_encoder_error_precedes_earlier_quantized_error(self):
        specs=[list(row) for row in batch._test_specs()];specs[0][5]=.000001
        args=bindings(specs)
        kp=[3.]*12;kp[-1]=37.
        command=batch._command(args[2],kp,(.15,)*12,(0.,)*12)
        actual=outcome(candidate,command,*args)
        self.assertEqual(actual,outcome(legacy,command,*args))
        self.assertEqual(actual[2],'Target/gain outside active software caps')

    def test_default_canonical_path_does_not_allocate_stream_parsers(self):
        args=bindings(batch._test_specs())
        command=batch._command(args[2],(3.,)*12,(.15,)*12,(0.,)*12)
        expected=legacy(command,*args)
        with patch.object(codec,'ATParser',side_effect=AssertionError('stream parser in hot path')):
            self.assertEqual(candidate(command,*args),expected)


if __name__=='__main__':unittest.main()
