"""Adversarial exact-bit equivalence of private tiny arithmetic/guard kernels."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import unittest

HERE=Path(__file__).resolve().parent
ARGS=None
torch=None
OPS=None

def bits(t):return t.detach().contiguous().reshape(-1).view(torch.uint8).numpy().tobytes()

class Components(unittest.TestCase):
    def values(self):
        ref=torch.tensor([[.2,.3,-.4]*4],dtype=torch.float64)
        activation=torch.tensor([.3],dtype=torch.float64)
        scale=torch.tensor([.27,.4,.5]*4,dtype=torch.float64)
        action=torch.tensor([[.3,-.4,.7]*4],dtype=torch.float64)
        lower=torch.tensor([-.5,-.9,-2.2]*4,dtype=torch.float64)
        upper=torch.tensor([.5,1.2,-.08]*4,dtype=torch.float64)
        return ref,activation,scale,action,lower,upper

    def compare(self,args):
        before=[bits(x) for x in args]
        one=OPS.reference(*args);two=OPS.candidate(*args)
        self.assertEqual([bits(x) for x in one],[bits(x) for x in two])
        self.assertEqual(before,[bits(x) for x in args])
        self.assertEqual([tuple(x.shape) for x in one],[tuple(x.shape) for x in two])
        self.assertTrue(all(a.data_ptr()!=b.data_ptr() for a,b in zip(one,two)))

    def test_seeded_random_4096(self):
        generator=torch.Generator().manual_seed(771304)
        with torch.inference_mode():
            for _ in range(4096):
                ref,active,scale,action,lo,hi=self.values()
                ref.copy_(torch.randn((1,12),generator=generator,dtype=torch.float64)*.5)
                active.copy_(torch.rand((1,),generator=generator,dtype=torch.float64)*2-1)
                action.copy_(torch.randn((1,12),generator=generator,dtype=torch.float64)*3)
                scale.copy_(torch.randn((12,),generator=generator,dtype=torch.float64))
                self.compare((ref,active,scale,action,lo,hi))

    def test_threshold_nan_infinity_signed_zero(self):
        special=[0.,-0.,1e-15,-1e-15,math.nextafter(1e-15,math.inf),
                 math.nextafter(-1e-15,-math.inf),math.inf,-math.inf,math.nan,
                 1e-300,-1e-300,1e300,-1e300]
        with torch.inference_mode():
            for position in range(12):
                for value in special:
                    args=list(self.values());args[1].fill_(1);args[2].fill_(1)
                    args[3][0,position]=value
                    self.compare(args)
                for field in (0,4,5):
                    for value in special:
                        args=list(self.values())
                        args[field].reshape(-1)[position]=value
                        self.compare(args)

    def test_equal_signed_zero_min_keeps_first(self):
        with torch.inference_mode():
            for zero in (0.,-0.):
                args=list(self.values());args[1].fill_(1);args[2].fill_(1);args[3].fill_(1)
                args[5].copy_(args[0][0]);args[5][0]=zero;args[0][0,0]=0.
                self.compare(args)

    def test_noncontiguous_float32_and_noninference_fallback(self):
        for dtype in (torch.float32,torch.float64):
            args=[x.to(dtype) for x in self.values()]
            with torch.inference_mode():self.compare(args)
            base=torch.empty((12,2),dtype=dtype)
            base[:,0]=args[0].reshape(12)
            args[0]=base[:,0].reshape(1,12)
            with torch.inference_mode():self.compare(args)
            self.compare(args)

    def guard_args(self):
        q=torch.zeros((1,12),dtype=torch.float64)
        result=q.clone();lo=torch.full((12,),-2.,dtype=torch.float64)
        hi=torch.full((12,),2.,dtype=torch.float64)
        feet=torch.randn((1,4,3),dtype=torch.float64)
        lift=torch.randn((1,4),dtype=torch.float64)*.01
        up=torch.tensor([[[.1,.2,.3]]],dtype=torch.float64)
        roundtrip=feet+lift.unsqueeze(2)*up
        return q,result,lo,hi,roundtrip,feet,lift,up

    def error(self,fn,args):
        try:fn(*args)
        except RuntimeError as e:return str(e).split('\n')[0]
        return None

    def test_all_error_precedence_and_boundary(self):
        with torch.inference_mode():
            self.assertIsNone(self.error(OPS.candidate_checks,self.guard_args()))
            for i in range(12):
                for where in ('margin','clip','closure','all'):
                    args=list(self.guard_args())
                    if where in ('margin','all'):args[0][0,i]=3.
                    if where in ('clip','all'):args[1][0,i]=.5
                    if where in ('closure','all'):args[4].reshape(-1)[i]=30.
                    self.assertEqual(self.error(OPS.reference_checks,args),self.error(OPS.candidate_checks,args))
            for i in range(12):
                for value in (math.nan,math.inf,-math.inf,0.,-0.):
                    for field in (0,1,2,3,4,5):
                        args=list(self.guard_args());args[field].reshape(-1)[i]=value
                        self.assertEqual(self.error(OPS.reference_checks,args),self.error(OPS.candidate_checks,args))
            for threshold,field in ((1e-12,1),(1e-9,4)):
                for val in (threshold,math.nextafter(threshold,math.inf),math.nextafter(threshold,-math.inf)):
                    args=list(self.guard_args())
                    if field==1:args[1][0,0]=val
                    else:args[4]=args[5]+args[6].unsqueeze(2)*args[7];args[4].reshape(-1)[0]+=val
                    self.assertEqual(self.error(OPS.reference_checks,args),self.error(OPS.candidate_checks,args))

    def test_guard_fallback_and_alias(self):
        for dtype in (torch.float32,torch.float64):
            args=[x.to(dtype) for x in self.guard_args()]
            before=[bits(x) for x in args]
            self.assertEqual(self.error(OPS.reference_checks,args),self.error(OPS.candidate_checks,args))
            self.assertEqual(before,[bits(x) for x in args])
            with torch.inference_mode():
                self.assertEqual(self.error(OPS.reference_checks,args),self.error(OPS.candidate_checks,args))

def main():
    global torch,OPS,ARGS
    p=argparse.ArgumentParser(allow_abbrev=False);p.add_argument('--build-dir',required=True)
    ARGS=p.parse_args();d=Path(ARGS.build_dir)
    record=json.loads((d/'build-record.json').read_text());lib=d/'target_envelope_r11.so'
    if hashlib.sha256(lib.read_bytes()).hexdigest()!=record['library_sha256']:raise ValueError('Build bytes differ')
    import torch as torch_module
    torch=torch_module;torch.set_num_threads(1);torch.set_num_interop_threads(1)
    torch.ops.load_library(str(lib));OPS=torch.ops.sd_target_envelope_components_fileonly_r11
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Components)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    summary=dict(status='PASS' if result.wasSuccessful() else 'FAIL',tests=result.testsRun,
                 errors=len(result.errors),failures=len(result.failures),torch_version=torch.__version__,
                 component_bit_parity_only=True,hardware_opened=False,output_allowed=False,
                 approved_for_runtime=False,whole_loop_performance_measured=False,
                 source_sha256=record['source_sha256'],library_sha256=record['library_sha256'])
    (d/'component-test-report.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary));return 0 if result.wasSuccessful() else 1

if __name__=='__main__':raise SystemExit(main())
