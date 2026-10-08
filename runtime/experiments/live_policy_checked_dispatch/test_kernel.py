from pathlib import Path
import struct
import sys
import unittest
import os

RUNTIME=Path(os.environ['LIVE_COMPONENT_RUNTIME'])
SOURCE_SHA=os.environ['LIVE_COMPONENT_ORIGINAL_SOURCE_SHA256']
sys.path.insert(0,str(RUNTIME))
import torch
from cpu_kernel import GuardedCpuPolicyKernel
from singularitydog_hw.policy_shadow import CAN_ORDER


class TensorPolicy:
    def __init__(self):
        self.total_calls=0;self.reset_calls=0;self.corrupt=None
    def reset(self,ids):self.reset_calls+=1
    def __call__(self,*inputs):
        self.total_calls+=1
        self.last_actor_output=torch.zeros((1,12),dtype=torch.float32)
        self.last_observation=torch.zeros((1,74),dtype=torch.float32)
        target=torch.tensor([[-0.,.4,-.8]*4],dtype=torch.float32)
        if self.corrupt=='target_nan':target[0,0]=float('nan')
        if self.corrupt=='target_dtype':target=target.to(torch.float64)
        if self.corrupt=='target_range':target[0,2]=0.
        if self.corrupt=='actor_inf':self.last_actor_output[0,3]=float('inf')
        if self.corrupt=='observation_nan':self.last_observation[0,73]=float('nan')
        if self.corrupt=='observation_shape':self.last_observation=torch.zeros((1,73),dtype=torch.float32)
        return target


class KernelBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.policy=TensorPolicy();self.kernel=GuardedCpuPolicyKernel(self.policy,torch,original_source_sha256=SOURCE_SHA)
        self.values=([0.,0.,0.],[0.,0.,-1.],[0.,0.,0.],[-.28,.4,-.8]*4,[0.]*12,[0.]*12)
    def ready(self):
        self.kernel.pre_pin_warmup();self.kernel.post_pin_prime();self.kernel.finish_startup()
    def test_original_ten_ten_reset_and_owned_storage(self):
        ptrs=[t.data_ptr() for t in self.kernel.tensors]
        self.ready()
        self.assertEqual(self.policy.total_calls,20);self.assertEqual(self.policy.reset_calls,1)
        self.assertEqual(ptrs,[t.data_ptr() for t in self.kernel.tensors])
        self.assertEqual(self.kernel.calls,0)
    def test_startup_order_rejects_unprimed_call(self):
        with self.assertRaisesRegex(ValueError,'reset is incomplete'):self.kernel(self.values)
        with self.assertRaisesRegex(ValueError,'pre-pin warmup'):self.kernel.post_pin_prime()
    def test_replaced_input_storage_rejects_prime(self):
        self.kernel.pre_pin_warmup()
        self.kernel.tensors=(self.kernel.tensors[0].clone(),*self.kernel.tensors[1:])
        with self.assertRaisesRegex(ValueError,'owned reused'):self.kernel.post_pin_prime()
    def test_original_shape_dtype_finite_and_range_rejections(self):
        self.ready()
        for corrupt in ('target_nan','target_dtype','target_range','actor_inf','observation_nan','observation_shape'):
            with self.subTest(corrupt=corrupt):
                self.policy.corrupt=corrupt
                with self.assertRaises((ValueError,RuntimeError)):self.kernel(self.values)
                self.assertEqual(self.kernel.calls,0)
    def test_can_permutation_preserves_signed_float_bits(self):
        self.ready();result=self.kernel(self.values);expected=[0.]*12
        for can,value in zip(CAN_ORDER,[-0.,.4,-.8]*4):expected[can-1]=float(torch.tensor(value,dtype=torch.float32).item())
        self.assertEqual(struct.pack('>12d',*result),struct.pack('>12d',*expected))
        self.assertTrue(all(self.kernel.provenance[k] is False for k in
            ('output_allowed','approved_for_runtime','active_controller_qualification','hardware_opened')))


if __name__=='__main__':unittest.main()
