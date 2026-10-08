"""Selected startup dispatch tests with CPU tensors/mocked checked operator; no CAN."""
from array import array
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import torch
from singularitydog_hw import policy_observer_replay as replay
from singularitydog_hw import policy_output_model as live
from singularitydog_hw import policy_checked_dispatch as checked
from singularitydog_hw import policy_shadow as shadow

class Policy:
    def __init__(self):
        self._c=object();self.calls=0;self.direct=0;self.selected=0;self.resets=[];self.via_checked=False
        self.last_actor_output=torch.zeros((1,12),dtype=torch.float32)
        self.last_observation=torch.zeros((1,74),dtype=torch.float32)
    def __call__(self,*tensors):
        self.calls+=1
        if self.via_checked:self.selected+=1
        else:self.direct+=1
        self.last_actor_output.fill_(self.calls);self.last_observation.fill_(self.calls)
        return tensors[3].clone()
    def reset(self,ids):
        self.resets.append(ids.tolist());self.last_actor_output.zero_();self.last_observation.zero_()

class CheckedWarmupTests(unittest.TestCase):
    def setup_fixture(self):
        policy=Policy();wrapper=SimpleNamespace(inner=policy);calls=[]
        def dispatch(selected,tensors):
            self.assertIs(selected,wrapper);calls.append(tuple(tensors));policy.via_checked=True
            try:row=policy(*tensors)[0].tolist()
            finally:policy.via_checked=False
            values=[0.]*12
            for can,value in zip(shadow.CAN_ORDER,row):values[can-1]=value
            return tuple(values)
        return policy,wrapper,calls,dispatch

    def owned(self):
        buffers=tuple(array('f',[0.]*count) for count in (3,3,3,12,12,12))
        return buffers,tuple(torch.frombuffer(value,dtype=torch.float32).reshape(1,len(value)) for value in buffers)

    def test_selected_exact_ten_plus_ten_actual_persistent_tensors_and_single_reset(self):
        policy,wrapper,calls,dispatch=self.setup_fixture();buffers,tensors=self.owned()
        value=object.__new__(live.LivePolicyModel)
        value.policy=policy;value._checked_dispatch_wrapper=wrapper;value.torch=torch
        value.profile={'h_hypothesis':0.};value._startup_stage='pending_warmup'
        value.input_buffers=buffers;value.tensors=tensors
        with patch.object(checked,'checked_call',side_effect=dispatch):
            value.pre_pin_warmup();self.assertEqual(len(calls),10)
            value.post_pin_prime();self.assertEqual(len(calls),20)
            value.finish_startup()
        self.assertEqual((policy.calls,policy.selected,policy.direct),(20,20,0))
        self.assertTrue(all(all(actual is expected for actual,expected in zip(row,tensors)) for row in calls[10:]))
        self.assertEqual(policy.resets,[[0]]);self.assertEqual(value._startup_stage,'ready')
        self.assertEqual(policy.last_actor_output.count_nonzero().item(),0)
        self.assertEqual(policy.last_observation.count_nonzero().item(),0)
        self.assertEqual(tensors[3][0].tolist(),[0.,.4000000059604645,-.800000011920929]*4)

    def test_none_exact_ordinary_ten_plus_ten_no_checked_dispatch(self):
        policy=Policy();_,tensors=self.owned()
        with patch.object(checked,'checked_call',side_effect=AssertionError('must remain unselected')):
            replay.warmup_policy(policy,torch,0.,10)
            replay.warmup_policy(policy,torch,0.,10,input_tensors=tensors,checked_dispatch_wrapper=None)
        self.assertEqual((policy.calls,policy.direct,policy.selected),(20,20,0))

    def test_selected_same_input_signature_and_original_model_order_tensor_check(self):
        policy,wrapper,calls,dispatch=self.setup_fixture()
        with patch.object(checked,'checked_call',side_effect=dispatch),patch.object(replay.observer,'_tensor_row',wraps=replay.observer._tensor_row) as validation:
            replay.warmup_policy(policy,torch,.0,1,checked_dispatch_wrapper=wrapper)
        self.assertEqual(tuple(tuple(t.shape) for t in calls[0]),((1,3),(1,3),(1,3),(1,12),(1,12),(1,12)))
        self.assertEqual([call.args[2] for call in validation.call_args_list],['warmup target','warmup actor','warmup observation'])
        self.assertEqual(validation.call_args_list[0].args[0][0].tolist(),calls[0][3][0].tolist())

    def test_foreign_inner_rejects_before_model_or_checked_call(self):
        policy,wrapper,_,_=self.setup_fixture();wrapper.inner=Policy()
        with patch.object(checked,'checked_call') as dispatch,self.assertRaisesRegex(ValueError,'exact original inner'):
            replay.warmup_policy(policy,torch,0,10,checked_dispatch_wrapper=wrapper)
        dispatch.assert_not_called();self.assertEqual(policy.calls,0)

    def test_invalid_count_and_persistent_signature_reject_before_dispatch(self):
        policy,wrapper,_,_=self.setup_fixture()
        with patch.object(checked,'checked_call') as dispatch:
            for count in (0,True,101):
                with self.assertRaises(ValueError):replay.warmup_policy(policy,torch,0,count,checked_dispatch_wrapper=wrapper)
            with self.assertRaisesRegex(ValueError,'six float32'):
                replay.warmup_policy(policy,torch,0,10,input_tensors=(torch.zeros(1,3),),checked_dispatch_wrapper=wrapper)
        dispatch.assert_not_called();self.assertEqual(policy.calls,0)

    def test_checked_warm_error_fatal_and_pre_warm_stage_not_completed(self):
        policy,wrapper,_,_=self.setup_fixture();value=object.__new__(live.LivePolicyModel)
        value.policy=policy;value._checked_dispatch_wrapper=wrapper;value.torch=torch
        value.profile={'h_hypothesis':0.};value._startup_stage='pending_warmup'
        primary=ValueError('checked target range rejected')
        with patch.object(checked,'checked_call',side_effect=primary) as dispatch:
            with self.assertRaises(ValueError) as caught:value.pre_pin_warmup()
        self.assertIs(caught.exception,primary);self.assertEqual(dispatch.call_count,1)
        self.assertEqual(value._startup_stage,'pending_warmup');self.assertEqual(policy.resets,[])
        with self.assertRaises(RuntimeError):value.finish_startup()

    def test_checked_post_prime_error_does_not_reset_or_mark_ready(self):
        policy,wrapper,calls,dispatch=self.setup_fixture();buffers,tensors=self.owned()
        value=object.__new__(live.LivePolicyModel)
        value.policy=policy;value._checked_dispatch_wrapper=wrapper;value.torch=torch
        value.profile={'h_hypothesis':0.};value._startup_stage='pending_warmup';value.input_buffers=buffers;value.tensors=tensors
        with patch.object(checked,'checked_call',side_effect=dispatch):value.pre_pin_warmup()
        with patch.object(checked,'checked_call',side_effect=RuntimeError('original wrapper failure')):
            with self.assertRaisesRegex(RuntimeError,'original wrapper failure'):value.post_pin_prime()
        self.assertEqual(value._startup_stage,'warmed');self.assertEqual(policy.resets,[])
        self.assertEqual(len(calls),10)

    def test_selected_warm_source_is_mandatory_in_own_model_proof(self):
        self.assertIn('singularitydog_hw/policy_observer_replay.py',checked.source_paths())

if __name__=='__main__':unittest.main(verbosity=2)
