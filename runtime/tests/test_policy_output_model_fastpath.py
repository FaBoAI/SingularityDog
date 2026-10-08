"""CPU adapter conversion/provenance parity; no hardware or output approval."""
import json
import math
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_accel_input_hypothesis as hypotheses
from singularitydog_hw import policy_output_model as live
from singularitydog_hw.policy_observer import _tensor_row
from test_policy_output_model import torch
import test_policy_output_provenance_cache as cache_fixtures
frozen_hypothesis=cache_fixtures.frozen_hypothesis


def bits(values):
    return b''.join(struct.pack('>d',x) for x in values)


@unittest.skipIf(torch is None,'CPU PyTorch unavailable')
class CpuTargetRowFastpathTests(unittest.TestCase):
    def test_extremes_signed_zero_views_and_gradients_are_exact_and_owned(self):
        values=[0.,-0.,1.e-45,-1.e-45,torch.finfo(torch.float32).tiny,
            -torch.finfo(torch.float32).tiny,torch.finfo(torch.float32).max,
            -torch.finfo(torch.float32).max,.4,-.8,1.2,-.08]
        storage=torch.tensor([values*2],dtype=torch.float32)
        cases=[storage[:,:12],storage[:,::2],torch._neg_view(storage[:,:12]),
               storage[:,:12].clone().requires_grad_()]
        for value in cases:
            before=value.detach().clone()
            expected=_tensor_row(value,12,'target')
            actual=live._cpu_float32_target_row(value,12,'target',torch)
            self.assertEqual(bits(actual),bits(expected))
            actual[0]=123.
            self.assertEqual(bits(_tensor_row(value,12,'target')),bits(expected))
            torch.testing.assert_close(value,before,rtol=0,atol=0)

    def test_deterministic_float_bit_patterns_match_generic_conversion(self):
        # Includes all exponents, both signs, and many mantissas without RNG or
        # an assumption that decimal round trips establish bitwise equality.
        words=[0,0x80000000,1,0x80000001,0x7f7fffff,0xff7fffff]
        for i in range(1024):
            word=(i*0x9e3779b9)&0xffffffff
            if word&0x7f800000!=0x7f800000:words.append(word)
        numbers=[struct.unpack('=f',struct.pack('=I',word))[0] for word in words]
        for i in range(0,len(numbers)-12,12):
            value=torch.tensor([numbers[i:i+12]],dtype=torch.float32)
            self.assertEqual(bits(live._cpu_float32_target_row(value,12,'target',torch)),
                             bits(_tensor_row(value,12,'target')))

    def test_all_nonfinite_positions_keep_original_exception(self):
        for bad in (math.nan,math.inf,-math.inf):
            for index in range(12):
                value=torch.zeros(1,12);value[0,index]=bad
                errors=[]
                for call in (lambda:_tensor_row(value,12,'target'),
                             lambda:live._cpu_float32_target_row(value,12,'target',torch)):
                    with self.assertRaises(ValueError) as caught:call()
                    errors.append((type(caught.exception),str(caught.exception)))
                self.assertEqual(errors[0],errors[1])

    def test_cpu_abi_failure_still_precedes_conversion(self):
        for value in (torch.zeros(12),torch.zeros(1,12,dtype=torch.float64),
                      torch.zeros(1,12).to_sparse(),torch.empty(1,12,device='meta'),None):
            with patch.object(torch.Tensor,'tolist',side_effect=AssertionError('not admitted')):
                with self.assertRaisesRegex(ValueError,'CPU float32'):
                    live._cpu_float32_target_row(value,12,'target',torch)

    def test_exact_cpu_tensor_has_no_detach_or_device_dispatch(self):
        value=torch.arange(12,dtype=torch.float32).reshape(1,12)
        with patch.object(torch.Tensor,'detach',side_effect=AssertionError('detach dispatch')), \
             patch.object(torch.Tensor,'cpu',side_effect=AssertionError('device dispatch')):
            self.assertEqual(live._cpu_float32_target_row(value,12,'target',torch),
                             [float(i) for i in range(12)])

    def test_subclass_retains_generic_conversion(self):
        calls=[]
        class Custom(torch.Tensor):
            def detach(self):calls.append('detach');return super().detach()
            def cpu(self):calls.append('cpu');return super().cpu()
        value=torch.zeros(1,12).as_subclass(Custom)
        self.assertEqual(live._cpu_float32_target_row(value,12,'target',torch),[0.]*12)
        self.assertEqual(calls,['detach','cpu'])

    def test_conversion_row_errors_remain_generic_even_if_tolist_is_overridden(self):
        value=torch.zeros(1,12)
        for result in (None,[],[[0.]*12,[0.]*12],[None],[[0.]*11],[[True]+[0.]*11]):
            with patch.object(torch.Tensor,'tolist',return_value=result):
                errors=[]
                for call in (lambda:_tensor_row(value,12,'target'),
                             lambda:live._cpu_float32_target_row(value,12,'target',torch)):
                    with self.assertRaises(ValueError) as caught:call()
                    errors.append((type(caught.exception),str(caught.exception)))
                self.assertEqual(errors[0],errors[1])


@unittest.skipIf(torch is None,'CPU PyTorch unavailable')
class FrozenProvenanceGuardTests(unittest.TestCase):
    def setUp(self):
        self.fixture=cache_fixtures.FrozenProvenanceCacheTests('test_exact_frozen_type_strict_parse_occurs_once_at_setup_not_per_tick')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)

    def model(self,candidate):return self.fixture.model(candidate)

    def validate(self,model):
        f=self.fixture.f
        model.validate_inputs(f.sample,f.imu(),f.now)
        return model.last_validation['accel_input_hypothesis']

    def test_changed_frozen_raw_json_uses_current_source(self):
        candidate=frozen_hypothesis();model=self.model(candidate)
        updated=candidate.provenance();updated['changed_after_setup']=True
        object.__setattr__(candidate,'_provenance_json',json.dumps(updated))
        self.assertEqual(self.validate(model),updated)

    def test_changed_method_and_parser_keep_per_tick_behavior(self):
        candidate=frozen_hypothesis();model=self.model(candidate)
        original=hypotheses.AccelInputHypothesis.provenance
        calls=[]
        def changed(instance):
            calls.append(instance)
            result=original(instance);result['call_number']=len(calls);return result
        with patch.object(hypotheses.AccelInputHypothesis,'provenance',changed):
            self.assertEqual(self.validate(model)['call_number'],1)
            self.assertEqual(self.validate(model)['call_number'],2)
        original_parser=hypotheses.strict_json
        with patch.object(hypotheses,'strict_json',wraps=original_parser) as parser:
            self.validate(model);self.validate(model)
            self.assertEqual(parser.call_count,2)

    def test_changed_parser_error_is_not_hidden_by_cached_provenance(self):
        model=self.model(frozen_hypothesis())
        with patch.object(hypotheses,'strict_json',side_effect=ValueError('changed parser')):
            with self.assertRaisesRegex(ValueError,'changed parser'):self.validate(model)
        self.assertEqual(model.calls,0)

    def test_changed_class_binding_uses_current_provenance(self):
        candidate=frozen_hypothesis();model=self.model(candidate)
        with patch.object(hypotheses,'AccelInputHypothesis',object), \
             patch.object(hypotheses,'strict_json',wraps=hypotheses.strict_json) as parser:
            self.assertEqual(self.validate(model),candidate.provenance())
            self.assertEqual(parser.call_count,2)

    def test_frozen_proof_is_checked_before_provenance_or_inference(self):
        candidate=frozen_hypothesis();model=self.model(candidate)
        object.__setattr__(candidate,'_proof',None)
        count=len(model.policy.inputs)
        with self.assertRaisesRegex(ValueError,'loader proof'):self.validate(model)
        self.assertEqual(len(model.policy.inputs),count)
        self.assertEqual(model.calls,0)

    def test_before_setup_changed_method_is_dynamic_not_frozen(self):
        candidate=frozen_hypothesis();original=hypotheses.AccelInputHypothesis.provenance
        calls=[]
        def changed(instance):
            calls.append(instance)
            result=original(instance);result['call_number']=len(calls);return result
        with patch.object(hypotheses.AccelInputHypothesis,'provenance',changed):
            model=self.model(candidate)
            self.assertIsNone(model._accel_provenance_cache)
            self.assertEqual(calls,[])
            self.assertEqual(self.validate(model)['call_number'],1)
            self.assertEqual(self.validate(model)['call_number'],2)

    def test_before_setup_changed_parser_errors_keep_per_tick_timing(self):
        candidate=frozen_hypothesis()
        with patch.object(hypotheses,'strict_json',side_effect=ValueError('dynamic parser')):
            model=self.model(candidate)
            self.assertIsNone(model._accel_provenance_cache)
            with self.assertRaisesRegex(ValueError,'dynamic parser'):self.validate(model)
        self.assertEqual(model.calls,0)

    def test_before_setup_changed_class_has_no_cache(self):
        candidate=frozen_hypothesis()
        with patch.object(hypotheses,'AccelInputHypothesis',object):
            model=self.model(candidate)
            self.assertIsNone(model._accel_provenance_cache)
            self.assertEqual(self.validate(model),candidate.provenance())

    def test_literal_and_bounded_fallback_copies_keep_bits_and_no_aliases(self):
        for use_fallback in (False,True):
            candidate=frozen_hypothesis()
            if use_fallback:
                data=candidate.provenance();data['large_metadata']='x'*4097
                object.__setattr__(candidate,'_provenance_json',json.dumps(data))
            model=self.model(candidate)
            self.assertEqual(model._accel_provenance_cache[-1] is None,use_fallback)
            expected=candidate.provenance();a=self.validate(model);b=self.validate(model)
            def check(left,right):
                self.assertIs(type(left),type(right))
                if type(left) is dict:
                    self.assertEqual(list(left),list(right));self.assertIsNot(left,right)
                    for key in left:check(left[key],right[key])
                elif type(left) is list:
                    self.assertEqual(len(left),len(right));self.assertIsNot(left,right)
                    for x,y in zip(left,right):check(x,y)
                elif type(left) is float:self.assertEqual(bits([left]),bits([right]))
                else:self.assertEqual(left,right)
            check(expected,a);check(a,b)
            a['nested']['list'][0]=999;a['nested']['source_sha256'].clear()
            a['R_body_from_sensor'][0].clear()
            self.assertEqual(b,expected)
            self.assertEqual(self.validate(model),expected)

    def test_unavailable_serialization_retains_tree_copy_at_tick(self):
        candidate=frozen_hypothesis()
        with patch.object(live.marshal,'dumps',side_effect=ValueError('unusual metadata')), \
             patch.object(live,'_frozen_json_literal_factory',return_value=None):
            model=self.model(candidate)
        self.assertIsNone(model._accel_provenance_cache[-2])
        self.assertIsNone(model._accel_provenance_cache[-1])
        a=self.validate(model);b=self.validate(model)
        a['nested']['list'][0]=999
        self.assertEqual(b,candidate.provenance())


if __name__=='__main__':unittest.main()
