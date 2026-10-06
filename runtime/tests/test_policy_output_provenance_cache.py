"""Setup-only frozen provenance parsing; real CPU tensors, synthetic sensors only."""
import copy
from dataclasses import replace
import json
import math
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_accel_input_hypothesis as hypotheses
from singularitydog_hw import policy_output_model as live
import test_policy_output_model as model_fixtures
TensorPolicy=model_fixtures.TensorPolicy
torch=model_fixtures.torch

REFERENCE = {'path': '/synthetic/not-opened/hypothesis.json', 'sha256': 'a'*64}


def frozen_hypothesis(**changes):
    """Consumer fixture only; loader/raw-capture authentication has separate tests."""
    data = {'kind':hypotheses.SCHEMA, 'hypothesis_sha256':'a'*64,
        'formal_calibration_approved':False, 'grants_motor_output':False,
        'absolute_orientation_error_bound_rad':None,
        'R_body_from_sensor':[[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]],
        'bias_sensor_m_s2':[.03,-.02,-.1], 'scale_sensor':[1.,1.,1.],
        'nested':{'list':[1,1.0,-0.0,True,None,'方向'], 'source_sha256':{'source.py':'b'*64}}}
    args = dict(bias_m_s2=(.03,-.02,-.1), scale=(1.,1.,1.),
        raw_norm_min_m_s2=9.,raw_norm_max_m_s2=10.8,
        corrected_norm_min_m_s2=9.,corrected_norm_max_m_s2=10.2,
        reference_sha256='a'*64,candidate_sha256='c'*64,manifest_sha256='d'*64,
        _provenance_json=json.dumps(data,ensure_ascii=False,allow_nan=False),
        _proof=hypotheses._VALIDATED)
    args.update(changes)
    return hypotheses.AccelInputHypothesis(**args)


class UncachedCorrection:
    """Old per-call provenance behavior using exactly the same correct() method."""
    def __init__(self, frozen): self.frozen=frozen; self.provenance_calls=0
    def correct(self, raw): return self.frozen.correct(raw)
    def provenance(self):
        self.provenance_calls+=1
        return self.frozen.provenance()


class JsonTreeCopyTests(unittest.TestCase):
    def test_all_json_types_float_bits_order_and_mutable_containers_are_preserved(self):
        source=frozen_hypothesis().provenance()
        result=live._copy_json_tree(source)
        def check(left,right):
            self.assertIs(type(left),type(right))
            if type(left)is dict:
                self.assertIsNot(left,right);self.assertEqual(list(left),list(right))
                for key in left:check(left[key],right[key])
            elif type(left)is list:
                self.assertIsNot(left,right);self.assertEqual(len(left),len(right))
                for a,b in zip(left,right):check(a,b)
            elif type(left)is float:self.assertEqual(struct.pack('>d',left),struct.pack('>d',right))
            else:self.assertEqual(left,right)
        check(source,result)
        result['nested']['list'][0]=999
        result['R_body_from_sensor'][0].clear()
        self.assertEqual(source,frozen_hypothesis().provenance())


@unittest.skipIf(torch is None, 'CPU PyTorch unavailable')
class FrozenProvenanceCacheTests(unittest.TestCase):
    def setUp(self):
        self.f=model_fixtures.PolicyOutputModelTests('test_warmup_then_one_reset_preserves_fresh_state_across_live_ticks')
        self.f.setUp();self.addCleanup(self.f.doCleanups)

    def model(self, correction):
        policy=TensorPolicy()
        with patch.object(live,'accel_input_hypothesis_settings',return_value=REFERENCE), \
             patch.object(hypotheses,'load_accel_input_hypothesis',return_value=correction):
            return live.LivePolicyModel(self.f.profile,policy=policy,torch_module=torch)

    def test_exact_frozen_type_strict_parse_occurs_once_at_setup_not_per_tick(self):
        candidate=frozen_hypothesis();original=hypotheses.strict_json
        with patch.object(hypotheses,'strict_json',wraps=original) as parse:
            model=self.model(candidate)
            self.assertEqual(parse.call_count,1)
            for tick in range(3):
                now=self.f.now+tick*20_000_000
                model(self.f.sample,self.f.imu(now),now)
            self.assertEqual(parse.call_count,1)
        self.assertIs(model._accel_provenance_source,candidate)
        self.assertEqual(model.calls,3)

    def test_every_tick_still_calls_correct_including_original_proof_norm_guards(self):
        candidate=frozen_hypothesis(corrected_norm_max_m_s2=9.75);model=self.model(candidate);calls=[]
        original=hypotheses.AccelInputHypothesis.correct
        def checked(instance,raw):
            calls.append((instance,tuple(raw)))
            return original(instance,raw)
        with patch.object(hypotheses.AccelInputHypothesis,'correct',checked):
            model.validate_inputs(self.f.sample,self.f.imu(),self.f.now)
            source=self.f.imu();source['accel_m_s2']=[0.,0.,-9.9]
            with self.assertRaises(ValueError):model.validate_inputs(self.f.sample,source,self.f.now)
        self.assertEqual(len(calls),2)
        self.assertTrue(all(instance is candidate for instance,_ in calls))

    def test_snapshot_mutation_cannot_change_cache_other_history_or_next_inputs(self):
        candidate=frozen_hypothesis();model=self.model(candidate)
        expected=candidate.provenance();snapshots=[];inputs=[]
        for tick in range(3):
            now=self.f.now+tick*20_000_000
            model(self.f.sample,self.f.imu(now),now)
            snapshots.append(model.last_validation)
            inputs.append(tuple(x.clone() for x in model.tensors))
        snapshots[1]['accel_input_hypothesis']['R_body_from_sensor'][0][0]=999
        snapshots[1]['accel_input_hypothesis']['bias_sensor_m_s2'][0]=-100
        snapshots[1]['accel_input_hypothesis']['nested']['source_sha256'].clear()
        snapshots[1]['accel_input_hypothesis']['nested']['list'].append({'x':[]})
        self.assertEqual(snapshots[0]['accel_input_hypothesis'],expected)
        self.assertEqual(snapshots[2]['accel_input_hypothesis'],expected)
        self.assertEqual(model._accel_provenance_snapshot,expected)
        self.assertEqual(candidate.provenance(),expected)
        now=self.f.now+60_000_000;model(self.f.sample,self.f.imu(now),now)
        self.assertEqual(model.last_validation['accel_input_hypothesis'],expected)
        for old,new in zip(inputs[0],model.tensors):torch.testing.assert_close(old,new,rtol=0,atol=0)

    def test_cached_and_original_per_call_behavior_have_identical_targets_tensors_and_state(self):
        candidate=frozen_hypothesis();cached=self.model(candidate);uncached=self.model(UncachedCorrection(candidate))
        for tick in range(5):
            now=self.f.now+tick*20_000_000;source=self.f.imu(now)
            source['accel_m_s2'][0]=tick*.001
            self.assertEqual(cached(self.f.sample,source,now),uncached(self.f.sample,source,now))
            self.assertEqual(cached.last_validation,uncached.last_validation)
            for a,b in zip(cached.policy.inputs[-1],uncached.policy.inputs[-1]):torch.testing.assert_close(a,b,rtol=0,atol=0)
            self.assertEqual(cached.policy.state,uncached.policy.state)
            self.assertEqual((cached.last_tick_ns,cached.last_imu_ns,cached.calls),
                             (uncached.last_tick_ns,uncached.last_imu_ns,uncached.calls))

    def test_raw_frame_timestamp_finite_norm_tilt_gyro_joint_rejections_match(self):
        candidate=frozen_hypothesis();cases=[]
        for field,value in [('frame','body'),('accel_bias_subtracted',True),
                ('read_started_monotonic_ns',self.f.now-100_000_000),
                ('read_finished_monotonic_ns',self.f.now+1),
                ('accel_m_s2',[0.,0.,float('nan')]),('accel_m_s2',[0.,0.,-8.]),
                ('accel_m_s2',[0.,0.,-10.8]),('accel_m_s2',[3.,0.,-9.4]),
                ('gyro_rad_s',[10.,0.,0.])]:
            source=self.f.imu();source[field]=value;cases.append((field,source,self.f.sample))
        bad=replace(self.f.sample,q_model_rad=(100.,)+self.f.sample.q_model_rad[1:])
        cases.append(('joint',self.f.imu(),bad))
        for label,source,sample in cases:
            with self.subTest(label=label,source=source):
                errors=[]
                for correction in (candidate,UncachedCorrection(candidate)):
                    model=self.model(correction)
                    with self.assertRaises(ValueError) as error:model.validate_inputs(sample,source,self.f.now)
                    errors.append((type(error.exception),str(error.exception)))
                    self.assertEqual(model.calls,0)
                self.assertEqual(errors[0],errors[1])

    def test_reused_tick_and_locomotion_override_are_still_rejected(self):
        for correction in (frozen_hypothesis(),UncachedCorrection(frozen_hypothesis())):
            model=self.model(correction);model(self.f.sample,self.f.imu(),self.f.now)
            with self.assertRaisesRegex(ValueError,'reused'):model(self.f.sample,self.f.imu(),self.f.now)
            now=self.f.now+20_000_000
            with self.assertRaisesRegex(ValueError,'boxed-only'):
                model(self.f.sample,self.f.imu(now),now,command_override=[0.,0.,0.])

    def test_custom_dynamic_correction_keeps_per_call_provenance(self):
        class Dynamic(UncachedCorrection):
            def provenance(self):
                result=super().provenance();result['dynamic_call']=self.provenance_calls
                return result
        custom=Dynamic(frozen_hypothesis());model=self.model(custom)
        self.assertIsNone(model._accel_provenance_snapshot)
        self.assertEqual(custom.provenance_calls,0)
        for n in (1,2):
            model.validate_inputs(self.f.sample,self.f.imu(),self.f.now)
            self.assertEqual(model.last_validation['accel_input_hypothesis']['dynamic_call'],n)

    def test_frozen_subclass_keeps_dynamic_provenance_behavior(self):
        class Subclass(hypotheses.AccelInputHypothesis):
            def provenance(self):
                result=super().provenance();result['extra']='subclass'
                return result
        original=frozen_hypothesis();sub=Subclass(**vars(original));model=self.model(sub)
        self.assertIsNone(model._accel_provenance_source)
        with patch.object(hypotheses,'strict_json',wraps=hypotheses.strict_json) as parse:
            for _ in range(2):model.validate_inputs(self.f.sample,self.f.imu(),self.f.now)
            self.assertEqual(parse.call_count,2)
        self.assertEqual(model.last_validation['accel_input_hypothesis']['extra'],'subclass')

    def test_replaced_correction_cannot_receive_previous_objects_cached_provenance(self):
        model=self.model(frozen_hypothesis());custom=UncachedCorrection(frozen_hypothesis())
        model.accel_calibration=custom
        model.validate_inputs(self.f.sample,self.f.imu(),self.f.now)
        self.assertEqual(custom.provenance_calls,1)

    def test_invalid_loader_proof_rejected_before_cache_or_model_start(self):
        forged=replace(frozen_hypothesis(),_proof=None)
        with self.assertRaisesRegex(ValueError,'loader proof'):self.model(forged)

    def test_unselected_path_has_no_cache_and_no_hypothesis_parse(self):
        with patch.object(hypotheses,'strict_json',side_effect=AssertionError('not selected')):
            model=live.LivePolicyModel(self.f.profile,policy=TensorPolicy(),torch_module=torch)
            model(self.f.sample,self.f.imu(),self.f.now)
        self.assertIsNone(model._accel_provenance_source)
        self.assertNotIn('accel_input_hypothesis',model.last_validation)


if __name__=='__main__':unittest.main()
