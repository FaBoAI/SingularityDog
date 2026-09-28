"""Actual CPU tensors, synthetic policy/sensors, no device or real model loading."""
from array import array
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

try:
    import torch
except ImportError:
    torch = None

from singularitydog_hw.policy_motion_envelope import MotionSample
from singularitydog_hw.policy_output_model import LivePolicyModel, _finite_cpu_float32_row
from singularitydog_hw.policy_observer import _tensor_row
from singularitydog_hw.policy_shadow import CAN_ORDER
from singularitydog_hw.policy_shadow import LOWER, UPPER
from test_policy_live_profile import synthetic_fixture


class TensorPolicy:
    def __init__(self):
        self.state = 0; self.resets = 0; self.inputs = []; self.state_before_calls = []
        self.input_pointers = []
        self.target = [0., .4, -.8]*4
        self.bad_actor = False; self.bad_observation = False

    def reset(self, ids):
        assert ids.tolist() == [0]
        self.state = 0; self.resets += 1

    def __call__(self, *values):
        self.state_before_calls.append(self.state); self.state += 1
        self.inputs.append(tuple(v.clone() for v in values))
        self.input_pointers.append(tuple(v.data_ptr() for v in values))
        self.last_actor_output = torch.zeros((1, 12), dtype=torch.float32)
        self.last_observation = torch.zeros((1, 74), dtype=torch.float32)
        if self.bad_actor: self.last_actor_output[0, 0] = float('nan')
        if self.bad_observation: self.last_observation[0, 0] = float('inf')
        return torch.tensor([self.target], dtype=torch.float32)


@unittest.skipIf(torch is None, 'CPU PyTorch is unavailable')
class FiniteTelemetryTensorTests(unittest.TestCase):
    def test_every_nan_inf_position_rejected_without_mutating_tensor(self):
        for count in (12,74):
            for bad in (float('nan'),float('inf'),float('-inf')):
                for index in range(count):
                    value=torch.linspace(-1.,1.,count).reshape(1,count)
                    value[0,index]=bad
                    before=value.clone()
                    with self.subTest(count=count,bad=bad,index=index):
                        with self.assertRaises(ValueError):
                            _finite_cpu_float32_row(value,count,'telemetry',torch)
                        with self.assertRaises(ValueError):
                            _tensor_row(value,count,'legacy')
                        torch.testing.assert_close(value,before,equal_nan=True,rtol=0,atol=0)

    def test_finite_extremes_subnormals_noncontiguous_and_negative_views(self):
        for count in (12,74):
            value=torch.zeros(1,count*2)[:,::2]
            value[0,:6]=torch.tensor([torch.finfo(torch.float32).max,-torch.finfo(torch.float32).max,
                                      torch.finfo(torch.float32).tiny,1.e-45,0.,-0.])
            for item in (value,torch._neg_view(value)):
                before=item.clone()
                self.assertIsNone(_finite_cpu_float32_row(item,count,'telemetry',torch))
                self.assertEqual(len(_tensor_row(item,count,'legacy')),count)
                torch.testing.assert_close(item,before,rtol=0,atol=0)

    def test_shape_dtype_layout_and_non_tensor_rejected(self):
        wrong=[torch.zeros(12),torch.zeros(12,1),torch.zeros(2,12),torch.zeros(1,11),torch.zeros(1,0),
               torch.zeros(1,12,dtype=torch.float64),torch.zeros(1,12,dtype=torch.float16),
               torch.zeros(1,12,dtype=torch.int64),torch.zeros(1,12,dtype=torch.bool),
               torch.zeros(1,12,dtype=torch.complex64),torch.zeros(1,12).to_sparse(),
               torch.empty(1,12,device='meta'),[[0.]*12],None]
        for value in wrong:
            with self.subTest(value=repr(value)),self.assertRaisesRegex(ValueError,'CPU float32'):
                _finite_cpu_float32_row(value,12,'telemetry',torch)

    def test_finite_check_never_calls_tensor_tolist_or_moves_devices(self):
        from unittest.mock import patch
        value=torch.randn(1,74)
        with patch.object(torch.Tensor,'tolist',side_effect=AssertionError('Unexpected tensor copy')), \
             patch.object(torch.Tensor,'cpu',side_effect=AssertionError('Unexpected device transfer')):
            _finite_cpu_float32_row(value,74,'telemetry',torch)


@unittest.skipIf(torch is None, 'CPU PyTorch is unavailable')
class PolicyOutputModelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.profile, documents, _ = synthetic_fixture(self.base)
        self.profile['output_allowed'] = True
        self.bias = [.01,.02,.03]
        documents['bias']['gyro_bias_candidate_rad_s'] = list(self.bias)
        documents['bias']['captures']['a']['gyro_mean_rad_s'] = list(self.bias)
        raw = json.dumps(documents['bias']).encode(); (self.base/'bias.json').write_bytes(raw)
        self.profile['artifacts']['bias']['sha256'] = hashlib.sha256(raw).hexdigest()
        for item in self.profile['artifacts'].values(): item['path'] = str(self.base/item['path'])
        self.policy = TensorPolicy()
        self.model = LivePolicyModel(self.profile, policy=self.policy, torch_module=torch)
        q = tuple(-.8-i*.01 if i%3 == 1 else (.3+i*.01 if i%3 == 2 else .1+i*.01) for i in range(1,13))
        self.sample = MotionSample(q, tuple(i*.01 for i in range(1,13)), (0.,)*12, (25.,)*12, .998)
        self.now = 1_000_000_000

    def imu(self, now=None):
        now = self.now if now is None else now
        return dict(frame='sensor', accel_m_s2=[0.,0.,-9.81], gyro_rad_s=[.11,.22,.33],
                    read_started_monotonic_ns=now-2_000_000, read_finished_monotonic_ns=now-1_000_000)

    def test_warmup_then_one_reset_preserves_fresh_state_across_live_ticks(self):
        self.assertEqual(self.policy.resets, 1)
        self.assertEqual(len(self.policy.inputs), 10)
        self.assertEqual(self.policy.state, 0)
        self.model(self.sample, self.imu(), self.now)
        self.assertEqual(self.policy.state_before_calls[-1], 0)
        self.model(self.sample, self.imu(self.now+20_000_000), self.now+20_000_000)
        self.assertEqual(self.policy.state_before_calls[-1], 1)
        self.assertEqual(self.policy.resets, 1); self.assertEqual(self.model.calls, 2)

    def test_scalar_selection_calls_verified_loader_with_both_manifest_pins(self):
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
        from singularitydog_hw import policy_live_profile as profiles
        from native_policy_overnight.model_call_fastpath import scalar_loader
        self.profile.update(schema=profiles.SCHEMA_V3,model_backend=profiles.SCALAR_BACKEND)
        selected={'path':str(self.base/'scalar.json'),'sha256':'a'*64}
        self.profile['artifacts']['scalar_step_manifest']=selected
        provenance={'schema':'native-step-scalar-file-only-loader-v1','output_allowed':False,
                    'approved_for_runtime':False,'manifest_sha256':'a'*64}
        candidate=TensorPolicy()
        with patch.object(scalar_loader,'load_file_only_verified',return_value=(candidate,provenance)) as loader, \
             patch('native_policy_overnight.load_verified',side_effect=AssertionError('No fallback loader')):
            model=LivePolicyModel(self.profile,torch_module=torch)
        baseline=self.profile['artifacts']['model_manifest']
        loader.assert_called_once_with(selected['path'],expected_sha256=selected['sha256'],
            baseline_manifest=baseline['path'],baseline_sha=baseline['sha256'],bundle=self.profile['bundle_path'])
        self.assertEqual(model.execution['model_backend'],profiles.SCALAR_BACKEND)
        self.assertFalse(model.provenance['output_allowed'])
        self.assertFalse(model.execution['model_artifact_grants_output'])
        self.assertEqual(candidate.resets,1)
        self.assertEqual(model(self.sample,self.imu(),self.now),self.model(self.sample,self.imu(),self.now))
        self.assertEqual(model(self.sample,self.imu(self.now+20_000_000),self.now+20_000_000),
                         self.model(self.sample,self.imu(self.now+20_000_000),self.now+20_000_000))
        self.assertEqual(candidate.state,self.policy.state)
        torch.testing.assert_close(candidate.last_observation,self.policy.last_observation,rtol=0,atol=0)
        torch.testing.assert_close(candidate.last_actor_output,self.policy.last_actor_output,rtol=0,atol=0)
        candidate.bad_actor=True
        with self.assertRaisesRegex(ValueError,'nonfinite'):
            model(self.sample,self.imu(self.now+40_000_000),self.now+40_000_000)

    def test_scalar_verification_failure_does_not_fall_back_to_old_model(self):
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
        from native_policy_overnight.model_call_fastpath import scalar_loader
        self.profile.update(schema='singularitydog.supported-policy-profile.v3',model_backend='scalar_step_cpp')
        self.profile['artifacts']['scalar_step_manifest']={'path':'scalar.json','sha256':'a'*64}
        with patch.object(scalar_loader,'load_file_only_verified',side_effect=ValueError('Scalar parity/source/ABI rejected')), \
             patch('native_policy_overnight.load_verified',side_effect=AssertionError('Fallback prohibited')):
            with self.assertRaisesRegex(ValueError,'Scalar parity/source/ABI'):
                LivePolicyModel(self.profile,torch_module=torch)

    def test_deferred_r22_warmup_prime_and_single_reset_use_owned_buffers(self):
        policy=TensorPolicy()
        model=LivePolicyModel(self.profile,policy=policy,torch_module=torch,defer_warmup=True)
        pointers=tuple(t.data_ptr() for t in model.tensors)
        self.assertEqual((policy.state,policy.resets,len(policy.inputs)),(0,0,0))
        with self.assertRaisesRegex(RuntimeError,'startup reset'):
            model(self.sample,self.imu(),self.now)
        model.pre_pin_warmup()
        self.assertEqual((policy.state,policy.resets,len(policy.inputs)),(10,0,10))
        model.post_pin_prime()
        self.assertEqual((policy.state,policy.resets,len(policy.inputs)),(20,0,20))
        self.assertEqual(policy.input_pointers[10:],[pointers]*10)
        model.finish_startup()
        self.assertEqual((policy.state,policy.resets,model.calls),(0,1,0))
        self.assertEqual(model(self.sample,self.imu(),self.now),
                         self.model(self.sample,self.imu(),self.now))
        self.assertEqual(policy.state_before_calls[-1],0)
        self.assertEqual(self.policy.state_before_calls[-1],0)
        for method in (model.pre_pin_warmup,model.post_pin_prime,model.finish_startup):
            with self.assertRaises(RuntimeError):method()

    def test_deferred_startup_without_prime_resets_once_and_rejects_foreign_buffer(self):
        policy=TensorPolicy()
        model=LivePolicyModel(self.profile,policy=policy,torch_module=torch,defer_warmup=True)
        model.pre_pin_warmup()
        owned=model.input_buffers
        model.input_buffers=(array('f',[0.]*3),*owned[1:])
        with self.assertRaisesRegex(RuntimeError,'owned reused'):
            model.post_pin_prime()
        self.assertEqual(len(policy.inputs),10)
        model.input_buffers=owned
        model.finish_startup()
        self.assertEqual((policy.resets,policy.state,model.calls),(1,0,0))

    def test_sensor_bias_removed_before_signed_body_rotation(self):
        self.model(self.sample, self.imu(), self.now)
        gyro, gravity, command, _, _, _ = self.policy.inputs[-1]
        torch.testing.assert_close(gyro, torch.tensor([[.2,.1,-.3]]))
        torch.testing.assert_close(gravity, torch.tensor([[0.,0.,-1.]]))
        torch.testing.assert_close(command, torch.zeros((1,3)))

    def test_can_order_to_model_and_back_are_inverse(self):
        self.policy.target = [(.01*index if index%3 == 0 else (.4+index*.01 if index%3 == 1 else -.8-index*.01))
                              for index in range(12)]
        target = self.model(self.sample, self.imu(), self.now)
        inputs = self.policy.inputs[-1]
        torch.testing.assert_close(inputs[3], torch.tensor([[self.sample.q_model_rad[i-1] for i in CAN_ORDER]]))
        torch.testing.assert_close(inputs[4], torch.tensor([[self.sample.velocity_rad_s[i-1] for i in CAN_ORDER]]))
        for index, mid in enumerate(CAN_ORDER): self.assertAlmostEqual(target[mid-1], self.policy.target[index], places=6)

    def test_repeated_tick_and_imu_are_rejected_before_another_model_call(self):
        self.model(self.sample, self.imu(), self.now)
        count = len(self.policy.inputs)
        with self.assertRaisesRegex(ValueError, 'reused'):
            self.model(self.sample, self.imu(), self.now+20_000_000)
        with self.assertRaisesRegex(ValueError, 'reused'):
            self.model(self.sample, self.imu(self.now+20_000_000), self.now)
        self.assertEqual(len(self.policy.inputs), count)

    def test_stale_future_reversed_and_bool_timestamps_rejected(self):
        cases = [dict(read_started_monotonic_ns=self.now-30_000_000),
                 dict(read_finished_monotonic_ns=self.now+1),
                 dict(read_finished_monotonic_ns=self.now-3_000_000),
                 dict(read_started_monotonic_ns=True)]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.model.validate_inputs(self.sample, dict(self.imu(), **change), self.now)
        self.assertEqual(self.model.calls, 0)

    def test_norm_tilt_and_angular_velocity_each_block(self):
        cases = [dict(accel_m_s2=[0.,0.,0.]), dict(accel_m_s2=[9.81,0.,0.]),
                 dict(gyro_rad_s=[2.,0.,0.]), dict(gyro_rad_s=[float('nan'),0.,0.]),
                 dict(accel_m_s2=[True,0.,-9.81])]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.model.validate_inputs(self.sample, dict(self.imu(), **change), self.now)
        self.assertEqual(self.model.calls, 0)

    def test_model_position_domain_checked_before_inference(self):
        bad = list(self.sample.q_model_rad); bad[0] = 0.
        sample = MotionSample(tuple(bad), self.sample.velocity_rad_s, self.sample.torque_nm,
                              self.sample.temperature_c, self.sample.monotonic_s)
        with self.assertRaisesRegex(ValueError, 'Measured joints'):
            self.model(sample, self.imu(), self.now)
        self.assertEqual(self.model.calls, 0)

    def test_nonfinite_actor_observation_or_target_does_not_return_command(self):
        for mode in ('bad_actor', 'bad_observation', 'bad_target', 'out_of_range_target'):
            actor = TensorPolicy(); instance = LivePolicyModel(self.profile, policy=actor, torch_module=torch)
            if mode == 'bad_target': actor.target[0] = float('nan')
            elif mode == 'out_of_range_target': actor.target[0] = .8
            else: setattr(actor, mode, True)
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                instance(self.sample, self.imu(), self.now)
            self.assertEqual(instance.calls, 0); self.assertIsNone(instance.last_target)

    def test_unapproved_profile_fails_before_candidate_files_or_warmup(self):
        with self.assertRaisesRegex(ValueError, 'Reviewed output'):
            LivePolicyModel({'output_allowed': False}, policy=TensorPolicy(), torch_module=torch)

    def test_float32_model_boundaries_are_accepted_without_expanding_physical_limits(self):
        #1.2 and-0.08 are not exactfloat32 values. Compare with the original
        # model's float32 endpoints; profile/envelope margins remain separate.
        for index, bound in enumerate((LOWER, UPPER)):
            self.policy.target = list(bound)
            now = self.now+index*20_000_000
            target = self.model(self.sample, self.imu(now), now)
            self.assertEqual(len(target), 12)

    def test_already_corrected_imu_is_rejected_instead_of_double_transform(self):
        cases = [dict(frame='body'), dict(gyro_bias_subtracted=True), dict(mount_rotation_applied=True)]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.model.validate_inputs(self.sample, dict(self.imu(), **change), self.now)

    def test_changed_mount_bytes_rejected_at_model_initialization(self):
        path = Path(self.profile['artifacts']['mount']['path'])
        value = json.loads(path.read_text()); value['R_body_from_sensor'] = [[1,0,0],[0,1,0],[0,0,1]]
        path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'SHA|hash|changed'):
            LivePolicyModel(self.profile, policy=TensorPolicy(), torch_module=torch)

    def test_input_storage_is_reused_and_command_override_reaches_model(self):
        pointers=tuple(t.data_ptr() for t in self.model.tensors)
        buffers=tuple(id(b) for b in self.model.input_buffers)
        for index,vx in enumerate((0.,.025,.05,0.)):
            now=self.now+index*20_000_000
            self.model(self.sample,self.imu(now),now,command_override=(vx,0.,0.))
            torch.testing.assert_close(self.policy.inputs[-1][2],torch.tensor([[vx,0.,0.]]))
            self.assertEqual(pointers,tuple(t.data_ptr() for t in self.model.tensors))
            self.assertEqual(buffers,tuple(id(b) for b in self.model.input_buffers))
        self.assertEqual(self.model.last_validation['frame'],'body')
        self.assertTrue(self.model.last_validation['gyro_bias_subtracted'])

    def test_out_of_scope_ground_commands_rejected_before_model_call(self):
        count=len(self.policy.inputs)
        for command in ((.051,0,0),(-.01,0,0),(0,.01,0),(0,0,.01),
                        (float('nan'),0,0),(True,0,0),(0,0)):
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'Ground command'):
                self.model(self.sample,self.imu(),self.now,command_override=command)
        self.assertEqual(count,len(self.policy.inputs))

    def test_invalid_model_tensor_abi_never_returns_command(self):
        class WrongTensorPolicy(TensorPolicy):
            wrong_field=None
            def __call__(self,*values):
                target=super().__call__(*values)
                if self.wrong_field=='target':return target.double()
                if self.wrong_field=='actor':self.last_actor_output=self.last_actor_output.double()
                if self.wrong_field=='observation':self.last_observation=self.last_observation.reshape(74,1)
                return target
        for field in ('target','actor','observation'):
            policy=WrongTensorPolicy()
            model=LivePolicyModel(self.profile,policy=policy,torch_module=torch)
            policy.wrong_field=field
            with self.subTest(field=field),self.assertRaises(ValueError):
                model(self.sample,self.imu(),self.now)
            self.assertEqual(model.calls,0)
            self.assertIsNone(model.last_target)


if __name__ == '__main__':
    unittest.main()
