"""File-only FK admission/delegation tests; synthetic policies are not approval."""
from contextlib import contextmanager
import copy
import hashlib
import json
import math
import importlib.util
from pathlib import Path
import subprocess
import shutil
import sys
import tempfile
from types import SimpleNamespace
from types import ModuleType
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_active_fk as fk
from singularitydog_hw import policy_live_profile as profiles


def selected_profile():
    return dict(schema=profiles.SCHEMA_V3, scope='supported_characterization_only',
        model_backend=profiles.SCALAR_BACKEND, native_target_fk_cache=True,
        local_characterization=profiles.LOCAL_RELATIVE_SUPPORTED,
        watchdog_review_policy=profiles.COMMAND_LOSS_ONLY_SUPPORTED,
        diagnostic_timing_acceptance=profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
        duration_s=2., policy_weight=.005, command=[0., 0., 0.],
        voltage_overlap=True, voltage_pipeline=True, period_ms=20, hard_cycle_ms=20,
        max_sample_age_ms=20, max_sample_gap_ms=21, max_consecutive_20ms_misses=0,
        axes={str(i):dict(kp=3.,kd=.15,max_displacement_from_start_rad=math.radians(1))
              for i in range(1,13)}, bundle_path='SYNTHETIC_FILE_ONLY_NOT_HARDWARE')


class ActiveFKFileTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name).resolve()
        self.profile = selected_profile()
        self.raw = {}
        def write(name, data):
            raw = json.dumps(data,sort_keys=True,allow_nan=False).encode()
            path = self.folder/name; path.write_bytes(raw); self.raw[name]=raw
            return {'path':str(path),'sha256':hashlib.sha256(raw).hexdigest()}
        baseline = write('baseline.json', {'schema':'SYNTHETIC_BASELINE_ONLY'})
        scalar = write('scalar.json', {'schema':'SYNTHETIC_SCALAR_ONLY'})
        source = Path(__file__).resolve().parents[1]/'experiments/native_policy_overnight/target_tail_fk_cache/diagnostic_loader.py'
        self.loader_ref = {'path':str(source),'sha256':fk.LOADER_SHA256}
        self.manifest = {'schema':'singularitydog.fk-cache-stop-diagnostic-artifact.v1',
            'status':'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL',
            'references':{'scalar_manifest':scalar,'baseline_manifest':baseline},
            'integration_sources':{'diagnostic_loader.py':self.loader_ref},
            **dict.fromkeys(fk._FALSE_FLAGS,False)}
        target = write('fk.json',self.manifest)
        self.profile['artifacts']={'model_manifest':baseline,'scalar_step_manifest':scalar,
                                   'target_fk_manifest':target}
        self.refs, _ = fk._bindings(self.profile)
        self.delegated_plan={'schema':'singularitydog.fk-cache-stop-diagnostic-loader-plan.v1',
            'manifest_sha256':target['sha256'],'model_sha256':'a'*64,'library_sha256':'b'*64,
            'backend':'pinned_fk_cache_cpp','validated_saved_calls':501,
            'torch_or_native_loaded':False,**dict.fromkeys(fk._FALSE_FLAGS,False)}
        self.raw_provenance={'schema':'singularitydog.fk-cache-stop-diagnostic-loader.v1',
            'manifest_sha256':target['sha256'],'model_sha256':'a'*64,'library_sha256':'b'*64,
            'baseline_provenance':{'manifest_sha256':baseline['sha256'],'kept':'raw baseline'},
            'original_scalar_dependency':{'manifest_sha256':scalar['sha256'],'kept':'raw scalar'},
            'candidate_policy_backend_provenance':'EXPERIMENTAL_STOP_DIAGNOSTIC_ONLY',
            'diagnostic_only':True,'hardware_opened':False,**dict.fromkeys(fk._FALSE_FLAGS,False)}
        self.calls=[]; self.policy=object()
        @contextmanager
        def delegated(loader_ref):
            self.assertEqual(loader_ref,self.loader_ref)
            def plan(path,**options):
                self.calls.append(('plan',path,options));return copy.deepcopy(self.delegated_plan)
            def load(path,**options):
                self.calls.append(('load',path,options));return self.policy,self.raw_provenance
            yield SimpleNamespace(plan=plan,load_diagnostic_verified=load)
        self.delegated=delegated

    def seal_manifest(self):
        path=Path(self.profile['artifacts']['target_fk_manifest']['path'])
        raw=json.dumps(self.manifest,sort_keys=True,allow_nan=False).encode();path.write_bytes(raw)
        self.profile['artifacts']['target_fk_manifest']['sha256']=hashlib.sha256(raw).hexdigest()

    def test_absent_false_selection_retains_legacy_without_artifact_reads(self):
        for value in (None,False):
            profile={} if value is None else {'native_target_fk_cache':value}
            with patch.object(fk,'_read',side_effect=AssertionError('No selected artifact read')):
                self.assertFalse(fk.selected(profile))

    def test_selection_boolean_and_every_narrow_scope_boundary(self):
        for key,value in [('native_target_fk_cache',1),('native_target_fk_cache','true'),
                ('schema',profiles.SCHEMA_V2),('model_backend','native_baseline'),
                ('duration_s',3.),('duration_s',1<<4096),('policy_weight',.006),('command',[.01,0.,0.]),
                ('scope','ground'),('local_characterization',None),('watchdog_review_policy',None),
                ('voltage_overlap',False),('voltage_pipeline',False),('hard_cycle_ms',21),
                ('max_sample_age_ms',21),('max_sample_gap_ms',22),('max_consecutive_20ms_misses',1)]:
            changed=copy.deepcopy(self.profile);changed[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):fk.selected(changed)
        for key,value in [('kp',3.001),('kd',.151),('max_displacement_from_start_rad',math.radians(1.001)),('kp',float('nan'))]:
            changed=copy.deepcopy(self.profile);changed['axes']['12'][key]=value
            with self.subTest(axis_key=key),self.assertRaises(ValueError):fk.selected(changed)

    def test_only_matching_two_ten_twenty_modes_are_selected(self):
        for duration,mode in [(2,profiles.SUPPORTED_POLICY_PROBE),(2,profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER),
                              (10,profiles.SUPPORTED_POLICY_PROBE_10S_AFTER_2S),(20,profiles.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)]:
            profile=copy.deepcopy(self.profile);profile.update(duration_s=duration,diagnostic_timing_acceptance=mode)
            self.assertTrue(fk.selected(profile))
            profile['duration_s']=2 if duration!=2 else 10
            with self.assertRaises(ValueError):fk.selected(profile)

    def test_plan_delegates_exact_refs_and_keeps_false_flags_independent(self):
        self.assertEqual(fk.source_paths(),('singularitydog_hw/policy_active_fk.py',
                                            'singularitydog_hw/policy_output_model.py'))
        before=copy.deepcopy(self.profile)
        with patch.object(fk,'_loader_scope',side_effect=self.delegated):proof=fk.plan(self.profile)
        call=self.calls[0];self.assertEqual(call[0],'plan');self.assertEqual(call[1],self.refs['target_fk_manifest']['path'])
        self.assertEqual(call[2],fk._kwargs(self.refs))
        self.assertEqual(proof['manifest_sha256'],self.refs['target_fk_manifest']['sha256'])
        self.assertEqual(proof['active_binding']['scalar_manifest'],self.refs['scalar_manifest'])
        self.assertEqual(proof['active_binding']['baseline_manifest'],self.refs['baseline_manifest'])
        self.assertTrue(all(proof[flag] is False for flag in fk._FALSE_FLAGS))
        self.assertFalse(proof['hardware_opened']);self.assertFalse(proof['torch_or_native_loaded'])
        proof['active_binding']['scalar_manifest']['sha256']='c'*64
        self.assertEqual(self.profile,before)

    def test_plan_checks_supplied_parsed_documents_against_actual_pinned_bytes(self):
        with patch.object(fk,'_loader_scope',side_effect=self.delegated):
            fk.plan(self.profile,{'target_fk_manifest':copy.deepcopy(self.manifest)})
            with self.assertRaisesRegex(ValueError,'Parsed FK document differs'):
                fk.plan(self.profile,{'target_fk_manifest':{'forged':True}})

    def test_changed_source_unapproved_flags_and_dependency_refs_fail_before_delegation(self):
        original=copy.deepcopy(self.manifest)
        for mutate in [lambda d:d.update(output_allowed=True),
                lambda d:d['references']['scalar_manifest'].update(sha256='c'*64),
                lambda d:d['integration_sources']['diagnostic_loader.py'].update(sha256='d'*64)]:
            self.manifest=copy.deepcopy(original);mutate(self.manifest);self.seal_manifest()
            with patch.object(fk,'_loader_scope',side_effect=AssertionError('Must reject before source execution')):
                with self.assertRaises(ValueError):fk.plan(self.profile)

    def test_primary_manifest_mutation_after_plan_is_rejected(self):
        @contextmanager
        def mutate(_):
            def plan(*args,**kwargs):
                Path(self.refs['target_fk_manifest']['path']).write_bytes(b'{}')
                return self.delegated_plan
            yield SimpleNamespace(plan=plan)
        with patch.object(fk,'_loader_scope',side_effect=mutate),self.assertRaisesRegex(ValueError,'SHA256'):
            fk.plan(self.profile)

    def test_bounded_regular_symlink_hash_and_duplicate_json_rejections(self):
        ref=dict(self.refs['target_fk_manifest']);ref['sha256']='e'*64
        with self.assertRaisesRegex(ValueError,'SHA256'):fk._read(ref)
        link=self.folder/'link.json';link.symlink_to(ref['path']);ref['path']=str(link)
        with self.assertRaisesRegex(ValueError,'non-symlink'):fk._read(ref)
        ref={'path':str(self.folder),'sha256':'f'*64}
        with self.assertRaisesRegex(ValueError,'regular'):fk._read(ref)
        for raw in (b'{"x":1,"x":2}',b'{"x":NaN}'):
            with self.assertRaises(ValueError):fk._json(raw)

    def test_real_frozen_loader_scope_uses_exact_path_and_restores_namespace_on_error(self):
        before=list(sys.path)
        with self.assertRaisesRegex(RuntimeError,'primary test error'):
            with fk._loader_scope(self.loader_ref) as loader:
                self.assertEqual(loader.__file__,self.loader_ref['path'])
                self.assertEqual(loader.__package__,fk._PACKAGE+'.target_tail_fk_cache')
                raise RuntimeError('primary test error')
        self.assertEqual(sys.path,before);self.assertNotIn(fk._PRIVATE,sys.modules)

    def test_real_loader_binding_mutation_is_rejected_and_cleaned(self):
        before=list(sys.path)
        with self.assertRaisesRegex(ValueError,'binding changed'):
            with fk._loader_scope(self.loader_ref) as loader:loader.plan=lambda *a,**k:None
        self.assertEqual(sys.path,before);self.assertNotIn(fk._PRIVATE,sys.modules)

    def test_private_prebinding_rejected_without_disturbing_existing_binding(self):
        sentinel=object()
        with patch.dict(sys.modules,{fk._PRIVATE:sentinel}):
            with self.assertRaisesRegex(ValueError,'already bound'):
                with fk._loader_scope(self.loader_ref):pass
            self.assertIs(sys.modules[fk._PRIVATE],sentinel)

    def test_ordinary_cached_foreign_module_is_never_used_or_changed(self):
        foreign=ModuleType('native_policy_overnight.contracts')
        foreign.require=lambda *a:(_ for _ in ()).throw(AssertionError('Foreign code used'))
        with patch.dict(sys.modules,{'native_policy_overnight.contracts':foreign}):
            with fk._loader_scope(self.loader_ref) as loader:
                self.assertIsNot(loader.require,foreign.require)
                loader.require(True,'authenticated private dependency')
            self.assertIs(sys.modules['native_policy_overnight.contracts'],foreign)

    def test_valid_stale_dependency_pyc_is_ignored_and_source_mutation_rejected(self):
        import struct
        import marshal
        folder=self.folder/'dependencies';folder.mkdir()
        for name in fk._DEPENDENCY_PINS:
            destination=folder/name;destination.parent.mkdir(exist_ok=True,parents=True)
            shutil.copyfile(fk._dependency_folder()/name,destination)
        source=folder/'contracts.py'
        cache=Path(importlib.util.cache_from_source(str(source)));cache.parent.mkdir(exist_ok=True)
        stat=source.stat()
        cache.write_bytes(importlib.util.MAGIC_NUMBER+struct.pack('<III',0,int(stat.st_mtime),stat.st_size)+
                          marshal.dumps(compile("raise AssertionError('stale dependency pyc executed')",str(source),'exec')))
        with patch.object(fk,'_dependency_folder',return_value=folder):
            with fk._loader_scope(self.loader_ref) as loader:loader.require(True,'source compiled directly')
            source.write_bytes(source.read_bytes()+b'\n')
            with self.assertRaisesRegex(ValueError,'SHA256'):
                with fk._loader_scope(self.loader_ref):pass
        self.assertFalse(any(n==fk._PACKAGE or n.startswith(fk._PACKAGE+'.') for n in sys.modules))

    def test_foreign_private_dependency_binding_is_rejected_and_all_aliases_removed(self):
        with self.assertRaisesRegex(ValueError,'dependency binding changed'):
            with fk._loader_scope(self.loader_ref):sys.modules[fk._PACKAGE+'.contracts']=object()
        self.assertFalse(any(n==fk._PACKAGE or n.startswith(fk._PACKAGE+'.') for n in sys.modules))

    def test_fresh_process_helper_and_real_loader_plan_import_no_torch_or_device_modules(self):
        runtime=str(Path(__file__).resolve().parents[1])
        script='''import sys
sys.path.insert(0,sys.argv[1])
class Block:
 def find_spec(self,name,path=None,target=None):
  if name.split('.')[0] in {'torch','serial','ctypes'}:raise AssertionError('Forbidden import: '+name)
sys.meta_path.insert(0,Block())
from singularitydog_hw import policy_active_fk as fk
with fk._loader_scope({'path':sys.argv[2],'sha256':sys.argv[3]}) as loader:
 try:loader.plan(sys.argv[4],expected_sha256=sys.argv[5],scalar_manifest=sys.argv[6],scalar_sha=sys.argv[7],baseline_manifest=sys.argv[8],baseline_sha=sys.argv[9])
 except ValueError:pass
assert not {'torch','serial','ctypes'}.intersection(sys.modules)
assert fk._PRIVATE not in sys.modules
'''
        args=[sys.executable,'-I','-B','-c',script,runtime,self.loader_ref['path'],self.loader_ref['sha256']]
        for key in ('target_fk_manifest','scalar_manifest','baseline_manifest'):
            args.extend((self.refs[key]['path'],self.refs[key]['sha256']))
        result=subprocess.run(args,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def admitted(self):
        self.profile['output_allowed']=True
        with patch.object(fk,'_loader_scope',side_effect=self.delegated):
            self.profile['_native_target_fk_cache_provenance']=fk.plan(self.profile)

    def test_load_keeps_actual_fk_and_entire_raw_diagnostic_provenance(self):
        self.admitted();original=copy.deepcopy(self.raw_provenance)
        with patch.object(fk,'_loader_scope',side_effect=self.delegated), \
             patch.object(profiles,'native_target_fk_cache_settings',return_value=self.refs['target_fk_manifest']):
            policy,proof=fk.load(self.profile)
        self.assertIs(policy,self.policy)
        self.assertEqual({k:v for k,v in proof.items() if k!='active_binding'},original)
        self.assertEqual(proof['manifest_sha256'],self.refs['target_fk_manifest']['sha256'])
        self.assertNotEqual(proof['manifest_sha256'],self.refs['scalar_manifest']['sha256'])
        self.assertEqual(self.raw_provenance,original)
        proof['original_scalar_dependency']['kept']='changed copy';self.assertEqual(self.raw_provenance,original)
        self.assertEqual([c[0] for c in self.calls],['plan','plan','load','plan'])
        self.assertEqual(self.calls[-2][2],{**fk._kwargs(self.refs),'bundle':self.profile['bundle_path']})

    def test_load_rejects_absent_admission_changed_held_proof_and_false_model_proof(self):
        self.admitted()
        with patch.object(profiles,'native_target_fk_cache_settings',return_value=self.refs['target_fk_manifest']), \
             patch.object(fk,'_loader_scope',side_effect=self.delegated):
            self.profile['output_allowed']=False
            with self.assertRaisesRegex(ValueError,'Admitted active FK profile'):fk.load(self.profile)
            self.profile['output_allowed']=True
            self.profile['_native_target_fk_cache_provenance']['model_sha256']='f'*64
            with self.assertRaisesRegex(ValueError,'proof changed'):fk.load(self.profile)
            self.profile['_native_target_fk_cache_provenance']=fk.plan(self.profile)
            self.raw_provenance['original_scalar_dependency']['manifest_sha256']='f'*64
            with self.assertRaisesRegex(ValueError,'provenance differs'):fk.load(self.profile)

    def test_explicit_disabled_diagnostic_load_never_requires_or_grants_active_admission(self):
        self.profile['output_allowed']=False
        with patch.object(fk,'_loader_scope',side_effect=self.delegated), \
             patch.object(profiles,'native_target_fk_cache_settings',side_effect=AssertionError('No active admission')):
            policy,proof=fk.diagnostic_load(self.profile)
        self.assertIs(policy,self.policy)
        self.assertTrue(all(proof[flag] is False for flag in fk._FALSE_FLAGS))
        self.assertFalse(proof['hardware_opened']);self.assertFalse(proof['active_binding']['model_artifact_grants_output'])
        self.assertNotIn('_native_target_fk_cache_provenance',self.profile)


try:
    import torch
except ImportError:
    torch = None


@unittest.skipIf(torch is None,'CPU PyTorch is unavailable')
class ActiveFKAdapterTests(unittest.TestCase):
    def setUp(self):
        from test_policy_output_model import TensorPolicy
        from test_policy_live_profile import synthetic_fixture
        from singularitydog_hw.policy_motion_envelope import MotionSample
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.folder=Path(temp.name).resolve()
        self.profile,_,_=synthetic_fixture(self.folder)
        selection=selected_profile()
        self.profile.update({k:v for k,v in selection.items() if k not in ('axes','bundle_path')})
        for axis in self.profile['axes'].values():
            axis.update(kp=3.,kd=.15,max_displacement_from_start_rad=math.radians(1),
                max_estimated_pd_torque_nm=.1,max_measured_torque_nm=1.,
                max_command_velocity_rad_s=math.radians(1),
                max_command_acceleration_rad_s2=math.radians(5),
                max_tracking_error_rad=math.radians(2),max_measured_velocity_rad_s=.25,
                max_temperature_c=45.)
        self.profile['output_allowed']=True
        for ref in self.profile['artifacts'].values():ref['path']=str(self.folder/ref['path'])
        self.profile['artifacts']['scalar_step_manifest']={'path':str(self.folder/'scalar.json'),'sha256':'c'*64}
        self.profile['artifacts']['target_fk_manifest']={'path':str(self.folder/'fk.json'),'sha256':'d'*64}
        self.policy=TensorPolicy()
        self.provenance={'schema':'SYNTHETIC_FK_LOADER_BOUNDARY_ONLY','manifest_sha256':'d'*64,
                         'output_allowed':False,'approved_for_runtime':False}
        q=tuple(-.8-i*.01 if i%3==1 else (.3+i*.01 if i%3==2 else .1+i*.01) for i in range(1,13))
        self.sample=MotionSample(q,(0.,)*12,(0.,)*12,(25.,)*12,.998)
        self.now=1_000_000_000

    def imu(self,now):
        return dict(frame='sensor',accel_m_s2=[0.,0.,-9.81],gyro_rad_s=[0.,0.,0.],
                    read_started_monotonic_ns=now-2_000_000,
                    read_finished_monotonic_ns=now-1_000_000)

    def model(self,**kwargs):
        from singularitydog_hw.policy_output_model import LivePolicyModel
        with patch.object(fk,'load',return_value=(self.policy,self.provenance)) as loader:
            model=LivePolicyModel(self.profile,torch_module=torch,**kwargs)
        loader.assert_called_once_with(self.profile)
        return model

    def test_selected_loader_keeps_six_owned_buffers_state_reset_and_legacy_targets(self):
        from test_policy_output_model import TensorPolicy
        from singularitydog_hw.policy_output_model import LivePolicyModel
        model=self.model()
        legacy_profile=copy.deepcopy(self.profile);legacy_profile.pop('native_target_fk_cache')
        legacy_profile['artifacts'].pop('target_fk_manifest')
        reference=TensorPolicy();legacy=LivePolicyModel(legacy_profile,policy=reference,torch_module=torch)
        pointers=tuple(t.data_ptr() for t in model.tensors)
        self.assertEqual((self.policy.resets,self.policy.state),(1,0))
        for tick in range(3):
            now=self.now+tick*20_000_000
            self.assertEqual(model(self.sample,self.imu(now),now),legacy(self.sample,self.imu(now),now))
            self.assertEqual(self.policy.state,reference.state)
            torch.testing.assert_close(self.policy.last_actor_output,reference.last_actor_output,rtol=0,atol=0)
            torch.testing.assert_close(self.policy.last_observation,reference.last_observation,rtol=0,atol=0)
        self.assertEqual(self.policy.input_pointers[-3:],[pointers]*3)
        self.assertEqual(model.execution['model_backend'],profiles.SCALAR_BACKEND)
        self.assertTrue(model.execution['native_target_fk_cache'])
        self.assertEqual(model.provenance['manifest_sha256'],'d'*64)
        self.assertFalse(model.execution['model_artifact_grants_output'])

    def test_selected_deferred_warmup_prime_and_reset_remain_in_existing_order(self):
        model=self.model(defer_warmup=True)
        self.assertEqual((self.policy.state,self.policy.resets),(0,0))
        with self.assertRaisesRegex(RuntimeError,'startup reset'):model(self.sample,self.imu(self.now),self.now)
        model.pre_pin_warmup();self.assertEqual((self.policy.state,self.policy.resets),(10,0))
        model.post_pin_prime();self.assertEqual((self.policy.state,self.policy.resets),(20,0))
        model.finish_startup();self.assertEqual((self.policy.state,self.policy.resets),(0,1))
        model(self.sample,self.imu(self.now),self.now);self.assertEqual(self.policy.state_before_calls[-1],0)

    def test_selected_loader_failure_never_falls_back_or_allows_policy_injection(self):
        from singularitydog_hw.policy_output_model import LivePolicyModel
        with (patch.object(fk,'load',side_effect=ValueError('Synthetic FK source/ABI rejected')),
              self.assertRaisesRegex(ValueError,'Synthetic FK source/ABI')):
            LivePolicyModel(self.profile,torch_module=torch)
        with (patch.object(fk,'load',side_effect=AssertionError('No injected FK load')),
              self.assertRaisesRegex(ValueError,'verified artifact loader')):
            LivePolicyModel(self.profile,policy=self.policy,torch_module=torch)

    def test_selected_route_retains_stale_reused_and_nonfinite_telemetry_rejection(self):
        model=self.model();stale=self.imu(self.now);stale['read_started_monotonic_ns']-=30_000_000
        with self.assertRaisesRegex(ValueError,'Stale/noncausal'):model(self.sample,stale,self.now)
        model(self.sample,self.imu(self.now),self.now)
        with self.assertRaisesRegex(ValueError,'reused'):model(self.sample,self.imu(self.now),self.now)
        self.policy.bad_observation=True
        with self.assertRaisesRegex(ValueError,'nonfinite'):
            model(self.sample,self.imu(self.now+20_000_000),self.now+20_000_000)


if __name__=='__main__':unittest.main()
