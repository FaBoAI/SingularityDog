"""Run the proposed hooks only against memory transports and PTYs.

No patched production module is installed. No synthetic result is a hardware
prerequisite, admission, source freeze or statement about physical support.
"""
import copy
from contextlib import ExitStack
import json
import os
from pathlib import Path
import pty
import types
import unittest
from unittest.mock import patch

from experiments.four_bus_type1 import type1_runner as original
from experiments.four_bus_type1 import test_type1_runner as cases
from experiments.four_bus_type1.test_type1_30hz import admitted30, Mock30
from . import profile
from ._proposal_fixture import reconstruct_proposal
from .supervision import HumanSupported30Execution
from .test_supervision import FakeAudio, ParkedThread, audio_manifest, synthetic_timer_scope, profile as supervisor_fixture

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'experiments/four_bus_type1/type1_runner.py'
PATCH = Path(__file__).with_name('type1_runner_hooks.proposed.patch')
MANIFEST = Path(__file__).with_name('runner-proposal-manifest.json')


def load_proposal():
    """Replay the historical pinned diff, never patch the current boxed source."""
    file=Path(__file__).with_name('type1_runner.py')
    frozen=reconstruct_proposal(MANIFEST,PATCH,file,'proposed_runner_sha256')
    value=types.ModuleType('experiments.four_bus_type1._human30_test_proposal')
    value.__package__='experiments.four_bus_type1'; value.__file__=str(file)
    exec(compile(frozen.proposed,str(file),'exec'),value.__dict__)
    return value


def synthetic_spec(mode):
    base=admitted30()
    spec=original.validate_admitted(base)
    spec=copy.deepcopy(spec)
    spec.update(mode=mode,duration_s=8 if mode=='human_current_hold' else 2,
                stop_at_s=7.36 if mode=='human_current_hold' else 1.36,max_stop_s=.6,
                first_cycle_post_reply=False)
    p=spec['profile']; p.update(duration_s=spec['duration_s'],policy_weight=0.,startup_duration_s=1.,startup_damping_duration_s=1.,
        stop_duration_s=.4,policy_ramp_s=.1)
    for axis in p['axes'].values():
        axis.update(kp=6.,kd=.15,max_measured_velocity_rad_s=.25,max_estimated_pd_torque_nm=.2)
    return spec


class RunnerProposalTests(cases.Harness, unittest.TestCase):
    def setUp(self):
        cases.Harness.setUp(self)
        self.proposed=load_proposal()
        self.master,self.slave=pty.openpty()
        self.addCleanup(os.close,self.master);self.addCleanup(os.close,self.slave)

    def factory(self,group):
        value=Mock30(group,self);self.adapters[group.port]=value;return value

    def synthetic_run(self,mode,**controls):
        self.controls.update(controls)
        spec=synthetic_spec(mode)
        self.execution=None; self.audio=None
        # Explicit test seam only: bypasses opaque admission/source gates in
        # this test process. Those production gates have separate negative tests.
        with ExitStack() as stack:
            stack.enter_context(patch.object(self.proposed,'validate_admitted',return_value=spec))
            stack.enter_context(patch.object(profile,'verify_runner_source',return_value=None))
            stack.enter_context(patch('experiments.four_bus_human_hold.supervision.supervisor_view',
                                     return_value=(supervisor_fixture(),audio_manifest())))
            if mode=='human_current_hold':
                self.audio=FakeAudio(self.clock)
                self.execution=HumanSupported30Execution(self.slave,clock=self.clock,
                    thread_factory=ParkedThread,stage_audio=self.audio,timer_scope=synthetic_timer_scope)
                self.addCleanup(self.execution.close)
            original_release=self.release_wait
            def release(when):
                actual=original_release(when)
                if self.audio:
                    for stage,(started,_,_) in list(self.audio.pending.items()):
                        if self.clock()-started>=round(audio_manifest()['clips'][stage]['duration_s']*1e9):
                            self.audio.finish(stage)
                            if stage=='resupport': os.write(self.master,b'\n')
                return actual
            return self.proposed.run({'synthetic_test_only':True},factory=self.factory,imu_read=self.imu,
                observer=cases.Observer(),check_current=self.current,check_cancelled=self.check,
                cancel_io=self.cancel_io,model_setup=lambda o,**k:dict(k,reset_verified=True,file_only_mock_model=True),
                worker_scope=self.worker_scope,main_scope=self.main_scope,release_wait=release,
                announce=lambda:self.events.append(('announce',)),stop_requested=self.stop_requested,
                backend_usage={'kind':'injected_file_only_mock'},clock=self.clock,execute=True,
                human_supervision=self.execution)

    def test_frozen_proposal_replay_does_not_modify_current_boxed_source(self):
        receipt=json.loads(MANIFEST.read_text())
        before=SOURCE.read_bytes()
        load_proposal()
        self.assertEqual(SOURCE.read_bytes(),before)
        self.assertEqual(self.proposed.plan(admitted30()),original.plan(admitted30()))
        self.assertFalse(receipt['live_execution_entry_installed'])
        self.assertFalse(receipt['output_allowed'])

    def test_proposed_human_zero_gain_finishes_all12_stop_without_partial_cue(self):
        result=self.synthetic_run('human_zero_gain_timing')
        self.assertEqual(result['status'],'COMPLETE_HUMAN_ZERO_GAIN_TIMING',result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['positive_gain_sent'])
        self.assertFalse(result['learned_targets_attempted'])
        self.assertIsNone(self.execution)
        self.assert_all_stopped(result)
        self.assertTrue(all(not any(row['command']['kp']) for row in result['cycles']))

    def test_current_hold_preserves_zero_weight_and_resupport_before_gain_down(self):
        result=self.synthetic_run('human_current_hold')
        self.assertEqual(result['status'],'COMPLETE_HUMAN_CURRENT_HOLD',result['errors'])
        self.assertTrue(result['positive_gain_sent'])
        self.assertFalse(result['learned_targets_attempted'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(all(row['weight']==0. for row in result['cycles']))
        self.assert_all_stopped(result)
        trial=result['human_supported_trial']
        self.assertIsNotNone(trial['slight_ease_cue_ns'])
        self.assertIsNotNone(trial['full_support_ack_ns'])
        self.assertLess(trial['full_support_ack_ns'],result['all_workers_joined_monotonic_ns'])
        self.assertFalse(trial['actual_full_support_or_weight_transfer_measured'])
        self.assertFalse(trial['standing_or_walking_authorized'])

    def test_current_guard_failure_during_easing_latches_immediate_resupport_and_stop(self):
        result=self.synthetic_run('human_current_hold',current_fail=(45,1))
        self.assertEqual(result['status'],'ABORTED')
        self.assertTrue(result['failure_retained'])
        self.assertTrue(result['stop_confirmed'])
        self.assert_all_stopped(result)
        trial=result['human_supported_trial']
        self.assertIsNotNone(trial['slight_ease_cue_ns'])
        self.assertIsNotNone(trial['full_support_cue_ns'])
        self.assertIsNotNone(trial['failure'])
        self.assertLessEqual(trial['full_support_cue_ns'],result['stop_latched_ns'])
        self.assertIsNone(trial['full_support_ack_ns'])

    def test_terminal_restore_failure_after_all_axis_stop_cannot_report_complete(self):
        original_set=os.set_blocking
        def fail_after_stop(fd,state):
            if fd==self.slave and state is True:
                raise OSError('synthetic end-of-run restore failure')
            return original_set(fd,state)
        with patch('experiments.four_bus_human_hold.supervision.os.set_blocking',side_effect=fail_after_stop):
            result=self.synthetic_run('human_current_hold')
        self.assertEqual(result['status'],'ABORTED_HUMAN_SUPERVISION_CLEANUP')
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['failure_retained'])
        self.assertFalse(result['human_supported_trial']['restoration_complete'])
        self.assertIsNotNone(result['human_supported_trial']['failure'])
        self.assert_all_stopped(result)

    def test_old_boxed_route_rejects_human_supervisor_without_opening_devices(self):
        fake=HumanSupported30Execution(self.slave,clock=self.clock,thread_factory=ParkedThread)
        self.addCleanup(fake.close)
        with self.assertRaisesRegex(ValueError,'Dedicated genuine'):
            self.proposed.run(admitted30(),factory=self.factory,imu_read=self.imu,observer=cases.Observer(),
                check_current=self.current,check_cancelled=self.check,cancel_io=self.cancel_io,
                model_setup=lambda *_a,**_k:None,worker_scope=self.worker_scope,main_scope=self.main_scope,
                release_wait=self.release_wait,announce=lambda:None,execute=True,
                backend_usage={'kind':'injected_file_only_mock'},human_supervision=fake)
        self.assertEqual(self.adapters,{})


if __name__=='__main__': unittest.main()
