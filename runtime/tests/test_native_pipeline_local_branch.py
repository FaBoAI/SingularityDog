"""Pinned diagnostic input branches; synthetic saved files, no devices or weights."""
import copy
import hashlib
import json
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_shadow as shadow
import test_native_pipeline_active_fk as active_fixture
from test_policy_observer import FakeTorch, Policy, mount, snapshot


class LocalBranchDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.fixture=active_fixture.ActiveFKDiagnosticTests();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.profile=copy.deepcopy(self.fixture.data)
        self.capture=copy.deepcopy(self.fixture.docs['local_reference_capture'])
        self.calibration=copy.deepcopy(self.fixture.docs['calibration'])
        self.profile['start_pose_bounds']={}
        for mid,axis in self.profile['axes'].items():
            q=self.capture['telemetry']['rows'][mid]['median_position_rad']
            self.profile['start_pose_bounds'][mid]=[q-math.radians(.5),q+math.radians(.5)]
        self.context=dict(profile=self.profile,
            reference=dict(path=str(self.fixture.path),sha256=self.fixture.digest))

    def bind_capture(self):
        path=self.fixture.base/'local_reference_capture.json'
        raw=(json.dumps(self.capture,sort_keys=True)+'\n').encode();path.write_bytes(raw)
        self.profile['artifacts']['local_reference_capture']=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())

    def derive(self):
        self.bind_capture()
        return bench._active_fk_observer_calibration(self.context,self.calibration)

    def selected(self):
        calibration,proof=self.derive()
        self.context.update(observer_calibration=calibration,local_branch_provenance=proof)
        return calibration,proof

    def live_snapshot(self, tick=1_000_000_000):
        value=snapshot(tick,calib=self.calibration)
        for row in value['motors']:
            if row['parameter']=='position':
                row['value']=self.capture['telemetry']['rows'][str(row['motor_id'])]['median_position_rad']
        return value

    def ordinary(self, calibration):
        policy=Policy()
        run=observer.StatefulPolicyObserver(policy,calibration,imu_mount_candidate=mount(),
            h_hypothesis=0,command=[0.,0.,0.],max_ticks=5,max_age_ns=10_000_000,
            max_spread_ns=5_000_000,torch_module=FakeTorch,measured_diagnostic_ticks=True)
        run.prepare_run(warmup_completed=True);run.arm_run(1_000_000_000)
        return run,policy

    def test_all_twelve_positive_negative_turns_apply_once_without_raw_or_nominal_mutation(self):
        original=copy.deepcopy((self.profile,self.capture,self.calibration))
        for mid in range(1,13):
            for turn in (-1,0,1):
                with self.subTest(mid=mid,turn=turn):
                    self.profile,self.capture,self.calibration=copy.deepcopy(original)
                    self.context['profile']=self.profile
                    row=self.capture['telemetry']['rows'][str(mid)]
                    row['median_position_rad']+=turn*2*math.pi
                    row['position_samples']=[{**sample,'rad':sample['rad']+turn*2*math.pi}
                        for sample in row['position_samples']]
                    before=copy.deepcopy((self.capture,self.calibration))
                    derived,proof=self.derive()
                    self.assertEqual(proof['reference_turns_by_id'][str(mid)],turn)
                    a=self.profile['axes'][str(mid)];c=derived['candidates'][mid-1]
                    self.assertEqual(c['offset_candidate_rad'],a['offset_rad']-a['sign']*turn*2*math.pi)
                    self.assertAlmostEqual(a['sign']*row['median_position_rad']+c['offset_candidate_rad'],
                        proof['model_rad_by_id'][str(mid)])
                    self.assertEqual((self.capture,self.calibration),before)
                    self.assertFalse(proof['approved_for_runtime']);self.assertFalse(proof['physical_branch_or_motion_proven'])
                    self.assertFalse(proof['motor_output_allowed']);self.assertIsNone(proof['absolute_calibration_error_rad'])

    def test_id11_reproduces_unadjusted_failure_then_consumes_correct_model_input(self):
        mid='11';raw=.166295719844358;offset=6.873025242495491
        axis=self.profile['axes'][mid];axis.update(sign=-1,offset_rad=offset)
        self.calibration['candidates'][10].update(sign_candidate=-1,offset_candidate_rad=offset)
        row=self.capture['telemetry']['rows'][mid];row['median_position_rad']=raw
        for sample in row['position_samples']:sample['rad']=raw
        q=-(raw+2*math.pi)+offset
        axis.update(physical_lower_rad=q-math.radians(3),physical_upper_rad=q+math.radians(3))
        self.profile['start_pose_bounds'][mid]=[q-math.radians(.5),q+math.radians(.5)]
        value=self.live_snapshot();before=copy.deepcopy(value)
        unadjusted,policy=self.ordinary(self.calibration)
        with self.assertRaisesRegex(observer.ObserverError,'outside registered joint range'):
            unadjusted.consume(value)
        self.assertEqual(policy.calls,[])
        derived,proof=self.selected();run,policy=self.ordinary(derived)
        guarded=bench._ActiveFKDiagnosticObserver(run,self.context)
        result=guarded.consume(value)
        self.assertEqual(proof['reference_turns_by_id'][mid],-1)
        self.assertAlmostEqual(policy.calls[0][3][shadow.CAN_ORDER.index(11)],q)
        self.assertEqual(value,before)
        self.assertFalse(result['output_allowed'])

    def test_new_boot_derived_copy_uses_fresh_capture_and_keeps_nominal_history(self):
        self.calibration.update(source_current_boot_id='old-nominal-boot',
            source_current_motor_power_epoch_label='old-nominal-power',
            source_capture_sha256='a'*64,source_raw_rad_by_id={'1':123.},
            model_rad_at_source_capture_by_id={'1':456.},
            current_operator_power_statement={'source':'historical-only'},
            diagnostic_branch_derivation={'source':'historical-only'})
        self.capture['boot_id']=self.profile['boot_id']='fresh-observation-boot'
        self.capture['motor_power_epoch']='NOT_INFERRED_FROM_JETSON_BOOT'
        raw_before=copy.deepcopy(self.capture);nominal_before=copy.deepcopy(self.calibration)
        # A negative turn on ID1 exercises the same fixed-branch path after a reboot.
        row=self.capture['telemetry']['rows']['1']
        row['median_position_rad']-=2*math.pi
        row['position_samples']=[{**sample,'rad':sample['rad']-2*math.pi}
            for sample in row['position_samples']]
        raw_before=copy.deepcopy(self.capture)
        derived,proof=self.derive()
        self.assertEqual(proof['reference_turns_by_id']['1'],-1)
        self.assertEqual(derived['source_current_boot_id'],self.profile['boot_id'])
        self.assertEqual(derived['source_current_motor_power_epoch_label'],self.capture['motor_power_epoch'])
        self.assertEqual(derived['source_capture_sha256'],self.profile['artifacts']['local_reference_capture']['sha256'])
        self.assertEqual(derived['source_raw_rad_by_id'],proof['raw_rad_by_id'])
        self.assertEqual(derived['model_rad_at_source_capture_by_id'],proof['model_rad_by_id'])
        nominal_source=derived['diagnostic_nominal_source']
        self.assertEqual(nominal_source['calibration'],self.profile['artifacts']['calibration'])
        self.assertEqual(nominal_source['historical_fields']['source_current_boot_id'],'old-nominal-boot')
        self.assertEqual(nominal_source['historical_fields']['source_capture_sha256'],'a'*64)
        self.assertFalse(nominal_source['original_artifact_modified'])
        self.assertFalse(nominal_source['historical_qualification_reused'])
        self.assertNotIn('current_operator_power_statement',derived)
        self.assertNotIn('diagnostic_branch_derivation',derived)
        self.assertEqual(self.calibration,nominal_before);self.assertEqual(self.capture,raw_before)
        for flag in ('approved_for_runtime','motor_output_available','output_allowed'):
            self.assertIs(derived[flag],False)

    def test_current_boot_rebinding_does_not_accept_a_foreign_or_approved_capture(self):
        self.calibration['source_current_boot_id']='historical-boot'
        for key,value in (('boot_id','foreign-current-boot'),('approved_for_runtime',True)):
            before=copy.deepcopy(self.capture);self.capture[key]=value
            with self.subTest(field=key),self.assertRaises(ValueError):self.derive()
            self.capture=before

    def test_capture_uid_boot_power_span_mode_offset_and_pin_mismatch_reject_before_model_load(self):
        changes=(lambda:self.capture['identities']['11'].update(mcu_uid_hex='f'*16),
            lambda:self.capture.update(boot_id='different'),
            lambda:self.capture.update(motor_power_epoch='different'),
            lambda:self.capture.update(approved_for_runtime=True),
            lambda:self.capture.update(angle_wrap_applied=True),
            lambda:self.capture['telemetry']['rows']['11'].update(run_mode=2),
            lambda:self.capture['telemetry']['rows']['11'].update(current=.1),
            lambda:self.capture['telemetry']['rows']['11'].update(voltage=34.9),
            lambda:self.capture['telemetry']['rows']['11']['position_samples'][0].update(rad=.1),
            lambda:self.calibration['candidates'][10].update(offset_candidate_rad=.1))
        initial=copy.deepcopy((self.capture,self.calibration))
        for change in changes:
            self.capture,self.calibration=copy.deepcopy(initial)
            change()
            with patch.object(bench.native,'load_library') as native,\
                 self.assertRaises(ValueError):self.derive()
            native.assert_not_called()
        self.capture,self.calibration=copy.deepcopy(initial);self.bind_capture()
        Path(self.profile['artifacts']['local_reference_capture']['path']).write_text('{}')
        with self.assertRaisesRegex(ValueError,'SHA256'):
            bench._active_fk_observer_calibration(self.context,self.calibration)

    def test_capture_and_live_timestamps_cannot_be_missing_backward_or_replayed(self):
        initial=copy.deepcopy(self.capture)
        for failure in ('uid_missing','uid_reverse','sample_missing','sample_reverse','sample_slow'):
            with self.subTest(failure=failure):
                self.capture=copy.deepcopy(initial)
                identity=self.capture['identities']['11'];sample=self.capture['telemetry']['rows']['11']['position_samples'][0]
                if failure=='uid_missing':identity.pop('request_monotonic_ns')
                elif failure=='uid_reverse':identity['reply_monotonic_ns']=identity['request_monotonic_ns']-1
                elif failure=='sample_missing':sample.pop('reply_monotonic_ns')
                elif failure=='sample_reverse':sample['request_monotonic_ns']=identity['reply_monotonic_ns']-1
                else:sample['reply_monotonic_ns']=sample['request_monotonic_ns']+30_000_001
                with self.assertRaisesRegex(ValueError,'timing|causality'):self.derive()
        self.capture=copy.deepcopy(initial);self.selected()
        run=Mock();guard=bench._ActiveFKDiagnosticObserver(run,self.context)
        value=self.live_snapshot();value['motors'][0]['request_ns']=1
        with self.assertRaisesRegex(observer.ObserverError,'predates local capture'):guard.consume(value)
        run.consume.assert_not_called()

    def test_wrong_local_bounds_and_missing_unique_branch_never_clip(self):
        original=copy.deepcopy((self.profile,self.capture))
        self.profile['axes']['11']['physical_upper_rad']+=.001
        with self.assertRaisesRegex(ValueError,'bounds differ'):self.derive()
        self.profile,self.capture=copy.deepcopy(original);self.context['profile']=self.profile
        row=self.capture['telemetry']['rows']['11'];row['median_position_rad']=1.5
        for sample in row['position_samples']:sample['rad']=1.5
        with self.assertRaisesRegex(ValueError,'Expected one physical branch, got 0'):self.derive()

    def test_passive_diagnostic_accepts_outside_start_but_rejects_physical_exit_and_invalid_input(self):
        self.selected()
        run=Mock();guard=bench._ActiveFKDiagnosticObserver(run,self.context)
        value=self.live_snapshot();row=next(r for r in value['motors']
            if r['motor_id']==11 and r['parameter']=='position')
        row['value']+=math.radians(.6)
        self.assertGreater(row['value'],self.profile['start_pose_bounds']['11'][1])
        guard.consume(value);run.consume.assert_called_once_with(value)
        for failure in ('local','full_turn','missing','nan','duplicate'):
            with self.subTest(failure=failure):
                run=Mock();guard=bench._ActiveFKDiagnosticObserver(run,self.context)
                value=self.live_snapshot();row=next(r for r in value['motors']
                    if r['motor_id']==11 and r['parameter']=='position')
                if failure=='local':row['value']+=math.radians(3.1)
                elif failure=='full_turn':row['value']+=2*math.pi
                elif failure=='missing':value['motors'].remove(row)
                elif failure=='nan':row['value']=float('nan')
                else:value['motors'].append(copy.deepcopy(row))
                before=copy.deepcopy(value)
                with self.assertRaises(observer.ObserverError):guard.consume(value)
                run.consume.assert_not_called()
                if failure!='nan':self.assertEqual(value,before)

    def test_ordinary_freshness_checks_and_model_failure_still_propagate(self):
        derived,_=self.selected();run,policy=self.ordinary(derived)
        guarded=bench._ActiveFKDiagnosticObserver(run,self.context)
        value=self.live_snapshot();value['motors'][0]['request_ns']-=20_000_000
        with self.assertRaises(observer.ObserverError):guarded.consume(value)
        self.assertEqual(policy.calls,[])
        stub=Mock();stub.consume.side_effect=RuntimeError('synthetic model failure')
        guarded=bench._ActiveFKDiagnosticObserver(stub,self.context)
        with self.assertRaisesRegex(RuntimeError,'synthetic model failure'):guarded.consume(self.live_snapshot())


if __name__=='__main__':unittest.main()
