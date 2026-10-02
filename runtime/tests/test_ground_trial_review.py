"""Synthetic protocol/video association fixtures; these are NOT robot evidence."""
import copy
import json
import struct
import tempfile
from pathlib import Path
import unittest

from singularitydog_hw import ground_trial_review as review
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.native_active_transport import encode_motion
from singularitydog_hw.motor_version_probe import version_request
from test_policy_live_profile import synthetic_fixture


def raw(obj):return json.dumps(obj,sort_keys=True,allow_nan=False).encode()

def wire(can_id,data):return b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'

def native_record(tx,rx,t):
    return dict(tx_hex=tx.hex(),rx_hex=rx.hex(),start_ns=t,finish_ns=t+1000,
                read_start_ns=t+2000,received_ns=t+3000,deadline_ns=t+1_000_000,written=17,received=17)


def fixture(base,stage='supported_stance'):
    profile,docs,_=synthetic_fixture(base)
    profile['duration_s']=4.;profile['policy_weight']=1.
    profile['startup_duration_s']=.2;profile['policy_ramp_s']=.2;profile['stop_duration_s']=.2
    plan={'stage':stage,'maximum_measured_distance_m':.15 if stage=='walk' else None,
          'trajectory':dict(stage=stage,duration_s=4.,initial_hold_s=.5,active_duration_s=1.,
             forward_velocity_m_s=.02 if stage=='walk' else 0.,ramp_up_s=.5 if stage=='walk' else 0.,
             ramp_down_s=.5 if stage=='walk' else 0.,final_stationary_s=.5,resupport_window_s=1.,shutdown_reserve_s=.5)}
    journal=[];start=2_000_000_000
    q={};payload={}
    for mid in range(1,13):
        a=profile['axes'][str(mid)];p=int(((a['physical_lower_rad']+a['physical_upper_rad'])/2+12.57)*65535/25.14)
        payload[mid]=struct.pack('>4H',p,32767,32767,250)
        q[mid]=p*25.14/65535-12.57
    def batch(bus,phase,records):
        return {'bus':bus,'phase':phase,'error':None,'rejected_total':0,'rejected_hex':'','records':records}
    def reply(mid,mode=2,fault=0):return wire((2<<24)|(mode<<22)|(fault<<16)|(mid<<8)|codec.HOST_ID,payload[mid])
    for bus,ids in review.BUSES.items():
        records=[]
        for mid in ids:
            t=start-50_000_000+mid*10_000
            records.append(native_record(codec.read_request(mid),wire((mid<<8)|0xfe,bytes.fromhex(profile['axes'][str(mid)]['uid'])),t))
            version_hex = docs['hardware_review']['device_watchdog'][str(mid)]['version_bytes_hex']
            records.append(native_record(version_request(mid),
                wire((2<<24)|(mid<<8)|codec.HOST_ID,b'\x00\xc4\x56'+bytes.fromhex(version_hex)+b'\xa5'),t+4000))
            for name,offset in (('run_mode',8000),('voltage',12000),('can_timeout',20000)):
                if name=='can_timeout':
                    setup=protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=mid)
                    records.append(native_record(setup,reply(mid,0),t+16000))
                ix,fmt,_=codec.PARAMETERS[name];val={'run_mode':0,'voltage':39.,'can_timeout':4000}[name]
                body=struct.pack('<H',ix)+b'\x00\x00'+struct.pack('<'+fmt,val).ljust(4,b'\x00')
                records.append(native_record(codec.read_request(mid,name),wire((17<<24)|(mid<<8)|codec.HOST_ID,body),t+offset))
        journal.append(batch(bus,'preflight',records))
    cycles=[]
    for index in range(111):
        begin=start+index*20_000_000;end=begin+10_000_000
        final=index==110
        gains=(0.,0.) if final else (6.,.15)
        out=[]
        for bus,ids in review.BUSES.items():
            ins=[];outs=[]
            for order,mid in enumerate(ids):
                tx=encode_motion(mid,q[mid],*gains)
                ins.append(native_record(tx,reply(mid),begin+order*10_000))
                r=native_record(tx,reply(mid),begin+5_000_000+order*10_000);outs.append(r);out.append(r)
            # Refresh every axis in fixture; actual runner refreshes a rotating pair.
            for order,mid in enumerate(ids):
                body=struct.pack('<H',codec.PARAMETERS['voltage'][0])+b'\x00\x00'+struct.pack('<f',39.)
                ins.append(native_record(codec.read_request(mid,'voltage'),wire((17<<24)|(mid<<8)|codec.HOST_ID,body),begin+100_000+order*10_000))
            journal.append(batch(bus,'feedback_hold',ins));journal.append(batch(bus,'graceful_stop' if final else 'policy_output',outs))
        sample=dict(q_model_rad=list(q.values()),velocity_rad_s=[32767*100/65535-50.]*12,
                    torque_nm=[32767*11/65535-5.5]*12,temperature_c=[25.]*12,monotonic_s=(begin+5_000_000)/1e9)
        imu=dict(frame='sensor',read_started_monotonic_ns=begin+100,
                 read_finished_monotonic_ns=begin+3_000_000,accel_m_s2=[0.,0.,-9.81],gyro_rad_s=[0.,0.,0.])
        command=dict(q_model_rad=list(q.values()),kp=[gains[0]]*12,kd=[gains[1]]*12,command_velocity_rad_s=[0.]*12,
                     velocity_reference_rad_s=[0.]*12,feedforward_torque_nm=[0.]*12,gain_scale=0. if final else 1.,
                     phase='stopped' if final else 'active',stop_stage='complete' if final else None,
                     monotonic_s=(begin+4_000_000)/1e9)
        cycles.append(dict(index=index,begin_ns=begin,end_ns=end,phase=command['phase'],feedback=sample,command=command,
            imu=imu,effective_policy_weight=1.,deadline20ms_missed=False,
            oldest_input_to_final_host_write_ms=(max(r['finish_ns'] for r in out)-begin)/1e6))
    stops={}
    for bus,ids in review.BUSES.items():
        rows=[native_record(protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=mid),reply(mid,0),cycles[-1]['end_ns']+1_000_000+order*10_000) for order,mid in enumerate(ids)]
        stops[bus]={'confirmed_ids':list(ids),'complete':True,'evidence':{'records':rows}}
    runtime=dict(status='COMPLETE_SUPPORTED_OUTPUT',errors=[],normal_ramp_completed=True,motor_enable_sent=True,
                 command_output_sent=True,actual_model_calls=len(cycles),model_provenance={'manifest_sha256':profile['artifacts']['model_manifest']['sha256']},
                 fixed_offsets_rad_by_id={str(i):0. for i in range(1,13)},journal=journal,cycles=cycles,
                 stop_confirmed=True,stop_faults_by_id={},stop_reports=stops)
    cues=[dict(key='GROUND_TIMELINE_STARTED',monotonic_ns=start)]
    for key,t in (('initial_hold',0.),('active_window_open',.5),('active_window_close',1.5),('resupport_window_open',2.)):
        cues.append(dict(key=key,monotonic_ns=start+int(t*1e9),scheduled_elapsed_s=t))
    if stage!='supported_stance':cues.append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=start+2_050_000_000))
    report=dict(schema='singularitydog.ground-trial-report.v1',stage=stage,scope='bounded_ground_characterization',
        status='COMPLETE_BOUNDED_GROUND_TRIAL',execution_kind='hardware',simulation_only=False,early_stop_requested=False,
        stage_plan_sha256=review.digest(raw(plan)),profile_sha256=review.digest(raw(profile)),assembly_id=profile['assembly_id'],
        boot_id=profile['boot_id'],motor_power_epoch=profile['motor_power_epoch'],runtime_report=runtime,
        planned_trajectory=plan['trajectory'],cues=cues,errors=[])
    return profile,docs,plan,report


class GroundReviewTests(unittest.TestCase):
    def test_supported_only_usb_waiver_cannot_be_ground_stage_evidence(self):
        self.profile['watchdog_review_policy']='command_loss_only_supported_trial'
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL')
        self.assertIn('Supported-only',str(result['errors']))

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.base=Path(self.temp.name)
        self.profile,self.docs,self.plan,self.report=fixture(self.base)

    def evaluate(self,*,human=True,alter_review=None,alter_association=None):
        from singularitydog_hw.ground_trial_trajectory import GroundTimeline
        self.report['effective_timing']=GroundTimeline(**self.plan['trajectory']).timing
        for event in self.report['cues']:
            if not isinstance(event,dict):continue
            if 'scheduled_elapsed_s' in event:event.setdefault('emitted_ns',event['monotonic_ns'])
            if event.get('key')=='OPERATOR_RESUPPORT_ACK':
                event.setdefault('cue_emitted_ns',4_000_000_000)
                event['accepted_ns']=event['monotonic_ns']+10_000_000
                event.setdefault('cue_step',100);event.setdefault('accepted_step',101)
                self.report['resupport_ack']={'monotonic_ns':event['monotonic_ns']}
        self.report['stage_plan_sha256']=review.digest(raw(self.plan))
        self.report['profile_sha256']=review.digest(raw(self.profile))
        refs={name:{'path':name+'.json','sha256':review.digest(raw(obj))} for name,obj in
              (('report',self.report),('plan',self.plan),('profile',self.profile))}
        refs['video']={'path':'source.mp4','sha256':'b'*64}
        association=dict(schema=review.ASSOCIATION_SCHEMA,references=refs,
            sync=dict(trial_start_ns=2_000_000_000,trial_end_ns=4_250_000_000,video_start_s=1.,video_end_s=3.25,uncertainty_ms=10.),
            synchronization_method='Visible terminal cue and original audio, synthetic fixture only')
        if alter_association:alter_association(association)
        physical=review.physical_review_template()
        physical.update(decision='APPROVE_THIS_RECORDED_STAGE',association_sha256=review.digest(raw(association)),
                        reviewed_by='SYNTHETIC UNIT TEST',reviewed_at='2026-09-28T20:00:00+09:00')
        for k in physical['observations']:physical['observations'][k]=k not in ('slip_observed','body_sinking_observed','overload_observed','unexpected_contact_observed','unplanned_catch_required')
        physical['observations']['body_support_removed_during_active_window']=self.report['stage'] in ('stand','walk')
        if alter_review:alter_review(physical)
        return review.evaluate_bytes(raw(self.report),raw(self.plan),raw(self.profile),raw(association),video_sha256='b'*64,
            physical_review_raw=raw(physical) if human else None,mount_raw=raw(self.docs['mount']),bias_raw=raw(self.docs['bias']),
            hardware_review_raw=raw(self.docs['hardware_review']))

    def test_complete_human_review_records_one_stage_not_authorization(self):
        result=self.evaluate();self.assertEqual(result['status'],'PASS_REVIEWED_STAGE',result)
        self.assertTrue(result['dependency_eligible']);self.assertTrue(result['actual_controller_20ms_pass'])
        self.assertFalse(result['full_controller_50Hz_verified']);self.assertIsNone(result['load_percentage'])
        self.assertEqual(result['metrics']['cycles'],111)
        self.assertEqual(result['metrics']['max_release_interval_ms'],20.)
        self.assertEqual(result['metrics']['release_intervals_over_21ms'],0)

    def test_final_stop_records_cannot_override_explicit_unconfirmed_bus_result(self):
        original=copy.deepcopy(self.report)
        for field,value in (('ambiguous_ids',[1]),('unconfirmed_ids',[1]),
                            ('complete',False),('complete',1),
                            ('errors',['unresolved Enable reply']),
                            ('error','deadline exceeded'),
                            ('sticky_boundary_uncertain',True)):
            with self.subTest(field=field,value=value):
                self.report=copy.deepcopy(original)
                self.report['runtime_report']['stop_reports']['front'][field]=value
                result=self.evaluate()
                self.assertEqual(result['status'],'FAIL',result)
                self.assertFalse(result['dependency_eligible'])
                self.assertFalse(result['all_axis_stop_confirmed'])

    def test_final_stop_bus_confirmed_ids_must_be_exact_integer_unique_set(self):
        original=copy.deepcopy(self.report)
        for ids in ([2,3,4,5,6],[1,1,2,3,4,5,6],[True,2,3,4,5,6],
                    ['1',2,3,4,5,6],[1,2,3,4,5,7],None):
            with self.subTest(ids=ids):
                self.report=copy.deepcopy(original)
                self.report['runtime_report']['stop_reports']['front']['confirmed_ids']=ids
                result=self.evaluate()
                self.assertEqual(result['status'],'FAIL',result)
                self.assertFalse(result['dependency_eligible'])
                self.assertFalse(result['all_axis_stop_confirmed'])

    def test_wrong_or_missing_firmware_rejects_stage_even_with_good_motion(self):
        original=copy.deepcopy(self.report)
        for missing in (True, False):
            with self.subTest(missing=missing):
                self.report=copy.deepcopy(original)
                records=self.report['runtime_report']['journal'][0]['records']
                index=next(i for i,r in enumerate(records) if r['tx_hex']==version_request(1).hex())
                if missing: records.pop(index)
                else: records[index]['rx_hex']=wire((2<<24)|(1<<8)|codec.HOST_ID,bytes.fromhex('00c45601020304a5')).hex()
                result=self.evaluate();self.assertEqual(result['status'],'FAIL');self.assertFalse(result['dependency_eligible'])

    def test_missing_or_invalid_watchdog_ack_rejects_stage_even_with_good_motion(self):
        original=copy.deepcopy(self.report)
        setup=protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=1).hex()
        for failure in ('missing','send_only','partial','wrong_source','wrong_host','active','fault','version','wrong_request'):
            with self.subTest(failure=failure):
                self.report=copy.deepcopy(original)
                records=self.report['runtime_report']['journal'][0]['records']
                row=next(r for r in records if r['tx_hex']==setup)
                frame=review._frame(row['rx_hex'])
                if failure=='missing': records.remove(row)
                elif failure=='send_only': row.update(rx_hex='',received=0,read_start_ns=0,received_ns=0)
                elif failure=='partial': row.update(rx_hex=row['rx_hex'][:-2],received=16)
                elif failure=='wrong_request':
                    tx=review._frame(row['tx_hex'])
                    row['tx_hex']=wire(tx.can_id,bytes.fromhex('28700000a10f0000')).hex()
                elif failure=='version':
                    row['rx_hex']=wire(frame.can_id,bytes.fromhex('00c45605001300a5')).hex()
                else:
                    can_id=(2<<24)|((2 if failure=='active' else 0)<<22)|((1 if failure=='fault' else 0)<<16)
                    can_id|=((2 if failure=='wrong_source' else 1)<<8)|(0xfe if failure=='wrong_host' else codec.HOST_ID)
                    row['rx_hex']=wire(can_id,frame.data).hex()
                result=self.evaluate()
                self.assertEqual(result['status'],'FAIL',result)
                self.assertFalse(result['dependency_eligible'])

    def test_watchdog_readback_must_start_strictly_after_complete_ack(self):
        original=copy.deepcopy(self.report)
        setup=protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=1).hex()
        for delta in (-1,0):
            with self.subTest(readback_minus_ack_ns=delta):
                self.report=copy.deepcopy(original)
                records=self.report['runtime_report']['journal'][0]['records']
                ack=next(r for r in records if r['tx_hex']==setup)
                readback=next(r for r in records if r['tx_hex']==codec.read_request(1,'can_timeout').hex())
                # Both individual records remain valid; only their causal order fails.
                readback['start_ns']=ack['received_ns']+delta
                self.assertGreater(readback['start_ns'],ack['finish_ns'])
                result=self.evaluate()
                self.assertEqual(result['status'],'FAIL',result)
                self.assertFalse(result['dependency_eligible'])

    def test_all_watchdog_acks_must_precede_output(self):
        records=self.report['runtime_report']['journal'][0]['records']
        setup=protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=1).hex()
        ack=next(r for r in records if r['tx_hex']==setup)
        readback=next(r for r in records if r['tx_hex']==codec.read_request(1,'can_timeout').hex())
        for row in (ack,readback):
            for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                row[key]+=60_000_000
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertFalse(result['dependency_eligible'])

    def test_version_response_cannot_substitute_for_final_stop(self):
        row=self.report['runtime_report']['stop_reports']['front']['evidence']['records'][0]
        row['rx_hex']=wire((2<<24)|(1<<8)|codec.HOST_ID,bytes.fromhex('00c45605001300a5')).hex()
        self.assertEqual(self.evaluate()['status'],'FAIL')

    def add_release_gap(self, delta_ns, index=50):
        """Shift a complete synthetic suffix; preserve all wire/timestamp links."""
        runtime=self.report['runtime_report']; boundary=runtime['cycles'][index]['begin_ns']
        for cycle in runtime['cycles'][index:]:
            cycle['begin_ns']+=delta_ns;cycle['end_ns']+=delta_ns
            cycle['feedback']['monotonic_s']+=delta_ns/1e9
            cycle['command']['monotonic_s']+=delta_ns/1e9
            for key in ('read_started_monotonic_ns','read_finished_monotonic_ns'):
                cycle['imu'][key]+=delta_ns
        batches=[*runtime['journal'],*(stop['evidence'] for stop in runtime['stop_reports'].values())]
        for batch in batches:
            for row in batch['records']:
                if row['start_ns']>=boundary:
                    for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                        row[key]+=delta_ns

    def test_forged_release_interval_is_recomputed_from_actual_begin_timestamps(self):
        cycle=self.report['runtime_report']['cycles'][1];cycle['release_interval_ms']=19.
        result=self.evaluate(human=False)
        self.assertEqual(result['status'],'FAIL')
        self.assertIn('Reported release interval',str(result['errors']))
        cycle['release_interval_ms']=20.
        self.report['runtime_report']['cycles'][0]['release_interval_ms']=0.
        self.assertIn('Reported release interval',str(self.evaluate(human=False)['errors']))

    def test_repeated_release_timestamp_is_not_a_valid_period(self):
        cycles=self.report['runtime_report']['cycles'];cycles[1]['begin_ns']=cycles[0]['begin_ns']
        result=self.evaluate(human=False)
        self.assertEqual(result['status'],'FAIL')
        self.assertIn('Overlapping/noncausal control cycles',str(result['errors']))

    def test_short_work_with_a_thirty_ms_stand_release_gap_is_not_twenty_ms_evidence(self):
        self.plan['stage']='stand';self.plan['trajectory']['stage']='stand';self.report['stage']='stand'
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_050_000_000))
        self.profile.update(hard_cycle_ms=40.,max_sample_gap_ms=41.,max_sample_age_ms=40.)
        self.add_release_gap(10_000_000)
        result=self.evaluate(human=False)
        self.assertEqual(result['status'],'RECORDED_REVIEW_REQUIRED',result)
        self.assertFalse(result['actual_controller_20ms_pass'])
        self.assertFalse(result['dependency_eligible'])
        self.assertEqual(result['metrics']['max_iteration_ms'],10.)
        self.assertEqual(result['metrics']['deadline20ms_misses'],0)
        self.assertEqual(result['metrics']['max_release_interval_ms'],30.)
        self.assertEqual(result['metrics']['release_intervals_over_21ms'],1)

    def test_one_ms_release_jitter_is_separate_from_twenty_ms_work_budget(self):
        self.add_release_gap(1_000_000)
        result=self.evaluate(human=False)
        self.assertTrue(result['actual_controller_20ms_pass'],result)
        self.assertFalse(result['strict_controller_20ms_pass'])
        self.assertEqual(result['metrics']['max_release_interval_ms'],21.)
        self.assertEqual(result['metrics']['release_intervals_over_20ms'],1)
        self.assertEqual(result['metrics']['release_intervals_over_21ms'],0)
        self.assertFalse(result['metrics']['strict_start_interval_20ms_met'])
        self.assertEqual(result['metrics']['max_iteration_ms'],10.)

    def test_walk_rejects_release_gap_even_when_every_work_interval_is_short(self):
        self.plan['stage']='walk';self.plan['trajectory'].update(stage='walk',forward_velocity_m_s=.02,ramp_up_s=.5,ramp_down_s=.5)
        self.plan['maximum_measured_distance_m']=.15;self.report['stage']='walk'
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_050_000_000))
        # The reviewer must independently reject a cadence defect; a supplied
        # relaxed profile must not make short computation look like 50Hz output.
        self.profile.update(hard_cycle_ms=40.,max_sample_gap_ms=41.,max_sample_age_ms=40.)
        self.add_release_gap(10_000_000)
        result=self.evaluate(human=False)
        self.assertEqual(result['status'],'FAIL')
        self.assertFalse(result['actual_controller_20ms_pass'])
        self.assertIn('release intervals <=21ms',str(result['errors']))

    def test_no_human_review_cannot_pass(self):
        result=self.evaluate(human=False);self.assertEqual(result['status'],'RECORDED_REVIEW_REQUIRED',result)
        self.assertFalse(result['dependency_eligible'])

    def test_simulated_and_replayed_even_complete_cannot_release(self):
        for kind in ('simulation','replay'):
            self.report['execution_kind']=kind
            result=self.evaluate();self.assertEqual(result['status'],'RECORDED_REVIEW_REQUIRED',result)
            self.assertFalse(result['genuine_hardware_capture'])

    def test_missing_stop_or_fault_fails(self):
        self.report['runtime_report']['stop_reports']['rear']['evidence']['records'].pop()
        self.assertIn('STOP incomplete',str(self.evaluate()['errors']))

    def test_every_final_stop_must_follow_the_last_motion_cycle(self):
        runtime=self.report['runtime_report']
        rows=runtime['stop_reports']['front']['evidence']['records']
        # Eleven fresh replies must not conceal one STOP taken before output.
        row=rows[0]
        for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
            row[key]-=500_000_000
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertFalse(result['all_axis_stop_confirmed'])
        self.assertIn('STOP preceded final control/output completion',str(result['errors']))

    def test_motion_after_final_stop_cannot_be_reviewed_as_stopped(self):
        runtime=self.report['runtime_report']
        batch=copy.deepcopy(runtime['journal'][-1])
        row=batch['records'][0]
        latest=max(r['received_ns'] for stop in runtime['stop_reports'].values()
                   for r in stop['evidence']['records'])
        shift=latest+1_000_000-row['start_ns']
        for item in batch['records']:
            for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                item[key]+=shift
        batch['phase']='startup_zero_gain'
        runtime['journal'].append(batch)
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertFalse(result['all_axis_stop_confirmed'])

    def test_final_stop_requires_the_canonical_stop_command(self):
        row=self.report['runtime_report']['stop_reports']['front']['evidence']['records'][0]
        frame=review._frame(row['tx_hex'])
        row['tx_hex']=wire(frame.can_id,b'\x01'+frame.data[1:]).hex()
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertIn('Final STOP request mismatch',str(result['errors']))

    def test_logged_zero_velocity_cannot_hide_a_target_position_jump(self):
        runtime=self.report['runtime_report'];cycle=runtime['cycles'][1]
        old=cycle['command']['q_model_rad'][0]
        delta=self.profile['axes']['1']['max_command_velocity_rad_s']*.02*2
        cycle['command']['q_model_rad'][0]=old+delta
        row=next(row for batch in runtime['journal'] if batch['phase']=='policy_output'
                 for row in batch['records'] if row['start_ns']==cycle['begin_ns']+5_000_000)
        row['tx_hex']=encode_motion(1,old+delta,6.,.15).hex()
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertIn('Actual target position slew',str(result['errors']))

    def test_normal_ramp_summary_cannot_replace_final_zero_gain_output(self):
        runtime=self.report['runtime_report'];cycle=runtime['cycles'][-1]
        cycle['command'].update(kp=[6.]*12,kd=[.15]*12,gain_scale=1.)
        for batch in runtime['journal']:
            if batch['phase']!='graceful_stop':continue
            for row in batch['records']:
                frame=review._frame(row['tx_hex']);mid=frame.destination
                row['tx_hex']=encode_motion(mid,cycle['command']['q_model_rad'][mid-1],6.,.15).hex()
        result=self.evaluate()
        self.assertEqual(result['status'],'FAIL',result)
        self.assertIn('Final command did not complete zero-gain ramp',str(result['errors']))

    def test_nominal_slew_does_not_differentiate_quantized_wire_positions(self):
        runtime=self.report['runtime_report'];cycles=runtime['cycles']
        axis=self.profile['axes']['1'];axis['max_command_velocity_rad_s']=.0001
        lsb=25.14/65535
        boundary=cycles[0]['command']['q_model_rad'][0]+lsb
        for index,cycle in enumerate(cycles):
            cycle['command']['q_model_rad'][0]=boundary+(-1e-7 if index<50 else 1e-7)
        by_begin={cycle['begin_ns']:cycle for cycle in cycles}
        for batch in runtime['journal']:
            if batch['phase'] not in ('policy_output','graceful_stop'):continue
            for row in batch['records']:
                frame=review._frame(row['tx_hex'])
                if frame.destination!=1:continue
                command=by_begin[row['start_ns']-5_000_000]['command']
                row['tx_hex']=encode_motion(1,command['q_model_rad'][0],command['kp'][0],command['kd'][0]).hex()
        before=review._frame(encode_motion(1,cycles[49]['command']['q_model_rad'][0],6.,.15).hex())
        after=review._frame(encode_motion(1,cycles[50]['command']['q_model_rad'][0],6.,.15).hex())
        self.assertEqual(int.from_bytes(after.data[:2],'big')-int.from_bytes(before.data[:2],'big'),1)
        self.assertGreater(lsb/.02,axis['max_command_velocity_rad_s'])
        # Wire quantization is not an observed velocity or the nominal target
        # derivative. The original continuous target remains the checked value.
        result=self.evaluate()
        self.assertEqual(result['status'],'PASS_REVIEWED_STAGE',result)

    def test_unrecognized_or_nonfinite_telemetry_fails(self):
        for field in ('temperature_c','torque_nm','velocity_rad_s','q_model_rad'):
            old=self.report['runtime_report']['cycles'][0]['feedback'][field]
            self.report['runtime_report']['cycles'][0]['feedback'][field]=[None]*12
            self.assertEqual(self.evaluate()['status'],'FAIL')
            self.report['runtime_report']['cycles'][0]['feedback'][field]=old

    def test_actual_bytes_not_summary_are_used(self):
        row=self.report['runtime_report']['journal'][-1]['records'][0]
        tx=bytearray.fromhex(row['tx_hex']);tx[8]^=8;row['tx_hex']=tx.hex()
        result=self.evaluate();self.assertIn('Actual command differs',str(result['errors']))

    def test_human_anomaly_and_unknown_cannot_pass(self):
        for field,value in (('slip_observed',True),('physical_stop_verified',False),('body_sinking_observed',None)):
            result=self.evaluate(alter_review=lambda r:r['observations'].__setitem__(field,value))
            self.assertEqual(result['status'],'FAIL')

    def test_video_interval_must_cover_stop_and_match_source_hash(self):
        result=self.evaluate(alter_association=lambda a:a['references']['report'].__setitem__('sha256','c'*64))
        self.assertIn('association mismatch',str(result['errors']))
        result=self.evaluate(alter_association=lambda a:a['sync'].__setitem__('trial_end_ns',4_000_000_000))
        self.assertEqual(result['status'],'FAIL')

    def test_repeat_imu_noncausal_and_tilt_rejected(self):
        cycle=self.report['runtime_report']['cycles'][1];imu=cycle['imu']
        imu['read_started_monotonic_ns']=self.report['runtime_report']['cycles'][0]['imu']['read_started_monotonic_ns']
        self.assertIn('IMU timestamp',str(self.evaluate()['errors']))
        imu['read_started_monotonic_ns']=cycle['begin_ns']+100;imu['accel_m_s2']=[9.81,0.,0.]
        self.assertIn('IMU tilt',str(self.evaluate()['errors']))

    def test_missed_cycle_and_recomputed20ms_guard(self):
        self.report['runtime_report']['cycles'][0]['deadline20ms_missed']=True
        self.assertIn('20ms miss classification',str(self.evaluate()['errors']))

    def test_stand_requires_full_learned_output_and_fresh_resupport(self):
        self.plan['stage']='stand';self.plan['trajectory']['stage']='stand';self.report['stage']='stand'
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_050_000_000))
        self.assertEqual(self.evaluate()['status'],'PASS_REVIEWED_STAGE')
        self.report['cues'][-1]['monotonic_ns']=3_000_000_000
        self.assertIn('re-support acknowledgement',str(self.evaluate()['errors']))
        self.report['cues'][-1]['monotonic_ns']=4_050_000_000
        for cycle in self.report['runtime_report']['cycles']:cycle['effective_policy_weight']=.1
        self.assertIn('full learned output',str(self.evaluate()['errors']))

    def test_early_stop_is_not_completed_stage(self):
        self.report['early_stop_requested']=True
        self.assertIn('Early-stopped',str(self.evaluate()['errors']))

    def test_walk_distance_is_measurement_not_command_integration(self):
        self.plan['stage']='walk';self.plan['trajectory'].update(stage='walk',forward_velocity_m_s=.02,ramp_up_s=.5,ramp_down_s=.5)
        self.plan['maximum_measured_distance_m']=.15;self.report['stage']='walk'
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_050_000_000))
        def measured(r):r['measured_distance']=dict(distance_m=.08,uncertainty_m=.01,method='video_with_measured_reference',reference='Marked 10cm ruler in source video')
        result=self.evaluate(alter_review=measured);self.assertEqual(result['status'],'PASS_REVIEWED_STAGE',result)
        def commanded(r):measured(r);r['measured_distance']['method']='command_velocity_times_time'
        self.assertEqual(self.evaluate(alter_review=commanded)['status'],'FAIL')

    def test_enter_before_actual_visible_cue_cannot_count_as_resupport(self):
        self.plan['stage']='stand';self.plan['trajectory']['stage']='stand';self.report['stage']='stand'
        self.report['cues'][-1]['emitted_ns']=4_010_000_000
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_005_000_000))
        self.assertIn('re-support acknowledgement',str(self.evaluate()['errors']))

    def test_standing_requires_video_observed_self_support(self):
        self.plan['stage']='stand';self.plan['trajectory']['stage']='stand';self.report['stage']='stand'
        self.report['cues'].append(dict(key='OPERATOR_RESUPPORT_ACK',monotonic_ns=4_050_000_000))
        result=self.evaluate(alter_review=lambda r:r['observations'].__setitem__('catch_non_load_bearing_during_standing',False))
        self.assertIn('Self-support',str(result['errors']))

    def test_low_voltage_and_fault_bytes_fail(self):
        record=next(r for r in self.report['runtime_report']['journal'][0]['records']
                    if r['tx_hex']==codec.read_request(1,'voltage').hex())
        saved=record['rx_hex'];frame=review._frame(saved)
        record['rx_hex']=wire(frame.can_id,frame.data[:4]+struct.pack('<f',30.)).hex()
        self.assertIn('Voltage outside',str(self.evaluate()['errors']))
        record['rx_hex']=saved
        record=self.report['runtime_report']['journal'][-1]['records'][0];frame=review._frame(record['rx_hex'])
        record['rx_hex']=wire(frame.can_id|(1<<16),frame.data).hex()
        self.assertIn('fault bits',str(self.evaluate()['errors']))

    def test_duplicate_json_and_malformed_event_fail_without_raising(self):
        result=review.evaluate_bytes(b'{"stage":1,"stage":2}',raw(self.plan),raw(self.profile),b'{}',video_sha256='b'*64)
        self.assertEqual(result['status'],'FAIL')
        self.report['cues'].append(None)
        self.assertEqual(self.evaluate()['status'],'FAIL')

    def test_missing_imu_norm_never_becomes_zero_tilt(self):
        self.report['runtime_report']['cycles'][0]['imu']['accel_m_s2']=[0.,0.,0.]
        result=self.evaluate();self.assertIn('acceleration norm',str(result['errors']))

    def test_command_velocity_and_gain_inconsistency_fail(self):
        cycle=self.report['runtime_report']['cycles'][0]
        cycle['command']['command_velocity_rad_s'][0]=.11
        self.assertIn('Command velocity',str(self.evaluate()['errors']))
        cycle['command']['command_velocity_rad_s'][0]=0.
        cycle['command']['feedforward_torque_nm'][0]=.1
        self.assertIn('feedforward',str(self.evaluate()['errors']))

    def test_uid_mismatch_rejected(self):
        self.profile['axes']['7']['uid']='f'*16
        self.assertIn('UID mismatch',str(self.evaluate()['errors']))


if __name__=='__main__':unittest.main()
