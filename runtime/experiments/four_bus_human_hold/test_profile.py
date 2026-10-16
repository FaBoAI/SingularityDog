"""File-only admission attacks and primary raw-clock/PCM fixtures; no real grants.

Synthetic direct-condition documents and raw packets below are test data. They
are never fed through prepare_from_files into an execution proof or published
as an actual physical observation, timing qualification or source receipt.
"""
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import wave

from singularitydog_hw.policy_clock import control_clock_payload
from singularitydog_hw.native_active_transport import encode_motion
from experiments.four_bus_type1 import test_type1_profile as fixtures
from experiments.four_bus_type1 import test_type1_30hz as thirty
from experiments.four_bus_type1 import type1_profile as boxed
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from . import profile as p


def runtime_profile():
    """Complete new numerical runtime fixture, never admitted."""
    clock = control_clock_payload(30)
    axis = {'kp': 6., 'kd': .15, **copy.deepcopy(p.CAPS), 'sign': 1,
            'fixed_offset_rad': 0., 'lower_rad': -1., 'upper_rad': 1.}
    return dict(schema=p.RUNTIME_SCHEMA, scope=p.SCOPE, diagnostic_timing_acceptance=p.ACCEPTANCE,
        human_supported_hold=copy.deepcopy(p.SETTINGS), duration_s=8., startup_duration_s=1., startup_damping_duration_s=1.,
        stop_duration_s=.4, policy_weight=0., hard_cycle_ms=1000/30,
        max_sample_age_ms=20., inference_tx_budget_ns=20_000_000,
        cycle_budget_ns=33_333_334, reply_budget_ns=33_333_334, control_clock=clock,
        axes={str(i): copy.deepcopy(axis) for i in p.IDS},
        start_pose_bounds={str(i): [-.01, .01] for i in p.IDS}, output_allowed=False)


def condition(contract):
    base = contract['base_numerical_contract']
    return dict(schema=p.CONDITIONS_SCHEMA, source='direct_current_user_reply', direct_human=True,
        user_statement='SYNTHETIC: two people retain full support throughout the test fixture',
        user_reply_id='SYNTHETIC-only', record_written_at='2026-10-14T00:00:00+09:00',
        contract_sha256=p.canonical_sha256(contract), boot_id=base['boot_id'],
        motor_power_epoch=base['motor_power_epoch'], operator_count=2,
        roles={'body_catch': 'synthetic-one', 'terminal_and_cutoff': 'synthetic-two'},
        local_clearance_rad=math.radians(3), capture_sha256='a'*64,
        **dict.fromkeys(p.REQUIRED_CONDITIONS, True), **dict.fromkeys(p.FORBIDDEN_CONDITIONS, False))


def wav_bytes(*, rate=48000, channels=2, width=2, frames=960):
    stream = io.BytesIO()
    with wave.open(stream, 'wb') as w:
        w.setframerate(rate); w.setnchannels(channels); w.setsampwidth(width)
        w.writeframes(bytes(frames*channels*width))
    return stream.getvalue()


def wire(can_id, payload):
    return b'AT'+((can_id << 3)|4).to_bytes(4,'big')+b'\x08'+payload+b'\r\n'


def type2(mid, q, mode):
    return wire((2<<24)|(mode<<22)|(mid<<8)|codec.HOST_ID,
                struct.pack('>4H', int((q+12.57)*65535/25.14),32767,32767,250))


def raw_zero_fixture(contract):
    """Raw source-format fixture only: no Prepared object or execution token."""
    base = contract['base_numerical_contract']
    first_epoch = 6_000_000_000
    journal = {port:[] for port in p.PORTS}
    cycles = []
    def raw(tx, rx, start, received, cap):
        return dict(tx_hex=tx.hex(), rx_hex=rx.hex(), start_ns=start, finish_ns=start+10,
                    read_start_ns=start+11, received_ns=received, deadline_ns=cap, written=17, received=17)
    # Actual Type1 journals contain setup STOP as well as final stop_subset.
    # The old setup request must not be mistaken for final shutdown proof.
    for port, ids in base['topology_by_port'].items():
        records=[]
        for slot,mid in enumerate(ids):
            q=base['model_plan']['provenance']['raw_rad_by_id'][str(mid)]
            start=first_epoch-100_000_000+slot*900_000
            records.append(raw(protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=mid),
                type2(mid,q,0),start,start+100_000,first_epoch))
        journal[port].append(['stop',{'records':records}])
    for index in range(53):
        begin = first_epoch+index*1_000_000_000//30
        hard, write = begin+33_333_334, begin+20_000_000
        row = dict(index=index,slot=index,release_ns=begin,begin_ns=begin,first_ns=begin+500,
            hard_end_ns=hard,write_deadline_ns=write,cycle_end_ns=begin+24_000_000,
            completed=True,request_count=28,weight=0,acquired_ns=begin+6_000_000,
            gather_ns=begin+7_000_000,infer_end_ns=begin+10_000_000,voltage_join_ns=begin+11_000_000,
            final_gate_ns=begin+12_000_000,encode_end_ns=begin+13_000_000,
            output_submit_ns=begin+13_500_000,reply_return_ns=begin+23_000_000,
            imu=dict(frame='sensor',accel_m_s2=[0.,0.,9.8],gyro_rad_s=[0.,0.,0.],
                read_started_monotonic_ns=begin+500,read_finished_monotonic_ns=begin+500_000))
        phase = 'starting' if index<30 else 'active' if index<40 else 'stopping' if index<52 else 'stopped'
        row['command']=dict(phase=phase,stop_stage='complete' if index==52 else 'gain_ramp' if index>=40 else None,
            monotonic_s=(begin+12_000_000)/1e9,
            q_model_rad=[base['model_plan']['provenance']['model_rad_by_id'][str(mid)] for mid in p.IDS],
            **{key:[0.]*12 for key in ('kp','kd','velocity_reference_rad_s','feedforward_torque_nm','command_velocity_rad_s')})
        writes,replies = [],[]
        for phase, count, offset in (('hold',3,1_000_000),('voltage',1,5_000_000),('output',3,14_000_000)):
            batches = {}
            for port, ids in base['topology_by_port'].items():
                records=[]
                for slot,mid in enumerate(ids[:count]):
                    q = base['model_plan']['provenance']['raw_rad_by_id'][str(mid)]
                    if phase=='voltage':
                        tx=codec.read_request(mid,'voltage')
                        rx=wire((17<<24)|(mid<<8)|codec.HOST_ID,struct.pack('<H2xf',0x701c,38.))
                    else:
                        tx=encode_motion(mid,q,0,0);rx=type2(mid,q,2)
                    start = begin+offset+slot*900_000
                    reply = begin+22_000_000 if phase=='output' else start+100_000
                    packet=raw(tx,rx,start,reply,hard)
                    records.append(packet)
                    if phase=='output':writes.append(packet['finish_ns']);replies.append(reply)
                batch=dict(physical_port=port,physical_ids=ids,original_record_count=count,
                    rejected_total_bytes=0,rejected_truncated=False,records=records)
                if phase=='output':batch.update(separate_write_deadline_monotonic_ns=write,record_deadlines_are_reply_deadlines=True)
                batches[port]=batch
                journal[port].append([phase,dict(records=copy.deepcopy(records))])
            row[phase]=batches
        row.update(final_write_ns=max(writes),output_last_reply_ns=max(replies))
        cycles.append(row)
    end=cycles[-1]['cycle_end_ns']
    stop_results={}
    for port,ids in base['topology_by_port'].items():
        records=[]
        for slot,mid in enumerate(ids):
            q=base['model_plan']['provenance']['raw_rad_by_id'][str(mid)]
            start=end+1_000_000+slot*900_000
            records.append(raw(protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=mid),
                type2(mid,q,0),start,start+100_000,end+100_000_000))
        records += [dict(tx_hex='00'*17,rx_hex='00'*17,start_ns=0,finish_ns=0,read_start_ns=0,
                         received_ns=0,deadline_ns=0,written=0,received=0) for _ in range(3)]
        journal[port].append(['stop_subset',{'records':records}])
        stop_results[port]=dict(complete=True,confirmed_ids=ids,unconfirmed_ids=[],ambiguous_ids=[],
            faults={str(mid):0 for mid in ids})
    workers={port:dict(cpu_mask=[i],timer_slack_ns=1000,file_only_mock_readback=False,native_tid=i+100)
             for i,port in enumerate(p.PORTS)}
    workers['imu']=dict(cpu_mask=[0,1,2,3],timer_slack_ns=1000,file_only_mock_readback=False,native_tid=104)
    m=dict(cycles=cycles,raw_journal=journal,stop_results=stop_results,worker_settings=workers,
        backend_usage=dict(kind='genuine_subset_active_cpp',source_manifest_sha256='9'*64,
            library_sha256='8'*64,build_record_sha256='7'*64),
        operating_settings=dict(main_cpu_mask=[4],nice=-10,timer_slack_ns=1000,switch_interval_s=.0001,
            single_thread_math_verified=True,power_scope_verified=True,gc_deferred_during_cycles=True),
        restoration=dict.fromkeys((*p.PORTS,'imu','main',*(port+'_session' for port in p.PORTS)),True),
        stop_confirmed=True,host_watchdog_reason='normal completion',all_workers_joined_monotonic_ns=end+50_000_000)
    return dict(schema=p.REPORT_SCHEMA,mode='human_zero_gain_timing',status='COMPLETE_HUMAN_ZERO_GAIN_TIMING',
        duration_s=2,contract_sha256=p.canonical_sha256(contract),boot_id=base['boot_id'],
        motor_power_epoch=base['motor_power_epoch'],control_clock=base['control_clock'],policy30_model=base['policy30_model'],
        source_manifest_sha256='9'*64,errors=[],failure_retained=False,normal_ramp_completed=True,
        motor_enable_sent=True,type1_sent=True,positive_gain_sent=False,learned_targets_attempted=False,
        physical_cutoff_required=False,restoration_complete=True,source_and_input_pins_unchanged=True,
        completed_cycles=len(cycles),measurement=m,
        input_sha256={'/synthetic/source':'9'*64,'/synthetic/library':'8'*64,'/synthetic/receipt':'7'*64})


class NumericalTests(unittest.TestCase):
    def test_safe_new_clock_caps_and_fade_reserve(self):
        data = runtime_profile()
        settings = p.human_supported_hold_settings(data)
        self.assertAlmostEqual(settings['latest_stop_s'], 8-.2-.4-.066666668)
        self.assertFalse(data['output_allowed'])
        self.assertEqual(data['policy_weight'], 0)

    def test_scope_schema_mode_copies_and_all_expansions_are_rejected(self):
        cases = [lambda x: x.update(scope='human_supported_partial_current_hold_only'),
            lambda x: x.update(schema=boxed.PROFILE_SCHEMA_30),
            lambda x: x.update(duration_s=20), lambda x: x.update(policy_weight=.005),
            lambda x: x.update(hard_cycle_ms=20), lambda x: x.update(max_sample_age_ms=33.33),
            lambda x: x.update(inference_tx_budget_ns=33_333_334),
            lambda x: x.update(control_clock=control_clock_payload(50)),
            lambda x: x.update(post_reply_deadline_policy={'max_lateness_ms': 1}),
            lambda x: x.update(fixed_catch={}),
            lambda x: x['human_supported_hold'].update(operator_count=True),
            lambda x: x['human_supported_hold'].update(slight_ease_max_duration_s=1),
            lambda x: x['human_supported_hold'].update(hands_remain_on_body=False),
            lambda x: x['axes']['5'].update(kp=12), lambda x: x['axes']['5'].update(kd=.3),
            lambda x: x['axes']['5'].update(max_measured_torque_nm=2),
            lambda x: x['axes']['5'].update(max_temperature_c=60),
            lambda x: x['axes']['5'].update(max_tracking_error_rad=math.radians(3)),
            lambda x: x['axes']['5'].update(max_measured_velocity_rad_s=.5),
            lambda x: x['axes']['5'].update(max_displacement_from_start_rad=math.radians(2)),
            lambda x: x['axes']['5'].update(max_command_velocity_rad_s=float('nan'))]
        for mutate in cases:
            with self.subTest(mutation=cases.index(mutate)):
                data = runtime_profile(); mutate(data)
                with self.assertRaises(ValueError): p.human_supported_hold_settings(data)

    def test_missing_axis_and_gain_bool_do_not_pass_numeric_equalities(self):
        for mutate in (lambda d: d['axes'].pop('12'), lambda d: d['axes']['1'].update(kp=True),
                       lambda d: d.update(duration_s=True)):
            d = runtime_profile(); mutate(d)
            with self.assertRaises(ValueError): p.human_supported_hold_settings(d)

    def test_constructor_dict_boxed_and_test_flags_never_produce_runtime_proof(self):
        for fake in ({'output_allowed': True, 'admitted': True}, runtime_profile(), object()):
            with self.assertRaises(ValueError): p.runtime_spec(fake)
            with self.assertRaises(ValueError): p.supervisor_view(fake, active=True)
        with self.assertRaises(ValueError):
            p.Prepared(object(), {}, p.Pins(), {}, {})

    def test22ms_reply_with17ms_write_passes_separate_clock_check_only(self):
        raw = dict(written=17, received=17, start_ns=1_015_000_000, finish_ns=1_017_000_000,
            read_start_ns=1_017_000_001, received_ns=1_022_000_000, deadline_ns=1_033_333_334)
        self.assertEqual(p._transaction(raw, deadline_ns=1_033_333_334,
                                      write_deadline_ns=1_020_000_000)[1], 1_017_000_000)
        for change in ({'finish_ns': 1_020_000_000}, {'read_start_ns': 1_016_999_999},
                       {'read_start_ns': 0}, {'received_ns': 1_033_333_334},
                       {'written': True}, {'received': 0}, {'deadline_ns': 1_020_000_000}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                p._transaction({**raw, **change}, deadline_ns=1_033_333_334,
                               write_deadline_ns=1_020_000_000)

    def test_canonical_frame_rejects_partial_duplicated_malformed_version_shaped_rx(self):
        can_id = (2 << 24)|(2 << 22)|(1 << 8)|0xfd
        raw = b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+struct.pack('>4H',32767,32767,32767,250)+b'\r\n'
        self.assertEqual(p._feedback(raw.hex(), 1, mode=2, profile=runtime_profile()).mode_state, 2)
        for value in (raw[:-1].hex(), (raw+raw).hex(), raw.hex().upper(), '00'*17):
            with self.assertRaises(ValueError): p._one_frame(value, 'synthetic')
        with self.assertRaises(ValueError): p._feedback(raw.hex(), 2, mode=2, profile=runtime_profile())


class FileAndAudioTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()

    def ref(self, name, raw):
        path = self.root/name
        path.write_bytes(raw)
        return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}

    def audio(self, **wave_args):
        clips = {}
        for stage in p.STAGES:
            raw = wav_bytes(**wave_args)
            clips[stage] = {**self.ref(stage+'.wav', raw),
                           'duration_s': wave_args.get('frames',960)/wave_args.get('rate',48000),
                           'transcript': 'SYNTHETIC only: '+stage}
        return dict(schema=p.AUDIO_SCHEMA, scope=p.SCOPE, contract_sha256='a'*64, clips=clips,
            prepare_ease_is_not_go=True, go_is_short_tone=True,
            resupport_starts_with_urgent_tone=True, operator_must_resupport_before_voice_finishes=True,
            audio_process_completion_is_not_proof_of_audibility=True,
            physical_ease_duration_not_verified_by_audio=True)

    def test_actual_wav_bytes_and_duration_are_authenticated_without_playback(self):
        pins = p.Pins()
        checked = p._audio(self.audio(), runtime_profile(), pins)
        self.assertEqual(set(checked['clips']), set(p.STAGES))
        self.assertEqual(checked['clips']['go']['duration_s'], .02)
        self.assertTrue(checked['audio_completion_does_not_prove_audibility'])
        self.assertEqual(len(pins.values), 5)
        pins.verify()

    def test_pcm_format_duration_sha_missing_clip_and_oversize_go_rejected(self):
        for args in ({'rate':16000}, {'channels':1}, {'width':1}, {'frames':5761}):
            with self.subTest(wave=args), self.assertRaises(ValueError):
                p._audio(self.audio(**args), runtime_profile(), p.Pins())
        for mutate in (lambda d: d['clips'].pop('abort'),
            lambda d: d['clips']['brief'].update(duration_s=.021),
            lambda d: d['clips']['go'].update(sha256='b'*64),
            lambda d: d.update(prepare_ease_is_not_go=False),
            lambda d: d.update(scope='human_supported_partial_current_hold_only')):
            data = self.audio(); mutate(data)
            with self.assertRaises(ValueError): p._audio(data, runtime_profile(), p.Pins())

    def test_pin_replacement_conflict_duplicate_json_symlink_all_rejected(self):
        ref = self.ref('pin.json', b'{"x": 1}')
        pins = p.Pins(); self.assertEqual(pins.json(ref), {'x':1})
        Path(ref['path']).write_bytes(b'{"x": 2}')
        with self.assertRaises(ValueError): pins.verify()
        duplicate = self.ref('duplicate.json', b'{"x": 1, "x": 2}')
        with self.assertRaises(ValueError): p.Pins().json(duplicate)
        linked = self.root/'linked.json'; linked.symlink_to(Path(duplicate['path']))
        with self.assertRaises(ValueError): p.Pins().raw({**duplicate, 'path': str(linked)})


@unittest.skipUnless(thirty.BUNDLE.exists(), 'File-only retrained30Hz model inventory is unavailable')
class ContractPlanTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.world = fixtures.World(Path(self.tmp.name).resolve())
        self.base = boxed.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry,
            control_clock=control_clock_payload(30), policy30_model=thirty.model_selection())
        self.human = p.build_contract(self.base)

    def test_new_scope_settings_are_independent_and_boxed_contract_remains_byte_identical(self):
        old_sha = boxed.contract_sha256(self.base)
        p.validate_contract(self.human)
        self.assertEqual(old_sha, boxed.contract_sha256(self.base))
        self.assertNotEqual(p.canonical_sha256(self.human), old_sha)
        self.assertEqual(self.human['runtime_profile']['axes']['5']['kp'], 6)
        self.assertEqual(self.base['axes']['5']['kp'], 3)
        self.assertFalse(self.human['boxed_permissions_reused'])
        self.assertTrue(all(self.human[k] is v for k,v in p.NO_GRANTS.items()))

    def test_legacy50hz_gain_relaxation_contract_or_permission_transplants_rejected(self):
        with self.assertRaisesRegex(ValueError, '30 Hz'): p.build_contract(self.world.contract())
        for mutate in (lambda d: d['runtime_profile']['axes']['5'].update(kp=12),
                       lambda d: d.update(output_allowed=True),
                       lambda d: d.update(boxed_permissions_reused=True)):
            data = copy.deepcopy(self.human); mutate(data)
            with self.assertRaises(ValueError): p.validate_contract(data)

    def test_default_review_plan_has_blockers_and_no_execution_side_effect_or_grant(self):
        plan = p.build_review_plan(self.base, condition(self.human))
        self.assertEqual(plan['status'], 'PLAN_ONLY')
        self.assertEqual(plan['duration_s'], 8)
        self.assertTrue(plan['blockers'])
        self.assertIn('MISSING_HUMAN_ZERO_GAIN_REPORT', plan['blockers'])
        self.assertTrue(all(plan[k] is v for k,v in p.NO_GRANTS.items()))
        with self.assertRaisesRegex(ValueError, 'Missing named primary'):
            p.prepare_from_files(plan)

    def test_zero_gain2_and_current_hold8_are_distinct_modes_share_only_numeric_contract(self):
        zero = p.build_review_plan(self.base, condition(self.human), mode='human_zero_gain_timing')
        positive = p.build_review_plan(self.base, condition(self.human), mode='human_current_hold')
        self.assertEqual(zero['contract_sha256'], positive['contract_sha256'])
        self.assertEqual(zero['duration_s'], 2)
        self.assertNotIn('MISSING_HUMAN_ZERO_GAIN_REPORT', zero['blockers'])
        for mode in ('zero_gain_timing', 'learned_boxed', 'human_current_hold8'):
            with self.assertRaises(ValueError): p.build_review_plan(self.base, mode=mode)

    def test_current_roles_pose_clearance_power_boot_and_capture_do_not_follow_old_boxed_fact(self):
        good = condition(self.human)
        self.assertIs(p.validate_conditions(good, self.human, capture_sha256='a'*64), good)
        cases = ({'operator_count':1}, {'operator_count':True}, {'boot_id':'other'},
            {'motor_power_epoch':'other'}, {'contract_sha256':boxed.contract_sha256(self.base)},
            {'source':'old-boxed-record'}, {'synthetic_interaction':True}, {'hands_remain_on_body':False},
            {'roles':{'body_catch':'same','terminal_and_cutoff':'same'}},
            {'local_clearance_rad':math.radians(10)}, {'walking_allowed':True},
            {'audio_heard':False}, {'capture_sha256':'b'*64})
        for changed in cases:
            with self.subTest(change=changed), self.assertRaises(ValueError):
                p.validate_conditions({**good, **changed}, self.human, capture_sha256='a'*64)

    def test_boxed_stop_proxy_or_low_information_reports_cannot_be_human_zero_type1(self):
        proxy = thirty.proxy30(self.world.lineage, self.base['topology_by_port'], 60_000_000_000,
            self.base['control_clock'], self.base['policy30_model'], split=True)
        for report in (proxy, {'schema':p.REPORT_SCHEMA, 'status':'COMPLETE_HUMAN_ZERO_GAIN_TIMING'},
                       fixtures.type1_report(self.base, 'zero_gain_timing', 2, 60_000_000_000,61_000_000_000)):
            with self.assertRaises(ValueError):
                p.validate_zero_gain_report(report, self.human, after_ns=50_000_000_000,
                                            source_manifest_sha256='9'*64)

    def test_boxed_or_old_two_bus_watchdog_does_not_qualify_new_human_watchdog(self):
        incomplete = dict(status='COMPLETE_COMMAND_LOSS_DIAGNOSTIC', errors=[], selected_ids=list(p.IDS),
            configured_timeout_ms=200, watchdog_ticks=4000, motor_enable_sent=True, stop_confirmed=True,
            positive_gain_sent=False, learned_targets_sent=False, usb_disconnect_tested=False,
            approved_for_runtime=False, boot_id=self.base['boot_id'], motor_power_epoch=self.base['motor_power_epoch'],
            source_manifest_sha256='9'*64, axes={str(i):{} for i in p.IDS})
        with self.assertRaisesRegex(ValueError, 'original four-owner'):
            p.validate_watchdog(incomplete, {'events_by_port':{'front':[], 'rear':[]}}, self.human,
                                after_ns=1, source_manifest_sha256='9'*64)

    def test_complete_synthetic_raw_shape_is_parseable_but_never_runtime_permission(self):
        data=raw_zero_fixture(self.human)
        summary=p.validate_zero_gain_report(data,self.human,after_ns=5_000_000_000,source_manifest_sha256='9'*64)
        self.assertEqual(summary['completed_cycles'],53)
        self.assertNotIn('output_allowed',summary)
        with self.assertRaises(ValueError):p.runtime_spec(summary)

    def test_preflight_stop_cannot_substitute_for_terminal_subset_stop(self):
        original=raw_zero_fixture(self.human)
        self.assertEqual(original['measurement']['raw_journal']['port3'][0][0], 'stop')
        p.validate_zero_gain_report(original,self.human,after_ns=5_000_000_000,source_manifest_sha256='9'*64)
        for mutate in (
            lambda journal: journal.pop(),
            lambda journal: journal[-1].__setitem__(0,'stop'),
            lambda journal: journal[-1].__setitem__(0,'stop_subset_arbitrary'),
            lambda journal: journal[-1][1]['records'][0].update(start_ns=5_900_000_000,
                finish_ns=5_900_000_010,read_start_ns=5_900_000_011,received_ns=5_900_100_000)):
            with self.subTest(case=repr(mutate)):
                data=copy.deepcopy(original)
                mutate(data['measurement']['raw_journal']['port3'])
                with self.assertRaises(ValueError):
                    p.validate_zero_gain_report(data,self.human,after_ns=5_000_000_000,source_manifest_sha256='9'*64)

    def test_one_late_write_uncausal_read_wrong_raw_gain_missing_journal_and_stop_rejected(self):
        base=raw_zero_fixture(self.human)
        row=lambda d:d['measurement']['cycles'][10]
        packet=lambda d:row(d)['output']['port3']['records'][2]
        mutations=[lambda d:packet(d).update(finish_ns=row(d)['write_deadline_ns']),
            lambda d:packet(d).update(read_start_ns=0),
            lambda d:packet(d).update(received_ns=row(d)['hard_end_ns']),
            lambda d:packet(d).update(tx_hex=encode_motion(12,0,1,.15).hex()),
            lambda d:row(d).update(completed=False),lambda d:row(d).update(slot=11),
            lambda d:row(d)['imu'].update(gyro_rad_s=[float('nan'),0,0]),
            lambda d:row(d)['imu'].update(read_started_monotonic_ns=1),
            lambda d:d['measurement']['raw_journal']['port3'].pop(10*3+2),
            lambda d:d['measurement']['stop_results']['port3'].update(ambiguous_ids=[12]),
            lambda d:d['measurement']['worker_settings']['imu'].update(file_only_mock_readback=True),
            lambda d:d['measurement']['backend_usage'].update(kind='injected_mock'),
            lambda d:d['measurement']['restoration'].update(main=False),
            lambda d:d['measurement']['raw_journal']['port3'].pop(),
            lambda d:d.update(source_manifest_sha256='a'*64)]
        for mutate in mutations:
            with self.subTest(case=mutations.index(mutate)):
                data=copy.deepcopy(base);mutate(data)
                with self.assertRaises(ValueError):
                    p.validate_zero_gain_report(data,self.human,after_ns=5_000_000_000,source_manifest_sha256='9'*64)


if __name__ == '__main__':
    unittest.main()
