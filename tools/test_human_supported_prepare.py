"""Synthetic file/TTY/child contracts only; no robot or audio device is opened."""
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import wave

spec=importlib.util.spec_from_file_location('human_supported_prepare',Path(__file__).with_name('human_supported_prepare.py'))
prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)
BOOT='12345678-1234-4234-9234-123456789abc'


def save(path,value):
    raw=json.dumps(value).encode();path.write_bytes(raw)
    return {'path':str(path),'sha256':hashlib.sha256(raw).hexdigest()}


def fixture(base):
    pins=[]
    for name in ('motor_epoch_readonly_capture.py','jetson_cpu_performance_scope.py'):
        path=base/name;path.write_text('SYNTHETIC ONLY')
        pins.append({'path':str(path),'sha256':prep.digest(path.read_bytes())})
    rehearsal=save(base/'rehearsal.json',dict(schema='singularitydog.human-supported-operator-receipt.v1',
        kind='rehearsal',observed_by='operator',motor_power_off=True,operator_count=2,
        body_full_support_continuous=True,box_removed_and_restored=True,cutoff_role_maintained=True,
        abnormal_noise_vibration_slip_sinking_contact=False))
    audio={}
    for stage in prep.STAGES:
        path=base/(stage+'.wav')
        with wave.open(str(path),'wb') as wav:
            wav.setnchannels(2);wav.setsampwidth(2);wav.setframerate(48000);wav.writeframes(bytes(1920))
        audio[stage]=dict(path=str(path),sha256=prep.digest(path.read_bytes()),duration_s=.01,
                          transcript='SYNTHETIC '+stage)
    stages={stage:dict(argv=['/fake/python',('--execute-readonly' if stage=='capture' else
        '--execute-human-supported-zero-gain' if stage=='watchdog' else '--supported-disabled')],
        result_path=str(base/(stage+'.json')),timeout_s=15) for stage in prep.STAGES}
    stages['watchdog']['argv'] += ['--audio',audio['watchdog']['path'],'--audio-sha256',
        audio['watchdog']['sha256'],'--audio-device','fake','--power-epoch','SYNTHETIC-new']
    stages['pipeline']['argv'] += ['--diagnostic','--execute',prep.MODE,'SYNTHETIC-new']
    stages['pipeline']['records_path']=str(base/'records.json')
    return dict(schema=prep.SCHEMA,boot_id=BOOT,motor_power_epoch='SYNTHETIC-new',pins=pins,
                rehearsal_receipt=rehearsal,audio=audio,audio_device='fake',stages=stages)


def results(plan):
    capture=dict(status='RECORDED_REVIEW_REQUIRED',errors=[],boot_id=BOOT,approved_for_runtime=False,
        motor_output_allowed=False,stop_state='UNVERIFIED_BY_READ_ONLY_PROTOCOL',
        motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT',identities={},telemetry={'rows':{}})
    for mid in map(str,range(1,13)):
        capture['identities'][mid]={'request_monotonic_ns':100,'reply_monotonic_ns':200}
        capture['telemetry']['rows'][mid]=dict(run_mode=0,current=0,position_samples=[
            dict(rad=0.,request_monotonic_ns=300+i*200,reply_monotonic_ns=400+i*200) for i in range(3)])
    watchdog=dict(status='COMPLETE_COMMAND_LOSS_DIAGNOSTIC',errors=[],boot_id=BOOT,
        motor_power_epoch=plan['motor_power_epoch'],positive_gain_sent=False,learned_targets_sent=False,
        stop_confirmed=True,axes={},stop_reports={},announcement={'process_completed':True,
        'sha256':plan['audio']['watchdog']['sha256']})
    for mid in map(str,range(1,13)):
        watchdog['axes'][mid]=dict(version={'request_start_ns':900},stop_probe={'received_ns':1000,
            'mode_state':0,'fault_bits':0},command_loss_tested=True,disabled_on_command_loss=True)
    for bus,ids in prep.BUSES.items():
        watchdog['stop_reports'][bus]=dict(complete=True,confirmed_ids=list(ids),unconfirmed_ids=[],
                                          ambiguous_ids=[],errors=[])
    pipeline=dict(status='COMPLETE_DIAGNOSTIC',mode='stop-proxy',errors=[],boot_id=BOOT,motor_power_epoch=plan['motor_power_epoch'],
        motor_enable_sent=False,learned_targets_sent=False,cycles_completed=501,cycles_requested=501,
        imu_restore_status='restored',source_provenance=dict(mode=prep.MODE,motor_power_epoch=plan['motor_power_epoch'],
        source_files_unchanged=True),observer=dict(status='COMPLETE_NO_OUTPUT_DIAGNOSTIC',ticks_completed=501,
        ticks_requested=501,failure=None,incomplete=False,output_allowed=False),measurements=[])
    records=[]
    for index in range(501):
        begin=1_000_000+index*20_000_000
        pipeline['measurements'].append(dict(release_ns=begin,oldest_input_start_ns=begin+1,
            last_proxy_reply_ns=begin+18_000_000,cycle_end_ns=begin+19_000_000,
            cadence_slot=index,skipped_slots_before=0))
        row={'cycle':index+1,'output':{}}
        for bus,ids in prep.BUSES.items():
            output=[]
            for mid in ids:
                rx=b'AT'+((((2<<24)|(mid<<8)|0xfd)<<3)|4).to_bytes(4,'big')+bytes([8])+bytes(8)+b'\r\n'
                tx=b'AT'+((((4<<24)|(0xfd<<8)|mid)<<3)|4).to_bytes(4,'big')+bytes([8])+bytes(8)+b'\r\n'
                output.append(dict(rx_hex=rx.hex(),tx_hex=tx.hex(),received=17,written=17,
                    start_ns=begin+13_000_000,finish_ns=begin+14_000_000,received_ns=begin+18_000_000))
            row['output'][bus]={'records':output}
        records.append(row)
    pipeline['v3_voltage_fast_pipeline']={'records_sha256':prep.digest(json.dumps(records).encode())}
    return dict(capture=capture,watchdog=watchdog,pipeline=pipeline),records


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name);self.plan=fixture(self.base)
        self.data,self.records=results(self.plan);self.called=[];self.played=[]

    def fake_child(self,argv,timeout,cancelled):
        stage=next(s for s in prep.STAGES if self.plan['stages'][s]['argv']==argv)
        self.called.append(stage)
        if stage=='pipeline':save(Path(self.plan['stages'][stage]['records_path']),self.records)
        save(Path(self.plan['stages'][stage]['result_path']),self.data[stage])

    def execute(self,answers=None,play=None,visible=True,boot=BOOT):
        iterator=iter(answers or ['y']*5)
        with patch('sys.stdout',new=io.StringIO()):
            return prep.run(self.plan,self.base/'out',confirm=lambda _:next(iterator),
                boot=lambda:boot,child=self.fake_child,play=play or (lambda *a,**k:self.played.append(a)),
                visible=lambda:visible)

    def test_success_records_honest_unapproved_receipts_and_never_duplicates_watchdog_audio(self):
        prep.check_plan(self.plan);result=self.execute()
        self.assertEqual(result['status'],'DIAGNOSTICS_RECORDED_REVIEW_REQUIRED')
        self.assertEqual(self.called,list(prep.STAGES));self.assertEqual(len(self.played),2)
        self.assertFalse(result['output_allowed']);self.assertFalse(result['partial_load_started'])
        self.assertEqual(set(result['source_receipts']),{'power','pose','video','clearance','physical_observation','rehearsal'})
        for kind,pin in result['source_receipts'].items():
            if kind=='rehearsal':continue
            receipt=json.loads(Path(pin['path']).read_text());self.assertIsNone(receipt['review'])
            self.assertIn('Operator response: y',receipt['user_statement'])
            self.assertTrue(receipt['source_message_id'].startswith('visible-tty:'))
        pose=json.loads(Path(result['source_receipts']['pose']['path']).read_text())
        self.assertEqual(pose['operator_count'],2)
        self.assertIn('1人が胴体の全重量を支え続け',pose['user_statement'])
        self.assertIn('もう1人が端末操作と即時40V Offを担当',pose['user_statement'])
        self.assertIn('担当者は合計2人',pose['user_statement'])

    def test_false_or_nonliteral_readiness_never_starts_audio_or_child(self):
        for answer in ('n','', ' ', 'yes please','y/n'):
            with self.subTest(answer=answer):
                out=self.base/('abort-'+str(len(list(self.base.iterdir()))))
                with patch('sys.stdout',new=io.StringIO()):
                    report=prep.run(self.plan,out,confirm=lambda _:answer,boot=lambda:BOOT,
                        child=self.fake_child,play=lambda *a,**k:self.played.append(a),visible=lambda:True)
                self.assertTrue(report['errors']);self.assertEqual(self.called,[]);self.assertEqual(self.played,[])

    def test_explicit_yes_variants_preserve_raw_answer_and_prompt(self):
        for index,answer in enumerate(('y','Y','yes',' YES ','はい',' はい ')):
            with self.subTest(answer=answer),patch('sys.stdout',new=io.StringIO()):
                iterator=iter([answer]+['y']*4)
                report=prep.run(self.plan,self.base/('yes-'+str(index)),confirm=lambda _:next(iterator),
                    boot=lambda:BOOT,child=self.fake_child,play=lambda *a,**k:None,visible=lambda:True)
                self.assertEqual(report['status'],'DIAGNOSTICS_RECORDED_REVIEW_REQUIRED')
                receipt=json.loads(Path(report['source_receipts']['power']['path']).read_text())
                self.assertIn('Operator response: '+answer,receipt['user_statement'])
                self.assertIsNone(receipt['review'])
                records=json.loads(Path(report['operator_confirmations']['path']).read_text())
                self.assertEqual(records[0]['operator_response'],answer)
                self.assertIn('y を入力してEnter（Enterだけでは中止）',records[0]['prompt'])

    def test_empty_answer_saved_as_original_and_does_not_dispatch(self):
        report=self.execute(answers=[''])
        self.assertEqual(report['status'],'ABORTED_PREPARATION')
        self.assertEqual(self.called,[]);self.assertEqual(self.played,[])
        records=json.loads(Path(report['operator_confirmations']['path']).read_text())
        self.assertEqual(records[0]['operator_response'],'')
        self.assertNotIn('source_receipts',report)

    def main_args(self,output):
        pin=save(self.base/'plan.json',self.plan)
        return ['--plan',pin['path'],'--expected-plan-sha256',pin['sha256'],'--output',str(output)]

    def test_main_existing_output_returns_json_no_traceback_or_dispatch(self):
        output=self.base/'used';output.mkdir();(output/'sentinel').write_text('unchanged')
        stream=io.StringIO()
        with patch('sys.stdout',new=stream),patch.object(prep,'visible_terminal',return_value=True),\
                patch.object(prep,'run') as run:
            rc=prep.main(self.main_args(output)+['--execute-human-full-support-diagnostics'])
        self.assertEqual(rc,2);run.assert_not_called()
        text=stream.getvalue();self.assertNotIn('Traceback',text);self.assertIn(prep.RECOVERY,text)
        report=json.loads(text.splitlines()[-1])
        self.assertEqual(report['status'],'ABORTED_PREPARATION')
        self.assertIn('Fresh private output required',report['errors'][0])
        for key in ('output_allowed','approvals_created','partial_load_started','hardware_opened'):
            self.assertIs(report[key],False)
        self.assertEqual((output/'sentinel').read_text(),'unchanged')

    def test_main_invalid_tty_returns_json_before_output_or_dispatch(self):
        output=self.base/'no-tty';stream=io.StringIO()
        with patch('sys.stdout',new=stream),patch.object(prep,'visible_terminal',return_value=False),\
                patch.object(prep,'run') as run:
            rc=prep.main(self.main_args(output)+['--execute-human-full-support-diagnostics'])
        self.assertEqual(rc,2);run.assert_not_called();self.assertFalse(output.exists())
        report=json.loads(stream.getvalue().splitlines()[-1])
        self.assertEqual(report['status'],'ABORTED_PREPARATION')
        self.assertIs(report['hardware_opened'],False)
        self.assertIn('visible local terminal',report['errors'][0])

    def test_plan_only_does_not_require_tty_or_create_output(self):
        output=self.base/'plan-only';stream=io.StringIO()
        with patch('sys.stdout',new=stream),patch.object(prep,'visible_terminal',return_value=False),\
                patch.object(prep,'run') as run:
            rc=prep.main(self.main_args(output))
        self.assertEqual(rc,0);run.assert_not_called();self.assertFalse(output.exists())
        self.assertEqual(json.loads(stream.getvalue())['status'],'PLAN_ONLY')

    def test_announcement_failure_prevents_hardware_and_next_stage(self):
        def fail(*a,**k):raise RuntimeError('audio failure')
        result=self.execute(play=fail);self.assertTrue(result['errors']);self.assertEqual(self.called,[])

    def test_wrong_or_nonvisible_tty_refused_before_output(self):
        with self.assertRaisesRegex(ValueError,'visible'):self.execute(visible=False)
        self.assertFalse((self.base/'out').exists())

    def test_changed_boot_prevents_first_child(self):
        result=self.execute(boot='changed');self.assertTrue(result['errors']);self.assertEqual(self.called,[])

    def test_wrong_capture_status_exit_zero_not_success(self):
        self.data['capture']['status']='COMPLETE_DIAGNOSTIC'
        result=self.execute();self.assertTrue(result['errors']);self.assertEqual(self.called,['capture'])

    def test_watchdog_sticky_ambiguity_stops_pipeline_even_with_stop_summary_true(self):
        self.data['watchdog']['stop_reports']['front']['ambiguous_ids']=[3]
        result=self.execute();self.assertTrue(result['errors']);self.assertEqual(self.called,['capture','watchdog'])

    def test_watchdog_unheard_or_failed_owned_announcement_stops_pipeline(self):
        self.data['watchdog']['announcement']['process_completed']=False
        result=self.execute();self.assertTrue(result['errors']);self.assertNotIn('pipeline',self.called)

    def test_positive_gain_and_old_epoch_watchdog_rejected(self):
        for key,value in (('positive_gain_sent',True),('motor_power_epoch','old')):
            with self.subTest(key=key),self.assertRaises(ValueError):
                changed=copy.deepcopy(self.data['watchdog']);changed[key]=value
                prep.verify_result('watchdog',changed,self.plan,800)

    def test_pipeline_missing_inference_and_twenty_ms_overrun_rejected(self):
        for key,value in (('cycles_completed',500),('motor_enable_sent',True)):
            with self.subTest(key=key),self.assertRaises(ValueError):
                changed=copy.deepcopy(self.data['pipeline']);changed[key]=value
                prep.verify_result('pipeline',changed,self.plan,1000)
        changed=copy.deepcopy(self.data['pipeline']);changed['measurements'][0]['cycle_end_ns']+=1_000_001
        with self.assertRaisesRegex(ValueError,'20ms'):prep.verify_result('pipeline',changed,self.plan,1000)

    def test_raw_stop_crossbus_fault_partial_and_hash_mutation_rejected(self):
        for mutation in ('crossbus','partial','fault','sha'):
            records=copy.deepcopy(self.records)
            if mutation=='crossbus':records[0]['output']['front']['records'][0]=records[0]['output']['rear']['records'][0]
            elif mutation=='partial':records[0]['output']['front']['records'].pop()
            elif mutation=='fault':
                item=records[0]['output']['front']['records'][0];raw=bytearray.fromhex(item['rx_hex']);raw[3]|=8;item['rx_hex']=raw.hex()
            changed=copy.deepcopy(self.data['pipeline']);pin=save(self.base/'records.json',records)
            changed['v3_voltage_fast_pipeline']['records_sha256']=pin['sha256'] if mutation!='sha' else '0'*64
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                prep.verify_result('pipeline',changed,self.plan,1000)

    def test_pin_mutation_and_actuation_argv_refused(self):
        Path(self.plan['pins'][0]['path']).write_text('changed')
        with self.assertRaisesRegex(ValueError,'SHA'):prep.check_plan(self.plan)
        self.plan=fixture(self.base)
        self.plan['stages']['pipeline']['argv'].append('--execute-human-supported-partial')
        with self.assertRaisesRegex(ValueError,'forbidden'):prep.check_plan(self.plan)

    def test_real_duration_and_wrong_watchdog_audio_argv_refused(self):
        self.plan['audio']['capture']['duration_s']=2
        with self.assertRaisesRegex(ValueError,'duration'):prep.check_plan(self.plan)
        self.plan['audio']['capture']['duration_s']=.01
        self.plan['stages']['watchdog']['argv'][-1]='different-epoch'
        with self.assertRaisesRegex(ValueError,'differs'):prep.check_plan(self.plan)

    def test_optional_saved501_layout_only_never_relabels_actual_evidence(self):
        supplied=os.environ.get('SINGULARITYDOG_SAVED_PIPELINE_REPORT')
        if not supplied:self.skipTest('Optional saved report was not explicitly supplied')
        path=Path(supplied)
        self.assertTrue(path.is_absolute() and path.is_file() and not path.is_symlink())
        self.assertLessEqual(path.stat().st_size,16*1024*1024)
        # Only synthetic plan labels change. The raw report and records are
        # never written, and this is not a present-power approval or review.
        raw=path.read_bytes();historical=json.loads(raw)
        adapted=copy.deepcopy(historical)
        adapted['motor_power_epoch']=self.plan['motor_power_epoch']
        adapted['source_provenance'].update(mode=prep.MODE,motor_power_epoch=self.plan['motor_power_epoch'])
        self.plan['boot_id']=historical['boot_id']
        self.plan['stages']['pipeline']['records_path']=str(path.parent/'records.json')
        self.assertGreater(prep.verify_result('pipeline',adapted,self.plan,1),0)
        self.assertEqual(path.read_bytes(),raw)




# Historical fixture contracts are reused unchanged for the branch-refresh tests.
import types
legacy=types.SimpleNamespace(fixture=fixture,results=results,save=save,BOOT=BOOT)
branch_spec=importlib.util.spec_from_file_location('branch_fixtures',
    Path(__file__).resolve().parents[1]/'runtime/tests/test_diagnostic_angle_branch.py')
branch_tests=importlib.util.module_from_spec(branch_spec);branch_spec.loader.exec_module(branch_tests)
HELPER=Path(__file__).resolve().parents[1]/'runtime/singularitydog_hw/diagnostic_angle_branch.py'

class RefreshTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.plan = legacy.fixture(self.base)
        self.data, self.records = legacy.results(self.plan)
        self.calls, self.played = [], []
        base, capture, uids = branch_tests.fixtures({1: 1, 3: 1, 11: 1})
        self.uid_pin = legacy.save(self.base/'uids.json', uids)
        self.base_pin = legacy.save(self.base/'base-calibration.json', base)
        helper = self.base/'diagnostic_angle_branch.py'
        helper.write_bytes(HELPER.read_bytes())
        self.helper_pin = {'path': str(helper), 'sha256': prep.digest(helper.read_bytes())}
        self.plan['pins'].extend([self.helper_pin, self.base_pin, self.uid_pin])
        self.plan['diagnostic_calibration_refresh'] = dict(
            schema='singularitydog.capture-bound-diagnostic-branch-refresh-plan.v1',
            helper=self.helper_pin, base_calibration=self.base_pin, expected_uids=self.uid_pin)
        self.plan['stages']['pipeline']['argv'].extend([
            '--calibration', self.base_pin['path'], '--expected-uids', self.uid_pin['path']])
        capture['boot_id'] = legacy.BOOT
        capture['expected_uids_sha256'] = self.uid_pin['sha256']
        self.data['capture'] = capture
        for axis in self.data['watchdog']['axes'].values():
            axis['version']['request_start_ns'] = 20_000
            axis['stop_probe']['received_ns'] = 30_000
        self.before = copy.deepcopy(self.plan)
        self.original_base = Path(self.base_pin['path']).read_bytes()

    def child(self, argv, timeout, cancelled):
        if argv == self.plan['stages']['capture']['argv']:
            stage = 'capture'
        elif argv == self.plan['stages']['watchdog']['argv']:
            stage = 'watchdog'
        else:
            stage = 'pipeline'
        self.calls.append((stage, copy.deepcopy(argv)))
        if stage == 'pipeline':
            calibration_path = argv[argv.index('--calibration')+1]
            self.data[stage]['input_sha256'] = {'calibration': prep.digest(Path(calibration_path).read_bytes())}
            legacy.save(Path(self.plan['stages'][stage]['records_path']), self.records)
        legacy.save(Path(self.plan['stages'][stage]['result_path']), self.data[stage])

    def run_case(self, *, child=None):
        answers = iter(['y']*5)
        with patch('sys.stdout', new=io.StringIO()):
            return prep.run(self.plan, self.base/'out', confirm=lambda _:next(answers),
                boot=lambda:legacy.BOOT, child=child or self.child,
                play=lambda *a, **k:self.played.append(a), visible=lambda:True)

    def test_success_derives_before_watchdog_and_uses_exact_new_pipeline_argument(self):
        prep.check_plan(self.plan)
        result = self.run_case()
        self.assertEqual(result['status'], 'DIAGNOSTICS_RECORDED_REVIEW_REQUIRED', result['errors'])
        self.assertEqual([x[0] for x in self.calls], list(prep.STAGES))
        expected = str(self.base/'out/capture-bound-diagnostic-calibration.json')
        actual = self.calls[-1][1]
        self.assertEqual(actual[actual.index('--calibration')+1], expected)
        original = copy.deepcopy(self.plan['stages']['pipeline']['argv'])
        original[original.index('--calibration')+1] = expected
        self.assertEqual(actual, original)
        self.assertEqual(result['pipeline_execution_argv'], actual)
        self.assertEqual(json.loads((self.base/'out/pipeline-effective-argv.json').read_text()), actual)
        self.assertEqual(self.plan, self.before)
        self.assertEqual(Path(self.base_pin['path']).read_bytes(), self.original_base)
        refresh = result['diagnostic_calibration_refresh']
        self.assertEqual({mid:turn for mid,turn in refresh['turns_by_id'].items() if turn},
                         {'1':1, '3':1, '11':1})
        derived = json.loads(Path(refresh['path']).read_bytes())
        for key in ('output_allowed','motor_output_allowed','approved_for_runtime',
                    'motor_output_available','motor_targets_generated','calibration_verified',
                    'live_50hz_verified','epoch_binding_created','physical_joint_limits_verified'):
            self.assertIs(derived[key], False)
        capture_pin = result['stages'][0]['result']
        self.assertEqual(derived['diagnostic_branch_derivation']['fresh_capture'], capture_pin)
        self.assertEqual(derived['source_capture_power_epoch_label_preserved'], 'NOT_INFERRED_FROM_JETSON_BOOT')
        self.assertEqual(derived['motor_power_epoch'], self.plan['motor_power_epoch'])
        self.assertIs(result['output_allowed'], False)
        self.assertIs(result['approvals_created'], False)
        for name,pin in result['source_receipts'].items():
            if name != 'rehearsal':self.assertIsNone(json.loads(Path(pin['path']).read_bytes())['review'])

    def test_invalid_refresh_config_fails_main_before_output_audio_or_children(self):
        cases = ('missing-pin','wrong-hash','unrecognized-key','wrong-schema','wrong-name',
                 'wrong-baseline-argument','duplicate-calibration','wrong-uid-argument')
        for case in cases:
            with self.subTest(case=case):
                plan = copy.deepcopy(self.plan)
                config = plan['diagnostic_calibration_refresh']
                if case == 'missing-pin':plan['pins'].remove(config['helper'])
                elif case == 'wrong-hash':config['helper']['sha256'] = '0'*64
                elif case == 'unrecognized-key':config['unsafe_permission'] = True
                elif case == 'wrong-schema':config['schema'] += '.unknown'
                elif case == 'wrong-name':
                    renamed = self.base/'arbitrary-helper.py';renamed.write_bytes(HELPER.read_bytes())
                    config['helper'] = {'path':str(renamed),'sha256':prep.digest(renamed.read_bytes())}
                    plan['pins'].append(config['helper'])
                elif case == 'wrong-baseline-argument':
                    argv = plan['stages']['pipeline']['argv'];argv[argv.index('--calibration')+1] = '/unreviewed/cal.json'
                elif case == 'duplicate-calibration':plan['stages']['pipeline']['argv'] += ['--calibration',self.base_pin['path']]
                else:
                    argv = plan['stages']['pipeline']['argv'];argv[argv.index('--expected-uids')+1] = '/unreviewed/uids.json'
                pin = legacy.save(self.base/('invalid-'+case+'.json'), plan)
                out = self.base/('invalid-out-'+case)
                with patch.object(prep,'run') as run, patch.object(prep,'subprocess') as subprocess,\
                        patch('sys.stderr',new=io.StringIO()), self.assertRaises(SystemExit):
                    prep.main(['--plan',pin['path'],'--expected-plan-sha256',pin['sha256'],
                               '--output',str(out),'--execute-human-full-support-diagnostics'])
                run.assert_not_called();subprocess.run.assert_not_called();self.assertFalse(out.exists())

    def test_derivation_failure_or_uid_mismatch_dispatches_capture_only(self):
        for kind in ('helper-error','uid-mismatch'):
            with self.subTest(kind=kind):
                # Separate fresh fixture/output for each attempt.
                self.calls.clear()
                if kind == 'uid-mismatch':self.data['capture']['identities']['1']['mcu_uid_hex'] = 'f'*16
                context = (patch.object(prep,'refresh_diagnostic_calibration',side_effect=ValueError('injected derive failure'))
                           if kind == 'helper-error' else patch.object(prep,'refresh_diagnostic_calibration',wraps=prep.refresh_diagnostic_calibration))
                with context:result = self.run_case()
                self.assertEqual(result['status'], 'ABORTED_PREPARATION')
                self.assertEqual([x[0] for x in self.calls], ['capture'])
                self.assertNotIn('diagnostic_calibration_refresh', result)
                self.assertFalse(result['output_allowed']);self.assertFalse(result['approvals_created'])
                # Only files created by this synthetic test are cleared for the second fixture.
                if kind == 'helper-error':
                    import shutil
                    shutil.rmtree(self.base/'out')
                    Path(self.plan['stages']['capture']['result_path']).unlink()

    def test_pinned_helper_changed_during_capture_aborts_before_derive_watchdog_pipeline(self):
        def changed(argv, timeout, cancelled):
            self.child(argv, timeout, cancelled)
            if argv == self.plan['stages']['capture']['argv']:
                Path(self.helper_pin['path']).write_text('raise RuntimeError("UNAUTHORIZED")')
        result = self.run_case(child=changed)
        self.assertEqual([x[0] for x in self.calls], ['capture'])
        self.assertEqual(result['status'], 'ABORTED_PREPARATION')
        self.assertIn('SHA256 mismatch', result['errors'][0])
        self.assertFalse((self.base/'out/capture-bound-diagnostic-calibration.json').exists())

    def test_mutation_of_derived_file_after_watchdog_prevents_pipeline_dispatch(self):
        def changed(argv, timeout, cancelled):
            self.child(argv, timeout, cancelled)
            if argv == self.plan['stages']['watchdog']['argv']:
                (self.base/'out/capture-bound-diagnostic-calibration.json').write_text('{}')
        result = self.run_case(child=changed)
        self.assertEqual([x[0] for x in self.calls], ['capture','watchdog'])
        self.assertEqual(result['status'], 'ABORTED_PREPARATION')
        self.assertIn('SHA256 mismatch', result['errors'][0])

    def test_pipeline_report_using_wrong_calibration_hash_never_completes_or_grants_receipts(self):
        def wrong(argv, timeout, cancelled):
            self.child(argv, timeout, cancelled)
            if '--calibration' in argv:
                self.data['pipeline']['input_sha256']['calibration'] = '0'*64
                legacy.save(Path(self.plan['stages']['pipeline']['result_path']),self.data['pipeline'])
        result = self.run_case(child=wrong)
        self.assertEqual(result['status'], 'ABORTED_PREPARATION')
        self.assertIn('exact capture-bound calibration', result['errors'][0])
        self.assertNotIn('source_receipts', result)
        self.assertFalse(result['output_allowed']);self.assertFalse(result['approvals_created'])

    def test_derived_approval_or_provenance_mutation_rejected_after_pinned_read(self):
        # The real helper is used; intercept only the read-back document to
        # exercise the coordinator's independent no-approval/provenance gate.
        original_decode = prep.decode
        mutations = [('output_allowed',True),('motor_output_available',True),
                     ('motor_targets_generated',True),('calibration_verified',True),
                     ('live_50hz_verified',True),('source_current_boot_id','wrong-boot'),
                     ('motor_power_epoch','old-power')]
        capture_pin = legacy.save(self.base/'fresh-capture.json',self.data['capture'])
        for index,(key,value) in enumerate(mutations):
            with self.subTest(key=key):
                out = self.base/('derived-'+str(index));out.mkdir()
                def invalid(raw):
                    document = original_decode(raw)
                    if 'diagnostic_branch_derivation' in document:document[key] = value
                    return document
                with patch.object(prep,'decode',side_effect=invalid),self.assertRaisesRegex(ValueError,'provenance'):
                    prep.refresh_diagnostic_calibration(self.plan,capture_pin,out)

    def test_derived_capture_or_static_source_binding_mismatch_rejected(self):
        original_decode = prep.decode
        capture_pin = legacy.save(self.base/'fresh-capture.json',self.data['capture'])
        for index,key in enumerate(('fresh_capture','base_calibration','expected_uids','scope')):
            with self.subTest(key=key):
                out = self.base/('binding-'+str(index));out.mkdir()
                def invalid(raw):
                    document = original_decode(raw)
                    if 'diagnostic_branch_derivation' in document:
                        document['diagnostic_branch_derivation'][key] = 'different'
                    return document
                with patch.object(prep,'decode',side_effect=invalid),self.assertRaisesRegex(ValueError,'provenance'):
                    prep.refresh_diagnostic_calibration(self.plan,capture_pin,out)



if __name__=='__main__':unittest.main()
