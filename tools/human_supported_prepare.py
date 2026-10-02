#!/usr/bin/env python3
"""Visible-TTY diagnostics only; never approves or starts partial-load output.

A root-reviewed pinned plan selects three diagnostic commands. This coordinator
records operator statements without named reviews and preserves raw child JSON.
Default is file-only PLAN_ONLY. Child stdout/stdin remain the visible terminal.
"""
import argparse
import copy
import types
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import wave

SCHEMA = 'singularitydog.human-supported-preparation-plan.v1'
MODE = 'human-supported-partial-current-hold-audio-8s-v1'
STAGES = ('capture', 'watchdog', 'pipeline')
BUSES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
RECOVERY = ('全て胴体を支え続け、40VをOffにしてください。'
            '支えたまま箱を安全に戻してください。力は緩めません。')


def need(value, message):
    if not value: raise ValueError(message)


def pairs(items):
    result = {}
    for key, value in items:
        need(key not in result, 'Duplicate JSON key')
        result[key] = value
    return result


def decode(raw):
    return json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def digest(raw): return hashlib.sha256(raw).hexdigest()


def file_bytes(path, expected=None, max_bytes=64*1024*1024):
    p = Path(path).expanduser()
    need(p.is_absolute() and p.is_file() and not p.is_symlink(), 'Regular absolute file required')
    need(0 < p.stat().st_size <= max_bytes, 'Bounded input file required')
    raw = p.read_bytes()
    need(expected is None or digest(raw) == expected, 'Input SHA256 mismatch: '+p.name)
    return raw


def write(out, name, value):
    path = out/name
    with os.fdopen(os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|
                   getattr(os, 'O_NOFOLLOW', 0), 0o600), 'w') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')
    return {'path': str(path), 'sha256': digest(path.read_bytes())}


def check_plan(plan):
    need(type(plan) is dict and plan.get('schema') == SCHEMA, 'Wrong plan schema')
    need(str(uuid.UUID(plan['boot_id'])) == plan['boot_id'], 'Canonical boot UUID required')
    epoch = plan['motor_power_epoch']
    need(type(epoch) is str and epoch.strip() == epoch and 0 < len(epoch) <= 1024 and
         epoch != 'NOT_INFERRED_FROM_JETSON_BOOT', 'Explicit new planned power epoch required')
    need(type(plan.get('pins')) is list and bool(plan['pins']), 'Executable/input pins required')
    need(type(plan.get('audio_device')) is str and bool(plan['audio_device']), 'Audio device required')
    need(set(plan.get('audio', {})) == set(STAGES) and
         set(plan.get('stages', {})) == set(STAGES), 'Exactly three preparation stages required')
    for pin in [*plan['pins'], plan['rehearsal_receipt']]:
        file_bytes(pin['path'], pin['sha256'])
    pinned_names={Path(pin['path']).name for pin in plan['pins']}
    need({'motor_epoch_readonly_capture.py','jetson_cpu_performance_scope.py'} <= pinned_names,
         'Capture module and imported CPU wrapper must also be pinned')
    for stage in STAGES:
        row = plan['stages'][stage]; argv = row.get('argv')
        need(type(argv) is list and bool(argv) and all(type(a) is str and a for a in argv)
             and Path(argv[0]).is_absolute(), 'Absolute executable argv required')
        need(not any('policy_output' in a or 'fixed-catch' in a or
                     a in ('--execute-supported', '--execute-supported-preload',
                           '--execute-human-supported-partial') for a in argv),
             'Actuation/partial-load commands are forbidden')
        required = '--execute-readonly' if stage == 'capture' else (
            '--execute-human-supported-zero-gain' if stage == 'watchdog' else '--supported-disabled')
        need(required in argv, 'Dedicated diagnostic flag missing: '+stage)
        if stage == 'pipeline':
            need('--diagnostic' in argv and '--execute' in argv and MODE in argv and
                 epoch in argv, 'No-output human provenance/power scope required')
            need(Path(row['records_path']).is_absolute(), 'Absolute pipeline records path required')
        if stage == 'watchdog':
            for option,value in (('--audio',plan['audio'][stage]['path']),
                                 ('--audio-sha256',plan['audio'][stage]['sha256']),
                                 ('--audio-device',plan['audio_device']),('--power-epoch',epoch)):
                need(option in argv and argv[argv.index(option)+1] == value,
                     'Owned watchdog audio/power argv differs from plan')
        need(Path(row['result_path']).is_absolute() and not Path(row['result_path']).exists(),
             'Fresh absolute child result required: '+stage)
        timeout = row['timeout_s']
        need(type(timeout) in (int, float) and math.isfinite(timeout) and 0 < timeout <= 120,
             'Finite diagnostic timeout at most120s required')
        clip = plan['audio'][stage]; raw = file_bytes(clip['path'], clip['sha256'], 16*1024*1024)
        with wave.open(io.BytesIO(raw), 'rb') as wav:
            need(wav.getcomptype() == 'NONE' and wav.getnchannels() == 2 and
                 wav.getsampwidth() == 2 and wav.getframerate() == 48000,
                 '48kHz stereo PCM16 announcement required')
            count = wav.getnframes(); duration = count/48000
            need(count > 0 and len(wav.readframes(count)) == count*4,
                 'Nonempty complete WAV required')
        need(type(clip.get('duration_s')) in (int,float) and
             abs(clip['duration_s']-duration) <= 1e-9 and duration <= 30 and
             type(clip.get('transcript')) is str and bool(clip['transcript']),
             'Pinned real duration and spoken meaning required')
    calibration_refresh_settings(plan)
    rehearsal = decode(file_bytes(plan['rehearsal_receipt']['path'], plan['rehearsal_receipt']['sha256']))
    need(rehearsal.get('schema') == 'singularitydog.human-supported-operator-receipt.v1' and
         rehearsal.get('kind') == 'rehearsal' and rehearsal.get('observed_by') == 'operator' and
         rehearsal.get('motor_power_off') is True and rehearsal.get('operator_count') == 2 and
         rehearsal.get('body_full_support_continuous') is True and
         rehearsal.get('box_removed_and_restored') is True and
         rehearsal.get('cutoff_role_maintained') is True and
         rehearsal.get('abnormal_noise_vibration_slip_sinking_contact') is False,
         'Prior real off-power rehearsal receipt required')
    return plan


def calibration_refresh_settings(plan):
    config = plan.get('diagnostic_calibration_refresh')
    if config is None:
        return None  # Old pinned plans retain their original, explicitly selected semantics.
    need(type(config) is dict and set(config) == {'schema','helper','base_calibration','expected_uids'} and
         config['schema'] == 'singularitydog.capture-bound-diagnostic-branch-refresh-plan.v1',
         'Exact diagnostic branch-refresh plan required')
    pins = {(row['path'],row['sha256']) for row in plan['pins']}
    for name in ('helper','base_calibration','expected_uids'):
        row=config[name]
        need(type(row) is dict and set(row)=={'path','sha256'} and
             (row['path'],row['sha256']) in pins, 'Branch-refresh source must be a pinned plan input: '+name)
        file_bytes(row['path'],row['sha256'])
    need(Path(config['helper']['path']).name == 'diagnostic_angle_branch.py', 'Dedicated file-only branch helper required')
    argv=plan['stages']['pipeline']['argv']
    for option,name in (('--calibration','base_calibration'),('--expected-uids','expected_uids')):
        need(argv.count(option)==1 and argv[argv.index(option)+1]==config[name]['path'],
             'Branch-refresh baseline argument mismatch: '+option)
    return config


def refresh_diagnostic_calibration(plan, capture_pin, out):
    config=calibration_refresh_settings(plan)
    need(config is not None, 'Explicit branch-refresh plan required')
    raw=file_bytes(config['helper']['path'],config['helper']['sha256'])
    module=types.ModuleType('capture_bound_diagnostic_branch_helper')
    module.__file__=config['helper']['path']
    exec(compile(raw,module.__file__,'exec'),module.__dict__)
    path=out/'capture-bound-diagnostic-calibration.json'
    result=module.derive_diagnostic_calibration(
        base_calibration_path=config['base_calibration']['path'],
        base_calibration_sha256=config['base_calibration']['sha256'],
        capture_path=capture_pin['path'],capture_sha256=capture_pin['sha256'],
        expected_uids_path=config['expected_uids']['path'],
        expected_uids_sha256=config['expected_uids']['sha256'],
        boot_id=plan['boot_id'],power_epoch=plan['motor_power_epoch'],output_path=str(path))
    need(type(result) is dict and result.get('path')==str(path) and
         result.get('output_allowed') is False and result.get('approved_for_runtime') is False,
         'Diagnostic derivation must not grant output approval')
    document=decode(file_bytes(result['path'],result['sha256']))
    source=document.get('diagnostic_branch_derivation',{})
    need(document.get('motor_output_allowed') is False and document.get('approved_for_runtime') is False and
         document.get('output_allowed') is False and document.get('motor_output_available') is False and
         document.get('motor_targets_generated') is False and document.get('calibration_verified') is False and
         document.get('live_50hz_verified') is False and
         document.get('raw_angles_modified') is False and document.get('source_current_boot_id')==plan['boot_id'] and
         document.get('motor_power_epoch')==plan['motor_power_epoch'] and
         source.get('fresh_capture')==capture_pin and source.get('base_calibration')==config['base_calibration'] and
         source.get('expected_uids')==config['expected_uids'] and
         source.get('scope')=='no_output_inference_only', 'Derived input provenance does not match fresh capture')
    return result


def pipeline_row_with_refreshed_calibration(plan, refresh):
    row=copy.deepcopy(plan['stages']['pipeline']);argv=row['argv']
    config=calibration_refresh_settings(plan)
    need(config is not None and argv.count('--calibration')==1 and
         argv[argv.index('--calibration')+1]==config['base_calibration']['path'],
         'Exactly one original calibration argument required')
    argv[argv.index('--calibration')+1]=refresh['path']
    return row


def verify_stops(report):
    for bus, ids in BUSES.items():
        row = report.get('stop_reports', {}).get(bus, {})
        need(row.get('complete') is True and row.get('confirmed_ids') == list(ids) and
             row.get('unconfirmed_ids') == [] and row.get('ambiguous_ids') == [] and
             row.get('errors') == [], 'Twelve causal unambiguous STOP replies required')


def verify_result(stage, result, plan, previous):
    need(type(result) is dict and result.get('errors') == [] and
         result.get('boot_id') == plan['boot_id'], 'Child errors or changed boot: '+stage)
    if stage == 'capture':
        need(result.get('status') == 'RECORDED_REVIEW_REQUIRED' and
             result.get('approved_for_runtime') is False and result.get('motor_output_allowed') is False and
             result.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL' and
             result.get('motor_power_epoch') in (plan['motor_power_epoch'], 'NOT_INFERRED_FROM_JETSON_BOOT'),
             'Read-only capture limitations must remain unchanged')
        need(set(result['identities']) == set(result['telemetry']['rows']) == {str(i) for i in range(1,13)},
             'Twelve identities/positions required')
        stamps = []; starts = []
        for mid, row in result['telemetry']['rows'].items():
            need(row.get('run_mode') == 0 and row.get('current') == 0 and
                 len(row.get('position_samples', [])) == 3, 'Quiet three-sample pose required')
            identity=result['identities'][mid]
            start,end=identity['request_monotonic_ns'],identity['reply_monotonic_ns']
            need(type(start) is int and type(end) is int and 0 < start <= end,
                 'Causal identity timestamps required')
            previous=end; values=[]
            for sample in row['position_samples']:
                start,end=sample['request_monotonic_ns'],sample['reply_monotonic_ns']
                need(type(start) is int and type(end) is int and 0 < previous <= start <= end and
                     end-start <= 30_000_000 and type(sample['rad']) in (int,float) and math.isfinite(sample['rad']),
                     'Causal finite fresh position samples required')
                previous=end; starts.append(start); stamps.append(end); values.append(sample['rad'])
            need(max(values)-min(values) <= math.radians(.1), 'Quiet pose span exceeded')
        need(max(stamps)-min(starts) <= 2_000_000_000, 'Whole stationary capture exceeds2s')
        return max(stamps)
    need(result.get('motor_power_epoch') == plan['motor_power_epoch'], 'Changed motor power epoch')
    if stage == 'watchdog':
        need(result.get('status') == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' and
             result.get('positive_gain_sent') is False and result.get('learned_targets_sent') is False and
             result.get('stop_confirmed') is True, 'Complete zero-gain watchdog required')
        verify_stops(result)
        need(set(result.get('axes', {})) == {str(i) for i in range(1,13)}, 'Twelve watchdog axes required')
        for row in result['axes'].values():
            need(previous <= row['version']['request_start_ns'] < row['stop_probe']['received_ns'] and
                 row.get('command_loss_tested') is True and row.get('disabled_on_command_loss') is True and
                 row['stop_probe'].get('mode_state') == 0 and row['stop_probe'].get('fault_bits') == 0,
                 'Fresh causal watchdog disable evidence required')
        return max(r['stop_probe']['received_ns'] for r in result['axes'].values())
    need(result.get('status') == 'COMPLETE_DIAGNOSTIC' and result.get('mode') == 'stop-proxy' and
         result.get('motor_enable_sent') is False and
         result.get('learned_targets_sent') is False and result.get('cycles_completed') ==
         result.get('cycles_requested') == 501 and result.get('imu_restore_status') in ('restored','not_needed'),
         'Complete restored disabled501 diagnostic required')
    observer=result.get('observer',{})
    need(observer.get('status') == 'COMPLETE_NO_OUTPUT_DIAGNOSTIC' and
         observer.get('ticks_requested') == observer.get('ticks_completed') == 501 and
         observer.get('failure') is None and observer.get('incomplete') is False and
         observer.get('output_allowed') is False, 'Actual501 no-output inference completion required')
    proof = result.get('source_provenance', {})
    need(proof.get('mode') == MODE and proof.get('motor_power_epoch') == plan['motor_power_epoch'] and
         proof.get('source_files_unchanged') is True, 'Exact current human source provenance required')
    measurements = result.get('measurements', [])
    need(len(measurements) == 501 and measurements[0]['release_ns'] >= previous,
         '501 diagnostic must follow watchdog')
    for index,row in enumerate(measurements):
        begin, oldest, replied, end = (row[k] for k in
            ('release_ns','oldest_input_start_ns','last_proxy_reply_ns','cycle_end_ns'))
        need(all(type(x) is int for x in (begin,oldest,replied,end)) and
             0 < begin <= oldest <= replied <= end and end-begin <= 20_000_000 and
             end-oldest <= 20_000_000 and replied-begin <= 20_000_000,
             'Strict whole/reply/input-age20ms required including startup')
        need(row.get('cadence_slot') == index and row.get('skipped_slots_before') == 0,
             'Absolute501 diagnostic skipped or reordered a slot')
    records_raw = file_bytes(plan['stages']['pipeline']['records_path'])
    need(digest(records_raw) == result['v3_voltage_fast_pipeline']['records_sha256'],
         'Pipeline records hash mismatch')
    records = decode(records_raw); need(type(records) is list and len(records) == 501, '501 raw traces required')
    for index, row in enumerate(records):
        need(row.get('cycle') == index+1 and
             measurements[index].get('cycle', index+1) == index+1,
             'Trace cycle order mismatch')
        for bus, ids in BUSES.items():
            output = row['output'][bus]['records']; need(len(output) == 6, 'Six STOP replies per bus required')
            for mid, item in zip(ids, output):
                rx, tx = bytes.fromhex(item['rx_hex']), bytes.fromhex(item['tx_hex'])
                need(len(rx) == len(tx) == 17 and rx[:2] == tx[:2] == b'AT' and
                     rx[-2:] == tx[-2:] == b'\r\n' and rx[6] == tx[6] == 8 and
                     int.from_bytes(rx[2:6],'big') == (((2<<24)|(mid<<8)|0xfd)<<3)|4 and
                     int.from_bytes(tx[2:6],'big') == (((4<<24)|(0xfd<<8)|mid)<<3)|4 and
                     tx[7:15] == bytes(8) and item['received'] == item['written'] == 17 and
                     item['start_ns'] <= item['finish_ns'] <= item['received_ns'] <= measurements[index]['cycle_end_ns'],
                     'Exact fault-free mode0 STOP frame required')
    return measurements[-1]['cycle_end_ns']


def run_child(argv, timeout_s, cancelled):
    child = subprocess.Popen(argv, start_new_session=True)  # inherit visible TTY, no shell/tee
    deadline = time.monotonic()+timeout_s
    try:
        while child.poll() is None:
            if cancelled or time.monotonic() >= deadline: raise InterruptedError('Stage interrupted or timed out')
            time.sleep(.05)
        need(child.returncode == 0, 'Child diagnostic exited '+str(child.returncode))
    except BaseException:
        print(RECOVERY, flush=True)
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try: child.wait(timeout=30.)  # outer power wrapper owns its nested actual child cleanup
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=2.)
        raise


def start_requirements(out, visible):
    need(visible(), 'stdin/stdout must be the same visible local terminal')
    out = Path(out).expanduser().absolute()
    need(not out.exists() and not any((p/'.git').exists() for p in (out,*out.parents)), 'Fresh private output required')
    return out


def visible_terminal():
    return os.isatty(0) and os.isatty(1) and os.fstat(0).st_rdev == os.fstat(1).st_rdev


def run(plan, out, *, confirm=input, boot=lambda:Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        play=subprocess.run, child=run_child, visible=lambda:os.isatty(0) and os.isatty(1) and
        os.fstat(0).st_rdev == os.fstat(1).st_rdev):
    out = start_requirements(out, visible)
    out.mkdir(parents=True, mode=0o700)
    session = str(uuid.uuid4()); cancelled=[]; handlers={}; answers=[]; receipts={}
    report = dict(schema='singularitydog.human-supported-preparation-session.v1',
        status='ABORTED_PREPARATION', errors=[], output_allowed=False, approvals_created=False,
        partial_load_started=False, learned_targets_sent=False, physical_support_measured=False,
        session_id=session, boot_id=plan['boot_id'], motor_power_epoch=plan['motor_power_epoch'], stages=[])
    def ask(key, proposition):
        need(not cancelled and boot() == plan['boot_id'], 'Interrupted or changed boot')
        prompt = proposition+'\n本当に確認済みなら y を入力してEnter（Enterだけでは中止）。yes・はいも可。未確認・中止は n: '
        answer = confirm(prompt)
        record = dict(key=key, prompt=prompt, operator_response=answer, recorded_monotonic_ns=time.monotonic_ns(),
                      source_message_id='visible-tty:'+session+':'+key)
        answers.append(record)
        need(type(answer) is str and answer.strip().casefold() in ('y','yes','はい'),
             'Operator did not confirm '+key)
        return dict(schema='singularitydog.human-supported-operator-receipt.v1', kind=key,
            observed_by='operator', source_message_id=record['source_message_id'],
            user_statement=prompt+'\nOperator response: '+answer, review=None,
            boot_id=plan['boot_id'], motor_power_epoch=plan['motor_power_epoch'])
    try:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            handlers[sig]=signal.signal(sig,lambda number,_:cancelled.append(number))
        power=ask('power','今回、40V Offで箱から2人の全支持へ移し、箱を外してから40V Onにしました。'
                  'その後は電源を操作していません。予定の新電源世代で診断します。')
        power.update(power_epoch_origin='operator_statement',off_on_confirmed=True,motor_power_on=True,
                     no_power_operation_since_capture=True)
        pose=ask('pose','箱は完全に外れ、1人が胴体の全重量を支え続け、手を離しません。'
                 'もう1人が端末操作と即時40V Offを担当し、担当者は合計2人です。'
                 '4足は床接地、全12軸は同時に静止、脚・配線の干渉はありません。'
                 'この診断では支える力を緩めません。')
        pose.update(pose_kind='human_full_support',box_removed=True,operator_count=2,full_body_weight_supported=True,
            all_four_paws_on_floor=True,all_axes_simultaneously_stationary=True,continuous_body_catch=True,
            hands_remain_on_body=True,legs_and_wiring_contact_free=True)
        video=ask('video','固定した横動画で、胴体・4足・支える手を同時に記録できる準備が整っています。')
        video.update(side_view_recording_ready=True,camera_fixed=True,body_four_paws_and_supporting_hands_visible=True)
        clearance=ask('clearance','現在姿勢から全12関節の±3°経路は、固定具・箱・配線に干渉しません。'
                      '姿勢を維持し、すぐ40VをOffにできます。')
        clearance.update(selected_ids=list(range(1,13)),local_clearance_rad=math.radians(3),legs_and_wiring_contact_free=True,
                         pose_maintained_since_capture=True,immediate_40v_cutoff_ready=True)
        previous=0; pins={}; branch_refresh=None
        for stage in STAGES:
            need(not cancelled and boot() == plan['boot_id'], 'Interrupted or changed boot')
            for pin in plan['pins']: file_bytes(pin['path'],pin['sha256'])
            clip=plan['audio'][stage]; raw=file_bytes(clip['path'],clip['sha256'])
            if stage != 'watchdog':  # The dedicated child announces once after its zero-gain setup.
                with tempfile.TemporaryFile() as snapshot:
                    snapshot.write(raw); snapshot.flush(); snapshot.seek(0)
                    play(['aplay','-D',plan['audio_device']],stdin=snapshot,stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL,check=True,timeout=clip['duration_s']+.25)
            need(not cancelled and boot() == plan['boot_id'], 'Interrupted after announcement')
            row=plan['stages'][stage]
            if stage == 'pipeline' and branch_refresh is not None:
                file_bytes(branch_refresh['path'],branch_refresh['sha256'])
                row=pipeline_row_with_refreshed_calibration(plan,branch_refresh)
                report['pipeline_execution_argv']=copy.deepcopy(row['argv'])
                write(out,'pipeline-effective-argv.json',row['argv'])
            child(row['argv'],row['timeout_s'],cancelled)
            need(not cancelled and boot() == plan['boot_id'], 'Interrupted after diagnostic')
            result_raw=file_bytes(row['result_path']); result=decode(result_raw)
            if stage == 'watchdog':
                speech=result.get('announcement',{})
                need(speech.get('process_completed') is True and speech.get('sha256') == clip['sha256'],
                     'Owned watchdog announcement did not complete')
            if stage == 'pipeline' and branch_refresh is not None:
                need(result.get('input_sha256',{}).get('calibration')==branch_refresh['sha256'],
                     'Pipeline did not use the exact capture-bound calibration')
                file_bytes(branch_refresh['path'],branch_refresh['sha256'])
            previous=verify_result(stage,result,plan,previous)
            pins[stage]={'path':row['result_path'],'sha256':digest(result_raw)}
            report['stages'].append({'stage':stage,'status':result['status'],'result':pins[stage]})
            if stage == 'capture' and plan.get('diagnostic_calibration_refresh') is not None:
                branch_refresh=refresh_diagnostic_calibration(plan,pins['capture'],out)
                report['diagnostic_calibration_refresh']=copy.deepcopy(branch_refresh)
                write(out,'diagnostic-branch-refresh-result.json',branch_refresh)
                print('新しい角度記録から360度の枝を診断用に合わせました。原点・方向・可動域は変更しません。',flush=True)
        pose['capture_sha256']=clearance['capture_sha256']=pins['capture']['sha256']
        physical=ask('physical_observation','watchdogの音声は聞こえました。診断中は異音・振動・滑り・沈み・接触なし、'
                     '人の全支持を継続し、収録から姿勢も電源も変えていません。')
        physical.update(report_sha256=pins['watchdog']['sha256'],audio_heard=True,
            abnormal_noise_vibration_slip_sinking_contact=False,human_full_support_maintained=True,
            pose_maintained_since_capture=True)
        power['post_capture_no_power_operation_source_message_id']=physical['source_message_id']
        for receipt in (power,pose,video,clearance,physical):
            receipts[receipt['kind']]=write(out,receipt['kind']+'-receipt.json',receipt)
        receipts['rehearsal']=plan['rehearsal_receipt']
        report.update(status='DIAGNOSTICS_RECORDED_REVIEW_REQUIRED',source_receipts=receipts,
                      raw_capture_unchanged=True,approvals_created=False)
        print('診断だけ完了しました。全支持を継続してください。部分荷重や保持試験は開始しません。',flush=True)
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error)); print(RECOVERY,flush=True)
    finally:
        for sig, handler in handlers.items():signal.signal(sig,handler)
        report['operator_confirmations']=write(out,'operator-confirmations.json',answers)
        write(out,'report.json',report)
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',required=True);p.add_argument('--expected-plan-sha256',required=True)
    p.add_argument('--output',required=True);p.add_argument('--execute-human-full-support-diagnostics',action='store_true')
    args=p.parse_args(argv)
    try:plan=check_plan(decode(file_bytes(args.plan,args.expected_plan_sha256)))
    except (OSError,ValueError,KeyError,TypeError,wave.Error) as error:p.error(str(error))
    if not args.execute_human_full_support_diagnostics:
        print(json.dumps(dict(status='PLAN_ONLY',hardware_opened=False,output_allowed=False,
            approvals_created=False,stages=list(STAGES),scope='human_full_support_diagnostics_only')));return 0
    try:
        start_requirements(args.output, visible_terminal)
    except (OSError, ValueError) as error:
        print(RECOVERY,flush=True)
        print(json.dumps(dict(status='ABORTED_PREPARATION',errors=[type(error).__name__+': '+str(error)],
            output_allowed=False,approvals_created=False,partial_load_started=False,hardware_opened=False),
            ensure_ascii=False))
        return 2
    report=run(plan,args.output)
    print(json.dumps({k:report[k] for k in ('status','errors','output_allowed','approvals_created','partial_load_started')},ensure_ascii=False))
    return 0 if report['status']=='DIAGNOSTICS_RECORDED_REVIEW_REQUIRED' else 2


if __name__=='__main__':raise SystemExit(main())
