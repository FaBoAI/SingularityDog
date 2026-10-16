"""Pinned four-owner zero-gain watchdog producer, file-only PLAN unless explicit.

This is the predecessor of the STOP501/zero2/human-hold8 route, so it authenticates
only source/native, current UID/pose/boot/power, prior boxed observation, human
conditions and audio. It never manufactures downstream evidence or a human
Prepared/output grant. The original command-loss engine owns serial reads.
"""
import argparse
import copy
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import stat
import sys
import time
from types import SimpleNamespace

from experiments.four_bus_diagnostic import foreground as FG, topology
from experiments.four_bus_type1 import type1_foreground as boxed_foreground
from experiments.four_bus_type1 import type1_profile as boxed
from . import foreground as human_foreground, profile as HP, watchdog as W

REQUIRED = {'source_root','source_count','source_manifest','source_review','implementation_receipt',
            'model_inputs','current_conditions','audio_manifest','prior_boxed_profile',
            'prior_boxed_report','prior_boxed_observation'}
_TOKEN = object()


class Prepared:
    """Distinct immutable watchdog-only source/pose proof; never human output admission."""
    __slots__ = ('_token','plan','pins','firmware','capture','audio','output','holder','digest')
    def __init__(self, token, plan, pins, firmware, capture, audio, output, holder):
        HP.need(token is _TOKEN, 'Use the watchdog file preparation entry')
        for key,value in (('_token',token),('plan',HP._freeze(copy.deepcopy(plan))),
            ('pins',HP._freeze(dict(pins.values))),('firmware',HP._freeze(copy.deepcopy(firmware))),
            ('capture',HP._freeze(copy.deepcopy(capture))),('audio',HP._freeze(copy.deepcopy(audio))),
            ('output',str(output)),('holder',HP._freeze(copy.deepcopy(holder))),
            ('digest',HP.canonical_sha256(plan))):
            object.__setattr__(self,key,value)
    def __setattr__(self,key,value): raise AttributeError('Watchdog proof is immutable')
    def __delattr__(self,key): raise AttributeError('Watchdog proof is immutable')


def verify_prepared(value):
    HP.need(type(value) is Prepared and value._token is _TOKEN and
            HP.canonical_sha256(HP._thaw(value.plan)) == value.digest,
            'Genuine distinct watchdog preparation required')
    plan = HP._thaw(value.plan)
    HP.validate_contract(plan['contract'])
    HP.need(all(plan.get(key) is expected for key,expected in HP.NO_GRANTS.items()),
            'Watchdog preparation acquired a public output grant')
    pins=HP.Pins()
    for path,digest in HP._thaw(value.pins).items():pins.read(path,digest)
    return value


def prepare(cli):
    """All predecessor files are checked before serial, CDLL, terminal or audio use."""
    pins=HP.Pins();plan=pins.json(cli.plan,cli.plan_sha256)
    HP.need(type(plan) is dict and plan.get('schema') == HP.SCHEMA and plan.get('status') == 'PLAN_ONLY' and
            all(plan.get(key) is expected for key,expected in HP.NO_GRANTS.items()),
            'Ungranting independent human numerical PLAN required')
    contract=HP.validate_contract(copy.deepcopy(plan.get('contract')))
    HP.need(plan.get('contract_sha256') == HP.canonical_sha256(contract) and
            plan.get('mode') in HP.MODES and plan.get('duration_s') == HP.DURATIONS[plan['mode']],
            'Human contract/mode/duration binding differs')
    refs,base=plan['evidence'],contract['base_numerical_contract']
    HP.need(type(refs) is dict and set(refs) <= HP.EVIDENCE_KEYS and REQUIRED <= set(refs),
            'Watchdog predecessor source/current/human/audio files incomplete')
    HP.need(refs['source_manifest'] == base['source_manifest'], 'New source differs from human contract')
    manifest=HP.verify_manifest(refs['source_root'],refs['source_manifest'],refs['source_count'],pins)
    for member,actual in (('watchdog.py',W.__file__),('watchdog_foreground.py',__file__)):
        path=Path(refs['source_root'])/'runtime/experiments/four_bus_human_hold'/member
        HP.need(Path(actual).absolute() == path and str(path.relative_to(refs['source_root'])) in manifest['files'],
                'Watchdog implementation is not from its new frozen kit')
    review=pins.json(refs['source_review'])
    HP.need(type(review) is dict and review.get('schema') == HP.SOURCE_REVIEW_SCHEMA and
            review.get('scope') == HP.SCOPE and review.get('decision') == 'REVIEWED_HUMAN_HOLD30_SOURCE_ONLY' and
            review.get('source_manifest_sha256') == refs['source_manifest']['sha256'] and
            review.get('old_boxed_or_two_bus_permission_reused') is False and
            review.get('tests_grant_output_permission') is False, 'Independent new-source review required')
    for key in ('reviewer','reviewed_at','rationale'):HP._text(review.get(key),'watchdog source review '+key)
    receipt=pins.json(refs['implementation_receipt'])
    paths=('runtime/experiments/four_bus_human_hold/runner.py',
           'runtime/experiments/four_bus_human_hold/type1_runner.py',
           'runtime/experiments/four_bus_human_hold/profile.py',
           'runtime/experiments/four_bus_human_hold/supervision.py')
    HP.need(receipt.get('schema') == HP.IMPLEMENTATION_SCHEMA and receipt.get('scope') == HP.SCOPE and
            receipt.get('source_manifest_sha256') == refs['source_manifest']['sha256'] and
            receipt.get('decision') == 'REVIEWED_INDEPENDENT_HUMAN_HOLD30_INTEGRATION' and
            receipt.get('tests_grant_output_permission') is False and receipt.get('old_boxed_permission_reused') is False and
            receipt.get('runtime_entry') == paths[0] and receipt.get('independent_type1_runner') == paths[1] and
            receipt.get('source_sha256') == {path:manifest['files'][path]['sha256'] for path in paths},
            'Independent source/native integration review required before watchdog enable')
    for key in ('reviewer','reviewed_at','rationale'):HP._text(receipt.get(key),'watchdog integration review '+key)
    HP._verify_native_integration(receipt,refs['source_root'],manifest,pins)
    model,capture=HP.rebuild_current_model(base,refs['model_inputs'],pins)
    HP.need(model == base['model_plan'],'Current human pose differs from the exact rebuilt numerical plan')
    from singularitydog_hw import policy_model_30hz
    HP.need(policy_model_30hz.plan(base['policy30_model']['artifacts']) == base['policy30_model'],
            'Retrained30Hz artifacts changed')
    for reference in base['policy30_model']['artifacts'].values():pins.raw(reference)
    conditions=pins.json(refs['current_conditions'])
    HP.need(conditions == plan['conditions'],'Pinned current human conditions differ')
    HP.validate_conditions(conditions,contract,capture_sha256=refs['model_inputs']['topology']['sha256'])
    prior_profile=pins.json(refs['prior_boxed_profile']);boxed.validate_profile(prior_profile)
    old=prior_profile['contract']
    HP.need(old.get('schema') == boxed.PROFILE_SCHEMA_30 and old['policy30_model'] == base['policy30_model'],
            'Prior boxed2 must use the exact retrained model')
    prior_report=pins.json(refs['prior_boxed_report'])
    observation_sha=HP.verified_predecessor_observation_sha(prior_report,prior_profile,
        report_ref=refs['prior_boxed_report'],profile_ref=refs['prior_boxed_profile'],
        source_root=refs['source_root'],manifest=manifest,pins=pins)
    prior=boxed.validate_type1_report(prior_report,contract=old,
        contract_digest=prior_profile['contract_sha256'],mode='learned_boxed',duration_s=2)
    observed=pins.json(refs['prior_boxed_observation'])
    HP.need(observed.get('source') == 'direct_current_user_reply' and
            observed.get('report_sha256') == observation_sha and
            observed.get('audio_heard') is True and observed.get('synthetic_interaction') is False and
            observed.get('abnormal_noise_vibration_slip_sinking_contact') is False,
            'Direct physical prior boxed2 observation required')
    HP._text(observed.get('user_statement'),'prior physical observation')
    HP._text(observed.get('user_reply_id'),'prior physical reply')
    HP.need(old['boot_id'] == base['boot_id'] and
            prior['terminal_stop_finished_monotonic_ns'] < capture['started_monotonic_ns'] < capture['finished_monotonic_ns'],
            'Fresh human capture must follow same-boot boxed2 STOP')
    firmware=boxed_foreground.reviewed_firmware(base,pins)
    audio_doc=pins.json(refs['audio_manifest'])
    HP.need(audio_doc.get('contract_sha256') == HP.canonical_sha256(contract),'Watchdog audio binds another contract')
    audio=HP._audio(audio_doc,contract['runtime_profile'],pins)['clips']['brief']
    HP.need(audio['duration_s'] <= 8 and type(cli.audio_device) is str and
            cli.audio_device.strip() == cli.audio_device != '', 'Pinned watchdog brief8s and explicit device required')
    audio={**audio,'device':cli.audio_device}
    output=FG.fresh_output(cli.output,refs['source_root'])
    HP.need((cli.holder_receipt is None) == (cli.holder_receipt_sha256 is None),'Holder observation path and SHA required')
    holder=(FG.validate_holder_receipt(pins.json(cli.holder_receipt,cli.holder_receipt_sha256),capture)
            if cli.holder_receipt is not None else None)
    pins.verify()
    return Prepared(_TOKEN,plan,pins,firmware,capture,audio,output,holder)


class LinuxEnvironment(human_foreground.LinuxEnvironment):
    """The canonical guarded OS source owns four serial Channels, never a Type1 CDLL."""
    def load_guard(self,refs):
        from experiments.four_bus_diagnostic import native_current_guard
        return native_current_guard.load_library(refs['library']['path'],
            expected_sha256=refs['library']['sha256'],source_sha256=refs['source']['sha256'],
            build_record_sha256=refs['build_record']['sha256'])

    def channels(self,stack,bindings,check,ids_by_port,channels):
        import serial
        descriptors,boot_fds={},{}
        for port in HP.PORTS:
            check()
            item=serial.Serial(port=None,baudrate=921600,timeout=0,write_timeout=.02,exclusive=True)
            stack.callback(item.close);item.dtr=False;item.rts=False
            item.port=bindings[port]['path'];item.open()
            info=os.fstat(item.fileno())
            HP.need(stat.S_ISCHR(info.st_mode) and info.st_rdev == bindings[port]['st_rdev'],
                    'Watchdog serial FD differs from fresh topology')
            descriptors[port]=item.fileno()
            boot_fds[port]=stack.enter_context(FG.BOOT_PATH.open('rb')).fileno()
            channels[port]=W.Channel(item,ids_by_port[port],check=check)
        return channels,descriptors,boot_fds


def execute(prepared,environment=None):
    prepared=verify_prepared(prepared)  # Before constructing/opening any capability.
    plan=HP._thaw(prepared.plan);base=plan['contract']['base_numerical_contract']
    capture,audio=HP._thaw(prepared.capture),HP._thaw(prepared.audio)
    refs=plan['evidence'];pins=HP.Pins()
    for path,digest in HP._thaw(prepared.pins).items():pins.read(path,digest)
    env=LinuxEnvironment() if environment is None else environment
    output=Path(prepared.output);output.mkdir(mode=0o700)
    channels,handlers={},{};cancelled=[];current_guard=None
    report={**W.plan(plan['contract']),'status':'ABORTED_BEFORE_ENABLE','errors':[],
        'motor_enable_sent':False,'stop_confirmed':False,'source_and_input_pins_unchanged':False,
        'input_sha256':dict(pins.values),'restoration_complete':False}
    def check():
        HP.need(not cancelled,'Operator cancelled watchdog')
        HP.need(env.boot_id() == base['boot_id'],'Watchdog boot changed')
        if current_guard is not None:current_guard()
    def note(error):
        report['errors'].append(W._error_text(error))
        if report['status'].startswith('COMPLETE'):report['status']='ABORTED'
    try:
        report['startup']=env.startup();report['terminal_binding']=env.verify_terminal()
        HP.need(env.boot_id() == capture['boot_before'] == base['boot_id'],'Current watchdog boot differs')
        bindings=env.bind(capture);HP.need(bindings == capture['ports'],'Current four USB identities differ')
        report['holders']=env.holders(bindings)
        native_guard=pins.json(refs['implementation_receipt'])['native_current_guard']
        guard_library=env.load_guard(native_guard)
        if prepared.holder is not None:
            holder=HP._thaw(prepared.holder);FG.validate_holder_receipt(holder,capture)
            HP.need(holder['checked_monotonic_ns'] <= env.clock(),'Holder observation is from a future clock')
            report['root_holder_observation']=holder
        with ExitStack() as stack:
            env.locks(stack,bindings);report['holders_after_locks']=env.holders(bindings)
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
                handlers[sig]=signal.signal(sig,lambda number,_frame:cancelled.append(number))
            try:
                _,descriptors,boot_fds=env.channels(stack,bindings,check,base['topology_by_port'],channels)
                current_guard=env.current_guard(guard_library,bindings,descriptors,
                    boot_fds[HP.PORTS[0]],base['boot_id'],SimpleNamespace(is_set=lambda:bool(cancelled)))
                stack.callback(FG.finish_current_guard,current_guard,report,note)
                measured=W.run(channels,plan['contract'],HP._thaw(prepared.firmware),check=check,
                    clock=env.clock,announce=env.announcer(audio,check),execute=True)
                report.update(measured)
            finally:
                if channels and 'stop_reports' not in report:
                    stops={}
                    for port,channel in channels.items():
                        try:stops[port]=channel.stop_all()
                        except BaseException as error:
                            stops[port]={'complete':False,'confirmed_ids':[],
                                'unconfirmed_ids':list(channel.ids),'ambiguous_ids':list(channel.ids),
                                'errors':[W._error_text(error)]}
                    report['stop_reports']=stops
                    report['stop_confirmed']=(set(stops) == set(HP.PORTS) and
                        all(row.get('complete') is True and not row.get('unconfirmed_ids') and
                            not row.get('ambiguous_ids') and not row.get('errors') for row in stops.values()))
    except BaseException as error:
        note(error)
    finally:
        # W.run owns normal/emergency STOP. If setup failed after opening owners,
        # every such owner must still be stopped before its descriptors disappear.
        if channels and 'stop_reports' not in report:
            report['status']='STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
            report['errors'].append('Setup ended before bounded owner STOP; physically switch40VOff')
        for sig,handler in handlers.items():
            try:signal.signal(sig,handler)
            except BaseException as error:note(error)
        try:
            verify_prepared(prepared);pins.verify()
            HP.verify_manifest(refs['source_root'],refs['source_manifest'],refs['source_count'],pins)
            report['source_and_input_pins_unchanged']=True
        except BaseException as error:note(error)
        report['restoration_complete']=(bool(channels) and 'stop_reports' in report and
            all(signal.getsignal(sig) == handler for sig,handler in handlers.items()) and
            report.get('current_check_profile',{}).get('native_guard_closed') is True and
            report.get('current_check_profile',{}).get('end_full_path_resolution_verified') is True)
        report['hardware_opened']=report['opens_devices']=bool(channels)
        report['physical_cutoff_required']=report.get('stop_confirmed') is not True and bool(channels)
        if not report['restoration_complete'] and report['status'].startswith('COMPLETE'):
            report['status']='ABORTED';report['errors'].append('Watchdog owner/guard restoration incomplete')
        report['input_sha256']=dict(pins.values)
        report['outer_power_restoration_verified_here']=False
        trace=W.trace(report,channels)
        for name,document in (('report.json',report),('trace.json',trace)):
            raw=(json.dumps(document,indent=2,allow_nan=False)+'\n').encode()
            with os.fdopen(os.open(output/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb') as handle:
                handle.write(raw);handle.flush();os.fsync(handle.fileno())
    return report


def parser():
    return human_foreground.parser()


def main(argv=None):
    cli=parser().parse_args(argv)
    if not cli.execute:
        result=human_foreground.file_plan(cli)
        result.update(status='PLAN_FOUR_CAN_WATCHDOG_NOT_EXECUTED',positive_gains_available=False,
            learned_targets_available=False,usb_disconnect_tested=False)
        print(json.dumps(result,indent=2,allow_nan=False));return 0
    report=execute(prepare(cli))
    print(json.dumps({key:report.get(key) for key in ('status','errors','motor_enable_sent','stop_confirmed',
        'physical_cutoff_required','positive_gain_sent','learned_targets_sent')}))
    return 0 if report['status'] == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' else 2


if __name__ == '__main__':raise SystemExit(main())
