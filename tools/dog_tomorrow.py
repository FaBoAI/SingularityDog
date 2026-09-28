#!/usr/bin/env python3
"""Short entry point for the overnight kit. Default prints commands; no devices.

Use the same existing Jetson CPU-PyTorch Python for build and all later actions.
This entry point only builds software and prepares diagnostics. Other explicit,
reviewed entry points in newer kits own actuation; building does not arm them.
"""
import argparse
from contextlib import contextmanager, nullcontext
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tempfile
import time


ROOT=Path(__file__).resolve().parents[1]
ACTIVE_DIR=Path('runtime/experiments/native_active_transport')
DIAGNOSTIC_DIR=Path('runtime/experiments/native_transport')


class DiagnosticBlocked(ValueError):
    def __init__(self,motor_ids):
        self.motor_ids=motor_ids
        super().__init__('Static Type17/Type2 comparison differs or is inconclusive for IDs '+','.join(map(str,motor_ids)))


def _regular_build_parents(root,path):
    """Check each bundled parent before accessing an artifact through it."""
    current=root
    for part in (None,*path.relative_to(root).parts[:-1]):
        if part is not None:current=current/part
        if current.is_symlink() or (current.exists() and not current.is_dir()):
            raise ValueError('Nonregular build artifact parent: '+str(current))


def _snapshot_build_artifact(root,path):
    _regular_build_parents(root,path)
    try:
        fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    except FileNotFoundError:
        return None
    except OSError as error:
        raise ValueError('Cannot safely snapshot build artifact: '+str(path)) from error
    with os.fdopen(fd,'rb') as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ValueError('Nonregular build artifact: '+str(path))
        return stream.read(),stat.S_IMODE(info.st_mode)


def _restore_build_artifact(root,path,snapshot):
    _regular_build_parents(root,path)
    if snapshot is None:
        try:info=path.lstat()
        except FileNotFoundError:return
        if not (stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode)):
            raise ValueError('Refusing to remove nonregular build artifact: '+str(path))
        path.unlink()  # Removes a new symlink itself; never follows its target.
        return
    raw,mode=snapshot
    fd,name=tempfile.mkstemp(prefix='.dog-build-restore-',dir=path.parent)
    temporary=Path(name)
    try:
        with os.fdopen(fd,'wb') as stream:
            os.fchmod(stream.fileno(),mode)
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)  # Atomic replacement, including a changed symlink.
    finally:
        if temporary.exists():temporary.unlink()


@contextmanager
def build_artifact_transaction(root,*,include_active,state_path):
    """Rollback only known builder outputs until their state is published.

    Native builders compile at canonical paths. Preserve their previous bytes
    and modes across compilation, verification and state-publication failures.
    Unique model-output directories remain available as failed diagnostics.
    """
    paths=[root/DIAGNOSTIC_DIR/name for name in ('libdog_transport.so','build-record.json')]
    if include_active:
        paths += [root/ACTIVE_DIR/name for name in ('libdog_active_transport.so','build-record.json')]
    targets=[(root,path) for path in paths]+[(state_path.parent,state_path)]
    snapshots={(base,path):_snapshot_build_artifact(base,path) for base,path in targets}
    try:
        yield
    except BaseException as original:
        failures=[]
        for (base,path),snapshot in snapshots.items():
            try:_restore_build_artifact(base,path,snapshot)
            except BaseException as error:failures.append(str(path)+': '+str(error))
        if failures:
            raise RuntimeError('Build failed and artifact rollback was incomplete: '+'; '.join(failures)) from original
        raise


def active_build_paths(config):
    """Only the canonical bundled builder may enter the no-hardware build plan."""
    section=config.get('supported_policy_output')
    if section is None:return None
    if (not isinstance(section,dict) or section.get('active_transport_build')!=str(ACTIVE_DIR/'build.py')
            or section.get('active_transport_library')!=str(ACTIVE_DIR/'libdog_active_transport.so')
            or section.get('build_active_library_on_target_required') is not True):
        raise ValueError('Invalid bundled active-transport build declaration')
    return ROOT/ACTIVE_DIR/'build.py', ROOT/ACTIVE_DIR/'libdog_active_transport.so'


def active_build_record(library):
    """Verify source/binary correspondence, without loading or opening a device."""
    source=library.parent/'transport.cpp';record_path=library.parent/'build-record.json'
    for path in (library,source,record_path):
        if not path.is_file() or path.is_symlink():raise ValueError('Missing regular active build artifact')
    record=json.loads(record_path.read_text())
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    if (type(record.get('abi')) is not int or record['abi']!=1 or record.get('source_sha256')!=digest(source)
            or record.get('binary_sha256')!=digest(library)):
        raise ValueError('Active transport build record/source/binary mismatch')
    return {'active_transport_library':str(library),
            'active_transport_library_sha256':digest(library),
            'active_transport_build_record':str(record_path),
            'active_transport_build_record_sha256':digest(record_path)}

def verify_kit(root):
    manifest=json.loads((root/'kit-manifest.json').read_text())
    if manifest.get('schema')!='private-overnight-kit-v1' or not manifest.get('files'):
        raise ValueError('Missing private kit manifest; prepare and copy the complete kit')
    for name,digest in manifest['files'].items():
        path=root/name
        if Path(name).is_absolute() or '..' in Path(name).parts or path.is_symlink():
            raise ValueError('Nonlocal kit manifest path')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Kit file missing or changed: '+name)
    for tree in ('runtime','tools'):
        for path in (root/tree).rglob('*'):
            if path.is_file() and path.suffix in ('.py','.cpp','.h') and str(path.relative_to(root)) not in manifest['files']:
                raise ValueError('Unlisted executable kit source: '+str(path.relative_to(root)))

def write_state(path,data):
    temp=path.with_suffix('.new')
    fd=os.open(temp,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    try:
        with os.fdopen(fd,'w') as f:
            json.dump(data,f,indent=2);f.write('\n')
        temp.replace(path)
    finally:
        if temp.exists():temp.unlink()


def diagnostics_plan(prefix,ports,input_path,work,state,stamp,cycles,*,request_window=3,request_gap_us=600):
    """Fixed diagnostic sequence; config cannot introduce commands or actions."""
    output=work/('diagnostics-'+stamp)
    capture=output/'capture.json';candidate=output/'calibration.json';audit=output/'angle-audit.json'
    stages=[{'name':'capture','output':str(output),'report':str(capture),'commands':[
        prefix+['singularitydog_hw.motor_epoch_readonly_capture',*ports,'--execute-readonly','--output',str(capture)],
        [sys.executable,str(ROOT/'tools/audit_angle_calibration.py'),'--profile',input_path('angle_profile'),
         '--capture',str(capture),'--output',str(audit),'--policy-candidate-output',str(candidate)]]}]
    for name in ('can','compare','full'):
        destination=output/name;count=min(3,cycles) if name=='compare' else cycles
        command=prefix+['singularitydog_hw.native_pipeline_benchmark','--execute',*ports,
            '--library',str(ROOT/DIAGNOSTIC_DIR/'libdog_transport.so'),'--output',str(destination),'--cycles',str(count),
            '--request-window',str(request_window),'--request-gap-us',str(request_gap_us)]
        if name=='can':command+=['--mode','type17','--acquisition-only']
        elif name=='compare':command+=['--mode','stop-proxy','--supported-disabled','--acquisition-only','--compare-feedback']
        else:
            command+=['--mode','stop-proxy','--supported-disabled','--calibration',str(candidate),
                '--mount',state.get('mount',input_path('mount')),'--bundle',input_path('bundle'),
                '--native-policy-manifest',state.get('native_policy_manifest','<build-success-manifest>'),
                '--native-policy-manifest-sha256',state.get('native_policy_manifest_sha256','<build-success-sha256>')]
            if state.get('gyro_bias'):command+=['--gyro-bias',state['gyro_bias']]
        stages.append({'name':name,'output':str(destination),'report':str(destination/'report.json'),
                       'cycles':count,'request_window':request_window,'request_gap_us':request_gap_us,
                       'commands':[command]})
    return output,stages,{'calibration':str(candidate),'angle_capture':str(capture),'angle_audit':str(audit)}


def _diagnostic_json(path):
    path=Path(path)
    if path.is_symlink() or not path.is_file():raise ValueError('Missing regular diagnostic artifact: '+str(path))
    raw=path.read_bytes();data=json.loads(raw)
    if type(data) is not dict:raise ValueError('Diagnostic artifact must be an object: '+str(path))
    return data,hashlib.sha256(raw).hexdigest()


def _diagnostic_build(state):
    path=state.get('native_policy_manifest');expected=state.get('native_policy_manifest_sha256')
    if not isinstance(path,str) or not isinstance(expected,str):
        raise ValueError('Run build successfully before diagnostics')
    manifest,digest=_diagnostic_json(path)
    if (digest!=expected or manifest.get('schema')!='native-policy-overnight-v1'
            or manifest.get('status')!='VALIDATED_FILE_ONLY'):
        raise ValueError('Build manifest changed or is unvalidated; run build again')
    return digest


@contextmanager
def diagnostics_work_lock(work):
    """One consolidated publisher per work state; never waits or retries."""
    fd=os.open(work/'diagnostics.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW|os.O_NONBLOCK,0o600)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):raise ValueError('Nonregular diagnostics lock')
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as error:raise ValueError('Another diagnostics sequence owns this work directory') from error
        yield
    finally:os.close(fd)


def _diagnostic_capture(paths,*,with_candidate):
    captured,capture_sha=_diagnostic_json(paths['angle_capture'])
    if (captured.get('status')!='RECORDED_REVIEW_REQUIRED' or captured.get('errors')!=[]
            or captured.get('motor_output_allowed') is not False or captured.get('approved_for_runtime') is not False
            or not isinstance(captured.get('boot_id'),str) or not captured['boot_id']):
        raise ValueError('Fresh capture incomplete or scope invalid')
    if not with_candidate:return captured['boot_id'],capture_sha,None
    candidate,candidate_sha=_diagnostic_json(paths['calibration'])
    audit,_=_diagnostic_json(paths['angle_audit'])
    if (candidate.get('status')!='MANUAL_NOMINAL_CANDIDATES_ONLY'
            or candidate.get('source_capture_sha256')!=capture_sha
            or candidate.get('source_current_boot_id')!=captured['boot_id']
            or audit.get('current_capture_sha256')!=capture_sha
            or audit.get('current_boot_id')!=captured['boot_id']):
        raise ValueError('Fresh calibration/audit does not bind this capture and boot')
    return captured['boot_id'],capture_sha,candidate_sha


def _diagnostic_report(stage,boot,manifest_sha,candidate_sha):
    report,digest=_diagnostic_json(stage['report'])
    if (report.get('status')!='COMPLETE_DIAGNOSTIC' or report.get('errors')!=[]
            or report.get('boot_id')!=boot or report.get('cycles_completed')!=stage['cycles']
            or any(report.get(key) is not False for key in
                   ('motor_enable_sent','learned_targets_sent','approved_for_runtime','full_controller_50Hz_verified'))):
        raise ValueError('Diagnostic failed, incomplete, boot changed, or output scope invalid')
    if stage['name']=='compare':
        if report.get('kind')!='native_feedback_comparison_report':raise ValueError('Expected Type17/Type2 comparison report')
        motors=report.get('per_motor',{})
        if not isinstance(motors,dict) or set(motors)!={str(i) for i in range(1,13)}:
            raise ValueError('Comparison report must contain twelve motor results')
        disagreed=[i for i in range(1,13) if motors.get(str(i),{}).get('all_direct_static_comparisons_agree') is not True]
        if disagreed:raise DiagnosticBlocked(disagreed)
    elif report.get('mode')!=('type17' if stage['name']=='can' else 'stop-proxy'):
        raise ValueError('Unexpected diagnostic mode')
    if stage['name']=='full':
        observer=report.get('observer') or {}
        if (report.get('model_source',{}).get('manifest_sha256')!=manifest_sha
                or report.get('input_sha256',{}).get('calibration')!=candidate_sha
                or observer.get('status')!='COMPLETE_NO_OUTPUT_DIAGNOSTIC'
                or observer.get('ticks_completed')!=stage['cycles'] or observer.get('ticks_requested')!=stage['cycles']
                or observer.get('failure') is not None or observer.get('incomplete') is not False
                or observer.get('output_allowed') is not False):
            raise ValueError('Full diagnostic lacks fresh calibration or complete real-inference evidence')
    return report,digest


def execute_diagnostics(output,stages,updates,state,statefile,env):
    """Never resumes/retries; each run promotes only its own complete evidence."""
    output.mkdir(mode=0o700,exist_ok=False)
    summary_path=output/'summary.json'
    summary={'schema':'dog-consolidated-diagnostics-v1','status':'RUNNING','output':str(output),
        'summary':str(summary_path),'failed_stage':None,'blocked_stage':None,'errors':[],'pending_checks':[],
        'motor_enable_available':False,'motor_enable_sent':False,'learned_targets_sent':False,'approved_for_runtime':False,
        'dynamic_scale_validated':False,'physical_failure_inferred':False,
        'automatic_retry':False,'fresh_capture_promoted':False,
        'stages':[{k:v for k,v in stage.items() if k!='commands'}|{'status':'NOT_RUN'} for stage in stages]}
    # Invalidate the previous latest-success marker before starting any child.
    prior=dict(state)
    def publish_state():
        value=dict(prior)
        if summary['status']=='COMPLETE_DIAGNOSTICS':value.update(updates)
        value['last_diagnostics']={'status':summary['status'],'summary':str(summary_path),
            'failed_stage':summary['failed_stage'],'blocked_stage':summary['blocked_stage'],
            'fresh_capture_promoted':summary['fresh_capture_promoted']}
        write_state(statefile,value)
    def save():write_state(summary_path,summary)
    current=None;started=time.monotonic_ns()
    try:
        save();publish_state()
        manifest_sha=_diagnostic_build(state)
        expected_capture=None;expected_candidate=None
        for stage,current in zip(stages,summary['stages']):
            current['status']='RUNNING';stage_start=time.monotonic_ns();save()
            try:
                # Build/capture pins are rechecked before every downstream step.
                if _diagnostic_build(state)!=manifest_sha:raise ValueError('Build manifest changed during diagnostics')
                if stage['name']!='capture':
                    boot,capture_sha,candidate_sha=_diagnostic_capture(updates,with_candidate=True)
                    if (capture_sha,candidate_sha)!=(expected_capture,expected_candidate):
                        raise ValueError('Fresh capture/calibration changed during diagnostics')
                for number,command in enumerate(stage['commands']):
                    result=subprocess.run(command,env=env,cwd=ROOT,check=False)
                    current.setdefault('returncodes',[]).append(result.returncode)
                    if result.returncode:raise RuntimeError('Command failed with exit code '+str(result.returncode))
                    if stage['name']=='capture' and number==0:_diagnostic_capture(updates,with_candidate=False)
                if stage['name']=='capture':
                    boot,expected_capture,expected_candidate=_diagnostic_capture(updates,with_candidate=True)
                    updates['calibration_sha256']=expected_candidate
                    current.update(capture_sha256=expected_capture,calibration_sha256=expected_candidate)
                else:
                    report,digest=_diagnostic_report(stage,boot,manifest_sha,expected_candidate)
                    current.update(report_sha256=digest,report_status=report['status'],
                        timing_ms=report.get('distributions_ms'),phase_timings=report.get('phase_timings'),
                        cycles_completed=report['cycles_completed'],comparison_by_id=report.get('per_motor'))
                    _,capture_sha,candidate_sha=_diagnostic_capture(updates,with_candidate=True)
                    if (capture_sha,candidate_sha)!=(expected_capture,expected_candidate):
                        raise ValueError('Fresh capture/calibration changed during diagnostic child')
                if _diagnostic_build(state)!=manifest_sha:raise ValueError('Build manifest changed during diagnostic child')
                current['status']='COMPLETE'
            finally:current['elapsed_ms']=(time.monotonic_ns()-stage_start)/1e6
            save()
        summary['status']='COMPLETE_DIAGNOSTICS';summary['fresh_capture_promoted']=True
    except BaseException as error:
        if isinstance(error,DiagnosticBlocked):
            summary['status']='DIAGNOSTIC_BLOCKED';summary['blocked_stage']=current['name']
            summary['pending_checks']=[{'kind':'static_type17_type2_review','motor_ids':error.motor_ids,
                'reason':'Inspect disagreement/movement/timing evidence; physical fault and calibration failure are not inferred.'}]
        else:
            summary['status']='ABORTED';summary['failed_stage']=current['name'] if current else 'preparation'
        summary['errors'].append(type(error).__name__+': '+str(error))
        if current is not None:
            current['status']='BLOCKED' if isinstance(error,DiagnosticBlocked) else 'FAILED'
            try:
                child,digest=_diagnostic_json(current['report'])
                current.update(report_status=child.get('status'),report_errors=child.get('errors',[]),report_sha256=digest,
                    timing_ms=child.get('distributions_ms'),phase_timings=child.get('phase_timings'),
                    comparison_by_id=child.get('per_motor'))
                for flag in ('motor_enable_sent','learned_targets_sent'):
                    if child.get(flag) is True:summary[flag]=True
            except (OSError,ValueError,TypeError):pass
    finally:
        summary['elapsed_ms']=(time.monotonic_ns()-started)/1e6
        # Publish first, then claim promotion in the saved summary. If state
        # publication fails, leave an explicit failed summary and never retry a
        # child or silently re-use the previous latest-success marker.
        try:publish_state()
        except BaseException as error:
            summary['status']='ABORTED';summary['fresh_capture_promoted']=False
            summary['failed_stage']='state_publication'
            summary['errors'].append(type(error).__name__+': '+str(error))
            try:publish_state()  # Persist failure, never retry the diagnostic.
            except BaseException as failed:
                summary['errors'].append('Failure state could not be persisted: '+repr(failed))
        save()
        terminal={key:summary[key] for key in ('status','summary','failed_stage','blocked_stage',
            'errors','pending_checks','motor_enable_sent','learned_targets_sent','fresh_capture_promoted')}
        terminal['stages']=[{key:row[key] for key in ('name','status','output','elapsed_ms','report_status','report_errors')
                            if key in row} for row in summary['stages']]
        print(json.dumps(terminal,ensure_ascii=False,indent=2),flush=True)
    return 0 if summary['status']=='COMPLETE_DIAGNOSTICS' else 2


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('build','imu','capture','can','full','compare','diagnostics'))
    p.add_argument('--config',type=Path,default=ROOT/'kit-config.json')
    p.add_argument('--work-dir',type=Path,default=Path('~/singularitydog-logs/overnight-20260928'))
    p.add_argument('--execute',action='store_true')
    p.add_argument('--supported-disabled',action='store_true')
    p.add_argument('--motor-power-off',action='store_true')
    p.add_argument('--cycles',type=int)
    p.add_argument('--request-window',type=int,choices=(1,2,3),default=3,
                   help='Outstanding requests per bus for can/compare/full/diagnostics')
    p.add_argument('--request-gap-us',type=int,default=600,metavar='600..5000',
                   help='Write gap in microseconds for can/compare/full/diagnostics')
    args=p.parse_args(argv)
    if not 600<=args.request_gap_us<=5000:p.error('--request-gap-us must be 600..5000')
    if args.cycles is None:args.cycles=3 if args.action=='compare' else 20
    if not 1<=args.cycles<=3000:p.error('cycles must be1..3000')
    if args.action=='compare' and args.cycles>5:p.error('compare cycles must be1..5')
    config=json.loads(args.config.read_text());base=args.config.resolve().parent
    def input_path(key):return str((base/config[key]).resolve())
    work=args.work_dir.expanduser().resolve();statefile=work/'state.json'
    state=json.loads(statefile.read_text()) if statefile.exists() else {}
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    py=sys.executable
    prefix=[py,'-B','-m']
    ports=['--front-port',config['front_port'],'--rear-port',config['rear_port'],
           '--expected-uids',input_path('expected_uids')]
    commands=[];updates={}
    if args.action=='diagnostics':
        if args.execute and not args.supported_disabled:
            p.error('Diagnostics require --supported-disabled; keep torso support in place')
        output,stages,updates=diagnostics_plan(prefix,ports,input_path,work,state,stamp,args.cycles,
            request_window=args.request_window,request_gap_us=args.request_gap_us)
        commands=[command for stage in stages for command in stage['commands']]
    elif args.action=='build':
        artifact=work/('native-policy-'+stamp)
        commands=[[py,str(ROOT/'runtime/experiments/native_transport/build.py')]]
        active=active_build_paths(config)
        if active:commands.append([py,str(active[0])])
        commands.append(prefix+['native_policy_overnight','build','--bundle',input_path('bundle'),'--output',str(artifact)])
        updates['native_policy_manifest']=str(artifact/'manifest.json')
    elif args.action=='imu':
        if args.execute and not args.motor_power_off:p.error('IMU movement requires --motor-power-off')
        output=work/('imu-'+stamp)
        commands=[prefix+['singularitydog_hw.imu_commissioning_capture','--execute',
            '--motor-power-off','--body-supported','--output',str(output)]]
        updates.update(mount=str(output/'imu-mount-candidate.json'),gyro_bias=str(output/'gyro-bias-candidate.json'))
    elif args.action=='capture':
        capture=work/('angles-'+stamp+'.json');candidate=work/('calibration-'+stamp+'.json')
        commands=[prefix+['singularitydog_hw.motor_epoch_readonly_capture',*ports,
            '--execute-readonly','--output',str(capture)],
            [py,str(ROOT/'tools/audit_angle_calibration.py'),'--profile',input_path('angle_profile'),
             '--capture',str(capture),'--output',str(work/('angle-audit-'+stamp+'.json')),
             '--policy-candidate-output',str(candidate)]]
        updates.update(calibration=str(candidate),angle_capture=str(capture))
    else:
        if args.action in ('full','compare') and args.execute and not args.supported_disabled:
            p.error('STOP proxy requires --supported-disabled; keep torso support in place')
        output=work/(args.action+'-'+stamp)
        command=prefix+['singularitydog_hw.native_pipeline_benchmark','--execute',*ports,
            '--library',str(ROOT/'runtime/experiments/native_transport/libdog_transport.so'),
            '--output',str(output),'--cycles',str(args.cycles),
            '--request-window',str(args.request_window),'--request-gap-us',str(args.request_gap_us)]
        if args.action=='can':command+=['--mode','type17','--acquisition-only']
        elif args.action=='compare':
            command+=['--mode','stop-proxy','--supported-disabled','--acquisition-only','--compare-feedback']
        else:
            if args.execute and not all(key in state for key in ('native_policy_manifest','calibration',
                    'calibration_sha256','angle_capture')):
                p.error('Run build and capture successfully before full')
            if args.execute:
                candidate_bytes=Path(state['calibration']).read_bytes()
                capture_sha=hashlib.sha256(Path(state['angle_capture']).read_bytes()).hexdigest()
                if (hashlib.sha256(candidate_bytes).hexdigest()!=state['calibration_sha256'] or
                    json.loads(candidate_bytes).get('source_capture_sha256')!=capture_sha):
                    raise ValueError('Calibration or source capture changed; run capture again')
            command+=['--mode','stop-proxy','--supported-disabled',
                '--calibration',state.get('calibration',input_path('calibration')),
                '--mount',state.get('mount',input_path('mount')),'--bundle',input_path('bundle'),
                '--native-policy-manifest',state.get('native_policy_manifest','<build-success-manifest>'),
                '--native-policy-manifest-sha256',state.get('native_policy_manifest_sha256','<build-success-sha256>')]
            if state.get('gyro_bias'):command+=['--gyro-bias',state['gyro_bias']]
        commands=[command]
    printed={'action':args.action,'execute':args.execute,'work_dir':str(work),
        'commands':[shlex.join(c) for c in commands],'motor_enable_available':False,
        'learned_targets_sent':False}
    if args.action in ('can','compare','full','diagnostics'):
        printed.update(request_window=args.request_window,request_gap_us=args.request_gap_us)
    if args.action=='diagnostics':printed.update(status='PLAN' if not args.execute else 'REQUESTED',
        stages=[{k:v for k,v in stage.items() if k!='commands'} for stage in stages],
        summary=str(output/'summary.json'),requires_successful_build=True,automatic_retry=False)
    print(json.dumps(printed,ensure_ascii=False,indent=2),flush=True)
    if not args.execute:return 0
    verify_kit(ROOT)
    if any((parent/'.git').exists() for parent in (work,*work.parents)):
        p.error('Private work directory must be outside Git')
    work.mkdir(parents=True,mode=0o700,exist_ok=True)
    env=dict(os.environ,PYTHONPATH=os.pathsep.join((str(ROOT/'runtime'),str(ROOT/'runtime/experiments'))))
    if args.action=='diagnostics':
        with diagnostics_work_lock(work):
            # Refuse a plan made from state that changed while another publisher
            # finished. An explicit new invocation must generate a fresh plan.
            fresh=json.loads(statefile.read_text()) if statefile.exists() else {}
            if fresh!=state:raise ValueError('Work state changed during diagnostics preparation; invoke again')
            return execute_diagnostics(output,stages,updates,state,statefile,env)
    transaction=build_artifact_transaction(ROOT,include_active=bool(active),state_path=statefile) if args.action=='build' else nullcontext()
    with transaction:
        for command in commands:
            subprocess.run(command,env=env,cwd=ROOT,check=True)
        if args.action=='build':
            artifact=Path(updates['native_policy_manifest']).parent
            built=json.loads((artifact/'build-report.json').read_text())
            digest=hashlib.sha256(Path(updates['native_policy_manifest']).read_bytes()).hexdigest()
            if built.get('manifest_sha256')!=digest:raise ValueError('Build report/manifest SHA mismatch')
            updates['native_policy_manifest_sha256']=digest
            if active:updates.update(active_build_record(active[1]))
        elif args.action=='capture':
            candidate=Path(updates['calibration']).read_bytes()
            capture_sha=hashlib.sha256(Path(updates['angle_capture']).read_bytes()).hexdigest()
            if json.loads(candidate).get('source_capture_sha256')!=capture_sha:
                raise ValueError('Generated calibration does not bind source capture')
            updates['calibration_sha256']=hashlib.sha256(candidate).hexdigest()
        # State is published only after the full sequence and all checks pass.
        for key,path in updates.items():
            if not key.endswith('_sha256') and not Path(path).is_file():
                raise ValueError('Expected artifact missing: '+path)
        state.update(updates);write_state(statefile,state)
    return 0

if __name__=='__main__':raise SystemExit(main())
