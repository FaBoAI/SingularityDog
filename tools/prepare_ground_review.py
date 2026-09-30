#!/usr/bin/env python3
"""File-only preparation of unreviewed video/log association documents.

Hashes are computed, never inferred. Video synchronization and physical
observations remain unknown until a person reviews the actual recording.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'runtime'))
from singularitydog_hw.ground_trial_output import read_document
from singularitydog_hw.ground_trial_review import physical_review_template


def file_hash(path):
    path=Path(path).expanduser()
    if path.is_symlink() or not path.is_file():raise ValueError('Regular source file required')
    digest=hashlib.sha256()
    before=path.stat()
    with path.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''):digest.update(part)
    after=path.stat()
    if path.is_symlink() or (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(
            after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):
        raise ValueError('Source changed while hashing: '+str(path))
    return digest.hexdigest()


def recheck_sources(sources,hashes):
    for name,path in sources.items():
        if file_hash(path)!=hashes[name]:raise ValueError('Source changed during preparation: '+name)


def association_sources(association,base):
    refs=association.get('references')
    if type(refs) is not dict or set(refs)!={'report','plan','profile','video'}:
        raise ValueError('Complete association source references required')
    sources={};hashes={}
    for name,ref in refs.items():
        if (type(ref) is not dict or set(ref)!={'path','sha256'} or type(ref['path']) is not str
                or not ref['path'] or type(ref['sha256']) is not str or len(ref['sha256'])!=64
                or any(c not in '0123456789abcdef' for c in ref['sha256'])):
            raise ValueError('Invalid association source reference: '+name)
        path=Path(ref['path']).expanduser()
        sources[name]=path if path.is_absolute() else base/path
        hashes[name]=ref['sha256']
    return sources,hashes


def private_new(path):
    path=Path(path).expanduser().absolute()
    if path.exists() or path.is_symlink():raise ValueError('Use a new private output path')
    path=path.resolve()
    if any((parent/'.git').exists() for parent in (path,*path.parents)):
        raise ValueError('Keep associated evidence outside Git')
    return path


def save(path,value):
    with os.fdopen(os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'w') as f:
        json.dump(value,f,ensure_ascii=False,allow_nan=False,indent=2);f.write('\n')


def review_template(association_path,output):
    raw,association=read_document(association_path)
    if association.get('schema')!='singularitydog.ground-video-association.v1':
        raise ValueError('Ground video association required')
    sources,hashes=association_sources(association,Path(association_path).expanduser().absolute().parent)
    recheck_sources(sources,hashes)
    out=private_new(output)
    value=physical_review_template();value['association_sha256']=hashlib.sha256(raw).hexdigest()
    if read_document(association_path)[0]!=raw:raise ValueError('Association changed during preparation')
    recheck_sources(sources,hashes)
    out.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    save(out,value)
    return {'status':'UNREVIEWED','output':str(out),'hardware_opened':False}


def prepare(report_path,plan_path,profile_path,video_path,output):
    sources={name:Path(path).expanduser().absolute() for name,path in
        (('report',report_path),('plan',plan_path),('profile',profile_path),('video',video_path))}
    report_raw,report=read_document(sources['report'])
    plan_raw,plan=read_document(sources['plan']);profile_raw,profile=read_document(sources['profile'])
    if report.get('schema')!='singularitydog.ground-trial-report.v1':
        raise ValueError('Ground trial report required')
    hashes={name:hashlib.sha256(raw).hexdigest() for name,raw in
            (('report',report_raw),('plan',plan_raw),('profile',profile_raw))}
    hashes['video']=file_hash(sources['video'])
    if (report.get('stage_plan_sha256')!=hashes['plan'] or
            report.get('profile_sha256')!=hashes['profile'] or report.get('stage')!=plan.get('stage')):
        raise ValueError('Report, stage plan and profile do not match')
    runtime=report.get('runtime_report') or {}
    if type(runtime) is not dict:raise ValueError('Runtime report must be an object or null')
    # These are only suggested log endpoints, not validated telemetry or video
    # synchronization. Missing records remain unknown, including aborted runs.
    starts=[c.get('begin_ns') for c in (runtime.get('cycles') or [])
            if type(c) is dict and type(c.get('begin_ns')) is int and c['begin_ns']>0]
    ends=[r.get('received_ns') for bus in (runtime.get('stop_reports') or {}).values()
          if type(bus) is dict for r in (bus.get('evidence') or {}).get('records',[])
          if type(r) is dict
          if type(r.get('received_ns')) is int and r['received_ns']>0]
    association={'schema':'singularitydog.ground-video-association.v1',
        'references':{name:{'path':str(path.resolve()),'sha256':hashes[name]} for name,path in sources.items()},
        'sync':{'trial_start_ns':min(starts) if starts else None,'trial_end_ns':max(ends) if ends else None,
                'video_start_s':None,'video_end_s':None,'uncertainty_ms':None},
        'synchronization_method':''}
    physical=physical_review_template()
    # Recheck both JSON and video after assembling the proposed association.
    # In particular, a recording still being written is not stable evidence.
    recheck_sources(sources,hashes)
    out=private_new(output);out.mkdir(parents=True,mode=0o700)
    save(out/'association.json',association)
    save(out/'physical-review-template.json',physical)
    return {'status':'RECORDED_REVIEW_REQUIRED','output':str(out),'hardware_opened':False,
        'dependency_eligible':False,'video_synchronization_verified':False,
        'next':'Fill actual video synchronization, then --association with --review-output to bind an unreviewed template.'}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('report','plan','profile','video','output','association','review-output'):p.add_argument('--'+name)
    a=p.parse_args(argv)
    if a.association or a.review_output:
        if not a.association or not a.review_output or any(getattr(a,k) for k in ('report','plan','profile','video','output')):
            p.error('Use --association and --review-output alone to bind a fresh unreviewed template')
        result=review_template(a.association,a.review_output)
    else:
        if any(not getattr(a,k) for k in ('report','plan','profile','video','output')):
            p.error('Use --report --plan --profile --video --output')
        result=prepare(a.report,a.plan,a.profile,a.video,a.output)
    print(json.dumps(result,ensure_ascii=False));return 0


if __name__=='__main__':raise SystemExit(main())
