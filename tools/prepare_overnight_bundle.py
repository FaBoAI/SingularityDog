#!/usr/bin/env python3
"""Assemble a private reproducible next-day kit from saved files, no SSH/devices."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from audit_angle_calibration import history_profile


def _sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def _regular_source(path):
    path=Path(path).absolute()
    if any(parent.is_symlink() for parent in (path,*path.parents)) or not path.is_file():
        raise ValueError('Regular nonsymlink packaging source required: '+str(path))
    return path


def copy_source(source,target,pins):
    """Copy one stable source; revalidate its identity at final publication too."""
    source=_regular_source(source);before=_sha(source)
    target.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    shutil.copyfile(source,target)
    if _sha(_regular_source(source))!=before or _sha(target)!=before:
        raise ValueError('Source changed during packaging: '+str(source))
    if source in pins and pins[source]!=before:
        raise ValueError('Source changed between packaging reads: '+str(source))
    pins[source]=before


def recheck_sources(pins):
    for source,digest in pins.items():
        if _sha(_regular_source(source))!=digest:
            raise ValueError('Source changed during packaging: '+str(source))


def new_private_output(output):
    out=Path(output).expanduser().absolute()
    if out.exists() or out.is_symlink():raise ValueError('Kit output must be new')
    out=out.resolve()
    if any((parent/'.git').exists() for parent in (out,*out.parents)):
        raise ValueError('Private kit must remain outside Git')
    return out


def publish(stage,out,manifest):
    # The complete marker is published only after every copied member verifies.
    def only_root_manifest(directory,names):
        return {'kit-manifest.json'} if Path(directory)==stage else set()
    shutil.copytree(stage,out,ignore=only_root_manifest)
    for name,digest in manifest['files'].items():
        if _sha(_regular_source(out/name))!=digest:
            raise ValueError('Published kit file differs from staged source: '+name)
    with (out/'kit-manifest.json').open('xb') as stream:
        os.chmod(out/'kit-manifest.json',0o600)
        stream.write((stage/'kit-manifest.json').read_bytes())


def build(snapshot_home,output):
    home=Path(snapshot_home).expanduser().resolve();out=new_private_output(output)
    repo=ROOT.resolve()
    # Missing/symlinked snapshot inputs fail before any final output is created.
    paths={'calibration':home/'singularitydog-logs/RO-policy-candidate-current-boot-20260927-r1.json',
           'mount':home/'singularitydog-policy-shadow/20260921-r16-mount/imu-mount-candidate.json',
           'saved-policy-report':home/'singularitydog-logs/dual-policy-once-current-boot-20260927-r4/summary.json'}
    source_bundle=home/'singularitydog-policy-shadow/20260921-r1'
    model_names=('model_149.pt','swing_core.py','swing_deployment.py')
    for path in (*paths.values(),*(source_bundle/name for name in model_names)):_regular_source(path)
    profile,contracts,current,raw,uids=history_profile(home.resolve())
    out.parent.mkdir(parents=True,mode=0o700,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.overnight-kit-stage-',dir=out.parent) as folder:
        stage=Path(folder)/'kit';stage.mkdir(mode=0o700);pins={}
        for tree in ('runtime','tools'):
            base=repo/tree
            if base.is_symlink() or not base.is_dir():raise ValueError('Regular source tree required: '+tree)
            for path in sorted(base.rglob('*')):
                if '__pycache__' in path.parts:continue
                if path.is_symlink():raise ValueError('Symlink packaging source is excluded: '+str(path))
                if path.is_file() and path.suffix in ('.py','.cpp','.h','.md'):
                    copy_source(path,stage/path.relative_to(repo),pins)
        inputs=stage/'inputs';inputs.mkdir(mode=0o700)
        def save(path,data):
            path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        save(inputs/'angle-profile.json',profile);save(inputs/'expected-uids.json',uids)
        for key,source in paths.items():copy_source(source,inputs/(key+'.json'),pins)
        expected_candidate=profile.get('source_sha256',{}).get('candidate')
        if expected_candidate is not None and _sha(inputs/'calibration.json')!=expected_candidate:
            raise ValueError('Historical profile and copied calibration source differ')
        bundle=inputs/'policy';bundle.mkdir()
        for name in model_names:copy_source(source_bundle/name,bundle/name,pins)
        config={'front_port':'/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0',
                'rear_port':'/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0',
                'expected_uids':'inputs/expected-uids.json','angle_profile':'inputs/angle-profile.json',
                'calibration':'inputs/calibration.json','mount':'inputs/mount.json','bundle':'inputs/policy',
                'raw_data_private':True,'calibration_approved_for_runtime':False}
        save(stage/'kit-config.json',config)
        for name in ('overnight-validation-20260927.md','angle-calibration-overnight-20260927.md',
                     'imu-commissioning-overnight-20260927.md','native-feedback-comparison-20260927.md'):
            source=repo/'docs'/name
            if source.exists() or source.is_symlink():copy_source(source,stage/'docs'/name,pins)
        files={str(p.relative_to(stage)):_sha(p) for p in sorted(stage.rglob('*')) if p.is_file()}
        manifest={'schema':'private-overnight-kit-v1','hardware_accessed':False,'files':files,
            'note':'Private motor UIDs, model and saved diagnostics. Never commit/publish this kit.'}
        save(stage/'kit-manifest.json',manifest)
        for p in (stage,*stage.rglob('*')):os.chmod(p,0o700 if p.is_dir() else 0o600)
        recheck_sources(pins)
        publish(stage,out,manifest)
    return {'output':str(out),'file_count':len(files),'model_and_logs_private':True,'hardware_accessed':False}

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot-home',required=True)
    p.add_argument('--output',required=True);a=p.parse_args(argv)
    print(json.dumps(build(a.snapshot_home,a.output),ensure_ascii=False));return 0

if __name__=='__main__':raise SystemExit(main())
