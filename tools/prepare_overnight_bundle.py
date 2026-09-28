#!/usr/bin/env python3
"""Assemble a private reproducible next-day kit from saved files, no SSH/devices."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from audit_angle_calibration import history_profile


def build(snapshot_home,output):
    home=Path(snapshot_home).resolve();out=Path(output).resolve()
    if out.exists():raise ValueError('Kit output must be new')
    out.mkdir(parents=True,mode=0o700)
    files=[]
    for tree in ('runtime','tools'):
        for path in sorted((ROOT/tree).rglob('*')):
            if path.is_file() and path.suffix in ('.py','.cpp','.h','.md'):
                if '__pycache__' in path.parts:continue
                target=out/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(path,target);files.append(target)
    inputs=out/'inputs';inputs.mkdir(mode=0o700)
    def save(name,data):
        path=inputs/name;path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n');files.append(path)
    profile,contracts,current,raw,uids=history_profile(home)
    save('angle-profile.json',profile);save('expected-uids.json',uids)
    paths={'calibration':home/'singularitydog-logs/RO-policy-candidate-current-boot-20260927-r1.json',
           'mount':home/'singularitydog-policy-shadow/20260921-r16-mount/imu-mount-candidate.json',
           'saved-policy-report':home/'singularitydog-logs/dual-policy-once-current-boot-20260927-r4/summary.json'}
    for key,source in paths.items():
        target=inputs/(key+'.json');shutil.copyfile(source,target);files.append(target)
    bundle=inputs/'policy';bundle.mkdir()
    source_bundle=home/'singularitydog-policy-shadow/20260921-r1'
    # Same source bundle contract as policy_shadow.load_policy; no environment copying.
    for name in ('model_149.pt','swing_core.py','swing_deployment.py'):
        target=bundle/name;shutil.copyfile(source_bundle/name,target);files.append(target)
    config={'front_port':'/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0',
            'rear_port':'/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0',
            'expected_uids':'inputs/expected-uids.json','angle_profile':'inputs/angle-profile.json',
            'calibration':'inputs/calibration.json','mount':'inputs/mount.json','bundle':'inputs/policy',
            'raw_data_private':True,'calibration_approved_for_runtime':False}
    config_path=out/'kit-config.json';config_path.write_text(json.dumps(config,indent=2)+'\n');files.append(config_path)
    for name in ('overnight-validation-20260927.md','angle-calibration-overnight-20260927.md',
                 'imu-commissioning-overnight-20260927.md','native-feedback-comparison-20260927.md'):
        source=ROOT/'docs'/name
        if source.exists():
            target=out/'docs'/name;target.parent.mkdir(exist_ok=True)
            shutil.copyfile(source,target);files.append(target)
    manifest={'schema':'private-overnight-kit-v1','hardware_accessed':False,
        'files':{str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'note':'Private motor UIDs, model and saved diagnostics. Never commit/publish this kit.'}
    (out/'kit-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for p in out.rglob('*'):
        os.chmod(p,0o700 if p.is_dir() else 0o600)
    return {'output':str(out),'file_count':len(files),'model_and_logs_private':True,'hardware_accessed':False}

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot-home',required=True)
    p.add_argument('--output',required=True);a=p.parse_args(argv)
    print(json.dumps(build(a.snapshot_home,a.output),ensure_ascii=False));return 0

if __name__=='__main__':raise SystemExit(main())
