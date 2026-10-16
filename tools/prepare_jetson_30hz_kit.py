#!/usr/bin/env python3
"""Freeze robot sources and the pinned 30 Hz candidate; PLAN is file-only.

--build copies into a fresh private directory. It neither contacts Jetson nor
opens hardware, compiles libraries, runs Torch, or grants motor output. The
generated verifier/installer has the same default; CPU loading is explicit.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
import shutil
import stat
import sys


SCHEMA = 'singularitydog.robot-30hz-offline-kit.v1'
MAX_MODEL_TOTAL = 64 * 1024 * 1024
REFERENCE_FILES = {
    'references/AGENTS.md': 'AGENTS.md',
    'references/jetson-best20-settings.md': 'docs/jetson-best20-settings.md',
    'references/jetson-best20-settings.json': 'evidence/jetson-best20-settings.json',
}
NO_GRANTS = dict(hardware_opened=False, model_loaded=False, output_allowed=False,
                 approved_for_runtime=False, walking_acceptance=False,
                 current_power_or_pose_confirmed=False, native_library_built=False,
                 hardware_measurement_completed=False)


VERIFIER = r'''#!/usr/bin/env python3
"""Verify a pinned offline kit. --install and --cpu-check are separate opt-ins."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import sys

SCHEMA='singularitydog.robot-30hz-offline-kit.v1'
MAX_FILE=64*1024*1024
MAX_TOTAL=512*1024*1024
FLAGS=('hardware_opened','model_loaded','output_allowed','approved_for_runtime',
       'walking_acceptance','current_power_or_pose_confirmed','native_library_built',
       'hardware_measurement_completed')

def need(value,message):
    if not value:raise ValueError(message)

def location(value):
    path=Path(value)
    need(path.is_absolute() and '..' not in path.parts and
         not any(p.is_symlink() for p in (path,*path.parents)),
         'Absolute non-symlink location required')
    return path

def directory_signature(path):
    try:entry=Path(path).lstat()
    except OSError:return None
    return (entry.st_dev,entry.st_ino) if stat.S_ISDIR(entry.st_mode) else None

def read(path,sha=None):
    path=location(path)
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    try:
        before=os.fstat(fd)
        need(stat.S_ISREG(before.st_mode) and 0<=before.st_size<=MAX_FILE,
             'Bounded regular kit file required')
        with os.fdopen(fd,'rb',closefd=False) as stream:raw=stream.read(before.st_size+1)
        after=os.fstat(fd)
    finally:os.close(fd)
    need(len(raw)==before.st_size and
         (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns,before.st_ctime_ns)==
         (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns,after.st_ctime_ns),
         'Kit file changed while reading')
    if sha is not None:
        need(type(sha) is str and re.fullmatch('[0-9a-f]{64}',sha) and
             hashlib.sha256(raw).hexdigest()==sha,'Kit SHA256 differs')
    return raw

def member(name):
    need(type(name) is str and name and chr(92) not in name and
         all(ord(c)>=32 and ord(c)!=127 for c in name),'Canonical kit member required')
    p=PurePosixPath(name)
    need(not p.is_absolute() and p.as_posix()==name and '..' not in p.parts,
         'Kit member traversal or alias rejected')
    return p

def strict_json(raw):
    def pairs(items):
        result={}
        for key,value in items:
            need(key not in result,'Duplicate manifest key');result[key]=value
        return result
    def bad(value):raise ValueError('Nonfinite manifest number: '+value)
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=bad)

def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    need(spec is not None and spec.loader is not None,'Module source required')
    result=importlib.util.module_from_spec(spec);sys.modules[name]=result
    spec.loader.exec_module(result)
    return result

def verify(path,digest,destination):
    manifest=location(path);root=manifest.parent
    need(manifest.name=='robot30-manifest.json','Canonical robot manifest required')
    raw=read(manifest,digest);data=strict_json(raw)
    need(type(data) is dict and data.get('schema')==SCHEMA and
         data.get('status')=='FROZEN_OFFLINE_CANDIDATE_NO_OUTPUT_APPROVAL' and
         all(data.get(k) is False for k in FLAGS),'Unapproved offline kit required')
    need(data.get('legacy_50hz_backend_selected') is False and
         data.get('previous_50hz_qualification_transferred') is False,
         'Old50Hz backend or qualification is not selected')
    files=data.get('files')
    need(type(files) is dict and 0<len(files)<=20000,'Exact kit inventory required')
    total=0
    for name,row in files.items():
        member(name)
        need(name!='robot30-manifest.json' and type(row) is dict and
             set(row)=={'bytes','sha256','mode'} and type(row['bytes']) is int and
             0<=row['bytes']<=MAX_FILE and type(row['mode']) is int and
             row['mode'] in (0o644,0o755),'Invalid kit metadata')
        content=read(root/name,row['sha256'])
        need(len(content)==row['bytes'],'Kit file size differs');total+=len(content)
    need(total<=MAX_TOTAL,'Kit total exceeds bound')
    actual=set()
    for item in root.rglob('*'):
        location(item)
        need(item.is_dir() or item.is_file(),'Nonregular kit entry rejected')
        if item.is_file():actual.add(item.relative_to(root).as_posix())
    need(actual==set(files)|{'robot30-manifest.json'},'Kit inventory differs')
    need(Path(__file__).absolute()==root/'verify-robot30-kit.py',
         'Verifier must run from this pinned kit')
    source=module(root/'install-source-kit.py','_source_kit_installer')
    dest,source_data,payload=source.verify(root/'manifest.json',files['manifest.json']['sha256'],
        root/'source.tar.gz',files['source.tar.gz']['sha256'],destination)
    copied={name[4:] for name in files if name.startswith('kit/')}
    need(copied==set(source_data['files']),'Source tree differs from source archive')
    for name,row in source_data['files'].items():
        need(files['kit/'+name]=={key:row[key] for key in ('bytes','sha256','mode')},
             'Source tree binding differs')
    # Only import the two pinned file-only modules; PLAN does not import Torch.
    clock=module(root/'kit/runtime/singularitydog_hw/policy_clock.py','_robot30_clock')
    model=module(root/'kit/runtime/singularitydog_hw/policy_model_30hz.py','_robot30_model')
    model_plan=model.plan(model.bundle_artifacts(root/'model-bundle'))
    need(data.get('control_clock')==clock.control_clock_payload(30),
         'Exact30Hz clock differs')
    releases={name:{'path':'model-bundle/'+relative,'sha256':model.RELEASE_SHA256[name]}
              for name,relative in model.ARTIFACT_FILES.items()}
    need(data.get('release_artifacts')==releases,'Pinned30Hz release differs')
    need(data.get('evaluation')=={'completed_cases':5,'total_cases':7,
         'unresolved_cases':['left','stop'],'walking_acceptance':False},
         'Candidate evaluation limitations differ')
    need(data.get('reference_documents')==['references/AGENTS.md',
         'references/jetson-best20-settings.json','references/jetson-best20-settings.md'],
         'Historical timing reference inventory differs')
    need(data.get('critical_input_to_all_host_write_budget_ns')==20000000 and
         data.get('type1_reply_cycle_budget_ns')==33333334 and
         data.get('proxy_exchange_budget_ns')==33333334 and
         data.get('proxy_input_to_all_host_write_budget_ns')==20000000 and
         type(data.get('diagnostic_proxy_separate_write_deadline_abi_required')) is int and
         data.get('diagnostic_proxy_separate_write_deadline_abi_required')==1,
         'Critical write/proxy/reply budgets differ')
    need('torch' not in sys.modules,'Default verification must not import Torch')
    need(read(manifest,digest)==raw,'Manifest changed after verification')
    return root,dest,data,source,source_data,payload,model,model_plan

def install(root,dest,data,source,source_data,payload):
    source.install(dest,source_data,payload)
    owned=directory_signature(dest)
    need(owned is not None,'Created source destination must remain a directory')
    try:
        for name,row in data['files'].items():
            if not name.startswith(('model-bundle/','references/')):continue
            target=dest.joinpath(*member(name).parts)
            target.parent.mkdir(mode=0o700,parents=True,exist_ok=True);location(target)
            content=read(root/name,row['sha256'])
            fd=os.open(target,os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os,'O_NOFOLLOW',0),row['mode'])
            with os.fdopen(fd,'wb') as stream:stream.write(content)
            os.chmod(target,row['mode']);read(target,row['sha256'])
        expected=set(source_data['files'])|{n for n in data['files'] if n.startswith(('model-bundle/','references/'))}
        actual={p.relative_to(dest).as_posix() for p in dest.rglob('*') if p.is_file()}
        need(actual==expected,'Installed source/model inventory differs')
        need(directory_signature(dest)==owned,'Installed destination replaced')
    except BaseException:
        if directory_signature(dest)==owned:shutil.rmtree(dest)
        raise

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--manifest',required=True);parser.add_argument('--manifest-sha256',required=True)
    parser.add_argument('--destination',required=True)
    parser.add_argument('--install',action='store_true')
    parser.add_argument('--cpu-check',action='store_true')
    args=parser.parse_args(argv)
    root,dest,data,source,source_data,payload,model,model_plan=verify(
        args.manifest,args.manifest_sha256,args.destination)
    proof=None
    if args.cpu_check:
        # Deliberately no device, CAN transport, performance wrapper or driver.
        _,proof=model.load(model.bundle_artifacts(root/'model-bundle'),execute_cpu=True)
    if args.install:install(root,dest,data,source,source_data,payload)
    result={'status':'INSTALLED_SOURCE_AND30HZ_CANDIDATE' if args.install else 'PLAN_VERIFIED_OFFLINE30HZ_KIT',
        'source_file_count':len(payload),'model_file_count':sum(n.startswith('model-bundle/') for n in data['files']),
        'reference_file_count':len(data['reference_documents']),
        'policy_hz':30,'critical_input_to_all_host_write_budget_ns':20000000,
        'type1_reply_cycle_budget_ns':33333334,'destination':str(dest),
        'cpu_model_check':proof,'model_loaded':bool(args.cpu_check),
        'hardware_opened':False,'output_allowed':False,'approved_for_runtime':False,
        'walking_acceptance':False,'native_library_built':False,
        'current_power_or_pose_confirmed':False,'hardware_measurement_completed':False}
    print(json.dumps(result,allow_nan=False,sort_keys=True));return 0

if __name__=='__main__':raise SystemExit(main())
'''.encode()


README = '''# 30Hzロボット側配置キット

既定はファイル照合だけです。Jetsonへの接続、CAN通信、モーター有効化は行いません。
新モデルは30Hz（1/30秒）用の候補です。入力取得から全軸のhost書込みまでの
20ms期限を保ち、実Type1と無出力STOP代理の返信を含む周期には
33,333,334nsの上限を使います。STOP代理でも全host書込みはC++で20msを
検査する新しい分離期限ABIを必要とします。旧50Hzの期限は変更しません。
host書込み時刻はCAN上の送信完了時刻ではありません。

まず配布時に受け取ったrobot30-manifest.jsonのSHA256を別途照合してください。
次の環境変数は、その端末上の実際の絶対パス・配布SHAに設定します。

```sh
: "${TASK_KIT:?Set the absolute extracted kit directory}"
: "${TASK_MANIFEST_SHA:?Set the separately supplied manifest SHA256}"
: "${TASK_INSTALL:?Set a fresh absolute directory outside Git}"
python3 -I -B "$TASK_KIT/verify-robot30-kit.py" \\
  --manifest "$TASK_KIT/robot30-manifest.json" --manifest-sha256 "$TASK_MANIFEST_SHA" \\
  --destination "$TASK_INSTALL"
```

同じ引数へ--installを追加すると、未使用の場所へruntime・tools・model-bundle・referencesを
配置します。既存の配置を上書きしません。--cpu-checkはTorchが入ったCPU環境での
モデル読込み・数値・状態・時計の検算を明示的に行います。どちらも実機試験ではありません。

Jetsonでの前提はPython、CPU推論対応Torch、C++17コンパイラ、現在の12軸校正資料と
4系統USB2CANの現物対応です。Mac用の共有ライブラリは同梱・選択しません。
ARM用ライブラリはJetson上で次のファイル専用PLANを確認してから、各コマンドへ
--buildを追加して新しい保存先に作成します。ビルド自体はデバイスを開きません。

```sh
: "${TASK_NATIVE_DIAGNOSTIC:?Set a fresh absolute native build directory}"
: "${TASK_NATIVE_TYPE1:?Set another fresh absolute native build directory}"
: "${TASK_NATIVE_CURRENT_GUARD:?Set a third fresh absolute native build directory}"
PYTHONPATH="$TASK_INSTALL/runtime" python3 -B -m experiments.four_bus_diagnostic.build \\
  --output "$TASK_NATIVE_DIAGNOSTIC"
PYTHONPATH="$TASK_INSTALL/runtime" python3 -B -m experiments.four_bus_type1.build \\
  --output "$TASK_NATIVE_TYPE1"
PYTHONPATH="$TASK_INSTALL/runtime" python3 -B -m experiments.four_bus_diagnostic.build_current_guard \\
  --output "$TASK_NATIVE_CURRENT_GUARD"
```

3つ目は現在の起動・電源・入力を監視するcurrent guardです。診断・Type1・current guardの
3つのARM build-recordと共有ライブラリを、それぞれ実行profileへSHAで結び付けます。

周期中の4つの通信所有者をC++へ統合する実験版を選ぶ場合は、通常Type1とは別の
保存先で次のPLANを確認し、--buildを追加してJetson上でビルドします。

```sh
: "${TASK_NATIVE_GROUP:?Set a fresh absolute composite native build directory}"
PYTHONPATH="$TASK_INSTALL/runtime" python3 -B -m experiments.four_bus_native_group.build \\
  --output "$TASK_NATIVE_GROUP"
```

この複合ライブラリは4つの元ソースと専用ABIを持ちます。契約作成時に
--native-group-cycleを明示し、foregroundの--native-group-base-source-sha256に、
含めた元subset-activeソースのSHAも渡してください。違うライブラリから作った
Sessionを混ぜられません。旧成功をこの候補の資格へ移さず、現在のソースから
前段診断を取り直します。CPU比較用experiments.four_bus_native_group.benchmarkは
通常PLANで、--execute-cpuを明示した場合もAF_UNIXの仮想通信のみです。
この比較はモデル推論・実CAN・実出力の20ms確認ではありません。

30Hzは同梱の新TorchScriptモデルと時計を明示選択します。旧50Hzのnative・scalar・FK・
checkedモデルは30Hzへ流用しません。旧50Hzの診断・実出力資格も転用しません。
基準の900µs・window3、CPU配置・一時性能設定・10+10回の事前推論とreset・GC延期・
単一スレッド・500µs release spinを参照し、実施時の設定と復元を新原票へ記録します。
毎回references/jetson-best20-settings.mdと同JSONを読みます。これらは旧50Hzの
有限実測の原設定であり、新30Hz・4系統通信の実測資格へ読み替えません。

Jetson起動後、現在の起動・40V電源区間・UID・校正・角度・電圧・箱支持と可動余裕を
新たな記録へ結び、無出力5→501周期、ゼロゲイン通信断・全12軸STOP、箱支持2→10秒の
順に別々の原票を取ります。キットはその実行条件や出力許可を作りません。
入力鮮度20ms、欠落・曖昧返信、関節範囲、STOP確認、200ms通信断保護は保持します。

モデルの7ケースの有限シミュレーションは5ケースが完了し、左移動と停止の2ケースは
途中終了しました。原因と実機での荷重支持・自立・歩行は未確定です。
補助入力hも実測荷重の推定器ではありません。歩行合格を意味するモデルではありません。
'''.encode()


def _module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result  # dataclass resolves its defining module.
    spec.loader.exec_module(result)
    return result


def dependencies():
    root = Path(__file__).resolve().parents[1]
    source = _module(root / 'tools/prepare_jetson_fk_test_kit.py', '_robot30_source_kit')
    ordinary_inventory = source.inventory
    def source_only_inventory(repository):
        files, payload, skipped = ordinary_inventory(repository)
        skipped = list(skipped)
        for name in list(files):
            raw = payload[name]
            try:
                raw.decode('utf-8'); is_text = b'\0' not in raw
            except UnicodeDecodeError:
                is_text = False
            if not is_text or PurePosixPath(name).suffix.lower() in source._BINARY_SUFFIXES:
                files.pop(name); payload.pop(name)
                skipped.append(dict(path=name, reason='30hz_source_only_target_native_rebuild'))
        source.need(files, 'Nonempty text source inventory required')
        return files, payload, skipped
    # This isolated helper instance retains all installer/atomic checks while
    # also excluding tracked historical native/model binaries from this kit.
    source.inventory = source_only_inventory
    return (source,
            _module(root / 'runtime/singularitydog_hw/policy_model_30hz.py', '_robot30_model_files'),
            _module(root / 'runtime/singularitydog_hw/policy_clock.py', '_robot30_model_clock'))


def model_inventory(source, bundle):
    bundle = source.location(bundle)
    source.need(bundle.is_dir(), 'Model distribution must be a regular directory')
    files, payload = {}, {}
    for item in sorted(bundle.rglob('*')):
        source.location(item)
        source.need(item.is_dir() or item.is_file(), 'Regular model distribution entries required')
        if not item.is_file():
            continue
        name = item.relative_to(bundle).as_posix()
        path = PurePosixPath(name)
        source.need(path.as_posix() == name and '..' not in path.parts and chr(92) not in name and
                    all(ord(c) >= 32 and ord(c) != 127 for c in name), 'Canonical model member required')
        raw, mode = source.read_regular(item)
        files[name] = dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(), mode=mode)
        payload[name] = raw
    source.need(files and sum(map(len, payload.values())) <= MAX_MODEL_TOTAL, 'Bounded model distribution required')
    return files, payload


def file_inventory(source, root):
    files = {}
    for path in sorted(root.rglob('*')):
        if path.is_file():
            raw, mode = source.read_regular(path)
            files[path.relative_to(root).as_posix()] = dict(
                bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(), mode=mode)
    return files


def directory_signature(path):
    """No-follow identity; a replacement or inaccessible path is not ours."""
    try:
        entry = Path(path).lstat()
    except OSError:
        return None
    return (entry.st_dev, entry.st_ino) if stat.S_ISDIR(entry.st_mode) else None


def prepare(repository, bundle, output, *, build=False):
    source, model, clock = dependencies()
    repository, bundle, output = (source.location(p) for p in (repository, bundle, output))
    source.destination(output)
    model_plan = model.plan(model.bundle_artifacts(bundle))
    model_files, model_payload = model_inventory(source, bundle)
    references = {name:source.read_regular(repository / relative)
                  for name, relative in REFERENCE_FILES.items()}
    source_result = source.prepare(repository, output, build=False)
    source_files, _, _ = source.inventory(repository)
    source.need({'runtime/singularitydog_hw/policy_clock.py',
                 'runtime/singularitydog_hw/policy_model_30hz.py'} <= set(source_files),
                '30Hz clock and model loader must be present in robot sources')
    result = dict(schema=SCHEMA, status='PLAN_OFFLINE30HZ_KIT', **NO_GRANTS,
                  policy_hz=30, control_clock=clock.control_clock_payload(30),
                  critical_input_to_all_host_write_budget_ns=20_000_000,
                  type1_reply_cycle_budget_ns=33_333_334, proxy_exchange_budget_ns=33_333_334,
                  proxy_input_to_all_host_write_budget_ns=20_000_000,
                  diagnostic_proxy_separate_write_deadline_abi_required=1,
                  source_file_count=source_result['file_count'],
                  source_inventory_sha256=source_result['source_inventory_sha256'],
                  model_file_count=len(model_files), model_bytes=sum(map(len, model_payload.values())),
                  reference_documents=sorted(REFERENCE_FILES),
                  release_artifacts={name:dict(path='model-bundle/' + relative, sha256=model.RELEASE_SHA256[name])
                                     for name, relative in model.ARTIFACT_FILES.items()},
                  evaluation=dict(completed_cases=5, total_cases=7, unresolved_cases=['left', 'stop'],
                                  walking_acceptance=False),
                  native_architecture='build_cpp_on_target_jetson_no_host_binary_reuse',
                  legacy_50hz_backend_selected=False, previous_50hz_qualification_transferred=False)
    if not build:
        return result
    # The source builder has its own fresh check and owns its own cleanup.
    # If another process creates output before that check, its directory must
    # never enter this composed kit's cleanup path.
    source.prepare(repository, output, build=True)
    owned = directory_signature(output)
    source.need(owned is not None, 'Created source output must remain a directory')
    try:
        for name, (raw, mode) in references.items():
            target = output / name; target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            target.write_bytes(raw); target.chmod(mode)
        for name, raw in model_payload.items():
            target = output / 'model-bundle' / name
            target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(raw)
            target.chmod(model_files[name]['mode'])
        verifier = output / 'verify-robot30-kit.py'
        verifier.write_bytes(VERIFIER); verifier.chmod(0o755)
        (output / 'README.md').write_bytes(README)
        source.need(model_inventory(source, bundle)[0] == model_files,
                    'Model distribution changed during packaging')
        source.need(model.plan(model.bundle_artifacts(bundle)) == model_plan,
                    'Model release changed during packaging')
        source.need(model.plan(model.bundle_artifacts(output / 'model-bundle'))['policy_hz'] == 30,
                    'Copied model distribution failed verification')
        source.need(all(source.read_regular(repository / REFERENCE_FILES[name]) == values
                        for name, values in references.items()),
                    'Timing reference changed during packaging')
        # The model copy can take time. Freeze only after a final source recheck.
        current_source, _, _ = source.inventory(repository)
        current_sha = hashlib.sha256(json.dumps(current_source, sort_keys=True).encode()).hexdigest()
        source.need(current_sha == result['source_inventory_sha256'],
                    'Source changed during combined packaging')
        data = {**result, 'status':'FROZEN_OFFLINE_CANDIDATE_NO_OUTPUT_APPROVAL',
                'files':file_inventory(source, output)}
        manifest = output / 'robot30-manifest.json'
        raw = (json.dumps(data, allow_nan=False, sort_keys=True, indent=2) + '\n').encode()
        manifest.write_bytes(raw)
        source.need(directory_signature(output) == owned, 'Created source output replaced')
        return {**result, 'status':data['status'], 'output':str(output),
                'manifest':str(manifest), 'manifest_sha256':hashlib.sha256(raw).hexdigest(),
                'verifier':str(verifier), 'source_archive':str(output / 'source.tar.gz')}
    except BaseException:
        if directory_signature(output) == owned:
            shutil.rmtree(output)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--repository', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--model-bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args(argv)
    print(json.dumps(prepare(args.repository, args.model_bundle, args.output, build=args.build),
                     allow_nan=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
