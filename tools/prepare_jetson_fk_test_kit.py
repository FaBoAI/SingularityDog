#!/usr/bin/env python3
"""Prepare a pinned runtime/tools source kit; default PLAN is read-only.

Use --build only after sources are frozen. The generated stdlib installer also
defaults to PLAN and installs only into a fresh directory. Neither entry opens
devices, loads models/native libraries, deploys remotely, or grants output.
"""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import tarfile


SCHEMA = 'singularitydog.fk-test-source-kit.v1'
MAX_BYTES = 64 * 1024 * 1024
MAX_TOTAL = 256 * 1024 * 1024
_BINARY_SUFFIXES = {'.bin', '.so', '.dylib', '.dll', '.exe', '.o', '.a', '.pt', '.pth', '.onnx',
                    '.zip', '.gz', '.xz', '.bz2', '.png', '.jpg', '.jpeg', '.gif', '.pdf'}

INSTALLER = r'''#!/usr/bin/env python3
"""Pinned source-only installer. PLAN is default; --install needs a fresh directory."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tarfile

MAX_BYTES=64*1024*1024
MAX_TOTAL=256*1024*1024

def need(condition,message):
    if not condition:raise ValueError(message)

def location(value):
    path=Path(value)
    need(path.is_absolute() and '..' not in path.parts and
         not any(p.is_symlink() for p in (path,*path.parents)),
         'Absolute non-symlink location required')
    return path

def read(path,sha=None,limit=MAX_BYTES):
    path=location(path)
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    try:
        before=os.fstat(fd)
        need(stat.S_ISREG(before.st_mode) and before.st_size<=limit,'Bounded regular input required')
        with os.fdopen(fd,'rb',closefd=False) as stream:raw=stream.read(before.st_size+1)
        after=os.fstat(fd)
    finally:os.close(fd)
    need(len(raw)==before.st_size and
         (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns,before.st_ctime_ns)==
         (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns,after.st_ctime_ns),
         'Input changed while reading')
    if sha is not None:
        need(type(sha) is str and re.fullmatch('[0-9a-f]{64}',sha) and
             hashlib.sha256(raw).hexdigest()==sha,'Input SHA256 differs')
    return raw

def member_name(name):
    need(type(name) is str and name and chr(92) not in name and
         all(ord(c)>=32 and ord(c)!=127 for c in name),
         'Canonical source member required')
    path=PurePosixPath(name)
    need(not path.is_absolute() and path.as_posix()==name and '..' not in path.parts and
         len(path.parts)>1 and path.parts[0] in ('runtime','tools'),
         'Source member traversal or scope differs')
    need('__pycache__' not in path.parts and path.suffix not in ('.pyc','.pyo'),
         'Bytecode is not source')
    return path

def strict_json(raw):
    def pairs(items):
        result={}
        for key,value in items:
            need(key not in result,'Duplicate manifest field');result[key]=value
        return result
    def reject(value):raise ValueError('Nonfinite JSON: '+value)
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=reject)

def verify(manifest,manifest_sha,archive,archive_sha,destination):
    dest=location(destination)
    need(not dest.exists() and not dest.is_symlink(),'Fresh destination required; no overwrite')
    need(dest.parent.is_dir() and not any((p/'.git').exists() for p in (dest,*dest.parents)),
         'Destination must be outside Git with an existing parent')
    raw=read(manifest,manifest_sha);data=strict_json(raw)
    need(type(data) is dict and data.get('schema')=='singularitydog.fk-test-source-kit.v1' and
         data.get('status')=='SOURCE_ONLY_NO_MODEL_OR_OUTPUT_APPROVAL' and
         all(data.get(k) is False for k in ('hardware_opened','model_loaded','output_allowed','approved_for_runtime')),
         'Source-only manifest required')
    files=data.get('files');need(type(files) is dict and 0<len(files)<=10000,'Exact source inventory required')
    total=0
    for name,row in files.items():
        member_name(name)
        need(type(row) is dict and set(row)=={'bytes','sha256','mode','tracked'} and
             type(row['bytes']) is int and 0<=row['bytes']<=MAX_BYTES and
             type(row['mode']) is int and row['mode'] in (0o644,0o755) and
             type(row['tracked']) is bool and type(row['sha256']) is str and
             re.fullmatch('[0-9a-f]{64}',row['sha256']),'Invalid source member metadata')
        total+=row['bytes']
    need(total<=MAX_TOTAL,'Source total exceeds bound')
    binding=data.get('archive');need(type(binding) is dict and set(binding)=={'name','bytes','sha256'} and
         binding['name']=='source.tar.gz' and binding['sha256']==archive_sha and
         type(binding['bytes']) is int and 0<binding['bytes']<=MAX_TOTAL,'Archive binding differs')
    archive_raw=read(archive,archive_sha,MAX_TOTAL)
    need(len(archive_raw)==binding['bytes'],'Archive size differs')
    installer=data.get('installer');need(type(installer) is dict and set(installer)=={'name','bytes','sha256'} and
         installer['name']=='install-source-kit.py','Installer binding differs')
    self_raw=read(Path(__file__).absolute(),installer['sha256'])
    need(len(self_raw)==installer['bytes'],'Installer size differs')
    payload={}
    with tarfile.open(fileobj=io.BytesIO(archive_raw),mode='r:gz') as stream:
        for member in stream:
            name=member.name;member_name(name)
            need(name in files and name not in payload and member.isreg() and
                 member.size==files[name]['bytes'] and member.mode==files[name]['mode'],
                 'Unexpected, duplicate, nonregular or changed archive member')
            source=stream.extractfile(member)
            need(source is not None,'Regular source payload required')
            with source:content=source.read(member.size+1)
            need(len(content)==member.size and hashlib.sha256(content).hexdigest()==files[name]['sha256'],
                 'Archive source bytes differ')
            payload[name]=content
    need(set(payload)==set(files),'Incomplete archive inventory')
    # Recheck exact input bytes after archive decoding, before any destination exists.
    need(read(manifest,manifest_sha)==raw and read(archive,archive_sha,MAX_TOTAL)==archive_raw,
         'Source kit changed after verification')
    return dest,data,payload

def install(dest,data,payload):
    location(dest);need(not dest.exists() and not dest.is_symlink(),'Fresh destination required; no overwrite')
    dest.mkdir(mode=0o700)
    try:
        for name in sorted(payload):
            target=dest.joinpath(*member_name(name).parts)
            target.parent.mkdir(mode=0o700,parents=True,exist_ok=True);location(target)
            fd=os.open(target,os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os,'O_NOFOLLOW',0),data['files'][name]['mode'])
            with os.fdopen(fd,'wb') as stream:stream.write(payload[name])
            os.chmod(target,data['files'][name]['mode'])
        actual={p.relative_to(dest).as_posix() for p in dest.rglob('*') if p.is_file()}
        need(actual==set(payload),'Installed source inventory differs')
        for name,row in data['files'].items():read(dest/name,row['sha256'])
    except BaseException:
        shutil.rmtree(dest)
        raise

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--manifest',required=True);p.add_argument('--manifest-sha256',required=True)
    p.add_argument('--archive',required=True);p.add_argument('--archive-sha256',required=True)
    p.add_argument('--destination',required=True);p.add_argument('--install',action='store_true')
    args=p.parse_args(argv)
    dest,data,payload=verify(args.manifest,args.manifest_sha256,args.archive,args.archive_sha256,args.destination)
    if args.install:install(dest,data,payload)
    print(json.dumps({'status':'INSTALLED_SOURCE_ONLY' if args.install else 'PLAN_SOURCE_ONLY',
        'file_count':len(payload),'bytes':sum(map(len,payload.values())),'destination':str(dest),
        'hardware_opened':False,'model_loaded':False,'output_allowed':False,'approved_for_runtime':False},sort_keys=True))
    return 0

if __name__=='__main__':raise SystemExit(main())
'''.encode()


def need(condition, message):
    if not condition:
        raise ValueError(message)


def location(value):
    path = Path(value)
    need(path.is_absolute() and '..' not in path.parts and
         not any(p.is_symlink() for p in (path, *path.parents)), 'Absolute non-symlink path required')
    return path


def read_regular(path):
    path = location(path)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        need(stat.S_ISREG(before.st_mode) and before.st_size <= MAX_BYTES, 'Bounded regular source required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    need(len(raw) == before.st_size and
         (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) ==
         (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns),
         'Source changed while reading')
    return raw, 0o755 if before.st_mode & 0o111 else 0o644


def member_name(name):
    need(type(name) is str and name and chr(92) not in name and
         all(ord(c) >= 32 and ord(c) != 127 for c in name), 'Canonical source path required')
    path = PurePosixPath(name)
    need(not path.is_absolute() and
         path.as_posix() == name and '..' not in path.parts and len(path.parts) > 1 and
         path.parts[0] in ('runtime', 'tools'), 'Canonical runtime/tools source path required')
    return path


def git_files(repository, tracked_only=False):
    args = ['git', 'ls-files', '--cached']
    if not tracked_only:
        args += ['--others', '--exclude-standard']
    result = subprocess.run(args + ['-z', '--', 'runtime', 'tools'], cwd=repository,
                            capture_output=True, check=True)
    return {item.decode('utf-8') for item in result.stdout.split(b'\0') if item}


def inventory(repository):
    repository = location(repository)
    root = subprocess.run(['git', 'rev-parse', '--show-toplevel'], cwd=repository,
                          capture_output=True, text=True, check=True).stdout.strip()
    need(Path(root).resolve() == repository, 'Repository root required')
    names = git_files(repository)
    tracked = git_files(repository, True)
    files, payload, skipped = {}, {}, []
    for name in sorted(names):
        path = member_name(name)
        if '__pycache__' in path.parts or path.suffix in ('.pyc', '.pyo'):
            skipped.append({'path': name, 'reason': 'bytecode'}); continue
        raw, mode = read_regular(repository / name)
        try:
            raw.decode('utf-8'); text = b'\0' not in raw
        except UnicodeDecodeError:
            text = False
        if (not text or path.suffix.lower() in _BINARY_SUFFIXES) and name not in tracked:
            skipped.append({'path': name, 'reason': 'untracked_binary'}); continue
        files[name] = {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(),
                       'mode': mode, 'tracked': name in tracked}
        payload[name] = raw
    need(files and sum(len(raw) for raw in payload.values()) <= MAX_TOTAL, 'Bounded nonempty source inventory required')
    need(git_files(repository) == names and git_files(repository, True) == tracked, 'Git inventory changed')
    return files, payload, skipped


def destination(value):
    path = location(value)
    need(not path.exists() and not path.is_symlink() and path.parent.is_dir(), 'Fresh output with existing parent required')
    need(not any((p / '.git').exists() for p in (path, *path.parents)), 'Private output must be outside Git')
    return path


def prepare(repository, output, *, build=False):
    repository = location(repository); output = destination(output)
    files, payload, skipped = inventory(repository)
    result = {'status': 'SOURCE_ONLY_NO_MODEL_OR_OUTPUT_APPROVAL' if build else 'PLAN_SOURCE_ONLY',
              'file_count': len(files), 'bytes': sum(row['bytes'] for row in files.values()),
              'source_inventory_sha256': hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest(),
              'skipped': skipped, 'hardware_opened': False, 'model_loaded': False,
              'output_allowed': False, 'approved_for_runtime': False}
    if not build:
        return result
    output.mkdir(mode=0o700)
    try:
        kit = output / 'kit'; kit.mkdir(mode=0o700)
        for name, raw in payload.items():
            target = kit / name; target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            with target.open('xb') as stream:stream.write(raw)
            target.chmod(files[name]['mode'])
        archive = output / 'source.tar.gz'
        with archive.open('xb') as stream:
            with gzip.GzipFile(filename='', fileobj=stream, mode='wb', mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode='w', format=tarfile.PAX_FORMAT) as tar:
                    for name, raw in payload.items():
                        member = tarfile.TarInfo(name); member.size = len(raw); member.mode = files[name]['mode']
                        member.mtime = 0; member.uid = member.gid = 0; member.uname = member.gname = ''
                        tar.addfile(member, io.BytesIO(raw))
        installer = output / 'install-source-kit.py'; installer.write_bytes(INSTALLER); installer.chmod(0o755)
        # Freeze only if every source and the complete Git inventory still match.
        after, _, after_skipped = inventory(repository)
        need(after == files and after_skipped == skipped, 'Source inventory changed during packaging')
        for name, row in files.items():
            need(hashlib.sha256(read_regular(kit / name)[0]).hexdigest() == row['sha256'], 'Kit copy differs')
        archive_raw = archive.read_bytes()
        data = {'schema': SCHEMA, **result, 'files': files,
                'archive': {'name': archive.name, 'bytes': len(archive_raw), 'sha256': hashlib.sha256(archive_raw).hexdigest()},
                'installer': {'name': installer.name, 'bytes': len(INSTALLER), 'sha256': hashlib.sha256(INSTALLER).hexdigest()}}
        manifest = output / 'manifest.json'
        raw = (json.dumps(data, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()
        with manifest.open('xb') as stream:stream.write(raw)
        return {**result, 'manifest': str(manifest), 'manifest_sha256': hashlib.sha256(raw).hexdigest(),
                'archive': str(archive), 'archive_sha256': data['archive']['sha256'],
                'installer': str(installer), 'installer_sha256': data['installer']['sha256'], 'kit': str(kit)}
    except BaseException:
        shutil.rmtree(output)
        raise


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--repository', type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--build', action='store_true')
    args = p.parse_args(argv)
    print(json.dumps(prepare(args.repository, args.output, build=args.build), sort_keys=True))
    return 0


if __name__ == '__main__':raise SystemExit(main())
