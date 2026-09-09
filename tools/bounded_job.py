#!/usr/bin/env python3
"""One explicit finite local job, optional detached supervisor, shared advisory lock and receipts."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from artifact_manifest import digest, verify


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def validated_request(path):
    data = json.loads(path.read_text())
    if data.get('schema') != 'singularitydog.job.v1' or not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', data.get('job_id', '')):
        raise ValueError('Invalid job request')
    argv = data.get('argv')
    if not isinstance(argv, list) or not argv or not all(isinstance(x, str) and x for x in argv):
        raise ValueError('Explicit nonempty argv required')
    if not Path(argv[0]).is_absolute() or not Path(data['cwd']).is_absolute() or not Path(data['cwd']).is_dir():
        raise ValueError('Explicit absolute executable and existing cwd required')
    limit = data.get('timeout_s')
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not 0 < limit <= 86400:
        raise ValueError('Finite timeout of at most one day required')
    if not isinstance(data.get('expected_completion'), dict) or not data['expected_completion']:
        raise ValueError('Expected completion fields required')
    completion = Path(data['completion_file'])
    if not completion.is_absolute() or completion.exists():
        raise ValueError('Fresh absolute completion path required')
    manifest = Path(data['manifest_file'])
    if digest(manifest) != data['manifest_sha256']:
        raise ValueError('Manifest file SHA mismatch')
    verify(Path(data['artifact_root']), json.loads(manifest.read_text()))
    return data


def supervise(job_dir, lock_path):
    job_dir = job_dir.resolve()
    request = validated_request(job_dir / 'request.json')
    state = {'job_id': request['job_id'], 'status': 'STARTING', 'started_utc': stamp(),
             'supervisor_pid': os.getpid(), 'request_sha256': digest(job_dir / 'request.json')}
    child = None
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with (job_dir / 'process.log').open('ab') as log:
                child = subprocess.Popen(request['argv'], cwd=request['cwd'], stdin=subprocess.DEVNULL,
                                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                state.update(status='RUNNING', child_pid=child.pid)
                save(job_dir / 'status.json', state)
                try:
                    code = child.wait(timeout=request['timeout_s'])
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
                    raise RuntimeError('Finite job timeout; no retry')
                state['child_exit_code'] = code
                if code != 0:
                    raise RuntimeError('Child exited unsuccessfully')
                completion = Path(request['completion_file'])
                result = json.loads(completion.read_text())
                if any(result.get(k) != v for k, v in request['expected_completion'].items()):
                    raise ValueError('Strict completion mismatch despite child exit code zero')
                state.update(status='COMPLETED', completion_sha256=digest(completion))
    except Exception as error:
        state.update(status='FAILED', error=str(error), automatic_retry=False)
    state['ended_utc'] = stamp()
    save(job_dir / 'status.json', state)
    return 0 if state['status'] == 'COMPLETED' else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['run', 'status', '_supervise'])
    parser.add_argument('--request', type=Path)
    parser.add_argument('--jobs', type=Path)
    parser.add_argument('--job-dir', type=Path)
    parser.add_argument('--lock', type=Path)
    parser.add_argument('--detach', action='store_true')
    args = parser.parse_args()
    if args.mode == 'status':
        if args.job_dir is None:
            parser.error('--job-dir required')
        print((args.job_dir / 'status.json').read_text())
        return
    if args.lock is None:
        parser.error('--lock required')
    if args.mode == '_supervise':
        if args.job_dir is None:
            parser.error('--job-dir required')
        try:
            code = supervise(args.job_dir, args.lock.resolve())
        except Exception as error:
            save(args.job_dir / 'status.json', {'status': 'FAILED', 'error': str(error), 'ended_utc': stamp(), 'automatic_retry': False})
            code = 2
        raise SystemExit(code)
    if args.request is None or args.jobs is None:
        parser.error('--request and --jobs required')
    request = validated_request(args.request)
    args.jobs.mkdir(parents=True, exist_ok=True)
    job_dir = (args.jobs / request['job_id']).resolve()
    job_dir.mkdir()  # Atomic unique job claim, never overwrite an existing ID.
    (job_dir / 'request.json').write_text(json.dumps(request, indent=2) + '\n')
    save(job_dir / 'status.json', {'status': 'CLAIMED', 'job_id': request['job_id'], 'created_utc': stamp()})
    if args.detach:
        with (job_dir / 'supervisor.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '_supervise', '--job-dir', str(job_dir),
                                        '--lock', str(args.lock.resolve())], stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        print(json.dumps({'status': 'SUPERVISOR_DISPATCHED_NOT_COMPLETION', 'pid': process.pid, 'job_dir': str(job_dir)}))
    else:
        raise SystemExit(supervise(job_dir, args.lock.resolve()))


if __name__ == '__main__':
    main()
