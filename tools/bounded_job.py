#!/usr/bin/env python3
"""One explicit finite local job, optional detached supervisor, shared advisory lock and receipts."""
import argparse
from contextlib import contextmanager
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from artifact_manifest import digest, parse_json, read_json, verify


class JobInterrupted(Exception):
    def __init__(self, signum):
        self.signum = signum
        super().__init__('Job interrupted by signal '+str(signum))


@contextmanager
def interrupt_signals():
    """Record cleanup-time signals without interrupting group termination."""
    previous = {}; state = {'caught_signal': None, 'cleaning_up': False, 'spawning': False}
    def interrupt(signum, frame):
        if state['caught_signal'] is None:
            state['caught_signal'] = signum
            if not state['cleaning_up'] and not state['spawning']:
                raise JobInterrupted(signum)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        yield state
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def _group_live_members(pgid):
    """POSIX ps works on macOS and Linux; zombies are not running work."""
    result = subprocess.run(['/bin/ps', '-A', '-o', 'pid=,pgid=,stat='],
                            capture_output=True, text=True, check=True, timeout=1)
    live = []
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) != 3 or not fields[0].isdigit() or not fields[1].isdigit():
            raise RuntimeError('Cannot parse process-group monitor output')
        if int(fields[1]) == pgid and not fields[2].startswith('Z'):
            live.append(int(fields[0]))
    return live


def _child_exit_status(child):
    # Do not reap the session/group leader yet: its reserved PID prevents a
    # later killpg from addressing an unrelated group after PID reuse.
    if child.returncode is not None:
        raise RuntimeError('Child was reaped before group cleanup; group identity is unverified')
    info = os.waitid(os.P_PID, child.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
    if info is None:
        return None
    return info.si_status if info.si_code == os.CLD_EXITED else -info.si_status


def wait_child_unreaped(child, timeout):
    deadline = time.monotonic()+timeout
    while True:
        code = _child_exit_status(child)
        if code is not None:
            return code
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise subprocess.TimeoutExpired(child.args, timeout)
        time.sleep(min(.02, remaining))


def _wait_group_empty(pgid, timeout=5):
    deadline = time.monotonic()+timeout
    while _group_live_members(pgid):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(.02, remaining))
    return True


def require_group_monitor():
    if not all(hasattr(os, name) for name in ('waitid', 'P_PID', 'WEXITED', 'WNOHANG', 'WNOWAIT', 'CLD_EXITED')):
        raise RuntimeError('Safe job cleanup requires POSIX waitid with WNOWAIT')
    if signal.getsignal(signal.SIGCHLD) != signal.SIG_DFL:
        raise RuntimeError('Safe job cleanup requires the default non-reaping SIGCHLD handler')
    _group_live_members(os.getpgrp())  # Permission/availability check before Popen.


def terminate_child(child):
    """Finish this isolated group before reaping its leader or releasing lock.

    Process cleanup is not a motor STOP acknowledgement. Hardware tools retain
    their own STOP/fault handling and device watchdog. Group-monitor failure
    is an explicit failed receipt, never a completed job.
    """
    if child is None:
        return
    _child_exit_status(child)  # Prove we still own an unreaped leader PID.
    try:
        if _group_live_members(child.pid):
            try:os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:pass
            if not _wait_group_empty(child.pid):
                try:os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:pass
                if not _wait_group_empty(child.pid):
                    raise RuntimeError('Child process group still live after SIGKILL')
    except BaseException:
        # The group leader is still reserved, so escalation remains confined to
        # this job even when ps itself fails during cleanup.
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        raise
    finally:
        child.wait(timeout=5)  # Reap only after all group signals are finished.


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def validated_request(path):
    data = read_json(path)
    if (type(data) is not dict or data.get('schema') != 'singularitydog.job.v1'
            or type(data.get('job_id')) is not str
            or not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', data['job_id'])):
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
    manifest_raw = manifest.read_bytes()
    if hashlib.sha256(manifest_raw).hexdigest() != data['manifest_sha256']:
        raise ValueError('Manifest file SHA mismatch')
    verify(Path(data['artifact_root']), parse_json(manifest_raw))
    return data


def supervise(job_dir, lock_path):
    job_dir = job_dir.resolve()
    request = validated_request(job_dir / 'request.json')
    state = {'job_id': request['job_id'], 'status': 'STARTING', 'started_utc': stamp(),
             'supervisor_pid': os.getpid(), 'request_sha256': digest(job_dir / 'request.json')}
    child = None
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    exit_code = 2
    try:
        with interrupt_signals() as interrupts, lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            require_group_monitor()
            try:
                with (job_dir / 'process.log').open('ab') as log:
                    # Defer asynchronous exceptions until the new process has
                    # been assigned; otherwise cleanup could see child=None.
                    interrupts['spawning'] = True
                    try:
                        child = subprocess.Popen(request['argv'], cwd=request['cwd'], stdin=subprocess.DEVNULL,
                                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    finally:interrupts['spawning'] = False
                    if interrupts['caught_signal'] is not None:
                        raise JobInterrupted(interrupts['caught_signal'])
                    state.update(status='RUNNING', child_pid=child.pid)
                    save(job_dir / 'status.json', state)
                    try:
                        code = wait_child_unreaped(child, request['timeout_s'])
                    except subprocess.TimeoutExpired:
                        raise RuntimeError('Finite job timeout; no retry')
                    state['child_exit_code'] = code
                    if code != 0:
                        raise RuntimeError('Child exited unsuccessfully')
                    completion = Path(request['completion_file'])
                    completion_raw = completion.read_bytes()
                    result = parse_json(completion_raw)
                    if (type(result) is not dict or any(k not in result or
                            json.dumps(result[k], sort_keys=True, allow_nan=False) !=
                            json.dumps(v, sort_keys=True, allow_nan=False)
                            for k, v in request['expected_completion'].items())):
                        raise ValueError('Strict completion mismatch despite child exit code zero')
                    state.update(status='COMPLETED',
                                 completion_sha256=hashlib.sha256(completion_raw).hexdigest())
            finally:
                # Any failure after Popen, including receipt write failure or
                # Ctrl+C, must finish child cleanup while we still own the lock.
                interrupts['cleaning_up'] = True
                terminate_child(child)
                if interrupts['caught_signal'] is not None:
                    raise JobInterrupted(interrupts['caught_signal'])
    except JobInterrupted as error:
        state.update(status='INTERRUPTED', caught_signal=error.signum,
                     error=str(error), automatic_retry=False)
        exit_code = 128+error.signum
    except KeyboardInterrupt:
        state.update(status='INTERRUPTED', caught_signal=signal.SIGINT,
                     error='KeyboardInterrupt', automatic_retry=False)
        exit_code = 130
    except Exception as error:
        state.update(status='FAILED', error=str(error), automatic_retry=False)
    state['ended_utc'] = stamp()
    save(job_dir / 'status.json', state)
    return 0 if state['status'] == 'COMPLETED' else exit_code


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
