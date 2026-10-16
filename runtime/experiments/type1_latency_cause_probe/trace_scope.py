"""Reversible Linux tracing of a sealed, CPU/AF_UNIX-only cause probe.

This is separate from the robot runtime. It never opens a motor device and
never creates a qualification. The child must be this experiment's probe.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import pwd
import re
import signal
import struct
import subprocess
import sys
import time
import traceback
import uuid


MODULE = 'experiments.type1_latency_cause_probe.probe'
TRACE_ROOT = Path('/sys/kernel/tracing')
SCHEDSTATS = Path('/proc/sys/kernel/sched_schedstats')
ARM64_PAC_EXEC_SHA256 = 'a7d56a8a764faf7bbf5c164055a48fd072be52287bdeb523a9e07b2042f4e7e1'


class TraceInterrupted(RuntimeError):
    """Catchable termination, so the reversible scope still owns cleanup."""


def stop_child(child, *, grace_s=5):
    """Reap an unfinished private process group even if it ignores SIGTERM."""
    if child is None or child.poll() is not None:
        return
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=grace_s)


def private_json(path, document):
    """Never follow a diagnostic child's replacement symlink as root."""
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    raw = (json.dumps(document, indent=2, allow_nan=False) + '\n').encode()
    with os.fdopen(os.open(path, flags, 0o600), 'wb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def elf_offset(path, address):
    """Translate an ELF64 little-endian symbol virtual address to file offset."""
    with Path(path).open('rb') as source:
        header = source.read(64)
        if header[:6] != b'\x7fELF\x02\x01':
            raise ValueError('ELF64 little-endian executable required')
        phoff = struct.unpack_from('<Q', header, 32)[0]
        phsize, phcount = struct.unpack_from('<HH', header, 54)
        if phsize < 56 or phcount > 4096:
            raise ValueError('Invalid ELF program headers')
        source.seek(phoff)
        for _ in range(phcount):
            row = source.read(phsize)
            kind, flags, offset, vaddr, _, filesz, _, _ = struct.unpack_from('<IIQQQQQQ', row)
            if kind == 1 and flags & 1 and vaddr <= address < vaddr + filesz:
                return offset + address - vaddr
    raise ValueError('Symbol not in a file-backed executable PT_LOAD segment')


def symbol_offset(path, name='PyEval_RestoreThread'):
    result = subprocess.run(['nm', '-D', str(path)], check=True, capture_output=True, text=True)
    values = [int(line.split()[0], 16) for line in result.stdout.splitlines()
              if len(line.split()) == 3 and line.split()[2] == name]
    if len(values) != 1:
        raise ValueError('One exported PyEval_RestoreThread symbol required')
    return elf_offset(path, values[0])


def probe_points(path, offset):
    """Select only reviewed instruction boundaries; never guess past ARM PAC.

    An ARM64 uretprobe after PACIASP can replace a signed LR with an unsigned
    trampoline. Use two ordinary entry probes without touching LR instead.
    This reviewed executable's exit is before its authenticated tail branch,
    so the measured path excludes the tail-called thread-state swap/return.
    """
    with Path(path).open('rb') as stream:
        header = stream.read(64)
        machine = struct.unpack_from('<H', header, 18)[0]
        stream.seek(offset)
        raw = stream.read(48)
    if machine != 183:  # ELF EM_AARCH64; other ELF platforms retain full path.
        return {'kind': 'restore_thread_entry_to_function_return',
                'semantics': 'PyEval_RestoreThread entry through actual return; includes thread-state tail',
                'lr_hijack_used': True,
                'points': [{'kind': 'p', 'name': 'restore_entry', 'file_offset': offset},
                           {'kind': 'r', 'name': 'restore_return', 'file_offset': offset}]}
    if sha256(path) != ARM64_PAC_EXEC_SHA256:
        raise ValueError('ARM64 PAC executable not independently reviewed; no guessed probe offsets')
    words = struct.unpack('<12I', raw)
    fixed = {0: 0xd503233f, 1: 0xa9be7bfd, 2: 0x910003fd, 3: 0xf9000bf3,
             4: 0xb4000100, 5: 0xaa0003f3, 7: 0xaa1303e0, 8: 0xf9400bf3,
             9: 0xa8c27bfd, 10: 0xd50323bf}
    if (any(words[i] != value for i, value in fixed.items()) or
            words[6] & 0xfc000000 != 0x94000000 or words[11] & 0xfc000000 != 0x14000000):
        raise ValueError('Reviewed PAC/frame/take_gil/authenticated-tail instruction layout differs')
    return {'kind': 'arm64_pac_safe_gil_acquisition_path',
            'semantics': 'After PACIASP before frame save, through take_gil, to authenticated pre-tail branch; excludes thread-state swap/return tail',
            'lr_hijack_used': False, 'reviewed_executable_sha256': ARM64_PAC_EXEC_SHA256,
            'function_entry_file_offset': offset, 'reviewed_instruction_bytes_hex': raw.hex(),
            'points': [{'kind': 'p', 'name': 'acquire_entry', 'file_offset': offset + 4,
                        'opcode_hex': f'{words[1]:08x}', 'instruction': 'stp x29, x30, [sp, #-32]!'},
                       {'kind': 'p', 'name': 'acquire_exit', 'file_offset': offset + 44,
                        'opcode_hex': f'{words[11]:08x}', 'instruction': 'b tail-call, after autiasp'}]}


def kernel_command(path, command):
    """One tracefs command/write; text append open's seek-to-end is unsupported."""
    if not command.endswith('\n') or '\n' in command[:-1]:
        raise ValueError('Exactly one newline-terminated kernel command required')
    raw = command.encode('ascii')
    fd = os.open(path, os.O_WRONLY | os.O_APPEND)
    try:
        if os.write(fd, raw) != len(raw):
            raise OSError('Incomplete atomic tracefs command write; no fragment retry')
    finally:
        os.close(fd)


def write(path, value):
    Path(path).write_text(str(value) + '\n', encoding='ascii')


def run(args, child_args):
    out = Path(args.output)
    if (not out.is_absolute() or '..' in out.parts or out.exists() or
            any(path.is_symlink() for path in (out, *out.parents)) or
            any((path / '.git').exists() for path in out.parents)):
        raise ValueError('Fresh absolute nonsymlink private output directory required')
    if not 1 <= args.timeout_s <= 60:
        raise ValueError('Timeout must be 1..60 seconds')
    runtime = Path(args.runtime_root).resolve()
    if out == runtime or runtime in out.parents:
        raise ValueError('Diagnostic output must stay outside the sealed runtime')
    source = runtime / 'experiments/type1_latency_cause_probe/probe.py'
    if sha256(source) != args.probe_sha256:
        raise ValueError('CPU-only probe source seal differs')
    if '--execute-cpu' not in child_args:
        raise ValueError('Explicit CPU-only child required')
    forbidden = ('--execute-supported', '--execute-fixed-catch', '--front-port', '--rear-port')
    if any(item.partition('=')[0] in forbidden for item in child_args):
        raise ValueError('Hardware execution arguments prohibited')
    # Resolve only the ELF used for symbol lookup/uprobes. Invoking the resolved
    # symlink target instead of a venv's bin/python loses its pyvenv.cfg context.
    invoked_python = Path(args.python)
    if not invoked_python.is_absolute():
        raise ValueError('Absolute invoked Python path required')
    python = invoked_python.resolve()
    offset = symbol_offset(python)
    endpoints = probe_points(python, offset)
    plan = {'schema': 'singularitydog.type1-cause-trace.v1', 'status': 'PLAN',
            'motor_hardware_opened': False, 'output_allowed': False,
            'python': str(python), 'python_sha256': sha256(python),
            'invoked_python': str(invoked_python), 'elf_python': str(python),
            'invoked_python_differs_from_elf': invoked_python != python,
            'probe_sha256': args.probe_sha256, 'restore_thread_file_offset': offset,
            'probe_points': endpoints,
            'endpoint_semantics': endpoints['semantics'], 'probe_kind': endpoints['kind'],
            'lr_hijack_used': endpoints['lr_hijack_used'],
            'clock': 'mono', 'all_system_scheduler_events_recorded': True,
            'trace_events_are_diagnostic_overhead': True}
    if not args.execute_cpu_trace:
        return plan
    if os.geteuid() != 0:
        raise ValueError('Root required for reversible tracefs scope')
    uid = int(os.environ.get('SUDO_UID', '-1'))
    if uid <= 0:
        raise ValueError('Run through sudo from a non-root diagnostic user')
    account = pwd.getpwuid(uid)
    tag = 'dogcause_' + uuid.uuid4().hex[:16]
    instance = TRACE_ROOT / 'instances' / tag
    events = TRACE_ROOT / 'uprobe_events'
    created, registered, child = False, [], None
    handlers, signals_received = {}, []
    unwinding = False
    original_schedstats = SCHEDSTATS.read_text().strip()
    report = dict(plan, status='STARTED', original_schedstats=original_schedstats)
    report['setup_steps'] = []
    out.mkdir(mode=0o700)
    os.chown(out, uid, account.pw_gid)

    def interrupted(signum, _frame):
        signals_received.append({'signal': signum, 'monotonic_ns': time.monotonic_ns()})
        if not unwinding:
            raise TraceInterrupted('Trace scope interrupted by signal ' + str(signum))

    def step(name):
        report['setup_step'] = name
        report['setup_steps'].append({'step': name, 'monotonic_ns': time.monotonic_ns()})

    try:
        # Retain recovery coordinates before changing global state. SIGKILL or
        # machine power loss cannot execute Python finally; this is diagnostic
        # information, never an automatically trusted recovery instruction.
        step('write_recovery_coordinates')
        private_json(out / 'trace-recovery.json', {
            'scope': 'private CPU-only trace scope; manual recovery review only',
            'instance': str(instance), 'probe_group': tag,
            'probe_names': [point['name'] for point in endpoints['points']],
            'original_schedstats': original_schedstats,
            'automatic_recovery_authorized': False})
        step('install_termination_handlers')
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, interrupted)
        step('create_trace_instance')
        instance.mkdir()
        created = True
        step('stop_initial_instance_tracing')
        write(instance / 'tracing_on', 0)
        step('set_instance_buffer_size')
        write(instance / 'buffer_size_kb', 8192)
        step('select_monotonic_trace_clock')
        write(instance / 'trace_clock', 'mono')
        report['trace_clock_readback'] = (instance / 'trace_clock').read_text().strip()
        if '[mono]' not in report['trace_clock_readback']:
            raise RuntimeError('Trace clock readback differs')
        report['timestamp_text_precision_ns'] = 1000
        for point in endpoints['points']:
            kind, name, point_offset = point['kind'], point['name'], point['file_offset']
            step('register_probe_' + name)
            registered.append(name)  # Retain intent before a possibly interrupted kernel write.
            kernel_command(events, f'{kind}:{tag}/{name} {python}:0x{point_offset:x}\n')
            step('enable_probe_' + name)
            write(instance / 'events' / tag / name / 'enable', 1)
        for name in ('sched_switch', 'sched_wakeup'):
            step('enable_scheduler_event_' + name)
            write(instance / 'events/sched' / name / 'enable', 1)
        step('enable_schedstats')
        write(SCHEDSTATS, 1)
        if SCHEDSTATS.read_text().strip() != '1':
            raise RuntimeError('schedstats enable readback failed')
        env = dict(os.environ, PYTHONPATH=str(runtime), PYTHONDONTWRITEBYTECODE='1',
                   HOME=account.pw_dir, USER=account.pw_name, LOGNAME=account.pw_name)
        def child_setup():
            os.setgroups([])
            os.setgid(account.pw_gid)
            os.setuid(uid)
            os.setsid()
        step('start_instance_tracing')
        write(instance / 'tracing_on', 1)
        with (out / 'child.log').open('w') as log:
            step('spawn_cpu_only_child')
            child = subprocess.Popen([str(invoked_python), '-B', '-m', MODULE, *child_args],
                env=env, cwd=str(runtime), stdout=log, stderr=subprocess.STDOUT,
                preexec_fn=child_setup)
            report['child_pid'] = child.pid
            try:
                report['child_exit_code'] = child.wait(timeout=args.timeout_s)
            except subprocess.TimeoutExpired:
                raise RuntimeError('CPU-only trace child timed out')
        step('stop_completed_instance_tracing')
        write(instance / 'tracing_on', 0)
        trace = (instance / 'trace').read_bytes()
        (out / 'trace.txt').write_bytes(trace)
        report['trace_sha256'] = hashlib.sha256(trace).hexdigest()
        report['trace_bytes'] = len(trace)
        report['per_cpu_stats'] = {p.parent.name: p.read_text() for p in
                                  sorted((instance / 'per_cpu').glob('cpu*/stats'))}
        losses = [int(m.group(1)) for value in report['per_cpu_stats'].values()
                  for m in re.finditer(r'^(?:overrun|commit overrun|dropped events):\s*(\d+)', value, re.M)]
        report['trace_buffer_loss_events'] = sum(losses) if losses else None
        report['status'] = 'RECORDED_CPU_ONLY_TRACE'
    except BaseException as error:
        unwinding = True
        report['status'] = 'FAILED_CPU_ONLY_TRACE'
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc()
    finally:
        unwinding = True
        cleanup_errors = []
        try:
            stop_child(child)
        except BaseException as error:
            cleanup_errors.append(f'child: {error}')
        report['child_reaped'] = child is None or child.poll() is not None
        if created:
            # A failed tracing_on write must not skip event disable/remove.
            for name, action in (
                    ('stop', lambda: write(instance / 'tracing_on', 0)),
                    ('disable', lambda: write(instance / 'events/enable', 0)),
                    ('remove', instance.rmdir)):
                try:
                    action()
                except BaseException as error:
                    cleanup_errors.append(f'instance {name}: {error}')
        for name in reversed(registered):
            try:
                # A failed registration does not imply a leaked probe. Read its
                # unique name before removing; never clear other users' probes.
                lines = events.read_text().splitlines()
                present = any(line.split()[0].partition(':')[2] == f'{tag}/{name}'
                              for line in lines if line.split())
                if present:
                    kernel_command(events, f'-:{tag}/{name}\n')
            except BaseException as error:
                cleanup_errors.append(f'probe {name}: {error}')
        try:
            write(SCHEDSTATS, original_schedstats)
            if SCHEDSTATS.read_text().strip() != original_schedstats:
                raise RuntimeError('schedstats restore readback differs')
        except BaseException as error:
            cleanup_errors.append(f'schedstats: {error}')
        for sig, handler in handlers.items():
            try:
                signal.signal(sig, handler)
                if signal.getsignal(sig) != handler:
                    raise RuntimeError('Signal handler restore readback differs')
            except BaseException as error:
                cleanup_errors.append(f'signal {sig}: {error}')
        report['signals_received'] = signals_received
        report['cleanup_errors'] = cleanup_errors
        report['restored'] = not cleanup_errors and report['child_reaped']
        try:
            report['source_unchanged'] = sha256(source) == args.probe_sha256
        except BaseException as error:
            report['source_unchanged'] = False
            report['source_recheck_error'] = f'{type(error).__name__}: {error}'
        if not report['source_unchanged']:
            report['status'] = 'FAILED_CPU_ONLY_TRACE'
        report['child_workload_completed'] = report.get('child_exit_code') == 0
        if report['status'] == 'RECORDED_CPU_ONLY_TRACE' and not report['child_workload_completed']:
            report['status'] = 'RECORDED_CPU_ONLY_TRACE_CHILD_FAILED'
        report['elapsed_end_monotonic_ns'] = time.monotonic_ns()
        report_path = out / 'trace-report.json'
        private_json(report_path, report)
        for path in out.iterdir():
            os.chown(path, uid, account.pw_gid, follow_symlinks=False)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', required=True)
    parser.add_argument('--runtime-root', required=True)
    parser.add_argument('--probe-sha256', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout-s', type=float, default=60.)
    parser.add_argument('--execute-cpu-trace', action='store_true')
    parser.add_argument('child_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not 1 <= args.timeout_s <= 60:
        parser.error('Timeout must be 1..60 seconds')
    child_args = args.child_args[1:] if args.child_args[:1] == ['--'] else args.child_args
    report = run(args, child_args)
    print(json.dumps(report))
    return 0 if report['status'] in ('PLAN', 'RECORDED_CPU_ONLY_TRACE') and report.get('restored', True) else 1


if __name__ == '__main__':
    raise SystemExit(main())
