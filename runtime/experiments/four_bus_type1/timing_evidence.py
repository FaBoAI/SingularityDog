"""Opt-in F0 timing evidence for the four-bus Type1 run (foreground --timing-evidence).

Diagnostic only: never a contract, admission or timing-qualification input,
and no output permission. The runner samples it once per cycle after the
post-reply admission, outside the release->output window, and once before and
after the cycle loop. Per-thread descriptors are opened once by bind() and
re-read with pread. Linux-only counters are None elsewhere (no /proc, no
RUSAGE_THREAD on macOS). Reads are total: a failure records None, never raises.
"""
import os
from pathlib import Path
import resource
import threading

SCHEMA = 'singularitydog.four-bus-type1-timing-evidence.v1'
THREAD_FIELDS = ('run_delay_ns', 'timeslices', 'minflt', 'majflt', 'nvcsw', 'nivcsw')
VMSTAT_PREFIXES = ('thp_', 'compact_')
_FILES = ('schedstat', 'stat', 'status')
RUN_FILE_LIMIT = 1 << 20  # /proc/interrupts is tens of KB on a Jetson; larger is recorded as None.


def _int(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def parse_schedstat(raw):
    """/proc/<tid>/schedstat: on-CPU ns, run-queue wait (run delay) ns, timeslices."""
    fields = (raw or '').split()
    return {'run_delay_ns': _int(fields[1]) if len(fields) > 2 else None,
            'timeslices': _int(fields[2]) if len(fields) > 2 else None}


def parse_stat(raw):
    """/proc/<tid>/stat fields 10 (minflt) and 12 (majflt), after the comm parenthesis."""
    tail = (raw or '').rsplit(')', 1)[-1].split() if raw and ')' in raw else []
    return {'minflt': _int(tail[7]) if len(tail) > 9 else None, 'majflt': _int(tail[9]) if len(tail) > 9 else None}


def parse_status(raw):
    values = {'nvcsw': None, 'nivcsw': None}
    for line in (raw or '').splitlines():
        name, _, value = line.partition(':')
        if name == 'voluntary_ctxt_switches':
            values['nvcsw'] = _int(value.strip())
        elif name == 'nonvoluntary_ctxt_switches':
            values['nivcsw'] = _int(value.strip())
    return values


def parse_vmstat(raw):
    if raw is None:
        return None
    values = {}
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].startswith(VMSTAT_PREFIXES):
            values[parts[0]] = _int(parts[1])
    return values


def parse_interrupts(raw):
    """{label: [count per CPU]} from /proc/interrupts (header gives the CPU count)."""
    if raw is None:
        return None
    lines = raw.splitlines()
    if not lines:
        return None
    cpus = len(lines[0].split())
    values = {}
    for line in lines[1:]:
        label, _, rest = line.partition(':')
        counts = []
        for token in rest.split()[:cpus]:
            if _int(token) is None:
                break
            counts.append(int(token))
        if label.strip() and counts:
            values[label.strip()] = counts
    return values


def delta(before, after):
    """Recursive after-before over equal-shaped counters; anything unmatched is None."""
    if type(before) is int and type(after) is int:
        return after-before
    if type(before) is dict and type(after) is dict:
        return {key: delta(before.get(key), value) for key, value in after.items()}
    if type(before) is list and type(after) is list and len(before) == len(after):
        return [delta(a, b) for a, b in zip(before, after)]
    return None


class TimingEvidence:
    """Read-only /proc and getrusage counters; opens no device and writes nothing."""
    def __init__(self, proc_root='/proc', *, rusage_thread=getattr(resource, 'RUSAGE_THREAD', None)):
        self.root = Path(proc_root)
        self.rusage_thread = rusage_thread
        self.fds, self.threads, self.sampler = {}, {}, None

    @staticmethod
    def _read(fd):
        if fd is None:
            return None
        try:
            return os.pread(fd, 8192, 0).decode('ascii', 'replace')
        except OSError:
            return None

    @staticmethod
    def _read_all(fd):
        """Whole seq_file to EOF (one read returns about a page); over the limit or failed is None."""
        if fd is None:
            return None
        chunks, size = [], 0
        try:
            while True:
                chunk = os.read(fd, 65536)
                if not chunk:
                    return b''.join(chunks).decode('ascii', 'replace')
                size += len(chunk)
                if size > RUN_FILE_LIMIT:
                    return None
                chunks.append(chunk)
        except OSError:
            return None

    def _open(self, path):
        try:
            return os.open(path, os.O_RDONLY | getattr(os, 'O_CLOEXEC', 0))
        except OSError:
            return None

    def bind(self, threads):
        """Open /proc/self/task/<tid>/{schedstat,stat,status} once per named thread."""
        self.sampler = threading.get_native_id()
        result = {}
        for name, tid in threads.items():
            self.threads[name] = tid
            self.fds[name] = {kind: self._open(self.root/'self'/'task'/str(tid)/kind) if type(tid) is int else None
                              for kind in _FILES}
            result[name] = {'native_tid': tid,
                            'proc_counters': {kind: fd is not None for kind, fd in self.fds[name].items()},
                            'getrusage_thread_self': tid == self.sampler and self.rusage_thread is not None}
        return {'schema': SCHEMA, 'fields': list(THREAD_FIELDS), 'threads': result,
                'sampled_on_native_tid': self.sampler}

    def sample(self):
        """One compact per-thread counter snapshot; the sampling thread uses its own getrusage."""
        values = {}
        for name, fds in self.fds.items():
            row = {**parse_schedstat(self._read(fds['schedstat'])), **parse_stat(self._read(fds['stat'])),
                   **parse_status(self._read(fds['status']))}
            if self.threads[name] == self.sampler and self.rusage_thread is not None:
                try:
                    usage = resource.getrusage(self.rusage_thread)
                    row.update(minflt=usage.ru_minflt, majflt=usage.ru_majflt,
                               nvcsw=usage.ru_nvcsw, nivcsw=usage.ru_nivcsw)
                except (OSError, ValueError):
                    pass
            values[name] = row
        return values

    def run_counters(self):
        """Run-level /proc/vmstat thp_*/compact_* and /proc/interrupts, read whole (None if absent)."""
        def read(name):
            fd = self._open(self.root/name)
            try:
                return self._read_all(fd)
            finally:
                if fd is not None:
                    os.close(fd)
        return {'vmstat': parse_vmstat(read('vmstat')), 'interrupts': parse_interrupts(read('interrupts'))}

    @staticmethod
    def delta(before, after):
        return delta(before, after)

    @staticmethod
    def run_delta(before, after):
        """Run delta; interrupt lines that did not change are omitted."""
        value = delta(before, after) or {}
        rows = value.get('interrupts')
        if type(rows) is dict:
            value['interrupts'] = {label: counts for label, counts in rows.items()
                                   if counts is None or any(counts)}
        return value

    def close(self):
        for fds in self.fds.values():
            for kind, fd in fds.items():
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    fds[kind] = None
