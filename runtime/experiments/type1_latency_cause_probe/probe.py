"""Explicit CPU/AF_UNIX experiment of the original Type1 Python stages.

No serial descriptor, physical CAN, motor-enable handshake, policy or output
qualification is available here. The synthetic last-output seed is an explicit
test seam, never an admission to the original physical execution path.
"""
import argparse
import ast
from concurrent.futures import Future
from contextlib import contextmanager
import hashlib
import gc
import inspect
import json
import multiprocessing as mp
import os
from pathlib import Path
import queue
import resource
import select
import socket
import stat
import struct
import sys
import tempfile
import textwrap
import threading
import time

from experiments.four_bus_type1 import type1_transport as T
from experiments.four_bus_type1 import build as TB
from experiments.four_bus_diagnostic.pipeline import _take as original_take
from experiments.four_bus_diagnostic.transport_adapter import Group
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser

SCHEMA = 'singularitydog.type1-latency-python-stages.v1'
BOOT = '11111111-2222-3333-4444-555555555555'  # Explicit virtual fixture, not a boot claim.
_INSTRUMENT_LOCK = threading.Lock()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def need(value, message):
    if not value:
        raise ValueError(message)


class Recorder:
    """Calling-thread counters with explicit sampling windows, including failures."""
    def __init__(self, detail='full'):
        self.detail = detail
        self.local = threading.local()
        self.events, self.lock, self.fds = [], threading.Lock(), {}
        self.schedstats_enabled = None
        try:
            self.schedstats_enabled = Path('/proc/sys/kernel/sched_schedstats').read_text().strip() == '1'
        except OSError:
            pass

    def bind(self, port, cycle=-1):
        self.local.port, self.local.cycle, self.local.label = port, cycle, None
        tid = threading.get_native_id()
        if tid not in self.fds:
            try:
                self.fds[tid] = os.open(f'/proc/self/task/{tid}/schedstat', os.O_RDONLY)
            except OSError:
                self.fds[tid] = None

    def context(self, cycle=None, label=None):
        if cycle is not None:
            self.local.cycle = cycle
        if label is not None:
            self.local.label = label

    def owner_before_submit(self, owner):
        """Only foreign-TID schedstat is sampled; main CPU is never called owner CPU."""
        values, error = None, None
        start = time.monotonic_ns()
        try:
            fd = self.fds.get(owner.tid)
            if fd is not None and self.detail == 'full':
                values = [int(v) for v in os.pread(fd, 256, 0).split()]
                need(len(values) == 3, 'Unexpected owner schedstat format')
        except (OSError, ValueError) as caught:
            error = str(caught)
        self.mark('owner_pre_submit', wall_ns=start, target_owner_tid=owner.tid,
                  target_owner_port=owner.group.port, target_owner_schedstat_raw=values,
                  target_owner_sched_cpu_ns=values[0] if values and self.schedstats_enabled is True else None,
                  target_owner_sched_run_delay_ns=values[1] if values and self.schedstats_enabled is True else None,
                  target_owner_sched_timeslices=values[2] if values and self.schedstats_enabled is True else None,
                  target_owner_thread_cpu_ns=None, target_owner_schedstat_error=error)

    def mark(self, stage, *, wall_ns=None, **extra):
        begun = time.monotonic_ns()
        row = {'stage': stage, 'wall_ns': begun if wall_ns is None else wall_ns,
               'sample_started_ns': begun, 'tid': threading.get_native_id(),
               'port': getattr(self.local, 'port', 'main'),
               'cycle': getattr(self.local, 'cycle', -1),
               'exchange': getattr(self.local, 'label', None),
               'thread_cpu_ns': time.thread_time_ns() if self.detail == 'full' else None, 'sched_cpu_ns': None,
               'sched_run_delay_ns': None, 'sched_timeslices': None,
               'schedstats_enabled': self.schedstats_enabled,
               'sched_run_delay_inference_usable': self.schedstats_enabled is True,
               'user_cpu_ns': None, 'system_cpu_ns': None, 'nvcsw': None,
               'nivcsw': None, 'minflt': None, 'majflt': None, 'counter_errors': [],
               'counter_detail': self.detail}
        if self.detail == 'full':
            try:
                r = resource.getrusage(resource.RUSAGE_THREAD)
                row.update(user_cpu_ns=int(r.ru_utime * 1e9), system_cpu_ns=int(r.ru_stime * 1e9),
                           nvcsw=r.ru_nvcsw, nivcsw=r.ru_nivcsw, minflt=r.ru_minflt, majflt=r.ru_majflt)
            except (AttributeError, OSError) as error:
                row['counter_errors'].append('thread_rusage: ' + str(error))
        fd = self.fds.get(row['tid'])
        if fd is not None and self.detail == 'full':
            try:
                values = [int(v) for v in os.pread(fd, 256, 0).split()]
                need(len(values) == 3, 'Unexpected schedstat format')
                row['schedstat_raw'] = values
                # Disabled zeros are retained as raw data, not OS-wait evidence.
                if self.schedstats_enabled is True:
                    row.update(sched_cpu_ns=values[0], sched_run_delay_ns=values[1], sched_timeslices=values[2])
            except (OSError, ValueError) as error:
                row['counter_errors'].append('schedstat: ' + str(error))
        row.update(extra)
        row['sample_finished_ns'] = time.monotonic_ns()
        with self.lock:
            self.events.append(row)
        return row

    def close(self):
        for fd in self.fds.values():
            if fd is not None:
                os.close(fd)
        self.fds.clear()


class Placement:
    """Exact calling-thread CPU/slack restoration; Darwin functional tests are labeled."""
    def __init__(self, cpus):
        self.cpus, self.original, self.timer = set(cpus), None, None
        self.report = {'requested_cpus': sorted(cpus), 'native_tid': None, 'supported': sys.platform == 'linux',
                       'original_cpus': None, 'during_cpus': None, 'after_cpus': None, 'restored': False}

    def __enter__(self):
        self.report['native_tid'] = threading.get_native_id()
        if sys.platform != 'linux':
            self.report.update(restored=True, note='Functional socket test only; Jetson placement not reproduced')
            return self
        from singularitydog_hw.thread_timer_slack import TimerSlack
        self.original = os.sched_getaffinity(0)
        self.report['original_cpus'] = sorted(self.original)
        try:
            os.sched_setaffinity(0, self.cpus)
            self.report['during_cpus'] = sorted(os.sched_getaffinity(0))
            need(set(self.report['during_cpus']) == self.cpus, 'CPU placement readback differs')
            self.timer = TimerSlack(1000)
            self.timer.__enter__()
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, exc_type, error, traceback):
        failures = []
        if self.timer is not None and self.timer._active:
            try:
                self.timer.__exit__(exc_type, error, traceback)
            except BaseException as caught:
                failures.append(str(caught))
        if self.original is not None:
            try:
                os.sched_setaffinity(0, self.original)
                self.report['after_cpus'] = sorted(os.sched_getaffinity(0))
                need(set(self.report['after_cpus']) == self.original, 'CPU restoration readback differs')
            except BaseException as caught:
                failures.append(str(caught))
        self.report['timer_slack'] = self.timer.report if self.timer else None
        self.report['errors'] = failures
        self.report['restored'] = not failures
        if failures:
            if error is not None:
                error.add_note('CPU/slack restoration: ' + str(failures))
            else:
                raise RuntimeError('CPU/slack restoration: ' + str(failures))


def intervals(events):
    """Pair only same-thread stage counters; cross-thread publication is wall-only."""
    ends = {'main_dispatch_before': 'main_dispatch_after',
            'native_call_before': 'native_first_python_statement',
            'checked_decode_before': 'checked_decode_after',
            'batch_constructor_before': 'batch_constructor_after',
            'voltage_buffers_prepare_before': 'voltage_buffers_prepare_after',
            'voltage_preflight_hook_before': 'voltage_preflight_hook_after',
            'current_check_before': 'current_check_after',
            'prefix_set_result_before': 'prefix_set_result_after',
            'outer_set_result_before': 'outer_set_result_after'}
    pending, rows = {}, []
    counters = ('thread_cpu_ns', 'user_cpu_ns', 'system_cpu_ns', 'nvcsw', 'nivcsw',
                'minflt', 'majflt', 'sched_cpu_ns', 'sched_run_delay_ns', 'sched_timeslices')
    for index, event in enumerate(events):
        for start, end in ends.items():
            key = event['tid'], event['cycle'], event['exchange'], start
            if event['stage'] == start:
                pending[key] = index, event
            elif event['stage'] == end and key in pending:
                first_index, first = pending.pop(key)
                row = {'stage': start.removesuffix('_before'), 'port': event['port'],
                       'tid': event['tid'], 'cycle': event['cycle'], 'exchange': event['exchange'],
                       'start_event': first_index, 'end_event': index,
                       'wall_delta_ns': event['wall_ns'] - first['wall_ns'],
                       'counter_sample_start_ns': first['sample_started_ns'],
                       'counter_sample_end_ns': event['sample_finished_ns'],
                       'counter_scope': 'same-thread sampling-window upper bound, not exact stage endpoints'}
                for name in counters:
                    a, b = first.get(name), event.get(name)
                    row[name + '_delta'] = b - a if a is not None and b is not None else None
                rows.append(row)
        if event['stage'] == 'native_first_python_statement':
            rows.append({'stage': 'native_stats_end_to_first_python_statement', 'port': event['port'],
                         'tid': event['tid'], 'cycle': event['cycle'], 'exchange': event['exchange'],
                         'end_event': index, 'native_status': event['native_status'],
                         'wall_delta_ns': event['wall_ns'] - event['native_stats_end_ns'],
                         'counter_scope': 'wall-only: no counter sampled at native Stats.end',
                         'exact_native_return': False, 'GIL_wait_proven': False})
    return rows


def _call_name(node):
    return ast.unparse(node.func) if isinstance(node, ast.Call) else ''


def _statement(source):
    return ast.parse(source).body[0]


class StageTransformer(ast.NodeTransformer):
    """Private copies retain original branches, Ctypes callables and checked bytes."""
    def __init__(self, method):
        self.method = method
        self.native_calls = 0

    def visit_Assign(self, node):
        self.generic_visit(node)
        target = ast.unparse(node.targets[0])
        called = _call_name(node.value)
        if called.startswith('self._library.sda_subset_exchange'):
            self.native_calls += 1
            return [_statement("_probe.mark('native_call_before')"), node,
                    # MUST remain the first Python statement after the real foreign call.
                    _statement('_probe_first_python_wall_ns = time.monotonic_ns()'),
                    _statement("_probe.mark('native_first_python_statement', wall_ns=_probe_first_python_wall_ns, native_stats_end_ns=int(stats.end_ns), native_status=int(status))")]
        labels = {('rows', 'decode_records'): 'checked_decode',
                  ('batch', 'Batch'): 'batch_constructor',
                  ('prepared', 'self._buffers'): 'voltage_buffers_prepare',
                  ('narrowed', 'before_native'): 'voltage_preflight_hook'}
        stage = labels.get((target, called))
        if stage:
            after = f"_probe.mark('{stage}_after'" + (", batch_argument_stamp_ns=batch.completed_ns" if target == 'batch' else '') + ')'
            return [_statement(f"_probe.mark('{stage}_before')"), node, _statement(after)]
        if target == 'self._batch_index[id(batch)]':
            return [node, _statement("_probe.mark('batch_registry_complete')")]
        return node

    def visit_Expr(self, node):
        self.generic_visit(node)
        called = _call_name(node.value)
        if called == 'prefix_future.set_result':
            return [_statement("_probe.mark('prefix_set_result_before')"), node,
                    _statement("_probe.mark('prefix_set_result_after')")]
        if called == 'check':
            return [_statement("_probe.mark('current_check_before')"), node,
                    _statement("_probe.mark('current_check_after')")]
        return node


@contextmanager
def instrument(recorder, expected_source_sha):
    """Global method swaps are bounded to this no-hardware process and restored."""
    need(_INSTRUMENT_LOCK.acquire(blocking=False), 'Another instrumentation scope is active')
    path = Path(T.__file__).resolve()
    originals, ledger = {}, []
    accepted = False
    try:
        need(sha(path) == expected_source_sha, 'Original Type1 Python source SHA differs')
        accepted = True
        for name in ('_native', '_checked_exchange', 'hold_then_voltage'):
            original = getattr(T.Type1Transport, name)
            source = textwrap.dedent(inspect.getsource(original))
            tree = ast.parse(source)
            transform = StageTransformer(name)
            tree = transform.visit(tree)
            if name == '_native':
                need(transform.native_calls == 3, 'Expected all three original Ctypes call sites')
            if name == '_checked_exchange':
                tree.body[0].body.insert(0, _statement('_probe.context(label=label)'))
                tree.body[0].body.insert(1, _statement("_probe.mark('checked_exchange_enter')"))
            ast.fix_missing_locations(tree)
            changed = ast.unparse(tree)
            namespace = dict(T.__dict__, _probe=recorder)
            exec(compile(tree, '<type1_latency_cause_probe:' + name + '>', 'exec'), namespace)
            originals[name] = original
            setattr(T.Type1Transport, name, namespace[name])
            ledger.append({'method': name, 'original_source_sha256': hashlib.sha256(source.encode()).hexdigest(),
                           'instrumented_source_sha256': hashlib.sha256(changed.encode()).hexdigest(),
                           'instrumented_source': changed})
        yield ledger
    finally:
        for name, original in originals.items():
            setattr(T.Type1Transport, name, original)
        _INSTRUMENT_LOCK.release()
        if accepted:
            need(sha(path) == expected_source_sha, 'Original source changed during instrumentation')


def _frame(can_id, data):
    return b'AT' + ((can_id << 3) | 4).to_bytes(4, 'big') + b'\x08' + data + b'\r\n'


def virtual_reply(wire):
    values = ATParser().feed(wire)
    need(len(values) == 1 and len(wire) == 17, 'Exactly one virtual frame required')
    value = values[0]
    mid = value.destination
    if value.kind == 1:
        fields = T.type1_fields(wire)
        need(fields is not None and fields[2] == fields[3] == 0,
             'Virtual peer forbids every positive gain, velocity and torque')
        return _frame((2 << 24) | (2 << 22) | (mid << 8) | 0xfd,
                      struct.pack('>4H', 32767, 32767, 32767, 250))
    need(value.kind == 17 and value.data[:2] == b'\x1c\x70',
         'Virtual peer permits only zero Type1 and voltage reads; enable/STOP forbidden')
    return _frame((17 << 24) | (mid << 8) | 0xfd, value.data[:4] + struct.pack('<f', 40.))


def serve_peer(sockets, delay_us, drop_first, connection):
    """Separate process: synthetic reply generation does not share the measured GIL."""
    buffers, pending, seen, errors = {s: bytearray() for s in sockets}, [], [], []
    placement = Placement((0, 1, 2, 3))
    try:
        placement.__enter__()
        connection.send({'record_kind': 'PEER_READY', 'pid': os.getpid(), 'ports': len(sockets),
                         'CAN_IO': False, 'serial_IO': False, 'output_allowed': False})
        while sockets or pending:
            now = time.monotonic_ns()
            timeout = max(0., (min(p[0] for p in pending) - now) / 1e9) if pending else .05
            readable, _, _ = select.select(sockets, [], [], min(timeout, .05)) if sockets else ([], [], [])
            for s in readable:
                raw = s.recv(4096)
                if not raw:
                    sockets.remove(s)
                    continue
                buffers[s].extend(raw)
                while len(buffers[s]) >= 17:
                    wire = bytes(buffers[s][:17]); del buffers[s][:17]
                    response = virtual_reply(wire)
                    seen.append({'wire_hex': wire.hex(), 'received_ns': time.monotonic_ns(),
                                 'kind': ATParser().feed(wire)[0].kind})
                    if drop_first and len(seen) == 1:
                        continue
                    pending.append((time.monotonic_ns() + delay_us * 1000, s, response))
            now = time.monotonic_ns()
            due = [p for p in pending if p[0] <= now]
            pending = [p for p in pending if p[0] > now]
            for _, s, response in due:
                s.sendall(response)
    except BaseException as error:
        errors.append(type(error).__name__ + ': ' + str(error))
    finally:
        try:
            placement.__exit__(None, None, None)
        except BaseException as error:
            errors.append('peer placement cleanup: ' + str(error))
        for s in buffers:
            s.close()
        try:
            connection.send({'record_kind': 'PEER_COMPLETE_REPORT', 'frames': seen, 'errors': errors, 'synthetic_mode2': True,
                             'motor_enable_sent': False, 'positive_gain_sent': False,
                             'placement': placement.report})
        finally:
            connection.close()


def checked_socket_fd(value):
    need(type(value) is socket.socket and value.family == socket.AF_UNIX and value.type == socket.SOCK_STREAM,
         'Only AF_UNIX stream socketpairs are admitted')
    need(stat.S_ISSOCK(os.fstat(value.fileno()).st_mode), 'Socket descriptor required')
    return value.fileno()


class PeerStartupError(ValueError):
    def __init__(self, message):
        self.peer_message = message
        super().__init__('Virtual peer startup failed or readiness identity/scope differs: ' + str(message))


def await_peer_ready(connection, peer_pid, ports):
    """A cold spawn must be ready before the original 200ms cycle budget begins."""
    if not connection.poll(5.):
        raise TimeoutError('Virtual peer did not become ready within bounded 5s startup wait')
    message = connection.recv()
    if not (type(message) is dict and message.get('record_kind') == 'PEER_READY' and
            message.get('pid') == peer_pid and message.get('ports') == ports and
            message.get('CAN_IO') is False and message.get('serial_IO') is False and
            message.get('output_allowed') is False):
        raise PeerStartupError(message)
    return message


class Owner:
    def __init__(self, library, host, group, boot, cancel_read, cancel_all, recorder):
        self.tasks, self.ready, self.errors = queue.Queue(), threading.Event(), []
        self.recorder, self.group, self.tid, self.transport = recorder, group, None, None
        self.raw_journal = []
        self.placement = Placement((int(group.port[-1]),))
        self.thread = threading.Thread(target=self._run,
            args=(library, host, boot, cancel_read, cancel_all), name='cause-' + group.port)
        self.thread.start()

    def _run(self, library, host, boot, cancel_read, cancel_all):
        self.tid = threading.get_native_id()
        self.recorder.bind(self.group.port)
        try:
            self.placement.__enter__()
            ids = self.group.ids
            self.transport = T.Type1Transport.create(library, checked_socket_fd(host), group=self.group,
                cancel_fd=cancel_read, boot_fd=boot.fileno(), boot_id=BOOT,
                axis_raw_bounds={mid: (-.025, .025) for mid in ids},
                kp_cap_by_id=dict.fromkeys(ids, 0.), kd_cap_by_id=dict.fromkeys(ids, 0.),
                cancel_all=cancel_all, decode_once=True)
            wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in ids)
            wires, fields = self.transport._checked_output(wires)
            # Explicit virtual-only seed. No enable/zero_gain/output call is made.
            self.transport._last_output, self.transport._last_fields = wires, fields
        except BaseException as error:
            self.errors.append(type(error).__name__ + ': ' + str(error))
        finally:
            self.ready.set()
        if self.errors:
            try:
                if self.transport is not None:
                    self.transport.close()
                self.placement.__exit__(None, None, None)
            except BaseException as error:
                self.errors.append('placement cleanup: ' + str(error))
            return
        try:
            while True:
                task = self.tasks.get()
                if task is None:
                    return
                cycle, prefix, outer, deadline = task
                self.recorder.context(cycle=cycle, label='dispatch')
                self.recorder.mark('owner_dispatch_enter')
                try:
                    result = self.transport.hold_then_voltage(wires, ids[cycle % 3], prefix,
                        deadline_ns=deadline, check=lambda: None)
                    self.recorder.mark('owner_call_returned')
                    self.recorder.mark('outer_set_result_before')
                    outer.set_result(result)
                    self.recorder.mark('outer_set_result_after')
                except BaseException as error:
                    self.recorder.mark('owner_cycle_failed', error=type(error).__name__ + ': ' + str(error))
                    if not prefix.done():
                        prefix.set_exception(error)
                    outer.set_exception(error)
                finally:
                    self.recorder.mark('owner_cycle_finally', poisoned=self.transport.poisoned)
        finally:
            try:
                self.raw_journal = [{'exchange': label, 'raw': T.exchange_evidence(*raw)}
                                    for label, raw in self.transport.journal]
            except BaseException as error:
                self.errors.append('raw_journal: ' + str(error))
            finally:
                try:
                    self.transport.close()
                    self.recorder.mark('owner_session_closed')
                except BaseException as error:
                    self.errors.append('close: ' + str(error))
                finally:
                    try:
                        self.placement.__exit__(None, None, None)
                    except BaseException as error:
                        self.errors.append('placement cleanup: ' + str(error))

    def close(self):
        self.tasks.put(None)
        self.thread.join(2.)
        need(not self.thread.is_alive(), 'Virtual owner failed to join')


def prepare(args):
    need(type(args.cycles) is int and 1 <= args.cycles <= 1000, 'Cycles must be 1..1000')
    need(args.ports in (1, 4), 'Exactly one or four virtual ports')
    need(0 <= args.reply_delay_us <= 10000, 'Virtual delay must be 0..10000 us')
    need(0 <= args.start_delay_s <= 10, 'Start delay must be 0..10 seconds')
    need(args.output.is_absolute() and not args.output.exists(), 'Fresh absolute output required')
    need(args.library.is_absolute() and args.library.name == TB.LIBRARY_NAME, 'Pinned original library path required')
    pins = {name: getattr(args, name) for name in ('ordinary_source_sha256', 'subset_stop_source_sha256',
             'extension_source_sha256', 'build_record_sha256')}
    pins['expected_sha256'] = args.library_sha256
    need(all(type(v) is str and len(v) == 64 and all(c in '0123456789abcdef' for c in v) for v in
             [*pins.values(), args.transport_source_sha256]), 'Explicit SHA256 pins required')
    need(sha(T.__file__) == args.transport_source_sha256, 'Original Type1 Python source SHA differs')
    need(TB.receipt_problem(args.library.parent) is None, 'Original library receipt rejected')
    record = json.loads((args.library.parent / 'build-record.json').read_bytes())
    scope = record['four_bus_subset_active']
    need(sha(args.library) == args.library_sha256 and sha(args.library.parent / 'build-record.json') == args.build_record_sha256,
         'Original binary or receipt SHA differs')
    need(all(scope[name] == value for name, value in pins.items() if name.endswith('_source_sha256')),
         'Original native source pins differ')
    if args.ready_fd is not None:
        need(type(args.ready_fd) is int and args.ready_fd > 2 and stat.S_ISFIFO(os.fstat(args.ready_fd).st_mode),
             'Ready FD must be an anonymous pipe/FIFO, never a device or terminal')
    return pins


def _run_impl(args, pins):
    """Only callers with a prepared file contract can create local sockets."""
    need(args.execute_cpu is True, 'Explicit execute-cpu required')
    need(prepare(args) == pins, 'Prepared arguments changed')
    args.output.mkdir(mode=0o700, parents=True)
    recorder = Recorder(args.counter_detail); recorder.bind('main')
    owners, sockets, peers, cycles, errors, peer_report = [], [], [], [], [], None
    process, control, peer_control, peer_ready = None, None, None, None
    cancel_read, cancel_write = os.pipe()
    os.set_blocking(cancel_read, False)
    boot = tempfile.TemporaryFile(); boot.write((BOOT + '\n').encode()); boot.flush()
    cancel_lock = threading.Lock()
    def cancel_all():
        with cancel_lock:
            try:
                os.write(cancel_write, b'x')
            except OSError:
                pass
    ledger, restored = [], False
    hot_calls = 0
    gc_before, gc_during, gc_restored = gc.isenabled(), None, False
    def hot():
        # Exact original Future takeout path, but this CPU-only experiment has
        # no physical/current-boot guard. Do not pretend this no-op is one.
        nonlocal hot_calls
        hot_calls += 1
    try:
        library = T.load_library(args.library, **pins)
        for index in range(args.ports):
            host, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
            host.setblocking(False); sockets.append(host); peers.append(peer)
        ctx = mp.get_context('spawn')
        control, peer_control = ctx.Pipe(duplex=False)
        process = ctx.Process(target=serve_peer, args=(peers, args.reply_delay_us, args.drop_first_reply, peer_control),
                              name='cause-virtual-peer')
        process.start(); peer_control.close()
        for peer in peers:
            peer.close()
        try:
            peer_ready = await_peer_ready(control, process.pid, args.ports)
        except PeerStartupError as error:
            if type(error.peer_message) is dict and error.peer_message.get('record_kind') == 'PEER_COMPLETE_REPORT':
                peer_report = error.peer_message
            raise
        groups = [Group('port' + str(index), tuple(range(1 + index * 3, 4 + index * 3))) for index in range(args.ports)]
        with instrument(recorder, args.transport_source_sha256) as instrumentation:
            ledger = instrumentation
            for host, group in zip(sockets, groups):
                owners.append(Owner(library, host, group, boot, cancel_read, cancel_all, recorder))
            for owner in owners:
                need(owner.ready.wait(5.) and not owner.errors, 'Virtual owner initialization failed: ' + str(owner.errors))
            ready = {'status': 'READY_CPU_AF_UNIX_ONLY', 'pid': os.getpid(),
                     'owner_native_tids': [o.tid for o in owners], 'peer_pid': process.pid,
                     'CAN_IO': False, 'serial_IO': False, 'output_allowed': False}
            line = (json.dumps(ready) + '\n').encode()
            if args.ready_fd is not None:
                need(os.write(args.ready_fd, line) == len(line), 'Incomplete ready pipe write')
            print(line.decode(), end='', flush=True)
            time.sleep(args.start_delay_s)
            gc.collect()
            gc.disable()
            gc_during = gc.isenabled()
            for cycle in range(args.cycles):
                recorder.context(cycle=cycle, label='dispatch')
                prefixes, futures = [Future() for _ in owners], [Future() for _ in owners]
                row = {'cycle': cycle, 'status': 'STARTED', 'errors': []}
                cycles.append(row)
                recorder.mark('main_dispatch_before')
                deadline = time.monotonic_ns() + 200_000_000  # Diagnostic bound, never a 20ms admission.
                for owner, prefix, outer in zip(owners, prefixes, futures):
                    recorder.owner_before_submit(owner)
                    owner.tasks.put((cycle, prefix, outer, deadline))
                recorder.mark('main_dispatch_after')
                for port, prefix in enumerate(prefixes):
                    try:
                        recorder.mark('main_prefix_take_before', awaited_port=port)
                        original_take(prefix, deadline, time.monotonic_ns, hot)
                        recorder.mark('main_prefix_future_returned', awaited_port=port)
                    except BaseException as error:
                        row['errors'].append(type(error).__name__ + ': ' + str(error))
                for port, outer in enumerate(futures):
                    try:
                        recorder.mark('main_outer_take_before', awaited_port=port)
                        result = original_take(outer, deadline, time.monotonic_ns, hot)
                        owners[port].transport.verify_batch(result[0], 'feedback_hold')
                        owners[port].transport.verify_batch(result[1], 'voltage')
                        recorder.mark('main_outer_future_returned', awaited_port=port)
                    except BaseException as error:
                        row['errors'].append(type(error).__name__ + ': ' + str(error))
                row['status'] = 'FAILED' if row['errors'] else 'COMPLETE'
                recorder.mark('main_cycle_finally', failed=bool(row['errors']))
                if row['errors']:
                    errors.extend(row['errors'])
                    break  # No unsafe retry/relabel of poisoned genuine sessions.
            if gc_before:
                gc.enable()
            else:
                gc.disable()
            for owner in owners:
                owner.close()
            T.verify_library(library)
        restored = True
    except BaseException as error:
        errors.append(type(error).__name__ + ': ' + str(error))
    finally:
        if gc_before:
            gc.enable()
        else:
            gc.disable()
        gc_restored = gc.isenabled() == gc_before
        # Owner work is bounded to 200ms; cancellation precedes joining/descriptor close.
        cancel_all()
        for owner in owners:
            if owner.thread.is_alive():
                try:
                    owner.close()
                except BaseException as error:
                    errors.append('owner_cleanup: ' + str(error))
            errors.extend(owner.errors)
        for host in sockets:
            host.close()
        for peer in peers:
            peer.close()
        if peer_control:
            peer_control.close()
        if process is not None and process.pid is not None:
            if peer_report is None:
                try:
                    if control.poll(2.):
                        peer_report = control.recv()
                        need(type(peer_report) is dict and peer_report.get('record_kind') == 'PEER_COMPLETE_REPORT',
                             'Final peer report kind differs')
                except (EOFError, ValueError) as error:
                    errors.append('peer_report: ' + str(error))
            process.join(2.)
            if process.is_alive():
                process.terminate(); process.join(2.)
                errors.append('Virtual peer required termination')
            if peer_report:
                errors.extend(peer_report['errors'])
            else:
                errors.append('Virtual peer report absent')
        if control:
            control.close()
        os.close(cancel_read); os.close(cancel_write); boot.close()
        recorder.close()
    report = {'schema': SCHEMA, 'status': 'ABORTED_CPU_PROBE' if errors else 'COMPLETE_CPU_PROBE',
        'errors': errors, 'cycles': cycles, 'failed_cycles': sum(c['status'] == 'FAILED' for c in cycles),
        'requested_cycles': args.cycles, 'completed_cycles': sum(c['status'] == 'COMPLETE' for c in cycles),
        'events': recorder.events, 'intervals': intervals(recorder.events),
        'peer': peer_report, 'peer_ready': peer_ready, 'instrumentation': ledger,
        'instrumentation_restored': restored, 'original_source_unchanged': sha(T.__file__) == args.transport_source_sha256,
        'source_sha256': sha(__file__), 'transport_source_sha256': args.transport_source_sha256,
        'native_library': str(args.library), 'native_pins': pins, 'pid': os.getpid(),
        'owner_native_tids': [o.tid for o in owners], 'ports': args.ports, 'reply_delay_us': args.reply_delay_us,
        'owner_raw_journals': {o.group.port: o.raw_journal for o in owners},
        'groups': {o.group.port: list(o.group.ids) for o in owners},
        'owner_placement': {o.group.port: o.placement.report for o in owners},
        'gc_scope': {'before_enabled': gc_before, 'cycle_enabled': gc_during, 'restored': gc_restored,
                     'collect_before_cycles': True}, 'counter_detail': args.counter_detail,
        'take_mode': 'exact_original_pipeline_take_with_counted_cpu_only_noop_hot',
        'take_source_sha256': sha(inspect.getsourcefile(original_take)), 'noop_hot_calls': hot_calls,
        'physical_current_guard_used': False, 'torch_or_model_loaded_by_probe': False,
        'torch_import_present_in_process': 'torch' in sys.modules, 'model_execution_called': False,
        'CAN_IO': False, 'serial_IO': False, 'physical_type1_sent': False,
        'motor_enable_sent': False, 'positive_gain_sent': False, 'learned_targets_sent': False,
        'output_allowed': False, 'runtime_qualified': False, 'original_r3_report_relabeled': False,
        'virtual_last_output_seed': True, 'virtual_boot_fixture': True,
        'method_calls': ['hold_then_voltage'], 'diagnostic_absolute_budget_ns': 200_000_000,
        'sampling_overhead_is_included': True, 'counter_samples_are_interval_upper_bounds': True,
        'native_stats_end_is_not_exact_function_return': True,
        'gil_or_os_wait_diagnosed_from_wall_alone': False,
        'clock_scope': 'Linux original transport CLOCK_MONOTONIC; other platforms require independent domain check'}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def run(args, pins):
    need(args.execute_cpu is True, 'Explicit execute-cpu required')
    need(prepare(args) == pins, 'Prepared arguments changed')
    placement, original_switch = Placement((4,)), sys.getswitchinterval()
    report, failure = None, None
    try:
        with placement:
            sys.setswitchinterval(.0001)
            report = _run_impl(args, pins)
    except BaseException as error:
        failure = error
    finally:
        sys.setswitchinterval(original_switch)
    if failure is not None:
        raise failure
    report['main_placement'] = placement.report
    report['python_switch_interval'] = {'before_s': original_switch, 'during_s': .0001,
        'after_s': sys.getswitchinterval(), 'restored': sys.getswitchinterval() == original_switch}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def parser():
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--execute-cpu', action='store_true')
    p.add_argument('--library', type=Path, required=True)
    for name in ('library', 'ordinary-source', 'subset-stop-source', 'extension-source', 'build-record', 'transport-source'):
        p.add_argument('--' + name + '-sha256', required=True)
    p.add_argument('--cycles', type=int, default=5)
    p.add_argument('--ports', type=int, choices=(1, 4), default=4)
    p.add_argument('--reply-delay-us', type=int, default=2000)
    p.add_argument('--start-delay-s', type=float, default=1.)
    p.add_argument('--ready-fd', type=int)
    p.add_argument('--counter-detail', choices=('full', 'minimal'), default='full')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--drop-first-reply', action='store_true', help='AF_UNIX negative control only')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    pins = prepare(args)
    if not args.execute_cpu:
        print(json.dumps({'schema': SCHEMA, 'status': 'PLAN', 'CAN_IO': False, 'serial_IO': False,
                          'loads_library': False, 'creates_sockets': False, 'output_allowed': False,
                          'native_pins': pins, 'transport_source_sha256': args.transport_source_sha256}))
        return 0
    report = run(args, pins)
    print(json.dumps({k: report[k] for k in ('schema', 'status', 'errors', 'completed_cycles', 'failed_cycles',
                                          'instrumentation_restored', 'output_allowed')}))
    return 0 if not report['errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
