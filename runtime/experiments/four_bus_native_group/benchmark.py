"""Explicit CPU/AF_UNIX comparison of Python and native four-port Type1 owners.

This is a transport-only synthetic fixture. It has no serial or CAN device,
robot boot/UID evidence, model, motion approval, or live timing qualification.
The default PLAN does not load a DSO, open a descriptor, or start a thread.
"""
import argparse
from concurrent.futures import Future
import gc
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import resource
import select
import socket
import struct
import sys
import tempfile
import threading
import time

from singularitydog_hw.can_readonly import ATParser

SCHEMA = 'singularitydog.native-group-cpu-comparison.v1'
BOOT = '11111111-2222-3333-4444-555555555555'  # Synthetic fixture, never robot identity.
WRITE_NS = 20_000_000
PEER_COMPLETE_TIMEOUT_S = 5.
PEER_JOIN_TIMEOUT_S = 2.
GROUP_IDS = {'port0': (7, 8, 9), 'port1': (10, 11, 12),
             'port2': (4, 5, 6), 'port3': (1, 2, 3)}


def _need(ok, message):
    if not ok:
        raise ValueError(message)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _frame(can_id, data):
    return b'AT' + ((can_id << 3) | 4).to_bytes(4, 'big') + b'\x08' + data + b'\r\n'


def _virtual_reply(wire):
    """Only synthetic enable/STOP, zero-gain Type1 and voltage requests."""
    from experiments.four_bus_type1 import type1_transport as T
    frames = ATParser().feed(wire)
    _need(len(frames) == 1 and len(wire) == 17, 'Virtual peer expected one AT frame')
    value = frames[0]
    mid = value.destination
    _need(mid in range(1, 13), 'Virtual peer received an out-of-range ID')
    if value.kind == 1:
        fields = T.type1_fields(wire)
        _need(fields is not None and fields[2] == fields[3] == 0,
              'Virtual peer forbids every positive gain, velocity and torque')
        return _frame((2 << 24) | (2 << 22) | (mid << 8) | 0xfd,
                      struct.pack('>4H', 32767, 32767, 32767, 250))
    if value.kind == 3:  # Synthetic enable acknowledgement only.
        return _frame((2 << 24) | (2 << 22) | (mid << 8) | 0xfd,
                      struct.pack('>4H', 32767, 32767, 32767, 250))
    if value.kind in (4, 18):  # Synthetic STOP acknowledgement only.
        return _frame((2 << 24) | (mid << 8) | 0xfd,
                      struct.pack('>4H', 32767, 32767, 32767, 250))
    _need(value.kind == 17 and value.data[:2] == b'\x1c\x70',
          'Virtual peer permits only a voltage read outside zero Type1 and control')
    return _frame((17 << 24) | (mid << 8) | 0xfd,
                  value.data[:4] + struct.pack('<f', 40.))


class _NoPlacement:
    def __init__(self):
        self.report = {'requested_cpus': [], 'supported': False, 'restored': True,
                       'note': 'Portable AF_UNIX functional fixture; no CPU/slack placement'}

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


def _placement(cpus, portable):
    if portable:
        return _NoPlacement()
    from experiments.type1_latency_cause_probe.probe import Placement
    return Placement(cpus)


def _serve_peer(sockets, delay_us, portable, connection):
    """A separate process owns all fake replies, never the measured GIL."""
    placement = _placement((0, 1, 2, 3), portable)
    all_sockets = tuple(sockets.values())
    buffers = {port: bytearray() for port in sockets}
    pending, seen, errors = [], [], []
    try:
        placement.__enter__()
        connection.send({'kind': 'READY', 'pid': os.getpid(), 'ports': sorted(sockets),
                         'serial_io': False, 'can_io': False})
        while sockets or pending:
            now = time.monotonic_ns()
            delay = max(0., (min(row[0] for row in pending)-now)/1e9) if pending else .05
            readable, _, _ = select.select(list(sockets.values()), [], [], min(delay, .05)) if sockets else ([], [], [])
            for sock in readable:
                port = next(key for key, value in sockets.items() if value is sock)
                raw = sock.recv(4096)
                if not raw:
                    del sockets[port]
                    continue
                buffers[port].extend(raw)
                while len(buffers[port]) >= 17:
                    wire = bytes(buffers[port][:17]); del buffers[port][:17]
                    response = _virtual_reply(wire)
                    seen.append({'port': port, 'kind': ATParser().feed(wire)[0].kind,
                                 'wire_hex': wire.hex(), 'received_ns': time.monotonic_ns()})
                    pending.append((time.monotonic_ns()+delay_us*1000, sock, response))
            now = time.monotonic_ns()
            due = [row for row in pending if row[0] <= now]
            pending = [row for row in pending if row[0] > now]
            for _, sock, response in due:
                sock.sendall(response)
    except BaseException as error:
        errors.append(type(error).__name__+': '+str(error))
    finally:
        try:
            placement.__exit__(None, None, None)
        except BaseException as error:
            errors.append('Peer placement cleanup: '+str(error))
        for sock in all_sockets:
            sock.close()
        try:
            connection.send({'kind': 'COMPLETE', 'frames': seen, 'errors': errors,
                             'placement': placement.report})
        finally:
            connection.close()


class _Owner:
    """Persistent original Python owner, retained idle while C owns its Session."""
    def __init__(self, port, lib, host, cancel_read, boot, cancel_all, *, portable):
        self.port, self.tasks, self.ready = port, queue.Queue(), threading.Event()
        self.transport, self.error, self.placement = None, None, None
        self.thread = threading.Thread(target=self._run,
            args=(lib, host, cancel_read, boot, cancel_all, portable),
            name='cpu-compare-'+port)
        self.thread.start()

    def _run(self, lib, host, cancel_read, boot, cancel_all, portable):
        from experiments.four_bus_diagnostic.transport_adapter import Group
        from experiments.four_bus_type1 import type1_transport as T
        from singularitydog_hw import native_active_transport as active
        self.placement = _placement((int(self.port[-1]),), portable)
        try:
            self.placement.__enter__()
            ids = GROUP_IDS[self.port]
            self.transport = T.Type1Transport.create(lib, host.fileno(), group=Group(self.port, ids),
                cancel_fd=cancel_read, boot_fd=boot.fileno(), boot_id=BOOT,
                axis_raw_bounds={mid: (-.05, .05) for mid in ids},
                kp_cap_by_id=dict.fromkeys(ids, 3.), kd_cap_by_id=dict.fromkeys(ids, .15),
                cancel_all=cancel_all, decode_once=True, separate_write_deadline=True)
            for mid in ids:
                self.transport.enable(mid, deadline_ns=time.monotonic_ns()+200_000_000)
                self.transport.zero_gain(mid, 0., deadline_ns=time.monotonic_ns()+200_000_000)
            wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in ids)
            now = time.monotonic_ns()
            self.transport.output(wires, deadline_ns=now+200_000_000,
                                  write_deadline_ns=now+100_000_000, check=lambda: None)
        except BaseException as error:
            self.error = type(error).__name__+': '+str(error)
        finally:
            self.ready.set()
        try:
            if self.error is None:
                while True:
                    task = self.tasks.get()
                    if task is None:
                        break
                    name, arguments, future = task
                    try:
                        if name == 'input':
                            wires, mid, deadline = arguments
                            prefix = Future()
                            result = self.transport.hold_then_voltage(
                                wires, mid, prefix, deadline_ns=deadline, check=lambda: None)
                        elif name == 'output':
                            wires, deadline, write_deadline = arguments
                            result = self.transport.output(wires, deadline_ns=deadline,
                                write_deadline_ns=write_deadline, check=lambda: None)
                        elif name == 'stop':
                            result = self.transport.stop_repeated(total_budget_ns=500_000_000, rounds=2)
                        else:
                            raise ValueError('Unknown virtual owner task')
                        future.set_result(result)
                    except BaseException as error:
                        future.set_exception(error)
        finally:
            try:
                if self.transport is not None:
                    self.transport.close()
            except BaseException as error:
                self.error = (self.error or '')+' close: '+type(error).__name__+': '+str(error)
            try:
                self.placement.__exit__(None, None, None)
            except BaseException as error:
                self.error = (self.error or '')+' placement: '+type(error).__name__+': '+str(error)

    def submit(self, name, *arguments):
        future = Future()
        self.tasks.put((name, arguments, future))
        return future

    def close(self):
        self.tasks.put(None)
        self.thread.join(2.)
        _need(not self.thread.is_alive(), 'Virtual Python owner did not join')


def _counter():
    row = resource.getrusage(resource.RUSAGE_SELF)
    return {'user_cpu_s': row.ru_utime, 'system_cpu_s': row.ru_stime,
            'voluntary_context_switches': row.ru_nvcsw,
            'involuntary_context_switches': row.ru_nivcsw,
            'minor_faults': row.ru_minflt, 'major_faults': row.ru_majflt}


def _counter_delta(before, after):
    return {key: after[key]-before[key] for key in before}


def _common_pacing_before():
    return {'target_switch_interval_s': .0001,
            'before': {'switch_interval_s': sys.getswitchinterval(),
                       'gc_enabled': gc.isenabled()},
            'pre_cycle_gc_collect': None, 'during': None, 'after': None,
            'restore_errors': [], 'restored': False}


def _common_pacing_activate(row):
    """Apply the same 100 us/GC deferral to either CPU-only variant."""
    sys.setswitchinterval(row['target_switch_interval_s'])
    begin = time.monotonic_ns()
    collected = None
    try:
        collected = gc.collect()  # Setup only, after bootstrap and before timed cycles.
    finally:
        end = time.monotonic_ns()
        row['pre_cycle_gc_collect'] = {'begin_ns': begin, 'end_ns': end,
                                       'duration_ns': end-begin,
                                       'collected_objects': collected}
    gc.disable()
    row['during'] = {'switch_interval_s': sys.getswitchinterval(),
                     'gc_enabled': gc.isenabled()}
    _need(abs(row['during']['switch_interval_s']-.0001) <= 1e-12 and
          row['during']['gc_enabled'] is False,
          'Common CPU-only benchmark pacing could not be applied')


def _common_pacing_restore(row):
    """Try both independent restorations even if one fails or cleanup raised."""
    before = row['before']
    try:
        sys.setswitchinterval(math.nextafter(before['switch_interval_s'], math.inf))
    except BaseException as error:
        row['restore_errors'].append('switch interval: '+type(error).__name__+': '+str(error))
    try:
        if before['gc_enabled']:
            gc.enable()
        else:
            gc.disable()
    except BaseException as error:
        row['restore_errors'].append('GC state: '+type(error).__name__+': '+str(error))
    try:
        row['after'] = {'switch_interval_s': sys.getswitchinterval(),
                        'gc_enabled': gc.isenabled()}
        row['restored'] = (row['after'] == before and not row['restore_errors'])
    except BaseException as error:
        row['restore_errors'].append('readback: '+type(error).__name__+': '+str(error))
    if not row['restored'] and not row['restore_errors']:
        row['restore_errors'].append('Pacing readback differs from original state')


def _summary(values):
    if not values:
        return {'count': 0, 'median_ms': None, 'p95_ms': None, 'p99_ms': None, 'max_ms': None}
    ordered = sorted(values)
    def percentile(p):
        return round(ordered[min(len(ordered)-1, int((len(ordered)-1)*p))]/1e6, 6)
    return {'count': len(values), 'median_ms': percentile(.5), 'p95_ms': percentile(.95),
            'p99_ms': percentile(.99), 'max_ms': round(ordered[-1]/1e6, 6)}


def _batch_evidence(batches):
    return {port: {'native_begin_ns': int(batch.stats.begin_ns),
                   'native_end_ns': int(batch.stats.end_ns),
                   'writes': int(batch.stats.writes),
                   'first_write_start_ns': min(int(row.start_ns) for row in batch.records),
                   'last_write_end_ns': max(int(row.finish_ns) for row in batch.records),
                   'last_reply_ns': max(int(row.received_ns) for row in batch.records),
                   'tx_hex': [bytes(row.tx).hex() for row in batch.records],
                   'rx_hex': [bytes(row.rx).hex() for row in batch.records],
                   'written': [int(row.written) for row in batch.records],
                   'received': [int(row.received) for row in batch.records]}
            for port, batch in batches.items()}


def _raw_image(raw):
    records, stats = raw
    return {'records': [{'start_ns': int(row.start_ns), 'finish_ns': int(row.finish_ns),
                         'read_start_ns': int(row.read_start_ns), 'received_ns': int(row.received_ns),
                         'deadline_ns': int(row.deadline_ns), 'written': int(row.written),
                         'received': int(row.received), 'tx_hex': bytes(row.tx).hex(),
                         'rx_hex': bytes(row.rx).hex()} for row in records],
            'stats': {name: int(getattr(stats, name)) for name in
                      ('begin_ns', 'end_ns', 'waits', 'reads', 'bytes', 'writes',
                       'rejected_size', 'rejected_total')},
            'rejected_hex': bytes(stats.rejected)[:min(4096, int(stats.rejected_size))].hex(),
            'rejected_sha256': hashlib.sha256(bytes(stats.rejected)).hexdigest()}


def _adapter_journal(adapters):
    return {port: [{'label': label, 'raw': _raw_image(raw)} for label, raw in adapter.journal]
            for port, adapter in adapters.items()}


def _group_journal(lease):
    if lease is None:
        return None
    return [{**{key: value for key, value in evidence.items() if key != 'raw'},
             'raw': _raw_image(evidence['raw'])} for evidence in lease.journal]


def _run_cycles(mode, owners, *, cycles, portable):
    from experiments.four_bus_diagnostic.transport_adapter import PORTS
    from experiments.four_bus_native_group import client
    from singularitydog_hw.policy_clock import PolicyClock
    adapters = {port: owners[port].transport for port in PORTS}
    wires = {port: adapter._last_output for port, adapter in adapters.items()}
    lease, rows, stop_rows, initial_hold = None, [], {}, None
    main_placement = _placement((4,), portable)
    common_pacing = _common_pacing_before()
    counter_before = _counter()
    clock = PolicyClock(30)
    failure = None
    try:
        main_placement.__enter__()
        if mode == 'native':
            lease = client.NativeGroup(adapters, cpu_masks=(0, 0, 0, 0) if portable else (1, 2, 4, 8),
                                       timer_slack_ns=0 if portable else 1000,
                                       socket_fixture=portable)
            initial_hold = lease.initial_hold(wires, time.monotonic_ns()+200_000_000, lambda: None)
        else:
            now = time.monotonic_ns()
            futures = {port: owners[port].submit('output', wires[port], now+200_000_000,
                                                   now+100_000_000) for port in PORTS}
            initial_hold = {port: future.result(timeout=.3) for port, future in futures.items()}
        _common_pacing_activate(common_pacing)
        epoch = time.monotonic_ns()+10_000_000
        for index in range(cycles):
            target = clock.release_ns(epoch, index)
            while True:
                remain = target-time.monotonic_ns()
                if remain <= 0:
                    break
                if remain > 300_000:
                    time.sleep((remain-200_000)/1e9)
            begin = time.monotonic_ns()
            reply_deadline = begin+clock.cycle_budget_ns
            write_deadline = begin+WRITE_NS
            item = {'cycle': index, 'planned_start_ns': target, 'actual_start_ns': begin,
                    'start_lateness_ns': max(0, begin-target), 'reply_deadline_ns': reply_deadline,
                    'write_deadline_ns': write_deadline}
            rows.append(item)
            item['input_submitted_ns'] = time.monotonic_ns()
            if mode == 'python':
                futures = {port: owners[port].submit('input', wires[port],
                    adapters[port].group.ids[index % 3], reply_deadline)
                           for port in PORTS}
                joined = {port: future.result(timeout=max(.001, (reply_deadline-time.monotonic_ns())/1e9))
                          for port, future in futures.items()}
                hold = {port: pair[0] for port, pair in joined.items()}
                voltages = {port: pair[1] for port, pair in joined.items()}
            else:
                rotating = {port: adapters[port].group.ids[index % 3] for port in PORTS}
                ticket = lease.start_hold(wires, rotating, reply_deadline, lambda: None)
                hold = lease.wait_hold(ticket)
                voltages = lease.wait_voltage(ticket)
            item['input_collected_ns'] = time.monotonic_ns()
            item['hold'] = _batch_evidence(hold)
            item['voltage'] = _batch_evidence(voltages)
            item['output_submitted_ns'] = time.monotonic_ns()
            if mode == 'python':
                futures = {port: owners[port].submit('output', wires[port], reply_deadline, write_deadline)
                           for port in PORTS}
                outputs = {port: future.result(timeout=max(.001, (reply_deadline-time.monotonic_ns())/1e9))
                           for port, future in futures.items()}
            else:
                outputs = lease.output(wires, reply_deadline, write_deadline, lambda: None)
            item['output_collected_ns'] = time.monotonic_ns()
            item['output'] = _batch_evidence(outputs)
            item['last_output_write_ns'] = max(value['last_write_end_ns'] for value in item['output'].values())
            item['last_reply_ns'] = max(value['last_reply_ns'] for value in item['output'].values())
            item['write_deadline_met'] = item['last_output_write_ns'] <= write_deadline
            item['reply_deadline_met'] = item['last_reply_ns'] <= reply_deadline
            _need(item['write_deadline_met'] and item['reply_deadline_met'],
                  'Virtual cycle exceeded original write/reply deadlines')
    except BaseException as error:
        failure = type(error).__name__+': '+str(error)
    finally:
        close_report = None
        if lease is not None:
            try:
                close_report = lease.close()
                if (close_report.get('closed') is not True or
                    close_report.get('restored') is not True or
                    close_report.get('snapshot_complete') is not True or
                    close_report.get('snapshot_error') is not None or
                    close_report.get('error') is not None or
                    close_report.get('raw_snapshot_conflicts')):
                    failure = (failure or '')+' Native group close/restoration/snapshot incomplete: '+str(close_report)
            except BaseException as error:
                failure = (failure or '')+' Native group close: '+type(error).__name__+': '+str(error)
        try:
            if lease is None or lease.closed:
                # Cleanup-only subset STOP is sent after the C owner has joined.
                stop_futures = {}
                for port in PORTS:
                    try:
                        stop_futures[port] = owners[port].submit('stop')
                    except BaseException as error:
                        stop_rows[port] = {'complete': False, 'error':
                            type(error).__name__+': '+str(error)}
                stop_deadline = time.monotonic()+2.
                for port, future in stop_futures.items():
                    try:
                        stop_rows[port] = future.result(timeout=max(0., stop_deadline-time.monotonic()))
                    except BaseException as error:
                        stop_rows[port] = {'complete': False, 'error':
                            type(error).__name__+': '+str(error)}
                if not all(row['complete'] for row in stop_rows.values()):
                    failure = (failure or '')+' Synthetic subset STOP incomplete'
            else:
                failure = (failure or '')+' C owners not joined; old STOP not attempted'
        except BaseException as error:
            failure = (failure or '')+' Synthetic subset STOP: '+type(error).__name__+': '+str(error)
        finally:
            try:
                main_placement.__exit__(None, None, None)
            except BaseException as error:
                failure = (failure or '')+' Main placement restoration: '+type(error).__name__+': '+str(error)
            finally:
                _common_pacing_restore(common_pacing)
                if not common_pacing['restored']:
                    failure = (failure or '')+' Common pacing restoration: '+\
                        '; '.join(common_pacing['restore_errors'])
    counter_after = _counter()
    metrics = {
        'start_interval': _summary([b['actual_start_ns']-a['actual_start_ns'] for a, b in zip(rows, rows[1:])]),
        'start_lateness': _summary([row['start_lateness_ns'] for row in rows]),
        'input_until_collected': _summary([row['input_collected_ns']-row['actual_start_ns'] for row in rows if 'input_collected_ns' in row]),
        'output_handoff': _summary([row['output_submitted_ns']-row['input_collected_ns'] for row in rows if 'output_submitted_ns' in row]),
        'hold_reply_to_voltage_native_begin': _summary([
            row['voltage'][port]['native_begin_ns'] - row['hold'][port]['last_reply_ns']
            for row in rows if 'voltage' in row for port in PORTS]),
        'voltage_reply_to_output_native_begin': _summary([
            min(port['native_begin_ns'] for port in row['output'].values()) -
            max(port['last_reply_ns'] for port in row['voltage'].values())
            for row in rows if 'output' in row]),
        'output_submit_to_native_begin': _summary([min(port['native_begin_ns'] for port in row['output'].values())-row['output_submitted_ns'] for row in rows if 'output' in row]),
        'last_output_write': _summary([row['last_output_write_ns']-row['actual_start_ns'] for row in rows if 'last_output_write_ns' in row]),
        'oldest_input_start_to_last_output_write': _summary([
            row['last_output_write_ns'] -
            min(port['first_write_start_ns'] for port in row['hold'].values())
            for row in rows if 'last_output_write_ns' in row]),
        'last_reply': _summary([row['last_reply_ns']-row['actual_start_ns'] for row in rows if 'last_reply_ns' in row]),
        'return_from_output': _summary([row['output_collected_ns']-row['actual_start_ns'] for row in rows if 'output_collected_ns' in row]),
        'native_output_end_to_collection': _summary([
            row['output_collected_ns'] - max(port['native_end_ns'] for port in row['output'].values())
            for row in rows if 'output_collected_ns' in row]),
        'write_deadline_misses': sum(row.get('write_deadline_met') is False for row in rows),
        'reply_deadline_misses': sum(row.get('reply_deadline_met') is False for row in rows),
        'start_lateness_over_1ms': sum(row['start_lateness_ns'] > 1_000_000 for row in rows),
    }
    return {'mode': mode, 'status': 'COMPLETE' if failure is None else 'ABORTED',
            'error': failure, 'cycles_requested': cycles,
            'cycles_completed': sum('output' in row for row in rows),
            'cycles_partially_observed': sum('output' not in row for row in rows),
            'cycles_not_started': cycles-len(rows),
            'rows': rows, 'metrics': metrics, 'counter_scope': 'whole process during variant; not per owner',
            'counter_delta': _counter_delta(counter_before, counter_after),
            'counter_window': 'variant main placement, initial hold, pre-cycle GC, cycles and synthetic STOP',
            'main_placement': main_placement.report,
            'common_pacing': common_pacing,
            'owner_placement': {port: owners[port].placement.report for port in PORTS},
            'native_group_settings': lease.settings if lease is not None else None,
            'native_group_close': close_report, 'stop': stop_rows,
            'initial_hold': _batch_evidence(initial_hold) if initial_hold is not None else None,
            'owner_raw_journal': _adapter_journal(adapters),
            'native_group_raw_journal': _group_journal(lease)}


def _run_variant(mode, library, *, cycles, delay_us, portable):
    from experiments.four_bus_diagnostic.transport_adapter import PORTS
    ctx = mp.get_context('spawn')
    pairs, owners = {}, {}
    parent_connection = child_connection = process = boot = None
    cancel_read = cancel_write = None
    peer_report = result = run_error = None
    cleanup_errors = []
    def cancel_all():
        try:
            os.write(cancel_write, b'x')
        except (BlockingIOError, OSError):
            pass
    try:
        for port in PORTS:
            pairs[port] = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
            pairs[port][0].setblocking(False)
        parent_connection, child_connection = ctx.Pipe(duplex=False)
        peers = {port: pair[1] for port, pair in pairs.items()}
        process = ctx.Process(target=_serve_peer, args=(peers, delay_us, portable, child_connection),
                              name='cpu-compare-peer-'+mode)
        cancel_read, cancel_write = os.pipe()
        os.set_blocking(cancel_read, False)
        os.set_blocking(cancel_write, False)
        boot = tempfile.TemporaryFile()
        boot.write((BOOT+'\n').encode()); boot.flush()
        process.start()
        child_connection.close()
        for peer in peers.values():
            peer.close()
        _need(parent_connection.poll(10.), 'Virtual peer startup timeout')
        ready = parent_connection.recv()
        _need(ready == {'kind': 'READY', 'pid': process.pid, 'ports': list(PORTS),
                        'serial_io': False, 'can_io': False}, 'Virtual peer startup identity differs')
        for port in PORTS:
            owners[port] = _Owner(port, library, pairs[port][0], cancel_read, boot, cancel_all,
                                  portable=portable)
        for owner in owners.values():
            _need(owner.ready.wait(5.), 'Virtual owner startup timeout')
            _need(owner.error is None, owner.error or '')
        result = _run_cycles(mode, owners, cycles=cycles, portable=portable)
    except BaseException as error:
        run_error = type(error).__name__+': '+str(error)
    finally:
        for owner in owners.values():
            try:
                owner.close()
            except BaseException as error:
                cleanup_errors.append('owner '+owner.port+': '+type(error).__name__+': '+str(error))
        for host, peer in pairs.values():
            for sock in (host, peer):
                try:
                    sock.close()
                except BaseException as error:
                    cleanup_errors.append('socket close: '+type(error).__name__+': '+str(error))
        for owner in owners.values():
            if owner.thread.is_alive():
                owner.thread.join(timeout=1.)
            if owner.thread.is_alive():
                cleanup_errors.append('Virtual owner remained alive after socket closure: '+owner.port)
        if process is not None and process.pid is not None:
            try:
                if parent_connection is not None and parent_connection.poll(PEER_COMPLETE_TIMEOUT_S):
                    peer_report = parent_connection.recv()
                else:
                    cleanup_errors.append('Virtual peer completion report timed out')
            except BaseException as error:
                cleanup_errors.append('Virtual peer report: '+type(error).__name__+': '+str(error))
            process.join(timeout=PEER_JOIN_TIMEOUT_S)
            if process.is_alive():
                process.terminate(); process.join(timeout=1.)
                cleanup_errors.append('Virtual peer required termination')
            if process.is_alive():
                process.kill(); process.join(timeout=1.)
                cleanup_errors.append('Virtual peer required kill')
            if process.is_alive():
                cleanup_errors.append('Virtual peer remained alive after kill')
        for connection in (parent_connection, child_connection):
            if connection is not None:
                try:
                    connection.close()
                except BaseException as error:
                    cleanup_errors.append('pipe close: '+type(error).__name__+': '+str(error))
        for fd in (cancel_read, cancel_write):
            if fd is not None:
                try:
                    os.close(fd)
                except BaseException as error:
                    cleanup_errors.append('cancel fd close: '+type(error).__name__+': '+str(error))
        if boot is not None:
            try:
                boot.close()
            except BaseException as error:
                cleanup_errors.append('boot fixture close: '+type(error).__name__+': '+str(error))
    if result is None:
        result = {'mode': mode, 'status': 'ABORTED', 'error': run_error,
                  'cycles_requested': cycles, 'cycles_completed': 0,
                  'cycles_partially_observed': 0, 'cycles_not_started': cycles,
                  'rows': [], 'owner_raw_journal': _adapter_journal({port: owner.transport
                      for port, owner in owners.items() if owner.transport is not None})}
    if peer_report is None or peer_report.get('kind') != 'COMPLETE':
        cleanup_errors.append('No complete synthetic peer report')
    elif peer_report.get('errors'):
        cleanup_errors.append('Synthetic peer errors: '+str(peer_report['errors']))
    if run_error or cleanup_errors or any(owner.error for owner in owners.values()):
        result['status'] = 'ABORTED'
        result['error'] = '; '.join(filter(None, [result.get('error'), run_error, *cleanup_errors,
                                           *(owner.error for owner in owners.values())]))
    result['cleanup_errors'] = cleanup_errors
    result['peer'] = peer_report
    result['owner_errors'] = {port: owner.error for port, owner in owners.items()}
    return result


def _expected_pins(args):
    from experiments.four_bus_native_group import client
    from experiments.four_bus_type1 import type1_transport as T
    from singularitydog_hw import policy_clock
    _need(_sha(T.__file__) == args.type1_python_sha256, 'Original Python Type1 source pin differs')
    _need(_sha(client.__file__) == args.client_python_sha256, 'Native group client source pin differs')
    _need(_sha(policy_clock.__file__) == args.policy_clock_python_sha256,
          'PolicyClock30 Python source pin differs')
    library = Path(args.library).resolve(strict=True)
    _need(library.name == 'libdog_four_bus_type1_transport.so', 'Explicit composite library required')
    return client.load_group_library(library,
        expected_sha256=args.expected_sha256,
        ordinary_source_sha256=args.ordinary_source_sha256,
        subset_stop_source_sha256=args.subset_stop_source_sha256,
        subset_active_source_sha256=args.subset_active_source_sha256,
        group_source_sha256=args.group_source_sha256,
        build_record_sha256=args.build_record_sha256)


def _verify_unchanged(args, library, benchmark_sha256):
    from experiments.four_bus_native_group import build, client
    from experiments.four_bus_type1 import type1_transport as T
    from singularitydog_hw import policy_clock
    directory = Path(args.library).resolve(strict=True).parent
    source_names = ('ordinary_transport.cpp', 'subset_stop.cpp', 'subset_active.cpp', 'transport.cpp')
    source_pins = (args.ordinary_source_sha256, args.subset_stop_source_sha256,
                   args.subset_active_source_sha256, args.group_source_sha256)
    client.verify_group_library(library)
    _need(build.receipt_problem(directory) is None and
          _sha(directory/'build-record.json') == args.build_record_sha256 and
          _sha(args.library) == args.expected_sha256 and
          tuple(_sha(directory/name) for name in source_names) == source_pins and
          _sha(T.__file__) == args.type1_python_sha256 and
          _sha(client.__file__) == args.client_python_sha256 and
          _sha(__file__) == benchmark_sha256 and
          _sha(policy_clock.__file__) == args.policy_clock_python_sha256,
          'Composite/Python source or build pin changed during CPU comparison')
    return {'unchanged': True, 'checked_after_both_variants': True}


def _validate_execution(args):
    _need(type(args.cycles) is int and 1 <= args.cycles <= 1000, 'Cycles must be 1..1000')
    _need(type(args.reply_delay_us) is int and 0 <= args.reply_delay_us <= 10000,
          'Virtual peer delay must be 0..10000 microseconds')
    _need(args.output is not None and args.output.is_absolute() and not args.output.exists(),
          'A fresh absolute output directory is required')
    repository = Path(__file__).resolve().parents[3]
    target = args.output.parent.resolve(strict=True)/args.output.name
    _need(repository not in (target, *target.parents), 'Raw benchmark output must be outside the repository')
    for name in ('library', 'expected_sha256', 'ordinary_source_sha256',
                 'subset_stop_source_sha256', 'subset_active_source_sha256',
                 'group_source_sha256', 'build_record_sha256',
                 'type1_python_sha256', 'client_python_sha256',
                 'policy_clock_python_sha256'):
        _need(getattr(args, name) is not None, 'Execution requires explicit '+name)
    if sys.platform != 'linux':
        _need(args.portable_socket_fixture, 'Non-Linux execution must be labeled portable socket fixture')


def _compare_wires(results):
    a, b = results['python'], results['native']
    if a['status'] != 'COMPLETE' or b['status'] != 'COMPLETE':
        return {'identical': False, 'reason': 'Aborted variant has no complete command-parity verdict',
                'completed_python': a['cycles_completed'], 'completed_native': b['cycles_completed']}
    if a['cycles_completed'] != b['cycles_completed']:
        return {'identical': False, 'reason': 'cycle count differs'}
    for left, right in zip(a['rows'], b['rows']):
        for phase in ('hold', 'voltage', 'output'):
            for port in GROUP_IDS:
                if left[phase][port]['tx_hex'] != right[phase][port]['tx_hex']:
                    return {'identical': False, 'cycle': left['cycle'], 'phase': phase, 'port': port}
    return {'identical': True, 'cycles': a['cycles_completed'],
            'scope': 'cycle hold/voltage/output Type1 and Type17 command bytes only'}


def execute(args):
    from singularitydog_hw.policy_clock import PolicyClock
    _validate_execution(args)
    benchmark_sha256 = _sha(__file__)
    library = _expected_pins(args)  # Authenticate before creating output, sockets or threads.
    args.output.mkdir(parents=False, exist_ok=False)
    report = {'schema': SCHEMA, 'status': 'RUNNING', 'scope': 'CPU_ONLY_AF_UNIX_TRANSPORT',
              'live_qualification': False, 'model_inference': False, 'serial_or_can_io': False,
              'physical_robot_boot_or_uid': False, 'cycles': args.cycles,
              'period_rule': 'PolicyClock(30).release_ns(epoch,index)',
              'reply_cap_ns': PolicyClock(30).cycle_budget_ns,
              'output_write_cap_ns': WRITE_NS,
              'native_gap_ns': 900_000, 'native_window': 3,
              'portable_socket_fixture': args.portable_socket_fixture,
              'library': str(Path(args.library).resolve()),
              'benchmark_python_sha256_at_start': benchmark_sha256,
              'pins': {name: getattr(args, name) for name in (
                  'expected_sha256', 'ordinary_source_sha256', 'subset_stop_source_sha256',
                  'subset_active_source_sha256', 'group_source_sha256',
                  'build_record_sha256', 'type1_python_sha256', 'client_python_sha256',
                  'policy_clock_python_sha256')},
              'variants': {}}
    error = None
    try:
        for mode in ('python', 'native'):
            report['variants'][mode] = _run_variant(mode, library, cycles=args.cycles,
                delay_us=args.reply_delay_us, portable=args.portable_socket_fixture)
            report['post_'+mode+'_source_pins'] = _verify_unchanged(args, library, benchmark_sha256)
        report['command_bytes'] = _compare_wires(report['variants'])
        report['post_run_source_pins'] = _verify_unchanged(args, library, benchmark_sha256)
        _need(all(value['status'] == 'COMPLETE' for value in report['variants'].values()),
              'One or both CPU-only transport variants aborted; inspect retained raw report')
        _need(report['command_bytes']['identical'], 'Virtual command bytes differ between variants')
        report['status'] = 'COMPLETE_CPU_FIXTURE_ONLY'
    except BaseException as caught:
        error = caught
        report['status'] = 'ABORTED_CPU_FIXTURE_ONLY'
        report['error'] = type(caught).__name__+': '+str(caught)
    finally:
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if error is not None:
        raise error
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-cpu', action='store_true')
    parser.add_argument('--portable-socket-fixture', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--library')
    for name in ('expected_sha256', 'ordinary_source_sha256', 'subset_stop_source_sha256',
                 'subset_active_source_sha256', 'group_source_sha256', 'build_record_sha256',
                 'type1_python_sha256', 'client_python_sha256',
                 'policy_clock_python_sha256'):
        parser.add_argument('--'+name.replace('_', '-'))
    parser.add_argument('--cycles', type=int, default=500)
    parser.add_argument('--reply-delay-us', type=int, default=0)
    args = parser.parse_args(argv)
    if not args.execute_cpu:
        print(json.dumps({'status': 'PLAN', 'schema': SCHEMA, 'opens_devices': False,
                          'loads_library': False, 'starts_threads': False,
                          'transport_only': True, 'live_qualification': False,
                          'execute_flag': '--execute-cpu'}))
        return 0
    report = execute(args)
    print(json.dumps({'status': report['status'], 'output': str(args.output),
                      'command_bytes_identical': report['command_bytes']['identical'],
                      'python': report['variants']['python']['metrics'],
                      'native': report['variants']['native']['metrics']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
