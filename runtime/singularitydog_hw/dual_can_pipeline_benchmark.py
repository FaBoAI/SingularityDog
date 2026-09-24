"""Two independent six-motor buses, finite CAN-only read benchmark.

No inference, IMU, motor settings or output. Each explicit port gets exactly six
identities and20 cycles of12 reads at window4/gap0.5ms/by-parameter. A shared
failure interrupts both owners; no retries or implicit port discovery exist.
"""
import argparse
from contextlib import contextmanager
import copy
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import threading
import time

from . import can_pipeline_probe as pipeline
from .event_snapshot import snapshot_event
from .sensor_pipeline_benchmark import BootIdentityGuard

SCOPES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
FLAGS = {'motor_output_available': False, 'output_allowed': False,
         'approved_for_runtime': False, 'full_controller_50Hz_verified': False,
         'live_50hz_verified': False, 'inference_calls': 0, 'imu_accessed': False}
_HELD_LEASES = []  # Unconfirmed close: retain common ownership until process exit.
MAX_EVENTS_PER_BUS = 10000
CYCLE_BARRIER_TIMEOUT_NS = 250_000_000


def require(value, reason):
    if not value: raise ValueError(reason)


def make_plan(front_port, rear_port, max_seconds=10., *, paired_cycle_sync=False,
              receive_mode='serial'):
    require(type(max_seconds) in (int, float) and math.isfinite(max_seconds)
            and 1 <= max_seconds <= 10, 'max_seconds must be finite and1..10')
    require(str(front_port) != str(rear_port), 'Ports must be distinct')
    require(type(paired_cycle_sync) is bool, 'paired_cycle_sync must be an explicit boolean')
    require(receive_mode in ('serial', 'select'), 'receive_mode must be serial or select')
    plan = {**FLAGS, 'max_seconds': max_seconds, 'cycles_per_bus': 20,
        'receive_mode': receive_mode,
        'requests_per_bus': 246, 'total_request_limit': 492, 'window': 4, 'gap_ms': .5,
        'request_order': 'by-parameter', 'request_timeout_seconds': .25,
        'automatic_retry': False, 'allowed_can_types': [0, 17],
        'ports': {'front': str(front_port), 'rear': str(rear_port)},
        'ids': {s: list(ids) for s, ids in SCOPES.items()},
        'baudrate': 921600, 'serial_exclusive': True, 'dtr': False, 'rts': False,
        'identity_barrier': 'both six-identity groups before any telemetry',
        'per_cycle_barrier': False,
        'cycle_alignment_caveat': 'independent cycles may drift; combined span includes observed start skew',
        'audit_logging': 'bounded per-bus memory; JSON/fsync only after both workers actually exit',
        'max_events_per_bus': MAX_EVENTS_PER_BUS,
        'same_cycle_measure': 'earliest original request to latest original reply across both buses',
        'late_same_key_previous_cycle_disambiguation': False}
    if paired_cycle_sync:
        plan.update(per_cycle_barrier=True, cycle_barrier_timeout_seconds=.25,
            cycle_alignment_caveat='paired start gate is not simultaneous UART delivery or50Hz freshness proof',
            cycle_wait_accounting='gate wait and paired-start intervals reported separately; original request/reply span unchanged')
    return plan


class PairedCycleGate:
    """Finite ordered two-party gate; abort or cancellation wakes both workers.

    The first arrival starts a250ms gate deadline, also bounded by the run budget.
    Waiting checks external cancellation at least every10ms. No CAN operation or
    timestamp rewriting occurs here; the original acquisition spans remain valid.
    """
    def __init__(self, check, deadline_ns, clock=time.monotonic_ns):
        self.check, self.deadline_ns, self.clock = check, deadline_ns, clock
        self.condition = threading.Condition()
        self.broken, self.cycle = False, 1
        self.arrivals, self.releases = {}, {}
        self.last_by_scope = {scope: 0 for scope in SCOPES}

    def abort(self):
        with self.condition:
            self.broken = True
            self.condition.notify_all()

    def wait(self, scope, cycle):
        try:
            # External checks never hold the gate mutex: a blocked boot read
            # must not prevent another worker/coordinator from aborting the gate.
            self.check()
            with self.condition:
                require(scope in SCOPES and type(cycle) is int and 1 <= cycle <= 20,
                        'Invalid paired cycle scope/index')
                require(not self.broken, 'Paired cycle gate was aborted')
                require(cycle == self.cycle and cycle == self.last_by_scope[scope]+1,
                        'Paired cycles must be ordered once per bus')
                arrived = self.clock()
                group = self.arrivals.setdefault(cycle, {})
                require(scope not in group, 'Duplicate paired cycle arrival')
                group[scope] = arrived
                self.last_by_scope[scope] = cycle
                end = min(self.deadline_ns, min(group.values())+CYCLE_BARRIER_TIMEOUT_NS)
                if len(group) == 2:
                    released = self.clock()
                    require(released < end, 'Paired cycle gate deadline exhausted')
                    self.releases[cycle] = released
                    self.cycle += 1
                    self.condition.notify_all()
            while True:
                self.check()
                with self.condition:
                    require(not self.broken, 'Paired cycle gate was aborted')
                    if cycle in self.releases:
                        finished = self.clock()
                        if finished >= end:raise TimeoutError('Paired cycle gate return missed its deadline')
                        return {'arrived_ns': arrived, 'released_ns': self.releases[cycle],
                                'wait_finished_ns': finished, 'wait_ms': (finished-arrived)/1e6}
                    remaining = end-self.clock()
                    if remaining <= 0:raise TimeoutError('Paired cycle gate deadline exhausted')
                    self.condition.wait(timeout=min(.01, remaining/1e9))
        except BaseException:
            self.abort()
            raise


def validate_ports(front_port, rear_port):
    result = {}
    for scope, value in (('front', front_port), ('rear', rear_port)):
        path = Path(value)
        require(path.is_absolute() and path.parent == Path('/dev/serial/by-path'),
                'Use explicit absolute /dev/serial/by-path ports')
        resolved = path.resolve(strict=True)
        info = resolved.stat()
        require(stat.S_ISCHR(info.st_mode), 'Port must resolve to a character device')
        result[scope] = {'path': str(path), 'resolved': str(resolved), 'st_rdev': info.st_rdev}
    require(result['front']['resolved'] != result['rear']['resolved']
            and result['front']['st_rdev'] != result['rear']['st_rdev'],
            'Two paths resolve to the same physical serial device')
    return result


def binding_matches(binding):
    resolved = Path(binding['path']).resolve(strict=True)
    info = resolved.stat()
    return (str(resolved) == binding['resolved'] and stat.S_ISCHR(info.st_mode)
            and info.st_rdev == binding['st_rdev'])


def opened_binding_matches(probe, binding):
    info = os.fstat(probe.raw_port.fileno())
    return (binding_matches(binding) and stat.S_ISCHR(info.st_mode)
            and info.st_rdev == binding['st_rdev'])


@contextmanager
def port_lock(resolved):
    name = hashlib.sha256(str(resolved).encode()).hexdigest()[:24]
    with open('/tmp/singularitydog-can-port-'+name+'.lock', 'a+') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


class CommonLease:
    """Coordinator and each worker retain ownership through actual port close."""
    def __init__(self, factory):
        self.context = factory()
        self.lock = threading.Lock()
        self.references, self.released, self.release_error = 1, False, None
        self.context.__enter__()

    def retain(self):
        with self.lock:
            require(self.references > 0, 'Common CAN ownership already released')
            self.references += 1

    def release(self):
        with self.lock:
            require(self.references > 0, 'Unbalanced common CAN ownership release')
            self.references -= 1
            if self.references == 0:
                try:
                    self.context.__exit__(None, None, None)
                    self.released = True
                except BaseException as error:
                    self.release_error = repr(error)
                    raise


class ScopedPipelineCAN(pipeline.PipelineCAN):
    """Reuse canonical write/receive boundaries; only explicit six-ID batches."""
    def __init__(self, scope, port, emit, *, max_seconds=10., serial_port=None,
                 clock=time.monotonic_ns, check_interrupt=lambda: None,
                 identity_barrier=lambda: None, cycle_barrier=None, receive_mode='serial'):
        require(scope in SCOPES, 'Scope must be front or rear')
        self.scope, self.port_path = scope, str(port)
        self.ids, self.identity_barrier = SCOPES[scope], identity_barrier
        self.cycle_barrier = cycle_barrier
        self.device_closed = False
        super().__init__(emit, window=4, gap_ms=.5, cycles=20, max_seconds=max_seconds,
            request_order='by-parameter', serial_port=serial_port, clock=clock,
            check_interrupt=check_interrupt, receive_mode=receive_mode)
        self.plan.update(ids=list(self.ids), reads_per_cycle=12, max_requests=246,
                         scope=scope, port=self.port_path)
        if cycle_barrier is not None:
            self.plan.update(per_cycle_barrier=True, cycle_barrier_timeout_seconds=.25)

    def __enter__(self):
        try:
            self.check_budget()
            if self.raw_port is None:
                import serial
                self.raw_port = serial.Serial(port=None, baudrate=921600, bytesize=8,
                    parity='N', stopbits=1, timeout=.003, write_timeout=.1,
                    xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
                self.raw_port.dtr = self.raw_port.rts = False
                self.raw_port.port = self.port_path
                self.raw_port.open()
            # Base entry installs the canonical write boundary and configures
            # the selected receiver once; setup failures close the raw port.
            return super().__enter__()
        except BaseException:
            if self.raw_port is not None: self.raw_port.close()
            self.device_closed = True
            raise

    def __exit__(self, *args):
        super().__exit__(*args)
        self.device_closed = True

    def validate_wire(self, wire):
        require(self.current_write is not None and self.current_write[0] in self.ids,
                'CAN write outside assigned six-motor scope')
        super().validate_wire(wire)

    def collect(self, expected_uids):
        report = {**FLAGS, 'status': 'INCOMPLETE', 'errors': [], 'plan': self.plan,
                  'requests': self.rows, 'cycles': self.cycle_rows}
        current = None
        try:
            expected = pipeline.validate_uids(expected_uids)
            self._batch([(mid, None) for mid in self.ids], window=1, cycle=0, expected_uids=expected)
            self.identities_verified = True
            self.emit({'kind': 'dual_identities_verified', 'scope': self.scope,
                       'ids': list(self.ids), 'monotonic_ns': self.clock()})
            self.identity_barrier()
            self.check_budget()
            for number in range(1, 21):
                self.check_budget()
                paired = None
                if self.cycle_barrier is not None:
                    self.emit({'kind': 'dual_cycle_wait', 'scope': self.scope,
                               'cycle': number, 'monotonic_ns': self.clock()})
                    paired = self.cycle_barrier(number)
                    self.check_budget()
                    self.emit({'kind': 'dual_cycle_barrier', 'scope': self.scope,
                               'cycle': number, 'monotonic_ns': self.clock(), **paired})
                started = self.clock()
                current = {'cycle': number, 'status': 'INCOMPLETE', 'started_monotonic_ns': started}
                if paired is not None:current['paired_barrier'] = paired
                self.cycle_rows.append(current)
                self._batch([(mid, name) for name in pipeline.READS for mid in self.ids],
                            window=4, cycle=number, expected_uids=expected)
                rows = [r for r in self.rows if r['cycle'] == number]
                received = [r['received_monotonic_ns'] for r in rows]
                finished = self.clock()
                current.update(status='COMPLETE', finished_monotonic_ns=finished,
                    requests=len(rows), replies=sum(r['ok'] for r in rows),
                    duration_ms=(finished-started)/1e6,
                    oldest_newest_spread_ms=(max(received)-min(received))/1e6,
                    oldest_request_to_newest_reply_ms=(max(received)-rows[0]['write_started_monotonic_ns'])/1e6,
                    residual=self.residual())
                self.emit({'kind': 'pipeline_cycle', **current})
                current = None
            self.check_budget()
            report['status'] = 'READONLY_PIPELINE_COMPLETE'
        except BaseException as error:
            self.poisoned = True
            report['errors'].append(repr(error))
            for row in self.pending.values(): row.setdefault('error', repr(error))
            if current is not None:
                current.update(error=repr(error), finished_monotonic_ns=self.clock(),
                    requests=sum(r['cycle']==current['cycle'] for r in self.rows),
                    replies=sum(r['cycle']==current['cycle'] and r['ok'] for r in self.rows))
        report.update(identities_verified=self.identities_verified, verified_ids=list(self.ids) if self.identities_verified else [],
            write_attempts=self.write_attempts, replies=self.replies, rx_bytes=self.rx_bytes,
            max_observed_pending=self.max_observed_pending, elapsed_s=(self.clock()-self.started_ns)/1e9,
            pending_at_end=[{'motor_id': k[0], 'parameter': k[1] or 'identity'} for k in self.pending])
        report['receiver_profile'] = self.receiver_profile()
        try:
            report['residual_final'] = self.residual()
            require(not any(report['residual_final'].values()), 'Residual bytes at final boundary')
        except BaseException as error: report['errors'].append(repr(error))
        complete = [c for c in self.cycle_rows if c['status']=='COMPLETE']
        report['cycles_completed'] = len(complete)
        report['statistics_ms'] = {
            'request_wire_RTT': pipeline.distribution([r['wire_round_trip_ms'] for r in self.rows if r['ok']]),
            'cycle_duration': pipeline.distribution([c['duration_ms'] for c in complete]),
            'cycle_oldest_newest_spread': pipeline.distribution([c['oldest_newest_spread_ms'] for c in complete])}
        if report['errors']: report['status'] = 'INCOMPLETE'
        return report


def require_complete(report, scope):
    require(report.get('status')=='READONLY_PIPELINE_COMPLETE' and not report.get('errors')
        and report.get('identities_verified') is True and report.get('verified_ids')==list(SCOPES[scope])
        and report.get('write_attempts')==246 and report.get('replies')==246
        and report.get('cycles_completed')==20 and report.get('pending_at_end')==[]
        and report.get('residual_final')=={'serial_pending_bytes':0,'parser_pending_bytes':0,'parser_discarded_bytes':0},
        'Incomplete scoped246-request collection')
    rows = report.get('requests', [])
    expected = [(0, mid, 'identity') for mid in SCOPES[scope]]+[
        (cycle, mid, name) for cycle in range(1,21) for name in pipeline.READS for mid in SCOPES[scope]]
    require([(r.get('cycle'),r.get('motor_id'),r.get('parameter')) for r in rows]==expected
        and all(r.get('ok') is True and r.get('write_expected_bytes')==17
            and type(r.get('write_returned_bytes')) is int and r['write_returned_bytes']==17 for r in rows),
        'Scoped request rows mismatch')
    require([c.get('cycle') for c in report.get('cycles', [])] == list(range(1,21))
        and all(c.get('status') == 'COMPLETE' and c.get('requests') == 12 and c.get('replies') == 12
            and not any(c.get('residual', {'missing': 1}).values()) for c in report['cycles']),
        'Scoped cycle rows mismatch')


def combined_cycles(front, rear):
    for scope, report in (('front',front),('rear',rear)): require_complete(report,scope)
    result = []
    for number in range(1,21):
        groups = [[r for r in report['requests'] if r['cycle']==number] for report in (front,rear)]
        starts = [min(r['write_started_monotonic_ns'] for r in rows) for rows in groups]
        ends = [max(r['received_monotonic_ns'] for r in rows) for rows in groups]
        result.append({'cycle':number, 'earliest_request_ns':min(starts), 'latest_reply_ns':max(ends),
            'combined_acquisition_span_ms':(max(ends)-min(starts))/1e6,
            'bus_start_skew_ms':abs(starts[0]-starts[1])/1e6,
            'bus_latest_reply_skew_ms':abs(ends[0]-ends[1])/1e6})
    return result


def paired_cycle_statistics(front, rear, combined):
    """Keep gate wait and actual successive acquisition periods visible."""
    result = []
    previous = None
    for a, b, acquisition in zip(front['cycles'], rear['cycles'], combined):
        gates = [row['paired_barrier'] for row in (a,b)]
        first = min(g['arrived_ns'] for g in gates)
        require(gates[0]['released_ns'] == gates[1]['released_ns'], 'Mismatched paired gate release')
        row = {'cycle': acquisition['cycle'], 'first_gate_arrival_ns': first,
            'gate_release_ns': gates[0]['released_ns'],
            'front_gate_wait_ms': gates[0]['wait_ms'], 'rear_gate_wait_ms': gates[1]['wait_ms'],
            'gate_arrival_to_latest_reply_ms': (acquisition['latest_reply_ns']-first)/1e6,
            'paired_request_start_interval_ms': None if previous is None else
                (acquisition['earliest_request_ns']-previous)/1e6}
        result.append(row)
        previous = acquisition['earliest_request_ns']
    return {'cycles': result,
        'front_gate_wait_ms': pipeline.distribution([r['front_gate_wait_ms'] for r in result]),
        'rear_gate_wait_ms': pipeline.distribution([r['rear_gate_wait_ms'] for r in result]),
        'gate_arrival_to_latest_reply_ms': pipeline.distribution([r['gate_arrival_to_latest_reply_ms'] for r in result]),
        'paired_request_start_interval_ms': pipeline.distribution([r['paired_request_start_interval_ms'] for r in result[1:]])}


def collect_dual(bindings, expected_uids, emit_by_scope, max_seconds=10., *,
                 pipeline_factory=ScopedPipelineCAN, common_lock_factory=pipeline.ownership_locks,
                 port_lock_factory=port_lock, check_external=lambda: None, clock=time.monotonic_ns,
                 binding_check=binding_matches, opened_binding_check=opened_binding_matches,
                 join_grace_seconds=1., paired_cycle_sync=False, receive_mode='serial'):
    expected = pipeline.validate_uids(expected_uids)
    plan = make_plan(bindings['front']['path'],bindings['rear']['path'],max_seconds,
                     paired_cycle_sync=paired_cycle_sync, receive_mode=receive_mode)
    require(bindings['front']['resolved']!=bindings['rear']['resolved']
            and bindings['front']['st_rdev']!=bindings['rear']['st_rdev'], 'Duplicate physical port')
    stop, barrier, mutex = threading.Event(), threading.Barrier(2), threading.Lock()
    results, errors = {}, []
    deadline = clock()+int(max_seconds*1e9)
    lease = CommonLease(common_lock_factory)
    cycle_gate = None
    def fail(scope,error):
        with mutex: errors.append({'scope':scope,'error':str(error),'monotonic_ns':clock()})
        stop.set(); barrier.abort()
        if cycle_gate is not None:cycle_gate.abort()
    def check():
        if stop.is_set(): raise InterruptedError('Dual CAN run interrupted')
        check_external()
        if clock() >= deadline: raise TimeoutError('Dual CAN total deadline exhausted')
    def ready():
        check()
        barrier.wait(timeout=max(0.,min(2.,(deadline-clock())/1e9)))
        check()
    if paired_cycle_sync:cycle_gate = PairedCycleGate(check, deadline, clock)
    def worker(scope):
        probe, closed, port_owned, report = None, False, False, None
        try:
            with port_lock_factory(bindings[scope]['resolved']):
                port_owned = True
                check()
                require(binding_check(bindings[scope]), 'Port mapping changed before open')
                options = {'max_seconds':max_seconds, 'clock':clock, 'check_interrupt':check,
                           'identity_barrier':ready, 'receive_mode':receive_mode}
                if cycle_gate is not None:
                    options['cycle_barrier'] = lambda number: cycle_gate.wait(scope, number)
                probe = pipeline_factory(scope,bindings[scope]['path'],emit_by_scope[scope], **options)
                with probe:
                    require(opened_binding_check(probe, bindings[scope]), 'Opened serial device does not match pinned mapping')
                    report = probe.collect(expected)
                    require_complete(report,scope)
                    check()
                closed = True
        except BaseException as error: fail(scope,repr(error))
        finally:
            closed = closed or probe is None or probe.device_closed
            with mutex: results[scope] = {'report':report,'device_closed':closed,'port_lock_acquired':port_owned}
            if closed:
                try: lease.release()
                except BaseException as error: fail(scope,'Common lease release: '+repr(error))
            else:
                _HELD_LEASES.append(lease)
    threads = []
    try:
        for scope in ('front','rear'):
            lease.retain()
            thread = threading.Thread(target=worker,args=(scope,),name='dual-can-'+scope,daemon=True)
            try: thread.start()
            except BaseException:
                lease.release()
                raise
            threads.append(thread)
        for thread in threads: thread.join(timeout=max(0.,(deadline-clock())/1e9))
        if any(t.is_alive() for t in threads):
            fail('coordinator','Worker exceeded finite acquisition budget')
            for thread in threads: thread.join(timeout=join_grace_seconds)
    except BaseException as error: fail('coordinator',repr(error))
    finally:
        stop.set(); barrier.abort()
        if cycle_gate is not None:cycle_gate.abort()
        for thread in threads:
            if thread.is_alive(): thread.join(timeout=join_grace_seconds)
        try: lease.release()
        except BaseException as error: fail('coordinator','Common lease release: '+repr(error))
        if any(t.is_alive() for t in threads):
            _HELD_LEASES.append(lease)
            fail('coordinator','Actual worker/port termination unconfirmed; common lease retained')
    with mutex: frozen_results, frozen_errors = copy.deepcopy(results), copy.deepcopy(errors)
    report = {**FLAGS,'status':'INCOMPLETE','plan':plan,'buses':frozen_results,'errors':frozen_errors,
        'producer_threads_exited':all(not t.is_alive() for t in threads),
        'common_lock_released':lease.released,'same_cycle':[]}
    if not frozen_errors and report['producer_threads_exited'] and lease.released and set(results)==set(SCOPES):
        report['same_cycle'] = combined_cycles(results['front']['report'],results['rear']['report'])
        report['combined_span_ms'] = pipeline.distribution([r['combined_acquisition_span_ms'] for r in report['same_cycle']])
        if paired_cycle_sync:
            report['paired_cycle_timing'] = paired_cycle_statistics(results['front']['report'],
                results['rear']['report'],report['same_cycle'])
        report['status'] = 'COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT'
    return report


class EventBuffer:
    """One producer owns this bounded list; the coordinator flushes after join.

    The fixed pipeline emits bounded schemas (raw chunks <=4096 bytes). Retain
    independent row values and original source timestamps without JSON or disk
    work in the acquisition path. No buffer is flushed while a worker survives.
    """
    def __init__(self, scope):
        self.scope, self.rows, self.sealed = scope, [], False

    def emit(self, row):
        require(not self.sealed, 'Audit buffer already sealed')
        require(len(self.rows) < MAX_EVENTS_PER_BUS, 'Finite audit event budget exhausted')
        self.rows.append({'scope': self.scope, 'wall_time_ns': time.time_ns(), **snapshot_event(row)})

    def flush(self, path):
        require(not self.sealed, 'Audit buffer already sealed')
        self.sealed = True
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            for row in self.rows:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
            os.fsync(stream.fileno())
        return len(self.rows)


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-no-output',action='store_true')
    for name in ('expected-uids','front-port','rear-port','output'):ap.add_argument('--'+name,type=Path,required=True)
    ap.add_argument('--max-seconds',type=float,default=10.)
    ap.add_argument('--receive-mode',choices=('serial','select'),default='serial',
                    help='Receive with existing serial timeout or absolute-deadline select')
    ap.add_argument('--paired-cycle-sync',action='store_true',help='Explicit diagnostic paired-cycle start gate; default remains independent')
    args=ap.parse_args(argv)
    try:
        plan=make_plan(args.front_port,args.rear_port,args.max_seconds,
                       paired_cycle_sync=args.paired_cycle_sync,receive_mode=args.receive_mode)
        if not args.execute_no_output:print(json.dumps(plan,indent=2));return 0
        from . import policy_shadow as shadow
        from . import policy_observer_replay as replay
        raw=args.expected_uids.read_bytes();expected=pipeline.validate_uids(shadow._json(raw))
        bindings=validate_ports(args.front_port,args.rear_port)
        output=replay._output_path(args.output)
        require(Path(__file__).resolve().parent.parent not in output.parents,'Output must be outside runtime')
        output.mkdir(mode=0o700,exist_ok=False)
    except (OSError,ValueError) as error:ap.error(str(error))
    handlers,buffers,counts,flushed={},{},{},{}
    signals=[];guard=None
    report={**FLAGS,'status':'INCOMPLETE','plan':plan,'bindings':bindings,
        'expected_uid_file_sha256':hashlib.sha256(raw).hexdigest(),
        'started_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
    try:
        for number in (signal.SIGINT,signal.SIGTERM):handlers[number]=signal.signal(number,lambda sig,_:signals.append(sig))
        guard=BootIdentityGuard();report['boot_id']=guard.boot_id
        def check():
            if signals:raise InterruptedError('signal '+str(signals[0]))
            guard.check()
        buffers={scope:EventBuffer(scope) for scope in SCOPES}
        # An unexpected escape must not imply that unknown worker lifetimes ended.
        report['producer_threads_exited']=False
        report.update(collect_dual(bindings,expected,{s:b.emit for s,b in buffers.items()},
                                  args.max_seconds,check_external=check,paired_cycle_sync=args.paired_cycle_sync,
                                  receive_mode=args.receive_mode))
    except BaseException as error:report.update(status='INCOMPLETE',failure=repr(error))
    finally:
        try:
            # Unknown worker lifetimes never permit freezing a mutable buffer.
            alive=report.get('producer_threads_exited') is False
            for scope,buffer in buffers.items():
                flushed[scope]=False
                counts[scope]=0
                if not alive:
                    try:
                        counts[scope]=buffer.flush(output/('events-'+scope+'.jsonl'))
                        flushed[scope]=True
                    except BaseException as error:report.update(status='INCOMPLETE',cleanup_error=repr(error))
            if guard is not None:
                try:guard.close()
                except BaseException as error:report.update(status='INCOMPLETE',boot_close_error=repr(error))
            report.update(signals=signals,events_written=counts,events_flush_confirmed=flushed,
                events_captured={s:len(b.rows) for s,b in buffers.items()},
                buffer_captures_final=not alive,
                completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
            report['events_sha256']={s:hashlib.sha256((output/('events-'+s+'.jsonl')).read_bytes()).hexdigest()
                for s in buffers if flushed[s]}
            report['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (Path(__file__),Path(pipeline.__file__),Path(__file__).with_name('can_readonly.py'),
                 Path(__file__).with_name('can_timing_probe.py'),Path(__file__).with_name('sensor_pipeline_benchmark.py'),
                 Path(__file__).with_name('event_snapshot.py'))}
            reader_source=Path(__file__).with_name('serial_deadline_reader.py')
            report['source_sha256'][reader_source.name]=hashlib.sha256(reader_source.read_bytes()).hexdigest()
            if signals or set(flushed)!=set(SCOPES) or not all(flushed.values()):report['status']='INCOMPLETE'
            fd=os.open(output/'summary.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            with os.fdopen(fd,'w',encoding='utf-8') as stream:
                json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
            fd=os.open(output,os.O_RDONLY)
            try:os.fsync(fd)
            finally:os.close(fd)
        finally:
            for number,handler in handlers.items():signal.signal(number,handler)
    print(json.dumps({'status':report['status'],'output':str(output),
                     'combined_span_ms':report.get('combined_span_ms'),**FLAGS}))
    return 0 if report['status']=='COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT' else 2


if __name__=='__main__':raise SystemExit(main())
