"""Finite live sensor-to-policy diagnostic; motor output does not exist.

One ReadOnlyCAN owner issues only Type0 identities and Type17 position/velocity
reads. A separate ICM20948 owner initializes/restores the existing IMU reader.
20ms model ticks consume original source times plus queue availability times.
Any blocked/late tick terminates the run: no catchup, fill, clamp or retry.
Diagnostic age limits and a completed run never authorize motor output.
"""
import argparse
import copy
from contextlib import contextmanager
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import threading
import time

from . import can_readonly as codec
from . import can_timing_probe as timing
from . import imu
from . import policy_observer as observer
from . import policy_observer_replay as replay
from . import policy_shadow as shadow
from . import telemetry_snapshot as telemetry
from .event_snapshot import snapshot_event

DT_NS = observer.DT_NS
STARTUP_NS = 2_000_000_000
MAX_CAN_QUERIES = 5000
FLAGS = dict(motor_output_available=False, output_allowed=False, approved_for_runtime=False,
             live_50hz_verified=False, calibration_verified=False, h_measured=False)


class SessionStopped(Exception):
    """Expected cooperative cancellation, distinct from device/cleanup failures."""


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def diagnostic_hypotheses(values):
    require(type(values) in (list, tuple) and all(type(h) is int for h in values)
            and tuple(values) in ((0, 1), (0,), (1,)),
            'Diagnostic hypotheses must be [0,1], [0] or [1]')
    return tuple(values)


def plan_for(*, max_ticks, max_seconds, max_age_ns, max_spread_ns, max_lateness_ns,
             hypotheses=(0, 1), profile_consume=False):
    hypotheses = diagnostic_hypotheses(hypotheses)
    require(type(profile_consume) is bool, 'profile_consume must be boolean')
    require(type(max_ticks) is int and 1 <= max_ticks <= 500, 'max_ticks must be1..500')
    require(type(max_seconds) in (int, float) and 1 <= max_seconds <= 10,
            'max_seconds must be finite and1..10')
    require(max_ticks*DT_NS <= max_seconds*1e9, 'Tick budget exceeds total acquisition budget')
    for value in (max_age_ns, max_spread_ns):
        require(type(value) is int and 0 < value <= 1_000_000_000, 'Diagnostic age/spread must be>0 and<=1000ms')
    require(type(max_lateness_ns) is int and 0 <= max_lateness_ns < DT_NS,
            'Start lateness must be nonnegative and less than20ms')
    return {**FLAGS, 'diagnostic_hypotheses': list(hypotheses),
            'hypothesis_scope': 'fixed load-history sensitivity; not a production h estimator',
            'profile_consume': profile_consume,
            'ids': list(range(1, 13)), 'allowed_can_types': [0, 17],
            'continuous_parameters': ['position', 'velocity'], 'one_can_owner': True,
            'serial': {'port': '/dev/robstride-usb2can', 'baudrate': 921600,
                       'exclusive': True, 'dtr': False, 'rts': False},
            'identity_checks_before_telemetry': 12, 'dt_ns': DT_NS, 'max_ticks': max_ticks,
            'max_seconds': max_seconds, 'startup_timeout_ns': STARTUP_NS,
            'max_age_ns': max_age_ns, 'max_spread_ns': max_spread_ns,
            'max_start_lateness_ns': max_lateness_ns, 'max_tick_execution_window_ns': DT_NS,
            'max_can_queries': MAX_CAN_QUERIES, 'input_queue_capacity': 2048,
            'log_queue_capacity': 8192, 'automatic_retry': False, 'catchup_available': False,
            'imu_configuration_written_and_restored': True, 'pure_hardware_readonly': False,
            'limits_are_diagnostic_only': True, 'command': [0., 0., 0.]}


class SessionBus:
    """Bounded queues. Insertion time is availability, never a replacement sensor time."""
    def __init__(self, *, clock=time.monotonic_ns, input_capacity=2048, log_capacity=8192):
        self.clock = clock
        self.inputs, self.logs = queue.Queue(input_capacity), queue.Queue(log_capacity)
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.errors = []
        self.dropped_events = 0
        self.pending = None
        self.producer_status = {}

    def fail(self, component, error):
        with self.lock:
            self.errors.append({'component': component, 'error': str(error), 'monotonic_ns': self.clock()})
            self.stop.set()

    def publish(self, event, *, telemetry_input=False):
        with self.lock:
            try:
                row = {'wall_time_ns': time.time_ns(), **snapshot_event(event)}
            except (TypeError, ValueError) as error:
                self.fail('event_snapshot', str(error))
                raise
            try:
                self.logs.put_nowait(row)
                if telemetry_input and not self.stop.is_set():
                    self.inputs.put_nowait(row)
            except queue.Full:
                self.dropped_events += 1
                self.fail('queue', 'Bounded queue overflow; no dropping/catchup continuation')
                raise RuntimeError('Bounded queue overflow')
            finally:
                # Both consumers take this lock before inspecting the committed row.
                # In particular, preemption during either queue insertion cannot
                # backdate availability to before the input actually arrived.
                row['available_monotonic_ns'] = self.clock()

    def available(self, tick_ns):
        # Ordered under publish's lock across both producers. Future arrivals stay queued.
        while True:
            with self.lock:
                if self.pending is None:
                    try:
                        self.pending = self.inputs.get_nowait()
                    except queue.Empty:
                        return
                if self.pending['available_monotonic_ns'] > tick_ns:
                    return
                item, self.pending = self.pending, None
            yield item

    def check(self, deadline_ns, check_external=lambda: None):
        check_external()
        if self.stop.is_set():
            raise SessionStopped('Producer/session stopped: '+str(self.errors[:1]))
        if self.clock() >= deadline_ns:
            raise TimeoutError('Finite acquisition deadline reached')


class AuditWriter:
    """File I/O cannot silently delay sensor reads or fill an unbounded backlog."""
    def __init__(self, path, bus):
        self.path, self.bus = Path(path), bus
        self.done = threading.Event()
        self.flushed = False
        self.written = 0
        self.thread = threading.Thread(target=self._run, name='observer-log', daemon=True)
        self.descriptor = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)

    def start(self):
        try:
            self.thread.start()
        except BaseException:
            if self.thread.ident is None:
                os.close(self.descriptor)
                self.descriptor = None
            raise

    def _run(self):
        try:
            with os.fdopen(self.descriptor, 'w', encoding='utf-8') as stream:
                while not self.done.is_set() or not self.bus.logs.empty():
                    try:
                        row = self.bus.logs.get(timeout=.02)
                    except queue.Empty:
                        continue
                    with self.bus.lock:
                        row = snapshot_event(row)
                    stream.write(json.dumps(row, allow_nan=False)+'\n')
                    self.written += 1
                stream.flush()
                os.fsync(stream.fileno())
                self.flushed = True
        except BaseException as error:
            self.bus.fail('log_writer', repr(error))

    def close(self):
        self.done.set()
        if self.thread.ident is None:
            if self.descriptor is not None:
                os.close(self.descriptor)
                self.descriptor = None
            return
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            self.bus.fail('log_writer', 'Writer termination/fsync unconfirmed')


def _can_event(bus, event):
    is_input = event.get('kind') == 'motor_parameter' and event.get('parameter') in ('position', 'velocity')
    bus.publish(event, telemetry_input=is_input)
    if event.get('kind') == 'motor_feedback' and (event.get('type') == 21
            or event.get('fault_bits', 0) != 0 or event.get('mode_state', 0) != 0):
        raise RuntimeError('Contradictory fault/enabled feedback during no-output diagnostic')


def can_producer(bus, expected_uids, deadline_ns, *, can_factory=codec.ReadOnlyCAN,
                 lock_factory=timing.ownership_locks, check_external=lambda: None):
    queries = 0
    context_exited = False
    locks_acquired = False
    def check():
        bus.check(deadline_ns, check_external)
        require(queries < MAX_CAN_QUERIES, 'CAN query budget exhausted')
    try:
        # The device owner holds both cross-process locks through serial close,
        # including when the main scheduler's bounded join cannot finish.
        with lock_factory():
            locks_acquired = True
            with can_factory(event_sink=lambda e: _can_event(bus, e)) as can:
                try:
                    for mid in range(1, 13):
                        check(); queries += 1
                        reply = can.query(mid)
                        require(not can.parser.buffer and not can.parser.discarded_bytes, 'CAN parser not clean')
                        require(reply.get('ok') is True and reply.get('mcu_uid_hex') == expected_uids[str(mid)],
                                f'ID{mid} fresh identity mismatch')
                    bus.publish({'kind': 'identity_all12_verified', 'ids': list(range(1, 13))}, telemetry_input=True)
                    while not bus.stop.is_set():
                        for mid in range(1, 13):
                            for name in ('position', 'velocity'):
                                check(); queries += 1
                                reply = can.query(mid, name)
                                require(reply.get('ok') is True, f'Invalid ID{mid} {name} reply')
                                require(not can.parser.buffer and not can.parser.discarded_bytes, 'CAN parser not clean')
                except SessionStopped:
                    pass
            context_exited = True
    except BaseException as error:
        bus.fail('can', repr(error))
    finally:
        with bus.lock:
            bus.producer_status['can'] = {'exited': True, 'queries': queries,
                'device_context_exited': context_exited, 'cross_process_locks_acquired': locks_acquired}


@contextmanager
def imu_ownership_lock():
    with open('/tmp/singularitydog-imu-i2c7-68.lock', 'a+') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def imu_producer(bus, deadline_ns, *, imu_factory=imu.ICM20948,
                 lock_factory=imu_ownership_lock, check_external=lambda: None):
    device = None
    restore = 'not_started'
    try:
        with lock_factory():
            bus.check(deadline_ns, check_external)
            device = imu_factory()
            try:
                configuration = device.start()
                bus.publish({'kind': 'imu_configured', 'configuration': configuration,
                             'original_registers': device.original_registers})
                last = bus.clock()
                while not bus.stop.is_set():
                    bus.check(deadline_ns, check_external)
                    sample = device.read_sample()
                    if sample is not None:
                        last = bus.clock()
                        bus.publish({'kind': 'imu', **sample}, telemetry_input=True)
                    elif bus.clock()-last > 100_000_000:
                        raise TimeoutError('No new IMU sample for100ms')
                    # Latest-value polling, no burst catchup or invented samples.
                    bus.stop.wait(.01)
            except SessionStopped:
                pass
            finally:
                try:
                    device.close()
                finally:
                    restore = device.restore_status
                    bus.publish({'kind': 'imu_restore', 'restore_status': restore})
                require(restore in ('restored', 'not_needed'), 'IMU register restoration unconfirmed')
    except SessionStopped:
        pass
    except BaseException as error:
        bus.fail('imu', repr(error))
    finally:
        with bus.lock:
            bus.producer_status['imu'] = {'exited': True, 'restore_status': restore}


class LiveInputs:
    """Single scheduler thread owns the existing non-thread-safe snapshot buffer."""
    def __init__(self, plan):
        self.buffer = telemetry.TelemetrySnapshotBuffer(history_per_key=64,
            max_age_ns=plan['max_age_ns'], max_spread_ns=plan['max_spread_ns'])
        self.identities_verified = False
        self.last_imu_sequence = 0

    def ingest(self, row):
        available = row['available_monotonic_ns']
        if row['kind'] == 'identity_all12_verified':
            require(not self.identities_verified and row['ids'] == list(range(1, 13)), 'Invalid identity barrier')
            self.identities_verified = True
        elif row['kind'] == 'motor_parameter':
            require(self.identities_verified, 'Telemetry before twelve verified identities')
            require(row.get('ok') is True and row.get('status') == 0, 'Rejected telemetry')
            require(row['request_monotonic_ns'] <= row['monotonic_ns'] <= available, 'Future motor source timestamp')
            self.buffer.ingest_motor(can_type=17, motor_id=row['motor_id'], parameter=row['parameter'],
                value=row['value'], unit=row['unit'], request_ns=row['request_monotonic_ns'],
                received_ns=row['monotonic_ns'])
        elif row['kind'] == 'imu':
            require(row.get('frame') == 'sensor' and type(row.get('sequence')) is int
                    and row['sequence'] == self.last_imu_sequence+1, 'Invalid/missing IMU sequence or frame')
            require(row['read_started_monotonic_ns'] <= row['read_finished_monotonic_ns'] <= available,
                    'Future IMU source timestamp')
            self.buffer.ingest_imu(accel_m_s2=row['accel_m_s2'], gyro_rad_s=row['gyro_rad_s'],
                read_started_ns=row['read_started_monotonic_ns'], read_finished_ns=row['read_finished_monotonic_ns'])
            self.last_imu_sequence = row['sequence']
        else:
            raise ValueError('Unexpected telemetry event')

    def ready(self):
        return self.identities_verified and all(self.buffer.history_sizes().values())


def prepare_observers(policies, calibration, mount, bias, plan, torch_module, warmup_ticks=3):
    hypotheses = diagnostic_hypotheses(plan.get('diagnostic_hypotheses', [0, 1]))
    require(len(policies) == len(hypotheses) and len({id(p) for p in policies}) == len(policies),
            'Require one independent policy per diagnostic hypothesis')
    runs = []
    profile = {'profile_consume': True} if plan.get('profile_consume', False) else {}
    for h, policy in zip(hypotheses, policies):
        replay.warmup_policy(policy, torch_module, h, warmup_ticks)
        runs.append(observer.StatefulPolicyObserver(policy, calibration, imu_mount_candidate=mount,
            gyro_bias_candidate=bias, h_hypothesis=h, command=[0., 0., 0.],
            max_ticks=plan['max_ticks'], max_age_ns=plan['max_age_ns'],
            max_spread_ns=plan['max_spread_ns'], torch_module=torch_module, **profile))
    return runs


def observe_live(bus, runs, plan, deadline_ns, *, check_external=lambda: None, wait=None):
    """Inject clock/bus/wait for deterministic offline scheduling tests."""
    wait = bus.stop.wait if wait is None else wait
    inputs = LiveInputs(plan)
    started = bus.clock()
    first_tick = None
    blocked = None
    tick_index = None
    try:
        hypotheses = diagnostic_hypotheses(plan.get('diagnostic_hypotheses', [0, 1]))
        require(len(runs) == len(hypotheses)
                and [r.summary()['h_hypothesis'] for r in runs] == list(hypotheses),
                'Observers do not match the declared diagnostic hypotheses')
        while not inputs.ready():
            bus.check(min(deadline_ns, started+STARTUP_NS), check_external)
            now = bus.clock()
            for row in bus.available(now): inputs.ingest(row)
            if not inputs.ready(): wait(.001)
        # Reset can allocate/JIT-initialize policy state. Do not spend the first
        # tick's execution budget on this one-time, explicitly timed preparation.
        for h, run in zip(hypotheses, runs):
            bus.check(deadline_ns, check_external)
            begin = bus.clock()
            bus.publish({'kind': 'live_policy_reset_begin', 'h_hypothesis': h,
                         'monotonic_ns': begin})
            run.prepare_run(warmup_completed=True)
            end = bus.clock()
            bus.publish({'kind': 'live_policy_reset_end', 'h_hypothesis': h,
                         'monotonic_ns': end, 'reset_duration_ns': end-begin})
            bus.check(deadline_ns, check_external)
        arm = bus.clock()
        first_tick = arm+DT_NS
        for run in runs: run.arm_run(first_tick)
        bus.publish({'kind': 'live_schedule_armed', 'monotonic_ns': arm,
                     'first_tick_ns': first_tick})
        for tick_index in range(plan['max_ticks']):
            tick = first_tick+tick_index*DT_NS
            while bus.clock() < tick:
                bus.check(deadline_ns, check_external)
                wait(min(.01, (tick-bus.clock())/1e9))
            bus.check(deadline_ns, check_external)
            actual = bus.clock()
            bus.publish({'kind': 'live_tick_start_check', 'tick_index': tick_index,
                         'scheduled_tick_ns': tick, 'monotonic_ns': actual,
                         'start_lateness_ns': actual-tick})
            require(0 <= actual-tick <= plan['max_start_lateness_ns'], '20ms tick start deadline missed; no catchup')
            for row in bus.available(tick): inputs.ingest(row)
            snapshot = inputs.buffer.snapshot(tick).as_dict()
            snapshot['source_flags'] = {'producer_identity_all12_verified': inputs.identities_verified,
                                        'source_timestamps_changed': False, 'queue_availability_enforced': True}
            oldest = snapshot['oldest_observation_age_ns']
            actual_age = None if oldest is None else oldest+bus.clock()-tick
            if snapshot['status'] != 'DIAGNOSTIC_READY':
                blocked = {'tick_index': tick_index, 'tick_ns': tick, 'actual_started_ns': actual,
                           'reason': '; '.join(snapshot['blocked_reasons']), 'snapshot': snapshot}
                raise ValueError('Telemetry snapshot blocked')
            require(actual_age <= plan['max_age_ns'], 'Input age at actual processing time exceeds diagnostic limit')
            for h, run in zip(hypotheses, runs):
                bus.check(deadline_ns, check_external)
                begin = bus.clock()
                require(begin-tick < DT_NS, 'Tick execution window exhausted before policy')
                require(oldest+begin-tick <= plan['max_age_ns'], 'Input became stale before policy')
                result = run.consume(snapshot)
                end = bus.clock()
                bus.publish({'kind': 'live_policy_tick', 'h_hypothesis': h, 'tick_index': tick_index,
                             'scheduled_tick_ns': tick, 'actual_started_ns': actual,
                             'start_lateness_ns': actual-tick, 'inference_started_ns': begin,
                             'inference_finished_ns': end, 'inference_duration_ns': end-begin,
                             'actual_oldest_observation_age_ns': oldest+begin-tick, 'result': result})
                bus.check(deadline_ns, check_external)
                require(end < tick+DT_NS, 'Policy inference exceeded20ms tick window; no catchup')
            require(bus.clock() < tick+DT_NS, 'Tick logging/processing exceeded20ms window; no catchup')
        bus.check(deadline_ns, check_external)
        for run in runs: run.finish()
    except BaseException as error:
        if blocked is None:
            blocked = {'tick_index': tick_index, 'first_tick_ns': first_tick,
                       'monotonic_ns': bus.clock(), 'reason': type(error).__name__+': '+str(error)}
        for run in runs: run.invalidate(blocked['reason'])
        bus.fail('observer', blocked['reason'])
        try: bus.publish({'kind': 'live_observer_blocked', **blocked})
        except BaseException: pass
    finally:
        bus.stop.set()
    return {**FLAGS, 'status': 'COMPLETE_NO_OUTPUT_DIAGNOSTIC' if blocked is None else 'INCOMPLETE',
            'diagnostic_hypotheses': plan.get('diagnostic_hypotheses', [0, 1]),
            'first_tick_ns': first_tick, 'first_blocked_tick': blocked,
            'hypotheses': [r.summary() for r in runs], 'started_monotonic_ns': started,
            'finished_monotonic_ns': bus.clock(), 'identity_all12_verified': inputs.identities_verified,
            'source_timestamps_changed': False, 'wall_clock_schedule_measured': True}


def run_acquisition(bus, runs, calibration, plan, *, check_external=lambda: None,
                    can_factory=codec.ReadOnlyCAN, imu_factory=imu.ICM20948,
                    can_lock_factory=timing.ownership_locks, imu_lock_factory=imu_ownership_lock):
    deadline = bus.clock()+int(plan['max_seconds']*1e9)
    threads = [threading.Thread(target=can_producer, name='observer-can', daemon=True,
                   args=(bus, calibration['identities'], deadline),
                   kwargs={'can_factory': can_factory, 'lock_factory': can_lock_factory,
                           'check_external': check_external}),
               threading.Thread(target=imu_producer, name='observer-imu', daemon=True,
                   args=(bus, deadline), kwargs={'imu_factory': imu_factory,
                       'lock_factory': imu_lock_factory, 'check_external': check_external})]
    result = {**FLAGS, 'status': 'INCOMPLETE', 'first_blocked_tick': None}
    try:
        for thread in threads: thread.start()
        result = observe_live(bus, runs, plan, deadline, check_external=check_external)
    finally:
        bus.stop.set()
        for thread in threads:
            if thread.ident is not None: thread.join(timeout=1.)
        for thread in threads:
            if thread.is_alive(): bus.fail(thread.name, 'Producer termination unconfirmed; kernel I/O may be blocked')
    result['producer_status'] = copy.deepcopy(bus.producer_status)
    result['producer_threads_exited'] = all(not thread.is_alive() for thread in threads)
    if bus.errors or not result['producer_threads_exited']:
        result['status'] = 'INCOMPLETE'
    return result


def _sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY)
    try: os.fsync(descriptor)
    finally: os.close(descriptor)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-no-output', action='store_true')
    for key in ('calibration', 'bundle', 'imu-mount-candidate', 'output'):
        ap.add_argument('--'+key, type=Path, required=True)
    ap.add_argument('--gyro-bias-candidate', type=Path)
    ap.add_argument('--max-age-ms', type=replay._milliseconds, required=True)
    ap.add_argument('--max-spread-ms', type=replay._milliseconds, required=True)
    ap.add_argument('--max-lateness-ms', type=replay._milliseconds, default=5_000_000)
    ap.add_argument('--max-ticks', type=int, default=100)
    ap.add_argument('--max-seconds', type=float, default=10.)
    ap.add_argument('--warmup-ticks', type=int, default=3)
    ap.add_argument('--hypothesis', choices=('both', '0', '1'), default='both',
                    help='Fixed diagnostic h only; single-h timing does not approve production inputs')
    ap.add_argument('--profile-consume', action='store_true',
                    help='Measure input/model/result stages; excludes acquisition and CAN output')
    args = ap.parse_args(argv)
    try:
        plan = plan_for(max_ticks=args.max_ticks, max_seconds=args.max_seconds,
            max_age_ns=args.max_age_ms, max_spread_ns=args.max_spread_ms,
            max_lateness_ns=args.max_lateness_ms,
            hypotheses=(0, 1) if args.hypothesis == 'both' else (int(args.hypothesis),),
            profile_consume=args.profile_consume)
        require(1 <= args.warmup_ticks <= 100, 'warmup_ticks must be1..100')
        if not args.execute_no_output:
            print(json.dumps(plan, indent=2)); return 0
        output = replay._output_path(args.output)
        require(Path(__file__).resolve().parent.parent not in output.parents, 'Output must be outside runtime')
        output.mkdir(mode=0o700, exist_ok=False)
        _sync_directory(output.parent)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    bus = SessionBus()
    writer = None
    handlers, signals = {}, []
    report = {**FLAGS, 'kind': 'live_policy_observer', 'status': 'INCOMPLETE', 'plan': plan,
              'started_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'source_sha256': {Path(m.__file__).name: shadow.sha(m.__file__) for m in
                  (codec, timing, imu, observer, replay, shadow, telemetry)},
              'limitations': ['Diagnostic freshness limits do not authorize output.',
                  'Read intervals are host times, not synchronized sensor conversion times.',
                  'Kernel I/O cannot be force-cancelled by the existing drivers; failed joins remain incomplete.',
                  'Zero policy command does not mean fixed posture or motor stop.',
                  'h0/h1 are separate load-history hypotheses, not measured load.']}
    report['source_sha256'][Path(__file__).name] = shadow.sha(__file__)
    snapshot_source = Path(__file__).with_name('event_snapshot.py')
    report['source_sha256'][snapshot_source.name] = shadow.sha(snapshot_source)
    phase = 'input_validation'
    try:
        for number in (signal.SIGINT, signal.SIGTERM):
            handlers[number] = signal.signal(number, lambda sig, _: (signals.append(sig), bus.stop.set()))
        boot_path = Path('/proc/sys/kernel/random/boot_id')
        boot = boot_path.read_text().strip()
        report['boot_id'] = boot
        def check_external():
            if signals: raise InterruptedError('signal '+str(signals[0]))
            if boot_path.read_text().strip() != boot: raise RuntimeError('Boot changed')
        calibration_raw = args.calibration.read_bytes()
        mount_raw = args.imu_mount_candidate.read_bytes()
        bias_raw = args.gyro_bias_candidate.read_bytes() if args.gyro_bias_candidate else None
        calibration = shadow._json(calibration_raw); shadow.validate_calibration(calibration)
        mount = shadow.validate_imu_mount_candidate(shadow._json(mount_raw))
        bias = shadow._json(bias_raw) if bias_raw is not None else None
        report['input_sha256'] = {'calibration': hashlib.sha256(calibration_raw).hexdigest(),
            'imu_mount_candidate': hashlib.sha256(mount_raw).hexdigest(),
            'gyro_bias_candidate': hashlib.sha256(bias_raw).hexdigest() if bias_raw is not None else None}
        phase = 'model_loading_and_warmup'
        loaded = [shadow.load_policy(args.bundle) for _ in plan['diagnostic_hypotheses']]
        report['model_sources'] = [x[1] for x in loaded]
        import torch
        runs = prepare_observers([x[0] for x in loaded], calibration, mount, bias, plan, torch, args.warmup_ticks)
        report['warmup'] = {'kind': 'synthetic_before_hardware', 'ticks_per_hypothesis': args.warmup_ticks}
        phase = 'acquisition'
        writer = AuditWriter(output/'events.jsonl', bus); writer.start()
        bus.publish({'kind': 'live_observer_plan', 'plan': plan})
        check_external()
        report.update(run_acquisition(bus, runs, calibration, plan, check_external=check_external))
    except BaseException as error:
        report.update(status='INCOMPLETE', failure_phase=phase, failure=type(error).__name__+': '+str(error))
        bus.fail(phase, report['failure'])
    finally:
        bus.stop.set()
        try:
            if writer is not None: writer.close()
            if not (output/'events.jsonl').exists():
                descriptor = os.open(output/'events.jsonl', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                os.fsync(descriptor); os.close(descriptor)
            report.update(errors=bus.errors, signals=signals, dropped_events=bus.dropped_events,
                log_flush_confirmed=bool(writer and writer.flushed),
                events_sha256=shadow.sha(output/'events.jsonl') if writer is None or not writer.thread.is_alive() else None,
                completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
            if bus.errors or signals or not report['log_flush_confirmed']:
                report['status'] = 'INCOMPLETE'
            descriptor = os.open(output/'summary.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
                stream.flush(); os.fsync(stream.fileno())
            _sync_directory(output)
        finally:
            for number, handler in handlers.items(): signal.signal(number, handler)
    print(json.dumps({'output': str(output), 'status': report['status'],
                      'first_blocked_tick': report.get('first_blocked_tick'), **FLAGS}))
    return 0 if report['status'] == 'COMPLETE_NO_OUTPUT_DIAGNOSTIC' else 2


if __name__ == '__main__':
    raise SystemExit(main())
