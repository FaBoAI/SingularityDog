"""Finite CAN+IMU sensor benchmark; no model, calibration or motor output.

The sole CAN owner uses the existing canonical read-only pipeline unchanged:
12 identities, then20 cycles of24 parameter reads, window4/gap0.5ms/by-parameter.
20ms observations measure source age/spread and timestamp updates, not motion.
Completion never establishes50Hz complete inputs or permits motor control.
"""
import argparse
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import uuid

from . import can_pipeline_probe as pipeline
from . import policy_observer_live as live

DT_NS = 20_000_000
STARTUP_NS = 2_000_000_000
AGE_NS = SPREAD_NS = 100_000_000
LATENESS_NS = 5_000_000
CYCLES = 20
MAX_REQUESTS = 492
FLAGS = {**live.FLAGS, 'calibration_applied': False, 'policy_loaded': False,
         'inference_calls': 0, 'full_controller_50Hz_verified': False}


class BootIdentityGuard:
    """Fresh pread at every check, with one owned descriptor and no caching."""
    def __init__(self, path='/proc/sys/kernel/random/boot_id'):
        self._lock = threading.Lock()
        self._fd = None
        try:
            self._fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC)
            self._expected = os.pread(self._fd, 80, 0).strip()
            self.boot_id = self._expected.decode('ascii')
            live.require(str(uuid.UUID(self.boot_id)) == self.boot_id,
                         'Invalid canonical boot UUID')
        except BaseException:
            self.close()
            raise

    def check(self):
        # close shares this lock so a late worker can never read a reused fd.
        with self._lock:
            live.require(self._fd is not None, 'Boot monitor already closed')
            live.require(os.pread(self._fd, 80, 0).strip() == self._expected,
                         'Boot changed or invalid boot read')

    def close(self):
        with self._lock:
            if self._fd is not None:
                try: os.close(self._fd)
                finally: self._fd = None


def make_plan(max_seconds=10.):
    live.require(type(max_seconds) in (int, float) and math.isfinite(max_seconds)
                 and 1 <= max_seconds <= 10, 'max_seconds must be finite and1..10')
    return {**FLAGS, 'kind': 'sensor_pipeline_benchmark', 'max_seconds': max_seconds,
            'can': pipeline.make_plan(4, .5, CYCLES, max_seconds, 'by-parameter'),
            'dt_ns': DT_NS, 'max_age_ns': AGE_NS, 'max_spread_ns': SPREAD_NS,
            'max_start_lateness_ns': LATENESS_NS, 'startup_timeout_ns': STARTUP_NS,
            'automatic_retry': False, 'catchup_available': False,
            'imu_configuration_written_and_restored': True,
            'pure_hardware_readonly': False,
            'update_measure': 'per-key source timestamp changes, not value changes'}


class PipelineEvents:
    """Preserve raw pipeline rows and causally bridge already validated replies."""
    def __init__(self, bus, expected_uids):
        self.bus = bus
        self.expected = pipeline.validate_uids(expected_uids)
        self.identities = set()
        self.checker = live.telemetry.TelemetrySnapshotBuffer(
            history_per_key=1, max_age_ns=AGE_NS, max_spread_ns=SPREAD_NS)

    def __call__(self, event):
        self.bus.publish(event)
        if event.get('kind') != 'pipeline_reply':
            return
        result = event.get('result', {})
        mid, name = event.get('motor_id'), event.get('parameter')
        live.require(type(mid) is int and mid in self.expected
                     and result.get('motor_id') == mid and result.get('parameter') == name
                     and event.get('ok') is True and result.get('ok') is True,
                     'Pipeline reply/result identity mismatch')
        if name == 'identity':
            live.require(not self.identities == set(range(1, 13)) and mid not in self.identities
                         and event.get('cycle') == 0
                         and result.get('mcu_uid_hex') == self.expected[mid],
                         'Duplicate or mismatched fresh identity')
            self.identities.add(mid)
            if len(self.identities) == 12:
                self.bus.publish({'kind': 'identity_all12_verified', 'ids': list(range(1, 13))},
                                 telemetry_input=True)
            return
        live.require(len(self.identities) == 12, 'Telemetry before twelve fresh identities')
        live.require(type(event.get('cycle')) is int and 1 <= event['cycle'] <= CYCLES,
                     'Invalid pipeline telemetry cycle')
        self.checker.ingest_pipeline_reply(event)
        # The adapter above independently validates ID/index/status/unit and
        # request <= write completion <= receive. Availability is assigned later
        # by SessionBus; never replace either source timestamp with that time.
        self.bus.publish({'kind': 'motor_parameter', 'motor_id': mid, 'parameter': name,
            'value': result['value'], 'unit': result['unit'], 'ok': True, 'status': 0,
            'index': result['index'], 'sequence': event['sequence'], 'cycle': event['cycle'],
            'request_monotonic_ns': event['write_started_monotonic_ns'],
            'write_finished_monotonic_ns': event['write_finished_monotonic_ns'],
            'monotonic_ns': event['received_monotonic_ns']}, telemetry_input=True)


def require_complete(report):
    live.require(report.get('status') == 'READONLY_PIPELINE_COMPLETE'
                 and report.get('identities_verified') is True and not report.get('errors')
                 and report.get('write_attempts') == MAX_REQUESTS
                 and report.get('replies') == MAX_REQUESTS
                 and report.get('cycles_completed') == CYCLES
                 and report.get('pending_at_end') == []
                 and report.get('residual_final') == {'serial_pending_bytes': 0,
                     'parser_pending_bytes': 0, 'parser_discarded_bytes': 0},
                 'Incomplete492-request pipeline collection')
    rows, cycles = report.get('requests', []), report.get('cycles', [])
    live.require(len(rows) == MAX_REQUESTS and len(cycles) == CYCLES
                 and all(r.get('ok') is True and r.get('write_expected_bytes') == 17
                         and type(r.get('write_returned_bytes')) is int
                         and r['write_returned_bytes'] == 17 for r in rows)
                 and [c.get('cycle') for c in cycles] == list(range(1, CYCLES+1))
                 and all(c.get('status') == 'COMPLETE' and c.get('requests') == 24
                         and c.get('replies') == 24 for c in cycles),
                 'Pipeline row/cycle completeness mismatch')


def can_producer(bus, done, expected_uids, plan, deadline_ns, *,
                 pipeline_factory=pipeline.PipelineCAN,
                 lock_factory=pipeline.ownership_locks, check_external=lambda: None):
    report, closed, locked = None, False, False
    try:
        # Both common process locks belong to this device worker until actual
        # context exit, even if the collector's finite join cannot finish.
        with lock_factory():
            locked = True
            bus.check(deadline_ns, check_external)
            with pipeline_factory(PipelineEvents(bus, expected_uids), window=4, gap_ms=.5,
                    cycles=CYCLES, max_seconds=plan['max_seconds'], request_order='by-parameter',
                    clock=bus.clock, check_interrupt=lambda: bus.check(deadline_ns, check_external)) as probe:
                report = probe.collect(expected_uids)
                require_complete(report)
                bus.check(deadline_ns, check_external)
            closed = True
    except BaseException as error:
        bus.fail('can_pipeline', repr(error))
    finally:
        with bus.lock:
            bus.producer_status['can'] = {'exited': True, 'device_context_exited': closed,
                'cross_process_locks_acquired': locked, 'report': report}
        done.set()  # completion/failure signal; never silently implies success


def _source_times(snapshot):
    values = {(r['motor_id'], r['parameter']): (r['request_ns'], r['received_ns'])
              for r in snapshot['motors']}
    im = snapshot['imu']
    values['imu'] = (im['read_started_ns'], im['read_finished_ns'])
    return values


def _snapshot(bus, inputs, tick, plan):
    for row in bus.available(tick):
        inputs.ingest(row)
    snapshot = inputs.buffer.snapshot(tick).as_dict()
    live.require(snapshot['status'] == 'DIAGNOSTIC_READY',
                 'Sensor snapshot blocked: '+'; '.join(snapshot['blocked_reasons']))
    live.require(snapshot['oldest_observation_age_ns']+bus.clock()-tick <= plan['max_age_ns'],
                 'Sensor input became stale during processing')
    return snapshot


def observe(bus, done, plan, deadline_ns, *, check_external=lambda: None, wait=None):
    wait = bus.stop.wait if wait is None else wait
    inputs = live.LiveInputs(plan)
    start = bus.clock()
    first_tick, failure, previous = None, None, None
    ticks, all_updated, imu_updated, comparisons = 0, 0, 0, 0
    lateness, ages, spreads, updates = [], [], [], []
    terminal = None
    try:
        while not inputs.ready():
            bus.check(min(deadline_ns, start+STARTUP_NS), check_external)
            for row in bus.available(bus.clock()): inputs.ingest(row)
            if not inputs.ready(): wait(.001)
        first_tick = bus.clock()+DT_NS
        bus.publish({'kind': 'sensor_schedule_armed', 'first_tick_ns': first_tick})
        while True:
            bus.check(deadline_ns, check_external)
            if done.is_set():
                live.require(ticks > 0, 'Pipeline completed before any scheduled observation')
                end = bus.clock()
                terminal = _snapshot(bus, inputs, end, plan)
                bus.publish({'kind': 'sensor_terminal_snapshot', 'monotonic_ns': end,
                             'snapshot': terminal, **FLAGS})
                break
            tick = first_tick+ticks*DT_NS
            while bus.clock() < tick and not done.is_set():
                bus.check(deadline_ns, check_external)
                wait(min(.005, max(0., (tick-bus.clock())/1e9)))
            if done.is_set(): continue
            bus.check(deadline_ns, check_external)
            actual = bus.clock()
            delay = actual-tick
            bus.publish({'kind': 'sensor_tick_start_check', 'tick_index': ticks,
                         'scheduled_tick_ns': tick, 'monotonic_ns': actual,
                         'start_lateness_ns': delay})
            live.require(0 <= delay <= plan['max_start_lateness_ns'],
                         '20ms sensor observation start deadline missed; no catchup')
            snapshot = _snapshot(bus, inputs, tick, plan)
            sources = _source_times(snapshot)
            changed = None if previous is None else [
                {'motor_id': key[0], 'parameter': key[1]} for key, value in sources.items()
                if key != 'imu' and value != previous[key]]
            im_changed = None if previous is None else sources['imu'] != previous['imu']
            # This event records the attempted observation. Only a tick that
            # passes the final processing deadline contributes to summary stats.
            bus.publish({'kind': 'sensor_benchmark_tick', 'tick_index': ticks,
                'scheduled_tick_ns': tick, 'actual_started_ns': actual, 'start_lateness_ns': delay,
                'snapshot': snapshot, 'updated_motor_keys_since_previous_tick': changed,
                'all24_motor_keys_updated': None if changed is None else len(changed) == 24,
                'imu_updated_since_previous_tick': im_changed, **FLAGS})
            bus.check(deadline_ns, check_external)
            live.require(bus.clock() < tick+DT_NS, 'Sensor observation processing exceeded20ms; no catchup')
            if previous is not None:
                comparisons += 1
                all_updated += int(len(changed) == 24)
                imu_updated += int(im_changed)
                updates.append(len(changed))
            previous = sources
            lateness.append(delay/1e6)
            ages.append(snapshot['oldest_observation_age_ns']/1e6)
            spreads.append(snapshot['acquisition_spread_ns']/1e6)
            ticks += 1
    except BaseException as error:
        failure = type(error).__name__+': '+str(error)
        bus.fail('sensor_observer', failure)
        try: bus.publish({'kind': 'sensor_benchmark_blocked', 'tick_index': ticks,
                         'monotonic_ns': bus.clock(), 'reason': failure})
        except BaseException: pass
    finally:
        bus.stop.set()
    return {**FLAGS, 'status': 'SENSOR_OBSERVATIONS_COMPLETE' if failure is None else 'INCOMPLETE',
        'failure': failure, 'first_tick_ns': first_tick, 'ticks_completed': ticks,
        'update_comparisons_excluding_initial_baseline': comparisons,
        'all24_updated_comparisons': all_updated, 'imu_updated_comparisons': imu_updated,
        'statistics_ms': {'start_lateness': pipeline.distribution(lateness),
            'oldest_observation_age': pipeline.distribution(ages),
            'acquisition_spread': pipeline.distribution(spreads)},
        'updated_motor_key_count': pipeline.distribution(updates),
        'identity_all12_verified': inputs.identities_verified,
        'terminal_snapshot': terminal, 'source_timestamps_changed': False}


def run_acquisition(bus, expected_uids, plan, *, check_external=lambda: None,
                    pipeline_factory=pipeline.PipelineCAN, can_lock_factory=pipeline.ownership_locks,
                    imu_factory=live.imu.ICM20948, imu_lock_factory=live.imu_ownership_lock):
    done = threading.Event()
    deadline = bus.clock()+int(plan['max_seconds']*1e9)
    threads = [threading.Thread(target=can_producer, name='sensor-pipeline-can', daemon=True,
        args=(bus, done, expected_uids, plan, deadline), kwargs={'pipeline_factory': pipeline_factory,
        'lock_factory': can_lock_factory, 'check_external': check_external}),
        threading.Thread(target=live.imu_producer, name='sensor-pipeline-imu', daemon=True,
        args=(bus, deadline), kwargs={'imu_factory': imu_factory, 'lock_factory': imu_lock_factory,
        'check_external': check_external})]
    result = {**FLAGS, 'status': 'INCOMPLETE'}
    try:
        for thread in threads: thread.start()
        result = observe(bus, done, plan, deadline, check_external=check_external)
    finally:
        bus.stop.set()
        for thread in threads:
            if thread.ident is not None: thread.join(timeout=1.)
        for thread in threads:
            if thread.is_alive(): bus.fail(thread.name, 'Producer termination unconfirmed')
    result['producer_status'] = copy.deepcopy(bus.producer_status)
    result['producer_threads_exited'] = all(not t.is_alive() for t in threads)
    can, im = bus.producer_status.get('can', {}), bus.producer_status.get('imu', {})
    if (result['status'] != 'SENSOR_OBSERVATIONS_COMPLETE' or bus.errors
            or not result['producer_threads_exited'] or not can.get('device_context_exited')
            or not can.get('cross_process_locks_acquired') or im.get('restore_status') not in ('restored', 'not_needed')):
        result['status'] = 'INCOMPLETE'
    else:
        try:
            require_complete(can.get('report', {}))
            result['status'] = 'COMPLETE_SENSOR_BENCHMARK_NO_OUTPUT'
        except BaseException as error:
            bus.fail('final_integrity', repr(error))
            result['status'] = 'INCOMPLETE'
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-no-output', action='store_true')
    ap.add_argument('--expected-uids', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--max-seconds', type=float, default=10.)
    args = ap.parse_args(argv)
    try:
        plan = make_plan(args.max_seconds)
        if not args.execute_no_output:
            print(json.dumps(plan, indent=2)); return 0
        raw = args.expected_uids.read_bytes()
        expected = pipeline.validate_uids(live.shadow._json(raw))
        output = live.replay._output_path(args.output)
        live.require(Path(__file__).resolve().parent.parent not in output.parents,
                     'Output must be outside runtime')
        output.mkdir(mode=0o700, exist_ok=False)
        live._sync_directory(output.parent)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    bus, writer, boot_guard, handlers, signals = live.SessionBus(), None, None, {}, []
    report = {**FLAGS, 'status': 'INCOMPLETE', 'plan': plan,
        'started_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'expected_uid_file_sha256': hashlib.sha256(raw).hexdigest(),
        'source_sha256': {Path(m.__file__).name: live.shadow.sha(m.__file__) for m in
            (pipeline, live, live.codec, live.timing, live.imu, live.observer,
             live.replay, live.shadow, live.telemetry)}}
    report['source_sha256'][Path(__file__).name] = live.shadow.sha(__file__)
    try:
        for number in (signal.SIGINT, signal.SIGTERM):
            handlers[number] = signal.signal(number, lambda sig, _: (signals.append(sig), bus.stop.set()))
        boot_guard = BootIdentityGuard()
        report['boot_id'] = boot_guard.boot_id
        def check_external():
            if signals: raise InterruptedError('signal '+str(signals[0]))
            boot_guard.check()
        writer = live.AuditWriter(output/'events.jsonl', bus); writer.start()
        bus.publish({'kind': 'sensor_benchmark_plan', 'plan': plan})
        report.update(run_acquisition(bus, expected, plan, check_external=check_external))
    except BaseException as error:
        report.update(status='INCOMPLETE', failure=type(error).__name__+': '+str(error))
        bus.fail('benchmark', report['failure'])
    finally:
        bus.stop.set()
        try:
            if writer is not None:
                try: writer.close()
                except BaseException as error: bus.fail('log_writer_close', repr(error))
            if boot_guard is not None:
                try: boot_guard.close()
                except BaseException as error: bus.fail('boot_monitor_close', repr(error))
            if not (output/'events.jsonl').exists():
                fd = os.open(output/'events.jsonl', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
                os.fsync(fd); os.close(fd)
            report.update(errors=bus.errors, signals=signals, dropped_events=bus.dropped_events,
                log_flush_confirmed=bool(writer and writer.flushed),
                events_sha256=live.shadow.sha(output/'events.jsonl') if writer is None or not writer.thread.is_alive() else None,
                completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
            if bus.errors or signals or not report['log_flush_confirmed']:
                report['status'] = 'INCOMPLETE'
            fd = os.open(output/'summary.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
                stream.flush(); os.fsync(stream.fileno())
            live._sync_directory(output)
        finally:
            for number, handler in handlers.items(): signal.signal(number, handler)
    print(json.dumps({'output': str(output), 'status': report['status'],
                     'ticks_completed': report.get('ticks_completed', 0), **FLAGS}))
    return 0 if report['status'] == 'COMPLETE_SENSOR_BENCHMARK_NO_OUTPUT' else 2


if __name__ == '__main__':
    raise SystemExit(main())
