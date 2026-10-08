"""Output coordination with byte-accurate in-memory buses; no devices opened.

Independent-watchdog tests use real monotonic time: short intentional stalls
terminate themselves and STOP timestamps must precede policy return.  Target,
codec and ownership tests may use shared causal time to isolate host scheduling.
These do not prove hardware deadlines, physical torque cutoff, or 50 Hz operation.
"""

from dataclasses import asdict
from concurrent.futures import wait as wait_for_test_workers
import math
import struct
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch, Mock

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw.native_diagnostic_transport import Record, Stats
from singularitydog_hw.policy_motion_envelope import AxisLimits


def wire(can_id, data):
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + data + b"\r\n"


def quantize(value, low, high):
    if not low <= value <= high:
        raise ValueError("Synthetic encoder range exceeded")
    return int((value - low) * 65535 / (high - low))


def encode_motion(mid, position, kp, kd):
    data = struct.pack(">4H", quantize(position, -12.57, 12.57),
                       quantize(0., -50., 50.), quantize(kp, 0., 500.), quantize(kd, 0., 5.))
    return wire((1 << 24) | (quantize(0., -5.5, 5.5) << 8) | mid, data)


def profile():
    axis = asdict(AxisLimits(-1., 1., 4., .2, .1, 1., .3, 1., 3., 60., 3., .175))
    return {"schema": "singularitydog.supported-policy-profile.v2",
            "motor_power_epoch": "synthetic-power-epoch-1",
            "request_gap_us": 600, "request_window": 3,
            "output_allowed": True, "duration_s": .7,
            "startup_duration_s": .1, "policy_ramp_s": .1, "stop_duration_s": .1,
            "policy_weight": .1,
            "max_sample_age_ms": 50., "hard_cycle_ms": 100.,
            "max_sample_gap_ms": 100.,
            "max_consecutive_20ms_misses": 0,
            "voltage_min_v": 35., "voltage_max_v": 43.,
            "watchdog_by_id": {str(mid): {"configured_timeout_ms":200.,
                "max_observed_disable_ms":180.,"version_bytes_hex":"05001300"} for mid in range(1,13)},
            "axes": {str(mid): {**axis, "uid": (bytes([mid]) * 8).hex(),
                                "sign": 1, "offset_rad": 0.,
                                "physical_lower_rad": -1.1,
                                "physical_upper_rad": 1.1,
                                "uncertainty_rad": .01}
                     for mid in range(1, 13)}}


def measured_startup_profile():
    data = profile()
    data.update(schema=live.SCHEMA_V3, telemetry_cadence=live.CADENCE_PRE_ENABLE,
        cadence_source_sha256=live.cadence_source_hashes(),
        model_backend=live.SCALAR_BACKEND, voltage_overlap=True,
        diagnostic_timing_acceptance=live.MEASURED_R17_STARTUP_TIMING,
        hard_cycle_ms=20., max_sample_age_ms=20., max_sample_gap_ms=21.)
    return data


class SimulatedClock:
    """Shared causal test time; actual thread handshakes still use Events."""
    def __init__(self):
        self.value=1_000_000_000;self.lock=threading.Lock()
    def __call__(self):
        with self.lock:
            self.value+=100
            return self.value
    def advance(self,nanoseconds):
        with self.lock:self.value+=nanoseconds
    def advance_to(self,nanoseconds):
        with self.lock:self.value=max(self.value,nanoseconds)
    def sleep(self,seconds):
        self.advance(math.ceil(seconds*1e9))


def causal_semantic_wait(futures, *, timeout, return_when):
    """Wait for fake bus handshakes without spending simulated device time.

    The original coordinator still checks every injected-clock deadline. Five
    wall-clock seconds only bound a broken test worker, rather than allowing
    unrelated host load to expire a synthetic 20ms/100ms device deadline.
    Real-monotonic watchdog and timing tests keep the production wait function.
    """
    return wait_for_test_workers(futures, timeout=5.0, return_when=return_when)


class FakeSession:
    """One independent simulated bus; records are the real native ABI structure."""
    def __init__(self, first_id, *, uid_mismatch=False, high_returned_torque=False,
                 fail_motion=False, stop_unconfirmed=False, stop_fault=False,
                 initial_temperature_c=25., torque_fault_after=0,
                 stop_complete=True, stop_ambiguous=False,
                 firmware_bytes='05001300',version_returns_feedback=False,version_timeout=False,
                 clock=time.monotonic_ns):
        self.clock=clock
        self.ids = tuple(range(first_id, first_id + 6))
        self.uid_mismatch = uid_mismatch
        self.high_returned_torque = high_returned_torque
        self.fail_motion = fail_motion
        self.stop_unconfirmed = stop_unconfirmed
        self.stop_fault = stop_fault
        self.initial_temperature_c = initial_temperature_c
        self.torque_fault_after = torque_fault_after
        self.stop_complete = stop_complete
        self.stop_ambiguous = stop_ambiguous
        self.firmware_bytes=firmware_bytes
        self.version_returns_feedback=version_returns_feedback
        self.version_timeout=version_timeout
        self.enabled = set()
        self.positions = {mid: 32768 * 25.14 / 65535 - 12.57 for mid in self.ids}
        self.calls = []
        self.stop_times = []
        self.positive_gain_writes = 0
        self._busy = threading.Lock()

    def exchange(self, wires, *, timeout_ns=100_000_000, deadline_ns=None):
        if deadline_ns is not None:
            timeout_ns=deadline_ns-self.clock()
            if timeout_ns<=0:raise TimeoutError('Synthetic absolute deadline expired')
        records,stats=self._exchange(wires, timeout_ns, send_only=False)
        if deadline_ns is not None:
            for record in records:record.deadline_ns=deadline_ns
        return records,stats

    def send_only(self, wires, *, timeout_ns):
        return self._exchange(wires, timeout_ns, send_only=True)

    def _exchange(self, wires, timeout_ns, send_only):
        if not self._busy.acquire(blocking=False):
            raise AssertionError("One bus had concurrent owners")
        try:
            records = []
            stats = Stats(); stats.begin_ns = self.clock()
            for request in wires:
                tx = codec.ATParser().feed(request)[0]
                mid = tx.destination
                if mid not in self.ids:
                    raise AssertionError("Cross-bus motor request")
                started = self.clock()
                self.calls.append((started, tx.kind, mid, request))
                high_torque = False
                if tx.kind == 3:
                    self.enabled.add(mid)
                elif tx.kind == 4:
                    self.enabled.discard(mid)
                elif tx.kind == 1:
                    p, _, kp, _ = struct.unpack(">4H", tx.data)
                    if kp:
                        self.positive_gain_writes += 1
                        if self.fail_motion:
                            raise OSError("Synthetic USB lost on motion")
                        high_torque = self.high_returned_torque and self.positive_gain_writes > self.torque_fault_after
                    self.positions[mid] = p * 25.14 / 65535 - 12.57
                if tx.kind==4 and tx.data==runtime.versions.VERSION_PAYLOAD and not self.version_returns_feedback:
                    if self.version_timeout:raise TimeoutError('Synthetic version deadline; no fallback')
                    response=wire((2<<24)|(mid<<8)|0xFD,
                                  runtime.versions.VERSION_PREFIX+bytes.fromhex(self.firmware_bytes)+b'\xa5')
                elif tx.kind in (1, 3, 4, 18):
                    if send_only:raise AssertionError('Active command skipped acknowledgement')
                    mode = 2 if mid in self.enabled else 0
                    data = struct.pack(">4H", quantize(self.positions[mid], -12.57, 12.57),
                        32768, quantize(4. if high_torque else 0., -5.5, 5.5),
                        int(self.initial_temperature_c * 10))
                    response = wire((2 << 24) | (mode << 22) | (mid << 8) | 0xFD, data)
                elif tx.kind == 0:
                    uid_byte = 0xEE if self.uid_mismatch else mid
                    response = wire((mid << 8) | 0xFE, bytes([uid_byte]) * 8)
                elif tx.kind == 17:
                    index = int.from_bytes(tx.data[:2], "little")
                    name, (_, fmt, _) = next((name, entry) for name, entry in codec.PARAMETERS.items()
                                            if entry[0] == index)
                    value = {"run_mode": 0, "position": self.positions[mid], "velocity": 0.,
                             "voltage": 40., "can_timeout": runtime.protocol.WATCHDOG_TICKS}[name]
                    payload = (tx.data[:4] + struct.pack("<" + fmt, value)).ljust(8, b"\0")
                    response = wire((17 << 24) | (mid << 8) | 0xFD, payload)
                else:
                    raise AssertionError(f"Unexpected request kind {tx.kind}")
                record = Record()
                record.start_ns = started
                record.finish_ns = started + 1
                record.read_start_ns = started + 1
                record.received_ns = started + 2 if response else 0
                record.deadline_ns = started + timeout_ns
                record.tx[:] = request
                record.rx[:] = response if response else bytes(17)
                record.written = 17; record.received = len(response)
                records.append(record)
            stats.end_ns = self.clock()
            stats.writes = len(records); stats.reads = sum(bool(row.received) for row in records)
            stats.bytes = 17 * stats.reads
            return records, stats
        finally:
            self._busy.release()

    def emergency_stop(self):
        if not self._busy.acquire(blocking=False):
            raise AssertionError("Emergency STOP raced another writer")
        try:
            self.stop_times.append(self.clock())
            self.enabled.clear()
            confirmed = self.ids[:-1] if self.stop_unconfirmed else self.ids
            return {"confirmed_ids": list(confirmed),
                    "unconfirmed_ids": [mid for mid in self.ids if mid not in confirmed],
                    "fault_by_id": {str(mid): int(self.stop_fault) for mid in self.ids},
                    "complete": self.stop_complete,
                    "ambiguous_ids": [self.ids[0]] if self.stop_ambiguous else []}
        finally:
            self._busy.release()


class FakeIMU:
    def __init__(self, *, frozen=False,clock=time.monotonic_ns):
        self.frozen = frozen
        self.clock=clock
        self.first = None

    def __call__(self):
        now = self.clock()
        if self.first is None:
            self.first = now
        stamp = self.first if self.frozen else now
        return {"read_started_monotonic_ns": stamp,
                "read_finished_monotonic_ns": stamp + 1,
                "body_angular_velocity_rad_s": [0., 0., 0.],
                "accel_m_s2": [0., 0., 9.80665], "gyro_rad_s": [0., 0., 0.],
                "projected_gravity": [0., 0., -1.], "valid": True}


class OutputRuntimeTests(unittest.TestCase):
    _FAILED_ACQUISITION_TIMESTAMPS = (
        'combined_acquisition_wait_begin_ns', 'combined_acquisition_wait_end_ns',
        'feedback_collect_begin_ns', 'feedback_collect_end_ns',
        'imu_wait_begin_ns', 'imu_wait_end_ns',
        'imu_read_started_ns', 'imu_read_finished_ns',
    )

    def run_case(self, *, front=None, rear=None, imu=None, policy=None, profile_data=None, **kwargs):
        sessions = {"front": front or FakeSession(1), "rear": rear or FakeSession(7)}
        cancelled = threading.Event()
        # Exercise runtime limits without macOS timer coalescing deciding an
        # unrelated ownership/codec test. Intentional policy sleeps still use
        # real time and the independent watchdog remains active.
        def fixture_sleep(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:time.sleep(0)
        kwargs.setdefault('sleep',fixture_sleep)
        report = runtime.run_supported_policy(profile_data or profile(), sessions, imu or FakeIMU(),
            policy or (lambda sample, imu, now: (.04,) * 12),
            cancel_io=cancelled.set, encode_motion=encode_motion, **kwargs)
        self.assertTrue(cancelled.is_set())
        self.assertTrue(all(len(session.stop_times) == 1 for session in sessions.values()))
        return report, sessions

    def test_startup_displacement_rejected_before_gains_at_each_transition(self):
        for stage in ('enable','zero_gain','all_axis_zero_gain'):
            for direction in (-1,1):
                with self.subTest(stage=stage,direction=direction):
                    clock=SimulatedClock();data=measured_startup_profile()
                    for axis in data['axes'].values():axis['max_displacement_from_start_rad']=math.radians(1)
                    class StartupDrift(FakeSession):
                        injected=False
                        def _exchange(self,wires,timeout_ns,send_only):
                            result=super()._exchange(wires,timeout_ns,send_only)
                            for record in result[0]:
                                tx=codec.ATParser().feed(bytes(record.tx))[0]
                                kind=('enable' if tx.kind==3 else 'zero_gain' if tx.kind==1 and len(wires)==1
                                      else 'all_axis_zero_gain' if tx.kind==1 and len(wires)==6 else None)
                                if not self.injected and tx.destination==1 and kind==stage:
                                    original=self.positions[1]
                                    record.rx[7:9]=quantize(original+direction*math.radians(1.25),
                                                           -12.57,12.57).to_bytes(2,'big')
                                    self.injected=True
                            return result
                    front=StartupDrift(1,clock=clock)
                    report,sessions=self.run_case(profile_data=data,front=front,
                        rear=FakeSession(7,clock=clock),imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
                    self.assertTrue(front.injected)
                    self.assertEqual(report['status'],'ABORTED',report['errors'])
                    self.assertIn('startup trial displacement from pre-enable origin',str(report['errors']))
                    self.assertEqual(report['startup_displacement_checks'][-1]['stage'],stage)
                    self.assertTrue(report['stop_confirmed'])
                    self.assertTrue(all(s.positive_gain_writes==0 for s in sessions.values()))
                    if stage!='all_axis_zero_gain':
                        self.assertEqual([mid for _,kind,mid,_ in front.calls if kind==3],[1])

    def run_small_startup_drift(self,*,late_drift=False,target=.04):
        clock=SimulatedClock();data=measured_startup_profile()
        for axis in data['axes'].values():axis['max_displacement_from_start_rad']=math.radians(1)
        class SmallDrift(FakeSession):
            def __init__(self):
                super().__init__(1,clock=clock)
                self.origin=self.positions[1];self.late_injected=False
            def _exchange(self,wires,timeout_ns,send_only):
                result=super()._exchange(wires,timeout_ns,send_only)
                for record in result[0]:
                    tx=codec.ATParser().feed(bytes(record.tx))[0]
                    if tx.kind!=1 or tx.destination!=1:continue
                    kp=int.from_bytes(tx.data[4:6],'big')
                    if self.positive_gain_writes==0:
                        position=self.origin+math.radians(.5)
                    elif late_drift and kp and not self.late_injected:
                        position=self.origin+math.radians(1.05);self.late_injected=True
                    else:continue
                    record.rx[7:9]=quantize(position,-12.57,12.57).to_bytes(2,'big')
                return result
        front=SmallDrift()
        return self.run_case(profile_data=data,front=front,rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),policy=lambda *args:(target,)*12,
            clock=clock,sleep=clock.sleep)

    def test_small_startup_drift_keeps_smooth_target_and_original_one_degree_bound(self):
        report,sessions=self.run_small_startup_drift()
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        origin=report['trial_origin_model_rad_by_id']['1']
        start=next(row['q_model_rad'] for row in report['startup_displacement_checks']
                   if row['motor_id']==1 and row['stage']=='all_axis_zero_gain')
        self.assertGreater(start-origin,math.radians(.4))
        self.assertAlmostEqual(report['cycles'][0]['command']['q_model_rad'][0],start,delta=1e-6)
        for row in report['cycles']:
            self.assertLessEqual(abs(row['command']['q_model_rad'][0]-origin),math.radians(1))
            self.assertLessEqual(abs(row['feedback']['q_model_rad'][0]-origin),math.radians(1))
        for _,kind,mid,raw in sessions['front'].calls:
            if kind==1 and mid==1:
                q=int.from_bytes(raw[7:9],'big')*25.14/65535-12.57
                self.assertLessEqual(abs(q-origin),math.radians(1))

    def test_cyclic_reply_cannot_reset_one_degree_origin_after_small_startup_drift(self):
        report,sessions=self.run_small_startup_drift(late_drift=True)
        self.assertTrue(sessions['front'].late_injected)
        self.assertEqual(report['status'],'ABORTED',report['errors'])
        self.assertIn('ID1 trial displacement',str(report['errors']))
        self.assertTrue(report['stop_confirmed'])

    def test_target_is_rejected_at_original_origin_bound_before_outside_command(self):
        report,sessions=self.run_small_startup_drift(target=.3)
        self.assertEqual(report['status'],'ABORTED',report['errors'])
        self.assertIn('target outside joint/supported displacement envelope',str(report['errors']))
        origin=report['trial_origin_model_rad_by_id']['1']
        for _,kind,mid,raw in sessions['front'].calls:
            if kind==1 and mid==1:
                q=int.from_bytes(raw[7:9],'big')*25.14/65535-12.57
                self.assertLessEqual(abs(q-origin),math.radians(1))
        self.assertTrue(report['stop_confirmed'])

    def test_all_axis_zero_gain_comparison_uses_type1_without_motion_gains(self):
        data=profile();data['policy_weight']=0.
        for axis in data['axes'].values():axis.update(kp=0.,kd=0.)
        # This case checks wire encoding, not host scheduling. Use the same
        # causal clock for both buses and the IMU; real watchdog tests below
        # retain wall time and independently assert stop-before-return.
        clock=SimulatedClock()
        report,sessions=self.run_case(profile_data=data,
            front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['motor_enable_sent'])
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent'])
        for session in sessions.values():
            commands=[codec.ATParser().feed(w)[0] for _,kind,_,w in session.calls if kind==1]
            self.assertGreater(len(commands),12)
            for command in commands:
                _,velocity,kp,kd=struct.unpack('>4H',command.data)
                self.assertEqual((velocity,kp,kd),(32767,0,0))
                self.assertEqual((command.can_id>>8)&65535,32767)

    def test_each_bus_owner_uses_bounded_stop_retries_when_available(self):
        owners={}
        class RetryingSession(FakeSession):
            def emergency_stop_repeated(self):
                owners[self.ids[0]]=threading.current_thread().name
                return {**self.emergency_stop(),'retry_policy':{'stop_only':True,'attempts_completed':1}}
        report,_=self.run_case(front=RetryingSession(1),rear=RetryingSession(7))
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(set(owners),{1,7})
        self.assertTrue(owners[1].startswith('policy-front'))
        self.assertTrue(owners[7].startswith('policy-rear'))
        self.assertTrue(all(r['retry_policy']['stop_only'] for r in report['stop_reports'].values()))

    def test_cancellation_hook_error_keeps_both_stops_and_returns_failure_report(self):
        for normal in (False,True):
            with self.subTest(normal=normal):
                clock=SimulatedClock()
                sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
                def cancel():raise OSError('Synthetic cancellation descriptor failed')
                def check():
                    if not normal:raise RuntimeError('Synthetic preparation failure')
                report=runtime.run_supported_policy(profile(),sessions,FakeIMU(clock=clock),
                    lambda *_:(.04,)*12,cancel_io=cancel,check=check,encode_motion=encode_motion,
                    clock=clock,sleep=clock.sleep)
                self.assertTrue(report['stop_confirmed'],report['errors'])
                self.assertEqual(report['status'],'ABORTED_STOP_DISPATCH' if normal else 'ABORTED')
                self.assertEqual(report['stop_dispatch_errors'],[{
                    'stage':'cancel_io','bus':None,'error':'OSError: Synthetic cancellation descriptor failed'}])
                self.assertTrue(any('cancellation descriptor failed' in error for error in report['errors']))
                self.assertTrue(all(len(s.stop_times)==1 for s in sessions.values()))

    def test_failed_stop_submission_on_one_owner_does_not_skip_other_owner(self):
        sessions={'front':FakeSession(1),'rear':FakeSession(7)}
        workers=runtime.BusWorkers(sessions,lambda:None)
        try:
            with patch.object(workers.pools['front'],'submit',side_effect=RuntimeError('Synthetic owner unavailable')):
                stopped=workers.finish_stops()
            self.assertEqual(stopped['front']['unconfirmed_ids'],list(runtime.BUSES['front']))
            self.assertIn('owner unavailable',stopped['front']['error'])
            self.assertEqual(stopped['rear']['confirmed_ids'],list(runtime.BUSES['rear']))
            self.assertEqual(len(sessions['rear'].stop_times),1)
            self.assertEqual(workers.emergency_errors[0]['bus'],'front')
            self.assertEqual(workers.emergency_errors[0]['stage'],'stop_submission')
        finally:workers.close()

    def test_reentrant_cancel_callback_schedules_only_one_stop_per_owner(self):
        sessions={'front':FakeSession(1),'rear':FakeSession(7)}
        workers=runtime.BusWorkers(sessions,lambda:workers.emergency('Nested cancellation'))
        try:
            workers.emergency('Original failure')
            stopped=workers.finish_stops()
            self.assertEqual(workers.reason,'Original failure')
            self.assertEqual(workers.emergency_errors,[])
            self.assertTrue(all(len(s.stop_times)==1 for s in sessions.values()))
            self.assertTrue(all(stopped[scope]['complete'] for scope in runtime.BUSES))
        finally:workers.close()

    def test_stop_collection_has_one_shared_deadline_with_dispatch_margin(self):
        timeouts=[]
        future=SimpleNamespace(result=lambda *,timeout:timeouts.append(timeout) or {'complete':True})
        workers=runtime.BusWorkers.__new__(runtime.BusWorkers)
        workers.stop_futures={'front':future,'rear':future}
        workers.emergency=Mock()
        with patch.object(runtime.time,'monotonic',side_effect=[100.,100.,100.8]):
            result=workers.finish_stops()
        self.assertEqual(set(result),{'front','rear'})
        self.assertAlmostEqual(timeouts[0],1.25)
        self.assertAlmostEqual(timeouts[1],.45)
        workers.emergency.assert_called_once_with('normal completion',normal_completion=True)

    def test_stop_collection_does_not_timeout_at_old_one_second_boundary(self):
        class SlowCleanup(FakeSession):
            def emergency_stop_repeated(self):
                time.sleep(1.03)
                return self.emergency_stop()
        workers=runtime.BusWorkers({'front':SlowCleanup(1),'rear':SlowCleanup(7)},lambda:None)
        try:
            stopped=workers.finish_stops()
            for scope,ids in runtime.BUSES.items():
                self.assertNotIn('error',stopped[scope])
                self.assertEqual(stopped[scope]['confirmed_ids'],list(ids))
        finally:workers.close()

    def test_zero_policy_high_gain_hold_never_follows_varying_model_targets(self):
        # This checks zero-mixture target isolation, not host scheduling speed.
        # Use causal simulated time so unrelated Mac load cannot trip 20ms.
        clock=SimulatedClock()
        data=profile();data.update(policy_weight=0., duration_s=2., startup_duration_s=1.)
        for axis in data['axes'].values():axis.update(kp=12.,kd=.15,max_estimated_pd_torque_nm=.5)
        calls=[]
        def varying_policy(sample,imu,now):
            calls.append(now)
            return ((.8 if len(calls)%2 else -.8),)*12
        with patch.object(runtime,'wait',side_effect=causal_semantic_wait):
            report,sessions=self.run_case(profile_data=data,policy=varying_policy,
                front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
                imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertGreater(len(calls),10)
        self.assertTrue(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(report['stop_confirmed'])
        for session in sessions.values():
            positions={mid:set() for mid in session.ids};gains=[]
            for _,kind,mid,wire_bytes in session.calls:
                if kind!=1:continue
                command=codec.ATParser().feed(wire_bytes)[0]
                pos,velocity,kp,kd=struct.unpack('>4H',command.data)
                positions[mid].add(pos);gains.append(kp)
                self.assertEqual(velocity,32767)
                self.assertEqual((command.can_id>>8)&65535,32767)
            self.assertTrue(all(len(values)==1 for values in positions.values()))
            self.assertIn(quantize(12.,0.,500.),gains)

    def test_hold_probe_skips_inference_but_checks_inputs_each_cycle(self):
        # Exercise input-validation and target-isolation semantics without
        # unrelated host scheduling counting as synthetic device latency.
        clock=SimulatedClock()
        data=profile();data.update(policy_weight=0.,duration_s=2.,startup_duration_s=1.)
        for axis in data['axes'].values():axis.update(kp=12.,kd=.15,max_estimated_pd_torque_nm=.5)
        checked=[]
        class ValidationOnly:
            def validate_inputs(self,sample,imu,now):checked.append(now)
            def __call__(self,*args):raise AssertionError('Hold must not run inference')
        with patch.object(runtime,'current_position_hold_only',return_value=True), \
                patch.object(runtime,'wait',side_effect=causal_semantic_wait):
            report,sessions=self.run_case(profile_data=data,policy=ValidationOnly(),
                front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
                imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['cyclic_inference_skipped'])
        self.assertTrue(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(report['stop_confirmed'])
        # One validation precedes enable; each control cycle validates again.
        self.assertEqual(len(checked),len(report['cycles'])+1)
        for session in sessions.values():
            positions={mid:set() for mid in session.ids}
            for _,kind,mid,wire_bytes in session.calls:
                if kind==1:positions[mid].add(struct.unpack('>4H',codec.ATParser().feed(wire_bytes)[0].data)[0])
            self.assertTrue(all(len(values)==1 for values in positions.values()))

    def test_hold_probe_input_fault_stops_before_positive_gains(self):
        data=profile();data['policy_weight']=0.
        class BadInputs:
            def validate_inputs(self,*args):raise ValueError('Synthetic tilt fault')
            def __call__(self,*args):raise AssertionError('Unexpected inference')
        with patch.object(runtime,'current_position_hold_only',return_value=True):
            report,_=self.run_case(profile_data=data,policy=BadInputs())
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Synthetic tilt fault',str(report['errors']))
        self.assertFalse(report['motion_gain_sent'])
        self.assertTrue(report['stop_confirmed'])

    def test_early_damping_reaches_wires_while_real_policy_still_runs(self):
        data=measured_startup_profile()
        data.update(duration_s=2.,startup_duration_s=.4,policy_ramp_s=.4,
                    stop_duration_s=.4,startup_damping_duration_s=.08,policy_weight=.005)
        calls=[]
        def actual_policy(*args):
            calls.append(1)
            return (.01,)*12
        logical_clock=SimulatedClock()
        report,sessions=self.run_case(profile_data=data,policy=actual_policy,
            front=FakeSession(1,clock=logical_clock),rear=FakeSession(7,clock=logical_clock),
            imu=FakeIMU(clock=logical_clock),clock=logical_clock,sleep=logical_clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertGreater(len(calls),20)
        self.assertTrue(report['learned_targets_sent'])
        self.assertFalse(report['cyclic_inference_skipped'])
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['startup_gain_schedule']['damping_duration_s'],.08)
        self.assertTrue(any(command['kd'][0] > command['kp'][0]*.2/4.
                            for c in report['cycles'] if c['phase']=='starting'
                            for command in (c['command'],)))
        for session in sessions.values():
            wires=[struct.unpack('>4H',codec.ATParser().feed(blob)[0].data)
                   for _,kind,_,blob in session.calls if kind==1]
            self.assertTrue(any(kd==quantize(.2,0.,5.) and kp<quantize(4.,0.,500.)
                                for _,_,kp,kd in wires))
        self.assertTrue(report['stop_confirmed'])

    def test_v3_cold_enable_gets_30ms_without_extending_control_period(self):
        clock=SimulatedClock();budgets=[]
        class ColdEnable(FakeSession):
            def exchange(self,wires,*,timeout_ns=100_000_000,deadline_ns=None):
                kind=codec.ATParser().feed(wires[0])[0].kind
                if kind==3:
                    budgets.append(timeout_ns)
                    if not self.enabled:
                        if timeout_ns<25_000_000:raise TimeoutError('Cold enable needs25ms')
                        clock.advance(25_000_000)
                return super().exchange(wires,timeout_ns=timeout_ns,deadline_ns=deadline_ns)
        report,_=self.run_case(profile_data=measured_startup_profile(),
            front=ColdEnable(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(budgets,[30_000_000]*6)
        self.assertTrue(report['zero_gain_enable_transition']['complete'])
        self.assertTrue(report['learned_targets_sent'])
        self.assertTrue(all(c['iteration_ms']<=20 for c in report['cycles']))

    def test_v3_enable_sequence_deadline_stops_before_positive_gains(self):
        clock=SimulatedClock()
        class SlowEveryEnable(FakeSession):
            def exchange(self,wires,*,timeout_ns=100_000_000,deadline_ns=None):
                if codec.ATParser().feed(wires[0])[0].kind==3:
                    if timeout_ns<25_000_000:raise TimeoutError('Enable sequence budget exhausted')
                    clock.advance(25_000_000)
                return super().exchange(wires,timeout_ns=timeout_ns,deadline_ns=deadline_ns)
        report,sessions=self.run_case(profile_data=measured_startup_profile(),
            front=SlowEveryEnable(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'ABORTED',report['errors'])
        self.assertIn('budget exhausted',str(report['errors']))
        self.assertFalse(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(all(s.positive_gain_writes==0 for s in sessions.values()))

    def test_v3_startup_checks_one_axis_enable_and_zero_before_next_bus(self):
        clock=SimulatedClock()
        report,sessions=self.run_case(profile_data=measured_startup_profile(),
            front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        expected=[mid for index in range(6) for mid in (index+1,index+7)]
        transition=report['zero_gain_enable_transition']
        self.assertEqual(transition['strategy'],'serial_axis_enable_then_zero_gain_v1')
        self.assertEqual([row['motor_id'] for row in transition['ordered_axes']],expected)
        self.assertEqual(transition['completed_axes'],expected)
        self.assertIsNone(transition['current_axis']);self.assertIsNone(transition['current_stage'])
        self.assertEqual(transition['enable_reply_budget_ms'],30.)
        self.assertEqual(transition['total_budget_ms'],120.)
        self.assertLess(transition['end_ns']-transition['begin_ns'],120_000_000)
        events=[row for row in report['journal'] if row['phase'] in ('startup_enable','startup_zero_gain')]
        self.assertEqual(len(events),24)
        for index,mid in enumerate(expected):
            pair=events[2*index:2*index+2]
            self.assertEqual([row['phase'] for row in pair],['startup_enable','startup_zero_gain'])
            self.assertEqual([row['bus'] for row in pair],['front' if mid<=6 else 'rear']*2)
            for row in pair:
                self.assertEqual(len(row['records']),1)
                record=row['records'][0]
                self.assertEqual(codec.ATParser().feed(bytes.fromhex(record['tx_hex']))[0].destination,mid)
                self.assertEqual(record['received'],17)
            if index:
                previous=events[2*index-1]['records'][0]
                self.assertGreater(pair[0]['records'][0]['start_ns'],previous['received_ns'])
        self.assertTrue(report['stop_confirmed'])

    def test_v3_all_twelve_startup_axes_fail_before_next_step_or_model(self):
        expected=[mid for index in range(6) for mid in (index+1,index+7)]
        for failure in ('invalid_mode','missing_reply'):
            for stage in ('enable','zero_gain'):
                for target in expected:
                    with self.subTest(failure=failure,stage=stage,target=target):
                        clock=SimulatedClock();model_calls=[]
                        class FailedHandshake(FakeSession):
                            def _exchange(self,wires,timeout_ns,send_only):
                                result=super()._exchange(wires,timeout_ns,send_only)
                                for record in result[0]:
                                    tx=codec.ATParser().feed(bytes(record.tx))[0]
                                    selected=(tx.kind==3 if stage=='enable' else tx.kind==1 and len(wires)==1)
                                    if tx.destination!=target or not selected:continue
                                    if failure=='missing_reply':
                                        # The request was written; no reply is admitted.
                                        raise TimeoutError('Synthetic startup reply missing after write')
                                    response_id=(int.from_bytes(bytes(record.rx)[2:6],'big')>>3)
                                    response_id=(response_id&~(3<<22))|(1<<22)
                                    record.rx[2:6]=((response_id<<3)|4).to_bytes(4,'big')
                                return result
                        front=(FailedHandshake if target<=6 else FakeSession)(1,clock=clock)
                        rear=(FailedHandshake if target>=7 else FakeSession)(7,clock=clock)
                        report,sessions=self.run_case(profile_data=measured_startup_profile(),
                            front=front,rear=rear,imu=FakeIMU(clock=clock),
                            policy=lambda *args:model_calls.append(1) or (.04,)*12,
                            clock=clock,sleep=clock.sleep)
                        self.assertEqual(report['status'],'ABORTED',report['errors'])
                        self.assertEqual(model_calls,[])
                        self.assertFalse(report['motion_gain_sent']);self.assertFalse(report['learned_targets_sent'])
                        self.assertFalse(report['zero_gain_enable_transition']['complete'])
                        current=report['zero_gain_enable_transition']['current_axis']
                        self.assertEqual(current,{'bus':'front' if target<=6 else 'rear','motor_id':target})
                        self.assertEqual(report['zero_gain_enable_transition']['current_stage'],stage)
                        stop_index=expected.index(target)
                        self.assertEqual(report['zero_gain_enable_transition']['completed_axes'],expected[:stop_index])
                        calls=sorted(call for session in sessions.values() for call in session.calls)
                        self.assertEqual([mid for _,kind,mid,_ in calls if kind==3],expected[:stop_index+1])
                        zeros=[mid for _,kind,mid,_ in calls if kind==1]
                        self.assertEqual(zeros,expected[:stop_index+(stage=='zero_gain')])
                        self.assertTrue(report['stop_confirmed'])
                        self.assertTrue(all(s.positive_gain_writes==0 for s in sessions.values()))

    def test_v3_missing_id3_enable_preserves_unconfirmed_ambiguous_stop(self):
        clock=SimulatedClock()
        class MissingID3(FakeSession):
            def _exchange(self,wires,timeout_ns,send_only):
                result=super()._exchange(wires,timeout_ns,send_only)
                if any(codec.ATParser().feed(bytes(r.tx))[0].kind==3 and
                       codec.ATParser().feed(bytes(r.tx))[0].destination==3 for r in result[0]):
                    raise TimeoutError('Synthetic ID3 enable reply missing after write')
                return result
            def emergency_stop(self):
                row=super().emergency_stop()
                row.update(complete=False,confirmed_ids=[i for i in self.ids if i!=3],
                           unconfirmed_ids=[3],ambiguous_ids=[3])
                return row
        report,sessions=self.run_case(profile_data=measured_startup_profile(),
            front=MissingID3(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED',report['errors'])
        self.assertFalse(report['stop_confirmed'])
        self.assertEqual(report['stop_reports']['front']['ambiguous_ids'],[3])
        self.assertFalse(report['motion_gain_sent']);self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(report['zero_gain_enable_transition']['completed_axes'],[1,7,2,8])
        self.assertEqual([mid for _,kind,mid,_ in sessions['rear'].calls if kind==3],[7,8])

    def test_non_v3_startup_retains_paired_bus_exchange(self):
        clock=SimulatedClock()
        report,_=self.run_case(profile_data=profile(),
            front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertNotIn('zero_gain_enable_transition',report)
        events=[row for row in report['journal'] if row['phase'] in ('startup_enable','startup_zero_gain')]
        for index in range(6):
            group=events[index*4:(index+1)*4]
            self.assertEqual({row['phase'] for row in group[:2]},{'startup_enable'})
            self.assertEqual({row['bus'] for row in group[:2]},{'front','rear'})
            self.assertEqual({row['phase'] for row in group[2:]},{'startup_zero_gain'})
            self.assertEqual({row['bus'] for row in group[2:]},{'front','rear'})

    def run_measured_startup_timing_case(self, *, delayed_cycle=None, late_reply=False,
                                         acceptance=True, delayed_elapsed_ns=20_010_000,
                                         second_cycle_processing_delay_ns=0):
        data = measured_startup_profile()
        if not acceptance:
            data['diagnostic_timing_acceptance'] = None
        logical_clock = SimulatedClock()
        state = {'index': None, 'begin': None, 'injected': False,
                 'second_cycle_delayed': False}
        stop_requested = threading.Event()
        original_begin = runtime._PendingCycleTiming.begin

        def capture_begin(pending, index, release, begun, previous_candidate, previous_sample):
            state.update(index=index, begin=begun)
            original_begin(pending, index, release, begun,
                           previous_candidate, previous_sample)
            if index == 0:
                # Leave a causal input-to-cycle-start margin for the unchanged
                # 20 ms sample-age check when the first cycle ends at 20.010 ms.
                logical_clock.advance(200_000)

        class TimedPolicy:
            def __call__(self, *args):
                return (.04,) * 12

            def validate_inputs(self, *args):
                if (second_cycle_processing_delay_ns and state['index'] == 1 and
                        not state['second_cycle_delayed']):
                    logical_clock.advance(second_cycle_processing_delay_ns)
                    state['second_cycle_delayed'] = True

            @property
            def last_validation(self):
                if state['index'] == 0:
                    stop_requested.set()
                if delayed_cycle == state['index'] and not state['injected']:
                    # First-cycle post-reply bookkeeping finishes just over
                    # 20 ms; every transport reply keeps its earlier timestamp.
                    logical_clock.advance_to(state['begin'] + delayed_elapsed_ns)
                    state['injected'] = True
                return None

        class LateReply(FakeSession):
            def __init__(self, first_id, **kwargs):
                super().__init__(first_id, **kwargs)
                self.cycle_type1_batches = 0

            def _exchange(self, wires, timeout_ns, send_only):
                records, stats = super()._exchange(wires, timeout_ns, send_only)
                if (state['index'] == 0 and len(records) == 6 and all(
                        codec.ATParser().feed(bytes(record.tx))[0].kind == 1
                        for record in records)):
                    self.cycle_type1_batches += 1
                if (late_reply and not state['injected'] and state['index'] == 0 and
                        self.cycle_type1_batches == 2):
                    late = state['begin'] + 20_050_000
                    records[-1].received_ns = late
                    records[-1].deadline_ns = late + 1_000_000
                    logical_clock.advance_to(late + 10_000)
                    state['injected'] = True
                return records, stats

        front = LateReply(1, clock=logical_clock)
        rear = FakeSession(7, clock=logical_clock)
        with patch.object(runtime._PendingCycleTiming, 'begin', capture_begin):
            report, sessions = self.run_case(profile_data=data, front=front, rear=rear,
                imu=FakeIMU(clock=logical_clock), policy=TimedPolicy(), clock=logical_clock,
                sleep=logical_clock.sleep, stop_requested=stop_requested)
        return report, sessions, state

    def test_strict_start_interval_is_distinct_from_21ms_wakeup_allowance(self):
        period = runtime.PERIOD_NS
        rows = [{'begin_ns': 1_000_000_000},
                {'begin_ns': 1_000_000_000 + period},
                {'begin_ns': 1_000_000_000 + 2*period + 500_000},
                {'begin_ns': 1_000_000_000 + 3*period + 1_500_001}]
        result = runtime._start_interval_metrics(rows)
        self.assertEqual(result['start_interval_count'], 3)
        self.assertEqual(result['start_intervals_over_20ms'], 2)
        self.assertEqual(result['start_intervals_over_21ms'], 1)
        self.assertEqual(result['max_start_interval_ms'], 21.000001)
        self.assertFalse(result['strict_start_interval_20ms_met'])
        self.assertFalse(runtime._start_interval_metrics(rows[:1])['strict_start_interval_20ms_met'])

    def test_success_runs_startup_policy_and_normal_stop_on_both_buses(self):
        # This case checks phase order, encoded output and both STOP owners.
        # Share causal simulated time across buses/IMU/coordinator so host
        # scheduling during the full suite cannot turn it into a timing test.
        # test_policy_stall_watchdog_stops_before_stalled_call_returns retains
        # real wall time and independently verifies asynchronous STOP.
        clock=SimulatedClock()
        report, sessions = self.run_case(front=FakeSession(1,clock=clock),
            rear=FakeSession(7,clock=clock),imu=FakeIMU(clock=clock),
            clock=clock,sleep=clock.sleep)
        self.assertEqual(report["status"], "COMPLETE_SUPPORTED_OUTPUT", report["errors"])
        self.assertTrue(report["motor_enable_sent"])
        self.assertTrue(report["learned_targets_sent"])
        self.assertTrue(report["normal_ramp_completed"])
        self.assertTrue(report["stop_confirmed"])
        phases = {row["phase"] for row in report["cycles"]}
        self.assertTrue({"starting", "active", "stopping", "stopped"} <= phases)
        self.assertEqual(report["cycles"][-1]["command"]["gain_scale"], 0.)
        self.assertTrue(all(session.positive_gain_writes > 0 for session in sessions.values()))
        self.assertFalse(report["full_controller_50Hz_verified"])
        setup=[r for b in report['journal'] if b['phase']=='watchdog_setup' for r in b['records']]
        readback=[r for b in report['journal'] if b['phase']=='watchdog_initial_readback' for r in b['records']]
        self.assertEqual(len(setup),12)
        self.assertTrue(all(r['received']==17 for r in setup))
        self.assertGreater(min(r['start_ns'] for r in readback),max(r['received_ns'] for r in setup))
        intervals = [right["begin_ns"] - left["begin_ns"]
                     for left, right in zip(report["cycles"], report["cycles"][1:])]
        self.assertEqual(report["start_interval_count"], len(intervals))
        self.assertEqual(report["start_intervals_over_20ms"],
                         sum(value > runtime.PERIOD_NS for value in intervals))
        self.assertEqual(report["start_intervals_over_21ms"],
                         sum(value > runtime.PERIOD_NS + 1_000_000 for value in intervals))
        self.assertEqual(report["max_start_interval_ms"], max(intervals) / 1e6)
        self.assertEqual(report["strict_start_interval_20ms_met"],
                         all(value <= runtime.PERIOD_NS for value in intervals))
        for cycle in report["cycles"]:
            self.assertLessEqual(cycle["output_reply_end_ns"], cycle["output_exchange_return_ns"])
            self.assertLessEqual(cycle["output_exchange_return_ns"], cycle["end_ns"])
            self.assertEqual(cycle["iteration_ms"], (cycle["end_ns"] - cycle["begin_ns"]) / 1e6)
            self.assertEqual(cycle["post_output_processing_ms"],
                             (cycle["end_ns"] - cycle["output_exchange_return_ns"]) / 1e6)

    def test_logged_positive_and_negative_power_branches_reach_inverse_wires(self):
        # This checks recorded encoder branches and inverse wire encoding;
        # independent watchdog timing is exercised by dedicated real-time tests.
        clock=SimulatedClock()
        front, rear = FakeSession(1,clock=clock), FakeSession(7,clock=clock)
        front.positions[3] = 6.262798309326172  # Recorded ID3 +361.307 degree case.
        rear.positions[9] = -2 * math.pi + .08  # ID9 negative-turn branch.
        reviewed = profile()
        reviewed['axes']['9']['sign'] = -1
        report, sessions = self.run_case(front=front, rear=rear,
            profile_data=reviewed,imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertEqual(report['fixed_branch_turns_by_id'][3], 1)
        self.assertEqual(report['fixed_branch_turns_by_id'][9], -1)
        self.assertEqual(report['fixed_branch_motor_power_epoch'], 'synthetic-power-epoch-1')
        first_feedback = report['cycles'][0]['feedback']['q_model_rad']
        self.assertAlmostEqual(first_feedback[2], 6.262798309326172 - 2 * math.pi,
                               delta=.002)
        self.assertAlmostEqual(first_feedback[8], -.08,
                               delta=.002)
        for motor_id, session, branch_side in ((3, sessions['front'], 1),
                                                (9, sessions['rear'], -1)):
            raw_wires = [int.from_bytes(request[7:9], 'big') * 25.14 / 65535 - 12.57
                         for _, kind, mid, request in session.calls
                         if kind == 1 and mid == motor_id]
            self.assertTrue(raw_wires)
            self.assertTrue(all(value * branch_side > 5 for value in raw_wires))
            sign = reviewed['axes'][str(motor_id)]['sign']
            offset = report['fixed_offsets_rad_by_id'][motor_id]
            self.assertEqual(len(raw_wires), 2 + 2 * len(report['cycles']))
            for cycle_index, cycle in enumerate(report['cycles']):
                encoded_output = raw_wires[3 + 2 * cycle_index]
                self.assertAlmostEqual(sign * encoded_output + offset,
                                       cycle['command']['q_model_rad'][motor_id - 1],
                                       delta=.0005)

    def test_unknown_epoch_and_uncertain_initial_branch_stop_before_enable(self):
        for change in ('epoch', 'uncertainty'):
            with self.subTest(change=change):
                reviewed = profile()
                if change == 'epoch':
                    reviewed['motor_power_epoch'] = 'NOT_INFERRED_FROM_JETSON_BOOT'
                else:
                    reviewed['axes']['3']['physical_upper_rad'] = .004
                    reviewed['axes']['3']['uncertainty_rad'] = .01
                report, sessions = self.run_case(profile_data=reviewed)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertFalse(report['motor_enable_sent'])
                self.assertTrue(all(session.positive_gain_writes == 0
                                    for session in sessions.values()))
                self.assertTrue(any(('power epoch' in error or 'branch' in error)
                                    for error in report['errors']), report['errors'])

    def test_all_axes_both_signs_keep_current_power_branch_in_encoded_targets(self):
        # Every axis uses -360, zero and +360 raw branches with both signs.
        # Compare encoded Type1 bytes with the requested model angle, not only
        # the reported offset, so a read-only modulo fix cannot satisfy this.
        for sign in (-1, 1):
            for rotation in range(3):
                with self.subTest(sign=sign, rotation=rotation):
                    clock=SimulatedClock()
                    front, rear = FakeSession(1,clock=clock), FakeSession(7,clock=clock)
                    reviewed = profile()
                    reviewed['motor_power_epoch'] = f'new-power-{sign}-{rotation}'
                    # Scheduler variability is covered elsewhere, not this
                    # hardware-free angle/command conversion matrix.
                    turns = {mid: (mid + rotation) % 3 - 1 for mid in runtime.IDS}
                    initial_raw = {}
                    for mid in runtime.IDS:
                        axis = reviewed['axes'][str(mid)]
                        axis['sign'] = sign
                        axis['offset_rad'] = .005 * mid
                        initial_raw[mid] = ((.03 - axis['offset_rad']) / sign
                                            + turns[mid] * 2 * math.pi)
                        (front if mid <= 6 else rear).positions[mid] = initial_raw[mid]
                    report, sessions = self.run_case(front=front, rear=rear,
                        profile_data=reviewed,imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
                    self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
                    self.assertEqual(report['fixed_branch_turns_by_id'], turns)
                    self.assertEqual(report['fixed_branch_motor_power_epoch'],
                                     reviewed['motor_power_epoch'])
                    for mid in runtime.IDS:
                        session = sessions['front' if mid <= 6 else 'rear']
                        raw_wires = [int.from_bytes(request[7:9], 'big') * 25.14 / 65535 - 12.57
                                     for _, kind, motor, request in session.calls
                                     if kind == 1 and motor == mid]
                        self.assertEqual(len(raw_wires), 2 + 2 * len(report['cycles']))
                        self.assertTrue(all(abs(raw - initial_raw[mid]) < .175 for raw in raw_wires))
                        for index, cycle in enumerate(report['cycles']):
                            encoded_model = (sign * (raw_wires[3 + 2 * index] - turns[mid] * 2 * math.pi)
                                             + reviewed['axes'][str(mid)]['offset_rad'])
                            # One wire LSB plus double-precision roundoff from the
                            # +/-2pi offset subtraction and model reconstruction.
                            self.assertAlmostEqual(encoded_model, cycle['command']['q_model_rad'][mid - 1],
                                                   delta=25.14 / 65535 + 8 * math.ulp(25.14))
                            self.assertLess(abs(cycle['feedback']['q_model_rad'][mid - 1] - .03), .02)

    def test_every_live_axis_turn_jump_aborts_and_stops_both_buses(self):
        # Byte corruption and branch ownership are semantic assertions. Keep
        # real host scheduling from stopping a fake phase before its injected
        # positive-gain reply; dedicated watchdog tests retain real time.
        for motor_id in runtime.IDS:
            for jump in (-2 * math.pi, 2 * math.pi):
                with self.subTest(mid=motor_id, jump=jump):
                    clock=SimulatedClock()
                    class JumpSession(FakeSession):
                        injected = False

                        def _exchange(self, wires, timeout_ns, send_only):
                            result = super()._exchange(wires, timeout_ns, send_only)
                            for record in result[0]:
                                request = codec.ATParser().feed(bytes(record.tx))[0]
                                if (not self.injected and request.kind == 1
                                        and request.destination == motor_id
                                        and int.from_bytes(request.data[4:6], 'big') > 0):
                                    response = bytearray(record.rx)
                                    raw = int.from_bytes(response[7:9], 'big') * 25.14 / 65535 - 12.57
                                    response[7:9] = quantize(raw + jump, -12.57, 12.57).to_bytes(2, 'big')
                                    record.rx[:] = response
                                    self.injected = True
                            return result

                    subject = JumpSession(1 if motor_id <= 6 else 7,clock=clock)
                    with patch.object(runtime,'wait',side_effect=causal_semantic_wait):
                        report, sessions = self.run_case(
                            front=subject if motor_id <= 6 else FakeSession(1,clock=clock),
                            rear=subject if motor_id > 6 else FakeSession(7,clock=clock),
                            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
                    self.assertTrue(subject.injected)
                    self.assertEqual(report['status'], 'ABORTED')
                    self.assertTrue(any(f'ID{motor_id} raw position discontinuity' in error
                                        for error in report['errors']), report['errors'])
                    self.assertTrue(report['stop_confirmed'])
                    self.assertTrue(all(not session.enabled for session in sessions.values()))

    def test_active_feedback_turn_jump_is_rejected_without_rebinding(self):
        reviewed = profile()
        old = {mid: SimpleNamespace(mode_state=2, fault_bits=0,
                                    protocol_position_rad=.1, velocity_rad_s=0.,
                                    torque_nm=0., temperature_c=25.)
               for mid in runtime.IDS}
        new = {mid: SimpleNamespace(**vars(value)) for mid, value in old.items()}
        new[3].protocol_position_rad += 2 * math.pi
        previous = {(mid, 'feedback'): (old[mid], 1_000_000_000, 1_000_000_100)
                    for mid in runtime.IDS}
        current = {(mid, 'feedback'): (new[mid], 1_020_000_000, 1_020_000_100)
                   for mid in runtime.IDS}
        with self.assertRaisesRegex(RuntimeError, 'ID3 raw position discontinuity'):
            runtime.feedback_sample(current, reviewed,
                                    {mid: 0. for mid in runtime.IDS},
                                    now_ns=1_020_000_200, previous=previous)

    def test_opt_in_r22_runs_after_workers_and_restores_only_main_affinity_after_stop(self):
        # Verify affinity ownership and ordering, independently of host load.
        clock=SimulatedClock()
        front,rear=FakeSession(1,clock=clock),FakeSession(7,clock=clock)
        main_ident=threading.get_ident();pinned=[False];events=[]
        class Startup:
            def __call__(self_inner,*_args):return (.04,)*12
            def pre_pin_warmup(self_inner):
                self.assertFalse(pinned[0]);events.append('warmup')
            def post_pin_prime(self_inner):
                self.assertTrue(pinned[0]);events.append('prime')
            def finish_startup(self_inner):
                self.assertTrue(pinned[0]);events.append('reset')
        def get_affinity(_pid):
            return {4} if threading.get_ident()==main_ident and pinned[0] else {0,4}
        def set_affinity(_pid,mask):
            self.assertEqual(threading.get_ident(),main_ident)
            pinned[0]=set(mask)=={4}
            if not pinned[0]:
                self.assertTrue(all(session.stop_times for session in (front,rear)),
                                'Both bus owners must stop before main affinity restoration')
            events.append('pin' if pinned[0] else 'restore')
        stop=threading.Event();stop.set()
        startup=Startup()
        with patch.object(runtime.os,'sched_getaffinity',side_effect=get_affinity,create=True), \
             patch.object(runtime.os,'sched_setaffinity',side_effect=set_affinity,create=True):
            report,sessions=self.run_case(stop_requested=stop,policy=startup,startup_model=startup,
                main_thread_cpu=4,pre_cycle_policy_warmup_calls=10,
                post_pin_policy_prime_calls=10,announce=lambda:events.append('announce'),
                front=front,rear=rear,imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(events,['announce','warmup','pin','prime','reset','restore'])
        self.assertFalse(pinned[0]);self.assertTrue(report['stop_confirmed'])
        self.assertTrue(report['setup_policy_warmup']['complete'])
        self.assertTrue(report['setup_policy_prime']['complete'])
        self.assertTrue(report['setup_policy_prime']['policy_reset_after'])
        affinity=report['main_thread_affinity']
        self.assertEqual(affinity['during'],[4]);self.assertTrue(affinity['restored'])
        self.assertEqual(set(affinity['worker_masks_after_pin']),{'front','rear','imu'})
        self.assertTrue(all(row['cpus']==[0,4] for row in affinity['worker_masks_after_pin'].values()))
        self.assertTrue(all(session.stop_times[0]<clock() for session in sessions.values()))

    def test_r22_warmup_failure_aborts_before_enable_and_still_stops_and_restores(self):
        main_ident=threading.get_ident();masks=[]
        class FailingStartup:
            def __call__(self_inner,*_args):raise AssertionError('Policy must not run')
            def pre_pin_warmup(self_inner):raise RuntimeError('Synthetic warmup failure')
            def post_pin_prime(self_inner):raise AssertionError('Prime must not run')
            def finish_startup(self_inner):raise AssertionError('Reset must not run')
        def get_affinity(_pid):return {0,4}
        def set_affinity(_pid,mask):
            self.assertEqual(threading.get_ident(),main_ident);masks.append(set(mask))
        startup=FailingStartup()
        with patch.object(runtime.os,'sched_getaffinity',side_effect=get_affinity,create=True), \
             patch.object(runtime.os,'sched_setaffinity',side_effect=set_affinity,create=True):
            report,sessions=self.run_case(policy=startup,startup_model=startup,main_thread_cpu=4,
                pre_cycle_policy_warmup_calls=10,post_pin_policy_prime_calls=10)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Synthetic warmup failure',report['errors'][0])
        self.assertFalse(report['setup_policy_warmup']['complete'])
        self.assertIsNotNone(report['setup_policy_warmup']['end_ns'])
        self.assertIsNone(report['setup_policy_prime']['begin_ns'])
        self.assertEqual(masks,[{0,4}]);self.assertTrue(report['main_thread_affinity']['restored'])
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(all(all(call[1]!=3 for call in session.calls) for session in sessions.values()))
        self.assertTrue(report['stop_confirmed'])

    def test_r22_rejects_warming_a_different_model_before_transport(self):
        front=FakeSession(1);rear=FakeSession(7)
        class Startup:
            def pre_pin_warmup(self):pass
            def post_pin_prime(self):pass
            def finish_startup(self):pass
        with self.assertRaisesRegex(RuntimeError,'R22 startup requires'):
            runtime.run_supported_policy(profile(),{'front':front,'rear':rear},FakeIMU(),
                lambda *_:(.04,)*12,cancel_io=lambda:None,encode_motion=encode_motion,
                startup_model=Startup(),main_thread_cpu=4,pre_cycle_policy_warmup_calls=10)
        self.assertEqual(front.calls,[]);self.assertEqual(rear.calls,[])

    def test_opt_in_gc_deferral_restores_after_stop_even_when_cycle_fails(self):
        automatic=[True];threshold=[700,10,10];events=[]
        class StopObservedSession(FakeSession):
            def emergency_stop(self_inner):
                events.append(('stop',automatic[0]))
                return super().emergency_stop()
        def disable():automatic[0]=False;events.append(('disable',False))
        def enable():automatic[0]=True;events.append(('enable',True))
        def set_threshold(*values):threshold[:]=values
        def failing_policy(*_args):
            self.assertFalse(automatic[0]);raise RuntimeError('Synthetic cycle failure')
        with patch.object(runtime.gc,'isenabled',side_effect=lambda:automatic[0]), \
             patch.object(runtime.gc,'get_threshold',side_effect=lambda:tuple(threshold)), \
             patch.object(runtime.gc,'disable',side_effect=disable), \
             patch.object(runtime.gc,'enable',side_effect=enable), \
             patch.object(runtime.gc,'set_threshold',side_effect=set_threshold):
            report,_=self.run_case(front=StopObservedSession(1),rear=StopObservedSession(7),
                policy=failing_policy,defer_gc_during_cycles=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual([name for name,_ in events],['disable','stop','stop','enable'])
        self.assertFalse(events[1][1]);self.assertFalse(events[2][1])
        self.assertTrue(automatic[0]);self.assertTrue(report['cycle_gc_defer']['restored'])
        self.assertEqual(report['cycle_gc_defer']['after_threshold'],(700,10,10))

    def test_gc_disabled_at_entry_rejects_before_transport_or_enable(self):
        front=FakeSession(1);rear=FakeSession(7)
        with patch.object(runtime.gc,'isenabled',return_value=False):
            with self.assertRaisesRegex(RuntimeError,'already disabled'):
                runtime.run_supported_policy(profile(),{'front':front,'rear':rear},FakeIMU(),
                    lambda *_:(.04,)*12,cancel_io=lambda:None,encode_motion=encode_motion,
                    defer_gc_during_cycles=True)
        self.assertEqual(front.calls,[]);self.assertEqual(rear.calls,[])

    def test_gc_restores_and_reports_unconfirmed_if_stop_collection_raises(self):
        automatic=[True]
        original=runtime.BusWorkers.finish_stops
        def broken_stop_collection(workers):
            original(workers)
            raise RuntimeError('Synthetic STOP collection failure')
        def policy_failure(*_args):raise RuntimeError('Synthetic cycle failure')
        with patch.object(runtime.gc,'isenabled',side_effect=lambda:automatic[0]), \
             patch.object(runtime.gc,'get_threshold',return_value=(700,10,10)), \
             patch.object(runtime.gc,'set_threshold'), \
             patch.object(runtime.gc,'disable',side_effect=lambda:automatic.__setitem__(0,False)), \
             patch.object(runtime.gc,'enable',side_effect=lambda:automatic.__setitem__(0,True)), \
             patch.object(runtime.BusWorkers,'finish_stops',broken_stop_collection):
            report,_=self.run_case(policy=policy_failure,defer_gc_during_cycles=True)
        self.assertTrue(automatic[0]);self.assertTrue(report['cycle_gc_defer']['restored'])
        self.assertEqual(report['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertTrue(any('STOP collection' in error for error in report['errors']))

    def test_delayed_stop_keeps_unconfirmed_status_after_gc_restoration(self):
        automatic=[True]
        class SlowStopSession(FakeSession):
            def emergency_stop(self_inner):
                # Exceed the extended shared 1.25-second collection deadline.
                time.sleep(1.4)
                return super().emergency_stop()
        stop=threading.Event();stop.set()
        with patch.object(runtime.gc,'isenabled',side_effect=lambda:automatic[0]), \
             patch.object(runtime.gc,'get_threshold',return_value=(700,10,10)), \
             patch.object(runtime.gc,'set_threshold'), \
             patch.object(runtime.gc,'disable',side_effect=lambda:automatic.__setitem__(0,False)), \
             patch.object(runtime.gc,'enable',side_effect=lambda:automatic.__setitem__(0,True)):
            report,_=self.run_case(rear=SlowStopSession(7),stop_requested=stop,
                defer_gc_during_cycles=True)
        self.assertTrue(automatic[0]);self.assertTrue(report['cycle_gc_defer']['restored'])
        self.assertEqual(report['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertEqual(report['stop_reports']['rear']['unconfirmed_ids'],list(runtime.BUSES['rear']))

    def test_uid_mismatch_fails_before_enable_and_stops_both_buses(self):
        report, sessions = self.run_case(front=FakeSession(1, uid_mismatch=True))
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("UID mismatch" in error for error in report["errors"]))
        self.assertFalse(report["motor_enable_sent"])
        self.assertTrue(all(all(call[1] != 3 for call in session.calls) for session in sessions.values()))
        self.assertTrue(report["stop_confirmed"])

    def test_firmware_matches_raw_bytes_before_enable_without_semantic_guess(self):
        report,sessions=self.run_case()
        self.assertTrue(report['firmware_versions_match_watchdog_review'])
        self.assertEqual(set(report['firmware_versions_by_id']),{str(i) for i in range(1,13)})
        for mid,row in report['firmware_versions_by_id'].items():
            self.assertEqual(row['version_bytes_hex'],'05001300')
            self.assertEqual(row['version_bytes'],[5,0,19,0])
            self.assertIsNone(row['semantic_firmware_version'])
            self.assertEqual(row['unspecified_byte7'],165)
            self.assertEqual(row['uid'],(bytes([int(mid)])*8).hex())
            self.assertTrue(row['matches_tested_firmware'])
            self.assertLess(row['request_started_monotonic_ns'],row['received_monotonic_ns'])
        for session in sessions.values():
            version_calls=[c for c in session.calls if c[1]==4 and codec.ATParser().feed(c[3])[0].data==runtime.versions.VERSION_PAYLOAD]
            self.assertEqual(len(version_calls),6)  # Once at preflight, not every cycle.
            self.assertLess(max(c[0] for c in version_calls),min(c[0] for c in session.calls if c[1]==3))

    def test_raw_firmware_mismatch_stops_before_enable_preserving_evidence(self):
        report,sessions=self.run_case(rear=FakeSession(7,firmware_bytes='01020304'))
        self.assertTrue(any('fresh firmware bytes differ' in e for e in report['errors']))
        self.assertFalse(report['motor_enable_attempted'])
        self.assertFalse(report['firmware_versions_match_watchdog_review'])
        self.assertEqual(report['firmware_versions_by_id']['7']['version_bytes_hex'],'01020304')
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(all(all(c[1]!=3 for c in s.calls) for s in sessions.values()))

    def test_semantic_version_without_raw_fingerprint_cannot_start(self):
        p=profile();p['watchdog_by_id']['1']={'firmware_version':'0.5.0.13'}
        report,sessions=self.run_case(profile_data=p)
        self.assertFalse(report['motor_enable_attempted'])
        self.assertTrue(any('raw firmware fingerprint' in e for e in report['errors']))
        self.assertTrue(all(not s.calls for s in sessions.values()))
        self.assertTrue(report['stop_confirmed'])

    def test_normal_feedback_or_timeout_cannot_substitute_for_version(self):
        for options in ({'version_returns_feedback':True},{'version_timeout':True}):
            with self.subTest(options=options):
                report,sessions=self.run_case(front=FakeSession(1,**options))
                self.assertEqual(report['status'],'ABORTED')
                self.assertFalse(report['motor_enable_attempted'])
                self.assertFalse(report['firmware_versions_match_watchdog_review'])
                self.assertTrue(report['stop_confirmed'])
                self.assertTrue(all(all(c[1]!=3 for c in s.calls) for s in sessions.values()))

    def test_policy_exception_stops_both_buses(self):
        def policy_error(*args):
            raise RuntimeError("Synthetic inference failed")
        report, _ = self.run_case(policy=policy_error)
        self.assertTrue(any("Synthetic inference failed" in error for error in report["errors"]))
        self.assertTrue(report["stop_confirmed"])
        self.assertFalse(report["normal_ramp_completed"])

    def test_policy_stall_watchdog_stops_before_stalled_call_returns(self):
        end = []
        def stalled_policy(*args):
            time.sleep(.2)
            end.append(time.monotonic_ns())
            return (.04,) * 12
        report, sessions = self.run_case(policy=stalled_policy)
        self.assertEqual(report["status"], "ABORTED")
        self.assertIn("heartbeat", report["host_watchdog_reason"])
        self.assertTrue(all(session.stop_times[0] < end[0] for session in sessions.values()))
        self.assertTrue(report["stop_confirmed"])

    def test_frozen_imu_is_not_reused_for_a_second_policy_cycle(self):
        report, _ = self.run_case(imu=FakeIMU(frozen=True))
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("Repeated" in error and "IMU" in error
                            for error in report["errors"]), report["errors"])
        self.assertLessEqual(len(report["cycles"]), 1)
        self.assertTrue(report["stop_confirmed"])

    def test_returned_feedback_torque_limit_stops_before_command_is_reused(self):
        front = FakeSession(1, high_returned_torque=True)
        report, sessions = self.run_case(front=front)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("torque" in error.lower() for error in report["errors"]), report["errors"])
        self.assertEqual(front.positive_gain_writes, 6)
        self.assertFalse(report["learned_targets_attempted"])
        self.assertFalse(report["learned_targets_sent"])
        self.assertTrue(report["command_output_sent"])
        self.assertTrue(report["motion_gain_sent"])
        self.assertTrue(report["stop_confirmed"])

    def test_owner_decoding_rejects_bad_output_reply_before_next_command(self):
        class BadOutputReply(FakeSession):
            def __init__(self, first_id):
                super().__init__(first_id)
                self.bad_reply_sent = False
                self.motion_batches = 0

            def _exchange(self, wires, timeout_ns, send_only):
                records, stats = super()._exchange(wires, timeout_ns, send_only)
                if len(records) == 6 and all(
                        codec.ATParser().feed(bytes(record.tx))[0].kind == 1
                        for record in records):
                    self.motion_batches += 1
                    if self.motion_batches == 2:
                        records[-1].rx[-1] = 0  # Invalid native frame trailer.
                        self.bad_reply_sent = True
                return records, stats

        front = BadOutputReply(1)
        report, sessions = self.run_case(front=front)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(front.bad_reply_sent)
        self.assertTrue(any("Invalid native frame" in error for error in report["errors"]),
                        report["errors"])
        self.assertFalse(report["cycles"])
        self.assertTrue(report["stop_confirmed"])
        # No owner may send another Type1 after the corrupted output batch.
        output_batches = [batch for batch in report['journal']
                          if batch['phase'] in ('startup_hold', 'policy_output', 'graceful_stop')]
        self.assertLessEqual(len(output_batches), 2)
        self.assertTrue(all(batch['phase'] == 'startup_hold' for batch in output_batches))
        self.assertEqual(front.motion_batches, 2)

    def test_fault_after_policy_blend_preserves_actual_learned_write_evidence(self):
        report, _ = self.run_case(front=FakeSession(1, high_returned_torque=True, torque_fault_after=120))
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("torque" in error.lower() for error in report["errors"]), report["errors"])
        self.assertTrue(report["learned_targets_attempted"])
        self.assertTrue(report["learned_targets_sent"])
        self.assertTrue(report["motion_gain_sent"])
        self.assertTrue(report["stop_confirmed"])

    def test_usb_exception_attempts_independent_stop_on_other_bus(self):
        report, _ = self.run_case(front=FakeSession(1, fail_motion=True))
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("Synthetic USB lost" in error for error in report["errors"]))
        self.assertTrue(report["stop_confirmed"])

    def test_one_unconfirmed_usb_stop_is_not_reported_as_safe(self):
        report, _ = self.run_case(rear=FakeSession(7, stop_unconfirmed=True),
                                 policy=lambda *args: (_ for _ in ()).throw(RuntimeError("abort")))
        self.assertEqual(report["status"], "STOP_UNCONFIRMED_POWER_OFF_REQUIRED")
        self.assertFalse(report["stop_confirmed"])
        self.assertEqual(report["stop_reports"]["rear"]["unconfirmed_ids"], [12])

    def test_cycle_overrun_is_not_hidden_by_next_release(self):
        def late_policy(*args):
            time.sleep(.03)
            return (.04,) * 12
        report, _ = self.run_case(policy=late_policy)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("timing misses" in error for error in report["errors"]), report["errors"])
        self.assertGreater(report["deadline20ms_misses"], 0)
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(report["start_interval_count"], 0)
        self.assertFalse(report["strict_start_interval_20ms_met"])

    def test_post_reply_validation_time_counts_toward_same_cycle_deadline(self):
        # Advance only the coordinator's clock at the post-output validation
        # boundary. This deterministically models 30 ms of work after native
        # replies, without a scheduler-dependent sleep or a hardware claim.
        offset = [0]
        original = runtime.feedback_sample

        def slow_returned_feedback(*args, **kwargs):
            sample = original(*args, **kwargs)
            previous = kwargs.get("previous") or {}
            if any(name == "voltage" for _, name in previous):
                offset[0] += 30_000_000
            return sample

        with patch.object(runtime, "feedback_sample", side_effect=slow_returned_feedback):
            report, _ = self.run_case(clock=lambda: time.monotonic_ns() + offset[0])
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("timing misses" in error for error in report["errors"]), report["errors"])
        self.assertEqual(len(report["cycles"]), 1)
        cycle = report["cycles"][0]
        self.assertGreaterEqual(cycle["post_output_processing_ms"], 30.)
        self.assertGreaterEqual(cycle["iteration_ms"], 30.)
        self.assertTrue(cycle["deadline20ms_missed"])
        self.assertFalse(report["normal_ramp_completed"])
        self.assertTrue(report["stop_confirmed"])

    def test_owner_decode_time_counts_toward_same_cycle_deadline(self):
        offset = [0]
        original = runtime.BusWorkers._exchange_decoded

        def slow_decoded_reply(worker, *args):
            result = original(worker, *args)
            if not offset[0]:
                offset[0] = 30_000_000
            return result

        with patch.object(runtime.BusWorkers, "_exchange_decoded", slow_decoded_reply):
            report, _ = self.run_case(clock=lambda: time.monotonic_ns() + offset[0])
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("timing misses" in error for error in report["errors"]),
                        report["errors"])
        self.assertEqual(len(report["cycles"]), 1)
        cycle = report["cycles"][0]
        self.assertGreaterEqual(cycle["output_exchange_return_ns"] -
                                cycle["output_reply_end_ns"], 30_000_000)
        self.assertGreaterEqual(cycle["iteration_ms"], 30.)
        self.assertTrue(cycle["deadline20ms_missed"])
        self.assertTrue(report["stop_confirmed"])

    def test_reviewed_startup_allows_only_post_reply_work_past_20ms(self):
        report, _, state = self.run_measured_startup_timing_case(delayed_cycle=0)
        self.assertTrue(state['injected'])
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertTrue(report['startup_20ms_allowance_enabled'])
        self.assertGreater(len(report['cycles']), 1)
        first = report['cycles'][0]
        self.assertGreater(first['iteration_ms'], 20.)
        self.assertLessEqual(first['iteration_ms'], 21.)
        self.assertLessEqual(first['output_reply_end_ns'] - first['begin_ns'], runtime.PERIOD_NS)
        self.assertTrue(first['deadline20ms_missed'])
        self.assertTrue(first['startup_20ms_allowance_used'])
        self.assertFalse(first['steady_deadline20ms_missed'])
        self.assertEqual(report['startup_20ms_misses'], 1)
        self.assertEqual(report['startup_20ms_allowance_uses'], 1)
        self.assertEqual(report['steady_deadline20ms_misses'], 0)
        self.assertEqual(report['deadline20ms_misses'], 1)
        self.assertFalse(report['full_controller_50Hz_verified'])

    def test_reviewed_startup_allowance_does_not_cover_next_command_gap(self):
        report, sessions, state = self.run_measured_startup_timing_case(
            delayed_cycle=0, second_cycle_processing_delay_ns=2_000_000)
        self.assertTrue(state['injected'])
        self.assertTrue(state['second_cycle_delayed'])
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(len(report['cycles']), 1)
        self.assertTrue(report['cycles'][0]['startup_20ms_allowance_used'])
        trace = report['failed_cycle_timing']
        self.assertEqual(trace['index'], 1)
        self.assertEqual(trace['stage'], 'motion_envelope')
        self.assertGreater(trace['command_interval_ms'], 21.)
        self.assertIsNone(trace['output_submit_ns'])
        self.assertTrue(any('Command gap exceeded' in error for error in report['errors']),
                        report['errors'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(all(session.stop_times[0] > trace['candidate_ns']
                            for session in sessions.values()))

    def test_reviewed_startup_allowance_never_covers_second_cycle(self):
        report, _, state = self.run_measured_startup_timing_case(delayed_cycle=1)
        self.assertTrue(state['injected'])
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles'][-1]['index'], 1)
        self.assertTrue(report['cycles'][-1]['deadline20ms_missed'])
        self.assertFalse(report['cycles'][-1]['startup_20ms_allowance_used'])
        self.assertTrue(report['cycles'][-1]['steady_deadline20ms_missed'])
        self.assertEqual(report['steady_deadline20ms_misses'], 1)
        self.assertTrue(any('Output cycle exceeded hard deadline' in error
                            for error in report['errors']), report['errors'])
        self.assertTrue(report['stop_confirmed'])

    def test_reviewed_startup_rejects_reply_after_20ms(self):
        report, _, state = self.run_measured_startup_timing_case(late_reply=True)
        self.assertTrue(state['injected'])
        self.assertEqual(report['status'], 'ABORTED')
        # The native owner now receives the original absolute deadline, so an
        # over-deadline reply is rejected before a validated cycle is recorded.
        self.assertEqual(report['cycles'], [])
        self.assertEqual(report['failed_cycle_timing']['stage'], 'output_exchange')
        self.assertEqual(report['startup_20ms_misses'], 0)
        self.assertEqual(report['startup_20ms_allowance_uses'], 0)
        self.assertTrue(any('Incomplete or noncausal motor transaction' in error
                            for error in report['errors']), report['errors'])
        self.assertTrue(report['stop_confirmed'])

    def test_reviewed_startup_keeps_oldest_input_age_and_21ms_ceiling(self):
        for elapsed_ns, maximum_ms in ((20_500_000, 21.), (21_100_000, 22.)):
            with self.subTest(elapsed_ns=elapsed_ns):
                report, _, state = self.run_measured_startup_timing_case(
                    delayed_cycle=0, delayed_elapsed_ns=elapsed_ns)
                self.assertTrue(state['injected'])
                self.assertEqual(report['status'], 'ABORTED')
                self.assertGreater(report['cycles'][0]['iteration_ms'], elapsed_ns/1e6)
                self.assertLess(report['cycles'][0]['iteration_ms'], maximum_ms)
                self.assertFalse(report['cycles'][0]['startup_20ms_allowance_used'])
                self.assertTrue(any('sample-age deadline' in error for error in report['errors']),
                                report['errors'])

    def test_unreviewed_route_keeps_first_cycle_hard_20ms(self):
        report, _, state = self.run_measured_startup_timing_case(
            delayed_cycle=0, acceptance=False)
        self.assertTrue(state['injected'])
        self.assertEqual(report['status'], 'ABORTED')
        self.assertFalse(report['startup_20ms_allowance_enabled'])
        self.assertFalse(report['cycles'][0]['startup_20ms_allowance_used'])
        self.assertTrue(any('Output cycle exceeded hard deadline' in error
                            for error in report['errors']), report['errors'])

    def test_cycle_metric_assembly_is_not_excluded_from_iteration_time(self):
        offset = [0]

        class SlowMetricsPolicy:
            def __call__(self, *args):
                return (.04,) * 12

            @property
            def last_validation(self):
                # This property is consumed while assembling cycle evidence,
                # after native feedback has already passed its limit checks.
                offset[0] += 30_000_000
                return None

        report, _ = self.run_case(policy=SlowMetricsPolicy(),
                                 clock=lambda: time.monotonic_ns() + offset[0])
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("timing misses" in error for error in report["errors"]), report["errors"])
        self.assertEqual(len(report["cycles"]), 1)
        self.assertGreaterEqual(report["cycles"][0]["post_output_processing_ms"], 30.)
        self.assertTrue(report["cycles"][0]["deadline20ms_missed"])
        self.assertTrue(report["stop_confirmed"])

    def test_final_stopped_cycle_cannot_complete_after_post_reply_hard_deadline(self):
        offset = [0]
        phase = [None]
        original_step = runtime.PolicyMotionEnvelope.step

        def record_phase(envelope, *args, **kwargs):
            command = original_step(envelope, *args, **kwargs)
            phase[0] = command.phase
            return command

        class SlowFinalMetricsPolicy:
            def __call__(self, *args):
                raise AssertionError("A requested stop must not invoke inference")

            @property
            def last_validation(self):
                if phase[0] == "stopped":
                    offset[0] += 55_000_000
                return None

        stop = threading.Event(); stop.set()
        data = profile()
        # Isolate the hard deadline from the separate consecutive-20ms gate.
        data["max_consecutive_20ms_misses"] = 100
        with patch.object(runtime.PolicyMotionEnvelope, "step", record_phase):
            report, _ = self.run_case(policy=SlowFinalMetricsPolicy(), profile_data=data,
                stop_requested=stop, clock=lambda: time.monotonic_ns() + offset[0])
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("Output cycle exceeded hard deadline" in error
                            for error in report["errors"]), report["errors"])
        self.assertEqual(report["cycles"][-1]["phase"], "stopped")
        self.assertGreaterEqual(report["cycles"][-1]["post_output_processing_ms"], 55.)
        self.assertTrue(report["cycles"][-1]["deadline20ms_missed"])
        self.assertFalse(report["normal_ramp_completed"])
        self.assertTrue(report["stop_confirmed"])

    def test_external_cancellation_stops_before_policy_command_send(self):
        cancellation = threading.Event()
        def policy_cancel(*args):
            cancellation.set()
            return (.04,) * 12
        def check():
            if cancellation.is_set():
                raise RuntimeError("External cancellation")
        report, sessions = self.run_case(policy=policy_cancel, check=check)
        self.assertTrue(any("External cancellation" in error for error in report["errors"]))
        self.assertTrue(all(session.positive_gain_writes == 0 for session in sessions.values()))
        self.assertTrue(report["stop_confirmed"])

    def test_requested_normal_stop_does_not_invoke_or_resume_policy(self):
        # The stop event is already set: policy isolation is causal, not timed.
        clock=SimulatedClock()
        stop = threading.Event(); stop.set()
        calls = []
        def unused_policy(*args):
            calls.append(True)
            return (.04,) * 12
        report, sessions = self.run_case(policy=unused_policy, stop_requested=stop,
            front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)
        self.assertEqual(report["status"], "COMPLETE_SUPPORTED_OUTPUT", report["errors"])
        self.assertEqual(calls, [])
        self.assertTrue(report["normal_ramp_completed"])
        self.assertFalse(report["learned_targets_sent"])
        self.assertTrue(all(session.positive_gain_writes == 0 for session in sessions.values()))

    def test_out_of_range_learned_target_is_rejected_before_policy_blending(self):
        report, sessions = self.run_case(policy=lambda *args: (1.1,) * 12)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("learned target outside physical range" in error
                            for error in report["errors"]), report["errors"])
        self.assertTrue(all(session.positive_gain_writes == 0 for session in sessions.values()))
        self.assertTrue(report["stop_confirmed"])

    def test_future_imu_metadata_is_rejected_before_policy(self):
        def future_imu():
            value = FakeIMU()()
            value["read_started_monotonic_ns"] += 1_000_000_000
            value["read_finished_monotonic_ns"] += 1_000_000_000
            return value
        report, _ = self.run_case(imu=future_imu)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("noncausal IMU" in error for error in report["errors"]), report["errors"])
        self.assertFalse(report["learned_targets_sent"])
        self.assertFalse(report["motor_enable_sent"])
        self.assertTrue(report["stop_confirmed"])

    def test_quantized_boundary_outside_reviewed_range_is_not_sent(self):
        data = profile()
        for axis in data["axes"].values():
            axis["lower_rad"] = .0001
        report, sessions = self.run_case(profile_data=data, policy=lambda *args: (.0001,) * 12)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("quantized target outside physical range" in error
                            for error in report["errors"]), report["errors"])
        for session in sessions.values():
            for _, kind, _, request in session.calls:
                if kind == 1:
                    q = int.from_bytes(request[7:9], "big") * 25.14 / 65535 - 12.57
                    self.assertGreaterEqual(q, .0001)
        self.assertTrue(report["stop_confirmed"])

    def test_mode_zero_stop_with_fault_is_confirmed_but_trial_is_not_passed(self):
        report, _ = self.run_case(rear=FakeSession(7, stop_fault=True))
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(report["status"], "ABORTED_STOP_FAULT")
        self.assertEqual(set(report["stop_faults_by_id"]), {str(mid) for mid in range(7, 13)})

    def test_reviewed_sample_gap_is_separate_from_hard_cycle_budget(self):
        data = profile(); data["max_sample_gap_ms"] = 5.
        report, _ = self.run_case(profile_data=data)
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("gap exceeded" in error for error in report["errors"]), report["errors"])
        self.assertTrue(report["stop_confirmed"])

    def test_failed_candidate_records_real_intervals_before_envelope_rejection(self):
        clock=SimulatedClock();calls=[]
        data=profile();data.update(hard_cycle_ms=20.,max_sample_age_ms=20.,max_sample_gap_ms=21.)
        def policy(*args):
            calls.append(True)
            clock.advance(3_500_000 if len(calls)==3 else 2_000_000)
            return (.04,)*12
        report,sessions=self.run_case(front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
            imu=FakeIMU(clock=clock),profile_data=data,policy=policy,clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'ABORTED',report['errors'])
        self.assertEqual(len(report['cycles']),2)
        self.assertEqual(len(calls),3)
        trace=report['failed_cycle_timing']
        self.assertEqual(trace['index'],2)
        self.assertEqual(trace['stage'],'motion_envelope')
        self.assertGreater(trace['command_interval_ms'],21.)
        self.assertLess(trace['sample_interval_ms'],21.)
        self.assertLess(trace['candidate_sample_age_ms'],20.)
        self.assertAlmostEqual(trace['policy_call_ms'],3.5,places=2)
        self.assertEqual(trace['command_gap_basis'],'validated_target_computation_not_transport_write')
        self.assertGreaterEqual(trace['candidate_ns'],trace['target_ready_ns'])
        self.assertGreaterEqual(trace['target_ready_ns'],trace['policy_call_return_ns'])
        self.assertIsNone(trace['output_submit_ns'])
        self.assertIsNone(trace['output_return_ns'])
        self.assertTrue(all(session.stop_times[0]>trace['candidate_ns'] for session in sessions.values()))
        self.assertIn('Command gap exceeded',str(report['errors']))

    def test_wait_and_read_stamps_are_causal_failure_evidence_in_both_paths(self):
        for pipeline in (False, True):
            with self.subTest(pipeline=pipeline):
                clock = SimulatedClock(); calls = []
                data = measured_startup_profile()
                def policy(*args):
                    calls.append(True)
                    clock.advance(3_500_000 if len(calls) == 3 else 2_000_000)
                    return (.04,) * 12
                original_settings = runtime.execution_settings
                with patch.object(runtime, 'execution_settings', side_effect=lambda data:
                        {**original_settings(data), 'voltage_pipeline': pipeline}):
                    report, _ = self.run_case(
                        profile_data=data, clock=clock, sleep=clock.sleep,
                        front=FakeSession(1, clock=clock), rear=FakeSession(7, clock=clock),
                        imu=FakeIMU(clock=clock), policy=policy)
                self.assertEqual(report['status'], 'ABORTED', report['errors'])
                trace = report['failed_cycle_timing']
                self.assertEqual(trace['stage'], 'motion_envelope')
                ordered = ('begin_ns', 'combined_acquisition_wait_begin_ns',
                           'combined_acquisition_wait_end_ns',
                           'feedback_collect_begin_ns', 'feedback_collect_end_ns',
                           'imu_wait_begin_ns', 'imu_wait_end_ns', 'acquisition_complete_ns',
                           'policy_call_begin_ns', 'policy_call_return_ns', 'candidate_ns')
                self.assertTrue(all(type(trace[key]) is int for key in ordered))
                self.assertEqual([trace[key] for key in ordered],
                                 sorted(trace[key] for key in ordered))
                self.assertLessEqual(trace['imu_read_started_ns'], trace['imu_read_finished_ns'])
                self.assertLessEqual(trace['imu_read_finished_ns'], trace['imu_wait_end_ns'])
                self.assertGreater(trace['command_interval_ms'], 21.)
                self.assertIsNone(trace['output_submit_ns'])
                self.assertTrue(report['stop_confirmed'])
                # Detailed acquisition timestamps remain failure-only evidence.
                self.assertTrue(all(not set(self._FAILED_ACQUISITION_TIMESTAMPS).intersection(row)
                                    for row in report['cycles']))

    def test_acquisition_failure_does_not_fill_unreached_stages(self):
        for pipeline in (False, True):
            for fail_at in ('feedback_collect', 'imu_wait'):
                with self.subTest(pipeline=pipeline, fail_at=fail_at):
                    clock = SimulatedClock(); pending = []; calls = []
                    original_begin = runtime._PendingCycleTiming.begin
                    original_collect = runtime.BusWorkers.collect
                    original_settings = runtime.execution_settings
                    imu = FakeIMU(clock=clock)
                    def capture_begin(record, *args):
                        original_begin(record, *args)
                        pending.append(record)
                    def collect(workers, futures):
                        if fail_at == 'feedback_collect' and pending and pending[-1].active:
                            raise OSError('Injected feedback collection failure')
                        return original_collect(workers, futures)
                    def read_imu():
                        calls.append(True)
                        if fail_at == 'imu_wait' and len(calls) == 2:
                            raise TimeoutError('Injected IMU acquisition failure')
                        return imu()
                    with patch.object(runtime._PendingCycleTiming, 'begin', capture_begin), \
                         patch.object(runtime.BusWorkers, 'collect', collect), \
                         patch.object(runtime, 'execution_settings', side_effect=lambda data:
                            {**original_settings(data), 'voltage_pipeline': pipeline}):
                        report, sessions = self.run_case(
                            profile_data=measured_startup_profile(), clock=clock, sleep=clock.sleep,
                            front=FakeSession(1, clock=clock), rear=FakeSession(7, clock=clock),
                            imu=read_imu)
                    self.assertEqual(report['status'], 'ABORTED', report['errors'])
                    self.assertEqual(report['cycles'], [])
                    trace = report['failed_cycle_timing']
                    self.assertEqual(trace['stage'], 'input_acquisition')
                    self.assertIsNotNone(trace['combined_acquisition_wait_begin_ns'])
                    if fail_at == 'feedback_collect':
                        self.assertIsNotNone(trace['combined_acquisition_wait_end_ns'])
                        self.assertIsNotNone(trace['feedback_collect_begin_ns'])
                        self.assertIsNone(trace['feedback_collect_end_ns'])
                        self.assertIsNone(trace['imu_wait_begin_ns'])
                    else:
                        # An IMU failure now interrupts the joint wait before
                        # either CAN result is collected or inputs are used.
                        self.assertIsNone(trace['combined_acquisition_wait_end_ns'])
                        self.assertIsNone(trace['feedback_collect_begin_ns'])
                        self.assertIsNone(trace['feedback_collect_end_ns'])
                        self.assertIsNone(trace['imu_wait_begin_ns'])
                    for key in ('imu_wait_end_ns', 'imu_read_started_ns', 'imu_read_finished_ns',
                                'acquisition_complete_ns', 'policy_call_begin_ns',
                                'candidate_ns', 'output_submit_ns'):
                        self.assertIsNone(trace[key], key)
                    self.assertTrue(report['stop_confirmed'])
                    self.assertTrue(all(session.positive_gain_writes == 0
                                        for session in sessions.values()))

    def test_reused_scalar_record_clears_previous_read_timestamps(self):
        record = runtime._PendingCycleTiming()
        record.begin(0, 1, 2, None, None)
        for name in self._FAILED_ACQUISITION_TIMESTAMPS:
            setattr(record, name, 3)
        record.begin(1, 4, 5, 3, 3)
        self.assertTrue(all(getattr(record, name) is None
                            for name in self._FAILED_ACQUISITION_TIMESTAMPS))

    def test_host_wakeup_gap_stops_before_reusing_positive_gain_hold(self):
        data=profile();data.update(hard_cycle_ms=20.,max_sample_age_ms=20.,max_sample_gap_ms=21.)
        clock=SimulatedClock();state={'sleeps':0,'wake_ns':None}
        def delayed_sleep(seconds):
            clock.sleep(seconds)
            state['sleeps']+=1
            if state['sleeps']==3:
                # Move every timestamp source forward exactly 15ms. Real host
                # scheduling cannot trigger the guard before this injection.
                clock.advance(15_000_000)
                state['wake_ns']=clock()
        report,sessions=self.run_case(profile_data=data,clock=clock,sleep=delayed_sleep,
            front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),imu=FakeIMU(clock=clock))
        self.assertIsNotNone(state['wake_ns'],report['errors'])
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Command/sample gap exceeded before feedback hold',str(report['errors']))
        self.assertTrue(all(s.positive_gain_writes for s in sessions.values()))
        self.assertTrue(all(not any(t>=state['wake_ns'] and kind==1 for t,kind,_,_ in s.calls)
                            for s in sessions.values()))
        self.assertTrue(report['stop_confirmed'])

    def test_watchdog_ack_failure_prevents_readback_and_enable(self):
        class WrongWatchdogAck(FakeSession):
            def _exchange(self,wires,timeout_ns,send_only):
                records,stats=super()._exchange(wires,timeout_ns,send_only)
                for record in records:
                    tx=codec.ATParser().feed(bytes(record.tx))[0]
                    if tx.kind==18:
                        rx=codec.ATParser().feed(bytes(record.rx))[0]
                        record.rx[:]=wire(rx.can_id|(2<<22),rx.data)
                return records,stats
        report,sessions=self.run_case(front=WrongWatchdogAck(1))
        self.assertIn('watchdog setup acknowledgement',str(report['errors']))
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(any(batch['phase']=='watchdog_initial_readback' for batch in report['journal']))
        self.assertTrue(report['stop_confirmed'])

    def test_initial_overtemperature_prevents_all_enable_commands(self):
        report, sessions = self.run_case(front=FakeSession(1, initial_temperature_c=65.))
        self.assertEqual(report["status"], "ABORTED")
        self.assertTrue(any("temperature" in error for error in report["errors"]), report["errors"])
        self.assertFalse(report["motor_enable_sent"])
        self.assertTrue(all(all(call[1] != 3 for call in session.calls) for session in sessions.values()))
        self.assertTrue(report["stop_confirmed"])

    def test_stop_complete_false_or_ambiguous_is_unconfirmed_even_with_all_ids(self):
        for complete, ambiguous in ((False, False), (True, True)):
            with self.subTest(complete=complete, ambiguous=ambiguous):
                stop = threading.Event(); stop.set()
                rear = FakeSession(7, stop_complete=complete, stop_ambiguous=ambiguous)
                report, _ = self.run_case(rear=rear, stop_requested=stop)
                self.assertEqual(report["stop_reports"]["rear"]["confirmed_ids"], list(range(7, 13)))
                self.assertFalse(report["stop_confirmed"])
                self.assertEqual(report["status"], "STOP_UNCONFIRMED_POWER_OFF_REQUIRED")

    def test_rear_failure_cancels_front_without_waiting_for_front_timeout(self):
        front_started = threading.Event()
        cancelled = threading.Event()
        failure_times, cancellation_times, front_observed_cancel = [], [], []

        class WaitingFront(FakeSession):
            def exchange(self, wires, *, timeout_ns):
                with self._busy:
                    front_started.set()
                    front_observed_cancel.append(cancelled.wait(.25))
                    raise InterruptedError("Front cancellation" if front_observed_cancel[-1]
                                           else "Front independent timeout")

        class FailingRear(FakeSession):
            def exchange(self, wires, *, timeout_ns):
                with self._busy:
                    if not front_started.wait(.5):
                        raise AssertionError("Front worker never started")
                    failure_times.append(time.monotonic())
                    raise OSError("Rear failed while front was waiting")

        def cancel():
            cancellation_times.append(time.monotonic())
            cancelled.set()

        sessions = {"front": WaitingFront(1), "rear": FailingRear(7)}
        workers = runtime.BusWorkers(sessions, cancel)
        try:
            # collect() visits front first. The rear owner must initiate emergency
            # itself, otherwise collection waits out the front's independent .25s.
            with self.assertRaises((InterruptedError, OSError)):
                workers.exchange({"front": [codec.read_request(1)],
                                  "rear": [codec.read_request(7)]}, timeout_ns=200_000_000)
            stops = workers.finish_stops()
            self.assertEqual(front_observed_cancel, [True])
            self.assertEqual(len(cancellation_times), 1)
            self.assertLess(cancellation_times[0] - failure_times[0], .1)
            self.assertIn("Rear failed", workers.reason)
            for scope, session in sessions.items():
                self.assertEqual(len(session.stop_times), 1)
                self.assertEqual(stops[scope]["confirmed_ids"], list(session.ids))
        finally:
            workers.finish_stops()
            workers.close()


class NonPipelinedDeadlineTests(unittest.TestCase):
    def test_queued_acquisition_and_voltage_keep_original_absolute_deadline(self):
        for phase in ('feedback_hold','overlapped_voltage'):
            for expired in (False,True):
                with self.subTest(phase=phase,expired=expired):
                    clock=SimulatedClock()
                    sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
                    for session in sessions.values():session.enabled.update(session.ids)
                    workers=runtime.BusWorkers(sessions,lambda:None,clock)
                    entered=threading.Event();release=threading.Event()
                    try:
                        def block_owner():
                            entered.set()
                            self.assertTrue(release.wait(2.))
                        blocked=workers.pools['front'].submit(block_owner)
                        self.assertTrue(entered.wait(2.))
                        deadline=clock()+5_000_000
                        if phase=='feedback_hold':
                            pending=workers.submit({'front':[encode_motion(1,0.,0.,0.)]},
                                deadline_ns=deadline,label=phase)
                        else:
                            pending=workers.submit_voltage({'front':1},profile(),deadline_ns=deadline)
                        clock.advance(10_000_000 if expired else 3_000_000)
                        release.set();blocked.result(timeout=2.)
                        if expired:
                            with self.assertRaisesRegex(RuntimeError,'absolute hard deadline'):
                                workers.collect(pending)
                            self.assertFalse(any(session.calls for session in sessions.values()))
                        else:
                            result=workers.collect(pending)['front']
                            exchange=result if phase=='feedback_hold' else result[0]
                            self.assertEqual(exchange[0][0].deadline_ns,deadline)
                            self.assertLess(exchange[0][0].received_ns,deadline)
                        stops=workers.finish_stops()
                        self.assertTrue(all(stops[scope]['complete'] for scope in sessions))
                        self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))
                    finally:
                        release.set();workers.finish_stops();workers.close()

    def test_nonpipelined_cycles_use_cycle_and_sample_age_deadlines(self):
        clock=SimulatedClock()
        data=profile()
        data.update(schema=live.SCHEMA_V3,telemetry_cadence=live.CADENCE_PRE_ENABLE,
                    cadence_source_sha256=live.cadence_source_hashes(),voltage_overlap=True)
        sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
        report=runtime.run_supported_policy(data,sessions,FakeIMU(clock=clock),lambda *_:(.04,)*12,
            cancel_io=lambda:None,encode_motion=encode_motion,clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertFalse(report['execution_settings']['voltage_pipeline'])
        self.assertTrue(report['stop_confirmed'])
        feedback={scope:[row for row in report['journal']
            if row['phase']=='feedback_hold' and row['bus']==scope] for scope in sessions}
        voltage={scope:[row for row in report['journal']
            if row['phase']=='overlapped_voltage' and row['bus']==scope] for scope in sessions}
        for scope in sessions:
            self.assertEqual(len(feedback[scope]),len(report['cycles']))
            self.assertEqual(len(voltage[scope]),len(report['cycles']))
        for index,cycle in enumerate(report['cycles']):
            hold_deadline=cycle['begin_ns']+int(data['hard_cycle_ms']*1e6)
            oldest=min(cycle['imu']['read_started_monotonic_ns'],
                *(record['start_ns'] for scope in sessions for record in feedback[scope][index]['records']))
            voltage_deadline=min(hold_deadline,oldest+int(data['max_sample_age_ms']*1e6))
            for scope in sessions:
                self.assertTrue(all(record['deadline_ns']==hold_deadline
                    for record in feedback[scope][index]['records']))
                self.assertTrue(all(record['deadline_ns']==voltage_deadline
                    for record in voltage[scope][index]['records']))


class OverlappedVoltageTests(unittest.TestCase):
    def fast_profile(self):
        from singularitydog_hw import policy_live_profile as profiles
        data=profile()
        data.update(schema=profiles.SCHEMA_V3,telemetry_cadence=profiles.CADENCE_PRE_ENABLE,
                    cadence_source_sha256=profiles.cadence_source_hashes(),voltage_overlap=True)
        return data

    def run_case(self,policy,**kwargs):
        clock=kwargs.get('clock',time.monotonic_ns)
        sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
        # macOS may coalesce a 20ms sleep into 25ms, which intentionally trips
        # the unchanged 126ms voltage-cache guard after six slots. This fixture
        # yields until the requested time so it tests ownership/ordering instead.
        def fixture_sleep(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:time.sleep(0)
        kwargs.setdefault('sleep',fixture_sleep)
        report=runtime.run_supported_policy(self.fast_profile(),sessions,FakeIMU(clock=clock),policy,
            cancel_io=lambda:None,encode_motion=encode_motion,**kwargs)
        self.assertTrue(report['stop_confirmed'],report['errors'])
        self.assertTrue(all(len(s.stop_times)==1 for s in sessions.values()))
        return report,sessions

    def test_voltage_owner_validation_overlaps_policy_and_joins_before_type1(self):
        # This tests concurrent ownership and ordering, not macOS wall-clock
        # deadlines. One shared synthetic clock stamps the runner, buses and
        # IMU; Event handshakes retain real independent worker execution.
        clock=SimulatedClock()
        entered=threading.Event();validated={i:threading.Event() for i in (1,7)}
        original=runtime.checked_voltage_rows
        first=[True]
        def validate(rows,ids,data,now):
            if len(ids)==1 and tuple(ids)[0] in validated and first[0]:
                self.assertTrue(entered.wait(2.),'Voltage worker must overlap model execution')
                result=original(rows,ids,data,clock())
                validated[tuple(ids)[0]].set()
                return result
            return original(rows,ids,data,now)
        def policy(*args):
            if first[0]:
                entered.set()
                self.assertTrue(all(event.wait(2.) for event in validated.values()))
                first[0]=False
            return (.04,)*12
        with patch.object(runtime,'checked_voltage_rows',side_effect=validate):
            report,sessions=self.run_case(policy,clock=clock,sleep=clock.sleep)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['execution_settings']['voltage_overlap'])
        self.assertTrue(report['learned_targets_sent'])
        for cycle in report['cycles']:
            self.assertEqual(set(cycle['overlapped_voltage_validated_ns']),{'front','rear'})
            latest=max(cycle['overlapped_voltage_validated_ns'].values())
            self.assertLessEqual(latest,cycle['output_reply_end_ns'])
            self.assertLessEqual(cycle['policy_return_ns'],cycle['end_ns'])
        self.assertFalse(report['full_controller_50Hz_verified'])

    def test_bad_voltage_stops_both_owners_while_policy_is_still_running(self):
        entered=threading.Event();returned=[];original=runtime.checked_voltage_rows
        stop_seen=threading.Event()
        original_stop=FakeSession.emergency_stop
        def stop(session):
            result=original_stop(session);stop_seen.set();return result
        def validate(rows,ids,data,now):
            if len(ids)==1:
                self.assertTrue(entered.wait(.05))
                raise RuntimeError('Injected invalid overlapped voltage')
            return original(rows,ids,data,now)
        def policy(*args):
            entered.set();self.assertTrue(stop_seen.wait(.1))
            time.sleep(.003);returned.append(time.monotonic_ns());return (.04,)*12
        with patch.object(runtime,'checked_voltage_rows',side_effect=validate), \
             patch.object(FakeSession,'emergency_stop',stop):
            report,sessions=self.run_case(policy)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(any('Injected invalid' in value for value in report['errors']),report['errors'])
        self.assertTrue(returned)
        self.assertTrue(all(s.stop_times[0]<returned[0] for s in sessions.values()))
        self.assertTrue(all(s.positive_gain_writes==0 for s in sessions.values()))

    def test_overlap_does_not_waive_stale_imu_or_inference_deadline(self):
        def late(*args):
            time.sleep(.055);return (.04,)*12
        report,_=self.run_case(late)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(any('hard cycle deadline' in error for error in report['errors']),report['errors'])


class VoltagePipelineCandidateTests(OverlappedVoltageTests):
    """Opt-in feedback/voltage chain with no device or motor transport."""

    def run_case(self,policy,**kwargs):
        original=runtime.execution_settings
        with patch.object(runtime,'execution_settings',
                          side_effect=lambda data:{**original(data),'voltage_pipeline':True}):
            return super().run_case(policy,**kwargs)

    def _workers(self,*,front=None,rear=None,cancel_io=lambda:None,deadline_ns=None):
        sessions={'front':front or FakeSession(1),'rear':rear or FakeSession(7)}
        for session in sessions.values():session.enabled.update(session.ids)
        workers=runtime.BusWorkers(sessions,cancel_io)
        requests={scope:[encode_motion(mid,0.,0.,0.) for mid in session.ids]
                  for scope,session in sessions.items()}
        deadline=[time.monotonic_ns()+1_000_000_000 if deadline_ns is None else deadline_ns]
        feedback,voltage=workers.submit_feedback_then_voltage(
            requests,{'front':1,'rear':7},self.fast_profile(),deadline_ns=deadline)
        return workers,sessions,deadline,feedback,voltage

    def test_feedback_is_published_before_same_owner_read_only_voltage(self):
        workers,sessions,deadline,feedback,voltage=self._workers()
        try:
            result=workers.collect(feedback)
            self.assertEqual(set(runtime.rows_from_decoded_pair(result)),
                             {(mid,'feedback') for mid in range(1,13)})
            verified=workers.collect(voltage)
            self.assertEqual(set(verified),{'front','rear'})
            for session in sessions.values():
                self.assertEqual([call[1] for call in session.calls],[1]*6+[17])
            for exchange,_ in result.values():
                self.assertTrue(all(record.deadline_ns==deadline[0] for record in exchange[0]))
        finally:
            workers.finish_stops();workers.close()

    def test_emergency_cancels_inflight_voltage_before_same_owner_stop(self):
        cancelled=threading.Event()
        class BlockingVoltage(FakeSession):
            def __init__(self,first_id):
                super().__init__(first_id)
                self.feedback_seen=False;self.voltage_entered=threading.Event()
            def _exchange(self,wires,timeout_ns,send_only):
                kind=codec.ATParser().feed(wires[0])[0].kind
                if self.feedback_seen and len(wires)==1 and kind==17:
                    if not self._busy.acquire(blocking=False):
                        raise AssertionError('One bus had concurrent owners')
                    try:
                        self.voltage_entered.set()
                        if not cancelled.wait(.5):raise TimeoutError('Synthetic cancellation missing')
                        raise TimeoutError('Synthetic voltage cancelled')
                    finally:self._busy.release()
                result=super()._exchange(wires,timeout_ns,send_only)
                if len(wires)==6 and kind==1:self.feedback_seen=True
                return result
        front,rear=BlockingVoltage(1),BlockingVoltage(7)
        workers,sessions,_,feedback,voltage=self._workers(
            front=front,rear=rear,cancel_io=cancelled.set)
        try:
            workers.collect(feedback)
            self.assertTrue(front.voltage_entered.wait(.05))
            self.assertTrue(rear.voltage_entered.wait(.05))
            began=time.monotonic()
            workers.emergency('Injected bad IMU with in-flight voltage')
            self.assertTrue(cancelled.is_set())
            with self.assertRaises(TimeoutError):workers.collect(voltage)
            stops=workers.finish_stops()
            self.assertLess(time.monotonic()-began,.1)
            self.assertTrue(all(stops[scope]['complete'] for scope in sessions))
            self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))
        finally:
            workers.finish_stops();workers.close()

    def test_bad_feedback_precheck_stops_without_bad_bus_voltage(self):
        class BadFeedback(FakeSession):
            def _exchange(self,wires,timeout_ns,send_only):
                records,stats=super()._exchange(wires,timeout_ns,send_only)
                if len(wires)==6 and codec.ATParser().feed(wires[0])[0].kind==1:
                    records[-1].rx[-1]=0
                return records,stats
        workers,sessions,_,feedback,voltage=self._workers(front=BadFeedback(1))
        try:
            with self.assertRaises(RuntimeError):workers.collect(feedback)
            with self.assertRaises(RuntimeError):workers.collect(voltage)
            stops=workers.finish_stops()
            self.assertTrue(all(stops[scope]['complete'] for scope in sessions))
            self.assertFalse(any(call[1]==17 for call in sessions['front'].calls))
        finally:
            workers.finish_stops();workers.close()

    def test_expired_absolute_deadline_stops_before_feedback_or_voltage(self):
        workers,sessions,_,feedback,voltage=self._workers(deadline_ns=time.monotonic_ns()-1)
        try:
            with self.assertRaises(RuntimeError):workers.collect(feedback)
            with self.assertRaises(RuntimeError):workers.collect(voltage)
            stops=workers.finish_stops()
            self.assertTrue(all(stops[scope]['complete'] for scope in sessions))
            self.assertFalse(any(session.calls for session in sessions.values()))
        finally:
            workers.finish_stops();workers.close()

    def test_queued_type1_cannot_restart_expired_absolute_deadline(self):
        clock=SimulatedClock()
        sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
        for session in sessions.values():session.enabled.update(session.ids)
        workers=runtime.BusWorkers(sessions,lambda:None,clock)
        entered=threading.Event();release=threading.Event()
        try:
            def block_owner():
                entered.set()
                self.assertTrue(release.wait(.1))
            blocked=workers.pools['front'].submit(block_owner)
            self.assertTrue(entered.wait(.05))
            deadline=clock()+5_000_000
            future=workers.submit_decoded({'front':[encode_motion(1,0.,0.,0.)]},
                                          deadline_ns=deadline,label='policy_output')
            clock.advance(10_000_000)
            release.set();blocked.result(timeout=.1)
            with self.assertRaisesRegex(RuntimeError,'absolute hard deadline'):
                workers.collect(future)
            stops=workers.finish_stops()
            self.assertTrue(all(stops[scope]['complete'] for scope in sessions))
            self.assertFalse(any(call[1]==1 for session in sessions.values()
                                 for call in session.calls))
        finally:
            release.set();workers.finish_stops();workers.close()

    def test_type1_owner_receives_exact_absolute_deadline(self):
        clock=SimulatedClock()
        sessions={'front':FakeSession(1,clock=clock),'rear':FakeSession(7,clock=clock)}
        for session in sessions.values():session.enabled.update(session.ids)
        workers=runtime.BusWorkers(sessions,lambda:None,clock)
        try:
            deadline=clock()+20_000_000
            result=workers.collect(workers.submit_decoded(
                {'front':[encode_motion(1,0.,0.,0.)]},deadline_ns=deadline,label='policy_output'))
            self.assertEqual(result['front'][0][0][0].deadline_ns,deadline)
            self.assertEqual([call[1] for call in sessions['front'].calls],[1])
        finally:
            workers.finish_stops();workers.close()

    def test_invalid_imu_stops_before_candidate_type1_output(self):
        with patch.object(runtime,'validate_imu_metadata',side_effect=RuntimeError('Injected bad IMU')):
            report,sessions=self.run_case(lambda *_:(.04,)*12)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('Injected bad IMU' in error for error in report['errors']))
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))

    def test_semantically_invalid_feedback_stops_before_bad_bus_voltage(self):
        original_exchange=FakeSession._exchange
        for fault,expected in (('mode','fault/mode'),('fault','fault/mode'),
                               ('position','raw position discontinuity')):
            with self.subTest(fault=fault):
                # The injected second feedback batch must be the cause of
                # rejection, not unrelated host load during pre-enable reads.
                # Preserve the same clock for owner, coordinator and IMU.
                clock=SimulatedClock()
                def inject(session,wires,timeout_ns,send_only):
                    records,stats=original_exchange(session,wires,timeout_ns,send_only)
                    if len(wires)==6 and codec.ATParser().feed(wires[0])[0].kind==1:
                        count=getattr(session,'feedback_batches',0)+1
                        session.feedback_batches=count
                        if session.ids[0]==1 and count==2:
                            record=records[0]
                            response=bytes(record.rx)
                            can_id=int.from_bytes(response[2:6],'big')>>3
                            data=response[7:15]
                            if fault=='mode':can_id&=~(3<<22)
                            elif fault=='fault':can_id|=1<<16
                            else:data=struct.pack('>H',quantize(.5,-12.57,12.57))+data[2:]
                            record.rx[:]=wire(can_id,data)
                    return records,stats

                with patch.object(FakeSession,'_exchange',inject):
                    report,sessions=self.run_case(lambda *_:(.04,)*12,
                                                  clock=clock,sleep=clock.sleep)
                self.assertEqual(report['status'],'ABORTED',report['errors'])
                self.assertTrue(any(expected in error for error in report['errors']),report['errors'])
                self.assertFalse(any(row['phase']=='overlapped_voltage' and row['bus']=='front'
                                     for row in report['journal']))
                self.assertFalse(report['learned_targets_sent'])
                self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))

    def test_slow_voltage_joins_after_policy_but_before_type1(self):
        # Events model a voltage owner delayed behind policy computation.
        # Waiting for the host to schedule those threads must not consume the
        # simulated sensor-freshness budget; live limits remain unchanged.
        clock=SimulatedClock()
        started={scope:threading.Event() for scope in runtime.BUSES}
        release=threading.Event();policy_done=threading.Event()
        output_before_release=[];probe=[]
        original_voltage=runtime.BusWorkers._voltage
        original_submit=runtime.BusWorkers.submit_decoded

        def slow_voltage(worker,scope,*args,**kwargs):
            started[scope].set()
            if not release.wait(2.):raise RuntimeError('Synthetic voltage wait timed out')
            return original_voltage(worker,scope,*args,**kwargs)

        def guarded_submit(worker,*args,**kwargs):
            if not release.is_set():output_before_release.append(True)
            return original_submit(worker,*args,**kwargs)

        def policy(*_):
            policy_done.set()
            return (.04,)*12

        def permit_voltage():
            if policy_done.wait(2.) and all(event.wait(2.) for event in started.values()):
                probe.append(not output_before_release)
            release.set()

        releaser=threading.Thread(target=permit_voltage,daemon=True)
        releaser.start()
        try:
            with patch.object(runtime.BusWorkers,'_voltage',slow_voltage), \
                 patch.object(runtime.BusWorkers,'submit_decoded',guarded_submit):
                report,_=self.run_case(policy,clock=clock,sleep=clock.sleep)
        finally:
            release.set();releaser.join(2.)
        self.assertFalse(releaser.is_alive(),'Synthetic voltage releaser did not finish')
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(report['voltage_pipeline'],'feedback_then_voltage.fast_v1')
        self.assertEqual(probe,[True])
        self.assertFalse(output_before_release)
        self.assertTrue(report['learned_targets_sent'])


if __name__ == "__main__":
    unittest.main()
