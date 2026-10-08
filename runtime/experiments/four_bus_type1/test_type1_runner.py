"""File-only four-bus Type1 runner tests: mock mode-2 transports, no library/device/model."""
from concurrent.futures import Future
from contextlib import contextmanager
import copy
import gc
import hashlib
import json
import math
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import motor_version_probe as versions
from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.native_active_transport import encode_motion
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw.policy_motion_envelope import MotionFault, PolicyMotionEnvelope
from singularitydog_hw.policy_output_runtime import decode_records
from experiments.four_bus_diagnostic import model_bridge as bridge
from experiments.four_bus_diagnostic.transport_adapter import Batch, Group, PORTS
from . import type1_runner as runner

IDS = tuple(range(1, 13))
PORT_IDS = {'port0': (7, 8, 9), 'port1': (10, 11, 12), 'port2': (4, 5, 6), 'port3': (1, 2, 3)}
SIGN = {mid: -1 if mid in (2, 5, 9) else 1 for mid in IDS}
TURNS = {mid: 1 if mid == 3 else 0 for mid in IDS}
BASE = {mid: -1.2 if mid % 3 == 1 else .6 if mid % 3 == 2 else 0. for mid in IDS}
UID = {mid: '%016x' % (0xabc000+mid) for mid in IDS}
FIRMWARE = {mid: 'a1b2c3%02x' % mid for mid in IDS}
DEG = math.radians(1)
IMU_LIMITS = {'imu_tilt_limit_rad': .2, 'imu_gyro_limit_rad_s': .5,
              'imu_accel_norm_min_m_s2': 9.4, 'imu_accel_norm_max_m_s2': 10.2}
MOUNT = {'schema_version': 1, 'status': 'IMU_MOUNT_CANDIDATE_ONLY', 'input_frame': 'sensor',
         'output_frame': 'body_x_forward_y_left_z_up', 'R_body_from_sensor': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
         'raw_driver_axes_verified': False, 'approved_for_runtime': False, 'provenance': {'fixture': 'unverified'}}
BIAS = {'schema_version': 1, 'kind': 'fixed_mount_baseline', 'status': 'GYRO_BIAS_CANDIDATE', 'frame': 'sensor',
        'axis_order': ['x', 'y', 'z'], 'gyro_bias_candidate_eligible': True, 'operator_confirmed_stationary': True,
        'approved_for_runtime': False, 'automatically_applied': False, 'mount_rotation_applied': False,
        'gyro_bias_candidate_rad_s': [0., 0., 0.], 'captures': {'a': {'gyro_mean_rad_s': [0., 0., 0.]}},
        'provenance': {k: {'summary_sha256': c*64, 'events_sha256': d*64} for k, c, d in (('a', 'a', 'b'), ('b', 'c', 'd'))}}


def u16(raw):
    return round((raw+12.57)*65535/25.14)


def from_u16(value):
    return value*25.14/65535-12.57


RAW = {mid: from_u16(u16(.1+.01*mid)) for mid in IDS}
FIXED = {mid: BASE[mid]-SIGN[mid]*RAW[mid] for mid in IDS}
NOMINAL = {mid: FIXED[mid]+SIGN[mid]*TURNS[mid]*2*math.pi for mid in IDS}
MODEL = {mid: BASE[mid]+.03*((mid % 4)-1.5) for mid in IDS}


def frame(can_id, data):
    return b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def type2(mid, raw, *, mode, fault=0, temperature_dc=250):
    return frame((2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 0xfd,
                 struct.pack('>4H', u16(raw), 32767, 32767, temperature_dc))


def type17(mid, name, value):
    index, fmt, _ = codec.PARAMETERS[name]
    data = struct.pack('<H', index)+bytes(2)+struct.pack('<'+fmt, value)
    return frame((17 << 24) | (mid << 8) | 0xfd, data+bytes(8-len(data)))


def model_plan():
    plan = {'schema': bridge.PLAN_SCHEMA, **bridge.NO_GRANTS,
            'topology_by_port': {port: list(ids) for port, ids in PORT_IDS.items()},
            'axes': {str(mid): {'physical_port': next(p for p, ids in PORT_IDS.items() if mid in ids),
                                'sign': SIGN[mid], 'fixed_offset_rad': FIXED[mid],
                                'local_bounds_rad': [BASE[mid]-3*DEG, BASE[mid]+3*DEG],
                                'capture_last_position_reply_ns': 1} for mid in IDS},
            'observer_kwargs': {'imu_mount_candidate': copy.deepcopy(MOUNT), 'gyro_bias_candidate': copy.deepcopy(BIAS),
                                'apply_reviewed_accel_calibration': False, 'accel_input_hypothesis': None}}
    plan['plan_canonical_sha256'] = hashlib.sha256(bridge._canonical(plan).encode()).hexdigest()
    return plan


def resealed(plan):
    plan.pop('plan_canonical_sha256', None)
    plan['plan_canonical_sha256'] = hashlib.sha256(bridge._canonical(plan).encode()).hexdigest()
    return plan


def admitted(mode='learned_boxed', duration=2):
    axes = {}
    for mid in IDS:
        lo, hi = BASE[mid]-3*DEG, BASE[mid]+3*DEG
        axes[str(mid)] = {'uid': UID[mid], 'sign': SIGN[mid], 'offset_rad': NOMINAL[mid],
            'physical_lower_rad': lo, 'physical_upper_rad': hi,
            'lower_rad': lo+runner.BRANCH_MARGIN_RAD, 'upper_rad': hi-runner.BRANCH_MARGIN_RAD,
            'kp': 3., 'kd': .15, 'max_command_velocity_rad_s': DEG, 'max_command_acceleration_rad_s2': 5*DEG,
            'max_tracking_error_rad': 2*DEG, 'max_measured_velocity_rad_s': .35, 'max_measured_torque_nm': 1.,
            'max_temperature_c': 45., 'max_estimated_pd_torque_nm': .1, 'max_displacement_from_start_rad': DEG}
    profile = {'axes': axes, 'start_pose_bounds': {str(mid): [BASE[mid]-.49*DEG, BASE[mid]+.49*DEG] for mid in IDS},
               'max_sample_age_ms': 20, 'max_sample_gap_ms': 21, 'hard_cycle_ms': 20, 'period_ms': 20,
               'max_consecutive_20ms_misses': 0, 'voltage_min_v': 35, 'voltage_max_v': 42,
               'startup_damping_duration_s': .08, 'policy_weight': .005, 'duration_s': duration,
               'startup_duration_s': .5, 'stop_duration_s': .5, 'policy_ramp_s': .5, **IMU_LIMITS}
    return {'mode': mode, 'duration_s': duration,
            'ids_by_port': {port: list(ids) for port, ids in PORT_IDS.items()},
            'boot_id': 'file-only-boot', 'motor_power_epoch': 'file-only-epoch', 'contract_sha256': 'a'*64,
            'profile': profile, 'offsets_by_id': {str(mid): FIXED[mid] for mid in IDS},
            'reference_turns_by_id': {str(mid): TURNS[mid] for mid in IDS},
            'uids_by_id': {str(mid): UID[mid] for mid in IDS},
            'firmware_by_id': {str(mid): FIRMWARE[mid] for mid in IDS},
            'pacing': copy.deepcopy(runner.PACING), 'post_reply_policy': dict(runner.POST_REPLY_V1),
            'first_cycle_post_reply': True, 'model_plan': model_plan()}


class VirtualClock:
    """Shared strictly increasing monotonic clock; host scheduling jitter is not under test."""
    def __init__(self):
        self.lock = threading.Lock()
        self.now = time.monotonic_ns()

    def __call__(self):
        with self.lock:
            self.now += 1000
            return self.now

    def advance_to(self, value):
        with self.lock:
            self.now = max(self.now+1000, value)
            return self.now


class MockType1:
    """Emulates the DESIGN section 2 owner contract with genuine Record/Batch bytes."""
    def __init__(self, group, harness):
        self.group, self.h, self.c = group, harness, harness.controls
        self.journal, self.calls, self.outputs = [], [], []
        self.last_batch = None
        self.owner = None
        self.last_output = None
        self.holds = self.stops = 0
        self.closed = False
        self.counts = {}
        self.last_hold = None

    def _owner(self, method, stop=False):
        tid = threading.get_native_id()
        if self.owner is None:
            self.owner = tid
        if tid != self.owner:
            raise RuntimeError('Mock physical owner changed')
        if not stop and self.h.cancelled.is_set():
            raise RuntimeError('Native cancel byte set')
        self.calls.append((method, self.h.clock()))

    def _exchange(self, label, pairs, deadline):
        n = self.counts[label] = self.counts.get(label, -1)+1
        age = self.c.get('stamp_age', {}).get((self.group.port, label, n), 0)  # An early-buffered reply.
        records, stats = (Record*len(pairs))(), active.Stats()
        start = self.h.clock()
        if start >= deadline:
            raise TimeoutError('Native late write')
        stats.begin_ns = start = start-age
        for record, (tx, rx) in zip(records, pairs):
            record.tx[:] = tx
            record.rx[:] = rx
            record.start_ns = record.finish_ns = record.read_start_ns = start
            record.deadline_ns = deadline
            record.written = record.received = 17
        received = self.h.clock()
        if received >= deadline:
            raise TimeoutError('Native late reply')
        for record in records:
            record.received_ns = received-age
        stats.end_ns = received-age
        stats.writes = stats.reads = len(pairs)
        stats.bytes = 17*len(pairs)
        rows = decode_records((records, stats))
        batch = Batch(self.group, label, records, stats, rows, bytes(records), bytes(stats), received)
        self.journal.append((label, (records, stats)))
        self.last_batch = batch
        return batch

    def raw(self, mid):
        return RAW[mid]+self.c.get('drift', {}).get(mid, 0.)

    def identify(self, *, deadline_ns):
        self._owner('identify')
        wrong = self.c.get('wrong_uid')
        batch = self._exchange('identity', [(codec.read_request(mid), frame(
            (mid << 8) | 0xfe, bytes.fromhex(UID[mid] if mid != wrong else '%016x' % 1)))
            for mid in self.group.ids], deadline_ns)
        return {mid: batch.rows[mid, 'identity'] for mid in self.group.ids}

    def stop(self, *, deadline_ns):
        self._owner('stop')
        self.stops += 1
        moved = self.c.get('moved_on_stop')
        batch = self._exchange('stop', [(stop_wire(mid), type2(mid, self.raw(mid)+(
            .03 if moved == self.stops and mid == self.group.ids[0] else 0.), mode=0))
            for mid in self.group.ids], deadline_ns)
        return {mid: batch.rows[mid, 'feedback'][0] for mid in self.group.ids}  # Bare, like Type1Transport.

    def version_probe(self, *, deadline_ns):
        self._owner('version')
        batch = self._exchange('version', [(versions.version_request(mid), frame(
            (2 << 24) | (mid << 8) | 0xfd, versions.VERSION_PREFIX+bytes.fromhex(
                FIRMWARE[mid] if self.c.get('wrong_firmware') != mid else '00000000')+b'\0'))
            for mid in self.group.ids], deadline_ns)
        return {mid: bytes.fromhex(batch.rows[mid, 'version'][0]['version_bytes_hex']) for mid in self.group.ids}

    def read_params(self, names, *, deadline_ns):
        self._owner('read_params')
        values = {'run_mode': 0, 'voltage': self.c.get('setup_voltage', 40.), 'velocity': self.c.get('velocity', 0.),
                  'can_timeout': self.c.get('timeout_ticks', protocol.WATCHDOG_TICKS)}
        pairs = [(codec.read_request(mid, name), type17(mid, name, self.raw(mid) if name == 'position'
                  else values[name])) for mid in self.group.ids for name in names]
        rows = self._exchange('read_'+'_'.join(names), pairs, deadline_ns).rows
        if self.c.get('param_rows'):
            return rows
        return {key: row[0]['value'] for key, row in rows.items()}  # Bare, like Type1Transport.

    def write_watchdog(self, ticks, *, deadline_ns):
        self._owner('watchdog')
        if ticks != protocol.WATCHDOG_TICKS:
            raise ValueError('Mock watchdog ticks')
        batch = self._exchange('watchdog', [(protocol.watchdog_setup_request(
            phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid), type2(mid, self.raw(mid), mode=0))
            for mid in self.group.ids], deadline_ns)
        return batch.rows

    def enable(self, mid, *, deadline_ns):
        self._owner('enable')
        self.h.sequence.append(('enable', self.group.port, mid))
        self.h.ledger.append(('enable', self.group.port))
        fault = 1 if self.c.get('enable_fault') == mid else 0
        raw = self.raw(mid)+self.c.get('enable_drift', {}).get(mid, 0.)
        batch = self._exchange('enable', [(protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=mid),
                                           type2(mid, raw, mode=0, fault=fault))], deadline_ns)
        if self.c.get('enable_stall') == mid:
            self.h.clock.advance_to(self.h.clock()+130_000_000)
        return batch.rows[mid, 'feedback'][0]

    def zero_gain(self, mid, q_raw, *, deadline_ns):
        self._owner('zero_gain')
        self.h.sequence.append(('zero_gain', self.group.port, mid))
        self.h.ledger.append(('zero_gain', self.group.port))
        raw = self.raw(mid)+self.c.get('zero_gain_drift', {}).get(mid, 0.)
        batch = self._exchange('zero_gain', [(encode_motion(mid, q_raw, 0., 0.), type2(mid, raw, mode=2))],
                               deadline_ns)
        stall = self.c.get('zero_gain_stall', {}).get(mid)
        if stall:
            self.h.clock.advance_to(self.h.clock()+stall)
        return batch.rows[mid, 'feedback']

    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check):
        self._owner('hold')
        cycle = self.holds
        self.holds += 1
        try:
            check()
            if tuple(wires) != self.last_output or voltage_id != self.group.ids[cycle % 3]:
                raise ValueError('Hold must equal the last validated output and rotate voltage')
            if self.c.get('hold_fail') == (self.group.port, cycle):
                raise RuntimeError('Injected owner hold failure')
            if self.c.get('late_hold') == (self.group.port, cycle):
                self.h.clock.advance_to(deadline_ns+1_000_000)
                raise TimeoutError('Injected native late hold reply')
            mode = 0 if self.c.get('hold_mode0') == (self.group.port, cycle) else 2
            hot = self.c.get('hold_temperature_dc', {}).get((self.group.port, cycle), {})
            self.h.ledger.append(('hold', self.group.port))
            hold = self._exchange('hold', [(wire, type2(mid, self.raw(mid), mode=mode, temperature_dc=hot.get(mid, 250)))
                                           for wire, mid in zip(wires, self.group.ids)], deadline_ns)
            self.last_hold = hold
            check()
            prefix_future.set_result(hold)
            voltage = self.c.get('voltage', {}).get((self.group.port, cycle), 40.)
            name = self.c.get('voltage_param', {}).get((self.group.port, cycle), 'voltage')
            batch = self._exchange('voltage', [(codec.read_request(voltage_id, name),
                                                type17(voltage_id, name, 0 if name == 'run_mode' else voltage))],
                                   deadline_ns)
            if self.c.get('hold_rebind') == (self.group.port, cycle):  # Same bytes, a different object.
                hold = Batch(hold.group, hold.label, hold.records, hold.stats, hold.rows, hold.record_image,
                             hold.stats_image, hold.completed_ns)
            return hold, batch
        except BaseException as error:
            if not prefix_future.done():
                prefix_future.set_exception(error)
            raise

    def output(self, wires, *, deadline_ns, check):
        self._owner('output')
        check()
        wires = tuple(wires)
        if len(wires) != 3 or any((int.from_bytes(w[2:6], 'big') >> 3) & 255 != mid
                                  for w, mid in zip(wires, self.group.ids)):
            raise ValueError('Mock output requires exactly three ascending group Type1 wires')
        cycle = len(self.outputs)-1
        mode, fault = 2, 0
        if self.c.get('output_mode0') == (self.group.port, cycle):
            mode = 0
        if self.c.get('output_fault') == (self.group.port, cycle):
            fault = 1
        sent = list(wires)
        if self.c.get('output_tx') == (self.group.port, cycle):  # Owner journals a different third wire.
            sent[2] = encode_motion(self.group.ids[2], RAW[self.group.ids[2]]+.01, 0., 0.)
        self.h.ledger.append(('output', self.group.port))
        batch = self._exchange('output', [(wire, type2(mid, self.raw(mid), mode=mode, fault=fault))
                                          for wire, mid in zip(sent, self.group.ids)], deadline_ns)
        self.outputs.append(wires)
        self.last_output = wires
        return batch

    def stop_repeated(self, *, total_budget_ns=1_000_000_000, rounds=3):
        self._owner('stop_repeated', stop=True)
        barrier = self.c.get('stop_barrier')
        if barrier is not None:
            barrier.wait(timeout=1)
        ids = list(self.group.ids)
        if self.c.get('stop_ambiguous') == self.group.port:
            return {'complete': False, 'confirmed_ids': ids[1:], 'unconfirmed_ids': [], 'ambiguous_ids': ids[:1],
                    'faults': {}, 'rounds': 3, 'physical_cutoff_required': True}
        return {'complete': True, 'confirmed_ids': ids, 'unconfirmed_ids': [], 'ambiguous_ids': [],
                'faults': {str(mid): 0 for mid in ids}, 'rounds': 1, 'physical_cutoff_required': False}

    def close(self):
        self.closed = True


class Observer:
    def __init__(self, hook=None):
        self.calls, self.hook, self.snapshots = 0, hook, []

    def consume(self, snapshot):
        if len(snapshot['motors']) != 24 or snapshot['output_allowed'] is not False:
            raise AssertionError('Ordinary observer snapshot contract')
        self.snapshots.append([(row['motor_id'], row['parameter'], row['value'], row['request_ns'], row['received_ns'])
                               for row in snapshot['motors']])
        self.calls += 1
        targets = {mid: MODEL[mid] for mid in IDS}
        if self.hook is not None:
            self.hook(self.calls, targets)
        return {'status': 'TICK_OBSERVED_NO_OUTPUT', 'tick_index': self.calls-1, 'output_allowed': False,
                'q_target_rad_diagnostic_only': [targets[mid] for mid in shadow.CAN_ORDER]}


class Harness:
    @classmethod
    def setUpClass(cls):
        # The foreground main scope selects a 100 us switch interval (PACING).
        cls.original_switch = sys.getswitchinterval()
        sys.setswitchinterval(.0001)

    @classmethod
    def tearDownClass(cls):
        sys.setswitchinterval(math.nextafter(cls.original_switch, math.inf))
        assert sys.getswitchinterval() == cls.original_switch

    def setUp(self):
        self.controls, self.adapters, self.sequence, self.events = {}, {}, [], []
        self.cancelled = threading.Event()
        self.injected_cancel = threading.Event()
        self.stop_requested = threading.Event()
        self.release_extra = {}
        self.cycle_release = 0
        self.imu_calls = 0
        self.clock = VirtualClock()
        self.ledger, self.current_calls, self.current_failed = [], [], False
        self.last_imu = None

    def factory(self, group):
        value = MockType1(group, self)
        self.adapters[group.port] = value
        return value

    @contextmanager
    def worker_scope(self, port, mask):
        self.events.append(('worker-enter', port))
        yield {'native_tid': threading.get_native_id(), 'cpu_mask': list(mask), 'timer_slack_ns': 1000,
               'file_only_mock_readback': True}
        self.events.append(('worker-exit', port))

    @contextmanager
    def main_scope(self):
        # Like the foreground MainScope: GC is deferred for the run and restored.
        self.events.append(('main-enter',))
        enabled = gc.isenabled()
        gc.disable()
        try:
            yield {'file_only_mock_readback': True, 'gc_deferred_during_cycles': True}
        finally:
            if enabled:
                gc.enable()
            self.events.append(('main-exit',))

    def check(self):
        if self.injected_cancel.is_set():
            raise RuntimeError('Injected cancellation')

    def current(self):
        """Full current guard, distinct from check_cancelled; a failure latches like the native guard."""
        cycle = self.cycle_release  # n+1 once cycle n has been released; 0 during setup.
        self.current_calls.append((cycle, threading.get_ident()))
        fail = self.controls.get('current_fail')  # (cycle, boundary 1..3)
        if fail is not None and cycle == fail[0]+1 and sum(1 for c, _ in self.current_calls if c == cycle) == fail[1]:
            self.current_failed = True
        if self.current_failed:
            raise RuntimeError('Injected current guard failure')
        self.check()

    def cancel_io(self):
        self.events.append(('cancel', self.clock()))
        self.cancelled.set()

    def imu(self):
        now = self.clock()
        sample = {'read_started_monotonic_ns': now, 'read_finished_monotonic_ns': now, 'frame': 'sensor',
                  'accel_m_s2': [0., 0., 9.81], 'gyro_rad_s': [0., 0., 0.]}
        hook = self.controls.get('imu')
        if hook is not None:
            hook(self.imu_calls, sample)  # Call 0 is the pre-enable sample; call n+1 is cycle n.
        self.imu_calls += 1
        self.last_imu = sample
        return sample

    def release_wait(self, when):
        extra = self.release_extra.get(self.cycle_release, 0)
        self.cycle_release += 1
        return self.clock.advance_to(when+extra)

    def run_case(self, mode='learned_boxed', observer=None, value=None, **kw):
        kw.setdefault('clock', self.clock)
        return runner.run(value or admitted(mode), factory=self.factory, imu_read=self.imu,
            observer=observer or Observer(), check_current=self.current, check_cancelled=self.check,
            cancel_io=self.cancel_io,
            model_setup=lambda o, **k: dict(k, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.worker_scope, main_scope=self.main_scope, release_wait=self.release_wait,
            announce=lambda: self.events.append(('announce',)), stop_requested=self.stop_requested,
            backend_usage={'kind': 'injected_file_only_mock'}, execute=True, **kw)

    def assert_all_stopped(self, result):
        for port in PORTS:
            self.assertEqual([m for m, _ in self.adapters[port].calls].count('stop_repeated'), 1, port)
            self.assertTrue(self.adapters[port].closed)
        self.assertIn(('main-exit',), self.events)

    def output_cycles(self, port):
        return len(self.adapters[port].outputs)-1  # Excludes the initial zero-gain hold.


class RunnerTests(Harness, unittest.TestCase):

    # PLAN and admission ------------------------------------------------------
    def test_default_plan_opens_nothing(self):
        def forbidden(*_, **__):
            self.fail('PLAN called a capability')
        result = runner.run(admitted(), factory=forbidden, imu_read=forbidden, observer=None,
                            cancel_io=forbidden, main_scope=forbidden)
        self.assertEqual(result['status'], 'PLAN')
        self.assertFalse(result['opens_devices'])
        self.assertEqual(result['per_cycle_requests'], 28)
        self.assertEqual([row['motor_id'] for row in result['enable_order']], [1, 4, 7, 10, 2, 5, 8, 11, 3, 6, 9, 12])
        self.assertEqual([row['port'] for row in result['enable_order']][:4], ['port3', 'port2', 'port0', 'port1'])
        for key in ('learned_targets_attempted', 'motor_enable_sent', 'type1_sent', 'positive_gain_sent',
                    'output_approval_granted_here', 'live_type1_qualified'):
            self.assertIs(result[key], False)
        self.assertTrue(all(value is None for value in result['physical_observations'].values()))

    def test_admission_rejections(self):
        cases = {
            'mode': lambda a: a.update(mode='walking'),
            'duration': lambda a: a.update(duration_s=30),
            'pacing': lambda a: a['pacing'].update(request_gap_us=890),
            'post_reply': lambda a: a.update(post_reply_policy=None),
            'weight': lambda a: a['profile'].update(policy_weight=.01),
            'kp_cap': lambda a: a['profile']['axes']['4'].update(kp=3.5),
            'reserve': lambda a: a['profile'].update(stop_duration_s=1.),
            'plan_sign': lambda a: a['profile']['axes']['3'].update(sign=-1),
            'topology': lambda a: a['ids_by_port'].update(port0=[1, 2, 3], port3=[7, 8, 9]),
            'uid': lambda a: a['uids_by_id'].update({'5': 'ff'}),
            'turns': lambda a: a['reference_turns_by_id'].update({'3': 0}),
            'first_cycle': lambda a: a.update(first_cycle_post_reply=1),
            'outer_effective_bounds': lambda a: a['profile']['axes']['8'].update(
                lower_rad=a['profile']['axes']['8']['physical_lower_rad']-runner.BRANCH_MARGIN_RAD),
            'missing': lambda a: a.pop('model_plan'),
            'imu_limit_missing': lambda a: a['profile'].pop('imu_gyro_limit_rad_s'),
            'imu_tilt_out_of_scope': lambda a: a['profile'].update(imu_tilt_limit_rad=.5),
            'imu_tilt_nonfinite': lambda a: a['profile'].update(imu_tilt_limit_rad=math.nan),
            'imu_norm_interval': lambda a: a['profile'].update(imu_accel_norm_min_m_s2=10.2),
            'imu_norm_wide': lambda a: a['profile'].update(imu_accel_norm_min_m_s2=8.8, imu_accel_norm_max_m_s2=10.4),
            'observer_kwargs': lambda a: resealed(a['model_plan']).pop('observer_kwargs') and resealed(a['model_plan']),
            'mount_path': lambda a: resealed(a['model_plan']['observer_kwargs'].update(
                imu_mount_candidate='/nonexistent/mount.json') or a['model_plan']),
            'reviewed_accel': lambda a: resealed(a['model_plan']['observer_kwargs'].update(
                apply_reviewed_accel_calibration=True) or a['model_plan']),
            'bias_candidate': lambda a: resealed(a['model_plan']['observer_kwargs'].update(
                gyro_bias_candidate=dict(BIAS, approved_for_runtime=True)) or a['model_plan']),
        }
        for name, mutate in cases.items():
            with self.subTest(name):
                value = admitted()
                mutate(value)
                with self.assertRaises(Exception):
                    runner.plan(value)

    # Full runs ---------------------------------------------------------------
    def check_cycle_rows(self, result):
        keys = ('release_ns', 'begin_ns', 'hold_first_write_ns', 'hold_last_reply_ns', 'gather_ns',
                'infer_end_ns', 'voltage_join_ns', 'final_gate_ns', 'encode_end_ns', 'output_first_write_ns',
                'output_last_reply_ns', 'cycle_end_ns')
        for row in result['cycles']:
            stamps = [row[key] for key in keys]
            self.assertTrue(all(type(value) is int for value in stamps), row['index'])
            self.assertEqual(stamps, sorted(stamps), row['index'])
            self.assertEqual(row['request_count'], 28)
            self.assertTrue(row['completed'])
            self.assertLessEqual(row['cycle_end_ns']-row['begin_ns'], 21_000_000)
        self.assertEqual([row['slot'] for row in result['cycles']], list(range(len(result['cycles']))))
        for port in PORTS:
            adapter = self.adapters[port]
            self.assertEqual(adapter.holds, len(result['cycles']))
            self.assertEqual(self.output_cycles(port), len(result['cycles']))

    def test_zero_gain_timing_full_run_truthful_flags(self):
        observer = Observer()
        result = self.run_case('zero_gain_timing', observer)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['physical_cutoff_required'])
        self.assertTrue(result['normal_ramp_completed'])
        self.assertIs(result['motor_enable_sent'], True)
        self.assertIs(result['type1_sent'], True)
        self.assertIs(result['learned_targets_attempted'], False)
        self.assertIs(result['positive_gain_sent'], False)
        for port in PORTS:
            for wires in self.adapters[port].outputs:
                self.assertTrue(all(wire[11:15] == bytes(4) for wire in wires))
        self.check_cycle_rows(result)
        self.assertEqual(result['cycles'][-1]['label'], 'graceful_stop')
        self.assertEqual(result['cycles'][-1]['command']['phase'], 'stopped')
        consumed = [row for row in result['cycles'] if row['observed'] is not None]
        self.assertEqual(observer.calls, len(consumed))
        self.assertTrue(consumed and all(row['model_target_by_id'] is not None for row in consumed))
        self.assertTrue(all(row['label'] == 'zero_gain_timing' for row in consumed))
        self.assert_all_stopped(result)
        self.assertEqual(self.sequence[0::2], [('enable', port, mid) for port, mid in
                         runner.enable_order(tuple(Group(p, ids) for p, ids in PORT_IDS.items()))])
        json.dumps(runner.evidence_report(result))
        # Exactly the three boundary current checks per cycle, all on the coordinating thread.
        main = threading.get_ident()
        self.assertTrue(all(ident == main for _, ident in self.current_calls))
        for row in result['cycles']:
            self.assertEqual(sum(1 for c, _ in self.current_calls if c == row['index']+1), 3, row['index'])
        # 12 Type3 + 12 zero-gain + initial hold + (hold + output) per cycle.
        self.assertEqual(result['voltage_guard']['checks_before_type1'], 25+2*len(result['cycles']))
        settings = result['watchdog_settings']
        self.assertEqual(settings['creator_native_tid'], result['worker_settings']['imu']['native_tid'])
        self.assertNotEqual(settings['native_tid'], threading.get_native_id())
        self.assertEqual(result['host_watchdog_placement'], 'created_on_imu_owner_inherits_cpu0_3_not_main_cpu4')

    def test_learned_boxed_full_run_maps_can_order_and_blends(self):
        observer = Observer()
        result = self.run_case('learned_boxed', observer)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        self.assertIs(result['learned_targets_attempted'], True)
        self.assertIs(result['positive_gain_sent'], True)
        self.assertTrue(any(wire[11:13] != bytes(2) for port in PORTS for wires in self.adapters[port].outputs
                            for wire in wires))
        self.check_cycle_rows(result)
        q0 = [result['q0_model_rad_by_id'][str(mid)] for mid in IDS]
        weighted = [row for row in result['cycles'] if row['weight']]
        self.assertTrue(weighted)
        for row in weighted:
            self.assertEqual(row['model_target_by_id'], [MODEL[mid] for mid in IDS])
            self.assertLessEqual(row['weight'], .005)
            for mid in IDS:
                self.assertAlmostEqual(row['blended_target_by_id'][mid-1],
                                       q0[mid-1]+row['weight']*(MODEL[mid]-q0[mid-1]), places=15)
        self.assertIn('policy_output', {row['label'] for row in result['cycles']})
        self.assertEqual(result['cycles'][-1]['label'], 'graceful_stop')
        self.assert_all_stopped(result)
        consumed = [row for row in result['cycles'] if row['observed'] is not None]
        self.assertEqual(len(consumed), len(observer.snapshots))
        for row, snapshot in zip(consumed, observer.snapshots):  # The observer saw this cycle's own hold.
            expected = []
            for port in PORTS:
                for record, mid in zip(row['hold'][port].records, PORT_IDS[port]):
                    expected += [(mid, 'position', record.start_ns, record.received_ns),
                                 (mid, 'velocity', record.start_ns, record.received_ns)]
            self.assertEqual([(m, name, a, b) for m, name, _, a, b in snapshot], expected, row['index'])
            for mid, name, value, _, _ in snapshot:
                if name == 'position':
                    self.assertAlmostEqual(value, RAW[mid], places=3)

    def test_bound_native_encoder_specs_follow_or_1810_1815(self):
        from singularitydog_hw.native_policy_batch_encode import _reference_wires
        bound = []
        class Module:
            binary_sha256 = 'b'*64
            def bind(self, specs):
                bound.append(specs)
                return lambda command: _reference_wires(command, specs)
        result = self.run_case('learned_boxed', encoder=Module())
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        self.assertEqual(result['encoder'], {'kind': 'verified_native_batch', 'binary_sha256': 'b'*64})
        self.assertEqual(len(bound), 1)
        origin = [result['trial_origin_model_rad_by_id'][str(mid)] for mid in IDS]
        axes = admitted()['profile']['axes']
        self.assertEqual(bound[0], tuple((FIXED[mid], SIGN[mid], axes[str(mid)]['lower_rad'], axes[str(mid)]['upper_rad'],
            origin[mid-1], DEG, .1) for mid in IDS))
        self.assertIs(result['positive_gain_sent'], True)

    def test_sigusr1_graceful_ramp_completes_early(self):
        self.controls['param_rows'] = True  # Row-shaped read_params replies are accepted too.
        def hook(call, _):
            if call == 3:
                self.stop_requested.set()
        result = self.run_case('learned_boxed', Observer(hook))
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        self.assertLess(len(result['cycles']), 60)
        self.assertEqual(result['cycles'][-1]['command']['phase'], 'stopped')
        self.check_cycle_rows(result)

    def test_stop_unconfirmed_requires_power_cutoff(self):
        self.stop_requested.set()
        self.controls['stop_ambiguous'] = 'port1'
        result = self.run_case('zero_gain_timing')
        self.assertEqual(result['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertIs(result['stop_confirmed'], False)
        self.assertIs(result['physical_cutoff_required'], True)
        self.assertIsNone(result['primary_error'])

    def test_budget_exhaustion_without_stopped_is_error(self):
        class NeverStops(PolicyMotionEnvelope):
            def _stop_command(self, now):
                return (*super()._stop_command(now)[:4], 'stopping', 'gain_ramp')
        with patch.object(runner, 'PolicyMotionEnvelope', NeverStops):
            result = self.run_case('zero_gain_timing')
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Finite run budget expired', result['primary_error']['message'])
        self.assertTrue(result['stop_confirmed'])
        self.assert_all_stopped(result)

    # Failure injection -------------------------------------------------------
    def assert_aborted_at(self, result, cycle, message, *, output_sent=False):
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn(message, result['primary_error']['message'])
        self.assertTrue(self.cancelled.is_set())
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['completed_cycles'], cycle)
        for port in PORTS:
            self.assertEqual(self.output_cycles(port), cycle+int(output_sent), port)
        self.assert_all_stopped(result)

    # IMU body limits (OM.validate_inputs) -----------------------------------
    def imu_at(self, call, **values):
        def hook(index, sample):
            if index == call:
                sample.update(values)
        self.controls['imu'] = hook

    def test_imu_body_limits_reject_before_enable(self):
        cases = {'tilt': ({'accel_m_s2': [3., 0., 9.3]}, 'Body tilt/angular velocity exceeded'),
                 'gyro': ({'gyro_rad_s': [0., 0., .6]}, 'Body tilt/angular velocity exceeded'),
                 'norm_low': ({'accel_m_s2': [0., 0., 9.]}, 'gravity-proxy norm'),
                 'norm_high': ({'accel_m_s2': [0., 0., 10.5]}, 'gravity-proxy norm'),
                 'sideways_saturated': ({'accel_m_s2': [3., 0., 0.], 'gyro_rad_s': [5., 5., 5.]}, 'gravity-proxy norm'),
                 'upside_down': ({'accel_m_s2': [0., 0., -9.81]}, 'Body tilt/angular velocity exceeded'),
                 'frame': ({'frame': 'body'}, 'Uncorrected sensor-frame IMU required'),
                 'no_frame': ({'frame': None}, 'Uncorrected sensor-frame IMU required'),
                 'corrected': ({'gyro_bias_subtracted': True}, 'Uncorrected sensor-frame IMU required')}
        for name, (values, message) in cases.items():
            with self.subTest(name):
                self.setUp()
                self.imu_at(0, **values)
                result = self.run_case('learned_boxed')
                self.assertEqual(result['status'], 'ABORTED')
                self.assertIn(message, result['primary_error']['message'])
                self.assertIs(result['motor_enable_sent'], False)
                self.assertIs(result['type1_sent'], False)
                self.assertIs(result['positive_gain_sent'], False)
                self.assertEqual(self.sequence, [])
                self.assertTrue(result['stop_confirmed'])
                self.assert_all_stopped(result)

    def test_imu_tilt_mid_run_stops_all_ports_before_output(self):
        self.imu_at(31, accel_m_s2=[0., 2.5, 9.45])  # Cycle 30, policy ramp active; tilt ~.26 rad.
        self.controls['stop_barrier'] = threading.Barrier(4)
        observer = Observer()
        result = self.run_case('learned_boxed', observer)
        self.assert_aborted_at(result, 30, 'Body tilt/angular velocity exceeded')
        self.assertFalse(self.controls['stop_barrier'].broken)
        self.assertEqual(observer.calls, 30)
        self.assertIs(result['positive_gain_sent'], True)
        self.assertEqual(result['cycles'][30]['imu_limit_check'], None)
        check = result['cycles'][29]['imu_limit_check']
        self.assertEqual(check['tilt_rad'], 0.)
        self.assertAlmostEqual(check['raw_accel_norm_m_s2'], 9.81)
        self.assertAlmostEqual(result['pre_enable_imu_limit_check']['tilt_rad'], 0.)

    def test_imu_gyro_during_gain_down_stops_all_ports(self):
        self.imu_at(71, gyro_rad_s=[.3, 0., .45])  # Cycle 70, after stop_at (cycle 63): no inference.
        observer = Observer()
        result = self.run_case('learned_boxed', observer)
        self.assert_aborted_at(result, 70, 'Body tilt/angular velocity exceeded')
        self.assertEqual(result['cycles'][69]['label'], 'graceful_stop')
        self.assertLess(observer.calls, 70)
        self.assertFalse(result['normal_ramp_completed'])

    def test_imu_norm_during_zero_gain_timing_stops(self):
        self.imu_at(6, accel_m_s2=[0., 0., 10.3])
        self.assert_aborted_at(self.run_case('zero_gain_timing'), 5, 'gravity-proxy norm')

    def test_accel_correction_injected_exactly_when_selected(self):
        class Correction:
            def correct(self, accel):
                return list(accel), math.hypot(*accel)
        with self.assertRaisesRegex(ValueError, 'Acceleration correction'):
            self.run_case(accel_correction=Correction())
        value = admitted()
        resealed(value['model_plan']['observer_kwargs'].update(
            accel_input_hypothesis={'path': '/synthetic/accel.json', 'sha256': 'c'*64}) or value['model_plan'])
        kw = dict(factory=self.factory, imu_read=self.imu, observer=Observer(), check_current=self.check,
                  check_cancelled=self.check, cancel_io=self.cancel_io, model_setup=lambda o, **k: k,
                  worker_scope=self.worker_scope, main_scope=self.main_scope, release_wait=self.release_wait,
                  announce=lambda: None, backend_usage={'kind': 'injected_file_only_mock'}, execute=True)
        self.assertIs(runner.plan(value)['imu_accel_input_hypothesis_selected'], True)
        with self.assertRaisesRegex(ValueError, 'Acceleration correction'):
            runner.run(value, **kw)
        with self.assertRaisesRegex(ValueError, 'Acceleration correction'):
            runner.run(value, accel_correction=object(), **kw)
        self.assertEqual(self.adapters, {})

    def test_late_hold_reply_aborts_before_output(self):
        self.controls['late_hold'] = ('port1', 2)
        result = self.run_case()
        self.assert_aborted_at(result, 2, '')
        self.assertTrue(result['primary_error']['type'] == 'TimeoutError' or
                        'late hold reply' in result['primary_error']['message'], result['primary_error'])

    def test_mode_zero_hold_reply_rejected_before_inference(self):
        self.controls['hold_mode0'] = ('port2', 2)
        observer = Observer()
        result = self.run_case(observer=observer)
        self.assert_aborted_at(result, 2, 'fault/mode')
        self.assertEqual(observer.calls, 2)

    def test_mode_zero_and_fault_output_replies_rejected(self):
        for key in ('output_mode0', 'output_fault'):
            with self.subTest(key):
                self.setUp()
                self.controls[key] = ('port0', 3)
                self.assert_aborted_at(self.run_case(), 3, 'fault/mode', output_sent=True)

    def test_out_of_range_voltage_aborts_before_output(self):
        self.controls['voltage'] = {('port3', 2): 43.}
        self.assert_aborted_at(self.run_case(), 2, 'voltage outside profile')

    def test_envelope_rejection_aborts_without_clipping(self):
        calls = []
        class Rejecting(PolicyMotionEnvelope):
            def step(self, target, sample, *, now_s):
                calls.append(target)
                if len(calls) == 4:
                    self.emergency_fault('Injected target outside envelope')
                return super().step(target, sample, now_s=now_s)
        with patch.object(runner, 'PolicyMotionEnvelope', Rejecting):
            result = self.run_case()
        self.assert_aborted_at(result, 3, 'Injected target outside envelope')
        self.assertEqual(result['primary_error']['type'], MotionFault.__name__)

    def test_model_target_out_of_range_rejected(self):
        def hook(call, targets):
            if call == 3:
                targets[6] = .6  # Hip limit is .5.
        self.assert_aborted_at(self.run_case(observer=Observer(hook)), 2, 'ID6 learned target outside model range')

    def test_blended_target_outside_local_range_rejected_not_clipped(self):
        # Defence in depth: unreachable with the reviewed model ranges, so only the test widens them.
        wide = {mid: (-30., 30.) for mid in IDS}
        def hook(call, targets):
            if call == 41:  # Cycle 40: weight ~.0034, so 20 rad blends ~.065 rad past q0 (> 3 deg box).
                targets[8] = 20.
        with patch.object(runner, 'MODEL_TARGET_LIMITS_BY_ID', wide):
            result = self.run_case(observer=Observer(hook))
        self.assert_aborted_at(result, 40, 'ID8 blended target outside local physical range')

    def test_skipped_slot_stops_before_another_hold(self):
        self.release_extra[3] = 25_000_000
        result = self.run_case()
        self.assert_aborted_at(result, 3, 'missed its slot')
        for port in PORTS:
            self.assertEqual(self.adapters[port].holds, 3)

    def test_late_release_within_slot_fails_hold_gate_before_refresh(self):
        self.release_extra[3] = 1_500_000
        result = self.run_case()
        self.assert_aborted_at(result, 3, 'Command/sample gap exceeded before feedback hold')
        for port in PORTS:
            self.assertEqual(self.adapters[port].holds, 3)

    def test_owner_exception_cancels_siblings_and_stops_all_ports_concurrently(self):
        self.controls['hold_fail'] = ('port2', 2)
        self.controls['stop_barrier'] = threading.Barrier(4)
        result = self.run_case()
        self.assert_aborted_at(result, 2, 'Injected owner hold failure')
        self.assertFalse(self.controls['stop_barrier'].broken)
        self.assertIn('port2 owner', result['stop_reason'])

    def test_cancellation_aborts(self):
        def hook(call, _):
            if call == 3:
                self.injected_cancel.set()
        result = self.run_case(observer=Observer(hook))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Injected cancellation', result['primary_error']['message'])
        self.assertEqual(result['completed_cycles'], 2)
        self.assert_all_stopped(result)

    def test_each_boundary_current_check_blocks_that_cycles_output(self):
        for boundary in (1, 2, 3):
            with self.subTest(boundary=boundary):
                self.setUp()
                self.controls['current_fail'] = (3, boundary)
                self.controls['stop_barrier'] = threading.Barrier(4)
                observer = Observer()
                result = self.run_case(observer=observer)
                self.assert_aborted_at(result, 3, 'Injected current guard failure')
                self.assertFalse(self.controls['stop_barrier'].broken)
                for port in PORTS:
                    self.assertEqual(self.adapters[port].holds, 3 if boundary == 1 else 4, port)
                self.assertEqual(observer.calls, 4 if boundary == 3 else 3)
                self.assertEqual(sum(1 for c, _ in self.current_calls if c == 4), boundary)

    def test_host_watchdog_stops_all_ports_while_main_is_stalled(self):
        seen = {}
        def hook(call, _):
            if call == 3:  # Cycle 2, positive gain held; the main thread stops kicking for 50 ms.
                self.clock.advance_to(self.clock()+50_000_000)
                end = time.monotonic()+2
                while time.monotonic() < end and not all(
                        'stop_repeated' in [m for m, _ in self.adapters[port].calls] for port in PORTS):
                    time.sleep(.001)
                seen['stopped_before_return'] = all(
                    'stop_repeated' in [m for m, _ in self.adapters[port].calls] for port in PORTS)
        self.controls['stop_barrier'] = threading.Barrier(4)
        result = self.run_case('learned_boxed', Observer(hook))
        self.assertIs(seen['stopped_before_return'], True)
        self.assertEqual(result['stop_reason'], 'Host output heartbeat expired')
        self.assertEqual(result['host_watchdog_reason'], 'Host output heartbeat expired')
        self.assertIs(result['positive_gain_sent'], True)
        self.assertFalse(self.controls['stop_barrier'].broken)
        self.assert_aborted_at(result, 2, 'Host output heartbeat expired')

    def test_cycle_gates_reject_before_output(self):
        def mutate_imu(call, _):
            if call == 4:
                self.last_imu['gyro_rad_s'][0] += 1e-6
        def mutate_hold(call, _):
            if call == 4:
                self.adapters['port0'].last_hold.records[0].rx[10] ^= 1
        cases = {  # name: (controls, observer hook, release extra, message, output sent, inferred)
            'measured_hold_limits': ({'hold_temperature_dc': {('port0', 3): {7: 500}}}, None, 0,
                                     'ID7 measured temperature', False, False),
            'stale_previous_feedback': ({}, None, 500_000, 'stale feedback before hold', False, False),
            'hold_future_binding': ({'hold_rebind': ('port2', 3)}, None, 0,
                                    'Current exact hold/full Future binding failed', False, True),
            'rotating_voltage_wire': ({'voltage_param': {('port1', 3): 'run_mode'}}, None, 0,
                                      'Current rotating same-group voltage required', False, True),
            'imu_changed_after_inference': ({}, mutate_imu, 0, 'IMU image changed after inference', False, True),
            'hold_bytes_changed_after_inference': ({}, mutate_hold, 0, 'changed after publication', False, True),
            'output_record_tx': ({'output_tx': ('port3', 3)}, None, 0,
                                 'Output record differs from the validated outgoing wire', True, True)}
        for name, (controls, hook, extra, message, sent, inferred) in cases.items():
            with self.subTest(name):
                self.setUp()
                self.controls.update(controls)
                if extra:
                    self.release_extra[3] = extra
                observer = Observer(hook)
                result = self.run_case(observer=observer)
                self.assert_aborted_at(result, 3, message, output_sent=sent)
                self.assertEqual(observer.calls, 4 if inferred else 3)
                if name == 'stale_previous_feedback':
                    self.assertTrue(all(self.adapters[port].holds == 3 for port in PORTS))

    # Setup gates -------------------------------------------------------------
    def test_preflight_and_setup_failures_never_enable(self):
        cases = {'wrong_uid': (5, 'ID5 UID mismatch'), 'wrong_firmware': (9, 'ID9 fresh firmware'),
                 'timeout_ticks': (3999, 'watchdog readback'), 'setup_voltage': (34., 'supply voltage'),
                 'moved_on_stop': (3, 'moved during preparation')}
        for key, (value, message) in cases.items():
            with self.subTest(key):
                self.setUp()
                self.controls[key] = value
                result = self.run_case()
                self.assertEqual(result['status'], 'ABORTED')
                self.assertIn(message, result['primary_error']['message'])
                self.assertIs(result['motor_enable_sent'], False)
                self.assertIs(result['type1_sent'], False)
                self.assertEqual(self.sequence, [])
                self.assert_all_stopped(result)

    def test_stale_voltage_blocks_zero_gain_type1(self):
        self.controls['enable_stall'] = 1
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('ID1 voltage stale', result['primary_error']['message'])
        self.assertEqual(self.sequence, [('enable', 'port3', 1)])
        self.assertIs(result['motor_enable_sent'], True)
        self.assertIs(result['type1_sent'], False)
        self.assert_all_stopped(result)

    def test_setup_gates_reject_before_the_next_write(self):
        def narrow(value):
            value['profile']['start_pose_bounds']['5'] = [BASE[5]+.1*DEG, BASE[5]+.49*DEG]
            return value
        order = [('enable', 'port3', 1), ('zero_gain', 'port3', 1), ('enable', 'port2', 4), ('zero_gain', 'port2', 4)]
        late = {mid: 31_000_000 for mid in (3, 6, 9, 12)}  # Last four axes; every kick gap stays below 40 ms.
        cases = {  # name: (controls, admitted, message, expected enable/zero-gain sequence)
            'initial_velocity': ({'velocity': .5}, None, 'initial velocity', []),
            'start_pose_bounds': ({}, narrow(admitted()), 'ID5 outside reviewed starting posture', []),
            'branch_turns': ({'drift': {1: 2*math.pi}}, None, 'ID1 branch differs from admitted reference turns', []),
            'announcement_order': ({'stamp_age': {(port, 'read_can_timeout', 1): 50_000_000 for port in PORTS}}, None,
                                   'watchdog read predates announcement', []),
            'enable_displacement': ({'enable_drift': {4: 1.5*DEG}}, None, 'ID4 startup trial displacement', order[:3]),
            'zero_gain_displacement': ({'zero_gain_drift': {4: 1.5*DEG}}, None, 'ID4 startup trial displacement',
                                       order),
            'enable_sequence_120ms': ({'zero_gain_stall': late}, None, 'Zero-gain enable sequence deadline exceeded',
                                      None)}
        for name, (controls, value, message, sequence) in cases.items():
            with self.subTest(name):
                self.setUp()
                self.controls.update(controls)
                result = self.run_case(value=value)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertIn(message, result['primary_error']['message'])
                if sequence is None:
                    self.assertEqual(len(self.sequence), 24)
                else:
                    self.assertEqual(self.sequence, sequence)
                self.assertNotIn('initial_hold', result)
                self.assertTrue(all(self.adapters[port].outputs == [] for port in PORTS))
                self.assertIs(result['positive_gain_sent'], False)
                self.assertTrue(result['stop_confirmed'])
                self.assert_all_stopped(result)

    def test_initial_hold_record_tx_mismatch_stops_before_cycles(self):
        self.controls['output_tx'] = ('port1', -1)
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Exact initial zero-gain hold batch required', result['primary_error']['message'])
        self.assertTrue(all(self.adapters[port].holds == 0 and len(self.adapters[port].outputs) == 1 for port in PORTS))
        self.assertEqual(result['cycles'], [])
        self.assert_all_stopped(result)

    def test_voltage_guard_precedes_every_enable_and_type1_group(self):
        original = runner.checked_voltage_cache
        def spy(*args):
            if sys._getframe(1).f_code.co_name == 'require_voltage_before_type1':
                self.ledger.append(('voltage_guard', None))
            return original(*args)
        with patch.object(runner, 'checked_voltage_cache', spy):
            result = self.run_case('learned_boxed')
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        size = {'enable': 1, 'zero_gain': 1, 'hold': 4, 'output': 4}
        armed, kind, count, groups = False, None, 0, []
        for event, port in self.ledger:
            if event == 'voltage_guard':
                self.assertEqual(count, 0, 'Guard inside a write group')
                armed = True
                continue
            if count == 0:
                self.assertTrue(armed, 'Type3/Type1 write group without a preceding voltage guard: '+event)
                armed, kind = False, event
                groups.append(event)
            self.assertEqual(event, kind)
            count = (count+1) % size[event]
        self.assertEqual(count, 0)
        cycles = len(result['cycles'])
        self.assertEqual(groups, ['enable', 'zero_gain']*12+['output']+['hold', 'output']*cycles)
        self.assertEqual(result['voltage_guard']['checks_before_type1'], 25+2*cycles)

    def test_stale_voltage_blocks_each_specific_write(self):
        class StallingEncoder:
            binary_sha256 = 'b'*64
            def __init__(self, clock):
                self.clock, self.calls = clock, 0
            def bind(self, specs):
                from singularitydog_hw.native_policy_batch_encode import _reference_wires
                def wires(command):
                    self.calls += 1
                    if self.calls == 4:  # Cycle 3: 2 ms between the voltage join check and the output guard.
                        self.clock.advance_to(self.clock()+2_000_000)
                    return _reference_wires(command, specs)
                return wires
        old = lambda label, n, age: {(port, label, n): age for port in PORTS}
        cases = {  # name: (controls, release extra for cycle 0, encoder, enable/zero-gain count, holds, outputs)
            'before_type3_enable': ({'stamp_age': old('read_voltage', 0, 100_000_000),
                                     'zero_gain_stall': {1: 30_000_000}}, 0, False, 2, 0, 0),
            'before_cycle_hold': ({'stamp_age': old('read_voltage', 1, 115_000_000)}, 15_000_000, False, 24, 0, 1),
            'before_cycle_output': ({'stamp_age': {('port3', 'voltage', 3): 125_000_000}}, 0, True, 24, 4, 4)}
        for name, (controls, extra, stalled, writes, holds, outputs) in cases.items():
            with self.subTest(name):
                self.setUp()
                self.controls.update(controls)
                if extra:
                    self.release_extra[0] = extra
                kw = {'encoder': StallingEncoder(self.clock)} if stalled else {}
                result = self.run_case(**kw)
                self.assertEqual(result['status'], 'ABORTED')
                self.assertIn('voltage stale', result['primary_error']['message'])
                self.assertEqual(len(self.sequence), writes)
                for port in PORTS:
                    self.assertEqual(self.adapters[port].holds, holds, port)
                    self.assertEqual(len(self.adapters[port].outputs), outputs, port)
                self.assertTrue(result['stop_confirmed'])
                self.assert_all_stopped(result)

    def test_enable_fault_stops_before_zero_gain(self):
        self.controls['enable_fault'] = 7
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('ID7 enable transition failed', result['primary_error']['message'])
        self.assertEqual(self.sequence, [('enable', 'port3', 1), ('zero_gain', 'port3', 1),
                                         ('enable', 'port2', 4), ('zero_gain', 'port2', 4),
                                         ('enable', 'port0', 7)])
        self.assertIs(result['motor_enable_sent'], True)
        self.assertIs(result['positive_gain_sent'], False)
        self.assert_all_stopped(result)

    def test_contract_violating_adapter_still_restores_and_reports(self):
        class Bad:
            closed = False
            def close(self):
                self.closed = True
        class StringGroup(Bad):
            group = 'port2'
        for kind in (Bad, StringGroup):
            with self.subTest(kind.__name__):
                self.setUp()
                bad, real = kind(), Harness.factory.__get__(self)
                self.factory = lambda group, bad=bad: bad if group.port == 'port2' else real(group)
                result = self.run_case()
                del self.factory
                self.assertEqual(result['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
                self.assertIs(result['physical_cutoff_required'], True)
                self.assertIs(result['motor_enable_sent'], False)
                self.assertIn(result['primary_error']['type'], ('AttributeError', 'ValueError'))
                self.assertEqual(result['stop_results']['port2']['unconfirmed_ids'], [4, 5, 6])
                self.assertTrue(bad.closed)
                for port in ('port0', 'port1'):
                    self.assertTrue(self.adapters[port].closed)
                    self.assertIs(result['restoration'][port+'_session'], True)
                for port in ('port0', 'port1', 'port2'):  # Pools shut down; scopes were never entered.
                    self.assertEqual(result['restoration'][port], {'scope_not_entered': True})
                self.assertIs(result['restoration']['port2_session'], True)
                self.assertIs(result['restoration']['main'], True)
                self.assertIn(('main-exit',), self.events)
                self.assertTrue(gc.isenabled())

    def test_supervisor_finish_failure_still_restores_and_requires_cutoff(self):
        self.stop_requested.set()
        with patch.object(runner._Supervisor, 'finish', side_effect=RuntimeError('Injected finish failure')):
            result = self.run_case('zero_gain_timing')
        self.assertEqual(result['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertIs(result['stop_confirmed'], False)
        self.assertIs(result['physical_cutoff_required'], True)
        self.assertIn('Injected finish failure', result['primary_error']['message'])
        for port in PORTS:
            self.assertTrue(self.adapters[port].closed)
            self.assertIs(result['restoration'][port], True)
        self.assertIs(result['restoration']['main'], True)
        self.assertIn(('main-exit',), self.events)

    def test_missing_capability_and_backend_rejected(self):
        with self.assertRaises(ValueError):
            runner.run(admitted(), execute=True)
        with self.assertRaises(ValueError):
            runner.run(admitted(), factory=self.factory, imu_read=self.imu, observer=Observer(),
                check_current=self.check, check_cancelled=self.check, cancel_io=self.cancel_io,
                model_setup=lambda o, **k: k, worker_scope=self.worker_scope, main_scope=self.main_scope,
                release_wait=self.release_wait, announce=lambda: None, backend_usage={'kind': 'guess'},
                execute=True)
        self.assertEqual(self.adapters, {})


class RealClockSmokeTests(Harness, unittest.TestCase):
    """Actual threads and monotonic time; host jitter may abort, never skip STOP."""
    def setUp(self):
        super().setUp()
        self.clock = time.monotonic_ns

    def release_wait(self, when):
        while True:  # Sleep, then spin the final 10 ms like the native release waiter.
            now = time.monotonic_ns()
            if now >= when:
                return now
            if when-now > 10_000_000:
                time.sleep((when-now-10_000_000)/1e9)

    def test_real_clock_short_ramp_completes_or_stops_all(self):
        self.stop_requested.set()
        result = self.run_case('zero_gain_timing')
        if result['status'] != 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING':
            self.assertEqual(result['status'], 'ABORTED')
            self.assertRegex(result['primary_error']['message'], 'gap|deadline|Deadline|20ms|Timeout|timing')
        self.assertTrue(result['stop_confirmed'])
        self.assert_all_stopped(result)


class PureHelperTests(unittest.TestCase):
    groups = tuple(Group(port, ids) for port, ids in PORT_IDS.items())

    def test_imu_limits_apply_mount_rotation_bias_and_hypothesis(self):
        check = runner.check_imu_limits
        rotation = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]  # Body z is sensor y.
        bias = [0., .4, 0.]
        sample = lambda accel, gyro=(0., .4, 0.), **kw: dict({'frame': 'sensor', 'accel_m_s2': list(accel),
                                                               'gyro_rad_s': list(gyro)}, **kw)
        tilt, raw, rate, norm = check(sample((0., 9.81, 0.)), IMU_LIMITS, rotation, bias)
        self.assertEqual((tilt, raw, rate), (0., 0., 0.))
        self.assertAlmostEqual(norm, 9.81)
        with self.assertRaisesRegex(ValueError, 'Body tilt'):
            check(sample((0., 0., 9.81)), IMU_LIMITS, rotation, bias)  # Level in sensor frame, 90 deg in body.
        self.assertAlmostEqual(check(sample((0., 9.81, 0.), (0., .85, 0.)), IMU_LIMITS, rotation, bias)[2], .45)
        with self.assertRaisesRegex(ValueError, 'Body tilt'):
            check(sample((0., 9.81, 0.), (0., -.2, 0.)), IMU_LIMITS, rotation, bias)  # Bias makes it .6.
        for bad in ((0., math.nan, 9.81), (0., 9.81), (0., math.inf, 0.)):
            with self.assertRaisesRegex(ValueError, 'Invalid IMU vector'):
                check(sample(bad), IMU_LIMITS, rotation, bias)
        with self.assertRaisesRegex(ValueError, 'Invalid IMU vector'):
            check(sample((0., 9.81, 0.), (0., math.nan, 0.)), IMU_LIMITS, rotation, bias)
        for flag in ('calibration_applied', 'orientation_applied', 'mount_rotation_applied', 'accel_scale_corrected'):
            with self.assertRaisesRegex(ValueError, 'Uncorrected'):
                check(sample((0., 9.81, 0.), **{flag: True}), IMU_LIMITS, rotation, bias)
        class Shift:
            def __init__(self, offset):
                self.offset = offset
            def correct(self, accel):
                corrected = [a-b for a, b in zip(accel, self.offset)]
                return corrected, math.hypot(*corrected)
        identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        level = sample((0., 0., 9.81), (0., 0., 0.))
        self.assertEqual(check(level, IMU_LIMITS, identity, [0.]*3, Shift((0., 0., 0.)))[:2], (0., 0.))
        with self.assertRaisesRegex(ValueError, 'Body tilt'):
            check(level, IMU_LIMITS, identity, [0.]*3, Shift((-3., 0., 0.)))  # Corrected tilt only.
        with self.assertRaisesRegex(ValueError, 'Raw body tilt exceeded'):
            check(sample((2.5, 0., 9.45), (0., 0., 0.)), IMU_LIMITS, identity, [0.]*3, Shift((2.5, 0., 0.)))
        class Refuses:
            def correct(self, accel):
                raise ValueError('Corrected acceleration norm outside hypothesis range')
        with self.assertRaisesRegex(ValueError, 'hypothesis range'):
            check(level, IMU_LIMITS, identity, [0.]*3, Refuses())

    def test_imu_frame_uses_inline_plan_candidates_only(self):
        frame_ = runner.imu_frame(model_plan())
        self.assertEqual(frame_, {'rotation': MOUNT['R_body_from_sensor'], 'gyro_bias': [0., 0., 0.],
                                  'accel_input_hypothesis': False})
        plan = model_plan()
        plan['observer_kwargs']['imu_mount_candidate'] = '/nonexistent/mount.json'
        with patch('pathlib.Path.read_bytes', side_effect=AssertionError('opened')):
            with self.assertRaisesRegex(ValueError, 'Inline IMU mount'):
                runner.imu_frame(plan)
        self.assertEqual(runner.plan(admitted())['imu_limits'], IMU_LIMITS)

    def test_genuine_transport_exposes_the_runner_contract(self):
        from .type1_transport import Type1Transport
        for name in runner.TRANSPORT_METHODS:
            self.assertTrue(callable(getattr(Type1Transport, name, None)), name)
        self.assertIsInstance(Type1Transport.last_batch, property)

    def encoded(self, kp=0.):
        return {'front': [encode_motion(mid, RAW[mid], kp, 0.) for mid in range(1, 7)],
                'rear': [encode_motion(mid, RAW[mid], kp, 0.) for mid in range(7, 13)]}

    def test_split_maps_by_destination_id_not_port_index(self):
        split = runner.split_wires_by_port(self.encoded(), self.groups, zero_gain=True)
        for port, ids in PORT_IDS.items():
            self.assertEqual([(int.from_bytes(w[2:6], 'big') >> 3) & 255 for w in split[port]], list(ids))

    def test_split_rejects_gain_in_zero_gain_mode_and_foreign_frames(self):
        with self.assertRaises(RuntimeError):
            runner.split_wires_by_port(self.encoded(kp=3.), self.groups, zero_gain=True)
        self.assertTrue(runner.split_wires_by_port(self.encoded(kp=3.), self.groups, zero_gain=False))
        swapped = self.encoded()
        swapped['front'][0], swapped['front'][1] = swapped['front'][1], swapped['front'][0]
        stop = self.encoded()
        stop['rear'][2] = stop_wire(9)
        for value in (swapped, stop, {'front': self.encoded()['front']}):
            with self.assertRaises(RuntimeError):
                runner.split_wires_by_port(value, self.groups, zero_gain=False)

    def hold(self, port, *, mode=2, wires=None):
        ids = PORT_IDS[port]
        wires = wires or tuple(encode_motion(mid, RAW[mid], 0., 0.) for mid in ids)
        records, stats = (Record*3)(), active.Stats()
        now = time.monotonic_ns()
        for record, wire, mid in zip(records, wires, ids):
            record.tx[:] = wire
            record.rx[:] = type2(mid, RAW[mid], mode=mode)
            record.start_ns = record.finish_ns = record.read_start_ns = now
            record.received_ns, record.deadline_ns = now+1, now+20_000_000
            record.written = record.received = 17
        rows = decode_records((records, stats))
        return Batch(Group(port, ids), 'hold', records, stats, rows, bytes(records), bytes(stats), now), wires

    def test_type1_hold_snapshot_requires_exact_previous_wire_and_mode2(self):
        build = runner.type1_hold_snapshot_builder(model_plan())
        holds, wires = {}, {}
        for port in PORTS:
            holds[port], wires[port] = self.hold(port)
        now = time.monotonic_ns()
        imu = {'read_started_monotonic_ns': now, 'read_finished_monotonic_ns': now,
               'accel_m_s2': [0., 0., 9.81], 'gyro_rad_s': [0., 0., 0.]}
        snapshot = build(holds, wires, imu, time.monotonic_ns()+10)
        self.assertEqual(len(snapshot['motors']), 24)
        self.assertIs(snapshot['source_flags']['type1_hold_feedback_mode2'], True)
        other = dict(wires, port1=tuple(encode_motion(mid, RAW[mid], 1., 0.) for mid in PORT_IDS['port1']))
        with self.assertRaisesRegex(ValueError, 'Exact previous'):
            build(holds, other, imu, time.monotonic_ns()+10)
        holds['port3'], _ = self.hold('port3', mode=0, wires=wires['port3'])
        with self.assertRaisesRegex(ValueError, 'mode-2'):
            build(holds, wires, imu, time.monotonic_ns()+10)

    def test_reply_rows_bind_bare_values_to_the_owner_batch(self):
        class Owner:
            last_batch = None
        owner = Owner()
        owner.last_batch, _ = self.hold('port3')
        rows = owner.last_batch.rows
        bare = {mid: rows[mid, 'feedback'][0] for mid in PORT_IDS['port3']}
        self.assertEqual(runner._feedback_rows(owner, bare, PORT_IDS['port3']), rows)
        self.assertEqual(runner._feedback_rows(object(), dict(rows), PORT_IDS['port3']), rows)
        changed = dict(bare)
        changed[1] = rows[2, 'feedback'][0]
        for transport, value in ((owner, changed), (object(), bare), (owner, dict(list(bare.items())[:2])),
                                 (owner, {**bare, 4: bare[1]})):
            with self.assertRaises(ValueError):
                runner._feedback_rows(transport, value, PORT_IDS['port3'])
        with self.assertRaises(ValueError):
            runner._single_feedback({2: bare[2]}, 1)
        self.assertIs(runner._single_feedback(rows[1, 'feedback'], 1), bare[1])

    def test_owned_refuses_new_work_after_abort_and_latches_owner_failure(self):
        class Supervisor:
            def __init__(self):
                self.aborted, self.reason, self.reasons = threading.Event(), None, []
            def emergency(self, reason):
                self.reasons.append(reason)
        supervisor, calls = Supervisor(), []
        self.assertEqual(runner._owned(supervisor, 'port1', lambda x: calls.append(x) or x, (5,), {}), 5)
        with self.assertRaisesRegex(ValueError, 'boom'):
            runner._owned(supervisor, 'port1', lambda: (_ for _ in ()).throw(ValueError('boom')), (), {})
        self.assertEqual(supervisor.reasons, ['port1 owner: ValueError: boom'])
        supervisor.aborted.set()
        supervisor.reason = 'latched'
        with self.assertRaisesRegex(RuntimeError, 'Output cancelled before owner work: latched'):
            runner._owned(supervisor, 'port2', calls.append, (6,), {})
        self.assertEqual(calls, [5])

    def test_watchdog_placement_readback(self):
        class Thread:
            native_id = 99999
        watcher = SimpleNamespace(thread=Thread())
        # The fake native id has no real Linux thread: read back a mock mask.
        with patch.object(runner.os, 'sched_getaffinity', lambda tid: {0, 1, 2, 3}, create=True):
            settings = runner._watchdog_placement(watcher, 7, 7, False)
        self.assertEqual((settings['creator_native_tid'], settings['native_tid']), (7, 99999))
        with self.assertRaisesRegex(ValueError, 'IMU owner'):
            runner._watchdog_placement(watcher, 7, 8, False)
        with self.assertRaisesRegex(ValueError, 'IMU owner'):
            runner._watchdog_placement(SimpleNamespace(thread=SimpleNamespace(native_id=threading.get_native_id())),
                                       7, 7, False)
        for mask, ok in (({0, 1, 2, 3}, True), ({0, 1, 2, 3, 4}, False), ({4}, False), ({0, 1}, False)):
            with self.subTest(mask=mask), patch.object(runner.os, 'sched_getaffinity', lambda tid: set(mask),
                                                       create=True):
                if ok:
                    self.assertEqual(runner._watchdog_placement(watcher, 7, 7, True)['cpu_mask'], [0, 1, 2, 3])
                else:
                    with self.assertRaisesRegex(ValueError, 'CPU0..3 placement'):
                        runner._watchdog_placement(watcher, 7, 7, True)
        if not hasattr(runner.os, 'sched_getaffinity'):
            with self.assertRaisesRegex(ValueError, 'CPU0..3 placement'):
                runner._watchdog_placement(watcher, 7, 7, True)

    def test_stop_complete_requires_all_confirmed_no_ambiguity_no_fault(self):
        good = {'complete': True, 'confirmed_ids': [1, 2, 3], 'unconfirmed_ids': [], 'ambiguous_ids': [],
                'faults': {'1': 0}, 'physical_cutoff_required': False}
        self.assertTrue(runner._stop_complete(good, (1, 2, 3)))
        for change in ({'ambiguous_ids': [2]}, {'faults': {'2': 4}}, {'confirmed_ids': [1, 2]},
                       {'complete': 1}, {'physical_cutoff_required': None}, {'faults': None}):
            self.assertFalse(runner._stop_complete({**good, **change}, (1, 2, 3)), change)


if __name__ == '__main__':
    unittest.main()
