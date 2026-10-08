"""File-only foreground wiring: pinned admission -> genuine Type1Transport -> runner -> report.

Synthetic RS05 peers sit behind the genuine subset-exchange contract; no
Torch, native library, device, signal to other processes or network.
"""
import contextlib
import copy
import gc
import hashlib
import io
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import stat
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import wave

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import motor_version_probe as versions
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.native_diagnostic_transport import stop_wire
from experiments.four_bus_diagnostic import model_bridge
from experiments.four_bus_diagnostic import test_model_bridge as mb
from experiments.four_bus_diagnostic.transport_adapter import Group, PORTS
from .test_type1_profile import geometry_for, write, write_capture
from .test_type1_runner import VirtualClock, type2, type17
from .test_type1_transport import SEAL, MockSession
from . import build, type1_foreground as F, type1_profile as P, type1_runner as R, type1_transport as T

IDS = tuple(range(1, 13))
FIRMWARE = {mid: 'a1b2c3%02x' % mid for mid in IDS}
BOOT, EPOCH = 'current-boot', 'explicit-current-label'
NAMES = {codec.PARAMETERS[name][0]: name for name in codec.PARAMETERS}
CONDITIONS = dict(motor_40v_on=True, box_supports_body=True, four_feet_touch_floor=True,
    all12_local_plus_minus3deg_clear=True, hands_off=True, immediate_40v_cutoff=True,
    other_drive_tools_stopped=True, box_will_remain=True, box_removal_allowed=False,
    load_transfer_allowed=False, standing_allowed=False, walking_allowed=False)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def put(path, raw, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw); path.chmod(mode)
    return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}


class World:
    """Pinned source kit, Type1/guard receipts, lineage plan, reviewed geometry and captures."""
    def __init__(self, root):
        self.root = root
        self.kit = root/'kit'
        members = {'runtime/experiments/native_active_transport/transport.cpp': b'// ordinary\n',
                   'runtime/experiments/four_bus_diagnostic/subset_stop.cpp': b'// subset stop\n',
                   'runtime/experiments/four_bus_type1/subset_active.cpp': b'// subset active\n',
                   'runtime/experiments/four_bus_diagnostic/native_current_guard.cpp': b'// guard\n'}
        files = {}
        for name, raw in members.items():
            put(self.kit/name, raw)
            files[name] = {'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw), 'mode': 0o644}
        self.manifest = write(root/'kit-manifest.json', {'file_count': len(files), 'files': files,
                                                         'output_allowed': False, 'approved_for_runtime': False})
        self.sources = {key: files[name]['sha256'] for key, name in (
            ('ordinary', 'runtime/experiments/native_active_transport/transport.cpp'),
            ('subset_stop', 'runtime/experiments/four_bus_diagnostic/subset_stop.cpp'),
            ('extension', 'runtime/experiments/four_bus_type1/subset_active.cpp'),
            ('guard', 'runtime/experiments/four_bus_diagnostic/native_current_guard.cpp'))}
        self.type1_library = self.receipt(root/'t1lib', members)
        guard = root/'guard'
        self.guard_library = put(guard/'libguard.so', b'synthetic guard binary\n')
        put(guard/'native_current_guard.cpp', members['runtime/experiments/four_bus_diagnostic/native_current_guard.cpp'])
        self.guard_build = write(guard/'build-record.json', {'schema': 'singularitydog.four-bus-current-guard-build.v1',
            'abi': 1, 'binary_sha256': self.guard_library['sha256'], 'source_sha256': self.sources['guard'],
            'source_bytes': len(b'// guard\n'), 'CAN_IO_available': False, 'output_allowed': False,
            'timing_admission_eligible': False})
        self.audio = root/'announce.wav'
        with wave.open(str(self.audio), 'wb') as clip:
            clip.setnchannels(1); clip.setsampwidth(2); clip.setframerate(8000); clip.writeframes(bytes(1600))
        fixture = mb.Fixture()
        self.raws, self.uids = fixture.capture.raws, fixture.capture.uids
        self.lineage, self.lineage_events, _ = write_capture(root, 'lineage', fixture.capture, fixture.topology)
        refs = {name: write(root/(name+'.json'), getattr(fixture, name)) for name in ('mount', 'bias')}
        refs['calibration'] = write(root/'calibration.json', fixture.nominal)
        fixture.profile['artifacts'] = dict(refs)
        refs['model_profile'] = write(root/'model-profile.json', fixture.profile)
        fixture.source['original_model_profile'] = refs['model_profile']
        fixture.source['four_bus_source_manifest'] = self.manifest
        refs['source_binding'] = write(root/'source-binding.json', fixture.source)
        self.refs = refs
        self.plan = model_bridge.prepare_model_plan(**self.envelopes(self.lineage, self.lineage_events))
        review = write(root/'hardware-review.json', {'device_watchdog': {str(mid): {
            'motor_model': 'RS05', 'actual_command_loss_test_passed': True, 'usb_disconnect_test_passed': False,
            'disabled_after_loss_verified': True, 'configured_timeout_ms': 200, 'max_observed_disable_ms': 210,
            'version_bytes_hex': FIRMWARE[mid]} for mid in IDS}})
        self.geometry_doc = geometry_for(self.plan)
        self.geometry_doc['artifacts']['hardware_review'] = review
        self.geometry = write(root/'geometry.json', self.geometry_doc)
        lineage = {'topology': self.lineage, 'events': self.lineage_events,
                   **{name: refs[name] for name in ('calibration', 'model_profile', 'source_binding', 'mount', 'bias')}}
        self.contract = P.build_contract(copy.deepcopy(self.plan), lineage, copy.deepcopy(self.geometry_doc),
                                         self.geometry)
        self.digest = P.contract_sha256(self.contract)

    def receipt(self, directory, members):
        copies = {'ordinary_transport.cpp': members['runtime/experiments/native_active_transport/transport.cpp'],
                  'subset_stop.cpp': members['runtime/experiments/four_bus_diagnostic/subset_stop.cpp'],
                  'transport.cpp': members['runtime/experiments/four_bus_type1/subset_active.cpp']}
        for name, raw in copies.items():
            put(directory/name, raw)
        library = put(directory/build.LIBRARY_NAME, b'synthetic subset-active binary\n')
        digest = lambda name: hashlib.sha256(copies[name]).hexdigest()
        record = {'abi': 1, 'command': ['c++'], 'platform': 'synthetic', 'machine': 'synthetic',
            'source_sha256': digest('transport.cpp'), 'binary_sha256': library['sha256'],
            'four_bus_subset_active': {'schema': build.SCHEMA, 'scope': build.SCOPE, 'type1_exchange_abi': 1,
                'subset_stop_abi': 1, 'original_active_abi': 1,
                'ordinary_source_sha256': digest('ordinary_transport.cpp'),
                'subset_stop_source_sha256': digest('subset_stop.cpp'),
                'extension_source_sha256': digest('transport.cpp'),
                'ordinary_source_bytes': len(copies['ordinary_transport.cpp']),
                'subset_stop_source_bytes': len(copies['subset_stop.cpp']),
                'extension_source_bytes': len(copies['transport.cpp']), 'sources': {}, 'library': build.LIBRARY_NAME,
                'allowed_masks': build.ALLOWED_MASKS, 'allowed_kinds': build.ALLOWED_KINDS,
                'type1_batch_sizes': [1, 3], 'output_allowed': False, 'timing_admission_eligible': False}}
        self.type1_build = write(directory/'build-record.json', record)
        return library

    def envelopes(self, capture, events):
        load = lambda ref: {'reference': ref, 'raw_json': Path(ref['path']).read_text()}
        result = {name: load(self.refs[name]) for name in ('calibration', 'model_profile', 'source_binding',
                                                            'mount', 'bias')}
        result['topology'] = load(capture)
        result['events'] = {'reference': events, 'raw_jsonl': Path(events['path']).read_text()}
        return result

    def capture(self, name, clock_ns):
        value = mb.Capture()
        value.clock_ns = clock_ns
        ref, events, document = write_capture(self.root, name, value)
        current = model_bridge.prepare_model_plan(**self.envelopes(ref, events))
        return {'topology': ref, 'events': events, 'started_monotonic_ns': document['started_monotonic_ns'],
                'finished_monotonic_ns': document['finished_monotonic_ns'],
                'model_rad_by_id': dict(current['provenance']['model_rad_by_id']),
                'fixed_offset_rad_by_id': {k: v['fixed_offset_rad'] for k, v in current['axes'].items()},
                'boot_id': BOOT, 'motor_power_epoch': EPOCH}

    def admission(self, name, mode, duration, capture, *, predecessor=None, contract_sha256=None,
                  modes=('zero_gain_timing', 'learned_boxed')):
        diag = None
        if P.required_evidence(mode, duration)[1]:
            diag = {'report': {'path': str(self.root/'diag.json'), 'sha256': 'e'*64},
                    'pinned_capture': capture['topology'], 'completed_cycles': 501,
                    'first_release_monotonic_ns': 60_000_000_000, 'workers_joined_monotonic_ns': 71_000_000_000}
        evidence = {'current_capture': capture, 'stop_proxy_diagnostic': diag, 'predecessor': predecessor}
        profile = P.assemble_profile(copy.deepcopy(self.contract), mode, duration, evidence,
                                     prepared_at='2026-10-08T23:00:00+09:00')
        conditions = P.conditions_record(user_statement='SYNTHETIC direct statement', user_reply_id=None,
            boot_id=BOOT, motor_power_epoch=EPOCH, contract_sha256=contract_sha256 or profile['contract_sha256'],
            authorized_modes=list(modes), authorized_durations_s=[2, 10],
            record_written_at='2026-10-08T23:01:00+09:00', **CONDITIONS)
        return write(self.root/(name+'-profile.json'), profile), write(self.root/(name+'-conditions.json'), conditions)

    def argv(self, profile, conditions, capture, mode, duration, output, *extra):
        values = []
        for flag, ref in (('source-manifest', self.manifest), ('source-binding', self.refs['source_binding']),
                          ('original-model-profile', self.refs['model_profile']), ('profile', profile),
                          ('conditions', conditions), ('topology', capture['topology']),
                          ('events', capture['events']), ('type1-library', self.type1_library),
                          ('current-guard-library', self.guard_library),
                          ('announcement-audio', {'path': str(self.audio), 'sha256': sha(self.audio)})):
            values += ['--'+flag, ref['path'], '--'+flag+'-sha256', ref['sha256']]
        return values+['--source-root', str(self.kit), '--source-count', '4',
            '--type1-build-sha256', self.type1_build['sha256'], '--ordinary-source-sha256', self.sources['ordinary'],
            '--subset-stop-source-sha256', self.sources['subset_stop'],
            '--type1-extension-source-sha256', self.sources['extension'],
            '--current-guard-build-sha256', self.guard_build['sha256'],
            '--current-guard-source-sha256', self.sources['guard'], '--mode', mode, '--duration', str(duration),
            '--power-epoch', EPOCH, '--audio-device', 'plughw:SYNTHETIC', '--output', str(output), *extra]


class Motors:
    """Synthetic RS05 peers behind the genuine sda_subset_exchange/STOP contract."""
    def __init__(self, world, clock):
        self.clock, self.uids = clock, world.uids
        self.raw = {mid: world.raws[mid] for mid in IDS}
        self.firmware = dict(FIRMWARE)
        self.enabled, self.calls, self.forbidden, self.stops = set(), [], [], []
        self.type1 = {mid: [] for mid in IDS}
        self.mode0 = self.stop_ambiguous = None
        self.lock = threading.Lock()
        self.refused_after_cancel, self.cancel_seen_by_stop = [], []

    def cancelled(self, handle):
        """The session's own cancel FD readable, as the native exchange polls it before each write."""
        fds = {session.settings['cancel_fd'] for session in MockSession.created if session.first_id == handle}
        return bool(fds) and any(select.select([fd], [], [], 0)[0] for fd in fds)

    def reply(self, wire):
        value = ATParser().feed(wire)[0]
        mid, kind = value.destination, value.kind
        if kind == 0:
            return mb.wire((mid << 8) | 0xfe, bytes.fromhex(self.uids[str(mid)]))
        if kind == 17:
            name = NAMES[int.from_bytes(value.data[:2], 'little')]
            values = {'run_mode': 0, 'voltage': 40., 'can_timeout': 4000, 'position': self.raw[mid], 'velocity': 0.}
            return type17(mid, name, values[name])
        if kind == 4 and value.data[:2] == b'\x00\xc4':
            return mb.wire((2 << 24) | (mid << 8) | 0xfd, versions.VERSION_PREFIX+bytes.fromhex(self.firmware[mid])+b'\0')
        if kind == 3:
            self.enabled.add(mid)
            return type2(mid, self.raw[mid], mode=2)
        if kind == 1:
            fields = T.type1_fields(wire)
            assert fields is not None and mid in self.enabled, 'Type1 to a disabled synthetic motor'
            self.type1[mid].append(wire)
            if fields[2]:
                self.raw[mid] = T.decoded_q(fields[1])  # Ideal stiff tracking with gain.
            mode = 0 if self.mode0 is not None and mid == self.mode0[0] and len(self.type1[mid]) > self.mode0[1] else 2
            return type2(mid, self.raw[mid], mode=mode)
        if kind == 4:
            self.enabled.discard(mid)
        return type2(mid, self.raw[mid], mode=0)

    def sda_subset_exchange(self, handle, mask, raw, count, send_only, deadline, records, stats, error, size):
        wires = [bytes(raw)[i*17:(i+1)*17] for i in range(count)]
        with self.lock:
            self.calls.append((handle, mask, wires))
        stats = stats._obj
        stats.begin_ns = self.clock()
        for record, wire in zip(records, wires):
            mid = (int.from_bytes(wire[2:6], 'big') >> 3) & 255
            if not 0 <= mid-handle < 6 or not mask & (1 << (mid-handle)):
                error.value = b'Synthetic native mask rejection'
                return -1
            if self.cancelled(handle):
                with self.lock:
                    self.refused_after_cancel.append((handle, mask, mid))
                error.value = b'Synthetic native cancelled'
                return -1
            record.tx[:] = wire; record.deadline_ns = deadline
            record.start_ns = self.clock(); record.finish_ns = self.clock(); record.read_start_ns = self.clock()
            record.written = 17
            response = self.reply(wire)
            received = self.clock()
            if received >= deadline:
                error.value = b'Synthetic late reply'
                return -1
            record.rx[:] = response; record.received_ns = received; record.received = 17
        stats.end_ns = self.clock(); stats.writes = stats.reads = count; stats.bytes = 17*count
        return 0

    def sda_subset_validate(self, handle, mask, raw, count, error, size):
        return 0

    def sda_emergency_stop_subset(self, handle, mask, deadline, records, stats, result, error, size):
        result, stats = result._obj, stats._obj
        with self.lock:
            self.stops.append((handle, mask))
            self.cancel_seen_by_stop.append(self.cancelled(handle))
        stats.begin_ns = self.clock()
        result.attempted_mask = mask
        confirmed = 0
        for slot in range(6):
            if mask & (1 << slot):
                mid = handle+slot
                record = records[slot]
                record.tx[:] = stop_wire(mid); record.written = 17
                record.start_ns = record.finish_ns = record.read_start_ns = self.clock()
                if mid == self.stop_ambiguous:
                    result.ambiguous_mask |= 1 << slot
                    continue
                self.enabled.discard(mid)
                record.rx[:] = type2(mid, self.raw[mid], mode=0); record.received = 17
                record.received_ns = self.clock(); record.deadline_ns = deadline
                result.fault[slot] = 0
                confirmed |= 1 << slot
        result.confirmed_mask = confirmed
        stats.end_ns = self.clock()
        error.value = b'synthetic'
        return 0 if confirmed == mask else 1

    def sda_exchange(self, *args):
        self.forbidden.append('sda_exchange'); raise AssertionError('ordinary sda_exchange called')

    def sda_emergency_stop(self, *args):
        self.forbidden.append('sda_emergency_stop'); raise AssertionError('fixed-six STOP called')


class Observer:
    def __init__(self, targets, hook=None):
        self.targets, self.hook, self.calls, self.finished = targets, hook, 0, False

    def consume(self, snapshot):
        if len(snapshot['motors']) != 24 or snapshot['output_allowed'] is not False:
            raise AssertionError('Ordinary observer snapshot contract')
        self.calls += 1
        if self.hook is not None:
            self.hook(self.calls)
        return {'status': 'TICK_OBSERVED_NO_OUTPUT', 'tick_index': self.calls-1, 'output_allowed': False,
                'q_target_rad_diagnostic_only': [self.targets[str(mid)] for mid in shadow.CAN_ORDER]}

    def finish(self):
        self.finished = True
        return {'status': 'SYNTHETIC_OBSERVER', 'ticks_completed': self.calls}


class MainScope:
    def __init__(self):
        self.report = {}

    def __enter__(self):
        self.enabled = gc.isenabled()
        gc.disable()
        return {'file_only_mock_readback': True, 'gc_deferred_during_cycles': True}

    def __exit__(self, *args):
        if self.enabled:
            gc.enable()
        self.report = {'restored': gc.isenabled() == self.enabled}
        return False


class Worker:
    slack = original = None

    def __init__(self, port, mask):
        self.mask = list(mask)

    def __enter__(self):
        return {'native_tid': threading.get_native_id(), 'cpu_mask': self.mask, 'timer_slack_ns': 1000,
                'file_only_mock_readback': True}

    def __exit__(self, *args):
        return False


class Guard:
    def __init__(self, stop):
        self.stop, self.setup, self.calls, self.closed = stop, {'file_only_mock': True}, [], False

    def __call__(self):
        F.need(not self.stop.is_set(), 'Foreground cancelled')

    def finish(self):
        return {'setup': self.setup, 'end_full_path_resolution_verified': True, 'calls': []}

    def close(self):
        self.closed = True


class IMU:
    restore_status = None

    def __init__(self, clock):
        self.clock = clock

    def start(self):
        return {'file_only_mock': True}

    def read_sample(self):
        start = self.clock()
        return {'read_started_monotonic_ns': start, 'read_finished_monotonic_ns': self.clock(), 'frame': 'sensor',
                'accel_m_s2': [0., 0., 9.81], 'gyro_rad_s': [0., 0., 0.]}

    def close(self):
        self.restore_status = 'not_needed'


class Environment:
    """Injected file-only capabilities for execute(); truthful mock backend kind."""
    kind = 'injected_file_only_mock'

    def __init__(self, motors, clock, observer):
        self.motors, self.clock, self.observer = motors, clock, observer
        self.events, self.cancel_fd, self.guard = [], None, None

    def startup(self):
        return {'file_only_mock_readback': True}

    def boot_id(self):
        return BOOT

    def bind(self, capture):
        return copy.deepcopy(capture['ports'])

    def holders(self, bindings):
        return {'observed_holders': [], 'file_only_mock_readback': True}

    def verify_sources(self, args, prepared, *, final):
        self.events.append(('verify_sources', final))

    def load_model(self, prepared):
        self.events.append('load_model')
        return {'observer': self.observer, 'model_source': {'file_only_mock_model': True},
                'model_setup': lambda run, **k: dict(k, reset_verified=True, file_only_mock_model=True)}

    def load_libraries(self, args):
        return self.motors, object(), None

    def locks(self, stack, bindings):
        self.events.append('locks')

    def open_ports(self, stack, bindings, check):
        check()
        return ({port: 100+i for i, port in enumerate(PORTS)}, {port: 200+i for i, port in enumerate(PORTS)})

    def current_guard(self, library, bindings, descriptors, boot_fd, boot_id, stop):
        self.guard = Guard(stop)
        return self.guard

    def imu(self):
        return IMU(self.clock)

    def main_scope(self):
        return MainScope()

    def worker_scope(self, port, mask):
        return Worker(port, mask)

    def release_wait(self, library, cancel_fd, observer):
        self.cancel_fd = cancel_fd
        return lambda when: self.clock.advance_to(when)

    def announcer(self, audio, check):
        return lambda: self.events.append(('announce', audio['sha256']))


@contextlib.contextmanager
def synthetic_native(clock):
    """Genuine Type1Transport class over the synthetic library and session (no dlopen)."""
    MockSession.created = []
    with patch.object(T.active, 'ActiveSession', MockSession), \
            patch.object(T, 'verify_library', lambda library: SEAL), \
            patch.object(T.active, 'verified_active_session_creation', lambda session: ('c', 'b')), \
            patch.object(T, 'time', SimpleNamespace(monotonic_ns=clock)):
        yield


@contextlib.contextmanager
def file_only_prepare():
    """Only the kit-origin and real model-dependency loaders are replaced."""
    dependencies = {'profile': {'h_hypothesis': 0.}, 'fk_plan': {'file_only': True},
                    'checked_plan': {'file_only': True}}
    with patch.object(F, 'verify_origins'), \
            patch.object(F, 'model_dependencies', return_value=dependencies) as loaded:
        yield loaded


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(tempfile.mkdtemp()).resolve()
        cls.world = World(cls.root)
        cls.current = cls.world.capture('current', 50_000_000_000)
        cls.zero = cls.world.admission('zero', 'zero_gain_timing', 2, cls.current)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root)

    def setUp(self):
        self.handlers = {sig: signal.getsignal(sig) for sig in
                         (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1)}

    def tearDown(self):
        for sig, handler in self.handlers.items():
            self.assertEqual(signal.getsignal(sig), handler, 'Signal handler not restored')

    def output(self):
        directory = Path(tempfile.mkdtemp()).resolve()
        self.addCleanup(shutil.rmtree, directory)
        return directory/'run'

    def targets(self, offset=.005):
        return {key: value+offset for key, value in self.world.plan['provenance']['model_rad_by_id'].items()}

    def run_mode(self, admission, capture, mode, *, hook=None, mutate=None, duration=2):
        argv = self.world.argv(*admission, capture, mode, duration, self.output())
        args = F.parser().parse_args(argv+['--execute'])
        with file_only_prepare():
            prepared = F.prepare(args)
        clock = VirtualClock()
        motors = Motors(self.world, clock)
        if mutate is not None:
            mutate(motors)
        observer = Observer(self.targets(), hook)
        environment = Environment(motors, clock, observer)
        with synthetic_native(clock):
            report = F.execute(args, prepared, environment)
        written = prepared['output']/'report.json'
        self.assertEqual(stat.S_IMODE(written.stat().st_mode), 0o600)
        self.assertEqual(json.loads(written.read_text()), json.loads(json.dumps(report)))
        self.assertEqual(motors.forbidden, [])
        return SimpleNamespace(report=report, motors=motors, observer=observer, environment=environment,
                               path=written, prepared=prepared)

    def assert_all_ports_stopped(self, result):
        first = {port: min(ids) for port, ids in self.world.contract['topology_by_port'].items()}
        expected = {(1 if first[p] < 7 else 7, 0x07 if first[p] in (1, 7) else 0x38) for p in PORTS}
        self.assertEqual(set(result.motors.stops), expected)
        self.assertTrue(all(session.closed for session in MockSession.created))
        self.assertEqual(len(MockSession.created), 4)


class PlanTests(Base):
    def test_plan_opens_nothing_and_binds_admission(self):
        output = self.output()
        argv = self.world.argv(*self.zero, self.current, 'zero_gain_timing', 2, output)
        with file_only_prepare(), patch.object(F, 'execute') as execute, \
                patch.object(F, 'LinuxEnvironment') as environment, \
                patch.object(T.Type1Transport, 'create') as create, \
                contextlib.redirect_stdout(io.StringIO()) as printed:
            self.assertEqual(F.main(argv), 0)
        execute.assert_not_called(); environment.assert_not_called(); create.assert_not_called()
        plan = json.loads(printed.getvalue())
        self.assertFalse(output.exists())
        self.assertEqual((plan['status'], plan['opens_devices'], plan['loads_library_model_or_torch']),
                         ('PLAN', False, False))
        for key in (*F.ATTEMPTS, 'output_approval_granted_here', 'live_type1_qualified', 'approved_for_runtime'):
            self.assertIs(plan[key], False, key)
        self.assertEqual(plan['firmware_by_id'], {str(mid): FIRMWARE[mid] for mid in IDS})
        self.assertEqual(plan['contract_sha256'], self.world.digest)
        self.assertEqual(plan['pacing'], P.PACING)
        self.assertEqual(plan['runner_plan']['pacing'], P.PACING)
        self.assertEqual(plan['observer_max_ticks'], 102)
        self.assertEqual(plan['transport_native_gain_caps'], 'zero')
        self.assertEqual([row['motor_id'] for row in plan['runner_plan']['enable_order']],
                         self.world.contract['enable_order_ids'])
        self.assertFalse(plan['type1_library_plan']['loads_library_in_plan'])
        self.assertFalse(plan['current_guard_plan']['loads_library_in_plan'])

    def args(self, *extra, admission=None, capture=None, mode='zero_gain_timing', duration=2):
        argv = self.world.argv(*(admission or self.zero), capture or self.current, mode, duration, self.output())
        return F.parser().parse_args(argv+list(extra))

    def test_prepare_rejections_before_any_capability(self):
        world = self.world
        lineage = {'topology': world.lineage, 'events': world.lineage_events}
        other = world.admission('other', 'zero_gain_timing', 2, self.current, contract_sha256='0'*64)
        cases = {
            'mode': (self.args(mode='learned_boxed'), 'mode/--duration'),
            'duration': (self.args(duration=10), 'mode/--duration'),
            'conditions contract': (self.args(admission=other), 'different contract'),
            'not the admitted capture': (self.args(capture=lineage), 'admitted current capture'),
            'power epoch': (self.args('--power-epoch', 'other-epoch'), 'power epoch'),
        }
        changed = self.args()
        changed.type1_library_sha256 = '0'*64
        cases['library pin'] = (changed, 'SHA|differs|changed')
        changed = self.args()
        changed.subset_stop_source_sha256 = changed.ordinary_source_sha256
        cases['source pin'] = (changed, 'SHA|differs|changed|source')
        for name, (args, pattern) in cases.items():
            with self.subTest(name), file_only_prepare(), self.assertRaisesRegex(Exception, pattern):
                F.prepare(args)

    def test_firmware_comes_only_from_pinned_reviewed_geometry(self):
        pins = F.FG.Pins()
        self.assertEqual(F.reviewed_firmware(self.world.contract, pins), {str(m): FIRMWARE[m] for m in IDS})
        self.assertIn(self.world.geometry['path'], pins.values)
        contract = copy.deepcopy(self.world.contract)
        contract['uids_by_id']['4'] = 'ff'*8
        with self.assertRaisesRegex(ValueError, 'geometry differs'):
            F.reviewed_firmware(contract, F.FG.Pins())

    def test_runner_mapping_and_duration_observer_budget(self):
        profile = json.loads(Path(self.zero[0]['path']).read_text())
        conditions = json.loads(Path(self.zero[1]['path']).read_text())
        admitted = P.admit(profile, conditions)
        spec = F.runner_admitted(admitted, {str(m): FIRMWARE[m] for m in IDS})
        checked = R.validate_admitted(spec)
        self.assertEqual(spec['pacing'], R.PACING)
        self.assertEqual(spec['post_reply_policy'], R.POST_REPLY_V1)
        self.assertIs(spec['first_cycle_post_reply'], False)
        self.assertEqual(checked['offsets'], {m: self.world.contract['axes'][str(m)]['fixed_offset_rad'] for m in IDS})
        with self.assertRaisesRegex(ValueError, 'firmware'):
            F.runner_admitted(admitted, {})
        with self.assertRaisesRegex(ValueError, 'Genuine'):
            F.runner_admitted(dict(spec), {str(m): FIRMWARE[m] for m in IDS})
        group = Group('port0', tuple(self.world.contract['topology_by_port']['port0']))
        bounds, kp, kd = F.transport_limits(self.world.contract, group, 'zero_gain_timing')
        self.assertEqual((set(kp.values()), set(kd.values())), ({0.}, {0.}))
        self.assertEqual(F.transport_limits(self.world.contract, group, 'learned_boxed')[1], dict.fromkeys(group.ids, 3.))
        for mid, (lo, hi) in bounds.items():
            self.assertEqual((lo, hi), (self.world.contract['axes'][str(mid)]['raw_lower_rad'],
                                        self.world.contract['axes'][str(mid)]['raw_upper_rad']))
        self.assertEqual([F.observer_ticks(d) for d in (2, 10, 20)], [102, 502, 1002])
        made = []
        def factory(policy, calibration, **kwargs):
            made.append(kwargs['max_ticks'])
            return SimpleNamespace(consume=lambda snapshot: None)
        observer = F.create_type1_observer(self.world.plan, factory, policy=object(), duration_s=20,
                                           torch_module=None, checked_dispatch_wrapper=object())
        self.assertEqual(made, [1002])
        self.assertIsInstance(observer, model_bridge._GuardedObserver)
        with self.assertRaisesRegex(ValueError, 'Duration'):
            F.observer_ticks(30)
        with self.assertRaisesRegex(ValueError, '5/501'):
            model_bridge.create_guarded_observer(self.world.plan, factory, policy=object(), max_ticks=1002,
                                                 checked_dispatch_wrapper=object())

    def test_terminal_stop_summary_is_strict(self):
        groups = tuple(Group(port, tuple(ids)) for port, ids in self.world.contract['topology_by_port'].items())
        complete = {port: {'complete': True, 'confirmed_ids': list(g.ids), 'ambiguous_ids': [],
                           'faults': {str(m): 0 for m in g.ids}} for port, g in zip(PORTS, groups)}
        value = F.terminal_stop({'stop_results': complete, 'stop_confirmed': True,
                                 'all_workers_joined_monotonic_ns': 7}, groups)
        self.assertEqual((value['stop_confirmed'], value['confirmed_ids'], value['finished_monotonic_ns']),
                         (True, list(IDS), 7))
        partial = copy.deepcopy(complete)
        partial['port1'] = {'complete': False, 'error': 'TimeoutError'}
        value = F.terminal_stop({'stop_results': partial, 'stop_confirmed': True}, groups)
        self.assertFalse(value['stop_confirmed']); self.assertTrue(value['physical_cutoff_required'])
        self.assertEqual(value['unconfirmed_ids'], sorted(groups[1].ids))
        self.assertIsNone(F.terminal_stop({'stop_results': {}, 'stop_confirmed': None}, groups)['stop_confirmed'])


class ExecuteTests(Base):
    def test_zero_gain_then_learned_two_second_chain(self):
        zero = self.run_mode(self.zero, self.current, 'zero_gain_timing')
        report = zero.report
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', report['errors'])
        self.assertEqual((report['motor_enable_sent'], report['type1_sent'], report['positive_gain_sent'],
                          report['learned_targets_attempted']), (True, True, False, False))
        self.assertTrue(report['all_cycles_passed'] and report['restoration_complete'])
        self.assertFalse(report['physical_cutoff_required'] or report['failure_retained'])
        self.assertEqual(report['terminal_stop']['confirmed_ids'], list(IDS))
        self.assertEqual(report['backend_kind'], 'injected_file_only_mock')
        self.assertTrue(all(T.type1_fields(w)[2:] == (0, 0) for wires in zero.motors.type1.values() for w in wires))
        for session in MockSession.created:
            self.assertEqual(set(session._limits.kp)|set(session._limits.kd), {0.})
        self.assertGreater(zero.observer.calls, 50)
        self.assertTrue(zero.observer.finished)
        self.assertIn(('announce', sha(self.world.audio)), zero.environment.events)
        self.assertEqual(zero.environment.events[-1], ('verify_sources', True))
        self.assert_all_ports_stopped(zero)
        self.assertTrue(zero.environment.guard.closed)
        for key in ('output_approval_granted_here', 'live_type1_qualified', 'approved_for_runtime'):
            self.assertIs(report[key], False)
        summary = P.validate_type1_report(json.loads(zero.path.read_text()), contract=self.world.contract,
                                          contract_digest=self.world.digest, mode='zero_gain_timing', duration_s=2)
        # A fresh read-only capture after the predecessor terminal STOP, then learned 2 s.
        later = self.world.capture('after-zero', summary['terminal_stop_finished_monotonic_ns']+1_000_000_000)
        predecessor = {'report': {'path': str(zero.path), 'sha256': sha(zero.path)}, **summary}
        learned = self.run_mode(self.world.admission('learned2', 'learned_boxed', 2, later, predecessor=predecessor),
                                later, 'learned_boxed')
        report = learned.report
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        self.assertEqual((report['motor_enable_sent'], report['type1_sent'], report['positive_gain_sent'],
                          report['learned_targets_attempted']), (True, True, True, True))
        self.assertTrue(any(T.type1_fields(w)[2] for wires in learned.motors.type1.values() for w in wires))
        self.assert_all_ports_stopped(learned)
        P.validate_type1_report(json.loads(learned.path.read_text()), contract=self.world.contract,
                                contract_digest=self.world.digest, mode='learned_boxed', duration_s=2)

    def test_mode0_type1_reply_cancels_siblings_and_stops_all_ports(self):
        mid = self.world.contract['topology_by_port']['port2'][1]
        result = self.run_mode(self.zero, self.current, 'zero_gain_timing',
                               mutate=lambda motors: setattr(motors, 'mode0', (mid, 10)))
        report = result.report
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(report['failure_retained'])
        self.assertTrue(report['transport_failures']['port2'])
        self.assertEqual(report['cancel_requests'][0]['source'], 'port2_transport_failure')
        self.assertTrue(report['terminal_stop']['stop_confirmed'])
        self.assertFalse(report['physical_cutoff_required'])
        self.assertLess(report['completed_cycles'], 20)
        self.assert_all_ports_stopped(result)
        # The cancel byte reached every session's own FD, and no motor got a Type1 after the failing one.
        self.assertEqual(result.motors.cancel_seen_by_stop, [True]*4)
        failing = len(result.motors.type1[mid])
        self.assertEqual(failing, 11)
        self.assertTrue(all(len(wires) <= failing for wires in result.motors.type1.values()), result.motors.type1)
        with self.assertRaises(ValueError):
            P.validate_type1_report(json.loads(result.path.read_text()), contract=self.world.contract,
                                    contract_digest=self.world.digest, mode='zero_gain_timing')

    def test_ambiguous_stop_requires_power_cutoff(self):
        result = self.run_mode(self.zero, self.current, 'zero_gain_timing',
                               mutate=lambda motors: setattr(motors, 'stop_ambiguous', 5))
        report = result.report
        self.assertEqual(report['status'], P.STOP_UNCONFIRMED_STATUS)
        self.assertTrue(report['physical_cutoff_required'])
        self.assertEqual(report['terminal_stop']['ambiguous_ids'], [5])
        self.assertNotIn(5, report['terminal_stop']['confirmed_ids'])
        with self.assertRaises(ValueError):
            P.validate_type1_report(json.loads(result.path.read_text()), contract=self.world.contract,
                                    contract_digest=self.world.digest, mode='zero_gain_timing')

    def test_sigusr1_starts_graceful_ramp_and_restores_handler(self):
        def hook(calls):
            if calls == 3:
                os.kill(os.getpid(), signal.SIGUSR1)
        result = self.run_mode(self.zero, self.current, 'zero_gain_timing', hook=hook)
        report = result.report
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', report['errors'])
        self.assertTrue(report['graceful_stop_requested'])
        self.assertLess(report['completed_cycles'], 60)
        self.assertLessEqual(result.observer.calls, 4)
        self.assert_all_ports_stopped(result)

    def test_sigterm_cancels_writes_and_stops_every_port(self):
        def hook(calls):
            if calls == 3:
                os.kill(os.getpid(), signal.SIGTERM)
        result = self.run_mode(self.zero, self.current, 'zero_gain_timing', hook=hook)
        report = result.report
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cancel_requests'][0]['source'], 'signal_SIGTERM')
        self.assertIn('cancelled', ' '.join(report['errors']))
        self.assertTrue(report['terminal_stop']['stop_confirmed'])
        self.assertLessEqual(report['completed_cycles'], 4)
        self.assert_all_ports_stopped(result)

    def test_firmware_mismatch_rejects_before_enable(self):
        mid = self.world.contract['topology_by_port']['port1'][0]
        result = self.run_mode(self.zero, self.current, 'zero_gain_timing',
                               mutate=lambda motors: motors.firmware.update({mid: '00000000'}))
        report = result.report
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIs(report['motor_enable_sent'], False)
        self.assertIs(report['type1_sent'], False)
        self.assertEqual(result.motors.enabled, set())
        self.assertFalse(any(result.motors.type1.values()))
        sent = [(int.from_bytes(w[2:6], 'big') >> 27) for _, _, wires in result.motors.calls for w in wires]
        self.assertNotIn(3, sent); self.assertNotIn(1, sent)
        self.assertIn('firmware', ' '.join(report['errors']))
        self.assert_all_ports_stopped(result)


if __name__ == '__main__':
    unittest.main()
