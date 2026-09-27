"""Frozen two-USB, two-second supported current-position hold diagnostic.

The fixed stand must fully support the torso throughout. Two operators keep
continuous catch and a physical 40 V cutoff. An external I2S wrapper must
finish its Japanese announcement first. No stand removal or load shift is
allowed in this run. The program never retries and never claims stance.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

BASE = Path('/home/jetson/singularitydog-tests/load-transfer-2s-supported-active-r2')
CURRENT = Path('/home/jetson/singularitydog-tests/fullbody-active-20260927-r1')
BOOT = '5662ee00-b2f5-4913-9bfd-33ae39642427'
SOURCES = ('__init__.py', 'bounded_pose_plan.py', 'can_readonly.py',
           'current_hold_review.py', 'position_response_evidence.py',
           'rs05_bus_transport.py', 'rs05_joint_trial.py', 'rs05_leg_trial.py',
           'rs05_load_transfer_hold.py', 'rs05_trial_protocol.py')
EVIDENCE = ('floor-summary.json', 'floor-events.jsonl', 'readonly-summary.json',
            'operator-rehearsal.json', 'disabled-r8-summary.json',
            'disabled-r8-events.jsonl', 'disabled-r8-manifest.json',
            'supported-r1-summary.json', 'supported-r1-events.jsonl',
            'supported-r1-manifest.json')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ExactWirePort:
    """Final pre-write gate for this fixed supported current-position trial."""

    def __init__(self, raw, ids, review, *, parser_type, read_request, protocol):
        self.raw, self.ids, self.review = raw, tuple(ids), review
        self.parser_type, self.read_request, self.protocol = parser_type, read_request, protocol
        self.enable_seen = False
        self.active_targets = {}
        self.center_target_bytes = {}

    def _floor_equivalent(self, raw, mid):
        reference = self.review['supported_floor_start_raw_rad_by_id'][str(mid)]
        delta = raw - reference
        if abs(delta) <= math.radians(3.):
            return True
        return (mid in (3, 9)
                and self.review.get('wrap_equivalence_motor_ids') == [3, 9]
                and self.review.get('physical_full_turn_excluded') is True
                and abs(abs(delta) - 2. * math.pi) <= math.radians(3.))

    def set_centers(self, centers):
        require(not self.center_target_bytes and set(centers) == set(self.ids),
                'Exact-wire gate requires one complete fresh bus center set')
        for mid in self.ids:
            center = centers[mid]
            require(type(center) in (int, float) and math.isfinite(center)
                    and self._floor_equivalent(center, mid),
                    'Fresh exact-wire center differs from supported floor envelope')
            phase = (self.protocol.TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10)
                     else self.protocol.TrialPhase.POSITION_STEP5)
            self.center_target_bytes[mid] = self.protocol.motion_request(
                phase=phase, center_rad=center, motor_id=mid)[7:9]

    @property
    def in_waiting(self):
        return self.raw.in_waiting

    @property
    def port(self):
        return self.raw.port

    def read(self, count):
        return self.raw.read(count)

    def write(self, wire):
        parser = self.parser_type()
        frames = parser.feed(wire) if type(wire) is bytes else []
        require(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes,
                'Exact-wire gate rejects malformed or compound frame')
        frame = frames[0]
        mid, kind = frame.destination, frame.kind
        require(mid in self.ids and frame.flags == 4 and len(frame.data) == 8,
                'Exact-wire gate rejects bus/ID/header')
        P = self.protocol
        if kind == 0:
            allowed = wire == self.read_request(mid)
        elif kind == 3:
            allowed = wire == P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=mid)
        elif kind == 4:
            allowed = wire == P.stop_request(phase=P.TrialPhase.STOP, motor_id=mid)
        elif kind == 18:
            allowed = wire == P.watchdog_setup_request(
                phase=P.TrialPhase.WATCHDOG_SETUP, motor_id=mid)
        elif kind == 17:
            allowed = any(wire == self.read_request(mid, name) for name in
                          ('run_mode', 'position', 'current', 'velocity', 'voltage', 'can_timeout'))
        elif kind == 1:
            phase = (P.TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10)
                     else P.TrialPhase.POSITION_STEP5) if self.enable_seen else P.TrialPhase.ZERO_GAIN
            canonical = P.motion_request(phase=phase, center_rad=0., motor_id=mid)
            allowed = wire[:7] == canonical[:7] and wire[9:] == canonical[9:]
            raw_target = int.from_bytes(wire[7:9], 'big')
            target_rad = P.POSITION_MIN + raw_target * (P.POSITION_MAX - P.POSITION_MIN) / 65535.
            allowed = (allowed and self._floor_equivalent(target_rad, mid))
            if self.enable_seen and allowed:
                allowed = wire[7:9] == self.center_target_bytes.get(mid)
                previous = self.active_targets.setdefault(mid, raw_target)
                allowed = allowed and raw_target == previous
        else:
            allowed = False
        require(allowed, 'Exact-wire gate rejected nonreviewed command')
        written = self.raw.write(wire)
        if kind == 3 and written == len(wire):
            self.enable_seen = True
        return written


def verify_files(base, expected_manifest_sha256):
    require(base.is_dir() and not base.is_symlink(), 'Frozen active directory missing')
    manifest_path = base / 'manifest.json'
    require(manifest_path.is_file() and not manifest_path.is_symlink()
            and sha(manifest_path) == expected_manifest_sha256,
            'Trusted active manifest SHA mismatch')
    manifest = json.loads(manifest_path.read_text())
    expected = ({'prepared_load_transfer.py', 'review.json'}
                | {f'singularitydog_hw/{name}' for name in SOURCES}
                | {f'evidence/{name}' for name in EVIDENCE})
    require(type(manifest) is dict and set(manifest) == expected,
            'Active manifest file set differs')
    actual = {str(path.relative_to(base)) for path in base.rglob('*') if path.is_file()}
    require(actual == expected | {'manifest.json'}, 'Active bundle has extra/missing file')
    for name, digest in manifest.items():
        path = base / name
        require(type(digest) is str and len(digest) == 64
                and all(c in '0123456789abcdef' for c in digest)
                and path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Active pin mismatch: ' + name)
    review = json.loads((base / 'review.json').read_text())
    require(review.get('boot_id') == BOOT and review.get('duration_s') == 2.
            and review.get('gain_profile') == 'id4-id10-kp4'
            and review.get('wrap_equivalence_motor_ids') == [3, 9]
            and review.get('physical_full_turn_excluded') is True
            and review.get('review_complete') is True
            and review.get('load_transfer_hold_authorized') is True
            and review.get('learned_policy_allowed') is False
            and review.get('standing_allowed') is False
            and review.get('automatic_retry_allowed') is False
            and review.get('continuous_human_support_required') is True
            and review.get('stand_fully_supporting_required') is True
            and review.get('partial_load_allowed') is False
            and review.get('self_supported_stance_proven') is False
            and review.get('timed_stop_catch_demonstrated') is False
            and all(review.get(flag) is True for flag in (
                'supported_stance_passed', 'physical_catch_reviewed',
                'power_cutoff_operator_reviewed', 'off_power_transfer_rehearsal_reviewed',
                'load_specific_limits_reviewed', 'serial_write_timeout_verified',
                'two_usb_80ms_workload_verified')),
            'Frozen active review is incomplete or exceeds supported scope')
    require('LIVE_OUTPUT_ENABLED = True' in
            (base / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text(),
            'Frozen runner live gate is not explicitly open')
    disabled_manifest = json.loads((base / 'evidence/disabled-r8-manifest.json').read_text())
    live_source = (base / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
    require(live_source.count('LIVE_OUTPUT_ENABLED = True') == 1,
            'Active source gate must occur exactly once')
    disabled_source = live_source.replace('LIVE_OUTPUT_ENABLED = True',
                                          'LIVE_OUTPUT_ENABLED = False')
    require(hashlib.sha256(disabled_source.encode()).hexdigest() ==
            disabled_manifest['singularitydog_hw/rs05_load_transfer_hold.py']
            and all(manifest['singularitydog_hw/' + name] ==
                    disabled_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Active runtime differs from disabled r8 beyond the one-line source gate')
    floor = json.loads((base / 'evidence/floor-summary.json').read_text())
    require(floor.get('boot_id') == BOOT
            and review.get('floor_hold_summary_sha256') == manifest['evidence/floor-summary.json']
            and floor.get('events_sha256') == manifest['evidence/floor-events.jsonl']
            and review.get('readonly_summary_sha256') == manifest['evidence/readonly-summary.json']
            and review.get('operator_rehearsal_sha256') == manifest['evidence/operator-rehearsal.json']
            and review.get('motor_uids') == floor.get('result', {}).get('review', {}).get('motor_uids'),
            'Active review differs from same-boot floor evidence')
    disabled = json.loads((base / 'evidence/disabled-r8-summary.json').read_text())
    require(disabled.get('boot_id') == BOOT
            and disabled.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and disabled.get('errors') == [] and disabled.get('signals') == []
            and disabled.get('motor_enable_sent') is False
            and disabled.get('motion_gain_sent') is False
            and disabled.get('trial_device_closed') is True
            and disabled.get('locks_released') is True
            and disabled.get('wrapper_sha256') == disabled_manifest['prepared_load_transfer.py']
            and disabled.get('events_sha256') == manifest['evidence/disabled-r8-events.jsonl']
            and disabled.get('result', {}).get('stop_confirmed') is True
            and all(disabled['result']['workers'][bus]['cycle_count'] == 25
                    and len(disabled['result']['workers'][bus]['electrical_samples']) == 25
                    and all(row['confirmed'] is True for row in
                            disabled['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear'))
            and review.get('disabled_r8_summary_sha256') == manifest['evidence/disabled-r8-summary.json']
            and review.get('disabled_r8_events_sha256') == manifest['evidence/disabled-r8-events.jsonl']
            and review.get('disabled_r8_manifest_sha256') == manifest['evidence/disabled-r8-manifest.json'],
            'Exact same-boot disabled 80 ms trial evidence is incomplete')
    prior_manifest = json.loads((base / 'evidence/supported-r1-manifest.json').read_text())
    prior = json.loads((base / 'evidence/supported-r1-summary.json').read_text())
    require(prior.get('boot_id') == BOOT and prior.get('status') == 'ABORTED'
            and prior.get('events_sha256') == manifest['evidence/supported-r1-events.jsonl']
            and prior.get('wrapper_sha256') == prior_manifest['prepared_load_transfer.py']
            and prior.get('motor_enable_sent') is True
            and prior.get('motion_gain_sent') is True
            and prior.get('trial_device_closed') is True
            and prior.get('locks_released') is True
            and prior.get('result', {}).get('stop_confirmed') is True
            and any('Stale feedback' in error for error in prior['result']['errors'])
            and all(prior['result']['workers'][bus]['cycle_count'] == 1
                    for bus in ('front', 'rear'))
            and review.get('supported_r1_summary_sha256') == manifest['evidence/supported-r1-summary.json']
            and review.get('supported_r1_events_sha256') == manifest['evidence/supported-r1-events.jsonl']
            and review.get('supported_r1_manifest_sha256') == manifest['evidence/supported-r1-manifest.json'],
            'Prior one-cycle abort/STOP evidence is incomplete')
    return review


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--active', action='store_true', required=True)
    parser.add_argument('--stand-fully-supporting', action='store_true', required=True)
    parser.add_argument('--paws-floor', action='store_true', required=True)
    parser.add_argument('--continuous-catch', action='store_true', required=True)
    parser.add_argument('--cutoff-ready', action='store_true', required=True)
    parser.add_argument('--off-power-rehearsal', action='store_true', required=True)
    parser.add_argument('--audio-announced', action='store_true', required=True)
    parser.add_argument('--trial-authorized', action='store_true', required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if (not all((args.active, args.stand_fully_supporting, args.paws_floor,
                 args.continuous_catch, args.cutoff_ready,
                 args.off_power_rehearsal, args.audio_announced,
                 args.trial_authorized)) or not args.output.is_absolute()
            or args.output.parent != Path('/home/jetson/singularitydog-logs')
            or args.output.exists() or args.output.is_symlink()):
        parser.error('All reviewed physical, audio and authorization flags plus a fresh log path are required')
    os.umask(0o077)
    sys.dont_write_bytecode = True
    args.output.mkdir(mode=0o700)
    report = {'status': 'INCOMPLETE', 'boot_id': BOOT, 'preflight_only': False,
              'supported_diagnostic_only': True, 'partial_load_allowed': False,
              'continuous_human_support_required': True,
              'load_transfer_proven': False, 'self_supported_stance_proven': False,
              'external_audio_operator_confirmed': True,
              'motor_enable_sent': False, 'motion_gain_sent': False,
              'errors': [], 'signals': [], 'typed_tx_counts': {}, 'captures': {},
              'trial_device_closed': None, 'locks_released': False,
              'wrapper_sha256': sha(__file__), 'started_wall_time_ns': time.time_ns()}
    events, ports, transports, handlers = [], {}, {}, {}
    emit_lock = threading.RLock()
    deadline = time.monotonic() + 45.
    bindings = None
    legacy = None

    def check():
        require(not report['signals'] and time.monotonic() < deadline,
                'Signal or 45 s finite trial deadline')

    def emit(event):
        row = {'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), **event}
        with emit_lock:
            require(len(events) < 50000, 'Bounded event log full')
            if row.get('kind') == 'load_transfer_bus_ready':
                transports[row['bus']].serial.set_centers(row['centers'])
            if row.get('kind') == 'can_tx':
                wire = bytes.fromhex(row['hex'])
                kind = ((int.from_bytes(wire[2:6], 'big') >> 3) >> 24) & 31
                counts = report['typed_tx_counts']
                counts[str(kind)] = counts.get(str(kind), 0) + 1
                if kind == 3:
                    report['motor_enable_sent'] = True
                if kind == 1 and wire[11:15] != bytes(4):
                    report['motion_gain_sent'] = True
            events.append(row)

    try:
        require(Path(__file__).resolve().parent == BASE, 'Use pinned remote active path')
        review = verify_files(BASE, args.manifest_sha256)
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == BOOT,
                'Jetson boot differs from frozen two-second review')
        current_spec = importlib.util.spec_from_file_location('current_fullbody', CURRENT / 'prepared_fullbody.py')
        current_wrapper = importlib.util.module_from_spec(current_spec)
        current_spec.loader.exec_module(current_wrapper)
        current_wrapper.verify_files(CURRENT)
        legacy_spec = importlib.util.spec_from_file_location('legacy_ownership', CURRENT / 'legacy/prepared_transaction.py')
        legacy = importlib.util.module_from_spec(legacy_spec)
        legacy_spec.loader.exec_module(legacy)
        legacy.verify_package(CURRENT / 'legacy')
        expected = legacy.validate_uids(json.loads((CURRENT / 'legacy/expected-uids.json').read_text()))
        require(review['motor_uids'] == expected, 'Current physical 12 UID binding differs')
        sys.path.insert(0, str(BASE))
        from singularitydog_hw.can_readonly import ATParser, read_request
        from singularitydog_hw import rs05_trial_protocol as protocol
        from singularitydog_hw.rs05_bus_transport import BusTrialTransport
        from singularitydog_hw.rs05_load_transfer_hold import _review, run_load_transfer_hold
        import serial
        for name in SOURCES:
            if name == '__init__.py':
                continue
            module = sys.modules.get('singularitydog_hw.' + name.removesuffix('.py'))
            if module is not None:
                require(Path(module.__file__).resolve() == BASE / 'singularitydog_hw' / name,
                        'Unexpected imported runtime module: ' + name)
        _review({int(k): v for k, v in expected.items()}, (5, 6, 8), review,
                False, 'id4-id10-kp4', 2., True)
        bindings = legacy.validate_bindings()
        report['bindings'] = bindings
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, lambda n, _: report['signals'].append(n))
        check()
        with legacy.ownership(bindings, report):
            report['trial_device_closed'] = False
            try:
                for bus in ('front', 'rear'):
                    p = serial.Serial(port=None, baudrate=921600, bytesize=8,
                                      parity='N', stopbits=1, timeout=.002,
                                      write_timeout=.02, exclusive=True,
                                      rtscts=False, dsrdtr=False, xonxoff=False)
                    ports[bus] = p
                    p.dtr = p.rts = False
                    p.port = bindings[bus]['path']
                    p.open()
                    legacy.check_binding(bindings[bus], p)
                    transports[bus] = BusTrialTransport(
                        ExactWirePort(p, legacy.BUS_IDS[bus], review,
                                      parser_type=ATParser, read_request=read_request,
                                      protocol=protocol),
                        lambda event, b=bus: emit({'bus': b, **event}), check,
                        ids=legacy.BUS_IDS[bus], interleave_feedback=True)
                require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == BOOT,
                        'Jetson boot changed before active trial')
                first_cycle_confirmed = threading.Event()
                runner_finished = threading.Event()

                def cue():
                    while not runner_finished.is_set():
                        if first_cycle_confirmed.wait(.01):
                            print('FIRST_HOLD_CYCLE_CONFIRMED', flush=True)
                            return

                cue_thread = threading.Thread(target=cue, name='supported-hold-cue', daemon=True)
                cue_thread.start()
                try:
                    report['result'] = run_load_transfer_hold(
                        transports, {int(k): v for k, v in expected.items()}, check, emit,
                        validated_review=review, preflight_only=False, live_output=True,
                        duration_s=2., gain_profile='id4-id10-kp4',
                        active_start_signal=first_cycle_confirmed)
                finally:
                    runner_finished.set()
                    cue_thread.join(timeout=.2)
                report['first_hold_cycle_confirmed'] = first_cycle_confirmed.is_set()
                require(report['result'].get('stop_confirmed') is True,
                        'All12 STOP not confirmed: CUT40V_POWER')
            finally:
                closed = {}
                for bus, p in ports.items():
                    try:
                        p.close()
                        closed[bus] = not p.is_open
                    except BaseException as error:
                        closed[bus] = False
                        report['errors'].append(repr(error))
                report['port_closes'] = closed
                report['trial_device_closed'] = all(closed.values())
        require(report['result']['status'] == 'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
                and report['result']['errors'] == []
                and report['result']['workers']['front']['cycle_count'] == 25
                and report['result']['workers']['rear']['cycle_count'] == 25
                and all(len(report['result']['workers'][bus]['electrical_samples']) == 25
                        for bus in ('front', 'rear'))
                and report['trial_device_closed'] is True
                and report['locks_released'] is True
                and report.get('first_hold_cycle_confirmed') is True
                and report['motor_enable_sent'] is True
                and report['motion_gain_sent'] is True,
                'Supported two-second current-position hold did not pass')
        report['status'] = 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED'
    except BaseException as error:
        report['errors'].append(repr(error))
        report['status'] = 'ABORTED'
    finally:
        for sig, prior in handlers.items():
            signal.signal(sig, prior)
        report['completed_wall_time_ns'] = time.time_ns()
        events_path = args.output / 'events.jsonl'
        with events_path.open('x') as stream:
            for event in events:
                stream.write(json.dumps(event, allow_nan=False) + '\n')
            stream.flush(); os.fsync(stream.fileno())
        report['events_sha256'] = sha(events_path)
        with (args.output / 'summary.json').open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.flush(); os.fsync(stream.fileno())
    print(json.dumps({key: report[key] for key in
        ('status', 'errors', 'motor_enable_sent', 'motion_gain_sent', 'locks_released')}), flush=True)
    return 0 if report['status'] == 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
