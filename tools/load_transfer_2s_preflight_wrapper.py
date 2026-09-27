"""Frozen two-USB 80 ms workload preflight; every motor output remains disabled.

Only STOP, parameter reads, volatile watchdog writes and zero-gain frames may
reach either UART. This wrapper has no active branch. Its source directory and
current-boot evidence must be SHA-pinned before placing it on the Jetson.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

BASE = Path('/home/jetson/singularitydog-tests/load-transfer-2s-preflight-r8')
CURRENT = Path('/home/jetson/singularitydog-tests/fullbody-active-20260927-r1')
BOOT = '5662ee00-b2f5-4913-9bfd-33ae39642427'
SOURCES = ('__init__.py', 'bounded_pose_plan.py', 'can_readonly.py',
           'current_hold_review.py', 'position_response_evidence.py',
           'rs05_bus_transport.py', 'rs05_joint_trial.py', 'rs05_leg_trial.py',
           'rs05_load_transfer_hold.py', 'rs05_trial_protocol.py')
EVIDENCE = ('floor-summary.json', 'floor-events.jsonl', 'readonly-summary.json',
            'operator-rehearsal.json')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_files(base, expected_manifest_sha256):
    require(base.is_dir() and not base.is_symlink(), 'Frozen preflight directory missing')
    manifest_path = base / 'manifest.json'
    require(manifest_path.is_file() and not manifest_path.is_symlink()
            and sha(manifest_path) == expected_manifest_sha256,
            'Trusted preflight manifest SHA mismatch')
    manifest = json.loads(manifest_path.read_text())
    expected = ({'prepared_load_transfer.py', 'preflight-review.json'}
                | {f'singularitydog_hw/{name}' for name in SOURCES}
                | {f'evidence/{name}' for name in EVIDENCE})
    require(type(manifest) is dict and set(manifest) == expected,
            'Preflight manifest file set differs')
    actual = {str(path.relative_to(base)) for path in base.rglob('*') if path.is_file()}
    require(actual == expected | {'manifest.json'}, 'Preflight bundle has extra/missing file')
    for name, digest in manifest.items():
        path = base / name
        require(type(digest) is str and len(digest) == 64
                and all(c in '0123456789abcdef' for c in digest)
                and path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Preflight pin mismatch: ' + name)
    review = json.loads((base / 'preflight-review.json').read_text())
    require(review.get('boot_id') == BOOT and review.get('duration_s') == 2.
            and review.get('gain_profile') == 'id4-id10-kp4'
            and review.get('preflight_review_complete') is True
            and review.get('wrap_equivalence_motor_ids') == [3, 9]
            and review.get('physical_full_turn_excluded') is True
            and review.get('review_complete') is False
            and review.get('load_transfer_hold_authorized') is False
            and review.get('learned_policy_allowed') is False,
            'Frozen review permits more than disabled preflight')
    require('LIVE_OUTPUT_ENABLED = False' in
            (base / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text(),
            'Frozen runner live gate changed')
    floor = json.loads((base / 'evidence/floor-summary.json').read_text())
    require(floor.get('boot_id') == BOOT
            and review.get('floor_hold_summary_sha256') == manifest['evidence/floor-summary.json']
            and floor.get('events_sha256') == manifest['evidence/floor-events.jsonl']
            and review.get('motor_uids') == floor.get('result', {}).get('review', {}).get('motor_uids'),
            'Preflight review differs from same-boot floor evidence')
    return review


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight-only', action='store_true', required=True)
    parser.add_argument('--supported', action='store_true', required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if (not args.preflight_only or not args.supported or not args.output.is_absolute()
            or args.output.parent != Path('/home/jetson/singularitydog-logs')
            or args.output.exists() or args.output.is_symlink()):
        parser.error('Use disabled supported preflight and a fresh pinned log directory')
    os.umask(0o077)
    sys.dont_write_bytecode = True
    args.output.mkdir(mode=0o700)
    report = {'status': 'INCOMPLETE', 'boot_id': BOOT, 'preflight_only': True,
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
                'Signal or 45 s preflight deadline')

    def emit(event):
        row = {'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), **event}
        with emit_lock:
            require(len(events) < 50000, 'Bounded event log full')
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
        require(Path(__file__).resolve().parent == BASE, 'Use pinned remote preflight path')
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
        from singularitydog_hw.rs05_bus_transport import BusTrialTransport
        from singularitydog_hw.rs05_load_transfer_hold import run_load_transfer_hold
        import serial
        for name in SOURCES:
            if name == '__init__.py':
                continue
            module = sys.modules.get('singularitydog_hw.' + name.removesuffix('.py'))
            if module is not None:
                require(Path(module.__file__).resolve() == BASE / 'singularitydog_hw' / name,
                        'Unexpected imported runtime module: ' + name)
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
                        current_wrapper.DisabledPort(p, legacy.BUS_IDS[bus]),
                        lambda event, b=bus: emit({'bus': b, **event}), check,
                        ids=legacy.BUS_IDS[bus], interleave_feedback=True)
                require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == BOOT,
                        'Jetson boot changed before disabled preflight')
                report['result'] = run_load_transfer_hold(
                    transports, {int(k): v for k, v in expected.items()}, check, emit,
                    validated_review=review, preflight_only=True, live_output=False,
                    duration_s=2., gain_profile='id4-id10-kp4')
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
        require(report['result']['status'] == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
                and report['result']['errors'] == []
                and report['result']['workers']['front']['cycle_count'] == 25
                and report['result']['workers']['rear']['cycle_count'] == 25
                and all(len(report['result']['workers'][bus]['electrical_samples']) == 25
                        for bus in ('front', 'rear'))
                and report['trial_device_closed'] is True
                and report['locks_released'] is True
                and not report['motor_enable_sent'] and not report['motion_gain_sent'],
                'Disabled two-USB 80 ms workload preflight did not pass')
        report['status'] = 'PREFLIGHT_PASSED_RESET_CONFIRMED'
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
    return 0 if report['status'] == 'PREFLIGHT_PASSED_RESET_CONFIRMED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
