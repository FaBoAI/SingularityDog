"""One explicitly selected RS05 .9 -> .13 update; default is plan only.

Optional START-only retries require --start-attempts 3; default is one attempt.
No DATA retries or automatic recovery. An incomplete OTA session receives no cleanup
STOP/settings commands. END ACK is not firmware-version verification: a separate
post-update identity/version/STOP/settings comparison is mandatory.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import time

from .active_report_probe import ActiveProbe
from . import dual_can_pipeline_benchmark as dual
from .can_timing_probe import ownership_locks, validate_uids
from .sensor_pipeline_benchmark import BootIdentityGuard
from .rs05_firmware_transfer import (FirmwareTransfer, verify_image, BIN_SHA256,
                                     BIN_SIZE, PACKETS, require)

_HELD_LOCKS = []


def plan(start_attempts=1, start_ack_timeout_s=2):
    require(type(start_attempts) is int and start_attempts in (1, 3), 'Invalid START attempts')
    require(type(start_ack_timeout_s) is int and start_ack_timeout_s in (2, 3), 'Invalid START timeout')
    return {'target_version': '0.5.0.13', 'required_baseline_version_bytes': [0, 5, 0, 9],
            'model': 'RS05', 'image_sha256': BIN_SHA256, 'image_bytes': BIN_SIZE,
            'data_packets': PACKETS, 'scope': 'one explicit ID, no automatic fleet loop',
            'ota_can_types': [11, 12, 13, 14], 'preflight_can_types': [0, 4, 17],
            'automatic_retry': start_attempts > 1, 'start_attempt_limit': start_attempts,
            'start_ack_timeout_seconds': start_ack_timeout_s, 'other_ack_timeout_seconds': 2,
            'retry_scope': 'START only; complete write, zero RX, ACK timeout, 50ms quiet',
            'data_retry_available': False, 'resume_available': False,
            'motion_command_available': False, 'motor_enabling_available': False,
            'cleanup_transmission_available': False, 'maximum_seconds': 210,
            'post_update_verification_required': True}


def validate_backup(blob, digest, expected, boot_id, now):
    require(hashlib.sha256(blob).hexdigest() == digest, 'Backup hash mismatch')
    backup = json.loads(blob)
    require(backup['status'] == 'FIRMWARE_BACKUP_COMPLETE' and backup['locks_released'],
            'Complete closed backup required')
    require(backup['boot_id'] == boot_id, 'Backup is from another boot')
    require(set(backup['results']) == {'front', 'rear'}, 'Both bus backups required')
    for scope, ids in dual.SCOPES.items():
        result = backup['results'][scope]
        require(result['status'] == 'FIRMWARE_BACKUP_COMPLETE' and result['port_closed'],
                'Backup bus incomplete')
        require(result['initial_quiet_observed'] and result['final_quiet_observed'],
                'Backup quiet boundary incomplete')
        require(0 <= now - result['last_host_clock_ns'] <= 3_600_000_000_000,
                'Backup must be at most one hour old in the same boot')
        require(set(result['motors']) == {str(mid) for mid in ids}, 'Backup IDs differ')
        for mid in ids:
            motor = result['motors'][str(mid)]
            require(motor['identity']['mcu_uid_hex'] == expected[mid], 'Backup UID mismatch')
            require(set(motor['parameters']) == {'run_mode', 'position', 'current', 'velocity',
                    'voltage', 'can_timeout', 'zero_state'}, 'Incomplete parameter set')
            require(all(v['ok'] for v in motor['parameters'].values()), 'Rejected backup value')
    return backup


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def save(path, value, *, replace=False):
    data = json.dumps(value, indent=2, allow_nan=False) + '\n'
    target = path.with_name(path.name + '.tmp') if replace else path
    with target.open('w' if replace else 'x') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    if replace:
        os.replace(target, path)
    sync_directory(path.parent)


def run_owned(args, image, expected, bindings, output):
    """No caller can supply arbitrary TX bytes. Default hardware factories only."""
    scope = 'front' if args.motor_id <= 6 else 'rear'
    binding = bindings[scope]
    stack = ExitStack()
    raw = guard = preflight = transfer = None
    signals, handlers = [], {}
    start_attempts = getattr(args, 'start_attempts', 1)
    start_ack_timeout_s = getattr(args, 'start_ack_timeout_seconds', 2)
    report = {**plan(start_attempts, start_ack_timeout_s), 'status': 'INCOMPLETE', 'motor_id': args.motor_id,
              'boot_id': args.expected_boot_id, 'binding': binding,
              'backup_sha256': args.backup_sha256, 'port_closed': False,
              'locks_released': False, 'bootloader_entry_attempted': False,
              'version_verified': False, 'flashed': False}
    deadline = time.monotonic_ns() + 210_000_000_000
    try:
        report['source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(Path(__file__).parent.glob('*.py'))}
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda number, _: signals.append(number))
        guard = BootIdentityGuard()
        require(guard.boot_id == args.expected_boot_id, 'Boot mismatch')
        stack.enter_context(ownership_locks())
        stack.enter_context(dual.port_lock(binding['resolved']))
        def check(cleaning=False):
            require(not signals, 'Update cancelled')
            require(time.monotonic_ns() < deadline, 'Update total deadline exhausted')
            guard.check()
            require(dual.binding_matches(binding), 'Port mapping changed')
        check()
        import serial
        raw = serial.Serial(port=None, baudrate=921600, bytesize=8, parity='N', stopbits=1,
                            timeout=0, write_timeout=.1, exclusive=True,
                            xonxoff=False, rtscts=False, dsrdtr=False)
        raw.dtr = raw.rts = False
        raw.port = binding['path']
        raw.open()
        require(os.fstat(raw.fileno()).st_rdev == binding['st_rdev'], 'Opened FD mismatch')
        # Verify every identity on this bus and stopped quiet operation before OTA.
        preflight = ActiveProbe(raw, tuple(dual.SCOPES[scope]), expected, seconds=1,
            preflight_only=True, read_versions=True, period_policy='observe-current', check=check)
        pre = preflight.run()
        report['preflight'] = pre
        require(pre['status'] == 'PREFLIGHT_COMPLETE', 'Update preflight incomplete')
        require(pre['versions_by_id'][str(args.motor_id)]['version_bytes'] == [0, 5, 0, 9],
                'Target baseline is not exactly RS05 0.5.0.9')
        save(output / 'preflight.json', pre)
        save(output / 'preflight-tx.json', preflight.tx_log)
        save(output / 'preflight-raw.json', [dict(read_started_ns=a, received_ns=b, hex=c.hex())
                                           for a, b, c in preflight.raw_log])
        # Durable evidence exists before the first bootloader-changing operation.
        save(output / 'update-intent.json', report)
        def progress(state):
            save(output / 'progress.json', state, replace=True)
            print(json.dumps({'motor_id': args.motor_id, 'status': state['status'],
                  'acknowledged_data_packets': state['acknowledged_data_packets'],
                  'total_data_packets': PACKETS, 'end_acknowledged': state['end_acknowledged']}), flush=True)
        transfer = FirmwareTransfer(raw, image, args.motor_id, bytes.fromhex(expected[args.motor_id]),
                                    check=check, progress=progress, start_attempts=start_attempts,
                                    start_ack_timeout_s=start_ack_timeout_s)
        report['transfer'] = transfer.run()
        report['bootloader_entry_attempted'] = transfer.report['bootloader_entry_attempted']
        if transfer.report['status'] == 'TRANSFER_ACK_COMPLETE_PENDING_VERSION':
            report['status'] = 'TRANSFER_ACK_COMPLETE_PENDING_VERSION'
    except BaseException as exc:
        report['failure'] = repr(exc)
    finally:
        if transfer is not None:
            report['transfer'] = transfer.report
            report['bootloader_entry_attempted'] = transfer.report['bootloader_entry_attempted']
        # Once OTA starts, even an ordinary STOP must not be sent to an unknown
        # bootloader/application state. Only close; preserve recovery evidence.
        if raw is not None:
            try:
                raw.close()
                report['port_closed'] = not raw.is_open
            except BaseException as exc:
                report['close_failure'] = repr(exc)
        else:
            report['port_closed'] = True
        if report['port_closed']:
            try:
                stack.close()
                report['locks_released'] = True
            except BaseException as exc:
                report['lock_release_failure'] = repr(exc)
        else:
            _HELD_LOCKS.append(stack)
        if guard is not None:
            try:
                guard.close()
            except BaseException as exc:
                report['guard_close_failure'] = repr(exc)
        for number, handler in handlers.items():
            signal.signal(number, handler)
    report['signals'] = signals
    if (signals or not report['port_closed'] or not report['locks_released'] or
            any(key in report for key in ('close_failure', 'lock_release_failure', 'guard_close_failure'))):
        report['status'] = 'INCOMPLETE'
    evidence = {}
    if transfer is not None:
        evidence.update({'ota-tx.json': transfer.tx_log, 'ota-raw.json': transfer.raw_log})
    if preflight is not None:
        evidence.update({'preflight.json': preflight.report, 'preflight-tx.json': preflight.tx_log,
            'preflight-raw.json': [dict(read_started_ns=a, received_ns=b, hex=c.hex())
                                  for a, b, c in preflight.raw_log]})
    # A disk failure must not suppress attempts to preserve the other evidence.
    for name, value in evidence.items():
        try:
            if not (output / name).exists():
                save(output / name, value)
        except BaseException as exc:
            report.setdefault('evidence_errors', []).append(name + ': ' + repr(exc))
            report['status'] = 'INCOMPLETE'
    try:
        save(output / 'summary.json', report)
    except BaseException as exc:
        report.setdefault('evidence_errors', []).append('summary.json: ' + repr(exc))
        report['status'] = 'INCOMPLETE'
        # The process console remains a last resort if the output volume fails.
        print(json.dumps({'status': report['status'], 'motor_id': args.motor_id,
              'bootloader_entry_attempted': report['bootloader_entry_attempted'],
              'evidence_errors': report['evidence_errors']}), flush=True)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-update', action='store_true')
    parser.add_argument('--motor-id', type=int, choices=range(1, 13))
    parser.add_argument('--start-attempts', type=int, choices=(1, 3), default=1)
    parser.add_argument('--start-ack-timeout-seconds', type=int, choices=(2, 3), default=2)
    for name in ('image', 'front-port', 'rear-port', 'expected-uids', 'expected-boot-id',
                 'backup-summary', 'backup-sha256', 'output'):
        parser.add_argument('--' + name)
    args = parser.parse_args(argv)
    if not args.execute_update:
        print(json.dumps(plan(args.start_attempts, args.start_ack_timeout_seconds), indent=2))
        return 0
    require(all(vars(args).values()), 'All pinned update arguments are required')
    image = verify_image(Path(args.image).read_bytes())
    expected = validate_uids(json.loads(Path(args.expected_uids).read_text()))
    validate_backup(Path(args.backup_summary).read_bytes(), args.backup_sha256,
                    expected, args.expected_boot_id, time.monotonic_ns())
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    output = Path(args.output).expanduser().resolve()
    require(not any((p / '.git').exists() for p in (output, *output.parents)),
            'Private output outside Git required')
    output.mkdir(mode=0o700, exist_ok=False)
    sync_directory(output.parent)
    report = run_owned(args, image, expected, bindings, output)
    print(json.dumps({k: report[k] for k in ('status', 'motor_id', 'port_closed',
                      'locks_released', 'bootloader_entry_attempted', 'version_verified')}), flush=True)
    return 0 if report['status'] == 'TRANSFER_ACK_COMPLETE_PENDING_VERSION' else 2


if __name__ == '__main__':
    raise SystemExit(main())
