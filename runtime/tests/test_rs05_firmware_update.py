import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_firmware_update as update

UIDS = {i: f'{i:016x}' for i in range(1, 13)}
BOOT = '11111111-1111-1111-1111-111111111111'


def backup():
    data = {'status': 'FIRMWARE_BACKUP_COMPLETE', 'locks_released': True,
            'boot_id': BOOT, 'results': {}}
    for scope, ids in update.dual.SCOPES.items():
        data['results'][scope] = {'status': 'FIRMWARE_BACKUP_COMPLETE', 'port_closed': True,
            'initial_quiet_observed': True, 'final_quiet_observed': True,
            'last_host_clock_ns': 1000, 'motors': {str(i): {
                'identity': {'mcu_uid_hex': UIDS[i]},
                'parameters': {k: {'ok': True} for k in ('run_mode', 'position', 'current',
                    'velocity', 'voltage', 'can_timeout', 'zero_state')}} for i in ids}}
    return data


class BackupGateTest(unittest.TestCase):
    def validate(self, data, now=1001):
        blob = json.dumps(data).encode()
        return update.validate_backup(blob, hashlib.sha256(blob).hexdigest(), UIDS, BOOT, now)

    def test_valid_and_default_plan_no_hardware(self):
        self.validate(backup())
        with patch.object(update, 'run_owned', side_effect=AssertionError('No I/O')), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(update.main([]), 0)

    def test_explicit_start_retry_plan_does_not_access_hardware(self):
        output = io.StringIO()
        with patch.object(update, 'run_owned', side_effect=AssertionError('No I/O')), contextlib.redirect_stdout(output):
            self.assertEqual(update.main(['--start-attempts', '3', '--start-ack-timeout-seconds', '3']), 0)
        plan = json.loads(output.getvalue())
        self.assertTrue(plan['automatic_retry'])
        self.assertEqual(plan['start_attempt_limit'], 3)
        self.assertFalse(plan['data_retry_available'])
        self.assertEqual(plan['start_ack_timeout_seconds'], 3)
        self.assertEqual(plan['other_ack_timeout_seconds'], 2)

    def test_wrong_hash(self):
        with self.assertRaises(ValueError):
            update.validate_backup(b'{}', '0' * 64, UIDS, BOOT, 1001)

    def test_reject_incomplete_or_changed(self):
        mutations = [
            lambda d: d.update(status='INCOMPLETE'),
            lambda d: d.update(locks_released=False),
            lambda d: d.update(boot_id='other'),
            lambda d: d['results'].pop('rear'),
            lambda d: d['results']['front'].update(port_closed=False),
            lambda d: d['results']['front'].update(final_quiet_observed=False),
            lambda d: d['results']['front']['motors']['1']['identity'].update(mcu_uid_hex='0' * 16),
            lambda d: d['results']['front']['motors']['1']['parameters'].pop('zero_state'),
            lambda d: d['results']['front']['motors']['1']['parameters']['zero_state'].update(ok=False),
        ]
        for change in mutations:
            with self.subTest(change=change):
                data = backup(); change(data)
                with self.assertRaises((ValueError, KeyError)):
                    self.validate(data)

    def test_stale_or_future(self):
        for now in (999, 3_600_000_001_001):
            with self.assertRaises(ValueError):
                self.validate(backup(), now)


class OwnedUpdateTest(unittest.TestCase):
    def run_fake(self, *, pre_status='PREFLIGHT_COMPLETE', version=None,
                 transfer_status='TRANSFER_ACK_COMPLETE_PENDING_VERSION', close_error=False,
                 guard_error=False, save_error=False, failed_save_name=None, start_attempts=1,
                 start_ack_timeout_s=2):
        events = []
        class Raw:
            def __init__(self, **kwargs): self.is_open = False
            def open(self): self.is_open = True; events.append('open')
            def fileno(self): return 123
            def close(self):
                events.append('close')
                if close_error: raise OSError('close')
                self.is_open = False
        class Guard:
            boot_id = BOOT
            def check(self):
                events.append('guard')
                if guard_error: raise ValueError('boot')
            def close(self): events.append('guard-close')
        class Preflight:
            def __init__(self, raw, ids, expected, **kwargs):
                events.append(('preflight-ids', ids))
                self.tx_log = self.raw_log = []
                self.report = {'status': pre_status, 'versions_by_id': {
                    '1': {'version_bytes': version or [0, 5, 0, 9]}}}
            def run(self): events.append('preflight'); return self.report
        class Transfer:
            def __init__(self, raw, image, mid, uid, **kwargs):
                events.append(('transfer-id', mid, uid))
                events.append(('start-attempts', kwargs['start_attempts']))
                events.append(('start-ack-timeout', kwargs['start_ack_timeout_s']))
                self.tx_log = self.raw_log = []
                self.report = {'status': transfer_status, 'bootloader_entry_attempted': True}
            def run(self): events.append('OTA'); return self.report
        @contextlib.contextmanager
        def lock(*args):
            events.append('lock')
            try: yield
            finally: events.append('unlock')
        args = types.SimpleNamespace(motor_id=1, expected_boot_id=BOOT, backup_sha256='a' * 64,
                                     start_attempts=start_attempts,
                                     start_ack_timeout_seconds=start_ack_timeout_s)
        bindings = {'front': {'path': '/dev/serial/by-path/front', 'resolved': '/dev/fake', 'st_rdev': 1}}
        saved = update.save
        def fake_save(path, value, **kwargs):
            if save_error and path.name == 'update-intent.json': raise OSError('disk full')
            if failed_save_name == path.name: raise OSError('save failure')
            return saved(path, value, **kwargs)
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(sys.modules, {'serial': types.SimpleNamespace(Serial=Raw)}), \
             patch.object(update, 'BootIdentityGuard', Guard), \
             patch.object(update, 'ownership_locks', lock), \
             patch.object(update.dual, 'port_lock', lock), \
             patch.object(update.dual, 'binding_matches', return_value=True), \
             patch.object(update.os, 'fstat', return_value=types.SimpleNamespace(st_rdev=1)), \
             patch.object(update, 'ActiveProbe', Preflight), \
             patch.object(update, 'FirmwareTransfer', Transfer), \
             patch.object(update, 'save', fake_save):
            result = update.run_owned(args, b'image', UIDS, bindings, Path(directory))
            if failed_save_name != 'summary.json':
                self.assertTrue((Path(directory) / 'summary.json').exists())
            if failed_save_name == 'ota-tx.json':
                self.assertTrue((Path(directory) / 'ota-raw.json').exists())
            if failed_save_name == 'preflight-tx.json':
                self.assertTrue((Path(directory) / 'preflight-raw.json').exists())
        return result, events

    def test_success_requires_separate_version_verification(self):
        result, events = self.run_fake()
        self.assertEqual(result['status'], 'TRANSFER_ACK_COMPLETE_PENDING_VERSION')
        self.assertFalse(result['flashed'])
        self.assertFalse(result['version_verified'])
        self.assertIn(('preflight-ids', (1, 2, 3, 4, 5, 6)), events)
        self.assertIn(('transfer-id', 1, bytes.fromhex(UIDS[1])), events)
        self.assertLess(events.index('close'), events.index('unlock'))
        self.assertLess(events.index('unlock'), events.index('guard-close'))

    def test_preflight_or_version_failure_cannot_enter_bootloader(self):
        for opts in ({'pre_status': 'INCOMPLETE'}, {'version': [0, 5, 0, 13]},
                     {'guard_error': True}, {'save_error': True}):
            result, events = self.run_fake(**opts)
            self.assertEqual(result['status'], 'INCOMPLETE')
            self.assertNotIn('OTA', events)
            self.assertFalse(result['bootloader_entry_attempted'])

    def test_failed_ota_has_no_postflight_or_stop_cleanup(self):
        result, events = self.run_fake(transfer_status='INCOMPLETE')
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertTrue(result['bootloader_entry_attempted'])
        after = events[events.index('OTA') + 1:]
        self.assertEqual(after, ['close', 'unlock', 'unlock', 'guard-close'])

    def test_explicit_start_retry_is_passed_and_keeps_failed_cleanup_closed(self):
        result, events = self.run_fake(start_attempts=3, start_ack_timeout_s=3, transfer_status='INCOMPLETE')
        self.assertIn(('start-attempts', 3), events)
        self.assertIn(('start-ack-timeout', 3), events)
        self.assertTrue(result['automatic_retry'])
        self.assertEqual(result['start_attempt_limit'], 3)
        self.assertFalse(result['data_retry_available'])
        self.assertEqual(events[events.index('OTA') + 1:], ['close', 'unlock', 'unlock', 'guard-close'])

    def test_close_failure_retains_ownership(self):
        count = len(update._HELD_LOCKS)
        try:
            result, events = self.run_fake(close_error=True)
            self.assertEqual(result['status'], 'INCOMPLETE')
            self.assertFalse(result['locks_released'])
            self.assertNotIn('unlock', events)
            self.assertEqual(len(update._HELD_LOCKS), count + 1)
        finally:
            for stack in update._HELD_LOCKS[count:]: stack.close()
            del update._HELD_LOCKS[count:]

    def test_evidence_failure_still_saves_other_evidence(self):
        for name in ('ota-tx.json', 'preflight-tx.json', 'summary.json'):
            with contextlib.redirect_stdout(io.StringIO()):
                result, _ = self.run_fake(failed_save_name=name)
            self.assertEqual(result['status'], 'INCOMPLETE')
            self.assertTrue(result['evidence_errors'])


if __name__ == '__main__':
    unittest.main()
