"""The evidence board must not turn missing or damaged logs into a pass."""
from __future__ import annotations

import json
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prestand_batch_status as board


def put(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class BatchStatusTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name) / 'session'
        self.root.mkdir()
        self.disabled = self.root / board.session.DISABLED
        self.disabled.mkdir()
        self.boot = 'test-boot-20260927'
        self.uids = {str(i): f'uid-{i}' for i in range(1, 13)}
        self.ledger = {'boot_id': self.boot,
                       'continuous_profile': board.session.role.CONTINUOUS_FRONT_HIP_PROFILE}
        put(self.root / 'session.json', self.ledger)
        put(self.disabled / 'step2-review.json',
            {'motor_uids': self.uids, 'firmware': '0.5.0.13'})
        self.hold = self.valid_hold()
        put(self.disabled / 'current-hold-summary.json', self.hold)
        self.check_session = patch.object(board.session, 'check_session',
            return_value=(self.ledger, self.disabled, self.root / 'preflight'))
        self.check_session.start()
        self.addCleanup(self.check_session.stop)

    def valid_hold(self):
        result = {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                  'stop_confirmed': True, 'errors': [], 'gain_profile': 'id4-id10-kp4',
                  'review': {'motor_uids': self.uids}, 'workers': {}}
        for bus, ids in board.BUSES.items():
            keys = [str(mid) for mid in ids]
            stop = {key: {'confirmed': True, 'feedback': {'fault_bits': 0}}
                    for key in keys}
            result['workers'][bus] = {
                'completed': True, 'cycle_count': 100,
                'centers': {key: 0.1 for key in keys},
                'initial_stop': stop, 'stop_reports': stop,
                'motors': {key: {'voltage': 40.0, 'watchdog_readback_ticks': 4000}
                           for key in keys}}
        return {'boot_id': self.boot, 'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                'errors': [], 'motor_enable_sent': True, 'motion_gain_sent': True,
                'trial_device_closed': True, 'locks_released': True, 'result': result}

    def test_no_session_is_missing_and_never_authorizes_standing(self):
        (self.root / 'session.json').unlink()
        report = board.collect(self.root)
        self.assertEqual(set(row['state'] for row in report['checks'].values()),
                         {board.MISSING})
        self.assertFalse(report['standing_authorized'])

    def test_completed_hold_counts_as_one_composite_check(self):
        report = board.collect(self.root)
        checks = report['checks']
        for name in ('session', 'hold', 'uid', 'voltage', 'fault', 'stop'):
            self.assertEqual(checks[name]['state'], board.PASS, name)
        for name in ('preflight', 'active_package', 'continuous_trial'):
            self.assertEqual(checks[name]['state'], board.MISSING, name)
        self.assertFalse(checks['uid']['firmware_live_verified'])
        self.assertFalse(report['standing_authorized'])

    def test_low_voltage_and_nonzero_fault_are_not_silently_accepted(self):
        self.hold['result']['workers']['front']['motors']['1']['voltage'] = 34.9
        self.hold['result']['workers']['rear']['stop_reports']['7']['feedback']['fault_bits'] = 1
        put(self.disabled / 'current-hold-summary.json', self.hold)
        report = board.collect(self.root)
        self.assertEqual(report['checks']['voltage']['state'], board.FAIL)
        self.assertEqual(report['checks']['fault']['state'], board.FAIL)
        self.assertEqual(report['checks']['hold']['state'], board.FAIL)

    def test_missing_voltage_is_distinct_from_bad_voltage(self):
        del self.hold['result']['workers']['front']['motors']['1']['voltage']
        put(self.disabled / 'current-hold-summary.json', self.hold)
        self.assertEqual(board.collect(self.root)['checks']['voltage']['state'],
                         board.MISSING)

    def test_wrong_boot_hold_fails_dependent_checks(self):
        self.hold['boot_id'] = 'other-boot'
        put(self.disabled / 'current-hold-summary.json', self.hold)
        report = board.collect(self.root)
        for name in ('hold', 'uid', 'voltage', 'fault', 'stop'):
            self.assertEqual(report['checks'][name]['state'], board.FAIL)

    def test_preflight_active_and_trial_are_separate_and_same_boot(self):
        put(self.root / 'preflight-receipt.json', {'preflight_passed': True})
        put(self.root / 'preflight' / 'summary.json', {'result': {'workers': {
            bus: {'centers': {str(mid): 0.1 for mid in ids}}
            for bus, ids in board.BUSES.items()}}})
        with patch.object(board.session, 'check_preflight',
                          return_value=(self.ledger, {'preflight_passed': True},
                                        self.disabled, self.root / 'preflight')):
            preflight = board.collect(self.root)
            self.assertEqual(preflight['checks']['preflight']['state'], board.PASS)
            self.assertEqual(preflight['checks']['active_package']['state'], board.MISSING)
            put(self.root / 'active-receipt.json', {'boot_id': self.boot})
            active = self.root / board.session.ACTIVE
            put(active / 'step2-active-review.json', {
                'continuous_profile': self.ledger['continuous_profile'],
                'continuous_waypoints_deg': [5.0, 10.0]})
            physical = {'boot_id': self.boot, 'continuous_19s_reviewed': True,
                        **{flag: True for flag in board.session.role.FLAGS}}
            put(active / 'physical-review.json', physical)
            with patch.object(board.session, 'status', return_value={
                    'boot_id': self.boot, 'active_package_ready': True}):
                ready = board.collect(self.root)
                self.assertEqual(ready['checks']['active_package']['state'], board.PASS)
                self.assertEqual(ready['checks']['continuous_trial']['state'], board.MISSING)
                physical['continuous_19s_reviewed'] = False
                put(active / 'physical-review.json', physical)
                self.assertEqual(board.collect(self.root)['checks']['active_package']['state'],
                                 board.FAIL)

    def test_trial_requires_same_boot_and_stop(self):
        attempt = self.root / 'attempts' / 'attempt-1'
        active = self.root / board.session.ACTIVE
        active.mkdir()
        put(active / 'step2-active-review.json', {'boot_id': self.boot})
        (active / 'prepared_current_hold.py').write_text('frozen = True\n')
        (attempt / 'events.jsonl').parent.mkdir(parents=True)
        (attempt / 'events.jsonl').write_text('{"kind":"trial"}\n')
        stop_reports = {bus: {'stop_reports': {
            str(mid): {'confirmed': True} for mid in ids}}
            for bus, ids in board.BUSES.items()}
        summary = {
            'boot_id': self.boot, 'wrapper_sha256': sha(active / 'prepared_current_hold.py'),
            'events_sha256': sha(attempt / 'events.jsonl'),
            'status': 'RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED', 'errors': [],
            'result': {'review': {'boot_id': self.boot}, 'motion_completed': True,
                       'stop_confirmed': False, 'workers': stop_reports}}
        put(attempt / 'summary.json', summary)
        receipt = {
            'boot_id': self.boot, 'status': 'RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED',
            'motion_completed': True, 'all_twelve_stop_confirmed': False,
            'continuous_profile': self.ledger['continuous_profile'], 'errors': [],
            'summary_sha256': sha(attempt / 'summary.json'),
            'events_sha256': sha(attempt / 'events.jsonl')}
        put(attempt / 'receipt.json', receipt)
        with patch.object(board.session, 'status', return_value={'boot_id': self.boot}):
            stop = board._trial(self.root, self.boot, board.PASS)
            self.assertEqual(stop['state'], board.FAIL)
            summary['result']['stop_confirmed'] = True
            put(attempt / 'summary.json', summary)
            receipt['summary_sha256'] = sha(attempt / 'summary.json')
            receipt['all_twelve_stop_confirmed'] = True
            put(attempt / 'receipt.json', receipt)
            self.assertEqual(board._trial(self.root, self.boot, board.PASS)['state'], board.PASS)
            receipt['boot_id'] = 'other-boot'
            put(attempt / 'receipt.json', receipt)
            self.assertEqual(board._trial(self.root, self.boot, board.PASS)['state'], board.FAIL)

    def test_trial_summary_tampering_fails_even_if_receipt_still_says_pass(self):
        attempt = self.root / 'attempts' / 'attempt-1'
        put(attempt / 'receipt.json', {'boot_id': self.boot,
                                      'all_twelve_stop_confirmed': True})
        with patch.object(board.session, 'status', return_value={'boot_id': self.boot}):
            self.assertEqual(board._trial(self.root, self.boot, board.PASS)['state'], board.FAIL)


if __name__ == '__main__':
    unittest.main()
