"""Synthetic file-only checks; no serial import and no motor access."""

from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest

import build_load_transfer_2s_package as builder
import load_transfer_2s_launcher as launcher


def write(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + '\n')


class PackageTests(unittest.TestCase):
    def fixture(self, root):
        source = Path(root)
        events = []
        for bus in ('front', 'rear'):
            for tick in range(100):
                events.append({'kind': 'fullbody_cycle', 'bus': bus, 'tick': tick,
                               'preflight_only': False})
            for mid in builder.BUS_IDS[bus]:
                for _ in range(100):
                    events.append({'kind': 'bus_trial_feedback', 'bus': bus,
                        'motor_id': mid, 'mode_state': 2, 'fault_bits': 0,
                        'protocol_position_rad': .5 + mid * .1,
                        'velocity_rad_s': 0., 'torque_nm': .01,
                        'temperature_c': 30.})
        events.append({'kind': 'can_tx', 'bus': 'front', 'motor_id': 1, 'type': 4})
        event_path = source / 'events.jsonl'
        event_path.write_text(''.join(json.dumps(event) + '\n' for event in events))
        uids = {str(i): f'{i:016x}' for i in builder.IDS}
        workers = {}
        for bus, ids in builder.BUS_IDS.items():
            workers[bus] = {'completed': True, 'cycle_count': 100,
                            'centers': {str(mid): .5 + mid * .1 for mid in ids},
                            'stop_reports': {str(mid): {'confirmed': True} for mid in ids}}
        floor = {'boot_id': builder.BOOT, 'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                 'errors': [], 'signals': [], 'preflight_only': False,
                 'supported_current_hold_only': True,
                 'learned_policy_allowed': False, 'standing_allowed': False,
                 'motor_enable_sent': True, 'motion_gain_sent': True,
                 'trial_device_closed': True, 'locks_released': True,
                 'port_closes': {'front': True, 'rear': True},
                 'events_sha256': hashlib.sha256(event_path.read_bytes()).hexdigest(),
                 'typed_tx_counts': {'4': 1},
                 'result': {'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED',
                    'errors': [], 'motion_completed': True, 'stop_confirmed': True,
                    'gain_profile': 'id4-id10-kp4', 'learned_policy_allowed': False,
                    'standing_allowed': False, 'l_target_replay_allowed': False,
                    'automatic_retry': False, 'workers': workers,
                    'review': {'boot_id': builder.BOOT, 'firmware': '0.5.0.13',
                               'motor_uids': uids, 'motor_reviews': {}}}}
        floor_path = source / 'summary.json'; write(floor_path, floor)
        readonly = {'boot_id': builder.BOOT, 'status': 'READ_ONLY_COMPLETE',
                    'read_only': True, 'motor_enable_sent': False, 'errors': [],
                    'motors': {str(mid): {'uid_match': True} for mid in builder.IDS}}
        readonly_path = source / 'readonly.json'; write(readonly_path, readonly)
        operator = {'schema': 'rs05-off-power-two-person-transfer-report-v1',
                    'boot_id': builder.BOOT, 'power_40v_off': True,
                    'off_power_transfer_rehearsal_reported': True,
                    'stand_removed_and_replaced': True, 'stand_restored': True,
                    'four_paws_floor_contact': True, 'two_operators_present': True,
                    'no_slip_reported': True, 'no_obstruction_reported': True,
                    'source_note': 'Synthetic operator report for offline test.'}
        operator_path = source / 'operator.json'; write(operator_path, operator)
        return floor_path, event_path, readonly_path, operator_path

    def test_manifest_boot_lock_and_active_rejection(self):
        with tempfile.TemporaryDirectory() as temporary:
            inputs = self.fixture(temporary)
            package = Path(temporary) / 'bundle'
            result = builder.build(*inputs, package)
            digest = result['manifest_sha256']
            self.assertEqual(result['status'], 'LOCAL_AUDIT_PASSED_LIVE_OUTPUT_DISABLED')
            self.assertEqual(launcher.verify_package(package, digest, builder.BOOT)['duration_s'], 2.)
            with self.assertRaisesRegex(ValueError, 'boot'):
                launcher.verify_package(package, digest, 'wrong-boot')
            with self.assertRaisesRegex(ValueError, 'manifest'):
                launcher.verify_package(package, '0'*64, builder.BOOT)
            with launcher.audit_lock(Path(temporary) / 'trial.lock'):
                with self.assertRaisesRegex(ValueError, 'lock already held'):
                    with launcher.audit_lock(Path(temporary) / 'trial.lock'):
                        pass
            with self.assertRaises(SystemExit):
                launcher.main(['--package', str(package), '--manifest-sha256', digest,
                               '--observed-boot-id', builder.BOOT, '--active'])
            (package / 'source/rs05_load_transfer_hold.py').write_text('LIVE_OUTPUT_ENABLED = True\n')
            with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
                launcher.verify_package(package, digest, builder.BOOT)

    def test_physical_flags_and_policy_scope_reject_before_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            paths = self.fixture(temporary)
            operator_path = paths[3]
            original = json.loads(operator_path.read_text())
            for field in ('power_40v_off', 'stand_restored', 'four_paws_floor_contact',
                          'two_operators_present', 'no_slip_reported', 'no_obstruction_reported'):
                row = deepcopy(original); row[field] = False; write(operator_path, row)
                output = Path(temporary) / ('bundle-' + field)
                with self.subTest(field=field), self.assertRaises(ValueError):
                    builder.build(*paths, output)
                self.assertFalse(output.exists())
            write(operator_path, original)
            floor = json.loads(paths[0].read_text())
            floor['learned_policy_allowed'] = True
            write(paths[0], floor)
            with self.assertRaises(ValueError):
                builder.build(*paths, Path(temporary) / 'policy-bundle')

    def test_start_reporter_is_async_cue_helper(self):
        class Stream(io.StringIO):
            def __init__(self):
                super().__init__(); self.flushes = 0
            def flush(self):
                self.flushes += 1
        stream = Stream(); started = threading.Event(); finished = threading.Event()
        started.set()
        self.assertTrue(launcher.active_start_reporter(started, finished, stream=stream))
        self.assertEqual(stream.getvalue(), 'ACTIVE_HOLD_STARTED\n')
        self.assertEqual(stream.flushes, 1)


if __name__ == '__main__':
    unittest.main()
