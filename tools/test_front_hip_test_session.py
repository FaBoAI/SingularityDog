"""The file-only session must preserve evidence and never replay a motor trial."""
from __future__ import annotations

from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import front_hip_test_session as session


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'session'
        self.source = self.base / 'disabled-source'
        self.source.mkdir()
        self.active_source = self.base / 'active-source'
        self.active_source.mkdir()
        self.boot = 'test-boot-123'
        self.uids = {str(i): f'uid-{i}' for i in range(1, 13)}
        put(self.source / 'fullbody-review.json',
            {'boot_id': self.boot, 'motor_uids': self.uids})
        put(self.source / 'manifest.json', {'source': 'frozen'})
        put(self.active_source / 'active-manifest.json', {'source': 'frozen'})
        self.hold = self.base / 'hold-summary.json'
        put(self.hold, {'boot_id': self.boot, 'status': 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'})
        self.calls = {'prepare': 0, 'disabled': 0, 'active': 0, 'preflight': 0}
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(session.role.disabled_base, 'validate_source'))
        stack.enter_context(patch.object(session.role.disabled_base, 'validate_hold'))
        stack.enter_context(patch.object(session.role.active_base, 'validate_frozen'))
        stack.enter_context(patch.object(session.role, 'prepare', side_effect=self.mock_prepare))
        stack.enter_context(patch.object(session.role, 'disabled', side_effect=self.mock_disabled))
        stack.enter_context(patch.object(session.role, 'active', side_effect=self.mock_active))
        stack.enter_context(patch.object(session.role, 'validate_role_group_preflight',
                                         side_effect=self.mock_preflight))
        stack.enter_context(patch.object(session.clearance, 'prepare',
                                         side_effect=self.mock_clearance))

    def mock_prepare(self, source, hold, group, output, profile, amplitude,
                     *, continuous_profile=None):
        self.calls['prepare'] += 1
        self.assertEqual((group, profile, amplitude),
                         ('front-hip', 'front-hip-mirrored', 10.))
        output.mkdir()
        evidence = output / 'fullbody-hold-evidence.json'
        put(evidence, {'revised_active_hold': {'summary_sha256': sha(hold)}})
        candidate = {'boot_id': self.boot, 'role_group': group, 'direction_profile': profile,
                     'amplitude_deg': amplitude, 'current_hold_source_sha256': sha(evidence)}
        if continuous_profile is not None:
            candidate['continuous_profile'] = continuous_profile
            candidate['continuous_waypoints_deg'] = [5., 10.]
        put(output / 'offline-raw-step2-candidate.json', candidate)

    def mock_disabled(self, source, hold, prepared, output):
        self.calls['disabled'] += 1
        output.mkdir()
        shutil.copy2(hold, output / 'current-hold-summary.json')
        put(output / 'manifest.json', {'disabled': True})
        candidate = json.loads((prepared / 'offline-raw-step2-candidate.json').read_text())
        review = {'boot_id': self.boot, 'role_group': 'front-hip',
                  'direction_profile': 'front-hip-mirrored', 'amplitude_deg': 10.}
        if 'continuous_profile' in candidate:
            review['continuous_profile'] = candidate['continuous_profile']
            review['continuous_waypoints_deg'] = candidate['continuous_waypoints_deg']
        put(output / 'step2-review.json', review)
        (output / 'prepared_fullbody.py').write_text('disabled = True\n')

    def mock_preflight(self, disabled, summary, events, boot):
        self.calls['preflight'] += 1
        self.assertEqual(boot, self.boot)
        self.assertEqual(json.loads(summary.read_text())['boot_id'], self.boot)
        self.assertTrue(events.is_file())

    def mock_clearance(self, disabled, preflight, draft):
        data = {'boot_id': self.boot,
                'clearance_reference_preflight_summary_sha256':
                    sha(preflight / 'summary.json'),
                'source_disabled_review_sha256': sha(disabled / 'step2-review.json'),
                **{flag: False for flag in session.role.FLAGS}}
        review = json.loads((disabled / 'step2-review.json').read_text())
        if review.get('continuous_profile'):
            data.update({'continuous_profile': review['continuous_profile'],
                         'continuous_waypoints_deg': [5., 10.],
                         'continuous_19s_reviewed': False})
        put(draft, data)

    def mock_active(self, source, disabled, preflight, physical, output):
        self.calls['active'] += 1
        self.assertEqual(json.loads(physical.read_text())['approved'], True)
        output.mkdir()
        (output / 'step2-preflight').mkdir()
        shutil.copy2(preflight / 'summary.json', output / 'step2-preflight' / 'summary.json')
        shutil.copy2(physical, output / 'physical-review.json')
        put(output / 'active-manifest.json', {'active': True})
        physical_data = json.loads(physical.read_text())
        approved = {'boot_id': self.boot, 'approved': True}
        for key in ('continuous_profile', 'continuous_waypoints_deg',
                    'continuous_19s_reviewed'):
            if key in physical_data:
                approved[key] = physical_data[key]
        put(output / 'step2-active-review.json', approved)
        (output / 'prepared_current_hold.py').write_text('active = True\n')

    def make_preflight_log(self, path=None):
        path = path or self.base / 'preflight-log'
        put(path / 'summary.json', {'boot_id': self.boot})
        (path / 'events.jsonl').write_text('{"event":"stop"}\n')
        return path

    def prepare_all(self):
        session.create(self.root, self.source, self.hold)
        log = self.make_preflight_log()
        session.ingest_preflight(self.root, log)
        physical = self.base / 'physical-reviewed.json'
        put(physical, {'approved': True})
        session.build_active(self.root, self.active_source, physical)
        return log, physical

    def make_result(self, *, stop=True, status='RAW_FRONT_HIP_COMPLETED_RESET_CONFIRMED'):
        log = self.base / 'result-log'
        events = log / 'events.jsonl'
        events.parent.mkdir(parents=True, exist_ok=True)
        events.write_text('{"event":"trial"}\n')
        workers = {}
        for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
            workers[bus] = {'stop_reports':
                            {str(mid): {'confirmed': stop} for mid in ids}}
        put(log / 'summary.json',
            {'boot_id': self.boot,
             'wrapper_sha256': sha(self.root / session.ACTIVE / 'prepared_current_hold.py'),
             'events_sha256': sha(events), 'status': status, 'errors': [],
             'result': {'review': json.loads((self.root / session.ACTIVE /
                                              'step2-active-review.json').read_text()),
                        'stop_confirmed': stop, 'motion_completed': stop,
                        'workers': workers}})
        return log

    def test_create_and_preflight_are_idempotent_and_same_boot(self):
        first = session.create(self.root, self.source, self.hold)
        self.assertEqual(session.create(self.root, self.source, self.hold), first)
        self.assertEqual(self.calls['prepare'], 1)
        self.assertEqual(self.calls['disabled'], 1)
        log = self.make_preflight_log()
        receipt = session.ingest_preflight(self.root, log)
        self.assertEqual(session.ingest_preflight(self.root, log), receipt)
        self.assertEqual(self.calls['preflight'], 4)
        self.assertFalse(json.loads((self.root / 'physical-review-draft.json').read_text())
                         [session.role.FLAGS[0]])
        self.assertFalse(session.status(self.root)['active_package_ready'])

    def test_changed_preflight_log_is_rejected_without_replacing_evidence(self):
        session.create(self.root, self.source, self.hold)
        log = self.make_preflight_log()
        session.ingest_preflight(self.root, log)
        saved = sha(self.root / 'preflight' / 'events.jsonl')
        (log / 'events.jsonl').write_text('{"event":"different"}\n')
        with self.assertRaisesRegex(ValueError, 'Existing evidence differs'):
            session.ingest_preflight(self.root, log)
        self.assertEqual(sha(self.root / 'preflight' / 'events.jsonl'), saved)

    def test_interrupted_create_reuses_prepared_candidate_only_with_same_inputs(self):
        with patch.object(session.role, 'disabled', side_effect=RuntimeError('interrupted')):
            with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                session.create(self.root, self.source, self.hold)
        self.assertEqual(self.calls['prepare'], 1)
        self.assertTrue((self.root / 'inputs.json').is_file())
        self.assertFalse((self.root / 'session.json').exists())
        session.create(self.root, self.source, self.hold)
        self.assertEqual(self.calls['prepare'], 1)
        self.assertEqual(self.calls['disabled'], 1)
        put(self.hold, {'boot_id': self.boot, 'status': 'DIFFERENT-HOLD'})
        with self.assertRaisesRegex(ValueError, 'Existing receipt differs|different source or hold'):
            session.create(self.root, self.source, self.hold)

    def test_active_requires_preflight_and_does_not_auto_run(self):
        session.create(self.root, self.source, self.hold)
        physical = self.base / 'physical-reviewed.json'
        put(physical, {'approved': True})
        with self.assertRaises(FileNotFoundError):
            session.build_active(self.root, self.active_source, physical)
        self.assertEqual(self.calls['active'], 0)
        session.ingest_preflight(self.root, self.make_preflight_log())
        receipt = session.build_active(self.root, self.active_source, physical)
        self.assertEqual(session.build_active(self.root, self.active_source, physical), receipt)
        self.assertEqual(self.calls['active'], 1)
        self.assertFalse(receipt['motor_output_performed'])
        self.assertTrue(receipt['fresh_pre_run_physical_confirmation_required'])

    def test_continuous_profile_is_opt_in_and_requires_nineteen_second_review(self):
        ledger = session.create(self.root, self.source, self.hold, continuous_5_10=True)
        self.assertEqual(ledger['continuous_profile'],
                         session.role.CONTINUOUS_FRONT_HIP_PROFILE)
        with self.assertRaisesRegex(ValueError, 'different source or hold'):
            session.create(self.root, self.source, self.hold)
        session.ingest_preflight(self.root, self.make_preflight_log())
        draft = json.loads((self.root / 'physical-review-draft.json').read_text())
        self.assertEqual(draft['continuous_waypoints_deg'], [5., 10.])
        self.assertFalse(draft['continuous_19s_reviewed'])
        physical = self.base / 'physical-reviewed.json'
        put(physical, {**draft, 'approved': True})
        with self.assertRaisesRegex(ValueError, 'explicit 19 s physical review'):
            session.build_active(self.root, self.active_source, physical)
        self.assertEqual(self.calls['active'], 0)
        put(physical, {**draft, 'approved': True, 'continuous_19s_reviewed': True})
        receipt = session.build_active(self.root, self.active_source, physical)
        self.assertEqual(receipt['continuous_profile'],
                         session.role.CONTINUOUS_FRONT_HIP_PROFILE)
        self.assertEqual(self.calls['active'], 1)
        log = self.make_result(status='RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED')
        attempt = session.ingest_result(self.root, log, 'continuous-attempt-1')
        self.assertTrue(attempt['all_twelve_stop_confirmed'])
        self.assertEqual(attempt['continuous_profile'],
                         session.role.CONTINUOUS_FRONT_HIP_PROFILE)

    def test_attempts_preserve_failures_and_never_auto_retry(self):
        self.prepare_all()
        log = self.make_result(stop=False, status='INCOMPLETE')
        receipt = session.ingest_result(self.root, log, 'attempt-1')
        self.assertFalse(receipt['all_twelve_stop_confirmed'])
        self.assertTrue(receipt['manual_review_required_before_next_motion'])
        self.assertFalse(receipt['automatic_retry_performed'])
        self.assertIn('STOP is unverified', session.status(self.root)['next'])
        self.assertEqual(session.ingest_result(self.root, log, 'attempt-1'), receipt)
        self.assertEqual(self.calls['active'], 1)
        log2 = self.make_result(stop=True)
        session.ingest_result(self.root, log2, 'attempt-2')
        self.assertEqual(len(session.status(self.root)['attempts']), 2)
        self.assertIn('STOP is unverified', session.status(self.root)['next'])

    def test_status_rechecks_active_physical_and_preflight_receipts(self):
        self.prepare_all()
        active = self.root / session.ACTIVE
        physical = active / 'physical-review.json'
        saved = physical.read_text()
        physical.write_text('{"changed":true}\n')
        with self.assertRaisesRegex(ValueError, 'evidence changed'):
            session.status(self.root)
        physical.write_text(saved)
        summary = active / 'step2-preflight' / 'summary.json'
        summary.write_text('{"changed":true}\n')
        with self.assertRaisesRegex(ValueError, 'evidence changed'):
            session.status(self.root)

    def test_early_abort_before_runner_review_is_recorded_if_wrapper_is_pinned(self):
        self.prepare_all()
        log = self.make_result(stop=False, status='INCOMPLETE')
        summary = json.loads((log / 'summary.json').read_text())
        summary['result'] = {}
        put(log / 'summary.json', summary)
        receipt = session.ingest_result(self.root, log, 'early-abort')
        self.assertEqual(receipt['status'], 'INCOMPLETE')
        self.assertFalse(receipt['all_twelve_stop_confirmed'])

    def test_changed_result_and_wrong_boot_are_rejected(self):
        self.prepare_all()
        log = self.make_result()
        session.ingest_result(self.root, log, 'attempt-1')
        (log / 'events.jsonl').write_text('{"event":"changed"}\n')
        with self.assertRaisesRegex(ValueError, 'frozen active package'):
            session.ingest_result(self.root, log, 'attempt-1')
        self.assertEqual(len(session.status(self.root)['attempts']), 1)
        put(log / 'summary.json',
            {**json.loads((log / 'summary.json').read_text()), 'boot_id': 'other-boot',
             'events_sha256': sha(log / 'events.jsonl')})
        with self.assertRaisesRegex(ValueError, 'frozen active package'):
            session.ingest_result(self.root, log, 'attempt-2')


if __name__ == '__main__':
    unittest.main()
