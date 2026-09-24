"""Evidence is identity-bound and cannot silently enable another leg or pose."""
from copy import deepcopy
from contextlib import contextmanager
import json
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw.position_response_evidence import load_position_response_evidence


def evidence_data(leg='FR', uids=None):
    ids = {'FR': (1, 2, 3), 'FL': (4, 5, 6), 'RR': (7, 8, 9), 'RL': (10, 11, 12)}[leg]
    uids = uids or {i: f'{i:016x}' for i in ids}
    return {'schema': 'rs05-manual-response-evidence-v1',
        'scope': f'supported-{leg}-relative-5deg-5s', 'review_complete': True,
        'calibration_verified': False, 'absolute_pose_replay': False,
        'motors': {str(i): {'mcu_uid_hex': uids[i], 'physical_movement_confirmed': True,
            'confirmation_source': 'manual_cli_y', 'position_span_deg': 5.,
            'position_velocity_correlation': .95, 'position_unique_values': 30,
            'source_sha256': {'events.jsonl': 'a'*64, 'summary.json': 'b'*64}}
            for i in ids}}


@contextmanager
def evidence_file(leg='FR', data=None):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'evidence.json'
        path.write_text(json.dumps(evidence_data(leg) if data is None else data))
        yield path


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.uids = {i: f'{i:016x}' for i in (1, 2, 3)}
        self.good = {'schema': 'rs05-manual-response-evidence-v1',
            'scope': 'supported-FR-relative-5deg-5s', 'review_complete': True,
            'calibration_verified': False, 'absolute_pose_replay': False,
            'motors': {str(i): {'mcu_uid_hex': u, 'physical_movement_confirmed': True,
                'confirmation_source': 'manual_cli_y', 'position_span_deg': 5.,
                'position_velocity_correlation': .95, 'position_unique_values': 30,
                'source_sha256': {'events.jsonl': 'a'*64, 'summary.json': 'b'*64}}
                for i,u in self.uids.items()}}

    def load(self, data):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'evidence.json'; p.write_text(json.dumps(data))
            return load_position_response_evidence(p, self.uids)

    def test_valid_record_does_not_claim_current_boot_test_or_calibration(self):
        r = self.load(self.good)
        self.assertTrue(r['prior_manual_response_reviewed'])
        self.assertFalse(r['current_boot_response_retested'])
        self.assertFalse(r['calibration_verified'])

    def test_other_identity_missing_joint_unconfirmed_motion_cannot_be_reused(self):
        for failure in ('identity', 'missing', 'unconfirmed', 'scope', 'calibration', 'small', 'frozen', 'uncorrelated', 'hash'):
            with self.subTest(failure=failure):
                v = deepcopy(self.good)
                if failure == 'identity': v['motors']['2']['mcu_uid_hex'] = 'f'*16
                if failure == 'missing': v['motors'].pop('3')
                if failure == 'unconfirmed': v['motors']['1']['physical_movement_confirmed'] = False
                if failure == 'scope': v['scope'] = 'all-legs'
                if failure == 'calibration': v['calibration_verified'] = True
                if failure == 'small': v['motors']['3']['position_span_deg'] = .05
                if failure == 'frozen': v['motors']['3']['position_unique_values'] = 1
                if failure == 'uncorrelated': v['motors']['3']['position_velocity_correlation'] = .1
                if failure == 'hash': v['motors']['3']['source_sha256'] = {}
                with self.assertRaises(ValueError): self.load(v)

    def test_explicit_fl_scope_accepts_only_fl_and_keeps_old_default_fr(self):
        all_uids = {i: f'{i:016x}' for i in range(1, 13)}
        with evidence_file('FL') as path:
            result = load_position_response_evidence(path, all_uids, leg='FL')
            self.assertEqual(result['motor_ids'], [4, 5, 6])
            self.assertEqual(result['scope'], 'supported-FL-relative-5deg-5s')
            self.assertFalse(result['current_boot_response_retested'])
            self.assertFalse(result['calibration_verified'])
            for leg in ('FR', 'RR', 'RL', 'all'):
                with self.subTest(leg=leg), self.assertRaises(ValueError):
                    load_position_response_evidence(path, all_uids, leg=leg)
            with self.assertRaises(ValueError):
                load_position_response_evidence(path, all_uids)
        with evidence_file() as path:
            self.assertEqual(load_position_response_evidence(path, self.uids)['motor_ids'], [1, 2, 3])
            with self.assertRaises(ValueError):
                load_position_response_evidence(path, all_uids, leg='FL')

    def test_fl_missing_extra_wrong_uid_and_ambiguous_expected_map_fail(self):
        uids = {i: f'{i:016x}' for i in (4, 5, 6)}
        for mutation in ('missing', 'extra', 'wrong_uid', 'duplicate_uid'):
            value = evidence_data('FL')
            if mutation == 'missing': value['motors'].pop('6')
            if mutation == 'extra': value['motors']['1'] = deepcopy(value['motors']['4'])
            if mutation == 'wrong_uid': value['motors']['6']['mcu_uid_hex'] = 'f'*16
            if mutation == 'duplicate_uid': value['motors']['6']['mcu_uid_hex'] = uids[4]
            with self.subTest(mutation=mutation), evidence_file('FL', value) as path:
                with self.assertRaises(ValueError): load_position_response_evidence(path, uids, leg='FL')
        for wrong in ({**uids, '4': uids[4]}, {**uids, 6: uids[4]}, {4: uids[4], 5: uids[5]},
                      {**uids, 6: 'Z'*16}):
            with evidence_file('FL') as path, self.assertRaises(ValueError):
                load_position_response_evidence(path, wrong, leg='FL')

    def test_every_leg_requires_its_own_complete_scoped_record(self):
        uids = {i: f'{i:016x}' for i in range(1, 13)}
        groups = {'FR': [1, 2, 3], 'FL': [4, 5, 6], 'RR': [7, 8, 9], 'RL': [10, 11, 12]}
        for leg, ids in groups.items():
            with evidence_file(leg) as path:
                result = load_position_response_evidence(path, uids, leg=leg)
                self.assertEqual(result['motor_ids'], ids)
                for other in groups:
                    if other != leg:
                        with self.subTest(leg=leg, other=other), self.assertRaises(ValueError):
                            load_position_response_evidence(path, uids, leg=other)

    def test_pinned_content_change_and_duplicate_json_are_rejected(self):
        with evidence_file() as path:
            initial = load_position_response_evidence(path, self.uids)
            self.assertEqual(load_position_response_evidence(path, self.uids,
                expected_sha256=initial['sha256']), initial)
            # Still valid evidence, but a different reviewed byte sequence.
            path.write_text(path.read_text() + '\n')
            with self.assertRaisesRegex(ValueError, 'changed'):
                load_position_response_evidence(path, self.uids, expected_sha256=initial['sha256'])
            path.write_text('{"schema":"bad",' + json.dumps(self.good)[1:])
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                load_position_response_evidence(path, self.uids)
            path.write_text(json.dumps({**self.good, 'extra': float('nan')}))
            with self.assertRaisesRegex(ValueError, 'Nonfinite'):
                load_position_response_evidence(path, self.uids)


if __name__ == '__main__': unittest.main()
