"""Evidence is identity-bound and cannot silently enable another leg or pose."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw.position_response_evidence import load_position_response_evidence


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


if __name__ == '__main__': unittest.main()
