"""Frozen proposal lineage checks; files and private patch copies only."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from ._proposal_fixture import reconstruct_proposal

HERE = Path(__file__).resolve().parent
ARTIFACTS = (
    ('runner', 'type1_runner.py', 'type1_runner_hooks.proposed.patch', 'proposed_runner_sha256'),
    ('transport', 'type1_transport.py', 'transport_hooks.proposed.patch', 'proposed_transport_sha256'),
)


class FrozenProposalFixtureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='frozen-proposal-negative-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def fixture(self, artifact=ARTIFACTS[0]):
        label, file, diff, key = artifact
        manifest = self.root/'manifest.json'
        patch = self.root/'proposal.patch'
        candidate = self.root/'materialized.py'
        manifest.write_bytes((HERE/(label+'-proposal-manifest.json')).read_bytes())
        patch.write_bytes((HERE/diff).read_bytes())
        candidate.write_bytes((HERE/file).read_bytes())
        return manifest, patch, candidate, key

    def test_both_frozen_lineages_reconstruct_exact_historical_base_without_mutation(self):
        for artifact in ARTIFACTS:
            with self.subTest(artifact=artifact[0]):
                args = self.fixture(artifact)
                before = [file.read_bytes() for file in args[:3]]
                result = reconstruct_proposal(*args)
                receipt = json.loads(before[0])
                self.assertEqual(hashlib.sha256(result.base).hexdigest(), receipt['base_sha256'])
                self.assertEqual(result.proposed, before[2])
                self.assertEqual([file.read_bytes() for file in args[:3]], before)

    def test_unreviewed_patch_is_rejected(self):
        args = self.fixture()
        args[1].write_bytes(args[1].read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'Unreviewed proposed patch'):
            reconstruct_proposal(*args)

    def test_unreviewed_materialized_source_is_rejected(self):
        args = self.fixture()
        args[2].write_bytes(args[2].read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'Unreviewed materialized proposal'):
            reconstruct_proposal(*args)

    def test_historical_base_pin_is_checked_after_reverse_replay(self):
        args = self.fixture()
        receipt = json.loads(args[0].read_text())
        receipt['base_sha256'] = '0'*64
        args[0].write_text(json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, 'Unreviewed historical proposal base'):
            reconstruct_proposal(*args)

    def test_reverse_replay_failure_is_retained_despite_matching_input_hashes(self):
        args = self.fixture()
        raw = args[1].read_bytes().replace(b'_real_operating_readback', b'_unmatchable_test_line', 1)
        args[1].write_bytes(raw)
        receipt = json.loads(args[0].read_text())
        receipt['patch_sha256'] = hashlib.sha256(raw).hexdigest()
        args[0].write_text(json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, 'Frozen proposal reverse replay failed'):
            reconstruct_proposal(*args)


if __name__ == '__main__':
    unittest.main()
