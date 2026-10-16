"""The exact reviewed CPU source must not admit other C++ or unsafe text."""
from pathlib import Path
import tempfile
import unittest

import check_publication as P


class CauseSourcePublicationTests(unittest.TestCase):
    name = 'runtime/experiments/type1_latency_cause_probe/gil_reacquire.cpp'

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.path = self.root / self.name
        self.path.parent.mkdir(parents=True)
        self.original = (Path(__file__).resolve().parents[1] / self.name).read_bytes()
        self.path.write_bytes(self.original)

    def tearDown(self):
        self.tmp.cleanup()

    def test_exact_reviewed_original_cpu_source_is_text(self):
        self.assertEqual(P.check(self.root, [self.name]), [])

    def test_changed_source_does_not_keep_review(self):
        self.path.write_bytes(self.original + b'\n// changed\n')
        self.assertIn('reviewed text source changed',
                      [v['reason'] for v in P.check(self.root, [self.name])])

    def test_other_cpp_is_still_excluded(self):
        name = self.name.replace('gil_reacquire.cpp', 'other.cpp')
        (self.root / name).write_bytes(self.original)
        self.assertIn('excluded asset or file type',
                      [v['reason'] for v in P.check(self.root, [name])])

    def test_reviewed_name_does_not_bypass_binary_or_privacy_scan(self):
        network = '.'.join(('192', '168', '1', '23')).encode()
        self.path.write_bytes(self.original + b'\x00\n' + network)
        reasons = [v['reason'] for v in P.check(self.root, [self.name])]
        self.assertIn('binary content', reasons)
        self.assertIn('private-network-address', reasons)
        self.assertIn('reviewed text source changed', reasons)


if __name__ == '__main__':
    unittest.main()
