"""Offline refusal checks for the fresh human-held baseline builder."""
import json
from pathlib import Path
import tempfile
import unittest

from build_load_transfer_2s_rebased import build, validate_capture, R4_SOURCE
from load_transfer_2s_rebased_wrapper import sha


PRIVATE = Path('/private/tmp/fabo-stance-20260927-private')
BOX = PRIVATE / 'fixed-stance-box-20260927-r1'
R4 = PRIVATE / 'load-transfer-2s-partial-r4'
SOURCE = Path(__file__).resolve().parents[1] / 'runtime/singularitydog_hw/fixed_stance_readonly_capture.py'


def require_saved_files(*paths):
    """Private raw replay is optional; supplied evidence keeps all validator gates."""
    if any(not path.is_file() for path in paths):
        raise unittest.SkipTest('Optional 2026-09-27 private raw capture/package is unavailable')


class RebasedCaptureTests(unittest.TestCase):
    def setUp(self):
        require_saved_files(BOX / 'summary.json', BOX / 'capture-draft.json',
                            R4 / 'review.json', R4_SOURCE)
        self.capture = json.loads((BOX / 'summary.json').read_text())
        self.review = json.loads((R4 / 'review.json').read_text())
        self.temp = tempfile.TemporaryDirectory(dir=PRIVATE)
        self.addCleanup(self.temp.cleanup)
        self.report = Path(self.temp.name) / 'report.json'
        self.operator = {
            'boot_id': self.capture['boot_id'],
            'baseline_summary_sha256': sha(BOX / 'summary.json'),
            'stand_removed_under_40v_off': False,
            'two_operators_continuous_support': True,
            'four_paws_floor': True,
            'no_slip_or_clamp_contact': True,
            'cutoff_ready': True,
            'same_pose_maintained_for_next_trial': True,
            'operator_note': 'Synthetic validator check; box capture is not eligible.',
        }

    def check(self, after=0):
        self.report.write_text(json.dumps(self.operator))
        return validate_capture(BOX / 'summary.json', BOX / 'events.jsonl',
                                BOX / 'capture-draft.json', SOURCE, self.report,
                                self.review, after)

    def test_box_supported_pose_is_not_eligible(self):
        with self.assertRaisesRegex(ValueError, 'physical report'):
            self.check()

    def test_capture_before_prior_abort_is_stale_even_with_report(self):
        self.operator['stand_removed_under_40v_off'] = True
        after = self.capture['pose']['started_monotonic_ns'] + 1
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.check(after)

    def test_nonempty_sampling_issues_refused(self):
        self.operator['stand_removed_under_40v_off'] = True
        unstable = PRIVATE / 'fixed-stance-capture-20260927-r4'
        require_saved_files(*(unstable / name for name in
                             ('summary.json','events.jsonl','capture-draft.json')))
        self.operator['baseline_summary_sha256'] = sha(unstable / 'summary.json')
        self.report.write_text(json.dumps(self.operator))
        with self.assertRaisesRegex(ValueError, 'stale, incomplete or unstable'):
            validate_capture(unstable / 'summary.json', unstable / 'events.jsonl',
                             unstable / 'capture-draft.json', SOURCE, self.report,
                             self.review, 0)

    def test_builder_refuses_to_freeze_before_new_supported_result(self):
        out = Path(self.temp.name) / 'r5'
        supported_box = PRIVATE / 'load-transfer-2s-supported-box-r1'
        old_run = PRIVATE / 'load-transfer-2s-supported-active-20260927-r2'
        require_saved_files(old_run / 'summary.json', old_run / 'events.jsonl',
                            supported_box / 'manifest.json', R4 / 'manifest.json',
                            PRIVATE / 'load-transfer-2s-partial-20260927-r2/summary.json',
                            PRIVATE / 'load-transfer-2s-partial-20260927-r2/events.jsonl',
                            BOX / 'events.jsonl',
                            PRIVATE / 'box-supported-physical-report-20260927-r1.json')
        with self.assertRaisesRegex(ValueError, 'New box-pose supported active hold'):
            build(R4,
                  PRIVATE / 'load-transfer-2s-partial-20260927-r2/summary.json',
                  PRIVATE / 'load-transfer-2s-partial-20260927-r2/events.jsonl',
                  BOX / 'summary.json', BOX / 'events.jsonl',
                  BOX / 'capture-draft.json', SOURCE,
                  PRIVATE / 'box-supported-physical-report-20260927-r1.json',
                  supported_box, old_run / 'summary.json',
                  old_run / 'events.jsonl',
                  out)
        self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
