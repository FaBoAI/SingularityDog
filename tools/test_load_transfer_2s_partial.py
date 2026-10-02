"""Offline checks for the frozen two-person partial-load diagnostic."""
import contextlib
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest

from build_load_transfer_2s_partial import build, SUPPORTED_R2_MANIFEST_SHA
from load_transfer_2s_partial_wrapper import (
    ExactWirePort, SOURCE_HELPER, announce_partial_window, partial_cue_deadline,
    patch_partial_source, unpatch_partial_source, sha, verify_files)
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw import rs05_trial_protocol as P


PRIVATE = Path('/private/tmp/fabo-stance-20260927-private')
SUPPORTED = PRIVATE / 'load-transfer-2s-supported-active-r2'
RUN = PRIVATE / 'load-transfer-2s-supported-active-20260927-r2'
PARTIAL = PRIVATE / 'load-transfer-2s-partial-r4'
PRIOR_PARTIAL = PRIVATE / 'load-transfer-2s-partial-r3'
PRIOR_RUN = PRIVATE / 'load-transfer-2s-partial-20260927-r1'


def require_saved_files(*paths):
    """Skip only log replay when private historical files are not supplied."""
    if any(not path.is_file() for path in paths):
        raise unittest.SkipTest('Optional 2026-09-27 private raw capture/package is unavailable')


def tick0(start):
    return [{'kind': 'load_transfer_cycle', 'bus': bus, 'tick': 0,
             'due_monotonic_s': start, 'completed_monotonic_s': start + .06}
            for bus in ('front', 'rear')]


class PartialPackageTests(unittest.TestCase):
    def test_post_enable_stop_marks_abort_before_wire_write(self):
        stopped = threading.Event()

        class Raw:
            def write(self, wire):
                self.assert_stop = stopped.is_set()
                return len(wire)

        raw = Raw()
        gate = ExactWirePort(raw, (1,),
            {'supported_floor_start_raw_rad_by_id': {'1': 0.}},
            parser_type=ATParser, read_request=read_request, protocol=P,
            active_stop_started=stopped)
        gate.write(P.stop_request(phase=P.TrialPhase.STOP, motor_id=1))
        self.assertFalse(stopped.is_set())
        gate.write(P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=1))
        gate.write(P.stop_request(phase=P.TrialPhase.STOP, motor_id=1))
        self.assertTrue(stopped.is_set())
        self.assertTrue(raw.assert_stop)

    def test_frozen_source_and_successful_supported_evidence(self):
        require_saved_files(SUPPORTED / 'manifest.json', PARTIAL / 'manifest.json',
                            PARTIAL / 'review.json')
        self.assertEqual(sha(SUPPORTED / 'manifest.json'), SUPPORTED_R2_MANIFEST_SHA)
        digest = sha(PARTIAL / 'manifest.json')
        review = verify_files(PARTIAL, digest)
        self.assertTrue(review['partial_load_allowed'])
        self.assertFalse(review['self_supported_stance_proven'])
        source = json.loads((SUPPORTED / 'manifest.json').read_text())
        partial = json.loads((PARTIAL / 'manifest.json').read_text())
        for name, expected in source.items():
            if name.startswith('singularitydog_hw/') and not name.endswith('rs05_load_transfer_hold.py'):
                self.assertEqual(partial[name], expected)
        original = (SUPPORTED / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
        changed = (PARTIAL / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
        self.assertEqual(changed, patch_partial_source(original))
        self.assertEqual(original, unpatch_partial_source(changed))

    def test_builder_rejects_failed_supported_run(self):
        require_saved_files(SUPPORTED / 'manifest.json', SUPPORTED / 'review.json',
                            RUN / 'summary.json', RUN / 'events.jsonl',
                            PRIOR_PARTIAL / 'manifest.json',
                            PRIOR_RUN / 'summary.json', PRIOR_RUN / 'events.jsonl')
        with tempfile.TemporaryDirectory(dir=PRIVATE) as temp:
            bad = Path(temp) / 'bad-summary.json'
            report = json.loads((RUN / 'summary.json').read_text())
            report['status'] = 'ABORTED'
            bad.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'Successful same-boot'):
                build(SUPPORTED, bad, RUN / 'events.jsonl', PRIOR_PARTIAL,
                      PRIOR_RUN / 'summary.json', PRIOR_RUN / 'events.jsonl',
                      Path(temp) / 'new-package')

    def test_runtime_tamper_rejected_even_if_manifest_rehashed(self):
        require_saved_files(PARTIAL / 'manifest.json',
                            PARTIAL / 'singularitydog_hw/rs05_bus_transport.py')
        with tempfile.TemporaryDirectory(dir=PRIVATE) as temp:
            folder = Path(temp) / 'tampered'
            shutil.copytree(PARTIAL, folder)
            source = folder / 'singularitydog_hw/rs05_bus_transport.py'
            source.write_text(source.read_text() + '\n# tamper\n')
            manifest = json.loads((folder / 'manifest.json').read_text())
            manifest['singularitydog_hw/rs05_bus_transport.py'] = sha(source)
            (folder / 'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(RuntimeError, 'runtime differs|source gate'):
                verify_files(folder, sha(folder / 'manifest.json'))

    def test_tick0_cue_is_bounded_by_absolute_start(self):
        start = 100.
        self.assertAlmostEqual(partial_cue_deadline(tick0(start), 100.07), 100.57)
        self.assertIsNone(partial_cue_deadline(tick0(start), 100.21))
        self.assertIsNone(partial_cue_deadline(tick0(start), 100.17))
        self.assertIsNone(partial_cue_deadline(tick0(start)[:1], 100.07))

    def test_only_reviewed_id3_full_window_range_exception(self):
        require_saved_files(PRIOR_RUN / 'summary.json', PARTIAL / 'review.json')
        scope = {'math': math, 'FULLBODY_POSITION_PROFILE': 'position-v2-all'}
        exec(SOURCE_HELPER, scope)
        adjust = scope['reviewed_human_supported_id3_window']
        failed = json.loads((PRIOR_RUN / 'summary.json').read_text())
        observed = failed['result']['workers']['front']['settled_windows']['FR']
        observed['motors'] = {int(mid): entry for mid, entry in observed['motors'].items()}
        review = json.loads((PARTIAL / 'review.json').read_text())
        accepted = adjust(deepcopy(observed), review, 'FR')
        self.assertTrue(accepted['passed'])
        self.assertEqual(accepted['errors'], [])
        self.assertTrue(accepted['partial_pre_enable_id3_range_exception_applied'])
        self.assertEqual(accepted['limits_by_motor'][3]['position_range_rad'], .003)
        self.assertEqual(accepted['limits_by_motor'][1]['position_range_rad'], .001)
        too_wide = deepcopy(observed)
        too_wide['motors'][3]['position_range_rad'] = .0031
        self.assertFalse(adjust(too_wide, review, 'FR')['passed'])
        unreviewed = dict(review, partial_load_allowed=False)
        self.assertFalse(adjust(deepcopy(observed), unreviewed, 'FR')['passed'])
        self.assertFalse(adjust(deepcopy(observed), review, 'RR')['passed'])
        for hard_gate in ('abs_OLS_slope_rad_s', 'tail_position_range_rad',
                          'tail_abs_OLS_slope_rad_s', 'abs_velocity_mean_rad_s',
                          'sample0: speed', 'sample0: fault', 'sample0: stale'):
            with self.subTest(hard_gate=hard_gate):
                extra = deepcopy(observed)
                extra['motors'][3]['errors'].append(hard_gate)
                extra['errors'].append('ID3: ' + hard_gate)
                checked = adjust(extra, review, 'FR')
                self.assertFalse(checked['passed'])
                self.assertIn('ID3: ' + hard_gate, checked['errors'])

    def test_early_abort_after_open_cues_immediate_resupport(self):
        start = time.monotonic() - .065
        first, finished, stopped, opened = (threading.Event() for _ in range(4))
        first.set()
        output = []
        worker = threading.Thread(target=announce_partial_window,
            args=(first, finished, stopped, opened, tick0(start), threading.RLock(),
                  threading.Lock()),
            kwargs={'writer': lambda value, **_: output.append(value)})
        worker.start()
        self.assertTrue(opened.wait(.1))
        finished.set()
        worker.join(.2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(output, ['FIRST_HOLD_CYCLE_CONFIRMED PARTIAL_LOAD_WINDOW_OPEN',
                                  'RE_SUPPORT_NOW'])

    def test_ended_or_late_runner_never_opens_window(self):
        for ended in (True, False):
            first, finished, stopped, opened = (threading.Event() for _ in range(4))
            first.set()
            if ended:
                finished.set()
            output = []
            announce_partial_window(first, finished, stopped, opened,
                                    tick0(time.monotonic() - .3),
                                    threading.RLock(), threading.Lock(),
                                    writer=lambda value, **_: output.append(value))
            self.assertFalse(opened.is_set())
            self.assertEqual(output, ['NO_PARTIAL_LOAD RE_SUPPORT_NOW'])

    def test_stop_before_or_during_window_suppresses_or_closes_cue(self):
        for stop_before in (True, False):
            start = time.monotonic() - .065
            first, finished, stopped, opened = (threading.Event() for _ in range(4))
            first.set()
            if stop_before:
                stopped.set()
            output = []
            worker = threading.Thread(target=announce_partial_window,
                args=(first, finished, stopped, opened, tick0(start),
                      threading.RLock(), threading.Lock()),
                kwargs={'writer': lambda value, **_: output.append(value)})
            worker.start()
            if not stop_before:
                self.assertTrue(opened.wait(.1))
                stopped.set()
            worker.join(.2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(output, (
                ['NO_PARTIAL_LOAD RE_SUPPORT_NOW'] if stop_before else
                ['FIRST_HOLD_CYCLE_CONFIRMED PARTIAL_LOAD_WINDOW_OPEN', 'RE_SUPPORT_NOW']))


if __name__ == '__main__':
    unittest.main()
