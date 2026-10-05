"""Synthetic callbacks only; no live operator, speaker, hardware or network."""
from contextlib import redirect_stderr, redirect_stdout
import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
from unittest import mock
import uuid

import record_level_reversal as r

NOW = lambda: datetime.datetime(2026, 10, 5, 9, 0, tzinfo=datetime.timezone.utc)


def answers(*, audio=False, unknown=True, cycles=1):
    values = ['y', 'u', 'u', 'u', 'u']
    for _ in range(cycles):
        for stage in ('正向き', '反転', '復帰'):
            values += ['u' if unknown else '-0.4 -0.2', stage+'で中央寄りに見えた']
        values += ['u']
    if audio: values += ['y']
    return values


def session(values, *, audio=False, callback=None, cycles=1, visual_only=False):
    iterator = iter(values); spoken, displayed = [], []
    def announce(text):
        spoken.append(text)
        if callback is not None: return callback(text)
    result = r.record_session(r.pinned_analyzer(), cycles=cycles, text_only=not audio,
        announce=announce, read=lambda: next(iterator), write=displayed.append, now=NOW, synthetic=True, visual_only=visual_only)
    return result, spoken, displayed


class RecorderTests(unittest.TestCase):
    def test_default_visual_audio_user_transcript_needs_only_three_observation_inputs(self):
        # Reproduce the words that failed in r1 without replaying any real
        # operator receipt or retrospectively changing its aborted outcome.
        words = ['水平です', '0', '  中央  ']
        supplied = iter(['y', *words, 'y', 'y']); prompts, audible = [], []
        result = r.record_session(r.pinned_analyzer(), cycles=1, text_only=False,
            announce=lambda text: audible.append(text), read=lambda: next(supplied),
            write=prompts.append, now=NOW, synthetic=True)
        self.assertEqual(result['status'], 'COMPLETED_VISUAL_OBSERVATION_INTERACTION_REVIEW_REQUIRED')
        self.assertEqual(result['reading_mode'], 'VISUAL_WORDS_ONLY')
        self.assertEqual([o['observation_note_original'] for o in result['observations']], words)
        self.assertEqual(result['completed_cycles'], 1)
        self.assertEqual(audible, list(r.AUDIO_TEXTS.values()))
        self.assertEqual(sum(e['kind'] == 'operator_input' for e in result['events']), 6)
        self.assertFalse(any('識別名' in p or '測定面を入力' in p or '名称' in p for p in prompts))
        self.assertFalse(any('温度' in p for p in prompts))
        cycle = result['records']['cycles'][0]
        self.assertEqual(result['records']['instrument_id'], 'UNKNOWN')
        for k in ('datum_id', 'contact_patch_id', 'measuring_face_id'): self.assertEqual(cycle[k], 'UNKNOWN')
        self.assertEqual(list(cycle['conditions'].values()), [True, True, True, True, None])
        self.assertTrue(all(o['displacement_interval'] is None for o in result['observations']))
        self.assertTrue(all(row['displacement_interval'] is None for row in cycle['readings']))
        analysis = r.pinned_analyzer().analyze(result['records'])
        self.assertTrue(all(v is None for v in analysis['cycles'][0]['conditional_linear_model_contrasts_units'].values()))
        for k in ('absolute_origin_error_rad', 'absolute_origin_uncertainty_rad'): self.assertIsNone(result[k])
        self.assertTrue(all(result[k] is False for k in r.FALSE_FLAGS))

    def test_visual_unknown_or_numeric_looking_text_stays_original_and_null(self):
        for words in (['u', '不明', '見えない'], ['0', '0 0', '範囲 0 0'], ['右寄り', '端寄り', 'unknown']):
            result, _, _ = session(['y', *words, 'u'], visual_only=True)
            self.assertEqual(result['completed_cycles'], 1)
            self.assertEqual([o['observation_note_original'] for o in result['observations']], words)
            self.assertTrue(all(o['displacement_interval'] is None for o in result['observations']))
            self.assertTrue(all(v is None for v in result['records']['cycles'][0]['conditions'].values()))

    def test_visual_empty_and_yes_retry_only_current_observation_without_extra_note(self):
        result, spoken, displayed = session(['y', '', 'y', '水平です', '0', '中央', 'y'], visual_only=True)
        self.assertEqual(result['completed_cycles'], 1)
        self.assertFalse(spoken)
        self.assertEqual(len(result['observations']), 3)
        self.assertEqual(sum(p.startswith('見えた言葉を入力') for p in displayed), 3)
        self.assertFalse(any(p.startswith('実際に見た気泡') for p in displayed))
        self.assertEqual(r.MAX_ATTEMPTS, 8)

    def test_visual_conditions_n_retains_uncertain_individuals_and_raw_answer(self):
        result, _, _ = session(['y', '中央', '端寄り', '水平です', 'n'], visual_only=True)
        self.assertEqual(result['completed_cycles'], 1)
        self.assertTrue(all(v is None for v in result['records']['cycles'][0]['conditions'].values()))
        self.assertEqual([e['raw_input'] for e in result['events'] if e['kind'] == 'operator_input'][-1], 'n')
        result, _, _ = session(['y', '中央', '端寄り', '水平です', 'y n u y'], visual_only=True)
        self.assertEqual(list(result['records']['cycles'][0]['conditions'].values()), [True, False, None, True, None])

    def test_visual_abort_preserves_partial_original_and_does_not_skip_audio_gate(self):
        result, _, _ = session(['y', '  中央  ', 'q'], visual_only=True)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(result['observations'][0]['observation_note_original'], '  中央  ')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertTrue(all(v is None for v in result['records']['cycles'][0]['conditions'].values()))
        result, _, _ = session(['y', '中央', '端寄り', '水平です', 'y', 'n'], audio=True, visual_only=True)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(len(result['observations']), 3)
        self.assertIsNone(result['all_work_audio_heard_confirmed'])

    def test_visual_callback_exception_stops_before_next_observation(self):
        def announce(text):
            if text == r.AUDIO_TEXTS['reverse']: raise RuntimeError('synthetic speaker failure')
        result, spoken, _ = session(['y', '水平です'], audio=True, callback=announce, visual_only=True)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(len(result['observations']), 1)
        self.assertNotIn(r.AUDIO_TEXTS['return'], spoken)

    def test_visual_default_and_explicit_modes_are_reported_in_plan(self):
        for argv, api, expected in [([], True, 'VISUAL_WORDS_ONLY'), (['--visual-only'], False, 'VISUAL_WORDS_ONLY'),
                                   (['--numeric-readings'], True, 'EXPLICIT_NUMERIC_INTERVALS'), ([], False, 'EXPLICIT_NUMERIC_INTERVALS')]:
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(r.main(argv, visual_only=api, announce=lambda _: self.fail('No plan audio')), 0)
            self.assertEqual(json.loads(out.getvalue())['reading_mode'], expected)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): r.main(['--visual-only', '--numeric-readings'])
        for api in (0, 1, 'yes', None):
            with redirect_stdout(io.StringIO()) as out: self.assertEqual(r.main([], visual_only=api), 1)
            self.assertIsNone(json.loads(out.getvalue())['artifact_sha256'])

    def test_visual_saved_file_and_receipt_distinguish_interaction_from_numeric_precision(self):
        with tempfile.TemporaryDirectory() as d:
            supplied = iter(['y', '水平です', '0', '中央', 'y'])
            with redirect_stdout(io.StringIO()) as out:
                code = r.main(['--record', '--text-only', '--visual-only', '--output-root', str(Path(d).resolve())],
                    read=lambda: next(supplied), write=lambda _: None, now=NOW)
            self.assertEqual(code, 0); receipt = json.loads(out.getvalue())
            self.assertEqual(receipt['status'], 'SAVED_VISUAL_OBSERVATION_INTERACTION_REVIEW_REQUIRED')
            self.assertEqual(receipt['reading_mode'], 'VISUAL_WORDS_ONLY')
            self.assertEqual(receipt['numeric_readings'], 0); self.assertEqual(receipt['observation_notes'], 3)
            path = Path(receipt['output']); bundle = json.loads(path.read_text())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), receipt['artifact_sha256'])
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertIsNone(bundle['records']['angular_sensitivity_rad_per_unit'])
            self.assertIsNone(bundle['absolute_origin_uncertainty_rad'])
            self.assertFalse(receipt['calibration_approved'])

    def test_text_only_unknown_numeric_notes_are_not_angles_or_precision(self):
        result, spoken, _ = session(answers())
        self.assertEqual(result['status'], 'COLLECTED_LEVEL_OBSERVATIONS_REVIEW_REQUIRED')
        self.assertFalse(spoken or result['audio_callback_invoked'])
        self.assertEqual(len(result['observations']), 3)
        self.assertTrue(all(o['displacement_interval'] is None for o in result['observations']))
        self.assertTrue(all(o['physical_observation_at_utc'] is None for o in result['observations']))
        self.assertEqual(result['records']['instrument_id'], 'UNKNOWN')
        self.assertEqual(result['records']['cycles'][0]['datum_id'], 'UNKNOWN')
        self.assertTrue(all(v is None for v in result['records']['cycles'][0]['conditions'].values()))
        self.assertIsNone(result['absolute_origin_error_rad'])
        self.assertTrue(all(result[k] is False for k in r.FALSE_FLAGS))
        analysis = r.pinned_analyzer().analyze(result['records'])
        self.assertIsNone(analysis['absolute_origin_uncertainty_rad'])

    def test_audio_start_end_confirmed_and_every_work_callback_finishes_before_input(self):
        values = iter(answers(audio=True, unknown=False)); events = []
        def announce(text): events.append(('audio', text)); return {'synthetic_playback': True}
        def read(): events.append(('input', None)); return next(values)
        result = r.record_session(r.pinned_analyzer(), cycles=1, text_only=False, announce=announce,
            read=read, write=lambda _: None, now=NOW, synthetic=True, visual_only=False)
        self.assertTrue(result['audio_test_heard_confirmed'] and result['all_work_audio_heard_confirmed'])
        self.assertEqual([e[1] for e in events if e[0] == 'audio'], list(r.AUDIO_TEXTS.values()))
        self.assertEqual(result['observations'][1]['displacement_interval'], [-.4, -.2])
        self.assertEqual(result['records']['record_kind'], 'SYNTHETIC_FIXTURE')

    def test_audio_unheard_or_blank_does_not_advance(self):
        result, spoken, _ = session(['n'], audio=True)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(spoken, [r.AUDIO_TEXTS['audio_test']]); self.assertFalse(result['observations'])
        result, spoken, _ = session(['']*r.MAX_ATTEMPTS, audio=True)
        self.assertEqual(spoken, [r.AUDIO_TEXTS['audio_test']]); self.assertFalse(result['observations'])

    def test_audio_failure_keeps_completed_forward_and_does_not_request_reverse_read(self):
        def announce(text):
            if text == r.AUDIO_TEXTS['reverse']: raise RuntimeError('synthetic speaker failure')
        result, spoken, _ = session(answers(audio=True), audio=True, callback=announce)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(len(result['observations']), 1)
        self.assertNotIn(r.AUDIO_TEXTS['return'], spoken)
        self.assertIsNone(result['all_work_audio_heard_confirmed'])

    def test_cancel_at_reverse_keeps_partial_notes_and_three_slots(self):
        values = ['y', 'u', 'u', 'u', 'u', 'u', '  正向きで中央。  ', 'q']
        result, _, _ = session(values)
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        self.assertEqual(result['observations'][0]['observation_note_original'], '  正向きで中央。  ')
        self.assertEqual(len(result['records']['cycles'][0]['readings']), 3)
        self.assertEqual(result['completed_cycles'], 0)
        self.assertTrue(all(v is None for v in result['records']['cycles'][0]['conditions'].values()))

    def test_blank_interval_or_note_retry_only_that_input(self):
        values = ['y', '', 'u', 'u', 'u', 'u', '', 'NaN 1', '2 1', 'u', '', 'y', '中央寄り', 'u', '反転中央', 'u', '復帰中央', 'u']
        result, spoken, _ = session(values)
        self.assertEqual(result['status'], 'COLLECTED_LEVEL_OBSERVATIONS_REVIEW_REQUIRED')
        self.assertEqual(len(result['observations']), 3); self.assertFalse(spoken)

    def test_individual_conditions_are_not_coerced_and_unknown_precision_stays_null(self):
        values = answers(); values[-1] = 'y n u y'
        result, _, _ = session(values)
        self.assertEqual(list(result['records']['cycles'][0]['conditions'].values()), [True, False, None, True, None])
        for key in ('linear_reading_range', 'angular_sensitivity_rad_per_unit', 'per_reading_seating_bound_units'):
            self.assertIsNone(result['records'][key])

    def test_conditions_are_requested_only_after_all_three_observations(self):
        supplied = iter(answers()); prompts, inputs = [], []
        def write(text): prompts.append(text)
        def read():
            if prompts[-1].startswith('今終えた3回'):
                self.assertEqual(len(inputs), 11)
                self.assertEqual(sum(p.startswith('実際に見た気泡') for p in prompts), 3)
            value = next(supplied); inputs.append(value); return value
        result = r.record_session(r.pinned_analyzer(), cycles=1, text_only=True,
            announce=None, read=read, write=write, now=NOW, synthetic=True, visual_only=False)
        self.assertEqual(result['completed_cycles'], 1)

    def test_twelve_cycles_bounded_and_unknowns_are_not_successful_calibration(self):
        result, _, _ = session(answers(cycles=12), cycles=12)
        self.assertEqual(result['completed_cycles'], 12)
        self.assertEqual(len(result['observations']), 36)
        self.assertLess(len(json.dumps(result).encode()), r.MAX_BYTES)
        self.assertFalse(result['calibration_approved'])
        self.assertTrue(all(o['displacement_interval'] is None for o in result['observations']))

    def test_eof_keyboardinterrupt_and_bad_callback_receipt_abort(self):
        for failure in (EOFError(), KeyboardInterrupt()):
            def read(): raise failure
            result = r.record_session(r.pinned_analyzer(), cycles=1, text_only=True, announce=None,
                read=read, write=lambda _: None, now=NOW, synthetic=True)
            self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')
        result, _, _ = session(answers(audio=True), audio=True, callback=lambda _: {'bad': float('nan')})
        self.assertEqual(result['status'], 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS')

    def test_default_plan_template_and_help_create_nothing_and_never_audio(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve()
            for argv in ([], ['--plan', '--cycles', '12'], ['--template']):
                with redirect_stdout(io.StringIO()) as out:
                    self.assertEqual(r.main(argv, announce=lambda _: self.fail('Audio in plan')), 0)
                result = json.loads(out.getvalue())
                if argv == ['--template']: self.assertEqual(result['record_kind'], 'PLANNED_UNMEASURED')
                else: self.assertEqual(result['status'], 'PLAN_ONLY')
            self.assertFalse(list(root.iterdir()))
            with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as e: r.main(['--help'])
            self.assertEqual(e.exception.code, 0)

    def test_missing_callback_and_nonTTY_refuse_before_output_or_input(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve()
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(r.main(['--record', '--output-root', str(root)]), 1)
            self.assertFalse(list(root.iterdir()))
            with mock.patch.object(r.sys.stdin, 'isatty', return_value=False), redirect_stdout(io.StringIO()):
                self.assertEqual(r.main(['--record', '--text-only', '--output-root', str(root)]), 1)
            self.assertFalse(list(root.iterdir()))

    def test_fresh_UUID_0600_saved_with_separate_hash_receipt_and_partial_is_nonzero(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve()
            for supplied, expected_code in [(answers(), 0), (['y', 'u', 'u', 'u', 'u', 'u', 'q'], 1)]:
                values = iter(supplied)
                with redirect_stdout(io.StringIO()) as out:
                    code = r.main(['--record', '--text-only', '--numeric-readings', '--output-root', str(root)], read=lambda: next(values), write=lambda _: None, now=NOW)
                self.assertEqual(code, expected_code); receipt = json.loads(out.getvalue()); p = Path(receipt['output'])
                self.assertEqual(stat.S_IMODE(p.stat().st_mode), 0o600)
                self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(), receipt['artifact_sha256'])
                self.assertEqual(str(uuid.UUID(p.stem.removeprefix('level-reversal-'))), p.stem.removeprefix('level-reversal-'))
                bundle = json.loads(p.read_text()); self.assertFalse(bundle['report_finalization_claimed'])
                self.assertIsNone(bundle['absolute_origin_uncertainty_rad'])
            self.assertEqual(len(list(root.iterdir())), 2)

    def test_output_relative_git_symlink_and_collision_refused_without_overwrite(self):
        with self.assertRaises(ValueError): r.directory('relative')
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve(); (root/'.git').mkdir()
            with self.assertRaises(ValueError): r.directory(root)
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve(); link = root/'link'; link.symlink_to(root)
            with self.assertRaises(ValueError): r.directory(link)
            tag = str(uuid.uuid4()); writer = r.ReservedJSON(root, tag); writer.save({'synthetic': True}); original = writer.path.read_bytes()
            with self.assertRaises(FileExistsError): r.ReservedJSON(root, tag)
            self.assertEqual(writer.path.read_bytes(), original)

    def test_file_fsync_failure_never_emits_success_hash(self):
        with tempfile.TemporaryDirectory() as d:
            values = iter(answers())
            with mock.patch.object(r.os, 'fsync', side_effect=OSError('synthetic fsync failure')), redirect_stdout(io.StringIO()) as out:
                code = r.main(['--record', '--text-only', '--numeric-readings', '--output-root', str(Path(d).resolve())], read=lambda: next(values), write=lambda _: None, now=NOW)
            self.assertEqual(code, 1); result = json.loads(out.getvalue())
            self.assertIsNone(result['artifact_sha256']); self.assertEqual(result['status'], 'INCOMPLETE_LEVEL_REVERSAL_RECORD')

    def test_parent_fsync_or_file_close_failure_never_emits_success_hash(self):
        for failure_stage in ('parent_fsync', 'file_close'):
            with self.subTest(failure_stage=failure_stage), tempfile.TemporaryDirectory() as d:
                root = Path(d).resolve(); supplied = iter(answers())
                real_fsync, real_close = r.os.fsync, r.os.close
                failed = []
                def fsync(fd):
                    if failure_stage == 'parent_fsync' and stat.S_ISDIR(r.os.fstat(fd).st_mode):
                        raise OSError('synthetic parent fsync failure')
                    return real_fsync(fd)
                def close(fd):
                    if failure_stage == 'file_close' and not failed and stat.S_ISREG(r.os.fstat(fd).st_mode):
                        failed.append(True)
                        # A real close error may leave the descriptor live. Only
                        # this synthetic injection does; final cleanup handles it.
                        raise OSError('synthetic file close failure')
                    return real_close(fd)
                with mock.patch.object(r.os, 'fsync', side_effect=fsync), mock.patch.object(r.os, 'close', side_effect=close), redirect_stdout(io.StringIO()) as out:
                    code = r.main(['--record', '--text-only', '--numeric-readings', '--output-root', str(root)],
                        read=lambda: next(supplied), write=lambda _: None, now=NOW)
                self.assertEqual(code, 1); receipt = json.loads(out.getvalue())
                self.assertEqual(receipt['status'], 'INCOMPLETE_LEVEL_REVERSAL_RECORD')
                self.assertIsNone(receipt['artifact_sha256'])

    def test_source_change_and_invalid_cycle_counts_refused(self):
        with mock.patch.object(r, 'ANALYZER_SHA256', '0'*64), redirect_stdout(io.StringIO()) as out:
            self.assertEqual(r.main([]), 1)
        self.assertEqual(json.loads(out.getvalue())['status'], 'INCOMPLETE_LEVEL_REVERSAL_RECORD')
        for cycles in (0, 13, True):
            with self.assertRaises(ValueError): r.record_session(r.pinned_analyzer(), cycles=cycles, text_only=True,
                announce=None, read=lambda: 'q', write=lambda _: None, now=NOW, synthetic=True)


if __name__ == '__main__':
    unittest.main()
