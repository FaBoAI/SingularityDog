"""Offline tests; subprocesses and speaker hardware are never used."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import wave

import level_reversal_audio_launcher as tool


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.bundle = self.root / 'bundle'; self.bundle.mkdir()
        self.output = self.root / 'output'; self.output.mkdir()
        (self.bundle / 'audio').mkdir()
        self.wav = self.bundle / 'audio' / 'test.wav'
        self.make_wav(self.wav)
        for name in tool.SOURCE_NAMES:
            (self.bundle / name).write_text('raise AssertionError("PLAN must not execute recorder")\n')
        (self.bundle / 'level_reversal_audio_launcher.py').write_bytes(Path(tool.__file__).read_bytes())
        self.manifest = {'schema': tool.SCHEMA, 'audio_device': tool.DEVICE,
            'files': {p.relative_to(self.bundle).as_posix(): tool.sha(p.read_bytes())
                      for p in self.bundle.rglob('*') if p.is_file()},
            'clips': [{'text': 'instruction', 'path': 'audio/test.wav', 'sha256': tool.sha(self.wav.read_bytes())}]}
        self.pin()

    def tearDown(self):
        self.temp.cleanup()

    def make_wav(self, path, channels=2, unequal=False, silent=False):
        with wave.open(str(path), 'wb') as wav:
            wav.setnchannels(channels); wav.setsampwidth(2); wav.setframerate(48000)
            sample = b'\0\0' if silent else b'\x10\0'
            frame = sample * channels if not unequal else sample + b'\x11\0'
            wav.writeframes(frame * 14400)

    def pin(self):
        raw = (json.dumps(self.manifest)+'\n').encode()
        (self.bundle / 'bundle-manifest.json').write_bytes(raw)
        self.digest = tool.sha(raw)

    def args(self):
        return ['--bundle', str(self.bundle), '--manifest-sha256', self.digest,
                '--output-root', str(self.output)]

    def test_plan_verifies_without_import_sound_or_records(self):
        with patch.object(tool.subprocess, 'run', side_effect=AssertionError('No audio')), patch.object(tool, 'speaker', side_effect=AssertionError('No speaker')), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main(self.args()), 0)
        self.assertEqual(json.loads(out.getvalue())['status'], 'PLAN_ONLY')
        self.assertEqual(list(self.output.iterdir()), [])

    def test_wrong_pin_and_member_mutation_refused(self):
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, '0'*64)
        (self.bundle / 'record_level_reversal.py').write_text('changed')
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, self.digest)

    def test_different_executing_launcher_refused_even_with_valid_manifest(self):
        name = 'level_reversal_audio_launcher.py'
        (self.bundle / name).write_text('different reviewed copy')
        self.manifest['files'][name] = tool.sha((self.bundle / name).read_bytes()); self.pin()
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main(self.args()), 1)
        self.assertEqual(json.loads(out.getvalue())['status'], 'SPOKEN_LEVEL_NOT_COMPLETED')
        self.assertEqual(list(self.output.iterdir()), [])

    def test_extra_file_and_symlink_refused(self):
        extra = self.bundle / 'unlisted'; extra.write_text('extra')
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, self.digest)
        extra.unlink(); self.wav.unlink(); self.wav.symlink_to(self.root / 'missing')
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, self.digest)

    def test_noncanonical_path_and_duplicate_text_refused(self):
        self.manifest['clips'][0]['path'] = 'audio//test.wav'; self.pin()
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, self.digest)
        self.manifest['clips'][0]['path'] = 'audio/test.wav'
        self.manifest['clips'].append(dict(self.manifest['clips'][0])); self.pin()
        with self.assertRaises(ValueError): tool.verify_bundle(self.bundle, self.digest)

    def test_strict_json_rejects_duplicates_nonfinite_and_overflow(self):
        for raw in (b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":1e999}'):
            with self.assertRaises(ValueError): tool.strict_json(raw)

    def test_deep_manifest_has_structured_failure_without_side_effects(self):
        raw = ('['*1100+'0'+']'*1100).encode()
        (self.bundle / 'bundle-manifest.json').write_bytes(raw); self.digest=tool.sha(raw)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main(self.args()), 1)
        self.assertEqual(json.loads(out.getvalue())['status'], 'SPOKEN_LEVEL_NOT_COMPLETED')
        self.assertEqual(list(self.output.iterdir()), [])

    def test_record_rejects_non_tty_before_creation(self):
        with patch.object(tool.sys.stdin, 'isatty', return_value=False), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(tool.main(self.args()+['--record']), 1)
        self.assertEqual(list(self.output.iterdir()), [])
        self.assertFalse(json.loads(out.getvalue())['output_allowed'])

    def test_output_in_git_or_symlink_refused(self):
        (self.root / '.git').mkdir()
        with self.assertRaises(ValueError): tool.output_root(self.output)
        (self.root / '.git').rmdir()
        link = self.root / 'link'; link.symlink_to(self.output, target_is_directory=True)
        with self.assertRaises(ValueError): tool.output_root(link)

    def test_audio_format_stereo_identity_and_silence_checked(self):
        self.assertAlmostEqual(tool.audio_info(self.wav), .3)
        for kwargs in ({'channels': 1}, {'unequal': True}, {'silent': True}):
            self.make_wav(self.wav, **kwargs)
            with self.assertRaises(ValueError): tool.audio_info(self.wav)

    def test_wrong_route_never_plays_and_saves_failure(self):
        receipts = self.output / 'receipts'; receipts.mkdir(); calls = []
        def run(args, **kwargs):
            calls.append(args)
            return subprocess.CompletedProcess(args, 0, stdout=': values=10\n')
        _, _, mapped = tool.verify_bundle(self.bundle, self.digest)
        with self.assertRaises(ValueError): tool.speaker(self.bundle, self.digest, mapped, receipts, run_process=run)('instruction')
        self.assertTrue(all(c[0] == 'amixer' for c in calls))
        receipt = json.loads(next(receipts.iterdir()).read_text())
        self.assertEqual(receipt['status'], 'PLAYBACK_FAILED'); self.assertIsNone(receipt['audio_heard'])

    def test_completed_playback_does_not_infer_human_hearing(self):
        receipts = self.output / 'receipts'; receipts.mkdir(); calls = []
        def run(args, **kwargs):
            calls.append(args)
            expected = dict(tool.ROUTE).get(args[-1].removeprefix('name='), '')
            return subprocess.CompletedProcess(args, 0, stdout='  : values='+expected+'\n')
        _, _, mapped = tool.verify_bundle(self.bundle, self.digest)
        receipt = tool.speaker(self.bundle, self.digest, mapped, receipts, run_process=run)('instruction')
        self.assertEqual(receipt['status'], 'PLAYBACK_PROCESS_COMPLETE'); self.assertIsNone(receipt['audio_heard'])
        self.assertEqual(calls[-1][:4], ['aplay', '-q', '-D', tool.DEVICE])
        self.assertFalse(receipt['can_opened']); self.assertFalse(receipt['output_allowed'])

    def test_changed_clip_after_playback_cannot_succeed(self):
        receipts = self.output / 'receipts'; receipts.mkdir()
        def run(args, **kwargs):
            if args[0] == 'aplay': self.wav.write_bytes(b'changed')
            expected = dict(tool.ROUTE).get(args[-1].removeprefix('name='), '')
            return subprocess.CompletedProcess(args, 0, stdout=': values='+expected+'\n')
        _, _, mapped = tool.verify_bundle(self.bundle, self.digest)
        with self.assertRaises(ValueError): tool.speaker(self.bundle, self.digest, mapped, receipts, run_process=run)('instruction')
        self.assertEqual(json.loads(next(receipts.iterdir()).read_text())['status'], 'PLAYBACK_FAILED')

    def test_unknown_instruction_and_changed_bundle_never_run_process(self):
        receipts = self.output / 'receipts'; receipts.mkdir()
        _, _, mapped = tool.verify_bundle(self.bundle, self.digest)
        callback = tool.speaker(self.bundle, self.digest, mapped, receipts,
                                run_process=lambda *a, **k: self.fail('Unexpected subprocess'))
        with self.assertRaises(ValueError): callback('unknown')
        (self.bundle / 'record_level_reversal.py').write_text('changed')
        with self.assertRaises(ValueError): callback('instruction')

    def test_receipt_never_overwrites_and_finite_serialization_first(self):
        path = self.output / 'receipt.json'; tool.save(path, {'done': False})
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        with self.assertRaises(FileExistsError): tool.save(path, {'done': True})
        self.assertEqual(json.loads(path.read_text()), {'done': False})
        invalid = self.output / 'invalid.json'
        with self.assertRaises(ValueError): tool.save(invalid, {'x': float('nan')})
        self.assertFalse(invalid.exists())

    def fixture_recorder(self, body):
        path = self.bundle / 'record_level_reversal.py'
        path.write_text("AUDIO_TEXTS={'audio_test':'instruction'}\ndef main(argv=None, *, announce=None):\n    "+body+'\n')
        self.manifest['files'][path.name] = tool.sha(path.read_bytes()); self.pin()

    def test_recorder_abort_return_preserved_and_import_path_restored(self):
        self.fixture_recorder('return 1')
        previous = list(tool.sys.path)
        with patch.object(tool.sys.stdin, 'isatty', return_value=True), patch.object(tool.sys.stdout, 'isatty', return_value=True):
            self.assertEqual(tool.main(self.args()+['--record']), 1)
        self.assertEqual(tool.sys.path, previous)
        final = json.loads(next(self.output.glob('*/final.json')).read_text())
        self.assertEqual(final['recorder_exit_code'], 1)
        self.assertFalse(final['calibration_approved_for_runtime']); self.assertFalse(final['output_allowed'])

    def test_operator_launcher_forces_visual_mode(self):
        self.fixture_recorder("assert '--visual-only' in argv and '--numeric-readings' not in argv; return 0")
        with patch.object(tool.sys.stdin, 'isatty', return_value=True), patch.object(tool.sys.stdout, 'isatty', return_value=True):
            self.assertEqual(tool.main(self.args()+['--record']), 0)
        attempt = next(self.output.iterdir())
        self.assertEqual(json.loads((attempt/'launch.json').read_text())['recording_mode'], 'VISUAL_OBSERVATIONS_ONLY')
        self.assertEqual(json.loads((attempt/'final.json').read_text())['recorder_exit_code'], 0)

    def test_boolean_exit_cannot_create_completed_receipt(self):
        self.fixture_recorder('return True')
        with contextlib.redirect_stdout(io.StringIO()) as out, patch.object(tool.sys.stdin, 'isatty', return_value=True), patch.object(tool.sys.stdout, 'isatty', return_value=True):
            self.assertEqual(tool.main(self.args()+['--record']), 1)
        self.assertEqual(list(self.output.glob('*/final.json')), [])
        self.assertEqual(json.loads(out.getvalue())['status'], 'SPOKEN_LEVEL_NOT_COMPLETED')
        self.assertIn('Recorder exit code', json.loads(out.getvalue())['error'])

    def test_exception_restores_import_path_and_cannot_claim_completion(self):
        self.fixture_recorder("raise ValueError('Injected recorder failure')")
        previous = list(tool.sys.path)
        with contextlib.redirect_stdout(io.StringIO()) as out, patch.object(tool.sys.stdin, 'isatty', return_value=True), patch.object(tool.sys.stdout, 'isatty', return_value=True):
            self.assertEqual(tool.main(self.args()+['--record']), 1)
        self.assertIn('Injected recorder failure', json.loads(out.getvalue())['error'])
        self.assertEqual(tool.sys.path, previous)
        self.assertEqual(list(self.output.glob('*/final.json')), [])


if __name__ == '__main__': unittest.main()
