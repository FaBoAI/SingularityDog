"""Regress the observed exit-zero/empty-audio failure without running TTS."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import wave


spec = importlib.util.spec_from_file_location('spoken_cue_generator',
    Path(__file__).resolve().parents[2]/'tools/generate_human_supported_spoken_cues.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class SpokenCueGenerationTests(unittest.TestCase):
    def test_empty_or_silent_successful_wav_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'voice.wav'
            for data in (b'', b'\x00'*1920):
                generator.write_pcm(path, data)
                with self.subTest(bytes=len(data)), self.assertRaises(ValueError):
                    generator.pcm_duration(path)

    def test_truncated_pcm_is_rejected_even_with_a_valid_wave_header(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'voice.wav'
            generator.write_pcm(path, generator.tone(.1, 1200))
            path.write_bytes(path.read_bytes()[:-8])
            with self.assertRaises(ValueError):
                generator.pcm_duration(path)

    def test_verified_short_tone_has_exact_format_duration(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'go.wav'
            generator.write_pcm(path, generator.tone(.1, 1200))
            self.assertEqual(generator.pcm_duration(path), .1)
            with wave.open(str(path), 'rb') as recording:
                self.assertEqual(recording.getnframes(), 4800)


if __name__ == '__main__':
    unittest.main()
