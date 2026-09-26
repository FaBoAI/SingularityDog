"""The speaker gate must complete before any QDD trial subprocess starts."""
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import wave

from singularitydog_hw import i2s_announcement as audio


def write_wav(path: Path, *, seconds=.5, silent=False, rate=48000, channels=2):
    count = int(rate * seconds)
    samples = (b'\0\0' * channels * count if silent else
               b''.join(struct.pack('<' + 'h' * channels,
                                    *([int(3000 * math.sin(i / 20))] * channels))
                        for i in range(count)))
    with wave.open(str(path), 'wb') as out:
        out.setnchannels(channels)
        out.setsampwidth(2)
        out.setframerate(rate)
        out.writeframes(samples)


class I2SAnnouncementTests(unittest.TestCase):
    def test_audio_route_and_playback_finish_before_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            wav = Path(directory) / 'test-start-ja.wav'
            write_wav(wav)
            calls = []

            def run(command, **kwargs):
                calls.append((command, kwargs))
                return subprocess.CompletedProcess(command, 7 if command[0] == 'python3' else 0)

            result = audio.run_after_announcement(['python3', 'trial.py'], path=wav,
                                                  runner=run)
            self.assertEqual(result, 7)
            self.assertEqual([call[0][0] for call in calls],
                             ['amixer', 'amixer', 'amixer', 'aplay', 'python3'])
            self.assertEqual(calls[3][0][-1], str(wav))
            self.assertEqual(calls[3][0][3], audio.DEFAULT_DEVICE)

    def test_route_or_playback_failure_never_starts_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            wav = Path(directory) / 'test-start-ja.wav'
            write_wav(wav)
            for failing in ('amixer', 'aplay'):
                calls = []

                def run(command, **kwargs):
                    calls.append(command[0])
                    if command[0] == failing:
                        raise subprocess.CalledProcessError(1, command)
                    return subprocess.CompletedProcess(command, 0)

                with self.subTest(failing=failing), self.assertRaises(RuntimeError):
                    audio.run_after_announcement(['python3', 'trial.py'],
                                                 path=wav, runner=run)
                self.assertNotIn('python3', calls)

    def test_missing_empty_silent_or_unsupported_wave_never_touches_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            files = [directory / 'missing.wav', directory / 'empty.wav',
                     directory / 'silent.wav', directory / 'unsupported.wav']
            write_wav(files[1], seconds=0)
            write_wav(files[2], silent=True)
            write_wav(files[3], rate=22050, channels=1)
            calls = []

            def run(command, **kwargs):
                calls.append(command)
                return subprocess.CompletedProcess(command, 0)

            for path in files:
                with self.subTest(path=path), self.assertRaises(ValueError):
                    audio.run_after_announcement(['python3', 'trial.py'],
                                                 path=path, runner=run)
            self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
