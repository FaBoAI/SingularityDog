"""Play the Jetson I2S test announcement before launching a QDD trial.

The guard uses the APE/ADMAIF1 -> I2S2 route and a pre-generated Japanese
PCM WAV. It never sends a CAN frame. A failed audio route or playback prevents
the supplied motor-test command from starting.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import wave


DEFAULT_DEVICE = 'plughw:CARD=APE,DEV=0'
DEFAULT_WAV = Path('/home/jetson/singularitydog-tests/audio/test-start-ja.wav')
MAX_WAV_BYTES = 2_000_000


def validate_announcement(path: Path) -> float:
    """Return PCM duration after rejecting missing, empty or unsuitable audio."""
    path = Path(path)
    if not path.is_file() or path.is_symlink() or path.stat().st_size > MAX_WAV_BYTES:
        raise ValueError(f'Announcement WAV is missing, linked or too large: {path}')
    try:
        with wave.open(str(path), 'rb') as wav:
            channels, width, rate, frames = (wav.getnchannels(), wav.getsampwidth(),
                                             wav.getframerate(), wav.getnframes())
            if (wav.getcomptype() != 'NONE' or channels != 2 or width != 2
                    or rate != 48000 or not .3 <= frames / rate <= 10.):
                raise ValueError('Announcement must be 0.3–10 s, 48 kHz stereo 16-bit PCM WAV')
            data = wav.readframes(frames)
    except (EOFError, OSError, wave.Error) as error:
        raise ValueError('Announcement WAV cannot be decoded') from error
    if len(data) != frames * channels * width or not any(data):
        raise ValueError('Announcement WAV contains no complete non-silent samples')
    return frames / rate


def play_test_start(path: Path = DEFAULT_WAV, *, device: str = DEFAULT_DEVICE,
                    runner=subprocess.run) -> float:
    """Synchronously announce 「テスト開始します」; raise on any audio failure."""
    duration = validate_announcement(path)
    if not device or any(c.isspace() for c in device):
        raise ValueError('A single ALSA PCM device name is required')
    commands = (
        (['amixer', '-q', '-c', 'APE', 'cset', 'name=I2S2 codec frame mode', 'i2s'], 3.),
        (['amixer', '-q', '-c', 'APE', 'cset', 'name=I2S2 codec master mode', 'cbs-cfs'], 3.),
        (['amixer', '-q', '-c', 'APE', 'cset', 'name=I2S2 Mux', 'ADMAIF1'], 3.),
        (['aplay', '-q', '-D', device, str(path)], min(15., duration + 5.)),
    )
    for command, timeout in commands:
        try:
            runner(command, check=True, capture_output=True, text=True, timeout=timeout)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                OSError) as error:
            raise RuntimeError(f'I2S announcement failed at {command[0]}') from error
    return duration


def run_after_announcement(command: list[str], *, path: Path = DEFAULT_WAV,
                           device: str = DEFAULT_DEVICE,
                           runner=subprocess.run) -> int:
    """Run one trial only after the complete announcement command succeeds."""
    if not command or any(not value for value in command):
        raise ValueError('A nonempty QDD trial command is required')
    play_test_start(path, device=device, runner=runner)
    return runner(command, check=False).returncode


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wav', type=Path, default=DEFAULT_WAV)
    ap.add_argument('--device', default=DEFAULT_DEVICE)
    ap.add_argument('--play-only', action='store_true',
                    help='Check the speaker without opening a QDD trial')
    ap.add_argument('command', nargs=argparse.REMAINDER)
    args = ap.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    try:
        if args.play_only:
            if command:
                ap.error('--play-only cannot include a command')
            duration = play_test_start(args.wav, device=args.device)
            print(f'Announcement playback finished ({duration:.2f} s)')
            return 0
        if not command:
            ap.error('Specify a trial command after --, or use --play-only')
        return run_after_announcement(command, path=args.wav, device=args.device)
    except (RuntimeError, ValueError) as error:
        print(f'QDD trial not started: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
