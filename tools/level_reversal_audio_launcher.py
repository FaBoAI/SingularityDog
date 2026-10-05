"""Pinned Jetson speaker adapter for a manual spirit-level record; no CAN.

The operator-facing mode records visual descriptions, never numeric precision.
Default PLAN verifies source/audio files without importing the recorder,
opening audio, or creating records. --record requires a real terminal. The
caller supplies a reviewed bundle manifest hash; no user response is supplied.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import types
import uuid
import wave

SCHEMA = 'singularitydog.level-reversal-spoken-bundle.v1'
SOURCE_NAMES = {'record_level_reversal.py', 'analyze_level_reversal.py',
                'level_reversal_audio_launcher.py'}
DEVICE = 'plughw:CARD=APE,DEV=0'
ROUTE = (('I2S2 codec frame mode', '1'), ('I2S2 codec master mode', '2'), ('I2S2 Mux', '1'))
FLAGS = {'can_opened': False, 'motor_enable_sent': False, 'motor_parameter_write_sent': False,
         'calibration_approved_for_runtime': False, 'output_allowed': False}


def need(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def regular_bytes(path, maximum):
    path = Path(path)
    need(path.is_absolute() and not any(p.is_symlink() for p in (path, *path.parents)),
         'Absolute regular path without symlinks required')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'rb') as stream:
        original = os.fstat(stream.fileno())
        need(stat.S_ISREG(original.st_mode) and 0 < original.st_size <= maximum, 'Bounded regular file required')
        raw = stream.read(maximum + 1)
        named = path.lstat()
        need((original.st_dev, original.st_ino, original.st_size) ==
             (named.st_dev, named.st_ino, named.st_size) and len(raw) == original.st_size,
             'File binding changed')
    return raw


def strict_json(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            need(key not in value, 'Duplicate JSON key')
            value[key] = item
        return value
    def finite_float(token):
        value = float(token)
        need(math.isfinite(value), 'Nonfinite JSON number')
        return value
    return json.loads(raw, object_pairs_hook=pairs, parse_float=finite_float,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON value')))


def verify_bundle(bundle, manifest_sha):
    bundle = Path(bundle)
    need(bundle.is_absolute() and bundle.resolve() == bundle and bundle.is_dir(), 'Canonical bundle directory required')
    need(type(manifest_sha) is str and len(manifest_sha) == 64
         and all(c in '0123456789abcdef' for c in manifest_sha), 'Explicit bundle SHA256 required')
    raw = regular_bytes(bundle / 'bundle-manifest.json', 65536)
    need(sha(raw) == manifest_sha, 'Bundle manifest changed')
    manifest = strict_json(raw)
    need(type(manifest) is dict and set(manifest) == {'schema', 'files', 'clips', 'audio_device'}, 'Canonical bundle manifest required')
    need(manifest['schema'] == SCHEMA and manifest['audio_device'] == DEVICE, 'Unsupported bundle/device')
    files, clips = manifest['files'], manifest['clips']
    need(type(files) is dict and type(clips) is list and 1 <= len(clips) <= 24, 'Bounded files/clips required')
    need(SOURCE_NAMES <= set(files) and len(files) == len(SOURCE_NAMES) + len(clips), 'Exact source/clip membership required')
    sources, mapped, clip_names = {}, {}, set()
    for clip in clips:
        need(type(clip) is dict and set(clip) == {'text', 'path', 'sha256'}, 'Canonical audio clip required')
        text, relative = clip['text'], clip['path']
        need(type(text) is str and 0 < len(text) <= 2000 and text not in mapped, 'Unique bounded audio text required')
        need(type(relative) is str and relative.startswith('audio/') and Path(relative).name.endswith('.wav')
             and len(Path(relative).parts) == 2 and Path(relative).as_posix() == relative
             and '..' not in Path(relative).parts and relative not in clip_names, 'Canonical unique WAV member required')
        clip_names.add(relative)
        need(files.get(relative) == clip['sha256'], 'Clip source hash differs')
        mapped[text] = dict(clip)
    need(set(files) == SOURCE_NAMES | clip_names, 'Unlisted source members')
    for relative, digest in files.items():
        need(type(digest) is str and len(digest) == 64 and all(c in '0123456789abcdef' for c in digest), 'Invalid member hash')
        raw = regular_bytes(bundle / relative, 12 * 1024 * 1024 if relative in clip_names else 1024 * 1024)
        need(sha(raw) == digest, 'Bundle member changed: ' + relative)
        if relative in SOURCE_NAMES:
            sources[relative] = raw
    observed = set()
    for path in bundle.rglob('*'):
        need(not path.is_symlink(), 'Symlink bundle member')
        if path.is_file():
            observed.add(path.relative_to(bundle).as_posix())
        else:
            need(path.is_dir() and path.relative_to(bundle).as_posix() == 'audio', 'Unexpected bundle directory')
    need(observed == set(files) | {'bundle-manifest.json'}, 'Unexpected/missing bundle file')
    return manifest, sources, mapped


def output_root(path):
    path = Path(path)
    need(path.is_absolute() and path.resolve() == path and path.is_dir()
         and not any(p.is_symlink() or (p / '.git').exists() for p in (path, *path.parents)),
         'Existing canonical private output directory required')
    return path


def save(path, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())


def audio_info(path):
    with wave.open(str(path), 'rb') as wav:
        need((wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) == (2, 2, 48000, 'NONE'), 'PCM16 stereo 48kHz required')
        frames = wav.getnframes(); pcm = wav.readframes(frames)
    duration = frames / 48000
    need(.2 <= duration <= 60 and len(pcm) == frames * 4 and any(pcm), 'Bounded non-silent WAV required')
    need(pcm[0::4] == pcm[2::4] and pcm[1::4] == pcm[3::4], 'Identical left/right channels required')
    return duration


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def speaker(bundle, manifest_sha, mapped, receipts, *, run_process=None):
    run_process = run_process or subprocess.run
    def announce(text):
        verify_bundle(bundle, manifest_sha)
        need(text in mapped, 'No reviewed clip for this instruction')
        clip = mapped[text]; path = bundle / clip['path']; duration = audio_info(path)
        receipt = {'text': text, 'clip_sha256': clip['sha256'], 'device': DEVICE, 'started_at': utc(),
                   'audio_heard': None, **FLAGS}
        try:
            # Existing reviewed speaker routing is read, never reconfigured.
            for name, expected in ROUTE:
                check = run_process(['amixer', '-c', 'APE', 'cget', 'name=' + name],
                                    capture_output=True, text=True, check=True, timeout=3)
                lines = check.stdout.splitlines()
                need(lines and lines[-1].strip() == ': values=' + expected, 'Speaker route differs: ' + name)
            verify_bundle(bundle, manifest_sha)
            run_process(['aplay', '-q', '-D', DEVICE, str(path)], check=True, timeout=duration + 5)
            verify_bundle(bundle, manifest_sha)
            receipt.update(status='PLAYBACK_PROCESS_COMPLETE', completed_at=utc())
        except BaseException as error:
            receipt.update(status='PLAYBACK_FAILED', error=type(error).__name__, completed_at=utc())
            save(receipts / ('audio-' + str(uuid.uuid4()) + '.json'), receipt)
            raise
        save(receipts / ('audio-' + str(uuid.uuid4()) + '.json'), receipt)
        return receipt
    return announce


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--bundle', required=True, type=Path)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--cycles', type=int, default=1, choices=range(1, 13))
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args(argv)
    try:
        manifest, sources, mapped = verify_bundle(args.bundle, args.manifest_sha256)
        need(regular_bytes(Path(__file__).absolute(), 1024 * 1024) == sources['level_reversal_audio_launcher.py'],
             'Executing launcher source differs from pinned bundle member')
        destination = output_root(args.output_root)
        if not args.record:
            print(json.dumps({'status': 'PLAN_ONLY', 'source_sha256': {k: sha(v) for k, v in sources.items()},
                'clip_count': len(mapped), 'cycles': args.cycles,
                'recording_mode': 'VISUAL_OBSERVATIONS_ONLY', 'audio_played': False, **FLAGS})); return 0
        need(sys.stdin.isatty() and sys.stdout.isatty(), 'Real operator terminal required; no automated answers')
        attempt = destination / ('spoken-level-' + str(uuid.uuid4()))
        attempt.mkdir(mode=0o700)
        records = attempt / 'records'; records.mkdir(mode=0o700)
        receipts = attempt / 'audio'; receipts.mkdir(mode=0o700)
        save(attempt / 'launch.json', {'bundle_manifest_sha256': args.manifest_sha256,
            'started_at': utc(), 'source_sha256': {k: sha(v) for k, v in sources.items()},
            'recording_mode': 'VISUAL_OBSERVATIONS_ONLY', **FLAGS})
        module = types.ModuleType('record_level_reversal')
        module.__file__ = str(args.bundle / 'record_level_reversal.py')
        previous_path = list(sys.path)
        try:
            sys.path.insert(0, str(args.bundle))
            exec(compile(sources['record_level_reversal.py'], module.__file__, 'exec'), module.__dict__)
            need(set(module.AUDIO_TEXTS.values()) == set(mapped), 'Instruction/clip map differs')
            announce = speaker(args.bundle, args.manifest_sha256, mapped, receipts)
            code = module.main(['--record', '--visual-only', '--output-root', str(records), '--cycles', str(args.cycles)], announce=announce)
            need(type(code) is int and code in (0, 1), 'Recorder exit code must be zero or one')
        finally:
            sys.path[:] = previous_path
        verify_bundle(args.bundle, args.manifest_sha256)
        save(attempt / 'final.json', {'status': 'RECORDER_RETURNED', 'recorder_exit_code': code, 'completed_at': utc(), **FLAGS})
        return code
    except (OSError, ValueError, TypeError, KeyError, EOFError, KeyboardInterrupt, RecursionError,
            subprocess.SubprocessError, wave.Error) as error:
        print(json.dumps({'status': 'SPOKEN_LEVEL_NOT_COMPLETED', 'error': type(error).__name__ + ': ' + str(error), **FLAGS}, ensure_ascii=False))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
