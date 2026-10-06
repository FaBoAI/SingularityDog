#!/usr/bin/env python3
"""Spoken IMU-only six-face acquisition; default PLAN does not open hardware.

Record six fitting poses followed by six independently repositioned check
poses. The signed-permutation mount is an explicitly selected hypothesis used
only to explain the poses. Face labels remain operator assertions. No motor
module is imported, no CAN command is sent, and no calibration is installed.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import wave

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'runtime'
BODY_POSES = (
    (2, 1, '通常の姿勢で、胴体の上側を上へ向けます。'),
    (2, -1, '機体の上下を反転し、胴体の底側を上へ向けます。'),
    (1, 1, '犬の左側面を上へ向けます。'),
    (1, -1, '犬の右側面を上へ向けます。'),
    (0, 1, '鼻先を真上へ向けます。'),
    (0, -1, '鼻先を真下へ向けます。'),
)
TEXTS = {
    'audio': '音声の確認です。この声が聞こえた場合だけ、ワイとエンターを押してください。聞こえなければキューで中止します。',
    'setup': 'モーター用の四十ボルトはオフにします。ジェットソンはオンです。胴体を支える人と端末操作を分担するか、姿勢を固定する支えを使います。脚や配線へ重さを掛けず、無理な向きは中止します。',
    'ready': '胴体を支え、姿勢を固定してください。静止できたらエンター。中止はキューです。合図があるまで動かさないでください。',
    'start': '読み取りを始めます。三秒待ってから十秒記録します。動かさず、支え続けてください。',
    'end': 'この向きの読み取りは終わりました。次の説明を待ってください。',
    'confirm': '六つの記録中、姿勢を固定し、センサーの取り付けと配線は変えず、モーター電源はオフのままでしたか。全て確認できた場合だけ、ワイとエンター。未確認はキューです。',
    'independent': '次は確認用に、もう一度六つの向きを作り直します。補正の計算用とは別の記録です。',
    'finish': '記録を終了します。モーター電源はオフのまま、胴体を普段の支持台へ戻してください。補正値は候補として保存し、自動では適用しません。',
}
for index, (_, _, description) in enumerate(BODY_POSES):
    TEXTS['pose_' + str(index)] = description + ' 胴体を支え、脚や配線に荷重や接触がないことを確認します。'
FLAGS = {'can_opened': False, 'motor_enable_sent': False, 'learned_targets_sent': False,
         'approved_for_runtime': False, 'automatically_applied': False}
SOURCES = ('imu.py', 'imu_capture.py', 'imu_fixed_mount_baseline.py', 'imu_calibration.py')
OPERATOR_SIGNALS = tuple(getattr(signal, name) for name in ('SIGINT', 'SIGTERM', 'SIGHUP')
                         if hasattr(signal, name))


class SessionCancelled(InterruptedError):
    """Operator signal; must pass through child cancellation and IMU restoration."""


@contextmanager
def operator_signal_handlers():
    original = {}
    def cancel(signum, _frame):
        raise SessionCancelled('Operator signal ' + str(signum))
    try:
        for sig in OPERATOR_SIGNALS:
            original[sig] = signal.signal(sig, cancel)
        yield
    finally:
        for sig, handler in original.items():
            signal.signal(sig, handler)


@contextmanager
def deferred_operator_signals(*, replay=False):
    """Finish process creation/cleanup despite repeated operator signals.

Creation replays a pending cancellation after Popen has returned a child handle.
Cleanup already has an original exception to re-raise, so repeats are retained
only until the child has been reaped. No handler changes occur in PLAN mode.
"""
    original, pending = {}, []
    def defer(signum, _frame):
        pending.append(signum)
    try:
        for sig in OPERATOR_SIGNALS:
            original[sig] = signal.signal(sig, defer)
        yield
    finally:
        for sig, handler in original.items():
            signal.signal(sig, handler)
    if replay and pending:
        raise SessionCancelled('Operator signal ' + str(pending[0]))


def need(value, message):
    if not value:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def load_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    path = Path(path)
    need(path.is_file() and not path.is_symlink(), 'Regular file required')
    raw = path.read_bytes()
    value = json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
    return value, digest(raw)


def pose_plan(mount):
    need(type(mount) is dict, 'Mount must be an object')
    rotation = mount.get('R_body_from_sensor')
    need(type(rotation) is list and len(rotation) == 3 and all(type(row) is list and len(row) == 3
         and all(type(v) in (int, float) and v in (-1, 0, 1) for v in row) for row in rotation),
         'An explicit signed-permutation mount is required')
    need(all(sum(abs(v) for v in row) == 1 for row in rotation)
         and all(sum(abs(rotation[i][j]) for i in range(3)) == 1 for j in range(3)),
         'Mount axes are not orthogonal')
    a, b, c = rotation
    determinant = a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    need(determinant == 1, 'Right-handed mount required')
    poses = []
    for index, (axis, sign, description) in enumerate(BODY_POSES):
        sensor_axis = next(j for j in range(3) if rotation[axis][j])
        sensor_sign = sign * rotation[axis][sensor_axis]
        poses.append({'body_axis_up': 'xyz'[axis] + ('+' if sign == 1 else '-'),
                      'sensor_face': 'xyz'[sensor_axis] + ('+' if sensor_sign == 1 else '-'),
                      'audio_key': 'pose_' + str(index), 'description': description})
    return poses


def source_hashes(runtime):
    result = {'imu_six_face_session.py': digest(Path(__file__).read_bytes())}
    for name in SOURCES:
        path = runtime / 'singularitydog_hw' / name
        need(path.is_file() and not path.is_symlink(), 'Capture source missing: ' + name)
        result[name] = digest(path.read_bytes())
    return result


def capture_command(python, output, face):
    return [str(python), '-B', '-m', 'singularitydog_hw.imu_capture', '--execute',
            '--seconds', '10', '--settle-seconds', '3', '--face', face, '--output', str(output)]


def save(path, value):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def file_sha256(path):
    path = Path(path)
    need(path.is_file() and not path.is_symlink(), 'Regular capture file required: ' + str(path))
    return digest(path.read_bytes())


def bind_capture(receipt, directory, audited):
    provenance = audited.get('provenance')
    need(type(provenance) is dict, 'Audited capture provenance required')
    files = {str(receipt): file_sha256(receipt)}
    for name, key in (('summary.json', 'summary_sha256'), ('events.jsonl', 'events_sha256')):
        expected = provenance.get(key)
        need(type(expected) is str and len(expected) == 64
             and all(v in '0123456789abcdef' for v in expected), 'Capture SHA missing: ' + key)
        path = directory / name
        need(file_sha256(path) == expected, 'Capture changed after audit: ' + name)
        files[str(path)] = expected
    return files


def verify_capture_bindings(bindings):
    for files in bindings:
        for path, expected in files.items():
            need(file_sha256(path) == expected, 'Confirmed capture or receipt changed: ' + path)


def response(prompt, *, yes, read=input, write=print):
    while True:
        write(prompt)
        answer = read().strip().lower()
        if answer in ('q', 'quit', 'n', 'no', 'いいえ', '中止'):
            raise ValueError('Operator cancelled')
        if (yes and answer in ('y', 'yes', 'はい')) or (not yes and answer == ''):
            return
        write('確認欄は y とEnter、静止欄はEnterだけです。中止は q。')


def run_session(output, poses, *, announce, capture, audit, fit, verify,
                read=input, write=print):
    """Callbacks isolate device access; failure preserves every finished capture."""
    state = {'schema': 'singularitydog.imu-six-face-session.v1', 'status': 'STARTED',
             'started_at': datetime.now(timezone.utc).isoformat(), 'captures': [],
             'capture_file_bindings': [], 'operator_confirmed_partitions': [], 'errors': [], **FLAGS}
    try:
        verify()
        announce('audio')
        response('音声が聞こえた場合だけ y＋Enter。聞こえない／中止は q。', yes=True, read=read, write=write)
        announce('setup')
        response('40V Off・胴体の支持と操作の分担／固定・配線非接触を確認したら y＋Enter。',
                 yes=True, read=read, write=write)
        state['operator_confirmed_motor_power_off_and_support'] = True
        paths = {'fit': {}, 'validation': {}}
        for partition in paths:
            if partition == 'validation':
                announce('independent')
            for index, pose in enumerate(poses):
                verify()
                write(f'{partition} {index+1}/6: {pose["description"]} センサー面候補 {pose["sensor_face"]}')
                announce(pose['audio_key']); announce('ready')
                response('静止できたらEnter、中止は q。', yes=False, read=read, write=write)
                verify(); announce('start')
                destination = output / (partition + '-' + pose['sensor_face'])
                capture(destination, pose['sensor_face'])
                verify()
                audited = audit(destination, pose['sensor_face'])
                row = {'partition': partition, **pose, 'directory': str(destination), **audited}
                receipt = output / (partition + '-' + pose['sensor_face'] + '-receipt.json')
                save(receipt, row)
                state['capture_file_bindings'].append(bind_capture(receipt, destination, audited))
                state['captures'].append(row)
                paths[partition][pose['sensor_face']] = destination
                announce('end')
            announce('confirm')
            response('6姿勢とも静止・取付不変・40V Offを確認したら y＋Enter。未確認は q。',
                     yes=True, read=read, write=write)
            state['operator_confirmed_partitions'].append(partition)
        verify()
        verify_capture_bindings(state['capture_file_bindings'])
        result = fit(paths['fit'], paths['validation'])
        need(result.get('approved_for_runtime') is False and result.get('automatically_applied') is False,
             'Only an unapproved offline candidate may be saved')
        verify()
        verify_capture_bindings(state['capture_file_bindings'])
        save(output / 'calibration-candidate.json', result)
        state.update(status='SIX_FACE_CANDIDATE_REVIEW_REQUIRED',
                     candidate_sha256=digest((output / 'calibration-candidate.json').read_bytes()))
    except (Exception, KeyboardInterrupt) as error:
        state['status'] = 'INCOMPLETE_SIX_FACE_SESSION'
        state['errors'].append(type(error).__name__ + ': ' + str(error))
    finally:
        write('40VはOffのまま、胴体を普段の支持台へ戻してください。補正は自動適用していません。')
        try:
            announce('finish')
        except (Exception, KeyboardInterrupt) as error:
            state['status'] = 'INCOMPLETE_SIX_FACE_SESSION'
            state['errors'].append('Finish audio failed: ' + str(error))
        state['completed_at'] = datetime.now(timezone.utc).isoformat()
        save(output / 'session.json', state)
    return state


def audio_player(manifest_path):
    manifest_path = Path(manifest_path).resolve()
    manifest, original_sha = load_json(manifest_path)
    need(type(manifest) is dict and set(manifest) == {'schema', 'device', 'clips'}
         and manifest['schema'] == 'singularitydog.imu-six-face-audio.v1'
         and manifest['device'] == 'plughw:CARD=APE,DEV=0'
         and type(manifest['clips']) is dict and set(manifest['clips']) == set(TEXTS),
         'Complete IMU instruction audio manifest required')
    for key, text in TEXTS.items():
        clip = manifest['clips'][key]
        need(type(clip) is dict and set(clip) == {'text', 'file', 'sha256'} and clip['text'] == text,
             'Audio text differs: ' + key)
        need(type(clip['file']) is str and Path(clip['file']).name == clip['file']
             and clip['file'].endswith('.wav'), 'Audio must be a local WAV filename')
        path = manifest_path.parent / clip['file']
        need(path.is_file() and not path.is_symlink() and path.stat().st_size <= 12*1024*1024
             and digest(path.read_bytes()) == clip['sha256'], 'Audio bytes differ: ' + key)
    def play(key):
        need(load_json(manifest_path)[1] == original_sha, 'Audio manifest changed')
        clip = manifest['clips'][key]; path = manifest_path.parent / clip['file']
        need(digest(path.read_bytes()) == clip['sha256'], 'Audio changed')
        with wave.open(str(path), 'rb') as wav:
            duration = wav.getnframes() / wav.getframerate()
            need(wav.getsampwidth() == 2 and wav.getcomptype() == 'NONE' and .2 <= duration <= 60,
                 'Bounded PCM16 audio required')
        subprocess.run(['/usr/bin/aplay', '-q', '-D', manifest['device'], str(path)],
                       check=True, timeout=duration+5)
    return play


def wait_through_interrupts(child, timeout=None):
    deadline = time.monotonic() + timeout if timeout is not None else None
    while True:
        remaining = max(0, deadline-time.monotonic()) if deadline is not None else None
        try:
            return child.wait(timeout=remaining)
        except (KeyboardInterrupt, InterruptedError):
            # Signal handlers normally defer these during cleanup. Retrying also
            # handles a Python interrupt already queued before handler changes.
            if deadline is not None and time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired('IMU restoration wait', timeout)


def cancel_capture_child(child):
    with deferred_operator_signals():
        try:
            child.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass  # Already exited; still reap below.
        try:
            wait_through_interrupts(child, timeout=10)
        except subprocess.TimeoutExpired:
            try:
                child.kill()
            except ProcessLookupError:
                pass
            # Never announce a new pose or return while the child is unreaped.
            wait_through_interrupts(child)


def run_capture(command, env, log_path):
    child = None
    with log_path.open('x') as log:
        try:
            with deferred_operator_signals(replay=True):
                child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL, start_new_session=True)
            code = child.wait(timeout=35)
        except BaseException:
            if child is not None:
                cancel_capture_child(child)
            raise
    need(code == 0, 'IMU capture failed; inspect ' + str(log_path))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mount', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, default=RUNTIME)
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--audio-manifest', type=Path)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args(argv)
    mount, mount_sha = load_json(args.mount)
    poses = pose_plan(mount)
    runtime = args.runtime_root.resolve()
    pins = source_hashes(runtime)
    plan = {'status': 'PLAN_ONLY', 'poses': poses, 'passes': ['fit', 'validation'],
            'settle_seconds_per_pose': 3, 'capture_seconds_per_pose': 10,
            'mount_sha256': mount_sha, 'source_sha256': pins,
            'face_mapping': 'selected mount hypothesis; operator pose assertions; not independently verified',
            'audio_texts': TEXTS, **FLAGS}
    if not args.record:
        print(json.dumps(plan, ensure_ascii=False, indent=2)); return 0
    need(sys.platform.startswith('linux') and sys.stdin.isatty() and sys.stdout.isatty(),
         'Record requires a real Linux operator terminal')
    need(args.output_root is not None and args.audio_manifest is not None, 'Private output root and audio required')
    root = args.output_root.expanduser().absolute()
    need(root.is_dir() and not any(p.is_symlink() or (p/'.git').exists() for p in (root, *root.parents)),
         'Existing private nonsymlink output directory required')
    announce = audio_player(args.audio_manifest)
    sys.path.insert(0, str(runtime))
    from singularitydog_hw import imu_fixed_mount_baseline as baseline, imu_calibration as calibration
    for module in (baseline, calibration):
        need(Path(module.__file__).resolve().parent == runtime/'singularitydog_hw', 'Wrong imported runtime')
    output = root / ('imu-six-face-' + str(uuid.uuid4())); output.mkdir(mode=0o700)
    save(output/'plan.json', plan)
    env = dict(os.environ, PYTHONPATH=str(runtime), PYTHONDONTWRITEBYTECODE='1')
    def verify():
        need(source_hashes(runtime) == pins and load_json(args.mount)[1] == mount_sha, 'Source or mount changed')
    def capture(destination, face):
        run_capture(capture_command(sys.executable, destination, face), env,
                    output/(destination.name+'.log'))
    def audit(destination, face):
        metadata, _, stats, origin = baseline._load_capture(destination, expected_face=face)
        need(metadata['source_sha256'] == {name:pins[name] for name in ('imu.py','imu_capture.py')},
             'Capture used different sources')
        # Catch a gross wrong pose before asking for the next one. The final
        # fit checks all sample, variance, span and independent-holdout gates.
        axis = 'xyz'.index(face[0]); sign = 1 if face[1] == '+' else -1
        mean = stats['accel_mean_m_s2']
        need(sign*mean[axis] > max(abs(mean[j]) for j in range(3) if j != axis),
             'Recorded dominant axis differs from the selected mount/pose; do not relabel raw data')
        return {'provenance':origin,'samples':stats['samples'],'accel_mean_m_s2':mean}
    with operator_signal_handlers():
        result = run_session(output, poses, announce=announce, capture=capture, audit=audit,
            fit=lambda fit, check: calibration.estimate_capture_directories(fit,
                validation_face_paths=check, operator_confirmed_stationary=True), verify=verify)
    print(json.dumps({'output':str(output),'status':result['status'],'errors':result['errors'],**FLAGS},ensure_ascii=False))
    return 0 if result['status'] == 'SIX_FACE_CANDIDATE_REVIEW_REQUIRED' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({'status':'SIX_FACE_NOT_STARTED','error':str(error),**FLAGS},ensure_ascii=False))
        raise SystemExit(2)
