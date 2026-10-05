"""Human spirit-level observations, with an injected completed audio callback.

Default PLAN/help/template never invokes audio or opens an output file. Actual
audio use is through main(argv, announce=callback); the caller pins its assets
and waits for playback. Direct text use requires --record --text-only. This
module contains no audio/device/network command. The only physical input is
what the operator enters. Timestamps are host input-receipt times, not calibrated
physical observation times. The envelope keeps notes/times apart from the exact
frozen analyzer schema. A saved interaction never approves joint precision.
The default asks only for what the bubble looks like; no number is inferred.
Numeric intervals require --numeric-readings or visual_only=False explicitly.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import types
import uuid

ANALYZER_SHA256 = 'f961465ebe919b5ec0445c360e51672b9762f1d07a9400eab97976f6842efed5'
SCHEMA = 'singularitydog.level-reversal-interaction.v1'
MAX_CYCLES, MAX_ATTEMPTS, MAX_INPUT, MAX_BYTES = 12, 8, 2048, 1024*1024
FALSE_FLAGS = dict.fromkeys(('motor_output_allowed', 'approved_for_runtime', 'calibration_approved',
    'profile_changed', 'existing_thresholds_changed', 'physical_stationarity_proven',
    'joint_datum_verified', 'uncertainty_inferred_from_repeatability', 'motor_hardware_opened'), False)
AUDIO_TEXTS = {
    'audio_test': '水準器は一台で大丈夫です。この音声が聞こえたら、ワイを入力してください。聞こえなければキューで中止してください。',
    'instructions': 'これから向きを変えて、三回、気泡を見ます。中央、端寄り、水平です、など見えた言葉をそのまま入力してください。見えない場合は、見えないで大丈夫です。途中でやめるときはキューを入力してください。',
    'setup': '机など、安定した場所に水準器を置いてください。三回とも同じ場所で、同じ底面を使います。机や対象を動かさず、気泡が止まるのを待ってください。',
    'forward': '一回目です。水準器を置いて、気泡が止まったら見てください。見えた言葉を一回入力してください。',
    'reverse': '二回目です。水準器だけを、机の上で百八十度、半回転させてください。左右の端を入れ替えます。上下は裏返しません。同じ場所に置き、気泡が止まったら、見えた言葉を入力してください。',
    'return': '三回目です。水準器を最初の向きへ戻し、同じ場所に置いてください。気泡が止まったら、見えた言葉を入力してください。',
    'finish': '記録はここまでです。全部の音声を聞けたら、ワイを入力してください。聞けなかった合図があればキューで中止してください。この記録だけでは、正確な角度や原点の誤差はまだわかりません。',
}
CONDITION_LABELS = ('同じ場所', '同じ底面', '机や対象を動かしていない', '気泡が止まるのを待った')


class Cancelled(ValueError):
    pass


def need(condition, message):
    if not condition:
        raise ValueError(message)


def pinned_analyzer():
    path = Path(__file__).with_name('analyze_level_reversal.py').absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Analyzer symlink refused')
    raw = path.read_bytes()
    need(hashlib.sha256(raw).hexdigest() == ANALYZER_SHA256, 'Frozen analyzer source changed')
    namespace = {'__name__': 'pinned_level_reversal_analyzer', '__file__': str(path)}
    exec(compile(raw, str(path), 'exec'), namespace)
    return types.SimpleNamespace(**namespace)


def source_hashes():
    return {'record_level_reversal.py': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'analyze_level_reversal.py': ANALYZER_SHA256}


def timestamp(now):
    value = now()
    need(isinstance(value, datetime.datetime) and value.tzinfo is not None,
         'Timezone-aware host datetime required')
    return value.astimezone(datetime.timezone.utc).isoformat()


def directory(value):
    path = Path(value)
    need(path.is_absolute(), 'Output root must be an absolute path')
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Output symlink refused')
    need(not any((p/'.git').exists() for p in (path, *path.parents)), 'Private output must stay outside Git')
    need(path.is_dir(), 'Output root must already be a directory')
    return path


class ReservedJSON:
    """One fresh 0600 UUID file. A failure never produces a successful receipt."""
    def __init__(self, root, tag):
        root = directory(root)
        need(str(uuid.UUID(tag)) == tag, 'Canonical UUID required')
        self.root, self.path = root, root/('level-reversal-'+tag+'.json')
        self.fd = self.parent_fd = None
        try:
            self.parent_fd = os.open(root, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | getattr(os, 'O_NOFOLLOW', 0))
            info = os.fstat(self.parent_fd); self.parent_identity = (info.st_dev, info.st_ino)
            self.fd = os.open(self.path.name, os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0),
                              0o600, dir_fd=self.parent_fd)
            os.fchmod(self.fd, 0o600); info = os.fstat(self.fd); self.identity = (info.st_dev, info.st_ino)
            self.binding()
        except BaseException:
            self.close(); raise

    def binding(self):
        directory(self.root)
        parent, named_parent = os.fstat(self.parent_fd), self.root.lstat()
        opened, named = os.fstat(self.fd), self.path.lstat()
        need((parent.st_dev, parent.st_ino) == self.parent_identity == (named_parent.st_dev, named_parent.st_ino), 'Output parent changed')
        need(stat.S_ISREG(named.st_mode) and (opened.st_dev, opened.st_ino) == self.identity == (named.st_dev, named.st_ino)
             and stat.S_IMODE(named.st_mode) == 0o600, 'Reserved output changed')

    def close(self):
        errors = []
        for name in ('fd', 'parent_fd'):
            fd = getattr(self, name, None)
            if fd is not None:
                setattr(self, name, None)
                try: os.close(fd)
                except OSError as error: errors.append(error)
        if errors: raise errors[0]

    def save(self, value):
        raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        need(0 < len(raw) <= MAX_BYTES, 'Interaction artifact exceeds bound')
        self.binding(); offset = 0
        while offset < len(raw):
            n = os.write(self.fd, raw[offset:]); need(type(n) is int and 0 < n <= len(raw)-offset, 'Write made no progress'); offset += n
        os.fsync(self.fd); self.binding()
        need(os.pread(self.fd, len(raw)+1, 0) == raw, 'Saved bytes changed')
        os.close(self.fd); self.fd = None
        os.fsync(self.parent_fd); os.close(self.parent_fd); self.parent_fd = None
        directory(self.root)
        info = self.path.lstat()
        need(not self.path.is_symlink() and (info.st_dev, info.st_ino) == self.identity
             and stat.S_ISREG(info.st_mode) and stat.S_IMODE(info.st_mode) == 0o600, 'Final output binding changed')
        fd = os.open(self.path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | os.O_NONBLOCK)
        try:
            opened = os.fstat(fd)
            need((opened.st_dev, opened.st_ino) == self.identity and opened.st_size == len(raw), 'Final output inode/size changed')
            saved = os.read(fd, len(raw)+1)
            need(saved == raw, 'Final saved bytes changed')
        finally:
            os.close(fd)
        return hashlib.sha256(raw).hexdigest()


def record_session(analyzer, *, cycles, text_only, announce, read, write, now, synthetic=False, reading_unit='marked_arbitrary_units', visual_only=True):
    need(type(cycles) is int and 1 <= cycles <= MAX_CYCLES, 'Cycle count must be 1..12')
    need(type(text_only) is bool and callable(read) and callable(write) and callable(now), 'Invalid interaction callbacks')
    need(type(visual_only) is bool, 'visual_only must be boolean')
    need(text_only or callable(announce), 'Audio callback required; use explicit text-only mode otherwise')
    record = analyzer.template(); record['record_kind'] = 'SYNTHETIC_FIXTURE' if synthetic else 'RECORDED_READINGS_UNREVIEWED'
    record['reading_unit'] = reading_unit; record['cycles'] = []
    bundle = {'schema': SCHEMA, 'status': 'INCOMPLETE', **FALSE_FLAGS,
        'records': record, 'interaction_source': 'INJECTED_CALLBACK_SYNTHETIC' if synthetic else 'LIVE_STDIN_TTY',
        'recording_mode': 'TEXT_ONLY_EXPLICIT' if text_only else 'AUDIO_CALLBACK', 'audio_callback_invoked': False,
        'reading_mode': 'VISUAL_WORDS_ONLY' if visual_only else 'EXPLICIT_NUMERIC_INTERVALS',
        'audio_test_heard_confirmed': None, 'all_work_audio_heard_confirmed': None,
        'started_at_utc': timestamp(now), 'completed_at_utc': None,
        'events': [], 'observations': [], 'completed_cycles': 0, 'errors': [],
        'absolute_origin_error_rad': None, 'absolute_origin_uncertainty_rad': None,
        'physical_observation_timestamp_independently_known': False,
        'timestamp_scope': 'Host timestamps when the operator input was received; physical observation latency is unknown.',
        'report_finalization_claimed': False,
        'persistence_scope': 'Require the separate final CLI receipt with matching saved artifact SHA256.'}

    def event(kind, **values):
        need(len(bundle['events']) < 1024, 'Interaction event bound exceeded')
        bundle['events'].append({'kind': kind, 'host_recorded_at_utc': timestamp(now), **values})

    def ask(prompt, validator):
        write(prompt)
        for _ in range(MAX_ATTEMPTS):
            raw = read(); need(type(raw) is str and len(raw) <= MAX_INPUT, 'Input must be bounded text')
            event('operator_input', prompt=prompt, raw_input=raw)
            token = raw.strip().lower()
            if token in ('q', 'quit', '中止'): raise Cancelled('Operator cancelled')
            try: return validator(raw)
            except Cancelled: raise
            except ValueError as error: write(str(error)+' 空Enterでは進みません。中止は q。')
        raise Cancelled('Too many invalid or empty answers')

    def yes(raw):
        token = raw.strip().lower()
        if token in ('n', 'no', 'いいえ'): raise Cancelled('Operator declined confirmation')
        need(token in ('y', 'yes', 'はい'), '確認した場合だけ y。')
        return True

    def label(raw):
        value = raw.strip()
        need(value and len(value) <= 256, '名称を入力。不明なら u。')
        return 'UNKNOWN' if value.lower() in ('u', 'unknown', '不明') else value

    def spoken(key):
        text = AUDIO_TEXTS[key]; write(text)
        if not text_only:
            event('audio_callback_started', key=key, text_sha256=hashlib.sha256(text.encode()).hexdigest())
            bundle['audio_callback_invoked'] = True
            result = announce(text)
            need(result is None or type(result) is dict, 'Audio callback receipt must be a JSON object or None')
            if result is not None: need(len(json.dumps(result, allow_nan=False).encode()) <= 4096, 'Audio receipt exceeds bound')
            event('audio_callback_returned', key=key, callback_receipt=result)

    def conditions(raw):
        tokens = raw.strip().lower().split()
        keys = analyzer.CONDITIONS[:4]
        result = dict.fromkeys(analyzer.CONDITIONS)
        if len(tokens) == 1 and tokens[0] in ('y', 'n', 'u', 'unknown', '不明'):
            # A single n means the combined conditions were not met. It does
            # not identify which condition failed, so none is marked false.
            if tokens[0] == 'y': result.update(dict.fromkeys(keys, True))
            return result
        need(len(tokens) == 4 and all(t in ('y', 'n', 'u') for t in tokens), '全部できたら y、できなかったら n、不明なら u。個別は4個の y/n/u。')
        result.update({k: {'y': True, 'n': False, 'u': None}[t] for k, t in zip(keys, tokens)})
        return result

    def visual_note(raw):
        need(raw.strip() and raw.strip().lower() not in ('y', 'yes', 'はい', 'n', 'no', 'いいえ', 'ok'),
             '中央・端寄り・水平です・見えないなど、見えた言葉を入力してください。')
        return raw

    def reading(raw):
        token = raw.strip().lower()
        if token in ('u', 'unknown', '不明'): return None
        parts = token.split(); need(len(parts) == 2, '印の端を＋として 下限 上限 を入力。数値不明なら u。')
        values = [float(p) for p in parts]
        return analyzer.interval(values)

    def note(raw):
        need(raw.strip() and raw.strip().lower() not in ('y', 'yes', 'はい', 'n', 'no', 'u', 'unknown', 'ok'),
             '中央・端寄りなど、実際に見た様子を文章で入力。')
        return raw

    try:
        if text_only:
            write('明示した文字のみモードです。音声を聞いた確認は作成しません。')
            ask('作業説明を読んで記録を始める場合だけ y。中止は q。', yes)
        else:
            spoken('audio_test')
            bundle['audio_test_heard_confirmed'] = ask('開始音声が聞こえた場合だけ y。聞こえなければ n/q。', yes)
        spoken('instructions')
        datum = patch = face = 'UNKNOWN'
        if not visual_only:
            write('明示した数値モードです。水準器の片端に印を決め、その端へ気泡が寄る向きをプラスとします。反転後も同じ印の端がプラスです。画面の左右ではありません。')
            record['instrument_id'] = ask('水準器の識別名を入力。不明なら u。', label)
            datum = ask('当てる対象・基準面の名前を入力。関節軸との対応が不明なら u。', label)
            patch = ask('同じ位置を識別する印・場所を入力。不明なら u。', label)
            face = ask('使う水準器の測定面を入力。不明なら u。', label)
        for index in range(cycles):
            spoken('setup')
            cycle = {'id': 'cycle-'+str(index+1), 'datum_id': datum, 'contact_patch_id': patch,
                'measuring_face_id': face, 'conditions': dict.fromkeys(analyzer.CONDITIONS),
                'readings': [{'orientation': o, 'displacement_interval': None} for o in ('forward', 'reverse', 'forward')]}
            record['cycles'].append(cycle)
            for row, key in zip(cycle['readings'], ('forward', 'reverse', 'return')):
                spoken(key)
                if visual_only:
                    value = None
                    actual_note = ask('見えた言葉を入力してください（中央、端寄り、水平です、0、見えない）。中止は q。', visual_note)
                else:
                    value = ask('気泡単位のsigned 下限 上限。数値を測っていなければ u。中止 q。', reading)
                    row['displacement_interval'] = value
                    actual_note = ask('実際に見た気泡と置き方を、そのまま文章で入力。空Enter不可、中止 q。', note)
                observation = {'cycle_id': cycle['id'], 'orientation': row['orientation'], 'stage': key,
                    'displacement_interval': value, 'observation_note_original': actual_note,
                    'host_input_received_at_utc': timestamp(now), 'physical_observation_at_utc': None,
                    'physical_observation_timestamp_independently_known': False}
                bundle['observations'].append(observation); event('observation_recorded', cycle_id=cycle['id'], stage=key)
            cycle['conditions'] = ask('今終えた3回で、'+ '・'.join(CONDITION_LABELS)+'。全部できたら y、できなかったら n、不明なら u。中止 q。', conditions)
            bundle['completed_cycles'] += 1
        if not text_only:
            spoken('finish')
            bundle['all_work_audio_heard_confirmed'] = ask('全作業音声を聞けた場合だけ y。一つでも聞けなければ n/q。', yes)
        analyzer.analyze(record)
        bundle['status'] = ('COMPLETED_VISUAL_OBSERVATION_INTERACTION_REVIEW_REQUIRED' if visual_only
                            else 'COLLECTED_LEVEL_OBSERVATIONS_REVIEW_REQUIRED')
    except BaseException as error:
        bundle['status'] = 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS'
        bundle['errors'].append(type(error).__name__+': '+str(error)[:512])
    bundle['completed_at_utc'] = timestamp(now)
    return bundle


def main(argv=None, *, announce=None, read=None, write=None, now=None, visual_only=True):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--record', action='store_true'); mode.add_argument('--plan', action='store_true'); mode.add_argument('--template', action='store_true')
    parser.add_argument('--output-root'); parser.add_argument('--cycles', type=int, choices=range(1, MAX_CYCLES+1), default=1)
    parser.add_argument('--text-only', action='store_true')
    reading_mode = parser.add_mutually_exclusive_group()
    reading_mode.add_argument('--visual-only', action='store_true', help='Accept visual words only, without numeric inference (default)')
    reading_mode.add_argument('--numeric-readings', action='store_true', help='Explicit legacy numeric intervals and notes; default accepts only visual words')
    parser.add_argument('--reading-unit', choices=('vial_divisions', 'marked_arbitrary_units'), default='marked_arbitrary_units')
    args = parser.parse_args(argv); writer = None; output = None
    try:
        need(type(visual_only) is bool, 'visual_only must be boolean')
        visual_only = True if args.visual_only else False if args.numeric_readings else visual_only
        analyzer = pinned_analyzer(); pins = source_hashes()
        if args.template:
            print(json.dumps(analyzer.template(), indent=2, allow_nan=False)); return 0
        if not args.record:
            print(json.dumps({'status': 'PLAN_ONLY', **FALSE_FLAGS, 'audio_callback_invoked': False,
                'cycles': args.cycles, 'reading_unit': args.reading_unit, 'recording_mode': 'TEXT_ONLY_EXPLICIT' if args.text_only else 'AUDIO_CALLBACK',
                'reading_mode': 'VISUAL_WORDS_ONLY' if visual_only else 'EXPLICIT_NUMERIC_INTERVALS',
                'analyzer_source_sha256': ANALYZER_SHA256, 'source_sha256': pins,
                'requires_live_stdin_or_explicit_synthetic_read_callback': True,
                'fresh_output_created': False, 'absolute_origin_error_rad': None}, allow_nan=False)); return 0
        need(args.output_root is not None, '--record requires an absolute private --output-root')
        need(args.text_only or callable(announce), 'Audio callback missing; use the pinned audio launcher or explicit --text-only')
        synthetic = read is not None
        need(synthetic or sys.stdin.isatty(), 'Actual record requires live TTY stdin')
        now = now or (lambda: datetime.datetime.now(datetime.timezone.utc))
        write = write or (lambda text: print(text, file=sys.stderr, flush=True))
        read = read or input
        tag = str(uuid.uuid4()); writer = ReservedJSON(args.output_root, tag); output = str(writer.path)
        bundle = record_session(analyzer, cycles=args.cycles, text_only=args.text_only, announce=announce,
            read=read, write=write, now=now, synthetic=synthetic, reading_unit=args.reading_unit, visual_only=visual_only)
        bundle['session_uuid'] = tag; bundle['source_sha256'] = pins
        if source_hashes() != pins or hashlib.sha256(Path(__file__).with_name('analyze_level_reversal.py').read_bytes()).hexdigest() != ANALYZER_SHA256:
            bundle['status'] = 'ABORTED_PARTIAL_LEVEL_OBSERVATIONS'; bundle['errors'].append('Source changed during recording')
        artifact_sha = writer.save(bundle)
        complete = bundle['status'] in ('COLLECTED_LEVEL_OBSERVATIONS_REVIEW_REQUIRED',
                                        'COMPLETED_VISUAL_OBSERVATION_INTERACTION_REVIEW_REQUIRED')
        saved_status = ('SAVED_VISUAL_OBSERVATION_INTERACTION_REVIEW_REQUIRED' if visual_only
                        else 'SAVED_LEVEL_REVERSAL_REVIEW_REQUIRED') if complete else 'SAVED_ABORTED_LEVEL_REVERSAL'
        print(json.dumps({'status': saved_status,
            **FALSE_FLAGS, 'output': output, 'artifact_sha256': artifact_sha, 'errors': bundle['errors'],
            'completed_cycles': bundle['completed_cycles'], 'numeric_readings': sum(row['displacement_interval'] is not None for c in bundle['records']['cycles'] for row in c['readings']),
            'observation_notes': len(bundle['observations']), 'synthetic_interaction': synthetic,
            'reading_mode': bundle['reading_mode'],
            'absolute_origin_error_rad': None, 'absolute_origin_uncertainty_rad': None}, allow_nan=False), flush=True)
        return 0 if complete else 1
    except (OSError, ValueError, TypeError, OverflowError) as error:
        print(json.dumps({'status': 'INCOMPLETE_LEVEL_REVERSAL_RECORD', **FALSE_FLAGS, 'output': output,
            'artifact_sha256': None, 'error': type(error).__name__+': '+str(error),
            'absolute_origin_error_rad': None}, allow_nan=False), flush=True); return 1
    finally:
        if writer is not None: writer.close()


if __name__ == '__main__':
    raise SystemExit(main())
