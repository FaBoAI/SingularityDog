"""Operator-moved FR position/velocity observations. No motor commands exist.

This records one joint at a time through Type0/17 only. It is not a zero/sign
calibration, a stationary gate, or permission to enable a motor.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import termios
import time

from .can_readonly import ReadOnlyCAN

IDS = (1, 2, 3)
NAMES = ("足先側", "中央（上脚）", "付け根")


def identities(value):
    if not isinstance(value, dict) or set(value) != {str(i) for i in range(1, 13)}:
        raise ValueError("Expected twelve reference identities")
    if any(not isinstance(v, str) or len(v) != 16 or
           any(c not in '0123456789abcdef' for c in v) for v in value.values()):
        raise ValueError("Malformed reference identity")
    if len(set(value.values())) != 12:
        raise ValueError("Duplicate reference identities")
    return value


def numeric_query(can, mid, parameter, check):
    check()
    r = can.query(mid, parameter)
    if can.parser.discarded_bytes or can.parser.buffer:
        raise RuntimeError('Incomplete/discarded CAN data during observation')
    if (not r.get('ok') or type(r.get('value')) not in (int, float)
            or not math.isfinite(r['value'])):
        raise RuntimeError(f"Invalid read ID{mid} {parameter}")
    check()
    if parameter == 'current' and abs(r['value']) > .05:
        raise RuntimeError(f"ID{mid} current exceeds diagnostic quiet-current threshold")
    return r


def check_feedback_event(event):
    # Save the original event before invoking this guard. Current-only reads
    # cannot prove disabled mode; any explicit contradictory feedback aborts.
    if event.get('kind') == 'motor_feedback' and (event.get('type') == 21 or
            event.get('fault_bits', 0) != 0 or event.get('mode_state', 0) != 0):
        raise RuntimeError('Received enabled or fault feedback during manual observation')


def collect(can, mid, seconds, phase, emit, check, *, clock=time.monotonic, wait=time.sleep):
    if mid not in IDS or seconds not in (3, 8) or phase not in ('before', 'moving', 'released'):
        raise ValueError("Only fixed FR manual observation windows are available")
    start, rows, next_poll = clock(), [], clock()
    # A monotonic deadline and independent iteration limit both bound the session.
    for _ in range(int(seconds * 50)):
        check()
        if clock() - start >= seconds:
            break
        values = {}
        for parameter in ('position', 'velocity', 'current'):
            values[parameter] = numeric_query(can, mid, parameter, check)
        row = {"kind": "manual_motion_sample", "motor_id": mid, "phase": phase,
               "sample_index": len(rows), "position_rad": values['position']['value'],
               "position_read_ns": values['position']['monotonic_ns'],
               "velocity_rad_s": values['velocity']['value'],
               "velocity_read_ns": values['velocity']['monotonic_ns'],
               "current_A": values['current']['value']}
        emit(row)
        rows.append(row)
        next_poll += .02
        wait(max(0, next_poll - clock()))
    check()
    if len(rows) < 50 or (rows[-1]['position_read_ns'] - rows[0]['position_read_ns']) / 1e9 < seconds - .3:
        raise RuntimeError("Insufficient observation duration/samples")
    return rows


def metrics(rows):
    if len(rows) < 2:
        raise ValueError("Need at least two samples")
    t = [(r['position_read_ns'] - rows[0]['position_read_ns']) / 1e9 for r in rows]
    p = [r['position_rad'] for r in rows]
    v = [r['velocity_rad_s'] for r in rows]
    if any(b <= a for a, b in zip(t, t[1:])):
        raise ValueError("Position timestamps must increase")
    if not all(math.isfinite(n) for n in t + p + v):
        raise ValueError("Nonfinite observations")
    tm, pm = statistics.fmean(t), statistics.fmean(p)
    return {"samples": len(rows), "duration_s": t[-1],
            "position_range_deg": math.degrees(max(p) - min(p)),
            "position_net_change_deg": math.degrees(p[-1] - p[0]),
            "position_unique_values": len(set(p)),
            "position_OLS_rad_s": sum((a-tm)*(b-pm) for a,b in zip(t,p))/sum((a-tm)**2 for a in t),
            "velocity_mean_rad_s": statistics.fmean(v),
            "velocity_RMS_rad_s": math.sqrt(statistics.fmean(x*x for x in v)),
            "max_abs_velocity_rad_s": max(map(abs,v)),
            "max_position_gap_s": max(b-a for a,b in zip(t,t[1:])),
            "approved_for_runtime": False, "stationarity_pass": None}


def fresh_input(prompt):
    termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    return input(prompt).strip().lower()


def confirm_observed_motion(mid, check):
    """Retry only the answer; never repeat or silently approve a measurement."""
    while True:
        check()
        value = fresh_input(f'ID{mid}の動きを目で確認できましたか。y=確認できた / n=判別できない / q=終了 [文字を入力してEnter]: ')
        check()
        if value in ('y', 'n'):
            return value == 'y'
        if value == 'q':
            raise InterruptedError('Operator quit at movement confirmation')
        print('Enterだけでは確認になりません。y または n を入力してEnter。取得済みデータは保持しています。', flush=True)


def wait_for_ready(prompt, check):
    """Accept Enter or explicit yes; other input never discards prior records."""
    while True:
        check()
        value = fresh_input(prompt)
        check()
        if value in ('', 'y'):
            return
        if value == 'q':
            raise InterruptedError('Operator quit')
        print('準備ができたらEnter（yでも可）、終了はq。記録はまだ始めていません。', flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-readonly', action='store_true')
    ap.add_argument('--expected-uids', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--motor-id', type=int, choices=IDS,
                    help='Record only one FR joint; previous sessions are not merged or approved')
    args = ap.parse_args(argv)
    selected = (args.motor_id,) if args.motor_id is not None else IDS
    plan = {'ids': list(selected), 'before_seconds_per_joint': 3, 'moving_seconds_per_joint': 8,
            'released_seconds_per_joint': 3, 'allowed_can_types': [0, 17],
            'motor_output_available': False, 'calibration_applied': False}
    if not args.execute_readonly:
        print(json.dumps(plan, indent=2))
        return 0
    if not sys.stdin.isatty():
        ap.error('Execute from an interactive terminal; no piped confirmations')
    expected = identities(json.loads(args.expected_uids.read_text()))
    out = args.output.expanduser().resolve()
    if any((p/'.git').exists() for p in (out,*out.parents)):
        ap.error('Private data must be outside Git')
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    interrupted, handlers = [], {}
    result = {'started_at': datetime.datetime.now().astimezone().isoformat(),
              'plan': plan, 'boot_id': boot, 'joints': {}, 'errors': [],
              'status': 'INCOMPLETE', 'approved_for_runtime': False,
              'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (Path(__file__), Path(__file__).with_name('can_readonly.py'))}}
    def check():
        if interrupted:
            raise InterruptedError('Operator interrupted recording')
        if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != boot:
            raise RuntimeError('Boot changed; do not join observations across boots')
    def stop(sig, _):
        interrupted.append(sig)
        raise InterruptedError('Operator interrupted recording')
    def enter(prompt):
        wait_for_ready(prompt, check)
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, stop)
        lock_dir = Path.home()/'.cache'/'singularitydog'
        lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        with (lock_dir/'manual-calibration.lock').open('a+') as lock, \
             (lock_dir/'can-readonly.lock').open('a+') as can_lock, \
             (out/'events.jsonl').open('x', buffering=1) as log:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(can_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            def emit(event):
                log.write(json.dumps({'wall_time_ns': time.time_ns(), **event}, allow_nan=False)+'\n')
                check_feedback_event(event)
            emit({'kind':'manual_motion_plan', **plan})
            print('右前脚だけを手で確認します。自動駆動・原点変更は行いません。', flush=True)
            print('支持台上で脚が自由に動き、モーターが脱力していることを確認。別の駆動ツールは終了。', flush=True)
            print('抵抗や引っ掛かりがあれば無理に動かさず q / Ctrl+C。L字にする必要はありません。', flush=True)
            for mid in selected:
                name = NAMES[mid-1]
                enter(f'\nID{mid} 右前脚の{name}：手を離して Enter。3秒静止記録した後、合図で8秒手動往復 [Enterまたはy / 終了q]: ')
                with ReadOnlyCAN(event_sink=emit) as can:
                    for i in IDS:
                        check()
                        r = can.query(i)
                        if not r.get('ok') or r.get('mcu_uid_hex') != expected[str(i)]:
                            raise RuntimeError(f'ID{i} identity mismatch')
                        numeric_query(can, i, 'current', check)
                    print('最初の3秒：手を離したままお待ちください。', flush=True)
                    before = collect(can, mid, 3, 'before', emit, check)
                    print('記録開始：指定した関節を手で5〜10°程度、ゆっくり往復してください（8秒）', flush=True)
                    moving = collect(can, mid, 8, 'moving', emit, check)
                    print('動作記録完了。手を離し、自然に落ち着くまで待ってください。', flush=True)
                    observed = confirm_observed_motion(mid, check)
                    enter('手を離して完全に落ち着いたら Enter。3秒間静止記録します [Enterまたはy / 終了q]: ')
                    released = collect(can, mid, 3, 'released', emit, check)
                    if can.parser.discarded_bytes or can.parser.buffer:
                        raise RuntimeError('Incomplete/discarded CAN data')
                entry = {'operator_observed_motion': observed,
                         'before': metrics(before), 'moving': metrics(moving), 'released': metrics(released)}
                entry['position_response_observed'] = (observed and
                    entry['moving']['position_range_deg'] >= 1 and
                    entry['moving']['position_unique_values'] >= 10)
                result['joints'][str(mid)] = entry
                (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
                print(f"保存：移動中の位置幅 {entry['moving']['position_range_deg']:.2f}° / 手離し後 {entry['released']['position_range_deg']:.3f}°", flush=True)
            result['status'] = 'RECORDED_REVIEW_REQUIRED'
    except BaseException as error:
        result['errors'].append(repr(error))
        result['status'] = 'INCOMPLETE'
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        result['completed_at'] = datetime.datetime.now().astimezone().isoformat()
        (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(f"保存先：{out}\n結果：{result['status']}。静止判定・モデル校正は自動承認していません。", flush=True)
    if result['errors']:
        print(' / '.join(result['errors']), flush=True)
    return int(bool(result['errors']))


if __name__ == '__main__':
    raise SystemExit(main())
