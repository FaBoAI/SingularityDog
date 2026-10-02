#!/usr/bin/env python3
"""Create Japanese spoken cues on macOS, without robot/device access.

The output is an unreviewed audio candidate. It never grants motor permission.
Run with access to the macOS speech service: sandboxed say can return an empty
file with exit status zero. Exact PCM frames are checked before accepting it.
"""
import argparse
from array import array
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import wave

TEXTS = {
    'brief': 'これから、今の足の位置を最大8秒保つ、部分荷重試験をします。一人は胴体を支え続け、もう一人は電源遮断を担当してください。短いピッという合図で、支える力を少し緩めます。0.5秒以内に自分で全て支え直してください。手は離さず、停止後まで支えます。準備ができたら、開始してください。',
    'prepare_ease': 'ピッで開始してください。',
    'resupport': '全て支えてください。',
    'abort': '中止します。胴体を支えてください。',
    'preview': 'これは音声だけの確認です。今は操作しないでください。動作の説明と合図の声を確認します。',
}


def pcm_duration(path):
    with wave.open(str(path), 'rb') as source:
        if (source.getnchannels(), source.getsampwidth(), source.getframerate(),
                source.getcomptype()) != (2, 2, 48000, 'NONE'):
            raise ValueError('Expected stereo 48kHz PCM16: ' + str(path))
        frames = source.getnframes()
        pcm = source.readframes(frames)
    if frames <= 0 or len(pcm) != frames * 4 or not any(pcm):
        raise ValueError('Empty, silent or truncated generated audio: ' + str(path))
    return frames / 48000.


def tone(duration, frequency):
    values = array('h')
    count = round(duration * 48000)
    for i in range(count):
        envelope = min(1., i/120., (count-1-i)/120.)
        value = round(5000 * envelope * math.sin(2*math.pi*frequency*i/48000))
        values.extend((value, value))
    if sys.byteorder != 'little':
        values.byteswap()
    return values.tobytes()


def write_pcm(path, pcm):
    with wave.open(str(path), 'wb') as result:
        result.setnchannels(2)
        result.setsampwidth(2)
        result.setframerate(48000)
        result.writeframes(pcm)


def generate(output):
    if sys.platform != 'darwin':
        raise RuntimeError('This generator uses the macOS Japanese Kyoko voice')
    output.mkdir(parents=True, exist_ok=False)
    for stage, transcript in TEXTS.items():
        aiff = output / (stage + '.aiff')
        wav = output / (stage + '.wav')
        rate = '260' if stage in ('prepare_ease', 'resupport') else '210'
        subprocess.run(['/usr/bin/say', '-v', 'Kyoko', '-r', rate,
                        '-o', str(aiff), transcript], check=True)
        subprocess.run(['/usr/bin/afconvert', '-f', 'WAVE', '-d', 'LEI16@48000',
                        '-c', '2', str(aiff), str(wav)], check=True)
        pcm_duration(wav)  # Exit zero from say/afconvert is insufficient.
        aiff.unlink()
    write_pcm(output / 'go.wav', tone(.1, 1200))
    with wave.open(str(output / 'resupport.wav'), 'rb') as source:
        body = source.readframes(source.getnframes())
    write_pcm(output / 'resupport.wav', tone(.08, 1500) + body)
    clips = {}
    for stage in ('brief', 'prepare_ease', 'go', 'resupport', 'abort'):
        path = output / (stage + '.wav')
        clips[stage] = dict(path=path.name,
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            duration_s=pcm_duration(path), transcript=TEXTS.get(stage,
                '短い開始合図。準備音声終了と正常周期確認の後にのみ鳴らす。'))
    # Named engineering review, fresh pose/power evidence and operator
    # audibility confirmation are separate from audio generation.
    manifest = dict(schema='singularitydog.human-supported-audio-manifest.v1',
        scope='human_supported_partial_current_hold_only',
        acceptance='human-supported-partial-current-hold-audio-8s-v1',
        prepare_ease_is_not_go=True, go_is_short_tone=True,
        resupport_starts_with_urgent_tone=True,
        operator_must_resupport_before_voice_finishes=True,
        audio_process_completion_is_not_proof_of_audibility=True,
        physical_ease_duration_not_verified_by_audio=True, clips=clips,
        review=dict(decision='REVIEW_REQUIRED', reviewer='unreviewed',
            reviewed_at=datetime.now(timezone.utc).isoformat(),
            rationale='Audio generation alone does not approve a trial.'))
    (output / 'manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    preview = output / 'preview.wav'
    (output / 'generation.json').write_text(json.dumps(dict(
        voice='Kyoko', language='ja-JP',
        preview_duration_s=pcm_duration(preview),
        preview_sha256=hashlib.sha256(preview.read_bytes()).hexdigest(),
        audibility_confirmed=False, motor_command_sent=False),
        ensure_ascii=False, indent=2) + '\n')
    return output / 'manifest.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New output directory; existing recordings are never overwritten')
    args = parser.parse_args()
    print(json.dumps({'manifest': str(generate(args.output)),
                      'motor_output_allowed': False, 'review_required': True}))


if __name__ == '__main__':
    main()
