#!/usr/bin/env python3
"""Render measured XY paths and foot clearance as charts; never load robot/vendor geometry."""
import argparse
import json
import math
from pathlib import Path
import subprocess

from artifact_manifest import digest


def validate(data):
    if data.get('schema') != 'singularitydog.measured-motion.v1':
        raise ValueError('Wrong measured-motion schema')
    if data.get('source_sample_hz') != 50 or data.get('vendor_geometry_included') is not False:
        raise ValueError('Expected original 50 Hz measurements without geometry')
    times = data['time_s']
    if len(times) != 600 or any(abs(t - (i + 1) * .02) > 1e-8 for i, t in enumerate(times)):
        raise ValueError('Expected the original first 600 consecutive post-step samples')
    if len(data['stages']) != 4 or len({s['id'] for s in data['stages']}) != 4:
        raise ValueError('Four distinct history stages required')
    for stage in data['stages']:
        for key in ('relative_x_m', 'relative_y_m', 'heading_rad'):
            if len(stage[key]) != 600 or any(not math.isfinite(x) for x in stage[key]):
                raise ValueError('Incomplete or nonfinite trajectory')
        if abs(stage['relative_x_m'][0]) > 1e-8 or abs(stage['relative_y_m'][0]) > 1e-8:
            raise ValueError('Plots must be relative to the first observed sample')
        heights = stage['foot_clearance_m']
        if heights is not None and (len(heights) != 600 or any(len(row) != 4 or any(not math.isfinite(x) for x in row) for row in heights)):
            raise ValueError('Wrong foot-height observations')
        if len(stage['completed_lifts_15mm']) != 4 or any(type(x) is not int or x < 0 for x in stage['completed_lifts_15mm']):
            raise ValueError('Invalid measured event counts')
        if stage['first_failure_step'] is not None:
            raise ValueError('This four-panel comparison requires complete first episodes; do not fill past failure')


def render(data, output, font_path):
    from PIL import Image, ImageDraw, ImageFont
    import imageio_ffmpeg

    validate(data)
    output.mkdir(parents=True, exist_ok=True)
    paths = {suffix: output / ('forward-history.' + suffix) for suffix in ('mp4', 'gif', 'png')}
    if any(p.exists() for p in paths.values()):
        raise FileExistsError('Choose an empty media output directory')
    width, height = 1280, 960
    colors = {'bg': '#f1f5f0', 'ink': '#163c35', 'muted': '#5a7069', 'line': '#d9e3dc', 'trail': '#187c60'}
    fonts = {size: ImageFont.truetype(str(font_path), size=size) for size in (14, 16, 18, 20, 23, 30)}
    foot_colors = ['#287dba', '#c88127', '#ad4681', '#378c68']
    base = Image.new('RGB', (width, height), colors['bg'])
    draw = ImageDraw.Draw(base)

    def text(xy, value, size=18, fill=None):
        draw.text(xy, value, font=fonts[size], fill=fill or colors['ink'])

    text((30, 20), 'SingularityDog｜実測ログで見る歩行の変化', 30)
    text((30, 65), '図による再描画・実シミュレーション録画ではありません　／　前進・各12秒・実時間1倍', 18, colors['muted'])
    panels = []
    for i, stage in enumerate(data['stages']):
        x, y = 30 + (i % 2) * 625, 110 + (i // 2) * 385
        draw.rounded_rectangle((x, y, x + 600, y + 365), radius=16, fill='white', outline=colors['line'], width=2)
        text((x + 20, y + 15), stage['title'], 23)
        text((x + 20, y + 51), stage['subtitle'], 16, colors['muted'])
        # All panels share a world-coordinate scale. Marker size has no physical meaning.
        scale = 206
        origin = (x + 40, y + 201)
        def project(px, py):
            return (origin[0] + px * scale, origin[1] - py * scale)
        for px in (0, .5, 1.0, 1.5):
            a, b = project(px, -.3), project(px, .3)
            draw.line((a, b), fill=colors['line'], width=1)
            text((a[0] - 8, y + 269), f'{px:g}', 14, colors['muted'])
        for py in (-.25, 0, .25):
            a, b = project(0, py), project(1.5, py)
            draw.line((a, b), fill=colors['line'], width=1 if py else 2)
        text((x + 36, y + 86), '上から見た移動軌跡（共通の縮尺）', 16, colors['muted'])
        text((x + 253, y + 290), '前進方向 X [m]', 14, colors['muted'])
        text((x + 390, y + 86), '各脚の高さ [mm]', 16, colors['muted'])
        text((x + 405, y + 274), '破線 = 15mm', 14, colors['muted'])
        text((x + 20, y + 319), '15mm以上の完了足上げ（全12秒）', 14, colors['muted'])
        counts = ' / '.join(str(n) for n in stage['completed_lifts_15mm'])
        text((x + 356, y + 316), counts, 20)
        panels.append((x, y, origin, scale))
    text((30, 899), '順序：前左 / 前右 / 後左 / 後右　｜　矢印は胴体の記録位置と向き。機体形状や寸法を表しません。', 16, colors['muted'])
    text((30, 928), '指令・制御方式が異なる履歴比較です。速度だけで合格にせず、足上げ不足や退行も残しています。', 16, colors['muted'])
    encoder = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', f'{width}x{height}', '-r', '50', '-i', '-', '-an', '-c:v', 'libx264', '-crf', '21', '-pix_fmt', 'yuv420p',
               '-metadata', 'title=SingularityDog measured-motion history', '-map_metadata', '-1', '-movflags', '+faststart', str(paths['mp4'])]
    process = subprocess.Popen(encoder, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    selected = sorted(set(range(0, 600, 5)) | {599})
    gifs = []
    try:
        for frame in range(600):
            im = base.copy(); d = ImageDraw.Draw(im)
            d.rounded_rectangle((1060, 21, 1248, 65), radius=10, fill=colors['ink'])
            d.text((1081, 29), f'{data["time_s"][frame]:05.2f} / 12.00 s', font=fonts[18], fill='white')
            for stage, (x, y, origin, scale) in zip(data['stages'], panels):
                xy = [(origin[0] + px * scale, origin[1] - py * scale) for px, py in zip(stage['relative_x_m'][:frame + 1], stage['relative_y_m'][:frame + 1])]
                if len(xy) > 1:
                    d.line(xy, fill=colors['trail'], width=3)
                px, py = xy[-1]; angle = stage['heading_rad'][frame]
                points = [(px + xx * math.cos(angle) + yy * math.sin(angle), py - xx * math.sin(angle) + yy * math.cos(angle))
                          for xx, yy in ((12, 0), (-8, -7), (-4, 0), (-8, 7))]
                d.polygon(points, fill=colors['trail'])
                d.text((x + 30, y + 112), f'前進の進み幅 {stage["relative_x_m"][frame] * 100:6.1f} cm', font=fonts[18], fill=colors['ink'])
                heights = stage['foot_clearance_m']
                for foot in range(4):
                    bx, by = x + 430, y + 125 + foot * 34
                    d.text((x + 390, by - 2), ['前左', '前右', '後左', '後右'][foot], font=fonts[14], fill=colors['muted'])
                    d.rounded_rectangle((bx, by, bx + 95, by + 17), radius=4, fill='#ecf1eb')
                    if heights is not None:
                        h = heights[frame][foot] * 1000
                        d.rounded_rectangle((bx, by, bx + max(1, min(95, max(0, h) / 40 * 95)), by + 17), radius=3, fill=foot_colors[foot])
                        d.text((bx + 99, by - 2), str(round(h)), font=fonts[14], fill=colors['ink'])
                    else:
                        d.text((bx + 25, by - 2), '未保存', font=fonts[14], fill=colors['muted'])
                    tick = bx + 15 / 40 * 95
                    for a in (0, 6, 12):
                        d.line((tick, by + a, tick, by + a + 3), fill='#456255', width=1)
            process.stdin.write(im.tobytes())
            if frame == 599:
                im.save(paths['png'])
            if frame in selected:
                gifs.append(im.quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE))
        process.stdin.close()
        errors = process.stderr.read().decode()
        if process.wait() != 0:
            raise RuntimeError('Video encoder failed: ' + errors)
    except BaseException:
        process.kill(); process.wait()
        raise
    delays = [(selected[i + 1] - current) * 20 if i + 1 < len(selected) else 20 for i, current in enumerate(selected)]
    assert sum(delays) == 12000
    gifs[0].save(paths['gif'], save_all=True, append_images=gifs[1:], duration=delays, loop=0, optimize=True, disposal=1)
    return {'schema': 'singularitydog.motion-render.v1', 'representation': 'MEASURED_CHART_REPLAY_NOT_SIMULATOR_FOOTAGE',
            'geometry_assets_used': False, 'smoothing_or_interpolation': False, 'mp4_frames': 600, 'mp4_fps': 50, 'duration_s': 12,
            'gif_source_frame_indices': selected, 'gif_duration_ms': sum(delays), 'gif_frame_durations_ms': delays,
            'files': {p.name: {'sha256': digest(p), 'bytes': p.stat().st_size} for p in paths.values()}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True); p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--font', type=Path, required=True, help='User-provided font with Japanese glyphs; never bundled')
    a = p.parse_args()
    result = render(json.loads(a.input.read_text()), a.output_dir, a.font)
    result['input_sha256'] = digest(a.input)
    with (a.output_dir / 'render-receipt.json').open('x') as f:
        json.dump(result, f, indent=2); f.write('\n')
    print(json.dumps({k: result[k] for k in ('representation', 'mp4_frames', 'duration_s', 'files')}, indent=2))


if __name__ == '__main__':
    main()
