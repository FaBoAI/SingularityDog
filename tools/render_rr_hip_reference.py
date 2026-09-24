#!/usr/bin/env python3
"""Render a model reference schematic from public numbers; no mesh, photo or IO to hardware.

Requires Pillow. Uses Arial or DejaVu Sans; a different installed font can change
PNG pixels, so publication is bound to the reviewed PNG and generator hashes.
"""
import argparse
import json
import math
from pathlib import Path


def render(source, output):
    from PIL import Image, ImageDraw, ImageFont

    data = json.loads(Path(source).read_text())
    if (data.get('schema') != 'singularitydog.rr-hip-reference.v1'
            or data.get('vendor_geometry_used') is not False
            or data.get('user_photos_used') is not False
            or data.get('approved_for_runtime') is not False):
        raise ValueError('Expected an illustrative, non-actuating RR reference')
    model, observation = data['model'], data['reference_observation']
    length, q = model['axis_center_distance_mm'], observation['old_candidate_angle_deg']
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (length, q))
            or not 1 <= length <= 100 or not -80 < q < 0
            or model['hip_motor_id'] != 9 or model['thigh_motor_id'] != 8
            or model['hip_axis'] != [1, 0, 0] or model['hip_origin_rpy_rad'] != [0, 0, 0]
            or model['thigh_origin_from_hip_mm'] != [0, -length, 0]
            or observation['external_angle_measured'] is not False):
        raise ValueError('Reference geometry or eye-only observation does not match this diagram')
    height = -length * math.sin(math.radians(q))
    lateral = length * math.cos(math.radians(q))
    scale = 2
    image = Image.new('RGB', (1200 * scale, 900 * scale), '#ffffff')
    draw = ImageDraw.Draw(image)
    ink, muted, blue, border, amber = '#172b43', '#52657a', '#12647c', '#8397a7', '#ad6f13'
    fonts = {}

    def font(size, bold=False):
        key = size, bold
        if key not in fonts:
            choices = (['Arial Bold.ttf', 'DejaVuSans-Bold.ttf',
                        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'] if bold else
                       ['Arial.ttf', 'DejaVuSans.ttf',
                        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'])
            for name in choices:
                try:
                    fonts[key] = ImageFont.truetype(name, size * scale)
                    break
                except OSError:
                    continue
            else:
                raise RuntimeError('Install Arial or DejaVu Sans for readable diagram labels')
        return fonts[key]

    def text(x, y, value, size=22, color=ink, bold=False, centered=False):
        selected = font(size, bold)
        anchor = 'mt' if centered else 'lt'
        box = draw.textbbox((x * scale, y * scale), value, font=selected, anchor=anchor)
        if box[0] < 0 or box[1] < 0 or box[2] > image.width or box[3] > image.height:
            raise ValueError('Text would be clipped: ' + value)
        draw.text((x * scale, y * scale), value, font=selected, fill=color, anchor=anchor)

    def line(points, color=ink, width=2):
        draw.line([(round(x * scale), round(y * scale)) for x, y in points], fill=color, width=round(width * scale))

    def arrow(x1, x2, y, color=ink, both=False):
        line([(x1, y), (x2, y)], color)
        direction = 1 if x2 > x1 else -1
        draw.polygon([(x2 * scale, y * scale), ((x2 - 10 * direction) * scale, (y - 5) * scale),
                      ((x2 - 10 * direction) * scale, (y + 5) * scale)], fill=color)
        if both:
            draw.polygon([(x1 * scale, y * scale), ((x1 + 10 * direction) * scale, (y - 5) * scale),
                          ((x1 + 10 * direction) * scale, (y + 5) * scale)], fill=color)

    def rect(box, fill, outline=None, radius=12):
        draw.rounded_rectangle(tuple(round(v * scale) for v in box), radius=radius * scale,
                               fill=fill, outline=outline, width=scale)

    def circle(x, y, radius=15, color=blue, cross=True):
        draw.ellipse(((x - radius) * scale, (y - radius) * scale,
                      (x + radius) * scale, (y + radius) * scale), fill='white', outline=color, width=2 * scale)
        if cross:
            line([(x - radius - 6, y), (x + radius + 6, y)], color, 1)
            line([(x, y - radius - 6), (x, y + radius + 6)], color, 1)

    def dashed(x1, x2, y):
        for x in range(round(x1), round(x2), 13):
            line([(x, y), (min(x + 7, x2), y)], border, 1)

    text(48, 30, 'RIGHT REAR HIP · MANUAL REFERENCE', 32, bold=True)
    text(48, 75, "Rear view: stand behind the dog and look toward its head.", 22, muted)
    rect((48, 111, 1152, 147), '#edf3f8', radius=8)
    text(66, 120, 'SCHEMATIC · not to scale · projected joint centers · no motor actuation', 18)
    rect((48, 168, 1152, 577), '#f6fafc', '#cedbe6')
    text(76, 187, 'A. D17 reference: hip angle q = 0°', 25, bold=True)
    arrow(507, 213, 247)
    text(225, 220, 'Body +Y / robot LEFT', 18)
    text(885, 220, 'robot RIGHT →', 18)
    rect((140, 280, 540, 433), '#dfe8ee', border, radius=10)
    text(177, 302, 'BODY', 25, bold=True)
    text(177, 342, 'Use its left-right reference.', 18)
    text(177, 370, 'Not the floor.', 18)
    circle(508, 392, 12, muted, cross=False)
    line([(501, 385), (515, 399)], muted)
    line([(515, 385), (501, 399)], muted)
    text(326, 400, '+X / head: into page', 18)
    dashed(540, 1044, 360)
    line([(640, 360), (960, 360)], '#1685a1', 8)
    for x, mid, name in ((640, 9, 'Hip'), (960, 8, 'Thigh')):
        circle(x, 360)
        text(x, 275, f'ID {mid} · C{mid}', 25, bold=True, centered=True)
        text(x, 308, f'{name} rotation center', 18, centered=True)
        line([(x, 390), (x, 479)], blue, 1)
    arrow(640, 960, 457, blue, both=True)
    text(800, 424, f'{length:g} mm · center to center', 23, blue, bold=True, centered=True)
    text(76, 501, 'C9 → C8: parallel to the body left-right direction; same body-relative height.', 22, bold=True)
    text(76, 535, 'Use the rotation centers, not curved PLA edges. The knee and foot need not form an L.', 18, muted)
    rect((48, 599, 1152, 818), '#fff9ed', '#e4cea5')
    text(76, 620, 'B. Diagnostic comparison only', 25, bold=True)
    text(76, 657, f'An old candidate reading of {q:.2f}°')
    text(76, 688, f'would place C8 about {height:.1f} mm above C9.')
    text(76, 731, 'Eye-aligned reference; not a measured angle.', 18, muted)
    text(76, 758, 'The height is predicted by the model only.', 18, muted)
    cx, cy, diagram_scale = 730, 782, 4
    ex, ey = cx + lateral * diagram_scale, cy - height * diagram_scale
    dashed(715, 1025, cy)
    line([(cx, cy), (ex, ey)], amber, 5)
    circle(cx, cy, 8, amber, cross=False)
    circle(ex, ey, 8, amber, cross=False)
    line([(1009, ey), (1034, ey)], amber, 1)
    line([(1009, cy), (1034, cy)], amber, 1)
    line([(1022, ey + 7), (1022, cy - 7)], amber, 1)
    text(1042, 703, f'{height:.1f}', 18)
    text(1042, 730, 'mm', 18)
    text(696, 747, 'C9', 18)
    text(ex, ey - 34, 'C8', 18)
    text(793, 758, f'{q:.2f}°', 18)
    text(48, 838, 'Record the physical reference; do not move to an old encoder number.', 22, bold=True)
    text(48, 870, 'No automatic offset update, angle wrapping, standing command, or runtime approval.', 18, muted)
    image = image.resize((1200, 900), Image.Resampling.LANCZOS)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    image.save(output, format='PNG', optimize=True)
    print(json.dumps({'output': str(output), 'size_px': list(image.size),
                      'model_height_difference_mm': height, 'model_lateral_mm': lateral,
                      'representation': data['representation'], 'output_allowed': False}))


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root / 'evidence/rr-hip-reference.json')
    parser.add_argument('--output', type=Path, default=root / 'docs/media/rr-hip-horizontal-reference.png')
    args = parser.parse_args()
    render(args.source, args.output)
