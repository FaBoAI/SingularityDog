#!/usr/bin/env python3
"""Draw the selected Trial35 displacement measurements, not simulator footage."""
import argparse
import json
import math
from pathlib import Path


def render(source, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = json.loads(Path(source).read_text())
    if data.get('schema') != 'singularitydog.l30-evaluation.v1':
        raise ValueError('Expected Trial35 selected results')
    panels = data['latest_checkpoint_comparison']
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.3))
    fig.patch.set_facecolor('#f5f7fb')
    for ax, motion, title, color in zip(
            axes, ('prone', 'wave'), ('Prone | projected R4b', 'Wave | scoped W4'),
            ('#116c9a', '#9f4d13')):
        rows = panels[motion]
        values = [row['maximum_xy_distance_m'] * 1000 for row in rows]
        if len(rows) != 3 or not all(math.isfinite(v) and 0 <= v <= 120 for v in values):
            raise ValueError('Expected three finite displacement values per motion')
        if not all(row['recorded_duration_s'] == row['requested_duration_s'] for row in rows):
            raise ValueError('This chart compares only complete requested intervals')
        labels = ['Parent', f"{rows[1]['updates']} updates", f"{rows[2]['updates']} updates"]
        ax.barh([2, 1, 0], values, height=.47, color=['#8e9aab', color, color])
        for y, value in zip([2, 1, 0], values):
            ax.text(value + 1.8, y, f'{value:.3f} mm', va='center', fontsize=11)
        ax.axvline(50, color='#b83542', linestyle='--', linewidth=1.5)
        ax.text(51.5, 2.47, '50 mm limit', color='#b83542', fontsize=10)
        ax.set_yticks([2, 1, 0], labels)
        ax.set_xlim(0, 120)
        ax.set_ylim(-.55, 2.8)
        ax.set_xticks([0, 25, 50, 75, 100])
        ax.set_xlabel('Maximum horizontal displacement (mm)', fontsize=10)
        ax.set_title(title, loc='left', fontsize=14, fontweight='bold', pad=14)
        ax.set_axisbelow(True)
        ax.grid(axis='x', color='#dde3ec', linewidth=.7)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color('#cbd4e0')
        ax.tick_params(axis='y', length=0)
    fig.suptitle('Trial 35 | Latest evaluated candidates', x=.06, ha='left',
                 fontsize=19, fontweight='bold', color='#172b4d')
    fig.text(.06, .87, 'Full 2-cycle evaluations: prone 24 s / wave 36 s. All six remain above the XY limit.',
             fontsize=11, color='#42516a')
    fig.text(.06, .055, 'Measured aggregates, not footage. One seed; different tasks and parents.\n'
             'Prone final also misses yaw and second hold. No candidate adopted.',
             fontsize=10, color='#42516a')
    fig.subplots_adjust(left=.10, right=.98, bottom=.23, top=.77, wspace=.32)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor=fig.get_facecolor(), metadata={'Software': 'Matplotlib'})
    plt.close(fig)


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root / 'evidence/l30-evaluation.json')
    parser.add_argument('--output', type=Path, default=root / 'docs/media/l30-checkpoint-comparison.png')
    args = parser.parse_args()
    render(args.source, args.output)
