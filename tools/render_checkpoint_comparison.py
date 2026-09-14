#!/usr/bin/env python3
"""Render the published Trial32 checkpoint aggregates (requires matplotlib)."""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def render(source, destination):
    data = json.loads(Path(source).read_text())
    if data.get('schema') != 'singularitydog.l27-evaluation.v1':
        raise ValueError('Expected Trial32 aggregate schema')
    rows = data['checkpoint_comparison']
    limit = data['checkpoint_evaluation_protocol']['stationary_xy_limit_m'] * 1000
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.1), dpi=160)
    fig.patch.set_facecolor('#f4f7fb')
    colors = {'A': '#1565c0', 'B': '#cf5700'}
    for ax, expert, title in zip(axes, ('prone', 'wave'), ('Lie down', 'Wave')):
        ax.set_title(title, fontsize=17, loc='left', fontweight='bold', pad=13)
        ax.set_facecolor('white')
        for branch in ('A', 'B'):
            group = sorted((r for r in rows if r['expert'] == expert and r['branch'] == branch),
                           key=lambda r: r['checkpoint_updates'])
            if [r['checkpoint_updates'] for r in group] != [0, 100, 200, 250]:
                raise ValueError('Expected parent, 100, 200 and 250 exactly once')
            xs = [r['checkpoint_updates'] for r in group]
            ys = [r['maximum_xy_distance_m'] * 1000 for r in group]
            ax.plot(xs, ys, color=colors[branch], marker='o' if branch == 'A' else 's',
                    linewidth=2.1, markersize=5, label=f'Condition {branch}', alpha=.9)
            for row, x, y in zip(group, xs, ys):
                requested = data['checkpoint_evaluation_protocol']['requested_duration_s'][expert]
                if row['duration_s'] < requested:
                    ax.scatter([x], [y], marker='x', color='#9d1728', s=75, zorder=5)
                    dx, dy = ((-12, -21) if branch == 'A' else (-12, 13))
                    ax.annotate(f"Stopped {row['duration_s']:.2f}s / {requested}s",
                                (x, y), xytext=(dx, dy), textcoords='offset points',
                                ha='right', fontsize=9, color='#9d1728',
                                bbox=dict(facecolor='white', edgecolor='none', alpha=.85, pad=1.5))
        ax.axhline(limit, color='#287044', linestyle='--', linewidth=1.5)
        ax.set_xlim(-10, 265)
        ax.set_xticks([0, 100, 200, 250], ['Parent', '100', '200', '250'])
        ax.set_xlabel('New training updates', labelpad=10, fontsize=11)
        ax.set_ylabel('Maximum XY displacement (mm)', fontsize=11)
        ax.grid(axis='y', alpha=.18)
        ax.spines[['top', 'right']].set_visible(False)
        ax.tick_params(labelsize=10)
        ax.set_ylim((0, 210) if expert == 'prone' else (45, 68))
        if expert == 'wave':
            ax.text(12, 65.2, 'All six trained checkpoints completed 36s.\nPosition target still missed.',
                    fontsize=10, color='#334155')
    handles = [Line2D([0], [0], color=colors[b], marker='o' if b == 'A' else 's', label=f'Condition {b}') for b in ('A', 'B')]
    handles.append(Line2D([0], [0], color='#287044', linestyle='--', label=f'Stationary XY limit: {limit:g} mm'))
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .875), ncol=3, frameon=False, fontsize=11)
    fig.suptitle('Trial 32  |  Checkpoint comparison', x=.055, y=.975, ha='left', fontsize=21, fontweight='bold', color='#172b4d')
    fig.text(.055, .906, 'A/B: 7 of 9 final motions passed. Lie down and wave remain below the registered targets.', fontsize=11, color='#334155')
    fig.text(.055, .085, 'Different vertical scales. A single XY metric is not the full skill acceptance test.', fontsize=10, color='#334155')
    fig.text(.055, .045, 'Seed 42 only. 100/200 checkpoints evaluated after 250 updates; no extra PPO updates. Early stops have shorter records.', fontsize=9.5, color='#334155')
    fig.subplots_adjust(left=.07, right=.98, top=.76, bottom=.22, wspace=.24)
    fig.savefig(destination, facecolor=fig.get_facecolor(), metadata={'Software': 'SingularityDog public aggregate renderer'})
    plt.close(fig)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root / 'evidence/l27-evaluation.json')
    parser.add_argument('--output', type=Path, default=root / 'docs/media/l27-checkpoint-comparison.png')
    args = parser.parse_args()
    render(args.source, args.output)


if __name__ == '__main__':
    main()
