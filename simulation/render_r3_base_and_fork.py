#!/usr/bin/env python3
"""Draw planned base_footprint and fork-root paths from one frozen r3 plan."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure

from r3_core import compose, fork_for_base, wrap
from r3_plot import draw_two_paths


def read_path(path):
    with path.open(newline='') as source:
        rows = list(csv.DictReader(source))
    return [(row['phase'], (float(row['x_m']), float(row['y_m']),
                             float(row['yaw_rad']))) for row in rows]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan_dir', type=Path)
    args = parser.parse_args()
    out = args.plan_dir
    result = json.loads((out / 'prepare_result.json').read_text())
    if result.get('status') != 'PREVIEW_WAIT':
        parser.error('plan preview is incomplete')
    frozen = json.loads((out / 'frozen_plan.json').read_text())
    geometry = json.loads(frozen['geometry_json'])
    case = json.loads((out / 'case.json').read_text())
    base = read_path(out / 'path_planned_base.csv')
    fork = read_path(out / 'path_planned_fork_root.csv')
    if not base or len(base) != len(fork) or [v[0] for v in base] != [v[0] for v in fork]:
        parser.error('base and fork-root samples do not correspond')
    for (_, body), (_, root) in zip(base, fork):
        predicted = fork_for_base(body, geometry)
        if math.dist(predicted[:2], root[:2]) > 1e-8 or abs(wrap(predicted[2]-root[2])) > 1e-8:
            parser.error('fork-root samples disagree with the frozen base-to-fork transform')
    phase = [name for name, _ in base]
    if phase[0] != 'APPROACH' or phase[-1] != 'STRAIGHT':
        parser.error('A-C-B phases are missing')
    c_index = phase.index('STRAIGHT')
    base_world = [compose(case['base_world'], pose) for _, pose in base]
    fork_world = [compose(case['base_world'], pose) for _, pose in fork]

    fig = Figure(figsize=(12, 6))
    ax = fig.subplots()
    draw_two_paths(ax, [pose for _, pose in fork], [], case['target_odom'], geometry, scene=case)
    for line in list(ax.lines):
        line.remove()
    ax.plot([p[0] for p in base_world], [p[1] for p in base_world],
            color='#6545aa', lw=2.8, ls='--', label='Tâm xe · base_footprint')
    ax.plot([p[0] for p in fork_world], [p[1] for p in fork_world],
            color='#078c70', lw=3.0, label='Tâm gốc càng · giữa hai càng')
    # A, C and B refer to the same three instants on both trajectories.
    for label, index in (('A', 0), ('C', c_index), ('B', len(base)-1)):
        b, f = base_world[index], fork_world[index]
        ax.plot([b[0], f[0]], [b[1], f[1]], color='#999', lw=.8, ls=':', zorder=2)
        ax.scatter([b[0]], [b[1]], s=55, marker='s', color='#6545aa', zorder=7)
        ax.scatter([f[0]], [f[1]], s=65, marker='o', color='#078c70', zorder=8)
    labels = [('A xe', base_world[0], (0, -24)),
              ('A càng', fork_world[0], (0, 13)),
              ('C xe', base_world[c_index], (-10, 16)),
              ('C càng', fork_world[c_index], (-5, -24)),
              ('B xe', base_world[-1], (-5, 16)),
              ('B càng', fork_world[-1], (-15, -24))]
    for label, pose, offset in labels:
        ax.annotate(label, pose[:2], xytext=offset, textcoords='offset points',
                    ha='center', fontsize=9, weight='bold', color='#222',
                    bbox=dict(boxstyle='round,pad=.12', fc='white', ec='none', alpha=.86))
    ax.set_title('Hai quỹ đạo từ cùng một kế hoạch A → C → B')
    ax.set_xlabel('world X (m)')
    ax.set_ylabel('world Y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(min(p[0] for p in base_world) - .42,
                case['pallet_world'][0] + geometry['pallet_length'] + .12)
    ax.set_ylim(case['pallet_world'][1] - .49, case['pallet_world'][1] + .49)
    ax.legend(loc='upper right')
    lateral_cm = geometry['base_fork'][1] * 100
    lateral_text = '0,0' if abs(lateral_cm) < 1e-9 else f'{lateral_cm:+.1f}'.replace('.', ',')
    fig.text(.5, .018,
             f'C: lệch Y gốc càng so tâm xe {lateral_text} cm trong kế hoạch này; '
             'URDF đặt hai càng đối xứng · xe chưa chạy',
             ha='center', color='#555', fontsize=9)
    fig.tight_layout(rect=(0, .05, 1, 1))
    for suffix in ('png', 'svg'):
        fig.savefig(out / f'planned_base_and_fork.{suffix}', dpi=175)
    print(out / 'planned_base_and_fork.png')


if __name__ == '__main__':
    main()
