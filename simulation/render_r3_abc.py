#!/usr/bin/env python3
"""Label A, C, B on an existing r3 planner output; never generate a path."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure

from r3_core import compose
from r3_plot import draw_two_paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan_dir', type=Path)
    args = parser.parse_args()
    out = args.plan_dir
    result = json.loads((out / 'prepare_result.json').read_text())
    if result.get('status') != 'PREVIEW_WAIT':
        parser.error('planner did not produce a complete preview')
    case = json.loads((out / 'case.json').read_text())
    if case.get('provenance') != 'LIVE_CAMERA_MEASUREMENT_IN_SYNTHETIC_GEOMETRY_PREVIEW':
        parser.error('expected a camera-measurement simulation case')
    frozen = json.loads((out / 'frozen_plan.json').read_text())
    geometry = json.loads(frozen['geometry_json'])
    with (out / 'path_planned_fork_root.csv').open(newline='') as source:
        rows = list(csv.DictReader(source))
    if not rows or rows[0]['phase'] != 'APPROACH' or rows[-1]['phase'] != 'STRAIGHT':
        parser.error('A-C-B phase boundary missing')
    planned = [(float(r['x_m']), float(r['y_m']), float(r['yaw_rad'])) for r in rows]
    straight_start = next(i for i, row in enumerate(rows) if row['phase'] == 'STRAIGHT')
    world_from_odom = case['base_world']
    world = [compose(world_from_odom, pose) for pose in planned]
    target = case['pallet_world']
    fig = Figure(figsize=(11, 6))
    ax = fig.subplots()
    draw_two_paths(ax, planned, [], case['target_odom'], geometry, scene=case)
    for line in list(ax.lines):
        line.remove()
    approach = world[:straight_start + 1]
    straight = world[straight_start:]
    approach_label = ('A → C · Smac chọn C + tối ưu đường gốc càng'
                      if result.get('approach_method') else
                      'A → C · Smac + Constrained Smoother')
    ax.plot([p[0] for p in approach], [p[1] for p in approach], lw=2.6,
            color='#246ac5', label=approach_label)
    ax.plot([p[0] for p in straight], [p[1] for p in straight], lw=3.2,
            color='#159b7e', label='C → B · đoạn thẳng theo trục pallet')
    for label, pose, offset in (('A', world[0], (-10, -25)),
                                ('C', world[straight_start], (-5, 13)),
                                ('B', world[-1], (-8, 13))):
        ax.scatter([pose[0]], [pose[1]], s=55, color='#222', zorder=8)
        ax.annotate(label, pose[:2], xytext=offset, textcoords='offset points',
                    weight='bold', fontsize=12, color='#222',
                    bbox=dict(boxstyle='round,pad=.15', fc='white', ec='none', alpha=.9))
    ax.set_title('Quỹ đạo r3 tính từ số đo camera: A → C → B')
    ax.legend(loc='upper right')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(min(p[0] for p in world) - .6, target[0] + geometry['pallet_length'] + .15)
    ax.set_ylim(target[1] - .48, target[1] + .48)
    measured = case['source_measurement']
    note = (f"Camera tại A: X từ gốc càng {measured['fork_root_forward_m']:.3f} m, "
            f"Y web +{measured['lateral_m']*100:.1f} cm, "
            f"góc {measured['filtered_fork_alignment_deg']:.2f}°; "
            'Y web tạm dùng cho gốc càng · hình học mô phỏng · xe chưa chạy')
    fig.text(.5, .02, note, ha='center', fontsize=9, color='#555')
    fig.tight_layout(rect=(0, .045, 1, 1))
    for suffix in ('png', 'svg'):
        fig.savefig(out / f'planned_A_C_B.{suffix}', dpi=175)
    print(out / 'planned_A_C_B.png')


if __name__ == '__main__':
    main()
