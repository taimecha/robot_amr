#!/usr/bin/env python3
"""Generate an explicitly synthetic r3 Gazebo fixture; leave production URDF alone."""
import json
import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from appearance import PALLET_COLOR_HEX,set_pallet_materials
root=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--case',type=Path);args=p.parse_args()
scenario=json.loads(args.case.read_text()) if args.case else None
out=Path('/tmp/tai_r3_gazebo_assets')
out.mkdir(exist_ok=True)
g=json.loads((root/'simulation/r3/fixture.json').read_text())
shutil.copytree(root/'ros2_ws/description',out/'description',dirs_exist_ok=True)
f=out/'description/forklift.xacro'
s=f.read_text().replace('name="fork_length" value="0.390"','name="fork_length" value="0.385"')
s=s.replace('name="fork_thickness" value="0.008"','name="fork_thickness" value="0.003"')
s=s.replace('name="fork_ground_clearance" value="0.065"','name="fork_ground_clearance" value="0.071"')
s=s.replace('<origin xyz="${mast_profile / 2} 0\n', '<origin xyz="${mast_profile / 2} -0.020\n')
f.write_text(s)
world=ET.parse(root/'ros2_ws/worlds/docking_one_pallet.sdf')
if scenario:
    positions=[scenario['base_world'],scenario['pallet_world']]
    extent=2*max(6.,max(abs(v) for pos in positions for v in pos[:2])+4.)
    for e in world.getroot().findall('.//plane/size'):e.text=f'{extent} {extent}'
    b=scenario['base_world'];pw=scenario['pallet_world']
    camera=world.getroot().find('.//gui/plugin[@name="3D View"]/camera_pose')
    if camera is not None:camera.text=f'{b[0]-2.2} {b[1]-2.8} 2.1 0 0.38 0.7'
    (out/'case.json').write_text(json.dumps(scenario,indent=2))
for e in world.getroot().findall('.//model[@name="single_floor_pallet"]/link/*'):
    if e.tag in ('visual','collision') and e.get('name','').startswith('top_'):
        e.find('pose').text='0 0 0.115 0 0 0'
        e.find('geometry/box/size').text='0.355 0.355 0.030'
visual_count=set_pallet_materials(world.getroot())
(out/'appearance.json').write_text(json.dumps(dict(pallet_color_hex=PALLET_COLOR_HEX,visual_count=visual_count,geometry_changed_by_color=False),indent=2))
# World-level model pose is read separately by gz topic, never by the follower.
world.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
(out/'fixture.json').write_text(json.dumps(g,indent=2))
print(out)
