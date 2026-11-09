#!/usr/bin/env python3
"""Read Gazebo truth for independent scoring only. No ROS or actuator imports."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
from r3_core import compose, errors, fork_for_base, inverse


def pose(v):
    p=v['position'];q=v['orientation']
    return (p.get('x',0.),p.get('y',0.),math.atan2(2*(q.get('w',1.)*q.get('z',0.)+q.get('x',0.)*q.get('y',0.)),
            1-2*(q.get('y',0.)**2+q.get('z',0.)**2)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--duration',type=float,default=0.,help='0: no run-duration limit')
    parser.add_argument('--case',type=Path)
    parser.add_argument('--fixture',type=Path,default=Path(__file__).resolve().parent/'r3/fixture.json')
    args=parser.parse_args()
    if os.environ.get('GZ_PARTITION')!='tai_r3_sim_only_89':parser.error('isolated Gazebo partition required')
    args.output.mkdir(parents=True,exist_ok=True)
    g=json.loads(args.fixture.read_text());last=-1.;count=0
    # This fixed world->odom registration is test setup, never planner input.
    # World spawn=(0.0225,0.10,0) corresponds to the exact-input odom case.
    scenario=json.loads(args.case.read_text()) if args.case else None
    T_odom_world=inverse(scenario['base_world'] if scenario else (.0225,.10,0.))
    command=['gz','topic','-e','-t','/world/warehouse/dynamic_pose/info','--json-output']
    if args.duration>0:command+=['-d',str(args.duration)]
    process=subprocess.Popen(command,stdout=subprocess.PIPE,text=True)
    try:
        with (args.output/'gazebo_truth.csv').open('w',newline='') as f:
            writer=None
            for line in process.stdout:
                try:
                    msg=json.loads(line);stamp=msg['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
                    if t-last<.020:continue
                    models={v.get('name'):v for v in msg['pose']}
                    if not all(k in models for k in ('tai_robot_one','single_floor_pallet')):continue
                    base=compose(T_odom_world,pose(models['tai_robot_one']))
                    target=compose(T_odom_world,compose(pose(models['single_floor_pallet']),(-.1775,0.,0.)))
                    fork=fork_for_base(base,g);e=errors(base,target,g)
                    row=dict(sim_stamp_s=t,x_m=base[0],y_m=base[1],yaw_rad=base[2],
                             fork_x_m=fork[0],fork_y_m=fork[1],fork_yaw_rad=fork[2],
                             pallet_rise_m=models['single_floor_pallet']['position'].get('z',0.),**e)
                    if writer is None:
                        writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader()
                        (args.output/'truth_first_snapshot.json').write_text(json.dumps(msg,indent=2))
                    writer.writerow(row);f.flush();last=t;count+=1
                    (args.output/'truth_last_snapshot.json').write_text(json.dumps(msg,indent=2))
                except (KeyError,ValueError):continue
    finally:
        if process.poll() is None:process.terminate()
        process.wait(timeout=5.)
    print('truth samples:',count)
if __name__=='__main__':main()
