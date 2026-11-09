#!/usr/bin/env python3
"""Create an immutable approved Nav2 plan for an entered case or frozen RGB-D."""
import argparse
import json
from pathlib import Path
from r3_case import load_case
from r3_core import collision,digest,fork_for_base
from r3_sim import build_backend,plan_once,write_csv,plot

def prepare(case_path,out,vision_path=None):
    scenario=load_case(case_path)
    g=json.loads((Path(__file__).parent/'r3/fixture.json').read_text())
    start=scenario['start_odom']
    if vision_path:
        vision=json.loads(Path(vision_path).read_text())
        if vision.get('case_id')!=scenario['case_id'] or not vision.get('locked_target'):
            raise ValueError('Chưa có pose RGB-D khóa cho đúng ca thử này.')
        snapshot=vision['locked_target']
    else:
        snapshot=dict(frame_id='odom',feature='front_entry_corridor_center',pose=scenario['target_odom'],
                      applied_bias=[],calibration_hash='entered_simulation_pose',capture_stamp_s=0.,
                      source='ENTERED_POSE_NOT_RGBD',case_id=scenario['case_id'])
    hit=collision(start,snapshot['pose'],g)
    if hit:raise ValueError('Tư thế ban đầu va chạm: '+hit)
    plan,phases,length,rotations=plan_once(start,snapshot,g,build_backend(),out)
    rows=[dict(phase=p['type'],x_m=v[0],y_m=v[1],yaw_rad=v[2]) for p in phases for v in p['poses']]
    write_csv(out/'path_planned_base.csv',rows)
    write_csv(out/'path_planned_fork_root.csv',[dict(phase=p['type'],**dict(zip(('x_m','y_m','yaw_rad'),fork_for_base(v,g)))) for p in phases for v in p['poses']])
    (out/'target_snapshot.json').write_text(json.dumps(snapshot,indent=2))
    (out/'case.json').write_text(json.dumps(scenario,indent=2))
    plot(out,phases,[],snapshot['pose'],g,{},preview_only=True)
    return dict(status='PREVIEW_WAIT',plan_id=plan.plan_id,length_m=length,rotations=rotations,source=snapshot['source'])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--vision',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    try:r=prepare(a.case,a.output,a.vision)
    except Exception as e:r=dict(status='INFEASIBLE',reason=str(e))
    (a.output/'prepare_result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,ensure_ascii=False,indent=2))
    if r['status']!='PREVIEW_WAIT':raise SystemExit(1)
if __name__=='__main__':main()
