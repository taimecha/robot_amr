#!/usr/bin/env python3
"""Offline S->A quintic + A->F insertion candidate screen; no robot interfaces."""
import csv
import json
import math
from pathlib import Path

from docking_preview import candidate


FORK_TIP_X = .3005 + .455  # chassis front + measured projection
FORK_OUTER_Y = .0775       # two forks, total outside width 155 mm
PALLET_HOLE_INNER_Y = .095 # synthetic model runner inner face
BODY_X = (-.33, .3005)
BODY_HALF_Y = .25
AISLE_HALF_Y = .60
KAPPA_LIMIT = 2.5          # illustrative only: real Rmin not measured
V_LIMIT = .08
W_LIMIT = .20


def transformed(x,y,yaw,dx,dy):
    c,s=math.cos(yaw),math.sin(yaw)
    return x+c*dx-s*dy,y+s*dx+c*dy


def swept_check(rows, face_x, obstacle=False):
    clearance=float('inf')
    touched_pallet=False
    for x,y,yaw,_,_ in rows:
        for dx in BODY_X:
            for dy in (-BODY_HALF_Y,BODY_HALF_Y):
                px,py=transformed(x,y,yaw,dx,dy)
                clearance=min(clearance,AISLE_HALF_Y-abs(py))
                if px>=face_x:
                    return -1., 'body intersects pallet face'
                if obstacle and .42<=px<=.58 and abs(py)<.40:
                    return -1., 'synthetic obstacle intersects body'
        for dx in (.3655,FORK_TIP_X):
            for dy in (-FORK_OUTER_Y,FORK_OUTER_Y):
                px,py=transformed(x,y,yaw,dx,dy)
                clearance=min(clearance,AISLE_HALF_Y-abs(py))
                if px>=face_x:
                    touched_pallet=True
                    clearance=min(clearance,PALLET_HOLE_INNER_Y-abs(py))
                    if abs(py)>=PALLET_HOLE_INNER_Y:
                        return clearance,'fork intersects synthetic runner'
    return clearance, 'synthetic 2D clearance only' if touched_pallet else 'no insertion'


def uncertainty_budget():
    # Explicit synthetic assumptions; never substitute for calibrated Pi data.
    terms={'target_lateral_m':.002,'target_yaw_m':FORK_TIP_X*math.tan(math.radians(.2)),
           'odom_drift_m':.002,'tracking_error_m':.003,'margin_m':.002}
    return terms,sum(terms.values())


def evaluate(lateral,yaw,distance,obstacle=False):
    # Search tangent scales; reject hard constraints before comparing length.
    face_x=distance+FORK_TIP_X-.20
    terms,uncertainty=uncertainty_budget()
    alternatives=[]
    for scale in (.7,1.0,1.3):
        path=candidate(lateral,yaw,distance,tangent_scale=scale)
        rows=path['rows']
        clearance,why=swept_check(rows,face_x,obstacle)
        kmax=path['max_abs_kappa_1pm']
        forward=all(b[0]>a[0]-1e-7 for a,b in zip(rows,rows[1:]))
        feasible=(forward and kmax<=KAPPA_LIMIT and clearance>uncertainty)
        # Linear acceleration and curvature-limited angular speed; rough estimate.
        time_s=path['length_m']/min(V_LIMIT,W_LIMIT/max(kmax,1e-6))+V_LIMIT/.08
        alternatives.append({'scale':scale,'feasible':feasible,
                             'length_m':path['length_m'],'max_kappa_1pm':kmax,
                             'swept_clearance_m':clearance,'uncertainty_m':uncertainty,
                             'estimated_time_s':time_s,'reason':why if not obstacle else why})
    candidates=[a for a in alternatives if a['feasible']]
    selected=min(candidates,key=lambda a:a['length_m']+.02*a['max_kappa_1pm']**2) if candidates else None
    best=selected or max(alternatives,key=lambda a:a['swept_clearance_m']-.02*a['max_kappa_1pm'])
    return {'lateral_m':lateral,'yaw_deg':yaw,'approach_m':distance,
            'result':'ILLUSTRATIVE_FEASIBLE' if selected else 'REPOSITION_REQUIRED',
            'tangent_scale':best['scale'],'length_m':round(best['length_m'],4),
            'max_kappa_1pm':round(best['max_kappa_1pm'],4),
            'swept_clearance_m':round(best['swept_clearance_m'],4),
            'uncertainty_m':round(uncertainty,4),
            'estimated_time_s':round(best['estimated_time_s'],2),
            'reason':best['reason'],'obstacle':obstacle}


def main(output=Path('simulation/output')):
    output.mkdir(parents=True,exist_ok=True)
    rows=[]
    for distance in (.8,1.0,1.2,1.4):
        for sign in (-1,1):
            for lateral in (.02,.05,.10):
                for yaw in (2,5,10):
                    rows.append(evaluate(sign*lateral,sign*yaw,distance))
    rows.append(evaluate(.02,2,1.2,obstacle=True))
    with (output/'plan_grid.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    terms,total=uncertainty_budget()
    report={'model':'synthetic 2D pallet and clear aisle',
            'geometry':{'fork_tip_from_base_m':FORK_TIP_X,'fork_outer_half_width_m':FORK_OUTER_Y,
                        'pallet_runner_inner_half_width_m':PALLET_HOLE_INNER_Y,
                        'nominal_lateral_clearance_m':PALLET_HOLE_INNER_Y-FORK_OUTER_Y},
            'assumed_uncertainty_terms_m':terms,'assumed_uncertainty_total_m':total,
            'cases':len(rows),'illustrative_feasible':sum(r['result']=='ILLUSTRATIVE_FEASIBLE' for r in rows),
            'reposition_required':sum(r['result']=='REPOSITION_REQUIRED' for r in rows),
            'blocked_obstacle_case':rows[-1],
            'limits':['2D only; no vertical clearance or load deformation',
                      '2.5 1/m curvature limit is hypothetical',
                      'uncertainty values are hypothetical, not measured',
                      'unknown cells and real obstacles are not modeled',
                      'no INSERT or LIFT authorization from this report']}
    (output/'plan_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
