#!/usr/bin/env python3
"""Match independent Gazebo truth to odom timestamps; score, never steer."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from r3_core import collision


def tracking_acceptance(metrics, odom, longitudinal_required=True):
    """Longitudinal waiver changes only that criterion, never yaw/lateral/speed."""
    stop=metrics.get('stopped_truth')
    return {
        'completed':stop is not None,
        'truth_coverage':metrics['matched_samples']==metrics['odom_samples'],
        'forward_speed':bool(odom) and all(0.<=float(r['cmd_v_mps'])<=.15+1e-10 for r in odom),
        'terminal_yaw':metrics['terminal_truth_max_yaw_deg'] is not None and metrics['terminal_truth_max_yaw_deg']<=.5,
        'terminal_lateral':metrics['terminal_truth_max_lateral_m'] is not None and metrics['terminal_truth_max_lateral_m']<=.020,
        'stopped_yaw':stop is not None and abs(math.degrees(stop['yaw_error_rad']))<=.5,
        'stopped_lateral':stop is not None and abs(stop['ey_m'])<=.020,
        'longitudinal_stop':not longitudinal_required or stop is not None and abs(stop['gap_m']-.020)<=.005,
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--plan',required=True,type=Path)
    parser.add_argument('--longitudinal-stop-report-only',action='store_true')
    args=parser.parse_args();out=args.run
    controller_file=out/'controller_result.json'
    result=json.loads((controller_file if controller_file.exists() else out/'mission_result.json').read_text())
    if not controller_file.exists():controller_file.write_text(json.dumps(result,indent=2))
    frozen=json.loads(args.plan.read_text());g=json.loads(frozen['geometry_json'])
    odom=list(csv.DictReader((out/'executed_odom_base.csv').open()))
    truth={}
    for file in sorted(out.glob('**/gazebo_truth.csv')):
        for r in csv.DictReader(file.open()):
            if r.get('sim_stamp_s'):truth[float(r['sim_stamp_s'])]={k:float(v) for k,v in r.items()}
    stamps=sorted(truth);matched=[]
    for r in odom:
        t=float(r['sim_stamp_s']);i=bisect.bisect_left(stamps,t)
        candidates=[j for j in (i-1,i) if 0<=j<len(stamps)]
        if not candidates:continue
        j=min(candidates,key=lambda j:abs(stamps[j]-t));stamp=stamps[j]
        if abs(stamp-t)>.030:continue
        tr=truth[stamp]
        matched.append(dict(time_s=float(r['time_s']),sim_stamp_s=t,state=r['state'],
                            alignment_error_s=stamp-t,**{k:v for k,v in tr.items() if k!='sim_stamp_s'}))
    if not matched:raise RuntimeError('no synchronized truth/odom samples')
    with (out/'executed_truth_base_fork.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(matched[0]));writer.writeheader();writer.writerows(matched)
    at_stop=next((r for r in matched if r['state'] in ('LIFTING','DONE_TRACKING')),None)
    line=[r for r in matched if r['state']=='STRAIGHT']
    metrics=dict(matched_samples=len(matched),odom_samples=len(odom),
                 max_timestamp_difference_s=max(abs(r['alignment_error_s']) for r in matched),
                 stopped_truth=at_stop,final_truth=matched[-1],
                 terminal_truth_max_yaw_deg=max((abs(math.degrees(r['yaw_error_rad'])) for r in line),default=None),
                 terminal_truth_max_lateral_m=max((abs(r['ey_m']) for r in line),default=None))
    if at_stop:
        metrics['stopped_truth_gap_pass']=abs(at_stop['gap_m']-.020)<=.005
        metrics['stopped_truth_yaw_pass']=abs(math.degrees(at_stop['yaw_error_rad']))<=.5
        metrics['stopped_truth_lateral_pass']=abs(at_stop['ey_m'])<=.020
    waiver=args.longitudinal_stop_report_only or result.get('longitudinal_stop_report_only',False)
    criteria=tracking_acceptance(metrics,odom,longitudinal_required=not waiver)
    tracking_pass=all(criteria.values())
    status='PASS_TRACKING_SIMULATION' if tracking_pass and result.get('tracking_only') else 'FAIL_SIMULATION' if not tracking_pass or result['state']=='STOP_LATCHED' else 'INCOMPLETE'
    result.update(status=status,tracking_status='PASS' if tracking_pass else 'FAIL',
                  tracking_criteria=criteria,longitudinal_stop_report_only=waiver,
                  independent_scorer=metrics,ground_truth_used_for_control=False,
                  remaining=['real extrinsics/tine positions confirmation','stationary real RGB-D pose/accuracy/preview','RGB-D input simulation case','contact force instrumentation'])
    result.pop('missing',None)
    (out/'mission_result_scored.json').write_text(json.dumps(result,indent=2))
    (out/'mission_result.json').write_text(json.dumps(result,indent=2))
    pp=[]
    for phase in json.loads(frozen['phases_json']):pp.extend(phase['poses'])
    from r3_core import fork_for_base,polygons,rectangle
    ff=[fork_for_base(p,g) for p in pp]
    from r3_plot import save_two_paths
    case_file=args.plan.parent/'case.json'
    scene=json.loads(case_file.read_text()) if case_file.exists() else None
    save_two_paths(out,ff,[(r['fork_x_m'],r['fork_y_m']) for r in matched],
                   json.loads(frozen['target_json'])['pose'],g,scene=scene)
    fig,ax=plt.subplots(figsize=(12,5))
    for path,label,color,style in [(pp,'planned base','#226bb0','-'),(ff,'planned fork root','#de8023','-'),
        ([(float(r['x_m']),float(r['y_m'])) for r in odom],'Gazebo odom base','#15a3b0','--'),
        ([(float(r['fork_x_m']),float(r['fork_y_m'])) for r in odom],'Gazebo odom fork root','#b82a80','--'),
        ([(r['x_m'],r['y_m']) for r in matched],'Gazebo truth base','#287b37',':'),
        ([(r['fork_x_m'],r['fork_y_m']) for r in matched],'Gazebo truth fork root','#734d1b',':')]:
        ax.plot(*zip(*[p[:2] for p in path]),style,color=color,label=label)
    target=json.loads(frozen['target_json'])['pose'];_,_,legs=polygons(pp[-1],target,g)
    from matplotlib.patches import Polygon
    for leg in legs:ax.add_patch(Polygon(leg,color='#614a3c'))
    ax.add_patch(Polygon(rectangle(target,(0.,g['pallet_length'],-g['pallet_width']/2,g['pallet_width']/2)),fill=False,color='black'))
    ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('odom X (m)');ax.set_ylabel('odom Y (m)');ax.grid(alpha=.25);ax.legend(fontsize=8)
    ax.set_title('r3 | planned / Gazebo odom / independent truth\nSynthetic exact-input target; independent scoring; no truth steering')
    # Millimetre-scale stop difference needs its own equal-aspect view.
    zoom=ax.inset_axes([.03,.56,.28,.38])
    for x,y,label,color in [(ff[-1][0],ff[-1][1],'planned F','#de8023'),
        (float(odom[-1]['fork_x_m']),float(odom[-1]['fork_y_m']),'odom F','#b82a80'),
        (matched[-1]['fork_x_m'],matched[-1]['fork_y_m'],'truth F','#287b37')]:
        zoom.scatter(x,y,color=color,s=20,label=label)
    zoom.axvline(target[0],color='black',label='front face')
    zoom.axvline(target[0]-.020,color='gray',ls=':')
    zoom.set_xlim(target[0]-.055,target[0]+.005);zoom.set_ylim(-.025,.025)
    zoom.set_aspect('equal');zoom.set_title('stop detail (m)',fontsize=8);zoom.tick_params(labelsize=6)
    zoom.legend(fontsize=5,loc='upper left')
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(out/f'overlay_planned_odom_truth.{ext}',dpi=150)
    plt.close(fig)
    fig,axs=plt.subplots(4,1,figsize=(11,9),sharex=True)
    for ax,key,label,scale,limits in [(axs[0],'yaw_error_rad','yaw F (deg)',180/math.pi,[-.5,.5]),
        (axs[1],'ey_m','ey F (mm)',1000.,[-20,20]),(axs[2],'gap_m','root gap (mm)',1000.,[15,20,25])]:
        ax.plot([float(r['time_s']) for r in odom],[float(r[key])*scale for r in odom],label='odom')
        ax.plot([r['time_s'] for r in matched],[r[key]*scale for r in matched],label='truth',ls='--');ax.set_ylabel(label)
        for lim in limits:ax.axhline(lim,color='r',ls=':',alpha=.5)
        ax.grid(alpha=.25);ax.legend(fontsize=8)
    axs[-1].plot([float(r['time_s']) for r in odom],[float(r['lift_m'])*1000 for r in odom],label='lift feedback')
    axs[-1].plot([r['time_s'] for r in matched],[r['pallet_rise_m']*1000 for r in matched],label='pallet rise truth')
    axs[-1].axhline(80,color='r',ls=':');axs[-1].set_ylabel('lift / rise (mm)');axs[-1].set_xlabel('trial time (s)');axs[-1].legend();axs[-1].grid(alpha=.25)
    fig.tight_layout();fig.savefig(out/'heading_lateral_gap_lift_truth.png',dpi=150);plt.close(fig)
    print(json.dumps(metrics,indent=2))
if __name__=='__main__':main()
