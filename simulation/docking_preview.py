#!/usr/bin/env python3
"""Offline docking concept preview. Pure Python; no ROS imports or publishers."""
import argparse
import csv
import json
import math
from pathlib import Path
from collections import deque


def quintic(p0, v0, p1, v1, u):
    # p''(0)=p''(1)=0, so curvature joins a straight segment continuously.
    a0, a1 = p0, v0
    a3 = 10*(p1-p0)-6*v0-4*v1
    a4 = -15*(p1-p0)+8*v0+7*v1
    a5 = 6*(p1-p0)-3*v0-3*v1
    p = a0+a1*u+a3*u**3+a4*u**4+a5*u**5
    d = a1+3*a3*u*u+4*a4*u**3+5*a5*u**4
    dd = 6*a3*u+12*a4*u*u+20*a5*u**3
    return p, d, dd


def candidate(lateral_m, yaw_deg, approach_m, tangent_scale=1.0):
    """Illustrative S->A curve, then straight A->F; F is pre-dock only."""
    yaw = math.radians(yaw_deg)
    a = .75*approach_m
    tangent = tangent_scale*a
    rows = []
    for i in range(201):
        u = i/200
        x, dx, ddx = quintic(0, tangent*math.cos(yaw), a, tangent, u)
        y, dy, ddy = quintic(lateral_m, tangent*math.sin(yaw), 0, 0, u)
        speed2 = dx*dx+dy*dy
        kappa = (dx*ddy-dy*ddx)/speed2**1.5 if speed2 > 1e-9 else float('inf')
        rows.append((x, y, math.atan2(dy, dx), kappa, math.sqrt(speed2)))
    for i in range(1, 51):
        rows.append((a+(approach_m-a)*i/50, 0., 0., 0., 1.))
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(rows,rows[1:]))
    kmax = max(abs(row[3]) for row in rows)
    # Example corridor only. Actual pallet holes and swept collision are unknown.
    corridor_half_width = .60
    footprint_half_width = .25
    lateral_clearance = min(corridor_half_width-footprint_half_width-abs(y)
                            for _,y,_,_,_ in rows)
    forward = all(b[0] > a[0]-1e-7 for a,b in zip(rows,rows[1:]))
    return {'rows': rows, 'length_m': length, 'max_abs_kappa_1pm': kmax,
            'example_corridor_clearance_m': lateral_clearance,
            'illustrative_feasible': bool(forward and kmax <= 2.5 and lateral_clearance >= .05),
            'reason': 'illustrative geometry/curvature only; actual collision and insertion not verified'}


class PreviewSupervisor:
    """Small executable state model for the offline preview, not a ROS supervisor."""
    def __init__(self, accuracy_validated=False, mode='FROZEN_TARGET'):
        if mode not in ('FROZEN_TARGET','DIAGNOSTIC'):
            raise ValueError('invalid mode')
        self.state = 'ACQUIRING'
        self.mode = mode
        self.samples = deque(maxlen=20)
        self.last_stamp = None
        self.target_id = None
        self.stop_requested_at = None
        self.frozen = False
        self.accuracy_validated = accuracy_validated
        self.reason = 'waiting for quality evidence'

    def sample(self, stamp, target_id, x, y, yaw, good, stationary=True):
        if self.state not in ('ACQUIRING', 'VERIFYING_RGBD'):
            return
        if not stationary or target_id != self.target_id and self.target_id is not None:
            self.samples.clear()
            self.state = 'ACQUIRING'
            self.reason = 'robot moved or target changed; evidence reset'
        self.target_id = target_id
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reason = 'duplicate or old frame'
            return
        self.last_stamp = stamp
        if not good or not stationary:
            self.samples.clear()
            self.reason = 'invalid frame or robot motion'
            return
        self.samples.append((stamp, x, y, yaw))

    def tick(self, now, detector_disabled=False, cancel=False, sensor_fault=False):
        if cancel:
            self.state, self.reason = 'CANCELED', 'operator canceled'
        elif sensor_fault:
            self.state, self.reason = 'SENSOR_FAULT', 'sensor fault'
        elif self.state == 'STOPPING_YOLO':
            if detector_disabled:
                self.samples.clear()
                self.state, self.reason = 'VERIFYING_RGBD', 'fresh post-stop RGB-D required'
        elif self.state in ('ACQUIRING', 'VERIFYING_RGBD'):
            needed, span = (6, 1.) if self.state == 'ACQUIRING' else (4, .6)
            rows = list(self.samples)
            if (len(rows) >= needed and rows[-1][0]-rows[0][0] >= span-.001
                    and now-rows[-1][0] <= .25
                    and max(r[1] for r in rows)-min(r[1] for r in rows) <= .03
                    and max(r[2] for r in rows)-min(r[2] for r in rows) <= .02
                    and max(r[3] for r in rows)-min(r[3] for r in rows) <= math.radians(2)):
                if self.state == 'ACQUIRING':
                    if self.accuracy_validated:
                        self.state, self.reason = 'STOPPING_YOLO', 'await disabled heartbeat'
                        self.stop_requested_at = now
                    else:
                        self.reason = 'absolute accuracy budget not validated'
                else:
                    self.state, self.reason = 'READY', 'quality passed; await operator'
                    self.frozen = self.mode == 'FROZEN_TARGET'

    def start_motion(self):
        if self.state != 'READY':
            return False
        self.state = 'DIAGNOSTIC_MOTION' if self.mode == 'DIAGNOSTIC' else 'FOLLOW_CURVE'
        return True

    @property
    def pallet_tracking(self):
        return not self.frozen and self.state not in ('CANCELED','SENSOR_FAULT')


def acquisition_events():
    """Drive the preview supervisor with fake frames; wait longer than five seconds."""
    # This nominal branch assumes an independent calibration check passed.
    supervisor = PreviewSupervisor(accuracy_validated=True)
    events = []
    for i in range(121):
        t = round(i*.1, 1)
        if i % 2 == 0 and t < 8.0:
            supervisor.sample(t,'pallet-A',1.2,.01,0.,False)
        if i % 2 == 0 and t >= 8.0 and supervisor.state in ('ACQUIRING','VERIFYING_RGBD'):
            supervisor.sample(t,'pallet-A',1.2+.002*math.sin(t),.01+.001*math.cos(t),
                              math.radians(.2*math.sin(t)),True)
        supervisor.tick(t, detector_disabled=(supervisor.stop_requested_at is not None
                        and t-supervisor.stop_requested_at >= .4))
        events.append({'time_s': t, 'state': supervisor.state, 'reason': supervisor.reason,
                       'motor_command': False, 'pallet_tracking': supervisor.pallet_tracking})
    return events


def fault_cases():
    cases=[]
    for case, fault in [('stale_odom','odom age > 0.20 s'),
                        ('stale_depth','obstacle data stale'),
                        ('obstacle','obstacle in swept corridor'),
                        ('odom_reset','odom epoch changed'),
                        ('second_publisher','command owner conflict'),
                        ('cancel','operator canceled')]:
        follower = ShadowFollower([(0.,0.,0.,0.),(1.,0.,0.,0.)])
        follower.start(True)
        follower.stop(fault)
        follower.step(0.,0.,0., restored=True)
        cases.append({'case':case,'initial':'FOLLOW_CURVE','fault':fault,
                      'result':follower.state,'auto_resume':False,'motor_command':False})
    return cases


class ShadowFollower:
    """Minimal unicycle path follower with fake odometry and a latched stop."""
    def __init__(self, path):
        self.path=path
        self.state='READY'
        self.index=0
        self.v=0.
        self.reason='operator start required'

    def start(self, operator_start):
        if operator_start and self.state=='READY':
            self.state='FOLLOW_CURVE'

    def stop(self, reason):
        self.state='STOP_LATCHED'
        self.v=0.
        self.reason=reason

    def step(self, x, y, yaw, restored=False):
        if self.state not in ('FOLLOW_CURVE','FOLLOW_STRAIGHT'):
            return 0.,0.
        nearest=min(range(self.index,min(self.index+35,len(self.path))),
                    key=lambda i:(self.path[i][0]-x)**2+(self.path[i][1]-y)**2)
        self.index=max(self.index,nearest)
        target=min(len(self.path)-1,self.index+12)
        tx,ty,_,_=self.path[target]
        dist=math.hypot(tx-x,ty-y)
        alpha=math.atan2(ty-y,tx-x)-yaw
        curvature=2*math.sin(alpha)/max(.10,dist)
        self.v=min(.10,self.v+.08*.05)
        w=max(-.20,min(.20,self.v*curvature))
        if self.index>=200:
            self.state='FOLLOW_STRAIGHT'
        if math.hypot(self.path[-1][0]-x,self.path[-1][1]-y)<.02:
            self.state='WAIT_INSERTION_VERIFICATION'
            self.reason='pre-dock reached; insertion remains unverified'
            return 0.,0.
        return self.v,w


def shadow_replay(path):
    x,y,yaw,_=path[0]
    follower=ShadowFollower(path)
    follower.start(True)
    rows=[]
    for i in range(1000):
        v,w=follower.step(x,y,yaw)
        rows.append({'time_s':round(i*.05,2),'state':follower.state,
                     'x_m':x,'y_m':y,'yaw_rad':yaw,'proposed_v_mps':v,
                     'proposed_w_rps':w,'motor_command_sent':False})
        if follower.state=='WAIT_INSERTION_VERIFICATION':
            break
        x+=v*math.cos(yaw)*.05
        y+=v*math.sin(yaw)*.05
        yaw+=w*.05
    return rows


def svg_preview(path, rows, events):
    width, height = 980, 560
    def sx(x): return 75+720*x/1.4
    def sy(y): return 220-440*y
    points = ' '.join(f'{sx(x):.1f},{sy(y):.1f}' for x,y,*_ in rows)
    colors = {'ACQUIRING':'#f7bf50','STOPPING_YOLO':'#e8864c',
              'VERIFYING_RGBD':'#69a8dd','READY':'#58b88a'}
    rects=[]
    for e in events[:-1]:
        rects.append(f'<rect x="{75+68*e["time_s"]:.1f}" y="470" width="6.8" height="26" fill="{colors[e["state"]]}"/>')
    path.write_text('''<svg xmlns="http://www.w3.org/2000/svg" width="980" height="560" viewBox="0 0 980 560">
<rect width="980" height="560" fill="#101827"/><g fill="#e5edf8" font-family="sans-serif">
<text x="75" y="43" font-size="24">Docking preview — laptop, simulated inputs</text>
<text x="75" y="75" font-size="14">Curve to pre-dock only; no pallet contact, motor output or lift</text>
<text x="75" y="395" font-size="17">Acquisition timeline (s)</text>
<text x="75" y="525" font-size="13">0</text><text x="415" y="525" font-size="13">5</text><text x="755" y="525" font-size="13">10</text>
<text x="75" y="448" font-size="13">Waiting beyond 5 s is allowed; READY still awaits operator</text>
<text x="78" y="355" font-size="13">S: initial pose</text><text x="690" y="355" font-size="13">F: pre-dock</text>
</g><rect x="75" y="66" width="720" height="285" fill="#182b3d"/>
<path d="M75 220 H795" stroke="#47647a" stroke-dasharray="5 5"/>
<path d="M75 66 H795 M75 351 H795" stroke="#a2755b" stroke-dasharray="8 5"/>
<polyline points="'''+points+'''" fill="none" stroke="#6de0c7" stroke-width="4"/>
<circle cx="'''+str(sx(rows[0][0]))+'''" cy="'''+str(sy(rows[0][1]))+'''" r="7" fill="#f7bf50"/>
<circle cx="'''+str(sx(rows[-1][0]))+'''" cy="'''+str(sy(rows[-1][1]))+'''" r="7" fill="#58b88a"/>
'''+''.join(rects)+'''</svg>''')


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    events = acquisition_events()
    chosen = candidate(.05, 5., 1.2)
    shadow=shadow_replay([r[:4] for r in chosen['rows']])
    grid=[]
    for distance in (.6, 1.0, 1.4):
        for side in (-1,1):
            for lateral in (.02,.05,.10):
                for yaw in (2,5,10):
                    result = candidate(side*lateral, side*yaw, distance)
                    grid.append({'approach_m':distance,'lateral_m':side*lateral,
                                 'yaw_deg':side*yaw,'length_m':round(result['length_m'],4),
                                 'max_abs_kappa_1pm':round(result['max_abs_kappa_1pm'],4),
                                 'example_corridor_clearance_m':round(result['example_corridor_clearance_m'],4),
                                 'illustrative_feasible':result['illustrative_feasible']})
    with (output/'path.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(('x_m','y_m','yaw_rad','kappa_1pm'))
        writer.writerows((x,y,yaw,kappa) for x,y,yaw,kappa,_ in chosen['rows'])
    with (output/'grid.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(grid[0])); writer.writeheader(); writer.writerows(grid)
    with (output/'timeline.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(events[0])); writer.writeheader(); writer.writerows(events)
    with (output/'shadow_follower.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(shadow[0])); writer.writeheader(); writer.writerows(shadow)
    summary={'mode':'OFFLINE_SIMULATION_ONLY','robot_command_publishers':0,
             'mode_contracts':{'DIAGNOSTIC':'pallet RGB-D remains active during motion; measurements do not steer',
                               'FROZEN_TARGET':'YOLO and pallet tracking stop before motion; odom/IMU follow fixed path'},
             'example_path':{k:v for k,v in chosen.items() if k!='rows'},
             'acquisition_ready_s':next((e['time_s'] for e in events if e['state']=='READY'),None),
             'nominal_accuracy_assumed_validated':True,
             'ready_requires_operator_start':True,
             'fault_cases':fault_cases(),'grid_cases':len(grid),
             'grid_illustrative_feasible':sum(row['illustrative_feasible'] for row in grid),
             'shadow_final_state':shadow[-1]['state'],
             'shadow_final_lateral_error_m':round(shadow[-1]['y_m'],4),
             'shadow_duration_s':shadow[-1]['time_s'],
             'limitations':['fake sensors and odometry','synthetic corridor','no Pi runtime configuration',
                            'no measured minimum turn radius or pallet hole dimensions',
                            'no Gazebo contact or independent insertion verification']}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    svg_preview(output/'preview.svg',chosen['rows'],events)
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('simulation/output'))
    run(parser.parse_args().output)
