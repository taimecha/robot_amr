#!/usr/bin/env python3
"""Offline RGB-D frame/pose replay for the stationary acquisition proposal."""
from collections import Counter, deque
import csv
import json
import math
import statistics
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def circular_location(angles):
    return math.atan2(statistics.median([math.sin(a) for a in angles]),
                      statistics.median([math.cos(a) for a in angles]))


def depth_stats(samples, original_area, low=.6, high=8.):
    valid=[v for v in samples if math.isfinite(v) and low<=v<=high]
    if original_area<=0 or len(valid)<60:
        return None
    med=statistics.median(valid)
    mad=statistics.median(abs(v-med) for v in valid)
    return {'valid_ratio':len(valid)/original_area,'valid_count':len(valid),
            'median_m':med,'mad_m':mad}


class StationaryGate:
    def __init__(self):
        self.last_stamp=None
        self.target_id=None
        self.rows=deque(maxlen=30)
        self.reasons=Counter()

    def add(self, row):
        stamp=row['stamp_s']
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.reasons['duplicate_or_out_of_order']+=1
            return False
        self.last_stamp=stamp
        if row['target_id']!=self.target_id and self.target_id is not None:
            self.rows.clear()
            self.reasons['target_change_reset']+=1
        self.target_id=row['target_id']
        checks=(('stale',row['now_s']-stamp>.25),
                ('sync',abs(row['depth_stamp_s']-stamp)>.05),
                ('blur',row['sharpness']<80),
                ('clipping',row['clip_fraction']>.12),
                ('camera_motion',row['camera_motion_px']>1.5),
                ('moving',not row['stationary']))
        for name,bad in checks:
            if bad:
                self.reasons[name]+=1
                self.rows.clear() if name=='moving' else None
                return False
        left=depth_stats(row['left_depth'],row['left_area'])
        right=depth_stats(row['right_depth'],row['right_area'])
        if not left or not right:
            self.reasons['depth_samples']+=1
            return False
        if min(left['valid_ratio'],right['valid_ratio'])<.70:
            self.reasons['depth_coverage']+=1
            return False
        if max(left['mad_m'],right['mad_m'])>.02:
            self.reasons['depth_mad']+=1
            return False
        self.rows.append((stamp,row['x_m'],row['y_m'],row['yaw_rad']))
        self.reasons['accepted']+=1
        return True

    def pose(self):
        rows=list(self.rows)
        if len(rows)<6 or rows[-1][0]-rows[0][0]<1.:
            return None
        xs=[r[1] for r in rows]; ys=[r[2] for r in rows]; yaws=[r[3] for r in rows]
        return {'x_m':statistics.median(xs),'y_m':statistics.median(ys),
                'yaw_rad':circular_location(yaws),'oldest_s':rows[0][0],
                'newest_s':rows[-1][0], 'samples':len(rows),
                'x_spread_m':max(xs)-min(xs),'y_spread_m':max(ys)-min(ys),
                'yaw_spread_rad':max(abs(wrap(a-circular_location(yaws))) for a in yaws)*2}


def synthetic_frames():
    rows=[]
    for i in range(40):
        t=i*.2
        # Known absolute +10 mm calibration bias remains after averaging.
        noise=.004*math.sin(i*1.7)
        depth=[1.2+.003*math.sin(i+j) for j in range(85)]+[0.]*15
        rows.append({'stamp_s':t,'now_s':t+.04,'depth_stamp_s':t+.015,
                     'target_id':'A','sharpness':120,'clip_fraction':.02,
                     'camera_motion_px':.2,'stationary':True,
                     'left_depth':depth,'right_depth':depth,'left_area':100,'right_area':100,
                     'x_m':1.21+noise,'y_m':.005+.002*math.cos(i),
                     'yaw_rad':math.radians(.3*math.sin(i))})
    rows[4]['sharpness']=30
    rows[7]['depth_stamp_s']+=.1
    rows[11]['left_depth']=[0.]*55+[1.2]*45
    rows[15]['camera_motion_px']=3.
    rows.insert(20,dict(rows[19]))  # exact duplicate stamp
    return rows


def quality_only(rows):
    seen=set(); out=[]
    for r in rows:
        if r['stamp_s'] in seen: continue
        seen.add(r['stamp_s'])
        if (r['now_s']-r['stamp_s']<=.25
                and abs(r['depth_stamp_s']-r['stamp_s'])<=.05
                and r['sharpness']>=80 and r['clip_fraction']<=.12
                and r['camera_motion_px']<=1.5 and r['stationary']):
            out.append(r)
    return out


def main(output=Path('simulation/output')):
    output.mkdir(parents=True,exist_ok=True)
    frames=synthetic_frames(); gate=StationaryGate()
    accepted=[]
    for f in frames:
        if gate.add(f): accepted.append(f)
    pose=gate.pose()
    truth={'x_m':1.2,'y_m':0.,'yaw_rad':0.}
    variants={}
    for name,rows in [('raw',frames),('gate_only',quality_only(frames)),
                      ('gate_depth',accepted),('gate_depth_pose',accepted)]:
        if name=='gate_depth_pose':
            estimate=pose
        else:
            estimate={'x_m':statistics.mean(r['x_m'] for r in rows),
                      'y_m':statistics.mean(r['y_m'] for r in rows),
                      'yaw_rad':circular_location([r['yaw_rad'] for r in rows])}
        variants[name]={'samples':len(rows),'x_absolute_error_m':abs(estimate['x_m']-truth['x_m']),
                        'y_absolute_error_m':abs(estimate['y_m']-truth['y_m']),
                        'yaw_absolute_error_deg':abs(math.degrees(wrap(estimate['yaw_rad']-truth['yaw_rad'])))}
    report={'synthetic_truth':truth,'reason_counts':dict(gate.reasons),
            'accepted_pose':pose,'ab_variants':variants,
            'absolute_x_budget_m':.005,
            'absolute_x_budget_pass':variants['gate_depth_pose']['x_absolute_error_m']<=.005,
            'warning':'Synthetic truth only; repeatability does not remove +10 mm calibration bias.'}
    (output/'quality_report.json').write_text(json.dumps(report,indent=2)+'\n')
    with (output/'quality_frames.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(('stamp_s','x_m','y_m','yaw_rad','sharpness','motion_px'))
        writer.writerows((r['stamp_s'],r['x_m'],r['y_m'],r['yaw_rad'],r['sharpness'],
                          r['camera_motion_px']) for r in frames)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
