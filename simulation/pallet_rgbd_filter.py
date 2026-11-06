"""Robust front-edge heading and temporal lock for the one-pallet RGB-D demo."""
from collections import deque
import math

import numpy as np


def front_edge_yaw(forward, lateral):
    """Face yaw from the nearest top-deck edge; left-farther is positive."""
    forward=np.asarray(forward,dtype=float)
    lateral=np.asarray(lateral,dtype=float)
    valid=np.isfinite(forward)&np.isfinite(lateral)
    forward,lateral=forward[valid],lateral[valid]
    if len(forward)<180:
        raise ValueError('too few pallet top points for heading')
    lo,hi=np.quantile(lateral,[.08,.92])
    if hi-lo<.19:
        raise ValueError('front edge is too narrow for heading')
    edges=np.linspace(lo,hi,13)
    ys=[];xs=[]
    for a,b in zip(edges[:-1],edges[1:]):
        values=forward[(lateral>=a)&(lateral<b)]
        if len(values)>=12:
            ys.append((a+b)/2)
            xs.append(float(np.quantile(values,.08)))
    if len(xs)<9:
        raise ValueError('front edge has too few supported bins')
    ys=np.asarray(ys);xs=np.asarray(xs)
    matrix=np.column_stack((ys,np.ones(len(ys))))
    weights=np.ones(len(xs))
    for _ in range(5):
        root=np.sqrt(weights)
        slope,offset=np.linalg.lstsq(matrix*root[:,None],xs*root,rcond=None)[0]
        residual=xs-(slope*ys+offset)
        scale=max(.003,1.4826*float(np.median(abs(residual-np.median(residual)))))
        weights=np.minimum(1.,1.5*scale/np.maximum(abs(residual),1e-8))
    if not np.isfinite(slope) or np.median(abs(residual))>.012:
        raise ValueError('front edge depth is not one stable line')
    angle=math.atan(float(slope))
    if abs(angle)>math.radians(20):
        raise ValueError('measured pallet heading outside 20 degrees')
    return angle


class StablePoseWindow:
    """Reject isolated RGB-D poses and lock only a fresh, consistent window."""
    def __init__(self, size=8, minimum=5, min_span_s=.8):
        self.rows=deque(maxlen=size)
        self.minimum=minimum
        self.min_span_s=min_span_s

    def clear(self):
        self.rows.clear()

    def add(self, stamp, x, y, heading):
        if not all(math.isfinite(v) for v in (stamp,x,y,heading)):
            self.clear();return None
        if self.rows and (stamp<=self.rows[-1][0] or stamp-self.rows[-1][0]>3.):
            self.clear()
        if self.rows:
            med=np.median(np.array(self.rows)[:,1:],axis=0)
            if (math.hypot(x-med[0],y-med[1])>.05
                    or abs(math.atan2(math.sin(heading-med[2]),math.cos(heading-med[2])))>math.radians(5)):
                return None
        self.rows.append((stamp,x,y,heading))
        if len(self.rows)<self.minimum or self.rows[-1][0]-self.rows[0][0]<self.min_span_s:
            return None
        values=np.asarray(self.rows)
        med=np.median(values[:,1:],axis=0)
        inlier=((np.hypot(values[:,1]-med[0],values[:,2]-med[1])<=.025)
                &(np.abs(values[:,3]-med[2])<=math.radians(2)))
        if inlier.sum()<self.minimum:
            return None
        accepted=values[inlier]
        if accepted[-1,0]-accepted[0,0]<self.min_span_s:
            return None
        result=np.median(accepted[:,1:],axis=0)
        return tuple(float(v) for v in result)
