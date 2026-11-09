"""Simulation RGB-D measurement core. Never consumes entered poses or truth."""
import math
import numpy as np
from pallet_rgbd_filter import front_edge_yaw
from r3_core import compose,errors


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def measured_front(mask,depth,k,camera_translation,camera_quaternion,odom_pose,g,optical=False):
    valid=(mask>.5)&np.isfinite(depth)&(depth>=.6)&(depth<=8.)
    v,u=np.nonzero(valid)
    if len(u)<180:raise ValueError('Chưa đủ điểm Depth hợp lệ trên pallet')
    stride=max(1,len(u)//6000);u=u[::stride];v=v[::stride];d=depth[v,u]
    fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    if min(fx,fy)<=0:raise ValueError('CameraInfo chưa hợp lệ')
    xx=(u-cx)*d/fx;yy=(v-cy)*d/fy
    points=np.column_stack((xx,yy,d)) if optical else np.column_stack((d,-xx,-yy))
    base=points@rotation(camera_quaternion).T+np.asarray(camera_translation)
    top=np.abs(base[:,2]-g['pallet_height'])<=.012
    if np.count_nonzero(top)<180:raise ValueError('Không thấy đủ mặt pallet bằng RGB-D')
    x,y=base[top,0],base[top,1]
    heading=-front_edge_yaw(x,y,max_angle_deg=None)
    n=np.array([math.cos(heading),math.sin(heading)]);t=np.array([-n[1],n[0]])
    projected=np.column_stack((x,y));normal=projected@n;lateral=projected@t
    front=float(np.quantile(normal,.03));center=float(sum(np.quantile(lateral,[.05,.95]))/2)
    point=n*front+t*center
    target=compose(odom_pose,(float(point[0]),float(point[1]),heading))
    e=errors((0.,0.,0.),(float(point[0]),float(point[1]),heading),g)
    return target,dict(distance_m=e['gap_m'],lateral_m=e['ey_m'],yaw_deg=math.degrees(e['yaw_error_rad']),depth_points=int(top.sum()))
