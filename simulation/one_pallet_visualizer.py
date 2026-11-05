#!/usr/bin/env python3
"""One-pallet Gazebo demo: RGB-D measurement, YOLO acquisition, frozen target display."""
from collections import deque
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import torch
from ultralytics import YOLO
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parent))
from one_pallet_case import OnePalletCase
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'camera/detect_astra/scripts'))
from pallet_detection_selection import unique_pallet_indices

# Gazebo robot_core.xacro / camera_astra_pro.xacro extrinsic, for this demo only.
CAMERA_X, CAMERA_Z, CAMERA_PITCH = .367, .800, math.radians(20)

def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def decode_rgb(msg):
    if msg.encoding.lower() not in ('rgb8','bgr8'):raise ValueError(f'RGB {msg.encoding}')
    a=np.frombuffer(msg.data,np.uint8)
    frame=a.reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
    return cv2.cvtColor(frame,cv2.COLOR_RGB2BGR) if msg.encoding.lower()=='rgb8' else frame

def decode_depth(msg):
    if msg.encoding!='32FC1':raise ValueError(f'Depth {msg.encoding}')
    return np.frombuffer(msg.data,np.float32).reshape(msg.height,msg.step//4)[:,:msg.width]

def measure_top_center(mask, depth, info, odom):
    """Approximate top-surface center from registered Depth inside the YOLO mask."""
    if mask.shape!=depth.shape:mask=cv2.resize(mask.astype(np.float32),(depth.shape[1],depth.shape[0]),interpolation=cv2.INTER_NEAREST)
    valid=(mask>.5)&np.isfinite(depth)&(depth>.6)&(depth<8.)
    v,u=np.nonzero(valid)
    if len(u)<80:raise ValueError('insufficient RGB-D mask pixels')
    # Sparse sampling keeps the calculation cheap at 640x480.
    u=u[::max(1,len(u)//3000)];v=v[::max(1,len(v)//3000)]
    d=depth[v,u]
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    if min(fx,fy)<=0:raise ValueError('invalid CameraInfo')
    xx=d; yy=-(u-cx)*d/fx; zz=-(v-cy)*d/fy
    bx=CAMERA_X+math.cos(CAMERA_PITCH)*xx+math.sin(CAMERA_PITCH)*zz
    by=yy
    bz=CAMERA_Z-math.sin(CAMERA_PITCH)*xx+math.cos(CAMERA_PITCH)*zz
    top=(bz>=.08)&(bz<=.14)
    if np.count_nonzero(top)<50:raise ValueError('no top-surface depth pixels')
    # The mask contains a perspective-biased number of near and far pixels.
    # Use the two top-surface extents to approximate the known rectangular top center.
    xlo,xhi=np.quantile(bx[top],[.05,.95])
    ylo,yhi=np.quantile(by[top],[.05,.95])
    x=float((xlo+xhi)/2);y=float((ylo+yhi)/2)
    p=odom.pose.pose.position; yaw=yaw_of(odom.pose.pose.orientation)
    wx=p.x+math.cos(yaw)*x-math.sin(yaw)*y
    wy=p.y+math.sin(yaw)*x+math.cos(yaw)*y
    return wx,wy,int(np.count_nonzero(top))

class Visualizer(Node):
    def __init__(self):
        super().__init__('one_pallet_visualizer')
        self.case=OnePalletCase.from_environment()
        self.yolo_conf=float(os.environ.get('TAI_ONE_PALLET_YOLO_CONF','.25'))
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(RosPath,'/pallet_demo/planned_path',latched)
        self.insertion_pub=self.create_publisher(RosPath,'/pallet_demo/insertion_path',latched)
        self.marker_pub=self.create_publisher(Marker,'/pallet_demo/pre_dock_marker',latched)
        self.lock_pub=self.create_publisher(Bool,'/pallet_demo/target_locked',latched)
        self.target_pub=self.create_publisher(PoseStamped,'/pallet_demo/frozen_target',latched)
        self.image_pub=self.create_publisher(Image,'/pallet_demo/annotated',2)
        self.status_pub=self.create_publisher(String,'/pallet_demo/status',5)
        self.case_pub=self.create_publisher(String,'/pallet_demo/case',latched)
        case_msg=String();case_msg.data=json.dumps({
            'distance_m':self.case.distance_m,'lateral_m':self.case.lateral_m,
            'yaw_deg':self.case.yaw_deg,'yolo_conf':self.yolo_conf})
        self.case_pub.publish(case_msg)
        self.create_subscription(Image,'/camera/image',self.on_image,qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/depth_image',self.on_depth,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.on_info,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.5,self.process_frame)
        self.create_timer(1.,self.publish_status)
        self.latest_image=None;self.latest_depth=None;self.info=None;self.odom=None
        self.rgb_buffer=deque(maxlen=12);self.depth_buffer=deque(maxlen=12)
        self.last_processed_stamp=None;self.last_rgb_stamp=None
        self.frame_intervals=deque(maxlen=20);self.yolo_times=deque(maxlen=10)
        self.measurements=deque(maxlen=8)
        self.verify_measurements=deque(maxlen=4)
        self.verify_mask=None;self.verify_candidate=None;self.verify_start_stamp=None
        self.state='ACQUIRING';self.locked_xy=None;self.rows=None
        self.last_detection='waiting for RGB-D, odom and YOLO'
        self.inference_ms=0.;self.depth_delta_ms=None;self.last_confidence=0.
        torch.set_num_threads(2)
        self.device='0' if torch.cuda.is_available() else 'cpu'
        model_path=Path(__file__).resolve().parents[1]/'camera/detect_astra/models/best_v3.pt'
        self.model=YOLO(str(model_path),task='segment')
        self.yolo_enabled=True
        self.get_logger().info(f'YOLO best_v3.pt on {self.device}; waiting for stable RGB-D target')

    def on_image(self,msg):
        t=stamp_s(msg)
        if self.last_rgb_stamp is not None and 0<t-self.last_rgb_stamp<2:
            self.frame_intervals.append(t-self.last_rgb_stamp)
        self.last_rgb_stamp=t;self.latest_image=msg;self.rgb_buffer.append(msg)
    def on_depth(self,msg):self.latest_depth=msg;self.depth_buffer.append(msg)
    def on_info(self,msg):self.info=msg
    def on_odom(self,msg):self.odom=msg

    def relative(self):
        if self.locked_xy is None or self.odom is None:return None
        p=self.odom.pose.pose.position;yaw=yaw_of(self.odom.pose.pose.orientation)
        dx=self.locked_xy[0]-p.x;dy=self.locked_xy[1]-p.y
        return (math.cos(yaw)*dx+math.sin(yaw)*dy,
                -math.sin(yaw)*dx+math.cos(yaw)*dy,
                -self.case.yaw_rad-yaw)

    def publish_plan(self):
        tx,ty=self.locked_xy
        try:
            self.rows=self.case.path_in_odom(tx,ty)
        except ValueError as exc:
            self.last_detection=f'REPOSITION_REQUIRED: {exc}'
            self.get_logger().warning(self.last_detection)
            return False
        now=self.get_clock().now().to_msg()
        path=RosPath();path.header.frame_id='odom';path.header.stamp=now
        for x,y,yaw,_ in self.rows:
            p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            path.poses.append(p)
        self.path_pub.publish(path)
        ix,iy,iyaw=self.case.insertion_end_in_odom(*self.case.target_in_odom())
        start_x,start_y=self.rows[-1][0],self.rows[-1][1]
        forward=(ix-start_x)*math.cos(iyaw)+(iy-start_y)*math.sin(iyaw)
        ix=start_x+forward*math.cos(iyaw)
        iy=start_y+forward*math.sin(iyaw)
        insertion=RosPath();insertion.header=path.header
        for x,y in ((self.rows[-1][0],self.rows[-1][1]),(ix,iy)):
            pose=PoseStamped();pose.header=path.header
            pose.pose.position.x=x;pose.pose.position.y=y
            pose.pose.orientation.z=math.sin(iyaw/2);pose.pose.orientation.w=math.cos(iyaw/2)
            insertion.poses.append(pose)
        self.insertion_pub.publish(insertion)
        marker=Marker();marker.header=path.header;marker.ns='pre_dock';marker.id=0
        marker.type=Marker.SPHERE;marker.action=Marker.ADD
        marker.pose.position.x=self.rows[-1][0];marker.pose.position.y=self.rows[-1][1];marker.pose.position.z=.08
        marker.pose.orientation.w=1.;marker.scale.x=.08;marker.scale.y=.08;marker.scale.z=.08
        marker.color.r=.05;marker.color.g=.95;marker.color.b=.2;marker.color.a=1.
        self.marker_pub.publish(marker)
        target=PoseStamped();target.header=path.header;target.pose.position.x=tx;target.pose.position.y=ty
        target.pose.orientation.w=1.;self.target_pub.publish(target)
        self.lock_pub.publish(Bool(data=True))
        return True

    def freeze_if_stable(self, mask, stamp):
        m=list(self.measurements)
        if len(m)<3 or m[-1][0]-m[0][0]<.8:return
        xs=[row[1] for row in m];ys=[row[2] for row in m]
        if max(xs)-min(xs)>.04 or max(ys)-min(ys)>.04:return
        tx=float(np.median(xs));ty=float(np.median(ys))
        # The SDF truth is a simulation-only independent check, never used as measurement.
        expected_x,expected_y=self.case.target_in_odom()
        if math.hypot(tx-expected_x,ty-expected_y)>.05:
            self.last_detection=f'RGB-D estimate fails Gazebo truth check: ({tx:.2f},{ty:.2f}) m'
            return
        self.yolo_enabled=False
        self.state='VERIFYING_RGBD'
        self.verify_mask=mask.copy();self.verify_candidate=(tx,ty)
        self.verify_start_stamp=stamp;self.verify_measurements.clear()
        self.last_detection=f'YOLO OFF; verifying new RGB-D after {stamp:.2f}s'
        self.get_logger().info('YOLO inference OFF; awaiting fresh post-stop RGB-D measurements')

    def verify_fresh_depth(self, stamp, depth_msg):
        if stamp<=self.verify_start_stamp or self.odom is None or self.info is None:return
        if abs(self.odom.twist.twist.linear.x)>.005 or abs(self.odom.twist.twist.angular.z)>.01:
            self.state='ACQUIRING';self.yolo_enabled=True;self.measurements.clear()
            self.last_detection='robot moved during verification; reacquiring';return
        try:
            tx,ty,n=measure_top_center(self.verify_mask,decode_depth(depth_msg),self.info,self.odom)
        except Exception as exc:
            self.last_detection=f'RGB-D verification waiting: {str(exc)[:70]}'
            return
        cx,cy=self.verify_candidate
        if math.hypot(tx-cx,ty-cy)>.03:
            self.state='ACQUIRING';self.yolo_enabled=True;self.measurements.clear()
            self.last_detection='post-stop RGB-D moved >30 mm; reacquiring';return
        self.verify_measurements.append((stamp,tx,ty))
        self.last_detection=f'YOLO OFF; fresh RGB-D {len(self.verify_measurements)}/2 ({tx:.3f},{ty:.3f}) m n={n}'
        if len(self.verify_measurements)>=2 and self.verify_measurements[-1][0]>self.verify_measurements[0][0]:
            all_x=[r[1] for r in self.verify_measurements]
            all_y=[r[2] for r in self.verify_measurements]
            fx=float(np.median(all_x));fy=float(np.median(all_y))
            self.locked_xy=(fx,fy)
            if not self.publish_plan():
                self.state='REPOSITION_REQUIRED';self.model=None
                return
            self.state='FROZEN_TARGET';self.model=None
            if torch.cuda.is_available():torch.cuda.empty_cache()
            self.last_detection=f'locked YOLO {self.last_confidence:.2f}; verified RGB-D=({fx:.3f},{fy:.3f}) m'
            self.get_logger().info(f'FROZEN_TARGET; YOLO OFF; target odom=({fx:.3f},{fy:.3f}) m')

    def publish_status(self):
        dt_ms=1000*sum(self.frame_intervals)/len(self.frame_intervals) if self.frame_intervals else 0.
        fps=1000/dt_ms if dt_ms else 0.
        yolo_fps=((len(self.yolo_times)-1)/(self.yolo_times[-1]-self.yolo_times[0])
                  if self.yolo_enabled and len(self.yolo_times)>1 else 0.)
        rel=self.relative()
        rel_text=(f'robot->pallet X={rel[0]:.3f} Y={rel[1]:+.3f} m heading_error={math.degrees(rel[2]):+.1f}deg'
                  if rel else 'robot->pallet: waiting for valid RGB-D lock')
        msg=String();msg.data=(f'{self.state}; start distance={self.case.distance_m:.2f}m '
            f'lateral={self.case.lateral_m:+.2f}m yaw={self.case.yaw_deg:+.1f}deg '
            f'YOLO conf>={self.yolo_conf:.2f}; {self.last_detection}; {rel_text}; '
            f'camera FPS={fps:.1f} frame_dt={dt_ms:.0f}ms; YOLO {"ON" if self.yolo_enabled else "OFF"} FPS={yolo_fps:.1f} '
            f'inference={self.inference_ms:.0f}ms depth_sync={self.depth_delta_ms}ms')
        self.status_pub.publish(msg)

    def process_frame(self):
        msg=None;depth_msg=None
        if self.state=='FROZEN_TARGET':
            msg=self.latest_image
        for rgb in reversed(self.rgb_buffer):
            if self.state=='FROZEN_TARGET':break
            t=stamp_s(rgb)
            if t==self.last_processed_stamp:break
            if not self.depth_buffer:continue
            near=min(self.depth_buffer,key=lambda d:abs(stamp_s(d)-t))
            if abs(stamp_s(near)-t)<=.08:
                msg=rgb;depth_msg=near;break
        if msg is None:
            if self.state=='ACQUIRING':
                self.last_detection='waiting for matched RGB/Depth frame'
            return
        stamp=stamp_s(msg)
        if stamp==self.last_processed_stamp:return
        self.last_processed_stamp=stamp
        try:frame=decode_rgb(msg)
        except Exception as exc:self.last_detection=str(exc);return
        if self.state=='ACQUIRING':
            if self.odom is None or depth_msg is None or self.info is None:
                self.last_detection='waiting for RGB-D/CameraInfo/odom'
            elif abs(self.odom.twist.twist.linear.x)>.005 or abs(self.odom.twist.twist.angular.z)>.01:
                self.measurements.clear();self.last_detection='robot moving; acquisition reset'
            else:
                t0=time.monotonic()
                try:
                    result=self.model.predict(frame,conf=self.yolo_conf,iou=.5,imgsz=640,
                                              retina_masks=True,device=self.device,verbose=False)[0]
                    now=time.monotonic();self.inference_ms=(now-t0)*1000
                    self.yolo_times.append(now)
                    boxes=result.boxes.xyxy.cpu().numpy();classes=result.boxes.cls.cpu().numpy()
                    conf=result.boxes.conf.cpu().numpy()
                    indices=unique_pallet_indices(boxes,classes,conf,result.names)
                    self.last_confidence=max((float(conf[i]) for i in indices),default=0.)
                    if len(indices)!=1 or result.masks is None:
                        self.measurements.clear();self.last_detection=f'YOLO found {len(indices)} pallets'
                    else:
                        i=indices[0];mask=result.masks.data[i].cpu().numpy()
                        depth=decode_depth(depth_msg)
                        self.depth_delta_ms=round(1000*(stamp-stamp_s(depth_msg)),1)
                        tx,ty,n=measure_top_center(mask,depth,self.info,self.odom)
                        self.measurements.append((stamp,tx,ty))
                        self.last_detection=f'YOLO {result.names[int(classes[i])]} {conf[i]:.2f}; RGB-D=({tx:.3f},{ty:.3f}) m n={n}'
                        if mask.shape!=frame.shape[:2]:mask=cv2.resize(mask,(msg.width,msg.height),interpolation=cv2.INTER_NEAREST)
                        region=mask>.5;frame[region]=(frame[region]*.55+np.array([0,255,0])*.45).astype(np.uint8)
                        x1,y1,x2,y2=map(int,boxes[i]);cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                        self.freeze_if_stable(mask,stamp)
                except Exception as exc:
                    self.measurements.clear();self.last_detection=f'acquisition rejected: {str(exc)[:100]}'
        elif self.state=='VERIFYING_RGBD':
            self.verify_fresh_depth(stamp,depth_msg)
        self.draw_overlay(frame)
        out=Image();out.header=msg.header;out.height=msg.height;out.width=msg.width
        out.encoding='bgr8';out.is_bigendian=0;out.step=msg.width*3;out.data=frame.tobytes()
        self.image_pub.publish(out)

    def draw_overlay(self,frame):
        dt_ms=1000*sum(self.frame_intervals)/len(self.frame_intervals) if self.frame_intervals else 0.
        fps=1000/dt_ms if dt_ms else 0.
        rel=self.relative()
        lines=[f'{self.state} | YOLO {"ON" if self.yolo_enabled else "OFF"} | best_v3.pt',
               f'Start: distance {self.case.distance_m:.2f}m  lateral {self.case.lateral_m:+.2f}m  yaw {self.case.yaw_deg:+.1f}deg',
               self.last_detection[:80],
               f'RGB FPS {fps:.1f} | frame interval {dt_ms:.0f} ms | inference {self.inference_ms:.0f} ms',
               (f'Robot -> pallet: X {rel[0]:.3f} m  Y {rel[1]:+.3f} m  heading error {math.degrees(rel[2]):+.1f} deg'
                if rel else 'Robot -> pallet: waiting for RGB-D target lock')]
        cv2.rectangle(frame,(0,0),(frame.shape[1],140),(18,18,18),-1)
        for j,line in enumerate(lines):
            cv2.putText(frame,line,(7,22+j*27),cv2.FONT_HERSHEY_SIMPLEX,.52,
                        (0,255,170) if j==0 else (255,255,255),1,cv2.LINE_AA)

def main():
    rclpy.init();node=Visualizer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
