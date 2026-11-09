#!/usr/bin/env python3
"""A read-only ROS camera worker; annotated image and measured results for Tk GUI."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'camera/detect_astra/scripts'))
from pallet_alignment_core import OdomHistory,OdomSample
from pallet_detection_selection import unique_pallet_indices
from pallet_rgbd_filter import StablePoseWindow
from r3_vision_core import measured_front
from r3_core import digest

def atomic(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2));tmp.replace(path)

def save_frame(folder,name,frame):
    tmp=folder/(name+'.tmp.jpg');cv2.imwrite(str(tmp),frame);tmp.replace(folder/(name+'.jpg'))


def annotate(frame,detections):
    frame=frame.copy()
    for d in detections:
        x1,y1,x2,y2=map(int,d['bbox']);cv2.rectangle(frame,(x1,y1),(x2,y2),(50,230,100),2)
        cv2.putText(frame,f"{d['label']} {d['confidence']:.2f}",(x1,max(20,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.5,(50,230,100),1)
    if not detections:cv2.putText(frame,'YOLO: pallet not detected',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(20,70,230),1)
    return frame

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def decode(m):
    return np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()

class Camera(Node):
    def __init__(self,args):
        super().__init__('r3_detection_ui')
        self.args=args;self.g=json.loads((ROOT/'simulation/r3/fixture.json').read_text());self.case=json.loads(args.case.read_text())
        self.result=dict(state='LOADING_MODEL',case_id=self.case['case_id'],provenance='CAMERA_RGBD_SIMULATION',message='Đang nạp YOLO best_v3.pt',yolo_enabled=False,pallet_measurement_enabled=False)
        self.publish()
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2);self.device='0' if torch.cuda.is_available() else 'cpu'
        self.model=YOLO(str(ROOT/'camera/detect_astra/models/best_v3.pt'),task='segment')
        self.result.update(state='ACQUIRE_YOLO',yolo_enabled=True,pallet_measurement_enabled=True,message='Chờ ảnh camera mô phỏng')
        self.rgb=None;self.rgbs=deque(maxlen=20);self.depth=deque(maxlen=20);self.info=None;self.history=OdomHistory();self.odom=None;self.last=None;self.mask=None
        self.stable=StablePoseWindow();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.create_subscription(Image,'/camera/image',self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/depth_image',lambda m:self.depth.append(m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera_info',lambda m:setattr(self,'info',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.4,self.tick)
        self.publish()

    def on_rgb(self,m):
        self.rgb=m;self.rgbs.append(m)

    def on_odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.history.add(OdomSample(stamp(m),p.x,p.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z));self.odom=m

    def publish(self):atomic(self.args.output/'vision.json',self.result)

    def tick(self):
        try:
            control=json.loads((self.args.output/'vision_control.json').read_text()) if (self.args.output/'vision_control.json').exists() else {}
            if control.get('measure') is False:
                self.result.update(state='MEASUREMENT_OFF',yolo_enabled=False,pallet_measurement_enabled=False,message='Đã tắt YOLO / đo pallet; giữ kết quả cuối')
                self.publish()
            if self.rgb is None:return
            rgb=self.rgb
            measuring=self.result['pallet_measurement_enabled']
            if measuring and not self.result['yolo_enabled'] and self.depth:
                pairs=[(m,min(self.depth,key=lambda d:abs(stamp(d)-stamp(m)))) for m in self.rgbs if self.last is None or stamp(m)>self.last]
                pairs=[(m,d) for m,d in pairs if abs(stamp(d)-stamp(m))<=.08]
                if pairs:rgb,dmsg=max(pairs,key=lambda pair:stamp(pair[0]))
                else:raise ValueError('Chờ cặp RGB/Depth cùng thời điểm')
            t=stamp(rgb)
            if t==self.last:return
            frame=decode(rgb)
            if rgb.encoding.lower()=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            if measuring:
                if self.odom and (abs(self.odom.twist.twist.linear.x)>.005 or abs(self.odom.twist.twist.angular.z)>.01):
                    self.stable.clear()
                    raise ValueError('Xe đang chạy: không đo/khóa pallet')
                if self.result['yolo_enabled']:
                    prediction=self.model.predict(frame,conf=.25,imgsz=640,retina_masks=True,device=self.device,verbose=False)[0]
                    boxes=prediction.boxes.xyxy.cpu().numpy();classes=prediction.boxes.cls.cpu().numpy();confidence=prediction.boxes.conf.cpu().numpy()
                    indices=unique_pallet_indices(boxes,classes,confidence,prediction.names)
                    self.result['detections']=[dict(label=prediction.names[int(c)],confidence=float(cf),bbox=b.tolist()) for b,c,cf in zip(boxes,classes,confidence)]
                    self.result['detection_stamp_s']=t
                    self.result['pallet_count']=len(indices);self.result['confidence_threshold']=.25
                    self.result['image_size']=[int(frame.shape[1]),int(frame.shape[0])]
                    save_frame(self.args.output,'detection',annotate(frame,self.result['detections']))
                    if len(indices)==0:raise ValueError('YOLO chưa xác nhận pallet trong ảnh; chưa thể đo vị trí RGB-D.')
                    if len(indices)!=1 or prediction.masks is None:raise ValueError(f'Có {len(indices)} mục tiêu pallet; cần đúng một mask để đo RGB-D.')
                    i=indices[0];self.mask=prediction.masks.data[i].cpu().numpy()
                    self.result.update(label=prediction.names[int(classes[i])],confidence=float(confidence[i]),bbox=boxes[i].tolist(),
                                       state='STABILIZE_RGBD',yolo_enabled=False,message='YOLO OFF; lọc RGB-D khi xe đứng yên')
                    save_frame(self.args.output,'camera',annotate(frame,self.result['detections']))
                    self.last=t;self.publish();return
                if not self.info or not self.odom or not self.depth:raise ValueError('Chờ RGB, Depth, CameraInfo và odom')
                at=self.history.at(t)
                if at is None:raise ValueError('Chờ odom tại thời điểm chụp')
                if dmsg.encoding!='32FC1':raise ValueError('Depth cần32FC1')
                dtype=np.dtype('>f4' if dmsg.is_bigendian else '<f4')
                depth=np.frombuffer(dmsg.data,dtype).reshape(dmsg.height,dmsg.step//4)[:,:dmsg.width]
                mask=cv2.resize(self.mask,(dmsg.width,dmsg.height),interpolation=cv2.INTER_NEAREST)
                transform=self.tf.lookup_transform('base_footprint',self.info.header.frame_id,Time.from_msg(rgb.header.stamp),timeout=Duration(seconds=.02)).transform
                tr=transform.translation;q=transform.rotation
                target,measurement=measured_front(mask,depth,self.info.k,[tr.x,tr.y,tr.z],[q.x,q.y,q.z,q.w],(at.x,at.y,at.yaw),self.g,'optical' in self.info.header.frame_id)
                self.result.update(measurement=measurement,capture_stamp_s=t,message='Đang ổn định pose RGB-D')
                locked=self.stable.add(t,*target)
                self.result['stable_samples']=len(self.stable.rows)
                if locked:
                    snapshot=dict(frame_id='odom',feature='front_entry_corridor_center',pose=locked,source='SIMULATED_RGBD_MEASUREMENT',
                                  capture_stamp_s=t,applied_bias=[],calibration_hash=digest(dict(camera_frame=self.info.header.frame_id,k=list(self.info.k),translation=[tr.x,tr.y,tr.z],quaternion=[q.x,q.y,q.z,q.w])),case_id=self.case['case_id'])
                    self.result.update(state='LOCKED',locked_target=snapshot,yolo_enabled=False,pallet_measurement_enabled=False,message='Đã khóa pose RGB-D; dừng đo pallet')
            if self.result.get('bbox') and measuring:
                x1,y1,x2,y2=map(int,self.result['bbox']);cv2.rectangle(frame,(x1,y1),(x2,y2),(50,230,100),2)
                cv2.putText(frame,f"{self.result['label']} {self.result['confidence']:.2f}",(x1,max(20,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.55,(50,230,100),2)
            img=self.args.output/'camera.tmp.jpg';cv2.imwrite(str(img),frame);img.replace(self.args.output/'camera.jpg')
            self.last=t
        except Exception as e:
            self.result['message']=str(e)
            if self.rgb is not None:
                try:
                    frame=decode(self.rgb)
                    if self.rgb.encoding.lower()=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
                    img=self.args.output/'camera.tmp.jpg';cv2.imwrite(str(img),frame);img.replace(self.args.output/'camera.jpg')
                except Exception:pass
        self.publish()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('GZ_PARTITION')!='tai_r3_sim_only_89':p.error('simulation isolation required')
    rclpy.init();node=None
    try:node=Camera(a);rclpy.spin(node)
    except KeyboardInterrupt:pass
    except Exception as e:atomic(a.output/'vision.json',dict(state='ERROR',message=str(e)))
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__':main()
