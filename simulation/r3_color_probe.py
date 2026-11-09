#!/usr/bin/env python3
"""Stationary A/B RGB-D render probe in its own Gazebo partition; no robot/controllers."""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from r3.appearance import set_pallet_materials,PALLET_COLOR_HEX


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    os.environ['GZ_PARTITION']='tai_r3_pallet_color_probe_94'
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    source=Path(__file__).parent.parent/'ros2_ws/worlds/docking_one_pallet.sdf'
    for name in ('blue','green_25a18e'):
        out=a.output/name;out.mkdir(exist_ok=True);tree=ET.parse(source);world=tree.getroot().find('world')
        gui=world.find('gui')
        if gui is not None:world.remove(gui)
        for e in world.findall('.//model[@name="single_floor_pallet"]/link/*'):
            if e.tag in ('visual','collision') and e.get('name','').startswith('top_'):
                e.find('pose').text='0 0 0.115 0 0 0';e.find('geometry/box/size').text='0.355 0.355 0.030'
        if name=='green_25a18e':set_pallet_materials(world)
        # Same synthetic default fork-root pose as the app: B_world=(-.043,.12,0).
        # Actual camera TF from the repo/runtime: (.367,0,.8), pitch20deg.
        camera=ET.fromstring('''<model name="color_probe_camera"><static>true</static><pose>0.324 0.12 0.8 0 0.3490658503988659 0</pose><link name="camera_link"><sensor name="color_probe" type="rgbd_camera"><always_on>true</always_on><update_rate>5</update_rate><topic>/r3_color/camera</topic><gz_frame_id>camera_link</gz_frame_id><camera><optical_frame_id>camera_link</optical_frame_id><horizontal_fov>1.0471975511965976</horizontal_fov><image><width>320</width><height>240</height></image><clip><near>0.005</near><far>8</far></clip><lens><intrinsics><fx>277.1281292110204</fx><fy>260.30012224251016</fy><cx>160</cx><cy>120</cy><s>0</s></intrinsics></lens></camera></sensor></link></model>''')
        world.append(camera);tree.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
        state={'frames':0};node=Node()
        def on_rgb(m):
            if m.width!=320 or m.height!=240:return
            state['frames']+=1
            arr=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
            state['rgb']=cv2.cvtColor(arr,cv2.COLOR_RGB2BGR);state['stamp']=str(m.header.stamp)
        def on_depth(m):
            if m.width!=320 or m.height!=240:return
            state['depth']=np.frombuffer(m.data,np.float32).reshape(m.height,m.step//4)[:,:m.width].copy()
        node.subscribe(Image,'/r3_color/camera/image',on_rgb);node.subscribe(Image,'/r3_color/camera/depth_image',on_depth)
        with (out/'gazebo.log').open('w') as log:
            child=subprocess.Popen(['gz','sim','-r','-s',str((out/'world.sdf').resolve())],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            until=time.monotonic()+40
            while time.monotonic()<until:
                if child.poll() is not None:raise RuntimeError('Probe Gazebo exited; see '+str(out/'gazebo.log'))
                if state['frames']>=5 and 'depth' in state:break
                time.sleep(.1)
            else:raise RuntimeError('No RGB-D render received; see '+str(out/'gazebo.log'))
            cv2.imwrite(str(out/'rgb.png'),state['rgb']);np.save(out/'depth_m.npy',state['depth'])
            (out/'capture.json').write_text(json.dumps(dict(source='SEPARATE_GAZEBO_RENDER_PROBE_NOT_ROBOT_MOTION',partition=os.environ['GZ_PARTITION'],fork_root_distance_m=1.5,lateral_m=0,yaw_deg=0,camera_world_pose=[.324,.12,.8,0,math.radians(20),0],width=320,height=240,material_hex=PALLET_COLOR_HEX if name!='blue' else 'legacy_blue',frames=state['frames'],stamp=state['stamp']),indent=2))
            print('Captured',name,flush=True)
        finally:
            node.unsubscribe('/r3_color/camera/image');node.unsubscribe('/r3_color/camera/depth_image')
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
if __name__=='__main__':main()
