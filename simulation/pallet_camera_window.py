#!/usr/bin/env python3
"""Separate live Gazebo pallet camera window for the one-pallet demo."""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class Viewer(Node):
    def __init__(self):
        super().__init__('pallet_camera_window')
        self.create_subscription(Image,'/pallet_demo/annotated',self.on_image,qos_profile_sensor_data)
        cv2.namedWindow('Pallet recognition - Gazebo RGB',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Pallet recognition - Gazebo RGB',640,480)
        self.create_timer(.05,lambda:cv2.waitKey(1))
    def on_image(self,msg):
        a=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)
        frame=a[:,:msg.width*3].reshape(msg.height,msg.width,3)
        cv2.imshow('Pallet recognition - Gazebo RGB',frame)

def main():
    rclpy.init();n=Viewer()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:n.destroy_node();cv2.destroyAllWindows();rclpy.try_shutdown()
if __name__=='__main__':main()
