"""Actual acquisition/alignment services with synthetic inputs in ROS domain 231."""
import json,math,os
from pathlib import Path
import sys,time,unittest
from types import SimpleNamespace
os.environ['ROS_DOMAIN_ID']='231'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT)]
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from detected_pallets_interfaces.msg import PalletTarget,PalletDetection
from publish_pallet_detection import DetectionNode
from publish_pallet_target import TargetNode
from pallet_perception_config import parse_args
from align_pallet import PalletAlignmentNode


class AcquisitionRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node=Node('acquisition_synthetic_inputs')
        self.detector=DetectionNode(SimpleNamespace(start_paused=False,max_age_ms=250))
        self.tracker=TargetNode(parse_args('tracker',[]))
        self.alignment=PalletAlignmentNode()
        self.executor=SingleThreadedExecutor()
        for node in (self.node,self.detector,self.tracker,self.alignment):self.executor.add_node(node)
        self.target=self.node.create_publisher(PalletTarget,'/pallet/target',1)
        self.odom=self.node.create_publisher(Odometry,'/odom',1)
        self.state=None;self.commands=[];self.emit_semantic=True;self.last_detection=0.
        self.node.create_subscription(String,'/pallet/recognition/status',self.on_state,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Twist,'/pallet/alignment/proposed_cmd_vel',
            lambda m:self.commands.append((m.linear.x,m.angular.z)),1)
        self.client=self.node.create_client(SetBool,'/pallet/alignment/set_enabled')
        self.recognition_client=self.node.create_client(SetBool,'/pallet/set_enabled')
        self.timer=self.node.create_timer(.05,self.inputs)

    def on_state(self,msg):self.state=json.loads(msg.data)

    def inputs(self):
        stamp=self.node.get_clock().now().to_msg()
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.pose.pose.orientation.w=1.;self.odom.publish(odom)
        target=PalletTarget();target.header.stamp=stamp;target.header.frame_id='camera_color_optical_frame'
        target.valid=True;target.status='TRACKED_RGBD; class=pallets; target_bias_applied'
        target.camera_forward_m=1.8;target.chassis_front_forward_m=1.8665;target.opening_lateral_m=.01
        target.entry_heading_rad=math.radians(1.);target.face_yaw_rad=-target.entry_heading_rad
        target.opening_width_m=.252;target.leg_center_separation_m=.32
        target.left_depth_samples=target.right_depth_samples=100;self.target.publish(target)
        wall=time.monotonic()
        enabled,generation=self.detector.inference_state()
        if self.emit_semantic and enabled and wall-self.last_detection>=.5:
            self.last_detection=wall
            detection=PalletDetection();detection.header=target.header;detection.valid=True
            detection.confidence=.9;detection.class_name='pallets';detection.bbox_xyxy=[270.,170.,390.,300.]
            self.detector.publish_result(detection,generation)

    def spin_until(self,predicate,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():return
        self.fail('Timed out: '+str(self.state))

    def request(self,client,enabled):
        self.spin_until(client.service_is_ready)
        req=SetBool.Request();req.data=enabled
        return client.call_async(req)

    def result(self,future):
        self.spin_until(future.done)
        return future.result()

    def acquired(self):
        self.spin_until(lambda:self.state is not None and self.state['phase']=='TRACKING'
            and not self.state['enabled'] and not self.state['busy'])
        self.spin_until(lambda:self.alignment.recognition_ready()[0])

    def test_automatic_handoff_waits_for_manual_start_and_resets_on_retry(self):
        self.acquired()
        self.assertGreaterEqual(self.state['elapsed_s'],3.)
        self.assertLessEqual(self.state['elapsed_s'],5.)
        self.assertTrue(self.state['locked']);self.assertFalse(self.alignment.core.active)
        self.assertTrue(self.commands);self.assertTrue(all(v==w==0 for v,w in self.commands))
        generation=self.detector.inference_generation-1
        self.assertFalse(self.detector.publish_result(PalletDetection(),generation))
        session=self.state['session'];start=self.tracker.recognition_start_ns
        self.assertTrue(self.result(self.request(self.recognition_client,True)).success)
        self.spin_until(lambda:self.state['session']>session and self.state['phase']=='ACQUIRING')
        self.spin_until(lambda:self.tracker.recognition_start_ns>start)
        self.assertFalse(self.state['locked']);self.assertFalse(self.alignment.core.active)
        self.assertTrue(self.tracker.recognition_reset_pending)

    def test_timeout_stops_yolo_without_authorizing_alignment(self):
        self.emit_semantic=False
        with self.detector.lock:self.detector.acquisition.started-=5.1
        self.spin_until(lambda:self.state is not None and self.state['phase']=='ACQUIRE_FAILED')
        self.assertFalse(self.state['enabled']);self.assertFalse(self.state['locked'])
        self.assertFalse(self.result(self.request(self.client,True)).success)
        self.assertFalse(self.alignment.core.active)
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def test_start_waits_for_inflight_yolo_and_rejects_reenable_during_alignment(self):
        self.acquired()
        with self.detector.lock:self.detector.inference_busy=True
        future=self.request(self.client,True)
        self.spin_until(lambda:self.alignment.preparing and bool(self.detector.prepare_waiters))
        self.assertFalse(future.done());self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        self.detector.end_inference()
        self.assertTrue(self.result(future).success)
        self.spin_until(lambda:self.alignment.core.v>0)
        self.assertGreater(self.alignment.core.target_stamp,self.alignment.measure_after_s)
        self.assertFalse(self.result(self.request(self.recognition_client,True)).success)
        self.assertTrue(self.result(self.request(self.client,False)).success)
        self.assertFalse(self.alignment.core.active)

    def test_stop_cancels_pending_start_and_late_ack_cannot_restart(self):
        self.acquired()
        with self.detector.lock:self.detector.inference_busy=True
        start=self.request(self.client,True)
        self.spin_until(lambda:self.alignment.preparing and bool(self.detector.prepare_waiters))
        self.assertTrue(self.result(self.request(self.client,False)).success)
        self.detector.end_inference()
        self.assertFalse(self.result(start).success)
        self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def tearDown(self):
        self.alignment.core.stop();self.alignment.zero()
        for node in (self.alignment,self.tracker,self.detector,self.node):
            self.executor.remove_node(node);node.destroy_node()
        self.executor.shutdown();rclpy.shutdown()

if __name__=='__main__':unittest.main()
