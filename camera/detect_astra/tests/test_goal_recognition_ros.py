"""Observe Nav2 statuses and call recognition services; no motion nodes."""
import os
from pathlib import Path
import sys
import time
import unittest
os.environ['ROS_DOMAIN_ID'] = '231'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT)]
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from detected_pallets_interfaces.msg import PalletDetection
from publish_pallet_detection import DetectionNode
from pallet_perception_config import parse_args


class GoalRecognitionRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = DetectionNode(parse_args('detector', ['--recognize-after-goal']))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def status(self, state, start=None, identifier=1, deliver=True):
        msg = GoalStatusArray(); goal = GoalStatus()
        goal.goal_info.goal_id.uuid = [identifier]*16
        ns = int((start or self.start)*1e9)
        goal.goal_info.stamp.sec, goal.goal_info.stamp.nanosec = divmod(ns, 10**9)
        goal.status = state; msg.status_list = [goal]
        if deliver:
            self.node.on_nav_status(msg)
        return msg

    def test_actual_ros_status_subscription_triggers_without_motor_publishers(self):
        self.start = self.node.get_clock().now().nanoseconds/1e9
        publisher = self.node.create_publisher(GoalStatusArray, self.node.args.nav_status_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for status, predicate in [(2, lambda: self.node.goal_gate.navigation_active),
                                  (4, lambda: self.node.inference_enabled)]:
            message = self.status(status, deliver=False)
            deadline = time.monotonic()+3
            while time.monotonic() < deadline and not predicate():
                publisher.publish(message)
                rclpy.spin_once(self.node, timeout_sec=.02)
            self.assertTrue(predicate())
        topics = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace())
        self.assertFalse(any('cmd_vel' in topic for topic, _ in topics))

    def test_success_starts_acquisition_once_and_new_navigation_stops_inflight_result(self):
        self.assertFalse(self.node.inference_enabled)
        self.start = self.node.get_clock().now().nanoseconds/1e9
        self.status(4)
        self.assertFalse(self.node.inference_enabled)
        self.status(2)
        self.assertFalse(self.node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success)
        self.status(4)
        self.assertTrue(self.node.inference_enabled)
        self.assertEqual(self.node.acquisition.session, 1)
        self.status(4)
        self.assertEqual(self.node.acquisition.session, 1)
        generation = self.node.inference_generation
        self.assertTrue(self.node.begin_inference(generation))
        self.status(2, self.start+.01, identifier=2)
        self.assertFalse(self.node.inference_enabled)
        self.assertFalse(self.node.publish_result(PalletDetection(), generation))
        self.node.end_inference()
        self.status(4, self.start+.01, identifier=2)
        self.assertFalse(self.node.inference_enabled)
        self.assertFalse(self.node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success)

    def test_abort_and_old_result_stay_paused_rearm_allows_new_goal(self):
        self.start = self.node.get_clock().now().nanoseconds/1e9
        self.status(2); self.status(6)
        self.assertFalse(self.node.inference_enabled)
        self.assertTrue(self.node.arm_goal(Trigger.Request(), Trigger.Response()).success)
        self.status(4)
        self.assertFalse(self.node.inference_enabled)
        self.start = self.node.get_clock().now().nanoseconds/1e9
        self.status(1, identifier=2); self.status(4, identifier=2)
        self.assertTrue(self.node.inference_enabled)


if __name__ == '__main__':
    unittest.main()
