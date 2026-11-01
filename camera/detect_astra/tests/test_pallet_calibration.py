"""Validate calibration evidence without reading a camera or commanding motion."""
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

os.environ['ROS_DOMAIN_ID'] = '231'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import rclpy
from detected_pallets_interfaces.msg import PalletTarget
from record_pallet_calibration import CalibrationRecorder, summary


class CalibrationEvidenceTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = CalibrationRecorder(SimpleNamespace(samples=10, fork_root_offset_m=0.065))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def message(self):
        msg = PalletTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.valid = True
        msg.camera_forward_m = 1.3985
        msg.chassis_front_forward_m = 1.465
        msg.opening_lateral_m = -0.02
        msg.face_yaw_rad = math.radians(5)
        msg.left_depth_samples = msg.right_depth_samples = 100
        return msg

    def test_reference_conversion_and_duplicate_exclusion(self):
        msg = self.message()
        self.node.receive(msg)
        self.node.receive(msg)
        self.assertEqual(len(self.node.samples), 1)
        sample = self.node.samples[0]
        self.assertAlmostEqual(sample['fork_root_m'], 1.4, places=6)
        self.assertAlmostEqual(sample['lateral_m'], -0.02, places=6)
        self.assertAlmostEqual(sample['face_yaw_deg'], 5, places=5)
        self.assertEqual(self.node.rejected['duplicate or out-of-order image'], 1)

    def test_invalid_stale_wrong_frame_and_nonfinite_samples_are_excluded(self):
        edits = (
            lambda m: setattr(m, 'valid', False),
            lambda m: setattr(m.header.stamp, 'sec', m.header.stamp.sec - 2),
            lambda m: setattr(m.header, 'frame_id', 'base_link'),
            lambda m: setattr(m, 'opening_lateral_m', math.nan),
            lambda m: setattr(m, 'left_depth_samples', 0),
        )
        for edit in edits:
            msg = self.message()
            edit(msg)
            self.node.receive(msg)
        self.assertFalse(self.node.samples)
        self.assertEqual(sum(self.node.rejected.values()), 5)

    def test_report_separates_bias_from_variability(self):
        report = summary([{'x': v} for v in (1.0, 1.01, 1.03)], {'x': 1.0})['x']
        self.assertAlmostEqual(report['median_error'], 0.01)
        self.assertAlmostEqual(report['p95_absolute_error'], 0.03)
        self.assertGreater(report['sample_stddev'], 0.015)


if __name__ == '__main__':
    unittest.main()
