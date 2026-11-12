"""Raw diagnostic streams stay independent of the fused control pose."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from r3_real_trial import OdomImuDiagnostics


class OdomImuDiagnosticsTest(unittest.TestCase):
    def test_raw_and_fused_measurements_keep_their_own_frames_and_timestamps(self):
        recorder = OdomImuDiagnostics()
        wheel = Odometry()
        wheel.header.frame_id = 'odom'
        wheel.header.stamp.sec = 100
        wheel.header.stamp.nanosec = 500_000_000
        wheel.child_frame_id = 'base_footprint'
        wheel.pose.pose.position.x = 12.
        wheel.pose.pose.orientation.w = 1.
        wheel.twist.twist.angular.z = .3
        wheel.twist.covariance[35] = .2
        fused = Odometry()
        fused.header.frame_id = 'odom'
        fused.header.stamp.sec = 101
        fused.pose.pose.position.x = .4
        fused.pose.pose.orientation.w = 1.
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.header.stamp.sec = 102
        imu.orientation.z = math.sin(.1)
        imu.orientation.w = math.cos(.1)
        imu.angular_velocity.z = .04
        imu.linear_acceleration.z = 9.81
        imu.angular_velocity_covariance[8] = .000025
        recorder.on_odom('wheel_odom', wheel, 10.)
        recorder.on_odom('fused_odom', fused, 10.02)
        recorder.on_imu(imu, 10.03)
        recorder.sample(10.05, 200., 'APPROACH', (.4, .02, .2), (.1, -.03))
        # A later callback or message reuse must not rewrite past snapshots.
        wheel.pose.pose.position.x = 99.
        recorder.on_odom('wheel_odom', wheel, 10.1)
        row = recorder.rows[0]
        self.assertEqual(row['wheel_odom']['position_m'][0], 12.)
        self.assertEqual(row['wheel_odom']['stamp_s'], 100.5)
        self.assertEqual(row['fused_odom']['position_m'][0], .4)
        self.assertEqual(row['imu']['frame_id'], 'imu_link')
        self.assertEqual(row['imu']['angular_velocity_rps'][2], .04)
        self.assertEqual(row['imu']['linear_acceleration_mps2'][2], 9.81)
        self.assertEqual(row['imu']['angular_velocity_covariance'][8], .000025)
        self.assertEqual(row['accepted_control_pose_xyyaw'], [.4, .02, .2])
        self.assertEqual(row['issued_cmd_v_mps'], .1)
        self.assertEqual(row['issued_cmd_w_rps'], -.03)

    def test_sampling_and_memory_are_bounded_without_fabricating_missing_sources(self):
        recorder = OdomImuDiagnostics(max_samples=2)
        for stamp in (0., .01, .05, .06, .10):
            recorder.sample(stamp, 100.+stamp, 'WAIT_SENSORS', None, (0., 0.))
        self.assertEqual([r['monotonic_s'] for r in recorder.rows], [.05, .10])
        self.assertEqual(recorder.dropped_samples, 1)
        self.assertIsNone(recorder.rows[0]['wheel_odom'])
        self.assertIsNone(recorder.rows[0]['imu'])

    def test_invalid_raw_values_survive_as_null_in_valid_json(self):
        recorder = OdomImuDiagnostics()
        imu = Imu()
        imu.orientation.x = float('nan')
        imu.angular_velocity.z = float('inf')
        imu.orientation_covariance[0] = -1.
        recorder.on_imu(imu, 1.)
        recorder.sample(1.05, 10., 'WAIT_SENSORS', None, (0., 0.))
        with tempfile.TemporaryDirectory() as directory:
            report = recorder.write(Path(directory))
            content = Path(report['path']).read_text()
            self.assertNotIn('NaN', content)
            self.assertNotIn('Infinity', content)
            row = json.loads(content)
            self.assertIsNone(row['imu']['quaternion_xyzw'][0])
            self.assertIsNone(row['imu']['angular_velocity_rps'][2])
            self.assertEqual(row['imu']['orientation_covariance'][0], -1.)
            self.assertFalse(report['physical_alignment_independently_measured'])
            self.assertFalse(row['imu']['calibration_status_available'])


if __name__ == '__main__':
    unittest.main()
