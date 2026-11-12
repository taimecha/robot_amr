"""Publish a low-obstacle scan from the Astra node already running on this Pi."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration = Path(__file__).resolve().parents[3] / 'ros2_ws/config/calibration/astra_pro_depth_x_skew_preview.yaml'
    return LaunchDescription([
        Node(
            package='tai_robot_one', executable='publish_rgb_calibrated_cloud.py',
            name='spatial_depth_cloud', output='screen',
            parameters=[str(calibration), {
                'pixel_stride': 4, 'max_rate_hz': 8.0,
                'output_topic': '/camera/depth/points_nav_corrected',
            }],
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='camera_obstacle_scan', output='screen',
            remappings=[('cloud_in', '/camera/depth/points_nav_corrected'),
                        ('scan', '/camera/obstacle_scan')],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'base_footprint', 'transform_tolerance': .10,
                'min_height': .06, 'max_height': .75,
                'angle_min': -.5235987756, 'angle_max': .5235987756,
                'angle_increment': .00872664626, 'queue_size': 1,
                'scan_time': 1.0 / 8.0, 'range_min': .60, 'range_max': 6.0,
                'use_inf': True,
            }],
        ),
    ])
