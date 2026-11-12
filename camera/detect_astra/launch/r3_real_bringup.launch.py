"""Real drive/odom and LiDAR obstacle scan for an r3 trial; no Nav2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(path, arguments=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=(arguments or {}).items())


def generate_launch_description():
    robot = Path(get_package_share_directory('tai_robot_one')) / 'launch'
    lidar = Path(get_package_share_directory('sllidar_ros2')) / 'launch'
    return LaunchDescription([
        include(robot/'real_hardware.launch.py', {
            'use_lift': 'false', 'use_rviz': 'false',
            'use_cm029_teleop': 'false', 'cmd_vel_input_topic': '/cmd_vel_safe',
        }),
        include(lidar/'sllidar_a1_launch.py', {
            'serial_port': '/dev/tai_lidar', 'frame_id': 'lidar_link',
        }),
        include(robot/'scan_self_filter.launch.py'),
    ])
