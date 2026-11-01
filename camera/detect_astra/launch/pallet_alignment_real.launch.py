"""Real staging to 1.20 m; starting this launch does not press the Start button."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    robot_share = Path(get_package_share_directory('tai_robot_one'))
    lidar_share = Path(get_package_share_directory('sllidar_ros2'))
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(robot_share/'launch/real_hardware.launch.py')),
        launch_arguments={
            'cmd_vel_input_topic': '/cmd_vel_safe',
            'use_cm029_teleop': 'false', 'use_lift': 'false',
            'home_lift_on_activate': 'false', 'use_rviz': 'false',
        }.items())
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lidar_share/'launch/sllidar_a1_launch.py')),
        launch_arguments={'serial_port': '/dev/tai_lidar', 'frame_id': 'lidar_link'}.items())
    scan_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(robot_share/'launch/scan_self_filter.launch.py')))
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(project/'launch/pallet_alignment_safety.launch.py')))
    alignment = ExecuteProcess(
        cmd=[os.environ.get('PALLET_PYTHON', str(project/'.venv/bin/python')),
             str(project/'scripts/align_pallet.py'), '--execute'],
        name='pallet_alignment_real', output='screen')
    stop_on_alignment_exit = RegisterEventHandler(OnProcessExit(
        target_action=alignment,
        on_exit=[EmitEvent(event=Shutdown(reason='Bộ căn đã dừng; tắt bộ chạy thật'))]))
    return LaunchDescription([hardware, lidar, scan_filter, safety,
                              stop_on_alignment_exit, alignment])
