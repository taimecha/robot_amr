"""Dedicated collision monitor. Does not start motors, Nav2 or a command source."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(__file__).resolve().parents[1] / 'config' / 'pallet_alignment_collision.yaml')
    return LaunchDescription([
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='pallet_alignment_collision_monitor', output='screen', parameters=[config]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='pallet_alignment_safety_manager', output='screen', parameters=[{
                 'autostart': True, 'node_names': ['pallet_alignment_collision_monitor'],
                 'bond_timeout': 2.0, 'attempt_respawn_reconnection': False}]),
    ])
