"""
Launch slave arms in normal (joint command) mode for HDF5 playback.
Subscribes to /YAM_VR_L and /YAM_VR_R published by the intervention relay.

Usage:
  pixi run ros2 launch yam_damiao_controller playback_slave.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'),
        'config',
        'playback_slave.yaml'
    )

    slave_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_l',
        output='screen',
        parameters=[params_file],
    )

    slave_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_r',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        slave_l_node,
        slave_r_node,
    ])
