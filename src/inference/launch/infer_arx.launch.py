# merged_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='arx',
        description='Choose the arm type: arx or yam'
    )

    arm_type_param = LaunchConfiguration('arm_type')

    joint_base_node = Node(
        package='inference',
        executable='joint_base_node',
        name='joint_base_node',
        output='screen',
        parameters=[{'arm_type': arm_type_param}],
    )

    camera_node = Node(
        package='inference',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    return LaunchDescription([
        arm_type_arg,
        joint_base_node,
        camera_node,
    ])