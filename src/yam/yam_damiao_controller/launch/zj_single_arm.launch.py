import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'),
        'config',
        'zj_single_arm.yaml'
    )

    # Controller node
    arm_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='yam_l',
        output='screen',
        parameters=[params_file],
    )

    # Launch argument to control whether to start description
    use_description_arg = DeclareLaunchArgument(
        'use_description',
        default_value='false',
        description='Launch yam_description/display.launch if true'
    )

    # Path to yam_description/display.launch.py
    yam_description_launch = os.path.join(
        get_package_share_directory('yam_description'),
        'launch',
        'display.launch.py'
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yam_description_launch),
        condition=IfCondition(LaunchConfiguration('use_description'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file
        ),
        use_description_arg,
        arm_node,
        description_launch,
    ])
