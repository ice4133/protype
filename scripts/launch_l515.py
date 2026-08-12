#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

# Get the absolute path of the current Python file
current_file_path = os.path.abspath(__file__)

# Get the directory of the current Python file
current_dir = os.path.dirname(current_file_path)

# Resolve the path to the parent directory's "config" folder
config_dir = os.path.realpath(os.path.join(current_dir, "..", "config"))

def generate_launch_description():
    # Define parameters programmatically
    parameters = {
        'camera_name': 'cam_l515',
        'serial_no': '_f0460629',
        'depth_module.profile': '640x480x30',
        'rgb_camera.profile': '640x480x30',
        'enable_depth': 'true',
        'enable_color': 'true',
        'pointcloud.enable': 'true',
        'depth_module.enable_auto_exposure': 'true',
        'enable_sync': 'false',
        'initial_reset': 'false',
        'wait_for_device_timeout': '10.',
        'clip_distance': '3.0',
        'json_file_path': f'{config_dir}/l515.json',
    }

    # Include rs_launch.py from the realsense2_camera package
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments=parameters.items()
    )

    return LaunchDescription([
        realsense_launch,
    ])

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
