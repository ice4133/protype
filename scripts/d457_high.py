#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os

def get_config_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.realpath(os.path.join(current_dir, "..", "config"))

def generate_launch_description():
    ld = LaunchDescription()
    config_dir = get_config_path()
    
    camera_config = {
        'camera_name': 'cam_d457',
        'serial_no': '_336622303642',
    }
    dev_params = config_dir + '/d457.yaml'
    print(dev_params)
    node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=camera_config['camera_name'],
        parameters=[dev_params],
        respawn=True,
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(node)

    return ld

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
