#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def get_config_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.realpath(os.path.join(current_dir, "..", "config"))

def generate_launch_description():
    config_dir = get_config_path()
    ld = LaunchDescription()

    # Configuration for 2x D435 cameras
    d435_configs = [
        {  # Wrist Left
            'camera_name': 'cam_left_wrist',
            'serial_no': '_243222075127',
            'depth_module.profile': '640x480x60',
            'rgb_camera.profile': '640x480x60',
            'rgb_camera.enable_auto_exposure': 1,
            'rgb_camera.enable_auto_white_balance': 1,
            'enable_depth': 0,
            'enable_color': 1,
            'depth_module.enable_auto_exposure': 1,
            'depth_module.emitter_enable': 0,
            'depth_module.laserpower': 0.,
            'enable_sync': 'false',
            'initial_reset': False,
            'wait_for_device_timeout': 10.,
        },
        {  # Wrist Right
            'camera_name': 'cam_left_wrist',
            'serial_no': '_243222075127',
            'depth_module.profile': '640x480x60',
            'rgb_camera.profile': '640x480x60',
            'rgb_camera.enable_auto_exposure': 1,
            'rgb_camera.enable_auto_white_balance': 1,
            'enable_depth': 0,
            'enable_color': 1,
            'depth_module.enable_auto_exposure': 1,
            'depth_module.emitter_enable': 0,
            'depth_module.laserpower': 0.,
            'enable_sync': 'false',
            'initial_reset': False,
            'wait_for_device_timeout': 10.,
        },
    ]

    # Create nodes for all cameras
    for config in d435_configs:
        camera_name = config['camera_name']
        parameters = {k: v for k, v in config.items() if k != 'camera_name'}
        
        node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=camera_name,
            namespace=camera_name,
            parameters=[parameters],
            remappings=[
                ('color/image_raw', 'color/image_rect_raw')
            ],
            output='screen'
        )
        ld.add_action(node)

    return ld

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
