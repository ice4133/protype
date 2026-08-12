#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def get_config_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.realpath(os.path.join(current_dir, "..", "config"))


def create_camera_launch(camera_config):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments=camera_config.items()
    )


def generate_launch_description():
    config_dir = get_config_path()

    # Configuration for 3x D405 cameras
    d405_configs = [
        {
            'camera_name': 'cam_high',
            'serial_no': '_218622273905',
            'depth_module.profile': '640x480x60',
            'enable_depth': 'true',
            'enable_color': 'true',
            'depth_module.enable_auto_exposure': 'true',
            'enable_sync': 'false',
            'initial_reset': 'false',
            'wait_for_device_timeout': '10.',
            # Low-latency settings
            'depth_module.frames_queue_size': '1',
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
            'depth_module.global_time_enabled': 'false',
            'align_depth.frames_queue_size': '1',
            'colorizer.frames_queue_size': '1',
            'depth_module.auto_exposure_limit': '16000',
            'decimation_filter.frames_queue_size': '1',
            'spatial_filter.frames_queue_size': '1',
            'temporal_filter.frames_queue_size': '1',
            'hole_filling_filter.frames_queue_size': '1',
            'hdr_merge.frames_queue_size': '1',
        }
    ]

    # Create launch descriptions for all cameras
    launches = [create_camera_launch(config) for config in d405_configs]

    return LaunchDescription(launches)


if __name__ == '__main__':
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
