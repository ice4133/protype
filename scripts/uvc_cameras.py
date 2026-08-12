#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def create_camera_node(camera_config):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=f"{camera_config['camera_name']}_node",
        parameters=[
            {
                'video_device': camera_config['device_path'],
                'camera_id': camera_config['camera_id'],
                'image_size': camera_config['image_size'],
                'time_per_frame': camera_config['time_per_frame']
            }
        ],
        remappings=[
            ('image_raw', camera_config['topic_name'])
        ]
    )

def generate_launch_description():
    # Common parameters for both cameras
    common_params = {
        'image_size': [640, 480],
        'time_per_frame': [1, 50],
    }
    
    # Configuration for both cameras
    v4l2_configs = [
        {  # Left camera
            'camera_name': 'cam_left_wrist',
            'device_path': '/dev/uvc_camera/camera_left',
            'camera_id': 'left_cam',
            'topic_name': '/cam_left_wrist/color/image_rect_raw',
            **common_params
        },
        {  # Right camera
            'camera_name': 'cam_right_wrist',
            'device_path': '/dev/uvc_camera/camera_right',
            'camera_id': 'right_cam',
            'topic_name': '/cam_right_wrist/color/image_rect_raw',
            **common_params
        },
        # {  # High camera
        #     'camera_name': 'cam_fish_top',
        #     'device_path': '/dev/uvc_camera/camera_top',
        #     'camera_id': 'top_cam',
        #     'topic_name': '/cam_fish_top/color/image_rect_raw',
        #     **common_params
        # }
    ]

    # Create nodes for all cameras
    nodes = [create_camera_node(config) for config in v4l2_configs]

    return LaunchDescription(nodes)

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
